// SPDX-License-Identifier: GPL-2.0
/*
 * cam_cap.c - self-contained CAMSV1 single-frame capture module for the
 *             MediaTek MT6895 (Redmi K50 "rubens").
 *
 * The module owns every resource it needs and depends on no kernel-internal
 * structure layout beyond the page allocator, ioremap()/readl()/writel() and
 * procfs:
 *
 *   1. one physically contiguous frame buffer.  The preferred allocation is
 *      dma_alloc_coherent() from a throwaway platform_device: this kernel has
 *      CONFIG_ARCH_FORCE_MAX_ORDER=10, so alloc_pages_exact() can never return
 *      more than 4 MiB, while CONFIG_DMA_CMA=y provides a 32 MiB contiguous
 *      pool that dma-direct reaches through kernel/dma/direct.c:132
 *      (dma_alloc_contiguous).  dma_alloc_coherent() also hands back a mapping
 *      the CPU may read without cache maintenance.  The physical address goes
 *      straight into IMGO_BASE_ADDR (0x700) / IMGO_BASE_ADDR_MSB (0x704).
 *      alloc_pages_exact() + the linear map remains as a fallback.
 *   2. no separate uncached alias: the coherent allocation above is already
 *      CPU-readable.  Only the alloc_pages_exact() fallback needs one, and
 *      neither ioremap_wc() nor memremap(MEMREMAP_WC) is allowed to alias
 *      System RAM on arm64, so that path uses the linear map and warns.
 *   3. the CAMSV1 register block at 0x1a110000 (size 0x1000).
 *   4. the SENINF TOP block at 0x1a010000, used to route CSI port 2
 *      (intf 4 -> mux 1 -> cam_mux 3) into CAMSV1.
 *   5. /proc/camcap (read = frame bytes, write = commands) and
 *      /proc/camcap_info (read = text status).
 *
 * All register offsets and bit names below are quoted from
 * isp71_ref/mtk_cam-sv-regs.h; the write sequences are transcribed from
 * camsv_report.md and camsv_register_sequence_report.md, which were
 * recovered instruction-by-instruction from the vendor mtk-cam-isp.ko
 * disassembly.  Section numbers ("8.8 step N") refer to CAMERA_NOTES.md.
 *
 * WARNING: this module drives the CAMSV1 block unconditionally.  It must
 * only be loaded when no other CAMSV1 user (vendor camera daemon, v4l2
 * stack) is running, because that driver would program the same registers
 * behind our back.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/sprintf.h>
#include <linux/slab.h>
#include <linux/gfp.h>
#include <linux/mm.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <linux/bits.h>
#include <linux/bitfield.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/iommu.h>
#include <linux/i2c.h>
#include <linux/math.h>
#include <linux/math64.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/completion.h>
#include <linux/cpumask.h>
#include <linux/poll.h>
#include <linux/timekeeping.h>
#include <linux/videodev2.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ctrls.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-vmalloc.h>
#include "imx582_modes.h"	/* generated: mode tables, scripts/gen_modes_header.py */

/* ------------------------------------------------------------------ */
/* Defaults / tunables                                                */
/* ------------------------------------------------------------------ */

/* CAMSV1: the only CAMSV instance this module needs (0x1a110000..0x1a110fff) */
#define CAMCAP_CAMSV_BASE_DEF	0x1a110000UL
#define CAMCAP_CAMSV_SIZE	0x1000UL

/* Bounded completion poll after the arm kick (8.8 step 8/9) */
#define CAMCAP_ARM_TIMEOUT_MS	2000

/* Bounce buffer for the frame readback (limits kmalloc size + stack use) */
#define CAMCAP_BOUNCE_SIZE	(128 * 1024)

/*
 * Smallest capture buffer we are willing to fall back to.  alloc_pages_exact()
 * is a buddy allocation, so a large contiguous request can simply be
 * unsatisfiable after boot; see cam_cap_init().
 */
#define CAMCAP_MIN_BUF		(256UL * 1024)
/* INT_EN value written by mtk_cam_sv_top_config (0x4699c): 0x00011070 */
#define CAMCAP_INT_EN_DEF	0x00011070u
/* name of the throwaway platform_device that fronts the coherent allocation */
#define CAMCAP_PDEV_NAME	"cam_cap"

/*
 * Device-tree plumbing for an IOMMU-backed capture buffer.
 *
 * CAMSV1 is not present in this kernel's device tree at all: the running
 * mainline DT has no camsv nodes, and NOT ONE node in the whole tree carries
 * the "iommus" property.  The consequence is measured, not theorised: CAMSV1's
 * IMGO writes anyway come out of larb0/port2 and go through the display IOMMU
 * bank at 0x1e802000, whose REG_MMU_PT_BASE_ADDR (offset 0x000) is still 0
 * because mtk_iommu_mt6895.c only programs it from mtk_iommu_attach_device().
 * With no client device there is never an attach, so every IMGO burst faults:
 *   mtk-iommu-mt6895 1e802000.iommu: fault type=0x5 iova=0xfa500000 pa=0x0
 *                                    larb=0 port=2 layer=0 write
 * The only supported way to get a translation is to hand the kernel a device
 * that does have "iommus", so that of_dma_configure() -> of_iommu_configure()
 * -> mtk_iommu_of_xlate() -> iommu_probe_device() attaches the default domain
 * and mtk_iommu_attach_device() writes the page-table base.  dma_alloc_coherent()
 * on that device then returns an IOVA, which is what IMGO_BASE_ADDR needs.
 *
 * The node is added at run time with of_changeset_* (all of them exported by
 * this kernel) attached under /soc, which the OF reconfiguration notifier
 * auto-populates into a platform_device.  Nothing here writes a struct device
 * field: this module is compiled against a tree whose struct device is 8 bytes
 * smaller than the running kernel's, and doing so previously wedged the
 * allocator through a corrupted dma_mask.
 *
 * iommus = <&disp_iommu CAMCAP_M4U_ID(0, 2)>; MTK_M4U_ID(larb, port) is
 * ((larb) << 5) | (port), and larb0/port2 is exactly what the IOMMU fault
 * decoder reports for CAMSV1.
 */
#define CAMCAP_DT_NODE_NAME	"camcap@1a110000"
#define CAMCAP_DT_COMPATIBLE	"camcap,camsv"
#define CAMCAP_DT_IOMMU_COMPAT	"mediatek,mt6895-disp-iommu"
#define CAMCAP_M4U_ID(larb, port)	(((larb) << 5) | (port))
#define CAMCAP_DT_IOMMU_ID	CAMCAP_M4U_ID(0, 2)

/* ------------------------------------------------------------------ */
/* CAMSV register offsets - REG_CAMSV_* from isp71_ref/mtk_cam-sv-regs.h */
/* ------------------------------------------------------------------ */

#define CAMSV_TOP_FBC_CNT_SET		0x0014	/* REG_CAMSV_TOP_FBC_CNT_SET */
#define CAMSV_MODULE_EN			0x0040	/* REG_CAMSV_MODULE_EN */
#define CAMSV_FMT_SEL			0x0044	/* REG_CAMSV_FMT_SEL */
#define CAMSV_INT_EN			0x0048	/* REG_CAMSV_INT_EN */
#define CAMSV_INT_STATUS		0x004c	/* REG_CAMSV_INT_STATUS */
#define CAMSV_CLK_EN			0x0060	/* REG_CAMSV_CLK_EN */
#define CAMSV_DCIF_SET			0x0074	/* REG_CAMSV_DCIF_SET */
#define CAMSV_SUB_CTRL			0x0078	/* REG_CAMSV_SUB_CTRL */
#define CAMSV_PAK			0x007c	/* REG_CAMSV_PAK */
#define CAMSV_MISC			0x0088	/* REG_CAMSV_MISC */
#define CAMSV_TG_SEN_MODE		0x0100	/* REG_CAMSV_TG_SEN_MODE */
#define CAMSV_TG_VF_CON			0x0104	/* REG_CAMSV_TG_VF_CON */
#define CAMSV_TG_SEN_GRAB_PXL		0x0108	/* REG_CAMSV_TG_SEN_GRAB_PXL */
#define CAMSV_TG_SEN_GRAB_LIN		0x010c	/* REG_CAMSV_TG_SEN_GRAB_LIN */
#define CAMSV_TG_PATH_CFG		0x0110	/* REG_CAMSV_TG_PATH_CFG */
#define CAMSV_TG_SUB_PERIOD		0x0164	/* REG_CAMSV_TG_SUB_PERIOD */
#define CAMSV_PAK_CON			0x01c0	/* REG_CAMSV_PAK_CON */
#define CAMSV_FBC_IMGO_CTL1		0x0240	/* REG_CAMSV_FBC_IMGO_CTL1 */
#define CAMSV_FBC_IMGO_CTL2		0x0244	/* REG_CAMSV_FBC_IMGO_CTL2 */
#define CAMSV_SPECIAL_FUN_EN		0x0600	/* REG_CAMSV_SPECIAL_FUN_EN */
#define CAMSV_IMGO_BASE_ADDR		0x0700	/* REG_CAMSV_IMGO_BASE_ADDR */
#define CAMSV_IMGO_BASE_ADDR_MSB	0x0704	/* REG_CAMSV_IMGO_BASE_ADDR_MSB */
#define CAMSV_IMGO_XSIZE		0x0710	/* REG_CAMSV_IMGO_XSIZE */
#define CAMSV_IMGO_YSIZE		0x0714	/* REG_CAMSV_IMGO_YSIZE */
#define CAMSV_IMGO_STRIDE		0x0718	/* REG_CAMSV_IMGO_STRIDE */
#define CAMSV_IMGO_CON0			0x0720	/* REG_CAMSV_IMGO_CON0 */
#define CAMSV_IMGO_CON1			0x0724	/* REG_CAMSV_IMGO_CON1 */
#define CAMSV_IMGO_CON2			0x0728	/* REG_CAMSV_IMGO_CON2 */
#define CAMSV_IMGO_CON3			0x072c	/* REG_CAMSV_IMGO_CON3 */
#define CAMSV_IMGO_CON4			0x0730	/* REG_CAMSV_IMGO_CON4 */
#define CAMSV_IMGO_CROP			0x074c	/* REG_CAMSV_IMGO_CROP */
#define CAMSV_FRAME_SEQ_NO		0x075c	/* REG_CAMSV_FRAME_SEQ_NO */

/* --- TOP_FBC_CNT_SET --- */
#define CAMSV_TOP_FBC_CNT_SET_RCNT_INC1	BIT(0)	/* RCNT_INC1 */

/* --- MODULE_EN --- */
#define CAMSV_MODULE_EN_TG_EN		BIT(0)	/* TG_EN b0 */
#define CAMSV_MODULE_EN_PAK_EN		BIT(2)	/* PAK_EN b2 */
#define CAMSV_MODULE_EN_PAK_SEL		BIT(3)	/* PAK_SEL b3 */
#define CAMSV_MODULE_EN_IMGO_EN		BIT(4)	/* IMGO_EN b4 */
#define CAMSV_MODULE_EN_UFE_EN		BIT(6)	/* UFE_EN b6 */
#define CAMSV_MODULE_EN_QBN_EN		BIT(7)	/* QBN_EN b7 */
#define CAMSV_MODULE_EN_DS_PERIOD	GENMASK(23, 16)	/* DOWN_SAMPLE_PERIOD b23:16 */
#define CAMSV_MODULE_EN_DS_EN		BIT(24)	/* DOWN_SAMPLE_EN b24 */
#define CAMSV_MODULE_EN_DB_LOAD_HOLD	BIT(25)	/* DB_LOAD_HOLD b25 */
#define CAMSV_MODULE_EN_DB_LOAD_SRC	GENMASK(29, 28)	/* DB_LOAD_SRC b29:28 */
#define CAMSV_MODULE_EN_DB_EN		BIT(30)	/* DB_EN b30 */

/* --- FMT_SEL --- */
#define CAMSV_FMT_SEL_TG1_FMT_MASK	GENMASK(2, 0)	/* TG1_FMT b2:0 */

/* --- INT_STATUS (isp71_ref: CAMSV_INT_*_ST) --- */
#define CAMSV_INT_VS_ST			BIT(0)	/* VS_ST */
#define CAMSV_INT_TG_ST1		BIT(1)	/* TG_ST1 */
#define CAMSV_INT_TG_ST2		BIT(2)	/* TG_ST2 */
#define CAMSV_INT_TG_ERR_ST		BIT(4)	/* SV_TG_ERR_ST */
#define CAMSV_INT_TG_GBERR_ST		BIT(5)	/* SV_TG_GBERR_ST */
#define CAMSV_INT_TG_SOF_INT_ST		BIT(6)	/* TG_SOF_INT_ST */
#define CAMSV_INT_DB_LOAD_ERR_ST	BIT(10)	/* DB_LOAD_ERR_ST */
#define CAMSV_INT_PASS1_DON_ST		BIT(11)	/* PASS1_DON_ST */
#define CAMSV_INT_SW_PASS1_DON_ST	BIT(12)	/* SV_SW_PASS1_DON_ST */
#define CAMSV_INT_DMA_ERR_ST		BIT(16)	/* SV_DMA_ERR_ST */
#define CAMSV_INT_IMGO_OVERR_ST		BIT(17)	/* SV_IMGO_OVERR_ST */
#define CAMSV_INT_IMGO_DROP_ST		BIT(19)	/* SV_IMGO_DROP_ST */
#define CAMSV_INT_IMGO_DONE_ST		BIT(20)	/* SV_IMGO_DONE_ST */

/* --- CLK_EN --- */
#define CAMSV_CLK_EN_TG_DP_CK_EN	BIT(0)	/* TG_DP_CK_EN b0 */
#define CAMSV_CLK_EN_QBN_DP_CK_EN	BIT(1)	/* QBN_DP_CK_EN b1 */
#define CAMSV_CLK_EN_PAK_DP_CK_EN	BIT(2)	/* PAK_DP_CK_EN b2 */
#define CAMSV_CLK_EN_IMGO_DP_CK_EN	BIT(15)	/* IMGO_DP_CK_EN b15 */

/* --- DCIF_SET --- */
#define CAMSV_DCIF_SET_MASK_DB_LOAD	BIT(7)	/* MASK_DB_LOAD b7 */
#define CAMSV_DCIF_SET_EN_CQ_START	BIT(8)	/* ENABLE_OUTPUT_CQ_START_SIGNAL b8 */
#define CAMSV_DCIF_SET_FOR_DCIF_SUB_EN	BIT(15)	/* FOR_DCIF_SUBSAMPLE_EN b15 */

/* --- SUB_CTRL --- */
#define CAMSV_SUB_CTRL_CENTRAL_SUB_EN	BIT(0)	/* CENTRAL_SUB_EN b0 */

/* --- PAK --- */
#define CAMSV_PAK_MODE_MASK		GENMASK(7, 0)	/* PAK_MODE b7:0 */
#define CAMSV_PAK_DBL_SHIFT		8		/* PAK_DBL_MODE b9:8 */

/* --- MISC --- */
#define CAMSV_MISC_VF_SRC		BIT(0)	/* VF_SRC b0 */

/* --- TG_SEN_MODE --- */
#define CAMSV_TG_SEN_MODE_CMOS_EN	BIT(0)	/* CMOS_EN b0 */
#define CAMSV_TG_SEN_MODE_DBL_DATA_BUS	GENMASK(5, 4)	/* DBL_DATA_BUS b5:4 */
#define CAMSV_TG_SEN_MODE_TIME_STP_EN	BIT(16)	/* TIME_STP_EN b16 */
#define CAMSV_TG_SEN_MODE_SOF_SUB_EN	BIT(17)	/* SOF_SUB_EN b17 */
#define CAMSV_TG_SEN_MODE_VS_SUB_EN	BIT(18)	/* VS_SUB_EN b18 */
#define CAMSV_TG_SEN_MODE_STAGGER_EN	BIT(23)	/* STAGGER_EN b23 */

/* --- TG_VF_CON --- */
#define CAMSV_TG_VF_CON_VFDATA_EN	BIT(0)	/* VFDATA_EN b0 */
#define CAMSV_TG_VF_CON_SINGLE_MODE	BIT(1)	/* SINGLE_MODE b1 */

/* --- TG_SEN_GRAB_PXL / LIN --- */
#define CAMSV_GRAB_PXL_START_MASK	GENMASK(15, 0)	/* PXL_START b15:0 */
#define CAMSV_GRAB_PXL_END_SHIFT	16		/* PXL_END b31:16 */
#define CAMSV_GRAB_LIN_START_MASK	GENMASK(15, 0)	/* LIN_START b15:0 */
#define CAMSV_GRAB_LIN_END_SHIFT	16		/* LIN_END b31:16 */

/* --- TG_PATH_CFG --- */
#define CAMSV_TG_PATH_CFG_SUB_SOF_SRC_SEL GENMASK(21, 20) /* SUB_SOF_SRC_SEL b21:20 */
#define CAMSV_TG_PATH_CFG_DB_LOAD_HOLD	BIT(11)	/* DB_LOAD_HOLD b11 */

/* --- TG_SUB_PERIOD --- */
#define CAMSV_TG_SUB_PERIOD_VS_PERIOD	GENMASK(7, 0)	/* VS_PERIOD b7:0 */
#define CAMSV_TG_SUB_PERIOD_SOF_PERIOD	GENMASK(15, 8)	/* SOF_PERIOD b15:8 */

/* --- PAK_CON.PAK_IN_BIT b20:16 = 14 (mtk_cam_sv_top_config) --- */
#define CAMSV_PAK_CON_PAK_IN_BIT_MASK	GENMASK(20, 16)	/* PAK_IN_BIT b20:16 */
#define CAMSV_PAK_CON_PAK_IN_BIT_VAL	14

/* --- FBC_IMGO_CTL1 --- */
#define CAMSV_FBC_IMGO_CTL1_FBC_RESET	BIT(8)	/* FBC_RESET b8 */
#define CAMSV_FBC_IMGO_CTL1_FBC_DB_EN	BIT(9)	/* FBC_DB_EN b9 */
#define CAMSV_FBC_IMGO_CTL1_FBC_EN	BIT(15)	/* FBC_EN b15 */
#define CAMSV_FBC_IMGO_CTL1_SUB_RATIO	GENMASK(31, 24)	/* SUB_RATIO b31:24 */
#define CAMSV_FBC_IMGO_CTL1_SUB_RATIO_SHIFT 24

/* --- FBC_IMGO_CTL2 (counters) --- */
#define CAMSV_FBC_IMGO_CTL2_IMGO_RCNT	GENMASK(7, 0)	/* IMGO_RCNT b7:0 */
#define CAMSV_FBC_IMGO_CTL2_IMGO_WCNT	GENMASK(15, 8)	/* IMGO_WCNT b15:8 */
#define CAMSV_FBC_IMGO_CTL2_IMGO_FBC_CNT GENMASK(23, 16) /* IMGO_FBC_CNT b23:16 */
#define CAMSV_FBC_IMGO_CTL2_IMGO_DROP_CNT GENMASK(31, 24) /* IMGO_DROP_CNT b31:24 */

/* --- SPECIAL_FUN_EN --- */
#define CAMSV_SPECIAL_FUN_EN_DCM_MODE	BIT(26)	/* DCM_MODE b26 */

/* ------------------------------------------------------------------ */
/* SENINF TOP routing: CSI port 2 -> intf 4 -> mux 1 -> CAM_MUX 3      */
/* ------------------------------------------------------------------ */

#define CAMCAP_SENINF_BASE_DEF	0x1a010000UL
#define CAMCAP_SENINF_SIZE	0x20000UL

/*
 * D-PHY_TOP of the CSI port 2 block.  The receiver's timing follows the
 * sensor's link rate: HS_TRAIL_PARAMETER in the D-PHY and DMY_CYCLE in the
 * CSI2 block are both derived from the MIPI data rate, so a mode switch that
 * changes the rate has to rewrite them.  Everything else about the receiver is
 * rate-independent and stays as the userspace bring-up (scripts/port2_rx71.py,
 * a port of isp71_ref's mtk_cam_seninf_set_csi_mipi()) programmed it.
 */
#define CAMCAP_DPHY_BASE_DEF	0x11c86000UL
#define CAMCAP_DPHY_SIZE	0x1000UL
#define CAMCAP_DPHY_DATA_LANE(i)	(0x20u + 4u * (i))	/* 0x20/0x24/0x28/0x2c */
#define CAMCAP_DPHY_HS_TRAIL		GENMASK(15, 8)
#define CAMCAP_DPHY_HS_TRAIL_EN		BIT(29)
#define CAMCAP_DPHY_CLK_LANE0		0x10u
#define CAMCAP_DPHY_CLK_LANE1		0x14u
#define CAMCAP_CSI2_OFF			0x4a00u	/* CSI2 regs inside SENINF TOP */
#define CAMCAP_CSI2_RESYNC		0x0010u	/* DMY_CYCLE b27:16 */
#define CAMCAP_CSI2_DMY_CYCLE		GENMASK(27, 16)
#define CAMCAP_SENINF_CK		273000000U	/* vendor SENINF_CK */
#define CAMCAP_DPHY_TRAIL_DT		68	/* vendor csi_param.dphy_trail */

/* TOP words - mtk_cam-seninf-top-ctrl.h.  Byte j of word i holds mux 4i+j. */
#define SENINF_TOP_MUX_CTRL(i)		(0x0010u + 4u * (i))
#define SENINF_TOP_PHY_CTRL_CSI2	0x0048u	/* DPHY_EN b0, CPHY_EN b1, CPHY_MODE b9:8 */

/* Per-SENINF-mux block: 0x0d00 + 0x1000*j, j = MUX NUMBER - seninf1-mux.h */
#define SENINF_MUX_BASE(j)		(0x0d00u + 0x1000u * (j))
#define SENINF_MUX_CTRL_0		0x0000u	/* EN b0, IRQ_SW_RST b1, SW_RST b2 */
#define SENINF_MUX_CTRL_1		0x0004u	/* SRC_SEL b3:0, PIX_MODE_SEL b9:8 */
#define SENINF_MUX_OPT			0x0008u	/* HSYNC_POL b16, VSYNC_POL b17 */
#define SENINF_MUX_EN			BIT(0)
#define SENINF_MUX_SW_RST		(BIT(1) | BIT(2))
#define SENINF_MUX_SRC_SEL_MASK		0x0000000fu
#define SENINF_MUX_PIX_MODE_MASK	0x00000300u
#define SENINF_MUX_POL_MASK		0x00030000u
#define SENINF_SRC_MIPI_SENSOR		0x8u	/* SENINF_SOURCE_ENUM, + group */

/* CAM_MUX PCSR: 0x0400 + 0x20*k - cammux-pcsr.h.  NOT a byte array. */
#define SENINF_PCSR_CTRL		0x0000u	/* SRC_SEL b4:0, EN b7, CHK_PIX_MODE b9:8 */
#define SENINF_PCSR_OPT			0x0004u	/* VC b4:0, VC_EN b7, DT b13:8, DT_EN b15 */
#define SENINF_PCSR_IRQ_STATUS		0x000cu
#define SENINF_PCSR_CHK_CTL		0x0014u	/* EXP_HSIZE b15:0, EXP_VSIZE b31:16 */
#define SENINF_PCSR_CHK_RES		0x0018u
#define SENINF_PCSR_EN			BIT(7)
#define SENINF_PCSR_SRC_SEL_MASK	0x0000001fu
#define SENINF_PCSR_CHK_PIX_MODE_MASK	0x00000300u
#define SENINF_PCSR_IRQ_CLR		0x00000103u	/* HSIZE_ERR|VSIZE_ERR|VSYNC */

/* CAM_MUX global control - cammux-gcsr.h */
#define SENINF_GCSR_CTRL		0x0300u	/* SW_RST b0 */
#define SENINF_GCSR_SW_RST		BIT(0)

/* Per-CSI2 block: 0x0a00 + 0x1000*intf; per-port CTRL: 0x0200 + 0x1000*intf */
#define SENINF_CSI2_BASE(i)		(0x0a00u + 0x1000u * (i))
#define SENINF_CTRL_BASE(i)		(0x0200u + 0x1000u * (i))

/* Physical base of the SENINF TOP block used for routing. */
static unsigned long seninf_base = CAMCAP_SENINF_BASE_DEF;
module_param(seninf_base, ulong, 0444);
MODULE_PARM_DESC(seninf_base, "physical base of SENINF TOP (default 0x1a010000)");

/* Physical base of the CSI D-PHY_TOP block (receiver timing). */
static unsigned long dphy_base = CAMCAP_DPHY_BASE_DEF;
module_param(dphy_base, ulong, 0444);
MODULE_PARM_DESC(dphy_base, "physical base of the CSI D-PHY_TOP (default 0x11c86000)");

/*
 * A mode switch can change the sensor's MIPI data rate (0x030e/0x030f):
 * 4000x2256@60 and 1080p240 run at 1964 Mbps per lane where every other mode
 * runs at 1370.  The D-PHY settle/trail and the CSI2 resynchronisation cycle
 * have to follow it or the receiver sees no packets at all.
 */
static bool rx_rate = true;
module_param(rx_rate, bool, 0644);
MODULE_PARM_DESC(rx_rate,
		 "retime the CSI receiver when a mode switch changes the MIPI data rate (default 1)");

/*
 * The power-on table is the vendor's INIT sequence, written once after the
 * hardware reset.  The vendor's own mode switch does not replay it: it writes
 * the mode table alone.  The bring-up scripts happen to replay both (they only
 * ever do it right after a reset), so this switch exists to A/B the two flows
 * on the device.
 */
static bool mode_init_replay = true;
module_param(mode_init_replay, bool, 0644);
MODULE_PARM_DESC(mode_init_replay,
		 "replay the power-on register table when switching sensor modes (default 1)");

/*
 * Mode register tables are byte registers, not 16-bit ones: the vendor lists
 * 0x0306 and 0x0307 as two separate pairs, so one write is a 3-byte transfer
 * (register address plus a single data byte).  Writing them with the 16-bit
 * helper sends a leading zero data byte instead, which the sensor
 * auto-increments into the next register - every value lands one register too
 * far and the mode registers end up reading back as zero.  mode_trace logs
 * each replay write and reads the key registers back afterwards.
 */
static bool mode_trace;
module_param(mode_trace, bool, 0644);
MODULE_PARM_DESC(mode_trace,
		 "log every write of a sensor mode table and verify the result (default 0)");

/* Master switch for the routing that 'arm' performs before it kicks CAMSV. */
static bool route_en = true;
module_param(route_en, bool, 0644);
MODULE_PARM_DESC(route_en,
		 "perform the SENINF/CAM_MUX routing before arming CAMSV (default 1)");

/*
 * The route is static configuration: SENINF mux, CAM_MUX and the CAMSV Ctrl
 * registers all keep their value for as long as the sensor keeps streaming, so
 * re-programming them before every single frame buys nothing and costs a
 * handful of register writes per frame.  By default it therefore happens once,
 * on the first arm after the module is loaded; route_once=0 restores the old
 * "route before every frame" behaviour (which is what the manual /proc flow
 * does).  A successful 'route' or 'reroute' command also clears the flag.
 */
static bool route_once = true;
module_param(route_once, bool, 0644);
MODULE_PARM_DESC(route_once,
		 "program the SENINF/CAM_MUX route once per load instead of before every frame (default 1)");

/*
 * Which colour sits on the (0,0) tap of the 2x2 grid.
 *
 * Default 0 = the (0,0) tap is red, i.e. an RGGB grid, which is what this
 * sensor is: the vendor driver reports SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_R
 * (src/imx586_Sensor.c:256, with .mirror = IMAGE_NORMAL), and the phase
 * correlation measured off the wire in docs/CAMERA_CAPTURE_WORKING.md 7.2
 * agrees with it (the two greens correlate at 0.98, the two coloured taps at
 * 0.84 -- the greens are the ones that sit on the same checkerboard).
 *
 * Setting 1 declares the opposite (BGGR) grid.  The AWB statistics and the
 * conversion both read this so the two stay consistent, and it is read on
 * every frame, so it can be flipped live:
 *
 *	echo 1 > /sys/module/cam_cap/parameters/rb_swap
 *
 * For a grey-world AWB a bare relabelling is invisible: every tap still gets
 * the gain that balances it.  What the setting really moves is the coefficient
 * matrix that turns the three taps into Cb/Cr: the right one is exactly
 * BT.601, the wrong one transposes the two chroma axes (mild hue error, not a
 * plain red/blue exchange).
 *
 * The plain exchange this parameter was originally introduced to explain
 * ("reds look blue, cyan comes out gold") was the YUYV chroma byte order in
 * cam_v4l2_convert_band() instead; see the comment at the store there.
 */
static bool rb_swap = false;
module_param(rb_swap, bool, 0644);
MODULE_PARM_DESC(rb_swap,
		 "(0,0) tap is blue and the (1,1) tap red (BGGR); 0 = they are red and blue (RGGB, this sensor; default 0)");

static unsigned int route_intf = 4;	/* CSI port 2 -> SENINF_5 */
module_param(route_intf, uint, 0444);
MODULE_PARM_DESC(route_intf, "SENINF interface index carrying CSI port 2 (default 4)");

static unsigned int route_mux = 1;	/* SENINF mux number */
module_param(route_mux, uint, 0444);
MODULE_PARM_DESC(route_mux, "SENINF mux number driven from that intf (default 1)");

static unsigned int cammux = 3;		/* cam_mux 3 -> CAMSV1 */
module_param(cammux, uint, 0444);
MODULE_PARM_DESC(cammux,
		 "CAM_MUX number wired into CAMSV1, DT cammux-id (default 3)");

static unsigned int route_pix_mode = 1;	/* log2 pixel-rate multiplier */
module_param(route_pix_mode, uint, 0444);
MODULE_PARM_DESC(route_pix_mode,
		 "mux PIX_MODE_SEL / cam_mux CHK_PIX_MODE (default 1 = vendor log value)");

static unsigned int route_dt = 0x2b;	/* data type of the VC0 RAW10 stream */
module_param(route_dt, uint, 0444);
MODULE_PARM_DESC(route_dt, "MIPI data type matched by CAM_MUX OPT (default 0x2b)");

static unsigned int route_vc;		/* virtual channel */
module_param(route_vc, uint, 0444);
MODULE_PARM_DESC(route_vc, "MIPI virtual channel matched by CAM_MUX OPT (default 0)");

static unsigned int exp_hsize = 4000;
module_param(exp_hsize, uint, 0444);
MODULE_PARM_DESC(exp_hsize, "CAM_MUX CHK_CTL.EXP_HSIZE (default 4000)");

static unsigned int exp_vsize = 3000;
module_param(exp_vsize, uint, 0444);
MODULE_PARM_DESC(exp_vsize, "CAM_MUX CHK_CTL.EXP_VSIZE (default 3000)");

/*
 * Raw-frame geometry of the sensor mode that is currently streaming.
 *
 * exp_hsize/exp_vsize are what the sensor puts on the wire; v4l2_src_stride is
 * how many bytes a single raw line occupies.  The sensor is RAW10, i.e. 1.5
 * bytes per pixel plus whatever the vendor tables pad, so 0 means "derive it
 * from exp_hsize" -- the default 4000x3000 preview mode then gets the verified
 * 6000-byte stride, and replaying another vendor table only needs
 *   insmod cam_cap.ko v4l2_enable=1 exp_hsize=W exp_vsize=H
 * The V4L2 output is cam_src_{w,h} / v4l2_bin, i.e. half size by default.
 */
static unsigned int v4l2_src_stride;
module_param(v4l2_src_stride, uint, 0444);
MODULE_PARM_DESC(v4l2_src_stride,
		 "bytes per raw line in the capture buffer (0 = 1.5 x exp_hsize)");

static unsigned int cam_src_w, cam_src_h, cam_src_stride;
static unsigned int cam_bin = 2;	/* normalised v4l2_bin: 1 or 2 */

/* ------------------------------------------------------------------ */
/* Module parameters                                                  */
/* ------------------------------------------------------------------ */

/*
 * frame_bytes: size of the physically contiguous capture buffer, in bytes.
 * 16 MiB by default, which covers a 4000x3000 RAW10 frame at two bytes per
 * pixel (12 MB) and leaves slack for the CAMSV output padding.
 */
static unsigned long frame_bytes = 16 * 1024 * 1024;
module_param(frame_bytes, ulong, 0444);
MODULE_PARM_DESC(frame_bytes,
		 "size of the physically contiguous frame buffer in bytes (default 16 MiB)");

/* Physical base of the CAMSV instance to drive. CAMSV1 = 0x1a110000. */
static unsigned long camsv_base = CAMCAP_CAMSV_BASE_DEF;
module_param(camsv_base, ulong, 0444);
MODULE_PARM_DESC(camsv_base,
		 "physical base address of the CAMSV register block (default 0x1a110000 = CAMSV1)");

/*
 * dbl_data_bus: TG_SEN_MODE.DBL_DATA_BUS (b5:4).  The vendor mtk_cam_sv_tg_config()
 * selects it from the sensor format enum (0 -> 0b00, 1 -> 0b10, 2 -> 0b11,
 * 3 -> 0b01).  We default to 0b00 (single data bus).
 */
static unsigned int dbl_data_bus;
module_param(dbl_data_bus, uint, 0444);
MODULE_PARM_DESC(dbl_data_bus,
		 "TG_SEN_MODE.DBL_DATA_BUS value 0..3 (default 0 = single data bus)");

/*
 * pak_mode: PAK.PAK_MODE (b7:0).  -1 = derive from the cfg <fmt> argument the
 * way mtk_cam_sv_pak_sel() does (fmt 1 -> 0x80 GR/BG, 2 -> 0x81 BA, 4 -> 0x8F).
 */
static int pak_mode = -1;
module_param(pak_mode, int, 0444);
MODULE_PARM_DESC(pak_mode,
		 "PAK.PAK_MODE byte, or -1 to derive it from the cfg fmt argument (default -1)");

/*
 * pak_dbl: PAK.PAK_DBL_MODE (b9:8), the same value as FMT_SEL.TG1_SW (b6:5).
 * The vendor's own config record hard-codes 0x300 | pak_sel(), i.e. 3, and
 * mtk_cam_sv_pak_sel() folds its second argument in as ((w1 & 3) << 8).
 * This is NOT the cfg <fmt> bit-depth code - the two must not be coupled.
 */
static unsigned int pak_dbl = 3;
module_param(pak_dbl, uint, 0444);
MODULE_PARM_DESC(pak_dbl,
		 "PAK.PAK_DBL_MODE b9:8 / FMT_SEL.TG1_SW b6:5 (default 3)");

/*
 * fmt_sel: full 32-bit override for FMT_SEL (0x044).  0 = write the cfg <fmt>
 * bit-depth code, which is what the vendor's own top_config() does.
 */
static unsigned int fmt_sel;
module_param(fmt_sel, uint, 0444);
MODULE_PARM_DESC(fmt_sel,
		 "full FMT_SEL (0x044) value, or 0 to write the cfg fmt code (default 0)");

/*
 * fbc_en: 0 (default) leaves FBC compression off: FBC_IMGO_CTL1 is only given
 * FBC_DB_EN, so the buffer holds raw pixels.  1 runs the full
 * mtk_cam_sv_fbc_enable() path (SUB_RATIO, FBC_EN, clear FBC_DB_EN).
 */
static unsigned int fbc_en;
module_param(fbc_en, uint, 0444);
MODULE_PARM_DESC(fbc_en,
		 "1 = enable FBC compression on IMGO (default 0 = raw pixels in the buffer)");

/* sub_ratio: FBC_IMGO_CTL1.SUB_RATIO (b31:24), only used when fbc_en=1 */
static unsigned int sub_ratio;
module_param(sub_ratio, uint, 0444);
MODULE_PARM_DESC(sub_ratio, "FBC_IMGO_CTL1.SUB_RATIO (b31:24), only used when fbc_en=1");

/*
 * con_set: selects the IMGO_CON0..4 DMA QoS threshold set.  mtk_cam_sv_dmao_config
 * picks between two literal sets based on a driver-internal field
 * ([x19+0x10] <= 9).  0 = the "<= 9" set (set A), 1 = the else set (set B).
 */
static unsigned int con_set;
module_param(con_set, uint, 0444);
MODULE_PARM_DESC(con_set,
		 "IMGO_CON0..4 threshold set: 0 = set A (0x10000300..), 1 = set B (0x10000080..)");

/* int_en: value written to INT_EN by mtk_cam_sv_top_config (0x00011070) */
static unsigned int int_en = CAMCAP_INT_EN_DEF;
module_param(int_en, uint, 0444);
MODULE_PARM_DESC(int_en, "value written to INT_EN (default 0x00011070)");

/*
 * single_mode: set TG_VF_CON.SINGLE_MODE (b1) together with VFDATA_EN when
 * arming, so the CAMSV TG stops itself after exactly one frame instead of
 * free-running.
 *
 * Why this matters: arm() writes IMGO_BASE_ADDR first and only then sets
 * VFDATA_EN, and completion is detected by polling INT_STATUS with
 * sub-jiffy sleeps - while one frame only takes ~65 ms.  Between the
 * IMGO_DONE interrupt and the vf_off() that follows the poll, the free-running
 * TG can already have started the NEXT frame into the same buffer and
 * overwritten its first few hundred lines.  SINGLE_MODE moves the stop into
 * the hardware, so the buffer holds exactly one complete frame no matter how
 * late the host gets around to reading INT_STATUS.
 */
static bool single_mode;
module_param(single_mode, bool, 0644);
MODULE_PARM_DESC(single_mode,
		 "arm with TG_VF_CON.SINGLE_MODE so the TG stops after one frame (default 0)");

/*
 * arm_trace: log every arm's sequence number, address and final interrupt
 * status to dmesg.  Two pr_info() per frame are not free (the console is
 * synchronous), and a streaming application produces several per second, so
 * this is off by default and can be flipped at runtime for a measurement.
 */
static bool arm_trace;
module_param(arm_trace, bool, 0644);
MODULE_PARM_DESC(arm_trace, "log every arm (address/status) to dmesg (default 0)");

/*
 * alias_try_memremap: try memremap(..., MEMREMAP_WC) for the readback alias
 * before falling back to ioremap_wc().  Off by default: memremap() refuses to
 * alias System RAM with a non-WB attribute (see kernel/iomem.c, "memremap
 * attempted on ram") and WARNs once, so on a buffer from alloc_pages_exact()
 * this is expected to fail.  Kept so the behaviour can be demonstrated.
 */
static bool alias_try_memremap;
module_param(alias_try_memremap, bool, 0644);
MODULE_PARM_DESC(alias_try_memremap,
		 "try memremap(MEMREMAP_WC) before ioremap_wc() for the readback alias (default 0)");

/*
 * use_dma_alloc: get the frame buffer from dma_alloc_coherent() (CMA) instead
 * of alloc_pages_exact() (buddy).  On for the reason documented in
 * cam_cap_init(): CONFIG_ARCH_FORCE_MAX_ORDER=10 caps alloc_pages_exact() at
 * 4 MiB, and no uncached alias of System RAM can be created on this kernel.
 */
static bool use_dma_alloc = true;
module_param(use_dma_alloc, bool, 0644);
MODULE_PARM_DESC(use_dma_alloc,
		 "allocate the frame buffer with dma_alloc_coherent() from CMA (default 1)");

/*
 * iommu_dev: give the capture buffer to a device that sits behind the display
 * IOMMU instead of inventing a device with no firmware node.  Without this the
 * IMGO address is a physical address while the hardware still translates it
 * through larb0/port2, so every burst faults with pa=0 and the buffer stays
 * empty.  With it, cam_frame_pa becomes an IOVA.  See CAMCAP_DT_NODE_NAME.
 */
static bool iommu_dev = true;
module_param(iommu_dev, bool, 0644);
MODULE_PARM_DESC(iommu_dev,
		 "put the frame buffer behind the disp IOMMU via a DT node (default 1)");

/*
 * map_iova: the fixed IOVA the CMA buffer is mapped at, so CAMSV IMGO writes
 * through the IOMMU into it.  0 selects the old behaviour of allocating the
 * buffer straight out of the IOMMU-backed device, which cannot work here: a
 * device with no "dma-coherent" property takes the non-contiguous
 * iommu_dma_alloc_noncontiguous() path, and __arm_v7s_map() then aborts with
 *
 *   WARNING: drivers/iommu/io-pgtable-arm-v7s.c:417 at __arm_v7s_map+0x408
 *   (WARN_ON(!selftest_running); return -EEXIST;  "We require an unmap first")
 *
 * so the buffer only ever came out at 1 MiB.  Mapping a CMA allocation with
 * iommu_map() instead keeps the whole 16 MiB and is a plain, single call.
 *
 * The default must sit in the dom0 IOVA window (SZ_4K .. SZ_4G*4 - SZ_4K) and
 * clear of the sub-regions mtk_iommu_get_resv_regions() reserves inside it,
 * the first of which is {0x20000000, 0x12c00000}.  256 MiB + 16 MiB is safe.
 */
static unsigned long map_iova = 0x10000000;
module_param(map_iova, ulong, 0644);
MODULE_PARM_DESC(map_iova,
		 "fixed IOVA for the frame buffer, 0 = allocate from the IOMMU device (default 0x10000000)");

/*
 * ---- V4L2 capture device -------------------------------------------
 *
 * v4l2_enable=1 additionally registers a normal V4L2 video capture device
 * (/dev/videoN, YUYV, 2000x1500) that streams frames on demand, so ordinary
 * camera software (cheese, gnome-snapshot, gstreamer's v4l2src, ffmpeg,
 * v4l2-ctl) can open it like any other webcam.
 *
 * It reuses exactly the capture path that the experiments proved out - CAMSV1
 * still writes the same 18 000 000-byte CMA buffer at IOVA 0x10000000 - and
 * converts in-kernel: 12-bit RAW -> 2x2 binning -> RGGB debayer -> white
 * balance -> tone curve -> YUYV.
 *
 * Enabling it forces the dozen parameters that path needs (see cam_cap_init),
 * so `insmod cam_cap.ko v4l2_enable=1` is enough.  The sensor, its regulators,
 * MCLK and the D-PHY/SENINF routing still have to bring up first: see
 * scripts/zz_v80.sh.
 */
static bool v4l2_enable;
module_param(v4l2_enable, bool, 0444);
MODULE_PARM_DESC(v4l2_enable,
		 "register a V4L2 capture device (/dev/videoN) and stream frames (default 0)");

/*
 * Overlap capture and conversion with extra frame buffers.  Serially a frame
 * costs arm + conv (the CAMSV only reports "done" after the sensor has shifted
 * the whole frame out, and the conversion then has to run before the next arm),
 * which caps the rate at ~1/(arm+conv); with two slots it costs max(arm, conv)
 * instead, and with three the arm thread no longer has to wait for a
 * conversion to release the slot it wants (see the pipe_slots parameter).  The
 * extra raw buffers come from alloc_contig_pages(); if that fails the driver
 * quietly stays serial.
 */
static bool pipeline = true;
module_param(pipeline, bool, 0644);
MODULE_PARM_DESC(pipeline,
		 "capture into a second buffer while the previous frame is converted (default 1; 0 = arm and convert serially)");

/*
 * How many raw frame buffers the capture pipeline rotates through.
 *
 * Two buffers hide the conversion behind the sensor readout, but they do not
 * keep the arm thread at the sensor: with only two slots, arming frame N+1 has
 * to wait for the converter to release frame N-1, which costs the conversion
 * plus (on average) half a sensor period -- measured as period = conv + P/2.
 * A third slot breaks that dependency, so the period becomes max(P, conv) and
 * a conversion that is shorter than the sensor period stops costing anything.
 */
static unsigned int pipe_slots = 3;
module_param(pipe_slots, uint, 0644);
MODULE_PARM_DESC(pipe_slots,
		 "raw frame buffers in the capture pipeline, 2 to 4 (default 3)");

/* Output geometry.  Must be CAMCAP_SRC_{WIDTH,HEIGHT} / CAMCAP_BIN. */
static unsigned int out_width = 2000;
module_param(out_width, uint, 0644);
MODULE_PARM_DESC(out_width, "V4L2 output width after 2x2 binning (default 2000)");

static unsigned int out_height = 1500;
module_param(out_height, uint, 0644);
MODULE_PARM_DESC(out_height, "V4L2 output height after 2x2 binning (default 1500)");

/*
 * Output binning (load time, normalised in cam_cap_init()).
 *
 *   2  the original path: every 2x2 raw block becomes one output pixel by
 *      averaging its two greens.  Cheap -- and for the 4000x3000 modes the
 *      only option, because a full-size frame is 24 MB out of a converter
 *      that tops out at ~200 Mpx/s.
 *
 *   1  full-size output: every raw pixel gets its missing colours from its
 *      neighbours (bilinear), so the 1920x1080 sensor modes stop coming out
 *      as 960x540.  This is what a "real 1080p" recording needs.
 *
 * Both paths write the same YUYV, same 180 degree flip, same chroma slots --
 * only the resolution and the interpolation differ.
 */
static unsigned int v4l2_bin = 2;
module_param(v4l2_bin, uint, 0644);
MODULE_PARM_DESC(v4l2_bin, "output binning: 2 = 2x2 average (default), 1 = full size bilinear");

/*
 * How the full size converter gets its samples.  The bilinear taps of adjacent
 * output pixels overlap, so decoding three bytes into two 12-bit samples again
 * for every neighbour repeats the same shift/mask/mask about eight times per
 * output pixel -- and that decoding, not the interpolation arithmetic, is what
 * the conversion actually costs.  Expanding each raw row once into u16 scratch
 * rows removes it.  The per-pixel version stays as the reference and as the
 * fallback when the scratch allocation is refused.
 */
static bool v4l2_full_cache = true;
module_param(v4l2_full_cache, bool, 0644);
MODULE_PARM_DESC(v4l2_full_cache, "full size converter: 1 = expand each raw row once (default), 0 = per-pixel bilinear");

/*
 * Tone curve parameters.  The sensor pedestal on this part reads ~248 of 4095
 * and the room is dim, so the raw frame needs a black-level subtract before the
 * tone curve.
 *
 * The pre-gain is applied BEFORE the square root, so it is not a cosmetic
 * "digital gain": every step of it also pushes more of the scene past the top
 * of the curve.  At the old default of 768 (3.0x) anything brighter than
 * (4095-248)/3 + 248 = 1530 RAW came out pure white, which is why the picture
 * looked washed out once the in-driver AE started pushing the mean to ~1200.
 * 256 (1.0x) leaves the whole 12-bit range inside the curve.
 */
static unsigned int v4l2_black = 248;
module_param(v4l2_black, uint, 0644);
MODULE_PARM_DESC(v4l2_black, "black level subtracted from the 12-bit RAW value (default 248)");

static unsigned int v4l2_gain_q8 = 256;
module_param(v4l2_gain_q8, uint, 0644);
MODULE_PARM_DESC(v4l2_gain_q8,
		 "linear pre-gain applied before the tone curve, Q8 (default 256 = 1.0x)");

static unsigned int wb_r_q8 = 320;
module_param(wb_r_q8, uint, 0644);
MODULE_PARM_DESC(wb_r_q8, "red white-balance gain, Q8 (default 320 = 1.25x for the RGGB tap assignment; AWB re-converges anyway)");

static unsigned int wb_b_q8 = 434;
module_param(wb_b_q8, uint, 0644);
MODULE_PARM_DESC(wb_b_q8, "blue white-balance gain, Q8 (default 434 = 1.69x for the RGGB tap assignment; AWB re-converges anyway)");

/* --- output-stage knobs that are also V4L2 controls (see the ctrl block) --- */

static int out_brightness;		/* -128..128, added after the pre-gain */
module_param(out_brightness, int, 0644);
MODULE_PARM_DESC(out_brightness, "brightness offset, -128..128 (default 0)");

static int out_contrast = 128;		/* 0..255, 128 = 1.0x around mid grey */
module_param(out_contrast, int, 0644);
MODULE_PARM_DESC(out_contrast, "contrast around mid grey, 128 = 1.0x (default 128)");

static int out_saturation = 128;	/* 0..255, 128 = 1.0x chroma */
module_param(out_saturation, int, 0644);
MODULE_PARM_DESC(out_saturation, "chroma gain, 128 = 1.0x (default 128)");

/*
 * ------------------------------------------------------------------
 * In-driver AE / AWB
 * ------------------------------------------------------------------
 *
 * The frame data never leaves the kernel, so the one place that can close
 * the exposure/white-balance loops is this module.  Both loops run once per
 * captured frame inside the capture kthread:
 *
 *   - AE measures the *raw* green mean (pedestal included, before any tone
 *     curve) and steers the sensor through I2C: coarse integration time
 *     (0x0202) first because it costs no SNR, then analogue gain (0x0204),
 *     then digital gain (0x020e) as the last resort.  Bands, not servos:
 *     one bounded step per frame keeps it from oscillating.
 *   - AWB is grey-world on the raw per-channel means and steers the two Q8
 *     white-balance gains the LUTs are built from.  It only moves a fraction
 *     of the way each frame and refuses to chase scenes that are too dark.
 *
 * Writing the sensor from the kernel is what makes this work, and it is the
 * only way this module talks to a bus someone else could be using: userspace
 * i2ctransfer on the same bus must not run at the same time.
 */
static bool sensor_ctl = true;		/* master switch for sensor writes */
module_param(sensor_ctl, bool, 0644);
MODULE_PARM_DESC(sensor_ctl, "let the AE loop write exposure/gain over I2C (default 1)");

static unsigned int i2c_bus = 10;	/* 11d05000.i2c */
module_param(i2c_bus, uint, 0644);
MODULE_PARM_DESC(i2c_bus, "I2C bus number of the sensor (default 10)");

/*
 * The adapter on bus 10 advertises itself as "i2c-mt65xx", not with its device
 * tree node name "11d05000.i2c" - checking for the DT name silently rejected
 * the right bus and left the AE/AWB loop turning knobs that went nowhere.
 * An empty string disables the sanity check.
 */
static char *i2c_name = "i2c-mt65xx";
module_param(i2c_name, charp, 0644);
MODULE_PARM_DESC(i2c_name,
		 "substring the I2C adapter name must contain, empty to skip the check (default i2c-mt65xx)");

static unsigned int i2c_addr = 0x10;
module_param(i2c_addr, uint, 0644);
MODULE_PARM_DESC(i2c_addr, "7-bit I2C address of the sensor (default 0x10)");

static bool ae_enable = true;		/* initial state of V4L2_CID_EXPOSURE_AUTO */
module_param(ae_enable, bool, 0644);
MODULE_PARM_DESC(ae_enable, "start with auto exposure enabled (default 1)");

static bool ae_trace;			/* log every AE correction */
module_param(ae_trace, bool, 0644);
MODULE_PARM_DESC(ae_trace, "log every auto-exposure correction to dmesg (default 0)");

static unsigned int ae_target = 1000;	/* raw green mean the AE loop aims at */
module_param(ae_target, uint, 0644);
MODULE_PARM_DESC(ae_target, "target raw green mean, 0..4095 (default 1000)");

static unsigned int ae_band = 120;	/* +/- dead band around ae_target */
module_param(ae_band, uint, 0644);
MODULE_PARM_DESC(ae_band, "AE dead band around the target (default 120)");

static unsigned int ae_clip_pct = 2;	/* back off above this % of clipped green */
module_param(ae_clip_pct, uint, 0644);
MODULE_PARM_DESC(ae_clip_pct, "back off when more than this %% of green clips (default 2)");

static bool awb_enable = true;		/* initial state of V4L2_CID_AUTO_WHITE_BALANCE */
module_param(awb_enable, bool, 0644);
MODULE_PARM_DESC(awb_enable, "start with auto white balance enabled (default 1)");

static unsigned int awb_rate = 2;	/* move 1/2^awb_rate of the way per frame */
module_param(awb_rate, uint, 0644);
MODULE_PARM_DESC(awb_rate, "AWB correction rate, 1/2^n per frame (default 2)");

static unsigned int awb_min_level = 200;	/* ignore frames darker than this */
module_param(awb_min_level, uint, 0644);
MODULE_PARM_DESC(awb_min_level, "raw green mean below which AWB freezes (default 200)");

/*
 * Sensor register defaults.  The values are the ones the userspace
 * experiments settled on (docs/CAMERA_CAPTURE_WORKING.md section 5):
 * exposure 0x0380 integrates the longest useful time, 0x0204 saturates at
 * 0x0300 (writing 0x0f00 reads back as 0x0300), 0x020e is a 10-bit
 * fixed-point gain with 0x0400 = 1.0x.
 */
static unsigned int exp_def = 0x0380;
module_param(exp_def, uint, 0644);
MODULE_PARM_DESC(exp_def, "initial sensor coarse integration time, 0x0202 (default 0x0380)");

/*
 * The AE may not integrate longer than one frame time.  0x0202 is a full
 * 16-bit register and responds linearly well past 0x6000, but the sensor
 * stretches the frame period to fit the exposure: with VTS 0x0e4a (3658 lines,
 * 33.3 ms) a 0x3000 exposure turns the 30 fps mode into a 9 fps one.  Capping
 * here is what makes the frame rate a property of the mode instead of a
 * property of the room; a dark scene is then answered with analogue and
 * digital gain, which costs noise rather than rate.
 *
 * The bring-up writes VTS 0x0ce4 (3300 lines), which measures a 29.8 ms frame
 * period (~33.5 fps); that leaves the loop, which also has to convert the
 * previous frame, enough slack to stay above 30 fps.  VTS 0x0dac (3500) was
 * 31.7 ms and landed at 29.6-31.5 fps, i.e. straddling the 30 fps line.
 * Keep this at or below whatever VTS the bring-up programs, because the sensor
 * measures the frame period against VTS and stretches it when the exposure
 * reaches it.
 */
static unsigned int exp_max = 0x0c64;
module_param(exp_max, uint, 0644);
MODULE_PARM_DESC(exp_max,
		 "largest coarse integration time the AE loop will ask for (default 0x0c64 = 3172 lines, 128 lines below the 3300 line VTS the bring-up programs; the sensor stretches the frame period once the exposure reaches VTS: 0x3000 gave ~9 fps).  Must not exceed the VTS programmed by the sensor bring-up minus a small margin.  Raise it only for low-light stills where the frame rate does not matter");

static unsigned int again_def = 0x0300;
module_param(again_def, uint, 0644);
MODULE_PARM_DESC(again_def, "initial sensor analogue gain, 0x0204 (default 0x0300)");

static unsigned int again_max = 0x03a0;
module_param(again_max, uint, 0644);
MODULE_PARM_DESC(again_max,
		 "largest analogue gain the AE loop will ask for (default 0x03a0 = 1.55x over the 0x0300 = 1.0x default); 0x0204 keeps only its low 10 bits, so 0x0f00/0x3f00 silently become 0x0300, and above 0x03a0 the code-to-gain curve turns sharply up (0x03c0 is already 2.0x, 0x03f0 4.9x) so a 1.33x loop step overshoots the dead band and hunts");

static unsigned int dgain_def = 0x0400;
module_param(dgain_def, uint, 0644);
MODULE_PARM_DESC(dgain_def, "initial sensor digital gain, 0x020e (default 0x0400 = 1.0x)");

static unsigned int dgain_max = 0x1000;
module_param(dgain_max, uint, 0644);
MODULE_PARM_DESC(dgain_max, "largest digital gain the AE loop will ask for (default 0x1000 = 4.0x)");

/* ------------------------------------------------------------------ */
/* State                                                              */
/* ------------------------------------------------------------------ */

static void __iomem *cam_base;		/* CAMSV register block */
static void __iomem *cam_seninf;	/* SENINF TOP block (routing, CSI2) */
static void __iomem *cam_dphy;		/* CSI D-PHY_TOP (receiver timing) */
static bool cam_mem_region_ok;		/* true: request_mem_region() succeeded */

static void *cam_frame;			/* CPU address of the capture buffer */
static unsigned long cam_buf_size;	/* bytes actually obtained (<= frame_bytes) */
static phys_addr_t cam_frame_pa;	/* IMGO target: IOVA or physical address */
static bool cam_buf_is_dma;		/* true: dma_alloc_coherent() owns it */
static struct platform_device *cam_pdev;	/* throwaway device (no IOMMU) */
static struct device *cam_dma_dev;	/* device that owns the capture buffer */
static bool cam_dma_dev_is_iommu;	/* true: cam_frame_pa is an IOVA */
static struct device *cam_iommu_dev;	/* DT-backed device behind the IOMMU */
static struct iommu_domain *cam_iommu_dom;	/* its domain, borrowed */
static phys_addr_t cam_buf_phys;	/* real physical address of the buffer */
static size_t cam_iommu_mapped;		/* bytes iommu_map()ed at map_iova */
static struct platform_device *cam_dt_pdev;	/* DT-backed device (IOMMU path) */
static bool cam_dt_pdev_ref;		/* true: we hold a platform_device ref */
static struct of_changeset cam_dt_cs;	/* changeset that added cam_dt_node */
static bool cam_dt_cs_inited;		/* true: of_changeset_init() was done */
static bool cam_dt_applied;		/* true: of_changeset_apply() succeeded */
static struct device_node *cam_dt_node;	/* owned by cam_dt_cs, do not put */
static void __iomem *cam_alias;		/* uncached readback alias */
static bool cam_alias_is_memremap;	/* true: memunmap(), false: iounmap() */
static bool cam_alias_is_linear;	/* true: cached linear map, do not unmap */
static void *cam_frame_wb;		/* cacheable linear-map alias of cam_buf_phys */
static void *cam_bounce;		/* kmalloc'd readback bounce buffer */

static struct proc_dir_entry *cam_proc;		/* /proc/camcap */
static struct proc_dir_entry *cam_proc_info;	/* /proc/camcap_info */

/* Serialises the register sequences and the shared status fields */
static DEFINE_MUTEX(cam_lock);

static bool cam_vf_on;			/* last known TG_VF_CON.VFDATA_EN */
static bool cam_frame_ready;		/* set by the arm poll */
static u32 cam_last_int_status;		/* final INT_STATUS of the last arm */
static u32 cam_last_fbc_ctl2;		/* final FBC_IMGO_CTL2 of the last arm */
static unsigned int cam_arm_count;	/* number of successful arms */
static bool cam_route_done;		/* route_once: already programmed? */
static u32 cam_last_seq;		/* FRAME_SEQ_NO of the last arm */
static int cam_last_ret;		/* last command result */

/*
 * Diagnostics.  cam_probe_trace makes cam_cap_arm() poll INT_STATUS at 1 ms
 * instead of 5 ms and log every sample, which answers two questions that decide
 * how the frame rate can be raised above ~20 fps:
 *
 *   1. when the sensor's frame start (INT_VS_ST) arrives relative to the arm,
 *      i.e. how much of the arm is "wait for the next frame start";
 *   2. whether FBC_IMGO_CTL2 counts the DMA progress down the frame (it does
 *      not have to be enabled for FBC compression to keep counting), which is
 *      what an incremental - overlap transmission with conversion - converter
 *      needs in order to know how many lines are safe to read.
 *
 * It is only set by the "probe" proc command and cleared straight after, so
 * the normal capture path keeps its 5 ms poll.
 */
static bool cam_probe_trace;

/*
 * AE/AWB state.  Everything here is written by the capture kthread (and, for
 * the controls, by the V4L2 control callbacks) and read by /proc.  The values
 * are independent scalars, so the races are benign: a reader either sees the
 * previous frame's value or this one's.
 */
struct cam_sensor_state {
	unsigned int exposure;	/* 0x0202 coarse integration time */
	unsigned int again;	/* 0x0204 analogue gain */
	unsigned int dgain;	/* 0x020e digital gain */
};

static struct cam_sensor_state cam_sensor;	/* wanted */
static bool cam_sensor_valid;			/* cam_sensor_hw is meaningful */
static struct cam_sensor_state cam_sensor_hw;	/* last written to the sensor */

/* the /proc diagnostic commands below need this before its definition */
static void cam_sensor_apply(void);

/*
 * Selected sensor mode (see the "sensor modes" section further down): an index
 * into cam_imx582_modes[] from the generated header, or -1 while the run-time
 * geometry comes from the exp_hsize/exp_vsize/v4l2_bin parameters alone.
 */
static int cam_mode_idx = -1;
static unsigned int cam_mode_fps;	/* fps_x100 of the selected mode */
static unsigned int cam_rx_mbps;	/* MIPI rate the receiver is timed for */

static struct i2c_adapter *cam_i2c_adap;	/* NULL until first use */

/* grey-world white balance, Q8; the LUTs are built from these */
static unsigned int cam_wb_r_cur = 263;
static unsigned int cam_wb_b_cur = 614;

/* last frame statistics, for /proc/camcap_info and the governor */
struct cam_stats {
	u64 sum_r, sum_g, sum_b;
	u64 count;		/* sampled 2x2 blocks (= green sample pairs) */
	u32 min_g, max_g;
	u64 clip;		/* green samples at or above CAMCAP_CLIP_LEVEL */
	u64 dark;		/* green samples at or below the pedestal */
	u32 mean_r, mean_g, mean_b;	/* per-channel means, raw 12-bit */
	/*
	 * Focus metric: the sum of |dY| between adjacent output pixels, over the
	 * same sampled rows as the statistics.  Blur removes exactly this, it is
	 * grey-world (so the AWB cannot move it) and it is monotone in the LUT
	 * gamma, so it orders lens positions the same way the eye does.
	 */
	u64 fv;
	u64 fv_n;		/* gradients summed, for the mean */
	u64 fv_y;		/* luma summed over the same pairs */
};

static struct cam_stats cam_stats;
static unsigned int cam_stat_mean_r, cam_stat_mean_g, cam_stat_mean_b;
static unsigned int cam_stat_clip_pct;
static unsigned int cam_ae_frames;	/* frames the AE loop has adjusted */
static unsigned int cam_awb_frames;
static unsigned int cam_stats_reads;	/* times /proc asked for stats */

/*
 * AE loop state.  Declared up here because /proc reports it; the loop that
 * maintains it is next to the governor further down.
 */
static unsigned int cam_ae_settle;	/* frames still to be ignored */
static unsigned int cam_ae_mean_s;	/* smoothed green mean, 0 = no sample */

/*
 * Defined further down, next to the V4L2 section they belong to, but the
 * /proc command handler above them wants to call them.
 */
static void cam_raw_stats(struct cam_stats *st);
static void cam_lut_rebuild(void);
static int cam_mode_match(unsigned int w, unsigned int h, unsigned int bin,
			  unsigned int fps_x100);
static int cam_mode_program(int idx);
static int cam_mode_select(const char *name, unsigned int bin);
static unsigned int cam_lut_ver;
static const char *cam_ae_state(void);
static int cam_af_cmd(const char *line);
static int cam_af_info(char *info, int i, size_t size);

/* live mirrors of the two auto controls, so /proc can report them */
static bool cam_ae_auto = true;
static bool cam_awb_auto = true;

/*
 * Human-readable description of where the capture buffer and its CPU mapping
 * came from; used by both the load log and /proc/camcap_info.
 */
static const char *cam_buf_kind(void)
{
	if (cam_buf_is_dma)
		return cam_dma_dev_is_iommu
			? "dma_alloc_coherent (CMA, non-cached, IOMMU IOVA)"
			: "dma_alloc_coherent (CMA, non-cached, physical)";
	if (cam_alias_is_memremap)
		return "memremap(MEMREMAP_WC)";
	if (cam_alias_is_linear)
		return "cached linear map (may be stale)";
	return "ioremap_wc";
}

/* ------------------------------------------------------------------ */
/* Register helpers                                                   */
/* ------------------------------------------------------------------ */

static inline u32 cam_rd(u32 off)
{
	return readl(cam_base + off);
}

static inline void cam_wr(u32 off, u32 val)
{
	writel(val, cam_base + off);
}

static inline void cam_set(u32 off, u32 bits)
{
	cam_wr(off, cam_rd(off) | bits);
}

static inline void cam_clr(u32 off, u32 bits)
{
	cam_wr(off, cam_rd(off) & ~bits);
}

/* SENINF TOP accessors (routing) */
static inline u32 sen_rd(u32 off)
{
	return readl(cam_seninf + off);
}

static inline void sen_wr(u32 off, u32 val)
{
	writel(val, cam_seninf + off);
}

/* ------------------------------------------------------------------ */
/* 8.8 step 2 - mtk_cam_sv_setup_cfg_info() window / geometry latch    */
/* ------------------------------------------------------------------ */

struct camcap_cfg {
	u32 fmt;	/* FMT_SEL.TG1_FMT: 1 = GR/BG, 2 = BA/RG, 4 = 8AB1 */
	u32 pxl_start;
	u32 pxl_end;
	u32 lin_start;
	u32 lin_end;
	u32 xsize;	/* line size in bytes (register gets xsize - 1) */
	u32 ysize;	/* number of lines (register gets ysize - 1) */
	u32 stride;	/* IMGO_STRIDE, bytes per line */
};

/* mtk_cam_sv_pak_sel(): FOURCC family -> PAK_MODE byte */
static u32 cam_pak_from_fmt(u32 fmt)
{
	if (pak_mode >= 0)
		return (u32)pak_mode & CAMSV_PAK_MODE_MASK;

	/*
	 * fmt is TG1_FMT, which is a BIT-DEPTH code (0/1/2/4 = 8/10/12/14-bit),
	 * not a Bayer order.  mtk_cam_sv_pak_sel() (isp_ko.asm:71714-71812)
	 * returns a packing CLASS instead:
	 *   0x80 -> 8-bit   Bayer      {BA81,GBRG,GRBG,RGGB}
	 *   0x81 -> 10-bit  MTISP      {MBBA,MBGA,MBRA,MBgA}
	 *   0x82 -> 12-bit  MTISP      {MBBC,MBGC,MBRC,MBgC}
	 *   0x8f -> 10-bit  "Enhanced" {pBAA,pGAA,pRAA,pgAA}
	 *   0x00 -> unmapped (incl. every 14-bit M*E fourcc)
	 * IMX582 4000x3000 RAW10 is the 10-bit MTISP family -> 0x81.  The 0x8f
	 * family is reachable only by forcing pak_mode=0x8f.
	 */
	switch (fmt) {
	case 0:			/* 8-bit */
		return 0x80;
	case 1:			/* 10-bit MTISP */
		return 0x81;
	case 2:			/* 12-bit MTISP */
		return 0x82;
	default:		/* 14-bit: the vendor maps nothing */
		return 0x00;
	}
}

/* FMT_SEL (0x044): the vendor writes the bare TG1_FMT code; allow a full override */
static u32 cam_fmt_sel(u32 fmt)
{
	if (fmt_sel)
		return fmt_sel;
	return fmt & CAMSV_FMT_SEL_TG1_FMT_MASK;
}

/* ------------------------------------------------------------------ */
/* Full static configuration - CAMERA_NOTES.md 8.8 steps 1..7          */
/* ------------------------------------------------------------------ */

/*
 * Runs the whole static bring-up and deliberately stops short of raising
 * TG_VF_CON.VFDATA_EN: that is the "go" and belongs to the arm command.
 * Ordering constraints honoured here:
 *   - the 0x110 DB_LOAD_HOLD window wraps the seven geometry/format writes
 *     and is released last (8.6/8.8 step 2);
 *   - every FBC write precedes any VFDATA_EN write (8.8, "must come before
 *     step 8"), which is why VFDATA_EN is never touched in this function.
 */
static void cam_cap_static_config(const struct camcap_cfg *c)
{
	u32 pak = cam_pak_from_fmt(c->fmt);
	u32 v;

	/* ---- 8.8 step 2: mtk_cam_sv_setup_cfg_info() (0x49dbc) ---- */
	/* TG_PATH_CFG.DB_LOAD_HOLD (b11) = 1: hold the shadow registers */
	cam_set(CAMSV_TG_PATH_CFG, CAMSV_TG_PATH_CFG_DB_LOAD_HOLD);

	/* TG_SEN_GRAB_PXL: PXL_END b31:16 | PXL_START b15:0 */
	cam_wr(CAMSV_TG_SEN_GRAB_PXL,
	       ((c->pxl_end & 0xffff) << CAMSV_GRAB_PXL_END_SHIFT) |
	       (c->pxl_start & CAMSV_GRAB_PXL_START_MASK));
	/* TG_SEN_GRAB_LIN: LIN_END b31:16 | LIN_START b15:0 */
	cam_wr(CAMSV_TG_SEN_GRAB_LIN,
	       ((c->lin_end & 0xffff) << CAMSV_GRAB_LIN_END_SHIFT) |
	       (c->lin_start & CAMSV_GRAB_LIN_START_MASK));
	/* FMT_SEL.TG1_FMT b2:0 (plain write, as the vendor does) */
	cam_wr(CAMSV_FMT_SEL, cam_fmt_sel(c->fmt));
	/*
	 * PAK: mtk_cam_sv_cal_cfg_info() stores 0x380/0x381/0x382/0x38F
	 * (i.e. 0x300 | pak_sel) and setup_cfg_info copies it verbatim.
	 */
	cam_wr(CAMSV_PAK, ((pak_dbl & 0x3) << CAMSV_PAK_DBL_SHIFT) |
			  (pak & CAMSV_PAK_MODE_MASK));
	/* IMGO_XSIZE.XSIZE b15:0 = line size in bytes - 1 */
	cam_wr(CAMSV_IMGO_XSIZE, c->xsize - 1);
	/* IMGO_YSIZE.YSIZE b15:0 = number of lines - 1 */
	cam_wr(CAMSV_IMGO_YSIZE, c->ysize - 1);
	/* IMGO_STRIDE.STRIDE b15:0 = bytes per line */
	cam_wr(CAMSV_IMGO_STRIDE, c->stride);

	/* TG_PATH_CFG.DB_LOAD_HOLD (b11) = 0: release, latch atomically */
	cam_clr(CAMSV_TG_PATH_CFG, CAMSV_TG_PATH_CFG_DB_LOAD_HOLD);

	/* ---- 8.8 step 4: mtk_cam_sv_tg_config() (0x4578c) ---- */
	/* TG_SEN_MODE.CMOS_EN (b0) = 0 while (re)configuring the window */
	cam_clr(CAMSV_TG_SEN_MODE, CAMSV_TG_SEN_MODE_CMOS_EN);
	/* sub-frame timing: no down-sample period -> clear VS/SOF sub-enables */
	cam_wr(CAMSV_TG_SUB_PERIOD,
	       cam_rd(CAMSV_TG_SUB_PERIOD) &
	       ~(CAMSV_TG_SUB_PERIOD_VS_PERIOD | CAMSV_TG_SUB_PERIOD_SOF_PERIOD));
	cam_clr(CAMSV_TG_SEN_MODE,
		CAMSV_TG_SEN_MODE_VS_SUB_EN | CAMSV_TG_SEN_MODE_SOF_SUB_EN);
	/* mode index 0 -> not a stagger/HDR mode */
	cam_clr(CAMSV_TG_SEN_MODE, CAMSV_TG_SEN_MODE_STAGGER_EN);
	/* TG_PATH_CFG.SUB_SOF_SRC_SEL b21:20 = 0 */
	cam_clr(CAMSV_TG_PATH_CFG, CAMSV_TG_PATH_CFG_SUB_SOF_SRC_SEL);
	/* TG_SEN_MODE.TIME_STP_EN (b16) = 1, unconditional in the vendor */
	cam_set(CAMSV_TG_SEN_MODE, CAMSV_TG_SEN_MODE_TIME_STP_EN);
	/* TG_VF_CON.SINGLE_MODE (b1) = 0 */
	cam_clr(CAMSV_TG_VF_CON, CAMSV_TG_VF_CON_SINGLE_MODE);
	/* TG_SEN_MODE.DBL_DATA_BUS b5:4 from the format switch */
	cam_wr(CAMSV_TG_SEN_MODE,
	       (cam_rd(CAMSV_TG_SEN_MODE) & ~CAMSV_TG_SEN_MODE_DBL_DATA_BUS) |
	       ((dbl_data_bus & 0x3) << 4));
	/* tg_config writes the window again (RMW-identical to the step above) */
	cam_wr(CAMSV_TG_SEN_GRAB_PXL,
	       ((c->pxl_end & 0xffff) << CAMSV_GRAB_PXL_END_SHIFT) |
	       (c->pxl_start & CAMSV_GRAB_PXL_START_MASK));
	cam_wr(CAMSV_TG_SEN_GRAB_LIN,
	       ((c->lin_end & 0xffff) << CAMSV_GRAB_LIN_END_SHIFT) |
	       (c->lin_start & CAMSV_GRAB_LIN_START_MASK));

	/* ---- 8.8 step 5: mtk_cam_sv_top_config() (0x4699c) ---- */
	/* MODULE_EN.TG_EN (b0) = 1 */
	cam_set(CAMSV_MODULE_EN, CAMSV_MODULE_EN_TG_EN);
	/* MODULE_EN.DB_EN (b30) = 0, then DB_LOAD_SRC b29:28 = 2 */
	cam_clr(CAMSV_MODULE_EN, CAMSV_MODULE_EN_DB_EN);
	cam_wr(CAMSV_MODULE_EN,
	       (cam_rd(CAMSV_MODULE_EN) & ~CAMSV_MODULE_EN_DB_LOAD_SRC) |
	       (2u << 28));
	/* SUB_CTRL.CENTRAL_SUB_EN (b0) = 0: no down-sample period */
	cam_clr(CAMSV_SUB_CTRL, CAMSV_SUB_CTRL_CENTRAL_SUB_EN);
	/* DCIF_SET: MASK_DB_LOAD b7 = 0; the (false) condition also clears b15/b8 */
	cam_clr(CAMSV_DCIF_SET, CAMSV_DCIF_SET_MASK_DB_LOAD);
	cam_clr(CAMSV_DCIF_SET, CAMSV_DCIF_SET_FOR_DCIF_SUB_EN);
	cam_clr(CAMSV_DCIF_SET, CAMSV_DCIF_SET_EN_CQ_START);
	/* MISC.VF_SRC (b0) = 0 */
	cam_clr(CAMSV_MISC, CAMSV_MISC_VF_SRC);
	/* FMT_SEL plain write (fmt code from the FOURCC table) */
	cam_wr(CAMSV_FMT_SEL, cam_fmt_sel(c->fmt));
	/* INT_EN, plain write of 0x00011070 by default */
	cam_wr(CAMSV_INT_EN, int_en);
	/* no down-sample: DOWN_SAMPLE_EN b24 = 0, DOWN_SAMPLE_PERIOD b23:16 = 0 */
	cam_wr(CAMSV_MODULE_EN,
	       (cam_rd(CAMSV_MODULE_EN) & ~(CAMSV_MODULE_EN_DS_EN |
					    CAMSV_MODULE_EN_DS_PERIOD)));
	/* MODULE_EN.PAK_EN (b2) = 1, PAK_SEL (b3) = 0 */
	cam_set(CAMSV_MODULE_EN, CAMSV_MODULE_EN_PAK_EN);
	cam_clr(CAMSV_MODULE_EN, CAMSV_MODULE_EN_PAK_SEL);
	/* PAK_CON.PAK_IN_BIT b20:16 = 14 */
	cam_wr(CAMSV_PAK_CON,
	       (cam_rd(CAMSV_PAK_CON) & ~CAMSV_PAK_CON_PAK_IN_BIT_MASK) |
	       (CAMSV_PAK_CON_PAK_IN_BIT_VAL << 16));
	/* PAK = pak_sel | ((fmt_code & 0x3) << PAK_DBL_MODE) */
	cam_wr(CAMSV_PAK,
	       (pak & CAMSV_PAK_MODE_MASK) |
	       ((pak_dbl & 0x3) << CAMSV_PAK_DBL_SHIFT));
	/* MODULE_EN.QBN_EN (b7) = 0 */
	cam_clr(CAMSV_MODULE_EN, CAMSV_MODULE_EN_QBN_EN);
	/* SPECIAL_FUN_EN.DCM_MODE (b26) = 1 */
	cam_wr(CAMSV_SPECIAL_FUN_EN, CAMSV_SPECIAL_FUN_EN_DCM_MODE);

	/* ---- 8.8 step 6: dmao_config (0x47ab4) + fbc_config (0x47f60) ---- */
	cam_wr(CAMSV_IMGO_XSIZE, c->xsize - 1);
	cam_wr(CAMSV_IMGO_YSIZE, c->ysize - 1);
	/* the vendor overrides a stride smaller than the line size with xsize */
	cam_wr(CAMSV_IMGO_STRIDE, c->stride < c->xsize ? c->xsize : c->stride);
	/* IMGO_CROP = 0 (no crop offset) */
	cam_wr(CAMSV_IMGO_CROP, 0);
	/* IMGO_CON0..4 FIFO/burst thresholds; two literal sets exist */
	if (con_set == 0) {
		cam_wr(CAMSV_IMGO_CON0, 0x10000300);
		cam_wr(CAMSV_IMGO_CON1, 0x00c00060);
		cam_wr(CAMSV_IMGO_CON2, 0x01800120);
		cam_wr(CAMSV_IMGO_CON3, 0x820001a0);
		cam_wr(CAMSV_IMGO_CON4, 0x812000c0);
	} else {
		cam_wr(CAMSV_IMGO_CON0, 0x10000080);
		cam_wr(CAMSV_IMGO_CON1, 0x00200010);
		cam_wr(CAMSV_IMGO_CON2, 0x00400030);
		cam_wr(CAMSV_IMGO_CON3, 0x80550045);
		cam_wr(CAMSV_IMGO_CON4, 0x80300020);
	}
	/* fbc_config: FBC_IMGO_CTL1 = 0 (full reset before enabling) */
	cam_wr(CAMSV_FBC_IMGO_CTL1, 0);

	/* ---- 8.8 step 7: tg_enable + top_enable, minus the VF raise ---- */
	/* mtk_cam_sv_tg_enable(): TG_SEN_MODE.CMOS_EN (b0) = 1 */
	cam_set(CAMSV_TG_SEN_MODE, CAMSV_TG_SEN_MODE_CMOS_EN);
	/* mtk_cam_sv_top_enable() step 1: CLK_EN += TG/QBN/PAK/IMGO DP clocks */
	cam_set(CAMSV_CLK_EN, CAMSV_CLK_EN_TG_DP_CK_EN |
			      CAMSV_CLK_EN_QBN_DP_CK_EN |
			      CAMSV_CLK_EN_PAK_DP_CK_EN |
			      CAMSV_CLK_EN_IMGO_DP_CK_EN);
	/* mtk_cam_sv_dmao_enable(): MODULE_EN.IMGO_EN (b4) = 1 */
	cam_set(CAMSV_MODULE_EN, CAMSV_MODULE_EN_IMGO_EN);

	/*
	 * mtk_cam_sv_fbc_enable() (0x48b8c) aborts with -1 while
	 * TG_VF_CON.VFDATA_EN is set, so it must run while the VF is stopped -
	 * which is guaranteed here because cam_cap_cfg() clears VFDATA_EN
	 * before calling this function and arm() raises it afterwards.
	 */
	v = cam_rd(CAMSV_FBC_IMGO_CTL1);
	if (fbc_en) {
		/* SUB_RATIO b31:24, then FBC_EN b15 = 1, then FBC_DB_EN b9 = 0 */
		cam_wr(CAMSV_FBC_IMGO_CTL1,
		       (v & ~CAMSV_FBC_IMGO_CTL1_SUB_RATIO) |
		       ((sub_ratio & 0xff) << CAMSV_FBC_IMGO_CTL1_SUB_RATIO_SHIFT));
		cam_set(CAMSV_FBC_IMGO_CTL1, CAMSV_FBC_IMGO_CTL1_FBC_EN);
		cam_clr(CAMSV_FBC_IMGO_CTL1, CAMSV_FBC_IMGO_CTL1_FBC_DB_EN);
	} else {
		/* uncompressed: FBC stays off, only the DB path is enabled */
		cam_wr(CAMSV_FBC_IMGO_CTL1,
		       v & ~CAMSV_FBC_IMGO_CTL1_FBC_EN);
	}
	/* top_enable step 2: FBC_IMGO_CTL1.FBC_DB_EN (b9) = 1 */
	cam_set(CAMSV_FBC_IMGO_CTL1, CAMSV_FBC_IMGO_CTL1_FBC_DB_EN);
}

/* ------------------------------------------------------------------ */
/* SENINF routing                                                     */
/* ------------------------------------------------------------------ */

/*
 * Carry CSI port 2 into the CAMSV instance this module drives.
 *
 *   0x1a010010 byte1  mux 1 listens to intf 4    (SENINF_5 == port 2)
 *   0x1a011d00        mux 1 takes SRC_SEL 8      (MIPI_SENSOR + group 0)
 *   0x1a010460        cam_mux 3 SRC_SEL 1, EN 1  -> CAMSV1 @ 0x1a110000
 *
 * The cam_mux -> CAMSV binding is fixed wiring, declared in the vendor DT as
 * mediatek,cammux-id = <3> on camsv1@1a110000; no register selects it.  Order
 * and values follow the vendor's static config_hw path
 * (isp71_ref/mtk_cam-seninf-drv.c:894-1000), see seninf_routing.md for the
 * per-line evidence.  The D-PHY/CSI2 half (0x1a014a00 / 0x1a014200) is already
 * up from port2_rx71.py and is only verified here, never rewritten.
 */
static int cam_route(void)
{
	u32 muxb = SENINF_MUX_BASE(route_mux);
	u32 pcsr = 0x0400u + 0x0020u * cammux;
	u32 csi2 = SENINF_CSI2_BASE(route_intf);
	u32 ctrl = SENINF_CTRL_BASE(route_intf);
	u32 topword, shift, r, tmp;
	int err = 0;

	if (!cam_seninf)
		return -ENODEV;

	/* 0. verify the D-PHY / CSI2 context that is already streaming */
	r = sen_rd(csi2 + 0x00);
	if ((r & 0x3f) != 0x0f)
		pr_warn("route: CSI2_EN=%#x, expected 0xF for 4 lanes\n", r & 0x3f);
	r = sen_rd(csi2 + 0x20);
	pr_info("route: S0_DI_CTRL was %#010x before step 0b\n", r);
	r = sen_rd(csi2 + 0x60);
	if (!(r & 0x100)) {
		sen_wr(csi2 + 0x60, r | 0x100);	/* CH0_CTRL.S0_GRP_EN */
		pr_info("route: CH0_CTRL=%#010x lacked S0 group enable, set it\n",
			r);
	}
	/*
	 * 0b. CSI2 DI/VC filter slot 0.  The device read 0x00000000 here before
	 * this step was added: SENINF_CSI2_S0_DI_CTRL (csi2 + 0x20) is written by
	 * the sensor driver at stream-on, and the userspace D-PHY scripts
	 * (port2_rx71.py / port2_v17.py) only ever set CSI2_EN and
	 * RESYNC_MERGE_CTRL.  Without it group 0 has no VC/DT assignment, so the
	 * SENINF mux (SRC_SEL = MIPI_SENSOR + group 0) receives nothing even
	 * though the packet counter runs.  0x002b0011 for VC0 / DT 0x2b:
	 *   b0     VC_INTERLEAVE_EN = 1
	 *   b4     DT_INTERLEAVE_MODE = 1
	 *   b8-12  VC_SEL = route_vc
	 *   b16-21 DT_SEL = route_dt
	 */
	sen_wr(csi2 + 0x20, BIT(0) | BIT(4) |
	       ((route_vc & 0x1fu) << 8) | ((route_dt & 0x3fu) << 16));
	r = sen_rd(csi2 + 0x20);
	if (r != (BIT(0) | BIT(4) | ((route_vc & 0x1fu) << 8) |
		  ((route_dt & 0x3fu) << 16)))
		pr_warn("route: S0_DI_CTRL=%#010x did not stick\n", r);
	else
		pr_info("route: S0_DI_CTRL=%#010x (VC%u / DT %#x)\n", r,
			route_vc, route_dt);
	r = sen_rd(SENINF_TOP_PHY_CTRL_CSI2);
	if (!(r & 0x1)) {
		sen_wr(SENINF_TOP_PHY_CTRL_CSI2, r | 0x1);	/* DPHY_EN */
		pr_info("route: TOP_PHY_CTRL_CSI2 %#010x had DPHY_EN clear, set it\n", r);
	}
	r = sen_rd(ctrl + 0x10);
	if (!(r & 0x1))
		pr_warn("route: SENINF_CSI2_CTRL=%#010x (b0 expected 1)\n", r);
	r = sen_rd(ctrl + 0x00);
	if (!(r & 0x1))
		pr_warn("route: SENINF_CTRL=%#010x (SENINF_EN b0 expected 1)\n", r);

	/* 1. cam_mux block into a known state (defined double-buffer page) */
	r = sen_rd(SENINF_GCSR_CTRL);
	sen_wr(SENINF_GCSR_CTRL, r | SENINF_GCSR_SW_RST);
	udelay(1);
	sen_wr(SENINF_GCSR_CTRL, sen_rd(SENINF_GCSR_CTRL) & ~SENINF_GCSR_SW_RST);
	udelay(1);

	/* 2. TOP mux array: byte (mux % 4) of word (mux / 4) = our intf */
	topword = SENINF_TOP_MUX_CTRL(route_mux / 4);
	shift = 8u * (route_mux % 4);
	r = sen_rd(topword);
	sen_wr(topword, (r & ~(0x1fu << shift)) |
	       ((route_intf & 0x1fu) << shift));

	/* 3. the SENINF mux: source group, pixel mode, polarity, reset, enable */
	r = sen_rd(muxb + SENINF_MUX_CTRL_1);
	sen_wr(muxb + SENINF_MUX_CTRL_1,
	       (r & ~(SENINF_MUX_SRC_SEL_MASK | SENINF_MUX_PIX_MODE_MASK)) |
	       (SENINF_SRC_MIPI_SENSOR & SENINF_MUX_SRC_SEL_MASK) |
	       ((route_pix_mode & 0x3u) << 8));
	r = sen_rd(muxb + SENINF_MUX_OPT);
	sen_wr(muxb + SENINF_MUX_OPT, r & ~SENINF_MUX_POL_MASK);
	tmp = sen_rd(muxb + SENINF_MUX_CTRL_0);
	sen_wr(muxb + SENINF_MUX_CTRL_0, tmp | SENINF_MUX_SW_RST);
	sen_wr(muxb + SENINF_MUX_CTRL_0, tmp & ~SENINF_MUX_SW_RST);
	sen_wr(muxb + SENINF_MUX_CTRL_0,
	       sen_rd(muxb + SENINF_MUX_CTRL_0) | SENINF_MUX_EN);

	/* 4. the CAM_MUX: VC/DT filter, source = our mux, size check, enable */
	sen_wr(pcsr + SENINF_PCSR_OPT,
	       (route_vc & 0x1fu) | BIT(7) |
	       ((route_dt & 0x3fu) << 8) | BIT(15));
	r = sen_rd(pcsr + SENINF_PCSR_CTRL);
	sen_wr(pcsr + SENINF_PCSR_CTRL,
	       (r & ~SENINF_PCSR_SRC_SEL_MASK) |
	       (route_mux & SENINF_PCSR_SRC_SEL_MASK));
	sen_wr(pcsr + SENINF_PCSR_CHK_CTL,
	       ((exp_vsize & 0xffffu) << 16) | (exp_hsize & 0xffffu));
	r = sen_rd(pcsr + SENINF_PCSR_CTRL);
	sen_wr(pcsr + SENINF_PCSR_CTRL,
	       (r & ~SENINF_PCSR_CHK_PIX_MODE_MASK) |
	       ((route_pix_mode & 0x3u) << 8));
	sen_wr(pcsr + SENINF_PCSR_IRQ_STATUS, SENINF_PCSR_IRQ_CLR);
	r = sen_rd(pcsr + SENINF_PCSR_CTRL);
	sen_wr(pcsr + SENINF_PCSR_CTRL, r | SENINF_PCSR_EN);

	/* 5. confirm */
	r = sen_rd(pcsr + SENINF_PCSR_CTRL);
	if (!(r & SENINF_PCSR_EN))
		err = -EIO;
	pr_info("route: cam_mux %u CTRL=%#010x OPT=%#010x CHK_CTL=%#010x CHK_RES=%#010x%s\n",
		cammux, r, sen_rd(pcsr + SENINF_PCSR_OPT),
		sen_rd(pcsr + SENINF_PCSR_CHK_CTL),
		sen_rd(pcsr + SENINF_PCSR_CHK_RES),
		err ? "  <-- EN did not stick, retry with DYN_PAGE_SEL=1" : "");
	pr_info("route: mux %u CTRL_0=%#010x CTRL_1=%#010x OPT=%#010x TOP_MUX_CTRL_%u=%#010x\n",
		route_mux, sen_rd(muxb + SENINF_MUX_CTRL_0),
		sen_rd(muxb + SENINF_MUX_CTRL_1),
		sen_rd(muxb + SENINF_MUX_OPT),
		route_mux / 4, sen_rd(topword));

	return err;
}

/* ------------------------------------------------------------------ */
/* stop / arm                                                         */
/* ------------------------------------------------------------------ */

/* mtk_cam_sv_vf_on(dev, 0): drop VFDATA_EN (b0).  Idempotent. */
static void cam_cap_vf_off(void)
{
	if (cam_rd(CAMSV_TG_VF_CON) & CAMSV_TG_VF_CON_VFDATA_EN)
		cam_clr(CAMSV_TG_VF_CON, CAMSV_TG_VF_CON_VFDATA_EN);
	cam_vf_on = false;
}

/*
 * arm: mtk_cam_sv_enquehwbuf() (0x49790) followed by mtk_cam_sv_vf_on(dev, 1)
 * - the two halves of "start one frame" in 8.8 steps 8 and 9.
 *
 * Completion detection is a bounded poll of INT_STATUS (0x04c) and
 * FBC_IMGO_CTL2 (0x244) using msleep(), never a busy-wait with interrupts
 * disabled.  When the poll finishes (or times out) the VF is stopped again.
 */
/*
 * Arm one frame into `addr` and wait for it.  `addr` is an IOVA when the
 * IOMMU is in use; the pipeline passes the address of the slot it wants the
 * frame in, everything else uses the module's single buffer.
 */
static int cam_cap_arm_addr(dma_addr_t addr)
{
	phys_addr_t pa = (phys_addr_t)addr;
	unsigned int i = 0;
	s64 t_start = ktime_get_ns();
	s64 elapsed;
	u32 seq = cam_arm_count + 1;
	bool done = false;

	cam_frame_ready = false;
	cam_last_int_status = 0;
	cam_last_fbc_ctl2 = 0;
	cam_last_seq = seq;

	/* Make sure nothing is streaming into the buffer before we re-point it */
	cam_cap_vf_off();

	/* ---- SENINF routing: port 2 -> mux 1 -> cam_mux 3 -> this CAMSV ---- */
	if (route_en && !(route_once && cam_route_done)) {
		int rerr = cam_route();

		if (rerr)
			pr_warn("arm: SENINF routing returned %d, capturing anyway\n",
				rerr);
		else
			cam_route_done = true;
	}

	/* ---- mtk_cam_sv_enquehwbuf(dev, addr, seq) ---- */
	/* FRAME_SEQ_NO = frame sequence number */
	cam_wr(CAMSV_FRAME_SEQ_NO, seq);
	/* IMGO_BASE_ADDR = destination address bits [31:0] */
	cam_wr(CAMSV_IMGO_BASE_ADDR, lower_32_bits(addr));
	/* IMGO_BASE_ADDR_MSB = address bits [35:32] (upper nibble) */
	cam_wr(CAMSV_IMGO_BASE_ADDR_MSB, upper_32_bits(addr) & 0xf);
	/* make the address visible before the kick */
	wmb();
	/* TOP_FBC_CNT_SET.RCNT_INC1 (b0) = 1 : kick the frame counter ring */
	cam_wr(CAMSV_TOP_FBC_CNT_SET, CAMSV_TOP_FBC_CNT_SET_RCNT_INC1);

	/* ---- mtk_cam_sv_vf_on(dev, 1): the final "go" ---- */
	if (single_mode)
		cam_set(CAMSV_TG_VF_CON, CAMSV_TG_VF_CON_VFDATA_EN |
					 CAMSV_TG_VF_CON_SINGLE_MODE);
	else
		cam_set(CAMSV_TG_VF_CON, CAMSV_TG_VF_CON_VFDATA_EN);
	cam_vf_on = true;

	/* ---- bounded completion poll ---- */
	for (;;) {
		cam_last_int_status = cam_rd(CAMSV_INT_STATUS);
		cam_last_fbc_ctl2 = cam_rd(CAMSV_FBC_IMGO_CTL2);

		if (cam_probe_trace)
			pr_info("probe %3u t=%6lldus INT_STATUS=%#010x CTL2=%#010x (RCNT=%u WCNT=%u FBC=%u DROP=%u)\n",
				i,
				div_s64(ktime_get_ns() - t_start, 1000),
				cam_last_int_status, cam_last_fbc_ctl2,
				(u32)FIELD_GET(CAMSV_FBC_IMGO_CTL2_IMGO_RCNT, cam_last_fbc_ctl2),
				(u32)FIELD_GET(CAMSV_FBC_IMGO_CTL2_IMGO_WCNT, cam_last_fbc_ctl2),
				(u32)FIELD_GET(CAMSV_FBC_IMGO_CTL2_IMGO_FBC_CNT, cam_last_fbc_ctl2),
				(u32)FIELD_GET(CAMSV_FBC_IMGO_CTL2_IMGO_DROP_CNT, cam_last_fbc_ctl2));

		if (cam_last_int_status & (CAMSV_INT_IMGO_DONE_ST |
					   CAMSV_INT_PASS1_DON_ST |
					   CAMSV_INT_SW_PASS1_DON_ST)) {
			done = true;
			break;
		}

		elapsed = ktime_get_ns() - t_start;
		if (elapsed >= (s64)CAMCAP_ARM_TIMEOUT_MS * 1000000LL)
			break;

		/*
		 * Poll finely for the whole frame, and never with msleep():
		 * msleep() rounds up to a jiffy, so a frame that finishes
		 * between two ticks is noticed a whole tick late.  For the
		 * 30 fps modes that costs an extra frame period -- measured as
		 * arm 40 ms against a 32.6 ms sensor period, i.e. 25 fps
		 * instead of 30 -- and for the short modes (1080p120/240,
		 * 4K60) it is fatal: their whole frame fits inside a coarse
		 * window, so the tick, not the sensor, becomes the frame
		 * period and the mode reads as a hard "12 ms floor that no
		 * VTS write moves".  usleep_range() returns in tens of
		 * microseconds without busy-waiting, and the extra MMIO reads
		 * cost ~1 us per poll.
		 */
		if (cam_probe_trace)
			usleep_range(500, 700);
		else
			usleep_range(150, 250);

		i++;
	}

	/* re-read so the recorded values are the final ones */
	cam_last_int_status = cam_rd(CAMSV_INT_STATUS);
	cam_last_fbc_ctl2 = cam_rd(CAMSV_FBC_IMGO_CTL2);

	cam_cap_vf_off();
	cam_frame_ready = done;
	cam_arm_count++;
	cam_last_ret = done ? 0 : -ETIMEDOUT;

	if (cam_probe_trace)
		pr_info("probe: %s after %lldus (%u polls)\n",
			done ? "FRAME READY" : "TIMEOUT",
			div_s64(ktime_get_ns() - t_start, 1000), i);

	if (arm_trace) {
		pr_info("arm seq=%u addr=%pa xsize=%u ysize=%u -> %s\n",
			seq, &pa, cam_rd(CAMSV_IMGO_XSIZE),
			cam_rd(CAMSV_IMGO_YSIZE),
			done ? "FRAME READY" : "TIMEOUT");
		pr_info("arm INT_STATUS=%#010x FBC_IMGO_CTL2=%#010x (RCNT=%u WCNT=%u FBC_CNT=%u DROP=%u)\n",
			cam_last_int_status, cam_last_fbc_ctl2,
			(u32)FIELD_GET(CAMSV_FBC_IMGO_CTL2_IMGO_RCNT, cam_last_fbc_ctl2),
			(u32)FIELD_GET(CAMSV_FBC_IMGO_CTL2_IMGO_WCNT, cam_last_fbc_ctl2),
			(u32)FIELD_GET(CAMSV_FBC_IMGO_CTL2_IMGO_FBC_CNT, cam_last_fbc_ctl2),
			(u32)FIELD_GET(CAMSV_FBC_IMGO_CTL2_IMGO_DROP_CNT, cam_last_fbc_ctl2));
	} else if (!done) {
		pr_warn("arm seq=%u timed out (INT_STATUS=%#010x)\n",
			seq, cam_last_int_status);
	}

	if (!done)
		return -ETIMEDOUT;
	return 0;
}

/* The single-buffer case: everything that is not the pipeline. */
static int cam_cap_arm(void)
{
	return cam_cap_arm_addr(cam_frame_pa);
}

/* ------------------------------------------------------------------ */
/* procfs - /proc/camcap                                              */
/* ------------------------------------------------------------------ */

/* Raw frame bytes, read through the uncached alias. */
static ssize_t camcap_read(struct file *file, char __user *ubuf,
			   size_t count, loff_t *ppos)
{
	loff_t pos = *ppos;
	size_t todo, done = 0;

	if (!cam_alias || !cam_bounce)
		return -ENODEV;
	if (pos < 0)
		return -EINVAL;
	if (pos >= (loff_t)cam_buf_size)
		return 0;

	todo = min_t(size_t, count, (size_t)cam_buf_size - (size_t)pos);

	while (done < todo) {
		size_t chunk = min_t(size_t, todo - done, CAMCAP_BOUNCE_SIZE);

		memcpy_fromio(cam_bounce,
			      (u8 __iomem *)cam_alias + pos + done, chunk);
		if (copy_to_user(ubuf + done, cam_bounce, chunk))
			return done ? (ssize_t)done : -EFAULT;
		done += chunk;
	}

	*ppos = pos + done;
	return done;
}

/* cfg <fmt> <pxl_start> <pxl_end> <lin_start> <lin_end> <xsize> <ysize> <stride> */
static int cam_cap_cmd_cfg(const char *line)
{
	struct camcap_cfg c;
	int n;

	n = sscanf(line, "cfg %u %u %u %u %u %u %u %u",
		   &c.fmt, &c.pxl_start, &c.pxl_end,
		   &c.lin_start, &c.lin_end,
		   &c.xsize, &c.ysize, &c.stride);
	if (n != 8) {
		pr_err("cfg: need 8 arguments: cfg <fmt> <pxl_start> <pxl_end> <lin_start> <lin_end> <xsize> <ysize> <stride>\n");
		return -EINVAL;
	}
	if (!c.xsize || !c.ysize || c.xsize > 0x10000 || c.ysize > 0x10000) {
		pr_err("cfg: xsize/ysize out of range (1..65536)\n");
		return -EINVAL;
	}
	if (c.pxl_start > 0xffff || c.pxl_end > 0xffff ||
	    c.lin_start > 0xffff || c.lin_end > 0xffff) {
		pr_err("cfg: pxl/lin window values must fit 16 bits\n");
		return -EINVAL;
	}
	if (c.stride > 0xffff) {
		pr_err("cfg: stride must fit 16 bits\n");
		return -EINVAL;
	}
	if (c.fmt != 0 && c.fmt != 1 && c.fmt != 2 && c.fmt != 4) {
		pr_warn("cfg: fmt %u is not one of the known TG1_FMT codes (0/1/2/4)\n",
			c.fmt);
	}

	/* 8.8 says the static config runs with the VF stopped */
	cam_cap_vf_off();

	pr_info("cfg: fmt=%u fmt_sel=%#x pak=%#x pak_dbl=%u pxl=[%u..%u] lin=[%u..%u] xsize=%u->reg %u ysize=%u->reg %u stride=%u->reg %u dbl_data_bus=%u fbc_en=%u\n",
		c.fmt, cam_fmt_sel(c.fmt), cam_pak_from_fmt(c.fmt), pak_dbl,
		c.pxl_start, c.pxl_end, c.lin_start, c.lin_end,
		c.xsize, c.xsize - 1, c.ysize, c.ysize - 1,
		c.stride, c.stride < c.xsize ? c.xsize : c.stride,
		dbl_data_bus, fbc_en);

	cam_cap_static_config(&c);

	cam_frame_ready = false;
	cam_last_int_status = 0;
	cam_last_fbc_ctl2 = 0;
	return 0;
}

/* stop: TG_VF_CON.VFDATA_EN = 0 */
static int cam_cap_cmd_stop(void)
{
	cam_cap_vf_off();
	pr_info("stop: TG_VF_CON=%#010x (VFDATA_EN=%u)\n",
		cam_rd(CAMSV_TG_VF_CON),
		!!(cam_rd(CAMSV_TG_VF_CON) & CAMSV_TG_VF_CON_VFDATA_EN));
	return 0;
}

/* regs: printk every register this module touches, with its current value */
static void cam_cap_dump_regs(void)
{
	u32 ctl1 = cam_rd(CAMSV_FBC_IMGO_CTL1);
	u32 ctl2 = cam_rd(CAMSV_FBC_IMGO_CTL2);
	u32 vf = cam_rd(CAMSV_TG_VF_CON);

	pr_info("== CAMSV regs @ %#lx ==\n", camsv_base);
	pr_info("0x014 TOP_FBC_CNT_SET    = %#010x\n", cam_rd(CAMSV_TOP_FBC_CNT_SET));
	pr_info("0x040 MODULE_EN          = %#010x (TG_EN=%u PAK_EN=%u IMGO_EN=%u DB_LOAD_SRC=%u DB_EN=%u)\n",
		cam_rd(CAMSV_MODULE_EN),
		!!(cam_rd(CAMSV_MODULE_EN) & CAMSV_MODULE_EN_TG_EN),
		!!(cam_rd(CAMSV_MODULE_EN) & CAMSV_MODULE_EN_PAK_EN),
		!!(cam_rd(CAMSV_MODULE_EN) & CAMSV_MODULE_EN_IMGO_EN),
		(u32)FIELD_GET(CAMSV_MODULE_EN_DB_LOAD_SRC, cam_rd(CAMSV_MODULE_EN)),
		!!(cam_rd(CAMSV_MODULE_EN) & CAMSV_MODULE_EN_DB_EN));
	pr_info("0x044 FMT_SEL            = %#010x (TG1_FMT=%u)\n",
		cam_rd(CAMSV_FMT_SEL),
		(u32)FIELD_GET(CAMSV_FMT_SEL_TG1_FMT_MASK, cam_rd(CAMSV_FMT_SEL)));
	pr_info("0x048 INT_EN             = %#010x\n", cam_rd(CAMSV_INT_EN));
	pr_info("0x04c INT_STATUS         = %#010x\n", cam_rd(CAMSV_INT_STATUS));
	pr_info("0x060 CLK_EN             = %#010x\n", cam_rd(CAMSV_CLK_EN));
	pr_info("0x074 DCIF_SET           = %#010x\n", cam_rd(CAMSV_DCIF_SET));
	pr_info("0x078 SUB_CTRL           = %#010x\n", cam_rd(CAMSV_SUB_CTRL));
	pr_info("0x07c PAK                = %#010x\n", cam_rd(CAMSV_PAK));
	pr_info("0x088 MISC               = %#010x\n", cam_rd(CAMSV_MISC));
	pr_info("0x100 TG_SEN_MODE        = %#010x (CMOS_EN=%u DBL_DATA_BUS=%u)\n",
		cam_rd(CAMSV_TG_SEN_MODE),
		!!(cam_rd(CAMSV_TG_SEN_MODE) & CAMSV_TG_SEN_MODE_CMOS_EN),
		(u32)FIELD_GET(CAMSV_TG_SEN_MODE_DBL_DATA_BUS,
			       cam_rd(CAMSV_TG_SEN_MODE)));
	pr_info("0x104 TG_VF_CON          = %#010x (VFDATA_EN=%u SINGLE_MODE=%u)\n",
		vf, !!(vf & CAMSV_TG_VF_CON_VFDATA_EN),
		!!(vf & CAMSV_TG_VF_CON_SINGLE_MODE));
	pr_info("0x108 TG_SEN_GRAB_PXL    = %#010x (START=%u END=%u)\n",
		cam_rd(CAMSV_TG_SEN_GRAB_PXL),
		cam_rd(CAMSV_TG_SEN_GRAB_PXL) & 0xffff,
		cam_rd(CAMSV_TG_SEN_GRAB_PXL) >> 16);
	pr_info("0x10c TG_SEN_GRAB_LIN    = %#010x (START=%u END=%u)\n",
		cam_rd(CAMSV_TG_SEN_GRAB_LIN),
		cam_rd(CAMSV_TG_SEN_GRAB_LIN) & 0xffff,
		cam_rd(CAMSV_TG_SEN_GRAB_LIN) >> 16);
	pr_info("0x110 TG_PATH_CFG        = %#010x (DB_LOAD_HOLD=%u)\n",
		cam_rd(CAMSV_TG_PATH_CFG),
		!!(cam_rd(CAMSV_TG_PATH_CFG) & CAMSV_TG_PATH_CFG_DB_LOAD_HOLD));
	pr_info("0x164 TG_SUB_PERIOD      = %#010x\n", cam_rd(CAMSV_TG_SUB_PERIOD));
	pr_info("0x1c0 PAK_CON            = %#010x (PAK_IN_BIT=%u)\n",
		cam_rd(CAMSV_PAK_CON),
		(u32)FIELD_GET(CAMSV_PAK_CON_PAK_IN_BIT_MASK, cam_rd(CAMSV_PAK_CON)));
	pr_info("0x240 FBC_IMGO_CTL1      = %#010x (FBC_EN=%u FBC_DB_EN=%u SUB_RATIO=%u)\n",
		ctl1, !!(ctl1 & CAMSV_FBC_IMGO_CTL1_FBC_EN),
		!!(ctl1 & CAMSV_FBC_IMGO_CTL1_FBC_DB_EN),
		(u32)FIELD_GET(CAMSV_FBC_IMGO_CTL1_SUB_RATIO, ctl1));
	pr_info("0x244 FBC_IMGO_CTL2      = %#010x (RCNT=%u WCNT=%u FBC_CNT=%u DROP=%u)\n",
		ctl2,
		(u32)FIELD_GET(CAMSV_FBC_IMGO_CTL2_IMGO_RCNT, ctl2),
		(u32)FIELD_GET(CAMSV_FBC_IMGO_CTL2_IMGO_WCNT, ctl2),
		(u32)FIELD_GET(CAMSV_FBC_IMGO_CTL2_IMGO_FBC_CNT, ctl2),
		(u32)FIELD_GET(CAMSV_FBC_IMGO_CTL2_IMGO_DROP_CNT, ctl2));
	pr_info("0x600 SPECIAL_FUN_EN     = %#010x\n", cam_rd(CAMSV_SPECIAL_FUN_EN));
	pr_info("0x700 IMGO_BASE_ADDR     = %#010x\n", cam_rd(CAMSV_IMGO_BASE_ADDR));
	pr_info("0x704 IMGO_BASE_ADDR_MSB = %#010x\n", cam_rd(CAMSV_IMGO_BASE_ADDR_MSB));
	pr_info("0x710 IMGO_XSIZE         = %#010x\n", cam_rd(CAMSV_IMGO_XSIZE));
	pr_info("0x714 IMGO_YSIZE         = %#010x\n", cam_rd(CAMSV_IMGO_YSIZE));
	pr_info("0x718 IMGO_STRIDE        = %#010x\n", cam_rd(CAMSV_IMGO_STRIDE));
	pr_info("0x720..0x730 IMGO_CON0..4= %#010x %#010x %#010x %#010x %#010x\n",
		cam_rd(CAMSV_IMGO_CON0), cam_rd(CAMSV_IMGO_CON1),
		cam_rd(CAMSV_IMGO_CON2), cam_rd(CAMSV_IMGO_CON3),
		cam_rd(CAMSV_IMGO_CON4));
	pr_info("0x74c IMGO_CROP          = %#010x\n", cam_rd(CAMSV_IMGO_CROP));
	pr_info("0x75c FRAME_SEQ_NO       = %#010x\n", cam_rd(CAMSV_FRAME_SEQ_NO));
	pr_info("buf imgo=%pa phys=%pa size=%lu alias=%s\n", &cam_frame_pa,
		&cam_buf_phys, cam_buf_size,
		cam_alias_is_memremap ? "memremap(WC)" : "ioremap_wc");
}

/*
 * burst [n] - arm n frames back to back, no conversion and no statistics.
 *
 * This measures the ceiling the sensor/CAMSV pair imposes on us: the arm poll
 * returns as soon as one frame's DMA has completed, so consecutive arms say how
 * fast full frames can be delivered at all.  Every frame-rate budget has to
 * start from this number - if it is below the target, no amount of converter
 * tuning will help, and the answer has to come from overlapping the converter
 * with the DMA (or from a cheaper sensor mode).
 */
static int cam_cap_cmd_burst(const char *line)
{
	unsigned int n = 8, i, ok = 0;
	s64 sum = 0, mn = 0, mx = 0, t0, t1, d;
	u64 fps100;

	if (sscanf(line, "burst %u", &n) != 1 || n < 1 || n > 64)
		n = 8;

	/*
	 * The sensor stretches its frame period to fit a longer exposure, so a
	 * ceiling measured with a dark-room exposure says nothing about the
	 * mode: exp 0x3000 (the old AE ceiling) turns the 30 fps mode into a
	 * 9 fps one that looks exactly like a regression.  Force a short
	 * exposure so this number is the mode's limit and not the governor's.
	 */
	cam_sensor.exposure = 0x0200;
	cam_sensor_apply();

	for (i = 0; i < n; i++) {
		t0 = ktime_get_ns();
		cam_cap_arm();
		t1 = ktime_get_ns();

		d = t1 - t0;
		if (cam_frame_ready)
			ok++;
		sum += d;
		if (!i || d < mn)
			mn = d;
		if (!i || d > mx)
			mx = d;
	}

	fps100 = sum > 0 ? div_u64((u64)n * 100000000000ULL, (u64)sum) : 0;
	pr_info("burst: n=%u ok=%u arm avg=%lldus min=%lldus max=%lldus -> %llu.%02llu fps ceiling\n",
		n, ok, div_s64(sum, n), mn, mx, fps100 / 100, fps100 % 100);

	return ok == n ? 0 : -ETIMEDOUT;
}

/*
 * probe - arm one frame with the per-poll register trace on.  It answers
 * whether the DMA progress is observable (FBC_IMGO_CTL2) and when the sensor's
 * frame start (INT_VS_ST) lands relative to the arm.
 */
static int cam_cap_cmd_probe(void)
{
	int ret;

	pr_info("probe: ---- begin, INT_STATUS=%#010x CTL2=%#010x\n",
		cam_rd(CAMSV_INT_STATUS), cam_rd(CAMSV_FBC_IMGO_CTL2));
	cam_probe_trace = true;
	ret = cam_cap_arm();
	cam_probe_trace = false;
	pr_info("probe: ---- end ret=%d frame_ready=%d\n", ret, cam_frame_ready);

	return ret;
}

static ssize_t camcap_write(struct file *file, const char __user *ubuf,
			    size_t count, loff_t *ppos)
{
	char *line;
	size_t n;
	int ret;

	if (!cam_base)
		return -ENODEV;
	if (count == 0 || count > 256)
		return -EINVAL;

	line = kmalloc(count + 1, GFP_KERNEL);
	if (!line)
		return -ENOMEM;

	n = count;
	if (copy_from_user(line, ubuf, n)) {
		kfree(line);
		return -EFAULT;
	}
	line[n] = '\0';
	/* make the parser tolerant of trailing newlines/spaces */
	strim(line);

	if (mutex_lock_interruptible(&cam_lock)) {
		kfree(line);
		return -ERESTARTSYS;
	}

	if (str_has_prefix(line, "cfg") && (line[3] == '\0' || line[3] == ' ')) {
		ret = cam_cap_cmd_cfg(line);
	} else if (!strcmp(line, "arm")) {
		ret = cam_cap_arm();
	} else if (str_has_prefix(line, "burst") &&
		   (line[5] == '\0' || line[5] == ' ')) {
		ret = cam_cap_cmd_burst(line);
	} else if (!strcmp(line, "probe")) {
		ret = cam_cap_cmd_probe();
	} else if (!strcmp(line, "route")) {
		ret = cam_route();
		if (!ret)
			cam_route_done = true;	/* satisfied route_once */
	} else if (!strcmp(line, "reroute")) {
		/*
		 * Force the route back into the hardware without reloading the
		 * module: the SENINF mux and cam_mux survive a sensor power
		 * cycle, but this is the cheap way to re-assert them (and to
		 * satisfy route_once before the next arm).
		 */
		cam_route_done = false;
		ret = cam_route();
		if (!ret)
			cam_route_done = true;
	} else if (!strcmp(line, "stop")) {
		ret = cam_cap_cmd_stop();
	} else if (!strcmp(line, "regs")) {
		cam_cap_dump_regs();
		ret = 0;
	} else if (str_has_prefix(line, "af") &&
		   (line[2] == '\0' || line[2] == ' ')) {
		ret = cam_af_cmd(line);
	} else if (str_has_prefix(line, "mode") &&
		   (line[4] == '\0' || line[4] == ' ')) {
		/*
		 * "mode <name> [bin]" switches the sensor table the same way
		 * VIDIOC_S_FMT does, which is how a mode is brought up before
		 * a stream starts and how the scripts check one.
		 */
		char name[16];
		unsigned int bin = 0;

		/*
		 * sscanf() returns the number of converted fields: 2 when the
		 * binning factor is there, 1 when it is not.  Testing for == 1
		 * rejected every "mode <name> <bin>" spelling.
		 */
		if (sscanf(line + 4, "%15s %u", name, &bin) >= 1)
			ret = cam_mode_select(name, bin);
		else
			ret = -EINVAL;
	} else if (!strcmp(line, "modes")) {
		unsigned int i;

		pr_info("modes: %u tables in imx582_modes.h\n",
			(unsigned int)CAMCAP_IMX582_NMODES);
		for (i = 0; i < CAMCAP_IMX582_NMODES; i++) {
			const struct cam_sensor_mode *m = &cam_imx582_modes[i];

			pr_info("mode%u: %-12s %ux%u, HTS %u VTS %u, %u.%02u fps, exp_max %u%s\n",
				i, m->name, m->hsize, m->vsize, m->hts, m->vts,
				m->fps_x100 / 100, m->fps_x100 % 100,
				m->exp_max,
				(int)i == cam_mode_idx ? "  <- active" : "");
		}
		ret = 0;
	} else if (!strcmp(line, "stats")) {
		/*
		 * Measure the frame sitting in the capture buffer and report it
		 * in the usual "key value" shape so callers can grep one field.
		 * Also refreshes the values /proc/camcap_info prints.
		 */
		struct cam_stats st;

		cam_raw_stats(&st);
		cam_stats = st;
		cam_stat_mean_r = st.mean_r;
		cam_stat_mean_g = st.mean_g;
		cam_stat_mean_b = st.mean_b;
		cam_stat_clip_pct = st.count ?
			(unsigned int)(st.clip * 100 / (st.count * 2)) : 0;
		cam_stats_reads++;
		ret = 0;
	} else {
		pr_err("unknown command '%s' (expected: cfg | route | reroute | arm | probe | burst | stop | regs | stats | af | modes | mode)\n",
		       line);
		ret = -EINVAL;
	}

	cam_last_ret = ret;
	mutex_unlock(&cam_lock);
	kfree(line);

	if (ret)
		return ret;
	return count;
}

static const struct proc_ops camcap_proc_ops = {
	.proc_read	= camcap_read,
	.proc_write	= camcap_write,
	/*
	 * default_llseek keeps FMODE_LSEEK: lseek(fd, 0, SEEK_SET) rewinds and
	 * re-reads the frame buffer.  Without a proc_lseek the VFS clears
	 * FMODE_LSEEK (fs/proc/inode.c:proc_reg_open()) and lseek fails with
	 * -ESPIPE.
	 */
	.proc_lseek	= default_llseek,
};

/* ------------------------------------------------------------------ */
/* procfs - /proc/camcap_info                                         */
/* ------------------------------------------------------------------ */

/*
 * The per-frame timing block of /proc/camcap_info lives in a helper because the
 * V4L2 context it reads is defined further down (it needs struct vb2_queue).
 */
static void cam_v4l2_info_timing(char *info, int *i, size_t size);

static ssize_t camcap_info_read(struct file *file, char __user *ubuf,
				size_t count, loff_t *ppos)
{
	char info[2048];
	u32 int_status, ctl2, vf;
	int len, i = 0;

	if (!cam_base)
		return -ENODEV;

	int_status = cam_rd(CAMSV_INT_STATUS);
	ctl2 = cam_rd(CAMSV_FBC_IMGO_CTL2);
	vf = cam_rd(CAMSV_TG_VF_CON);

	i += scnprintf(info + i, sizeof(info) - i, "cam_cap / CAMSV1 status\n");
	i += scnprintf(info + i, sizeof(info) - i, "base_phys    : 0x%lx\n",
		       camsv_base);
	i += scnprintf(info + i, sizeof(info) - i, "buffer_dma   : 0x%llx (IMGO target)\n",
		       (unsigned long long)cam_frame_pa);
	i += scnprintf(info + i, sizeof(info) - i, "buffer_phys  : 0x%llx\n",
		       (unsigned long long)cam_buf_phys);
	i += scnprintf(info + i, sizeof(info) - i, "iova_mapped  : %zu at 0x%lx\n",
		       cam_iommu_mapped, map_iova);
	i += scnprintf(info + i, sizeof(info) - i, "buffer_size  : %lu\n",
		       cam_buf_size);
	i += scnprintf(info + i, sizeof(info) - i, "buffer_want  : %lu\n",
		       frame_bytes);
	i += scnprintf(info + i, sizeof(info) - i, "mapping      : %s\n",
		       cam_buf_kind());
	i += scnprintf(info + i, sizeof(info) - i, "iommu        : %s\n",
		       cam_dma_dev_is_iommu ? "disp IOMMU (address above is an IOVA)"
					    : "none (address above is a physical address)");
	i += scnprintf(info + i, sizeof(info) - i, "vf_on        : %u\n",
		       vf & CAMSV_TG_VF_CON_VFDATA_EN ? 1 : 0);
	i += scnprintf(info + i, sizeof(info) - i, "int_status   : 0x%08x\n",
		       int_status);
	i += scnprintf(info + i, sizeof(info) - i,
		       "fbc_imgo_ctl2: 0x%08x (IMGO_RCNT=%u IMGO_WCNT=%u IMGO_FBC_CNT=%u IMGO_DROP_CNT=%u)\n",
		       ctl2,
		       (u32)FIELD_GET(CAMSV_FBC_IMGO_CTL2_IMGO_RCNT, ctl2),
		       (u32)FIELD_GET(CAMSV_FBC_IMGO_CTL2_IMGO_WCNT, ctl2),
		       (u32)FIELD_GET(CAMSV_FBC_IMGO_CTL2_IMGO_FBC_CNT, ctl2),
		       (u32)FIELD_GET(CAMSV_FBC_IMGO_CTL2_IMGO_DROP_CNT, ctl2));
	i += scnprintf(info + i, sizeof(info) - i,
		       "route        : intf %u -> mux %u -> cam_mux %u (seninf %s)\n",
		       route_intf, route_mux, cammux,
		       cam_seninf ? "mapped" : "UNMAPPED");
	if (cam_seninf) {
		u32 pcsr = 0x0400u + 0x0020u * cammux;

		i += scnprintf(info + i, sizeof(info) - i,
			       "cam_mux_ctrl : 0x%08x (SRC_SEL=%u EN=%u)\n",
			       sen_rd(pcsr + SENINF_PCSR_CTRL),
			       sen_rd(pcsr + SENINF_PCSR_CTRL) &
				       SENINF_PCSR_SRC_SEL_MASK,
			       !!(sen_rd(pcsr + SENINF_PCSR_CTRL) & SENINF_PCSR_EN));
		i += scnprintf(info + i, sizeof(info) - i,
			       "cam_mux_chk  : 0x%08x (CHK_RES)\n",
			       sen_rd(pcsr + SENINF_PCSR_CHK_RES));
	}
	i += scnprintf(info + i, sizeof(info) - i, "frame_ready  : %u\n",
		       cam_frame_ready ? 1 : 0);
	i += scnprintf(info + i, sizeof(info) - i, "arm_count    : %u\n",
		       cam_arm_count);
	i += scnprintf(info + i, sizeof(info) - i, "last_seq     : %u\n",
		       cam_last_seq);
	i += scnprintf(info + i, sizeof(info) - i,
		       "last_int_st  : 0x%08x\n", cam_last_int_status);
	i += scnprintf(info + i, sizeof(info) - i,
		       "last_fbc_ctl2: 0x%08x\n", cam_last_fbc_ctl2);
	i += scnprintf(info + i, sizeof(info) - i, "last_result  : %d\n",
		       cam_last_ret);
	/*
	 * Frame statistics and the AE/AWB state derived from them.  mean_* are
	 * raw 12-bit values (pedestal included), not the 8-bit output.
	 */
	i += scnprintf(info + i, sizeof(info) - i,
		       "stats        : n=%llu r=%u g=%u b=%u min_g=%u max_g=%u clip=%u%% dark=%u%%\n",
		       (unsigned long long)cam_stats.count,
		       cam_stat_mean_r, cam_stat_mean_g, cam_stat_mean_b,
		       cam_stats.min_g, cam_stats.max_g, cam_stat_clip_pct,
		       cam_stats.count ? (unsigned int)(cam_stats.dark * 100 /
							(cam_stats.count * 2)) : 0);
	i += scnprintf(info + i, sizeof(info) - i,
		       "ae           : %s target=%u band=%u exp=0x%04x again=0x%04x dgain=0x%04x hw=(0x%04x,0x%04x,0x%04x) frames=%u mean_s=%u\n",
		       cam_ae_state(), ae_target, ae_band,
		       cam_sensor.exposure, cam_sensor.again, cam_sensor.dgain,
		       cam_sensor_hw.exposure, cam_sensor_hw.again,
		       cam_sensor_hw.dgain, cam_ae_frames, cam_ae_mean_s);
	i += scnprintf(info + i, sizeof(info) - i,
		       "awb          : %s wb_r=%u.%02u wb_b=%u.%02u rate=1/%u frames=%u\n",
		       cam_awb_auto ? "auto" : "manual",
		       cam_wb_r_cur >> 8, ((cam_wb_r_cur & 0xff) * 100) >> 8,
		       cam_wb_b_cur >> 8, ((cam_wb_b_cur & 0xff) * 100) >> 8,
		       1u << awb_rate, cam_awb_frames);
	i = cam_af_info(info, i, sizeof(info));
	i += scnprintf(info + i, sizeof(info) - i,
		       "sensor_i2c   : %s bus=%u addr=0x%02x adapter=%s\n",
		       sensor_ctl ? "on" : "off", i2c_bus, i2c_addr,
		       cam_i2c_adap ? cam_i2c_adap->name : "(not opened yet)");

	/*
	 * Per-frame timing.  This is where the frame period actually goes; the
	 * fps is derived from the measured period, not from a nominal frame
	 * rate.  arm includes the hardware wait for the frame, conv is the
	 * debayer/convert (parallel or inline), gov is the AE/AWB update, and
	 * idle is the time the capture thread spent with no buffer queued.
	 */
	cam_v4l2_info_timing(info, &i, sizeof(info));

	len = i;
	return simple_read_from_buffer(ubuf, count, ppos, info, len);
}

static const struct proc_ops camcap_info_proc_ops = {
	.proc_read	= camcap_info_read,
	.proc_lseek	= default_llseek,
};

/* ------------------------------------------------------------------ */
/* Device-tree / IOMMU plumbing                                       */
/* ------------------------------------------------------------------ */

static void cam_release_iommu_dev(void);

/*
 * Add camcap@1a110000 under /soc with an "iommus" property, let the OF
 * reconfiguration notifier populate it into a platform_device, and ask
 * of_dma_configure() to put that device behind the display IOMMU.
 *
 * On success cam_dma_dev is set and cam_dma_dev_is_iommu is true, so
 * dma_alloc_coherent() returns an IOVA.  Every failure is non-fatal: the caller
 * falls back to the plain platform_device_register_full() device.
 */
static int cam_setup_iommu_dev(void)
{
	struct device_node *iommu_np, *soc, *np;
	struct platform_device *pdev;
	u32 ph = 0, iommus[2];
	int ret;

	iommu_np = of_find_compatible_node(NULL, NULL, CAMCAP_DT_IOMMU_COMPAT);
	if (!iommu_np) {
		pr_warn("no \"%s\" node; cannot put the buffer behind the IOMMU\n",
			CAMCAP_DT_IOMMU_COMPAT);
		return -ENODEV;
	}

	/* /soc, which carries OF_POPULATED_BUS and so auto-populates children */
	soc = of_get_parent(iommu_np);
	if (!soc) {
		pr_warn("%pOF has no parent; cannot add " CAMCAP_DT_NODE_NAME "\n",
			iommu_np);
		of_node_put(iommu_np);
		return -ENODEV;
	}

	ret = of_property_read_u32(iommu_np, "phandle", &ph);
	if (ret)
		ret = of_property_read_u32(iommu_np, "linux,phandle", &ph);
	if (ret) {
		pr_warn("%pOF has no phandle; cannot reference it from a node\n",
			iommu_np);
		goto out_put;
	}

	of_changeset_init(&cam_dt_cs);
	cam_dt_cs_inited = true;

	np = of_changeset_create_node(&cam_dt_cs, soc, CAMCAP_DT_NODE_NAME);
	if (!np) {
		pr_warn("of_changeset_create_node(" CAMCAP_DT_NODE_NAME ") failed\n");
		ret = -ENOMEM;
		goto out_put;
	}

	ret = of_changeset_add_prop_string(&cam_dt_cs, np, "compatible",
					   CAMCAP_DT_COMPATIBLE);
	if (ret)
		goto out_put;

	iommus[0] = ph;
	iommus[1] = CAMCAP_DT_IOMMU_ID;
	ret = of_changeset_add_prop_u32_array(&cam_dt_cs, np, "iommus", iommus,
					      2);
	if (ret)
		goto out_put;

	ret = of_changeset_apply(&cam_dt_cs);
	if (ret) {
		pr_warn("of_changeset_apply(" CAMCAP_DT_NODE_NAME ") = %d\n", ret);
		goto out_put;
	}
	cam_dt_applied = true;
	cam_dt_node = np;

	/*
	 * of_platform_notify() has almost certainly populated the bus already
	 * (its ADD case fires for children of OF_POPULATED_BUS parents), in
	 * which case of_platform_device_create() would refuse because the node
	 * is flagged OF_POPULATED.  Look the device up first either way.
	 */
	pdev = of_find_device_by_node(np);
	if (!pdev)
		pdev = of_platform_device_create(np, NULL, NULL);
	if (!pdev) {
		pr_warn("no platform_device for " CAMCAP_DT_NODE_NAME "\n");
		ret = -ENODEV;
		goto out_put;
	}
	cam_dt_pdev_ref = true;

	/*
	 * of_platform_device_create_pdata() deliberately does not call this,
	 * and no driver will ever probe our node, so the IOMMU is configured
	 * here.  With CONFIG_IOMMU_DEFAULT_DMA_STRICT=y this ends in
	 * iommu_probe_device() -> attach_dev -> mtk_iommu_hw_init() +
	 * REG_MMU_PT_BASE_ADDR, which is the whole point.
	 */
	ret = of_dma_configure(&pdev->dev, np, true);
	if (ret) {
		pr_warn("of_dma_configure(" CAMCAP_DT_NODE_NAME ") = %d\n", ret);
		goto out_put;
	}

	cam_dt_pdev = pdev;
	cam_iommu_dev = &pdev->dev;
	cam_iommu_dom = iommu_get_domain_for_dev(cam_iommu_dev);
	if (!cam_iommu_dom)
		pr_warn("no IOMMU domain on %s; CAMSV IMGO will not be mapped\n",
			dev_name(cam_iommu_dev));
	pr_info("iommu device: %s, iommus = <%u %u> (larb %u port %u), domain %p\n",
		dev_name(cam_iommu_dev), ph, CAMCAP_DT_IOMMU_ID,
		CAMCAP_DT_IOMMU_ID >> 5, CAMCAP_DT_IOMMU_ID & 0x1f,
		cam_iommu_dom);
	ret = 0;

out_put:
	of_node_put(soc);
	of_node_put(iommu_np);
	if (ret)
		cam_release_iommu_dev();
	return ret;
}

/*
 * Undo cam_setup_iommu_dev().  The changeset revert comes first on purpose: it
 * is what makes of_platform_notify() unregister the device, and the reference
 * this module holds is what keeps the object alive until it has.
 * cam_dt_node is owned by the changeset and must never be of_node_put() here.
 */
static void cam_release_iommu_dev(void)
{
	if (cam_dt_cs_inited) {
		if (cam_dt_applied)
			of_changeset_revert(&cam_dt_cs);
		of_changeset_destroy(&cam_dt_cs);
		cam_dt_cs_inited = false;
		cam_dt_applied = false;
	}
	cam_dt_node = NULL;
	if (cam_dt_pdev_ref) {
		platform_device_put(cam_dt_pdev);
		cam_dt_pdev_ref = false;
	}
	cam_dt_pdev = NULL;
	cam_iommu_dev = NULL;
	cam_iommu_dom = NULL;
	cam_dma_dev = NULL;
	cam_dma_dev_is_iommu = false;
}

/*
 * Tear the explicit iommu_map() down again.  Must run before the buffer is
 * handed back, so no in-flight CAMSV burst can walk into freed memory.
 */
static void cam_unmap_iova(void)
{
	size_t n;

	if (!cam_iommu_mapped || !cam_iommu_dom)
		return;

	n = iommu_unmap(cam_iommu_dom, (dma_addr_t)map_iova, cam_iommu_mapped);
	if (n != cam_iommu_mapped)
		pr_warn("iommu_unmap(%#lx, %zu) = %zu\n", map_iova,
			cam_iommu_mapped, n);
	cam_iommu_mapped = 0;
}

/* ------------------------------------------------------------------ */
/* V4L2 / videobuf2 capture device                                    */
/* ------------------------------------------------------------------ */

/*
 * Geometry of what CAMSV1 actually writes into the CMA buffer: 4000 x 3000
 * 12-bit pixels packed 2-per-3-bytes, i.e. 6000 bytes per line.  This is the
 * layout the full-frame captures settled on (docs/CAMERA_CAPTURE_WORKING.md).
 */
#define CAMCAP_SRC_WIDTH	4000
#define CAMCAP_SRC_HEIGHT	3000
#define CAMCAP_SRC_STRIDE	6000
#define CAMCAP_SRC_BYTES	(CAMCAP_SRC_STRIDE * CAMCAP_SRC_HEIGHT)	/* 18 000 000 */
/*
 * Buffer size for that frame.  It must be a multiple of the 4 KiB page size:
 * iommu_map() rejects a non-page-aligned size, and the module then silently
 * falls back to physical addressing, where the CAMSV write faults and every
 * frame comes back as zeros:
 *   iommu: unaligned: iova 0x10000000 pa 0xfa500000 size 0x112a880 min_pagesz 0x1000
 *   cam_cap: iommu_map(iova 0x10000000, pa 0xfa500000, 18000000) = -22
 * 18000000 = 0x112a880 is exactly that trap; 18874368 = 0x1200000 (18 MiB) is
 * the value every verified capture used, and the module rounds other sizes up.
 */
#define CAMCAP_FRAME_BYTES	18874368UL
#define CAMCAP_BIN		2
#define CAMCAP_FS		4095	/* 12-bit full scale */

/* one vb2 buffer plus our own queue linkage */
struct cam_v4l2_buf {
	struct vb2_v4l2_buffer	vb;
	struct list_head	list;
};

/*
 * A conversion worker: one contiguous band of output rows, and the generation
 * counter of the last frame it converted.  A worker initialises `seen` to the
 * pool's current generation, so it sleeps until the next frame is published.
 */
struct cam_v4l2_ctx;

struct cam_conv_worker {
	struct cam_v4l2_ctx	*ctx;
	unsigned int		id;
	struct task_struct	*task;
	unsigned int		seen;
	unsigned int		y0, y1;
	struct cam_stats	st;		/* this worker's share of the sums */
};

#define CAMCAP_MAX_CONV	8

/* Upper bound on the capture pipeline depth; see the pipe_slots parameter. */
#define CAMCAP_PIPE_SLOTS	4

/*
 * One raw frame buffer of the capture pipeline.
 *
 * Serial capture costs arm + conv per frame: the CAMSV only raises "frame
 * done" once the sensor has shifted the whole frame out (~32 ms of a 32.6 ms
 * sensor period), and the conversion then runs for another ~30 ms before the
 * next arm -- so every frame pays both and the rate is stuck at arm+conv
 * (measured 11.7 fps with arm 50.6 ms + conv 33.6 ms, and the arm itself is
 * 25 fps because the next frame starts before the poll notices this one).
 *
 * With two buffers the arm thread fills one while the converter thread reads
 * the other, so a frame costs max(arm, conv) and nothing else.  slot[0] is the
 * CMA buffer the module already owns; the rest are allocated by
 * cam_slots_alloc() and handed out by cam_pipe_setup().
 */
struct cam_slot {
	u8		*va;		/* cacheable CPU view of the frame */
	dma_addr_t	iova;		/* what CAMSV IMGO is pointed at */
	phys_addr_t	phys;		/* for the DMA cache maintenance */
	size_t		size;
	struct page	*pages;		/* contiguous block, extra slots only */
	struct cam_v4l2_buf *vb;	/* buffer the converter fills in */
	bool		full;		/* DMA done, converter has to pick it up */
	bool		busy;		/* converter is reading it right now */
	u32		seq;
};

/*
 * How many rows apart the scene statistics are sampled.  1 would measure
 * every converted row (the conversion loop already holds all four taps, so
 * folding them in is nearly free) but the accumulate block is still ~20
 * instructions in the innermost loop, and AE/AWB only need a sample: the pass
 * this replaced measured every 64th 2x2 block.  A strided row is enough.
 */
#define CAMCAP_STATS_ROW_STEP	4

struct cam_v4l2_ctx {
	struct v4l2_device	v4l2_dev;
	struct video_device	vdev;
	struct vb2_queue	queue;
	struct mutex		lock;		/* ioctl serialisation (q->lock) */
	spinlock_t		qlock;		/* guards queued/streaming */
	struct list_head	queued;
	wait_queue_head_t	wq;
	struct task_struct	*thread;
	bool			streaming;
	u32			seq;

	/* parallel conversion pool (see cam_conv_pool_start) */
	struct cam_conv_worker	conv[CAMCAP_MAX_CONV];
	unsigned int		nconv;		/* 0 = convert inline */
	unsigned int		job_gen;	/* bumped once per frame */
	const u8		*job_src;
	u8			*job_dst;
	wait_queue_head_t	conv_wq;
	struct completion	conv_done;

	/* capture pipeline (see struct cam_slot / cam_pipe_setup) */
	struct cam_slot		slot[CAMCAP_PIPE_SLOTS];
	unsigned int		nslots;		/* 1 = serial, else 2..CAP */
	unsigned int		slot_next;	/* slot the arm thread arms next */
	struct task_struct	*cthread;	/* converter thread */
	wait_queue_head_t	pipe_wq;
	bool			pipe_used;	/* extra slots are up and in use */
	u32			pipe_frames;
	/*
	 * Frame-period histogram: the mean hides the frames that slipped a
	 * sensor period because the arm thread missed the frame start.  See
	 * cam_per_hist_add().
	 */
	u32			per_hist[4];

	/* per-frame timings in ns (last frame) and a frame counter */
	u64			t_arm, t_conv, t_gov, t_wait, t_total, t_period;
	unsigned int		t_frames;
	/* running sums over the current stream, for a trustworthy fps figure */
	u64			s_arm, s_conv, s_gov, s_period;
	unsigned int		s_n;

	/* controls */
	struct v4l2_ctrl_handler hdl;
	struct v4l2_ctrl	*ct_exposure;	/* V4L2_CID_EXPOSURE_ABSOLUTE */
	struct v4l2_ctrl	*ct_again;	/* V4L2_CID_ANALOGUE_GAIN */
	struct v4l2_ctrl	*ct_exp_auto;	/* V4L2_CID_EXPOSURE_AUTO */
	struct v4l2_ctrl	*ct_awb;	/* V4L2_CID_AUTO_WHITE_BALANCE */
	struct v4l2_ctrl	*ct_red;	/* V4L2_CID_RED_BALANCE */
	struct v4l2_ctrl	*ct_blue;	/* V4L2_CID_BLUE_BALANCE */
	struct v4l2_ctrl	*ct_dgain;	/* V4L2_CID_DIGITAL_GAIN */
	struct v4l2_ctrl	*ct_focus;	/* V4L2_CID_FOCUS_ABSOLUTE */
	struct v4l2_ctrl	*ct_focus_auto;	/* V4L2_CID_FOCUS_AUTO */
	unsigned int		dgain;		/* sensor 0x020e, AE-steered */
};

/* the single instance; NULL until the V4L2 device is registered */
static struct cam_v4l2_ctx *cam_v4l2;

/*
 * Per-frame timing for /proc/camcap_info, in the same key/value shape as the
 * rest of that file.  arm is the hardware wait plus the register sequence, conv
 * is the debayer/convert (parallel or inline), gov is the AE/AWB update, and
 * idle is the time the capture thread spent with no buffer queued.  period is
 * measured between two consecutive frames, so fps is a measurement.
 */
static void cam_v4l2_info_timing(char *info, int *ip, size_t size)
{
	struct cam_v4l2_ctx *vc = cam_v4l2;
	int i = *ip;
	u64 per;
	u32 fps100;

	if (!vc) {
		i += scnprintf(info + i, size - i,
			       "convert      : (no V4L2 device)\n");
		*ip = i;
		return;
	}

	per = vc->t_period ? vc->t_period : (vc->t_total + vc->t_wait);
	fps100 = per ? (u32)div_u64(100000000000ULL, per) : 0;

	i += scnprintf(info + i, size - i,
		       "convert      : %s threads=%u rows %u..%u gen=%u\n",
		       vc->nconv ? "parallel" : "inline", vc->nconv,
		       vc->nconv ? vc->conv[0].y0 : 0,
		       vc->nconv ? vc->conv[vc->nconv - 1].y1 : out_height,
		       vc->job_gen);
	i += scnprintf(info + i, size - i,
		       "timing       : period=%lluus arm=%lluus conv=%lluus gov=%lluus idle=%lluus fps=%u.%02u frames=%u\n",
		       div_u64(per, 1000), div_u64(vc->t_arm, 1000),
		       div_u64(vc->t_conv, 1000), div_u64(vc->t_gov, 1000),
		       div_u64(vc->t_wait, 1000),
		       fps100 / 100, fps100 % 100, vc->t_frames);
	if (vc->s_n) {
		u64 n = vc->s_n;
		u64 av = div_u64(vc->s_period, n);
		u32 afps = av ? (u32)div_u64(100000000000ULL, av) : 0;

		i += scnprintf(info + i, size - i,
			       "avg          : period=%lluus arm=%lluus conv=%lluus gov=%lluus fps=%u.%02u frames=%llu pipe=%u\n",
			       div_u64(av, 1000), div_u64(vc->s_arm, n * 1000),
			       div_u64(vc->s_conv, n * 1000),
			       div_u64(vc->s_gov, n * 1000),
			       afps / 100, afps % 100, n, vc->pipe_frames);
		i += scnprintf(info + i, size - i,
			       "dist         : clean(<32ms)=%u late(32-40ms)=%u slip(40-58ms)=%u lost(>=58ms)=%u\n",
			       vc->per_hist[0], vc->per_hist[1],
			       vc->per_hist[2], vc->per_hist[3]);
	}
	*ip = i;
}


/* 12-bit sensor value -> 8-bit YUV component, one table per Bayer channel */
static u8 cam_lut_r[4096];
static u8 cam_lut_g[4096];
static u8 cam_lut_b[4096];

/*
 * Build one tone-curve table:
 *   1. subtract the black pedestal
 *   2. scale by the white-balance gain for that channel (R/B)
 *   3. contrast around mid grey
 *   4. scale by the shared linear pre-gain, clamp at full scale
 *   5. add the brightness offset
 *   6. sqrt() for gamma
 *
 * Step 6 is x^0.5 rather than a true 1/2.2 gamma because the kernel has no
 * libm; int_sqrt() is exact, monotonic and cheap, and it is what the offline
 * renderer used to produce the reference images.
 */
static void cam_lut_build(u8 *lut, unsigned int wb_q8)
{
	unsigned int v;
	int con = out_contrast < 0 ? 0 : (out_contrast > 255 ? 255 : out_contrast);
	int bri = out_brightness < -128 ? -128 :
		  (out_brightness > 128 ? 128 : out_brightness);

	for (v = 0; v < 4096; v++) {
		u32 n, x, y;
		int t;

		n = v > v4l2_black ? v - v4l2_black : 0;
		n = (u32)(((u64)n * wb_q8) >> 8);

		/* contrast pivots around mid grey (2048 of 4095) */
		t = ((int)n - 2048) * con / 128 + 2048;
		n = t < 0 ? 0 : (u32)t;

		n = (u32)(((u64)n * v4l2_gain_q8) >> 8);

		/* brightness is a fraction of full scale: +/-128 -> +/-2048 */
		t = (int)n + bri * 16;
		n = t < 0 ? 0 : (u32)t;
		if (n > CAMCAP_FS)
			n = CAMCAP_FS;

		/* normalise to 0..65535 then take the square root: 256*sqrt(x) */
		x = (n << 16) / CAMCAP_FS;
		y = (u32)int_sqrt((u64)x << 16);
		y = (y + 128) >> 8;
		lut[v] = y > 255 ? 255 : (u8)y;
	}
}

/* Set by the control callbacks; the capture thread does the real work. */
static bool cam_lut_dirty = true;

/* Rebuild all three tables from the current white balance and output knobs. */
static void cam_lut_rebuild(void)
{
	cam_lut_build(cam_lut_r, cam_wb_r_cur);
	cam_lut_build(cam_lut_g, 256);
	cam_lut_build(cam_lut_b, cam_wb_b_cur);
	cam_lut_dirty = false;
	cam_lut_ver++;
}

/* The two pixels held in one 3-byte 12-bit group (MTK packing, LSB first). */
static inline u16 cam_px_even(const u8 *line, unsigned int pair)
{
	const u8 *p = line + (size_t)pair * 3;

	return (u16)p[0] | ((u16)(p[1] & 0x0f) << 8);
}

static inline u16 cam_px_odd(const u8 *line, unsigned int pair)
{
	const u8 *p = line + (size_t)pair * 3;

	return (u16)(p[1] >> 4) | ((u16)p[2] << 4);
}

/* The same two pixels addressed by absolute column instead of by group. */
static inline u16 cam_px_at(const u8 *line, unsigned int col)
{
	const u8 *p = line + (size_t)(col >> 1) * 3;

	return (col & 1) ? ((u16)(p[1] >> 4) | ((u16)p[2] << 4))
			 : ((u16)p[0] | ((u16)(p[1] & 0x0f) << 8));
}

/*
 * cam_px_at() with the column pulled back inside the frame.  The bilinear path
 * wants the neighbours of the pixels on the border, which do not exist; the
 * edge sample is repeated instead, the same way the sensor clamps its own
 * readout window.
 */
static inline u16 cam_px_clamp(const u8 *line, int col, unsigned int w)
{
	if (col < 0)
		col = 0;
	else if ((unsigned int)col >= w)
		col = (int)w - 1;
	return cam_px_at(line, (unsigned int)col);
}

/*
 * Bilinear demosaic of one pixel of the RGGB grid the vendor declares
 * (src/imx586_Sensor.c:256, SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_R):
 *
 *   (even x, even y) R    (odd x, even y) Gr    (even x, odd y) Gb    (odd x, odd y) B
 *
 * A pixel keeps the tap it owns; the colour across it is the average of its
 * four orthogonal neighbours, the colour opposite it the average of the four
 * diagonals.  The averages stay on raw 12-bit samples and only their result
 * goes through a tone table, because the tables end in a square-root gamma:
 * averaging after them would lift the peaks.
 *
 * lm/l0/lp are the raw rows y-1, y and y+1 (clamped by the caller).
 */
static inline void cam_bayer_px(const u8 *lm, const u8 *l0, const u8 *lp,
				unsigned int x, unsigned int y, unsigned int w,
				u32 *pr, u32 *pg, u32 *pb)
{
	const int xm = (int)x - 1, xp = (int)x + 1;
	u32 own = cam_px_at(l0, x);

	if (!(x & 1)) {
		if (!(y & 1)) {			/* R: greens around, blues diagonal */
			*pr = own;
			*pg = (cam_px_clamp(l0, xm, w) + cam_px_clamp(l0, xp, w) +
			       cam_px_clamp(lm, x, w) + cam_px_clamp(lp, x, w)) >> 2;
			*pb = (cam_px_clamp(lm, xm, w) + cam_px_clamp(lm, xp, w) +
			       cam_px_clamp(lp, xm, w) + cam_px_clamp(lp, xp, w)) >> 2;
		} else {			/* Gb: reds vertical, blues horizontal */
			*pg = own;
			*pr = (cam_px_clamp(lm, x, w) + cam_px_clamp(lp, x, w)) >> 1;
			*pb = (cam_px_clamp(l0, xm, w) + cam_px_clamp(l0, xp, w)) >> 1;
		}
	} else {
		if (!(y & 1)) {			/* Gr: reds horizontal, blues vertical */
			*pg = own;
			*pr = (cam_px_clamp(l0, xm, w) + cam_px_clamp(l0, xp, w)) >> 1;
			*pb = (cam_px_clamp(lm, x, w) + cam_px_clamp(lp, x, w)) >> 1;
		} else {			/* B: greens around, reds diagonal */
			*pb = own;
			*pg = (cam_px_clamp(l0, xm, w) + cam_px_clamp(l0, xp, w) +
			       cam_px_clamp(lm, x, w) + cam_px_clamp(lp, x, w)) >> 2;
			*pr = (cam_px_clamp(lm, xm, w) + cam_px_clamp(lm, xp, w) +
			       cam_px_clamp(lp, xm, w) + cam_px_clamp(lp, xp, w)) >> 2;
		}
	}
}

/*
 * The raw frame lives in a dma_alloc_coherent() buffer.  On arm64 a
 * non-coherent device gets a *non-cached* vmalloc remap of those pages
 * (CONFIG_DMA_DIRECT_REMAP=y, see dma_direct_alloc()), while the linear-map
 * alias of the same physical pages stays cacheable.  Reading the frame
 * through the uncached mapping is the single biggest cost in the pipeline
 * (measured: conv 106 ms for 18 MB with four threads, i.e. 4.57 fps), so the
 * conversion and the statistics read the cacheable linear-map alias instead.
 *
 * That is exactly what dma_sync_single_for_cpu() is for: on a non-coherent
 * device it invalidates the linear-map alias of the range
 * (arch_sync_dma_for_cpu() -> dcache_inval_poc_nosync(phys_to_virt(paddr))),
 * so the CPU can never keep a stale copy of a frame the sensor wrote behind
 * its back.  The buffer is never written by the CPU, so no clean is needed.
 *
 * cam_frame_wb is NULL or equal to cam_frame when the buffer is not a DMA
 * allocation (alloc_pages_exact() already returns the linear map) or when the
 * coherent address is an IOVA, i.e. when no cacheable alias can be found.
 */
static const u8 *cam_raw_src(void)
{
	return (const u8 *)(cam_frame_wb ? cam_frame_wb : cam_frame);
}

/* Call after CAMSV has written a frame and before the CPU reads it. */
static void cam_raw_sync(void)
{
	if (!cam_frame_wb || cam_frame_wb == cam_frame || !cam_dma_dev)
		return;
	dma_sync_single_for_cpu(cam_dma_dev, cam_buf_phys, cam_buf_size,
				DMA_FROM_DEVICE);
}

/*
 * Scene statistics, measured on the raw 12-bit frame with the black pedestal
 * still in it (the tone curve has not been applied yet).
 *
 * Sampling uses a fixed stride, so the cost is flat no matter the mode: every
 * 8th line and every 8th 2x2 block is ~190k blocks, which is plenty of
 * statistics and only a fraction of a millisecond on this SoC.
 */
#define CAMCAP_STATS_STEP	8
#define CAMCAP_CLIP_LEVEL	4000

static void cam_raw_stats(struct cam_stats *st)
{
	const u8 *src;
	unsigned int pair, line;
	const unsigned int half = cam_src_w / 2;
	const unsigned int stride = cam_src_stride;

	memset(st, 0, sizeof(*st));
	st->min_g = 0xffff;

	cam_raw_sync();
	src = cam_raw_src();
	if (!src)
		return;

	for (line = 0; line + 1 < cam_src_h; line += CAMCAP_STATS_STEP) {
		const u8 *l0 = src + (size_t)line * stride;
		const u8 *l1 = l0 + stride;

		for (pair = 0; pair < half; pair += CAMCAP_STATS_STEP) {
			/*
			 * The (0,0) tap is a 2x2 position, not a colour: which colour
			 * it carries is rb_swap's business, and the AWB below trusts
			 * these two labels, so they must move together with the
			 * conversion.
			 */
			u32 t00 = cam_px_even(l0, pair);
			u32 t11 = cam_px_odd(l1, pair);
			u32 r = rb_swap ? t11 : t00;
			u32 g0 = cam_px_odd(l0, pair);
			u32 g1 = cam_px_even(l1, pair);
			u32 b = rb_swap ? t00 : t11;
			u32 gm = (g0 + g1) >> 1;

			st->sum_r += r;
			st->sum_g += g0 + g1;
			st->sum_b += b;
			st->count++;

			if (gm < st->min_g)
				st->min_g = gm;
			if (gm > st->max_g)
				st->max_g = gm;
			if (g0 >= CAMCAP_CLIP_LEVEL)
				st->clip++;
			if (g1 >= CAMCAP_CLIP_LEVEL)
				st->clip++;
			if (g0 <= v4l2_black)
				st->dark++;
			if (g1 <= v4l2_black)
				st->dark++;
		}
	}

	if (!st->count) {
		st->min_g = 0;
		return;
	}

	/* green has two samples per 2x2 block, red and blue one each */
	st->mean_r = (u32)(st->sum_r / st->count);
	st->mean_g = (u32)(st->sum_g / (st->count * 2));
	st->mean_b = (u32)(st->sum_b / st->count);
}

/* ------------------------------------------------------------------ */
/* Sensor I2C                                                         */
/* ------------------------------------------------------------------ */

/*
 * Every IMX582 control register this module touches is 16 bits wide behind a
 * 16-bit register address, so one write is a single 4-byte transfer.  Writing
 * only one data byte silently no-ops in the sensor, which cost a lot of time
 * to discover, hence the shape of this helper.
 */
static int cam_sensor_write16(unsigned int reg, unsigned int val)
{
	u8 buf[4] = { reg >> 8, reg & 0xff, val >> 8, val & 0xff };
	struct i2c_msg msg = {
		.addr = i2c_addr,
		.flags = 0,
		.len = sizeof(buf),
		.buf = buf,
	};
	int ret;

	if (!cam_i2c_adap)
		return -ENODEV;

	ret = i2c_transfer(cam_i2c_adap, &msg, 1);
	if (ret < 0)
		return ret;
	return ret == 1 ? 0 : -EIO;
}

/*
 * Byte-register write: register address plus exactly one data byte.  The mode
 * tables are full of byte registers (0x0306, 0x0307, 0x0340, 0x0341), and a
 * two-byte payload is auto-incremented into the following register instead.
 * scripts/imx582_bring.py writes them the same way.
 */
static int cam_sensor_write8(unsigned int reg, unsigned int val)
{
	u8 buf[3] = { reg >> 8, reg & 0xff, val & 0xff };
	struct i2c_msg msg = {
		.addr = i2c_addr,
		.flags = 0,
		.len = sizeof(buf),
		.buf = buf,
	};
	int ret;

	if (!cam_i2c_adap)
		return -ENODEV;

	ret = i2c_transfer(cam_i2c_adap, &msg, 1);
	if (ret < 0)
		return ret;
	return ret == 1 ? 0 : -EIO;
}

/* Read one byte register back (two messages: register address, then data). */
static int cam_sensor_read8(unsigned int reg, unsigned int *val)
{
	u8 addr[2] = { reg >> 8, reg & 0xff };
	u8 data;
	struct i2c_msg msg[2] = {
		{ .addr = i2c_addr, .flags = 0, .len = sizeof(addr), .buf = addr },
		{ .addr = i2c_addr, .flags = I2C_M_RD, .len = 1, .buf = &data },
	};
	int ret;

	if (!cam_i2c_adap)
		return -ENODEV;

	ret = i2c_transfer(cam_i2c_adap, msg, 2);
	if (ret < 0)
		return ret;
	if (ret != 2)
		return -EIO;
	*val = data;
	return 0;
}

static struct i2c_adapter *cam_i2c_get(void)
{
	struct i2c_adapter *adap;
	static bool warned;

	if (cam_i2c_adap)
		return cam_i2c_adap;
	if (!sensor_ctl)
		return NULL;

	adap = i2c_get_adapter(i2c_bus);
	if (!adap) {
		if (!warned) {
			warned = true;
			pr_warn("cam_cap: no I2C adapter %u\n", i2c_bus);
		}
		return NULL;
	}
	if (i2c_name && *i2c_name && !strstr(adap->name, i2c_name)) {
		if (!warned) {
			warned = true;
			pr_warn("cam_cap: adapter %u is '%s', which does not contain '%s'\n",
				i2c_bus, adap->name, i2c_name);
		}
		i2c_put_adapter(adap);
		return NULL;
	}
	warned = false;

	cam_i2c_adap = adap;
	pr_info("cam_cap: sensor I2C bus %u ('%s'), address 0x%02x\n",
		i2c_bus, adap->name, i2c_addr);
	return adap;
}

/* Program exposure (0x0202), analogue gain (0x0204) and digital gain (0x020e). */
static void cam_sensor_apply(void)
{
	struct cam_sensor_state s = cam_sensor;

	if (!sensor_ctl)
		return;
	if (cam_sensor_valid &&
	    cam_sensor_hw.exposure == s.exposure &&
	    cam_sensor_hw.again == s.again &&
	    cam_sensor_hw.dgain == s.dgain)
		return;

	if (!cam_i2c_get())
		return;

	if (!cam_sensor_valid || cam_sensor_hw.exposure != s.exposure) {
		if (cam_sensor_write16(0x0202, s.exposure))
			pr_warn("cam_cap: exposure write 0x%04x failed\n", s.exposure);
	}
	if (!cam_sensor_valid || cam_sensor_hw.again != s.again) {
		if (cam_sensor_write16(0x0204, s.again))
			pr_warn("cam_cap: gain write 0x%04x failed\n", s.again);
	}
	if (!cam_sensor_valid || cam_sensor_hw.dgain != s.dgain) {
		if (cam_sensor_write16(0x020e, s.dgain))
			pr_warn("cam_cap: digital gain write 0x%04x failed\n", s.dgain);
	}

	cam_sensor_hw = s;
	cam_sensor_valid = true;
}

/* ------------------------------------------------------------------ */
/* Autofocus: DW9800V voice-coil actuator + contrast-detect search    */
/* ------------------------------------------------------------------ */

/*
 * The lens is moved by a DW9800V at 0x0c on the same I2C bus as the sensor.  It
 * is an open-loop DAC: 0x03/0x04 hold the 10-bit code, the coil current is
 * code/1023 * 100 mA, and the chip's own AAC block ramps to that code (0x06 =
 * AAC mode + clock divider, 0x07 = ramp time, 0x02 = power/AAC switch).
 * mainline's dw9768.c uses the same register map, and the vendor driver's own
 * power-on writes in the stock dmesg (hyperos dmesg 13889-13897) are the same
 * five values used below.
 *
 * There is no position feedback, so focus is found by contrast: the mean |dY| of
 * a frame (cam_stats.fv) peaks exactly when the lens is in focus.  The search
 * sweeps the DAC, keeps the best position and then refines around it.  The lens
 * is mechanically at rest at DAC 0, which on this module is focused at macro
 * distance, so every position the search uses is a real move towards infinity.
 */

static unsigned int vcm_addr = 0x0c;
module_param(vcm_addr, uint, 0644);
MODULE_PARM_DESC(vcm_addr, "7-bit I2C address of the VCM driver (default 0x0c)");

static bool vcm_enable = true;
module_param(vcm_enable, bool, 0644);
MODULE_PARM_DESC(vcm_enable, "power and drive the lens actuator (default 1)");

static unsigned int af_enable = 1;
module_param(af_enable, uint, 0644);
MODULE_PARM_DESC(af_enable, "contrast autofocus: 0 = leave the lens alone (default 1)");

static unsigned int af_min;
module_param(af_min, uint, 0644);
MODULE_PARM_DESC(af_min, "lowest DAC code the autofocus search may use");

static unsigned int af_max = 1023;
module_param(af_max, uint, 0644);
MODULE_PARM_DESC(af_max, "highest DAC code the autofocus search may use (default 1023)");

static unsigned int af_step = 128;
module_param(af_step, uint, 0644);
MODULE_PARM_DESC(af_step, "coarse search step in DAC codes (default 128)");

static unsigned int af_pos;
module_param(af_pos, uint, 0644);
MODULE_PARM_DESC(af_pos, "focus DAC code to hold when the search is off");

static bool af_auto = true;
module_param(af_auto, bool, 0644);
MODULE_PARM_DESC(af_auto, "re-focus when the scene changes (default 1)");

static bool af_trace = true;
module_param(af_trace, bool, 0644);
MODULE_PARM_DESC(af_trace, "log every autofocus measurement (default 1)");

/* One register address followed by n data bytes, in a single transfer. */
static int cam_vcm_write(unsigned int reg, const u8 *data, unsigned int n)
{
	u8 buf[8];
	struct i2c_msg msg = {
		.addr = vcm_addr,
		.flags = 0,
		.buf = buf,
	};
	int ret;

	if (!cam_i2c_adap || n > sizeof(buf) - 1)
		return -EINVAL;
	buf[0] = reg & 0xff;
	memcpy(buf + 1, data, n);
	msg.len = n + 1;

	ret = i2c_transfer(cam_i2c_adap, &msg, 1);
	if (ret < 0)
		return ret;
	return ret == 1 ? 0 : -EIO;
}

static int cam_vcm_write8(unsigned int reg, u8 val)
{
	return cam_vcm_write(reg, &val, 1);
}

static int cam_vcm_read8(unsigned int reg, u8 *val)
{
	u8 r = reg & 0xff;
	struct i2c_msg msg[2] = {
		{ .addr = vcm_addr, .flags = 0, .len = 1, .buf = &r },
		{ .addr = vcm_addr, .flags = I2C_M_RD, .len = 1, .buf = val },
	};
	int ret;

	if (!cam_i2c_adap)
		return -ENODEV;
	ret = i2c_transfer(cam_i2c_adap, msg, 2);
	if (ret < 0)
		return ret;
	return ret == 2 ? 0 : -EIO;
}

static bool cam_vcm_ready;
static unsigned int cam_vcm_pos = 0xffff;	/* 0xffff = not written yet */

static int cam_vcm_set(unsigned int pos)
{
	u8 d[2];
	int ret;

	if (!vcm_enable)
		return 0;
	if (pos > 1023)
		pos = 1023;
	if (pos == cam_vcm_pos)
		return 0;
	d[0] = (u8)((pos >> 8) & 0x03);
	d[1] = (u8)(pos & 0xff);
	ret = cam_vcm_write(0x03, d, 2);
	if (ret) {
		pr_warn("cam_cap: VCM DAC write %u failed (%d)\n", pos, ret);
		return ret;
	}
	cam_vcm_pos = pos;
	af_pos = pos;
	return 0;
}

/*
 * Power-on sequence.  0x02 = 0x01 resets the driver, 0x02 = 0x00 takes it out of
 * reset, 0x02 = 0x02 enables AAC, 0x06 = 0x40 selects AAC3 (0.70 * Tvib) with
 * the /2 clock and 0x07 = 0x01 gives Tvib = 12.8 ms - the vendor's values.
 */
static void cam_vcm_init(void)
{
	u8 ver = 0;

	if (cam_vcm_ready || !vcm_enable)
		return;
	if (!cam_i2c_get())
		return;

	if (cam_vcm_read8(0x00, &ver))
		pr_warn("cam_cap: VCM at 0x%02x does not answer\n", vcm_addr);

	if (cam_vcm_write8(0x02, 0x01))
		goto fail;
	usleep_range(1000, 1500);		/* DW9768_T_OPR_US */
	if (cam_vcm_write8(0x02, 0x00))
		goto fail;
	if (cam_vcm_write8(0x02, 0x02))
		goto fail;
	if (cam_vcm_write8(0x06, 0x40))
		goto fail;
	if (cam_vcm_write8(0x07, 0x01))
		goto fail;

	cam_vcm_ready = true;
	cam_vcm_pos = 0xffff;			/* force the position write */
	if (cam_vcm_set(af_pos))
		pr_warn("cam_cap: VCM did not accept its first position\n");
	pr_info("cam_cap: VCM 0x%02x ready (id 0x%02x), AAC3 Tvib 12.8ms, position %u\n",
		vcm_addr, ver, cam_vcm_pos);
	return;

fail:
	pr_warn("cam_cap: VCM 0x%02x initialisation failed\n", vcm_addr);
}

/*
 * The search is a coarse sweep followed by two refinement rounds.  Each position
 * costs SKIP + SAMPLES frames: the first frame after a move is discarded because
 * the DAC and the AAC ramp are still on their way, and two measured frames are
 * averaged so that sensor noise cannot pick the winner.
 */
#define CAMCAP_AF_SAMPLES	2
#define CAMCAP_AF_SKIP		1
#define CAMCAP_AF_ROUNDS	2
#define CAMCAP_AF_RESCAN_FRAMES	12	/* low frames before a re-scan */
#define CAMCAP_AF_RESCAN_PCT	70	/* ...and this is what "low" means */

/*
 * Tracking.  The contrast curve of this lens module is shallow - the peak only
 * stands a few percent above the shoulders in a dim room, and in a scene with
 * no texture at all the curve is flat, so the drop-below-70% test above can
 * never fire there.  (It is a good test for a scene change: lights on/off moves
 * the metric far more than 30%.)  What does work is comparing positions
 * back to back, because 750k pixel pairs make the measurement stable to a small
 * fraction of a percent.  So every CAMCAP_AF_WOBBLE_FRAMES frames the search
 * re-measures the held position and one step either side of it, and walks if
 * one of them wins by more than WOBBLE_GAIN_PCT.  The margin is what keeps
 * sensor noise from walking the lens in a flat scene.
 */
#define CAMCAP_AF_WOBBLE_FRAMES	240	/* ~8 s at 30 fps */
#define CAMCAP_AF_WOBBLE_STEP	64
#define CAMCAP_AF_WOBBLE_POINTS	3
#define CAMCAP_AF_WOBBLE_GAIN_PCT 101	/* move only if >1% better */

enum cam_af_state {
	CAM_AF_IDLE = 0,
	CAM_AF_COARSE,
	CAM_AF_FINE,
	CAM_AF_WOBBLE,
	CAM_AF_HOLD,
};

static const char *cam_af_state_name(unsigned int s)
{
	switch (s) {
	case CAM_AF_COARSE:
		return "coarse";
	case CAM_AF_FINE:
		return "fine";
	case CAM_AF_WOBBLE:
		return "wobble";
	case CAM_AF_HOLD:
		return "hold";
	default:
		return "idle";
	}
}

static struct {
	unsigned int	state;
	unsigned int	pos;		/* position being measured now */
	unsigned int	skip;		/* frames to discard after a move */
	u32		acc;		/* metric sum at this position */
	unsigned int	n;		/* samples folded into acc */
	unsigned int	idx;		/* index of the *next* point */
	unsigned int	coarse_n;	/* points in the coarse sweep */
	unsigned int	step;		/* coarse step in force */
	unsigned int	best_pos;
	u32		best_metric;
	u32		hold_metric;	/* metric of the position being held */
	u32		metric_before;	/* hold_metric when the last scan started */
	unsigned int	low;		/* consecutive low-metric frames */
	unsigned int	stubborn;	/* consecutive scans that did not help */
	unsigned int	scans;
	unsigned int	hold_frames;	/* frames since the search last parked */
	unsigned int	wobble_base;	/* position held when a wobble started */
	unsigned int	log_n;
	u32		log_pos[32];
	u32		log_metric[32];
} cam_af;

/*
 * Focus metric, Q8: the mean |dY| over the sampled pixel pairs.  0 when the
 * converter sampled no rows.
 *
 * This is deliberately *not* normalised by the mean luma.  Dividing by the
 * brightness looks attractive - the exposure sets the slope of the LUT, so the
 * raw mean moves when the AE moves (the same position read 547 and 742 in one
 * evening) - but the division quantises the number down to a handful of counts
 * (measured on the device: a flat 7..8 across the whole sweep, with the real
 * peak at 785 against 547 shoulders), which destroys exactly the contrast the
 * search is looking for.  What keeps the sweep comparable instead is the AE:
 * cam_af_scanning() freezes the exposure while the sweep runs, and the IDLE
 * branch of cam_af_step() waits for the AE to settle before it starts.
 */
static u32 cam_af_metric(const struct cam_stats *st)
{
	if (!st->fv_n)
		return 0;
	return (u32)((st->fv * 256) / st->fv_n);
}

/*
 * Mean luma of the same sampled pairs, 8 bit.  Diagnostics only: it is the
 * exposure reference the metric above rides on, so seeing it move tells us when
 * a metric difference between two runs is the AE and not the lens.
 */
static u32 cam_af_luma(const struct cam_stats *st)
{
	if (!st->fv_n)
		return 0;
	return (u32)(st->fv_y / (2 * st->fv_n));
}

/* True while the contrast sweep is running: the exposure must not move. */
static bool cam_af_scanning(void)
{
	if (!vcm_enable || !af_enable || !af_auto)
		return false;
	return cam_af.state == CAM_AF_COARSE || cam_af.state == CAM_AF_FINE ||
	       cam_af.state == CAM_AF_WOBBLE;
}

static unsigned int cam_af_coarse_pos(unsigned int idx)
{
	unsigned int p = af_min + idx * cam_af.step;

	return p > af_max ? af_max : p;
}

/* Start measuring at pos: move there, then discard the transition frames. */
static void cam_af_point(unsigned int pos)
{
	cam_af.pos = pos;
	cam_af.acc = 0;
	cam_af.n = 0;
	cam_af.skip = CAMCAP_AF_SKIP;
	cam_vcm_init();
	cam_vcm_set(pos);
}

static void cam_af_start(void)
{
	unsigned int span = af_max > af_min ? af_max - af_min : 0;

	cam_af.step = af_step ? af_step : 1;
	/* rounded up: the top of the range is always sampled */
	cam_af.coarse_n = (span + cam_af.step - 1) / cam_af.step + 1;
	if (cam_af.coarse_n > ARRAY_SIZE(cam_af.log_pos))
		cam_af.coarse_n = ARRAY_SIZE(cam_af.log_pos);
	cam_af.metric_before = cam_af.hold_metric;
	cam_af.best_metric = 0;
	cam_af.best_pos = cam_af_coarse_pos(0);
	cam_af.idx = 0;
	cam_af.low = 0;
	cam_af.log_n = 0;
	cam_af.scans++;
	cam_af.state = CAM_AF_COARSE;
	cam_af_point(cam_af.best_pos);
}

/* One step either side of base, clamped into the DAC range. */
static unsigned int cam_af_wobble_pos(unsigned int idx)
{
	int p = (int)cam_af.wobble_base +
		((int)idx - (CAMCAP_AF_WOBBLE_POINTS / 2)) * CAMCAP_AF_WOBBLE_STEP;

	if (p < (int)af_min)
		p = (int)af_min;
	if (p > (int)af_max)
		p = (int)af_max;
	return (unsigned int)p;
}

/*
 * Re-measure the held position and its two neighbours.  best_pos/best_metric are
 * reused as the scratch winner of this wobble; cam_af_step() puts the lens back
 * on wobble_base when nothing beats the held metric, so a flat scene cannot walk
 * the lens by noise.
 */
static void cam_af_wobble_start(void)
{
	cam_af.wobble_base = cam_af.best_pos;
	cam_af.metric_before = cam_af.hold_metric;
	cam_af.best_metric = 0;
	cam_af.best_pos = cam_af.wobble_base;
	cam_af.idx = 0;
	cam_af.log_n = 0;
	cam_af.scans++;
	cam_af.state = CAM_AF_WOBBLE;
	cam_af_point(cam_af_wobble_pos(0));
}

/*
 * One frame of the search.  Called from the capture thread right after the
 * governor, i.e. once per frame, with that frame's statistics.
 */
static void cam_af_step(const struct cam_stats *st)
{
	u32 m;
	unsigned int need;

	if (!vcm_enable || !af_enable)
		return;
	if (!cam_vcm_ready)
		cam_vcm_init();
	if (!cam_vcm_ready)
		return;

	if (!af_auto) {
		/* a manual position, set through /proc or V4L2 */
		if (cam_vcm_pos != af_pos)
			cam_vcm_set(af_pos);
		return;
	}

	if (!st->fv_n)
		return;		/* this converter path has no focus metric */
	m = cam_af_metric(st);

	if (cam_af.state == CAM_AF_IDLE) {
		/*
		 * Let the exposure land first.  The AE steps at most every
		 * CAMCAP_AE_SETTLE frames, so a scan started on a frame where
		 * the AE did move would measure its first points at the old
		 * exposure and the rest at the new one.
		 */
		if (cam_ae_auto && cam_ae_settle)
			return;
		if (!cam_af.scans)
			cam_af_start();
		return;
	}

	if (cam_af.state == CAM_AF_HOLD) {
		cam_af.hold_metric = m;
		if (cam_af.hold_frames < CAMCAP_AF_WOBBLE_FRAMES)
			cam_af.hold_frames++;
		if (cam_af.hold_frames >= CAMCAP_AF_WOBBLE_FRAMES &&
		    cam_af.best_metric) {
			cam_af.hold_frames = 0;
			cam_af_wobble_start();
			return;
		}
		need = CAMCAP_AF_RESCAN_FRAMES <<
		       (cam_af.stubborn < 3 ? cam_af.stubborn : 3);
		if (cam_af.best_metric &&
		    m * 100 < cam_af.best_metric * CAMCAP_AF_RESCAN_PCT) {
			if (++cam_af.low >= need) {
				cam_af.low = 0;
				cam_af_start();
			}
		} else {
			cam_af.low = 0;
		}
		return;
	}

	/* CAM_AF_COARSE / CAM_AF_FINE: collect this position's samples */
	if (cam_af.skip) {
		cam_af.skip--;
		return;
	}
	cam_af.acc += m;
	if (++cam_af.n < CAMCAP_AF_SAMPLES)
		return;

	m = cam_af.acc / cam_af.n;

	if (cam_af.log_n < ARRAY_SIZE(cam_af.log_pos)) {
		cam_af.log_pos[cam_af.log_n] = cam_af.pos;
		cam_af.log_metric[cam_af.log_n] = m;
		cam_af.log_n++;
	}
	if (af_trace)
		pr_info("cam_af: %s pos=%u metric=%u (best %u @ %u)\n",
			cam_af_state_name(cam_af.state), cam_af.pos, m,
			cam_af.best_metric, cam_af.best_pos);

	if (m > cam_af.best_metric) {
		cam_af.best_metric = m;
		cam_af.best_pos = cam_af.pos;
	}

	if (cam_af.state == CAM_AF_WOBBLE) {
		cam_af.idx++;
		if (cam_af.idx < CAMCAP_AF_WOBBLE_POINTS) {
			cam_af_point(cam_af_wobble_pos(cam_af.idx));
			return;
		}
		cam_af.state = CAM_AF_HOLD;
		cam_af.hold_frames = 0;
		if (cam_af.best_metric * 100 >=
		    cam_af.metric_before * CAMCAP_AF_WOBBLE_GAIN_PCT) {
			cam_af.hold_metric = cam_af.best_metric;
			cam_vcm_set(cam_af.best_pos);
			pr_info("cam_af: wobble -> pos=%u metric=%u (was %u @ %u)\n",
				cam_af.best_pos, cam_af.best_metric,
				cam_af.metric_before, cam_af.wobble_base);
		} else {
			u32 won = cam_af.best_metric;
			unsigned int won_pos = cam_af.best_pos;

			cam_af.best_pos = cam_af.wobble_base;
			cam_af.best_metric = cam_af.metric_before;
			cam_af.hold_metric = cam_af.metric_before;
			cam_vcm_set(cam_af.wobble_base);
			if (af_trace)
				pr_info("cam_af: wobble held pos=%u metric=%u (best %u @ %u)\n",
					cam_af.wobble_base,
					cam_af.metric_before, won, won_pos);
		}
		return;
	}

	if (cam_af.state == CAM_AF_COARSE) {
		cam_af.idx++;
		if (cam_af.idx < cam_af.coarse_n) {
			cam_af_point(cam_af_coarse_pos(cam_af.idx));
			return;
		}
		cam_af.state = CAM_AF_FINE;
		cam_af.idx = 0;
	}

	/* CAM_AF_FINE: twice a shrinking bracket around the winner */
	if (cam_af.idx < 2 * CAMCAP_AF_ROUNDS) {
		unsigned int r = cam_af.idx >> 1;
		unsigned int s = cam_af.step >> (r + 1);
		int p;

		if (!s)
			s = 1;
		p = (int)cam_af.best_pos + ((cam_af.idx & 1) ? (int)s : -(int)s);
		if (p < (int)af_min)
			p = (int)af_min;
		if (p > (int)af_max)
			p = (int)af_max;
		cam_af.idx++;
		cam_af_point((unsigned int)p);
		return;
	}

	/* done: park on the winner and start watching for a scene change */
	cam_af.state = CAM_AF_HOLD;
	cam_af.hold_frames = 0;
	if (cam_af.metric_before * 110 < cam_af.best_metric * 100)
		cam_af.stubborn = 0;
	else if (cam_af.stubborn < 3)
		cam_af.stubborn++;
	cam_af.low = 0;
	cam_af.hold_metric = cam_af.best_metric;
	cam_vcm_set(cam_af.best_pos);
	pr_info("cam_af: scan %u done, pos=%u metric=%u (was %u), stubborn=%u\n",
		cam_af.scans, cam_af.best_pos, cam_af.best_metric,
		cam_af.metric_before, cam_af.stubborn);
}

/* Forget the search state; called whenever a stream starts. */
static void cam_af_reset(void)
{
	cam_af.state = CAM_AF_IDLE;
	cam_af.scans = 0;
	cam_af.low = 0;
	cam_af.stubborn = 0;
	cam_af.best_metric = 0;
	cam_af.hold_metric = 0;
	cam_af.metric_before = 0;
	cam_af.n = 0;
	cam_af.skip = 0;
	cam_af.hold_frames = 0;
}

/*
 * Keep the V4L2 focus controls showing what was just changed through /proc, so
 * the two interfaces cannot disagree.  Called from the /proc path, which does
 * not hold the control handler lock -- unlike the s_ctrl callbacks themselves.
 */
static void cam_af_sync_ctrls(void)
{
	if (!cam_v4l2)
		return;
	if (cam_v4l2->ct_focus_auto)
		v4l2_ctrl_s_ctrl(cam_v4l2->ct_focus_auto, af_auto);
	if (cam_v4l2->ct_focus)
		v4l2_ctrl_s_ctrl(cam_v4l2->ct_focus, af_pos);
}

/* /proc/camcap "af ..." command.  Returns 0 or a negative errno. */
static int cam_af_cmd(const char *line)
{
	const char *a = line + 2;

	while (*a == ' ')
		a++;

	if (!*a || !strcmp(a, "show")) {
		pr_info("cam_cap: af %s (%s) state=%s pos=%u best=%u metric=%u hold=%u low=%u stubborn=%u scans=%u vcm@0x%02x\n",
			af_enable ? (af_auto ? "auto" : "manual") : "off",
			cam_vcm_ready ? "vcm ready" : "vcm idle",
			cam_af_state_name(cam_af.state), cam_vcm_pos,
			cam_af.best_pos, cam_af.best_metric, cam_af.hold_metric,
			cam_af.low, cam_af.stubborn, cam_af.scans, vcm_addr);
		return 0;
	}
	if (!strcmp(a, "on") || !strcmp(a, "auto")) {
		af_enable = 1;
		af_auto = true;
		cam_af_reset();
		cam_af_sync_ctrls();
		return 0;
	}
	if (!strcmp(a, "off")) {
		af_enable = 0;
		cam_af.state = CAM_AF_IDLE;
		cam_af_sync_ctrls();
		return 0;
	}
	if (!strcmp(a, "hold")) {
		af_auto = false;
		cam_af.state = CAM_AF_IDLE;
		cam_af_sync_ctrls();
		return 0;
	}
	if (!strcmp(a, "init")) {
		cam_vcm_init();
		return cam_vcm_ready ? 0 : -EIO;
	}
	if (!strcmp(a, "scan")) {
		af_enable = 1;
		af_auto = true;
		cam_vcm_init();
		cam_af_start();
		cam_af_sync_ctrls();
		return 0;
	}
	if (!strcmp(a, "wobble")) {
		/* force the tracking step instead of waiting for the timer */
		af_enable = 1;
		af_auto = true;
		cam_vcm_init();
		if (cam_af.best_metric)
			cam_af_wobble_start();
		else
			cam_af_start();
		cam_af_sync_ctrls();
		return 0;
	}
	if (!strncmp(a, "pos", 3)) {
		unsigned int v;
		int ret;

		a += 3;
		while (*a == ' ')
			a++;
		if (kstrtouint(a, 10, &v) || v > 1023)
			return -EINVAL;
		af_auto = false;
		af_enable = 1;
		af_pos = v;
		cam_af.state = CAM_AF_IDLE;
		cam_vcm_init();
		ret = cam_vcm_ready ? cam_vcm_set(v) : -EIO;
		cam_af_sync_ctrls();
		pr_info("cam_cap: af manual position %u (%s)\n", v,
			ret ? "failed" : "applied");
		return ret;
	}

	return -EINVAL;
}

/*
 * The af line of /proc/camcap_info.  Written from here because the search state
 * and the VCM are private to this section; the reader above only forwards to it.
 */
static int cam_af_info(char *info, int i, size_t size)
{
	/*
	 * metric is the *live* contrast of the frame that was just converted --
	 * what a manual position sweep has to read.  best/best_pos/hold are the
	 * search's own bookkeeping.
	 */
	i += scnprintf(info + i, size - i,
		       "af           : %s state=%s pos=%u metric=%u y=%u best=%u best_pos=%u hold=%u low=%u stubborn=%u scans=%u vcm=0x%02x%s\n",
		       !af_enable ? "off" : (af_auto ? "auto" : "manual"),
		       cam_af_state_name(cam_af.state), cam_vcm_pos,
		       cam_af_metric(&cam_stats), cam_af_luma(&cam_stats),
		       cam_af.best_metric, cam_af.best_pos, cam_af.hold_metric,
		       cam_af.low, cam_af.stubborn, cam_af.scans, vcm_addr,
		       cam_vcm_ready ? " ready" : " (not ready)");
	return i;
}

/* ------------------------------------------------------------------ */
/* AE / AWB governor                                                  */
/* ------------------------------------------------------------------ */

#define CAMCAP_EXP_MIN		0x0010
#define CAMCAP_AGAIN_MIN	0x0100
#define CAMCAP_DGAIN_MIN	0x0100
#define CAMCAP_WB_MIN		128	/* Q8: 0.5x  */
#define CAMCAP_WB_MAX		1024	/* Q8: 4.0x  */

static int cam_clamp_int(int v, int lo, int hi)
{
	return v < lo ? lo : (v > hi ? hi : v);
}

/* v * num / den, rounded */
static unsigned int cam_scale(unsigned int v, unsigned int num, unsigned int den)
{
	return (unsigned int)(((u64)v * num + den / 2) / den);
}

static const char *cam_ae_state(void)
{
	return cam_ae_auto ? "auto" : "manual";
}

/*
 * One bounded AE step, in one direction only.
 *
 * Going up the chain costs image quality in a specific order: integration time
 * is free, analogue gain adds noise, digital gain adds noise without even
 * improving the analogue signal.  So raise exposure first and digital gain
 * last, and undo in the exact reverse order.
 *
 * The size of the step is proportional to the error, measured on the
 * black-corrected signal, and capped per knob:
 *
 *   - the coarse integration time is linear in the collected signal, so it may
 *     take a 2x step;
 *   - the gain registers are not linear (0x0204's code-to-gain curve is steep
 *     near the top), so they take at most a 1.25x step and the loop just walks
 *     in.  Fixed 1.6x steps used to overshoot the dead band and hunt forever.
 *
 * The requested multiplier is the geometric *half* step, sqrt(tgt/sig), and the
 * up and down clamps are exact reciprocals of each other (384/256 vs 256/171).
 * That symmetry is what stops the loop from oscillating: any overshoot is worth
 * one step in the opposite direction, whatever its size, so the loop cannot end
 * up bouncing between "too bright" and "normal".  Taking the whole error each
 * time - 2x up but only -20% down - needed 3.1 down steps to pay for one upward
 * one, which is exactly the slow breathing this loop was reported for.
 *
 * Inside [lo, hi] nothing moves at all; clipping is reported but by itself
 * never moves the loop.  That matters because on this sensor one analogue gain
 * step is worth far more light than one dead-band width (0x032c -> mean 783,
 * 0x03a0 -> mean 1296): treating every blown highlight as an emergency made the
 * loop alternate between the darkest and the brightest point of the band.
 *
 * Returns true when the sensor registers were changed.
 */
#define CAMCAP_AE_UP_MAX	384	/* Q8: 1.50x */
#define CAMCAP_AE_GAIN_MAX	320	/* Q8: 1.25x, the gain curves are steep */
#define CAMCAP_AE_DOWN_MIN	171	/* Q8: 256/1.5: one step undoes one up step */

static bool cam_ae_step(unsigned int mean_g, unsigned int clip_pct)
{
	unsigned int lo = ae_target > ae_band ? ae_target - ae_band : 0;
	unsigned int hi = ae_target + ae_band;
	struct cam_sensor_state s = cam_sensor;
	unsigned int sig, tgt, f;
	bool up;

	if (mean_g > lo && mean_g < hi)
		return false;			/* inside the dead band */
	/* the pedestal is ~248 of a ~385 mean here: correct before comparing */
	sig = mean_g > v4l2_black ? mean_g - v4l2_black : 1;
	tgt = ae_target > v4l2_black ? ae_target - v4l2_black : 1;
	f = (unsigned int)(((u64)tgt << 8) / sig);	/* wanted multiplier, Q8 */

	up = mean_g < lo;
	if (!up && clip_pct > ae_clip_pct) {
		/*
		 * Too bright and blowing out: back off harder the more of the
		 * frame is clipped.  Clipping on its own never moves the loop -
		 * only a mean outside the band does - because a small clipped
		 * highlight is normal while the mean is on target.
		 */
		unsigned int cut = clip_pct * 8;

		if (cut > 64)
			cut = 64;
		f = 256 - cut;
	}

	/*
	 * Geometric half step: sqrt(f/256) * 256.  One correction removes half
	 * the error in log space, so the next one cancels an overshoot instead
	 * of compounding it.  int_sqrt() is exact and costs a few tens of
	 * cycles once per frame.
	 */
	f = (unsigned int)int_sqrt((u64)f << 8);
	if (f < 1)
		f = 1;

	if (up) {
		if (f < 256)
			f = 256;
		if (s.exposure < exp_max) {
			unsigned int g = f > CAMCAP_AE_UP_MAX ? CAMCAP_AE_UP_MAX : f;

			s.exposure = cam_scale(s.exposure, g, 256);
			if (s.exposure > exp_max)
				s.exposure = exp_max;
		} else if (s.again < again_max) {
			unsigned int g = f > CAMCAP_AE_GAIN_MAX ? CAMCAP_AE_GAIN_MAX : f;

			s.again = cam_scale(s.again, g, 256);
			if (s.again > again_max)
				s.again = again_max;
		} else if (s.dgain < dgain_max) {
			unsigned int g = f > CAMCAP_AE_GAIN_MAX ? CAMCAP_AE_GAIN_MAX : f;

			s.dgain = cam_scale(s.dgain, g, 256);
			if (s.dgain > dgain_max)
				s.dgain = dgain_max;
		} else {
			return false;	/* maxed out: the scene is simply dim */
		}
	} else {
		if (f > 256)
			f = 256;
		if (f < CAMCAP_AE_DOWN_MIN)
			f = CAMCAP_AE_DOWN_MIN;	/* reciprocal of the up clamp */
		if (s.dgain > CAMCAP_DGAIN_MIN) {
			s.dgain = cam_scale(s.dgain, f, 256);
			if (s.dgain < CAMCAP_DGAIN_MIN)
				s.dgain = CAMCAP_DGAIN_MIN;
		} else if (s.again > CAMCAP_AGAIN_MIN) {
			s.again = cam_scale(s.again, f, 256);
			if (s.again < CAMCAP_AGAIN_MIN)
				s.again = CAMCAP_AGAIN_MIN;
		} else if (s.exposure > CAMCAP_EXP_MIN) {
			s.exposure = cam_scale(s.exposure, f, 256);
			if (s.exposure < CAMCAP_EXP_MIN)
				s.exposure = CAMCAP_EXP_MIN;
		} else {
			return false;	/* already as dark as it goes */
		}
	}

	if (s.exposure == cam_sensor.exposure &&
	    s.again == cam_sensor.again &&
	    s.dgain == cam_sensor.dgain)
		return false;

	if (ae_trace)
		pr_info("cam_ae: mean=%u clip=%u sig=%u tgt=%u f=%u up=%u exp 0x%04x->0x%04x again 0x%04x->0x%04x dgain 0x%04x->0x%04x\n",
			mean_g, clip_pct, sig, tgt, f, up,
			cam_sensor.exposure, s.exposure,
			cam_sensor.again, s.again,
			cam_sensor.dgain, s.dgain);

	cam_sensor = s;
	cam_ae_frames++;
	return true;
}

/*
 * Run both loops on the frame that was just captured.  Called from the capture
 * thread, because this is the only context allowed to sleep on I2C and the
 * only one that may take the control handler lock without deadlocking.
 *
 * The AE is deliberately slow: a sensor register write is only guaranteed to
 * be in effect for a frame that starts integrating after the write, and the
 * frame period here is long (the integration time alone can be 12288 lines).
 * Correcting on every single frame therefore controls a plant that is still
 * one frame behind, which is exactly how a bang-bang loop ends up alternating
 * between "too dark" and "too bright".  After every change the loop now waits
 * cam_ae_settle frames, so each correction is based on a frame the sensor has
 * actually settled into.
 */
#define CAMCAP_AE_SETTLE	3

/*
 * The brightness the loop steers on is an IIR of the frame means, not the last
 * frame's mean.  A single frame of a hand-held scene, or of a display being
 * photographed, is a poor estimate of the scene brightness: the frame that
 * happens to catch a dark patch dragged the whole loop with it and the picture
 * visibly breathed.  Shift 2 = 25% of each frame, so the loop still reaches a
 * new scene in a handful of frames.
 */
#define CAMCAP_AE_SMOOTH_SHIFT	2

/*
 * One governor step per frame.  The statistics arrive from the conversion pass
 * (they are a by-product of it now, see cam_v4l2_convert_band()), which removed
 * a whole second memory pass over the 18 MB raw frame.
 */
static void cam_isp_governor(const struct cam_stats *in)
{
	struct cam_stats st = *in;
	unsigned int clip_pct;

	cam_stats = st;
	cam_stat_mean_r = st.mean_r;
	cam_stat_mean_g = st.mean_g;
	cam_stat_mean_b = st.mean_b;
	clip_pct = st.count ? (unsigned int)(st.clip * 100 / (st.count * 2)) : 0;
	cam_stat_clip_pct = clip_pct;

	if (!st.count)
		return;

	/* grey world: drive the corrected channel means towards each other */
	if (cam_awb_auto && st.mean_g >= awb_min_level &&
	    st.mean_r && st.mean_b) {
		unsigned int shift = awb_rate > 8 ? 8 : awb_rate;
		unsigned int div = 1u << shift;
		/*
		 * Grey world.  Compare the *black-corrected* channel means and
		 * solve for the gains that would make them equal:
		 *
		 *     wb_r = 256 * (G - black) / (R - black)
		 *     wb_b = 256 * (G - black) / (B - black)
		 *
		 * Subtracting the pedestal first matters enormously here: on this
		 * sensor the black level is around 248 of a ~385 green mean, so a
		 * target computed from the raw means is off by more than 2x.
		 * Then walk a fraction of the remaining error, which converges
		 * geometrically instead of overshooting.
		 */
		int sg_r = (int)st.mean_r - (int)v4l2_black;
		int sg_g = (int)st.mean_g - (int)v4l2_black;
		int sg_b = (int)st.mean_b - (int)v4l2_black;
		unsigned int tr, tb;

		if (sg_r < 1)
			sg_r = 1;
		if (sg_g < 1)
			sg_g = 1;
		if (sg_b < 1)
			sg_b = 1;

		tr = (unsigned int)(((u64)sg_g << 8) / (unsigned int)sg_r);
		tb = (unsigned int)(((u64)sg_g << 8) / (unsigned int)sg_b);
		int dr = (int)tr - (int)cam_wb_r_cur;
		int db = (int)tb - (int)cam_wb_b_cur;

		/* a fraction of the remaining error, at least one step */
		dr /= (int)div;
		db /= (int)div;
		if (!dr && tr != cam_wb_r_cur)
			dr = tr > cam_wb_r_cur ? 1 : -1;
		if (!db && tb != cam_wb_b_cur)
			db = tb > cam_wb_b_cur ? 1 : -1;

		cam_wb_r_cur = cam_clamp_int((int)cam_wb_r_cur + dr,
					     CAMCAP_WB_MIN, CAMCAP_WB_MAX);
		cam_wb_b_cur = cam_clamp_int((int)cam_wb_b_cur + db,
					     CAMCAP_WB_MIN, CAMCAP_WB_MAX);

		if (dr || db) {
			cam_awb_frames++;
			cam_lut_dirty = true;
		}

		/* keep userspace's view of the manual controls accurate */
		if (cam_v4l2 && cam_v4l2->ct_red)
			v4l2_ctrl_s_ctrl(cam_v4l2->ct_red, cam_wb_r_cur);
		if (cam_v4l2 && cam_v4l2->ct_blue)
			v4l2_ctrl_s_ctrl(cam_v4l2->ct_blue, cam_wb_b_cur);
	}

	/*
	 * exposure.  Two changes over the plain proportional loop:
	 *
	 *  - the measurement is an IIR of the green mean, not the last frame's
	 *    mean.  One frame of a hand-held scene (or of a screen) is a poor
	 *    brightness estimate, and steering a proportional loop with that
	 *    noise is what made the picture breathe;
	 *  - the loop is frozen while the autofocus sweeps, because the contrast
	 *    metric it compares across lens positions is only meaningful if the
	 *    exposure is not moving underneath it.
	 */
	if (cam_ae_auto) {
		bool frozen = cam_af_scanning();

		if (!cam_ae_mean_s)
			cam_ae_mean_s = st.mean_g;
		else
			cam_ae_mean_s += ((int)st.mean_g - (int)cam_ae_mean_s +
					  (1 << (CAMCAP_AE_SMOOTH_SHIFT - 1))) >>
					 CAMCAP_AE_SMOOTH_SHIFT;

		if (frozen) {
			/* hold: the AF needs a stable scene */
		} else if (cam_ae_settle) {
			/*
			 * The sensor is still catching up with the last change:
			 * measuring now would compare a stale frame against the
			 * new registers, which is what used to make this loop
			 * alternate between too dark and too bright.
			 */
			cam_ae_settle--;
		} else if (cam_ae_step(cam_ae_mean_s, clip_pct)) {
			cam_ae_settle = CAMCAP_AE_SETTLE;
		}
		cam_sensor_apply();

		if (cam_v4l2 && cam_v4l2->ct_exposure)
			v4l2_ctrl_s_ctrl(cam_v4l2->ct_exposure,
					 cam_sensor.exposure);
		if (cam_v4l2 && cam_v4l2->ct_again)
			v4l2_ctrl_s_ctrl(cam_v4l2->ct_again, cam_sensor.again);
		if (cam_v4l2 && cam_v4l2->ct_dgain)
			v4l2_ctrl_s_ctrl(cam_v4l2->ct_dgain, cam_sensor.dgain);
	} else {
		cam_ae_mean_s = 0;
	}

	cam_af_step(&st);
}

/* One byte of YUV, clamped: chroma scaling can push U/V out of range. */
static inline u8 cam_u8_clamp(int v)
{
	return v < 0 ? 0 : (v > 255 ? 255 : (u8)v);
}

/*
 * 2x2 binning + RGGB debayer + gain/white balance + tone curve + BT.601 YUYV,
 * with the 180 degree rotation fused into the write.
 *
 * Bayer phase is RGGB: R on (even line, even column), B on (odd, odd),
 * G on both mixed positions.  The sensor is read out 180 degrees rotated
 * relative to the physical scene, so each row is written to its mirrored
 * destination row and each 4-byte group to its mirrored group, instead of
 * converting first and rotating the finished 6 MB image afterwards.  Fusing it
 * removes a whole pass over the frame and keeps the result byte for byte what
 * the old cam_yuyv_flip() produced:
 *
 *     dst row    = H - 1 - y                (rows mirrored)
 *     dst group  = n - 1 - pair             (groups mirrored)
 *     dst bytes  = [ly1, v, ly0, u]         (luma swapped, U/V in place)
 *
 * Converting from the raw Bayer grid to the flipped output in one step is also
 * the only correct order: rotating the raw Bayer grid itself would quietly turn
 * RGGB into BGGR.
 *
 * [first, last) is a row range so several threads can share the frame: rows are
 * independent, every row costs the same, and each row's destination is derived
 * from that same row index, so a static split needs no locking at all.
 */
/*
 * Convert the rows [first, last) of *src (the cacheable view of a raw frame
 * slot) into *dst and accumulate the scene statistics of the rows sampled.
 *
 * The measurement used to be a second pass over the raw frame (cam_raw_stats(),
 * a 1-in-64 sample of it).  Every tap this loop already loads is exactly the tap
 * that pass sampled, so the sums are folded in here for free: the loop pays two
 * adds per tap extra and the governor loses its whole memory pass (measured at
 * 55-66 ms per frame, i.e. ~25% of the frame period).  The sums are per worker,
 * accumulated in u64, and merged by cam_convert_frame().
 *
 * The accumulate block is not free though -- it was ~20 of the ~85 instructions
 * per pixel pair, and with the converter on the critical path of a 30 fps
 * pipeline that is 15% of the frame period for statistics nobody looks at.
 * Only every CAMCAP_STATS_ROW_STEP'th row is measured now: still 192x the
 * sample count of the old pass, and the branch is constant for a whole row.
 */
/*
 * Tone tables, colour matrix and the YUYV byte order -- in one place, because
 * more than one converter stores into YUYV and they must not drift apart.
 *
 * YUYV is Y0 Cb Y1 Cr: byte 1 always carries Cb and byte 3 always Cr, whichever
 * of the pair's two luma bytes was written first.  The luma order is reversed to
 * undo the 180 degree flip; the chroma slots must *not* follow it, because a
 * pair has exactly one Cb and one Cr.  Cr in byte 1 is what "reds look blue,
 * cyan comes out gold" was: a pure Cb/Cr exchange, which the grey-world AWB
 * cannot see because it mirrors every gain.
 *
 * r/g/b are raw 12-bit samples.  Averages have to happen before the tables,
 * because the tables end in a square-root gamma: averaging after them would
 * lift the peaks.
 */
static inline void cam_yuyv_pair(u8 *q, u32 r0, u32 g0, u32 b0,
				 u32 r1, u32 g1, u32 b1, int sat,
				 const u8 *lut_ee, const u8 *lut_oo)
{
	const int ya = rb_swap ? 29 : 77;	/* (0,0) tap -> luma */
	const int yb = rb_swap ? 77 : 29;	/* (1,1) tap -> luma */
	const int ua = rb_swap ? 128 : -43;
	const int ub = rb_swap ? -43 : 128;
	const int va = rb_swap ? -21 : 128;
	const int vb = rb_swap ? 128 : -21;
	u8 a0 = lut_ee[r0], d0 = lut_oo[b0], gg0 = cam_lut_g[g0];
	u8 a1 = lut_ee[r1], d1 = lut_oo[b1], gg1 = cam_lut_g[g1];
	int ly0 = (ya * a0 + 150 * gg0 + yb * d0) >> 8;
	int ly1 = (ya * a1 + 150 * gg1 + yb * d1) >> 8;
	/*
	 * YUYV carries one Cb and one Cr for the pair, and the colour matrix is
	 * linear, so the chroma of the averaged tap triple is the average of the
	 * two pixels' chromas.  Doing it on the summed taps costs three multiplies
	 * per pair instead of six, and differs from the per-pixel version only in
	 * the rounding of the individual shifts (at most one LSB).
	 */
	int ar = a0 + a1, ag = gg0 + gg1, ab = d0 + d1;
	int u = ((ua * ar - 85 * ag + ub * ab) >> 9) + 128;
	int v = ((va * ar - 107 * ag + vb * ab) >> 9) + 128;

	/* chroma gain around 128, then clamp to byte range */
	if (sat != 128) {
		u = ((u - 128) * sat) / 128 + 128;
		v = ((v - 128) * sat) / 128 + 128;
	}

	q[0] = cam_u8_clamp(ly1);
	q[1] = cam_u8_clamp(u);
	q[2] = cam_u8_clamp(ly0);
	q[3] = cam_u8_clamp(v);
}

/*
 * Grey-world luma of one raw tap triple, 0..255, for the focus metric.  The
 * metric is only ever compared between frames of the same mode, and this is
 * monotone in the LUT'd luma, so it orders lens positions the same way without
 * a second pair of table lookups.
 */
static inline int cam_luma8(u32 r, u32 g, u32 b)
{
	const int ya = rb_swap ? 29 : 77;
	const int yb = rb_swap ? 77 : 29;

	return cam_clamp_int((ya * (int)r + 150 * (int)g + yb * (int)b) >> 8,
			     0, 255);
}

/*
 * Full-size output (v4l2_bin=1): one output pixel per raw pixel, bilinear.
 *
 * The 2x2 path only ever reads the two non-green taps of a block, so it can
 * afford to throw three quarters of the frame away.  This one reads nine taps
 * per pixel, i.e. about four times the work for four times the pixels -- worth
 * it where the sensor mode already is what we want to record: replaying the
 * 1920x1080 modes and letting the converter halve them again produced 960x540.
 *
 * Only valid for the *binned* sensor modes (0x0900=1), where the sensor hands
 * out one sample per logical Bayer pixel: preview 4000x3000, normal_video and
 * custom3 4000x2256, custom2 and hs_video 1920x1080.  The unbinned modes
 * (custom4 8000x6000, custom5 4000x3000, both 0x0900=0) put one *photosite*
 * per pixel, so their Bayer period is 2 and the 2x2 path is the one that is
 * correct there.
 *
 * Row y of the output is raw row y, with the flip done by writing the rows in
 * reverse, so a row only needs its two vertical neighbours.
 */
static void cam_v4l2_convert_full_ref(const u8 *src, u8 *dst, unsigned int first,
				      unsigned int last, struct cam_stats *acc)
{
	const unsigned int W = out_width;	/* == cam_src_w */
	const unsigned int H = out_height;	/* == cam_src_h */
	const unsigned int n = W / 2;		/* 4-byte YUYV groups per row */
	const unsigned int stride = cam_src_stride;
	const unsigned int rw = cam_src_w;
	const unsigned int rh = cam_src_h;
	int sat = cam_clamp_int(out_saturation, 0, 255);
	unsigned int y, x;
	u64 sum_r = 0, sum_g = 0, sum_b = 0;
	unsigned int count = 0, clip = 0, dark = 0;
	u32 min_g = 0xffff, max_g = 0;
	u64 fv = 0, fv_n = 0, fv_y = 0;		/* focus metric, see cam_stats */
	/* same tap-to-colour switch as the 2x2 path, see there */
	const u8 *lut_ee = rb_swap ? cam_lut_b : cam_lut_r;	/* (0,0) tap */
	const u8 *lut_oo = rb_swap ? cam_lut_r : cam_lut_b;	/* (1,1) tap */

	for (y = first; y < last; y++) {
		const u8 *l0 = src + (size_t)y * stride;
		const u8 *lm = src + (size_t)(y ? y - 1 : 0) * stride;
		const u8 *lp = src + (size_t)(y + 1 < rh ? y + 1 : y) * stride;
		u8 *o = dst + (size_t)(H - 1 - y) * W * 2;
		/*
		 * Measure on row *pairs*.  A raw row is either R/Gr or Gb/B,
		 * so gating on y alone samples one parity only: the red sum
		 * fills up while the blue one stays at zero, and measured
		 * mean_g comes out at half its value.  (y >> 1) keeps whole
		 * pairs together.
		 */
		bool st_on = !((y >> 1) & (CAMCAP_STATS_ROW_STEP - 1));

		for (x = 0; x + 1 < W; x += 2) {
			u32 r0, g0, b0, r1, g1, b1;
			u8 *q = o + (size_t)(n - 1 - (x >> 1)) * 4;

			cam_bayer_px(lm, l0, lp, x, y, rw, &r0, &g0, &b0);
			cam_bayer_px(lm, l0, lp, x + 1, y, rw, &r1, &g1, &b1);
			cam_yuyv_pair(q, r0, g0, b0, r1, g1, b1, sat,
				      lut_ee, lut_oo);

			if (!st_on)
				continue;
			/*
			 * Statistics come from the tap the pixel owns, not from
			 * the interpolated value: those are the samples the 2x2
			 * path measured, so mean_r:mean_g:mean_b -- and the
			 * whole AWB with it -- do not move when the binning
			 * does.  count is the red (and blue) sample count,
			 * which is what cam_stats_finish() doubles for green,
			 * and it only advances on the rows that hold a red
			 * sample, or the two would drift apart.
			 */
			if (!(y & 1)) {			/* even row: R, Gr */
				u32 gg = g1;

				if (rb_swap)
					sum_b += r0;
				else
					sum_r += r0;
				sum_g += gg;
				count++;
				clip += (gg >= CAMCAP_CLIP_LEVEL);
				dark += (gg <= v4l2_black);
				if (gg < min_g)
					min_g = gg;
				if (gg > max_g)
					max_g = gg;
			} else {			/* odd row: Gb, B */
				u32 gg = g0;

				if (rb_swap)
					sum_r += b1;
				else
					sum_b += b1;
				sum_g += gg;
				clip += (gg >= CAMCAP_CLIP_LEVEL);
				dark += (gg <= v4l2_black);
				if (gg < min_g)
					min_g = gg;
				if (gg > max_g)
					max_g = gg;
			}

			/* focus metric: |dY| between the two output pixels */
			{
				int c0 = cam_luma8(r0, g0, b0);
				int c1 = cam_luma8(r1, g1, b1);

				fv += (u32)(c0 > c1 ? c0 - c1 : c1 - c0);
				fv_y += (u64)c0 + (u64)c1;
				fv_n++;
			}
		}
	}

	acc->sum_r += sum_r;
	acc->sum_g += sum_g;
	acc->sum_b += sum_b;
	acc->count += count;
	acc->clip += clip;
	acc->dark += dark;
	acc->fv += fv;
	acc->fv_n += fv_n;
	acc->fv_y += fv_y;
	if (count) {
		if (min_g < acc->min_g)
			acc->min_g = min_g;
		if (max_g > acc->max_g)
			acc->max_g = max_g;
	}
}

/* Expand one raw 12-bit row (1.5 bytes per pixel) into u16 samples. */
static void cam_unpack_row(const u8 *line, u16 *out, unsigned int n)
{
	unsigned int i;

	for (i = 0; i + 1 < n; i += 2) {
		const u8 *p = line + (size_t)(i >> 1) * 3;

		out[i] = (u16)p[0] | ((u16)(p[1] & 0x0f) << 8);
		out[i + 1] = (u16)(p[1] >> 4) | ((u16)p[2] << 4);
	}
	if (n & 1)
		out[n - 1] = cam_px_at(line, n - 1);
}

/*
 * The same, but into a row padded with one duplicated sample at each end, so
 * that the bilinear taps of the inner loop can index x-1 .. x+3 unconditionally.
 * The row has to be laid out at out[1..n], i.e. the caller passes the buffer
 * base and the loop reads column x at index x + 1.
 */
static inline void cam_unpack_row_pad(const u8 *line, u16 *out, unsigned int n)
{
	cam_unpack_row(line, out + 1, n);
	out[0] = out[1];
	out[n + 1] = out[n];
}

/*
 * The same full-size bilinear conversion as the reference above, with the raw
 * rows expanded once into u16 scratch rows instead of decoded again for every
 * neighbour.  The bilinear taps overlap so heavily that the per-pixel version
 * unpacks the same three bytes about eight times per output pixel, and it is
 * that unpacking -- load three bytes, shift, mask, or -- that the conversion is
 * made of, not the arithmetic.
 *
 * Three rows stay live (y-1, y, y+1) and rotate, so each raw row is unpacked
 * once per band.  The window belongs to the call, i.e. to one worker band, and
 * a refused allocation falls back to the per-pixel reference path.
 */
static void cam_v4l2_convert_full_fast(const u8 *src, u8 *dst,
				       unsigned int first, unsigned int last,
				       struct cam_stats *acc)
{
	const unsigned int W = out_width;
	const unsigned int H = out_height;
	const unsigned int n = W / 2;
	const unsigned int stride = cam_src_stride;
	const unsigned int rw = cam_src_w;
	const unsigned int rh = cam_src_h;
	int sat = cam_clamp_int(out_saturation, 0, 255);
	u16 *rows, *lm, *l0, *lp, *tmp;
	unsigned int y, x;
	u64 sum_r = 0, sum_g = 0, sum_b = 0;
	unsigned int count = 0, clip = 0, dark = 0;
	u32 min_g = 0xffff, max_g = 0;
	u64 fv = 0, fv_n = 0, fv_y = 0;		/* focus metric, see cam_stats */
	/* same tap-to-colour switch as the 2x2 path, see there */
	const u8 *lut_ee = rb_swap ? cam_lut_b : cam_lut_r;	/* (0,0) tap */
	const u8 *lut_oo = rb_swap ? cam_lut_r : cam_lut_b;	/* (1,1) tap */

	rows = kmalloc_array(3 * (size_t)(rw + 2), sizeof(u16), GFP_KERNEL);
	if (!rows) {
		cam_v4l2_convert_full_ref(src, dst, first, last, acc);
		return;
	}

	lm = rows;
	l0 = rows + rw + 2;
	lp = rows + 2 * (rw + 2);
	cam_unpack_row_pad(src + (size_t)(first ? first - 1 : 0) * stride, lm, rw);
	cam_unpack_row_pad(src + (size_t)first * stride, l0, rw);

	for (y = first; y < last; y++) {
		u8 *o = dst + (size_t)(H - 1 - y) * W * 2;
		bool st_on = !((y >> 1) & (CAMCAP_STATS_ROW_STEP - 1));
		unsigned int nx = y + 1 < rh ? y + 1 : y;

		cam_unpack_row_pad(src + (size_t)nx * stride, lp, rw);

		for (x = 0; x + 1 < W; x += 2) {
			unsigned int xp = x + 2;
			u32 r0, g0, b0, r1, g1, b1;
			u8 *q = o + (size_t)(n - 1 - (x >> 1)) * 4;

			/* columns sit at index x+1 (see cam_unpack_row_pad) */
			if (!(y & 1)) {
				/* x holds R, x+1 holds Gr */
				r0 = l0[x + 1];
				g0 = (l0[x] + l0[xp] + lm[x + 1] + lp[x + 1]) >> 2;
				b0 = (lm[x] + lm[xp] + lp[x] + lp[xp]) >> 2;
				g1 = l0[xp];
				r1 = (l0[x + 1] + l0[x + 3]) >> 1;
				b1 = (lm[xp] + lp[xp]) >> 1;
			} else {
				/* x holds Gb, x+1 holds B */
				g0 = l0[x + 1];
				r0 = (lm[x + 1] + lp[x + 1]) >> 1;
				b0 = (l0[x] + l0[xp]) >> 1;
				b1 = l0[xp];
				g1 = (l0[x + 1] + l0[x + 3] + lm[xp] + lp[xp]) >> 2;
				r1 = (lm[x + 1] + lm[x + 3] + lp[x + 1] + lp[x + 3]) >> 2;
			}
			cam_yuyv_pair(q, r0, g0, b0, r1, g1, b1, sat,
				      lut_ee, lut_oo);

			if (!st_on)
				continue;
			if (!(y & 1)) {			/* even row: R, Gr */
				u32 gg = g1;

				if (rb_swap)
					sum_b += r0;
				else
					sum_r += r0;
				sum_g += gg;
				count++;
				clip += (gg >= CAMCAP_CLIP_LEVEL);
				dark += (gg <= v4l2_black);
				if (gg < min_g)
					min_g = gg;
				if (gg > max_g)
					max_g = gg;
			} else {			/* odd row: Gb, B */
				u32 gg = g0;

				if (rb_swap)
					sum_r += b1;
				else
					sum_b += b1;
				sum_g += gg;
				clip += (gg >= CAMCAP_CLIP_LEVEL);
				dark += (gg <= v4l2_black);
				if (gg < min_g)
					min_g = gg;
				if (gg > max_g)
					max_g = gg;
			}

			/* focus metric: |dY| between the two output pixels */
			{
				int c0 = cam_luma8(r0, g0, b0);
				int c1 = cam_luma8(r1, g1, b1);

				fv += (u32)(c0 > c1 ? c0 - c1 : c1 - c0);
				fv_y += (u64)c0 + (u64)c1;
				fv_n++;
			}
		}

		tmp = lm;
		lm = l0;
		l0 = lp;
		lp = tmp;
	}

	kfree(rows);

	acc->sum_r += sum_r;
	acc->sum_g += sum_g;
	acc->sum_b += sum_b;
	acc->count += count;
	acc->clip += clip;
	acc->dark += dark;
	acc->fv += fv;
	acc->fv_n += fv_n;
	acc->fv_y += fv_y;
	if (count) {
		if (min_g < acc->min_g)
			acc->min_g = min_g;
		if (max_g > acc->max_g)
			acc->max_g = max_g;
	}
}

static void cam_v4l2_convert_band(const u8 *src, u8 *dst, unsigned int first,
				  unsigned int last, struct cam_stats *acc)
{
	unsigned int W = out_width, H = out_height;
	unsigned int n = W / 2;		/* 4-byte YUYV groups per row */

	if (cam_bin == 1) {
		if (v4l2_full_cache)
			cam_v4l2_convert_full_fast(src, dst, first, last, acc);
		else
			cam_v4l2_convert_full_ref(src, dst, first, last, acc);
		return;
	}
	const unsigned int stride = cam_src_stride;	/* raw bytes per line */
	int sat = cam_clamp_int(out_saturation, 0, 255);
	unsigned int y, x;
	u64 sum_r = 0, sum_g = 0, sum_b = 0;
	unsigned int count = 0, clip = 0, dark = 0;
	u32 min_g = 0xffff, max_g = 0;
	u64 fv = 0, fv_n = 0, fv_y = 0;		/* focus metric, see cam_stats */
	/*
	 * The two non-green taps are (0,0) and (1,1).  cam_lut_r and cam_lut_b
	 * hold the white balance that belongs to the *colour*, not to the tap, so
	 * rb_swap redirects which tap feeds which LUT and the matching BT.601
	 * coefficients -- chosen once here so the pixel loop stays branch free.
	 * The statistics labels follow the same switch, otherwise the AWB would
	 * chase the wrong ratio.
	 */
	const u8 *lut_ee = rb_swap ? cam_lut_b : cam_lut_r;	/* (0,0) tap */
	const u8 *lut_oo = rb_swap ? cam_lut_r : cam_lut_b;	/* (1,1) tap */
	const int ya = rb_swap ? 29 : 77;	/* (0,0) tap -> luma */
	const int yb = rb_swap ? 77 : 29;	/* (1,1) tap -> luma */
	const int ua = rb_swap ? 128 : -43;
	const int ub = rb_swap ? -43 : 128;
	const int va = rb_swap ? -21 : 128;
	const int vb = rb_swap ? 128 : -21;

	for (y = first; y < last; y++) {
		const u8 *l0 = src + (size_t)(y * CAMCAP_BIN) * stride;
		const u8 *l1 = l0 + stride;
		u8 *o = dst + (size_t)(H - 1 - y) * W * 2;
		bool st_on = !(y & (CAMCAP_STATS_ROW_STEP - 1));

		for (x = 0; x < W; x += 2) {
			u32 e0 = cam_px_even(l0, x);
			u32 o0 = cam_px_odd(l1, x);
			u32 n0 = cam_px_odd(l0, x);
			u32 m0 = cam_px_even(l1, x);
			u32 e1 = cam_px_even(l0, x + 1);
			u32 o1 = cam_px_odd(l1, x + 1);
			u32 n1 = cam_px_odd(l0, x + 1);
			u32 m1 = cam_px_even(l1, x + 1);
			u8 a0 = lut_ee[e0];
			u8 d0 = lut_oo[o0];
			u8 g0 = cam_lut_g[(n0 + m0) >> 1];
			u8 a1 = lut_ee[e1];
			u8 d1 = lut_oo[o1];
			u8 g1 = cam_lut_g[(n1 + m1) >> 1];
			int ly0 = (ya * a0 + 150 * g0 + yb * d0) >> 8;
			int ly1 = (ya * a1 + 150 * g1 + yb * d1) >> 8;
			int u0 = ((ua * a0 - 85 * g0 + ub * d0) >> 8) + 128;
			int u1 = ((ua * a1 - 85 * g1 + ub * d1) >> 8) + 128;
			int v0 = ((va * a0 - 107 * g0 + vb * d0) >> 8) + 128;
			int v1 = ((va * a1 - 107 * g1 + vb * d1) >> 8) + 128;
			u8 *q = o + (size_t)(n - 1 - (x >> 1)) * 4;

			/* chroma gain around 128, then clamp to byte range */
			u0 = ((u0 - 128) * sat) / 128 + 128;
			u1 = ((u1 - 128) * sat) / 128 + 128;
			v0 = ((v0 - 128) * sat) / 128 + 128;
			v1 = ((v1 - 128) * sat) / 128 + 128;

			/*
			 * YUYV is Y0 Cb Y1 Cr: byte 1 always carries Cb and
			 * byte 3 always carries Cr, whichever of the pair's two
			 * luma bytes was written first.  The luma order above is
			 * reversed to undo the 180 degree flip; the chroma slots
			 * must *not* follow it, because a pair has exactly one Cb
			 * and one Cr.  For the declared CFA grid u is the Cb axis
			 * and v the Cr axis.
			 *
			 * Cr in byte 1 is what "reds look blue, cyan comes out
			 * gold" was: a pure Cb/Cr exchange, which the grey-world
			 * AWB cannot see because it mirrors every gain.
			 */
			q[0] = cam_u8_clamp(ly1);
			q[1] = cam_u8_clamp((u0 + u1) >> 1);
			q[2] = cam_u8_clamp(ly0);
			q[3] = cam_u8_clamp((v0 + v1) >> 1);

			if (!st_on)
				continue;

			{
				u32 mg0 = (n0 + m0) >> 1;
				u32 mg1 = (n1 + m1) >> 1;

				sum_r += rb_swap ? o0 + o1 : e0 + e1;
				sum_b += rb_swap ? e0 + e1 : o0 + o1;
				sum_g += n0 + m0 + n1 + m1;
				count += 2;
				clip += (n0 >= CAMCAP_CLIP_LEVEL) +
					(m0 >= CAMCAP_CLIP_LEVEL) +
					(n1 >= CAMCAP_CLIP_LEVEL) +
					(m1 >= CAMCAP_CLIP_LEVEL);
				dark += (n0 <= v4l2_black) + (m0 <= v4l2_black) +
					(n1 <= v4l2_black) + (m1 <= v4l2_black);
				if (mg0 < min_g)
					min_g = mg0;
				if (mg0 > max_g)
					max_g = mg0;
				if (mg1 < min_g)
					min_g = mg1;
				if (mg1 > max_g)
					max_g = mg1;

				/* focus metric: |dY| between the two output pixels */
				{
					int c0 = cam_clamp_int(ly0, 0, 255);
					int c1 = cam_clamp_int(ly1, 0, 255);

					fv += (u32)(c0 > c1 ? c0 - c1 : c1 - c0);
					fv_y += (u64)c0 + (u64)c1;
					fv_n++;
				}
			}
		}
	}

	acc->sum_r += sum_r;
	acc->sum_g += sum_g;
	acc->sum_b += sum_b;
	acc->count += count;
	acc->clip += clip;
	acc->dark += dark;
	acc->fv += fv;
	acc->fv_n += fv_n;
	acc->fv_y += fv_y;
	if (count) {
		if (min_g < acc->min_g)
			acc->min_g = min_g;
		if (max_g > acc->max_g)
			acc->max_g = max_g;
	}
}

/* Zero an accumulator and make it ready for cam_v4l2_convert_band(). */
static void cam_stats_reset(struct cam_stats *st)
{
	memset(st, 0, sizeof(*st));
	st->min_g = 0xffff;
}

/*
 * Fold one worker's accumulator into the frame total.  mean_* stay untouched
 * until cam_stats_finish() has divided the sums, so a half-merged accumulator
 * is never mistaken for a finished measurement.
 */
static void cam_stats_merge(struct cam_stats *d, const struct cam_stats *s)
{
	d->sum_r += s->sum_r;
	d->sum_g += s->sum_g;
	d->sum_b += s->sum_b;
	d->count += s->count;
	d->clip += s->clip;
	d->dark += s->dark;
	d->fv += s->fv;
	d->fv_n += s->fv_n;
	d->fv_y += s->fv_y;
	if (s->count) {
		if (s->min_g < d->min_g)
			d->min_g = s->min_g;
		if (s->max_g > d->max_g)
			d->max_g = s->max_g;
	}
}

/* Turn the merged sums into the means the AWB/AE governor consumes. */
static void cam_stats_finish(struct cam_stats *st)
{
	if (!st->count) {
		st->min_g = 0;
		return;
	}
	/* green has two samples per 2x2 block, red and blue one each */
	st->mean_r = (u32)(st->sum_r / st->count);
	st->mean_g = (u32)(st->sum_g / (st->count * 2));
	st->mean_b = (u32)(st->sum_b / st->count);
}

/*
 * ---- parallel conversion -------------------------------------------------
 *
 * The convert is the most expensive thing the capture thread does and it is
 * pure arithmetic over independent rows, so it is spread over a small pool of
 * kthreads.  Every row costs the same, so the bands are a static split of the
 * row range and no two workers ever touch the same output byte.
 *
 * The pool lives exactly as long as a stream does.  For each frame the capture
 * thread publishes the destination and bumps a generation counter, wakes every
 * worker and waits for all of them to complete().
 */
static unsigned int conv_threads = 8;	/* measured: 8 threads is what the two
					 * fast sensor modes need (1080p120 and
					 * 4000x2256@60 are converter bound); the
					 * smaller frames do not care, and the
					 * workers are niced to 10 so they cannot
					 * starve the desktop.
					 */
module_param(conv_threads, uint, 0644);
MODULE_PARM_DESC(conv_threads,
		 "conversion threads: 1 = single threaded, 2..8 = parallel worker pool (default 8; the pool only runs for the duration of a frame and its workers are niced to 10)");

static int cam_conv_thread(void *arg)
{
	struct cam_conv_worker *w = arg;
	struct cam_v4l2_ctx *c = w->ctx;

	while (!kthread_should_stop()) {
		wait_event_interruptible(c->conv_wq,
			kthread_should_stop() ||
			READ_ONCE(c->job_gen) != w->seen);
		if (kthread_should_stop())
			break;
		w->seen = READ_ONCE(c->job_gen);
		cam_stats_reset(&w->st);
		cam_v4l2_convert_band(c->job_src, c->job_dst, w->y0, w->y1, &w->st);
		complete(&c->conv_done);
	}
	return 0;
}

static void cam_conv_pool_stop(struct cam_v4l2_ctx *c)
{
	unsigned int i;

	if (!c->nconv)
		return;
	wake_up_all(&c->conv_wq);
	for (i = 0; i < c->nconv; i++) {
		if (c->conv[i].task) {
			kthread_stop(c->conv[i].task);
			c->conv[i].task = NULL;
		}
	}
	c->nconv = 0;
}

static void cam_conv_pool_start(struct cam_v4l2_ctx *c)
{
	unsigned int want = conv_threads ? conv_threads : 1;
	unsigned int i, j;

	if (want > CAMCAP_MAX_CONV)
		want = CAMCAP_MAX_CONV;
	if (want < 2) {
		c->nconv = 0;
		pr_info("convert: single threaded (1 of %u online CPUs)\n",
			num_online_cpus());
		return;
	}

	init_completion(&c->conv_done);
	for (i = 0; i < want; i++) {
		struct cam_conv_worker *w = &c->conv[i];

		w->ctx = c;
		w->id = i;
		/*
		 * Synchronise with the generator, not with 0: job_gen keeps
		 * counting across streams, so a fresh worker must ignore
		 * everything that happened in a previous one.
		 */
		w->seen = c->job_gen;
		/* contiguous, equal-sized row bands: the work is uniform */
		w->y0 = out_height * i / want;
		w->y1 = out_height * (i + 1) / want;
		w->task = kthread_run(cam_conv_thread, w, "cam_conv/%u", i);
		if (IS_ERR(w->task)) {
			pr_warn("convert: worker %u failed (%ld), falling back to inline\n",
				i, PTR_ERR(w->task));
			w->task = NULL;
			break;
		}
		/*
		 * Conversion is pure batch work: run it below the rest of the
		 * system so that eight busy cores can never make the box feel
		 * unresponsive (ssh, the desktop, the USB console all keep
		 * their share).
		 */
		set_user_nice(w->task, 10);
	}

	if (i < want) {
		/* a partial pool would leave some rows unconverted */
		for (j = 0; j < i; j++) {
			kthread_stop(c->conv[j].task);
			c->conv[j].task = NULL;
		}
		c->nconv = 0;
		return;
	}

	c->nconv = i;
	pr_info("convert: %u workers, %u rows each\n", i, out_height / i);
}

/*
 * Convert one frame, in parallel when the pool is up, and leave the frame's
 * scene statistics in *out (means filled in, ready for the governor).
 * Returns nanoseconds.
 */
static u64 cam_convert_frame(struct cam_v4l2_ctx *c, const u8 *src, u8 *dst,
			     struct cam_stats *out)
{
	ktime_t t0 = ktime_get();
	unsigned int i;
	bool inline_done = false;

	if (!c->nconv) {
		cam_stats_reset(out);
		cam_v4l2_convert_band(src, dst, 0, out_height, out);
	} else {
		c->job_src = src;
		c->job_dst = dst;
		reinit_completion(&c->conv_done);
		WRITE_ONCE(c->job_gen, c->job_gen + 1);
		wake_up_all(&c->conv_wq);
		/*
		 * Every worker completes once per job, so wait for ALL of them.
		 * Waiting for a single completion would let the capture thread
		 * hand a half-converted buffer to userspace and would keep
		 * issuing new jobs while the old ones were still running, which
		 * turns the pool into a permanent full-speed load.
		 *
		 * The timeout is a safety net: a frame takes ~0.44 s, so two
		 * seconds of silence means a worker really died.  Finishing the
		 * frame inline keeps the camera alive instead of stalling the
		 * capture thread forever.
		 */
		for (i = 0; i < c->nconv; i++) {
			if (wait_for_completion_timeout(&c->conv_done, 2 * HZ))
				continue;
			pr_warn_once("convert: worker stalled, finishing frame inline\n");
			cam_stats_reset(out);
			cam_v4l2_convert_band(src, dst, 0, out_height, out);
			inline_done = true;
			break;
		}
		if (!inline_done) {
			cam_stats_reset(out);
			for (i = 0; i < c->nconv; i++)
				cam_stats_merge(out, &c->conv[i].st);
		}
	}
	cam_stats_finish(out);
	return ktime_to_ns(ktime_sub(ktime_get(), t0));
}

/* Program the TG/DMA exactly like `cfg 1 0 4000 0 3000 6000 3000 6000`. */
static void cam_v4l2_program(void)
{
	struct camcap_cfg c = {
		.fmt = 1,
		.pxl_start = 0,
		.pxl_end = cam_src_w,
		.lin_start = 0,
		.lin_end = cam_src_h,
		.xsize = cam_src_stride,
		.ysize = cam_src_h,
		.stride = cam_src_stride,
	};

	cam_cap_static_config(&c);
}

/* One frame: arm CAMSV1, then convert the CMA buffer into the vb2 buffer. */
static int cam_v4l2_grab(struct cam_v4l2_buf *buf)
{
	struct cam_v4l2_ctx *c = cam_v4l2;
	u8 *dst;
	ktime_t t0, t1;
	int ret;

	if (!cam_frame)
		return -ENODEV;

	/* push a pending exposure/gain change before the next frame starts */
	cam_sensor_apply();

	if (mutex_lock_interruptible(&cam_lock))
		return -ERESTARTSYS;
	t0 = ktime_get();
	ret = cam_cap_arm();
	if (c)
		c->t_arm = ktime_to_ns(ktime_sub(ktime_get(), t0));
	mutex_unlock(&cam_lock);
	if (ret)
		return ret;

	/* the TG stopped itself (single_mode); order the DMA writes first */
	dma_rmb();

	/*
	 * The frame is read through the cacheable linear-map alias of the buffer
	 * (see cam_raw_src()), so whatever the CPU still holds for those lines
	 * from the previous frame has to go before this one is read.  On a
	 * non-coherent device this is exactly what dma_sync_single_for_cpu()
	 * does: it invalidates the linear-map alias of the range.
	 */
	cam_raw_sync();

	dst = vb2_plane_vaddr(&buf->vb.vb2_buf, 0);
	if (!dst)
		return -EINVAL;

	/* a control may have changed the curve while we were armed */
	if (cam_lut_dirty)
		cam_lut_rebuild();

	if (c) {
		struct cam_stats st;

		c->t_conv = cam_convert_frame(c, cam_raw_src(), dst, &st);
		t1 = ktime_get();
		cam_isp_governor(&st);
		c->t_gov = ktime_to_ns(ktime_sub(ktime_get(), t1));
		c->t_total = c->t_arm + c->t_conv + c->t_gov;
		c->t_frames++;
		c->s_arm += c->t_arm;
		c->s_conv += c->t_conv;
		c->s_gov += c->t_gov;
		c->s_n++;
	} else {
		struct cam_stats st;

		cam_stats_reset(&st);
		cam_v4l2_convert_band(cam_raw_src(), dst, 0, out_height, &st);
		cam_stats_finish(&st);
		cam_isp_governor(&st);
	}
	return 0;
}

/*
 * The frame the CPU reads was written by CAMSV through the IOMMU, so whatever
 * the CPU still holds for those lines from the previous frame is stale.  On a
 * non-coherent device dma_sync_single_for_cpu() is exactly the maintenance
 * that is needed: it invalidates arch_sync_dma_for_cpu() the linear-map alias
 * of the *physical* range (never the IOVA -- the sync device has no IOMMU, so
 * dma_to_phys() would translate nothing and the invalidate would land on
 * unrelated memory).
 */
static void cam_slot_sync(struct cam_slot *s)
{
	if (!cam_dma_dev || !s->phys || !s->va)
		return;
	dma_sync_single_for_cpu(cam_dma_dev, (dma_addr_t)s->phys, s->size,
				DMA_FROM_DEVICE);
}

/*
 * The extra frame buffers of the capture pipeline.  Slot[0] is the buffer the
 * module already owns (CMA, mapped at cam_frame_pa); the slots handed out below
 * are what make the pipeline possible.  CMA cannot hold two 18 MB buffers
 * (measured CmaTotal 32768 kB / CmaFree 10144 kB) and the buddy allocator
 * cannot return one block of that size either (MAX_ORDER is 10, 4 MiB), but
 * alloc_contig_pages() can -- it migrates pages and takes the range out of the
 * normal allocator, which is what a frame buffer wants anyway.  The block is
 * ordinary System RAM, so page_address() is a cacheable linear-map address and
 * cam_slot_sync() is all the maintenance it needs; the IOVA is installed by
 * hand because iommu_dma_alloc_noncontiguous() aborts with -EEXIST on
 * io-pgtable-arm-v7s.
 */
static struct cam_slot cam_slot_buf[CAMCAP_PIPE_SLOTS - 1];
static unsigned int cam_slot_extra;	/* how many of them are live */

static void cam_slots_release(void)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(cam_slot_buf); i++) {
		struct cam_slot *s = &cam_slot_buf[i];
		unsigned long nr;

		if (!s->pages)
			continue;
		nr = (unsigned long)(PAGE_ALIGN(s->size) >> PAGE_SHIFT);
		if (cam_iommu_dom && s->iova)
			iommu_unmap(cam_iommu_dom, s->iova, s->size);
		free_contig_range(page_to_pfn(s->pages), nr);
		memset(s, 0, sizeof(*s));
	}
	cam_slot_extra = 0;
}

static void cam_slots_alloc(void)
{
	/*
	 * The display IOMMU domain is shared, so the window is picked by trying
	 * candidates until iommu_map() accepts one (it returns -EEXIST for a
	 * range that is already mapped).  0x10000000 and below belongs to the
	 * display and to this module's own buffer.
	 */
	static const dma_addr_t probe[] = {
		0x20000000, 0x24000000, 0x28000000, 0x2c000000,
		0x30000000, 0x34000000, 0x38000000, 0x3c000000,
		0x40000000, 0x48000000, 0x50000000, 0x60000000,
	};
	unsigned int want, k;

	cam_slot_extra = 0;
	if (!pipeline || !cam_buf_size)
		return;
	if (!cam_iommu_dom) {
		pr_info("pipeline: no IOMMU domain, capturing serially\n");
		return;
	}

	want = clamp(pipe_slots, 2u, (unsigned int)CAMCAP_PIPE_SLOTS) - 1;

	for (k = 0; k < want; k++) {
		struct cam_slot *s = &cam_slot_buf[k];
		unsigned long nr;
		unsigned int i;
		int mret = -ENOMEM;

		nr = (unsigned long)(PAGE_ALIGN(cam_buf_size) >> PAGE_SHIFT);
		/*
		 * The node has to be a real one.  alloc_contig_frozen_pages()
		 * starts from node_zonelist(nid, gfp_mask), so NUMA_NO_NODE
		 * walks node_data[-1] and faults inside the allocator
		 * (measured: a page fault at
		 * alloc_contig_frozen_pages_noprof+0x9c that killed the
		 * caller); in-tree callers pass first_online_node.
		 */
		s->pages = alloc_contig_pages(nr, GFP_KERNEL,
					      numa_node_id(), NULL);
		if (!s->pages) {
			pr_warn("pipeline: no contiguous %lu page block, capturing serially\n",
				nr);
			break;
		}
		s->va = page_address(s->pages);
		s->phys = page_to_phys(s->pages);
		s->size = cam_buf_size;
		if (!s->va) {
			pr_warn("pipeline: contiguous block %u has no linear mapping, capturing serially\n",
				k + 1);
			goto err;
		}

		for (i = 0; i < ARRAY_SIZE(probe); i++) {
			unsigned int j;
			bool used = false;

			/* an earlier slot may already own this window */
			for (j = 0; j < k; j++) {
				if (cam_slot_buf[j].iova == probe[i]) {
					used = true;
					break;
				}
			}
			if (used)
				continue;

			mret = iommu_map(cam_iommu_dom, probe[i], s->phys,
					 s->size, IOMMU_READ | IOMMU_WRITE,
					 GFP_KERNEL);
			if (!mret) {
				s->iova = probe[i];
				break;
			}
		}
		if (mret) {
			pr_warn("pipeline: no free IOVA window for buffer %u (%d), capturing serially\n",
				k + 1, mret);
			goto err;
		}

		cam_slot_extra = k + 1;
		pr_info("pipeline: frame buffer %u of %u, %lu bytes at pa %#llx iova %#llx (cacheable %p)\n",
			k + 1, want, (unsigned long)s->size,
			(unsigned long long)s->phys,
			(unsigned long long)s->iova, s->va);
	}
	return;

err:
	cam_slots_release();
}

static int cam_pipe_setup(struct cam_v4l2_ctx *c)
{
	struct cam_slot *s0 = &c->slot[0];
	unsigned int i;

	memset(c->slot, 0, sizeof(c->slot));
	s0->va = (u8 *)(cam_frame_wb ? cam_frame_wb : cam_frame);
	s0->iova = cam_frame_pa;
	s0->phys = cam_buf_phys;
	s0->size = cam_buf_size;
	c->slot_next = 0;
	c->nslots = 1;
	c->pipe_used = false;

	if (!pipeline || !cam_slot_extra || !s0->va || !cam_buf_size)
		return -ENODEV;

	for (i = 0; i < cam_slot_extra; i++)
		c->slot[i + 1] = cam_slot_buf[i];	/* template; full/busy/vb stay zero */
	c->nslots = cam_slot_extra + 1;
	c->pipe_used = true;
	return 0;
}

/*
 * Only the per-stream state goes away here: the extra buffers and their IOVA
 * mappings belong to the module and are reused by the next streamon.
 */
static void cam_pipe_free(struct cam_v4l2_ctx *c)
{
	c->pipe_used = false;
	c->nslots = 1;
	memset(c->slot, 0, sizeof(c->slot));
}

/*
 * Frame-period histogram, for /proc/camcap_info.  The mean frame rate hides
 * the frames that slipped: the hardware finishes a frame about a millisecond
 * before the next one starts, so anything that delays the arm thread past that
 * window makes the next frame start late and costs part or all of a period.
 * `per` is nanoseconds and the buckets are absolute, assuming the nominal
 * period of the mode in use (VTS 3300 gives 30.2 ms): clean is a frame that
 * used the whole period, late started a couple of milliseconds into the next
 * one, slip is most of a period gone, lost is a whole period (60.4 ms) or more.
 */
static void cam_per_hist_add(struct cam_v4l2_ctx *c, u64 per)
{
	if (per < 32000000ULL)
		c->per_hist[0]++;
	else if (per < 40000000ULL)
		c->per_hist[1]++;
	else if (per < 58000000ULL)
		c->per_hist[2]++;
	else
		c->per_hist[3]++;
}

/*
 * Arm one slot and wait for its frame, then publish it for the converter
 * thread.  Nothing is read out of the slot here, so the arm thread is back at
 * the sensor as fast as the hardware allows.
 *
 * The exposure/gain registers are deliberately *not* written here: the
 * converter thread applies them after the AE has run, because this thread only
 * has about a millisecond between the frame-done of one frame and the start of
 * the next, and an I2C sequence is easily longer than that.
 */
static int cam_v4l2_arm_slot(struct cam_v4l2_ctx *c, struct cam_slot *s,
			     struct cam_v4l2_buf *buf)
{
	ktime_t t0;
	int ret;

	if (mutex_lock_interruptible(&cam_lock))
		return -ERESTARTSYS;
	t0 = ktime_get();
	ret = cam_cap_arm_addr(s->iova);
	c->t_arm = ktime_to_ns(ktime_sub(ktime_get(), t0));
	mutex_unlock(&cam_lock);
	if (ret)
		return ret;

	/* the TG stopped itself (single_mode); order the DMA writes first */
	dma_rmb();

	s->vb = buf;
	s->seq = cam_last_seq;
	s->full = true;
	return 0;
}

/* Read one finished slot, convert it and complete its buffer. */
static void cam_v4l2_finish_slot(struct cam_v4l2_ctx *c, struct cam_slot *s)
{
	struct cam_v4l2_buf *buf = s->vb;
	struct cam_stats st;
	u8 *dst;
	ktime_t t1;

	s->vb = NULL;
	if (!buf)
		return;

	/*
	 * Whatever the CPU still holds for this slot's lines is from the
	 * previous frame: drop it before reading the new one.
	 */
	cam_slot_sync(s);

	dst = vb2_plane_vaddr(&buf->vb.vb2_buf, 0);
	if (!dst) {
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
		return;
	}

	/* a control may have changed the curve while the frame was in flight */
	if (cam_lut_dirty)
		cam_lut_rebuild();

	c->t_conv = cam_convert_frame(c, s->va, dst, &st);
	/*
	 * Every read of this slot is finished, so hand it back before doing
	 * anything else: the AE/AWB update and the vb2 hand-off below do not
	 * touch it, and the arm thread may already be waiting for it.
	 */
	s->busy = false;
	wake_up_interruptible(&c->pipe_wq);

	t1 = ktime_get();
	cam_isp_governor(&st);
	c->t_gov = ktime_to_ns(ktime_sub(ktime_get(), t1));
	c->t_total = c->t_arm + c->t_conv + c->t_gov;
	c->t_frames++;
	c->s_arm += c->t_arm;
	c->s_conv += c->t_conv;
	c->s_gov += c->t_gov;
	c->s_n++;
	c->pipe_frames++;

	buf->vb.vb2_buf.timestamp = ktime_get_ns();
	buf->vb.sequence = c->seq++;
	vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
}

/* Take the first slot that is full and not already being read. */
static struct cam_slot *cam_pipe_take(struct cam_v4l2_ctx *c)
{
	unsigned int i;

	for (i = 0; i < c->nslots; i++) {
		struct cam_slot *s = &c->slot[i];

		if (s->full && !s->busy) {
			s->busy = true;
			s->full = false;
			return s;
		}
	}
	return NULL;
}

/* Is any slot waiting to be read?  Used as the converter thread's wait test. */
static bool cam_pipe_any_full(struct cam_v4l2_ctx *c)
{
	unsigned int i;

	for (i = 0; i < c->nslots; i++) {
		if (c->slot[i].full)
			return true;
	}
	return false;
}

/*
 * Converter thread: drains whichever slot the arm thread has published, so the
 * conversion of frame N happens while frame N+1 is still on the wire.  Its
 * cost is hidden as long as it is shorter than the sensor period, which is the
 * whole point of the second buffer.
 */
static int cam_v4l2_conv_thread(void *arg)
{
	struct cam_v4l2_ctx *c = arg;

	while (!kthread_should_stop()) {
		struct cam_slot *s;

		wait_event_interruptible(c->pipe_wq,
			kthread_should_stop() || cam_pipe_any_full(c));
		if (kthread_should_stop())
			break;

		s = cam_pipe_take(c);
		if (!s)
			continue;

		/* finish_slot() releases the slot as soon as its reads are done */
		cam_v4l2_finish_slot(c, s);
	}
	return 0;
}

static void cam_v4l2_return_all(struct cam_v4l2_ctx *c, enum vb2_buffer_state state)
{
	struct cam_v4l2_buf *buf, *tmp;
	unsigned long flags;

	spin_lock_irqsave(&c->qlock, flags);
	list_for_each_entry_safe(buf, tmp, &c->queued, list) {
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, state);
	}
	spin_unlock_irqrestore(&c->qlock, flags);
}

/*
 * Capture loop.  Each iteration captures exactly one frame (single_mode makes
 * the TG stop by itself, so the buffer can never be overwritten mid-read) and
 * hands the buffer to vb2.  A 200 ms wait keeps the thread idle when no
 * application is queueing buffers.
 */
static int cam_v4l2_thread(void *arg)
{
	struct cam_v4l2_ctx *c = arg;
	ktime_t t_prev = ktime_get();

	while (!kthread_should_stop()) {
		struct cam_v4l2_buf *buf = NULL;
		unsigned long flags;
		ktime_t t_now;

		spin_lock_irqsave(&c->qlock, flags);
		if (!list_empty(&c->queued)) {
			buf = list_first_entry(&c->queued, struct cam_v4l2_buf,
					       list);
			list_del(&buf->list);
		}
		spin_unlock_irqrestore(&c->qlock, flags);

		if (!buf) {
			ktime_t t_idle = ktime_get();

			wait_event_interruptible_timeout(c->wq,
				kthread_should_stop() ||
				!list_empty(&c->queued),
				msecs_to_jiffies(200));
			c->t_wait = ktime_to_ns(ktime_sub(ktime_get(), t_idle));
			continue;
		}

		/* stop_streaming() is waiting: give the buffer back and leave */
		if (kthread_should_stop()) {
			vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
			break;
		}

		if (c->pipe_used) {
			struct cam_slot *s = &c->slot[c->slot_next];

			/*
			 * Wait until the slot is completely idle.  A slot that
			 * is still `full` holds a frame the converter has not
			 * read yet, so arming into it would overwrite that
			 * frame; `busy` means the converter is reading it right
			 * now.  The timeout is a safety net: normally the
			 * converter releases the slot inside one frame period.
			 */
			if (wait_event_interruptible_timeout(c->pipe_wq,
					kthread_should_stop() ||
					(!s->busy && !s->full),
					msecs_to_jiffies(1000)) <= 0) {
				pr_warn_once("pipeline: slot %u not released, dropping a frame\n",
					     c->slot_next);
				vb2_buffer_done(&buf->vb.vb2_buf,
						VB2_BUF_STATE_ERROR);
				continue;
			}
			if (kthread_should_stop()) {
				vb2_buffer_done(&buf->vb.vb2_buf,
						VB2_BUF_STATE_ERROR);
				break;
			}

			/* interval between two consecutive frames: this is the fps */
			t_now = ktime_get();
			c->t_period = ktime_to_ns(ktime_sub(t_now, t_prev));
			t_prev = t_now;
			c->s_period += c->t_period;
			cam_per_hist_add(c, c->t_period);

			if (cam_v4l2_arm_slot(c, s, buf)) {
				vb2_buffer_done(&buf->vb.vb2_buf,
						VB2_BUF_STATE_ERROR);
				continue;
			}
			c->slot_next++;
			if (c->slot_next >= c->nslots)
				c->slot_next = 0;
			/* hand the finished slot to the converter thread */
			wake_up_interruptible(&c->pipe_wq);
			continue;
		}

		/* interval between two consecutive frames: this is the fps */
		t_now = ktime_get();
		c->t_period = ktime_to_ns(ktime_sub(t_now, t_prev));
		t_prev = t_now;
		c->s_period += c->t_period;
		cam_per_hist_add(c, c->t_period);

		if (cam_v4l2_grab(buf) == 0) {
			buf->vb.vb2_buf.timestamp = ktime_get_ns();
			buf->vb.sequence = c->seq++;
			vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
		} else {
			vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
		}
	}
	return 0;
}

/* ---- vb2 queue ops ---- */

static int cam_vb2_queue_setup(struct vb2_queue *q, unsigned int *num_buffers,
			       unsigned int *num_planes, unsigned int sizes[],
			       struct device *alloc_devs[])
{
	unsigned int size = out_width * out_height * 2;

	if (*num_planes) {
		if (sizes[0] < size)
			return -EINVAL;
		return 0;
	}

	*num_planes = 1;
	sizes[0] = size;
	if (*num_buffers < 2)
		*num_buffers = 2;

	return 0;
}

static int cam_vb2_buf_prepare(struct vb2_buffer *vb)
{
	unsigned int size = out_width * out_height * 2;

	if (vb2_plane_size(vb, 0) < size)
		return -EINVAL;
	vb2_set_plane_payload(vb, 0, size);

	return 0;
}

static void cam_vb2_buf_queue(struct vb2_buffer *vb)
{
	struct cam_v4l2_ctx *c = vb2_get_drv_priv(vb->vb2_queue);
	struct cam_v4l2_buf *buf = container_of(to_vb2_v4l2_buffer(vb),
						struct cam_v4l2_buf, vb);
	unsigned long flags;

	spin_lock_irqsave(&c->qlock, flags);
	list_add_tail(&buf->list, &c->queued);
	spin_unlock_irqrestore(&c->qlock, flags);

	wake_up(&c->wq);
}

static int cam_vb2_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct cam_v4l2_ctx *c = vb2_get_drv_priv(q);
	int ret;

	/*
	 * Push whatever userspace set before streamon through the control
	 * callbacks, then build the tone curves from the result.
	 */
	v4l2_ctrl_handler_setup(&c->hdl);
	cam_lut_rebuild();
	/*
	 * A new stream starts with no exposure history, and the contrast search
	 * starts from scratch: whatever lens position the last session ended on
	 * says nothing about this scene.
	 */
	cam_ae_settle = 0;
	cam_ae_mean_s = 0;
	cam_af_reset();
	/* the arm thread no longer applies the sensor state, so do it once here */
	cam_sensor_apply();

	if (mutex_lock_interruptible(&cam_lock))
		return -ERESTARTSYS;
	cam_v4l2_program();
	mutex_unlock(&cam_lock);

	c->t_frames = 0;
	c->t_period = 0;
	c->s_n = 0;
	c->s_arm = c->s_conv = c->s_gov = c->s_period = 0;
	memset(c->per_hist, 0, sizeof(c->per_hist));
	cam_conv_pool_start(c);

	/*
	 * The converter thread comes up before the capture thread: it sleeps
	 * until a slot is published, so it cannot get ahead of anything, and
	 * this way a failure can still be turned into the serial path before
	 * the capture thread has seen pipe_used = true.
	 */
	cam_pipe_setup(c);
	if (c->pipe_used) {
		c->cthread = kthread_run(cam_v4l2_conv_thread, c, "cam_cap_conv");
		if (IS_ERR(c->cthread)) {
			long err = PTR_ERR(c->cthread);

			c->cthread = NULL;
			c->pipe_used = false;
			cam_pipe_free(c);
			pr_warn("v4l2: converter thread failed (%ld), capturing serially\n",
				err);
		}
	}

	c->streaming = true;
	c->thread = kthread_run(cam_v4l2_thread, c, "cam_cap_v4l2");
	if (IS_ERR(c->thread)) {
		ret = PTR_ERR(c->thread);
		c->thread = NULL;
		c->streaming = false;
		cam_conv_pool_stop(c);
		if (c->cthread) {
			kthread_stop(c->cthread);
			c->cthread = NULL;
		}
		cam_pipe_free(c);
		cam_v4l2_return_all(c, VB2_BUF_STATE_QUEUED);
		pr_err("v4l2: cannot start capture thread (%d)\n", ret);
		return ret;
	}

	pr_info("v4l2: streaming %ux%u YUYV, %u convert thread(s), %s\n",
		out_width, out_height, c->nconv,
		c->pipe_used ? "pipelined capture" : "serial capture");
	return 0;
}

static void cam_vb2_stop_streaming(struct vb2_queue *q)
{
	struct cam_v4l2_ctx *c = vb2_get_drv_priv(q);
	unsigned int i;

	c->streaming = false;
	if (c->thread) {
		kthread_stop(c->thread);
		c->thread = NULL;
	}
	if (c->cthread) {
		kthread_stop(c->cthread);
		c->cthread = NULL;
	}
	cam_conv_pool_stop(c);

	/*
	 * A slot still holding a buffer means the converter thread was stopped
	 * between the arm and the conversion; vb2 is waiting for that buffer, so
	 * it has to be given back before the queue can be torn down.
	 */
	c->pipe_used = false;
	for (i = 0; i < ARRAY_SIZE(c->slot); i++) {
		if (c->slot[i].vb) {
			vb2_buffer_done(&c->slot[i].vb->vb.vb2_buf,
					VB2_BUF_STATE_ERROR);
			c->slot[i].vb = NULL;
		}
		c->slot[i].full = false;
		c->slot[i].busy = false;
	}
	cam_pipe_free(c);

	if (cam_base) {
		mutex_lock(&cam_lock);
		cam_cap_vf_off();
		mutex_unlock(&cam_lock);
	}

	cam_v4l2_return_all(c, VB2_BUF_STATE_ERROR);
	pr_info("v4l2: streaming stopped\n");
}

static const struct vb2_ops cam_vb2_ops = {
	.queue_setup = cam_vb2_queue_setup,
	.buf_prepare = cam_vb2_buf_prepare,
	.buf_queue = cam_vb2_buf_queue,
	.start_streaming = cam_vb2_start_streaming,
	.stop_streaming = cam_vb2_stop_streaming,
};

/* ---- sensor modes ---- */

/*
 * The five tables in the generated header are the recording formats this
 * IMX582 variant offers.  Selecting one replays the power-on table, the mode
 * table and the VTS this port was verified with over I2C, which is the same
 * sequence the bring-up script performs by hand, so an application can switch
 * format through V4L2 instead of the module being reloaded with other
 * exp_hsize/exp_vsize parameters.  cam_mode_idx (declared with the other
 * state above) is the selected entry, or -1 while the run-time geometry comes
 * from those parameters alone.
 */
static const struct cam_sensor_mode *cam_mode_get(int idx)
{
	if (idx < 0 || idx >= CAMCAP_IMX582_NMODES)
		return NULL;

	return &cam_imx582_modes[idx];
}

/*
 * Which mode produces w x h when the converter bins by "bin" (1 = one output
 * pixel per raw pixel, 2 = 2x2 average).  fps_x100 == 0 accepts any frame
 * rate, otherwise the closest wins, which is how VIDIOC_S_PARM chooses
 * between the 1080p120 and 1080p240 modes.
 */
static int cam_mode_match(unsigned int w, unsigned int h, unsigned int bin,
			  unsigned int fps_x100)
{
	int best = -1;
	unsigned int best_diff = 0;
	int i;

	if (bin != 1 && bin != 2)
		return -EINVAL;

	for (i = 0; i < CAMCAP_IMX582_NMODES; i++) {
		const struct cam_sensor_mode *m = &cam_imx582_modes[i];
		unsigned int diff;

		if (m->hsize / bin != w || m->vsize / bin != h)
			continue;
		if (!fps_x100)
			return i;

		diff = m->fps_x100 > fps_x100 ? m->fps_x100 - fps_x100
					      : fps_x100 - m->fps_x100;
		if (best < 0 || diff < best_diff) {
			best = i;
			best_diff = diff;
		}
		if (!diff)
			break;
	}

	return best;
}

/* the geometry the converter and the CAMSV registers work from */
static void cam_mode_geometry(const struct cam_sensor_mode *m, unsigned int bin)
{
	cam_bin = bin;
	cam_src_w = m->hsize;
	cam_src_h = m->vsize;
	cam_src_stride = m->hsize * 3 / 2;
	out_width = m->hsize / bin;
	out_height = m->vsize / bin;
	exp_max = m->exp_max;
	/*
	 * 1080p240 holds 1236 lines in a frame, so an exposure the previous
	 * mode was using can be longer than a whole frame here.  Clamp it, or
	 * the sensor stretches every frame and the mode delivers a fraction of
	 * its rate (the same failure as running preview with a 0x0c64
	 * exposure).
	 */
	if (cam_sensor.exposure > exp_max)
		cam_sensor.exposure = exp_max;
	cam_mode_fps = m->fps_x100;
}

/*
 * Keep V4L2_CID_EXPOSURE_ABSOLUTE in step with the mode: 1080p240 has 1236
 * lines in a frame, so it cannot hold the exposure a 4000x3000 mode can, and
 * an application writing a value past the range would be refused by the
 * control framework for the wrong reason.
 */
static void cam_ctrl_update_exp_max(void)
{
	struct v4l2_ctrl *ctrl;

	if (!cam_v4l2 || !cam_v4l2->ct_exposure)
		return;

	ctrl = cam_v4l2->ct_exposure;
	__v4l2_ctrl_modify_range(ctrl, CAMCAP_EXP_MIN, (int)exp_max, 1,
				 ctrl->val > (int)exp_max ? (int)exp_max
							  : ctrl->val);
}

/*
 * Retime the receiver for a new MIPI data rate.  Only two fields track the
 * rate: HS_TRAIL_PARAMETER (and its enable) on each data lane of the D-PHY, and
 * DMY_CYCLE in the CSI2 resynchronisation control.  The formulas are the
 * vendor's (isp71_ref mtk_cam-seninf-hw_phy_3_0.c), i.e. exactly what
 * scripts/port2_rx71.py computes for the bring-up rate, so for the 1370 Mbps
 * modes this writes the values userspace already programmed.
 */
static void cam_rx_set_rate(unsigned int mbps, bool force)
{
	u64 data_rate, t;
	u32 cycles, ui_224, hs_trail, v;
	unsigned int i;

	if (!mbps || !rx_rate)
		return;
	if (!force && mbps == cam_rx_mbps)
		return;
	if (!cam_dphy || !cam_seninf) {
		pr_warn("rx: no D-PHY mapping, cannot retime for %u Mbps\n",
			mbps);
		return;
	}

	data_rate = (u64)mbps * 1000000;
	cycles = (u32)div_u64(64ULL * CAMCAP_SENINF_CK, data_rate) + 1;
	ui_224 = (u32)div_u64(224ULL * 1000, mbps);
	if (ui_224 <= CAMCAP_DPHY_TRAIL_DT) {
		hs_trail = 0;
	} else {
		t = (u64)(ui_224 - CAMCAP_DPHY_TRAIL_DT) * CAMCAP_SENINF_CK;
		hs_trail = (u32)div_u64(t + 999999999ULL, 1000000000ULL);
	}

	for (i = 0; i < 4; i++) {
		v = readl(cam_dphy + CAMCAP_DPHY_DATA_LANE(i));
		v = (v & ~CAMCAP_DPHY_HS_TRAIL) | ((hs_trail & 0xff) << 8);
		v &= ~CAMCAP_DPHY_HS_TRAIL_EN;
		if (hs_trail)
			v |= CAMCAP_DPHY_HS_TRAIL_EN;
		writel(v, cam_dphy + CAMCAP_DPHY_DATA_LANE(i));
	}

	v = readl(cam_seninf + CAMCAP_CSI2_OFF + CAMCAP_CSI2_RESYNC);
	v = (v & ~CAMCAP_CSI2_DMY_CYCLE) | ((cycles & 0xfff) << 16);
	writel(v, cam_seninf + CAMCAP_CSI2_OFF + CAMCAP_CSI2_RESYNC);

	cam_rx_mbps = mbps;
	pr_info("rx: %u Mbps/lane, DMY_CYCLE %u, HS_TRAIL %u\n",
		mbps, cycles, hs_trail);
}

/*
 * Replay one vendor table as byte writes.  Each pair in the vendor header is a
 * single byte register, so this is a 3-byte transfer per entry; the failure
 * path reports the exact register, which is what turned up the leading-zero
 * bug that made a mode switch leave every mode register at zero.
 */
static int cam_mode_write_table(const struct cam_mode_reg *regs,
				unsigned int nregs, const char *what)
{
	unsigned int i;
	int ret;

	for (i = 0; i < nregs; i++) {
		ret = cam_sensor_write8(regs[i].reg, regs[i].val);
		if (ret) {
			pr_err("mode: %s write %u/%u 0x%04x = 0x%02x failed (%d)\n",
			       what, i + 1, nregs, regs[i].reg, regs[i].val, ret);
			return ret;
		}
		if (mode_trace)
			pr_info("mode: %s %u/%u 0x%04x = 0x%02x\n", what, i + 1,
				nregs, regs[i].reg, regs[i].val);
	}

	return 0;
}

/* Read back the registers that prove the replay landed where it should. */
static void cam_mode_verify(const struct cam_sensor_mode *m)
{
	static const unsigned int want[] = { 0x0100, 0x0101, 0x0112, 0x0114,
					     0x0306, 0x0307, 0x0340, 0x0341 };
	unsigned int i, j, v;

	for (i = 0; i < ARRAY_SIZE(want); i++) {
		unsigned int expect = 0xff;

		for (j = 0; j < m->nregs; j++)
			if (m->regs[j].reg == want[i])
				expect = m->regs[j].val;
		/* VTS is written from the descriptor, not from the table */
		if (want[i] == 0x0340)
			expect = m->vts >> 8;
		else if (want[i] == 0x0341)
			expect = m->vts & 0xff;
		if (cam_sensor_read8(want[i], &v)) {
			pr_warn("mode: verify 0x%04x read failed\n", want[i]);
		} else {
			pr_info("mode: verify 0x%04x = 0x%02x (table 0x%02x)%s\n",
				want[i], v, expect,
				expect != 0xff && v != expect ? "  <-- MISMATCH" : "");
		}
	}
}

/*
 * Replay the power-on table and the mode table over I2C, then write the VTS.
 * The vendor tables never touch the stream-on register (0x0100), so it is
 * driven here; cam_sensor_valid is cleared so the governor rewrites exposure
 * and gain from scratch on the next frame.
 */
static int cam_mode_program(int idx)
{
	const struct cam_sensor_mode *m = cam_mode_get(idx);
	int ret;

	if (!m)
		return -EINVAL;
	if (!cam_i2c_get())
		return -ENODEV;

	/*
	 * The delays are the ones the verified userspace bring-up uses (see
	 * scripts/imx582_bring.py, whose replay order and sleeps were measured
	 * on the device): the sensor's PLL settles after the power-on table,
	 * the mode table needs its own gap, and the MIPI output starts a moment
	 * after 0x0100=1.
	 */
	ret = cam_sensor_write8(0x0100, 0x00);
	msleep(20);
	if (mode_init_replay) {
		ret = cam_mode_write_table(cam_imx582_tbl_init,
					   CAMCAP_IMX582_INIT_NREGS, "init");
		msleep(20);
	}
	if (!ret)
		ret = cam_mode_write_table(m->regs, m->nregs, m->name);
	/* VTS is written separately: it is what sets the frame rate */
	if (!ret)
		ret = cam_sensor_write8(0x0340, m->vts >> 8);
	if (!ret)
		ret = cam_sensor_write8(0x0341, m->vts & 0xff);
	msleep(50);

	/*
	 * The receiver has to be timed for the new link rate before the sensor
	 * starts sending: m->mipi_mbps is what 0x030e/0x030f now holds.
	 */
	if (!ret)
		cam_rx_set_rate(m->mipi_mbps, false);

	if (!ret)
		ret = cam_sensor_write8(0x0100, 0x01);
	msleep(50);
	if (ret)
		return ret;

	cam_mode_idx = idx;
	cam_sensor_valid = false;
	pr_info("mode: %s %ux%u, HTS %u VTS %u, %u.%02u fps, exp_max %u\n",
		m->name, m->hsize, m->vsize, m->hts, m->vts,
		m->fps_x100 / 100, m->fps_x100 % 100, m->exp_max);
	if (mode_trace)
		cam_mode_verify(m);

	return 0;
}

/*
 * Switch to a mode by name (the /proc/camcap "mode" command).  bin 0 keeps the
 * current binning factor.
 */
static int cam_mode_select(const char *name, unsigned int bin)
{
	int i, ret;

	for (i = 0; i < CAMCAP_IMX582_NMODES; i++)
		if (!strcmp(cam_imx582_modes[i].name, name))
			break;
	if (i == CAMCAP_IMX582_NMODES) {
		pr_err("mode: no such mode '%s'\n", name);
		return -ENOENT;
	}
	if (cam_v4l2 && vb2_is_busy(&cam_v4l2->queue)) {
		pr_err("mode: the device is streaming\n");
		return -EBUSY;
	}
	if (!bin)
		bin = cam_bin;
	/* every table in the generated header has an even output window */
	bin = bin == 1 ? 1 : 2;

	ret = cam_mode_program(i);
	if (ret)
		return ret;
	cam_mode_geometry(&cam_imx582_modes[i], bin);
	cam_ctrl_update_exp_max();
	pr_info("mode: output %ux%u YUYV, bin %u, %u bytes/frame\n",
		out_width, out_height, cam_bin, out_width * out_height * 2);

	return 0;
}

/* ---- V4L2 ioctls ---- */

static int cam_vidioc_querycap(struct file *file, void *priv,
			       struct v4l2_capability *cap)
{
	strscpy(cap->driver, "cam_cap", sizeof(cap->driver));
	strscpy(cap->card, "MT6895 CAMSV1 (IMX582)", sizeof(cap->card));
	strscpy(cap->bus_info, "platform:cam_cap", sizeof(cap->bus_info));

	return 0;
}

static int cam_vidioc_enum_fmt(struct file *file, void *priv,
			       struct v4l2_fmtdesc *f)
{
	if (f->index)
		return -EINVAL;

	f->pixelformat = V4L2_PIX_FMT_YUYV;
	strscpy(f->description, "YUYV 4:2:2", sizeof(f->description));

	return 0;
}

static void cam_fill_pix(struct v4l2_pix_format *pix)
{
	pix->width = out_width;
	pix->height = out_height;
	pix->pixelformat = V4L2_PIX_FMT_YUYV;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = out_width * 2;
	pix->sizeimage = out_width * out_height * 2;
	pix->colorspace = V4L2_COLORSPACE_SRGB;
	pix->ycbcr_enc = V4L2_YCBCR_ENC_601;
	pix->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	pix->xfer_func = V4L2_XFER_FUNC_SRGB;
}

static int cam_vidioc_g_fmt(struct file *file, void *priv,
			    struct v4l2_format *f)
{
	cam_fill_pix(&f->fmt.pix);

	return 0;
}

/*
 * try_fmt reports the frame the application asked for when one of the sensor
 * modes can produce it, and the frame that is running otherwise.  Failing
 * outright makes camera applications give up, and silently answering with a
 * different size makes them carry on with the wrong idea of the device.
 */
static int cam_vidioc_try_fmt(struct file *file, void *priv,
			      struct v4l2_format *f)
{
	struct v4l2_pix_format *pix = &f->fmt.pix;

	pix->pixelformat = V4L2_PIX_FMT_YUYV;
	if (pix->width && pix->height &&
	    (cam_mode_match(pix->width, pix->height, 1, 0) >= 0 ||
	     cam_mode_match(pix->width, pix->height, 2, 0) >= 0)) {
		pix->field = V4L2_FIELD_NONE;
		pix->bytesperline = pix->width * 2;
		pix->sizeimage = pix->width * pix->height * 2;
		pix->colorspace = V4L2_COLORSPACE_SRGB;
		pix->ycbcr_enc = V4L2_YCBCR_ENC_601;
		pix->quantization = V4L2_QUANTIZATION_FULL_RANGE;
		pix->xfer_func = V4L2_XFER_FUNC_SRGB;
	} else {
		cam_fill_pix(pix);
	}

	return 0;
}

/*
 * s_fmt selects a sensor mode and the binning factor the converter will use.
 * The requested size is looked up at the current binning first, then at the
 * other one; a size no mode can produce leaves the device as it is, which is
 * what the single-format version of this driver always did.
 */
static int cam_vidioc_s_fmt(struct file *file, void *priv,
			    struct v4l2_format *f)
{
	struct cam_v4l2_ctx *c = video_drvdata(file);
	struct v4l2_pix_format *pix = &f->fmt.pix;
	unsigned int w = pix->width, h = pix->height;
	unsigned int bin;
	int idx, ret = 0;

	if (vb2_is_busy(&c->queue))
		return -EBUSY;

	if (pix->pixelformat && pix->pixelformat != V4L2_PIX_FMT_YUYV)
		return -EINVAL;

	/* the current binning first, then the other one */
	bin = cam_bin;
	idx = cam_mode_match(w, h, bin, 0);
	if (idx < 0) {
		bin = bin == 1 ? 2 : 1;
		idx = cam_mode_match(w, h, bin, 0);
	}
	if (idx < 0) {
		/* no mode can produce this size: keep the frame */
		cam_fill_pix(pix);
		return 0;
	}

	mutex_lock(&cam_lock);
	if (idx != cam_mode_idx)
		ret = cam_mode_program(idx);
	if (!ret) {
		cam_mode_geometry(&cam_imx582_modes[idx], bin);
		cam_ctrl_update_exp_max();
	}
	mutex_unlock(&cam_lock);
	if (ret)
		return ret;

	cam_fill_pix(pix);
	pr_info("v4l2: s_fmt %ux%u -> %s %ux%u, bin %u\n", w, h,
		cam_imx582_modes[idx].name, out_width, out_height, cam_bin);

	return 0;
}

static int cam_vidioc_enum_framesizes(struct file *file, void *priv,
				      struct v4l2_frmsizeenum *fsize)
{
	struct {
		unsigned int w, h;
	} seen[CAMCAP_IMX582_NMODES * 2];
	unsigned int n = 0;
	int i, b;

	if (fsize->pixel_format != V4L2_PIX_FMT_YUYV)
		return -EINVAL;

	for (i = 0; i < CAMCAP_IMX582_NMODES; i++) {
		for (b = 1; b <= 2; b++) {
			unsigned int w = cam_imx582_modes[i].hsize / b;
			unsigned int h = cam_imx582_modes[i].vsize / b;
			unsigned int k;

			for (k = 0; k < n; k++)
				if (seen[k].w == w && seen[k].h == h)
					break;
			if (k < n)	/* another mode already lists it */
				continue;
			if (n == fsize->index) {
				fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
				fsize->discrete.width = w;
				fsize->discrete.height = h;
				return 0;
			}
			seen[n].w = w;
			seen[n].h = h;
			n++;
		}
	}

	return -EINVAL;
}

static unsigned int cam_gcd(unsigned int a, unsigned int b)
{
	while (b) {
		unsigned int t = a % b;

		a = b;
		b = t;
	}

	return a;
}

static int cam_vidioc_enum_frameintervals(struct file *file, void *priv,
					  struct v4l2_frmivalenum *fival)
{
	unsigned int k = 0;
	int i, b;

	if (fival->pixel_format != V4L2_PIX_FMT_YUYV)
		return -EINVAL;

	for (i = 0; i < CAMCAP_IMX582_NMODES; i++) {
		for (b = 1; b <= 2; b++) {
			unsigned int fps = cam_imx582_modes[i].fps_x100;
			unsigned int g;

			if (cam_imx582_modes[i].hsize / b != fival->width ||
			    cam_imx582_modes[i].vsize / b != fival->height)
				continue;
			if (k++ != fival->index)
				continue;

			g = cam_gcd(fps, 100) ? cam_gcd(fps, 100) : 1;
			fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
			fival->discrete.numerator = 100 / g;
			fival->discrete.denominator = fps / g;

			return 0;
		}
	}

	return -EINVAL;
}

/*
 * The frame rate is a property of the selected sensor mode, so s_parm is what
 * chooses between two modes that produce the same size (4000x2256 at 30 or
 * 60 fps, 1920x1080 at 120 or 240 fps).
 */
static int cam_vidioc_g_parm(struct file *file, void *priv,
			     struct v4l2_streamparm *p)
{
	if (p->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	p->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
	p->parm.capture.timeperframe.numerator = 1;
	p->parm.capture.timeperframe.denominator = cam_mode_fps / 100;
	p->parm.capture.readbuffers = 0;

	return 0;
}

static int cam_vidioc_s_parm(struct file *file, void *priv,
			     struct v4l2_streamparm *p)
{
	struct cam_v4l2_ctx *c = video_drvdata(file);
	unsigned int num, den, want;
	int idx, ret = 0;

	if (p->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	if (vb2_is_busy(&c->queue))
		return -EBUSY;

	num = p->parm.capture.timeperframe.numerator;
	den = p->parm.capture.timeperframe.denominator;
	want = (num && den) ? (unsigned int)div_u64((u64)den * 100, num) : 0;
	idx = want ? cam_mode_match(out_width, out_height, cam_bin, want) : -1;
	if (idx < 0)
		return cam_vidioc_g_parm(file, priv, p);	/* nothing closer */

	mutex_lock(&cam_lock);
	if (idx != cam_mode_idx)
		ret = cam_mode_program(idx);
	if (!ret) {
		cam_mode_geometry(&cam_imx582_modes[idx], cam_bin);
		cam_ctrl_update_exp_max();
	}
	mutex_unlock(&cam_lock);
	if (ret)
		return ret;

	pr_info("v4l2: s_parm %u/%u -> %s, %u.%02u fps\n", num, den,
		cam_imx582_modes[idx].name, cam_mode_fps / 100,
		cam_mode_fps % 100);

	return cam_vidioc_g_parm(file, priv, p);
}

static const struct v4l2_ioctl_ops cam_v4l2_ioctl_ops = {
	.vidioc_querycap = cam_vidioc_querycap,
	.vidioc_enum_fmt_vid_cap = cam_vidioc_enum_fmt,
	.vidioc_g_fmt_vid_cap = cam_vidioc_g_fmt,
	.vidioc_try_fmt_vid_cap = cam_vidioc_try_fmt,
	.vidioc_s_fmt_vid_cap = cam_vidioc_s_fmt,
	.vidioc_enum_framesizes = cam_vidioc_enum_framesizes,
	.vidioc_enum_frameintervals = cam_vidioc_enum_frameintervals,
	.vidioc_g_parm = cam_vidioc_g_parm,
	.vidioc_s_parm = cam_vidioc_s_parm,
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

static const struct v4l2_file_operations cam_v4l2_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.read = vb2_fop_read,
	.poll = vb2_fop_poll,
	.mmap = vb2_fop_mmap,
	.unlocked_ioctl = video_ioctl2,
};

static void cam_v4l2_vdev_release(struct video_device *vdev)
{
	/* the context itself is freed by cam_v4l2_unregister() */
}

/* ---- V4L2 controls ---- */

/*
 * Control callbacks.  This is called with the control handler's own mutex held
 * (v4l2_ctrl_handler_init_class() always installs one), so nothing in here may
 * call a public v4l2_ctrl_* helper - that would deadlock on the same mutex.
 * The callbacks therefore only record intent: the capture thread applies it,
 * which also keeps I2C traffic and table rebuilds out of the ioctl path.
 *
 * While an automatic loop owns a control, writes to it are accepted and
 * ignored; the loop pushes the values it settles on back through
 * v4l2_ctrl_s_ctrl() so userspace always reads what the sensor is doing.
 */
static int cam_ctrl_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct cam_v4l2_ctx *c = cam_v4l2;
	int v = ctrl->val;

	if (!c)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE_AUTO:
		cam_ae_auto = (v == V4L2_EXPOSURE_AUTO);
		if (cam_ae_auto) {
			/* the sample from before the switch is not ours */
			cam_ae_settle = 0;
			cam_ae_mean_s = 0;
		}
		break;
	case V4L2_CID_EXPOSURE_ABSOLUTE:
		if (!cam_ae_auto)
			cam_sensor.exposure = cam_clamp_int(v, CAMCAP_EXP_MIN,
							    (int)exp_max);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		if (!cam_ae_auto)
			cam_sensor.again = cam_clamp_int(v, CAMCAP_AGAIN_MIN,
							 (int)again_max);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		if (!cam_ae_auto)
			cam_sensor.dgain = cam_clamp_int(v, CAMCAP_DGAIN_MIN,
							 (int)dgain_max);
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		cam_awb_auto = v != 0;
		break;
	case V4L2_CID_RED_BALANCE:
		if (!cam_awb_auto) {
			cam_wb_r_cur = cam_clamp_int(v, CAMCAP_WB_MIN,
						     CAMCAP_WB_MAX);
			cam_lut_dirty = true;
		}
		break;
	case V4L2_CID_BLUE_BALANCE:
		if (!cam_awb_auto) {
			cam_wb_b_cur = cam_clamp_int(v, CAMCAP_WB_MIN,
						     CAMCAP_WB_MAX);
			cam_lut_dirty = true;
		}
		break;
	case V4L2_CID_BRIGHTNESS:
		out_brightness = cam_clamp_int(v, -128, 128);
		cam_lut_dirty = true;
		break;
	case V4L2_CID_CONTRAST:
		out_contrast = cam_clamp_int(v, 0, 255);
		cam_lut_dirty = true;
		break;
	case V4L2_CID_SATURATION:
		out_saturation = cam_clamp_int(v, 0, 255);
		break;
	case V4L2_CID_FOCUS_ABSOLUTE:
		/*
		 * Recorded here, applied by the capture thread: with FOCUS_AUTO
		 * on the contrast search owns the lens and this is ignored.
		 */
		if (!af_auto)
			af_pos = cam_clamp_int(v, (int)af_min, (int)af_max);
		break;
	case V4L2_CID_FOCUS_AUTO:
		if (v && !af_auto)
			cam_af_reset();
		af_auto = v != 0;
		break;
	default:
		break;
	}

	return 0;
}

static const struct v4l2_ctrl_ops cam_ctrl_ops = {
	.s_ctrl = cam_ctrl_s_ctrl,
};

/*
 * The exposure and gain controls carry raw sensor register values, not
 * microseconds or decibels, because that is what the AE loop has to steer and
 * what the calibration in docs/CAMERA_CAPTURE_WORKING.md measured.
 */
static int cam_ctrl_init(struct cam_v4l2_ctx *c)
{
	struct v4l2_ctrl_handler *h = &c->hdl;

	v4l2_ctrl_handler_init(h, 14);

	c->ct_exp_auto = v4l2_ctrl_new_std_menu(h, &cam_ctrl_ops,
			V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_APERTURE_PRIORITY,
			0, cam_ae_auto ? V4L2_EXPOSURE_AUTO :
					 V4L2_EXPOSURE_MANUAL);
	c->ct_exposure = v4l2_ctrl_new_std(h, &cam_ctrl_ops,
			V4L2_CID_EXPOSURE_ABSOLUTE, CAMCAP_EXP_MIN,
			(int)exp_max, 1, cam_sensor.exposure);
	c->ct_again = v4l2_ctrl_new_std(h, &cam_ctrl_ops,
			V4L2_CID_ANALOGUE_GAIN, CAMCAP_AGAIN_MIN,
			(int)again_max, 1, cam_sensor.again);
	c->ct_dgain = v4l2_ctrl_new_std(h, &cam_ctrl_ops,
			V4L2_CID_DIGITAL_GAIN, CAMCAP_DGAIN_MIN,
			(int)dgain_max, 1, cam_sensor.dgain);
	c->ct_awb = v4l2_ctrl_new_std(h, &cam_ctrl_ops,
			V4L2_CID_AUTO_WHITE_BALANCE, 0, 1, 1, cam_awb_auto);
	c->ct_red = v4l2_ctrl_new_std(h, &cam_ctrl_ops, V4L2_CID_RED_BALANCE,
			CAMCAP_WB_MIN, CAMCAP_WB_MAX, 1, cam_wb_r_cur);
	c->ct_blue = v4l2_ctrl_new_std(h, &cam_ctrl_ops, V4L2_CID_BLUE_BALANCE,
			CAMCAP_WB_MIN, CAMCAP_WB_MAX, 1, cam_wb_b_cur);
	v4l2_ctrl_new_std(h, &cam_ctrl_ops, V4L2_CID_BRIGHTNESS, -128, 128, 1,
			out_brightness);
	v4l2_ctrl_new_std(h, &cam_ctrl_ops, V4L2_CID_CONTRAST, 0, 255, 1,
			out_contrast);
	v4l2_ctrl_new_std(h, &cam_ctrl_ops, V4L2_CID_SATURATION, 0, 255, 1,
			out_saturation);
	/*
	 * Focus.  FOCUS_ABSOLUTE is the VCM position the contrast search would
	 * drive the lens to; the search only reads it while FOCUS_AUTO is off.
	 */
	c->ct_focus = v4l2_ctrl_new_std(h, &cam_ctrl_ops,
			V4L2_CID_FOCUS_ABSOLUTE, (int)af_min, (int)af_max, 1,
			af_pos);
	c->ct_focus_auto = v4l2_ctrl_new_std(h, &cam_ctrl_ops,
			V4L2_CID_FOCUS_AUTO, 0, 1, 1, af_auto);

	if (h->error) {
		pr_err("v4l2: control handler error (%d)\n", h->error);
		return h->error;
	}

	/* v4l2-dev.c picks this up while registering the device node */
	c->vdev.ctrl_handler = h;
	return 0;
}

static int cam_v4l2_register(void)
{
	struct cam_v4l2_ctx *c;
	int ret;

	c = kzalloc(sizeof(*c), GFP_KERNEL);
	if (!c)
		return -ENOMEM;

	mutex_init(&c->lock);
	spin_lock_init(&c->qlock);
	INIT_LIST_HEAD(&c->queued);
	init_waitqueue_head(&c->wq);
	/*
	 * The conversion pool's wait queue must be initialised here as well.
	 * A zeroed wait_queue_head makes the first worker that sleeps walk a
	 * NULL list head inside prepare_to_wait_event() and oops the kernel.
	 */
	init_waitqueue_head(&c->conv_wq);
	init_completion(&c->conv_done);
	/*
	 * The pipeline's queue is shared by the arm thread (waiting for a slot to
	 * be released) and the converter thread (waiting for a slot to be
	 * published), so it has to exist before either of them runs.
	 */
	init_waitqueue_head(&c->pipe_wq);
	cam_v4l2 = c;		/* the control callbacks need it */

	strscpy(c->v4l2_dev.name, "cam_cap", sizeof(c->v4l2_dev.name));
	ret = v4l2_device_register(NULL, &c->v4l2_dev);
	if (ret) {
		pr_err("v4l2: v4l2_device_register failed (%d)\n", ret);
		goto err_free;
	}

	ret = cam_ctrl_init(c);
	if (ret)
		goto err_v4l2;

	c->queue.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	c->queue.io_modes = VB2_MMAP | VB2_DMABUF | VB2_READ;
	c->queue.drv_priv = c;
	c->queue.buf_struct_size = sizeof(struct cam_v4l2_buf);
	c->queue.ops = &cam_vb2_ops;
	c->queue.mem_ops = &vb2_vmalloc_memops;
	c->queue.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	c->queue.lock = &c->lock;
	c->queue.min_queued_buffers = 1;
	ret = vb2_queue_init(&c->queue);
	if (ret) {
		pr_err("v4l2: vb2_queue_init failed (%d)\n", ret);
		goto err_v4l2;
	}

	strscpy(c->vdev.name, "cam_cap", sizeof(c->vdev.name));
	c->vdev.v4l2_dev = &c->v4l2_dev;
	c->vdev.fops = &cam_v4l2_fops;
	c->vdev.ioctl_ops = &cam_v4l2_ioctl_ops;
	c->vdev.queue = &c->queue;
	c->vdev.release = cam_v4l2_vdev_release;
	c->vdev.device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING |
			      V4L2_CAP_READWRITE;
	video_set_drvdata(&c->vdev, c);

	ret = video_register_device(&c->vdev, VFL_TYPE_VIDEO, -1);
	if (ret) {
		pr_err("v4l2: video_register_device failed (%d)\n", ret);
		goto err_v4l2;
	}

	cam_v4l2 = c;
	pr_info("v4l2: registered /dev/video%d, %ux%u YUYV, %u bytes/frame\n",
		c->vdev.num, out_width, out_height, out_width * out_height * 2);

	return 0;

err_v4l2:
	v4l2_ctrl_handler_free(&c->hdl);
	v4l2_device_unregister(&c->v4l2_dev);
err_free:
	cam_v4l2 = NULL;
	kfree(c);

	return ret;
}

static void cam_v4l2_unregister(void)
{
	struct cam_v4l2_ctx *c = cam_v4l2;

	if (!c)
		return;

	cam_v4l2 = NULL;
	/*
	 * streamoff normally tears the pool down; do it again defensively so an
	 * unloaded module can never leave conversion kthreads behind.
	 */
	if (c->thread) {
		kthread_stop(c->thread);
		c->thread = NULL;
	}
	if (c->cthread) {
		kthread_stop(c->cthread);
		c->cthread = NULL;
	}
	cam_conv_pool_stop(c);
	c->pipe_used = false;
	cam_pipe_free(c);
	video_unregister_device(&c->vdev);
	v4l2_ctrl_handler_free(&c->hdl);
	v4l2_device_unregister(&c->v4l2_dev);
	kfree(c);
	pr_info("v4l2: unregistered\n");
}

/* ------------------------------------------------------------------ */
/* Init / exit                                                        */
/* ------------------------------------------------------------------ */

static int __init cam_cap_init(void)
{
	int ret;

	if (!frame_bytes || frame_bytes > (256UL * 1024 * 1024)) {
		pr_err("frame_bytes=%lu out of range\n", frame_bytes);
		return -EINVAL;
	}

	/* Raw-frame geometry of the mode the bring-up script put the sensor in. */
	cam_src_w = ALIGN(exp_hsize, 2);
	cam_src_h = ALIGN(exp_vsize, 2);
	cam_src_stride = v4l2_src_stride ? v4l2_src_stride : cam_src_w * 3 / 2;
	if (!cam_src_w || !cam_src_h || !cam_src_stride ||
	    cam_src_w > 0xffff || cam_src_h > 0xffff || cam_src_stride > 0xffff) {
		pr_err("bad source geometry: %ux%u stride %u\n",
		       cam_src_w, cam_src_h, cam_src_stride);
		return -EINVAL;
	}
	pr_info("source: %ux%u pixels, %u bytes/line, %u raw bytes/frame\n",
		cam_src_w, cam_src_h, cam_src_stride,
		cam_src_stride * cam_src_h);

	/*
	 * The live white balance starts from the module parameters and is then
	 * steered by the AWB loop, so the parameters stay the calibration input
	 * while the loop iterates on the running values.
	 */
	cam_wb_r_cur = cam_clamp_int((int)wb_r_q8, CAMCAP_WB_MIN,
				     CAMCAP_WB_MAX);
	cam_wb_b_cur = cam_clamp_int((int)wb_b_q8, CAMCAP_WB_MIN,
				     CAMCAP_WB_MAX);

	/*
	 * v4l2_enable implies the whole 12-bit full-frame configuration that the
	 * experiments settled on (docs/CAMERA_CAPTURE_WORKING.md):
	 *
	 *   PAK_MODE 0x82  -> CAMSV unpacks 12-bit pixels, 2 per 3 bytes
	 *   PAK_DBL_MODE 2 / DBL_DATA_BUS 2 -> the matching data-bus pair
	 *   route_pix_mode 2 -> the one that yields a full 3000-line frame
	 *   single_mode -> the TG stops itself after one frame (no tearing)
	 *   18 000 000 bytes -> 4000 px * 1.5 B * 3000 lines
	 *
	 * Force them here instead of making every caller repeat six parameters,
	 * and grow the buffer if the default 16 MiB is too small for a frame.
	 */
	if (v4l2_enable) {
		if (pak_mode != 0x82) {
			pr_info("v4l2_enable: forcing pak_mode=%#x (was %#x)\n",
				0x82, pak_mode);
			pak_mode = 0x82;
		}
		if (pak_dbl != 2) {
			pr_info("v4l2_enable: forcing pak_dbl=2 (was %u)\n",
				pak_dbl);
			pak_dbl = 2;
		}
		if (dbl_data_bus != 2) {
			pr_info("v4l2_enable: forcing dbl_data_bus=2 (was %u)\n",
				dbl_data_bus);
			dbl_data_bus = 2;
		}
		if (route_pix_mode != 2) {
			pr_info("v4l2_enable: forcing route_pix_mode=2 (was %u)\n",
				route_pix_mode);
			route_pix_mode = 2;
		}
		route_en = true;
		if (!single_mode) {
			pr_info("v4l2_enable: forcing single_mode=1\n");
			single_mode = true;
		}
		/*
		 * One raw frame, rounded up to the next MiB.  For the verified
		 * 4000x3000 mode that is exactly the 18 874 368 bytes every
		 * capture used; a mode with another geometry now gets its own
		 * size instead of the hard-coded one.
		 */
		{
			unsigned long need =
				ALIGN((unsigned long)cam_src_stride * cam_src_h,
				      1024UL * 1024);
			/*
			 * Leave room for the largest mode VIDIOC_S_FMT can
			 * select, so switching format never has to reallocate
			 * the frame buffer.  For the verified 4000x3000 mode
			 * this is exactly the same 18 874 368 bytes as before.
			 */
			unsigned long maxneed =
				ALIGN((unsigned long)CAMCAP_IMX582_MAX_FRAME,
				      1024UL * 1024);

			if (need < maxneed)
				need = maxneed;
			if (frame_bytes < need) {
				pr_info("v4l2_enable: growing frame_bytes %lu -> %lu\n",
					frame_bytes, need);
				frame_bytes = need;
			}
		}
		if (frame_bytes % PAGE_SIZE) {
			unsigned long aligned = ALIGN(frame_bytes, PAGE_SIZE);

			pr_info("v4l2_enable: rounding frame_bytes %lu -> %lu (iommu_map needs a page multiple)\n",
				frame_bytes, aligned);
			frame_bytes = aligned;
		}
		/*
		 * Normalise the binning, then derive the output size from the
		 * raw one.  out_width/out_height may still be given explicitly,
		 * but they have to agree with v4l2_bin.
		 */
		cam_bin = v4l2_bin == 1 ? 1 : 2;
		if (out_width * cam_bin != cam_src_w ||
		    out_height * cam_bin != cam_src_h) {
			pr_warn("v4l2_enable: %ux%u is not the /%u frame of %ux%u, using %ux%u\n",
				out_width, out_height, cam_bin,
				cam_src_w, cam_src_h,
				cam_src_w / cam_bin, cam_src_h / cam_bin);
			out_width = cam_src_w / cam_bin;
			out_height = cam_src_h / cam_bin;
		}
		/*
		 * Which table the boot parameters correspond to, so the mode
		 * commands can name it and a later S_FMT for the same size
		 * does not have to replay the register tables.  The match is on
		 * the *output* frame: a table's window is the sensor's, so
		 * 4000x3000 at bin 2 is the 2000x1500 the application sees.
		 */
		cam_mode_idx = cam_mode_match(out_width, out_height, cam_bin, 0);
		cam_mode_fps = cam_mode_idx >= 0 ?
			cam_imx582_modes[cam_mode_idx].fps_x100 : 0;
		if (cam_mode_idx >= 0)
			pr_info("mode: %s (from exp_hsize/exp_vsize)\n",
				cam_imx582_modes[cam_mode_idx].name);
		else
			pr_warn("mode: %ux%u (bin %u) matches no table in imx582_modes.h\n",
				out_width, out_height, cam_bin);
		pr_info("output: %ux%u YUYV, bin %u, %u bytes/frame\n",
			out_width, out_height, cam_bin,
			out_width * out_height * 2);
	}

	/* --- 1. physically contiguous frame buffer --- */
	/*
	 * Preferred path: dma_alloc_coherent() on a throwaway platform_device.
	 *
	 * alloc_pages_exact() bottoms out in the buddy allocator, whose largest
	 * single order here is CONFIG_ARCH_FORCE_MAX_ORDER=10, i.e. 4 MiB, and
	 * post-boot fragmentation makes even that unreliable.  A full
	 * 4000x3000 RAW10 frame is 15,000,000 bytes packed, so the buddy
	 * allocator can never satisfy it:
	 *   WARNING: mm/page_alloc.c:5280 at __alloc_frozen_pages_noprof+0x1dc
	 *   cam_cap: alloc_pages_exact(16777216) failed
	 * dma_alloc_coherent() reaches the 32 MiB CMA pool instead, through
	 * kernel/dma/direct.c:132 (dma_alloc_contiguous -> the default CMA area
	 * set up from CONFIG_CMA_SIZE_MBYTES=32).  Because this platform_device
	 * has no firmware node it is treated as non-coherent, so with
	 * CONFIG_DMA_DIRECT_REMAP=y dma-direct hands back a non-cached vmalloc
	 * mapping: the CPU can read the DMA output directly and no cache
	 * maintenance is needed.  cam_frame_pa receives the physical address for
	 * CAMSV IMGO_BASE_ADDR (0x700) / IMGO_BASE_ADDR_MSB (0x704).
	 */
	/*
	 * platform_device_register_full() is used on purpose: it sets
	 * dev.dma_mask and dev.coherent_dma_mask from pdevinfo.dma_mask inside
	 * the kernel (drivers/base/platform.c:875-879), using the kernel's own
	 * struct device layout.  This module must never write those fields
	 * itself.  The reason is historical: the build tree originally had
	 * CONFIG_PM_GENERIC_DOMAINS unset while the phone kernel has it =y, so the
	 * running struct device was 8 bytes larger than the one this module was
	 * compiled against, and a module-side write landed 8 bytes early and
	 * corrupted an unrelated field:
	 * write would land 8 bytes early and corrupt an unrelated field:
	 *   Internal error: SP/PC alignment exception: 000000008a000000 [#1]
	 *   pc : 0xffffffff
	 *   lr : dma_alloc_attrs+0xc8/0x134
	 *   init_module+0x1e4/0xf04 [cam_cap]
	 * The build tree has since been aligned with the device's own config
	 * (docs/k50_mainline_config.gz), so that skew is gone - but setting the
	 * masks kernel-side remains the right way to do it.
	 * struct platform_device_info is a small POD structure whose layout is
	 * identical under both configurations, so passing it across the boundary
	 * is safe.
	 */
	if (iommu_dev) {
		cam_setup_iommu_dev();
		if (!cam_iommu_dom)
			pr_warn("IOMMU path unavailable; the IMGO address will be a physical one and CAMSV1 will fault (larb0 port2, pa=0)\n");
	}

	/*
	 * The buffer must come from a device whose dma_alloc_coherent() returns a
	 * real physical address, i.e. one with no IOMMU.  That is the throwaway
	 * platform_device, and it is needed even when the DT/IOMMU device exists,
	 * because the IOVA is then installed by the explicit iommu_map() below.
	 */
	if (use_dma_alloc && (!cam_iommu_dev || map_iova)) {
		struct platform_device_info pdevinfo = {
			.name = CAMCAP_PDEV_NAME,
			.id = PLATFORM_DEVID_NONE,
			.dma_mask = DMA_BIT_MASK(32),
		};

		cam_pdev = platform_device_register_full(&pdevinfo);
		if (IS_ERR(cam_pdev)) {
			pr_warn("platform_device_register_full() failed (%ld); using alloc_pages_exact()\n",
				PTR_ERR(cam_pdev));
			cam_pdev = NULL;
		} else {
			cam_dma_dev = &cam_pdev->dev;
		}
	}

	if (cam_dma_dev) {
		const char *what = cam_dma_dev_is_iommu ? "disp IOMMU (IOVA)"
						       : "CMA (physical)";
		unsigned long want = frame_bytes;
		dma_addr_t dma = 0;

		for (;;) {
			cam_frame = dma_alloc_coherent(cam_dma_dev, want,
						       &dma, GFP_KERNEL);
			if (cam_frame || want <= CAMCAP_MIN_BUF)
				break;
			want = want > CAMCAP_MIN_BUF * 2 ? want >> 1
							 : CAMCAP_MIN_BUF;
		}
		if (cam_frame) {
			cam_buf_size = want;
			cam_frame_pa = (phys_addr_t)dma;
			cam_buf_phys = (phys_addr_t)dma;
			cam_buf_is_dma = true;
			/*
			 * The coherent mapping is already CPU-readable, so the
			 * buffer is its own alias.  It must never be iounmap()ed
			 * or freed with free_pages_exact().
			 */
			cam_alias = (__force void __iomem *)cam_frame;
			if (want != frame_bytes)
				pr_warn("frame_bytes=%lu unobtainable from CMA; keeping %lu bytes\n",
					frame_bytes, want);
			pr_info("buffer: %lu bytes from dma_alloc_coherent via %s, dma addr %#llx\n",
				want, what, (unsigned long long)cam_frame_pa);
		} else {
			pr_warn("dma_alloc_coherent(%lu) failed\n", frame_bytes);
		}
	}

	if (!cam_frame) {
		unsigned long want = frame_bytes;

		for (;;) {
			cam_frame = alloc_pages_exact(want, GFP_KERNEL);
			if (cam_frame || want <= CAMCAP_MIN_BUF)
				break;
			want = want > CAMCAP_MIN_BUF * 2 ? want >> 1
							 : CAMCAP_MIN_BUF;
		}
		if (!cam_frame) {
			pr_err("no contiguous block >= %lu bytes available\n",
			       CAMCAP_MIN_BUF);
			ret = -ENOMEM;
			goto err_free_pages;
		}
		cam_buf_size = want;
		if (want != frame_bytes)
			pr_warn("frame_bytes=%lu unobtainable; keeping %lu bytes (order %u)\n",
				frame_bytes, want, get_order((unsigned int)want));
	}
	if (!cam_buf_is_dma) {
		cam_frame_pa = virt_to_phys(cam_frame);
		cam_buf_phys = cam_frame_pa;
	}

	/*
	 * --- 1a-bis. find the cacheable alias of the frame buffer ---
	 *
	 * dma_alloc_coherent() on a non-coherent device hands back a *non-cached*
	 * vmalloc remap of the CMA pages (CONFIG_DMA_DIRECT_REMAP=y), and reading
	 * 18 MB of raw frame through it is the single most expensive thing in the
	 * pipeline (measured: conv 106 ms with four worker threads).  The
	 * linear-map alias of those same physical pages is cacheable, and
	 * dma_sync_single_for_cpu() invalidates exactly that alias on a
	 * non-coherent device, so the converter and the statistics read through
	 * it instead (cam_raw_src() / cam_raw_sync()).
	 *
	 * alloc_pages_exact() already returns a linear-map address, so in that
	 * case there is nothing to do and cam_frame_wb stays NULL.
	 */
	if (cam_buf_is_dma && cam_buf_phys && pfn_valid(PHYS_PFN(cam_buf_phys))) {
		cam_frame_wb = phys_to_virt(cam_buf_phys);
		pr_info("buffer: raw frame is read through cacheable alias %p\n",
			cam_frame_wb);
	}

	/*
	 * --- 1b. install the IOVA CAMSV writes to ---
	 *
	 * Doing this here rather than through dma_alloc_coherent() on the DT
	 * device is deliberate: a device with no "dma-coherent" property takes
	 * iommu_dma_alloc_noncontiguous(), whose __arm_v7s_map() aborts with
	 * -EEXIST (io-pgtable-arm-v7s.c:417) and leaves the whole thing capped at
	 * 1 MiB.  iommu_map() maps the CMA block in one call and has no such
	 * limit, so the full 15 MB frame fits.
	 */
	if (cam_iommu_dom && map_iova && cam_buf_phys) {
		int mret;

		/* The release path unmaps exactly cam_iommu_mapped bytes. */
		cam_iommu_mapped = cam_buf_size;
		mret = iommu_map(cam_iommu_dom, (dma_addr_t)map_iova,
				 cam_buf_phys, cam_buf_size,
				 IOMMU_READ | IOMMU_WRITE, GFP_KERNEL);
		if (mret) {
			pr_err("iommu_map(iova %#lx, pa %#llx, %lu) = %d\n",
			       map_iova, (unsigned long long)cam_buf_phys,
			       cam_buf_size, mret);
			cam_iommu_mapped = 0;
			/*
			 * Falling back to physical addressing only "works" for a
			 * sensor that is not streaming.  With the IOMMU on, a CAMSV
			 * write to a physical address faults (fault type=0x5,
			 * pa=0x0) and every v4l2 frame is silently black, so refuse
			 * to come up at all in that configuration.
			 */
			if (v4l2_enable) {
				pr_err("cannot stream through the IOMMU with a %lu-byte buffer; aborting\n",
				       cam_buf_size);
				ret = mret;
				goto err_free_pages;
			}
		} else {
			cam_frame_pa = (phys_addr_t)map_iova;
			cam_dma_dev_is_iommu = true;
			pr_info("iommu_map: iova %#lx -> pa %#llx, %lu bytes\n",
				map_iova, (unsigned long long)cam_buf_phys,
				cam_buf_size);
		}
	}

	/*
	 * --- 1b-bis. the extra frame buffers, for the capture/conversion
	 * pipeline ---
	 *
	 * Allocated here and not at streamon: a failure is then a line in dmesg
	 * while the module still comes up serial, instead of a failure inside
	 * VIDIOC_STREAMON that takes the caller's ioctl with it.
	 */
	cam_slots_alloc();

	/* --- 2. uncached readback alias (alloc_pages_exact path only) --- */
	/*
	 * A buffer from alloc_pages_exact() is System RAM, so both
	 * memremap(..., MEMREMAP_WC) (kernel/iomem.c refuses a non-WB mapping of
	 * IORESOURCE_SYSTEM_RAM after WARN_ONCE("memremap attempted on ram"))
	 * and ioremap_wc() (arch/arm64/mm/ioremap.c:28, "ioremap attempted on
	 * RAM pfn") are refused, as measured on the device:
	 *   cam_cap: failed to map an uncached alias of 0x0000000186400000
	 * There is no uncached alias available at all on this kernel, so keep
	 * the cached linear map and warn: the readback then needs the DMA to
	 * have completed plus an explicit cache invalidate, which this module
	 * does not do.  The dma_alloc_coherent() path above is the only one that
	 * really works here, which is why it is the default.
	 */
	if (!cam_alias) {
		if (alias_try_memremap) {
			void *m = memremap(cam_frame_pa, cam_buf_size,
					   MEMREMAP_WC);

			if (m) {
				cam_alias = (__force void __iomem *)m;
				cam_alias_is_memremap = true;
			} else {
				pr_warn("memremap(MEMREMAP_WC) refused for System RAM\n");
			}
		}
		if (!cam_alias) {
			void __iomem *wc = ioremap_wc(cam_frame_pa,
						      cam_buf_size);

			if (wc) {
				cam_alias = wc;
				cam_alias_is_memremap = false;
			}
		}
		if (!cam_alias) {
			pr_warn("no uncached alias for %pa; reading through the cached linear map, frame bytes may be stale\n",
				&cam_frame_pa);
			cam_alias = (__force void __iomem *)cam_frame;
			cam_alias_is_memremap = false;
			cam_alias_is_linear = true;
		}
	}
	if (!cam_alias) {
		pr_err("failed to map an uncached alias of %pa\n", &cam_frame_pa);
		ret = -ENOMEM;
		goto err_free_pages;
	}

	/* --- 3. readback bounce buffer --- */
	cam_bounce = kmalloc(CAMCAP_BOUNCE_SIZE, GFP_KERNEL);
	if (!cam_bounce) {
		ret = -ENOMEM;
		goto err_unmap_alias;
	}

	/* --- 4. CAMSV register block --- */
	/*
	 * Claim the MMIO window when we can.  If somebody else already holds it
	 * we still ioremap and drive the block: this module exists precisely to
	 * poke CAMSV1 from outside any existing driver.  cam_mem_region_ok
	 * records ownership so the teardown never releases a region we never
	 * got.
	 */
	cam_mem_region_ok = request_mem_region(camsv_base, CAMCAP_CAMSV_SIZE,
					       KBUILD_MODNAME) != NULL;
	if (!cam_mem_region_ok)
		pr_warn("CAMSV region %#lx already claimed, continuing anyway\n",
			camsv_base);

	cam_base = ioremap(camsv_base, CAMCAP_CAMSV_SIZE);
	if (!cam_base) {
		pr_err("ioremap(%#lx) failed\n", camsv_base);
		ret = -ENOMEM;
		goto err_release_region;
	}

	/*
	 * --- 4b. SENINF TOP, for the routing that feeds this CAMSV.
	 * Optional: without it 'arm' still programs CAMSV, it just receives no
	 * data because cam_mux 3 is never switched on.
	 */
	cam_seninf = ioremap(seninf_base, CAMCAP_SENINF_SIZE);
	if (!cam_seninf)
		pr_warn("ioremap(%#lx) failed, SENINF routing disabled\n",
			seninf_base);

	/*
	 * --- 4c. CSI D-PHY_TOP, so a mode switch can retime the receiver for
	 * the sensor's new link rate.  Optional in the same sense as 4b: without
	 * it the modes that keep 1370 Mbps per lane still switch, the 1964 Mbps
	 * ones do not.
	 */
	cam_dphy = ioremap(dphy_base, CAMCAP_DPHY_SIZE);
	if (!cam_dphy)
		pr_warn("ioremap(%#lx) failed, receiver retiming disabled\n",
			dphy_base);

	/* --- 5. procfs entries --- */
	cam_proc = proc_create("camcap", 0644, NULL, &camcap_proc_ops);
	if (!cam_proc) {
		ret = -ENOMEM;
		goto err_iounmap;
	}
	cam_proc_info = proc_create("camcap_info", 0444, NULL,
				    &camcap_info_proc_ops);
	if (!cam_proc_info) {
		ret = -ENOMEM;
		goto err_remove_proc;
	}

	/*
	 * --- 6. optional V4L2 capture device.
	 *
	 * Registers /dev/videoN backed by the vb2 vmalloc queue.  The CAMSV keeps
	 * writing its own CMA buffer (cam_frame); every dequeued v4l2 buffer is
	 * filled by CPU-side conversion from that buffer, so no DMA plumbing is
	 * needed on the v4l2 side.
	 */
	if (v4l2_enable) {
		/*
		 * Seed the exposure/gain and white-balance state the AE/AWB loops
		 * work from.  The sensor itself was brought up by userspace with
		 * these same values, so the first frame is already correct.
		 */
		cam_ae_auto = ae_enable;
		cam_awb_auto = awb_enable;
		cam_sensor.exposure = cam_clamp_int((int)exp_def, CAMCAP_EXP_MIN,
						    (int)exp_max);
		cam_sensor.again = cam_clamp_int((int)again_def, CAMCAP_AGAIN_MIN,
						 (int)again_max);
		cam_sensor.dgain = cam_clamp_int((int)dgain_def, CAMCAP_DGAIN_MIN,
						 (int)dgain_max);
		cam_lut_dirty = true;

		ret = cam_v4l2_register();
		if (ret) {
			pr_err("v4l2_enable: video device registration failed (%d)\n",
			       ret);
			goto err_remove_info;
		}
	}

	cam_arm_count = 0;
	cam_frame_ready = false;
	cam_last_ret = 0;
	cam_vf_on = !!(cam_rd(CAMSV_TG_VF_CON) & CAMSV_TG_VF_CON_VFDATA_EN);

	pr_info("loaded: CAMSV @ %#lx, buffer %lu bytes at %pa, mapping %s\n",
		camsv_base, cam_buf_size, &cam_frame_pa, cam_buf_kind());
	pr_info("loaded: TG_VF_CON=%#010x INT_STATUS=%#010x FBC_IMGO_CTL1=%#010x\n",
		cam_rd(CAMSV_TG_VF_CON), cam_rd(CAMSV_INT_STATUS),
		cam_rd(CAMSV_FBC_IMGO_CTL1));
	pr_info("loaded: use /proc/camcap (cfg/arm/stop/regs/stats) and /proc/camcap_info\n");

	return 0;

err_remove_info:
	cam_v4l2_unregister();
	proc_remove(cam_proc_info);
	cam_proc_info = NULL;
err_remove_proc:
	proc_remove(cam_proc);
	cam_proc = NULL;
err_iounmap:
	if (cam_dphy) {
		iounmap(cam_dphy);
		cam_dphy = NULL;
	}
	if (cam_seninf) {
		iounmap(cam_seninf);
		cam_seninf = NULL;
	}
	iounmap(cam_base);
	cam_base = NULL;
err_release_region:
	if (cam_mem_region_ok)
		release_mem_region(camsv_base, CAMCAP_CAMSV_SIZE);
err_unmap_alias:
	if (cam_alias && !cam_alias_is_linear && !cam_buf_is_dma) {
		if (cam_alias_is_memremap)
			memunmap((void *)cam_alias);
		else
			iounmap(cam_alias);
	}
	cam_alias = NULL;
err_free_pages:
	cam_slots_release();
	cam_unmap_iova();
	if (cam_frame) {
		if (cam_buf_is_dma && cam_dma_dev)
			dma_free_coherent(cam_dma_dev, cam_buf_size,
					  cam_frame, cam_buf_phys);
		else
			free_pages_exact(cam_frame, cam_buf_size);
		cam_frame = NULL;
	}
	if (cam_pdev) {
		platform_device_unregister(cam_pdev);
		cam_pdev = NULL;
	}
	cam_release_iommu_dev();
	return ret;
}

static void __exit cam_cap_exit(void)
{
	/*
	 * 1. take the character device away first.  If user space still has
	 *    /dev/videoN open this waits in video_unregister_device(), which is
	 *    exactly what we want: no callback can start another capture while
	 *    the rest of the module is being torn down.
	 */
	cam_v4l2_unregister();

	/* 2. stop the video frames before tearing anything down */
	if (cam_base)
		cam_cap_vf_off();

	/* 2b. release the sensor I2C adapter taken by the AE loop */
	if (cam_i2c_adap) {
		i2c_put_adapter(cam_i2c_adap);
		cam_i2c_adap = NULL;
	}

	/* 3. remove the procfs entries first: no new reader/writer can appear */
	proc_remove(cam_proc_info);
	cam_proc_info = NULL;
	proc_remove(cam_proc);
	cam_proc = NULL;

	/* 4. unmap the register blocks */
	if (cam_dphy) {
		iounmap(cam_dphy);
		cam_dphy = NULL;
	}
	if (cam_seninf) {
		iounmap(cam_seninf);
		cam_seninf = NULL;
	}
	if (cam_base) {
		iounmap(cam_base);
		cam_base = NULL;
	}
	if (cam_mem_region_ok) {
		release_mem_region(camsv_base, CAMCAP_CAMSV_SIZE);
		cam_mem_region_ok = false;
	}

	/* 5. unmap the uncached alias, then release the frame buffer */
	if (cam_alias && !cam_alias_is_linear && !cam_buf_is_dma) {
		if (cam_alias_is_memremap)
			memunmap((void *)cam_alias);
		else
			iounmap(cam_alias);
	}
	cam_alias = NULL;
	kfree(cam_bounce);
	cam_bounce = NULL;

	cam_slots_release();
	cam_unmap_iova();
	if (cam_frame) {
		if (cam_buf_is_dma && cam_dma_dev)
			dma_free_coherent(cam_dma_dev, cam_buf_size,
					  cam_frame, cam_buf_phys);
		else
			free_pages_exact(cam_frame, cam_buf_size);
		cam_frame = NULL;
	}
	if (cam_pdev) {
		platform_device_unregister(cam_pdev);
		cam_pdev = NULL;
	}
	cam_release_iommu_dev();

	pr_info("unloaded\n");
}

module_init(cam_cap_init);
module_exit(cam_cap_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("K50 CAMSV reverse-engineering effort");
MODULE_DESCRIPTION("Self-contained CAMSV1 single-frame capture module for MT6895 (Redmi K50)");
MODULE_VERSION("1.0");
