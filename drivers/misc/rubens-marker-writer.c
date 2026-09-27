// SPDX-License-Identifier: GPL-2.0
/*
 * rubens boot-stage marker writer (XAGR ring), built into the kernel.
 *
 * rubens (Redmi Note 11T Pro / POCO X4 GT / Redmi K50i, MT6895) boot trace.
 * Arms a 64KB "XAGR" header + circular text ring in the log_store reserved
 * DRAM region (0x7ffbf000) at the head of arm64 setup_arch - the earliest
 * point the arm64 MMU fixmap makes the region writable - and mirrors every
 * printk() (via vprintk_emit) into the ring. Markers survive an AP watchdog
 * reboot in DRAM; LK's PL_LOG_STORE restores this region into the expdb
 * partition on the next boot, so a boot hang can be located even when the
 * kernel dies before any console is up.
 *
 * Layout matches the reader (lineage_rubens drivers/misc/rubens-marker.c):
 *   u32 magic @0x0000, u32 cursor @0x0004, u32 total @0x0008,
 *   u32 stage @0x1000, text ring @0x2000 (0xE000 bytes).
 *
 * The ring lives in log_store (0x7ffbf000), NOT minirdump (0x48170000):
 * writing minirdump triggers MTK's mrdump machinery and reboots the device
 * immediately (device findings 2026-08-09). log_store is a non-secure
 * reserved area not managed by mrdump/aee, and LK's PL_LOG_STORE dumps it
 * into expdb on every boot.
 *
 * The early_ioremap() mapping used for the setup_arch window is a fixmap
 * slot that paging_init()/early_ioremap_reset() invalidate; a permanent
 * memremap() mapping is established once paging_init is done so the mirror
 * keeps writing for the whole boot (and the panic tail is captured).
 *
 * Built-in (it was a module until the vendor-ramdisk module never wrote -
 * never confirmed loaded): CONFIG_RUBENS_MARKER_WRITER is set only by the rubens
 * defconfig fragment; other devices leave it off. The module-load notifier
 * still logs every later module load, so a hang in a vendor module probe
 * leaves that module's name as the last ring entry.
 */
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/memremap.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/printk.h>
#include <linux/rubens_marker.h>

#include <asm/cacheflush.h>
#include <asm/memory.h>

/* MTK log_store compat: LK dumps the kernel log into expdb on the next boot
 * only if the sram_log_header at 0x11DF00 carries a valid klog_addr/size and
 * the NEED_SAVE_TO_EMMC flag (layout matches MiCode rubens-s-oss
 * drivers/misc/mediatek/log_store/log_store_kernel.h). PL/LK pre-fill the
 * header sigs every boot; mainline only has to point klog at its log_buf. */
#define RUBENS_LS_SRAM_PA		0x11DF00UL
#define RUBENS_LS_SRAM_SZ		0x100UL
#define RUBENS_LS_SRAM_SIG	0xABCD1234U	/* sram_log_header.sig */
#define RUBENS_LS_DRAM_SIG	0x5678EF90U	/* dram_buf_header.sig */
/* BUFF_VALID|CAN_FREE|NEED_SAVE_TO_EMMC|ARRAY_BUFF|BUFF_READY|BUFF_EARLY_PRINTK */
#define RUBENS_LS_FLAG_SAVE	0x627U

/* sram_log_header field offsets (u32) */
#define RUBENS_LS_OFF_SIG		0x00
#define RUBENS_LS_OFF_DRAM_SIG	0x0C
#define RUBENS_LS_OFF_DRAM_FLAG	0x10
#define RUBENS_LS_OFF_KLOG_ADDR	0x24
#define RUBENS_LS_OFF_KLOG_SIZE	0x28

char *log_buf_addr_get(void);
u32 log_buf_len_get(void);

static void __init rubens_mtk_logstore_arm(void)
{
	void __iomem *sh;
	u32 sig, dsig, flag, kpa, klen;

	sh = early_ioremap(RUBENS_LS_SRAM_PA, RUBENS_LS_SRAM_SZ);
	if (!sh) {
		pr_err("rubens-marker-writer: logstore sram early_ioremap failed\n");
		return;
	}
	sig = readl(sh + RUBENS_LS_OFF_SIG);
	dsig = readl(sh + RUBENS_LS_OFF_DRAM_SIG);
	if (sig != RUBENS_LS_SRAM_SIG || dsig != RUBENS_LS_DRAM_SIG) {
		pr_err("rubens-marker-writer: logstore sram sig mismatch %08x/%08x\n",
		       sig, dsig);
		goto unmap;
	}
	kpa = (u32)__virt_to_phys((u64)log_buf_addr_get());
	klen = log_buf_len_get();
	writel(kpa, sh + RUBENS_LS_OFF_KLOG_ADDR);
	writel(klen, sh + RUBENS_LS_OFF_KLOG_SIZE);
	flag = readl(sh + RUBENS_LS_OFF_DRAM_FLAG);
	flag |= RUBENS_LS_FLAG_SAVE;
	writel(flag, sh + RUBENS_LS_OFF_DRAM_FLAG);
	pr_info("rubens-marker-writer: logstore compat armed klog=0x%08x len=%u flag=0x%08x\n",
		kpa, klen, flag);
	rubens_marker_put("logstore compat armed klog=0x%08x len=%u flag=0x%08x\n",
			kpa, klen, flag);
unmap:
	early_iounmap(sh, RUBENS_LS_SRAM_SZ);
}

/* log_store reserved region: non-secure, survives the WDT reboot in DRAM */
#define MTK_MRDUMP_PA	0x7ffbf000UL
#define MTK_MRDUMP_SZ	0x10000UL
#define RUBENS_RING_OFF	0x2000U
#define RUBENS_RING_SZ	0xE000U
#define RUBENS_MAGIC	0x52474158UL	/* "XAGR" */
#define RUBENS_MAX_MSG	256

static void __iomem *rubens_mr_base;
static bool rubens_early_map;	/* still using the early_ioremap fixmap slot */
static bool rubens_direct_map;	/* using the direct map (needs cache clean) */

static void rubens_marker_ring_write(const char *buf, int n)
{
	void __iomem *ring;
	u32 cursor;
	int i;

	if (!rubens_mr_base)
		return;
	/* Re-assert our magic on every write: MTK aee/mrdump_mini may rewrite
	 * the region header; the next write restores it. */
	writel(RUBENS_MAGIC, rubens_mr_base + 0x0000);
	cursor = readl(rubens_mr_base + 0x0004);
	ring = rubens_mr_base + RUBENS_RING_OFF;
	for (i = 0; i < n; i++)
		writeb(buf[i], ring + ((cursor + i) % RUBENS_RING_SZ));
	writel(cursor + n, rubens_mr_base + 0x0004);
	writel(readl(rubens_mr_base + 0x0008) + n, rubens_mr_base + 0x0008);
	/*
	 * The direct-map alias is write-back cached. A WDT hard reset does
	 * NOT flush the CPU cache, so push the writes to DRAM now or the
	 * ring is lost before LK can restore it into expdb.
	 */
	if (rubens_direct_map)
		dcache_clean_poc((unsigned long)rubens_mr_base,
				 (unsigned long)rubens_mr_base + MTK_MRDUMP_SZ);
}

void rubens_marker_put(const char *fmt, ...)
{
	va_list args;
	char buf[RUBENS_MAX_MSG];
	int n;

	va_start(args, fmt);
	n = vscnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);
	if (n <= 0)
		return;
	rubens_marker_ring_write(buf, n);
}
EXPORT_SYMBOL_GPL(rubens_marker_put);

void rubens_marker_stage(u32 stage)
{
	if (!rubens_mr_base)
		return;
	writel(stage, rubens_mr_base + 0x1000);
	rubens_marker_put("stage=%u\n", stage);
}
EXPORT_SYMBOL_GPL(rubens_marker_stage);

/* Mirrors every printk() into the ring while armed; called from
 * vprintk_emit. Must be safe in any printk context: no printk, no locks, no
 * allocation. The ring is lock-free: concurrent writers may occasionally
 * interleave, acceptable for a diagnostic ring. Every 64th message also
 * writes a MIRROR:n heartbeat so the LK log_store recovery (which dumps
 * this region into expdb on the next boot) proves the mirror is live. */
static unsigned int rubens_mirror_cnt;

void rubens_marker_early_printk(const char *fmt, va_list args)
{
	va_list ap;
	char buf[RUBENS_MAX_MSG];
	int n;

	if (!rubens_mr_base)
		return;
	va_copy(ap, args);
	n = vscnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);
	if (n <= 0)
		return;
	rubens_marker_ring_write(buf, n);
	if (++rubens_mirror_cnt % 64 == 0) {
		char hb[32];
		int hn = snprintf(hb, sizeof(hb), "MIRROR:%u\n", rubens_mirror_cnt);

		rubens_marker_ring_write(hb, hn);
	}
}

/* Called from the head of arm64 setup_arch, right after
 * early_fixmap_init()/early_ioremap_init() - the earliest point the arm64
 * MMU maps the reserved region (it is not in the linear map before
 * paging_init). Everything printed from here on lands in the ring. */
void __init rubens_marker_early_init(void)
{
	rubens_mr_base = early_ioremap(MTK_MRDUMP_PA, MTK_MRDUMP_SZ);
	if (!rubens_mr_base) {
		pr_info("rubens-marker-writer: early_ioremap 0x%08lx failed\n",
			MTK_MRDUMP_PA);
		return;
	}
	rubens_early_map = true;
	/* fresh ring per boot: only the last boot's markers survive */
	writel(RUBENS_MAGIC, rubens_mr_base + 0x0000);
	writel(0, rubens_mr_base + 0x0004);
	writel(0, rubens_mr_base + 0x0008);
	writel(0, rubens_mr_base + 0x1000);
	pr_info("rubens-marker-writer: XAGR ring armed at 0x%08lx\n",
		MTK_MRDUMP_PA);
	rubens_marker_stage(1);
	rubens_mtk_logstore_arm();
}

static int rubens_marker_module_nb(struct notifier_block *nb,
				 unsigned long action, void *data)
{
	struct module *mod = data;

	switch (action) {
	case MODULE_STATE_COMING:
	case MODULE_STATE_LIVE:
		rubens_marker_put("module: %s\n", mod->name);
		break;
	default:
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block rubens_marker_nb = {
	.notifier_call = rubens_marker_module_nb,
};

static int __init rubens_marker_w_late_init(void)
{
	void *perm;

	/*
	 * Replace the setup_arch-era early_ioremap fixmap mapping (which
	 * paging_init()/early_ioremap_reset() invalidates) with a stable
	 * direct-map alias. log_store is reserved System RAM (not no-map),
	 * so memremap() returns the direct map pointer; the ring writes go
	 * through it and dcache_clean_poc() pushes them to DRAM so a WDT
	 * hard reset doesn't lose them. Only swap if still on the early
	 * slot; memremap() needs the real MM/paging_init.
	 */
	if (rubens_early_map) {
		perm = memremap(MTK_MRDUMP_PA, MTK_MRDUMP_SZ, MEMREMAP_WB);
		if (perm) {
			rubens_mr_base = perm;
			rubens_early_map = false;
			rubens_direct_map = true;
			dcache_clean_poc((unsigned long)rubens_mr_base,
					 (unsigned long)rubens_mr_base +
					 MTK_MRDUMP_SZ);
			rubens_marker_put("marker writer: permanent mapping armed\n");
		}
	}
	rubens_marker_put("marker writer built-in init\n");
	register_module_notifier(&rubens_marker_nb);
	return 0;
}
core_initcall(rubens_marker_w_late_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("rubens boot-stage marker writer (XAGR ring), built-in");
