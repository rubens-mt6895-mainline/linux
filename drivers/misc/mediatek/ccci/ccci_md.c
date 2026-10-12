// SPDX-License-Identifier: GPL-2.0
/*
 * Minimal MediaTek MT6895 (Dimensity 8100) CCCI / modem (MD) bring-up driver.
 *
 * 这是 vendor "eccci" 栈的极简主线替代品。它只做「让 modem CPU 出复位并完成第一次
 * CCIF 握手」所必需的事：
 *
 *   1. 设置三路 modem 供电（VBUCK6 / VSRAM_MODEM / VBUCK1）；
 *   2. 清 topckgen md1_clk_mod 的 bit8|bit9，放开 md1 时钟；
 *   3. SMC MD_FLIGHT_MODE_SET(0)  -- 退出飞行模式；
 *   4. 打开 MD 电源域（scpsys 的 MD 专用上电时序）；
 *   5. SMC MD_POWER_CONFIG / MD_KERNEL_BOOT_UP -- 放行 boot vector，让 MD 起跳；
 *   6. SMC MD_POWER_CONFIG / MD_BOOT_STATUS    -- 回读启动状态；
 *   7. 遍历 LK 写在 ccci_tag_mem 保留区里的 tag 表；
 *   8. dump CCIF0 寄存器，并轮询 MD 发来的异常 init 握手。
 *
 * modem 固件由原厂 LK 装载（不是 AP 内核），所以这里没有镜像下载路径。
 *
 * 第一版刻意不注册 CCIF 中断：241/242 哪个是 AP 侧尚未在设备上证实，而未应答的
 * 电平中断会形成风暴，影响同一台设备上并行进行的指纹/相机工作。改用有界轮询。
 */

#include <linux/arm-smccc.h>
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/sizes.h>
#include <linux/soc/mediatek/mtk_sip_svc.h>
#include <linux/string.h>
#include <linux/workqueue.h>

/* ------------------------------------------------------------------ *
 * ATF SMC：stock ATF 的 CCCI 控制通道
 * （vendor eccci/platform/modem_secure_base.h，逐字核对）
 * ------------------------------------------------------------------ */
#define MTK_SIP_KERNEL_CCCI_CONTROL	MTK_SIP_SMC_CMD(0x505)

/* arm_smccc_smc() 的 x1 */
enum ccci_secure_req_id {
	MD_DBGSYS_REG_DUMP = 0,
	MD_BANK0_HW_REMAP,
	MD_BANK1_HW_REMAP,
	MD_BANK4_HW_REMAP,
	MD_SIB_HW_REMAP,
	MD_CLOCK_REQUEST,
	MD_POWER_CONFIG,
	MD_FLIGHT_MODE_SET,
	MD_HW_REMAP_LOCKED,
};

/* arm_smccc_smc() 的 x2，当 x1 == MD_POWER_CONFIG */
enum md_power_config_id {
	MD_KERNEL_BOOT_UP = 0,
	MD_LK_BOOT_UP,
	MD_CHECK_FLAG,
	MD_CHECK_DONE,
	MD_BOOT_STATUS,
	MD_LK_BOOT_PLAT = 8,
};

/* vendor 认为 MD 正常启动时 boot_status_0 是这个值 */
#define MD_BOOT_STATUS_OK	0x5443000c

/* ------------------------------------------------------------------ *
 * AP CCIF0 寄存器（vendor eccci/hif/ccif_hif_reg.h）
 * ------------------------------------------------------------------ */
#define APCCIF_CON	0x00
#define APCCIF_BUSY	0x04
#define APCCIF_START	0x08
#define APCCIF_TCHNUM	0x0c
#define APCCIF_RCHNUM	0x10
#define APCCIF_ACK	0x14
#define APCCIF_CHDATA	0x100

/* ------------------------------------------------------------------ *
 * LK tag 表（设备实测，见 BASEBAND_NOTES.md §7.6）
 *
 * 结构体 76 字节；data_offset / next_tag_offset 都是「相对 tag 区基址的绝对偏移」。
 * tag 链直接从保留区 offset 0 开始，没有头部结构。
 * ------------------------------------------------------------------ */
#define CCCI_TAG_NAME_LEN	64
#define CCCI_TAG_MAX		128
#define CCCI_TAG_END		0xffffffff

struct ccci_tag {
	char	name[CCCI_TAG_NAME_LEN];
	u32	data_offset;
	u32	data_size;
	u32	next_tag_offset;
};

/* 轮询参数：200ms x 30 = 6s */
#define CCCI_MD_POLL_MS		200
#define CCCI_MD_POLL_MAX	30

struct ccci_md {
	struct device		*dev;
	void __iomem		*ccif;
	void __iomem		*md_cfg;
	void __iomem		*tag;
	size_t			tag_size;
	struct regmap		*topckgen;
	struct regulator	*vmodem;	/* VBUCK6 */
	struct regulator	*vsram;		/* VSRAM_MODEM */
	struct regulator	*vdigrf;	/* VBUCK1 */
	struct delayed_work	poll_work;
	u32			last_busy;
	u32			last_rchnum;
	int			poll_count;
	bool			saw_md;
};

static u64 ccci_md_smc(u32 req, u32 arg, struct arm_smccc_res *res)
{
	arm_smccc_smc(MTK_SIP_KERNEL_CCCI_CONTROL, req, arg,
		      0, 0, 0, 0, 0, res);
	return res->a0;
}

static void ccci_md_dump_ccif(struct ccci_md *md, const char *why)
{
	dev_info(md->dev,
		 "[%s] CCIF0 AP: CON=%#08x BUSY=%#08x START=%#08x TCHNUM=%#08x RCHNUM=%#08x ACK=%#08x\n",
		 why,
		 readl(md->ccif + APCCIF_CON),
		 readl(md->ccif + APCCIF_BUSY),
		 readl(md->ccif + APCCIF_START),
		 readl(md->ccif + APCCIF_TCHNUM),
		 readl(md->ccif + APCCIF_RCHNUM),
		 readl(md->ccif + APCCIF_ACK));
}

/*
 * 取三路 modem 供电。
 *
 * 注意：DT 里**不能**写 `xxx-supply`。本树 mt6363 的 regulator probe 没有设置
 * config.of_node，所有 rdev 的 of_node 都是 NULL，任何 supply 属性都会让
 * of_regulator_dev_lookup() 返回 -EPROBE_DEFER，消费者永久 defer。
 * 不写 supply 属性时 DT 查找返回 -ENODEV，会落到 core.c 的
 * regulator_lookup_by_name()，按 desc->name 匹配 —— 也就是下面这些名字。
 */
static struct regulator *ccci_md_get_regulator(struct device *dev,
					       const char *name)
{
	struct regulator *r;

	r = devm_regulator_get(dev, name);
	if (IS_ERR(r)) {
		dev_warn(dev, "regulator %s 取不到: %ld（继续，电压由 LK 设好）\n",
			 name, PTR_ERR(r));
		return NULL;
	}
	return r;
}

static void ccci_md_set_rail(struct device *dev, struct regulator *r,
			     const char *name, int uv)
{
	int ret;

	if (!r)
		return;

	/*
	 * mt6363 的 rdev 是 constraints==NULL 注册的，core.c 的
	 * regulator_ops_is_valid() 对 NULL constraints 返回 false，于是
	 * regulator_set_voltage() 会以 -EPERM 失败。这不是致命错误：电压本来
	 * 就由 LK 设好了，这里只打日志。
	 */
	ret = regulator_set_voltage(r, uv, uv);
	if (ret)
		dev_warn(dev, "%s set_voltage(%d uV) 失败: %d（LK 已设好，忽略）\n",
			 name, uv, ret);

	dev_info(dev, "%s 当前 %d uV\n", name, regulator_get_voltage(r));
}

static int ccci_md_power_up(struct ccci_md *md)
{
	struct device *dev = md->dev;
	struct arm_smccc_res res;
	int ret;

	/* step 1: 三路供电。vendor 注释：VMODEM+NR+FE -> 2ms -> VSRAM_MD */
	ccci_md_set_rail(dev, md->vmodem, "VBUCK6", 825000);
	udelay(2000);
	ccci_md_set_rail(dev, md->vsram, "VSRAM_MODEM", 825000);
	ccci_md_set_rail(dev, md->vdigrf, "VBUCK1", 700000);

	/* step 2: topckgen 偏移 0 的 md1_clk_mod，清 bit8|bit9 = 放开 md1 时钟 */
	if (md->topckgen) {
		u32 val;

		ret = regmap_read(md->topckgen, 0, &val);
		if (ret)
			return dev_err_probe(dev, ret, "读 topckgen 失败\n");

		regmap_write(md->topckgen, 0, val & ~(BIT(8) | BIT(9)));
		regmap_read(md->topckgen, 0, &val);
		dev_info(dev, "topckgen md1_clk_mod = %#08x\n", val);
	}

	/* step 3: 退出飞行模式（x2 = 0 表示上电） */
	ccci_md_smc(MD_FLIGHT_MODE_SET, 0, &res);
	dev_info(dev, "MD_FLIGHT_MODE_SET(0): ret=%lu a1=%lu a2=%lu a3=%lu\n",
		 res.a0, res.a1, res.a2, res.a3);

	/*
	 * step 4: 打开 MD 电源域。等价于 vendor 的
	 * clk_prepare_enable("scp-sys-md1-main")，走 scpsys 的 MD 专用时序。
	 */
	ret = pm_runtime_resume_and_get(dev);
	if (ret)
		return dev_err_probe(dev, ret, "MD 电源域上电失败\n");
	dev_info(dev, "MD 电源域已上电\n");

	/* step 5: 放行 boot vector —— 这一步才真正让 MD 起跳 */
	ccci_md_smc(MD_POWER_CONFIG, MD_KERNEL_BOOT_UP, &res);
	dev_info(dev, "MD_KERNEL_BOOT_UP: ret=%lu boot_status_0=%#lx boot_status_1=%#lx\n",
		 res.a0, res.a1, res.a2);
	if (res.a1 != MD_BOOT_STATUS_OK)
		dev_warn(dev, "boot_status_0 不是 %#x（vendor 视其为 MD 正常启动）\n",
			 MD_BOOT_STATUS_OK);

	/* step 6: 回读启动状态 */
	ccci_md_smc(MD_POWER_CONFIG, MD_BOOT_STATUS, &res);
	dev_info(dev, "MD_BOOT_STATUS: ret=%lu boot_status_0=%#lx boot_status_1=%#lx\n",
		 res.a0, res.a1, res.a2);

	/* 顺手把 md-cfg 里的 boot stats 打出来（vendor: MD1_CFG_BOOT_STATS = base+0） */
	dev_info(dev, "md-cfg boot_stats=%#08x\n", readl(md->md_cfg));

	return 0;
}

static void ccci_md_parse_tags(struct ccci_md *md)
{
	struct device *dev = md->dev;
	struct ccci_tag tag;
	u32 off = 0;
	int i;

	if (!md->tag)
		return;

	for (i = 0; i < CCCI_TAG_MAX; i++) {
		if ((u64)off + sizeof(tag) > md->tag_size) {
			dev_warn(dev, "tag 偏移 %#x 越界\n", off);
			return;
		}

		memcpy_fromio(&tag, md->tag + off, sizeof(tag));
		tag.name[sizeof(tag.name) - 1] = '\0';	/* 防止越界读 */

		/* 未使用区域被填成 0xff，链尾就是 next == 0xffffffff */
		if (tag.next_tag_offset == CCCI_TAG_END)
			break;

		if (tag.data_size == 4 &&
		    (u64)tag.data_offset + 4 <= md->tag_size) {
			u32 v;

			memcpy_fromio(&v, md->tag + tag.data_offset, 4);
			dev_info(dev, "tag[%02d] %-26s = %u (%#x)\n",
				 i, tag.name, v, v);
		} else if (!strcmp(tag.name, "md_bank0_base") &&
			   (u64)tag.data_offset + 8 <= md->tag_size) {
			u64 v;

			memcpy_fromio(&v, md->tag + tag.data_offset, 8);
			dev_info(dev, "tag[%02d] %-26s = %#llx\n",
				 i, tag.name, v);
		} else {
			dev_info(dev, "tag[%02d] %-26s size=%u\n",
				 i, tag.name, tag.data_size);
		}

		if (!tag.next_tag_offset || tag.next_tag_offset == off)
			break;
		off = tag.next_tag_offset;
	}
}

static void ccci_md_poll_work(struct work_struct *work)
{
	struct ccci_md *md = container_of(to_delayed_work(work),
					   struct ccci_md, poll_work);
	u32 busy = readl(md->ccif + APCCIF_BUSY);
	u32 rchnum = readl(md->ccif + APCCIF_RCHNUM);

	if (busy != md->last_busy || rchnum != md->last_rchnum) {
		md->last_busy = busy;
		md->last_rchnum = rchnum;
		md->saw_md = true;
		ccci_md_dump_ccif(md, "poll");
	}

	if (++md->poll_count < CCCI_MD_POLL_MAX) {
		schedule_delayed_work(&md->poll_work,
				      msecs_to_jiffies(CCCI_MD_POLL_MS));
		return;
	}

	dev_info(md->dev,
		 "CCIF 轮询结束：BUSY=%#08x RCHNUM=%#08x（%s）\n",
		 md->last_busy, md->last_rchnum,
		 md->saw_md ? "看到 MD 侧活动" : "未看到 MD 侧活动");
}

static int ccci_md_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np;
	struct ccci_md *md;
	struct resource res;
	int ret;

	md = devm_kzalloc(dev, sizeof(*md), GFP_KERNEL);
	if (!md)
		return -ENOMEM;

	md->dev = dev;
	platform_set_drvdata(pdev, md);

	md->ccif = devm_platform_ioremap_resource_byname(pdev, "ccif");
	if (IS_ERR(md->ccif))
		return dev_err_probe(dev, PTR_ERR(md->ccif), "映射 ccif 失败\n");

	md->md_cfg = devm_platform_ioremap_resource_byname(pdev, "md-cfg");
	if (IS_ERR(md->md_cfg))
		return dev_err_probe(dev, PTR_ERR(md->md_cfg), "映射 md-cfg 失败\n");

	md->topckgen = syscon_regmap_lookup_by_phandle(dev->of_node,
						      "ccci-topckgen");
	if (IS_ERR(md->topckgen)) {
		ret = PTR_ERR(md->topckgen);
		if (ret == -EPROBE_DEFER)
			return ret;
		dev_warn(dev, "取 ccci-topckgen 失败: %d，跳过时钟放开\n", ret);
		md->topckgen = NULL;
	}

	/* LK 写的 tag 表在 reserved-memory 里（no-map），必须 ioremap */
	np = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (np) {
		if (!of_address_to_resource(np, 0, &res)) {
			md->tag_size = resource_size(&res);
			md->tag = devm_ioremap(dev, res.start, md->tag_size);
			if (!md->tag)
				dev_warn(dev, "tag 区 ioremap 失败 (%pR)\n", &res);
		} else {
			dev_warn(dev, "memory-region 没有地址\n");
		}
		of_node_put(np);
	} else {
		dev_warn(dev, "缺 memory-region，跳过 tag 解析\n");
	}

	md->vmodem = ccci_md_get_regulator(dev, "VBUCK6");
	md->vsram = ccci_md_get_regulator(dev, "VSRAM_MODEM");
	md->vdigrf = ccci_md_get_regulator(dev, "VBUCK1");

	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return dev_err_probe(dev, ret, "pm_runtime 使能失败\n");

	INIT_DELAYED_WORK(&md->poll_work, ccci_md_poll_work);

	ret = ccci_md_power_up(md);
	if (ret)
		return ret;

	ccci_md_dump_ccif(md, "init");
	ccci_md_parse_tags(md);

	md->last_busy = readl(md->ccif + APCCIF_BUSY);
	md->last_rchnum = readl(md->ccif + APCCIF_RCHNUM);
	schedule_delayed_work(&md->poll_work,
			      msecs_to_jiffies(CCCI_MD_POLL_MS));

	dev_info(dev, "CCCI/MD 驱动就绪\n");
	return 0;
}

static void ccci_md_remove(struct platform_device *pdev)
{
	struct ccci_md *md = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&md->poll_work);
	pm_runtime_put(md->dev);
}

static const struct of_device_id ccci_md_of_match[] = {
	{ .compatible = "mediatek,mt6895-ccci-md" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ccci_md_of_match);

static struct platform_driver ccci_md_driver = {
	.probe	= ccci_md_probe,
	.remove	= ccci_md_remove,
	.driver	= {
		.name		= "mtk-ccci-md",
		.of_match_table	= ccci_md_of_match,
	},
};
module_platform_driver(ccci_md_driver);

MODULE_DESCRIPTION("MediaTek MT6895 CCCI / modem bring-up");
MODULE_LICENSE("GPL");
