// SPDX-License-Identifier: GPL-2.0-or-later
/* Copyright (C) 2019 ASPEED Technology Inc. */
/* Copyright (C) 2019 IBM Corp. */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/mmc/host.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/spinlock.h>

#include "sdhci-pltfm.h"

#define ASPEED_SDC_INFO			0x00
#define   ASPEED_SDC_S1_MMC8		BIT(25)
#define   ASPEED_SDC_S0_MMC8		BIT(24)
#define ASPEED_SDC_PHASE		0xf4
#define   ASPEED_SDC_S1_PHASE_IN	GENMASK(25, 21)
#define   ASPEED_SDC_S0_PHASE_IN	GENMASK(20, 16)
#define	  ASPEED_SDC_S0_PHASE_IN_SHIFT	16
#define   ASPEED_SDC_S1_PHASE_OUT	GENMASK(15, 11)
#define   ASPEED_SDC_S1_PHASE_IN_EN	BIT(10)
#define   ASPEED_SDC_S1_PHASE_OUT_EN	GENMASK(9, 8)
#define   ASPEED_SDC_S0_PHASE_OUT	GENMASK(7, 3)
#define   ASPEED_SDC_S0_PHASE_IN_EN	BIT(2)
#define   ASPEED_SDC_S0_PHASE_OUT_EN	GENMASK(1, 0)
#define   ASPEED_SDC_PHASE_MAX		31

/* SDIO{10,20} */
#define ASPEED_SDC_CAP1_1_8V	       (0 * 32 + 26)
/* SDIO{14,24} */
#define ASPEED_SDC_CAP2_SDR104	       (1 * 32 + 1)
#define ASPEED_SDC_CAP2_SDR50	       (1 * 32 + 0)

#define PROBE_AFTER_ASSET_DEASSERT 0x1

struct aspeed_sdc_info {
	u32 flag;
};

struct aspeed_sdc {
	struct clk *clk;
	struct resource *res;
	struct reset_control *rst;

	spinlock_t lock;
	void __iomem *regs;
	u32 max_tap_delay_ps;
};

struct aspeed_sdhci_tap_param {
	bool valid;

#define ASPEED_SDHCI_TAP_PARAM_INVERT_CLK	BIT(4)
	u8 in;
	u8 out;
};

struct aspeed_sdhci_tap_desc {
	u32 tap_mask;
	u32 enable_mask;
	u8 enable_value;
};

struct aspeed_sdhci_phase_desc {
	struct aspeed_sdhci_tap_desc in;
	struct aspeed_sdhci_tap_desc out;
	bool non_uniform_delay;
	u32 nr_taps;
};

struct aspeed_sdhci_pdata {
	unsigned int clk_div_start;
	const struct aspeed_sdhci_phase_desc *phase_desc;
	size_t nr_phase_descs;
};

struct aspeed_sdhci {
	const struct aspeed_sdhci_pdata *pdata;
	struct aspeed_sdc *parent;
	u32 width_mask;
	struct mmc_clk_phase_map phase_map;
	const struct aspeed_sdhci_phase_desc *phase_desc;
};

static struct aspeed_sdc_info ast2600_sdc_info = {
	.flag = PROBE_AFTER_ASSET_DEASSERT
};

/*
 * The function sets the mirror register for updating
 * capbilities of the current slot.
 *
 *   slot | capability	| caps_reg | mirror_reg
 *   -----|-------------|----------|------------
 *     0  | CAP1_1_8V	| SDIO140  |   SDIO10
 *     0  | CAP2_SDR104 | SDIO144  |   SDIO14
 *     1  | CAP1_1_8V	| SDIO240  |   SDIO20
 *     1  | CAP2_SDR104 | SDIO244  |   SDIO24
 */
static void aspeed_sdc_set_slot_capability(struct sdhci_host *host, struct aspeed_sdc *sdc,
					   int capability, bool enable, u8 slot)
{
	u32 mirror_reg_offset;
	u32 cap_val;
	u8 cap_reg;

	if (slot > 1)
		return;

	cap_reg = capability / 32;
	cap_val = sdhci_readl(host, 0x40 + (cap_reg * 4));
	if (enable)
		cap_val |= BIT(capability % 32);
	else
		cap_val &= ~BIT(capability % 32);
	mirror_reg_offset = ((slot + 1) * 0x10) + (cap_reg * 4);
	writel(cap_val, sdc->regs + mirror_reg_offset);
}

static void aspeed_sdc_configure_8bit_mode(struct aspeed_sdc *sdc,
					   struct aspeed_sdhci *sdhci,
					   bool bus8)
{
	u32 info;

	/* Set/clear 8 bit mode */
	spin_lock(&sdc->lock);
	info = readl(sdc->regs + ASPEED_SDC_INFO);
	if (bus8)
		info |= sdhci->width_mask;
	else
		info &= ~sdhci->width_mask;

	writel(info, sdc->regs + ASPEED_SDC_INFO);
	spin_unlock(&sdc->lock);
}

static u32
aspeed_sdc_set_phase_tap(const struct aspeed_sdhci_tap_desc *desc,
			 u8 tap, bool enable, u32 reg)
{
	reg &= ~(desc->enable_mask | desc->tap_mask);
	if (enable) {
		reg |= tap << __ffs(desc->tap_mask);
		reg |= desc->enable_value << __ffs(desc->enable_mask);
	}

	return reg;
}

static void
aspeed_sdc_set_phase_taps(struct aspeed_sdc *sdc,
			  const struct aspeed_sdhci_phase_desc *desc,
			  const struct aspeed_sdhci_tap_param *taps)
{
	u32 reg;

	spin_lock(&sdc->lock);
	reg = readl(sdc->regs + ASPEED_SDC_PHASE);

	reg = aspeed_sdc_set_phase_tap(&desc->in, taps->in, taps->valid, reg);
	reg = aspeed_sdc_set_phase_tap(&desc->out, taps->out, taps->valid, reg);

	writel(reg, sdc->regs + ASPEED_SDC_PHASE);
	spin_unlock(&sdc->lock);
}

#define PICOSECONDS_PER_SECOND		1000000000000ULL
#define ASPEED_SDHCI_NR_TAPS		15

/* Measured value with *handwave* environmentals and static loading */
static int aspeed_sdhci_phase_to_tap(struct device *dev, unsigned long rate_hz,
				     bool invert, int phase_deg, bool non_uniform_delay, u32 nr_taps)
{
	u64 phase_period_ps;
	u64 prop_delay_ps;
	u64 clk_period_ps;
	u32 tap = 0;
	struct aspeed_sdc *sdc = dev_get_drvdata(dev->parent);

	if (sdc->max_tap_delay_ps == 0)
		return 0;

	prop_delay_ps = sdc->max_tap_delay_ps / nr_taps;
	clk_period_ps = div_u64(PICOSECONDS_PER_SECOND, (u64)rate_hz);

	/*
	 * For ast2600, if clock phase degree is negative, clock signal is
	 * output from falling edge first by default. Namely, clock signal
	 * is leading to data signal by 180 degrees at least.
	 */
	if (invert) {
		if (phase_deg >= 180)
			phase_deg -= 180;
		else
			return -EINVAL;
	}

	phase_period_ps = div_u64((u64)phase_deg * clk_period_ps, 360ULL);

	/*
	 * The delay cell is non-uniform for eMMC controller.
	 * The time period of the first tap is two times of others.
	 */
	if (non_uniform_delay && phase_period_ps > prop_delay_ps * 2) {
		phase_period_ps -= prop_delay_ps * 2;
		tap++;
	}

	tap += div_u64(phase_period_ps, prop_delay_ps);
	if (tap > ASPEED_SDHCI_NR_TAPS) {
		dev_dbg(dev,
			"Requested out of range phase tap %d for %d degrees of phase compensation at %luHz, clamping to tap %d\n",
			tap, phase_deg, rate_hz, ASPEED_SDHCI_NR_TAPS);
		tap = ASPEED_SDHCI_NR_TAPS;
	}

	if (invert)
		tap |= ASPEED_SDHCI_TAP_PARAM_INVERT_CLK;

	return tap;
}

static void
aspeed_sdhci_phases_to_taps(struct device *dev, unsigned long rate,
			    const struct mmc_clk_phase *phases,
			    struct aspeed_sdhci_tap_param *taps)
{
	int tmp_ret;
	struct sdhci_host *host = dev->driver_data;
	struct aspeed_sdhci *sdhci;

	sdhci = sdhci_pltfm_priv(sdhci_priv(host));
	taps->valid = phases->valid;

	if (!phases->valid)
		return;

	tmp_ret = aspeed_sdhci_phase_to_tap(dev, rate, phases->inv_in_deg,
					    phases->in_deg, sdhci->phase_desc->non_uniform_delay,
					    sdhci->phase_desc->nr_taps);
	if (tmp_ret < 0)
		return;

	taps->in = tmp_ret;

	tmp_ret = aspeed_sdhci_phase_to_tap(dev, rate, phases->inv_out_deg,
					    phases->out_deg, sdhci->phase_desc->non_uniform_delay,
					    sdhci->phase_desc->nr_taps);
	if (tmp_ret < 0)
		return;

	taps->out = tmp_ret;
	taps->valid = phases->valid;
}

static void
aspeed_sdhci_configure_phase(struct sdhci_host *host, unsigned long rate)
{
	struct aspeed_sdhci_tap_param _taps = {0}, *taps = &_taps;
	struct mmc_clk_phase *params;
	struct aspeed_sdhci *sdhci;
	struct device *dev;

	dev = mmc_dev(host->mmc);
	sdhci = sdhci_pltfm_priv(sdhci_priv(host));

	if (!sdhci->phase_desc)
		return;

	params = &sdhci->phase_map.phase[host->timing];
	aspeed_sdhci_phases_to_taps(dev, rate, params, taps);
	aspeed_sdc_set_phase_taps(sdhci->parent, sdhci->phase_desc, taps);
	dev_dbg(dev,
		"Using taps [%d, %d] for [%d, %d] degrees of phase correction at %luHz (%d)\n",
		taps->in & ASPEED_SDHCI_NR_TAPS,
		taps->out & ASPEED_SDHCI_NR_TAPS,
		params->in_deg, params->out_deg, rate, host->timing);
}

static void aspeed_sdhci_set_clock(struct sdhci_host *host, unsigned int clock)
{
	if (clock >= 50000000)
		aspeed_sdhci_configure_phase(host, clock);

	sdhci_set_clock(host, clock);
}

static unsigned int aspeed_sdhci_get_max_clock(struct sdhci_host *host)
{
	return sdhci_pltfm_clk_get_max_clock(host);
}

static void aspeed_sdhci_set_bus_width(struct sdhci_host *host, int width)
{
	struct sdhci_pltfm_host *pltfm_priv;
	struct aspeed_sdhci *aspeed_sdhci;
	struct aspeed_sdc *aspeed_sdc;
	u8 ctrl;

	pltfm_priv = sdhci_priv(host);
	aspeed_sdhci = sdhci_pltfm_priv(pltfm_priv);
	aspeed_sdc = aspeed_sdhci->parent;

	/* Set/clear 8-bit mode */
	aspeed_sdc_configure_8bit_mode(aspeed_sdc, aspeed_sdhci,
				       width == MMC_BUS_WIDTH_8);

	/* Set/clear 1 or 4 bit mode */
	ctrl = sdhci_readb(host, SDHCI_HOST_CONTROL);
	if (width == MMC_BUS_WIDTH_4)
		ctrl |= SDHCI_CTRL_4BITBUS;
	else
		ctrl &= ~SDHCI_CTRL_4BITBUS;
	sdhci_writeb(host, ctrl, SDHCI_HOST_CONTROL);
}

static u32 aspeed_sdhci_readl(struct sdhci_host *host, int reg)
{
	u32 val = readl(host->ioaddr + reg);

	if (unlikely(reg == SDHCI_PRESENT_STATE) &&
	    (host->mmc->caps2 & MMC_CAP2_CD_ACTIVE_HIGH))
		val ^= SDHCI_CARD_PRESENT;

	return val;
}

static void aspeed_sdhci_reset(struct sdhci_host *host, u8 mask)
{
	struct sdhci_pltfm_host *pltfm_priv;
	struct aspeed_sdhci *aspeed_sdhci;
	struct aspeed_sdc *aspeed_sdc;
	u32 save_array[8];
	u32 reg_array[] = {SDHCI_DMA_ADDRESS,
			SDHCI_BLOCK_SIZE,
			SDHCI_ARGUMENT,
			SDHCI_HOST_CONTROL,
			SDHCI_CLOCK_CONTROL,
			SDHCI_INT_ENABLE,
			SDHCI_SIGNAL_ENABLE,
			SDHCI_AUTO_CMD_STATUS};
	int i;
	u16 tran_mode;
	u32 mmc8_mode;

	pltfm_priv = sdhci_priv(host);
	aspeed_sdhci = sdhci_pltfm_priv(pltfm_priv);
	aspeed_sdc = aspeed_sdhci->parent;

	if (!IS_ERR(aspeed_sdc->rst)) {
		for (i = 0; i < ARRAY_SIZE(reg_array); i++)
			save_array[i] = sdhci_readl(host, reg_array[i]);

		tran_mode = sdhci_readw(host, SDHCI_TRANSFER_MODE);
		mmc8_mode = readl(aspeed_sdc->regs);

		reset_control_assert(aspeed_sdc->rst);
		mdelay(1);
		reset_control_deassert(aspeed_sdc->rst);
		mdelay(1);

		for (i = 0; i < ARRAY_SIZE(reg_array); i++)
			sdhci_writel(host, save_array[i], reg_array[i]);

		sdhci_writew(host, tran_mode, SDHCI_TRANSFER_MODE);
		writel(mmc8_mode, aspeed_sdc->regs);

		aspeed_sdhci_set_clock(host, host->clock);
	}

	sdhci_reset(host, mask);
}

static int aspeed_sdhci_execute_tuning(struct sdhci_host *host, u32 opcode)
{
	struct sdhci_pltfm_host *pltfm_priv;
	struct aspeed_sdhci *sdhci;
	struct aspeed_sdc *sdc;
	struct device *dev;

	u32 val, left, right, edge;
	u32 window, oldwindow = 0, center;
	u32 in_phase, out_phase, enable_mask, inverted = 0;

	dev = mmc_dev(host->mmc);
	pltfm_priv = sdhci_priv(host);
	sdhci = sdhci_pltfm_priv(pltfm_priv);
	sdc = sdhci->parent;

	out_phase = readl(sdc->regs + ASPEED_SDC_PHASE) & ASPEED_SDC_S0_PHASE_OUT;

	enable_mask = ASPEED_SDC_S0_PHASE_OUT_EN | ASPEED_SDC_S0_PHASE_IN_EN;

	/*
	 * There are two window upon clock rising and falling edge.
	 * Iterate each tap delay to find the valid window and choose the
	 * bigger one, set the tap delay at the middle of window.
	 */
	for (edge = 0; edge < 2; edge++) {
		if (edge == 1)
			inverted = ASPEED_SDHCI_TAP_PARAM_INVERT_CLK;

		val = (out_phase | enable_mask | (inverted << ASPEED_SDC_S0_PHASE_IN_SHIFT));

		/* find the left boundary */
		for (left = 0; left < ASPEED_SDHCI_NR_TAPS + 1; left++) {
			in_phase = val | (left << ASPEED_SDC_S0_PHASE_IN_SHIFT);
			writel(in_phase, sdc->regs + ASPEED_SDC_PHASE);

			if (!mmc_send_tuning(host->mmc, opcode, NULL))
				break;
		}

		/* find the right boundary */
		for (right = left + 1; right < ASPEED_SDHCI_NR_TAPS + 1; right++) {
			in_phase = val | (right << ASPEED_SDC_S0_PHASE_IN_SHIFT);
			writel(in_phase, sdc->regs + ASPEED_SDC_PHASE);

			if (mmc_send_tuning(host->mmc, opcode, NULL))
				break;
		}

		window = right - left;
		pr_debug("tuning window = %d\n", window);

		if (window > oldwindow) {
			oldwindow = window;
			center = (((right - 1) + left) / 2) | inverted;
		}
	}

	val = (out_phase | enable_mask | (center << ASPEED_SDC_S0_PHASE_IN_SHIFT));
	writel(val, sdc->regs + ASPEED_SDC_PHASE);

	pr_debug("tuning result=%x\n", val);

	return mmc_send_tuning(host->mmc, opcode, NULL);
}

static const struct sdhci_ops aspeed_sdhci_ops = {
	.read_l = aspeed_sdhci_readl,
	.set_clock = aspeed_sdhci_set_clock,
	.get_max_clock = aspeed_sdhci_get_max_clock,
	.set_bus_width = aspeed_sdhci_set_bus_width,
	.get_timeout_clock = sdhci_pltfm_clk_get_max_clock,
	.reset = aspeed_sdhci_reset,
	.set_uhs_signaling = sdhci_set_uhs_signaling,
	.platform_execute_tuning = aspeed_sdhci_execute_tuning,
};

static const struct sdhci_pltfm_data aspeed_sdhci_pdata = {
	.ops = &aspeed_sdhci_ops,
	.quirks = SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN,
	.quirks2 = SDHCI_QUIRK2_PRESET_VALUE_BROKEN,
};

static inline int aspeed_sdhci_calculate_slot(struct aspeed_sdhci *dev,
					      struct resource *res)
{
	resource_size_t delta;

	if (!res || resource_type(res) != IORESOURCE_MEM)
		return -EINVAL;

	if (res->start < dev->parent->res->start)
		return -EINVAL;

	delta = res->start - dev->parent->res->start;
	if (delta & (0x100 - 1))
		return -EINVAL;

	return (delta / 0x100) - 1;
}

static int aspeed_sdhci_probe(struct platform_device *pdev)
{
	const struct aspeed_sdhci_pdata *aspeed_pdata;
	struct device_node *np = pdev->dev.of_node;
	struct sdhci_pltfm_host *pltfm_host;
	struct aspeed_sdhci *dev;
	struct sdhci_host *host;
	struct resource *res;
	int slot;
	int ret;

	aspeed_pdata = of_device_get_match_data(&pdev->dev);
	if (!aspeed_pdata) {
		dev_err(&pdev->dev, "Missing platform configuration data\n");
		return -EINVAL;
	}

	host = sdhci_pltfm_init(pdev, &aspeed_sdhci_pdata, sizeof(*dev));
	if (IS_ERR(host))
		return PTR_ERR(host);

	pltfm_host = sdhci_priv(host);
	dev = sdhci_pltfm_priv(pltfm_host);
	dev->pdata = aspeed_pdata;
	dev->parent = dev_get_drvdata(pdev->dev.parent);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	slot = aspeed_sdhci_calculate_slot(dev, res);

	if (slot < 0)
		return slot;
	else if (slot >= 2)
		return -EINVAL;

	if (slot < dev->pdata->nr_phase_descs) {
		dev->phase_desc = &dev->pdata->phase_desc[slot];
	} else {
		dev_info(&pdev->dev,
			 "Phase control not supported for slot %d\n", slot);
		dev->phase_desc = NULL;
	}

	dev->width_mask = !slot ? ASPEED_SDC_S0_MMC8 : ASPEED_SDC_S1_MMC8;

	dev_info(&pdev->dev, "Configured for slot %d\n", slot);

	sdhci_get_of_property(pdev);

	if (of_property_read_bool(np, "mmc-hs200-1_8v") ||
	    of_property_read_bool(np, "sd-uhs-sdr50") ||
	    of_property_read_bool(np, "sd-uhs-sdr104")) {
		aspeed_sdc_set_slot_capability(host, dev->parent, ASPEED_SDC_CAP1_1_8V,
					       true, slot);
	}

	if (of_property_read_bool(np, "sd-uhs-sdr50")) {
		aspeed_sdc_set_slot_capability(host, dev->parent, ASPEED_SDC_CAP2_SDR50,
					       true, slot);
	}

	if (of_property_read_bool(np, "sd-uhs-sdr104")) {
		aspeed_sdc_set_slot_capability(host, dev->parent, ASPEED_SDC_CAP2_SDR104,
					       true, slot);
	}

	pltfm_host->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(pltfm_host->clk))
		return PTR_ERR(pltfm_host->clk);

	ret = clk_prepare_enable(pltfm_host->clk);
	if (ret) {
		dev_err(&pdev->dev, "Unable to enable SDIO clock\n");
		goto err_pltfm_free;
	}

	ret = mmc_of_parse(host->mmc);
	if (ret)
		goto err_sdhci_add;

	if (dev->phase_desc)
		mmc_of_parse_clk_phase(host->mmc, &dev->phase_map);

	ret = sdhci_add_host(host);
	if (ret)
		goto err_sdhci_add;

	return 0;

err_sdhci_add:
	clk_disable_unprepare(pltfm_host->clk);
err_pltfm_free:
	sdhci_pltfm_free(pdev);
	return ret;
}

static void aspeed_sdhci_remove(struct platform_device *pdev)
{
	struct sdhci_pltfm_host *pltfm_host;
	struct sdhci_host *host;

	host = platform_get_drvdata(pdev);
	pltfm_host = sdhci_priv(host);

	sdhci_remove_host(host, 0);

	clk_disable_unprepare(pltfm_host->clk);

	sdhci_pltfm_free(pdev);
}

static const struct aspeed_sdhci_pdata ast2400_sdhci_pdata = {
	.clk_div_start = 2,
};

static const struct aspeed_sdhci_phase_desc ast2600_sdhci_phase[] = {
	/* SDHCI/Slot 0 */
	[0] = {
		.in = {
			.tap_mask = ASPEED_SDC_S0_PHASE_IN,
			.enable_mask = ASPEED_SDC_S0_PHASE_IN_EN,
			.enable_value = 1,
		},
		.out = {
			.tap_mask = ASPEED_SDC_S0_PHASE_OUT,
			.enable_mask = ASPEED_SDC_S0_PHASE_OUT_EN,
			.enable_value = 3,
		},
		.non_uniform_delay = false,
		.nr_taps = 15,
	},
	/* SDHCI/Slot 1 */
	[1] = {
		.in = {
			.tap_mask = ASPEED_SDC_S1_PHASE_IN,
			.enable_mask = ASPEED_SDC_S1_PHASE_IN_EN,
			.enable_value = 1,
		},
		.out = {
			.tap_mask = ASPEED_SDC_S1_PHASE_OUT,
			.enable_mask = ASPEED_SDC_S1_PHASE_OUT_EN,
			.enable_value = 3,
		},
		.non_uniform_delay = false,
		.nr_taps = 15,
	},
};

static const struct aspeed_sdhci_phase_desc ast2600_emmc_phase[] = {
	/* eMMC slot 0 */
	[0] = {
		.in = {
			.tap_mask = ASPEED_SDC_S0_PHASE_IN,
			.enable_mask = ASPEED_SDC_S0_PHASE_IN_EN,
			.enable_value = 1,
		},
		.out = {
			.tap_mask = ASPEED_SDC_S0_PHASE_OUT,
			.enable_mask = ASPEED_SDC_S0_PHASE_OUT_EN,
			.enable_value = 3,
		},

		/*
		 * There are 15 taps recorded in AST2600 datasheet.
		 * But, actually, the time period of the first tap
		 * is two times of others. Thus, 16 tap is used to
		 * emulate this situation.
		 */
		.non_uniform_delay = true,
		.nr_taps = 16,
	},
};

static const struct aspeed_sdhci_pdata ast2600_sdhci_pdata = {
	.clk_div_start = 1,
	.phase_desc = ast2600_sdhci_phase,
	.nr_phase_descs = ARRAY_SIZE(ast2600_sdhci_phase),
};

static const struct aspeed_sdhci_pdata ast2600_emmc_pdata = {
	.clk_div_start = 1,
	.phase_desc = ast2600_emmc_phase,
	.nr_phase_descs = ARRAY_SIZE(ast2600_emmc_phase),
};

static const struct of_device_id aspeed_sdhci_of_match[] = {
	{ .compatible = "aspeed,ast2400-sdhci", .data = &ast2400_sdhci_pdata, },
	{ .compatible = "aspeed,ast2500-sdhci", .data = &ast2400_sdhci_pdata, },
	{ .compatible = "aspeed,ast2600-sdhci", .data = &ast2600_sdhci_pdata, },
	{ .compatible = "aspeed,ast2600-emmc", .data = &ast2600_emmc_pdata, },
	{ }
};

static struct platform_driver aspeed_sdhci_driver = {
	.driver		= {
		.name	= "sdhci-aspeed",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.of_match_table = aspeed_sdhci_of_match,
	},
	.probe		= aspeed_sdhci_probe,
	.remove_new	= aspeed_sdhci_remove,
};

static const struct of_device_id aspeed_sdc_of_match[] = {
	{ .compatible = "aspeed,ast2400-sd-controller", },
	{ .compatible = "aspeed,ast2500-sd-controller", },
	{ .compatible = "aspeed,ast2600-sd-controller", .data = &ast2600_sdc_info},
	{ }
};

MODULE_DEVICE_TABLE(of, aspeed_sdc_of_match);

static int aspeed_sdc_probe(struct platform_device *pdev)

{
	struct device_node *parent, *child;
	struct aspeed_sdc *sdc;
	const struct of_device_id *match = NULL;
	const struct aspeed_sdc_info *info = NULL;
	int ret;

	sdc = devm_kzalloc(&pdev->dev, sizeof(*sdc), GFP_KERNEL);
	if (!sdc)
		return -ENOMEM;

	spin_lock_init(&sdc->lock);

	match = of_match_device(aspeed_sdc_of_match, &pdev->dev);
	if (!match)
		return -ENODEV;

	if (match->data)
		info = match->data;

	if (info) {
		if (info->flag & PROBE_AFTER_ASSET_DEASSERT) {
			sdc->rst = devm_reset_control_get(&pdev->dev, NULL);
			if (!IS_ERR(sdc->rst)) {
				reset_control_assert(sdc->rst);
				reset_control_deassert(sdc->rst);
			}
		}
	}

	sdc->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(sdc->clk))
		return PTR_ERR(sdc->clk);

	ret = clk_prepare_enable(sdc->clk);
	if (ret) {
		dev_err(&pdev->dev, "Unable to enable SDCLK\n");
		return ret;
	}

	sdc->regs = devm_platform_get_and_ioremap_resource(pdev, 0, &sdc->res);
	if (IS_ERR(sdc->regs)) {
		ret = PTR_ERR(sdc->regs);
		goto err_clk;
	}

	ret = of_property_read_u32(pdev->dev.of_node, "aspeed-max-tap-delay",
				   &sdc->max_tap_delay_ps);
	if (ret)
		sdc->max_tap_delay_ps = 0;

	dev_set_drvdata(&pdev->dev, sdc);

	parent = pdev->dev.of_node;
	for_each_available_child_of_node(parent, child) {
		struct platform_device *cpdev;

		cpdev = of_platform_device_create(child, NULL, &pdev->dev);
		if (!cpdev) {
			of_node_put(child);
			ret = -ENODEV;
			goto err_clk;
		}
	}

	return 0;

err_clk:
	clk_disable_unprepare(sdc->clk);
	return ret;
}

static void aspeed_sdc_remove(struct platform_device *pdev)
{
	struct aspeed_sdc *sdc = dev_get_drvdata(&pdev->dev);

	clk_disable_unprepare(sdc->clk);
}

static struct platform_driver aspeed_sdc_driver = {
	.driver		= {
		.name	= "sd-controller-aspeed",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.pm	= &sdhci_pltfm_pmops,
		.of_match_table = aspeed_sdc_of_match,
	},
	.probe		= aspeed_sdc_probe,
	.remove_new	= aspeed_sdc_remove,
};

#if defined(CONFIG_MMC_SDHCI_OF_ASPEED_TEST)
#include "sdhci-of-aspeed-test.c"
#endif

static int __init aspeed_sdc_init(void)
{
	int rc;

	rc = platform_driver_register(&aspeed_sdhci_driver);
	if (rc < 0)
		return rc;

	rc = platform_driver_register(&aspeed_sdc_driver);
	if (rc < 0)
		platform_driver_unregister(&aspeed_sdhci_driver);

	return rc;
}
module_init(aspeed_sdc_init);

static void __exit aspeed_sdc_exit(void)
{
	platform_driver_unregister(&aspeed_sdc_driver);
	platform_driver_unregister(&aspeed_sdhci_driver);
}
module_exit(aspeed_sdc_exit);

MODULE_DESCRIPTION("Driver for the ASPEED SD/SDIO/SDHCI Controllers");
MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_AUTHOR("Andrew Jeffery <andrew@aj.id.au>");
MODULE_LICENSE("GPL");
