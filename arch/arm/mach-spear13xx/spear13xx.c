/*
 * arch/arm/mach-spear13xx/spear13xx.c
 *
 * SPEAr13XX machines common source file
 *
 * Copyright (C) 2012 ST Microelectronics
 * Viresh Kumar <viresh.linux@gmail.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#define pr_fmt(fmt) "SPEAr13xx: " fmt

#include <linux/amba/pl022.h>
#include <linux/clk.h>
#include <linux/dw_dmac.h>
#include <linux/err.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/stmmac.h>
#include <asm/hardware/cache-l2x0.h>
#include <asm/hardware/gic.h>
#include <asm/mach/map.h>
#include <asm/smp_twd.h>
#include <mach/dma.h>
#include <mach/generic.h>
#include <mach/spear.h>
#include <plat/adc.h>
#include <sound/designware_i2s.h>
#include <sound/pcm.h>

/* common dw_dma filter routine to be used by peripherals */
bool dw_dma_filter(struct dma_chan *chan, void *slave)
{
	struct dw_dma_slave *dws = (struct dw_dma_slave *)slave;

	if (chan->dev->dev_id == dws->dma_master_id) {
		chan->private = slave;
		return true;
	} else {
		return false;
	}
}

/* ssp device registration */
static struct dw_dma_slave ssp_dma_param[] = {
	{
		/* Tx */
		.dma_master_id = 0,
		.cfg_hi = DWC_CFGH_DST_PER(DMA_REQ_SSP0_TX),
		.cfg_lo = 0,
		.src_master = DMA_MASTER_MEMORY,
		.dst_master = DMA_MASTER_SSP0,
	}, {
		/* Rx */
		.dma_master_id = 0,
		.cfg_hi = DWC_CFGH_SRC_PER(DMA_REQ_SSP0_RX),
		.cfg_lo = 0,
		.src_master = DMA_MASTER_SSP0,
		.dst_master = DMA_MASTER_MEMORY,
	}
};

struct pl022_ssp_controller pl022_plat_data = {
	.enable_dma = 0,
	.dma_filter = dw_dma_filter,
	.dma_rx_param = &ssp_dma_param[1],
	.dma_tx_param = &ssp_dma_param[0],
};

/* adc device registeration */
static struct dw_dma_slave adc_dma_param = {
	.dma_master_id = 0,
	.cfg_hi = DWC_CFGH_SRC_PER(DMA_REQ_ADC),
	.cfg_lo = 0,
	.src_master = DMA_MASTER_ADC,
	.dst_master = DMA_MASTER_MEMORY,
};

struct adc_plat_data adc_pdata = {
	.dma_filter = dw_dma_filter,
	.dma_data = &adc_dma_param,
	.config = {CONTINUOUS_CONVERSION, EXTERNAL_VOLT, 2500, INTERNAL_SCAN,
		NORMAL_RESOLUTION, 14000000, 0},
};

/* CF device registration */
struct dw_dma_slave cf_dma_priv = {
	.dma_master_id = 0,
	.cfg_hi = 0,
	.cfg_lo = 0,
	.src_master = 0,
	.dst_master = 0,
};

/* dmac device registeration */
struct dw_dma_platform_data dmac_plat_data0 = {
	.nr_channels = 8,
	.chan_allocation_order = CHAN_ALLOCATION_DESCENDING,
	.chan_priority = CHAN_PRIORITY_DESCENDING,
	.dev_id = 0,
};

struct dw_dma_platform_data dmac_plat_data1 = {
	.nr_channels = 8,
	.chan_allocation_order = CHAN_ALLOCATION_DESCENDING,
	.chan_priority = CHAN_PRIORITY_DESCENDING,
	.dev_id = 1,
};

/* Ethernet clock initialization */
int spear13xx_eth_phy_clk_cfg(struct platform_device *pdev)
{
	int ret;
	struct clk *input_clk, *input_pclk, *phy_pclk, *phy_clk;
	struct plat_stmmacenet_data *pdata = dev_get_platdata(&pdev->dev);
	const char *phy_clk_src_name[] = {
		"phy_input_mclk",
		"phy_syn_gclk",
	};
	const char *input_clk_src_name[] = {
		"pll2_clk",
		"gmii_pad_clk",
		"osc_25m_clk",
	};
	const char *phy_clk_name[] = {
		"stmmacphy.0"
	};

	if (!pdata)
		return -EINVAL;

	/* Get the Pll-2 Clock as parent for PHY Input Clock Source */
	input_pclk = clk_get(NULL, input_clk_src_name[0]);
	if (IS_ERR(input_pclk)) {
		ret = PTR_ERR(input_pclk);
		goto fail_get_input_pclk;
	}

	/*
	 * Get the Phy Input clock source as parent for Phy clock. Default
	 * selection is gmac_phy_input_clk. This selection would be driving both
	 * the synthesizer and phy clock.
	 */
	input_clk = clk_get(NULL, phy_clk_src_name[0]);
	if (IS_ERR(input_clk)) {
		ret = PTR_ERR(input_clk);
		goto fail_get_input_clk;
	}

	/* Fetch the phy clock */
	phy_clk = clk_get(NULL, phy_clk_name[pdata->bus_id]);
	if (IS_ERR(phy_clk)) {
		ret = PTR_ERR(phy_clk);
		goto fail_get_phy_clk;
	}

	/* Set the pll-2 to 125 MHz */
	ret = clk_set_rate(input_pclk, 125000000);
	if (IS_ERR_VALUE(ret)) {
		pr_err("%s:couldn't set rate for input phy clk\n", __func__);
		goto fail_set_rate;
	}

	/* Set the Pll-2 as parent for gmac_phy_input_clk */
	ret = clk_set_parent(input_clk, input_pclk);
	if (IS_ERR_VALUE(ret)) {
		pr_err("%s:couldn't set parent for inout phy clk\n", __func__);
		goto fail_set_rate;
	}
	if (pdata->interface == PHY_INTERFACE_MODE_RMII) {
		/*
		 * For the rmii interface select gmac_phy_synth_clk
		 * as the parent and set the clock to 50 Mhz
		 */
		phy_pclk = clk_get(NULL, phy_clk_src_name[1]);
		ret = clk_set_rate(phy_pclk, 50000000);
		if (IS_ERR_VALUE(ret)) {
			pr_err("%s:couldn't set rate for phy synth clk\n",
					__func__);
			goto fail_set_rate;
		}
	} else {
		/*
		 * Set the gmac_phy_input_clk as the parent,
		 * and pll-2 is already running as parent of
		 * gmac_phy_input_clk at 125 Mhz
		 */
		phy_pclk = input_clk;
	}

	/* Select the parent for phy clock */
	ret = clk_set_parent(phy_clk, phy_pclk);
	if (IS_ERR_VALUE(ret)) {
		pr_err("%s:couldn't set parent for phy clk\n", __func__);
		goto fail_set_rate;
	}

	ret = clk_prepare_enable(phy_clk);

	return ret;
fail_set_rate:
	clk_put(phy_clk);
fail_get_phy_clk:
	clk_put(input_clk);
fail_get_input_clk:
	clk_put(input_pclk);
fail_get_input_pclk:
	return ret;
}

/*
 * configure i2s ref clk and sclk
 *
 * Depending on these parameters sclk and ref clock will be configured.
 * For sclk:
 * sclk = channel_num * data_len * sample_rate
 *
 * For ref clock:
 *
 * ref_clock = 256 * sample_rate
 */

int audio_clk_config(struct i2s_clk_config_data *config)
{
	struct clk *i2s_sclk_gclk, *i2s_ref_clk, *i2s_src_clk, *i2s_prs1_clk;
	int ret;
	u32 bclk;

	i2s_sclk_gclk = clk_get_sys(NULL, "i2s_sclk_gclk");
	if (IS_ERR(i2s_sclk_gclk)) {
		pr_err("%s:couldn't get i2s_sclk_gclk\n", __func__);
		return PTR_ERR(i2s_sclk_gclk);
	}

	i2s_ref_clk = clk_get_sys(NULL, "i2s_ref_mclk");
	if (IS_ERR(i2s_ref_clk)) {
		pr_err("%s:couldn't get i2s_ref_clk\n", __func__);
		ret = PTR_ERR(i2s_ref_clk);
		goto put_i2s_sclk_gclk;
	}

	i2s_src_clk = clk_get_sys(NULL, "i2s_src_mclk");
	if (IS_ERR(i2s_src_clk)) {
		pr_err("%s:couldn't get i2s_src_clk\n", __func__);
		ret = PTR_ERR(i2s_src_clk);
		goto put_i2s_ref_clk;
	}

	i2s_prs1_clk = clk_get_sys(NULL, "i2s_prs1_clk");
	if (IS_ERR(i2s_prs1_clk)) {
		pr_err("%s:couldn't get i2s_prs1_clk\n", __func__);
		ret = PTR_ERR(i2s_prs1_clk);
		goto put_i2s_src_clk;
	}

	if (of_machine_is_compatible("st,spear1340")) {
		if (config->sample_rate != 48000) {
			ret = clk_set_parent(i2s_ref_clk, i2s_prs1_clk);
			if (ret) {
				pr_err("%s:set_parent of ref_clk fail\n",
						__func__);
				goto put_i2s_prs1_clk;
			}
		} else {
			ret = clk_set_parent(i2s_ref_clk, i2s_src_clk);
			if (ret) {
				pr_err("%s:set_parent of ref_clk fail\n",
						__func__);
				goto put_i2s_prs1_clk;
			}
			goto config_bclk;
		}
	}

	ret = clk_set_rate(i2s_ref_clk, 256 * config->sample_rate);
	if (ret) {
		pr_err("%s:couldn't set i2s_ref_clk rate\n", __func__);
		goto put_i2s_prs1_clk;
	}

config_bclk:
	bclk = config->chan_nr * config->data_width * config->sample_rate;

	ret = clk_set_rate(i2s_sclk_gclk, bclk);
	if (ret) {
		pr_err("%s:couldn't set i2s_sclk_gclk rate\n", __func__);
		goto put_i2s_prs1_clk;
	}

	ret = clk_prepare_enable(i2s_sclk_gclk);
	if (ret) {
		pr_err("%s:enabling i2s_sclk_gclk\n", __func__);
		goto put_i2s_prs1_clk;
	}

	return 0;

put_i2s_prs1_clk:
	clk_put(i2s_prs1_clk);
put_i2s_src_clk:
	clk_put(i2s_src_clk);
put_i2s_ref_clk:
	clk_put(i2s_ref_clk);
put_i2s_sclk_gclk:
	clk_put(i2s_sclk_gclk);

	return ret;
}

void i2s_clk_init(void)
{
	int ret;
	struct clk *i2s_ref_pad_clk, *i2s_sclk_clk, *i2s_src_clk, *i2s_ref_clk;
	struct clk *src_pclk, *ref_pclk;
	char *src_pclk_name, *ref_pclk_name;

	if (of_machine_is_compatible("st,lcad") ||
			!(of_machine_is_compatible("st,spear1340"))) {
		if (of_machine_is_compatible("st,lcad"))
			src_pclk_name = "pll2_clk";
		else
			src_pclk_name = "pll3_clk";

		ref_pclk_name = "i2s_prs1_clk";

	} else {
		src_pclk_name = "i2s_src_pad_clk";
		ref_pclk_name = "i2s_src_mclk";
	}

	src_pclk = clk_get_sys(NULL, src_pclk_name);
	if (IS_ERR(src_pclk)) {
		pr_err("%s:couldn't get i2s_src parent clk\n", __func__);
		return;
	}

	if (of_machine_is_compatible("st,lcad") ||
			!(of_machine_is_compatible("st,spear1340"))) {
		/* set pll to 49.15 Mhz */
		ret = clk_set_rate(src_pclk, 49152000);
		if (ret) {
			pr_err("%s:couldn't set src_pclk rate\n", __func__);
			goto put_src_pclk;
		}
	}

	i2s_src_clk = clk_get_sys(NULL, "i2s_src_mclk");

	if (IS_ERR(i2s_src_clk)) {
		pr_err("%s:couldn't get i2s_src_clk\n", __func__);
		goto put_src_pclk;
	}

	/*
	 * After this this src_clk is correctly programmed, either to
	 * pll2, pll3 or pad.
	 */

	if (clk_set_parent(i2s_src_clk, src_pclk)) {
		pr_err("%s:set_parent for i2s src clk fail\n",
				__func__);
		goto put_i2s_src_clk;
	}

	ref_pclk = clk_get_sys(NULL, ref_pclk_name);
	if (IS_ERR(ref_pclk)) {
		pr_err("%s:couldn't get ref_pclk\n", __func__);
		goto put_i2s_src_clk;
	}

	/* program prescalar if required */
	if (of_machine_is_compatible("st,lcad") ||
			!(of_machine_is_compatible("st,spear1340"))) {
		/* set to 12.288 Mhz */
		if (clk_set_rate(ref_pclk, 12288000)) {
			pr_err("%s:ref_pclk set_rate of %s fail\n", __func__,
					ref_pclk_name);
			goto put_ref_pclk;
		}
	}

	/*
	 * After this this ref_clk is correctly programmed to 12.288 and
	 * sclk_clk to 1.536 MHz
	 */

	i2s_ref_clk = clk_get_sys(NULL, "i2s_ref_mclk");
	if (IS_ERR(i2s_ref_clk)) {
		pr_err("%s:couldn't get i2s_ref_clk\n", __func__);
		goto put_ref_pclk;
	}

	if (clk_set_parent(i2s_ref_clk, ref_pclk)) {
		pr_err("%s:set_parent of i2s_ref_clk fail\n", __func__);
		goto put_i2s_ref_clk;
	}

	i2s_sclk_clk = clk_get_sys(NULL, "i2s_sclk_clk");
	if (IS_ERR(i2s_sclk_clk)) {
		pr_err("%s:couldn't get i2s_sclk_clk\n", __func__);
		goto put_i2s_ref_clk;
	}

	i2s_ref_pad_clk = clk_get_sys(NULL, "i2s_ref_pad_clk");
	if (IS_ERR(i2s_ref_pad_clk)) {
		pr_err("%s:couldn't get i2s_ref_pad_clk\n", __func__);
		goto put_sclk_clk;
	}

	if (clk_prepare_enable(i2s_ref_pad_clk)) {
		pr_err("%s:enabling i2s_ref_pad_clk_fail\n", __func__);
		goto put_ref_pad_clk;
	}

	if (clk_prepare_enable(i2s_sclk_clk)) {
		pr_err("%s:enabling i2s_sclk_clk\n", __func__);
		goto put_ref_pad_clk;
	}
put_ref_pad_clk:
	clk_put(i2s_ref_pad_clk);
put_sclk_clk:
	clk_put(i2s_sclk_clk);
put_i2s_ref_clk:
	clk_put(i2s_ref_clk);
put_ref_pclk:
	clk_put(ref_pclk);
put_i2s_src_clk:
	clk_put(i2s_src_clk);
put_src_pclk:
	clk_put(src_pclk);
}

void spear13xx_l2x0_init(void)
{
#ifdef CONFIG_CACHE_L2X0
	/*
	 * 512KB (64KB/way), 8-way associativity, parity supported
	 *
	 * FIXME: 9th bit, of Auxillary Controller register must be set
	 * for some spear13xx devices for stable L2 operation.
	 *
	 * Enable Early BRESP, L2 prefetch for Instruction and Data,
	 * write alloc and 'Full line of zero' options
	 *
	 */

	writel_relaxed(0x06, VA_L2CC_BASE + L2X0_PREFETCH_CTRL);

	/*
	 * Program following latencies in order to make
	 * SPEAr1340 work at 600 MHz
	 */
	writel_relaxed(0x221, VA_L2CC_BASE + L2X0_TAG_LATENCY_CTRL);
	writel_relaxed(0x441, VA_L2CC_BASE + L2X0_DATA_LATENCY_CTRL);
	if (of_have_populated_dt())
		l2x0_of_init(0x70A60001, 0xfe00ffff);
	else
		l2x0_init(VA_L2CC_BASE, 0x70A60001, 0xfe00ffff);
#endif
}

/*
 * Following will create 16MB static virtual/physical mappings
 * PHYSICAL		VIRTUAL
 * 0xB3000000		0xFE000000
 * 0xE0000000		0xFD000000
 * 0xEC000000		0xFC000000
 * 0xED000000		0xFB000000
 */
struct map_desc spear13xx_io_desc[] __initdata = {
	{
		.virtual	= VA_PERIP_GRP2_BASE,
		.pfn		= __phys_to_pfn(PERIP_GRP2_BASE),
		.length		= SZ_16M,
		.type		= MT_DEVICE
	}, {
		.virtual	= VA_PERIP_GRP1_BASE,
		.pfn		= __phys_to_pfn(PERIP_GRP1_BASE),
		.length		= SZ_8M,
		.type		= MT_DEVICE
	}, {
		.virtual	= VA_SYSRAM1_BASE,
		.pfn		= __phys_to_pfn(SYSRAM1_BASE),
		.length		= SZ_1M,
		.type		= MT_MEMORY_NONCACHED
	}, {
		.virtual	= VA_A9SM_AND_MPMC_BASE,
		.pfn		= __phys_to_pfn(A9SM_AND_MPMC_BASE),
		.length		= SZ_16M,
		.type		= MT_DEVICE
	}, {
		.virtual	= (unsigned long)VA_L2CC_BASE,
		.pfn		= __phys_to_pfn(L2CC_BASE),
		.length		= SZ_4K,
		.type		= MT_DEVICE
	},
};

/* This will create static memory mapping for selected devices */
void __init spear13xx_map_io(void)
{
	iotable_init(spear13xx_io_desc, ARRAY_SIZE(spear13xx_io_desc));
}

static int spear13xx_set_wake(struct irq_data *d, unsigned int on)
{
	return 0;
}

static void __init spear13xx_clk_init(void)
{
	if (of_machine_is_compatible("st,spear1310"))
		spear1310_clk_init();
	else if (of_machine_is_compatible("st,spear1340")) {
		spear1340_clk_init();
		spear1340_sys_clk_init();
	}
	else
		pr_err("%s: Unknown machine\n", __func__);
}

static void __init spear13xx_timer_init(void)
{
	char pclk_name[] = "osc_24m_clk";
	struct clk *gpt_clk, *pclk;

	spear13xx_clk_init();

	/* get the system timer clock */
	gpt_clk = clk_get_sys("gpt0", NULL);
	if (IS_ERR(gpt_clk)) {
		pr_err("%s:couldn't get clk for gpt\n", __func__);
		BUG();
	}

	/* get the suitable parent clock for timer*/
	pclk = clk_get(NULL, pclk_name);
	if (IS_ERR(pclk)) {
		pr_err("%s:couldn't get %s as parent for gpt\n", __func__,
				pclk_name);
		BUG();
	}

	clk_set_parent(gpt_clk, pclk);
	clk_put(gpt_clk);
	clk_put(pclk);

	spear_setup_of_timer();
	twd_local_timer_of_register();
}

struct sys_timer spear13xx_timer = {
	.init = spear13xx_timer_init,
};

static const struct of_device_id gic_of_match[] __initconst = {
	{ .compatible = "arm,cortex-a9-gic", .data = gic_of_init },
	{ /* Sentinel */ }
};

void __init spear13xx_dt_init_irq(void)
{
	gic_arch_extn.irq_set_wake = spear13xx_set_wake;
	of_irq_init(gic_of_match);
}
