/*
 * arch/arm/mach-spear3xx/clock.c
 *
 * SPEAr3xx machines clock framework source file
 *
 * Copyright (C) 2009 ST Microelectronics
 * Viresh Kumar<viresh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <mach/hardware.h>
#include <mach/misc_regs.h>
#include <plat/clock.h>

/* root clks */
/* 32 KHz oscillator clock */
static struct clk osc_32k_clk = {
	.flags = ALWAYS_ENABLED | SYSTEM_CLK,
	.rate = 32000,
};

/* 24 MHz oscillator clock */
static struct clk osc_24m_clk = {
	.flags = ALWAYS_ENABLED | SYSTEM_CLK,
	.rate = 24000000,
};

/* clock derived from 32 KHz osc clk */
/* rtc clock */
static struct clk rtc_clk = {
	.pclk = &osc_32k_clk,
	.en_reg = VA_PERIP1_CLK_ENB,
	.en_reg_bit = RTC_CLK_ENB,
	.recalc = &follow_parent,
};

/* clock derived from 24 MHz osc clk */
/* vco masks structure */
static struct vco_clk_masks vco_masks = {
	.mode_mask = PLL_MODE_MASK,
	.mode_shift = PLL_MODE_SHIFT,
	.norm_fdbk_m_mask = PLL_NORM_FDBK_M_MASK,
	.norm_fdbk_m_shift = PLL_NORM_FDBK_M_SHIFT,
	.dith_fdbk_m_mask = PLL_DITH_FDBK_M_MASK,
	.dith_fdbk_m_shift = PLL_DITH_FDBK_M_SHIFT,
	.div_p_mask = PLL_DIV_P_MASK,
	.div_p_shift = PLL_DIV_P_SHIFT,
	.div_n_mask = PLL_DIV_N_MASK,
	.div_n_shift = PLL_DIV_N_SHIFT,
	.pll_lock_mask = PLL_LOCK_MASK,
	.pll_lock_shift = PLL_LOCK_SHIFT,
};

/* vco rate configuration table, in ascending order of rates */
struct vco_rate_tbl vco_rtbl[] = {
	{.mode = 0, .m = 0x53, .n = 0x0C, .p = 0x1}, /* vco 332 & pll 166 MHz */
	{.mode = 0, .m = 0x85, .n = 0x0C, .p = 0x1}, /* vco 532 & pll 266 MHz */
	{.mode = 0, .m = 0xA6, .n = 0x0C, .p = 0x1}, /* vco 664 & pll 332 MHz */
};

/* vco1 configuration structure */
static struct vco_clk_config vco1_config = {
	.mode_reg = VA_PLL1_CTR,
	.cfg_reg = VA_PLL1_FRQ,
	.masks = &vco_masks,
};

/* vco1 clock */
static struct clk vco1_clk = {
	.flags = ENABLED_ON_INIT | SYSTEM_CLK,
	.pclk = &osc_24m_clk,
	.en_reg = VA_PLL1_CTR,
	.en_reg_bit = PLL_ENABLE,
	.calc_rate = &vco_calc_rate,
	.recalc = &vco_clk_recalc,
	.set_rate = &vco_clk_set_rate,
	.rate_config = {vco_rtbl, ARRAY_SIZE(vco_rtbl), 2},
	.private_data = &vco1_config,
};

/* pll1 clock */
static struct clk pll1_clk = {
	.flags = ALWAYS_ENABLED | SYSTEM_CLK,
	.pclk = &vco1_clk,
	.recalc = &pll_clk_recalc,
};

/* vco2 configuration structure */
static struct vco_clk_config vco2_config = {
	.mode_reg = VA_PLL2_CTR,
	.cfg_reg = VA_PLL2_FRQ,
	.masks = &vco_masks,
};

/* vco2 clock */
static struct clk vco2_clk = {
	.flags = ENABLED_ON_INIT | SYSTEM_CLK,
	.pclk = &osc_24m_clk,
	.en_reg = VA_PLL2_CTR,
	.en_reg_bit = PLL_ENABLE,
	.calc_rate = &vco_calc_rate,
	.recalc = &vco_clk_recalc,
	.set_rate = &vco_clk_set_rate,
	.rate_config = {vco_rtbl, ARRAY_SIZE(vco_rtbl), 2},
	.private_data = &vco2_config,
};

/* pll2 clock */
static struct clk pll2_clk = {
	.flags = ALWAYS_ENABLED | SYSTEM_CLK,
	.pclk = &vco2_clk,
	.recalc = &pll_clk_recalc,
};

/* PLL3 48 MHz clock */
static struct clk pll3_48m_clk = {
	.flags = ALWAYS_ENABLED | SYSTEM_CLK,
	.pclk = &osc_24m_clk,
	.rate = 48000000,
};

/* watch dog timer clock */
static struct clk wdt_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &osc_24m_clk,
	.recalc = &follow_parent,
};

/* clock derived from pll1 clk */
/* cpu clock */
static struct clk cpu_clk = {
	.flags = ALWAYS_ENABLED | SYSTEM_CLK,
	.pclk = &pll1_clk,
	.recalc = &follow_parent,
};

/* ahb masks structure */
static struct bus_clk_masks ahb_masks = {
	.mask = PLL_HCLK_RATIO_MASK,
	.shift = PLL_HCLK_RATIO_SHIFT,
};

/* ahb configuration structure */
static struct bus_clk_config ahb_config = {
	.reg = VA_CORE_CLK_CFG,
	.masks = &ahb_masks,
};

/* ahb rate configuration table, in ascending order of rates */
struct bus_rate_tbl bus_rtbl[] = {
	{.div = 3}, /* == parent divided by 4 */
	{.div = 2}, /* == parent divided by 3 */
	{.div = 1}, /* == parent divided by 2 */
	{.div = 0}, /* == parent divided by 1 */
};

/* ahb clock */
static struct clk ahb_clk = {
	.flags = ALWAYS_ENABLED | SYSTEM_CLK,
	.pclk = &pll1_clk,
	.calc_rate = &bus_calc_rate,
	.recalc = &bus_clk_recalc,
	.set_rate = &bus_clk_set_rate,
	.rate_config = {bus_rtbl, ARRAY_SIZE(bus_rtbl), 2},
	.private_data = &ahb_config,
};

/* auxiliary synthesizers masks */
static struct aux_clk_masks aux_masks = {
	.eq_sel_mask = AUX_EQ_SEL_MASK,
	.eq_sel_shift = AUX_EQ_SEL_SHIFT,
	.eq1_mask = AUX_EQ1_SEL,
	.eq2_mask = AUX_EQ2_SEL,
	.xscale_sel_mask = AUX_XSCALE_MASK,
	.xscale_sel_shift = AUX_XSCALE_SHIFT,
	.yscale_sel_mask = AUX_YSCALE_MASK,
	.yscale_sel_shift = AUX_YSCALE_SHIFT,
};

/* uart synth configurations */
static struct aux_clk_config uart_synth_config = {
	.synth_reg = VA_UART_CLK_SYNT,
	.masks = &aux_masks,
};

/* aux rate configuration table, in ascending order of rates */
struct aux_rate_tbl aux_rtbl[] = {
	/* For PLL1 = 332 MHz */
	{.xscale = 1, .yscale = 81, .eq = 0}, /* 2.049 MHz */
	{.xscale = 1, .yscale = 59, .eq = 0}, /* 2.822 MHz */
	{.xscale = 2, .yscale = 81, .eq = 0}, /* 4.098 MHz */
	{.xscale = 3, .yscale = 89, .eq = 0}, /* 5.644 MHz */
	{.xscale = 4, .yscale = 81, .eq = 0}, /* 8.197 MHz */
	{.xscale = 4, .yscale = 59, .eq = 0}, /* 11.254 MHz */
	{.xscale = 2, .yscale = 27, .eq = 0}, /* 12.296 MHz */
	{.xscale = 2, .yscale = 8, .eq = 0}, /* 41.5 MHz */
	{.xscale = 2, .yscale = 4, .eq = 0}, /* 83 MHz */
	{.xscale = 1, .yscale = 2, .eq = 1}, /* 166 MHz */
};

/* uart synth clock */
static struct clk uart_synth_clk = {
	.en_reg = VA_UART_CLK_SYNT,
	.en_reg_bit = AUX_SYNT_ENB,
	.pclk = &pll1_clk,
	.calc_rate = &aux_calc_rate,
	.recalc = &aux_clk_recalc,
	.set_rate = &aux_clk_set_rate,
	.rate_config = {aux_rtbl, ARRAY_SIZE(aux_rtbl), 8},
	.private_data = &uart_synth_config,
};

/* uart parents */
static struct pclk_info uart_pclk_info[] = {
	{
		.pclk = &uart_synth_clk,
		.pclk_val = AUX_CLK_PLL1_VAL,
	}, {
		.pclk = &pll3_48m_clk,
		.pclk_val = AUX_CLK_PLL3_VAL,
	},
};

/* uart parent select structure */
static struct pclk_sel uart_pclk_sel = {
	.pclk_info = uart_pclk_info,
	.pclk_count = ARRAY_SIZE(uart_pclk_info),
	.pclk_sel_reg = VA_PERIP_CLK_CFG,
	.pclk_sel_mask = UART_CLK_MASK,
};

/* uart clock */
static struct clk uart_clk = {
	.en_reg = VA_PERIP1_CLK_ENB,
	.en_reg_bit = UART_CLK_ENB,
	.pclk_sel = &uart_pclk_sel,
	.pclk_sel_shift = UART_CLK_SHIFT,
	.recalc = &follow_parent,
};

/* firda configurations */
static struct aux_clk_config firda_synth_config = {
	.synth_reg = VA_FIRDA_CLK_SYNT,
	.masks = &aux_masks,
};

/* firda synth clock */
static struct clk firda_synth_clk = {
	.en_reg = VA_FIRDA_CLK_SYNT,
	.en_reg_bit = AUX_SYNT_ENB,
	.pclk = &pll1_clk,
	.calc_rate = &aux_calc_rate,
	.recalc = &aux_clk_recalc,
	.set_rate = &aux_clk_set_rate,
	.rate_config = {aux_rtbl, ARRAY_SIZE(aux_rtbl), 8},
	.private_data = &firda_synth_config,
};

/* firda parents */
static struct pclk_info firda_pclk_info[] = {
	{
		.pclk = &firda_synth_clk,
		.pclk_val = AUX_CLK_PLL1_VAL,
	}, {
		.pclk = &pll3_48m_clk,
		.pclk_val = AUX_CLK_PLL3_VAL,
	},
};

/* firda parent select structure */
static struct pclk_sel firda_pclk_sel = {
	.pclk_info = firda_pclk_info,
	.pclk_count = ARRAY_SIZE(firda_pclk_info),
	.pclk_sel_reg = VA_PERIP_CLK_CFG,
	.pclk_sel_mask = FIRDA_CLK_MASK,
};

/* firda clock */
static struct clk firda_clk = {
	.en_reg = VA_PERIP1_CLK_ENB,
	.en_reg_bit = FIRDA_CLK_ENB,
	.pclk_sel = &firda_pclk_sel,
	.pclk_sel_shift = FIRDA_CLK_SHIFT,
	.recalc = &follow_parent,
};

/* gpt synthesizer masks */
static struct gpt_clk_masks gpt_masks = {
	.mscale_sel_mask = GPT_MSCALE_MASK,
	.mscale_sel_shift = GPT_MSCALE_SHIFT,
	.nscale_sel_mask = GPT_NSCALE_MASK,
	.nscale_sel_shift = GPT_NSCALE_SHIFT,
};

/* gpt rate configuration table, in ascending order of rates */
struct gpt_rate_tbl gpt_rtbl[] = {
	/* For pll1 = 332 MHz */
	{.mscale = 4, .nscale = 0}, /* 41.5 MHz */
	{.mscale = 2, .nscale = 0}, /* 55.3 MHz */
	{.mscale = 1, .nscale = 0}, /* 83 MHz */
};

/* gpt0 synth clk config*/
static struct gpt_clk_config gpt0_synth_config = {
	.synth_reg = VA_PRSC1_CLK_CFG,
	.masks = &gpt_masks,
};

/* gpt synth clock */
static struct clk gpt0_synth_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &pll1_clk,
	.calc_rate = &gpt_calc_rate,
	.recalc = &gpt_clk_recalc,
	.set_rate = &gpt_clk_set_rate,
	.rate_config = {gpt_rtbl, ARRAY_SIZE(gpt_rtbl), 2},
	.private_data = &gpt0_synth_config,
};

/* gpt parents */
static struct pclk_info gpt0_pclk_info[] = {
	{
		.pclk = &gpt0_synth_clk,
		.pclk_val = AUX_CLK_PLL1_VAL,
	}, {
		.pclk = &pll3_48m_clk,
		.pclk_val = AUX_CLK_PLL3_VAL,
	},
};

/* gpt parent select structure */
static struct pclk_sel gpt0_pclk_sel = {
	.pclk_info = gpt0_pclk_info,
	.pclk_count = ARRAY_SIZE(gpt0_pclk_info),
	.pclk_sel_reg = VA_PERIP_CLK_CFG,
	.pclk_sel_mask = GPT_CLK_MASK,
};

/* gpt0 timer clock */
static struct clk gpt0_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk_sel = &gpt0_pclk_sel,
	.pclk_sel_shift = GPT0_CLK_SHIFT,
	.recalc = &follow_parent,
};

/* gpt1 synth clk configurations */
static struct gpt_clk_config gpt1_synth_config = {
	.synth_reg = VA_PRSC2_CLK_CFG,
	.masks = &gpt_masks,
};

/* gpt1 synth clock */
static struct clk gpt1_synth_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &pll1_clk,
	.calc_rate = &gpt_calc_rate,
	.recalc = &gpt_clk_recalc,
	.set_rate = &gpt_clk_set_rate,
	.rate_config = {gpt_rtbl, ARRAY_SIZE(gpt_rtbl), 2},
	.private_data = &gpt1_synth_config,
};

static struct pclk_info gpt1_pclk_info[] = {
	{
		.pclk = &gpt1_synth_clk,
		.pclk_val = AUX_CLK_PLL1_VAL,
	}, {
		.pclk = &pll3_48m_clk,
		.pclk_val = AUX_CLK_PLL3_VAL,
	},
};

/* gpt parent select structure */
static struct pclk_sel gpt1_pclk_sel = {
	.pclk_info = gpt1_pclk_info,
	.pclk_count = ARRAY_SIZE(gpt1_pclk_info),
	.pclk_sel_reg = VA_PERIP_CLK_CFG,
	.pclk_sel_mask = GPT_CLK_MASK,
};

/* gpt1 timer clock */
static struct clk gpt1_clk = {
	.en_reg = VA_PERIP1_CLK_ENB,
	.en_reg_bit = GPT1_CLK_ENB,
	.pclk_sel = &gpt1_pclk_sel,
	.pclk_sel_shift = GPT1_CLK_SHIFT,
	.recalc = &follow_parent,
};

/* gpt2 synth clk configurations */
static struct gpt_clk_config gpt2_synth_config = {
	.synth_reg = VA_PRSC3_CLK_CFG,
	.masks = &gpt_masks,
};

/* gpt1 synth clock */
static struct clk gpt2_synth_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &pll1_clk,
	.calc_rate = &gpt_calc_rate,
	.recalc = &gpt_clk_recalc,
	.set_rate = &gpt_clk_set_rate,
	.rate_config = {gpt_rtbl, ARRAY_SIZE(gpt_rtbl), 2},
	.private_data = &gpt2_synth_config,
};

static struct pclk_info gpt2_pclk_info[] = {
	{
		.pclk = &gpt2_synth_clk,
		.pclk_val = AUX_CLK_PLL1_VAL,
	}, {
		.pclk = &pll3_48m_clk,
		.pclk_val = AUX_CLK_PLL3_VAL,
	},
};

/* gpt parent select structure */
static struct pclk_sel gpt2_pclk_sel = {
	.pclk_info = gpt2_pclk_info,
	.pclk_count = ARRAY_SIZE(gpt2_pclk_info),
	.pclk_sel_reg = VA_PERIP_CLK_CFG,
	.pclk_sel_mask = GPT_CLK_MASK,
};

/* gpt2 timer clock */
static struct clk gpt2_clk = {
	.en_reg = VA_PERIP1_CLK_ENB,
	.en_reg_bit = GPT2_CLK_ENB,
	.pclk_sel = &gpt2_pclk_sel,
	.pclk_sel_shift = GPT2_CLK_SHIFT,
	.recalc = &follow_parent,
};

/*
 * There are four general purpose synths present in SPEAr3xx, they can be used
 * by fixed part as well as RAS. For enabling them in RAS there are separate
 * gating bits present in RAS_CLK_ENB register
 */
/* RAS synth configurations */
static struct aux_clk_config synth0_config = {
	.synth_reg = VA_RAS0_CLK_SYNT,
	.masks = &aux_masks,
};

static struct aux_clk_config synth1_config = {
	.synth_reg = VA_RAS1_CLK_SYNT,
	.masks = &aux_masks,
};

static struct aux_clk_config synth2_config = {
	.synth_reg = VA_RAS2_CLK_SYNT,
	.masks = &aux_masks,
};

static struct aux_clk_config synth3_config = {
	.synth_reg = VA_RAS3_CLK_SYNT,
	.masks = &aux_masks,
};

/* synth clock */
static struct clk synth0_clk = {
	.en_reg = VA_RAS0_CLK_SYNT,
	.en_reg_bit = AUX_SYNT_ENB,
	.pclk = &pll1_clk,
	.calc_rate = &aux_calc_rate,
	.recalc = &aux_clk_recalc,
	.set_rate = &aux_clk_set_rate,
	.rate_config = {aux_rtbl, ARRAY_SIZE(aux_rtbl), 7},
	.private_data = &synth0_config,
};

static struct clk synth1_clk = {
	.en_reg = VA_RAS1_CLK_SYNT,
	.en_reg_bit = AUX_SYNT_ENB,
	.pclk = &pll1_clk,
	.calc_rate = &aux_calc_rate,
	.recalc = &aux_clk_recalc,
	.set_rate = &aux_clk_set_rate,
	.rate_config = {aux_rtbl, ARRAY_SIZE(aux_rtbl), 7},
	.private_data = &synth1_config,
};

/* synth 2 and 3 parents */
static struct pclk_info synth2_3_pclk_info[] = {
	{
		.pclk = &pll1_clk,
		.pclk_val = RAS_SYNTH2_3_CLK_PLL1_VAL,
	}, {
		.pclk = &pll2_clk,
		.pclk_val = RAS_SYNTH2_3_CLK_PLL2_VAL,
	},
};

/* synth2_3 parent select structure */
static struct pclk_sel synth2_3_pclk_sel = {
	.pclk_info = synth2_3_pclk_info,
	.pclk_count = ARRAY_SIZE(synth2_3_pclk_info),
	.pclk_sel_reg = VA_CORE_CLK_CFG,
	.pclk_sel_mask = RAS_SYNTH2_3_CLK_MASK,
};

/* synth2_3 clock */
static struct clk synth2_3_pclk = {
	.flags = ALWAYS_ENABLED,
	.pclk_sel = &synth2_3_pclk_sel,
	.pclk_sel_shift = RAS_SYNTH2_3_CLK_SHIFT,
	.recalc = &follow_parent,
};

static struct clk synth2_clk = {
	.en_reg = VA_RAS2_CLK_SYNT,
	.en_reg_bit = AUX_SYNT_ENB,
	.pclk = &synth2_3_pclk,
	.calc_rate = &aux_calc_rate,
	.recalc = &aux_clk_recalc,
	.set_rate = &aux_clk_set_rate,
	.rate_config = {aux_rtbl, ARRAY_SIZE(aux_rtbl), 7},
	.private_data = &synth2_config,
};

static struct clk synth3_clk = {
	.en_reg = VA_RAS3_CLK_SYNT,
	.en_reg_bit = AUX_SYNT_ENB,
	.pclk = &synth2_3_pclk,
	.calc_rate = &aux_calc_rate,
	.recalc = &aux_clk_recalc,
	.set_rate = &aux_clk_set_rate,
	.rate_config = {aux_rtbl, ARRAY_SIZE(aux_rtbl), 7},
	.private_data = &synth3_config,
};

static struct clk ras_synth0_clk = {
	.en_reg = VA_RAS_CLK_ENB,
	.en_reg_bit = RAS_SYNT0_CLK_ENB,
	.pclk = &synth0_clk,
	.recalc = &follow_parent,
};

static struct clk ras_synth1_clk = {
	.en_reg = VA_RAS_CLK_ENB,
	.en_reg_bit = RAS_SYNT1_CLK_ENB,
	.pclk = &synth1_clk,
	.recalc = &follow_parent,
};

static struct clk ras_synth2_clk = {
	.en_reg = VA_RAS_CLK_ENB,
	.en_reg_bit = RAS_SYNT2_CLK_ENB,
	.pclk = &synth2_clk,
	.recalc = &follow_parent,
};

static struct clk ras_synth3_clk = {
	.en_reg = VA_RAS_CLK_ENB,
	.en_reg_bit = RAS_SYNT3_CLK_ENB,
	.pclk = &synth3_clk,
	.recalc = &follow_parent,
};

/* clock derived from pll3 clk */
/* usbh clock */
static struct clk usbh_clk = {
	.pclk = &pll3_48m_clk,
	.en_reg = VA_PERIP1_CLK_ENB,
	.en_reg_bit = USBH_CLK_ENB,
	.recalc = &follow_parent,
};

/* usbd clock */
static struct clk usbd_clk = {
	.pclk = &pll3_48m_clk,
	.en_reg = VA_PERIP1_CLK_ENB,
	.en_reg_bit = USBD_CLK_ENB,
	.recalc = &follow_parent,
};

/* clock derived from ahb clk */
/* ahb multiplied by 2 clock */
static struct clk ahbmult2_clk = {
	.flags = ALWAYS_ENABLED | SYSTEM_CLK,
	.pclk = &ahb_clk,
	.recalc = &ahbmult2_clk_recalc,
};

/* ddr clock */
struct ddr_rate_tbl ddr_rate_tbl = {
	.minrate = 166000000,
	.maxrate = 332000000,
};

static struct pclk_info ddr_pclk_info[] = {
	{
		.pclk = &ahb_clk,
		.pclk_val = MCTR_CLK_HCLK_VAL,
	}, {
		.pclk = &ahbmult2_clk,
		.pclk_val = MCTR_CLK_2HCLK_VAL,
	}, {
		.pclk = &pll2_clk,
		.pclk_val = MCTR_CLK_PLL2_VAL,
	},
};

/* ddr parent select structure */
static struct pclk_sel ddr_pclk_sel = {
	.pclk_info = ddr_pclk_info,
	.pclk_count = ARRAY_SIZE(ddr_pclk_info),
	.pclk_sel_reg = VA_PLL_CLK_CFG,
	.pclk_sel_mask = MCTR_CLK_MASK,
};

static struct clk ddr_clk = {
	.flags = ALWAYS_ENABLED | SYSTEM_CLK,
	.recalc = &follow_parent,
	.pclk_sel = &ddr_pclk_sel,
	.pclk_sel_shift = MCTR_CLK_SHIFT,
	.private_data = &ddr_rate_tbl,
};

/* apb masks structure */
static struct bus_clk_masks apb_masks = {
	.mask = HCLK_PCLK_RATIO_MASK,
	.shift = HCLK_PCLK_RATIO_SHIFT,
};

/* apb configuration structure */
static struct bus_clk_config apb_config = {
	.reg = VA_CORE_CLK_CFG,
	.masks = &apb_masks,
};

/* apb clock */
static struct clk apb_clk = {
	.flags = ALWAYS_ENABLED | SYSTEM_CLK,
	.pclk = &ahb_clk,
	.calc_rate = &bus_calc_rate,
	.recalc = &bus_clk_recalc,
	.set_rate = &bus_clk_set_rate,
	.rate_config = {bus_rtbl, ARRAY_SIZE(bus_rtbl), 2},
	.private_data = &apb_config,
};

/*amem clock: Must be enabled for accessing RAM from RAS peripherals */
static struct clk amem_clk = {
	.en_reg = VA_AMEM_CLK_CFG,
	.en_reg_bit = AMEM_CLK_ENB,
	.pclk = &ahb_clk,
	.recalc = &follow_parent,
};

/* i2c clock */
static struct clk i2c_clk = {
	.pclk = &ahb_clk,
	.en_reg = VA_PERIP1_CLK_ENB,
	.en_reg_bit = I2C_CLK_ENB,
	.recalc = &follow_parent,
};

/* dma clock */
static struct clk dma_clk = {
	.pclk = &ahb_clk,
	.en_reg = VA_PERIP1_CLK_ENB,
	.en_reg_bit = DMA_CLK_ENB,
	.recalc = &follow_parent,
};

/* jpeg clock */
static struct clk jpeg_clk = {
	.pclk = &ahb_clk,
	.en_reg = VA_PERIP1_CLK_ENB,
	.en_reg_bit = JPEG_CLK_ENB,
	.recalc = &follow_parent,
};

/* gmac clock */
static struct clk gmac_clk = {
	.pclk = &ahb_clk,
	.en_reg = VA_PERIP1_CLK_ENB,
	.en_reg_bit = GMAC_CLK_ENB,
	.recalc = &follow_parent,
};

/* smi clock */
static struct clk smi_clk = {
	.pclk = &ahb_clk,
	.en_reg = VA_PERIP1_CLK_ENB,
	.en_reg_bit = SMI_CLK_ENB,
	.recalc = &follow_parent,
};

/* c3 clock */
static struct clk c3_clk = {
	.pclk = &ahb_clk,
	.en_reg = VA_PERIP1_CLK_ENB,
	.en_reg_bit = C3_CLK_ENB,
	.recalc = &follow_parent,
};

/* clock derived from apb clk */
/* adc clock */
static struct clk adc_clk = {
	.pclk = &apb_clk,
	.en_reg = VA_PERIP1_CLK_ENB,
	.en_reg_bit = ADC_CLK_ENB,
	.recalc = &follow_parent,
};

/* ssp clock */
static struct clk ssp0_clk = {
	.pclk = &apb_clk,
	.en_reg = VA_PERIP1_CLK_ENB,
	.en_reg_bit = SSP_CLK_ENB,
	.recalc = &follow_parent,
};

/* gpio clock */
static struct clk gpio_clk = {
	.pclk = &apb_clk,
	.en_reg = VA_PERIP1_CLK_ENB,
	.en_reg_bit = GPIO_CLK_ENB,
	.recalc = &follow_parent,
};

static struct clk dummy_apb_pclk;

/* spear300 machine specific clock structures */
#ifdef CONFIG_CPU_SPEAR300
/* gpio1 clock */
static struct clk gpio1_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &apb_clk,
	.recalc = &follow_parent,
};

/* fsmc0 clock */
static struct clk fsmc0_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &ahb_clk,
	.recalc = &follow_parent,
};

/* fsmc1 clock */
static struct clk fsmc1_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &ahb_clk,
	.recalc = &follow_parent,
};

/* fsmc2 clock */
static struct clk fsmc2_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &ahb_clk,
	.recalc = &follow_parent,
};

/* fsmc3 clock */
static struct clk fsmc3_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &ahb_clk,
	.recalc = &follow_parent,
};

/* sdhci clk */
static struct clk spear300_sdhci_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &ahb_clk,
	.recalc = &follow_parent,
};

/* keyboard clock */
static struct clk kbd_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &apb_clk,
	.recalc = &follow_parent,
};
#endif

/* spear310 machine specific clock structures */
#ifdef CONFIG_CPU_SPEAR310
/* tdm clock */
static struct clk spear310_tdm_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &ahb_clk,
	.recalc = &follow_parent,
};

/* uart1 clock */
static struct clk spear310_uart1_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &apb_clk,
	.recalc = &follow_parent,
};

/* uart2 clock */
static struct clk spear310_uart2_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &apb_clk,
	.recalc = &follow_parent,
};

/* uart3 clock */
static struct clk spear310_uart3_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &apb_clk,
	.recalc = &follow_parent,
};

/* uart4 clock */
static struct clk spear310_uart4_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &apb_clk,
	.recalc = &follow_parent,
};

/* uart5 clock */
static struct clk spear310_uart5_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &apb_clk,
	.recalc = &follow_parent,
};
#endif

/* spear320 machine specific clock structures */
#ifdef CONFIG_CPU_SPEAR320
/* can0 clock */
static struct clk can0_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &apb_clk,
	.recalc = &follow_parent,
};

/* can1 clock */
static struct clk can1_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &apb_clk,
	.recalc = &follow_parent,
};

/* i2c1 clock */
static struct clk i2c1_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &ahb_clk,
	.recalc = &follow_parent,
};

/* smii external pad clock */
static struct clk smii_125m_pad = {
	.flags = ALWAYS_ENABLED,
	.rate = 125000000,
};

/* mii parent clocks: smii for 320 */
static struct pclk_info smii_pclk_info[] = {
	{
		.pclk = &smii_125m_pad,
		.pclk_val = SMII_PCLK_VAL_PAD,
	}, {
		.pclk = &pll2_clk,
		.pclk_val = SMII_PCLK_VAL_PLL2,
	}, {
		.pclk = &ras_synth0_clk,
		.pclk_val = SMII_PCLK_VAL_SYNTH0,
	},
};

/* mii parent select structure */
static struct pclk_sel smii_pclk_sel = {
	.pclk_info = smii_pclk_info,
	.pclk_count = ARRAY_SIZE(smii_pclk_info),
	.pclk_sel_reg = VA_SPEAR320_CONTROL_REG,
	.pclk_sel_mask = SMII_PCLK_MASK,
};

/* mii clock */
static struct clk spear320_smii_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk_sel = &smii_pclk_sel,
	.pclk_sel_shift = SMII_PCLK_SHIFT,
	.recalc = &follow_parent,
};

/* ssp1 clock */
static struct clk ssp1_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &apb_clk,
	.recalc = &follow_parent,
};

/* ssp2 clock */
static struct clk ssp2_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &apb_clk,
	.recalc = &follow_parent,
};

/* pwm clock */
static struct clk pwm_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &apb_clk,
	.recalc = &follow_parent,
};

/* uartx parent clocks */
static struct pclk_info uartx_pclk_info[] = {
	{
		.pclk = &apb_clk,
		.pclk_val = SPEAR320_UARTX_PCLK_VAL_APB,
	}, {
		.pclk = &ras_synth1_clk,
		.pclk_val = SPEAR320_UARTX_PCLK_VAL_SYNTH1,
	},
};

/* uart1_2 parent select structure */
static struct pclk_sel uart1_2_pclk_sel = {
	.pclk_info = uartx_pclk_info,
	.pclk_count = ARRAY_SIZE(uartx_pclk_info),
	.pclk_sel_reg = VA_SPEAR320_CONTROL_REG,
	.pclk_sel_mask = UART1_2_PCLK_MASK,
};

/* uart1_2 clock */
static struct clk spear320_uart1_2_pclk = {
	.flags = ALWAYS_ENABLED,
	.pclk_sel = &uart1_2_pclk_sel,
	.pclk_sel_shift = UART1_2_PCLK_SHIFT,
	.recalc = &follow_parent,
};

/* uart1 clock */
static struct clk spear320_uart1_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &spear320_uart1_2_pclk,
	.recalc = &follow_parent,
};

/* uart2 clock */
static struct clk spear320_uart2_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &spear320_uart1_2_pclk,
	.recalc = &follow_parent,
};

/* sdhci parent clocks */
static struct pclk_info sdhci_pclk_info[] = {
	{
		.pclk = &pll3_48m_clk,
		.pclk_val = SDHCI_PCLK_VAL_48M,
	}, {
		.pclk = &ras_synth3_clk,
		.pclk_val = SDHCI_PCLK_VAL_SYNTH3,
	},
};

/* sdhci parent select structure */
static struct pclk_sel sdhci_pclk_sel = {
	.pclk_info = sdhci_pclk_info,
	.pclk_count = ARRAY_SIZE(sdhci_pclk_info),
	.pclk_sel_reg = VA_SPEAR320_CONTROL_REG,
	.pclk_sel_mask = SDHCI_PCLK_MASK,
};

/* sdhci clock */
static struct clk spear320_sdhci_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk_sel = &sdhci_pclk_sel,
	.pclk_sel_shift = SDHCI_PCLK_SHIFT,
	.recalc = &follow_parent,
};

/* SPEAr320S specific clock structures for Extended mode */
/* uartx parent select structure */
static struct pclk_sel uartx_pclk_sel = {
	.pclk_info = uartx_pclk_info,
	.pclk_count = ARRAY_SIZE(uartx_pclk_info),
	.pclk_sel_reg = VA_SPEAR320S_EXT_CTRL_REG,
	.pclk_sel_mask = SPEAR320S_UARTX_PCLK_MASK,
};

/* uartx clocks */
/*
 * Clock source for uart2 is selected using bits present in EXT_CTRL_REG in
 * extended mode. Only one uart2_clk 320 or 320s must be registered finally.
 */
static struct clk spear320s_uart2_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk_sel = &uartx_pclk_sel,
	.pclk_sel_shift = SPEAR320S_UART2_PCLK_SHIFT,
	.recalc = &follow_parent,
};

static struct clk spear320s_uart3_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk_sel = &uartx_pclk_sel,
	.pclk_sel_shift = SPEAR320S_UART3_PCLK_SHIFT,
	.recalc = &follow_parent,
};

static struct clk spear320s_uart4_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk_sel = &uartx_pclk_sel,
	.pclk_sel_shift = SPEAR320S_UART4_PCLK_SHIFT,
	.recalc = &follow_parent,
};

static struct clk spear320s_uart5_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk_sel = &uartx_pclk_sel,
	.pclk_sel_shift = SPEAR320S_UART5_PCLK_SHIFT,
	.recalc = &follow_parent,
};

static struct clk spear320s_uart6_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk_sel = &uartx_pclk_sel,
	.pclk_sel_shift = SPEAR320S_UART6_PCLK_SHIFT,
	.recalc = &follow_parent,
};

static struct clk spear320s_rs485_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk_sel = &uartx_pclk_sel,
	.pclk_sel_shift = SPEAR320S_RS485_PCLK_SHIFT,
	.recalc = &follow_parent,
};

/* i2s clock */
static struct clk spear320s_i2s_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &apb_clk,
	.recalc = &follow_parent,
};

static struct pclk_info i2s_ref_pclk_info[] = {
	{
		.pclk = &ras_synth2_clk,
		.pclk_val = I2S_REF_PCLK_SYNTH_VAL,
	}, {
		.pclk = &pll2_clk,
		.pclk_val = I2S_REF_PCLK_PLL2_VAL,
	},
};

static struct pclk_sel i2s_ref_pclk_sel = {
	.pclk_info = i2s_ref_pclk_info,
	.pclk_count = ARRAY_SIZE(i2s_ref_pclk_info),
	.pclk_sel_reg = VA_SPEAR320_CONTROL_REG,
	.pclk_sel_mask = I2S_REF_PCLK_MASK,
};

/* i2s ref clock */
static struct clk spear320s_i2s_ref_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk_sel = &i2s_ref_pclk_sel,
	.pclk_sel_shift = I2S_REF_PCLK_SHIFT,
	.recalc = &follow_parent,
};

/* i2s sclk clock */
static struct clk spear320s_i2s_sclk_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &spear320s_i2s_ref_clk,
	.div_factor = 4,
	.recalc = &follow_parent,
};
#endif

/* clk structures common to several machines */

#if defined(CONFIG_CPU_SPEAR310) || defined(CONFIG_CPU_SPEAR320)
/* emi clock */
static struct clk emi_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &ahb_clk,
	.recalc = &follow_parent,
};

/* fsmc clock */
static struct clk fsmc_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &ahb_clk,
	.recalc = &follow_parent,
};
#endif

/* common clocks to spear300 and spear320 */
#if defined(CONFIG_CPU_SPEAR300) || defined(CONFIG_CPU_SPEAR320)
/* clcd clock */
static struct clk clcd_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &pll3_48m_clk,
	.recalc = &follow_parent,
};
#endif /* CONFIG_CPU_SPEAR300 || CONFIG_CPU_SPEAR320 */

/* array of all spear 3xx clock lookups */
static struct clk_lookup spear_clk_lookups[] = {
	{ .con_id = "apb_pclk",		.clk = &dummy_apb_pclk},
	/* root clks */
	{ .con_id = "osc_32k_clk",	.clk = &osc_32k_clk},
	{ .con_id = "osc_24m_clk",	.clk = &osc_24m_clk},
	/* clock derived from 32 KHz osc clk */
	{ .dev_id = "rtc-spear",	.clk = &rtc_clk},
	/* clock derived from 24 MHz osc clk */
	{ .con_id = "vco1_clk",		.clk = &vco1_clk},
	{ .con_id = "vco2_clk",		.clk = &vco2_clk},
	{ .con_id = "pll3_48m_clk",	.clk = &pll3_48m_clk},
	{ .dev_id = "wdt",		.clk = &wdt_clk},
	/* clock derived from vco1 clk */
	{ .con_id = "pll1_clk",		.clk = &pll1_clk},
	/* clock derived from vco2 clk */
	{ .con_id = "pll2_clk",		.clk = &pll2_clk},
	/* clock derived from pll1 clk */
	{ .con_id = "cpu_clk",		.clk = &cpu_clk},
	{ .con_id = "ahb_clk",		.clk = &ahb_clk},
	{ .con_id = "uart_synth_clk",	.clk = &uart_synth_clk},
	{ .con_id = "firda_synth_clk",	.clk = &firda_synth_clk},
	{ .con_id = "gpt0_synth_clk",	.clk = &gpt0_synth_clk},
	{ .con_id = "gpt1_synth_clk",	.clk = &gpt1_synth_clk},
	{ .con_id = "gpt2_synth_clk",	.clk = &gpt2_synth_clk},
	{ .dev_id = "uart",		.clk = &uart_clk},
	{ .dev_id = "dice_ir",		.clk = &firda_clk},
	{ .dev_id = "gpt0",		.clk = &gpt0_clk},
	{ .dev_id = "gpt1",		.clk = &gpt1_clk},
	{ .dev_id = "gpt2",		.clk = &gpt2_clk},
	{ .con_id = "synth0_clk",	.clk = &synth0_clk},
	{ .con_id = "synth1_clk",	.clk = &synth1_clk},
	{ .con_id = "synth2_3_pclk",	.clk = &synth2_3_pclk},
	{ .con_id = "synth2_clk",	.clk = &synth2_clk},
	{ .con_id = "synth3_clk",	.clk = &synth3_clk},
	{ .con_id = "ras_synth0_clk",	.clk = &ras_synth0_clk},
	{ .con_id = "ras_synth1_clk",	.clk = &ras_synth1_clk},
	{ .con_id = "ras_synth2_clk",	.clk = &ras_synth2_clk},
	{ .con_id = "ras_synth3_clk",	.clk = &ras_synth3_clk},
	/* clock derived from pll3 clk */
	{ .dev_id = "designware_udc",   .clk = &usbd_clk},
	{ .con_id = "usbh_clk",		.clk = &usbh_clk},
	/* clock derived from ahb clk */
	{ .con_id = "ahbmult2_clk",	.clk = &ahbmult2_clk},
	{ .con_id = "ddr_clk",		.clk = &ddr_clk},
	{ .con_id = "apb_clk",		.clk = &apb_clk},
	{ .con_id = "amem_clk",		.clk = &amem_clk},
	{ .dev_id = "i2c_designware.0",	.clk = &i2c_clk},
	{ .dev_id = "pl080_dmac",	.clk = &dma_clk},
	{ .dev_id = "jpeg-designware",	.clk = &jpeg_clk},
	{ .dev_id = "stmmaceth",	.clk = &gmac_clk},
	{ .dev_id = "smi",		.clk = &smi_clk},
	{ .dev_id = "c3",		.clk = &c3_clk},
	/* clock derived from apb clk */
	{ .dev_id = "adc",		.clk = &adc_clk},
	{ .dev_id = "ssp-pl022.0",	.clk = &ssp0_clk},
	{ .dev_id = "gpio",		.clk = &gpio_clk},
};

/* array of all spear 300 clock lookups */
static struct clk_lookup spear300_clk_lookups[] = {
#ifdef CONFIG_CPU_SPEAR300
	{ .dev_id = "clcd",		.clk = &clcd_clk},
	{ .dev_id = "fsmc-nand.0",	.clk = &fsmc0_clk},
	{ .dev_id = "fsmc-nand.1",	.clk = &fsmc1_clk},
	{ .dev_id = "fsmc-nand.2",	.clk = &fsmc2_clk},
	{ .dev_id = "fsmc-nand.3",	.clk = &fsmc3_clk},
	{ .dev_id = "gpio1",		.clk = &gpio1_clk},
	{ .dev_id = "keyboard",		.clk = &kbd_clk},
	{ .dev_id = "sdhci",		.clk = &spear300_sdhci_clk},
#endif
};

/* array of all spear 310 clock lookups */
static struct clk_lookup spear310_clk_lookups[] = {
#ifdef CONFIG_CPU_SPEAR310
	{ .dev_id = "fsmc-nand",	.clk = &fsmc_clk},
	{ .con_id = "emi",		.clk = &emi_clk},
	{ .con_id = "tdm_hdlc",		.clk = &spear310_tdm_clk},
	{ .dev_id = "uart1",		.clk = &spear310_uart1_clk},
	{ .dev_id = "uart2",		.clk = &spear310_uart2_clk},
	{ .dev_id = "uart3",		.clk = &spear310_uart3_clk},
	{ .dev_id = "uart4",		.clk = &spear310_uart4_clk},
	{ .dev_id = "uart5",		.clk = &spear310_uart5_clk},
#endif
};

/* array of all spear 320 clock lookups */
static struct clk_lookup spear320_clk_lookups[] = {
#ifdef CONFIG_CPU_SPEAR320
	{ .dev_id = "clcd",		.clk = &clcd_clk},
	{ .dev_id = "fsmc-nand",	.clk = &fsmc_clk},
	{ .dev_id = "i2c_designware.1",	.clk = &i2c1_clk},
	{ .con_id = "emi",		.clk = &emi_clk},
	{ .con_id = "smii_125m_clk",	.clk = &smii_125m_pad},
	{ .con_id = "smii_clk",		.clk = &spear320_smii_clk},
	{ .dev_id = "pwm",		.clk = &pwm_clk},
	{ .dev_id = "sdhci",		.clk = &spear320_sdhci_clk},
	{ .dev_id = "c_can_platform.0",	.clk = &can0_clk},
	{ .dev_id = "c_can_platform.1",	.clk = &can1_clk},
	{ .dev_id = "ssp-pl022.1",	.clk = &ssp1_clk},
	{ .dev_id = "ssp-pl022.2",	.clk = &ssp2_clk},
	{ .con_id = "uart1_2_pclk",	.clk = &spear320_uart1_2_pclk},
	{ .dev_id = "uart1",		.clk = &spear320_uart1_clk},
	{ .dev_id = "uart2",		.clk = &spear320_uart2_clk},

	/* Extended mode clocks */
	{ .dev_id = "designware-i2s",	.clk = &spear320s_i2s_clk},
	{ .con_id = "i2s_ref_clk",	.clk = &spear320s_i2s_ref_clk},
	{ .con_id = "i2s_sclk_clk",	.clk = &spear320s_i2s_sclk_clk},
	{ .dev_id = "uart2",		.clk = &spear320s_uart2_clk},
	{ .dev_id = "uart3",		.clk = &spear320s_uart3_clk},
	{ .dev_id = "uart4",		.clk = &spear320s_uart4_clk},
	{ .dev_id = "uart5",		.clk = &spear320s_uart5_clk},
	{ .dev_id = "uart6",		.clk = &spear320s_uart6_clk},
	{ .dev_id = "uart7",		.clk = &spear320s_rs485_clk},
#endif
};

/* machine clk init */
void __init spear3xx_clk_init(void)
{
	int i, cnt;
	struct clk_lookup *lookups;

	if (cpu_is_spear300()) {
		cnt = ARRAY_SIZE(spear300_clk_lookups);
		lookups = spear300_clk_lookups;
	} else if (cpu_is_spear310()) {
		cnt = ARRAY_SIZE(spear310_clk_lookups);
		lookups = spear310_clk_lookups;
	} else {
		cnt = ARRAY_SIZE(spear320_clk_lookups);
		lookups = spear320_clk_lookups;
	}

	for (i = 0; i < ARRAY_SIZE(spear_clk_lookups); i++)
		clk_register(&spear_clk_lookups[i]);

	for (i = 0; i < cnt; i++)
		clk_register(&lookups[i]);

	clk_init(&ddr_clk);
}
