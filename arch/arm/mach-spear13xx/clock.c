/*
 * arch/arm/mach-spear13xx/clock.c
 *
 * SPEAr13xx machines clock framework source file
 *
 * Copyright (C) 2010 ST Microelectronics
 * shiraz hashim<shiraz.hashim@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <asm/mach-types.h>
#include <plat/clock.h>
#include <mach/hardware.h>
#include <mach/misc_regs.h>

/* root clks */
/* 24 MHz oscillator clock */
static struct clk osc1_24m_clk = {
	.flags = ALWAYS_ENABLED,
	.rate = 24000000,
};

/* 32 KHz oscillator clock */
static struct clk osc2_32k_clk = {
	.flags = ALWAYS_ENABLED,
	.rate = 32000,
};

/* 25 MHz MIPHY oscillator clock */
static struct clk osc3_25m_clk = {
	.flags = ALWAYS_ENABLED,
	.rate = 25000000,
};

/* clock derived from 32 KHz osc clk */
/* rtc clock */
static struct clk rtc_clk = {
	.pclk = &osc2_32k_clk,
	.en_reg = PERIP1_CLK_ENB,
	.en_reg_bit = RTC_CLK_ENB,
	.recalc = &follow_parent,
};

/* clock derived from osc1 or osc3 */
/* pll[1-3] parents */
static struct pclk_info pll_pclk_info[] = {
	{
		.pclk = &osc1_24m_clk,
		.pclk_val = OSC_24M_VAL,
	}, {
		.pclk = &osc3_25m_clk,
		.pclk_val = OSC_25M_VAL,
	},
};

/* pll[1-3] parent select structure */
static struct pclk_sel pll_pclk_sel = {
	.pclk_info = pll_pclk_info,
	.pclk_count = ARRAY_SIZE(pll_pclk_info),
	.pclk_sel_reg = PLL_CFG,
	.pclk_sel_mask = PLL_CLK_MASK,
};

/* pll masks structure */
static struct pll_clk_masks pll_masks = {
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
};
/* pll1 configuration structure */
static struct pll_clk_config pll1_config = {
	.mode_reg = PLL1_CTR,
	.cfg_reg = PLL1_FRQ,
	.masks = &pll_masks,
};

/* pll rate configuration table, in ascending order of rates */
struct pll_rate_tbl pll_rtbl[] = {
	/* PCLK 24MHz */
	{.mode = 0, .m = 0x7D, .n = 0x03, .p = 0x2}, /* 500 MHz */
	{.mode = 0, .m = 0xA6, .n = 0x03, .p = 0x2}, /* 664 MHz */
	{.mode = 0, .m = 0xC8, .n = 0x03, .p = 0x2}, /* 800 MHz */
	{.mode = 0, .m = 0xFA, .n = 0x06, .p = 0x1}, /* 1000 MHz */
};

/* pll1 clock */
static struct clk pll1_clk = {
	.flags = ENABLED_ON_INIT,
	.pclk_sel = &pll_pclk_sel,
	.pclk_sel_shift = PLL1_CLK_SHIFT,
	.en_reg = PLL1_CTR,
	.en_reg_bit = PLL_ENABLE,
	.calc_rate = &pll_calc_rate,
	.recalc = &pll_clk_recalc,
	.set_rate = &pll_clk_set_rate,
	.rate_config = {pll_rtbl, ARRAY_SIZE(pll_rtbl), 3},
	.private_data = &pll1_config,
};

/* pll1div2 clock */
static struct clk pll1div2_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &pll1_clk,
	.div_factor = 2,
	.recalc = &follow_parent,
};

/* pll1div4 clock */
static struct clk pll1div4_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &pll1_clk,
	.div_factor = 4,
	.recalc = &follow_parent,
};

/* pll2 configuration structure */
static struct pll_clk_config pll2_config = {
	.mode_reg = PLL2_CTR,
	.cfg_reg = PLL2_FRQ,
	.masks = &pll_masks,
};

/* pll2 clock */
static struct clk pll2_clk = {
	.pclk_sel = &pll_pclk_sel,
	.pclk_sel_shift = PLL2_CLK_SHIFT,
	.en_reg = PLL2_CTR,
	.en_reg_bit = PLL_ENABLE,
	.calc_rate = &pll_calc_rate,
	.recalc = &pll_clk_recalc,
	.set_rate = &pll_clk_set_rate,
	.rate_config = {pll_rtbl, ARRAY_SIZE(pll_rtbl), 3},
	.private_data = &pll2_config,
};

/* pll3 configuration structure */
static struct pll_clk_config pll3_config = {
	.mode_reg = PLL3_CTR,
	.cfg_reg = PLL3_FRQ,
	.masks = &pll_masks,
};

/* pll3 clock */
static struct clk pll3_clk = {
	.pclk_sel = &pll_pclk_sel,
	.pclk_sel_shift = PLL3_CLK_SHIFT,
	.en_reg = PLL3_CTR,
	.en_reg_bit = PLL_ENABLE,
	.calc_rate = &pll_calc_rate,
	.recalc = &pll_clk_recalc,
	.set_rate = &pll_clk_set_rate,
	.rate_config = {pll_rtbl, ARRAY_SIZE(pll_rtbl), 3},
	.private_data = &pll3_config,
};

/* pll4 (DDR) configuration structure */
static struct pll_clk_config pll4_config = {
	.mode_reg = PLL4_CTR,
	.cfg_reg = PLL4_FRQ,
	.masks = &pll_masks,
};

/* pll4 rate configuration table, in ascending order of rates */
struct pll_rate_tbl pll4_rtbl[] = {
	{.mode = 0, .m = 0x7D, .n = 0x03, .p = 0x2}, /* 500 MHz */
	{.mode = 0, .m = 0xA6, .n = 0x03, .p = 0x2}, /* 664 MHz */
	{.mode = 0, .m = 0xC8, .n = 0x03, .p = 0x2}, /* 800 MHz */
	{.mode = 0, .m = 0xFA, .n = 0x06, .p = 0x1}, /* 1000 MHz */
};

/* pll4 (DDR) clock */
static struct clk pll4_clk = {
	.flags = ENABLED_ON_INIT,
	.pclk = &osc1_24m_clk,
	.en_reg = PLL4_CTR,
	.en_reg_bit = PLL_ENABLE,
	.calc_rate = &pll_calc_rate,
	.recalc = &pll_clk_recalc,
	.set_rate = &pll_clk_set_rate,
	.rate_config = {pll4_rtbl, ARRAY_SIZE(pll4_rtbl), 3},
	.private_data = &pll4_config,
};

/* pll5 USB 48 MHz clock */
static struct clk pll5_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &osc1_24m_clk,
	.rate = 48000000,
};

/* pll6 (MIPHY) clock */
static struct clk pll6_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &osc3_25m_clk,
	.rate = 25000000,
};

/* clocks derived from pll1 clk */
/* cpu clock */
static struct clk cpu_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &pll1_clk,
	.div_factor = 2,
	.recalc = &follow_parent,
};

/* ahb clock */
static struct clk ahb_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &pll1_clk,
	.div_factor = 6,
	.recalc = &follow_parent,
};

/* apb clock */
static struct clk apb_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &pll1_clk,
	.div_factor = 12,
	.recalc = &follow_parent,
};

/* clocks derived from osc1, ahb or apb */
/* gpt[0-3] parents */
static struct pclk_info gpt_pclk_info[] = {
	{
		.pclk = &osc1_24m_clk,
		.pclk_val = GPT_OSC24_VAL,
	}, {
		.pclk = &apb_clk,
		.pclk_val = GPT_APB_VAL,
	},
};

/* gpt[0-3] parent select structure */
static struct pclk_sel gpt_pclk_sel = {
	.pclk_info = gpt_pclk_info,
	.pclk_count = ARRAY_SIZE(gpt_pclk_info),
	.pclk_sel_reg = PERIP_CLK_CFG,
	.pclk_sel_mask = GPT_CLK_MASK,
};

/* gpt0 timer clock */
static struct clk gpt0_clk = {
	.en_reg = PERIP1_CLK_ENB,
	.en_reg_bit = GPT0_CLK_ENB,
	.pclk_sel = &gpt_pclk_sel,
	.pclk_sel_shift = GPT0_CLK_SHIFT,
	.recalc = &follow_parent,
};

/* gpt1 timer clock */
static struct clk gpt1_clk = {
	.en_reg = PERIP1_CLK_ENB,
	.en_reg_bit = GPT1_CLK_ENB,
	.pclk_sel = &gpt_pclk_sel,
	.pclk_sel_shift = GPT1_CLK_SHIFT,
	.recalc = &follow_parent,
};

/* gpt2 timer clock */
static struct clk gpt2_clk = {
	.en_reg = PERIP2_CLK_ENB,
	.en_reg_bit = GPT2_CLK_ENB,
	.pclk_sel = &gpt_pclk_sel,
	.pclk_sel_shift = GPT2_CLK_SHIFT,
	.recalc = &follow_parent,
};

/* gpt3 timer clock */
static struct clk gpt3_clk = {
	.en_reg = PERIP2_CLK_ENB,
	.en_reg_bit = GPT3_CLK_ENB,
	.pclk_sel = &gpt_pclk_sel,
	.pclk_sel_shift = GPT3_CLK_SHIFT,
	.recalc = &follow_parent,
};

/* watch dog timer clock */
static struct clk wdt_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk = &cpu_clk,
	.div_factor = 2,
	.recalc = &follow_parent,
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

/* clocks derived multiple parents (pll1, pll5, synthesizers or others) */
/* uart configurations */
static struct aux_clk_config uart_synth_config = {
	.synth_reg = UART_CLK_SYNT,
	.masks = &aux_masks,
};

/* aux rate configuration table, in ascending order of rates */
struct aux_rate_tbl aux_rtbl[] = {
	/* For PLL1div2 = 500 MHz */
	{.xscale = 1, .yscale = 6, .eq = 1}, /* 83 MHz */
	{.xscale = 1, .yscale = 4, .eq = 1}, /* 125 MHz */
	{.xscale = 1, .yscale = 3, .eq = 1}, /* 166 MHz */
	{.xscale = 1, .yscale = 2, .eq = 1}, /* 250 MHz */
};

/* uart synth clock */
static struct clk uart_synth_clk = {
	.en_reg = UART_CLK_SYNT,
	.en_reg_bit = AUX_SYNT_ENB,
	.pclk = &pll1div2_clk,
	.calc_rate = &aux_calc_rate,
	.recalc = &aux_clk_recalc,
	.set_rate = &aux_clk_set_rate,
	.rate_config = {aux_rtbl, ARRAY_SIZE(aux_rtbl), 0},
	.private_data = &uart_synth_config,
};

/* uart parents */
static struct pclk_info uart_pclk_info[] = {
	{
		.pclk = &pll5_clk,
		.pclk_val = AUX_CLK_PLL5_VAL,
	}, {
		.pclk = &uart_synth_clk,
		.pclk_val = AUX_CLK_SYNT_VAL,
	},
};

/* uart parent select structure */
static struct pclk_sel uart_pclk_sel = {
	.pclk_info = uart_pclk_info,
	.pclk_count = ARRAY_SIZE(uart_pclk_info),
	.pclk_sel_reg = PERIP_CLK_CFG,
	.pclk_sel_mask = UART_CLK_MASK,
};

/* uart clock */
static struct clk uart_clk = {
	.en_reg = PERIP1_CLK_ENB,
	.en_reg_bit = UART_CLK_ENB,
	.pclk_sel = &uart_pclk_sel,
	.pclk_sel_shift = UART_CLK_SHIFT,
	.recalc = &follow_parent,
};

/* sdhci configurations */
static struct aux_clk_config sdhci_synth_config = {
	.synth_reg = SDHCI_CLK_SYNT,
	.masks = &aux_masks,
};

/* sdhci synth clock */
static struct clk sdhci_synth_clk = {
	.en_reg = SDHCI_CLK_SYNT,
	.en_reg_bit = AUX_SYNT_ENB,
	.pclk = &pll1div2_clk,
	.calc_rate = &aux_calc_rate,
	.recalc = &aux_clk_recalc,
	.set_rate = &aux_clk_set_rate,
	.rate_config = {aux_rtbl, ARRAY_SIZE(aux_rtbl), 2},
	.private_data = &sdhci_synth_config,
};

/* sdhci clock */
static struct clk sdhci_clk = {
	.en_reg = PERIP1_CLK_ENB,
	.en_reg_bit = SDHCI_CLK_ENB,
	.pclk = &sdhci_synth_clk,
	.recalc = &follow_parent,
};

/* cfxd configurations */
static struct aux_clk_config cfxd_synth_config = {
	.synth_reg = CFXD_CLK_SYNT,
	.masks = &aux_masks,
};

/* cfxd synth clock */
static struct clk cfxd_synth_clk = {
	.en_reg = CFXD_CLK_SYNT,
	.en_reg_bit = AUX_SYNT_ENB,
	.pclk = &pll1div2_clk,
	.calc_rate = &aux_calc_rate,
	.recalc = &aux_clk_recalc,
	.set_rate = &aux_clk_set_rate,
	.rate_config = {aux_rtbl, ARRAY_SIZE(aux_rtbl), 2},
	.private_data = &cfxd_synth_config,
};

/* cfxd clock */
static struct clk cfxd_clk = {
	.en_reg = PERIP1_CLK_ENB,
	.en_reg_bit = CFXD_CLK_ENB,
	.pclk = &cfxd_synth_clk,
	.recalc = &follow_parent,
};

/* C3 clk configurations */
static struct aux_clk_config c3_synth_config = {
	.synth_reg = C3_CLK_SYNT,
	.masks = &aux_masks,
};

/* c3 synth clock */
static struct clk c3_synth_clk = {
	.en_reg = C3_CLK_SYNT,
	.en_reg_bit = AUX_SYNT_ENB,
	.pclk = &pll1div2_clk,
	.calc_rate = &aux_calc_rate,
	.recalc = &aux_clk_recalc,
	.set_rate = &aux_clk_set_rate,
	.rate_config = {aux_rtbl, ARRAY_SIZE(aux_rtbl), 0},
	.private_data = &c3_synth_config,
};

/* c3 parents */
static struct pclk_info c3_pclk_info[] = {
	{
		.pclk = &pll5_clk,
		.pclk_val = AUX_CLK_PLL5_VAL,
	}, {
		.pclk = &c3_synth_clk,
		.pclk_val = AUX_CLK_SYNT_VAL,
	},
};

/* c3 parent select structure */
static struct pclk_sel c3_pclk_sel = {
	.pclk_info = c3_pclk_info,
	.pclk_count = ARRAY_SIZE(c3_pclk_info),
	.pclk_sel_reg = PERIP_CLK_CFG,
	.pclk_sel_mask = C3_CLK_MASK,
};

/* c3 clock */
static struct clk c3_clk = {
	.en_reg = PERIP1_CLK_ENB,
	.en_reg_bit = C3_CLK_ENB,
	.pclk_sel = &c3_pclk_sel,
	.pclk_sel_shift = C3_CLK_SHIFT,
	.recalc = &follow_parent,
};

/* gmac phy clk configurations */
static struct aux_clk_config gmac_phy_synth_config = {
	.synth_reg = GMAC_CLK_SYNT,
	.masks = &aux_masks,
};

/* gmii external pad clock for phy operation */
static struct clk gmii_txclk125_pad = {
	.flags = ALWAYS_ENABLED,
	.rate = 125000000,
};

/* gmac phy set of input clks*/
static struct pclk_info gmac_phy_input_pclk_info[] = {
	{
		.pclk = &gmii_txclk125_pad,
		.pclk_val = GMAC_PHY_PAD_VAL,
	}, {
		.pclk = &pll2_clk,
		.pclk_val = GMAC_PHY_PLL2_VAL,
	}, {
		.pclk = &osc3_25m_clk,
		.pclk_val = GMAC_PHY_OSC3_VAL,
	},
};

static struct pclk_sel gmac_phy_input_pclk_sel = {
	.pclk_info = gmac_phy_input_pclk_info,
	.pclk_count = ARRAY_SIZE(gmac_phy_input_pclk_info),
	.pclk_sel_reg = GMAC_CLK_CFG,
	.pclk_sel_mask = GMAC_PHY_INPUT_CLK_MASK,
};

static struct clk gmac_phy_input_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk_sel = &gmac_phy_input_pclk_sel,
	.pclk_sel_shift = GMAC_PHY_INPUT_CLK_SHIFT,
	.recalc = &follow_parent,
};

/* gmac rate configuration table, in ascending order of rates */
struct aux_rate_tbl gmac_rtbl[] = {
	/* For gmac phy input clk */
	{.xscale = 1, .yscale = 6, .eq = 1}, /* divided by 6 */
	{.xscale = 1, .yscale = 4, .eq = 1}, /* divided by 4 */
	{.xscale = 1, .yscale = 3, .eq = 1}, /* divided by 3 */
	{.xscale = 1, .yscale = 2, .eq = 1}, /* divided by 2 */
};

static struct clk gmac_phy_synth_clk = {
	.en_reg = GMAC_CLK_CFG,
	.en_reg_bit = GMAC_PHY_SYNT_ENB,
	.pclk = &gmac_phy_input_clk,
	.calc_rate = &aux_calc_rate,
	.recalc = &aux_clk_recalc,
	.set_rate = &aux_clk_set_rate,
	.rate_config = {gmac_rtbl, ARRAY_SIZE(gmac_rtbl), 0},
	.private_data = &gmac_phy_synth_config,
};

/* gmac phy parents */
static struct pclk_info gmac_phy_pclk_info[] = {
	{
		.pclk = &gmac_phy_input_clk,
		.pclk_val = 0,
	}, {
		.pclk = &gmac_phy_synth_clk,
		.pclk_val = 1,
	}
};

/* gmac phy parent select structure */
static struct pclk_sel gmac_phy_pclk_sel = {
	.pclk_info = gmac_phy_pclk_info,
	.pclk_count = ARRAY_SIZE(gmac_phy_pclk_info),
	.pclk_sel_reg = GMAC_CLK_CFG,
	.pclk_sel_mask = GMAC_PHY_CLK_MASK,
};

/* gmac phy clock */
static struct clk gmac_phy_clk = {
	.flags = ALWAYS_ENABLED,
	.pclk_sel = &gmac_phy_pclk_sel,
	.pclk_sel_shift = GMAC_PHY_CLK_SHIFT,
	.recalc = &follow_parent,
};

/* clcd synthesizers masks */
static struct clcd_synth_masks clcd_masks = {
	.div_factor_mask = CLCD_SYNT_DIV_FACTOR_MASK,
	.div_factor_shift = CLCD_SYNT_DIV_FACTOR_SHIFT,
};

static struct clcd_clk_config clcd_synth_config = {
	.synth_reg = CLCD_CLK_SYNT,
	.masks = &clcd_masks,
};

/* clcd synth parents */
static struct pclk_info clcd_synth_pclk_info[] = {
	{
		.pclk = &pll1div4_clk,
		.pclk_val = CLCD_SYNT_PLL1_DIV4_VAL,
	}, {
		.pclk = &pll2_clk,
		.pclk_val = CLCD_SYNT_PLL2_VAL,
	},
};

/* clcd synth parent select structure */
static struct pclk_sel clcd_synth_pclk_sel = {
	.pclk_info = clcd_synth_pclk_info,
	.pclk_count = ARRAY_SIZE(clcd_synth_pclk_info),
	.pclk_sel_reg = PLL_CFG,
	.pclk_sel_mask = CLCD_SYNT_CLK_MASK,
};

/* clcd rate configuration table, in ascending order of rates */
struct clcd_rate_tbl clcd_rtbl[] = {
	/* For pll1div4 = 250 MHz */
	{.div = 0x4000}, /* 62.5 MHz */
	{.div = 0x2000}, /* 125 MHz */
};

/* clcd synth clock */
static struct clk clcd_synth_clk = {
	.en_reg = CLCD_CLK_SYNT,
	.en_reg_bit = CLCD_SYNT_ENB,
	.pclk_sel = &clcd_synth_pclk_sel,
	.pclk_sel_shift = CLCD_SYNT_CLK_SHIFT,
	.calc_rate = &clcd_calc_rate,
	.recalc = &clcd_clk_recalc,
	.set_rate = &clcd_clk_set_rate,
	.rate_config = {clcd_rtbl, ARRAY_SIZE(clcd_rtbl), 1},
	.private_data = &clcd_synth_config,
};

/* clcd clock parents */
static struct pclk_info clcd_pclk_info[] = {
	{
		.pclk = &pll5_clk,
		.pclk_val = AUX_CLK_PLL5_VAL,
	}, {
		.pclk = &clcd_synth_clk,
		.pclk_val = AUX_CLK_SYNT_VAL,
	},
};

/* clcd parent select structure */
static struct pclk_sel clcd_pclk_sel = {
	.pclk_info = clcd_pclk_info,
	.pclk_count = ARRAY_SIZE(clcd_pclk_info),
	.pclk_sel_reg = PERIP_CLK_CFG,
	.pclk_sel_mask = CLCD_CLK_MASK,
};

/* clcd clock */
static struct clk clcd_clk = {
	.en_reg = PERIP1_CLK_ENB,
	.en_reg_bit = CLCD_CLK_ENB,
	.pclk_sel = &clcd_pclk_sel,
	.pclk_sel_shift = CLCD_CLK_SHIFT,
	.recalc = &follow_parent,
};

/* clock derived from ahb clk */

/* i2c clock */
static struct clk i2c_clk = {
	.pclk = &ahb_clk,
	.en_reg = PERIP1_CLK_ENB,
	.en_reg_bit = I2C_CLK_ENB,
	.recalc = &follow_parent,
};

/* dma clock */
static struct clk dma0_clk = {
	.pclk = &ahb_clk,
	.en_reg = PERIP1_CLK_ENB,
	.en_reg_bit = DMA0_CLK_ENB,
	.recalc = &follow_parent,
};

static struct clk dma1_clk = {
	.pclk = &ahb_clk,
	.en_reg = PERIP1_CLK_ENB,
	.en_reg_bit = DMA1_CLK_ENB,
	.recalc = &follow_parent,
};

/* jpeg clock */
static struct clk jpeg_clk = {
	.pclk = &ahb_clk,
	.en_reg = PERIP1_CLK_ENB,
	.en_reg_bit = JPEG_CLK_ENB,
	.recalc = &follow_parent,
};

/* gmac clock */
static struct clk gmac_clk = {
	.pclk = &ahb_clk,
	.en_reg = PERIP1_CLK_ENB,
	.en_reg_bit = GMAC_CLK_ENB,
	.recalc = &follow_parent,
};

/* fsmc clock */
static struct clk fsmc_clk = {
	.pclk = &ahb_clk,
	.en_reg = PERIP1_CLK_ENB,
	.en_reg_bit = FSMC_CLK_ENB,
	.recalc = &follow_parent,
};

/* smi clock */
static struct clk smi_clk = {
	.pclk = &ahb_clk,
	.en_reg = PERIP1_CLK_ENB,
	.en_reg_bit = SMI_CLK_ENB,
	.recalc = &follow_parent,
};

/* uhc0 clock */
static struct clk uhci0_clk = {
	.pclk = &ahb_clk,
	.en_reg = PERIP1_CLK_ENB,
	.en_reg_bit = UHC0_CLK_ENB,
	.recalc = &follow_parent,
};

/* uhc1 clock */
static struct clk uhci1_clk = {
	.pclk = &ahb_clk,
	.en_reg = PERIP1_CLK_ENB,
	.en_reg_bit = UHC1_CLK_ENB,
	.recalc = &follow_parent,
};

/* usbd clock */
static struct clk usbd_clk = {
	.pclk = &ahb_clk,
	.en_reg = PERIP1_CLK_ENB,
	.en_reg_bit = USBD_CLK_ENB,
	.recalc = &follow_parent,
};

/* pci clocks */
static struct clk pcie0_clk = {
	.pclk = &ahb_clk,
	.en_reg = PERIP1_CLK_ENB,
	.en_reg_bit = PCIE0_CLK_ENB,
	.recalc = &follow_parent,
};

static struct clk pcie1_clk = {
	.pclk = &ahb_clk,
	.en_reg = PERIP1_CLK_ENB,
	.en_reg_bit = PCIE1_CLK_ENB,
	.recalc = &follow_parent,
};

static struct clk pcie2_clk = {
	.pclk = &ahb_clk,
	.en_reg = PERIP1_CLK_ENB,
	.en_reg_bit = PCIE2_CLK_ENB,
	.recalc = &follow_parent,
};

/* sysram clocks */
static struct clk sysram0_clk = {
	.pclk = &ahb_clk,
	.en_reg = PERIP1_CLK_ENB,
	.en_reg_bit = SYSRAM0_CLK_ENB,
	.recalc = &follow_parent,
};

static struct clk sysram1_clk = {
	.pclk = &ahb_clk,
	.en_reg = PERIP1_CLK_ENB,
	.en_reg_bit = SYSRAM1_CLK_ENB,
	.recalc = &follow_parent,
};

/* clock derived from apb clk */
/* adc clock */
static struct clk adc_clk = {
	.pclk = &apb_clk,
	.en_reg = PERIP1_CLK_ENB,
	.en_reg_bit = ADC_CLK_ENB,
	.recalc = &follow_parent,
};

/* ssp clock */
static struct clk ssp_clk = {
	.pclk = &apb_clk,
	.en_reg = PERIP1_CLK_ENB,
	.en_reg_bit = SSP_CLK_ENB,
	.recalc = &follow_parent,
};

/* gpio clock */
static struct clk gpio0_clk = {
	.pclk = &apb_clk,
	.en_reg = PERIP1_CLK_ENB,
	.en_reg_bit = GPIO0_CLK_ENB,
	.recalc = &follow_parent,
};

/* gpio clock */
static struct clk gpio1_clk = {
	.pclk = &apb_clk,
	.en_reg = PERIP1_CLK_ENB,
	.en_reg_bit = GPIO1_CLK_ENB,
	.recalc = &follow_parent,
};

/* i2s0 clock */
static struct clk i2s0_clk = {
	.pclk = &apb_clk,
	.en_reg = PERIP1_CLK_ENB,
	.en_reg_bit = I2S0_CLK_ENB,
	.recalc = &follow_parent,
};

/* i2s1 clock */
static struct clk i2s1_clk = {
	.pclk = &apb_clk,
	.en_reg = PERIP1_CLK_ENB,
	.en_reg_bit = I2S1_CLK_ENB,
	.recalc = &follow_parent,
};

/* keyboard clock */
static struct clk kbd_clk = {
	.pclk = &apb_clk,
	.en_reg = PERIP2_CLK_ENB,
	.en_reg_bit = KBD_CLK_ENB,
	.recalc = &follow_parent,
};

/* spear1300 machine specific clock structures */
#ifdef CONFIG_MACH_SPEAR1300

#endif

/* spear1310 machine specific clock structures */
#ifdef CONFIG_MACH_SPEAR1310
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
#endif

/* array of all spear 13xx clock lookups */
static struct clk_lookup spear_clk_lookups[] = {
	/* root clks */
	{.con_id = "osc1_24m_clk",	.clk = &osc1_24m_clk},
	{.con_id = "osc2_32k_clk",	.clk = &osc2_32k_clk},
	{.con_id = "osc3_25m_clk",	.clk = &osc3_25m_clk},

	/* clock derived from 32 KHz osc clk */
	{.dev_id = "rtc-spear",		.clk = &rtc_clk},

	/* clock derived from 24/25 MHz osc1/osc3 clk */
	{.con_id = "pll1_clk",		.clk = &pll1_clk},
	{.con_id = "pll2_clk",		.clk = &pll2_clk},
	{.con_id = "pll3_clk",		.clk = &pll3_clk},
	{.con_id = "pll4_clk",		.clk = &pll4_clk},
	{.con_id = "pll5_clk",		.clk = &pll5_clk},
	{.con_id = "pll6_clk",		.clk = &pll6_clk},

	/* clock derived from pll1 clk */
	{.con_id = "cpu_clk",		.clk = &cpu_clk},
	{.con_id = "ahb_clk",		.clk = &ahb_clk},
	{.con_id = "apb_clk",		.clk = &apb_clk},

	/* synthesizers/prescaled clocks */
	{.con_id = "pll1div2_clk",		.clk = &pll1div2_clk},
	{.con_id = "pll1div4_clk",		.clk = &pll1div4_clk},
	{.con_id = "c3_synth_clk",		.clk = &c3_synth_clk},
	{.con_id = "gmii_txclk123_pad_clk",	.clk = &gmii_txclk125_pad},
	{.con_id = "clcd_synth_clk",		.clk = &clcd_synth_clk},
	{.con_id = "uart_synth_clk",		.clk = &uart_synth_clk},
	{.con_id = "sdhci_synth_clk",		.clk = &sdhci_synth_clk},
	{.con_id = "cfxd_synth_clk",		.clk = &cfxd_synth_clk},
	{.con_id = "gmac_phy_input_clk",	.clk = &gmac_phy_input_clk},
	{.con_id = "gmac_phy_synth_clk",	.clk = &gmac_phy_synth_clk},
	{.con_id = "gmac_phy_clk",		.clk = &gmac_phy_clk},

	/* clocks having multiple parent source from above clocks */
	{.dev_id = "clcd",		.clk = &clcd_clk},
	{.dev_id = "gpt0",		.clk = &gpt0_clk},
	{.dev_id = "gpt1",		.clk = &gpt1_clk},
	{.dev_id = "gpt2",		.clk = &gpt2_clk},
	{.dev_id = "gpt3",		.clk = &gpt3_clk},
	{.dev_id = "uart",		.clk = &uart_clk},

	/* clock derived from ahb clk */
	{.dev_id = "smi",		.clk = &smi_clk},
	{.con_id = "usbh.0_clk",	.clk = &uhci0_clk},
	{.con_id = "usbh.1_clk",	.clk = &uhci1_clk},
	{.dev_id = "usbd",		.clk = &usbd_clk},
	{.dev_id = "i2c_designware.0",	.clk = &i2c_clk},
	{.dev_id = "dma0",		.clk = &dma0_clk},
	{.dev_id = "dma1",		.clk = &dma1_clk},
	{.dev_id = "jpeg",		.clk = &jpeg_clk},
	{.dev_id = "gmac",		.clk = &gmac_clk},
	{.dev_id = "c3",		.clk = &c3_clk},
	{.dev_id = "pcie0",		.clk = &pcie0_clk},
	{.dev_id = "pcie1",		.clk = &pcie1_clk},
	{.dev_id = "pcie2",		.clk = &pcie2_clk},
	{.dev_id = "cfxd",		.clk = &cfxd_clk},
	{.dev_id = "sdhci",		.clk = &sdhci_clk},
	{.con_id = "fsmc",		.clk = &fsmc_clk},
	{.dev_id = "sysram0",		.clk = &sysram0_clk},
	{.dev_id = "sysram1",		.clk = &sysram1_clk},

	/* clock derived from apb clk */
	{.dev_id = "i2s0",		.clk = &i2s0_clk},
	{.dev_id = "i2s1",		.clk = &i2s1_clk},
	{.dev_id = "adc",		.clk = &adc_clk},
	{.dev_id = "ssp-pl022",		.clk = &ssp_clk},
	{.dev_id = "gpio0",		.clk = &gpio0_clk},
	{.dev_id = "gpio1",		.clk = &gpio1_clk},
	{.dev_id = "keyboard",		.clk = &kbd_clk},
	{.dev_id = "wdt",		.clk = &wdt_clk},

	/* spear1300 machine specific clock structures */
#ifdef CONFIG_MACH_SPEAR1300
#endif

	/* spear1310 machine specific clock structures */
#ifdef CONFIG_MACH_SPEAR1310
	{.dev_id = "c_can_platform.0",	.clk = &can0_clk},
	{.dev_id = "c_can_platform.1",	.clk = &can1_clk},
#endif
};

void __init clk_init(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(spear_clk_lookups); i++)
		clk_register(&spear_clk_lookups[i]);

	recalc_root_clocks();
}
