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
		.pclk_mask = OSC_24M_MASK,
	}, {
		.pclk = &osc3_25m_clk,
		.pclk_mask = OSC_25M_MASK,
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

/* pll1 clock */
static struct clk pll1_clk = {
	.pclk_sel = &pll_pclk_sel,
	.pclk_sel_shift = PLL1_CLK_SHIFT,
	.en_reg = PLL1_CTR,
	.en_reg_bit = PLL_ENABLE,
	.recalc = &pll_clk_recalc,
	.private_data = &pll1_config,
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
	.recalc = &pll_clk_recalc,
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
	.recalc = &pll_clk_recalc,
	.private_data = &pll3_config,
};

/* pll4 (DDR) configuration structure */
static struct pll_clk_config pll4_config = {
	.mode_reg = PLL4_CTR,
	.cfg_reg = PLL4_FRQ,
	.masks = &pll_masks,
};

/* pll4 (DDR) clock */
static struct clk pll4_clk = {
	.pclk = &osc1_24m_clk,
	.en_reg = PLL4_CTR,
	.en_reg_bit = PLL_ENABLE,
	.recalc = &pll_clk_recalc,
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
		.pclk_mask = GPT_OSC24_MASK,
	}, {
		.pclk = &apb_clk,
		.pclk_mask = GPT_APB_MASK,
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
	.pclk = &apb_clk,
	.recalc = &follow_parent,
};

/* smi clock */
static struct clk smi_clk = {
	.pclk = &ahb_clk,
	.en_reg = PERIP1_CLK_ENB,
	.en_reg_bit = SMI_CLK_ENB,
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

/* uart configurations */
static struct aux_clk_config uart_config = {
	.synth_reg = UART_CLK_SYNT,
	.masks = &aux_masks,
};

/* clocks derived from pll1 or pll5 */
/* uart parents */
static struct pclk_info uart_pclk_info[] = {
	{
		.pclk = &pll5_clk,
		.pclk_mask = AUX_CLK_PLL5_MASK,
	}, {
		.pclk = &pll1_clk,
		.pclk_mask = AUX_CLK_PLL1_MASK,
		.scalable = 1,
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
	.recalc = &aux_clk_recalc,
	.private_data = &uart_config,
};

/* array of all spear 13xx clock lookups */
static struct clk_lookup spear_clk_lookups[] = {
	/* root clks */
	{.con_id = "osc1_24m_clk",	.clk = &osc1_24m_clk},
	{.con_id = "osc2_32k_clk",	.clk = &osc2_32k_clk},
	{.con_id = "osc3_25m_clk",	.clk = &osc3_25m_clk},

	/* clock derived from 32 KHz osc clk */
	{.dev_id = "rtc",		.clk = &rtc_clk},

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
	{ .con_id = "apb_clk",		.clk = &apb_clk},

	/* clocks having multiple parent source from above clocks */
	{.dev_id = "uart",		.clk = &uart_clk},
	{.dev_id = "gpt0",		.clk = &gpt0_clk},
	{.dev_id = "gpt1",		.clk = &gpt1_clk},
	{.dev_id = "gpt2",		.clk = &gpt2_clk},
	{.dev_id = "gpt3",		.clk = &gpt3_clk},

	/* clock derived from ahb/apb clk */
	{ .dev_id = "smi",		.clk = &smi_clk},
	{ .dev_id = "wdt",		.clk = &wdt_clk},
};

void __init clk_init(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(spear_clk_lookups); i++)
		clk_register(&spear_clk_lookups[i]);

	recalc_root_clocks();
}
