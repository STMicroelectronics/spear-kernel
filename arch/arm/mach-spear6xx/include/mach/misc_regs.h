/*
 * arch/arm/mach-spear6xx/include/mach/misc_regs.h
 *
 * Miscellaneous registers definitions for SPEAr6xx machine family
 *
 * Copyright (C) 2009 ST Microelectronics
 * Viresh Kumar<viresh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __MACH_MISC_REGS_H
#define __MACH_MISC_REGS_H

#include <mach/hardware.h>

#define VA_MISC_BASE		IOMEM(VA_SPEAR6XX_ICM3_MISC_REG_BASE)

#define VA_SOC_CFG_CTR		(VA_MISC_BASE + 0x000)
#define VA_DIAG_CFG_CTR		(VA_MISC_BASE + 0x004)
#define VA_PLL1_CTR		(VA_MISC_BASE + 0x008)
#define VA_PLL1_FRQ		(VA_MISC_BASE + 0x00C)
#define VA_PLL1_MOD		(VA_MISC_BASE + 0x010)
#define VA_PLL2_CTR		(VA_MISC_BASE + 0x014)
/* PLL_CTR register masks */
#define PLL_ENABLE		2
#define PLL_MODE_SHIFT		4
#define PLL_MODE_MASK		0x3
#define PLL_MODE_NORMAL		0
#define PLL_MODE_FRACTION	1
#define PLL_MODE_DITH_DSB	2
#define PLL_MODE_DITH_SSB	3

#define PLL_LOCK_SHIFT		0
#define PLL_LOCK_MASK		1

#define VA_PLL2_FRQ		(VA_MISC_BASE + 0x018)
/* PLL FRQ register masks */
#define PLL_DIV_N_SHIFT		0
#define PLL_DIV_N_MASK		0xFF
#define PLL_DIV_P_SHIFT		8
#define PLL_DIV_P_MASK		0x7
#define PLL_NORM_FDBK_M_SHIFT	24
#define PLL_NORM_FDBK_M_MASK	0xFF
#define PLL_DITH_FDBK_M_SHIFT	16
#define PLL_DITH_FDBK_M_MASK	0xFFFF

#define VA_PLL2_MOD		(VA_MISC_BASE + 0x01C)
#define VA_PLL_CLK_CFG		(VA_MISC_BASE + 0x020)
/* PLL_CLK_CFG register masks */
#define MCTR_CLK_SHIFT		28
#define MCTR_CLK_MASK		0x7
#define MCTR_CLK_HCLK_VAL	0x0
#define MCTR_CLK_2HCLK_VAL	0x1
#define MCTR_CLK_PLL2_VAL	0x3

#define VA_CORE_CLK_CFG		(VA_MISC_BASE + 0x024)

/* CORE CLK CFG register masks */
#define PLL_HCLK_RATIO_SHIFT	10
#define PLL_HCLK_RATIO_MASK	0x3
#define HCLK_PCLK_RATIO_SHIFT	8
#define HCLK_PCLK_RATIO_MASK	0x3

#define VA_PERIP_CLK_CFG	(VA_MISC_BASE + 0x028)
/* PERIP_CLK_CFG register masks */
#define CLCD_CLK_SHIFT		2
#define CLCD_CLK_MASK		0x3
#define UART_CLK_SHIFT		4
#define UART_CLK_MASK		0x1
#define FIRDA_CLK_SHIFT		5
#define FIRDA_CLK_MASK		0x3
#define GPT0_CLK_SHIFT		8
#define GPT1_CLK_SHIFT		10
#define GPT2_CLK_SHIFT		11
#define GPT3_CLK_SHIFT		12
#define GPT_CLK_MASK		0x1
#define AUX_CLK_PLL3_VAL	0
#define AUX_CLK_PLL1_VAL	1

#define VA_PERIP1_CLK_ENB	(VA_MISC_BASE + 0x02C)
/* PERIP1_CLK_ENB register masks */
#define UART0_CLK_ENB		3
#define UART1_CLK_ENB		4
#define SSP0_CLK_ENB		5
#define SSP1_CLK_ENB		6
#define I2C_CLK_ENB		7
#define JPEG_CLK_ENB		8
#define FSMC_CLK_ENB		9
#define FIRDA_CLK_ENB		10
#define GPT2_CLK_ENB		11
#define GPT3_CLK_ENB		12
#define GPIO2_CLK_ENB		13
#define SSP2_CLK_ENB		14
#define ADC_CLK_ENB		15
#define GPT1_CLK_ENB		11
#define RTC_CLK_ENB		17
#define GPIO1_CLK_ENB		18
#define DMA_CLK_ENB		19
#define SMI_CLK_ENB		21
#define CLCD_CLK_ENB		22
#define GMAC_CLK_ENB		23
#define USBD_CLK_ENB		24
#define USBH0_CLK_ENB		25
#define USBH1_CLK_ENB		26

#define VA_SOC_CORE_ID		(VA_MISC_BASE + 0x030)
#define VA_RAS_CLK_ENB		(VA_MISC_BASE + 0x034)
#define VA_PERIP1_SOF_RST	(VA_MISC_BASE + 0x038)
/* PERIP1_SOF_RST register masks */
#define JPEG_SOF_RST		8

#define VA_SOC_USER_ID		(VA_MISC_BASE + 0x03C)
#define VA_RAS_SOF_RST		(VA_MISC_BASE + 0x040)
#define VA_PRSC1_CLK_CFG	(VA_MISC_BASE + 0x044)
#define VA_PRSC2_CLK_CFG	(VA_MISC_BASE + 0x048)
#define VA_PRSC3_CLK_CFG	(VA_MISC_BASE + 0x04C)
/* gpt synthesizer register masks */
#define GPT_MSCALE_SHIFT	0
#define GPT_MSCALE_MASK		0xFFF
#define GPT_NSCALE_SHIFT	12
#define GPT_NSCALE_MASK		0xF

#define VA_AMEM_CLK_CFG		(VA_MISC_BASE + 0x050)
#define VA_EXPI_CLK_CFG		(VA_MISC_BASE + 0x054)
#define VA_CLCD_CLK_SYNT	(VA_MISC_BASE + 0x05C)
#define VA_FIRDA_CLK_SYNT	(VA_MISC_BASE + 0x060)
#define VA_UART_CLK_SYNT	(VA_MISC_BASE + 0x064)
#define VA_GMAC_CLK_SYNT	(VA_MISC_BASE + 0x068)
#define VA_RAS1_CLK_SYNT	(VA_MISC_BASE + 0x06C)
#define VA_RAS2_CLK_SYNT	(VA_MISC_BASE + 0x070)
#define VA_RAS3_CLK_SYNT	(VA_MISC_BASE + 0x074)
#define VA_RAS4_CLK_SYNT	(VA_MISC_BASE + 0x078)
/* aux clk synthesiser register masks for irda to ras4 */
#define AUX_SYNT_ENB		31
#define AUX_EQ_SEL_SHIFT	30
#define AUX_EQ_SEL_MASK		1
#define AUX_EQ1_SEL		0
#define AUX_EQ2_SEL		1
#define AUX_XSCALE_SHIFT	16
#define AUX_XSCALE_MASK		0xFFF
#define AUX_YSCALE_SHIFT	0
#define AUX_YSCALE_MASK		0xFFF

#define VA_ICM1_ARB_CFG		(VA_MISC_BASE + 0x07C)
#define VA_ICM2_ARB_CFG		(VA_MISC_BASE + 0x080)
#define VA_ICM3_ARB_CFG		(VA_MISC_BASE + 0x084)
#define VA_ICM4_ARB_CFG		(VA_MISC_BASE + 0x088)
#define VA_ICM5_ARB_CFG		(VA_MISC_BASE + 0x08C)
#define VA_ICM6_ARB_CFG		(VA_MISC_BASE + 0x090)
#define VA_ICM7_ARB_CFG		(VA_MISC_BASE + 0x094)
#define VA_ICM8_ARB_CFG		(VA_MISC_BASE + 0x098)
#define VA_ICM9_ARB_CFG		(VA_MISC_BASE + 0x09C)
#define VA_DMA_CHN_CFG		(VA_MISC_BASE + 0x0A0)
#define VA_USB2_PHY_CFG		(VA_MISC_BASE + 0x0A4)
#define VA_GMAC_CFG_CTR		(VA_MISC_BASE + 0x0A8)
#define VA_EXPI_CFG_CTR		(VA_MISC_BASE + 0x0AC)
#define VA_PRC1_LOCK_CTR	(VA_MISC_BASE + 0x0C0)
#define VA_PRC2_LOCK_CTR	(VA_MISC_BASE + 0x0C4)
#define VA_PRC3_LOCK_CTR	(VA_MISC_BASE + 0x0C8)
#define VA_PRC4_LOCK_CTR	(VA_MISC_BASE + 0x0CC)
#define VA_PRC1_IRQ_CTR		(VA_MISC_BASE + 0x0D0)
#define VA_PRC2_IRQ_CTR		(VA_MISC_BASE + 0x0D4)
#define VA_PRC3_IRQ_CTR		(VA_MISC_BASE + 0x0D8)
#define VA_PRC4_IRQ_CTR		(VA_MISC_BASE + 0x0DC)
#define VA_PWRDOWN_CFG_CTR	(VA_MISC_BASE + 0x0E0)
#define VA_COMPSSTL_1V8_CFG	(VA_MISC_BASE + 0x0E4)
#define VA_COMPSSTL_2V5_CFG	(VA_MISC_BASE + 0x0E8)
#define VA_COMPCOR_3V3_CFG	(VA_MISC_BASE + 0x0EC)
#define VA_SSTLPAD_CFG_CTR	(VA_MISC_BASE + 0x0F0)
#define VA_BIST1_CFG_CTR	(VA_MISC_BASE + 0x0F4)
#define VA_BIST2_CFG_CTR	(VA_MISC_BASE + 0x0F8)
#define VA_BIST3_CFG_CTR	(VA_MISC_BASE + 0x0FC)
#define VA_BIST4_CFG_CTR	(VA_MISC_BASE + 0x100)
#define VA_BIST5_CFG_CTR	(VA_MISC_BASE + 0x104)
#define VA_BIST1_STS_RES	(VA_MISC_BASE + 0x108)
#define VA_BIST2_STS_RES	(VA_MISC_BASE + 0x10C)
#define VA_BIST3_STS_RES	(VA_MISC_BASE + 0x110)
#define VA_BIST4_STS_RES	(VA_MISC_BASE + 0x114)
#define VA_BIST5_STS_RES	(VA_MISC_BASE + 0x118)
#define VA_SYSERR_CFG_CTR	(VA_MISC_BASE + 0x11C)

#endif /* __MACH_MISC_REGS_H */
