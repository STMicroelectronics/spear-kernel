/*
 * arch/arm/mach-spear13xx/include/mach/misc_regs.h
 *
 * Miscellaneous registers definitions for spear13xx machine family
 *
 * Copyright (C) 2010 ST Microelectronics
 * Shiraz Hashim <shiraz.hashim@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __MACH_MISC_REGS_H
#define __MACH_MISC_REGS_H

#include <mach/hardware.h>

#define MISC_BASE		IOMEM(VA_SPEAR13XX_MISC_BASE)

/* General Configuration */
#define SOC_CFG			(MISC_BASE + 0x000)
#define BOOTSTRAP_CFG		(MISC_BASE + 0x004)

/* Power Management Registers */
#define PCM_CFG			(MISC_BASE + 0x100)
#define PCM_WKUP_CFG		(MISC_BASE + 0x104)
#define SWITCH_CTR		(MISC_BASE + 0x108)
#define SYS_CLK_CTRL		(MISC_BASE + 0x200)
#define SYS_SW_RES		(MISC_BASE + 0x204)

/* Clock Configuration Registers */
#define SYS_CLK_PLLTIMER	(MISC_BASE + 0x208)
#define SYS_CLK_OSCITIMER	(MISC_BASE + 0x20c)

/* PLL related registers and bit values */
#define PLL_CFG			(MISC_BASE + 0x210)
	/* PLL_CFG bit values */
	#define OSC_24M_VAL			0
	#define OSC_25M_VAL			1
	#define PLL_CLK_MASK			3
	#define PLL1_CLK_SHIFT			20
	#define PLL2_CLK_SHIFT			22
	#define PLL3_CLK_SHIFT			24
	#define CLCD_SYNT_PLL1_DIV4_VAL		0
	#define CLCD_SYNT_PLL2_VAL		1
	#define CLCD_SYNT_CLK_MASK		1
	#define CLCD_SYNT_CLK_SHIFT		31

#define PLL1_CTR		(MISC_BASE + 0x214)
#define PLL1_FRQ		(MISC_BASE + 0x218)
#define PLL1_MOD		(MISC_BASE + 0x21c)
#define PLL2_CTR		(MISC_BASE + 0x220)
#define PLL2_FRQ		(MISC_BASE + 0x224)
#define PLL2_MOD		(MISC_BASE + 0x228)
#define PLL3_CTR		(MISC_BASE + 0x22c)
#define PLL3_FRQ		(MISC_BASE + 0x230)
#define PLL3_MOD		(MISC_BASE + 0x234)
#define PLL4_CTR		(MISC_BASE + 0x238)
	/* PLL_CTR register masks */
	#define PLL_ENABLE		2
	#define PLL_MODE_SHIFT		4
	#define PLL_MODE_MASK		3
	#define PLL_MODE_NORMAL		0
	#define PLL_MODE_FRACTION	1
	#define PLL_MODE_DITH_DSB	2
	#define PLL_MODE_DITH_SSB	3

#define PLL4_FRQ		(MISC_BASE + 0x23c)
	/* PLL FRQ register masks */
	#define PLL_DIV_N_SHIFT		0
	#define PLL_DIV_N_MASK		0xFF
	#define PLL_DIV_P_SHIFT		8
	#define PLL_DIV_P_MASK		0x7
	#define PLL_NORM_FDBK_M_SHIFT	24
	#define PLL_NORM_FDBK_M_MASK	0xFF
	#define PLL_DITH_FDBK_M_SHIFT	16
	#define PLL_DITH_FDBK_M_MASK	0xFFFF

#define PLL4_MOD		(MISC_BASE + 0x240)

#define PERIP_CLK_CFG		(MISC_BASE + 0x244)
	/* PERIP_CLK_CFG bit values */
	#define GPT_OSC24_VAL		0
	#define GPT_APB_VAL		1
	#define GPT_CLK_MASK		1
	#define GPT0_CLK_SHIFT		8
	#define GPT1_CLK_SHIFT		9
	#define GPT2_CLK_SHIFT		12
	#define GPT3_CLK_SHIFT		13
	#define AUX_CLK_PLL5_VAL	0
	#define AUX_CLK_SYNT_VAL	1
	#define UART_CLK_MASK		1
	#define UART_CLK_SHIFT		4
	#define CLCD_PLL5_VAL		0
	#define CLCD_SYNT_MASK		1
	#define CLCD_CLK_MASK		3
	#define CLCD_CLK_SHIFT		2
	#define C3_CLK_MASK		1
	#define C3_CLK_SHIFT		1

#define GMAC_CLK_CFG		(MISC_BASE + 0x248)

	#define GMAC_PHY_PAD_VAL		0
	#define GMAC_PHY_PLL2_VAL		1
	#define GMAC_PHY_OSC3_VAL		2
	#define GMAC_PHY_INPUT_CLK_MASK		3
	#define GMAC_PHY_INPUT_CLK_SHIFT	1
	#define GMAC_PHY_SYNT_ENB		3
	#define GMAC_PHY_CLK_MASK		1
	#define GMAC_PHY_CLK_SHIFT		3
	#define GMAC_PHY_SYNT_ENB_VAL		4

#define C3_CLK_SYNT		(MISC_BASE + 0x24c)
#define CLCD_CLK_SYNT		(MISC_BASE + 0x250)
	/* CLCD synth reg masks */
	#define CLCD_SYNT_ENB			31
	#define CLCD_SYNT_DIV_FACTOR_MASK	0x1ffff
	#define CLCD_SYNT_DIV_FACTOR_SHIFT	0

#define UART_CLK_SYNT		(MISC_BASE + 0x254)
#define GMAC_CLK_SYNT		(MISC_BASE + 0x258)
#define SDHCI_CLK_SYNT		(MISC_BASE + 0x25c)
#define CFXD_CLK_SYNT		(MISC_BASE + 0x260)
#define RAS_CLK_SYNT0		(MISC_BASE + 0x264)
#define RAS_CLK_SYNT1		(MISC_BASE + 0x268)
#define RAS_CLK_SYNT2		(MISC_BASE + 0x26c)
#define RAS_CLK_SYNT3		(MISC_BASE + 0x270)
	/* aux clk synthesizer register masks */
	#define AUX_SYNT_ENB		31
	#define AUX_EQ_SEL_SHIFT	30
	#define AUX_EQ_SEL_MASK		1
	#define AUX_EQ1_SEL		0
	#define AUX_EQ2_SEL		1
	#define AUX_XSCALE_SHIFT	16
	#define AUX_XSCALE_MASK		0xFFF
	#define AUX_YSCALE_SHIFT	0
	#define AUX_YSCALE_MASK		0xFFF

#define PERIP1_CLK_ENB		(MISC_BASE + 0x274)
	/* PERIP1_CLK_ENB register masks */
	#define BUS_CLK_ENB		0
	#define SYSROM_CLK_ENB		1
	#define SYSRAM1_CLK_ENB		2
	#define SYSRAM0_CLK_ENB		3
	#define FSMC_CLK_ENB		4
	#define SMI_CLK_ENB		5
	#define SDHCI_CLK_ENB		6
	#define CFXD_CLK_ENB		7
	#define GMAC_CLK_ENB		8
	#define UHC0_CLK_ENB		9
	#define UHC1_CLK_ENB		10
	#define USBD_CLK_ENB		11
	#define PCIE0_CLK_ENB		12
	#define PCIE1_CLK_ENB		13
	#define PCIE2_CLK_ENB		14
	#define UART_CLK_ENB		15
	#define SSP_CLK_ENB		17
	#define I2C_CLK_ENB		18
	#define I2S0_CLK_ENB		19
	#define I2S1_CLK_ENB		20
	#define GPT0_CLK_ENB		21
	#define GPT1_CLK_ENB		22
	#define GPIO0_CLK_ENB		23
	#define GPIO1_CLK_ENB		24
	#define DMA0_CLK_ENB		25
	#define DMA1_CLK_ENB		26
	#define CLCD_CLK_ENB		27
	#define JPEG_CLK_ENB		28
	#define C3_CLK_ENB		29
	#define ADC_CLK_ENB		30
	#define RTC_CLK_ENB		31

#define PERIP2_CLK_ENB		(MISC_BASE + 0x278)
	/* PERIP2_CLK_ENB register masks */
	#define DDR_CTRL_CLK_ENB	0
	#define DDR_CORE_CLK_ENB	1
	#define CPU_DBG_CLK_ENB		2
	#define KBD_CLK_ENB		3
	#define GPT2_CLK_ENB		4
	#define GPT3_CLK_ENB		5
	#define ACP_CLK_ENB		6
	#define I2S_REFOUT_CLK_ENB	7
	#define THSENS_CLK_ENB		8

#define PERIP1_SW_RST		(MISC_BASE + 0x27c)
	#define JPEG_SOF_RST		28
#define PERIP2_SW_RST		(MISC_BASE + 0x280)
#define RAS_CLK_ENB		(MISC_BASE + 0x284)
#define RAS_SW_RST		(MISC_BASE + 0x288)
#define PLL1_SYNT		(MISC_BASE + 0x28c)
#define I2S_CLK_CFG		(MISC_BASE + 0x290)

/* Peripheral Configuration Registers */
#define DMAC_HS_SEL		(MISC_BASE + 0x300)
#define DMAC_SEL		(MISC_BASE + 0x304)
#define DMAC_FLOW_SEL		(MISC_BASE + 0x308)
#define DMAC_DIR_SEL		(MISC_BASE + 0x30c)
#define DMAC_CFG		(MISC_BASE + 0x310)
#define USBPHY_GEN_CFG		(MISC_BASE + 0x314)
#define USBPHY_P1_CFG		(MISC_BASE + 0x318)
#define USBPHY_P2_CFG		(MISC_BASE + 0x31c)
#define USBPHY_P3_CFG		(MISC_BASE + 0x320)
#define PCIE_CFG		(MISC_BASE + 0x324)
	/* PCIE CFG MASks */
	#define PCIE2_CFG_AUX_CLK	(1 << 0)
	#define PCIE1_CFG_AUX_CLK	(1 << 1)
	#define PCIE0_CFG_AUX_CLK	(1 << 2)
	#define PCIE2_CFG_CORE_CLK	(1 << 3)
	#define PCIE1_CFG_CORE_CLK	(1 << 4)
	#define PCIE0_CFG_CORE_CLK	(1 << 5)
	#define PCIE2_CFG_POWERUP_RESET	(1 << 6)
	#define PCIE1_CFG_POWERUP_RESET	(1 << 7)
	#define PCIE0_CFG_POWERUP_RESET	(1 << 8)
	#define PCIE2_CFG_DEVICE_PRESENT	(1 << 9)
	#define PCIE1_CFG_DEVICE_PRESENT	(1 << 10)
	#define PCIE0_CFG_DEVICE_PRESENT	(1 << 11)
	#define PCIE0_CFG_VAL	(PCIE0_CFG_AUX_CLK | PCIE0_CFG_CORE_CLK \
			| PCIE0_CFG_POWERUP_RESET | PCIE0_CFG_DEVICE_PRESENT)
	#define PCIE1_CFG_VAL	(PCIE1_CFG_AUX_CLK | PCIE1_CFG_CORE_CLK \
			| PCIE1_CFG_POWERUP_RESET | PCIE1_CFG_DEVICE_PRESENT)
	#define PCIE2_CFG_VAL	(PCIE2_CFG_AUX_CLK | PCIE2_CFG_CORE_CLK \
			| PCIE2_CFG_POWERUP_RESET | PCIE2_CFG_DEVICE_PRESENT)

#define PCIE_MIPHY_CFG		(MISC_BASE + 0x328)
#define PERIP_CFG		(MISC_BASE + 0x32c)
	#define MCIF_SEL_SHIFT	3
	#define MCIF_SEL_MASK	0x3
	#define SD_MMC_ACTIVE	0x1
	#define CF_MMC_ACTIVE	0x2
	#define XD_MMC_ACTIVE	0x3
#define FSMC_CFG		(MISC_BASE + 0x330)
	/* FSMC_CFG register masks */
	#define FSMC_MEMSEL_MASK	0x3
	#define FSMC_MEMSEL_SHIFT	0
	#define FSMC_MEM_NOR		0
	#define FSMC_MEM_NAND		1
	#define FSMC_MEM_SRAM		2
	#define NAND_BANK_MASK		0x3
	#define NAND_BANK_SHIFT		2
	#define NAND_DEV_WIDTH16	4

#define MPMC_CTR_STS		(MISC_BASE + 0x334)

/* Inter-Processor Communication Registers */
#define PRC1_LOCK_CTR		(MISC_BASE + 0x500)
#define PRC2_LOCK_CTR		(MISC_BASE + 0x504)
#define PRC1_IRQ_CTR		(MISC_BASE + 0x508)
#define PRC2_IRQ_CTR		(MISC_BASE + 0x51C)

/* Pad Configuration Registers */
#define PAD_PU_CFG_1		(MISC_BASE + 0x600)
#define PAD_PU_CFG_2		(MISC_BASE + 0x604)
#define PAD_PU_CFG_3		(MISC_BASE + 0x608)
#define PAD_PU_CFG_4		(MISC_BASE + 0x60c)
#define PAD_PU_CFG_5		(MISC_BASE + 0x610)
#define PAD_PU_CFG_6		(MISC_BASE + 0x614)
#define PAD_PU_CFG_7		(MISC_BASE + 0x618)
#define PAD_PU_CFG_8		(MISC_BASE + 0x61c)
#define PAD_PD_CFG_1		(MISC_BASE + 0x620)
#define PAD_PD_CFG_2		(MISC_BASE + 0x624)
#define PAD_PD_CFG_3		(MISC_BASE + 0x628)
#define PAD_PD_CFG_4		(MISC_BASE + 0x62c)
#define PAD_PD_CFG_5		(MISC_BASE + 0x630)
#define PAD_PD_CFG_6		(MISC_BASE + 0x634)
#define PAD_PD_CFG_7		(MISC_BASE + 0x638)
#define PAD_PD_CFG_8		(MISC_BASE + 0x63c)
#define PAD_SLEEP_CFG		(MISC_BASE + 0x640)
#define PAD_HYST_CFG		(MISC_BASE + 0x644)
#define PAD_DRV_CFG		(MISC_BASE + 0x648)
#define PAD_SLEW_CFG		(MISC_BASE + 0x64c)
#define PAD_FUNCTION_EN_1	(MISC_BASE + 0x650)
#define PAD_FUNCTION_EN_2	(MISC_BASE + 0x654)
#define PAD_FUNCTION_EN_3	(MISC_BASE + 0x658)
#define DDR_PAD_CFG		(MISC_BASE + 0x65c)
#define THSENS_CFG		(MISC_BASE + 0x6c4)

/* Compensation Configuration Registers */
#define COMP_1V8_2V5_3V3__1_CFG	(MISC_BASE + 0x700)
#define COMP_1V8_2V5_3V3__2_CFG	(MISC_BASE + 0x704)
#define COMP_3V3_1_CFG		(MISC_BASE + 0x708)
#define COMP_3V3_2_CFG		(MISC_BASE + 0x70c)
#define COMP_DDR_CFG		(MISC_BASE + 0x710)

/* OTP Programming Registers */
#define OTP_PROG_CTR		(MISC_BASE + 0x800)
#define OTP_WDATA1_1		(MISC_BASE + 0x804)
#define OTP_WDATA1_2		(MISC_BASE + 0x808)
#define OTP_WDATA1_3		(MISC_BASE + 0x80c)
#define OTP_WDATA1_4		(MISC_BASE + 0x810)
#define OTP_WDATA1_5		(MISC_BASE + 0x814)
#define OTP_WDATA1_6		(MISC_BASE + 0x818)
#define OTP_WDATA1_7		(MISC_BASE + 0x81c)
#define OTP_WDATA1_8		(MISC_BASE + 0x820)
#define OTP_WDATA2_1		(MISC_BASE + 0x824)
#define OTP_WDATA2_2		(MISC_BASE + 0x828)
#define OTP_WDATA2_3		(MISC_BASE + 0x82c)
#define OTP_WDATA2_4		(MISC_BASE + 0x830)
#define OTP_WDATA2_5		(MISC_BASE + 0x834)
#define OTP_WDATA2_6		(MISC_BASE + 0x838)
#define OTP_WDATA2_7		(MISC_BASE + 0x83c)
#define OTP_WDATA2_8		(MISC_BASE + 0x840)
#define OTP_MASK_1		(MISC_BASE + 0x844)
#define OTP_MASK_2		(MISC_BASE + 0x848)
#define OTP_MASK_3		(MISC_BASE + 0x84c)
#define OTP_MASK_4		(MISC_BASE + 0x850)
#define OTP_MASK_5		(MISC_BASE + 0x854)
#define OTP_MASK_6		(MISC_BASE + 0x858)
#define OTP_MASK_7		(MISC_BASE + 0x85c)
#define OTP_MASK_8		(MISC_BASE + 0x860)
#define OTP_RDATA1_1		(MISC_BASE + 0x864)
#define OTP_RDATA1_2		(MISC_BASE + 0x868)
#define OTP_RDATA1_3		(MISC_BASE + 0x86c)
#define OTP_RDATA1_4		(MISC_BASE + 0x870)
#define OTP_RDATA1_5		(MISC_BASE + 0x874)
#define OTP_RDATA1_6		(MISC_BASE + 0x878)
#define OTP_RDATA1_7		(MISC_BASE + 0x87c)
#define OTP_RDATA1_8		(MISC_BASE + 0x880)
#define OTP_RDATA2_1		(MISC_BASE + 0x884)
#define OTP_RDATA2_2		(MISC_BASE + 0x888)
#define OTP_RDATA2_3		(MISC_BASE + 0x88c)
#define OTP_RDATA2_4		(MISC_BASE + 0x890)
#define OTP_RDATA2_5		(MISC_BASE + 0x894)
#define OTP_RDATA2_6		(MISC_BASE + 0x898)
#define OTP_RDATA2_7		(MISC_BASE + 0x89c)
#define OTP_RDATA2_8		(MISC_BASE + 0x8a0)
#define OTP_RDATAM_1		(MISC_BASE + 0x8a4)
#define OTP_RDATAM_2		(MISC_BASE + 0x8a8)
#define OTP_RDATAM_3		(MISC_BASE + 0x8ac)
#define OTP_RDATAM_4		(MISC_BASE + 0x8b0)
#define OTP_RDATAM_5		(MISC_BASE + 0x8b4)
#define OTP_RDATAM_6		(MISC_BASE + 0x8b8)
#define OTP_RDATAM_7		(MISC_BASE + 0x8bc)
#define OTP_RDATAM_8		(MISC_BASE + 0x8c0)

/* A9SM Registers */
#define A9SM_CLUSTERID		(MISC_BASE + 0x900)
#define A9SM_STATUS		(MISC_BASE + 0x904)
#define A9SM_DEBUG		(MISC_BASE + 0x908)
#define A9SM_FILTER		(MISC_BASE + 0x90c)
#define A9SM_PARITY_CFG		(MISC_BASE + 0x910)
#define A9SM_PARITY_ERR		(MISC_BASE + 0x914)

/* SOC ID Registers */
#define DIE_ID_1		(MISC_BASE + 0xa00)
#define DIE_ID_2		(MISC_BASE + 0xa04)
#define DIE_ID_3		(MISC_BASE + 0xa08)
#define DIE_ID_4		(MISC_BASE + 0xa0c)
#define DIE_ID_VALID		(MISC_BASE + 0xa10)

/* SOC TEST & DEBUG Registers */
#define MIPHY_TEST		(MISC_BASE + 0x1000)
#define PCIE_MSTR_P0		(MISC_BASE + 0x1004)
#define PCIE_AWMISC_P0		(MISC_BASE + 0x1008)
#define PCIE_ARMISC_P0		(MISC_BASE + 0x100c)
#define PCIE_MSTR_P1		(MISC_BASE + 0x1010)
#define PCIE_AWMISC_P1		(MISC_BASE + 0x1014)
#define PCIE_ARMISC_P1		(MISC_BASE + 0x1018)
#define PCIE_MSTR_P2		(MISC_BASE + 0x101c)
#define PCIE_AWMISC_P2		(MISC_BASE + 0x1020)
#define PCIE_ARMISC_P2		(MISC_BASE + 0x1024)

#endif /* __MACH_MISC_REGS_H */
