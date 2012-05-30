/*
 * arch/arm/mach-spear3xx/spear320.c
 *
 * SPEAr320 machine source file
 *
 * Copyright (C) 2009 ST Microelectronics
 * Viresh Kumar<viresh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/amba/pl022.h>
#include <linux/amba/pl08x.h>
#include <linux/amba/serial.h>
#include <linux/can/platform/c_can.h>
#include <linux/designware_i2s.h>
#include <linux/mtd/physmap.h>
#include <linux/ptrace.h>
#include <linux/types.h>
#include <linux/mmc/sdhci-spear.h>
#include <linux/mtd/fsmc.h>
#include <asm/irq.h>
#include <plat/clock.h>
#include <plat/pl080.h>
#include <plat/shirq.h>
#include <mach/generic.h>
#include <mach/gpio.h>
#include <mach/hardware.h>
#include <mach/macb_eth.h>
#include <mach/misc_regs.h>
#include <sound/pcm.h>

/* modes */
#define AUTO_NET_SMII_MODE	(1 << 0)
#define AUTO_NET_MII_MODE	(1 << 1)
#define AUTO_EXP_MODE		(1 << 2)
#define SMALL_PRINTERS_MODE	(1 << 3)
#define ALL_LEGACY_MODES	0xF
#define SPEAR320S_EXTENDED_MODE	(1 << 4)

struct pmx_mode spear320_auto_net_smii_mode = {
	.id = AUTO_NET_SMII_MODE,
	.name = "Automation Networking SMII Mode",
	.value = AUTO_NET_SMII_MODE_VAL,
};

struct pmx_mode spear320_auto_net_mii_mode = {
	.id = AUTO_NET_MII_MODE,
	.name = "Automation Networking MII Mode",
	.value = AUTO_NET_MII_MODE_VAL,
};

struct pmx_mode spear320_auto_exp_mode = {
	.id = AUTO_EXP_MODE,
	.name = "Automation Expanded Mode",
	.value = AUTO_EXP_MODE_VAL,
};

struct pmx_mode spear320_small_printers_mode = {
	.id = SMALL_PRINTERS_MODE,
	.name = "Small Printers Mode",
	.value = SMALL_PRINTERS_MODE_VAL,
};

struct pmx_mode spear320s_extended_mode = {
	.id = SPEAR320S_EXTENDED_MODE,
	.name = "spear320s extended mode",
	.value = SPEAR320S_EXTENDED_MODE_VAL,
};

/* devices */
/* Pad multiplexing for CLCD device */
static struct pmx_mux_reg pmx_clcd_mux[] = {
	{
		.address = SPEAR320S_IP_SEL_PAD_60_69_REG,
		.mask = SPEAR320S_PMX_PL_69_MASK,
		.value = SPEAR320S_PMX_CLCD_PL_69_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_70_79_REG,
		.mask = SPEAR320S_PMX_PL_70_MASK | SPEAR320S_PMX_PL_71_72_MASK |
			SPEAR320S_PMX_PL_73_MASK | SPEAR320S_PMX_PL_74_MASK |
			SPEAR320S_PMX_PL_75_76_MASK |
			SPEAR320S_PMX_PL_77_78_79_MASK,
		.value = SPEAR320S_PMX_CLCD_PL_70_VAL |
			SPEAR320S_PMX_CLCD_PL_71_72_VAL |
			SPEAR320S_PMX_CLCD_PL_73_VAL |
			SPEAR320S_PMX_CLCD_PL_74_VAL |
			SPEAR320S_PMX_CLCD_PL_75_76_VAL |
			SPEAR320S_PMX_CLCD_PL_77_78_79_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_80_89_REG,
		.mask = SPEAR320S_PMX_PL_80_TO_85_MASK |
			SPEAR320S_PMX_PL_86_87_MASK |
			SPEAR320S_PMX_PL_88_89_MASK,
		.value = SPEAR320S_PMX_CLCD_PL_80_TO_85_VAL |
			SPEAR320S_PMX_CLCD_PL_86_87_VAL |
			SPEAR320S_PMX_CLCD_PL_88_89_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_90_99_REG,
		.mask = SPEAR320S_PMX_PL_90_91_MASK |
			SPEAR320S_PMX_PL_92_93_MASK |
			SPEAR320S_PMX_PL_94_95_MASK |
			SPEAR320S_PMX_PL_96_97_MASK | SPEAR320S_PMX_PL_98_MASK,
		.value = SPEAR320S_PMX_CLCD_PL_90_91_VAL |
			SPEAR320S_PMX_CLCD_PL_92_93_VAL |
			SPEAR320S_PMX_CLCD_PL_94_95_VAL |
			SPEAR320S_PMX_CLCD_PL_96_97_VAL |
			SPEAR320S_PMX_CLCD_PL_98_VAL,
	},
};

static struct pmx_dev_mode pmx_clcd_modes[] = {
	{
		.ids = SPEAR320S_EXTENDED_MODE,
		.mux_regs = pmx_clcd_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_clcd_mux),
	},
};

struct pmx_dev spear320s_pmx_clcd = {
	.name = "clcd",
	.modes = pmx_clcd_modes,
	.mode_count = ARRAY_SIZE(pmx_clcd_modes),
};

/* Pad multiplexing for EMI (Parallel NOR flash) device */
static struct pmx_mux_reg pmx_emi_mux1[] = {
	{
		.mask = PMX_TIMER_1_2_MASK | PMX_TIMER_3_4_MASK,
		.value = 0,
	},
};

static struct pmx_mux_reg pmx_emi_ext_mux[] = {
	{
		.address = SPEAR320S_IP_SEL_PAD_40_49_REG,
		.mask = SPEAR320S_PMX_PL_46_47_MASK |
			SPEAR320S_PMX_PL_48_49_MASK,
		.value = SPEAR320S_PMX_FSMC_EMI_PL_46_47_VAL |
			SPEAR320S_PMX_FSMC_EMI_PL_48_49_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_50_59_REG,
		.mask = SPEAR320S_PMX_PL_50_51_MASK |
			SPEAR320S_PMX_PL_52_53_MASK |
			SPEAR320S_PMX_PL_54_55_56_MASK |
			SPEAR320S_PMX_PL_58_59_MASK,
		.value = SPEAR320S_PMX_EMI_PL_50_51_VAL |
			SPEAR320S_PMX_EMI_PL_52_53_VAL |
			SPEAR320S_PMX_FSMC_EMI_PL_54_55_56_VAL |
			SPEAR320S_PMX_FSMC_EMI_PL_58_59_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_60_69_REG,
		.mask = SPEAR320S_PMX_PL_69_MASK,
		.value = SPEAR320S_PMX_EMI_PL_69_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_70_79_REG,
		.mask = SPEAR320S_PMX_PL_70_MASK | SPEAR320S_PMX_PL_71_72_MASK |
			SPEAR320S_PMX_PL_73_MASK | SPEAR320S_PMX_PL_74_MASK |
			SPEAR320S_PMX_PL_75_76_MASK |
			SPEAR320S_PMX_PL_77_78_79_MASK,
		.value = SPEAR320S_PMX_FSMC_EMI_PL_70_VAL |
			SPEAR320S_PMX_FSMC_EMI_PL_71_72_VAL |
			SPEAR320S_PMX_FSMC_EMI_PL_73_VAL |
			SPEAR320S_PMX_EMI_PL_74_VAL |
			SPEAR320S_PMX_EMI_PL_75_76_VAL |
			SPEAR320S_PMX_EMI_PL_77_78_79_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_80_89_REG,
		.mask = SPEAR320S_PMX_PL_80_TO_85_MASK |
			SPEAR320S_PMX_PL_86_87_MASK |
			SPEAR320S_PMX_PL_88_89_MASK,
		.value = SPEAR320S_PMX_EMI_PL_80_TO_85_VAL |
			SPEAR320S_PMX_EMI_PL_86_87_VAL |
			SPEAR320S_PMX_EMI_PL_88_89_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_90_99_REG,
		.mask = SPEAR320S_PMX_PL_90_91_MASK |
			SPEAR320S_PMX_PL_92_93_MASK |
			SPEAR320S_PMX_PL_94_95_MASK |
			SPEAR320S_PMX_PL_96_97_MASK,
		.value = SPEAR320S_PMX_EMI1_PL_90_91_VAL |
			SPEAR320S_PMX_EMI1_PL_92_93_VAL |
			SPEAR320S_PMX_EMI1_PL_94_95_VAL |
			SPEAR320S_PMX_EMI1_PL_96_97_VAL,
	}, {
		.address = SPEAR320S_EXT_CTRL_REG,
		.mask = SPEAR320S_EMI_FSMC_DYNAMIC_MUX_MASK,
		.value = SPEAR320S_EMI_FSMC_DYNAMIC_MUX_MASK,
	},
};

static struct pmx_dev_mode pmx_emi_modes[] = {
	{
		.ids = AUTO_EXP_MODE | SPEAR320S_EXTENDED_MODE,
		.mux_regs = pmx_emi_mux1,
		.mux_reg_cnt = ARRAY_SIZE(pmx_emi_mux1),
	}, {
		.ids = SPEAR320S_EXTENDED_MODE,
		.mux_regs = pmx_emi_ext_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_emi_ext_mux),
	},
};

struct pmx_dev spear320_pmx_emi = {
	.name = "emi",
	.modes = pmx_emi_modes,
	.mode_count = ARRAY_SIZE(pmx_emi_modes),
};

/* Pad multiplexing for FSMC (NAND flash) device */
static struct pmx_mux_reg pmx_fsmc_8bit_mux[] = {
	{
		.address = SPEAR320S_IP_SEL_PAD_50_59_REG,
		.mask = SPEAR320S_PMX_PL_52_53_MASK |
			SPEAR320S_PMX_PL_54_55_56_MASK |
			SPEAR320S_PMX_PL_57_MASK | SPEAR320S_PMX_PL_58_59_MASK,
		.value = SPEAR320S_PMX_FSMC_PL_52_53_VAL |
			SPEAR320S_PMX_FSMC_EMI_PL_54_55_56_VAL |
			SPEAR320S_PMX_FSMC_PL_57_VAL |
			SPEAR320S_PMX_FSMC_EMI_PL_58_59_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_60_69_REG,
		.mask = SPEAR320S_PMX_PL_60_MASK |
			SPEAR320S_PMX_PL_61_TO_64_MASK |
			SPEAR320S_PMX_PL_65_TO_68_MASK,
		.value = SPEAR320S_PMX_FSMC_PL_60_VAL |
			SPEAR320S_PMX_FSMC_PL_61_TO_64_VAL |
			SPEAR320S_PMX_FSMC_PL_65_TO_68_VAL,
	}, {
		.address = SPEAR320S_EXT_CTRL_REG,
		.mask = SPEAR320S_EMI_FSMC_DYNAMIC_MUX_MASK,
		.value = SPEAR320S_EMI_FSMC_DYNAMIC_MUX_MASK,
	},
};
static struct pmx_mux_reg pmx_fsmc_16bit_mux[] = {
	{
		.address = SPEAR320S_IP_SEL_PAD_40_49_REG,
		.mask = SPEAR320S_PMX_PL_46_47_MASK |
			SPEAR320S_PMX_PL_48_49_MASK,
		.value = SPEAR320S_PMX_FSMC_EMI_PL_46_47_VAL |
			SPEAR320S_PMX_FSMC_EMI_PL_48_49_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_70_79_REG,
		.mask = SPEAR320S_PMX_PL_70_MASK | SPEAR320S_PMX_PL_71_72_MASK |
			SPEAR320S_PMX_PL_73_MASK,
		.value = SPEAR320S_PMX_FSMC_EMI_PL_70_VAL |
			SPEAR320S_PMX_FSMC_EMI_PL_71_72_VAL |
			SPEAR320S_PMX_FSMC_EMI_PL_73_VAL,
	}
};

static struct pmx_dev_mode pmx_fsmc_8bit_modes[] = {
	{
		.ids = SPEAR320S_EXTENDED_MODE,
		.mux_regs = pmx_fsmc_8bit_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_fsmc_8bit_mux),
	},
};

static struct pmx_dev_mode pmx_fsmc_16bit_modes[] = {
	{
		.ids = SPEAR320S_EXTENDED_MODE,
		.mux_regs = pmx_fsmc_8bit_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_fsmc_8bit_mux),
	}, {
		.ids = SPEAR320S_EXTENDED_MODE,
		.mux_regs = pmx_fsmc_16bit_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_fsmc_16bit_mux),
	},
};

struct pmx_dev spear320s_pmx_fsmc[] = {
	{
		.name = "fsmc - 8bit",
		.modes = pmx_fsmc_8bit_modes,
		.mode_count = ARRAY_SIZE(pmx_fsmc_8bit_modes),
	}, {
		.name = "fsmc - 16bit",
		.modes = pmx_fsmc_16bit_modes,
		.mode_count = ARRAY_SIZE(pmx_fsmc_16bit_modes),
	},
};

/* Pad multiplexing for SPP device */
static struct pmx_mux_reg pmx_spp_mux[] = {
	{
		.address = SPEAR320S_IP_SEL_PAD_60_69_REG,
		.mask = SPEAR320S_PMX_PL_69_MASK,
		.value = SPEAR320S_PMX_SPP_PL_69_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_70_79_REG,
		.mask = SPEAR320S_PMX_PL_70_MASK | SPEAR320S_PMX_PL_71_72_MASK |
			SPEAR320S_PMX_PL_73_MASK | SPEAR320S_PMX_PL_74_MASK |
			SPEAR320S_PMX_PL_75_76_MASK |
			SPEAR320S_PMX_PL_77_78_79_MASK,
		.value = SPEAR320S_PMX_SPP_PL_70_VAL |
			SPEAR320S_PMX_SPP_PL_71_72_VAL |
			SPEAR320S_PMX_SPP_PL_73_VAL |
			SPEAR320S_PMX_SPP_PL_74_VAL |
			SPEAR320S_PMX_SPP_PL_75_76_VAL |
			SPEAR320S_PMX_SPP_PL_77_78_79_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_80_89_REG,
		.mask = SPEAR320S_PMX_PL_80_TO_85_MASK,
		.value = SPEAR320S_PMX_SPP_PL_80_TO_85_VAL,
	},
};

static struct pmx_dev_mode pmx_spp_modes[] = {
	{
		.ids = SPEAR320S_EXTENDED_MODE,
		.mux_regs = pmx_spp_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_spp_mux),
	},
};

struct pmx_dev spear320s_pmx_spp = {
	.name = "spp",
	.modes = pmx_spp_modes,
	.mode_count = ARRAY_SIZE(pmx_spp_modes),
};

/* Pad multiplexing for SDHCI device */
static struct pmx_mux_reg pmx_sdhci_mux[] = {
	{
		.mask = PMX_TIMER_1_2_MASK | PMX_TIMER_3_4_MASK,
		.value = 0,
	},
};

static struct pmx_mux_reg pmx_sdhci_ext_mux[] = {
	{
		.address = SPEAR320S_IP_SEL_PAD_40_49_REG,
		.mask = SPEAR320S_PMX_PL_43_MASK | SPEAR320S_PMX_PL_44_45_MASK |
			SPEAR320S_PMX_PL_46_47_MASK |
			SPEAR320S_PMX_PL_48_49_MASK,
		.value = SPEAR320S_PMX_SDHCI_PL_43_VAL |
			SPEAR320S_PMX_SDHCI_PL_44_45_VAL |
			SPEAR320S_PMX_SDHCI_PL_46_47_VAL |
			SPEAR320S_PMX_SDHCI_PL_48_49_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_50_59_REG,
		.mask = SPEAR320S_PMX_PL_50_MASK,
		.value = SPEAR320S_PMX_SDHCI_PL_50_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_90_99_REG,
		.mask = SPEAR320S_PMX_PL_99_MASK,
		.value = SPEAR320S_PMX_SDHCI_PL_99_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_MIX_PAD_REG,
		.mask = SPEAR320S_PMX_PL_100_101_MASK,
		.value = SPEAR320S_PMX_SDHCI_PL_100_101_VAL,
	},
};

static struct pmx_mux_reg pmx_sdhci_cd_12_mux[] = {
	{
		.address = SPEAR320S_IP_SEL_PAD_10_19_REG,
		.mask = SPEAR320S_PMX_PL_12_MASK,
		.value = SPEAR320S_PMX_SDHCI_CD_PL_12_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_MIX_PAD_REG,
		.mask = SPEAR320S_PMX_SDHCI_CD_PORT_SEL_MASK,
		.value = SPEAR320S_PMX_SDHCI_CD_PORT_12_VAL,
	}, {
		.mask = PMX_MII_MASK,
		.value = 0,
	},
};

static struct pmx_mux_reg pmx_sdhci_cd_51_mux[] = {
	{
		.address = SPEAR320S_IP_SEL_PAD_50_59_REG,
		.mask = SPEAR320S_PMX_PL_51_MASK,
		.value = SPEAR320S_PMX_SDHCI_CD_PL_51_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_MIX_PAD_REG,
		.mask = SPEAR320S_PMX_SDHCI_CD_PORT_SEL_MASK,
		.value = SPEAR320S_PMX_SDHCI_CD_PORT_51_VAL,
	},
};

#define pmx_sdhci_common_modes						\
	{								\
		.ids = AUTO_NET_SMII_MODE | AUTO_NET_MII_MODE |		\
			SMALL_PRINTERS_MODE | SPEAR320S_EXTENDED_MODE,	\
		.mux_regs = pmx_sdhci_mux,				\
		.mux_reg_cnt = ARRAY_SIZE(pmx_sdhci_mux),		\
	}, {								\
		.ids = SPEAR320S_EXTENDED_MODE,				\
		.mux_regs = pmx_sdhci_ext_mux,				\
		.mux_reg_cnt = ARRAY_SIZE(pmx_sdhci_ext_mux),		\
	}

static struct pmx_dev_mode pmx_sdhci_modes[][3] = {
	{
		/* select pin 12 for cd */
		pmx_sdhci_common_modes,
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_sdhci_cd_12_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_sdhci_cd_12_mux),
		},
	}, {
		/* select pin 51 for cd */
		pmx_sdhci_common_modes,
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_sdhci_cd_51_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_sdhci_cd_51_mux),
		},
	}
};

struct pmx_dev spear320_pmx_sdhci[] = {
	{
		.name = "sdhci, cd-12",
		.modes = pmx_sdhci_modes[0],
		.mode_count = ARRAY_SIZE(pmx_sdhci_modes[0]),
	}, {
		.name = "sdhci, cd-51",
		.modes = pmx_sdhci_modes[1],
		.mode_count = ARRAY_SIZE(pmx_sdhci_modes[1]),
	},
};

/* Pad multiplexing for I2S device */
static struct pmx_mux_reg pmx_i2s_mux[] = {
	{
		.mask = PMX_UART0_MODEM_MASK,
		.value = 0,
	},
};

static struct pmx_mux_reg pmx_i2s_ext_mux[] = {
	{
		.mask = PMX_SSP_CS_MASK,
		.value = 0,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_30_39_REG,
		.mask = SPEAR320S_PMX_PL_35_MASK | SPEAR320S_PMX_PL_39_MASK,
		.value = SPEAR320S_PMX_I2S_REF_CLK_PL_35_VAL |
			SPEAR320S_PMX_I2S_PL_39_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_40_49_REG,
		.mask = SPEAR320S_PMX_PL_40_MASK | SPEAR320S_PMX_PL_41_42_MASK,
		.value = SPEAR320S_PMX_I2S_PL_40_VAL |
			SPEAR320S_PMX_I2S_PL_41_42_VAL,
	},
};

static struct pmx_dev_mode pmx_i2s_modes[] = {
	{
		.ids = AUTO_NET_SMII_MODE | AUTO_NET_MII_MODE |
			SPEAR320S_EXTENDED_MODE,
		.mux_regs = pmx_i2s_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_i2s_mux),
	}, {
		.ids = SPEAR320S_EXTENDED_MODE,
		.mux_regs = pmx_i2s_ext_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_i2s_ext_mux),
	},
};

struct pmx_dev spear320_pmx_i2s = {
	.name = "i2s",
	.modes = pmx_i2s_modes,
	.mode_count = ARRAY_SIZE(pmx_i2s_modes),
};

/* Pad multiplexing for UART1 device */
static struct pmx_mux_reg pmx_uart1_mux[] = {
	{
		.mask = PMX_GPIO_PIN0_MASK | PMX_GPIO_PIN1_MASK,
		.value = 0,
	},
};

static struct pmx_mux_reg pmx_uart1_ext_mux[] = {
	{
		.address = SPEAR320S_IP_SEL_PAD_20_29_REG,
		.mask = SPEAR320S_PMX_PL_28_29_MASK,
		.value = SPEAR320S_PMX_UART1_PL_28_29_VAL,
	},
};

static struct pmx_dev_mode pmx_uart1_modes[] = {
	{
		.ids = ALL_LEGACY_MODES | SPEAR320S_EXTENDED_MODE,
		.mux_regs = pmx_uart1_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_uart1_mux),
	}, {
		.ids = SPEAR320S_EXTENDED_MODE,
		.mux_regs = pmx_uart1_ext_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_uart1_ext_mux),
	},
};

struct pmx_dev spear320_pmx_uart1 = {
	.name = "uart1",
	.modes = pmx_uart1_modes,
	.mode_count = ARRAY_SIZE(pmx_uart1_modes),
};

/* Pad multiplexing for UART1 Modem device */
static struct pmx_mux_reg pmx_uart1_modem_autoexp_mux[] = {
	{
		.mask = PMX_TIMER_1_2_MASK | PMX_TIMER_3_4_MASK |
			PMX_SSP_CS_MASK,
		.value = 0,
	},
};

static struct pmx_mux_reg pmx_uart1_modem_smallpri_mux[] = {
	{
		.mask = PMX_GPIO_PIN3_MASK | PMX_GPIO_PIN4_MASK |
			PMX_GPIO_PIN5_MASK | PMX_SSP_CS_MASK,
		.value = 0,
	},
};

static struct pmx_mux_reg pmx_uart1_modem_ext_2_7_mux[] = {
	{
		.mask = PMX_UART0_MASK | PMX_I2C_MASK | PMX_SSP_MASK,
		.value = 0,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_0_9_REG,
		.mask = SPEAR320S_PMX_PL_2_3_MASK | SPEAR320S_PMX_PL_6_7_MASK,
		.value = SPEAR320S_PMX_UART1_ENH_PL_2_3_VAL |
			SPEAR320S_PMX_UART1_ENH_PL_4_5_VAL |
			SPEAR320S_PMX_UART1_ENH_PL_6_7_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_MIX_PAD_REG,
		.mask = SPEAR320S_PMX_UART1_ENH_PORT_SEL_MASK,
		.value = SPEAR320S_PMX_UART1_ENH_PORT_3_TO_5_7_VAL,
	},
};

static struct pmx_mux_reg pmx_uart1_modem_ext_31_36_mux[] = {
	{
		.mask = PMX_GPIO_PIN3_MASK | PMX_GPIO_PIN4_MASK |
			PMX_GPIO_PIN5_MASK | PMX_SSP_CS_MASK,
		.value = 0,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_30_39_REG,
		.mask = SPEAR320S_PMX_PL_31_MASK | SPEAR320S_PMX_PL_32_33_MASK |
			SPEAR320S_PMX_PL_34_MASK | SPEAR320S_PMX_PL_35_MASK |
			SPEAR320S_PMX_PL_36_MASK,
		.value = SPEAR320S_PMX_UART1_ENH_PL_31_VAL |
			SPEAR320S_PMX_UART1_ENH_PL_32_33_VAL |
			SPEAR320S_PMX_UART1_ENH_PL_34_VAL |
			SPEAR320S_PMX_UART1_ENH_PL_35_VAL |
			SPEAR320S_PMX_UART1_ENH_PL_36_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_MIX_PAD_REG,
		.mask = SPEAR320S_PMX_UART1_ENH_PORT_SEL_MASK,
		.value = SPEAR320S_PMX_UART1_ENH_PORT_32_TO_34_36_VAL,
	},
};

static struct pmx_mux_reg pmx_uart1_modem_ext_34_45_mux[] = {
	{
		.mask = PMX_TIMER_1_2_MASK | PMX_TIMER_3_4_MASK |
			PMX_SSP_CS_MASK,
		.value = 0,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_30_39_REG,
		.mask = SPEAR320S_PMX_PL_34_MASK | SPEAR320S_PMX_PL_35_MASK |
			SPEAR320S_PMX_PL_36_MASK,
		.value = SPEAR320S_PMX_UART1_ENH_PL_34_VAL |
			SPEAR320S_PMX_UART1_ENH_PL_35_VAL |
			SPEAR320S_PMX_UART1_ENH_PL_36_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_40_49_REG,
		.mask = SPEAR320S_PMX_PL_43_MASK | SPEAR320S_PMX_PL_44_45_MASK,
		.value = SPEAR320S_PMX_UART1_ENH_PL_43_VAL |
			SPEAR320S_PMX_UART1_ENH_PL_44_45_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_MIX_PAD_REG,
		.mask = SPEAR320S_PMX_UART1_ENH_PORT_SEL_MASK,
		.value = SPEAR320S_PMX_UART1_ENH_PORT_44_45_34_36_VAL,
	},
};

static struct pmx_mux_reg pmx_uart1_modem_ext_80_85_mux[] = {
	{
		.address = SPEAR320S_IP_SEL_PAD_80_89_REG,
		.mask = SPEAR320S_PMX_PL_80_TO_85_MASK,
		.value = SPEAR320S_PMX_UART1_ENH_PL_80_TO_85_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_40_49_REG,
		.mask = SPEAR320S_PMX_PL_43_MASK | SPEAR320S_PMX_PL_44_45_MASK,
		.value = SPEAR320S_PMX_UART1_ENH_PL_43_VAL |
			SPEAR320S_PMX_UART1_ENH_PL_44_45_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_MIX_PAD_REG,
		.mask = SPEAR320S_PMX_UART1_ENH_PORT_SEL_MASK,
		.value = SPEAR320S_PMX_UART1_ENH_PORT_81_TO_85_VAL,
	},
};

#define pmx_uart1_modem_common_modes					\
	{								\
		.ids = AUTO_EXP_MODE,					\
		.mux_regs = pmx_uart1_modem_autoexp_mux,		\
		.mux_reg_cnt = ARRAY_SIZE(pmx_uart1_modem_autoexp_mux),	\
	}, {								\
		.ids = SMALL_PRINTERS_MODE,				\
		.mux_regs = pmx_uart1_modem_smallpri_mux,		\
		.mux_reg_cnt = ARRAY_SIZE(pmx_uart1_modem_smallpri_mux),\
	}

static struct pmx_dev_mode pmx_uart1_modem_modes[][3] = {
	{
		/* Select signals on pins 2-7 */
		pmx_uart1_modem_common_modes,
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_uart1_modem_ext_2_7_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_uart1_modem_ext_2_7_mux),
		},
	}, {
		/* Select signals on pins 31_36 */
		pmx_uart1_modem_common_modes,
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_uart1_modem_ext_31_36_mux,
			.mux_reg_cnt =
				ARRAY_SIZE(pmx_uart1_modem_ext_31_36_mux),
		},
	}, {
		/* Select signals on pins 34_45 */
		pmx_uart1_modem_common_modes,
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_uart1_modem_ext_34_45_mux,
			.mux_reg_cnt =
				ARRAY_SIZE(pmx_uart1_modem_ext_34_45_mux),
		},
	}, {
		/* Select signals on pins 80_85 */
		pmx_uart1_modem_common_modes,
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_uart1_modem_ext_80_85_mux,
			.mux_reg_cnt =
				ARRAY_SIZE(pmx_uart1_modem_ext_80_85_mux),
		},
	},
};

struct pmx_dev spear320_pmx_uart1_modem[] = {
	{
		.name = "uart1_modem-2-7",
		.modes = pmx_uart1_modem_modes[0],
		.mode_count = ARRAY_SIZE(pmx_uart1_modem_modes[0]),
	}, {
		.name = "uart1_modem-31-36",
		.modes = pmx_uart1_modem_modes[1],
		.mode_count = ARRAY_SIZE(pmx_uart1_modem_modes[1]),
	}, {
		.name = "uart1_modem-34-45",
		.modes = pmx_uart1_modem_modes[2],
		.mode_count = ARRAY_SIZE(pmx_uart1_modem_modes[2]),
	}, {
		.name = "uart1_modem-80-85",
		.modes = pmx_uart1_modem_modes[3],
		.mode_count = ARRAY_SIZE(pmx_uart1_modem_modes[3]),
	},
};

/* Pad multiplexing for UART2 device */
static struct pmx_mux_reg pmx_uart2_mux[] = {
	{
		.mask = PMX_FIRDA_MASK,
		.value = 0,
	},
};

static struct pmx_mux_reg pmx_uart2_ext_mux[] = {
	{
		.address = SPEAR320S_IP_SEL_PAD_0_9_REG,
		.mask = SPEAR320S_PMX_PL_0_1_MASK,
		.value = SPEAR320S_PMX_UART2_PL_0_1_VAL,
	},
};

static struct pmx_dev_mode pmx_uart2_modes[] = {
	{
		.ids = ALL_LEGACY_MODES | SPEAR320S_EXTENDED_MODE,
		.mux_regs = pmx_uart2_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_uart2_mux),
	}, {
		.ids = SPEAR320S_EXTENDED_MODE,
		.mux_regs = pmx_uart2_ext_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_uart2_ext_mux),
	},
};

struct pmx_dev spear320_pmx_uart2 = {
	.name = "uart2",
	.modes = pmx_uart2_modes,
	.mode_count = ARRAY_SIZE(pmx_uart2_modes),
};

/* Pad multiplexing for uart3 device */
static struct pmx_mux_reg pmx_uart3_ext_8_9_mux[] = {
	{
		.mask = PMX_SSP_MASK,
		.value = 0,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_0_9_REG,
		.mask = SPEAR320S_PMX_PL_8_9_MASK,
		.value = SPEAR320S_PMX_UART3_PL_8_9_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_MIX_PAD_REG,
		.mask = SPEAR320S_PMX_UART3_PORT_SEL_MASK,
		.value = SPEAR320S_PMX_UART3_PORT_8_VAL,
	},
};

static struct pmx_mux_reg pmx_uart3_ext_15_16_mux[] = {
	{
		.mask = PMX_MII_MASK,
		.value = 0,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_10_19_REG,
		.mask = SPEAR320S_PMX_PL_15_16_MASK,
		.value = SPEAR320S_PMX_UART3_PL_15_16_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_MIX_PAD_REG,
		.mask = SPEAR320S_PMX_UART3_PORT_SEL_MASK,
		.value = SPEAR320S_PMX_UART3_PORT_15_VAL,
	},
};

static struct pmx_mux_reg pmx_uart3_ext_41_42_mux[] = {
	{
		.mask = PMX_UART0_MODEM_MASK,
		.value = 0,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_40_49_REG,
		.mask = SPEAR320S_PMX_PL_41_42_MASK,
		.value = SPEAR320S_PMX_UART3_PL_41_42_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_MIX_PAD_REG,
		.mask = SPEAR320S_PMX_UART3_PORT_SEL_MASK,
		.value = SPEAR320S_PMX_UART3_PORT_41_VAL,
	},
};

static struct pmx_mux_reg pmx_uart3_ext_52_53_mux[] = {
	{
		.address = SPEAR320S_IP_SEL_PAD_50_59_REG,
		.mask = SPEAR320S_PMX_PL_52_53_MASK,
		.value = SPEAR320S_PMX_UART3_PL_52_53_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_MIX_PAD_REG,
		.mask = SPEAR320S_PMX_UART3_PORT_SEL_MASK,
		.value = SPEAR320S_PMX_UART3_PORT_52_VAL,
	},
};

static struct pmx_mux_reg pmx_uart3_ext_73_74_mux[] = {
	{
		.address = SPEAR320S_IP_SEL_PAD_70_79_REG,
		.mask = SPEAR320S_PMX_PL_73_MASK | SPEAR320S_PMX_PL_74_MASK,
		.value = SPEAR320S_PMX_UART3_PL_73_VAL |
			SPEAR320S_PMX_UART3_PL_74_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_MIX_PAD_REG,
		.mask = SPEAR320S_PMX_UART3_PORT_SEL_MASK,
		.value = SPEAR320S_PMX_UART3_PORT_73_VAL,
	},
};

static struct pmx_mux_reg pmx_uart3_ext_94_95_mux[] = {
	{
		.address = SPEAR320S_IP_SEL_PAD_90_99_REG,
		.mask = SPEAR320S_PMX_PL_94_95_MASK,
		.value = SPEAR320S_PMX_UART3_PL_94_95_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_MIX_PAD_REG,
		.mask = SPEAR320S_PMX_UART3_PORT_SEL_MASK,
		.value = SPEAR320S_PMX_UART3_PORT_94_VAL,
	},
};

static struct pmx_mux_reg pmx_uart3_ext_98_99_mux[] = {
	{
		.address = SPEAR320S_IP_SEL_PAD_90_99_REG,
		.mask = SPEAR320S_PMX_PL_98_MASK | SPEAR320S_PMX_PL_99_MASK,
		.value = SPEAR320S_PMX_UART3_PL_98_VAL |
			SPEAR320S_PMX_UART3_PL_99_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_MIX_PAD_REG,
		.mask = SPEAR320S_PMX_UART3_PORT_SEL_MASK,
		.value = SPEAR320S_PMX_UART3_PORT_99_VAL,
	},
};

static struct pmx_dev_mode pmx_uart3_modes[][1] = {
	{
		/* Select signals on pins 8_9 */
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_uart3_ext_8_9_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_uart3_ext_8_9_mux),
		},
	}, {
		/* Select signals on pins 15_16 */
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_uart3_ext_15_16_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_uart3_ext_15_16_mux),
		},
	}, {
		/* Select signals on pins 41_42 */
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_uart3_ext_41_42_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_uart3_ext_41_42_mux),
		},
	}, {
		/* Select signals on pins 52_53 */
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_uart3_ext_52_53_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_uart3_ext_52_53_mux),
		},
	}, {
		/* Select signals on pins 73_74 */
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_uart3_ext_73_74_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_uart3_ext_73_74_mux),
		},
	}, {
		/* Select signals on pins 94_95 */
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_uart3_ext_94_95_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_uart3_ext_94_95_mux),
		},
	}, {
		/* Select signals on pins 98_99_ */
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_uart3_ext_98_99_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_uart3_ext_98_99_mux),
		},
	},
};

struct pmx_dev spear320s_pmx_uart3[] = {
	{
		.name = "uart3-8_9",
		.modes = pmx_uart3_modes[0],
		.mode_count = ARRAY_SIZE(pmx_uart3_modes[0]),
	}, {
		.name = "uart3-15_16",
		.modes = pmx_uart3_modes[1],
		.mode_count = ARRAY_SIZE(pmx_uart3_modes[1]),
	}, {
		.name = "uart3-41_42",
		.modes = pmx_uart3_modes[2],
		.mode_count = ARRAY_SIZE(pmx_uart3_modes[2]),
	}, {
		.name = "uart3-52_53",
		.modes = pmx_uart3_modes[3],
		.mode_count = ARRAY_SIZE(pmx_uart3_modes[3]),
	}, {
		.name = "uart3-73_74",
		.modes = pmx_uart3_modes[4],
		.mode_count = ARRAY_SIZE(pmx_uart3_modes[4]),
	}, {
		.name = "uart3-94_95",
		.modes = pmx_uart3_modes[4],
		.mode_count = ARRAY_SIZE(pmx_uart3_modes[4]),
	}, {
		.name = "uart3-98_99_",
		.modes = pmx_uart3_modes[4],
		.mode_count = ARRAY_SIZE(pmx_uart3_modes[4]),
	},
};

/* Pad multiplexing for uart4 device */
static struct pmx_mux_reg pmx_uart4_ext_6_7_mux[] = {
	{
		.mask = PMX_SSP_MASK,
		.value = 0,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_0_9_REG,
		.mask = SPEAR320S_PMX_PL_6_7_MASK,
		.value = SPEAR320S_PMX_UART4_PL_6_7_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_MIX_PAD_REG,
		.mask = SPEAR320S_PMX_UART4_PORT_SEL_MASK,
		.value = SPEAR320S_PMX_UART4_PORT_6_VAL,
	},
};

static struct pmx_mux_reg pmx_uart4_ext_13_14_mux[] = {
	{
		.mask = PMX_MII_MASK,
		.value = 0,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_10_19_REG,
		.mask = SPEAR320S_PMX_PL_13_14_MASK,
		.value = SPEAR320S_PMX_UART4_PL_13_14_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_MIX_PAD_REG,
		.mask = SPEAR320S_PMX_UART4_PORT_SEL_MASK,
		.value = SPEAR320S_PMX_UART4_PORT_13_VAL,
	},
};

static struct pmx_mux_reg pmx_uart4_ext_39_40_mux[] = {
	{
		.mask = PMX_UART0_MODEM_MASK,
		.value = 0,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_30_39_REG,
		.mask = SPEAR320S_PMX_PL_39_MASK,
		.value = SPEAR320S_PMX_UART4_PL_39_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_40_49_REG,
		.mask = SPEAR320S_PMX_PL_40_MASK,
		.value = SPEAR320S_PMX_UART4_PL_40_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_MIX_PAD_REG,
		.mask = SPEAR320S_PMX_UART4_PORT_SEL_MASK,
		.value = SPEAR320S_PMX_UART4_PORT_39_VAL,
	},
};

static struct pmx_mux_reg pmx_uart4_ext_71_72_mux[] = {
	{
		.address = SPEAR320S_IP_SEL_PAD_70_79_REG,
		.mask = SPEAR320S_PMX_PL_71_72_MASK,
		.value = SPEAR320S_PMX_UART4_PL_71_72_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_MIX_PAD_REG,
		.mask = SPEAR320S_PMX_UART4_PORT_SEL_MASK,
		.value = SPEAR320S_PMX_UART4_PORT_71_VAL,
	},
};

static struct pmx_mux_reg pmx_uart4_ext_92_93_mux[] = {
	{
		.address = SPEAR320S_IP_SEL_PAD_90_99_REG,
		.mask = SPEAR320S_PMX_PL_92_93_MASK,
		.value = SPEAR320S_PMX_UART4_PL_92_93_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_MIX_PAD_REG,
		.mask = SPEAR320S_PMX_UART4_PORT_SEL_MASK,
		.value = SPEAR320S_PMX_UART4_PORT_92_VAL,
	},
};

static struct pmx_mux_reg pmx_uart4_ext_100_101_mux[] = {
	{
		.address = SPEAR320S_IP_SEL_MIX_PAD_REG,
		.mask = SPEAR320S_PMX_PL_100_101_MASK |
			SPEAR320S_PMX_UART4_PORT_SEL_MASK,
		.value = SPEAR320S_PMX_UART4_PL_100_101_VAL |
			SPEAR320S_PMX_UART4_PORT_101_VAL,
	},
};

static struct pmx_dev_mode pmx_uart4_modes[][1] = {
	{
		/* Select signals on pins 6_7 */
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_uart4_ext_6_7_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_uart4_ext_6_7_mux),
		},
	}, {
		/* Select signals on pins 13_14 */
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_uart4_ext_13_14_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_uart4_ext_13_14_mux),
		},
	}, {
		/* Select signals on pins 39_40 */
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_uart4_ext_39_40_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_uart4_ext_39_40_mux),
		},
	}, {
		/* Select signals on pins 71_72 */
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_uart4_ext_71_72_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_uart4_ext_71_72_mux),
		},
	}, {
		/* Select signals on pins 92_93 */
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_uart4_ext_92_93_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_uart4_ext_92_93_mux),
		},
	}, {
		/* Select signals on pins 100_101_ */
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_uart4_ext_100_101_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_uart4_ext_100_101_mux),
		},
	},
};

struct pmx_dev spear320s_pmx_uart4[] = {
	{
		.name = "uart4-6_7",
		.modes = pmx_uart4_modes[0],
		.mode_count = ARRAY_SIZE(pmx_uart4_modes[0]),
	}, {
		.name = "uart4-13_14",
		.modes = pmx_uart4_modes[1],
		.mode_count = ARRAY_SIZE(pmx_uart4_modes[1]),
	}, {
		.name = "uart4-39_40",
		.modes = pmx_uart4_modes[2],
		.mode_count = ARRAY_SIZE(pmx_uart4_modes[2]),
	}, {
		.name = "uart4-71_72",
		.modes = pmx_uart4_modes[4],
		.mode_count = ARRAY_SIZE(pmx_uart4_modes[4]),
	}, {
		.name = "uart4-92_93",
		.modes = pmx_uart4_modes[4],
		.mode_count = ARRAY_SIZE(pmx_uart4_modes[4]),
	}, {
		.name = "uart4-100_101_",
		.modes = pmx_uart4_modes[4],
		.mode_count = ARRAY_SIZE(pmx_uart4_modes[4]),
	},
};

/* Pad multiplexing for uart5 device */
static struct pmx_mux_reg pmx_uart5_ext_4_5_mux[] = {
	{
		.mask = PMX_I2C_MASK,
		.value = 0,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_0_9_REG,
		.mask = SPEAR320S_PMX_PL_4_5_MASK,
		.value = SPEAR320S_PMX_UART5_PL_4_5_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_MIX_PAD_REG,
		.mask = SPEAR320S_PMX_UART5_PORT_SEL_MASK,
		.value = SPEAR320S_PMX_UART5_PORT_4_VAL,
	},
};

static struct pmx_mux_reg pmx_uart5_ext_37_38_mux[] = {
	{
		.mask = PMX_UART0_MODEM_MASK,
		.value = 0,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_30_39_REG,
		.mask = SPEAR320S_PMX_PL_37_38_MASK,
		.value = SPEAR320S_PMX_UART5_PL_37_38_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_MIX_PAD_REG,
		.mask = SPEAR320S_PMX_UART5_PORT_SEL_MASK,
		.value = SPEAR320S_PMX_UART5_PORT_37_VAL,
	},
};

static struct pmx_mux_reg pmx_uart5_ext_69_70_mux[] = {
	{
		.address = SPEAR320S_IP_SEL_PAD_60_69_REG,
		.mask = SPEAR320S_PMX_PL_69_MASK,
		.value = SPEAR320S_PMX_UART5_PL_69_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_70_79_REG,
		.mask = SPEAR320S_PMX_PL_70_MASK,
		.value = SPEAR320S_PMX_UART5_PL_70_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_MIX_PAD_REG,
		.mask = SPEAR320S_PMX_UART5_PORT_SEL_MASK,
		.value = SPEAR320S_PMX_UART5_PORT_69_VAL,
	},
};

static struct pmx_mux_reg pmx_uart5_ext_90_91_mux[] = {
	{
		.address = SPEAR320S_IP_SEL_PAD_90_99_REG,
		.mask = SPEAR320S_PMX_PL_90_91_MASK,
		.value = SPEAR320S_PMX_UART5_PL_90_91_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_MIX_PAD_REG,
		.mask = SPEAR320S_PMX_UART5_PORT_SEL_MASK,
		.value = SPEAR320S_PMX_UART5_PORT_90_VAL,
	},
};

static struct pmx_dev_mode pmx_uart5_modes[][1] = {
	{
		/* Select signals on pins 4_5 */
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_uart5_ext_4_5_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_uart5_ext_4_5_mux),
		},
	}, {
		/* Select signals on pins 37_38 */
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_uart5_ext_37_38_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_uart5_ext_37_38_mux),
		},
	}, {
		/* Select signals on pins 69_70 */
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_uart5_ext_69_70_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_uart5_ext_69_70_mux),
		},
	}, {
		/* Select signals on pins 90_91 */
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_uart5_ext_90_91_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_uart5_ext_90_91_mux),
		},
	},
};

struct pmx_dev spear320s_pmx_uart5[] = {
	{
		.name = "uart5-4_5",
		.modes = pmx_uart5_modes[0],
		.mode_count = ARRAY_SIZE(pmx_uart5_modes[0]),
	}, {
		.name = "uart5-37_38",
		.modes = pmx_uart5_modes[2],
		.mode_count = ARRAY_SIZE(pmx_uart5_modes[2]),
	}, {
		.name = "uart5-69_70",
		.modes = pmx_uart5_modes[4],
		.mode_count = ARRAY_SIZE(pmx_uart5_modes[4]),
	}, {
		.name = "uart5-90_91",
		.modes = pmx_uart5_modes[4],
		.mode_count = ARRAY_SIZE(pmx_uart5_modes[4]),
	},
};

/* Pad multiplexing for uart6 device */
static struct pmx_mux_reg pmx_uart6_ext_2_3_mux[] = {
	{
		.mask = PMX_UART0_MASK,
		.value = 0,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_0_9_REG,
		.mask = SPEAR320S_PMX_PL_2_3_MASK,
		.value = SPEAR320S_PMX_UART6_PL_2_3_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_MIX_PAD_REG,
		.mask = SPEAR320S_PMX_UART6_PORT_SEL_MASK,
		.value = SPEAR320S_PMX_UART6_PORT_2_VAL,
	},
};

static struct pmx_mux_reg pmx_uart6_ext_88_89_mux[] = {
	{
		.address = SPEAR320S_IP_SEL_PAD_80_89_REG,
		.mask = SPEAR320S_PMX_PL_88_89_MASK,
		.value = SPEAR320S_PMX_UART6_PL_88_89_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_MIX_PAD_REG,
		.mask = SPEAR320S_PMX_UART6_PORT_SEL_MASK,
		.value = SPEAR320S_PMX_UART6_PORT_88_VAL,
	},
};

static struct pmx_dev_mode pmx_uart6_modes[][1] = {
	{
		/* Select signals on pins 2_3 */
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_uart6_ext_2_3_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_uart6_ext_2_3_mux),
		},
	}, {
		/* Select signals on pins 88_89 */
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_uart6_ext_88_89_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_uart6_ext_88_89_mux),
		},
	},
};

struct pmx_dev spear320s_pmx_uart6[] = {
	{
		.name = "uart6-2_3",
		.modes = pmx_uart6_modes[0],
		.mode_count = ARRAY_SIZE(pmx_uart6_modes[0]),
	}, {
		.name = "uart6-88_89",
		.modes = pmx_uart6_modes[4],
		.mode_count = ARRAY_SIZE(pmx_uart6_modes[4]),
	},
};

/* UART - RS485 pmx */
static struct pmx_mux_reg pmx_rs485_mux[] = {
	{
		.address = SPEAR320S_IP_SEL_PAD_70_79_REG,
		.mask = SPEAR320S_PMX_PL_77_78_79_MASK,
		.value = SPEAR320S_PMX_RS485_PL_77_78_79_VAL,
	},
};

static struct pmx_dev_mode pmx_rs485_modes[] = {
	{
		.ids = SPEAR320S_EXTENDED_MODE,
		.mux_regs = pmx_rs485_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_rs485_mux),
	},
};

struct pmx_dev spear320s_pmx_rs485 = {
	.name = "rs485",
	.modes = pmx_rs485_modes,
	.mode_count = ARRAY_SIZE(pmx_rs485_modes),
};

/* Pad multiplexing for Touchscreen device */
static struct pmx_mux_reg pmx_touchscreen_mux[] = {
	{
		.mask = PMX_SSP_CS_MASK,
		.value = 0,
	},
};

static struct pmx_mux_reg pmx_touchscreen_ext_mux[] = {
	{
		.mask = PMX_I2C_MASK,
		.value = 0,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_0_9_REG,
		.mask = SPEAR320S_PMX_PL_5_MASK,
		.value = SPEAR320S_PMX_TOUCH_Y_PL_5_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_30_39_REG,
		.mask = SPEAR320S_PMX_PL_36_MASK,
		.value = SPEAR320S_PMX_TOUCH_X_PL_36_VAL,
	},
};

static struct pmx_dev_mode pmx_touchscreen_modes[] = {
	{
		.ids = AUTO_NET_SMII_MODE | SPEAR320S_EXTENDED_MODE,
		.mux_regs = pmx_touchscreen_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_touchscreen_mux),
	}, {
		.ids = SPEAR320S_EXTENDED_MODE,
		.mux_regs = pmx_touchscreen_ext_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_touchscreen_ext_mux),
	},
};

struct pmx_dev spear320_pmx_touchscreen = {
	.name = "touchscreen",
	.modes = pmx_touchscreen_modes,
	.mode_count = ARRAY_SIZE(pmx_touchscreen_modes),
};

/* Pad multiplexing for CAN device */
static struct pmx_mux_reg pmx_can0_mux[] = {
	{
		.mask = PMX_GPIO_PIN4_MASK | PMX_GPIO_PIN5_MASK,
		.value = 0,
	},
};

static struct pmx_mux_reg pmx_can0_ext_mux[] = {
	{
		.address = SPEAR320S_IP_SEL_PAD_30_39_REG,
		.mask = SPEAR320S_PMX_PL_32_33_MASK,
		.value = SPEAR320S_PMX_CAN0_PL_32_33_VAL,
	},
};

static struct pmx_dev_mode pmx_can0_modes[] = {
	{
		.ids = AUTO_NET_SMII_MODE | AUTO_NET_MII_MODE | AUTO_EXP_MODE |
			SPEAR320S_EXTENDED_MODE,
		.mux_regs = pmx_can0_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_can0_mux),
	}, {
		.ids = SPEAR320S_EXTENDED_MODE,
		.mux_regs = pmx_can0_ext_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_can0_ext_mux),
	},
};

struct pmx_dev spear320_pmx_can0 = {
	.name = "can0",
	.modes = pmx_can0_modes,
	.mode_count = ARRAY_SIZE(pmx_can0_modes),
};

static struct pmx_mux_reg pmx_can1_mux[] = {
	{
		.mask = PMX_GPIO_PIN2_MASK | PMX_GPIO_PIN3_MASK,
		.value = 0,
	},
};

static struct pmx_mux_reg pmx_can1_ext_mux[] = {
	{
		.address = SPEAR320S_IP_SEL_PAD_30_39_REG,
		.mask = SPEAR320S_PMX_PL_30_31_MASK,
		.value = SPEAR320S_PMX_CAN1_PL_30_31_VAL,
	},
};

static struct pmx_dev_mode pmx_can1_modes[] = {
	{
		.ids = AUTO_NET_SMII_MODE | AUTO_NET_MII_MODE | AUTO_EXP_MODE |
			SPEAR320S_EXTENDED_MODE,
		.mux_regs = pmx_can1_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_can1_mux),
	}, {
		.ids = SPEAR320S_EXTENDED_MODE,
		.mux_regs = pmx_can1_ext_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_can1_ext_mux),
	},
};

struct pmx_dev spear320_pmx_can1 = {
	.name = "can1",
	.modes = pmx_can1_modes,
	.mode_count = ARRAY_SIZE(pmx_can1_modes),
};

/* Pad multiplexing for SDHCI LED device */
static struct pmx_mux_reg pmx_sdhci_led_mux[] = {
	{
		.mask = PMX_SSP_CS_MASK,
		.value = 0,
	},
};

static struct pmx_mux_reg pmx_sdhci_led_ext_mux[] = {
	{
		.address = SPEAR320S_IP_SEL_PAD_30_39_REG,
		.mask = SPEAR320S_PMX_PL_34_MASK,
		.value = SPEAR320S_PMX_PWM2_PL_34_VAL,
	},
};

static struct pmx_dev_mode pmx_sdhci_led_modes[] = {
	{
		.ids = AUTO_NET_SMII_MODE | AUTO_NET_MII_MODE |
			SPEAR320S_EXTENDED_MODE,
		.mux_regs = pmx_sdhci_led_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_sdhci_led_mux),
	}, {
		.ids = SPEAR320S_EXTENDED_MODE,
		.mux_regs = pmx_sdhci_led_ext_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_sdhci_led_ext_mux),
	},
};

struct pmx_dev spear320_pmx_sdhci_led = {
	.name = "sdhci_led",
	.modes = pmx_sdhci_led_modes,
	.mode_count = ARRAY_SIZE(pmx_sdhci_led_modes),
};

/* Pad multiplexing for PWM0_1 device */
static struct pmx_mux_reg pmx_pwm0_1_net_mux[] = {
	{
		.mask = PMX_UART0_MODEM_MASK,
		.value = 0,
	},
};

static struct pmx_mux_reg pmx_pwm0_1_autoexpsmallpri_mux[] = {
	{
		.mask = PMX_MII_MASK,
		.value = 0,
	},
};

static struct pmx_mux_reg pmx_pwm0_1_pin_8_9_mux[] = {
	{
		.mask = PMX_SSP_MASK,
		.value = 0,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_0_9_REG,
		.mask = SPEAR320S_PMX_PL_8_9_MASK,
		.value = SPEAR320S_PMX_PWM_0_1_PL_8_9_VAL,
	},
};

static struct pmx_mux_reg pmx_pwm0_1_pin_14_15_mux[] = {
	{
		.mask = PMX_MII_MASK,
		.value = 0,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_10_19_REG,
		.mask = SPEAR320S_PMX_PL_14_MASK | SPEAR320S_PMX_PL_15_MASK,
		.value = SPEAR320S_PMX_PWM1_PL_14_VAL |
			SPEAR320S_PMX_PWM0_PL_15_VAL,
	},
};

static struct pmx_mux_reg pmx_pwm0_1_pin_30_31_mux[] = {
	{
		.mask = PMX_GPIO_PIN2_MASK | PMX_GPIO_PIN3_MASK,
		.value = 0,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_30_39_REG,
		.mask = SPEAR320S_PMX_PL_30_MASK | SPEAR320S_PMX_PL_31_MASK,
		.value = SPEAR320S_PMX_PWM1_EXT_PL_30_VAL |
			SPEAR320S_PMX_PWM0_EXT_PL_31_VAL,
	},
};

static struct pmx_mux_reg pmx_pwm0_1_pin_37_38_mux[] = {
	{
		.mask = PMX_UART0_MODEM_MASK,
		.value = 0,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_30_39_REG,
		.mask = SPEAR320S_PMX_PL_37_38_MASK,
		.value = SPEAR320S_PMX_PWM0_1_PL_37_38_VAL,
	},
};

static struct pmx_mux_reg pmx_pwm0_1_pin_42_43_mux[] = {
	{
		.mask = PMX_UART0_MODEM_MASK | PMX_TIMER_1_2_MASK ,
		.value = 0,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_40_49_REG,
		.mask = SPEAR320S_PMX_PL_42_MASK | SPEAR320S_PMX_PL_43_MASK,
		.value = SPEAR320S_PMX_PWM1_PL_42_VAL |
			SPEAR320S_PMX_PWM0_PL_43_VAL,
	},
};

static struct pmx_mux_reg pmx_pwm0_1_pin_59_60_mux[] = {
	{
		.address = SPEAR320S_IP_SEL_PAD_50_59_REG,
		.mask = SPEAR320S_PMX_PL_59_MASK,
		.value = SPEAR320S_PMX_PWM1_PL_59_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_60_69_REG,
		.mask = SPEAR320S_PMX_PL_60_MASK,
		.value = SPEAR320S_PMX_PWM0_PL_60_VAL,
	},
};

static struct pmx_mux_reg pmx_pwm0_1_pin_88_89_mux[] = {
	{
		.address = SPEAR320S_IP_SEL_PAD_80_89_REG,
		.mask = SPEAR320S_PMX_PL_88_89_MASK,
		.value = SPEAR320S_PMX_PWM0_1_PL_88_89_VAL,
	},
};

#define pmx_pwm0_1_common_modes					\
	{							\
		.ids = AUTO_NET_SMII_MODE | AUTO_NET_MII_MODE,	\
		.mux_regs = pmx_pwm0_1_net_mux,			\
		.mux_reg_cnt = ARRAY_SIZE(pmx_pwm0_1_net_mux),	\
	}, {							\
		.ids = AUTO_EXP_MODE | SMALL_PRINTERS_MODE,	\
		.mux_regs = pmx_pwm0_1_autoexpsmallpri_mux,	\
		.mux_reg_cnt = ARRAY_SIZE(pmx_pwm0_1_autoexpsmallpri_mux), \
	}

static struct pmx_dev_mode pmx_pwm0_1_modes[][3] = {
	{
		/* pin_8_9 */
		pmx_pwm0_1_common_modes,
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_pwm0_1_pin_8_9_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_pwm0_1_pin_8_9_mux),
		},
	}, {
		/* pin_14_15 */
		pmx_pwm0_1_common_modes,
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_pwm0_1_pin_14_15_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_pwm0_1_pin_14_15_mux),
		},
	}, {
		/* pin_30_31 */
		pmx_pwm0_1_common_modes,
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_pwm0_1_pin_30_31_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_pwm0_1_pin_30_31_mux),
		},
	}, {
		/* pin_37_38 */
		pmx_pwm0_1_common_modes,
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_pwm0_1_pin_37_38_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_pwm0_1_pin_37_38_mux),
		},
	}, {
		/* pin_42_43 */
		pmx_pwm0_1_common_modes,
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_pwm0_1_pin_42_43_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_pwm0_1_pin_42_43_mux),
		},
	}, {
		/* pin_59_60 */
		pmx_pwm0_1_common_modes,
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_pwm0_1_pin_59_60_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_pwm0_1_pin_59_60_mux),
		},
	}, {
		/* pin_88_89 */
		pmx_pwm0_1_common_modes,
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_pwm0_1_pin_88_89_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_pwm0_1_pin_88_89_mux),
		},
	},
};

struct pmx_dev spear320_pmx_pwm0_1[] = {
	{
		.name = "pwm0_1_pin_8_9",
		.modes = pmx_pwm0_1_modes[0],
		.mode_count = ARRAY_SIZE(pmx_pwm0_1_modes[0]),
	}, {
		.name = "pwm0_1_pin_14_15",
		.modes = pmx_pwm0_1_modes[1],
		.mode_count = ARRAY_SIZE(pmx_pwm0_1_modes[1]),
	}, {
		.name = "pwm0_1_pin_30_31",
		.modes = pmx_pwm0_1_modes[2],
		.mode_count = ARRAY_SIZE(pmx_pwm0_1_modes[2]),
	}, {
		.name = "pwm0_1_pin_37_38",
		.modes = pmx_pwm0_1_modes[3],
		.mode_count = ARRAY_SIZE(pmx_pwm0_1_modes[3]),
	}, {
		.name = "pwm0_1_pin_42_43",
		.modes = pmx_pwm0_1_modes[4],
		.mode_count = ARRAY_SIZE(pmx_pwm0_1_modes[4]),
	}, {
		.name = "pwm0_1_pin_59_60",
		.modes = pmx_pwm0_1_modes[5],
		.mode_count = ARRAY_SIZE(pmx_pwm0_1_modes[5]),
	}, {
		.name = "pwm0_1_pin_88_89",
		.modes = pmx_pwm0_1_modes[6],
		.mode_count = ARRAY_SIZE(pmx_pwm0_1_modes[6]),
	},
};

/* Pad multiplexing for PWM2 device */
static struct pmx_mux_reg pmx_pwm2_net_mux[] = {
	{
		.mask = PMX_SSP_CS_MASK,
		.value = 0,
	},
};

static struct pmx_mux_reg pmx_pwm2_autoexpsmallpri_mux[] = {
	{
		.mask = PMX_MII_MASK,
		.value = 0,
	},
};

static struct pmx_mux_reg pmx_pwm2_pin_7_mux[] = {
	{
		.mask = PMX_SSP_MASK,
		.value = 0,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_0_9_REG,
		.mask = SPEAR320S_PMX_PL_7_MASK,
		.value = SPEAR320S_PMX_PWM_2_PL_7_VAL,
	},
};

static struct pmx_mux_reg pmx_pwm2_pin_13_mux[] = {
	{
		.mask = PMX_MII_MASK,
		.value = 0,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_10_19_REG,
		.mask = SPEAR320S_PMX_PL_13_MASK,
		.value = SPEAR320S_PMX_PWM2_PL_13_VAL,
	},
};

static struct pmx_mux_reg pmx_pwm2_pin_29_mux[] = {
	{
		.mask = PMX_GPIO_PIN1_MASK,
		.value = 0,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_20_29_REG,
		.mask = SPEAR320S_PMX_PL_29_MASK,
		.value = SPEAR320S_PMX_PWM_2_PL_29_VAL,
	},
};

static struct pmx_mux_reg pmx_pwm2_pin_34_mux[] = {
	{
		.mask = PMX_SSP_CS_MASK,
		.value = 0,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_30_39_REG,
		.mask = SPEAR320S_PMX_PL_34_MASK,
		.value = SPEAR320S_PMX_PWM2_PL_34_VAL,
	},
};

static struct pmx_mux_reg pmx_pwm2_pin_41_mux[] = {
	{
		.mask = PMX_UART0_MODEM_MASK,
		.value = 0,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_40_49_REG,
		.mask = SPEAR320S_PMX_PL_41_MASK,
		.value = SPEAR320S_PMX_PWM2_PL_41_VAL,
	},
};

static struct pmx_mux_reg pmx_pwm2_pin_58_mux[] = {
	{
		.address = SPEAR320S_IP_SEL_PAD_50_59_REG,
		.mask = SPEAR320S_PMX_PL_58_MASK,
		.value = SPEAR320S_PMX_PWM2_PL_58_VAL,
	},
};

static struct pmx_mux_reg pmx_pwm2_pin_87_mux[] = {
	{
		.address = SPEAR320S_IP_SEL_PAD_80_89_REG,
		.mask = SPEAR320S_PMX_PL_87_MASK,
		.value = SPEAR320S_PMX_PWM2_PL_87_VAL,
	},
};

#define pmx_pwm2_common_modes						\
	{								\
		.ids = AUTO_NET_SMII_MODE | AUTO_NET_MII_MODE,		\
		.mux_regs = pmx_pwm2_net_mux,				\
		.mux_reg_cnt = ARRAY_SIZE(pmx_pwm2_net_mux),		\
	}, {								\
		.ids = AUTO_EXP_MODE | SMALL_PRINTERS_MODE,		\
		.mux_regs = pmx_pwm2_autoexpsmallpri_mux,		\
		.mux_reg_cnt = ARRAY_SIZE(pmx_pwm2_autoexpsmallpri_mux),\
	}

static struct pmx_dev_mode pmx_pwm2_modes[][3] = {
	{
		/* pin_7 */
		pmx_pwm2_common_modes,
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_pwm2_pin_7_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_pwm2_pin_7_mux),
		},
	}, {
		/* pin_13 */
		pmx_pwm2_common_modes,
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_pwm2_pin_13_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_pwm2_pin_13_mux),
		},
	}, {
		/* pin_29 */
		pmx_pwm2_common_modes,
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_pwm2_pin_29_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_pwm2_pin_29_mux),
		},
	}, {
		/* pin_34 */
		pmx_pwm2_common_modes,
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_pwm2_pin_34_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_pwm2_pin_34_mux),
		},
	}, {
		/* pin_41 */
		pmx_pwm2_common_modes,
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_pwm2_pin_41_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_pwm2_pin_41_mux),
		},
	}, {
		/* pin_58 */
		pmx_pwm2_common_modes,
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_pwm2_pin_58_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_pwm2_pin_58_mux),
		},
	}, {
		/* pin_87 */
		pmx_pwm2_common_modes,
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_pwm2_pin_87_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_pwm2_pin_87_mux),
		},
	},
};

struct pmx_dev spear320_pmx_pwm2[] = {
	{
		.name = "pwm2_pin_7",
		.modes = pmx_pwm2_modes[0],
		.mode_count = ARRAY_SIZE(pmx_pwm2_modes[0]),
	}, {
		.name = "pwm2_pin_13",
		.modes = pmx_pwm2_modes[1],
		.mode_count = ARRAY_SIZE(pmx_pwm2_modes[1]),
	}, {
		.name = "pwm2_pin_29",
		.modes = pmx_pwm2_modes[2],
		.mode_count = ARRAY_SIZE(pmx_pwm2_modes[2]),
	}, {
		.name = "pwm2_pin_34",
		.modes = pmx_pwm2_modes[3],
		.mode_count = ARRAY_SIZE(pmx_pwm2_modes[3]),
	}, {
		.name = "pwm2_pin_41",
		.modes = pmx_pwm2_modes[4],
		.mode_count = ARRAY_SIZE(pmx_pwm2_modes[4]),
	}, {
		.name = "pwm2_pin_58",
		.modes = pmx_pwm2_modes[5],
		.mode_count = ARRAY_SIZE(pmx_pwm2_modes[5]),
	}, {
		.name = "pwm2_pin_87",
		.modes = pmx_pwm2_modes[6],
		.mode_count = ARRAY_SIZE(pmx_pwm2_modes[6]),
	},
};

/* Pad multiplexing for PWM3 device */
static struct pmx_mux_reg pmx_pwm3_mux[] = {
	{
		.mask = PMX_MII_MASK,
		.value = 0,
	},
};

static struct pmx_mux_reg pmx_pwm3_pin_6_mux[] = {
	{
		.mask = PMX_SSP_MASK,
		.value = 0,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_0_9_REG,
		.mask = SPEAR320S_PMX_PL_6_MASK,
		.value = SPEAR320S_PMX_PWM_3_PL_6_VAL,
	},
};

static struct pmx_mux_reg pmx_pwm3_pin_12_mux[] = {
	{
		.mask = PMX_MII_MASK,
		.value = 0,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_10_19_REG,
		.mask = SPEAR320S_PMX_PL_12_MASK,
		.value = SPEAR320S_PMX_PWM3_PL_12_VAL,
	},
};

static struct pmx_mux_reg pmx_pwm3_pin_28_mux[] = {
	{
		.mask = PMX_GPIO_PIN0_MASK,
		.value = 0,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_20_29_REG,
		.mask = SPEAR320S_PMX_PL_28_MASK,
		.value = SPEAR320S_PMX_PWM_3_PL_28_VAL,
	},
};

static struct pmx_mux_reg pmx_pwm3_pin_40_mux[] = {
	{
		.mask = PMX_UART0_MODEM_MASK,
		.value = 0,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_40_49_REG,
		.mask = SPEAR320S_PMX_PL_40_MASK,
		.value = SPEAR320S_PMX_PWM3_PL_40_VAL,
	},
};

static struct pmx_mux_reg pmx_pwm3_pin_57_mux[] = {
	{
		.address = SPEAR320S_IP_SEL_PAD_50_59_REG,
		.mask = SPEAR320S_PMX_PL_57_MASK,
		.value = SPEAR320S_PMX_PWM3_PL_57_VAL,
	},
};

static struct pmx_mux_reg pmx_pwm3_pin_86_mux[] = {
	{
		.address = SPEAR320S_IP_SEL_PAD_80_89_REG,
		.mask = SPEAR320S_PMX_PL_86_MASK,
		.value = SPEAR320S_PMX_PWM3_PL_86_VAL,
	},
};

#define pmx_pwm3_common_modes						\
	{								\
		.ids = AUTO_EXP_MODE | SMALL_PRINTERS_MODE |		\
			AUTO_NET_SMII_MODE,				\
		.mux_regs = pmx_pwm3_mux,				\
		.mux_reg_cnt = ARRAY_SIZE(pmx_pwm3_mux),		\
	}

static struct pmx_dev_mode pmx_pwm3_modes[][3] = {
	{
		/* pin_6 */
		pmx_pwm3_common_modes,
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_pwm3_pin_6_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_pwm3_pin_6_mux),
		},
	}, {
		/* pin_12 */
		pmx_pwm3_common_modes,
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_pwm3_pin_12_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_pwm3_pin_12_mux),
		},
	}, {
		/* pin_28 */
		pmx_pwm3_common_modes,
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_pwm3_pin_28_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_pwm3_pin_28_mux),
		},
	}, {
		/* pin_40 */
		pmx_pwm3_common_modes,
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_pwm3_pin_40_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_pwm3_pin_40_mux),
		},
	}, {
		/* pin_57 */
		pmx_pwm3_common_modes,
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_pwm3_pin_57_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_pwm3_pin_57_mux),
		},
	}, {
		/* pin_86 */
		pmx_pwm3_common_modes,
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_pwm3_pin_86_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_pwm3_pin_86_mux),
		},
	},
};

struct pmx_dev spear320_pmx_pwm3[] = {
	{
		.name = "pwm3_pin_6",
		.modes = pmx_pwm3_modes[0],
		.mode_count = ARRAY_SIZE(pmx_pwm3_modes[0]),
	}, {
		.name = "pwm3_pin_12",
		.modes = pmx_pwm3_modes[1],
		.mode_count = ARRAY_SIZE(pmx_pwm3_modes[1]),
	}, {
		.name = "pwm3_pin_28",
		.modes = pmx_pwm3_modes[2],
		.mode_count = ARRAY_SIZE(pmx_pwm3_modes[2]),
	}, {
		.name = "pwm3_pin_40",
		.modes = pmx_pwm3_modes[3],
		.mode_count = ARRAY_SIZE(pmx_pwm3_modes[3]),
	}, {
		.name = "pwm3_pin_57",
		.modes = pmx_pwm3_modes[4],
		.mode_count = ARRAY_SIZE(pmx_pwm3_modes[4]),
	}, {
		.name = "pwm3_pin_86",
		.modes = pmx_pwm3_modes[5],
		.mode_count = ARRAY_SIZE(pmx_pwm3_modes[5]),
	},
};

/* Pad multiplexing for SSP1 device */
static struct pmx_mux_reg pmx_ssp1_mux[] = {
	{
		.mask = PMX_MII_MASK,
		.value = 0,
	},
};

static struct pmx_mux_reg pmx_ssp1_ext_17_20_mux[] = {
	{
		.mask = PMX_MII_MASK,
		.value = 0,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_10_19_REG,
		.mask = SPEAR320S_PMX_PL_17_18_MASK | SPEAR320S_PMX_PL_19_MASK,
		.value = SPEAR320S_PMX_SSP1_PL_17_18_19_20_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_20_29_REG,
		.mask = SPEAR320S_PMX_PL_20_MASK,
		.value = SPEAR320S_PMX_SSP1_PL_17_18_19_20_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_MIX_PAD_REG,
		.mask = SPEAR320S_PMX_SSP1_PORT_SEL_MASK,
		.value = SPEAR320S_PMX_SSP1_PORT_17_TO_20_VAL,
	},
};

static struct pmx_mux_reg pmx_ssp1_ext_36_39_mux[] = {
	{
		.mask = PMX_UART0_MODEM_MASK | PMX_SSP_CS_MASK,
		.value = 0,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_30_39_REG,
		.mask = SPEAR320S_PMX_PL_36_MASK | SPEAR320S_PMX_PL_37_38_MASK |
			SPEAR320S_PMX_PL_39_MASK,
		.value = SPEAR320S_PMX_SSP1_PL_36_VAL |
			SPEAR320S_PMX_SSP1_PL_37_38_VAL |
			SPEAR320S_PMX_SSP1_PL_39_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_MIX_PAD_REG,
		.mask = SPEAR320S_PMX_SSP1_PORT_SEL_MASK,
		.value = SPEAR320S_PMX_SSP1_PORT_36_TO_39_VAL,
	},
};

static struct pmx_mux_reg pmx_ssp1_ext_48_51_mux[] = {
	{
		.mask = PMX_TIMER_1_2_MASK | PMX_TIMER_3_4_MASK,
		.value = 0,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_40_49_REG,
		.mask = SPEAR320S_PMX_PL_48_49_MASK,
		.value = SPEAR320S_PMX_SSP1_PL_48_49_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_50_59_REG,
		.mask = SPEAR320S_PMX_PL_50_51_MASK,
		.value = SPEAR320S_PMX_SSP1_PL_50_51_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_MIX_PAD_REG,
		.mask = SPEAR320S_PMX_SSP1_PORT_SEL_MASK,
		.value = SPEAR320S_PMX_SSP1_PORT_48_TO_51_VAL,
	},
};

static struct pmx_mux_reg pmx_ssp1_ext_65_68_mux[] = {
	{
		.address = SPEAR320S_IP_SEL_PAD_60_69_REG,
		.mask = SPEAR320S_PMX_PL_65_TO_68_MASK,
		.value = SPEAR320S_PMX_SSP1_PL_65_TO_68_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_MIX_PAD_REG,
		.mask = SPEAR320S_PMX_SSP1_PORT_SEL_MASK,
		.value = SPEAR320S_PMX_SSP1_PORT_65_TO_68_VAL,
	},
};

static struct pmx_mux_reg pmx_ssp1_ext_94_97_mux[] = {
	{
		.address = SPEAR320S_IP_SEL_PAD_90_99_REG,
		.mask = SPEAR320S_PMX_PL_94_95_MASK |
			SPEAR320S_PMX_PL_96_97_MASK,
		.value = SPEAR320S_PMX_SSP1_PL_94_95_VAL |
			SPEAR320S_PMX_SSP1_PL_96_97_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_MIX_PAD_REG,
		.mask = SPEAR320S_PMX_SSP1_PORT_SEL_MASK,
		.value = SPEAR320S_PMX_SSP1_PORT_94_TO_97_VAL,
	},
};

#define pmx_ssp1_common_modes						\
	{								\
		.ids = SMALL_PRINTERS_MODE | AUTO_NET_SMII_MODE,	\
		.mux_regs = pmx_ssp1_mux,				\
		.mux_reg_cnt = ARRAY_SIZE(pmx_ssp1_mux),		\
	}

static struct pmx_dev_mode pmx_ssp1_modes[][2] = {
	{
		/* Select signals on pins 17-20 */
		pmx_ssp1_common_modes,
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_ssp1_ext_17_20_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_ssp1_ext_17_20_mux),
		},
	}, {
		/* Select signals on pins 36-39 */
		pmx_ssp1_common_modes,
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_ssp1_ext_36_39_mux,
			.mux_reg_cnt =
				ARRAY_SIZE(pmx_ssp1_ext_36_39_mux),
		},
	}, {
		/* Select signals on pins 48-51 */
		pmx_ssp1_common_modes,
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_ssp1_ext_48_51_mux,
			.mux_reg_cnt =
				ARRAY_SIZE(pmx_ssp1_ext_48_51_mux),
		},
	}, {
		/* Select signals on pins 65-68 */
		pmx_ssp1_common_modes,
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_ssp1_ext_65_68_mux,
			.mux_reg_cnt =
				ARRAY_SIZE(pmx_ssp1_ext_65_68_mux),
		},
	}, {
		/* Select signals on pins 94-97 */
		pmx_ssp1_common_modes,
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_ssp1_ext_94_97_mux,
			.mux_reg_cnt =
				ARRAY_SIZE(pmx_ssp1_ext_94_97_mux),
		},
	},
};

struct pmx_dev spear320_pmx_ssp1[] = {
	{
		.name = "ssp1-17-20",
		.modes = pmx_ssp1_modes[0],
		.mode_count = ARRAY_SIZE(pmx_ssp1_modes[0]),
	}, {
		.name = "ssp1-36-39",
		.modes = pmx_ssp1_modes[1],
		.mode_count = ARRAY_SIZE(pmx_ssp1_modes[1]),
	}, {
		.name = "ssp1-48-51",
		.modes = pmx_ssp1_modes[2],
		.mode_count = ARRAY_SIZE(pmx_ssp1_modes[2]),
	}, {
		.name = "ssp1-65-68",
		.modes = pmx_ssp1_modes[3],
		.mode_count = ARRAY_SIZE(pmx_ssp1_modes[3]),
	}, {
		.name = "ssp1-94-97",
		.modes = pmx_ssp1_modes[4],
		.mode_count = ARRAY_SIZE(pmx_ssp1_modes[4]),
	},
};

/* Pad multiplexing for SSP2 device */
static struct pmx_mux_reg pmx_ssp2_mux[] = {
	{
		.mask = PMX_MII_MASK,
		.value = 0,
	},
};

static struct pmx_mux_reg pmx_ssp2_ext_13_16_mux[] = {
	{
		.mask = PMX_MII_MASK,
		.value = 0,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_10_19_REG,
		.mask = SPEAR320S_PMX_PL_13_14_MASK |
			SPEAR320S_PMX_PL_15_16_MASK,
		.value = SPEAR320S_PMX_SSP2_PL_13_14_15_16_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_MIX_PAD_REG,
		.mask = SPEAR320S_PMX_SSP2_PORT_SEL_MASK,
		.value = SPEAR320S_PMX_SSP2_PORT_13_TO_16_VAL,
	},
};

static struct pmx_mux_reg pmx_ssp2_ext_32_35_mux[] = {
	{
		.mask = PMX_SSP_CS_MASK | PMX_GPIO_PIN4_MASK |
			PMX_GPIO_PIN5_MASK,
		.value = 0,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_30_39_REG,
		.mask = SPEAR320S_PMX_PL_32_33_MASK | SPEAR320S_PMX_PL_34_MASK |
			SPEAR320S_PMX_PL_35_MASK,
		.value = SPEAR320S_PMX_SSP2_PL_32_33_VAL |
			SPEAR320S_PMX_SSP2_PL_34_VAL |
			SPEAR320S_PMX_SSP2_PL_35_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_MIX_PAD_REG,
		.mask = SPEAR320S_PMX_SSP2_PORT_SEL_MASK,
		.value = SPEAR320S_PMX_SSP2_PORT_32_TO_35_VAL,
	},
};

static struct pmx_mux_reg pmx_ssp2_ext_44_47_mux[] = {
	{
		.mask = PMX_TIMER_1_2_MASK | PMX_TIMER_3_4_MASK,
		.value = 0,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_40_49_REG,
		.mask = SPEAR320S_PMX_PL_44_45_MASK |
			SPEAR320S_PMX_PL_46_47_MASK,
		.value = SPEAR320S_PMX_SSP2_PL_44_45_VAL |
			SPEAR320S_PMX_SSP2_PL_46_47_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_MIX_PAD_REG,
		.mask = SPEAR320S_PMX_SSP2_PORT_SEL_MASK,
		.value = SPEAR320S_PMX_SSP2_PORT_44_TO_47_VAL,
	},
};

static struct pmx_mux_reg pmx_ssp2_ext_61_64_mux[] = {
	{
		.address = SPEAR320S_IP_SEL_PAD_60_69_REG,
		.mask = SPEAR320S_PMX_PL_61_TO_64_MASK,
		.value = SPEAR320S_PMX_SSP2_PL_61_TO_64_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_MIX_PAD_REG,
		.mask = SPEAR320S_PMX_SSP2_PORT_SEL_MASK,
		.value = SPEAR320S_PMX_SSP2_PORT_61_TO_64_VAL,
	},
};

static struct pmx_mux_reg pmx_ssp2_ext_90_93_mux[] = {
	{
		.address = SPEAR320S_IP_SEL_PAD_90_99_REG,
		.mask = SPEAR320S_PMX_PL_90_91_MASK |
			SPEAR320S_PMX_PL_92_93_MASK,
		.value = SPEAR320S_PMX_SSP2_PL_90_91_VAL |
			SPEAR320S_PMX_SSP2_PL_92_93_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_MIX_PAD_REG,
		.mask = SPEAR320S_PMX_SSP2_PORT_SEL_MASK,
		.value = SPEAR320S_PMX_SSP2_PORT_90_TO_93_VAL,
	},
};

#define pmx_ssp2_common_modes						\
	{								\
		.ids = AUTO_NET_SMII_MODE,				\
		.mux_regs = pmx_ssp2_mux,				\
		.mux_reg_cnt = ARRAY_SIZE(pmx_ssp2_mux),		\
	}

static struct pmx_dev_mode pmx_ssp2_modes[][2] = {
	{
		/* Select signals on pins 13-16 */
		pmx_ssp2_common_modes,
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_ssp2_ext_13_16_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_ssp2_ext_13_16_mux),
		},
	}, {
		/* Select signals on pins 32-35 */
		pmx_ssp2_common_modes,
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_ssp2_ext_32_35_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_ssp2_ext_32_35_mux),
		},
	}, {
		/* Select signals on pins 44-47 */
		pmx_ssp2_common_modes,
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_ssp2_ext_44_47_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_ssp2_ext_44_47_mux),
		},
	}, {
		/* Select signals on pins 61-64 */
		pmx_ssp2_common_modes,
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_ssp2_ext_61_64_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_ssp2_ext_61_64_mux),
		},
	}, {
		/* Select signals on pins 90-93 */
		pmx_ssp2_common_modes,
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_ssp2_ext_90_93_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_ssp2_ext_90_93_mux),
		},
	},
};

struct pmx_dev spear320_pmx_ssp2[] = {
	{
		.name = "ssp2-13-16",
		.modes = pmx_ssp2_modes[0],
		.mode_count = ARRAY_SIZE(pmx_ssp2_modes[0]),
	}, {
		.name = "ssp2-32-35",
		.modes = pmx_ssp2_modes[1],
		.mode_count = ARRAY_SIZE(pmx_ssp2_modes[1]),
	}, {
		.name = "ssp2-44-47",
		.modes = pmx_ssp2_modes[2],
		.mode_count = ARRAY_SIZE(pmx_ssp2_modes[2]),
	}, {
		.name = "ssp2-61-64",
		.modes = pmx_ssp2_modes[3],
		.mode_count = ARRAY_SIZE(pmx_ssp2_modes[3]),
	}, {
		.name = "ssp2-90-93",
		.modes = pmx_ssp2_modes[4],
		.mode_count = ARRAY_SIZE(pmx_ssp2_modes[4]),
	},
};

/* Pad multiplexing for cadence mii2 as mii device */
static struct pmx_mux_reg pmx_mii2_mux[] = {
	{
		.address = SPEAR320S_IP_SEL_PAD_80_89_REG,
		.mask = SPEAR320S_PMX_PL_80_TO_85_MASK |
			SPEAR320S_PMX_PL_86_87_MASK |
			SPEAR320S_PMX_PL_88_89_MASK,
		.value = SPEAR320S_PMX_MII2_PL_80_TO_85_VAL |
			SPEAR320S_PMX_MII2_PL_86_87_VAL |
			SPEAR320S_PMX_MII2_PL_88_89_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_90_99_REG,
		.mask = SPEAR320S_PMX_PL_90_91_MASK |
			SPEAR320S_PMX_PL_92_93_MASK |
			SPEAR320S_PMX_PL_94_95_MASK |
			SPEAR320S_PMX_PL_96_97_MASK,
		.value = SPEAR320S_PMX_MII2_PL_90_91_VAL |
			SPEAR320S_PMX_MII2_PL_92_93_VAL |
			SPEAR320S_PMX_MII2_PL_94_95_VAL |
			SPEAR320S_PMX_MII2_PL_96_97_VAL,
	}, {
		.address = SPEAR320S_EXT_CTRL_REG,
		.mask = (SPEAR320S_MAC_MODE_MASK << SPEAR320S_MAC2_MODE_SHIFT) |
			(SPEAR320S_MAC_MODE_MASK << SPEAR320S_MAC1_MODE_SHIFT) |
			SPEAR320S_MII_MDIO_MASK,
		.value = (SPEAR320S_MAC_MODE_MII << SPEAR320S_MAC2_MODE_SHIFT) |
			(SPEAR320S_MAC_MODE_MII << SPEAR320S_MAC1_MODE_SHIFT) |
			SPEAR320S_MII_MDIO_81_VAL,
	},
};

static struct pmx_dev_mode pmx_mii2_modes[] = {
	{
		.ids = SPEAR320S_EXTENDED_MODE,
		.mux_regs = pmx_mii2_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_mii2_mux),
	},
};

struct pmx_dev spear320s_pmx_mii2 = {
	.name = "mii2",
	.modes = pmx_mii2_modes,
	.mode_count = ARRAY_SIZE(pmx_mii2_modes),
};

/* Pad multiplexing for cadence mii 1_2 as smii or rmii device */
static struct pmx_mux_reg pmx_mii1_2_mux[] = {
	{
		.mask = PMX_MII_MASK,
		.value = 0,
	},
};

static struct pmx_mux_reg pmx_smii1_2_ext_mux[] = {
	{
		.address = SPEAR320S_IP_SEL_PAD_10_19_REG,
		.mask = SPEAR320S_PMX_PL_10_11_MASK,
		.value = SPEAR320S_PMX_SMII_PL_10_11_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_20_29_REG,
		.mask = SPEAR320S_PMX_PL_21_TO_27_MASK,
		.value = SPEAR320S_PMX_SMII_PL_21_TO_27_VAL,
	}, {
		.address = SPEAR320S_EXT_CTRL_REG,
		.mask = (SPEAR320S_MAC_MODE_MASK << SPEAR320S_MAC2_MODE_SHIFT) |
			(SPEAR320S_MAC_MODE_MASK << SPEAR320S_MAC1_MODE_SHIFT) |
			SPEAR320S_MII_MDIO_MASK,
		.value = (SPEAR320S_MAC_MODE_SMII << SPEAR320S_MAC2_MODE_SHIFT)
			| (SPEAR320S_MAC_MODE_SMII << SPEAR320S_MAC1_MODE_SHIFT)
			| SPEAR320S_MII_MDIO_10_11_VAL,
	},
};

static struct pmx_mux_reg pmx_rmii1_2_ext_mux[] = {
	{
		.address = SPEAR320S_IP_SEL_PAD_10_19_REG,
		.mask = SPEAR320S_PMX_PL_10_11_MASK |
			SPEAR320S_PMX_PL_13_14_MASK |
			SPEAR320S_PMX_PL_15_16_MASK |
			SPEAR320S_PMX_PL_17_18_MASK | SPEAR320S_PMX_PL_19_MASK,
		.value = SPEAR320S_PMX_RMII_PL_10_11_VAL |
			SPEAR320S_PMX_RMII_PL_13_14_VAL |
			SPEAR320S_PMX_RMII_PL_15_16_VAL |
			SPEAR320S_PMX_RMII_PL_17_18_VAL |
			SPEAR320S_PMX_RMII_PL_19_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_20_29_REG,
		.mask = SPEAR320S_PMX_PL_20_MASK |
			SPEAR320S_PMX_PL_21_TO_27_MASK,
		.value = SPEAR320S_PMX_RMII_PL_20_VAL |
			SPEAR320S_PMX_RMII_PL_21_TO_27_VAL,
	}, {
		.address = SPEAR320S_EXT_CTRL_REG,
		.mask = (SPEAR320S_MAC_MODE_MASK << SPEAR320S_MAC2_MODE_SHIFT) |
			(SPEAR320S_MAC_MODE_MASK << SPEAR320S_MAC1_MODE_SHIFT) |
			SPEAR320S_MII_MDIO_MASK,
		.value = (SPEAR320S_MAC_MODE_RMII << SPEAR320S_MAC2_MODE_SHIFT)
			| (SPEAR320S_MAC_MODE_RMII << SPEAR320S_MAC1_MODE_SHIFT)
			| SPEAR320S_MII_MDIO_10_11_VAL,
	},
};

#define pmx_mii1_2_common_modes						\
	{								\
		.ids = AUTO_NET_SMII_MODE | AUTO_EXP_MODE |		\
			SMALL_PRINTERS_MODE | SPEAR320S_EXTENDED_MODE,	\
		.mux_regs = pmx_mii1_2_mux,				\
		.mux_reg_cnt = ARRAY_SIZE(pmx_mii1_2_mux),		\
	}

static struct pmx_dev_mode pmx_mii1_2_modes[][2] = {
	{
		/* configure as smii */
		pmx_mii1_2_common_modes,
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_smii1_2_ext_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_smii1_2_ext_mux),
		},
	}, {
		/* configure as rmii */
		pmx_mii1_2_common_modes,
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_rmii1_2_ext_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_rmii1_2_ext_mux),
		},
	},
};

struct pmx_dev spear320_pmx_mii1_2[] = {
	{
		.name = "smii1_2",
		.modes = pmx_mii1_2_modes[0],
		.mode_count = ARRAY_SIZE(pmx_mii1_2_modes[0]),
	}, {
		.name = "rmii1_2",
		.modes = pmx_mii1_2_modes[1],
		.mode_count = ARRAY_SIZE(pmx_mii1_2_modes[1]),
	},
};

/* Pad multiplexing for i2c1 device */
static struct pmx_mux_reg pmx_i2c1_ext_8_9_mux[] = {
	{
		.address = SPEAR320S_IP_SEL_PAD_0_9_REG,
		.mask = SPEAR320S_PMX_PL_8_9_MASK,
		.value = SPEAR320S_PMX_I2C1_PL_8_9_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_MIX_PAD_REG,
		.mask = SPEAR320S_PMX_I2C1_PORT_SEL_MASK,
		.value = SPEAR320S_PMX_I2C1_PORT_8_9_VAL,
	},
};

static struct pmx_mux_reg pmx_i2c1_ext_98_99_mux[] = {
	{
		.address = SPEAR320S_IP_SEL_PAD_90_99_REG,
		.mask = SPEAR320S_PMX_PL_98_MASK |
			SPEAR320S_PMX_PL_99_MASK,
		.value = SPEAR320S_PMX_I2C1_PL_98_VAL |
			SPEAR320S_PMX_I2C1_PL_99_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_MIX_PAD_REG,
		.mask = SPEAR320S_PMX_I2C1_PORT_SEL_MASK,
		.value = SPEAR320S_PMX_I2C1_PORT_98_99_VAL,
	},
};

static struct pmx_dev_mode pmx_i2c1_modes[][1] = {
	{
		/* Select signals on pins 8-9 */
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_i2c1_ext_8_9_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_i2c1_ext_8_9_mux),
		},
	}, {
		/* Select signals on pins 98-99 */
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_i2c1_ext_98_99_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_i2c1_ext_98_99_mux),
		},
	},
};

struct pmx_dev spear320_pmx_i2c1[] = {
	{
		.name = "i2c1-8-9",
		.modes = pmx_i2c1_modes[0],
		.mode_count = ARRAY_SIZE(pmx_i2c1_modes[0]),
	}, {
		.name = "i2c1-98_99",
		.modes = pmx_i2c1_modes[1],
		.mode_count = ARRAY_SIZE(pmx_i2c1_modes[1]),
	},
};

/* Pad multiplexing for i2c2 device */
static struct pmx_mux_reg pmx_i2c2_ext_0_1_mux[] = {
	{
		.mask = PMX_FIRDA_MASK,
		.value = 0,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_0_9_REG,
		.mask = SPEAR320S_PMX_PL_0_1_MASK,
		.value = SPEAR320S_PMX_I2C2_PL_0_1_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_MIX_PAD_REG,
		.mask = SPEAR320S_PMX_I2C2_PORT_SEL_MASK,
		.value = SPEAR320S_PMX_I2C2_PORT_0_1_VAL,
	},
};

static struct pmx_mux_reg pmx_i2c2_ext_2_3_mux[] = {
	{
		.mask = PMX_UART0_MASK,
		.value = 0,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_0_9_REG,
		.mask = SPEAR320S_PMX_PL_2_3_MASK,
		.value = SPEAR320S_PMX_I2C2_PL_2_3_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_MIX_PAD_REG,
		.mask = SPEAR320S_PMX_I2C2_PORT_SEL_MASK,
		.value = SPEAR320S_PMX_I2C2_PORT_2_3_VAL,
	},
};

static struct pmx_mux_reg pmx_i2c2_ext_19_20_mux[] = {
	{
		.mask = PMX_MII_MASK,
		.value = 0,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_10_19_REG,
		.mask = SPEAR320S_PMX_PL_19_MASK,
		.value = SPEAR320S_PMX_I2C2_PL_19_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_PAD_20_29_REG,
		.mask = SPEAR320S_PMX_PL_20_MASK,
		.value = SPEAR320S_PMX_I2C2_PL_20_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_MIX_PAD_REG,
		.mask = SPEAR320S_PMX_I2C2_PORT_SEL_MASK,
		.value = SPEAR320S_PMX_I2C2_PORT_19_20_VAL,
	},
};

static struct pmx_mux_reg pmx_i2c2_ext_75_76_mux[] = {
	{
		.address = SPEAR320S_IP_SEL_PAD_70_79_REG,
		.mask = SPEAR320S_PMX_PL_75_76_MASK,
		.value = SPEAR320S_PMX_I2C2_PL_75_76_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_MIX_PAD_REG,
		.mask = SPEAR320S_PMX_I2C2_PORT_SEL_MASK,
		.value = SPEAR320S_PMX_I2C2_PORT_75_76_VAL,
	},
};

static struct pmx_mux_reg pmx_i2c2_ext_96_97_mux[] = {
	{
		.address = SPEAR320S_IP_SEL_PAD_90_99_REG,
		.mask = SPEAR320S_PMX_PL_96_97_MASK,
		.value = SPEAR320S_PMX_I2C2_PL_96_97_VAL,
	}, {
		.address = SPEAR320S_IP_SEL_MIX_PAD_REG,
		.mask = SPEAR320S_PMX_I2C2_PORT_SEL_MASK,
		.value = SPEAR320S_PMX_I2C2_PORT_96_97_VAL,
	},
};

static struct pmx_dev_mode pmx_i2c2_modes[][1] = {
	{
		/* Select signals on pins 0_1 */
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_i2c2_ext_0_1_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_i2c2_ext_0_1_mux),
		},
	}, {
		/* Select signals on pins 2_3 */
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_i2c2_ext_2_3_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_i2c2_ext_2_3_mux),
		},
	}, {
		/* Select signals on pins 19_20 */
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_i2c2_ext_19_20_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_i2c2_ext_19_20_mux),
		},
	}, {
		/* Select signals on pins 75_76 */
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_i2c2_ext_75_76_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_i2c2_ext_75_76_mux),
		},
	}, {
		/* Select signals on pins 96_97 */
		{
			.ids = SPEAR320S_EXTENDED_MODE,
			.mux_regs = pmx_i2c2_ext_96_97_mux,
			.mux_reg_cnt = ARRAY_SIZE(pmx_i2c2_ext_96_97_mux),
		},
	},
};

struct pmx_dev spear320s_pmx_i2c2[] = {
	{
		.name = "i2c2-0_1",
		.modes = pmx_i2c2_modes[0],
		.mode_count = ARRAY_SIZE(pmx_i2c2_modes[0]),
	}, {
		.name = "i2c2-2_3",
		.modes = pmx_i2c2_modes[1],
		.mode_count = ARRAY_SIZE(pmx_i2c2_modes[1]),
	}, {
		.name = "i2c2-19_20",
		.modes = pmx_i2c2_modes[2],
		.mode_count = ARRAY_SIZE(pmx_i2c2_modes[2]),
	}, {
		.name = "i2c2-75_76",
		.modes = pmx_i2c2_modes[3],
		.mode_count = ARRAY_SIZE(pmx_i2c2_modes[3]),
	}, {
		.name = "i2c2-96_97",
		.modes = pmx_i2c2_modes[4],
		.mode_count = ARRAY_SIZE(pmx_i2c2_modes[4]),
	},
};

/* pmx driver structure */
static struct pmx_driver pmx_driver = {
	.mode_reg = {.address = SPEAR320_CONTROL_REG, .mask = 0x00000007},
};

/* Add spear320 specific devices here */
/* CLCD device registration */
struct amba_device spear320_clcd_device = {
	.dev = {
		.init_name = "clcd",
		.coherent_dma_mask = ~0,
		.platform_data = &clcd_plat_data,
	},
	.res = {
		.start = SPEAR320_CLCD_BASE,
		.end = SPEAR320_CLCD_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	.dma_mask = ~0,
	.irq = {SPEAR320_VIRQ_CLCD, NO_IRQ},
};

/* DMAC platform data's slave info */
static struct pl08x_channel_data pl080_slave_channels[] = {
	{
		.bus_id = "uart0_rx",
		.min_signal = 2,
		.max_signal = 2,
		.muxval = 0,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "uart0_tx",
		.min_signal = 3,
		.max_signal = 3,
		.muxval = 0,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "ssp0_rx",
		.min_signal = 8,
		.max_signal = 8,
		.muxval = 0,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "ssp0_tx",
		.min_signal = 9,
		.max_signal = 9,
		.muxval = 0,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "i2c0_rx",
		.min_signal = 10,
		.max_signal = 10,
		.muxval = 0,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "i2c0_tx",
		.min_signal = 11,
		.max_signal = 11,
		.muxval = 0,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "irda",
		.min_signal = 12,
		.max_signal = 12,
		.muxval = 0,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "adc",
		.min_signal = 13,
		.max_signal = 13,
		.muxval = 0,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "to_jpeg",
		.min_signal = 14,
		.max_signal = 14,
		.muxval = 0,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "from_jpeg",
		.min_signal = 15,
		.max_signal = 15,
		.muxval = 0,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "ssp1_rx",
		.min_signal = 0,
		.max_signal = 0,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB2,
	}, {
		.bus_id = "ssp1_tx",
		.min_signal = 1,
		.max_signal = 1,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB2,
	}, {
		.bus_id = "ssp2_rx",
		.min_signal = 2,
		.max_signal = 2,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB2,
	}, {
		.bus_id = "ssp2_tx",
		.min_signal = 3,
		.max_signal = 3,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB2,
	}, {
		.bus_id = "uart1_rx",
		.min_signal = 4,
		.max_signal = 4,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB2,
	}, {
		.bus_id = "uart1_tx",
		.min_signal = 5,
		.max_signal = 5,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB2,
	}, {
		.bus_id = "uart2_rx",
		.min_signal = 6,
		.max_signal = 6,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB2,
	}, {
		.bus_id = "uart2_tx",
		.min_signal = 7,
		.max_signal = 7,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB2,
	}, {
		.bus_id = "i2c1_rx",
		.min_signal = 8,
		.max_signal = 8,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB2,
	}, {
		.bus_id = "i2c1_tx",
		.min_signal = 9,
		.max_signal = 9,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB2,
	}, {
		.bus_id = "i2c2_rx",
		.min_signal = 10,
		.max_signal = 10,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB2,
	}, {
		.bus_id = "i2c2_tx",
		.min_signal = 11,
		.max_signal = 11,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB2,
	}, {
		.bus_id = "i2s_rx",
		.min_signal = 12,
		.max_signal = 12,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB2,
	}, {
		.bus_id = "i2s_tx",
		.min_signal = 13,
		.max_signal = 13,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB2,
	}, {
		.bus_id = "rs485_rx",
		.min_signal = 14,
		.max_signal = 14,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB2,
	}, {
		.bus_id = "rs485_tx",
		.min_signal = 15,
		.max_signal = 15,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB2,
	},
};

/* ssp device registeration */
static struct pl022_ssp_controller ssp_platform_data[] = {
	{
		.bus_id = 1,
		.enable_dma = 1,
		.dma_filter = pl08x_filter_id,
		.dma_tx_param = "ssp1_tx",
		.dma_rx_param = "ssp1_rx",
		.num_chipselect = 2,
	}, {
		.bus_id = 2,
		.enable_dma = 1,
		.dma_filter = pl08x_filter_id,
		.dma_tx_param = "ssp2_tx",
		.dma_rx_param = "ssp2_rx",
		.num_chipselect = 2,
	}
};

struct amba_device spear320_ssp_device[] = {
	{
		.dev = {
			.coherent_dma_mask = ~0,
			.init_name = "ssp-pl022.1",
			.platform_data = &ssp_platform_data[0],
		},
		.res = {
			.start = SPEAR320_SSP1_BASE,
			.end = SPEAR320_SSP1_BASE + SZ_4K - 1,
			.flags = IORESOURCE_MEM,
		},
		.irq = {SPEAR320_VIRQ_SSP1, NO_IRQ},
	}, {
		.dev = {
			.coherent_dma_mask = ~0,
			.init_name = "ssp-pl022.2",
			.platform_data = &ssp_platform_data[1],
		},
		.res = {
			.start = SPEAR320_SSP2_BASE,
			.end = SPEAR320_SSP2_BASE + SZ_4K - 1,
			.flags = IORESOURCE_MEM,
		},
		.irq = {SPEAR320_VIRQ_SSP2, NO_IRQ},
	}
};

/* uart devices plat data */
static struct amba_pl011_data uart_data[] = {
	{
		.dma_filter = pl08x_filter_id,
		.dma_tx_param = "uart1_tx",
		.dma_rx_param = "uart1_rx",
	}, {
		.dma_filter = pl08x_filter_id,
		.dma_tx_param = "uart2_tx",
		.dma_rx_param = "uart2_rx",
	},
};

/* uart devices registeration */
struct amba_device spear320_uart1_device = {
	.dev = {
		.init_name = "uart1",
		.platform_data = &uart_data[0],
	},
	.res = {
		.start = SPEAR320_UART1_BASE,
		.end = SPEAR320_UART1_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	.irq = {SPEAR320_VIRQ_UART1, NO_IRQ},
};

struct amba_device spear320_uart2_device = {
	.dev = {
		.init_name = "uart2",
		.platform_data = &uart_data[1],
	},
	.res = {
		.start = SPEAR320_UART2_BASE,
		.end = SPEAR320_UART2_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	.irq = {SPEAR320_VIRQ_UART2, NO_IRQ},
};

struct amba_device spear320s_uart3_device = {
	.dev.init_name = "uart3",
	.res = {
		.start = SPEAR320S_UART3_BASE,
		.end = SPEAR320S_UART3_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	.irq = {SPEAR320S_VIRQ_UART3, NO_IRQ},
};

struct amba_device spear320s_uart4_device = {
	.dev.init_name = "uart4",
	.res = {
		.start = SPEAR320S_UART4_BASE,
		.end = SPEAR320S_UART4_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	.irq = {SPEAR320S_VIRQ_UART4, NO_IRQ},
};

struct amba_device spear320s_uart5_device = {
	.dev.init_name = "uart5",
	.res = {
		.start = SPEAR320S_UART5_BASE,
		.end = SPEAR320S_UART5_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	.irq = {SPEAR320S_VIRQ_UART5, NO_IRQ},
};

struct amba_device spear320s_uart6_device = {
	.dev.init_name = "uart6",
	.res = {
		.start = SPEAR320S_UART6_BASE,
		.end = SPEAR320S_UART6_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	.irq = {SPEAR320S_VIRQ_UART6, NO_IRQ},
};

struct amba_device spear320s_rs485_device = {
	.dev.init_name = "rs485",
	.res = {
		.start = SPEAR320S_RS485_BASE,
		.end = SPEAR320S_RS485_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	.irq = {SPEAR320S_VIRQ_RS485, NO_IRQ},
};

/* CAN device registeration */
static struct c_can_platform_data c_can_pdata;

static struct resource can0_resources[] = {
	{
		.start = SPEAR320_CAN0_BASE,
		.end = SPEAR320_CAN0_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM | IORESOURCE_MEM_32BIT,
	}, {
		.start = SPEAR320_VIRQ_CANU,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear320_can0_device = {
	.name = "c_can_platform",
	.id = 0,
	.num_resources = ARRAY_SIZE(can0_resources),
	.resource = can0_resources,
	.dev.platform_data = &c_can_pdata,
};

static struct resource can1_resources[] = {
	{
		.start = SPEAR320_CAN1_BASE,
		.end = SPEAR320_CAN1_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM | IORESOURCE_MEM_32BIT,
	}, {
		.start = SPEAR320_VIRQ_CANL,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear320_can1_device = {
	.name = "c_can_platform",
	.id = 1,
	.num_resources = ARRAY_SIZE(can1_resources),
	.resource = can1_resources,
	.dev.platform_data = &c_can_pdata,
};

/* emi nor flash device registeration */
static struct physmap_flash_data emi_norflash_data;
struct platform_device spear320_emi_nor_device = {
	.name	= "physmap-flash",
	.id	= -1,
	.dev.platform_data = &emi_norflash_data,
};

/* SPEAr320 RAS ethernet devices */
static u64 macb0_dmamask = ~0;
static struct resource eth0_resources[] = {
	{
		.start = SPEAR320_ETH0_BASE,
		.end = SPEAR320_ETH0_BASE + SZ_8K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR320_VIRQ_ETH0,
		.flags = IORESOURCE_IRQ,
	},
};

static u64 macb1_dmamask = ~0;
static struct resource eth1_resources[] = {
	{
		.start = SPEAR320_ETH1_BASE,
		.end = SPEAR320_ETH1_BASE + SZ_8K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR320_VIRQ_ETH1,
		.flags = IORESOURCE_IRQ,
	},
};

/*
 * ETH with pdev.id == 1, will control the MDIO lines. So, we give id == 1, for
 * ETH/ETH 1, for both 320 and 320s.
 */
struct platform_device spear320_eth0_device = {
	.name = "macb",
	.id = 0,
	.dev = {
		.dma_mask = &macb0_dmamask,
		.coherent_dma_mask = ~0,
	},
	.resource = eth0_resources,
	.num_resources = ARRAY_SIZE(eth0_resources),
};

struct platform_device spear320_eth1_device = {
	.name = "macb",
	.id = 1,
	.dev = {
		.dma_mask = &macb1_dmamask,
		.coherent_dma_mask = ~0,
	},
	.resource = eth1_resources,
	.num_resources = ARRAY_SIZE(eth1_resources),
};

/* i2c device registeration */
static struct resource i2c1_resources[] = {
	{
		.start = SPEAR320_I2C_BASE,
		.end = SPEAR320_I2C_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR320_VIRQ_I2C1 ,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear320_i2c1_device = {
	.name = "i2c_designware",
	.id = 1,
	.dev = {
		.coherent_dma_mask = ~0,
	},
	.num_resources = ARRAY_SIZE(i2c1_resources),
	.resource = i2c1_resources,
};

static struct resource i2c2_resources[] = {
	{
		.start = SPEAR320S_I2C2_BASE,
		.end = SPEAR320S_I2C2_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR320S_VIRQ_I2C2,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear320s_i2c2_device = {
	.name = "i2c_designware",
	.id = 2,
	.dev = {
		.coherent_dma_mask = ~0,
	},
	.num_resources = ARRAY_SIZE(i2c2_resources),
	.resource = i2c2_resources,
};

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
	struct clk *i2s_sclk_clk, *i2s_ref_clk;
	int ret = 0;


	i2s_sclk_clk = clk_get_sys(NULL, "i2s_sclk_clk");
	if (IS_ERR(i2s_sclk_clk)) {
		pr_err("%s:couldn't get i2s_sclk_clk\n", __func__);
		return PTR_ERR(i2s_sclk_clk);
	}

	i2s_ref_clk = clk_get_sys(NULL, "i2s_ref_clk");
	if (IS_ERR(i2s_ref_clk)) {
		pr_err("%s:couldn't get i2s_ref_clk\n", __func__);
		ret = PTR_ERR(i2s_ref_clk);
		goto put_i2s_sclk_clk;
	}

	/*
	 * 320s cannot generate accurate clock in some cases but slightly
	 * more than that which is under acceptable limits.
	 * But since clk_set_rate fails for this clock we explicitly try
	 * to set the higher clock.
	 */
#define REF_CLK_DELTA	10000
	ret = clk_set_rate(i2s_ref_clk, 256 * config->sample_rate +
			REF_CLK_DELTA);
	if (ret) {
		pr_err("%s:couldn't set i2s_ref_clk rate\n", __func__);
		goto put_i2s_ref_clk;
	}

	ret = clk_enable(i2s_sclk_clk);
	if (ret) {
		pr_err("%s:enabling i2s_sclk_clk\n", __func__);
		goto put_i2s_ref_clk;
	}

	return 0;

put_i2s_ref_clk:
	clk_put(i2s_ref_clk);
put_i2s_sclk_clk:
	clk_put(i2s_sclk_clk);

	return ret;

}

static struct i2s_platform_data i2s_data = {
	.cap = PLAY | RECORD,
	.channel = 2,
	.snd_fmts = SNDRV_PCM_FMTBIT_S16_LE,
	.snd_rates = (SNDRV_PCM_RATE_8000 | \
		 SNDRV_PCM_RATE_11025 | \
		 SNDRV_PCM_RATE_16000 | \
		 SNDRV_PCM_RATE_22050 | \
		 SNDRV_PCM_RATE_32000 | \
		 SNDRV_PCM_RATE_44100 | \
		 SNDRV_PCM_RATE_48000),
	.play_dma_data = "i2s_tx",
	.capture_dma_data = "i2s_rx",
	.filter = pl08x_filter_id,
	.i2s_clk_cfg = audio_clk_config,
};

/* i2s device registeration */
static struct resource i2s_resources[] = {
	{
		.start	= SPEAR320S_I2S_BASE,
		.end	= SPEAR320S_I2S_BASE + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device spear320_i2s_device = {
	.name = "designware-i2s",
	.id = -1,
	.dev = {
		.coherent_dma_mask = ~0,
		.platform_data = &i2s_data,
	},
	.num_resources = ARRAY_SIZE(i2s_resources),
	.resource = i2s_resources,
};

struct platform_device spear320_pcm_device = {
	.name		= "spear-pcm-audio",
	.id		= -1,
};

/* nand device registeration */
static struct fsmc_nand_platform_data nand_platform_data = {
	.mode = USE_WORD_ACCESS,
};

static struct resource nand_resources[] = {
	{
		.name = "nand_data",
		.start = SPEAR320_NAND_BASE,
		.end = SPEAR320_NAND_BASE + SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.name = "fsmc_regs",
		.start = SPEAR320_FSMC_BASE,
		.end = SPEAR320_FSMC_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
};

struct platform_device spear320_nand_device = {
	.name = "fsmc-nand",
	.id = -1,
	.resource = nand_resources,
	.num_resources = ARRAY_SIZE(nand_resources),
	.dev.platform_data = &nand_platform_data,
};

/* plgpio device registeration */
static struct plgpio_platform_data plgpio_plat_data = {
	.gpio_base = 8,
	.irq_base = SPEAR3XX_PLGPIO_INT_BASE,
	.gpio_count = SPEAR3XX_PLGPIO_COUNT,
	.regs = {
		.enb = SPEAR320_PLGPIO_ENB_OFF,
		.wdata = SPEAR320_PLGPIO_WDATA_OFF,
		.dir = SPEAR320_PLGPIO_DIR_OFF,
		.ie = SPEAR320_PLGPIO_IE_OFF,
		.rdata = SPEAR320_PLGPIO_RDATA_OFF,
		.mis = SPEAR320_PLGPIO_MIS_OFF,
		.eit = -1,
	},
};

static struct resource plgpio_resources[] = {
	{
		.start = SPEAR320_SOC_CONFIG_BASE,
		.end = SPEAR320_SOC_CONFIG_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR320_VIRQ_PLGPIO,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear320_plgpio_device = {
	.name = "plgpio",
	.id = -1,
	.dev = {
		.platform_data = &plgpio_plat_data,
	},
	.num_resources = ARRAY_SIZE(plgpio_resources),
	.resource = plgpio_resources,
};

/* pwm device registeration */
static struct resource pwm_resources[] = {
	{
		.start = SPEAR320_PWM_BASE,
		.end = SPEAR320_PWM_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
};

struct platform_device spear320_pwm_device = {
	.name = "pwm",
	.id = -1,
	.num_resources = ARRAY_SIZE(pwm_resources),
	.resource = pwm_resources,
};

/* sdhci (sdio) device registeration */
static struct resource sdhci_resources[] = {
	{
		.start	= SPEAR320_SDHCI_BASE,
		.end	= SPEAR320_SDHCI_BASE + SZ_256 - 1,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= SPEAR320_IRQ_SDHCI,
		.flags	= IORESOURCE_IRQ,
	}
};

struct platform_device spear320_sdhci_device = {
	.dev = {
		.coherent_dma_mask = ~0,
	},
	.name = "sdhci",
	.id = -1,
	.num_resources = ARRAY_SIZE(sdhci_resources),
	.resource = sdhci_resources,
};

/* standard parallel port device */
static struct resource spp_resources[] = {
	[0] = {
		.start = SPEAR320_PAR_PORT_BASE,
		.end = SPEAR320_PAR_PORT_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = SPEAR320_VIRQ_SPP,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear320_spp_device = {
	.name = "spear-spp",
	.id = -1,
	.num_resources = ARRAY_SIZE(spp_resources),
	.resource = spp_resources,
};

/* spear3xx shared irq */
static struct shirq_dev_config shirq_ras1_config[] = {
	{
		.virq = SPEAR320_VIRQ_EMI,
		.status_mask = SPEAR320_EMI_IRQ_MASK,
		.clear_mask = SPEAR320_EMI_IRQ_MASK,
	}, {
		.virq = SPEAR320_VIRQ_CLCD,
		.status_mask = SPEAR320_CLCD_IRQ_MASK,
		.clear_mask = SPEAR320_CLCD_IRQ_MASK,
	}, {
		.virq = SPEAR320_VIRQ_SPP,
		.status_mask = SPEAR320_SPP_IRQ_MASK,
		.clear_mask = SPEAR320_SPP_IRQ_MASK,
	},
};

static struct spear_shirq shirq_ras1 = {
	.irq = SPEAR3XX_IRQ_GEN_RAS_1,
	.dev_config = shirq_ras1_config,
	.dev_count = ARRAY_SIZE(shirq_ras1_config),
	.regs = {
		.base = IOMEM(VA_SPEAR320_SOC_CONFIG_BASE),
		.enb_reg = -1,
		.status_reg = SPEAR320_INT_STS_MASK_REG,
		.status_reg_mask = SPEAR320_SHIRQ_RAS1_MASK,
		.clear_reg = SPEAR320_INT_CLR_MASK_REG,
		.reset_to_clear = 1,
	},
};

static struct shirq_dev_config shirq_ras3_config[] = {
	{
		.virq = SPEAR320_VIRQ_PLGPIO,
		.enb_mask = SPEAR320_GPIO_IRQ_MASK,
		.status_mask = SPEAR320_GPIO_IRQ_MASK,
		.clear_mask = SPEAR320_GPIO_IRQ_MASK,
	}, {
		.virq = SPEAR320_VIRQ_I2S_PLAY,
		.enb_mask = SPEAR320S_I2S_IRQ_PLAY_OR_M,
		.status_mask = SPEAR320S_I2S_IRQ_PLAY_OR_M,
		.clear_mask = SPEAR320S_I2S_IRQ_PLAY_OR_M,
	}, {
		.virq = SPEAR320_VIRQ_I2S_REC,
		.enb_mask = SPEAR320S_I2S_IRQ_REC_OR_S,
		.status_mask = SPEAR320S_I2S_IRQ_REC_OR_S,
		.clear_mask = SPEAR320S_I2S_IRQ_REC_OR_S,
	},
};

static struct spear_shirq shirq_ras3 = {
	.irq = SPEAR3XX_IRQ_GEN_RAS_3,
	.dev_config = shirq_ras3_config,
	.dev_count = ARRAY_SIZE(shirq_ras3_config),
	.regs = {
		.base = IOMEM(VA_SPEAR320_SOC_CONFIG_BASE),
		.enb_reg = SPEAR320_INT_ENB_MASK_REG,
		.reset_to_enb = 1,
		.status_reg = SPEAR320_INT_STS_MASK_REG,
		.status_reg_mask = SPEAR320_SHIRQ_RAS3_MASK,
		.clear_reg = SPEAR320_INT_CLR_MASK_REG,
		.reset_to_clear = 1,
	},
};

static struct shirq_dev_config shirq_intrcomm_ras_config[] = {
	{
		.virq = SPEAR320_VIRQ_CANU,
		.status_mask = SPEAR320_CAN_U_IRQ_MASK,
		.clear_mask = SPEAR320_CAN_U_IRQ_MASK,
	}, {
		.virq = SPEAR320_VIRQ_CANL,
		.status_mask = SPEAR320_CAN_L_IRQ_MASK,
		.clear_mask = SPEAR320_CAN_L_IRQ_MASK,
	}, {
		.virq = SPEAR320_VIRQ_UART1,
		.status_mask = SPEAR320_UART1_IRQ_MASK,
		.clear_mask = SPEAR320_UART1_IRQ_MASK,
	}, {
		.virq = SPEAR320_VIRQ_UART2,
		.status_mask = SPEAR320_UART2_IRQ_MASK,
		.clear_mask = SPEAR320_UART2_IRQ_MASK,
	}, {
		.virq = SPEAR320_VIRQ_SSP1,
		.status_mask = SPEAR320_SSP1_IRQ_MASK,
		.clear_mask = SPEAR320_SSP1_IRQ_MASK,
	}, {
		.virq = SPEAR320_VIRQ_SSP2,
		.status_mask = SPEAR320_SSP2_IRQ_MASK,
		.clear_mask = SPEAR320_SSP2_IRQ_MASK,
	}, {
		.virq = SPEAR320_VIRQ_ETH0,
		.status_mask = SPEAR320_ETH0_IRQ_MASK,
		.clear_mask = SPEAR320_ETH0_IRQ_MASK,
	}, {
		.virq = SPEAR320_VIRQ_ETH1,
		.status_mask = SPEAR320_ETH1_IRQ_MASK,
		.clear_mask = SPEAR320_ETH1_IRQ_MASK,
	}, {
		.virq = SPEAR320_VIRQ_WAKEUP_ETH0,
		.status_mask = SPEAR320_WAKEUP_ETH0_IRQ_MASK,
		.clear_mask = SPEAR320_WAKEUP_ETH0_IRQ_MASK,
	}, {
		.virq = SPEAR320_VIRQ_WAKEUP_ETH1,
		.status_mask = SPEAR320_WAKEUP_ETH1_IRQ_MASK,
		.clear_mask = SPEAR320_WAKEUP_ETH1_IRQ_MASK,
	}, {
		.virq = SPEAR320_VIRQ_I2C1,
		.status_mask = SPEAR320_I2C1_IRQ_MASK,
		.clear_mask = SPEAR320_I2C1_IRQ_MASK,
	}, {
		.virq = SPEAR320S_VIRQ_I2C2,
		.status_mask = SPEAR320S_I2C2_IRQ_MASK,
		.clear_mask = SPEAR320S_I2C2_IRQ_MASK,
	}, {
		.virq = SPEAR320S_VIRQ_UART3,
		.status_mask = SPEAR320S_UART3_IRQ_MASK,
		.clear_mask = SPEAR320S_UART3_IRQ_MASK,
	}, {
		.virq = SPEAR320S_VIRQ_UART4,
		.status_mask = SPEAR320S_UART4_IRQ_MASK,
		.clear_mask = SPEAR320S_UART4_IRQ_MASK,
	}, {
		.virq = SPEAR320S_VIRQ_UART5,
		.status_mask = SPEAR320S_UART5_IRQ_MASK,
		.clear_mask = SPEAR320S_UART5_IRQ_MASK,
	}, {
		.virq = SPEAR320S_VIRQ_UART6,
		.status_mask = SPEAR320S_UART6_IRQ_MASK,
		.clear_mask = SPEAR320S_UART6_IRQ_MASK,
	}, {
		.virq = SPEAR320S_VIRQ_RS485,
		.status_mask = SPEAR320S_RS485_IRQ_MASK,
		.clear_mask = SPEAR320S_RS485_IRQ_MASK,
	},
};

static struct spear_shirq shirq_intrcomm_ras = {
	.irq = SPEAR3XX_IRQ_INTRCOMM_RAS_ARM,
	.dev_config = shirq_intrcomm_ras_config,
	.dev_count = ARRAY_SIZE(shirq_intrcomm_ras_config),
	.regs = {
		.base = IOMEM(VA_SPEAR320_SOC_CONFIG_BASE),
		.enb_reg = -1,
		.status_reg = SPEAR320_INT_STS_MASK_REG,
		.status_reg_mask = SPEAR320_SHIRQ_INTRCOMM_RAS_MASK,
		.clear_reg = SPEAR320_INT_CLR_MASK_REG,
		.reset_to_clear = 1,
	},
};

/* Following will create 320 specific static virtual/physical mappings */
static struct map_desc spear320_io_desc[] __initdata = {
	{
		.virtual	= VA_SPEAR320_SOC_CONFIG_BASE,
		.pfn		= __phys_to_pfn(SPEAR320_SOC_CONFIG_BASE),
		.length		= SZ_4K,
		.type		= MT_DEVICE
	},
};

/* spear320s routines */
static void c_can_disable_bugfix(struct platform_device *c_can)
{
	struct c_can_platform_data *pdata = dev_get_platdata(&c_can->dev);

	pdata->is_quirk_required = false;
	pdata->devtype_data.rx_first = 1;
	pdata->devtype_data.rx_split = 20;
	pdata->devtype_data.rx_last = 26;
	pdata->devtype_data.tx_num = 6;
}

/* spear320s routines */
static void c_can_enable_bugfix(struct platform_device *c_can)
{
	struct c_can_platform_data *pdata = dev_get_platdata(&c_can->dev);

	pdata->is_quirk_required = true;
	pdata->devtype_data.rx_first = 1;
	pdata->devtype_data.rx_split = 25;
	pdata->devtype_data.rx_last = 31;
	pdata->devtype_data.tx_num = 1;
}

static void i2s_clk_init(void)
{
	int ret;
	struct clk *i2s_sclk_clk;

	i2s_sclk_clk = clk_get(NULL, "i2s_sclk_clk");
	if (IS_ERR(i2s_sclk_clk)) {
		pr_err("%s:couldn't get i2s_sclk_clk\n", __func__);
		return;
	}

	ret = clk_set_parent_sys(NULL, "synth2_3_pclk", NULL, "pll1_clk");
	if (ret) {
		pr_err("%s:set_parent synth2_3_pclk of pll1_clk fail\n",
				__func__);
		goto put_sclk_clk;
	}

	ret = clk_set_parent_sys(NULL, "i2s_ref_clk", NULL, "ras_synth2_clk");
	if (ret) {
		pr_err("%s:set_parent ras_synth2_clk of i2s_ref_clk fail\n",
				__func__);
		goto put_sclk_clk;
	}

	if (clk_set_rate(i2s_sclk_clk, 3073000)) /* 3.072 Mhz */
		goto put_sclk_clk;

	if (clk_enable(i2s_sclk_clk))
		pr_err("%s:enabling i2s_sclk_clk\n", __func__);
put_sclk_clk:
	clk_put(i2s_sclk_clk);
}

/*
 * retreive the SoC-id for differentiating between SPEAr320
 * and future variants of the same (for e.g. SPEAr320s)
 */
#define SOC_SPEAR320_ID		0x1
#define SOC_SPEAR320S_ID	0x2
static int get_soc_id(void)
{
	int soc_id = readl(VA_SOC_CORE_ID);

	return soc_id ? SOC_SPEAR320S_ID : SOC_SPEAR320_ID;
}

/* This will create static memory mapping for selected devices */
void __init spear320_map_io(void)
{
	iotable_init(spear320_io_desc,
			ARRAY_SIZE(spear320_io_desc));
	spear3xx_map_io();
}

/*
 * There are two Soc Versions of SPEAr320. One is called SPEAr320 and other is
 * called SPEAr320s.
 */
void __init spear320_init(void)
{
	/* enable bug-fix for CAN controllers */
	c_can_enable_bugfix(&spear320_can0_device);
	c_can_enable_bugfix(&spear320_can1_device);

	if (spear_shirq_register(&shirq_ras3))
		printk(KERN_ERR "Error registering Shared IRQ 3\n");

}

void __init spear320s_init(struct pmx_mode *pmx_mode)
{
	/* disable bug-fix for CAN controllers */
	c_can_disable_bugfix(&spear320_can0_device);
	c_can_disable_bugfix(&spear320_can1_device);

	if (pmx_mode == &spear320s_extended_mode) {
		/* Fix SPEAr320s specific pmx stuff */
		pmx_driver.mode_reg.address = SPEAR320S_EXT_CTRL_REG;
		pmx_driver.mode_reg.mask = SPEAR320S_EXTENDED_MODE_VAL;

		/* Update platform data for 320s extended mode */
		plgpio_plat_data.regs.mis = SPEAR320S_PLGPIO_MIS_OFF;
		plgpio_plat_data.regs.eit = SPEAR320S_PLGPIO_EI_OFF;
	} else {
		/* Clear ext ctrl reg to select legacy modes */
		writel(0, VA_SPEAR320S_EXT_CTRL_REG);
	}
}

void __init spear320_common_init(struct pmx_mode *pmx_mode, struct pmx_dev
		**pmx_devs, u8 pmx_dev_count)
{
	int ret = 0;
	int soc_id;

	/* call spear3xx family common init function */
	spear3xx_init();

	i2s_clk_init();

	/* shared irq registration */
	ret = spear_shirq_register(&shirq_ras1);
	if (ret)
		printk(KERN_ERR "Error registering Shared IRQ 1\n");

	ret = spear_shirq_register(&shirq_intrcomm_ras);
	if (ret)
		printk(KERN_ERR "Error registering Shared IRQ 4\n");

	/* Set DMAC platform data's slave info */
	pl080_set_slaveinfo(&spear3xx_dma_device, pl080_slave_channels,
			ARRAY_SIZE(pl080_slave_channels));

	/* retreive soc-id */
	soc_id = get_soc_id();
	if (soc_id == SOC_SPEAR320_ID)
		spear320_init();
	else
		spear320s_init(pmx_mode);

	/* pmx initialization */
	pmx_driver.mode = pmx_mode;
	pmx_driver.devs = pmx_devs;
	pmx_driver.devs_count = pmx_dev_count;

	/* This fixes addresses of all pmx devices for spear320 */
	spear3xx_pmx_init_addr(&pmx_driver, SPEAR320_PAD_MUX_CONFIG_REG);
	ret = pmx_register(&pmx_driver);
	if (ret)
		pr_err("padmux: registeration failed. err no: %d\n", ret);
}
