/*
 * arch/arm/mach-spear13xx/spear1310.c
 *
 * SPEAr1310 machine source file
 *
 * Copyright (C) 2011 ST Microelectronics
 * Viresh Kumar <viresh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/ahci_platform.h>
#include <linux/amba/pl022.h>
#include <linux/can/platform/c_can.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/i2c-designware.h>
#include <linux/mtd/fsmc.h>
#include <asm/irq.h>
#include <plat/hdlc.h>
#include <mach/dw_pcie.h>
#include <mach/generic.h>
#include <mach/hardware.h>
#include <mach/spear1310_misc_regs.h>

/* pmx driver structure */
static struct pmx_driver pmx_driver;

/* Pad multiplexing for uart1 device */
/* Muxed with I2C */
static struct pmx_mux_reg pmx_uart1_dis_i2c_mux[] = {
	{
		.address = SPEAR1310_PAD_FUNCTION_EN_1,
		.mask = SPEAR13XX_PMX_I2C_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_uart1_dis_i2c_modes[] = {
	{
		.mux_regs = pmx_uart1_dis_i2c_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_uart1_dis_i2c_mux),
	},
};

struct pmx_dev spear1310_pmx_uart_1_dis_i2c = {
	.name = "uart1 disable i2c",
	.modes = pmx_uart1_dis_i2c_modes,
	.mode_count = ARRAY_SIZE(pmx_uart1_dis_i2c_modes),
};

/* Muxed with SD/MMC */
static struct pmx_mux_reg pmx_uart1_dis_sd_mux[] = {
	{
		.address = SPEAR1310_PAD_FUNCTION_EN_2,
		.mask = SPEAR13XX_PMX_MCIDATA1_MASK |
			SPEAR13XX_PMX_MCIDATA2_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_uart1_dis_sd_modes[] = {
	{
		.mux_regs = pmx_uart1_dis_sd_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_uart1_dis_sd_mux),
	},
};

struct pmx_dev spear1310_pmx_uart_1_dis_sd = {
	.name = "uart1 disable sd",
	.modes = pmx_uart1_dis_sd_modes,
	.mode_count = ARRAY_SIZE(pmx_uart1_dis_sd_modes),
};

/* Pad multiplexing for uart2_3 device */
static struct pmx_mux_reg pmx_uart2_3_mux[] = {
	{
		.address = SPEAR1310_PAD_FUNCTION_EN_1,
		.mask = SPEAR13XX_PMX_I2S1_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_uart2_3_modes[] = {
	{
		.mux_regs = pmx_uart2_3_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_uart2_3_mux),
	},
};

struct pmx_dev spear1310_pmx_uart_2_3 = {
	.name = "uart2_3",
	.modes = pmx_uart2_3_modes,
	.mode_count = ARRAY_SIZE(pmx_uart2_3_modes),
};

/* Pad multiplexing for uart4 device */
static struct pmx_mux_reg pmx_uart4_mux[] = {
	{
		.address = SPEAR1310_PAD_FUNCTION_EN_1,
		.mask = SPEAR13XX_PMX_I2S1_MASK | SPEAR13XX_PMX_CLCD1_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_uart4_modes[] = {
	{
		.mux_regs = pmx_uart4_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_uart4_mux),
	},
};

struct pmx_dev spear1310_pmx_uart_4 = {
	.name = "uart4",
	.modes = pmx_uart4_modes,
	.mode_count = ARRAY_SIZE(pmx_uart4_modes),
};

/* Pad multiplexing for uart5 device */
static struct pmx_mux_reg pmx_uart5_mux[] = {
	{
		.address = SPEAR1310_PAD_FUNCTION_EN_1,
		.mask = SPEAR13XX_PMX_CLCD1_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_uart5_modes[] = {
	{
		.mux_regs = pmx_uart5_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_uart5_mux),
	},
};

struct pmx_dev spear1310_pmx_uart_5 = {
	.name = "uart5",
	.modes = pmx_uart5_modes,
	.mode_count = ARRAY_SIZE(pmx_uart5_modes),
};

/* Pad multiplexing for rs485_0_1_tdm_0_1 device */
static struct pmx_mux_reg pmx_rs485_0_1_tdm_0_1_mux[] = {
	{
		.address = SPEAR1310_PAD_FUNCTION_EN_1,
		.mask = SPEAR13XX_PMX_CLCD1_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_rs485_0_1_tdm_0_1_modes[] = {
	{
		.mux_regs = pmx_rs485_0_1_tdm_0_1_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_rs485_0_1_tdm_0_1_mux),
	},
};

struct pmx_dev spear1310_pmx_rs485_0_1_tdm_0_1 = {
	.name = "rs485_0_1_tdm_0_1",
	.modes = pmx_rs485_0_1_tdm_0_1_modes,
	.mode_count = ARRAY_SIZE(pmx_rs485_0_1_tdm_0_1_modes),
};

/* Pad multiplexing for i2c_1_2 device */
static struct pmx_mux_reg pmx_i2c_1_2_mux[] = {
	{
		.address = SPEAR1310_PAD_FUNCTION_EN_1,
		.mask = SPEAR13XX_PMX_CLCD1_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_i2c_1_2_modes[] = {
	{
		.mux_regs = pmx_i2c_1_2_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_i2c_1_2_mux),
	},
};

struct pmx_dev spear1310_pmx_i2c_1_2 = {
	.name = "i2c_1_2",
	.modes = pmx_i2c_1_2_modes,
	.mode_count = ARRAY_SIZE(pmx_i2c_1_2_modes),
};

/* Pad multiplexing for i2c3_dis_smi_clcd device */
/* Muxed with SMI & CLCD */
static struct pmx_mux_reg pmx_i2c3_dis_smi_clcd_mux[] = {
	{
		.address = SPEAR1310_PAD_FUNCTION_EN_1,
		.mask = SPEAR13XX_PMX_CLCD1_MASK | SPEAR13XX_PMX_SMI_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_i2c3_dis_smi_clcd_modes[] = {
	{
		.mux_regs = pmx_i2c3_dis_smi_clcd_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_i2c3_dis_smi_clcd_mux),
	},
};

struct pmx_dev spear1310_pmx_i2c3_dis_smi_clcd = {
	.name = "i2c3_dis_smi_clcd",
	.modes = pmx_i2c3_dis_smi_clcd_modes,
	.mode_count = ARRAY_SIZE(pmx_i2c3_dis_smi_clcd_modes),
};

/* Pad multiplexing for i2c3_dis_sd_i2s1 device */
/* Muxed with SD/MMC & I2S1 */
static struct pmx_mux_reg pmx_i2c3_dis_sd_i2s1_mux[] = {
	{
		.address = SPEAR1310_PAD_FUNCTION_EN_2,
		.mask = SPEAR13XX_PMX_I2S2_MASK | SPEAR13XX_PMX_MCIDATA3_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_i2c3_dis_sd_i2s1_modes[] = {
	{
		.mux_regs = pmx_i2c3_dis_sd_i2s1_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_i2c3_dis_sd_i2s1_mux),
	},
};

struct pmx_dev spear1310_pmx_i2c3_dis_sd_i2s1 = {
	.name = "i2c3_dis_sd_i2s1",
	.modes = pmx_i2c3_dis_sd_i2s1_modes,
	.mode_count = ARRAY_SIZE(pmx_i2c3_dis_sd_i2s1_modes),
};

/* Pad multiplexing for i2c_4_5_dis_smi device */
/* Muxed with SMI */
static struct pmx_mux_reg pmx_i2c_4_5_dis_smi_mux[] = {
	{
		.address = SPEAR1310_PAD_FUNCTION_EN_1,
		.mask = SPEAR13XX_PMX_SMI_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_i2c_4_5_dis_smi_modes[] = {
	{
		.mux_regs = pmx_i2c_4_5_dis_smi_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_i2c_4_5_dis_smi_mux),
	},
};

struct pmx_dev spear1310_pmx_i2c_4_5_dis_smi = {
	.name = "i2c_4_5_dis_smi",
	.modes = pmx_i2c_4_5_dis_smi_modes,
	.mode_count = ARRAY_SIZE(pmx_i2c_4_5_dis_smi_modes),
};

/* Pad multiplexing for i2c4_dis_sd device */
/* Muxed with SD/MMC */
static struct pmx_mux_reg pmx_i2c4_dis_sd_mux[] = {
	{
		.address = SPEAR1310_PAD_FUNCTION_EN_2,
		.mask = SPEAR13XX_PMX_MCIDATA4_MASK,
		.value = 0,
	}, {
		.address = SPEAR1310_PAD_FUNCTION_EN_3,
		.mask = SPEAR13XX_PMX_MCIDATA5_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_i2c4_dis_sd_modes[] = {
	{
		.mux_regs = pmx_i2c4_dis_sd_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_i2c4_dis_sd_mux),
	},
};

struct pmx_dev spear1310_pmx_i2c4_dis_sd = {
	.name = "i2c4_dis_sd",
	.modes = pmx_i2c4_dis_sd_modes,
	.mode_count = ARRAY_SIZE(pmx_i2c4_dis_sd_modes),
};

/* Pad multiplexing for i2c5_dis_sd device */
/* Muxed with SD/MMC */
static struct pmx_mux_reg pmx_i2c5_dis_sd_mux[] = {
	{
		.address = SPEAR1310_PAD_FUNCTION_EN_3,
		.mask = SPEAR13XX_PMX_MCIDATA6_MASK |
			SPEAR13XX_PMX_MCIDATA7_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_i2c5_dis_sd_modes[] = {
	{
		.mux_regs = pmx_i2c5_dis_sd_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_i2c5_dis_sd_mux),
	},
};

struct pmx_dev spear1310_pmx_i2c5_dis_sd = {
	.name = "i2c5_dis_sd",
	.modes = pmx_i2c5_dis_sd_modes,
	.mode_count = ARRAY_SIZE(pmx_i2c5_dis_sd_modes),
};

/* Pad multiplexing for i2c_6_7_dis_kbd device */
/* Muxed with KBD */
static struct pmx_mux_reg pmx_i2c_6_7_dis_kbd_mux[] = {
	{
		.address = SPEAR1310_PAD_FUNCTION_EN_2,
		.mask = SPEAR13XX_PMX_KBD_ROWCOL25_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_i2c_6_7_dis_kbd_modes[] = {
	{
		.mux_regs = pmx_i2c_6_7_dis_kbd_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_i2c_6_7_dis_kbd_mux),
	},
};

struct pmx_dev spear1310_pmx_i2c_6_7_dis_kbd = {
	.name = "i2c_6_7_dis_kbd",
	.modes = pmx_i2c_6_7_dis_kbd_modes,
	.mode_count = ARRAY_SIZE(pmx_i2c_6_7_dis_kbd_modes),
};

/* Pad multiplexing for i2c6_dis_sd device */
/* Muxed with SD/MMC */
static struct pmx_mux_reg pmx_i2c6_dis_sd_mux[] = {
	{
		.address = SPEAR1310_PAD_FUNCTION_EN_3,
		.mask = SPEAR13XX_PMX_MCIIORDRE_MASK |
			SPEAR13XX_PMX_MCIIOWRWE_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_i2c6_dis_sd_modes[] = {
	{
		.mux_regs = pmx_i2c6_dis_sd_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_i2c6_dis_sd_mux),
	},
};

struct pmx_dev spear1310_pmx_i2c6_dis_sd = {
	.name = "i2c6_dis_sd",
	.modes = pmx_i2c6_dis_sd_modes,
	.mode_count = ARRAY_SIZE(pmx_i2c6_dis_sd_modes),
};

/* Pad multiplexing for i2c7_dis_sd device */
static struct pmx_mux_reg pmx_i2c7_dis_sd_mux[] = {
	{
		.address = SPEAR1310_PAD_FUNCTION_EN_3,
		.mask = SPEAR13XX_PMX_MCIRESETCF_MASK |
			SPEAR13XX_PMX_MCICS0CE_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_i2c7_dis_sd_modes[] = {
	{
		.mux_regs = pmx_i2c7_dis_sd_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_i2c7_dis_sd_mux),
	},
};

struct pmx_dev spear1310_pmx_i2c7_dis_sd = {
	.name = "i2c7_dis_sd",
	.modes = pmx_i2c7_dis_sd_modes,
	.mode_count = ARRAY_SIZE(pmx_i2c7_dis_sd_modes),
};

/* Pad multiplexing for rgmii device */
static struct pmx_mux_reg pmx_rgmii_mux[] = {
	{
		.address = SPEAR1310_PAD_FUNCTION_EN_1,
		.mask = SPEAR1310_PMX_RGMII_REG0_MASK,
		.value = 0,
	}, {
		.address = SPEAR1310_PAD_FUNCTION_EN_2,
		.mask = SPEAR1310_PMX_RGMII_REG1_MASK,
		.value = 0,
	}, {
		.address = SPEAR1310_PAD_FUNCTION_EN_3,
		.mask = SPEAR1310_PMX_RGMII_REG2_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_rgmii_modes[] = {
	{
		.mux_regs = pmx_rgmii_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_rgmii_mux),
	},
};

struct pmx_dev spear1310_pmx_rgmii = {
	.name = "rgmii",
	.modes = pmx_rgmii_modes,
	.mode_count = ARRAY_SIZE(pmx_rgmii_modes),
};

/* Pad multiplexing for can0_dis_nor device */
/* Muxed with NOR */
static struct pmx_mux_reg pmx_can0_dis_nor_mux[] = {
	{
		.address = SPEAR1310_PAD_FUNCTION_EN_1,
		.mask = SPEAR13XX_PMX_NFRSTPWDWN2_MASK,
		.value = 0,
	}, {
		.address = SPEAR1310_PAD_FUNCTION_EN_2,
		.mask = SPEAR13XX_PMX_NFRSTPWDWN3_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_can0_dis_nor_modes[] = {
	{
		.mux_regs = pmx_can0_dis_nor_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_can0_dis_nor_mux),
	},
};

struct pmx_dev spear1310_pmx_can0_dis_nor = {
	.name = "can0_dis_nor",
	.modes = pmx_can0_dis_nor_modes,
	.mode_count = ARRAY_SIZE(pmx_can0_dis_nor_modes),
};

/* Pad multiplexing for can0_dis_sd device */
/* Muxed with SD/MMC */
static struct pmx_mux_reg pmx_can0_dis_sd_mux[] = {
	{
		.address = SPEAR1310_PAD_FUNCTION_EN_3,
		.mask = SPEAR13XX_PMX_MCICFINTR_MASK |
			SPEAR13XX_PMX_MCIIORDY_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_can0_dis_sd_modes[] = {
	{
		.mux_regs = pmx_can0_dis_sd_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_can0_dis_sd_mux),
	},
};

struct pmx_dev spear1310_pmx_can0_dis_sd = {
	.name = "can0_dis_sd",
	.modes = pmx_can0_dis_sd_modes,
	.mode_count = ARRAY_SIZE(pmx_can0_dis_sd_modes),
};

/* Pad multiplexing for can1_dis_sd device */
/* Muxed with SD/MMC */
static struct pmx_mux_reg pmx_can1_dis_sd_mux[] = {
	{
		.address = SPEAR1310_PAD_FUNCTION_EN_3,
		.mask = SPEAR13XX_PMX_MCICS1_MASK |
			SPEAR13XX_PMX_MCIDMAACK_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_can1_dis_sd_modes[] = {
	{
		.mux_regs = pmx_can1_dis_sd_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_can1_dis_sd_mux),
	},
};

struct pmx_dev spear1310_pmx_can1_dis_sd = {
	.name = "can1_dis_sd",
	.modes = pmx_can1_dis_sd_modes,
	.mode_count = ARRAY_SIZE(pmx_can1_dis_sd_modes),
};

/* Pad multiplexing for can1_dis_kbd device */
/* Muxed with KBD */
static struct pmx_mux_reg pmx_can1_dis_kbd_mux[] = {
	{
		.address = SPEAR1310_PAD_FUNCTION_EN_2,
		.mask = SPEAR13XX_PMX_KBD_ROWCOL25_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_can1_dis_kbd_modes[] = {
	{
		.mux_regs = pmx_can1_dis_kbd_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_can1_dis_kbd_mux),
	},
};

struct pmx_dev spear1310_pmx_can1_dis_kbd = {
	.name = "can1_dis_kbd",
	.modes = pmx_can1_dis_kbd_modes,
	.mode_count = ARRAY_SIZE(pmx_can1_dis_kbd_modes),
};

/* Pad multiplexing for pci device */
static struct pmx_mux_reg pmx_pci_mux[] = {
	{
		.address = SPEAR1310_PAD_FUNCTION_EN_1,
		.mask = SPEAR1310_PMX_PCI_REG0_MASK,
		.value = 0,
	}, {
		.address = SPEAR1310_PAD_FUNCTION_EN_2,
		.mask = SPEAR1310_PMX_PCI_REG1_MASK,
		.value = 0,
	}, {
		.address = SPEAR1310_PAD_FUNCTION_EN_3,
		.mask = SPEAR1310_PMX_PCI_REG2_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_pci_modes[] = {
	{
		.mux_regs = pmx_pci_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_pci_mux),
	},
};

struct pmx_dev spear1310_pmx_pci = {
	.name = "pci",
	.modes = pmx_pci_modes,
	.mode_count = ARRAY_SIZE(pmx_pci_modes),
};

/* Pad multiplexing for smii_0_1_2 device */
static struct pmx_mux_reg pmx_smii_0_1_2_mux[] = {
	{
		.address = SPEAR1310_PAD_FUNCTION_EN_2,
		.mask = SPEAR1310_PMX_SMII_0_1_2_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_smii_0_1_2_modes[] = {
	{
		.mux_regs = pmx_smii_0_1_2_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_smii_0_1_2_mux),
	},
};

struct pmx_dev spear1310_pmx_smii_0_1_2 = {
	.name = "smii_0_1_2",
	.modes = pmx_smii_0_1_2_modes,
	.mode_count = ARRAY_SIZE(pmx_smii_0_1_2_modes),
};

/* Pad multiplexing for ssp1_dis_kbd device */
static struct pmx_mux_reg pmx_ssp1_dis_kbd_mux[] = {
	{
		.address = SPEAR1310_PAD_FUNCTION_EN_2,
		.mask = SPEAR13XX_PMX_KBD_ROWCOL25_MASK |
			SPEAR13XX_PMX_KBD_COL1_MASK |
			SPEAR13XX_PMX_KBD_COL0_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_ssp1_dis_kbd_modes[] = {
	{
		.mux_regs = pmx_ssp1_dis_kbd_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_ssp1_dis_kbd_mux),
	},
};

struct pmx_dev spear1310_pmx_ssp1_dis_kbd = {
	.name = "ssp1_dis_kbd",
	.modes = pmx_ssp1_dis_kbd_modes,
	.mode_count = ARRAY_SIZE(pmx_ssp1_dis_kbd_modes),
};

/* Pad multiplexing for ssp1_dis_sd device */
static struct pmx_mux_reg pmx_ssp1_dis_sd_mux[] = {
	{
		.address = SPEAR1310_PAD_FUNCTION_EN_3,
		.mask = SPEAR13XX_PMX_MCIADDR0ALE_MASK |
			SPEAR13XX_PMX_MCIADDR2_MASK | SPEAR13XX_PMX_MCICECF_MASK
			| SPEAR13XX_PMX_MCICEXD_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_ssp1_dis_sd_modes[] = {
	{
		.mux_regs = pmx_ssp1_dis_sd_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_ssp1_dis_sd_mux),
	},
};

struct pmx_dev spear1310_pmx_ssp1_dis_sd = {
	.name = "ssp1_dis_sd",
	.modes = pmx_ssp1_dis_sd_modes,
	.mode_count = ARRAY_SIZE(pmx_ssp1_dis_sd_modes),
};

/* Pad multiplexing for gpt64 device */
static struct pmx_mux_reg pmx_gpt64_mux[] = {
	{
		.address = SPEAR1310_PAD_FUNCTION_EN_3,
		.mask = SPEAR13XX_PMX_MCICDCF1_MASK |
			SPEAR13XX_PMX_MCICDCF2_MASK |
			SPEAR13XX_PMX_MCICDXD_MASK |
			SPEAR13XX_PMX_MCILEDS_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_gpt64_modes[] = {
	{
		.mux_regs = pmx_gpt64_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_gpt64_mux),
	},
};

struct pmx_dev spear1310_pmx_gpt64 = {
	.name = "gpt64",
	.modes = pmx_gpt64_modes,
	.mode_count = ARRAY_SIZE(pmx_gpt64_modes),
};

/* Pad multiplexing for ras_mii_txclk device */
static struct pmx_mux_reg pmx_ras_mii_txclk_mux[] = {
	{
		.address = SPEAR1310_PAD_FUNCTION_EN_2,
		.mask = SPEAR13XX_PMX_NFCE2_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_ras_mii_txclk_modes[] = {
	{
		.mux_regs = pmx_ras_mii_txclk_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_ras_mii_txclk_mux),
	},
};

struct pmx_dev spear1310_pmx_ras_mii_txclk = {
	.name = "ras_mii_txclk",
	.modes = pmx_ras_mii_txclk_modes,
	.mode_count = ARRAY_SIZE(pmx_ras_mii_txclk_modes),
};

/* pad multiplexing for pcie0 device */
static struct pmx_mux_reg pmx_pcie0_mux[] = {
	{
		.address = SPEAR1310_PCIE_SATA_CFG,
		.mask = SPEAR1310_PCIE_CFG_VAL(0),
		.value = SPEAR1310_PCIE_CFG_VAL(0),
	},
};

static struct pmx_dev_mode pmx_pcie0_modes[] = {
	{
		.mux_regs = pmx_pcie0_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_pcie0_mux),
	},
};

struct pmx_dev spear1310_pmx_pcie0 = {
	.name = "pcie0",
	.modes = pmx_pcie0_modes,
	.mode_count = ARRAY_SIZE(pmx_pcie0_modes),
};

/* pad multiplexing for pcie1 device */
static struct pmx_mux_reg pmx_pcie1_mux[] = {
	{
		.address = SPEAR1310_PCIE_SATA_CFG,
		.mask = SPEAR1310_PCIE_CFG_VAL(1),
		.value = SPEAR1310_PCIE_CFG_VAL(1),
	},
};

static struct pmx_dev_mode pmx_pcie1_modes[] = {
	{
		.mux_regs = pmx_pcie1_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_pcie1_mux),
	},
};

struct pmx_dev spear1310_pmx_pcie1 = {
	.name = "pcie1",
	.modes = pmx_pcie1_modes,
	.mode_count = ARRAY_SIZE(pmx_pcie1_modes),
};

/* pad multiplexing for pcie2 device */
static struct pmx_mux_reg pmx_pcie2_mux[] = {
	{
		.address = SPEAR1310_PCIE_SATA_CFG,
		.mask = SPEAR1310_PCIE_CFG_VAL(2),
		.value = SPEAR1310_PCIE_CFG_VAL(2),
	},
};

static struct pmx_dev_mode pmx_pcie2_modes[] = {
	{
		.mux_regs = pmx_pcie2_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_pcie2_mux),
	},
};

struct pmx_dev spear1310_pmx_pcie2 = {
	.name = "pcie2",
	.modes = pmx_pcie2_modes,
	.mode_count = ARRAY_SIZE(pmx_pcie2_modes),
};

/* pad multiplexing for sata0 device */
static struct pmx_mux_reg pmx_sata0_mux[] = {
	{
		.address = SPEAR1310_PCIE_SATA_CFG,
		.mask = SPEAR1310_SATA_CFG_VAL(0),
		.value = SPEAR1310_SATA_CFG_VAL(0),
	},
};

static struct pmx_dev_mode pmx_sata0_modes[] = {
	{
		.mux_regs = pmx_sata0_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_sata0_mux),
	},
};

struct pmx_dev spear1310_pmx_sata0 = {
	.name = "sata0",
	.modes = pmx_sata0_modes,
	.mode_count = ARRAY_SIZE(pmx_sata0_modes),
};

/* pad multiplexing for sata1 device */
static struct pmx_mux_reg pmx_sata1_mux[] = {
	{
		.address = SPEAR1310_PCIE_SATA_CFG,
		.mask = SPEAR1310_SATA_CFG_VAL(1),
		.value = SPEAR1310_SATA_CFG_VAL(1),
	},
};

static struct pmx_dev_mode pmx_sata1_modes[] = {
	{
		.mux_regs = pmx_sata1_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_sata1_mux),
	},
};

struct pmx_dev spear1310_pmx_sata1 = {
	.name = "sata1",
	.modes = pmx_sata1_modes,
	.mode_count = ARRAY_SIZE(pmx_sata1_modes),
};

/* pad multiplexing for sata2 device */
static struct pmx_mux_reg pmx_sata2_mux[] = {
	{
		.address = SPEAR1310_PCIE_SATA_CFG,
		.mask = SPEAR1310_SATA_CFG_VAL(2),
		.value = SPEAR1310_SATA_CFG_VAL(2),
	},
};

static struct pmx_dev_mode pmx_sata2_modes[] = {
	{
		.mux_regs = pmx_sata2_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_sata2_mux),
	},
};

struct pmx_dev spear1310_pmx_sata2 = {
	.name = "sata2",
	.modes = pmx_sata2_modes,
	.mode_count = ARRAY_SIZE(pmx_sata2_modes),
};

/* Pad multiplexing for sdhci device */
static struct pmx_mux_reg pmx_sdhci_mux[] = {
	{
		.address = SPEAR1310_PERIP_CFG,
		.mask = SPEAR1310_MCIF_SEL_MASK << SPEAR1310_MCIF_SEL_SHIFT,
		.value = SPEAR1310_MCIF_SEL_SD << SPEAR1310_MCIF_SEL_SHIFT,
	},
};

static struct pmx_dev_mode pmx_sdhci_modes[] = {
	{
		.mux_regs = pmx_sdhci_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_sdhci_mux),
	},
};

struct pmx_dev spear1310_pmx_sdhci = {
	.name = "sdhci",
	.modes = pmx_sdhci_modes,
	.mode_count = ARRAY_SIZE(pmx_sdhci_modes),
};

/* Pad multiplexing for cf device */
static struct pmx_mux_reg pmx_cf_mux[] = {
	{
		.address = SPEAR1310_PERIP_CFG,
		.mask = SPEAR1310_MCIF_SEL_MASK << SPEAR1310_MCIF_SEL_SHIFT,
		.value = SPEAR1310_MCIF_SEL_CF << SPEAR1310_MCIF_SEL_SHIFT,
	},
};

static struct pmx_dev_mode pmx_cf_modes[] = {
	{
		.mux_regs = pmx_cf_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_cf_mux),
	},
};

struct pmx_dev spear1310_pmx_cf = {
	.name = "cf",
	.modes = pmx_cf_modes,
	.mode_count = ARRAY_SIZE(pmx_cf_modes),
};

/* Pad multiplexing for xd device */
static struct pmx_mux_reg pmx_xd_mux[] = {
	{
		.address = SPEAR1310_PERIP_CFG,
		.mask = SPEAR1310_MCIF_SEL_MASK << SPEAR1310_MCIF_SEL_SHIFT,
		.value = SPEAR1310_MCIF_SEL_XD << SPEAR1310_MCIF_SEL_SHIFT,
	},
};

static struct pmx_dev_mode pmx_xd_modes[] = {
	{
		.mux_regs = pmx_xd_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_xd_mux),
	},
};

struct pmx_dev spear1310_pmx_xd = {
	.name = "xd",
	.modes = pmx_xd_modes,
	.mode_count = ARRAY_SIZE(pmx_xd_modes),
};

/* Add spear1310 specific devices here */
/* ssp1 device registeration */
static struct pl022_ssp_controller ssp1_platform_data = {
	.bus_id = 1,
	.enable_dma = 0,
	/*
	 * This is number of spi devices that can be connected to spi. This
	 * number depends on cs lines supported by soc.
	 */
	.num_chipselect = 3,
};

struct amba_device spear1310_ssp1_device = {
	.dev = {
		.coherent_dma_mask = ~0,
		.init_name = "ssp-pl022.1",
		.platform_data = &ssp1_platform_data,
	},
	.res = {
		.start = SPEAR1310_SSP1_BASE,
		.end = SPEAR1310_SSP1_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	.irq = {SPEAR1310_IRQ_SSP1, NO_IRQ},
};

/* uart1 device registeration */
struct amba_device spear1310_uart1_device = {
	.dev = {
		.init_name = "uart1",
	},
	.res = {
		.start = SPEAR1310_UART1_BASE,
		.end = SPEAR1310_UART1_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	.irq = {SPEAR1310_IRQ_UART1, NO_IRQ},
};

/* uart2 device registeration */
struct amba_device spear1310_uart2_device = {
	.dev = {
		.init_name = "uart2",
	},
	.res = {
		.start = SPEAR1310_UART2_BASE,
		.end = SPEAR1310_UART2_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	.irq = {SPEAR1310_IRQ_UART2, NO_IRQ},
};

/* uart3 device registeration */
struct amba_device spear1310_uart3_device = {
	.dev = {
		.init_name = "uart3",
	},
	.res = {
		.start = SPEAR1310_UART3_BASE,
		.end = SPEAR1310_UART3_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	.irq = {SPEAR1310_IRQ_UART3, NO_IRQ},
};

/* uart4 device registeration */
struct amba_device spear1310_uart4_device = {
	.dev = {
		.init_name = "uart4",
	},
	.res = {
		.start = SPEAR1310_UART4_BASE,
		.end = SPEAR1310_UART4_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	.irq = {SPEAR1310_IRQ_UART4, NO_IRQ},
};

/* uart5 device registeration */
struct amba_device spear1310_uart5_device = {
	.dev = {
		.init_name = "uart5",
	},
	.res = {
		.start = SPEAR1310_UART5_BASE,
		.end = SPEAR1310_UART5_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	.irq = {SPEAR1310_IRQ_UART5, NO_IRQ},
};

/* CAN device registeration */
static struct c_can_platform_data can0_pdata = {
	.is_quirk_required = true,
	.devtype_data = {
		.rx_first = 1,
		.rx_split = 20,
		.rx_last = 26,
		.tx_num = 6,
	},
};

static struct resource can0_resources[] = {
	{
		.start = SPEAR1310_CAN0_BASE,
		.end = SPEAR1310_CAN0_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR1310_IRQ_CAN0,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear1310_can0_device = {
	.name = "c_can_platform",
	.id = 0,
	.num_resources = ARRAY_SIZE(can0_resources),
	.resource = can0_resources,
	.dev.platform_data = &can0_pdata,
};

static struct c_can_platform_data can1_pdata = {
	.is_quirk_required = true,
	.devtype_data = {
		.rx_first = 1,
		.rx_split = 20,
		.rx_last = 26,
		.tx_num = 6,
	},
};

static struct resource can1_resources[] = {
	{
		.start = SPEAR1310_CAN1_BASE,
		.end = SPEAR1310_CAN1_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR1310_IRQ_CAN1,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear1310_can1_device = {
	.name = "c_can_platform",
	.id = 1,
	.num_resources = ARRAY_SIZE(can1_resources),
	.resource = can1_resources,
	.dev.platform_data = &can1_pdata,
};

static int get_i2c_gpio(unsigned gpio_nr)
{
	struct pmx_dev *pmxdev;

	pmxdev = &spear13xx_pmx_i2c;
	/* take I2C SLCK control as pl-gpio */
	config_io_pads(&pmxdev, 1, false);

	return 0;
}

static int put_i2c_gpio(unsigned gpio_nr)
{
	struct pmx_dev *pmxdev;

	pmxdev = &spear13xx_pmx_i2c;
	/* restore I2C SLCK control to I2C controller*/
	config_io_pads(&pmxdev, 1, true);

	return 0;
}

static struct i2c_dw_pdata spear1310_i2c0_dw_pdata = {
	.recover_bus = NULL,
	.scl_gpio = PLGPIO_103,
	.scl_gpio_flags = GPIOF_OUT_INIT_LOW,
	.get_gpio = get_i2c_gpio,
	.put_gpio = put_i2c_gpio,
	.is_gpio_recovery = true,
	.skip_sda_polling = false,
	.sda_gpio = PLGPIO_102,
	.sda_gpio_flags = GPIOF_OUT_INIT_LOW,
};

/* i2c1 device registeration */
static struct resource i2c1_resources[] = {
	{
		.start = SPEAR1310_I2C1_BASE,
		.end = SPEAR1310_I2C1_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR1310_IRQ_I2C1,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear1310_i2c1_device = {
	.name = "i2c_designware",
	.id = 1,
	.dev = {
		.coherent_dma_mask = ~0,
	},
	.num_resources = ARRAY_SIZE(i2c1_resources),
	.resource = i2c1_resources,
};

/* i2c2 device registeration */
static struct resource i2c2_resources[] = {
	{
		.start = SPEAR1310_I2C2_BASE,
		.end = SPEAR1310_I2C2_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR1310_IRQ_I2C2,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear1310_i2c2_device = {
	.name = "i2c_designware",
	.id = 2,
	.dev = {
		.coherent_dma_mask = ~0,
	},
	.num_resources = ARRAY_SIZE(i2c2_resources),
	.resource = i2c2_resources,
};

/* i2c3 device registeration */
static struct resource i2c3_resources[] = {
	{
		.start = SPEAR1310_I2C3_BASE,
		.end = SPEAR1310_I2C3_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR1310_IRQ_I2C3,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear1310_i2c3_device = {
	.name = "i2c_designware",
	.id = 3,
	.dev = {
		.coherent_dma_mask = ~0,
	},
	.num_resources = ARRAY_SIZE(i2c3_resources),
	.resource = i2c3_resources,
};

/* i2c4 device registeration */
static struct resource i2c4_resources[] = {
	{
		.start = SPEAR1310_I2C4_BASE,
		.end = SPEAR1310_I2C4_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR1310_IRQ_I2C4,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear1310_i2c4_device = {
	.name = "i2c_designware",
	.id = 4,
	.dev = {
		.coherent_dma_mask = ~0,
	},
	.num_resources = ARRAY_SIZE(i2c4_resources),
	.resource = i2c4_resources,
};

/* i2c5 device registeration */
static struct resource i2c5_resources[] = {
	{
		.start = SPEAR1310_I2C5_BASE,
		.end = SPEAR1310_I2C5_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR1310_IRQ_I2C5,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear1310_i2c5_device = {
	.name = "i2c_designware",
	.id = 5,
	.dev = {
		.coherent_dma_mask = ~0,
	},
	.num_resources = ARRAY_SIZE(i2c5_resources),
	.resource = i2c5_resources,
};

/* i2c6 device registeration */
static struct resource i2c6_resources[] = {
	{
		.start = SPEAR1310_I2C6_BASE,
		.end = SPEAR1310_I2C6_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR1310_IRQ_I2C6,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear1310_i2c6_device = {
	.name = "i2c_designware",
	.id = 6,
	.dev = {
		.coherent_dma_mask = ~0,
	},
	.num_resources = ARRAY_SIZE(i2c6_resources),
	.resource = i2c6_resources,
};

/* i2c7 device registeration */
static struct resource i2c7_resources[] = {
	{
		.start = SPEAR1310_I2C7_BASE,
		.end = SPEAR1310_I2C7_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR1310_IRQ_I2C7,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear1310_i2c7_device = {
	.name = "i2c_designware",
	.id = 7,
	.dev = {
		.coherent_dma_mask = ~0,
	},
	.num_resources = ARRAY_SIZE(i2c7_resources),
	.resource = i2c7_resources,
};

/* plgpio */
static struct plgpio_platform_data plgpio_plat_data = {
	.gpio_base = 16,
	.irq_base = SPEAR_PLGPIO_INT_BASE,
	.gpio_count = SPEAR_PLGPIO_COUNT,
	.regs = {
		.enb = SPEAR1310_PLGPIO_ENB_OFF,
		.wdata = SPEAR1310_PLGPIO_WDATA_OFF,
		.dir = SPEAR1310_PLGPIO_DIR_OFF,
		.rdata = SPEAR1310_PLGPIO_RDATA_OFF,
		.ie = SPEAR1310_PLGPIO_IE_OFF,
		.mis = SPEAR1310_PLGPIO_MIS_OFF,
		.eit = SPEAR1310_PLGPIO_EIT_OFF,
	},
};

static struct resource plgpio_resources[] = {
	{
		.start = SPEAR1310_RAS_BASE,
		.end = SPEAR1310_RAS_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR1310_IRQ_PLGPIO,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear1310_plgpio_device = {
	.name = "plgpio",
	.id = -1,
	.dev = {
		.platform_data = &plgpio_plat_data,
	},
	.num_resources = ARRAY_SIZE(plgpio_resources),
	.resource = plgpio_resources,
};

static struct tdm_hdlc_platform_data tdm_hdlc_0_plat_data = {
	.ip_type = SPEAR1310_TDM_HDLC,
	.nr_channel = 2,
	.nr_timeslot = 128,
};

static struct resource tdm_hdlc_0_resources[] = {
	{
		.start = SPEAR1310_TDM_E1_0_BASE,
		.end = SPEAR1310_TDM_E1_0_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR1310_IRQ_TDM0,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear1310_tdm_hdlc_0_device = {
	.name = "tdm_hdlc",
	.id = 0,
	.dev = {
		.platform_data = &tdm_hdlc_0_plat_data,
		.coherent_dma_mask = ~0,
	},
	.num_resources = ARRAY_SIZE(tdm_hdlc_0_resources),
	.resource = tdm_hdlc_0_resources,
};

static struct tdm_hdlc_platform_data tdm_hdlc_1_plat_data = {
	.ip_type = SPEAR1310_TDM_HDLC,
	.nr_channel = 2,
	.nr_timeslot = 128,
};

static struct resource tdm_hdlc_1_resources[] = {
	{
		.start = SPEAR1310_TDM_E1_1_BASE,
		.end = SPEAR1310_TDM_E1_1_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR1310_IRQ_TDM1,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear1310_tdm_hdlc_1_device = {
	.name = "tdm_hdlc",
	.id = 1,
	.dev = {
		.platform_data = &tdm_hdlc_1_plat_data,
		.coherent_dma_mask = ~0,
	},
	.num_resources = ARRAY_SIZE(tdm_hdlc_1_resources),
	.resource = tdm_hdlc_1_resources,
};

static struct rs485_hdlc_platform_data rs485_0_plat_data = {
	.tx_falling_edge = 1,
	.rx_rising_edge = 1,
	.cts_enable = 1,
	.cts_delay = 50,
};

static struct rs485_hdlc_platform_data rs485_1_plat_data = {
	.tx_falling_edge = 1,
	.rx_rising_edge = 1,
	.cts_enable = 1,
	.cts_delay = 50,
};

static struct resource rs485_0_resources[] = {
	{
		.start = SPEAR1310_RS485_0_BASE,
		.end = SPEAR1310_RS485_0_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR1310_IRQ_RS485_0,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource rs485_1_resources[] = {
	{
		.start = SPEAR1310_RS485_1_BASE,
		.end = SPEAR1310_RS485_1_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR1310_IRQ_RS485_1,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear1310_rs485_0_device = {
	.name = "rs485_hdlc",
	.id = 0,
	.dev = {
		.platform_data = &rs485_0_plat_data,
		.coherent_dma_mask = ~0,
	},
	.num_resources = ARRAY_SIZE(rs485_0_resources),
	.resource = rs485_0_resources,
};

struct platform_device spear1310_rs485_1_device = {
	.name = "rs485_hdlc",
	.id = 1,
	.dev = {
		.platform_data = &rs485_1_plat_data,
		.coherent_dma_mask = ~0,
	},
	.num_resources = ARRAY_SIZE(rs485_1_resources),
	.resource = rs485_1_resources,
};

static void sata_miphy_exit(struct device *dev)
{
	u32 temp;

	/*TBD: Workaround to support multiple sata port.*/
	temp = readl(VA_SPEAR1310_PCIE_SATA_CFG);
	temp &= ~SPEAR1310_SATA_CFG_MASK(0);

	writel(temp, VA_SPEAR1310_PCIE_SATA_CFG);
	writel(~SPEAR1310_PCIE_SATA_MIPHY_CFG_SATA_MASK,
			VA_SPEAR1310_PCIE_MIPHY_CFG_1);

	/* Enable PCIE SATA Controller reset */
	writel((readl(VA_SPEAR1310_PERIP1_SW_RST) | (0x1000)),
			VA_SPEAR1310_PERIP1_SW_RST);
	msleep(20);
	/* Switch off sata power domain */
	writel((readl(VA_SPEAR1310_PCM_CFG) & (~0x800)),
			VA_SPEAR1310_PCM_CFG);
	msleep(20);
}


static int sata_miphy_init(struct device *dev, void __iomem *addr)
{
	u32 temp;

	/*TBD: Workaround to support multiple sata port.*/
	temp = readl(VA_SPEAR1310_PCIE_SATA_CFG);
	temp &= ~SPEAR1310_SATA_CFG_MASK(0);
	temp |= SPEAR1310_SATA_CFG_VAL(0);

	writel(temp, VA_SPEAR1310_PCIE_SATA_CFG);

	temp = readl(VA_SPEAR1310_PCIE_MIPHY_CFG_1);
	temp &= ~SPEAR1310_PCIE_SATA_MIPHY_CFG_SATA_MASK;
	temp |= SPEAR1310_PCIE_SATA_MIPHY_CFG_SATA_25M_CRYSTAL_CLK;

	writel(temp, VA_SPEAR1310_PCIE_MIPHY_CFG_1);
	/* Switch on sata power domain */
	writel((readl(VA_SPEAR1310_PCM_CFG) | (0x800)),
			VA_SPEAR1310_PCM_CFG);
	msleep(20);
	/* Disable PCIE SATA Controller reset */
	writel((readl(VA_SPEAR1310_PERIP1_SW_RST) & (~0x1000)),
			VA_SPEAR1310_PERIP1_SW_RST);
	msleep(20);

	return 0;
}

static struct ahci_platform_data sata_pdata = {
	.init = sata_miphy_init,
	.exit = sata_miphy_exit,
};

static u64 ahci_dmamask = DMA_BIT_MASK(32);

/* SATA0 device registration */
static struct resource sata0_resources[] = {
	{
		.start = SPEAR1310_SATA0_BASE,
		.end = SPEAR1310_SATA0_BASE + SZ_8K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR1310_IRQ_SATA0,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear1310_sata0_device = {
	.name = "ahci",
	.id = 0,
	.num_resources = ARRAY_SIZE(sata0_resources),
	.resource = sata0_resources,
	.dev = {
		.platform_data = &sata_pdata,
		.dma_mask = &ahci_dmamask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};

/* SATA1 device registration */
static struct resource sata1_resources[] = {
	{
		.start = SPEAR1310_SATA1_BASE,
		.end = SPEAR1310_SATA1_BASE + SZ_8K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR1310_IRQ_SATA1,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear1310_sata1_device = {
	.name = "ahci",
	.id = 1,
	.num_resources = ARRAY_SIZE(sata1_resources),
	.resource = sata1_resources,
	.dev = {
		.platform_data = &sata_pdata,
		.dma_mask = &ahci_dmamask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};

/* SATA2 device registration */
static struct resource sata2_resources[] = {
	{
		.start = SPEAR1310_SATA2_BASE,
		.end = SPEAR1310_SATA2_BASE + SZ_8K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR1310_IRQ_SATA2,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear1310_sata2_device = {
	.name = "ahci",
	.id = 2,
	.num_resources = ARRAY_SIZE(sata2_resources),
	.resource = sata2_resources,
	.dev = {
		.platform_data = &sata_pdata,
		.dma_mask = &ahci_dmamask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};

/* OTG device registration */
static struct dwc_otg_plat_data otg_platform_data = {
	.phy_init = otg_phy_init,
	.param_init = otg_param_init,
};

static struct resource otg_resources[] = {
	{
		.start = SPEAR1310_UOC_BASE,
		.end = SPEAR1310_UOC_BASE + SZ_256K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR1310_IRQ_UOC,
		.flags = IORESOURCE_IRQ,
	},
};

static u64 otg_dmamask = ~0;
struct platform_device spear1310_otg_device = {
	.name = "dwc_otg",
	.id = -1,
	.dev = {
		.coherent_dma_mask = ~0,
		.dma_mask = &otg_dmamask,
		.platform_data = &otg_platform_data,
	},
	.num_resources = ARRAY_SIZE(otg_resources),
	.resource = otg_resources,
};

/* nand device registeration */
static void spear1310_nand_select_bank(u32 bank, u32 busw)
{
	u32 fsmc_cfg = readl(VA_SPEAR1310_FSMC_CFG);

	fsmc_cfg &= ~(SPEAR1310_FSMC_CS_MEMSEL_MASK << (bank & 3));

	writel(fsmc_cfg, VA_SPEAR1310_FSMC_CFG);
}
/* nand device registeration */
static struct fsmc_nand_platform_data nand_platform_data = {
	.select_bank = spear1310_nand_select_bank,
	.mode = USE_WORD_ACCESS,
	.read_dma_priv = &nand_read_dma_priv,
	.write_dma_priv = &nand_write_dma_priv,
};

static struct resource nand_resources[] = {
	{
		.name = "nand_data",
		.start = SPEAR1310_NAND_MEM_BASE,
		.end = SPEAR1310_NAND_MEM_BASE + SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.name = "fsmc_regs",
		.start = SPEAR13XX_FSMC_BASE,
		.end = SPEAR13XX_FSMC_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
};

struct platform_device spear1310_nand_device = {
	.name = "fsmc-nand",
	.id = -1,
	.resource = nand_resources,
	.num_resources = ARRAY_SIZE(nand_resources),
	.dev.platform_data = &nand_platform_data,
};

static void tdm_hdlc_setup(void)
{
	struct clk *synth_clk, *vco_clk, *tdm0_clk, *tdm1_clk;
	char *synth_clk_name = "ras_synth1_clk";
	char *vco_clk_name = "vco1div4_clk";
	int ret;

	vco_clk = clk_get(NULL, vco_clk_name);
	if (IS_ERR(vco_clk)) {
		pr_err("Failed to get %s\n", vco_clk_name);
		return;
	}

	synth_clk = clk_get(NULL, synth_clk_name);
	if (IS_ERR(synth_clk)) {
		pr_err("Failed to get %s\n", synth_clk_name);
		goto free_vco_clk;
	}

	/* use vco1div4 source for gen_clk_synt1 */
	ret = clk_set_parent(synth_clk, vco_clk);
	if (ret < 0) {
		pr_err("Failed to set parent %s to %s\n", vco_clk_name,
				synth_clk_name);
		goto free_synth_clk;
	}

	/* select gen_clk_synt1 as source for TDM */
	tdm0_clk = clk_get_sys(NULL, "tdm_hdlc.0");
	if (IS_ERR(tdm0_clk)) {
		pr_err("Failed to get tdm 0 clock\n");
		goto free_synth_clk;
	}
	/* select gen_clk_synt1 as source for TDM */
	tdm1_clk = clk_get_sys(NULL, "tdm_hdlc.1");
	if (IS_ERR(tdm1_clk)) {
		pr_err("Failed to get tdm 1 clock\n");
		goto free_tdm0_clk;
	}

	ret = clk_set_parent(tdm0_clk, synth_clk);
	if (ret < 0) {
		pr_err("Failed to set parent %s to tdm0 clock\n",
				synth_clk_name);
		goto free_tdm1_clk;
	}
	ret = clk_set_parent(tdm1_clk, synth_clk);
	if (ret < 0) {
		pr_err("Failed to set parent %s to tdm1 clock\n",
				synth_clk_name);
		goto free_tdm1_clk;
	}

free_tdm1_clk:
	clk_put(tdm1_clk);
free_tdm0_clk:
	clk_put(tdm0_clk);
free_synth_clk:
	clk_put(synth_clk);
free_vco_clk:
	clk_put(vco_clk);
}

/* Following will create 1310 specific static virtual/physical mappings */
static struct map_desc spear1310_io_desc[] __initdata = {
	{
		.virtual	= IO_ADDRESS(SPEAR1310_RAS_BASE),
		.pfn		= __phys_to_pfn(SPEAR1310_RAS_BASE),
		.length		= SZ_4K,
		.type		= MT_DEVICE
	},
};

/* This will create static memory mapping for selected devices */
void __init spear1310_map_io(void)
{
	iotable_init(spear1310_io_desc,
			ARRAY_SIZE(spear1310_io_desc));
	spear13xx_map_io();
}

void __init spear1310_init(struct pmx_mode *pmx_mode,
		struct pmx_dev **pmx_devs, u8 pmx_dev_count)
{
	int ret;

	/* call spear13xx family common init function */
	spear13xx_init();

	tdm_hdlc_setup();

	spear13xx_i2c_device.dev.platform_data = &spear1310_i2c0_dw_pdata;

	/* pmx initialization */
	pmx_driver.mode = pmx_mode;
	pmx_driver.devs = pmx_devs;
	pmx_driver.devs_count = pmx_dev_count;

	ret = pmx_register(&pmx_driver);
	if (ret)
		pr_err("padmux: registeration failed. err no: %d\n", ret);
}

int spear1310_pcie_clk_init(struct pcie_port *pp)
{
	u32 temp;

	temp = readl(VA_SPEAR1310_PCIE_MIPHY_CFG_1);
	temp &= ~SPEAR1310_PCIE_SATA_MIPHY_CFG_PCIE_MASK;
	temp |= SPEAR1310_PCIE_SATA_MIPHY_CFG_PCIE;

	writel(temp, VA_SPEAR1310_PCIE_MIPHY_CFG_1);

	temp = readl(VA_SPEAR1310_PCIE_SATA_CFG);

	switch (pp->port) {
	case 0:
		temp &= ~SPEAR1310_PCIE_CFG_MASK(0);
		temp |= SPEAR1310_PCIE_CFG_VAL(0);
		break;
	case 1:
		temp &= ~SPEAR1310_PCIE_CFG_MASK(1);
		temp |= SPEAR1310_PCIE_CFG_VAL(1);
		break;
	case 2:
		temp &= ~SPEAR1310_PCIE_CFG_MASK(2);
		temp |= SPEAR1310_PCIE_CFG_VAL(2);
		break;
	default:
		return -EINVAL;
	}
	writel(temp, VA_SPEAR1310_PCIE_SATA_CFG);

	return 0;
}

int spear1310_pcie_clk_exit(struct pcie_port *pp)
{
	u32 temp;

	temp = readl(VA_SPEAR1310_PCIE_SATA_CFG);

	switch (pp->port) {
	case 0:
		temp &= ~SPEAR1310_PCIE_CFG_MASK(0);
		break;
	case 1:
		temp &= ~SPEAR1310_PCIE_CFG_MASK(1);
		break;
	case 2:
		temp &= ~SPEAR1310_PCIE_CFG_MASK(2);
		break;
	}

	writel(temp, VA_SPEAR1310_PCIE_SATA_CFG);
	writel(~SPEAR1310_PCIE_SATA_MIPHY_CFG_PCIE_MASK,
			VA_SPEAR1310_PCIE_MIPHY_CFG_1);

	return 0;
}
