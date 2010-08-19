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
#include <linux/mtd/physmap.h>
#include <linux/ptrace.h>
#include <linux/types.h>
#include <linux/mmc/sdhci-spear.h>
#include <linux/mtd/fsmc.h>
#include <asm/irq.h>
#include <mach/generic.h>
#include <mach/spear.h>
#include <plat/gpio.h>
#include <plat/shirq.h>

/* modes */
#define AUTO_NET_SMII_MODE	(1 << 0)
#define AUTO_NET_MII_MODE	(1 << 1)
#define AUTO_EXP_MODE		(1 << 2)
#define SMALL_PRINTERS_MODE	(1 << 3)
#define ALL_MODES		0xF

struct pmx_mode auto_net_smii_mode = {
	.id = AUTO_NET_SMII_MODE,
	.name = "Automation Networking SMII Mode",
	.value = 0x00,
};

struct pmx_mode auto_net_mii_mode = {
	.id = AUTO_NET_MII_MODE,
	.name = "Automation Networking MII Mode",
	.value = 0x01,
};

struct pmx_mode auto_exp_mode = {
	.id = AUTO_EXP_MODE,
	.name = "Automation Expanded Mode",
	.value = 0x02,
};

struct pmx_mode small_printers_mode = {
	.id = SMALL_PRINTERS_MODE,
	.name = "Small Printers Mode",
	.value = 0x03,
};

/* devices */
/* Pad multiplexing for CLCD device */
static struct pmx_mux_reg pmx_clcd_mux[] = {
	{
		.mask = 0x0,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_clcd_modes[] = {
	{
		.ids = AUTO_NET_SMII_MODE,
		.mux_regs = pmx_clcd_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_clcd_mux),
	},
};

struct pmx_dev pmx_clcd = {
	.name = "clcd",
	.modes = pmx_clcd_modes,
	.mode_count = ARRAY_SIZE(pmx_clcd_modes),
};

/* Pad multiplexing for EMI (Parallel NOR flash) device */
static struct pmx_mux_reg pmx_emi_mux[] = {
	{
		.mask = PMX_TIMER_1_2_MASK | PMX_TIMER_3_4_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_emi_modes[] = {
	{
		.ids = AUTO_EXP_MODE,
		.mux_regs = pmx_emi_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_emi_mux),
	},
};

struct pmx_dev pmx_emi = {
	.name = "emi",
	.modes = pmx_emi_modes,
	.mode_count = ARRAY_SIZE(pmx_emi_modes),
};

/* Pad multiplexing for FSMC (NAND flash) device */
static struct pmx_mux_reg pmx_fsmc_mux[] = {
	{
		.mask = 0x0,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_fsmc_modes[] = {
	{
		.ids = ALL_MODES,
		.mux_regs = pmx_fsmc_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_fsmc_mux),
	},
};

struct pmx_dev pmx_fsmc = {
	.name = "fsmc",
	.modes = pmx_fsmc_modes,
	.mode_count = ARRAY_SIZE(pmx_fsmc_modes),
};

/* Pad multiplexing for SPP device */
static struct pmx_mux_reg pmx_spp_mux[] = {
	{
		.mask = 0x0,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_spp_modes[] = {
	{
		.ids = SMALL_PRINTERS_MODE,
		.mux_regs = pmx_spp_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_spp_mux),
	},
};

struct pmx_dev pmx_spp = {
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

static struct pmx_dev_mode pmx_sdhci_modes[] = {
	{
		.ids = AUTO_NET_SMII_MODE | AUTO_NET_MII_MODE |
			SMALL_PRINTERS_MODE,
		.mux_regs = pmx_sdhci_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_sdhci_mux),
	},
};

struct pmx_dev pmx_sdhci = {
	.name = "sdhci",
	.modes = pmx_sdhci_modes,
	.mode_count = ARRAY_SIZE(pmx_sdhci_modes),
};

/* Pad multiplexing for I2S device */
static struct pmx_mux_reg pmx_i2s_mux[] = {
	{
		.mask = PMX_UART0_MODEM_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_i2s_modes[] = {
	{
		.ids = AUTO_NET_SMII_MODE | AUTO_NET_MII_MODE,
		.mux_regs = pmx_i2s_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_i2s_mux),
	},
};

struct pmx_dev pmx_i2s = {
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

static struct pmx_dev_mode pmx_uart1_modes[] = {
	{
		.ids = ALL_MODES,
		.mux_regs = pmx_uart1_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_uart1_mux),
	},
};

struct pmx_dev pmx_uart1 = {
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

static struct pmx_dev_mode pmx_uart1_modem_modes[] = {
	{
		.ids = AUTO_EXP_MODE,
		.mux_regs = pmx_uart1_modem_autoexp_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_uart1_modem_autoexp_mux),
	}, {
		.ids = SMALL_PRINTERS_MODE,
		.mux_regs = pmx_uart1_modem_smallpri_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_uart1_modem_smallpri_mux),
	},
};

struct pmx_dev pmx_uart1_modem = {
	.name = "uart1_modem",
	.modes = pmx_uart1_modem_modes,
	.mode_count = ARRAY_SIZE(pmx_uart1_modem_modes),
};

/* Pad multiplexing for UART2 device */
static struct pmx_mux_reg pmx_uart2_mux[] = {
	{
		.mask = PMX_FIRDA_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_uart2_modes[] = {
	{
		.ids = ALL_MODES,
		.mux_regs = pmx_uart2_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_uart2_mux),
	},
};

struct pmx_dev pmx_uart2 = {
	.name = "uart2",
	.modes = pmx_uart2_modes,
	.mode_count = ARRAY_SIZE(pmx_uart2_modes),
};

/* Pad multiplexing for Touchscreen device */
static struct pmx_mux_reg pmx_touchscreen_mux[] = {
	{
		.mask = PMX_SSP_CS_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_touchscreen_modes[] = {
	{
		.ids = AUTO_NET_SMII_MODE,
		.mux_regs = pmx_touchscreen_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_touchscreen_mux),
	},
};

struct pmx_dev pmx_touchscreen = {
	.name = "touchscreen",
	.modes = pmx_touchscreen_modes,
	.mode_count = ARRAY_SIZE(pmx_touchscreen_modes),
};

/* Pad multiplexing for CAN device */
static struct pmx_mux_reg pmx_can_mux[] = {
	{
		.mask = PMX_GPIO_PIN2_MASK | PMX_GPIO_PIN3_MASK |
			PMX_GPIO_PIN4_MASK | PMX_GPIO_PIN5_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_can_modes[] = {
	{
		.ids = AUTO_NET_SMII_MODE | AUTO_NET_MII_MODE | AUTO_EXP_MODE,
		.mux_regs = pmx_can_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_can_mux),
	},
};

struct pmx_dev pmx_can = {
	.name = "can",
	.modes = pmx_can_modes,
	.mode_count = ARRAY_SIZE(pmx_can_modes),
};

/* Pad multiplexing for SDHCI LED device */
static struct pmx_mux_reg pmx_sdhci_led_mux[] = {
	{
		.mask = PMX_SSP_CS_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_sdhci_led_modes[] = {
	{
		.ids = AUTO_NET_SMII_MODE | AUTO_NET_MII_MODE,
		.mux_regs = pmx_sdhci_led_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_sdhci_led_mux),
	},
};

struct pmx_dev pmx_sdhci_led = {
	.name = "sdhci_led",
	.modes = pmx_sdhci_led_modes,
	.mode_count = ARRAY_SIZE(pmx_sdhci_led_modes),
};

/* Pad multiplexing for PWM0 device */
static struct pmx_mux_reg pmx_pwm0_net_mux[] = {
	{
		.mask = PMX_UART0_MODEM_MASK,
		.value = 0,
	},
};

static struct pmx_mux_reg pmx_pwm0_autoexpsmallpri_mux[] = {
	{
		.mask = PMX_MII_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_pwm0_modes[] = {
	{
		.ids = AUTO_NET_SMII_MODE | AUTO_NET_MII_MODE,
		.mux_regs = pmx_pwm0_net_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_pwm0_net_mux),
	}, {
		.ids = AUTO_EXP_MODE | SMALL_PRINTERS_MODE,
		.mux_regs = pmx_pwm0_autoexpsmallpri_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_pwm0_autoexpsmallpri_mux),
	},
};

struct pmx_dev pmx_pwm0 = {
	.name = "pwm0",
	.modes = pmx_pwm0_modes,
	.mode_count = ARRAY_SIZE(pmx_pwm0_modes),
};

/* Pad multiplexing for PWM1 device */
static struct pmx_mux_reg pmx_pwm1_net_mux[] = {
	{
		.mask = PMX_UART0_MODEM_MASK,
		.value = 0,
	},
};

static struct pmx_mux_reg pmx_pwm1_autoexpsmallpri_mux[] = {
	{
		.mask = PMX_MII_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_pwm1_modes[] = {
	{
		.ids = AUTO_NET_SMII_MODE | AUTO_NET_MII_MODE,
		.mux_regs = pmx_pwm1_net_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_pwm1_net_mux),
	}, {
		.ids = AUTO_EXP_MODE | SMALL_PRINTERS_MODE,
		.mux_regs = pmx_pwm1_autoexpsmallpri_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_pwm1_autoexpsmallpri_mux),
	},
};

struct pmx_dev pmx_pwm1 = {
	.name = "pwm1",
	.modes = pmx_pwm1_modes,
	.mode_count = ARRAY_SIZE(pmx_pwm1_modes),
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

static struct pmx_dev_mode pmx_pwm2_modes[] = {
	{
		.ids = AUTO_NET_SMII_MODE | AUTO_NET_MII_MODE,
		.mux_regs = pmx_pwm2_net_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_pwm2_net_mux),
	}, {
		.ids = AUTO_EXP_MODE | SMALL_PRINTERS_MODE,
		.mux_regs = pmx_pwm2_autoexpsmallpri_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_pwm2_autoexpsmallpri_mux),
	},
};

struct pmx_dev pmx_pwm2 = {
	.name = "pwm2",
	.modes = pmx_pwm2_modes,
	.mode_count = ARRAY_SIZE(pmx_pwm2_modes),
};

/* Pad multiplexing for PWM3 device */
static struct pmx_mux_reg pmx_pwm3_mux[] = {
	{
		.mask = PMX_MII_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_pwm3_modes[] = {
	{
		.ids = AUTO_EXP_MODE | SMALL_PRINTERS_MODE | AUTO_NET_SMII_MODE,
		.mux_regs = pmx_pwm3_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_pwm3_mux),
	},
};

struct pmx_dev pmx_pwm3 = {
	.name = "pwm3",
	.modes = pmx_pwm3_modes,
	.mode_count = ARRAY_SIZE(pmx_pwm3_modes),
};

/* Pad multiplexing for SSP1 device */
static struct pmx_mux_reg pmx_ssp1_mux[] = {
	{
		.mask = PMX_MII_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_ssp1_modes[] = {
	{
		.ids = SMALL_PRINTERS_MODE | AUTO_NET_SMII_MODE,
		.mux_regs = pmx_ssp1_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_ssp1_mux),
	},
};

struct pmx_dev pmx_ssp1 = {
	.name = "ssp1",
	.modes = pmx_ssp1_modes,
	.mode_count = ARRAY_SIZE(pmx_ssp1_modes),
};

/* Pad multiplexing for SSP2 device */
static struct pmx_mux_reg pmx_ssp2_mux[] = {
	{
		.mask = PMX_MII_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_ssp2_modes[] = {
	{
		.ids = AUTO_NET_SMII_MODE,
		.mux_regs = pmx_ssp2_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_ssp2_mux),
	},
};

struct pmx_dev pmx_ssp2 = {
	.name = "ssp2",
	.modes = pmx_ssp2_modes,
	.mode_count = ARRAY_SIZE(pmx_ssp2_modes),
};

/* Pad multiplexing for mii1 device */
static struct pmx_mux_reg pmx_mii1_mux[] = {
	{
		.mask = 0x0,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_mii1_modes[] = {
	{
		.ids = AUTO_NET_MII_MODE,
		.mux_regs = pmx_mii1_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_mii1_mux),
	},
};

struct pmx_dev pmx_mii1 = {
	.name = "mii1",
	.modes = pmx_mii1_modes,
	.mode_count = ARRAY_SIZE(pmx_mii1_modes),
};

/* Pad multiplexing for smii0 device */
static struct pmx_mux_reg pmx_smii0_mux[] = {
	{
		.mask = PMX_MII_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_smii0_modes[] = {
	{
		.ids = AUTO_NET_SMII_MODE | AUTO_EXP_MODE | SMALL_PRINTERS_MODE,
		.mux_regs = pmx_smii0_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_smii0_mux),
	},
};

struct pmx_dev pmx_smii0 = {
	.name = "smii0",
	.modes = pmx_smii0_modes,
	.mode_count = ARRAY_SIZE(pmx_smii0_modes),
};

/* Pad multiplexing for smii1 device */
static struct pmx_mux_reg pmx_smii1_mux[] = {
	{
		.mask = PMX_MII_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_smii1_modes[] = {
	{
		.ids = AUTO_NET_SMII_MODE | SMALL_PRINTERS_MODE,
		.mux_regs = pmx_smii1_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_smii1_mux),
	},
};

struct pmx_dev pmx_smii1 = {
	.name = "smii1",
	.modes = pmx_smii1_modes,
	.mode_count = ARRAY_SIZE(pmx_smii1_modes),
};

/* Pad multiplexing for i2c1 device */
static struct pmx_mux_reg pmx_i2c1_mux[] = {
	{
		.mask = 0x0,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_i2c1_modes[] = {
	{
		.ids = AUTO_EXP_MODE,
		.mux_regs = pmx_i2c1_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_i2c1_mux),
	},
};

struct pmx_dev pmx_i2c1 = {
	.name = "i2c1",
	.modes = pmx_i2c1_modes,
	.mode_count = ARRAY_SIZE(pmx_i2c1_modes),
};

/* pmx driver structure */
struct pmx_driver pmx_driver = {
	.mode_reg = {.address = SPEAR320_MODE_CONFIG_REG, .mask = 0x00000007},
};

/* Add spear320 specific devices here */
/* CLCD device registration */
struct amba_device clcd_device = {
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
	.irq = {VIRQ_CLCD, NO_IRQ},
};

/* ssp device registeration */
static struct pl022_ssp_controller ssp_platform_data[] = {
	{
		.bus_id = 1,
		.enable_dma = 0,
		.num_chipselect = 2,
	}, {
		.bus_id = 2,
		.enable_dma = 0,
		.num_chipselect = 2,
	}
};

struct amba_device ssp_device[] = {
	{
		.dev = {
			.coherent_dma_mask = ~0,
			.init_name = "ssp-pl022.1",
			.platform_data = &ssp_platform_data[0],
		},
		.res = {
			.start = SPEAR320_SSP0_BASE,
			.end = SPEAR320_SSP0_BASE + SZ_4K - 1,
			.flags = IORESOURCE_MEM,
		},
		.irq = {VIRQ_SSP1, NO_IRQ},
	}, {
		.dev = {
			.coherent_dma_mask = ~0,
			.init_name = "ssp-pl022.2",
			.platform_data = &ssp_platform_data[1],
		},
		.res = {
			.start = SPEAR320_SSP1_BASE,
			.end = SPEAR320_SSP1_BASE + SZ_4K - 1,
			.flags = IORESOURCE_MEM,
		},
		.irq = {VIRQ_SSP2, NO_IRQ},
	}
};

/* uart1 device registeration */
struct amba_device uart1_device = {
	.dev = {
		.init_name = "uart1",
	},
	.res = {
		.start = SPEAR320_UART1_BASE,
		.end = SPEAR320_UART1_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	.irq = {VIRQ_UART1, NO_IRQ},
};

/* uart2 device registeration */
struct amba_device uart2_device = {
	.dev = {
		.init_name = "uart2",
	},
	.res = {
		.start = SPEAR320_UART2_BASE,
		.end = SPEAR320_UART2_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	.irq = {VIRQ_UART2, NO_IRQ},
};

/* CAN device registeration */
static struct resource can0_resources[] = {
	{
		.start = SPEAR320_CAN0_BASE,
		.end = SPEAR320_CAN0_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM | IORESOURCE_MEM_32BIT,
	}, {
		.start = VIRQ_CANU,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device can0_device = {
	.name = "c_can_platform",
	.id = 0,
	.num_resources = ARRAY_SIZE(can0_resources),
	.resource = can0_resources,
};

static struct resource can1_resources[] = {
	{
		.start = SPEAR320_CAN1_BASE,
		.end = SPEAR320_CAN1_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM | IORESOURCE_MEM_32BIT,
	}, {
		.start = VIRQ_CANL,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device can1_device = {
	.name = "c_can_platform",
	.id = 1,
	.num_resources = ARRAY_SIZE(can1_resources),
	.resource = can1_resources,
};

/* emi nor flash device registeration */
static struct physmap_flash_data emi_norflash_data;
struct platform_device emi_nor_device = {
	.name	= "physmap-flash",
	.id	= -1,
	.dev.platform_data = &emi_norflash_data,
};

/* i2c1 device registeration */
static struct resource i2c1_resources[] = {
	{
		.start = SPEAR320_I2C_BASE,
		.end = SPEAR320_I2C_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = VIRQ_I2C1 ,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device i2c1_device = {
	.name = "i2c_designware",
	.id = 1,
	.dev = {
		.coherent_dma_mask = ~0,
	},
	.num_resources = ARRAY_SIZE(i2c1_resources),
	.resource = i2c1_resources,
};

/* nand device registeration */
static struct fsmc_nand_platform_data nand_platform_data;

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

struct platform_device nand_device = {
	.name = "fsmc-nand",
	.id = -1,
	.resource = nand_resources,
	.num_resources = ARRAY_SIZE(nand_resources),
	.dev.platform_data = &nand_platform_data,
};

/* plgpio device registeration */
static struct plgpio_platform_data plgpio_plat_data = {
	.gpio_base = 8,
	.irq_base = SPEAR_PLGPIO_INT_BASE,
	.gpio_count = SPEAR_PLGPIO_COUNT,
};

static struct resource plgpio_resources[] = {
	{
		.start = SPEAR320_SOC_CONFIG_BASE,
		.end = SPEAR320_SOC_CONFIG_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = VIRQ_PLGPIO,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device plgpio_device = {
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

struct platform_device pwm_device = {
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
		.start	= IRQ_SDHCI,
		.flags	= IORESOURCE_IRQ,
	}
};

struct platform_device sdhci_device = {
	.dev = {
		.coherent_dma_mask = ~0,
	},
	.name = "sdhci",
	.id = -1,
	.num_resources = ARRAY_SIZE(sdhci_resources),
	.resource = sdhci_resources,
};

/* spear3xx shared irq */
struct shirq_dev_config shirq_ras1_config[] = {
	{
		.virq = VIRQ_EMI,
		.status_mask = EMI_IRQ_MASK,
		.clear_mask = EMI_IRQ_MASK,
	}, {
		.virq = VIRQ_CLCD,
		.status_mask = CLCD_IRQ_MASK,
		.clear_mask = CLCD_IRQ_MASK,
	}, {
		.virq = VIRQ_SPP,
		.status_mask = SPP_IRQ_MASK,
		.clear_mask = SPP_IRQ_MASK,
	},
};

struct spear_shirq shirq_ras1 = {
	.irq = IRQ_GEN_RAS_1,
	.dev_config = shirq_ras1_config,
	.dev_count = ARRAY_SIZE(shirq_ras1_config),
	.regs = {
		.enb_reg = -1,
		.status_reg = INT_STS_MASK_REG,
		.status_reg_mask = SHIRQ_RAS1_MASK,
		.clear_reg = INT_CLR_MASK_REG,
		.reset_to_clear = 1,
	},
};

struct shirq_dev_config shirq_ras3_config[] = {
	{
		.virq = VIRQ_PLGPIO,
		.enb_mask = GPIO_IRQ_MASK,
		.status_mask = GPIO_IRQ_MASK,
		.clear_mask = GPIO_IRQ_MASK,
	}, {
		.virq = VIRQ_I2S_PLAY,
		.enb_mask = I2S_PLAY_IRQ_MASK,
		.status_mask = I2S_PLAY_IRQ_MASK,
		.clear_mask = I2S_PLAY_IRQ_MASK,
	}, {
		.virq = VIRQ_I2S_REC,
		.enb_mask = I2S_REC_IRQ_MASK,
		.status_mask = I2S_REC_IRQ_MASK,
		.clear_mask = I2S_REC_IRQ_MASK,
	},
};

struct spear_shirq shirq_ras3 = {
	.irq = IRQ_GEN_RAS_3,
	.dev_config = shirq_ras3_config,
	.dev_count = ARRAY_SIZE(shirq_ras3_config),
	.regs = {
		.enb_reg = INT_ENB_MASK_REG,
		.reset_to_enb = 1,
		.status_reg = INT_STS_MASK_REG,
		.status_reg_mask = SHIRQ_RAS3_MASK,
		.clear_reg = INT_CLR_MASK_REG,
		.reset_to_clear = 1,
	},
};

struct shirq_dev_config shirq_intrcomm_ras_config[] = {
	{
		.virq = VIRQ_CANU,
		.status_mask = CAN_U_IRQ_MASK,
		.clear_mask = CAN_U_IRQ_MASK,
	}, {
		.virq = VIRQ_CANL,
		.status_mask = CAN_L_IRQ_MASK,
		.clear_mask = CAN_L_IRQ_MASK,
	}, {
		.virq = VIRQ_UART1,
		.status_mask = UART1_IRQ_MASK,
		.clear_mask = UART1_IRQ_MASK,
	}, {
		.virq = VIRQ_UART2,
		.status_mask = UART2_IRQ_MASK,
		.clear_mask = UART2_IRQ_MASK,
	}, {
		.virq = VIRQ_SSP1,
		.status_mask = SSP1_IRQ_MASK,
		.clear_mask = SSP1_IRQ_MASK,
	}, {
		.virq = VIRQ_SSP2,
		.status_mask = SSP2_IRQ_MASK,
		.clear_mask = SSP2_IRQ_MASK,
	}, {
		.virq = VIRQ_SMII0,
		.status_mask = SMII0_IRQ_MASK,
		.clear_mask = SMII0_IRQ_MASK,
	}, {
		.virq = VIRQ_MII1_SMII1,
		.status_mask = MII1_SMII1_IRQ_MASK,
		.clear_mask = MII1_SMII1_IRQ_MASK,
	}, {
		.virq = VIRQ_WAKEUP_SMII0,
		.status_mask = WAKEUP_SMII0_IRQ_MASK,
		.clear_mask = WAKEUP_SMII0_IRQ_MASK,
	}, {
		.virq = VIRQ_WAKEUP_MII1_SMII1,
		.status_mask = WAKEUP_MII1_SMII1_IRQ_MASK,
		.clear_mask = WAKEUP_MII1_SMII1_IRQ_MASK,
	}, {
		.virq = VIRQ_I2C1,
		.status_mask = I2C1_IRQ_MASK,
		.clear_mask = I2C1_IRQ_MASK,
	},
};

struct spear_shirq shirq_intrcomm_ras = {
	.irq = IRQ_INTRCOMM_RAS_ARM,
	.dev_config = shirq_intrcomm_ras_config,
	.dev_count = ARRAY_SIZE(shirq_intrcomm_ras_config),
	.regs = {
		.enb_reg = -1,
		.status_reg = INT_STS_MASK_REG,
		.status_reg_mask = SHIRQ_INTRCOMM_RAS_MASK,
		.clear_reg = INT_CLR_MASK_REG,
		.reset_to_clear = 1,
	},
};

/* spear320 routines */
void __init spear320_init(void)
{
	void __iomem *base;
	int ret = 0;

	/* call spear3xx family common init function */
	spear3xx_init();

	/* shared irq registration */
	base = ioremap(SPEAR320_SOC_CONFIG_BASE, SZ_4K);
	if (base) {
		/* shirq 1 */
		shirq_ras1.regs.base = base;
		ret = spear_shirq_register(&shirq_ras1);
		if (ret)
			printk(KERN_ERR "Error registering Shared IRQ 1\n");

		/* shirq 3 */
		shirq_ras3.regs.base = base;
		ret = spear_shirq_register(&shirq_ras3);
		if (ret)
			printk(KERN_ERR "Error registering Shared IRQ 3\n");

		/* shirq 4 */
		shirq_intrcomm_ras.regs.base = base;
		ret = spear_shirq_register(&shirq_intrcomm_ras);
		if (ret)
			printk(KERN_ERR "Error registering Shared IRQ 4\n");
	}

	/* This fixes addresses of all pmx devices for spear320 */
	spear3xx_pmx_init_addr(&pmx_driver, SPEAR320_PAD_MUX_CONFIG_REG);

	/* pmx initialization */
	ret = pmx_register(&pmx_driver);
	if (ret)
		pr_err("padmux: registeration failed. err no: %d\n", ret);
}
