/*
 * arch/arm/mach-spear3xx/spear300.c
 *
 * SPEAr300 machine source file
 *
 * Copyright (C) 2009 ST Microelectronics
 * Viresh Kumar<viresh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/types.h>
#include <linux/amba/pl061.h>
#include <linux/ptrace.h>
#include <linux/mtd/fsmc.h>
#include <asm/irq.h>
#include <mach/generic.h>
#include <mach/spear.h>
#include <plat/shirq.h>

/* pad multiplexing support */
#define MODE_CONFIG_REG		0x04

/* modes */
#define NAND_MODE			(1 << 0)
#define NOR_MODE			(1 << 1)
#define PHOTO_FRAME_MODE		(1 << 2)
#define LEND_IP_PHONE_MODE		(1 << 3)
#define HEND_IP_PHONE_MODE		(1 << 4)
#define LEND_WIFI_PHONE_MODE		(1 << 5)
#define HEND_WIFI_PHONE_MODE		(1 << 6)
#define ATA_PABX_WI2S_MODE		(1 << 7)
#define ATA_PABX_I2S_MODE		(1 << 8)
#define CAML_LCDW_MODE			(1 << 9)
#define CAMU_LCD_MODE			(1 << 10)
#define CAMU_WLCD_MODE			(1 << 11)
#define CAML_LCD_MODE			(1 << 12)
#define ALL_MODES			0x1FFF

struct pmx_mode nand_mode = {
	.id = NAND_MODE,
	.name = "nand mode",
	.value = 0x00,
};

struct pmx_mode nor_mode = {
	.id = NOR_MODE,
	.name = "nor mode",
	.value = 0x01,
};

struct pmx_mode photo_frame_mode = {
	.id = PHOTO_FRAME_MODE,
	.name = "photo frame mode",
	.value = 0x02,
};

struct pmx_mode lend_ip_phone_mode = {
	.id = LEND_IP_PHONE_MODE,
	.name = "lend ip phone mode",
	.value = 0x03,
};

struct pmx_mode hend_ip_phone_mode = {
	.id = HEND_IP_PHONE_MODE,
	.name = "hend ip phone mode",
	.value = 0x04,
};

struct pmx_mode lend_wifi_phone_mode = {
	.id = LEND_WIFI_PHONE_MODE,
	.name = "lend wifi phone mode",
	.value = 0x05,
};

struct pmx_mode hend_wifi_phone_mode = {
	.id = HEND_WIFI_PHONE_MODE,
	.name = "hend wifi phone mode",
	.value = 0x06,
};

struct pmx_mode ata_pabx_wi2s_mode = {
	.id = ATA_PABX_WI2S_MODE,
	.name = "ata pabx wi2s mode",
	.value = 0x07,
};

struct pmx_mode ata_pabx_i2s_mode = {
	.id = ATA_PABX_I2S_MODE,
	.name = "ata pabx i2s mode",
	.value = 0x08,
};

struct pmx_mode caml_lcdw_mode = {
	.id = CAML_LCDW_MODE,
	.name = "caml lcdw mode",
	.value = 0x0C,
};

struct pmx_mode camu_lcd_mode = {
	.id = CAMU_LCD_MODE,
	.name = "camu lcd mode",
	.value = 0x0D,
};

struct pmx_mode camu_wlcd_mode = {
	.id = CAMU_WLCD_MODE,
	.name = "camu wlcd mode",
	.value = 0x0E,
};

struct pmx_mode caml_lcd_mode = {
	.id = CAML_LCD_MODE,
	.name = "caml lcd mode",
	.value = 0x0F,
};

/* devices */
/* Pad multiplexing for FSMC 2 NAND devices */
static struct pmx_mux_reg pmx_fsmc_2_mux[] = {
	{
		.offset = PAD_MUX_CONFIG_REG,
		.mask = PMX_FIRDA_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_fsmc_2_chips_modes[] = {
	{
		.ids = NAND_MODE | NOR_MODE | PHOTO_FRAME_MODE |
			ATA_PABX_WI2S_MODE | ATA_PABX_I2S_MODE,
		.mux_regs = pmx_fsmc_2_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_fsmc_2_mux),
	},
};

struct pmx_dev pmx_fsmc_2_chips = {
	.name = "fsmc_2_chips",
	.modes = pmx_fsmc_2_chips_modes,
	.mode_count = ARRAY_SIZE(pmx_fsmc_2_chips_modes),
};

/* Pad multiplexing for FSMC 4 NAND devices */
static struct pmx_mux_reg pmx_fsmc_4_mux[] = {
	{
		.offset = PAD_MUX_CONFIG_REG,
		.mask = PMX_FIRDA_MASK | PMX_UART0_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_fsmc_4_chips_modes[] = {
	{
		.ids = NAND_MODE | NOR_MODE | PHOTO_FRAME_MODE |
			ATA_PABX_WI2S_MODE | ATA_PABX_I2S_MODE,
		.mux_regs = pmx_fsmc_4_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_fsmc_4_mux),
	},
};

struct pmx_dev pmx_fsmc_4_chips = {
	.name = "fsmc_4_chips",
	.modes = pmx_fsmc_4_chips_modes,
	.mode_count = ARRAY_SIZE(pmx_fsmc_4_chips_modes),
};

/* Pad multiplexing for Keyboard device */
static struct pmx_mux_reg pmx_kbd_mux[] = {
	{
		.offset = PAD_MUX_CONFIG_REG,
		.mask = 0x0,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_keyboard_modes[] = {
	{
		.ids = LEND_IP_PHONE_MODE | HEND_IP_PHONE_MODE |
			LEND_WIFI_PHONE_MODE | HEND_WIFI_PHONE_MODE |
			CAML_LCDW_MODE | CAMU_LCD_MODE | CAMU_WLCD_MODE |
			CAML_LCD_MODE,
		.mux_regs = pmx_kbd_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_kbd_mux),
	},
};

struct pmx_dev pmx_keyboard = {
	.name = "keyboard",
	.modes = pmx_keyboard_modes,
	.mode_count = ARRAY_SIZE(pmx_keyboard_modes),
};

/* Pad multiplexing for CLCD device */
static struct pmx_mux_reg pmx_clcd_pfmode_mux[] = {
	{
		.offset = PAD_MUX_CONFIG_REG,
		.mask = PMX_TIMER_1_2_MASK | PMX_TIMER_3_4_MASK,
		.value = 0,
	},
};

static struct pmx_mux_reg pmx_clcd_lcdmode_mux[] = {
	{
		.offset = PAD_MUX_CONFIG_REG,
		.mask = PMX_TIMER_3_4_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_clcd_modes[] = {
	{
		.ids = PHOTO_FRAME_MODE,
		.mux_regs = pmx_clcd_pfmode_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_clcd_pfmode_mux),
	}, {
		.ids = HEND_IP_PHONE_MODE | HEND_WIFI_PHONE_MODE |
			CAMU_LCD_MODE | CAML_LCD_MODE,
		.mux_regs = pmx_clcd_lcdmode_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_clcd_lcdmode_mux),
	},
};

struct pmx_dev pmx_clcd = {
	.name = "clcd",
	.modes = pmx_clcd_modes,
	.mode_count = ARRAY_SIZE(pmx_clcd_modes),
};

/* Pad multiplexing for Telecom GPIO device */
static struct pmx_mux_reg pmx_gpio_lcdmode_mux[] = {
	{
		.offset = PAD_MUX_CONFIG_REG,
		.mask = PMX_MII_MASK,
		.value = 0,
	},
};

static struct pmx_mux_reg pmx_gpio_fonemode_mux[] = {
	{
		.offset = PAD_MUX_CONFIG_REG,
		.mask = PMX_MII_MASK | PMX_TIMER_1_2_MASK | PMX_TIMER_3_4_MASK,
		.value = 0,
	},
};

static struct pmx_mux_reg pmx_gpio_atai2smode_mux[] = {
	{
		.offset = PAD_MUX_CONFIG_REG,
		.mask = PMX_MII_MASK | PMX_TIMER_3_4_MASK,
		.value = 0,
	},
};

static struct pmx_mux_reg pmx_gpio_lendfonemode_mux[] = {
	{
		.offset = PAD_MUX_CONFIG_REG,
		.mask = PMX_MII_MASK | PMX_TIMER_1_2_MASK,
		.value = 0,
	},
};

static struct pmx_mux_reg pmx_gpio_atawi2smode_mux[] = {
	{
		.offset = PAD_MUX_CONFIG_REG,
		.mask = PMX_MII_MASK | PMX_TIMER_1_2_MASK | PMX_TIMER_3_4_MASK
			| PMX_UART0_MODEM_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_telecom_gpio_modes[] = {
	{
		.ids = PHOTO_FRAME_MODE | CAMU_LCD_MODE | CAML_LCD_MODE,
		.mux_regs = pmx_gpio_lcdmode_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_gpio_lcdmode_mux),
	}, {
		.ids = LEND_IP_PHONE_MODE | LEND_WIFI_PHONE_MODE,
		.mux_regs = pmx_gpio_fonemode_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_gpio_fonemode_mux),
	}, {
		.ids = ATA_PABX_I2S_MODE | CAML_LCDW_MODE | CAMU_WLCD_MODE,
		.mux_regs = pmx_gpio_atai2smode_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_gpio_atai2smode_mux),
	}, {
		.ids = HEND_IP_PHONE_MODE | HEND_WIFI_PHONE_MODE,
		.mux_regs = pmx_gpio_lendfonemode_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_gpio_lendfonemode_mux),
	}, {
		.ids = ATA_PABX_WI2S_MODE,
		.mux_regs = pmx_gpio_atawi2smode_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_gpio_atawi2smode_mux),
	},
};

struct pmx_dev pmx_telecom_gpio = {
	.name = "telecom_gpio",
	.modes = pmx_telecom_gpio_modes,
	.mode_count = ARRAY_SIZE(pmx_telecom_gpio_modes),
};

/* Pad multiplexing for TDM device */
static struct pmx_mux_reg pmx_tdm_mux[] = {
	{
		.offset = PAD_MUX_CONFIG_REG,
		.mask = PMX_UART0_MODEM_MASK | PMX_SSP_CS_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_telecom_tdm_modes[] = {
	{
		.ids = PHOTO_FRAME_MODE | LEND_IP_PHONE_MODE |
			HEND_IP_PHONE_MODE | LEND_WIFI_PHONE_MODE
			| HEND_WIFI_PHONE_MODE | ATA_PABX_WI2S_MODE
			| ATA_PABX_I2S_MODE | CAML_LCDW_MODE | CAMU_LCD_MODE
			| CAMU_WLCD_MODE | CAML_LCD_MODE,
		.mux_regs = pmx_tdm_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_tdm_mux),
	},
};

struct pmx_dev pmx_telecom_tdm = {
	.name = "telecom_tdm",
	.modes = pmx_telecom_tdm_modes,
	.mode_count = ARRAY_SIZE(pmx_telecom_tdm_modes),
};

/* Pad multiplexing for spi cs i2c device */
static struct pmx_mux_reg pmx_spi_cs_i2c_mux[] = {
	{
		.offset = PAD_MUX_CONFIG_REG,
		.mask = PMX_TIMER_1_2_MASK | PMX_TIMER_3_4_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_telecom_spi_cs_i2c_clk_modes[] = {
	{
		.ids = LEND_IP_PHONE_MODE | HEND_IP_PHONE_MODE |
			LEND_WIFI_PHONE_MODE | HEND_WIFI_PHONE_MODE
			| ATA_PABX_WI2S_MODE | ATA_PABX_I2S_MODE |
			CAML_LCDW_MODE | CAML_LCD_MODE,
		.mux_regs = pmx_spi_cs_i2c_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_spi_cs_i2c_mux),
	},
};

struct pmx_dev pmx_telecom_spi_cs_i2c_clk = {
	.name = "telecom_spi_cs_i2c_clk",
	.modes = pmx_telecom_spi_cs_i2c_clk_modes,
	.mode_count = ARRAY_SIZE(pmx_telecom_spi_cs_i2c_clk_modes),
};

static struct pmx_mux_reg pmx_caml_mux[] = {
	{
		.offset = PAD_MUX_CONFIG_REG,
		.mask = PMX_MII_MASK,
		.value = 0,
	},
};

static struct pmx_mux_reg pmx_camu_mux[] = {
	{
		.offset = PAD_MUX_CONFIG_REG,
		.mask = PMX_TIMER_1_2_MASK | PMX_TIMER_3_4_MASK | PMX_MII_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_telecom_camera_modes[] = {
	{
		.ids = CAML_LCDW_MODE | CAML_LCD_MODE,
		.mux_regs = pmx_caml_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_caml_mux),
	}, {
		.ids = CAMU_LCD_MODE | CAMU_WLCD_MODE,
		.mux_regs = pmx_camu_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_camu_mux),
	},
};

struct pmx_dev pmx_telecom_camera = {
	.name = "telecom_camera",
	.modes = pmx_telecom_camera_modes,
	.mode_count = ARRAY_SIZE(pmx_telecom_camera_modes),
};

/* Pad multiplexing for dac device */
static struct pmx_mux_reg pmx_dac_mux[] = {
	{
		.offset = PAD_MUX_CONFIG_REG,
		.mask = PMX_TIMER_1_2_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_telecom_dac_modes[] = {
	{
		.ids = ATA_PABX_I2S_MODE | CAML_LCDW_MODE | CAMU_LCD_MODE
			| CAMU_WLCD_MODE | CAML_LCD_MODE,
		.mux_regs = pmx_dac_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_dac_mux),
	},
};

struct pmx_dev pmx_telecom_dac = {
	.name = "telecom_dac",
	.modes = pmx_telecom_dac_modes,
	.mode_count = ARRAY_SIZE(pmx_telecom_dac_modes),
};

/* Pad multiplexing for spi cs i2c device */
static struct pmx_mux_reg pmx_i2s_mux[] = {
	{
		.offset = PAD_MUX_CONFIG_REG,
		.mask = PMX_UART0_MODEM_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_telecom_i2s_modes[] = {
	{
		.ids = LEND_IP_PHONE_MODE | HEND_IP_PHONE_MODE
			| LEND_WIFI_PHONE_MODE | HEND_WIFI_PHONE_MODE |
			ATA_PABX_I2S_MODE | CAML_LCDW_MODE | CAMU_LCD_MODE
			| CAMU_WLCD_MODE | CAML_LCD_MODE,
		.mux_regs = pmx_i2s_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_i2s_mux),
	},
};

struct pmx_dev pmx_telecom_i2s = {
	.name = "telecom_i2s",
	.modes = pmx_telecom_i2s_modes,
	.mode_count = ARRAY_SIZE(pmx_telecom_i2s_modes),
};

/* Pad multiplexing for bootpins device */
static struct pmx_mux_reg pmx_bootpins_mux[] = {
	{
		.offset = PAD_MUX_CONFIG_REG,
		.mask = PMX_UART0_MODEM_MASK | PMX_TIMER_1_2_MASK |
			PMX_TIMER_3_4_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_telecom_boot_pins_modes[] = {
	{
		.ids = NAND_MODE | NOR_MODE,
		.mux_regs = pmx_bootpins_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_bootpins_mux),
	},
};

struct pmx_dev pmx_telecom_boot_pins = {
	.name = "telecom_boot_pins",
	.modes = pmx_telecom_boot_pins_modes,
	.mode_count = ARRAY_SIZE(pmx_telecom_boot_pins_modes),
};

/* Pad multiplexing for sdhci 4bit device */
static struct pmx_mux_reg pmx_sdhci_4bit_mux[] = {
	{
		.offset = PAD_MUX_CONFIG_REG,
		.mask = PMX_GPIO_PIN0_MASK | PMX_GPIO_PIN1_MASK |
			PMX_GPIO_PIN2_MASK | PMX_GPIO_PIN3_MASK |
			PMX_GPIO_PIN4_MASK | PMX_GPIO_PIN5_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_telecom_sdhci_4bit_modes[] = {
	{
		.ids = PHOTO_FRAME_MODE | LEND_IP_PHONE_MODE |
			HEND_IP_PHONE_MODE | LEND_WIFI_PHONE_MODE |
			HEND_WIFI_PHONE_MODE | CAML_LCDW_MODE | CAMU_LCD_MODE |
			CAMU_WLCD_MODE | CAML_LCD_MODE | ATA_PABX_WI2S_MODE |
			ATA_PABX_I2S_MODE,
		.mux_regs = pmx_sdhci_4bit_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_sdhci_4bit_mux),
	},
};

struct pmx_dev pmx_telecom_sdhci_4bit = {
	.name = "telecom_sdhci_4bit",
	.modes = pmx_telecom_sdhci_4bit_modes,
	.mode_count = ARRAY_SIZE(pmx_telecom_sdhci_4bit_modes),
};

/* Pad multiplexing for spi cs i2c device */
static struct pmx_mux_reg pmx_sdhci_8bit_mux[] = {
	{
		.offset = PAD_MUX_CONFIG_REG,
		.mask = PMX_GPIO_PIN0_MASK | PMX_GPIO_PIN1_MASK |
			PMX_GPIO_PIN2_MASK | PMX_GPIO_PIN3_MASK |
			PMX_GPIO_PIN4_MASK | PMX_GPIO_PIN5_MASK | PMX_MII_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_telecom_sdhci_8bit_modes[] = {
	{
		.ids = PHOTO_FRAME_MODE | LEND_IP_PHONE_MODE |
			HEND_IP_PHONE_MODE | LEND_WIFI_PHONE_MODE |
			HEND_WIFI_PHONE_MODE | CAML_LCDW_MODE | CAMU_LCD_MODE |
			CAMU_WLCD_MODE | CAML_LCD_MODE,
		.mux_regs = pmx_sdhci_8bit_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_sdhci_8bit_mux),
	},
};

struct pmx_dev pmx_telecom_sdhci_8bit = {
	.name = "telecom_sdhci_8bit",
	.modes = pmx_telecom_sdhci_8bit_modes,
	.mode_count = ARRAY_SIZE(pmx_telecom_sdhci_8bit_modes),
};

/* Pad multiplexing for spi cs i2c device */
static struct pmx_mux_reg pmx_gpio1_mux[] = {
	{
		.offset = PAD_MUX_CONFIG_REG,
		.mask = PMX_UART0_MODEM_MASK | PMX_TIMER_1_2_MASK |
			PMX_TIMER_3_4_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_gpio1_modes[] = {
	{
		.ids = PHOTO_FRAME_MODE,
		.mux_regs = pmx_gpio1_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_gpio1_mux),
	},
};

struct pmx_dev pmx_gpio1 = {
	.name = "arm gpio1",
	.modes = pmx_gpio1_modes,
	.mode_count = ARRAY_SIZE(pmx_gpio1_modes),
};

/* pmx driver structure */
struct pmx_driver pmx_driver = {
	.mode_reg = {.offset = MODE_CONFIG_REG, .mask = 0x0000000f},
};

/* Add spear300 specific devices here */
/* CLCD device registration */
struct amba_device clcd_device = {
	.dev = {
		.init_name = "clcd",
		.coherent_dma_mask = ~0,
		.platform_data = &clcd_plat_data,
	},
	.res = {
		.start = SPEAR300_CLCD_BASE,
		.end = SPEAR300_CLCD_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	.dma_mask = ~0,
	.irq = {IRQ_CLCD, NO_IRQ},
};

/* arm gpio1 device registration */
static struct pl061_platform_data gpio1_plat_data = {
	.gpio_base	= 8,
	.irq_base	= SPEAR_GPIO1_INT_BASE,
};

struct amba_device gpio1_device = {
	.dev = {
		.init_name = "gpio1",
		.platform_data = &gpio1_plat_data,
	},
	.res = {
		.start = SPEAR300_GPIO_BASE,
		.end = SPEAR300_GPIO_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	.irq = {VIRQ_GPIO1, NO_IRQ},
};

/* keyboard device registration */
static struct resource kbd_resources[] = {
	{
		.start = SPEAR300_KEYBOARD_BASE,
		.end = SPEAR300_KEYBOARD_BASE + SZ_1K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = VIRQ_KEYBOARD,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device kbd_device = {
	.name = "keyboard",
	.id = -1,
	.num_resources = ARRAY_SIZE(kbd_resources),
	.resource = kbd_resources,
};

/* nand device registeration */
static struct fsmc_nand_platform_data nand0_platform_data;

static struct resource nand0_resources[] = {
	{
		.name = "nand_data",
		.start = SPEAR300_NAND_0_BASE,
		.end = SPEAR300_NAND_0_BASE + SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.name = "fsmc_regs",
		.start = SPEAR300_FSMC_BASE,
		.end = SPEAR300_FSMC_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
};

struct platform_device nand0_device = {
	.name = "fsmc-nand",
	.id = 0,
	.resource = nand0_resources,
	.num_resources = ARRAY_SIZE(nand0_resources),
	.dev.platform_data = &nand0_platform_data,
};

static struct fsmc_nand_platform_data nand1_platform_data;

static struct resource nand1_resources[] = {
	{
		.name = "nand_data",
		.start = SPEAR300_NAND_1_BASE,
		.end = SPEAR300_NAND_1_BASE + SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.name = "fsmc_regs",
		.start = SPEAR300_FSMC_BASE,
		.end = SPEAR300_FSMC_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
};

struct platform_device nand1_device = {
	.name = "fsmc-nand",
	.id = 1,
	.resource = nand1_resources,
	.num_resources = ARRAY_SIZE(nand1_resources),
	.dev.platform_data = &nand1_platform_data,
};

static struct fsmc_nand_platform_data nand2_platform_data;

static struct resource nand2_resources[] = {
	{
		.name = "nand_data",
		.start = SPEAR300_NAND_2_BASE,
		.end = SPEAR300_NAND_2_BASE + SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.name = "fsmc_regs",
		.start = SPEAR300_FSMC_BASE,
		.end = SPEAR300_FSMC_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
};

struct platform_device nand2_device = {
	.name = "fsmc-nand",
	.id = 2,
	.resource = nand2_resources,
	.num_resources = ARRAY_SIZE(nand2_resources),
	.dev.platform_data = &nand2_platform_data,
};

static struct fsmc_nand_platform_data nand3_platform_data;

static struct resource nand3_resources[] = {
	{
		.name = "nand_data",
		.start = SPEAR300_NAND_3_BASE,
		.end = SPEAR300_NAND_3_BASE + SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.name = "fsmc_regs",
		.start = SPEAR300_FSMC_BASE,
		.end = SPEAR300_FSMC_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
};

struct platform_device nand3_device = {
	.name = "fsmc-nand",
	.id = 3,
	.resource = nand3_resources,
	.num_resources = ARRAY_SIZE(nand3_resources),
	.dev.platform_data = &nand3_platform_data,
};

/* sdhci (sdio) device declaration */
static struct resource sdhci_resources[] = {
	{
		.start	= SPEAR300_SDHCI_BASE,
		.end	= SPEAR300_SDHCI_BASE + SZ_256 - 1,
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
		.virq = VIRQ_IT_PERS_S,
		.enb_mask = IT_PERS_S_IRQ_MASK,
		.status_mask = IT_PERS_S_IRQ_MASK,
	}, {
		.virq = VIRQ_IT_CHANGE_S,
		.enb_mask = IT_CHANGE_S_IRQ_MASK,
		.status_mask = IT_CHANGE_S_IRQ_MASK,
	}, {
		.virq = VIRQ_I2S,
		.enb_mask = I2S_IRQ_MASK,
		.status_mask = I2S_IRQ_MASK,
	}, {
		.virq = VIRQ_TDM,
		.enb_mask = TDM_IRQ_MASK,
		.status_mask = TDM_IRQ_MASK,
	}, {
		.virq = VIRQ_CAMERA_L,
		.enb_mask = CAMERA_L_IRQ_MASK,
		.status_mask = CAMERA_L_IRQ_MASK,
	}, {
		.virq = VIRQ_CAMERA_F,
		.enb_mask = CAMERA_F_IRQ_MASK,
		.status_mask = CAMERA_F_IRQ_MASK,
	}, {
		.virq = VIRQ_CAMERA_V,
		.enb_mask = CAMERA_V_IRQ_MASK,
		.status_mask = CAMERA_V_IRQ_MASK,
	}, {
		.virq = VIRQ_KEYBOARD,
		.enb_mask = KEYBOARD_IRQ_MASK,
		.status_mask = KEYBOARD_IRQ_MASK,
	}, {
		.virq = VIRQ_GPIO1,
		.enb_mask = GPIO1_IRQ_MASK,
		.status_mask = GPIO1_IRQ_MASK,
	},
};

struct spear_shirq shirq_ras1 = {
	.irq = IRQ_GEN_RAS_1,
	.dev_config = shirq_ras1_config,
	.dev_count = ARRAY_SIZE(shirq_ras1_config),
	.regs = {
		.enb_reg = INT_ENB_MASK_REG,
		.status_reg = INT_STS_MASK_REG,
		.status_reg_mask = SHIRQ_RAS1_MASK,
		.clear_reg = -1,
	},
};

/* Function handling sdhci and i2s memory sharing */
#define SDHCI_MEM_SELECT	0x20000000
void sdhci_i2s_mem_enable(u8 mask)
{
	u32 val;
	void __iomem *base = ioremap(SPEAR300_SOC_CONFIG_BASE, SZ_4K);
	if (!base) {
		pr_debug("sdhci_i2s_enb: ioremap fail\n");
		return;
	}

	val = readl(base + MODE_CONFIG_REG);
	if (mask == SDHCI_MEM_ENB)
		val |= SDHCI_MEM_SELECT;
	else
		val &= ~SDHCI_MEM_SELECT;
	writel(val, base + MODE_CONFIG_REG);
}

/* spear300 routines */
void __init spear300_init(void)
{
	int ret = 0;

	/* call spear3xx family common init function */
	spear3xx_init();

	/* shared irq registration */
	shirq_ras1.regs.base = ioremap(SPEAR300_TELECOM_BASE, SZ_4K);
	if (shirq_ras1.regs.base) {
		ret = spear_shirq_register(&shirq_ras1);
		if (ret)
			printk(KERN_ERR "Error registering Shared IRQ\n");
	}

	/* pmx initialization */
	pmx_driver.base = ioremap(SPEAR300_SOC_CONFIG_BASE, SZ_4K);
	if (pmx_driver.base) {
		ret = pmx_register(&pmx_driver);
		if (ret)
			printk(KERN_ERR "padmux: registeration failed. err no"
					": %d\n", ret);
		/* Free Mapping, device selection already done */
		iounmap(pmx_driver.base);
	}
}
