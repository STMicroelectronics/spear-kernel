/*
 * arch/arm/mach-spear3xx/spear3xx.c
 *
 * SPEAr3XX machines common source file
 *
 * Copyright (C) 2009 ST Microelectronics
 * Viresh Kumar<viresh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/types.h>
#include <linux/amba/pl022.h>
#include <linux/amba/pl061.h>
#include <linux/ptrace.h>
#include <linux/io.h>
#include <asm/hardware/vic.h>
#include <asm/irq.h>
#include <asm/mach/arch.h>
#include <mach/generic.h>
#include <mach/spear.h>

#define SPEAR3XX_WKUP_SRCS	(1 << IRQ_MAC_1 | 1 << IRQ_USB_DEV | \
				1 << IRQ_BASIC_RTC | 1 << IRQ_BASIC_GPIO)
/* Add spear3xx machines common devices here */
/* gpio device registration */
static struct pl061_platform_data gpio_plat_data = {
	.gpio_base	= 0,
	.irq_base	= SPEAR_GPIO_INT_BASE,
};

struct amba_device gpio_device = {
	.dev = {
		.init_name = "gpio",
		.platform_data = &gpio_plat_data,
	},
	.res = {
		.start = SPEAR3XX_ICM3_GPIO_BASE,
		.end = SPEAR3XX_ICM3_GPIO_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	.irq = {IRQ_BASIC_GPIO, NO_IRQ},
};

/* ssp device registration */
static struct pl022_ssp_controller ssp_platform_data = {
	.bus_id = 0,
	.enable_dma = 0,
	/*
	 * This is number of spi devices that can be connected to spi. There are
	 * two type of chipselects on which slave devices can work. One is chip
	 * select provided by spi masters other is controlled through external
	 * gpio's. We can't use chipselect provided from spi master (because as
	 * soon as FIFO becomes empty, CS is disabled and transfer ends). So
	 * this number now depends on number of gpios available for spi. each
	 * slave on each master requires a separate gpio pin.
	 */
	.num_chipselect = 2,
};

struct amba_device ssp0_device = {
	.dev = {
		.coherent_dma_mask = ~0,
		.init_name = "ssp-pl022.0",
		.platform_data = &ssp_platform_data,
	},
	.res = {
		.start = SPEAR3XX_ICM1_SSP_BASE,
		.end = SPEAR3XX_ICM1_SSP_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	.irq = {IRQ_SSP, NO_IRQ},
};

/* uart device registration */
struct amba_device uart_device = {
	.dev = {
		.init_name = "uart",
	},
	.res = {
		.start = SPEAR3XX_ICM1_UART_BASE,
		.end = SPEAR3XX_ICM1_UART_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	.irq = {IRQ_UART, NO_IRQ},
};

/* watchdog device registeration */
struct amba_device wdt_device = {
	.dev = {
		.init_name = "wdt",
	},
	.res = {
		.start = SPEAR3XX_ICM3_WDT_BASE,
		.end = SPEAR3XX_ICM3_WDT_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
};

/* i2c device registeration */
static struct resource i2c_resources[] = {
	{
		.start = SPEAR3XX_ICM1_I2C_BASE,
		.end = SPEAR3XX_ICM1_I2C_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = IRQ_I2C,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device i2c_device = {
	.name = "i2c_designware",
	.id = 0,
	.dev = {
		.coherent_dma_mask = ~0,
	},
	.num_resources = ARRAY_SIZE(i2c_resources),
	.resource = i2c_resources,
};

/* usb host device registeration */
static struct resource ehci_resources[] = {
	[0] = {
		.start = SPEAR3XX_ICM4_USB_EHCI0_1_BASE,
		.end = SPEAR3XX_ICM4_USB_EHCI0_1_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_USB_H_EHCI_0,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource ohci0_resources[] = {
	[0] = {
		.start = SPEAR3XX_ICM4_USB_OHCI0_BASE,
		.end = SPEAR3XX_ICM4_USB_OHCI0_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_USB_H_OHCI_0,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource ohci1_resources[] = {
	[0] = {
		.start = SPEAR3XX_ICM4_USB_OHCI1_BASE,
		.end = SPEAR3XX_ICM4_USB_OHCI1_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_USB_H_OHCI_1,
		.flags = IORESOURCE_IRQ,
	},
};

static u64 ehci_dmamask = ~0;
static int usbh_id = -1;

struct platform_device ehci_device = {
	.name = "spear-ehci",
	.id = -1,
	.dev = {
		.coherent_dma_mask = ~0,
		.dma_mask = &ehci_dmamask,
		.platform_data = &usbh_id,
	},
	.num_resources = ARRAY_SIZE(ehci_resources),
	.resource = ehci_resources,
};

static u64 ohci0_dmamask = ~0;

struct platform_device ohci0_device = {
	.name = "spear-ohci",
	.id = 0,
	.dev = {
		.coherent_dma_mask = ~0,
		.dma_mask = &ohci0_dmamask,
		.platform_data = &usbh_id,
	},
	.num_resources = ARRAY_SIZE(ohci0_resources),
	.resource = ohci0_resources,
};

static u64 ohci1_dmamask = ~0;

struct platform_device ohci1_device = {
	.name = "spear-ohci",
	.id = 1,
	.dev = {
		.coherent_dma_mask = ~0,
		.dma_mask = &ohci1_dmamask,
		.platform_data = &usbh_id,
	},
	.num_resources = ARRAY_SIZE(ohci1_resources),
	.resource = ohci1_resources,
};

/* rtc device registration */
static struct resource rtc_resources[] = {
	{
		.start = SPEAR3XX_ICM3_RTC_BASE,
		.end = SPEAR3XX_ICM3_RTC_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = IRQ_BASIC_RTC,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device rtc_device = {
	.name = "rtc-spear",
	.id = -1,
	.num_resources = ARRAY_SIZE(rtc_resources),
	.resource = rtc_resources,
};

/* Do spear3xx familiy common initialization part here */
void __init spear3xx_init(void)
{
	/* nothing to do for now */
}

/* This will initialize vic */
void __init spear3xx_init_irq(void)
{
	vic_init((void __iomem *)VA_SPEAR3XX_ML1_VIC_BASE, 0, ~0,
			SPEAR3XX_WKUP_SRCS);
}

/* Following will create static virtual/physical mappings */
struct map_desc spear3xx_io_desc[] __initdata = {
	{
		.virtual	= VA_SPEAR3XX_ICM1_UART_BASE,
		.pfn		= __phys_to_pfn(SPEAR3XX_ICM1_UART_BASE),
		.length		= SZ_4K,
		.type		= MT_DEVICE
	}, {
		.virtual	= VA_SPEAR3XX_ML1_VIC_BASE,
		.pfn		= __phys_to_pfn(SPEAR3XX_ML1_VIC_BASE),
		.length		= SZ_4K,
		.type		= MT_DEVICE
	}, {
		.virtual	= VA_SPEAR3XX_ICM3_SYS_CTRL_BASE,
		.pfn		= __phys_to_pfn(SPEAR3XX_ICM3_SYS_CTRL_BASE),
		.length		= SZ_4K,
		.type		= MT_DEVICE
	}, {
		.virtual	= VA_SPEAR3XX_ICM3_MISC_REG_BASE,
		.pfn		= __phys_to_pfn(SPEAR3XX_ICM3_MISC_REG_BASE),
		.length		= SZ_4K,
		.type		= MT_DEVICE
	}, {
		.virtual	= IO_ADDRESS(SPEAR3XX_ICM3_SDRAM_CTRL_BASE),
		.pfn		= __phys_to_pfn(SPEAR3XX_ICM3_SDRAM_CTRL_BASE),
		.length		= SZ_4K,
		.type		= MT_DEVICE
	},
};

/* This will create static memory mapping for selected devices */
void __init spear3xx_map_io(void)
{
	iotable_init(spear3xx_io_desc, ARRAY_SIZE(spear3xx_io_desc));

	/* This will initialize clock framework */
	spear3xx_clk_init();
}

/* pad multiplexing support */
/* devices */

/* Pad multiplexing for firda device */
static struct pmx_mux_reg pmx_firda_mux[] = {
	{
		.offset = PAD_MUX_CONFIG_REG,
		.mask = PMX_FIRDA_MASK,
		.value = PMX_FIRDA_MASK,
	},
};

static struct pmx_dev_mode pmx_firda_modes[] = {
	{
		.ids = 0xffffffff,
		.mux_regs = pmx_firda_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_firda_mux),
	},
};

struct pmx_dev pmx_firda = {
	.name = "firda",
	.modes = pmx_firda_modes,
	.mode_count = ARRAY_SIZE(pmx_firda_modes),
};

/* Pad multiplexing for i2c device */
static struct pmx_mux_reg pmx_i2c_mux[] = {
	{
		.offset = PAD_MUX_CONFIG_REG,
		.mask = PMX_I2C_MASK,
		.value = PMX_I2C_MASK,
	},
};

static struct pmx_dev_mode pmx_i2c_modes[] = {
	{
		.ids = 0xffffffff,
		.mux_regs = pmx_i2c_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_i2c_mux),
	},
};

struct pmx_dev pmx_i2c = {
	.name = "i2c",
	.modes = pmx_i2c_modes,
	.mode_count = ARRAY_SIZE(pmx_i2c_modes),
};

/* Pad multiplexing for firda device */
static struct pmx_mux_reg pmx_ssp_cs_mux[] = {
	{
		.offset = PAD_MUX_CONFIG_REG,
		.mask = PMX_SSP_CS_MASK,
		.value = PMX_SSP_CS_MASK,
	},
};

static struct pmx_dev_mode pmx_ssp_cs_modes[] = {
	{
		.ids = 0xffffffff,
		.mux_regs = pmx_ssp_cs_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_ssp_cs_mux),
	},
};

struct pmx_dev pmx_ssp_cs = {
	.name = "ssp_chip_selects",
	.modes = pmx_ssp_cs_modes,
	.mode_count = ARRAY_SIZE(pmx_ssp_cs_modes),
};

/* Pad multiplexing for ssp device */
static struct pmx_mux_reg pmx_ssp_mux[] = {
	{
		.offset = PAD_MUX_CONFIG_REG,
		.mask = PMX_SSP_MASK,
		.value = PMX_SSP_MASK,
	},
};

static struct pmx_dev_mode pmx_ssp_modes[] = {
	{
		.ids = 0xffffffff,
		.mux_regs = pmx_ssp_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_ssp_mux),
	},
};

struct pmx_dev pmx_ssp = {
	.name = "ssp",
	.modes = pmx_ssp_modes,
	.mode_count = ARRAY_SIZE(pmx_ssp_modes),
};

/* Pad multiplexing for mii device */
static struct pmx_mux_reg pmx_mii_mux[] = {
	{
		.offset = PAD_MUX_CONFIG_REG,
		.mask = PMX_MII_MASK,
		.value = PMX_MII_MASK,
	},
};

static struct pmx_dev_mode pmx_mii_modes[] = {
	{
		.ids = 0xffffffff,
		.mux_regs = pmx_mii_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_mii_mux),
	},
};

struct pmx_dev pmx_mii = {
	.name = "mii",
	.modes = pmx_mii_modes,
	.mode_count = ARRAY_SIZE(pmx_mii_modes),
};

/* Pad multiplexing for gpio pin0 device */
static struct pmx_mux_reg pmx_gpio_pin0_mux[] = {
	{
		.offset = PAD_MUX_CONFIG_REG,
		.mask = PMX_GPIO_PIN0_MASK,
		.value = PMX_GPIO_PIN0_MASK,
	},
};

static struct pmx_dev_mode pmx_gpio_pin0_modes[] = {
	{
		.ids = 0xffffffff,
		.mux_regs = pmx_gpio_pin0_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_gpio_pin0_mux),
	},
};

struct pmx_dev pmx_gpio_pin0 = {
	.name = "gpio_pin0",
	.modes = pmx_gpio_pin0_modes,
	.mode_count = ARRAY_SIZE(pmx_gpio_pin0_modes),
};

/* Pad multiplexing for gpio pin1 device */
static struct pmx_mux_reg pmx_gpio_pin1_mux[] = {
	{
		.offset = PAD_MUX_CONFIG_REG,
		.mask = PMX_GPIO_PIN1_MASK,
		.value = PMX_GPIO_PIN1_MASK,
	},
};

static struct pmx_dev_mode pmx_gpio_pin1_modes[] = {
	{
		.ids = 0xffffffff,
		.mux_regs = pmx_gpio_pin1_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_gpio_pin1_mux),
	},
};

struct pmx_dev pmx_gpio_pin1 = {
	.name = "gpio_pin1",
	.modes = pmx_gpio_pin1_modes,
	.mode_count = ARRAY_SIZE(pmx_gpio_pin1_modes),
};

/* Pad multiplexing for gpio pin2 device */
static struct pmx_mux_reg pmx_gpio_pin2_mux[] = {
	{
		.offset = PAD_MUX_CONFIG_REG,
		.mask = PMX_GPIO_PIN2_MASK,
		.value = PMX_GPIO_PIN2_MASK,
	},
};

static struct pmx_dev_mode pmx_gpio_pin2_modes[] = {
	{
		.ids = 0xffffffff,
		.mux_regs = pmx_gpio_pin2_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_gpio_pin2_mux),
	},
};

struct pmx_dev pmx_gpio_pin2 = {
	.name = "gpio_pin2",
	.modes = pmx_gpio_pin2_modes,
	.mode_count = ARRAY_SIZE(pmx_gpio_pin2_modes),
};

/* Pad multiplexing for gpio pin3 device */
static struct pmx_mux_reg pmx_gpio_pin3_mux[] = {
	{
		.offset = PAD_MUX_CONFIG_REG,
		.mask = PMX_GPIO_PIN3_MASK,
		.value = PMX_GPIO_PIN3_MASK,
	},
};

static struct pmx_dev_mode pmx_gpio_pin3_modes[] = {
	{
		.ids = 0xffffffff,
		.mux_regs = pmx_gpio_pin3_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_gpio_pin3_mux),
	},
};

struct pmx_dev pmx_gpio_pin3 = {
	.name = "gpio_pin3",
	.modes = pmx_gpio_pin3_modes,
	.mode_count = ARRAY_SIZE(pmx_gpio_pin3_modes),
};

/* Pad multiplexing for gpio pin4 device */
static struct pmx_mux_reg pmx_gpio_pin4_mux[] = {
	{
		.offset = PAD_MUX_CONFIG_REG,
		.mask = PMX_GPIO_PIN4_MASK,
		.value = PMX_GPIO_PIN4_MASK,
	},
};

static struct pmx_dev_mode pmx_gpio_pin4_modes[] = {
	{
		.ids = 0xffffffff,
		.mux_regs = pmx_gpio_pin4_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_gpio_pin4_mux),
	},
};

struct pmx_dev pmx_gpio_pin4 = {
	.name = "gpio_pin4",
	.modes = pmx_gpio_pin4_modes,
	.mode_count = ARRAY_SIZE(pmx_gpio_pin4_modes),
};

/* Pad multiplexing for gpio pin5 device */
static struct pmx_mux_reg pmx_gpio_pin5_mux[] = {
	{
		.offset = PAD_MUX_CONFIG_REG,
		.mask = PMX_GPIO_PIN5_MASK,
		.value = PMX_GPIO_PIN5_MASK,
	},
};

static struct pmx_dev_mode pmx_gpio_pin5_modes[] = {
	{
		.ids = 0xffffffff,
		.mux_regs = pmx_gpio_pin5_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_gpio_pin5_mux),
	},
};

struct pmx_dev pmx_gpio_pin5 = {
	.name = "gpio_pin5",
	.modes = pmx_gpio_pin5_modes,
	.mode_count = ARRAY_SIZE(pmx_gpio_pin5_modes),
};

/* Pad multiplexing for uart0 modem device */
static struct pmx_mux_reg pmx_uart0_modem_mux[] = {
	{
		.offset = PAD_MUX_CONFIG_REG,
		.mask = PMX_UART0_MODEM_MASK,
		.value = PMX_UART0_MODEM_MASK,
	},
};

static struct pmx_dev_mode pmx_uart0_modem_modes[] = {
	{
		.ids = 0xffffffff,
		.mux_regs = pmx_uart0_modem_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_uart0_modem_mux),
	},
};

struct pmx_dev pmx_uart0_modem = {
	.name = "uart0_modem",
	.modes = pmx_uart0_modem_modes,
	.mode_count = ARRAY_SIZE(pmx_uart0_modem_modes),
};

/* Pad multiplexing for uart0 device */
static struct pmx_mux_reg pmx_uart0_mux[] = {
	{
		.offset = PAD_MUX_CONFIG_REG,
		.mask = PMX_UART0_MASK,
		.value = PMX_UART0_MASK,
	},
};

static struct pmx_dev_mode pmx_uart0_modes[] = {
	{
		.ids = 0xffffffff,
		.mux_regs = pmx_uart0_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_uart0_mux),
	},
};

struct pmx_dev pmx_uart0 = {
	.name = "uart0",
	.modes = pmx_uart0_modes,
	.mode_count = ARRAY_SIZE(pmx_uart0_modes),
};

/* Pad multiplexing for timer 3, 4 device */
static struct pmx_mux_reg pmx_timer_3_4_mux[] = {
	{
		.offset = PAD_MUX_CONFIG_REG,
		.mask = PMX_TIMER_3_4_MASK,
		.value = PMX_TIMER_3_4_MASK,
	},
};

static struct pmx_dev_mode pmx_timer_3_4_modes[] = {
	{
		.ids = 0xffffffff,
		.mux_regs = pmx_timer_3_4_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_timer_3_4_mux),
	},
};

struct pmx_dev pmx_timer_3_4 = {
	.name = "timer_3_4",
	.modes = pmx_timer_3_4_modes,
	.mode_count = ARRAY_SIZE(pmx_timer_3_4_modes),
};

/* Pad multiplexing for gpio pin0 device */
static struct pmx_mux_reg pmx_timer_1_2_mux[] = {
	{
		.offset = PAD_MUX_CONFIG_REG,
		.mask = PMX_TIMER_1_2_MASK,
		.value = PMX_TIMER_1_2_MASK,
	},
};

static struct pmx_dev_mode pmx_timer_1_2_modes[] = {
	{
		.ids = 0xffffffff,
		.mux_regs = pmx_timer_1_2_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_timer_1_2_mux),
	},
};

struct pmx_dev pmx_timer_1_2 = {
	.name = "timer_1_2",
	.modes = pmx_timer_1_2_modes,
	.mode_count = ARRAY_SIZE(pmx_timer_1_2_modes),
};

#if defined(CONFIG_MACH_SPEAR310) || defined(CONFIG_MACH_SPEAR320)
/* plgpios devices */
/* Pad multiplexing for plgpio_0_1 devices */
static struct pmx_mux_reg pmx_plgpio_0_1_mux[] = {
	{
		.offset = PAD_MUX_CONFIG_REG,
		.mask = PMX_FIRDA_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_plgpio_0_1_modes[] = {
	{
		.ids = 0xffffffff,
		.mux_regs = pmx_plgpio_0_1_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_plgpio_0_1_mux),
	},
};

struct pmx_dev pmx_plgpio_0_1 = {
	.name = "plgpio 0 and 1",
	.modes = pmx_plgpio_0_1_modes,
	.mode_count = ARRAY_SIZE(pmx_plgpio_0_1_modes),
};

/* Pad multiplexing for plgpio_2_3 devices */
static struct pmx_mux_reg pmx_plgpio_2_3_mux[] = {
	{
		.offset = PAD_MUX_CONFIG_REG,
		.mask = PMX_UART0_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_plgpio_2_3_modes[] = {
	{
		.ids = 0xffffffff,
		.mux_regs = pmx_plgpio_2_3_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_plgpio_2_3_mux),
	},
};

struct pmx_dev pmx_plgpio_2_3 = {
	.name = "plgpio 2 and 3",
	.modes = pmx_plgpio_2_3_modes,
	.mode_count = ARRAY_SIZE(pmx_plgpio_2_3_modes),
};

/* Pad multiplexing for plgpio_4_5 devices */
static struct pmx_mux_reg pmx_plgpio_4_5_mux[] = {
	{
		.offset = PAD_MUX_CONFIG_REG,
		.mask = PMX_I2C_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_plgpio_4_5_modes[] = {
	{
		.ids = 0xffffffff,
		.mux_regs = pmx_plgpio_4_5_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_plgpio_4_5_mux),
	},
};

struct pmx_dev pmx_plgpio_4_5 = {
	.name = "plgpio 4 and 5",
	.modes = pmx_plgpio_4_5_modes,
	.mode_count = ARRAY_SIZE(pmx_plgpio_4_5_modes),
};

/* Pad multiplexing for plgpio_6_9 devices */
static struct pmx_mux_reg pmx_plgpio_6_9_mux[] = {
	{
		.offset = PAD_MUX_CONFIG_REG,
		.mask = PMX_SSP_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_plgpio_6_9_modes[] = {
	{
		.ids = 0xffffffff,
		.mux_regs = pmx_plgpio_6_9_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_plgpio_6_9_mux),
	},
};

struct pmx_dev pmx_plgpio_6_9 = {
	.name = "plgpio 6 to 9",
	.modes = pmx_plgpio_6_9_modes,
	.mode_count = ARRAY_SIZE(pmx_plgpio_6_9_modes),
};

/* Pad multiplexing for plgpio_10_27 devices */
static struct pmx_mux_reg pmx_plgpio_10_27_mux[] = {
	{
		.offset = PAD_MUX_CONFIG_REG,
		.mask = PMX_MII_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_plgpio_10_27_modes[] = {
	{
		.ids = 0xffffffff,
		.mux_regs = pmx_plgpio_10_27_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_plgpio_10_27_mux),
	},
};

struct pmx_dev pmx_plgpio_10_27 = {
	.name = "plgpio 10 to 27",
	.modes = pmx_plgpio_10_27_modes,
	.mode_count = ARRAY_SIZE(pmx_plgpio_10_27_modes),
};

/* Pad multiplexing for plgpio_28 devices */
static struct pmx_mux_reg pmx_plgpio_28_mux[] = {
	{
		.offset = PAD_MUX_CONFIG_REG,
		.mask = PMX_GPIO_PIN0_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_plgpio_28_modes[] = {
	{
		.ids = 0xffffffff,
		.mux_regs = pmx_plgpio_28_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_plgpio_28_mux),
	},
};

struct pmx_dev pmx_plgpio_28 = {
	.name = "plgpio 28",
	.modes = pmx_plgpio_28_modes,
	.mode_count = ARRAY_SIZE(pmx_plgpio_28_modes),
};

/* Pad multiplexing for plgpio_29 devices */
static struct pmx_mux_reg pmx_plgpio_29_mux[] = {
	{
		.offset = PAD_MUX_CONFIG_REG,
		.mask = PMX_GPIO_PIN1_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_plgpio_29_modes[] = {
	{
		.ids = 0xffffffff,
		.mux_regs = pmx_plgpio_29_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_plgpio_29_mux),
	},
};

struct pmx_dev pmx_plgpio_29 = {
	.name = "plgpio 29",
	.modes = pmx_plgpio_29_modes,
	.mode_count = ARRAY_SIZE(pmx_plgpio_29_modes),
};

/* Pad multiplexing for plgpio_30 device */
static struct pmx_mux_reg pmx_plgpio_30_mux[] = {
	{
		.offset = PAD_MUX_CONFIG_REG,
		.mask = PMX_GPIO_PIN2_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_plgpio_30_modes[] = {
	{
		.ids = 0xffffffff,
		.mux_regs = pmx_plgpio_30_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_plgpio_30_mux),
	},
};

struct pmx_dev pmx_plgpio_30 = {
	.name = "plgpio 30",
	.modes = pmx_plgpio_30_modes,
	.mode_count = ARRAY_SIZE(pmx_plgpio_30_modes),
};

/* Pad multiplexing for plgpio_31 device */
static struct pmx_mux_reg pmx_plgpio_31_mux[] = {
	{
		.offset = PAD_MUX_CONFIG_REG,
		.mask = PMX_GPIO_PIN3_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_plgpio_31_modes[] = {
	{
		.ids = 0xffffffff,
		.mux_regs = pmx_plgpio_31_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_plgpio_31_mux),
	},
};

struct pmx_dev pmx_plgpio_31 = {
	.name = "plgpio 31",
	.modes = pmx_plgpio_31_modes,
	.mode_count = ARRAY_SIZE(pmx_plgpio_31_modes),
};

/* Pad multiplexing for plgpio_32 device */
static struct pmx_mux_reg pmx_plgpio_32_mux[] = {
	{
		.offset = PAD_MUX_CONFIG_REG,
		.mask = PMX_GPIO_PIN4_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_plgpio_32_modes[] = {
	{
		.ids = 0xffffffff,
		.mux_regs = pmx_plgpio_32_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_plgpio_32_mux),
	},
};

struct pmx_dev pmx_plgpio_32 = {
	.name = "plgpio 32",
	.modes = pmx_plgpio_32_modes,
	.mode_count = ARRAY_SIZE(pmx_plgpio_32_modes),
};

/* Pad multiplexing for plgpio_33 device */
static struct pmx_mux_reg pmx_plgpio_33_mux[] = {
	{
		.offset = PAD_MUX_CONFIG_REG,
		.mask = PMX_GPIO_PIN5_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_plgpio_33_modes[] = {
	{
		.ids = 0xffffffff,
		.mux_regs = pmx_plgpio_33_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_plgpio_33_mux),
	},
};

struct pmx_dev pmx_plgpio_33 = {
	.name = "plgpio 33",
	.modes = pmx_plgpio_33_modes,
	.mode_count = ARRAY_SIZE(pmx_plgpio_33_modes),
};

/* Pad multiplexing for plgpio_34_36 device */
static struct pmx_mux_reg pmx_plgpio_34_36_mux[] = {
	{
		.offset = PAD_MUX_CONFIG_REG,
		.mask = PMX_SSP_CS_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_plgpio_34_36_modes[] = {
	{
		.ids = 0xffffffff,
		.mux_regs = pmx_plgpio_34_36_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_plgpio_34_36_mux),
	},
};

struct pmx_dev pmx_plgpio_34_36 = {
	.name = "plgpio 34 to 36",
	.modes = pmx_plgpio_34_36_modes,
	.mode_count = ARRAY_SIZE(pmx_plgpio_34_36_modes),
};

/* Pad multiplexing for plgpio_37_42 device */
static struct pmx_mux_reg pmx_plgpio_37_42_mux[] = {
	{
		.offset = PAD_MUX_CONFIG_REG,
		.mask = PMX_UART0_MODEM_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_plgpio_37_42_modes[] = {
	{
		.ids = 0xffffffff,
		.mux_regs = pmx_plgpio_37_42_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_plgpio_37_42_mux),
	},
};

struct pmx_dev pmx_plgpio_37_42 = {
	.name = "plgpio 37 to 42",
	.modes = pmx_plgpio_37_42_modes,
	.mode_count = ARRAY_SIZE(pmx_plgpio_37_42_modes),
};

/* Pad multiplexing for plgpio_43_44_47_48 device */
static struct pmx_mux_reg pmx_plgpio_43_44_47_48_mux[] = {
	{
		.offset = PAD_MUX_CONFIG_REG,
		.mask = PMX_TIMER_1_2_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_plgpio_43_44_47_48_modes[] = {
	{
		.ids = 0xffffffff,
		.mux_regs = pmx_plgpio_43_44_47_48_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_plgpio_43_44_47_48_mux),
	},
};

struct pmx_dev pmx_plgpio_43_44_47_48 = {
	.name = "plgpio 43, 44, 47 and 48",
	.modes = pmx_plgpio_43_44_47_48_modes,
	.mode_count = ARRAY_SIZE(pmx_plgpio_43_44_47_48_modes),
};

/* Pad multiplexing for plgpio_45_46_49_50 device */
static struct pmx_mux_reg pmx_plgpio_45_46_49_50_mux[] = {
	{
		.offset = PAD_MUX_CONFIG_REG,
		.mask = PMX_TIMER_3_4_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_plgpio_45_46_49_50_modes[] = {
	{
		.ids = 0xffffffff,
		.mux_regs = pmx_plgpio_45_46_49_50_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_plgpio_45_46_49_50_mux),
	},
};

struct pmx_dev pmx_plgpio_45_46_49_50 = {
	.name = "plgpio 45, 46, 49 and 50",
	.modes = pmx_plgpio_45_46_49_50_modes,
	.mode_count = ARRAY_SIZE(pmx_plgpio_45_46_49_50_modes),
};
#endif /* CONFIG_MACH_SPEAR310 || CONFIG_MACH_SPEAR320 */

static void __init spear3xx_timer_init(void)
{
	char pclk_name[] = "pll3_48m_clk";
	struct clk *gpt_clk, *pclk;

	/* get the system timer clock */
	gpt_clk = clk_get_sys("gpt0", NULL);
	if (IS_ERR(gpt_clk)) {
		pr_err("%s:couldn't get clk for gpt\n", __func__);
		BUG();
	}

	/* get the suitable parent clock for timer*/
	pclk = clk_get(NULL, pclk_name);
	if (IS_ERR(pclk)) {
		pr_err("%s:couldn't get %s as parent for gpt\n",
				__func__, pclk_name);
		BUG();
	}

	clk_set_parent(gpt_clk, pclk);
	clk_put(gpt_clk);
	clk_put(pclk);

	spear_setup_timer();
}

struct sys_timer spear3xx_timer = {
	.init = spear3xx_timer_init,
};
