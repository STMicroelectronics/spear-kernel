/*
 * arch/arm/mach-spear13xx/spear900_lcad.c
 *
 * SPEAr900 Low Cost Access Device board source file
 *
 * Copyright (C) 2011 ST Microelectronics
 * Viresh Kumar <viresh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/mtd/fsmc.h>
#include <linux/mtd/nand.h>
#include <linux/spi/spi.h>
#include <linux/stmpe610.h>
#include <asm/mach-types.h>
#include <asm/setup.h>
#include <plat/adc.h>
#include <plat/fsmc.h>
#include <plat/jpeg.h>
#include <plat/spi.h>
#include <mach/db9000fb_info.h>
#include <mach/generic.h>
#include <mach/gpio.h>
#include <mach/hardware.h>

#ifdef CONFIG_ANDROID_PMEM
#include <linux/android_pmem.h>
#endif

#ifdef CONFIG_USB_ANDROID
#include <linux/usb/android_composite.h>
#endif

/* SPEAr GPIO Buttons Info */
#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <mach/misc_regs.h>
#include <linux/gpio.h>

/* SPEAr GPIO Buttons definition */
#define SPEAR_GPIO_BTN7	7
#define SPEAR_GPIO_BTN5	5
#define SPEAR_GPIO_BTN6	6
#define SPEAR_GPIO_BTN8	8

static struct gpio_keys_button spear_gpio_keys_table[] = {
	{
		.code = KEY_POWER, 
		.gpio = SPEAR_GPIO_BTN7, 
		.active_low = 1, 
		.desc = "gpio-keys: BTN0:POWER",
		.type = EV_KEY,
		.wakeup = 1,
		.debounce_interval = 20,
	},
	{
		.code = KEY_BACK, 
		.gpio = SPEAR_GPIO_BTN5, 
		.active_low = 1, 
		.desc = "gpio-keys: BTN1:BACK",
		.type = EV_KEY,
		.wakeup = 0,
		.debounce_interval = 20,
	},
	{
		.code = KEY_MENU, 
		.gpio = SPEAR_GPIO_BTN6, 
		.active_low = 1, 
		.desc = "gpio-keys: BTN2:MENU",
		.type = EV_KEY,
		.wakeup = 0,
		.debounce_interval = 20,
	},
	{
		.code = KEY_HOME, 
		.gpio = SPEAR_GPIO_BTN8, 
		.active_low = 1, 
		.desc = "gpio-keys: BTN3:HOME",
		.type = EV_KEY,
		.wakeup = 0,
		.debounce_interval = 20,
	},
};
 
static struct gpio_keys_platform_data spear_gpio_keys_data = {
	.buttons        = spear_gpio_keys_table,
	.nbuttons       = ARRAY_SIZE(spear_gpio_keys_table),
};
 
static struct platform_device spear900_device_gpiokeys = {
	.name      = "gpio-keys",
	.dev = {
		.platform_data = &spear_gpio_keys_data,
	},
};

static void spear900_gpio7_fixup(void)
{
		u32 pm_cfg;
	
		pm_cfg = readl(VA_PCM_CFG);
		/* source gpio interrupt through GIC */
		pm_cfg &= ~(1 << 2);
		writel(pm_cfg, VA_PCM_CFG);
}
#endif

#define PARTITION(n, off, sz)	{.name = n, .offset = off, .size = sz}

/* NAND partition table */
static struct mtd_partition partition_info[] = {
	PARTITION("X-loader", 0, 4 * 0x20000),
	PARTITION("U-Boot", 0x80000, 3 * 0x20000),
	PARTITION("Kernel", 0xe0000, 57 * 0x20000),
#ifdef CONFIG_ANDROID
	PARTITION("Root File System", 0x800000, 2000 * 0x20000),
	PARTITION("System", 0x10200000, 4500 * 0x20000),
	PARTITION("Data", 0x33480000, 1000 * 0x20000),
	PARTITION("Cache", 0x3B180000, 628 * 0x20000),
#else
	PARTITION("Root File System", 0x800000, 8128 * 0x20000),
#endif
};

#ifdef CONFIG_ANDROID_PMEM
static int __init early_pmem_generic_parse(char *p, struct android_pmem_platform_data * data)
{
	data->size = memparse(p, &p);
	if (*p == '@')
		data->start = memparse(p + 1, &p);

	return 0;
}

/********************************************************************************
 * Pmem device used by surface flinger
 ********************************************************************************/

static struct android_pmem_platform_data pmem_pdata = {
	.name = "pmem",
	.no_allocator = 1,	/* MemoryHeapBase is having an allocator */
	.cached = 1,
	.start = 0,
	.size = 0,
};

static int __init early_pmem(char *p)
{
	return early_pmem_generic_parse(p, &pmem_pdata);
}
early_param("pmem", early_pmem);

static struct platform_device SPEAr_pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = {
		.platform_data = &pmem_pdata,
	},
};
#endif

#ifdef CONFIG_USB_ANDROID
#define VENDOR_ID  0x04CC
#define PRODUCT_ID  0x0001
#define ADB_PRODUCT_ID 0x0002

static char *usb_functions_adb[] = {
#ifdef CONFIG_USB_ANDROID_ADB
       "adb",
#endif
};


static struct android_usb_product usb_products[] = {
       {
               .product_id     = PRODUCT_ID,
               .num_functions  = ARRAY_SIZE(usb_functions_adb),
               .functions      = usb_functions_adb,
       },
};

static struct android_usb_platform_data android_usb_pdata = {
       .vendor_id      = VENDOR_ID,
       .product_id     = ADB_PRODUCT_ID,
       .version        = 0x0100,
       .product_name   = "Android SPEAr",
       .manufacturer_name = "STM_CCI",
       .num_products = ARRAY_SIZE(usb_products),
       .products = usb_products,
       .num_functions = ARRAY_SIZE(usb_functions_adb),
       .functions = usb_functions_adb,
};

static struct platform_device android_usb_device = {
       .name   = "android_usb",
       .id             = -1,
       .dev            = {
       .platform_data = &android_usb_pdata,
       },
};
#endif /* CONFIG_USB_ANDROID */

/* padmux devices to enable */
static struct pmx_dev *pmx_devs[] = {
	/* spear13xx specific devices */
	&spear13xx_pmx_i2c,
	&spear13xx_pmx_i2s1,
	&spear13xx_pmx_i2s2,
	&spear13xx_pmx_clcd,
	&spear13xx_pmx_egpio_grp,
	&spear13xx_pmx_mcif,
	&spear13xx_pmx_nand_8bit,
	&spear13xx_pmx_ssp,
	&spear13xx_pmx_uart0,
	&spear13xx_pmx_sdhci,

	/* spear900 specific devices */
};

static struct amba_device *amba_devs[] __initdata = {
	&spear13xx_gpio_device[0],
	&spear13xx_gpio_device[1],
	&spear13xx_ssp_device,
	&spear13xx_uart_device,
};

static struct platform_device *plat_devs[] __initdata = {
	/* spear13xx specific devices */
	&spear13xx_adc_device,
	&spear13xx_db9000_clcd_device,
	&spear13xx_dmac_device[0],
	&spear13xx_dmac_device[1],
	&spear13xx_ehci0_device,
	&spear13xx_ehci1_device,
	&spear13xx_i2c_device,
	&spear13xx_i2s0_device,
	&spear13xx_jpeg_device,
	&spear13xx_nand_device,
	&spear13xx_ohci0_device,
	&spear13xx_ohci1_device,
	&spear13xx_pcm_device,
	&spear13xx_rtc_device,
	&spear13xx_sdhci_device,
	&spear13xx_wdt_device,
	
	/* spear900 specific devices */
#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
	&spear900_device_gpiokeys,
#endif

#ifdef CONFIG_ANDROID_PMEM
    	&SPEAr_pmem_device,
#endif

#ifdef CONFIG_USB_ANDROID
  	&android_usb_device,
#endif
};

/* spi0 touch screen Chip Select Control function, controlled by gpio pin */
static struct stmpe610_pdata stmpe610_spi_pdata = {
	.irq_gpio = GPIO1_6,
	.irq_type = IRQ_TYPE_EDGE_FALLING,
	.fifo_threshhold = 1,
	.tracking_index = TI_64,
	.operating_mode = XYZ_ACQUISITION,
	.average_ctrl = SAMPLES_4,
	.touch_det_delay = TD_1MS,
	.settling_time = ST_5MS,
	.x_min = 0x00,
	.x_max = 0xFFF,
	.y_min = 0x00,
	.y_max = 0xFFF,
	.sample_time = SAMP_TIME_80,
	.mod_12b = MOD_12B,
	.ref_sel = REF_SEL_INT,
	.adc_freq = ADC_FREQ_3250K,
	.fraction_z = 7,
	.i_drive = IDRIVE_50_80MA,
};

DECLARE_SPI_CS_GPIO_CONTROL(0, ts, GPIO1_7);
/* spi0 touch screen Info structure */
static struct pl022_config_chip spi0_ts_chip_info = {
	.iface = SSP_INTERFACE_MOTOROLA_SPI,
	.hierarchy = SSP_MASTER,
	.slave_tx_disable = 0,
	.com_mode = INTERRUPT_TRANSFER,
	.rx_lev_trig = SSP_RX_1_OR_MORE_ELEM,
	.tx_lev_trig = SSP_TX_1_OR_MORE_EMPTY_LOC,
	.ctrl_len = SSP_BITS_8,
	.wait_state = SSP_MWIRE_WAIT_ZERO,
	.duplex = SSP_MICROWIRE_CHANNEL_FULL_DUPLEX,
	.cs_control = spi0_ts_cs_gpio_control,
};

static struct spi_board_info __initdata spi_board_info[] = {
	/* spi0 board info */
	{
		.modalias = "stmpe610-spi",
		.platform_data = &stmpe610_spi_pdata,
		.controller_data = &spi0_ts_chip_info,
		.max_speed_hz = 1000000,
		.bus_num = 0,
		.chip_select = 0,
		.mode = SPI_MODE_1,
	},
};
	
static void spear900_lcad_fixup(struct machine_desc *desc, struct tag *tags,
		char **cmdline, struct meminfo *mi)
{
#if defined(CONFIG_FB_DB9000) || defined(CONFIG_FB_DB9000_MODULE)
	unsigned long size;

	size = clcd_get_fb_size(&hannstar_hsd07_info, NUM_OF_FRAMEBUFFERS);
	hannstar_hsd07_info.frame_buf_base =
		reserve_mem(mi, ALIGN(size, SZ_1M));
	if (hannstar_hsd07_info.frame_buf_base == ~0)
		pr_err("Unable to allocate fb buffer\n");
#endif
}

static void __init spear900_lcad_init(void)
{
	unsigned int i;

	/* set adc platform data */
	set_adc_plat_data(&spear13xx_adc_device, &spear13xx_dmac_device[0].dev);

#if (defined(CONFIG_FB_DB9000) || defined(CONFIG_FB_DB9000_MODULE))
	/* db9000_clcd plat data */
	clcd_set_plat_data(&spear13xx_db9000_clcd_device,
			&hannstar_hsd07_info);
#endif
	/* set jpeg configurations for DMA xfers */
	set_jpeg_dma_configuration(&spear13xx_jpeg_device,
			&spear13xx_dmac_device[0].dev);

	/* set nand device's plat data */
	fsmc_nand_set_plat_data(&spear13xx_nand_device, partition_info,
			ARRAY_SIZE(partition_info), NAND_SKIP_BBTSCAN,
			FSMC_NAND_BW8);
	nand_mach_init(FSMC_NAND_BW8);

#ifdef CONFIG_SND_SOC_STA529
	/* configure i2s configuration for dma xfer */
	pcm_init(&spear13xx_dmac_device[0].dev);
#endif

	/* call spear900 machine init function */
	spear900_init(NULL, pmx_devs, ARRAY_SIZE(pmx_devs));

	/* Register slave devices on the I2C buses */
	i2c_register_default_devices();

	/* Add Platform Devices */
	platform_add_devices(plat_devs, ARRAY_SIZE(plat_devs));

	/* Add Amba Devices */
	for (i = 0; i < ARRAY_SIZE(amba_devs); i++)
		amba_device_register(amba_devs[i], &iomem_resource);

	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
	
	/* SPEAr GPIO Button 7 Fix */
#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
	spear900_gpio7_fixup();
#endif
}

MACHINE_START(SPEAR900_LCAD, "ST-SPEAR900-LCAD")
	.boot_params	=	0x00000100,
	.fixup		=	spear900_lcad_fixup,
	.map_io		=	spear13xx_map_io,
	.init_irq	=	spear13xx_init_irq,
	.timer		=	&spear13xx_timer,
	.init_machine	=	spear900_lcad_init,
MACHINE_END
