/*
 * arch/arm/mach-spear3xx/spear310_evb.c
 *
 * SPEAr310 evaluation board source file
 *
 * Copyright (C) 2009 ST Microelectronics
 * Viresh Kumar<viresh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/gpio.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/fsmc.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <linux/spi/flash.h>
#include <linux/spi/spi.h>
#include <mach/emi.h>
#include <mach/generic.h>
#include <mach/spear.h>
#include <plat/fsmc.h>
#include <plat/spi.h>

#define PARTITION(n, off, sz)	{.name = n, .offset = off, .size = sz}

static struct mtd_partition partition_info[] = {
	PARTITION("X-loader", 0, 1 * 0x20000),
	PARTITION("U-Boot", 0x20000, 3 * 0x20000),
	PARTITION("Kernel", 0x80000, 24 * 0x20000),
	PARTITION("Root File System", 0x380000, 84 * 0x20000),
};

/* emi nor flash resources registeration */
static struct resource emi_nor_resources[] = {
	{
		.start	= SPEAR310_EMI_MEM_0_BASE,
		.end	= SPEAR310_EMI_MEM_0_BASE + SPEAR310_EMI_MEM_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

/* padmux devices to enable */
static struct pmx_dev *pmx_devs[] = {
	/* spear3xx specific devices */
	&pmx_i2c,
	&pmx_ssp,
	&pmx_gpio_pin0,
	&pmx_gpio_pin1,
	&pmx_gpio_pin2,
	&pmx_gpio_pin3,
	&pmx_gpio_pin4,
	&pmx_gpio_pin5,
	&pmx_uart0,

	/* spear310 specific devices */
	&pmx_emi_cs_0_1_4_5,
	&pmx_emi_cs_2_3,
	&pmx_uart1,
	&pmx_uart2,
	&pmx_uart3_4_5,
	&pmx_fsmc,
	&pmx_rs485_0_1,
	&pmx_tdm0,
};

static struct amba_device *amba_devs[] __initdata = {
	/* spear3xx specific devices */
	&gpio_device,
	&ssp0_device,
	&uart_device,
	&wdt_device,

	/* spear310 specific devices */
};

static struct platform_device *plat_devs[] __initdata = {
	/* spear3xx specific devices */
	&ehci_device,
	&i2c_device,
	&nand_device,
	&ohci0_device,
	&ohci1_device,
	&rtc_device,

	/* spear310 specific devices */
	&emi_nor_device,
	&plgpio_device,
};

/* spi board information */
/* spi0 flash Chip Select Control function, controlled by gpio pin mentioned */
DECLARE_SPI_CS_CONTROL(0, flash, BASIC_GPIO_3);
/* spi0 flash Chip Info structure */
DECLARE_SPI_CHIP_INFO(0, flash, spi0_flash_cs_control);

/* spi0 spidev Chip Select Control function, controlled by gpio pin mentioned */
DECLARE_SPI_CS_CONTROL(0, dev, BASIC_GPIO_4);
/* spi0 spidev Chip Info structure */
DECLARE_SPI_CHIP_INFO(0, dev, spi0_dev_cs_control);

static struct spi_board_info __initdata spi_board_info[] = {
	/* spi0 board info */
	{
		.modalias = "spidev",
		.controller_data = &spi0_dev_chip_info,
		.max_speed_hz = 25000000,
		.bus_num = 0,
		.chip_select = 0,
		.mode = SPI_MODE_1,
	}, {
		.modalias = "m25p80",
		.controller_data = &spi0_flash_chip_info,
		.max_speed_hz = 25000000,
		.bus_num = 0,
		.chip_select = 1,
		.mode = SPI_MODE_1,
	}
};

static void __init spear310_evb_init(void)
{
	unsigned int i;

	/* padmux initialization, must be done before spear310_init */
	pmx_driver.mode = NULL;
	pmx_driver.devs = pmx_devs;
	pmx_driver.devs_count = ARRAY_SIZE(pmx_devs);

	/* set nand device's plat data */
	fsmc_nand_set_plat_data(&nand_device, NULL, 0, NAND_SKIP_BBTSCAN,
			FSMC_NAND_BW8);

	/* call spear310 machine init function */
	spear310_init();

	/* Register slave devices on the I2C buses */
	i2c_register_default_devices();

	/* initialize emi related data in emi plat data */
	emi_init_board_info(&emi_nor_device, emi_nor_resources,
			ARRAY_SIZE(emi_nor_resources), partition_info,
			ARRAY_SIZE(partition_info), EMI_FLASH_WIDTH32);

	/* Initialize emi regiters */
	emi_init(&emi_nor_device, SPEAR310_EMI_REG_BASE, 0, EMI_FLASH_WIDTH32);

	/* Add Platform Devices */
	platform_add_devices(plat_devs, ARRAY_SIZE(plat_devs));

	/* Add Amba Devices */
	for (i = 0; i < ARRAY_SIZE(amba_devs); i++)
		amba_device_register(amba_devs[i], &iomem_resource);

	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
}

MACHINE_START(SPEAR310, "ST-SPEAR310-EVB")
	.boot_params	=	0x00000100,
	.map_io		=	spear3xx_map_io,
	.init_irq	=	spear3xx_init_irq,
	.timer		=	&spear3xx_timer,
	.init_machine	=	spear310_evb_init,
MACHINE_END
