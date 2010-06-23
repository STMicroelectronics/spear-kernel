/*
 * arch/arm/mach-spear6xx/spear600_evb.c
 *
 * SPEAr600 evaluation board source file
 *
 * Copyright (C) 2009 ST Microelectronics
 * Viresh Kumar<viresh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <linux/mtd/nand.h>
#include <linux/gpio.h>
#include <linux/spi/flash.h>
#include <linux/spi/spi.h>
#include <mach/generic.h>
#include <mach/gpio.h>
#include <mach/spear.h>
#include <plat/adc.h>
#include <plat/jpeg.h>
#include <plat/nand.h>
#include <plat/smi.h>
#include <plat/spi.h>

static struct amba_device *amba_devs[] __initdata = {
	&clcd_device,
	&gpio_device[0],
	&gpio_device[1],
	&gpio_device[2],
	&ssp_device[0],
	&ssp_device[1],
	&ssp_device[2],
	&uart_device[0],
	&uart_device[1],
	&wdt_device,
};

static struct platform_device *plat_devs[] __initdata = {
	&adc_device,
	&dmac_device,
	&ehci0_device,
	&ehci1_device,
	&eth_device,
	&i2c_device,
	&ohci0_device,
	&ohci1_device,
	&phy_device,
	&jpeg_device,
	&nand_device,
	&rtc_device,
	&smi_device,
};

/* Currently no gpios are free on eval board so it is kept commented */
#if 0
/* spi board information */
/* spi0 flash Chip Select Control function, controlled by gpio pin mentioned */
DECLARE_SPI_CS_CONTROL(0, flash, /* mention gpio number here */);
/* spi0 flash Chip Info structure */
DECLARE_SPI_CHIP_INFO(0, flash, spi0_flash_cs_control);

/* spi0 spidev Chip Select Control function, controlled by gpio pin mentioned */
DECLARE_SPI_CS_CONTROL(0, dev, /* mention gpio number here */);
/* spi0 spidev Chip Info structure */
DECLARE_SPI_CHIP_INFO(0, dev, spi0_dev_cs_control);
#endif

static struct spi_board_info __initdata spi_board_info[] = {
#if 0
	/* spi0 board info */
	{
		.modalias = "spidev",
		.controller_data = &spi0_dev_chip_info,
		.max_speed_hz = 10000000,
		.bus_num = 0,
		.chip_select = 0,
		.mode = 0,
	}, {
		.modalias = "m25p80",
		.controller_data = &spi0_flash_chip_info,
		.max_speed_hz = 400000,
		.bus_num = 0,
		.chip_select = 1,
		.mode = 0,
	}
#endif
	/* Similarly can be done for other spi masters */
};

static void __init spi_init(void)
{
	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
}

static void __init spear600_evb_init(void)
{
	/* set adc platform data */
	set_adc_plat_data(&adc_device, &dmac_device.dev);

	/* set jpeg configurations for DMA xfers */
	set_jpeg_dma_configuration(&jpeg_device, &dmac_device.dev);

	/* set nand device's plat data */
	nand_set_plat_data(&nand_device, NULL, 0, NAND_SKIP_BBTSCAN,
			SPEAR_NAND_BW8);

	/* call spear600 machine init function */
	spear600_init();

	/* Register slave devices on the I2C buses */
	i2c_register_board_devices();

	/* initialize serial nor related data in smi plat data */
	smi_init_board_info(&smi_device);

	/* Add Platform Devices */
	platform_add_devices(plat_devs, ARRAY_SIZE(plat_devs));

	/* Add Amba Devices */
	spear_amba_device_register(amba_devs, ARRAY_SIZE(amba_devs));

	spi_init();
}

MACHINE_START(SPEAR600, "ST-SPEAR600-EVB")
	.boot_params	=	0x00000100,
	.map_io		=	spear6xx_map_io,
	.init_irq	=	spear6xx_init_irq,
	.timer		=	&spear6xx_timer,
	.init_machine	=	spear600_evb_init,
MACHINE_END
