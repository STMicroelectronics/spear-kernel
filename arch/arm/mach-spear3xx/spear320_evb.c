/*
 * arch/arm/mach-spear3xx/spear320_evb.c
 *
 * SPEAr320 evaluation board source file
 *
 * Copyright (C) 2009 ST Microelectronics
 * Viresh Kumar<viresh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/mtd/nand.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>
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

/* padmux devices to enable */
static struct pmx_dev *pmx_devs[] = {
	/* spear3xx specific devices */
	&pmx_i2c,
	&pmx_ssp,
	&pmx_mii,
	&pmx_uart0,

	/* spear320 specific devices */
	&pmx_fsmc,
	&pmx_sdio,
	&pmx_i2s,
	&pmx_uart1,
	&pmx_uart2,
	&pmx_can,
	&pmx_pwm0,
	&pmx_pwm1,
	&pmx_pwm2,
	&pmx_mii1,
};

static struct amba_device *amba_devs[] __initdata = {
	/* spear3xx specific devices */
	&gpio_device,
	&uart_device,
	&wdt_device,

	/* spear320 specific devices */
	&clcd_device,
};

static struct platform_device *plat_devs[] __initdata = {
	/* spear3xx specific devices */
	&adc_device,
	&dmac_device,
	&ehci_device,
	&eth_device,
	&i2c_device,
	&jpeg_device,
	&nand_device,
	&ohci0_device,
	&ohci1_device,
	&phy_device,
	&rtc_device,
	&smi_device,

	/* spear320 specific devices */
	&i2c1_device,
	&plgpio_device,
	&pwm_device,
};

/* Currently no gpios are free on eval board so it is kept commented */
#if 0
/* spi board information */
static const struct flash_platform_data spix_flash_data = {
	.type = "m25p40",
};

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
		.platform_data = &spix_flash_data,
		.controller_data = &spi0_flash_chip_info,
		.max_speed_hz = 10000000,
		.bus_num = 0,
		.chip_select = 1,
		.mode = 0,
	}
#endif
};

static void spi_init(void)
{
	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
}

static void __init spear320_evb_init(void)
{
	unsigned int i;

	/* padmux initialization, must be done before spear320_init */
	pmx_driver.mode = &auto_net_mii_mode;
	pmx_driver.devs = pmx_devs;
	pmx_driver.devs_count = ARRAY_SIZE(pmx_devs);

	/* set adc platform data */
	set_adc_plat_data(&adc_device, &dmac_device.dev);

	/* set jpeg configurations for DMA xfers */
	set_jpeg_dma_configuration(&jpeg_device, &dmac_device.dev);

	/* set nand device's plat data */
	nand_set_plat_data(&nand_device, NULL, 0, NAND_SKIP_BBTSCAN,
			SPEAR_NAND_BW8);

	/* call spear320 machine init function */
	spear320_init();

	/* initialize serial nor related data in smi plat data */
	smi_init_board_info(&smi_device);

	/* Register slave devices on the I2C buses */
	i2c_register_board_devices();

	/* Add Platform Devices */
	platform_add_devices(plat_devs, ARRAY_SIZE(plat_devs));

	/* Add Amba Devices */
	for (i = 0; i < ARRAY_SIZE(amba_devs); i++)
		amba_device_register(amba_devs[i], &iomem_resource);

	spi_init();
}

MACHINE_START(SPEAR320, "ST-SPEAR320-EVB")
	.boot_params	=	0x00000100,
	.map_io		=	spear3xx_map_io,
	.init_irq	=	spear3xx_init_irq,
	.timer		=	&spear3xx_timer,
	.init_machine	=	spear320_evb_init,
MACHINE_END
