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

#include <linux/gpio.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/fsmc.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <linux/spi/flash.h>
#include <linux/spi/spi.h>
#include <mach/generic.h>
#include <mach/spear.h>
#include <plat/fsmc.h>
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
	&ehci0_device,
	&ehci1_device,
	&i2c_device,
	&ohci0_device,
	&ohci1_device,
	&nand_device,
	&rtc_device,
};

static struct spi_board_info __initdata spi_board_info[] = {
};

static void __init spear600_evb_init(void)
{
	unsigned int i;

	/* set nand device's plat data */
	fsmc_nand_set_plat_data(&nand_device, NULL, 0, NAND_SKIP_BBTSCAN,
			FSMC_NAND_BW8);

	/* call spear600 machine init function */
	spear600_init();

	/* Register slave devices on the I2C buses */
	i2c_register_default_devices();

	/* Add Platform Devices */
	platform_add_devices(plat_devs, ARRAY_SIZE(plat_devs));

	/* Add Amba Devices */
	for (i = 0; i < ARRAY_SIZE(amba_devs); i++)
		amba_device_register(amba_devs[i], &iomem_resource);

	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
}

MACHINE_START(SPEAR600, "ST-SPEAR600-EVB")
	.boot_params	=	0x00000100,
	.map_io		=	spear6xx_map_io,
	.init_irq	=	spear6xx_init_irq,
	.timer		=	&spear6xx_timer,
	.init_machine	=	spear600_evb_init,
MACHINE_END
