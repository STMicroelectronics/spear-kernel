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

#include <linux/mtd/nand.h>
#include <linux/mtd/fsmc.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <mach/generic.h>
#include <mach/spear.h>
#include <plat/fsmc.h>

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
	&plgpio_device,
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

	/* Add Platform Devices */
	platform_add_devices(plat_devs, ARRAY_SIZE(plat_devs));

	/* Add Amba Devices */
	for (i = 0; i < ARRAY_SIZE(amba_devs); i++)
		amba_device_register(amba_devs[i], &iomem_resource);
}

MACHINE_START(SPEAR310, "ST-SPEAR310-EVB")
	.boot_params	=	0x00000100,
	.map_io		=	spear3xx_map_io,
	.init_irq	=	spear3xx_init_irq,
	.timer		=	&spear3xx_timer,
	.init_machine	=	spear310_evb_init,
MACHINE_END
