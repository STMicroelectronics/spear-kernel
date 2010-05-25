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

#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <mach/generic.h>
#include <mach/spear.h>
#include <plat/adc.h>
#include <plat/jpeg.h>
#include <plat/smi.h>

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
}

MACHINE_START(SPEAR320, "ST-SPEAR320-EVB")
	.boot_params	=	0x00000100,
	.map_io		=	spear3xx_map_io,
	.init_irq	=	spear3xx_init_irq,
	.timer		=	&spear3xx_timer,
	.init_machine	=	spear320_evb_init,
MACHINE_END
