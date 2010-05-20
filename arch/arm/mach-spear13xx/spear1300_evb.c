/*
 * arch/arm/mach-spear13xx/spear1300_evb.c
 *
 * SPEAr1300 evaluation board source file
 *
 * Copyright (C) 2010 ST Microelectronics
 * Shiraz Hashim <shiraz.hashim@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/types.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <mach/generic.h>
#include <mach/spear.h>
#include <plat/adc.h>
#include <plat/jpeg.h>
#include <plat/keyboard.h>
#include <plat/smi.h>

static struct amba_device *amba_devs[] __initdata = {
	&gpio_device[0],
	&gpio_device[1],
	&uart_device,
};

static struct platform_device *plat_devs[] __initdata = {
	&adc_device,
	&dmac_device[0],
	&dmac_device[1],
	&ehci0_device,
	&ehci1_device,
	&eth_device,
	&i2c_device,
	&jpeg_device,
	&kbd_device,
	&ohci0_device,
	&ohci1_device,
	&phy_device,
	&rtc_device,
	&smi_device,
	&wdt_device,
};

/* keyboard specific platform data */
static DECLARE_KEYMAP(spear_keymap);

static struct kbd_platform_data kbd_data = {
	.keymap = spear_keymap,
	.keymapsize = ARRAY_SIZE(spear_keymap),
	.rep = 1,
};

static void __init spear1300_evb_init(void)
{
	unsigned int i;

	/* set adc platform data */
	set_adc_plat_data(&adc_device, &dmac_device[0].dev);

	/* set keyboard plat data */
	kbd_set_plat_data(&kbd_device, &kbd_data);

	/* set jpeg configurations for DMA xfers */
	set_jpeg_dma_configuration(&jpeg_device, &dmac_device[0].dev);

	/* call spear1300 machine init function */
	spear1300_init();

	/* Register slave devices on the I2C buses */
	i2c_register_board_devices();

	/* initialize serial nor related data in smi plat data */
	smi_init_board_info(&smi_device);

	/* Add Platform Devices */
	platform_add_devices(plat_devs, ARRAY_SIZE(plat_devs));

	/* Add Amba Devices */
	for (i = 0; i < ARRAY_SIZE(amba_devs); i++)
		amba_device_register(amba_devs[i], &iomem_resource);
}

MACHINE_START(SPEAR1300, "ST-SPEAR1300-EVB")
	.boot_params	=	0x00000100,
	.map_io		=	spear13xx_map_io,
	.init_irq	=	spear13xx_init_irq,
	.timer		=	&spear13xx_timer,
	.init_machine	=	spear1300_evb_init,
MACHINE_END
