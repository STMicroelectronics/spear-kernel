/*
 * arch/arm/mach-spear3xx/spear300_evb.c
 *
 * SPEAr300 evaluation board source file
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
#include <plat/keyboard.h>

/* padmux devices to enable */
static struct pmx_dev *pmx_devs[] = {
	/* spear3xx specific devices */
	&pmx_i2c,
	&pmx_ssp_cs,
	&pmx_ssp,
	&pmx_mii,
	&pmx_uart0,

	/* spear300 specific devices */
	&pmx_fsmc_2_chips,
	&pmx_clcd,
	&pmx_telecom_sdio_4bit,
	&pmx_gpio1,
};

static struct amba_device *amba_devs[] __initdata = {
	/* spear3xx specific devices */
	&gpio_device,
	&uart_device,
	&wdt_device,

	/* spear300 specific devices */
	&clcd_device,
	&gpio1_device,
};

static struct platform_device *plat_devs[] __initdata = {
	/* spear3xx specific devices */
	&ehci_device,
	&i2c_device,
	&nand0_device,
	&ohci0_device,
	&ohci1_device,
	&rtc_device,

	/* spear300 specific devices */
	&kbd_device,
};

/* keyboard specific platform data */
static DECLARE_KEYMAP(keymap);
static struct matrix_keymap_data keymap_data = {
	.keymap = keymap,
	.keymap_size = ARRAY_SIZE(keymap),
};

static struct kbd_platform_data kbd_data = {
	.keymap = &keymap_data,
	.rep = 1,
};

static void __init spear300_evb_init(void)
{
	unsigned int i;

	/* padmux initialization, must be done before spear300_init */
	pmx_driver.mode = &photo_frame_mode;
	pmx_driver.devs = pmx_devs;
	pmx_driver.devs_count = ARRAY_SIZE(pmx_devs);

	/* set keyboard plat data */
	kbd_set_plat_data(&kbd_device, &kbd_data);

	/* set nand0 device's plat data */
	fsmc_nand_set_plat_data(&nand0_device, NULL, 0, NAND_SKIP_BBTSCAN,
			FSMC_NAND_BW8);

	/* call spear300 machine init function */
	spear300_init();

	/* Register slave devices on the I2C buses */
	i2c_register_default_devices();

	/* Add Platform Devices */
	platform_add_devices(plat_devs, ARRAY_SIZE(plat_devs));

	/* Add Amba Devices */
	for (i = 0; i < ARRAY_SIZE(amba_devs); i++)
		amba_device_register(amba_devs[i], &iomem_resource);
}

MACHINE_START(SPEAR300, "ST-SPEAR300-EVB")
	.boot_params	=	0x00000100,
	.map_io		=	spear3xx_map_io,
	.init_irq	=	spear3xx_init_irq,
	.timer		=	&spear3xx_timer,
	.init_machine	=	spear300_evb_init,
MACHINE_END
