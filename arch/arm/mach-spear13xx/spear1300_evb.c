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
#include <linux/gpio.h>
#include <linux/mtd/nand.h>
#include <linux/spi/flash.h>
#include <linux/spi/spi.h>
#include <linux/mtd/fsmc.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <mach/generic.h>
#include <mach/spear.h>
#include <mach/pcie.h>
#include <plat/keyboard.h>
#include <plat/fsmc.h>
#include <plat/smi.h>
#include <plat/spi.h>

/* fsmc nor partition info */
#if 0
#define PARTITION(n, off, sz)	{.name = n, .offset = off, .size = sz}
static struct mtd_partition partition_info[] = {
	PARTITION("X-loader", 0, 1 * 0x20000),
	PARTITION("U-Boot", 0x20000, 3 * 0x20000),
	PARTITION("Kernel", 0x80000, 24 * 0x20000),
	PARTITION("Root File System", 0x380000, 84 * 0x20000),
};
#endif

/* padmux devices to enable */
static struct pmx_dev *pmx_devs[] = {
	/* spear13xx specific devices */
	&pmx_i2c,
	&pmx_i2s1,
	&pmx_i2s2,
	&pmx_clcd,
	&pmx_egpio_grp,
	&pmx_gmii,
	&pmx_keyboard_6x6,
	&pmx_mcif,
	&pmx_nand_8bit,
	&pmx_smi_4_chips,
	&pmx_ssp,
	&pmx_uart0,

	/* spear1300 specific devices */
};

static struct amba_device *amba_devs[] __initdata = {
	&spear13xx_gpio_device[0],
	&spear13xx_gpio_device[1],
	&spear13xx_ssp_device,
	&spear13xx_uart_device,
};

static struct platform_device *plat_devs[] __initdata = {
	&spear13xx_ehci0_device,
	&spear13xx_ehci1_device,
	&spear13xx_i2c_device,
	&spear13xx_kbd_device,
	&spear13xx_nand_device,
	&spear13xx_ohci0_device,
	&spear13xx_ohci1_device,
	&spear13xx_pcie_gadget0_device,
	&spear13xx_rtc_device,
	&spear13xx_sdhci_device,
	&spear13xx_smi_device,
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

static struct spi_board_info __initdata spi_board_info[] = {
};

#ifdef CONFIG_PCIEPORTBUS
/*
 * This function is needed for PCIE host and device driver. Same
 * controller can not be programmed as host as well as device. So host
 * driver must call this function and if this function returns 1 then
 * only host should add that particular port as RC.
 * A port to be added as device, one must also add device's information
 * in plat_devs array defined in this file.
 */
static int spear1300_pcie_port_is_host(int port)
{
	switch (port) {
	case 0:
		return 0;
	case 1:
		return 1;
	case 2:
		return 1;
	}

	return -EINVAL;
}
#endif

static void __init spear1300_evb_init(void)
{
	unsigned int i;

	/* set keyboard plat data */
	kbd_set_plat_data(&spear13xx_kbd_device, &kbd_data);

	/* set nand device's plat data */
	fsmc_nand_set_plat_data(&spear13xx_nand_device, NULL, 0,
			NAND_SKIP_BBTSCAN, FSMC_NAND_BW8);
	nand_mach_init(FSMC_NAND_BW8);

	/*
	 * FSMC cannot used as NOR and NAND at the same time For the moment,
	 * disable NOR and use NAND only. If NOR is needed, enable the following
	 * code and disable all code for NAND. Also enable nand in padmux
	 * configuration to use it
	 */
#if 0
	/* initialize fsmc related data in fsmc plat data */
	fsmc_init_board_info(&spear13xx_fsmc_nor_device, partition_info,
			ARRAY_SIZE(partition_info), FSMC_FLASH_WIDTH8);

	/* Initialize fsmc regiters */
	fsmc_nor_init(&spear13xx_fsmc_nor_device, SPEAR13XX_FSMC_BASE, 0,
			FSMC_FLASH_WIDTH8);
#endif

	/* call spear1300 machine init function */
	spear1300_init(NULL, pmx_devs, ARRAY_SIZE(pmx_devs));

	/* Register slave devices on the I2C buses */
	i2c_register_default_devices();

#ifdef CONFIG_PCIEPORTBUS
	/* Enable PCIE0 clk */
	enable_pcie0_clk();
	pcie_init(spear1300_pcie_port_is_host);
#endif

	/* initialize serial nor related data in smi plat data */
	smi_init_board_info(&spear13xx_smi_device);

	/* Add Platform Devices */
	platform_add_devices(plat_devs, ARRAY_SIZE(plat_devs));

	/* Add Amba Devices */
	for (i = 0; i < ARRAY_SIZE(amba_devs); i++)
		amba_device_register(amba_devs[i], &iomem_resource);

	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
}

MACHINE_START(SPEAR1300, "ST-SPEAR1300-EVB")
	.boot_params	=	0x00000100,
	.map_io		=	spear13xx_map_io,
	.init_irq	=	spear13xx_init_irq,
	.timer		=	&spear13xx_timer,
	.init_machine	=	spear1300_evb_init,
MACHINE_END
