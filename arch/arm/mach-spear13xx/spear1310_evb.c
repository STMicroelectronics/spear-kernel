/*
 * arch/arm/mach-spear13xx/spear1310_evb.c
 *
 * SPEAr1310 evaluation board source file
 *
 * Copyright (C) 2010 ST Microelectronics
 * Bhupesh Sharma <bhupesh.sharma@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/types.h>
#include <linux/gpio.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/fsmc.h>
#include <linux/spi/flash.h>
#include <linux/spi/spi.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <mach/generic.h>
#include <mach/spear.h>
#include <mach/pcie.h>
#include <plat/keyboard.h>
#include <plat/fsmc.h>
#include <plat/spi.h>

#define PARTITION(n, off, sz)	{.name = n, .offset = off, .size = sz}

static struct mtd_partition partition_info[] = {
	PARTITION("X-loader", 0, 1 * 0x20000),
	PARTITION("U-Boot", 0x20000, 3 * 0x20000),
	PARTITION("Kernel", 0x80000, 24 * 0x20000),
	PARTITION("Root File System", 0x380000, 84 * 0x20000),
};

/* padmux devices to enable */
static struct pmx_dev *pmx_devs[] = {
	/* spear13xx specific devices */
	&pmx_i2c,
	&pmx_i2s1,
	&pmx_egpio_grp,
	&pmx_gmii,
	&pmx_keyboard_6x6,
	&pmx_mcif,
	&pmx_nand_8bit,
	&pmx_smi_2_chips,
	&pmx_uart0,

	/* spear1310 specific devices */
	&pmx_can,
	&pmx_i2c1,
	&pmx_smii_0_1_2,
	&pmx_fsmc16bit_4_chips,
	&pmx_rs485_hdlc_1_2,
	&pmx_tdm_hdlc_1_2,
	&pmx_uart_1,
	&pmx_uart_2,
	&pmx_uart_3_4_5,
};

static struct amba_device *amba_devs[] __initdata = {
	/* spear13xx specific devices */
	&spear13xx_gpio_device[0],
	&spear13xx_gpio_device[1],
	&spear13xx_ssp_device,
	&spear13xx_uart_device,
};

static struct platform_device *plat_devs[] __initdata = {
	/* spear13xx specific devices */
	&spear13xx_ehci0_device,
	&spear13xx_ehci1_device,
	&spear13xx_i2c_device,
	&spear13xx_kbd_device,
	&spear13xx_nand_device,
	&spear13xx_ohci0_device,
	&spear13xx_ohci1_device,
	&spear13xx_rtc_device,
	&spear13xx_sdhci_device,

	/* spear1310 specific devices */
	&spear1310_can0_device,
	&spear1310_can1_device,
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

static void __init spi_init(void)
{
	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
}

#ifdef CONFIG_PCIEPORTBUS
/* this function is needed for PCIE host and device driver. Same
 * controller can not be programmed as host as well as device. So host
 * driver must call this function and if this function returns 1 then
 * only host should add that particular port as RC.
 * A port to be added as device, one must also add device's information
 * in plat_devs array defined in this file.
 * it is the responsibility of calling function to not send port number
 * greter than max no of controller(3)
 */
int spear1310_pcie_port_is_host(int port)
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

static void __init spear1310_evb_init(void)
{
	unsigned int i;

	/* padmux initialization, must be done before spear1300_init */
	pmx_driver.mode = NULL;
	pmx_driver.devs = pmx_devs;
	pmx_driver.devs_count = ARRAY_SIZE(pmx_devs);

	/* set keyboard plat data */
	kbd_set_plat_data(&spear13xx_kbd_device, &kbd_data);

	/* set nand device's plat data */
	fsmc_nand_set_plat_data(&spear13xx_nand_device, NULL, 0,
			NAND_SKIP_BBTSCAN, FSMC_NAND_BW8);
	nand_mach_init(FSMC_NAND_BW8);

	/* call spear1310 machine init function */
	spear1310_init();

	/* Register slave devices on the I2C buses */
	i2c_register_board_devices();

	/* initialize fsmc related data in fsmc plat data */
	fsmc_init_board_info(&spear13xx_fsmc_nor_device, partition_info,
			ARRAY_SIZE(partition_info), FSMC_FLASH_WIDTH8);
	fsmc_init_board_info(&spear1310_ras_fsmc_nor_device, NULL,
			0, FSMC_FLASH_WIDTH16);

	/* Initialize fsmc regiters */
	fsmc_nor_init(&spear13xx_fsmc_nor_device, SPEAR13XX_FSMC_BASE, 0,
			FSMC_FLASH_WIDTH8);

	/* initialize fsmc related data in fsmc plat data */
	fsmc_init_board_info(&spear13xx_fsmc_nor_device, partition_info,
			ARRAY_SIZE(partition_info), FSMC_FLASH_WIDTH8);

	/* Initialize fsmc regiters */
	fsmc_nor_init(&spear13xx_fsmc_nor_device, SPEAR13XX_FSMC_BASE, 0,
			FSMC_FLASH_WIDTH8);

#ifdef CONFIG_PCIEPORTBUS
	/* Enable PCIE0 clk */
	enable_pcie0_clk();
	pcie_init(&spear1310_pcie_port_is_host);
#endif

	/* Add Platform Devices */
	platform_add_devices(plat_devs, ARRAY_SIZE(plat_devs));

	/* Add Amba Devices */
	for (i = 0; i < ARRAY_SIZE(amba_devs); i++)
		amba_device_register(amba_devs[i], &iomem_resource);

	spi_init();
}

MACHINE_START(SPEAR1310, "ST-SPEAR1310-EVB")
	.boot_params	=	0x00000100,
	.map_io		=	spear13xx_map_io,
	.init_irq	=	spear13xx_init_irq,
	.timer		=	&spear13xx_timer,
	.init_machine	=	spear1310_evb_init,
MACHINE_END
