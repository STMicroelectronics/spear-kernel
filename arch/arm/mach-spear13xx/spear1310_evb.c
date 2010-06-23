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
#include <linux/spi/flash.h>
#include <linux/spi/spi.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <mach/generic.h>
#include <mach/spear.h>
#include <mach/pcie.h>
#include <plat/adc.h>
#include <plat/fsmc.h>
#include <plat/jpeg.h>
#include <plat/keyboard.h>
#include <plat/nand.h>
#include <plat/smi.h>
#include <plat/spi.h>

#define PARTITION(n, off, sz)	{.name = n, .offset = off, .size = sz}

static struct mtd_partition partition_info[] = {
	PARTITION("X-loader", 0, 1 * 0x20000),
	PARTITION("U-Boot", 0x20000, 3 * 0x20000),
	PARTITION("Kernel", 0x80000, 24 * 0x20000),
	PARTITION("Root File System", 0x380000, 84 * 0x20000),
};

#if defined(CONFIG_FB_DB9000) || defined(CONFIG_FB_DB9000_MODULE)
extern void __init spear1300_evb_init_lcd(void);
#endif

static struct amba_device *amba_devs[] __initdata = {
	/* spear13xx specific devices */
	&gpio_device[0],
	&gpio_device[1],
	&ssp_device,
	&uart_device,
};

static struct platform_device *plat_devs[] __initdata = {
	/* spear13xx specific devices */
	&adc_device,
	&dmac_device[0],
	&dmac_device[1],
	&ehci0_device,
	&ehci1_device,
	&eth_device,
	&i2c_device,
	&jpeg_device,
	&kbd_device,
	&nand_device,
	&ohci0_device,
	&ohci1_device,
	&phy_device,
	&rtc_device,
	&sdhci_device,
	&smi_device,
	&wdt_device,
	&pcie_gadget0_device,

	/* spear1310 specific devices */
	&can0_device,
	&can1_device,
};

/* keyboard specific platform data */
static DECLARE_KEYMAP(spear_keymap);

static struct kbd_platform_data kbd_data = {
	.keymap = spear_keymap,
	.keymapsize = ARRAY_SIZE(spear_keymap),
	.rep = 1,
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
		.max_speed_hz = 1000000,
		.bus_num = 0,
		.chip_select = 1,
		.mode = 0,
	}
#endif
};

static void __init spi_init(void)
{
	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
}

static void __init spear1310_evb_init(void)
{
	/* set adc platform data */
	set_adc_plat_data(&adc_device, &dmac_device[0].dev);

	/* set keyboard plat data */
	kbd_set_plat_data(&kbd_device, &kbd_data);

	/* set jpeg configurations for DMA xfers */
	set_jpeg_dma_configuration(&jpeg_device, &dmac_device[0].dev);

	/* set nand device's plat data */
	nand_set_plat_data(&nand_device, NULL, 0, NAND_SKIP_BBTSCAN,
			SPEAR_NAND_BW8);
	nand_mach_init(SPEAR_NAND_BW8);

	/* call spear1310 machine init function */
	spear1310_init();

	/* Register slave devices on the I2C buses */
	i2c_register_board_devices();

	/* initialize serial nor related data in smi plat data */
	smi_init_board_info(&smi_device);
	/* initialize fsmc related data in fsmc plat data */
	fsmc_init_board_info(&fsmc_nor_device, partition_info,
			ARRAY_SIZE(partition_info), FSMC_FLASH_WIDTH8);

#ifdef CONFIG_PCIEPORTBUS
	/* Enable PCIE0 clk */
	enable_pcie0_clk();
#endif

	/* Add Platform Devices */
	platform_add_devices(plat_devs, ARRAY_SIZE(plat_devs));

	/* Add Amba Devices */
	spear_amba_device_register(amba_devs, ARRAY_SIZE(amba_devs));

	/* Initialize fsmc regiters */
	fsmc_nor_init(&fsmc_nor_device, SPEAR13XX_FSMC_BASE, 0,
			FSMC_FLASH_WIDTH8);

	spi_init();
#if defined(CONFIG_FB_DB9000) || defined(CONFIG_FB_DB9000_MODULE)
   spear1300_evb_init_lcd();
#endif

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
int spear13xx_pcie_port_is_host(int port)
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

MACHINE_START(SPEAR1310, "ST-SPEAR1310-EVB")
	.boot_params	=	0x00000100,
	.map_io		=	spear13xx_map_io,
	.init_irq	=	spear13xx_init_irq,
	.timer		=	&spear13xx_timer,
	.init_machine	=	spear1310_evb_init,
MACHINE_END
