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
#include <linux/mtd/fsmc.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <linux/spi/flash.h>
#include <linux/mmc/sdhci-spear.h>
#include <linux/spi/spi.h>
#include <mach/emi.h>
#include <mach/generic.h>
#include <mach/gpio.h>
#include <mach/spear.h>
#include <plat/fsmc.h>
#include <plat/spi.h>

#define PARTITION(n, off, sz)	{.name = n, .offset = off, .size = sz}

static struct mtd_partition partition_info[] = {
	PARTITION("X-loader", 0, 1 * 0x20000),
	PARTITION("U-Boot", 0x20000, 3 * 0x20000),
	PARTITION("Kernel", 0x80000, 24 * 0x20000),
	PARTITION("Root File System", 0x380000, 84 * 0x20000),
};

/* emi nor flash resources registeration */
static struct resource emi_nor_resources[] = {
	{
		.start	= SPEAR310_EMI_MEM_0_BASE,
		.end	= SPEAR310_EMI_MEM_0_BASE + SPEAR310_EMI_MEM_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

/* padmux devices to enable */
static struct pmx_dev *pmx_devs[] = {
	/* spear3xx specific devices */
	&spear3xx_pmx_i2c,
	&spear3xx_pmx_ssp,
	&spear3xx_pmx_mii,
	&spear3xx_pmx_uart0,

	/* spear320 specific devices */
	&spear320_pmx_fsmc,
	&spear320_pmx_sdhci,
	&spear320_pmx_i2s,
	&spear320_pmx_uart1,
	&spear320_pmx_uart2,
	&spear320_pmx_can,
	&spear320_pmx_pwm0,
	&spear320_pmx_pwm1,
	&spear320_pmx_pwm2,
	&spear320_pmx_mii1,
};

static struct amba_device *amba_devs[] __initdata = {
	/* spear3xx specific devices */
	&spear3xx_gpio_device,
	&spear3xx_uart_device,
	&spear3xx_wdt_device,

	/* spear320 specific devices */
	&spear320_clcd_device,
};

static struct platform_device *plat_devs[] __initdata = {
	/* spear3xx specific devices */
	&spear3xx_ehci_device,
	&spear3xx_i2c_device,
	&spear3xx_ohci0_device,
	&spear3xx_ohci1_device,
	&spear3xx_rtc_device,

	/* spear320 specific devices */
	&spear320_can0_device,
	&spear320_can1_device,
	&spear320_i2c1_device,
	&spear320_nand_device,
	&spear320_plgpio_device,
	&spear320_pwm_device,
	&spear320_sdhci_device,
};

/* sdhci board specific information */
static struct sdhci_plat_data sdhci_plat_data = {
	.card_power_gpio = PLGPIO_61,
	.power_active_high = 0,
	.power_always_enb = 1,
	.card_int_gpio = -1,
};

static struct spi_board_info __initdata spi_board_info[] = {
};

static void __init spi_init(void)
{
	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
}

static void __init spear320_evb_init(void)
{
	unsigned int i;

	/* set sdhci device platform data */
	sdhci_set_plat_data(&spear320_sdhci_device, &sdhci_plat_data);

	/* set nand device's plat data */
	fsmc_nand_set_plat_data(&spear320_nand_device, NULL, 0,
			NAND_SKIP_BBTSCAN, FSMC_NAND_BW8);

	/* call spear320 machine init function */
	spear320_init(&spear320_auto_net_mii_mode, pmx_devs,
			ARRAY_SIZE(pmx_devs));

	/* Register slave devices on the I2C buses */
	i2c_register_board_devices();

	/* initialize emi related data in emi plat data */
	emi_init_board_info(&spear320_emi_nor_device, emi_nor_resources,
			ARRAY_SIZE(emi_nor_resources), partition_info,
			ARRAY_SIZE(partition_info), EMI_FLASH_WIDTH16);

	/* Add Platform Devices */
	platform_add_devices(plat_devs, ARRAY_SIZE(plat_devs));

	/* Add Amba Devices */
	for (i = 0; i < ARRAY_SIZE(amba_devs); i++)
		amba_device_register(amba_devs[i], &iomem_resource);

	/* Initialize emi regiters */
	emi_init(&spear320_emi_nor_device, SPEAR320_EMI_CTRL_BASE, 0,
			EMI_FLASH_WIDTH16);

	spi_init();
}

MACHINE_START(SPEAR320, "ST-SPEAR320-EVB")
	.boot_params	=	0x00000100,
	.map_io		=	spear3xx_map_io,
	.init_irq	=	spear3xx_init_irq,
	.timer		=	&spear3xx_timer,
	.init_machine	=	spear320_evb_init,
MACHINE_END
