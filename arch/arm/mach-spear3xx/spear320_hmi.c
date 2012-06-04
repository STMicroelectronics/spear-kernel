/*
 * arch/arm/mach-spear3xx/spear320_hmi.c
 *
 * SPEAr320 Human Machine Interface board source file
 *
 * Copyright (C) 2011 ST Microelectronics
 * Viresh Kumar <viresh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/mfd/stmpe.h>
#include <linux/mmc/sdhci-spear.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/fsmc.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <plat/fsmc.h>
#include <plat/smi.h>
#include <plat/spi.h>
#include <mach/generic.h>
#include <mach/hardware.h>
#include <mach/macb_eth.h>
#include <mach/misc_regs.h>

/* Ethernet Private data */
static struct macb_base_data hmi_macb_data[] = {
	{
		.bus_id = 1,
		.is_rmii = 1,
		.phy_mask = 0,
		.gpio_num = -1,
		.phy_addr = 0x1,
		.mac_addr = {0xf2, 0xf2, 0xf2, 0x45, 0x67, 0x89},
		.plat_mdio_control = spear3xx_macb_plat_mdio_control,
	}, {
		.bus_id = 2,
		.is_rmii = 1,
		.phy_mask = 0,
		.gpio_num = -1,
		.phy_addr = 0x0,
		.mac_addr = {0xf2, 0xf2, 0xf2, 0x45, 0x67, 0x8a},
		.plat_mdio_control = spear3xx_macb_plat_mdio_control,
	},
};

/* STMPE811 platform data */

static struct stmpe_gpio_platform_data stmpe811_gpio = {
	.gpio_base = SPEAR_STMPE811_GPIO_BASE,
	.norequest_mask = 0xf3,
};

static struct stmpe_ts_platform_data stmpe811_ts_pdata = {
	.sample_time = 4, /* 80 clocks */
	.mod_12b = 1, /* 12 bit */
	.ref_sel = 0, /* Internal */
	.adc_freq = 1, /* 3.25 MHz */
	.ave_ctrl = 2, /* 2 samples */
	.touch_det_delay = 3, /* 500 us */
	.settling = 4, /* 500 us */
	.fraction_z = 7,
	.i_drive = 1, /* 50 to 80 mA */
};

static struct stmpe_platform_data stmpe811_pdata = {
	.id = 0,
	.blocks = STMPE_BLOCK_TOUCHSCREEN | STMPE_BLOCK_GPIO,
	.irq_base = SPEAR320_STMPE_INT_BASE,
	.irq_trigger = IRQ_TYPE_EDGE_RISING,
	.irq_invert_polarity = false,
	.autosleep = false,
	.irq_over_gpio = true,
	.irq_gpio = BASIC_GPIO_1,
	.ts = &stmpe811_ts_pdata,
	.gpio = &stmpe811_gpio,
};

static struct i2c_board_info __initdata i2c_board_info[] = {
	{
		I2C_BOARD_INFO("stmpe811", 0x41),
		.platform_data = &stmpe811_pdata,
	},
};

/* padmux devices to enable */
static struct pmx_dev *pmx_devs[] = {
	/* spear3xx specific devices */
	&spear3xx_pmx_gpio_pin0,
	&spear3xx_pmx_gpio_pin1,
	&spear3xx_pmx_i2c,
	&spear3xx_pmx_ssp,
	&spear3xx_pmx_uart0,

	/* hmi specific devices */
	&spear320_pmx_can0,
	&spear320_pmx_can1,
	&spear320s_pmx_clcd,
	&spear320s_pmx_fsmc[0],
	&spear320_pmx_i2s,
	&spear320_pmx_pwm0_1[3],
	&spear320_pmx_pwm2[3],
	&spear320_pmx_sdhci[0],
	&spear320_pmx_uart2,
	&spear320_pmx_mii1_2[1],
};

static struct amba_device *amba_devs[] __initdata = {
	/* spear3xx specific devices */
	&spear3xx_dma_device,
	&spear3xx_gpio_device,
	&spear3xx_ssp0_device,
	&spear3xx_uart_device,
	&spear3xx_wdt_device,

	/* hmi specific devices */
	&spear320_clcd_device,
	&spear320_uart1_device,
	&spear320_uart2_device,
};

static struct platform_device *plat_devs[] __initdata = {
	/* spear3xx specific devices */
	&spear3xx_adc_device,
	&spear3xx_cpufreq_device,
	&spear3xx_ehci_device,
	&spear3xx_i2c_device,
	&spear3xx_jpeg_device,
	&spear3xx_ohci0_device,
	&spear3xx_ohci1_device,
	&spear3xx_rtc_device,
	&spear3xx_smi_device,
	&spear3xx_udc_device,
	/* hmi specific devices */
	&spear320_can0_device,
	&spear320_can1_device,
	&spear320_eth0_device,
	&spear320_eth1_device,
	&spear320_i2s_device,
	&spear320_nand_device,
	&spear320_pcm_device,
	&spear320_plgpio_device,
	&spear320_pwm_device,
	&spear320_sdhci_device,
};

/* sdhci board specific information */
static struct sdhci_plat_data sdhci_plat_data = {
	.card_power_gpio = PLGPIO_50,
	.power_active_high = 0,
	.power_always_enb = 1,
	.card_int_gpio = -1,
};

static void __init spear320_hmi_init(void)
{
	unsigned int i;

	/* set sdhci device platform data */
	sdhci_set_plat_data(&spear320_sdhci_device, &sdhci_plat_data);

	/* set nand device's plat data */
	fsmc_nand_set_plat_data(&spear320_nand_device, NULL, 0,
			NAND_SKIP_BBTSCAN, FSMC_NAND_BW8, NULL, 1);

	/* initialize macb related data in macb plat data */
	spear3xx_macb_setup();
	macb_set_plat_data(&spear320_eth0_device, &hmi_macb_data[0]);
	macb_set_plat_data(&spear320_eth1_device, &hmi_macb_data[1]);

	/* call spear320 machine init function */
	spear320_common_init(&spear320s_extended_mode, pmx_devs,
			ARRAY_SIZE(pmx_devs));

	/* Register slave devices on the I2C buses */
	i2c_register_default_devices();

	/* initialize serial nor related data in smi plat data */
	smi_init_board_info(&spear3xx_smi_device);

	/* Add Platform Devices */
	platform_add_devices(plat_devs, ARRAY_SIZE(plat_devs));

	/* Add Amba Devices */
	for (i = 0; i < ARRAY_SIZE(amba_devs); i++)
		amba_device_register(amba_devs[i], &iomem_resource);

	i2c_register_board_info(0, i2c_board_info, ARRAY_SIZE(i2c_board_info));
}

MACHINE_START(SPEAR320_HMI, "ST-SPEAR320-HMI")
	.boot_params	=	0x00000100,
	.map_io		=	spear320_map_io,
	.init_irq	=	spear3xx_init_irq,
	.timer		=	&spear3xx_timer,
	.init_machine	=	spear320_hmi_init,
MACHINE_END
