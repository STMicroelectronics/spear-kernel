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

#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <linux/mtd/nand.h>
#include <linux/phy.h>
#include <linux/spi/flash.h>
#include <linux/spi/spi.h>
#include <linux/stmmac.h>
#include <mach/emi.h>
#include <mach/generic.h>
#include <mach/gpio.h>
#include <mach/macb_eth.h>
#include <mach/misc_regs.h>
#include <mach/spear.h>
#include <plat/adc.h>
#include <plat/jpeg.h>
#include <plat/nand.h>
#include <plat/smi.h>
#include <plat/spi.h>
#include <plat/hdlc.h>

#define PARTITION(n, off, sz)	{.name = n, .offset = off, .size = sz}

static struct mtd_partition partition_info[] = {
	PARTITION("X-loader", 0, 1 * 0x20000),
	PARTITION("U-Boot", 0x20000, 3 * 0x20000),
	PARTITION("Kernel", 0x80000, 24 * 0x20000),
	PARTITION("Root File System", 0x380000, 84 * 0x20000),
};

/* ethernet device */
static struct plat_stmmacphy_data phy_private_data = {
	.bus_id = 0,
	.phy_addr = -1,
	.phy_mask = 0,
	.interface = PHY_INTERFACE_MODE_MII,
};

static struct resource phy_resources = {
	.name = "phyirq",
	.start = -1,
	.end = -1,
	.flags = IORESOURCE_IRQ,
};

static struct platform_device spear310_phy0_device = {
	.name = "stmmacphy",
	.id = -1,
	.num_resources = 1,
	.resource = &phy_resources,
	.dev.platform_data = &phy_private_data,
};

static struct macb_base_data spear310_macb1_data = {
	.phy_mask = 0,
	.gpio_num = -1,
	.phy_addr = 0x1,
	.mac_addr = {0xf2, 0xf2, 0xf2, 0x45, 0x67, 0x89},
};

static struct macb_base_data spear310_macb2_data = {
	.phy_mask = 0,
	.gpio_num = -1,
	.phy_addr = 0x3,
	.mac_addr = {0xf2, 0xf2, 0xf2, 0x22, 0x22, 0x22},
};

static struct macb_base_data spear310_macb3_data = {
	.phy_mask = 0,
	.gpio_num = -1,
	.phy_addr = 0x5,
	.mac_addr = {0xf2, 0xf2, 0xf2, 0x34, 0x56, 0x78},
};

static struct macb_base_data spear310_macb4_data = {
	.phy_mask = 0,
	.gpio_num = -1,
	.phy_addr = 0x7,
	.mac_addr = {0xf2, 0xf2, 0xf2, 0x11, 0x11, 0x11},
};

/* padmux devices to enable */
static struct pmx_dev *pmx_devs[] = {
	/* spear3xx specific devices */
	&spear3xx_pmx_i2c,
	&spear3xx_pmx_ssp,
	&spear3xx_pmx_gpio_pin0,
	&spear3xx_pmx_gpio_pin1,
	&spear3xx_pmx_gpio_pin2,
	&spear3xx_pmx_gpio_pin3,
	&spear3xx_pmx_gpio_pin4,
	&spear3xx_pmx_gpio_pin5,
	&spear3xx_pmx_uart0,

	/* spear310 specific devices */
	&spear310_pmx_emi_cs_0_1_4_5,
	&spear310_pmx_emi_cs_2_3,
	&spear310_pmx_uart1,
	&spear310_pmx_uart2,
	&spear310_pmx_uart3_4_5,
	&spear310_pmx_fsmc,
	&spear310_pmx_rs485_0_1,
	&spear310_pmx_tdm0,
};

static struct amba_device *amba_devs[] __initdata = {
	/* spear3xx specific devices */
	&spear3xx_gpio_device,
	&spear3xx_ssp0_device,
	&spear3xx_uart_device,
	&spear3xx_wdt_device,

	/* spear310 specific devices */
	&spear310_uart1_device,
	&spear310_uart2_device,
	&spear310_uart3_device,
	&spear310_uart4_device,
	&spear310_uart5_device,
};

static struct platform_device *plat_devs[] __initdata = {
	/* spear3xx specific devices */
	&spear3xx_adc_device,
	&spear3xx_dmac_device,
	&spear3xx_ehci_device,
	&spear3xx_eth_device,
	&spear3xx_i2c_device,
	&spear3xx_jpeg_device,
	&spear3xx_ohci0_device,
	&spear3xx_ohci1_device,
	&spear3xx_rtc_device,
	&spear3xx_smi_device,
	&spear3xx_udc_device,

	/* spear310 specific devices */
	&spear310_emi_nor_device,
	&spear310_eth_macb1_device,
	&spear310_eth_macb2_device,
	&spear310_eth_macb3_device,
	&spear310_eth_macb4_device,
	&spear310_nand_device,
	&spear310_phy0_device,
	&spear310_plgpio_device,
	&spear310_tdm_hdlc_device,
	&spear310_rs485_0_device,
	&spear310_rs485_1_device,
};

/*
 * select_e1_interface: config CPLD to enable select E1 interface
 *
 * By default, TDM is selected. To switch the hardware connection, SW should
 * call this function in machine init routine to enable E1 interface
 */
#if 0
static void __init select_e1_interface(struct platform_device *pdev)
{
	/*
	 * selection is through CPLD which is connected on EMI bus
	 * before config, initialize EMI controller here
	 */
	emi_init(&spear310_tdm_hdlc_device, SPEAR310_EMI_REG_BASE, 2, EMI_FLASH_WIDTH8);

	e1phy_init(SPEAR310_EMI_MEM_2_BASE, 2);
	tdm_hdlc_set_plat_data(pdev, 32);
}
#endif

/* spi board information */
/* spi0 flash Chip Select Control function, controlled by gpio pin mentioned */
DECLARE_SPI_CS_CONTROL(0, flash, BASIC_GPIO_3);
/* spi0 flash Chip Info structure */
DECLARE_SPI_CHIP_INFO(0, flash, spi0_flash_cs_control);

/* spi0 spidev Chip Select Control function, controlled by gpio pin mentioned */
DECLARE_SPI_CS_CONTROL(0, dev, BASIC_GPIO_4);
/* spi0 spidev Chip Info structure */
DECLARE_SPI_CHIP_INFO(0, dev, spi0_dev_cs_control);

static struct spi_board_info __initdata spi_board_info[] = {
	/* spi0 board info */
	{
		.modalias = "spidev",
		.controller_data = &spi0_dev_chip_info,
		.max_speed_hz = 25000000,
		.bus_num = 0,
		.chip_select = 0,
		.mode = 0,
	}, {
		.modalias = "m25p80",
		.controller_data = &spi0_flash_chip_info,
		.max_speed_hz = 25000000,
		.bus_num = 0,
		.chip_select = 1,
		.mode = 0,
	}
};

static void __init spi_init(void)
{
	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
}

#define ENABLE_MEM_CLK	1
static void macb_enable_mem_clk(void)
{
	u32 tmp;

	/* Enable memory Port-1 clock */
	tmp = readl(AMEM_CLK_CFG) | ENABLE_MEM_CLK;
	writel(tmp, AMEM_CLK_CFG);

	/*
	 * Program the pad strengths of PLGPIO to drive the IO's
	 * The Magic number being used have direct correlations
	 * with the driving capabilities of the IO pads.
	 */
	writel(0x2f7bc210, PLGPIO3_PAD_PRG);
	writel(0x017bdef6, PLGPIO4_PAD_PRG);

}

static void __init spear310_evb_init(void)
{
	/* set adc platform data */
	set_adc_plat_data(&spear3xx_adc_device, &spear3xx_dmac_device.dev);

	/* set jpeg configurations for DMA xfers */
	set_jpeg_dma_configuration(&spear3xx_jpeg_device,
			&spear3xx_dmac_device.dev);

	/* set nand device's plat data */
	nand_set_plat_data(&spear310_nand_device, NULL, 0, NAND_SKIP_BBTSCAN,
			SPEAR_NAND_BW8);

	/* call spear310 machine init function */
	spear310_init(NULL, pmx_devs, ARRAY_SIZE(pmx_devs));

	/* Register slave devices on the I2C buses */
	i2c_register_default_devices();

	/* initialize serial nor related data in smi plat data */
	smi_init_board_info(&spear3xx_smi_device);

	/* initialize macb related data in macb plat data */
	macb_enable_mem_clk();
	macb_set_plat_data(&spear310_eth_macb1_device, &spear310_macb1_data);
	macb_set_plat_data(&spear310_eth_macb2_device, &spear310_macb2_data);
	macb_set_plat_data(&spear310_eth_macb3_device, &spear310_macb3_data);
	macb_set_plat_data(&spear310_eth_macb4_device, &spear310_macb4_data);

	/* initialize emi related data in emi plat data */
	emi_init_board_info(&spear310_emi_nor_device, partition_info,
			ARRAY_SIZE(partition_info), EMI_FLASH_WIDTH32);

	/* Add Platform Devices */
	platform_add_devices(plat_devs, ARRAY_SIZE(plat_devs));

	/* Add Amba Devices */
	spear_amba_device_register(amba_devs, ARRAY_SIZE(amba_devs));

	 /*
	  * Note: Remove the comment to enable E1 interface for one HDLC port
	  */
	/* select_e1_interface(&spear310_tdm_hdlc_device); */

	/* Initialize emi regiters */
	emi_init(&spear310_emi_nor_device, SPEAR310_EMI_REG_BASE, 0,
			EMI_FLASH_WIDTH32);

	spi_init();
}

MACHINE_START(SPEAR310, "ST-SPEAR310-EVB")
	.boot_params	=	0x00000100,
	.map_io		=	spear3xx_map_io,
	.init_irq	=	spear3xx_init_irq,
	.timer		=	&spear3xx_timer,
	.init_machine	=	spear310_evb_init,
MACHINE_END
