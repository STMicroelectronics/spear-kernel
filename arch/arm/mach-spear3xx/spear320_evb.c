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
#include <linux/io.h>
#include <linux/mtd/nand.h>
#include <linux/phy.h>
#include <linux/spi/flash.h>
#include <linux/mmc/sdhci-spear.h>
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

#define PARTITION(n, off, sz)	{.name = n, .offset = off, .size = sz}

static struct mtd_partition partition_info[] = {
	PARTITION("X-loader", 0, 1 * 0x20000),
	PARTITION("U-Boot", 0x20000, 3 * 0x20000),
	PARTITION("Kernel", 0x80000, 24 * 0x20000),
	PARTITION("Root File System", 0x380000, 84 * 0x20000),
};

/* ethernet phy device */
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

struct platform_device spear320_phy_device = {
	.name = "stmmacphy",
	.id = -1,
	.num_resources = 1,
	.resource = &phy_resources,
	.dev.platform_data = &phy_private_data,
};

/* Ethernet Private data */
static struct macb_base_data spear320_macb_data = {
	.phy_mask = 0,
	.gpio_num = PLGPIO_76,
	.phy_addr = 0x2,
	.mac_addr = {0xf2, 0xf2, 0xf2, 0x45, 0x67, 0x89},
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
	&spear3xx_ssp0_device,
	&spear3xx_uart_device,
	&spear3xx_wdt_device,

	/* spear320 specific devices */
	&spear320_clcd_device,
	&spear320_ssp_device[0],
	&spear320_ssp_device[1],
	&spear320_uart1_device,
	&spear320_uart2_device,
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

	/* spear320 specific devices */
	&spear320_can0_device,
	&spear320_can1_device,
	&spear320_eth_macb1_mii_device,
	&spear320_emi_nor_device,
	&spear320_i2c1_device,
	&spear320_nand_device,
	&spear320_phy_device,
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
#endif
};

static void __init spi_init(void)
{
	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
}

#define SPEAR320_CONFIG_REG	0x10
#define SPEAR_CFG_MII		0x20
#define ENABLE_MEM_CLK		0x1
static void macb_init_board_info(struct platform_device *pdev)
{
	void __iomem *io_base;
	u32 tmp;

	macb_set_plat_data(pdev, &spear320_macb_data);
	/*
	 * Select the MDIO Muxed configuration for the MII interface.
	 * The RAS control register should have the following cfg
	 * For SMII-0 interface
	 * Reset Bit-5 of RAS CONTROL REGISTER (0xB3000010)
	 *
	 * For SMII-1/MII interface
	 * Set Bit-5 of RAS CONTROL REGISTER (0xB3000010).
	 *
	 * This needs to be done at run time. At present the SMII
	 * interfaces are not functional, hence has been kept static for
	 * the MII interface only.
	 */
	io_base = ioremap(SPEAR320_SOC_CONFIG_BASE, 0x80);
	tmp = readl(io_base + SPEAR320_CONFIG_REG) | SPEAR_CFG_MII;
	writel(tmp, io_base + SPEAR320_CONFIG_REG);
	iounmap(io_base);

	/* Enable memory Port-1 clock */
	tmp = readl(AMEM_CLK_CFG) | ENABLE_MEM_CLK;
	writel(tmp, AMEM_CLK_CFG);
}

static void __init spear320_evb_init(void)
{
	/* set sdhci device platform data */
	sdhci_set_plat_data(&spear320_sdhci_device, &sdhci_plat_data);

	/* set adc platform data */
	set_adc_plat_data(&spear3xx_adc_device, &spear3xx_dmac_device.dev);

	/* set jpeg configurations for DMA xfers */
	set_jpeg_dma_configuration(&spear3xx_jpeg_device,
			&spear3xx_dmac_device.dev);

	/* set nand device's plat data */
	nand_set_plat_data(&spear320_nand_device, NULL, 0, NAND_SKIP_BBTSCAN,
			SPEAR_NAND_BW8);

	/* call spear320 machine init function */
	spear320_init(&spear320_auto_net_mii_mode, pmx_devs,
			ARRAY_SIZE(pmx_devs));

	/* initialize serial nor related data in smi plat data */
	smi_init_board_info(&spear3xx_smi_device);

	/* initialize macb related data in macb plat data */
	macb_init_board_info(&spear320_eth_macb1_mii_device);

	/* Register slave devices on the I2C buses */
	i2c_register_default_devices();

	/* initialize emi related data in emi plat data */
	emi_init_board_info(&spear320_emi_nor_device, partition_info,
			ARRAY_SIZE(partition_info), EMI_FLASH_WIDTH16);

	/* Add Platform Devices */
	platform_add_devices(plat_devs, ARRAY_SIZE(plat_devs));

	/* Add Amba Devices */
	spear_amba_device_register(amba_devs, ARRAY_SIZE(amba_devs));

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
