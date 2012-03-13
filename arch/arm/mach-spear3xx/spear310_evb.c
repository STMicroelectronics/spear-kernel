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

#include <linux/gpio.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/fsmc.h>
#include <linux/mtd/physmap.h>
#include <linux/mtd/spear_smi.h>
#include <linux/phy.h>
#include <linux/spi/flash.h>
#include <linux/spi/spi.h>
#include <linux/stmmac.h>
#include <asm/hardware/vic.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <plat/hdlc.h>
#include <plat/spi.h>
#include <mach/emi.h>
#include <mach/generic.h>
#include <mach/hardware.h>
#include <mach/macb_eth.h>
#include <mach/misc_regs.h>

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

static struct physmap_flash_data emi_norflash_data = {
	.parts = partition_info,
	.nr_parts = ARRAY_SIZE(partition_info),
	.width = 4,
};
static struct platform_device spear310_emi_nor_device = {
	.name	= "physmap-flash",
	.id	= -1,
	.dev.platform_data = &emi_norflash_data,
	.resource = emi_nor_resources,
	.num_resources = ARRAY_SIZE(emi_nor_resources),
};

/* ethernet device */
static struct macb_base_data spear310_macb1_data = {
	.bus_id = 1,
	.phy_mask = 0,
	.gpio_num = -1,
	.phy_addr = 0x1,
	.mac_addr = {0xf2, 0xf2, 0xf2, 0x45, 0x67, 0x89},
	.plat_mdio_control = spear3xx_macb_plat_mdio_control,
};

static struct macb_base_data spear310_macb2_data = {
	.bus_id = 2,
	.phy_mask = 0,
	.gpio_num = -1,
	.phy_addr = 0x3,
	.mac_addr = {0xf2, 0xf2, 0xf2, 0x22, 0x22, 0x22},
	.plat_mdio_control = spear3xx_macb_plat_mdio_control,
};

static struct macb_base_data spear310_macb3_data = {
	.bus_id = 3,
	.phy_mask = 0,
	.gpio_num = -1,
	.phy_addr = 0x5,
	.mac_addr = {0xf2, 0xf2, 0xf2, 0x34, 0x56, 0x78},
	.plat_mdio_control = spear3xx_macb_plat_mdio_control,
};

static struct macb_base_data spear310_macb4_data = {
	.bus_id = 4,
	.phy_mask = 0,
	.gpio_num = -1,
	.phy_addr = 0x7,
	.mac_addr = {0xf2, 0xf2, 0xf2, 0x11, 0x11, 0x11},
	.plat_mdio_control = spear3xx_macb_plat_mdio_control,
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
	&spear3xx_pmx_mii,
	&spear3xx_pmx_uart0,

	/* spear310 specific devices */
	&spear310_pmx_emi_cs_0_1_4_5,
	&spear310_pmx_emi_cs_2_3,
	&spear310_pmx_uart1,
	&spear310_pmx_uart2,
	&spear310_pmx_uart3_4_5,
	&spear310_pmx_fsmc,
};

static struct amba_device *amba_devs[] __initdata = {
	/* spear3xx specific devices */
	&spear3xx_dma_device,
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
	&spear3xx_cpufreq_device,
	&spear3xx_ehci_device,
	&spear3xx_eth_device,
	&spear3xx_i2c_device,
	&spear3xx_irda_device,
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
	&spear310_plgpio_device,
};

/* Ethernet PLatform data */
/* MDIO Bus Data */
static struct stmmac_mdio_bus_data mdio0_private_data = {
	.bus_id = 0,
	.phy_mask = 0,
};

static struct stmmac_dma_cfg dma0_private_data = {
	.pbl = 8,
	.fixed_burst = 1,
	.burst_len_supported = DMA_AXI_BLEN_ALL,
};

static struct plat_stmmacenet_data eth_data = {
	.bus_id = 0,
	.phy_addr = -1,
	.interface = PHY_INTERFACE_MODE_MII,
	.has_gmac = 1,
	.enh_desc = 1,
	.tx_coe = 1,
	.dma_cfg = &dma0_private_data,
	.rx_coe_type = STMMAC_RX_COE_T2,
	.bugged_jumbo = 1,
	.pmt = 1,
	.mdio_bus_data = &mdio0_private_data,
	.clk_csr = STMMAC_CSR_150_250M,
};

/* fsmc platform data */
static const struct fsmc_nand_platform_data nand_plat_data __initconst = {
	.options = NAND_SKIP_BBTSCAN,
	.width = FSMC_NAND_BW8,
};

/* spi board information */
/* spi0 flash Chip Select Control function, controlled by gpio pin mentioned */
DECLARE_SPI_CS_GPIO_CONTROL(0, flash, BASIC_GPIO_3);
/* spi0 flash Chip Info structure */
DECLARE_SPI_CHIP_INFO(0, flash, spi0_flash_cs_gpio_control);

/* spi0 spidev Chip Select Control function, controlled by gpio pin mentioned */
DECLARE_SPI_CS_GPIO_CONTROL(0, dev, BASIC_GPIO_4);
/* spi0 spidev Chip Info structure */
DECLARE_SPI_CHIP_INFO(0, dev, spi0_dev_cs_gpio_control);

static struct spi_board_info __initdata spi_board_info[] = {
	/* spi0 board info */
	{
		.modalias = "spidev",
		.controller_data = &spi0_dev_chip_info,
		.max_speed_hz = 25000000,
		.bus_num = 0,
		.chip_select = 0,
		.mode = SPI_MODE_1,
	}, {
		.modalias = "m25p80",
		.controller_data = &spi0_flash_chip_info,
		.max_speed_hz = 22000000, /* Actual 20.75 */
		.bus_num = 0,
		.chip_select = 1,
		.mode = SPI_MODE_3,
	}
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

static void __init spear310_evb_init(void)
{
	unsigned int i;

	/* call spear310 machine init function */
	spear310_init(NULL, pmx_devs, ARRAY_SIZE(pmx_devs));

	/* Initialize emi regiters */
	emi_init(SPEAR310_EMI_REG_BASE, 0, EMI_FLASH_WIDTH32);

	/* set nand device's plat data */
	if (platform_device_add_data(&spear310_nand_device, &nand_plat_data,
				sizeof(nand_plat_data)))
		printk(KERN_WARNING "%s: couldn't add plat_data",
				spear310_nand_device.name);

	/* Set stmmac plat data */
	if (platform_device_add_data(&spear3xx_eth_device, &eth_data,
			sizeof(eth_data)))
		printk(KERN_WARNING "%s: couldn't add plat_data",
				spear3xx_eth_device.name);

	/* initialize macb related data in macb plat data */
	spear3xx_macb_setup();
	macb_set_plat_data(&spear310_eth_macb1_device, &spear310_macb1_data);
	macb_set_plat_data(&spear310_eth_macb2_device, &spear310_macb2_data);
	macb_set_plat_data(&spear310_eth_macb3_device, &spear310_macb3_data);
	macb_set_plat_data(&spear310_eth_macb4_device, &spear310_macb4_data);

	/* initialize serial nor related data in smi plat data */
	smi_init_board_info(&spear3xx_smi_device);

	/*
	 * Note: Remove the comment to enable E1 interface for one HDLC port
	 */
	/* select_e1_interface(&spear310_tdm_hdlc_device); */
	/* Register slave devices on the I2C buses */

	i2c_register_default_devices();

	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));

	/* Add Platform Devices */
	platform_add_devices(plat_devs, ARRAY_SIZE(plat_devs));

	/* Add Amba Devices */
	for (i = 0; i < ARRAY_SIZE(amba_devs); i++)
		amba_device_register(amba_devs[i], &iomem_resource);
}

MACHINE_START(SPEAR310_EVB, "ST-SPEAR310-EVB")
	.atag_offset	=	0x100,
	.map_io		=	spear310_map_io,
	.init_irq	=	spear3xx_init_irq,
	.handle_irq	=	vic_handle_irq,
	.timer		=	&spear3xx_timer,
	.init_machine	=	spear310_evb_init,
	.restart	=	spear_restart,
MACHINE_END
