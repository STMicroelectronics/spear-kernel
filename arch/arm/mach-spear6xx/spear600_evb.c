/*
 * arch/arm/mach-spear6xx/spear600_evb.c
 *
 * SPEAr600 evaluation board source file
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
#include <linux/mtd/spear_smi.h>
#include <linux/phy.h>
#include <linux/spi/flash.h>
#include <linux/spi/spi.h>
#include <linux/stmmac.h>
#include <asm/hardware/vic.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <plat/spi.h>
#include <mach/generic.h>
#include <mach/hardware.h>

static struct amba_device *amba_devs[] __initdata = {
	&clcd_device,
	&dma_device,
	&gpio_device[0],
	&gpio_device[1],
	&gpio_device[2],
	&ssp_device[0],
	&ssp_device[1],
	&ssp_device[2],
	&uart_device[0],
	&uart_device[1],
	&wdt_device,
};

static struct platform_device *plat_devs[] __initdata = {
	&adc_device,
	&cpufreq_device,
	&ehci0_device,
	&ehci1_device,
	&eth_device,
	&i2c_device,
	&irda_device,
	&jpeg_device,
	&ohci0_device,
	&ohci1_device,
	&nand_device,
	&rtc_device,
	&smi_device,
	&touchscreen_device,
	&udc_device,
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
	.phy_addr = 1,
	.interface = PHY_INTERFACE_MODE_GMII,
	.has_gmac = 1,
	.enh_desc = 0,
	.tx_coe = 0,
	.dma_cfg = &dma0_private_data,
	.rx_coe_type = STMMAC_RX_COE_T1,
	.bugged_jumbo = 0,
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
/* Currently no gpios are free on eval board so it is kept commented */
#if 0
/* spi0 flash Chip Select Control function, controlled by gpio pin mentioned */
DECLARE_SPI_CS_GPIO_CONTROL(0, flash, /* mention gpio number here */);
/* spi0 flash Chip Info structure */
DECLARE_SPI_CHIP_INFO(0, flash, spi0_flash_cs_gpio_control);

/* spi0 spidev Chip Select Control function, controlled by gpio pin mentioned */
DECLARE_SPI_CS_GPIO_CONTROL(0, dev, /* mention gpio number here */);
/* spi0 spidev Chip Info structure */
DECLARE_SPI_CHIP_INFO(0, dev, spi0_dev_cs_gpio_control);
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
		.mode = SPI_MODE_1,
	}, {
		.modalias = "m25p80",
		.controller_data = &spi0_flash_chip_info,
		.max_speed_hz = 22000000, /* Actual 20.75 */
		.bus_num = 0,
		.chip_select = 1,
		.mode = SPI_MODE_3,
	}
#endif
};

static void __init spear600_evb_init(void)
{
	unsigned int i;

	/* set nand device's plat data */
	if (platform_device_add_data(&nand_device, &nand_plat_data,
				sizeof(nand_plat_data)))
		printk(KERN_WARNING "%s: couldn't add plat_data",
				nand_device.name);

	/* call spear600 machine init function */
	spear600_init();

	/* Set stmmac plat data */
	if (platform_device_add_data(&eth_device, &eth_data,
			sizeof(eth_data)))
		printk(KERN_WARNING "%s: couldn't add plat_data",
				eth_device.name);

	/* initialize serial nor related data in smi plat data */
	smi_init_board_info(&smi_device);

	/* Register slave devices on the I2C buses */
	i2c_register_default_devices();

	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));

	/* Add Platform Devices */
	platform_add_devices(plat_devs, ARRAY_SIZE(plat_devs));

	/* Add Amba Devices */
	for (i = 0; i < ARRAY_SIZE(amba_devs); i++)
		amba_device_register(amba_devs[i], &iomem_resource);
}

MACHINE_START(SPEAR600_EVB, "ST-SPEAR600-EVB")
	.atag_offset	=	0x100,
	.map_io		=	spear6xx_map_io,
	.init_irq	=	spear6xx_init_irq,
	.handle_irq	=	vic_handle_irq,
	.timer		=	&spear6xx_timer,
	.init_machine	=	spear600_evb_init,
	.restart	=	spear_restart,
MACHINE_END
