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

#include <linux/gpio.h>
#include <linux/mmc/sdhci-spear.h>
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
#include <plat/keyboard.h>
#include <plat/spi.h>
#include <mach/generic.h>
#include <mach/hardware.h>

/* padmux devices to enable */
static struct pmx_dev *pmx_devs[] = {
	/* spear3xx specific devices */
	&spear3xx_pmx_i2c,
	&spear3xx_pmx_ssp_cs,
	&spear3xx_pmx_ssp,
	&spear3xx_pmx_mii,
	&spear3xx_pmx_uart0,

	/* spear300 specific devices */
	&spear300_pmx_fsmc_2_chips,
	&spear300_pmx_clcd,
	&spear300_pmx_telecom_sdhci_4bit,
	&spear300_pmx_gpio1,
};

static struct amba_device *amba_devs[] __initdata = {
	/* spear3xx specific devices */
	&spear3xx_dma_device,
	&spear3xx_gpio_device,
	&spear3xx_ssp0_device,
	&spear3xx_uart_device,
	&spear3xx_wdt_device,

	/* spear300 specific devices */
	&spear300_clcd_device,
	&spear300_gpio1_device,
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

	/* spear300 specific devices */
	&spear300_kbd_device,
	&spear300_nand0_device,
	&spear300_sdhci_device,
	&spear300_touchscreen_device,
};

/* sdhci board specific information */
static struct sdhci_plat_data sdhci_plat_data = {
	.card_power_gpio = RAS_GPIO_2,
	.power_active_high = 0,
	.power_always_enb = 0,
	.card_int_gpio = RAS_GPIO_0,
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
static const struct fsmc_nand_platform_data nand0_plat_data __initconst = {
	.options = NAND_SKIP_BBTSCAN,
	.width = FSMC_NAND_BW8,
	.ale_off = SPEAR300_PLAT_NAND_ALE,
	.cle_off = SPEAR300_PLAT_NAND_CLE,
};

/* keyboard specific platform data */
static const __initconst DECLARE_9x9_KEYMAP(keymap);
static const struct matrix_keymap_data keymap_data __initconst = {
	.keymap = keymap,
	.keymap_size = ARRAY_SIZE(keymap),
};

static const struct kbd_platform_data kbd_data __initconst = {
	.keymap = &keymap_data,
	.rep = 1,
};

/* spi board information */
/* spi0 flash Chip Select Control function, controlled by gpio pin mentioned */
DECLARE_SPI_CS_GPIO_CONTROL(0, flash, RAS_GPIO_3);
/* spi0 flash Chip Info structure */
DECLARE_SPI_CHIP_INFO(0, flash, spi0_flash_cs_gpio_control);

/*
 * Chip select of spidev, currently no gpio is free on eval board so it is kept
 * commented
 */
#if 0
/* spi0 spidev Chip Select Control function, controlled by gpio pin mentioned */
DECLARE_SPI_CS_GPIO_CONTROL(0, dev, /* mention gpio number here */);
/* spi0 spidev Chip Info structure */
DECLARE_SPI_CHIP_INFO(0, dev, spi0_dev_cs_gpio_control);
#endif

static struct spi_board_info __initdata spi_board_info[] = {
	/* spi0 board info */
	{
#if 0
		.modalias = "spidev",
		.controller_data = &spi0_dev_chip_info,
		.max_speed_hz = 25000000,
		.bus_num = 0,
		.chip_select = 0,
		.mode = SPI_MODE_1,
	}, {
#endif
		.modalias = "m25p80",
		.controller_data = &spi0_flash_chip_info,
		.max_speed_hz = 22000000, /* Actual 20.75 */
		.bus_num = 0,
		.chip_select = 1,
		.mode = SPI_MODE_3,
	}
};

static void __init spear300_evb_init(void)
{
	unsigned int i;

	/* call spear300 machine init function */
	spear300_init(&spear300_photo_frame_mode, pmx_devs,
			ARRAY_SIZE(pmx_devs));

	/* Set stmmac plat data */
	if (platform_device_add_data(&spear3xx_eth_device, &eth_data,
			sizeof(eth_data)))
		printk(KERN_WARNING "%s: couldn't add plat_data",
				spear3xx_eth_device.name);

	/* set nand0 device's plat data */
	if (platform_device_add_data(&spear300_nand0_device, &nand0_plat_data,
				sizeof(nand0_plat_data)))
		printk(KERN_WARNING "%s: couldn't add plat_data",
				spear300_nand0_device.name);

	/* set keyboard plat data */
	if (platform_device_add_data(&spear300_kbd_device, &kbd_data,
				sizeof(kbd_data)))
		printk(KERN_WARNING "%s: couldn't add plat_data",
				spear300_kbd_device.name);

	/* set sdhci device platform data */
	sdhci_set_plat_data(&spear300_sdhci_device, &sdhci_plat_data);

	/* Enable sdhci memory */
	sdhci_i2s_mem_enable(SDHCI_MEM_ENB);

	/* initialize serial nor related data in smi plat data */
	smi_init_board_info(&spear3xx_smi_device);

	/* Register slave devices on the I2C buses */
	i2c_register_default_devices();

	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));

	/* Add Platform Devices */
	platform_add_devices(plat_devs, ARRAY_SIZE(plat_devs));

	/* Add Amba Devices */
	for (i = 0; i < ARRAY_SIZE(amba_devs); i++)
		amba_device_register(amba_devs[i], &iomem_resource);
}

MACHINE_START(SPEAR300_EVB, "ST-SPEAR300-EVB")
	.atag_offset	=	0x100,
	.map_io		=	spear3xx_map_io,
	.init_irq	=	spear3xx_init_irq,
	.handle_irq	=	vic_handle_irq,
	.timer		=	&spear3xx_timer,
	.init_machine	=	spear300_evb_init,
	.restart	=	spear_restart,
MACHINE_END
