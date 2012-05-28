/*
 * arch/arm/mach-spear13xx/spear1310_evb.c
 *
 * SPEAr1310 evaluation board source file
 *
 * Copyright (C) 2011 ST Microelectronics
 * Viresh Kumar <viresh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/clk.h>
#include <linux/types.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/phy.h>
#include <linux/mfd/stmpe.h>
#include <linux/mtd/fsmc.h>
#include <linux/mtd/nand.h>
#include <linux/pata_arasan_cf_data.h>
#include <linux/spi/spi.h>
#include <linux/stmmac.h>
#include <video/db9000fb.h>
#include <asm/mach-types.h>
#include <plat/fsmc.h>
#include <plat/hdlc.h>
#include <plat/keyboard.h>
#include <plat/smi.h>
#include <plat/spi.h>
#include <mach/generic.h>
#include <mach/hardware.h>
#include <mach/spear1310_misc_regs.h>
#include <mach/spear_pcie.h>

#if 0
/* fsmc nor partition info */
#define PARTITION(n, off, sz)	{.name = n, .offset = off, .size = sz}
static struct mtd_partition partition_info[] = {
	PARTITION("X-loader", 0, 1 * 0x20000),
	PARTITION("U-Boot", 0x20000, 3 * 0x20000),
	PARTITION("Kernel", 0x80000, 24 * 0x20000),
	PARTITION("Root File System", 0x380000, 84 * 0x20000),
};
#endif

/* Ethernet phy-0 device registeration */
static struct plat_stmmacphy_data phy0_private_data = {
	.bus_id = 0,
	.phy_addr = 5,
	.phy_mask = 0,
	.interface = PHY_INTERFACE_MODE_GMII,
	.phy_clk_cfg = spear13xx_eth_phy_clk_cfg,
};

static struct resource phy0_resources = {
	.name = "phyirq",
	.start = -1,
	.end = -1,
	.flags = IORESOURCE_IRQ,
};

static struct platform_device spear1310_phy0_device = {
	.name		= "stmmacphy",
	.id		= 0,
	.num_resources	= 1,
	.resource	= &phy0_resources,
	.dev.platform_data = &phy0_private_data,
};

/* padmux devices to enable */
static struct pmx_dev *pmx_devs[] = {
	/* spear13xx specific devices */
	&spear13xx_pmx_clcd,
	&spear13xx_pmx_i2c,
	&spear13xx_pmx_i2s1,
	&spear13xx_pmx_egpio_grp,
	&spear13xx_pmx_gmii,
	&spear13xx_pmx_keyboard_6x6,
	&spear13xx_pmx_mcif,
	&spear13xx_pmx_smi_2_chips,
	&spear13xx_pmx_ssp,
	&spear13xx_pmx_uart0,

	/* spear1310 specific devices */
	&spear1310_pmx_sdhci,
	&spear13xx_pmx_nand_8bit,
	&spear1310_pmx_sata0,
	&spear1310_pmx_pcie1,
	&spear1310_pmx_pcie2,

};

static struct amba_device *amba_devs[] __initdata = {
	/* spear13xx specific devices */
	&spear13xx_gpio_device[0],
	&spear13xx_gpio_device[1],
	&spear13xx_ssp_device,
	&spear13xx_uart_device,

	/* spear1310 specific devices */
};

static struct platform_device *plat_devs[] __initdata = {
	/* spear13xx specific devices */
	&spear13xx_adc_device,
	&spear13xx_db9000_clcd_device,
	&spear13xx_cpufreq_device,
#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
	&spear13xx_device_gpiokeys,
#endif
	&spear13xx_dmac_device[0],
	&spear13xx_dmac_device[1],
	&spear13xx_ehci0_device,
	&spear13xx_ehci1_device,
	&spear13xx_eth_device,
	&spear13xx_i2c_device,
	&spear13xx_i2s0_device,
	&spear13xx_jpeg_device,
	&spear13xx_kbd_device,
	&spear1310_sata0_device,
	&spear13xx_pcie_gadget1_device,
	&spear13xx_pcie_host2_device,
	&spear13xx_pcm_device,
	&spear13xx_rtc_device,
	&spear13xx_sdhci_device,
	&spear13xx_smi_device,
	&spear13xx_thermal_device,
	&spear13xx_wdt_device,

	/* spear1310 specific devices */
	&spear1310_nand_device,
	&spear1310_otg_device,
	&spear1310_phy0_device,
	&spear1310_plgpio_device,
};

/* keyboard specific platform data */
static DECLARE_9x9_KEYMAP(keymap);
static struct matrix_keymap_data keymap_data = {
	.keymap = keymap,
	.keymap_size = ARRAY_SIZE(keymap),
};

static struct kbd_platform_data kbd_data = {
	.keymap = &keymap_data,
	.rep = 1,
	.mode = KEYPAD_9x9,
	.suspended_rate = 2000000,
};

/* spi master's configuration routine */
DECLARE_SPI_CS_CFG(0, VA_SPEAR1310_PERIP_CFG, SPEAR1310_SSP0_CS_SEL_MASK,
		SPEAR1310_SSP0_CS_SEL_SHIFT, SPEAR1310_SSP0_CS_CTL_MASK,
		SPEAR1310_SSP0_CS_CTL_SHIFT, SPEAR1310_SSP0_CS_CTL_SW,
		SPEAR1310_SSP0_CS_VAL_MASK, SPEAR1310_SSP0_CS_VAL_SHIFT);

/* spi0 flash Chip Select Control function */
DECLARE_SPI_CS_CONTROL(0, flash, SPEAR1310_SSP0_CS_SEL_CS1);
/* spi0 flash Chip Info structure */
DECLARE_SPI_CHIP_INFO(0, flash, spi0_flash_cs_control);

/* spi0 spidev Chip Select Control function */
DECLARE_SPI_CS_CONTROL(0, dev, SPEAR1310_SSP0_CS_SEL_CS2);
/* spi0 spidev Chip Info structure */
DECLARE_SPI_CHIP_INFO(0, dev, spi0_dev_cs_control);

/* spi0 touch screen Chip Select Control function, controlled by gpio pin */
DECLARE_SPI_CS_GPIO_CONTROL(0, ts, GPIO1_7);
/* spi0 touch screen Info structure */
static struct pl022_config_chip spi0_ts_chip_info = {
	.iface = SSP_INTERFACE_MOTOROLA_SPI,
	.hierarchy = SSP_MASTER,
	.slave_tx_disable = 0,
	.com_mode = INTERRUPT_TRANSFER,
	.rx_lev_trig = SSP_RX_1_OR_MORE_ELEM,
	.tx_lev_trig = SSP_TX_1_OR_MORE_EMPTY_LOC,
	.ctrl_len = SSP_BITS_8,
	.wait_state = SSP_MWIRE_WAIT_ZERO,
	.duplex = SSP_MICROWIRE_CHANNEL_FULL_DUPLEX,
	.cs_control = spi0_ts_cs_gpio_control,
};

static struct stmpe_ts_platform_data stmpe610_ts_pdata = {
	.sample_time = 4, /* 80 clocks */
	.mod_12b = 1, /* 12 bit */
	.ref_sel = 0, /* Internal */
	.adc_freq = 1, /* 3.25 MHz */
	.ave_ctrl = 1, /* 2 samples */
	.touch_det_delay = 2, /* 100 us */
	.settling = 2, /* 500 us */
	.fraction_z = 7,
	.i_drive = 1, /* 50 to 80 mA */
};

static struct stmpe_platform_data stmpe610_pdata = {
	.id = 0,
	.blocks = STMPE_BLOCK_TOUCHSCREEN,
	.irq_base = SPEAR_STMPE610_INT_BASE,
	.irq_trigger = IRQ_TYPE_EDGE_FALLING,
	.irq_invert_polarity = false,
	.autosleep = false,
	.irq_over_gpio = true,
	.irq_gpio = GPIO1_6,
	.ts = &stmpe610_ts_pdata,
};

static struct spi_board_info __initdata spi_board_info[] = {
	/* spi0 board info */
	{
		.modalias = "stmpe610",
		.platform_data = &stmpe610_pdata,
		.controller_data = &spi0_ts_chip_info,
		.max_speed_hz = 1000000,
		.bus_num = 0,
		.chip_select = 0,
		.mode = SPI_MODE_1,
	}, {
		.modalias = "m25p80",
		.controller_data = &spi0_flash_chip_info,
		.max_speed_hz = 12000000,
		.bus_num = 0,
		.chip_select = SPEAR1310_SSP0_CS_SEL_CS1,
		.mode = SPI_MODE_3,
	}, {
		.modalias = "spidev",
		.controller_data = &spi0_dev_chip_info,
		.max_speed_hz = 25000000,
		.bus_num = 0,
		.chip_select = SPEAR1310_SSP0_CS_SEL_CS2,
		.mode = SPI_MODE_1,
	}
};

#ifdef CONFIG_SPEAR_PCIE_REV370
/* This function is needed for board specific PCIe initilization */
static void __init spear1310_pcie_board_init(void)
{
	void *plat_data;

	plat_data = dev_get_platdata(&spear13xx_pcie_host2_device.dev);
	PCIE_PORT_INIT((struct pcie_port_info *)plat_data, SPEAR_PCIE_REV_3_70);
	((struct pcie_port_info *)plat_data)->clk_init =
		spear1310_pcie_clk_init;
	((struct pcie_port_info *)plat_data)->clk_exit =
		spear1310_pcie_clk_exit;
}
#endif

static void spear1310_evb_fixup(struct machine_desc *desc,
		struct tag *tags, char **cmdline, struct meminfo *mi)
{
#if defined(CONFIG_FB_DB9000) || defined(CONFIG_FB_DB9000_MODULE)
	spear13xx_panel_fixup(mi);
#endif
}

static void __init spear1310_evb_init(void)
{
	unsigned int i;

#if (defined(CONFIG_FB_DB9000) || defined(CONFIG_FB_DB9000_MODULE))
	/* db9000_clcd plat data */
	spear13xx_panel_init(&spear13xx_db9000_clcd_device);
#endif

	/* set keyboard plat data */
	kbd_set_plat_data(&spear13xx_kbd_device, &kbd_data);

	/* initialize serial nor related data in smi plat data */
	smi_init_board_info(&spear13xx_smi_device);

	/* set nand device's plat data */
	fsmc_nand_set_plat_data(&spear1310_nand_device, NULL, 0,
			NAND_SKIP_BBTSCAN, FSMC_NAND_BW8, NULL, 1);

	/* call spear1310 machine init function */
	spear1310_init(NULL, pmx_devs, ARRAY_SIZE(pmx_devs));

	/* Register slave devices on the I2C buses */
	i2c_register_default_devices();

#ifdef CONFIG_SPEAR_PCIE_REV370
	spear1310_pcie_board_init();
#endif

	/* Add Platform Devices */
	platform_add_devices(plat_devs, ARRAY_SIZE(plat_devs));

	/* Add Amba Devices */
	for (i = 0; i < ARRAY_SIZE(amba_devs); i++)
		amba_device_register(amba_devs[i], &iomem_resource);

	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));

	/*
	 * Note: Remove the comment to enable E1 interface for one HDLC port
	 */
	/* select_e1_interface(&spear1310_tdm_hdlc_0_device); */
	/* select_e1_interface(&spear1310_tdm_hdlc_1_device); */
}

MACHINE_START(SPEAR1310_EVB, "ST-SPEAR1310-EVB")
	.boot_params	=	0x00000100,
	.fixup		=	spear1310_evb_fixup,
	.map_io		=	spear1310_map_io,
	.init_irq	=	spear13xx_init_irq,
	.timer		=	&spear13xx_timer,
	.init_machine	=	spear1310_evb_init,
MACHINE_END
