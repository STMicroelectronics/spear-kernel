/*
 * arch/arm/mach-spear13xx/spear1310_reva_evb.c
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

#include <linux/clk.h>
#include <linux/types.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/mfd/stmpe.h>
#include <linux/mtd/fsmc.h>
#include <linux/mtd/nand.h>
#include <linux/pata_arasan_cf_data.h>
#include <linux/phy.h>
#include <linux/spi/flash.h>
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
#include <mach/gpio.h>
#include <mach/hardware.h>
#include <mach/spear_pcie.h>

/* fsmc nor partition info */
#if 0
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

static struct platform_device spear1310_reva_phy0_device = {
	.name		= "stmmacphy",
	.id		= 0,
	.num_resources	= 1,
	.resource	= &phy0_resources,
	.dev.platform_data = &phy0_private_data,
};

/* Ethernet phy-1 device registeration */
static struct plat_stmmacphy_data phy1_private_data = {
	.bus_id = 1,
	.phy_addr = 1,
	.phy_mask = 0,
	.interface = PHY_INTERFACE_MODE_MII,
	.phy_clk_cfg = spear1310_reva_eth_phy_clk_cfg,
};

static struct resource phy1_resources = {
	.name = "phyirq",
	.start = -1,
	.end = -1,
	.flags = IORESOURCE_IRQ,
};

static struct platform_device spear1310_reva_phy1_device = {
	.name = "stmmacphy",
	.id = 1,
	.num_resources = 1,
	.resource = &phy1_resources,
	.dev.platform_data = &phy1_private_data,
};

/* Ethernet phy-2 device registeration */
static struct plat_stmmacphy_data phy2_private_data = {
	.bus_id = 2,
	.phy_addr = 2,
	.phy_mask = 0,
	.interface = PHY_INTERFACE_MODE_MII,
	.phy_clk_cfg = spear1310_reva_eth_phy_clk_cfg,
};

static struct resource phy2_resources = {
	.name = "phyirq",
	.start = -1,
	.end = -1,
	.flags = IORESOURCE_IRQ,
};

static struct platform_device spear1310_reva_phy2_device = {
	.name = "stmmacphy",
	.id = 2,
	.num_resources = 1,
	.resource = &phy2_resources,
	.dev.platform_data = &phy2_private_data,
};

/* Ethernet phy-3 device registeration */
static struct plat_stmmacphy_data phy3_private_data = {
	.bus_id = 3,
	.phy_addr = 3,
	.phy_mask = 0,
	.interface = PHY_INTERFACE_MODE_RMII,
	.phy_clk_cfg = spear1310_reva_eth_phy_clk_cfg,
};

static struct resource phy3_resources = {
	.name = "phyirq",
	.start = -1,
	.end = -1,
	.flags = IORESOURCE_IRQ,
};

static struct platform_device spear1310_reva_phy3_device = {
	.name = "stmmacphy",
	.id = 3,
	.num_resources = 1,
	.resource = &phy3_resources,
	.dev.platform_data = &phy3_private_data,
};

/* Ethernet phy-4 device registeration */
static struct plat_stmmacphy_data phy4_private_data = {
	.bus_id = 4,
	.phy_addr = 4,
	.phy_mask = 0,
	.interface = PHY_INTERFACE_MODE_RGMII,
	.phy_clk_cfg = spear1310_reva_eth_phy_clk_cfg,
};

static struct resource phy4_resources = {
	.name = "phyirq",
	.start = -1,
	.end = -1,
	.flags = IORESOURCE_IRQ,
};

static struct platform_device spear1310_reva_phy4_device = {
	.name = "stmmacphy",
	.id = 4,
	.num_resources = 1,
	.resource = &phy4_resources,
	.dev.platform_data = &phy4_private_data,
};

/* padmux devices to enable */
static struct pmx_dev *pmx_devs[] = {
	/* spear13xx specific devices */
	&spear13xx_pmx_i2c,
	&spear13xx_pmx_i2s1,
	&spear13xx_pmx_egpio_grp,
	&spear13xx_pmx_gmii,
	&spear13xx_pmx_keyboard_6x6,
	&spear13xx_pmx_mcif,
	&spear13xx_pmx_smi_2_chips,
	&spear13xx_pmx_uart0,
	&spear13xx_pmx_sdhci,

	/* spear1310_reva specific devices */
	&spear1310_reva_pmx_can,
	&spear1310_reva_pmx_i2c1,
	&spear1310_reva_pmx_smii_0_1_2,
	&spear1310_reva_pmx_fsmc16bit_4_chips,
	&spear1310_reva_pmx_rs485_hdlc_1_2,
	&spear1310_reva_pmx_tdm_hdlc_1_2,
	&spear1310_reva_pmx_uart_1,
	&spear1310_reva_pmx_uart_2,
	&spear1310_reva_pmx_uart_3_4_5,
};

static struct amba_device *amba_devs[] __initdata = {
	/* spear13xx specific devices */
	&spear13xx_gpio_device[0],
	&spear13xx_gpio_device[1],
	&spear13xx_ssp_device,
	&spear13xx_uart_device,

	/* spear1310_reva specific devices */
	&spear1310_reva_uart1_device,
	&spear1310_reva_uart2_device,
	&spear1310_reva_uart3_device,
	&spear1310_reva_uart4_device,
	&spear1310_reva_uart5_device,
};

static struct platform_device *plat_devs[] __initdata = {
	/* spear13xx specific devices */
	&spear13xx_adc_device,
	&spear13xx_cpufreq_device,
	&spear13xx_db9000_clcd_device,
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
	&spear13xx_ohci0_device,
	&spear13xx_ohci1_device,
	&spear13xx_pcie_gadget0_device,
	&spear13xx_pcie_host1_device,
	&spear13xx_pcie_host2_device,
	&spear13xx_pcm_device,
	&spear13xx_rtc_device,
	&spear13xx_sdhci_device,
	&spear13xx_smi_device,
	&spear13xx_thermal_device,
	&spear13xx_udc_device,
	&spear13xx_wdt_device,

	/* spear1310_reva specific devices */
	&spear1310_reva_can0_device,
	&spear1310_reva_can1_device,
	&spear1310_reva_eth1_device,
	&spear1310_reva_eth2_device,
	&spear1310_reva_eth3_device,
	&spear1310_reva_eth4_device,
	&spear1310_reva_i2c1_device,
	&spear1310_reva_plgpio_device,
	&spear1310_reva_phy0_device,
	&spear1310_reva_phy1_device,
	&spear1310_reva_phy2_device,
	&spear1310_reva_phy3_device,
	&spear1310_reva_phy4_device,
	&spear1310_reva_ras_fsmc_nor_device,
	&spear1310_reva_rs485_0_device,
	&spear1310_reva_rs485_1_device,
	&spear1310_reva_tdm_hdlc_0_device,
	&spear1310_reva_tdm_hdlc_1_device,
};

static struct arasan_cf_pdata cf_pdata = {
	.cf_if_clk = CF_IF_CLK_166M,
	.quirk = CF_BROKEN_UDMA,
	.dma_priv = &cf_dma_priv,
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
	},
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
		.max_speed_hz = 12000000,
		.bus_num = 0,
		.chip_select = 1,
		.mode = SPI_MODE_3,
	}
#endif
};

#ifdef CONFIG_SPEAR_PCIE_REV341
 /* This function is needed for board specific PCIe initilization */
static void __init spear1310_reva_pcie_board_init(void)
{
	void *plat_data;

	plat_data = dev_get_platdata(&spear13xx_pcie_host1_device.dev);
	PCIE_PORT_INIT((struct pcie_port_info *)plat_data, SPEAR_PCIE_REV_3_41);
	((struct pcie_port_info *)plat_data)->clk_init =
		spear1310_reva_pcie_clk_init;
	((struct pcie_port_info *)plat_data)->clk_exit =
		spear1310_reva_pcie_clk_exit;

	plat_data = dev_get_platdata(&spear13xx_pcie_host2_device.dev);
	PCIE_PORT_INIT((struct pcie_port_info *)plat_data, SPEAR_PCIE_REV_3_41);
	((struct pcie_port_info *)plat_data)->clk_init =
		spear1310_reva_pcie_clk_init;
	((struct pcie_port_info *)plat_data)->clk_exit =
		spear1310_reva_pcie_clk_exit;
}
#endif

/* spear1310_reva ras misc configurations */
static void __init ras_fsmc_config(u32 mode, u32 width)
{
	u32 val, *address;

	address = ioremap(SPEAR1310_REVA_RAS_CTRL_REG0, SZ_16);

	val = readl(address);
	val &= ~(RAS_FSMC_MODE_MASK | RAS_FSMC_WIDTH_MASK);
	val |= mode;
	val |= width;
	val |= RAS_FSMC_CS_SPLIT;

	writel(val, address);

	iounmap(address);
}

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
	 * selection is through CPLD which is connected on FSMC bus
	 * before config, initialize FSMC controller here
	 */
	ras_fsmc_config(RAS_FSMC_MODE_NOR, RAS_FSMC_WIDTH_8);
	fsmc_nor_init(NULL, SPEAR1310_REVA_FSMC1_BASE, 2, FSMC_FLASH_WIDTH8);

	e1phy_init(SPEAR1310_REVA_FSMC1_CS2_BASE + (pdev->id * 0x100), 0);
	tdm_hdlc_set_plat_data(pdev, 32);
}
#endif

static void spear1310_reva_evb_fixup(struct machine_desc *desc,
		struct tag *tags, char **cmdline, struct meminfo *mi)
{
#if defined(CONFIG_FB_DB9000) || defined(CONFIG_FB_DB9000_MODULE)
	spear13xx_panel_fixup(mi);
#endif
}

static void __init spear1310_reva_evb_init(void)
{
	unsigned int i;

	/* set compact flash plat data */
	set_arasan_cf_pdata(&spear13xx_cf_device, &cf_pdata);

#if (defined(CONFIG_FB_DB9000) || defined(CONFIG_FB_DB9000_MODULE))
	/* db9000_clcd plat data */
	spear13xx_panel_init(&spear13xx_db9000_clcd_device);
#endif

	/* set keyboard plat data */
	kbd_set_plat_data(&spear13xx_kbd_device, &kbd_data);

	/* initialize serial nor related data in smi plat data */
	smi_init_board_info(&spear13xx_smi_device);

	/*
	 * SPEAr1310 FSMC cannot used as NOR and NAND at the same time
	 * For the moment, disable NAND and use NOR only
	 * If NAND is needed, enable the following code and disable all code for
	 * NOR. Also enable nand in padmux configuration to use it.
	 */
	/* set nand device's plat data */
#if 0
	/* set nand device's plat data */
	fsmc_nand_set_plat_data(&spear13xx_nand_device, NULL, 0,
			NAND_SKIP_BBTSCAN, FSMC_NAND_BW8, NULL, 1);
	nand_mach_init(FSMC_NAND_BW8);
#endif

	/* call spear1310_reva machine init function */
	spear1310_reva_init(NULL, pmx_devs, ARRAY_SIZE(pmx_devs));

	/* Register slave devices on the I2C buses */
	i2c_register_default_devices();

	/*
	 * Only one of Fixed or RAS part FSMC can be used at one time.
	 * Default selection is RAS part FSMC for NOR.
	 */
#if 0
	/* fixed part fsmc nor device */
	/* initialize fsmc related data in fsmc plat data */
	fsmc_init_board_info(&spear13xx_fsmc_nor_device, partition_info,
			ARRAY_SIZE(partition_info), FSMC_FLASH_WIDTH8);
	/* Initialize fsmc regiters */
	fsmc_nor_init(&spear13xx_fsmc_nor_device, SPEAR13XX_FSMC_BASE, 0,
			FSMC_FLASH_WIDTH8);
#endif

	/* ras part fsmc nor device */
	/* initialize fsmc related data in fsmc plat data */
	ras_fsmc_config(RAS_FSMC_MODE_NOR, RAS_FSMC_WIDTH_16);
	fsmc_init_board_info(&spear1310_reva_ras_fsmc_nor_device, NULL,
			0, FSMC_FLASH_WIDTH16);
	/* Initialize fsmc regiters */
	fsmc_nor_init(&spear1310_reva_ras_fsmc_nor_device,
			SPEAR1310_REVA_FSMC1_BASE, 3, FSMC_FLASH_WIDTH16);

#ifdef CONFIG_SPEAR_PCIE_REV341
	spear1310_reva_pcie_board_init();
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
	/* select_e1_interface(&spear1310_reva_tdm_hdlc_0_device); */
	/* select_e1_interface(&spear1310_reva_tdm_hdlc_1_device); */
}

MACHINE_START(SPEAR1310_REVA_EVB, "ST-SPEAR1310-REVA-EVB")
	.boot_params	=	0x00000100,
	.fixup		=	spear1310_reva_evb_fixup,
	.map_io		=	spear1310_reva_map_io,
	.init_irq	=	spear13xx_init_irq,
	.timer		=	&spear13xx_timer,
	.init_machine	=	spear1310_reva_evb_init,
MACHINE_END
