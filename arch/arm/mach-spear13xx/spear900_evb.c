/*
 * arch/arm/mach-spear13xx/spear900_evb.c
 *
 * SPEAr900 evaluation board source file
 *
 * Copyright (C) 2010 ST Microelectronics
 * Viresh Kumar <viresh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/irq.h>
#include <linux/types.h>
#include <linux/gpio.h>
#include <linux/mtd/nand.h>
#include <linux/phy.h>
#include <linux/spi/flash.h>
#include <linux/spi/spi.h>
#include <linux/stmmac.h>
#include <linux/stmpe610.h>
#include <asm/mach-types.h>
#include <asm/setup.h>
#include <plat/adc.h>
#include <plat/fsmc.h>
#include <plat/jpeg.h>
#include <plat/keyboard.h>
#include <plat/nand.h>
#include <plat/smi.h>
#include <plat/spi.h>
#include <mach/generic.h>
#include <mach/gpio.h>
#include <mach/spear.h>
#include <mach/pcie.h>
#include <mach/hardware.h>
#include <mach/db9000fb_info.h>

static unsigned long db900fb_buffer_phys;

#define PARTITION(n, off, sz)	{.name = n, .offset = off, .size = sz}
static struct mtd_partition partition_info[] = {
	PARTITION("X-loader", 0, 1 * 0x20000),
	PARTITION("U-Boot", 0x20000, 3 * 0x20000),
	PARTITION("Kernel", 0x80000, 24 * 0x20000),
	PARTITION("Root File System", 0x380000, 84 * 0x20000),
};

/* Ethernet phy device registeration */
static struct plat_stmmacphy_data phy0_private_data = {
	.bus_id = 0,
	.phy_addr = 5,
	.phy_mask = 0,
	.interface = PHY_INTERFACE_MODE_GMII,
};

static struct resource phy0_resources = {
	.name = "phyirq",
	.start = -1,
	.end = -1,
	.flags = IORESOURCE_IRQ,
};

struct platform_device spear900_phy0_device = {
	.name		= "stmmacphy",
	.id		= 0,
	.num_resources	= 1,
	.resource	= &phy0_resources,
	.dev.platform_data = &phy0_private_data,
};

/* padmux devices to enable */
static struct pmx_dev *pmx_devs[] = {
	/* spear13xx specific devices */
	&pmx_i2c,
	&pmx_i2s1,
	&pmx_i2s2,
	&pmx_clcd,
	&pmx_egpio_grp,
	&pmx_gmii,
	&pmx_keyboard_6x6,
	&pmx_mcif,
	&pmx_nand_8bit,
	&pmx_smi_4_chips,
	&pmx_ssp,
	&pmx_uart0,
	&pmx_sdhci,

	/* spear900 specific devices */
};

static struct amba_device *amba_devs[] __initdata = {
	&spear13xx_gpio_device[0],
	&spear13xx_gpio_device[1],
	&spear13xx_ssp_device,
	&spear13xx_uart_device,
};

static struct platform_device *plat_devs[] __initdata = {
	&spear13xx_adc_device,
	&spear13xx_db9000_clcd_device,
	&spear13xx_dmac_device[0],
	&spear13xx_dmac_device[1],
	&spear13xx_ehci0_device,
	&spear13xx_ehci1_device,
	&spear13xx_eth0_device,
	&spear13xx_i2c_device,
	&spear13xx_jpeg_device,
	&spear13xx_kbd_device,
	&spear13xx_nand_device,
	&spear13xx_i2s0_device,
	&spear13xx_ohci0_device,
	&spear13xx_ohci1_device,
	&spear13xx_rtc_device,
	&spear13xx_sdhci_device,
	&spear13xx_smi_device,
	&spear13xx_udc_device,
	&spear13xx_wdt_device,
	&spear13xx_pcie_gadget0_device,

	/* Board Specific devices */
	&spear900_phy0_device,
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

/* spi0 touch screen Chip Select Control function, controlled by gpio pin */
static struct stmpe610_pdata stmpe610_spi_pdata = {
	.irq_gpio = GPIO1_6,
	.irq_type = IRQ_TYPE_EDGE_FALLING,
	.fifo_threshhold = 1,
	.tracking_index = TI_0,
	.operating_mode = XYZ_ACQUISITION,
	.average_ctrl = SAMPLES_2,
	.touch_det_delay = TD_500US,
	.settling_time = ST_500US,
	.x_min = 0x00,
	.x_max = 0xFFF,
	.y_min = 0x00,
	.y_max = 0xFFF,
	.sample_time = SAMP_TIME_80,
	.mod_12b = MOD_12B,
	.ref_sel = REF_SEL_INT,
	.adc_freq = ADC_FREQ_3250K,
	.fraction_z = 7,
	.i_drive = IDRIVE_50_80MA,
};

DECLARE_SPI_CS_CONTROL(0, ts, GPIO1_7);
/* spi0 touch screen Info structure */
static struct pl022_config_chip spi0_ts_chip_info = {
	.lbm = LOOPBACK_DISABLED,
	.iface = SSP_INTERFACE_MOTOROLA_SPI,
	.hierarchy = SSP_MASTER,
	.slave_tx_disable = 0,
	.endian_tx = SSP_TX_LSB,
	.endian_rx = SSP_RX_LSB,
	.data_size = SSP_DATA_BITS_8,
	.com_mode = INTERRUPT_TRANSFER,
	.rx_lev_trig = SSP_RX_1_OR_MORE_ELEM,
	.tx_lev_trig = SSP_TX_1_OR_MORE_EMPTY_LOC,
	.clk_phase = SSP_CLK_SECOND_EDGE,
	.clk_pol = SSP_CLK_POL_IDLE_HIGH,
	.cs_control = spi0_ts_cs_control,
};

static struct spi_board_info __initdata spi_board_info[] = {
	/* spi0 board info */
	{
		.modalias = "stmpe610-spi",
		.platform_data = &stmpe610_spi_pdata,
		.controller_data = &spi0_ts_chip_info,
		.max_speed_hz = 1000000,
		.bus_num = 0,
		.chip_select = 0,
	},
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

#ifdef CONFIG_PCIEPORTBUS
static struct pcie_port_info __initdata pcie_port_info[] = {
	/*pcie0 port info*/
	{
		.is_host = 0,
	}, {
	/*pcie1 port info*/
		.is_host = 1,
	}, {
	/*pcie2 port info*/
		.is_host = 1,
	}
};
/* this function is needed for PCIE host and device driver. Same
 * controller can not be programmed as host as well as device. So host
 * driver must call this function and if this function returns a
 * configuration structure which tells that this port should be a host, then
 * only host controller driver should add that particular port as RC.
 * For a port to be added as device, one must also add device's information
 * in plat_devs array defined in this file.
 * it is the responsibility of calling function to not send port number
 * greter than max no of controller(3)
 */
static struct pcie_port_info *spear1300_pcie_port_init(int port)
{
	if (port < 3)
		return &pcie_port_info[port];
	else
		return NULL;
}
#endif

static void spear900_evb_fixup(struct machine_desc *desc, struct tag *tags,
		char **cmdline, struct meminfo *mi)
{
#if defined(CONFIG_FB_DB9000) || defined(CONFIG_FB_DB9000_MODULE)
	unsigned long size;

	size = clcd_get_fb_size(&chimei_b101aw02_info, NUM_OF_FRAMEBUFFERS);
	db900fb_buffer_phys = reserve_mem(mi, ALIGN(size, SZ_1M));
	if (db900fb_buffer_phys == ~0)
		pr_err("Unable to allocate fb buffer\n");
#endif
}

static void __init spear900_evb_init(void)
{

	/* set adc platform data */
	set_adc_plat_data(&spear13xx_adc_device, &spear13xx_dmac_device[0].dev);

	/* set keyboard plat data */
	kbd_set_plat_data(&spear13xx_kbd_device, &kbd_data);
#if (defined(CONFIG_FB_DB9000) || defined(CONFIG_FB_DB9000_MODULE))
	chimei_b101aw02_info.frame_buf_base = db900fb_buffer_phys;
	clcd_set_plat_data(&spear13xx_db9000_clcd_device,
			&chimei_b101aw02_info);
#endif
	/* set jpeg configurations for DMA xfers */
	set_jpeg_dma_configuration(&spear13xx_jpeg_device,
			&spear13xx_dmac_device[0].dev);

	/* set nand device's plat data */
	nand_set_plat_data(&spear13xx_nand_device, NULL, 0, NAND_SKIP_BBTSCAN,
			SPEAR_NAND_BW8);
	nand_mach_init(SPEAR_NAND_BW8);

#ifdef CONFIG_SND_SOC_STA529
	/* configure i2s configuration for dma xfer */
	pcm_init(&spear13xx_dmac_device[0].dev);
#endif
	/* call spear900 machine init function */
	spear900_init(NULL, pmx_devs, ARRAY_SIZE(pmx_devs));

	/* Register slave devices on the I2C buses */
	i2c_register_board_devices();

	/* initialize serial nor related data in smi plat data */
	smi_init_board_info(&spear13xx_smi_device);

	/* initialize fsmc related data in fsmc plat data */
	fsmc_init_board_info(&spear13xx_fsmc_nor_device, partition_info,
			ARRAY_SIZE(partition_info), FSMC_FLASH_WIDTH8);

#ifdef CONFIG_PCIEPORTBUS
	/* Enable PCIE0 clk */
	enable_pcie0_clk();
	pcie_init(&spear1300_pcie_port_init);
#endif

	/* Add Platform Devices */
	platform_add_devices(plat_devs, ARRAY_SIZE(plat_devs));

	/* Add Amba Devices */
	spear_amba_device_register(amba_devs, ARRAY_SIZE(amba_devs));

	/* Initialize fsmc regiters */
	fsmc_nor_init(&spear13xx_fsmc_nor_device, SPEAR13XX_FSMC_BASE, 0,
			FSMC_FLASH_WIDTH8);

	spi_init();
}

MACHINE_START(SPEAR900, "ST-SPEAR900-EVB")
	.boot_params	=	0x00000100,
	.fixup          =	spear900_evb_fixup,
	.map_io		=	spear13xx_map_io,
	.init_irq	=	spear13xx_init_irq,
	.timer		=	&spear13xx_timer,
	.init_machine	=	spear900_evb_init,
MACHINE_END
