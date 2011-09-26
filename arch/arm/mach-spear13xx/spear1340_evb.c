/*
 * arch/arm/mach-spear13xx/spear1340_evb.c
 *
 * SPEAr1340 evaluation board source file
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
#include <linux/i2c.h>
#include <linux/i2c/l3g4200d.h>
#include <linux/irq.h>
#include <linux/mtd/fsmc.h>
#include <linux/mtd/nand.h>
#include <linux/pata_arasan_cf_data.h>
#include <linux/phy.h>
#include <linux/spi/spi.h>
#include <linux/stmmac.h>
#include <linux/stmpe610.h>
#include <plat/adc.h>
#include <plat/fsmc.h>
#include <plat/keyboard.h>
#include <plat/smi.h>
#include <plat/spi.h>
#include <mach/db9000fb_info.h>
#include <mach/generic.h>
#include <mach/gpio.h>
#include <mach/hardware.h>
#include <mach/spear1340_misc_regs.h>
#include <mach/spear_pcie.h>
#include <media/soc_camera.h>

#ifdef CONFIG_SPEAR1340_PLUG_BOARDS
/* Variable specifying which plug boards are requested */
extern char spear1340_plug_board[50];
#endif

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
	.phy_addr = -1,
	.phy_mask = 0,
	.interface = PHY_INTERFACE_MODE_RGMII,
	.phy_clk_cfg = spear13xx_eth_phy_clk_cfg,
};

static struct resource phy0_resources = {
	.name = "phyirq",
	.start = -1,
	.end = -1,
	.flags = IORESOURCE_IRQ,
};

struct platform_device spear1340_phy0_device = {
	.name		= "stmmacphy",
	.id		= 0,
	.num_resources	= 1,
	.resource	= &phy0_resources,
	.dev.platform_data = &phy0_private_data,
};

/*
 * Pad multiplexing for making few pads as plgpio's.
 * Please retain original values and addresses, and update only mask as
 * required.
 * For example: if we need to enable plgpio's on pads: 15, 28, 45 & 102.
 * They corresponds to following bits in registers: 16, 29, 46 & 103
 * So following mask entries will solve this purpose:
 * Reg1: .mask = 0x20010000,
 * Reg2: .mask = 0x00004000,
 * Reg4: .mask = 0x00000080,
 *
 * Note: Counting of bits and pads start from 0.
 */
static struct pmx_mux_reg pmx_plgpios_mux[] = {
	{
		.address = SPEAR1340_PAD_FUNCTION_EN_1,
		.mask = 0x0,
		.value = 0x0,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_2,
		.mask = 0x0,
		.value = 0x0,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_3,
		.mask = 0x0,
		.value = 0x0,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_4,
		.mask = 0x0,
		.value = 0x0,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_5,
		.mask = 0x0,
		.value = 0x0,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_6,
		.mask = 0x0,
		.value = 0x0,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_7,
		.mask = 0x0,
		.value = 0x0,
	}, {
		.address = SPEAR1340_PAD_FUNCTION_EN_8,
		.mask = 0x0,
		.value = 0x0,
	},
};

static struct pmx_dev_mode pmx_plgpios_modes[] = {
	{
		.mux_regs = pmx_plgpios_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_plgpios_mux),
	},
};

static struct pmx_dev spear1340_pmx_plgpios = {
	.name = "plgpios",
	.modes = pmx_plgpios_modes,
	.mode_count = ARRAY_SIZE(pmx_plgpios_modes),
};

/* camera sensor registeration */
static struct i2c_board_info vs6725_camera_sensor_info[] = {
	{
		I2C_BOARD_INFO("vs6725", 0x10),
	},
};

static struct soc_camera_link vs6725_cam3_sensor_iclink = {
	.bus_id = 3,	/* sensor is connected to cam3 */
	.i2c_adapter_id = 0,
	.board_info = &vs6725_camera_sensor_info[0],
	.module_name = "vs6725",
};

struct platform_device spear1340_cam3_sensor_device = {
	.name = "soc-camera-pdrv",
	.id = -1,
	.dev = {
		.platform_data = &vs6725_cam3_sensor_iclink,
	},
};

/* padmux devices to enable */
static struct pmx_dev *pmx_devs[] = {
	/*
	 * Keep pads_as_gpio as the first element in this array. Don't ever
	 * remove it. It makes all pads as gpio's in starting, and then pads are
	 * configured as peripherals wherever required.
	 */
	&spear1340_pmx_pads_as_gpio,
	&spear1340_pmx_fsmc_8bit,
	&spear1340_pmx_keyboard_row_col,
	&spear1340_pmx_keyboard_col5,
	&spear1340_pmx_uart0_enh,
	&spear1340_pmx_i2c1,
	&spear1340_pmx_spdif_in,
	&spear1340_pmx_ssp0_cs1,
	&spear1340_pmx_pwm2,
	&spear1340_pmx_pwm3,
	&spear1340_pmx_video_in_mux_cam0,
	&spear1340_pmx_video_in_mux_cam1,
	&spear1340_pmx_video_in_mux_cam2,
	&spear1340_pmx_cam3,
	&spear1340_pmx_smi,
	&spear1340_pmx_ssp0,
	&spear1340_pmx_ssp0_cs2,
	&spear1340_pmx_uart0,
	&spear1340_pmx_uart1,
	&spear1340_pmx_i2s_in,
	&spear1340_pmx_i2s_out,
	&spear1340_pmx_gmac,
	&spear1340_pmx_ssp0_cs3,
	&spear1340_pmx_i2c0,
	&spear1340_pmx_cec0,
	&spear1340_pmx_cec1,
	&spear1340_pmx_spdif_out,
	&spear1340_pmx_mcif,
	&spear1340_pmx_sdhci,
	&spear1340_pmx_clcd,
	&spear1340_pmx_devs_grp,
	&spear1340_pmx_rgmii,
	&spear1340_pmx_pcie,

	/* Keep this entry at the bottom of table to override earlier setting */
	&spear1340_pmx_plgpios,
};

static struct amba_device *amba_devs[] __initdata = {
	/* spear13xx specific devices */
	&spear13xx_gpio_device[0],
	&spear13xx_gpio_device[1],
	&spear13xx_ssp_device,
	&spear13xx_uart_device,

	/* spear1340 specific devices */
	&spear1340_uart1_device,
};

static struct platform_device *plat_devs[] __initdata = {
	/* spear13xx specific devices */
	&spear13xx_adc_device,
	&spear13xx_db9000_clcd_device,
	&spear13xx_dmac_device[0],
	&spear13xx_dmac_device[1],
	&spear13xx_ehci0_device,
	&spear13xx_ehci1_device,
	&spear13xx_eth_device,
	&spear13xx_i2c_device,
	&spear1340_nand_device,
	&spear1340_i2s_play_device,
	&spear1340_i2s_record_device,
	&spear13xx_ohci0_device,
	&spear13xx_ohci1_device,
	&spear13xx_pcm_device,
	&spear13xx_pcie_host0_device,
	&spear13xx_rtc_device,
	&spear13xx_sdhci_device,
	&spear13xx_smi_device,
	&spear13xx_wdt_device,

	/* spear1340 specific devices */
	&spear1340_camif3_device,
	&spear1340_cam3_sensor_device,
	&spear1340_i2c1_device,
	&spear1340_pwm_device,
	&spear1340_phy0_device,
	&spear1340_plgpio_device,
	&spear1340_otg_device,
};

static struct arasan_cf_pdata cf_pdata = {
	.cf_if_clk = CF_IF_CLK_166M,
	.quirk = CF_BROKEN_UDMA,
	.dma_priv = &cf_dma_priv,
};

/* keyboard specific platform data */
static DECLARE_6x6_KEYMAP(keymap);
static struct matrix_keymap_data keymap_data = {
	.keymap = keymap,
	.keymap_size = ARRAY_SIZE(keymap),
};

static struct kbd_platform_data kbd_data = {
	.keymap = &keymap_data,
	.rep = 1,
	.mode = KEYPAD_6x6,
};

/* Initializing platform data for spear1340 evb specific I2C devices */
/* Gyroscope platform data */
static struct l3g4200d_gyr_platform_data l3g4200d_pdata = {
	.poll_interval = 5,
	.min_interval = 2,
	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,
};

static struct i2c_board_info __initdata i2c_board_info[] = {
	/* gyroscope board info */
	{
		.type = "l3g4200d_gyr",
		.addr = 0x69,
		.platform_data = &l3g4200d_pdata,
	},
};

/* spi master's configuration routine */
DECLARE_SPI_CS_CFG(0, VA_SPEAR1340_PERIP_CFG, SPEAR1340_SSP_CS_SEL_MASK,
		SPEAR1340_SSP_CS_SEL_SHIFT, SPEAR1340_SSP_CS_CTL_MASK,
		SPEAR1340_SSP_CS_CTL_SHIFT, SPEAR1340_SSP_CS_CTL_SW,
		SPEAR1340_SSP_CS_VAL_MASK, SPEAR1340_SSP_CS_VAL_SHIFT);

/* spi0 flash Chip Select Control function */
DECLARE_SPI_CS_CONTROL(0, flash, SPEAR1340_SSP_CS_SEL_CS0);
/* spi0 flash Chip Info structure */
DECLARE_SPI_CHIP_INFO(0, flash, spi0_flash_cs_control);

/* spi0 spidev Chip Select Control function */
DECLARE_SPI_CS_CONTROL(0, dev, SPEAR1340_SSP_CS_SEL_CS2);
/* spi0 spidev Chip Info structure */
DECLARE_SPI_CHIP_INFO(0, dev, spi0_dev_cs_control);

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

/* spi0 stmpe610 Chip Select Control function */
DECLARE_SPI_CS_CONTROL(0, ts, SPEAR1340_SSP_CS_SEL_CS1);
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
	.cs_control = spi0_ts_cs_control,
};

static struct spi_board_info __initdata spi_board_info[] = {
	{
		.modalias = "m25p80",
		.controller_data = &spi0_flash_chip_info,
		.max_speed_hz = 12000000,
		.bus_num = 0,
		.chip_select = SPEAR1340_SSP_CS_SEL_CS0,
		.mode = SPI_MODE_3,
	}, {
		.modalias = "stmpe610-spi",
		.platform_data = &stmpe610_spi_pdata,
		.controller_data = &spi0_ts_chip_info,
		.max_speed_hz = 1000000,
		.bus_num = 0,
		.chip_select = SPEAR1340_SSP_CS_SEL_CS1,
		.mode = SPI_MODE_1,
	}, {
		.modalias = "spidev",
		.controller_data = &spi0_dev_chip_info,
		.max_speed_hz = 25000000,
		.bus_num = 0,
		.chip_select = SPEAR1340_SSP_CS_SEL_CS2,
		.mode = SPI_MODE_1,
	}
};

#ifdef CONFIG_SPEAR_PCIE_REV370
/* This function is needed for board specific PCIe initilization */
static void __init spear1340_pcie_board_init(void)
{
	void *plat_data;

	plat_data = dev_get_platdata(&spear13xx_pcie_host0_device.dev);
	PCIE_PORT_INIT((struct pcie_port_info *)plat_data, SPEAR_PCIE_REV_3_70);
}
#endif


static void spear1340_evb_fixup(struct machine_desc *desc, struct tag *tags,
		char **cmdline, struct meminfo *mi)
{
#if defined(CONFIG_FB_DB9000) || defined(CONFIG_FB_DB9000_MODULE)
	unsigned long size;

	size = clcd_get_fb_size(&chimei_b101aw02_info, NUM_OF_FRAMEBUFFERS);
	chimei_b101aw02_info.frame_buf_base =
		reserve_mem(mi, ALIGN(size, SZ_1M));
	if (chimei_b101aw02_info.frame_buf_base == ~0)
		pr_err("Unable to allocate fb buffer\n");
#endif
}

static void __init spear1340_evb_init(void)
{
	unsigned int i;

	/* set adc platform data */
	set_adc_plat_data(&spear13xx_adc_device, &spear13xx_dmac_device[0].dev);

	/* set compact flash plat data */
	set_arasan_cf_pdata(&spear13xx_cf_device, &cf_pdata);

#if (defined(CONFIG_FB_DB9000) || defined(CONFIG_FB_DB9000_MODULE))
	/* db9000_clcd plat data */
	clcd_set_plat_data(&spear13xx_db9000_clcd_device,
			&chimei_b101aw02_info);
#endif

	/* set keyboard plat data */
	kbd_set_plat_data(&spear13xx_kbd_device, &kbd_data);

	/* initialize serial nor related data in smi plat data */
	smi_init_board_info(&spear13xx_smi_device);

	/*
	 * SPEAr1340 FSMC cannot used as NOR and NAND at the same time
	 * For the moment, disable NOR and use NAND only
	 * If NOR is needed, enable NOR's code and disable all code for NOR.
	 */
	/* set nand device's plat data */
	/* set nand device's plat data */
	fsmc_nand_set_plat_data(&spear1340_nand_device, NULL, 0,
			NAND_SKIP_BBTSCAN, FSMC_NAND_BW8);
	nand_mach_init(FSMC_NAND_BW8);

#if 0
	/* fixed part fsmc nor device */
	/* initialize fsmc related data in fsmc plat data */
	fsmc_init_board_info(&spear13xx_fsmc_nor_device, partition_info,
			ARRAY_SIZE(partition_info), FSMC_FLASH_WIDTH8);
	/* Initialize fsmc regiters */
	fsmc_nor_init(&spear13xx_fsmc_nor_device, SPEAR13XX_FSMC_BASE, 0,
			FSMC_FLASH_WIDTH8);
#endif

#ifdef CONFIG_SPEAR_PCIE_REV370
	/* Enable PCIE0 clk */
	enable_pcie0_clk();
	spear1340_pcie_board_init();
	writel(SPEAR1340_PCIE_SATA_MIPHY_CFG_PCIE,
			VA_SPEAR1340_PCIE_MIPHY_CFG);
#endif

#if 0
	/* Miphy configuration for SATA */
	writel(SPEAR1340_PCIE_SATA_MIPHY_CFG_SATA_25M_CRYSTAL_CLK,
			VA_SPEAR1340_PCIE_MIPHY_CFG);
#endif

	/* call spear1340 machine init function */
	spear1340_init(NULL, pmx_devs, ARRAY_SIZE(pmx_devs));

	/* Register spear1340 evb board specific i2c slave devices */
	i2c_register_board_info(0, i2c_board_info,
				ARRAY_SIZE(i2c_board_info));

	/* Register slave devices on the I2C buses */
	i2c_register_default_devices();

	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));

#ifdef CONFIG_SPEAR1340_PLUG_BOARDS
	/* Check if plug boards are requested or not */
	if (spear1340_plug_board[0] != '\0') {
		int ret = spear1340_pb_init(plat_devs, ARRAY_SIZE(plat_devs),
				amba_devs, ARRAY_SIZE(amba_devs));
		if (!ret)
			return;
	}
#endif

	/* Add Platform Devices */
	platform_add_devices(plat_devs, ARRAY_SIZE(plat_devs));

	/* Add Amba Devices */
	for (i = 0; i < ARRAY_SIZE(amba_devs); i++)
		amba_device_register(amba_devs[i], &iomem_resource);
}

MACHINE_START(SPEAR1340_EVB, "ST-SPEAR1340-EVB")
	.boot_params	=	0x00000100,
	.fixup		=	spear1340_evb_fixup,
	.map_io		=	spear13xx_map_io,
	.init_irq	=	spear13xx_init_irq,
	.timer		=	&spear13xx_timer,
	.init_machine	=	spear1340_evb_init,
MACHINE_END
