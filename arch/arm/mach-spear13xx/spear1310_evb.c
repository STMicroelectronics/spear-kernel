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
#include <linux/mtd/fsmc.h>
#include <linux/mtd/nand.h>
#include <linux/pata_arasan_cf_data.h>
#include <linux/spi/spi.h>
#include <linux/stmpe610.h>
#include <asm/mach-types.h>
#include <plat/adc.h>
#include <plat/fsmc.h>
#include <plat/hdlc.h>
#include <plat/jpeg.h>
#include <plat/keyboard.h>
#include <plat/smi.h>
#include <plat/spi.h>
#include <mach/db9000fb_info.h>
#include <mach/generic.h>
#include <mach/hardware.h>
#include <mach/spear1310_misc_regs.h>
#include <mach/pcie.h>

/* fsmc nor partition info */
#define PARTITION(n, off, sz)	{.name = n, .offset = off, .size = sz}
static struct mtd_partition partition_info[] = {
	PARTITION("X-loader", 0, 1 * 0x20000),
	PARTITION("U-Boot", 0x20000, 3 * 0x20000),
	PARTITION("Kernel", 0x80000, 24 * 0x20000),
	PARTITION("Root File System", 0x380000, 84 * 0x20000),
};

/* padmux devices to enable */
static struct pmx_dev *pmx_devs[] = {
	/* spear13xx specific devices */
	&spear13xx_pmx_i2c,
	&spear13xx_pmx_ssp,
	&spear13xx_pmx_i2s1,
	&spear13xx_pmx_i2s2,
	&spear13xx_pmx_clcd,
	&spear13xx_pmx_clcd_hires,
	&spear13xx_pmx_egpio_grp,
	&spear13xx_pmx_smi_2_chips,
	&spear13xx_pmx_smi_4_chips,
	&spear13xx_pmx_gmii,
	&spear13xx_pmx_nand_8bit,
	&spear13xx_pmx_nand_16bit,
	&spear13xx_pmx_keyboard_6x6,
	&spear13xx_pmx_keyboard_9x9,
	&spear13xx_pmx_uart0,
	&spear13xx_pmx_uart0_modem,
	&spear13xx_pmx_gpt_0_1,
	&spear13xx_pmx_gpt_0_2,
	&spear13xx_pmx_gpt_1_1,
	&spear13xx_pmx_gpt_1_2,
	&spear13xx_pmx_mcif,
	&spear13xx_pmx_sdhci,
	&spear13xx_pmx_cf,
	&spear13xx_pmx_xd,

	/* spear1310 specific devices */
	&spear1310_pmx_uart_1_dis_i2c,
	&spear1310_pmx_uart_1_dis_sd,
	&spear1310_pmx_uart_2_3,
	&spear1310_pmx_uart_4,
	&spear1310_pmx_uart_5,
	&spear1310_pmx_rs485_0_1_tdm_0_1,
	&spear1310_pmx_i2c_1_2,
	&spear1310_pmx_i2c3_dis_smi_clcd,
	&spear1310_pmx_i2c3_dis_sd_i2s1,
	&spear1310_pmx_i2c_4_5_dis_smi,
	&spear1310_pmx_i2c4_dis_sd,
	&spear1310_pmx_i2c5_dis_sd,
	&spear1310_pmx_i2c_6_7_dis_kbd,
	&spear1310_pmx_i2c6_dis_sd,
	&spear1310_pmx_i2c7_dis_sd,
	&spear1310_pmx_rgmii,
	&spear1310_pmx_can0_dis_nor,
	&spear1310_pmx_can0_dis_sd,
	&spear1310_pmx_can1_dis_sd,
	&spear1310_pmx_can1_dis_kbd,
	&spear1310_pmx_pci,
	&spear1310_pmx_smii_0_1_2,
	&spear1310_pmx_ssp1_dis_kbd,
	&spear1310_pmx_ssp1_dis_sd,
	&spear1310_pmx_gpt64,
	&spear1310_pmx_ras_mii_txclk,
	&spear1310_pmx_pcie0,
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
	&spear1310_uart1_device,
	&spear1310_uart2_device,
	&spear1310_uart3_device,
	&spear1310_uart4_device,
	&spear1310_uart5_device,
	&spear1310_ssp1_device,
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
	&spear13xx_fsmc_nor_device,
	&spear13xx_i2c_device,
	&spear13xx_i2s0_device,
	&spear13xx_jpeg_device,
	&spear13xx_kbd_device,
	&spear13xx_nand_device,
	&spear13xx_ohci0_device,
	&spear13xx_ohci1_device,
	&spear13xx_pcie_gadget0_device,
	&spear13xx_pcm_device,
	&spear13xx_rtc_device,
	&spear13xx_sdhci_device,
	&spear13xx_smi_device,
	&spear13xx_wdt_device,

	/* spear1310 specific devices */
	&spear1310_can0_device,
	&spear1310_can1_device,
	&spear1310_i2c1_device,
	&spear1310_i2c2_device,
	&spear1310_i2c3_device,
	&spear1310_i2c4_device,
	&spear1310_i2c5_device,
	&spear1310_i2c6_device,
	&spear1310_i2c7_device,
	&spear1310_plgpio_device,
	&spear1310_tdm_hdlc_0_device,
	&spear1310_tdm_hdlc_1_device,
	&spear1310_rs485_0_device,
	&spear1310_rs485_1_device,
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
DECLARE_SPI_CS_CONTROL(0, ts, SPEAR1310_SSP0_CS_SEL_CS0);
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
		.modalias = "stmpe610-spi",
		.platform_data = &stmpe610_spi_pdata,
		.controller_data = &spi0_ts_chip_info,
		.max_speed_hz = 1000000,
		.bus_num = 0,
		.chip_select = SPEAR1310_SSP0_CS_SEL_CS0,
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

/*
 * This function is needed for PCIE host and device driver. Same
 * controller can not be programmed as host as well as device. So host
 * driver must call this function and if this function returns a
 * configuration structure which tells that this port should be a host, then
 * only host controller driver should add that particular port as RC.
 * For a port to be added as device, one must also add device's information
 * in plat_devs array defined in this file.
 */
static struct pcie_port_info *__init spear1310_pcie_port_init(int port)
{
	if (port < 3)
		return &pcie_port_info[port];
	else
		return NULL;
}
#endif

/* spear1310 ras misc configurations */
#if 0
static void __init ras_fsmc_config(u32 mode, u32 width)
{
	u32 val, *address;

	address = ioremap(SPEAR1310_RAS_CTRL_REG0, SZ_16);

	val = readl(address);
	val &= ~(RAS_FSMC_MODE_MASK | RAS_FSMC_WIDTH_MASK);
	val |= mode;
	val |= width;
	val |= RAS_FSMC_CS_SPLIT;

	writel(val, address);

	iounmap(address);
}
#endif

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
	fsmc_nor_init(NULL, SPEAR1310_FSMC1_BASE, 2, FSMC_FLASH_WIDTH8);

	e1phy_init(SPEAR1310_FSMC1_CS2_BASE + (pdev->id * 0x100), 0);
	tdm_hdlc_set_plat_data(pdev, 32);
}
#endif

static void spear1310_evb_fixup(struct machine_desc *desc,
		struct tag *tags, char **cmdline, struct meminfo *mi)
{
#if defined(CONFIG_FB_DB9000) || defined(CONFIG_FB_DB9000_MODULE)
	unsigned long size;

	size = clcd_get_fb_size(&sharp_lcd_info, NUM_OF_FRAMEBUFFERS);
	sharp_lcd_info.frame_buf_base = reserve_mem(mi, ALIGN(size, SZ_1M));
	if (sharp_lcd_info.frame_buf_base == ~0)
		pr_err("Unable to allocate fb buffer\n");
#endif
}

static void __init spear1310_evb_init(void)
{
	unsigned int i;

	/* set adc platform data */
	set_adc_plat_data(&spear13xx_adc_device, &spear13xx_dmac_device[0].dev);

	/* set compact flash plat data */
	set_arasan_cf_pdata(&spear13xx_cf_device, &cf_pdata);

#if (defined(CONFIG_FB_DB9000) || defined(CONFIG_FB_DB9000_MODULE))
	/* db9000_clcd plat data */
	clcd_set_plat_data(&spear13xx_db9000_clcd_device, &sharp_lcd_info);
#endif
	/* set jpeg configurations for DMA xfers */
	set_jpeg_dma_configuration(&spear13xx_jpeg_device,
			&spear13xx_dmac_device[0].dev);

	/* set keyboard plat data */
	kbd_set_plat_data(&spear13xx_kbd_device, &kbd_data);

	/* initialize serial nor related data in smi plat data */
	smi_init_board_info(&spear13xx_smi_device);

	/*
	 * SPEAr13xx FSMC cannot used as NOR and NAND at the same time
	 * For the moment, disable NAND and use NOR only
	 * If NAND is needed, enable the following code and disable all code for
	 * NOR. Also enable nand in padmux configuration to use it.
	 */
	/* set nand device's plat data */
#if 0
	/* set nand device's plat data */
	fsmc_nand_set_plat_data(&spear13xx_nand_device, NULL, 0,
			NAND_SKIP_BBTSCAN, FSMC_NAND_BW8);
	nand_mach_init(FSMC_NAND_BW8);
#endif

	/* fixed part fsmc nor device */
	/* initialize fsmc related data in fsmc plat data */
	fsmc_init_board_info(&spear13xx_fsmc_nor_device, partition_info,
			ARRAY_SIZE(partition_info), FSMC_FLASH_WIDTH8);
	/* Initialize fsmc regiters */
	fsmc_nor_init(&spear13xx_fsmc_nor_device, SPEAR13XX_FSMC_BASE, 0,
			FSMC_FLASH_WIDTH8);

#ifdef CONFIG_SND_SOC_STA529
	/* configure i2s configuration for dma xfer */
	pcm_init(&spear13xx_dmac_device[0].dev);
#endif

	/* call spear1310 machine init function */
	spear1310_init(NULL, pmx_devs, ARRAY_SIZE(pmx_devs));

	/* Register slave devices on the I2C buses */
	i2c_register_default_devices();

#ifdef CONFIG_PCIEPORTBUS
	/* Enable PCIE0 clk */
	enable_pcie0_clk();
	pcie_init(spear1310_pcie_port_init);
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
