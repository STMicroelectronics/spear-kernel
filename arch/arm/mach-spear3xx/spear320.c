/*
 * arch/arm/mach-spear3xx/spear320.c
 *
 * SPEAr320 machine source file
 *
 * Copyright (C) 2009-2012 ST Microelectronics
 * Viresh Kumar <viresh.linux@gmail.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#define pr_fmt(fmt) "SPEAr320: " fmt

#include <linux/amba/pl022.h>
#include <linux/amba/pl08x.h>
#include <linux/amba/serial.h>
#include <linux/of_platform.h>
#include <linux/phy.h>
#include <linux/platform_data/macb.h>
#include <linux/stmmac.h>
#include <linux/usb/ch9.h>
#include <plat/udc.h>
#include <asm/hardware/vic.h>
#include <asm/mach/arch.h>
#include <mach/generic.h>
#include <mach/spear.h>
#include <sound/pcm.h>

#define SPEAR320_UART1_BASE		UL(0xA3000000)
#define SPEAR320_UART2_BASE		UL(0xA4000000)
#define SPEAR320_SSP0_BASE		UL(0xA5000000)
#define SPEAR320_SSP1_BASE		UL(0xA6000000)
#define SPEAR320_MACB0_BASE		UL(0xAA000000)
#define SPEAR320_MACB1_BASE		UL(0xAB000000)
#define SPEAR320_CLCD_BASE		UL(0x90000000)
#define SPEAR320_I2S_BASE              UL(0xA9400000)

void spear320_macb_plat_mdio_control(struct platform_device *pdev)
{
	u32 tmp, mask, shift;
	void __iomem *reg;
	struct macb_platform_data *pdata = dev_get_platdata(&pdev->dev);

	reg = SPEAR320_CONTROL_REG;
	mask = 0x1;
	shift = SPEAR320_MDIO_SEL_SHIFT;

	tmp = readl(reg);
	tmp &= ~(mask << shift);
	tmp |= pdata->bus_id << shift;
	writel(tmp, reg);
}

/* MACB platform data for SPEAr320 HMI board*/
static struct macb_platform_data spear320_hmi_macb_data[] = {
	{
		.bus_id = 0,
		.phy_mask = 0,
		.phy_addr = 0x1,
		.phy_irq_pin = -1,
		.plat_mdio_control = spear320_macb_plat_mdio_control,
	}, {
		.bus_id = 1,
		.phy_mask = 0,
		.phy_addr = 0x0,
		.phy_irq_pin = -1,
		.plat_mdio_control = spear320_macb_plat_mdio_control,
	},
};

/* MACB platform data for SPEAr320 Evaluation board*/
static struct macb_platform_data spear320_macb_data = {
	.bus_id = 1,
	.phy_mask = 0,
	.phy_addr = 0x2,
#ifdef PLGPIO_320
	.gpio_num = PLGPIO_76,
#endif
	.plat_mdio_control = spear320_macb_plat_mdio_control,
};

/* DMAC platform data's slave info */
struct pl08x_channel_data spear320_dma_info[] = {
	{
		.bus_id = "uart0_rx",
		.min_signal = 2,
		.max_signal = 2,
		.muxval = 0,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "uart0_tx",
		.min_signal = 3,
		.max_signal = 3,
		.muxval = 0,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "ssp0_rx",
		.min_signal = 8,
		.max_signal = 8,
		.muxval = 0,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "ssp0_tx",
		.min_signal = 9,
		.max_signal = 9,
		.muxval = 0,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "i2c0_rx",
		.min_signal = 10,
		.max_signal = 10,
		.muxval = 0,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "i2c0_tx",
		.min_signal = 11,
		.max_signal = 11,
		.muxval = 0,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "irda",
		.min_signal = 12,
		.max_signal = 12,
		.muxval = 0,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "adc",
		.min_signal = 13,
		.max_signal = 13,
		.muxval = 0,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "to_jpeg",
		.min_signal = 14,
		.max_signal = 14,
		.muxval = 0,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "from_jpeg",
		.min_signal = 15,
		.max_signal = 15,
		.muxval = 0,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "ssp1_rx",
		.min_signal = 0,
		.max_signal = 0,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB2,
	}, {
		.bus_id = "ssp1_tx",
		.min_signal = 1,
		.max_signal = 1,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB2,
	}, {
		.bus_id = "ssp2_rx",
		.min_signal = 2,
		.max_signal = 2,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB2,
	}, {
		.bus_id = "ssp2_tx",
		.min_signal = 3,
		.max_signal = 3,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB2,
	}, {
		.bus_id = "uart1_rx",
		.min_signal = 4,
		.max_signal = 4,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB2,
	}, {
		.bus_id = "uart1_tx",
		.min_signal = 5,
		.max_signal = 5,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB2,
	}, {
		.bus_id = "uart2_rx",
		.min_signal = 6,
		.max_signal = 6,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB2,
	}, {
		.bus_id = "uart2_tx",
		.min_signal = 7,
		.max_signal = 7,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB2,
	}, {
		.bus_id = "i2c1_rx",
		.min_signal = 8,
		.max_signal = 8,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB2,
	}, {
		.bus_id = "i2c1_tx",
		.min_signal = 9,
		.max_signal = 9,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB2,
	}, {
		.bus_id = "i2c2_rx",
		.min_signal = 10,
		.max_signal = 10,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB2,
	}, {
		.bus_id = "i2c2_tx",
		.min_signal = 11,
		.max_signal = 11,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB2,
	}, {
		.bus_id = "i2s_rx",
		.min_signal = 12,
		.max_signal = 12,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB2,
	}, {
		.bus_id = "i2s_tx",
		.min_signal = 13,
		.max_signal = 13,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB2,
	}, {
		.bus_id = "rs485_rx",
		.min_signal = 14,
		.max_signal = 14,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB2,
	}, {
		.bus_id = "rs485_tx",
		.min_signal = 15,
		.max_signal = 15,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB2,
	},
};

static struct pl022_ssp_controller spear320_ssp_data[] = {
	{
		.bus_id = 1,
		.enable_dma = 1,
		.dma_filter = pl08x_filter_id,
		.dma_tx_param = "ssp1_tx",
		.dma_rx_param = "ssp1_rx",
		.num_chipselect = 2,
	}, {
		.bus_id = 2,
		.enable_dma = 1,
		.dma_filter = pl08x_filter_id,
		.dma_tx_param = "ssp2_tx",
		.dma_rx_param = "ssp2_rx",
		.num_chipselect = 2,
	}
};

static struct amba_pl011_data spear320_uart_data[] = {
	{
		.dma_filter = pl08x_filter_id,
		.dma_tx_param = "uart1_tx",
		.dma_rx_param = "uart1_rx",
	}, {
		.dma_filter = pl08x_filter_id,
		.dma_tx_param = "uart2_tx",
		.dma_rx_param = "uart2_rx",
	},
};

int spear320_uart_clk_config(void)
{
	struct clk *uart1_clk, *ras_apb_clk, *uart2_clk;
	int ret;

	/* Get the ahb clock */
	ras_apb_clk = clk_get(NULL, "ras_apb_clk");
	if (IS_ERR(ras_apb_clk)) {
		ret = PTR_ERR(ras_apb_clk);
		return ret;
	}

	uart1_clk = clk_get_sys("a3000000.serial", NULL);
	if (IS_ERR(uart1_clk)) {
		ret = PTR_ERR(uart1_clk);
		goto fail_get_uart1_clk;
	}

	uart2_clk = clk_get_sys("a4000000.serial", NULL);
	if (IS_ERR(uart2_clk)) {
		ret = PTR_ERR(uart2_clk);
		goto fail_get_uart2_clk;
	}

	if (clk_set_parent(uart1_clk, ras_apb_clk)) {
		ret = -EPERM;
		pr_err("SPEAr320_evb: Failed to set uart1 parent clk\n");
		goto fail_set_parent_clk;
	}

	if (clk_set_parent(uart2_clk, ras_apb_clk)) {
		ret = -EPERM;
		pr_err("SPEAr320_evb: Failed to set uart2 parent clk\n");
		goto fail_set_parent_clk;
	}

fail_set_parent_clk:
	clk_put(uart2_clk);
fail_get_uart2_clk:
	clk_put(uart1_clk);
fail_get_uart1_clk:
	clk_put(ras_apb_clk);

	return ret;
}

/*
 * configure i2s ref clk and sclk
 *
 * Depending on these parameters sclk and ref clock will be configured.
 * For sclk:
 * sclk = channel_num * data_len * sample_rate
 *
 * For ref clock:
 *
 * ref_clock = 256 * sample_rate
 */

int audio_clk_config(struct i2s_hw_config_data *config)
{
	struct clk *i2s_sclk, *i2s_ref_clk;
	int ret = 0;


	i2s_sclk = clk_get_sys(NULL, "i2s_sclk");
	if (IS_ERR(i2s_sclk)) {
		pr_err("%s:couldn't get i2s_sclk\n", __func__);
		return PTR_ERR(i2s_sclk);
	}

	i2s_ref_clk = clk_get_sys(NULL, "i2s_ref_clk");
	if (IS_ERR(i2s_ref_clk)) {
		pr_err("%s:couldn't get i2s_ref_clk\n", __func__);
		ret = PTR_ERR(i2s_ref_clk);
		goto put_i2s_sclk;
	}

	/*
	 * 320 cannot generate accurate clock in some cases but slightly
	 * more than that which is under acceptable limits.
	 * But since clk_set_rate fails for this clock we explicitly try
	 * to set the higher clock.
	 */
#define REF_CLK_DELTA	10000
	ret = clk_set_rate(i2s_ref_clk, 256 * config->sample_rate +
			REF_CLK_DELTA);
	if (ret) {
		pr_err("%s:couldn't set i2s_ref_clk rate\n", __func__);
		goto put_i2s_ref_clk;
	}

	ret = clk_prepare_enable(i2s_sclk);
	if (ret) {
		pr_err("%s:enabling i2s_sclk\n", __func__);
		goto put_i2s_ref_clk;
	}

	return 0;

put_i2s_ref_clk:
	clk_put(i2s_ref_clk);
put_i2s_sclk:
	clk_put(i2s_sclk);

	return ret;

}

static void i2s_clk_init(void)
{
	int ret;
	struct clk *i2s_sclk, *pll1_clk, *gen2_3_par_clk;
	struct clk *i2s_ref_clk, *ras_syn2_gclk;

	pll1_clk = clk_get(NULL, "pll1_clk");
	if (IS_ERR(pll1_clk)) {
		pr_err("%s:couldn't get gen2_3_par_clk\n", __func__);
		return;
	}

	i2s_sclk = clk_get(NULL, "i2s_sclk");
	if (IS_ERR(i2s_sclk)) {
		pr_err("%s:couldn't get i2s_sclk\n", __func__);
		return;
	}

	gen2_3_par_clk = clk_get(NULL, "gen2_3_par_clk");
	if (IS_ERR(gen2_3_par_clk)) {
		pr_err("%s:couldn't get gen2_3_par_clk\n", __func__);
		goto put_i2s_sclk;
	}

	i2s_ref_clk = clk_get(NULL, "i2s_ref_clk");
	if (IS_ERR(i2s_ref_clk)) {
		pr_err("%s:couldn't get i2s_ref_clk\n", __func__);
		goto put_gen2_3_par_clk;
	}

	ras_syn2_gclk = clk_get(NULL, "ras_syn2_gclk");
	if (IS_ERR(ras_syn2_gclk)) {
		pr_err("%s:couldn't get ras_syn2_gclk\n", __func__);
		goto put_i2s_ref_clk;
	}

	ret = clk_set_parent(gen2_3_par_clk, pll1_clk);
	if (ret) {
		pr_err("%s:set_parent of gen2_3_par_clk,\n", __func__);
		goto put_i2s_ref_clk;
	}

	ret = clk_set_parent(i2s_ref_clk, ras_syn2_gclk);
	if (ret) {
		pr_err("%s:set_parent of i2s_ref_clk\n", __func__);
		goto put_i2s_ref_clk;
	}

	if (clk_set_rate(i2s_sclk, 3073000)) /* 3.072 Mhz */
		goto put_i2s_ref_clk;

	if (clk_prepare_enable(i2s_sclk))
		pr_err("%s:enabling i2s_sclk\n", __func__);

put_i2s_ref_clk:
	clk_put(i2s_ref_clk);
put_gen2_3_par_clk:
	clk_put(gen2_3_par_clk);
put_i2s_sclk:
	clk_put(i2s_sclk);
}

static struct i2s_platform_data i2s_data = {
	.snd_fmts = SNDRV_PCM_FMTBIT_S32_LE,
	.snd_rates = (SNDRV_PCM_RATE_8000 | \
			SNDRV_PCM_RATE_11025 | \
			SNDRV_PCM_RATE_16000 | \
			SNDRV_PCM_RATE_22050 | \
			SNDRV_PCM_RATE_32000 | \
			SNDRV_PCM_RATE_44100 | \
			SNDRV_PCM_RATE_48000),
	.play_dma_data = "i2s_tx",
	.capture_dma_data = "i2s_rx",
	.filter = pl08x_filter_id,
	.i2s_clk_cfg = audio_clk_config,
	.clk_init = i2s_clk_init,
};

/* AMBA clcd panel information */
static struct clcd_panel et057010_640x480 = {
	.mode = {
		.name = "ET057010 640x480",
		.refresh = 0,
		.xres = 640,
		.yres = 480,
		.pixclock = 48000,
		.left_margin = 144,
		.right_margin = 16,
		.upper_margin = 33,
		.lower_margin = 10,
		.hsync_len = 30,
		.vsync_len = 3,
		.sync = 0,
		.vmode = FB_VMODE_NONINTERLACED,
	},
	.width = -1,
	.height = -1,
	.tim2 = TIM2_CLKSEL | TIM2_IPC,
	.cntl = CNTL_LCDTFT | CNTL_BGR,
	.caps= CLCD_CAP_5551 | CLCD_CAP_565 | CLCD_CAP_888,
	.bpp = 32,
};

/* usbd plat data */
#define EP(idx, nam, size, maxpkt, address)	\
	[idx] = {				\
		.name		= nam,		\
		.fifo_size	= size,		\
		.maxpacket	= maxpkt,	\
		.addr		= address,	\
	}

struct udc_ep_data udc_ep[] = {
	EP(0, "ep0-ctrl", 64/4 , 64 , 0),
	EP(1, "ep1in"   , 512/4, 512, USB_DIR_IN  | 1),
	EP(2, "ep2out"  , 512/4, 512, USB_DIR_OUT | 2),
	EP(3, "ep3in"   , 512/4, 512, USB_DIR_IN  | 3),
	EP(4, "ep4out"  , 512/4, 512, USB_DIR_OUT | 4),
	EP(5, "ep5in"   , 512/4, 512, USB_DIR_IN  | 5),
	EP(6, "ep6out"  , 512/4, 512, USB_DIR_OUT | 6),
	EP(7, "ep7in"   , 512/4, 512, USB_DIR_IN  | 7),
	EP(8, "ep8out"  , 512/4, 512, USB_DIR_OUT | 8),
	EP(9, "ep9in"   , 512/4, 512, USB_DIR_IN  | 9),
	EP(10, "ep10out"   , 512/4, 512, USB_DIR_OUT | 10),
	EP(11, "ep11in"   , 512/4, 512, USB_DIR_IN  | 11),
	EP(12, "ep12out"   , 512/4, 512, USB_DIR_OUT | 12),
	EP(13, "ep13in"   , 512/4, 512, USB_DIR_IN  | 13),
	EP(14, "ep14out"   , 512/4, 512, USB_DIR_OUT | 14),
	EP(15, "ep15in"   , 512/4, 512, USB_DIR_IN  | 15),
};

struct udc_platform_data udc_plat_data = {
	.num_ep = 16,
	.ep = &udc_ep[0],
};

/* Ethernet platform data */
static struct stmmac_mdio_bus_data mdio0_private_data = {
	.bus_id = 0,
	.phy_mask = 0,
};

static struct stmmac_dma_cfg dma0_private_data = {
	.pbl = 8,
	.fixed_burst = 1,
	.burst_len = DMA_AXI_BLEN_ALL,
};

static struct plat_stmmacenet_data eth_data = {
	.bus_id = 0,
	.phy_addr = -1,
	.interface = PHY_INTERFACE_MODE_MII,
	.has_gmac = 1,
	.enh_desc = 1,
	.tx_coe = 1,
	.dma_cfg = &dma0_private_data,
	.rx_coe = STMMAC_RX_COE_TYPE2,
	.bugged_jumbo = 1,
	.pmt = 1,
	.mdio_bus_data = &mdio0_private_data,
	.clk_csr = STMMAC_CSR_150_250M,
};

/* Add SPEAr320 HMI auxdata to pass platform data */
static struct of_dev_auxdata spear320_hmi_auxdata_lookup[] __initdata = {
	OF_DEV_AUXDATA("st,spear-adc", SPEAR3XX_ICM1_ADC_BASE, NULL,
			&adc_pdata),
	OF_DEV_AUXDATA("arm,pl022", SPEAR3XX_ICM1_SSP_BASE, NULL,
			&pl022_plat_data),
	OF_DEV_AUXDATA("arm,pl080", SPEAR3XX_ICM3_DMA_BASE, NULL,
			&pl080_plat_data),
	OF_DEV_AUXDATA("arm,pl022", SPEAR320_SSP0_BASE, NULL,
			&spear320_ssp_data[0]),
	OF_DEV_AUXDATA("arm,pl022", SPEAR320_SSP1_BASE, NULL,
			&spear320_ssp_data[1]),
	OF_DEV_AUXDATA("arm,pl011", SPEAR320_UART1_BASE, NULL,
			&spear320_uart_data[0]),
	OF_DEV_AUXDATA("arm,pl011", SPEAR320_UART2_BASE, NULL,
			&spear320_uart_data[1]),
	OF_DEV_AUXDATA("st,spear320-macb", SPEAR320_MACB0_BASE, NULL,
			&spear320_hmi_macb_data[0]),
	OF_DEV_AUXDATA("st,spear320-macb", SPEAR320_MACB1_BASE, NULL,
			&spear320_hmi_macb_data[1]),
	OF_DEV_AUXDATA("arm,pl110", SPEAR320_CLCD_BASE, NULL,
			&pl110_plat_data),
	OF_DEV_AUXDATA("snps,designware-udc", SPEAR3XX_USBD_CSR_BASE, NULL,
			&udc_plat_data),
	OF_DEV_AUXDATA("snps,designware-i2s", SPEAR320_I2S_BASE, NULL,
			&i2s_data),
	OF_DEV_AUXDATA("st,designware-jpeg", SPEAR3XX_ICM1_JPEG_BASE, NULL,
			&jpeg_pdata),
	{}
};

/* Add SPEAr320 evb auxdata to pass platform data */
static struct of_dev_auxdata spear320_evb_auxdata_lookup[] __initdata = {
	OF_DEV_AUXDATA("st,spear-adc", SPEAR3XX_ICM1_ADC_BASE, NULL,
			&adc_pdata),
	OF_DEV_AUXDATA("arm,pl022", SPEAR3XX_ICM1_SSP_BASE, NULL,
			&pl022_plat_data),
	OF_DEV_AUXDATA("arm,pl080", SPEAR3XX_ICM3_DMA_BASE, NULL,
			&pl080_plat_data),
	OF_DEV_AUXDATA("arm,pl022", SPEAR320_SSP0_BASE, NULL,
			&spear320_ssp_data[0]),
	OF_DEV_AUXDATA("arm,pl022", SPEAR320_SSP1_BASE, NULL,
			&spear320_ssp_data[1]),
	OF_DEV_AUXDATA("arm,pl011", SPEAR320_UART1_BASE, NULL,
			&spear320_uart_data[0]),
	OF_DEV_AUXDATA("arm,pl011", SPEAR320_UART2_BASE, NULL,
			&spear320_uart_data[1]),
	OF_DEV_AUXDATA("st,spear320-macb", SPEAR320_MACB1_BASE, NULL,
			&spear320_macb_data),
	OF_DEV_AUXDATA("st,spear600-gmac", SPEAR3XX_GETH_BASE, NULL,
			&eth_data),
	OF_DEV_AUXDATA("snps,designware-udc", SPEAR3XX_USBD_CSR_BASE, NULL,
			&udc_plat_data),
	OF_DEV_AUXDATA("st,designware-jpeg", SPEAR3XX_ICM1_JPEG_BASE, NULL,
			&jpeg_pdata),
	{}
};

static void __init spear320_dt_init(void)
{
	pl080_plat_data.slave_channels = spear320_dma_info;
	pl080_plat_data.num_slave_channels = ARRAY_SIZE(spear320_dma_info);

	if (of_machine_is_compatible("st,spear320-evb")) {
		of_platform_populate(NULL, of_default_bus_match_table,
				spear320_evb_auxdata_lookup, NULL);
	} else if (of_machine_is_compatible("st,spear320-hmi")){
		/* clcd panel information */
		if (clcd_panel_setup(&et057010_640x480))
			pr_err("Error amba clcd panel configurtion\n");

		of_platform_populate(NULL, of_default_bus_match_table,
				spear320_hmi_auxdata_lookup, NULL);
	}

	spear3xx_emi_init(SPEAR320_SOC_EMI_BASE, 4);

	/* initialize macb related data in macb plat data */
	spear3xx_macb_setup();
}

static const char * const spear320_dt_board_compat[] = {
	"st,spear320",
	"st,spear320-evb",
	"st,spear320-hmi",
	NULL,
};

struct map_desc spear320_io_desc[] __initdata = {
	{
		.virtual	= VA_SPEAR320_SOC_CONFIG_BASE,
		.pfn		= __phys_to_pfn(SPEAR320_SOC_CONFIG_BASE),
		.length		= SZ_16M,
		.type		= MT_DEVICE
	},
};

static void __init spear320_map_io(void)
{
	iotable_init(spear320_io_desc, ARRAY_SIZE(spear320_io_desc));
	spear3xx_map_io();
}

DT_MACHINE_START(SPEAR320_DT, "ST SPEAr320 SoC with Flattened Device Tree")
	.map_io		=	spear320_map_io,
	.init_irq	=	spear3xx_dt_init_irq,
	.handle_irq	=	vic_handle_irq,
	.timer		=	&spear3xx_timer,
	.init_machine	=	spear320_dt_init,
	.restart	=	spear_restart,
	.dt_compat	=	spear320_dt_board_compat,
MACHINE_END
