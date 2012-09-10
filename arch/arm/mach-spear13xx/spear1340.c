/*
 * arch/arm/mach-spear13xx/spear1340.c
 *
 * SPEAr1340 machine source file
 *
 * Copyright (C) 2012 ST Microelectronics
 * Viresh Kumar <viresh.linux@gmail.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#define pr_fmt(fmt) "SPEAr1340: " fmt

#include <linux/ahci_platform.h>
#include <linux/amba/serial.h>
#include <linux/delay.h>
#include <linux/dw_dmac.h>
#include <linux/of_platform.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/stmmac.h>
#include <linux/usb/dwc_otg.h>
#include <asm/hardware/gic.h>
#include <asm/mach/arch.h>
#include <mach/dma.h>
#include <mach/generic.h>
#include <mach/spdif.h>
#include <mach/spear.h>
#include <sound/designware_i2s.h>
#include <sound/pcm.h>

/* Base addresses */
#define SPEAR1340_SATA_BASE			UL(0xB1000000)
#define SPEAR1340_UART1_BASE			UL(0xB4100000)
#define SPEAR1340_I2S_PLAY_BASE			UL(0xB2400000)
#define SPEAR1340_I2S_REC_BASE			UL(0xB2000000)
#define SPEAR1340_SPDIF_IN_BASE			UL(0xD0100000)
#define SPEAR1340_SPDIF_OUT_BASE		UL(0xD0000000)

/* Power Management Registers */
#define SPEAR1340_PCM_CFG			(VA_MISC_BASE + 0x100)
#define SPEAR1340_PCM_WKUP_CFG			(VA_MISC_BASE + 0x104)
#define SPEAR1340_SWITCH_CTR			(VA_MISC_BASE + 0x108)

#define SPEAR1340_PERIP1_SW_RST			(VA_MISC_BASE + 0x318)
#define SPEAR1340_PERIP2_SW_RST			(VA_MISC_BASE + 0x31C)
#define SPEAR1340_PERIP3_SW_RST			(VA_MISC_BASE + 0x320)
	#define SPEAR1340_SPDIF_IN_RST			(1 << 12)

#define SPEAR1340_PERIP1_CLK_ENB		(VA_MISC_BASE + 0x30C)

/* PCIE - SATA configuration registers */
#define SPEAR1340_PCIE_SATA_CFG			(VA_MISC_BASE + 0x424)
	/* PCIE CFG MASks */
	#define SPEAR1340_PCIE_CFG_DEVICE_PRESENT	(1 << 11)
	#define SPEAR1340_PCIE_CFG_POWERUP_RESET	(1 << 10)
	#define SPEAR1340_PCIE_CFG_CORE_CLK_EN		(1 << 9)
	#define SPEAR1340_PCIE_CFG_AUX_CLK_EN		(1 << 8)
	#define SPEAR1340_SATA_CFG_TX_CLK_EN		(1 << 4)
	#define SPEAR1340_SATA_CFG_RX_CLK_EN		(1 << 3)
	#define SPEAR1340_SATA_CFG_POWERUP_RESET	(1 << 2)
	#define SPEAR1340_SATA_CFG_PM_CLK_EN		(1 << 1)
	#define SPEAR1340_PCIE_SATA_SEL_PCIE		(0)
	#define SPEAR1340_PCIE_SATA_SEL_SATA		(1)
	#define SPEAR1340_SATA_PCIE_CFG_MASK		0xF1F
	#define SPEAR1340_PCIE_CFG_VAL	(SPEAR1340_PCIE_SATA_SEL_PCIE | \
			SPEAR1340_PCIE_CFG_AUX_CLK_EN | \
			SPEAR1340_PCIE_CFG_CORE_CLK_EN | \
			SPEAR1340_PCIE_CFG_POWERUP_RESET | \
			SPEAR1340_PCIE_CFG_DEVICE_PRESENT)
	#define SPEAR1340_SATA_CFG_VAL	(SPEAR1340_PCIE_SATA_SEL_SATA | \
			SPEAR1340_SATA_CFG_PM_CLK_EN | \
			SPEAR1340_SATA_CFG_POWERUP_RESET | \
			SPEAR1340_SATA_CFG_RX_CLK_EN | \
			SPEAR1340_SATA_CFG_TX_CLK_EN)

#define VA_SPEAR1340_USBPHY_GEN_CFG		(VA_MISC_BASE + 0x414)
	#define USBPLLLOCK			(1 << 24)
	#define USBPHYRST			(1 << 15)
	#define USBPRSNT			(1 << 13)
	#define USBPHYPOR			(1 << 12)


#define SPEAR1340_PCIE_MIPHY_CFG		(VA_MISC_BASE + 0x428)
	#define SPEAR1340_MIPHY_OSC_BYPASS_EXT		(1 << 31)
	#define SPEAR1340_MIPHY_CLK_REF_DIV2		(1 << 27)
	#define SPEAR1340_MIPHY_CLK_REF_DIV4		(2 << 27)
	#define SPEAR1340_MIPHY_CLK_REF_DIV8		(3 << 27)
	#define SPEAR1340_MIPHY_PLL_RATIO_TOP(x)	(x << 0)
	#define SPEAR1340_PCIE_SATA_MIPHY_CFG_SATA \
			(SPEAR1340_MIPHY_OSC_BYPASS_EXT | \
			SPEAR1340_MIPHY_CLK_REF_DIV2 | \
			SPEAR1340_MIPHY_PLL_RATIO_TOP(60))
	#define SPEAR1340_PCIE_SATA_MIPHY_CFG_SATA_25M_CRYSTAL_CLK \
			(SPEAR1340_MIPHY_PLL_RATIO_TOP(120))
	#define SPEAR1340_PCIE_SATA_MIPHY_CFG_PCIE \
			(SPEAR1340_MIPHY_OSC_BYPASS_EXT | \
			SPEAR1340_MIPHY_PLL_RATIO_TOP(25))

#define SPEAR1340_UOC_RST_ENB                   11

static struct dw_dma_slave uart1_dma_param[] = {
	{
		/* Tx */
		.cfg_hi = DWC_CFGH_DST_PER(SPEAR1340_DMA_REQ_UART1_TX),
		.cfg_lo = 0,
		.src_master = DMA_MASTER_MEMORY,
		.dst_master = SPEAR1340_DMA_MASTER_UART1,
	}, {
		/* Rx */
		.cfg_hi = DWC_CFGH_SRC_PER(SPEAR1340_DMA_REQ_UART1_RX),
		.cfg_lo = 0,
		.src_master = SPEAR1340_DMA_MASTER_UART1,
		.dst_master = DMA_MASTER_MEMORY,
	}
};

static struct amba_pl011_data uart1_data = {
	.dma_filter = dw_dma_filter,
	.dma_tx_param = &uart1_dma_param[0],
	.dma_rx_param = &uart1_dma_param[1],
};

/* Ethernet platform data */
static struct stmmac_mdio_bus_data mdio0_private_data = {
	.bus_id = 0,
	.phy_mask = 0,
};

static struct stmmac_dma_cfg dma0_private_data = {
	.pbl = 16,
	.fixed_burst = 1,
	.burst_len = DMA_AXI_BLEN_ALL,
};

int spear1340_otg_phy_init(void)
{
	u32 temp, msec = 1000;
	void __iomem *usbphy_gen_cfg_reg, *perip1_clk_enb, *perip1_sw_rst;

	usbphy_gen_cfg_reg = VA_SPEAR1340_USBPHY_GEN_CFG;
	perip1_clk_enb = SPEAR1340_PERIP1_CLK_ENB;
	perip1_sw_rst = SPEAR1340_PERIP1_SW_RST;

	/* phy for deassert */
	temp = readl(usbphy_gen_cfg_reg);
	temp &= ~USBPHYPOR;
	writel(temp, usbphy_gen_cfg_reg);

	/* phy clock enable */
	temp = readl(usbphy_gen_cfg_reg);
	temp |= USBPHYRST;
	writel(temp, usbphy_gen_cfg_reg);

	/* wait for pll lock */
	while (!(readl(usbphy_gen_cfg_reg) & USBPLLLOCK)) {
		if (msec--) {
			pr_err(" Problem with USB PHY PLL Lock\n");
			return -ETIMEDOUT;
		}
		udelay(1);
	}

	/* otg prstnt deassert */
	temp = readl(usbphy_gen_cfg_reg);
	temp |= USBPRSNT;
	writel(temp, usbphy_gen_cfg_reg);

	/* OTG HCLK Disable */
	temp = readl(perip1_clk_enb);
	temp &= ~(1 << SPEAR1340_UOC_RST_ENB);
	writel(temp, perip1_clk_enb);

	/* OTG HRESET deassert */
	temp = readl(perip1_sw_rst);
	temp &= ~(1 << SPEAR1340_UOC_RST_ENB);
	writel(temp, perip1_sw_rst);

	/* OTG HCLK Enable */
	temp = readl(perip1_clk_enb);
	temp |= (1 << SPEAR1340_UOC_RST_ENB);
	writel(temp, perip1_clk_enb);

	return 0;
}

static struct dwc_otg_plat_data spear1340_otg_plat_data = {
	.phy_init = spear1340_otg_phy_init,
};

/* spdif-in device registeration */
static void spdif_in_reset(void)
{
	writel(readl(SPEAR1340_PERIP3_SW_RST) | SPEAR1340_SPDIF_IN_RST,
		SPEAR1340_PERIP3_SW_RST);

	writel(readl(SPEAR1340_PERIP3_SW_RST) & ~SPEAR1340_SPDIF_IN_RST,
		SPEAR1340_PERIP3_SW_RST);
}

static struct dw_dma_slave spdif_in_dma_data = {
	/* Record */
	.dma_master_id = 0,
	.cfg_hi = DWC_CFGH_SRC_PER(SPEAR1340_DMA_REQ_SPDIF_RX),
	.cfg_lo = 0,
	.src_master = SPEAR1340_DMA_MASTER_SPDIF,
	.dst_master = DMA_MASTER_MEMORY,
};

static struct spdif_platform_data spdif_in_data = {
	.dma_params = &spdif_in_dma_data,
	.filter = dw_dma_filter,
	.reset_perip = spdif_in_reset,
};

/* spdif-out device registeration */
static struct dw_dma_slave spdif_out_dma_data = {
	/* Play */
	.dma_master_id = 0,
	.cfg_hi = DWC_CFGH_DST_PER(SPEAR1340_DMA_REQ_SPDIF_TX),
	.cfg_lo = 0,
	.src_master = DMA_MASTER_MEMORY,
	.dst_master = SPEAR1340_DMA_MASTER_SPDIF,
};

static struct spdif_platform_data spdif_out_data = {
	.dma_params = &spdif_out_dma_data,
	.filter = dw_dma_filter,
};


/* i2s:play device registration */
static struct dw_dma_slave i2s_play_dma_data = {
	/* Play */
	.dma_master_id = 0,
	.cfg_hi = DWC_CFGH_DST_PER(SPEAR1340_DMA_REQ_I2S_TX),
	.cfg_lo = 0,
	.src_master = DMA_MASTER_MEMORY,
	.dst_master = SPEAR1340_DMA_MASTER_I2S_PLAY,
};

static struct i2s_platform_data i2s_play_data = {
	.play_dma_data = &i2s_play_dma_data,
	.snd_fmts = SNDRV_PCM_FMTBIT_S16_LE,
	.snd_rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_48000),
	.filter = dw_dma_filter,
	.i2s_clk_cfg = audio_clk_config,
	.clk_init = i2s_clk_init,
};

/* i2s:record device registeration */
static struct dw_dma_slave i2s_capture_dma_data = {
	/* Record */
	.dma_master_id = 0,
	.cfg_hi = DWC_CFGH_SRC_PER(SPEAR1340_DMA_REQ_I2S_RX),
	.cfg_lo = 0,
	.src_master = SPEAR1340_DMA_MASTER_I2S_REC,
	.dst_master = DMA_MASTER_MEMORY,
};

static struct i2s_platform_data i2s_capture_data = {
	.capture_dma_data = &i2s_capture_dma_data,
	.snd_fmts = SNDRV_PCM_FMTBIT_S16_LE,
	.snd_rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_48000),
	.filter = dw_dma_filter,
	.i2s_clk_cfg = audio_clk_config,
	.clk_init = i2s_clk_init,
};
static struct plat_stmmacenet_data eth_data = {
	.bus_id = 0,
	.phy_addr = -1,
	.interface = PHY_INTERFACE_MODE_RGMII,
	.has_gmac = 1,
	.enh_desc = 1,
	.tx_coe = 1,
	.dma_cfg = &dma0_private_data,
	.rx_coe = STMMAC_RX_COE_TYPE2,
	.bugged_jumbo = 1,
	.pmt = 1,
	.mdio_bus_data = &mdio0_private_data,
	.init = spear13xx_eth_phy_clk_cfg,
	.clk_csr = STMMAC_CSR_150_250M,
};

/* SATA device registration */
static int sata_miphy_init(struct device *dev, void __iomem *addr)
{
	writel(SPEAR1340_SATA_CFG_VAL, SPEAR1340_PCIE_SATA_CFG);
	writel(SPEAR1340_PCIE_SATA_MIPHY_CFG_SATA_25M_CRYSTAL_CLK,
			SPEAR1340_PCIE_MIPHY_CFG);
	/* Switch on sata power domain */
	writel((readl(SPEAR1340_PCM_CFG) | (0x800)), SPEAR1340_PCM_CFG);
	msleep(20);
	/* Disable PCIE SATA Controller reset */
	writel((readl(SPEAR1340_PERIP1_SW_RST) & (~0x1000)),
			SPEAR1340_PERIP1_SW_RST);
	msleep(20);

	return 0;
}

void sata_miphy_exit(struct device *dev)
{
	writel(0, SPEAR1340_PCIE_SATA_CFG);
	writel(0, SPEAR1340_PCIE_MIPHY_CFG);

	/* Enable PCIE SATA Controller reset */
	writel((readl(SPEAR1340_PERIP1_SW_RST) | (0x1000)),
			SPEAR1340_PERIP1_SW_RST);
	msleep(20);
	/* Switch off sata power domain */
	writel((readl(SPEAR1340_PCM_CFG) & (~0x800)), SPEAR1340_PCM_CFG);
	msleep(20);
}

int sata_suspend(struct device *dev)
{
	if (dev->power.power_state.event == PM_EVENT_FREEZE)
		return 0;

	sata_miphy_exit(dev);

	return 0;
}

int sata_resume(struct device *dev)
{
	if (dev->power.power_state.event == PM_EVENT_THAW)
		return 0;

	return sata_miphy_init(dev, NULL);
}

static struct ahci_platform_data sata_pdata = {
	.init = sata_miphy_init,
	.exit = sata_miphy_exit,
	.suspend = sata_suspend,
	.resume = sata_resume,
};

/* Add SPEAr1340 auxdata to pass platform data */
static struct of_dev_auxdata spear1340_auxdata_lookup[] __initdata = {
	OF_DEV_AUXDATA("st,spear-adc", SPEAR13XX_ADC_BASE, NULL, &adc_pdata),
	OF_DEV_AUXDATA("arasan,cf-spear1340", MCIF_CF_BASE, NULL, &cf_dma_priv),
	OF_DEV_AUXDATA("snps,dma-spear1340", DMAC0_BASE, NULL, &dmac_plat_data0),
	OF_DEV_AUXDATA("snps,dma-spear1340", DMAC1_BASE, NULL, &dmac_plat_data1),
	OF_DEV_AUXDATA("arm,pl022", SSP_BASE, NULL, &pl022_plat_data),

	OF_DEV_AUXDATA("snps,spear-ahci", SPEAR1340_SATA_BASE, NULL,
			&sata_pdata),
	OF_DEV_AUXDATA("arm,pl011", SPEAR1340_UART1_BASE, NULL, &uart1_data),
	OF_DEV_AUXDATA("st,spear600-gmac", SPEAR13XX_GETH_BASE, NULL,
			&eth_data),
	OF_DEV_AUXDATA("snps,usb-otg", SPEAR_UOC_BASE, NULL,
			&spear1340_otg_plat_data),
	OF_DEV_AUXDATA("st,db9000-clcd", SPEAR13XX_CLCD_BASE, NULL,
			&clcd_plat_info),
	OF_DEV_AUXDATA("snps,designware-i2s", SPEAR1340_I2S_PLAY_BASE, NULL,
			&i2s_play_data),
	OF_DEV_AUXDATA("snps,designware-i2s", SPEAR1340_I2S_REC_BASE, NULL,
			&i2s_capture_data),
	OF_DEV_AUXDATA("st,spdif-out", SPEAR1340_SPDIF_OUT_BASE, NULL,
			&spdif_out_data),
	OF_DEV_AUXDATA("st,spdif-in", SPEAR1340_SPDIF_IN_BASE, NULL,
			&spdif_in_data),
	{}
};

static void __init spear1340_dt_init(void)
{
	spear13xx_l2x0_init();
	of_platform_populate(NULL, of_default_bus_match_table,
			spear1340_auxdata_lookup, NULL);
}

static const char * const spear1340_dt_board_compat[] = {
	"st,spear1340",
	"st,spear1340-evb",
	NULL,
};

DT_MACHINE_START(SPEAR1340_DT, "ST SPEAr1340 SoC with Flattened Device Tree")
	.map_io		=	spear13xx_map_io,
	.init_irq	=	spear13xx_dt_init_irq,
	.reserve	=	spear13xx_reserve_mem,
	.handle_irq	=	gic_handle_irq,
	.timer		=	&spear13xx_timer,
	.init_machine	=	spear1340_dt_init,
	.restart	=	spear_restart,
	.dt_compat	=	spear1340_dt_board_compat,
MACHINE_END
