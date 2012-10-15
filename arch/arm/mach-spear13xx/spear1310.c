/*
 * arch/arm/mach-spear13xx/spear1310.c
 *
 * SPEAr1310 machine source file
 *
 * Copyright (C) 2012 ST Microelectronics
 * Viresh Kumar <viresh.linux@gmail.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#define pr_fmt(fmt) "SPEAr1310: " fmt

#include <linux/ahci_platform.h>
#include <linux/delay.h>
#include <linux/amba/pl022.h>
#include <linux/clk.h>
#include <linux/of_platform.h>
#include <linux/pata_arasan_cf_data.h>
#include <linux/phy.h>
#include <linux/stmmac.h>
#include <linux/usb/dwc_otg.h>
#include <asm/hardware/gic.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <mach/generic.h>
#include <mach/spear.h>

/* Base addresses */
#define SPEAR1310_GETH1_BASE			UL(0x6D000000)
#define SPEAR1310_GETH2_BASE			UL(0x6D100000)
#define SPEAR1310_GETH3_BASE			UL(0x6D200000)
#define SPEAR1310_GETH4_BASE			UL(0x6D300000)
#define SPEAR1310_SSP1_BASE			UL(0x5D400000)
#define SPEAR1310_SATA0_BASE			UL(0xB1000000)
#define SPEAR1310_SATA1_BASE			UL(0xB1800000)
#define SPEAR1310_SATA2_BASE			UL(0xB4000000)

/* RAS Area Control Register */
#define VA_SPEAR1310_RAS_CTRL_REG1	(VA_SPEAR1310_RAS_BASE + 0x4)

#define SPEAR1310_GETH1_PHY_INTF_MASK	(0x7 << 4)
#define SPEAR1310_GETH2_PHY_INTF_MASK	(0x7 << 7)
#define SPEAR1310_GETH3_PHY_INTF_MASK	(0x7 << 10)
#define SPEAR1310_GETH4_PHY_INTF_MASK	(0x7 << 13)
#define SPEAR1310_PHY_RGMII_VAL		0x1
#define SPEAR1310_PHY_RMII_VAL		0x4
#define SPEAR1310_PHY_SMII_VAL		0x6

#define SPEAR1310_PCM_CFG			(VA_MISC_BASE + 0x100)

#define VA_SPEAR1310_USBPHY_GEN_CFG		(VA_MISC_BASE + 0x394)
	#define USBPLLLOCK			(1 << 24)
	#define USBPHYRST			(1 << 15)
	#define USBPRSNT			(1 << 13)
	#define USBPHYPOR			(1 << 12)

#define SPEAR1310_PCIE_SATA_CFG			(VA_MISC_BASE + 0x3A4)
	#define SPEAR1310_PCIE_SATA2_SEL_PCIE		(0 << 31)
	#define SPEAR1310_PCIE_SATA1_SEL_PCIE		(0 << 30)
	#define SPEAR1310_PCIE_SATA0_SEL_PCIE		(0 << 29)
	#define SPEAR1310_PCIE_SATA2_SEL_SATA		(1 << 31)
	#define SPEAR1310_PCIE_SATA1_SEL_SATA		(1 << 30)
	#define SPEAR1310_PCIE_SATA0_SEL_SATA		(1 << 29)
	#define SPEAR1310_SATA2_CFG_TX_CLK_EN		(1 << 27)
	#define SPEAR1310_SATA2_CFG_RX_CLK_EN		(1 << 26)
	#define SPEAR1310_SATA2_CFG_POWERUP_RESET	(1 << 25)
	#define SPEAR1310_SATA2_CFG_PM_CLK_EN		(1 << 24)
	#define SPEAR1310_SATA1_CFG_TX_CLK_EN		(1 << 23)
	#define SPEAR1310_SATA1_CFG_RX_CLK_EN		(1 << 22)
	#define SPEAR1310_SATA1_CFG_POWERUP_RESET	(1 << 21)
	#define SPEAR1310_SATA1_CFG_PM_CLK_EN		(1 << 20)
	#define SPEAR1310_SATA0_CFG_TX_CLK_EN		(1 << 19)
	#define SPEAR1310_SATA0_CFG_RX_CLK_EN		(1 << 18)
	#define SPEAR1310_SATA0_CFG_POWERUP_RESET	(1 << 17)
	#define SPEAR1310_SATA0_CFG_PM_CLK_EN		(1 << 16)
	#define SPEAR1310_PCIE2_CFG_DEVICE_PRESENT	(1 << 11)
	#define SPEAR1310_PCIE2_CFG_POWERUP_RESET	(1 << 10)
	#define SPEAR1310_PCIE2_CFG_CORE_CLK_EN		(1 << 9)
	#define SPEAR1310_PCIE2_CFG_AUX_CLK_EN		(1 << 8)
	#define SPEAR1310_PCIE1_CFG_DEVICE_PRESENT	(1 << 7)
	#define SPEAR1310_PCIE1_CFG_POWERUP_RESET	(1 << 6)
	#define SPEAR1310_PCIE1_CFG_CORE_CLK_EN		(1 << 5)
	#define SPEAR1310_PCIE1_CFG_AUX_CLK_EN		(1 << 4)
	#define SPEAR1310_PCIE0_CFG_DEVICE_PRESENT	(1 << 3)
	#define SPEAR1310_PCIE0_CFG_POWERUP_RESET	(1 << 2)
	#define SPEAR1310_PCIE0_CFG_CORE_CLK_EN		(1 << 1)
	#define SPEAR1310_PCIE0_CFG_AUX_CLK_EN		(1 << 0)

	#define SPEAR1310_PCIE_CFG_MASK(x) ((0xF << (x * 4)) | (1 << (x + 29)))
	#define SPEAR1310_SATA_CFG_MASK(x) ((0xF << (x * 4 + 16)) | \
						(1 << (x + 29)))
	#define SPEAR1310_PCIE_CFG_VAL(x) \
			(SPEAR1310_PCIE_SATA##x##_SEL_PCIE | \
			SPEAR1310_PCIE##x##_CFG_AUX_CLK_EN | \
			SPEAR1310_PCIE##x##_CFG_CORE_CLK_EN | \
			SPEAR1310_PCIE##x##_CFG_POWERUP_RESET | \
			SPEAR1310_PCIE##x##_CFG_DEVICE_PRESENT)
	#define SPEAR1310_SATA_CFG_VAL(x) \
			(SPEAR1310_PCIE_SATA##x##_SEL_SATA | \
			SPEAR1310_SATA##x##_CFG_PM_CLK_EN | \
			SPEAR1310_SATA##x##_CFG_POWERUP_RESET | \
			SPEAR1310_SATA##x##_CFG_RX_CLK_EN | \
			SPEAR1310_SATA##x##_CFG_TX_CLK_EN)

#define SPEAR1310_PCIE_MIPHY_CFG_1		(VA_MISC_BASE + 0x3A8)
	#define SPEAR1310_MIPHY_DUAL_OSC_BYPASS_EXT	(1 << 31)
	#define SPEAR1310_MIPHY_DUAL_CLK_REF_DIV2	(1 << 28)
	#define SPEAR1310_MIPHY_DUAL_PLL_RATIO_TOP(x)	(x << 16)
	#define SPEAR1310_MIPHY_SINGLE_OSC_BYPASS_EXT	(1 << 15)
	#define SPEAR1310_MIPHY_SINGLE_CLK_REF_DIV2	(1 << 12)
	#define SPEAR1310_MIPHY_SINGLE_PLL_RATIO_TOP(x)	(x << 0)
	#define SPEAR1310_PCIE_SATA_MIPHY_CFG_SATA_MASK (0xFFFF)
	#define SPEAR1310_PCIE_SATA_MIPHY_CFG_PCIE_MASK (0xFFFF << 16)
	#define SPEAR1310_PCIE_SATA_MIPHY_CFG_SATA \
			(SPEAR1310_MIPHY_DUAL_OSC_BYPASS_EXT | \
			SPEAR1310_MIPHY_DUAL_CLK_REF_DIV2 | \
			SPEAR1310_MIPHY_DUAL_PLL_RATIO_TOP(60) | \
			SPEAR1310_MIPHY_SINGLE_OSC_BYPASS_EXT | \
			SPEAR1310_MIPHY_SINGLE_CLK_REF_DIV2 | \
			SPEAR1310_MIPHY_SINGLE_PLL_RATIO_TOP(60))
	#define SPEAR1310_PCIE_SATA_MIPHY_CFG_SATA_25M_CRYSTAL_CLK \
			(SPEAR1310_MIPHY_SINGLE_PLL_RATIO_TOP(120))
	#define SPEAR1310_PCIE_SATA_MIPHY_CFG_PCIE \
			(SPEAR1310_MIPHY_DUAL_OSC_BYPASS_EXT | \
			SPEAR1310_MIPHY_DUAL_PLL_RATIO_TOP(25) | \
			SPEAR1310_MIPHY_SINGLE_OSC_BYPASS_EXT | \
			SPEAR1310_MIPHY_SINGLE_PLL_RATIO_TOP(25))

#define SPEAR1310_PERIP1_CLK_ENB		(VA_MISC_BASE + 0x300)
#define SPEAR1310_PERIP1_SW_RST			(VA_MISC_BASE + 0x308)
#define SPEAR1310_UOC_RST_ENB			11

static struct arasan_cf_pdata cf_pdata = {
	.cf_if_clk = CF_IF_CLK_166M,
	.quirk = CF_BROKEN_UDMA,
	.dma_priv = &cf_dma_priv,
};

static int spear1310_eth_phy_clk_cfg(struct platform_device *pdev)
{
	struct plat_stmmacenet_data *pdata = dev_get_platdata(&pdev->dev);
	void __iomem *addr = IOMEM(VA_SPEAR1310_RAS_CTRL_REG1);
	struct clk *clk, *phy_clk = NULL;
	u32 tmp;
	int ret = 0;
	char *pclk_name[] = {
		"ras_pll2_clk",
		"ras_tx125_clk",
		"ras_tx50_clk",
		"ras_syn0_clk",
	};
	const char *phy_clk_name[] = {
		"stmmacphy.0",
		"stmmacphy.1",
		"stmmacphy.2",
		"stmmacphy.3",
		"stmmacphy.4",
	};

	phy_clk = clk_get(NULL, phy_clk_name[pdata->bus_id]);
	if (IS_ERR(phy_clk)) {
		ret = PTR_ERR(phy_clk);
		goto fail_get_phy_clk;
	}

	tmp = (pdata->interface == PHY_INTERFACE_MODE_RMII) ? 3 : 0;
	clk = clk_get(NULL, pclk_name[tmp]);
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		goto fail_get_pclk;
	}

	tmp = readl(addr);
	switch (pdata->bus_id) {
	case 1:
		tmp &= (~SPEAR1310_GETH1_PHY_INTF_MASK);
		tmp |= (pdata->interface == PHY_INTERFACE_MODE_MII) ?
			(SPEAR1310_PHY_SMII_VAL << 4) :
			(SPEAR1310_PHY_RMII_VAL << 4);
		break;
	case 2:
		tmp &= (~SPEAR1310_GETH2_PHY_INTF_MASK);
		tmp |= (pdata->interface == PHY_INTERFACE_MODE_MII) ?
			(SPEAR1310_PHY_SMII_VAL << 7) :
			(SPEAR1310_PHY_RMII_VAL << 7);
		break;
	case 3:
		tmp &= (~SPEAR1310_GETH3_PHY_INTF_MASK);
		tmp |= (pdata->interface == PHY_INTERFACE_MODE_MII) ?
			(SPEAR1310_PHY_SMII_VAL << 10) :
			(SPEAR1310_PHY_RMII_VAL << 10);
		break;
	case 4:
		tmp &= (~SPEAR1310_GETH4_PHY_INTF_MASK);
		tmp |= SPEAR1310_PHY_RGMII_VAL << 13;
		break;
	default:
		goto fail_phy_cfg;
	}

	writel(tmp, addr);
	ret = clk_set_parent(phy_clk, clk);
	if (IS_ERR_VALUE(ret))
		goto fail_phy_cfg;

	if (pdata->interface == PHY_INTERFACE_MODE_RMII) {
		ret = clk_set_rate(clk, 50000000);
		if (IS_ERR_VALUE(ret))
			goto fail_phy_cfg;
	}

	ret = clk_prepare_enable(phy_clk);

	if (!(IS_ERR_VALUE(ret)))
		return ret;
fail_phy_cfg:
	clk_put(clk);
fail_get_pclk:
	clk_put(phy_clk);
fail_get_phy_clk:
	return ret;
}

/* Ethernet GETH-0 device configuration */
static struct stmmac_mdio_bus_data mdio0_private_data = {
	.bus_id = 0,
	.phy_mask = 0,
};

static struct stmmac_dma_cfg dma0_private_data = {
	.pbl = 8,
	.fixed_burst = 1,
	.burst_len = DMA_AXI_BLEN_ALL,
};

static struct plat_stmmacenet_data eth0_data = {
	.bus_id = 0,
	.phy_addr = 5,
	.interface = PHY_INTERFACE_MODE_GMII,
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

/* Ethernet GETH-1 device configuration */
static struct stmmac_mdio_bus_data mdio1_private_data = {
	.bus_id = 1,
	.phy_mask = 0,
};

static struct stmmac_dma_cfg dma1_private_data = {
	.pbl = 8,
	.fixed_burst = 1,
	.burst_len = DMA_AXI_BLEN_ALL,
};

static struct plat_stmmacenet_data eth1_data = {
	.bus_id = 1,
	.phy_addr = 1,
	.interface = PHY_INTERFACE_MODE_MII,
	.has_gmac = 1,
	.enh_desc = 1,
	.tx_coe = 0,
	.dma_cfg = &dma1_private_data,
	.rx_coe = STMMAC_RX_COE_NONE,
	.bugged_jumbo = 1,
	.pmt = 1,
	.mdio_bus_data = &mdio1_private_data,
	.init = spear1310_eth_phy_clk_cfg,
	.clk_csr = STMMAC_CSR_150_250M,
};

/* Ethernet GETH-2 device configuration */
static struct stmmac_mdio_bus_data mdio2_private_data = {
	.bus_id = 2,
	.phy_mask = 0,
};

static struct stmmac_dma_cfg dma2_private_data = {
	.pbl = 8,
	.fixed_burst = 1,
	.burst_len = DMA_AXI_BLEN_ALL,
};

static struct plat_stmmacenet_data eth2_data = {
	.bus_id = 2,
	.phy_addr = 2,
	.interface = PHY_INTERFACE_MODE_MII,
	.has_gmac = 1,
	.enh_desc = 1,
	.tx_coe = 0,
	.dma_cfg = &dma2_private_data,
	.rx_coe = STMMAC_RX_COE_NONE,
	.bugged_jumbo = 1,
	.pmt = 1,
	.mdio_bus_data = &mdio2_private_data,
	.init = spear1310_eth_phy_clk_cfg,
	.clk_csr = STMMAC_CSR_150_250M,
};

/* Ethernet GETH-3 device configuration */
static struct stmmac_mdio_bus_data mdio3_private_data = {
	.bus_id = 3,
	.phy_mask = 0,
};

static struct stmmac_dma_cfg dma3_private_data = {
	.pbl = 8,
	.fixed_burst = 1,
	.burst_len = DMA_AXI_BLEN_ALL,
};

static struct plat_stmmacenet_data eth3_data = {
	.bus_id = 3,
	.phy_addr = 3,
	.interface = PHY_INTERFACE_MODE_RMII,
	.has_gmac = 1,
	.enh_desc = 1,
	.tx_coe = 0,
	.dma_cfg = &dma3_private_data,
	.rx_coe = STMMAC_RX_COE_NONE,
	.bugged_jumbo = 1,
	.pmt = 1,
	.mdio_bus_data = &mdio3_private_data,
	.init = spear1310_eth_phy_clk_cfg,
	.clk_csr = STMMAC_CSR_150_250M,
};

/* Ethernet GETH-4 device configuration */
static struct stmmac_mdio_bus_data mdio4_private_data = {
	.bus_id = 4,
	.phy_mask = 0,
};

static struct stmmac_dma_cfg dma4_private_data = {
	.pbl = 8,
	.fixed_burst = 1,
	.burst_len = DMA_AXI_BLEN_ALL,
};

static struct plat_stmmacenet_data eth4_data = {
	.bus_id = 4,
	.phy_addr = 4,
	.interface = PHY_INTERFACE_MODE_RGMII,
	.has_gmac = 1,
	.enh_desc = 1,
	.tx_coe = 0,
	.dma_cfg = &dma4_private_data,
	.rx_coe = STMMAC_RX_COE_NONE,
	.bugged_jumbo = 1,
	.pmt = 1,
	.mdio_bus_data = &mdio4_private_data,
	.init = spear1310_eth_phy_clk_cfg,
	.clk_csr = STMMAC_CSR_150_250M,
};

/* sata platfrom data */
static void sata_miphy_exit(struct device *dev)
{
	u32 temp;

	/*TBD: Workaround to support multiple sata port.*/
	temp = readl(SPEAR1310_PCIE_SATA_CFG);
	temp &= ~SPEAR1310_SATA_CFG_MASK(0);

	writel(temp, SPEAR1310_PCIE_SATA_CFG);
	writel(~SPEAR1310_PCIE_SATA_MIPHY_CFG_SATA_MASK,
			SPEAR1310_PCIE_MIPHY_CFG_1);

	/* Enable PCIE SATA Controller reset */
	writel((readl(SPEAR1310_PERIP1_SW_RST) | (0x1000)),
			SPEAR1310_PERIP1_SW_RST);
	msleep(20);
	/* Switch off sata power domain */
	writel((readl(SPEAR1310_PCM_CFG) & (~0x800)), SPEAR1310_PCM_CFG);
	msleep(20);
}

static int sata_miphy_init(struct device *dev, void __iomem *addr)
{
	u32 temp;

	/*TBD: Workaround to support multiple sata port.*/
	temp = readl(SPEAR1310_PCIE_SATA_CFG);
	temp &= ~SPEAR1310_SATA_CFG_MASK(0);
	temp |= SPEAR1310_SATA_CFG_VAL(0);

	writel(temp, SPEAR1310_PCIE_SATA_CFG);

	temp = readl(SPEAR1310_PCIE_MIPHY_CFG_1);
	temp &= ~SPEAR1310_PCIE_SATA_MIPHY_CFG_SATA_MASK;
	temp |= SPEAR1310_PCIE_SATA_MIPHY_CFG_SATA_25M_CRYSTAL_CLK;

	writel(temp, SPEAR1310_PCIE_MIPHY_CFG_1);
	/* Switch on sata power domain */
	writel((readl(SPEAR1310_PCM_CFG) | (0x800)), SPEAR1310_PCM_CFG);
	msleep(20);
	/* Disable PCIE SATA Controller reset */
	writel((readl(SPEAR1310_PERIP1_SW_RST) & (~0x1000)),
			SPEAR1310_PERIP1_SW_RST);
	msleep(20);

	return 0;
}

static struct ahci_platform_data sata_pdata = {
	.init = sata_miphy_init,
	.exit = sata_miphy_exit,
	.suspend = sata_suspend,
	.resume = sata_resume,
};

/* ssp device registration */
static struct pl022_ssp_controller ssp1_plat_data = {
	.enable_dma = 0,
};

int spear1310_otg_phy_init(void)
{
	u32 temp, msec = 1000;
	void __iomem *usbphy_gen_cfg_reg, *perip1_clk_enb, *perip1_sw_rst;

	usbphy_gen_cfg_reg = VA_SPEAR1310_USBPHY_GEN_CFG;
	perip1_clk_enb = SPEAR1310_PERIP1_CLK_ENB;
	perip1_sw_rst = SPEAR1310_PERIP1_SW_RST;

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
	temp &= ~(1 << SPEAR1310_UOC_RST_ENB);
	writel(temp, perip1_clk_enb);

	/* OTG HRESET deassert */
	temp = readl(perip1_sw_rst);
	temp &= ~(1 << SPEAR1310_UOC_RST_ENB);
	writel(temp, perip1_sw_rst);

	/* OTG HCLK Enable */
	temp = readl(perip1_clk_enb);
	temp |= (1 << SPEAR1310_UOC_RST_ENB);
	writel(temp, perip1_clk_enb);

	return 0;
}

static struct dwc_otg_plat_data spear1310_otg_plat_data = {
	.phy_init = spear1310_otg_phy_init,
};

/* Add SPEAr1310 auxdata to pass platform data */
static struct of_dev_auxdata spear1310_auxdata_lookup[] __initdata = {
	OF_DEV_AUXDATA("st,spear-adc", SPEAR13XX_ADC_BASE, NULL, &adc_pdata),
	OF_DEV_AUXDATA("arasan,cf-spear1340", MCIF_CF_BASE, NULL, &cf_pdata),
	OF_DEV_AUXDATA("snps,dma-spear1340", DMAC0_BASE, NULL, &dmac_plat_data0),
	OF_DEV_AUXDATA("snps,dma-spear1340", DMAC1_BASE, NULL, &dmac_plat_data1),
	OF_DEV_AUXDATA("arm,pl022", SSP_BASE, NULL, &pl022_plat_data),

	OF_DEV_AUXDATA("arm,pl022", SPEAR1310_SSP1_BASE, NULL, &ssp1_plat_data),
	OF_DEV_AUXDATA("st,spear600-gmac", SPEAR13XX_GETH_BASE, NULL,
			&eth0_data),
	OF_DEV_AUXDATA("st,spear600-gmac", SPEAR1310_GETH1_BASE, NULL,
			&eth1_data),
	OF_DEV_AUXDATA("st,spear600-gmac", SPEAR1310_GETH2_BASE, NULL,
			&eth2_data),
	OF_DEV_AUXDATA("st,spear600-gmac", SPEAR1310_GETH3_BASE, NULL,
			&eth3_data),
	OF_DEV_AUXDATA("st,spear600-gmac", SPEAR1310_GETH4_BASE, NULL,
			&eth4_data),
	OF_DEV_AUXDATA("snps,usb-otg", SPEAR_UOC_BASE, NULL,
			&spear1310_otg_plat_data),
	OF_DEV_AUXDATA("st,db9000-clcd", SPEAR13XX_CLCD_BASE, NULL,
			&clcd_plat_info),
	OF_DEV_AUXDATA("snps,spear-ahci", SPEAR1310_SATA0_BASE, NULL,
			&sata_pdata),
	{}
};

static void __init spear1310_dt_init(void)
{
	spear13xx_l2x0_init();
	of_platform_populate(NULL, of_default_bus_match_table,
			spear1310_auxdata_lookup, NULL);
}

static const char * const spear1310_dt_board_compat[] = {
	"st,spear1310",
	"st,spear1310-evb",
	NULL,
};

/*
 * Following will create 16MB static virtual/physical mappings
 * PHYSICAL		VIRTUAL
 * 0xD8000000		0xFA000000
 */
struct map_desc spear1310_io_desc[] __initdata = {
	{
		.virtual	= VA_SPEAR1310_RAS_GRP1_BASE,
		.pfn		= __phys_to_pfn(SPEAR1310_RAS_GRP1_BASE),
		.length		= SZ_16M,
		.type		= MT_DEVICE
	},
};

static void __init spear1310_map_io(void)
{
	iotable_init(spear1310_io_desc, ARRAY_SIZE(spear1310_io_desc));
	spear13xx_map_io();
}

DT_MACHINE_START(SPEAR1310_DT, "ST SPEAr1310 SoC with Flattened Device Tree")
	.map_io		=	spear1310_map_io,
	.reserve	=	spear13xx_reserve_mem,
	.init_irq	=	spear13xx_dt_init_irq,
	.handle_irq	=	gic_handle_irq,
	.timer		=	&spear13xx_timer,
	.init_machine	=	spear1310_dt_init,
	.restart	=	spear_restart,
	.dt_compat	=	spear1310_dt_board_compat,
MACHINE_END
