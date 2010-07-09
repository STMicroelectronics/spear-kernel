/*
 * arch/arm/mach-spear13xx/spear1310.c
 *
 * SPEAr1310 machine source file
 *
 * Copyright (C) 2010 ST Microelectronics
 * Bhupesh Sharma <bhupesh.sharma@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/ptrace.h>
#include <linux/phy.h>
#include <linux/stmmac.h>
#include <linux/clk.h>
#include <asm/irq.h>
#include <mach/generic.h>
#include <mach/hardware.h>
#include <plat/clock.h>

#define GETH1_PHY_INTF_MASK	(0x7 << 4)
#define GETH2_PHY_INTF_MASK	(0x7 << 7)
#define GETH3_PHY_INTF_MASK	(0x7 << 10)
#define GETH4_PHY_INTF_MASK	(0x7 << 13)
#define PHY_INTF_MODE_RGMII	0x1
#define PHY_INTF_MODE_RMII	0x4
#define PHY_INTF_MODE_SMII	0x6

static int phy_clk_cfg(void *data)
{
	struct platform_device *pdev = (struct platform_device *)data;
	struct plat_stmmacphy_data *plat_dat = dev_get_platdata(&pdev->dev);
	void __iomem *addr = __io_address(RAS_CTRL_REG1);
	char *pclk_name[] = {
		"ras_pll2_clk",
		"ras_tx125_clk",
		"ras_tx50_clk",
	};
	struct clk *clk;
	u32 tmp;
	int ret;

	plat_dat->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(plat_dat->clk)) {
		ret = PTR_ERR(plat_dat->clk);
		goto fail_get_phy_clk;
	}
	/*
	 * Select 125 MHz clock for SMII mode, else the clock
	 * for RMII/RGMII mode is 50 Mhz.
	 * The default clock for the GMAC is driven by pll-2
	 * set to 125Mhz. In case the clock source is required to
	 * be from tx pad, the gmac0 interface should select that
	 * to pad clock.
	 */
	tmp = (plat_dat->interface == PHY_INTERFACE_MODE_MII) ? 0 : 2;

	clk = clk_get(NULL, pclk_name[tmp]);
	if (IS_ERR(clk)) {
		pr_err("%s:couldn't get %s as parent for MAC\n",
				__func__, pclk_name[tmp]);
		ret = PTR_ERR(clk);
		goto fail_get_pclk;
	}

	tmp = readl(addr);
	switch (plat_dat->bus_id) {
	case 1:
		tmp &= (~GETH1_PHY_INTF_MASK);
		tmp |= (plat_dat->interface == PHY_INTERFACE_MODE_MII) ?
			(PHY_INTF_MODE_SMII << 4) : (PHY_INTF_MODE_RMII << 4);
		break;
	case 2:
		tmp &= (~GETH2_PHY_INTF_MASK);
		tmp |= (plat_dat->interface == PHY_INTERFACE_MODE_MII) ?
			(PHY_INTF_MODE_SMII << 7) : (PHY_INTF_MODE_RMII << 7);
		break;
	case 3:
		tmp &= (~GETH3_PHY_INTF_MASK);
		tmp |= (plat_dat->interface == PHY_INTERFACE_MODE_MII) ?
			(PHY_INTF_MODE_SMII << 10) : (PHY_INTF_MODE_RMII << 10);
		break;
	case 4:
		tmp &= (~GETH4_PHY_INTF_MASK);
		tmp |= PHY_INTF_MODE_RGMII << 13;
		break;
	default:
		return -EINVAL;
		break;
	}
	writel(tmp, addr);
	clk_set_parent(plat_dat->clk, clk);
	ret = clk_enable(plat_dat->clk);
	return ret;
fail_get_pclk:
	clk_put(plat_dat->clk);
fail_get_phy_clk:
	return ret;
}

/* pmx driver structure */
struct pmx_driver pmx_driver;

/* Add spear1310 specific devices here */

/* CAN device registeration */
static struct resource can0_resources[] = {
	{
		.start = SPEAR1310_CAN0_BASE,
		.end = SPEAR1310_CAN0_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = IRQ_CCAN0,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device can0_device = {
	.name = "spear_can",
	.id = 0,
	.num_resources = ARRAY_SIZE(can0_resources),
	.resource = can0_resources,
};

static struct resource can1_resources[] = {
	{
		.start = SPEAR1310_CAN1_BASE,
		.end = SPEAR1310_CAN1_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = IRQ_CCAN1,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device can1_device = {
	.name = "spear_can",
	.id = 1,
	.num_resources = ARRAY_SIZE(can1_resources),
	.resource = can1_resources,
};

/* Ethernet device registeration */
static struct plat_stmmacphy_data phy1_private_data = {
	.bus_id = 1,
	.phy_addr = -1,
	.phy_mask = 0,
	.interface = PHY_INTERFACE_MODE_MII,
	.phy_clk_cfg = phy_clk_cfg,
};

static struct resource phy1_resources = {
	.name = "phyirq",
	.start = -1,
	.end = -1,
	.flags = IORESOURCE_IRQ,
};

struct platform_device phy1_device = {
	.name = "stmmacphy",
	.id = 1,
	.num_resources = 1,
	.resource = &phy1_resources,
	.dev.platform_data = &phy1_private_data,
};

static struct plat_stmmacenet_data ether1_platform_data = {
	.bus_id = 1,
	.has_gmac = 1,
	.enh_desc = 1,
	.tx_csum = 1,
};

static struct resource eth1_resources[] = {
	[0] = {
		.start = SPEAR1310_GETH1_BASE,
		.end = SPEAR1310_GETH1_BASE + SZ_8K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_GETH1_SBD,
		.flags = IORESOURCE_IRQ,
		.name = "macirq",
	},
	[2] = {
		.start = IRQ_GETH1_PMT,
		.flags = IORESOURCE_IRQ,
	},
};

static u64 eth1_dma_mask = ~(u32) 0;

struct platform_device eth1_device = {
	.name = "stmmaceth",
	.id = 1,
	.num_resources = ARRAY_SIZE(eth1_resources),
	.resource = eth1_resources,
	.dev = {
		.platform_data = &ether1_platform_data,
		.dma_mask = &eth1_dma_mask,
		.coherent_dma_mask = ~0,
	},
};

static struct plat_stmmacphy_data phy2_private_data = {
	.bus_id = 2,
	.phy_addr = -1,
	.phy_mask = 0,
	.interface = PHY_INTERFACE_MODE_MII,
	.phy_clk_cfg = phy_clk_cfg,
};

static struct resource phy2_resources = {
	.name = "phyirq",
	.start = -1,
	.end = -1,
	.flags = IORESOURCE_IRQ,
};

struct platform_device phy2_device = {
	.name = "stmmacphy",
	.id = 2,
	.num_resources = 1,
	.resource = &phy2_resources,
	.dev.platform_data = &phy2_private_data,
};

static struct plat_stmmacenet_data ether2_platform_data = {
	.bus_id = 2,
	.has_gmac = 1,
	.enh_desc = 1,
	.tx_csum = 1,
};

static struct resource eth2_resources[] = {
	[0] = {
		.start = SPEAR1310_GETH2_BASE,
		.end = SPEAR1310_GETH2_BASE + SZ_8K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_GETH2_SBD,
		.flags = IORESOURCE_IRQ,
		.name = "macirq",
	},
	[2] = {
		.start = IRQ_GETH2_PMT,
		.flags = IORESOURCE_IRQ,
	},
};

static u64 eth2_dma_mask = ~(u32) 0;

struct platform_device eth2_device = {
	.name = "stmmaceth",
	.id = 2,
	.num_resources = ARRAY_SIZE(eth2_resources),
	.resource = eth2_resources,
	.dev = {
		.platform_data = &ether2_platform_data,
		.dma_mask = &eth2_dma_mask,
		.coherent_dma_mask = ~0,
	},
};

static struct plat_stmmacphy_data phy3_private_data = {
	.bus_id = 3,
	.phy_addr = -1,
	.phy_mask = 0,
	.interface = PHY_INTERFACE_MODE_MII,
	.phy_clk_cfg = phy_clk_cfg,
};

static struct resource phy3_resources = {
	.name = "phyirq",
	.start = -1,
	.end = -1,
	.flags = IORESOURCE_IRQ,
};

struct platform_device phy3_device = {
	.name = "stmmacphy",
	.id = 3,
	.num_resources = 1,
	.resource = &phy3_resources,
	.dev.platform_data = &phy3_private_data,
};

static struct plat_stmmacenet_data ether3_platform_data = {
	.bus_id = 3,
	.has_gmac = 1,
	.enh_desc = 1,
	.tx_csum = 1,
};

static struct resource eth3_resources[] = {
	[0] = {
		.start = SPEAR1310_GETH3_BASE,
		.end = SPEAR1310_GETH3_BASE + SZ_8K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_GETH3_SBD,
		.flags = IORESOURCE_IRQ,
		.name = "macirq",
	},
	[2] = {
		.start = IRQ_GETH3_PMT,
		.flags = IORESOURCE_IRQ,
	},
};

static u64 eth3_dma_mask = ~(u32) 0;

struct platform_device eth3_device = {
	.name = "stmmaceth",
	.id = 3,
	.num_resources = ARRAY_SIZE(eth3_resources),
	.resource = eth3_resources,
	.dev = {
		.platform_data = &ether3_platform_data,
		.dma_mask = &eth3_dma_mask,
		.coherent_dma_mask = ~0,
	},
};

static struct plat_stmmacphy_data phy4_private_data = {
	.bus_id = 4,
	.phy_addr = -1,
	.phy_mask = 0,
	.interface = PHY_INTERFACE_MODE_RGMII,
	.phy_clk_cfg = phy_clk_cfg,
};

static struct resource phy4_resources = {
	.name = "phyirq",
	.start = -1,
	.end = -1,
	.flags = IORESOURCE_IRQ,
};

struct platform_device phy4_device = {
	.name = "stmmacphy",
	.id = 4,
	.num_resources = 1,
	.resource = &phy4_resources,
	.dev.platform_data = &phy4_private_data,
};

static struct plat_stmmacenet_data ether4_platform_data = {
	.bus_id = 4,
	.has_gmac = 1,
	.enh_desc = 1,
	.tx_csum = 1,
};

static struct resource eth4_resources[] = {
	[0] = {
		.start = SPEAR1310_GETH4_BASE,
		.end = SPEAR1310_GETH4_BASE + SZ_8K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_GETH4_SBD,
		.flags = IORESOURCE_IRQ,
		.name = "macirq",
	},
	[2] = {
		.start = IRQ_GETH4_PMT,
		.flags = IORESOURCE_IRQ,
	},
};

static u64 eth4_dma_mask = ~(u32) 0;

struct platform_device eth4_device = {
	.name = "stmmaceth",
	.id = 4,
	.num_resources = ARRAY_SIZE(eth4_resources),
	.resource = eth4_resources,
	.dev = {
		.platform_data = &ether4_platform_data,
		.dma_mask = &eth4_dma_mask,
		.coherent_dma_mask = ~0,
	},
};

void __init spear1310_init(void)
{
	int ret;

	/* call spear13xx family common init function */
	spear13xx_init();

	/* pmx initialization */
	pmx_driver.base = ioremap(SPEAR13XX_FUNC_ENB_BASE, SZ_4K);
	if (pmx_driver.base) {
		ret = pmx_register(&pmx_driver);
		if (ret)
			pr_err("padmux: registeration failed. err no: %d\n",
					ret);
		iounmap(pmx_driver.base);
	}
}
