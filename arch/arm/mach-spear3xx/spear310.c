/*
 * arch/arm/mach-spear3xx/spear310.c
 *
 * SPEAr310 machine source file
 *
 * Copyright (C) 2009-2012 ST Microelectronics
 * Viresh Kumar <viresh.linux@gmail.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#define pr_fmt(fmt) "SPEAr310: " fmt

#include <linux/amba/pl08x.h>
#include <linux/amba/serial.h>
#include <linux/of_platform.h>
#include <linux/phy.h>
#include <linux/platform_data/macb.h>
#include <linux/stmmac.h>
#include <asm/hardware/vic.h>
#include <asm/mach/arch.h>
#include <mach/generic.h>
#include <mach/spear.h>

#define SPEAR310_SOC_CONFIG_BASE	UL(0xB4000000)
#define VA_SPEAR310_SOC_CONFIG_BASE	UL(0xFE000000)

#define SPEAR310_SMII_MAC_CONF_OFF	0x000C
	#define PHY_CONTROL_MASK	0x3
	#define PHY_CONTROL_SHIFT	0x0

#define SPEAR310_UART1_BASE		UL(0xB2000000)
#define SPEAR310_UART2_BASE		UL(0xB2080000)
#define SPEAR310_UART3_BASE		UL(0xB2100000)
#define SPEAR310_UART4_BASE		UL(0xB2180000)
#define SPEAR310_UART5_BASE		UL(0xB2200000)
#define SPEAR310_MACB0_BASE		UL(0xb0000000)
#define SPEAR310_MACB1_BASE		UL(0xb0800000)
#define SPEAR310_MACB2_BASE		UL(0xb1000000)
#define SPEAR310_MACB3_BASE		UL(0xb1800000)

/* DMAC platform data's slave info */
struct pl08x_channel_data spear310_dma_info[] = {
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
		.bus_id = "i2c_rx",
		.min_signal = 10,
		.max_signal = 10,
		.muxval = 0,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "i2c_tx",
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
		.bus_id = "uart1_rx",
		.min_signal = 0,
		.max_signal = 0,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "uart1_tx",
		.min_signal = 1,
		.max_signal = 1,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "uart2_rx",
		.min_signal = 2,
		.max_signal = 2,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "uart2_tx",
		.min_signal = 3,
		.max_signal = 3,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "uart3_rx",
		.min_signal = 4,
		.max_signal = 4,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "uart3_tx",
		.min_signal = 5,
		.max_signal = 5,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "uart4_rx",
		.min_signal = 6,
		.max_signal = 6,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "uart4_tx",
		.min_signal = 7,
		.max_signal = 7,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "uart5_rx",
		.min_signal = 8,
		.max_signal = 8,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "uart5_tx",
		.min_signal = 9,
		.max_signal = 9,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "ras5_rx",
		.min_signal = 10,
		.max_signal = 10,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "ras5_tx",
		.min_signal = 11,
		.max_signal = 11,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "ras6_rx",
		.min_signal = 12,
		.max_signal = 12,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "ras6_tx",
		.min_signal = 13,
		.max_signal = 13,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "ras7_rx",
		.min_signal = 14,
		.max_signal = 14,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "ras7_tx",
		.min_signal = 15,
		.max_signal = 15,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	},
};

/* uart devices plat data */
static struct amba_pl011_data spear310_uart_data[] = {
	{
		.dma_filter = pl08x_filter_id,
		.dma_tx_param = "uart1_tx",
		.dma_rx_param = "uart1_rx",
	}, {
		.dma_filter = pl08x_filter_id,
		.dma_tx_param = "uart2_tx",
		.dma_rx_param = "uart2_rx",
	}, {
		.dma_filter = pl08x_filter_id,
		.dma_tx_param = "uart3_tx",
		.dma_rx_param = "uart3_rx",
	}, {
		.dma_filter = pl08x_filter_id,
		.dma_tx_param = "uart4_tx",
		.dma_rx_param = "uart4_rx",
	}, {
		.dma_filter = pl08x_filter_id,
		.dma_tx_param = "uart5_tx",
		.dma_rx_param = "uart5_rx",
	},
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

/* MACB platform data for SPEAr310 board*/
void spear310_macb_plat_mdio_control(struct platform_device *pdev)
{
	u32 tmp, mask, shift;
	void __iomem *reg;

	reg = IOMEM(VA_SPEAR310_SOC_CONFIG_BASE) +
			SPEAR310_SMII_MAC_CONF_OFF;
	mask = PHY_CONTROL_MASK;
	shift = PHY_CONTROL_SHIFT;

	tmp = readl(reg);
	tmp &= ~(mask << shift);
	tmp |= pdev->id << shift;
	writel(tmp, reg);
}

static struct macb_platform_data spear310_macb_data[] = {
	{
		.bus_id = 0,
		.phy_mask = 0,
		.phy_addr = 0x1,
		.phy_irq_pin = -1,
		.plat_mdio_control = spear310_macb_plat_mdio_control,
	}, {
		.bus_id = 1,
		.phy_mask = 0,
		.phy_addr = 0x3,
		.phy_irq_pin = -1,
		.plat_mdio_control = spear310_macb_plat_mdio_control,
	}, {
		.bus_id = 2,
		.phy_mask = 0,
		.phy_addr = 0x5,
		.phy_irq_pin = -1,
		.plat_mdio_control = spear310_macb_plat_mdio_control,
	}, {
		.bus_id = 3,
		.phy_mask = 0,
		.phy_addr = 0x7,
		.phy_irq_pin = -1,
		.plat_mdio_control = spear310_macb_plat_mdio_control,
	},
};

/* Add SPEAr310 auxdata to pass platform data */
static struct of_dev_auxdata spear310_auxdata_lookup[] __initdata = {
	OF_DEV_AUXDATA("st,spear-adc", SPEAR3XX_ICM1_ADC_BASE, NULL,
			&adc_pdata),
	OF_DEV_AUXDATA("arm,pl022", SPEAR3XX_ICM1_SSP_BASE, NULL,
			&pl022_plat_data),
	OF_DEV_AUXDATA("arm,pl080", SPEAR3XX_ICM3_DMA_BASE, NULL,
			&pl080_plat_data),
	OF_DEV_AUXDATA("arm,pl011", SPEAR310_UART1_BASE, NULL,
			&spear310_uart_data[0]),
	OF_DEV_AUXDATA("arm,pl011", SPEAR310_UART2_BASE, NULL,
			&spear310_uart_data[1]),
	OF_DEV_AUXDATA("arm,pl011", SPEAR310_UART3_BASE, NULL,
			&spear310_uart_data[2]),
	OF_DEV_AUXDATA("arm,pl011", SPEAR310_UART4_BASE, NULL,
			&spear310_uart_data[3]),
	OF_DEV_AUXDATA("arm,pl011", SPEAR310_UART5_BASE, NULL,
			&spear310_uart_data[4]),
	OF_DEV_AUXDATA("st,spear600-gmac", SPEAR3XX_GETH_BASE, NULL,
			&eth_data),
	OF_DEV_AUXDATA("st,spear320-macb", SPEAR310_MACB0_BASE, NULL,
			&spear310_macb_data[0]),
	OF_DEV_AUXDATA("st,spear320-macb", SPEAR310_MACB1_BASE, NULL,
			&spear310_macb_data[1]),
	OF_DEV_AUXDATA("st,spear320-macb", SPEAR310_MACB2_BASE, NULL,
			&spear310_macb_data[2]),
	OF_DEV_AUXDATA("st,spear320-macb", SPEAR310_MACB3_BASE, NULL,
			&spear310_macb_data[3]),
	OF_DEV_AUXDATA("st,designware-jpeg", SPEAR3XX_ICM1_JPEG_BASE, NULL,
			&jpeg_pdata),
	{}
};

static void __init spear310_dt_init(void)
{
	pl080_plat_data.slave_channels = spear310_dma_info;
	pl080_plat_data.num_slave_channels = ARRAY_SIZE(spear310_dma_info);

	of_platform_populate(NULL, of_default_bus_match_table,
			spear310_auxdata_lookup, NULL);

	spear3xx_macb_setup();
	spear3xx_emi_init(SPEAR310_SOC_EMI_BASE, 6);
}

static const char * const spear310_dt_board_compat[] = {
	"st,spear310",
	"st,spear310-evb",
	NULL,
};

struct map_desc spear310_io_desc[] __initdata = {
	{
		.virtual	= VA_SPEAR310_SOC_CONFIG_BASE,
		.pfn		= __phys_to_pfn(SPEAR310_SOC_CONFIG_BASE),
		.length		= SZ_16M,
		.type		= MT_DEVICE
	},
};

static void __init spear310_map_io(void)
{
	iotable_init(spear310_io_desc, ARRAY_SIZE(spear310_io_desc));
	spear3xx_map_io();
}

DT_MACHINE_START(SPEAR310_DT, "ST SPEAr310 SoC with Flattened Device Tree")
	.map_io		=	spear310_map_io,
	.init_irq	=	spear3xx_dt_init_irq,
	.handle_irq	=	vic_handle_irq,
	.timer		=	&spear3xx_timer,
	.init_machine	=	spear310_dt_init,
	.restart	=	spear_restart,
	.dt_compat	=	spear310_dt_board_compat,
MACHINE_END
