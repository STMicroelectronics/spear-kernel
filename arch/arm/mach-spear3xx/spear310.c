/*
 * arch/arm/mach-spear3xx/spear310.c
 *
 * SPEAr310 machine source file
 *
 * Copyright (C) 2009 ST Microelectronics
 * Viresh Kumar<viresh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/amba/pl08x.h>
#include <linux/amba/serial.h>
#include <linux/mtd/physmap.h>
#include <linux/ptrace.h>
#include <linux/mtd/fsmc.h>
#include <asm/irq.h>
#include <plat/hdlc.h>
#include <plat/pl080.h>
#include <plat/shirq.h>
#include <mach/generic.h>
#include <mach/gpio.h>
#include <mach/hardware.h>

/* pad multiplexing support */

/* devices */
/* Pad multiplexing for emi_cs_0_1_4_5 devices */
static struct pmx_mux_reg pmx_emi_cs_0_1_4_5_mux[] = {
	{
		.mask = PMX_TIMER_3_4_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_emi_cs_0_1_4_5_modes[] = {
	{
		.mux_regs = pmx_emi_cs_0_1_4_5_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_emi_cs_0_1_4_5_mux),
	},
};

struct pmx_dev spear310_pmx_emi_cs_0_1_4_5 = {
	.name = "emi_cs_0_1_4_5",
	.modes = pmx_emi_cs_0_1_4_5_modes,
	.mode_count = ARRAY_SIZE(pmx_emi_cs_0_1_4_5_modes),
};

/* Pad multiplexing for emi_cs_2_3 devices */
static struct pmx_mux_reg pmx_emi_cs_2_3_mux[] = {
	{
		.mask = PMX_TIMER_1_2_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_emi_cs_2_3_modes[] = {
	{
		.mux_regs = pmx_emi_cs_2_3_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_emi_cs_2_3_mux),
	},
};

struct pmx_dev spear310_pmx_emi_cs_2_3 = {
	.name = "emi_cs_2_3",
	.modes = pmx_emi_cs_2_3_modes,
	.mode_count = ARRAY_SIZE(pmx_emi_cs_2_3_modes),
};

/* Pad multiplexing for uart1 device */
static struct pmx_mux_reg pmx_uart1_mux[] = {
	{
		.mask = PMX_FIRDA_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_uart1_modes[] = {
	{
		.mux_regs = pmx_uart1_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_uart1_mux),
	},
};

struct pmx_dev spear310_pmx_uart1 = {
	.name = "uart1",
	.modes = pmx_uart1_modes,
	.mode_count = ARRAY_SIZE(pmx_uart1_modes),
};

/* Pad multiplexing for uart2 device */
static struct pmx_mux_reg pmx_uart2_mux[] = {
	{
		.mask = PMX_TIMER_1_2_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_uart2_modes[] = {
	{
		.mux_regs = pmx_uart2_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_uart2_mux),
	},
};

struct pmx_dev spear310_pmx_uart2 = {
	.name = "uart2",
	.modes = pmx_uart2_modes,
	.mode_count = ARRAY_SIZE(pmx_uart2_modes),
};

/* Pad multiplexing for uart3_4_5 devices */
static struct pmx_mux_reg pmx_uart3_4_5_mux[] = {
	{
		.mask = PMX_UART0_MODEM_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_uart3_4_5_modes[] = {
	{
		.mux_regs = pmx_uart3_4_5_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_uart3_4_5_mux),
	},
};

struct pmx_dev spear310_pmx_uart3_4_5 = {
	.name = "uart3_4_5",
	.modes = pmx_uart3_4_5_modes,
	.mode_count = ARRAY_SIZE(pmx_uart3_4_5_modes),
};

/* Pad multiplexing for fsmc device */
static struct pmx_mux_reg pmx_fsmc_mux[] = {
	{
		.mask = PMX_SSP_CS_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_fsmc_modes[] = {
	{
		.mux_regs = pmx_fsmc_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_fsmc_mux),
	},
};

struct pmx_dev spear310_pmx_fsmc = {
	.name = "fsmc",
	.modes = pmx_fsmc_modes,
	.mode_count = ARRAY_SIZE(pmx_fsmc_modes),
};

/* Pad multiplexing for rs485_0_1 devices */
static struct pmx_mux_reg pmx_rs485_0_1_mux[] = {
	{
		.mask = PMX_MII_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_rs485_0_1_modes[] = {
	{
		.mux_regs = pmx_rs485_0_1_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_rs485_0_1_mux),
	},
};

struct pmx_dev spear310_pmx_rs485_0_1 = {
	.name = "rs485_0_1",
	.modes = pmx_rs485_0_1_modes,
	.mode_count = ARRAY_SIZE(pmx_rs485_0_1_modes),
};

/* Pad multiplexing for tdm0 device */
static struct pmx_mux_reg pmx_tdm0_mux[] = {
	{
		.mask = PMX_MII_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_tdm0_modes[] = {
	{
		.mux_regs = pmx_tdm0_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_tdm0_mux),
	},
};

struct pmx_dev spear310_pmx_tdm0 = {
	.name = "tdm0",
	.modes = pmx_tdm0_modes,
	.mode_count = ARRAY_SIZE(pmx_tdm0_modes),
};

/* pmx driver structure */
static struct pmx_driver pmx_driver;

/* Add spear310 specific devices here */
/* DMAC platform data's slave info */
static struct pl08x_channel_data pl080_slave_channels[] = {
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
static struct amba_pl011_data uart_data[] = {
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

/* uart1 device registeration */
struct amba_device spear310_uart1_device = {
	.dev = {
		.init_name = "uart1",
		.platform_data = &uart_data[0],
	},
	.res = {
		.start = SPEAR310_UART1_BASE,
		.end = SPEAR310_UART1_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	.irq = {SPEAR310_VIRQ_UART1, NO_IRQ},
};

/* uart2 device registeration */
struct amba_device spear310_uart2_device = {
	.dev = {
		.init_name = "uart2",
		.platform_data = &uart_data[1],
	},
	.res = {
		.start = SPEAR310_UART2_BASE,
		.end = SPEAR310_UART2_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	.irq = {SPEAR310_VIRQ_UART2, NO_IRQ},
};

/* uart3 device registeration */
struct amba_device spear310_uart3_device = {
	.dev = {
		.init_name = "uart3",
		.platform_data = &uart_data[2],
	},
	.res = {
		.start = SPEAR310_UART3_BASE,
		.end = SPEAR310_UART3_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	.irq = {SPEAR310_VIRQ_UART3, NO_IRQ},
};

/* uart4 device registeration */
struct amba_device spear310_uart4_device = {
	.dev = {
		.init_name = "uart4",
		.platform_data = &uart_data[3],
	},
	.res = {
		.start = SPEAR310_UART4_BASE,
		.end = SPEAR310_UART4_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	.irq = {SPEAR310_VIRQ_UART4, NO_IRQ},
};

/* uart5 device registeration */
struct amba_device spear310_uart5_device = {
	.dev = {
		.init_name = "uart5",
		.platform_data = &uart_data[4],
	},
	.res = {
		.start = SPEAR310_UART5_BASE,
		.end = SPEAR310_UART5_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	.irq = {SPEAR310_VIRQ_UART5, NO_IRQ},
};

/* Ethernet Device registeration */
static u64 macb1_dmamask = ~0;
static struct resource macb1_resources[] = {
	{
		.start = SPEAR310_MACB1_BASE,
		.end = SPEAR310_MACB1_BASE + SZ_8K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR310_VIRQ_SMII0,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear310_eth_macb1_device = {
	.name = "macb",
	.id = 0,
	.dev = {
		.dma_mask = &macb1_dmamask,
		.coherent_dma_mask = ~0,
		},
	.resource = macb1_resources,
	.num_resources = ARRAY_SIZE(macb1_resources),
};

static u64 macb2_dmamask = ~0;
static struct resource macb2_resources[] = {
	{
		.start = SPEAR310_MACB2_BASE,
		.end = SPEAR310_MACB2_BASE + SZ_8K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR310_VIRQ_SMII1,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear310_eth_macb2_device = {
	.name = "macb",
	.id = 1,
	.dev = {
		.dma_mask = &macb2_dmamask,
		.coherent_dma_mask = ~0,
		},
	.resource = macb2_resources,
	.num_resources = ARRAY_SIZE(macb2_resources),
};

static u64 macb3_dmamask = ~0;
static struct resource macb3_resources[] = {
	{
		.start = SPEAR310_MACB3_BASE,
		.end = SPEAR310_MACB3_BASE + SZ_8K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR310_VIRQ_SMII2,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear310_eth_macb3_device = {
	.name = "macb",
	.id = 2,
	.dev = {
		.dma_mask = &macb3_dmamask,
		.coherent_dma_mask = ~0,
		},
	.resource = macb3_resources,
	.num_resources = ARRAY_SIZE(macb3_resources),
};

static u64 macb4_dmamask = ~0;
static struct resource macb4_resources[] = {
	{
		.start = SPEAR310_MACB4_BASE,
		.end = SPEAR310_MACB4_BASE + SZ_8K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR310_VIRQ_SMII3,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear310_eth_macb4_device = {
	.name = "macb",
	.id = 3,
	.dev = {
		.dma_mask = &macb4_dmamask,
		.coherent_dma_mask = ~0,
		},
	.resource = macb4_resources,
	.num_resources = ARRAY_SIZE(macb4_resources),
};

/* nand device registeration */
static struct fsmc_nand_platform_data nand_platform_data = {
	.mode = USE_WORD_ACCESS,
};

static struct resource nand_resources[] = {
	{
		.name = "nand_data",
		.start = SPEAR310_NAND_BASE,
		.end = SPEAR310_NAND_BASE + SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.name = "fsmc_regs",
		.start = SPEAR310_FSMC_BASE,
		.end = SPEAR310_FSMC_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
};

struct platform_device spear310_nand_device = {
	.name = "fsmc-nand",
	.id = -1,
	.resource = nand_resources,
	.num_resources = ARRAY_SIZE(nand_resources),
	.dev.platform_data = &nand_platform_data,
};

/* plgpio device registeration */
/*
 * pin to offset and offset to pin converter functions
 *
 * In spear310 there is inconsistency among bit positions in plgpio regiseters,
 * for different plgpio pins. For example: for pin 27, bit offset is 23, pin
 * 28-33 are not supported, pin 95 has offset bit 95, bit 100 has offset bit 1
 */
static int spear300_p2o(int pin)
{
	int offset = pin;

	if (pin <= 27)
		offset += 4;
	else if (pin <= 33)
		offset = -1;
	else if (pin <= 97)
		offset -= 2;
	else if (pin <= 101)
		offset = 101 - pin;
	else
		offset = -1;

	return offset;
}

int spear300_o2p(int offset)
{
	if (offset <= 3)
		return 101 - offset;
	else if (offset <= 31)
		return offset - 4;
	else
		return offset + 2;
}

/* emi nor flash device registeration */
static struct physmap_flash_data emi_norflash_data;
struct platform_device spear310_emi_nor_device = {
	.name	= "physmap-flash",
	.id	= -1,
	.dev.platform_data = &emi_norflash_data,
};

static struct plgpio_platform_data plgpio_plat_data = {
	.gpio_base = 8,
	.irq_base = SPEAR3XX_PLGPIO_INT_BASE,
	.gpio_count = SPEAR3XX_PLGPIO_COUNT,
	.p2o = spear300_p2o,
	.o2p = spear300_o2p,
	/* list of registers with inconsistency */
	.p2o_regs = PTO_RDATA_REG | PTO_WDATA_REG | PTO_DIR_REG |
		PTO_IE_REG | PTO_RDATA_REG | PTO_MIS_REG,
	.regs = {
		.enb = SPEAR310_PLGPIO_ENB_OFF,
		.wdata = SPEAR310_PLGPIO_WDATA_OFF,
		.dir = SPEAR310_PLGPIO_DIR_OFF,
		.ie = SPEAR310_PLGPIO_IE_OFF,
		.rdata = SPEAR310_PLGPIO_RDATA_OFF,
		.mis = SPEAR310_PLGPIO_MIS_OFF,
		.eit = -1,
	},
};

static struct resource plgpio_resources[] = {
	{
		.start = SPEAR310_SOC_CONFIG_BASE,
		.end = SPEAR310_SOC_CONFIG_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR310_VIRQ_PLGPIO,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear310_plgpio_device = {
	.name = "plgpio",
	.id = -1,
	.dev = {
		.platform_data = &plgpio_plat_data,
	},
	.num_resources = ARRAY_SIZE(plgpio_resources),
	.resource = plgpio_resources,
};

static struct tdm_hdlc_platform_data tdm_hdlc_plat_data = {
	.ip_type = SPEAR310_TDM_HDLC,
	.nr_channel = 2,
	.nr_timeslot = 128,
};

static struct resource tdm_hdlc_resources[] = {
	{
		.start = SPEAR310_HDLC_BASE,
		.end = SPEAR310_HDLC_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR310_VIRQ_TDM_HDLC,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear310_tdm_hdlc_device = {
	.name = "tdm_hdlc",
	.id = -1,
	.dev = {
		.platform_data = &tdm_hdlc_plat_data,
		.coherent_dma_mask = ~0,
	},
	.num_resources = ARRAY_SIZE(tdm_hdlc_resources),
	.resource = tdm_hdlc_resources,
};

static struct rs485_hdlc_platform_data rs485_0_plat_data = {
	.tx_falling_edge = 1,
	.rx_rising_edge = 1,
	.cts_enable = 1,
	.cts_delay = 24,
};

static struct rs485_hdlc_platform_data rs485_1_plat_data = {
	.tx_falling_edge = 1,
	.rx_rising_edge = 1,
	.cts_enable = 1,
	.cts_delay = 24,
};

static struct resource rs485_0_resources[] = {
	{
		.start = SPEAR310_RS485_0_BASE,
		.end = SPEAR310_RS485_0_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR310_VIRQ_RS485_0,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource rs485_1_resources[] = {
	{
		.start = SPEAR310_RS485_1_BASE,
		.end = SPEAR310_RS485_1_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR310_VIRQ_RS485_1,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear310_rs485_0_device = {
	.name = "rs485_hdlc",
	.id = 0,
	.dev = {
		.platform_data = &rs485_0_plat_data,
		.coherent_dma_mask = ~0,
	},
	.num_resources = ARRAY_SIZE(rs485_0_resources),
	.resource = rs485_0_resources,
};

struct platform_device spear310_rs485_1_device = {
	.name = "rs485_hdlc",
	.id = 1,
	.dev = {
		.platform_data = &rs485_1_plat_data,
		.coherent_dma_mask = ~0,
	},
	.num_resources = ARRAY_SIZE(rs485_1_resources),
	.resource = rs485_1_resources,
};

/* spear3xx shared irq */
static struct shirq_dev_config shirq_ras1_config[] = {
	{
		.virq = SPEAR310_VIRQ_SMII0,
		.status_mask = SPEAR310_SMII0_IRQ_MASK,
	}, {
		.virq = SPEAR310_VIRQ_SMII1,
		.status_mask = SPEAR310_SMII1_IRQ_MASK,
	}, {
		.virq = SPEAR310_VIRQ_SMII2,
		.status_mask = SPEAR310_SMII2_IRQ_MASK,
	}, {
		.virq = SPEAR310_VIRQ_SMII3,
		.status_mask = SPEAR310_SMII3_IRQ_MASK,
	}, {
		.virq = SPEAR310_VIRQ_WAKEUP_SMII0,
		.status_mask = SPEAR310_WAKEUP_SMII0_IRQ_MASK,
	}, {
		.virq = SPEAR310_VIRQ_WAKEUP_SMII1,
		.status_mask = SPEAR310_WAKEUP_SMII1_IRQ_MASK,
	}, {
		.virq = SPEAR310_VIRQ_WAKEUP_SMII2,
		.status_mask = SPEAR310_WAKEUP_SMII2_IRQ_MASK,
	}, {
		.virq = SPEAR310_VIRQ_WAKEUP_SMII3,
		.status_mask = SPEAR310_WAKEUP_SMII3_IRQ_MASK,
	},
};

static struct spear_shirq shirq_ras1 = {
	.irq = SPEAR3XX_IRQ_GEN_RAS_1,
	.dev_config = shirq_ras1_config,
	.dev_count = ARRAY_SIZE(shirq_ras1_config),
	.regs = {
		.base = IOMEM(VA_SPEAR310_SOC_CONFIG_BASE),
		.enb_reg = -1,
		.status_reg = SPEAR310_INT_STS_MASK_REG,
		.status_reg_mask = SPEAR310_SHIRQ_RAS1_MASK,
		.clear_reg = -1,
	},
};

static struct shirq_dev_config shirq_ras2_config[] = {
	{
		.virq = SPEAR310_VIRQ_UART1,
		.status_mask = SPEAR310_UART1_IRQ_MASK,
	}, {
		.virq = SPEAR310_VIRQ_UART2,
		.status_mask = SPEAR310_UART2_IRQ_MASK,
	}, {
		.virq = SPEAR310_VIRQ_UART3,
		.status_mask = SPEAR310_UART3_IRQ_MASK,
	}, {
		.virq = SPEAR310_VIRQ_UART4,
		.status_mask = SPEAR310_UART4_IRQ_MASK,
	}, {
		.virq = SPEAR310_VIRQ_UART5,
		.status_mask = SPEAR310_UART5_IRQ_MASK,
	},
};

static struct spear_shirq shirq_ras2 = {
	.irq = SPEAR3XX_IRQ_GEN_RAS_2,
	.dev_config = shirq_ras2_config,
	.dev_count = ARRAY_SIZE(shirq_ras2_config),
	.regs = {
		.base = IOMEM(VA_SPEAR310_SOC_CONFIG_BASE),
		.enb_reg = -1,
		.status_reg = SPEAR310_INT_STS_MASK_REG,
		.status_reg_mask = SPEAR310_SHIRQ_RAS2_MASK,
		.clear_reg = -1,
	},
};

static struct shirq_dev_config shirq_ras3_config[] = {
	{
		.virq = SPEAR310_VIRQ_EMI,
		.status_mask = SPEAR310_EMI_IRQ_MASK,
	},
};

static struct spear_shirq shirq_ras3 = {
	.irq = SPEAR3XX_IRQ_GEN_RAS_3,
	.dev_config = shirq_ras3_config,
	.dev_count = ARRAY_SIZE(shirq_ras3_config),
	.regs = {
		.base = IOMEM(VA_SPEAR310_SOC_CONFIG_BASE),
		.enb_reg = -1,
		.status_reg = SPEAR310_INT_STS_MASK_REG,
		.status_reg_mask = SPEAR310_SHIRQ_RAS3_MASK,
		.clear_reg = -1,
	},
};

static struct shirq_dev_config shirq_intrcomm_ras_config[] = {
	{
		.virq = SPEAR310_VIRQ_TDM_HDLC,
		.status_mask = SPEAR310_TDM_HDLC_IRQ_MASK,
	}, {
		.virq = SPEAR310_VIRQ_RS485_0,
		.status_mask = SPEAR310_RS485_0_IRQ_MASK,
	}, {
		.virq = SPEAR310_VIRQ_RS485_1,
		.status_mask = SPEAR310_RS485_1_IRQ_MASK,
	},
};

static struct spear_shirq shirq_intrcomm_ras = {
	.irq = SPEAR3XX_IRQ_INTRCOMM_RAS_ARM,
	.dev_config = shirq_intrcomm_ras_config,
	.dev_count = ARRAY_SIZE(shirq_intrcomm_ras_config),
	.regs = {
		.base = IOMEM(VA_SPEAR310_SOC_CONFIG_BASE),
		.enb_reg = -1,
		.status_reg = SPEAR310_INT_STS_MASK_REG,
		.status_reg_mask = SPEAR310_SHIRQ_INTRCOMM_RAS_MASK,
		.clear_reg = -1,
	},
};

/* Following will create 310 specific static virtual/physical mappings */
static struct map_desc spear310_io_desc[] __initdata = {
	{
		.virtual	= VA_SPEAR310_SOC_CONFIG_BASE,
		.pfn		= __phys_to_pfn(SPEAR310_SOC_CONFIG_BASE),
		.length		= SZ_4K,
		.type		= MT_DEVICE
	},
};

/* This will create static memory mapping for selected devices */
void __init spear310_map_io(void)
{
	iotable_init(spear310_io_desc, ARRAY_SIZE(spear310_io_desc));
	spear3xx_map_io();
}

/* spear310 routines */
void __init spear310_init(struct pmx_mode *pmx_mode, struct pmx_dev **pmx_devs,
		u8 pmx_dev_count)
{
	int ret = 0;

	/* call spear3xx family common init function */
	spear3xx_init();

	/* shared irq registration */
	/* shirq 1 */
	ret = spear_shirq_register(&shirq_ras1);
	if (ret)
		printk(KERN_ERR "Error registering Shared IRQ 1\n");

	/* shirq 2 */
	ret = spear_shirq_register(&shirq_ras2);
	if (ret)
		printk(KERN_ERR "Error registering Shared IRQ 2\n");

	/* shirq 3 */
	ret = spear_shirq_register(&shirq_ras3);
	if (ret)
		printk(KERN_ERR "Error registering Shared IRQ 3\n");

	/* shirq 4 */
	ret = spear_shirq_register(&shirq_intrcomm_ras);
	if (ret)
		printk(KERN_ERR "Error registering Shared IRQ 4\n");

	/* pmx initialization */
	pmx_driver.mode = pmx_mode;
	pmx_driver.devs = pmx_devs;
	pmx_driver.devs_count = pmx_dev_count;

	/* This fixes addresses of all pmx devices for spear310 */
	spear3xx_pmx_init_addr(&pmx_driver, SPEAR310_PAD_MUX_CONFIG_REG);
	ret = pmx_register(&pmx_driver);
	if (ret)
		pr_err("padmux: registeration failed. err no: %d\n", ret);

	/* Set DMAC platform data's slave info */
	pl080_set_slaveinfo(&spear3xx_dma_device, pl080_slave_channels,
			ARRAY_SIZE(pl080_slave_channels));
}
