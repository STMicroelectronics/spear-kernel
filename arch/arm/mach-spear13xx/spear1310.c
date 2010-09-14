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

#include <linux/clk.h>
#include <linux/mtd/physmap.h>
#include <linux/ptrace.h>
#include <linux/stmmac.h>
#include <asm/irq.h>
#include <asm/delay.h>
#include <mach/generic.h>
#include <mach/hardware.h>
#include <mach/misc_regs.h>
#include <plat/clock.h>
#include <plat/hdlc.h>

/* pmx driver structure */
static struct pmx_driver pmx_driver;

/* Pad multiplexing for uart1_modem device */
static struct pmx_mux_reg pmx_uart1_modem_mux[] = {
	{
		.address = PAD_MUX_CONFIG_REG_0,
		.mask = PMX_I2S1_MASK | PMX_SSP_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_uart1_modem_modes[] = {
	{
		.mux_regs = pmx_uart1_modem_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_uart1_modem_mux),
	},
};

struct pmx_dev pmx_uart1_modem = {
	.name = "uart1_modem",
	.modes = pmx_uart1_modem_modes,
	.mode_count = ARRAY_SIZE(pmx_uart1_modem_modes),
};

/* Pad multiplexing for uart1 device */
static struct pmx_mux_reg pmx_uart1_mux[] = {
	{
		.address = PAD_MUX_CONFIG_REG_0,
		.mask = PMX_SSP_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_uart1_modes[] = {
	{
		.mux_regs = pmx_uart1_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_uart1_mux),
	},
};

struct pmx_dev pmx_uart_1 = {
	.name = "uart1",
	.modes = pmx_uart1_modes,
	.mode_count = ARRAY_SIZE(pmx_uart1_modes),
};

/* Pad multiplexing for uart2 device */
static struct pmx_mux_reg pmx_uart2_mux[] = {
	{
		.address = PAD_MUX_CONFIG_REG_0,
		.mask = PMX_SSP_MASK | PMX_CLCD1_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_uart2_modes[] = {
	{
		.mux_regs = pmx_uart2_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_uart2_mux),
	},
};

struct pmx_dev pmx_uart_2 = {
	.name = "uart2",
	.modes = pmx_uart2_modes,
	.mode_count = ARRAY_SIZE(pmx_uart2_modes),
};

/* Pad multiplexing for uart_3_4_5 device */
static struct pmx_mux_reg pmx_uart_3_4_5_mux[] = {
	{
		.address = PAD_MUX_CONFIG_REG_0,
		.mask = PMX_CLCD1_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_uart_3_4_5_modes[] = {
	{
		.mux_regs = pmx_uart_3_4_5_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_uart_3_4_5_mux),
	},
};

struct pmx_dev pmx_uart_3_4_5 = {
	.name = "uart_3_4_5",
	.modes = pmx_uart_3_4_5_modes,
	.mode_count = ARRAY_SIZE(pmx_uart_3_4_5_modes),
};

/* Pad multiplexing for rs485_hdlc_1_2 device */
static struct pmx_mux_reg pmx_rs485_hdlc_1_2_mux[] = {
	{
		.address = PAD_MUX_CONFIG_REG_0,
		.mask = PMX_CLCD1_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_rs485_hdlc_1_2_modes[] = {
	{
		.mux_regs = pmx_rs485_hdlc_1_2_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_rs485_hdlc_1_2_mux),
	},
};

struct pmx_dev pmx_rs485_hdlc_1_2 = {
	.name = "rs485_hdlc_1_2",
	.modes = pmx_rs485_hdlc_1_2_modes,
	.mode_count = ARRAY_SIZE(pmx_rs485_hdlc_1_2_modes),
};

/* Pad multiplexing for tdm_hdlc_1_2 device */
static struct pmx_mux_reg pmx_tdm_hdlc_1_2_mux[] = {
	{
		.address = PAD_MUX_CONFIG_REG_0,
		.mask = PMX_CLCD1_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_tdm_hdlc_1_2_modes[] = {
	{
		.mux_regs = pmx_tdm_hdlc_1_2_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_tdm_hdlc_1_2_mux),
	},
};

struct pmx_dev pmx_tdm_hdlc_1_2 = {
	.name = "tdm_hdlc_1_2",
	.modes = pmx_tdm_hdlc_1_2_modes,
	.mode_count = ARRAY_SIZE(pmx_tdm_hdlc_1_2_modes),
};

/* Pad multiplexing for fsmc32bit device */
static struct pmx_mux_reg pmx_fsmc32bit_mux[] = {
	{
		.address = PAD_MUX_CONFIG_REG_0,
		.mask = PMX_EGPIO_0_GRP_MASK | PMX_SMI_MASK | \
			PMX_NAND16BIT4DEV_0_MASK | PMX_CLCD1_MASK,
		.value = 0,
	}, {
		.address = PAD_MUX_CONFIG_REG_1,
		.mask = PMX_KEYBOARD_6X6_MASK | PMX_NAND16BIT4DEV_1_MASK,
		.value = 0,
	}, {
		.address = SPEAR13XX_PCM_CFG_BASE,
		.mask = PMX_EGPIO7_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_fsmc32bit_modes[] = {
	{
		.mux_regs = pmx_fsmc32bit_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_fsmc32bit_mux),
	},
};

struct pmx_dev pmx_fsmc32bit_4_chips = {
	.name = "fsmc32bit",
	.modes = pmx_fsmc32bit_modes,
	.mode_count = ARRAY_SIZE(pmx_fsmc32bit_modes),
};

/* Pad multiplexing for fsmc16bit device */
static struct pmx_mux_reg pmx_fsmc16bit_mux[] = {
	{
		.address = PAD_MUX_CONFIG_REG_0,
		.mask = PMX_NAND16BIT4DEV_0_MASK,
		.value = 0,
	}, {
		.address = PAD_MUX_CONFIG_REG_1,
		.mask = PMX_KEYBOARD_6X6_MASK | PMX_NAND16BIT4DEV_1_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_fsmc16bit_modes[] = {
	{
		.mux_regs = pmx_fsmc16bit_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_fsmc16bit_mux),
	},
};

struct pmx_dev pmx_fsmc16bit_4_chips = {
	.name = "fsmc16bit",
	.modes = pmx_fsmc16bit_modes,
	.mode_count = ARRAY_SIZE(pmx_fsmc16bit_modes),
};

/* Pad multiplexing for gmii1 device */
static struct pmx_mux_reg pmx_gmii1_mux[] = {
	{
		.address = PAD_MUX_CONFIG_REG_0,
		.mask = PMX_GMII_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_gmii1_modes[] = {
	{
		.mux_regs = pmx_gmii1_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_gmii1_mux),
	},
};

struct pmx_dev pmx_gmii1 = {
	.name = "gmii1",
	.modes = pmx_gmii1_modes,
	.mode_count = ARRAY_SIZE(pmx_gmii1_modes),
};

/* Pad multiplexing for rgmii device */
static struct pmx_mux_reg pmx_rgmii_mux[] = {
	{
		.address = PAD_MUX_CONFIG_REG_0,
		.mask = PMX_GMII_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_rgmii_modes[] = {
	{
		.mux_regs = pmx_rgmii_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_rgmii_mux),
	},
};

struct pmx_dev pmx_rgmii = {
	.name = "rgmii",
	.modes = pmx_rgmii_modes,
	.mode_count = ARRAY_SIZE(pmx_rgmii_modes),
};

/* Pad multiplexing for i2c1 device */
static struct pmx_mux_reg pmx_i2c1_mux[] = {
	{
		.address = PAD_MUX_CONFIG_REG_1,
		.mask = PMX_SMINCS2_MASK | PMX_SMINCS3_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_i2c1_modes[] = {
	{
		.mux_regs = pmx_i2c1_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_i2c1_mux),
	},
};

struct pmx_dev pmx_i2c1 = {
	.name = "i2c1",
	.modes = pmx_i2c1_modes,
	.mode_count = ARRAY_SIZE(pmx_i2c1_modes),
};

/* Pad multiplexing for smii_0_1_2 device */
static struct pmx_mux_reg pmx_smii_0_1_2_mux[] = {
	{
		.address = PAD_MUX_CONFIG_REG_1,
		.mask = PMX_CLCD2_MASK | PMX_KBD_ROWCOL68_MASK | \
			PMX_EGPIO_1_GRP_MASK | PMX_GPT0_TMR1_MASK | \
			PMX_GPT0_TMR2_MASK | PMX_GPT1_TMR1_MASK | \
			PMX_GPT1_TMR2_MASK,
		.value = 0,
	}, {
		.address = SPEAR1310_FUNC_CNTL_0,
		.mask = PMX_SMII_MASK,
		.value = PMX_SMII_MASK,
	},
};

static struct pmx_dev_mode pmx_smii_0_1_2_modes[] = {
	{
		.mux_regs = pmx_smii_0_1_2_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_smii_0_1_2_mux),
	},
};

struct pmx_dev pmx_smii_0_1_2 = {
	.name = "smii_0_1_2",
	.modes = pmx_smii_0_1_2_modes,
	.mode_count = ARRAY_SIZE(pmx_smii_0_1_2_modes),
};

/* Pad multiplexing for pci1 device */
static struct pmx_mux_reg pmx_pci1_mux[] = {
	{
		.address = PAD_MUX_CONFIG_REG_1,
		.mask = PMX_CLCD2_MASK | PMX_KBD_ROWCOL68_MASK | \
			PMX_EGPIO_1_GRP_MASK | PMX_GPT0_TMR1_MASK | \
			PMX_GPT0_TMR2_MASK | PMX_GPT1_TMR1_MASK | \
			PMX_GPT1_TMR2_MASK,
		.value = 0,
	}, {
		.address = SPEAR1310_FUNC_CNTL_0,
		.mask = PMX_SMII_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_pci1_modes[] = {
	{
		.mux_regs = pmx_pci1_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_pci1_mux),
	},
};

struct pmx_dev pmx_pci1 = {
	.name = "pci1",
	.modes = pmx_pci1_modes,
	.mode_count = ARRAY_SIZE(pmx_pci1_modes),
};

/* Pad multiplexing for can device */
static struct pmx_mux_reg pmx_can_mux[] = {
	{
		.address = PAD_MUX_CONFIG_REG_1,
		.mask = PMX_I2S2_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_can_modes[] = {
	{
		.mux_regs = pmx_can_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_can_mux),
	},
};

struct pmx_dev pmx_can = {
	.name = "can",
	.modes = pmx_can_modes,
	.mode_count = ARRAY_SIZE(pmx_can_modes),
};

/* Add spear1310 specific devices here */
/* uart1 device registeration */
struct amba_device spear1310_uart1_device = {
	.dev = {
		.init_name = "uart1",
	},
	.res = {
		.start = SPEAR1310_UART1_BASE,
		.end = SPEAR1310_UART1_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	.irq = {IRQ_UART1, NO_IRQ},
};

/* uart2 device registeration */
struct amba_device spear1310_uart2_device = {
	.dev = {
		.init_name = "uart2",
	},
	.res = {
		.start = SPEAR1310_UART2_BASE,
		.end = SPEAR1310_UART2_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	.irq = {IRQ_UART2, NO_IRQ},
};

/* uart3 device registeration */
struct amba_device spear1310_uart3_device = {
	.dev = {
		.init_name = "uart3",
	},
	.res = {
		.start = SPEAR1310_UART3_BASE,
		.end = SPEAR1310_UART3_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	.irq = {IRQ_UART3, NO_IRQ},
};

/* uart4 device registeration */
struct amba_device spear1310_uart4_device = {
	.dev = {
		.init_name = "uart4",
	},
	.res = {
		.start = SPEAR1310_UART4_BASE,
		.end = SPEAR1310_UART4_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	.irq = {IRQ_UART4, NO_IRQ},
};

/* uart5 device registeration */
struct amba_device spear1310_uart5_device = {
	.dev = {
		.init_name = "uart5",
	},
	.res = {
		.start = SPEAR1310_UART5_BASE,
		.end = SPEAR1310_UART5_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	.irq = {IRQ_UART5, NO_IRQ},
};

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

struct platform_device spear1310_can0_device = {
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

struct platform_device spear1310_can1_device = {
	.name = "spear_can",
	.id = 1,
	.num_resources = ARRAY_SIZE(can1_resources),
	.resource = can1_resources,
};

/* Ethernet GETH-1 device registeration */
static struct plat_stmmacenet_data ether1_platform_data = {
	.bus_id = 1,
	.has_gmac = 1,
	.enh_desc = 1,
	.tx_csum = 0,
	.pbl = 8,
	.csum_off_engine = STMAC_TYPE_0,
};

static struct resource eth1_resources[] = {
	[0] = {
		.start = SPEAR1310_GETH1_BASE,
		.end = SPEAR1310_GETH1_BASE + SZ_32K - 1,
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

struct platform_device spear1310_eth1_device = {
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

/* Ethernet GETH-2 device registeration */
static struct plat_stmmacenet_data ether2_platform_data = {
	.bus_id = 2,
	.has_gmac = 1,
	.enh_desc = 1,
	.tx_csum = 0,
	.pbl = 8,
	.csum_off_engine = STMAC_TYPE_0,
};

static struct resource eth2_resources[] = {
	[0] = {
		.start = SPEAR1310_GETH2_BASE,
		.end = SPEAR1310_GETH2_BASE + SZ_32K - 1,
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

struct platform_device spear1310_eth2_device = {
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

/* Ethernet GETH-3 device registeration */
static struct plat_stmmacenet_data ether3_platform_data = {
	.bus_id = 3,
	.has_gmac = 1,
	.enh_desc = 1,
	.tx_csum = 0,
	.pbl = 8,
	.csum_off_engine = STMAC_TYPE_0,
};

static struct resource eth3_resources[] = {
	[0] = {
		.start = SPEAR1310_GETH3_BASE,
		.end = SPEAR1310_GETH3_BASE + SZ_32K - 1,
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

struct platform_device spear1310_eth3_device = {
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

/* Ethernet GETH-4 device registeration */
static struct plat_stmmacenet_data ether4_platform_data = {
	.bus_id = 4,
	.has_gmac = 1,
	.enh_desc = 1,
	.tx_csum = 0,
	.pbl = 8,
	.csum_off_engine = STMAC_TYPE_0,
};

static struct resource eth4_resources[] = {
	[0] = {
		.start = SPEAR1310_GETH4_BASE,
		.end = SPEAR1310_GETH4_BASE + SZ_32K - 1,
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

struct platform_device spear1310_eth4_device = {
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

/* i2c1 device registeration */
static struct resource i2c1_resources[] = {
	{
		.start = SPEAR1310_I2C1_BASE,
		.end = SPEAR1310_I2C1_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = IRQ_I2C_CNTR,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear1310_i2c1_device = {
	.name = "i2c_designware",
	.id = 1,
	.dev = {
		.coherent_dma_mask = ~0,
	},
	.num_resources = ARRAY_SIZE(i2c1_resources),
	.resource = i2c1_resources,
};

static struct tdm_hdlc_platform_data tdm_hdlc_0_plat_data = {
	.ip_type = SPEAR1310_TDM_HDLC,
	.nr_channel = 2,
	.nr_timeslot = 128,
};

static struct resource tdm_hdlc_0_resources[] = {
	{
		.start = SPEAR1310_TDM_E1_0_BASE,
		.end   = SPEAR1310_TDM_E1_0_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = IRQ_TDM0,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear1310_tdm_hdlc_0_device = {
	.name = "tdm_hdlc",
	.id = 0,
	.dev = {
		.platform_data = &tdm_hdlc_0_plat_data,
		.coherent_dma_mask = ~0,
	},
	.num_resources = ARRAY_SIZE(tdm_hdlc_0_resources),
	.resource = tdm_hdlc_0_resources,
};

static struct tdm_hdlc_platform_data tdm_hdlc_1_plat_data = {
	.ip_type = SPEAR1310_TDM_HDLC,
	.nr_channel = 2,
	.nr_timeslot = 128,
};

static struct resource tdm_hdlc_1_resources[] = {
	{
		.start = SPEAR1310_TDM_E1_1_BASE,
		.end   = SPEAR1310_TDM_E1_1_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = IRQ_TDM1,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear1310_tdm_hdlc_1_device = {
	.name = "tdm_hdlc",
	.id = 1,
	.dev = {
		.platform_data = &tdm_hdlc_1_plat_data,
		.coherent_dma_mask = ~0,
	},
	.num_resources = ARRAY_SIZE(tdm_hdlc_1_resources),
	.resource = tdm_hdlc_1_resources,
};

static struct rs485_hdlc_platform_data rs485_0_plat_data = {
	.tx_falling_edge = 1,
	.rx_rising_edge = 1,
	.cts_enable = 1,
	.cts_delay = 50,
};

static struct rs485_hdlc_platform_data rs485_1_plat_data = {
	.tx_falling_edge = 1,
	.rx_rising_edge = 1,
	.cts_enable = 1,
	.cts_delay = 50,
};

static struct resource rs485_0_resources[] = {
	{
		.start = SPEAR1310_RS485_0_BASE,
		.end   = SPEAR1310_RS485_0_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = IRQ_RS4850,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource rs485_1_resources[] = {
	{
		.start = SPEAR1310_RS485_1_BASE,
		.end   = SPEAR1310_RS485_1_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = IRQ_RS4851,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear1310_rs485_0_device = {
	.name = "rs485_hdlc",
	.id = 0,
	.dev = {
		.platform_data = &rs485_0_plat_data,
		.coherent_dma_mask = ~0,
	},
	.num_resources = ARRAY_SIZE(rs485_0_resources),
	.resource = rs485_0_resources,
};

struct platform_device spear1310_rs485_1_device = {
	.name = "rs485_hdlc",
	.id = 1,
	.dev = {
		.platform_data = &rs485_1_plat_data,
		.coherent_dma_mask = ~0,
	},
	.num_resources = ARRAY_SIZE(rs485_1_resources),
	.resource = rs485_1_resources,
};

/* fsmc nor flash device registeration */
static struct physmap_flash_data ras_fsmc_norflash_data;

static struct resource ras_fsmc_nor_resources[] = {
	{
		.start	= SPEAR1310_FSMC1_CS3_BASE,
		.end	= SPEAR1310_FSMC1_CS3_BASE + SZ_64M - 1,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device spear1310_ras_fsmc_nor_device = {
	.name	= "physmap-flash",
	.id	= -1,
	.resource = ras_fsmc_nor_resources,
	.num_resources = ARRAY_SIZE(ras_fsmc_nor_resources),
	.dev.platform_data = &ras_fsmc_norflash_data,
};

static void tdm_hdlc_setup(void)
{
	unsigned long val;
	struct clk *pll3_clk;

	/* get pll3 clk */
	pll3_clk = clk_get(NULL, "pll3_clk");

	/* set pll3 clk to 500M */
	clk_set_rate(pll3_clk, 500000000);

	/* use pll3 source for ras_clk_synt1 */
	val = readl(PLL_CFG);
	val &= ~0x18000000;
	val |= 0x10000000;
	writel(val, PLL_CFG);

	writel(0x80001000, RAS_CLK_SYNT1);	/* generate 250MHz clock */

	udelay(2000);

	/* enable proper clock going to RAS */
	val = readl(RAS_CLK_ENB);
	val |= (1 << SYN1_CLK_ENB);
	writel(val, RAS_CLK_ENB);

	val = readl(RAS_SW_RST);
	val &= ~(1 << SYN1_CLK_ENB);
	writel(val, RAS_SW_RST);

	val = readl(IO_ADDRESS(SPEAR1310_RAS_CTRL_REG0));
	val |= 0x02000000;
	writel(val, IO_ADDRESS(SPEAR1310_RAS_CTRL_REG0));
}

void __init spear1310_init(struct pmx_mode *pmx_mode, struct pmx_dev **pmx_devs,
		u8 pmx_dev_count)
{
	int ret;

	/* call spear13xx family common init function */
	spear13xx_init();

	tdm_hdlc_setup();

	/* pmx initialization */
	pmx_driver.mode = pmx_mode;
	pmx_driver.devs = pmx_devs;
	pmx_driver.devs_count = pmx_dev_count;

	ret = pmx_register(&pmx_driver);
	if (ret)
		pr_err("padmux: registeration failed. err no: %d\n", ret);
}
