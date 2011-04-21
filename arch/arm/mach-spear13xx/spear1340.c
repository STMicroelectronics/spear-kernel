/*
 * arch/arm/mach-spear13xx/spear1340.c
 *
 * SPEAr1340 machine source file
 *
 * Copyright (C) 2011 ST Microelectronics
 * Viresh Kumar <viresh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <asm/irq.h>
#include <linux/amba/serial.h>
#include <linux/dw_dmac.h>
#include <mach/dma.h>
#include <mach/generic.h>
#include <mach/hardware.h>
#include <mach/spear1340_misc_regs.h>

/* pmx driver structure */
static struct pmx_driver pmx_driver;

/* Pad multiplexing for fsmc_16bit device */
static struct pmx_mux_reg pmx_fsmc_16bit_mux[] = {
	{
		.address = SPEAR1340_PAD_MUX_CONFIG_REG_0,
		.mask = SPEAR1340_PMX_KBD_ROW_COL_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_fsmc_16bit_modes[] = {
	{
		.mux_regs = pmx_fsmc_16bit_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_fsmc_16bit_mux),
	},
};

struct pmx_dev spear1340_pmx_fsmc_16bit = {
	.name = "fsmc_16bit",
	.modes = pmx_fsmc_16bit_modes,
	.mode_count = ARRAY_SIZE(pmx_fsmc_16bit_modes),
};

/* pad multiplexing for keyboard rows-cols device */
static struct pmx_mux_reg pmx_keyboard_row_col_mux[] = {
	{
		.address = SPEAR1340_PAD_MUX_CONFIG_REG_0,
		.mask = SPEAR1340_PMX_KBD_ROW_COL_MASK,
		.value = SPEAR1340_PMX_KBD_ROW_COL_MASK,
	},
};

static struct pmx_dev_mode pmx_keyboard_row_col_modes[] = {
	{
		.mux_regs = pmx_keyboard_row_col_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_keyboard_row_col_mux),
	},
};

struct pmx_dev spear1340_pmx_keyboard_row_col= {
	.name = "keyboard_row_col",
	.modes = pmx_keyboard_row_col_modes,
	.mode_count = ARRAY_SIZE(pmx_keyboard_row_col_modes),
};

/* pad multiplexing for keyboard col5 device */
static struct pmx_mux_reg pmx_keyboard_col5_mux[] = {
	{
		.address = SPEAR1340_PAD_MUX_CONFIG_REG_0,
		.mask = SPEAR1340_PMX_KBD_COL5_MASK,
		.value = SPEAR1340_PMX_KBD_COL5_MASK,
	},
};

static struct pmx_dev_mode pmx_keyboard_col5_modes[] = {
	{
		.mux_regs = pmx_keyboard_col5_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_keyboard_col5_mux),
	},
};

struct pmx_dev spear1340_pmx_keyboard_col5 = {
	.name = "keyboard_col5",
	.modes = pmx_keyboard_col5_modes,
	.mode_count = ARRAY_SIZE(pmx_keyboard_col5_modes),
};

/* pad multiplexing for uart0_enh device */
static struct pmx_mux_reg pmx_uart0_enh_mux[] = {
	{
		.address = SPEAR1340_PAD_MUX_CONFIG_REG_0,
		.mask = SPEAR1340_PMX_GPT_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_uart0_enh_modes[] = {
	{
		.mux_regs = pmx_uart0_enh_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_uart0_enh_mux),
	},
};

struct pmx_dev spear1340_pmx_uart0_enh = {
	.name = "uart0_enh",
	.modes = pmx_uart0_enh_modes,
	.mode_count = ARRAY_SIZE(pmx_uart0_enh_modes),
};

/* pad multiplexing for gpt_0_1 device */
static struct pmx_mux_reg pmx_gpt_0_1_mux[] = {
	{
		.address = SPEAR1340_PAD_MUX_CONFIG_REG_0,
		.mask = SPEAR1340_PMX_GPT_MASK |
			SPEAR1340_PMX_GPT0_TMR0_CPT_MASK |
			SPEAR1340_PMX_GPT0_TMR1_CLK_MASK,
		.value = SPEAR1340_PMX_GPT_MASK |
			SPEAR1340_PMX_GPT0_TMR0_CPT_MASK |
			SPEAR1340_PMX_GPT0_TMR1_CLK_MASK,
	},
};

static struct pmx_dev_mode pmx_gpt_0_1_modes[] = {
	{
		.mux_regs = pmx_gpt_0_1_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_gpt_0_1_mux),
	},
};

struct pmx_dev spear1340_pmx_gpt_0_1 = {
	.name = "gpt_0_1",
	.modes = pmx_gpt_0_1_modes,
	.mode_count = ARRAY_SIZE(pmx_gpt_0_1_modes),
};

/* pad multiplexing for pwm1 device */
static struct pmx_mux_reg pmx_pwm1_mux[] = {
	{
		.address = SPEAR1340_PAD_MUX_CONFIG_REG_0,
		.mask = SPEAR1340_PMX_KBD_COL5_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_pwm1_modes[] = {
	{
		.mux_regs = pmx_pwm1_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_pwm1_mux),
	},
};

struct pmx_dev spear1340_pmx_pwm1 = {
	.name = "pwm1",
	.modes = pmx_pwm1_modes,
	.mode_count = ARRAY_SIZE(pmx_pwm1_modes),
};

/* pad multiplexing for pwm2 device */
static struct pmx_mux_reg pmx_pwm2_mux[] = {
	{
		.address = SPEAR1340_PAD_MUX_CONFIG_REG_0,
		.mask = SPEAR1340_PMX_GPT0_TMR0_CPT_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_pwm2_modes[] = {
	{
		.mux_regs = pmx_pwm2_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_pwm2_mux),
	},
};

struct pmx_dev spear1340_pmx_pwm2 = {
	.name = "pwm2",
	.modes = pmx_pwm2_modes,
	.mode_count = ARRAY_SIZE(pmx_pwm2_modes),
};

/* pad multiplexing for pwm3 device */
static struct pmx_mux_reg pmx_pwm3_mux[] = {
	{
		.address = SPEAR1340_PAD_MUX_CONFIG_REG_0,
		.mask = SPEAR1340_PMX_GPT0_TMR1_CLK_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_pwm3_modes[] = {
	{
		.mux_regs = pmx_pwm3_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_pwm3_mux),
	},
};

struct pmx_dev spear1340_pmx_pwm3 = {
	.name = "pwm3",
	.modes = pmx_pwm3_modes,
	.mode_count = ARRAY_SIZE(pmx_pwm3_modes),
};

/* pad multiplexing for pwm0 device */
static struct pmx_mux_reg pmx_pwm0_mux[] = {
	{
		.address = SPEAR1340_PAD_MUX_CONFIG_REG_0,
		.mask = SPEAR1340_PMX_SSP0_CS0_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_pwm0_modes[] = {
	{
		.mux_regs = pmx_pwm0_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_pwm0_mux),
	},
};

struct pmx_dev spear1340_pmx_pwm0 = {
	.name = "pwm0",
	.modes = pmx_pwm0_modes,
	.mode_count = ARRAY_SIZE(pmx_pwm0_modes),
};

/* pad multiplexing for ssp0_cs0 device */
static struct pmx_mux_reg pmx_ssp0_cs0_mux[] = {
	{
		.address = SPEAR1340_PAD_MUX_CONFIG_REG_0,
		.mask = SPEAR1340_PMX_SSP0_CS0_MASK,
		.value = SPEAR1340_PMX_SSP0_CS0_MASK,
	},
};

static struct pmx_dev_mode pmx_ssp0_cs0_modes[] = {
	{
		.mux_regs = pmx_ssp0_cs0_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_ssp0_cs0_mux),
	},
};

struct pmx_dev spear1340_pmx_ssp0_cs0 = {
	.name = "ssp0_cs0",
	.modes = pmx_ssp0_cs0_modes,
	.mode_count = ARRAY_SIZE(pmx_ssp0_cs0_modes),
};

/* pad multiplexing for video_in_mux_cam0 (disables cam0) device */
static struct pmx_mux_reg pmx_video_in_mux_cam0_mux[] = {
	{
		.address = SPEAR1340_PAD_MUX_CONFIG_REG_0,
		.mask = SPEAR1340_PMX_CAM0_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_video_in_mux_cam0_modes[] = {
	{
		.mux_regs = pmx_video_in_mux_cam0_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_video_in_mux_cam0_mux),
	},
};

struct pmx_dev spear1340_pmx_video_in_mux_cam0 = {
	.name = "video_in_mux_cam0",
	.modes = pmx_video_in_mux_cam0_modes,
	.mode_count = ARRAY_SIZE(pmx_video_in_mux_cam0_modes),
};

/* pad multiplexing for video_in_mux_cam1 (disables cam1) device */
static struct pmx_mux_reg pmx_video_in_mux_cam1_mux[] = {
	{
		.address = SPEAR1340_PAD_MUX_CONFIG_REG_0,
		.mask = SPEAR1340_PMX_CAM1_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_video_in_mux_cam1_modes[] = {
	{
		.mux_regs = pmx_video_in_mux_cam1_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_video_in_mux_cam1_mux),
	},
};

struct pmx_dev spear1340_pmx_video_in_mux_cam1 = {
	.name = "video_in_mux_cam1",
	.modes = pmx_video_in_mux_cam1_modes,
	.mode_count = ARRAY_SIZE(pmx_video_in_mux_cam1_modes),
};

/* pad multiplexing for video_in_mux_cam2 (disables cam2) device */
static struct pmx_mux_reg pmx_video_in_mux_cam2_mux[] = {
	{
		.address = SPEAR1340_PAD_MUX_CONFIG_REG_0,
		.mask = SPEAR1340_PMX_CAM2_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_video_in_mux_cam2_modes[] = {
	{
		.mux_regs = pmx_video_in_mux_cam2_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_video_in_mux_cam2_mux),
	},
};

struct pmx_dev spear1340_pmx_video_in_mux_cam2 = {
	.name = "video_in_mux_cam2",
	.modes = pmx_video_in_mux_cam2_modes,
	.mode_count = ARRAY_SIZE(pmx_video_in_mux_cam2_modes),
};

/* pad multiplexing for video_in_mux_cam3 (disables cam3) device */
static struct pmx_mux_reg pmx_video_in_mux_cam3_mux[] = {
	{
		.address = SPEAR1340_PAD_MUX_CONFIG_REG_0,
		.mask = SPEAR1340_PMX_CAM3_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_video_in_mux_cam3_modes[] = {
	{
		.mux_regs = pmx_video_in_mux_cam3_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_video_in_mux_cam3_mux),
	},
};

struct pmx_dev spear1340_pmx_video_in_mux_cam3 = {
	.name = "video_in_mux_cam3",
	.modes = pmx_video_in_mux_cam3_modes,
	.mode_count = ARRAY_SIZE(pmx_video_in_mux_cam3_modes),
};

/* pad multiplexing for cam3 device */
static struct pmx_mux_reg pmx_cam3_mux[] = {
	{
		.address = SPEAR1340_PAD_MUX_CONFIG_REG_0,
		.mask = SPEAR1340_PMX_CAM3_MASK,
		.value = SPEAR1340_PMX_CAM3_MASK,
	},
};

static struct pmx_dev_mode pmx_cam3_modes[] = {
	{
		.mux_regs = pmx_cam3_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_cam3_mux),
	},
};

struct pmx_dev spear1340_pmx_cam3 = {
	.name = "cam3",
	.modes = pmx_cam3_modes,
	.mode_count = ARRAY_SIZE(pmx_cam3_modes),
};

/* pad multiplexing for cam2 device */
static struct pmx_mux_reg pmx_cam2_mux[] = {
	{
		.address = SPEAR1340_PAD_MUX_CONFIG_REG_0,
		.mask = SPEAR1340_PMX_CAM2_MASK,
		.value = SPEAR1340_PMX_CAM2_MASK,
	},
};

static struct pmx_dev_mode pmx_cam2_modes[] = {
	{
		.mux_regs = pmx_cam2_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_cam2_mux),
	},
};

struct pmx_dev spear1340_pmx_cam2 = {
	.name = "cam2",
	.modes = pmx_cam2_modes,
	.mode_count = ARRAY_SIZE(pmx_cam2_modes),
};

/* pad multiplexing for cam1 device */
static struct pmx_mux_reg pmx_cam1_mux[] = {
	{
		.address = SPEAR1340_PAD_MUX_CONFIG_REG_0,
		.mask = SPEAR1340_PMX_CAM1_MASK,
		.value = SPEAR1340_PMX_CAM1_MASK,
	},
};

static struct pmx_dev_mode pmx_cam1_modes[] = {
	{
		.mux_regs = pmx_cam1_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_cam1_mux),
	},
};

struct pmx_dev spear1340_pmx_cam1 = {
	.name = "cam1",
	.modes = pmx_cam1_modes,
	.mode_count = ARRAY_SIZE(pmx_cam1_modes),
};

/* pad multiplexing for cam0 device */
static struct pmx_mux_reg pmx_cam0_mux[] = {
	{
		.address = SPEAR1340_PAD_MUX_CONFIG_REG_0,
		.mask = SPEAR1340_PMX_CAM0_MASK,
		.value = SPEAR1340_PMX_CAM0_MASK,
	},
};

static struct pmx_dev_mode pmx_cam0_modes[] = {
	{
		.mux_regs = pmx_cam0_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_cam0_mux),
	},
};

struct pmx_dev spear1340_pmx_cam0 = {
	.name = "cam0",
	.modes = pmx_cam0_modes,
	.mode_count = ARRAY_SIZE(pmx_cam0_modes),
};

/* pad multiplexing for ssp0_cs1 device */
static struct pmx_mux_reg pmx_ssp0_cs1_mux[] = {
	{
		.address = SPEAR1340_PAD_MUX_CONFIG_REG_0,
		.mask = SPEAR1340_PMX_SSP0_CS1_MASK,
		.value = SPEAR1340_PMX_SSP0_CS1_MASK,
	},
};

static struct pmx_dev_mode pmx_ssp0_cs1_modes[] = {
	{
		.mux_regs = pmx_ssp0_cs1_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_ssp0_cs1_mux),
	},
};

struct pmx_dev spear1340_pmx_ssp0_cs1 = {
	.name = "ssp0_cs1",
	.modes = pmx_ssp0_cs1_modes,
	.mode_count = ARRAY_SIZE(pmx_ssp0_cs1_modes),
};

/* pad multiplexing for fsmc_pnor device */
static struct pmx_mux_reg pmx_fsmc_pnor_mux[] = {
	{
		.address = SPEAR1340_PAD_MUX_CONFIG_REG_0,
		.mask = SPEAR1340_PMX_MCIF_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_fsmc_pnor_modes[] = {
	{
		.mux_regs = pmx_fsmc_pnor_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_fsmc_pnor_mux),
	},
};

struct pmx_dev spear1340_pmx_fsmc_pnor = {
	.name = "fsmc_pnor",
	.modes = pmx_fsmc_pnor_modes,
	.mode_count = ARRAY_SIZE(pmx_fsmc_pnor_modes),
};

/* pad multiplexing for mcif device */
static struct pmx_mux_reg pmx_mcif_mux[] = {
	{
		.address = SPEAR1340_PAD_MUX_CONFIG_REG_0,
		.mask = SPEAR1340_PMX_MCIF_MASK,
		.value = SPEAR1340_PMX_MCIF_MASK,
	},
};

static struct pmx_dev_mode pmx_mcif_modes[] = {
	{
		.mux_regs = pmx_mcif_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_mcif_mux),
	},
};

struct pmx_dev spear1340_pmx_mcif = {
	.name = "mcif",
	.modes = pmx_mcif_modes,
	.mode_count = ARRAY_SIZE(pmx_mcif_modes),
};

/* pad multiplexing for clcd device */
static struct pmx_mux_reg pmx_clcd_mux[] = {
	{
		.address = SPEAR1340_PAD_MUX_CONFIG_REG_0,
		.mask = SPEAR1340_PMX_ARM_TRACE_MASK |
			SPEAR1340_PMX_MIPHY_DBG_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_clcd_modes[] = {
	{
		.mux_regs = pmx_clcd_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_clcd_mux),
	},
};

struct pmx_dev spear1340_pmx_clcd = {
	.name = "clcd",
	.modes = pmx_clcd_modes,
	.mode_count = ARRAY_SIZE(pmx_clcd_modes),
};

/* pad multiplexing for arm_trace device */
static struct pmx_mux_reg pmx_arm_trace_mux[] = {
	{
		.address = SPEAR1340_PAD_MUX_CONFIG_REG_0,
		.mask = SPEAR1340_PMX_ARM_TRACE_MASK,
		.value = SPEAR1340_PMX_ARM_TRACE_MASK,
	},
};

static struct pmx_dev_mode pmx_arm_trace_modes[] = {
	{
		.mux_regs = pmx_arm_trace_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_arm_trace_mux),
	},
};

struct pmx_dev spear1340_pmx_arm_trace = {
	.name = "arm_trace",
	.modes = pmx_arm_trace_modes,
	.mode_count = ARRAY_SIZE(pmx_arm_trace_modes),
};

/* pad multiplexing for device group: I2S, SSP0_CS2, CEC0-1, SPDIFF out, CLCD */
static struct pmx_mux_reg pmx_devs_grp_mux[] = {
	{
		.address = SPEAR1340_PAD_MUX_CONFIG_REG_0,
		.mask = SPEAR1340_PMX_MIPHY_DBG_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_devs_grp_modes[] = {
	{
		.mux_regs = pmx_devs_grp_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_devs_grp_mux),
	},
};

struct pmx_dev spear1340_pmx_devs_grp = {
	.name = "devs_grp",
	.modes = pmx_devs_grp_modes,
	.mode_count = ARRAY_SIZE(pmx_devs_grp_modes),
};

/* pad multiplexing for miphy_dbg device */
static struct pmx_mux_reg pmx_miphy_dbg_mux[] = {
	{
		.address = SPEAR1340_PAD_MUX_CONFIG_REG_0,
		.mask = SPEAR1340_PMX_MIPHY_DBG_MASK,
		.value = SPEAR1340_PMX_MIPHY_DBG_MASK,
	},
};

static struct pmx_dev_mode pmx_miphy_dbg_modes[] = {
	{
		.mux_regs = pmx_miphy_dbg_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_miphy_dbg_mux),
	},
};

struct pmx_dev spear1340_pmx_miphy_dbg = {
	.name = "miphy_dbg",
	.modes = pmx_miphy_dbg_modes,
	.mode_count = ARRAY_SIZE(pmx_miphy_dbg_modes),
};

/* Add spear1340 specific devices here */
/* uart device registeration */
struct dw_dma_slave uart1_dma_param[] = {
	{
		/* Tx */
		.dma_dev = &spear13xx_dmac_device[0].dev,
		.tx_reg = SPEAR1340_UART1_BASE + UART01x_DR,
		.reg_width = DW_DMA_SLAVE_WIDTH_8BIT,
		.cfg_hi = DWC_CFGH_DST_PER(SPEAR1340_DMA_REQ_UART1_TX),
		.cfg_lo = 0,
		.src_master = SPEAR1340_DMA_MASTER_MEMORY,
		.dst_master = SPEAR1340_DMA_MASTER_UART1,
		.src_msize = DW_DMA_MSIZE_8,
		.dst_msize = DW_DMA_MSIZE_8,
		.fc = DW_DMA_FC_D_M2P,
	}, {
		/* Rx */
		.dma_dev = &spear13xx_dmac_device[0].dev,
		.rx_reg = SPEAR1340_UART1_BASE + UART01x_DR,
		.reg_width = DW_DMA_SLAVE_WIDTH_8BIT,
		.cfg_hi = DWC_CFGH_SRC_PER(SPEAR1340_DMA_REQ_UART1_RX),
		.cfg_lo = 0,
		.src_master = SPEAR1340_DMA_MASTER_UART1,
		.dst_master = SPEAR1340_DMA_MASTER_MEMORY,
		.src_msize = DW_DMA_MSIZE_8,
		.dst_msize = DW_DMA_MSIZE_8,
		.fc = DW_DMA_FC_D_P2M,
	}
};

struct amba_pl011_data uart1_data = {
	.dma_filter = dw_dma_filter,
	.dma_tx_param = &uart1_dma_param[0],
	.dma_rx_param = &uart1_dma_param[1],
};

/* uart1 device registeration */
struct amba_device spear1340_uart1_device = {
	.dev = {
		.init_name = "uart",
	},
	.res = {
		.start = SPEAR1340_UART1_BASE,
		.end = SPEAR1340_UART1_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	.irq = {SPEAR1340_IRQ_UART1, NO_IRQ},
};

/* i2c device registeration */
static struct resource i2c1_resources[] = {
	{
		.start = SPEAR1340_I2C1_BASE,
		.end = SPEAR1340_I2C1_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR1340_IRQ_I2C1,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear1340_i2c1_device = {
	.name = "i2c_designware",
	.id = 1,
	.dev = {
		.coherent_dma_mask = ~0,
	},
	.num_resources = ARRAY_SIZE(i2c1_resources),
	.resource = i2c1_resources,
};

/* plgpio */
static struct plgpio_platform_data plgpio_plat_data = {
	.gpio_base = 16,
	.irq_base = SPEAR_PLGPIO_INT_BASE,
	.gpio_count = SPEAR_PLGPIO_COUNT,
	.regs = {
		.wdata = SPEAR1340_PLGPIO_WDATA_OFF,
		.dir = SPEAR1340_PLGPIO_DIR_OFF,
		.rdata = SPEAR1340_PLGPIO_RDATA_OFF,
		.ie = SPEAR1340_PLGPIO_IE_OFF,
		.mis = SPEAR1340_PLGPIO_MIS_OFF,
		.eit = SPEAR1340_PLGPIO_EIT_OFF,
	},
};

static struct resource plgpio_resources[] = {
	{
		.start = SPEAR1340_PLGPIO_BASE,
		.end = SPEAR1340_PLGPIO_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR1340_IRQ_PLGPIO,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear1340_plgpio_device = {
	.name = "plgpio",
	.id = -1,
	.dev = {
		.platform_data = &plgpio_plat_data,
	},
	.num_resources = ARRAY_SIZE(plgpio_resources),
	.resource = plgpio_resources,
};

/* pwm device registeration */
static struct resource pwm_resources[] = {
	{
		.start = SPEAR1340_PWM_BASE,
		.end = SPEAR1340_PWM_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
};

struct platform_device spear1340_pwm_device = {
	.name = "pwm",
	.id = -1,
	.num_resources = ARRAY_SIZE(pwm_resources),
	.resource = pwm_resources,
};

/* SATA device registration */
static struct resource sata_resources[] = {
	{
		.start = SPEAR1340_SATA_BASE,
		.end = SPEAR1340_SATA_BASE + SZ_8K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR1340_IRQ_SATA,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear1340_sata0_device = {
	.name = "ahci",
	.id = 0,
	.num_resources = ARRAY_SIZE(sata_resources),
	.resource = sata_resources,
};

void __init spear1340_init(struct pmx_mode *pmx_mode, struct pmx_dev **pmx_devs,
		u8 pmx_dev_count)
{
	int ret;

	/* call spear13xx family common init function */
	spear13xx_init();

	/* pmx initialization */
	pmx_driver.mode = pmx_mode;
	pmx_driver.devs = pmx_devs;
	pmx_driver.devs_count = pmx_dev_count;

	ret = pmx_register(&pmx_driver);
	if (ret)
		pr_err("padmux: registeration failed. err no: %d\n", ret);
}
