/*
 * arch/arm/mach-spear3xx/spear3xx.c
 *
 * SPEAr3XX machines common source file
 *
 * Copyright (C) 2009-2012 ST Microelectronics
 * Viresh Kumar <viresh.linux@gmail.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#define pr_fmt(fmt) "SPEAr3xx: " fmt

#include <linux/amba/pl022.h>
#include <linux/amba/pl08x.h>
#include <linux/of_irq.h>
#include <linux/io.h>
#include <asm/hardware/pl080.h>
#include <asm/hardware/vic.h>
#include <plat/adc.h>
#include <plat/jpeg.h>
#include <plat/pl080.h>
#include <plat/shirq.h>
#include <mach/emi.h>
#include <mach/generic.h>
#include <mach/misc_regs.h>
#include <mach/spear.h>

/* adc device registeration */
struct adc_plat_data adc_pdata = {
	.dma_filter = pl08x_filter_id,
	.dma_data = "adc",
	.config = {CONTINUOUS_CONVERSION, EXTERNAL_VOLT, 2500, INTERNAL_SCAN,
		NORMAL_RESOLUTION, 14000000, 0},
};

#define VA_SPEAR3XX_PERIP1_SW_RST	(VA_SPEAR3XX_ICM3_MISC_REG_BASE + 0x038)
	#define SPEAR3XX_JPEG_SOF_RST	(1 << 8)
/* jpeg dma platform data */
static void jpeg_plat_reset(void)
{
	jpeg_ip_reset((void __iomem *) VA_SPEAR3XX_PERIP1_SW_RST,
			SPEAR3XX_JPEG_SOF_RST);
}

struct jpeg_plat_data jpeg_pdata = {
	.dma_filter = pl08x_filter_id,
	.mem2jpeg_slave = "to_jpeg",
	.jpeg2mem_slave = "from_jpeg",
	.plat_reset = jpeg_plat_reset,
};

/* ssp device registration */
struct pl022_ssp_controller pl022_plat_data = {
	.bus_id = 0,
	.enable_dma = 1,
	.dma_filter = pl08x_filter_id,
	.dma_tx_param = "ssp0_tx",
	.dma_rx_param = "ssp0_rx",
	/*
	 * This is number of spi devices that can be connected to spi. There are
	 * two type of chipselects on which slave devices can work. One is chip
	 * select provided by spi masters other is controlled through external
	 * gpio's. We can't use chipselect provided from spi master (because as
	 * soon as FIFO becomes empty, CS is disabled and transfer ends). So
	 * this number now depends on number of gpios available for spi. each
	 * slave on each master requires a separate gpio pin.
	 */
	.num_chipselect = 2,
};

/* dmac device registration */
struct pl08x_platform_data pl080_plat_data = {
	.memcpy_channel = {
		.bus_id = "memcpy",
		.cctl = (PL080_BSIZE_16 << PL080_CONTROL_SB_SIZE_SHIFT | \
			PL080_BSIZE_16 << PL080_CONTROL_DB_SIZE_SHIFT | \
			PL080_WIDTH_32BIT << PL080_CONTROL_SWIDTH_SHIFT | \
			PL080_WIDTH_32BIT << PL080_CONTROL_DWIDTH_SHIFT | \
			PL080_CONTROL_PROT_BUFF | PL080_CONTROL_PROT_CACHE | \
			PL080_CONTROL_PROT_SYS),
	},
	.lli_buses = PL08X_AHB1,
	.mem_buses = PL08X_AHB1,
	.get_signal = pl080_get_signal,
	.put_signal = pl080_put_signal,
};

/*
 * Following will create 16MB static virtual/physical mappings
 * PHYSICAL		VIRTUAL
 * 0xD0000000		0xFD000000
 * 0xFC000000		0xFC000000
 */
struct map_desc spear3xx_io_desc[] __initdata = {
	{
		.virtual	= VA_SPEAR3XX_ICM1_2_BASE,
		.pfn		= __phys_to_pfn(SPEAR3XX_ICM1_2_BASE),
		.length		= SZ_16M,
		.type		= MT_DEVICE
	}, {
		.virtual	= VA_SPEAR3XX_ICM3_SMI_CTRL_BASE,
		.pfn		= __phys_to_pfn(SPEAR3XX_ICM3_SMI_CTRL_BASE),
		.length		= SZ_16M,
		.type		= MT_DEVICE
	}, {
		.virtual	= VA_SPEAR3XX_ICM1_SRAM_BASE,
		.pfn		= __phys_to_pfn(SPEAR3XX_ICM1_SRAM_BASE),
		.length		= SZ_4K,
		.type		= MT_MEMORY_NONCACHED
	},
};

void spear3xx_macb_setup(void)
{
	struct clk *amem_clk;

	/* Enable memory Port-1 clock */
	amem_clk = clk_get(NULL, "amem_clk");
	if (IS_ERR(amem_clk)) {
		pr_err("%s:couldn't get %s\n", __func__, "amem_clk");
		return;
	}

	if (clk_prepare_enable(amem_clk)) {
		pr_err("%s:couldn't enable %s\n", __func__, "amem_clk");
		clk_put(amem_clk);
		return;
	}
	if (of_machine_is_compatible("st,spear310")) {
		/*
		 * Program the pad strengths of PLGPIO to drive the IO's
		 * The Magic number being used have direct correlations
		 * with the driving capabilities of the IO pads.
		 */
		writel(0x2f7bc210, PLGPIO3_PAD_PRG);
		writel(0x017bdef6, PLGPIO4_PAD_PRG);
	}
}

/* This will create static memory mapping for selected devices */
void __init spear3xx_map_io(void)
{
	iotable_init(spear3xx_io_desc, ARRAY_SIZE(spear3xx_io_desc));
}

int __init spear3xx_emi_init(u32 base, int numbanks)
{
	struct device_node *flash_node;
	void __iomem *regs;
	struct clk *clk;
	const char *status, *devid;
	int ret;
	u32 bank;
	u32 ctrl, ack_reg, width;
	/* u32 timeout_reg, irq_reg; */

	/* fixing machine dependent values */
	if (of_machine_is_compatible("st,spear310")) {
		devid = "50000000.flash";
		ack_reg = SPEAR310_ACK_REG;
		/* timeout_reg = SPEAR310_TIMEOUT_REG; */
		/* irq_reg = SPEAR310_IRQ_REG; */
	} else if (of_machine_is_compatible("st,spear320")) {
		devid = "44000000.flash";
		ack_reg = SPEAR320_ACK_REG;
		/* timeout_reg = SPEAR320_TIMEOUT_REG; */
		/* irq_reg = SPEAR320_IRQ_REG; */
	} else
		return -EINVAL;

	flash_node = of_find_compatible_node(NULL, NULL, "cfi-flash");

	if (!flash_node)
		/* cfi-flash node not found */
		return -ENOENT;

	if (!of_property_read_string(flash_node, "status", &status) &&
			!strcmp(status, "disabled"))
		return -ENOENT;

	clk = clk_get_sys(devid, NULL);
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		pr_err("emi-nor: clock get failed\n");
		goto eclkgetsys;
	}

	ret = clk_prepare_enable(clk);
	if (ret) {
		pr_err("emi-nor: clock enable failed\n");
		goto eclkenable;
	}

	if (of_property_read_u32(flash_node, "bank-width", &width)) {
		pr_err("emi-nor: Initialization failed\n");
		goto epropread;
	}

	regs = ioremap(base, 0x100);
	if (!regs) {
		pr_err("emi-nor: ioremap failed\n");
		goto eioremap;
	}

	ctrl = EMI_CNTL_ENBBYTERW;

	if (width == 1)
		ctrl |= EMI_CNTL_WIDTH8;
	else if (width == 2)
		ctrl |= EMI_CNTL_WIDTH16;
	else if (width == 4)
		ctrl |= EMI_CNTL_WIDTH32;

	/*
	 * Note: These are relaxed NOR device timings. Nor devices on spear
	 * eval machines are working fine with these timings. Specific board
	 * files can optimize these timings based on devices found on board.
	 */
	for (bank = 0; bank < numbanks; bank++) {
		writel(0x10, EMI_REG(regs, bank, TAP));
		writel(0x05, EMI_REG(regs, bank, TSDP));
		writel(0x0a, EMI_REG(regs, bank, TDPW));
		writel(0x0a, EMI_REG(regs, bank, TDPR));
		writel(0x05, EMI_REG(regs, bank, TDCS));

		writel(ctrl, EMI_REG(regs, bank, CTRL));
	}

	/* disable all the acks */
	writel(0x3f, regs + ack_reg);

	iounmap(regs);
	of_node_put(flash_node);
	return 0;

eioremap:
epropread:
	clk_disable_unprepare(clk);
eclkenable:
	clk_put(clk);
eclkgetsys:
	of_node_put(flash_node);
	return ret;
}

static void __init spear3xx_timer_init(void)
{
	char pclk_name[] = "pll3_clk";
	struct clk *gpt_clk, *pclk;

	spear3xx_clk_init();

	if (of_machine_is_compatible("st,spear320"))
		spear320_uart_clk_config();

	/* get the system timer clock */
	gpt_clk = clk_get_sys("gpt0", NULL);
	if (IS_ERR(gpt_clk)) {
		pr_err("%s:couldn't get clk for gpt\n", __func__);
		BUG();
	}

	/* get the suitable parent clock for timer*/
	pclk = clk_get(NULL, pclk_name);
	if (IS_ERR(pclk)) {
		pr_err("%s:couldn't get %s as parent for gpt\n",
				__func__, pclk_name);
		BUG();
	}

	clk_set_parent(gpt_clk, pclk);
	clk_put(gpt_clk);
	clk_put(pclk);

	spear_setup_of_timer();
}

struct sys_timer spear3xx_timer = {
	.init = spear3xx_timer_init,
};

static const struct of_device_id vic_of_match[] __initconst = {
	{ .compatible = "arm,pl190-vic", .data = vic_of_init, },
	{ .compatible = "st,spear300-shirq", .data = spear3xx_shirq_of_init, },
	{ .compatible = "st,spear310-shirq", .data = spear3xx_shirq_of_init, },
	{ .compatible = "st,spear320-shirq", .data = spear3xx_shirq_of_init, },
	{ /* Sentinel */ }
};

void __init spear3xx_dt_init_irq(void)
{
	of_irq_init(vic_of_match);
}
