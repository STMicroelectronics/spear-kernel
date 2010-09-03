/*
 * arch/arm/mach-spear13xx/spear13xx.c
 *
 * SPEAr13XX machines common source file
 *
 * Copyright (C) 2010 ST Microelectronics
 * Shiraz Hashim <shiraz.hashim@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/types.h>
#include <linux/ptrace.h>
#include <linux/io.h>
#include <asm/hardware/gic.h>
#include <asm/irq.h>
#include <asm/localtimer.h>
#include <asm/mach/arch.h>
#include <asm/smp_twd.h>
#include <mach/irqs.h>
#include <mach/generic.h>
#include <mach/hardware.h>

/* Add spear13xx machines common devices here */
/* uart device registeration */
struct amba_device spear13xx_uart_device = {
	.dev = {
		.init_name = "uart",
	},
	.res = {
		.start = SPEAR13XX_UART_BASE,
		.end = SPEAR13XX_UART_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	.irq = {IRQ_UART, NO_IRQ},
};

/* rtc device registration */
static struct resource rtc_resources[] = {
	{
		.start = SPEAR13XX_RTC_BASE,
		.end = SPEAR13XX_RTC_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = IRQ_RTC,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear13xx_rtc_device = {
	.name = "rtc-spear",
	.id = -1,
	.num_resources = ARRAY_SIZE(rtc_resources),
	.resource = rtc_resources,
};

/* Do spear13xx familiy common initialization part here */
void __init spear13xx_init(void)
{
	/* nothing to do for now */
}

/* This will initialize vic */
void __init spear13xx_init_irq(void)
{
	gic_dist_init(0, __io_address(SPEAR13XX_GIC_DIST_BASE), 29);
	gic_cpu_init(0, __io_address(SPEAR13XX_GIC_CPU_BASE));
}

/* Following will create static virtual/physical mappings */
struct map_desc spear13xx_io_desc[] __initdata = {
	{
		.virtual	= IO_ADDRESS(SPEAR13XX_UART_BASE),
		.pfn		= __phys_to_pfn(SPEAR13XX_UART_BASE),
		.length		= SZ_4K,
		.type		= MT_DEVICE
	}, {
		.virtual	= IO_ADDRESS(SPEAR13XX_A9SM_PERIP_BASE),
		.pfn		= __phys_to_pfn(SPEAR13XX_A9SM_PERIP_BASE),
		.length		= SZ_8K,
		.type		= MT_DEVICE
	}, {
		.virtual	= IO_ADDRESS(SPEAR13XX_MISC_BASE),
		.pfn		= __phys_to_pfn(SPEAR13XX_MISC_BASE),
		.length		= SZ_8K,
		.type		= MT_DEVICE
	}, {
		.virtual	= IO_ADDRESS(SPEAR13XX_SYSRAM0_BASE),
		.pfn		= __phys_to_pfn(SPEAR13XX_SYSRAM0_BASE),
		.length		= SZ_32K,
		.type		= MT_DEVICE
	},
};

/* This will create static memory mapping for selected devices */
void __init spear13xx_map_io(void)
{
	iotable_init(spear13xx_io_desc, ARRAY_SIZE(spear13xx_io_desc));

	/* This will initialize clock framework */
	clk_init();
}

static void __init spear13xx_timer_init(void)
{
	char pclk_name[] = "osc1_24m_clk";
	struct clk *gpt_clk, *pclk;

#ifdef CONFIG_LOCAL_TIMERS
	/* Setup the local timer base */
	twd_base = __io_address(SPEAR13XX_LOCAL_TMR_BASE);
#endif

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

	spear_setup_timer();
}

struct sys_timer spear13xx_timer = {
	.init = spear13xx_timer_init,
};
