/*
 * arch/arm/plat-spear/pm.c
 *
 * SPEAr3xx & SPEAr6xx Power Management source file
 *
 * Copyright (C) 2010 ST Microelectronics
 * Deepak Sikri <deepak.sikri@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/suspend.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/io.h>
#include <asm/cacheflush.h>
#include <mach/irqs.h>
#include <mach/suspend.h>

static void (*saved_idle)(void);
static void __iomem *spear_sram_base;

static int spear_pm_sleep(suspend_state_t state)
{
	void (*spear_sram_sleep)(suspend_state_t state) = NULL;

	/* Copy the Sleep code on to the SRAM*/
	spear_sram_sleep = memcpy((void *)spear_sram_base,
			(void *)spear_sleep_mode, spear_sleep_mode_sz);
	flush_cache_all();
	/* Jump to the suspend routines in sram */
	spear_sram_sleep(state);
	return 0;
}

/*
 *	spear_pm_prepare - Do preliminary suspend work.
 *
 */
static int spear_pm_prepare(void)
{
	/* We cannot sleep in idle until we have resumed */
	saved_idle = pm_idle;
	pm_idle = NULL;
	return 0;
}

/*
 *	spear_pm_enter - Actually enter a sleep state.
 *	@state:		State we're entering.
 *
 */
static int spear_pm_enter(suspend_state_t state)
{
	int ret;

	switch (state) {
	case PM_SUSPEND_STANDBY:
	case PM_SUSPEND_MEM:
		ret = spear_pm_sleep(state);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

/*
 *	spear_pm_finish - Finish up suspend sequence.
 *
 *	This is called after we wake back up (or if entering the sleep state
 *	failed).
 */
static void spear_pm_finish(void)
{
	pm_idle = saved_idle;
}

static struct platform_suspend_ops spear_pm_ops = {
	.prepare	= spear_pm_prepare,
	.enter		= spear_pm_enter,
	.finish		= spear_pm_finish,
	.valid		= suspend_valid_only_mem,
};

static int __init spear_pm_init(void)
{

	spear_sram_base = ioremap(SPEAR_START_SRAM, SPEAR_SRAM_SIZE);

	if (!spear_sram_base)
		return -ENOMEM;

	/* In case the suspend code size is more than sram size return */
	if (spear_sleep_mode_sz > (SPEAR_SRAM_SIZE))
		return	-ENOMEM;

	suspend_set_ops(&spear_pm_ops);
	return 0;
}
arch_initcall(spear_pm_init);
