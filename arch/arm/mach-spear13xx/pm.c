/*
 * arch/arm/mach-spear13xx/pm.c
 *
 * SPEAr13xx Power Management source file
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
#include <asm/cacheflush.h>
#include <mach/irqs.h>
#include <mach/suspend.h>
#include <mach/hardware.h>

static int spear_pm_sleep(suspend_state_t state)
{
	void (*spear_sram_sleep)(suspend_state_t state) = NULL;
	void (*spear_sram_wake)(void) = NULL;
	void *sram_dest = (void *)IO_ADDRESS(SPEAR_START_SRAM);

	if (state == PM_SUSPEND_MEM) {
		spear_sram_wake = memcpy(sram_dest, (void *)spear_wakeup,
				spear_wakeup_sz);
		/* Increment destination pointer by the size copied*/
		sram_dest += roundup(spear_wakeup_sz, 4);
	}

	/* Copy the Sleep code on to the SRAM*/
	spear_sram_sleep = memcpy(sram_dest, (void *)spear_sleep_mode,
			spear_sleep_mode_sz);
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
	disable_hlt();
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
	enable_hlt();
}

static struct platform_suspend_ops spear_pm_ops = {
	.prepare	= spear_pm_prepare,
	.enter		= spear_pm_enter,
	.finish		= spear_pm_finish,
	.valid		= suspend_valid_only_mem,
};

static int __init spear_pm_init(void)
{
	void * sram_limit_va = (void *)IO_ADDRESS(SPEAR_LIMIT_SRAM);
	void * sram_st_va = (void *)IO_ADDRESS(SPEAR_START_SRAM);

	/* In case the suspend code size is more than sram size return */
	if (spear_sleep_mode_sz > (sram_limit_va - sram_st_va))
		return	-ENOMEM;

	suspend_set_ops(&spear_pm_ops);

	return 0;
}
arch_initcall(spear_pm_init);
