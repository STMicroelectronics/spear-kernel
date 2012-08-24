/*
 * arch/arm/plat-spear/pm.c
 *
 * SPEAr3xx & SPEAr6xx Power Management source file
 *
 * Copyright (C) 2012 ST Microelectronics
 * Deepak Sikri <deepak.sikri@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/suspend.h>
#include <linux/module.h>
#include <linux/io.h>
#include <asm/cacheflush.h>
#include <asm/suspend.h>
#include <mach/suspend.h>

static void (*spear_sram_sleep)(unsigned long *saveblk);

static int spear_sys_suspend(unsigned long arg)
{
	spear_sram_sleep((unsigned long *)cpu_resume);

	return 0;
}

static int spear_pm_enter(suspend_state_t state)
{
	switch (state) {
	case PM_SUSPEND_STANDBY:
	case PM_SUSPEND_MEM:
		cpu_suspend(0, spear_sys_suspend);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static struct platform_suspend_ops spear_pm_ops = {
	.enter		= spear_pm_enter,
	.valid		= suspend_valid_only_mem,
};

static int __init spear_pm_init(void)
{
	void *dest = (void *)SRAM_START_VA;
	void *src = (void *)spear_sleep_mode;
	unsigned int size = spear_sleep_mode_sz;

	/* In case the suspend code size is more than sram size return */
	if (size > SRAM_SIZE)
		return	-ENOMEM;

	/* Copy the Sleep code on to the SRAM*/
	spear_sram_sleep = memcpy(dest, src, size);
	flush_icache_range((unsigned long)dest, (unsigned long)(dest + size));

	suspend_set_ops(&spear_pm_ops);
	return 0;
}
arch_initcall(spear_pm_init);
