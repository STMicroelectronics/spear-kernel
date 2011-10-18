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

#include <linux/io.h>
#include <linux/module.h>
#include <linux/suspend.h>
#include <linux/sysfs.h>
#include <asm/cacheflush.h>
#include <asm/hardware/cache-l2x0.h>
#include <asm/smp_twd.h>
#include <mach/hardware.h>
#include <mach/misc_regs.h>
#include <mach/suspend.h>
#include <asm/cpu_pm.h>
#include <asm/hardware/gic.h>

#define PLAT_PHYS_OFFSET	0x00000000
#define PCM_SET_WAKEUP_CFG	0xfffff
#define PCM_SET_CFG	0x3c1f

static void __iomem *mpmc_regs_base;

static void memcpy_decr_ptr(void *dest, void *src, u32 len)
{
	int i;

	for (i = 0; i < len ; i++)
		*((u32 *)(dest - (i<<2))) = *((u32 *)(src + (i<<2)));
}

static int spear_pm_sleep(suspend_state_t state)
{
	u32 twd_ctrl, twd_load;

	twd_ctrl = readl(twd_base + TWD_TIMER_CONTROL);
	twd_load = readl(twd_base + TWD_TIMER_LOAD);
	/* twd timer stop */
	writel(0, twd_base + TWD_TIMER_CONTROL);

	/* Do the GIC specific latch ups for suspend mode */
	if (state == PM_SUSPEND_MEM) {
		gic_cpu_exit(0);
		gic_dist_save(0);
		/* Suspend PCIE bus */
		spear_pcie_suspend();
	}

	/* Suspend the event timer */
	spear_clocksource_suspend();
	/* Move the cpu into suspend */
	spear_cpu_suspend(state, PLAT_PHYS_OFFSET - PAGE_OFFSET);
	/* Resume Operations begin */
	l2x0_init(__io_address(SPEAR13XX_L2CC_BASE), 0x00260249, 0xfe00ffff);
	/* Call the CPU PM notifiers to notify exit from sleep */
	cpu_pm_exit();
	/* twd timer restart */
	writel(twd_ctrl, twd_base + TWD_TIMER_CONTROL);
	writel(twd_load, twd_base + TWD_TIMER_LOAD);

	/* Do the GIC restoration for suspend mode */
	if (state == PM_SUSPEND_MEM) {
		gic_dist_restore(0);
		/* Resume PCIE bus */
		spear_pcie_resume();
	}

	/* Resume the event timer */
	spear_clocksource_resume();

	return 0;
}

/*
 * This function call is made post the CPU suspend is done.
 */
void spear_sys_suspend(suspend_state_t state)
{

	void (*spear_sram_sleep)(suspend_state_t state, unsigned long *saveblk,
			int revision) = NULL;
	void (*spear_sram_wake)(void) = NULL;
	void *sram_dest = (void *)IO_ADDRESS(SPEAR_START_SRAM);
	void *sram_limit_va = (void *)IO_ADDRESS(SPEAR_LIMIT_SRAM);
	u32 pm_cfg = readl(VA_PCM_CFG);

	if (state == PM_SUSPEND_MEM) {
		spear_sram_wake = memcpy(sram_dest, (void *)spear_wakeup,
				spear_wakeup_sz);
		/* Increment destination pointer by the size copied*/
		sram_dest += roundup(spear_wakeup_sz, 4);
		/*
		 * Set up the Power Domains specific registers.
		 * 1. Setup the wake up enable of the desired sources.
		 * 2. Set the wake up trigger field to zero
		 * 3. Clear config_ack and config_bad
		 * 4. Set sw_config equal to ack_power_state
		 * The currrent S2R operations enable all the wake up
		 * sources by default.
		 */
		writel(pm_cfg | PCM_SET_CFG, VA_PCM_CFG);
		/* Set up the desired wake up state */
		pm_cfg = readl(VA_PCM_WKUP_CFG);
		/* Set the states for all power island on */
		writel(pm_cfg | PCM_SET_WAKEUP_CFG, VA_PCM_WKUP_CFG);
	} else {
		/* source gpio interrupt through GIC */
		writel((pm_cfg & (~(1 << 2))), VA_PCM_CFG);
	}

	/*
	 * Copy in the MPMC registers at the end of SRAM
	 * Ensure that the backup of these registers does not
	 * overlap the code being copied.
	 */
	memcpy_decr_ptr(sram_limit_va , mpmc_regs_base, 200);

	/* Copy the Sleep code on to the SRAM*/
	spear_sram_sleep = memcpy(sram_dest, (void *)spear_sleep_mode,
			spear_sleep_mode_sz);
	/* Call the CPU PM notifiers to notify entry in sleep */
	cpu_pm_enter();
	/* Flush the cache */
	flush_cache_all();
	outer_flush_all();
	outer_disable();
	outer_sync();
	/* Jump to the suspend routines in sram */
	if (cpu_is_spear1340() || cpu_is_spear1310())
		spear_sram_sleep(state, (unsigned long *)cpu_resume, 1);
	else
		spear_sram_sleep(state, (unsigned long *)cpu_resume, 0);
}

/*
 *	spear_pm_prepare - Do preliminary suspend work.
 *
 */
static int spear_pm_prepare(void)
{
	mpmc_regs_base = ioremap(SPEAR13XX_MPMC_BASE, 1024);
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
	iounmap(mpmc_regs_base);
	enable_hlt();
}

/*
 *	spear_pm_valid_state- check the valid states in PM for the SPEAr
 *	platform
 */
static int spear_pm_valid_state(suspend_state_t state)
{
	switch (state) {
	case PM_SUSPEND_STANDBY:
	case PM_SUSPEND_MEM:
		return 1;
	default:
		return 0;
	}
}

static struct platform_suspend_ops spear_pm_ops = {
	.prepare	= spear_pm_prepare,
	.enter		= spear_pm_enter,
	.finish		= spear_pm_finish,
	.valid		= spear_pm_valid_state,
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
