/*
 * arch/arm/mach-spear13xx/platsmp.c
 *
 * based upon linux/arch/arm/mach-realview/platsmp.c
 *
 * Copyright (C) 2010 ST Microelectronics Ltd.
 * Shiraz Hashim <shiraz.hashim@st.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/smp.h>
#include <asm/cacheflush.h>
#include <asm/hardware/gic.h>
#include <asm/localtimer.h>
#include <asm/mach-types.h>
#include <asm/smp_scu.h>
#include <asm/system.h>
#include <asm/unified.h>
#include <mach/generic.h>
#include <mach/hardware.h>

/*
 * control for which core is the next to come out of the secondary
 * boot "holding pen"
 */
volatile int __cpuinitdata pen_release = -1;
static DEFINE_SPINLOCK(boot_lock);

static void __iomem *scu_base_addr(void)
{
	return __io_address(SPEAR13XX_SCU_BASE);
}

static inline unsigned int get_core_count(void)
{
	void __iomem *scu_base = scu_base_addr();

	if (scu_base)
		return scu_get_core_count(scu_base);
	return 1;
}

void __cpuinit platform_secondary_init(unsigned int cpu)
{
	trace_hardirqs_off();

	/*
	 * if any interrupts are already enabled for the primary
	 * core (e.g. timer irq), then they will not have been enabled
	 * for us: do so
	 */
	gic_cpu_init(0, __io_address(SPEAR13XX_GIC_CPU_BASE));

	/*
	 * let the primary processor know we're out of the
	 * pen, then head off into the C entry point
	 */
	pen_release = -1;
	smp_wmb();

	/*
	 * Synchronise with the boot thread.
	 */
	spin_lock(&boot_lock);
	spin_unlock(&boot_lock);
}

int __cpuinit boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	unsigned long timeout;

	/*
	 * set synchronisation state between this boot processor
	 * and the secondary one
	 */
	spin_lock(&boot_lock);

	/*
	 * The secondary processor is waiting to be released from
	 * the holding pen - release it, then wait for it to flag
	 * that it has been released by resetting pen_release.
	 *
	 * Note that "pen_release" is the hardware CPU ID, whereas
	 * "cpu" is Linux's internal ID.
	 */
	pen_release = cpu;
	flush_cache_all();

	timeout = jiffies + (1 * HZ);
	while (time_before(jiffies, timeout)) {
		smp_rmb();
		if (pen_release == -1)
			break;

		udelay(10);
	}

	/*
	 * now the secondary core is starting up let it run its
	 * calibrations, then wait for it to finish
	 */
	spin_unlock(&boot_lock);

	return pen_release != -1 ? -ENOSYS : 0;
}

static void __init wakeup_secondary(void)
{
	/* nobody is to be released from the pen yet */
	pen_release = -1;

	/*
	 * Write the address of secondary startup into the system-wide
	 * location (presently it is in SRAM). The BootMonitor waits
	 * for this register to become non-zero.
	 * We must also send an sev to wake it up
	 */
	__raw_writel(BSYM(virt_to_phys(spear13xx_secondary_startup)),
			__io_address(SPEAR13XX_SYS_LOCATION));

	mb();

	/*
	 * Send a 'sev' to wake the secondary core from WFE.
	 */
	sev();
}

/*
 * Initialise the CPU possible map early - this describes the CPUs
 * which may be present or become present in the system.
 */
void __init smp_init_cpus(void)
{
	unsigned int i, ncores = get_core_count();

	for (i = 0; i < ncores; i++)
		set_cpu_possible(i, true);
}

void __init smp_prepare_cpus(unsigned int max_cpus)
{
	unsigned int ncores = get_core_count();
	unsigned int cpu = smp_processor_id();
	int i;

	/* sanity check */
	if (ncores == 0) {
		pr_err("Realview: strange CM count of 0? Default to 1\n");

		ncores = 1;
	}

	if (ncores > num_possible_cpus()) {
		ncores = num_possible_cpus();
		pr_err(
		       "spear13xx: no. of cores (%d) greater than configured "
		       "maximum of %d - clipping\n",
		       ncores, ncores);
	}

	smp_store_cpu_info(cpu);

	/*
	 * are we trying to boot more cores than exist?
	 */
	if (max_cpus > ncores)
		max_cpus = ncores;

	/*
	 * Initialise the present map, which describes the set of CPUs
	 * actually populated at the present time.
	 */
	for (i = 0; i < max_cpus; i++)
		set_cpu_present(i, true);

	/*
	 * Initialise the SCU if there are more than one CPU and let
	 * them know where to start.
	 */
	if (max_cpus > 1) {
		/*
		 * Enable the local timer or broadcast device for the
		 * boot CPU, but only if we have more than one CPU.
		 */
		percpu_timer_setup();

		scu_enable(scu_base_addr());
		wakeup_secondary();
	}
}
