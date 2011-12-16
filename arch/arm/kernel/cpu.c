/*
 * Hibernation support specific for ARM
 *
 * Derived from work on ARM hibernation support by:
 *
 * Ubuntu project, hibernation support for mach-dove
 * Copyright (C) 2010 Nokia Corporation (Hiroshi Doyu)
 * Copyright (C) 2010 Texas Instruments, Inc. (Teerth Reddy et al.)
 *	https://lkml.org/lkml/2010/6/18/4
 *	https://lists.linux-foundation.org/pipermail/linux-pm/2010-June/027422.html
 *	https://patchwork.kernel.org/patch/96442/
 *
 * Copyright (C) 2006 Rafael J. Wysocki <rjw@sisk.pl>
 *
 * License terms: GNU General Public License (GPL) version 2
 */

#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/suspend.h>
#include <asm/tlbflush.h>
#include <asm/cacheflush.h>
#include <asm/pgalloc.h>
#include <asm/sections.h>

extern const void __nosave_begin, __nosave_end;

int pfn_is_nosave(unsigned long pfn)
{
	unsigned long nosave_begin_pfn = __pa_symbol(&__nosave_begin) >> PAGE_SHIFT;
	unsigned long nosave_end_pfn = PAGE_ALIGN(__pa_symbol(&__nosave_end)) >> PAGE_SHIFT;

	return (pfn >= nosave_begin_pfn) && (pfn < nosave_end_pfn);
}

void notrace save_processor_state(void)
{
	flush_thread();
	local_fiq_disable();
}

void notrace restore_processor_state(void)
{
	flush_tlb_all();
	flush_cache_all();
	local_fiq_enable();
}

u8 __swsusp_resume_stk[PAGE_SIZE/2] __nosavedata;
u32 __swsusp_save_sp;

int __swsusp_arch_resume_finish(void)
{
	identity_mapping_del(swapper_pg_dir, __pa(_stext), __pa(_etext));
	return 0;
}

/*
 * The framework loads the hibernation image into a linked list anchored
 * at restore_pblist, for swsusp_arch_resume() to copy back to the proper
 * destinations.
 *
 * To make this work if resume is triggered from initramfs, the
 * pagetables need to be switched to allow writes to kernel mem.
 */
void notrace __swsusp_arch_restore_image(void)
{
	extern struct pbe *restore_pblist;
	extern void cpu_resume(void);
	extern unsigned long sleep_save_sp;
	struct pbe *pbe;
	typeof(cpu_reset) *phys_reset = (typeof(cpu_reset) *)virt_to_phys(cpu_reset);

	cpu_switch_mm(swapper_pg_dir, &init_mm);

	for (pbe = restore_pblist; pbe; pbe = pbe->next)
		copy_page(pbe->orig_address, pbe->address);

	sleep_save_sp = __swsusp_save_sp;
	flush_tlb_all();
	flush_cache_all();

	identity_mapping_add(swapper_pg_dir, __pa(_stext), __pa(_etext));

	flush_tlb_all();
	flush_cache_all();
	cpu_proc_fin();

	flush_tlb_all();
	flush_cache_all();

	phys_reset(virt_to_phys(cpu_resume));
}

