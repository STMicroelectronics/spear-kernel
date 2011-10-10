/*
 * arch/arm/mach-spear13xx/include/mach/system.h
 *
 * spear13xx Machine family specific architecture functions
 *
 * Copyright (C) 2010 ST Microelectronics
 * Shiraz Hashim <shiraz.hashim@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __MACH_SYSTEM_H
#define __MACH_SYSTEM_H

#include <linux/io.h>
#include <asm/proc-fns.h>
#include <mach/hardware.h>
#include <mach/misc_regs.h>

static inline void arch_idle(void)
{
	/*
	 * This should do all the clock switching
	 * and wait for interrupt tricks
	 */
	cpu_do_idle();
}

static inline void arch_reset(char mode, const char *cmd)
{
	if (cpu_is_spear1340()) {
#ifdef CONFIG_CPU_SPEAR1340
		writel_relaxed(0x01, VA_SPEAR1340_SYS_SW_RES);
#endif
	} else if (cpu_is_spear1310()) {
#ifdef CONFIG_CPU_SPEAR1310
		writel_relaxed(0x01, VA_SPEAR1310_SYS_SW_RES);
#endif
	} else
		writel_relaxed(0x01, VA_SYS_SW_RES);
}

#endif /* __MACH_SYSTEM_H */
