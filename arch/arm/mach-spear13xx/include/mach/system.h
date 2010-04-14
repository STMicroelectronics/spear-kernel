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
#include <mach/hardware.h>

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
#if 0
	if (mode == 's') {
		/* software reset, Jump into ROM at address 0 */
		cpu_reset(0);
	} else {
		/* hardware reset, Use on-chip reset capability */
		sysctl_soft_reset(SPEAR13XX_SYS_CTRL_BASE);
	}
#endif /* TODO: */
}

#endif /* __MACH_SYSTEM_H */
