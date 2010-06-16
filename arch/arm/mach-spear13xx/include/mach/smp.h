/*
 * arch/arm/mach-spear13xx/include/mach/smp.h
 *
 * Few SMP related definitions for spear13xx machine family
 *
 * Copyright (C) 2010 ST Microelectronics
 * Shiraz Hashim <shiraz.hashim@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __MACH_SMP_H
#define __MACH_SMP_H

#include <asm/hardware/gic.h>

#define hard_smp_processor_id()			\
	({						\
		unsigned int cpunum;			\
		__asm__("mrc p15, 0, %0, c0, c0, 5"	\
			: "=r" (cpunum));		\
		cpunum &= 0x0F;				\
	})

/*
 * set_event() is used to wake up secondary core from wfe using sev. Booting
 * code puts the second core into wfe(standby).
 */
#define set_event()	__asm__ __volatile__ ("sev" : : : "memory")

/* We use IRQ1 as the IPI */
static inline void smp_cross_call(const struct cpumask *mask)
{
	gic_raise_softirq(mask, 1);
}

#endif
