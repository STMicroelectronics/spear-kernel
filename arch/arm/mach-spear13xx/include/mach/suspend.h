/*
 * arch/arm/mach-spear13xx/include/mach/suspend.h
 *
 * Sleep mode defines for SPEAr13xx machine family
 *
 * Copyright (C) 2010 ST Microelectronics
 * AUTHOR : Deepak Sikri <deepak.sikri@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __MACH_SUSPEND_H
#define __MACH_SUSPEND_H

#include <mach/hardware.h>

#ifndef __ASSEMBLER__
extern void spear_sleep_mode(suspend_state_t state, unsigned long *saveblk,
		int revision);
extern unsigned int spear_sleep_mode_sz;
extern void spear_wakeup(void);
extern unsigned int spear_wakeup_sz;
extern int spear_cpu_suspend(suspend_state_t, long);
extern void spear_clocksource_resume(void);
extern void spear_clocksource_suspend(void);
extern int spear_pcie_suspend(void);
extern int spear_pcie_resume(void);

#endif

/* SRAM related defines*/
#define SRAM_STACK_STRT_OFF	0x800
#define SRAM_STACK_SCR_OFFS	0x900
#define SPEAR_START_SRAM	SPEAR13XX_SYSRAM1_BASE
#define SPEAR_LIMIT_SRAM	(SPEAR_START_SRAM + SZ_4K - 4)
#define SPEAR_SRAM_START_PA	SPEAR_START_SRAM
#define SPEAR_SRAM_STACK_L2	(SPEAR_START_SRAM + SRAM_STACK_STRT_OFF - 0x30)
#define SPEAR_SRAM_STACK_PA	(SPEAR_START_SRAM + SRAM_STACK_STRT_OFF)
#define SPEAR_SRAM_SCR_REG	(SPEAR_START_SRAM + SRAM_STACK_SCR_OFFS)
#define SRAM_SCRATCH_PA		(SPEAR13XX_SYS_LOCATION)
/* SPEAr subsystem physical addresses */
#define MPMC_BASE_PA		SPEAR13XX_MPMC_BASE
#define MISC_BASE_PA		SPEAR13XX_MISC_BASE
#define GPIO_START_PA		SPEAR13XX_GPIO0_BASE
#define GPIO_START_UPD		SPEAR13XX_UPD_BASE

#endif /* __MACH_SUSPEND_H */
