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

#include <mach/spear.h>

#ifndef __ASSEMBLER__
extern void spear_sleep_mode(suspend_state_t state);
extern unsigned int spear_sleep_mode_sz;
extern void spear_wakeup(void);
extern unsigned int spear_wakeup_sz;
#endif

/* SRAM related defines*/
#define SRAM_STACK_STRT_OFF	0x500
#define SRAM_STACK_SCR_OFFS	0xF00
#define SPEAR_START_SRAM	SPEAR13XX_SYSRAM1_BASE
#define SPEAR_LIMIT_SRAM	(SPEAR_START_SRAM + SZ_4K - 1)
#define SPEAR_SRAM_STACK_PA	(SPEAR_START_SRAM + SRAM_STACK_STRT_OFF)
#define SPEAR_SRAM_SCR_REG	(SPEAR_START_SRAM + SRAM_STACK_SCR_OFFS)
/* SPEAr subsystem physical addresses */
#define MPMC_BASE_PA		SPEAR13XX_MPMC_BASE
#define MISC_BASE_PA		SPEAR13XX_MISC_BASE

/* ARM Modes of Operation */
#define MODE_USR_32		0x10
#define MODE_FIQ_32		0x11
#define MODE_IRQ_32		0x12
#define MODE_SVC_32		0x13
#define MODE_ABT_32		0x17
#define MODE_UND_32		0x1B
#define MODE_SYS_32		0x1F
#define MODE_BITS		0x1F

#endif /* __MACH_SUSPEND_H */
