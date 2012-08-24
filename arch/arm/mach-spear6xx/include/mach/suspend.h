/*
 * arch/arm/mach-spear6xx/include/mach/suspend.h
 *
 * Sleep mode defines for SPEAr6xx machine family
 *
 * Copyright (C) 2012 ST Microelectronics
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
extern void spear_sleep_mode(suspend_state_t state, unsigned long *saveblk);
extern unsigned int spear_sleep_mode_sz;
#endif

/* SRAM Stack offsets */
#define SRAM_STACK_STRT_OFF	0x650
/* SRAM related defines*/
#define SRAM_START_PA		SPEAR6XX_ICM1_SRAM_BASE
#define SRAM_START_VA		VA_SPEAR6XX_ICM1_SRAM_BASE
#define SRAM_SIZE		SZ_4K
#define SRAM_STACK_PA		(SRAM_START_PA + SRAM_STACK_STRT_OFF)
#define SRAM_STACK_VA		(SRAM_START_VA + SRAM_STACK_STRT_OFF)

/* SPEAr subsystem physical addresses */
#define SYS_CTRL_BASE_PA	SPEAR6XX_ICM3_SYS_CTRL_BASE
#define MPMC_BASE_PA		SPEAR6XX_ICM3_SDRAM_CTRL_BASE
#define MISC_BASE_PA		SPEAR6XX_ICM3_MISC_REG_BASE

#endif /* __MACH_SUSPEND_H */
