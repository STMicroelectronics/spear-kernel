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
#include <mach/spear.h>


#ifndef __ASSEMBLER__
extern void spear1340_sleep_mode(suspend_state_t state, unsigned long *saveblk);
extern unsigned int spear1340_sleep_mode_sz;
extern void spear13xx_sleep_mode(suspend_state_t state, unsigned long *saveblk);
extern unsigned int spear13xx_sleep_mode_sz;
extern void spear1310_sleep_mode(suspend_state_t state, unsigned long *saveblk);
extern unsigned int spear1310_sleep_mode_sz;
extern void spear_wakeup(void);
extern unsigned int spear_wakeup_sz;
extern int spear_cpu_suspend(suspend_state_t, long);
extern void spear_clocksource_resume(void);
extern void spear_clocksource_suspend(void);
extern int spear_pcie_suspend(void);
extern int spear_pcie_resume(void);
#endif

#define PCM_SET_WAKEUP_CFG	0xfffff
/* Wake up Configurations */
#define PCIE_WKUP	0x20
#define ETH_WKUP	0x10
#define RTC_WKUP	0x8
#define GPIO_WKUP	0x4
#define USB_WKUP	0x2
#define RAS_WKUP	0x1
#define PWR_DOM_ON	0x3c00
#define PWR_DOM_ON_1310	0xf000
/* DDR PHY low power config macros */
#define DDR_PHY_NO_SHUTOFF_CFG_1310	(~BIT(22))
#define DDR_PHY_NO_SHUTOFF_CFG	(~BIT(20))
#define SWITCH_CTR_CFG	0xff
#define MPMC_REG_CNT 208

/* SRAM related defines for Always on SRAM*/
#define SRAM_START_PA	SYSRAM1_BASE
#define SRAM_START_VA	VA_SYSRAM1_BASE
#define SRAM_LIMIT_VA	(SRAM_START_VA + SZ_4K - 16)

/* Stack Address Space on SRAM to store MPMC configuration */
#define SRAM_STACK_STRT_OFF	0x800
#define SRAM_STACK_PA	(SRAM_START_PA + SRAM_STACK_STRT_OFF)
#define SRAM_STACK_VA	(SRAM_START_VA + SRAM_STACK_STRT_OFF)

/* Scratch offset location to get the core-1 up */
#define SRAM_SCRATCH_PA		(PA_SYS_LOCATION)

/* SPEAr subsystem physical addresses for MISC regsiters*/
#define PCM_CFG			(MISC_BASE + 0x100)
#define VA_PCM_CFG		(VA_MISC_BASE + 0x100)
#define VA_PCM_WKUP_CFG		(VA_MISC_BASE + 0x104)
#define VA_SWITCH_CTR		(VA_MISC_BASE + 0x108)


#define DISABLE_I_C_M_V	0x1805
#define MISC_PLL_OFFS	0x214
#define MPMC_REG_END	0xff0
#define SRAM_SCR_REG	0xffc
#define PLL_VAL1	0x060a
#define PLL_VAL2	0x060e
#define PLL_VAL3	0x0606

#define	MODE_IRQ_32	0x12
#define	MODE_SVC_32	0x13
#define	MODE_ABT_32	0x17
#define	MODE_UND_32	0x1B
#define	MODE_SYS_32	0x1F
#define	MODE_BITS	0x1F

#ifdef __ASSEMBLER__
.macro	io_v2p, pa, va, tmp
	ldr	\tmp, =0xfff
	bic	\pa, \va, \tmp

	/*
	 * Following code uses VA to PA Translation Registers to
	 * translate the virtual address provided by a general-purpose
	 * register and store the corresponding physical address in the
	 * PA Register.
	 */
	mcr	p15, 0, \pa, c7, c8, 1
	mrc	p15, 0, \pa, c7, c4, 0
	bic	\pa, \pa, \tmp
	and	\tmp, \va, \tmp
	orr	\pa, \pa, \tmp

.endm

.macro	io_p2v, pa, va, tmp

	ldr	\tmp, =0xfff00000
	and	\va, \pa, \tmp
	lsr	\va, \va, #4

	ldr	\tmp, =0xffff
	and	\tmp, \pa, \tmp
	orr	\va, \va, \tmp

	ldr	\tmp, =0xf0000000
	orr	\va, \va, \tmp
.endm

.macro disable_mmu, rc, tmp
	mrc	p15, 0, \rc, c1, c0, 0
	ldr	\tmp, =DISABLE_I_C_M_V
	bic	\rc, \rc, \tmp
	mcr	p15, 0, \rc, c1, c0, 0
.endm

.macro ddr_in_srefresh, rc, misc_b, mpmc_b , misc_off
	/* Program MPMC Control Status register in Misc Space */
	ldr	\rc, [\mpmc_b, #0x2C]
	/* Set srefresh_enter bit(2) */
	orr	\rc, \rc, #0x10000
	str	\rc, [\mpmc_b, #0x2c]
wait_till_srefresh_on_r0:
	ldr	\rc, [\misc_b, \misc_off]
	/* check for cke_status bit(13) */
	tst	\rc, #0x2000
	bne	wait_till_srefresh_on_r0
.endm

.macro system_slow_mode, rc, misc_b
	/* Put the system in slow mode */
	ldr	\rc, [\misc_b, #0x200]
	bic	\rc, \rc, #0x7
	/* Set the apt mode bits(2:0) in SCCTRL register */
	orr	\rc, \rc, #0x2
	str	\rc, [\misc_b, #0x200]	/* System is now in slow mode */
wait_till_slow_mode:
	ldr	\rc, [\misc_b, #0x200]
	/* Wait for the mode to be updated */
	and	\rc, \rc, #0xF0000
	/* Poll the SCCTRL register status bits (6:3) */
	cmp	\rc, #0xA0000
	bne wait_till_slow_mode

.endm

.macro switch_off_sys_plls, rc, cnt, misc_b, pll_off
	/*
	 * Put the all the system pll's to off state
	 * The loop of count 3 is provided below to
	 * switch off the pll-1/2/3.
	 * tmp contains the offset for the pll control
	 * registers in the misc space.
	 * DDR pll-4 requires different processing.
	 */
	ldr	\pll_off, =MISC_PLL_OFFS
	ldr	\cnt, =0x0	/* PLL Counter 1, 2, 3, 4 */
swoff_pll:
	ldr	\rc, [\misc_b, \pll_off]
	/* Clear pll_enable bit(1) of PLL1_CTR register in Misc registers */
	bic	\rc, \rc, #0x04
	str	\rc, [\misc_b, \pll_off]
	add	\pll_off, #0xc
	add	\cnt, #0x1
	cmp	\cnt, #0x3	/* Switch off pll-1/2/3 */
	bne	swoff_pll

	/* Switch off pll-4 */
	ldr	\rc, [\misc_b, \pll_off]
	/* Clear pll_enable bit(2) of PLL1_CTR register in Misc registers */
	bic	\rc, \rc, #0x04
	str	\rc, [\misc_b, \pll_off]
.endm

.macro switch_on_pll4, rc, misc_b
	/* Switch on PLL-4, strobe the pll also */
	ldr     \rc, [\misc_b, #0x238]
	ldr	\rc, =PLL_VAL1
	str	\rc, [\misc_b, #0x238]
	ldr	\rc, =PLL_VAL2
	str	\rc, [\misc_b, #0x238]
	ldr	\rc, =PLL_VAL3
	str	\rc, [\misc_b, #0x238]
	ldr	\rc, =PLL_VAL2
	str	\rc, [\misc_b, #0x238]
pll_lock_4:
	/* Set the pll_lock bit(0) in PLLX_CTR register in misc space*/
	ldr	\rc, [\misc_b, #0x238]
	and	\rc, \rc, #0x1
	/* Wait for pll lock status */
	cmp	\rc, #0x1
	bne	pll_lock_4
.endm

.macro switch_on_pll_1_to_3, rc, cnt, misc_b, pll_off
	/* Switch on Pll 1/2/3 */
	ldr	\cnt, =0x0	/* PLL Counter 1, 2, 3, 4 */
swon_pll_1_3:
	/* Switch on Pll-1/2/3 */
	ldr	\rc, [\misc_b, \pll_off]
	orr	\rc, \rc, #0x6
	str	\rc, [\misc_b, \pll_off]
pll_lock_1_3:
	/* Set the pll_lock bit(0) in PLLX_CTR register in misc space*/
	ldr	\rc, [\misc_b, \pll_off]
	and	\rc, \rc, #0x1
	/* Wait for pll lock status */
	cmp	\rc, #0x1
	bne	pll_lock_1_3

	/* Loop for all the pll's */
	add	\pll_off, #0xc
	add	\cnt, #0x1
	cmp	\cnt, #0x3	/* Switch on till pll-3 */
	bne	swon_pll_1_3
.endm

.macro system_normal_mode, rc, misc_b
	/* Put the system in normal mode */
	ldr	\rc, [\misc_b, #0x200]
	bic	\rc, \rc, #0x7
	/* Set the apt mode bits(2:0) in SCCTRL register */
	orr	\rc, \rc, #0x4
	str	\rc, [\misc_b, #0x200]	/* System is now in normal mode */
wait_till_normal_mode:
	ldr	\rc, [\misc_b, #0x200]
	/* Wait for the mode to be updated */
	and	\rc, \rc, #0xF0000
	/* Poll the SCCTRL register status bits (6:3) */
	cmp	\rc, #0xf0000
	bne wait_till_normal_mode
.endm

.macro mpmc_restore_regs, cnt, rc, sram_strt, mpmc_b, off
	mov	\cnt, #0
restore_regs:
	ldr	\rc, [\sram_strt, \off]
	cmp	\cnt, #0x2c
	bne	mpmc_nostart
	nop
	nop
	/* Avoid MPMC restart */
	bic	\rc,\rc, #0x1000000
mpmc_nostart:
	str	\rc, [\mpmc_b, \cnt]
	sub	\off, \off, #4
	add	\cnt, \cnt, #4
	cmp	\cnt, #0x340
	bne	restore_regs
	dsb
	isb

.endm	
.macro uart_restore, rc, tmp, ra
	/* Reprogram UART0 before wakeup */
	ldr	\ra, =UART_BASE

	ldr	\rc, [\ra, #0x024]
	orr	\rc, \rc, #0x1A
	str	\rc, [\ra, #0x024]

	ldr	\rc, [\ra, #0x028]
	orr	\rc, \rc, #0x3
	str	\rc, [\ra, #0x028]

	ldr	\rc, [\ra, #0x02C]
	orr	\rc, \rc, #0x70
	str	\rc, [\ra, #0x02C]

	ldr	\rc, [\ra, #0x030]
	ldr	\tmp, =0xF01
	orr	\rc, \rc, \tmp
	str	\rc, [\ra, #0x030]

	ldr	\rc, [\ra, #0x034]
	orr	\rc, \rc, #0x12
	str	\rc, [\ra, #0x034]

	ldr	\rc, [\ra, #0x038]
	orr	\rc, \rc, #0x50
	str	\rc, [\ra, #0x038]
.endm
#endif
#endif /* __MACH_SUSPEND_H */
