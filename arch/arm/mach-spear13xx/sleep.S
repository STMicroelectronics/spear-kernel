/*
 * linux/arch/arm/mach-spear13xx/sleep.S
 *
 * SPEAR13xx specific functions that will run in internal SRAM.
 * The functions are used in power management.
 *
 * Copyright (C) 2010 ST MicroElectronics
 * Deepak Sikri <deepak.sikri@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/linkage.h>
#include <asm/assembler.h>
#include <mach/hardware.h>
#include <mach/suspend.h>

/* #define DDR_PLL_SREFRESH */
/* #define TEST_PWRDOMAINS */
.text
ENTRY(spear_wakeup)

spear_wakeup:
	b	spear_wakeup
	adr	r0, spear_sleep_restore
	bx	r0

ENTRY(spear_wakeup_sz)
	.word	. - spear_wakeup
/*
 * spear_sleep_mode()
 * Forces SPEAr into sleep
 */
ENTRY(spear_sleep_mode)
	stmfd	sp!, {r0-r12, lr}		@ save registers on stack
	/* Store Stack address in r8 */
	ldr	r8, SRAM_STACK_VA

	/* Store sp and spsr to SDRAM */
	mov	r4, sp
	mrs	r5, spsr
	mov	r6, lr
	stmia	r8!, {r4-r6}

	/* Save all ARM registers */
	/* Coprocessor access control register */
	mrc	p15, 0, r6, c1, c0, 2
	stmia	r8!, {r6}
	/* TTBR0, TTBR1 and Translation table base control */
	mrc	p15, 0, r4, c2, c0, 0
	mrc	p15, 0, r5, c2, c0, 1
	mrc	p15, 0, r6, c2, c0, 2
	stmia	r8!, {r4-r6}
	/*
	 * Domain access control register, data fault status register,
	 * and instruction fault status register
	 */
	mrc	p15, 0, r4, c3, c0, 0
	mrc	p15, 0, r5, c5, c0, 0
	mrc	p15, 0, r6, c5, c0, 1
	stmia	r8!, {r4-r6}
	/*
	 * Data aux fault status register, instruction aux fault status,
	 * data fault address register and instruction fault address register
	 */
	mrc	p15, 0, r4, c5, c1, 0
	mrc	p15, 0, r5, c5, c1, 1
	mrc	p15, 0, r6, c6, c0, 0
	mrc	p15, 0, r7, c6, c0, 2
	stmia	r8!, {r4-r7}
	/*
	 * user r/w thread and process ID, user r/o thread and process ID,
	 * priv only thread and process ID, cache size selection
	 */
	mrc	p15, 0, r4, c13, c0, 2
	mrc	p15, 0, r5, c13, c0, 3
	mrc	p15, 0, r6, c13, c0, 4
	mrc	p15, 2, r7, c0, c0, 0
	stmia	r8!, {r4-r7}
	/* Data TLB lockdown, instruction TLB lockdown registers */
	mrc	p15, 0, r5, c10, c0, 0
	mrc	p15, 0, r6, c10, c0, 1
	stmia	r8!, {r5-r6}
	/* Secure or non secure vector base address, FCSE PID, Context PID*/
	mrc	p15, 0, r4, c12, c0, 0
	mrc	p15, 0, r5, c13, c0, 0
	mrc	p15, 0, r6, c13, c0, 1
	stmia	r8!, {r4-r6}
	/* Primary remap, normal remap registers */
	mrc	p15, 0, r4, c10, c2, 0
	mrc	p15, 0, r5, c10, c2, 1
	stmia	r8!, {r4-r5}
	/* Store current cpsr*/
	mrs	r2, cpsr
	stmia	r8!, {r2}
	mrc	p15, 0, r4, c1, c0, 0
	/* save control register */
	stmia	r8!, {r4}
	/* Data memory barrier and Data sync barrier */
	mov	r1, #0
	mcr	p15, 0, r1, c7, c10, 4
	mcr	p15, 0, r1, c7, c10, 5
	dsb
	isb
	/* Extract the physical address to jump to */
	adr	r0, mmu_off
	mov	r1, #0xcfffffff
	and	r0, r0, r1
	ldr	r1, =0x20000000
	orr	r0, r0, r1
	mov	r2, r0

	/* Disable MMU */
	mrc	p15, 0, r0, c1, c0, 0
	ldr	r1, DISABLE_I_C_M_V
	bic	r0, r0, r1
	mcr	p15, 0, r0, c1, c0, 0
	/* Move the Physical address into PC */
	bx	r2
	nop
mmu_off:
	/* Put the DDR in self refresh mode */
	ldr	r6, MISC_BASE_P
	/* Program MPMC Control Status register in Misc Space */
	ldr	r0, [r6, #0x334]
	/* Set srefresh_enter bit(2) */
	orr	r0, r0, #0x4
	str	r0, [r6, #0x334]
wait_till_srefresh_on:
	ldr	r0, [r6, #0x334]
	/* check for cke_status bit(13) */
	tst	r0, #0x2000
	beq	wait_till_srefresh_on

	/* Put the system in slow mode */
	ldr	r0, [r6, #0x200]
	bic	r0, r0, #0x4
	/* Set the apt mode bits(2:0) in SCCTRL register */
	orr	r0, r0, #0x2
	str	r0, [r6, #0x200]	/* System is now in slow mode */
wait_till_slow_mode:
	ldr	r0, [r6, #0x200]
	/* Wait for the mode to be updated */
	and	r0, r0, #0xF0000
	/* Poll the SCCTRL register status bits (6:3) */
	cmp	r0, #0xA0000
	bne wait_till_slow_mode

	/*
	 * Put the all the system pll's to off state
	 * The loop of count 3 is provided below to
	 * switch off the pll-1/2/3.
	 * r1 contains the offset for the pll control
	 * registers in the misc space.
	 * DDR pll-4 requires different processing.
	 */
	ldr	r1, MISC_PLL_OFFS
	ldr	r2, =0x0	/* PLL Counter 1, 2, 3, 4 */
swoff_pll:
	ldr	r0, [r6, r1]
	/* Clear pll_enable bit(1) of PLL1_CTR register in Misc registers */
	bic	r0, r0, #0x02
	str	r0, [r6, r1]
	add	r1, #0xc
	add	r2, #0x1
	cmp	r2, #0x3	/* Switch off pll-1/2/3 */
	bne	swoff_pll

#ifdef DDR_PLL_SREFRESH
	/* Switch off pll-4 */
	ldr	r0, [r6, r1]
	/* Clear pll_enable bit(2) of PLL1_CTR register in Misc registers */
	bic	r0, r0, #0x04
	str	r0, [r6, r1]
#endif

#ifdef TEST_PWRDOMAINS
	/* Switch off the undesired PLL's */
	nop
	ldr	r6, MISC_BASE_P
	ldr	r0, [r6, #0x200]
	bic	r0, r0, #0x7
	orr	r0, r0, #0x2
	str	r0, [r6, #0x200	]
wait_ack0:
	ldr	r0, [r6, #0x200]
	and	r0, r0, #0xF0000
	cmp	r0, #0xA0000
	bne	wait_ack0
	ldr	r6, MISC_BASE_P
	ldr	r0, [r6, #0x100]

	/*
	 * Switch off the power domains.
	 * Clear the ack bit
	 */
	bic	r0, r0, #0xc000
	str	r0, [r6, #0x100]

	bic	r0, r0, #0x1000
	str	r0, [r6, #0x100]

wait_ack1:
	ldr	r0, [r6, #0x100]
	tst	r0, #0x4000
	beq	wait_ack1

	/* Clear the ack bit */
	bic	r0, r0, #0xc000
	str	r0, [r6, #0x100]

	bic	r0, r0, #0x0800
	str	r0, [r6, #0x100]
wait_ack2:
	ldr	r0, [r6, #0x100]
	tst	r0, #0x4000
	beq	wait_ack2

	/* Clear the ack bit */
	bic	r0, r0, #0xc000
	str	r0, [r6, #0x100]

	bic	r0, r0, #0x2400
	str	r0, [r6, #0x100]
wait_ack3:
	ldr	r0, [r6, #0x100]
	tst	r0, #0x4000
	beq	wait_ack3
#endif
	wfi				@ wait for interrupt
	nop
spear_sleep_restore:
	/*
	 * Reenable the switched off pll's. The Pll's are
	 * enabled using loop count of 4 to activalte all the
	 * pll-1/2/3/4.
	 * The strobing is done for pll-4 only.
	 */

	ldr	r6, MISC_BASE_P
	ldr	r1, MISC_PLL_OFFS
	ldr	r2, =0x0	/* PLL Counter 1, 2, 3, 4 */
swon_pll_1_3:
	/* Switch on Pll-1/2/3 */
	ldr	r0, [r6, r1]
	orr	r0, r0, #0x2
	str	r0, [r6, r1]
pll_lock_1_3:
	/* Set the pll_lock bit(0) in PLLX_CTR register in misc space*/
	ldr	r5, [r6, r1]
	and	r5, r5, #0x1
	/* Wait for pll lock status */
	cmp	r5, #0x1
	bne	pll_lock_1_3

	/* Loop for all the pll's */
	add	r1, #0xc
	add	r2, #0x1
	cmp	r2, #0x3	/* Switch on till pll-3 */
	bne	swon_pll_1_3

#ifdef DDR_PLL_SREFRESH
	/* Switch on PLL-4, strobe the pll also */
	ldr	r0, [r6, r1]
	ldr	r0, PLL_VAL1
	str	r0, [r6, r1]
	ldr	r0, PLL_VAL2
	str	r0, [r6, r1]
	ldr	r0, PLL_VAL3
	str	r0, [r6, r1]
	ldr	r0, PLL_VAL2
	str	r0, [r6, r1]
pll_lock_4:
	/* Set the pll_lock bit(0) in PLLX_CTR register in misc space*/
	ldr	r5, [r6, r1]
	and	r5, r5, #0x1
	/* Wait for pll lock status */
	cmp	r5, #0x1
	bne	pll_lock_4
#endif

	/* Put the system in normal mode */
	ldr	r0, [r6, #0x200]
	bic	r0, r0, #0x7
	/* Set the apt mode bits(2:0) in SCCTRL register */
	orr	r0, r0, #0x4
	str	r0, [r6, #0x200]	/* System is now in slow mode */
wait_till_normal_mode:
	ldr	r0, [r6, #0x200]
	/* Wait for the mode to be updated */
	and	r0, r0, #0xF0000
	/* Poll the SCCTRL register status bits (6:3) */
	cmp	r0, #0xf0000
	bne wait_till_normal_mode

	/*
	 * Invalidate all instruction caches to PoU
	 * and flush branch target cache
	 */
	mov	r1, #0
	mcr	p15, 0, r1, c7, c5, 0

	ldr	r3, SRAM_STACK_PA
	ldmia	r3!, {r4-r6}
	mov	sp, r4
	msr	spsr_cxsf, r5
	mov	lr, r6

	ldmia	r3!, {r4-r9}
	/* Coprocessor access Control Register */
	mcr	p15, 0, r4, c1, c0, 2

	/* TTBR0 */
	mcr	p15, 0, r5, c2, c0, 0
	/* TTBR1 */
	mcr	p15, 0, r6, c2, c0, 1
	/* Translation table base control register */
	mcr	p15, 0, r7, c2, c0, 2
	/*domain access Control Register */
	mcr	p15, 0, r8, c3, c0, 0
	/* data fault status Register */
	mcr	p15, 0, r9, c5, c0, 0

	ldmia	r3!, {r4-r8}
	/* instruction fault status Register */
	mcr	p15, 0, r4, c5, c0, 1
	/*Data Auxiliary Fault Status Register */
	mcr	p15, 0, r5, c5, c1, 0
	/*Instruction Auxiliary Fault Status Register*/
	mcr	p15, 0, r6, c5, c1, 1
	/*Data Fault Address Register */
	mcr	p15, 0, r7, c6, c0, 0
	/*Instruction Fault Address Register*/
	mcr	p15, 0, r8, c6, c0, 2
	ldmia	r3!, {r4-r7}

	/* user r/w thread and process ID */
	mcr	p15, 0, r4, c13, c0, 2
	/* user ro thread and process ID */
	mcr	p15, 0, r5, c13, c0, 3
	/*Privileged only thread and process ID */
	mcr	p15, 0, r6, c13, c0, 4
	/* cache size selection */
	mcr	p15, 2, r7, c0, c0, 0
	ldmia	r3!, {r4-r8}
	/* Data TLB lockdown registers */
	mcr	p15, 0, r4, c10, c0, 0
	/* Instruction TLB lockdown registers */
	mcr	p15, 0, r5, c10, c0, 1
	/* Secure or Nonsecure Vector Base Address */
	mcr	p15, 0, r6, c12, c0, 0
	/* FCSE PID */
	mcr	p15, 0, r7, c13, c0, 0
	/* Context PID */
	mcr	p15, 0, r8, c13, c0, 1

	ldmia	r3!, {r4-r5}
	/* primary memory remap register */
	mcr	p15, 0, r4, c10, c2, 0
	/*normal memory remap register */
	mcr	p15, 0, r5, c10, c2, 1

	/* Restore cpsr */
	ldmfd	r3!, {r4}	/*load CPSR from SDRAM*/
	msr	cpsr, r4	/*store cpsr */
	dsb
	isb
	mov	r0, #0
	mcr	p15, 0, r0, c7, c5, 4	@ Flush prefetch buffer
	mcr	p15, 0, r0, c7, c5, 6	@ Invalidate branch predictor array
	mcr	p15, 0, r0, c8, c5, 0	@ Invalidate instruction TLB
	mcr	p15, 0, r0, c8, c6, 0	@ Invalidate data TLB

	adr	r5, mmu_on
	mov	r1, #0xcfffffff
	and	r5, r5, r1
	ldr	r1, =0x30000000
	orr	r5, r5, r1
	mov	r4, r5

	/* Move the DDR out of self refresh mode */
	ldr	r6, MISC_BASE_P
	ldr	r7, MPMC_BASE_P
	/* Program MPMC Control Status register in Misc Space */
	ldr	r0, [r6, #0x334]
	/* Clear srefresh_enter bit(2) */
	bic	r0, r0, #0x4
	str	r0, [r6, #0x334]
	/* Additional clearance is required in the mpmc space */
	ldr	r0, [r7, #0x2c]
	/*
	 * Clear bit srefresh bit (2) of MPMC_11 register
	 * The misc wrapper does not works fine by itself till
	 * this bit is also cleared.
	 */
	bic	r0, r0, #0x10000
	str	r0, [r7, #0x2c]
wait_for_refresh_exit:
	ldr	r0, [r6, #0x334]
	tst	r0, #0x2000
	bne	wait_for_refresh_exit

	ldmfd	r3!, {r2}
	/* restore the MMU control register from stack to enable mmu */
	mcr	p15, 0, r2, c1, c0, 0
	bx	r4

mmu_on:
	ldmfd	sp!, {r0-r12, pc}		@ restore regs and return
	nop

MPMC_BASE_P :
	.word MPMC_BASE_PA
MISC_BASE_P :
	.word MISC_BASE_PA
SRAM_STACK_VA :
	.word IO_ADDRESS(SPEAR_SRAM_STACK_PA)
SRAM_STACK_PA :
	.word SPEAR_SRAM_STACK_PA
DISABLE_I_C_M_V:
	.word 0x1805
MISC_PLL_OFFS:
	.word 0x214
#ifdef DDR_PLL_SREFRESH
PLL_VAL1:
	.word 0x060a
PLL_VAL2:
	.word 0x060e
PLL_VAL3:
	.word 0x0606
#endif
ENTRY(spear_sleep_mode_sz)
	.word	. - spear_sleep_mode
