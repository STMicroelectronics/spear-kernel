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
#include <mach/suspend.h>

#define DDR_PLL_SREFRESH
#define MPMC_RESET
#define PWRDOMAINS_ON
#define GPIO_DDR_ISOLATE

#define DISABLE_I_C_M_V	0x1805
#define MISC_PLL_OFFS	0x214
#define MPMC_REG_END	0xffc
#define PLL_VAL1	0x060a
#define PLL_VAL2	0x060e
#define PLL_VAL3	0x0606
#define ASSOC_MASK	0x3ff
#define NUMSET_MASK	0x7fff

#define	MODE_IRQ_32	0x12
#define	MODE_SVC_32	0x13
#define	MODE_ABT_32	0x17
#define	MODE_UND_32	0x1B
#define	MODE_SYS_32	0x1F
#define	MODE_BITS	0x1F

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

.text
ENTRY(spear_wakeup)
	mrc	p15, 0, r0, c0, c0, 5
	and	r0, r0, #0xf
	cmp	r0, #0
	bne	cpux_sleep
back:
//;	b	back
	bl	cache_invalidate
	nop
	adr	r0, spear_sleep_restore
	bx	r0
cpux_sleep:
	/* Core-2 executed this code */
//;	b	cpux_sleep
	ldr	r7, =SRAM_SCRATCH_PA
	ldr	r1, =0xFFFFFFFF
	str	r1, [r7]
	dsb
check:
	wfe
	ldr	r1, [r7]
	tst	r1, #3
	bne	check
	bl	cache_invalidate

	ldr	r7, =SRAM_SCRATCH_PA
	ldr	r1, [r7]
	bx	r1

/* L1 invalidate */
cache_invalidate:
	/* Data Cache Invalidate */
	/* read clidr */
	mrc	p15, 1, r0, c0, c0, 1
	/* extract loc from clidr */
	ands	r3, r0, #0x7000000
	/* left align loc bit field */
	mov	r3, r3, lsr #23
	/* if loc is 0, then no need to clean */
	beq	finished
	/* start clean at cache level 0 */
	mov	r10, #0
loop1:
	/* work out 3x current cache level */
	add	r2, r10, r10, lsr #1
	/* extract cache type bits from clidr*/
	mov	r1, r0, lsr r2
	/* mask of the bits for current cache only */
	and	r1, r1, #7
	/* see what cache we have at this level */
	cmp	r1, #2
	/* skip if no cache, or just i-cache */
	blt	skip
	/* select current cache level in cssr */
	mcr	p15, 2, r10, c0, c0, 0
	/* isb to sych the new cssr&csidr */
	isb
	/* read the new csidr */
	mrc	p15, 1, r1, c0, c0, 0
	/* extract the length of the cache lines */
	and	r2, r1, #7
	/* add 4 (line length offset) */
	add	r2, r2, #4
	ldr	r4, =ASSOC_MASK
	/* find maximum number on the way size */
	ands	r4, r4, r1, lsr #3
	/* find bit position of way size increment */
	clz	r5, r4
	ldr	r7, =NUMSET_MASK
	/* extract max number of the index size*/
	ands	r7, r7, r1, lsr #13
loop2:
	mov	r9, r4
	/* create working copy of max way size*/
loop3:
	/* factor way and cache number into r11 */
	orr	r11, r10, r9, lsl r5
	/* factor index number into r11 */
	orr	r11, r11, r7, lsl r2
	/*clean & invalidate by set/way */
	mcr	p15, 0, r11, c7, c6, 2
	/* decrement the way*/
	subs	r9, r9, #1
	bge	loop3
	/*decrement the index */
	subs	r7, r7, #1
	bge	loop2
skip:
	add	r10, r10, #2
	/* increment cache number */
	cmp	r3, r10
	bgt	loop1
finished:
	/* Switch back to cache level 0 */
	mov	r10, #0
	/* select current cache level in cssr */
	mcr	p15, 2, r10, c0, c0, 0
	dsb
	isb
	mov	r0, #0
	/* L1 Invalidation complete */

	/* Disable SMP, prefetch */
	mcr	p15, 0, r0, c1, c0, 1
	dsb
	/* Instruction cache Invalidate */
	mcr	p15, 0, r0, c7, c5, 0
	/* Invalidate branch pred array */
	mcr	p15, 0, r0, c7, c5, 6
	/* Invalidate unified TLB */
	mcr	p15, 0, r0, c8, c7, 0
	mcr	p15, 0, r0, c8, c5, 0
	mcr	p15, 0, r0, c8, c6, 0
	/* memory barrier to sync up the things */
	mcr	p15, 0, r0, c7, c10, 4
	dsb
	isb
	mov	r0, #0x1800
	mcr	p15, 0, r0, c1, c0, 0
	dsb
	isb

	bx	lr
	.ltorg
	.align

ENTRY(spear_wakeup_sz)
	.word	. - spear_wakeup
	.text
/*
 * spear_sleep_mode()
 * Forces SPEAr into sleep
 * r0: state
 * r1: revision:
 *	- 0 for spear13xx devices
 *	- 1 for spear1310 and spear1340 devices
 */
ENTRY(spear_sleep_mode)
	mov	r10, r1		/* latch the address of cpu_resume */
	mov	r11, r0		/* latch the suspend state */
	mov	r12, r2		/* latch the soc revision */

	ldr	r8, =IO_ADDRESS(SPEAR_SRAM_STACK_PA)

	/* latch the soc type on SRAM */
	stmia	r8!, {r12}
	/* Store Context ID, User R/W thread, secure or NS VBAR */
	mrc	p15, 0, r2, c10, c2, 0
	mrc	p15, 0, r3, c13, c0, 1
	mrc	p15, 0, r4, c13, c1, 2
	mrc	p15, 0, r5, c13, c2, 3
	mrc	p15, 0, r6, c13, c3, 4
	mrc	p15, 0, r7, c12, c0, 0
	stmia	r8!, {r2-r7}

	/* Store the Current CPSR */
	mrs	r0, cpsr
	stmia	r8!, {r0}

	/* Move to IRQ Mode */
	mrs	r0, cpsr
	bic	r0, r0, #MODE_BITS
	orr	r1, r0, #(MODE_IRQ_32)
	msr	cpsr_c, r1
	mrs	r1, spsr

	/* Saving FIQ stack pointer on to Stack in DDR */
	stmia	r8!, {r1, r13, r14}

	/* Move to svc mode */
	mrs	r0, cpsr
	bic	r0, r0, #MODE_BITS
	orr	r1, r0, #(MODE_SVC_32)
	msr	cpsr_c, r1
	mrs	r1, spsr
	stmia	r8!, {r1, r13, r14}	/* Saving IRQ stack pointer */

	/* Move to ABT mode */
	mrs	r0, cpsr
	bic	r0, r0, #MODE_BITS
	orr	r1, r0, #(MODE_ABT_32)
	msr	cpsr_c, r1
	mrs	r1, spsr
	stmia	r8!, {r1, r13, r14}	/* Saving IRQ stack pointer */

	/* Move to undefined mode */
	mrs	r0, cpsr
	bic	r0, r0, #MODE_BITS
	orr	r1, r0, #(MODE_UND_32)
	msr	cpsr_c, r1
	mrs	r1, spsr
	stmia	r8!, {r1, r13, r14}	/* Saving IRQ stack pointer */

	/* Move to sys_32 mode */
	mrs	r0, cpsr
	bic	r0, r0, #MODE_BITS
	orr	r1, r0, #(MODE_SYS_32)
	msr	cpsr_c, r1
	mrs	r1, spsr
	stmia	r8!, {r1, r13, r14}	/* Saving IRQ stack pointer */

	/* Move back to svc mode */
	mrs	r0, cpsr
	bic	r0, r0, #MODE_BITS
	orr	r1, r0, #(MODE_SVC_32)
	msr	cpsr_c, r1

	/* Latch the physical address of cpu_resume on to SRAM */
//;	mov	r0, r10
//;	sub	r0, r0, #0x80000000
	io_v2p	r0, r10, r1
	stmia	r8!, {r0}

	/* Extract the physical address to jump to */
	adr	r0, mmu_off
	io_v2p	r2, r0, r1

	/* Disable MMU */
	mrc	p15, 0, r0, c1, c0, 0
	ldr	r1, =DISABLE_I_C_M_V
	bic	r0, r0, r1
	mcr	p15, 0, r0, c1, c0, 0
	/* Move the Physical address into PC */
	bx	r2
	nop
mmu_off:
	/* Put the DDR in self refresh mode */
	ldr	r6, =MISC_BASE_PA
	/* Check for the SPEAr revision */
	cmp	r12, #1
	bne	srefresh_enter_rev_0

	/* This is Rev-1 */
	/* Program MPMC Control Status register in Misc Space */
	ldr	r0, [r6, #0x438]
	/* Set srefresh_enter bit(2) */
	orr	r0, r0, #0x4
	str	r0, [r6, #0x438]
wait_till_srefresh_on_r1:
	ldr	r0, [r6, #0x438]
	/* check for cke_status bit(13) */
	tst	r0, #0x2000
	bne	wait_till_srefresh_on_r1
	b	srefresh_on

	/* This is Rev-0 */
srefresh_enter_rev_0:
	/* Program MPMC Control Status register in Misc Space */
	ldr	r0, [r6, #0x334]
	/* Set srefresh_enter bit(2) */
	orr	r0, r0, #0x4
	str	r0, [r6, #0x334]
wait_till_srefresh_on_r0:
	ldr	r0, [r6, #0x334]
	/* check for cke_status bit(13) */
	tst	r0, #0x2000
	beq	wait_till_srefresh_on_r0

srefresh_on:
#ifdef MPMC_RESET
	ldr	r7, =MPMC_BASE_PA
	/* Stop the MPMC controller */
	ldr	r0, [r7, #0x2c]
	bic	r0, r0, #0x1000000
	str	r0, [r7, #0x2c]

	/* reset the MPMC controller */
	ldr	r1, [r6, #0x280]
	orr	r1, r1, #0x3
	str	r1, [r6, #0x280]

	/* Stop the MPMC clock */
	ldr	r1, [r6, #0x278]
	bic	r1, r1, #0x3
	str	r1, [r6, #0x278]
#endif
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
	ldr	r1, =MISC_PLL_OFFS
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
	/* Change the source clock to pll-1 */
	ldr	r0, [r6, #0x244]
	bic	r0, r0, #0x0400
	str	r0, [r6, #0x244]
#endif
#ifdef GPIO_DDR_ISOLATE
	/* enable gpio clock */
	ldr	r0, [r6, #0x274]
	orr	r0, r0, #0x00800000
	str	r0, [r6, #0x274]

	ldr	r6, =GPIO_START_PA
	/* Set the Direction of GPIO-2 as output */
	ldr	r0, [r6, #0x400]
	orr	r0, r0, #0xc
	str	r0, [r6, #0x400]

	/* Set the value of GPIO-2/3 */
	ldr	r0, [r6, #0x3fc]
	orr	r0, r0, #0x4
	str	r0, [r6, #0x3fc]

#endif
#ifdef PWRDOMAINS_ON
	cmp	r11, #0x1
	beq	standby_mode
	/* Switch on certan sections as suggested by HW team */
	/* Let the USB Host clock be on */
	ldr	r6, =MISC_BASE_PA
	ldr	r0, [r6, #0x274]
	orr	r0, r0, #0xE00
	str	r0, [r6, #0x274]
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

standby_mode:
	wfi				@ wait for interrupt
	nop
spear_sleep_restore:
	nop
	ldr	r6, =MISC_BASE_PA
	ldr	r1, =MISC_PLL_OFFS

#ifdef DDR_PLL_SREFRESH
	/* Switch on PLL-4, strobe the pll also */
	ldr	r0, [r6, #0x238]
	ldr	r0, =PLL_VAL1
	str	r0, [r6, #0x238]
	ldr	r0, =PLL_VAL2
	str	r0, [r6, #0x238]
	ldr	r0, =PLL_VAL3
	str	r0, [r6, #0x238]
	ldr	r0, =PLL_VAL2
	str	r0, [r6, #0x238]
pll_lock_4:
	/* Set the pll_lock bit(0) in PLLX_CTR register in misc space*/
	ldr	r5, [r6, #0x238]
	and	r5, r5, #0x1
	/* Wait for pll lock status */
	cmp	r5, #0x1
	bne	pll_lock_4

	/* Switch the pll source back to pll-4 */
	ldr	r0, [r6, #0x244]
	orr	r0, r0, #0x0400
	str	r0, [r6, #0x244]
#endif

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
	ldr	r6, =SPEAR_SRAM_START_PA
#ifdef MPMC_RESET
	ldr	r7, =MISC_BASE_PA
	/* Enable the MPMC clock */
	ldr	r1, [r7, #0x278]
	orr	r1, r1, #0x3
	str	r1, [r7, #0x278]
	/* Provide Software Reset to MPMC */
	ldr	r1, [r7, #0x280]
	orr	r1, r1, #0x3
	str	r1, [r7, #0x280]
	/* Enable MPMC back */
	bic	r1, r1, #0x3
	str	r1, [r7, #0x280]
#endif
	/* Restore the MPMC registers */
	ldr	r7, =MPMC_BASE_PA
	mov	r0, #0
	ldr	r3, =MPMC_REG_END
mpmc_restore_regs:
	ldr	r1, [r6, r3]
	cmp	r0, #0x2c
	bne	ddf
	nop
	nop
	bic	r1, r1, #0x1000000
ddf:
	str	r1, [r7, r0]
	sub	r3, r3, #4
	add	r0, r0, #4
	cmp	r0, #0x320
	bne	mpmc_restore_regs
	/* Enable MPMC */
	ldr	r1, [r7, #0x2c]
	orr	r1, r1, #0x1000000
	str	r1, [r7, #0x2c]
	dsb
	isb
	mov	r0, #0
	mcr	p15, 0, r0, c7, c5, 4	@ Flush prefetch buffer
	mcr	p15, 0, r0, c7, c5, 6	@ Invalidate branch predictor array
	mcr	p15, 0, r0, c8, c5, 0	@ Invalidate instruction TLB
	mcr	p15, 0, r0, c8, c6, 0	@ Invalidate data TLB

#ifdef GPIO_DDR_ISOLATE
	ldr	r7, =GPIO_START_PA
	/* Set the Direction of GPIO-2 as output */
	ldr	r0, [r7, #0x400]
	orr	r0, r0, #0xc
	str	r0, [r7, #0x400]

	/* Set the value of GPIO-2 */
	ldr	r0, [r7, #0x3fc]
	bic	r0, r0, #0x4
	str	r0, [r7, #0x3fc]
#endif
	/* Move the DDR out of self refresh mode */
	ldr	r6, =MISC_BASE_PA
	ldr	r7, =MPMC_BASE_PA
	ldr	r8, =SPEAR_SRAM_STACK_PA
	/* fetch the SOC revision version from stack */
	ldmia	r8!, {r12}

	/* Clearance is required in the mpmc space */
	ldr	r1, [r7, #0x2c]
	/*
	 * Clear bit srefresh bit (2) of MPMC_11 register
	 * The misc wrapper does not works fine by itself till
	 * this bit is also cleared.
	 */
	bic	r1, r1, #0x10000

	/* Check for the SPEAr revision */
	cmp	r12, #1
	bne	srefresh_exit_rev_0

	/* This is Rev-1 */
	ldr	r0, [r6, #0x438]
	/* Clear srefresh_enter bit(2) */
	bic	r0, r0, #0x4
	str	r0, [r6, #0x438]
	str	r1, [r7, #0x2c]
wait_for_srefresh_exit_r1:
	ldr	r0, [r6, #0x438]
	tst	r0, #0x2000
	beq	wait_for_srefresh_exit_r1
	b	srefresh_off

	/* This is Rev-0 */
srefresh_exit_rev_0:
	/* Program MPMC Control Status register in Misc Space */
	ldr	r0, [r6, #0x334]
	/* Clear srefresh_enter bit(2) */
	bic	r0, r0, #0x4
	str	r0, [r6, #0x334]
	str	r1, [r7, #0x2c]
wait_for_srefresh_exit_r0:
	ldr	r0, [r6, #0x334]
	tst	r0, #0x2000
	bne	wait_for_srefresh_exit_r0

srefresh_off:
	/* Store Context ID, User R/W thread, secure or NS VBAR */
	ldmia	r8!, {r2-r7}
	mcr	p15, 0, r2, c10, c2, 0
	mcr	p15, 0, r3, c13, c0, 1
	mcr	p15, 0, r4, c13, c1, 2
	mcr	p15, 0, r5, c13, c2, 3
	mcr	p15, 0, r6, c13, c3, 4
	mcr	p15, 0, r7, c12, c0, 0

	ldmia	r8!, {r4}
	msr	cpsr, r4
	mov	r0, r4

	/* Move to IRQ mode */
	mrs	r0, cpsr
	bic	r0, r0, #MODE_BITS
	orr	r1, r0, #(MODE_IRQ_32)
	msr	cpsr_c, r1
	ldmia	r8!, {r1, r13, r14}
	msr	spsr, r1

	/* Move to svc mode */
	mrs	r0, cpsr
	bic	r0, r0, #MODE_BITS
	orr	r1, r0, #(MODE_SVC_32)
	msr	cpsr_c, r1
	ldmia	r8!, {r1, r13, r14}
	msr	spsr, r1

	/* Move to ABT mode */
	mrs	r0, cpsr
	bic	r0, r0, #MODE_BITS
	orr	r1, r0, #(MODE_ABT_32)
	msr	cpsr_c, r1
	ldmia	r8!, {r1, r13, r14}
	msr	spsr, r1

	/* Move to undefined mode */
	mrs	r0, cpsr
	bic	r0, r0, #MODE_BITS
	orr	r1, r0, #(MODE_UND_32)
	msr	cpsr_c, r1
	ldmia	r8!, {r1, r13, r14}
	msr	spsr, r1

	/* Move to sys_32 mode */
	orr	r1, r0, #(MODE_SYS_32)
	msr	cpsr_c, r1
	ldmia	r8!, {r0, r13, r14}
	msr	spsr_cxsf, r0

	msr	cpsr, r4
	ldmia	r8!, {r0}
	mov	pc, r0
	nop
	.ltorg
	.align
ENTRY(spear_sleep_mode_sz)
	.word	. - spear_sleep_mode
