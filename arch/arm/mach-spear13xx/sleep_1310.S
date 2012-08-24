/*
 * linux/arch/arm/mach-spear13xx/sleep_1310.S
 *
 * SPEAR1310 specific functions that will run in internal SRAM.
 * The functions are used in power management.
 *
 * Copyright (C) 2012 ST MicroElectronics
 * Deepak Sikri <deepak.sikri@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/linkage.h>
#include <asm/assembler.h>
#include <mach/suspend.h>

/*
 * spear1310_sleep_mode()
 * Forces SPEAr into sleep
 */

.text
ENTRY(spear1310_sleep_mode)
	mov	r10, r1		/* latch the address of cpu_resume */
	mov	r11, r0		/* latch the suspend state */

	ldr	r8, =SRAM_STACK_VA
	/* Latch the physical address of cpu_resume on to SRAM */
	io_v2p	r0, r10, r1
	stmia	r8!, {r0}

	/* Extract the physical address to jump to */
	adr	r0, mmu_off
	io_v2p	r2, r0, r1

	/* Disable MMU and Caches */
	disable_mmu	r0, r1
	/* Move the Physical address into PC */
	bx	r2
	nop
mmu_off:
	/* Put the DDR in self refresh mode */
	ldr	r6, =MISC_BASE
	ldr	r7, =A9SM_AND_MPMC_BASE
	/* Program the Status check offset in misc space */
	ldr	r1, =0x3DC
	/* Move DDR into self-refresh */
	ddr_in_srefresh r0, r6, r7, r1

	/* enable gpio clock */
	ldr	r0, [r6, #0x300]
	orr	r0, r0, #0x00800000
	str	r0, [r6, #0x300]

	ldr	r6, =GPIO0_BASE
	/* Set the Direction of GPIO-2 as output */
	ldr	r0, [r6, #0x400]
	orr	r0, r0, #0xc
	str	r0, [r6, #0x400]

	/* Set the value of GPIO-2/3 */
	ldr	r0, [r6, #0x3fc]
	orr	r0, r0, #0xc
	str	r0, [r6, #0x3fc]
	ldr	r6, =MISC_BASE
	/* Stop the MPMC controller */
	ldr	r0, [r7, #0x2c]
	bic	r0, r0, #0x1000000
	str	r0, [r7, #0x2c]

	/* reset the MPMC controller */
	ldr	r1, [r6, #0x30c]
	orr	r1, r1, #0x3
	str	r1, [r6, #0x30c]

	/* Stop the MPMC clock */
	ldr	r1, [r6, #0x304]
	bic	r1, r1, #0x3
	str	r1, [r6, #0x304]

	/* Move the system into slow mode */
	system_slow_mode	r0, r6

	/* Switch off the system pll's */
	switch_off_sys_plls  r0, r2, r6, r1

	/* Switch off Power Domains for s2r */
	cmp	r11, #0x1
	beq	standby_mode
	/* Store the Resume Physical Address on Stack Secure location */
	adr	r0, spear_sleep_restore
	ldr	r3, =SRAM_SCR_REG
	ldr	r6, =SRAM_START_PA
	str	r0, [r6, r3]

	/* Enable bootrom and always on ram clock */
	ldr	r6, =MISC_BASE
	ldr	r0, [r6, #0x300]
	orr	r0, r0, #0x6
	str	r0, [r6, #0x300]
	ldr	r0, [r6, #0x100]
	/*
	 * Switch off the power domains.
	 * Clear the ack bit
	 */
	bic	r0, r0, #0x30000
	str	r0, [r6, #0x100]

	bic	r0, r0, #0x4000
	str	r0, [r6, #0x100]

wait_ack1:
	ldr	r0, [r6, #0x100]
	tst	r0, #0x10000
	beq	wait_ack1

	/* Clear the ack bit */
	bic	r0, r0, #0x30000
	str	r0, [r6, #0x100]

	bic	r0, r0, #0x2000
	str	r0, [r6, #0x100]
wait_ack2:
	ldr	r0, [r6, #0x100]
	tst	r0, #0x10000
	beq	wait_ack2

	/* Clear the ack bit */
	bic	r0, r0, #0x30000
	str	r0, [r6, #0x100]

	bic	r0, r0, #0x9000
	str	r0, [r6, #0x100]
wait_ack3:
	ldr	r0, [r6, #0x100]
	tst	r0, #0x10000
	beq	wait_ack3

standby_mode:
	wfi				@ wait for interrupt
	nop
spear_sleep_restore:
	nop
	ldr	r6, =MISC_BASE
	ldr	r1, =MISC_PLL_OFFS

	/* Switch on Pll 4 */
	switch_on_pll4 r0, r6

	/* Switch on pll-1/2/3 */	
	switch_on_pll_1_to_3 r0, r2, r6, r1

	/* Move the system into normal mode */
	system_normal_mode r0, r6

	ldr	r7, =MISC_BASE
	/* Enable the MPMC clock */
	ldr	r1, [r7, #0x304]
	orr	r1, r1, #0x3
	str	r1, [r7, #0x304]
	/* Provide Software Reset to MPMC */
	ldr	r1, [r7, #0x30c]
	orr	r1, r1, #0x3
	str	r1, [r7, #0x30c]
	/* Enable MPMC back */
	bic	r1, r1, #0x3
	str	r1, [r7, #0x30c]
	/* Restore the MPMC registers */
	ldr	r1, =MPMC_REG_END
	ldr	r6, =SRAM_START_PA
	ldr	r7, =A9SM_AND_MPMC_BASE
	/* Restor the mpmc registers */
	mpmc_restore_regs r0, r2, r6, r7, r1

	ldr	r7, =GPIO0_BASE
	/* Set the Direction of GPIO-2 as output */
	ldr	r0, [r7, #0x400]
	orr	r0, r0, #0xc
	str	r0, [r7, #0x400]

	/* Set the value of GPIO-2 DDR CLKEN*/
	ldr	r0, [r7, #0x3fc]
	bic	r0, r0, #0x7
	str	r0, [r7, #0x3fc]

	mov	r0, #0
ddr_out_srefresh:
	/* Add some delay */
	add	r0, r0, #0x1
	cmp	r0, #0x60000
	bne	ddr_out_srefresh

	/* Move the DDR out of self refresh mode */
	ldr	r7, =A9SM_AND_MPMC_BASE
	ldr	r8, =SRAM_STACK_PA

	/* Exit from SR (bit 8) */
	ldr	r1, [r7, #0x18]
	orr	r1, r1, #0x100
	str	r1, [r7, #0x18]

	/* Set the MPMC controller to exit from Self-Refresh */
	ldr	r0, [r7, #0x2c]
	orr	r0, r0, #0x100
	str	r0, [r7, #0x2c]

	/* Set the MPMC controller to exit from Self-Refresh */
	ldr	r0, [r7, #0x2c]
	orr	r0, r0, #0x1
	str	r0, [r7, #0x2c]

	/* Enable MPMC */
	ldr	r1, [r7, #0x2c]
	orr	r1, r1, #0x1000000
	str	r1, [r7, #0x2c]

	dsb
	isb

	/* Clearance is required in the mpmc space */
	ldr	r1, [r7, #0x2c]
	/*
	 * Clear bit srefresh bit (2) of MPMC_11 register
	 * The misc wrapper does not works fine by itself till
	 * this bit is also cleared.
	 */
	bic	r1, r1, #0x10000

	ldr	r6, =MISC_BASE
	/* Program MPMC Control Status register in Misc Space */
	ldr	r0, [r6, #0x3DC]
	/* Clear srefresh_enter bit(2) */
	bic	r0, r0, #0x4
	str	r0, [r6, #0x3DC]
	str	r1, [r7, #0x2c]

wait_for_srefresh_exit_r0:
	ldr	r0, [r6, #0x3DC]
	tst	r0, #0x2000
	beq	wait_for_srefresh_exit_r0
	ldr	r7, =GPIO0_BASE
	ldr	r0, [r7, #0x3fc]
	bic	r0, r0, #0x8
	str	r0, [r7, #0x3fc]

	mov	r0, #0
ddr_out_srefresh1:
	add	r0, r0, #0x1
	cmp	r0, #0x60000
	bne	ddr_out_srefresh1

	uart_restore r0, r2, r7

	ldmia	r8!, {r0}
	mov	pc, r0
	nop
	.ltorg
	.align
ENTRY(spear1310_sleep_mode_sz)
	.word	. - spear1310_sleep_mode
