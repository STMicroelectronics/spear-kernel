/*
 * linux/arch/arm/mach-spear13xx/sleep_1340.S
 *
 * SPEAR1340 specific functions that will run in internal SRAM.
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
 * spear1340_sleep_mode()
 * Forces SPEAr into sleep
 */
.text
ENTRY(spear1340_sleep_mode)
	mov	r10, r1		/* latch the address of cpu_resume */
	mov	r11, r0		/* latch the suspend state */

	ldr	r8, =SRAM_STACK_VA
	ldr	r6, =VA_MISC_BASE
	ldr	r0, [r6, #0x200]
	stmia	r8!, {r0}

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
	ldr	r1, =0x438
	/* Move DDR into self-refresh */
	ddr_in_srefresh r0, r6, r7, r1

	/* Enable XGPIO clk */
	ldr	r0, [r6, #0x314]
	ldr	r2, =0x40000
	orr	r0, r0, r2
	str	r0, [r6, #0x314]

	ldr	r6, =XGPIO_BASE
	/* Set xgpio[88-89] driven for cke & reset patch enable */
	ldr	r0, [r6, #0x048]
	ldr	r2, =0x03000000
	orr	r0, r0, r2
	str	r0, [r6, #0x048]

	ldr	r6, =MISC_BASE
	/* Stop the MPMC controller */
	ldr	r0, [r7, #0x2c]
	bic	r0, r0, #0x1000000
	str	r0, [r7, #0x2c]

	/* reset the MPMC controller */
	ldr	r1, [r6, #0x31C]
	orr	r1, r1, #0x3
	str	r1, [r6, #0x31C]

	/* Stop the MPMC clock */
	ldr	r1, [r6, #0x310]
	bic	r1, r1, #0x3
	str	r1, [r6, #0x310]

	/* Put the system in slow mode */
	system_slow_mode	r0, r6

	/* Move the ahb clock control to cpu/3 */
	ldr	r0, [r6, #0x200]
	and	r0, r0, #0x7ffffff
	str	r0, [r6, #0x200]

	switch_off_sys_plls  r0, r2, r6, r1

	/* Switch off Power Domains for s2r */
	cmp	r11, #0x1
	beq	standby_mode

	/* Store the Resume Physical Address on Stack Secure location */
	adr	r0, spear_sleep_restore
	ldr	r3, =SRAM_SCR_REG
	ldr	r6, =SRAM_START_PA
	str	r0, [r6, r3]

	/* Switch on certan sections as suggested by HW team */
	/* Let the USB Host clock be on */
	ldr	r6, =MISC_BASE

	/* Enable BootRom and Always On Ram Clock */
	ldr	r0, [r6, #0x30c]
	orr	r0, r0, #0x6
	str	r0, [r6, #0x30c]

	ldr	r0, [r6, #0x100]
	/*
	 * Switch off the power domains.
	 * Clear the ack bit
	 */
	bic	r0, r0, #0xc000
	str	r0, [r6, #0x100]

	bic	r0, r0, #0x800
	str	r0, [r6, #0x100]

wait_ack1:
	ldr	r0, [r6, #0x100]
	tst	r0, #0x4000
	beq	wait_ack1

	/* Clear the ack bit */
	bic	r0, r0, #0xc000
	str	r0, [r6, #0x100]

	bic	r0, r0, #0x1000
	str	r0, [r6, #0x100]
wait_ack2:
	ldr	r0, [r6, #0x100]
	tst	r0, #0x4000
	beq	wait_ack2

	/* Clear the ack bit */
	bic	r0, r0, #0xc000
	str	r0, [r6, #0x100]

	bic	r0, r0, #0x03e0
	bic	r0, r0, #0x2400
	str	r0, [r6, #0x100]
wait_ack3:
	ldr	r0, [r6, #0x100]
	tst	r0, #0x4000
	beq	wait_ack3
	nop
	nop
	nop
	nop
standby_mode:
	wfi				@ wait for interrupt
	nop
spear_sleep_restore:
	nop
	ldr	r6, =MISC_BASE
	ldr	r1, =MISC_PLL_OFFS

	/* Switch on Pll 4 */
	switch_on_pll4 r0, r6

	/* Switch the pll source back to pll-4 */
	ldr	r0, [r6, #0x244]
	orr	r0, r0, #0x0400
	str	r0, [r6, #0x244]

	/* Switch on pll-1/2/3 */	
	switch_on_pll_1_to_3 r0, r2, r6, r1

	ldr	r8, =SRAM_STACK_PA
	/* fetch the AHB clock config */
	ldmia	r8!, {r0}

	/* Move the Bus Parent clock to CPU/3 */
	str	r0, [r6, #0x200]

	/* Move the system into normal mode */
	system_normal_mode r0, r6

	ldr	r7, =MISC_BASE
	/* Enable the MPMC clock */
	ldr	r1, [r7, #0x310]
	orr	r1, r1, #0x3
	str	r1, [r7, #0x310]
	/* Provide Software Reset to MPMC */
	ldr	r1, [r7, #0x31C]
	orr	r1, r1, #0x3
	str	r1, [r7, #0x31C]
	/* Enable MPMC back */
	bic	r1, r1, #0x3
	str	r1, [r7, #0x31C]
	/* Restore the MPMC registers */
	ldr	r1, =MPMC_REG_END
	ldr	r6, =SRAM_START_PA
	ldr	r7, =A9SM_AND_MPMC_BASE
	
	mpmc_restore_regs r0, r2, r6, r7, r1

	ldr	r7, =XGPIO_BASE
	/* Set xgpio88 driven low for cke patch disable */
	ldr	r0, [r7, #0x048]
	ldr	r2, =0x02000000
	and	r0, r0, r2
	str	r0, [r7, #0x048]
	mov	r9, #0x0
ddr_out_srefresh:
	/* Add some delay for the cke signal to propagate */
	add	r9, r9, #0x1
	cmp	r9, #0x30000
	bne	ddr_out_srefresh

	/* Move the DDR out of self refresh mode */
	ldr	r6, =MISC_BASE
	ldr	r7, =A9SM_AND_MPMC_BASE

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

finalize_exit:
	/* Clearance is required in the mpmc space */
	ldr	r1, [r7, #0x2c]
	/*
	 * Clear bit srefresh bit (2) of MPMC_11 register
	 * The misc wrapper does not works fine by itself till
	 * this bit is also cleared.
	 */
	bic	r1, r1, #0x10000

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

	/* Check for the SPEAr revision */
	ldr	r7, =XGPIO_BASE
	/* Set xgpio89 driven low for cke patch disable */
	ldr	r0, [r7, #0x048]
	and	r0, r0, #0x0
	str	r0, [r7, #0x048]
	mov	r9, #0x0
wait_for_reset_low:
	/* Add some delay for the reset signal to propagate */
	add	r9, r9, #0x1
	cmp	r9, #0x30000
	bne	wait_for_reset_low

	uart_restore r0, r2, r7

	ldmia	r8!, {r0}
	mov	pc, r0
	nop
	.ltorg
	.align
ENTRY(spear1340_sleep_mode_sz)
	.word	. - spear1340_sleep_mode
