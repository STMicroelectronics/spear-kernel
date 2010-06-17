/*
 * arch/arm/plat-spear/sleep.S
 *
 * SPEAR3xx and SPEAR6xx specific functions that will run in
 * internal SRAM. The functions are used in power management.
 *
 * Copyright (ST) 2010 Deepak Sikri <deepak.sikri@.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/linkage.h>
#include <asm/assembler.h>
#include <mach/hardware.h>
#include <mach/suspend.h>

.text
ENTRY(spear_sleep_mode)
	stmfd	sp!, {r0-r12, lr}

	/* Latch some of MMU registers on to stack */
	mrc	p15, 0, r0, c5, c0, 0 /* FSR--Domain Fault */
	mrc	p15, 0, r1, c5, c0, 1 /* FSR--Instruction Fault */
	mrc	p15, 0, r2, c6, c0, 0 /* FAR */
	mrc	p15, 0, r3, c9, c0, 0 /* Read Dcache Lockdown */
	mrc	p15, 0, r4, c9, c0, 1 /* Read ICache Lockdown */
	mrc	p15, 0, r5, c9, c1, 0 /* Read Data TLB */
	mrc	p15, 0, r6, c9, c1, 1 /* Read Instr TCM region register */

	mrc	p15, 0, r7, c10, c0, 0 /* Data TLBLock Down operation */
	mrc	p15, 0, r8, c13, c0, 0 /* FCSE--PID */
	mrc	p15, 0, r9, c13, c0, 1 /* Context-ID */

	/* Save all these registers onto the stack */
	stmfd	sp!, {r0-r9}
	/* Save the stack pointer */
	mov	r3, sp
	/* Store the two mode registers */
	stmfd	r3!, {sp, lr}
	/* Save the current mode with irq disabled */
	mrs	r0, cpsr
	stmfd	r3!, {r0}
	/*
	 * Save the MMU registers on the SRAM Stack
	 * Domain Register on Back-up RAM structure
	 */
	mrc	p15, 0, r2, c3, c0, 0
	/* TTB Register */
	mrc	p15, 0, r1, c2, c0, 0
	/* MMU Enable Register */
	mrc	p15, 0, r0, c1, c0, 0
	stmfd	r3!, {r0, r1, r2}
	/*
	 * Capture the Physical Address.
	 * This will be used once MMU is Off
	 */
	adr	r0, mmu_off
	adr	r1, spear_sleep_mode
	/* Store the virtual address on to DDR */
	stmfd	r3!, {r1}
	sub	r1, r0, r1
	ldr	r0, SRAM_START_P
	add	r2, r1, r0

	/* Disable MMU */
	mrc	p15, 0, r0, c1, c0, 0
	ldr	r1, DISABLE_I_C_M_V
	bic	r0, r0, r1
	mcr	p15, 0, r0, c1, c0, 0
	/* Move the Physical address into PC */
	bx	r2

	/*
	 * This portion of code is executed from SRAM
	 * post MMU has been turned off
	 */
mmu_off:
	/* Store the DDR stack address onto scratch pad location */
	ldr	r0, SCRATCH_PAD
	str	r3, [r0]

	ldr	r6, MISC_BASE_P
	ldr	r7, MPMC_BASE_P
	ldr	r8, SYS_CTRL_BASE_P

	/*
	 * Put SDRAM in self-refresh mode
	 * Clear START bit(24) of MEMCTL_GP_03 register in MPMC
	 */
	ldr	r0, [r7, #0x1c]
	ldr	r4, =0x1000000
	/* Begin the command processing in controller */
	bic	r0, r0, r4
	str	r0, [r7, #0x1c]
	ldr	r0, [r7, #0x1c]
	/* set the SREFRESH bit(16) */
	ldr	r4, =0x10000
	orr	r0, r0, r4
	str	r0, [r7, #0x1c]

	/* Put the DDR into low power mode */
	ldr	r0, [r6, #0xf0]
	ldr	r4, =0x00000001
	/* Clear DDR_LOW_POWER_DDR2_MODE bit(1) of DDR_PAD register */
	bic	r0, r0, r4
	str	r0, [r6, #0xf0]

	/* Put the system in slow mode, use system controller */
	ldr	r0, [r8]
	bic	r0, r0, #0x4
	/* Set the apt mode bits(2:0) in SCCTRL register */
	orr	r0, r0, #0x2
	str	r0, [r8]	/* System is now in slow mode */

wait_till_slow_mode:
	ldr	r0, [r8]
	and	r0, r0, #0x78	/* Wait for the mode to be updated */
	cmp	r0, #0x10	/* Poll the SCCTRL register status bits (6:3) */
	bne wait_till_slow_mode

	/* Put the Pll-1 to off. */
	ldr	r0, [r6, #0x08]
	/* Clear pll_enable bit(2) of PLL1_CTR register in Misc registers */
	bic	r0, r0, #0x04
	str	r0, [r6, #0x08]

	/* Put the Pll-2 to off */
	ldr	r0, [r6, #0x14]
	/* Clear pll_enable bit(2) of PLL2_CTR register in Misc registers */
	bic	r0, r0, #0x04
	str	r0, [r6, #0x14]
	mov	r2, #0
	/* Put the system in sleep */
	ldr	r0, [r8]
	/* Set the apt mode bits(2:0) in SCCTRL register */
	bic	r0, r0, #0x7
#ifdef TEST_SLOW
	orr	r0, r0, #0x2 /* Slow Mode */
#endif
	str	r0, [r8]
	/* Put system in WFI */
	mcr	p15, 0, r2, c7, c0, 4
wakeup_addr:
	ldr	r6, MISC_BASE_P
	ldr	r7, MPMC_BASE_P
	ldr	r8, SYS_CTRL_BASE_P
	/* Reenable pll1 and pll2 */
	ldr	r0, PLL_VAL1
	str	r0, [r6, #0x08]
	str	r0, [r6, #0x14]
	ldr	r0, PLL_VAL2
	str	r0, [r6, #0x08]
	str	r0, [r6, #0x14]
	/* Strobe */
	ldr	r2, PLL_VAL3
	str	r2, [r6, #0x08]
	str	r2, [r6, #0x14]
	ldr	r2, PLL_VAL2
	str	r2, [r6, #0x08]
	str	r2, [r6, #0x14]
pll1_lock_1:
	/* Set the pll_lock bit(0) in PLL1_CTR register in misc space*/
	ldr	r2, [r6, #0x08]
	and	r2, r2, #0x1
	/* Wait for pll-1 lock status */
	cmp	r2, #0x1
	bne	pll1_lock_1

pll2_lock_2:
	/* Set the pll_lock bit(0) in PLL2_CTR register in misc space*/
	ldr	r2, [r6, #0x14]
	and	r2, r2, #0x1
	/* Wait for pll-2 lock status */
	cmp	r2, #0x1
	bne	pll2_lock_2

	/* Move the system in Normal mode, use system controller */
	ldr	r3, [r8]
	/* Set the apt mode bits(2:0) in SCCTRL register */
	bic	r3, r3, #0x7
	orr	r3, r3, #0x4
	str	r3, [r8]

wait_till_norm_mode:
	ldr	r3, [r8]
	and	r3, r3, #0x78
	cmp	r3, #0x20	/* Poll the SCCTRL register status bits (6:3) */
	bne	wait_till_norm_mode

	/* Resume the DDR from Low power mode. */
	ldr	r0, [r6, #0xf0]
	/* Set DDR_LOW_POWER_DDR2_MODE bit(1) of DDR_PAD register */
	orr	r0, r0, #0x01
	str	r0, [r6, #0xf0]

	/* Exit DDR-SDRAM from self-refresh mode */
	ldr	r1, [r7, #0x1c]
	/* clear the SREFRESH bit(16) */
	ldr	r4, =0x10000
	bic	r1, r1, r4
	str	r1, [r7, #0x1c]

	/* Begin the command processing in controller */
	ldr	r4, =0x1000000
	/* Set START bit(24) of MEMCTL_GP_03 register in MPMC*/
	orr	r1, r1, r4
	str	r1, [r7, #0x1c]

	mov	r0, r0
	/* Start the Restore Processing */
	ldr	r0, SCRATCH_PAD
	ldr	r6, [r0]

	/* Restore the Virtual Address to be used */
	/* Once MMU is made on */
	ldr	r0, SRAM_START_P
	adr	r1, mmu_on
	sub	r0, r1, r0
	/* Get the physical Address */
	mov	r3, #0xc0000000
	sub	r6, r6, r3
	/* Fetch the sram virtual address */
	ldmfd	r6!, {r1}
	add	r4, r1, r0

	/* Fetch the MMU Related information latched on SDRAM */
	ldmfd	r6!, {r0, r1, r2}
	/* Enable the MMU */
	mcr	p15, 0, r2, c3, c0, 0
	mcr	p15, 0, r1, c2, c0, 0
	mcr	p15, 0, r0, c1, c0, 0
	bx	r4
mmu_on:
	add	r6, r6, r3
	/* Store the value of cpsr in R0 */
	mrs	r0, cpsr
	bic	r0, r0, #MODE_BITS

	/* Here we will restore our cpsr IRQ/FIQ Disabled */
	ldr	r0, [r6]
	msr	cpsr_cxsf, r0
	add	r6, r6, #4

	/* Now only two user-mode registers are left */
	ldmfd	r6!, {sp, lr}
	mov	r0, r0

	/* Restore stack pointer for the current mode */
	mov	sp, r6

	ldmfd	sp!, {r0-r9}
	mcr	p15, 0, r0, c5, c0, 0 /* FSR--Domain Fault */
	mcr	p15, 0, r1, c5, c0, 1 /* FSR--Instruction Fault */
	mcr	p15, 0, r2, c6, c0, 0 /* FAR */
	mcr	p15, 0, r3, c9, c0, 0 /* Read Dcache Lockdown */
	mcr	p15, 0, r4, c9, c0, 1 /* Read ICache Lockdown */
	mcr 	p15, 0, r5, c9, c1, 0 /* Read Data TLB */
	mcr	p15, 0, r6, c9, c1, 1 /* Read Instruction Lockdown */

	mcr	p15, 0, r7, c10, c0, 0 /* Data TLB LockDown operation */
	mcr	p15, 0, r8, c13, c0, 0 /* FCSE--PID */
	mcr	p15, 0, r9, c13, c0, 1 /* Context-ID */

	mov	r0, #0
	ldmfd	sp!, {r0-r12, pc}

SYS_CTRL_BASE_P :
	.word SYS_CTRL_BASE_PA
MPMC_BASE_P :
	.word MPMC_BASE_PA
MISC_BASE_P :
	.word MISC_BASE_PA
SRAM_START_P:
	.word SPEAR_START_SRAM
SCRATCH_PAD:
	.word SPEAR_SRAM_SCR_REG
DISABLE_I_C_M_V:
	.word 0x1005
PLL_VAL1:
	.word 0x1c0a
PLL_VAL2:
	.word 0x1c0e
PLL_VAL3:
	.word 0x1c06
ENTRY(spear_sleep_mode_sz)
	.word . - spear_sleep_mode
