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

.text
ENTRY(spear_sleep_mode)
	ldr	r3, =SRAM_STACK_VA
	/* Store the physical address of cpu_resume routine */
	stmia	r3!, {r0}
	/*
	 * Capture the Physical Address.
	 * This will be used once MMU is Off
	 */
	adr	r0, mmu_off
	adr	r1, spear_sleep_mode
	sub	r1, r0, r1
	ldr	r0, =SRAM_START_PA
	add	r2, r1, r0

	/* Disable MMU */
	mrc	p15, 0, r0, c1, c0, 0
	ldr	r1, =DISABLE_I_C_M_V
	bic	r0, r0, r1
	mcr	p15, 0, r0, c1, c0, 0
	/* Move the Physical address into PC */
	bx	r2

	/*
	 * This portion of code is executed from SRAM
	 * post MMU has been turned off
	 */
mmu_off:
	ldr	r6, =MISC_BASE_PA
	ldr	r7, =MPMC_BASE_PA
	ldr	r8, =SYS_CTRL_BASE_PA

	ldr	r3, [r7, #0xe4]
	ldr	r4, =0xffff0000

	/* Latch the current self refresh time */
	mov	r9, r3

	/* Increase the self refresh exit time */
	bic	r3, r3, r4
	ldr	r4, =0xffff

	/* Program the SDRAM self refresh exit time on read command */
	orr	r3, r3, r4, LSL #16
	str	r3, [r7, #0xe4]

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
	ldr	r6, =MISC_BASE_PA
	ldr	r7, =MPMC_BASE_PA
	ldr	r8, =SYS_CTRL_BASE_PA

	/* Reenable pll1 and pll2 */
	ldr	r0, =PLL_VAL1
	str	r0, [r6, #0x08]
	str	r0, [r6, #0x14]
	ldr	r0, =PLL_VAL2
	str	r0, [r6, #0x08]
	str	r0, [r6, #0x14]
	/* Strobe */
	ldr	r2, =PLL_VAL3
	str	r2, [r6, #0x08]
	str	r2, [r6, #0x14]
	ldr	r2, =PLL_VAL2
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

	/* Restore the SDRAM self refresh exit time on read command */
	mov	r1, r9
	str	r1, [r7, #0xe4]
	/* Begin the command processing in controller */
	ldr	r4, =0x1000000
	/* Set START bit(24) of MEMCTL_GP_03 register in MPMC*/
	orr	r1, r1, r4
	str	r1, [r7, #0x1c]

	mov	r0, #0
	/* Start the Restore Processing */
	ldr	r6, =SRAM_STACK_PA
	ldmia	r6!, {r0}
	mov	pc, r0
	nop
	.ltorg
	.align
ENTRY(spear_sleep_mode_sz)
	.word . - spear_sleep_mode
