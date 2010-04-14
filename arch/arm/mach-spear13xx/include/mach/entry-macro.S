/*
 * arch/arm/mach-spear13xx/include/mach/entry-macro.S
 *
 * Copyright (C) 2009 ST Microelectronics
 * Shiraz Hashim <shiraz.hashim@st.com>
 *
 * Copy of RealView platform implementation
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <mach/hardware.h>
#include <asm/hardware/entry-macro-gic.S>

		.macro	disable_fiq
		.endm

		.macro	arch_ret_to_user, tmp1, tmp2
		.endm

