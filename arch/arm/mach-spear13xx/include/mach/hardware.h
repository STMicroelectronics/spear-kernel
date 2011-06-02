/*
 * arch/arm/mach-spear13xx/include/mach/hardware.h
 *
 * Hardware definitions for spear13xx machine family
 *
 * Copyright (C) 2010 ST Microelectronics
 * Shiraz Hashim <shiraz.hashim@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __MACH_HARDWARE_H
#define __MACH_HARDWARE_H

#include <plat/hardware.h>
#include <mach/spear.h>

/* Vitual to physical translation of statically mapped space
 * Physical			Virtual
 * 6c800000-6c801000		edc00000-edc01000
 * e0000000-e0001000		fd000000-fd001000
 * ec800000-ec802000		fdc00000-fdc02000
 * ed000000-ed001000		fed00000-fed01000
 * e0700000-e0702000		fd700000-fd702000
 * b3800000-b3808000		fab00000-fab08000
 * e0800000-e0900000		fd800000-fd900000
 * 80000000-80010000		f7000000-f7010000
 * 90000000-90010000		f8000000-f8010000
 * c0000000-c0010000		fb000000-fb010000
 *
 */

#define IO_ADDRESS(x)		((((x) | ((((x) >> 31) << 28) | 0xE0000000)) \
				| (((x) & 0xff000000) >> 4)) - 0x01000000)

/* typesafe io address */
#define __io_address(n)		__io(IO_ADDRESS(n))

#define PCIBIOS_MIN_IO		0x1000
#define PCIBIOS_MIN_MEM		0
#define pcibios_assign_all_busses()	0

#endif /* __MACH_HARDWARE_H */
