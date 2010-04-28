/*
 * arch/arm/mach-spear3xx/include/mach/gpio.h
 *
 * GPIO macros for SPEAr3xx machine family
 *
 * Copyright (C) 2009 ST Microelectronics
 * Viresh Kumar<viresh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __MACH_GPIO_H
#define __MACH_GPIO_H

#include <plat/gpio.h>

#ifdef CONFIG_MACH_SPEAR310
#define PLGPIO_ENB		0x0010
#define PLGPIO_WDATA		0x0020
#define PLGPIO_DIR		0x0030
#define PLGPIO_IE		0x0040
#define PLGPIO_RDATA		0x0050
#define PLGPIO_MIS		0x0060

#elif defined(CONFIG_MACH_SPEAR320)
#define PLGPIO_ENB		0x0024
#define PLGPIO_WDATA		0x0034
#define PLGPIO_DIR		0x0044
#define PLGPIO_RDATA		0x0054
#define PLGPIO_IE		0x0064
#define PLGPIO_MIS		0x0074
#endif

#define BASIC_GPIO_0		0
#define BASIC_GPIO_1		1
#define BASIC_GPIO_2		2
#define BASIC_GPIO_3		3
#define BASIC_GPIO_4		4
#define BASIC_GPIO_5		5
#define BASIC_GPIO_6		6
#define BASIC_GPIO_7		7

#ifdef CONFIG_MACH_SPEAR300
#define RAS_GPIO_0		8
#define RAS_GPIO_1		9
#define RAS_GPIO_2		10
#define RAS_GPIO_3		11
#define RAS_GPIO_4		12
#define RAS_GPIO_5		13
#define RAS_GPIO_6		14
#define RAS_GPIO_7		15

#elif defined(CONFIG_MACH_SPEAR310) || defined(CONFIG_MACH_SPEAR320)
#define PLGPIO_0		8
#define PLGPIO_1		9
#define PLGPIO_2		10
#define PLGPIO_3		11
#define PLGPIO_4		12
#define PLGPIO_5		13
#define PLGPIO_6		14
#define PLGPIO_7		15
#define PLGPIO_8		16
#define PLGPIO_9		17
#define PLGPIO_10		18
#define PLGPIO_11		19
#define PLGPIO_12		20
#define PLGPIO_13		21
#define PLGPIO_14		22
#define PLGPIO_15		23
#define PLGPIO_16		24
#define PLGPIO_17		25
#define PLGPIO_18		26
#define PLGPIO_19		27
#define PLGPIO_20		28
#define PLGPIO_21		29
#define PLGPIO_22		30
#define PLGPIO_23		31
#define PLGPIO_24		32
#define PLGPIO_25		33
#define PLGPIO_26		34
#define PLGPIO_27		35
#define PLGPIO_28		36
#define PLGPIO_29		37
#define PLGPIO_30		38
#define PLGPIO_31		39
#define PLGPIO_32		40
#define PLGPIO_33		41
#define PLGPIO_34		42
#define PLGPIO_35		43
#define PLGPIO_36		44
#define PLGPIO_37		45
#define PLGPIO_38		46
#define PLGPIO_39		47
#define PLGPIO_40		48
#define PLGPIO_41		49
#define PLGPIO_42		50
#define PLGPIO_43		51
#define PLGPIO_44		52
#define PLGPIO_45		53
#define PLGPIO_46		54
#define PLGPIO_47		55
#define PLGPIO_48		56
#define PLGPIO_49		57
#define PLGPIO_50		58
#define PLGPIO_51		59
#define PLGPIO_52		60
#define PLGPIO_53		61
#define PLGPIO_54		62
#define PLGPIO_55		63
#define PLGPIO_56		64
#define PLGPIO_57		65
#define PLGPIO_58		66
#define PLGPIO_59		67
#define PLGPIO_60		68
#define PLGPIO_61		69
#define PLGPIO_62		70
#define PLGPIO_63		71
#define PLGPIO_64		72
#define PLGPIO_65		73
#define PLGPIO_66		74
#define PLGPIO_67		75
#define PLGPIO_68		76
#define PLGPIO_69		77
#define PLGPIO_70		78
#define PLGPIO_71		79
#define PLGPIO_72		80
#define PLGPIO_73		81
#define PLGPIO_74		82
#define PLGPIO_75		83
#define PLGPIO_76		84
#define PLGPIO_77		85
#define PLGPIO_78		86
#define PLGPIO_79		87
#define PLGPIO_80		88
#define PLGPIO_81		89
#define PLGPIO_82		90
#define PLGPIO_83		91
#define PLGPIO_84		92
#define PLGPIO_85		93
#define PLGPIO_86		94
#define PLGPIO_87		95
#define PLGPIO_88		96
#define PLGPIO_89		97
#define PLGPIO_90		98
#define PLGPIO_91		99
#define PLGPIO_92		100
#define PLGPIO_93		101
#define PLGPIO_94		102
#define PLGPIO_95		103
#define PLGPIO_96		104
#define PLGPIO_97		105
#define PLGPIO_98		106
#define PLGPIO_99		107
#define PLGPIO_100		108
#define PLGPIO_101		109
#endif /* CONFIG_MACH_SPEAR310 || CONFIG_MACH_SPEAR320 */

#endif /* __MACH_GPIO_H */
