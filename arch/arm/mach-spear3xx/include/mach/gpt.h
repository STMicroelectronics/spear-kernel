/*
 * arch/arm/mach-spear3xx/include/mach/gpt.h
 *
 * General Purpose Timer definitions for SPEAr3xx machine family
 *
 * Copyright (C) 2010 ST Microelectronics
 * Deepak Sikri<deepak.sikri@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __MACH_GPT_H
#define __MACH_GPT_H

#include <linux/io.h>

/* register offsets */
#define GPT_CTRL_OFF	0x80 /* Control Register */
#define GPT_INT_OFF	0x84 /* Interrupt Regiset */
#define GPT_LOAD_OFF	0x88 /* Load Register */
#define GPT_COUNT_OFF	0x8C /* Current Count Register */

/* clock sources */
#define GPT_SRC_PLL3_CLK	0x00

/* CTRL Reg Bit Values */
#define GPT_CTRL_MATCH_INT	0x0100
#define GPT_CTRL_ENABLE	0x0020
#define GPT_CTRL_MODE_AR	0x0010

#define GPT_CTRL_PRESCALER1	0x0
#define GPT_CTRL_PRESCALER2	0x1
#define GPT_CTRL_PRESCALER4	0x2
#define GPT_CTRL_PRESCALER8	0x3
#define GPT_CTRL_PRESCALER16	0x4
#define GPT_CTRL_PRESCALER32	0x5
#define GPT_CTRL_PRESCALER64	0x6
#define GPT_CTRL_PRESCALER128	0x7
#define GPT_CTRL_PRESCALER256	0x8

/* INT Reg Bit Values */
#define GPT_STATUS_MATCH	0x0001

struct spear_timer {
	int id;
	unsigned long phys_base;
	int irq;
	struct clk *clk;
	void __iomem *io_base;
	unsigned reserved:1;
	unsigned enabled:1;
};

struct spear_timer_ops {
	/*
	 * Specify the source clock for the given timer. The current
	 * framework sets it at 48 Mhz, PLL3 clock.
	 */
	int (*timer_set_source)(struct spear_timer *timer, int source);
	/* Start the timer */
	int (*timer_start)(struct spear_timer *timer);
	/* Stop the timer */
	int (*timer_stop)(struct spear_timer *timer);
	/* Fetch the functional clock of timer */
	struct clk* (*timer_get_clk)(struct spear_timer *timer);
	/* Read the timer interrupt status register */
	int (*timer_read_status)(struct spear_timer *timer);
	/*Clear timer match interrupt */
	int (*timer_write_status)(struct spear_timer *timer, u16 value);
	/*
	 * The timer will be programmed as per the value in load.
	 * The time period will depend on the clock and prescaler selected.
	 * The autoreload parameteri decides, whether the count will be reloaded
	 */
	int (*timer_set_load_start)(struct spear_timer *timer, int autoreload,
			u16 load);
	/* Set the timer prescalar from 1 to 8 */
	int (*timer_set_prescalar)(struct spear_timer *timer, int prescaler);
	/* Set the timer interrupt on count match */
	int (*timer_set_match)(struct spear_timer *timer, int enable);
	/*
	 * Return a timer which is available, returns NULL if none is
	 * available.
	 */
	struct spear_timer* (*timer_request)(void);
	/* Free the timer */
	int (*timer_free)(struct spear_timer *timer);
};
#endif
