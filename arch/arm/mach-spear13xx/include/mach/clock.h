/*
 * arch/arm/mach-spear13xx/include/mach/clock.h
 *
 * Copyright (C) 2010 ST Microelectronics
 * Shiraz Hashim <shiraz.hashim@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __MACH_CLOCK_H
#define __MACH_CLOCK_H

#include <linux/list.h>
#include <asm/clkdev.h>
#include <linux/types.h>

void pll_clk_recalc(struct clk *clk);
void cpu_clk_recalc(struct clk *clk);
void ahb_clk_recalc(struct clk *clk);
void apb_clk_recalc(struct clk *clk);

#endif /* __MACH_CLOCK_H */
