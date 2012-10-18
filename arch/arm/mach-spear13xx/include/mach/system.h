/*
 * arch/arm/mach-spear13xx/include/mach/system.h
 *
 * spear13xx Machine family specific architecture functions
 *
 * Copyright (C) 2010 ST Microelectronics
 * Shiraz Hashim <shiraz.hashim@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __MACH_SYSTEM_H
#define __MACH_SYSTEM_H

#include <linux/io.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <mach/spear.h>

#define VA_SYS_CLK_CTRL		(VA_MISC_BASE + 0x200)

/* System Mode Control Bit Masks */
#define SYS_MODE_MASK		(0x7 << 0)
#define SYS_MODE_DOZE		(0x1 << 0)
#define SYS_MODE_SLOW		(0x2 << 0)
#define SYS_MODE_NORMAL		(0x4 << 0)
#define SYS_MODE_STS_MASK	(0xF << 16)
#define SYS_MODE_STS_DOZE	(0x0 << 16)
#define SYS_MODE_STS_SLOW	(0xA << 16)
#define SYS_MODE_STS_NORMAL	(0xF << 16)

static inline int arch_change_mode(int mode)
{
	u32 val, mode_sts;
	unsigned long finish;
	void __iomem *sys_reg;

	sys_reg = VA_SYS_CLK_CTRL;

	switch (mode) {
	case SYS_MODE_DOZE:
		mode_sts = SYS_MODE_STS_DOZE;
		break;
	case SYS_MODE_SLOW:
		mode_sts = SYS_MODE_STS_SLOW;
		break;
	case SYS_MODE_NORMAL:
		mode_sts = SYS_MODE_STS_NORMAL;
		break;
	default:
		pr_err("Wrong system mode\n");
		return -EINVAL;
	}

	val = readl(sys_reg);
	if ((val & SYS_MODE_STS_MASK) == mode_sts)
		return 0;

	val &= ~SYS_MODE_MASK;
	val |= mode;
	writel(val, sys_reg);

	/* read back if mode is set */
	finish = jiffies + 2 * HZ;
	do {
		val = readl(sys_reg);
		if ((val & SYS_MODE_STS_MASK) == mode_sts)
			return 0;
		udelay(1000);
	} while (!time_after_eq(jiffies, finish));

	return -EFAULT;
}

#endif /* __MACH_SYSTEM_H */
