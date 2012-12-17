/*
 * arch/arm/plat-spear/include/plat/pm.h
 *
 * SPEAr platform sleep save info for shared peripherals.
 *
 * Copyright (C) 2012 ST Microelectronics
 * Rajeev Kumar <rajeev-dlh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __PLAT_PM_H
#define __PLAT_PM_H

/**
 * struct sleep_save - save information for shared peripherals.
 * @reg: Pointer to the register to save.
 * @val: Holder for the value saved from reg.
 *
 * This describes a list of registers which is used by the pm core and
 * other subsystem to save and restore register values over suspend.
 */
struct sleep_save {
	void __iomem	*reg;
	unsigned long	val;
};

#define SAVE_ITEM(x) \
	{ .reg = (x) }

/* helper functions to save and restore register state */

/*
 * spear_pm_do_save() - save a set of registers for restoration on resume.
 * @ptr: Pointer to an array of registers.
 * @count: Size of the ptr array.
 *
 * Run through the list of registers given, saving their contents in the
 * array for later restoration when we wakeup.
 */
static inline void spear_pm_do_save(struct sleep_save *ptr, int count)
{
	for (; count > 0; count--, ptr++)
		ptr->val = __raw_readl(ptr->reg);
}

/*
 * spear_pm_do_restore_core() - early restore register values from save list.
 *
 * Restore the register values saved from spear_pm_do_save().
 *
 * WARNING: Do not put any debug in here that may effect memory or use
 * peripherals, as things may be changing!
 */
static inline void spear_pm_do_restore_core(struct sleep_save *ptr, int count)
{
	for (; count > 0; count--, ptr++)
		__raw_writel(ptr->val, ptr->reg);
}

#endif /* __PLAT_PM_H */
