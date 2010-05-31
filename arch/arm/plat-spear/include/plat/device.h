/*
 * arch/arm/plat-spear/include/plat/device.h
 *
 * device definitions for SPEAr platform
 *
 * Copyright (C) 2010 ST Microelectronics
 * Viresh Kumar<viresh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __PLAT_DEVICE_H
#define __PLAT_DEVICE_H

#include <linux/amba/bus.h>
#include <linux/clk.h>

static inline void
spear_amba_device_register(struct amba_device **devices, u32 count)
{
	u32 i;

	for (i = 0; i < count; i++) {
		struct clk *clk = clk_get_sys(devices[i]->dev.init_name, NULL);
		if (!clk)
			continue;

		clk_enable(clk);
		amba_device_register(devices[i], &iomem_resource);
		clk_disable(clk);
	}
}

#endif /* __PLAT_DEVICE_H */
