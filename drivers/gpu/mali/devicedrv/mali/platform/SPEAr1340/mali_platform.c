/*
 * Copyright (C) 2010-2012 ARM Limited. All rights reserved.
 * 
 * This program is free software and is provided to you under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation, and any use by you of this program is subject to the terms of such GNU licence.
 * 
 * A copy of the licence is included with the program, and can also be obtained from Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

/**
 * @file mali_platform.c
 * Platform specific Mali driver functions for a default platform
 */
#include "mali_kernel_common.h"
#include "mali_osk.h"
#include "mali_platform.h"

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/device.h>

#define MALI_DEF_RATE (200*1000*1000)

struct clk *mali_clk;

_mali_osk_errcode_t mali_platform_init(void)
{
	int err;

	/*clock init*/
	//mali_clk = clk_get(&pdev->dev, NULL);
	mali_clk = clk_get_sys("mali", NULL);
	if (IS_ERR(mali_clk)) {
		MALI_DEBUG_PRINT(1, ("%s: Failed to get clock %s\n", __func__, "mali"));
		goto error;
	}

	clk_set_rate(mali_clk, MALI_DEF_RATE);

	err = clk_enable(mali_clk);
	if (err) {
		MALI_DEBUG_PRINT(1, ("%s: Failed to set clock rate\n", __func__));
		goto put_clk;
	}

	MALI_SUCCESS;
	
put_clk:
	clk_put(mali_clk);
error:
	MALI_DEBUG_PRINT(1, ("%s: initialization failed.\n", __func__));
	MALI_ERROR(_MALI_OSK_ERR_FAULT);	
}

_mali_osk_errcode_t mali_platform_deinit(void)
{
	clk_put(mali_clk);
	MALI_DEBUG_PRINT(2, ("%s: terminated\n", __func__));
	MALI_SUCCESS;
}

_mali_osk_errcode_t mali_platform_power_mode_change(mali_power_mode power_mode)
{
    MALI_SUCCESS;
}

void mali_gpu_utilization_handler(u32 utilization)
{
}

void set_mali_parent_power_domain(void* dev)
{
}


