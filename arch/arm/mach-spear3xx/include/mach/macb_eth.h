/*
 * arch/arm/mach-spear3xx/include/mach/macb_eth.h
 *
 * SPEAr Ethernet Controller specific info
 *
 * Copyright (ST) 2010 Deepak Sikri (deepak.sikri@st.com)
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
#ifndef __MACH_ETH_H
#define __MACB_ETH_H

#include <linux/types.h>
#include <linux/platform_device.h>

#define MAX_ADDR_LEN 32

struct macb_base_data {
	int bus_id;
	u32 phy_mask;
	int phy_addr;
	u8 mac_addr[MAX_ADDR_LEN];
	s8 gpio_num;
	u8 is_rmii; /* using RMII interface? */
	void (*plat_mdio_control)(struct platform_device *);
};

static inline void macb_set_plat_data(struct platform_device *pdev,
		struct macb_base_data *pdata)
{
	pdev->dev.platform_data = pdata;
}

#endif
