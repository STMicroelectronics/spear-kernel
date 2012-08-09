/*
 * Copyright (C) 2004-2006 Atmel Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __MACB_PDATA_H__
#define __MACB_PDATA_H__
#include <linux/platform_device.h>

struct macb_platform_data {
	int		bus_id;
	u32		phy_mask;
	int		phy_addr;
	int		phy_irq_pin;	/* PHY IRQ */
	u8              is_rmii;        /* using RMII interface? */
	void		(*plat_mdio_control)(struct platform_device *);
};

#endif /* __MACB_PDATA_H__ */
