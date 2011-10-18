/*
 * arch/arm/mach-spear13xx/include/mach/plug_board.h
 *
 * spear13xx machine family plug board header file
 *
 * Copyright (C) 2011 ST Microelectronics
 * Pratyush Anand <pratyush.anand@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __MACH_PLUG_BOARD_H
#define __MACH_PLUG_BOARD_H

#include <linux/amba/bus.h>
#include <linux/platform_device.h>

struct plug_board_info {
	struct platform_device **pdevs;
	u8 pcnt;
	struct amba_device **adevs;
	u8 acnt;
};

int __init spear1340_pb_init(struct plug_board_info *pb_info);
#endif
