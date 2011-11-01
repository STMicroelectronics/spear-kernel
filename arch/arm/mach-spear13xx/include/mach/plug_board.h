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
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>

struct i2c_dev_info {
	struct i2c_board_info *board;
	int busnum;
};

struct plug_board_info {
	struct platform_device **pdevs;
	u8 pcnt;
	struct amba_device **adevs;
	u8 acnt;
	struct spi_board_info **spi_devs;
	u8 spi_cnt;
	struct i2c_dev_info **i2c_devs;
	u8 i2c_cnt;
};

/* Add spear1340 i2c devices structure declaration */
extern struct i2c_dev_info spear1340_evb_i2c_l3g4200d_gyr;
extern struct i2c_dev_info spear1340_evb_i2c_eeprom0;
extern struct i2c_dev_info spear1340_evb_i2c_eeprom1;
extern struct i2c_dev_info spear1340_evb_i2c_sta529;

int __init spear1340_pb_init(struct plug_board_info *pb_info);
#endif
