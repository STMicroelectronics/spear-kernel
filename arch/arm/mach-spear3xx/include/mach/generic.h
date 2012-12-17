/*
 * arch/arm/mach-spear3xx/generic.h
 *
 * SPEAr3XX machine family generic header file
 *
 * Copyright (C) 2009 ST Microelectronics
 * Viresh Kumar<viresh.linux@gmail.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __MACH_GENERIC_H
#define __MACH_GENERIC_H

#include <linux/amba/bus.h>
#include <linux/amba/clcd.h>
#include <linux/amba/pl08x.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/amba/bus.h>
#include <asm/mach/time.h>
#include <asm/mach/map.h>
#include <sound/designware_i2s.h>
#include <plat/jpeg.h>

/* Add spear3xx family device structure declarations here */
extern struct adc_plat_data adc_pdata;
extern struct sys_timer spear3xx_timer;
extern struct pl022_ssp_controller pl022_plat_data;
extern struct jpeg_plat_data jpeg_pdata;
extern struct pl08x_platform_data pl080_plat_data;
extern struct clcd_board pl110_plat_data; /* amba clcd platform data */

/* Add spear3xx family function declarations here */
void __init spear_setup_of_timer(void);
void __init spear3xx_clk_init(void);
void __init spear3xx_map_io(void);
void __init spear3xx_dt_init_irq(void);
int __init spear3xx_emi_init(u32 base, int numbanks);
int clcd_panel_setup(struct clcd_panel *panel);
int audio_clk_config(struct i2s_hw_config_data *config);
void spear3xx_macb_setup(void);

/* Add SPEAr320 function declaration here */
int spear320_uart_clk_config(void);

void spear_restart(char, const char *);

#endif /* __MACH_GENERIC_H */
