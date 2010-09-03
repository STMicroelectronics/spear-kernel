/*
 * arch/arm/mach-spear13xx/include/mach/generic.h
 *
 * spear13xx machine family generic header file
 *
 * Copyright (C) 2010 ST Microelectronics
 * Shiraz Hashim <shiraz.hashim@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __MACH_GENERIC_H
#define __MACH_GENERIC_H

#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/amba/bus.h>
#include <asm/mach/time.h>
#include <asm/mach/map.h>

/*
 * Each GPT has 2 timer channels
 * Following GPT channels will be used as clock source and clockevent
 */
#define SPEAR_GPT0_BASE		SPEAR13XX_GPT0_BASE
#define SPEAR_GPT0_CHAN0_IRQ	IRQ_GPT0_TMR0
#define SPEAR_GPT0_CHAN1_IRQ	IRQ_GPT0_TMR1

/* Add spear13xx family device structure declarations here */
extern struct amba_device spear13xx_uart_device;
extern struct platform_device spear13xx_ehci0_device;
extern struct platform_device spear13xx_ehci1_device;
extern struct platform_device spear13xx_i2c_device;
extern struct platform_device spear13xx_ohci0_device;
extern struct platform_device spear13xx_ohci1_device;
extern struct platform_device spear13xx_rtc_device;
extern struct sys_timer spear13xx_timer;

/* Add spear1300 machine device structure declarations here */

/* Add spear13xx family function declarations here */
void __init clk_init(void);
void __init i2c_register_default_devices(void);
void __init spear_setup_timer(void);
void __init spear1300_init(void);
void __init spear13xx_map_io(void);
void __init spear13xx_init_irq(void);
void __init spear13xx_init(void);
void spear13xx_secondary_startup(void);

#endif /* __MACH_GENERIC_H */
