/*
 * arch/arm/mach-spear6xx/include/mach/generic.h
 *
 * SPEAr6XX machine family specific generic header file
 *
 * Copyright (C) 2009 ST Microelectronics
 * Rajeev Kumar<rajeev-dlh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __MACH_GENERIC_H
#define __MACH_GENERIC_H

#include <asm/mach/time.h>
#include <asm/mach/map.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/amba/bus.h>

/*
 * Each GPT has 2 timer channels
 * Following GPT channels will be used as clock source and clockevent
 */
#define SPEAR_GPT0_BASE		SPEAR6XX_CPU_TMR_BASE
#define SPEAR_GPT0_CHAN0_IRQ	IRQ_CPU_GPT1_1
#define SPEAR_GPT0_CHAN1_IRQ	IRQ_CPU_GPT1_2

/* Add spear6xx family device structure declarations here */
extern struct amba_device clcd_device;
extern struct amba_device dma_device;
extern struct amba_device gpio_device[];
extern struct amba_device ssp_device[];
extern struct amba_device uart_device[];
extern struct amba_device wdt_device;
extern struct platform_device adc_device;
extern struct platform_device cpufreq_device;
extern struct platform_device ehci0_device;
extern struct platform_device ehci1_device;
extern struct platform_device eth_device;
extern struct platform_device i2c_device;
extern struct platform_device irda_device;
extern struct platform_device jpeg_device;
extern struct platform_device nand_device;
extern struct platform_device ohci0_device;
extern struct platform_device ohci1_device;
extern struct platform_device rtc_device;
extern struct platform_device smi_device;
extern struct platform_device touchscreen_device;
extern struct platform_device udc_device;
extern struct sys_timer spear6xx_timer;

/* Add spear6xx family function declarations here */
void __init i2c_register_default_devices(void);
void __init spear_setup_timer(void);
void __init spear6xx_map_io(void);
void __init spear6xx_init_irq(void);
void __init spear6xx_init(void);
void __init spear600_init(void);
void __init spear6xx_clk_init(void);

/* Add spear600 machine device structure declarations here */

/* Add misc structure declarations here */
extern struct clcd_board clcd_plat_data;

#endif /* __MACH_GENERIC_H */
