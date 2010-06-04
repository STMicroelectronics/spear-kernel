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

#include <asm/mach/time.h>
#include <asm/mach/map.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/amba/bus.h>
#include <plat/device.h>

/*
 * Each GPT has 2 timer channels
 * Following GPT channels will be used as clock source and clockevent
 */
#define SPEAR_GPT0_BASE		SPEAR13XX_GPT0_BASE
#define SPEAR_GPT0_CHAN0_IRQ	IRQ_GPT0_TMR0
#define SPEAR_GPT0_CHAN1_IRQ	IRQ_GPT0_TMR1

/* Add spear13xx family device structure declarations here */
extern struct amba_device gpio_device[];
extern struct amba_device ssp_device;
extern struct amba_device uart_device;
extern struct platform_device adc_device;
extern struct platform_device dmac_device[];
extern struct platform_device ehci0_device;
extern struct platform_device ehci1_device;
extern struct platform_device eth_device;
extern struct platform_device i2c_device;
extern struct platform_device jpeg_device;
extern struct platform_device kbd_device;
extern struct platform_device nand_device;
extern struct platform_device ohci0_device;
extern struct platform_device ohci1_device;
extern struct platform_device phy_device;
extern struct platform_device rtc_device;
extern struct platform_device sdhci_device;
extern struct platform_device smi_device;
extern struct platform_device wdt_device;
extern struct sys_timer spear13xx_timer;

/* Add spear13xx family function declarations here */
void __init clk_init(void);
void __init i2c_register_board_devices(void);
void __init spear_setup_timer(void);
void __init spear13xx_map_io(void);
void __init spear13xx_init_irq(void);
void __init spear13xx_init(void);
void __init nand_mach_init(u32 busw);
void spear13xx_secondary_startup(void);

/* spear1300 declarations */
#ifdef CONFIG_MACH_SPEAR1300
/* Add spear1300 machine function declarations here */
void __init spear1300_init(void);

#endif /* CONFIG_MACH_SPEAR1300 */

/* spear1310 declarations */
#ifdef CONFIG_MACH_SPEAR1310
/* Add spear1310 machine device structure declarations here */
extern struct platform_device can0_device;
extern struct platform_device can1_device;

/* Add spear1310 machine function declarations here */
void __init spear1310_init(void);

#endif /* CONFIG_MACH_SPEAR1310 */

#endif /* __MACH_GENERIC_H */
