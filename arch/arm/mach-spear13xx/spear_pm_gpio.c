/*
 * arch/arm/mach-spear13xx/spear_gpio.c
 *
 * SPEAr13xx Power Management GPIO management
 *
 * Copyright (C) 2011 ST Microelectronics
 * Vincenzo Frascino <vincenzo.frascino@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
 
 #include <linux/clk.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/list.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/lockdep.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/gpio.h>
#include <asm/system.h>
#include <linux/gpio.h>
#include <mach/hardware.h>
#include <mach/spear_pm_gpio.h>

static int __init spear_gpio_init(void)
{
	int index = 0;
	
	printk("SPEAr Power Management GPIOs: Init\n");
	for(index = 0; index <  ARRAY_SIZE(spear_gpio); index++) {
		if (gpio_request_one(spear_gpio[index].gpio, spear_gpio[index].flags, spear_gpio[index].label) == 0) {
			if (0 == gpio_export(spear_gpio[index].gpio, 0))
				printk("SPEAr Power Management GPIOs: exported gpio %d %s\n", spear_gpio[index].gpio, spear_gpio[index].label);
			else
				printk("SPEAr Power Management GPIOs: didn't export gpio %d\n", spear_gpio[index].gpio);
		} else
			printk(KERN_ERR "SPEAr Power Management GPIOs: could not obtain gpios\n");
	}
	
	return 0;
}

static void __exit spear_gpio_exit(void)
{
	int index = 0;
	
	printk("SPEAr Power Management GPIOs: Exit\n");
	for(index = 0; index <  ARRAY_SIZE(spear_gpio); index++)
		gpio_unexport(spear_gpio[index].gpio);
	
	gpio_free_array(spear_gpio, ARRAY_SIZE(spear_gpio));
}


MODULE_AUTHOR("Vincenzo Frascino <vincenzo.frascino@st.com>");
MODULE_DESCRIPTION("SPEAr PM GPIO driver");
MODULE_LICENSE("GPL");
module_init(spear_gpio_init);
module_exit(spear_gpio_exit);