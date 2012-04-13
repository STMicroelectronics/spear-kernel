/*
 * Synopsys DesignWare I2C adapter driver (master only).
 *
 * Copyright (C) 2011 ST Microelectronics.
 * Author:Vincenzo Frascino <vincenzo.frascino@st.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef I2C_DESIGNWARE_H
#define I2C_DESIGNWARE_H

#include <linux/platform_device.h>
#include <linux/i2c.h>

/*
 * struct i2c_dw_pdata - Designware I2c platform data
 * @scl_gpio: gpio number of the scl line.
 * @scl_gpio_flags: flag for gpio_request_one of scl_gpio. 0 implies
 *	GPIOF_OUT_INIT_LOW.
 * @get_gpio: called before recover_bus() to get padmux configured for scl line
 *	as gpio. Only required if is_gpio_recovery == true
 * @put_gpio: called after recover_bus() to get padmux configured for scl line
 *	as scl. Only required if is_gpio_recovery == true
 * @skip_sda_polling: if true, bus recovery will not poll sda line to check if
 *	it became high or not. Below are required only if this is false.
 * @sda_gpio: gpio number of the sda line.
 * @sda_gpio_flags: flag for gpio_request_one of sda_gpio. 0 implies
 *	GPIOF_OUT_INIT_LOW.
 */
struct i2c_dw_pdata {
	void (*i2c_recover_bus)(struct platform_device *pdev);
	int (*recover_bus)(struct i2c_adapter *);
	int scl_gpio;
	int scl_gpio_flags;
	int (*get_gpio)(unsigned gpio);
	int (*put_gpio)(unsigned gpio);

	/* sda polling specific */
	bool is_gpio_recovery;
	bool skip_sda_polling;
	int sda_gpio;
	int sda_gpio_flags;
};
#endif /* I2C_DESIGNWARE_H */
