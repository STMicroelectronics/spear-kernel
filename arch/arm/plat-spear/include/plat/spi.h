/*
 * arch/arm/plat-spear/include/plat/spi.h
 *
 * SPI board specific definitions common to multiple boards on multiple
 * machines.
 *
 * Copyright (C) 2010 ST Microelectronics
 * Viresh Kumar<viresh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __PLAT_SPI_H
#define __PLAT_SPI_H

#include <linux/amba/pl022.h>
#include <linux/gpio.h>

/* spi board information */
static inline int spi_cs_gpio_request(u32 gpio_pin)
{
	int ret;

	ret = gpio_request(gpio_pin, "SPI_CS");
	if (ret < 0) {
		printk(KERN_ERR "SPI: gpio:%d request fail\n", gpio_pin);
		return ret;
	} else {
		ret = gpio_direction_output(gpio_pin, 1);
		if (ret) {
			printk(KERN_ERR "SPI: gpio:%d direction set fail\n",
					gpio_pin);
			return ret;
		}
	}
	return 0;
}

/* This will define cs_control function for a specific spi slave */
#define DECLARE_SPI_CS_CONTROL(id, type, gpio)		\
static void spi##id##_##type##_cs_control(u32 control)	\
{							\
	static int count, ret;				\
							\
	if (unlikely(!count)) {				\
		count++;				\
		ret = spi_cs_gpio_request(gpio);	\
	}						\
							\
	if (!ret)					\
		gpio_set_value(gpio, control);		\
}

/* This will define CHIP_INFO structure for a specific spi slave */
#define DECLARE_SPI_CHIP_INFO(id, type, chip_select_control)	\
struct pl022_config_chip spi##id##_##type##_chip_info = {	\
	.lbm = LOOPBACK_DISABLED,			\
	.iface = SSP_INTERFACE_MOTOROLA_SPI,		\
	.hierarchy = SSP_MASTER,			\
	.slave_tx_disable = 0,				\
	.endian_tx = SSP_TX_LSB,			\
	.endian_rx = SSP_RX_LSB,			\
	.data_size = SSP_DATA_BITS_8,			\
	.com_mode = INTERRUPT_TRANSFER,			\
	.rx_lev_trig = SSP_RX_1_OR_MORE_ELEM,		\
	.tx_lev_trig = SSP_TX_1_OR_MORE_EMPTY_LOC,	\
	.clk_phase = SSP_CLK_SECOND_EDGE,		\
	.clk_pol = SSP_CLK_POL_IDLE_LOW,		\
	.cs_control = chip_select_control,		\
};

#define DECLARE_SPI_CHIP_INFO_NULL_ID(chip_select_control)	\
DECLARE_SPI_CHIP_INFO(, chip_select_control)

#endif /* __PLAT_SPI_H */
