/*
 * arch/arm/mach-spear6xx/spear600.c
 *
 * SPEAr600 machine source file
 *
 * Copyright (C) 2009 ST Microelectronics
 * Rajeev Kumar<rajeev-dlh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/amba/pl08x.h>
#include <plat/pl080.h>
#include <mach/generic.h>

/* Add spear600 specific devices here */
/* DMAC device registration */
static struct pl08x_channel_data pl080_slave_channels[] = {
	{
		.bus_id = "ssp1_rx",
		.min_signal = 0,
		.max_signal = 0,
		.muxval = 0,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "ssp1_tx",
		.min_signal = 1,
		.max_signal = 1,
		.muxval = 0,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "uart0_rx",
		.min_signal = 2,
		.max_signal = 2,
		.muxval = 0,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "uart0_tx",
		.min_signal = 3,
		.max_signal = 3,
		.muxval = 0,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "uart1_rx",
		.min_signal = 4,
		.max_signal = 4,
		.muxval = 0,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "uart1_tx",
		.min_signal = 5,
		.max_signal = 5,
		.muxval = 0,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "ssp2_rx",
		.min_signal = 6,
		.max_signal = 6,
		.muxval = 0,
		.cctl = 0,
		.periph_buses = PL08X_AHB2,
	}, {
		.bus_id = "ssp2_tx",
		.min_signal = 7,
		.max_signal = 7,
		.muxval = 0,
		.cctl = 0,
		.periph_buses = PL08X_AHB2,
	}, {
		.bus_id = "ssp0_rx",
		.min_signal = 8,
		.max_signal = 8,
		.muxval = 0,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "ssp0_tx",
		.min_signal = 9,
		.max_signal = 9,
		.muxval = 0,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "i2c_rx",
		.min_signal = 10,
		.max_signal = 10,
		.muxval = 0,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "i2c_tx",
		.min_signal = 11,
		.max_signal = 11,
		.muxval = 0,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "irda",
		.min_signal = 12,
		.max_signal = 12,
		.muxval = 0,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "adc",
		.min_signal = 13,
		.max_signal = 13,
		.muxval = 0,
		.cctl = 0,
		.periph_buses = PL08X_AHB2,
	}, {
		.bus_id = "to_jpeg",
		.min_signal = 14,
		.max_signal = 14,
		.muxval = 0,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "from_jpeg",
		.min_signal = 15,
		.max_signal = 15,
		.muxval = 0,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "ras0_rx",
		.min_signal = 0,
		.max_signal = 0,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "ras0_tx",
		.min_signal = 1,
		.max_signal = 1,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "ras1_rx",
		.min_signal = 2,
		.max_signal = 2,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "ras1_tx",
		.min_signal = 3,
		.max_signal = 3,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "ras2_rx",
		.min_signal = 4,
		.max_signal = 4,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "ras2_tx",
		.min_signal = 5,
		.max_signal = 5,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "ras3_rx",
		.min_signal = 6,
		.max_signal = 6,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "ras3_tx",
		.min_signal = 7,
		.max_signal = 7,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "ras4_rx",
		.min_signal = 8,
		.max_signal = 8,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "ras4_tx",
		.min_signal = 9,
		.max_signal = 9,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "ras5_rx",
		.min_signal = 10,
		.max_signal = 10,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "ras5_tx",
		.min_signal = 11,
		.max_signal = 11,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "ras6_rx",
		.min_signal = 12,
		.max_signal = 12,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "ras6_tx",
		.min_signal = 13,
		.max_signal = 13,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "ras7_rx",
		.min_signal = 14,
		.max_signal = 14,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "ras7_tx",
		.min_signal = 15,
		.max_signal = 15,
		.muxval = 1,
		.cctl = 0,
		.periph_buses = PL08X_AHB1,
	}, {
		.bus_id = "ext0_rx",
		.min_signal = 0,
		.max_signal = 0,
		.muxval = 2,
		.cctl = 0,
		.periph_buses = PL08X_AHB2,
	}, {
		.bus_id = "ext0_tx",
		.min_signal = 1,
		.max_signal = 1,
		.muxval = 2,
		.cctl = 0,
		.periph_buses = PL08X_AHB2,
	}, {
		.bus_id = "ext1_rx",
		.min_signal = 2,
		.max_signal = 2,
		.muxval = 2,
		.cctl = 0,
		.periph_buses = PL08X_AHB2,
	}, {
		.bus_id = "ext1_tx",
		.min_signal = 3,
		.max_signal = 3,
		.muxval = 2,
		.cctl = 0,
		.periph_buses = PL08X_AHB2,
	}, {
		.bus_id = "ext2_rx",
		.min_signal = 4,
		.max_signal = 4,
		.muxval = 2,
		.cctl = 0,
		.periph_buses = PL08X_AHB2,
	}, {
		.bus_id = "ext2_tx",
		.min_signal = 5,
		.max_signal = 5,
		.muxval = 2,
		.cctl = 0,
		.periph_buses = PL08X_AHB2,
	}, {
		.bus_id = "ext3_rx",
		.min_signal = 6,
		.max_signal = 6,
		.muxval = 2,
		.cctl = 0,
		.periph_buses = PL08X_AHB2,
	}, {
		.bus_id = "ext3_tx",
		.min_signal = 7,
		.max_signal = 7,
		.muxval = 2,
		.cctl = 0,
		.periph_buses = PL08X_AHB2,
	}, {
		.bus_id = "ext4_rx",
		.min_signal = 8,
		.max_signal = 8,
		.muxval = 2,
		.cctl = 0,
		.periph_buses = PL08X_AHB2,
	}, {
		.bus_id = "ext4_tx",
		.min_signal = 9,
		.max_signal = 9,
		.muxval = 2,
		.cctl = 0,
		.periph_buses = PL08X_AHB2,
	}, {
		.bus_id = "ext5_rx",
		.min_signal = 10,
		.max_signal = 10,
		.muxval = 2,
		.cctl = 0,
		.periph_buses = PL08X_AHB2,
	}, {
		.bus_id = "ext5_tx",
		.min_signal = 11,
		.max_signal = 11,
		.muxval = 2,
		.cctl = 0,
		.periph_buses = PL08X_AHB2,
	}, {
		.bus_id = "ext6_rx",
		.min_signal = 12,
		.max_signal = 12,
		.muxval = 2,
		.cctl = 0,
		.periph_buses = PL08X_AHB2,
	}, {
		.bus_id = "ext6_tx",
		.min_signal = 13,
		.max_signal = 13,
		.muxval = 2,
		.cctl = 0,
		.periph_buses = PL08X_AHB2,
	}, {
		.bus_id = "ext7_rx",
		.min_signal = 14,
		.max_signal = 14,
		.muxval = 2,
		.cctl = 0,
		.periph_buses = PL08X_AHB2,
	}, {
		.bus_id = "ext7_tx",
		.min_signal = 15,
		.max_signal = 15,
		.muxval = 2,
		.cctl = 0,
		.periph_buses = PL08X_AHB2,
	},
};

void __init spear600_init(void)
{
	/* call spear6xx family common init function */
	spear6xx_init();

	/* Set DMAC platform data's slave info */
	pl080_set_slaveinfo(&dma_device, pl080_slave_channels,
			ARRAY_SIZE(pl080_slave_channels));
}
