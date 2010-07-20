/*
 * drivers/input/touchscreen/stmpe610.h
 *
 * ST Microelectronics S-Touch Advance Touchscreen controller's header file
 *
 * Copyright (C) 2010 ST Microelectronics
 * Viresh Kumar <viresh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __STMPE610_H
#define __STMPE610_H

/**
 * struct stmpe610_pdata - stmpe610 touch screen controller platform
 * data
 *
 * @irq_gpio: gpio pin to be used for touch interrupt
 * @irq_type: irq type of gpio pin
 * @fifo_threshhold: Fifo threshhold for interrupt trigger.
 * @tracking_index: tracking index
 * @operating_mode: operating mode.
 * @average_ctrl: Sample average control
 * @touch_det_delay: Touch detect interrupt delay. recommended is 500 us
 * @settling_time: Panel driver settling time. recommended is 500 us
 * @x_min: xmin of screen
 * @x_max: xmax of screen
 * @y_min: ymin of screen
 * @y_max: ymax of screen
 * @sample_time: ADC converstion time in number of clock. recommended is 80
 * @mod_12b: ADC Bit mode
 * @ref_sel: ADC reference source
 * @adc_freq: ADC Clock speed
 * @fraction_z: Length of the fractional part in z. recommended is 7
 * @i_drive: current limit value of the touchscreen drivers.
 */
struct stmpe610_pdata {
	u32	irq_gpio;
	u32	irq_type;
	u16	fifo_threshhold;	/* 1 to 255 */
	u8	tracking_index;
#define TI_0		0
#define TI_4		1
#define TI_8		2
#define TI_16		3
#define TI_32		4
#define TI_64		5
#define TI_92		6
#define TI_127		7
	u8	operating_mode;
#define XYZ_ACQUISITION	0
#define XY_ACQUISITION	1
#define X_ACQUISITION	2
#define Y_ACQUISITION	3
#define Z_ACQUISITION	4
	u8	average_ctrl;
#define SAMPLES_1	0
#define SAMPLES_2	1
#define SAMPLES_4	2
#define SAMPLES_8	3
	u8	touch_det_delay;
#define TD_10US		0
#define TD_50US		1
#define TD_100US	2
#define TD_500US	3
#define TD_1MS		4
#define TD_5MS		5
#define TD_10MS		6
#define TD_50MS		7
	u8	settling_time;
#define ST_10US		0
#define ST_100US	1
#define ST_500US	2
#define ST_1MS		3
#define ST_5MS		4
#define ST_10MS		5
#define ST_50MS		6
#define ST_100MS	7
	u16	x_min, x_max;
	u16	y_min, y_max;
	u8 sample_time;
#define SAMP_TIME_36	(0 << 4)
#define SAMP_TIME_44	(1 << 4)
#define SAMP_TIME_56	(2 << 4)
#define SAMP_TIME_64	(3 << 4)
#define SAMP_TIME_80	(4 << 4)
#define SAMP_TIME_96	(5 << 4)
#define SAMP_TIME_124	(6 << 4)
	u8 mod_12b;
#define MOD_10B		(0 << 3)
#define MOD_12B		(1 << 3)
	u8 ref_sel;
#define REF_SEL_INT	(0 << 1)
#define REF_SEL_EXT	(1 << 1)
	u8 adc_freq;
#define ADC_FREQ_1625K	0
#define ADC_FREQ_3250K	1
#define ADC_FREQ_6500K	2
	u8 fraction_z;
	u8 i_drive;
#define IDRIVE_20_35MA	0
#define IDRIVE_50_80MA	1
};

#endif /* __STMPE610_H */
