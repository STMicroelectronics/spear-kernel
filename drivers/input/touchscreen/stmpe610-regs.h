/*
 * drivers/input/touchscreen/stmpe610-regs.h
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

#ifndef __STMPE610_REGS_H
#define __STMPE610_REGS_H

#include <linux/spi/spi.h>
#include <linux/device.h>
#include <linux/types.h>

/* Register Definitions */
/* System and Identification registers */
#define CHIP_ID		0x00	/* Device identification */
#define ID_VER		0x02	/* Revision number */
#define SYS_CTRL1	0x03	/* Reset control */
	#define HIBERNATE_MODE		(1 << 0)
	#define SOFT_RESET		(1 << 1)
#define SYS_CTRL2	0x04	/* Clock control */
	#define ADC_CLK_OFF		(1 << 0)
	#define TSC_CLK_OFF		(1 << 1)
	#define GPIO_CLK_OFF		(1 << 2)
	#define CLK_ENABLE		(0xc)
#define SPI_CFG		0x08	/* SPI interface configuration */
	#define SPI_MODE(x)		(x)
	#define SPI_AUTO_INC		(1 << 2)

/* Interrupt system registers */
#define INT_CTRL	0x09	/* Interrupt control register */
	#define GLOBAL_INT_EN		(1 << 0)
	#define EDGE_INT		(1 << 1)
	#define LEVEL_INT		(~EDGE_INT)
	#define ACTIVE_HIGH_RISING	(1 << 2)
	#define ACTIVE_LOW_FALLING	(~ACTIVE_HIGH_RISING)
#define INT_EN		0x0A	/* Interrupt enable register */
#define INT_STA		0x0B	/* interrupt status register */
	#define TOUCH_INT		(1 << 0)
	#define FIFO_TH_INT		(1 << 1)
	#define FIFO_OFLOW_INT		(1 << 2)
	#define FIFO_FULL_INT		(1 << 3)
	#define FIFO_EMPTY_INT		(1 << 4)
	#define ADC_INT_EN_MASK		(1 << 6)
	#define GPIO_INT_EN_MASK	(1 << 7)
#define GPIO_EN		0x0C	/* GPIO interrupt enable register */
#define GPIO_INT_STA	0x0D	/* GPIO interrupt status register */

/* GPIO system registers */
#define ADC_INT_EN	0x0E	/* ADC interrupt enable register */
#define ADC_INT_STA	0x0F	/* ADC interrupt status register */
#define GPIO_SET_PIN	0x10	/* GPIO set pin register */
#define GPIO_CLR_PIN	0x11	/* GPIO clear pin register */
#define GPIO_MP_STA	0x12	/* GPIO monitor pin state register */
#define GPIO_DIR	0x13	/* GPIO direction register */
#define GPIO_ED		0x14	/* GPIO edge detect register */
#define GPIO_RE		0x15	/* GPIO rising edge register */
#define GPIO_FE		0x16	/* GPIO falling edge register */
#define GPIO_AF		0x17	/* Alternate function register */

/* ADC system registers */
#define ADC_CTRL1	0x20	/* ADC control */
	#define SAMP_TIME_36		(0 << 4)
	#define SAMP_TIME_44		(1 << 4)
	#define SAMP_TIME_56		(2 << 4)
	#define SAMP_TIME_64		(3 << 4)
	#define SAMP_TIME_80		(4 << 4)
	#define SAMP_TIME_96		(5 << 4)
	#define SAMP_TIME_124		(6 << 4)
	#define MOD_10B			(0 << 3)
	#define MOD_12B			(1 << 3)
	#define REF_SEL_INT		(0 << 1)
	#define REF_SEL_EXT		(1 << 1)
#define ADC_CTRL2	0x21	/* ADC control */
	#define ADC_FREQ_1625K		0
	#define ADC_FREQ_3250K		1
	#define ADC_FREQ_6500K		2
#define ADC_CAPT	0x22	/* To initiate ADC data acquisition */
#define ADC_DATA_CH0	0x30	/* ADC channel 0 */
#define ADC_DATA_CH1	0x32	/* ADC channel 1 */
#define ADC_DATA_CH4	0x38	/* ADC channel 4 */
#define ADC_DATA_CH5	0x3A	/* ADC channel 5 */
#define ADC_DATA_CH6	0x3C	/* ADC channel 6 */
#define ADC_DATA_CH7	0x3E	/* ADC channel 7 */

/* TOUCHSCREEN Controller registers */
#define TSC_CTRL	0x40	/* 4-wire touchscreen controller setup */
	#define TSC_ENB			(1 << 0)
	#define OP_MODE(x)		(((x) & 0x7) << 1)
		#define XYZ_ACQUISITION		0
		#define XY_ACQUISITION		1
		#define X_ACQUISITION		2
		#define Y_ACQUISITION		3
		#define Z_ACQUISITION		4
	#define TRACKING_INDEX(x)		(((x) & 0x7) << 4)
		#define TI_0			0
		#define TI_4			1
		#define TI_8			2
		#define TI_16			3
		#define TI_32			4
		#define TI_64			5
		#define TI_92			6
		#define TI_127			7
	#define TOUCH_STATUS(x)			(((x) >> 0x7) & 1)
#define TSC_CFG		0x41	/* Touchscreen controller configuration */
	#define SETTLING_TIME(x)		(((x) & 0x7) << 0)
		#define ST_10US			0
		#define ST_100US		1
		#define ST_500US		2
		#define ST_1MS			3
		#define ST_5MS			4
		#define ST_10MS			5
		#define ST_50MS			6
		#define ST_100MS		7
	#define TOUCH_DET_DELAY(x)		(((x) & 0x7) << 3)
		#define TD_10US			0
		#define TD_50US			1
		#define TD_100US		2
		#define TD_500US		3
		#define TD_1MS			4
		#define TD_5MS			5
		#define TD_10MS			6
		#define TD_50MS			7
	#define AVG_CTRL(x)			(((x) & 0x3) << 6)
		#define SAMPLES_1		0
		#define SAMPLES_2		1
		#define SAMPLES_4		2
		#define SAMPLES_8		3
#define WDW_TR_X	0x42	/* Window setup for top right X */
#define WDW_TR_Y	0x44	/* Window setup for top right Y */
#define WDW_BL_X	0x46	/* Window setup for bottom left X */
#define WDW_BL_Y	0x48	/* Window setup for bottom left Y */
	#define XY_MAX	0xFFF
	#define Z_MAX	0xFF
#define FIFO_TH		0x4A	/* FIFO level to generate interrupt */
#define FIFO_STA	0x4B	/* Current status of FIFO */
	#define RESET_FIFO		(1 << 0)
	#define ENB_FIFO		(0)
	#define FIFO_TH_TRIG		(1 << 4) /* FIFO data is >= Threshold */
	#define FIFO_EMPTY		(1 << 5)
	#define FIFO_FULL		(1 << 6)
	#define FIFO_OFLOW		(1 << 7)
#define FIFO_SIZE	0x4C	/* Current filled level of FIFO */
#define TSC_DATA_X	0x4D	/* Data port for tsc data access */
#define TSC_DATA_Y	0x4F	/* Data port for tsc data access */
#define TSC_DATA_Z	0x51	/* Data port for tsc data access */
#define TSC_DATA_XYZ	0x52	/* Data port for tsc data access */
#define TSC_FRACT_Z	0x56	/* range and accuracy of pressure measurement */
#define TSC_DATA	0x57	/* Data port for tsc data access */
#define TSC_I_DRIVE	0x58	/* Touchscreen controller drive Current */
	#define MAX_I_35mA		0
	#define MAX_I_80mA		1
#define TSC_SHIELD	0x59	/* Touchscreen controller shield */

#define READ_CMD	(1 << 7)

struct stmpe610 {
	struct device		*dev;
	struct input_dev	*input;
	struct stmpe610_pdata	*pdata;
	char			phys[32];
	char			name[32];
	u32			event_count;
	u8			mode;
#define DATAX			0x1
#define DATAY			0x2
#define DATAZ			0x4
	void			*idata;		/* Interface Data */
	spinlock_t		lock;
	struct workqueue_struct	*wq;
	struct work_struct	work;
};

/* function declarations */
int stmpe610_block_read(struct spi_device *spi, u8 reg, u8 len, u8 *rx_buf);
int stmpe610_read(struct spi_device *spi, u8 reg);
int stmpe610_block_write(struct spi_device *spi, u8 reg, u8 len, u8 *tx_buf);
int stmpe610_write(struct spi_device *spi, u8 reg, u8 val);
struct stmpe610 *stmpe610_probe(struct device *dev);
void stmpe610_remove(struct device *dev);
void stmpe610_config(struct stmpe610 *ts);

#endif /* __STMPE610_REGS_H */
