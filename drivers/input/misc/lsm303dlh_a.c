/*
 * lsm303dlh_a.c
 * ST 3-Axis Accelerometer Driver
 *
 * Copyright (C) 2010 STMicroelectronics
 * Author: Carmine Iascone (carmine.iascone@st.com)
 * Author: Matteo Dameno (matteo.dameno@st.com)
 *
 * Copyright (C) 2010 STEricsson
 * Author: Mian Yousaf Kaukab <mian.yousaf.kaukab@stericsson.com>
 * Updated:Preetham Rao Kaskurthi <preetham.rao@stericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>

#ifdef CONFIG_INPUT_ST_LSM303DLH_INPUT_DEVICE
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#endif

#include <linux/i2c/lsm303dlh.h>
#include <linux/regulator/consumer.h>

/* lsm303dlh accelerometer registers */
#define WHO_AM_I	0x0F

/* ctrl 1: pm2 pm1 pm0 dr1 dr0 zenable yenable zenable */
#define CTRL_REG1	0x20	/* power control reg */
#define CTRL_REG2	0x21	/* power control reg */
#define CTRL_REG3	0x22	/* power control reg */
#define CTRL_REG4	0x23	/* interrupt control reg */
#define CTRL_REG5	0x24	/* interrupt control reg */

#define STATUS_REG	0x27	/* status register */

#define AXISDATA_REG	0x28	/* axis data */

#define INT1_CFG	0x30	/* interrupt 1 configuration */
#define INT1_SRC	0x31	/* interrupt 1 source reg */
#define INT1_THS	0x32	/* interrupt 1 threshold */
#define INT1_DURATION	0x33	/* interrupt 1 duration */

#define INT2_CFG	0x34	/* interrupt 2 configuration */
#define INT2_SRC	0x35	/* interrupt 2 source reg */
#define INT2_THS	0x36	/* interrupt 2 threshold */
#define INT2_DURATION	0x37	/* interrupt 2 duration */

/* Sensitivity adjustment */
#define SHIFT_ADJ_2G	4 /*	1/16*/
#define SHIFT_ADJ_4G	3 /*	2/16*/
#define SHIFT_ADJ_8G	2 /* ~3.9/16*/

/* Control register 1 */
#define LSM303DLH_A_CR1_PM_BIT 5
#define LSM303DLH_A_CR1_PM_MASK (0x7 << LSM303DLH_A_CR1_PM_BIT)
#define LSM303DLH_A_CR1_DR_BIT 3
#define LSM303DLH_A_CR1_DR_MASK (0x3 << LSM303DLH_A_CR1_DR_BIT)
#define LSM303DLH_A_CR1_EN_BIT 0
#define LSM303DLH_A_CR1_EN_MASK (0x7 << LSM303DLH_A_CR1_EN_BIT)

/* Control register 2 */
#define LSM303DLH_A_CR4_ST_BIT 1
#define LSM303DLH_A_CR4_ST_MASK (0x1 << LSM303DLH_A_CR4_ST_BIT)
#define LSM303DLH_A_CR4_STS_BIT 3
#define LSM303DLH_A_CR4_STS_MASK (0x1 << LSM303DLH_A_CR4_STS_BIT)
#define LSM303DLH_A_CR4_FS_BIT 4
#define LSM303DLH_A_CR4_FS_MASK (0x3 << LSM303DLH_A_CR4_FS_BIT)
#define LSM303DLH_A_CR4_BLE_BIT 6
#define LSM303DLH_A_CR4_BLE_MASK (0x3 << LSM303DLH_A_CR4_BLE_BIT)
#define LSM303DLH_A_CR4_BDU_BIT 7
#define LSM303DLH_A_CR4_BDU_MASK (0x1 << LSM303DLH_A_CR4_BDU_BIT)

/* Control register 3 */
#define LSM303DLH_A_CR3_I1_BIT 0
#define LSM303DLH_A_CR3_I1_MASK (0x3 << LSM303DLH_A_CR3_I1_BIT)
#define LSM303DLH_A_CR3_LIR1_BIT 2
#define LSM303DLH_A_CR3_LIR1_MASK (0x1 << LSM303DLH_A_CR3_LIR1_BIT)
#define LSM303DLH_A_CR3_I2_BIT 3
#define LSM303DLH_A_CR3_I2_MASK (0x3 << LSM303DLH_A_CR3_I2_BIT)
#define LSM303DLH_A_CR3_LIR2_BIT 5
#define LSM303DLH_A_CR3_LIR2_MASK (0x1 << LSM303DLH_A_CR3_LIR2_BIT)
#define LSM303DLH_A_CR3_PPOD_BIT 6
#define LSM303DLH_A_CR3_PPOD_MASK (0x1 << LSM303DLH_A_CR3_PPOD_BIT)
#define LSM303DLH_A_CR3_IHL_BIT 7
#define LSM303DLH_A_CR3_IHL_MASK (0x1 << LSM303DLH_A_CR3_IHL_BIT)

#define LSM303DLH_A_CR3_I_SELF 0x0
#define LSM303DLH_A_CR3_I_OR	0x1
#define LSM303DLH_A_CR3_I_DATA 0x2
#define LSM303DLH_A_CR3_I_BOOT 0x3

#define LSM303DLH_A_CR3_LIR_LATCH 0x1

/* Range */
#define LSM303DLH_A_RANGE_2G 0x00
#define LSM303DLH_A_RANGE_4G 0x01
#define LSM303DLH_A_RANGE_8G 0x03

/* Mode */
#define LSM303DLH_A_MODE_OFF 0x00
#define LSM303DLH_A_MODE_NORMAL 0x01
#define LSM303DLH_A_MODE_LP_HALF 0x02
#define LSM303DLH_A_MODE_LP_1 0x03
#define LSM303DLH_A_MODE_LP_2 0x04
#define LSM303DLH_A_MODE_LP_5 0x05
#define LSM303DLH_A_MODE_LP_10 0x06

/* Rate */
#define LSM303DLH_A_RATE_50 0x00
#define LSM303DLH_A_RATE_100 0x01
#define LSM303DLH_A_RATE_400 0x02
#define LSM303DLH_A_RATE_1000 0x03

/* Axis */
#define LSM303DLH_A_X_AXIS 0x01
#define LSM303DLH_A_Y_AXIS 0x02
#define LSM303DLH_A_Z_AXIS 0x04
#define LSM303DLH_A_ALL_AXIS (LSM303DLH_A_X_AXIS | \
		LSM303DLH_A_Y_AXIS | LSM303DLH_A_Z_AXIS)

/* Multiple byte transfer enable */
#define MULTIPLE_I2C_TR 0x80

/* 2g: Range -2048 to 2047 */
/* 4g: Range -5196 to 5195 */
/* 8g: Range -10392 to 10391 */
struct lsm303dlh_a_t {
	short	x;
	short	y;
	short	z;
};

struct lsm303dlh_a_data *lsm303dlh_a_ddata;

/*
 * accelerometer local data
 */
struct lsm303dlh_a_data {
	struct i2c_client *client;
	/* lock for sysfs operations */
	struct mutex lock;
	struct lsm303dlh_a_t data;

#ifdef CONFIG_INPUT_ST_LSM303DLH_INPUT_DEVICE
	struct input_dev *input_dev;
	struct input_dev *input_dev2;
	struct delayed_work input_work;
#endif
	struct lsm303dlh_platform_data pdata;
	struct regulator *regulator;

	unsigned char range;
	unsigned char mode;
	unsigned char rate;
	int shift_adjust;

	int input_poll_dev;
	unsigned int poll_dev_delay;

	unsigned char interrupt_control;
	unsigned int interrupt_channel;

	unsigned char interrupt_configure[2];
	unsigned char interrupt_duration[2];
	unsigned char interrupt_threshold[2];
};

static int lsm303dlh_a_write(struct lsm303dlh_a_data *ddata, u8 reg,
		u8 val, char *msg)
{
	int ret = i2c_smbus_write_byte_data(ddata->client, reg, val);
	if (ret < 0)
		dev_err(&ddata->client->dev,
			"i2c_smbus_write_byte_data failed error %d\
			Register (%s)\n", ret, msg);
	return ret;
}

static int lsm303dlh_a_read(struct lsm303dlh_a_data *ddata, u8 reg,
		char *msg)
{
	int ret = i2c_smbus_read_byte_data(ddata->client, reg);
	if (ret < 0)
		dev_err(&ddata->client->dev,
			"i2c_smbus_read_byte_data failed error %d\
			 Register (%s)\n", ret, msg);
	return ret;
}

static int lsm303dlh_a_restore(struct lsm303dlh_a_data *ddata)
{
	unsigned char reg;
	unsigned char shifted_mode = (ddata->mode << LSM303DLH_A_CR1_PM_BIT);
	unsigned char shifted_rate = (ddata->rate << LSM303DLH_A_CR1_DR_BIT);
	unsigned char axis = LSM303DLH_A_ALL_AXIS;
	unsigned char context = (shifted_mode | shifted_rate | axis);
	int ret = 0;

	/* BDU should be enabled by default/recommened */
	reg = ddata->range;
	reg |= LSM303DLH_A_CR4_BDU_MASK;

	ret = lsm303dlh_a_write(ddata, CTRL_REG1, context, "CTRL_REG1");
	if (ret < 0)
		goto fail;

	ret = lsm303dlh_a_write(ddata, CTRL_REG4, reg, "CTRL_REG4");
	if (ret < 0)
		goto fail;

	ret = lsm303dlh_a_write(ddata, CTRL_REG3, ddata->interrupt_control,
			"CTRL_REG3");
	if (ret < 0)
		goto fail;

	ret = lsm303dlh_a_write(ddata, INT1_CFG,
			ddata->interrupt_configure[0], "INT1_CFG");
	if (ret < 0)
		goto fail;

	ret = lsm303dlh_a_write(ddata, INT2_CFG,
			ddata->interrupt_configure[1], "INT2_CFG");
	if (ret < 0)
		goto fail;

	ret = lsm303dlh_a_write(ddata, INT1_THS,
			ddata->interrupt_threshold[0], "INT1_THS");
	if (ret < 0)
		goto fail;

	ret = lsm303dlh_a_write(ddata, INT2_THS,
			ddata->interrupt_threshold[1], "INT2_THS");
	if (ret < 0)
		goto fail;

	ret = lsm303dlh_a_write(ddata, INT1_DURATION,
			ddata->interrupt_duration[0], "INT1_DURATION");
	if (ret < 0)
		goto fail;

	ret = lsm303dlh_a_write(ddata, INT1_DURATION,
			ddata->interrupt_duration[1], "INT1_DURATION");
	if (ret < 0)
		goto fail;

fail:
	return ret;
}

static int lsm303dlh_a_readdata(struct lsm303dlh_a_data *ddata)
{
	unsigned char acc_data[6];
	short data[3];

	int ret = i2c_smbus_read_i2c_block_data(ddata->client,
			AXISDATA_REG | MULTIPLE_I2C_TR, 6, acc_data);
	if (ret < 0) {
		dev_err(&ddata->client->dev,
			"i2c_smbus_read_i2c_block_data failed error \
			%d Register AXISDATA_REG\n", ret);
		return ret;
	}

	data[0] = (short) (((acc_data[1]) << 8) | acc_data[0]);
	data[1] = (short) (((acc_data[3]) << 8) | acc_data[2]);
	data[2] = (short) (((acc_data[5]) << 8) | acc_data[4]);

	data[0] >>= ddata->shift_adjust;
	data[1] >>= ddata->shift_adjust;
	data[2] >>= ddata->shift_adjust;

	/* taking position and orientation of x, y, z axis into account*/
	if (ddata->pdata.axis_map_x || ddata->pdata.axis_map_y ||
			ddata->pdata.axis_map_z) {
		data[ddata->pdata.axis_map_x] = ddata->pdata.negative_x ?
			-data[ddata->pdata.axis_map_x] :
			 data[ddata->pdata.axis_map_x];
		data[ddata->pdata.axis_map_y] = ddata->pdata.negative_y ?
			-data[ddata->pdata.axis_map_y] :
			 data[ddata->pdata.axis_map_y];
		data[ddata->pdata.axis_map_z] = ddata->pdata.negative_z ?
			-data[ddata->pdata.axis_map_z] :
			 data[ddata->pdata.axis_map_z];

		ddata->data.x = data[ddata->pdata.axis_map_x];
		ddata->data.y = data[ddata->pdata.axis_map_y];
		ddata->data.z = data[ddata->pdata.axis_map_z];
	} else {
		ddata->data.x = data[0];
		ddata->data.y = data[1];
		ddata->data.z = data[2];
	}

	return ret;
}

static ssize_t lsm303dlh_a_show_data(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct lsm303dlh_a_data *ddata = platform_get_drvdata(pdev);
	int ret = 0;

	if (ddata->mode == LSM303DLH_A_MODE_OFF) {
		dev_info(&ddata->client->dev,
			 "device is switched off, make it ON using MODE.\n");
		return ret;
	}

	mutex_lock(&ddata->lock);

	ret = lsm303dlh_a_readdata(ddata);

	if (ret < 0) {
		mutex_unlock(&ddata->lock);
		return ret;
	}

	mutex_unlock(&ddata->lock);

	return sprintf(buf, "(%d, %d, %d)\n", ddata->data.x, ddata->data.y,
			ddata->data.z);
}

#ifdef CONFIG_INPUT_ST_LSM303DLH_INPUT_DEVICE
static irqreturn_t lsm303dlh_a_gpio_irq(int irq, void *device_data)
{

	struct lsm303dlh_a_data *ddata = device_data;
	int ret;
	unsigned char reg;
	struct input_dev *input;

	/* know your interrupt source */
	if (irq == gpio_to_irq(ddata->pdata.irq_a1)) {
		reg = INT1_SRC;
		input = ddata->input_dev;
	} else if (irq == gpio_to_irq(ddata->pdata.irq_a2)) {
		reg = INT2_SRC;
		input = ddata->input_dev2;
	} else {
		dev_err(&ddata->client->dev, "spurious interrupt");
		return IRQ_HANDLED;
	}

	/* read the axis */
	ret = lsm303dlh_a_readdata(ddata);
	if (ret < 0)
		dev_err(&ddata->client->dev,
				"reading data of xyz failed error %d\n", ret);

	input_report_abs(input, ABS_X, ddata->data.x);
	input_report_abs(input, ABS_Y, ddata->data.y);
	input_report_abs(input, ABS_Z, ddata->data.z);
	input_sync(input);

	/* clear the value by reading it */
	ret = lsm303dlh_a_read(ddata, reg, "INTTERUPT SOURCE");
	if (ret < 0)
		dev_err(&ddata->client->dev,
			"clearing interrupt source failed error %d\n", ret);

	return IRQ_HANDLED;

}

static void lsm303dlh_a_work_function(struct work_struct *work)
{
	struct input_dev *input = lsm303dlh_a_ddata->input_dev;

	lsm303dlh_a_readdata(lsm303dlh_a_ddata);

	input_report_abs(input, ABS_X, lsm303dlh_a_ddata->data.x);
	input_report_abs(input, ABS_Y, lsm303dlh_a_ddata->data.y);
	input_report_abs(input, ABS_Z, lsm303dlh_a_ddata->data.z);
	input_sync(input);
	schedule_delayed_work(&lsm303dlh_a_ddata->input_work,
		msecs_to_jiffies(lsm303dlh_a_ddata->poll_dev_delay));
}
#endif

static ssize_t lsm303dlh_a_show_interrupt_control(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct lsm303dlh_a_data *ddata = platform_get_drvdata(pdev);

	return sprintf(buf, "%d\n", ddata->interrupt_control);
}

static ssize_t lsm303dlh_a_store_interrupt_control(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct lsm303dlh_a_data *ddata = platform_get_drvdata(pdev);
	unsigned long val;
	int error;

	if (ddata->mode == LSM303DLH_A_MODE_OFF) {
		dev_info(&ddata->client->dev,
			"device is switched off, make it ON using MODE");
		return count;
	}

	error = strict_strtoul(buf, 0, &val);
	if (error)
		return error;

	mutex_lock(&ddata->lock);

	ddata->interrupt_control = val;

	error = lsm303dlh_a_write(ddata, CTRL_REG3, val, "CTRL_REG3");
	if (error < 0) {
		mutex_unlock(&ddata->lock);
		return error;
	}

	mutex_unlock(&ddata->lock);

	return count;
}

static ssize_t lsm303dlh_a_show_interrupt_channel(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct lsm303dlh_a_data *ddata = platform_get_drvdata(pdev);

	return sprintf(buf, "%d\n", ddata->interrupt_channel);
}

static ssize_t lsm303dlh_a_store_interrupt_channel(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct lsm303dlh_a_data *ddata = platform_get_drvdata(pdev);
	unsigned long val;
	int error;

	error = strict_strtoul(buf, 0, &val);
	if (error)
		return error;

	ddata->interrupt_channel = val;

	return count;
}

static ssize_t lsm303dlh_a_show_interrupt_configure(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct lsm303dlh_a_data *ddata = platform_get_drvdata(pdev);

	return sprintf(buf, "%d\n",
			ddata->interrupt_configure[ddata->interrupt_channel]);
}

static ssize_t lsm303dlh_a_store_interrupt_configure(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct lsm303dlh_a_data *ddata = platform_get_drvdata(pdev);
	unsigned long val;
	int error;

	if (ddata->mode == LSM303DLH_A_MODE_OFF) {
		dev_info(&ddata->client->dev,
			"device is switched off, make it ON using MODE");
		return count;
	}

	error = strict_strtoul(buf, 0, &val);
	if (error)
		return error;

	mutex_lock(&ddata->lock);

	ddata->interrupt_configure[ddata->interrupt_channel] = val;

	if (ddata->interrupt_channel == 0x0)
		error = lsm303dlh_a_write(ddata, INT1_CFG, val, "INT1_CFG");
	else
		error = lsm303dlh_a_write(ddata, INT2_CFG, val, "INT2_CFG");

	if (error < 0) {
		mutex_unlock(&ddata->lock);
		return error;
	}

	mutex_unlock(&ddata->lock);

	return count;
}

static ssize_t lsm303dlh_a_show_interrupt_duration(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct lsm303dlh_a_data *ddata = platform_get_drvdata(pdev);

	return sprintf(buf, "%d\n",
			ddata->interrupt_duration[ddata->interrupt_channel]);
}

static ssize_t lsm303dlh_a_store_interrupt_duration(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct lsm303dlh_a_data *ddata = platform_get_drvdata(pdev);
	unsigned long val;
	int error;

	if (ddata->mode == LSM303DLH_A_MODE_OFF) {
		dev_info(&ddata->client->dev,
			"device is switched off, make it ON using MODE");
		return count;
	}

	error = strict_strtoul(buf, 0, &val);
	if (error)
		return error;

	mutex_lock(&ddata->lock);

	ddata->interrupt_duration[ddata->interrupt_channel] = val;

	if (ddata->interrupt_channel == 0x0)
		error = lsm303dlh_a_write(ddata, INT1_DURATION, val,
				"INT1_DURATION");
	else
		error = lsm303dlh_a_write(ddata, INT2_DURATION, val,
				"INT2_DURATION");

	if (error < 0) {
		mutex_unlock(&ddata->lock);
		return error;
	}

	mutex_unlock(&ddata->lock);

	return count;
}

static ssize_t lsm303dlh_a_show_interrupt_threshold(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct lsm303dlh_a_data *ddata = platform_get_drvdata(pdev);

	return sprintf(buf, "%d\n",
			ddata->interrupt_threshold[ddata->interrupt_channel]);
}

static ssize_t lsm303dlh_a_store_interrupt_threshold(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct lsm303dlh_a_data *ddata = platform_get_drvdata(pdev);
	unsigned long val;
	int error;

	if (ddata->mode == LSM303DLH_A_MODE_OFF) {
		dev_info(&ddata->client->dev,
			"device is switched off, make it ON using MODE");
		return count;
	}

	error = strict_strtoul(buf, 0, &val);
	if (error)
		return error;

	mutex_lock(&ddata->lock);

	ddata->interrupt_threshold[ddata->interrupt_channel] = val;

	if (ddata->interrupt_channel == 0x0)
		error = lsm303dlh_a_write(ddata, INT1_THS, val, "INT1_THS");
	else
		error = lsm303dlh_a_write(ddata, INT2_THS, val, "INT2_THS");

	if (error < 0) {
		mutex_unlock(&ddata->lock);
		return error;
	}

	mutex_unlock(&ddata->lock);

	return count;
}

static ssize_t lsm303dlh_a_show_range(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct lsm303dlh_a_data *ddata = platform_get_drvdata(pdev);

	return sprintf(buf, "%d\n", ddata->range >> LSM303DLH_A_CR4_FS_BIT);
}

static ssize_t lsm303dlh_a_store_range(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct lsm303dlh_a_data *ddata = platform_get_drvdata(pdev);
	unsigned long val;
	unsigned long bdu_enabled_val;
	int error;

	if (ddata->mode == LSM303DLH_A_MODE_OFF) {
		dev_info(&ddata->client->dev,
			 "device is switched off, make it ON using MODE.\n");
		return count;
	}

	error = strict_strtoul(buf, 0, &val);
	if (error)
		return error;

	if (!(val == LSM303DLH_A_RANGE_2G || val == LSM303DLH_A_RANGE_4G ||
				val == LSM303DLH_A_RANGE_8G))
		return -EINVAL;

	mutex_lock(&ddata->lock);

	ddata->range = val;
	ddata->range <<= LSM303DLH_A_CR4_FS_BIT;

	/*
	 * Block mode update is recommended for not
	 * ending up reading different values
	 */
	bdu_enabled_val = ddata->range;
	bdu_enabled_val |= LSM303DLH_A_CR4_BDU_MASK;

	error = lsm303dlh_a_write(ddata, CTRL_REG4, bdu_enabled_val,
				"CTRL_REG4");
	if (error < 0) {
		mutex_unlock(&ddata->lock);
		return error;
	}

	switch (val) {
	case LSM303DLH_A_RANGE_2G:
		ddata->shift_adjust = SHIFT_ADJ_2G;
		break;
	case LSM303DLH_A_RANGE_4G:
		ddata->shift_adjust = SHIFT_ADJ_4G;
		break;
	case LSM303DLH_A_RANGE_8G:
		ddata->shift_adjust = SHIFT_ADJ_8G;
		break;
	default:
		mutex_unlock(&ddata->lock);
		return -EINVAL;
	}

	mutex_unlock(&ddata->lock);

	return count;
}

static ssize_t lsm303dlh_a_show_mode(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct lsm303dlh_a_data *ddata = platform_get_drvdata(pdev);

	return sprintf(buf, "%d\n", ddata->mode);
}

static ssize_t lsm303dlh_a_store_mode(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct lsm303dlh_a_data *ddata = platform_get_drvdata(pdev);
	unsigned long val;
	unsigned char data;
	int error;

	error = strict_strtoul(buf, 0, &val);
	if (error)
		return error;

	/* not in correct range */

	if (val < LSM303DLH_A_MODE_OFF || val > LSM303DLH_A_MODE_LP_10)
		return -EINVAL;

	/* if same mode as existing, return */
	if (ddata->mode == val)
		return count;

	/* turn on the supplies if already off */
	if (ddata->mode == LSM303DLH_A_MODE_OFF) {
		if (ddata->regulator)
			regulator_enable(ddata->regulator);

#ifdef CONFIG_INPUT_ST_LSM303DLH_INPUT_DEVICE
		if (!ddata->input_poll_dev) {
			enable_irq(gpio_to_irq(ddata->pdata.irq_a1));
			enable_irq(gpio_to_irq(ddata->pdata.irq_a2));
		}
#endif
	}

	mutex_lock(&ddata->lock);

	data = lsm303dlh_a_read(ddata, CTRL_REG1, "CTRL_REG1");

	data &= ~LSM303DLH_A_CR1_PM_MASK;

	ddata->mode = val;

	data |= ((val << LSM303DLH_A_CR1_PM_BIT) & LSM303DLH_A_CR1_PM_MASK);

	error = lsm303dlh_a_write(ddata, CTRL_REG1, data, "CTRL_REG1");
	if (error < 0) {
		if (ddata->regulator)
			regulator_disable(ddata->regulator);
		mutex_unlock(&ddata->lock);
		return error;
	}

	mutex_unlock(&ddata->lock);

	if (val == LSM303DLH_A_MODE_OFF) {
#ifdef CONFIG_INPUT_ST_LSM303DLH_INPUT_DEVICE
		if (!ddata->input_poll_dev) {
			disable_irq(gpio_to_irq(ddata->pdata.irq_a1));
			disable_irq(gpio_to_irq(ddata->pdata.irq_a2));
		} else {
			cancel_delayed_work_sync(&lsm303dlh_a_ddata->
						 input_work);
		}
#endif
		/*
		 * No need to store context here
		 * it is not like suspend/resume
		 * but fall back to default values
		 */
		ddata->rate = LSM303DLH_A_RATE_50;
		ddata->range = LSM303DLH_A_RANGE_2G;
		ddata->shift_adjust = SHIFT_ADJ_2G;

		if (ddata->regulator)
			regulator_disable(ddata->regulator);

	}
#ifdef CONFIG_INPUT_ST_LSM303DLH_INPUT_DEVICE
	if ((val != LSM303DLH_A_MODE_OFF) && ddata->input_poll_dev) {
		schedule_delayed_work(&lsm303dlh_a_ddata->input_work,
			       msecs_to_jiffies(lsm303dlh_a_ddata->
						poll_dev_delay));
	}
#endif

	return count;
}

static ssize_t lsm303dlh_a_show_rate(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct lsm303dlh_a_data *ddata = platform_get_drvdata(pdev);

	return sprintf(buf, "%d\n", ddata->rate);
}

static ssize_t lsm303dlh_a_store_rate(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct lsm303dlh_a_data *ddata = platform_get_drvdata(pdev);
	unsigned long val;
	unsigned char data;
	int error;

	if (ddata->mode == LSM303DLH_A_MODE_OFF) {
		dev_info(&ddata->client->dev,
			 "device is switched off, make it ON using MODE.\n");
		return count;
	}

	error = strict_strtoul(buf, 0, &val);
	if (error)
		return error;

	if (val < LSM303DLH_A_RATE_50 || val > LSM303DLH_A_RATE_1000)
		return -EINVAL;

	mutex_lock(&ddata->lock);

	data = lsm303dlh_a_read(ddata, CTRL_REG1, "CTRL_REG1");

	data &= ~LSM303DLH_A_CR1_DR_MASK;

	ddata->rate = val;

	data |= ((val << LSM303DLH_A_CR1_DR_BIT) & LSM303DLH_A_CR1_DR_MASK);

	error = lsm303dlh_a_write(ddata, CTRL_REG1, data, "CTRL_REG1");
	if (error < 0) {
		mutex_unlock(&ddata->lock);
		return error;
	}

	mutex_unlock(&ddata->lock);

	return count;
}

static ssize_t lsm303dlh_a_show_poll_dev_delay(struct device *dev,
					       struct device_attribute *attr,
					       char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct lsm303dlh_a_data *ddata = platform_get_drvdata(pdev);

	return sprintf(buf, "%u\n", ddata->poll_dev_delay);
}

static ssize_t lsm303dlh_a_set_poll_dev_delay(struct device *dev,
					      struct device_attribute
					      *attr, const char *buf,
					      size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct lsm303dlh_a_data *ddata = platform_get_drvdata(pdev);
	unsigned long val;
	int error;

	error = strict_strtoul(buf, 0, &val);
	if (error)
		return error;
	if (val)
		ddata->poll_dev_delay = val;
	else
		return -EINVAL;

	return count;
}

static DEVICE_ATTR(data, S_IRUGO, lsm303dlh_a_show_data, NULL);
static DEVICE_ATTR(range, S_IWUSR | S_IWGRP | S_IRUGO,
		lsm303dlh_a_show_range, lsm303dlh_a_store_range);
static DEVICE_ATTR(mode, S_IWUSR | S_IWGRP | S_IRUGO,
		lsm303dlh_a_show_mode, lsm303dlh_a_store_mode);
static DEVICE_ATTR(rate, S_IWUSR | S_IWGRP | S_IRUGO,
		lsm303dlh_a_show_rate, lsm303dlh_a_store_rate);
static DEVICE_ATTR(interrupt_control, S_IWUSR | S_IWGRP | S_IRUGO,
		lsm303dlh_a_show_interrupt_control,
		lsm303dlh_a_store_interrupt_control);
static DEVICE_ATTR(interrupt_channel, S_IWUSR | S_IWGRP | S_IRUGO,
		lsm303dlh_a_show_interrupt_channel,
		lsm303dlh_a_store_interrupt_channel);
static DEVICE_ATTR(interrupt_configure, S_IWUSR | S_IWGRP | S_IRUGO,
		lsm303dlh_a_show_interrupt_configure,
		lsm303dlh_a_store_interrupt_configure);
static DEVICE_ATTR(interrupt_duration, S_IWUSR | S_IWGRP | S_IRUGO,
		lsm303dlh_a_show_interrupt_duration,
		lsm303dlh_a_store_interrupt_duration);
static DEVICE_ATTR(interrupt_threshold, S_IWUSR | S_IWGRP | S_IRUGO,
		lsm303dlh_a_show_interrupt_threshold,
		lsm303dlh_a_store_interrupt_threshold);
static DEVICE_ATTR(delay, S_IWUSR | S_IWGRP | S_IRUGO,
		   lsm303dlh_a_show_poll_dev_delay,
		   lsm303dlh_a_set_poll_dev_delay);

static struct attribute *lsm303dlh_a_attributes[] = {
	&dev_attr_data.attr,
	&dev_attr_range.attr,
	&dev_attr_mode.attr,
	&dev_attr_rate.attr,
	&dev_attr_interrupt_control.attr,
	&dev_attr_interrupt_channel.attr,
	&dev_attr_interrupt_configure.attr,
	&dev_attr_interrupt_duration.attr,
	&dev_attr_interrupt_threshold.attr,
	&dev_attr_delay.attr,
	NULL
};

static const struct attribute_group lsm303dlh_a_attr_group = {
	.attrs = lsm303dlh_a_attributes,
};

#ifdef CONFIG_OF
static int __devinit lsm303dlh_a_probe_config_dt(
		struct lsm303dlh_platform_data *pdata,
		struct device_node *np)
{
	if (of_property_read_u32(np, "axis_map_x", &pdata->axis_map_x))
		pr_debug("unable to get axis_map_x.");
	if (of_property_read_u32(np, "axis_map_y", &pdata->axis_map_y))
		pr_debug("unable to get axis_map_y.");
	if (of_property_read_u32(np, "axis_map_z", &pdata->axis_map_z))
		pr_debug("unable to get axis_map_z.");
	if (of_property_read_u32(np, "negative_x", &pdata->negative_x))
		pr_debug("unable to get negative_x.");
	if (of_property_read_u32(np, "negative_y", &pdata->negative_y))
		pr_debug("unable to get negative_y.");
	if (of_property_read_u32(np, "negative_z", &pdata->negative_z))
		pr_debug("unable to get negative_z.");
	if (of_property_read_u32(np, "input_poll_dev", &pdata->input_poll_dev))
		pr_debug("unable to get input_poll_dev.");
	if (of_property_read_string(np, "name_a", &pdata->name_a))
		pr_debug("unable to get name_a.");
#ifdef CONFIG_INPUT_ST_LSM303DLH_INPUT_DEVICE
	pdata->irq_a1 = of_get_named_gpio(np, "irq_a1", 0);
	if (!pdata->irq_a1)
		pr_debug("unable to get irq_a1 from %s.", __func__);
	pdata->irq_a2 = of_get_named_gpio(np, "irq_a2", 0);
	if (!pdata->irq_a2)
		pr_debug("unable to get irq_a2 from %s.", __func__);
#endif
	return 0;
}
#else
static int __devinit lsm303dlh_a_probe_config_dt(
		struct lsm303dlh_platform_data *pdata,
		struct device_node *np)
{
	return -ENOSYS;
}
#endif

static int __devinit lsm303dlh_a_probe(struct i2c_client *client,
						const struct i2c_device_id *id)
{
	int ret;
	struct lsm303dlh_a_data *ddata = NULL;
	struct device_node *np = client->dev.of_node;

	if (!client->dev.platform_data && !np) {
		dev_err(&client->dev, "platform data is NULL. exiting.\n");
		ret = -ENODEV;
		goto exit_free_regulator;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "client not i2c capable\n");
		ret = -ENODEV;
		goto exit_free_regulator;
	}

	ddata = kzalloc(sizeof(struct lsm303dlh_a_data), GFP_KERNEL);
	if (ddata == NULL) {
		ret = -ENOMEM;
		goto err_op_failed;
	}

	ddata->client = client;
	i2c_set_clientdata(client, ddata);

	if (np) {
		ret = lsm303dlh_a_probe_config_dt(&ddata->pdata, np);
		if (ret) {
			dev_err(&client->dev, "no platform data\n");
			return -ENODEV;
		}
	} else {
		/* copy platform specific data */
		memcpy(&ddata->pdata, client->dev.platform_data,
				sizeof(ddata->pdata));
	}
	ddata->mode = LSM303DLH_A_MODE_OFF;
	ddata->rate = LSM303DLH_A_RATE_50;
	ddata->range = LSM303DLH_A_RANGE_2G;
	ddata->shift_adjust = SHIFT_ADJ_2G;
	dev_set_name(&client->dev, ddata->pdata.name_a);

	ddata->regulator = regulator_get(&client->dev, "v-accel");
	if (IS_ERR(ddata->regulator)) {
		dev_err(&client->dev, "failed to get regulator\n");
		ret = PTR_ERR(ddata->regulator);
		ddata->regulator = NULL;
	}

	if (ddata->regulator)
		regulator_enable(ddata->regulator);

	ret = lsm303dlh_a_read(ddata, WHO_AM_I, "WHO_AM_I");
	if (ret < 0)
		goto exit_free_regulator;

	dev_info(&client->dev, "3-Axis Accelerometer, ID : %d\n", ret);

	mutex_init(&ddata->lock);

	ret = sysfs_create_group(&client->dev.kobj, &lsm303dlh_a_attr_group);
	if (ret)
		goto exit_free_regulator;

#ifdef CONFIG_INPUT_ST_LSM303DLH_INPUT_DEVICE

	/* accelerometer has two interrupt channels
	 * (thresholds, durations and sources)
	 * and can support two input devices */

	ddata->input_poll_dev = ddata->pdata.input_poll_dev;

	ddata->input_dev = input_allocate_device();
	if (!ddata->input_dev) {
		ret = -ENOMEM;
		dev_err(&client->dev, "Failed to allocate input device\n");
		goto exit_free_regulator;
	}

	ddata->input_dev2 = input_allocate_device();
	if (!ddata->input_dev2) {
		ret = -ENOMEM;
		dev_err(&client->dev, "Failed to allocate input device\n");
		goto err_input_alloc_failed;
	}

	set_bit(EV_ABS, ddata->input_dev->evbit);
	set_bit(EV_ABS, ddata->input_dev2->evbit);

	/* x-axis acceleration */
	input_set_abs_params(ddata->input_dev, ABS_X, -32768, 32767, 0, 0);
	input_set_abs_params(ddata->input_dev2, ABS_X, -32768, 32767, 0, 0);
	/* y-axis acceleration */
	input_set_abs_params(ddata->input_dev, ABS_Y, -32768, 32767, 0, 0);
	input_set_abs_params(ddata->input_dev2, ABS_Y, -32768, 32767, 0, 0);
	/* z-axis acceleration */
	input_set_abs_params(ddata->input_dev, ABS_Z, -32768, 32767, 0, 0);
	input_set_abs_params(ddata->input_dev2, ABS_Z, -32768, 32767, 0, 0);

	ddata->input_dev->name = "accelerometer";
	ddata->input_dev->phys = "accelerometer/input0";
	ddata->input_dev2->name = "motion";
	ddata->input_dev2->phys = "motion/input0";

	ret = input_register_device(ddata->input_dev);
	if (ret) {
		dev_err(&client->dev, "Unable to register input device: %s\n",
			ddata->input_dev->name);
		goto err_input_register_failed;
	}

	ret = input_register_device(ddata->input_dev2);
	if (ret) {
		dev_err(&client->dev, "Unable to register input device: %s\n",
			ddata->input_dev->name);
		goto err_input_register_failed2;
	}
	if (ddata->input_poll_dev) {
		lsm303dlh_a_ddata = ddata;
		lsm303dlh_a_ddata->poll_dev_delay = 1;
		INIT_DELAYED_WORK(&lsm303dlh_a_ddata->input_work,
				  lsm303dlh_a_work_function);
	} else {
		/* Request GPIOs for interrupt lines */
		ret = gpio_request_one(ddata->pdata.irq_a1, GPIOF_IN,
			"LSM303_INT_1");
		if (ret) {
			pr_err("GPIO request for LSM303 accel mag irq_a2 failed\n");
			goto err_gpio1_req_failed;
		}

		ret = gpio_request_one(ddata->pdata.irq_a2, GPIOF_IN,
			"LSM303_INT_2");
		if (ret) {
			pr_err("GPIO request for LSM303 accel irq_a2 failed\n");
			goto err_gpio2_req_failed;
		}

		/* Register interrupt */
		ret = request_threaded_irq(gpio_to_irq(ddata->pdata.irq_a1),
				NULL, lsm303dlh_a_gpio_irq,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				"lsm303dlh_a", ddata);
		if (ret) {
			dev_err(&client->dev, "request irq1 failed\n");
			goto err_input_failed;
		}

		ret = request_threaded_irq(gpio_to_irq(ddata->pdata.irq_a2),
				NULL, lsm303dlh_a_gpio_irq,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				"lsm303dlh_a", ddata);
		if (ret) {
			dev_err(&client->dev, "request irq2 failed\n");
			goto err_input_failed;
		}

		/* only mode can enable it */
		disable_irq(gpio_to_irq(ddata->pdata.irq_a1));
		disable_irq(gpio_to_irq(ddata->pdata.irq_a2));
	}

#endif

	return ret;

#ifdef CONFIG_INPUT_ST_LSM303DLH_INPUT_DEVICE
err_input_failed:
	if (!ddata->input_poll_dev)
		gpio_free(ddata->pdata.irq_a2);
err_gpio2_req_failed:
	gpio_free(ddata->pdata.irq_a1);
err_gpio1_req_failed:
	input_unregister_device(ddata->input_dev2);
err_input_register_failed2:
	input_unregister_device(ddata->input_dev);
err_input_register_failed:
	input_free_device(ddata->input_dev2);
err_input_alloc_failed:
	input_free_device(ddata->input_dev);
#endif
exit_free_regulator:
	if (ddata->regulator) {
		regulator_disable(ddata->regulator);
		regulator_put(ddata->regulator);
	}
err_op_failed:
	kfree(ddata);
	dev_err(&client->dev, "probe function fails %x\n", ret);
	return ret;
}

static int __devexit lsm303dlh_a_remove(struct i2c_client *client)
{
	struct lsm303dlh_a_data *ddata;

	ddata = i2c_get_clientdata(client);
#ifdef CONFIG_INPUT_ST_LSM303DLH_INPUT_DEVICE
	if (ddata->input_poll_dev) {
		cancel_delayed_work_sync(&lsm303dlh_a_ddata->input_work);
	} else {
		gpio_free(ddata->pdata.irq_a1);
		gpio_free(ddata->pdata.irq_a2);
	}
	input_unregister_device(ddata->input_dev);
	input_unregister_device(ddata->input_dev2);
	input_free_device(ddata->input_dev);
	input_free_device(ddata->input_dev2);
#endif
	sysfs_remove_group(&client->dev.kobj, &lsm303dlh_a_attr_group);

	/* safer to make device off */
	if (ddata->mode != LSM303DLH_A_MODE_OFF) {
		lsm303dlh_a_write(ddata, CTRL_REG1, 0, "CONTROL");
		if (ddata->regulator) {
			regulator_disable(ddata->regulator);
			regulator_put(ddata->regulator);
		}
	}

	i2c_set_clientdata(client, NULL);
	kfree(ddata);

	return 0;
}

#ifdef CONFIG_PM
static int lsm303dlh_a_suspend(struct device *dev)
{
	struct lsm303dlh_a_data *ddata;
	int ret;

	ddata = dev_get_drvdata(dev);

	if (ddata->mode == LSM303DLH_A_MODE_OFF)
		return 0;

#ifdef CONFIG_INPUT_ST_LSM303DLH_INPUT_DEVICE
	if (!ddata->input_poll_dev) {
		disable_irq(gpio_to_irq(ddata->pdata.irq_a1));
		disable_irq(gpio_to_irq(ddata->pdata.irq_a2));
	} else {
	    cancel_delayed_work_sync(&lsm303dlh_a_ddata->input_work);
	}
#endif

	ret = lsm303dlh_a_write(ddata, CTRL_REG1,
				LSM303DLH_A_MODE_OFF, "CONTROL");

	if (ddata->regulator)
		regulator_disable(ddata->regulator);

	return ret;
}

static int __lsm303dlh_a_resume(struct device *dev, bool csave)
{
	struct lsm303dlh_a_data *ddata;
	int ret;

	ddata = dev_get_drvdata(dev);

	/* in correct mode, no need to change it */
	if (ddata->mode == LSM303DLH_A_MODE_OFF)
		return 0;

#ifdef CONFIG_INPUT_ST_LSM303DLH_INPUT_DEVICE
	if (!ddata->input_poll_dev) {
		disable_irq(gpio_to_irq(ddata->pdata.irq_a1));
		disable_irq(gpio_to_irq(ddata->pdata.irq_a2));
	} else {
		cancel_delayed_work_sync(&lsm303dlh_a_ddata->
					 input_work);
	}
#endif

	if (ddata->regulator)
		regulator_enable(ddata->regulator);

	if (!csave)
		return 0;

	ret = lsm303dlh_a_restore(ddata);
	if (ret < 0)
		dev_err(&ddata->client->dev,
			"Error while resuming the device.\n");

	return ret;
}

static int lsm303dlh_a_resume(struct device *dev)
{
	return __lsm303dlh_a_resume(dev, true);
}

static int lsm303dlh_a_thaw(struct device *dev)
{
	return __lsm303dlh_a_resume(dev, false);
}

static const struct dev_pm_ops lsm303dlh_a_dev_pm_ops = {
	.suspend = lsm303dlh_a_suspend,
	.resume = lsm303dlh_a_resume,
	.freeze = lsm303dlh_a_suspend,
	.thaw = lsm303dlh_a_thaw,
	.poweroff = lsm303dlh_a_suspend,
	.restore = lsm303dlh_a_resume,
};
#endif /* CONFIG_PM */

static const struct i2c_device_id lsm303dlh_a_id[] = {
	{ "lsm303dlh_a", 0 },
	{ },
};

#ifdef CONFIG_OF
static const struct of_device_id lsm303dlh_a_of_match[] = {
	{.compatible = "st,lsm303dlh_a",},
	{}
};

MODULE_DEVICE_TABLE(of, lsm303dlh_a_of_match);
#endif

static struct i2c_driver lsm303dlh_a_driver = {
	.probe		= lsm303dlh_a_probe,
	.remove		= lsm303dlh_a_remove,
	.id_table	= lsm303dlh_a_id,
	.driver = {
		.name = "lsm303dlh_a",
		.owner = THIS_MODULE,
	#ifdef CONFIG_OF
		.of_match_table = lsm303dlh_a_of_match,
	#endif
	#ifdef CONFIG_PM
		.pm	=	&lsm303dlh_a_dev_pm_ops,
	#endif
	},
};

static int __init lsm303dlh_a_init(void)
{
	return i2c_add_driver(&lsm303dlh_a_driver);
}
module_init(lsm303dlh_a_init)

static void __exit lsm303dlh_a_exit(void)
{
	i2c_del_driver(&lsm303dlh_a_driver);
}
module_exit(lsm303dlh_a_exit)

MODULE_DESCRIPTION("LSM303DLH 3-Axis Accelerometer Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("STMicroelectronics");
