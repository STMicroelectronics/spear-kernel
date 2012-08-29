/*
 * lsm303dlh_m.c
 * ST 3-Axis Magnetometer Driver
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
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>

#ifdef CONFIG_INPUT_ST_LSM303DLH_INPUT_DEVICE
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#endif

#include <linux/i2c/lsm303dlh.h>
#include <linux/regulator/consumer.h>
#include <linux/kernel.h>

/* lsm303dlh magnetometer registers */
#define IRA_REG_M	0x0A

/* Magnetometer registers */
#define CRA_REG_M	0x00 /* Configuration register A */
#define CRB_REG_M	0x01 /* Configuration register B */
#define MR_REG_M	0x02 /* Mode register */
#define SR_REG_M	0x09 /* Status register */

/* Output register start address*/
#define OUT_X_M		0x03
#define OUT_Y_M		0x05
#define OUT_Z_M		0x07

/* Magnetometer X-Y gain */
#define XY_GAIN_1_3	1055 /* XY gain at 1.3G */
#define XY_GAIN_1_9	 795 /* XY gain at 1.9G */
#define XY_GAIN_2_5	 635 /* XY gain at 2.5G */
#define XY_GAIN_4_0	 430 /* XY gain at 4.0G */
#define XY_GAIN_4_7	 375 /* XY gain at 4.7G */
#define XY_GAIN_5_6	 320 /* XY gain at 5.6G */
#define XY_GAIN_8_1	 230 /* XY gain at 8.1G */

/* Magnetometer Z gain */
#define Z_GAIN_1_3	950 /* Z gain at 1.3G */
#define Z_GAIN_1_9	710 /* Z gain at 1.9G */
#define Z_GAIN_2_5	570 /* Z gain at 2.5G */
#define Z_GAIN_4_0	385 /* Z gain at 4.0G */
#define Z_GAIN_4_7	335 /* Z gain at 4.7G */
#define Z_GAIN_5_6	285 /* Z gain at 5.6G */
#define Z_GAIN_8_1	205 /* Z gain at 8.1G */

/* Control A regsiter. */
#define LSM303DLH_M_CRA_DO_BIT 2
#define LSM303DLH_M_CRA_DO_MASK (0x7 << LSM303DLH_M_CRA_DO_BIT)
#define LSM303DLH_M_CRA_MS_BIT 0
#define LSM303DLH_M_CRA_MS_MASK (0x3 << LSM303DLH_M_CRA_MS_BIT)

/* Control B regsiter. */
#define LSM303DLH_M_CRB_GN_BIT 5
#define LSM303DLH_M_CRB_GN_MASK (0x7 << LSM303DLH_M_CRB_GN_BIT)

/* Control Mode regsiter. */
#define LSM303DLH_M_MR_MD_BIT 0
#define LSM303DLH_M_MR_MD_MASK (0x3 << LSM303DLH_M_MR_MD_BIT)

/* Control Status regsiter. */
#define LSM303DLH_M_SR_RDY_BIT 0
#define LSM303DLH_M_SR_RDY_MASK (0x1 << LSM303DLH_M_SR_RDY_BIT)
#define LSM303DLH_M_SR_LOC_BIT 1
#define LSM303DLH_M_SR_LCO_MASK (0x1 << LSM303DLH_M_SR_LOC_BIT)
#define LSM303DLH_M_SR_REN_BIT 2
#define LSM303DLH_M_SR_REN_MASK (0x1 << LSM303DLH_M_SR_REN_BIT)

/* Magnetometer gain setting */
#define LSM303DLH_M_RANGE_1_3G	0x01
#define LSM303DLH_M_RANGE_1_9G	0x02
#define LSM303DLH_M_RANGE_2_5G	0x03
#define LSM303DLH_M_RANGE_4_0G	0x04
#define LSM303DLH_M_RANGE_4_7G	0x05
#define LSM303DLH_M_RANGE_5_6G	0x06
#define LSM303DLH_M_RANGE_8_1G	0x07

/* Magnetometer capturing mode */
#define LSM303DLH_M_MODE_CONTINUOUS 0
#define LSM303DLH_M_MODE_SINGLE 1
#define LSM303DLH_M_MODE_SLEEP 3

/* Magnetometer output data rate */
#define LSM303DLH_M_RATE_00_75 0x00
#define LSM303DLH_M_RATE_01_50 0x01
#define LSM303DLH_M_RATE_03_00 0x02
#define LSM303DLH_M_RATE_07_50 0x03
#define LSM303DLH_M_RATE_15_00 0x04
#define LSM303DLH_M_RATE_30_00 0x05
#define LSM303DLH_M_RATE_75_00 0x06

/* Multiple byte transfer enable */
#define MULTIPLE_I2C_TR 0x80

/*
 * magnetometer local data
 */
struct lsm303dlh_m_data {
	struct i2c_client *client;
	/* lock for sysfs operations */
	struct mutex lock;

#ifdef CONFIG_INPUT_ST_LSM303DLH_INPUT_DEVICE
	struct input_dev *input_dev;
#endif
	struct regulator *regulator;
	struct lsm303dlh_platform_data pdata;

	short gain[3];
	short data[3];
	unsigned char mode;
	unsigned char rate;
	unsigned char range;
};

static int lsm303dlh_m_set_mode(struct lsm303dlh_m_data *ddata,
		unsigned char mode);
static int lsm303dlh_m_write(struct lsm303dlh_m_data *ddata,
		u8 reg, u8 val, char *msg)
{
	int ret = i2c_smbus_write_byte_data(ddata->client, reg, val);
	if (ret < 0)
		dev_err(&ddata->client->dev,
			"i2c_smbus_write_byte_data failed error "
			"%d Register (%s)\n", ret, msg);
	return ret;
}

static int lsm303dlh_m_restore(struct lsm303dlh_m_data *ddata)
{
	int ret = 0;

	ret = lsm303dlh_m_write(ddata, CRB_REG_M, ddata->range, "SET RANGE");
	if (ret < 0)
		goto fail;

	ret = lsm303dlh_m_write(ddata, CRA_REG_M, ddata->rate, "SET RATE");
	if (ret < 0)
		goto fail;

	ret = lsm303dlh_m_set_mode(ddata, ddata->mode);
	if (ret < 0)
		goto fail;

fail:
	return ret;
}

static int lsm303dlh_m_read_multi(struct lsm303dlh_m_data *ddata, u8 reg,
		u8 count, u8 *val, char *msg)
{
	int ret = i2c_smbus_read_i2c_block_data(ddata->client,
			reg | MULTIPLE_I2C_TR, count, val);
	if (ret < 0)
		dev_err(&ddata->client->dev,
			"i2c_smbus_read_i2c_block_data failed error %d"
			" Register (%s)\n", ret, msg);
	return ret;
}

static ssize_t lsm303dlh_m_show_rate(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct lsm303dlh_m_data *ddata = platform_get_drvdata(pdev);

	return sprintf(buf, "%d\n", ddata->rate >> LSM303DLH_M_CRA_DO_BIT);
}

/* set lsm303dlh magnetometer bandwidth */
static ssize_t lsm303dlh_m_store_rate(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct lsm303dlh_m_data *ddata = platform_get_drvdata(pdev);
	unsigned long val;
	unsigned char data;
	int error;

	if (ddata->mode == LSM303DLH_M_MODE_SLEEP) {
		dev_info(&ddata->client->dev,
			 "device is switched off, make it ON using MODE.\n");
		return count;
	}

	error = strict_strtoul(buf, 0, &val);
	if (error)
		return error;

	if (val < LSM303DLH_M_RATE_00_75 || val > LSM303DLH_M_RATE_75_00)
		return -EINVAL;

	mutex_lock(&ddata->lock);

	data = ((val << LSM303DLH_M_CRA_DO_BIT) & LSM303DLH_M_CRA_DO_MASK);
	ddata->rate = data;

	error = lsm303dlh_m_write(ddata, CRA_REG_M, data, "SET RATE");
	if (error < 0) {
		mutex_unlock(&ddata->lock);
		return error;
	}

	mutex_unlock(&ddata->lock);

	return count;
}

static int lsm303dlh_m_xyz_read(struct lsm303dlh_m_data *ddata)
{
	unsigned char xyz_data[6];
	int ret = lsm303dlh_m_read_multi(ddata, OUT_X_M,
			6, xyz_data, "OUT_X_M");
	if (ret < 0)
		return -EINVAL;

	/* MSB is at lower address */
	ddata->data[0] = (short)
		(((xyz_data[0]) << 8) | xyz_data[1]);
	ddata->data[1] = (short)
		(((xyz_data[2]) << 8) | xyz_data[3]);
	ddata->data[2] = (short)
		(((xyz_data[4]) << 8) | xyz_data[5]);

	/* taking orientation of x, y, z axis into account*/
	if (ddata->pdata.axis_map_x ||	ddata->pdata.axis_map_y ||
			ddata->pdata.axis_map_z) {
		ddata->data[ddata->pdata.axis_map_x] = ddata->pdata.negative_x ?
			-ddata->data[ddata->pdata.axis_map_x] :
			 ddata->data[ddata->pdata.axis_map_x];
		ddata->data[ddata->pdata.axis_map_y] = ddata->pdata.negative_y ?
			-ddata->data[ddata->pdata.axis_map_y] :
			 ddata->data[ddata->pdata.axis_map_y];
		ddata->data[ddata->pdata.axis_map_z] = ddata->pdata.negative_z ?
			-ddata->data[ddata->pdata.axis_map_z] :
			 ddata->data[ddata->pdata.axis_map_z];
	}

	return ret;
}

static ssize_t lsm303dlh_m_gain(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct lsm303dlh_m_data *ddata = platform_get_drvdata(pdev);
	int ret;

	if (ddata->pdata.axis_map_x || ddata->pdata.axis_map_y ||
			ddata->pdata.axis_map_z) {
		ret = sprintf(buf, "(%d, %d, %d)\n",
				ddata->gain[ddata->pdata.axis_map_x],
				ddata->gain[ddata->pdata.axis_map_y],
				ddata->gain[ddata->pdata.axis_map_z]);
	} else {
		ret = sprintf(buf, "(%d, %d, %d)\n",
				ddata->gain[0],
				ddata->gain[1],
				ddata->gain[2]);
	}

	return ret;
}

static ssize_t lsm303dlh_m_values(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct lsm303dlh_m_data *ddata = platform_get_drvdata(pdev);
	int ret = 0;

	if (ddata->mode == LSM303DLH_M_MODE_SLEEP) {
		dev_info(&ddata->client->dev,
			 "device is switched off, make it ON using MODE.\n");
		return ret;
	}

	mutex_lock(&ddata->lock);

	ret = lsm303dlh_m_xyz_read(ddata);
	if (ret < 0) {
		mutex_unlock(&ddata->lock);
		return -EINVAL;
	}

	mutex_unlock(&ddata->lock);

	/* taking orientation of x, y, z axis into account*/
	if (ddata->pdata.axis_map_x || ddata->pdata.axis_map_y ||
			ddata->pdata.axis_map_z)
		ret = sprintf(buf, "(%d, %d, %d)\n",
				ddata->data[ddata->pdata.axis_map_x],
				ddata->data[ddata->pdata.axis_map_y],
				ddata->data[ddata->pdata.axis_map_z]);
	else
		ret = sprintf(buf, "(%d, %d, %d)\n",
				ddata->data[0],
				ddata->data[1],
				ddata->data[2]);

	return ret;
}

static int lsm303dlh_m_set_mode(struct lsm303dlh_m_data *ddata,
		unsigned char mode)
{
	int ret;

	mode = (mode << LSM303DLH_M_MR_MD_BIT);

	ret = i2c_smbus_write_byte_data(ddata->client, MR_REG_M, mode);

	if (ret < 0)
		dev_err(&ddata->client->dev,
			"i2c_smbus_write_byte_data failed error %d "
			"Register (%s)\n", ret, "MODE CONTROL");

	return ret;
}

#ifdef CONFIG_INPUT_ST_LSM303DLH_INPUT_DEVICE

static irqreturn_t lsm303dlh_m_gpio_irq(int irq, void *device_data)
{
	struct lsm303dlh_m_data *ddata = device_data;
	int ret;

	ret = lsm303dlh_m_xyz_read(ddata);

	if (ret < 0) {
		dev_err(&ddata->client->dev,
				"reading data of xyz failed error %d\n", ret);
		return IRQ_NONE;
	}

	/* taking orientation of x, y, z axis into account*/

	input_report_abs(ddata->input_dev, ABS_X,
			ddata->data[ddata->pdata.axis_map_x]);
	input_report_abs(ddata->input_dev, ABS_Y,
			ddata->data[ddata->pdata.axis_map_y]);
	input_report_abs(ddata->input_dev, ABS_Z,
			ddata->data[ddata->pdata.axis_map_z]);
	input_sync(ddata->input_dev);

	return IRQ_HANDLED;

}
#endif

static ssize_t lsm303dlh_m_show_range(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct lsm303dlh_m_data *ddata = platform_get_drvdata(pdev);

	return sprintf(buf, "%d\n", ddata->range >> LSM303DLH_M_CRB_GN_BIT);
}

static ssize_t lsm303dlh_m_store_range(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct lsm303dlh_m_data *ddata = platform_get_drvdata(pdev);
	short xy_gain;
	short z_gain;
	unsigned long range;

	int error;

	if (ddata->mode == LSM303DLH_M_MODE_SLEEP) {
		dev_info(&ddata->client->dev,
			 "device is switched off, make it ON using MODE.\n");
		return count;
	}

	error = strict_strtoul(buf, 0, &range);
	if (error)
		return error;

	switch (range) {
	case LSM303DLH_M_RANGE_1_3G:
		xy_gain = XY_GAIN_1_3;
		z_gain = Z_GAIN_1_3;
		break;
	case LSM303DLH_M_RANGE_1_9G:
		xy_gain = XY_GAIN_1_9;
		z_gain = Z_GAIN_1_9;
		break;
	case LSM303DLH_M_RANGE_2_5G:
		xy_gain = XY_GAIN_2_5;
		z_gain = Z_GAIN_2_5;
		break;
	case LSM303DLH_M_RANGE_4_0G:
		xy_gain = XY_GAIN_4_0;
		z_gain = Z_GAIN_4_0;
		break;
	case LSM303DLH_M_RANGE_4_7G:
		xy_gain = XY_GAIN_4_7;
		z_gain = Z_GAIN_4_7;
		break;
	case LSM303DLH_M_RANGE_5_6G:
		xy_gain = XY_GAIN_5_6;
		z_gain = Z_GAIN_5_6;
		break;
	case LSM303DLH_M_RANGE_8_1G:
		xy_gain = XY_GAIN_8_1;
		z_gain = Z_GAIN_8_1;
		break;
	default:
		return -EINVAL;
	}

	mutex_lock(&ddata->lock);

	if (ddata->pdata.axis_map_x || ddata->pdata.axis_map_y ||
			ddata->pdata.axis_map_z) {
		ddata->gain[ddata->pdata.axis_map_x] = xy_gain;
		ddata->gain[ddata->pdata.axis_map_y] = xy_gain;
		ddata->gain[ddata->pdata.axis_map_z] = z_gain;
	} else {
		ddata->gain[0] = xy_gain;
		ddata->gain[1] = xy_gain;
		ddata->gain[2] = z_gain;
	}

	range <<= LSM303DLH_M_CRB_GN_BIT;
	range &= LSM303DLH_M_CRB_GN_MASK;

	ddata->range = range;

	error = lsm303dlh_m_write(ddata, CRB_REG_M, range, "SET RANGE");
	mutex_unlock(&ddata->lock);

	if (error < 0)
		return error;

	return count;
}

static ssize_t lsm303dlh_m_show_mode(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct lsm303dlh_m_data *ddata = platform_get_drvdata(pdev);

	return sprintf(buf, "%d\n", ddata->mode);
}

static ssize_t lsm303dlh_m_store_mode(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct lsm303dlh_m_data *ddata = platform_get_drvdata(pdev);
	unsigned long mode;
	int error;

	error = strict_strtoul(buf, 0, &mode);
	if (error)
		return error;

	/* if same mode as existing, return */
	if (ddata->mode == mode)
		return count;

	/* not in correct range */
	if (!(mode == LSM303DLH_M_MODE_CONTINUOUS ||
				mode == LSM303DLH_M_MODE_SINGLE ||
				mode == LSM303DLH_M_MODE_SLEEP))
		return -EINVAL;

	/* turn on the supplies if already off */
	if (ddata->mode == LSM303DLH_M_MODE_SLEEP) {
		regulator_enable(ddata->regulator);

#ifdef CONFIG_INPUT_ST_LSM303DLH_INPUT_DEVICE
	enable_irq(gpio_to_irq(ddata->pdata.irq_m));
#endif
	}

	mutex_lock(&ddata->lock);

	error = lsm303dlh_m_set_mode(ddata, mode);

	ddata->mode = (mode >> LSM303DLH_M_MR_MD_BIT);
	mutex_unlock(&ddata->lock);

	if (error < 0)
		return error;

	if (mode == LSM303DLH_M_MODE_SLEEP) {

#ifdef CONFIG_INPUT_ST_LSM303DLH_INPUT_DEVICE
		disable_irq(gpio_to_irq(ddata->pdata.irq_m));
#endif

		/*
		 * No need to store context here, it is not like
		 * suspend/resume but fall back to default values
		 */
		ddata->rate = LSM303DLH_M_RATE_00_75;
		ddata->range = LSM303DLH_M_RANGE_1_3G;
		ddata->range <<= LSM303DLH_M_CRB_GN_BIT;
		ddata->range &= LSM303DLH_M_CRB_GN_MASK;

		if (ddata->pdata.axis_map_x || ddata->pdata.axis_map_y ||
				ddata->pdata.axis_map_z) {
			ddata->gain[ddata->pdata.axis_map_x] = XY_GAIN_1_3;
			ddata->gain[ddata->pdata.axis_map_y] = XY_GAIN_1_3;
			ddata->gain[ddata->pdata.axis_map_z] = Z_GAIN_1_3;
		} else {
			ddata->gain[0] = XY_GAIN_1_3;
			ddata->gain[1] = XY_GAIN_1_3;
			ddata->gain[2] = Z_GAIN_1_3;
		}

		regulator_disable(ddata->regulator);
	}

	return count;
}

static DEVICE_ATTR(gain, S_IRUGO, lsm303dlh_m_gain, NULL);
static DEVICE_ATTR(data, S_IRUGO, lsm303dlh_m_values, NULL);
static DEVICE_ATTR(mode, S_IWUGO | S_IRUGO,
		lsm303dlh_m_show_mode, lsm303dlh_m_store_mode);
static DEVICE_ATTR(range, S_IWUGO | S_IRUGO,
		lsm303dlh_m_show_range, lsm303dlh_m_store_range);
static DEVICE_ATTR(rate, S_IWUGO | S_IRUGO,
		lsm303dlh_m_show_rate, lsm303dlh_m_store_rate);

static struct attribute *lsm303dlh_m_attributes[] = {
	&dev_attr_gain.attr,
	&dev_attr_data.attr,
	&dev_attr_mode.attr,
	&dev_attr_range.attr,
	&dev_attr_rate.attr,
	NULL
};

static const struct attribute_group lsm303dlh_m_attr_group = {
	.attrs = lsm303dlh_m_attributes,
};

#ifdef CONFIG_OF
static int __devinit lsm303dlh_m_probe_config_dt(
		struct lsm303dlh_platform_data *pdata,
		struct device_node *np)
{
	if (of_property_read_u32(np, "axis_map_x", &pdata->axis_map_x))
		pr_debug("unable to get axis_map_x from %s.", __func__);
	if (of_property_read_u32(np, "axis_map_y", &pdata->axis_map_y))
		pr_debug("unable to get axis_map_y from %s.", __func__);
	if (of_property_read_u32(np, "axis_map_z", &pdata->axis_map_z))
		pr_debug("unable to get axis_map_z from %s.", __func__);
	if (of_property_read_u32(np, "negative_x", &pdata->negative_x))
		pr_debug("unable to get negative_x from %s.", __func__);
	if (of_property_read_u32(np, "negative_y", &pdata->negative_y))
		pr_debug("unable to get negative_y from %s.", __func__);
	if (of_property_read_u32(np, "negative_z", &pdata->negative_z))
		pr_debug("unable to get negative_z from %s.", __func__);
	if (of_property_read_string(np, "name_m", &pdata->name_m))
		pr_debug("unable to get name_m from %s.", __func__);
#ifdef CONFIG_INPUT_ST_LSM303DLH_INPUT_DEVICE
	pdata->irq_m = of_get_named_gpio(np, "irq_m", 0);
	if (!pdata->irq_m)
		pr_debug("unable to get irq_m from %s.", __func__);
#endif
	return 0;
}
#else
static int __devinit lsm303dlh_m_probe_config_dt(
		struct lsm303dlh_platform_data *pdata,
		struct device_node *np)
{
	return -ENOSYS;
}
#endif

static int __devinit lsm303dlh_m_probe(struct i2c_client *client,
						const struct i2c_device_id *id)
{
	int ret;
	struct lsm303dlh_m_data *ddata = NULL;
	unsigned char version[3];
	struct device_node *np = client->dev.of_node;

	ddata = kzalloc(sizeof(struct lsm303dlh_m_data), GFP_KERNEL);
	if (ddata == NULL) {
		ret = -ENOMEM;
		goto err_op_failed;
	}

	ddata->client = client;
	i2c_set_clientdata(client, ddata);

	if (np) {
		ret = lsm303dlh_m_probe_config_dt(&ddata->pdata, np);
		if (ret) {
			dev_err(&client->dev, "no platform data\n");
			return -ENODEV;
		}
	} else {
		/* copy platform specific data */
		memcpy(&ddata->pdata, client->dev.platform_data,
				sizeof(ddata->pdata));
	}
	ddata->mode = LSM303DLH_M_MODE_SLEEP;
	ddata->rate = LSM303DLH_M_RATE_00_75;
	ddata->range = LSM303DLH_M_RANGE_1_3G;
	ddata->range <<= LSM303DLH_M_CRB_GN_BIT;
	ddata->range &= LSM303DLH_M_CRB_GN_MASK;
	ddata->gain[ddata->pdata.axis_map_x] = XY_GAIN_1_3;
	ddata->gain[ddata->pdata.axis_map_y] = XY_GAIN_1_3;
	ddata->gain[ddata->pdata.axis_map_z] = Z_GAIN_1_3;
	dev_set_name(&client->dev, ddata->pdata.name_m);
	ddata->regulator = regulator_get(&client->dev, "v-mag");

	if (IS_ERR(ddata->regulator)) {
		dev_err(&client->dev, "failed to get regulator\n");
		ret = PTR_ERR(ddata->regulator);
		ddata->regulator = NULL;
	}

	if (ddata->regulator)
		regulator_enable(ddata->regulator);

	ret = lsm303dlh_m_read_multi(ddata, IRA_REG_M, 3,
			version, "IRA_REG_M");
	if (ret < 0)
		goto exit_free_regulator;

	dev_info(&client->dev, "Magnetometer, ID : %x:%x:%x\n",
			version[0], version[1], version[2]);

	mutex_init(&ddata->lock);

	ret = sysfs_create_group(&client->dev.kobj, &lsm303dlh_m_attr_group);
	if (ret)
		goto exit_free_regulator;

#ifdef CONFIG_INPUT_ST_LSM303DLH_INPUT_DEVICE

	ddata->input_dev = input_allocate_device();
	if (!ddata->input_dev) {
		ret = -ENOMEM;
		dev_err(&client->dev, "Failed to allocate input device\n");
		goto exit_free_regulator;
	}

	set_bit(EV_ABS, ddata->input_dev->evbit);

	/* x-axis acceleration */
	input_set_abs_params(ddata->input_dev, ABS_X, -32768, 32767, 0, 0);
	/* y-axis acceleration */
	input_set_abs_params(ddata->input_dev, ABS_Y, -32768, 32767, 0, 0);
	/* z-axis acceleration */
	input_set_abs_params(ddata->input_dev, ABS_Z, -32768, 32767, 0, 0);

	ddata->input_dev->name = "magnetometer";
	ddata->input_dev->phys = "magnetometer/input0";

	ret = input_register_device(ddata->input_dev);
	if (ret) {
		dev_err(&client->dev, "Unable to register input device: %s\n",
				ddata->input_dev->name);
		goto err_input_register_failed;
	}

	ret = gpio_request_one(ddata->pdata.irq_m, GPIOF_IN, "LSM303_DRDY_M");
	if (ret) {
		pr_err("GPIO request for LSM303 mag irq_m failed\n");
		goto err_gpio_req_failed;
	}

	/* register interrupt */
	ret = request_threaded_irq(gpio_to_irq(ddata->pdata.irq_m), NULL,
		lsm303dlh_m_gpio_irq,
		IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "lsm303dlh_m",
		ddata);
	if (ret) {
		dev_err(&client->dev, "request irq EGPIO_PIN_1 failed\n");
		goto err_input_failed;
	}

	disable_irq(gpio_to_irq(ddata->pdata.irq_m));
#endif

	return ret;

#ifdef CONFIG_INPUT_ST_LSM303DLH_INPUT_DEVICE
err_input_failed:
	gpio_free(ddata->pdata.irq_m);
err_gpio_req_failed:
	input_unregister_device(ddata->input_dev);
err_input_register_failed:
	input_free_device(ddata->input_dev);
#endif
exit_free_regulator:
	if (ddata->regulator) {
		regulator_disable(ddata->regulator);
		regulator_put(ddata->regulator);
	}
err_op_failed:
	dev_err(&client->dev, "lsm303dlh_m_probe failed %x\n", ret);
	kfree(ddata);
	return ret;
}

static int __devexit lsm303dlh_m_remove(struct i2c_client *client)
{
	struct lsm303dlh_m_data *ddata;

	ddata = i2c_get_clientdata(client);

#ifdef CONFIG_INPUT_ST_LSM303DLH_INPUT_DEVICE
	gpio_free(ddata->pdata.irq_m);
	input_unregister_device(ddata->input_dev);
	input_free_device(ddata->input_dev);
#endif

	sysfs_remove_group(&client->dev.kobj, &lsm303dlh_m_attr_group);

	/* safer to make device off */
	if (ddata->mode != LSM303DLH_M_MODE_SLEEP) {
		lsm303dlh_m_set_mode(ddata, LSM303DLH_M_MODE_SLEEP);
		regulator_disable(ddata->regulator);
	}

	regulator_put(ddata->regulator);
	i2c_set_clientdata(client, NULL);
	kfree(ddata);

	return 0;
}

#ifdef CONFIG_PM
static int lsm303dlh_m_suspend(struct device *dev)
{
	struct lsm303dlh_m_data *ddata;
	int ret;

	ddata = dev_get_drvdata(dev);

	if (ddata->mode == LSM303DLH_M_MODE_SLEEP)
		return 0;

#ifdef CONFIG_INPUT_ST_LSM303DLH_INPUT_DEVICE
	disable_irq(gpio_to_irq(ddata->pdata.irq_m));
#endif

	ret = lsm303dlh_m_set_mode(ddata, LSM303DLH_M_MODE_SLEEP);
	if (ret < 0)
		return ret;

	regulator_disable(ddata->regulator);
	return ret;
}

static int __lsm303dlh_m_resume(struct device *dev, bool csave)
{
	struct lsm303dlh_m_data *ddata;
	int ret;

	ddata = dev_get_drvdata(dev);

	/* in correct mode, no need to change it */
	if (ddata->mode == LSM303DLH_M_MODE_SLEEP)
		return 0;

#ifdef CONFIG_INPUT_ST_LSM303DLH_INPUT_DEVICE
	enable_irq(gpio_to_irq(ddata->pdata.irq_m));
#endif

	regulator_enable(ddata->regulator);

	if (!csave)
		return 0;

	ret = lsm303dlh_m_restore(ddata);
	if (ret < 0)
		return ret;

	return ret;
}

static int lsm303dlh_m_resume(struct device *dev)
{
	return __lsm303dlh_m_resume(dev, true);
}

static int lsm303dlh_m_thaw(struct device *dev)
{
	return __lsm303dlh_m_resume(dev, false);
}

static const struct dev_pm_ops lsm303dlh_m_dev_pm_ops = {
	.suspend = lsm303dlh_m_suspend,
	.resume = lsm303dlh_m_resume,
	.freeze = lsm303dlh_m_suspend,
	.thaw = lsm303dlh_m_thaw,
	.poweroff = lsm303dlh_m_suspend,
	.restore = lsm303dlh_m_resume,
};
#endif /* CONFIG_PM */

static const struct i2c_device_id lsm303dlh_m_id[] = {
	{ "lsm303dlh_m", 0 },
	{ },
};

#ifdef CONFIG_OF
static const struct of_device_id lsm303dlh_m_of_match[] = {
	{.compatible = "st,lsm303dlh_m",},
	{}
};

MODULE_DEVICE_TABLE(of, lsm303dlh_m_of_match);
#endif

static struct i2c_driver lsm303dlh_m_driver = {
	.probe		= lsm303dlh_m_probe,
	.remove		= lsm303dlh_m_remove,
	.id_table	= lsm303dlh_m_id,
	.driver = {
		.name = "lsm303dlh_m",
		.owner = THIS_MODULE,
	#ifdef CONFIG_OF
		.of_match_table = lsm303dlh_m_of_match,
	#endif
	#ifdef CONFIG_PM
		.pm = &lsm303dlh_m_dev_pm_ops,
	#endif
	},
};

static int __init lsm303dlh_m_init(void)
{
	return i2c_add_driver(&lsm303dlh_m_driver);
}
module_init(lsm303dlh_m_init);

static void __exit lsm303dlh_m_exit(void)
{
	i2c_del_driver(&lsm303dlh_m_driver);
}
module_exit(lsm303dlh_m_exit);

MODULE_DESCRIPTION("LSM303DLH 3-Axis Magnetometer Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("STMicroelectronics");
