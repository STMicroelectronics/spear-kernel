/*
 * drivers/input/misc/l3g4200d_gyr.c
 *
 * ST Microelectronics L3G4200D digital output gyroscope sensor API
 *
 * Copyright (C) 2010 ST Microelectronics
 *
 * Developed by:
 * Carmine Iascone <carmine.iascone@st.com>
 * Matteo Dameno <matteo.dameno@st.com>
 *
 * Currently maintained by:
 * Amit Virdi <amit.virdi@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/i2c.h>
#include <linux/i2c/l3g4200d.h>
#include <linux/input-polldev.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/slab.h>

/* Maximum polled-device-reported rot speed value value in dps */
#define FS_MAX		32768

/* l3g4200d gyroscope registers */
#define WHO_AM_I	0x0F
#define CTRL_REG1	0x20
#define CTRL_REG2	0x21
#define CTRL_REG3	0x22
#define CTRL_REG4	0x23
#define CTRL_REG5	0x24

/* CTRL_REG1 */
#define PM_OFF		0x00
#define PM_NORMAL	0x08
#define ENABLE_ALL_AXES	0x07
#define BW00		0x00
#define BW01		0x10
#define BW10		0x20
#define BW11		0x30
#define ODR100		0x00	/* ODR = 100Hz */
#define ODR200		0x40	/* ODR = 200Hz */
#define ODR400		0x80	/* ODR = 400Hz */
#define ODR800		0xC0	/* ODR = 800Hz */

#define AXISDATA_REG    0x28

#define FUZZ		0
#define FLAT		0
#define AUTO_INCREMENT	0x80
#define FS_RANGE_MASK	0xCF

/* Minimum polling interval in ms */
#define MIN_POLL_INTERVAL	2
#define MIN_FS_RANGE		250

/** Registers Contents */
#define WHOAMI_L3G4200D	0x00D3	/* Expected content for WAI register*/

/*
 * L3G4200D gyroscope data
 * brief structure containing gyroscope values for yaw, pitch and roll in
 * signed short
 */
struct l3g4200d_t {
	short	x,	/* x-axis angular rate data. */
		y,	/* y-axis angluar rate data. */
		z;	/* z-axis angular rate data. */
};

/**
 * L3G4200D gyroscope output data rate.
 * @poll_rate_ms: polling rate (in ms)
 * @mask: The value by which CTRL_REG1 is masked to give an appropriate
 *	output data rate.
 */
struct output_rate {
	int poll_rate_ms;
	u8 mask;
};

/**
 * L3G4200D gyroscope parameters' current settings
 * @poll_rate_ms: current polling rate
 * @fs_range: current full scale range
 */
struct gyro_parameters {
	int poll_rate_ms;
	int fs_range;
};

struct full_scale_range {
	int range;
	int reg_val;
};

struct l3g4200d_data {
	struct i2c_client *client;
	struct l3g4200d_gyr_platform_data *pdata;
	struct mutex lock;
	struct gyro_parameters params;
	struct input_polled_dev *input_poll_dev;
	int hw_initialized;
	atomic_t enabled;
	u8 reg_addr;
	u8 register_state[5];
};

static const struct output_rate odr_table[] = {
	{2,	ODR800|BW10},
	{3,	ODR400|BW01},
	{5,	ODR200|BW00},
	{10,	ODR100|BW00},
};

static const struct full_scale_range fs_range_table[] = {
	{250,	0x00},
	{500,	0x10},
	{2000,	0x20},
};

static int l3g4200d_i2c_read(struct l3g4200d_data *gyro, u8 *buf, int len)
{
	int num_msgs = 0;
	struct i2c_msg msgs[] = {
		{
			.addr = gyro->client->addr,
			.flags = gyro->client->flags & I2C_M_TEN,
			.len = 1,
			.buf = buf,
		}, {
			.addr = gyro->client->addr,
			.flags = (gyro->client->flags & I2C_M_TEN) | I2C_M_RD,
			.len = len,
			.buf = buf,
		},
	};

	num_msgs = i2c_transfer(gyro->client->adapter, msgs, 2);
	if (num_msgs != 2) {
		dev_err(&gyro->client->dev, "read transfer error\n");
		/* Return the num_msgs as it contains the error code */
		return num_msgs;
	}

	return 0;
}

static int l3g4200d_i2c_write(struct l3g4200d_data *gyro, u8 *buf, u8 len)
{
	int num_msgs = 0;
	struct i2c_msg msgs[] = {
		{
			.addr = gyro->client->addr,
			.flags = gyro->client->flags & I2C_M_TEN,
			.len = len + 1,
			.buf = buf,
		},
	};

	num_msgs = i2c_transfer(gyro->client->adapter, msgs, 1);
	if (num_msgs != 1) {
		dev_err(&gyro->client->dev, "write transfer error\n");
		/* Return the num_msgs as it contains the error code */
		return num_msgs;
	}

	return 0;
}

static int l3g4200d_update_fs_range(struct l3g4200d_data *gyro, u32 new_fs)
{
	int err = -EIO, res = 0;
	int idx = 0;
	int reg_val = 0;
	u8 buf[2];

	if (new_fs < MIN_FS_RANGE)
		idx = 0;
	else
		for (idx = ARRAY_SIZE(fs_range_table) - 1; idx >= 0; idx--)
			if (fs_range_table[idx].range <= new_fs)
				break;

	reg_val = fs_range_table[idx].reg_val;

	if (atomic_read(&gyro->enabled)) {
		buf[0] = CTRL_REG4;
		res = l3g4200d_i2c_read(gyro, buf, 1);
		if (res == 0)
			buf[0] = buf[0] & FS_RANGE_MASK;
		else
			return res;

		buf[1] = reg_val|buf[0];
		buf[0] = CTRL_REG4;
		err = l3g4200d_i2c_write(gyro, buf, 1);
		if (err < 0)
			return err;
	}
	gyro->register_state[3] = buf[1];
	gyro->params.fs_range = fs_range_table[idx].range;

	return err;
}

static int l3g4200d_update_odr(struct l3g4200d_data *gyro, int poll_interval)
{
	int err = -EIO, index;
	u8 config[2];

	if (poll_interval < MIN_POLL_INTERVAL)
		index = 0;
	else
		for (index = ARRAY_SIZE(odr_table) - 1; index >= 0; index--)
			if (odr_table[index].poll_rate_ms <= poll_interval)
				break;

	config[1] = odr_table[index].mask;
	config[1] |= ENABLE_ALL_AXES + PM_NORMAL;

	/*
	 * If device is currently enabled, we need to write new
	 * configuration out to it
	 * */
	if (atomic_read(&gyro->enabled)) {
		config[0] = CTRL_REG1;
		err = l3g4200d_i2c_write(gyro, config, 1);
		if (err < 0)
			return err;
	}

	gyro->register_state[0] = config[1];
	gyro->params.poll_rate_ms = odr_table[index].poll_rate_ms;

	return 0;
}

/* Read gyroscope data from the registers */
static int
l3g4200d_get_data(struct l3g4200d_data *gyro, struct l3g4200d_t *data)
{
	int err = -EIO;
	u8 gyro_out[6] = {0};
	/* y,p,r hardware data */
	s16 hw_d[3] = {0};

	gyro_out[0] = AUTO_INCREMENT | AXISDATA_REG;
	err = l3g4200d_i2c_read(gyro, gyro_out, 6);
	if (err < 0)
		return err;

	hw_d[0] = (s16)((gyro_out[1] << 8) | gyro_out[0]);
	hw_d[1] = (s16)((gyro_out[3] << 8) | gyro_out[2]);
	hw_d[2] = (s16)((gyro_out[5] << 8) | gyro_out[4]);

	data->x = gyro->pdata->negate_x ? -hw_d[gyro->pdata->axis_map_x]
			: hw_d[gyro->pdata->axis_map_x];
	data->y = gyro->pdata->negate_y ? -hw_d[gyro->pdata->axis_map_y]
			: hw_d[gyro->pdata->axis_map_y];
	data->z = gyro->pdata->negate_z ? -hw_d[gyro->pdata->axis_map_z]
			: hw_d[gyro->pdata->axis_map_z];

	dev_dbg(&gyro->client->dev, "out: x = %d y = %d z= %d\n",
			data->x, data->y, data->z);

	return err;
}

static void
l3g4200d_report_values(struct l3g4200d_data *l3g, struct l3g4200d_t *data)
{
	struct input_dev *input = l3g->input_poll_dev->input;

	input_report_abs(input, ABS_X, data->x);
	input_report_abs(input, ABS_Y, data->y);
	input_report_abs(input, ABS_Z, data->z);
	input_sync(input);
}

static int l3g4200d_hw_init(struct l3g4200d_data *gyro)
{
	int err = -EIO;
	u8 buf[6];

	dev_dbg(&gyro->client->dev, "%s hw init\n", L3G4200D_DEV_NAME);

	buf[0] = AUTO_INCREMENT | CTRL_REG1;
	buf[1] = gyro->register_state[0];
	buf[2] = gyro->register_state[1];
	buf[3] = gyro->register_state[2];
	buf[4] = gyro->register_state[3];
	buf[5] = gyro->register_state[4];

	err = l3g4200d_i2c_write(gyro, buf, 5);
	if (err < 0)
		return err;

	gyro->hw_initialized = 1;
	return 0;
}

static void l3g4200d_device_power_off(struct l3g4200d_data *dev_data)
{
	int err = -EIO;
	u8 buf[2] = {CTRL_REG1, PM_OFF} ;

	dev_dbg(&dev_data->client->dev, "%s power off\n", L3G4200D_DEV_NAME);

	err = l3g4200d_i2c_write(dev_data, buf, 1);
	if (err < 0)
		dev_err(&dev_data->client->dev, "soft power off failed\n");

	else if (dev_data->hw_initialized)
		dev_data->hw_initialized = 0;
}

static int l3g4200d_device_power_on(struct l3g4200d_data *dev_data)
{
	int err = -EIO;

	if (!dev_data->hw_initialized) {
		err = l3g4200d_hw_init(dev_data);
		if (err < 0) {
			l3g4200d_device_power_off(dev_data);
			return err;
		}
	}

	return 0;
}

static int l3g4200d_enable(struct l3g4200d_data *dev_data)
{
	int err = -EIO;

	if (!atomic_cmpxchg(&dev_data->enabled, 0, 1)) {
		err = l3g4200d_device_power_on(dev_data);
		if (err < 0) {
			atomic_set(&dev_data->enabled, 0);
			return err;
		}
	}

	return 0;
}

static int l3g4200d_disable(struct l3g4200d_data *dev_data)
{
	if (atomic_cmpxchg(&dev_data->enabled, 1, 0))
		l3g4200d_device_power_off(dev_data);

	return 0;
}

static ssize_t
attr_get_polling_rate(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int val;
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);

	mutex_lock(&gyro->lock);
	val = gyro->params.poll_rate_ms;
	mutex_unlock(&gyro->lock);

	return sprintf(buf, "%d\n", val);
}

static ssize_t
attr_set_polling_rate(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);
	unsigned long interval_ms;
	size_t err = 0;

	if (strict_strtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;

	mutex_lock(&gyro->lock);
	err = l3g4200d_update_odr(gyro, interval_ms);
	mutex_unlock(&gyro->lock);

	if (err < 0)
		return err;

	return size;
}

static ssize_t
attr_get_range(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);
	int range = 0;

	mutex_lock(&gyro->lock);
	range = gyro->params.fs_range;
	mutex_unlock(&gyro->lock);

	return sprintf(buf, "%d\n", range);
}

static ssize_t
attr_set_range(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);
	unsigned long val;
	size_t err = 0;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	mutex_lock(&gyro->lock);
	err = l3g4200d_update_fs_range(gyro, val);
	mutex_unlock(&gyro->lock);

	if (err < 0)
		return err;

	return size;
}

static ssize_t
attr_get_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);
	int val = atomic_read(&gyro->enabled);

	return sprintf(buf, "%d\n", val);
}

static ssize_t
attr_set_enable(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);
	unsigned long val;
	size_t err = 0;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		err = l3g4200d_enable(gyro);
	else
		err = l3g4200d_disable(gyro);

	if (err < 0)
		return err;

	return size;
}

#ifdef L3G4200D_DEBUG
static ssize_t
attr_reg_set(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	size_t err = 0;
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);
	u8 x[2];
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;

	mutex_lock(&gyro->lock);
	x[0] = gyro->reg_addr;
	mutex_unlock(&gyro->lock);

	x[1] = val;
	err = l3g4200d_i2c_write(gyro, x, 1);

	if (err < 0)
		return err;

	return size;
}

static ssize_t
attr_reg_get(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret;
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);
	int rc;
	u8 data;

	mutex_lock(&gyro->lock);
	data = gyro->reg_addr;
	mutex_unlock(&gyro->lock);

	rc = l3g4200d_i2c_read(gyro, &data, 1);

	if (rc < 0)
		return rc;

	ret = sprintf(buf, "0x%02x\n", data);
	return ret;
}

static ssize_t
attr_addr_set(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;

	mutex_lock(&gyro->lock);
	gyro->reg_addr = val;
	mutex_unlock(&gyro->lock);

	return size;
}
#endif

static struct device_attribute attributes[] = {
	__ATTR(pollrate_ms, 0666, attr_get_polling_rate, attr_set_polling_rate),
	__ATTR(range, 0666, attr_get_range, attr_set_range),
	__ATTR(enable, 0666, attr_get_enable, attr_set_enable),
#ifdef L3G4200D_DEBUG
	__ATTR(reg_value, 0600, attr_reg_get, attr_reg_set),
	__ATTR(reg_addr, 0200, NULL, attr_addr_set),
#endif
};

static int create_sysfs_interfaces(struct device *dev)
{
	int index;
	for (index = 0; index < ARRAY_SIZE(attributes); index++)
		if (device_create_file(dev, attributes + index))
			goto error_dev_create;
	return 0;

error_dev_create:
	index--;
	for ( ; index >= 0; index--)
		device_remove_file(dev, attributes + index);

	dev_err(dev, "%s:Unable to create interface\n", __func__);
	return -EINVAL;
}

static int remove_sysfs_interfaces(struct device *dev)
{
	int index;

	for (index = 0; index < ARRAY_SIZE(attributes); index++)
		device_remove_file(dev, attributes + index);
	return 0;
}

static void l3g4200d_input_poll_func(struct input_polled_dev *dev)
{
	struct l3g4200d_data *gyro = dev->private;
	struct l3g4200d_t data_out;
	int err = -EIO;

	if (!gyro->hw_initialized)
		return;

	mutex_lock(&gyro->lock);
	err = l3g4200d_get_data(gyro, &data_out);
	if (err < 0)
		dev_err(&gyro->client->dev, "get_gyroscope_data failed\n");
	else
		l3g4200d_report_values(gyro, &data_out);

	mutex_unlock(&gyro->lock);
}

int l3g4200d_input_open(struct input_dev *input)
{
	struct l3g4200d_data *gyro = input_get_drvdata(input);

	return l3g4200d_enable(gyro);
}

void l3g4200d_input_close(struct input_dev *dev)
{
	struct l3g4200d_data *gyro = input_get_drvdata(dev);

	l3g4200d_disable(gyro);
}

static int l3g4200d_validate_pdata(struct l3g4200d_data *gyro)
{
	gyro->pdata->poll_interval = max(gyro->pdata->poll_interval,
			gyro->pdata->min_interval);

	if (gyro->pdata->axis_map_x > 2 ||
			gyro->pdata->axis_map_y > 2 ||
			gyro->pdata->axis_map_z > 2) {
		dev_err(&gyro->client->dev,
				"invalid axis_map value x:%u y:%u z%u\n",
				gyro->pdata->axis_map_x,
				gyro->pdata->axis_map_y,
				gyro->pdata->axis_map_z);
		return -EINVAL;
	}

	/* Only allow 0 and 1 for negation boolean flag */
	if (gyro->pdata->negate_x > 1 ||
			gyro->pdata->negate_y > 1 ||
			gyro->pdata->negate_z > 1) {
		dev_err(&gyro->client->dev,
				"invalid negate value x:%u y:%u z:%u\n",
				gyro->pdata->negate_x,
				gyro->pdata->negate_y,
				gyro->pdata->negate_z);
		return -EINVAL;
	}

	/* Enforce minimum polling interval */
	if (gyro->pdata->poll_interval < gyro->pdata->min_interval) {
		dev_err(&gyro->client->dev,
				"minimum poll interval violated\n");
		return -EINVAL;
	}
	return 0;
}

static int l3g4200d_input_init(struct l3g4200d_data *gyro)
{
	int err = -EIO;
	struct input_dev *input;

	gyro->input_poll_dev = input_allocate_polled_device();
	if (!gyro->input_poll_dev) {
		dev_err(&gyro->client->dev, "input device allocate failed\n");
		return -ENOMEM;
	}

	gyro->input_poll_dev->private = gyro;
	gyro->input_poll_dev->poll = l3g4200d_input_poll_func;
	gyro->input_poll_dev->poll_interval = gyro->pdata->poll_interval;

	input = gyro->input_poll_dev->input;

	input->open = l3g4200d_input_open;
	input->close = l3g4200d_input_close;

	input->id.bustype = BUS_I2C;
	input->dev.parent = &gyro->client->dev;

	input_set_drvdata(gyro->input_poll_dev->input, gyro);

	set_bit(EV_ABS, input->evbit);

	input_set_abs_params(input, ABS_X, -FS_MAX, FS_MAX, FUZZ, FLAT);
	input_set_abs_params(input, ABS_Y, -FS_MAX, FS_MAX, FUZZ, FLAT);
	input_set_abs_params(input, ABS_Z, -FS_MAX, FS_MAX, FUZZ, FLAT);

	input->name = L3G4200D_DEV_NAME;
	input->phys = L3G4200D_DEV_NAME "/input0";

	err = input_register_polled_device(gyro->input_poll_dev);
	if (err) {
		dev_err(&gyro->client->dev,
				"unable to register input polled device %s\n",
				gyro->input_poll_dev->input->name);
		goto err_register_polled_dev;
	}

	return 0;

err_register_polled_dev:
	input_free_polled_device(gyro->input_poll_dev);
	return err;
}

static void l3g4200d_input_cleanup(struct l3g4200d_data *gyro)
{
	input_unregister_polled_device(gyro->input_poll_dev);
}

#ifdef CONFIG_OF
static int __devinit l3g4200d_gyr_probe_config_dt(
		struct l3g4200d_gyr_platform_data *pdata,
		struct device_node *np)
{
	if (of_property_read_u32(np, "poll_interval",
				&pdata->poll_interval))
		pr_warn("unable to get poll_interval.");
	if (of_property_read_u32(np, "min_interval", &pdata->min_interval))
		pr_warn("unable to get minimum poll interval.");
	if (of_property_read_u32(np, "axis_map_x", &pdata->axis_map_x))
		pr_warn("unable to get axis_map_x value.");
	if (of_property_read_u32(np, "axis_map_y", &pdata->axis_map_y))
		pr_warn("unable to get axis_map_y");
	if (of_property_read_u32(np, "axis_map_z", &pdata->axis_map_z))
		pr_warn("unable to get axis_map_z");
	if (of_property_read_u32(np, "negate_x", &pdata->negate_x))
		pr_warn("unable to get negate_x, default to %d\n",
				pdata->negate_x);
	if (of_property_read_u32(np, "negate_y", &pdata->negate_y))
		pr_warn("unable to get negate_x, default to %d\n",
				pdata->negate_y);
	if (of_property_read_u32(np, "negate_z", &pdata->negate_z))
		pr_warn("unable to get negate_x, default to %d\n",
				pdata->negate_z);
	return 0;
}
#else
static int __devinit l3g4200d_gyr_probe_config_dt(
		struct l3g4200d_gyr_platform_data *pdata,
		struct device_node *np)
{
	return -ENOSYS;
}
#endif

static int
l3g4200d_probe(struct i2c_client *client, const struct i2c_device_id *devid)
{
	struct l3g4200d_data *gyro;
	struct device_node *np = client->dev.of_node;
	int err = -EIO;
	u8 dummy_reg;
	int ret;

	dev_dbg(&client->dev, "%s: probe start.\n", L3G4200D_DEV_NAME);

	if (client == NULL) {
		pr_err("%s: I2C Client data is NULL. Exiting.",
						L3G4200D_DEV_NAME);
		return -EINVAL;
	}

	if (!client->dev.platform_data && !np) {
		dev_err(&client->dev, "platform data is NULL. Exiting.\n");
		return -ENODEV;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "client not i2c capable.\n");
		return -ENODEV;
	}

	dummy_reg = i2c_smbus_read_byte_data(client, WHO_AM_I);

	if (WHOAMI_L3G4200D != dummy_reg) {
		dev_err(&client->dev, "Device not present.\n");
		return -ENODEV;
	}

	gyro = kzalloc(sizeof(*gyro), GFP_KERNEL);
	if (gyro == NULL) {
		dev_err(&client->dev,
				"failed to allocate memory for module data\n");
		err = -ENOMEM;
		goto err_pdata_null;
	}

	mutex_init(&gyro->lock);
	mutex_lock(&gyro->lock);
	gyro->client = client;
	gyro->pdata = kzalloc(sizeof(*gyro->pdata), GFP_KERNEL);
	if (gyro->pdata == NULL) {
		dev_err(&client->dev,
			"failed to allocate memory for pdata: %d\n", err);
		goto err_malloc_failed;
	}

	if (np) {
		ret = l3g4200d_gyr_probe_config_dt(gyro->pdata, np);
		if (ret) {
			dev_err(&client->dev, "no platform data\n");
			return -ENODEV;
		}
	} else
		memcpy(gyro->pdata, client->dev.platform_data,
				sizeof(*gyro->pdata));

	err = l3g4200d_validate_pdata(gyro);
	if (err < 0) {
		dev_err(&client->dev, "failed to validate platform data\n");
		goto err_invalid_pdata;
	}

	i2c_set_clientdata(client, gyro);

	gyro->register_state[0] = 0x07;
	gyro->register_state[1] = 0x00;
	gyro->register_state[2] = 0x00;
	gyro->register_state[3] = 0x00;
	gyro->register_state[4] = 0x00;

	err = l3g4200d_device_power_on(gyro);
	if (err < 0) {
		dev_err(&client->dev, "power on failed: %d\n", err);
		goto err_reset_cdata;
	}

	atomic_set(&gyro->enabled, 1);
	err = l3g4200d_update_fs_range(gyro, gyro->pdata->fs_range);
	if (err < 0) {
		dev_err(&client->dev, "update_fs_range failed\n");
		goto err_reset_cdata;
	}

	err = l3g4200d_update_odr(gyro, gyro->pdata->poll_interval);
	if (err < 0) {
		dev_err(&client->dev, "update_odr failed\n");
		goto err_reset_cdata;
	}

	err = l3g4200d_input_init(gyro);
	if (err < 0)
		goto err_input_init;

	err = create_sysfs_interfaces(&client->dev);
	if (err < 0) {
		dev_err(&client->dev,
			"%s device register failed\n", L3G4200D_DEV_NAME);
		goto err_sysfs_intf;
	}

	l3g4200d_device_power_off(gyro);

	/* As default, do not report information */
	atomic_set(&gyro->enabled, 0);

	mutex_unlock(&gyro->lock);
	dev_info(&client->dev, "%s probed: device created successfully\n",
			L3G4200D_DEV_NAME);

	return 0;

err_sysfs_intf:
	l3g4200d_input_cleanup(gyro);
err_input_init:
	l3g4200d_device_power_off(gyro);
err_reset_cdata:
	i2c_set_clientdata(client, NULL);
err_invalid_pdata:
	kfree(gyro->pdata);
err_malloc_failed:
	mutex_unlock(&gyro->lock);
	kfree(gyro);
err_pdata_null:
	dev_err(&client->dev, "%s: Driver Initialization failed\n",
			L3G4200D_DEV_NAME);
	return err;
}

static int l3g4200d_remove(struct i2c_client *client)
{
	struct l3g4200d_data *gyro = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "L3G4200D driver removing\n");
	l3g4200d_input_cleanup(gyro);
	l3g4200d_device_power_off(gyro);
	remove_sysfs_interfaces(&client->dev);

	i2c_set_clientdata(client, NULL);
	kfree(gyro->pdata);
	kfree(gyro);
	return 0;
}

#ifdef CONFIG_PM
static int l3g4200d_suspend(struct device *dev)
{
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);

	if (atomic_read(&gyro->enabled))
		l3g4200d_device_power_off(gyro);

	return 0;
}

static int l3g4200d_resume(struct device *dev)
{
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);

	if (!atomic_read(&gyro->enabled))
		return 0;

	l3g4200d_device_power_on(gyro);

	return 0;
}

static const struct dev_pm_ops l3g4200d_pm = {
	.suspend = l3g4200d_suspend,
	.resume = l3g4200d_resume,
	.freeze = l3g4200d_suspend,
	.thaw = l3g4200d_resume,
	.poweroff = l3g4200d_suspend,
	.restore = l3g4200d_resume,
};
#endif

static const struct i2c_device_id l3g4200d_id[] = {
	{ L3G4200D_DEV_NAME , 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, l3g4200d_id);

#ifdef CONFIG_OF
static const struct of_device_id l3g4200d_gyro_of_match[] = {
	{.compatible = "st,l3g4200d_gyro",},
	{}
};

MODULE_DEVICE_TABLE(of, l3g4200d_gyro_of_match);
#endif

static struct i2c_driver l3g4200d_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = L3G4200D_DEV_NAME,
#ifdef CONFIG_OF
		.of_match_table = l3g4200d_gyro_of_match,
#endif
#ifdef CONFIG_PM
		.pm = &l3g4200d_pm,
#endif
	},
	.probe = l3g4200d_probe,
	.remove = __devexit_p(l3g4200d_remove),
	.id_table = l3g4200d_id,
};

static int __init l3g4200d_init(void)
{
	pr_debug("%s: gyroscope sysfs driver init\n", L3G4200D_DEV_NAME);
	return i2c_add_driver(&l3g4200d_driver);
}
module_init(l3g4200d_init);

static void __exit l3g4200d_exit(void)
{
	pr_debug("%s: L3G4200D exit\n", L3G4200D_DEV_NAME);
	i2c_del_driver(&l3g4200d_driver);
	return;
}
module_exit(l3g4200d_exit);

MODULE_DESCRIPTION("l3g4200d digital gyroscope sysfs driver");
MODULE_AUTHOR("Carmine Iascone <carmine.iascone@st.com>,\
		Matteo Dameno <matteo.dameno@st.com> ");
MODULE_LICENSE("GPL");
