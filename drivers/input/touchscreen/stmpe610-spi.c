/*
 * drivers/input/touchscreen/stmpe610-spi.c
 *
 * ST Microelectronics S-Touch Advance Touchscreen controller with 6 bit port
 * expander's spi interface source file
 *
 * Copyright (C) 2010 ST Microelectronics
 * Viresh Kumar <viresh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/err.h>
#include "stmpe610-regs.h"

struct spi_req {
	u16			cmd;
	u16			response;
	struct spi_message	msg;
	struct spi_transfer	xfer;
};

int stmpe610_read(struct spi_device *spi, u8 reg)
{
	struct spi_req req;
	int status;

	req.cmd = reg | READ_CMD;

	spi_message_init(&req.msg);
	req.xfer.len = 2;
	req.xfer.tx_buf = &req.cmd;
	req.xfer.rx_buf = &req.response;

	spi_message_add_tail(&req.xfer, &req.msg);

	status = spi_sync(spi, &req.msg);
	return status ? status : (req.response >> 8);
}

int stmpe610_block_read(struct spi_device *spi, u8 reg, u8 len, u8 *rx_buf)
{
	int ret, i;

	for (i = 0; i < len; i++) {
		ret = stmpe610_read(spi, reg + i);
		if (ret < 0)
			return ret;
		*(rx_buf + i) = ret;
	}

	return 0;
}

int stmpe610_write(struct spi_device *spi, u8 reg, u8 val)
{
	struct spi_req req;
	int status;

	spi_message_init(&req.msg);

	req.cmd = (val << 8) | reg;
	req.xfer.len = 2;
	req.xfer.tx_buf = &req.cmd;
	req.xfer.rx_buf = NULL;

	spi_message_add_tail(&req.xfer, &req.msg);

	status = spi_sync(spi, &req.msg);
	return status;
}

int stmpe610_block_write(struct spi_device *spi, u8 reg, u8 len, u8 *tx_buf)
{
	int ret = 0, i;

	for (i = len; i > 0; i--, reg++)
		ret = stmpe610_write(spi, reg, *(tx_buf + i - 1));

	return ret;
}

static int __devinit stmpe610_spi_probe(struct spi_device *spi)
{
	struct stmpe610 *ts;
	int err;

	/* don't exceed max specified rate - 1MHz */
	if (spi->max_speed_hz > 1000000) {
		dev_dbg(&spi->dev, "f(sample) %d KHz?\n",
				(spi->max_speed_hz/1000));
		return -EINVAL;
	}

	ts = stmpe610_probe(&spi->dev);
	if (IS_ERR(ts))
		return PTR_ERR(ts);

	ts->idata = spi;
	spi->bits_per_word = 8;
	err = spi_setup(spi);
	if (err < 0)
		goto err_spi_setup;

	stmpe610_config(ts);
	return 0;

err_spi_setup:
	stmpe610_remove(&spi->dev);
	return err;
}

static int __devexit stmpe610_spi_remove(struct spi_device *spi)
{
	stmpe610_remove(&spi->dev);
	return 0;
}

static struct spi_driver stmpe610_driver = {
	.driver = {
		.name	= "stmpe610-spi",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe		= stmpe610_spi_probe,
	.remove		= __devexit_p(stmpe610_spi_remove),
};

static int __init stmpe610_init(void)
{
	return spi_register_driver(&stmpe610_driver);
}
module_init(stmpe610_init);

static void __exit stmpe610_exit(void)
{
	spi_unregister_driver(&stmpe610_driver);
}
module_exit(stmpe610_exit);

MODULE_AUTHOR("Viresh Kumar <viresh.kumar@st.com>");
MODULE_DESCRIPTION("STMPE610 TouchScreen Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:stmpe610");
