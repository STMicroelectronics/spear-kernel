/*
 * drivers/input/touchscreen/stmpe610.c
 *
 * ST Microelectronics S-Touch Advance Touchscreen controller's source file
 *
 * Copyright (C) 2010 ST Microelectronics
 * Viresh Kumar <viresh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/stmpe610.h>
#include <linux/workqueue.h>
#include "stmpe610-regs.h"

void stmpe610_dump(struct stmpe610 *ts)
{
	u32 val = 0;

	dev_info(ts->dev, "ID_VER: %x\n", stmpe610_read(ts->idata, ID_VER));
	dev_info(ts->dev, "SYS_CTRL1: %x\n", stmpe610_read(ts->idata,
				SYS_CTRL1));
	dev_info(ts->dev, "SYS_CTRL2: %x\n", stmpe610_read(ts->idata,
				SYS_CTRL2));
	dev_info(ts->dev, "SPI_CFG: %x\n", stmpe610_read(ts->idata, SPI_CFG));
	dev_info(ts->dev, "INT_CTRL: %x\n", stmpe610_read(ts->idata, INT_CTRL));
	dev_info(ts->dev, "INT_EN: %x\n", stmpe610_read(ts->idata, INT_EN));
	dev_info(ts->dev, "INT_STA: %x\n", stmpe610_read(ts->idata, INT_STA));
	dev_info(ts->dev, "GPIO_EN: %x\n", stmpe610_read(ts->idata, GPIO_EN));
	dev_info(ts->dev, "GPIO_INT_STA: %x\n", stmpe610_read(ts->idata,
				GPIO_INT_STA));
	dev_info(ts->dev, "GPIO_AF: %x\n", stmpe610_read(ts->idata, GPIO_AF));
	dev_info(ts->dev, "ADC_CTRL1: %x\n", stmpe610_read(ts->idata,
				ADC_CTRL1));
	dev_info(ts->dev, "ADC_CTRL2: %x\n", stmpe610_read(ts->idata,
				ADC_CTRL2));
	dev_info(ts->dev, "TSC_CTRL: %x\n", stmpe610_read(ts->idata, TSC_CTRL));
	dev_info(ts->dev, "TSC_CFG: %x\n", stmpe610_read(ts->idata, TSC_CFG));

	stmpe610_block_read(ts->idata, WDW_TR_X, 2, (u8 *)&val);
	dev_info(ts->dev, "WDW_TR_X: %x\n", val);
	stmpe610_block_read(ts->idata, WDW_TR_Y, 2, (u8 *)&val);
	dev_info(ts->dev, "WDW_TR_Y: %x\n", val);
	dev_info(ts->dev, "WDW_BL_X: %x\n", stmpe610_read(ts->idata, WDW_BL_X));
	dev_info(ts->dev, "WDW_BL_Y: %x\n", stmpe610_read(ts->idata, WDW_BL_Y));
	dev_info(ts->dev, "FIFO_TH: %x\n", stmpe610_read(ts->idata, FIFO_TH));
	dev_info(ts->dev, "FIFO_STA: %x\n", stmpe610_read(ts->idata, FIFO_STA));
	dev_info(ts->dev, "FIFO_SIZE: %x\n", stmpe610_read(ts->idata,
				FIFO_SIZE));
	dev_info(ts->dev, "TSC_I_DRIVE: %x\n", stmpe610_read(ts->idata,
				TSC_I_DRIVE));
	dev_info(ts->dev, "TSC_SHIELD: %x\n", stmpe610_read(ts->idata,
				TSC_SHIELD));
}

void reset_fifo(struct stmpe610 *ts)
{
	stmpe610_write(ts->idata, FIFO_STA, RESET_FIFO);
	stmpe610_write(ts->idata, FIFO_STA, ENB_FIFO);
}

void set_operating_mode(struct stmpe610 *ts)
{
	switch (ts->pdata->operating_mode) {
	case XYZ_ACQUISITION:
		ts->mode |= DATAZ;
	case XY_ACQUISITION:
		ts->mode |= DATAY;
	case X_ACQUISITION:
		ts->mode |= DATAX;
		break;
	case Y_ACQUISITION:
		ts->mode = DATAY;
		break;
	case Z_ACQUISITION:
		ts->mode = DATAZ;
		break;
	default:
		break;
	}
}

static void stmpe_work(struct work_struct *work)
{
	struct stmpe610 *ts = container_of(work, struct stmpe610, work);
	u32 x = 0, y = 0, z = 0, xtmp, ytmp, status;
	int size = 0, release = 0;

	status = stmpe610_read(ts->idata, INT_STA);
	if (!(status & 0x3)) {
		enable_irq(gpio_to_irq(ts->pdata->irq_gpio));
		return;
	}

	/* Disable interrupts */
	stmpe610_write(ts->idata, INT_EN, 0x0);

	/* Return the 'Release' event */
	if (ts->event_count != 0 && (status & TOUCH_INT)) {
		release = 1;
		ts->event_count = 0;
	} else {
		ts->event_count++;
		if (ts->event_count == 1)
			input_report_key(ts->input, BTN_TOUCH, 1);
	}

	/* Clear FIFO_TH_INT and TOUCH_INT Status event */
	stmpe610_write(ts->idata, INT_STA, FIFO_TH_INT | TOUCH_INT);

	size = stmpe610_read(ts->idata, FIFO_SIZE);
	while (size > 0) {
		u8 data[4] = {0, };

		stmpe610_block_read(ts->idata, TSC_DATA_XYZ, 4, data);
		if (ts->mode & DATAX) {
			x = (data[0] << 4) | (data[1] >> 4);
			xtmp = XY_MAX - x;
			input_report_abs(ts->input, ABS_X, xtmp);
		}
		if (ts->mode & DATAY) {
			y = ((data[1] & 0xf) << 8) | data[2];
			ytmp = XY_MAX - y;
			input_report_abs(ts->input, ABS_Y, ytmp);
		}
		if (ts->mode & DATAZ) {
			z = data[3];
			input_report_abs(ts->input, ABS_PRESSURE, z);
		}

		input_sync(ts->input);
		size -= 4;
	}

	if (release) {
		input_report_abs(ts->input, ABS_PRESSURE, 0);
		input_report_key(ts->input, BTN_TOUCH, 0);
		input_sync(ts->input);
	}

	/* Re-enable interrupts */
	reset_fifo(ts);

	msleep(10);
	stmpe610_write(ts->idata, INT_EN, FIFO_TH_INT | TOUCH_INT);
	enable_irq(gpio_to_irq(ts->pdata->irq_gpio));
}

static irqreturn_t stmpe610_irq(int irq, void *dev_id)
{
	struct stmpe610 *ts = dev_id;

	disable_irq_nosync(gpio_to_irq(ts->pdata->irq_gpio));
	queue_work(ts->wq, &ts->work);

	return IRQ_HANDLED;
}

void stmpe610_config(struct stmpe610 *ts)
{
	u32 val = 0, ctrl, chip_id = 0;
	struct stmpe610_pdata *pd = ts->pdata;

	/* reset TS */
	stmpe610_write(ts->idata, SYS_CTRL1, SOFT_RESET);
	/* Enable clk of TS, ADC, GPIO */
	stmpe610_write(ts->idata, SYS_CTRL2, CLK_ENABLE);
	/* configure adc */
	stmpe610_write(ts->idata, ADC_CTRL1, pd->sample_time | pd->ref_sel |
			pd->mod_12b | 1);
	stmpe610_write(ts->idata, ADC_CTRL2, pd->adc_freq);
	stmpe610_write(ts->idata, GPIO_AF, 0x0);
	/* configure TS */
	ctrl = OP_MODE(pd->operating_mode) |
		TRACKING_INDEX(pd->tracking_index);
	stmpe610_write(ts->idata, TSC_CTRL, ctrl);

	/* Configure interrupts */
	val = GLOBAL_INT_EN;
	stmpe610_write(ts->idata, INT_CTRL, val);

	/* Configure Touch Detection & set FIFO threshhold */
	stmpe610_write(ts->idata, INT_EN, FIFO_TH_INT | TOUCH_INT);

	/* set average ctrl, settling time and touch det delay */
	val = SETTLING_TIME(pd->settling_time) |
		TOUCH_DET_DELAY(pd->touch_det_delay) |
		AVG_CTRL(pd->average_ctrl);
	stmpe610_write(ts->idata, TSC_CFG, val);

	/* set window coordinates */
	stmpe610_write(ts->idata, WDW_BL_X, pd->x_min);
	stmpe610_write(ts->idata, WDW_TR_X,
			pd->x_max > XY_MAX ? XY_MAX : pd->x_max);
	stmpe610_write(ts->idata, WDW_BL_Y, pd->y_min);
	stmpe610_write(ts->idata, WDW_TR_Y,
			pd->y_max > XY_MAX ? XY_MAX : pd->y_max);
	stmpe610_write(ts->idata, TSC_FRACT_Z, pd->fraction_z);
	stmpe610_write(ts->idata, TSC_I_DRIVE, pd->i_drive);
	reset_fifo(ts);
	/* Enable FIFO threshhold interrupt */
	stmpe610_write(ts->idata, FIFO_TH, pd->fifo_threshhold > 0xFF ?
			0xFF : pd->fifo_threshhold);

	stmpe610_block_read(ts->idata, CHIP_ID, 2, (u8 *)&chip_id);
	dev_info(ts->dev, "Detected Touch Screen with chip id: %x and "
			"version: %x\n", chip_id, stmpe610_read(ts->idata,
				ID_VER));

	/* Enable touchscreen */
	ctrl |= TSC_ENB;
	stmpe610_write(ts->idata, TSC_CTRL, ctrl);
}

struct stmpe610 *stmpe610_probe(struct device *dev)
{
	struct stmpe610 *ts;
	struct stmpe610_pdata *pdata;
	int ret = 0;

	pdata = dev_get_platdata(dev);
	if (!pdata) {
		dev_dbg(dev, "no platform data?\n");
		return ERR_PTR(-ENODEV);
	}
	if (pdata->irq_gpio < 0)
		return ERR_PTR(-ENODEV);

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return ERR_PTR(-ENOMEM);

	ts->dev = dev;
	ts->pdata = pdata;

	ret = gpio_request(pdata->irq_gpio, "stmpe610");
	if (ret < 0) {
		dev_dbg(dev, "gpio request fail: %d\n", pdata->irq_gpio);
		goto err_gpio_request;
	}

	ret = gpio_direction_input(pdata->irq_gpio);
	if (ret) {
		dev_dbg(dev, "gpio set direction fail: %d\n", pdata->irq_gpio);
		goto err_igpio_direction;
	}

	ret = request_irq(gpio_to_irq(pdata->irq_gpio), stmpe610_irq,
			ts->pdata->irq_type, "stmpe610", ts);
	if (ret) {
		dev_dbg(dev, "request irq fail: %d\n", pdata->irq_gpio);
		goto err_request_irq;
	}

	ts->input = input_allocate_device();
	if (!ts->input) {
		ret = -ENOMEM;
		goto err_input_alloc;
	}
	spin_lock_init(&ts->lock);

	snprintf(ts->phys, sizeof(ts->phys), "%s/input0", dev_name(dev));
	snprintf(ts->name, sizeof(ts->name), "STMPE610 Touchscreen");

	ts->input->name = ts->name;
	ts->input->phys = ts->phys;
	ts->input->dev.parent = dev;

	set_operating_mode(ts);
	ts->input->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	ts->input->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	if (ts->mode & DATAX)
		input_set_abs_params(ts->input, ABS_X, 0, XY_MAX, 0, 0);
	if (ts->mode & DATAY)
		input_set_abs_params(ts->input, ABS_Y, 0, XY_MAX, 0, 0);
	if (ts->mode & DATAZ)
		input_set_abs_params(ts->input, ABS_PRESSURE, 0, ~0, 0, 0);

	ret = input_register_device(ts->input);
	if (ret)
		goto err_input_register;

	INIT_WORK(&ts->work, stmpe_work);
	ts->wq = create_singlethread_workqueue(dev_name(dev->parent));
	if (!ts->wq) {
		ret = -EBUSY;
		goto err_create_wq;
	}

	dev_set_drvdata(dev, ts);
	return ts;

err_create_wq:
	input_unregister_device(ts->input);
err_input_register:
	input_free_device(ts->input);
err_input_alloc:
	free_irq(gpio_to_irq(pdata->irq_gpio), ts);
err_request_irq:
err_igpio_direction:
	gpio_free(pdata->irq_gpio);
err_gpio_request:
	kfree(ts);

	return ERR_PTR(ret);
}

void stmpe610_remove(struct device *dev)
{
	struct stmpe610 *ts = dev_get_drvdata(dev);

	destroy_workqueue(ts->wq);
	input_unregister_device(ts->input);
	dev_set_drvdata(dev, NULL);
	free_irq(gpio_to_irq(ts->pdata->irq_gpio), ts);
	gpio_free(ts->pdata->irq_gpio);
	input_free_device(ts->input);
	kfree(ts);
}
