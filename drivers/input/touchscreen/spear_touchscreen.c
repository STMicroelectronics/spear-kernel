/*
 * linux/drivers/input/touchscreen/spear_touchscreen.c
 * Touch screen driver
 *
 * Copyright (C) 2010 STmicroelectronics. (vipul kumar samar)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License aspublished by
 * the Free Software Foundation; either version 2 of the License,or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will beuseful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/freezer.h>
#include <linux/slab.h>
#include <plat/touchscreen.h>
#include "../../char/spear_adc.h"

#define DEBUG 0
/* Values to provide to Input layer, right now just arbit*/
#define X_AXIS_MAX	1023
#define X_AXIS_MIN      0
#define Y_AXIS_MAX	1023
#define Y_AXIS_MIN	0
#define PRESSURE_MIN	0
#define PRESSURE_MAX	15000

/* Number of reading from ADC Channels to get Idle value */
#define IDLE_SAMPLES	50

/* Valid touch is considered, if digital value corsses Threshold */
#define TS_THRESHOLD	50

/* Since 1 jiffie is 10mS, therefore 10 will be 100mS */
#define THREAD_POLLING_TIME 10

/* Name used by platform bus and Input Layer, ID used by Input Layer */
#define SPEAR_TS_DRIVER_NAME	"spear-ts"
#define SPEAR_TS_VENDOR_ID	0x0001
#define SPEAR_TS_PRODUCT_ID	0x0002
#define SPEAR_TS_VERSION_ID	0x0100

struct ts_cordinate {
	int x;
	int y;
};

enum touch_state {
	IDLE,
	PRESS,
	RELEASE
};

/* Structure for Context of Touchscreen */
struct ts_context {
	struct ts_cordinate	idle;
	struct ts_cordinate	touched_point;
};

/**
 * struct spear_ts_dev - Structure for Touchscreen
 *
 * @pdev:		Driver model hookup
 * @input:		Input device Layer structure
 * @timer:		Timer for thread
 * @time_complete:	Completion for sync between thread and timer
 * @thread_exit:	Completion for indicating exit of kernel thread
 * @finger_state:	Finger state of Touchscreen
 * @ts_ctx:		Touchscreen related information
 * @exit_ts_thread:	Exit flag for the Kernel Thread
 */
struct spear_ts_dev {
	struct platform_device	*pdev;
	struct input_dev	*input;
	struct timer_list	timer;
	struct completion	time_complete;
	struct completion	thread_exit;
	struct spear_touchscreen *spr_ts_info;
	enum touch_state	finger_state;
	struct ts_context	ts_ctx;
	unsigned int		exit_ts_thread;
	unsigned int		ts_open_done;
};

/**
 * ts_adc_read_channel - Read ADC channels to get Touchscreen points.
 * @spear_ts:		Touchscreen data structure
 * @channel_num:	ADC channel number
 *
 * The Touch cordinate(x & y) will be given by ADC channels,
 * moreover GPIO also need to be set/reset for this purpose.
 */
static void ts_adc_read_channel(struct spear_ts_dev *spear_ts, int channel_num)
{
	int gpio_val;
	int adc_err;
	int digital_val;

	gpio_val = (spear_ts->spr_ts_info->adc_channel_x == channel_num)
								? 1 : 0;
	gpio_set_value(spear_ts->spr_ts_info->gpio_pin, gpio_val);

	msleep(21);

	/*
	 * Looping is added because Hw need some Delay after Pwr-Up, in absence
	 * of it Hw shows busy.
	 */
	while ((adc_err = spear_adc_get_data(spear_ts, channel_num,
					&digital_val, 1)) == -EAGAIN)
		;
	if (adc_err < 0) {
		dev_err(&spear_ts->pdev->dev, "Err in ADC get data=%d\n",
				adc_err);
		return;
	}

	/* Assign as per channel number */
	if (spear_ts->spr_ts_info->adc_channel_x == channel_num)
		spear_ts->ts_ctx.touched_point.x = digital_val;
	else
		spear_ts->ts_ctx.touched_point.y = digital_val;
#if DEBUG
	/* Debug code*/
	if (spear_ts->ts_ctx.idle.x < spear_ts->ts_ctx.touched_point.x)
		dev_dbg(&spear_ts->pdev->dev,
			"idle_x=%d, touched_x=%d\n",
			spear_ts->ts_ctx.idle.x,
			spear_ts->ts_ctx.touched_point.x);
	if (spear_ts->ts_ctx.idle.y < spear_ts->ts_ctx.touched_point.y)
		dev_dbg(&spear_ts->pdev->dev,
			"idle_y=%d, touched_y=%d\n",
			spear_ts->ts_ctx.idle.y,
			spear_ts->ts_ctx.touched_point.y);
#endif

}
/**
 * ts_get_coordinates - Read (x,y) coordinates and check whether
 * Touchscreen is touched.
 * @spear_ts:	Touchscreen data structure
 * @point:	Coordinates of Touch point
 *
 * The routine is in polling, and read ADC channels at every iterations. If
 * touched value crosses the Idle value then Touchscreen is assumed to be
 * touched and coordinates are sent to application.
 */
static void ts_get_coordinates(struct spear_ts_dev *spear_ts,
				     struct ts_cordinate *point)
{
	ts_adc_read_channel(spear_ts, spear_ts->spr_ts_info->adc_channel_x);
	ts_adc_read_channel(spear_ts, spear_ts->spr_ts_info->adc_channel_y);

	/*
	 * Sometime only one coordinate gets BAD! So, take it valid only if
	 * both coordinate gets change
	 */
	if ((spear_ts->ts_ctx.touched_point.x > spear_ts->ts_ctx.idle.x) &&
	    (spear_ts->ts_ctx.touched_point.y > spear_ts->ts_ctx.idle.y)) {
		point->x = spear_ts->ts_ctx.touched_point.x;
		point->y = spear_ts->ts_ctx.touched_point.y;
	} else {
		point->x = 0;
		point->y = 0;
	}
}

/**
 * ts_get_adc_idle_values - Read Idle value of touchscreen
 * @spear_ts:	 Touchscreen data structure
 *
 * The Touchscreen read the Idle value of ADC channels, while Touchscreen is not
 * Touched. Now this helps to know whether Touchscreen is touched.
 */
static void ts_get_adc_idle_values(struct spear_ts_dev *spear_ts)
{
	int x_idle = 0, y_idle = 0;
	int i;

	for (i = 0; i < IDLE_SAMPLES; i++) {

		/* Read coordinate X */
		ts_adc_read_channel(spear_ts,
				spear_ts->spr_ts_info->adc_channel_x);

		/* Valid read only if higher than Idle value */
		if (spear_ts->ts_ctx.touched_point.x > x_idle)
			x_idle = spear_ts->ts_ctx.touched_point.x;

		/* Read coordinate Y */
		ts_adc_read_channel(spear_ts,
				spear_ts->spr_ts_info->adc_channel_y);

		/* Valid read only if higher than Idle value */
		if (spear_ts->ts_ctx.touched_point.y > y_idle)
			y_idle = spear_ts->ts_ctx.touched_point.y;
	}

	spear_ts->ts_ctx.idle.x = x_idle + TS_THRESHOLD;
	spear_ts->ts_ctx.idle.y = y_idle + TS_THRESHOLD;

	dev_dbg(&spear_ts->pdev->dev,
	     "idle value of x is=%d, idle value of y is=%d\n", x_idle, y_idle);

}

/**
 * ts_write - Reports events to Input Layer
 * @spear_ts:	Touchscreen data structure
 * @point:	Coordinates of Touch point
 * @state:
 *
 * The routine reports basically 3 events: Button Press/Release, (x,y) and Sync.
 * The routine gets called when a correct touch point is obtained and need to be
 * send to application.
 */
static void ts_write(struct spear_ts_dev *spear_ts,
			   struct ts_cordinate *point,
			   enum touch_state state)
{
	struct input_dev *dev = spear_ts->input;

	/* Button Press-down */
	if (PRESS == state) {
		dev_dbg(&spear_ts->pdev->dev, "input_rep_key.BTN_TOUCHi"
				"down\n");
		input_report_key(dev, BTN_TOUCH, 1);
	} else if (RELEASE == state) {
		dev_dbg(&spear_ts->pdev->dev, "input_report_key.BTN_TOUCH"
				"up\n");
		input_report_key(dev, BTN_TOUCH, 0);
	}

	/*
	 * In case of -ve coordinate make it zero, but expect -ve coordinate
	 * before Calibration
	 */
	if (point->x < 0)
		point->x = 0;

	dev_dbg(&spear_ts->pdev->dev, "input_report_abs.x = %d\n", point->x);
	/* X coordinates */
	input_report_abs(dev, ABS_X, point->x);

	/*
	 * In case of -ve coordinate make it zero, but expect -ve coordinate
	 * before Calibration
	 */
	if (point->y < 0)
		point->y = 0;

	dev_dbg(&spear_ts->pdev->dev, "input_report_abs.y = %d\n", point->y);
	/* Y coordinates */
	input_report_abs(dev, ABS_Y, point->y);

	if (PRESS == state)
		input_report_abs(dev, ABS_PRESSURE, 255);
	else
		input_report_abs(dev, ABS_PRESSURE, 0);

	input_sync(dev);
}

/**
 * ts_adc_open - Acquire and Configure ADC channels.
 * @spear_ts:	 Touchscreen data structure
 *
 * The routine acquire and configure the ADC channels. Some configuration
 * parameters were taken as default.
 */
int ts_adc_open(struct spear_ts_dev *spear_ts)
{
	struct adc_chan_config x_cfg;
	struct adc_chan_config y_cfg;
	int adc_err;
	/* Get ADC for x coordinate */
	adc_err = spear_adc_chan_get(spear_ts,
			spear_ts->spr_ts_info->adc_channel_x);
	if (adc_err < 0) {
		dev_err(&spear_ts->pdev->dev,
			"Err in ADC channel get for x coordinate=%d\n",
			adc_err);
		return -1;
	}

	/* Get ADC for y coordinate*/
	adc_err = spear_adc_chan_get(spear_ts,
			spear_ts->spr_ts_info->adc_channel_y);
	if (adc_err < 0) {
		dev_err(&spear_ts->pdev->dev,
			"Err in ADC channel get for y coordinate=%d\n",
			adc_err);
		return -1;
	}

	/* Configure ADC for x coordinate*/
	x_cfg.chan_id	= spear_ts->spr_ts_info->adc_channel_x;
	x_cfg.avg_samples	= SAMPLE64;
	x_cfg.scan_rate	= 5000; /* micro second*/
	x_cfg.scan_rate_fixed = false;/* dma only available for chan-0 */
	adc_err = spear_adc_chan_configure(spear_ts, &x_cfg);
	if (adc_err < 0) {
		dev_err(&spear_ts->pdev->dev,
			"Err in ADC config of x coordinate=%d\n", adc_err);
		return -1;
	}

	/* Configure ADC for y coordinate*/
	y_cfg.chan_id	= spear_ts->spr_ts_info->adc_channel_y;
	y_cfg.avg_samples	= SAMPLE64;
	y_cfg.scan_rate	= 5000; /* micro second*/
	y_cfg.scan_rate_fixed = false;/* dma is only available for chan-0 */

	adc_err = spear_adc_chan_configure(spear_ts, &y_cfg);
	if (adc_err < 0) {
		dev_err(&spear_ts->pdev->dev,
			"Err in ADC config of y coordinate=%d\n", adc_err);
		return -1;
	}

	return 0;
}

/**
 * ts_adc_close - Close ADC channels
 * @spear_ts:	Touchscreen data structure
 *
 * The routine is closing ADC channels, and it is done each time after reading
 * coordinates from them.
 */
static int ts_adc_close(struct spear_ts_dev *spear_ts)
{
	int adc_err;

	/* Put ADC for for x coordinate */
	adc_err = spear_adc_chan_put(spear_ts,
			spear_ts->spr_ts_info->adc_channel_x);
	if (adc_err < 0) {
		dev_err(&spear_ts->pdev->dev,
			"Err in ADC channel put for x coordinate=%d\n",
			adc_err);
		return -1;
	}
	/* Put ADC for y coordinate */
	adc_err = spear_adc_chan_put(spear_ts,
			spear_ts->spr_ts_info->adc_channel_y);
	if (adc_err < 0) {
		dev_err(&spear_ts->pdev->dev,
			"Err in ADC channel put for y coordinate=%d\n",
			adc_err);
		return -1;
	}
	return 0;
}


/**
 * ts_open - Performs open operation for Touchscreen.
 * @spear_ts:	 Touchscreen data structure
 *
 * The routine does various operations which are required for Opening up
 * Touchscreen. Such as, Getting idle (without touch) value of ADC channels
 * and checking whether calibration data is present.
 * Routine is not called, until completion is sent by CLCD module, after CLCD
 * power on.
 */
int ts_open(struct spear_ts_dev *spear_ts)
{
	int status, gpio_err;

	gpio_err = gpio_request(spear_ts->spr_ts_info->gpio_pin, "ts_gpio");
	if (gpio_err < 0) {
		dev_err(&spear_ts->pdev->dev, "Err gpio request=%d\n",
				gpio_err);
		return -1;
	}

	gpio_err = gpio_direction_output(spear_ts->spr_ts_info->gpio_pin, 1);
	if (gpio_err < 0) {
		dev_err(&spear_ts->pdev->dev, "Err gpio dir set=%d\n",
				gpio_err);
		return -1;
	}

	/* Acquire and Configure ADC for X-Y coordinate*/
	status = ts_adc_open(spear_ts);
	if (status < 0) {
		dev_err(&spear_ts->pdev->dev, "Err adc open=%d\n", status);
		return status;
	}

	ts_get_adc_idle_values(spear_ts);

	spear_ts->finger_state = IDLE;

	spear_ts->ts_open_done = 1;

	return 0;
}

/**
 * ts_close - clear the open flag
 *
 * The routine clear the flag, which indicates whether 'Open' is done for
 * Touchscreen.
 */
void ts_close(struct spear_ts_dev *spear_ts)
{
	spear_ts->ts_open_done = 0;
	ts_adc_close(spear_ts);
	gpio_free(spear_ts->spr_ts_info->gpio_pin);
}


/**
 * ts_state_machine - State Machine of Touchscreen
 * @spear_ts:	 Touchscreen data structure
 *
 * This routine is the heart of Touchscreen module. It has a state machine and
 * initial state is IDLE, it read ADC channels repeatedly to get coordinates.
 * Once ADC value are obtained (which means Touchscreen is being touched) it
 * change State to PRESS and be there(send coordinates as well), until finger
 * is removed. After that state goes to RELEASE, sending final points the state
 * again move to IDLE.
 * The routine is not invoked until 'ts_open_done' flag is set and that
 * is done by 'ts_open'.
 */
static void ts_state_machine(struct spear_ts_dev *spear_ts)
{
	struct ts_cordinate cur_point, last_point;

	if (spear_ts->ts_open_done != 1)
		return;

	/* Get coordinates from ADC Channels */
	ts_get_coordinates(spear_ts, &cur_point);

	switch (spear_ts->finger_state) {

	case IDLE:
		dev_dbg(&spear_ts->pdev->dev, "IDLE\n");
		if ((cur_point.x > 0) && (cur_point.y > 0))
			spear_ts->finger_state = PRESS;

		break;

	case PRESS:
		dev_dbg(&spear_ts->pdev->dev, "PRESS\n");
		if (cur_point.x == 0 && cur_point.y == 0) {
			spear_ts->finger_state = RELEASE;
			break;
		}

		ts_write(spear_ts, &cur_point, PRESS);

		/*
		 * Preserving Last point, as after removing finger cordinate
		 * value will not come from Touchscreen, so we will send last
		 * cordinate
		 */
		last_point = cur_point;
		break;

	case RELEASE:
		dev_dbg(&spear_ts->pdev->dev, "RELEASE\n");
		spear_ts->finger_state = IDLE;
		ts_write(spear_ts, &last_point, RELEASE);
		break;
	} /* switch (finger_state) */
}

/**
 * spear_ts_thread - Kernel Thread
 * @data:	 Touchscreen data structure
 *
 * This kernel thread is calling 'Open for Touchscreen' once and 'State Machine'
 * repeatedly. It gets invoked at every 100 milli Second by Timer.
 */
int spear_ts_thread(void *data)
{
	struct spear_ts_dev *spear_ts = (struct spear_ts_dev *)data;

	allow_signal(SIGINT);
	allow_signal(SIGTERM);
	allow_signal(SIGKILL);
	allow_signal(SIGUSR1);

	/* Allow the thread to be frozen */
	set_freezable();

	/* If set by 'spear_ts_remove' then Exit the thread */
	while (!spear_ts->exit_ts_thread) {
		try_to_freeze();

		/* Wait for completion from Timer */
		wait_for_completion(&spear_ts->time_complete);
		if (spear_ts->ts_open_done != 1)
			ts_open(spear_ts);

		ts_state_machine(spear_ts);

		/*
		 * Program Timer (at 100 milli-Second interval) immediately
		 * after execution of TS, Benefit is that Other program will
		 * get ADC for 100 milli-Second interval
		 */
		mod_timer(&spear_ts->timer, jiffies +
						THREAD_POLLING_TIME);
	}
	/* Send signal to Removal routine */
	complete(&spear_ts->thread_exit);
	return 0;
}

/**
 * spear_ts_timer - Timer routine
 * @data:	 Touchscreen data structure
 *
 * A Timer which is programmed for very  first time at 500mSec and later at
 * 100mSec. This routine sync with kernel thread using completion.
 */
void spear_ts_timer(unsigned long data)
{
	struct spear_ts_dev *spear_ts = (struct spear_ts_dev *)data;

	/* Timer send completion to Thread after each Interval */
	complete(&spear_ts->time_complete);
}

/**
 * spear_ts_probe - Entry routine
 * @pdev:	platform device structure
 *
 * This is the initialization routine, which is invoked during boot-up.
 * It allocates memory for touch screen data structure, register input layer
 * device, creates a Kernel-thread and reads all MTD partitions for looking
 * calibration information.
 */
int __devinit spear_ts_probe(struct platform_device *pdev)
{
	struct input_dev *input_dev;
	struct spear_ts_dev *spear_ts;
	int err = -ENOMEM;

	/* Allocating Structure for Touchscreen */
	spear_ts = kzalloc(sizeof(struct spear_ts_dev), GFP_KERNEL);
	if (!spear_ts)
		goto struct_alloc_fail;

	input_dev = input_allocate_device();
	if (!input_dev)
		goto dev_alloc_fail;

	platform_set_drvdata(pdev, spear_ts);

	spear_ts->input = input_dev;
	spear_ts->spr_ts_info = pdev->dev.platform_data;
	spear_ts->pdev = pdev;

	/* Assiging data for input-device */
	input_dev->name = SPEAR_TS_DRIVER_NAME;
	input_dev->phys = "spear_ts/input0";
	input_dev->id.bustype = BUS_HOST;
	input_dev->id.vendor  = SPEAR_TS_VENDOR_ID;
	input_dev->id.product = SPEAR_TS_PRODUCT_ID;
	input_dev->id.version = SPEAR_TS_VERSION_ID;
	input_dev->dev.parent = &pdev->dev;

	/*
	 * Enable Touch and X-Y info to send to user-application via
	 * Input Device
	 */
	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

	input_set_abs_params(input_dev, ABS_X, X_AXIS_MIN, X_AXIS_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, Y_AXIS_MIN, Y_AXIS_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, PRESSURE_MIN,
			PRESSURE_MAX, 0, 0);

	init_completion(&spear_ts->time_complete);
	init_completion(&spear_ts->thread_exit);

	spear_ts->exit_ts_thread = 0;
	kernel_thread(spear_ts_thread, spear_ts, 0);

	init_timer(&spear_ts->timer);
	spear_ts->timer.data = (unsigned long)spear_ts;
	spear_ts->timer.function = spear_ts_timer;
	mod_timer(&spear_ts->timer, jiffies + THREAD_POLLING_TIME);

	/* Register the input-device */
	err = input_register_device(spear_ts->input);
	if (err)
		goto dev_regis_fail;

	return 0;

dev_regis_fail:
	del_timer_sync(&spear_ts->timer);
	input_free_device(input_dev);
dev_alloc_fail:
	kfree(spear_ts);
struct_alloc_fail:
	return err;
}

/*
 * spear_ts_remove - Exit Routine
 * @pdev:	platform device structure
 *
 * The routine is called at exit to free allocated timer, memory, devices and
 * others allocations.
 */
int __devexit spear_ts_remove(struct platform_device *pdev)
{
	struct spear_ts_dev *spear_ts = platform_get_drvdata(pdev);

	spear_ts->exit_ts_thread = 1;
	wait_for_completion(&spear_ts->thread_exit);

	del_timer_sync(&spear_ts->timer);

	ts_close(spear_ts);

	input_unregister_device(spear_ts->input);

	kfree(spear_ts);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

#ifdef CONFIG_PM
static int spear_ts_suspend(struct platform_device *dev, pm_message_t state)
{
	struct spear_ts_dev *spear_ts = platform_get_drvdata(dev);

	del_timer_sync(&spear_ts->timer);
	spear_ts->finger_state = IDLE;
	ts_close(spear_ts);

	return 0;
}

static int spear_ts_resume(struct platform_device *dev)
{
	struct spear_ts_dev *spear_ts = platform_get_drvdata(dev);
	complete(&spear_ts->time_complete);
	return 0;
}
#endif

static struct platform_driver spear_ts_driver = {
	.probe = spear_ts_probe,
	.remove = spear_ts_remove,
#ifdef CONFIG_PM
	.suspend = spear_ts_suspend,
	.resume	= spear_ts_resume,
#endif
	.driver = {
		   .name = SPEAR_TS_DRIVER_NAME,
		   .owner = THIS_MODULE,
	},
};

static int __init spear_ts_init(void)
{
	return platform_driver_register(&spear_ts_driver);
}
module_init(spear_ts_init);

static void __exit spear_ts_exit(void)
{
	platform_driver_unregister(&spear_ts_driver);
}
module_exit(spear_ts_exit);

MODULE_AUTHOR("Vipul Kumar Samar <vipulkumar.samar@st.com>");
MODULE_DESCRIPTION("SPEAr TouchScreen Driver");
MODULE_LICENSE("GPL");
