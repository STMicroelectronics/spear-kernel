/*
 * AD9889B - Analog Devices AD9889B HDMI Transmitter
 *
 * SPEAr1340 evaluation board source file
 *
 * Copyright (C) 2011 ST Microelectronics
 * Author: Imran Khan <imran.khan@st.com>
 * Pratyush Anand <pratyush.anand@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/ad9889b.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/param.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>

/* Interrupt MASK/STATUS bits */
#define MASK_AD9889B_EDID_RDY_INT	0x04
#define MASK_AD9889B_MSEN_INT		0x40
#define MASK_AD9889B_HPD_INT		0x80
#define MASK_AD9889B_AUDIO_FIFO_INT	0x10

/* Event detection bits */
#define MASK_AD9889B_HPD_DETECT		0x40
#define MASK_AD9889B_MSEN_DETECT	0x20
#define MASK_AD9889B_EDID_RDY		0x10

/* EDID read related macros */
#define EDID_MAX_RETRIES (8)
#define EDID_DELAY 250

/* I2C write try count */
#define I2C_WRITE_TRIES		10
#define I2C_READ_TRIES		10

/* AD9889B edid status */
struct ad9889b_state_edid {
	u8 edid_data[256];
	unsigned read_retries;
	u8 edid_read_incomplete;
	int segment;
};

/* AD9889B status */
struct ad9889b_state {
	u8 power_on;
	/* Did we receive hotplug and rx-sense signals? */
	u8 have_monitor;
	int status;
	/* Running counter of the number of detected EDIDs (for debugging) */
	struct ad9889b_state_edid edid;
	struct workqueue_struct *work_queue;
	struct i2c_client *client;
	struct i2c_client *edid_client;
	struct delayed_work edid_handler;
	struct mutex lock_sync;
};

/* Get AD9889B register values */
static int ad9889b_rd(struct i2c_client *client, u8 reg)
{
	int value = -1;
	int count;

	for (count = 0; count < I2C_READ_TRIES; count++) {
		value = i2c_smbus_read_byte_data(client, reg);
		if (value < 0) /* error in i2c reading */
			continue;
		else
			break;
	}

	return value;
}

/* Set AD9889B register values */
static int ad9889b_wr(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;
	int count;

	for (count = 0; count < I2C_WRITE_TRIES; count++) {
		ret = i2c_smbus_write_byte_data(client, reg, val);
		if (ret == 0)
			return 0;
	}

	return ret;
}

/*
 * To set specific bits in the register, a clear-mask is given (to be AND-ed),
 * and then the value-mask (to be OR-ed).
 */
static void ad9889b_wr_and_or(struct i2c_client *client, u8 reg,
		u8 clr_mask, u8 val_mask)
{
	ad9889b_wr(client, reg,
			(ad9889b_rd(client, reg) & clr_mask) | val_mask);
}

/* Check hotplug status */
static u8 ad9889b_have_hotplug(struct i2c_client *client)
{
	return ad9889b_rd(client, 0x42) & MASK_AD9889B_HPD_DETECT;
}

/* SET AVMUTE */
static void ad9889b_av_mute_on(struct i2c_client *client)
{
	/* set chxPwrDwnI2C */
	ad9889b_wr(client, 0xa1, 0x38);
	/* set avmute*/
	ad9889b_wr(client, 0x45, 0x40);
}

/* CLEAR AVMUTE */
static void ad9889b_av_mute_off(struct i2c_client *client)
{
	/* clear chxPwrDwnI2C */
	ad9889b_wr(client, 0xa1, 0x0);
	/* clear avmute*/
	ad9889b_wr(client, 0x45, 0x80);
}

/* Read edid from edid registers */
static void ad9889b_edid_rd(struct i2c_client *client, u16 len, u8 *buf)
{
	int i;

	for (i = 0; i < len; i++)
		buf[i] = ad9889b_rd(client, i);
}

/*
 * Check if EDID is ready to be read.
 * Configure CLCD as per EDID params.
 */
static int ad9889b_check_edid_status(struct ad9889b_state *state)
{
	struct fb_var_screeninfo screen;
	uint8_t edidRdy;
	struct i2c_client *client = state->client;
	struct ad9889b_pdata *pdata = dev_get_platdata(&client->dev);
	int ret = 0, i;

	memset(&screen, 0, sizeof(struct fb_var_screeninfo));

	msleep(500);

	dev_dbg(&client->dev, "EDID READY = %x\n", (edidRdy & 0x10));
	dev_dbg(&client->dev, "EDID READY (retries: %d)\n",
			EDID_MAX_RETRIES - state->edid.read_retries);

	if (!registered_fb[pdata->fb]) {
		dev_err(&client->dev, "framebuffer device not found\n");
		return -ENODEV;
	}

	edidRdy = ad9889b_rd(client, 0xc5);

	if (edidRdy & MASK_AD9889B_EDID_RDY) {

		/* TODO: Check if EDID ready bit is reliable */
		state->edid.segment = ad9889b_rd(client, 0xc4);

		/* Read EDID information */
		ad9889b_edid_rd(state->edid_client,
				sizeof(state->edid.edid_data),
				state->edid.edid_data);

		/* Parse edid first 128 byte */
		for (i = 0 ; i < 2 ; i++) {
			ret = fb_parse_edid(&state->edid.edid_data[i * 128],
					&screen);
			if (!ret) {
				screen.activate = FB_ACTIVATE_NOW;
				/*
				 * TODO: Find a way to fill following
				 * information automatically
				 */
				screen.bits_per_pixel = 32;
				ret = fb_set_var(registered_fb[pdata->fb],
						&screen);
				if (!ret)
					return ret;
				else
					continue;
			}
		}

		ad9889b_wr(client, 0xc4, state->edid.segment + 1);
	} else {
		if (state->edid.read_retries <= 0)
			dev_err(&client->dev, "unable to read edid\n");
		return -EINPROGRESS;
	}

	return ret;
}

/* Work queue handler for reading edid */
static void ad9889b_edid_handler(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct ad9889b_state *state = container_of(dwork,
			struct ad9889b_state, edid_handler);
	int ret;

	/* Return if we received the EDID. */
	ret = ad9889b_check_edid_status(state);
	if (!ret || ret == -ENODEV)
		return;

	/*
	 * We must retry reading the EDID several times, it is possible
	 * that initially the EDID couldn't be read due to i2c errors
	 * (DVI connectors are particularly prone to this problem)
	 */
	if (state->edid.read_retries) {
		state->edid.read_retries--;
		/* EDID read failed, trigger a retry */
		ad9889b_wr(state->client, 0xc9, 0xf);
		queue_delayed_work(state->work_queue,
				&state->edid_handler, EDID_DELAY);
		return;
	}
}

/*
 * Interrupt handler for reading edid
 */
static void ad9889b_edid_reader(struct ad9889b_state *state)
{
	state->edid.read_retries = EDID_MAX_RETRIES;

	queue_delayed_work(state->work_queue, &state->edid_handler,
			EDID_DELAY);
	return;
}

/*
 * Set N value for ACR. CTS will be configured automatically.
 */
static int ad9889b_s_clock_freq(struct i2c_client *client, u32 freq)
{
	u32 N;
	switch (freq) {
	case 32000:
		N = 4096;
		break;
	case 44100:
		N = 6272;
		break;
	case 48000:
		N = 6144;
		break;
	case 88200:
		N = 12544;
		break;
	case 96000:
		N = 12288;
		break;
	case 176400:
		N = 25088;
		break;
	case 192000:
		N = 24576;
		break;
	default:
		return -EINVAL;
	}

	/* Set N (used with CTS to regenerate the audio clock) */
	ad9889b_wr(client, 0x01, (N >> 16) & 0xf);
	ad9889b_wr(client, 0x02, (N >> 8) & 0xff);
	ad9889b_wr(client, 0x03, N & 0xff);
	ad9889b_wr_and_or(client, 0x0c, 0xc3, 0x04);

	return 0;
}

/* Set I2S sampling frequency. */
static int ad9889b_s_i2s_clock_freq(struct i2c_client *client, u32 freq)
{
	u32 i2s_sf;

	switch (freq) {
	case 32000:
		i2s_sf = 0x30;
		break;
	case 44100:
		i2s_sf = 0x00;
		break;
	case 48000:
		i2s_sf = 0x20;
		break;
	case 88200:
		i2s_sf = 0x80;
		break;
	case 96000:
		i2s_sf = 0xa0;
		break;
	case 176400:
		i2s_sf = 0xc0;
		break;
	case 192000:
		i2s_sf = 0xe0;
		break;
	default:
		return -EINVAL;
	}

	/* Set sampling frequency for I2S audio to i2s_sf */
	ad9889b_wr_and_or(client, 0x15, 0xf, i2s_sf);

	return 0;
}

/* Set audio routing */
static int ad9889b_s_routing(struct i2c_client *client, u32 input, u32 output,
		u32 config)
{
	/* Only 2 channels in use for application */
	ad9889b_wr_and_or(client, 0x50, 0x1f, 0x20);
	/* Speaker mapping */
	ad9889b_wr(client, 0x51, 0x00);

	/* TODO Where should this be placed? */
	/* 16 bit audio word length */
	ad9889b_wr_and_or(client, 0x14, 0xf0, 0x02);

	return 0;
}

/* Setup audio : Set sampling freq to 44.1 KHz */
static void ad9889b_audio_setup(struct i2c_client *client)
{
	ad9889b_s_i2s_clock_freq(client, 44100);
	ad9889b_s_clock_freq(client, 44100);
	ad9889b_s_routing(client, 0, 0, 0);
}

/* Configure hdmi transmitter. */
static void ad9889b_setup(struct i2c_client *client)
{
	struct ad9889b_pdata *pdata = dev_get_platdata(&client->dev);

	/* Input format: RGB 4:4:4 */
	ad9889b_wr_and_or(client, 0x15, 0xf1, 0x0);
	/* Output format: RGB 4:4:4 */
	ad9889b_wr_and_or(client, 0x16, 0x3f, 0x0);
	/*
	 * CSC fixed point: +/-2,
	 * 1st order interpolation 4:2:2 -> 4:4:4 up conversion,
	 * Aspect ratio: 16:9
	 */
	ad9889b_wr_and_or(client, 0x17, 0xe1, 0x0e);
	/* Disable pixel repetition and CSC */
	ad9889b_wr_and_or(client, 0x3b, 0x9e, 0x0);
	/* Output format: RGB 4:4:4, Active Format Information is valid. */
	ad9889b_wr_and_or(client, 0x45, 0xc7, 0x08);
	/* Underscanned */
	ad9889b_wr_and_or(client, 0x46, 0x3f, 0x80);
	/* Full range RGB (0-255) */
	ad9889b_wr_and_or(client, 0xcd, 0xf9, 0x00);
	/* Setup video format */
	ad9889b_wr(client, 0x3c, 0x0);
	/* Active format aspect ratio: same as picure. */
	ad9889b_wr(client, 0x47, 0x80);
	ad9889b_wr_and_or(client, 0xaf, 0xfd, 0x02);
	ad9889b_wr_and_or(client, 0x44, 0xef, 0x10);
	/* No encryption */
	ad9889b_wr_and_or(client, 0xaf, 0xef, 0x0);
	/* Positive clk edge capture for input video clock */
	ad9889b_wr_and_or(client, 0xba, 0x1f, 0x60);

	if (pdata->ain == HDMI_AUDIO_IN_SPDIF)
		ad9889b_wr(client, 0x44, 0xF8);
	else
		ad9889b_wr(client, 0x44, 0x78);
	ad9889b_wr_and_or(client, 0x73, 0xf8, 0x01);

	ad9889b_audio_setup(client);

	dev_dbg(&client->dev, "HDMI initital setup complete\n");
}

/* Power up/down ad9889b */
static int ad9889b_s_power(struct i2c_client *client, u8 on)
{
	struct ad9889b_state *state = i2c_get_clientdata(client);
	struct ad9889b_pdata *pdata = dev_get_platdata(&client->dev);
	const int retries = 20;
	int i;

	dev_dbg(&client->dev, "power %s\n", on ? "on" : "off");

	state->power_on = on;

	if (!on) {
		/* Power down */
		ad9889b_wr_and_or(client, 0x41, 0xbf, 0x40);
		return true;
	}

	/*
	 * Power up. The ad9889b does not always come up immediately.
	 * Retry multiple times.
	 */
	for (i = 0; i < retries; i++) {
		ad9889b_wr_and_or(client, 0x41, 0xbf, 0x0);
		if ((ad9889b_rd(client, 0x41) & 0x40) == 0)
			break;
		ad9889b_wr_and_or(client, 0x41, 0xbf, 0x40);
		msleep(10); /*TODO Or msleep_interruptible(10) */
	}
	if (i == retries) {
		dev_dbg(&client->dev, "Failed to powerup the ad9889b!\n");
		ad9889b_s_power(client, 0);
		return false;
	}
	if (i > 1)
		dev_dbg(&client->dev,
			"needed %d retries to powerup the ad9889b\n", i);

	/* Select chip: AD9389B */
	ad9889b_wr_and_or(client, 0xba, 0xef, 0x00);
	udelay(10);
	ad9889b_av_mute_on(client);

	/* Reserved registers that must be set */
	ad9889b_wr(client, 0x98, 0x03); /* for 9389 - 0x03 for 9889-0x07 */
	udelay(10); /* adding udelays between successive writes */
	ad9889b_wr(client, 0x9c, 0x38);
	udelay(10); /* adding udelays between successive writes */
	ad9889b_wr_and_or(client, 0x9d, 0xfc, 0x01);
	udelay(10); /* adding udelays between successive writes */
	ad9889b_wr(client, 0xa2, 0x84)
		;/* pixel clock > 80 then 87 otherwise 84 */
	udelay(10); /* adding udelays between successive writes */
	ad9889b_wr(client, 0xa3, 0x84);
	/* pixel clock > 80 then 87 otherwise 84 */
	udelay(10); /* adding udelays between successive writes */
	if (pdata->ain == HDMI_AUDIO_IN_SPDIF)
		ad9889b_wr_and_or(client, 0x0a, 0x9f, 0x10);
	else
		ad9889b_wr_and_or(client, 0x0a, 0x9f, 0x00);
	udelay(10); /* adding udelays between successive writes */
	ad9889b_wr(client, 0x9f, 0x70);
	/* according to document it shud be 00 for proper operation */
	udelay(10); /* adding udelays between successive writes */
	ad9889b_wr(client, 0xbb, 0xff);
	udelay(10); /* adding udelays between successive writes */

	/* Set number of attempts to read the EDID */
	ad9889b_wr(client, 0xc9, 0xf);
	udelay(10); /* adding udelays between successive writes */

	return true;
}

static void ad9889b_check_monitor_present_status(struct i2c_client *client)
{
	struct ad9889b_state *state = i2c_get_clientdata(client);
	uint8_t status;

	/* wait for Rx sense and HPD status bits to get stable */
	mdelay(200);
	/* read hotplug and rx-sense state */
	status = ad9889b_rd(client, 0x42);

	dev_dbg(&client->dev, "status: 0x%x%s%s\n",
			 status,
			 status & MASK_AD9889B_HPD_DETECT ? ", hotplug" : "",
			 status & MASK_AD9889B_MSEN_DETECT ? ", rx-sense" : "");

	if (status & MASK_AD9889B_HPD_DETECT) {
		dev_dbg(&client->dev, "HPD detected ; wait for Rx sense\n");
		mdelay(500);
	}

	if ((status & MASK_AD9889B_HPD_DETECT)
			&& ((status & MASK_AD9889B_MSEN_DETECT))) {
		dev_dbg(&client->dev, "hotplug and rx-sense\n");
		if (!state->have_monitor) {
			dev_dbg(&client->dev, "monitor detected\n");
			state->have_monitor = true;
			if (!ad9889b_s_power(client, true)) {
				dev_dbg(&client->dev, "monitor detected, \
						powerup failed\n");
				return;
			}
			ad9889b_setup(client);
			ad9889b_av_mute_off(client);
		}
	} else if (status & MASK_AD9889B_HPD_DETECT) {
		dev_dbg(&client->dev, "ONLY hotplug detected\n");
	} else if (!(status & MASK_AD9889B_HPD_DETECT)) {
		dev_dbg(&client->dev, "hotplug not detected\n");
		if (state->have_monitor) {
			dev_dbg(&client->dev, "monitor not detected\n");
			state->have_monitor = false;
			state->edid.edid_read_incomplete = 1;
		}
		ad9889b_s_power(client, false);
		memset(&state->edid, 0, sizeof(struct ad9889b_state_edid));
	}
}

/* Enable/disable ad9889b output */
static int ad9889b_s_stream(struct i2c_client *client, u8 enable)
{
	struct ad9889b_state *state = i2c_get_clientdata(client);

	ad9889b_wr_and_or(client, 0xa1, ~0x3c, (enable ? 0 : 0x3c));
	if (enable) {
		ad9889b_check_monitor_present_status(client);
	} else {
		ad9889b_s_power(client, 0);
		state->have_monitor = false;
	}
	return 0;
}

/* Enable/disable interrupts */
static void ad9889b_set_isr(struct i2c_client *client, u8 enable)
{
	uint8_t irqs = MASK_AD9889B_HPD_INT | MASK_AD9889B_MSEN_INT;
	uint8_t irqs_rd;
	int retries = 100;
	u8 hpd = ad9889b_have_hotplug(client);
	struct ad9889b_state *state = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "hot_plug_state = %x edid_read_incomplete = %x \
			enable = %d \n", hpd, state->edid.edid_read_incomplete,
			enable);
	/*
	 * The datasheet says that the EDID ready interrupt should be
	 * disabled if there is no hotplug.
	 */
	if (!enable) {
		irqs = 0;
		dev_dbg(&client->dev, "GOING TO DISABLE ALL INTERRUPTS\n");
	} else if (ad9889b_have_hotplug(client)
			&& state->edid.edid_read_incomplete) {
		dev_dbg(&client->dev, "Enabling EDID READY INTERRUPT \
				with HPD STATE = %d \n",
				ad9889b_have_hotplug(client));
		irqs |= MASK_AD9889B_EDID_RDY_INT;
	}
	/*
	 * This i2c write can fail (approx. 1 in 1000 writes). But it
	 * is essential that this register is correct, so retry it
	 * multiple times.
	 *
	 * Note that the i2c write does not report an error, but the readback
	 * clearly shows the wrong value.
	 */
set_isr:
	ad9889b_wr(client, 0x94, irqs);
	irqs_rd = ad9889b_rd(client, 0x94);
	if (retries-- && irqs_rd != irqs)
		goto set_isr;
	if (irqs_rd == irqs)
		return;
	dev_dbg(&client->dev, "Could not set interrupts: hw failure?\n");
}

/* Quick Interrupt handler*/
static irqreturn_t ad9889b_quick_isr(int irq, void *dev_id)
{
	return IRQ_WAKE_THREAD;
}

/* Thread Interrupt handler */
static irqreturn_t ad9889b_thread_isr(int irq, void *dev_id)
{
	uint8_t irq_status;
	struct i2c_client *client = dev_id;
	struct ad9889b_state *state = i2c_get_clientdata(client);

	mutex_lock(&state->lock_sync);

	/* disable interrupts to prevent a race condition */
	ad9889b_set_isr(client, false);
	irq_status = ad9889b_rd(client, 0x96);

	/* clear detected interrupts */
	ad9889b_wr(client, 0x96, irq_status);

	if (irq_status & (MASK_AD9889B_HPD_INT))
		ad9889b_check_monitor_present_status(client);
	ad9889b_edid_reader(state);
	/* enable interrupts */
	ad9889b_set_isr(client, true);

	mutex_unlock(&state->lock_sync);

	return 0;
}

/* Setup AD9889b */
void ad9889b_init_setup(struct i2c_client *client, u8 enable)
{
	struct ad9889b_state *state = i2c_get_clientdata(client);
	struct ad9889b_state_edid *edid = &state->edid;

	/* clear all interrupts */
	ad9889b_wr(client, 0x96, 0xff);

	memset(edid, 0, sizeof(struct ad9889b_state_edid));
	state->have_monitor = false;
	if (ad9889b_rd(client, 0x42) & MASK_AD9889B_HPD_DETECT)
		ad9889b_s_stream(client, enable);
	else
		ad9889b_s_stream(client, 0);

	ad9889b_edid_reader(state);
}

static int ad9889b_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret;
	struct ad9889b_pdata *pdata;
	static struct ad9889b_state *state;

	pdata = dev_get_platdata(&client->dev);
	if (!pdata) {
		dev_err(&client->dev, "no platform data\n");
		return -ENODEV;
	}
	if (pdata->irq_gpio < 0) {
		dev_err(&client->dev, "no gpio data\n");
		return -ENODEV;
	}

	dev_info(&client->dev, "Probing in %s, addr 0x%x\n",
			client->name, client->addr);
	if (client->addr != 0x39) {
		dev_err(&client->dev, "wrong i2c address\n");
		return -ENODEV;
	}

	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_I2C_BLOCK)) {
		dev_err(&client->dev, "i2c bus does not support the adi9889\n");
		return -ENODEV;
	}

	state = kmalloc(sizeof(struct ad9889b_state), GFP_KERNEL);
	if (!state) {
		dev_err(&client->dev, "no memory for ad9889b state\n");
		return -ENOMEM;
	}

	state->edid.edid_read_incomplete = true;
	state->client = client;

	mutex_init(&state->lock_sync);
	mutex_lock(&state->lock_sync);

	i2c_set_clientdata(client, state);

	/* disable interrupts at the beginning */
	ad9889b_set_isr(client, false);

	ret = gpio_request(pdata->irq_gpio, "ad9889b");
	if (ret < 0) {
		dev_err(&client->dev, "gpio request fail: %d\n",
				pdata->irq_gpio);
		goto err_kfree;
	}

	ret = gpio_direction_input(pdata->irq_gpio);
	if (ret) {
		dev_err(&client->dev, "gpio set direction fail: %d\n",
				pdata->irq_gpio);
		goto err_free_gpio;
	}

	ret = request_threaded_irq(gpio_to_irq(pdata->irq_gpio),
				ad9889b_quick_isr, ad9889b_thread_isr,
				pdata->irq_type, "ad9889b_isr", client);
	if (ret) {
		dev_err(&client->dev, "could not request IRQ %d for detect \
				pin\n", gpio_to_irq(pdata->irq_type));
		goto err_free_gpio;
	}

	state->edid_client = i2c_new_dummy(client->adapter, (0x7e>>1));
	if (state->edid_client == NULL) {
		dev_err(&client->dev, "FATAL :failed to register edid i2c \
				client\n");
		ret = -ENODEV;
		goto err_free_irq;
	}
	state->work_queue = create_singlethread_workqueue("ad9889b_event");
	if (state->work_queue == NULL) {
		dev_err(&client->dev, "could not create workqueue\n");
		ret = -EINVAL;
		goto err_unreg;
	}

	INIT_DELAYED_WORK(&state->edid_handler, ad9889b_edid_handler);
	/* Initially start everything */
	ad9889b_init_setup(client, 1);
	/* enable isr at the end */
	ad9889b_set_isr(client, true);
	mutex_unlock(&state->lock_sync);
	return ret;

err_unreg:
	i2c_unregister_device(state->edid_client);
err_free_irq:
	free_irq(gpio_to_irq(pdata->irq_gpio), client);
err_free_gpio:
	gpio_free(pdata->irq_gpio);
err_kfree:
	i2c_set_clientdata(client, NULL);
	mutex_unlock(&state->lock_sync);
	kfree(state);

	return ret;
}

static int ad9889b_i2c_remove(struct i2c_client *client)
{
	struct ad9889b_state *state = i2c_get_clientdata(client);
	struct ad9889b_pdata *pdata = dev_get_platdata(&client->dev);

	ad9889b_init_setup(client, 0);
	cancel_delayed_work(&state->edid_handler);
	destroy_workqueue(state->work_queue);
	i2c_unregister_device(state->edid_client);
	free_irq(gpio_to_irq(pdata->irq_gpio), client);
	gpio_free(pdata->irq_gpio);
	i2c_set_clientdata(client, NULL);
	kfree(state);

	return 0;
}

static const struct i2c_device_id ad9889b_register_id[] = {
	{ "adi9889_i2c", 0 },
	{ "adi9889_edid_i2c", 0},
};

static struct i2c_driver ad9889b_driver = {
	.driver = {
		.name = "adi9889_i2c",
		.owner = THIS_MODULE,
	},
	.probe = ad9889b_i2c_probe,
	.remove = ad9889b_i2c_remove,
	.id_table = ad9889b_register_id,
};

static int __init ad9889b_init(void)
{

	return i2c_add_driver(&ad9889b_driver);
}

static void __exit ad9889b_exit(void)
{
	i2c_del_driver(&ad9889b_driver);
}

module_init(ad9889b_init);
module_exit(ad9889b_exit);

MODULE_AUTHOR("imran.khan@st.com");
MODULE_DESCRIPTION("AD9889b HDMI transmitter driver");
MODULE_LICENSE("GPL");

