/*
 * drivers/char/spear_adc_st10.c
 * ST ADC driver - source file
 *
 * Copyright (ST) 2009 Viresh Kumar (viresh.kumar@st.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/clk.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <plat/adc.h>
#include <linux/spear_adc.h>

#define DRIVER_NAME "spear-adc"

/* adc declarations */
#define ADC_CHANNEL_NUM 8
#define ADC_DMA_WIDTH	4 /* ADC Width for DMA */

#ifndef CONFIG_ARCH_SPEAR6XX
struct adc_regs {
	u32 STATUS;
	u32 AVG;
	u32 SCAN_RATE_REG;
	u32 CLK; /* Not avail for 1340 & 1310 */
	u32 CHAN_CTRL[ADC_CHANNEL_NUM];
	u32 CHAN_DATA[ADC_CHANNEL_NUM];
};
#else
struct chan_data {
	u32 LSB;
	u32 MSB;
};
struct adc_regs {
	u32 STATUS;
	u32 pad[2];
	u32 CLK;
	u32 CHAN_CTRL[ADC_CHANNEL_NUM];
	struct chan_data CHAN_DATA[ADC_CHANNEL_NUM];
	u32 SCAN_RATE_LO_REG;
	u32 SCAN_RATE_HI_REG;
	struct chan_data AVG;
};
#endif

#define ADC_MAX_SCAN_RATE	0xFFFFFFFF

/* constants for configuring adc registers */
/* adc status reg */
#define START_CONVERSION	(1 << 0)
#define STOP_CONVERSION		(~START_CONVERSION)
#define CHANNEL_NUM(x)		((x) << 1)
#define ADC_ENABLE		(1 << 4)
#define AVG_SAMPLE(x)		((x) << 5)
#define GET_AVG_SAMPLE(x)	(((x) >> 5) & 7)
#define CONV_READY		(1 << 8)
#define VOLT_REF(x)		((x) << 9)
#define CONV_MODE(x)		((x) << 10)
#define SCAN_REF(x)		((x) << 11)
#define DMA_ENABLE		(1 << 12)
#define DMA_DISABLE		(~DMA_ENABLE)

#ifndef CONFIG_ARCH_SPEAR6XX
#define RESOLUTION(x)		((x) << 13)
#define SPEAR1340_RESOLUTION(x)	((x) << 16)
#endif

#define AVG_SAMPLE_RESET	(~(7 << 5))

/* adc average register */
#define DATA_MASK		0x000003FF

#ifndef CONFIG_ARCH_SPEAR6XX
#define DATA_MASK_HR		0x0001FFFF
#else
#define DATA_MASK_LSB		0x0000007F
#endif

/* adc scan rate reg */
#ifndef CONFIG_ARCH_SPEAR6XX
#define SCAN_RATE(x)		((x) << 0)
#else
#define SCAN_RATE_LO(x)		((x) & 0xFFFF)
#define SCAN_RATE_HI(x)		(((x) >> 0x10) & 0xFFFF)
#endif

/* adc clk reg, Not avail for 1340 & 1310 */
#define CLK_LOW(x)		(((x) & 0xf) << 0)
#define CLK_HIGH(x)		(((x) & 0xf) << 4)
#define GET_CLK_LOW(x)		((x) & 0xf)
#define GET_CLK_HIGH(x)		(((x) >> 4) & 0xf)

/* adc channel reg */
#define CHAN_AVG_SAMPLE(x)	((x) << 1)
#define CHAN_GET_AVG_SAMPLE(x)	(((x) >> 1) & 7)
#define DATA_VALID		(1 << 10)

#ifndef CONFIG_ARCH_SPEAR6XX
#define	DATA_VALID_HR		(1 << 17)	/* high resolution mask */
#endif

#define adc_readl(regs, name) \
	readl(&((regs)->name))
#define adc_writel(regs, name, val) \
	writel((val), &((regs)->name))

/* timeout for sleep */
#define ADC_TIMEOUT 100

/**
 * struct adc_chan: structure representing adc channel
 * @chan_id: channel id
 * @cdev: structure representing adc channel char device
 * @dev_num: type used to represent adc chan device numbers within the kernel
 * @owner: current owner process of channel
 * @configured: is channel configured, true or false
 * @scan_rate_fixed: configured scan rate should be equal to requested scan
 * rate or it can be less than equal to requsted scan rate
 * @avg_samples: no of average samples programmed.
 */
struct adc_chan {
	enum adc_chan_id id;
	struct cdev cdev;
	u32 dev_num;
	void *owner;
	bool configured;
	int scan_rate_fixed;
	enum adc_avg_samples avg_samples;
#ifdef CONFIG_PM
	struct adc_chan_config adc_saved_chan_config;
#endif
};

/**
 * struct adc_driver_data: adc device structure containing device specific info
 * @pdev: pointer to platform dev structure of adc.
 * @clk: clock structure for adc device
 * @clk_enbld: is clock enabled, true or false
 * @regs: base address of adc device registers
 * @rx_reg: physical address of rx register
 * @chan: array of struct adc_chan containing channel specific info
 * @data: dma specific data structure
 * @dma_chan: allocated dma channel
 * @sg: pointer to head of scatter list
 * @sg_len: number of scatter list nodes allocated
 * @cookie: cookie value returned from dma
 * @wait_queue: wait queue for single mode conversion
 * @flag_arrived: flags required for sleep for single mode conversion
 * @adc_lock: spinlock required for critical section safety
 * @usage_count: number of users using adc currently
 * @mode: current configured mode
 * @mvolt: reference voltage in volts
 * @resolution: current configured resolution, for SPEAR300 only
 * @digital_volt: pointer to memory to save data at
 * @configured: is adc configured, true or false
 * @scan_rate: current configured scan rate
 * @fixed_sr_count: number of users requested for same fixed scan rate.
 */
struct adc_driver_data {
	/* driver model hookup */
	struct platform_device *pdev;
	struct clk *clk;
	bool clk_enbld;

	/* adc register addresses */
	struct adc_regs *regs;
	dma_addr_t rx_reg;

	struct adc_chan chan[ADC_CHANNEL_NUM];
	struct adc_plat_data *data;
#ifdef CONFIG_PM
	struct adc_config adc_saved_config;
#endif
#ifdef CONFIG_SPEAR_ADC_DMA_IF
	/*dma transfer*/
	struct dma_chan *dma_chan;
	struct scatterlist *sg;
	u32 sg_len;
	dma_cookie_t cookie;
#endif

	/* wait queue and flags */
	wait_queue_head_t wait_queue;
	uint flag_arrived;
	spinlock_t adc_lock;
	u32 usage_count;

	enum adc_conv_mode mode;
	unsigned int mvolt;
#ifndef CONFIG_ARCH_SPEAR6XX
	enum adc_resolution resolution;
#endif
	u32 digital_volt;
	bool configured;
	u32 scan_rate;
	u32 fixed_sr_count;
};

/* dev structure to be kept global, as there is no way to pass this from char
 ** framework */
struct adc_driver_data *g_drv_data;

void adc_reset(void)
{
	enum adc_chan_id i = ADC_CHANNEL0;

	adc_writel(g_drv_data->regs, STATUS, 0);
	if (!of_machine_is_compatible("st,spear1340") &&
			!of_machine_is_compatible("st,spear1310"))
		adc_writel(g_drv_data->regs, CLK, 0);
	while (i <= ADC_CHANNEL7) {
		adc_writel(g_drv_data->regs, CHAN_CTRL[i], 0);
		i += ADC_CHANNEL1;
	}
#ifndef CONFIG_ARCH_SPEAR6XX
	adc_writel(g_drv_data->regs, SCAN_RATE_REG, 0);
#else
	adc_writel(g_drv_data->regs, SCAN_RATE_LO_REG, 0);
	adc_writel(g_drv_data->regs, SCAN_RATE_HI_REG, 0);
#endif
}

static s32 adc_sleep(u64 time_out)
{
	/* goto sleep */
	wait_event_interruptible_timeout(g_drv_data->wait_queue,
			(g_drv_data->flag_arrived != 0), time_out);

	/* if the flag arrived is 1, then it is normal wakeup */
	if (g_drv_data->flag_arrived == 1)
		g_drv_data->flag_arrived = 0;
	/* if the flag arrived is 0, then it is not normal wakeup */
	else {
		g_drv_data->flag_arrived = 0;
		dev_err(&g_drv_data->pdev->dev, "xfer taken too long, exiting.."
				"\n");
		return -ETIME;
	}

	return 0;
}

static void adc_wakeup(void)
{
	g_drv_data->flag_arrived = 1;
	wake_up_interruptible(&g_drv_data->wait_queue);
}

#ifdef CONFIG_SPEAR_ADC_DMA_IF
static u32 get_sg_count(size_t size)
{
	u32 max_xfer;

	if (!size) {
		dev_err(&g_drv_data->pdev->dev, "xfer size can't be zero\n");
		return 0;
	}

	/*
	 * calculate the max transfer size supported by src device for a single
	 * sg
	 */
	max_xfer = ADC_DMA_MAX_COUNT * ADC_DMA_WIDTH;

	return (size + max_xfer - 1) / max_xfer;
}

static void sg_fill(u32 size, dma_addr_t digital_volt)
{
	u32 max_xfer;
	struct scatterlist *sg;
	dma_addr_t addr = digital_volt;

	sg = g_drv_data->sg;

	/* Calculate the max transfer size supported by Src Device for a single
	 ** SG */
	max_xfer = ADC_DMA_MAX_COUNT * ADC_DMA_WIDTH;

	while (size) {
		int len = size < max_xfer ? size : max_xfer;
		sg_dma_address(sg) = (dma_addr_t)virt_to_phys(addr);
		sg_set_page(sg, virt_to_page(addr), len,
				offset_in_page(addr));
		addr += len;
		size -= len;
		sg++;
	}
}

static s32 get_sg(u32 size, dma_addr_t digital_volt)
{
	u32 num;

	num = get_sg_count(size);

	g_drv_data->sg = kmalloc(num * sizeof(struct scatterlist), GFP_DMA);
	if (!g_drv_data->sg) {
		dev_err(&g_drv_data->pdev->dev, "scatter list memory alloc fail"
				"\n");
		return -ENOMEM;
	}

	g_drv_data->sg_len = num;

	sg_init_table(g_drv_data->sg, g_drv_data->sg_len);

	sg_fill(size, digital_volt);
	return 0;
}

static void put_sg(void)
{
	/* free scatter list */
	kfree(g_drv_data->sg);

	g_drv_data->sg = NULL;
	g_drv_data->sg_len = 0;
}

/*
 * dma_callback: interrupt handler for dma transfer.
 */
static void dma_callback(void *param)
{
	adc_wakeup();
}

/* function doing dma transfers */
static s32 dma_xfer(u32 len, void *digital_volt)
{
	struct dma_async_tx_descriptor *tx;
	struct dma_chan *chan;
	dma_cap_mask_t mask;
	u32 size = len * sizeof(u32), dma_addr;
	s32 status = 0;
	struct dma_slave_config dma_conf = {
		.src_addr = g_drv_data->rx_reg,
		.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES,
		.src_maxburst = 1,
		.direction = DMA_FROM_DEVICE,
		.device_fc = false,
	};

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);

	chan = dma_request_channel(mask, g_drv_data->data->dma_filter,
			g_drv_data->data->dma_data);
	if (!chan)
		return -EAGAIN;

	dmaengine_slave_config(chan, (void *) &dma_conf);

	g_drv_data->dma_chan = chan;

	status = get_sg(size, (dma_addr_t)digital_volt);
	if (status)
		goto out_dma_channel;

	dma_addr = dma_map_single(&g_drv_data->pdev->dev,
			digital_volt, size, DMA_FROM_DEVICE);

	tx = chan->device->device_prep_slave_sg(chan,
			g_drv_data->sg,
			g_drv_data->sg_len, DMA_FROM_DEVICE,
			DMA_PREP_INTERRUPT, NULL);

	if (!tx) {
		dev_err(&g_drv_data->pdev->dev, "error preparing dma xfer\n");
		status = -EAGAIN;
		goto out_map_single;
	}

	tx->callback = dma_callback;
	tx->callback_param = NULL;

	g_drv_data->cookie = tx->tx_submit(tx);
	if (dma_submit_error(g_drv_data->cookie)) {
		dev_err(&g_drv_data->pdev->dev, "error submitting dma xfer\n");
		status = -EAGAIN;
		goto out_slave_sg;
	}
	chan->device->device_issue_pending(chan);

	/* timeout for ~80 conversions in dma mode is ADC_TIMEOUT */
	status = adc_sleep(ADC_TIMEOUT + (len * g_drv_data->scan_rate));
	if (!status)
		goto out_map_single;

out_slave_sg:
	chan->device->device_control(chan, DMA_TERMINATE_ALL, 0);
	g_drv_data->flag_arrived = 0;

out_map_single:
	dma_unmap_single(chan->device->dev, dma_addr, size,
			DMA_FROM_DEVICE);
	put_sg();

out_dma_channel:
	dma_release_channel(chan);
	g_drv_data->dma_chan = NULL;

	return status;
}
#endif

/**
 * spear_adc_chan_get - allocate adc channel
 * @dev_id: pointer to structure of user or process, to identify channel owner
 * @chan_id: requested channel id
 *
 * This function allocates requested adc channel if available, return 0 on
 * success and negative value on error.
 */
s32 spear_adc_chan_get(void *dev_id, enum adc_chan_id chan_id)
{
	s32 status = 0;
	unsigned long flags;

	if (!dev_id) {
		dev_err(&g_drv_data->pdev->dev, "null devid ptr passed\n");
		return -EFAULT;
	}

	spin_lock_irqsave(&g_drv_data->adc_lock, flags);

	if ((g_drv_data->mode == SINGLE_CONVERSION) &&
			g_drv_data->usage_count) {
		dev_err(&g_drv_data->pdev->dev, "only one user is allowed in "
				"single conversion mode\n");
		status = -EBUSY;
		goto out_lock;
	}

	/* Checking that the requested ADC is Free or Busy */
	if (!g_drv_data->chan[chan_id].owner) {
		if (!g_drv_data->clk_enbld) {
			status = clk_prepare_enable(g_drv_data->clk);
			if (status) {
				dev_err(&g_drv_data->pdev->dev,
						"error enabling clock\n");
				goto out_lock;
			}

			g_drv_data->clk_enbld = true;
		}

		g_drv_data->chan[chan_id].owner = dev_id;
		g_drv_data->usage_count++;
	}

out_lock:
	spin_unlock_irqrestore(&g_drv_data->adc_lock, flags);
	return status;
}
EXPORT_SYMBOL(spear_adc_chan_get);

/**
 * spear_adc_chan_put - release adc channel
 * @dev_id: pointer to structure of user or process, to identify channel owner
 * @chan_id: requested channel id
 *
 * This function releases requested adc channel if requested by owner, return 0
 * on success and negative value on error.
 */
s32 spear_adc_chan_put(void *dev_id, enum adc_chan_id chan_id)
{
	s32 status = 0;
	unsigned long flags;

	if (!dev_id) {
		dev_err(&g_drv_data->pdev->dev, "null devid ptr passed\n");
		return -EFAULT;
	}

	if ((chan_id < ADC_CHANNEL0) || (chan_id > ADC_CHANNEL7)) {
		dev_err(&g_drv_data->pdev->dev, "invalid channel id\n");
		return -ENODEV;
	}

	spin_lock_irqsave(&g_drv_data->adc_lock, flags);

	if (g_drv_data->chan[chan_id].owner == dev_id) {
		if (g_drv_data->mode == CONTINUOUS_CONVERSION)
			adc_writel(g_drv_data->regs, CHAN_CTRL[chan_id], 0);

		g_drv_data->chan[chan_id].owner = NULL;
		g_drv_data->usage_count--;
		g_drv_data->chan[chan_id].configured = false;

		if (g_drv_data->chan[chan_id].scan_rate_fixed) {
			g_drv_data->fixed_sr_count--;
			g_drv_data->chan[chan_id].scan_rate_fixed = 0;
		}
	}
	/* If no more user is using ADC then reset it */
	if (!g_drv_data->usage_count) {
		g_drv_data->scan_rate = ADC_MAX_SCAN_RATE;

		if (g_drv_data->clk_enbld) {
			clk_disable_unprepare(g_drv_data->clk);
			g_drv_data->clk_enbld = false;
		}
	}

	spin_unlock_irqrestore(&g_drv_data->adc_lock, flags);
	return status;
}
EXPORT_SYMBOL(spear_adc_chan_put);

/**
 * adc_get_data - read data from adc
 * @chan_id: channel id
 * @digital_volt: memory to store value
 *
 * This function reads data from specified adc channel.
 */
u32 adc_get_data(enum adc_chan_id chan_id, uint *digital_volt)
{
	u32 data_valid = DATA_VALID;
	u32 val;

#ifndef CONFIG_ARCH_SPEAR6XX
	/* valid bit is always 17th in case of SPEAr300(NORMAL & HR both).
	   this is inconsistent with SPEAr300-um1.2. */
	data_valid = DATA_VALID_HR;
#endif

	if (g_drv_data->mode == CONTINUOUS_CONVERSION) {
#ifndef CONFIG_ARCH_SPEAR6XX
		val = adc_readl(g_drv_data->regs, CHAN_DATA[chan_id]);
#else
		val = adc_readl(g_drv_data->regs, CHAN_DATA[chan_id].MSB);
#endif
		/* check if data is valid or not */
		if (!(val & data_valid))
			return -EAGAIN;
	} else {
#ifndef CONFIG_ARCH_SPEAR6XX
		val = adc_readl(g_drv_data->regs, AVG);
#else
		val = adc_readl(g_drv_data->regs, AVG.MSB);
#endif
	}
	*digital_volt = val;
	return 0;
}

/**
 * spear_adc_get_data - reads converted digital data from adc
 * @dev_id: pointer to structure of user or process, to identify channel owner
 * @chan_id: requested channel id
 * @digital_volt: memory to store value
 * @count: number of values to read. Can be > 1 only for channel 0.
 *
 * This function reads data from specified adc channel.
 */
s32 spear_adc_get_data(void *dev_id, enum adc_chan_id chan_id,
		uint *digital_volt, u32 count)
{
	s32 i, status = 0;
	unsigned long flags;
	u32 data_mask = DATA_MASK;

	if (!dev_id || !digital_volt) {
		dev_err(&g_drv_data->pdev->dev, "null devid or digital_volt ptr"
				"passed\n");
		return -EFAULT;
	}

	spin_lock_irqsave(&g_drv_data->adc_lock, flags);
	if (g_drv_data->chan[chan_id].owner != dev_id) {
		dev_err(&g_drv_data->pdev->dev, "user not owner of channel\n");
		status = -EACCES;
		goto out_lock;
	}

	/* check if adc is configured or not */
	if (!g_drv_data->chan[chan_id].configured) {
		dev_err(&g_drv_data->pdev->dev, "channel not configured\n");
		status = -EINVAL;
		goto out_lock;
	}
	spin_unlock_irqrestore(&g_drv_data->adc_lock, flags);

	/* Enabling DMA for Channel0 */
	if (count > 1) {
#ifdef CONFIG_SPEAR_ADC_DMA_IF
		if ((chan_id == ADC_CHANNEL0) &&
				(g_drv_data->mode == CONTINUOUS_CONVERSION)) {
			uint status = adc_readl(g_drv_data->regs, STATUS);
			adc_writel(g_drv_data->regs, STATUS,
					STOP_CONVERSION & status);
			adc_writel(g_drv_data->regs, STATUS,
					DMA_ENABLE | status);
			msleep(100);
			status = dma_xfer(count, digital_volt);
			adc_writel(g_drv_data->regs, STATUS, DMA_DISABLE &
					adc_readl(g_drv_data->regs, STATUS));

			if (status)
				return -EAGAIN;
		} else {
			dev_err(&g_drv_data->pdev->dev, "len > 0 only for"
					" channel 0\n");
			return -ENODATA;
		}
#else
		dev_err(&g_drv_data->pdev->dev, "len can't be > than 0\n");
		return -EINVAL;
#endif
	} else if (g_drv_data->mode == SINGLE_CONVERSION) {
		status = adc_sleep(ADC_TIMEOUT);
		if (status)
			return status;
		*digital_volt = g_drv_data->digital_volt;
	} else {
		status = adc_get_data(chan_id, digital_volt);
		if (status) {
			dev_dbg(&g_drv_data->pdev->dev, "error reading data\n");
			return status;
		}
	}

#ifndef CONFIG_ARCH_SPEAR6XX
	if (g_drv_data->resolution == HIGH_RESOLUTION)
		data_mask = DATA_MASK_HR;
#endif
	for (i = 0; i < count; i++) {
		*(digital_volt + i) = ((*(digital_volt + i) & data_mask) *
				g_drv_data->mvolt)/DATA_MASK;
#ifndef CONFIG_ARCH_SPEAR6XX
		if (g_drv_data->resolution == HIGH_RESOLUTION)
			*(digital_volt + i) = (*(digital_volt + i)) >>
				g_drv_data->chan[chan_id].avg_samples;
#endif
	}

	return 0;

out_lock:
	spin_unlock_irqrestore(&g_drv_data->adc_lock, flags);
	return status;
}
EXPORT_SYMBOL(spear_adc_get_data);

static irqreturn_t spear_adc_irq(int irq, void *dev_id)
{
	adc_get_data(ADC_CHANNEL_NONE, &g_drv_data->digital_volt);
	adc_wakeup();

	return IRQ_HANDLED;
}

/**
 * adc_configure - configure adc
 * @config: adc configuration
 *
 * This function configures adc after resetting it. This will configure all the
 * configuration common to all adc channels. This should be called only when no
 * conversion is in progress.
 */
u32 adc_configure(struct adc_config *config)
{
	u32 status_reg = 0, ret = 0, clk_high = 0, clk_low = 0, count, clk_min,
	    clk_max;

	if (!of_machine_is_compatible("st,spear1340") &&
			!of_machine_is_compatible("st,spear1310")) {
		clk_min = 2500000;
		clk_max = 20000000;
	} else {
		clk_min = 3000000;
		clk_max = 14000000;
	}

	if ((config->req_clk < clk_min) || (config->req_clk > clk_max))
		return -EINVAL;

	if (!of_machine_is_compatible("st,spear1340") &&
			!of_machine_is_compatible("st,spear1310")) {
		u32 apb_clk = clk_get_rate(g_drv_data->clk);

		count = (apb_clk + config->req_clk - 1) / config->req_clk;
		clk_low = count/2;
		clk_high = count - clk_low;
		config->avail_clk = apb_clk / count;
	} else {
		ret = clk_set_rate(g_drv_data->clk, config->req_clk);
		if (ret)
			return ret;
		config->avail_clk = clk_get_rate(g_drv_data->clk);
	}

	if ((config->avail_clk < clk_min) || (config->avail_clk > clk_max))
		return -EINVAL;

	adc_reset();

	status_reg |= CONV_MODE(config->mode);

	if (config->mode == CONTINUOUS_CONVERSION) {
		status_reg |= SCAN_REF(config->scan_ref);
		status_reg |= START_CONVERSION;
	}

	status_reg |= VOLT_REF(config->volt_ref);
#ifndef CONFIG_ARCH_SPEAR6XX
	if (of_machine_is_compatible("st,spear1340") ||
			of_machine_is_compatible("st,spear1310"))
		status_reg |= SPEAR1340_RESOLUTION(config->resolution);
	else
		status_reg |= RESOLUTION(config->resolution);
#endif
	status_reg |= ADC_ENABLE;

	adc_writel(g_drv_data->regs, STATUS, status_reg);
	if (!of_machine_is_compatible("st,spear1340") &&
			!of_machine_is_compatible("st,spear1310"))
		adc_writel(g_drv_data->regs, CLK, CLK_LOW(clk_low) |
				CLK_HIGH(clk_high));
	return 0;
}

/**
 * spear_adc_configure - configure adc device
 * @dev_id: pointer to structure of user or process, to identify channel owner
 * @chan_id: requested channel id
 * @config: adc configuration structure
 *
 * This function configures adc device if all the adc channels are free. This is
 * called from the probe function with configurations from platform data.
 * It return 0 on success and negative value on error.
 */
s32 spear_adc_configure(void *dev_id, enum adc_chan_id chan_id,
		struct adc_config *config)
{
	s32 status = 0, irq;
	unsigned long flags;

	if (!dev_id || !config) {
		dev_err(&g_drv_data->pdev->dev, "null devid or config ptr "
				"passed\n");
		return -EFAULT;
	}

	if (config->mode >= CONVERSION_NONE) {
		dev_err(&g_drv_data->pdev->dev, "invalid mode\n");
		return -EINVAL;
	}

	if (!config->req_clk) {
		dev_err(&g_drv_data->pdev->dev, "clock can't be zero\n");
		return -EINVAL;
	}

	if ((chan_id < ADC_CHANNEL0) || (chan_id > ADC_CHANNEL7)) {
		dev_err(&g_drv_data->pdev->dev, "invalid channel id\n");
		return -ENODEV;
	}

	spin_lock_irqsave(&g_drv_data->adc_lock, flags);

	if (g_drv_data->chan[chan_id].owner != dev_id) {
		dev_err(&g_drv_data->pdev->dev, "user not owner of channel\n");
		status = -EACCES;
		goto out_lock;
	}

	/* adc can't be configured if in use */
	if ((g_drv_data->configured) && (g_drv_data->usage_count > 1)) {
		dev_err(&g_drv_data->pdev->dev, "more than one users using adc."
				" can't configure adc.\n");
		status = -EBUSY;
		goto out_lock;
	}

	status = adc_configure(config);
	if (!status) {
		enum adc_conv_mode old_mode = g_drv_data->mode;
		g_drv_data->mode = config->mode;
		g_drv_data->configured = true;
		g_drv_data->mvolt = config->mvolt;
#ifndef CONFIG_ARCH_SPEAR6XX
		g_drv_data->resolution = config->resolution;
#endif
		irq = platform_get_irq(g_drv_data->pdev, 0);
		if (irq < 0) {
			dev_err(&g_drv_data->pdev->dev, "irq resource "
					"not defined\n");
			status = -ENODEV;
			goto out_lock;
		}

		if ((config->mode == CONTINUOUS_CONVERSION) &&
				(old_mode == SINGLE_CONVERSION))
			free_irq(irq, g_drv_data);

		spin_unlock_irqrestore(&g_drv_data->adc_lock, flags);

		if ((config->mode == SINGLE_CONVERSION) &&
				(old_mode != SINGLE_CONVERSION)) {
			status = request_irq(irq, spear_adc_irq, 0, "spear-adc",
					g_drv_data);
			if (status < 0) {
				dev_err(&g_drv_data->pdev->dev, "request irq"
						" error\n");
				return status;
			}
		}
		return 0;
	}

out_lock:
	spin_unlock_irqrestore(&g_drv_data->adc_lock, flags);
	return status;
}
EXPORT_SYMBOL(spear_adc_configure);

/**
 * spear_adc_get_configure - get adc device configuration
 * @dev_id: pointer to structure of user or process, to identify channel owner
 * @chan_id: requested channel id
 * @config: adc configuration structure
 *
 * This function gives current adc configuration. It return 0 on success and
 * negative value on error.
 */
s32 spear_adc_get_configure(void *dev_id, enum adc_chan_id chan_id,
		struct adc_config *config)
{
	s32 status = 0;
	unsigned long flags;

	if (!dev_id || !config) {
		dev_err(&g_drv_data->pdev->dev, "null devid or config ptr "
				"passed\n");
		return -EFAULT;
	}

	if ((chan_id < ADC_CHANNEL0) || (chan_id > ADC_CHANNEL7)) {
		dev_err(&g_drv_data->pdev->dev, "invalid channel id\n");
		return -ENODEV;
	}

	spin_lock_irqsave(&g_drv_data->adc_lock, flags);

	if (g_drv_data->chan[chan_id].owner != dev_id) {
		dev_err(&g_drv_data->pdev->dev, "user not owner of channel\n");
		status = -EACCES;
		goto out_lock;
	}

	if (g_drv_data->configured) {
		u32 reg = adc_readl(g_drv_data->regs, STATUS);

		config->mode = g_drv_data->mode;
		config->volt_ref = (reg & VOLT_REF(INTERNAL_VOLT)) ?
			INTERNAL_VOLT : EXTERNAL_VOLT;
		config->scan_ref = (reg & SCAN_REF(EXTERNAL_SCAN)) ?
			EXTERNAL_SCAN : INTERNAL_SCAN;
#ifndef CONFIG_ARCH_SPEAR6XX
		config->resolution = g_drv_data->resolution;
#endif
		config->mvolt = g_drv_data->mvolt;

		if (!of_machine_is_compatible("st,spear1340") &&
				!of_machine_is_compatible("st,spear1310")) {
			reg = adc_readl(g_drv_data->regs, CLK);
			config->req_clk = clk_get_rate(g_drv_data->clk) /
				(GET_CLK_HIGH(reg) + GET_CLK_LOW(reg));
			config->avail_clk = config->req_clk;
		} else {
			config->avail_clk = config->req_clk =
				clk_get_rate(g_drv_data->clk);
		}
	} else {
		dev_err(&g_drv_data->pdev->dev, "adc not configured\n");
		status = -EPERM;
	}

out_lock:
	spin_unlock_irqrestore(&g_drv_data->adc_lock, flags);
	return status;
}
EXPORT_SYMBOL(spear_adc_get_configure);

/**
 * adc_chan_configure - configure adc channel
 * @config: adc channel configuration
 *
 * This function configures a adc channel and then starts conversion on the
 * channel.
 */
u32 adc_chan_configure(struct adc_chan_config *config)
{
	u32 num_apb_clk = 0;

	/* read the configured mode of adc */
	if (adc_readl(g_drv_data->regs, STATUS) &
			CONV_MODE(CONTINUOUS_CONVERSION)) {
		/* continuous mode */
		num_apb_clk = (clk_get_rate(g_drv_data->clk) / 1000000) *
			config->scan_rate;

		if (num_apb_clk <= ADC_MAX_SCAN_RATE) {
#ifndef CONFIG_ARCH_SPEAR6XX
			adc_writel(g_drv_data->regs, SCAN_RATE_REG,
					SCAN_RATE(num_apb_clk));
#else
			adc_writel(g_drv_data->regs, SCAN_RATE_LO_REG,
					SCAN_RATE_LO(num_apb_clk));
			adc_writel(g_drv_data->regs, SCAN_RATE_HI_REG,
					SCAN_RATE_HI(num_apb_clk));
#endif
		} else
			return -EINVAL;

		adc_writel(g_drv_data->regs, CHAN_CTRL[config->chan_id], 0);
		adc_writel(g_drv_data->regs, CHAN_CTRL[config->chan_id],
				CHAN_AVG_SAMPLE(config->avg_samples) |
				START_CONVERSION);
	} else {
		/* single mode */
		adc_writel(g_drv_data->regs, STATUS, STOP_CONVERSION &
				adc_readl(g_drv_data->regs, STATUS));
		adc_writel(g_drv_data->regs, STATUS, (AVG_SAMPLE_RESET &
					adc_readl(g_drv_data->regs, STATUS)) |
				AVG_SAMPLE(config->avg_samples) |
				CHANNEL_NUM(config->chan_id) |
				START_CONVERSION);
	}
	return 0;
}

/**
 * spear_adc_chan_configure - configure adc channel
 * @dev_id: pointer to structure of user or process, to identify channel owner
 * @config: adc channel configuration structure
 *
 * This function configures adc channel if adc is already configured.
 * If user don't want scan rate to be configured to any other value till the
 * lifetime of the requesting user, then it will pass scan_rate_fixed as true.
 * If scan rate is already fixed then user can't change the scan rate of ADC.
 * If scan rate is not already fixed and scan rate requested is higher than
 * already configured scan rate, then error is returned.
 * Otherwise ADC is configured to the requested scan rate.
 * It return 0 on success and negative value on error.
 */
s32 spear_adc_chan_configure(void *dev_id, struct adc_chan_config *config)
{
	s32 status = 0;
	unsigned long flags;

	if (!dev_id || !config) {
		dev_err(&g_drv_data->pdev->dev, "null devid or config ptr "
				"passed\n");
		return -EFAULT;
	}

	spin_lock_irqsave(&g_drv_data->adc_lock, flags);
	if (g_drv_data->chan[config->chan_id].owner != dev_id) {
		dev_err(&g_drv_data->pdev->dev, "user not owner of channel\n");
		status = -EACCES;
		goto out_lock;
	}

	if (!g_drv_data->configured) {
		dev_err(&g_drv_data->pdev->dev, "adc not configured\n");
		status = -EPERM;
		goto out_lock;
	}

	/* checking if more than one user are present and checking if current
	 ** user has already requested for fixed scan rate and fixed scan rate
	 ** is requested by only one user */
	if ((g_drv_data->usage_count > 1) &&
			!(g_drv_data->chan[config->chan_id].scan_rate_fixed &&
				(g_drv_data->fixed_sr_count == 1))) {
		/* checking if there are more than one user who have requested
		 ** for fixed scan rates and the current requested scan rate is
		 ** not equal to already configured scan rate */
		if (g_drv_data->fixed_sr_count) {
			if (config->scan_rate != g_drv_data->scan_rate) {
				dev_err(&g_drv_data->pdev->dev, "ADC configured"
						"for fixed scan rate, can't "
						"reconfigure scan rate\n");
				status = -EBUSY;
				goto out_lock;
			}
		} else if (config->scan_rate > g_drv_data->scan_rate) {
			/* if scan rate requested is greater than already
			 ** configured scan rate */
			dev_err(&g_drv_data->pdev->dev, "ADC can't be "
					"configured with a scan rate greater "
					"than already configured scan rate\n");
			status = -EBUSY;
			goto out_lock;
		}
	}

	status = adc_chan_configure(config);
	if (status)
		goto out_lock;

	/* if user has not requested for fixed scan rate previously */
	if (!g_drv_data->chan[config->chan_id].scan_rate_fixed &&
			config->scan_rate_fixed)
		g_drv_data->fixed_sr_count++;

	/* If user has requested for fixed scan rate previously and now
	 ** requested for a flexible scan rate */
	if (g_drv_data->chan[config->chan_id].scan_rate_fixed &&
			!config->scan_rate_fixed)
		g_drv_data->fixed_sr_count--;

	g_drv_data->chan[config->chan_id].scan_rate_fixed =
		config->scan_rate_fixed;

	g_drv_data->chan[config->chan_id].configured = true;
	g_drv_data->scan_rate = config->scan_rate;
	g_drv_data->chan[config->chan_id].avg_samples = config->avg_samples;

out_lock:
	spin_unlock_irqrestore(&g_drv_data->adc_lock, flags);
	return status;
}
EXPORT_SYMBOL(spear_adc_chan_configure);

/**
 * spear_adc_get_chan_configure - get adc channel configuration
 * @dev_id: pointer to structure of user or process, to identify channel owner
 * @config: adc configuration structure
 *
 * This function gives current adc configuration. It return 0 on success and
 * negative value on error.
 */
s32 spear_adc_get_chan_configure(void *dev_id, struct adc_chan_config *config)
{
	s32 status = 0;
	unsigned long flags;

	if (!dev_id || !config) {
		dev_err(&g_drv_data->pdev->dev, "null devid or config ptr "
				"passed\n");
		return -EFAULT;
	}

	if ((config->chan_id < ADC_CHANNEL0) ||
			(config->chan_id > ADC_CHANNEL7)) {
		dev_err(&g_drv_data->pdev->dev, "invalid channel id\n");
		return -ENODEV;
	}

	spin_lock_irqsave(&g_drv_data->adc_lock, flags);

	if (g_drv_data->chan[config->chan_id].owner != dev_id) {
		dev_err(&g_drv_data->pdev->dev, "user not owner of channel\n");
		status = -EACCES;
		goto out_lock;
	}

	if (g_drv_data->chan[config->chan_id].configured) {
#ifndef CONFIG_ARCH_SPEAR6XX
		u32 num_apb_clk;
#endif
		uint val;
		if (adc_readl(g_drv_data->regs, STATUS) &
				CONV_MODE(CONTINUOUS_CONVERSION)) {
			val = adc_readl(g_drv_data->regs,
					CHAN_CTRL[config->chan_id]);
			config->avg_samples = CHAN_GET_AVG_SAMPLE(val);
		} else {
			val = adc_readl(g_drv_data->regs, STATUS);
			config->avg_samples = GET_AVG_SAMPLE(val);
		}
#ifndef CONFIG_ARCH_SPEAR6XX
		num_apb_clk = adc_readl(g_drv_data->regs, SCAN_RATE_REG);
#else
		val = adc_readl(g_drv_data->regs, SCAN_RATE_HI_REG) << 16;
		val = adc_readl(g_drv_data->regs, SCAN_RATE_LO_REG) | val;
#endif
		config->scan_rate = val/(clk_get_rate(g_drv_data->clk)/1000000);
		config->scan_rate_fixed =
			g_drv_data->chan[config->chan_id].scan_rate_fixed;
		config->avg_samples =
			g_drv_data->chan[config->chan_id].avg_samples;
	} else {
		dev_err(&g_drv_data->pdev->dev, "adc channel not configured\n");
		status = -EPERM;
	}

out_lock:
	spin_unlock_irqrestore(&g_drv_data->adc_lock, flags);
	return status;
}
EXPORT_SYMBOL(spear_adc_get_chan_configure);

static int spear_adc_open(struct inode *inode, struct file *file)
{
	s32 status = 0;
	enum adc_chan_id chan_id;

	chan_id = iminor(inode);

	/* Acquiring a ADC Device. */
	status = spear_adc_chan_get(current, chan_id);

	/* Check if ADC channel is acquired or not.. */
	if (!status)
		file->private_data = &g_drv_data->chan[chan_id].id;

	return status;
}

static int spear_adc_release(struct inode *inode, struct file *file)
{
	s32 status = 0;
	enum adc_chan_id chan_id;

	chan_id = iminor(inode);

	status = spear_adc_chan_put(current, chan_id);

	return status;
}

static ssize_t
spear_adc_read(struct file *file, char __user *buf, size_t len, loff_t *ppos)
{
	enum adc_chan_id chan_id;
	s32 status = 0;
	u32 size;
	u32 *digital_volt;
	chan_id = *((enum adc_chan_id *)(file->private_data));

	if (len <= 0) {
		dev_err(&g_drv_data->pdev->dev, "length can't be zero\n");
		return -EINVAL;
	}

	size = sizeof(*digital_volt) * len;
	digital_volt = kzalloc(size, GFP_DMA);

	status = spear_adc_get_data(current, chan_id, digital_volt, len);
	if (!status) {
		status = copy_to_user(buf, digital_volt, size);
		status = len - status;
	} else
	kfree(digital_volt);

	return status;
}

static long spear_adc_ioctl(struct file *file, u32 cmd, unsigned long arg)
{
	enum adc_chan_id chan_id;
	s32 status = 0;

	chan_id = *((enum adc_chan_id *)(file->private_data));

	switch (cmd) {
	case ADCIOC_CONFIG:
		{
			struct adc_config config;
			status = copy_from_user(&config,
					(struct adc_config *)arg,
					sizeof(config));
			if (status) {
				dev_err(&g_drv_data->pdev->dev, "error copying"
						"data from user space\n");
				return -EAGAIN;
			}
			status = spear_adc_configure(current, chan_id, &config);
			if (!status) {
				u32 *buf =
					&((struct adc_config *)arg)->avail_clk;
				if (!access_ok(VERIFY_WRITE, (void __user *)buf,
							sizeof(*buf)))
					return -EFAULT;
				status = put_user(config.avail_clk, buf);
				if (status) {
					dev_err(&g_drv_data->pdev->dev, "error "
							"copying avail freq\n");
					return -EAGAIN;
				}
			}
		}
		break;
	case ADCIOC_CHAN_CONFIG:
		{
			struct adc_chan_config config;
			status = copy_from_user(&config,
					(struct adc_chan_config *)arg,
					sizeof(config));
			if (status) {
				dev_err(&g_drv_data->pdev->dev, "error copying"
						"data from user space\n");
				return -EAGAIN;
			}

			config.chan_id = chan_id;
			status = spear_adc_chan_configure(current, &config);
		}
		break;
	case ADCIOC_GET_CONFIG:
		{
			struct adc_config config;
			status = spear_adc_get_configure(current, chan_id,
					&config);
			if (!status) {
				status = copy_to_user((struct adc_config *)arg,
						&config, sizeof(config));
				if (status) {
					dev_err(&g_drv_data->pdev->dev, "error"
							"copying config to user"
							" space %d\n", status);
				}
			} else {
				dev_err(&g_drv_data->pdev->dev, "error getting "
						"adc configuration\n");
			}
		}
		break;
	case ADCIOC_GET_CHAN_CONFIG:
		{
			struct adc_chan_config config;
			config.chan_id = chan_id;
			status = spear_adc_get_chan_configure(current, &config);
			if (!status) {
				status = copy_to_user((struct adc_config *)arg,
						&config, sizeof(config));
				if (status) {
					dev_err(&g_drv_data->pdev->dev, "error "
							"copying config to user"
							" space\n");
				}
			} else {
				dev_err(&g_drv_data->pdev->dev, "error getting "
						"adc channel configuration\n");
			}
		}
		break;
	default:
		return -EINVAL;
		break;
	}

	return status;
}

static const struct file_operations spear_adc_fops = {
	.owner	= THIS_MODULE,
	.open = spear_adc_open,
	.release = spear_adc_release,
	.read	= spear_adc_read,
	.unlocked_ioctl = spear_adc_ioctl
};

int default_configure(void)
{
	int status, ret;

	status = spear_adc_chan_get(g_drv_data, ADC_CHANNEL0);
	if (status) {
		dev_err(&g_drv_data->pdev->dev, "chan get failed\n");
		return status;
	}

	status = spear_adc_configure(g_drv_data, ADC_CHANNEL0,
			&g_drv_data->data->config);
	ret = spear_adc_chan_put(g_drv_data, ADC_CHANNEL0);

	if (status) {
		dev_err(&g_drv_data->pdev->dev, "default configure failed\n");
		return status;
	}

	if (ret)
		dev_err(&g_drv_data->pdev->dev, "chan put failed\n");

	return ret;
}

static s32 spear_adc_probe(struct platform_device *pdev)
{
	s32 status = 0, i = 0;
	struct adc_plat_data *plat_data = pdev->dev.platform_data;
	struct resource	*io;

	if (!plat_data) {
		dev_err(&pdev->dev, "Invalid platform data\n");
		return -EINVAL;
	}

	io = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!io) {
		dev_err(&pdev->dev, "error getting platform resource\n");
		return -EINVAL;
	}

	if (!request_mem_region(io->start, io->end - io->start + 1,
				pdev->dev.driver->name)) {
		dev_err(&pdev->dev, "error requesting mem region\n");
		return -EBUSY;
	}

	g_drv_data = kzalloc(sizeof(*g_drv_data), GFP_KERNEL);
	if (!g_drv_data) {
		dev_err(&pdev->dev, "error allocating memory\n");
		status = -ENOMEM;
		goto err_mem_region;
	}

	g_drv_data->pdev = pdev;
	g_drv_data->data = plat_data;

	g_drv_data->regs = ioremap(io->start, io->end - io->start + 1);
	if (!g_drv_data->regs) {
		dev_err(&pdev->dev, "error in ioremap\n");
		status = -ENOMEM;
		goto err_kzalloc;
	}

	g_drv_data->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(g_drv_data->clk)) {
		dev_err(&pdev->dev, "error in clk get\n");
		status = PTR_ERR(g_drv_data->clk);
		goto err_ioremap;
	}

	g_drv_data->mode = CONVERSION_NONE;
	g_drv_data->scan_rate = ADC_MAX_SCAN_RATE;

	/* register char driver */
	status = alloc_chrdev_region(&g_drv_data->chan[0].dev_num, 0,
			ADC_CHANNEL_NUM, DRIVER_NAME);
	if (status) {
		dev_err(&pdev->dev, "unable to register adc driver\n");
		goto err_clock_enabled;
	}

	for (i = 0; i < ADC_CHANNEL_NUM; i++) {
		g_drv_data->chan[i].id = i;
		g_drv_data->chan[i].dev_num =
			MKDEV(MAJOR(g_drv_data->chan[0].dev_num), i);
#ifdef CONFIG_PM
		g_drv_data->chan[i].adc_saved_chan_config.chan_id = i;
#endif
	}

	for (i = 0; i < ADC_CHANNEL_NUM; i++) {
		g_drv_data->chan[i].cdev.owner = THIS_MODULE;
		cdev_init(&(g_drv_data->chan[i].cdev),
				&spear_adc_fops);
		status = cdev_add(&(g_drv_data->chan[i].cdev),
				g_drv_data->chan[i].dev_num, 1);
		if (status) {
			dev_err(&pdev->dev, "error registering adc devices with"
					"char framework\n");
			goto out_chrdev_add;
		}
	}

	/* Initialize wait queue heads. */
	init_waitqueue_head(&g_drv_data->wait_queue);
	spin_lock_init(&g_drv_data->adc_lock);

	platform_set_drvdata(pdev, g_drv_data);

#ifdef CONFIG_SPEAR_ADC_DMA_IF
#ifndef CONFIG_ARCH_SPEAR6XX
	g_drv_data->rx_reg = (dma_addr_t)&((struct adc_regs
				*)io->start)->CHAN_DATA[ADC_CHANNEL0];
#else
	g_drv_data->rx_reg = (dma_addr_t)&((struct adc_regs
				*)io->start)->CHAN_DATA[ADC_CHANNEL0].MSB;
#endif
#endif

	status = default_configure();
	if (status)
		goto out_chrdev_add;

	dev_info(&pdev->dev, "registeration successful\n");
	return 0;

out_chrdev_add:
	/*unregister char driver*/
	for (i = 0; i < ADC_CHANNEL_NUM; i++)
		cdev_del(&g_drv_data->chan[i].cdev);

	unregister_chrdev_region(g_drv_data->chan[0].dev_num, ADC_CHANNEL_NUM);
err_clock_enabled:
	clk_put(g_drv_data->clk);

err_ioremap:
	iounmap(g_drv_data->regs);

err_kzalloc:
	kfree(g_drv_data);

err_mem_region:
	release_mem_region(io->start, io->end - io->start + 1);

	return status;
}

static s32 spear_adc_remove(struct platform_device *pdev)
{
	s32 irq, i = 0;
	struct resource	*io;

	adc_reset();

#ifdef CONFIG_SPEAR_ADC_DMA_IF
	if (g_drv_data->dma_chan)
		dma_release_channel(g_drv_data->dma_chan);
	g_drv_data->dma_chan = NULL;
#endif

	/* Release IRQ */
	if (g_drv_data->mode == SINGLE_CONVERSION) {
		irq = platform_get_irq(pdev, 0);
		if (irq >= 0)
			free_irq(irq, g_drv_data);
	}

	/* unregister char driver */
	for (i = 0; i < ADC_CHANNEL_NUM; i++)
		cdev_del(&g_drv_data->chan[i].cdev);

	unregister_chrdev_region(g_drv_data->chan[0].dev_num, ADC_CHANNEL_NUM);

	if (g_drv_data->clk_enbld) {
		clk_disable_unprepare(g_drv_data->clk);
		g_drv_data->clk_enbld = false;
	}

	clk_put(g_drv_data->clk);

	iounmap(g_drv_data->regs);

	kfree(g_drv_data);

	io = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(io->start, io->end - io->start + 1);

	/* Prevent double remove */
	platform_set_drvdata(pdev, NULL);

	return 0;
}

#ifdef CONFIG_PM
static s32 adc_suspend(struct device *dev, bool csave)
{
	int i, ret = 0;
	bool got_config = false;

	if (!g_drv_data->usage_count)
		return 0;

	if (!csave)
		goto sus_clk_dis;

	for (i = 0; i < ADC_CHANNEL_NUM && g_drv_data->chan[i].configured;
			i++) {
		if (got_config == false) {
			ret = spear_adc_get_configure(g_drv_data->chan[i].owner,
					i, &g_drv_data->adc_saved_config);
			if (ret) {
				dev_err(dev, "Suspend: get configure failed\n");
				break;
			}
			got_config = true;
		}

		ret = spear_adc_get_chan_configure(g_drv_data->chan[i].owner,
				&g_drv_data->chan[i].adc_saved_chan_config);
		if (ret) {
			dev_err(dev, "Suspend: get chan configure failed\n");
			break;
		}
	}

sus_clk_dis:
	clk_disable_unprepare(g_drv_data->clk);

	return ret;
}

static s32 spear_adc_suspend(struct device *dev)
{
	return adc_suspend(dev, true);
}

static s32 spear_adc_poweroff(struct device *dev)
{
	return adc_suspend(dev, false);
}

static s32 adc_resume(struct device *dev, bool csave)
{
	int i, ret;
	bool configured = false;

	if (!g_drv_data->usage_count) {
		ret = default_configure();
		return ret;
	}

	ret = clk_prepare_enable(g_drv_data->clk);
	if (ret) {
		dev_err(dev, "Resume: clk enable failed\n");
		return ret;
	}

	if (!csave)
		return 0;

	for (i = 0; i < ADC_CHANNEL_NUM && g_drv_data->chan[i].configured;
			i++) {
		if (configured == false) {
			ret = spear_adc_configure(g_drv_data->chan[i].owner,
					i, &g_drv_data->adc_saved_config);
			if (ret) {
				dev_err(dev, "Resume: configure failed\n");
				goto disable_clk;
			}
			configured = true;
		}

		ret = spear_adc_chan_configure(g_drv_data->chan[i].owner,
				&g_drv_data->chan[i].adc_saved_chan_config);
		if (ret) {
			dev_err(dev, "Resume: chan configure failed\n");
			goto disable_clk;
		}
	}

	return 0;

disable_clk:
	clk_disable_unprepare(g_drv_data->clk);

	return ret;
}

static s32 spear_adc_resume(struct device *dev)
{
	return adc_resume(dev, true);
}

static s32 spear_adc_thaw(struct device *dev)
{
	return adc_resume(dev, false);
}

static const struct dev_pm_ops spear_adc_dev_pm_ops = {
	.suspend = spear_adc_suspend,
	.resume = spear_adc_resume,
	.freeze = spear_adc_suspend,
	.thaw = spear_adc_thaw,
	.poweroff = spear_adc_poweroff,
	.restore = spear_adc_resume,
};

#define SPEAR_ADC_DEV_PM_OPS (&spear_adc_dev_pm_ops)
#else
#define SPEAR_ADC_DEV_PM_OPS NULL
#endif /* CONFIG_PM */

#ifdef CONFIG_OF
static const struct of_device_id spear_adc_of_match[] = {
	{ .compatible = "st,spear-adc" },
	{}
};
MODULE_DEVICE_TABLE(of, spear_adc_of_match);
#endif

static struct platform_driver spear_adc_driver = {
	.driver = {
		.name = "adc",
		.bus = &platform_bus_type,
		.owner = THIS_MODULE,
		.pm = SPEAR_ADC_DEV_PM_OPS,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(spear_adc_of_match),
#endif
	},
	.probe = spear_adc_probe,
	.remove = __devexit_p(spear_adc_remove),
};

static s32 __init spear_adc_init(void)
{
	platform_driver_register(&spear_adc_driver);

	return 0;
}
module_init(spear_adc_init);

static void __exit spear_adc_exit(void)
{
	platform_driver_unregister(&spear_adc_driver);
}
module_exit(spear_adc_exit);

MODULE_AUTHOR("Viresh Kumar");
MODULE_DESCRIPTION("SPEAR ADC Contoller");
MODULE_LICENSE("GPL");
