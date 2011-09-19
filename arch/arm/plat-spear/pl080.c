/*
 * arch/arm/plat-spear/pl080.c
 *
 * DMAC pl080 definitions for SPEAr platform
 *
 * Copyright (C) 2011 ST Microelectronics
 * Viresh Kumar <viresh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/amba/pl08x.h>
#include <linux/amba/bus.h>
#include <linux/bug.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <mach/misc_regs.h>

static spinlock_t lock = SPIN_LOCK_UNLOCKED;

struct {
	unsigned char busy;
	unsigned char val;
} signals[16] = {{0, 0}, };

int pl080_get_signal(struct pl08x_dma_chan *ch)
{
	const struct pl08x_channel_data *cd = ch->cd;
	unsigned int signal = cd->min_signal, val;
	unsigned long flags;

	spin_lock_irqsave(&lock, flags);

	/* Return if signal is already acquired by somebody else */
	if (signals[signal].busy &&
			(signals[signal].val != cd->muxval)) {
		spin_unlock_irqrestore(&lock, flags);
		return -EBUSY;
	}

	/* If acquiring for the first time, configure it */
	if (!signals[signal].busy) {
		val = readl(VA_DMA_CHN_CFG);

		/*
		 * Each request line has two bits in DMA_CHN_CFG register. To
		 * goto the bits of current request line, do left shift of
		 * value by 2 * signal number.
		 */
		val &= ~(0x3 << (signal * 2));
		val |= cd->muxval << (signal * 2);
		writel(val, VA_DMA_CHN_CFG);
	}

	signals[signal].busy++;
	signals[signal].val = cd->muxval;
	spin_unlock_irqrestore(&lock, flags);

	return signal;
}

void pl080_put_signal(struct pl08x_dma_chan *ch)
{
	const struct pl08x_channel_data *cd = ch->cd;
	unsigned long flags;

	spin_lock_irqsave(&lock, flags);

	/* if signal is not used */
	if (!signals[cd->min_signal].busy)
		BUG();

	signals[cd->min_signal].busy--;

	spin_unlock_irqrestore(&lock, flags);
}

void pl080_set_slaveinfo(struct amba_device *pl080,
		struct pl08x_channel_data *cd, u32 num)
{
	struct pl08x_platform_data *pdata = dev_get_platdata(&pl080->dev);

	pdata->slave_channels = cd;
	pdata->num_slave_channels = num;
	pdata->get_signal = pl080_get_signal;
	pdata->put_signal = pl080_put_signal;
}
