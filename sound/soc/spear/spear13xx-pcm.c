/*
 * ALSA PCM interface for ST spear Processor
 *
 * sound/soc/spear/spear13xx-pcm.c
 *
 * Copyright (C) 2010 ST Microelectronics
 * Rajeev Kumar<rajeev-dlh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <mach/dma.h>
#include <asm/dma.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include "spear13xx-i2s.h"
#include "spear13xx-pcm.h"

static u64 spear13xx_pcm_dmamask = 0xffffffff;
static s32 dma_xfer(struct snd_pcm_substream *substream);

struct pcm_dma_data data;

struct snd_pcm_hardware spear13xx_pcm_hardware = {
	.info = SNDRV_PCM_INFO_NONINTERLEAVED ,
	.formats = SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_S16_LE |
		SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE,
	.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
		SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100,
	.rate_min = 48000,
	.rate_max = 8000,
	.channels_min = MIN_CHANNEL_NUM,
	.channels_max = MAX_CHANNEL_NUM,
	.buffer_bytes_max = 16 * 1024, /* max buffer size */
	.period_bytes_min = 2 * 1024, /* 1 msec data minimum period size */
	.period_bytes_max = 2 * 1024, /* maximum period size */
	.periods_min = 1, /* min # periods */
	.periods_max = 8, /* max # of periods */
	.fifo_size = 0, /* fifo size in bytes */
};

void pcm_init(struct device *dma_dev)
{

	data.mem2i2s_slave.dma_dev = dma_dev;
	data.i2s2mem_slave.dma_dev = dma_dev;
	data.mem2i2s_slave.reg_width = DW_DMA_SLAVE_WIDTH_16BIT;
	data.mem2i2s_slave.cfg_hi = DWC_CFGH_DST_PER(DMA_REQ_I2S_TX);
	data.mem2i2s_slave.cfg_lo = 0;
	data.mem2i2s_slave.sms = DW_DMA_MASTER1;
	data.mem2i2s_slave.dms = DW_DMA_MASTER1;
	data.mem2i2s_slave.smsize = DW_DMA_MSIZE_16;
	data.mem2i2s_slave.dmsize = DW_DMA_MSIZE_16;
	data.mem2i2s_slave.fc = DW_DMA_FC_D_M2P;

	data.i2s2mem_slave.reg_width = DW_DMA_SLAVE_WIDTH_16BIT;
	data.i2s2mem_slave.cfg_hi = DWC_CFGH_SRC_PER(DMA_REQ_I2S_RX);
	data.i2s2mem_slave.sms = DW_DMA_MASTER1;
	data.i2s2mem_slave.dms = DW_DMA_MASTER1;
	data.i2s2mem_slave.smsize = DW_DMA_MSIZE_16;
	data.i2s2mem_slave.dmsize = DW_DMA_MSIZE_16;
	data.i2s2mem_slave.fc = DW_DMA_FC_D_P2M;
	data.i2s2mem_slave.cfg_lo = 0;

}

/* functions manipulating scatter lists */
void sg_fill(struct snd_pcm_substream *substream, int data_type)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct spear13xx_runtime_data *prtd = runtime->private_data;
	u32 period_size, dma_offset;
	dma_addr_t addr;
	struct scatterlist *sg;

	period_size = snd_pcm_lib_period_bytes(substream);
	dma_offset = prtd->period * period_size;
	addr = (dma_addr_t)runtime->dma_area + dma_offset;
	sg = &prtd->sg[data_type];
	sg_dma_address(sg) = addr;
	sg_set_page(sg, pfn_to_page((runtime->dma_addr + dma_offset) >>
				PAGE_SHIFT), period_size, offset_in_page(addr));
}

static s32 get_sg(struct snd_pcm_substream *substream)
{
	struct spear13xx_runtime_data *prtd = substream->runtime->private_data;
	int data_type;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		data_type = 0;
	else
		data_type = 1;

	prtd->sg_len[data_type] = 1;
	sg_init_table(&prtd->sg[data_type], 1);

	sg_fill(substream, data_type);

	return 0;
}

static void pcm_dma_callback(void *param)
{
	struct snd_pcm_substream *substream = param;
	struct spear13xx_runtime_data *prtd = substream->runtime->private_data;
	struct dma_chan *chan;
	int data_type;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		data_type = 0;
	else
		data_type = 1;

	chan = prtd->dma_chan[data_type];
	prtd->sg_len[data_type] = 0;
	if (snd_pcm_running(substream)) {
		snd_pcm_period_elapsed(substream);
		tasklet_schedule(&prtd->tasklet);
	}

}

/* xfer data between i2s controller and memory */
static s32 dma_xfer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct spear13xx_runtime_data *prtd = runtime->private_data;
	u32 ret;
	struct dma_chan *chan;
	struct dma_async_tx_descriptor *tx;
	enum dma_data_direction direction;
	dma_async_tx_callback callback;
	int data_type;
	u32 period_size;
	static int count;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		direction = DMA_TO_DEVICE;
		data_type = 0; /*play */
	} else {
		direction = DMA_FROM_DEVICE;
		data_type = 1; /*capture*/
	}

	chan = prtd->dma_chan[data_type];

	if (count > 0) {
		u32 status = DMA_SUCCESS;
		count++;
		status = chan->device->device_is_tx_complete(chan,
				prtd->cookie[data_type], NULL, NULL);

		if (unlikely(status != DMA_SUCCESS))
			return status;
	}
	period_size = snd_pcm_lib_period_bytes(substream);
	ret = get_sg(substream);
	if (ret)
		return ret;

	callback = pcm_dma_callback;
	tx = chan->device->device_prep_slave_sg(chan, &prtd->sg[data_type],
			prtd->sg_len[data_type], direction, DMA_PREP_INTERRUPT);

	if (!tx) {
		dev_err(&prtd->dev, "error in slave sg\n");
		return -ENOMEM;
	}

	tx->callback = callback;
	tx->callback_param = substream;
	prtd->cookie[data_type] = tx->tx_submit(tx);

	if (dma_submit_error(prtd->cookie[data_type])) {
		dev_err(&prtd->dev, "submit error %d\n",
				prtd->cookie[data_type]);
		return -EAGAIN;
	}
	chan->device->device_issue_pending(chan);

	spin_lock(&prtd->lock);
	prtd->dma_addr = substream->runtime->dma_addr +
		prtd->period * period_size;
	spin_unlock(&prtd->lock);

	prtd->period++;
	if (unlikely(prtd->period >= runtime->periods))
		prtd->period = 0;

	return 0;
}
static void pcm_dma_tasklet(unsigned long stream)
{
	struct snd_pcm_substream *substream =
		(struct snd_pcm_substream *)stream;
	struct spear13xx_runtime_data *prtd = substream->runtime->private_data;

	if (dma_xfer(substream))
		dev_err(&prtd->dev, "error with dma_xfer\n");
}

static void dma_configure(struct snd_pcm_substream *substream)
{
	struct spear13xx_runtime_data *prtd = substream->runtime->private_data;

	dma_cap_zero(prtd->mask);
	dma_cap_set(DMA_SLAVE, prtd->mask);

	prtd->slaves = &data;
	/* we need to pass physical address here */
	prtd->slaves->mem2i2s_slave.tx_reg = (dma_addr_t)prtd->txdma;
	prtd->slaves->mem2i2s_slave.rx_reg = 0;
	prtd->slaves->i2s2mem_slave.tx_reg = 0;
	prtd->slaves->i2s2mem_slave.rx_reg = (dma_addr_t)prtd->rxdma;

	substream->runtime->private_data = prtd;

}

static void dma_stop(struct snd_pcm_substream *substream)
{
	struct spear13xx_runtime_data *prtd = substream->runtime->private_data;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		dma_release_channel(prtd->dma_chan[0]);
	else
		dma_release_channel(prtd->dma_chan[1]);

}

static bool filter(struct dma_chan *chan, void *slave)
{
	chan->private = slave;
	return true;
}

static int spear13xx_pcm_dma_request(struct snd_pcm_substream *substream)
{
	struct spear13xx_runtime_data *prtd = substream->runtime->private_data;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		prtd->dma_chan[0] = dma_request_channel(prtd->mask, filter,
				&prtd->slaves->mem2i2s_slave);
		if (!prtd->dma_chan[0])
			return -EAGAIN;
	} else {
		prtd->dma_chan[1] = dma_request_channel(prtd->mask, filter,
				&prtd->slaves->i2s2mem_slave);
		if (!prtd->dma_chan[1])
			return -EAGAIN;

	}
	substream->runtime->private_data = prtd;

	return 0;
}

static int spear13xx_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct spear13xx_runtime_data *prtd = substream->runtime->private_data;
	struct dma_chan *chan = prtd->dma_chan[substream->stream];
	int data_type, ret = 0;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		data_type = 0;
	else
		data_type = 1;

	chan = prtd->dma_chan[data_type];

	spin_lock(&prtd->lock);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		tasklet_schedule(&prtd->tasklet);
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		break;
	default:
		ret = -EINVAL;
		break;
	}
	spin_unlock(&prtd->lock);

	return ret;
}

static int spear13xx_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct spear13xx_runtime_data *prtd = substream->runtime->private_data;

	prtd->period = 0;

	return 0;
}

/*
 * The period defines the size at which a PCM interrupt is generated. Now
 * ALSA (audio) buffer is divided into periods, i.e. a chain of small
 * packets and the size is each packet is equal to max_period_bytes.
 * This callback is called when the PCM middle layer inquires the current
 * hardware position on the buffer. The position must be returned in frames,
 * ranging from 0 to buffer_size - 1.
 * runtime->dma_addr is the base address of the buffer.
 * prtd->dma_addr will update on each interrupt by period * period_size
 */
static snd_pcm_uframes_t
spear13xx_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct spear13xx_runtime_data *prtd = runtime->private_data;
	unsigned int offset;
	dma_addr_t count;

	spin_lock(&prtd->lock);
	count = prtd->dma_addr - runtime->dma_addr;
	spin_unlock(&prtd->lock);
	offset = bytes_to_frames(runtime, count);
	if (offset >= runtime->buffer_size)
		offset = 0;
	return offset;
}

static int spear13xx_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct spear13xx_runtime_data *prtd;
	int ret = 0;
	u32 sts;

	snd_soc_set_runtime_hwparams(substream, &spear13xx_pcm_hardware);

	/* ensure that buffer size is a multiple of period size */
	ret = snd_pcm_hw_constraint_integer(runtime,
			SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0)
		goto out;

	prtd = kzalloc(sizeof(*prtd), GFP_KERNEL);
	if (prtd == NULL)
		return -ENOMEM;

	spin_lock_init(&prtd->lock);

	runtime->private_data = prtd;

	get_dma_start_addr(substream);
	dma_configure(substream);
	sts = spear13xx_pcm_dma_request(substream);
	if (sts) {
		dev_err(&prtd->dev, "pcm:Failed to get dma channels\n");
		kfree(prtd);

	}

	tasklet_init(&prtd->tasklet , pcm_dma_tasklet,
			(unsigned long)substream);
out:
	return ret;
}

static int spear13xx_pcm_close(struct snd_pcm_substream *substream)
{
	struct spear13xx_runtime_data *prtd = substream->runtime->private_data;
	struct dma_chan *chan;

	tasklet_kill(&prtd->tasklet);
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		chan = prtd->dma_chan[0];
	else
		chan = prtd->dma_chan[1];

	chan->device->device_terminate_all(chan);
	dma_stop(substream);
	kfree(prtd);

	return 0;
}

static int spear13xx_pcm_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *hw_params)
{
	return snd_pcm_lib_malloc_pages(substream,
			params_buffer_bytes(hw_params));
}

static int spear13xx_pcm_hw_free(struct snd_pcm_substream *substream)
{
	return snd_pcm_lib_free_pages(substream);
}

struct snd_pcm_ops spear13xx_pcm_ops = {
	.open		= spear13xx_pcm_open,
	.close		= spear13xx_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= spear13xx_pcm_hw_params,
	.hw_free	= spear13xx_pcm_hw_free,
	.prepare	= spear13xx_pcm_prepare,
	.trigger	= spear13xx_pcm_trigger,
	.pointer	= spear13xx_pcm_pointer,
};

static int spear13xx_pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	size_t size = spear13xx_pcm_hardware.buffer_bytes_max;

	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;
	buf->area = dma_alloc_writecombine(pcm->card->dev, size,
			&buf->addr, GFP_KERNEL);
	dev_info(buf->dev.dev,
			" preallocate_dma_buffer: area=%p, addr=%p, size=%d\n",
			(void *)buf->area, (void *)buf->addr, size);
	if (!buf->area)
		return -ENOMEM;
	buf->bytes = size;
	return 0;
}

static void spear13xx_pcm_free(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;
	int stream;

	for (stream = 0; stream < 2; stream++) {
		substream = pcm->streams[stream].substream;
		if (!substream)
			continue;

		buf = &substream->dma_buffer;
		if (!buf->area)
			continue;

		dma_free_writecombine(pcm->card->dev, buf->bytes,
				buf->area, buf->addr);
		buf->area = NULL;
	}
}

static int spear13xx_pcm_new(struct snd_card *card,
		struct snd_soc_dai *dai, struct snd_pcm *pcm)
{
	int ret;

	if (!card->dev->dma_mask)
		card->dev->dma_mask = &spear13xx_pcm_dmamask;
	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = 0xffffffff;

	if (dai->playback.channels_min) {
		ret = spear13xx_pcm_preallocate_dma_buffer(pcm,
				SNDRV_PCM_STREAM_PLAYBACK);
		if (ret)
			return ret;
	}

	if (dai->capture.channels_min) {
		ret = spear13xx_pcm_preallocate_dma_buffer(pcm,
				SNDRV_PCM_STREAM_CAPTURE);
		if (ret)
			return ret;
	}

	return 0;
}

struct snd_soc_platform spear13xx_soc_platform = {
	.name = "spear-audio",
	.pcm_ops = &spear13xx_pcm_ops,
	.pcm_new = spear13xx_pcm_new,
	.pcm_free = spear13xx_pcm_free,
};
EXPORT_SYMBOL_GPL(spear13xx_soc_platform);

static int __init spear13xx_soc_platform_init(void)
{
	return snd_soc_register_platform(&spear13xx_soc_platform);
}
module_init(spear13xx_soc_platform_init);

static void __exit spear13xx_soc_platform_exit(void)
{
	snd_soc_unregister_platform(&spear13xx_soc_platform);
}
module_exit(spear13xx_soc_platform_exit);

MODULE_AUTHOR("Rajeev Kumar");
MODULE_DESCRIPTION("spear PCM DMA module");
MODULE_LICENSE("GPL");
