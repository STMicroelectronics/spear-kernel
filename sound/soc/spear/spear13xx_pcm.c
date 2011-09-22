/*
 * ALSA PCM interface for ST spear Processor
 *
 * sound/soc/spear/spear13xx_pcm.c
 *
 * Copyright (C) 2011 ST Microelectronics
 * Rajeev Kumar<rajeev-dlh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/designware_i2s.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <mach/dma.h>
#include "spear13xx_pcm.h"

static u64 spear13xx_pcm_dmamask = 0xFFFFFFFF;

struct snd_pcm_hardware spear13xx_pcm_hardware = {
	.info = (SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER |
		 SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID |
		 SNDRV_PCM_INFO_PAUSE | SNDRV_PCM_INFO_RESUME),
	.formats = (SNDRV_PCM_FMTBIT_S16_LE),
	.rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
			SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 |
			SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 |
			SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000 |
			SNDRV_PCM_RATE_KNOT),
	.rate_min = 8000,
	.rate_max = 96000,
	.channels_min = 2,
	.channels_max = 8,
	.buffer_bytes_max = 16 * 1024, /* max buffer size */
	.period_bytes_min = 2 * 1024, /* 1 msec data minimum period size */
	.period_bytes_max = 2 * 1024, /* maximum period size */
	.periods_min = 1, /* min # periods */
	.periods_max = 8, /* max # of periods */
	.fifo_size = 0, /* fifo size in bytes */
};

static void pcm_dma_complete(void *arg);

static int spear13xx_pcm_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct spear13xx_runtime_data *prtd = runtime->private_data;
	int ret;

	ret = snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(params));
	if (ret < 0)
		return ret;

	prtd->substream = substream;
	return 0;
}

static int spear13xx_pcm_hw_free(struct snd_pcm_substream *substream)
{
	return snd_pcm_lib_free_pages(substream);
}

static int spear13xx_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct spear13xx_runtime_data *prtd = runtime->private_data;
	unsigned long flags;

	spin_lock_irqsave(&prtd->lock, flags);
	prtd->dma_addr = runtime->dma_addr;
	prtd->buffer_bytes = snd_pcm_lib_buffer_bytes(substream);
	prtd->period_bytes = snd_pcm_lib_period_bytes(substream);

	prtd->buf_index = 0;
	prtd->dmacount = 0;
	prtd->xfer_len = prtd->period_bytes;
	prtd->xfer_cnt = prtd->buffer_bytes / prtd->period_bytes;
	spin_unlock_irqrestore(&prtd->lock, flags);

	return 0;
}

static int start_dma(struct spear13xx_runtime_data *prtd)
{
	struct dma_chan *chan;
	struct dma_async_tx_descriptor *desc;
	struct scatterlist sg;
	enum dma_data_direction direction;
	dma_addr_t addr;

	addr = prtd->dma_addr + prtd->buf_index * prtd->xfer_len;
	if (prtd->substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		chan = prtd->dma_chan[0];
		direction = DMA_TO_DEVICE;
	} else {
		chan = prtd->dma_chan[1];
		direction = DMA_FROM_DEVICE;
	}

	/* Prepare sg's */
	sg_init_table(&sg, 1);
	sg_set_page(&sg, pfn_to_page(PFN_DOWN(addr)), prtd->xfer_len,
			addr & (PAGE_SIZE - 1));
	sg_dma_address(&sg) = addr;

	desc = chan->device->device_prep_slave_sg(chan, &sg, 1, direction,
			DMA_PREP_INTERRUPT);
	if (!desc) {
		dev_err(&chan->dev->device, "cannot prepare slave dma\n");
		return -EAGAIN;
	}

	desc->callback = pcm_dma_complete;
	desc->callback_param = prtd;
	desc->tx_submit(desc);

	return 0;
}

static void pcm_dma_xfer(struct spear13xx_runtime_data *prtd,
		bool from_callback)
{
	struct snd_pcm_substream *substream = prtd->substream;
	struct dma_chan *chan;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&prtd->lock, flags);
	if (!prtd->pcm_running) {
		spin_unlock_irqrestore(&prtd->lock, flags);
		return;
	}

	BUG_ON(prtd->dmacount >= prtd->xfer_cnt);

	while (prtd->dmacount < prtd->xfer_cnt) {

		ret = start_dma(prtd);
		if (ret) {
			spin_unlock_irqrestore(&prtd->lock, flags);
			return;
		}

		prtd->dmacount++;
		prtd->buf_index++;

		/* Set to zero, if crosses buffer size */
		prtd->buf_index %= prtd->xfer_cnt;

		/* Inform framework that a transfer is finished */
		if (from_callback)
			snd_pcm_period_elapsed(substream);
	}
	spin_unlock_irqrestore(&prtd->lock, flags);

	/* Issue pending should be called after locks */
	if (prtd->substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		chan = prtd->dma_chan[0];
	else
		chan = prtd->dma_chan[1];

	chan->device->device_issue_pending(chan);
}

static void pcm_dma_complete(void *arg)
{
	struct spear13xx_runtime_data *prtd = arg;
	unsigned long flags;

	spin_lock_irqsave(&prtd->lock, flags);
	prtd->dmacount--;
	spin_unlock_irqrestore(&prtd->lock, flags);

	if (prtd->pcm_running)
		pcm_dma_xfer(prtd, true);
}

static int spear13xx_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct spear13xx_runtime_data *prtd = substream->runtime->private_data;
	struct dma_chan *chan;
	unsigned long flags;
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_RESUME:
		spin_lock_irqsave(&prtd->lock, flags);
		/* Go to last successfully transferred index + 1 */
		prtd->buf_index += prtd->xfer_cnt - prtd->dmacount;
		prtd->buf_index %= prtd->xfer_cnt;
		prtd->dmacount = 0;
		spin_unlock_irqrestore(&prtd->lock, flags);
	case SNDRV_PCM_TRIGGER_START:
		spin_lock_irqsave(&prtd->lock, flags);
		prtd->pcm_running = true;
		spin_unlock_irqrestore(&prtd->lock, flags);
		pcm_dma_xfer(prtd, false);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		prtd->pcm_running = false;
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			chan = prtd->dma_chan[0];
		else
			chan = prtd->dma_chan[1];

		chan->device->device_control(chan, DMA_TERMINATE_ALL, 0);
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static snd_pcm_uframes_t
spear13xx_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct spear13xx_runtime_data *prtd = substream->runtime->private_data;

	return bytes_to_frames(substream->runtime, prtd->buf_index *
			prtd->xfer_len);
}

static bool filter(struct dma_chan *chan, void *slave)
{
	chan->private = slave;
	return true;
}

static int pcm_alloc_dma_chan(struct snd_pcm_substream *substream)
{
	struct spear13xx_runtime_data *prtd = substream->runtime->private_data;
	dma_cap_mask_t smask;
	struct dma_slaves *ds = substream_to_ds(substream, &smask);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		prtd->dma_chan[0] = dma_request_channel(smask, filter,
				&ds->mem2i2s_slave);
		if (!prtd->dma_chan[0])
			return -EAGAIN;
	} else {
		prtd->dma_chan[1] = dma_request_channel(smask, filter,
				&ds->i2s2mem_slave);
		if (!prtd->dma_chan[1])
			return -EAGAIN;
	}

	return 0;
}

static void pcm_dma_free_chan(struct snd_pcm_substream *substream)
{
	struct spear13xx_runtime_data *prtd = substream->runtime->private_data;
	struct dma_chan *chan;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		chan = prtd->dma_chan[0];
	else
		chan = prtd->dma_chan[1];

	chan->device->device_control(chan, DMA_TERMINATE_ALL, 0);
	dma_release_channel(chan);
}

static int spear13xx_pcm_open(struct snd_pcm_substream *substream)
{
	struct spear13xx_runtime_data *prtd;
	int ret;

	ret = snd_soc_set_runtime_hwparams(substream, &spear13xx_pcm_hardware);
	if (ret)
		return ret;

	/* ensure that buffer size is a multiple of period size */
	ret = snd_pcm_hw_constraint_integer(substream->runtime,
			SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0)
		return ret;

	prtd = kzalloc(sizeof(*prtd), GFP_KERNEL);
	if (!prtd)
		return -ENOMEM;

	spin_lock_init(&prtd->lock);
	substream->runtime->private_data = prtd;

	ret = pcm_alloc_dma_chan(substream);
	if (ret) {
		dev_err(&prtd->dev, "pcm:Failed to get dma channels\n");
		kfree(prtd);

	}
	return 0;
}

static int spear13xx_pcm_close(struct snd_pcm_substream *substream)
{
	struct spear13xx_runtime_data *prtd = substream->runtime->private_data;

	pcm_dma_free_chan(substream);
	kfree(prtd);
	return 0;
}

static int spear13xx_pcm_mmap(struct snd_pcm_substream *substream,
		struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	return dma_mmap_writecombine(substream->pcm->card->dev, vma,
			runtime->dma_area, runtime->dma_addr,
			runtime->dma_bytes);
}

static struct snd_pcm_ops spear13xx_pcm_ops = {
	.open		= spear13xx_pcm_open,
	.close		= spear13xx_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= spear13xx_pcm_hw_params,
	.hw_free	= spear13xx_pcm_hw_free,
	.prepare	= spear13xx_pcm_prepare,
	.trigger	= spear13xx_pcm_trigger,
	.pointer	= spear13xx_pcm_pointer,
	.mmap		= spear13xx_pcm_mmap,
};

static int
spear13xx_pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream,
		size_t size)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;

	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;

	buf->area = dma_alloc_writecombine(pcm->card->dev, size,
			&buf->addr, GFP_KERNEL);
	if (!buf->area)
		return -ENOMEM;

	dev_info(buf->dev.dev,
			" preallocate_dma_buffer: area=%p, addr=%p, size=%d\n",
			(void *)buf->area, (void *)buf->addr, size);

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
		card->dev->coherent_dma_mask = 0xFFFFFFFF;

	if (dai->driver->playback.channels_min) {
		ret = spear13xx_pcm_preallocate_dma_buffer(pcm,
				SNDRV_PCM_STREAM_PLAYBACK,
				spear13xx_pcm_hardware.buffer_bytes_max);
		if (ret)
			return ret;
	}

	if (dai->driver->capture.channels_min) {
		ret = spear13xx_pcm_preallocate_dma_buffer(pcm,
				SNDRV_PCM_STREAM_CAPTURE,
				spear13xx_pcm_hardware.buffer_bytes_max);
		if (ret)
			return ret;
	}

	return 0;
}

struct snd_soc_platform_driver spear13xx_soc_platform = {
	.ops		=	&spear13xx_pcm_ops,
	.pcm_new	=	spear13xx_pcm_new,
	.pcm_free	=	spear13xx_pcm_free,
};

static int __devinit spear13xx_soc_platform_probe(struct platform_device *pdev)
{
	return snd_soc_register_platform(&pdev->dev, &spear13xx_soc_platform);
}

static int __devexit spear13xx_soc_platform_remove(struct platform_device *pdev)
{
	snd_soc_unregister_platform(&pdev->dev);

	return 0;
}

static struct platform_driver spear13xx_pcm_driver = {
	.driver = {
		.name = "spear-pcm-audio",
		.owner = THIS_MODULE,
	},

	.probe = spear13xx_soc_platform_probe,
	.remove = __devexit_p(spear13xx_soc_platform_remove),
};
static int __init snd_spear13xx_pcm_init(void)
{
	return platform_driver_register(&spear13xx_pcm_driver);
}
module_init(snd_spear13xx_pcm_init);

static void __exit snd_spear13xx_pcm_exit(void)
{
	platform_driver_unregister(&spear13xx_pcm_driver);
}
module_exit(snd_spear13xx_pcm_exit);

MODULE_AUTHOR("Rajeev Kumar <rajeev-dlh.kumar@st.com>");
MODULE_DESCRIPTION("spear PCM DMA module");
MODULE_LICENSE("GPL");
