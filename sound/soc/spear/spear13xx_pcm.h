/*
 * ALSA PCM interface for ST spear Processor
 *
 * sound/soc/spear/spear13xx_pcm.h
 *
 * Copyright (C) 2011 ST Microelectronics
 * Rajeev Kumar<rajeev-dlh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef SPEAR13XX_PCM_H
#define SPEAR13XX_PCM_H

#define WORD_WIDTH	0x02
#define PCM_WIDTH WORD_WIDTH
#define MAX_CHANNEL 2

struct spear13xx_pcm_dma_params {
	char *name;		/* stream identifier */
	dma_addr_t dma_addr;	/* device physical address for DMA */
};

struct spear13xx_runtime_data {
	struct device dev;
	struct dma_chan *dma_chan[2];
	struct tasklet_struct tasklet;
	spinlock_t lock;
	struct spear13xx_pcm_dma_params *params;	/* DMA params */
	int stream;
	struct snd_pcm_substream *substream;
	unsigned long pos;
	dma_addr_t dma_addr;
	unsigned long buffer_bytes;
	unsigned long period_bytes;
	unsigned long frag_bytes;
	int frags;
	int frag_count;
	int dmacount;
};
#endif /* end of pcm header file */
