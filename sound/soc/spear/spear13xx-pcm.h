/*
 * ALSA PCM interface for ST spear Processor
 *
 * sound/soc/spear/spear13xx-pcm.h
 *
 * Copyright (C) 2010 ST Microelectronics
 * Rajeev Kumar<rajeev-dlh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef SPEAR_PCM_H
#define SPEAR_PCM_H

#include <linux/dw_dmac.h>

#define WORD_WIDTH	0x02
#define PCM_WIDTH WORD_WIDTH
#define MAX_CHANNEL 2

struct pcm_dma_data {
	struct dw_dma_slave mem2i2s_slave;
	struct dw_dma_slave i2s2mem_slave;
};

struct spear13xx_pcm_dma_params {
	char *name;		/* stream identifier */
	dma_addr_t dma_addr;	/* device physical address for DMA */
};

struct spear13xx_runtime_data {
	struct device dev;
	struct pcm_dma_data *slaves;
	struct dma_chan *dma_chan[2];
	struct tasklet_struct tasklet;
	spinlock_t lock;
	dma_addr_t txdma;
	struct spear13xx_pcm_dma_params *params;	/* DMA params */
	dma_addr_t rxdma;
	dma_cap_mask_t mask;
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
