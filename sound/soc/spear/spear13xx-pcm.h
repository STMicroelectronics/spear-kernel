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
	spinlock_t lock;
	int period;		/* current DMA period */
	dma_addr_t dma_addr;	/* device physical address for DMA */
	struct spear13xx_pcm_dma_params *params;	/* DMA params */

	/* DMA Transfer */
	dma_addr_t txdma;
	dma_addr_t rxdma;
	struct tasklet_struct tasklet;
	struct pcm_dma_data *slaves;
	struct dma_chan *dma_chan[MAX_CHANNEL];
	dma_cap_mask_t mask;
	struct scatterlist sg[MAX_CHANNEL];
	u32 sg_len[MAX_CHANNEL];
	dma_cookie_t cookie[MAX_CHANNEL];

};

extern struct snd_soc_platform spear13xx_soc_platform;

#endif /* end of pcm header file */
