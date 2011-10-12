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

struct spear13xx_runtime_data {
	struct dma_chan *dma_chan[2];
	spinlock_t lock;
	struct snd_pcm_substream *substream;
	dma_addr_t dma_addr;
	unsigned long buffer_bytes;
	unsigned long period_bytes;

	/* DMA related mask */
	dma_cap_mask_t smask;

	/* For Keeping track of buffers */
	unsigned long xfer_len;	/* Data transfered by one transfer */
	int xfer_cnt; /* Total number of transfers to be done */
	int buf_index; /* Current buffer count */
	int dmacount; /* No. of DMA transfer ongoing */
	bool pcm_running; /* Current state of pcm, true if running */
};
#endif /* end of pcm header file */
