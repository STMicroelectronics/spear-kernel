/*
* linux/designware_i2s.h
*
* Copyright (ST) 2011 Rajeev Kumar (rajeev-dlh.kumar@st.com)
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
*
*/

#ifndef DESIGNWARE_I2S_H
#define DESIGNWARE_I2S_H

#include <linux/platform_device.h>
#include <linux/dw_dmac.h>
#include <sound/pcm.h>

struct dma_slaves {
	struct dw_dma_slave mem2i2s_slave;
	struct dw_dma_slave i2s2mem_slave;
};

struct i2s_platform_data {
	#define PLAY	(1 << 0)
	#define RECORD	(1 << 1)
	unsigned int cap;
	int channel;
	struct dma_slaves ds;
};

#define TWO_CHANNEL_SUPPORT	2	/* up to 2.0 */
#define FOUR_CHANNEL_SUPPORT	4	/* up to 3.1 */
#define SIX_CHANNEL_SUPPORT	6	/* up to 5.1 */
#define EIGHT_CHANNEL_SUPPORT	8	/* up to 7.1 */

struct dma_slaves *substream_to_ds(struct snd_pcm_substream *substream,
		dma_cap_mask_t *smask);

#endif /* DESIGNWARE_I2S_H */
