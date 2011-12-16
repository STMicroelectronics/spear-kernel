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

struct i2s_platform_data {
	#define PLAY	(1 << 0)
	#define RECORD	(1 << 1)
	unsigned int cap;
	int channel;
	u8 swidth;

	void *play_dma_data;
	void *capture_dma_data;
};

/* I2S DMA registers */
#define I2S_RXDMA		0x01C0
#define I2S_TXDMA		0x01C8

#define TWO_CHANNEL_SUPPORT	2	/* up to 2.0 */
#define FOUR_CHANNEL_SUPPORT	4	/* up to 3.1 */
#define SIX_CHANNEL_SUPPORT	6	/* up to 5.1 */
#define EIGHT_CHANNEL_SUPPORT	8	/* up to 7.1 */

#endif /* DESIGNWARE_I2S_H */
