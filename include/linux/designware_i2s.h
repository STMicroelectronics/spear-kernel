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

#include <sound/pcm.h>

struct i2s_platform_data {
	#define PLAY	(1 << 0)
	#define RECORD	(1 << 1)
	unsigned int cap;
	int channel;
};

void get_dma_start_addr(struct snd_pcm_substream *substream);

#endif /* DESIGNWARE_I2S_H */
