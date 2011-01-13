/*
 * ALSA SoC I2S Audio Layer for ST spear13xx processor
 *
 * sound/soc/spear/spear13xx-i2s.h
 *
 * Copyright (C) 2010 ST Microelectronics
 * Rajeev Kumar<rajeev-dlh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef SPEAR_I2S_H
#define SPEAR_I2S_H

#define MAX_CHANNEL_NUM		2
#define MIN_CHANNEL_NUM		2

void get_dma_start_addr(struct snd_pcm_substream *substream);

#endif /*end if i2s header file */
