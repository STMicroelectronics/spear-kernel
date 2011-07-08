/*
 * drivers/char/spear_adc_st10.h
 * ST ADC driver - header file
 *
 * Copyright (ST) 2009 Viresh Kumar (viresh.kumar@st.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef SPEAR_ADC_H
#define SPEAR_ADC_H

#include <linux/spear_adc_usr.h>

s32 spear_adc_chan_get(void *dev_id, enum adc_chan_id chan_id);
s32 spear_adc_chan_put(void *dev_id, enum adc_chan_id chan_id);
s32 spear_adc_configure(void *dev_id, enum adc_chan_id chan_id, struct
		adc_config * config);
s32 spear_adc_chan_configure(void *dev_id, struct adc_chan_config * config);
s32 spear_adc_get_data(void *dev_id, enum adc_chan_id chan_id,
		unsigned int *digital_volt, u32 count);
s32 spear_adc_get_configure(void *dev_id, enum adc_chan_id chan_id,
		struct adc_config *config);
s32 spear_adc_get_chan_configure(void *dev_id, struct adc_chan_config *config);

#endif /* SPEAR_ADC_H */
