/*
 * Copyright (ST) 2011 Vipin Kumar (vipin.kumar@st.com)
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
 */

#ifndef MACH_SPDIF_OUT_H
#define MACH_SPDIF_OUT_H

struct spdif_out_platform_data {
	/* DMA params */
	void *dma_params;
};

#define SPDIF_OUT_FIFO_DATA	0x04

#endif /* MACH_SPDIF_OUT_H */
