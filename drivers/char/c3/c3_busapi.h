/*
 * Copyright (C) 2007-2010 STMicroelectronics Ltd
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 */

/* --------------------------------------------------------------------
 * C3 driver BUS functions
 *
 * ST - 2007-04-10
 * STM Grenoble
 *
 * - 2010-02-11 MGR SPEAr basic rev. BA addon.
 * - 2011-03-08 SK: Removed other archs. Just Spear supported now.
 * ----------------------------------------------------------------- */

#ifndef __C3_BUS_API_H__
#define __C3_BUS_API_H__

/* --------------------------------------------------------------------
 * INCLUDES
 * ----------------------------------------------------------------- */

/* Primitives for spear */
#include "c3_spear.h"

/* -------------------------------------------------------------------
 * MACROS
 * ---------------------------------------------------------------- */

#define c3bus_init			c3spear_init
#define c3bus_cleanup			c3spear_cleanup
#define c3bus_search			c3spear_search
#define c3bus_get_address		c3spear_get_address
#define c3bus_periph_number		c3spear_periph_number
#define c3bus_interrupt_line		c3spear_interrupt_line

#endif /*  __C3_BUS_API_H__                                           */
