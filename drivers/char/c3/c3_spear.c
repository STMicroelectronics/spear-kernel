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
 * C3 driver SPEAR functions
 *
 * ST - 2007-04-10
 * Alessandro Miglietti
 *
 * - 2010-02-11 MGR Adaptation to spearbasic rev.BA and STLinux distro.
 * - 2010-09-16 SK: Added Spear13xx support.
 * ----------------------------------------------------------------- */

/* --------------------------------------------------------------------
 * INCLUDES
 * ----------------------------------------------------------------  */

#include "c3_common.h"

#if defined(C3_SPEARBASIC_REVA) || defined(C3_SPEARBASIC_REVBA) || \
	defined(C3_SPEAR13XX)

/* include the header */
#include "c3_spear.h"

#include <linux/init.h>

static unsigned int *__c3_spear_bus_address;

#include <mach/hardware.h>
#include <mach/spear.h>
#include <mach/irqs.h>

#if defined(C3_SPEAR13xx)

	#define C3_IRQ	IRQ_C3 /*mach-spear13xx/include/mach/irqs.h*/

#else /*so C3_SPEARBASIC_REVA || C3_SPEARBASIC_REVBA*/

	#  ifdef C3_SPEARBASIC_REVA
		#define C3_IRQ	29 /* FIQ_GEN_RAS_2 */
static unsigned int *__c3_va_ras_2;
static unsigned int *__c3_mem_cfg;

	#  else /*so C3_SPEARBASIC_REVBA*/
		#define C3_IRQ	31 /* IRQ_HW_ACCEL_MOD_1 */

	#  endif        /*C3_SPEARBASIC_REVBA*/

#endif                  /*ARCH*/

#define C3_BASE_ADDRESS    __c3_spear_bus_address

/* -------------------------------------------------------------------
* FUNCTIONS
* ----------------------------------------------------------------- */

/* ===================================================================
 *
 * INITIALIZATION AND CLEANUP
 *
 * ================================================================ */

unsigned int __init c3spear_init(void)
{
#ifdef C3_SPEARBASIC_REVA
	__c3_va_ras_2 = ioremap_nocache(0x99000004, 0x4);
	__c3_mem_cfg  = ioremap_nocache(0xfca80050, 0x4);

	(*__c3_va_ras_2) = 0x3;
	(*__c3_mem_cfg)  = 0x1;

	__c3_spear_bus_address = ioremap_nocache(0x40000000, 0xFFFF);

#elif defined(C3_SPEAR13xx)
	__c3_spear_bus_address = ioremap_nocache(0xE1800000, 0xFFFF);
#else   /*!C3_SPEARBASIC_REVA*/
	__c3_spear_bus_address = ioremap_nocache(0xD9000000, 0xFFFF);
#endif /*!C3_SPEARBASIC_REVA*/

	return C3_OK;

} /* c3_status __init  c3spear_init()  */

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

void c3spear_cleanup(unsigned int periph_id)
{
	iounmap(__c3_spear_bus_address);

#ifdef C3_SPEARBASIC_REVA
	iounmap(__c3_va_ras_2);
	iounmap(__c3_mem_cfg);
#endif  /*C3_SPEARBASIC_REVA*/

} /*void c3spear_cleanup() */

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

/* ===================================================================
 *
 * ASSIGNEMENTS
 *
 * ================================================================ */

unsigned int *c3spear_get_address(unsigned int periph_index)
{
	return (unsigned int *)C3_BASE_ADDRESS;
} /* c3_addr_t c3spear_get_address */

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3spear_periph_number(void)
{
	return 1;

} /* c3_uint8_t c3spear_periph_number */

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3spear_interrupt_line(unsigned int pci_index)
{
	return (unsigned int)C3_IRQ;

} /* c3_uint32_t c3spear_get_interrupt_line  */

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#endif /* C3_SPEARBASIC_REVA  || C3_SPEARBASIC_REVBA ...*/

