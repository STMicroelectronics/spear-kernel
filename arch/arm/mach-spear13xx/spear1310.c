/*
 * arch/arm/mach-spear13xx/spear1310.c
 *
 * SPEAr1310 machine source file
 *
 * Copyright (C) 2010 ST Microelectronics
 * Bhupesh Sharma <bhupesh.sharma@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/ptrace.h>
#include <asm/irq.h>
#include <mach/generic.h>
#include <mach/hardware.h>

/* Add spear1310 specific devices here */

/* CAN device registeration */
static struct resource can0_resources[] = {
	{
		.start = SPEAR1310_CAN0_BASE,
		.end = SPEAR1310_CAN0_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = IRQ_CCAN0,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear1310_can0_device = {
	.name = "c_can_platform",
	.id = 0,
	.num_resources = ARRAY_SIZE(can0_resources),
	.resource = can0_resources,
};

static struct resource can1_resources[] = {
	{
		.start = SPEAR1310_CAN1_BASE,
		.end = SPEAR1310_CAN1_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = IRQ_CCAN1,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear1310_can1_device = {
	.name = "c_can_platform",
	.id = 1,
	.num_resources = ARRAY_SIZE(can1_resources),
	.resource = can1_resources,
};

void __init spear1310_init(void)
{
	/* call spear13xx family common init function */
	spear13xx_init();
}
