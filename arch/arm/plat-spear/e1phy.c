/*
 * arch/arm/plat-spear/e1phy.c
 *
 * E1 PHY initialization for SPEAr HDLC controller
 *
 * Copyright (C) 2010 ST Microelectronics
 * Frank Shi<frank.shi@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/kernel.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <mach/spear.h>

int e1phy_init(u32 base, int shift)
{
	int i;
	void __iomem *reg_base = ioremap(base, 256 << shift);

	/* Disable TDM_EN
	 * offset 0xef is not E1 PHY, it is captured by CPLD
	 */
	writeb(0, reg_base + (0xef << shift));

	/* init all registers */
	for (i = 0; i <= 0xbf; i++)
		writeb(0, reg_base + (i << shift));

	msleep(10);

	/* write LIRST reg */
	writeb(0x80, reg_base + (0xaa << shift));

	msleep(50);

	/* write TAF & TNAF flags */
	writeb(0x1b, reg_base + (0x20 << shift));
	writeb(0x40, reg_base + (0x21 << shift));

	/* set TSYNC output */
	writeb(0x1, reg_base + (0x12 << shift));

	/* Uncomment the following to enable Rx Elastic store */
	/* writeb(0x6, reg_base + (0x11 << shift));
	 */

	/* HDB3 code, Rx CCS mode */
	writeb(0x4c, reg_base + (0x14 << shift));

	/* Uncomment to following to enable Framer loopback */
	/* writeb(0x40, reg_base + (0xa8 * shift));
	 */

	msleep(20);

#ifdef DEBUG
	/* dump all the registers */
	printk("E1 phy regs:");
	for (i = 0; i <= 0xbf; i++) {
		if (i % 8 == 0)
			printk("\n");
		printk("    Reg[%02x]=%02x", i, readb(reg_base + (i << shift)));
	}
	printk("\n");
#endif
	iounmap(reg_base);

	return 0;
}
