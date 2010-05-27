/*
 * arch/arm/mach-spear13xx/pcie.c
 *
 * PCIe functions for SPEAr 13xx SoCs
 *
 * Copyright (C) 2010 ST Microelectronics
 * Pratyush Anand <pratyush.anand@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/mbus.h>
#include <linux/sched.h>
#include <asm/irq.h>
#include <asm/mach/pci.h>
#include <mach/pcie.h>
#include <mach/misc_regs.h>

#define NUM_PCIE_PORTS	3

#define IN0_MEM_SIZE	(200 * 1024 * 1024 - 1)
/* In current implementation address translation is done using IN0 only.
 * So IN1 start address and IN0 end address has been kept same
*/
#define IN1_MEM_SIZE	(0 * 1024 * 1024 - 1)
#define IN_IO_SIZE	(20 * 1024 * 1024 - 1)
#define IN_CFG0_SIZE	(12 * 1024 * 1024 - 1)
#define IN_CFG1_SIZE	(12 * 1024 * 1024 - 1)
#define IN_MSG_SIZE	(12 * 1024 * 1024 - 1)

#define MAX_LINK_UP_WAIT_JIFFIES	10

struct pcie_port {
	u8			port;
	u8			root_bus_nr;
	void __iomem		*base;
	void __iomem		*app_base;
	void __iomem		*va_app_base;
	spinlock_t		conf_lock;
	char			mem_space_name[16];
	char			io_space_name[16];
	struct resource		res[2];
};

static struct pcie_port pcie_port[NUM_PCIE_PORTS];
static u32 spr_pcie_base[NUM_PCIE_PORTS] = {
	SPEAR13XX_PCIE0_BASE,
	SPEAR13XX_PCIE1_BASE,
	SPEAR13XX_PCIE2_BASE,
};
static u32 spr_pcie_app_base[NUM_PCIE_PORTS] = {
	SPEAR13XX_PCIE0_APP_BASE,
	SPEAR13XX_PCIE1_APP_BASE,
	SPEAR13XX_PCIE2_APP_BASE,
};

static int spear13xx_pcie_link_up(void __iomem *va_app_base)
{
	struct pcie_app_reg *app_reg = (struct pcie_app_reg *) va_app_base;
	unsigned long deadline = jiffies + MAX_LINK_UP_WAIT_JIFFIES;

	do {
		if (readl(&app_reg->app_status_1) &
				((u32)1 << XMLH_LINK_UP_ID))
			return 1;

		cond_resched();
	} while (!time_after_eq(jiffies, deadline));

	return 0;
}

static void spear13xx_pcie_host_init(void *va_app_base, u32 base)
{
	struct pcie_app_reg *app_reg = (struct pcie_app_reg *) va_app_base;

	writel(base, &app_reg->in0_mem_addr_start);
	writel(app_reg->in0_mem_addr_start + IN0_MEM_SIZE,
			&app_reg->in0_mem_addr_limit);
	writel(app_reg->in0_mem_addr_limit + 1, &app_reg->in1_mem_addr_start);
	writel(app_reg->in1_mem_addr_start + IN1_MEM_SIZE,
			&app_reg->in1_mem_addr_limit);
	writel(app_reg->in1_mem_addr_limit + 1, &app_reg->in_io_addr_start);
	writel(app_reg->in_io_addr_start + IN_IO_SIZE,
			&app_reg->in_io_addr_limit);
	writel(app_reg->in_io_addr_limit + 1, &app_reg->in_cfg0_addr_start);
	writel(app_reg->in_cfg0_addr_start + IN_CFG0_SIZE,
			&app_reg->in_cfg0_addr_limit);
	writel(app_reg->in_cfg0_addr_limit + 1, &app_reg->in_cfg1_addr_start);
	writel(app_reg->in_cfg1_addr_start + IN_CFG1_SIZE,
			&app_reg->in_cfg1_addr_limit);
	writel(app_reg->in_cfg1_addr_limit + 1, &app_reg->in_msg_addr_start);
	writel(app_reg->in_msg_addr_start + IN_MSG_SIZE,
			&app_reg->in_msg_addr_limit);

	writel(app_reg->in0_mem_addr_start, &app_reg->pom0_mem_addr_start);
	writel(app_reg->in1_mem_addr_start, &app_reg->pom1_mem_addr_start);
	writel(app_reg->in_io_addr_start, &app_reg->pom_io_addr_start);

	writel(0xFFFFFFF0, &app_reg->mem0_addr_offset_limit);
	writel(0x0, &app_reg->pim0_mem_addr_start);
	writel(0x0, &app_reg->pim1_mem_addr_start);
	writel(0x0, &app_reg->pim_io_addr_start);
	writel(0x0, &app_reg->pim_io_addr_start);
	writel(0x0, &app_reg->pim_rom_addr_start);

	writel(DEVICE_TYPE_RC | (1 << MISCTRL_EN_ID)
			| (1 << APP_LTSSM_ENABLE_ID)
			| ((u32)1 << REG_TRANSLATION_ENABLE),
			&app_reg->app_ctrl_0);
}

static void __init spear13xx_pcie_preinit(void)
{
	int i;

	for (i = 0; i < NUM_PCIE_PORTS; i++) {
		struct pcie_port *pp = pcie_port + i;
		struct pcie_app_reg *app_reg =
			(struct pcie_app_reg *) (pp->va_app_base);

		snprintf(pp->mem_space_name, sizeof(pp->mem_space_name),
			"PCIe %d MEM", pp->port);
		pp->mem_space_name[sizeof(pp->mem_space_name) - 1] = 0;
		pp->res[0].name = pp->mem_space_name;
		pp->res[0].start = app_reg->in0_mem_addr_start;
		pp->res[0].end = app_reg->in0_mem_addr_limit;
		pp->res[0].flags = IORESOURCE_MEM;

		snprintf(pp->io_space_name, sizeof(pp->io_space_name),
			"PCIe %d I/O", pp->port);
		pp->io_space_name[sizeof(pp->io_space_name) - 1] = 0;
		pp->res[1].name = pp->io_space_name;
		pp->res[1].start = app_reg->in_io_addr_start;
		pp->res[1].end = app_reg->in_io_addr_limit;
		pp->res[1].flags = IORESOURCE_IO;

		if (request_resource(&iomem_resource, &pp->res[0]))
			panic("can't allocate PCIe I/O space");
		if (request_resource(&iomem_resource, &pp->res[1]))
			panic("can't allocate PCIe MEM space");
	}
}

static int __init spear13xx_pcie_setup(int nr, struct pci_sys_data *sys)
{
	struct pcie_port *pp;

	if (nr >= NUM_PCIE_PORTS)
		return 0;

	pp = &pcie_port[nr];
	if (!spear13xx_pcie_link_up((void __iomem *)pp->va_app_base))
		return 0;
	pp->root_bus_nr = sys->busnr;

	/*Generic PCIe unit setup.*/

	/*Need to come back here*/

	sys->resource[0] = &pp->res[0];
	sys->resource[1] = &pp->res[1];
	sys->resource[2] = NULL;

	return 1;
}

static struct pcie_port *bus_to_port(int bus)
{
	int i;

	for (i = NUM_PCIE_PORTS - 1; i >= 0; i--) {
		int rbus = pcie_port[i].root_bus_nr;
		if (rbus != -1 && rbus <= bus)
			break;
	}

	return i >= 0 ? pcie_port + i : NULL;
}

static int pcie_valid_config(struct pcie_port *pp, int bus, int dev)
{
	/*If there is no link, then there is no device*/
	if (!spear13xx_pcie_link_up((void __iomem *)pp->va_app_base))
		return 0;
	/*
	 * Don't go out when trying to access nonexisting devices
	 * on the local bus.
	 * we have only one slot on each root port.
	 */
	if (bus == pp->root_bus_nr && dev > 0)
		return 0;
	return 1;
}

static int spear13xx_pcie_rd_conf(struct pcie_port *pp, struct pci_bus *bus,
		u32 devfn, int where, int size, u32 *val)
{
	struct pcie_app_reg *app_reg = (struct pcie_app_reg *) pp->va_app_base;
	u32 *va_address;
	u32 address = readl(&app_reg->in_cfg0_addr_start)
			| (PCI_FUNC(devfn) << 16)
			| (where & 0xFFFC);

	writel((bus->number << 24) | (PCI_SLOT(devfn) << 19),
			&app_reg->pom_cfg0_addr_start);
	writel(readl(&app_reg->slv_armisc) & ~(AXI_OP_TYPE_MASK),
			&app_reg->slv_armisc);
	writel(readl(&app_reg->slv_armisc) | AXI_OP_TYPE_CONFIG_RDRW_TYPE0,
			&app_reg->slv_armisc);
	va_address = ioremap(address, 4);
	if (!va_address) {
		pr_err("error with ioremap in function %s\n", __func__);
		return -ENOMEM;
	}

	*val = readl(va_address);
	if (size == 1)
		*val = (*val >> (8 * (where & 3))) & 0xff;
	else if (size == 2)
		*val = (*val >> (8 * (where & 3))) & 0xffff;

	iounmap((void *)va_address);
	writel(readl(&app_reg->slv_armisc) & ~(AXI_OP_TYPE_MASK),
			&app_reg->slv_armisc);

	return PCIBIOS_SUCCESSFUL;
}

static int pcie_rd_conf(struct pci_bus *bus, u32 devfn, int where,
			int size, u32 *val)
{
	struct pcie_port *pp = bus_to_port(bus->number);
	unsigned long flags;
	int ret;

	if (pcie_valid_config(pp, bus->number, PCI_SLOT(devfn)) == 0) {
		*val = 0xffffffff;
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	spin_lock_irqsave(&pp->conf_lock, flags);
	ret = spear13xx_pcie_rd_conf(pp, bus, devfn, where, size, val);
	spin_unlock_irqrestore(&pp->conf_lock, flags);

	return ret;
}

static int spear13xx_pcie_wr_conf(struct pcie_port *pp, struct pci_bus *bus,
		u32 devfn, int where, int size, u32 val)
{
	int ret = PCIBIOS_SUCCESSFUL;
	struct pcie_app_reg *app_reg = (struct pcie_app_reg *) pp->va_app_base;
	u32 *va_address;
	u32 address = readl(&app_reg->in_cfg0_addr_start)
			| (PCI_FUNC(devfn) << 16)
			| (where & 0xFFFC);

	writel((bus->number << 24) | (PCI_SLOT(devfn) << 19),
			&app_reg->pom_cfg0_addr_start);
	writel(readl(&app_reg->slv_awmisc) & ~(AXI_OP_TYPE_MASK),
			&app_reg->slv_awmisc);
	writel(readl(&app_reg->slv_awmisc) | AXI_OP_TYPE_CONFIG_RDRW_TYPE0,
			&app_reg->slv_awmisc);
	va_address = ioremap(address, 4);
	if (!va_address) {
		pr_err("error with ioremap in function %s\n", __func__);
		return -ENOMEM;
	}

	if (size == 4)
		writel(val, va_address);
	else if (size == 2)
		writew(val, va_address + (where & 2));
	else if (size == 1)
		writeb(val, va_address + (where & 3));
	else
		ret = PCIBIOS_BAD_REGISTER_NUMBER;

	iounmap((void *)va_address);
	writel(readl(&app_reg->slv_awmisc) & ~(AXI_OP_TYPE_MASK),
			&app_reg->slv_awmisc);
	return ret;
}

static int pcie_wr_conf(struct pci_bus *bus, u32 devfn,
			int where, int size, u32 val)
{
	struct pcie_port *pp = bus_to_port(bus->number);
	unsigned long flags;
	int ret;

	if (pcie_valid_config(pp, bus->number, PCI_SLOT(devfn)) == 0)
		return PCIBIOS_DEVICE_NOT_FOUND;

	spin_lock_irqsave(&pp->conf_lock, flags);
	ret = spear13xx_pcie_wr_conf(pp, bus, devfn, where, size, val);
	spin_unlock_irqrestore(&pp->conf_lock, flags);

	return ret;
}

static struct pci_ops pcie_ops = {
	.read = pcie_rd_conf,
	.write = pcie_wr_conf,
};

static struct pci_bus __init *
spear13xx_pcie_scan_bus(int nr, struct pci_sys_data *sys)
{
	struct pci_bus *bus;

	if (nr < NUM_PCIE_PORTS) {
		bus = pci_scan_bus(sys->busnr, &pcie_ops, sys);
	} else {
		bus = NULL;
		BUG();
	}

	return bus;
}

static int __init spear13xx_pcie_map_irq(struct pci_dev *dev, u8 slot, u8 pin)
{
	struct pcie_port *pp = bus_to_port(dev->bus->number);

	return IRQ_PCIE0 + pp->port;
}

static struct hw_pci spear13xx_pci __initdata = {
	.nr_controllers	= NUM_PCIE_PORTS,
	.preinit	= spear13xx_pcie_preinit,
	.swizzle	= pci_std_swizzle,
	.setup		= spear13xx_pcie_setup,
	.scan		= spear13xx_pcie_scan_bus,
	.map_irq	= spear13xx_pcie_map_irq,
};

static void __init add_pcie_port(int port, u32 base, u32 app_base)
{
	struct pcie_port *pp = &pcie_port[port];

	pp->port = port;
	pp->root_bus_nr = -1;
	pp->base = (void __iomem *)base;
	pp->app_base = (void __iomem *)app_base;
	pp->va_app_base = (void __iomem *) ioremap(app_base, 0x200);
	if (!pp->va_app_base) {
		pr_err("error with ioremap in function %s\n", __func__);
		return;
	}
	spin_lock_init(&pp->conf_lock);
	memset(pp->res, 0, sizeof(pp->res));
	pr_info("spear13xx PCIe port %d\n", port);
	if (spear13xx_pcie_link_up((void __iomem *)pp->va_app_base)) {
		pr_info("link up in bios\n");
	} else {
		pr_info("link down in bios\n");
		spear13xx_pcie_host_init(pp->va_app_base, base);
	}
}

static int __init spear13xx_pcie_init(void)
{
	int port;
	char port_name[20];
	struct clk *clk;

	/*
	* enable core clock in PCIE_CFG MISC register
	* for all three controller
	* need to find a better way to do it (TBD)
	*/
	writel(0xFFF, PCIE_CFG);
	for (port = 0; port < NUM_PCIE_PORTS; port++) {
		snprintf(port_name, sizeof(port_name), "pcie%d", port);
		port_name[sizeof(port_name) - 1] = 0;
		clk = clk_get_sys(port_name, NULL);
		if (!clk) {
			pr_err("%s:couldn't get clk for pcie%d\n",
					__func__, port);
			continue;
		}
		if (clk_enable(clk)) {
			pr_err("%s:couldn't enable clk for pcie%d\n",
					__func__, port);
			continue;
		}
		add_pcie_port(port,
				spr_pcie_base[port],
				spr_pcie_app_base[port]);
	}

	pci_common_init(&spear13xx_pci);

	return 0;
}
subsys_initcall(spear13xx_pcie_init);
