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
#include <linux/pci_regs.h>
#include <linux/msi.h>
#include <linux/mbus.h>
#include <linux/sched.h>
#include <asm/irq.h>
#include <asm/mach/pci.h>
#include <asm/mach/irq.h>
#include <mach/pcie.h>
#include <mach/irqs.h>
#include <mach/misc_regs.h>

#define NUM_PCIE_PORTS	3

/* Sum of all these space can maximum be 256MB*/
#define IN0_MEM_SIZE	(200 * 1024 * 1024 - 1)
/* In current implementation address translation is done using IN0 only.
 * So IN1 start address and IN0 end address has been kept same
*/
#define IN1_MEM_SIZE	(0 * 1024 * 1024 - 1)
#define IN_IO_SIZE	(20 * 1024 * 1024 - 1)
#define IN_CFG0_SIZE	(1 * 1024 * 1024 - 1)
#define IN_CFG1_SIZE	(1 * 1024 * 1024 - 1)
#define IN_MSG_SIZE	(1 * 1024 * 1024 - 1)

#define MAX_LINK_UP_WAIT_JIFFIES	10

int (*pcie_port_is_host)(int port);
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

/* Keeping all DDR area of 256MB accesible for inbound transaction */
#define INBOUND_ADDR_MASK	0xFFFFFFF

#ifdef CONFIG_PCI_MSI
static DECLARE_BITMAP(msi_irq_in_use[NUM_PCIE_PORTS], SPEAR_NUM_MSI_IRQS);
static unsigned int spear_msi_data[NUM_PCIE_PORTS];

static void spear13xx_msi_init(struct pcie_port *pp);
#endif

static void spear_pcie_int_handler(unsigned int irq, struct irq_desc *desc);

static void enable_dbi_access(struct pcie_app_reg *app_reg)
{
	/* Enable DBI access */
	writel(readl(&app_reg->slv_armisc) | (1 << AXI_OP_DBI_ACCESS_ID),
			&app_reg->slv_armisc);
	writel(readl(&app_reg->slv_awmisc) | (1 << AXI_OP_DBI_ACCESS_ID),
			&app_reg->slv_awmisc);

}

static void disable_dbi_access(struct pcie_app_reg *app_reg)
{
	/* disable DBI access */
	writel(readl(&app_reg->slv_armisc) & ~(1 << AXI_OP_DBI_ACCESS_ID),
			&app_reg->slv_armisc);
	writel(readl(&app_reg->slv_awmisc) & ~(1 << AXI_OP_DBI_ACCESS_ID),
			&app_reg->slv_awmisc);

}

static void spear_dbi_read_reg(struct pcie_port *pp, int where, int size,
		u32 *val)
{
	struct pcie_app_reg *app_reg = (struct pcie_app_reg *) pp->va_app_base;
	u32 va_address;

	/* Enable DBI access */
	enable_dbi_access(app_reg);

	va_address = (u32)pp->va_dbi_base + (where & ~0x3);

	*val = readl(va_address);

	if (size == 1)
		*val = (*val >> (8 * (where & 3))) & 0xff;
	else if (size == 2)
		*val = (*val >> (8 * (where & 3))) & 0xffff;

	/* Disable DBI access */
	disable_dbi_access(app_reg);
}

static void spear_dbi_write_reg(struct pcie_port *pp, int where, int size,
		u32 val)
{
	struct pcie_app_reg *app_reg = (struct pcie_app_reg *) pp->va_app_base;
	u32 va_address;

	/* Enable DBI access */
	enable_dbi_access(app_reg);

	va_address = (u32)pp->va_dbi_base + (where & ~0x3);

	if (size == 4)
		writel(val, va_address);
	else if (size == 2)
		writew(val, va_address + (where & 2));
	else if (size == 1)
		writeb(val, va_address + (where & 3));

	/* Disable DBI access */
	disable_dbi_access(app_reg);
}

#define PCI_FIND_CAP_TTL	48

static int pci_find_own_next_cap_ttl(struct pcie_port *pp,
		u32 pos, int cap, int *ttl)
{
	u32 id;

	while ((*ttl)--) {
		spear_dbi_read_reg(pp, pos, 1, &pos);
		if (pos < 0x40)
			break;
		pos &= ~3;
		spear_dbi_read_reg(pp, pos + PCI_CAP_LIST_ID, 1, &id);
		if (id == 0xff)
			break;
		if (id == cap)
			return pos;
		pos += PCI_CAP_LIST_NEXT;
	}
	return 0;
}

static int pci_find_own_next_cap(struct pcie_port *pp, u32 pos, int cap)
{
	int ttl = PCI_FIND_CAP_TTL;

	return pci_find_own_next_cap_ttl(pp, pos, cap, &ttl);
}

static int pci_find_own_cap_start(struct pcie_port *pp, u8 hdr_type)
{
	u32 status;

	spear_dbi_read_reg(pp, PCI_STATUS, 2, &status);
	if (!(status & PCI_STATUS_CAP_LIST))
		return 0;

	switch (hdr_type) {
	case PCI_HEADER_TYPE_NORMAL:
	case PCI_HEADER_TYPE_BRIDGE:
		return PCI_CAPABILITY_LIST;
	case PCI_HEADER_TYPE_CARDBUS:
		return PCI_CB_CAPABILITY_LIST;
	default:
		return 0;
	}

	return 0;
}

/**
 * Tell if a device supports a given PCI capability.
 * Returns the address of the requested capability structure within the
 * device's PCI configuration space or 0 in case the device does not
 * support it. Possible values for @cap:
 *
 * %PCI_CAP_ID_PM	Power Management
 * %PCI_CAP_ID_AGP	Accelerated Graphics Port
 * %PCI_CAP_ID_VPD	Vital Product Data
 * %PCI_CAP_ID_SLOTID	Slot Identification
 * %PCI_CAP_ID_MSI	Message Signalled Interrupts
 * %PCI_CAP_ID_CHSWP	CompactPCI HotSwap
 * %PCI_CAP_ID_PCIX	PCI-X
 * %PCI_CAP_ID_EXP	PCI Express
 */
static int pci_find_own_capability(struct pcie_port *pp, int cap)
{
	u32 pos;
	u32 hdr_type;

	spear_dbi_read_reg(pp, PCI_HEADER_TYPE, 1, &hdr_type);

	pos = pci_find_own_cap_start(pp, hdr_type);
	if (pos)
		pos = pci_find_own_next_cap(pp, pos, cap);

	return pos;
}

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

static void spear13xx_pcie_host_init(struct pcie_port *pp)
{
	struct pcie_app_reg *app_reg = (struct pcie_app_reg *)pp->va_app_base;
	u32 cap, val;

	/*setup registers for outbound translation */

	writel(pp->base, &app_reg->in0_mem_addr_start);
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

	/*setup registers for inbound translation */

	writel(INBOUND_ADDR_MASK + 1, &app_reg->mem0_addr_offset_limit);
	writel(0, &app_reg->pim0_mem_addr_start);
	writel(0, &app_reg->pim1_mem_addr_start);
	spear_dbi_write_reg(pp, PCIE_BAR0_MASK_REG, 4, INBOUND_ADDR_MASK);
	spear_dbi_write_reg(pp, PCI_BASE_ADDRESS_0, 4, 0);

	writel(0x0, &app_reg->pim_io_addr_start);
	writel(0x0, &app_reg->pim_io_addr_start);
	writel(0x0, &app_reg->pim_rom_addr_start);

	cap = pci_find_own_capability(pp, PCI_CAP_ID_EXP);
	/*this controller support only 128 bytes read size, however its
	 * default value in capability register is 512 bytes. So force
	 * it to 128 here */

	spear_dbi_read_reg(pp, cap + PCI_EXP_DEVCTL, 4, &val);
	val &= ~PCI_EXP_DEVCTL_READRQ;
	spear_dbi_write_reg(pp, cap + PCI_EXP_DEVCTL, 4, val);

	writel(DEVICE_TYPE_RC | (1 << MISCTRL_EN_ID)
			| (1 << APP_LTSSM_ENABLE_ID)
			| ((u32)1 << REG_TRANSLATION_ENABLE),
			&app_reg->app_ctrl_0);
}

static void __init spear13xx_pcie_preinit(void)
{
	int i;
	struct pcie_port *pp;
	struct pcie_app_reg *app_reg;

	for (i = 0; i < NUM_PCIE_PORTS; i++) {
		pp = pcie_port + i;
		app_reg = (struct pcie_app_reg *) (pp->va_app_base);

		/* init hosts only */
		if ((*pcie_port_is_host)(i) != 1)
			continue;
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
			panic("can't allocate PCIe Mem space");
		if (request_resource(&ioport_resource, &pp->res[1]))
			panic("can't allocate PCIe IO space");
	}
}

static struct hw_pci spear13xx_pci;
static struct pcie_port *bus_to_port(int bus);

static int pcie_get_payload(struct pci_dev *dev)
{
	int ret, cap;
	u16 ctl;

	cap = pci_find_capability(dev, PCI_CAP_ID_EXP);
	if (!cap)
		return -EINVAL;

	ret = pci_read_config_word(dev, cap + PCI_EXP_DEVCTL, &ctl);
	if (!ret)
		ret = 128 << ((ctl & PCI_EXP_DEVCTL_PAYLOAD) >> 5);

	return ret;
}

static int pcie_set_payload(struct pci_dev *dev, int rq)
{
	int cap, err = -EINVAL;
	u16 ctl, v;

	if (rq < 128 || rq > 4096 || !is_power_of_2(rq))
		goto out;

	v = (ffs(rq) - 8) << 5;

	cap = pci_find_capability(dev, PCI_CAP_ID_EXP);
	if (!cap)
		goto out;

	err = pci_read_config_word(dev, cap + PCI_EXP_DEVCTL, &ctl);
	if (err)
		goto out;

	if ((ctl & PCI_EXP_DEVCTL_PAYLOAD) != v) {
		ctl &= ~PCI_EXP_DEVCTL_PAYLOAD;
		ctl |= v;
		err = pci_write_config_dword(dev, cap + PCI_EXP_DEVCTL, ctl);
	}

out:
	return err;
}
static void __init spear13xx_pcie_postinit(void)
{
	struct hw_pci *hw = &spear13xx_pci;
	struct pci_sys_data *sys;
	struct pci_dev *dev;
	int cap, ctl, payload;
	int max_payload = 4096;
	struct pci_bus *bus;
	struct pcie_port *pp;

	/* allign Max_Payload_Size for all devices to the minimum
	 * Max_Payload_Size of any of the device in tree.
	 * Max_Read_Request_Size of any of the DS device should be less
	 * than or equal to that of RC's Max_Read_Request_Size*/

	list_for_each_entry(sys, &hw->buses, node) {
		bus = sys->bus;
		pp = bus_to_port(bus->number);
		cap = pci_find_own_capability(pp, PCI_CAP_ID_EXP);
		spear_dbi_read_reg(pp, cap + PCI_EXP_DEVCTL, 2, &ctl);
		payload = 128 << ((ctl & PCI_EXP_DEVCTL_PAYLOAD) >> 5);
		if (payload < max_payload)
			max_payload = payload;
		ctl = 128 << ((ctl & PCI_EXP_DEVCTL_READRQ) >> 12);

		list_for_each_entry(dev, &bus->devices, bus_list) {
			if (ctl < pcie_get_readrq(dev))
				pcie_set_readrq(dev, ctl);
			payload = pcie_get_payload(dev);
			if (payload < max_payload)
				max_payload = payload;
		}
	}
	/* now set max_payload for all devices */
	list_for_each_entry(sys, &hw->buses, node) {
		bus = sys->bus;
		pp = bus_to_port(bus->number);
		cap = pci_find_own_capability(pp, PCI_CAP_ID_EXP);
		spear_dbi_read_reg(pp, cap + PCI_EXP_DEVCTL, 2, &ctl);
		payload = (ffs(max_payload) - 8) << 5;
		if ((ctl & PCI_EXP_DEVCTL_PAYLOAD) != max_payload) {
			ctl &= ~PCI_EXP_DEVCTL_PAYLOAD;
			ctl |= payload;
			spear_dbi_write_reg(pp, cap + PCI_EXP_DEVCTL, 2, ctl);
		}
		list_for_each_entry(dev, &bus->devices, bus_list)
			pcie_set_payload(dev, max_payload);
	}
}

static int __init spear13xx_pcie_setup(int nr, struct pci_sys_data *sys)
{
	struct pcie_port *pp;
	u32 val = 0;

	if (nr >= NUM_PCIE_PORTS)
		return 0;

	if ((*pcie_port_is_host)(nr) != 1)
		return 0;

	pp = &pcie_port[nr];
	if (!spear13xx_pcie_link_up((void __iomem *)pp->va_app_base))
		return 0;
	pp->root_bus_nr = sys->busnr;

	/* Generic PCIe unit setup.*/

	/* Enable own BME. It is necessary to enable own BME to do a
	 * memory transaction on a downstream device
	 */
	spear_dbi_read_reg(pp, PCI_COMMAND, 2, &val);
	val |= (PCI_COMMAND_IO | PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER
			| PCI_COMMAND_PARITY | PCI_COMMAND_SERR);
	spear_dbi_write_reg(pp, PCI_COMMAND, 2, val);

	/* Need to come back here*/

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
		if ((*pcie_port_is_host)(i) != 1)
			continue;
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
	u32 address = (u32)pp->va_cfg0_base | (PCI_FUNC(devfn) << 16)
		| (where & 0xFFFC);

	writel((bus->number << 24) | (PCI_SLOT(devfn) << 19),
			&app_reg->pom_cfg0_addr_start);
	writel(readl(&app_reg->slv_armisc) & ~(AXI_OP_TYPE_MASK),
			&app_reg->slv_armisc);
	writel(readl(&app_reg->slv_armisc) | AXI_OP_TYPE_CONFIG_RDRW_TYPE0,
			&app_reg->slv_armisc);

	*val = readl(address);
	if (size == 1)
		*val = (*val >> (8 * (where & 3))) & 0xff;
	else if (size == 2)
		*val = (*val >> (8 * (where & 3))) & 0xffff;

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
	u32 address = (u32)pp->va_cfg0_base | (PCI_FUNC(devfn) << 16)
			| (where & 0xFFFC);

	writel((bus->number << 24) | (PCI_SLOT(devfn) << 19),
			&app_reg->pom_cfg0_addr_start);
	writel(readl(&app_reg->slv_awmisc) & ~(AXI_OP_TYPE_MASK),
			&app_reg->slv_awmisc);
	writel(readl(&app_reg->slv_awmisc) | AXI_OP_TYPE_CONFIG_RDRW_TYPE0,
			&app_reg->slv_awmisc);
	if (size == 4)
		writel(val, address);
	else if (size == 2)
		writew(val, address + (where & 2));
	else if (size == 1)
		writeb(val, address + (where & 3));
	else
		ret = PCIBIOS_BAD_REGISTER_NUMBER;
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

	if ((nr < NUM_PCIE_PORTS) && ((*pcie_port_is_host)(nr)) == 1) {
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
	int irq = (SPEAR_INTX0_BASE + pp->port * SPEAR_NUM_INTX_IRQS + pin - 1);

	return irq;
}

static struct hw_pci spear13xx_pci __initdata = {
	.nr_controllers	= NUM_PCIE_PORTS,
	.preinit	= spear13xx_pcie_preinit,
	.postinit	= spear13xx_pcie_postinit,
	.swizzle	= pci_std_swizzle,
	.setup		= spear13xx_pcie_setup,
	.scan		= spear13xx_pcie_scan_bus,
	.map_irq	= spear13xx_pcie_map_irq,
};

void mask_intx_irq(unsigned int irq)
{
	int irq_offset = (irq - SPEAR_INTX0_BASE) % SPEAR_NUM_INTX_IRQS;
	int port = (irq - SPEAR_INTX0_BASE) / SPEAR_NUM_INTX_IRQS;
	struct pcie_port *pp = &pcie_port[port];
	struct pcie_app_reg *app_reg = (struct pcie_app_reg *)pp->va_app_base;

	switch (irq_offset) {
	case 0:
		writel(readl(&app_reg->int_mask) & ~INTA_CTRL_INT,
				&app_reg->int_mask);
		break;
	case 1:
		writel(readl(&app_reg->int_mask) & ~INTB_CTRL_INT,
				&app_reg->int_mask);
		break;
	case 2:
		writel(readl(&app_reg->int_mask) & ~INTC_CTRL_INT,
				&app_reg->int_mask);
		break;
	case 3:
		writel(readl(&app_reg->int_mask) & ~INTD_CTRL_INT,
				&app_reg->int_mask);
		break;
	}
}

void unmask_intx_irq(unsigned int irq)
{
	int irq_offset = (irq - SPEAR_INTX0_BASE) % SPEAR_NUM_INTX_IRQS;
	int port = (irq - SPEAR_INTX0_BASE) / SPEAR_NUM_INTX_IRQS;
	struct pcie_port *pp = &pcie_port[port];
	struct pcie_app_reg *app_reg = (struct pcie_app_reg *)pp->va_app_base;

	switch (irq_offset) {
	case 0:
		writel(readl(&app_reg->int_mask) | INTA_CTRL_INT,
				&app_reg->int_mask);
		break;
	case 1:
		writel(readl(&app_reg->int_mask) | INTB_CTRL_INT,
				&app_reg->int_mask);
		break;
	case 2:
		writel(readl(&app_reg->int_mask) | INTC_CTRL_INT,
				&app_reg->int_mask);
		break;
	case 3:
		writel(readl(&app_reg->int_mask) | INTD_CTRL_INT,
				&app_reg->int_mask);
		break;
	}
}

static struct irq_chip spear13xx_intx_chip = {
	.name = "PCI-INTX",
	.mask = mask_intx_irq,
	.unmask = unmask_intx_irq,
};

static void spear13xx_int_init(struct pcie_port *pp)
{
	int i, irq;
	struct pcie_app_reg *app_reg;

	set_irq_chained_handler(IRQ_PCIE0 + pp->port, spear_pcie_int_handler);

#ifdef CONFIG_PCI_MSI
	spear13xx_msi_init(pp);
#endif
	/* Enbale INTX interrupt*/
	app_reg = (struct pcie_app_reg *)pp->va_app_base;
	writel(readl(&app_reg->int_mask) | INTA_CTRL_INT
			| INTB_CTRL_INT	| INTC_CTRL_INT
			| INTD_CTRL_INT, &app_reg->int_mask);

	/* initilize INTX chip here only. MSI chip will be
	 * initilized dynamically.*/
	irq = (SPEAR_INTX0_BASE + pp->port * SPEAR_NUM_INTX_IRQS);
	for (i = 0; i < SPEAR_NUM_INTX_IRQS; i++) {
		set_irq_chip_and_handler(irq + i, &spear13xx_intx_chip,
				handle_simple_irq);
		set_irq_flags(irq + i, IRQF_VALID);
	}
}

static void __init add_pcie_port(int port, u32 base, u32 app_base)
{
	struct pcie_port *pp = &pcie_port[port];
	struct pcie_app_reg *app_reg;

	pp->port = port;
	pp->root_bus_nr = -1;
	pp->base = (void __iomem *)base;
	pp->app_base = (void __iomem *)app_base;
	pp->va_app_base = (void __iomem *) ioremap(app_base, 0x200);
	if (!pp->va_app_base) {
		pr_err("error with ioremap in function %s\n", __func__);
		return;
	}
	pp->va_dbi_base = (void __iomem *) ioremap(base, 0x2000);
	if (!pp->va_dbi_base) {
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
		spear13xx_pcie_host_init(pp);
		spear13xx_int_init(pp);
		app_reg = (struct pcie_app_reg *)pp->va_app_base;
		pp->va_cfg0_base = (void __iomem *)
			ioremap(app_reg->in_cfg0_addr_start, IN_CFG0_SIZE);
		if (!pp->va_cfg0_base) {
			pr_err("error with ioremap in function %s\n", __func__);
			return;
		}

	}
}

static int __init spear13xx_pcie_init(void)
{
	int port;
	struct clk *clk;

	for (port = 0; port < NUM_PCIE_PORTS; port++) {
		/* do not enable clock if it is PCIE0. Ideally , all controller
		 * should have been independent from others with respect to
		 * clock. But PCIE1 and 2 depends on PCIE0.So PCIE0 clk
		 * is provided during board init.*/
		if (port == 1) {
			/* Ideally CFG Clock should have been also enabled
			 * here. But it is done currently during board
			 * init routne*/
			clk = clk_get_sys("pcie1", NULL);
			if (IS_ERR(clk)) {
				pr_err("%s:couldn't get clk for pcie1\n",
						__func__);
				continue;
			}
			if (clk_enable(clk)) {
				pr_err("%s:couldn't enable clk for pcie1\n",
						__func__);
				continue;
			}
		} else if (port == 2) {
			/* Ideally CFG Clock should have been also enabled
			 * here. But it is done currently during board
			 * init routne*/
			clk = clk_get_sys("pcie2", NULL);
			if (IS_ERR(clk)) {
				pr_err("%s:couldn't get clk for pcie2\n",
						__func__);
				continue;
			}
			if (clk_enable(clk)) {
				pr_err("%s:couldn't enable clk for pcie2\n",
						__func__);
				continue;
			}
		}

		if ((*pcie_port_is_host)(port) == 1)
			add_pcie_port(port, spr_pcie_base[port],
					spr_pcie_app_base[port]);
	}

	pci_common_init(&spear13xx_pci);

	return 0;
}
subsys_initcall(spear13xx_pcie_init);

#ifdef CONFIG_PCI_MSI
/* MSI int handler
 */
static void handle_msi(struct pcie_port *pp)
{
	unsigned long val;
	int i, pos;

	for (i = 0; i < 8; i++) {
		spear_dbi_read_reg(pp, PCIE_MSI_INTR0_STATUS + i * 12, 4,
				(u32 *)&val);
		if (val) {
			pos = 0;
			while ((pos = find_next_bit(&val, 32, pos)) != 32) {
				generic_handle_irq(SPEAR_MSI0_INT_BASE
					+ pp->port * SPEAR_NUM_MSI_IRQS
					+ (i * 32) + pos);
				pos++;
			}
		}
		spear_dbi_write_reg(pp, PCIE_MSI_INTR0_STATUS + i * 12, 4, val);
	}
}
#else
static void handle_msi(struct pcie_port *pp)
{
}
#endif

static void spear_pcie_int_handler(unsigned int irq, struct irq_desc *desc)
{
	struct pcie_port *pp = &pcie_port[irq - IRQ_PCIE0];
	struct pcie_app_reg *app_reg = (struct pcie_app_reg *)pp->va_app_base;
	unsigned int status;

	status = readl(&app_reg->int_sts);

	desc->chip->ack(irq);

	if (status & MSI_CTRL_INT) {
		handle_msi(pp);
		writel(MSI_CTRL_INT, &app_reg->int_clr);
	} else if (status & INTA_CTRL_INT)
		generic_handle_irq(SPEAR_INTX0_BASE
				+ pp->port * SPEAR_NUM_INTX_IRQS);
	else if (status & INTB_CTRL_INT)
		generic_handle_irq(SPEAR_INTX0_BASE
				+ pp->port * SPEAR_NUM_INTX_IRQS + 1);
	else if (status & INTC_CTRL_INT)
		generic_handle_irq(SPEAR_INTX0_BASE
				+ pp->port * SPEAR_NUM_INTX_IRQS + 2);
	else if (status & INTD_CTRL_INT)
		generic_handle_irq(SPEAR_INTX0_BASE
				+ pp->port * SPEAR_NUM_INTX_IRQS + 3);
	else
		writel(status, &app_reg->int_clr);

	desc->chip->unmask(irq);
}

#ifdef CONFIG_PCI_MSI
static int find_valid_pos0(int port, int nvec, int pos, int *pos0)
{
	int flag = 1;
	do {
		pos = find_next_zero_bit(msi_irq_in_use[port],
				SPEAR_NUM_MSI_IRQS, pos);
		/*if you have reached to the end then get out from here.*/
		if (pos == SPEAR_NUM_MSI_IRQS)
			return -ENOSPC;
		/* Check if this position is at correct offset.nvec is always a
		 * power of two. pos0 must be nvec bit alligned.
		 */
		if (pos % nvec)
			pos += nvec - (pos % nvec);
		else
			flag = 0;
	} while (flag);

	*pos0 = pos;
	return 0;
}

static void spear13xx_msi_nop(unsigned int irq)
{
	return;
}

static struct irq_chip spear13xx_msi_chip = {
	.name = "PCI-MSI",
	.ack = spear13xx_msi_nop,
	.irq_enable = unmask_msi_irq,
	.irq_disable = mask_msi_irq,
	.irq_mask = mask_msi_irq,
	.irq_unmask = unmask_msi_irq,
};

/*
 * Dynamic irq allocate and deallocation
 */
static int get_irq(int nvec, struct msi_desc *desc, int *pos)
{
	int res, bit, irq, pos0, pos1, i;
	u32 val;
	struct pcie_port *pp = bus_to_port(desc->dev->bus->number);

	pos0 = find_first_zero_bit(msi_irq_in_use[pp->port],
			SPEAR_NUM_MSI_IRQS);
	if (pos0 % nvec) {
		if (find_valid_pos0(pp->port, nvec, pos0, &pos0))
			goto no_valid_irq;
	}
	if (nvec > 1) {
		pos1 = find_next_bit(msi_irq_in_use[pp->port],
				SPEAR_NUM_MSI_IRQS, pos0);
		/* there must be nvec number of consecutive free bits */
		while ((pos1 - pos0) < nvec) {
			if (find_valid_pos0(pp->port, nvec, pos1, &pos0))
				goto no_valid_irq;
			pos1 = find_next_bit(msi_irq_in_use[pp->port],
					SPEAR_NUM_MSI_IRQS, pos0);
		}
	}

	irq = (SPEAR_MSI0_INT_BASE + (pp->port * SPEAR_NUM_MSI_IRQS)) + pos0;

	if ((irq + nvec) > (SPEAR_MSI0_INT_END
				+ (pp->port * SPEAR_NUM_MSI_IRQS)))
		goto no_valid_irq;

	i = 0;
	while (i < nvec) {
		set_bit(pos0 + i, msi_irq_in_use[pp->port]);
		dynamic_irq_init(irq + i);
		set_irq_msi(irq + i, desc);
		set_irq_chip_and_handler(irq + i, &spear13xx_msi_chip,
				handle_simple_irq);

		/* Enable corresponding interrupt on MSI interrupt
		 * controller.
		 */
		res = ((pos0 + i) / 32) * 12;
		bit = (pos0 + i) % 32;
		spear_dbi_read_reg(pp, PCIE_MSI_INTR0_ENABLE + res, 4, &val);
		val |= 1 << bit;
		spear_dbi_write_reg(pp, PCIE_MSI_INTR0_ENABLE + res, 4, val);

		i++;
	}

	*pos = pos0;
	return irq;
no_valid_irq:
	*pos = pos0;
	return -ENOSPC;
}

static void clean_irq(unsigned int irq)
{
	int res, bit, val, pos;
	struct irq_desc *desc = irq_to_desc(irq);
	struct pcie_port *pp = bus_to_port(desc->msi_desc->dev->bus->number);

	pos = irq - (SPEAR_MSI0_INT_BASE + (pp->port * SPEAR_NUM_MSI_IRQS));

	dynamic_irq_cleanup(irq);

	clear_bit(pos, msi_irq_in_use[pp->port]);

	/* Disable corresponding interrupt on MSI interrupt
	 * controller.
	 */
	res = (pos / 32) * 12;
	bit = pos % 32;
	spear_dbi_read_reg(pp, PCIE_MSI_INTR0_ENABLE + res, 4, &val);
	val &= ~(1 << bit);
	spear_dbi_write_reg(pp, PCIE_MSI_INTR0_ENABLE + res, 4, val);

}

int arch_setup_msi_irq(struct pci_dev *pdev, struct msi_desc *desc)
{
	int cvec, rvec, irq, pos;
	struct msi_msg msg;
	uint16_t control;
	struct pcie_port *pp = bus_to_port(pdev->bus->number);

	/*
	 * Read the MSI config to figure out how many IRQs this device
	 * wants.Most devices only want 1, which will give
	 * configured_private_bits and request_private_bits equal 0.
	 */
	pci_read_config_word(pdev, desc->msi_attrib.pos + PCI_MSI_FLAGS,
			&control);

	/*
	 * If the number of private bits has been configured then use
	 * that value instead of the requested number. This gives the
	 * driver the chance to override the number of interrupts
	 * before calling pci_enable_msi().
	 */

	cvec = (control & PCI_MSI_FLAGS_QSIZE) >> 4;

	if (cvec == 0) {
		/* Nothing is configured, so use the hardware requested size */
		rvec = (control & PCI_MSI_FLAGS_QMASK) >> 1;
	} else {
		/*
		 * Use the number of configured bits, assuming the
		 * driver wanted to override the hardware request
		 * value.
		 */
		rvec = cvec;
	}

	/*
	 * The PCI 2.3 spec mandates that there are at most 32
	 * interrupts. If this device asks for more, only give it one.
	 */
	if (rvec > 5)
		rvec = 0;

	irq = get_irq((1 << rvec), desc, &pos);

	 if (irq < 0)
		return irq;

	 /* Update the number of IRQs the device has available to it */
	 control &= ~PCI_MSI_FLAGS_QSIZE;
	 control |= rvec << 4;
	 pci_write_config_word(pdev, desc->msi_attrib.pos + PCI_MSI_FLAGS,
			 control);
	 desc->msi_attrib.multiple = rvec;

	/* An EP will modify lower 8 bits(max) of msi data while
	 * sending any msi interrupt
	 */
	msg.address_hi = 0x0;
	msg.address_lo = __virt_to_phys((u32)(&spear_msi_data[pp->port]));
	msg.data = pos;
	write_msi_msg(irq, &msg);

	return 0;
}

void arch_teardown_msi_irq(unsigned int irq)
{
	clean_irq(irq);
}

static void spear13xx_msi_init(struct pcie_port *pp)
{
	struct pcie_app_reg *app_reg = (struct pcie_app_reg *)pp->va_app_base;

	spear_dbi_write_reg(pp, PCIE_MSI_ADDR_LO, 4,
			__virt_to_phys((u32)(&spear_msi_data[pp->port])));
	spear_dbi_write_reg(pp, PCIE_MSI_ADDR_HI, 4, 0);
	/* Enbale MSI interrupt*/
	writel(readl(&app_reg->int_mask) | MSI_CTRL_INT,
			&app_reg->int_mask);
}
#endif
