/*
 * drivers/misc/spear13xx_pcie_gadget.c
 *
 * Copyright (C) 2010 ST Microelectronics
 * Pratyush Anand<pratyush.anand@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/clk.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/pci_regs.h>
#include <mach/pcie.h>
#include <mach/misc_regs.h>

#define IN0_MEM_SIZE	(200 * 1024 * 1024 - 1)
/* In current implementation address translation is done using IN0 only.
 * So IN1 start address and IN0 end address has been kept same
*/
#define IN1_MEM_SIZE	(0 * 1024 * 1024 - 1)
#define IN_IO_SIZE	(20 * 1024 * 1024 - 1)
#define IN_CFG0_SIZE	(12 * 1024 * 1024 - 1)
#define IN_CFG1_SIZE	(12 * 1024 * 1024 - 1)
#define IN_MSG_SIZE	(12 * 1024 * 1024 - 1)
/* Keep default BAR size as 4K*/
/* AORAM would be mapped by default*/
#define INBOUND_ADDR_MASK	(SPEAR13XX_SYSRAM1_SIZE - 1)

#define INT_TYPE_NO_INT	0
#define INT_TYPE_INTX	1
#define INT_TYPE_MSI	2
struct spear_pcie_gadget_config {
	void __iomem *base;
	void __iomem *va_app_base;
	void __iomem *va_dbi_base;
	char int_type[10];
	u32 requested_msi;
	u32 configured_msi;
	u32 bar0_size;
	u32 bar0_rw_offset;
	u32 va_bar0_address;
};

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

static void spear_dbi_read_reg(struct spear_pcie_gadget_config *config,
		int where, int size, u32 *val)
{
	struct pcie_app_reg *app_reg
		= (struct pcie_app_reg *) config->va_app_base;
	u32 va_address;

	/* Enable DBI access */
	enable_dbi_access(app_reg);

	va_address = (u32)config->va_dbi_base + (where & ~0x3);

	*val = readl(va_address);

	if (size == 1)
		*val = (*val >> (8 * (where & 3))) & 0xff;
	else if (size == 2)
		*val = (*val >> (8 * (where & 3))) & 0xffff;

	/* Disable DBI access */
	disable_dbi_access(app_reg);
}

static void spear_dbi_write_reg(struct spear_pcie_gadget_config *config,
		int where, int size, u32 val)
{
	struct pcie_app_reg *app_reg
		= (struct pcie_app_reg *) config->va_app_base;
	u32 va_address;

	/* Enable DBI access */
	enable_dbi_access(app_reg);

	va_address = (u32)config->va_dbi_base + (where & ~0x3);

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

static int pci_find_own_next_cap_ttl(struct spear_pcie_gadget_config *config,
		u32 pos, int cap, int *ttl)
{
	u32 id;

	while ((*ttl)--) {
		spear_dbi_read_reg(config, pos, 1, &pos);
		if (pos < 0x40)
			break;
		pos &= ~3;
		spear_dbi_read_reg(config, pos + PCI_CAP_LIST_ID, 1, &id);
		if (id == 0xff)
			break;
		if (id == cap)
			return pos;
		pos += PCI_CAP_LIST_NEXT;
	}
	return 0;
}

static int pci_find_own_next_cap(struct spear_pcie_gadget_config *config,
			u32 pos, int cap)
{
	int ttl = PCI_FIND_CAP_TTL;

	return pci_find_own_next_cap_ttl(config, pos, cap, &ttl);
}

static int pci_find_own_cap_start(struct spear_pcie_gadget_config *config,
				u8 hdr_type)
{
	u32 status;

	spear_dbi_read_reg(config, PCI_STATUS, 2, &status);
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
static int pci_find_own_capability(struct spear_pcie_gadget_config *config,
		int cap)
{
	u32 pos;
	u32 hdr_type;

	spear_dbi_read_reg(config, PCI_HEADER_TYPE, 1, &hdr_type);

	pos = pci_find_own_cap_start(config, hdr_type);
	if (pos)
		pos = pci_find_own_next_cap(config, pos, cap);

	return pos;
}

static irqreturn_t spear_pcie_gadget_irq(int irq, void *dev_id)
{
	return 0;
}

static ssize_t pcie_gadget_show_link(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct spear_pcie_gadget_config *config = dev_get_drvdata(dev);
	struct pcie_app_reg *app_reg =
		(struct pcie_app_reg *)config->va_app_base;

	if (readl(&app_reg->app_status_1) & ((u32)1 << XMLH_LINK_UP_ID))
		return sprintf(buf, "UP");
	else
		return sprintf(buf, "DOWN");
}

static ssize_t pcie_gadget_store_link(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct spear_pcie_gadget_config *config = dev_get_drvdata(dev);
	struct pcie_app_reg *app_reg =
		(struct pcie_app_reg *)config->va_app_base;
	char link[10];

	if (sscanf(buf, "%s", link) != 1)
		return -EINVAL;

	if (!strcmp(link, "UP"))
		writel(readl(&app_reg->app_ctrl_0) | (1 << APP_LTSSM_ENABLE_ID),
			&app_reg->app_ctrl_0);
	else
		writel(readl(&app_reg->app_ctrl_0)
				& ~(1 << APP_LTSSM_ENABLE_ID),
				&app_reg->app_ctrl_0);
	return count;
}

static DEVICE_ATTR(link, S_IWUSR | S_IRUGO, pcie_gadget_show_link,
		pcie_gadget_store_link);

static ssize_t pcie_gadget_show_int_type(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct spear_pcie_gadget_config *config = dev_get_drvdata(dev);

	return sprintf(buf, "%s", config->int_type);
}

static ssize_t pcie_gadget_store_int_type(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct spear_pcie_gadget_config *config = dev_get_drvdata(dev);
	char int_type[10];
	u32 cap, vector, vec, flags;

	if (sscanf(buf, "%s", int_type) != 1)
		return -EINVAL;

	if (!strcmp(int_type, "INTA"))
		spear_dbi_write_reg(config, PCI_INTERRUPT_LINE, 1, 1);

	else if (!strcmp(int_type, "MSI")) {
		vector = config->requested_msi;
		vec = 0;
		while (vector > 1) {
			vector /= 2;
			vec++;
		}
		spear_dbi_write_reg(config, PCI_INTERRUPT_LINE, 1, 0);
		cap = pci_find_own_capability(config, PCI_CAP_ID_MSI);
		spear_dbi_read_reg(config, cap + PCI_MSI_FLAGS, 1, &flags);
		flags &= ~PCI_MSI_FLAGS_QMASK;
		flags |= vec << 1;
		spear_dbi_write_reg(config, cap + PCI_MSI_FLAGS, 1, flags);
	}

	strcpy(config->int_type, int_type);

	return count;
}

static DEVICE_ATTR(int_type, S_IWUSR | S_IRUGO, pcie_gadget_show_int_type,
		pcie_gadget_store_int_type);

static ssize_t pcie_gadget_show_no_of_msi(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct spear_pcie_gadget_config *config = dev_get_drvdata(dev);
	struct pcie_app_reg *app_reg =
		(struct pcie_app_reg *)config->va_app_base;
	u32 cap, vector, vec, flags;

	if ((readl(&app_reg->msg_status) & (1 << CFG_MSI_EN_ID))
			!= (1 << CFG_MSI_EN_ID))
		vector = 0;
	else {
		cap = pci_find_own_capability(config, PCI_CAP_ID_MSI);
		spear_dbi_read_reg(config, cap + PCI_MSI_FLAGS, 1, &flags);
		flags &= ~PCI_MSI_FLAGS_QSIZE;
		vec = flags >> 4;
		vector = 1;
		while (vec--)
			vector *= 2;
	}
	config->configured_msi = vector;

	return sprintf(buf, "%u", vector);
}

static ssize_t pcie_gadget_store_no_of_msi(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct spear_pcie_gadget_config *config = dev_get_drvdata(dev);

	if (sscanf(buf, "%u", &config->requested_msi) != 1)
		return -EINVAL;
	if (config->requested_msi > 32)
		config->requested_msi = 32;

	return count;
}

static DEVICE_ATTR(no_of_msi, S_IWUSR | S_IRUGO, pcie_gadget_show_no_of_msi,
		pcie_gadget_store_no_of_msi);

static ssize_t pcie_gadget_store_inta(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct spear_pcie_gadget_config *config = dev_get_drvdata(dev);
	struct pcie_app_reg *app_reg =
		(struct pcie_app_reg *)config->va_app_base;
	int en;

	if (sscanf(buf, "%d", &en) != 1)
		return -EINVAL;

	if (en)
		writel(readl(&app_reg->app_ctrl_0) | (1 << SYS_INT_ID),
				&app_reg->app_ctrl_0);
	else
		writel(readl(&app_reg->app_ctrl_0) & ~(1 << SYS_INT_ID),
				&app_reg->app_ctrl_0);

	return count;
}

static DEVICE_ATTR(inta, S_IWUSR, NULL, pcie_gadget_store_inta);

static ssize_t pcie_gadget_store_send_msi(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct spear_pcie_gadget_config *config = dev_get_drvdata(dev);
	struct pcie_app_reg *app_reg =
		(struct pcie_app_reg *)config->va_app_base;
	int vector;
	u32 ven_msi;

	if (sscanf(buf, "%d", &vector) != 1)
		return -EINVAL;

	if (!config->configured_msi)
		return -EINVAL;

	if (vector >= config->configured_msi)
		return -EINVAL;

	ven_msi = readl(&app_reg->ven_msi_1);
	ven_msi &= ~VEN_MSI_FUN_NUM_MASK;
	ven_msi |= 0 << VEN_MSI_FUN_NUM_ID;
	ven_msi &= ~VEN_MSI_TC_MASK;
	ven_msi |= 0 << VEN_MSI_TC_ID;
	ven_msi &= ~VEN_MSI_VECTOR_MASK;
	ven_msi |= vector << VEN_MSI_VECTOR_ID;

	/*generating interrupt for msi vector*/
	ven_msi |= VEN_MSI_REQ_EN;
	writel(ven_msi, &app_reg->ven_msi_1);
	/*need to wait till this bit is cleared, it is not cleared
	 * autometically[Bug RTL] TBD*/
	udelay(1);
	ven_msi &= ~VEN_MSI_REQ_EN;
	writel(ven_msi, &app_reg->ven_msi_1);

	return count;
}

static DEVICE_ATTR(send_msi, S_IWUSR, NULL, pcie_gadget_store_send_msi);

static ssize_t pcie_gadget_show_vendor_id(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct spear_pcie_gadget_config *config = dev_get_drvdata(dev);
	u32 id;

	spear_dbi_read_reg(config, PCI_VENDOR_ID, 2, &id);

	return sprintf(buf, "%x", id);
}

static ssize_t pcie_gadget_store_vendor_id(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct spear_pcie_gadget_config *config = dev_get_drvdata(dev);
	u32 id;

	if (sscanf(buf, "%x", &id) != 1)
		return -EINVAL;

	spear_dbi_write_reg(config, PCI_VENDOR_ID, 2, id);

	return count;
}

static DEVICE_ATTR(vendor_id, S_IWUSR | S_IRUGO, pcie_gadget_show_vendor_id,
		pcie_gadget_store_vendor_id);

static ssize_t pcie_gadget_show_device_id(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct spear_pcie_gadget_config *config = dev_get_drvdata(dev);
	u32 id;

	spear_dbi_read_reg(config, PCI_DEVICE_ID, 2, &id);

	return sprintf(buf, "%x", id);
}

static ssize_t pcie_gadget_store_device_id(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct spear_pcie_gadget_config *config = dev_get_drvdata(dev);
	u32 id;

	if (sscanf(buf, "%x", &id) != 1)
		return -EINVAL;

	spear_dbi_write_reg(config, PCI_DEVICE_ID, 2, id);

	return count;
}

static DEVICE_ATTR(device_id, S_IWUSR | S_IRUGO, pcie_gadget_show_device_id,
		pcie_gadget_store_device_id);

static ssize_t pcie_gadget_show_bar0_size(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct spear_pcie_gadget_config *config = dev_get_drvdata(dev);

	return sprintf(buf, "%x", config->bar0_size);
}

static ssize_t pcie_gadget_store_bar0_size(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct spear_pcie_gadget_config *config = dev_get_drvdata(dev);
	u32 size, pos, pos1;
	u32 no_of_bit = 0;

	if (sscanf(buf, "%x", &size) != 1)
		return -EINVAL;
	/* as per PCIE specs, min bar size supported is 128 bytes. But
	 * our controller supports min as 256*/
	if (size <= 0x100)
		size = 0x100;
	/* max bar size is 1MB*/
	else if (size >= 0x100000)
		size = 0x100000;
	else {
		pos = 0;
		pos1 = 0;
		while (pos < 21) {
			pos = find_next_bit((unsigned long *)&size, 21, pos);
			if (pos != 21)
				pos1 = pos + 1;
			pos++;
			no_of_bit++;
		}
		if (no_of_bit == 2)
			pos1--;

		size = 1 << pos1;
	}
	config->bar0_size = size;
	spear_dbi_write_reg(config, PCIE_BAR0_MASK_REG, 4, size - 1);

	return count;
}

static DEVICE_ATTR(bar0_size, S_IWUSR | S_IRUGO, pcie_gadget_show_bar0_size,
		pcie_gadget_store_bar0_size);

static ssize_t pcie_gadget_show_bar0_address(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct spear_pcie_gadget_config *config = dev_get_drvdata(dev);
	struct pcie_app_reg *app_reg =
		(struct pcie_app_reg *)config->va_app_base;

	u32 address = readl(&app_reg->pim0_mem_addr_start);

	return sprintf(buf, "%x", address);
}

static ssize_t pcie_gadget_store_bar0_address(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct spear_pcie_gadget_config *config = dev_get_drvdata(dev);
	struct pcie_app_reg *app_reg =
		(struct pcie_app_reg *)config->va_app_base;
	u32 address;

	if (sscanf(buf, "%x", &address) != 1)
		return -EINVAL;

	address &= ~(config->bar0_size - 1);
	if (config->va_bar0_address)
		iounmap((void *)config->va_bar0_address);
	config->va_bar0_address = (u32)ioremap(address, config->bar0_size);
	if (!config->va_bar0_address)
		return -ENOMEM;

	writel(address, &app_reg->pim0_mem_addr_start);

	return count;
}

static DEVICE_ATTR(bar0_address, S_IWUSR | S_IRUGO,
		pcie_gadget_show_bar0_address, pcie_gadget_store_bar0_address);

static ssize_t pcie_gadget_show_bar0_rw_offset(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct spear_pcie_gadget_config *config = dev_get_drvdata(dev);

	return sprintf(buf, "%x", config->bar0_rw_offset);
}

static ssize_t pcie_gadget_store_bar0_rw_offset(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct spear_pcie_gadget_config *config = dev_get_drvdata(dev);
	u32 offset;

	if (sscanf(buf, "%x", &offset) != 1)
		return -EINVAL;

	if (offset % 4)
		return -EINVAL;

	config->bar0_rw_offset = offset;

	return count;
}

static DEVICE_ATTR(bar0_rw_offset, S_IWUSR | S_IRUGO,
	pcie_gadget_show_bar0_rw_offset, pcie_gadget_store_bar0_rw_offset);

static ssize_t pcie_gadget_show_bar0_data(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct spear_pcie_gadget_config *config = dev_get_drvdata(dev);
	u32 data;

	if (!config->va_bar0_address)
		return -ENOMEM;

	data = readl(config->va_bar0_address + config->bar0_rw_offset);

	return sprintf(buf, "%x", data);
}

static ssize_t pcie_gadget_store_bar0_data(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct spear_pcie_gadget_config *config = dev_get_drvdata(dev);
	u32 data;

	if (sscanf(buf, "%x", &data) != 1)
		return -EINVAL;

	if (!config->va_bar0_address)
		return -ENOMEM;

	writel(data, config->va_bar0_address + config->bar0_rw_offset);

	return count;
}

static DEVICE_ATTR(bar0_data, S_IWUSR | S_IRUGO,
		pcie_gadget_show_bar0_data, pcie_gadget_store_bar0_data);

static ssize_t pcie_gadget_show_help(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	char text[] = "\t\tlink read->ltssm status\n \
		link write->arg1 = UP to enable ltsmm DOWN to disable\n \
		int_type read->type of supported interrupt\n \
		int_type write->arg1 = interrupt type to be configured and\n \
		can be INTA, MSI or NO_INT\n \
		(select MSI only when you have programmed no_of_msi)\n \
		no_of_msi read->zero if MSI is not enabled by host\n \
		and positive value is the number of MSI vector granted\n \
		no_of_msi write->arg1 = number of MSI vector needed\n \
		inta write->arg1 = 1 to assert INTA and 0 to de-assert\n \
		send_msi write->arg1 = MSI vector to be send\n \
		vendor_id read->programmed vendor id (hex)\n\
		vendor_id write->arg1 = vendor id(hex) to be programmed\n \
		device_id read->programmed device id(hex)\n \
		device_id write->arg1 = device id(hex) to be programmed\n \
		bar0_size read->size of bar0 in hex\n \
		bar0_size write->arg1= size of bar0 in hex\n \
		(default bar0 size is 1000 (hex) bytes)\n \
		bar0_address read->address of bar0 mapped area in hex\n \
		bar0_address write->arg1 = address of bar0 mapped area in hex\n\
		(default mapping of bar0 is SYSRAM1(E0800000)\n \
		(always program bar size before bar address)\n \
		(kernel might modify bar size and address to align)\n \
		(read back bar size and address after writing to check)\n \
		bar0_rw_offset read->offset of bar0 for which bar0_data \n \
		will return value\n \
		bar0_rw_offset write->arg1 = offset of bar0 for which\n \
		bar0_data will write value\n \
		bar0_data read->data at bar0_rw_offset\n \
		bar0_data write->arg1 = data to be written at\n \
		bar0_rw_offset\n";

	int size = (sizeof(text) < PAGE_SIZE) ? sizeof(text) : PAGE_SIZE;

	return snprintf(buf, size, "%s", text);
}

static DEVICE_ATTR(help, S_IRUGO, pcie_gadget_show_help, NULL);

static struct attribute *pcie_gadget_attributes[] = {
	&dev_attr_link.attr,
	&dev_attr_int_type.attr,
	&dev_attr_no_of_msi.attr,
	&dev_attr_inta.attr,
	&dev_attr_send_msi.attr,
	&dev_attr_vendor_id.attr,
	&dev_attr_device_id.attr,
	&dev_attr_bar0_size.attr,
	&dev_attr_bar0_address.attr,
	&dev_attr_bar0_rw_offset.attr,
	&dev_attr_bar0_data.attr,
	&dev_attr_help.attr,
	NULL
};

static const struct attribute_group pcie_gadget_attr_group = {
	.attrs = pcie_gadget_attributes,
};

static void spear13xx_pcie_device_init(struct spear_pcie_gadget_config *config)
{
	struct pcie_app_reg *app_reg =
		(struct pcie_app_reg *)config->va_app_base;

	/*setup registers for outbound translation */

	writel(config->base, &app_reg->in0_mem_addr_start);
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

	/* Keep AORAM mapped at BAR0 as default */
	config->bar0_size = INBOUND_ADDR_MASK + 1;
	spear_dbi_write_reg(config, PCIE_BAR0_MASK_REG, 4, INBOUND_ADDR_MASK);
	spear_dbi_write_reg(config, PCI_BASE_ADDRESS_0, 4, 0xC);
	config->va_bar0_address = (u32)ioremap(SPEAR13XX_SYSRAM1_BASE,
			config->bar0_size);

	writel(SPEAR13XX_SYSRAM1_BASE, &app_reg->pim0_mem_addr_start);
	writel(0, &app_reg->pim1_mem_addr_start);
	writel(INBOUND_ADDR_MASK + 1, &app_reg->mem0_addr_offset_limit);

	writel(0x0, &app_reg->pim_io_addr_start);
	writel(0x0, &app_reg->pim_io_addr_start);
	writel(0x0, &app_reg->pim_rom_addr_start);

	writel(DEVICE_TYPE_EP | (1 << MISCTRL_EN_ID)
			| ((u32)1 << REG_TRANSLATION_ENABLE),
			&app_reg->app_ctrl_0);
	/* disable all rx interrupts */
	writel(0, &app_reg->int_mask);

	/* Select INTA as default*/
	spear_dbi_write_reg(config, PCI_INTERRUPT_LINE, 1, 1);
}

static int __devinit spear_pcie_gadget_probe(struct platform_device *pdev)
{
	struct resource *res0, *res1;
	struct spear_pcie_gadget_config *config;
	unsigned int status = 0;
	int irq;
	struct clk *clk;

	/* get resource for application registers*/

	res0 = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res0) {
		dev_err(&pdev->dev, "no resource defined\n");
		return -EBUSY;
	}
	if (!request_mem_region(res0->start, resource_size(res0),
				pdev->name)) {
		dev_err(&pdev->dev, "pcie gadget region already	claimed\n");
		return -EBUSY;
	}
	/* get resource for dbi registers*/

	res1 = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res1) {
		dev_err(&pdev->dev, "no resource defined\n");
		goto err_rel_res0;
	}
	if (!request_mem_region(res1->start, resource_size(res1),
				pdev->name)) {
		dev_err(&pdev->dev, "pcie gadget region already	claimed\n");
		goto err_rel_res0;
	}

	config = kzalloc(sizeof(*config), GFP_KERNEL);
	if (!config) {
		dev_err(&pdev->dev, "out of memory\n");
		status = -ENOMEM;
		goto err_rel_res;
	}

	config->va_app_base = ioremap(res0->start, resource_size(res0));
	if (!config->va_app_base) {
		dev_err(&pdev->dev, "ioremap fail\n");
		status = -ENOMEM;
		goto err_kzalloc;
	}

	config->base = (void *)res1->start;

	config->va_dbi_base = ioremap(res1->start, resource_size(res1));
	if (!config->va_dbi_base) {
		dev_err(&pdev->dev, "ioremap fail\n");
		status = -ENOMEM;
		goto err_iounmap_app;
	}

	dev_set_drvdata(&pdev->dev, config);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "no update irq?\n");
		status = irq;
		goto err_iounmap;
	}

	status = request_irq(irq, spear_pcie_gadget_irq, 0, pdev->name, NULL);
	if (status) {
		dev_err(&pdev->dev, "pcie gadget interrupt IRQ%d already \
				claimed\n", irq);
		goto err_get_irq;
	}
	/* Register sysfs hooks */
	status = sysfs_create_group(&pdev->dev.kobj, &pcie_gadget_attr_group);
	if (status)
		goto err_irq;

	/* init basic pcie application registers*/
	/* do not enable clock if it is PCIE0.Ideally , all controller should
	 * have been independent from others with respect to clock. But PCIE1
	 * and 2 depends on PCIE0.So PCIE0 clk is provided during board init.*/
	if (pdev->id == 1) {
		/* Ideally CFG Clock should have been also enabled here. But
		 * it is done currently during board init routne*/
		clk = clk_get_sys("pcie1", NULL);
		if (!clk) {
			pr_err("%s:couldn't get clk for pcie1\n", __func__);
			goto err_irq;
		}
		if (clk_enable(clk)) {
			pr_err("%s:couldn't enable clk for pcie1\n", __func__);
			goto err_irq;
		}
	} else if (pdev->id == 2) {
		/* Ideally CFG Clock should have been also enabled here. But
		 * it is done currently during board init routne*/
		clk = clk_get_sys("pcie2", NULL);
		if (!clk) {
			pr_err("%s:couldn't get clk for pcie2\n", __func__);
			goto err_irq;
		}
		if (clk_enable(clk)) {
			pr_err("%s:couldn't enable clk for pcie2\n", __func__);
			goto err_irq;
		}
	}
	spear13xx_pcie_device_init(config);

	return 0;
err_irq:
	free_irq(irq, NULL);
err_get_irq:
	dev_set_drvdata(&pdev->dev, NULL);
err_iounmap:
	iounmap(config->va_dbi_base);
err_iounmap_app:
	iounmap(config->va_app_base);
err_kzalloc:
	kfree(config);
err_rel_res:
	release_mem_region(res1->start, resource_size(res1));
err_rel_res0:
	release_mem_region(res0->start, resource_size(res0));
	return status;
}

static int __devexit spear_pcie_gadget_remove(struct platform_device *pdev)
{
	struct resource *res0, *res1;
	struct spear_pcie_gadget_config *config;
	int irq;

	res0 = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	res1 = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	irq = platform_get_irq(pdev, 0);
	config = dev_get_drvdata(&pdev->dev);

	free_irq(irq, NULL);
	dev_set_drvdata(&pdev->dev, NULL);
	iounmap(config->va_dbi_base);
	iounmap(config->va_app_base);
	kfree(config);
	release_mem_region(res1->start, resource_size(res1));
	release_mem_region(res0->start, resource_size(res0));
	sysfs_remove_group(&pdev->dev.kobj, &pcie_gadget_attr_group);

	return 0;
}

static void spear_pcie_gadget_shutdown(struct platform_device *pdev)
{
}

static struct platform_driver spear_pcie_gadget_driver = {
	.probe = spear_pcie_gadget_probe,
	.remove = spear_pcie_gadget_remove,
	.shutdown = spear_pcie_gadget_shutdown,
	.driver = {
		.name = "pcie-gadget-spear",
		.bus = &platform_bus_type
	},
};

static int __init spear_pcie_gadget_init(void)
{
	return platform_driver_register(&spear_pcie_gadget_driver);
}
module_init(spear_pcie_gadget_init);

static void __exit spear_pcie_gadget_exit(void)
{
	platform_driver_unregister(&spear_pcie_gadget_driver);
}
module_exit(spear_pcie_gadget_exit);

MODULE_ALIAS("pcie-gadget-spear");
MODULE_AUTHOR("Pratyush Anand");
MODULE_LICENSE("GPL");
