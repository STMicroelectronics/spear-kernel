/*
 * Encoder device driver (kernel module)
 *
 * Copyright (C) 2012 Google Finland Oy.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
--------------------------------------------------------------------------------
--
--  Abstract : 6280/7280/8270/8290 Encoder device driver (kernel module)
--
------------------------------------------------------------------------------*/



#include <asm/irq.h>
#include <linux/clk.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/signal.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

/* our own stuff */
#include "hx280enc.h"

/* module description */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Google Finland Oy");
MODULE_DESCRIPTION("Hantro 6280/7280/8270/8290 Encoder driver");

/* Encoder interrupt register */
#define X280_INTERRUPT_REGISTER_ENC	(0x04)
#define X280_REGISTER_DISABLE_ENC	(0x38)

#define ENC_HW_ID1			0x62800000
#define ENC_HW_ID2			0x72800000
#define ENC_HW_ID3			0x82700000
#define ENC_HW_ID4			0x82900000

#define VIDEO_ENC_DEF_RATE	(250*1000*1000)
#define VIDEO_ENC_CHRDEV_NAME	"video_enc"


struct clk *video_enc_clk;

static struct hx280enc_dev device;

char *encirq = "plat";

static const char hx280enc_dev_name[] = "hx280";

/* module_param(name, type, perm) */
module_param(encirq, charp, 0);

/* and this is our MAJOR; use 0 for dynamic allocation (recommended)*/
static int hx280enc_major;

/* here's all the must remember stuff */
struct hx280enc_t {
	char *buffer;
	unsigned int buffsize;
	unsigned long iobaseaddr;
	unsigned int iosize;
	volatile u8 *hwregs;
	unsigned int irq;
	struct fasync_struct *async_queue;
};

/* dynamic allocation? */
static struct hx280enc_t hx280enc_data;

static int ReserveIO(void);
static void ReleaseIO(void);
static void ResetAsic(struct hx280enc_t *dev);

#ifdef CONFIG_VIDEO_SPEAR_VIDEOENC_DEBUG
static void dump_regs(unsigned long data);
#endif

/* IRQ handler */
static irqreturn_t hx280enc_isr(int irq, void *dev_id);

/* VM operations */
static int hx280enc_vm_fault(struct vm_area_struct *vma,
				struct vm_fault *vmf)
{
	PDEBUG("hx280enc_vm_fault: problem with mem access\n");
	return VM_FAULT_SIGBUS; /* send a SIGBUS */
}

static void hx280enc_vm_open(struct vm_area_struct *vma)
{
	PDEBUG("hx280enc_vm_open:\n");
}

static void hx280enc_vm_close(struct vm_area_struct *vma)
{
	PDEBUG("hx280enc_vm_close:\n");
}

static struct vm_operations_struct hx280enc_vm_ops = {
	.open	= hx280enc_vm_open,
	.close	= hx280enc_vm_close,
	.fault	= hx280enc_vm_fault,
};

/* the device's mmap method. The VFS has kindly prepared the process's
 * vm_area_struct for us, so we examine this to see what was requested.
 */

static int hx280enc_mmap(struct file *filp, struct vm_area_struct *vma)
{
	int result = -EINVAL;

	result = -EINVAL;

	vma->vm_ops = &hx280enc_vm_ops;

	return result;
}

static long hx280enc_ioctl(struct file *filp,
				unsigned int cmd, unsigned long arg)
{
	int err = 0;

	PDEBUG("ioctl cmd 0x%08ux\n", cmd);
	/*
	 * extract the type and number bitfields, and don't encode
	 * wrong cmds: return ENOTTY (inappropriate ioctl) before access_ok()
	 */
	if (_IOC_TYPE(cmd) != HX280ENC_IOC_MAGIC)
		return -ENOTTY;
	if (_IOC_NR(cmd) > HX280ENC_IOC_MAXNR)
		return -ENOTTY;

	/*
	 * the direction is a bitmask, and VERIFY_WRITE catches R/W
	 * transfers. `Type' is user-oriented, while
	 * access_ok is kernel-oriented, so the concept of "read" and
	 * "write" is reversed
	 */
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void *) arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void *) arg, _IOC_SIZE(cmd));
	if (err)
		return -EFAULT;

	switch (cmd) {
	case HX280ENC_IOCGHWOFFSET:
		__put_user(hx280enc_data.iobaseaddr, (unsigned long *) arg);
		break;

	case HX280ENC_IOCGHWIOSIZE:
		__put_user(hx280enc_data.iosize, (unsigned int *) arg);
		break;
	}
	return 0;
}

static int hx280enc_open(struct inode *inode, struct file *filp)
{
	int result = 0;
	struct hx280enc_t *dev = &hx280enc_data;

	filp->private_data = (void *) dev;

	PDEBUG("dev opened\n");
	return result;
}

static int hx280enc_fasync(int fd, struct file *filp, int mode)
{
	struct hx280enc_t *dev = (struct hx280enc_t *) filp->private_data;

	PDEBUG("fasync called\n");

	return fasync_helper(fd, filp, mode, &dev->async_queue);
}

static int hx280enc_release(struct inode *inode, struct file *filp)
{
#ifdef CONFIG_VIDEO_SPEAR_VIDEOENC_DEBUG
	struct hx280enc_t *dev = (struct hx280enc_t *) filp->private_data;

	dump_regs((unsigned long) dev); /* dump the regs */
#endif

	/* remove this filp from the asynchronusly notified filp's */
	hx280enc_fasync(-1, filp, 0);

	PDEBUG("dev closed\n");
	return 0;
}

/* VFS methods */
static const struct file_operations hx280enc_fops = {
	.mmap		= hx280enc_mmap,
	.open		= hx280enc_open,
	.release	= hx280enc_release,
	.unlocked_ioctl	= hx280enc_ioctl,
	.fasync		= hx280enc_fasync,
};

#ifdef CONFIG_PM
static int spear_video_enc_suspend(struct device *dev)
{
	clk_disable(video_enc_clk);

	dev_info(dev, "Suspended.\n");

	return 0;
}

static int spear_video_enc_resume(struct device *dev)
{
	int result;

	result = clk_enable(video_enc_clk);
	if (result) {
		dev_err(dev, "Can't enable clock\n");
		return result;
	}

	dev_info(dev, "Resumed.\n");

	return 0;
}

static const struct dev_pm_ops spear_video_enc_pm_ops = {
	.suspend = spear_video_enc_suspend,
	.resume = spear_video_enc_resume,
	.freeze = spear_video_enc_suspend,
	.restore = spear_video_enc_resume,
};
#endif /* CONFIG_PM */

int hx280enc_sysfs_register(struct hx280enc_dev *device, dev_t dev,
				const char *hx280enc_dev_name)
{
	int err = 0;
	struct device *mdev;

	device->hx280enc_class = class_create(THIS_MODULE, hx280enc_dev_name);
	if (IS_ERR(device->hx280enc_class)) {
		err = PTR_ERR(device->hx280enc_class);
		goto init_class_err;
	}
	mdev = device_create(device->hx280enc_class, NULL, dev, NULL,
				hx280enc_dev_name);
	if (IS_ERR(mdev)) {
		err = PTR_ERR(mdev);
		goto init_mdev_err;
	}

	/* Success! */
	return 0;

init_mdev_err:
	class_destroy(device->hx280enc_class);
init_class_err:

	return err;
}


static int spear_video_enc_probe(struct platform_device *pdev)
{
	int result;
	dev_t dev = 0;
	long unsigned int parse_irq = 0;
	struct resource *venc_mem;
	struct resource *venc_irq;


	/* get platform resources */
	venc_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!venc_mem) {
		dev_err(&pdev->dev, "Can't get memory resource\n");
		goto no_res;
	}

	if (encirq == NULL || !strncmp(encirq, "plat", 4)) {
		venc_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
		if (!venc_irq) {
			dev_err(&pdev->dev, "Can't get interrupt resource\n");
			goto no_res;
		}
		hx280enc_data.irq = venc_irq->start;
		dev_info(&pdev->dev, "Platform IRQ mode selected\n");
	} else if (!strncmp(encirq, "no", 2)) {
		hx280enc_data.irq = 0;
		dev_info(&pdev->dev, "Disable IRQ mode selected\n");
	} else {
		if (strict_strtoul(encirq, 10, &(parse_irq))) {
			dev_err(&pdev->dev, "Wrong IRQ parameter: %s\n", encirq);
			goto no_res;
		}
		hx280enc_data.irq = parse_irq;
		dev_info(&pdev->dev, "IRQ param: %d\n", hx280enc_data.irq);
	}

	/* clock init */
	video_enc_clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(video_enc_clk)) {
		dev_err(&pdev->dev, "Can't get clock\n");
		goto no_clk;
	}

	clk_set_rate(video_enc_clk, VIDEO_ENC_DEF_RATE);

	result = clk_enable(video_enc_clk);
	if (result) {
		dev_err(&pdev->dev, "Can't enable clock\n");
		goto put_clk;
	}

	hx280enc_data.iobaseaddr = venc_mem->start;
	hx280enc_data.iosize = venc_mem->end - venc_mem->start + 4;

	hx280enc_data.async_queue = NULL;

	if (0 == hx280enc_major) {
		/* auto select a major */
		result = alloc_chrdev_region(&dev, 0, 1, hx280enc_dev_name);
		hx280enc_major = MAJOR(dev);
	} else {
		/* use load time defined major number */
		dev = MKDEV(hx280enc_major, 0);
		result = register_chrdev_region(dev, 1, hx280enc_dev_name);
	}

	if (result)
		goto init_chrdev_err;


	memset(&device, 0, sizeof(device));

	/* initialize our char dev data */
	cdev_init(&device.cdev, &hx280enc_fops);
	device.cdev.owner = THIS_MODULE;
	device.cdev.ops = &hx280enc_fops;

	/* register char dev with the kernel */
	result = cdev_add(&device.cdev, dev, 1/*count*/);
	if (result)
		goto init_cdev_err;

	result = hx280enc_sysfs_register(&device, dev, hx280enc_dev_name);
	if (result)
		goto init_sysfs_err;

	result = ReserveIO();
	if (result < 0)
		goto err;


	ResetAsic(&hx280enc_data);  /* reset hardware */
	/* get the IRQ line */
	if (hx280enc_data.irq > 0) {
		result = request_irq(hx280enc_data.irq, hx280enc_isr,
				IRQF_DISABLED | IRQF_SHARED,
				"hx280enc", (void *) &hx280enc_data);
		if (result != 0) {
			if (result == -EINVAL) {
				dev_err(&pdev->dev, "Bad irq number or handler\n");
			} else if (result == -EBUSY) {
				dev_err(&pdev->dev, "IRQ <%d> busy, change your config\n",
					hx280enc_data.irq);
			}

			ReleaseIO();
			goto err;
		}
	} else {
		dev_info(&pdev->dev, "IRQ not in use!\n");
	}

	dev_info(&pdev->dev, "Video encoder initialized. Major = %d\n",
			hx280enc_major);

	return 0;

put_clk:
	clk_put(video_enc_clk);
no_clk:
	return -EFAULT;
init_sysfs_err:
	cdev_del(&device.cdev);
init_cdev_err:
	unregister_chrdev_region(dev, 1/*count*/);
init_chrdev_err:
	return -EFAULT;
no_res:
	return -ENODEV;
err:
	dev_err(&pdev->dev, "probe failed\n");
	return result;
}


static int spear_video_enc_exit(struct platform_device *pdev)
{
	struct hx280enc_t *dev = (struct hx280enc_t *) &hx280enc_data;

	/* disable HW */
	writel(0, dev->hwregs + X280_REGISTER_DISABLE_ENC);
	/* clear enc IRQ */
	writel(0, dev->hwregs + X280_INTERRUPT_REGISTER_ENC);

	/* free the encoder IRQ */
	if (dev->irq != -1)
		free_irq(dev->irq, (void *) dev);

	ReleaseIO();

	unregister_chrdev(hx280enc_major, VIDEO_ENC_CHRDEV_NAME);

	dev_info(&pdev->dev, "module removed\n");
	return 0;
}

static struct platform_driver spear1340_video_enc_driver = {
	.probe = spear_video_enc_probe,
	.remove = spear_video_enc_exit,
	.driver = {
		.name = "video_enc",
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &spear_video_enc_pm_ops,
#endif

	},
};

static int __init spear_video_enc_init(void)
{
	return platform_driver_register(&spear1340_video_enc_driver);
}
module_init(spear_video_enc_init);

static void __exit spear_video_enc_cleanup(void)
{
	platform_driver_unregister(&spear1340_video_enc_driver);
}
module_exit(spear_video_enc_cleanup);


static int ReserveIO(void)
{
	long int hwid;

	hx280enc_data.hwregs =
		(volatile u8 *) ioremap_nocache(hx280enc_data.iobaseaddr,
						hx280enc_data.iosize);

	if (hx280enc_data.hwregs == NULL) {
		printk(KERN_INFO "hx280enc: failed to ioremap HW regs\n");
		ReleaseIO();
		return -EBUSY;
	}

	hwid = readl(hx280enc_data.hwregs);

	/* check for encoder HW ID */
	if ((((hwid >> 16) & 0xFFFF) != ((ENC_HW_ID1 >> 16) & 0xFFFF)) &&
	   (((hwid >> 16) & 0xFFFF) != ((ENC_HW_ID2 >> 16) & 0xFFFF)) &&
	   (((hwid >> 16) & 0xFFFF) != ((ENC_HW_ID3 >> 16) & 0xFFFF)) &&
	   (((hwid >> 16) & 0xFFFF) != ((ENC_HW_ID4 >> 16) & 0xFFFF))) {
		printk(KERN_INFO "hx280enc: HW not found at 0x%08lx (hwid: 0x%08lx)\n",
			hx280enc_data.iobaseaddr, hwid);
#ifdef CONFIG_VIDEO_SPEAR_VIDEOENC_DEBUG
		dump_regs((unsigned long) &hx280enc_data);
#endif
		ReleaseIO();
		return -EBUSY;
	}

	printk(KERN_INFO
		   "hx280enc: HW at base <0x%08lx> with ID <0x%08lx>\n",
		   hx280enc_data.iobaseaddr, hwid);

	return 0;
}

static void ReleaseIO(void)
{
	if (hx280enc_data.hwregs)
		iounmap((void *) hx280enc_data.hwregs);
}


irqreturn_t hx280enc_isr(int irq, void *dev_id)
{
	struct hx280enc_t *dev = (struct hx280enc_t *) dev_id;
	u32 irq_status;

	irq_status = readl(dev->hwregs + 0x04);

	if (irq_status & 0x01) {

		/* clear enc IRQ and slice ready interrupt bit */
		writel(irq_status & (~0x101), dev->hwregs + 0x04);

		/* Handle slice ready interrupts. The reference implementation
		 * doesn't signal slice ready interrupts to EWL.
		 * The EWL will poll the slices ready register value. */
		if ((irq_status & 0x1FE) == 0x100) {
			PDEBUG("Slice ready IRQ handled!\n");
			return IRQ_HANDLED;
		}

		/* All other interrupts will be signaled to EWL. */
		if (dev->async_queue)
			kill_fasync(&dev->async_queue, SIGIO, POLL_IN);
		else {
			printk(KERN_WARNING
				"hx280enc: IRQ received w/o anybody waiting for it!\n");
		}

		PDEBUG("IRQ handled!\n");
		return IRQ_HANDLED;
	} else {
		PDEBUG("IRQ received, but NOT handled!\n");
		return IRQ_NONE;
	}

}

void ResetAsic(struct hx280enc_t *dev)
{
	int i;

	writel(0, dev->hwregs + 0x38);

	for (i = 4; i < dev->iosize; i += 4)
		writel(0, dev->hwregs + i);

}

#ifdef CONFIG_VIDEO_SPEAR_VIDEOENC_DEBUG
void dump_regs(unsigned long data)
{
	struct hx280enc_t *dev = (struct hx280enc_t *) data;
	int i;

	PDEBUG("Reg Dump Start\n");
	for (i = 0; i < dev->iosize; i += 4)
		PDEBUG("\toffset %02X = %08X\n", i, readl(dev->hwregs + i));

	PDEBUG("Reg Dump End\n");
}
#endif
