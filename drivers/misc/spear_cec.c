/*
 * drivers/misc/spear_cec.c
 *
 * SPEAr cec driver - source file
 *
 * Copyright (C) 2011 ST Microelectronics
 * Vipul Kumar Samar <vipulkumar.samar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spear_cec.h>
#include <linux/spinlock.h>
#include <linux/wait.h>

/* cec registers */
#define STM_CEC_PRE_L		0x0
#define STM_CEC_PRE_H		0x4
	#define CEC_CLK_FACTOR			50
#define STM_CEC_ESR		0x8
	#define STM_ESR_BIT_TMG_ERR		BIT(0)
	#define STM_ESR_BIT_PER_ERR		BIT(1)
	#define STM_ESR_RBTF_ERR		BIT(2)
	#define STM_ESR_START_ERR		BIT(3)
	#define STM_ESR_BLK_ACK_ERR		BIT(4)
	#define STM_ESR_LINE_ERR		BIT(5)
	#define STM_ESR_TBTF_ERR		BIT(6)

#define STM_CEC_CTL		0xC
	#define STM_CTL_TSOM			BIT(0)
	#define STM_CTL_TEOM			BIT(1)
	#define STM_CTL_TX_ERR			BIT(2)
	#define STM_CTL_TBTF			BIT(3)
	#define STM_CTL_RSOM			BIT(4)
	#define STM_CTL_REOM			BIT(5)
	#define STM_CTL_RX_ERR			BIT(6)
	#define STM_CTL_RBTF			BIT(7)

#define STM_CEC_TXD		0x10
#define STM_CEC_RXD		0x14
#define STM_CEC_CFG		0x18
	#define STM_CFG_P_ENB			BIT(0)
	#define STM_CFG_INT_ENB			BIT(1)
	#define STM_CFG_BIT_TMG_ERR_MODE	BIT(2)
	#define STM_CFG_BIT_PER_ERR_SPEC_MODE	BIT(3)
	#define STM_CFG_ERR_RESYNC_MODE		BIT(4)
	#define STM_CFG_NAIV			BIT(5)
	#define STM_CFG_WAKEUP_ENABLE		BIT(6)

#define STM_CEC_OAR		0x1C

#define DRIVER_NAME		"spear_cec"
#define CEC_TIMEOUT		(1 * HZ)

enum cec_state {
	OP_RX = 1 << 0,
	OP_TX = 1 << 1,
	TX_ERR = 1 << 2,
	RX_ERR = 1 << 3,
	TX_DATA_SENT = 1 << 4,
	RX_DATA_AVAIL = 1 << 5,
};

struct cec_dev {
	char state;
	int cec_enable;
	struct clk *clk;
	void __iomem *io_base;
	struct device *dev;
	struct miscdevice misc;
	spinlock_t lock;

	/* transfer related variables */
	wait_queue_head_t tx_waitq;
	wait_queue_head_t rx_waitq;
	char *tx_buf;
	int tx_size;
	int tx_index;
	char *rx_buf;
	int rx_size;
	int rx_index;
};

static inline struct cec_dev *get_cec_priv_data(struct miscdevice *miscdev)
{
	return container_of(miscdev, struct cec_dev, misc);
}

static inline void cec_write(struct cec_dev *cdev, unsigned int off,
		unsigned int val)
{
	writel(val, cdev->io_base + off);
}

static inline long cec_read(struct cec_dev *cdev, unsigned int off)
{
	return readl(cdev->io_base + off);
}

static inline void cec_tx(struct cec_dev *cdev)
{
	int val = 0;

	if (!(cdev->state & OP_TX)) {
		cec_write(cdev, STM_CEC_CTL, val);
		pr_err("%s:Trying to tx without any write call\n",
				__func__);
		return;
	}

	/* send start of message for first byte */
	if (!cdev->tx_index)
		val |= STM_CTL_TSOM;

	/* send end of message for last byte */
	if (cdev->tx_index == cdev->tx_size - 1)
		val |= STM_CTL_TEOM;

	/* write data untill all data is written */
	if (cdev->tx_index != cdev->tx_size) {
		cec_write(cdev, STM_CEC_TXD,
				*(cdev->tx_buf + cdev->tx_index++));
	} else {
		cdev->state |= TX_DATA_SENT;
		wake_up_interruptible(&cdev->tx_waitq);
	}

	cec_write(cdev, STM_CEC_CTL, val);
}

static inline void cec_rx(struct cec_dev *cdev)
{
	*(cdev->rx_buf + cdev->rx_index++) = cec_read(cdev,
			STM_CEC_RXD);
}

static inline void spear_cec_set_addr(struct cec_dev *cdev, unsigned
		long addr)
{
	cec_write(cdev, STM_CEC_OAR, addr & 0xF);
}

static void spear_cec_set_divider(struct cec_dev *cdev)
{
	u32 scale_val;

	scale_val = (clk_get_rate(cdev->clk) / 1000000) * CEC_CLK_FACTOR;
	cec_write(cdev, STM_CEC_PRE_L, (scale_val & 0xFF));
	cec_write(cdev, STM_CEC_PRE_H, ((scale_val >> 8) & 0xFF));
}

static void spear_cec_enable(struct cec_dev *cdev)
{
	u32 reg;

	reg = cec_read(cdev, STM_CEC_CFG);
	reg |= STM_CFG_P_ENB | STM_CFG_BIT_PER_ERR_SPEC_MODE |
			STM_CFG_INT_ENB;
	cec_write(cdev, STM_CEC_CFG, reg);
}

static void spear_cec_disable(struct cec_dev *cdev)
{
	u32 reg;

	reg = cec_read(cdev, STM_CEC_CFG);
	reg &= ~(STM_CFG_P_ENB | STM_CFG_INT_ENB);
	cec_write(cdev, STM_CEC_CFG, reg);
}

static int spear_cec_open(struct inode *inode, struct file *file)
{
	int ret = 0;
	unsigned long flags;
	struct cec_dev *cdev = get_cec_priv_data(file->private_data);

	/* init registers */
	spin_lock_irqsave(&cdev->lock, flags);
	if (!cdev->cec_enable++) {
		ret = clk_enable(cdev->clk);
		if (ret)
			goto out;

		cec_write(cdev, STM_CEC_CTL, 0);
		spear_cec_set_divider(cdev);
		spear_cec_enable(cdev);
	}

out:
	spin_unlock_irqrestore(&cdev->lock, flags);
	return ret;
}

static int spear_cec_release(struct inode *inode, struct file *file)
{
	struct cec_dev *cdev = get_cec_priv_data(file->private_data);
	unsigned long flags;

	spin_lock_irqsave(&cdev->lock, flags);
	if (!--cdev->cec_enable) {
		if (cdev->rx_buf)
			kfree(cdev->rx_buf);

		spear_cec_disable(cdev);
		clk_disable(cdev->clk);
	}
	spin_unlock_irqrestore(&cdev->lock, flags);

	return 0;
}

static ssize_t spear_cec_read(struct file *file, char __user *buffer,
		size_t count, loff_t *ppos)
{
	struct cec_dev *cdev = get_cec_priv_data(file->private_data);
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&cdev->lock, flags);
	/* Allocate buffer on first read */
	if (!cdev->rx_buf) {
		cdev->rx_buf = kmalloc(count, GFP_ATOMIC);
		if (!cdev->rx_buf) {
			pr_err("%s:Memory allocation failed!\n", __func__);
			ret = -ENOMEM;
			goto rx_err;
		}
	}

	cdev->rx_size = count;
	cdev->rx_index = 0;
	cdev->state |= OP_RX;

	spin_unlock_irqrestore(&cdev->lock, flags);

	ret = wait_event_interruptible(cdev->rx_waitq,
			(cdev->state & (RX_ERR | RX_DATA_AVAIL)));

	spin_lock_irqsave(&cdev->lock, flags);

	if (ret || (cdev->state & RX_ERR)) {
		ret = -ERESTARTSYS;
		goto rx_err;
	}

	if (copy_to_user(buffer, cdev->rx_buf, cdev->rx_index)) {
		pr_err("%s:Copy to user space failed!\n", __func__);
		ret = -EFAULT;
		goto rx_err;
	}

	ret = cdev->rx_index;

rx_err:
	cdev->state &= ~(OP_RX | RX_ERR | RX_DATA_AVAIL);
	spin_unlock_irqrestore(&cdev->lock, flags);
	return ret;
}

static ssize_t
spear_cec_write(struct file *file, const char __user *buffer, size_t count,
		loff_t *ppos)
{
	struct cec_dev *cdev = get_cec_priv_data(file->private_data);
	unsigned long flags;
	int ret;

	/* check data size */
	if (!count)
		return -EINVAL;

	spin_lock_irqsave(&cdev->lock, flags);
	cdev->tx_buf = kmalloc(count, GFP_ATOMIC);
	if (!cdev->tx_buf) {
		pr_err("%s:Memory allocation failed!\n", __func__);
		ret = -ENOMEM;
		goto tx_err;
	}

	if (copy_from_user(cdev->tx_buf, buffer, count)) {
		pr_err("%s:Copy data from user space failed!\n", __func__);
		ret = -EFAULT;
		goto tx_err;
	}

	cdev->tx_index = 0;
	cdev->tx_size = count;
	cdev->state |= OP_TX;

	cec_tx(cdev);

	spin_unlock_irqrestore(&cdev->lock, flags);

	ret = wait_event_interruptible_timeout(cdev->tx_waitq,
			(cdev->state & (TX_ERR | TX_DATA_SENT)), CEC_TIMEOUT);

	spin_lock_irqsave(&cdev->lock, flags);

	if (ret <= 0 || (cdev->state & TX_ERR)) {
		pr_err("%s:TX timeout\n", __func__);
		ret = -ETIMEDOUT;
		goto tx_err;
	}

	ret = count;
tx_err:
	kfree(cdev->tx_buf);
	cdev->state &= ~(OP_TX | TX_ERR | TX_DATA_SENT);
	spin_unlock_irqrestore(&cdev->lock, flags);
	return ret;
}

static long
spear_cec_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct cec_dev *cdev = get_cec_priv_data(file->private_data);
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&cdev->lock, flags);
	if (cdev->state) {
		ret = -EBUSY;
		goto err_ioctl;
	}

	switch (cmd) {
	case CEC_IOC_SETLADDR:
		spear_cec_set_addr(cdev, arg);
		break;
	default:
		ret = -EINVAL;
	}

err_ioctl:
	spin_unlock_irqrestore(&cdev->lock, flags);
	return ret;
}

static const struct file_operations cec_fops = {
	.owner = THIS_MODULE,
	.open = spear_cec_open,
	.release = spear_cec_release,
	.read = spear_cec_read,
	.write = spear_cec_write,
	.unlocked_ioctl	= spear_cec_ioctl,
};

static irqreturn_t spear_cec_irq_handler(int irq, void *dev_id)
{
	struct cec_dev *cdev = dev_id;
	unsigned long flags;
	u32 ctl;

	spin_lock_irqsave(&cdev->lock, flags);
	ctl = cec_read(cdev, STM_CEC_CTL);

	if (ctl & STM_CTL_TX_ERR) {
		pr_err("%s:TX ERROR: %lx\n", __func__,
				cec_read(cdev, STM_CEC_ESR));
		cec_write(cdev, STM_CEC_CTL, 0);
		cdev->state |= TX_ERR;
		wake_up_interruptible(&cdev->tx_waitq);
	}

	if (ctl & STM_CTL_RX_ERR) {
		pr_err("%s:RX ERROR: %lx\n", __func__,
				cec_read(cdev, STM_CEC_ESR));
		cec_write(cdev, STM_CEC_CTL, 0);
		cdev->state |= RX_ERR;
		wake_up_interruptible(&cdev->rx_waitq);
	}

	if (ctl & STM_CTL_TBTF)
		cec_tx(cdev);

	if (ctl & STM_CTL_RBTF) {
		if (!(cdev->state & OP_RX)) {
			cec_write(cdev, STM_CEC_CTL, 0);
			pr_err("%s:Got rx interrupt without any read call\n",
					__func__);
			spin_unlock_irqrestore(&cdev->lock, flags);
			return IRQ_HANDLED;
		}

		if (cdev->rx_index < cdev->rx_size) {
			cec_rx(cdev);
			cec_write(cdev, STM_CEC_CTL, 0);

			if (ctl & STM_CTL_REOM) {
				cdev->state |= RX_DATA_AVAIL;
				wake_up_interruptible(&cdev->rx_waitq);
			}
		} else {
			pr_err("%s:Rx buffer is filled without EOM\n",
					__func__);
			cdev->state |= RX_ERR;
			cec_write(cdev, STM_CEC_CTL, 0);
			wake_up_interruptible(&cdev->rx_waitq);
		}
	}

	spin_unlock_irqrestore(&cdev->lock, flags);
	return IRQ_HANDLED;
}

static int __init
spear_cec_probe(struct platform_device *pdev)
{
	struct cec_dev *cdev;
	struct resource *res;
	int irq_num, ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Failed to get memory resource\n");
		ret = -EINVAL;
	}

	if (!devm_request_mem_region(&pdev->dev, res->start, resource_size(res),
				DRIVER_NAME)) {
		dev_err(&pdev->dev, "Failed to request memory region\n");
		ret = -EBUSY;
	}

	cdev = devm_kzalloc(&pdev->dev, sizeof(*cdev), GFP_KERNEL);
	if (!cdev) {
		dev_err(&pdev->dev, "Can't allocate memory for CEC\n");
		ret = -ENOMEM;
		goto err_mem_region;
	}

	cdev->io_base = devm_ioremap(&pdev->dev, res->start,
			resource_size(res));
	if (!cdev->io_base) {
		dev_err(&pdev->dev, "Failed to remap register\n");
		ret = -ENOMEM;
		goto err_kfree;
	}

	cdev->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(cdev->clk)) {
		ret = PTR_ERR(cdev->clk);
		dev_err(&pdev->dev, "Clock get fail\n");
		goto err_iounmap;
	}

	irq_num = platform_get_irq(pdev, 0);
	if (irq_num < 0) {
		dev_err(&pdev->dev, "Failed to get cec irq resource\n");
		ret = -EBUSY;
		goto err_clk_put;
	}

	if (request_irq(irq_num, spear_cec_irq_handler, IRQF_DISABLED,
				pdev->name, cdev)) {
		dev_err(&pdev->dev,
				"Failed to install %s irq (%d)\n", "cec", ret);
		ret = -EBUSY;
		goto err_clk_put;
	}

	init_waitqueue_head(&cdev->rx_waitq);
	init_waitqueue_head(&cdev->tx_waitq);
	spin_lock_init(&cdev->lock);

	platform_set_drvdata(pdev, cdev);

	cdev->misc.minor = MISC_DYNAMIC_MINOR;
	cdev->misc.name = dev_name(&pdev->dev);
	cdev->misc.fops = &cec_fops;
	cdev->cec_enable = 0;

	ret = misc_register(&cdev->misc);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register as misc device\n");
		ret = -EBUSY;
		goto err_free_irq;
	}

	dev_info(&pdev->dev, "%s:registered·at·0x%p\n",
			KBUILD_MODNAME, cdev->io_base);
	return 0;

err_free_irq:
	free_irq(irq_num, cdev);
err_clk_put:
	clk_put(cdev->clk);
err_iounmap:
	iounmap(cdev->io_base);
err_kfree:
	kfree(cdev);
err_mem_region:
	release_mem_region(res->start, resource_size(res));

	return ret;
}

static int spear_cec_remove(struct platform_device *pdev)
{
	struct resource *res;
	struct cec_dev *cdev = platform_get_drvdata(pdev);
	int irq_num = platform_get_irq(pdev, 0);

	misc_deregister(&cdev->misc);
	free_irq(irq_num, &pdev->id);
	clk_put(cdev->clk);
	iounmap(cdev->io_base);
	kfree(cdev);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, resource_size(res));

	platform_set_drvdata(pdev, NULL);

	return 0;
}

#ifdef CONFIG_PM
static int spear_cec_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct cec_dev *cdev = platform_get_drvdata(pdev);
	unsigned long flags;

	spin_lock_irqsave(&cdev->lock, flags);
	if (cdev->cec_enable > 0) {
		spear_cec_disable(cdev);
		clk_disable(cdev->clk);
	}
	spin_unlock_irqrestore(&cdev->lock, flags);

	return 0;
}

static int spear_cec_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct cec_dev *cdev = platform_get_drvdata(pdev);
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&cdev->lock, flags);
	if (cdev->cec_enable > 0) {
		ret = clk_enable(cdev->clk);
		if (ret)
			pr_err("%s:Error enabling clock\n", __func__);
		cec_write(cdev, STM_CEC_CTL, 0);
		spear_cec_set_divider(cdev);
		spear_cec_enable(cdev);
	}
	spin_unlock_irqrestore(&cdev->lock, flags);

	return 0;
}
static SIMPLE_DEV_PM_OPS(spear_cec_pm_ops, spear_cec_suspend, spear_cec_resume);
#endif

static struct platform_driver spear_cec_driver = {
	.probe		= spear_cec_probe,
	.remove		= spear_cec_remove,
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &spear_cec_pm_ops,
#endif
	},
};

static int __init spear_cec_init(void)
{
	return platform_driver_register(&spear_cec_driver);
}
module_init(spear_cec_init);

static void __exit spear_cec_exit(void)
{
	platform_driver_unregister(&spear_cec_driver);
}
module_exit(spear_cec_exit);

MODULE_AUTHOR("Vipul Kumar Samar <vipulkumar.samar@st.com>");
MODULE_DESCRIPTION("ST Microelectronics CEC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
