/*
 * linux/drivers/char/designware_jpeg.c
 *
 * SYNOPSYS JPEG Encoder/Decoder driver - source file
 *
 * Copyright (ST) 2010 Viresh Kumar (viresh.kumar@st.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <plat/jpeg.h>
#include <mach/hardware.h>
#include "designware_jpeg.h"

u8 djpeg_burst;
#define DJPEG_WIDTH	4

/* global dev structure for jpeg. as there is no way to pass it from char
 ** interface */
static struct jpeg_dev *g_drv_data;

/*
 * sg_per_buf: num of sg required for transfer size amount of data with dma
 */
static s32 sg_per_buf(enum jpeg_dev_type dev_type, u32 size)
{
	u32 max_xfer, num_sg = 0, size_left = size;

	if (!size)
		return 0;

	/* calculate the max transfer size supported by src device for a
	 ** single sg */
	max_xfer = JPEG_DMA_MAX_COUNT * DJPEG_WIDTH;

	/* calculate the number of sgs with size max_xfer */
	num_sg	= size_left/max_xfer;
	size_left -= num_sg * max_xfer;

	/* calculate the number of sgs with size less than max_xfer but with
	 ** width as DJPEG_WIDTH */
	if (size_left / DJPEG_WIDTH) {
		num_sg++;
		size_left -= (size_left / DJPEG_WIDTH) * DJPEG_WIDTH;
	}

	/* calculate the number of sgs with size less than max_xfer and width
	 ** as WIDTH_BYTE */
	if (size_left)
		num_sg++;

	return num_sg;
}

/* functions manipulating scatter lists */
void fill_sg(enum jpeg_dev_type dev_type, size_t size)
{
	u32 max_xfer;
	dma_addr_t addr;
	struct scatterlist *sg;

	if (dev_type == JPEG_READ) {
		addr = g_drv_data->read_dbuf[g_drv_data->current_rbuf];
		sg = g_drv_data->sg[JPEG_READ];
		sg_dma_address(sg) = addr;
		sg_set_page(sg, pfn_to_page(addr >> PAGE_SHIFT), 0,
				offset_in_page(addr));
	} else {
		addr = g_drv_data->write_dbuf[g_drv_data->current_wbuf];
		sg = g_drv_data->sg[JPEG_WRITE];

		/* calculate the max transfer size supported by src device for
		 ** a single sg */
		max_xfer = JPEG_DMA_MAX_COUNT * DJPEG_WIDTH;

		while (size/max_xfer) {
			sg_dma_address(sg) = addr;
			sg_set_page(sg, pfn_to_page(addr >> PAGE_SHIFT),
					max_xfer, offset_in_page(addr));
			addr += max_xfer;
			size -= max_xfer;
			sg++;
		}

		if (size / DJPEG_WIDTH) {
			sg_dma_address(sg) = addr;
			sg_set_page(sg, pfn_to_page(addr >> PAGE_SHIFT),
					(size / DJPEG_WIDTH) * DJPEG_WIDTH,
					offset_in_page(addr));
			addr += (size / DJPEG_WIDTH) * DJPEG_WIDTH;
			size -= (size / DJPEG_WIDTH) * DJPEG_WIDTH;
			sg++;
		}

		if (size) {
			sg_dma_address(sg) = addr;
			sg_set_page(sg, pfn_to_page(addr >> PAGE_SHIFT), size,
					offset_in_page(addr));
		}
	}
}

static s32 get_sg(enum jpeg_dev_type dev_type, size_t size)
{
	u32 num = 1;

	/* allocate scatter list */
	if (dev_type == JPEG_WRITE)
		num = sg_per_buf(JPEG_WRITE, size);

	g_drv_data->sg[dev_type] = kmalloc(num *
			sizeof(*g_drv_data->sg[dev_type]), GFP_DMA);
	if (!g_drv_data->sg[dev_type]) {
		dev_err(&g_drv_data->pdev->dev, "%s: scatter list mem alloc "
				"fail\n", dev_type == JPEG_WRITE ?
				"jpeg_write" : "jpeg_read");
		return -ENOMEM;
	}

	g_drv_data->sg_len[dev_type] = num;
	sg_init_table(g_drv_data->sg[dev_type], num);

	fill_sg(dev_type, size);
	return 0;
}

static void put_sg(enum jpeg_dev_type dev_type)
{
	/* free scatter list */
	kfree(g_drv_data->sg[dev_type]);
	g_drv_data->sg[dev_type] = NULL;
	g_drv_data->sg_len[dev_type] = 0;
}

/*
 * find_num_of_sg: returns total number of sg's required for current image
 * size.
 */
static s32 find_num_of_sg(void)
{
	u32 num_sg = 0, transfers = 0;

	/* This is number of different transfers. for ex: Img Size = 586, buf
	 ** size = 100, so there will be 5 transfers of size 100 and one of
	 ** size 86. Now transfers = 6 */
	transfers = (g_drv_data->img_size[JPEG_WRITE] +
			g_drv_data->buf_size[JPEG_WRITE]
			- 1)/g_drv_data->buf_size[JPEG_WRITE];

	if (transfers > 1) {
		num_sg = sg_per_buf(JPEG_WRITE,
				g_drv_data->buf_size[JPEG_WRITE]);

		/* this will give num_sg for all the buf_size transfers */
		num_sg = num_sg * (transfers - 1);
	}

	/* this adds num_sg of the last transfer */
	num_sg += sg_per_buf(JPEG_WRITE, (g_drv_data->img_size[JPEG_WRITE] -
				(transfers - 1) *
				g_drv_data->buf_size[JPEG_WRITE]));

	/* one extra sg count is required by the jpeg ip for eachtransfer except
	 ** the first */
	num_sg--;
	return num_sg;
}

/*
 * jpeg_sleep: adds current process to wait queue.
 */
static s32 jpeg_sleep(enum jpeg_dev_type dev_type)
{
	s32 status = -ETIME;
	wait_event_interruptible_timeout(g_drv_data->wait_queue[dev_type],
			g_drv_data->flag_arrived[dev_type] != 0, JPEG_TIMEOUT);

	/* if the flag arrived is 1, then it is normal wakeup */
	if (g_drv_data->flag_arrived[dev_type]) {
		g_drv_data->flag_arrived[dev_type]--;
		status = 0;
	}
	return status;
}

/*
 * jpeg_wakeup: removes a process from wait queue.
 */
static void jpeg_wakeup(enum jpeg_dev_type dev_type)
{
	/* wake up from sleep */
	g_drv_data->flag_arrived[dev_type]++;
	wake_up_interruptible(&g_drv_data->wait_queue[dev_type]);
}

/*
 * jpeg_flush: resets the software state of jpeg.
 */
static void jpeg_flush(void)
{
	unsigned long flags;
	put_sg(JPEG_WRITE);
	put_sg(JPEG_READ);

	spin_lock_irqsave(&g_drv_data->lock, flags);
	g_drv_data->sg_len[JPEG_READ] = 0;
	g_drv_data->sg_len[JPEG_WRITE] = 0;
	g_drv_data->cookie[JPEG_READ] = 0;
	g_drv_data->cookie[JPEG_WRITE] = 0;

	g_drv_data->flag_arrived[JPEG_READ] = 0;
	g_drv_data->flag_arrived[JPEG_WRITE] = 1;
	g_drv_data->rdma_flag = 0;
	g_drv_data->rjpeg_flag = 0;
	g_drv_data->img_size[JPEG_READ] = 0;
	g_drv_data->img_size[JPEG_WRITE] = 0;
	g_drv_data->current_wbuf = 1;
	g_drv_data->current_rbuf = 0;

	g_drv_data->operation_complete = 1;
	g_drv_data->read_complete = 0;
	g_drv_data->first_write = 1;

	g_drv_data->data_to_jpeg = 0;
	g_drv_data->data_from_jpeg = 0;

	g_drv_data->error = 0;
	spin_unlock_irqrestore(&g_drv_data->lock, flags);
}

/*
 * jpeg_abort: aborts the ongoing jpeg encoding/decoding.
 */
static void jpeg_abort(s32 error)
{
	struct dma_chan *dchan;

	/* if dma is busy doing transfer with jpeg, then abort them */
	if (g_drv_data->dma_chan[JPEG_READ]) {
		dchan = g_drv_data->dma_chan[JPEG_READ];
		dchan->device->device_control(dchan, DMA_TERMINATE_ALL, 0);
	}

	if (g_drv_data->dma_chan[JPEG_WRITE]) {
		dchan = g_drv_data->dma_chan[JPEG_WRITE];
		dchan->device->device_control(dchan, DMA_TERMINATE_ALL, 0);
	}

	/*reset jpeg hardware and reset all jpeg flags.*/
	jpeg_reset(g_drv_data->regs);
	jpeg_flush();

	/*indicate occurance of error*/
	g_drv_data->error = error;

	/*wake up any sleeping process.*/
	jpeg_wakeup(JPEG_WRITE);
	jpeg_wakeup(JPEG_READ);

	g_drv_data->flag_arrived[JPEG_READ] = 0;
	g_drv_data->flag_arrived[JPEG_WRITE] = 1;

	dev_err(&g_drv_data->pdev->dev, "jpeg aborted. error no. %d\n", error);
}

/**
 * jpeg_enable - starts jpeg encoding/decoding.
 * @num_lli: number of lli's present in the transfer list to be passed to dma.
 * @num_burst: number of bursts after which interrupt is requested.
 *
 * This function starts jpeg encoding or decoding depending upon its first
 * parameter. It first resets the jpeg core, then it configures lli count, burst
 * count and header enable, and then enables jpeg.
 */
void jpeg_enable(u32 num_lli, u32 num_burst)
{
	u32 val;

	if (cpu_is_spear1310()) {
		/* configure ctrl register for number of llis */
		val = SPEAR1310_NUM_LLI(num_lli);
		/* configure words in last transfer in Tx FIFO */
		val |= LAST_XFER_WORDS((g_drv_data->img_size[JPEG_WRITE] / 4)
				% 16);
	} else {
		val = NUM_LLI(num_lli);
	}

	writel(val, &g_drv_data->regs->CTRL_REG.STATUS);

	/* configure jpeg for interrupt on burst count transfers */
	if (num_burst)
		writel(BURST_COUNT_ENABLE | num_burst,
				&g_drv_data->regs->CTRL_REG.BYTES_BEF_INT);

	val = readl(&g_drv_data->regs->REG1);
	/* configure jpeg core register */
	if (g_drv_data->hdr_enable)
		val |= HDR_ENABLE;

	if (g_drv_data->operation == JPEG_DECODE)
		val |= DECODER_ENABLE;
	else
		val &= ENCODER_ENABLE;

	writel(val, &g_drv_data->regs->REG1);
	writel(JPEG_ENABLE, &g_drv_data->regs->REG0);
}

/* recalculates number of mcus */
static void fix_mcu_num(struct jpeg_hdr *hdr_info)
{
	u32 xsize, ysize, h0, v0;

	h0 = hdr_info->mcu_comp[0].h;
	v0 = hdr_info->mcu_comp[0].v;

	if (!h0 || !v0)
		return;

	xsize = roundup(hdr_info->xsize, h0 * 8);
	ysize = roundup(hdr_info->ysize, v0 * 8);

	hdr_info->mcu_num = ((xsize * ysize) / (64 * h0 * v0));
}

/**
 * jpeg_get_hdr - gives the header info of the decoded image
 * @hdr_info: header info will be filled in this structure after decoding.
 *
 * This functions is called after JPEG decoding is completed. This will fill
 * the header information structure passed to it, with the info of the last
 * decoded jpeg file. This information is important when the output data from
 * the decoder must be converted to some other format or must be displayed in
 * some format.
 */
void jpeg_get_hdr(struct jpeg_hdr *hdr_info)
{
	s32 temp;
	temp = readl(&g_drv_data->regs->REG1);
	hdr_info->num_clr_cmp = NUM_CLR_CMP(temp);
	hdr_info->clr_spc_type = COL_SPC_TYPE(temp);
	hdr_info->num_cmp_for_scan_hdr = NUM_CMP_SCAN_HDR(temp);
	hdr_info->rst_mark_en = IS_RST_ENABLE(temp);
	hdr_info->ysize = Y_SIZE(temp);

	hdr_info->mcu_num = NUM_MCU(readl(&g_drv_data->regs->REG2));

	temp = readl(&g_drv_data->regs->REG3);
	hdr_info->xsize = X_SIZE(temp);
	hdr_info->mcu_num_in_rst = RST_NUM_MCU(temp);

#define GET_MCU_COMP(x, reg) \
	do {\
		temp = reg; \
		hdr_info->mcu_comp[x].hdc = HUFF_TBL_DC(temp); \
		hdr_info->mcu_comp[x].hac = HUFF_TBL_AC(temp); \
		hdr_info->mcu_comp[x].qt = QUANT_TBL(temp); \
		hdr_info->mcu_comp[x].nblock = NUM_DATA_UNITS(temp); \
		hdr_info->mcu_comp[x].h = HORZ_SAMP(temp); \
		hdr_info->mcu_comp[x].v = VERT_SAMP(temp); \
	} while (0);

	GET_MCU_COMP(0, readl(&g_drv_data->regs->REG4));
	GET_MCU_COMP(1, readl(&g_drv_data->regs->REG5));
	GET_MCU_COMP(2, readl(&g_drv_data->regs->REG6));
	GET_MCU_COMP(3, readl(&g_drv_data->regs->REG7));

	fix_mcu_num(hdr_info);
}

/**
 * jpeg_set_hdr - sets the header info of the image
 * @hdr_info: header info
 *
 * This functions fills JPEG registers as per the values specified by
 * hdr_info structure.
 */
void jpeg_set_hdr(struct jpeg_hdr *hdr_info)
{
	s32 temp;

	/* Enable JPEG codec by writing 0 on status register */
	writel(0, &g_drv_data->regs->CTRL_REG.STATUS);

	temp = SET_NUM_CLR_CMP(hdr_info->num_clr_cmp);
	temp |= SET_COL_SPC_TYPE(hdr_info->clr_spc_type);
	temp |= SET_NUM_CMP_SCAN_HDR(hdr_info->num_cmp_for_scan_hdr);
	temp |= SET_Y_SIZE(hdr_info->ysize);
	temp |= RST_ENABLE(hdr_info->rst_mark_en);
	writel(temp, &g_drv_data->regs->REG1);

	writel(SET_NUM_MCU(hdr_info->mcu_num), &g_drv_data->regs->REG2);

	temp = SET_X_SIZE(hdr_info->xsize);
	temp |= SET_RST_NUM_MCU(hdr_info->mcu_num_in_rst);
	writel(temp, &g_drv_data->regs->REG3);

#define SET_MCU_COMP(x, reg) \
	do {\
		temp = HUFF_TBL_DC(hdr_info->mcu_comp[x].hdc);\
		temp |= SET_HUFF_TBL_AC(hdr_info->mcu_comp[x].hac);\
		temp |= SET_QUANT_TBL(hdr_info->mcu_comp[x].qt);\
		temp |= SET_NUM_DATA_UNITS(hdr_info->mcu_comp[x].nblock);\
		temp |= SET_HORZ_SAMP(hdr_info->mcu_comp[x].h);\
		temp |= SET_VERT_SAMP(hdr_info->mcu_comp[x].v);\
		writel(temp, reg);\
	} while (0);

	SET_MCU_COMP(0, &g_drv_data->regs->REG4);
	SET_MCU_COMP(1, &g_drv_data->regs->REG5);
	SET_MCU_COMP(2, &g_drv_data->regs->REG6);
	SET_MCU_COMP(3, &g_drv_data->regs->REG7);
}

/**
 * get_jpeg_info - gets jpeg header and jpeg tables
 * @info: jpeg info
 *
 * This functions reads jpeg header info and jpeg tables info.
 */
s32 get_jpeg_info(struct jpeg_info *info)
{
	struct jpeg_hdr hdr;
	s32 status;

	/* get hdr info */
	jpeg_get_hdr(&hdr);
	status = copy_to_user(&info->hdr, &hdr, sizeof(hdr));
	if (status) {
		dev_err(&g_drv_data->pdev->dev, "jpeg info not copied\n");
		return -EFAULT;
	}

	/* copy memory details one by one */
	status = jcopy_to_user(&info->qnt_mem, &g_drv_data->regs->QNT_MEM,
			S_QNT_MEM_SIZE * hdr.clr_spc_type, QNT_MEM_WIDTH);
	if (status)
		return -EFAULT;
	status = jcopy_to_user(&info->hmin_mem, &g_drv_data->regs->HMIN_MEM,
			HMIN_MEM_SIZE, HMIN_MEM_WIDTH);
	if (status)
		return -EFAULT;
	status = jcopy_to_user(&info->hbase_mem, &g_drv_data->regs->HBASE_MEM,
			HBASE_MEM_SIZE, HBASE_MEM_WIDTH);
	if (status)
		return -EFAULT;
	status = jcopy_to_user(&info->hsymb_mem, &g_drv_data->regs->HSYMB_MEM,
			HSYMB_MEM_SIZE, HSYMB_MEM_WIDTH);
	if (status)
		return -EFAULT;
	status = jcopy_to_user(&info->dht_mem, &g_drv_data->regs->DHT_MEM,
			DHT_MEM_SIZE, DHT_MEM_WIDTH);
	if (status)
		return -EFAULT;
	status = jcopy_to_user(&info->henc_mem, &g_drv_data->regs->HENC_MEM,
			HENC_MEM_SIZE, HENC_MEM_WIDTH);
	if (status)
		return -EFAULT;
	return 0;
}

/**
 * set_jpeg_info - sets jpeg header and jpeg tables
 * @info: jpeg info
 *
 * This functions writes jpeg header info and jpeg tables info on jpeg registers
 */
s32 set_jpeg_info(struct jpeg_dec_info *info, enum jpeg_operation op)
{
	struct jpeg_hdr hdr;
	int ret;

	g_drv_data->operation = op;

	/* check if hdr processing is required or not */
	if (!access_ok(VERIFY_READ, &info->hdr_enable, sizeof(u32)))
		return -EFAULT;
	ret = get_user(g_drv_data->hdr_enable, &info->hdr_enable);

	/* if hdr processing is enabled for jpeg decoding, then we don't need to
	 ** program registers and memories */
	if ((op == JPEG_DECODE) && g_drv_data->hdr_enable)
		return 0;

	ret = copy_from_user(&hdr, &info->hdr, sizeof(hdr));
	if (ret) {
		dev_err(&g_drv_data->pdev->dev, "jpeg info not copied\n");
		return -EFAULT;
	}
	jpeg_set_hdr(&hdr);

	/* copy memory details one by one */
	if (op == JPEG_DECODE) {
		ret = jcopy_from_user(&g_drv_data->regs->QNT_MEM,
				&info->qnt_mem, S_QNT_MEM_SIZE *
				hdr.clr_spc_type, QNT_MEM_WIDTH);
		if (ret)
			return -EFAULT;

		ret = jcopy_from_user(&g_drv_data->regs->HMIN_MEM,
				&info->hmin_mem, HMIN_MEM_SIZE, HMIN_MEM_WIDTH);
		if (ret)
			return -EFAULT;

		ret = jcopy_from_user(&g_drv_data->regs->HBASE_MEM,
				&info->hbase_mem, HBASE_MEM_SIZE,
				HBASE_MEM_WIDTH);
		if (ret)
			return -EFAULT;

		ret = jcopy_from_user(&g_drv_data->regs->HSYMB_MEM,
				&info->hsymb_mem, HSYMB_MEM_SIZE,
				HSYMB_MEM_WIDTH);
		if (ret)
			return -EFAULT;

	} else {
		struct jpeg_enc_info *enc_info = (struct jpeg_enc_info *)info;
		ret = jcopy_from_user(&g_drv_data->regs->QNT_MEM,
				&enc_info->qnt_mem, S_QNT_MEM_SIZE *
				hdr.clr_spc_type, QNT_MEM_WIDTH);
		if (ret)
			return -EFAULT;

		ret = jcopy_from_user(&g_drv_data->regs->DHT_MEM,
				&enc_info->dht_mem, DHT_MEM_SIZE,
				DHT_MEM_WIDTH);
		if (ret)
			return -EFAULT;

		ret = jcopy_from_user(&g_drv_data->regs->HENC_MEM,
				&enc_info->henc_mem, HENC_MEM_SIZE,
				HENC_MEM_WIDTH);
		if (ret)
			return -EFAULT;
	}
	return 0;
}

/*
 * dma_write_callback: interrupt handler for TO_JPEG/WRITE dma transfer.
 */
static void dma_write_callback(void *param)
{
	jpeg_wakeup(JPEG_WRITE);
}

/*
 * dma_read_callback: interrupt handler for FROM_JPEG/READ dma transfer.
 */
static void dma_read_callback(void *param)
{
	unsigned long flags;

	/* set flag of occurance of dma interupt. both dma and jpeg interrupts
	 ** should come before the read call is wake up */
	spin_lock_irqsave(&g_drv_data->lock, flags);
	if (g_drv_data->rdma_flag) {
		spin_unlock_irqrestore(&g_drv_data->lock, flags);
		return;
	}

	g_drv_data->rdma_flag = 1;
	if (g_drv_data->rjpeg_flag) {
		g_drv_data->rdma_flag = g_drv_data->rjpeg_flag = 0;
		spin_unlock_irqrestore(&g_drv_data->lock, flags);
		jpeg_wakeup(JPEG_READ);
		return;
	}
	spin_unlock_irqrestore(&g_drv_data->lock, flags);
}

/*
 * jfree: frees memory allocated with jmalloc.
 */
static void jfree(enum jpeg_dev_type dev_type)
{
	u32 size;
	dma_addr_t dma;
	void *buf;

	size = NUM_OF_BUF * g_drv_data->buf_size[dev_type];
	if (dev_type == JPEG_READ) {
		if (!g_drv_data->read_buf[0])
			return;

		buf = g_drv_data->read_buf[0];
		dma = g_drv_data->read_dbuf[0];
		g_drv_data->read_buf[0] = g_drv_data->read_buf[1] = 0;
		g_drv_data->read_dbuf[0] = g_drv_data->read_dbuf[1] = 0;
	} else {
		if (!g_drv_data->write_buf[0])
			return;

		buf = g_drv_data->write_buf[0];
		dma = g_drv_data->write_dbuf[0];
		g_drv_data->write_buf[0] = g_drv_data->write_buf[1] = 0;
		g_drv_data->write_dbuf[0] = g_drv_data->write_dbuf[1] = 0;
	}

	dma_free_writecombine(&g_drv_data->pdev->dev, size, buf, dma);
	g_drv_data->buf_size[dev_type] = 0;
}

/*
 * jmalloc: allocates memory for read and write buffers.
 */
static s32 jmalloc(enum jpeg_dev_type dev_type, size_t size)
{
	void *buf;
	dma_addr_t dbuf;

	if (!g_drv_data->operation_complete)
		return -EBUSY;

	/* free any memory if previously aquired after opening current jpeg
	 ** nodes */
	jfree(dev_type);

	buf = dma_alloc_writecombine(&g_drv_data->pdev->dev, size, &dbuf,
			GFP_KERNEL);
	if (!buf) {
		dev_err(&g_drv_data->pdev->dev, "%s buffer mem alloc fail\n",
				dev_type == JPEG_WRITE ? "write" : "read");
		return -ENOMEM;
	}

	/* size is total read memory. each buffer is half of size */
	size /= NUM_OF_BUF;

	if (dev_type == JPEG_READ) {
		g_drv_data->read_buf[0] = buf;
		g_drv_data->read_buf[1] = buf + size;
		g_drv_data->read_dbuf[0] = dbuf;
		g_drv_data->read_dbuf[1] = dbuf + size;
	} else {
		g_drv_data->write_buf[0] = buf;
		g_drv_data->write_buf[1] = buf + size;
		g_drv_data->write_dbuf[0] = dbuf;
		g_drv_data->write_dbuf[1] = dbuf + size;
	}

	g_drv_data->buf_size[dev_type] = size;

	return 0;
}

/*
 * jpeg_remap: remap physical memory in virtual space.
 */
static s32 jpeg_remap(struct vm_area_struct *vma, dma_addr_t dma, void *buf)
{
	/* calculate size requested for mapping */
	size_t size = vma->vm_end - vma->vm_start;

	return dma_mmap_writecombine(&g_drv_data->pdev->dev, vma, buf, dma,
			size);
}

/* xfer data between jpeg controller and memory */
static s32 dma_xfer(enum jpeg_dev_type dev_type, size_t size)
{
	u32 ret;
	struct dma_async_tx_descriptor *tx;
	struct dma_chan *chan = g_drv_data->dma_chan[dev_type];
	enum dma_data_direction direction;
	dma_async_tx_callback callback;

	ret = get_sg(dev_type, size);
	if (ret)
		return ret;

	if (dev_type == JPEG_READ) {
		direction = DMA_FROM_DEVICE;
		callback = dma_read_callback;
	} else {
		direction = DMA_TO_DEVICE;
		callback = dma_write_callback;
	}

	tx = chan->device->device_prep_slave_sg(chan, g_drv_data->sg[dev_type],
			g_drv_data->sg_len[dev_type], direction,
			DMA_PREP_INTERRUPT);

	if (!tx)
		return -ENOMEM;

	tx->callback = callback;
	tx->callback_param = NULL;
	g_drv_data->cookie[dev_type] = tx->tx_submit(tx);

	if (dma_submit_error(g_drv_data->cookie[dev_type])) {
		dev_err(&g_drv_data->pdev->dev, "dma submit error %d\n",
				g_drv_data->cookie[dev_type]);
		return -EAGAIN;
	}
	chan->device->device_issue_pending(chan);

	return 0;
}

/* check status of dma transfer */
static s32 dma_status(enum jpeg_dev_type dev_type, size_t size)
{
	s32 status;
	struct dma_chan *chan = g_drv_data->dma_chan[dev_type];

	put_sg(dev_type);
	status = dma_async_is_tx_complete(chan, g_drv_data->cookie[dev_type],
			NULL, NULL);

	if (status != DMA_SUCCESS) {
		dev_err(&g_drv_data->pdev->dev, "%s dma xfer hang. src image "
				"may contain extra data\n",
				dev_type == JPEG_READ ? "read" : "write");

		/* if dma is busy doing transfer with jpeg, then abort them */
		if (g_drv_data->dma_chan[JPEG_READ]) {
			chan = g_drv_data->dma_chan[JPEG_READ];
			chan->device->device_control(chan, DMA_TERMINATE_ALL,
					0);
		}
		if (g_drv_data->dma_chan[JPEG_WRITE]) {
			chan = g_drv_data->dma_chan[JPEG_WRITE];
			chan->device->device_control(chan, DMA_TERMINATE_ALL,
					0);
		}
		return -EAGAIN;
	}
	return status;
}

/**
 * jpeg_start - xfer src image chunk by chunk to jpeg for encoding/decoding.
 * @len: length of image to xfer to jpeg. image is present in the already
 * mmapped buffer.
 */
s32 jpeg_start(u32 len)
{
	unsigned long flags;
	s32 num_sg = 0, burst_num = 0;
	s32 status;

	/* make len multiple of word size */
	len = roundup(len, sizeof(unsigned int));

	/* sleep while data transfer to jpeg is not over */
	status = jpeg_sleep(JPEG_WRITE);
	if (status) {
		dev_err(&g_drv_data->pdev->dev, "write: sleep timeout. exiting"
				".\n");
		jpeg_abort(status);
		return status;
	}

	if (!g_drv_data->first_write) {
		status = dma_status(JPEG_WRITE, len);
		if (status)
			return status;
	}

	/* check if some error occured while this process is waiting */
	if (g_drv_data->error) {
		dev_err(&g_drv_data->pdev->dev, "write: error occured while "
				"sleeping. exiting.\n");
		return g_drv_data->error;
	}

	spin_lock_irqsave(&g_drv_data->lock, flags);
	/*
	 * if encoding/decoding is completed and all data is read from jpeg,
	 * then no more data should be supplied to jpeg_start
	 */
	if (g_drv_data->read_complete) {
		dev_dbg(&g_drv_data->pdev->dev, "write: jpeg file too big. "
				"encoding/decoding already completed\n");
		status = -EFBIG;
		goto err_lock;
	}

	if (!g_drv_data->img_size[JPEG_WRITE]) {
		dev_err(&g_drv_data->pdev->dev, "write: src image size zero\n");
		status = -EFAULT;
		goto err_lock;
	}

	/* if read and write buffers are not allocated or len is zero then
	 ** return error */
	if (!g_drv_data->buf_size[JPEG_READ] ||
			!g_drv_data->buf_size[JPEG_WRITE]) {
		dev_err(&g_drv_data->pdev->dev, "write: read/write buffer not"
				"allocated\n");
		status = -EFAULT;
		goto err_lock;
	}

	spin_unlock_irqrestore(&g_drv_data->lock, flags);

	if (!len) {
		dev_err(&g_drv_data->pdev->dev, "write length can't be 0\n");
		return -ENODATA;
	}

	shuffle_buf(g_drv_data->current_wbuf);
	/* write image packet to JPEG */
	if (dma_xfer(JPEG_WRITE, len)) {
		dev_err(&g_drv_data->pdev->dev, "write: dma xfer failed\n");
		jpeg_abort(-EAGAIN);
		return -EAGAIN;
	}

	/* first write */
	if (g_drv_data->first_write) {
		g_drv_data->operation_complete = 0;

		/* find number of sgs for xferring len amount of data */
		num_sg = find_num_of_sg();

		/* max no. of sg can be MAX_LLI */
		if (num_sg > MAX_LLI) {
			dev_err(&g_drv_data->pdev->dev, "write: jpeg image too "
					"large.\n");
			jpeg_abort(-EFBIG);
			return -EFBIG;
		}

		/* find number of bursts after which we want interrupt.
		 ** interrupt should come after transfer of buf_size amount of
		 ** data */
		burst_num = g_drv_data->buf_size[JPEG_READ]/
			(DJPEG_WIDTH * djpeg_burst);

		/*
		 * start transfer of encoded/decoded image to the allocated
		 * buffer
		 */
		if (dma_xfer(JPEG_READ, 0)) {
			dev_err(&g_drv_data->pdev->dev, "write: dma xfer from "
					"jpeg fails\n");
			jpeg_abort(-EAGAIN);
			return -EAGAIN;
		}

		/* start jpeg */
		jpeg_enable(num_sg, burst_num);
		g_drv_data->first_write = 0;
	}

	return 0;

err_lock:
	spin_unlock_irqrestore(&g_drv_data->lock, flags);
	return status;
}

/**
 * jpeg_get_data - reads encoded/decoded data.
 * @len: amount of data xferred.
 *
 * This function gives lenght of data written in the current_rbuf during this
 * read and starts a new xfer from jpeg. It also checks for the end of xfer */
s32 jpeg_get_data(u32 *len)
{
	unsigned long flags;
	s32 status;

	/* if all data is read and no more data is present */
	if (g_drv_data->read_complete) {
		*len = 0;
		return 0;
	}

	/* sleep while data transfer from jpeg is not over */
	if (jpeg_sleep(JPEG_READ)) {
		dev_err(&g_drv_data->pdev->dev, "read: sleep timeout. exiting"
				".\n");
		jpeg_abort(-ETIME);
		return -ETIME;
	}

	status = dma_status(JPEG_READ, g_drv_data->buf_size[JPEG_READ]);
	if (status)
		return status;

	/* check if some error occured while this process is waiting */
	if (g_drv_data->error)
		return g_drv_data->error;

	if (g_drv_data->operation_complete) {
		/* calculate data transfer in last transfer */
		spin_lock_irqsave(&g_drv_data->lock, flags);
		/* if data_from_jpeg is < data already read */
		if (g_drv_data->data_from_jpeg <
				g_drv_data->img_size[JPEG_READ]) {
			*len = 0;
			spin_unlock_irqrestore(&g_drv_data->lock, flags);
			return -EAGAIN;
		}

		*len = g_drv_data->data_from_jpeg -
			g_drv_data->img_size[JPEG_READ];

		if ((g_drv_data->operation == JPEG_ENCODE) &&
				!g_drv_data->hdr_enable) {
			unsigned char *addr;
			addr = g_drv_data->read_buf[g_drv_data->current_rbuf];
			/* we need to add EOF (ffd9) at the end of image */
			if (*len + 2 <= g_drv_data->buf_size[JPEG_READ]) {
				*(addr + *len) = 0xff;
				*(addr + *len + 1) = 0xd9;
				*len = *len + 2;
			}
		}

		g_drv_data->read_complete = 1;
		spin_unlock_irqrestore(&g_drv_data->lock, flags);

		jpeg_wakeup(JPEG_WRITE);
	} else {
		/* go here for burst transfer interrupts */
		*len = g_drv_data->buf_size[JPEG_READ];

		/* change buffer for data and start jpeg transfer */
		shuffle_buf(g_drv_data->current_rbuf);
		if (dma_xfer(JPEG_READ, 0)) {
			dev_err(&g_drv_data->pdev->dev,
					"read: dma xfer fails\n");
			jpeg_abort(-EAGAIN);
			return -EAGAIN;
		}
	}

	/* update total amount of output */
	g_drv_data->img_size[JPEG_READ] += *len;

	return 0;
}

/**
 * designware_jpeg_open - opens jpeg read or write node.
 * @inode: used by the kernel internally to represent files.
 * @file: represents a open file in kernel.
 *
 * If jpeg read/write node is already opened then it returns EBUSY error
 * otherwise return 0.
 */
static s32 designware_jpeg_open(struct inode *inode, struct file *file)
{
	s32 status = 0;
	enum jpeg_dev_type dev_type;
	void *dma_filter = g_drv_data->slaves->dma_filter;
	void *write_data = g_drv_data->slaves->mem2jpeg_slave;
	void *read_data = g_drv_data->slaves->jpeg2mem_slave;
	struct dma_slave_config jpeg_conf[] = {
		{
			/* jpeg2mem */
			.src_addr = g_drv_data->rx_reg,
			.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES,
			.src_maxburst = djpeg_burst,
			.dst_maxburst = djpeg_burst,
			.direction = DMA_FROM_DEVICE,
			.device_fc = true,
		}, {
			/* mem2jpeg */
			.dst_addr = g_drv_data->tx_reg,
			.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES,
			.src_maxburst = djpeg_burst,
			.dst_maxburst = djpeg_burst,
			.direction = DMA_TO_DEVICE,
			.device_fc = false,
		},
	};

	/* getting the minor number of the caller */
	dev_type = (enum jpeg_dev_type)iminor(inode);

	/* check if jpeg not used by any other user */
	if (g_drv_data->jpeg_busy[dev_type])
		return -EBUSY;

	/* mark jpeg as busy */
	g_drv_data->jpeg_busy[dev_type] = 1;

	/* checking that other jpeg node is closed or not. we will enable clock
	 ** and register dma only for the first call to open */
	if (g_drv_data->jpeg_busy[
			dev_type == JPEG_READ ? JPEG_WRITE : JPEG_READ])
		return status;

	/* request dma channels */
	if (!g_drv_data->dma_chan[JPEG_READ] ||
			!g_drv_data->dma_chan[JPEG_WRITE]) {
		g_drv_data->dma_chan[JPEG_READ] =
			dma_request_channel(g_drv_data->mask, dma_filter,
					read_data);
		g_drv_data->dma_chan[JPEG_WRITE] =
			dma_request_channel(g_drv_data->mask, dma_filter,
					write_data);
	}

	if (!g_drv_data->dma_chan[JPEG_READ] ||
			!g_drv_data->dma_chan[JPEG_WRITE]) {
		if (g_drv_data->dma_chan[JPEG_READ])
			dma_release_channel(g_drv_data->dma_chan[JPEG_READ]);
		if (g_drv_data->dma_chan[JPEG_WRITE])
			dma_release_channel(g_drv_data->dma_chan[JPEG_WRITE]);

		g_drv_data->jpeg_busy[dev_type] = 0;
		return -EBUSY;
	}

	/* Configure channel parameters if runtime_config is true */
	dmaengine_slave_config(g_drv_data->dma_chan[JPEG_READ],
			(void *) &jpeg_conf[0]);
	dmaengine_slave_config(g_drv_data->dma_chan[JPEG_WRITE],
			(void *) &jpeg_conf[1]);

	/* set default operation to jpeg decoding with hdr enable */
	g_drv_data->operation = JPEG_DECODE;
	g_drv_data->hdr_enable = 1;
	status = clk_enable(g_drv_data->clk);
	if (status)
		return status;

	return status;
}

/**
 * designware_jpeg_release - closes jpeg read or write node.
 * @inode: used by the kernel internally to represent files.
 * @file: represents a open file in kernel.
 *
 * It marks state of jpeg as free or not busy.
 */
static s32 designware_jpeg_release(struct inode *inode, struct file *file)
{
	enum jpeg_dev_type dev_type;

	/* getting the minor number of the caller */
	dev_type = (enum jpeg_dev_type)iminor(inode);

	if (!g_drv_data->jpeg_busy[dev_type])
		return -ENOENT;

	jfree(dev_type);
	/* checking that other jpeg node is already closed or not */
	if (!g_drv_data->jpeg_busy[
			dev_type == JPEG_READ ? JPEG_WRITE : JPEG_READ]) {
		struct dma_chan *dma_chan;

		if (g_drv_data->dma_chan[JPEG_WRITE]) {
			dma_chan = g_drv_data->dma_chan[JPEG_WRITE];
			dma_chan->device->device_control(dma_chan,
					DMA_TERMINATE_ALL, 0);
			dma_release_channel(dma_chan);
		}
		if (g_drv_data->dma_chan[JPEG_READ]) {
			dma_chan = g_drv_data->dma_chan[JPEG_READ];
			dma_chan->device->device_control(dma_chan,
					DMA_TERMINATE_ALL, 0);
			dma_release_channel(dma_chan);
		}

		g_drv_data->dma_chan[JPEG_READ] =
			g_drv_data->dma_chan[JPEG_WRITE] = NULL;

		clk_disable(g_drv_data->clk);
	}

	/* mark jpeg as busy */
	g_drv_data->jpeg_busy[dev_type] = 0;

	return 0;
}

/**
 * designware_jpeg_ioctl - IOCTL for performing different types of RW operation.
 * @file: represents a open file in kernel.
 * @cmd: ioctl command representing particular operation.
 * @buf: buffer for IO.
 */
static long designware_jpeg_ioctl(struct file *file, u32 cmd, unsigned long buf)
{
	enum jpeg_dev_type dev_type;
	s32 status = 0;
	u32 len = 0;

	/* getting the minor number of the caller */
	dev_type = (enum jpeg_dev_type)iminor(file->f_mapping->host);

	switch (dev_type) {
	case JPEG_READ:
		switch (cmd) {
		case JPEGIOC_GET_INFO:
			/* reading header info after decoding finishes */
			if (g_drv_data->operation_complete)
				status = get_jpeg_info((struct jpeg_info *)buf);
			else
				status = -EBUSY;
			break;
		case JPEGIOC_GET_OUT_DATA_SIZE:
			/* get len of data read */
			status = jpeg_get_data(&len);
			if (!status) {
				if (!access_ok(VERIFY_WRITE, (void __user *)buf,
							sizeof(len)))
					return -EFAULT;
				status = put_user(len, (u32 *)buf);
			}
			break;
		default:
			status = -EINVAL;
			break;
		}
		break;

	case JPEG_WRITE:
		switch (cmd) {
		case JPEGIOC_START:
			/* start encoding/decoding */
			status = jpeg_start(buf);
			break;
		case JPEGIOC_SET_SRC_IMG_SIZE:
			/* set src image size */
			jpeg_reset(g_drv_data->regs);
			jpeg_flush();
			if (!buf) {
				dev_dbg(&g_drv_data->pdev->dev, "src image "
						"size can't be zero\n");
				status = -ENODATA;
				break;
			}
			buf = roundup(buf, sizeof(unsigned int));
			g_drv_data->img_size[JPEG_WRITE] = buf;
			break;
		case JPEGIOC_SET_ENC_INFO:
			status = set_jpeg_info((void *)buf, JPEG_ENCODE);
			break;
		case JPEGIOC_SET_DEC_INFO:
			status = set_jpeg_info((void *)buf, JPEG_DECODE);
			break;
		default:
			status = -EINVAL;
			break;
		}
		break;

	default:
		status = -EINVAL;
		break;
	}
	return status;
}

/**
 * designware_jpeg_rmmap - maps allocated memory from driver to user level for
 * read buffer.
 * @file: represents a open file in kernel.
 * @vma: object having info of requested virtual memory region.
 *
 * It allocates memory required for storing the encoded/decoded image data. Then
 * it maps that to virtual space.
 */
static s32 designware_jpeg_rmmap(struct file *file, struct vm_area_struct *vma)
{
	size_t size = vma->vm_end - vma->vm_start;
	s32 status = 0;

	/* memory for both read buffers is allocated in a single call and is
	 ** remaped */
	status = jmalloc(JPEG_READ, size);
	if (!status)
		jpeg_remap(vma, g_drv_data->read_dbuf[0],
				g_drv_data->read_buf[0]);

	return status;
}

/**
 * designware_jpeg_wmmap - maps allocated memory from driver to user level for
 * write buffer.
 * @file: represents a open file in kernel.
 * @vma: object having info of requested virtual memory region.
 *
 * It allocates memory required for passing the src image to jpeg, and then it
 * maps that to virtual space.
 */
static s32 designware_jpeg_wmmap(struct file *file, struct vm_area_struct *vma)
{
	size_t size = vma->vm_end - vma->vm_start;
	s32 status = 0;

	/* memory for both write buffers is allocated in a single call and is
	 ** remaped */
	status = jmalloc(JPEG_WRITE, size);
	if (!status)
		jpeg_remap(vma, g_drv_data->write_dbuf[0],
				g_drv_data->write_buf[0]);

	return status;
}

/* this structure is used to show interface to user level */
static const struct file_operations designware_jpeg_fops[MAX_DEV] = {
	{
		.owner	= THIS_MODULE,
		.open = designware_jpeg_open,
		.release = designware_jpeg_release,
		.unlocked_ioctl = designware_jpeg_ioctl,
		.mmap	= designware_jpeg_rmmap
	}, {
		.owner	= THIS_MODULE,
		.open = designware_jpeg_open,
		.release = designware_jpeg_release,
		.unlocked_ioctl = designware_jpeg_ioctl,
		.mmap	= designware_jpeg_wmmap
	}
};

/*
 * designware_jpeg_irq - jpeg interrput handler. it reads the status of the
 * encoding/decoding and then clears relevant interrupts. then it wakes the
 * sleeping process if both the dma and jpeg interrupts have occured.
 */
static irqreturn_t designware_jpeg_irq(s32 irq, void *dev_id)
{
	unsigned long flags;
	u32 status_reg;

	spin_lock_irqsave(&g_drv_data->lock, flags);
	/* get amount of data transfered to and from jpeg */
	g_drv_data->data_to_jpeg =
		readl(&g_drv_data->regs->CTRL_REG.BYTES_FIFO_TO_CORE);
	g_drv_data->data_from_jpeg =
		readl(&g_drv_data->regs->CTRL_REG.BYTES_CORE_TO_FIFO);

	/* check if interrupt came due to end of encoding/decoding */
	g_drv_data->operation_complete = 0;
	status_reg = readl(&g_drv_data->regs->CTRL_REG.STATUS);
	if (status_reg & END_OF_CONVERSION) {
		/* end of encoding/decoding */
		g_drv_data->operation_complete = 1;

		/*
		 * In SPEAr1310, JPEG doesn't give last burst request to DMA and
		 * thus DMA transfer on output side never completes. As after
		 * this interrupt, we will have only 16 x 4 bytes of data in
		 * FIFO. That will get transfered before we disable DMA.
		 */
		if (!g_drv_data->rdma_flag && cpu_is_spear1310())
			g_drv_data->rdma_flag = 1;

		writel(status_reg & INT_CLR,
				&g_drv_data->regs->CTRL_REG.STATUS);
		writel(JPEG_DISABLE, &g_drv_data->regs->REG0);
	} else {
		/* burst interrupt */
		writel(status_reg & INT_CLR,
				&g_drv_data->regs->CTRL_REG.STATUS);
		writel(status_reg | BURST_INT_CLR,
				&g_drv_data->regs->CTRL_REG.STATUS);
	}

	/* set flag of occurance of jpeg interupt. both dma and jpeg interrupts
	 ** should come before the read call is wake up */
	g_drv_data->rjpeg_flag = 1;
	if (g_drv_data->rdma_flag) {
		g_drv_data->rdma_flag = g_drv_data->rjpeg_flag = 0;
		/* need to do unlock before wake up */
		spin_unlock_irqrestore(&g_drv_data->lock, flags);
		jpeg_wakeup(JPEG_READ);
	} else {
		spin_unlock_irqrestore(&g_drv_data->lock, flags);
	}

	return IRQ_HANDLED;
}

static int designware_jpeg_suspend(struct device *dev)
{
	/* aborting all jpeg operations */
	if (g_drv_data->jpeg_busy[JPEG_READ] ||
			g_drv_data->jpeg_busy[JPEG_WRITE]) {
		jpeg_abort(-EINTR);

		clk_disable(g_drv_data->clk);
	}

	return 0;
}

#ifdef CONFIG_PM
static int designware_jpeg_resume(struct device *dev)
{
	if (g_drv_data->jpeg_busy[JPEG_READ] ||
			g_drv_data->jpeg_busy[JPEG_WRITE]) {
		int ret = clk_enable(g_drv_data->clk);

		if (ret)
			return ret;
	}

	return 0;
}

static SIMPLE_DEV_PM_OPS(designware_jpeg_dev_pm_ops, designware_jpeg_suspend,
		designware_jpeg_resume);
#endif /* CONFIG_PM */

static s32 designware_jpeg_probe(struct platform_device *pdev)
{
	enum jpeg_dev_type dev_type;
	struct resource	*res;
	struct jpeg_plat_data *plat_data = pdev->dev.platform_data;
	s32 irq, status = 0;

	if (!plat_data) {
		dev_err(&pdev->dev, "Invalid platforma data\n");
		return -EINVAL;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "error getting platform resource\n");
		return -EINVAL;
	}

	if (!request_mem_region(res->start, resource_size(res),
				pdev->dev.driver->name)) {
		dev_err(&pdev->dev, "error requesting mem region\n");
		return -EBUSY;
	}

	/* allocate memory for jpeg device */
	g_drv_data = kzalloc(sizeof(*g_drv_data), GFP_KERNEL);
	if (!g_drv_data) {
		dev_err(&pdev->dev, "can not alloc jpeg_dev\n");
		status = -ENOMEM;
		goto err_mem_region;
	}

	g_drv_data->regs = ioremap(res->start, resource_size(res));
	if (!g_drv_data->regs) {
		dev_err(&pdev->dev, "error in ioremap\n");
		status = -ENOMEM;
		goto out_error_kzalloc;
	}

	g_drv_data->pdev = pdev;
	g_drv_data->operation_complete = 1;

	/* attach to irq */
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "irq resource not defined\n");
		status = -ENODEV;
		goto err_ioremap;
	}

	status = request_irq(irq, designware_jpeg_irq, 0, "jpeg-designware",
			g_drv_data);
	if (status < 0) {
		dev_err(&pdev->dev, "can not get IRQ\n");
		goto err_ioremap;
	}

	g_drv_data->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(g_drv_data->clk)) {
		status = PTR_ERR(g_drv_data->clk);
		goto out_error_irq_alloc;
	}

	/* register char driver */
	status = alloc_chrdev_region(&g_drv_data->char_dev_num[JPEG_READ], 0,
			MAX_DEV, DRIVER_NAME);
	if (status) {
		dev_err(&pdev->dev, "unable to register jpeg driver\n");
		goto out_error_clk;
	}

	g_drv_data->char_dev_num[JPEG_WRITE] =
		MKDEV(MAJOR(g_drv_data->char_dev_num[JPEG_READ]), 1);

	for (dev_type = JPEG_READ; dev_type < MAX_DEV; dev_type += JPEG_WRITE) {
		g_drv_data->char_dev[dev_type].owner = THIS_MODULE;
		cdev_init(&g_drv_data->char_dev[dev_type],
				&designware_jpeg_fops[dev_type]);
		status = cdev_add(&g_drv_data->char_dev[dev_type],
				g_drv_data->char_dev_num[dev_type], 1);
		if (status)
			goto out_error_chrdev_add;

		init_waitqueue_head(&g_drv_data->wait_queue[dev_type]);
	}

	spin_lock_init(&g_drv_data->lock);
	platform_set_drvdata(pdev, g_drv_data);

	/* set tx and rx regs for dma xfers */
	g_drv_data->slaves = plat_data;
	dma_cap_set(DMA_SLAVE, g_drv_data->mask);

	/* we need to pass physical address here */
	g_drv_data->tx_reg =
		(dma_addr_t)&((struct jpeg_regs *)(res->start))->FIFO_IN;
	g_drv_data->rx_reg =
		(dma_addr_t)&((struct jpeg_regs *)(res->start))->FIFO_OUT;

	if (cpu_is_spear1310())
		djpeg_burst = 16;
	else
		djpeg_burst = 8;

	return status;

out_error_chrdev_add:
	for (dev_type = JPEG_READ; dev_type < MAX_DEV; dev_type += JPEG_WRITE)
		cdev_del(&(g_drv_data->char_dev[dev_type]));

	unregister_chrdev_region(g_drv_data->char_dev_num[JPEG_READ], MAX_DEV);

out_error_clk:
	clk_put(g_drv_data->clk);

out_error_irq_alloc:
	free_irq(irq, g_drv_data);

err_ioremap:
	iounmap(g_drv_data->regs);

out_error_kzalloc:
	kfree(g_drv_data);

err_mem_region:
	release_mem_region(res->start, resource_size(res));

	return status;
}

static s32 designware_jpeg_remove(struct platform_device *pdev)
{
	enum jpeg_dev_type dev_type;
	s32 irq;
	struct resource	*res;

	if (!g_drv_data)
		return 0;

	/* prevent double remove */
	platform_set_drvdata(pdev, NULL);

	designware_jpeg_suspend(&pdev->dev);

	/*unregister char driver */
	for (dev_type = JPEG_READ; dev_type < MAX_DEV; dev_type += JPEG_WRITE)
		cdev_del(&(g_drv_data->char_dev[dev_type]));

	unregister_chrdev_region(g_drv_data->char_dev_num[JPEG_READ], MAX_DEV);

	clk_put(g_drv_data->clk);

	/* release irq */
	irq = platform_get_irq(pdev, 0);
	if (irq >= 0)
		free_irq(irq, g_drv_data);

	iounmap(g_drv_data->regs);

	/* free memory of g_drv_data */
	kfree(g_drv_data);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "remove:error getting platform resource\n");
		return -EINVAL;
	}

	release_mem_region(res->start, resource_size(res));

	return 0;
}

static struct platform_driver designware_jpeg_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.bus = &platform_bus_type,
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &designware_jpeg_dev_pm_ops,
#endif
	},
	.probe = designware_jpeg_probe,
	.remove = __devexit_p(designware_jpeg_remove),
};

static s32 __init designware_jpeg_init(void)
{
	platform_driver_register(&designware_jpeg_driver);

	return 0;
}
module_init(designware_jpeg_init);

static void __exit designware_jpeg_exit(void)
{
	platform_driver_unregister(&designware_jpeg_driver);
}
module_exit(designware_jpeg_exit);

MODULE_AUTHOR("viresh kumar");
MODULE_DESCRIPTION("Synopsys Designware JPEG Encoder/Decoder");
MODULE_LICENSE("GPL");
