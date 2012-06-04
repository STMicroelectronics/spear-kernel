/*
 * drivers/char/designware_jpeg.h
 *
 * Synopsys Designware JPEG controller header file
 *
 * Copyright (C) 2010 ST Microelectronics
 * Viresh Kumar<viresh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef _DESIGNWARE_JPEG_H
#define _DESIGNWARE_JPEG_H

#include <linux/cdev.h>
#include <linux/clk.h>
#include <linux/designware_jpeg_usr.h>
#include <linux/dmaengine.h>
#include <linux/io.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <mach/misc_regs.h>

#define DRIVER_NAME	"jpeg-designware"

#define NUM_OF_BUF	2

/*Timeout for sleep*/
#define JPEG_TIMEOUT 2500

/* Constants for configuring JPEG Registers */
/* jpeg0 reg */
#define JPEG_ENABLE		1
#define JPEG_DISABLE		0

/* jpeg1 reg */
#define	NUM_CLR_CMP(x)		(((x) & 0x3) + 1)
#define	SET_NUM_CLR_CMP(x)	(((x) & 0x3) - 1)
#define	RST_ENABLE(x)		((x) << 2)
#define	IS_RST_ENABLE(x)	(((x) & 0x4) ? 1 : 0)
#define	DECODER_ENABLE		(1 << 3)
#define	ENCODER_ENABLE		(~DECODER_ENABLE)
#define	COL_SPC_TYPE(x)		((((x) >> 4) & 0x3) + 1)
#define	SET_COL_SPC_TYPE(x)	((((x) & 0x3) - 1) << 4)
#define	NUM_CMP_SCAN_HDR(x)	((((x) >> 6) & 0x3) + 1)
#define	SET_NUM_CMP_SCAN_HDR(x)	((((x) & 0x3) - 1) << 6)
#define	HDR_ENABLE		(1 << 8)
#define	HDR_DISABLE		(~HDR_ENABLE)
#define	Y_SIZE(x)		(((x) >> 16) & 0xFFFF)
#define	SET_Y_SIZE(x)		(((x) & 0xFFFF) << 16)

/* jpeg2 reg */
#define	NUM_MCU(x)		(((x) & 0x3FFFFFF) + 1)
#define	SET_NUM_MCU(x)		((((x) & 0x3FFFFFF) - 1))

/* jpeg3 reg */
#define	RST_NUM_MCU(x)		(((x) & 0xFFFF) + 1)
#define	SET_RST_NUM_MCU(x)	(((x) & 0xFFFF) - 1)
#define	X_SIZE(x)		(((x) >> 16) & 0xFFFF)
#define	SET_X_SIZE(x)		(((x) & 0xFFFF) << 16)

/* jpeg4-7 reg */
#define	HUFF_TBL_DC(x)		((x) & 0x1)
#define	HUFF_TBL_AC(x)		(((x) >> 1) & 0x1)
#define	SET_HUFF_TBL_AC(x)	(((x) & 0x1) << 1)
#define	QUANT_TBL(x)		(((x) >> 2) & 0x3)
#define	SET_QUANT_TBL(x)	(((x) & 0x3) << 2)
#define	NUM_DATA_UNITS(x)	((((x) >> 4) & 0xF) + 1)
#define	SET_NUM_DATA_UNITS(x)	((((x) & 0xF) - 1) << 4)
#define	HORZ_SAMP(x)		(((x) >> 8) & 0xF)
#define	SET_HORZ_SAMP(x)	(((x) & 0xF) << 8)
#define	VERT_SAMP(x)		(((x) >> 12) & 0xF)
#define	SET_VERT_SAMP(x)	(((x) & 0xF) << 12)

/*constants for configuring jpeg controller registers*/
/*control status reg*/
#define	INT_CLR				(~1)
#define	BURST_INT_CLR			1
#define	NUM_BYTE_INVALID_LWORD(x)	(((x) >> 1) & 3)
#define	JPEG_RESET			(1 << 30)
#define	NUM_LLI(x)			(((x) & 0x7FFF) << 3)
#define SPEAR1310_NUM_LLI(x)		(((x) & 0x7FFF) << 4)
#define	MAX_LLI				0x7FFF
#define LAST_XFER_WORDS(x)		(((x-1) & 0xF) << 20)
#define	END_OF_CONVERSION		(1 << 31)

/*burst count before interrupt reg*/
#define	NUM_BURST_TX			0x7FFFFFFF
#define	BURST_COUNT_ENABLE		(1 << 31)

/* jpeg controller registers */
struct jpeg_ctrl_regs {
	u32 STATUS;
	u32 BYTES_FIFO_TO_CORE;
	u32 BYTES_CORE_TO_FIFO;
	u32 BYTES_BEF_INT;
};

/* jpeg registers */
struct jpeg_regs {
	u32 REG0;
	u32 REG1;
	u32 REG2;
	u32 REG3;
	u32 REG4;
	u32 REG5;
	u32 REG6;
	u32 REG7;
	u32 pad1[120];
	struct jpeg_ctrl_regs CTRL_REG;
	u32 pad2[124];
	u32 FIFO_IN;
	u32 pad3[127];
	u32 FIFO_OUT;
	u32 pad4[127];
	u32 QNT_MEM;
	u32 pad5[255];
	u32 HMIN_MEM;
	u32 pad6[255];
	u32 HBASE_MEM;
	u32 pad7[255];
	u32 HSYMB_MEM;
	u32 pad8[255];
	u32 DHT_MEM;
	u32 pad9[255];
	u32 HENC_MEM;
};

/* JPEG Device types */
enum jpeg_dev_type {
	JPEG_READ,
	JPEG_WRITE,
	MAX_DEV
};

/**
 * struct jpeg_dev - jpeg device structure containing device specific info.
 * @pdev: pointer to platform dev structure of jpeg.
 * @char_dev: structure representing jpeg char device.
 * @char_dev_num: type used to represent jpeg device numbers within the kernel.
 * @lock: spinlock required for critical section safety
 * @regs: base address of jpeg device registers
 * @tx_reg: physical address of tx register
 * @rx_reg: physical address of rx register
 * @slaves: dma specific slaves info
 * @dma_client: dma specific client structure
 * @dma_chan: allocated dma channel
 * @sg: pointer to head of scatter list
 * @sg_len: number of scatter list nodes allocated
 * @cookie: cookie value returned from dma
 * @wait_queue: wait queue for reading and writing
 * @flag_arrived: flags required for sleep, using wait_event_interruptible,
 *		during read and write.
 * @rdma_flag: flag indicating dma completion interrupt
 * @rjpeg_flag: flag indicating jpeg completion interrupt
 * @access_width: access width of jpeg for dma read and write
 * @buf_size: buffer size allcated for read and write
 * @read_buf: pointers to read bufs
 * @write_buf: pointer to write bufs
 * @img_size: src and dest image size
 * @current_wbuf: current write buf used for encoding/decoding
 * @current_rbuf: current read buf used for encoding/decoding
 * @operation_complete: flag indicating jpeg encoding/decoding status
 * @read_complete: flag indicating if all encoded/decoded data is read or not
 * @first_write: flag indicating read is called for the first time or not
 * @jpeg_busy: flag indicating jpeg is busy doing encoding/decoding
 * @data_to_jpeg: total data transferred from jpeg controller to jpeg
 * @data_from_jpeg: total data transferred from jpeg to jpeg controller
 * @error: error encountered while encoding/decoding
 *
 * This structure is used by the driver to used information related to jpeg
 * device within the functions of source file. Instance of this structure is
 * created at probe.
 */
struct jpeg_dev {
	/* Device registration */
	struct platform_device *pdev;
	struct cdev char_dev[MAX_DEV];
	dev_t char_dev_num[MAX_DEV];
	struct clk *clk;

	/* spin lock */
	spinlock_t lock;

	/* JPEG register addresses */
	struct jpeg_regs *regs;
	dma_addr_t tx_reg;
	dma_addr_t rx_reg;

	/*DMA transfer*/
	struct jpeg_plat_data *slaves;
	struct dma_chan *dma_chan[MAX_DEV];
	dma_cap_mask_t mask;
	struct scatterlist *sg[MAX_DEV];
	u32 sg_len[MAX_DEV];
	dma_cookie_t cookie[MAX_DEV];

	/* wait queue and dma flags */
	wait_queue_head_t wait_queue[MAX_DEV];
	u32 flag_arrived[MAX_DEV];
	u32 rdma_flag;
	u32 rjpeg_flag;

	/*Read Write Buffers*/
	u32 buf_size[MAX_DEV];
	void *read_buf[NUM_OF_BUF];
	void *write_buf[NUM_OF_BUF];
	dma_addr_t read_dbuf[NUM_OF_BUF];
	dma_addr_t write_dbuf[NUM_OF_BUF];
	u32 img_size[MAX_DEV];
	s32 current_wbuf;
	s32 current_rbuf;

	/*Flags*/
	enum jpeg_operation operation;
	int hdr_enable;
	u8 operation_complete;
	u8 read_complete;
	u8 first_write;
	u8 jpeg_busy[MAX_DEV];

	/*Data Counts*/
	u32 data_to_jpeg;
	u32 data_from_jpeg;

	/*Error encountered while last decoding*/
	s32 error;
};

/* resetting jpeg ip */
static inline void jpeg_reset(struct jpeg_regs *regs)
{
	u32 tmp = 0;
	void __iomem *addr;

	/*done bcoz h/w doesn't reset this register.*/
	writel(0, &regs->CTRL_REG.BYTES_BEF_INT);

	writel(JPEG_DISABLE, &regs->REG0);
	writel(JPEG_RESET, &regs->CTRL_REG.STATUS);

	/*
	 * we need to reset ip from misc due to issue: hardware not resetting
	 * jpeg completely.
	 */
#ifdef CONFIG_ARCH_SPEAR13XX
	if (cpu_is_spear1310())
		addr = VA_SPEAR1310_PERIP1_SW_RST;
	else
		addr = VA_PERIP1_SW_RST;
#else
	addr = VA_PERIP1_SOF_RST;
#endif
	tmp = readl(addr);
	writel(((1 << JPEG_SOF_RST) | tmp), addr);
	writel(tmp, addr);
}

static inline unsigned long
jmemcpy(void *to, const void __user *from, unsigned long n, int width)
{
	switch (width) {
	case 1:
		while (n) {
			n -= width;
			*((char *)to) = *((char *)from);
			to += width;
			from += width;
		}
		break;
	case 2:
		while (n) {
			n -= width;
			*((short *)to) = *((short *)from);
			to += width;
			from += width;
		}
		break;
	case 4:
		while (n) {
			n -= width;
			*((int *)to) = *((int *)from);
			to += width;
			from += width;
		}
		break;
	}
	return n;
}

static inline unsigned long
__must_check jcopy_from_user(void *to, const void __user *from, unsigned long n,
		int width)
{
	if (access_ok(VERIFY_READ, from, n))
		n = jmemcpy(to, from, n, width);
	return n;
}

static inline unsigned long
__must_check jcopy_to_user(void __user *to, const void *from, unsigned long n,
		int width)
{
	if (access_ok(VERIFY_WRITE, to, n))
		n = jmemcpy(to, from, n, width);
	return n;
}

#endif /* _DESIGNWARE_JPEG_H */
