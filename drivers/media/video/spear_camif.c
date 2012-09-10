/*
 * V4L2 SoC Camera driver for SPEAr Camera Interface(CAMIF)
 *
 * Copyright (C) 2011 ST Microelectronics
 * Bhupesh Sharma <bhupesh.sharma@st.com>
 *
 * Based on PXA SoC camera driver
 * Copyright (C) 2006, Sascha Hauer, Pengutronix
 * Copyright (C) 2008, Guennadi Liakhovetski <kernel@pengutronix.de>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

/*
 * TODO:
 *
 * 1. Add support for camera sense (as generally PCLK is used to latch
 * data from the sensor.
 *
 * 2. Interleaving dma channels are not supported as of now. We support
 * even dma channel only as of now. Add interleaving support later, i.e.
 * odd and even frames. It is also not very clear as of now how the
 * driver should perform DMA in such a case:
 * Should we DMA the even and odd lines received from the interface onto
 * different locations in the DDR and the application should be
 * intelligent enough to pick up the data from these locations
 * automatically in a proper fashion?
 */

#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/dw_dmac.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/v4l2-mediabus.h>
#include <linux/platform_device.h>
#include <linux/version.h>
#include <linux/videodev2.h>
#include <linux/vmalloc.h>

#include <media/soc_camera.h>
#include <media/soc_mediabus.h>
#include <media/v4l2-common.h>
#include <media/v4l2-dev.h>
#include <media/videobuf-dma-sg.h>

#include <plat/camif.h>

/* CAMIF register map */
#define CAMIF_CTRL		0x00	/* Control Register */
#define CAMIF_EFEC		0x04	/* Even Field Embedded Codes Register */
#define CAMIF_CSTARTP		0x08	/* Crop Start Point Register */
#define CAMIF_CSTOPP		0x0C	/* Crop Stop Point Register */
#define CAMIF_IR_DUR		0x10	/* Interrupt & DMA Mask Register */
#define CAMIF_STATUS		0x14	/* Interrupt Status Register */
#define CAMIF_DMA_PTR		0x18	/* DMA Pointer Register */
#define CAMIF_OFEC		0x1C	/* Odd Field Embedded Codes Register */

/* control register bits */
#define CTRL_RGB_ALPHA_VALUE(a)	((a) << 24)
#define CTRL_INTL_EN		BIT(23)
#define CTRL_SGA_TFR_EN		BIT(22)
#define CTRL_DMA_BURST_SHIFT	19
#define CTRL_DMA_BURST_MASK	0xffc7ffff
#define CTRL_DMA_BSIZE(a, b)	((a) | ((b) << CTRL_DMA_BURST_SHIFT))
#define CTRL_WAIT_STATE_EN	BIT(18)
#define CTRL_CROP_SELECT	BIT(17)
#define CTRL_EAV_SEL(a)		((a) << 16)
#define CTRL_IF_SYNC_TYPE(a)	((a) << 12)
#define CTRL_IF_TRANS(a)	((a) << 8)
#define CTRL_VS_POL(a)		((a) << 7)
#define CTRL_HS_POL(a)		((a) << 6)
#define CTRL_PCK_POL(a)		((a) << 5)
#define CTRL_EMB(a)		((a) << 4)
#define CTRL_CAPTURE_MODES(a)	((a) & 0xf)
#define CTRL_VS_POL_HI		BIT(7)
#define CTRL_HS_POL_HI		BIT(6)
#define CTRL_PCK_POL_HI		BIT(5)

/* camif supported storage swappings */
enum camif_transformation {
	DISABLED = 0,
	YUVCbYCrY,
	YUVCrYCbY,
	RGB888A,
	BGR888A,
	ARGB888,
	ABGR888,
	RGB565,
	BGR565,
	RGB888,
	JPEG,
};

/* interrupt and dma mask register bits */
#define IR_DUR_MASK_BREQ	BIT(15)
#define IR_DUR_UNMASK_BREQ	(~BIT(15))
#define IR_DUR_MASK_SREQ	BIT(14)
#define IR_DUR_UNMASK_SREQ	(~BIT(14))
#define IR_DUR_DMA_DUAL_EN	BIT(12)
#define IR_DUR_DMA_DUAL_DIS	(~BIT(12))
#define IR_DUR_DMA_EN		BIT(11)
#define IR_DUR_DMA_DIS		(~BIT(11))
#define IR_DUR_FRAME_START_EN	BIT(6)
#define IR_DUR_FRAME_START_DIS	(~BIT(6))
#define IR_DUR_FRAME_END_EN	BIT(5)
#define IR_DUR_FRAME_END_DIS	(~BIT(5))
#define IR_DUR_LINE_END_EN	BIT(4)
#define IR_DUR_LINE_END_DIS	(~BIT(4))

/* interrupt status register bits */
#define IRQ_STATUS_FRAME_START	BIT(6)
#define IRQ_STATUS_FRAME_END	BIT(5)
#define IRQ_STATUS_LINE_END	BIT(4)

/* embedded codes */
#define ITU656_EMBED_EVEN_CODE	0xABB6809D
#define ITU656_EMBED_ODD_CODE	0xECF1C7DA

/* camif version */
#define CAM_IF_VERSION_CODE	KERNEL_VERSION(0, 0, 1)

/* camif bus capabilities */
#define CAM_IF_BUS_FLAGS	(V4L2_MBUS_MASTER |			\
				V4L2_MBUS_HSYNC_ACTIVE_HIGH |		\
				V4L2_MBUS_HSYNC_ACTIVE_LOW |		\
				V4L2_MBUS_VSYNC_ACTIVE_HIGH |		\
				V4L2_MBUS_VSYNC_ACTIVE_LOW |		\
				V4L2_MBUS_PCLK_SAMPLE_RISING |		\
				V4L2_MBUS_PCLK_SAMPLE_FALLING |		\
				V4L2_MBUS_DATA_ACTIVE_HIGH |		\
				V4L2_MBUS_DATA_ACTIVE_LOW)

/* crop masks */
#define CROP_V_MASK(a)		(((a) & 0xffff) << 15)
#define CROP_H_MASK(a)		((a) & 0xffff)

/* video memory limit */
#define MAX_VIDEO_MEM		16	/* MiB */

/* buffer count */
#define CAMIF_MIN_BUFFER_CNT	3

/* camif image size boundary */
#define CAMIF_MAX_WIDTH		1920
#define CAMIF_MAX_HEIGHT	1200

/* number of CAMIF regs which need to be banked */
#define CAMIF_REG_COUNT		6

/* possible camif power states */
enum camif_power_state {
	CAMIF_SHUTDOWN = 0,
	CAMIF_BRINGUP,
};

/* Buffer for one video frame */
struct camif_buffer {
	/* common v4l buffer stuff -- must be first */
	struct videobuf_buffer vb;
	enum v4l2_mbus_pixelcode code;
	/* rest of the details */
	int inwork;
	struct dma_async_tx_descriptor *desc;
};

/* camif device */
struct camif {
	/* soc camera related */
	struct soc_camera_host ici;
	struct soc_camera_device *icd;
	struct clk *clk;

	/* resource related */
	int line_irq;
	int frm_start_end_irq;
	void __iomem *base;

	/* dma related */
	struct dma_chan *chan;
	dma_cookie_t cookie;

	/* locks */
	spinlock_t lock;

	/* dma queue */
	struct list_head dma_queue;

	/* flags to indicate states */
	int frame_cnt;

	/* current videobuffer */
	struct camif_buffer *cur_frm;

	/* current format supported by camif */
	struct v4l2_format fmt;

	/* camif controller data specific to a platform */
	struct camif_controller *pdata;

	/* banked copy of the camif regs */
	u32 banked_regs[CAMIF_REG_COUNT];

	/* physical variant of camif->base */
	dma_addr_t phys_base_addr;

	/* used to store the crop window */
	struct v4l2_rect crop;

	/* keep a track for present power state */
	bool is_running;

	/* keep a track of 1st frame-end interrupt */
	bool first_frame_end;

	/* timer to handle DMA hang case */
	struct timer_list idle_timer;

	/* camif instance id */
	int id;

	/* keep track of whether we have applied some quirks */
	bool sw_workaround_applied;
	bool hw_workaround_applied;
};

/* camif interrupt selection types */
enum camif_interrupt_config {
	ENABLE_ALL = 0,
	DISABLE_ALL,
	ENABLE_FRAME_START_INT,
	ENABLE_FRAME_END_INT,
	ENABLE_HSYNC_VSYNC_INT,
	DISABLE_HSYNC_INT,
	DISABLE_FRAME_START_INT,
	DISABLE_FRAME_END_INT,
	DISABLE_HSYNC_VYSNC_INT,
	ENABLE_EVEN_FIELD_DMA,
	DISABLE_EVEN_FIELD_DMA,
	ENABLE_BOTH_FIELD_DMA,
	DISABLE_BOTH_FIELD_DMA,
};

/* camif supported burst sizes */
enum camif_burst_size {
	CAMIF_BURST_SIZE_1 = 1,
	CAMIF_BURST_SIZE_4 = 4,
	CAMIF_BURST_SIZE_8 = 8,
	CAMIF_BURST_SIZE_16 = 16,
	CAMIF_BURST_SIZE_32 = 32,
	CAMIF_BURST_SIZE_64 = 64,
	CAMIF_BURST_SIZE_128 = 128,
	CAMIF_BURST_SIZE_256 = 256,
};

/*
 * here we specify the pixel formats, to which the host controller can convert
 * some other formats on the hardware.
 *
 * For e.g. consider that a client (sensor) supports the mode
 * V4L2_MBUS_FMT_UYVY8_2X8. Then we have a entry for this mode here if the host
 * recognises that it supports that format natively and apart from serving it to
 * the application in the pass-through mode, it can also convert it to
 * some other format (for e.g. say the planar V4L2_PIX_FMT_YUV422P format)
 */
static const struct soc_mbus_pixelfmt camif_formats[] = {
	/* added */
	[V4L2_MBUS_FMT_UYVY8_2X8] = {
		.fourcc			= V4L2_PIX_FMT_YUYV,
		.name			= "YUYV",
		.bits_per_sample	= 8,
		.packing		= SOC_MBUS_PACKING_2X8_PADHI,
		.order			= SOC_MBUS_ORDER_BE,
	},

	/* 3-byte RGB-888 received -> 4-byte RGBa stored */
	[V4L2_MBUS_FMT_RGB24_2X8_LE] = {
		.fourcc			= V4L2_PIX_FMT_RGB32,
		.name			= "RGBa 32 bit",
		.bits_per_sample	= 8,
		.packing		= SOC_MBUS_PACKING_2X8_PADHI,
		.order			= SOC_MBUS_ORDER_LE,
	},
	/* 3-byte BGR-888 received -> 4-byte BGRa stored */
	[V4L2_MBUS_FMT_BGR24_2X8_LE] = {
		.fourcc			= V4L2_PIX_FMT_BGR32,
		.name			= "BGRa 32 bit",
		.bits_per_sample	= 8,
		.packing		= SOC_MBUS_PACKING_2X8_PADHI,
		.order			= SOC_MBUS_ORDER_LE,
	},
};

static void camif_do_idle(unsigned long _arg);

/* reset camif */
static void camif_module_enable(struct camif *camif, bool enable)
{
	if (camif->pdata->config->camif_module_enable)
		camif->pdata->config->camif_module_enable(camif->id, enable);
}

/*
 * Helper routine to configure CAMIF interrupts:
 * Note that we keep the SINGLE Requests from the CAMIF
 * side disabled as this may cause a DMA stuck case
 */
static void camif_configure_interrupts(struct camif *camif,
		enum camif_interrupt_config config)
{
	int intr_dma_mask_reg = readl(camif->base + CAMIF_IR_DUR);

	switch (config) {
	case ENABLE_ALL:
		intr_dma_mask_reg |= IR_DUR_LINE_END_EN | IR_DUR_FRAME_END_EN |
				IR_DUR_FRAME_START_EN | IR_DUR_DMA_EN |
				IR_DUR_DMA_DUAL_EN;
		intr_dma_mask_reg &= IR_DUR_UNMASK_BREQ & IR_DUR_UNMASK_SREQ;
		writel(intr_dma_mask_reg, camif->base + CAMIF_IR_DUR);
		break;
	case DISABLE_ALL:
		intr_dma_mask_reg |= IR_DUR_MASK_BREQ | IR_DUR_MASK_SREQ;
		intr_dma_mask_reg &= IR_DUR_DMA_DIS & IR_DUR_DMA_DUAL_DIS &
				IR_DUR_FRAME_START_DIS & IR_DUR_FRAME_END_DIS &
				IR_DUR_LINE_END_DIS;
		writel(intr_dma_mask_reg, camif->base + CAMIF_IR_DUR);
		break;
	case ENABLE_FRAME_START_INT:
		intr_dma_mask_reg |= IR_DUR_FRAME_START_EN;
		writel(intr_dma_mask_reg, camif->base + CAMIF_IR_DUR);
		break;
	case ENABLE_FRAME_END_INT:
		intr_dma_mask_reg |= IR_DUR_FRAME_END_EN;
		writel(intr_dma_mask_reg, camif->base + CAMIF_IR_DUR);
		break;
	case ENABLE_HSYNC_VSYNC_INT:
		intr_dma_mask_reg |= IR_DUR_FRAME_START_EN |
			IR_DUR_LINE_END_EN | IR_DUR_FRAME_END_EN;
		writel(intr_dma_mask_reg, camif->base + CAMIF_IR_DUR);
		break;
	case DISABLE_HSYNC_INT:
		intr_dma_mask_reg &= IR_DUR_LINE_END_DIS;
		writel(intr_dma_mask_reg, camif->base + CAMIF_IR_DUR);
		break;
	case DISABLE_FRAME_START_INT:
		intr_dma_mask_reg &= IR_DUR_FRAME_START_DIS;
		writel(intr_dma_mask_reg, camif->base + CAMIF_IR_DUR);
		break;
	case DISABLE_FRAME_END_INT:
		intr_dma_mask_reg &= IR_DUR_FRAME_END_DIS;
		writel(intr_dma_mask_reg, camif->base + CAMIF_IR_DUR);
		break;
	case DISABLE_HSYNC_VYSNC_INT:
		intr_dma_mask_reg &= IR_DUR_FRAME_START_DIS &
				IR_DUR_LINE_END_DIS & IR_DUR_FRAME_END_DIS;
		writel(intr_dma_mask_reg, camif->base + CAMIF_IR_DUR);
		break;
	case ENABLE_EVEN_FIELD_DMA:
		intr_dma_mask_reg |= IR_DUR_DMA_EN | IR_DUR_MASK_SREQ;
		intr_dma_mask_reg &= IR_DUR_DMA_DUAL_DIS & IR_DUR_UNMASK_BREQ;
		writel(intr_dma_mask_reg, camif->base + CAMIF_IR_DUR);
		break;
	case DISABLE_EVEN_FIELD_DMA:
		intr_dma_mask_reg &= IR_DUR_DMA_DIS;
		intr_dma_mask_reg |= IR_DUR_MASK_BREQ | IR_DUR_MASK_SREQ;
		writel(intr_dma_mask_reg, camif->base + CAMIF_IR_DUR);
		break;
	case ENABLE_BOTH_FIELD_DMA:
		intr_dma_mask_reg |= IR_DUR_DMA_EN | IR_DUR_DMA_DUAL_EN |
			IR_DUR_MASK_SREQ;
		intr_dma_mask_reg &= IR_DUR_UNMASK_BREQ;
		writel(intr_dma_mask_reg, camif->base + CAMIF_IR_DUR);
		break;
	case DISABLE_BOTH_FIELD_DMA:
		intr_dma_mask_reg &= IR_DUR_DMA_DIS & IR_DUR_DMA_DUAL_DIS;
		intr_dma_mask_reg |= IR_DUR_MASK_BREQ | IR_DUR_MASK_SREQ;
		writel(intr_dma_mask_reg, camif->base + CAMIF_IR_DUR);
		break;
	default:
		dev_err(camif->ici.v4l2_dev.dev,
				"invalid interrupt config type\n");
		break;
	}
}

/* Program new DMA burst settings for CAMIF */
static void camif_prog_dma_burst(struct camif *camif,
					enum camif_burst_size size)
{
	u32 ctrl = readl(camif->base + CAMIF_CTRL);
	u32 ctrl_save = ctrl & CTRL_DMA_BURST_MASK;

	switch (size) {
	case CAMIF_BURST_SIZE_1:
		ctrl = CTRL_DMA_BSIZE(ctrl_save, BURST_SIZE_1);
		break;
	case CAMIF_BURST_SIZE_4:
		ctrl = CTRL_DMA_BSIZE(ctrl_save, BURST_SIZE_4);
		break;
	case CAMIF_BURST_SIZE_8:
		ctrl = CTRL_DMA_BSIZE(ctrl_save, BURST_SIZE_8);
		break;
	case CAMIF_BURST_SIZE_16:
		ctrl = CTRL_DMA_BSIZE(ctrl_save, BURST_SIZE_16);
		break;
	case CAMIF_BURST_SIZE_32:
		ctrl = CTRL_DMA_BSIZE(ctrl_save, BURST_SIZE_32);
		break;
	case CAMIF_BURST_SIZE_64:
		ctrl = CTRL_DMA_BSIZE(ctrl_save, BURST_SIZE_64);
		break;
	case CAMIF_BURST_SIZE_128:
		ctrl = CTRL_DMA_BSIZE(ctrl_save, BURST_SIZE_128);
		break;
	case CAMIF_BURST_SIZE_256:
		ctrl = CTRL_DMA_BSIZE(ctrl_save, BURST_SIZE_256);
		break;
	}

	writel(ctrl, camif->base + CAMIF_CTRL);
}

/* Program default values for CAMIF control register */
static void camif_prog_default_ctrl(struct camif *camif)
{
	u32 ctrl = 0, ctrl_save;
	bool emb_synchro = 0;
	bool eav_sel = 1;

	ctrl_save = readl(camif->base + CAMIF_CTRL);

	if (camif->pdata->config->sync_type == EMBEDDED_SYNC) {
		emb_synchro = 1;
		eav_sel = 0;
		writel(ITU656_EMBED_EVEN_CODE, camif->base + CAMIF_EFEC);
		writel(ITU656_EMBED_ODD_CODE, camif->base + CAMIF_OFEC);
	}

	ctrl = CTRL_DMA_BSIZE(ctrl_save & CTRL_DMA_BURST_MASK,
				camif->pdata->config->burst_size) |
		CTRL_EAV_SEL(eav_sel) |
		CTRL_IF_SYNC_TYPE(emb_synchro) |
		CTRL_IF_TRANS(DISABLED) |
		CTRL_VS_POL(camif->pdata->config->vsync_polarity) |
		CTRL_HS_POL(camif->pdata->config->hsync_polarity) |
		CTRL_PCK_POL(camif->pdata->config->pclk_polarity) |
		CTRL_EMB(camif->pdata->config->sync_type) |
		CTRL_CAPTURE_MODES(camif->pdata->config->capture_mode);

	writel(ctrl, camif->base + CAMIF_CTRL);
}

/* Reset the CAMIF registers to their default values */
static void camif_reset_regs(struct camif *camif)
{
	writel(0x0, camif->base + CAMIF_CTRL);
	writel(0x0, camif->base + CAMIF_IR_DUR);
	writel(0x0, camif->base + CAMIF_EFEC);
	writel(0x0, camif->base + CAMIF_OFEC);
	writel(0x0, camif->base + CAMIF_CSTARTP);
	writel(0x0, camif->base + CAMIF_CSTOPP);
}

/* Save copies of the CAMIF registers */
static void camif_save_regs(struct camif *camif)
{
	int i = 0;

	camif->banked_regs[i++] = readl(camif->base + CAMIF_CTRL);
	camif->banked_regs[i++] = readl(camif->base + CAMIF_IR_DUR);
	camif->banked_regs[i++] = readl(camif->base + CAMIF_EFEC);
	camif->banked_regs[i++] = readl(camif->base + CAMIF_OFEC);
	camif->banked_regs[i++] = readl(camif->base + CAMIF_CSTARTP);
	camif->banked_regs[i++] = readl(camif->base + CAMIF_CSTOPP);
}

/* Restore the save copies of the CAMIF registers */
static void camif_restore_regs(struct camif *camif)
{
	int i = 0;

	writel(camif->banked_regs[i++], camif->base + CAMIF_CTRL);
	writel(camif->banked_regs[i++], camif->base + CAMIF_IR_DUR);
	writel(camif->banked_regs[i++], camif->base + CAMIF_EFEC);
	writel(camif->banked_regs[i++], camif->base + CAMIF_OFEC);
	writel(camif->banked_regs[i++], camif->base + CAMIF_CSTARTP);
	writel(camif->banked_regs[i++], camif->base + CAMIF_CSTOPP);
}

/*
 * Recover from the classic producer/consumer videobuf throttle mismatch issue.
 */
void camif_set_sw_recovery_state(struct camif *camif)
{
	/* stop all interrupts */
	camif_configure_interrupts(camif, DISABLE_ALL);

	/* bank the camif regs */
	camif_save_regs(camif);

	/* mark that we have no more active frames */
	camif->cur_frm = NULL;

	/* again ignore the 1st frame coming from the sensor */
	camif->first_frame_end = true;

	/* applied a SW throttle control */
	camif->sw_workaround_applied = true;
}

/*
 * Recover from undefined camif state:
 * It has been noticed that the CAMIF module will hang the system
 * while performing a DMA request. This is purel random and then we
 * are left in an state where we need to disable the entire
 * CAMIF module (as CAMIF has no specific bit to enable/disable the
 * module and also there is no mechanism to flush the data present
 * in the CAMIF line buffer
 */
static void camif_set_hw_recovery_state(struct camif *camif)
{
	/* stop all interrupts */
	camif_configure_interrupts(camif, DISABLE_ALL);

	/* bank the camif regs */
	camif_save_regs(camif);

	/* terminate any pending DMA transfers */
	if (camif->chan)
		dmaengine_terminate_all(camif->chan);

	/* turn off camif module */
	camif_module_enable(camif, false);

	/* mark that we have no more active frames */
	camif->cur_frm = NULL;

	/* again ignore the 1st frame coming from the sensor */
	camif->first_frame_end = true;

	/* applied a HW reset */
	camif->hw_workaround_applied = true;
}

/* Start CAMIF functionality and DMA transfers */
static void camif_start_capture(struct camif *camif,
			enum camif_power_state state)
{
	/*
	 * HW related changes are common to all bringup/power-alive
	 * states
	 */
	/* turn on the CAMIF module */
	camif_module_enable(camif, true);

	/* enable CAMIF clk */
	clk_prepare_enable(camif->clk);

	/* reset global flags */
	camif->first_frame_end = true;

	/* setup a timer to keep track of the frame DMA completion */
	init_timer(&camif->idle_timer);
	camif->idle_timer.function = &camif_do_idle;
	camif->idle_timer.expires = jiffies + HZ;
	camif->idle_timer.data = (unsigned long)camif;

	switch (state) {
	case CAMIF_BRINGUP:

		/* reset global status flags */
		camif->sw_workaround_applied = false;
		camif->hw_workaround_applied = false;
		camif->frame_cnt = -1;

		/*
		 * program default values for the CTRL register using the info
		 * passed from platform data.
		 * Note: These will be changed as per the sensor resolution
		 * settings, once SET_FMT is called from user-space
		 */
		camif_prog_default_ctrl(camif);

		/*
		 * Keep all interrupts disabled for now and enable frame-end
		 * interrupts after QBUF call from user-space
		 */
		camif_configure_interrupts(camif, DISABLE_ALL);
		break;
	default:
		break;
	}

	/* change state to running */
	camif->is_running = true;
}

/* Stop CAMIF functionality and DMA transfers */
static void camif_stop_capture(struct camif *camif,
			enum camif_power_state state)
{
	/* stop all interrupts */
	camif_configure_interrupts(camif, DISABLE_ALL);

	switch (state) {
	case CAMIF_SHUTDOWN:
		/* clear CAMIF registers */
		camif_reset_regs(camif);

		/* stop DMA engine and release channel */
		if (camif->chan) {
			dmaengine_terminate_all(camif->chan);
			dma_release_channel(camif->chan);
		}
		break;
	default:
		break;
	}


	/* clear global flags */
	camif->first_frame_end = false;

	/* delete recovery timer */
	del_timer_sync(&camif->idle_timer);

	/*
	 * HW related changes are common to all shutdown/power-save
	 * states
	 */
	/* disable CAMIF clk */
	clk_disable_unprepare(camif->clk);

	/* turn off camif module */
	camif_module_enable(camif, false);

	/* change state to stopped */
	camif->is_running = false;
}

/*
 * Add error handling mechanism to avoid hanging of system in case
 * CAMIF stops a DMA transfer in-between.
 */
static void camif_do_idle(unsigned long arg)
{
	struct camif *camif = (struct camif *)arg;
	struct videobuf_buffer *vb;
	struct camif_buffer *buf;
	unsigned long flags;

	spin_lock_irqsave(&camif->lock, flags);

	/*
	 * if camif was stopped earlier due to a empty dma_queue
	 * we need to handle the same here.
	 */
	if (!camif->sw_workaround_applied) {
		/*
		 * change states to error state of all active buffers
		 * and delete them from queue
		 */
		camif->cur_frm = list_first_entry(&camif->dma_queue,
				struct camif_buffer, vb.queue);

		while (!list_empty(&camif->dma_queue)) {
			vb = &camif->cur_frm->vb;
			buf = container_of(vb, struct camif_buffer, vb);
			vb->state = VIDEOBUF_ERROR;
			do_gettimeofday(&vb->ts);

			/*
			 * Set the field_count field. Note that the soc_camera
			 * layer divides the field_count field by 2, so for
			 * progressive mode we need to mark completion of 2
			 * fields whereas for interlaced mode we need to mark
			 * completion of 1 field here.
			 */
			if (camif->fmt.fmt.pix.field == V4L2_FIELD_NONE)
				vb->field_count = camif->frame_cnt * 2;
			if (camif->fmt.fmt.pix.field == V4L2_FIELD_INTERLACED)
				vb->field_count = camif->frame_cnt;

			/* wake-up any process interested in using this buf */
			wake_up_interruptible(&vb->done);

			/* get the next frame from dma_queue */
			list_del_init(&vb->queue);
			camif->cur_frm = list_entry(camif->dma_queue.next,
					struct camif_buffer, vb.queue);
		}

		/* put camif in a HW recovery state */
		camif_set_hw_recovery_state(camif);
	}

	spin_unlock_irqrestore(&camif->lock, flags);
}

void camif_schedule_next_buffer(struct camif *camif, struct videobuf_buffer *vb,
				struct camif_buffer *buf)
{
	/* check if dma queue is empty */
	if (list_empty(&camif->dma_queue)) {
		/* put camif in a SW recovery state */
		camif_set_sw_recovery_state(camif);
		return;
	}

	/* get the next frame from dma_queue and mark it as ACTIVE */
	camif->cur_frm = list_entry(camif->dma_queue.next,
					struct camif_buffer, vb.queue);

	camif->frame_cnt++;
	camif->cur_frm->vb.state = VIDEOBUF_ACTIVE;
}

void camif_process_buffer_complete(struct camif *camif,
					struct videobuf_buffer *vb,
					struct camif_buffer *buf)
{
	/* mark timestamp for this buffer and mark it as DONE */
	list_del_init(&vb->queue);
	vb->state = VIDEOBUF_DONE;
	do_gettimeofday(&vb->ts);

	/*
	 * Set the field_count field. Note that the soc_camera layer
	 * divides the field_count field by 2, so for progressive
	 * mode we need to mark completion of 2 fields whereas for
	 * interlaced mode we need to mark completion of 1 field here.
	 */
	if (camif->fmt.fmt.pix.field == V4L2_FIELD_NONE)
		vb->field_count = camif->frame_cnt * 2;
	if (camif->fmt.fmt.pix.field == V4L2_FIELD_INTERLACED)
		vb->field_count = camif->frame_cnt;

	/* wake-up any process interested in using this buffer */
	wake_up_interruptible(&vb->done);

	/* schedule the next buffer present in the buffer queue */
	camif_schedule_next_buffer(camif, vb, buf);
}

/* DMA completion callback */
static void camif_rx_dma_complete(void *arg)
{
	struct camif *camif = (struct camif *)arg;
	struct camif_buffer *buf;
	struct videobuf_buffer *vb;
	unsigned long flags;

	spin_lock_irqsave(&camif->lock, flags);

	/* cur_frm should not be NULL in DMA handler */
	if (!camif->cur_frm) {
		spin_unlock_irqrestore(&camif->lock, flags);
		goto out;
	}

	/* mark current frame as done */
	vb = &camif->cur_frm->vb;
	buf = container_of(vb, struct camif_buffer, vb);
	WARN_ON(buf->inwork || list_empty(&vb->queue));
	camif_process_buffer_complete(camif, vb, buf);

	/* schedule timer again */
	mod_timer(&camif->idle_timer, jiffies + 3 * HZ);

	spin_unlock_irqrestore(&camif->lock, flags);

out:
	return;
}

static int camif_init_dma_channel(struct camif *camif, struct camif_buffer *buf)
{
	int direction = DMA_FROM_DEVICE;
	struct videobuf_dmabuf *dma = videobuf_to_dma(&buf->vb);

	buf->desc =
		camif->chan->device->device_prep_slave_sg(
				camif->chan,
				dma->sglist, dma->sglen, direction,
				DMA_PREP_INTERRUPT | DMA_CTRL_ACK, "sg");
	if (!buf->desc) {
		dev_err(camif->ici.v4l2_dev.dev,
				"%s: buf->desc is NULL\n", __func__);
		return -ENOMEM;
	}

	buf->desc->callback = camif_rx_dma_complete;
	buf->desc->callback_param = camif;

	return 0;
}

/*
 * line interrupt signifies that we have received a complete line in the
 * line buffer. Currently we don't perform a lot of tasks here.
 */
static irqreturn_t camif_line_int(int irq, void *dev_id)
{
	struct camif *camif = (struct camif *)dev_id;
	int status_reg;

	status_reg = readl(camif->base + CAMIF_STATUS);
	if (!status_reg)
		return IRQ_NONE;

	/* perform a dummy write to clear line-end interrupt */
	writel(IRQ_STATUS_LINE_END, camif->base + CAMIF_STATUS);

	return IRQ_HANDLED;
}

/*
 * Note: a single interrupt line is shared for indicating both the frame start
 * and the frame end. So, in the ISR we read the status register to determine
 * the exact source of interrupt.
 *
 * At the moment frame start interrupt is used merely to set a
 * corresponding flag and no major action is performed in case of frame
 * start interrupt. In case of frame end interrupt we trigger the
 * next dma transfer.
 */
static irqreturn_t camif_frame_start_end_int(int irq, void *dev_id)
{
	int status_reg;
	unsigned long flags;
	struct camif *camif = (struct camif *)dev_id;
	struct videobuf_buffer *vb;
	struct camif_buffer *buf;

	status_reg = readl(camif->base + CAMIF_STATUS);
	if (!status_reg)
		return IRQ_NONE;

	if (status_reg & IRQ_STATUS_FRAME_START)
		/* perform a dummy write to clear frame-start interrupt */
		writel(IRQ_STATUS_FRAME_START, camif->base + CAMIF_STATUS);

	if (status_reg & IRQ_STATUS_FRAME_END) {
		/* perform a dummy write to clear frame-end interrupt */
		writel(IRQ_STATUS_FRAME_END, camif->base + CAMIF_STATUS);

		spin_lock_irqsave(&camif->lock, flags);
		/* always ignore the 1st frame */
		if (camif->first_frame_end) {
			camif->first_frame_end = false;
			spin_unlock_irqrestore(&camif->lock, flags);
			goto out;
		}

		/* from 2nd frame onwards, enable DMA engine */
		camif->cur_frm =
			list_first_entry(&camif->dma_queue,
					struct camif_buffer, vb.queue);

		vb = &camif->cur_frm->vb;
		buf = container_of(vb, struct camif_buffer, vb);

		/* mark state of the current frame to active */
		camif->cur_frm->vb.state = VIDEOBUF_ACTIVE;

		/* increment total frame count */
		camif->frame_cnt++;

		/* enable DMA */
		camif_configure_interrupts(camif,
				ENABLE_EVEN_FIELD_DMA);
		camif_configure_interrupts(camif,
				DISABLE_FRAME_END_INT);

		/*
		 * start the timer which keeps watch on DMA completion
		 * of a buffer.
		 */
		mod_timer(&camif->idle_timer, jiffies + HZ);

		spin_unlock_irqrestore(&camif->lock, flags);
	}

out:
	return IRQ_HANDLED;
}

static void free_buffer(struct videobuf_queue *vq, struct camif_buffer *buf)
{
	struct soc_camera_device *icd = vq->priv_data;
	struct videobuf_dmabuf *dma = videobuf_to_dma(&buf->vb);
	struct videobuf_buffer *vb = &buf->vb;

	BUG_ON(in_interrupt());

	dev_dbg(icd->parent, "(vb=0x%p) 0x%08lx %d\n",
			&buf->vb, buf->vb.baddr, buf->vb.bsize);

	vb->state = VIDEOBUF_DONE;

	/*
	 * This waits until this buffer is out of danger, i.e., until it is no
	 * longer in STATE_QUEUED or STATE_ACTIVE
	 */
	videobuf_waiton(vq, &buf->vb, 0, 0);
	videobuf_dma_unmap(vq->dev, dma);
	videobuf_dma_free(dma);

	buf->vb.state = VIDEOBUF_NEEDS_INIT;
}

static int camif_videobuf_setup(struct videobuf_queue *vq,
		unsigned int *count, unsigned int *size)
{
	struct soc_camera_device *icd = vq->priv_data;
	int bytes_per_line = soc_mbus_bytes_per_line(icd->user_width,
						icd->current_fmt->host_fmt);

	if (bytes_per_line < 0)
		return bytes_per_line;

	*size = bytes_per_line * icd->user_height;

	/* check for buffer count and size sanity */
	if (*count > VIDEO_MAX_FRAME)
		*count = VIDEO_MAX_FRAME;

	if (*count < CAMIF_MIN_BUFFER_CNT)
		*count = CAMIF_MIN_BUFFER_CNT;

	if (*size * *count > MAX_VIDEO_MEM * 1024 * 1024)
		*count = (MAX_VIDEO_MEM * 1024 * 1024) / *size;

	dev_dbg(icd->parent, "count=%d, size=%d\n", *count, *size);

	return 0;
}

static int camif_videobuf_prepare(struct videobuf_queue *vq,
		struct videobuf_buffer *vb, enum v4l2_field field)
{
	int ret;
	struct soc_camera_device *icd = vq->priv_data;
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct camif *camif = ici->priv;
	struct device *dev = camif->ici.v4l2_dev.dev;
	struct camif_buffer *buf = container_of(vb, struct camif_buffer, vb);
	int bytes_per_line = soc_mbus_bytes_per_line(icd->user_width,
						icd->current_fmt->host_fmt);

	if (bytes_per_line < 0)
		return bytes_per_line;

	WARN_ON(!list_empty(&vb->queue));

	dev_dbg(dev, "%s (vb=0x%p) 0x%08lx %d\n", __func__,
		vb, vb->baddr, vb->bsize);

#ifdef DEBUG
	/* This can be useful if you want to see if we actually fill
	 * the buffer with something */
	memset((void *)vb->baddr, 0xaa, vb->bsize);
#endif

	BUG_ON(NULL == icd->current_fmt);

	/* I think, in buf_prepare you only have to protect global data,
	 * the actual buffer is yours */
	buf->inwork = 1;

	if ((buf->code != icd->current_fmt->code) ||
			(vb->width != icd->user_width) ||
			(vb->height != icd->user_height) ||
			(vb->field != field)) {
		buf->code = icd->current_fmt->code;
		vb->width = icd->user_width;
		vb->height = icd->user_height;
		vb->field = field;
		vb->state = VIDEOBUF_NEEDS_INIT;
	}

	vb->size = bytes_per_line * vb->height;
	if (0 != vb->baddr && vb->bsize < vb->size) {
		ret = -EINVAL;
		goto out;
	}

	if (vb->state == VIDEOBUF_NEEDS_INIT) {
		ret = videobuf_iolock(vq, vb, NULL);
		if (ret)
			goto fail;

		vb->state = VIDEOBUF_PREPARED;
	}
	buf->inwork = 0;

	return 0;
fail:
	free_buffer(vq, buf);
out:
	buf->inwork = 0;

	return ret;
}

/* Called under spinlock_irqsave(&camif->lock, ...) */
static void camif_videobuf_queue(struct videobuf_queue *vq,
		struct videobuf_buffer *vb)
{
	int ret;
	dma_cookie_t cookie;
	struct soc_camera_device *icd = vq->priv_data;
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct camif_buffer *buf = container_of(vb, struct camif_buffer, vb);
	struct camif *camif = ici->priv;

	dev_dbg(icd->parent, "%s (vb=0x%p) 0x%08lx %d\n",
		__func__, vb, vb->baddr, vb->bsize);

	/* add the buffer to the DMA queue */
	list_add_tail(&vb->queue, &camif->dma_queue);

	/* change state of the buffer */
	vb->state = VIDEOBUF_QUEUED;

	/*
	 * prepare DMA SG requests for the buffer just
	 * requested by user-land
	 */
	ret = camif_init_dma_channel(camif, buf);
	if (ret) {
		dev_err(icd->parent,
				"DMA initialization failed\n");
		return;
	}

	/* submit the DMA descriptor */
	cookie = buf->desc->tx_submit(buf->desc);
	ret = dma_submit_error(cookie);
	if (ret) {
		dev_err(icd->parent,
				"dma submit error %d\n", cookie);
		return;
	}

	/*
	 * if camif was stopped earlier due to a empty dma_queue
	 * we need to handle the same here.
	 */
	if (camif->hw_workaround_applied) {
		/* turn on CAMIF module and restore registers */
		camif_module_enable(camif, true);
		camif_restore_regs(camif);

		/* enable frame-end interrupt as we don't have a cur_frm */
		camif_configure_interrupts(camif, ENABLE_FRAME_END_INT);

		camif->hw_workaround_applied = false;
	} else if (camif->sw_workaround_applied) {
		/* restore saved CAMIF registers */
		camif_restore_regs(camif);

		/* enable frame-end interrupt as we don't have a cur_frm */
		camif_configure_interrupts(camif, ENABLE_FRAME_END_INT);

		camif->sw_workaround_applied = false;
	} else if (!camif->cur_frm)
		/* enable frame-end interrupt as we don't have a cur_frm */
		camif_configure_interrupts(camif, ENABLE_FRAME_END_INT);
}

/* Called under spinlock_irqsave(&camif->lock, ...) */
static void camif_videobuf_release(struct videobuf_queue *vq,
				 struct videobuf_buffer *vb)
{
	struct soc_camera_device *icd = vq->priv_data;
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct camif_buffer *buf = container_of(vb, struct camif_buffer, vb);
	struct camif *camif = ici->priv;
	unsigned long flags;

	spin_lock_irqsave(&camif->lock, flags);

	if (camif->cur_frm == buf) {
		/* stop DMA transfers */
		camif_configure_interrupts(camif, DISABLE_EVEN_FIELD_DMA);
		if (camif->chan)
			dmaengine_terminate_all(camif->chan);

		camif->cur_frm = NULL;
	}

	if ((vb->state == VIDEOBUF_ACTIVE || vb->state == VIDEOBUF_QUEUED) &&
			!list_empty(&vb->queue)) {
		vb->state = VIDEOBUF_ERROR;
		list_del_init(&vb->queue);
	}

	spin_unlock_irqrestore(&camif->lock, flags);

	free_buffer(vq, buf);
}

static struct videobuf_queue_ops camif_videobuf_ops = {
	.buf_setup = camif_videobuf_setup,
	.buf_prepare = camif_videobuf_prepare,
	.buf_queue = camif_videobuf_queue,
	.buf_release = camif_videobuf_release,
};

/* SOC Camera host operations */

/*
 * The following two functions absolutely depend on the fact, that
 * there can be only one camera on CAMIF camera sensor interface
 * Called with .video_lock held.
 */
static int camif_add_device(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct camif *camif = ici->priv;

	/* camif can manage a single camera at one time */
	if (camif->icd)
		return -EBUSY;

	camif->icd = icd;

	/* initialize dma_queue queue again */
	INIT_LIST_HEAD(&camif->dma_queue);

	camif_start_capture(camif, CAMIF_BRINGUP);

	dev_info(icd->parent, "SPEAr Camera driver attached to camera %d\n",
		 icd->devnum);

	return 0;
}

/* Called with .video_lock held */
static void camif_remove_device(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct camif *camif = ici->priv;

	BUG_ON(icd != camif->icd);

	/* put CAMIF in SHUTDOWN state */
	camif_stop_capture(camif, CAMIF_SHUTDOWN);

	dev_info(icd->parent,
		"SPEAr Camera driver detached from camera %d\n", icd->devnum);

	camif->icd = NULL;
}

static int camif_test_param(struct camif *camif,
			       unsigned char buswidth, unsigned long *flags)
{
	/*
	 * If requested data width is not supported by the platform,
	 * exit with error
	 */
	if (!((1 << (buswidth - 1)) & SOCAM_DATAWIDTH_8))
		return -EINVAL;

	*flags = CAM_IF_BUS_FLAGS;

	return 0;
}

static int camif_try_bus_param(struct soc_camera_device *icd,
					unsigned char buswidth)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct camif *camif = ici->priv;
	struct v4l2_mbus_config cfg = {.type = V4L2_MBUS_PARALLEL,};
	unsigned long bus_flags, common_flags;
	int ret = camif_test_param(camif, buswidth, &bus_flags);

	if (ret < 0)
		return ret;

	ret = v4l2_subdev_call(sd, video, g_mbus_config, &cfg);
	if (!ret) {
		common_flags = soc_mbus_config_compatible(&cfg,
							  bus_flags);
		if (!common_flags) {
			dev_warn(icd->parent,
				 "Flags incompatible: camera 0x%x, host 0x%lx\n",
				 cfg.flags, bus_flags);
			return -EINVAL;
		}
	} else if (ret == -ENOIOCTLCMD) {
		ret = 0;
	}

	return ret;
}

static int camif_get_formats(struct soc_camera_device *icd, unsigned int idx,
				struct soc_camera_format_xlate *xlate)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct device *dev = icd->parent;
	int formats = 0, ret;
	enum v4l2_mbus_pixelcode code;
	const struct soc_mbus_pixelfmt *fmt;

	ret = v4l2_subdev_call(sd, video, enum_mbus_fmt, idx, &code);
	if (ret < 0)
		/* No more formats */
		return 0;

	fmt = soc_mbus_get_fmtdesc(code);
	if (!fmt) {
		dev_err(dev, "Invalid format code #%u: %d\n", idx, code);
		return 0;
	}

	/* This also checks support for the requested bits-per-sample */
	ret = camif_try_bus_param(icd, fmt->bits_per_sample);
	if (ret < 0)
		return 0;

	switch (code) {
	case V4L2_MBUS_FMT_RGB24_2X8_LE:
	case V4L2_MBUS_FMT_BGR24_2X8_LE:
		formats++;
		if (xlate) {
			xlate->host_fmt	= &camif_formats[code];
			xlate->code = code;
			xlate++;
			dev_dbg(dev, "providing format %s using code %d\n",
					camif_formats[code].name, code);
		}
	default:
		if (xlate)
			dev_dbg(dev, "providing format %s"
					"in pass-through mode\n", fmt->name);
	}

	/* Generic pass-through */
	formats++;
	if (xlate) {
		xlate->host_fmt	= fmt;
		xlate->code = code;
		xlate++;
	}

	return formats;
}

static void camif_put_formats(struct soc_camera_device *icd)
{
	kfree(icd->host_priv);
	icd->host_priv = NULL;
}

/* Alter DMA settings on the run as per the new format selected */
static int camif_change_dma_settings(struct camif *camif, u32 bytesperline)
{
	struct dma_slave_config conf;
	int dma_burst_size = 0, camif_burst_size = 0;
	int dma_tx_width = DMA_SLAVE_BUSWIDTH_8_BYTES;
	int camif_tx_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	int ret = 0;

	/*
	 * Note: There are three corner cases we need to take care of here
	 * to ensure that DMA works for all possible resolutions
	 * selected by user-space application:
	 * 1. camif DMA_BURST_SIZE (programmed in CAM_IF_CTRL reg
	 *    cannot be greater then the length of a line.
	 * 2. For all resolutions:
	 *    (bytes_per_line * 8) % (CAMIF.TR_WIDTH * CAMIF.BURST_SIZE)
	 *    should be 0. If not, change the DMA settings to the next
	 *    best possible ones. This is required because of the issue
	 *    listed in (3) below.
	 * 3. CAMIF cannot support SINGLE DMA requests with the V4L2
	 *    framework as the.
	 */
	for (camif_burst_size = CAMIF_BURST_SIZE_256;
			camif_burst_size >= CAMIF_BURST_SIZE_4;
			camif_burst_size = camif_burst_size / 2) {
		if (bytesperline %
				(camif_tx_width * camif_burst_size) == 0) {
			if (camif_burst_size > (bytesperline * 8))
				continue;

			goto prog_dma;
		}
	}

	if (camif_burst_size < CAMIF_BURST_SIZE_4) {
		/*
		 * CAMIF will stall while performing a SREQ, so
		 * we need to warn the user and return an error here
		 */
		dev_err(camif->icd->parent,
				"CAMIF cannot support SINGLE DMA requests "
				"program a different resolution\n");

		ret = -EINVAL;
		goto out;
	}

prog_dma:
	/* program the new config settings */

	/*
	 * Note that while camif supports a 32-bit DMA
	 * slave bus width the underlying DMA
	 * controller can support more,
	 * so we need to handle the same as well
	 */
	if (dma_tx_width > DMA_SLAVE_BUSWIDTH_4_BYTES)
		dma_burst_size = camif_burst_size / 2;
	else
		dma_burst_size = camif_burst_size;

	conf.src_maxburst = conf.dst_maxburst = dma_burst_size;
	conf.src_addr_width = dma_tx_width;

	conf.src_addr = camif->phys_base_addr + CAMIF_MEM_BUFFER;
	conf.direction = DMA_FROM_DEVICE;
	conf.device_fc = false;

	dmaengine_slave_config(camif->chan, &conf);

	/* Re-program the camif side DMA burst settings */
	camif_prog_dma_burst(camif, camif_burst_size);

out:
	return ret;
}

static int camif_crop(struct soc_camera_device *icd, struct v4l2_crop *crop)
{
	struct v4l2_rect *rect = &crop->c;
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct camif *camif = ici->priv;
	int ret = 0;

	/* check for image xtreme boundary */
	if (rect->width > CAMIF_MAX_WIDTH)
		rect->width = CAMIF_MAX_WIDTH;

	if (rect->height > CAMIF_MAX_HEIGHT)
		rect->height = CAMIF_MAX_HEIGHT;

	/*
	 * Use CAMIF cropping to crop to the new window:
	 * program the crop start and stop registers to ensure correct cropping
	 * coordinates
	 */
	writel(CROP_H_MASK(rect->left), camif->base + CAMIF_CSTARTP);
	writel(CROP_V_MASK(rect->top), camif->base + CAMIF_CSTARTP);

	writel(CROP_H_MASK(rect->left + rect->width),
			camif->base + CAMIF_CSTOPP);
	writel(CROP_V_MASK(rect->top + rect->height),
			camif->base + CAMIF_CSTOPP);

	/* enable cropping */
	writel(readl(camif->base + CAMIF_CTRL) | CTRL_CROP_SELECT,
			camif->base + CAMIF_CTRL);

	/* save cropped dimensions for later use */
	camif->fmt.fmt.pix.width = icd->user_width = rect->width;
	camif->fmt.fmt.pix.height = icd->user_height = rect->height;
	camif->fmt.fmt.pix.bytesperline =
		soc_mbus_bytes_per_line(icd->user_width,
						icd->current_fmt->host_fmt);
	camif->fmt.fmt.pix.sizeimage =
		camif->fmt.fmt.pix.bytesperline *
		camif->fmt.fmt.pix.height;

	camif->crop = crop->c;

	/*
	 * To ensure that DMA works for all possible resolutions
	 * selected by user-space application the DNA controller and
	 * camif DMA interface must be re-programmed with new correct
	 * values.
	 */
	if (camif->chan)
		ret = camif_change_dma_settings(camif,
				camif->fmt.fmt.pix.bytesperline);

	return ret;
}

/*
 * CAMIF supports cropping, but we first should check if the underlying
 * sub-dev supports cropcrop. If it does we should try to get the
 * cropcap window of the sub-dev. Otherwise, we need to provide
 * CAMIF's own cropcap window to the user-space.
 */
static int camif_cropcap(struct soc_camera_device *icd,
				struct v4l2_cropcap *crop)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct device *dev = icd->parent;
	int ret;

	/* try to check if the sub-device supports cropcap */
	ret = v4l2_subdev_call(sd, video, cropcap, crop);
	if (ret == -ENOIOCTLCMD) {
		/*
		 * Sensor does not support cropcrop, so return CAMIF's
		 * present cropcap window
		 */
		memset(crop, 0, sizeof(struct v4l2_cropcap));
		crop->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		crop->bounds.width = crop->defrect.width = CAMIF_MAX_WIDTH;
		crop->bounds.height = crop->defrect.height = CAMIF_MAX_HEIGHT;
		crop->pixelaspect.numerator = 1;
		crop->pixelaspect.denominator = 1;
	} else if (ret < 0 && (ret != -ENOIOCTLCMD)) {
		dev_warn(dev, "subdev failed in cropcrop\n");
		goto out;
	}

	return 0;

out:
	return ret;
}

/*
 * CAMIF supports cropping, but we first should check if the underlying
 * sub-dev supports g_crop. If it does we should try to get the
 * cropping window of the sub-dev. Otherwise, we need to provide
 * CAMIF's own cropping window to the user-space.
 */
static int camif_get_crop(struct soc_camera_device *icd, struct v4l2_crop *crop)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct device *dev = icd->parent;
	struct camif *camif = ici->priv;
	int ret;

	/* try to get the cropping window of the sub-device */
	ret = v4l2_subdev_call(sd, video, g_crop, crop);
	if (ret == -ENOIOCTLCMD) {
		/*
		 * Sensor does not support g_crop, so return CAMIF's
		 * present cropping window
		 */
		crop->c = camif->crop;
	} else if (ret < 0 && (ret != -ENOIOCTLCMD)) {
		dev_warn(dev, "subdev failed in g_crop\n");
		goto out;
	}

	return 0;

out:
	return ret;
}

/*
 * CAMIF can perform cropping, but we don't want to waste bandwidth
 * and kill the framerate by always requesting the maximum image
 * from the sub-device (client/sensor). So, first we request the exact
 * user rectangle from the sensor. The sensor will try to preserve its
 * output frame as far as possible, but it could have changed, so we
 * retreive it again. In case the sub-device does not support s_crop
 * or cannot produce an cropping rectangle with desired accuracy
 * we invoke the crop functionality of CAMIF, to ensure that we have
 * a final rectangle which is as close as possible to what has been
 * requested by the user.
 */
static int camif_set_crop(struct soc_camera_device *icd, struct v4l2_crop *crop)
{
	struct v4l2_rect *rect = &crop->c;
	struct device *dev = icd->parent;
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct v4l2_mbus_framefmt mf;
	int ret;

	/* check if the sub-device supports cropping */
	ret = v4l2_subdev_call(sd, video, s_crop, crop);
	if (ret == -ENOIOCTLCMD) {
		/* sensor does not support cropping but camif does */
		ret = camif_crop(icd, crop);
		if (ret < 0)
			goto out;
	} else if (ret < 0 && (ret != -ENOIOCTLCMD)) {
		dev_warn(dev, "subdev failed to crop to %ux%u@%u:%u\n",
			 rect->width, rect->height, rect->left, rect->top);
		goto out;
	} else {
		/* retrieve camera output window */
		ret = v4l2_subdev_call(sd, video, g_mbus_fmt, &mf);
		if (ret < 0) {
			dev_warn(dev, "failed to get current subdev format\n");
			goto out;
		}

		/*
		 * It may be that the camera cropping produced a frame beyond
		 * our capabilities (i.e. > CAMIF_MAX_WIDTH * CAMIF_MAX_HEIGHT).
		 * Simply extract a subframe, that we can process.
		 */
		if (mf.width > CAMIF_MAX_WIDTH ||
				mf.height > CAMIF_MAX_HEIGHT) {
			ret = camif_crop(icd, crop);
			if (ret < 0)
				goto out;
			/*
			 * Try to align the underlying camera to our cropped
			 * subframe
			 */
			mf.width = rect->width;
			mf.height = rect->height;

			ret = v4l2_subdev_call(sd, video, s_mbus_fmt, &mf);
			if (ret < 0)
				goto out;

			if (mf.width > CAMIF_MAX_WIDTH ||
				mf.height > CAMIF_MAX_HEIGHT) {
				dev_warn(dev, "Inconsistent state after s_crop."
					" Use S_FMT to repair\n");
				ret = -EINVAL;
				goto out;
			}
		}

		icd->user_width = mf.width;
		icd->user_height = mf.height;
	}

	return 0;

out:
	return ret;
}

static int camif_set_fmt(struct soc_camera_device *icd, struct v4l2_format *f)
{
	struct device *dev = icd->parent;
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct camif *camif = ici->priv;
	const struct soc_camera_format_xlate *xlate = NULL;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct v4l2_mbus_framefmt mf;
	int ret;
	u32 ctrl;

	xlate = soc_camera_xlate_by_fourcc(icd, pix->pixelformat);
	if (!xlate) {
		dev_warn(dev, "format %x not found\n", pix->pixelformat);
		return -EINVAL;
	}

	v4l2_fill_mbus_format(&mf, pix, xlate->code);

	ctrl = readl(camif->base + CAMIF_CTRL);
	switch (xlate->code) {
	case V4L2_MBUS_FMT_RGB24_2X8_LE:
		ctrl |= CTRL_IF_TRANS(RGB888);
		break;
	case V4L2_MBUS_FMT_RGB444_2X8_PADHI_BE:
	case V4L2_MBUS_FMT_RGB565_2X8_BE:
		ctrl |= CTRL_IF_TRANS(RGB565);
		break;
	case V4L2_MBUS_FMT_BGR565_2X8_BE:
		ctrl |= CTRL_IF_TRANS(BGR565);
		break;
	case V4L2_MBUS_FMT_YUYV8_2X8:
	case V4L2_MBUS_FMT_YVYU8_2X8:
	case V4L2_MBUS_FMT_UYVY8_2X8:
	case V4L2_MBUS_FMT_VYUY8_2X8:
	default:
		ctrl |= CTRL_IF_TRANS(YUVCbYCrY);
		break;
	}
	writel(ctrl, camif->base + CAMIF_CTRL);

	/* limit to sensor capabilities */
	ret = v4l2_subdev_call(sd, video, s_mbus_fmt, &mf);
	if (ret < 0)
		goto out;

	if (mf.code != xlate->code) {
		ret = -EINVAL;
		goto out;
	}

	v4l2_fill_pix_format(pix, &mf);

	icd->current_fmt = xlate;

	/*
	 * Underlying subdev may support all fields types but we
	 * need to set the field type we support.
	 */
	switch (mf.field) {
	case V4L2_FIELD_ANY:
	case V4L2_FIELD_NONE:
		pix->field = V4L2_FIELD_NONE;
		break;
	default:
		/* TODO: support interlaced at least in pass-through mode */
		dev_warn(icd->parent, "field type %d unsupported, "
				"resorting to the default PROGRESSIVE mode\n",
				mf.field);
		pix->field = V4L2_FIELD_NONE;
	}

	/* image boundary checks for CAMIF */
	if (pix->width > CAMIF_MAX_WIDTH)
		pix->width = CAMIF_MAX_WIDTH;

	if (pix->height > CAMIF_MAX_HEIGHT)
		pix->height = CAMIF_MAX_HEIGHT;

	/* update the values of sizeimage and bytesperline */
	pix->bytesperline = soc_mbus_bytes_per_line(pix->width,
						xlate->host_fmt);
	pix->sizeimage = pix->bytesperline * pix->height;

	dev_info(dev, "adjusted:: width=%d, height=%d, "
			"bytesperline=%d, sizeimage=%d\n",
			pix->width, pix->height,
			pix->bytesperline, pix->sizeimage);

	/*
	 * To ensure that DMA works for all possible resolutions
	 * selected by user-space application the DNA controller and
	 * camif DMA interface must be re-programmed with new correct
	 * values.
	 */
	if (camif->chan) {
		ret = camif_change_dma_settings(camif, pix->bytesperline);
		if (ret < 0)
			goto out;
	}

	/* keep a local copy of the format for later usage */
	camif->fmt = *f;

	return 0;

out:
	return ret;
}

static int camif_try_fmt(struct soc_camera_device *icd, struct v4l2_format *f)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	const struct soc_camera_format_xlate *xlate;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	__u32 pixfmt = pix->pixelformat;
	struct v4l2_mbus_framefmt mf;
	int ret;

	xlate = soc_camera_xlate_by_fourcc(icd, pixfmt);
	if (!xlate) {
		dev_warn(icd->parent, "format %x not found\n", pixfmt);
		return -EINVAL;
	}

	pix->bytesperline = soc_mbus_bytes_per_line(pix->width,
						xlate->host_fmt);
	if (pix->bytesperline < 0)
		return pix->bytesperline;

	pix->sizeimage = pix->height * pix->bytesperline;

	v4l2_fill_mbus_format(&mf, pix, xlate->code);

	/* limit to sensor capabilities */
	ret = v4l2_subdev_call(sd, video, try_mbus_fmt, &mf);
	if (ret < 0)
		goto out;

	v4l2_fill_pix_format(pix, &mf);

	pix->pixelformat = pixfmt;

	/*
	 * Underlying subdev may support all fields types but we
	 * need to report the field type we support.
	 */
	switch (mf.field) {
	case V4L2_FIELD_ANY:
	case V4L2_FIELD_NONE:
		pix->field = V4L2_FIELD_NONE;
		break;
	default:
		/* TODO: support interlaced at least in pass-through mode */
		dev_warn(icd->parent, "field type %d unsupported, "
				"resorting to the default PROGRESSIVE mode\n",
				mf.field);
		pix->field = V4L2_FIELD_NONE;
	}

	/* image boundary checks for CAMIF */
	if (pix->width > CAMIF_MAX_WIDTH)
		pix->width = CAMIF_MAX_WIDTH;

	if (pix->height > CAMIF_MAX_HEIGHT)
		pix->height = CAMIF_MAX_HEIGHT;

	/* update the values of sizeimage and bytesperline */
	pix->bytesperline = soc_mbus_bytes_per_line(pix->width,
						xlate->host_fmt);
	pix->sizeimage = pix->bytesperline * pix->height;

	return 0;

out:
	return ret;
}

static void camif_init_videobuf(struct videobuf_queue *q,
		struct soc_camera_device *icd)
{
	dma_cap_mask_t mask;
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct camif *camif = ici->priv;

	/* dma related settings */
	struct dma_slave_config dma_conf = {
		.src_addr = camif->phys_base_addr,
		.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES,
		.src_maxburst = 64,
		.direction = DMA_FROM_DEVICE,
		.device_fc = false,
	};

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);

	switch (camif->pdata->config->channel) {
	case EVEN_CHANNEL:
		camif->chan = dma_request_channel(mask,
			camif->pdata->dma_filter, camif->pdata->dma_even_param);
		if (!camif->chan) {
			dev_err(icd->parent,
				"unable to get DMA channel for even lines\n");
			return;
		}
		break;

	case BOTH_ODD_EVEN_CHANNELS:
		/*
		 * not supported as of now, warn and return to default
		 * 'even line' _only_ settings
		 */
		dev_warn(icd->parent, "Interlaced fields are not supported"
				"defaulting to even line settings\n");
		camif->chan = dma_request_channel(mask,
			camif->pdata->dma_filter, camif->pdata->dma_even_param);
		if (!camif->chan) {
			dev_err(icd->parent,
				"unable to get DMA channel for even lines\n");
			return;
		}
		break;
	}

	dmaengine_slave_config(camif->chan, (void *) &dma_conf);

	videobuf_queue_sg_init(q, &camif_videobuf_ops, icd->parent,
			&camif->lock, V4L2_BUF_TYPE_VIDEO_CAPTURE,
			camif->fmt.fmt.pix.field, sizeof(struct camif_buffer),
			icd, NULL);
}

static int camif_reqbufs(struct soc_camera_device *icd,
			struct v4l2_requestbuffers *p)
{
	int i;

	/*
	 * This is for locking debugging only. I removed spinlocks and now I
	 * check whether .prepare is ever called on a linked buffer, or whether
	 * a dma IRQ can occur for an in-work or unlinked buffer. Until now
	 * it hadn't triggered
	 */
	for (i = 0; i < p->count; i++) {
		struct camif_buffer *buf = container_of(icd->vb_vidq.bufs[i],
				struct camif_buffer, vb);
		buf->inwork = 0;
		INIT_LIST_HEAD(&buf->vb.queue);
	}

	return 0;
}

static int camif_querycap(struct soc_camera_host *ici,
				struct v4l2_capability *cap)
{
	/* cap->name is set by the friendly caller:-> */
	strlcpy(cap->card, "SPEAr Camera Interface", sizeof(cap->card));
	cap->version = CAM_IF_VERSION_CODE;
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;

	return 0;
}

static void camif_setup_ctrl(struct soc_camera_device *icd, unsigned long flags)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct camif *camif = ici->priv;
	u32 ctrl;

	ctrl = readl(camif->base + CAMIF_CTRL);
	ctrl |= CTRL_VS_POL_HI | CTRL_HS_POL_HI;

	if (flags & V4L2_MBUS_PCLK_SAMPLE_FALLING)
		ctrl &= ~CTRL_PCK_POL_HI;
	if (flags & V4L2_MBUS_HSYNC_ACTIVE_LOW)
		ctrl &= ~CTRL_HS_POL_HI;
	if (flags & V4L2_MBUS_VSYNC_ACTIVE_LOW)
		ctrl &= ~CTRL_VS_POL_HI;

	writel(ctrl, camif->base + CAMIF_CTRL);
}

static int camif_set_bus_param(struct soc_camera_device *icd)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct camif *camif = ici->priv;
	struct v4l2_mbus_config cfg = {.type = V4L2_MBUS_PARALLEL,};
	unsigned long bus_flags, common_flags;
	int ret = camif_test_param(camif, icd->current_fmt->host_fmt->bits_per_sample,
				&bus_flags);
	if (ret < 0)
		return ret;

	/* get bus configuration supported by underlying sensor */
	ret = v4l2_subdev_call(sd, video, g_mbus_config, &cfg);
	if (!ret) {
		common_flags = soc_mbus_config_compatible(&cfg,
							  bus_flags);
		if (!common_flags) {
			dev_warn(icd->parent,
				 "Flags incompatible: camera 0x%x, host 0x%lx\n",
				 cfg.flags, bus_flags);
			return -EINVAL;
		}
	} else if (ret != -ENOIOCTLCMD) {
		return ret;
	} else {
		common_flags = bus_flags;
	}

	/* Make choises, based on platform preferences */
	if ((common_flags & V4L2_MBUS_HSYNC_ACTIVE_HIGH) &&
	    (common_flags & V4L2_MBUS_HSYNC_ACTIVE_LOW)) {
		if (camif->pdata->config->hsync_polarity == ACTIVE_LOW)
			common_flags &= ~V4L2_MBUS_HSYNC_ACTIVE_HIGH;
		else
			common_flags &= ~V4L2_MBUS_HSYNC_ACTIVE_LOW;
	}

	if ((common_flags & V4L2_MBUS_VSYNC_ACTIVE_HIGH) &&
	    (common_flags & V4L2_MBUS_VSYNC_ACTIVE_LOW)) {
		if (camif->pdata->config->vsync_polarity == ACTIVE_LOW)
			common_flags &= ~V4L2_MBUS_VSYNC_ACTIVE_HIGH;
		else
			common_flags &= ~V4L2_MBUS_VSYNC_ACTIVE_LOW;
	}

	if ((common_flags & V4L2_MBUS_PCLK_SAMPLE_RISING) &&
	    (common_flags & V4L2_MBUS_PCLK_SAMPLE_FALLING)) {
		if (camif->pdata->config->pclk_polarity == ACTIVE_LOW)
			common_flags &= ~V4L2_MBUS_PCLK_SAMPLE_RISING;
		else
			common_flags &= ~V4L2_MBUS_PCLK_SAMPLE_FALLING;
	}

	cfg.flags = common_flags;
	ret = v4l2_subdev_call(sd, video, s_mbus_config, &cfg);
	if (ret < 0 && ret != -ENOIOCTLCMD) {
		dev_dbg(icd->parent, "camera s_mbus_config(0x%lx) returned %d\n",
			common_flags, ret);
		return ret;
	}

	camif_setup_ctrl(icd, common_flags);

	return 0;
}

static unsigned int camif_camera_poll(struct file *file, poll_table *pt)
{
	struct soc_camera_device *icd = file->private_data;
	struct camif_buffer *buf;

	buf = list_entry(icd->vb_vidq.stream.next, struct camif_buffer,
			 vb.stream);

	poll_wait(file, &buf->vb.done, pt);

	if (buf->vb.state == VIDEOBUF_DONE || buf->vb.state == VIDEOBUF_ERROR)
		return POLLIN | POLLRDNORM;

	return 0;
}

static struct soc_camera_host_ops camif_soc_camera_host_ops = {
	.owner = THIS_MODULE,
	.add = camif_add_device,
	.remove	= camif_remove_device,
	.get_formats = camif_get_formats,
	.put_formats = camif_put_formats,
	.cropcap = camif_cropcap,
	.get_crop = camif_get_crop,
	.set_crop = camif_set_crop,
	.set_fmt = camif_set_fmt,
	.try_fmt = camif_try_fmt,
	.init_videobuf = camif_init_videobuf,
	.reqbufs = camif_reqbufs,
	.querycap = camif_querycap,
	.set_bus_param = camif_set_bus_param,
	.poll = camif_camera_poll,
};

static int __devinit camif_probe(struct platform_device *pdev)
{
	int ret;
	struct camif *camif;
	void __iomem *addr;
	struct clk *clk;
	struct resource *mem;
	int line_irq, frm_start_end_irq;

	/* must have platform data */
	if (!pdev) {
		dev_err(&pdev->dev, "missing platform data\n");
		ret = -EINVAL;
		goto exit;
	}
	/* get the platform data */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	line_irq = platform_get_irq_byname(pdev, "line_end_irq");
	frm_start_end_irq = platform_get_irq_byname(pdev,
				"frame_start_frame_end_irq");
	if (!mem || (line_irq <= 0) || (frm_start_end_irq <= 0)) {
		ret = -ENODEV;
		goto exit;
	}

	/* allocate camif resource */
	camif = kzalloc(sizeof(struct camif), GFP_KERNEL);
	if (!camif) {
		dev_err(&pdev->dev, "not enough memory for camif device\n");
		ret = -ENOMEM;
		goto exit;
	}

	camif->phys_base_addr = mem->start;

	if (!request_mem_region(mem->start, resource_size(mem),
				KBUILD_MODNAME)) {
		dev_err(&pdev->dev, "resource unavailable\n");
		ret = -ENODEV;
		goto exit_free_camif;
	}

	addr = ioremap(mem->start, resource_size(mem));
	if (!addr) {
		dev_err(&pdev->dev, "failed to map camif port\n");
		ret = -ENOMEM;
		goto exit_release_mem;
	}

	camif->base = addr;

	clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "no clock defined\n");
		ret = PTR_ERR(clk);
		goto exit_iounmap;
	}

	camif->clk = clk;
	camif->frm_start_end_irq = frm_start_end_irq;
	camif->line_irq = line_irq;
	camif->pdata = pdev->dev.platform_data;
	camif->id = camif->pdata->id;

	ret = request_irq(camif->line_irq, camif_line_int, 0,
				"camif_line", camif);
	if (ret) {
		dev_err(&pdev->dev,
				"failed to allocate IRQ %d\n", camif->line_irq);
		goto exit_clk_put;
	}

	ret = request_irq(camif->frm_start_end_irq, camif_frame_start_end_int,
				0, "camif_frame_start_end", camif);
	if (ret) {
		dev_err(&pdev->dev, "failed to allocate IRQ %d\n",
				camif->frm_start_end_irq);
		goto exit_free_line_irq;
	}

	/*
	 * There is always a async relationship between when the
	 * application will request some data and when the sensor will
	 * start sending the captured data.
	 *
	 * To prevent flooding of CPU with interrupts when application
	 * has not yet requested data and the sensor has been
	 * initialized, disable interrupts initially.
	 */
	camif_configure_interrupts(camif, DISABLE_ALL);

	camif->ici.v4l2_dev.dev = &pdev->dev;
	camif->ici.nr = camif->pdata->id;
	camif->ici.priv = camif;
	camif->ici.drv_name = "spear_camif";
	camif->ici.ops = &camif_soc_camera_host_ops;

	ret = soc_camera_host_register(&camif->ici);
	if (ret) {
		dev_err(&pdev->dev,
			"failed to register camif as SoC camera host\n");
		goto exit_free_frm_start_end_irq;
	}

	INIT_LIST_HEAD(&camif->dma_queue);

	/* init locks */
	spin_lock_init(&camif->lock);

	dev_info(&pdev->dev, "%s registered, (base-addr=%p, "
			"lineIRQ=%d, vsyncIRQ=%d)\n",
			KBUILD_MODNAME, camif->base, camif->line_irq,
			camif->frm_start_end_irq);

	return 0;

exit_free_frm_start_end_irq:
	free_irq(camif->frm_start_end_irq, camif);
exit_free_line_irq:
	free_irq(camif->line_irq, camif);
exit_clk_put:
	clk_put(clk);
exit_iounmap:
	iounmap(addr);
exit_release_mem:
	release_mem_region(mem->start, resource_size(mem));
exit_free_camif:
	kfree(camif);
exit:
	dev_err(&pdev->dev, "probe failed\n");

	return ret;
}

static int __devexit camif_remove(struct platform_device *pdev)
{
	struct soc_camera_host *ici = to_soc_camera_host(&pdev->dev);
	struct camif *camif = container_of(ici, struct camif, ici);
	struct resource *mem;

	soc_camera_host_unregister(&camif->ici);

	dma_release_channel(camif->chan);

	free_irq(camif->frm_start_end_irq, camif);
	free_irq(camif->line_irq, camif);

	clk_put(camif->clk);

	iounmap(camif->base);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(mem->start, resource_size(mem));

	kfree(camif);

	return 0;
}

#ifdef CONFIG_OF
static struct of_device_id camif_id_match[] = {
	{ .compatible = "st,camif", },
	{}
};
MODULE_DEVICE_TABLE(of, camif_id_match);
#endif

static struct platform_driver camif_driver = {
	.probe = camif_probe,
	.remove = __devexit_p(camif_remove),
	.driver = {
		.name = "spear_camif",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = camif_id_match,
#endif
	},
};

static int __init camif_init(void)
{
	return platform_driver_register(&camif_driver);
}
module_init(camif_init);

static void __exit camif_exit(void)
{
	platform_driver_unregister(&camif_driver);
}
module_exit(camif_exit);

MODULE_AUTHOR("Bhupesh Sharma <bhupesh.sharma@st.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:spear_camif");
MODULE_DESCRIPTION("SoC Camera Driver for SPEAr CAMIF");
