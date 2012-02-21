/*
 * V4L2 driver for SPEAr Video Input Parallel Port (VIP)
 *
 * Copyright (C) 2011 ST Microelectronics
 * Bhupesh Sharma <bhupesh.sharma@st.com>
 *
 * Based on DaVinci VPIF capture driver
 * Copyright (C) 2009 Texas Instruments Inc
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 *
 *
 * Notes about VIP:
 * ----------------
 * VIP supports a 48-bit RGB interface and is more suited to Dual-TMDS
 * link DVI architecture. One 24-bit interface is dedicated to ODD
 * signals and the other 24-bit interface is dedicated for EVEN signals.
 *
 * However, the VIP documentation suggests that it can support CVBS,
 * 24-bit single-link DVI, 48-bit dual-link DVI and Display Port
 * interfaces. One interesting point to note here is that HDMI is
 * backward compliant to 24-bit DVI and hence we can also support HDMI
 * receiver chips with this interface.
 *
 * VIP provides a VDO register where the driver can program the base
 * address of the frame to be stored in system RAM. But, to enhance
 * rendering performance, we implement a classic ping-pong architecture
 * by allocating 2 HD buffers in system DDR and while one is being
 * WRITTEN to by VIP (producer), the other can be READ by some consumer
 * application like frame-buffer driver displaying the data on a LCD.
 *
 * TODO:
 * ----
 *  - Implement and check support for CVBS and DP.
 *  - HD frames should be allocated at boot-time to ensure they are
 *    contiguous and pointers to these addresses should be passed to
 *    driver somehow (CMA?)
 */

#include <linux/clk.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>

#include <media/v4l2-chip-ident.h>
#include <media/v4l2-common.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf-dma-contig.h>
#include <media/vip.h>

/* VIP register map */
#define VIP_CONTROL		0x00	/* control register */
#define VIP_INT_STATUS		0x04	/* interrupt status register */
#define VIP_INT_MASK		0x08	/* interrupt mask register */
#define VIP_VDO_BASE		0x0C	/* video base address register */

/* control register bit offsets */
#define CTRL_VSYNC_POL_SHIFT	5
#define CTRL_HSYNC_POL_SHIFT	4
#define CTRL_RGB_WIDTH_SHIFT	2
#define CTRL_VDO_MODE_SHIFT	1
#define CTRL_PIXCLK_POL_SHIFT	0

/* interrupt status register bits */
#define IRQ_STATUS_FIFO_OVF	BIT(0)
#define CLEAR_FIFO_OVF_IRQ	IRQ_STATUS_FIFO_OVF

/* interrupt mask register bits */
#define IR_MASK_FIFO_OVF_INT	BIT(0)
#define IR_UNMASK_FIFO_OVF_INT	(~BIT(0))

/* interrupt enable/disable macros */
#define ENABLE_FIFO_OVF_INTR	1
#define DISABLE_FIFO_OVF_INTR	0

/* vip versioning info */
#define VIP_MAJOR_RELEASE	0
#define VIP_MINOR_RELEASE	1
#define VIP_VERSION		((VIP_MAJOR_RELEASE << 8) | \
				VIP_MINOR_RELEASE)

/* vip field end indicators */
#define VIP_BOTTOM_FIELD_END	0
#define VIP_TOP_FIELD_END	1

static int debug_level;
module_param(debug_level, int, 0644);

/**
 * struct vip_standard - analog and digital standards supported by vip
 * @name: name of the standard
 * @width:
 * @height:
 * @frame_format: 0 - progressive frames, 1 - interlaced frames
 * @eav2sav: length of eav 2 sav
 * @sav2eav: length of sav 2 eav
 * @l1, l3, l5, l7, l9, l11: other parameter configurations
 * @vsize: vertical size of the image
 * @hd_sd: 0 - SDTV format, 1 - HDTV format
 * @std_id: SDTV format type
 * @dv_preset: HDTV format type
 */
struct vip_standard {
	char name[32];
	unsigned int width;
	unsigned int height;
	int frame_format;
	u16 eav2sav;
	u16 sav2eav;
	u16 l1, l3, l5, l7, l9, l11;
	u16 vsize;
	u8 hd_sd;
	v4l2_std_id std_id;
	u32 dv_preset;
	struct v4l2_bt_timings bt_timings;
};

static struct vip_standard vip_standards[] = {
	{
		/* SDTV format created for GE */
		.name = "PAL_GE",
		.width = 640,
		.height = 480,
		.frame_format = 0,
		.hd_sd = 0,
		.std_id = V4L2_STD_625_50,
	},
	/* HDTV formats */
	{
		.name = "480p59_94",
		.width = 720,
		.height = 480,
		.frame_format = 0,
		.hd_sd = 1,
		.dv_preset = V4L2_DV_480P59_94,
	},
	{
		.name = "576p50",
		.width = 720,
		.height = 576,
		.frame_format = 0,
		.hd_sd = 1,
		.dv_preset = V4L2_DV_576P50,
	},
	{
		.name = "720p50",
		.width = 1280,
		.height = 720,
		.frame_format = 0,
		.hd_sd = 1,
		.dv_preset = V4L2_DV_720P50,
	},
	{
		.name = "720p60",
		.width = 1280,
		.height = 720,
		.frame_format = 0,
		.hd_sd = 1,
		.dv_preset = V4L2_DV_720P60,
	},
	{
		.name = "1080I50",
		.width = 1920,
		.height = 1080,
		.frame_format = 1,
		.hd_sd = 1,
		.dv_preset = V4L2_DV_1080I50,
	},
	{
		.name = "1080I60",
		.width = 1920,
		.height = 1080,
		.frame_format = 1,
		.hd_sd = 1,
		.dv_preset = V4L2_DV_1080I60,
	},
	{
		.name = "1080p60",
		.width = 1920,
		.height = 1080,
		.frame_format = 0,
		.hd_sd = 1,
		.dv_preset = V4L2_DV_1080P60,
	},
	/*
	 * SDTV formats.
	 * FIXME: verify if only these modes are supported when VIP is
	 * configured for CVBS mode
	 */
	{
		.name = "NTSC_M",
		.width = 720,
		.height = 480,
		.frame_format = 0,
		.hd_sd = 0,
		.std_id = V4L2_STD_525_60,
	},
	{
		.name = "PAL_BDGHIK",
		.width = 720,
		.height = 576,
		.frame_format = 0,
		.hd_sd = 0,
		.std_id = V4L2_STD_625_50,
	},
};

/**
 * struct vip_fmt - vip supported formats
 * @name: name of the format
 * @fourcc: v4l2 format id
 * @pixelformat: v4l2 pixel-format
 * @depth: depth in terms of bits
 */
struct vip_fmt {
	char name[32];
	u32 fourcc;
	u32 pixelformat;
	int depth;
};

/*
 * vip supports the following formats:
 * RGB565(16bit depth) :
 * BGR888(24bit depth) : b0 is stored at least-significant-bit of byte 0
 * ABGR8888(32bit depth) : b0 is stored at least-significant-bit of byte 0
 */
static struct vip_fmt formats[] = {
	{
		.name		= "RGB-16 (5/B-6/G-5/R)",
		.fourcc		= V4L2_PIX_FMT_RGB565,
		.pixelformat	= V4L2_PIX_FMT_RGB565,
		.depth		= 16,
	}, {
		.name		= "BGR-24 (8/B-8/G-8/R)",
		.fourcc		= V4L2_PIX_FMT_BGR24,
		.pixelformat	= V4L2_PIX_FMT_BGR24,
		.depth		= 24,
	}, {
		.name		= "BGR-32 (8/A-8/R-8/G-8/B)",
		.fourcc		= V4L2_PIX_FMT_BGR32,
		.pixelformat	= V4L2_PIX_FMT_BGR32,
		.depth		= 32,
	}
};

/**
 * struct vip - vip device
 * @v4l2_dev: v4l2 device
 * @vfd: ptr to video device
 * @sd: ptr of ptr to sub devices
 * @current_subdev: ptr to currently selected sub device
 * @current_input: current input at the sub device
 * @std_info: standard info supported by this vip device
 * @std_index: index into std table
 * @fifo_irq: fifo overflow irq
 * @base: ptr to vip register base
 * @clk : ptr to vip clk source
 * @fmt: used to store pixel format
 * @vq: ptr to video buffer queue
 * @dma_queue: queue of filled frames
 * @vip_pdata: ptr to platform data
 * @lock: lock used to access this vip structure
 * @usr_count: number of users using this vip
 * @is_streaming: streaming status
 * @io_usrs: number of users performing IO
 * @irqlock: irqlock for video-buf
 * @cur_frm: ptr to current v4l2_buffer
 * @next_frm: ptr to next v4l2_buffer
 * @field_id: id of the field which is being displayed
 * @initialized: flag to indicate whether VIP is initialized
 * @prio: used to keep track of state of the priority
 * @dma_queue_lock: irqlock for DMA queue
 * @isr_count: counts the occurence of frame/field end interrupt
 * @field_off: offset where second field starts from the starting of
 *             the buffer for field separated RGB formats
 * @pdev: pointer to parent device
 */
struct vip {
	struct v4l2_device v4l2_dev;
	struct video_device *vfd;
	struct v4l2_subdev **sd;
	struct vip_subdev_info *current_subdev;
	int current_input;
	struct vip_standard std_info;
	int std_index;
	unsigned int fifo_irq;
	void __iomem *base;
	struct clk *clk;
	struct v4l2_format fmt;
	struct videobuf_queue vq;
	struct list_head dma_queue;
	struct vip_plat_data *vip_pdata;
	struct mutex lock;
	int usr_count;
	bool is_streaming;
	u32 io_usrs;
	spinlock_t irqlock;
	struct videobuf_buffer *cur_frm;
	struct videobuf_buffer *next_frm;
	u32 field_id;
	u8 initialized;
	struct v4l2_prio_state prio;
	spinlock_t dma_queue_lock;
	int isr_count;
	u32 field_off;
	struct device *pdev;
};

/**
 * struct vip_fh - vip file handle
 * @vip: file handle for this vip instance
 * @io_allowed: indicates whether this file handle is doing IO
 * @prio: used to keep track priority of this instance
 */
struct vip_fh {
	struct vip *vip;
	u8 io_allowed;
	enum v4l2_priority prio;
};

/*
 * helper routines
 */

/**
 * vip_calculate_offsets - This function calculates buffers offset
 * for top and bottom field
 * @vip: ptr to vip instance
 */
static void vip_calculate_offsets(struct vip *vip)
{
	v4l2_dbg(1, debug_level, &vip->v4l2_dev, "vip_calculate_offsets\n");

	vip->field_off = VIP_MAX_WIDTH * (VIP_MAX_HEIGHT / 2);
}

/**
 * vip_check_format - check if the passed fmt is supported by VIP
 * @f: v4l2 fomat
 */
static struct vip_fmt *vip_check_format(struct v4l2_format *f)
{
	struct vip_fmt *fmt;
	unsigned int k;

	for (k = 0; k < ARRAY_SIZE(formats); k++) {
		fmt = &formats[k];
		if (fmt->fourcc == f->fmt.pix.pixelformat)
			break;
	}

	if (k == ARRAY_SIZE(formats))
		return NULL;

	return &formats[k];
}

/**
 * vip_get_subdev_input_index - Get subdev index and subdev input index for a
 * given app input index
 * @vip: ptr to vip instance
 * @subdev_index: ptr to subdev index
 * @subdev_input_index: ptr to subdev input index
 * @app_input_index: application input index
 */
static int vip_get_subdev_input_index(struct vip *vip,
					int *subdev_index,
					int *subdev_input_index,
					int app_input_index)
{
	struct vip_plat_data *pdata = vip->vip_pdata;
	struct vip_subdev_info *sdinfo;
	int i, j = 0;

	for (i = 0; i < pdata->subdev_count; i++) {
		sdinfo = &pdata->subdev_info[i];
		if (app_input_index < (j + sdinfo->num_inputs)) {
			*subdev_index = i;
			*subdev_input_index = app_input_index - j;
			return 0;
		}
		j += sdinfo->num_inputs;
	}

	return -EINVAL;
}

/**
 * vip_get_app_input - Get app input index for a given subdev input index
 * driver stores the input index of the current sub device and translate it
 * when application request the current input
 * @vip: ptr to vip instance
 * @app_input_index: application input index
 */
static int vip_get_app_input_index(struct vip *vip,
					int *app_input_index)
{
	int i, j = 0;
	struct vip_plat_data *pdata = vip->vip_pdata;
	struct vip_subdev_info *sdinfo;

	for (i = 0; i < pdata->subdev_count; i++) {
		sdinfo = &pdata->subdev_info[i];
		if (!strcmp(sdinfo->name, vip->current_subdev->name)) {
			if (vip->current_input >= sdinfo->num_inputs)
				return -1;

			*app_input_index = j + vip->current_input;
			return 0;
		}
		j += sdinfo->num_inputs;
	}

	return -EINVAL;
}

/**
 * vip_config_image_format - For a given standard, this functions
 * sets up the default pix format in the vip instance. It first
 * starts with defaults based values from the standard table.
 * It then checks if sub device support g_mbus_fmt and then
 * overrides the values based on that.
 * @vip: ptr to vip instance
 */
static int vip_config_image_format(struct vip *vip)
{
	int i, ret;
	struct vip_subdev_info *sdinfo = vip->current_subdev;
	struct v4l2_mbus_framefmt mbus_fmt;
	enum v4l2_mbus_pixelcode code = V4L2_MBUS_FMT_BGR888_2X8_LE;
	struct v4l2_pix_format *pix = &vip->fmt.fmt.pix;

	for (i = 0; i < ARRAY_SIZE(vip_standards); i++) {
		if (!vip_standards[i].hd_sd) {
			/* SD format */
			if (vip_standards[i].std_id & vip->std_info.std_id) {
				memcpy(&vip->std_info, &vip_standards[i],
						sizeof(struct vip_standard));
				vip->std_index = i;
				break;
			}
		} else {
			/* HD format */
			if (vip_standards[i].dv_preset ==
					vip->std_info.dv_preset) {
				memcpy(&vip->std_info, &vip_standards[i],
						sizeof(struct vip_standard));
				vip->std_index = i;
				break;
			}
		}
	}

	if (i == ARRAY_SIZE(vip_standards)) {
		v4l2_err(&vip->v4l2_dev, "standard not supported\n");
		return -EINVAL;
	}

	/* set the pixel dimensions */
	pix->width = vip->std_info.width;
	pix->height = vip->std_info.height;

	/* set the pixel fmt as per the rgb_width set in the plat data */
	switch (vip->vip_pdata->config->rgb_width) {
	case SIXTEEN_BIT:
		pix->pixelformat = V4L2_PIX_FMT_RGB565;
		code = V4L2_MBUS_FMT_RGB565_2X8_LE;
		break;
	case TWENTYFOUR_BIT:
		pix->pixelformat = V4L2_PIX_FMT_BGR24;
		code = V4L2_MBUS_FMT_BGR888_2X8_LE;
		break;
	case THIRTYTWO_BIT:
		pix->pixelformat = V4L2_PIX_FMT_BGR32;
		code = V4L2_MBUS_FMT_BGR8888_2X8_LE;
		break;
	}

	/* set the pixel field */
	if (vip->std_info.frame_format)
		pix->field = V4L2_FIELD_INTERLACED;
	else
		pix->field = V4L2_FIELD_NONE;

	v4l2_fill_mbus_format(&mbus_fmt, pix, code);

	/* if sub device supports g_mbus_fmt, override the defaults */
	ret = v4l2_device_call_until_err(&vip->v4l2_dev,
			sdinfo->grp_id, video, g_mbus_fmt, &mbus_fmt);
	if (ret && ret != -ENOIOCTLCMD) {
		v4l2_err(&vip->v4l2_dev,
			"error in getting g_mbus_fmt from sub device\n");
		return ret;
	}

	v4l2_fill_pix_format(pix, &mbus_fmt);

	/* update the values of sizeimage and bytesperline */
	pix->bytesperline = pix->width *
		(vip->vip_pdata->config->rgb_width / 8);
	pix->sizeimage = pix->bytesperline * pix->height;

	return ret;
}

/*
 * videobuf helper routines
 */

/**
 * vip_free_buffer: free this videobuf and set its state to INIT
 * @vq: ptr to videobuf_queue
 * @vb: ptr to the videobuf which we want to free
 */
static void vip_free_buffer(struct videobuf_queue *vq,
				struct videobuf_buffer *vb)
{
	void *vaddr = NULL;

	BUG_ON(in_interrupt());

	videobuf_waiton(vq, vb, 0, 0);

	if (vq->int_ops && vq->int_ops->vaddr)
		vaddr = vq->int_ops->vaddr(vb);

	if (vaddr)
		videobuf_dma_contig_free(vq, vb);

	vb->state = VIDEOBUF_NEEDS_INIT;
}

/*
 * videobuf operations
 */

/**
 * vip_buffer_setup : Callback function for buffer setup.
 * @vq: buffer queue ptr
 * @count: number of buffers
 * @size: size of the buffer
 *
 * This callback function is called when reqbuf() is called to adjust
 * the buffer count and buffer size
 */
static int vip_buffer_setup(struct videobuf_queue *vq, unsigned int *count,
				unsigned int *size)
{
	struct vip_fh *fh = vq->priv_data;
	struct vip *vip = fh->vip;

	v4l2_dbg(1, debug_level, &vip->v4l2_dev, "vip_buffer_setup\n");

	*size = vip->fmt.fmt.pix.sizeimage;

	/* check for buffer count and size sanity */
	if (*count > VIDEO_MAX_FRAME)
		*count = VIDEO_MAX_FRAME;

	if (*count < VIP_MIN_BUFFER_CNT)
		*count = VIP_MIN_BUFFER_CNT;

	if (*size >= VIP_OPTIMAL_BUFFER_SIZE)
		*size = VIP_OPTIMAL_BUFFER_SIZE;

	v4l2_dbg(1, debug_level, &vip->v4l2_dev, "count=%d, size=%d\n",
			*count, *size);

	return 0;
}

/**
 * vip_buffer_prepare : callback function for buffer prepare
 * This is the callback function for buffer prepare when videobuf_qbuf()
 * function is called.
 * @vq : buffer queue ptr
 * @vb: ptr to video buffer
 * @field: field info
 */
static int vip_buffer_prepare(struct videobuf_queue *vq,
				struct videobuf_buffer *vb,
				enum v4l2_field field)
{
	int ret;
	struct vip_fh *fh = vq->priv_data;
	struct vip *vip = fh->vip;

	v4l2_dbg(1, debug_level, &vip->v4l2_dev, "vip_buffer_prepare\n");

	/* if buffer is not initialized, initialize it */
	if (vb->state == VIDEOBUF_NEEDS_INIT) {
		vb->width = vip->fmt.fmt.pix.width;
		vb->height = vip->fmt.fmt.pix.height;
		vb->size = vip->fmt.fmt.pix.sizeimage;
		vb->field = field;

		ret = videobuf_iolock(vq, vb, NULL);
		if (ret)
			goto fail;

		vb->state = VIDEOBUF_PREPARED;
	}

	return 0;
fail:
	vip_free_buffer(vq, vb);

	return ret;
}

/**
 * vip_buffer_queue : Callback function to add buffer to DMA queue
 * @vq: ptr to videobuf_queue
 * @vb: ptr to videobuf_buffer
 */
static void vip_buffer_queue(struct videobuf_queue *vq,
				struct videobuf_buffer *vb)
{
	unsigned long flags;
	struct vip_fh *fh = vq->priv_data;
	struct vip *vip = fh->vip;

	v4l2_dbg(1, debug_level, &vip->v4l2_dev, "vip_buffer_queue\n");

	/* add the buffer to the DMA queue */
	spin_lock_irqsave(&vip->dma_queue_lock, flags);
	list_add_tail(&vb->queue, &vip->dma_queue);
	spin_unlock_irqrestore(&vip->dma_queue_lock, flags);

	/* change state of the buffer */
	vb->state = VIDEOBUF_QUEUED;
}

/**
 * vip_buffer_release : Callback function to free buffer
 * @vq: buffer queue ptr
 * @vb: ptr to video buffer
 *
 * This function is called from the videobuf layer to free memory
 * allocated to the buffers
 */
static void vip_buffer_release(struct videobuf_queue *vq,
				struct videobuf_buffer *vb)
{
	unsigned long flags;
	struct vip_fh *fh = vq->priv_data;
	struct vip *vip = fh->vip;

	v4l2_dbg(1, debug_level, &vip->v4l2_dev, "vip_buffer_release\n");

	/*
	 * we need to flush the buffer from the dma queue since
	 * they are de-allocated
	 */
	spin_lock_irqsave(&vip->dma_queue_lock, flags);
	INIT_LIST_HEAD(&vip->dma_queue);
	spin_unlock_irqrestore(&vip->dma_queue_lock, flags);

	videobuf_dma_contig_free(vq, vb);
	vb->state = VIDEOBUF_NEEDS_INIT;
}

/* vip videobuf operations */
static struct videobuf_queue_ops vip_qops = {
	.buf_setup = vip_buffer_setup,
	.buf_prepare = vip_buffer_prepare,
	.buf_queue = vip_buffer_queue,
	.buf_release = vip_buffer_release,
};

/**
 * vip_querycap() - QUERYCAP handler
 * @file: file ptr
 * @priv: file handle
 * @cap: ptr to v4l2_capability structure
 */
static int vip_querycap(struct file *file, void *priv,
				struct v4l2_capability *cap)
{
	struct vip *vip = video_drvdata(file);

	v4l2_dbg(1, debug_level, &vip->v4l2_dev, "vip_querycap\n");

	/* we support capture and streaming */
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;

	/* fill other capabilities */
	cap->version = VIP_VERSION;
	strlcpy(cap->driver, "spear_vip", sizeof(cap->driver));
	strlcpy(cap->card, vip->vip_pdata->card_name, sizeof(cap->card));
	strlcpy(cap->bus_info, vip->v4l2_dev.name, sizeof(cap->bus_info));

	return 0;
}

/**
 * vip_enum_fmt_vid_cap() - ENUM_FMT handler
 * @file: file ptr
 * @priv: file handle
 * @f: v4l2 specific format desc
 */
static int vip_enum_fmt_vid_cap(struct file *file, void *priv,
					struct v4l2_fmtdesc *f)
{
	struct vip_fmt *fmt;
	struct vip *vip = video_drvdata(file);

	v4l2_dbg(1, debug_level, &vip->v4l2_dev, "vip_enum_fmt_vid_cap\n");

	if (f->index >= ARRAY_SIZE(formats))
		return -EINVAL;

	fmt = &formats[f->index];

	strlcpy(f->description, fmt->name, sizeof(f->description));
	f->pixelformat = fmt->fourcc;

	return 0;
}

/**
 * vip_g_fmt_vid_cap() - Set INPUT handler
 * @file: file ptr
 * @priv: file handle
 * @f: ptr to v4l2 format structure
 */
static int vip_g_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct vip *vip = video_drvdata(file);

	v4l2_dbg(1, debug_level, &vip->v4l2_dev, "vip_g_fmt_vid_cap\n");

	/* fill-in the format information */
	*f = vip->fmt;

	return 0;
}

/**
 * vip_try_fmt_vid_cap() - TRY_FMT handler
 * @file: file ptr
 * @priv: file handle
 * @f: ptr to v4l2 format structure
 */
static int vip_try_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct vip_fmt *fmt;
	enum v4l2_field field;
	struct vip *vip = video_drvdata(file);

	v4l2_dbg(1, debug_level, &vip->v4l2_dev, "vip_try_fmt_vid_cap\n");

	fmt = vip_check_format(f);
	if (!fmt) {
		v4l2_err(&vip->v4l2_dev,
			"fourcc format (0x%08x) invalid\n",
			f->fmt.pix.pixelformat);
		return -EINVAL;
	}

	field = f->fmt.pix.field;

	switch (field) {
	case V4L2_FIELD_NONE:
		/*
		 * progressive frame request.
		 * Check underlying subdev field type.
		 * If it supports INTERLACED, use that otherwise
		 * we support PROGRESSIVE mode.
		 */
		if (vip->std_info.frame_format)
			field = V4L2_FIELD_INTERLACED;
		break;
	case V4L2_FIELD_INTERLACED:
	case V4L2_FIELD_SEQ_TB:
		/*
		 * interlace field request.
		 * Check underlying subdev field type.
		 * If it supports PROGRESSIVE, use that otherwise
		 * we support INTERLACED mode.
		 */
		if (!vip->std_info.frame_format)
			field = V4L2_FIELD_NONE;
		break;
	case V4L2_FIELD_ANY:
		/* if field is any, use current value as default */
		field = vip->fmt.fmt.pix.field;
		break;
	default:
		/*
		 * we don't recognize this field type.
		 * Warn user and restore current field as default
		 */
		v4l2_warn(&vip->v4l2_dev,
			"field type %d not recognized\n",
			field);
		field = vip->fmt.fmt.pix.field;
		break;
	}

	f->fmt.pix.field = field;

	v4l2_info(&vip->v4l2_dev,
			"requested :: width = %d, height = %d\n",
			f->fmt.pix.width, f->fmt.pix.height);

	/* ensure image boundary sanity */
	if (f->fmt.pix.width < VIP_MIN_WIDTH)
		f->fmt.pix.width = VIP_MIN_WIDTH;

	if (f->fmt.pix.width > VIP_MAX_WIDTH)
		f->fmt.pix.width = VIP_MAX_WIDTH;

	if (f->fmt.pix.height < VIP_MIN_HEIGHT)
		f->fmt.pix.height = VIP_MIN_HEIGHT;

	if (f->fmt.pix.height > VIP_MAX_HEIGHT)
		f->fmt.pix.height = VIP_MAX_HEIGHT;

	/* calculate bytes-per-line and image size */
	f->fmt.pix.bytesperline =
		(f->fmt.pix.width * fmt->depth) / 8;
	f->fmt.pix.sizeimage =
		f->fmt.pix.height * f->fmt.pix.bytesperline;

	v4l2_info(&vip->v4l2_dev,
			"adjusted :: width = %d, height = %d, bpp = %d, "
			"bytesperline = %d, sizeimage = %d\n",
			f->fmt.pix.width, f->fmt.pix.height, fmt->depth,
			f->fmt.pix.bytesperline, f->fmt.pix.sizeimage);

	return 0;
}

/**
 * vip_s_fmt_vid_cap() - Set FMT handler
 * @file: file ptr
 * @priv: file handle
 * @f: ptr to v4l2 format structure
 */
static int vip_s_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	int ret;
	struct vip_fh *fh = priv;
	struct vip *vip = fh->vip;

	v4l2_dbg(1, debug_level, &vip->v4l2_dev, "vip_s_fmt_vid_cap\n");

	/* if streaming is already started, return error */
	if (vip->is_streaming) {
		v4l2_err(&vip->v4l2_dev, "streaming is already started\n");
		ret = -EBUSY;
		goto out;
	}

	/* first check if we support this new fmt */
	ret = vip_try_fmt_vid_cap(file, fh, f);
	if (ret < 0)
		goto out;

	vip->fmt = *f;

	return 0;

out:
	return ret;
}

/**
 * vip_enum_input() - ENUMINPUT handler
 * @file: file ptr
 * @priv: file handle
 * @inp: ptr to input structure
 */
static int vip_enum_input(struct file *file, void *priv,
				struct v4l2_input *inp)
{
	struct vip *vip = video_drvdata(file);
	struct vip_subdev_info *sdinfo;
	int subdev, index;

	v4l2_dbg(1, debug_level, &vip->v4l2_dev, "vip_enum_input\n");

	if (vip_get_subdev_input_index(vip,
					&subdev,
					&index,
					inp->index) < 0) {
		v4l2_err(&vip->v4l2_dev, "input information not found"
			 " for the subdev\n");
		return -EINVAL;
	}
	sdinfo = &vip->vip_pdata->subdev_info[subdev];
	memcpy(inp, &sdinfo->inputs[index], sizeof(struct v4l2_input));

	return 0;
}

/**
 * vip_g_input() - Get INPUT handler
 * @file: file ptr
 * @priv: file handle
 * @index: ptr to input index
 */
static int vip_g_input(struct file *file, void *priv, unsigned int *index)
{
	struct vip *vip = video_drvdata(file);

	v4l2_dbg(1, debug_level, &vip->v4l2_dev, "vip_g_input\n");

	return vip_get_app_input_index(vip, index);
}

/**
 * vip_reqbufs. request buffer handler
 * Note: currently we support REQBUF to open the device only once.
 * @file: file ptr
 * @priv: file handle
 * @req_buf: request buffer structure ptr
 */
static int vip_reqbufs(struct file *file, void *priv,
			struct v4l2_requestbuffers *req_buf)
{
	int ret;
	struct vip *vip = video_drvdata(file);
	struct vip_fh *fh = file->private_data;

	v4l2_dbg(1, debug_level, &vip->v4l2_dev, "vip_reqbufs\n");

	if (req_buf->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		v4l2_err(&vip->v4l2_dev, "invalid buffer type in reqbufs\n");
		return -EINVAL;
	}

	ret = mutex_lock_interruptible(&vip->lock);
	if (ret)
		return ret;

	if (vip->io_usrs != 0) {
		v4l2_err(&vip->v4l2_dev, "Only one IO user allowed\n");
		ret = -EBUSY;
		goto unlock_out;
	}

	videobuf_queue_dma_contig_init(&vip->vq,
				&vip_qops, vip->pdev,
				&vip->irqlock, req_buf->type,
				vip->fmt.fmt.pix.field,
				sizeof(struct videobuf_buffer),
				fh, NULL);

	fh->io_allowed = 1;
	vip->io_usrs = 1;
	INIT_LIST_HEAD(&vip->dma_queue);
	ret = videobuf_reqbufs(&vip->vq, req_buf);

unlock_out:
	mutex_unlock(&vip->lock);

	return ret;
}

/**
 * vip_querybuf() - query buffer handler
 * @file: file ptr
 * @priv: file handle
 * @buf: v4l2 buffer structure ptr
 */
static int vip_querybuf(struct file *file, void *priv,
			struct v4l2_buffer *buf)
{
	struct vip *vip = video_drvdata(file);

	v4l2_dbg(1, debug_level, &vip->v4l2_dev, "vip_querybuf\n");

	if (buf->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		v4l2_err(&vip->v4l2_dev, "invalid buf type in querybuf\n");
		return -EINVAL;
	}

	return videobuf_querybuf(&vip->vq, buf);
}

/**
 * vip_qbuf() - queque new buffer handler
 * @file: file ptr
 * @priv: file handle
 * @p: v4l2 buffer structure ptr
 */
static int vip_qbuf(struct file *file, void *priv,
			struct v4l2_buffer *p)
{
	struct vip *vip = video_drvdata(file);
	struct vip_fh *fh = file->private_data;

	v4l2_dbg(1, debug_level, &vip->v4l2_dev, "vip_qbuf\n");

	if (p->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		v4l2_err(&vip->v4l2_dev, "invalid buf type in qbuf\n");
		return -EINVAL;
	}

	/*
	 * if this file handle is not allowed to do IO,
	 * return error
	 */
	if (!fh->io_allowed) {
		v4l2_err(&vip->v4l2_dev, "fh->io_allowed = 0\n");
		return -EACCES;
	}

	return videobuf_qbuf(&vip->vq, p);
}

/**
 * vip_dqbuf() - de-queque buffer handler
 * @file: file ptr
 * @priv: file handle
 * @buf: v4l2 buffer structure ptr
 */
static int vip_dqbuf(struct file *file, void *priv,
			struct v4l2_buffer *buf)
{
	struct vip *vip = video_drvdata(file);

	v4l2_dbg(1, debug_level, &vip->v4l2_dev, "vip_dqbuf\n");

	if (buf->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		v4l2_err(&vip->v4l2_dev, "invalid buf type in dqbuf\n");
		return -EINVAL;
	}

	return videobuf_dqbuf(&vip->vq,
			buf, file->f_flags & O_NONBLOCK);
}

/*
 * vip_streamon. streamon handler
 * Assume the DMA queue is not empty.
 * application is expected to call QBUF before calling
 * this ioctl. If not, driver returns error.
 * @file: file ptr
 * @priv: file handle
 * @buf_type: v4l2 buffer type
 */
static int vip_streamon(struct file *file, void *priv,
			 enum v4l2_buf_type buf_type)
{
	int ret;
	unsigned long addr;
	struct vip *vip = video_drvdata(file);
	struct vip_fh *fh = file->private_data;
	struct vip_subdev_info *sdinfo;

	v4l2_dbg(1, debug_level, &vip->v4l2_dev, "vip_streamon\n");

	if (buf_type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		v4l2_err(&vip->v4l2_dev, "invalid buf type in streamon\n");
		return -EINVAL;
	}

	/* if this file handle is not allowed to perfrom IO, return error */
	if (!fh->io_allowed) {
		v4l2_err(&vip->v4l2_dev, "fh->io_allowed = 0\n");
		return -EACCES;
	}

	/* if streaming is already started, return error */
	if (vip->is_streaming) {
		v4l2_err(&vip->v4l2_dev, "streaming is already started\n");
		return -EINVAL;
	}

	sdinfo = vip->current_subdev;

	/* call s_stream of the subdev */
	ret = v4l2_device_call_until_err(&vip->v4l2_dev, sdinfo->grp_id,
					video, s_stream, 1);
	if (ret && (ret != -ENOIOCTLCMD)) {
		v4l2_err(&vip->v4l2_dev, "streamon failed in subdev\n");
		return -EINVAL;
	}

	/* if buffer queue is empty, return error */
	if (list_empty(&vip->vq.stream)) {
		v4l2_err(&vip->v4l2_dev, "buffer queue is empty\n");
		return -EIO;
	}

	/* call videobuf_streamon to start streaming in videobuf */
	ret = videobuf_streamon(&vip->vq);
	if (ret)
		return ret;

	ret = mutex_lock_interruptible(&vip->lock);
	if (ret) {
		mutex_unlock(&vip->lock);
		goto streamoff;
	}

	/* get the next frame from the buffer queue */
	vip->next_frm = list_entry(vip->dma_queue.next,
					struct videobuf_buffer, queue);
	vip->cur_frm = vip->next_frm;
	/* remove buffer from the buffer queue */
	list_del(&vip->cur_frm->queue);
	/* mark state of the current frame to active */
	vip->cur_frm->state = VIDEOBUF_ACTIVE;

	/*
	 * initialize field_id and calculate system memory address where
	 * the frames will be stored
	 */
	vip->field_id = VIP_BOTTOM_FIELD_END;
	addr = videobuf_to_dma_contig(vip->cur_frm);

	/* calculate field offset */
	vip_calculate_offsets(vip);

	/* program the base address in vip register */
	writel(addr, vip->base + VIP_VDO_BASE);

	/* set streaming flag */
	vip->is_streaming = 1;

	mutex_unlock(&vip->lock);

	return ret;

streamoff:
	ret = videobuf_streamoff(&vip->vq);

	return ret;
}

/**
 * vip_streamoff() - streamoff handler
 * @file: file ptr
 * @priv: file handle
 * @buf_type: v4l2 buffer type
 */
static int vip_streamoff(struct file *file, void *priv,
			enum v4l2_buf_type buf_type)
{
	int ret;
	struct vip *vip = video_drvdata(file);
	struct vip_fh *fh = file->private_data;
	struct vip_subdev_info *sdinfo;

	v4l2_dbg(1, debug_level, &vip->v4l2_dev, "vip_streamoff\n");

	if (buf_type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		v4l2_err(&vip->v4l2_dev, "invalid buf type in streamoff\n");
		return -EINVAL;
	}

	/* if io is not allowed for this file handle, return error */
	if (!fh->io_allowed) {
		v4l2_err(&vip->v4l2_dev, "fh->io_allowed = 0\n");
		return -EACCES;
	}

	/* if streaming is not started, return error */
	if (!vip->is_streaming) {
		v4l2_err(&vip->v4l2_dev, "streaming is already stopped\n");
		return -EINVAL;
	}

	ret = mutex_lock_interruptible(&vip->lock);
	if (ret)
		return ret;

	/* clear streaming flag */
	vip->is_streaming = 0;

	sdinfo = vip->current_subdev;

	ret = v4l2_device_call_until_err(&vip->v4l2_dev, sdinfo->grp_id,
					video, s_stream, 0);
	if (ret && (ret != -ENOIOCTLCMD))
		v4l2_err(&vip->v4l2_dev, "stream off failed in subdev\n");

	ret = videobuf_streamoff(&vip->vq);
	mutex_unlock(&vip->lock);

	return ret;
}

/**
 * vip_s_input() - Set INPUT handler
 * @file: file ptr
 * @priv: file handle
 * @index: input index
 */
static int vip_s_input(struct file *file, void *priv, unsigned int index)
{
	struct vip *vip = video_drvdata(file);
	struct vip_subdev_info *sdinfo;
	int subdev_index, inp_index;
	struct vip_subdev_route *route;
	u32 input = 0, output = 0;
	int ret;

	v4l2_dbg(1, debug_level, &vip->v4l2_dev, "vip_s_input\n");

	ret = mutex_lock_interruptible(&vip->lock);
	if (ret)
		return ret;

	/* if streaming is started return device busy error */
	if (vip->is_streaming) {
		v4l2_err(&vip->v4l2_dev, "busy in streaming\n");
		ret = -EBUSY;
		goto unlock_out;
	}

	if (vip_get_subdev_input_index(vip,
					&subdev_index,
					&inp_index,
					index) < 0) {
		v4l2_err(&vip->v4l2_dev, "invalid input index\n");
		goto unlock_out;
	}

	sdinfo = &vip->vip_pdata->subdev_info[subdev_index];
	route = &sdinfo->routes[inp_index];
	if (route && sdinfo->can_route) {
		input = route->input;
		output = route->output;
	}

	ret = v4l2_device_call_until_err(&vip->v4l2_dev, sdinfo->grp_id,
					 video, s_routing, input, output, 0);

	if (ret) {
		v4l2_err(&vip->v4l2_dev,
			"vip_doioctl:error in setting input on subdev\n");
		ret = -EINVAL;
		goto unlock_out;
	}
	vip->current_subdev = sdinfo;
	vip->current_input = index;
	vip->std_index = 0;
	vip->std_info.std_id = vip_standards[vip->std_index].std_id;

	/* set the default image parameters in the device */
	ret = vip_config_image_format(vip);
unlock_out:
	mutex_unlock(&vip->lock);

	return ret;
}

/**
 * vip_querystd() - querystd handler
 * @file: file ptr
 * @priv: file handle
 * @std_id: ptr to std id
 *
 * This function is called to detect standard at the selected input
 */
static int vip_querystd(struct file *file, void *priv, v4l2_std_id *std_id)
{
	int ret;
	struct vip *vip = video_drvdata(file);
	struct vip_subdev_info *sdinfo;

	v4l2_dbg(1, debug_level, &vip->v4l2_dev, "vip_querystd\n");

	ret = mutex_lock_interruptible(&vip->lock);
	if (ret)
		return ret;

	sdinfo = vip->current_subdev;

	/* call querystd function of the subdev */
	ret = v4l2_device_call_until_err(&vip->v4l2_dev, sdinfo->grp_id,
					 video, querystd, std_id);
	if (ret < 0)
		v4l2_err(&vip->v4l2_dev, "querystd failed for sub device\n");

	mutex_unlock(&vip->lock);

	return ret;
}

/**
 * vip_g_std() - get STD handler
 * @file: file ptr
 * @priv: file handle
 * @std_id: ptr to std id
 */
static int vip_g_std(struct file *file, void *priv,
		v4l2_std_id *std_id)
{
	struct vip *vip = video_drvdata(file);

	v4l2_dbg(1, debug_level, &vip->v4l2_dev, "vip_g_std\n");

	*std_id = vip_standards[vip->std_index].std_id;

	return 0;
}

/**
 * vip_s_std() - set STD handler
 * @file: file ptr
 * @priv: file handle
 * @std_id: ptr to std id
 */
static int vip_s_std(struct file *file, void *priv, v4l2_std_id *std_id)
{
	int ret;
	struct vip *vip = video_drvdata(file);
	struct vip_subdev_info *sdinfo;

	v4l2_dbg(1, debug_level, &vip->v4l2_dev, "vip_s_std\n");

	ret = mutex_lock_interruptible(&vip->lock);
	if (ret)
		return ret;

	sdinfo = vip->current_subdev;

	/* if streaming is already started, return device busy error */
	if (vip->is_streaming) {
		v4l2_err(&vip->v4l2_dev, "busy in streaming\n");
		ret = -EBUSY;
		goto unlock_out;
	}

	/* call subdev's s_std function to set the standard */
	ret = v4l2_device_call_until_err(&vip->v4l2_dev, sdinfo->grp_id,
					 core, s_std, *std_id);
	if (ret < 0) {
		v4l2_err(&vip->v4l2_dev, "s_std failed for sub device\n");
		goto unlock_out;
	}

	/*
	 * update the analog standard information set for subdev for
	 * vip as well
	 */
	vip->std_info.std_id = *std_id;
	vip->std_info.dv_preset = V4L2_DV_INVALID;
	memset(&vip->std_info.bt_timings, 0, sizeof(vip->std_info.bt_timings));

	ret = vip_config_image_format(vip);

unlock_out:
	mutex_unlock(&vip->lock);

	return ret;
}

/**
 * vip_enum_dv_presets() - ENUM_DV_PRESETS handler
 * @file: file ptr
 * @priv: file handle
 * @preset: input preset
 */
static int vip_enum_dv_presets(struct file *file, void *priv,
		struct v4l2_dv_enum_preset *preset)
{
	int ret;
	struct vip *vip = video_drvdata(file);
	struct vip_subdev_info *sdinfo;

	v4l2_dbg(1, debug_level, &vip->v4l2_dev, "vip_enum_dv_presets\n");

	ret = mutex_lock_interruptible(&vip->lock);
	if (ret)
		return ret;

	sdinfo = vip->current_subdev;

	/*
	 * call subdev's enum_dv_presets to get the dv presets supported
	 * by the subdev
	 */
	ret = v4l2_device_call_until_err(&vip->v4l2_dev, sdinfo->grp_id,
					video, enum_dv_presets, preset);
	if (ret < 0) {
		v4l2_err(&vip->v4l2_dev,
				"enum_dv_presets failed for sub device\n");
		goto unlock_out;
	}

unlock_out:
	mutex_unlock(&vip->lock);

	return ret;
}

/**
 * vip_query_dv_presets() - QUERY_DV_PRESET handler
 * @file: file ptr
 * @priv: file handle
 * @preset: input preset
 */
static int vip_query_dv_preset(struct file *file, void *priv,
				struct v4l2_dv_preset *preset)
{
	int ret;
	struct vip *vip = video_drvdata(file);
	struct vip_subdev_info *sdinfo;

	v4l2_dbg(1, debug_level, &vip->v4l2_dev, "vip_query_dv_preset\n");

	ret = mutex_lock_interruptible(&vip->lock);
	if (ret)
		return ret;

	sdinfo = vip->current_subdev;

	/*
	 * call subdev's query_dv_preset to get the dv presets supported
	 * by the subdev
	 */
	ret = v4l2_device_call_until_err(&vip->v4l2_dev, sdinfo->grp_id,
					video, query_dv_preset, preset);
	if (ret < 0) {
		v4l2_err(&vip->v4l2_dev,
				"query_dv_preset failed for sub device\n");
		goto unlock_out;
	}

unlock_out:
	mutex_unlock(&vip->lock);

	return ret;
}

/**
 * vip_s_dv_presets() - S_DV_PRESETS handler
 * @file: file ptr
 * @priv: file handle
 * @preset: input preset
 */
static int vip_s_dv_preset(struct file *file, void *priv,
		struct v4l2_dv_preset *preset)
{
	int ret;
	struct vip *vip = video_drvdata(file);
	struct vip_subdev_info *sdinfo;

	v4l2_dbg(1, debug_level, &vip->v4l2_dev, "vip_query_dv_preset\n");

	ret = mutex_lock_interruptible(&vip->lock);
	if (ret)
		return ret;

	sdinfo = vip->current_subdev;

	/* if streaming is already started, return device busy error */
	if (vip->is_streaming) {
		v4l2_err(&vip->v4l2_dev, "busy in streaming\n");
		ret = -EBUSY;
		goto unlock_out;
	}

	/*
	 * call subdev's query_dv_preset to get the dv presets supported
	 * by the subdev
	 */
	ret = v4l2_device_call_until_err(&vip->v4l2_dev, sdinfo->grp_id,
					video, query_dv_preset, preset);
	if (ret < 0) {
		v4l2_err(&vip->v4l2_dev,
				"query_dv_preset failed for sub device\n");
		goto unlock_out;
	}

	/*
	 * update the digital standard information set for subdev for
	 * vip as well
	 */
	vip->std_info.std_id = V4L2_STD_UNKNOWN;
	vip->std_info.dv_preset = preset->preset;
	memset(&vip->std_info.bt_timings, 0, sizeof(vip->std_info.bt_timings));

	ret = vip_config_image_format(vip);

unlock_out:
	mutex_unlock(&vip->lock);

	return ret;
}

/**
 * vip_g_dv_presets() - G_DV_PRESETS handler
 * @file: file ptr
 * @priv: file handle
 * @preset: input preset
 */
static int vip_g_dv_preset(struct file *file, void *priv,
				struct v4l2_dv_preset *preset)
{
	struct vip *vip = video_drvdata(file);

	v4l2_dbg(1, debug_level, &vip->v4l2_dev, "vip_g_dv_preset\n");

	preset->preset = vip->std_info.dv_preset;

	return 0;
}

/**
 * vip_s_dv_timings() - S_DV_TIMINGS handler
 * @file: file ptr
 * @priv: file handle
 * @timings: digital video timings
 */
static int vip_s_dv_timings(struct file *file, void *priv,
				struct v4l2_dv_timings *timings)
{
	int ret;
	struct vip *vip = video_drvdata(file);
	struct vip_subdev_info *sdinfo;
	struct v4l2_bt_timings *bt = &vip->std_info.bt_timings;

	v4l2_dbg(1, debug_level, &vip->v4l2_dev, "vip_s_dv_timings\n");

	if (timings->type != V4L2_DV_BT_656_1120) {
		v4l2_err(&vip->v4l2_dev, "BT.1120 timing not defined\n");
		return -EINVAL;
	}

	ret = mutex_lock_interruptible(&vip->lock);
	if (ret)
		return ret;

	sdinfo = vip->current_subdev;

	/* if streaming is already started, return device busy error */
	if (vip->is_streaming) {
		v4l2_err(&vip->v4l2_dev, "busy in streaming\n");
		ret = -EBUSY;
		goto unlock_out;
	}

	/*
	 * call subdev's s_dv_timings to set the dv timings supported
	 * by the subdev
	 */
	ret = v4l2_device_call_until_err(&vip->v4l2_dev, sdinfo->grp_id,
				video, s_dv_timings, timings);
	if (ret == -ENOIOCTLCMD) {
		v4l2_err(&vip->v4l2_dev, "custom DV timings not supported by "
					"subdevice\n");
		ret = -EINVAL;
		goto unlock_out;

	}
	if (ret < 0) {
		v4l2_err(&vip->v4l2_dev, "error setting custom DV timings\n");
		goto unlock_out;
	}

	if (!(timings->bt.width && timings->bt.height &&
				(timings->bt.hbackporch ||
				 timings->bt.hfrontporch ||
				 timings->bt.hsync) &&
				timings->bt.vfrontporch &&
				(timings->bt.vbackporch ||
				 timings->bt.vsync))) {
		v4l2_err(&vip->v4l2_dev, "Timings for width, height, "
				"horizontal back porch, horizontal sync, "
				"horizontal front porch, vertical back porch, "
				"vertical sync and vertical back porch "
				"must be defined\n");
		return -EINVAL;
	}

	*bt = timings->bt;

	/* configure vip timings */
	vip->std_info.eav2sav = bt->hbackporch + bt->hfrontporch +
				bt->hsync - 8;
	vip->std_info.sav2eav = bt->width;

	vip->std_info.l1 = 1;
	vip->std_info.l3 = bt->vsync + bt->vbackporch + 1;

	if (bt->interlaced) {
		if (bt->il_vbackporch || bt->il_vfrontporch || bt->il_vsync) {
			vip->std_info.vsize = bt->height * 2 +
				bt->vfrontporch + bt->vsync + bt->vbackporch +
				bt->il_vfrontporch + bt->il_vsync +
				bt->il_vbackporch;
			vip->std_info.l5 = vip->std_info.vsize/2 -
				(bt->vfrontporch - 1);
			vip->std_info.l7 = vip->std_info.vsize/2 + 1;
			vip->std_info.l9 = vip->std_info.l7 + bt->il_vsync +
				bt->il_vbackporch + 1;
			vip->std_info.l11 = vip->std_info.vsize -
				(bt->il_vfrontporch - 1);
		} else {
			v4l2_err(&vip->v4l2_dev, "Required timing values for "
					"interlaced BT format missing\n");
			return -EINVAL;
		}
	} else {
		vip->std_info.vsize = bt->height + bt->vfrontporch +
			bt->vsync + bt->vbackporch;
		vip->std_info.l5 = vip->std_info.vsize - (bt->vfrontporch - 1);
	}

	strncpy(vip->std_info.name, "Custom timings BT656/1120", 32);
	vip->std_info.width = bt->width;
	vip->std_info.height = bt->height;
	vip->std_info.frame_format = bt->interlaced ? 0 : 1;
	vip->std_info.hd_sd = 1;
	vip->std_info.std_id = 0;
	vip->std_info.dv_preset = V4L2_DV_INVALID;

unlock_out:
	mutex_unlock(&vip->lock);

	return ret;
}

/**
 * vip_g_dv_timings() - G_DV_TIMINGS handler
 * @file: file ptr
 * @priv: file handle
 * @timings: digital video timings
 */
static int vip_g_dv_timings(struct file *file, void *priv,
		struct v4l2_dv_timings *timings)
{
	struct vip *vip = video_drvdata(file);
	struct v4l2_bt_timings *bt = &vip->std_info.bt_timings;

	timings->bt = *bt;

	return 0;
}

/*
 * vip_g_chip_ident() - Identify the chip
 * @file: file ptr
 * @priv: file handle
 * @chip: chip identity
 *
 * Returns zero or -EINVAL if read operations fails.
 */
static int vip_g_chip_ident(struct file *file, void *priv,
				struct v4l2_dbg_chip_ident *chip)
{
	int ret;
	struct vip *vip = video_drvdata(file);
	struct vip_subdev_info *sdinfo;

	v4l2_dbg(1, debug_level, &vip->v4l2_dev, "vip_g_chip_ident\n");

	ret = mutex_lock_interruptible(&vip->lock);
	if (ret)
		return ret;

	sdinfo = vip->current_subdev;

	if (chip->match.type != V4L2_CHIP_MATCH_I2C_DRIVER &&
			chip->match.type != V4L2_CHIP_MATCH_I2C_ADDR) {
		v4l2_err(&vip->v4l2_dev, "match_type is invalid\n");
		ret = -EINVAL;
		goto unlock_out;
	}

	chip->ident = V4L2_IDENT_NONE;
	chip->revision = 0;

	/* call subdev's g_chip_ident to get the chip identification */
	ret = v4l2_device_call_until_err(&vip->v4l2_dev, sdinfo->grp_id,
					core, g_chip_ident, chip);
	if (ret < 0) {
		v4l2_err(&vip->v4l2_dev,
				"g_chip_ident failed for sub device\n");
		goto unlock_out;
	}

unlock_out:
	mutex_unlock(&vip->lock);

	return ret;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
/*
 * vip_dbg_g_register() - Read register
 * @file: file ptr
 * @priv: file handle
 * @reg: register to be read
 *
 * debugging only
 * Returns zero or -EINVAL if read operations fails.
 */
static int vip_dbg_g_register(struct file *file, void *priv,
				struct v4l2_dbg_register *reg)
{
	int ret;
	struct vip *vip = video_drvdata(file);

	v4l2_dbg(1, debug_level, &vip->v4l2_dev, "vip_dbg_g_register\n");

	ret = mutex_lock_interruptible(&vip->lock);
	if (ret)
		return ret;

	sdinfo = vip->current_subdev;

	/* call subdev's g_register to get the chip identification */
	ret = v4l2_device_call_until_err(&vip->v4l2_dev, sdinfo->grp_id,
					core, g_register, reg);
	if (ret < 0)
		v4l2_err(&vip->v4l2_dev,
				"g_register failed for sub device\n");

	mutex_unlock(&vip->lock);

	return ret;
}

/*
 * vip_dbg_s_register() - Write to register
 * @file: file ptr
 * @priv: file handle
 * @reg: register to be modified
 *
 * Debugging only
 * Returns zero or -EINVAL if write operations fails.
 */
static int vip_dbg_s_register(struct file *file, void *priv,
				struct v4l2_dbg_register *reg)
{
	int ret;
	struct vip *vip = video_drvdata(file);

	v4l2_dbg(1, debug_level, &vip->v4l2_dev, "vip_dbg_s_register\n");

	ret = mutex_lock_interruptible(&vip->lock);
	if (ret)
		return ret;

	sdinfo = vip->current_subdev;

	/* call subdev's s_register to get the chip identification */
	ret = v4l2_device_call_until_err(&vip->v4l2_dev, sdinfo->grp_id,
					core, s_register, reg);
	if (ret < 0)
		v4l2_err(&vip->v4l2_dev,
				"s_register failed for sub device\n");

	mutex_unlock(&vip->lock);

	return ret;
}
#endif /* CONFIG_VIDEO_ADV_DEBUG */

/*
 * vip_log_status() - Status information
 * @file: file ptr
 * @priv: file handle
 *
 * Returns zero.
 */
static int vip_log_status(struct file *file, void *priv)
{
	int ret;
	struct vip *vip = video_drvdata(file);

	v4l2_dbg(1, debug_level, &vip->v4l2_dev, "vip_log_status\n");

	/*
	 * get log status for all sub devices connected to this host
	 * driver
	 */
	ret = v4l2_device_call_until_err(&vip->v4l2_dev, 0,
					core, log_status);
	if (ret < 0)
		v4l2_err(&vip->v4l2_dev,
				"log_status s_register failed\n");

	return ret;
}

/**
 * vip_cropcap() - cropcap handler
 * Note: VIP by itself does not provide any cropping capabilities,
 * so we rely entirely on a subdev capable of supporting cropping.
 * @file: file ptr
 * @priv: file handle
 * @crop: ptr to v4l2_cropcap
 */
static int vip_cropcap(struct file *file, void *priv,
			struct v4l2_cropcap *crop)
{
	int ret;
	struct vip *vip = video_drvdata(file);
	struct vip_subdev_info *sdinfo;

	v4l2_dbg(1, debug_level, &vip->v4l2_dev, "vip_cropcap\n");

	ret = mutex_lock_interruptible(&vip->lock);
	if (ret)
		return ret;

	sdinfo = vip->current_subdev;

	/* call subdev's cropcap function to get the cropping capabilities */
	ret = v4l2_device_call_until_err(&vip->v4l2_dev, sdinfo->grp_id,
					 video, cropcap, crop);
	if (ret < 0) {
		v4l2_err(&vip->v4l2_dev, "cropcap failed for sub device\n");
		goto unlock_out;
	}

unlock_out:
	mutex_unlock(&vip->lock);

	return ret;
}

/**
 * vip_g_crop() - g_crop handler
 * Note: VIP by itself does not provide any cropping capabilities,
 * so we rely entirely on a subdev capable of supporting cropping.
 * @file: file ptr
 * @priv: file handle
 * @crop: ptr to v4l2_crop
 */
static int vip_g_crop(struct file *file, void *priv,
			struct v4l2_crop *crop)
{
	int ret;
	struct vip *vip = video_drvdata(file);
	struct vip_subdev_info *sdinfo;

	v4l2_dbg(1, debug_level, &vip->v4l2_dev, "vip_g_crop\n");

	ret = mutex_lock_interruptible(&vip->lock);
	if (ret)
		return ret;

	sdinfo = vip->current_subdev;

	/*
	 * call subdev's g_crop function to get the current cropping window
	 * details
	 */
	ret = v4l2_device_call_until_err(&vip->v4l2_dev, sdinfo->grp_id,
					 video, g_crop, crop);
	if (ret < 0) {
		v4l2_err(&vip->v4l2_dev, "g_crop failed for sub device\n");
		goto unlock_out;
	}

unlock_out:
	mutex_unlock(&vip->lock);

	return ret;
}

/**
 * vip_s_crop() - s_crop handler
 * Note: VIP by itself does not provide any cropping capabilities,
 * so we rely entirely on a subdev capable of supporting cropping.
 * @file: file ptr
 * @priv: file handle
 * @crop: ptr to v4l2_crop
 */
static int vip_s_crop(struct file *file, void *priv,
			struct v4l2_crop *crop)
{
	int ret;
	struct vip *vip = video_drvdata(file);
	struct vip_subdev_info *sdinfo;

	v4l2_dbg(1, debug_level, &vip->v4l2_dev, "vip_s_crop\n");

	ret = mutex_lock_interruptible(&vip->lock);
	if (ret)
		return ret;

	sdinfo = vip->current_subdev;

	/* if streaming is already started, return device busy error */
	if (vip->is_streaming) {
		v4l2_err(&vip->v4l2_dev, "busy in streaming\n");
		ret = -EBUSY;
		goto unlock_out;
	}

	/* call subdev's s_crop function to set the current cropping window */
	ret = v4l2_device_call_until_err(&vip->v4l2_dev, sdinfo->grp_id,
					 video, s_crop, crop);
	if (ret < 0) {
		v4l2_err(&vip->v4l2_dev, "s_crop failed for sub device\n");
		goto unlock_out;
	}

	/*
	 * if cropping was successful at subdev end, we must update the
	 * values stored in vip instance to operate on correct values of
	 * width, height etc for future user-space calls.
	 * Note that VIP by itself has no CROPPING capability.
	 */
	vip->fmt.fmt.pix.width = crop->c.width;
	vip->fmt.fmt.pix.height = crop->c.height;
	vip->fmt.fmt.pix.bytesperline = vip->fmt.fmt.pix.width *
		(vip->vip_pdata->config->rgb_width / 8);
	vip->fmt.fmt.pix.sizeimage =
		vip->fmt.fmt.pix.bytesperline *
		vip->fmt.fmt.pix.height;

unlock_out:
	mutex_unlock(&vip->lock);

	return ret;
}

/* vip ioctl operations */
static const struct v4l2_ioctl_ops vip_ioctl_ops = {
	.vidioc_querycap = vip_querycap,
	.vidioc_enum_fmt_vid_cap = vip_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap = vip_g_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap = vip_s_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap = vip_try_fmt_vid_cap,
	.vidioc_enum_input = vip_enum_input,
	.vidioc_s_input	= vip_s_input,
	.vidioc_g_input	= vip_g_input,
	.vidioc_reqbufs = vip_reqbufs,
	.vidioc_querybuf = vip_querybuf,
	.vidioc_querystd = vip_querystd,
	.vidioc_s_std = vip_s_std,
	.vidioc_g_std = vip_g_std,
	.vidioc_qbuf = vip_qbuf,
	.vidioc_dqbuf = vip_dqbuf,
	.vidioc_streamon = vip_streamon,
	.vidioc_streamoff = vip_streamoff,
	.vidioc_cropcap = vip_cropcap,
	.vidioc_g_crop = vip_g_crop,
	.vidioc_s_crop = vip_s_crop,
	.vidioc_enum_dv_presets = vip_enum_dv_presets,
	.vidioc_s_dv_preset = vip_s_dv_preset,
	.vidioc_g_dv_preset = vip_g_dv_preset,
	.vidioc_query_dv_preset = vip_query_dv_preset,
	.vidioc_s_dv_timings = vip_s_dv_timings,
	.vidioc_g_dv_timings = vip_g_dv_timings,
	.vidioc_g_chip_ident = vip_g_chip_ident,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.vidioc_g_register = vip_dbg_g_register,
	.vidioc_s_register = vip_dbg_s_register,
#endif
	.vidioc_log_status = vip_log_status,
};

static void vip_config_fifo_ovf_intr(struct vip *vip, bool enable)
{
	int int_mask = readl(vip->base + VIP_INT_MASK);

	if (enable)
		/* enable fifo overflow interrupt */
		writel(int_mask & IR_UNMASK_FIFO_OVF_INT,
				vip->base + VIP_INT_MASK);
	else
		/* disable fifo overflow interrupt */
		writel(int_mask | IR_MASK_FIFO_OVF_INT,
				vip->base + VIP_INT_MASK);
}

static void vip_set_default_reg_val(struct vip *vip)
{
	u8 rgb_width = 0;
	u32 ctrl;

	switch (vip->vip_pdata->config->rgb_width) {
	case SIXTEEN_BIT:
		rgb_width = 0;
		break;
	case TWENTYFOUR_BIT:
		rgb_width = 1;
		break;
	case THIRTYTWO_BIT:
		rgb_width = 2;
		break;
	}

	/* program the control register bits */
	ctrl = vip->vip_pdata->config->vsync_pol << CTRL_VSYNC_POL_SHIFT |
		vip->vip_pdata->config->hsync_pol << CTRL_HSYNC_POL_SHIFT |
		rgb_width << CTRL_RGB_WIDTH_SHIFT |
		vip->vip_pdata->config->vdo_mode << CTRL_VDO_MODE_SHIFT |
		vip->vip_pdata->config->pix_clk_pol;

	writel(ctrl, vip->base + VIP_CONTROL);
}

static int vip_initialize(struct vip *vip)
{
	int ret = 0;

	/* set first input of current subdevice as the current input */
	vip->current_input = 0;

	/* set default standard */
	vip->std_index = 0;

	memcpy(&vip->std_info, &vip_standards[0], sizeof(struct vip_standard));
	/* configure the default format information */
	ret = vip_config_image_format(vip);
	if (ret)
		return ret;

	/* now set the vip registers as per the platfrom data */
	vip_set_default_reg_val(vip);

	/* enable fifo overflow interrupt */
	vip_config_fifo_ovf_intr(vip, ENABLE_FIFO_OVF_INTR);

	vip->initialized = 1;

	return ret;
}

/*
 * vip_open : It creates object of file handle structure and
 * stores it in private_data member of filepointer
 */
static int vip_open(struct file *file)
{
	struct vip *vip = video_drvdata(file);
	struct vip_fh *fh;
	int ret;

	v4l2_dbg(1, debug_level, &vip->v4l2_dev, "vip_open\n");

	if (!vip->vip_pdata->subdev_count) {
		v4l2_err(&vip->v4l2_dev,
				"no subdev registered with this host\n");
		return -ENODEV;
	}

	/* allocate memory for the file handle object */
	fh = kmalloc(sizeof(struct vip_fh), GFP_KERNEL);
	if (!fh) {
		v4l2_err(&vip->v4l2_dev,
			"unable to allocate memory for file handle object\n");
		return -ENOMEM;
	}

	/* store pointer to fh in private_data member of file */
	file->private_data = fh;
	fh->vip = vip;

	mutex_lock(&vip->lock);

	/* enable vip clock */
	ret = clk_enable(vip->clk);
	if (ret < 0)
		return ret;

	/* if vip is not initialized. initialize it */
	if (!vip->initialized) {
		if (vip_initialize(vip)) {
			mutex_unlock(&vip->lock);
			return -ENODEV;
		}
	}

	/* increment device usrs counter */
	vip->usr_count++;

	/* set io_allowed member to false */
	fh->io_allowed = 0;

	/* initialize priority of this instance to default priority */
	fh->prio = V4L2_PRIORITY_UNSET;
	v4l2_prio_open(&vip->prio, &fh->prio);

	mutex_unlock(&vip->lock);

	return 0;
}

/*
 * vip_release : This function deletes buffer queue, frees the
 * buffers and the vip file handle
 */
static int vip_release(struct file *file)
{
	int ret;
	struct vip *vip = video_drvdata(file);
	struct vip_fh *fh = file->private_data;
	struct vip_subdev_info *sdinfo;

	v4l2_dbg(1, debug_level, &vip->v4l2_dev, "vip_release\n");

	/* get the device lock */
	mutex_lock(&vip->lock);

	/* check if this instance is doing io and streaming */
	if (fh->io_allowed) {
		if (vip->is_streaming) {
			sdinfo = vip->current_subdev;
			ret = v4l2_device_call_until_err(&vip->v4l2_dev,
					sdinfo->grp_id,
					video, s_stream, 0);
			if (ret && (ret != -ENOIOCTLCMD))
				v4l2_err(&vip->v4l2_dev,
					"stream off failed in subdev\n");
			vip->is_streaming = 0;
			videobuf_streamoff(&vip->vq);
		}
		vip->io_usrs = 0;
	}

	/* disable fifo overflow interrupt */
	vip_config_fifo_ovf_intr(vip, DISABLE_FIFO_OVF_INTR);

	/* disable vip clock */
	clk_disable(vip->clk);

	/* decrement device usage counter */
	vip->usr_count--;

	/* close the priority */
	v4l2_prio_close(&vip->prio, fh->prio);

	/* handle last file handle case */
	if (!vip->usr_count)
		vip->initialized = 0;

	mutex_unlock(&vip->lock);
	file->private_data = NULL;

	/* free memory allocated to file handle object */
	kfree(fh);

	return 0;
}

/**
 * vip_mmap : used to map kernel space buffers
 * into user spaces
 */
static int vip_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct vip *vip = video_drvdata(file);

	v4l2_dbg(1, debug_level, &vip->v4l2_dev, "vip_mmap\n");

	/*
	 * FIXME: later modify this somehow to provide pointer to area
	 * allocated at boot-time.
	 */
	return videobuf_mmap_mapper(&vip->vq, vma);
}

/**
 * vip_poll: used for select/poll system call
 */
static unsigned int vip_poll(struct file *file, poll_table *wait)
{
	struct vip *vip = video_drvdata(file);

	v4l2_dbg(1, debug_level, &vip->v4l2_dev, "vip_poll\n");

	if (vip->is_streaming)
		return videobuf_poll_stream(file,
				&vip->vq, wait);
	return 0;
}

/* vip file operations */
static struct v4l2_file_operations vip_fops = {
	.owner = THIS_MODULE,
	.open = vip_open,
	.release = vip_release,
	.ioctl = video_ioctl2,
	.mmap = vip_mmap,
	.poll = vip_poll,
};

/* vip video template */
static struct video_device vip_video_template = {
	.name = "spear_vip",
	.fops = &vip_fops,
	.minor = -1,
	.ioctl_ops = &vip_ioctl_ops,
};

static void vip_schedule_next_buffer(struct vip *vip)
{
	unsigned long addr;

	vip->next_frm = list_entry(vip->dma_queue.next,
					struct videobuf_buffer, queue);
	list_del(&vip->next_frm->queue);
	vip->next_frm->state = VIDEOBUF_ACTIVE;
	addr = videobuf_to_dma_contig(vip->next_frm);

	/*
	 * set the base address of the next buffer in the VIP VDO
	 * register (if we were working on PING buffer this would be the
	 * address of the PONG buffer and vice-versa).
	 */
	writel(addr, vip->base + VIP_VDO_BASE);
}

static void vip_schedule_bottom_field(struct vip *vip)
{
	unsigned long addr;

	addr = videobuf_to_dma_contig(vip->cur_frm);
	addr += vip->field_off;

	/*
	 * set the base address of the bottom field in the VIP VDO
	 * register
	 */
	writel(addr, vip->base + VIP_VDO_BASE);
}

static void vip_process_buffer_complete(struct vip *vip)
{
	struct timeval timevalue;

	/* mark timestamp for this buffer and mark it as DONE */
	do_gettimeofday(&timevalue);
	vip->cur_frm->ts = timevalue;
	vip->cur_frm->state = VIDEOBUF_DONE;
	vip->cur_frm->size = vip->fmt.fmt.pix.sizeimage;

	/* wake-up any process interested in using this buffer */
	wake_up_interruptible(&vip->cur_frm->done);

	vip->cur_frm = vip->next_frm;
}

/* fifo overflow isr */
static irqreturn_t vip_fifo_ovf_isr(int irq, void *dev_id)
{
	struct vip *vip = dev_id;

	/*
	 * FiFo Overflow condition:
	 * Will loose one frame. But SW has no control over it.
	 * The HW should take care to give us a new clean frame.
	 * But the occurence of this interrupt is a good indication that
	 * the system bandwidth is choked now.
	 */
	v4l2_warn(&vip->v4l2_dev, "vip_fifo_ovf, frame will be lost\n");

	/* clear the interrupt source */
	writel(CLEAR_FIFO_OVF_IRQ, vip->base + VIP_INT_STATUS);

	return IRQ_HANDLED;
}

/*
 * frame/field end isr:
 * The frame/field end interrupt is asserted at the end of each frame
 * (for progressive mode) and at the end of each field
 * (for interlaced/interleaved mode).
 *
 * Accordingly there are two possible approaches here:
 *	- Progressive mode: As we have two HD frames allocated from the system
 *	  memory (called PING-PONG buffers), here the interrupt handling is
 *	  simpler as we program the base address of the next buffer in
 *	  the VIP VDO register while marking the current buffer as DONE.
 *
 *	- Interlaced/Interleaved mode: Here also as we have two HD frames
 *	  allocated from the system memory (called PING-PONG buffers).
 *	  The interrupt handling is a bit trickier, as we do not receive
 *	  any info from the VIP hardware regarding whether the received
 *	  field is EVEN or ODD. In such a case, we workaround by using
 *	  the feature of certain smart chips like the SiL9135a HDMI
 *	  receiver chip which provide a EVN/ODD output signal which can
 *	  be connected to the VIC/GIC, so as to interrupt the CPU at the end
 *	  of each field. The driver maintains a notion of TOP(ODD) and
 *	  BOTTOM(EVEN) field using a SW counter which is incremented on
 *	  every occurence of this interrupt.
 */
static irqreturn_t vip_frame_end_isr(int irq, void *dev_id)
{
	struct vip *vip = dev_id;
	enum v4l2_field field;

	v4l2_dbg(1, debug_level, &vip->v4l2_dev, "vip_frame_end_isr\n");

	field = vip->fmt.fmt.pix.field;

	/* if streaming is not started, don't do anything */
	if (!vip->is_streaming)
		goto clear_intr;

	/* we need to handle both progressive and interlaced frames */
	if (field == V4L2_FIELD_NONE) {
		/* handle progressive frame capture */
		v4l2_dbg(1, debug_level, &vip->v4l2_dev,
			"frame format is progressive\n");

		/* check if dma queue is empty */
		if (list_empty(&vip->dma_queue))
			goto clear_intr;

		if (vip->cur_frm != vip->next_frm)
			/* mark current frame as done */
			vip_process_buffer_complete(vip);

		/* schedule the next buffer present in the buffer queue */
		vip_schedule_next_buffer(vip);

		goto clear_intr;
	} else {
		/* handle interlaced/sequential-TB frame capture */

		/*
		 * note that the VIP provides no information regarding
		 * the field_id of the received field. It is upto the
		 * driver to somehow manage the notion of field ids. We
		 * assume here that the first field end interrupt will
		 * be for the end of the TOP(ODD) field whereas the
		 * second interrupt will be for the end of the BOTTOM(EVEN)
		 * field and so on..
		 */
		if (vip->isr_count % 2 == 0)
			vip->field_id = VIP_TOP_FIELD_END;
		else
			vip->field_id = VIP_BOTTOM_FIELD_END;

		vip->isr_count++;

		v4l2_dbg(1, debug_level, &vip->v4l2_dev,
			"frame format is interlaced/sequential-BT, "
			"sw field_id=%x\n", vip->field_id);

		if (vip->field_id == VIP_BOTTOM_FIELD_END) {
			/*
			 * end of bottom/even field
			 */

			/* check if dma queue is empty */
			if (list_empty(&vip->dma_queue))
				goto clear_intr;

			if (vip->cur_frm != vip->next_frm)
				/* mark current frame as done */
				vip_process_buffer_complete(vip);

			/*
			 * schedule the next buffer present in
			 * the buffer queue
			 */
			vip_schedule_next_buffer(vip);

			goto clear_intr;
		} else if (vip->field_id == VIP_TOP_FIELD_END) {
			/*
			 * end of top/odd field
			 */

			/* check if dma queue is empty */
			if (list_empty(&vip->dma_queue) ||
					vip->cur_frm != vip->next_frm)
				goto clear_intr;

			/*
			 * based on whether the two fields are stored
			 * interleavely or separately in memory, reconfigure
			 * the VIP memory address to store the bottom
			 * field
			 */
			if (field == V4L2_FIELD_SEQ_TB)
				vip_schedule_bottom_field(vip);

			goto clear_intr;
		}
	}

clear_intr:
	return IRQ_HANDLED;
}

/**
 * vip_probe() - driver probe handler
 * @pdev: ptr to platform device structure
 */
static int __devinit vip_probe(struct platform_device *pdev)
{
	int i, j, ret, subdev_count = 0;
	void __iomem *addr;
	struct clk *clk;
	struct vip *vip;
	struct resource *mem, *fifo_ovf_irq;
	struct vip_subdev_info *sdinfo;
	struct vip_plat_data *vip_pdata;
	struct video_device *vfd;
	struct v4l2_input *inps;
	struct i2c_adapter *i2c_adap;

	/* must have platform data */
	if (!pdev || !pdev->dev.platform_data) {
		v4l2_err(pdev->dev.driver, "missing platform data\n");
		ret = -EINVAL;
		goto exit;
	}

	/* get the platform data */
	vip_pdata = pdev->dev.platform_data;
	if (!vip_pdata->subdev_info) {
		v4l2_err(pdev->dev.driver,
				"subdev_info is null in vip_pdata\n");
		ret = -ENOENT;
		goto exit;
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	fifo_ovf_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!mem || (fifo_ovf_irq <= 0)) {
		ret = -ENODEV;
		goto exit;
	}

	/* allocate vip resource */
	vip = kzalloc(sizeof(struct vip), GFP_KERNEL);
	if (!vip) {
		v4l2_err(pdev->dev.driver,
				"not enough memory for vip device\n");
		ret = -ENOMEM;
		goto exit;
	}

	if (!request_mem_region(mem->start, resource_size(mem),
				KBUILD_MODNAME)) {
		v4l2_err(pdev->dev.driver, "resource unavailable\n");
		ret = -ENODEV;
		goto exit_free_vip;
	}

	addr = ioremap(mem->start, resource_size(mem));
	if (!addr) {
		v4l2_err(pdev->dev.driver, "failed to map vip port\n");
		ret = -ENOMEM;
		goto exit_release_mem;
	}

	vip->pdev = &pdev->dev;
	vip->vip_pdata = vip_pdata;

	vip->base = addr;
	clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(clk)) {
		v4l2_err(pdev->dev.driver, "no clock defined\n");
		ret = -ENODEV;
		goto exit_iounmap;
	}

	vip->clk = clk;
	vip->fifo_irq = fifo_ovf_irq->start;

	ret = request_irq(vip->fifo_irq, vip_fifo_ovf_isr, 0,
				"vip_fifo_overflow", vip);
	if (ret) {
		v4l2_err(pdev->dev.driver,
				"failed to allocate IRQ %d\n", vip->fifo_irq);
		goto exit_clk_put;
	}

	/*
	 * if a gpio pin is used to signify a field-end interrupt to the CPU,
	 * get the gpio and register an isr on it.
	 */
	if (vip_pdata->is_field_end_gpio_based) {
		ret = gpio_request(vip->vip_pdata->gpio_for_frame_end_intr,
				"spear_vip frame end interrupt");
		if (ret < 0) {
			v4l2_err(pdev->dev.driver,
				"unable to request frame end gpio %d\n",
				vip->vip_pdata->gpio_for_frame_end_intr);
			goto exit_free_fifo_irq;
		}

		ret = gpio_direction_input(
				vip->vip_pdata->gpio_for_frame_end_intr);
		if (ret < 0) {
			v4l2_err(pdev->dev.driver,
				"could not set gpio in input direction\n");
			goto exit_gpio_unreq;
		}

		ret = request_irq(gpio_to_irq(
				vip->vip_pdata->gpio_for_frame_end_intr),
				vip_frame_end_isr, 0, "vip_frame_end",
				vip);
		if (ret) {
			v4l2_err(pdev->dev.driver,
				"failed to allocate IRQ %d\n",
				gpio_to_irq(vip->vip_pdata->
					gpio_for_frame_end_intr));
			goto exit_gpio_unreq;
		}
	}

	ret = v4l2_device_register(&pdev->dev, &vip->v4l2_dev);
	if (ret) {
		v4l2_err(pdev->dev.driver,
			"Unable to register v4l2 device.\n");
		goto exit_free_gpio_irq;
	}

	vfd = video_device_alloc();
	if (!vfd) {
		ret = -ENOMEM;
		v4l2_err(pdev->dev.driver, "Unable to alloc video device\n");
		goto exit_unreg_v4l2_dev;
	}

	/* initialize fields of video device */
	*vfd = vip_video_template;
	vfd->release = video_device_release;
	vfd->v4l2_dev = &vip->v4l2_dev;
	snprintf(vfd->name, sizeof(vfd->name), "spear_vip:%d.%d",
		 VIP_MAJOR_RELEASE, VIP_MINOR_RELEASE);

	/* set vfd to the video device */
	vip->vfd = vfd;
	video_set_drvdata(vip->vfd, vip);

	/* register video device */
	ret = video_register_device(vip->vfd, VFL_TYPE_GRABBER, -1);
	if (ret) {
		video_device_release(vip->vfd);
		v4l2_err(pdev->dev.driver, "failed to register video device\n");
		goto exit_unreg_v4l2_dev;
	}

	/* init locks */
	spin_lock_init(&vip->irqlock);
	spin_lock_init(&vip->dma_queue_lock);
	mutex_init(&vip->lock);

	/* initialize prio */
	v4l2_prio_init(&vip->prio);

	v4l2_info(pdev->dev.driver,
		"spear_vip driver registered as /dev/video%d "
		"(base addr=%p, FiFO IRQ=%d)\n",
		vfd->num, vip->base, vip->fifo_irq);

	if (vip_pdata->is_field_end_gpio_based)
		v4l2_info(pdev->dev.driver,
				"spear_vip driver (Frame-End GPIO=%d)\n",
				vip->vip_pdata->gpio_for_frame_end_intr);

	/* register subdevices attached with this host driver */
	i2c_adap = i2c_get_adapter(vip_pdata->i2c_adapter_id);
	subdev_count = vip_pdata->subdev_count;
	vip->sd = kzalloc(sizeof(struct v4l2_subdev *) * subdev_count,
				GFP_KERNEL);
	if (!vip->sd) {
		v4l2_err(&vip->v4l2_dev,
			"unable to allocate memory for subdevice pointers\n");
		ret = -ENOMEM;
		goto exit_unreg_video_dev;
	}

	for (i = 0; i < subdev_count; i++) {
		sdinfo = &vip_pdata->subdev_info[i];

		/* load up the subdevice */
		vip->sd[i] =
			v4l2_i2c_new_subdev_board(&vip->v4l2_dev,
						i2c_adap,
						&sdinfo->board_info,
						NULL);
		if (vip->sd[i]) {
			v4l2_info(&vip->v4l2_dev,
				"v4l2 sub device %s registered\n",
				sdinfo->name);
			vip->sd[i]->grp_id = sdinfo->grp_id;
			/* update tvnorms from the sub devices */
			for (j = 0; j < sdinfo->num_inputs; j++) {
				inps = &sdinfo->inputs[j];
				vfd->tvnorms |= inps->std;
			}
		} else {
			v4l2_info(&vip->v4l2_dev,
				"v4l2 sub device %s registeration fails\n",
				sdinfo->name);
			ret = -ENODEV;
			goto exit_free_subdev;
		}
	}

	/* set first sub device as current one */
	vip->current_subdev = &vip_pdata->subdev_info[0];

	/* disable fifo overflow interrupt */
	vip_config_fifo_ovf_intr(vip, DISABLE_FIFO_OVF_INTR);

	/* clear streaming flag */
	vip->is_streaming = 0;

	/*
	 * get the pointer for the contiguous memory pointer allocated
	 * at boot-time and declare the region used to store the VIP
	 * frames as contiguous with appropriate DMA mappings
	 */
	ret = dma_declare_coherent_memory(&pdev->dev, vip_pdata->vb_base,
			vip_pdata->vb_base, vip_pdata->vb_size,
			DMA_MEMORY_MAP | DMA_MEMORY_EXCLUSIVE);
	if (!ret) {
		dev_err(&pdev->dev, "Unable to declare MMAP memory.\n");
		ret = -ENOENT;
		goto exit_free_subdev;
	}

	return 0;

exit_free_subdev:
	kfree(vip->sd);
exit_unreg_video_dev:
	video_unregister_device(vip->vfd);
exit_unreg_v4l2_dev:
	v4l2_device_unregister(&vip->v4l2_dev);
exit_free_gpio_irq:
	if (vip_pdata->is_field_end_gpio_based)
		free_irq(gpio_to_irq(vip->vip_pdata->gpio_for_frame_end_intr),
				vip);
exit_gpio_unreq:
	if (vip_pdata->is_field_end_gpio_based)
		gpio_free(vip->vip_pdata->gpio_for_frame_end_intr);
exit_free_fifo_irq:
	free_irq(vip->fifo_irq, vip);
exit_clk_put:
	clk_put(clk);
exit_iounmap:
	iounmap(addr);
exit_release_mem:
	release_mem_region(mem->start, resource_size(mem));
exit_free_vip:
	kfree(vip);
exit:
	dev_err(&pdev->dev, "probe failed with status %d\n", ret);

	return ret;
}

/**
 * vip_remove() - driver remove handler
 * @pdev: ptr to platform device structure
 */
static int __devexit vip_remove(struct platform_device *pdev)
{
	struct vip *vip = platform_get_drvdata(pdev);
	struct resource *mem;

	kfree(vip->sd);
	video_unregister_device(vip->vfd);
	v4l2_device_unregister(&vip->v4l2_dev);

	if (vip->vip_pdata->is_field_end_gpio_based) {
		free_irq(gpio_to_irq(vip->vip_pdata->gpio_for_frame_end_intr),
				vip);
		gpio_free(vip->vip_pdata->gpio_for_frame_end_intr);
	}

	free_irq(vip->fifo_irq, vip);

	clk_put(vip->clk);

	iounmap(vip->base);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(mem->start, resource_size(mem));

	kfree(vip);

	v4l2_info(pdev->dev.driver, "spear_vip driver removed\n");

	return 0;
}

static struct platform_driver vip_driver = {
	.probe = vip_probe,
	.remove = __devexit_p(vip_remove),
	.driver = {
		.name = "spear_vip",
		.owner = THIS_MODULE,
	},
};

static int __init vip_init(void)
{
	return platform_driver_register(&vip_driver);
}
module_init(vip_init);

static void __exit vip_exit(void)
{
	platform_driver_unregister(&vip_driver);
}
module_exit(vip_exit);

MODULE_AUTHOR("Bhupesh Sharma <bhupesh.sharma@st.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:spear_vip");
MODULE_DESCRIPTION("V4L2 Driver for SPEAr VIP");
