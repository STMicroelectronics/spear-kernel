/*
 * arch/arm/mach-spear13xx/spear1340_plug_boards.c
 *
 * SPEAr1340 plug boards source file
 *
 * Copyright (C) 2011 ST Microelectronics
 * Viresh Kumar <viresh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
#define pr_fmt(fmt) "spear1340_pb: " fmt

/*
 * Plug boards allow the different standard test and debug procedures on the
 * same physical interface without the need to develop a specific board and use
 * a different manufacturer’s chips. They are connected to the main board
 * through small high speed shielded connectors to avoid quality degradation of
 * the signals. Each plug has the interface connectors on different positions to
 * prevent any insertions errors.
 *
 * Following are the plug boards available:
 * - gmii: gmii interface present for ethernet
 * - rgmii: rgmii interface present for ethernet
 * - etm: etm trace module
 * - hdmi_rx: hdmi receiver
 * - hdmi_tx: hdmi transmitter
 * - cam0: camera sensor connected to camera device 0 of SoC
 * - vga:
 * - sata: It is not a separate physical plug board but a board
 *   configuration
 * - pcie: It is not a separate physical plug board but a board
 *   configuration. sata and pcie are muxed and cannot be used together.
 *
 * Plug boards details can be found at:
 * https://codex.cro.st.com/plugins/docman/?group_id=1309&action=show&id=164214
 *
 * The philosophy picked for maintaining them in software is as follows. Firstly
 * device arrays will be prepared in 1340_evb file and these will be passed to
 * plug board init routine, if plug boards are passed from bootargs. This
 * routine may:
 * - override the padmux settings done earlier by evb board
 * - skip registeration of some devices passed from evb_init()
 * - add new amba/plat devs, etc devices
 * - If multiple boards are passed from bootargs, then boards will be
 *   initialized in the order they are mentioned in bootargs.
 *
 * Passing param from bootargs
 * ---------------------------
 * "pb=" must be used to pass plug boards request from bootargs. This variable
 * can contain string values mentioned above in board descriptions"
 *
 * More than one board can be requested by passing ',' separated board list, eg:
 * bootargs: console=ttyAMA0,115200 pb=rgmii,hdmi_tx,cam0
 */

#include <linux/ad9889b.h>
#include <linux/bug.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/designware_i2s.h>
#include <linux/list.h>
#include <linux/phy.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/stmmac.h>
#include <media/soc_camera.h>
#include <media/vip.h>
#include <mach/db9000fb_info.h>
#include <mach/generic.h>
#include <mach/hardware.h>
#include <mach/plug_board.h>
#include <mach/spear1340_misc_regs.h>
#include <mach/spear_pcie.h>

/* name of an individual plug-board is limited to 10 chars */
#define PB_NAME_LIMIT	10

/* max plug-boards which can be requested via bootargs is limited to 50 */
#define MAX_REQ_PB	50

#define DEBUG 1
#define INIT_PB(pb_name, pb)						\
	do {								\
		if (strlen(#pb_name) > PB_NAME_LIMIT) {			\
			pr_err("Error: name choosen for plug board is "	\
				"more than 10 chars, use a smaller "	\
				"name instead\n");			\
			continue;					\
		}							\
									\
		pb = kmalloc(sizeof(struct plug_board), GFP_KERNEL);	\
		if (!pb) {						\
			pr_err("Error allocating memory for pb: %s\n",	\
				#pb_name);				\
			continue;					\
		}							\
									\
		strcpy(pb->name, #pb_name);				\
		pb->pmx_devs = (struct pmx_dev **) pb_name##_pb_pmx_devs;		\
		pb->rm_adevs = (struct amba_device **) pb_name##_pb_rm_adevs;		\
		pb->add_adevs = (struct amba_device **) pb_name##_pb_add_adevs;		\
		pb->rm_pdevs = (struct platform_device **) pb_name##_pb_rm_pdevs;	\
		pb->add_pdevs = (struct platform_device **) pb_name##_pb_add_pdevs;	\
		pb->rm_spi_devs = (struct spi_board_info **) pb_name##_pb_rm_spi_devs;	\
		pb->add_spi_devs = (struct spi_board_info **) pb_name##_pb_add_spi_devs;\
		pb->rm_i2c_devs = (struct i2c_dev_info **) pb_name##_pb_rm_i2c_devs;	\
		pb->add_i2c_devs = (struct i2c_dev_info **) pb_name##_pb_add_i2c_devs;	\
		pb->pmx_cnt = ARRAY_SIZE(pb_name##_pb_pmx_devs);	\
		pb->rm_acnt = ARRAY_SIZE(pb_name##_pb_rm_adevs);	\
		pb->add_acnt = ARRAY_SIZE(pb_name##_pb_add_adevs);	\
		pb->rm_pcnt = ARRAY_SIZE(pb_name##_pb_rm_pdevs);	\
		pb->add_pcnt = ARRAY_SIZE(pb_name##_pb_add_pdevs);	\
		pb->rm_spi_cnt = ARRAY_SIZE(pb_name##_pb_rm_spi_devs);	\
		pb->add_spi_cnt = ARRAY_SIZE(pb_name##_pb_add_spi_devs); \
		pb->rm_i2c_cnt = ARRAY_SIZE(pb_name##_pb_rm_i2c_devs);	\
		pb->add_i2c_cnt = ARRAY_SIZE(pb_name##_pb_add_i2c_devs); \
		pb->pb_init = pb_name##_pb_init;			\
									\
		pr_info("Adding plug board: %s\n", #pb_name);		\
	} while (0)

/*
 * FIXME: Update this later when the HDMI receiver chip driver is available:
 *
 * 1. Here, we assume that the SIL9135A HDMI receiver chip supports
 *    two inputs and a single output. The names of these macros can be
 *    updated later.
 * 2. Assume that SIL HDMI Rx chip supports all DV standards.
 *    Also note that analog standards like PAL and NTSC is also supported by
 *    VIP IP. So the STD supported by VIP driver will be a super-set
 *    of these DV and analog standards.
 */
#define SIL9135A_INPUT_1	1
#define SIL9135A_INPUT_2	2
#define SIL9135A_OUTPUT		3
#define SIL9135A_I2C_ADDR	0x18
#define SIL9135A_STD_ALL	(V4L2_DV_480P59_94 | V4L2_DV_576P50 |	\
				V4L2_DV_720P24 | V4L2_DV_720P25 |	\
				V4L2_DV_720P30 | V4L2_DV_720P50 |	\
				V4L2_DV_720P59_94 | V4L2_DV_720P60 |	\
				V4L2_DV_1080I29_97 | V4L2_DV_1080I30 |	\
				V4L2_DV_1080I25 | V4L2_DV_1080I50 |	\
				V4L2_DV_1080I60 | V4L2_DV_1080P24 |	\
				V4L2_DV_1080P25 | V4L2_DV_1080P30 |	\
				V4L2_DV_1080P50 | V4L2_DV_1080P60)

struct plug_board {
	struct pmx_dev **pmx_devs;
	struct amba_device **rm_adevs;
	struct amba_device **add_adevs;
	struct platform_device **rm_pdevs;
	struct platform_device **add_pdevs;
	struct spi_board_info **rm_spi_devs;
	struct spi_board_info **add_spi_devs;
	struct i2c_dev_info **rm_i2c_devs;
	struct i2c_dev_info **add_i2c_devs;
	u32 pmx_cnt;
	u32 rm_acnt;
	u32 add_acnt;
	u32 rm_pcnt;
	u32 add_pcnt;
	u32 rm_spi_cnt;
	u32 add_spi_cnt;
	u32 rm_i2c_cnt;
	u32 add_i2c_cnt;
	void (*pb_init)(void);

	struct list_head node;
	char name[PB_NAME_LIMIT];
};

enum skip_device_type {
	SKIP_AMBA_DEVICE,
	SKIP_PLAT_DEVICE,
	SKIP_SPI_DEVICE,
	SKIP_I2C_DEVICE
};

/* string specifying which plug boards are requested */
char spear1340_plug_board[MAX_REQ_PB] = {'\0', };
static char pb_empty_array[] __initdata = {};


/* Definitions specific to GMII plug board */
#define gmii_pb_rm_adevs		pb_empty_array
#define gmii_pb_rm_pdevs		pb_empty_array
#define gmii_pb_add_adevs		pb_empty_array
#define gmii_pb_add_pdevs		pb_empty_array
#define gmii_pb_rm_spi_devs		pb_empty_array
#define gmii_pb_add_spi_devs		pb_empty_array
#define gmii_pb_rm_i2c_devs		pb_empty_array
#define gmii_pb_add_i2c_devs		pb_empty_array

/* padmux devices to enable */
static struct pmx_dev *gmii_pb_pmx_devs[] = {
	&spear1340_pmx_gmii,
};

static void __init gmii_pb_init(void)
{
	/* Enable GMII */
	struct plat_stmmacphy_data *phy_data;

	phy_data = spear1340_phy0_device.dev.platform_data;
	phy_data->interface = PHY_INTERFACE_MODE_GMII;
}


/* Definitions specific to RGMII plug board */
#define rgmii_pb_rm_adevs		pb_empty_array
#define rgmii_pb_rm_pdevs		pb_empty_array
#define rgmii_pb_add_adevs		pb_empty_array
#define rgmii_pb_add_pdevs		pb_empty_array
#define rgmii_pb_rm_spi_devs		pb_empty_array
#define rgmii_pb_add_spi_devs		pb_empty_array
#define rgmii_pb_rm_i2c_devs		pb_empty_array
#define rgmii_pb_add_i2c_devs		pb_empty_array

/* padmux devices to enable */
static struct pmx_dev *rgmii_pb_pmx_devs[] = {
	&spear1340_pmx_rgmii,
};

static void __init rgmii_pb_init(void)
{
	/* Enable RGMII */
	struct plat_stmmacphy_data *phy_data;

	phy_data = spear1340_phy0_device.dev.platform_data;
	phy_data->interface = PHY_INTERFACE_MODE_RGMII;
}


/* Definitions specific to ETM plug board */
#define etm_pb_pmx_devs			pb_empty_array
#define etm_pb_rm_adevs			pb_empty_array
#define etm_pb_rm_pdevs			pb_empty_array
#define etm_pb_add_adevs		pb_empty_array
#define etm_pb_add_pdevs		pb_empty_array
#define etm_pb_rm_spi_devs		pb_empty_array
#define etm_pb_add_spi_devs		pb_empty_array
#define etm_pb_rm_i2c_devs		pb_empty_array
#define etm_pb_add_i2c_devs		pb_empty_array
#define etm_pb_init			NULL


/* Definitions specific to HDMI RX plug board */
#define hdmi_rx_pb_rm_adevs		pb_empty_array
#define hdmi_rx_pb_rm_pdevs		pb_empty_array
#define hdmi_rx_pb_add_adevs		pb_empty_array
#define hdmi_rx_pb_rm_spi_devs		pb_empty_array
#define hdmi_rx_pb_add_spi_devs		pb_empty_array
#define hdmi_rx_pb_rm_i2c_devs		pb_empty_array
#define hdmi_rx_pb_add_i2c_devs		pb_empty_array

/* sil9135a hdmi rx chip related */

/*
 * inputs available at the SIL9135A HDMI receiver chip
 * FIXME: Appropriate names should be added for HDMI receiver inputs
 */
static struct v4l2_input sil9135a_inputs[] = {
	{
		.index = 0,
		.name = "1st Input",
		.type = V4L2_INPUT_TYPE_CAMERA,
		.std = SIL9135A_STD_ALL,
	}, {
		.index = 1,
		.name = "2nd Input",
		.type = V4L2_INPUT_TYPE_CAMERA,
		.std = SIL9135A_STD_ALL,
	},
};

/*
 * this is the route info for connecting each input of the SIL9135A
 * hdmi receiver to its output which eventually goes to vip.
 * There is a one to one correspondence with sil9135a_inputs.
 */
static struct vip_subdev_route sil9135a_routes[] = {
	{
		.input = SIL9135A_INPUT_1,
		.output = SIL9135A_OUTPUT,
	}, {
		.input = SIL9135A_INPUT_2,
		.output = SIL9135A_OUTPUT,
	},
};

/* info regarding the various subdevs connected to VIP */
static struct vip_subdev_info vip_sdev_info[] = {
	/* SIL9135A hdmi receiver */
	{
		.name = "sil9135a",
		.grp_id = 0,
		.num_inputs = ARRAY_SIZE(sil9135a_inputs),
		.inputs = sil9135a_inputs,
		.routes = sil9135a_routes,
		.can_route = 1,
		.board_info = {
			I2C_BOARD_INFO("sil9135a", SIL9135A_I2C_ADDR),
			/*
			 * TODO: we can add some platform specific
			 * data for HDMI receiver chip here, if needed.
			 */
		},
	},
};

/*
 * some of the VIP features cannot be programmed via standard V4L2
 * ioctls, so we configure them here.
 */
static struct vip_config vip_config_info = {
	.vsync_pol = POL_ACTIVE_LOW,
	.hsync_pol = POL_ACTIVE_LOW,
	.rgb_width = SIXTEEN_BIT,
	.vdo_mode = SINGLE_PORT,
	.pix_clk_pol = POL_ACTIVE_LOW,
};

/*
 * a lot of VIP subdev specific params can change with a change in the
 * EVB being used, so we need to be careful while populating these
 * details.
 */
static struct vip_plat_data vip_board_specific_data = {
	.card_name = "spear_vip",
	.config = &vip_config_info,
	.subdev_info = vip_sdev_info,
	.subdev_count = ARRAY_SIZE(vip_sdev_info),
	.i2c_adapter_id = 0,
	.is_field_end_gpio_based = 1,
	.gpio_for_frame_end_intr = PLGPIO_100, /* I2S_OUT_DATA_3 */
};

/* padmux devices to enable */
static struct pmx_dev *hdmi_rx_pb_pmx_devs[] = {
	&spear1340_pmx_vip_mux_cam0,
	&spear1340_pmx_vip_mux_cam1,
	&spear1340_pmx_vip_mux_cam2,
	&spear1340_pmx_vip_mux_cam3,
};

static struct platform_device *hdmi_rx_pb_add_pdevs[] __initdata = {
	&spear1340_vip_device,
};

static void __init hdmi_rx_pb_init(void)
{
	vip_set_vb_base(&vip_board_specific_data);

	/* set vip plat data */
	vip_set_plat_data(&spear1340_vip_device,
				&vip_board_specific_data);
}


/* Definitions specific to HDMI TX plug board */
#define hdmi_tx_pb_pmx_devs		pb_empty_array
#define hdmi_tx_pb_rm_adevs		pb_empty_array
#define hdmi_tx_pb_rm_pdevs		pb_empty_array
#define hdmi_tx_pb_add_adevs		pb_empty_array
#define hdmi_tx_pb_add_pdevs		pb_empty_array
#define hdmi_tx_pb_rm_spi_devs		pb_empty_array
#define hdmi_tx_pb_add_spi_devs		pb_empty_array
#define hdmi_tx_pb_rm_i2c_devs		pb_empty_array

static struct ad9889b_pdata ad9889b_platdata = {
	.irq_gpio = STMPE801_GPIO_7,
	.irq_type = IRQF_DISABLED | IRQF_SHARED | IRQF_TRIGGER_FALLING,
	.fb = 0,
	.ain = HDMI_AUDIO_IN_SPDIF,
};

static struct i2c_board_info spear1340_pb_i2c_board_hdmi_tx = {
		.type = "adi9889_i2c",
		.addr = 0x39,
		.platform_data = &ad9889b_platdata,
};

static struct i2c_dev_info spear1340_pb_i2c_hdmi_tx = {
	.board = &spear1340_pb_i2c_board_hdmi_tx,
	.busnum = 0,
};

/* I2C devices to be added */
static struct i2c_dev_info *hdmi_tx_pb_add_i2c_devs[] __initdata = {
	&spear1340_pb_i2c_hdmi_tx,
};

static void __init hdmi_tx_pb_init(void)
{
	struct clk *i2s_sclk_clk;
	struct i2s_platform_data *pdata
		= dev_get_platdata(&spear1340_i2s_play_device.dev);

	i2s_sclk_clk = clk_get_sys(NULL, "i2s_sclk_clk");
	if (IS_ERR(i2s_sclk_clk))
		pr_err("%s:couldn't get i2s_sclk_clk\n", __func__);

	if (clk_set_rate(i2s_sclk_clk, 3070000)) {
		pr_err("%s:couldn't set i2s_sclk_clk rate\n", __func__);
		clk_put(i2s_sclk_clk);
	}

	if (clk_enable(i2s_sclk_clk)) {
		pr_err("%s:enabling i2s_sclk_clk\n", __func__);
		clk_put(i2s_sclk_clk);
	}

	pdata->swidth = 32;
}


/* Definitions specific to CAM plug board with single sensor mounted */
#define cam0_pb_rm_adevs		pb_empty_array
#define cam0_pb_rm_pdevs		pb_empty_array
#define cam0_pb_add_adevs		pb_empty_array
#define cam0_pb_rm_spi_devs		pb_empty_array
#define cam0_pb_add_spi_devs		pb_empty_array
#define cam0_pb_rm_i2c_devs		pb_empty_array
#define cam0_pb_add_i2c_devs		pb_empty_array
#define cam0_pb_init			NULL

/* padmux devices to enable */
static struct pmx_dev *cam0_pb_pmx_devs[] = {
	&spear1340_pmx_cam0,
};

/* camera sensor registeration */
static struct i2c_board_info vs6725_camera_sensor_info[] = {
	{
		I2C_BOARD_INFO("vs6725", 0x10),
	},
};

static struct soc_camera_link vs6725_cam0_sensor_iclink = {
	.bus_id = 0,	/* sensor is connected to camera device 0 */
	.i2c_adapter_id = 0, /* sensor is connected to i2c controller 0 */
	.board_info = &vs6725_camera_sensor_info[0],
	.module_name = "vs6725",
};

static struct platform_device spear1340_cam0_sensor_device = {
	.name = "soc-camera-pdrv",
	.id = -1,
	.dev = {
		.platform_data = &vs6725_cam0_sensor_iclink,
	},
};

static struct platform_device *cam0_pb_add_pdevs[] __initdata = {
	&spear1340_camif0_device,
	&spear1340_cam0_sensor_device,
};


/* Definitions specific to VGA plug board */
#define vga_pb_pmx_devs			pb_empty_array
#define vga_pb_rm_adevs			pb_empty_array
#define vga_pb_rm_pdevs			pb_empty_array
#define vga_pb_add_adevs		pb_empty_array
#define vga_pb_add_pdevs		pb_empty_array
#define vga_pb_rm_spi_devs		pb_empty_array
#define vga_pb_add_spi_devs		pb_empty_array
#define vga_pb_rm_i2c_devs		pb_empty_array
#define vga_pb_add_i2c_devs		pb_empty_array
#define vga_pb_init			NULL

/*
 * Definitions specific to SATA configuration
 * This is an exception as SATA is not a separate plug board but is a
 * change in normal evaulation board for supporting SATA.
 */
#define sata_pb_rm_adevs		pb_empty_array
#define sata_pb_add_adevs		pb_empty_array
#define sata_pb_rm_spi_devs		pb_empty_array
#define sata_pb_add_spi_devs		pb_empty_array
#define sata_pb_rm_i2c_devs		pb_empty_array
#define sata_pb_add_i2c_devs		pb_empty_array

/* padmux devices to enable */
static struct pmx_dev *sata_pb_pmx_devs[] = {
	&spear1340_pmx_sata,
};

static struct platform_device *sata_pb_rm_pdevs[] __initdata = {
	&spear13xx_pcie_host0_device,
};

static struct platform_device *sata_pb_add_pdevs[] __initdata = {
	&spear1340_sata0_device,
};

static void __init sata_pb_init(void)
{
	/* Miphy configuration for SATA */
	writel(SPEAR1340_PCIE_SATA_MIPHY_CFG_SATA_25M_CRYSTAL_CLK,
			VA_SPEAR1340_PCIE_MIPHY_CFG);
}

/*
 * Definitions specific to PCIe configuration
 * This is an exception as PCIe is not a separate plug board but is a
 * change in normal evaulation board for supporting PCIe.
 */
#define pcie_pb_rm_adevs		pb_empty_array
#define pcie_pb_add_adevs		pb_empty_array
#define pcie_pb_rm_spi_devs		pb_empty_array
#define pcie_pb_add_spi_devs		pb_empty_array
#define pcie_pb_rm_i2c_devs		pb_empty_array
#define pcie_pb_add_i2c_devs		pb_empty_array

/* padmux devices to enable */
static struct pmx_dev *pcie_pb_pmx_devs[] = {
	&spear1340_pmx_pcie,
};

static struct platform_device *pcie_pb_rm_pdevs[] __initdata = {
	&spear1340_sata0_device,
};

static struct platform_device *pcie_pb_add_pdevs[] __initdata = {
	&spear13xx_pcie_host0_device,
};

static void __init pcie_pb_init(void)
{
#ifdef CONFIG_SPEAR_PCIE_REV370
	spear1340_pcie_board_init(&spear13xx_pcie_host0_device.dev);
#endif
}

static int __init spear1340_pb_select(char *boards)
{
	strcpy(spear1340_plug_board, boards);

	return 0;
}
__setup("pb=", spear1340_pb_select);

static int make_pb_list(struct list_head *pb_list)
{
	char *pb_name;
	struct plug_board *pb = NULL;
	char *str = spear1340_plug_board;

	pr_debug("%s: Plug board string passed from bootargs: %s\n", __func__,
			spear1340_plug_board);
	while ((pb_name = strsep(&str, ",")) != NULL) {
		if (!strcmp(pb_name, "gmii")) {
			INIT_PB(gmii, pb);
		} else if (!strcmp(pb_name, "rgmii")) {
			INIT_PB(rgmii, pb);
		} else if (!strcmp(pb_name, "etm")) {
			INIT_PB(etm, pb);
		} else if (!strcmp(pb_name, "hdmi_rx")) {
			INIT_PB(hdmi_rx, pb);
		} else if (!strcmp(pb_name, "hdmi_tx")) {
			INIT_PB(hdmi_tx, pb);
		} else if (!strcmp(pb_name, "cam0")) {
			INIT_PB(cam0, pb);
		} else if (!strcmp(pb_name, "vga")) {
			INIT_PB(vga, pb);
		} else if (!strcmp(pb_name, "sata")) {
			INIT_PB(sata, pb);
		} else if (!strcmp(pb_name, "pcie")) {
			INIT_PB(pcie, pb);
		} else {
			pr_err("Invalid plug board requested: %s\n", pb_name);
			goto release_pb;
		}

		if (!pb)
			goto release_pb;

		list_add_tail(&pb->node, pb_list);
	}

	if (list_empty(pb_list)) {
		pr_err("Board list can't be empty\n");
			goto release_pb;
	}

	return 0;

release_pb:
	list_for_each_entry(pb, pb_list, node)
		kfree(pb);

	return -EINVAL;
}

static bool skip_device(struct list_head *pb_list, void *dev,
		enum skip_device_type skip)
{
	struct plug_board *pb;
	int i;

	list_for_each_entry(pb, pb_list, node) {
		switch (skip) {
		case SKIP_PLAT_DEVICE:
			for (i = 0; i < pb->rm_pcnt; i++) {
				if (dev == pb->rm_pdevs[i]) {
					pr_debug("%s: skip %s.%d\n",
						pb->name, pb->rm_pdevs[i]->name,
						pb->rm_pdevs[i]->id == -1 ? 0 :
						pb->rm_pdevs[i]->id);
					return true;
				}
			}
			break;
		case SKIP_AMBA_DEVICE:
			for (i = 0; i < pb->rm_acnt; i++) {
				if (dev == pb->rm_adevs[i]) {
					pr_debug("%s: skip %s\n", pb->name,
						pb->rm_adevs[i]->dev.init_name);
					return true;
				}
			}
			break;
		case SKIP_SPI_DEVICE:
			for (i = 0; i < pb->rm_spi_cnt; i++) {
				if (dev == pb->rm_spi_devs[i]) {
					pr_debug("%s: skip %s\n", pb->name,
						pb->rm_spi_devs[i]->modalias);
					return true;
				}
			}
			break;
		case SKIP_I2C_DEVICE:
			for (i = 0; i < pb->rm_i2c_cnt; i++) {
				if (dev == pb->rm_i2c_devs[i]) {
					pr_debug("%s: skip %s\n", pb->name,
					pb->rm_i2c_devs[i]->board->type);
					return true;
				}
			}
			break;

		default:
			return false;
		}
	}

	return false;
}

int __init spear1340_pb_init(struct plug_board_info *pb_info)
{
	struct platform_device **pdevs;
	struct amba_device **adevs;
	struct plug_board *pb;
	struct spi_board_info **spi_devs;
	struct i2c_dev_info **i2c_devs;
	int ret, i;
	u8 pcnt, acnt, spi_cnt, i2c_cnt;
	LIST_HEAD(pb_list);

	if (!pb_info)
		return -EINVAL;

	pdevs = pb_info->pdevs;
	pcnt = pb_info->pcnt;
	adevs = pb_info->adevs;
	acnt = pb_info->acnt;
	spi_devs = pb_info->spi_devs;
	spi_cnt = pb_info->spi_cnt;
	i2c_devs = pb_info->i2c_devs;
	i2c_cnt = pb_info->i2c_cnt;

	ret = make_pb_list(&pb_list);
	if (ret) {
		pr_err("Error creating pb_list: %d\n", ret);
		return ret;
	}

	/* Call board specific init routine */
	list_for_each_entry(pb, &pb_list, node) {
		pr_debug("%s: Initializing plug board\n", pb->name);

		if (pb->pb_init)
			pb->pb_init();
	}

	list_for_each_entry(pb, &pb_list, node) {
		if (!pb->pmx_cnt)
			continue;

		ret = pmx_devs_enable(pb->pmx_devs, pb->pmx_cnt);
		if (ret)
			pr_err("padmux: Failed adding pmx devs: %d\n", ret);

		pr_debug("%s: Added %d pmx devices\n", pb->name, pb->pmx_cnt);
	}

	/* Add SPI Devices passed from evb.c */
	for (i = 0; i < spi_cnt; i++) {
		if (skip_device(&pb_list, spi_devs[i], SKIP_SPI_DEVICE))
			continue;

		spi_register_board_info(spi_devs[i], 1);
	}

	/* Add SPI Devices requested by plug boards */
	list_for_each_entry(pb, &pb_list, node) {
		for (i = 0; i < pb->add_spi_cnt; i++)
			spi_register_board_info(pb->add_spi_devs[i], 1);

		pr_debug("%s: Added %d SPI devices\n",
				pb->name, pb->add_spi_cnt);
	}

	/* Add I2C Devices passed from evb.c */
	for (i = 0; i < i2c_cnt; i++) {
		if (skip_device(&pb_list, i2c_devs[i], SKIP_I2C_DEVICE))
			continue;

		i2c_register_board_info(i2c_devs[i]->busnum,
				i2c_devs[i]->board, 1);
	}

	/* Add I2C Devices requested by plug boards */
	list_for_each_entry(pb, &pb_list, node) {
		for (i = 0; i < pb->add_i2c_cnt; i++)
			i2c_register_board_info(pb->add_i2c_devs[i]->busnum,
				pb->add_i2c_devs[i]->board, 1);

		pr_debug("%s: Added %d I2C devices\n",
				pb->name, pb->add_i2c_cnt);
	}

	/* Add Amba Devices passed from evb.c */
	for (i = 0; i < acnt; i++) {
		if (skip_device(&pb_list, adevs[i], SKIP_AMBA_DEVICE))
			continue;

		amba_device_register(adevs[i], &iomem_resource);
	}

	/* Add Amba Devices requested by plug boards */
	list_for_each_entry(pb, &pb_list, node) {
		for (i = 0; i < pb->add_acnt; i++)
			amba_device_register(pb->add_adevs[i], &iomem_resource);

		pr_debug("%s: Added %d amba devices\n", pb->name, pb->add_acnt);
	}

	/* Add Platform Devices passed from evb.c */
	for (i = 0; i < pcnt; i++) {
		if (skip_device(&pb_list, pdevs[i], SKIP_PLAT_DEVICE))
			continue;

		platform_device_register(pdevs[i]);
	}

	/* Add Platform Devices requested by plug boards */
	list_for_each_entry(pb, &pb_list, node) {
		for (i = 0; i < pb->add_pcnt; i++)
			platform_device_register(pb->add_pdevs[i]);

		pr_debug("%s: Added %d plat devices\n", pb->name, pb->add_pcnt);
	}

	list_for_each_entry(pb, &pb_list, node)
		kfree(pb);

	return 0;
}
