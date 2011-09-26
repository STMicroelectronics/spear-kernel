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
 * - cam: camera sensors for all camera devices
 * - vga:
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
 * bootargs: console=ttyAMA0,115200 pb=rgmii,hdmi_tx,cam
 */

#include <linux/bug.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <mach/generic.h>
#include <mach/hardware.h>
#include <mach/spear1340_misc_regs.h>

#define DEBUG 1
#define INIT_PB(pb_name, pb)						\
	do {								\
		pb = kmalloc(sizeof(struct plug_board), GFP_KERNEL);	\
		if (!pb) {						\
			pr_err("Error allocating memory for pb: %s\n",	\
				#pb_name);				\
			continue;					\
		}							\
									\
		strcpy(pb->name, #pb_name);				\
		pb->pmx_devs = pb_name##_pb_pmx_devs;			\
		pb->rm_adevs = pb_name##_pb_rm_adevs;			\
		pb->add_adevs = pb_name##_pb_add_adevs;			\
		pb->rm_pdevs = pb_name##_pb_rm_pdevs;			\
		pb->add_pdevs = pb_name##_pb_add_pdevs;			\
		pb->pmx_cnt = ARRAY_SIZE(pb_name##_pb_pmx_devs);	\
		pb->rm_acnt = ARRAY_SIZE(pb_name##_pb_rm_adevs);	\
		pb->add_acnt = ARRAY_SIZE(pb_name##_pb_add_adevs);	\
		pb->rm_pcnt = ARRAY_SIZE(pb_name##_pb_rm_pdevs);	\
		pb->add_pcnt = ARRAY_SIZE(pb_name##_pb_add_pdevs);	\
		pb->pb_init = pb_name##_pb_init;			\
									\
		pr_info("Adding plug board: %s\n", #pb_name);		\
	} while (0)

struct plug_board {
	struct pmx_dev **pmx_devs;
	struct amba_device **rm_adevs;
	struct amba_device **add_adevs;
	struct platform_device **rm_pdevs;
	struct platform_device **add_pdevs;
	u32 pmx_cnt;
	u32 rm_acnt;
	u32 add_acnt;
	u32 rm_pcnt;
	u32 add_pcnt;
	void (*pb_init)(void);

	struct list_head node;
	char name[10];
};

/* string specifying which plug boards are requested */
char spear1340_plug_board[50] = {'\0', };


/* Definitions specific to GMII plug board */
/* padmux devices to enable */
static struct pmx_dev *gmii_pb_pmx_devs[] = {
};

/* Amba and platform devices to be removed, added previously by evb board */
static struct amba_device *gmii_pb_rm_adevs[] __initdata = {
};

static struct platform_device *gmii_pb_rm_pdevs[] __initdata = {
};

/* Amba and platform devices to be added */
static struct amba_device *gmii_pb_add_adevs[] __initdata = {
};

static struct platform_device *gmii_pb_add_pdevs[] __initdata = {
};

static void __init gmii_pb_init(void)
{
}


/* Definitions specific to RGMII plug board */
/* padmux devices to enable */
static struct pmx_dev *rgmii_pb_pmx_devs[] = {
};

/* Amba and platform devices to be removed, added previously by evb board */
static struct amba_device *rgmii_pb_rm_adevs[] __initdata = {
};

static struct platform_device *rgmii_pb_rm_pdevs[] __initdata = {
};

/* Amba and platform devices to be added */
static struct amba_device *rgmii_pb_add_adevs[] __initdata = {
};

static struct platform_device *rgmii_pb_add_pdevs[] __initdata = {
};

static void __init rgmii_pb_init(void)
{
}


/* Definitions specific to ETM plug board */
/* padmux devices to enable */
static struct pmx_dev *etm_pb_pmx_devs[] = {
};

/* Amba and platform devices to be removed, added previously by evb board */
static struct amba_device *etm_pb_rm_adevs[] __initdata = {
};

static struct platform_device *etm_pb_rm_pdevs[] __initdata = {
};

/* Amba and platform devices to be added */
static struct amba_device *etm_pb_add_adevs[] __initdata = {
};

static struct platform_device *etm_pb_add_pdevs[] __initdata = {
};

static void __init etm_pb_init(void)
{
}


/* Definitions specific to HDMI RX plug board */
/* padmux devices to enable */
static struct pmx_dev *hdmi_rx_pb_pmx_devs[] = {
};

/* Amba and platform devices to be removed, added previously by evb board */
static struct amba_device *hdmi_rx_pb_rm_adevs[] __initdata = {
};

static struct platform_device *hdmi_rx_pb_rm_pdevs[] __initdata = {
};

/* Amba and platform devices to be added */
static struct amba_device *hdmi_rx_pb_add_adevs[] __initdata = {
};

static struct platform_device *hdmi_rx_pb_add_pdevs[] __initdata = {
};

static void __init hdmi_rx_pb_init(void)
{
}


/* Definitions specific to HDMI TX plug board */
/* padmux devices to enable */
static struct pmx_dev *hdmi_tx_pb_pmx_devs[] = {
};

/* Amba and platform devices to be removed, added previously by evb board */
static struct amba_device *hdmi_tx_pb_rm_adevs[] __initdata = {
};

static struct platform_device *hdmi_tx_pb_rm_pdevs[] __initdata = {
};

/* Amba and platform devices to be added */
static struct amba_device *hdmi_tx_pb_add_adevs[] __initdata = {
};

static struct platform_device *hdmi_tx_pb_add_pdevs[] __initdata = {
};

static void __init hdmi_tx_pb_init(void)
{
}


/* Definitions specific to CAM plug board */
/* padmux devices to enable */
static struct pmx_dev *cam_pb_pmx_devs[] = {
};

/* Amba and platform devices to be removed, added previously by evb board */
static struct amba_device *cam_pb_rm_adevs[] __initdata = {
};

static struct platform_device *cam_pb_rm_pdevs[] __initdata = {
};

/* Amba and platform devices to be added */
static struct amba_device *cam_pb_add_adevs[] __initdata = {
};

static struct platform_device *cam_pb_add_pdevs[] __initdata = {
};

static void __init cam_pb_init(void)
{
}


/* Definitions specific to VGA plug board */
/* padmux devices to enable */
static struct pmx_dev *vga_pb_pmx_devs[] = {
};

/* Amba and platform devices to be removed, added previously by evb board */
static struct amba_device *vga_pb_rm_adevs[] __initdata = {
};

static struct platform_device *vga_pb_rm_pdevs[] __initdata = {
};

/* Amba and platform devices to be added */
static struct amba_device *vga_pb_add_adevs[] __initdata = {
};

static struct platform_device *vga_pb_add_pdevs[] __initdata = {
};

static void __init vga_pb_init(void)
{
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
	struct plug_board *pb;
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
		} else if (!strcmp(pb_name, "cam")) {
			INIT_PB(cam, pb);
		} else if (!strcmp(pb_name, "vga")) {
			INIT_PB(vga, pb);
		} else {
			pr_err("Invalid plug board requested: %s\n", pb_name);
			goto release_pb;
		}

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

static bool skip_device(struct list_head *pb_list, void *dev, bool is_pdev)
{
	struct plug_board *pb;
	int i;

	list_for_each_entry(pb, pb_list, node) {
		if (is_pdev) {
			for (i = 0; i < pb->rm_pcnt; i++) {
				if (dev == pb->rm_pdevs[i]) {
					pr_debug("%s: skip %s.%d\n",
						pb->name, pb->rm_pdevs[i]->name,
						pb->rm_pdevs[i]->id == -1 ? 0 :
						pb->rm_pdevs[i]->id);
					return true;
				}
			}
		} else {
			for (i = 0; i < pb->rm_acnt; i++) {
				if (dev == pb->rm_adevs[i]) {
					pr_debug("%s: skip %s\n", pb->name,
						pb->rm_adevs[i]->dev.init_name);
					return true;
				}
			}
		}
	}

	return false;
}

int __init spear1340_pb_init(struct platform_device **pdevs, u8 pcnt,
		struct amba_device **adevs, u8 acnt)
{
	LIST_HEAD(pb_list);
	struct plug_board *pb;
	int ret, i;

	ret = make_pb_list(&pb_list);
	if (ret) {
		pr_err("Error creating pb_list: %d\n", ret);
		return ret;
	}

	/* Call board specific init routine */
	list_for_each_entry(pb, &pb_list, node) {
		pr_debug("%s: Initializing plug board\n", pb->name);

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

	/* Add Amba Devices passed from evb.c */
	for (i = 0; i < acnt; i++) {
		if (skip_device(&pb_list, adevs[i], false))
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
		if (skip_device(&pb_list, pdevs[i], false))
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
