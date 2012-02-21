/*
 * sil9135a - Silicon Image SIL9135A HDMI Receiver
 *
 * Copyright 2011 ST Microelectronics. All rights reserved.
 *
 * This program is free software; you may redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * Contributors:
 * pratyush.anand@st.com
 * imran.khan@st.com
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <media/v4l2-hdmi.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-chip-ident.h>
#include <media/sil9135a.h>

static int debug;

MODULE_DESCRIPTION("Silicon Image SIL9135A HDMI Receiver driver");
MODULE_AUTHOR("Pratyush Anand <pratyush.anand@st.com>");
MODULE_LICENSE("GPL");
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "debug level (0-2)");

struct sil9135a_state {
	struct v4l2_subdev sd;
	struct v4l2_ctrl_handler hdl;
	const struct sil9135a_platform_data *pdata;
	bool is_analog;
	u32 preset;
	u8 edid[256];
	struct v4l2_ctrl *audio_sample_freq;
	struct i2c_client *i2c_video;
	struct i2c_client *i2c_audio;
	char dev_id[10];
};

static inline struct sil9135a_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct sil9135a_state, sd);
}

static inline struct v4l2_subdev *to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct sil9135a_state, hdl)->sd;
}

static inline struct device *to_dev(struct v4l2_subdev *sd)
{
	return sd->v4l2_dev->dev;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int sil9135a_g_register(struct v4l2_subdev *sd,
					struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (!v4l2_chip_match_i2c_client(client, &reg->match))
		return -EINVAL;
	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;

	/* TBD */
	return 0;
}

static int sil9135a_s_register(struct v4l2_subdev *sd,
					struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (!v4l2_chip_match_i2c_client(client, &reg->match))
		return -EINVAL;
	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;

	/* TBD */
	return 0;
}
#endif

static int sil9135a_s_ctrl(struct v4l2_ctrl *ctrl)
{
	/* struct v4l2_subdev *sd = to_sd(ctrl); */

	switch (ctrl->id) {
	/* standard ctrls */
	/* TBD: More option may be needed */
	case V4L2_CID_AUDIO_MUTE:
		/* TBD */
		return 0;
	}
	return -EINVAL;
}

static int sil9135a_g_chip_ident(struct v4l2_subdev *sd,
					struct v4l2_dbg_chip_ident *chip)
{
	/* struct i2c_client *client = v4l2_get_subdevdata(sd); */

	/* TBD */
	return -ENOIOCTLCMD;
}

static int sil9135a_log_status(struct v4l2_subdev *sd)
{
	/* TBD: All debug info need to be printed here */
	return -ENOIOCTLCMD;
}

static int sil9135a_s_routing(struct v4l2_subdev *sd,
		u32 input, u32 output, u32 config)
{
	/* TBD */
	return -ENOIOCTLCMD;
}

static int sil9135a_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	/* TBD */
	return -ENOIOCTLCMD;
}

static int sil9135a_enum_dv_presets(struct v4l2_subdev *sd,
		struct v4l2_dv_enum_preset *preset)
{
	/* TBD */
	return -ENOIOCTLCMD;
}

static int sil9135a_s_dv_preset(struct v4l2_subdev *sd,
			struct v4l2_dv_preset *preset)
{
	/* TBD */
	return -ENOIOCTLCMD;
}

static int sil9135a_query_dv_preset(struct v4l2_subdev *sd,
		struct v4l2_dv_preset *preset)
{
	/* TBD */
	return -ENOIOCTLCMD;
}

static int sil9135a_s_dv_timings(struct v4l2_subdev *sd,
			struct v4l2_dv_timings *timings)
{
	/* TBD */
	return -ENOIOCTLCMD;
}

static int sil9135a_g_dv_timings(struct v4l2_subdev *sd,
			struct v4l2_dv_timings *timings)
{
	/* TBD */
	return -ENOIOCTLCMD;
}

static int sil9135a_enum_mbus_fmt(struct v4l2_subdev *sd, unsigned int index,
			enum v4l2_mbus_pixelcode *code)
{
	/* TBD : To revisit */
	if (index)
		return -EINVAL;
	/* Good enough for now */
	*code = V4L2_MBUS_FMT_FIXED;
	return 0;
}

static int sil9135a_g_mbus_fmt(struct v4l2_subdev *sd,
			struct v4l2_mbus_framefmt *fmt)
{
	return -ENOIOCTLCMD;
}

static int sil9135a_isr(struct v4l2_subdev *sd, u32 status, bool *handled)
{
	return 0;
}

static long sil9135a_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	/* struct sil9135a_state *state = to_state(sd); */
	/* int err; */

	switch (cmd) {
	case V4L2_S_EDID:
		/* TBD */
		break;
	default:
		v4l2_err(sd, "unknown ioctl %08x\n", cmd);
		return -EINVAL;
	}
	return 0;
}

static const struct v4l2_ctrl_ops sil9135a_ctrl_ops = {
	.s_ctrl = sil9135a_s_ctrl,
};

static const struct v4l2_subdev_core_ops sil9135a_core_ops = {
	/* TBD : See, if all these functions are needed. */
	/* TBD : See, if other function is needed. */
	.log_status = sil9135a_log_status,
	.g_ext_ctrls = v4l2_subdev_g_ext_ctrls,
	.try_ext_ctrls = v4l2_subdev_try_ext_ctrls,
	.s_ext_ctrls = v4l2_subdev_s_ext_ctrls,
	.g_ctrl = v4l2_subdev_g_ctrl,
	.s_ctrl = v4l2_subdev_s_ctrl,
	.queryctrl = v4l2_subdev_queryctrl,
	.querymenu = v4l2_subdev_querymenu,
	.ioctl = sil9135a_ioctl,
	.g_chip_ident = sil9135a_g_chip_ident,
	.interrupt_service_routine = sil9135a_isr,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = sil9135a_g_register,
	.s_register = sil9135a_s_register,
#endif
};

static const struct v4l2_subdev_video_ops sil9135a_video_ops = {
	/* TBD : See, if all these functions are needed. */
	/* TBD : See, if other function is needed. */
	.s_routing = sil9135a_s_routing,
	.g_input_status = sil9135a_g_input_status,
	.enum_dv_presets = sil9135a_enum_dv_presets,
	.s_dv_preset = sil9135a_s_dv_preset,
	.query_dv_preset = sil9135a_query_dv_preset,
	.s_dv_timings = sil9135a_g_dv_timings,
	.g_dv_timings = sil9135a_s_dv_timings,
	.enum_mbus_fmt = sil9135a_enum_mbus_fmt,
	.g_mbus_fmt = sil9135a_g_mbus_fmt,
	.try_mbus_fmt = sil9135a_g_mbus_fmt,
	.s_mbus_fmt = sil9135a_g_mbus_fmt,
};

static const struct v4l2_subdev_ops sil9135a_ops = {
	/* TBD : See, if other ops are needed*/
	.core = &sil9135a_core_ops,
	.video = &sil9135a_video_ops,
};

static const struct v4l2_ctrl_config sil9135a_ctrl_audio_sample_freq = {
	.ops = &sil9135a_ctrl_ops,
	.id = V4L2_CID_DV_TX_AUDIO_SAMPLE_FREQ,
	.name = "Audio Sample Freq",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 32000,
	.max = 768000,
	.step = 0,
	.def = 480000,
	.flags = V4L2_CTRL_FLAG_UPDATE,
};

static int sil9135a_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct sil9135a_state *state;
	struct sil9135a_platform_data *pdata = client->dev.platform_data;
	struct v4l2_ctrl_handler *hdl;
	struct v4l2_subdev *sd;

	/* Check if the adapter supports the needed features */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	v4l_dbg(1, debug, client, "detecting sil9135a client on address 0x%x\n",
			client->addr << 1);
	state = kzalloc(sizeof(struct sil9135a_state), GFP_KERNEL);
	if (state == NULL) {
		v4l_err(client, "Could not allocate sil9135a_state memory!\n");
		return -ENOMEM;
	}

	sd = &state->sd;
	v4l2_i2c_subdev_init(sd, client, &sil9135a_ops);
	state->pdata = pdata;
	state->preset = V4L2_DV_1080P60;

	hdl = &state->hdl;

	v4l2_info(sd, "%s found @ 0x%x (%s)\n", client->name,
			client->addr << 1, client->adapter->name);
	return 0;
}

/* ----------------------------------------------------------------------- */

static int sil9135a_remove(struct i2c_client *client)
{
	return 0;
}

/* ----------------------------------------------------------------------- */

static struct i2c_device_id sil9135a_id[] = {
	{ "sil9135a", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sil9135a_id);

static struct i2c_driver sil9135a_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "sil9135a",
	},
	.probe = sil9135a_probe,
	.remove = sil9135a_remove,
	.id_table = sil9135a_id,
};

static int __init sil9135a_init(void)
{
	return i2c_add_driver(&sil9135a_driver);
}

static void __exit sil9135a_exit(void)
{
	i2c_del_driver(&sil9135a_driver);
}

module_init(sil9135a_init);
module_exit(sil9135a_exit);
