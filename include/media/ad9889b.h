/*
 * Analog Devices AD9389B/AD9889B video encoder driver header
 *
 * Copyright 2011 ST Microelectronics. All rights reserved.
 *
 * This program is free software; you may redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef AD9889B_H
#define AD9889B_H

/* notify events */
#define AD9889B_MONITOR_DETECT 0
#define AD9889B_EDID_DETECT 1

#define ST_MAX 5
#define DTD_MAX 5
#define SAD_MAX 5
#define SVD_MAX 15

struct fb_var_params {
	unsigned int xres;		/* visible resolution */
	unsigned int yres;

	/* Timing: All values in pixclocks, except pixclock (of course) */
	unsigned int pixclock;		/* pixel clock in ps (pico seconds) */
	unsigned int left_margin;	/* time from sync to picture	*/
	unsigned int right_margin;	/* time from picture to sync	*/
	unsigned int upper_margin;	/* time from sync to picture	*/
	unsigned int lower_margin;
	unsigned int hsync_len;		/* length of horizontal sync	*/
	unsigned int vsync_len;		/* length of vertical sync	*/
};

struct ad9889b_monitor_detect {
	int present;
};

struct ad9889b_edid_detect {
	int present;
	int segment;
};

struct sink_vendor_product_id {
	char manuf[4];
	int prodcode[3];
	unsigned short serial[2];
	int week;
	int year;
};

struct sink_edid_ver_rev {
	int ver;
	int rev;
};

struct sink_basic_disp_params {
	int anadig;
	int siglev;
	int setup;
	int sepsyn;
	int compsync;
	int sog;
	int vsync_ser;
	int dfp1x;
	int maxh;
	int maxv;
	int dtc_gam;
	int standby;
	int suspend;
	int aovlp;
	int disptype;
	int def_col_sp;
	int pt_mode;
	int def_gtf;
};

struct sink_color_characteristics {
	int rx;
	int ry;
	int gx;
	int gy;
	int bx;
	int by;
	int wx;
	int wy;
};

struct sink_established_timing {
	/* Established Timing I */
	int m720_400_70;
	int m720_400_88;
	int m640_480_60;
	int m640_480_67;
	int m640_480_72;
	int m640_480_75;
	int m800_600_56;
	int m800_600_60;
	/* Established Timing II */
	int m800_600_72;
	int m800_600_75;
	int m832_624_75;
	int m1024_768_87;
	int m1024_768_60;
	int m1024_768_70;
	int m1024_768_75;
	int m1280_1024_75;
	/* Manufacturer's Timings */
	int m1152_870_75;
	/* Reserved */
	int res_7;
	int res_6;
	int res_5;
	int res_4;
	int res_3;
	int res_2;
	int res_1;
};

struct sink_standard_timing_list {
	int hap;
	int imasp;
	int ref;
};

struct sink_detailed_timing_blocklist {
	int preferred_timing;
	int pix_clk;
	int h_active;
	int h_blank;
	int h_act_blank;
	int v_active;
	int v_blank;
	int v_act_blank;
	int hsync_offset;
	int hsync_pw;
	int vsync_offset;
	int vsync_pw;
	int h_image_size;
	int v_image_size;
	int h_border;
	int v_border;
	int ilace;
	int stereo;
	int comp_sep;
	int serr_v_pol;
	int sync_comp_h_pol;
};

struct sink_monitor_descriptor_block {
	char mon_snum[14];
	char ascii_data[14];
	int mrl_min_vrate;
	int mrl_max_vrate;
	int mrl_min_hrate;
	int mrl_max_hrate;
	int mrl_max_pix_clk;
	int mrl_gtf_support;
	int mrl_bad_stf;
	int mrl_reserved_11;
	int mrl_gtf_start_freq;
	int mrl_gtf_c;
	int mrl_gtf_m;
	int mrl_gtf_k;
	int mrl_gtf_j;
	char mon_name[14];
	int cp_wpoint_index1;
	int cp_w_lb1;
	int cp_w_x1;
	int cp_w_y1;
	int cp_w_gam1;
	int cp_wpoint_index2;
	int cp_w_lb2;
	int cp_w_x2;
	int cp_w_y2;
	int cp_w_gam2;
	int cp_bad_set;
	int cp_pad1;
	int cp_pad2;
	int cp_pad3;
	int st9_hap;
	int st9_imasp;
	int st9_ref;
	int st10_hap;
	int st10_imasp;
	int st10_ref;
	int st11_hap;
	int st11_imasp;
	int st11_ref;
	int st12_hap;
	int st12_imasp;
	int st12_ref;
	int st13_hap;
	int st13_imasp;
	int st13_ref;
	int st14_hap;
	int st14_imasp;
	int st14_ref;
	int st_set;
	int ms_byte[13];
};

struct sink_sv_descriptorlist {
	int native;
	int vid_id;
};

struct sink_sa_descriptorlist {
	int rsvd1;
	int aud_format;
	int max_num_chan;
	int rsvd2;
	int khz_192;
	int khz_176;
	int khz_96;
	int khz_88;
	int khz_48;
	int khz_44;
	int khz_32;
	int unc_rsrvd;
	int unc_24bit;
	int unc_20bit;
	int unc_16bit;
	int comp_maxbitrate;
	unsigned char sample_sizes;
	unsigned char sample_rates;
};

struct sink_spad_payload {
	int rlc_rrc;
	int flc_frc;
	int rc;
	int rl_rr;
	int fc;
	int lfe;
	int fl_fr;
};

struct sink_cea_data_block {
	struct sink_sv_descriptorlist svd[SVD_MAX];
	struct sink_sa_descriptorlist sad[SAD_MAX];
	struct sink_spad_payload spad;
	unsigned short ieee_reg[2];
	int vsdb_hdmi;
	char *vsdb_payload;
	int hdmi_addr_a;
	int hdmi_addr_b;
	int hdmi_addr_c;
	int hdmi_addr_d;
	int vsdb_ext[26];
	int supports_ai;
	int svd_count;
};

struct sink_cea_timing_extension {
	int cea_rev;
	int underscan;
	int audio;
	int YCC444;
	int YCC422;
	int native_formats;
	struct sink_cea_data_block cea;
	int checksum;
};

struct sink_edid_info {
	int edid_header;
	int edid_extensions;
	int edid_checksum;
	int ext_block_count;
	struct sink_vendor_product_id vpi;
	struct sink_edid_ver_rev evr;
	struct sink_basic_disp_params bdp;
	struct sink_color_characteristics cc;
	struct sink_established_timing et;
	struct sink_standard_timing_list st[ST_MAX];
	struct sink_detailed_timing_blocklist dtb[DTD_MAX];
	struct sink_monitor_descriptor_block mdb;
	struct sink_cea_timing_extension cea;
	int st_count;
	int dtb_count;
	int segment_number_counter;
	int segment_number[128];
	int dtd_count;
	int sad_count;
	int svd_count;
};

int parse_edid(struct sink_edid_info *edid_info, u8 *d);
void init_edid_info(struct sink_edid_info *edid);
void clear_edid(struct sink_edid_info *edid);
#endif
