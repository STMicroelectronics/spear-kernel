/*
 * AD9889B - EDID data parser
 *
 * SPEAr1340 evaluation board source file
 *
 * Copyright (C) 2011 ST Microelectronics
 * Author: Imran Khan <imran.khan@st.com>
 * Pratyush Anand <pratyush.anand@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/param.h>
#include <media/ad9889b.h>

#define FALSE 0
#define TRUE 0

/* Parse EDID block */
static void parse_block_map(struct sink_edid_info *edid,
		u8 *d, int block_number)
{
	int array_size = 0;
	int i;

	edid->segment_number_counter = 0;
	for (i = block_number + 1; i < (block_number+0x80); i += 2) {
		if ((d[i] == 0x02) || (d[i+1] == 0x02))
			array_size++;
	}
	for (i = block_number + 1; i < (block_number+0x80); i += 2) {
		if ((d[i] == 0x02) || (d[i+1] == 0x02)) {
			edid->segment_number[edid->segment_number_counter]
				= (((i - block_number) + 1)/2);
			edid->segment_number_counter++;
		}
	}
	edid->segment_number[edid->segment_number_counter] = 0;

	if (edid->segment_number_counter != (array_size))
		pr_err("Error: Problem parsing block map");

	edid->segment_number_counter = 0;
}

static int get_bits(unsigned char in, int high_bit, int low_bit)
{

	int i;
	unsigned char mask = 0;

	for (i = 0; i <= (high_bit-low_bit); i++)
		mask += (1<<i);
	mask <<= low_bit;
	in &= mask;

	return (int)(in >>= low_bit);
}

/* Parse EDID detailed timing block */
static struct sink_detailed_timing_blocklist parse_dtb(int i, unsigned char *d)
{
	struct sink_detailed_timing_blocklist tmp;

	tmp.pix_clk = ((int)d[i] | (int)(d[i+1] << 8)) * 10000;
	tmp.h_active = d[i+2] | (get_bits(d[i+4], 7, 4) << 8);
	tmp.h_blank = d[i+3] | (get_bits(d[i+4], 3, 0) << 8);
	tmp.v_active = d[i+5] | (get_bits(d[i+7], 7, 4) << 8);
	tmp.v_blank = d[i+6] | (get_bits(d[i+7], 3, 0) << 8);
	tmp.hsync_offset = d[i+8] | (get_bits(d[i+11], 7, 6) << 8);
	tmp.hsync_pw = d[i+9] | (get_bits(d[i+11], 5, 4) << 8);
	tmp.vsync_offset = get_bits(d[i+10], 8, 4)
		| (get_bits(d[i+11], 3, 2) << 8);
	tmp.vsync_pw = get_bits(d[i+10], 3, 0) | (get_bits(d[i+11], 0, 1) << 8);
	tmp.h_image_size = d[i+12] | (get_bits(d[i+14], 7, 4) << 8);
	tmp.v_image_size = d[i+13] | (get_bits(d[i+14], 3, 0) << 8);
	tmp.h_border = d[i+15];
	tmp.v_border = d[i+16];
	tmp.ilace = get_bits(d[i+17], 7, 7);
	tmp.stereo = get_bits(d[i+17], 6, 5);
	tmp.comp_sep = get_bits(d[i+17], 4, 3);
	tmp.serr_v_pol = get_bits(d[i+17], 2, 2);
	tmp.sync_comp_h_pol = get_bits(d[i+17], 1, 1);
	tmp.v_act_blank = 0;
	tmp.h_act_blank = 0;
	tmp.preferred_timing = 0;

	return tmp;
}

static void add_st(struct sink_standard_timing_list st,
		struct sink_edid_info *e)
{
	if (e->st_count < ST_MAX) {
		e->st[e->st_count] = st;
		e->st_count++;
	} else {
		/* TODO */
	}
}

/* Add detailed timing descriptor to generic EDID block */
static void add_dtb(struct sink_detailed_timing_blocklist dtb,
		struct sink_edid_info *e)
{
	if (e->dtd_count < DTD_MAX) {
		e->dtb[e->dtd_count] = dtb;
		e->dtd_count++;
	}
}

/* Add short video descriptor to CEA extension block */
static void add_svd(struct sink_sv_descriptorlist svd,
		struct sink_edid_info *e)
{
	if (e->svd_count < SVD_MAX) {
		e->cea.cea.svd[e->svd_count] = svd;
		e->svd_count++;
	}
}

/* Add short audio descriptor to CEA extension block */
static void add_sad(struct sink_sa_descriptorlist sad,
		struct sink_edid_info *e)
{
	if (e->sad_count < SAD_MAX) {
		e->cea.cea.sad[e->sad_count] = sad;
		e->sad_count++;
	}
}

static void delete_sts(struct sink_edid_info *e)
{
	int i;

	for (i = 0; i < ST_MAX; i++) {
		e->st[i].hap = 0;
		e->st[i].imasp = 0;
		e->st[i].ref = 0;
	}
}

/* Remove detailed timing block(s) from generic edid block */
static void delete_dtbs(struct sink_edid_info *e)
{
	int i;

	for (i = 0; i < DTD_MAX; i++) {
		e->dtb[i].preferred_timing = 0;
		e->dtb[i].pix_clk = 0;
		e->dtb[i].h_active = 0;
		e->dtb[i].h_blank = 0;
		e->dtb[i].h_act_blank = 0;
		e->dtb[i].v_active = 0;
		e->dtb[i].v_blank = 0;
		e->dtb[i].v_act_blank = 0;
		e->dtb[i].hsync_offset = 0;
		e->dtb[i].hsync_pw = 0;
		e->dtb[i].vsync_offset = 0;
		e->dtb[i].vsync_pw = 0;
		e->dtb[i].h_image_size = 0;
		e->dtb[i].v_image_size = 0;
		e->dtb[i].h_border = 0;
		e->dtb[i].v_border = 0;
		e->dtb[i].ilace = 0;
		e->dtb[i].stereo = 0;
		e->dtb[i].comp_sep = 0;
		e->dtb[i].serr_v_pol = 0;
		e->dtb[i].sync_comp_h_pol = 0;
	}
}

/* Remove short video descriptor from CEA extension block */
static void delete_svds(struct sink_edid_info *e)
{
	int i;

	for (i = 0; i < SVD_MAX; i++) {
		e->cea.cea.svd[i].native = 0;
		e->cea.cea.svd[i].vid_id = 0;
	}
}

/* Remove short audio descriptor from CEA extension block */
static void delete_sads(struct sink_edid_info *e)
{
	int i;

	for (i = 0; i < SAD_MAX; i++) {
		e->cea.cea.sad[i].aud_format = 0;
		e->cea.cea.sad[i].max_num_chan = 0;
		e->cea.cea.sad[i].rsvd1 = 0;
		e->cea.cea.sad[i].rsvd2 = 0;
		e->cea.cea.sad[i].khz_192 = 0;
		e->cea.cea.sad[i].khz_176 = 0;
		e->cea.cea.sad[i].khz_96 = 0;
		e->cea.cea.sad[i].khz_88 = 0;
		e->cea.cea.sad[i].khz_48 = 0;
		e->cea.cea.sad[i].khz_44 = 0;
		e->cea.cea.sad[i].khz_32 = 0;
		e->cea.cea.sad[i].unc_rsrvd = 0;
		e->cea.cea.sad[i].unc_24bit = 0;
		e->cea.cea.sad[i].unc_20bit = 0;
		e->cea.cea.sad[i].unc_16bit = 0;
		e->cea.cea.sad[i].comp_maxbitrate = 0;
		e->cea.cea.sad[i].sample_sizes = 0;
		e->cea.cea.sad[i].sample_rates = 0;
	}
}

/* Clear EDID information */
void clear_edid(struct sink_edid_info *e)
{
	delete_dtbs(e);
	delete_sads(e);
	delete_svds(e);
	delete_sts(e);
	init_edid_info(e);
	e->segment_number_counter = 0;
}

/* Parse generic edid block(byte0-byte127) */
static void parse_0_block(struct sink_edid_info *e, u8 *d)
{
	int first_dtb = 0x36;
	int last_dtb = 0x6C;
	int i, j;
	unsigned char header[8] = {0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0x00};
	/* unsigned char sum = 0; */
	struct sink_standard_timing_list st;

	if (memcmp(header, d, 8))
		e->edid_header = FALSE;
	else
		e->edid_header = TRUE;

	e->edid_extensions = d[0x7E];

#if 0
	/* Checksum calculation for block 0 */
	for (i = 0; i < 128; i++)
		sum += d[i];
	e->edid_checksum = TRUE;
}
#endif
	/*
	 * Manufacturer name is 3 5-bit ascii characters packed into 16-bit,
	 * big endian format
	 */

	e->vpi.manuf[0] = (char)(get_bits(d[0x8], 6, 2) + 'A' - 1);
	e->vpi.manuf[1] = (char)(((get_bits(d[0x8], 1, 0) << 3)
				| get_bits(d[9], 7, 5)) + ('A' - 1));
	e->vpi.manuf[2] = (char)(get_bits(d[0x9], 4, 0) + 'A' - 1);
	e->vpi.manuf[3] = '\0';
	/* product code is little endian 16-bit */
	e->vpi.prodcode[0] = d[0xA];
	e->vpi.prodcode[1] = d[0xB];

	/* optional product serial is little endian 32-bit 0x010101 = UNUSED */
	e->vpi.serial[1] = (unsigned short)d[0xC]
		| (unsigned short)(d[0xD] << 8);
	e->vpi.serial[0] = (unsigned short)d[0xE]
		|(unsigned short)(d[0xF] << 8);

	e->vpi.week = (int)d[0x10];
	e->vpi.year = 1990 + (int)d[0x11];

	e->evr.ver = d[0x12];
	e->evr.rev = d[0x13];
	pr_info("EDID version : %d.%d\n", e->evr.ver, e->evr.rev);
	pr_info("Manufacturing week : %d Manufacturing year: %d\n",
			e->vpi.week, e->vpi.year);

	/*
	 * Basic display parameter byte bits[6:0] are
	 * defined differently for analog and digital
	 */
	e->bdp.anadig = get_bits(d[0x14], 7, 7);
	if (e->bdp.anadig)
		e->bdp.dfp1x = get_bits(d[0x14], 0, 0);
	else {
		e->bdp.siglev = get_bits(d[0x14], 6, 5);
		e->bdp.setup = get_bits(d[0x14], 4, 4);
		e->bdp.sepsyn = get_bits(d[0x14], 3, 3);
		e->bdp.compsync = get_bits(d[0x14], 2, 2);
		e->bdp.sog = get_bits(d[0x14], 1, 1);
		e->bdp.vsync_ser = get_bits(d[0x14], 0, 0);
	}

	e->bdp.maxh = d[0x15];
	e->bdp.maxv = d[0x16];
	e->bdp.dtc_gam = d[0x17];
	e->bdp.standby = get_bits(d[0x18], 7, 7);
	e->bdp.suspend = get_bits(d[0x18], 6, 6);
	e->bdp.aovlp = get_bits(d[0x18], 5, 5);
	e->bdp.disptype = get_bits(d[0x18], 4, 3);
	e->bdp.def_col_sp = get_bits(d[0x18], 2, 2);
	e->bdp.pt_mode = get_bits(d[0x18], 1, 1);
	e->bdp.def_gtf = get_bits(d[0x18], 0, 0);

	/* colorimetry bits are converted into binary fractions */

	e->cc.rx = get_bits(d[0x19], 7, 6)|(d[0x1B] << 2);
	e->cc.ry = get_bits(d[0x19], 5, 4)|(d[0x1C] << 2);
	e->cc.gx = get_bits(d[0x19], 3, 2)|(d[0x1D] << 2);
	e->cc.gy = get_bits(d[0x19], 1, 0)|(d[0x1E] << 2);
	e->cc.bx = get_bits(d[0x19], 7, 6)|(d[0x1F] << 2);
	e->cc.by = get_bits(d[0x19], 5, 4)|(d[0x20] << 2);
	e->cc.wx = get_bits(d[0x19], 3, 2)|(d[0x21] << 2);
	e->cc.wy = get_bits(d[0x19], 1, 0)|(d[0x22] << 2);

	e->et.m720_400_70 = get_bits(d[0x23], 7, 7);
	e->et.m720_400_88 = get_bits(d[0x23], 6, 6);
	e->et.m640_480_60 = get_bits(d[0x23], 5, 5);
	e->et.m640_480_67 = get_bits(d[0x23], 4, 4);
	e->et.m640_480_72 = get_bits(d[0x23], 3, 3);
	e->et.m640_480_75 = get_bits(d[0x23], 2, 2);
	e->et.m800_600_56 = get_bits(d[0x23], 1, 1);
	e->et.m800_600_60 = get_bits(d[0x23], 0, 0);

	/* Established Timing II */
	e->et.m800_600_72 = get_bits(d[0x24], 7, 7);
	e->et.m800_600_75 = get_bits(d[0x24], 6, 6);
	e->et.m832_624_75 = get_bits(d[0x24], 5, 5);
	e->et.m1024_768_87 = get_bits(d[0x24], 4, 4);
	e->et.m1024_768_60 = get_bits(d[0x24], 3, 3);
	e->et.m1024_768_70 = get_bits(d[0x24], 2, 2);
	e->et.m1024_768_75 = get_bits(d[0x24], 1, 1);
	e->et.m1280_1024_75 = get_bits(d[0x24], 0, 0);
	/* Manufacturer's Timings */
	e->et.m1152_870_75 = get_bits(d[0x25], 7, 7);
	/* Reserved */
	e->et.res_7 = get_bits(d[0x25], 6, 6);
	e->et.res_6 = get_bits(d[0x25], 5, 5);
	e->et.res_5 = get_bits(d[0x25], 4, 4);
	e->et.res_4 = get_bits(d[0x25], 3, 3);
	e->et.res_3 = get_bits(d[0x25], 2, 2);
	e->et.res_2 = get_bits(d[0x25], 1, 1);
	e->et.res_1 = get_bits(d[0x25], 0, 0);

	for (j = 0x26; j <= 0x34; j += 2) {
		st.hap = d[j] * 8 + 248;
		st.imasp = get_bits(d[j+1], 7, 6);
		st.ref = get_bits(d[j+1], 5, 0) + 60;
		add_st(st, e);
	}

	for (i = first_dtb; i <= last_dtb; i += 0x12) {
		/* Monitor Descriptor */
		if ((d[i] == 0x0) && (d[i+1] == 0x0)) {
			/* Monitor S/N */
			if (d[i+3] == 0xFF) {
				pr_info("Monitor S/N\n");
				for (j = 0; j < 13; j++) {
					e->mdb.mon_snum[j] = d[i + 5 + j];
					if (d[i + 5 + j] == 0xA)
						e->mdb.mon_name[j] = '\0';
				}
			}

			/* ASCII Data String */
			if (d[i+3] == 0xFE) {
				pr_info("ASCII String\n");
				for (j = 0; j < 13; j++) {
					e->mdb.ascii_data[j] = d[i + 5 + j];
					if (d[i + 5 + j] == 0xA)
						e->mdb.mon_name[j] = '\0';
				}
			}

			/* Monitor Range Limits */
			if (d[i+3] == 0xFD) {
				pr_info("Range Limits\n");
				e->mdb.mrl_min_vrate = d[i+5];
				e->mdb.mrl_max_vrate = d[i+6];
				e->mdb.mrl_min_hrate = d[i+7];
				e->mdb.mrl_max_hrate = d[i+8];
				e->mdb.mrl_max_pix_clk = d[i+9] * 10;

				if (d[i+10] == 0x2) {
					e->mdb.mrl_gtf_support = TRUE;
					e->mdb.mrl_gtf_start_freq
						= d[i+12] * 2000;
					e->mdb.mrl_gtf_c = d[i+13]/2;
					e->mdb.mrl_gtf_m = d[i+14]|(d[1+15]<<8);
					e->mdb.mrl_gtf_k = d[i+16];
					e->mdb.mrl_gtf_j = d[i+17]/2;
				} else
					e->mdb.mrl_gtf_support = FALSE;
			}

			/* Monitor Name */
			if (d[i+3] == 0xFC) {
				pr_info("Monitor Name : ");
				for (j = 0; j < 13; j++) {
					e->mdb.mon_name[j] = d[i + 5 + j];
					if (d[i + 5 + j] == 0xA)
						e->mdb.mon_name[j] = '\0';
				}
				pr_info("%s\n", e->mdb.mon_name);
			}
			/* Additional Color Point Data */
			if (d[i+3] == 0xFB) {
				pr_info("Additional Color Point Data\n");
				e->mdb.cp_wpoint_index1 = d[i+5];
				e->mdb.cp_w_lb1 = d[i+6];
				e->mdb.cp_w_x1 = d[i+7];
				e->mdb.cp_w_y1 = d[i+8];
				e->mdb.cp_w_gam1 = d[i+9];
				if (d[i+10] != 0x00) {
					e->mdb.cp_wpoint_index2 = d[i+10];
					e->mdb.cp_w_lb2 = d[i+11];
					e->mdb.cp_w_x2 = d[i+12];
					e->mdb.cp_w_y2 = d[i+13];
					e->mdb.cp_w_gam2 = d[i+14];
				}
			}
			/* Additional Standard Timing Definitions */
			if (d[i+3] == 0xFA) {
				pr_info("Additional Standard Timing \
						Definitions\n");
				for (j = 5; j < 18; j += 2) {
					st.hap = (d[i + j]/28)-31;
					st.imasp = get_bits(d[i + j+1], 7, 6);
					st.ref = get_bits(d[i + j+1], 5, 0)
						+ 60;
					add_st(st, e);
				}
			}

			/* Dummy Descriptor */
			if (d[i+3] == 0x10)
				pr_info("Dummy Descriptor\n");

			/* Manufacturer Defined descriptor
			 * Only supports one for now. To more e->mbd->ms_byte
			 * can be converted into a linked list of ms[]
			 */
			if (d[i+3] <= 0x0F) {
				pr_info("Manufacturer Defined Descriptor\n");
				for (j = 0; j < 13; j++)
					e->mdb.ms_byte[j] = d[i + 5 + j];
			}

		} else	{
			pr_info("DTB\n");
			/* Flag 1st DTB as preferred timing */
			add_dtb(parse_dtb(i, d), e);

		}
	}
}

static void parse_cea_block(struct sink_edid_info *e, u8 *d,
		int block_number)
{
	struct sink_sa_descriptorlist sad;
	int i, j = 0, k, tag;
	/* unsigned char sum = 0; */
	struct sink_sv_descriptorlist svd;

	i = block_number;

	/* Checksum calculation for CEA extension */
#if 0
	for (j = i; j < (i + 128); j++)
		sum += d[j];
	if (sum) {
		pr_info("Check sum does not match, EDID information \
				corrupted\n");
		e->cea->checksum = FALSE;
	} else {
		pr_info("checksum ok\n");
		e->cea->checksum = TRUE;
	}
#else
	e->cea.checksum = TRUE;
#endif
	e->cea.cea_rev = d[i+1];
	e->cea.underscan = get_bits(d[i+3], 7, 7);
	e->cea.audio = get_bits(d[i+3], 6, 6);
	e->cea.YCC444 = get_bits(d[i+3], 5, 5);
	e->cea.YCC422 = get_bits(d[i+3], 4, 4);
	e->cea.native_formats = get_bits(d[i+3], 3, 0);

	/*iterrate through data block */
	while (j < i+d[i+2]) {
		tag = get_bits(d[j], 7, 5);
		if (tag == 1) {
			pr_info("Short Audio Descriptor\n");

			sad.aud_format = get_bits(d[j+1], 6, 3);
			sad.max_num_chan = get_bits(d[j+1], 2, 0)+1;
			sad.khz_192 = get_bits(d[j+2], 6, 6);
			sad.khz_176 = get_bits(d[j+2], 5, 5);
			sad.khz_96 = get_bits(d[j+2], 4, 4);
			sad.khz_88 = get_bits(d[j+2], 3, 3);
			sad.khz_48 = get_bits(d[j+2], 2, 2);
			sad.khz_44 = get_bits(d[j+2], 1, 1);
			sad.khz_32 = get_bits(d[j+2], 0, 0);
			sad.sample_rates = get_bits(d[j+2], 6, 0);

			if (sad.aud_format == 1) {
				sad.unc_24bit = get_bits(d[j+3], 2, 2);
				sad.unc_20bit = get_bits(d[j+3], 1, 1);
				sad.unc_16bit = get_bits(d[j+3], 0, 0);
				sad.sample_sizes = get_bits(d[j+3], 2, 0);
			} else
				sad.comp_maxbitrate = d[j+3] * 8;
			add_sad(sad, e);
		} else if (tag == 2) {
			pr_info("Short Video Descriptor\n");
			for (k = j + 1; k <= (j + get_bits(d[j], 4, 0)); k++) {
				svd.vid_id = get_bits(d[k], 6, 0);
				svd.native = get_bits(d[k], 7, 7);
				add_svd(svd, e);
				e->cea.cea.svd_count++;
			}
		} else if (tag == 3) {
			pr_info("Vendor Specific Data Block\n");
			/*lsb */
			e->cea.cea.ieee_reg[1] = (u16)((d[j+1])
					| (d[j+2] << 8));
			/*msb */
			e->cea.cea.ieee_reg[0] = (u16)(d[j+3]);
			e->cea.cea.hdmi_addr_a = get_bits(d[j+4], 7, 4);
			e->cea.cea.hdmi_addr_b = get_bits(d[j+4], 3, 0);
			e->cea.cea.hdmi_addr_c = get_bits(d[j+5], 7, 4);
			e->cea.cea.hdmi_addr_d = get_bits(d[j+5], 3, 0);
			if (get_bits(d[j], 4, 0) >= 6) {
				e->cea.cea.supports_ai
					= get_bits(d[j+6], 1, 1);
				e->cea.cea.vsdb_ext[0]
					= get_bits(d[j+6], 6, 0);
#if 0
				while ((k < (get_bits(d[j], 4, 0)-5))
						&& (k < 24))
					e->cea->cea->vsdb_ext[k] = d[j+6+k];
#endif

			}
			if ((e->cea.cea.ieee_reg[1] == 0x0C03)
					|| (e->cea.cea.ieee_reg[0] == 0x00))
				e->cea.cea.vsdb_hdmi = TRUE;
		} else if (tag == 4) {
			pr_info("Speaker Allocation Block\n");
			e->cea.cea.spad.rlc_rrc = get_bits(d[j+1], 6, 6);
			e->cea.cea.spad.flc_frc = get_bits(d[j+1], 5, 5);
			e->cea.cea.spad.rc = get_bits(d[j+1], 4, 4);
			e->cea.cea.spad.rl_rr = get_bits(d[j+1], 3, 3);
			e->cea.cea.spad.fc = get_bits(d[j+1], 2, 2);
			e->cea.cea.spad.lfe = get_bits(d[j+1], 1, 1);
			e->cea.cea.spad.fl_fr = get_bits(d[j+1], 0, 0);
			/*e->cea->cea->spad-> */
		} else if ((tag == 0) || (tag == 5) || (tag == 6) || (tag == 7))
			pr_info("Reserved\n");
		else
			pr_info("invalid tag in cea data block\n");

		j += (get_bits(d[j], 4, 0) + 1);
	}
	/*iterrate through Detailed Timing Descriptors */
	/*look for 18 bit detailed timing descriptors until 0xFF - 18 */
	while ((d[j] != 0x0) && (j <= 0xED)) {
		add_dtb(parse_dtb(j, d), e);
		j += 18;
		e->dtb_count++;
	}
}

/*
 * This function should locate every segment of EDID with understandable blocks
 * and return the next segment number, or zero if finished.
 */
int parse_edid(struct sink_edid_info *e, u8 *d)
{
	int i;

	if (!((d[0] == 0) || (d[0] == 2) || (d[0] == 0xF0) || (d[0x80] == 0)
				|| (d[0x80] == 2) || (d[0x80] == 0xF0))) {
		pr_info("Error: Trying to parse edid segment with \
				no valid blocks");

		/* to keep from returning a strange value */
		e->segment_number[0] = 0;
		e->segment_number_counter = 0;
	}

	if ((d[0] == 0) && (d[1] == 0xFF)) {
		parse_0_block(e, d);
		if (e->edid_extensions <= 1)
			e->segment_number[0] = 0;
		e->segment_number_counter = 0;
	}

	if (d[0x00] == 0xF0)
		parse_block_map(e, d, 0x00);

	if (d[0x80] == 0xF0)
		parse_block_map(e, d, 0x80);

	for (i = 0; i < 2; i++) {
		if ((d[i*0x80]) == 0x2) {
			pr_info("cea extension block found\n");
			parse_cea_block(e, d, i*0x80);
		}
	}

	if ((e->segment_number[e->segment_number_counter] == 0)
			&& (e->edid_extensions > 127))
		return 0x40;
	else
		e->segment_number_counter++;

	return e->segment_number[e->segment_number_counter-1];
}
EXPORT_SYMBOL(parse_edid);

static void init_vendor_product_id(struct sink_vendor_product_id *vpi)
{
	vpi->manuf[0] = 0;
	vpi->manuf[1] = 0;
	vpi->manuf[2] = 0;
	vpi->manuf[3] = '\0';
	vpi->prodcode[0] = 0;
	vpi->prodcode[1] = 0;
	vpi->prodcode[2] = '\0';
	vpi->serial[0] = 0;
	vpi->serial[1] = 0;
	vpi->week = 0;
	vpi->year = 0;
}

static void init_edid_ver_rev(struct sink_edid_ver_rev *evr)
{
	evr->ver = 0;
	evr->rev = 0;
}

static void init_basic_disp_params(struct sink_basic_disp_params *bdp)
{
	bdp->anadig = FALSE;
	bdp->siglev = 0;
	bdp->setup = FALSE;
	bdp->sepsyn = FALSE;
	bdp->compsync = FALSE;
	bdp->sog = FALSE;
	bdp->vsync_ser = FALSE;
	bdp->dfp1x = FALSE;
	bdp->maxh = 0;
	bdp->maxv = 0;
	bdp->dtc_gam = 0;
	bdp->standby = FALSE;
	bdp->suspend = FALSE;
	bdp->aovlp = FALSE;
	bdp->disptype = 0;
	bdp->def_col_sp = FALSE;
	bdp->pt_mode = FALSE;
	bdp->def_gtf = FALSE;
}

static void init_color_characteristics(struct sink_color_characteristics *cc)
{
	cc->rx = 0;
	cc->ry = 0;
	cc->gx = 0;
	cc->gy = 0;
	cc->bx = 0;
	cc->by = 0;
	cc->wx = 0;
	cc->wy = 0;
}

static void init_established_timing(struct sink_established_timing *et)
{
	/* Established Timing I */
	et->m720_400_70 = FALSE;
	et->m720_400_88 = FALSE;
	et->m640_480_60 = FALSE;
	et->m640_480_67 = FALSE;
	et->m640_480_72 = FALSE;
	et->m640_480_75 = FALSE;
	et->m800_600_56 = FALSE;
	et->m800_600_60 = FALSE;
	/* Established Timing II */
	et->m800_600_72 = FALSE;
	et->m800_600_75 = FALSE;
	et->m832_624_75 = FALSE;
	et->m1024_768_87 = FALSE;
	et->m1024_768_60 = FALSE;
	et->m1024_768_70 = FALSE;
	et->m1024_768_75 = FALSE;
	et->m1280_1024_75 = FALSE;
	/* Manufacturer's Timings */
	et->m1152_870_75 = FALSE;
	/* Reserved */
	et->res_7 = FALSE;
	et->res_6 = FALSE;
	et->res_5 = FALSE;
	et->res_4 = FALSE;
	et->res_3 = FALSE;
	et->res_2 = FALSE;
	et->res_1 = FALSE;
}

static void init_monitor_descriptor_block(
		struct sink_monitor_descriptor_block *mdb)
{
	int i;

	for (i = 0; i < 13; i++) {
		mdb->mon_snum[i] = 0;
		mdb->ascii_data[i] = 0;
		mdb->mon_name[i] = 0;
	}
	mdb->mon_snum[13] = '\0';
	mdb->ascii_data[13] = '\0';
	mdb->mon_name[13] = '\0';
	mdb->mrl_min_vrate = 0;
	mdb->mrl_max_vrate = 0;
	mdb->mrl_min_hrate = 0;
	mdb->mrl_max_hrate = 0;
	mdb->mrl_max_pix_clk = 0;
	mdb->mrl_gtf_support = 0;
	mdb->mrl_bad_stf = FALSE;
	mdb->mrl_reserved_11 = 0;
	mdb->mrl_gtf_start_freq = 0;
	mdb->mrl_gtf_c = 0;
	mdb->mrl_gtf_m = 0;
	mdb->mrl_gtf_k = 0;
	mdb->mrl_gtf_j = 0;
	mdb->cp_wpoint_index1 = 0;
	mdb->cp_w_lb1 = 0;
	mdb->cp_w_x1 = 0;
	mdb->cp_w_y1 = 0;
	mdb->cp_w_gam1 = 0;
	mdb->cp_wpoint_index2 = 0;
	mdb->cp_w_lb2 = 0;
	mdb->cp_w_x2 = 0;
	mdb->cp_w_y2 = 0;
	mdb->cp_w_gam2 = 0;
	mdb->cp_bad_set = FALSE;
	mdb->cp_pad1 = 0;
	mdb->cp_pad2 = 0;
	mdb->cp_pad3 = 0;
	mdb->st9_hap = 0;
	mdb->st9_imasp = 0;
	mdb->st9_ref = 0;
	mdb->st10_hap = 0;
	mdb->st10_imasp = 0;
	mdb->st10_ref = 0;
	mdb->st11_hap = 0;
	mdb->st11_imasp = 0;
	mdb->st11_ref = 0;
	mdb->st12_hap = 0;
	mdb->st12_imasp = 0;
	mdb->st12_ref = 0;
	mdb->st13_hap = 0;
	mdb->st13_imasp = 0;
	mdb->st13_ref = 0;
	mdb->st14_hap = 0;
	mdb->st14_imasp = 0;
	mdb->st14_ref = 0;
	mdb->st_set = FALSE;
	for (i = 0; i < 13; i++)
		mdb->ms_byte[i] = 0;
}

static void init_spad_payload(struct sink_spad_payload *sp)
{
	sp->rlc_rrc = FALSE;
	sp->flc_frc = FALSE;
	sp->rc = FALSE;
	sp->rl_rr = FALSE;
	sp->fc = FALSE;
	sp->lfe = FALSE;
	sp->fl_fr = FALSE;
}

static void init_cea_data_block(struct sink_cea_data_block *cea)
{
	int i;

	init_spad_payload(&cea->spad);
	cea->ieee_reg[0] = 0;
	cea->ieee_reg[1] = 0;
	cea->vsdb_hdmi = FALSE;
	cea->vsdb_payload = NULL;
	cea->hdmi_addr_a = 0;
	cea->hdmi_addr_b = 0;
	cea->hdmi_addr_c = 0;
	cea->hdmi_addr_d = 0;

	for (i = 0; i < 25; i++)
		cea->vsdb_ext[i] = 0;

	cea->supports_ai = FALSE;
}

static void init_cea_timing_extension(struct sink_cea_timing_extension *cea)
{
	cea->cea_rev = 0;
	cea->underscan = FALSE;
	cea->audio = FALSE;
	cea->YCC444 = FALSE;
	cea->YCC422 = FALSE;
	cea->native_formats = 0;
	init_cea_data_block(&cea->cea);
	cea->checksum = 0;
}

/* Init EDID parameters to default values */
void init_edid_info(struct sink_edid_info *edid)
{
	edid->edid_header = FALSE;
	edid->edid_extensions = 0;
	edid->edid_checksum = 0;
	edid->ext_block_count = 0;
	init_vendor_product_id(&edid->vpi);
	init_edid_ver_rev(&edid->evr);
	init_basic_disp_params(&edid->bdp);
	init_color_characteristics(&edid->cc);
	init_established_timing(&edid->et);
	/* edid->st = NULL; */
	/* edid->dtb = NULL; */
	/* init_detailed_timing_block_list(); */
	init_monitor_descriptor_block(&edid->mdb);
	init_cea_timing_extension(&edid->cea);
}
EXPORT_SYMBOL(init_edid_info);
