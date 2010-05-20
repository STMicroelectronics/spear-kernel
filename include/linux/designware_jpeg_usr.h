/*
 * include/linux/designware_jpeg_usr.h
 *
 * Synopsys Designware JPEG controller - user application header file
 *
 * Copyright (C) 2010 ST Microelectronics
 * Viresh Kumar<viresh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef _DESIGNWARE_JPEG_USR_H
#define _DESIGNWARE_JPEG_USR_H

/* size of jpeg memories */
#define S_QNT_MEM_SIZE	0x40
#define QNT_MEM_SIZE	0x100
#define QNT_MEM_WIDTH	1
#define HMIN_MEM_SIZE	0x40
#define HMIN_MEM_WIDTH	4
#define HBASE_MEM_SIZE	0x80
#define HBASE_MEM_WIDTH	2
#define HSYMB_MEM_SIZE	0x150
#define HSYMB_MEM_WIDTH	1
#define DHT_MEM_SIZE	0x19C
#define DHT_MEM_WIDTH	1
#define HENC_MEM_SIZE	0x300
#define HENC_MEM_WIDTH	2

/**
 * enum jpeg_operation - possible jpeg operations
 */
enum jpeg_operation {
	JPEG_DECODE,
	JPEG_ENCODE,
	JPEG_NONE
};

/**
 * struct mcu_composition - represents MCU composition.
 * @hdc: hdc bit selects the Huffman table for the encoding of the DC
 * coefficient in the data units belonging to the color component
 * @hac: hac bit selects the Huffman table for the encoding of the AC
 * coefficients in the data units belonging to the color component.
 * @qt: QTi indicates the quantization table to be used for the color component.
 * @nblock: nblock value is the number of data units (8 x 8 blocks of data) of
 * the color component contained in the MCU.
 * @h: Horizontal Sampling factor for component.
 * @v: Vertical Sampling factor for component.
 *
 * This structure represents the MCU composition.
 */
struct mcu_composition {
	u32 hdc;
	u32 hac;
	u32 qt;
	u32 nblock;
	u32 h;
	u32 v;
};
/**
 * struct jpeg_hdr - jpeg image header.
 * @num_clr_cmp: number of color components.
 * @clr_spc_type: number of quantization tables in the output stream.
 * @num_cmp_for_scan_hdr: number of components for scan header marker segment
 * @rst_mark_en: restart marker enable/disable.
 * @xsize: number of pixels per line.
 * @ysize: number of lines.
 * @mcu_num: this value defines the number of minimum coded units to be coded,
 * @mcu_num_in_rst: number of mcu's between two restart markers.
 * @mcu_comp: array for mcu composition of all four color components
 *
 * This structure represents the header information of a jpeg image. After the
 * image is decoded we can use information present in this structure to further
 * convert this image to some other format or to do something else.
 */
struct jpeg_hdr {
	u32 num_clr_cmp;
	u32 clr_spc_type;
	u32 num_cmp_for_scan_hdr;
	u32 rst_mark_en;
	u32 xsize;
	u32 ysize;
	u32 mcu_num;
	u32 mcu_num_in_rst;
#define	MAX_MCU_COMP 4
	struct mcu_composition mcu_comp[MAX_MCU_COMP];
};

/**
 * struct jpeg_enc_info - encoding information.
 * @hdr: jpeg image header.
 * @qnt_mem: quantization memory
 * @dht_mem: DHT memory
 * @henc_mem:Huff enc memory
 *
 * This structure represents information for jpeg encoding.
 */
struct jpeg_enc_info {
	struct jpeg_hdr hdr;
	int hdr_enable;
	char qnt_mem[QNT_MEM_SIZE];
	char dht_mem[DHT_MEM_SIZE];
	char henc_mem[HENC_MEM_SIZE];
};

/**
 * struct jpeg_dec_info - decoding information.
 * @hdr: jpeg image header.
 * @qnt_mem: quantization memory
 * @hmin_mem: Huff min memory
 * @hbase_mem: Huff base memory
 * @hsymb_mem: Huff symb memory
 *
 * This structure represents information for jpeg decoding.
 */
struct jpeg_dec_info {
	struct jpeg_hdr hdr;
	int hdr_enable;
	char qnt_mem[QNT_MEM_SIZE];
	char hmin_mem[HMIN_MEM_SIZE];
	char hbase_mem[HBASE_MEM_SIZE];
	char hsymb_mem[HSYMB_MEM_SIZE];
};

/**
 * struct jpeg_info - jpeg information after decoding.
 * @hdr: jpeg image header.
 * @qnt_mem: quantization memory
 * @hmin_mem: Huff min memory
 * @hbase_mem: Huff base memory
 * @hsymb_mem: Huff symb memory
 * @dht_mem: DHT memory
 * @henc_mem: Huff enc memory
 *
 * This structure represents information after jpeg decoding.
 */
struct jpeg_info {
	struct jpeg_hdr hdr;
	char qnt_mem[QNT_MEM_SIZE];
	char hmin_mem[HMIN_MEM_SIZE];
	char hbase_mem[HBASE_MEM_SIZE];
	char hsymb_mem[HSYMB_MEM_SIZE];
	char dht_mem[DHT_MEM_SIZE];
	char henc_mem[HENC_MEM_SIZE];
};

/* ioctl defines */
#define JPEG_BASE 'J'
#define	JPEGIOC_SET_ENC_INFO	_IOW(JPEG_BASE, 1, struct jpeg_enc_info)
#define	JPEGIOC_SET_DEC_INFO	_IOW(JPEG_BASE, 2, struct jpeg_dec_info)
#define	JPEGIOC_GET_INFO	_IOR(JPEG_BASE, 3, struct jpeg_info)
#define	JPEGIOC_START		_IOW(JPEG_BASE, 4, u32)
#define	JPEGIOC_SET_SRC_IMG_SIZE	_IOW(JPEG_BASE, 5, u32)
#define	JPEGIOC_GET_OUT_DATA_SIZE	_IOR(JPEG_BASE, 6, u32)

/*Macro for shuffling b/w the two read bufs.*/
#define shuffle_buf(Buf) (Buf = (Buf == 0 ? 1 : 0))

#endif /* _DESIGNWARE_JPEG_USR_H */
