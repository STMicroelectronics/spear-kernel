/*
 * arch/arm/mach-spear13xx/include/mach/pcie.h
 *
 * Spear SoC PCIe handling.
 *
 * Copyright (C) 2010 ST Microelectronics
 * Pratyush Anand <pratyush.anand@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __MACH_PCIE_H
#define __MACH_PCIE_H

extern int enable_pcie0_clk(void);

struct pcie_port_info {
	u8	is_host;
	u8	is_gen1;
};
extern struct pcie_port_info *(*pcie_port_init)(int port);

struct pcie_port {
	u8			port;
	u8			root_bus_nr;
	void __iomem		*base;
	void __iomem		*app_base;
	void __iomem		*va_app_base;
	void __iomem		*va_dbi_base;
	void __iomem		*va_cfg0_base;
	spinlock_t		conf_lock;
	char			mem_space_name[16];
	char			io_space_name[16];
	struct resource		res[2];
	struct pcie_port_info	config;
};

struct pcie_app_reg {
	u32	app_ctrl_0;		/*cr0*/
	u32	app_ctrl_1;		/*cr1*/
	u32	app_status_0;		/*cr2*/
	u32	app_status_1;		/*cr3*/
	u32	msg_status;		/*cr4*/
	u32	msg_payload;		/*cr5*/
	u32	int_sts;		/*cr6*/
	u32	int_clr;		/*cr7*/
	u32	int_mask;		/*cr8*/
	u32	mst_bmisc;		/*cr9*/
	u32	phy_ctrl;		/*cr10*/
	u32	phy_status;		/*cr11*/
	u32	cxpl_debug_info_0;	/*cr12*/
	u32	cxpl_debug_info_1;	/*cr13*/
	u32	ven_msg_ctrl_0;		/*cr14*/
	u32	ven_msg_ctrl_1;		/*cr15*/
	u32	ven_msg_data_0;		/*cr16*/
	u32	ven_msg_data_1;		/*cr17*/
	u32	ven_msi_0;		/*cr18*/
	u32	ven_msi_1;		/*cr19*/
	u32	mst_rmisc;		/*cr 20*/
	u32	slv_awmisc;		/*cr 21*/
	u32	slv_armisc;		/*cr 22*/
	u32	pom0_mem_addr_start;	/*cr23*/
	u32	pom1_mem_addr_start;	/*cr24*/
	u32	pom_io_addr_start;	/*cr25*/
	u32	pom_cfg0_addr_start;	/*cr26*/
	u32	pom_cfg1_addr_start;	/*cr27*/
	u32	in0_mem_addr_start;	/*cr28*/
	u32	in1_mem_addr_start;	/*cr29*/
	u32	in_io_addr_start;	/*cr30*/
	u32	in_cfg0_addr_start;	/*cr31*/
	u32	in_cfg1_addr_start;	/*cr32*/
	u32	in_msg_addr_start;	/*cr33*/
	u32	in0_mem_addr_limit;	/*cr34*/
	u32	in1_mem_addr_limit;	/*cr35*/
	u32	in_io_addr_limit;	/*cr36*/
	u32	in_cfg0_addr_limit;	/*cr37*/
	u32	in_cfg1_addr_limit;	/*cr38*/
	u32	in_msg_addr_limit;	/*cr39*/
	u32	mem0_addr_offset_limit;	/*cr40*/
	u32	pim0_mem_addr_start;	/*cr41*/
	u32	pim1_mem_addr_start;	/*cr42*/
	u32	pim_io_addr_start;	/*cr43*/
	u32	pim_rom_addr_start;	/*cr44*/
};

/*CR0 ID*/
#define RX_LANE_FLIP_EN_ID	0
#define TX_LANE_FLIP_EN_ID	1
#define SYS_AUX_PWR_DET_ID	2
#define APP_LTSSM_ENABLE_ID	3
#define SYS_ATTEN_BUTTON_PRESSED_ID	4
#define SYS_MRL_SENSOR_STATE_ID 5
#define SYS_PWR_FAULT_DET_ID	6
#define SYS_MRL_SENSOR_CHGED_ID 7
#define SYS_PRE_DET_CHGED_ID	8
#define SYS_CMD_CPLED_INT_ID	9
#define APP_INIT_RST_0_ID	11
#define APP_REQ_ENTR_L1_ID	12
#define APP_READY_ENTR_L23_ID	13
#define APP_REQ_EXIT_L1_ID	14
#define DEVICE_TYPE_EP		(0 << 25)
#define DEVICE_TYPE_LEP		(1 << 25)
#define DEVICE_TYPE_RC		(4 << 25)
#define SYS_INT_ID		29
#define MISCTRL_EN_ID		30
#define REG_TRANSLATION_ENABLE	31

/*CR1 ID*/
#define APPS_PM_XMT_TURNOFF_ID	2
#define APPS_PM_XMT_PME_ID	5

/*CR3 ID*/
#define XMLH_LTSSM_STATE_ID	0
#define XMLH_LTSSM_STATE_L0	((u32)0x11 << XMLH_LTSSM_STATE_ID)
#define XMLH_LTSSM_STATE_MASK	((u32)0x1F << XMLH_LTSSM_STATE_ID)
#define XMLH_LINK_UP_ID	5

/*CR4 ID*/
#define CFG_MSI_EN_ID	18

/*CR6*/
#define INTA_CTRL_INT	(1 << 7)
#define INTB_CTRL_INT	(1 << 8)
#define INTC_CTRL_INT	(1 << 9)
#define INTD_CTRL_INT	(1 << 10)
#define MSI_CTRL_INT	(1 << 26)

/*CR19 ID*/
#define VEN_MSI_REQ_ID	11
#define VEN_MSI_FUN_NUM_ID	8
#define VEN_MSI_TC_ID	5
#define VEN_MSI_VECTOR_ID	0
#define VEN_MSI_REQ_EN	((u32)0x1 << VEN_MSI_REQ_ID)
#define VEN_MSI_FUN_NUM_MASK	((u32)0x7 << VEN_MSI_FUN_NUM_ID)
#define VEN_MSI_TC_MASK	((u32)0x7 << VEN_MSI_TC_ID)
#define VEN_MSI_VECTOR_MASK	((u32)0x1F << VEN_MSI_VECTOR_ID)
#endif

/*CE21-22 ID*/
/*ID definitio of ARMISC*/
#define AXI_OP_TYPE_ID	0
#define AXI_OP_BCM_ID	5
#define AXI_OP_EP_ID	6
#define AXI_OP_TD_ID	7
#define AXI_OP_ATTRIBUTE_ID	8
#define AXI_OP_TC_ID	10
#define AXI_OP_MSG_CODE_ID	13
#define AXI_OP_DBI_ACCESS_ID	21
#define AXI_OP_TYPE_MASK	0x1F
#define AXI_OP_TYPE_MEM_RDRW	0
#define AXI_OP_TYPE_MEM_RDRW_LOCKED	1
#define AXI_OP_TYPE_IO_RDRW	2
#define AXI_OP_TYPE_CONFIG_RDRW_TYPE0	4
#define AXI_OP_TYPE_CONFIG_RDRW_TYPE1	5
#define AXI_OP_TYPE_MSG_REQ	16
#define AXI_OP_TYPE_COMPLETION	10
#define AXI_OP_TYPE_COMPLETION_LOCKED	11
#define AXI_OP_TYPE_DBI_ELBI_ENABLE	1

/* synopsis specific PCIE configuration registers*/
#define PCIE_MSI_ADDR_LO	0x820	/* 32 bits */
#define PCIE_MSI_ADDR_HI	0x824	/* 32 bits */
#define PCIE_MSI_INTR0_ENABLE	0x828	/* 32 bits */
#define PCIE_MSI_INTR0_MASK	0x82C	/* 32 bits */
#define PCIE_MSI_INTR0_STATUS	0x830	/* 32 bits */

/*BAR MASK registers*/
#define PCIE_BAR0_MASK_REG	0x1010

static inline void pcie_init(struct pcie_port_info * (*fptr)(int port))
{
	pcie_port_init = fptr;
}
