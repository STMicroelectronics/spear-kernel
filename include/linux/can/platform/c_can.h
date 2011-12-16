/*
 * CAN bus driver platform header for Bosch C_CAN controller
 *
 * Copyright (C) 2010 ST Microelectronics
 * Bhupesh Sharma <bhupesh.sharma@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __CAN_PLATFORM_C_CAN_H__
#define __CAN_PLATFORM_C_CAN_H__

/**
 * struct c_can_platform_data - C_CAN Platform Data
 * @is_quirk_required: Depending on the SoC being used
 * determine if a SW fix (/quirk) is equired for the c_can controller.
 */
struct c_can_platform_data {
	bool is_quirk_required;
};

#endif /* __CAN_PLATFORM_C_CAN_H__ */
