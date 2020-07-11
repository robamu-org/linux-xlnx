/* 
 * xiphos-v4l2-controls.h
 *
 * Xiphos V4L2 controls header
 *
 * Copyright (C) 2020 Xiphos Systems Corporation.
 *
 * Authors: Detlev Casanova <dec@xiphos.ca>
 *
 */


#define	V4L2_CID_XIPHOS_OFFSET	0x1000
#define	V4L2_CID_XIPHOS_BASE	(V4L2_CID_USER_BASE + V4L2_CID_XIPHOS_OFFSET)

/*
 * V4L2_CID_XIPHOS_SIGNAL_IGNORE
 * 
 * This control allows the user to configure which signals must be ignored.
 * The control takes 3 u8 values: One for each channel.
 * The values are used as bit masks:
 * - Bit 0: Ignore LVAL
 * - Bit 1: Ignore FVAL
 * - Bit 2: Ignore DVAL
 * - Bits 3-7: Reserved
 *
 * This control can be set with the VIDIOC_EXT_CTRLS. Note that it is Read Only.
 */
#define V4L2_CID_XIPHOS_SIGNAL_IGNORE	(V4L2_CID_XIPHOS_BASE)

/*
 * V4L2_CID_XIPHOS_INPUT_DELAY
 *
 * This control allows the user to configure the input delays.
 * The control takes 3 u32 values: One for each channel.
 * The values must follow this format:
 * - Bits 4-0:   Input Delay units for C0
 * - Bits 9-5:   Input Delay units for C1
 * - Bits 14-10: Input delay units for C2
 * - Bits 19-15: Input delay units for C3
 * - Bits 24-20: Input delay units for Channel clock
 * - Bits 31-25: Reserved
 * Note 1 : Sets Xilinx IDELAY primitive value for each camera link pairs for proper data sampling at input. This is empiric value.
 * Note 2 : Each delay unit is about 78 ps
 *
 * This control can be set with the VIDIOC_EXT_CTRLS. Note that it is Read Only.
 */
#define V4L2_CID_XIPHOS_INPUT_DELAY	(V4L2_CID_XIPHOS_BASE + 1)


/*
 * V4L2_CID_XIPHOS_RAMP
 *
 * This control allows the user to activate the Debug ramp counter in
 * the cameralink core.
 *
 * This is not supported yet.
 */
//#define V4L2_CID_XIPHOS_RAMP		(V4L2_CID_XIPHOS_BASE + 2)
