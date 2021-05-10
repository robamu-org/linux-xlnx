/* 
 * xiphos-v4l2-controls.h
 *
 * Xiphos V4L2 controls header
 *
 * Copyright (C) 2021 Xiphos Systems Corporation.
 *
 * Authors: Detlev Casanova <dec@xiphos.ca>
 *
 * This is a copy of the xiphos-v4l2-controls.h file in the kernel uapi headers.
 * https://www.yoctoproject.org/pipermail/yocto/2015-March/024155.html recommands
 * to keep a copy in sync here.
 * A better way is to have yocto install it in the sdk and in the sysroot when building
 * an image.
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
 * The control takes 3 * 5 u16 values (2 dimensions): One for each signal of each channel.
 * The values must follow this format:
 * The first dimension represent the channel (X, Y and Z)
 * The second dimension represent the signal (0, 1, 2, 3 and CLK)
 *
 * The table must be flattened in memory:
 * |X0|X1|X2|X3|XC|Y0|Y1|Y2|Y3|YC|Z0|Z1|Z2|Z3|ZC|
 *
 * Q7: each value must be in the range 0-0x01F (5 bits)
 * Q8: each value must be in the range 0-0x1FF (9 bits)
 *
 * Note 1 : Sets Xilinx IDELAY primitive value for each camera link pairs for proper data sampling at input. This is empiric value.
 * Note 2 : Each delay unit is about 78 ps
 *
 * This control can be set with the VIDIOC_EXT_CTRLS. Note that it is Read Only.
 */
#define V4L2_CID_XIPHOS_INPUT_DELAY	(V4L2_CID_XIPHOS_BASE + 1)

/*
 * xsc_cl_ctrl_trigger
 * This control allows the user to configure the trigger.
 *
 * This control is only available on the Q7.
 *
 * The control takes 3 u32 values:
 * - Value 0: first duration in 100 us increments (0-0x3ffff)
 * - Value 1: second duration in 100 us increments (0-0x3ffff)
 * - Value 2:
 *   * Bit 0: set trigger polarity
 *     - 0: first half of pulse low, second half high
 *     - 1: first half of pulse high, second half low
 *
 * Setting value 0 or value 1 to 0 deactivates the trigger. Useful to use an external
 * trigger or free-run mode.
 *
 * This control can be set with the VIDIOC_EXT_CTRLS. Note that it is Read Only.
 */
#define V4L2_CID_XIPHOS_TRIGGER         (V4L2_CID_XIPHOS_BASE + 2)

/*
 * xsc_cl_ctrl_trigger_table
 * This control allows the user to configure the trigger table.
 *
 * This control is only available on the Q8.
 *
 * The control takes a table of 0x1000 u32 values formatted as follow:
 * The table allows to configure 0x800 (2048) triggers. Each trigger trigger is
 * configured with 2 18-bits values, each value padded to be stored on a 32-bit word.
 * The both values represent the duration for the trigger to be high and the duration
 * for the trigger to be low. Each increment represents 100 us.
 *
 * The first 0x800 u32 of the table represent the high duration of each trigger.
 * The last 0x800 u32 of the table represent the low duration of each trigger.
 *
 * For a trigger of a duration of 6 ms high and 10 ms low (giving a frame rate of 60 FPS),
 * the table would be configured as follow:
 * TABLE[0x0]	= 60  (0x3C)
 * TABLE[0x800]	= 100 (0x64)
 *
 * The following trigger durations would be configured at the positions 0x1 and 0x801.
 *
 * It is also possible to configure less than 2048 values. To do that, set the next value to 0:
 *
 * TABLE[0x0]	= 200  (0x3C)
 * TABLE[0x1]	= 200  (0x3C)
 * TABLE[0x2]	= 0
 *
 * TABLE[0x800]	= 800 (0x64)
 * TABLE[0x801]	= 800 (0x64)
 * TABLE[0x802]	= X (don't care)
 *
 * This will configure the ip core to take 2 frames at 10 FPS.
 *
 * This control can be set with the VIDIOC_EXT_CTRLS. Note that it is Read Only.
 */
#define V4L2_CID_XIPHOS_TRIGGER_TABLE         (V4L2_CID_XIPHOS_BASE + 3)

/*
 * xsc_cl_ctrl_trigger_config
 * This control allows the user to configure the trigger parameters.
 *
 * This control is only available on the Q8.
 *
 * The control takes 2 u32 value.
 * The first value can contain the following defined flags (See description below) :
 * * XIPHOS_TRIGGER_INVERT_POLARITY
 * * XIPHOS_TRIGGER_CONTINUOUS
 * * XIPHOS_TRIGGER_WAIT_PPS
 *
 * The second value is the delay before launching the trig sequence on a PPS signal.
 * It is encoded on an 18-bits value, each increment represents 100 us.
 *
 * This control can be set with the VIDIOC_EXT_CTRLS. Note that it is Read Only.
 */
#define V4L2_CID_XIPHOS_TRIGGER_CONFIG         (V4L2_CID_XIPHOS_BASE + 4)

/*
 * Define Trigger configuration flags
 */
#define XIPHOS_TRIGGER_CONTINUOUS	(0x1)		// Run continuously by repeating the TRIG[0] until stream off
#define XIPHOS_TRIGGER_INVERT_POLARITY	(0x1 << 1)	// Invert the polarity of the Trigger
#define XIPHOS_TRIGGER_WAIT_PPS		(0x1 << 2)	// Wait for PPS to send trigger

/*
 * V4L2_CID_XIPHOS_RAMP
 *
 * This control allows the user to activate the Debug ramp counter in
 * the cameralink core.
 *
 * This is not supported yet.
 */
//#define V4L2_CID_XIPHOS_RAMP		(V4L2_CID_XIPHOS_BASE + 2)
