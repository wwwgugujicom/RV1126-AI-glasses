// SPDX-License-Identifier: GPL-2.0
/*
 * OC0TB1B camera sensor driver - 400x400, 10-bit, MIPI
 *
 * Copyright (C) 2022 Rockchip Electronics Co., Ltd.
 *
 * V0.0X01.0X03 - replaced init table with Dothinkey reference config
 *                400x400 @ 240fps, MIPI 1-lane 10-bit (Bayer SRGGB)
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/rk-camera-module.h>
#include <linux/pinctrl/consumer.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <linux/version.h>
#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x03)

#define REG_NULL			0xFFFF
#define REG_DELAY			0xFFFE

#define OC0TB1B_REG_VALUE_08BIT		1
#define OC0TB1B_REG_VALUE_16BIT		2

#define OC0TB1B_LANES			1
#define OC0TB1B_BITS_PER_SAMPLE	10

/* 1-lane @ 400Mbps/lane -> pixel rate = 400M * 2 / 10 = 80 MP/s */
#define OC0TB1B_LINK_FREQ		400000000ULL
#define OC0TB1B_PIXEL_RATE \
	(OC0TB1B_LINK_FREQ * 2LL * OC0TB1B_LANES / OC0TB1B_BITS_PER_SAMPLE)

#define OC0TB1B_XVCLK_FREQ		24000000

/* Chip ID */
#define OC0TB1B_REG_CHIP_ID_H		0x300a
#define OC0TB1B_REG_CHIP_ID_L		0x300b
#define OC0TB1B_CHIP_ID_H			0xC7

/* Stream control */
#define OC0TB1B_REG_CTRL_MODE		0x0100
#define OC0TB1B_MODE_SW_STANDBY		0x00
#define OC0TB1B_MODE_STREAMING		0x01

/* Exposure (MDS 5.6): 0x3500/0x3501/0x3502 = expo_coarse[23:16]/[15:8]/[7:0] */
#define OC0TB1B_REG_EXPOSURE_H		0x3500
#define OC0TB1B_REG_EXPOSURE_M		0x3501
#define OC0TB1B_REG_EXPOSURE_L		0x3502
#define OC0TB1B_EXPOSURE_MIN		1
#define OC0TB1B_EXPOSURE_STEP		1

/*
 * Gain (MDS 5.6):
 *   0x3508 Bit[3:0] = gain_coarse[3:0]
 *   0x3509 Bit[7:1] = gain_fine[7:1]
 */
#define OC0TB1B_REG_GAIN_COARSE		0x3508
#define OC0TB1B_REG_GAIN_FINE		0x3509
#define OC0TB1B_REG_GAIN		OC0TB1B_REG_GAIN_COARSE
#define OC0TB1B_GAIN_MIN		0x01
#define OC0TB1B_GAIN_MAX		0x0f
#define OC0TB1B_GAIN_STEP		1
#define OC0TB1B_GAIN_DEFAULT		0x01

/* Frame length (VTS) */
#define OC0TB1B_REG_VTS_H		0x380e
#define OC0TB1B_REG_VTS_L		0x380f
#define OC0TB1B_VTS_MAX			0x7fff

/* HTS */
#define OC0TB1B_REG_HTS_H		0x380c
#define OC0TB1B_REG_HTS_L		0x380d

/* Test pattern (MDS 5.24): 0x5101 Bit[0] enable, 0x5102 Bit[2:0] mode */
#define OC0TB1B_REG_TEST_PATTERN_EN	0x5101
#define OC0TB1B_TEST_PATTERN_ENABLE	BIT(0)
#define OC0TB1B_REG_TEST_PATTERN	0x5102
#define OC0TB1B_TEST_PATTERN_MASK	0x07

/*
 * Flip / Mirror (MDS 5.22 / 5.23):
 *   0x5003 Bit[5] manual flip option enable, Bit[4] manual mirror option enable
 *   0x5015 Bit[5] manual flip,             Bit[4] manual mirror
 */
#define OC0TB1B_REG_FLIP_MIRROR_EN	0x5003
#define OC0TB1B_REG_FLIP_MIRROR		0x5015
#define OC0TB1B_FLIP_MASK		BIT(5)
#define OC0TB1B_MIRROR_MASK		BIT(4)

#define OC0TB1B_NAME			"oc0tb1b"

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"sleep"

static const char * const oc0tb1b_supply_names[] = {
	"avdd",		/* Analog power */
	"dvdd",		/* Digital core power */
};

#define OC0TB1B_NUM_SUPPLIES	ARRAY_SIZE(oc0tb1b_supply_names)

struct regval {
	u16 addr;
	u8 val;
};

struct oc0tb1b_mode {
	u32 bus_fmt;
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	const struct regval *reg_list;
	u32 hdr_mode;
	u32 xvclk_freq;
	u32 link_freq_idx;
};

struct oc0tb1b {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[OC0TB1B_NUM_SUPPLIES];

	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pins_default;
	struct pinctrl_state	*pins_sleep;

	struct v4l2_subdev	subdev;
	struct media_pad	pad;
	struct v4l2_ctrl_handler handler;
	struct v4l2_ctrl	*exposure;
	struct v4l2_ctrl	*gain;
	struct v4l2_ctrl	*hblank;
	struct v4l2_ctrl	*vblank;
	struct v4l2_ctrl	*pixel_rate;
	struct v4l2_ctrl	*link_freq;
	struct v4l2_ctrl	*test_pattern;

	struct mutex		mutex;
	bool			streaming;
	bool			power_on;
	const struct oc0tb1b_mode *cur_mode;

	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
};

#define to_oc0tb1b(sd) container_of(sd, struct oc0tb1b, subdev)

/*
 * Xclk 24MHz, max framerate 240fps
 * mipi_datarate per lane 400Mbps, 1-lane
 *
 * Register list converted from:
 *   OC0TB1B_0x6C_400x400_240fps_10bit.ini
 */
static const struct regval oc0tb1b_400x400_240fps_regs[] = {
	{0x0100, 0x00},
	{0x0103, 0x01},
	{0x0301, 0x69},
	{0x0305, 0xf0},
	{0x0306, 0x01},
	{0x0307, 0x00},
	{0x0326, 0x93},
	{0x0327, 0x0e},
	{0x0329, 0x01},
	{0x032f, 0x00},
	{0x0360, 0x01},
	{0x0361, 0x03},
	{0x3006, 0x04},
	{0x300d, 0x08},
	{0x3018, 0x78},
	{0x301a, 0xf0},
	{0x301c, 0x78},
	{0x3020, 0x00},
	{0x3022, 0x01},
	{0x3024, 0x00},
	{0x3037, 0xb4},
	{0x303c, 0x02},
	{0x303d, 0x02},
	{0x3106, 0x08},
	{0x3216, 0x00},
	{0x3217, 0x00},
	{0x3218, 0x00},
	{0x3219, 0x55},
	{0x3400, 0x0c},
	{0x3406, 0x08},
	{0x3408, 0x05},
	{0x340e, 0x70},
	{0x3410, 0x00},
	{0x3420, 0x00},
	{0x3421, 0x00},
	{0x3422, 0x80},
	{0x3423, 0x00},
	{0x3424, 0x10},
	{0x3425, 0x14},
	{0x3426, 0x51},
	{0x3500, 0x00},
	{0x3504, 0x48},
	{0x3501, 0x00},
	{0x3502, 0x40},
	{0x3506, 0x23},
	{0x3508, 0x01},
	{0x3509, 0x00},
	{0x350a, 0x01},
	{0x350b, 0x00},
	{0x350c, 0x00},
	{0x3510, 0x01},
	{0x3513, 0x01},
	{0x3516, 0x01},
	{0x3541, 0x00},
	{0x3542, 0x30},
	{0x351e, 0x00},
	{0x351f, 0x00},
	{0x3503, 0xa8},
	{0x3c00, 0x78},
	{0x3c01, 0x01},
	{0x3c02, 0x0f},
	{0x3c03, 0xe0},
	{0x3c05, 0x30},
	{0x3c07, 0x18},
	{0x3c0a, 0x00},
	{0x3c0b, 0x7f},
	{0x3c17, 0x01},
	{0x3c1a, 0x06},
	{0x3605, 0x00},
	{0x3606, 0x80},
	{0x360b, 0x48},
	{0x360c, 0x12},
	{0x360d, 0x08},
	{0x3612, 0x01},
	{0x3620, 0x09},
	{0x3622, 0x81},
	{0x3633, 0x48},
	{0x3634, 0x48},
	{0x3635, 0x48},
	{0x363a, 0x01},
	{0x3660, 0x42},
	{0x3661, 0x07},
	{0x3662, 0x80},
	{0x366e, 0x00},
	{0x3674, 0x00},
	{0x3677, 0x3f},
	{0x367a, 0x10},
	{0x367b, 0x03},
	{0x367d, 0x12},
	{0x367e, 0x2b},
	{0x3686, 0xfe},
	{0x3687, 0x2e},
	{0x3692, 0x06},
	{0x3693, 0x07},
	{0x3694, 0x00},
	{0x369f, 0x19},
	{0x36a0, 0x0b},
	{0x36a1, 0x1f},
	{0x36a2, 0x1e},
	{0x36a3, 0x0b},
	{0x36a4, 0x61},
	{0x36a5, 0x37},
	{0x36a6, 0x1f},
	{0x36a7, 0x1f},
	{0x3700, 0x09},
	{0x3701, 0x22},
	{0x3702, 0x02},
	{0x3703, 0x29},
	{0x3704, 0x1f},
	{0x3705, 0x09},
	{0x3706, 0x3e},
	{0x3707, 0x28},
	{0x370a, 0x25},
	{0x370b, 0x0a},
	{0x370d, 0x7e},
	{0x370e, 0x0b},
	{0x370f, 0x80},
	{0x3710, 0x00},
	{0x3713, 0x86},
	{0x3715, 0x03},
	{0x3716, 0x00},
	{0x3717, 0xa2},
	{0x373c, 0x0f},
	{0x373f, 0x4f},
	{0x375d, 0x35},
	{0x375f, 0x3c},
	{0x3761, 0x27},
	{0x3770, 0x36},
	{0x3778, 0x00},
	{0x377a, 0x30},
	{0x37a8, 0x00},
	{0x37a9, 0x00},
	{0x37d1, 0x0f},
	{0x37da, 0x44},
	{0x37db, 0x6b},
	{0x37dc, 0x60},
	{0x37df, 0x7d},
	{0x37e9, 0x01},
	{0x37e6, 0x82},
	{0x37e7, 0x05},
	{0x37eb, 0x01},
	{0x3800, 0x00},
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x00},
	{0x3804, 0x01},
	{0x3805, 0x9f},
	{0x3806, 0x01},
	{0x3807, 0x9f},
	{0x3808, 0x01},
	{0x3809, 0x90},
	{0x380a, 0x01},
	{0x380b, 0x90},
	{0x380c, 0x00},
	{0x380d, 0xdd},
	{0x380e, 0x02},
	{0x380f, 0x52},
	{0x3810, 0x00},
	{0x3811, 0x08},
	{0x3812, 0x00},
	{0x3813, 0x08},
	{0x3814, 0x11},
	{0x3815, 0x11},
	{0x3816, 0x02},
	{0x3817, 0x42},
	{0x3818, 0x02},
	{0x381c, 0x00},
	{0x381d, 0x00},
	{0x381e, 0x00},
	{0x381f, 0x00},
	{0x3819, 0x44},
	{0x3820, 0x40},
	{0x3821, 0x04},
	{0x3823, 0x04},
	{0x3824, 0x00},
	{0x3825, 0x00},
	{0x3826, 0x00},
	{0x3827, 0x00},
	{0x382a, 0x00},
	{0x382c, 0x08},
	{0x382d, 0x52},
	{0x382e, 0x01},
	{0x3830, 0x0f},
	{0x383e, 0x13},
	{0x3840, 0x00},
	{0x384a, 0xa2},
	{0x3858, 0x00},
	{0x3859, 0x00},
	{0x3860, 0x00},
	{0x3864, 0x12},
	{0x3861, 0x00},
	{0x3866, 0x08},
	{0x3867, 0x07},
	{0x38a0, 0x00},
	{0x38a5, 0x00},
	{0x38a6, 0x00},
	{0x38a7, 0x00},
	{0x38a8, 0x00},
	{0x3b1e, 0x09},
	{0x3b1f, 0x00},
	{0x3b20, 0xff},
	{0x3b21, 0x00},
	{0x3b22, 0x00},
	{0x3b23, 0x00},
	{0x3b24, 0x00},
	{0x3b25, 0x00},
	{0x3b26, 0x00},
	{0x3b27, 0x00},
	{0x3b28, 0x00},
	{0x3b29, 0x01},
	{0x3b2a, 0xb4},
	{0x3b2b, 0x00},
	{0x3b2c, 0x10},
	{0x3b2d, 0x05},
	{0x3b2e, 0xf2},
	{0x3b2f, 0x42},
	{0x3b31, 0x00},
	{0x3904, 0x00},
	{0x3905, 0x08},
	{0x3908, 0x92},
	{0x390a, 0xd7},
	{0x390f, 0x0e},
	{0x3910, 0xd0},
	{0x3911, 0x07},
	{0x3912, 0x80},
	{0x3913, 0x40},
	{0x3918, 0x00},
	{0x391a, 0x04},
	{0x391b, 0xfa},
	{0x391c, 0x0e},
	{0x391d, 0xd0},
	{0x391f, 0x08},
	{0x3920, 0x00},
	{0x3921, 0x00},
	{0x3924, 0x12},
	{0x3925, 0x00},
	{0x3926, 0x02},
	{0x3927, 0x00},
	{0x3928, 0x8d},
	{0x3929, 0x04},
	{0x392a, 0x7e},
	{0x392b, 0x0e},
	{0x392c, 0xaa},
	{0x392f, 0x02},
	{0x3931, 0x04},
	{0x3932, 0x00},
	{0x3933, 0x12},
	{0x3936, 0x0e},
	{0x3937, 0xaa},
	{0x393a, 0x0f},
	{0x393b, 0x6c},
	{0x3941, 0x86},
	{0x3943, 0xdb},
	{0x3949, 0x30},
	{0x394b, 0xb0},
	{0x394f, 0x00},
	{0x3951, 0x97},
	{0x3953, 0xf2},
	{0x3954, 0x07},
	{0x3955, 0xe0},
	{0x3956, 0x0a},
	{0x3957, 0x42},
	{0x3959, 0x97},
	{0x395b, 0xf2},
	{0x395c, 0x0b},
	{0x395d, 0xe4},
	{0x395e, 0x0e},
	{0x395f, 0x39},
	{0x3960, 0x04},
	{0x3961, 0x9c},
	{0x3962, 0x0e},
	{0x3963, 0x98},
	{0x3966, 0x03},
	{0x3967, 0x13},
	{0x3968, 0x00},
	{0x396a, 0x0e},
	{0x396b, 0xb0},
	{0x396e, 0x0a},
	{0x396f, 0x93},
	{0x3970, 0x0b},
	{0x3971, 0xcd},
	{0x3972, 0x0a},
	{0x3973, 0xb3},
	{0x3974, 0x0b},
	{0x3975, 0xad},
	{0x3976, 0x04},
	{0x3977, 0x60},
	{0x3978, 0x0a},
	{0x3979, 0x93},
	{0x397a, 0x0e},
	{0x397b, 0x98},
	{0x397d, 0xff},
	{0x397e, 0x38},
	{0x3a06, 0x08},
	{0x3a07, 0x10},
	{0x3a08, 0x0a},
	{0x3a09, 0x93},
	{0x3a0a, 0x0c},
	{0x3a0b, 0x1d},
	{0x3a0c, 0x0e},
	{0x3a0d, 0x98},
	{0x3a0e, 0x04},
	{0x3a0f, 0x60},
	{0x3a10, 0x0e},
	{0x3a11, 0xd0},
	{0x3a12, 0x04},
	{0x3a13, 0x60},
	{0x3a14, 0x0e},
	{0x3a15, 0xd0},
	{0x3a17, 0x0b},
	{0x3a18, 0x28},
	{0x3a19, 0x0c},
	{0x3a1a, 0x00},
	{0x3a1b, 0x0b},
	{0x3a1c, 0xf3},
	{0x3a1d, 0x0b},
	{0x3a1e, 0x7d},
	{0x3a1f, 0x0b},
	{0x3a20, 0xe7},
	{0x3a21, 0x0b},
	{0x3a22, 0x95},
	{0x3a23, 0x0a},
	{0x3a24, 0xb3},
	{0x3a25, 0x0b},
	{0x3a26, 0x9d},
	{0x3a28, 0x0b},
	{0x3a29, 0xad},
	{0x3a2a, 0x0b},
	{0x3a2b, 0xbd},
	{0x3a2c, 0x0b},
	{0x3a2d, 0xcd},
	{0x3a2e, 0x0a},
	{0x3a2f, 0x93},
	{0x3a30, 0x0c},
	{0x3a31, 0x00},
	{0x3a32, 0x0a},
	{0x3a33, 0x96},
	{0x3a39, 0x0a},
	{0x3a3a, 0x46},
	{0x3a3b, 0x0a},
	{0x3a3c, 0x93},
	{0x3a3d, 0x0e},
	{0x3a3e, 0x3d},
	{0x3a3f, 0x0e},
	{0x3a40, 0x98},
	{0x3a41, 0x0a},
	{0x3a42, 0x42},
	{0x3a43, 0x0a},
	{0x3a44, 0xac},
	{0x3a45, 0x0e},
	{0x3a46, 0x39},
	{0x3a47, 0x0e},
	{0x3a48, 0xaa},
	{0x3a49, 0xb3},
	{0x3a4a, 0x0a},
	{0x3a4b, 0xad},
	{0x3a4c, 0x0b},
	{0x3a4d, 0x93},
	{0x3a4e, 0x0a},
	{0x3a4f, 0xcd},
	{0x3a50, 0x0b},
	{0x3a52, 0x00},
	{0x3a53, 0x01},
	{0x3a54, 0x0f},
	{0x3a55, 0x78},
	{0x3a58, 0x18},
	{0x3a59, 0x08},
	{0x3a5a, 0xb3},
	{0x3a5b, 0x0a},
	{0x3a5c, 0xad},
	{0x3a5d, 0x0b},
	{0x3a5e, 0x93},
	{0x3a5f, 0x0a},
	{0x3a60, 0xcd},
	{0x3a61, 0x0b},
	{0x3a6a, 0x30},
	{0x3a6b, 0x00},
	{0x3a6c, 0x41},
	{0x3a6d, 0x01},
	{0x3a6e, 0x30},
	{0x3a6f, 0x00},
	{0x3a70, 0xb0},
	{0x3a71, 0x01},
	{0x3a72, 0x30},
	{0x3a73, 0x00},
	{0x3a74, 0x41},
	{0x3a75, 0x01},
	{0x3a76, 0x30},
	{0x3a77, 0x00},
	{0x3a78, 0xb0},
	{0x3a79, 0x01},
	{0x3a7a, 0xaa},
	{0x3a7b, 0xa0},
	{0x3a7c, 0x00},
	{0x3a85, 0x88},
	{0x3d8c, 0x70},
	{0x3d8d, 0x5d},
	{0x3f31, 0x07},
	{0x3f32, 0xe0},
	{0x3f33, 0xc4},
	{0x428e, 0x02},
	{0x4300, 0xff},
	{0x4580, 0x09},
	{0x4700, 0xf8},
	{0x4701, 0x44},
	{0x4702, 0x01},
	{0x4703, 0x00},
	{0x4800, 0x64},
	{0x4802, 0x00},
	{0x4805, 0x00},
	{0x4813, 0x00},
	{0x481b, 0x3c},
	{0x481f, 0x26},
	{0x4837, 0x10},
	{0x484b, 0x27},
	{0x4860, 0x00},
	{0x4861, 0xec},
	{0x4862, 0x00},
	{0x4863, 0x00},
	{0x4883, 0x00},
	{0x4884, 0x08},
	{0x4888, 0x90},
	{0x488b, 0x01},
	{0x488c, 0x00},
	{0x488e, 0x00},
	{0x4000, 0xf3},
	{0x4001, 0x6c},
	{0x4003, 0x40},
	{0x4008, 0x04},
	{0x4009, 0x0b},
	{0x4040, 0x04},
	{0x4041, 0x0b},
	{0x4043, 0x52},
	{0x4045, 0x52},
	{0x4047, 0x52},
	{0x4049, 0x52},
	{0x430b, 0x03},
	{0x430c, 0xff},
	{0x430d, 0x00},
	{0x430e, 0x00},
	{0x4509, 0x00},
	{0x450b, 0x83},
	{0x4604, 0x48},
	{0x5000, 0x17},
	{0x5001, 0x00},
	{0x5002, 0x3f},
	{0x5007, 0x01},
	{0x500a, 0x00},
	{0x5101, 0x00},
	{0x5180, 0x70},
	{0x5181, 0x14},
	{0x5182, 0x70},
	{0x5183, 0x53},
	{0x0100, 0x01},
	{REG_NULL, 0x00},
};

/* Global init registers (written once at power-on) */
static const struct regval oc0tb1b_global_regs[] = {
	{REG_NULL, 0x00},
};

static const struct oc0tb1b_mode supported_modes[] = {
	{
		.bus_fmt = MEDIA_BUS_FMT_SRGGB10_1X10,
		.width = 400,
		.height = 400,
		.max_fps = {
			.numerator = 10000,
			.denominator = 2400000,
		},
		.hts_def = 0x00dd,	/* 221 */
		.vts_def = 0x0252,	/* 594 */
		.exp_def = 0x0040,
		.reg_list = oc0tb1b_400x400_240fps_regs,
		.hdr_mode = 0,
		.xvclk_freq = OC0TB1B_XVCLK_FREQ,
		.link_freq_idx = 0,
	},
};

static const s64 link_freq_menu_items[] = {
	OC0TB1B_LINK_FREQ,
};

/* Test pattern menu */
static const char * const oc0tb1b_test_pattern_menu[] = {
	"Disabled",
	"Color Bar",
	"Random Data",
	"Solid Color",
};

static int oc0tb1b_read_reg(struct i2c_client *client, u16 reg, u32 len, u32 *val)
{
	struct i2c_msg msgs[2];
	u8 addr_buf[2] = { reg >> 8, reg & 0xff };
	u8 data_buf[2] = { 0 };
	int ret;

	if (!len || len > sizeof(data_buf))
		return -EINVAL;

	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = sizeof(addr_buf);
	msgs[0].buf = addr_buf;

	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = data_buf;

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = data_buf[0];
	if (len == 2)
		*val = (*val << 8) | data_buf[1];

	return 0;
}

static int oc0tb1b_write_reg(struct i2c_client *client, u16 reg, u32 len, u32 val)
{
	u8 buf[6];
	int i;
	int ret;

	if (len > 4)
		return -EINVAL;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;
	for (i = len; i > 0; i--) {
		buf[1 + i] = val & 0xff;
		val >>= 8;
	}

	ret = i2c_master_send(client, buf, 2 + len);
	if (ret != 2 + len) {
		dev_err(&client->dev, "failed to write reg 0x%04x\n", reg);
		return ret < 0 ? ret : -EIO;
	}

	return 0;
}

static int oc0tb1b_write_array(struct i2c_client *client, const struct regval *regs)
{
	int i;
	int ret;

	for (i = 0; regs[i].addr != REG_NULL; i++) {
		ret = oc0tb1b_write_reg(client, regs[i].addr,
				      OC0TB1B_REG_VALUE_08BIT, regs[i].val);
		if (ret)
			return ret;
		if (regs[i].addr == 0x0103 && regs[i].val == 0x01)
			usleep_range(5000, 10000);
	}

	return 0;
}

/* ---------- control helpers ---------- */

static int oc0tb1b_set_exposure(struct oc0tb1b *oc0tb1b, u32 exposure)
{
	int ret;

	ret  = oc0tb1b_write_reg(oc0tb1b->client, OC0TB1B_REG_EXPOSURE_H,
				 OC0TB1B_REG_VALUE_08BIT, (exposure >> 16) & 0xff);
	ret |= oc0tb1b_write_reg(oc0tb1b->client, OC0TB1B_REG_EXPOSURE_M,
				 OC0TB1B_REG_VALUE_08BIT, (exposure >> 8) & 0xff);
	ret |= oc0tb1b_write_reg(oc0tb1b->client, OC0TB1B_REG_EXPOSURE_L,
				 OC0TB1B_REG_VALUE_08BIT, exposure & 0xff);

	return ret;
}

static int oc0tb1b_set_gain(struct oc0tb1b *oc0tb1b, u32 gain)
{
	int ret;

	ret  = oc0tb1b_write_reg(oc0tb1b->client, OC0TB1B_REG_GAIN_FINE,
				 OC0TB1B_REG_VALUE_08BIT, 0x00);
	ret |= oc0tb1b_write_reg(oc0tb1b->client, OC0TB1B_REG_GAIN_COARSE,
				 OC0TB1B_REG_VALUE_08BIT, gain & 0x0f);

	return ret;
}

static int oc0tb1b_set_vts(struct oc0tb1b *oc0tb1b, u32 vts)
{
	int ret;

	ret  = oc0tb1b_write_reg(oc0tb1b->client, OC0TB1B_REG_VTS_H,
				 OC0TB1B_REG_VALUE_08BIT, (vts >> 8) & 0xff);
	ret |= oc0tb1b_write_reg(oc0tb1b->client, OC0TB1B_REG_VTS_L,
				 OC0TB1B_REG_VALUE_08BIT, vts & 0xff);

	return ret;
}

static int oc0tb1b_enable_test_pattern(struct oc0tb1b *oc0tb1b, u32 pattern)
{
	u32 val = 0;
	int ret;

	ret = oc0tb1b_read_reg(oc0tb1b->client, OC0TB1B_REG_TEST_PATTERN_EN,
			       OC0TB1B_REG_VALUE_08BIT, &val);
	if (ret)
		return ret;

	if (pattern)
		val |= OC0TB1B_TEST_PATTERN_ENABLE;
	else
		val &= ~OC0TB1B_TEST_PATTERN_ENABLE;

	ret  = oc0tb1b_write_reg(oc0tb1b->client, OC0TB1B_REG_TEST_PATTERN_EN,
				 OC0TB1B_REG_VALUE_08BIT, val);
	ret |= oc0tb1b_write_reg(oc0tb1b->client, OC0TB1B_REG_TEST_PATTERN,
				 OC0TB1B_REG_VALUE_08BIT,
				 pattern & OC0TB1B_TEST_PATTERN_MASK);
	return ret;
}

/* ---------- power ---------- */

static int __oc0tb1b_power_on(struct oc0tb1b *oc0tb1b)
{
	struct device *dev = &oc0tb1b->client->dev;
	int ret;

	if (!IS_ERR_OR_NULL(oc0tb1b->pins_default)) {
		ret = pinctrl_select_state(oc0tb1b->pinctrl,
					   oc0tb1b->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}

	ret = clk_set_rate(oc0tb1b->xvclk, oc0tb1b->cur_mode->xvclk_freq);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (%dHz)\n",
			 oc0tb1b->cur_mode->xvclk_freq);

	ret = clk_prepare_enable(oc0tb1b->xvclk);
	if (ret)
		return ret;

	if (oc0tb1b->pwdn_gpio)
		gpiod_set_value_cansleep(oc0tb1b->pwdn_gpio, 1);

	ret = regulator_bulk_enable(OC0TB1B_NUM_SUPPLIES, oc0tb1b->supplies);
	if (ret) {
		clk_disable_unprepare(oc0tb1b->xvclk);
		return ret;
	}

	if (oc0tb1b->reset_gpio) {
		gpiod_set_value_cansleep(oc0tb1b->reset_gpio, 1);
		usleep_range(5000, 10000);
		gpiod_set_value_cansleep(oc0tb1b->reset_gpio, 0);
	}

	if (oc0tb1b->pwdn_gpio)
		gpiod_set_value_cansleep(oc0tb1b->pwdn_gpio, 0);

	usleep_range(10000, 20000);

	return 0;
}

static void __oc0tb1b_power_off(struct oc0tb1b *oc0tb1b)
{
	if (oc0tb1b->pwdn_gpio)
		gpiod_set_value_cansleep(oc0tb1b->pwdn_gpio, 1);
	if (oc0tb1b->reset_gpio)
		gpiod_set_value_cansleep(oc0tb1b->reset_gpio, 1);

	regulator_bulk_disable(OC0TB1B_NUM_SUPPLIES, oc0tb1b->supplies);
	clk_disable_unprepare(oc0tb1b->xvclk);

	if (!IS_ERR_OR_NULL(oc0tb1b->pins_sleep)) {
		if (pinctrl_select_state(oc0tb1b->pinctrl,
					 oc0tb1b->pins_sleep) < 0)
			dev_err(&oc0tb1b->client->dev, "could not set sleep pins\n");
	}
}

/* ---------- stream ---------- */

static int __oc0tb1b_start_stream(struct oc0tb1b *oc0tb1b)
{
	int ret;

	ret = oc0tb1b_write_array(oc0tb1b->client,
				  oc0tb1b->cur_mode->reg_list);
	if (ret)
		return ret;

	return __v4l2_ctrl_handler_setup(&oc0tb1b->handler);
}

static int __oc0tb1b_stop_stream(struct oc0tb1b *oc0tb1b)
{
	return oc0tb1b_write_reg(oc0tb1b->client, OC0TB1B_REG_CTRL_MODE,
				 OC0TB1B_REG_VALUE_08BIT,
				 OC0TB1B_MODE_SW_STANDBY);
}

/* ---------- runtime PM ---------- */

static int oc0tb1b_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	return __oc0tb1b_power_on(to_oc0tb1b(sd));
}

static int oc0tb1b_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	__oc0tb1b_power_off(to_oc0tb1b(sd));
	return 0;
}

static const struct dev_pm_ops oc0tb1b_pm_ops = {
	SET_RUNTIME_PM_OPS(oc0tb1b_runtime_suspend, oc0tb1b_runtime_resume, NULL)
};

/* ---------- V4L2 subdev ops ---------- */

static int oc0tb1b_s_stream(struct v4l2_subdev *sd, int on)
{
	struct oc0tb1b *oc0tb1b = to_oc0tb1b(sd);
	struct i2c_client *client = oc0tb1b->client;
	int ret = 0;

	mutex_lock(&oc0tb1b->mutex);

	on = !!on;
	if (on == oc0tb1b->streaming)
		goto unlock;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock;
		}
		ret = __oc0tb1b_start_stream(oc0tb1b);
		if (ret) {
			pm_runtime_put(&client->dev);
			goto unlock;
		}
	} else {
		__oc0tb1b_stop_stream(oc0tb1b);
		pm_runtime_put(&client->dev);
	}

	oc0tb1b->streaming = on;

unlock:
	mutex_unlock(&oc0tb1b->mutex);
	return ret;
}

static int oc0tb1b_s_power(struct v4l2_subdev *sd, int on)
{
	struct oc0tb1b *oc0tb1b = to_oc0tb1b(sd);
	struct i2c_client *client = oc0tb1b->client;
	int ret = 0;

	mutex_lock(&oc0tb1b->mutex);

	if (oc0tb1b->power_on == !!on)
		goto unlock;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock;
		}
		ret = oc0tb1b_write_array(oc0tb1b->client, oc0tb1b_global_regs);
		if (ret) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock;
		}
		oc0tb1b->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		oc0tb1b->power_on = false;
	}

unlock:
	mutex_unlock(&oc0tb1b->mutex);
	return ret;
}

static int oc0tb1b_g_frame_interval(struct v4l2_subdev *sd,
				    struct v4l2_subdev_frame_interval *fi)
{
	struct oc0tb1b *oc0tb1b = to_oc0tb1b(sd);
	const struct oc0tb1b_mode *mode = oc0tb1b->cur_mode;

	fi->interval = mode->max_fps;
	return 0;
}

/* ---------- V4L2 controls ---------- */

static int oc0tb1b_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct oc0tb1b *oc0tb1b =
		container_of(ctrl->handler, struct oc0tb1b, handler);
	struct i2c_client *client = oc0tb1b->client;
	const struct oc0tb1b_mode *mode = oc0tb1b->cur_mode;
	s64 max;
	int ret = 0;
	u32 val;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		max = mode->height + ctrl->val - 6;
		__v4l2_ctrl_modify_range(oc0tb1b->exposure,
					 oc0tb1b->exposure->minimum,
					 max,
					 oc0tb1b->exposure->step,
					 min_t(s64, oc0tb1b->exposure->default_value, max));
		break;
	}

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ret = oc0tb1b_set_exposure(oc0tb1b, ctrl->val);
		break;
	case V4L2_CID_GAIN:
		ret = oc0tb1b_set_gain(oc0tb1b, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		ret = oc0tb1b_set_vts(oc0tb1b,
				    ctrl->val + mode->height);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = oc0tb1b_enable_test_pattern(oc0tb1b, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		/* enable the manual mirror/flip options first */
		ret = oc0tb1b_read_reg(client, OC0TB1B_REG_FLIP_MIRROR_EN,
				       OC0TB1B_REG_VALUE_08BIT, &val);
		if (ret)
			break;
		val |= OC0TB1B_FLIP_MASK | OC0TB1B_MIRROR_MASK;
		ret = oc0tb1b_write_reg(client, OC0TB1B_REG_FLIP_MIRROR_EN,
					OC0TB1B_REG_VALUE_08BIT, val);
		if (ret)
			break;

		ret = oc0tb1b_read_reg(client, OC0TB1B_REG_FLIP_MIRROR,
				       OC0TB1B_REG_VALUE_08BIT, &val);
		if (ret)
			break;

		if (ctrl->id == V4L2_CID_HFLIP) {
			if (ctrl->val)
				val |= OC0TB1B_MIRROR_MASK;
			else
				val &= ~OC0TB1B_MIRROR_MASK;
		} else {
			if (ctrl->val)
				val |= OC0TB1B_FLIP_MASK;
			else
				val &= ~OC0TB1B_FLIP_MASK;
		}
		ret = oc0tb1b_write_reg(client, OC0TB1B_REG_FLIP_MIRROR,
					OC0TB1B_REG_VALUE_08BIT, val);
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops oc0tb1b_ctrl_ops = {
	.s_ctrl = oc0tb1b_s_ctrl,
};

/* ---------- V4L2 pad ops ---------- */

static int oc0tb1b_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *st,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SRGGB10_1X10;
	return 0;
}

static int oc0tb1b_enum_frame_size(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *st,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	fse->min_width  = supported_modes[fse->index].width;
	fse->max_width  = supported_modes[fse->index].width;
	fse->min_height = supported_modes[fse->index].height;
	fse->max_height = supported_modes[fse->index].height;
	return 0;
}

static int oc0tb1b_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *st,
			   struct v4l2_subdev_format *fmt)
{
	struct oc0tb1b *oc0tb1b = to_oc0tb1b(sd);
	const struct oc0tb1b_mode *mode = oc0tb1b->cur_mode;
	struct v4l2_mbus_framefmt *mf = &fmt->format;

	mf->width       = mode->width;
	mf->height      = mode->height;
	mf->code        = mode->bus_fmt;
	mf->colorspace  = V4L2_COLORSPACE_SRGB;
	mf->field       = V4L2_FIELD_NONE;
	return 0;
}

static int oc0tb1b_s_fmt(struct v4l2_subdev *sd,
			 struct v4l2_subdev_state *st,
			 struct v4l2_subdev_format *fmt)
{
	return oc0tb1b_get_fmt(sd, st, fmt);
}

static int oc0tb1b_get_mbus_config(struct v4l2_subdev *sd,
				   unsigned int pad_id,
				   struct v4l2_mbus_config *config)
{
	config->type = V4L2_MBUS_CSI2_DPHY;
	config->bus.mipi_csi2.num_data_lanes = OC0TB1B_LANES;
	return 0;
}

/* ---------- rkmodule ioctl ---------- */

static void oc0tb1b_get_module_inf(struct oc0tb1b *oc0tb1b,
				   struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strscpy(inf->base.sensor, OC0TB1B_NAME, sizeof(inf->base.sensor));
	strscpy(inf->base.module,
		oc0tb1b->module_name ? oc0tb1b->module_name : "unknown",
		sizeof(inf->base.module));
	strscpy(inf->base.lens,
		oc0tb1b->len_name ? oc0tb1b->len_name : "default",
		sizeof(inf->base.lens));
}

static long oc0tb1b_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		oc0tb1b_get_module_inf(to_oc0tb1b(sd), (struct rkmodule_inf *)arg);
		return 0;
	default:
		return -ENOIOCTLCMD;
	}
}

/* ---------- subdev ops tables ---------- */

static const struct v4l2_subdev_video_ops oc0tb1b_video_ops = {
	.s_stream         = oc0tb1b_s_stream,
	.g_frame_interval = oc0tb1b_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops oc0tb1b_pad_ops = {
	.enum_mbus_code  = oc0tb1b_enum_mbus_code,
	.enum_frame_size = oc0tb1b_enum_frame_size,
	.get_fmt         = oc0tb1b_get_fmt,
	.set_fmt         = oc0tb1b_s_fmt,
	.get_mbus_config = oc0tb1b_get_mbus_config,
};

static const struct v4l2_subdev_core_ops oc0tb1b_core_ops = {
	.s_power = oc0tb1b_s_power,
	.ioctl   = oc0tb1b_ioctl,
};

static const struct v4l2_subdev_ops oc0tb1b_subdev_ops = {
	.core  = &oc0tb1b_core_ops,
	.video = &oc0tb1b_video_ops,
	.pad   = &oc0tb1b_pad_ops,
};

/* ---------- controls init ---------- */

static int oc0tb1b_initialize_controls(struct oc0tb1b *oc0tb1b)
{
	const struct oc0tb1b_mode *mode = oc0tb1b->cur_mode;
	struct v4l2_ctrl_handler *handler = &oc0tb1b->handler;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;

	ret = v4l2_ctrl_handler_init(handler, 9);
	if (ret)
		return ret;

	handler->lock = &oc0tb1b->mutex;

	oc0tb1b->link_freq = v4l2_ctrl_new_int_menu(handler, NULL,
						    V4L2_CID_LINK_FREQ,
						    ARRAY_SIZE(link_freq_menu_items) - 1,
						    0, link_freq_menu_items);
	if (oc0tb1b->link_freq)
		oc0tb1b->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	oc0tb1b->pixel_rate = v4l2_ctrl_new_std(handler, NULL,
						V4L2_CID_PIXEL_RATE,
						0, OC0TB1B_PIXEL_RATE, 1,
						OC0TB1B_PIXEL_RATE);
	if (oc0tb1b->pixel_rate)
		oc0tb1b->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	h_blank = (mode->hts_def > mode->width) ?
		  (mode->hts_def - mode->width) : 0;
	oc0tb1b->hblank = v4l2_ctrl_new_std(handler, NULL,
					    V4L2_CID_HBLANK,
					    h_blank, h_blank, 1, h_blank);
	if (oc0tb1b->hblank)
		oc0tb1b->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;
	oc0tb1b->vblank = v4l2_ctrl_new_std(handler, &oc0tb1b_ctrl_ops,
					    V4L2_CID_VBLANK,
					    vblank_def,
					    OC0TB1B_VTS_MAX - mode->height,
					    1, vblank_def);

	exposure_max = mode->vts_def - 6;
	oc0tb1b->exposure = v4l2_ctrl_new_std(handler, &oc0tb1b_ctrl_ops,
					      V4L2_CID_EXPOSURE,
					      OC0TB1B_EXPOSURE_MIN,
					      exposure_max,
					      OC0TB1B_EXPOSURE_STEP,
					      mode->exp_def);

	oc0tb1b->gain = v4l2_ctrl_new_std(handler, &oc0tb1b_ctrl_ops,
					  V4L2_CID_GAIN,
					  OC0TB1B_GAIN_MIN,
					  OC0TB1B_GAIN_MAX,
					  OC0TB1B_GAIN_STEP,
					  OC0TB1B_GAIN_DEFAULT);

	oc0tb1b->test_pattern = v4l2_ctrl_new_std_menu_items(handler,
						&oc0tb1b_ctrl_ops,
						V4L2_CID_TEST_PATTERN,
						ARRAY_SIZE(oc0tb1b_test_pattern_menu) - 1,
						0, 0,
						oc0tb1b_test_pattern_menu);

	v4l2_ctrl_new_std(handler, &oc0tb1b_ctrl_ops,
			  V4L2_CID_HFLIP, 0, 1, 1, 0);

	v4l2_ctrl_new_std(handler, &oc0tb1b_ctrl_ops,
			  V4L2_CID_VFLIP, 0, 1, 1, 0);

	if (handler->error) {
		ret = handler->error;
		dev_err(&oc0tb1b->client->dev,
			"Failed to init controls (%d)\n", ret);
		v4l2_ctrl_handler_free(handler);
		return ret;
	}

	oc0tb1b->subdev.ctrl_handler = handler;
	return 0;
}

/* ---------- sensor ID ---------- */

static int oc0tb1b_check_sensor_id(struct oc0tb1b *oc0tb1b)
{
	u32 chip_id_h = 0;
	u32 chip_id_l = 0;
	int ret;

	ret = oc0tb1b_write_reg(oc0tb1b->client, 0x0103,
				OC0TB1B_REG_VALUE_08BIT, 0x01);
	if (ret)
		return ret;
	usleep_range(10000, 20000);

	ret = oc0tb1b_read_reg(oc0tb1b->client, OC0TB1B_REG_CHIP_ID_H,
			       OC0TB1B_REG_VALUE_08BIT, &chip_id_h);
	if (ret)
		return ret;

	ret = oc0tb1b_read_reg(oc0tb1b->client, OC0TB1B_REG_CHIP_ID_L,
			       OC0TB1B_REG_VALUE_08BIT, &chip_id_l);
	if (ret)
		return ret;

	if (chip_id_h != OC0TB1B_CHIP_ID_H) {
		dev_err(&oc0tb1b->client->dev,
			"unexpected sensor id 0x%02x%02x\n",
			chip_id_h, chip_id_l);
		return -ENODEV;
	}

	dev_info(&oc0tb1b->client->dev, "detected sensor id 0x%02x%02x\n",
		 chip_id_h, chip_id_l);
	return 0;
}

/* ---------- probe / remove ---------- */

static int oc0tb1b_configure_regulators(struct oc0tb1b *oc0tb1b)
{
	unsigned int i;

	for (i = 0; i < OC0TB1B_NUM_SUPPLIES; i++)
		oc0tb1b->supplies[i].supply = oc0tb1b_supply_names[i];

	return devm_regulator_bulk_get(&oc0tb1b->client->dev,
				       OC0TB1B_NUM_SUPPLIES,
				       oc0tb1b->supplies);
}

static int oc0tb1b_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct oc0tb1b *oc0tb1b;
	int ret;
	int i, hdr_mode = 0;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		 DRIVER_VERSION >> 16,
		 (DRIVER_VERSION & 0xff00) >> 8,
		 DRIVER_VERSION & 0x00ff);

	oc0tb1b = devm_kzalloc(dev, sizeof(*oc0tb1b), GFP_KERNEL);
	if (!oc0tb1b)
		return -ENOMEM;

	oc0tb1b->client = client;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &oc0tb1b->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &oc0tb1b->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &oc0tb1b->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &oc0tb1b->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	/* Select default mode */
	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		if (hdr_mode == supported_modes[i].hdr_mode) {
			oc0tb1b->cur_mode = &supported_modes[i];
			break;
		}
	}
	if (i == ARRAY_SIZE(supported_modes))
		oc0tb1b->cur_mode = &supported_modes[0];

	oc0tb1b->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(oc0tb1b->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	oc0tb1b->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(oc0tb1b->reset_gpio)) {
		dev_warn(dev, "No reset GPIO\n");
		oc0tb1b->reset_gpio = NULL;
	}

	oc0tb1b->pwdn_gpio = devm_gpiod_get_optional(dev, "pwdn", GPIOD_OUT_HIGH);
	if (IS_ERR(oc0tb1b->pwdn_gpio)) {
		dev_warn(dev, "No pwdn GPIO\n");
		oc0tb1b->pwdn_gpio = NULL;
	}

	oc0tb1b->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(oc0tb1b->pinctrl)) {
		oc0tb1b->pins_default =
			pinctrl_lookup_state(oc0tb1b->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(oc0tb1b->pins_default)){
			dev_err(dev, "could not get default pinstate\n");
			return PTR_ERR(oc0tb1b->pins_default);
		}
		oc0tb1b->pins_sleep =
			pinctrl_lookup_state(oc0tb1b->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(oc0tb1b->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	} else {
		dev_info(dev, "no pinctrl\n");
	}

	ret = oc0tb1b_configure_regulators(oc0tb1b);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	mutex_init(&oc0tb1b->mutex);

	v4l2_i2c_subdev_init(&oc0tb1b->subdev, client, &oc0tb1b_subdev_ops);
	oc0tb1b->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	ret = oc0tb1b_initialize_controls(oc0tb1b);
	if (ret)
		goto err_destroy_mutex;

	ret = __oc0tb1b_power_on(oc0tb1b);
	if (ret)
		goto err_free_handler;

	ret = oc0tb1b_check_sensor_id(oc0tb1b);
	if (ret)
		goto err_power_off;

	oc0tb1b->subdev.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	oc0tb1b->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&oc0tb1b->subdev.entity, 1, &oc0tb1b->pad);
	if (ret < 0)
		goto err_power_off;

	ret = v4l2_async_register_subdev_sensor(&oc0tb1b->subdev);
	if (ret)
		goto err_clean_entity;

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	dev_info(dev, "OC0TB1B probed OK\n");
	return 0;

err_clean_entity:
	media_entity_cleanup(&oc0tb1b->subdev.entity);
err_power_off:
	__oc0tb1b_power_off(oc0tb1b);
err_free_handler:
	v4l2_ctrl_handler_free(&oc0tb1b->handler);
err_destroy_mutex:
	mutex_destroy(&oc0tb1b->mutex);
	return ret;
}

static void oc0tb1b_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct oc0tb1b *oc0tb1b = to_oc0tb1b(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_ctrl_handler_free(&oc0tb1b->handler);
	media_entity_cleanup(&sd->entity);
	mutex_destroy(&oc0tb1b->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__oc0tb1b_power_off(oc0tb1b);
	pm_runtime_set_suspended(&client->dev);
}

static const struct i2c_device_id oc0tb1b_id[] = {
	{ "oc0tb1b", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, oc0tb1b_id);

static const struct of_device_id oc0tb1b_of_match[] = {
	{ .compatible = "sony,oc0tb1b" },
	{ }
};
MODULE_DEVICE_TABLE(of, oc0tb1b_of_match);

static struct i2c_driver oc0tb1b_driver = {
	.driver = {
		.name = OC0TB1B_NAME,
		.pm = &oc0tb1b_pm_ops,
		.of_match_table = of_match_ptr(oc0tb1b_of_match),
	},
	.probe    = oc0tb1b_probe,
	.remove   = oc0tb1b_remove,
	.id_table = oc0tb1b_id,
};

module_i2c_driver(oc0tb1b_driver);
MODULE_DESCRIPTION("OC0TB1B camera sensor driver");
MODULE_LICENSE("GPL");
