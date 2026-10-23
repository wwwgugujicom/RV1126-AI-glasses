// SPDX-License-Identifier: GPL-2.0
/*
 * SC533HGS driver - adapted from SC233HGS
 *
 * Copyright (C) 2022 Rockchip Electronics Co., Ltd.
 *
 * V0.0X01.0X01 first version
 */

// #define DEBUG
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/rk-camera-module.h>
#include <linux/rk-preisp.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <linux/pinctrl/consumer.h>
#include "../platform/rockchip/isp/rkisp_tb_helper.h"

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x01)

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif

#define SC533HGS_LANES					2
#define SC533HGS_BITS_PER_SAMPLE		12
#define SC533HGS_LINK_FREQ_486		 486000000   // 972Mbps per lane / 2 = 486MHz link freq

#define PIXEL_RATE_WITH_486M_12BIT	(SC533HGS_LINK_FREQ_486 * 2LL * \
					SC533HGS_LANES / SC533HGS_BITS_PER_SAMPLE)

#define CHIP_ID				        0xce6f
#define SC533HGS_REG_CHIP_ID		0x3107

#define SC533HGS_REG_CTRL_MODE		0x2100
#define SC533HGS_MODE_SW_STANDBY	0x0
#define SC533HGS_MODE_STREAMING		BIT(0)

#define SC533HGS_REG_EXPOSURE_H		0x3e00
#define SC533HGS_REG_EXPOSURE_M		0x3e01
#define SC533HGS_REG_EXPOSURE_L		0x3e02
#define SC533HGS_EXPOSURE_MIN		1
#define SC533HGS_EXPOSURE_STEP		1
#define SC533HGS_EXPOSURE_MARGIN	10
#define SC533HGS_VTS_MAX			0x7fff

#define SC533HGS_REG_DIG_GAIN		0x3e06
#define SC533HGS_REG_DIG_FINE_GAIN	0x3e07
#define SC533HGS_REG_ANA_COARSE_GAIN	0x3e08
#define SC533HGS_REG_ANA_FINE_GAIN	0x3e09
#define SC533HGS_GAIN_MIN			0x0020
#define SC533HGS_GAIN_MAX			(4096)
#define SC533HGS_GAIN_STEP		1
#define SC533HGS_GAIN_DEFAULT		0x80

#define SC533HGS_REG_GROUP_HOLD     0x3817
#define SC533HGS_GROUP_HOLD_START	0x3235
#define SC533HGS_GROUP_HOLD_END		0x3236

#define SC533HGS_REG_TEST_PATTERN		0x4501
#define SC533HGS_TEST_PATTERN_BIT_MASK	BIT(3)

#define SC533HGS_REG_HTS_H		0x320C
#define SC533HGS_REG_HTS_L		0x320D

#define SC533HGS_REG_VTS_H		0x320E
#define SC533HGS_REG_VTS_L		0x320F

#define SC533HGS_FLIP_MIRROR_REG		0x3221

#define SC533HGS_FETCH_EXP_H(VAL)		(((VAL) >> 12) & 0xF)
#define SC533HGS_FETCH_EXP_M(VAL)		(((VAL) >> 4) & 0xFF)
#define SC533HGS_FETCH_EXP_L(VAL)		(((VAL) & 0xF) << 4)

#define SC533HGS_FETCH_AGAIN_H(VAL)	(((VAL) >> 8) & 0x03)
#define SC533HGS_FETCH_AGAIN_L(VAL)	((VAL) & 0xFF)

#define SC533HGS_FETCH_MIRROR(VAL, ENABLE) (ENABLE ? VAL | 0x06 : VAL & 0xf9)
#define SC533HGS_FETCH_FLIP(VAL, ENABLE)  (ENABLE ? VAL | 0x60 : VAL & 0x9f)

#define REG_DELAY			0xFFFE
#define REG_NULL			0xFFFF

#define SC533HGS_REG_VALUE_08BIT		1
#define SC533HGS_REG_VALUE_16BIT		2
#define SC533HGS_REG_VALUE_24BIT		3

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"
#define SC533HGS_NAME			"sc533hgs"

static const char * const SC533HGS_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define SC533HGS_NUM_SUPPLIES ARRAY_SIZE(SC533HGS_supply_names)

struct regval {
	u16 addr;
	u8 val;
};

struct SC533HGS_mode {
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
	u32 vc[PAD_MAX];
};

struct SC533HGS {
	struct i2c_client	*client;
	struct clk		    *xvclk;
	struct gpio_desc	*reset_gpio;
	struct regulator_bulk_data supplies[SC533HGS_NUM_SUPPLIES];

	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pins_default;
	struct pinctrl_state	*pins_sleep;

	struct v4l2_subdev	subdev;
	struct media_pad	pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl	*exposure;
	struct v4l2_ctrl	*anal_gain;
	struct v4l2_ctrl	*digi_gain;
	struct v4l2_ctrl	*hblank;
	struct v4l2_ctrl	*vblank;
	struct v4l2_ctrl	*pixel_rate;
	struct v4l2_ctrl	*link_freq;
	struct v4l2_ctrl	*test_pattern;
	struct mutex		mutex;
	bool			streaming;
	bool			power_on;
	const struct SC533HGS_mode *cur_mode;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
	u32			cur_vts;
	bool			has_init_exp;
	bool			is_thunderboot;
	bool			is_first_streamoff;
	bool			auto_exp;
	struct preisp_hdrae_exp_s init_hdrae_exp;
};

#define to_SC533HGS(sd) container_of(sd, struct SC533HGS, subdev)

/*
 * Xclk 27Mhz
 */
static const struct regval SC533HGS_global_regs[] = {
	{REG_NULL, 0x00},
};

/*
 * Xclk 27Mhz
 * max_framerate 30fps
 * mipi_datarate per lane 972Mbps, 2lane
 * 2592x1944 12bit
 */
static const struct regval SC533HGS_linear_12_2592x1944_30fps_regs[] = {
	{0x2100, 0x00},	// streaming off
	{0x36e9, 0x80},	// PLL unlock
	{0x37f9, 0x80},	// PLL2 unlock
	{0x300c, 0x44},
	{0x3018, 0x3a},
	{0x3019, 0x0c},
	{0x301f, 0x17},
	{0x3031, 0x0c},	// 12bit mode
	{0x3045, 0x01},
	{0x304a, 0x02},
	{0x3062, 0x00},
	{0x320c, 0x06},	// HTS_H
	{0x320d, 0x40},	// HTS_L (HTS=1600)
	{0x320e, 0x07},	// VTS_H
	{0x320f, 0xd0},	// VTS_L (VTS=2000)
	{0x321f, 0x0b},
	{0x322f, 0x00},
	{0x3231, 0x00},
	{0x3250, 0x03},
	{0x325f, 0x25},
	{0x32c0, 0x07},
	{0x3301, 0x20},
	{0x3303, 0x50},
	{0x3304, 0x40},
	{0x3306, 0x58},
	{0x3309, 0x50},
	{0x330b, 0xc0},
	{0x3310, 0x48},
	{0x3312, 0x24},
	{0x3314, 0x30},
	{0x3316, 0x24},
	{0x3317, 0x74},
	{0x331d, 0x32},
	{0x331f, 0x02},
	{0x3320, 0xf1},
	{0x333b, 0x30},
	{0x3352, 0x16},
	{0x3357, 0x18},
	{0x3361, 0x20},
	{0x3385, 0x31},
	{0x3387, 0x41},
	{0x33b0, 0x00},
	{0x33ef, 0x04},
	{0x33f8, 0x01},
	{0x33fa, 0x01},
	{0x34af, 0x02},
	{0x34f2, 0x00},
	{0x361a, 0x90},
	{0x3630, 0x80},
	{0x3633, 0x54},
	{0x363c, 0x0f},
	{0x363f, 0x08},
	{0x3648, 0x0d},
	{0x3670, 0xf0},
	{0x3671, 0xc8},
	{0x3672, 0xaa},
	{0x367c, 0x54},
	{0x367d, 0x54},
	{0x367e, 0x54},
	{0x36c0, 0x08},
	{0x36c1, 0x38},
	{0x36c6, 0x18},
	{0x36c7, 0x38},
	{0x36ea, 0x09},
	{0x36eb, 0x1e},
	{0x36ec, 0x43},
	{0x36ed, 0x14},
	{0x3718, 0x04},
	{0x3719, 0x04},
	{0x3722, 0x08},
	{0x3724, 0xa1},
	{0x3728, 0xe0},
	{0x372a, 0x08},
	{0x3778, 0xc8},
	{0x3779, 0xd8},
	{0x377a, 0xd8},
	{0x3798, 0x08},
	{0x3799, 0x08},
	{0x379a, 0x08},
	{0x37c4, 0x08},
	{0x37c5, 0x38},
	{0x37d4, 0x08},
	{0x37d5, 0x38},
	{0x37fa, 0x08},
	{0x37fb, 0x52},
	{0x37fc, 0x20},
	{0x37fd, 0x14},
	{0x3900, 0x1d},
	{0x3901, 0x04},
	{0x3903, 0x20},
	{0x3904, 0x0a},
	{0x391f, 0x41},
	{0x393f, 0x80},
	{0x3940, 0x00},
	{0x3941, 0x00},
	{0x3942, 0x00},
	{0x3943, 0x80},
	{0x3944, 0x80},
	{0x3945, 0x81},
	{0x3946, 0x81},
	{0x3e00, 0x00},	// exposure H
	{0x3e01, 0x7a},	// exposure M
	{0x3e02, 0x40},	// exposure L
	{0x3e03, 0x0b},
	{0x3e08, 0x00},
	{0x3e09, 0x20},	// analog gain
	{0x3e15, 0x00},
	{0x3e1c, 0x00},
	{0x3e1d, 0x00},
	{0x3e8e, 0x00},
	{0x3f16, 0x20},
	{0x4314, 0x03},
	{0x4315, 0x1c},
	{0x4330, 0x50},
	{0x4331, 0x30},
	{0x4338, 0xae},
	{0x4350, 0x30},
	{0x4360, 0x0f},
	{0x4362, 0xc0},
	{0x4364, 0xd0},
	{0x4365, 0x08},
	{0x4366, 0x18},
	{0x4368, 0x68},
	{0x436a, 0x78},
	{0x436b, 0x18},
	{0x436c, 0x38},
	{0x436d, 0x38},
	{0x436f, 0xe8},
	{0x4370, 0x28},
	{0x4371, 0x20},
	{0x4372, 0x08},
	{0x4373, 0x18},
	{0x437a, 0x18},
	{0x437b, 0x38},
	{0x4380, 0x28},
	{0x4381, 0x20},
	{0x4382, 0x18},
	{0x4383, 0x08},
	{0x4384, 0x18},
	{0x4385, 0x38},
	{0x4402, 0x02},
	{0x4403, 0x0a},
	{0x4404, 0x1d},
	{0x4405, 0x26},
	{0x4408, 0x00},
	{0x4409, 0x00},
	{0x440a, 0x03},
	{0x440b, 0xff},
	{0x440c, 0x30},
	{0x440d, 0x30},
	{0x440e, 0x24},
	{0x440f, 0x3c},
	{0x4412, 0x01},
	{0x4424, 0x01},
	{0x4503, 0x20},
	{0x4509, 0x20},
	{0x450a, 0x00},
	{0x450d, 0x13},
	{0x4837, 0x10},
	{0x4b0a, 0x00},
	{0x5000, 0x38},
	{0x5002, 0x00},
	{0x502e, 0xa1},
	{0x5034, 0x01},
	{0x5104, 0x14},
	{0x5105, 0x10},
	{0x5106, 0x04},
	{0x5107, 0x59},
	{0x510d, 0x00},
	{0x5302, 0x00},
	{0x5784, 0x08},
	{0x5785, 0x03},
	{0x5787, 0x0a},
	{0x5788, 0x0a},
	{0x5789, 0x08},
	{0x578a, 0x0a},
	{0x578b, 0x0a},
	{0x578c, 0x08},
	{0x578d, 0x40},
	{0x5790, 0x08},
	{0x5791, 0x04},
	{0x5792, 0x04},
	{0x5793, 0x08},
	{0x5794, 0x04},
	{0x5795, 0x04},
	{0x57aa, 0x2a},
	{0x57ab, 0x7f},
	{0x57ac, 0x00},
	{0x57ad, 0x00},
	{0x36e9, 0x04},	// PLL lock
	{0x37f9, 0x14},	// PLL2 lock
	{0x304a, 0x02},
	{0x4408, 0x00},
	{0x4409, 0x00},
	{0x440a, 0x03},
	{0x440b, 0xff},
	{0x4412, 0x01},
	{0x4424, 0x01},
	{0x4402, 0x02},
	{0x4403, 0x0a},
	{0x4404, 0x1d},
	{0x4405, 0x26},
	{0x440c, 0x30},
	{0x440d, 0x30},
	{0x440e, 0x24},
	{0x440f, 0x3c},
	{REG_NULL, 0x00},
};

static const struct SC533HGS_mode supported_modes[] = {
	{
		.width = 2592,
		.height = 1944,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x07a4,
		.hts_def = 0x0640,
		.vts_def = 0x07D0,
		.bus_fmt = MEDIA_BUS_FMT_SBGGR12_1X12,
		.reg_list = SC533HGS_linear_12_2592x1944_30fps_regs,
		.hdr_mode = NO_HDR,
		.xvclk_freq = 27000000,
		.link_freq_idx = 0,
		.vc[PAD0] = 0,
	},
};

static const s64 link_freq_menu_items[] = {
	SC533HGS_LINK_FREQ_486,
};

static const char * const SC533HGS_test_pattern_menu[] = {
	"Disabled",
	"Incremental Pattern",
};

/* Write registers up to 4 at a time */
static int SC533HGS_write_reg(struct i2c_client *client, u16 reg,
			    u32 len, u32 val)
{
	u32 buf_i, val_i;
	u8 buf[6];
	u8 *val_p;
	__be32 val_be;

	if (len > 4)
		return -EINVAL;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	val_be = cpu_to_be32(val);
	val_p = (u8 *)&val_be;
	buf_i = 2;
	val_i = 4 - len;

	while (val_i < 4)
		buf[buf_i++] = val_p[val_i++];

	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;
	return 0;
}

static int SC533HGS_write_array(struct i2c_client *client,
			       const struct regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++)
		ret = SC533HGS_write_reg(
				client,
				regs[i].addr,
				SC533HGS_REG_VALUE_08BIT,
				regs[i].val);

	return ret;
}

/* Read registers up to 4 at a time */
static int SC533HGS_read_reg(struct i2c_client *client, u16 reg, unsigned int len,
			    u32 *val)
{
	struct i2c_msg msgs[2];
	u8 *data_be_p;
	__be32 data_be = 0;
	__be16 reg_addr_be = cpu_to_be16(reg);
	int ret;

	if (len > 4 || !len)
		return -EINVAL;

	data_be_p = (u8 *)&data_be;
	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = (u8 *)&reg_addr_be;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_be_p[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = be32_to_cpu(data_be);

	return 0;
}

static int SC533HGS_set_gain_reg(struct SC533HGS *SC533HGS, u32 gain)
{
	u32 coarse_again = 0, fine_again = 0x20;
	u32 coarse_dgain = 0, fine_dgain = 0x80;
	u32 analog_gain, digital_gain;
	int ret = 0;

	gain = clamp(gain, (u32)SC533HGS_GAIN_MIN, (u32)SC533HGS_GAIN_MAX);

	/*
	 * Gain control follows the datasheet real-gain mode:
	 * analog gain uses 0x3e08/0x3e09, digital gain uses 0x3e06/0x3e07.
	 * The V4L2 control uses 0x20 as 1x gain.
	 */
	analog_gain = min(gain, 504U);
	if (analog_gain < 64) {
		coarse_again = 0x00;
		fine_again = analog_gain;
	} else if (analog_gain < 128) {
		coarse_again = 0x01;
		fine_again = analog_gain / 2;
	} else if (analog_gain < 256) {
		coarse_again = 0x03;
		fine_again = analog_gain / 4;
	} else {
		coarse_again = 0x07;
		fine_again = analog_gain / 8;
	}
	fine_again = clamp(fine_again, 0x20U, 0x3fU);

	digital_gain = DIV_ROUND_UP(gain * 32, analog_gain);
	if (digital_gain < 64) {
		coarse_dgain = 0x00;
		fine_dgain = digital_gain * 4;
	} else if (digital_gain < 128) {
		coarse_dgain = 0x01;
		fine_dgain = digital_gain * 2;
	} else if (digital_gain < 256) {
		coarse_dgain = 0x03;
		fine_dgain = digital_gain;
	} else {
		coarse_dgain = 0x07;
		fine_dgain = digital_gain / 2;
	}
	fine_dgain = clamp(fine_dgain, 0x80U, 0xf8U);
	fine_dgain = fine_dgain / 8 * 8;

	dev_dbg(&SC533HGS->client->dev,
		"total_gain: 0x%x, a_gain: 0x%x, a_fine_gain: 0x%x, d_gain: 0x%x, d_fine_gain: 0x%x\n",
		gain, coarse_again, fine_again, coarse_dgain, fine_dgain);

	ret = SC533HGS_write_reg(SC533HGS->client,
				SC533HGS_REG_DIG_GAIN,
				SC533HGS_REG_VALUE_08BIT,
				coarse_dgain);
	ret |= SC533HGS_write_reg(SC533HGS->client,
				 SC533HGS_REG_DIG_FINE_GAIN,
				 SC533HGS_REG_VALUE_08BIT,
				 fine_dgain);
	ret |= SC533HGS_write_reg(SC533HGS->client,
				 SC533HGS_REG_ANA_COARSE_GAIN,
				 SC533HGS_REG_VALUE_08BIT,
				 coarse_again);
	ret |= SC533HGS_write_reg(SC533HGS->client,
				 SC533HGS_REG_ANA_FINE_GAIN,
				 SC533HGS_REG_VALUE_08BIT,
				 fine_again);

	return ret;
}

static int SC533HGS_get_reso_dist(const struct SC533HGS_mode *mode,
				 struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct SC533HGS_mode *
SC533HGS_find_best_fit(struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = SC533HGS_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int SC533HGS_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *sd_state,
			   struct v4l2_subdev_format *fmt)
{
	struct SC533HGS *SC533HGS = to_SC533HGS(sd);
	const struct SC533HGS_mode *mode;
	s64 h_blank, vblank_def;
	u64 dst_link_freq = 0;
	u64 dst_pixel_rate = 0;

	mutex_lock(&SC533HGS->mutex);

	mode = SC533HGS_find_best_fit(fmt);
	fmt->format.code = mode->bus_fmt;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, sd_state, fmt->pad) = fmt->format;
#else
		mutex_unlock(&SC533HGS->mutex);
		return -ENOTTY;
#endif
	} else {
		SC533HGS->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(SC533HGS->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(SC533HGS->vblank, vblank_def,
					 SC533HGS_VTS_MAX - mode->height,
					 1, vblank_def);
		dst_link_freq = mode->link_freq_idx;
		dst_pixel_rate = (u32)link_freq_menu_items[mode->link_freq_idx] /
						 SC533HGS_BITS_PER_SAMPLE * 2 * SC533HGS_LANES;
		__v4l2_ctrl_s_ctrl_int64(SC533HGS->pixel_rate,
					 dst_pixel_rate);
		__v4l2_ctrl_s_ctrl(SC533HGS->link_freq,
				   dst_link_freq);
	}

	mutex_unlock(&SC533HGS->mutex);

	return 0;
}

static int SC533HGS_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *sd_state,
			   struct v4l2_subdev_format *fmt)
{
	struct SC533HGS *SC533HGS = to_SC533HGS(sd);
	const struct SC533HGS_mode *mode = SC533HGS->cur_mode;

	mutex_lock(&SC533HGS->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, sd_state, fmt->pad);
#else
		mutex_unlock(&SC533HGS->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = mode->bus_fmt;
		fmt->format.field = V4L2_FIELD_NONE;
		if (fmt->pad < PAD_MAX && mode->hdr_mode != NO_HDR)
			fmt->reserved[0] = mode->vc[fmt->pad];
		else
			fmt->reserved[0] = mode->vc[PAD0];
	}
	mutex_unlock(&SC533HGS->mutex);

	return 0;
}

static int SC533HGS_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	struct SC533HGS *SC533HGS = to_SC533HGS(sd);

	if (code->index != 0)
		return -EINVAL;
	code->code = SC533HGS->cur_mode->bus_fmt;

	return 0;
}

static int SC533HGS_enum_frame_sizes(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *sd_state,
				    struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fse->code != supported_modes[0].bus_fmt)
		return -EINVAL;

	fse->min_width  = supported_modes[0].width;
	fse->max_width  = supported_modes[0].width;
	fse->max_height = supported_modes[0].height;
	fse->min_height = supported_modes[0].height;

	return 0;
}

static int SC533HGS_enable_test_pattern(struct SC533HGS *SC533HGS, u32 pattern)
{
	u32 val = 0;
	int ret = 0;

	ret = SC533HGS_read_reg(SC533HGS->client, SC533HGS_REG_TEST_PATTERN,
			       SC533HGS_REG_VALUE_08BIT, &val);
	if (pattern)
		val |= SC533HGS_TEST_PATTERN_BIT_MASK;
	else
		val &= ~SC533HGS_TEST_PATTERN_BIT_MASK;

	ret |= SC533HGS_write_reg(SC533HGS->client, SC533HGS_REG_TEST_PATTERN,
				 SC533HGS_REG_VALUE_08BIT, val);
	return ret;
}

static int SC533HGS_g_frame_interval(struct v4l2_subdev *sd,
				    struct v4l2_subdev_frame_interval *fi)
{
	struct SC533HGS *SC533HGS = to_SC533HGS(sd);
	const struct SC533HGS_mode *mode = SC533HGS->cur_mode;

	fi->interval = mode->max_fps;

	return 0;
}

static int SC533HGS_g_mbus_config(struct v4l2_subdev *sd,
				unsigned int pad_id,
				struct v4l2_mbus_config *config)
{

	config->type = V4L2_MBUS_CSI2_DPHY;
	config->bus.mipi_csi2.num_data_lanes = SC533HGS_LANES;

	return 0;
}

static void SC533HGS_get_module_inf(struct SC533HGS *SC533HGS,
				   struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strscpy(inf->base.sensor, SC533HGS_NAME, sizeof(inf->base.sensor));
	strscpy(inf->base.module, SC533HGS->module_name,
		sizeof(inf->base.module));
	strscpy(inf->base.lens, SC533HGS->len_name, sizeof(inf->base.lens));
}

static long SC533HGS_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct SC533HGS *SC533HGS = to_SC533HGS(sd);
	struct rkmodule_hdr_cfg *hdr;
	u32 i, h, w;
	long ret = 0;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		SC533HGS_get_module_inf(SC533HGS, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		hdr->esp.mode = HDR_NORMAL_VC;
		hdr->hdr_mode = SC533HGS->cur_mode->hdr_mode;
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		w = SC533HGS->cur_mode->width;
		h = SC533HGS->cur_mode->height;
		for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
			if (w == supported_modes[i].width &&
			    h == supported_modes[i].height &&
			    supported_modes[i].hdr_mode == hdr->hdr_mode) {
				SC533HGS->cur_mode = &supported_modes[i];
				break;
			}
		}
		if (i == ARRAY_SIZE(supported_modes)) {
			dev_err(&SC533HGS->client->dev,
				"not find hdr mode:%d %dx%d config\n",
				hdr->hdr_mode, w, h);
			ret = -EINVAL;
		} else {
			w = SC533HGS->cur_mode->hts_def - SC533HGS->cur_mode->width;
			h = SC533HGS->cur_mode->vts_def - SC533HGS->cur_mode->height;
			__v4l2_ctrl_modify_range(SC533HGS->hblank, w, w, 1, w);
			__v4l2_ctrl_modify_range(SC533HGS->vblank, h,
						 SC533HGS_VTS_MAX - SC533HGS->cur_mode->height, 1, h);
		}
		break;
	case PREISP_CMD_SET_HDRAE_EXP:
		break;
	case RKMODULE_SET_QUICK_STREAM:

		stream = *((u32 *)arg);

		if (stream)
			ret = SC533HGS_write_reg(SC533HGS->client, SC533HGS_REG_CTRL_MODE,
				 SC533HGS_REG_VALUE_08BIT, SC533HGS_MODE_STREAMING);
		else
			ret = SC533HGS_write_reg(SC533HGS->client, SC533HGS_REG_CTRL_MODE,
				 SC533HGS_REG_VALUE_08BIT, SC533HGS_MODE_SW_STANDBY);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long SC533HGS_compat_ioctl32(struct v4l2_subdev *sd,
				   unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_hdr_cfg *hdr;
	struct preisp_hdrae_exp_s *hdrae;
	long ret;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = SC533HGS_ioctl(sd, cmd, inf);
		if (!ret) {
			if (copy_to_user(up, inf, sizeof(*inf)))
				ret = -EFAULT;
		}
		kfree(inf);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = kzalloc(sizeof(*hdr), GFP_KERNEL);
		if (!hdr) {
			ret = -ENOMEM;
			return ret;
		}

		ret = SC533HGS_ioctl(sd, cmd, hdr);
		if (!ret) {
			if (copy_to_user(up, hdr, sizeof(*hdr)))
				ret = -EFAULT;
		}
		kfree(hdr);
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr = kzalloc(sizeof(*hdr), GFP_KERNEL);
		if (!hdr) {
			ret = -ENOMEM;
			return ret;
		}

		ret = copy_from_user(hdr, up, sizeof(*hdr));
		if (!ret)
			ret = SC533HGS_ioctl(sd, cmd, hdr);
		else
			ret = -EFAULT;
		kfree(hdr);
		break;
	case PREISP_CMD_SET_HDRAE_EXP:
		hdrae = kzalloc(sizeof(*hdrae), GFP_KERNEL);
		if (!hdrae) {
			ret = -ENOMEM;
			return ret;
		}

		ret = copy_from_user(hdrae, up, sizeof(*hdrae));
		if (!ret)
			ret = SC533HGS_ioctl(sd, cmd, hdrae);
		else
			ret = -EFAULT;
		kfree(hdrae);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		ret = copy_from_user(&stream, up, sizeof(u32));
		if (!ret)
			ret = SC533HGS_ioctl(sd, cmd, &stream);
		else
			ret = -EFAULT;
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif


static int __SC533HGS_start_stream(struct SC533HGS *SC533HGS)
{
	struct i2c_client *client = SC533HGS->client;
	const struct SC533HGS_mode *mode __attribute__((unused)) = SC533HGS->cur_mode;

	int ret;
	if (!SC533HGS->is_thunderboot) {
		ret = SC533HGS_write_array(SC533HGS->client, SC533HGS->cur_mode->reg_list);
		if (ret)
			return ret;
		ret = __v4l2_ctrl_handler_setup(&SC533HGS->ctrl_handler);
		if (ret)
			return ret;
		if (SC533HGS->has_init_exp && SC533HGS->cur_mode->hdr_mode != NO_HDR)
		{
			ret = SC533HGS_ioctl(&SC533HGS->subdev, PREISP_CMD_SET_HDRAE_EXP,
				&SC533HGS->init_hdrae_exp);

			if (ret) {
				dev_err(&SC533HGS->client->dev,
					"init exp fail in hdr mode\n");
				return ret;
			}
		}
	}
	/* FSYNC Trigger Mode for quad-camera sync:
	 * 0x3037 = 0x0010: bit[4]=1 enable trigger mode
	 * 0x3222 = 0x0002: gate mode (level control)
	 * 0x3224 = 0x0001: EFSYNC active HIGH gate (match SC132)
	 *
	 * DISABLED: sensor free-runs (no external FSYNC needed).
	 * FSYNC not wired/running on this board => capture hung waiting
	 * for trigger. Remove trigger mode so a frame is produced.
	 */
	SC533HGS_write_reg(SC533HGS->client,
			0x3037, SC533HGS_REG_VALUE_16BIT, 0x0000);
	dev_info(&client->dev,
		 "SC533HGS: FSYNC trigger DISABLED (free-run)\n");

	ret = SC533HGS_write_reg(SC533HGS->client,
				SC533HGS_REG_CTRL_MODE,
				SC533HGS_REG_VALUE_08BIT,
				SC533HGS_MODE_STREAMING);

	return ret;
}


static int __SC533HGS_stop_stream(struct SC533HGS *SC533HGS)
{
	SC533HGS->has_init_exp = false;
	if (SC533HGS->is_thunderboot) {
		SC533HGS->is_first_streamoff = true;
		pm_runtime_put(&SC533HGS->client->dev);
	}
	return SC533HGS_write_reg(SC533HGS->client,
				SC533HGS_REG_CTRL_MODE,
				SC533HGS_REG_VALUE_08BIT,
				SC533HGS_MODE_SW_STANDBY);
}

static int __SC533HGS_power_on(struct SC533HGS *SC533HGS);

static int SC533HGS_s_stream(struct v4l2_subdev *sd, int on)
{
	struct SC533HGS *SC533HGS = to_SC533HGS(sd);
	struct i2c_client *client = SC533HGS->client;
	int ret = 0;

	mutex_lock(&SC533HGS->mutex);
	on = !!on;
	if (on == SC533HGS->streaming)
		goto unlock_and_return;

	if (on) {
		if (SC533HGS->is_thunderboot && rkisp_tb_get_state() == RKISP_TB_NG) {
			SC533HGS->is_thunderboot = false;
			__SC533HGS_power_on(SC533HGS);
		}
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}
		ret = __SC533HGS_start_stream(SC533HGS);
		if (ret) {
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__SC533HGS_stop_stream(SC533HGS);
		pm_runtime_put(&client->dev);
	}

	SC533HGS->streaming = on;

unlock_and_return:
	mutex_unlock(&SC533HGS->mutex);
	return ret;
}

static int SC533HGS_s_power(struct v4l2_subdev *sd, int on)
{
	struct SC533HGS *SC533HGS = to_SC533HGS(sd);
	struct i2c_client *client = SC533HGS->client;
	int ret = 0;

	mutex_lock(&SC533HGS->mutex);

	if (SC533HGS->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		if (!SC533HGS->is_thunderboot) {
			ret = SC533HGS_write_array(SC533HGS->client, SC533HGS_global_regs);
			if (ret) {
				v4l2_err(sd, "could not set init registers\n");
				pm_runtime_put_noidle(&client->dev);
				goto unlock_and_return;
			}
		}

		SC533HGS->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		SC533HGS->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&SC533HGS->mutex);

	return ret;
}

static inline u32 SC533HGS_cal_delay(u32 cycles, struct SC533HGS *SC533HGS)
{
	return DIV_ROUND_UP(cycles, SC533HGS->cur_mode->xvclk_freq / 1000 / 1000);
}

static int __SC533HGS_power_on(struct SC533HGS *SC533HGS)
{
	int ret;
	u32 delay_us;
	struct device *dev = &SC533HGS->client->dev;

	if (!IS_ERR_OR_NULL(SC533HGS->pins_default)) {
		ret = pinctrl_select_state(SC533HGS->pinctrl,
					   SC533HGS->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}
	ret = clk_set_rate(SC533HGS->xvclk, SC533HGS->cur_mode->xvclk_freq);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (%dHz)\n", SC533HGS->cur_mode->xvclk_freq);
	if (clk_get_rate(SC533HGS->xvclk) != SC533HGS->cur_mode->xvclk_freq)
		dev_warn(dev, "xvclk mismatched, modes are based on %dHz\n",
			 SC533HGS->cur_mode->xvclk_freq);
	ret = clk_prepare_enable(SC533HGS->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}

	if (SC533HGS->is_thunderboot)
		return 0;

	if (!IS_ERR(SC533HGS->reset_gpio))
		gpiod_set_value_cansleep(SC533HGS->reset_gpio, 0);

	ret = regulator_bulk_enable(SC533HGS_NUM_SUPPLIES, SC533HGS->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	if (!IS_ERR(SC533HGS->reset_gpio))
		gpiod_set_value_cansleep(SC533HGS->reset_gpio, 1);

	usleep_range(500, 1000);

	if (!IS_ERR(SC533HGS->reset_gpio))
		usleep_range(6000, 8000);
	else
		usleep_range(12000, 16000);

	delay_us = SC533HGS_cal_delay(8192, SC533HGS);
	usleep_range(delay_us, delay_us * 2);

	return 0;

disable_clk:
	clk_disable_unprepare(SC533HGS->xvclk);

	return ret;
}

static void __SC533HGS_power_off(struct SC533HGS *SC533HGS)
{
	int ret;
	struct device *dev = &SC533HGS->client->dev;

	clk_disable_unprepare(SC533HGS->xvclk);
	if (SC533HGS->is_thunderboot) {
		if (SC533HGS->is_first_streamoff) {
			SC533HGS->is_thunderboot = false;
			SC533HGS->is_first_streamoff = false;
		} else {
			return;
		}
	}

	if (!IS_ERR(SC533HGS->reset_gpio))
		gpiod_set_value_cansleep(SC533HGS->reset_gpio, 0);
	if (!IS_ERR_OR_NULL(SC533HGS->pins_sleep)) {
		ret = pinctrl_select_state(SC533HGS->pinctrl,
					   SC533HGS->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}
	regulator_bulk_disable(SC533HGS_NUM_SUPPLIES, SC533HGS->supplies);
}

static int SC533HGS_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct SC533HGS *SC533HGS = to_SC533HGS(sd);

	return __SC533HGS_power_on(SC533HGS);
}

static int SC533HGS_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct SC533HGS *SC533HGS = to_SC533HGS(sd);

	__SC533HGS_power_off(SC533HGS);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int SC533HGS_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct SC533HGS *SC533HGS = to_SC533HGS(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->state, 0);
	const struct SC533HGS_mode *def_mode = &supported_modes[0];

	mutex_lock(&SC533HGS->mutex);
	try_fmt->width  = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code   = def_mode->bus_fmt;
	try_fmt->field  = V4L2_FIELD_NONE;

	mutex_unlock(&SC533HGS->mutex);

	return 0;
}
#endif

static int SC533HGS_enum_frame_interval(struct v4l2_subdev *sd,
				       struct v4l2_subdev_state *sd_state,
				       struct v4l2_subdev_frame_interval_enum *fie)
{
	if (fie->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	fie->code        = supported_modes[fie->index].bus_fmt;
	fie->width       = supported_modes[fie->index].width;
	fie->height      = supported_modes[fie->index].height;
	fie->interval    = supported_modes[fie->index].max_fps;
	fie->reserved[0] = supported_modes[fie->index].hdr_mode;
	return 0;
}

static const struct dev_pm_ops SC533HGS_pm_ops = {
	SET_RUNTIME_PM_OPS(SC533HGS_runtime_suspend,
			   SC533HGS_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops SC533HGS_internal_ops = {
	.open = SC533HGS_open,
};
#endif

static const struct v4l2_subdev_core_ops SC533HGS_core_ops = {
	.s_power = SC533HGS_s_power,
	.ioctl = SC533HGS_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = SC533HGS_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops SC533HGS_video_ops = {
	.s_stream = SC533HGS_s_stream,
	.g_frame_interval = SC533HGS_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops sc533hgs_pad_ops = {
	.enum_mbus_code      = SC533HGS_enum_mbus_code,
	.enum_frame_size     = SC533HGS_enum_frame_sizes,
	.enum_frame_interval = SC533HGS_enum_frame_interval,
	.get_fmt             = SC533HGS_get_fmt,
	.set_fmt             = SC533HGS_set_fmt,
	.get_mbus_config     = SC533HGS_g_mbus_config,
};

static const struct v4l2_subdev_ops SC533HGS_subdev_ops = {
	.core	= &SC533HGS_core_ops,
	.video	= &SC533HGS_video_ops,
	.pad	= &sc533hgs_pad_ops,
};

static int SC533HGS_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct SC533HGS *SC533HGS = container_of(ctrl->handler,
					       struct SC533HGS, ctrl_handler);
	struct i2c_client *client = SC533HGS->client;
	s64 max;
	int ret = 0;
	u32 val = 0;

	switch (ctrl->id) {
		case V4L2_CID_VBLANK:
			max = SC533HGS->cur_mode->height + ctrl->val - SC533HGS_EXPOSURE_MARGIN;
			__v4l2_ctrl_modify_range(SC533HGS->exposure,
						 SC533HGS->exposure->minimum,
						 max,
						 SC533HGS->exposure->step,
						 SC533HGS->exposure->default_value);
			break;
		case V4L2_CID_EXPOSURE_AUTO:
			SC533HGS->auto_exp = (ctrl->val == V4L2_EXPOSURE_AUTO);
			break;
	}

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		dev_dbg(&client->dev, "set exposure 0x%x\n", ctrl->val);
		if (SC533HGS->cur_mode->hdr_mode == NO_HDR) {
			val = ctrl->val;
			ret = SC533HGS_write_reg(SC533HGS->client,
						SC533HGS_REG_EXPOSURE_H,
						SC533HGS_REG_VALUE_08BIT,
						SC533HGS_FETCH_EXP_H(val));
			ret |= SC533HGS_write_reg(SC533HGS->client,
						 SC533HGS_REG_EXPOSURE_M,
						 SC533HGS_REG_VALUE_08BIT,
						 SC533HGS_FETCH_EXP_M(val));
			ret |= SC533HGS_write_reg(SC533HGS->client,
						 SC533HGS_REG_EXPOSURE_L,
						 SC533HGS_REG_VALUE_08BIT,
						 SC533HGS_FETCH_EXP_L(val));
		}
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		dev_dbg(&client->dev, "set gain 0x%x\n", ctrl->val);
		if (SC533HGS->cur_mode->hdr_mode == NO_HDR)
			ret = SC533HGS_set_gain_reg(SC533HGS, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		dev_dbg(&client->dev, "set vblank 0x%x\n", ctrl->val);
		ret = SC533HGS_write_reg(SC533HGS->client,
					SC533HGS_REG_VTS_H,
					SC533HGS_REG_VALUE_08BIT,
					(ctrl->val + SC533HGS->cur_mode->height)>> 8
					);
		ret |= SC533HGS_write_reg(SC533HGS->client,
					 SC533HGS_REG_VTS_L,
					 SC533HGS_REG_VALUE_08BIT,
					 (ctrl->val + SC533HGS->cur_mode->height) & 0xff);
		SC533HGS->cur_vts = ctrl->val + SC533HGS->cur_mode->height;
		break;
	case V4L2_CID_HBLANK:
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = SC533HGS_enable_test_pattern(SC533HGS, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
		ret = SC533HGS_read_reg(SC533HGS->client, SC533HGS_FLIP_MIRROR_REG,
				       SC533HGS_REG_VALUE_08BIT, &val);
		ret |= SC533HGS_write_reg(SC533HGS->client, SC533HGS_FLIP_MIRROR_REG,
					 SC533HGS_REG_VALUE_08BIT,
					 SC533HGS_FETCH_MIRROR(val, ctrl->val));
		break;
	case V4L2_CID_VFLIP:
		ret = SC533HGS_read_reg(SC533HGS->client, SC533HGS_FLIP_MIRROR_REG,
				       SC533HGS_REG_VALUE_08BIT, &val);
		ret |= SC533HGS_write_reg(SC533HGS->client, SC533HGS_FLIP_MIRROR_REG,
					 SC533HGS_REG_VALUE_08BIT,
					 SC533HGS_FETCH_FLIP(val, ctrl->val));
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops SC533HGS_ctrl_ops = {
	.s_ctrl = SC533HGS_set_ctrl,
};

static int SC533HGS_initialize_controls(struct SC533HGS *SC533HGS)
{
	const struct SC533HGS_mode *mode;
	struct v4l2_ctrl_handler *handler;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;
	u64 dst_link_freq = 0;
	u64 dst_pixel_rate = 0;

	handler = &SC533HGS->ctrl_handler;
	mode = SC533HGS->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 10);
	if (ret)
		return ret;
	handler->lock = &SC533HGS->mutex;

	SC533HGS->link_freq = v4l2_ctrl_new_int_menu(handler,
						NULL,
						V4L2_CID_LINK_FREQ,
						ARRAY_SIZE(link_freq_menu_items) - 1,
						0,
						link_freq_menu_items);
	if (SC533HGS->link_freq)
		SC533HGS->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	dst_link_freq = mode->link_freq_idx;
	dst_pixel_rate = (u32)link_freq_menu_items[mode->link_freq_idx] /
					 SC533HGS_BITS_PER_SAMPLE * 2 * SC533HGS_LANES;

	SC533HGS->pixel_rate = v4l2_ctrl_new_std(handler,
						NULL,
						V4L2_CID_PIXEL_RATE,
						0,
						PIXEL_RATE_WITH_486M_12BIT,
						1,
						dst_pixel_rate);

	__v4l2_ctrl_s_ctrl(SC533HGS->link_freq, dst_link_freq);

	h_blank = mode->hts_def - mode->width;

	SC533HGS->hblank = v4l2_ctrl_new_std(handler,
						NULL,
						V4L2_CID_HBLANK,
					    h_blank,
						h_blank,
						1,
						h_blank);

	if (SC533HGS->hblank)
		SC533HGS->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;

	SC533HGS->vblank = v4l2_ctrl_new_std(handler,
						&SC533HGS_ctrl_ops,
					    V4L2_CID_VBLANK,
						vblank_def,
					    SC533HGS_VTS_MAX - mode->height,
					    1,
						vblank_def);

	exposure_max = mode->vts_def - SC533HGS_EXPOSURE_MARGIN;
	SC533HGS->exposure = v4l2_ctrl_new_std(handler,
							&SC533HGS_ctrl_ops,
							V4L2_CID_EXPOSURE,
							SC533HGS_EXPOSURE_MIN,
							exposure_max,
							SC533HGS_EXPOSURE_STEP,
							mode->exp_def);

	SC533HGS->anal_gain = v4l2_ctrl_new_std(handler,
							&SC533HGS_ctrl_ops,
							V4L2_CID_ANALOGUE_GAIN,
							SC533HGS_GAIN_MIN,
							SC533HGS_GAIN_MAX,
							SC533HGS_GAIN_STEP,
							SC533HGS_GAIN_DEFAULT);

	SC533HGS->test_pattern = v4l2_ctrl_new_std_menu_items(handler,
							&SC533HGS_ctrl_ops,
							V4L2_CID_TEST_PATTERN,
							ARRAY_SIZE(SC533HGS_test_pattern_menu) - 1,
							0,
							0,
							SC533HGS_test_pattern_menu);

	v4l2_ctrl_new_std(handler,
							&SC533HGS_ctrl_ops,
							V4L2_CID_HFLIP,
							0,
							1,
							1,
							0);

	v4l2_ctrl_new_std(handler,
							&SC533HGS_ctrl_ops,
							V4L2_CID_VFLIP,
							0,
							1,
							1,
							0);

	if (handler->error)
	{
		ret = handler->error;
		dev_err(&SC533HGS->client->dev,"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	v4l2_ctrl_new_std_menu(handler, &SC533HGS_ctrl_ops,
			       V4L2_CID_EXPOSURE_AUTO,
			       V4L2_EXPOSURE_MANUAL, 0, V4L2_EXPOSURE_AUTO);

	SC533HGS->subdev.ctrl_handler = handler;
	SC533HGS->has_init_exp = false;
	SC533HGS->auto_exp = true;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int SC533HGS_check_sensor_id(struct SC533HGS *SC533HGS,
				   struct i2c_client *client)
{
	struct device *dev = &SC533HGS->client->dev;
	u32 id = 0;
	int ret;

	if (SC533HGS->is_thunderboot) {
		dev_info(dev, "Enable thunderboot mode, skip sensor id check\n");
		return 0;
	}

	ret = SC533HGS_read_reg(client, SC533HGS_REG_CHIP_ID,
			       SC533HGS_REG_VALUE_16BIT, &id);
	if (id != CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%06x), ret(%d)\n", id, ret);
		return -ENODEV;
	}

	dev_info(dev, "Detected SC533HGS sensor, chip_id=0x%04x\n", CHIP_ID);

	return 0;
}

static int SC533HGS_configure_regulators(struct SC533HGS *SC533HGS)
{
	unsigned int i;

	for (i = 0; i < SC533HGS_NUM_SUPPLIES; i++)
		SC533HGS->supplies[i].supply = SC533HGS_supply_names[i];

	return devm_regulator_bulk_get(&SC533HGS->client->dev,
				       SC533HGS_NUM_SUPPLIES,
				       SC533HGS->supplies);
}

static int SC533HGS_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct SC533HGS *SC533HGS;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;
	int i, hdr_mode = 0;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		 DRIVER_VERSION >> 16,
		 (DRIVER_VERSION & 0xff00) >> 8,
		 DRIVER_VERSION & 0x00ff);

	SC533HGS = devm_kzalloc(dev, sizeof(*SC533HGS), GFP_KERNEL);
	if (!SC533HGS)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
					&SC533HGS->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &SC533HGS->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &SC533HGS->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &SC533HGS->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	SC533HGS->is_thunderboot = false;

	SC533HGS->client = client;
	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		if (hdr_mode == supported_modes[i].hdr_mode) {
			SC533HGS->cur_mode = &supported_modes[i];
			break;
		}
	}
	if (i == ARRAY_SIZE(supported_modes))
		SC533HGS->cur_mode = &supported_modes[0];

	SC533HGS->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(SC533HGS->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	SC533HGS->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(SC533HGS->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	SC533HGS->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(SC533HGS->pinctrl)) {
		SC533HGS->pins_default =
			pinctrl_lookup_state(SC533HGS->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(SC533HGS->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		SC533HGS->pins_sleep =
			pinctrl_lookup_state(SC533HGS->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(SC533HGS->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	} else {
		dev_err(dev, "no pinctrl\n");
	}

	ret = SC533HGS_configure_regulators(SC533HGS);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	mutex_init(&SC533HGS->mutex);

	sd = &SC533HGS->subdev;
	v4l2_i2c_subdev_init(sd, client, &SC533HGS_subdev_ops);
	ret = SC533HGS_initialize_controls(SC533HGS);
	if (ret)
		goto err_destroy_mutex;

	ret = __SC533HGS_power_on(SC533HGS);
	if (ret)
		goto err_free_handler;

	ret = SC533HGS_check_sensor_id(SC533HGS, client);
	if (ret)
		goto err_power_off;

#if defined(CONFIG_MEDIA_CONTROLLER)
	SC533HGS->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &SC533HGS->pad);
	if (ret < 0)
		goto err_power_off;
#endif

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &SC533HGS_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(SC533HGS->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 SC533HGS->module_index, facing,
		 SC533HGS_NAME, dev_name(sd->dev));

	ret = v4l2_async_register_subdev_sensor(sd);
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto err_clean_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	return 0;

err_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
err_power_off:
	__SC533HGS_power_off(SC533HGS);
err_free_handler:
	v4l2_ctrl_handler_free(&SC533HGS->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&SC533HGS->mutex);

	return ret;
}

static void SC533HGS_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct SC533HGS *SC533HGS = to_SC533HGS(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&SC533HGS->ctrl_handler);
	mutex_destroy(&SC533HGS->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__SC533HGS_power_off(SC533HGS);
	pm_runtime_set_suspended(&client->dev);
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id SC533HGS_of_match[] = {
	{ .compatible = "smartsens,sc533hgs" },
	{},
};
MODULE_DEVICE_TABLE(of, SC533HGS_of_match);
#endif

static const struct i2c_device_id SC533HGS_match_id[] = {
	{ "smartsens,sc533hgs", 0 },
	{ },
};

static struct i2c_driver SC533HGS_i2c_driver = {
	.driver = {
		.name = SC533HGS_NAME,
		.pm = &SC533HGS_pm_ops,
		.of_match_table = of_match_ptr(SC533HGS_of_match),
	},
	.probe    = &SC533HGS_probe,
	.remove   = &SC533HGS_remove,
	.id_table = SC533HGS_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&SC533HGS_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&SC533HGS_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_AUTHOR("RV1126B Driver");
MODULE_DESCRIPTION("smartsens SC533HGS sensor driver");
MODULE_LICENSE("GPL");
