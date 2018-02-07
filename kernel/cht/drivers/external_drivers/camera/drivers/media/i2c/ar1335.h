/*
 * Support for Omnivision AR1335 camera sensor.
 * Based on Aptina mt9e013 driver.
 *
 * Copyright (c) 2010 Intel Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#ifndef __AR1335_H__
#define __AR1335_H__
#include <linux/atomisp_platform.h>
#include <linux/atomisp.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/videodev2.h>
#include <linux/v4l2-mediabus.h>
#include <linux/types.h>

#include <media/media-entity.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#define to_dw9761_device(sd)	(&container_of(sd, struct ar1335_device, sd) \
				->dw9761)



#define DW9761_MAX_FOCUS_POS 1023

#define DW9761_I2C_EFLASH_ADDR                  (0xB0 >> 1)
#define DW9761_I2C_ADDR				(0x18 >> 1)//0x0E
#define DW9761_INFO					0x0
#define DW9761_VER					0x1
#define DW9761_CONTROL				2
#define DW9761_VCM_CURRENT			3
#define DW9761_STATUS				5
#define DW9761_MODE					6
#define DW9761_VCM_FREQ				7
#define DW9761_VCM_PRELOAD			8
#define LILP3_INFO				2
#define LILP3_DEBUG				1

#define DW9761_MAX_FOCUS_POS			1023
struct dw9761_device {
	bool initialized;		/* true if dw9761 is detected */
	s32 focus;			/* Current focus value */
	struct timespec focus_time;	/* Time when focus was last time set */
	__u8 buffer[4];			/* Used for i2c transactions */
	const struct camera_af_platform_data *platform_data;
};
#define	AR1335_NAME	"ar1335"
#define	AR1335_ADDR	0x10


#define AR1335_CHIP_ID	0x0153


#define	LAST_REG_SETING		{0xffff, 0xff}
#define	is_last_reg_setting(item) ((item).reg == 0xffff)
#define I2C_MSG_LENGTH		0x2

#define AR1335_INVALID_CONFIG	0xffffffff

#define AR1335_INTG_UNIT_US	100
#define AR1335_MCLK		192

#define AR1335_REG_BITS	16
#define AR1335_REG_MASK	0xFFFF

/* This should be added into include/linux/videodev2.h */
#ifndef V4L2_IDENT_AR1335
#define V4L2_IDENT_AR1335	8245
#endif

/*
 * ar1335 System control registers
 */
#define AR1335_PLL_PLL10			0x3090
#define AR1335_PLL_PLL11			0x3091
#define AR1335_PLL_PLL12			0x3092
#define AR1335_PLL_PLL13			0x3093
#define AR1335_TIMING_VTS			0x380e
#define AR1335_TIMING_HTS			0x380C

#define PLL1_SYS_PRE_DIV			0x030F

#define AR1335_HORIZONTAL_START_H		0x0344
#define AR1335_VERTICAL_START_H			0x0346
#define AR1335_HORIZONTAL_END_H			0x3804
#define AR1335_HORIZONTAL_END_L			0x3805
#define AR1335_VERTICAL_END_H			0x3806
#define AR1335_VERTICAL_END_L			0x3807
#define AR1335_HORIZONTAL_OUTPUT_SIZE_H		0x034C
#define AR1335_VERTICAL_OUTPUT_SIZE_H		0x034E

#define AR1335_GROUP_ACCESS			0x0104
#define AR1335_GROUP_ACCESS_HOLD_START		0x01
#define AR1335_GROUP_ACCESS_HOLD_END		0x00
#define AR1335_GROUP_ACCESS_DELAY_LAUNCH	0xA0
#define AR1335_GROUP_ACCESS_QUICK_LAUNCH	0xE0

#define AR1335_LONG_EXPO			0x0202
#define AR1335_AGC_ADJ				0x305e
#define AR1335_TEST_PATTERN_MODE		0x3070

/* ar1335 SCCB */
#define AR1335_SCCB_CTRL			0x3100
#define AR1335_AEC_PK_EXPO_H			0x3500
#define AR1335_AEC_PK_EXPO_M			0x3501
#define AR1335_AEC_PK_EXPO_L			0x3502
#define AR1335_AEC_MANUAL_CTRL			0x3503
#define AR1335_AGC_ADJ_H			0x3508
#define AR1335_AGC_ADJ_L			0x3509

#define AR1335_MWB_RED_GAIN_H			0x3400
#define AR1335_MWB_GREEN_GAIN_H			0x3402
#define AR1335_MWB_BLUE_GAIN_H			0x3404
#define AR1335_DIGI_GAIN				0x350A
#define AR1335_MWB_GAIN_MAX			0x3fff

#define AR1335_OTP_BANK0_PID			0x3d00
#define AR1335_CHIP_ID_HIGH			0x0000
#define AR1335_CHIP_ID_LOW			0x300C
#define AR1335_STREAM_MODE			0x0100

#define AR1335_FOCAL_LENGTH_NUM	439	/*4.39mm*/
#define AR1335_FOCAL_LENGTH_DEM	100
#define AR1335_F_NUMBER_DEFAULT_NUM	24
#define AR1335_F_NUMBER_DEM	10

#define AR1335_TIMING_X_INC		0x3814
#define AR1335_TIMING_Y_INC		0x382A

#define AR1335_ISP_CTRL 0x501E

/* sensor_mode_data read_mode adaptation */
#define AR1335_READ_MODE_BINNING_ON	0x0400
#define AR1335_READ_MODE_BINNING_OFF	0x00
#define AR1335_INTEGRATION_TIME_MARGIN	8

#define AR1335_MAX_VTS_VALUE		0xFFFF
#define AR1335_MAX_EXPOSURE_VALUE \
		(AR1335_MAX_VTS_VALUE - AR1335_INTEGRATION_TIME_MARGIN)
#define AR1335_MAX_GAIN_VALUE		0xffff /*x32 gain*/


#define PLL_MULTIPLIER1		0x0306
#define PRE_PLL_CLK_DIV1	0x0304
#define VT_SYS_CLK_DIV		0x0302
#define VT_PIX_CLK_DIV		0x0300
#define ROW_SPEED			0x3016


/*
 * focal length bits definition:
 * bits 31-16: numerator, bits 15-0: denominator
 */
#define AR1335_FOCAL_LENGTH_DEFAULT ((385<<16)|100) //3.85

/*
 * current f-number bits definition:
 * bits 31-16: numerator, bits 15-0: denominator
 */
#define AR1335_F_NUMBER_DEFAULT ((220<<16)|100) //2.2

/*
 * f-number range bits definition:
 * bits 31-24: max f-number numerator
 * bits 23-16: max f-number denominator
 * bits 15-8: min f-number numerator
 * bits 7-0: min f-number denominator
 */
#define AR1335_F_NUMBER_RANGE 0x180a180a
#define OTPM_ADD_START_1		0x1000
#define OTPM_DATA_LENGTH_1		0x0100
#define OTPM_COUNT 0x200

/* Defines for register writes and register array processing */
#define AR1335_BYTE_MAX	32
#define AR1335_SHORT_MAX	16
#define I2C_RETRY_COUNT		5
#define AR1335_TOK_MASK	0xfff0

#define	AR1335_STATUS_POWER_DOWN	0x0
#define	AR1335_STATUS_STANDBY		0x2
#define	AR1335_STATUS_ACTIVE		0x3
#define	AR1335_STATUS_VIEWFINDER	0x4

#define MAX_FPS_OPTIONS_SUPPORTED	3

#define	v4l2_format_capture_type_entry(_width, _height, \
		_pixelformat, _bytesperline, _colorspace) \
	{\
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,\
		.fmt.pix.width = (_width),\
		.fmt.pix.height = (_height),\
		.fmt.pix.pixelformat = (_pixelformat),\
		.fmt.pix.bytesperline = (_bytesperline),\
		.fmt.pix.colorspace = (_colorspace),\
		.fmt.pix.sizeimage = (_height)*(_bytesperline),\
	}

#define	s_output_format_entry(_width, _height, _pixelformat, \
		_bytesperline, _colorspace, _fps) \
	{\
		.v4l2_fmt = v4l2_format_capture_type_entry(_width, \
			_height, _pixelformat, _bytesperline, \
				_colorspace),\
		.fps = (_fps),\
	}

#define	s_output_format_reg_entry(_width, _height, _pixelformat, \
		_bytesperline, _colorspace, _fps, _reg_setting) \
	{\
		.s_fmt = s_output_format_entry(_width, _height,\
				_pixelformat, _bytesperline, \
				_colorspace, _fps),\
		.reg_setting = (_reg_setting),\
	}

struct s_ctrl_id {
	struct v4l2_queryctrl qc;
	int (*s_ctrl)(struct v4l2_subdev *sd, u32 val);
	int (*g_ctrl)(struct v4l2_subdev *sd, u32 *val);
};

#define	v4l2_queryctrl_entry_integer(_id, _name,\
		_minimum, _maximum, _step, \
		_default_value, _flags)	\
	{\
		.id = (_id), \
		.type = V4L2_CTRL_TYPE_INTEGER, \
		.name = _name, \
		.minimum = (_minimum), \
		.maximum = (_maximum), \
		.step = (_step), \
		.default_value = (_default_value),\
		.flags = (_flags),\
	}
#define	v4l2_queryctrl_entry_boolean(_id, _name,\
		_default_value, _flags)	\
	{\
		.id = (_id), \
		.type = V4L2_CTRL_TYPE_BOOLEAN, \
		.name = _name, \
		.minimum = 0, \
		.maximum = 1, \
		.step = 1, \
		.default_value = (_default_value),\
		.flags = (_flags),\
	}

#define	s_ctrl_id_entry_integer(_id, _name, \
		_minimum, _maximum, _step, \
		_default_value, _flags, \
		_s_ctrl, _g_ctrl)	\
	{\
		.qc = v4l2_queryctrl_entry_integer(_id, _name,\
				_minimum, _maximum, _step,\
				_default_value, _flags), \
		.s_ctrl = _s_ctrl, \
		.g_ctrl = _g_ctrl, \
	}

#define	s_ctrl_id_entry_boolean(_id, _name, \
		_default_value, _flags, \
		_s_ctrl, _g_ctrl)	\
	{\
		.qc = v4l2_queryctrl_entry_boolean(_id, _name,\
				_default_value, _flags), \
		.s_ctrl = _s_ctrl, \
		.g_ctrl = _g_ctrl, \
	}


#define	macro_string_entry(VAL)	\
	{ \
		.val = VAL, \
		.string = #VAL, \
	}

enum ar1335_tok_type {
	AR1335_8BIT  = 0x0001,
	AR1335_16BIT = 0x0002,
	AR1335_TOK_TERM   = 0xf000,	/* terminating token for reg list */
	AR1335_TOK_DELAY  = 0xfe00	/* delay token for reg list */
};

/*
 * If register address or register width is not 32 bit width,
 * user needs to convert it manually
 */

struct s_register_setting {
	u32 reg;
	u32 val;
};

struct s_output_format {
	struct v4l2_format v4l2_fmt;
	int fps;
};

/**
 * struct ar1335_fwreg - Firmware burst command
 * @type: FW burst or 8/16 bit register
 * @addr: 16-bit offset to register or other values depending on type
 * @val: data value for burst (or other commands)
 *
 * Define a structure for sensor register initialization values
 */
struct ar1335_fwreg {
	enum ar1335_tok_type type; /* value, register or FW burst string */
	u16 addr;	/* target address */
	u32 val[8];
};

/**
 * struct ar1335_reg - MI sensor  register format
 * @type: type of the register
 * @reg: 16-bit offset to register
 * @val: 8/16/32-bit register value
 *
 * Define a structure for sensor register initialization values
 */
struct ar1335_reg {
	enum ar1335_tok_type type;
	union {
		u16 sreg;
		struct ar1335_fwreg *fwreg;
	} reg;
	u32 val;	/* @set value for read/mod/write, @mask */
	u32 val2;	/* optional: for rmw, OR mask */
};

struct ar1335_fps_setting {
	int fps;
	unsigned short pixels_per_line;
	unsigned short lines_per_frame;
};

/* Store macro values' debug names */
struct macro_string {
	u8 val;
	char *string;
};

static inline const char *
macro_to_string(const struct macro_string *array, int size, u8 val)
{
	int i;
	for (i = 0; i < size; i++) {
		if (array[i].val == val)
			return array[i].string;
	}
	return "Unknown VAL";
}

struct ar1335_control {
	struct v4l2_queryctrl qc;
	int (*query)(struct v4l2_subdev *sd, s32 *value);
	int (*tweak)(struct v4l2_subdev *sd, s32 value);
};

struct ar1335_resolution {
	u8 *desc;
	int res;
	int width;
	int height;
	bool used;
	const struct ar1335_reg *regs;
	u8 bin_factor_x;
	u8 bin_factor_y;
	unsigned short skip_frames;
	const struct ar1335_fps_setting fps_options[MAX_FPS_OPTIONS_SUPPORTED];
        int mipi_freq;
};

struct ar1335_format {
	u8 *desc;
	u32 pixelformat;
	struct s_register_setting *regs;
};

/* ar1335 device structure */
struct ar1335_device {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_mbus_framefmt format;

	u8 *otp_data;
	struct camera_sensor_platform_data *platform_data;
	int fmt_idx;
	int streaming;
	int power;
	u16 sensor_id;
	u8 sensor_revision;
	int exposure;
	int gain;
	int digital_gain;
	struct dw9761_device dw9761;
	struct mutex input_lock; /* serialize sensor's ioctl */

	const struct ar1335_reg *basic_settings_list;
	const struct ar1335_resolution *curr_res_table;
	int entries_curr_table;
	int fps_index;

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *run_mode;
};

/*
 * The i2c adapter on Intel Medfield can transfer 32 bytes maximum
 * at a time. In burst mode we require that the buffer is transferred
 * in one shot, so limit the buffer size to 32 bytes minus a safety.
 */
#define AR1335_MAX_WRITE_BUF_SIZE	30
struct ar1335_write_buffer {
	u16 addr;
	u8 data[AR1335_MAX_WRITE_BUF_SIZE];
};

struct ar1335_write_ctrl {
	int index;
	struct ar1335_write_buffer buffer;
};

#define AR1335_RES_WIDTH_MAX	4208
#define AR1335_RES_HEIGHT_MAX	3120

/* please refer to 0x3800 0x3801 setting*/
#define AR1335_CROP_X_START_FOR_MAX_SIZE 0x0C
/* please refer to 0x3802 0x3803 setting*/
#define AR1335_CROP_Y_START_FOR_MAX_SIZE 0x0C

#define ISP_PADDING_W 16
#define ISP_PADDING_H 16
#define AR1335_ISP_MAX_WIDTH	(AR1335_RES_WIDTH_MAX - ISP_PADDING_W)
#define AR1335_ISP_MAX_HEIGHT	(AR1335_RES_HEIGHT_MAX - ISP_PADDING_H)
static const struct ar1335_reg ar1335_Reset[] = {
	{AR1335_8BIT, {0x0103}, 0x01},
	{AR1335_TOK_DELAY, {0}, 0x064},
	{AR1335_8BIT, {0x0103}, 0x00},
	{AR1335_TOK_DELAY, {0}, 0x0A},
	{ AR1335_TOK_TERM, {0}, 0}

};
static const struct ar1335_reg corrections_recommended[] = {
	{AR1335_16BIT, {0x3042}, 0x1004}, //vtx_lo_control on, BW ON
	{AR1335_16BIT, {0x30D2}, 0x0120}, //enable crm, disable crm for CC rows
	//cc and rnf off
	{AR1335_16BIT,  {0x30D4}, 0x00},
	{AR1335_16BIT,  {0x3090}, 0x00},
	{AR1335_16BIT, {0x30FC}, 0x0068},
	{AR1335_16BIT, {0x30FE}, 0x0068},
	{AR1335_16BIT, {0x31E0}, 0x0781}, //enable 2ddc
	{AR1335_16BIT, {0x3180}, 0x9434}, //fdoc settings
	{AR1335_16BIT, {0x317C}, 0xEFF4}, //analog control7
	{AR1335_16BIT, {0x30EE}, 0x613E}, //dark_control3, enable rnc diter
	{AR1335_16BIT, {0x3F2C}, 0x4428}, //gth_thres_rtn
	{ AR1335_TOK_TERM, {0}, 0}
};

static const struct ar1335_reg pixel_timing_recommended[] = {
	{AR1335_16BIT, {0x3D00}, 0x0446},
	{AR1335_16BIT, {0x3D02}, 0x4C66},
	{AR1335_16BIT, {0x3D04}, 0xFFFF},
	{AR1335_16BIT, {0x3D06}, 0xFFFF},
	// @04 Read
	{AR1335_16BIT, {0x3D08}, 0x5E40},
	{AR1335_16BIT, {0x3D0A}, 0x1146},
	{AR1335_16BIT, {0x3D0C}, 0x5D41},
	{AR1335_16BIT, {0x3D0E}, 0x1088},
	{AR1335_16BIT, {0x3D10}, 0x8342},
	{AR1335_16BIT, {0x3D12}, 0x00C0},
	{AR1335_16BIT, {0x3D14}, 0x5580},
	{AR1335_16BIT, {0x3D16}, 0x5B83},
	{AR1335_16BIT, {0x3D18}, 0x6084},
	{AR1335_16BIT, {0x3D1A}, 0x5A8D},
	{AR1335_16BIT, {0x3D1C}, 0x00C0},
	{AR1335_16BIT, {0x3D1E}, 0x8342},
	{AR1335_16BIT, {0x3D20}, 0x925A},
	{AR1335_16BIT, {0x3D22}, 0x8664},
	{AR1335_16BIT, {0x3D24}, 0x1030},
	{AR1335_16BIT, {0x3D26}, 0x801C},
	{AR1335_16BIT, {0x3D28}, 0x00A0},
	{AR1335_16BIT, {0x3D2A}, 0x56B0},
	{AR1335_16BIT, {0x3D2C}, 0x5788},
	{AR1335_16BIT, {0x3D2E}, 0x5150},
	{AR1335_16BIT, {0x3D30}, 0x824D},
	{AR1335_16BIT, {0x3D32}, 0x8858},
	{AR1335_16BIT, {0x3D34}, 0x58D7},
	{AR1335_16BIT, {0x3D36}, 0x438A},
	{AR1335_16BIT, {0x3D38}, 0x4592},
	{AR1335_16BIT, {0x3D3A}, 0x458A},
	{AR1335_16BIT, {0x3D3C}, 0x43A7},
	{AR1335_16BIT, {0x3D3E}, 0x51FF},
	{AR1335_16BIT, {0x3D40}, 0x8451},
	{AR1335_16BIT, {0x3D42}, 0x8410},
	{AR1335_16BIT, {0x3D44}, 0x0C83},
	{AR1335_16BIT, {0x3D46}, 0x5959},
	{AR1335_16BIT, {0x3D48}, 0x8C5F},
	{AR1335_16BIT, {0x3D4A}, 0xD842},
	{AR1335_16BIT, {0x3D4C}, 0x9361},
	{AR1335_16BIT, {0x3D4E}, 0x8262},
	{AR1335_16BIT, {0x3D50}, 0x8342},
	{AR1335_16BIT, {0x3D52}, 0x8010},
	{AR1335_16BIT, {0x3D54}, 0xC041},
	{AR1335_16BIT, {0x3D56}, 0x64FF},
	{AR1335_16BIT, {0x3D58}, 0xFF9E},
	{AR1335_16BIT, {0x3D5A}, 0x4081},
	{AR1335_16BIT, {0x3D5C}, 0x4080},
	{AR1335_16BIT, {0x3D5E}, 0x4180},
	{AR1335_16BIT, {0x3D60}, 0x4280},
	{AR1335_16BIT, {0x3D62}, 0x438D},
	{AR1335_16BIT, {0x3D64}, 0x44BA},
	{AR1335_16BIT, {0x3D66}, 0x4488},
	{AR1335_16BIT, {0x3D68}, 0x4380},
	{AR1335_16BIT, {0x3D6A}, 0x4241},
	{AR1335_16BIT, {0x3D6C}, 0x8140},
	{AR1335_16BIT, {0x3D6E}, 0x8240},
	{AR1335_16BIT, {0x3D70}, 0x8041},
	{AR1335_16BIT, {0x3D72}, 0x8042},
	{AR1335_16BIT, {0x3D74}, 0x8043},
	{AR1335_16BIT, {0x3D76}, 0x8D44},
	{AR1335_16BIT, {0x3D78}, 0xBA44},
	{AR1335_16BIT, {0x3D7A}, 0x875E},
	{AR1335_16BIT, {0x3D7C}, 0x4354},
	{AR1335_16BIT, {0x3D7E}, 0x4241},
	{AR1335_16BIT, {0x3D80}, 0x8140},
	{AR1335_16BIT, {0x3D82}, 0x8120},
	{AR1335_16BIT, {0x3D84}, 0x2881},
	{AR1335_16BIT, {0x3D86}, 0x6026},
	{AR1335_16BIT, {0x3D88}, 0x8055},
	{AR1335_16BIT, {0x3D8A}, 0x8070},
	{AR1335_16BIT, {0x3D8C}, 0x8040},
	{AR1335_16BIT, {0x3D8E}, 0x4C81},
	{AR1335_16BIT, {0x3D90}, 0x45C3},
	{AR1335_16BIT, {0x3D92}, 0x4581},
	{AR1335_16BIT, {0x3D94}, 0x4C40},
	{AR1335_16BIT, {0x3D96}, 0x8070},
	{AR1335_16BIT, {0x3D98}, 0x8040},
	{AR1335_16BIT, {0x3D9A}, 0x4C85},
	{AR1335_16BIT, {0x3D9C}, 0x6CA8},
	{AR1335_16BIT, {0x3D9E}, 0x6C8C},
	{AR1335_16BIT, {0x3DA0}, 0x000E},
	{AR1335_16BIT, {0x3DA2}, 0xBE44},
	{AR1335_16BIT, {0x3DA4}, 0x8844},
	{AR1335_16BIT, {0x3DA6}, 0xBC78},
	{AR1335_16BIT, {0x3DA8}, 0x0900},
	{AR1335_16BIT, {0x3DAA}, 0x8904},
	{AR1335_16BIT, {0x3DAC}, 0x8080},
	{AR1335_16BIT, {0x3DAE}, 0x0240},
	{AR1335_16BIT, {0x3DB0}, 0x8609},
	{AR1335_16BIT, {0x3DB2}, 0x008E},
	{AR1335_16BIT, {0x3DB4}, 0x0900},
	{AR1335_16BIT, {0x3DB6}, 0x8002},
	{AR1335_16BIT, {0x3DB8}, 0x4080},
	{AR1335_16BIT, {0x3DBA}, 0x0480},
	{AR1335_16BIT, {0x3DBC}, 0x887C},
	{AR1335_16BIT, {0x3DBE}, 0xAA86},
	{AR1335_16BIT, {0x3DC0}, 0x0900},
	{AR1335_16BIT, {0x3DC2}, 0x877A},
	{AR1335_16BIT, {0x3DC4}, 0x000E},
	{AR1335_16BIT, {0x3DC6}, 0xC379},
	{AR1335_16BIT, {0x3DC8}, 0x4C40},
	{AR1335_16BIT, {0x3DCA}, 0xBF70},
	{AR1335_16BIT, {0x3DCC}, 0x5E40},
	{AR1335_16BIT, {0x3DCE}, 0x114E},
	{AR1335_16BIT, {0x3DD0}, 0x5D41},
	{AR1335_16BIT, {0x3DD2}, 0x5383},
	{AR1335_16BIT, {0x3DD4}, 0x4200},
	{AR1335_16BIT, {0x3DD6}, 0xC055},
	{AR1335_16BIT, {0x3DD8}, 0xA400},
	{AR1335_16BIT, {0x3DDA}, 0xC083},
	{AR1335_16BIT, {0x3DDC}, 0x4288},
	{AR1335_16BIT, {0x3DDE}, 0x6083},
	{AR1335_16BIT, {0x3DE0}, 0x5B80},
	{AR1335_16BIT, {0x3DE2}, 0x5A64},
	{AR1335_16BIT, {0x3DE4}, 0x1030},
	{AR1335_16BIT, {0x3DE6}, 0x801C},
	{AR1335_16BIT, {0x3DE8}, 0x00A5},
	{AR1335_16BIT, {0x3DEA}, 0x5697},
	{AR1335_16BIT, {0x3DEC}, 0x57A5},
	{AR1335_16BIT, {0x3DEE}, 0x5180},
	{AR1335_16BIT, {0x3DF0}, 0x505A},
	{AR1335_16BIT, {0x3DF2}, 0x814D},
	{AR1335_16BIT, {0x3DF4}, 0x8358},
	{AR1335_16BIT, {0x3DF6}, 0x8058},
	{AR1335_16BIT, {0x3DF8}, 0xA943},
	{AR1335_16BIT, {0x3DFA}, 0x8345},
	{AR1335_16BIT, {0x3DFC}, 0xB045},
	{AR1335_16BIT, {0x3DFE}, 0x8343},
	{AR1335_16BIT, {0x3E00}, 0xA351},
	{AR1335_16BIT, {0x3E02}, 0xE251},
	{AR1335_16BIT, {0x3E04}, 0x8C59},
	{AR1335_16BIT, {0x3E06}, 0x8059},
	{AR1335_16BIT, {0x3E08}, 0x8A5F},
	{AR1335_16BIT, {0x3E0A}, 0xEC7C},
	{AR1335_16BIT, {0x3E0C}, 0xCC84},
	{AR1335_16BIT, {0x3E0E}, 0x6182},
	{AR1335_16BIT, {0x3E10}, 0x6283},
	{AR1335_16BIT, {0x3E12}, 0x4283},
	{AR1335_16BIT, {0x3E14}, 0x10CC},
	{AR1335_16BIT, {0x3E16}, 0x6496},
	{AR1335_16BIT, {0x3E18}, 0x4281},
	{AR1335_16BIT, {0x3E1A}, 0x41BB},
	{AR1335_16BIT, {0x3E1C}, 0x4082},
	{AR1335_16BIT, {0x3E1E}, 0x407E},
	{AR1335_16BIT, {0x3E20}, 0xCC41},
	{AR1335_16BIT, {0x3E22}, 0x8042},
	{AR1335_16BIT, {0x3E24}, 0x8043},
	{AR1335_16BIT, {0x3E26}, 0x8300},
	{AR1335_16BIT, {0x3E28}, 0xC088},
	{AR1335_16BIT, {0x3E2A}, 0x44BA},
	{AR1335_16BIT, {0x3E2C}, 0x4488},
	{AR1335_16BIT, {0x3E2E}, 0x00C8},
	{AR1335_16BIT, {0x3E30}, 0x8042},
	{AR1335_16BIT, {0x3E32}, 0x4181},
	{AR1335_16BIT, {0x3E34}, 0x4082},
	{AR1335_16BIT, {0x3E36}, 0x4080},
	{AR1335_16BIT, {0x3E38}, 0x4180},
	{AR1335_16BIT, {0x3E3A}, 0x4280},
	{AR1335_16BIT, {0x3E3C}, 0x4383},
	{AR1335_16BIT, {0x3E3E}, 0x00C0},
	{AR1335_16BIT, {0x3E40}, 0x8844},
	{AR1335_16BIT, {0x3E42}, 0xBA44},
	{AR1335_16BIT, {0x3E44}, 0x8800},
	{AR1335_16BIT, {0x3E46}, 0xC880},
	{AR1335_16BIT, {0x3E48}, 0x4241},
	{AR1335_16BIT, {0x3E4A}, 0x8240},
	{AR1335_16BIT, {0x3E4C}, 0x8140},
	{AR1335_16BIT, {0x3E4E}, 0x8041},
	{AR1335_16BIT, {0x3E50}, 0x8042},
	{AR1335_16BIT, {0x3E52}, 0x8043},
	{AR1335_16BIT, {0x3E54}, 0x8300},
	{AR1335_16BIT, {0x3E56}, 0xC088},
	{AR1335_16BIT, {0x3E58}, 0x44BA},
	{AR1335_16BIT, {0x3E5A}, 0x4488},
	{AR1335_16BIT, {0x3E5C}, 0x00C8},
	{AR1335_16BIT, {0x3E5E}, 0x8042},
	{AR1335_16BIT, {0x3E60}, 0x4181},
	{AR1335_16BIT, {0x3E62}, 0x4082},
	{AR1335_16BIT, {0x3E64}, 0x4080},
	{AR1335_16BIT, {0x3E66}, 0x4180},
	{AR1335_16BIT, {0x3E68}, 0x4280},
	{AR1335_16BIT, {0x3E6A}, 0x4383},
	{AR1335_16BIT, {0x3E6C}, 0x00C0},
	{AR1335_16BIT, {0x3E6E}, 0x8844},
	{AR1335_16BIT, {0x3E70}, 0xBA44},
	{AR1335_16BIT, {0x3E72}, 0x8800},
	{AR1335_16BIT, {0x3E74}, 0xC880},
	{AR1335_16BIT, {0x3E76}, 0x4241},
	{AR1335_16BIT, {0x3E78}, 0x8140},
	{AR1335_16BIT, {0x3E7A}, 0x9F5E},
	{AR1335_16BIT, {0x3E7C}, 0x8A54},
	{AR1335_16BIT, {0x3E7E}, 0x8620},
	{AR1335_16BIT, {0x3E80}, 0x2881},
	{AR1335_16BIT, {0x3E82}, 0x6026},
	{AR1335_16BIT, {0x3E84}, 0x8055},
	{AR1335_16BIT, {0x3E86}, 0x8070},
	{AR1335_16BIT, {0x3E88}, 0x0000},
	{AR1335_16BIT, {0x3E8A}, 0x0000},
	{AR1335_16BIT, {0x3E8C}, 0x0000},
	{AR1335_16BIT, {0x3E8E}, 0x0000},
	{AR1335_16BIT, {0x3E90}, 0x0000},
	{AR1335_16BIT, {0x3E92}, 0x0000},
	{AR1335_16BIT, {0x3E94}, 0x0000},
	{AR1335_16BIT, {0x3E96}, 0x0000},
	{AR1335_16BIT, {0x3E98}, 0x0000},
	{AR1335_16BIT, {0x3E9A}, 0x0000},
	{AR1335_16BIT, {0x3E9C}, 0x0000},
	{AR1335_16BIT, {0x3E9E}, 0x0000},
	{AR1335_16BIT, {0x3EA0}, 0x0000},
	{AR1335_16BIT, {0x3EA2}, 0x0000},
	{AR1335_16BIT, {0x3EA4}, 0x0000},
	{AR1335_16BIT, {0x3EA6}, 0x0000},
	{AR1335_16BIT, {0x3EA8}, 0x0000},
	{AR1335_16BIT, {0x3EAA}, 0x0000},
	{AR1335_16BIT, {0x3EAC}, 0x0000},
	{AR1335_16BIT, {0x3EAE}, 0x0000},
	{AR1335_16BIT, {0x3EB0}, 0x0000},
	{AR1335_16BIT, {0x3EB2}, 0x0000},
	{AR1335_16BIT, {0x3EB4}, 0x0000},
	{ AR1335_TOK_TERM, {0}, 0}
};

static const struct ar1335_reg analog_setup_recommended[] = {
	{AR1335_16BIT, {0x3EB6}, 0x004D},//ECL
	{AR1335_16BIT, {0x3EB8}, 0x010B},//FSC
	{AR1335_16BIT, {0x3EBC}, 0xAA06},//Bias current of vdac booster
	{AR1335_16BIT, {0x3EC0}, 0x1E02},//ADC bias
	{AR1335_16BIT, {0x3EC2}, 0x7700},//ramplo and Dac ref
	{AR1335_16BIT, {0x3EC4}, 0x1C08},//load cap disable, ramp buffer driving current
	{AR1335_16BIT, {0x3EC6}, 0xEA44},//Vtxlo = 4, vtxlo_2 off
	{AR1335_16BIT, {0x3EC8}, 0x0F0F},//Vtxlo0 and vtxlo1
	{AR1335_16BIT, {0x3ECA}, 0x0F4A},//Vtxlo2 and Vrstg
	{AR1335_16BIT, {0x3ECC}, 0x0706},//Vrstdlo
	{AR1335_16BIT, {0x3ECE}, 0x443B},//Again 1x rb and c, vtxlo_2 off
	{AR1335_16BIT, {0x3ED0}, 0x12F0},//
	{AR1335_16BIT, {0x3ED2}, 0x0039},//Tx latch enable
	{AR1335_16BIT, {0x3ED4}, 0x862F},//txlo open loop enable
	{AR1335_16BIT, {0x3ED6}, 0x4080},//cfpn_tx_latch_en
	{AR1335_16BIT, {0x3ED8}, 0x0523},//sample_delay
	{AR1335_16BIT, {0x3EDA}, 0x80FA},//Rst under range
	{AR1335_16BIT, {0x3EDC}, 0x5078},//Sig under range, rst over range
	{AR1335_16BIT, {0x3EDE}, 0x5005},//Sig over range

	//context switching
	{AR1335_16BIT, {0x316A}, 0x8200},//rstdlo context swtiching enable, gain threshold = 2x
	{AR1335_16BIT, {0x316E}, 0x8200},//ECL and FSC context switching enable, gain threshold = 2x
	{AR1335_16BIT, {0x316C}, 0x8200},//TXLO context switching enable, gain threshold = 2x
	{AR1335_16BIT, {0x3EF0}, 0x414D},//ECL setting, low gain bit[14-8], high gain bit[6-0], copied from Rev2, need further tuning.
	{AR1335_16BIT, {0x3EF2}, 0x0101},//FSC setting, low gain bit[11-8], high gain bit[3-0], copied from Rev2, need further tuning. 
	{AR1335_16BIT, {0x3EF6}, 0x0307},//rstdlo setting, copied from Rev2, need further tuning.
	{AR1335_16BIT, {0x3EFA}, 0x0F0F},//txlo0 setting
	{AR1335_16BIT, {0x3EFC}, 0x0F0F},//txlo1 setting
	{AR1335_16BIT, {0x3EFE}, 0x0F0F},//txlo2 setting
	{ AR1335_TOK_TERM, {0}, 0}
};

static const struct ar1335_reg mipi_timing_990M[] = {
	{AR1335_16BIT, {0x31B0}, 0x0053},//Frame preamble 53
	{AR1335_16BIT, {0x31B2}, 0x002A},//Line preamble 2A
	{AR1335_16BIT, {0x31B4}, 0x5390},//MIPI timing0 2390//huyj3,change to 5390
	{AR1335_16BIT, {0x31B6}, 0x23A9},//MIPI timing1 13A9//huyj3,change to 23A9
	{AR1335_16BIT, {0x31B8}, 0x2013},//MIPI timing2 2013
	{AR1335_16BIT, {0x31BA}, 0x1868},//MIPI timing3 1868 
	{AR1335_16BIT, {0x31BC}, 0x858A},//MIPI timing4 858A 
	{ AR1335_TOK_TERM, {0}, 0}
};

static const struct ar1335_reg pll_setup_990M[] = {
	{AR1335_16BIT, {0x0300}, 0x0003}, //VT_PIX_CLK_DIV=3
	{AR1335_16BIT, {0x0302}, 0x0001}, //VT_SYS_CLK_DIV=1
	{AR1335_16BIT, {0x0304}, 0x0203},//PRE_PLL_CLK_DIV2=2// PRE_PLL_CLK_DIV1=3
	{AR1335_16BIT, {0x0306}, 0x6767},//PLL_MULTIPLIER2=103// PLL_MULTIPLIER1=103
	{AR1335_16BIT, {0x0308}, 0x000A},//OP_PIX_CLK_DIV=10
	{AR1335_16BIT, {0x030A}, 0x0001},//OP_SYS_CLK_DIV=1
	{AR1335_16BIT, {0x0112}, 0x0A0A},// data_format=10	
	{AR1335_16BIT, {0x3016}, 0x0101},//op/pc speed
	{ AR1335_TOK_TERM, {0}, 0}
};

static const struct ar1335_reg Defect_correction[] = {
//BITFIELD=0x30FE,0x10,0  //disable clock gating for LSC
	{AR1335_16BIT, {0x31E0}, 0x0781},
	{AR1335_16BIT, {0x3F00}, 0x004F},//BM_T0                
	{AR1335_16BIT, {0x3F02}, 0x0125},//BM_T1       
	{AR1335_8BIT, {0x3F04}, 0x0020},// if Ana_gain<2, use noise_floor0, multiply 2x gain by 64 sacle factor        
	{AR1335_8BIT, {0x3F06}, 0x0040},// if 2<Ana_gain<4, use noise_floor1        
	{AR1335_8BIT, {0x3F08}, 0x0070},// if 4<Ana_gain<7 use noise_floor2 and Ana_gain>7, use noise_floor3      
	{AR1335_16BIT, {0x3F0A}, 0x0101},// Define noise_floor0(low address) and noise_floor1(high address)       
	{AR1335_16BIT, {0x3F0C}, 0x0302},// Define noise_floor2 and noise_floor3       
	{AR1335_8BIT, {0x3F1E}, 0x0022},
	//[low 7.75x*66ms < g*t]
	{AR1335_16BIT, {0x3F1A}, 0x01FF},//cross factor 2
	{AR1335_16BIT, {0x3F14}, 0x0101},//single k factor 2
	{AR1335_16BIT, {0x3F44}, 0x0707},//couple k factor 2
	//[medium 2x*66ms < g*t < 7.75x*66ms]
	{AR1335_16BIT, {0x3F18}, 0x011E},//cross factor 1
	{AR1335_16BIT, {0x3F12}, 0x0303},//single k factor 1
	{AR1335_16BIT, {0x3F42}, 0x1511}, //couple k factor 1
	//[high g*t < 2x*66ms]
	{AR1335_16BIT, {0x3F16}, 0x011E},//cross factor 0
	{AR1335_16BIT, {0x3F10}, 0x0505},//single k factor 0
	{AR1335_16BIT, {0x3F40}, 0x1511}, //couple k factor 0
	{ AR1335_TOK_TERM, {0}, 0}
};

static const struct ar1335_reg Stop_streaming[] = {
	{AR1335_16BIT, {0x3F3C}, 0x0002},
	{AR1335_16BIT, {0x3FE0}, 0x0001},
	{AR1335_8BIT, {0x0100}, 0x0000}, //mode_select
	{AR1335_16BIT, {0x3FE0}, 0x0000},
	{ AR1335_TOK_TERM, {0}, 0}
};

static const struct ar1335_reg Start_streaming[] = {
	{AR1335_16BIT, {0x3F3C}, 0x0003},
	{AR1335_8BIT, {0x0100}, 0x0001}, //mode_select
	{ AR1335_TOK_TERM, {0}, 0}
};

static const struct ar1335_reg LSC_off[] = {
	{AR1335_16BIT, {0x3780}, 0x0000},
	{AR1335_16BIT, {0x30FE}, 0x0068}, //mode_select
	{ AR1335_TOK_TERM, {0}, 0}
};

static const struct ar1335_reg array_setup_2104x1184_30fps[] = {
	//[2.5M_array_setup_4208_2368_Ybin2_Xscale2]
	{AR1335_16BIT, {0x0344}, 0x0010}, //X_ADDR_START 16
	{AR1335_16BIT, {0x0348}, 0x107F}, //X_ADDR_END 4223
	{AR1335_16BIT, {0x0346}, 0x0188}, //Y_ADDR_START 392
	{AR1335_16BIT, {0x034A}, 0x0AC5}, //Y_ADDR_END 2381
	{AR1335_16BIT, {0x034C}, 0x0838}, //X_OUTPUT_SIZE 2104
	{AR1335_16BIT, {0x034E}, 0x04A0}, //Y_OUTPUT_SIZE 1184
	{AR1335_16BIT, {0x3040}, 0xC043}, //X_BIN, X_ODD_INC, Y_ODD_INC

	{AR1335_16BIT, {0x3172}, 0x0206}, //DIGBIN_ENABLE
	{AR1335_16BIT, {0x317A}, 0x516E}, //SF_BIN_ENABLE
	{AR1335_16BIT, {0x3F3C}, 0x0003}, //SF_BIN_ENABLE

	{AR1335_16BIT, {0x0400}, 0x0001}, //Scaling Enabling: 0= disable, 1= x-dir
	{AR1335_16BIT, {0x0404}, 0x0020}, //Scale_M = 32

	{AR1335_16BIT, {0x0342}, 0x1200},//LINE_LENGTH_PCK 2304
	{AR1335_16BIT, {0x0340}, 0x0C6A},//FRAME_LENGTH_LINES 3178
	{AR1335_16BIT, {0x0202}, 0x0C4B}, //COARSE_INTEGRATION_TIME 3147
	{AR1335_16BIT, {0x3F00}, 0x0044},//BM_T0
	{AR1335_16BIT, {0x3F02}, 0x0128},//BM_T1

	{AR1335_16BIT, {0x0112}, 0x0A0A},//CCP_DATA_FORMAT
	
	{ AR1335_TOK_TERM, {0}, 0},
};

static const struct ar1335_reg array_setup_2104x1560_30fps[] = {
	//[Hidden: 3M_array_setup_4208_3120_Ybin2_Xscale2]
	{AR1335_16BIT, {0x0344}, 0x0010}, //X_ADDR_START 16 
	{AR1335_16BIT, {0x0348}, 0x107D}, //X_ADDR_END 4221 
	{AR1335_16BIT, {0x0346}, 0x0010}, //Y_ADDR_START 16 
	{AR1335_16BIT, {0x034A}, 0x0C3D}, //Y_ADDR_END 3133 
	{AR1335_16BIT, {0x034C}, 0x0838}, //X_OUTPUT_SIZE 2104	
	{AR1335_16BIT, {0x034E}, 0x0618}, //Y_OUTPUT_SIZE 1560	

	{AR1335_16BIT, {0x3040}, 0xC043}, //X_BIN, X_ODD_INC, Y_ODD_INC
	{AR1335_16BIT, {0x3172}, 0x0206}, //digbin_enable
	{AR1335_16BIT, {0x317A}, 0x516E}, //sfbin_enable
	{AR1335_16BIT, {0x3F3C}, 0x0003}, //bin4

	//scale Configuration	
	{AR1335_16BIT, {0x0400}, 0x0001}, //Scaling Enabling: 0= disable, 1= x-dir
	{AR1335_16BIT, {0x0404}, 0x0020}, //Scale_M = 32

	//timming Configuration 	
	{AR1335_16BIT, {0x0342}, 0x1200},//LINE_LENGTH_PCK 2304 
	{AR1335_16BIT, {0x0340}, 0x0C6E},//FRAME_LENGTH_LINES 3182
	{AR1335_16BIT, {0x0202}, 0x0C4F}, //COARSE_INTEGRATION_TIME 3151
	{AR1335_16BIT, {0x3F00}, 0x0044},//BM_T0
	{AR1335_16BIT, {0x3F02}, 0x0128},//BM_T1

	{AR1335_16BIT, {0x0112}, 0x0A0A},//CCP_DATA_FORMAT

	{ AR1335_TOK_TERM, {0}, 0},
};

static const struct ar1335_reg array_setup_4208x2368_30fps[] = {
	//[Hidden: 10M_array_setup_4208_2368]
	{AR1335_16BIT, {0x0344}, 0x0010}, //X_ADDR_START 16
	{AR1335_16BIT, {0x0348}, 0x107F}, //X_ADDR_END 4223
	{AR1335_16BIT, {0x0346}, 0x0188}, //Y_ADDR_START 392
	{AR1335_16BIT, {0x034A}, 0x0AC7}, //Y_ADDR_END 2383
	{AR1335_16BIT, {0x034C}, 0x1070}, //X_OUTPUT_SIZE 4208
	{AR1335_16BIT, {0x034E}, 0x0940}, //Y_OUTPUT_SIZE 2368

	{AR1335_16BIT, {0x3040}, 0xC041}, //X_BIN, X_ODD_INC, Y_ODD_INC
	{AR1335_16BIT, {0x3172}, 0x0206}, //DIGBIN_ENABLE
	{AR1335_16BIT, {0x317A}, 0x416E}, //SF_BIN_ENABLE
	{AR1335_16BIT, {0x3F3C}, 0x0003}, //SF_BIN_ENABLE
        {AR1335_16BIT, {0x0400}, 0x0000}, //Scaling Enabling: 0= disable, 1= x-dir

//	REG=0x0342, 0x13FC//LINE_LENGTH_PCK 2558
//	REG=0x0340, 0xB2E//FRAME_LENGTH_LINES 2862
//	REG=0x0202, 0xB12 //COARSE_INTEGRATION_TIME 2834

	{AR1335_16BIT, {0x0342}, 0x13FC},//LINE_LENGTH_PCK 2558
	{AR1335_16BIT, {0x0340}, 0x0B2E},//FRAME_LENGTH_LINES 2862
	{AR1335_16BIT, {0x0202}, 0x0B12}, //COARSE_INTEGRATION_TIME 2834
	{AR1335_16BIT, {0x3F00}, 0x003D},//BM_T0
	{AR1335_16BIT, {0x3F02}, 0x010A},//BM_T1

	{AR1335_16BIT, {0x0112}, 0x0A0A},//CCP_DATA_FORMAT

	{ AR1335_TOK_TERM, {0}, 0},
};

static const struct ar1335_reg array_setup_1936x1096_30fps[] = {
//[2M_array_setup_3872_2192_Ybin2_Xscale2]
	{AR1335_16BIT, {0x0344}, 0x00B8}, //X_ADDR_START 184
	{AR1335_16BIT, {0x0348}, 0x0FD7}, //X_ADDR_END 4055
	{AR1335_16BIT, {0x0346}, 0x01E0}, //Y_ADDR_START 480
	{AR1335_16BIT, {0x034A}, 0x0A6D}, //Y_ADDR_END 2373
	{AR1335_16BIT, {0x034C}, 0x0790}, //X_OUTPUT_SIZE 1936
	{AR1335_16BIT, {0x034E}, 0x0448}, //Y_OUTPUT_SIZE 1096

//binning Configuration
	{AR1335_16BIT, {0x3040}, 0xC043}, //X_BIN, X_ODD_INC, Y_ODD_INC
	{AR1335_16BIT, {0x3172}, 0x0206}, //DIGBIN_ENABLE
	{AR1335_16BIT, {0x317A}, 0x516E}, //SF_BIN_ENABLE
	{AR1335_16BIT, {0x3F3C}, 0x0003}, //SF_BIN_ENABLE

//scale Configuration
	{AR1335_16BIT, {0x0400}, 0x0001}, //Scaling Enabling: 0= disable, 1= x-dir
	{AR1335_16BIT, {0x0404}, 0x0020}, //Scale_M = 32

//timing Configuration
	{AR1335_16BIT, {0x0342}, 0x1200},//LINE_LENGTH_PCK 2304
	{AR1335_16BIT, {0x0340}, 0x0C6A},//FRAME_LENGTH_LINES 3178
	{AR1335_16BIT, {0x0202}, 0x0C4B}, //COARSE_INTEGRATION_TIME 3147
	{AR1335_16BIT, {0x3F00}, 0x0044},//BM_T0
	{AR1335_16BIT, {0x3F02}, 0x0128},//BM_T1

	{AR1335_16BIT, {0x0112}, 0x0A0A},//CCP_DATA_FORMAT
	
	{ AR1335_TOK_TERM, {0}, 0},
};

static const struct ar1335_reg array_setup_1636x1096_30fps[] = {
	{AR1335_16BIT, {0x31B0}, 0x0036},//Frame preamble 53
	{AR1335_16BIT, {0x31B2}, 0x0020},//Line preamble 2A
	{AR1335_16BIT, {0x31B4}, 0x218A},//MIPI timing0 2390//huyj3,change to 5390
	{AR1335_16BIT, {0x31B6}, 0x11A5},//MIPI timing1 13A9//huyj3,change to 23A9
	{AR1335_16BIT, {0x31B8}, 0x1012},//MIPI timing2 2013
	{AR1335_16BIT, {0x31BA}, 0x1030},//MIPI timing3 1868
	{AR1335_16BIT, {0x31BC}, 0x8305},//MIPI timing4 858A

	{AR1335_16BIT, {0x0300}, 0x0003}, //VT_PIX_CLK_DIV=3
	{AR1335_16BIT, {0x0302}, 0x0001}, //VT_SYS_CLK_DIV=1
	{AR1335_16BIT, {0x0304}, 0x0303},//PRE_PLL_CLK_DIV2=2// PRE_PLL_CLK_DIV1=3
	{AR1335_16BIT, {0x0306}, 0x4848},//PLL_MULTIPLIER2=103// PLL_MULTIPLIER1=103
	{AR1335_16BIT, {0x0308}, 0x000A},//OP_PIX_CLK_DIV=10
	{AR1335_16BIT, {0x030A}, 0x0001},//OP_SYS_CLK_DIV=1
	{AR1335_16BIT, {0x0112}, 0x0A0A},// data_format=10
	{AR1335_16BIT, {0x3016}, 0x0101},//op/pc speed
#if 1   //ts-mjw old
	{AR1335_16BIT, {0x0344}, 0x01E4}, //X_ADDR_START 784
	{AR1335_16BIT, {0x0348}, 0x0EA9}, //X_ADDR_END 3455
	{AR1335_16BIT, {0x0346}, 0x01E0}, //Y_ADDR_START 480
	{AR1335_16BIT, {0x034A}, 0x0A6D}, //Y_ADDR_END 2973
	{AR1335_16BIT, {0x034C}, 0x0664}, //X_OUTPUT_SIZE 1336
	{AR1335_16BIT, {0x034E}, 0x0448}, //Y_OUTPUT_SIZE 1096
	{AR1335_16BIT, {0x3040}, 0xC8C3}, //X_BIN, X_ODD_INC, Y_ODD_INC
#else   //ar fae
	{AR1335_16BIT, {0x0344}, 0x0010}, //X_ADDR_START 784
	{AR1335_16BIT, {0x0348}, 0x0cd7}, //X_ADDR_END 3455
	{AR1335_16BIT, {0x0346}, 0x0188}, //Y_ADDR_START 480
	{AR1335_16BIT, {0x034A}, 0x089D}, //Y_ADDR_END 2973
	{AR1335_16BIT, {0x034C}, 0x0664}, //X_OUTPUT_SIZE 1336
	{AR1335_16BIT, {0x034E}, 0x0448}, //Y_OUTPUT_SIZE 1096
	{AR1335_16BIT, {0x3040}, 0xC043}, //X_BIN, X_ODD_INC, Y_ODD_INC
#endif
//Binning Configuration
	{AR1335_16BIT, {0x3172}, 0x0226}, //DIGBIN_ENABLE
	{AR1335_16BIT, {0x317A}, 0x516E}, //SF_BIN_ENABLE
	{AR1335_16BIT, {0x3F3C}, 0x0003}, //SF_BIN_ENABLE

#if 1
//Scale Configuration
	{AR1335_16BIT, {0x0400}, 0x0001}, //Scaling Enabling: 0= disable, 1= x-dir
	{AR1335_16BIT, {0x0404}, 0x0040}, //Scale_M = 32
#else
//Scale Configuration
	{AR1335_16BIT, {0x0400}, 0x0001}, //Scaling Enabling: 0= disable, 1= x-dir
	{AR1335_16BIT, {0x0404}, 0x0020}, //Scale_M = 32
#endif 

//Timing Configuration
	{AR1335_16BIT, {0x0342}, 0x1230},//LINE_LENGTH_PCK 2304
	{AR1335_16BIT, {0x0340}, 0x0c80},//FRAME_LENGTH_LINES 3178
	{AR1335_16BIT, {0x0202}, 0x0881}, //COARSE_INTEGRATION_TIME 3147

	{AR1335_16BIT, {0x3F00}, 0x0044},//FRAME_LENGTH_LINES 3178
	{AR1335_16BIT, {0x3F02}, 0x0125}, //COARSE_INTEGRATION_TIME 3147
	{AR1335_16BIT, {0x0112}, 0x0A0A},

	{ AR1335_TOK_TERM, {0}, 0}
};

static const struct ar1335_reg array_setup_1336x1096_30fps[] = {
////[1M_array_setup_2672_2192_Ybin2_Xscale2]
	{AR1335_16BIT, {0x0344}, 0x0310}, //X_ADDR_START 784
	{AR1335_16BIT, {0x0348}, 0x0D7F}, //X_ADDR_END 3455
	{AR1335_16BIT, {0x0346}, 0x01E0}, //Y_ADDR_START 480
	{AR1335_16BIT, {0x034A}, 0x0A6D}, //Y_ADDR_END 2973
	{AR1335_16BIT, {0x034C}, 0x0538}, //X_OUTPUT_SIZE 1336
	{AR1335_16BIT, {0x034E}, 0x0448}, //Y_OUTPUT_SIZE 1096
	{AR1335_16BIT, {0x3040}, 0xC043}, //X_BIN, X_ODD_INC, Y_ODD_INC

//Binning Configuration
	{AR1335_16BIT, {0x3172}, 0x0206}, //DIGBIN_ENABLE
	{AR1335_16BIT, {0x317A}, 0x516E}, //SF_BIN_ENABLE
	{AR1335_16BIT, {0x3F3C}, 0x0003}, //SF_BIN_ENABLE

//Scale Configuration
	{AR1335_16BIT, {0x0400}, 0x0001}, //Scaling Enabling: 0= disable, 1= x-dir
	{AR1335_16BIT, {0x0404}, 0x0020}, //Scale_M = 32

//Timing Configuration
	{AR1335_16BIT, {0x0342}, 0x1200},//LINE_LENGTH_PCK 2304
	{AR1335_16BIT, {0x0340}, 0x0C6A},//FRAME_LENGTH_LINES 3178
	{AR1335_16BIT, {0x0202}, 0x0C4B}, //COARSE_INTEGRATION_TIME 3147
	{AR1335_16BIT, {0x3F00}, 0x0044},//BM_T0
	{AR1335_16BIT, {0x3F02}, 0x0128},//BM_T1

	{AR1335_16BIT, {0x0112}, 0x0A0A},//CCP_DATA_FORMAT
	{ AR1335_TOK_TERM, {0}, 0},
};

static const struct ar1335_reg array_setup_4208x3120_27fps[] = {
	{AR1335_16BIT, {0x0344}, 0x0010}, //X_ADDR_START 16	
	{AR1335_16BIT, {0x0348}, 0x107F}, //X_ADDR_END 4223	
	{AR1335_16BIT, {0x0346}, 0x0010}, //Y_ADDR_START 16	
	{AR1335_16BIT, {0x034A}, 0x0C3F}, //Y_ADDR_END 3135	
	{AR1335_16BIT, {0x034C}, 0x1070}, //X_OUTPUT_SIZE 4208	
	{AR1335_16BIT, {0x034E}, 0x0C30}, //Y_OUTPUT_SIZE 3120	

	{AR1335_16BIT, {0x3040}, 0xC041}, //read_mode
	{AR1335_16BIT, {0x3172}, 0x0206}, //digbin_enable
	{AR1335_16BIT, {0x317A}, 0x416E}, //sfbin_enable
	{AR1335_16BIT, {0x3F3C}, 0x0003}, //bin4
        {AR1335_16BIT, {0x0400}, 0x0000}, //Scaling Enabling: 0= disable, 1= x-dir

	{AR1335_16BIT, {0x0342}, 0x1230},//LINE_LENGTH_PCK Default	
	{AR1335_16BIT, {0x0340}, 0x0C4E},//FRAME_LENGTH_LINES 3134	
	{AR1335_16BIT, {0x0202}, 0x0C3C}, //COARSE_INTEGRATION_TIME 3268

	{AR1335_16BIT, {0x0112}, 0x0A0A},//CCP_DATA_FORMAT
	//12fps setting
	{AR1335_16BIT, {0x0342}, 0x13FC},//LINE_LENGTH_PCK 2558
	{AR1335_16BIT, {0x0340}, 0x0C6C},//FRAME_LENGTH_LINES 3180
	{AR1335_16BIT, {0x0202}, 0x0B12}, //COARSE_INTEGRATION_TIME 2834
	{AR1335_16BIT, {0x3F00}, 0x003D},//BM_T0
	{AR1335_16BIT, {0x3F02}, 0x010A},//BM_T1
	{ AR1335_TOK_TERM, {0}, 0},
};
static const struct ar1335_reg array_setup_4208x2368_12fps[] = {
	//[Hidden: 10M_array_setup_4208_2368]
	{AR1335_16BIT, {0x0344}, 0x0010}, //X_ADDR_START 16
	{AR1335_16BIT, {0x0348}, 0x107F}, //X_ADDR_END 4223
	{AR1335_16BIT, {0x0346}, 0x0188}, //Y_ADDR_START 392
	{AR1335_16BIT, {0x034A}, 0x0AC7}, //Y_ADDR_END 2383
	{AR1335_16BIT, {0x034C}, 0x1070}, //X_OUTPUT_SIZE 4208
	{AR1335_16BIT, {0x034E}, 0x0940}, //Y_OUTPUT_SIZE 2368

	{AR1335_16BIT, {0x3040}, 0xC041}, //X_BIN, X_ODD_INC, Y_ODD_INC
	{AR1335_16BIT, {0x3172}, 0x0206}, //DIGBIN_ENABLE
	{AR1335_16BIT, {0x317A}, 0x416E}, //SF_BIN_ENABLE
	{AR1335_16BIT, {0x3F3C}, 0x0003}, //SF_BIN_ENABLE
        {AR1335_16BIT, {0x0400}, 0x0000}, //Scaling Enabling: 0= disable, 1= x-dir

	//REG=0x0342, 0x13FC//LINE_LENGTH_PCK 2558
	//REG=0x0340, 0x1BF6//FRAME_LENGTH_LINES 7158
	//REG=0x0202, 0xB12 //COARSE_INTEGRATION_TIME 2834

        {AR1335_16BIT, {0x0342}, 0x1800},//LINE_LENGTH_PCK 2558
        {AR1335_16BIT, {0x0340}, 0x1700},//FRAME_LENGTH_LINES 2862
        {AR1335_16BIT, {0x0202}, 0x0B12}, //COARSE_INTEGRATION_TIME 2834
	{AR1335_16BIT, {0x3F00}, 0x003D},//BM_T0
	{AR1335_16BIT, {0x3F02}, 0x010A},//BM_T1

	{AR1335_16BIT, {0x0112}, 0x0A0A},//CCP_DATA_FORMAT

	{ AR1335_TOK_TERM, {0}, 0},
};

static const struct ar1335_reg array_setup_4208x3120_12fps[] = {
	{AR1335_16BIT, {0x0344}, 0x0010}, //X_ADDR_START 16	
	{AR1335_16BIT, {0x0348}, 0x107F}, //X_ADDR_END 4223	
	{AR1335_16BIT, {0x0346}, 0x0010}, //Y_ADDR_START 16	
	{AR1335_16BIT, {0x034A}, 0x0C3F}, //Y_ADDR_END 3135	
	{AR1335_16BIT, {0x034C}, 0x1070}, //X_OUTPUT_SIZE 4208	
	{AR1335_16BIT, {0x034E}, 0x0C30}, //Y_OUTPUT_SIZE 3120	

	{AR1335_16BIT, {0x3040}, 0xC041}, //read_mode
	{AR1335_16BIT, {0x3172}, 0x0206}, //digbin_enable
	{AR1335_16BIT, {0x317A}, 0x416E}, //sfbin_enable
	{AR1335_16BIT, {0x3F3C}, 0x0003}, //bin4
        {AR1335_16BIT, {0x0400}, 0x0000}, //Scaling Enabling: 0= disable, 1= x-dir
	{AR1335_16BIT, {0x0342}, 0x1230},//LINE_LENGTH_PCK Default	
	{AR1335_16BIT, {0x0340}, 0x0C4E},//FRAME_LENGTH_LINES 3134	
	{AR1335_16BIT, {0x0202}, 0x0C3C}, //COARSE_INTEGRATION_TIME 3268
	{AR1335_16BIT, {0x3F00}, 0x003D},//BM_T0
	{AR1335_16BIT, {0x3F02}, 0x010A},//BM_T1

	{AR1335_16BIT, {0x0112}, 0x0A0A},//CCP_DATA_FORMAT
	//12fps setting
	{AR1335_16BIT, {0x0342}, 0x1800},//LINE_LENGTH_PCK 2558
        {AR1335_16BIT, {0x0340}, 0x1700},//FRAME_LENGTH_LINES 7158
	{AR1335_16BIT, {0x0202}, 0x0B12}, //COARSE_INTEGRATION_TIME 2834
	{ AR1335_TOK_TERM, {0}, 0},
};

static const struct ar1335_reg array_setup_640x480_30fps[] = {
{AR1335_16BIT,{0x0344}, 0x328},//X_ADDR_START 808
{AR1335_16BIT,{0x0348}, 0xD67},//X_ADDR_END 3431
{AR1335_16BIT,{0x0346}, 0x248},//Y_ADDR_START 584
{AR1335_16BIT,{0x034A}, 0xA01}, //Y_ADDR_END 2785
{AR1335_16BIT,{0x034C}, 0x290}, //X_OUTPUT_SIZE 656
{AR1335_16BIT,{0x034E}, 0x1F0}, //Y_OUTPUT_SIZE 496
{AR1335_16BIT,{0x3040}, 0xC047}, //X_BIN, X_ODD_INC, Y_ODD_INC
//Binning Configuration
{AR1335_16BIT,{0x3172}, 0x206},//DIGBIN_ENABLE
{AR1335_16BIT,{0x317A}, 0x416E},//SF_BIN_ENABLE
{AR1335_16BIT,{0x3F3C}, 0xB}, //SF_BIN_ENABLE
//Scale Configuration
{AR1335_16BIT,{0x0400}, 0x1}, //Scaling Enabling: 0= disable, 1= x-dir
{AR1335_16BIT,{0x0404}, 0x40}, //Scale_M = 64
//Timing Configuration
{AR1335_16BIT,{0x0342}, 0x4AEC},//LINE_LENGTH_PCK 9590
{AR1335_16BIT,{0x0340}, 0x302},//FRAME_LENGTH_LINES 770
{AR1335_16BIT,{0x0202}, 0x30A},//COARSE_INTEGRATION_TIME 778
{ AR1335_TOK_TERM, {0}, 0}
};

static struct ar1335_resolution ar1335_res_preview[] = {
#if 0
	{
        	.desc = "array_setup_640x480_30fps",
         	.width = 656,
         	.height = 496,
         	.used = 0,
         	.regs = array_setup_640x480_30fps,
         	.bin_factor_x = 2,
         	.bin_factor_y = 2,
         	.skip_frames = 0,
         	.fps_options = {
            		{
                	.fps = 30,
                	.pixels_per_line = 0x4AEC, /* 1936*/
                	.lines_per_frame = 0x302, /* 2486 */
	     		},
			{}
		},
		.mipi_freq = 495000, /*990M data rate*/
	},

	{
		 .desc = "array_setup_1636x1096_30fps",
		 .width = 1636,
		 .height = 1096,
		 .used = 0,
		 .regs = array_setup_1636x1096_30fps,
		 .bin_factor_x = 1,
		 .bin_factor_y = 1,
		 .skip_frames = 0,
		 .fps_options = {
			{
				.fps = 30,
				.pixels_per_line = 0x1200, /* 1936*/
				.lines_per_frame = 0xC6A, /* 2486 */
			},
			{
			}
		},
                .mipi_freq = 495000, /*990M data rate*/
	},
#endif
#if 1
	{
		 .desc = "array_setup_1336x1096_30fps",
		 .width = 1336,
		 .height = 1096,
		 .used = 0,
		 .regs = array_setup_1336x1096_30fps,
		 .bin_factor_x = 1,
		 .bin_factor_y = 1,
		 .skip_frames = 0,
		 .fps_options = {
			{
				.fps = 30,
				.pixels_per_line = 0x1200, /* 1936*/
				.lines_per_frame = 0x0C6A, /* 2486 */
			},
			{
			}
		},
                .mipi_freq = 495000, /*990M data rate*/
	},
#endif

	{
		 .desc = "array_setup_2104x1184_30fps",
		 .width = 2104,
		 .height = 1184,
		 .used = 0,
		 .regs = array_setup_2104x1184_30fps,
		 .bin_factor_x = 1,
		 .bin_factor_y = 1,
		 .skip_frames = 0,
		 .fps_options = {
			{
				.fps = 30,
				.pixels_per_line = 0x1200, /* 1936*/
				.lines_per_frame = 0xC6A, /* 2486 */
			},
			{
			}
		},
                .mipi_freq = 495000, /*990M data rate*/
	},
	
	{
		 .desc = "array_setup_2104x1560_30fps",
		 .width = 2104,
		 .height = 1560,
		 .used = 0,
		 .regs = array_setup_2104x1560_30fps,
		 .bin_factor_x = 1,
		 .bin_factor_y = 1,
		 .skip_frames = 0,
		 .fps_options = {
			{
				.fps = 30,
				.pixels_per_line = 0x1200, /* 1936*/
				.lines_per_frame = 0xC6E, /* 2486 */
			},
			{
			}
		},
                .mipi_freq = 495000, /*990M data rate*/
	},
	
	{
		 .desc = "array_setup_4208x2368_30fps",
		 .width = 4208,
		 .height = 2368,
		 .used = 0,
		 .regs = array_setup_4208x2368_30fps,
		 .bin_factor_x = 0,
		 .bin_factor_y = 0,
		 .skip_frames = 0,
		 .fps_options = {
			{
				.fps = 30,
				.pixels_per_line = 0x13FC, /* 1936*/
				.lines_per_frame = 0x0B2E, /* 2486 */
			},
			{
			}
		},
                .mipi_freq = 495000, /*990M data rate*/
	},
	
	{
		 .desc = "array_setup_4208x3120_27fps",
		 .width = 4208,
		 .height = 3120,
		 .used = 0,
		 .regs = array_setup_4208x3120_27fps,
		 .bin_factor_x = 0,
		 .bin_factor_y = 0,
		 .skip_frames = 0,
		 .fps_options = {
			{
				.fps = 27,
				.pixels_per_line = 0x13FC, /* 1936*/
				.lines_per_frame = 0x0C6C, /* 2486 */
			},
			{
			}
		},
                .mipi_freq = 495000, /*990M data rate*/
	},
};

static struct ar1335_resolution ar1335_res_still[] = {
#if 0
	{
        	.desc = "array_setup_640x480_30fps",
         	.width = 656,
         	.height = 496,
         	.used = 0,
         	.regs = array_setup_640x480_30fps,
         	.bin_factor_x = 2,
         	.bin_factor_y = 2,
         	.skip_frames = 0,
         	.fps_options = {
            		{
                	.fps = 30,
                	.pixels_per_line = 0x4AEC, /* 1936*/
                	.lines_per_frame = 0x302, /* 2486 */
	     		},
			{}
		},
		.mipi_freq = 495000, /*990M data rate*/
	},



#endif

#if 1
	{
		 .desc = "array_setup_1336x1096_30fps",
		 .width = 1336,
		 .height = 1096,
		 .used = 0,
		 .regs = array_setup_1336x1096_30fps,
		 .bin_factor_x = 1,
		 .bin_factor_y = 1,
		 .skip_frames = 0,
		 .fps_options = {
			{
				.fps = 30,
				.pixels_per_line = 0x1200, /* 1936*/
				.lines_per_frame = 0x0C6A, /* 2486 */
			},
			{
			}
		},
                .mipi_freq = 495000, /*990M data rate*/
	},
#endif
	{
		 .desc = "array_setup_2104x1184_30fps",
		 .width = 2104,
		 .height = 1184,
		 .used = 0,
		 .regs = array_setup_2104x1184_30fps,
		 .bin_factor_x = 1,
		 .bin_factor_y = 1,
		 .skip_frames = 0,
		 .fps_options = {
			{
				.fps = 30,
				.pixels_per_line = 0x1200, /* 1936*/
				.lines_per_frame = 0xC6A, /* 2486 */
			},
			{
			}
		},
                .mipi_freq = 495000, /*990M data rate*/
	},

	{
		 .desc = "array_setup_2104x1560_30fps",
		 .width = 2104,
		 .height = 1560,
		 .used = 0,
		 .regs = array_setup_2104x1560_30fps,
		 .bin_factor_x = 1,
		 .bin_factor_y = 1,
		 .skip_frames = 0,
		 .fps_options = {
			{
				.fps = 30,
				.pixels_per_line = 0x1200, /* 1936*/
				.lines_per_frame = 0xC6E, /* 2486 */
			},
			{
			}
		},
                .mipi_freq = 495000, /*990M data rate*/
	},

	{
		 .desc = "array_setup_4208x2368_12fps",
		 .width = 4208,
		 .height = 2368,
		 .used = 0,
		 .regs = array_setup_4208x2368_12fps,
		 .bin_factor_x = 0,
		 .bin_factor_y = 0,
		 .skip_frames = 0,
		 .fps_options = {
			{
				.fps = 12,
				.pixels_per_line = 0x1800, /* 1936*/
				.lines_per_frame = 0x1700, /* 2486 */
			},
			{
			}
		},
                .mipi_freq = 495000, /*990M data rate*/
	},
	
	{
		 .desc = "array_setup_4208x3120_12fps",
		 .width = 4208,
		 .height = 3120,
		 .used = 0,
		 .regs = array_setup_4208x3120_12fps,
		 .bin_factor_x = 0,
		 .bin_factor_y = 0,
		 .skip_frames = 0,
		 .fps_options = {
			{
				.fps = 12,
				.pixels_per_line = 0x1800, /* 1936*/
				.lines_per_frame = 0x1700, /* 2486 */
			},
			{
			}
		},
                .mipi_freq = 495000, /*990M data rate*/
	},

};

static struct ar1335_resolution ar1335_res_video[] = {
#if 0
	{
        	.desc = "array_setup_640x480_30fps",
         	.width = 656,
         	.height = 496,
         	.used = 0,
         	.regs = array_setup_640x480_30fps,
         	.bin_factor_x = 2,
         	.bin_factor_y = 2,
         	.skip_frames = 0,
         	.fps_options = {
            		{
                	.fps = 30,
                	.pixels_per_line = 0x4AEC, /* 1936*/
                	.lines_per_frame = 0x302, /* 2486 */
	     		},
			{}
		},
		.mipi_freq = 495000, /*990M data rate*/
	},
#endif
	{
		 .desc = "array_setup_1636x1096_30fps",
		 .width = 1636,
		 .height = 1096,
		 .used = 0,
		 .regs = array_setup_1636x1096_30fps,
		 .bin_factor_x = 1,
		 .bin_factor_y = 1,
		 .skip_frames = 0,
		 .fps_options = {
			{
				.fps = 30,
				.pixels_per_line = 0x1200, /* 1936*/
				.lines_per_frame = 0x0C6A, /* 2486 */
			},
			{
			}
		},
                .mipi_freq = 495000, /*990M data rate*/
	},

	{
		 .desc = "array_setup_1936x1096_30fps",
		 .width = 1936,
		 .height = 1096,
		 .used = 0,
		 .regs = array_setup_1936x1096_30fps,
		 .bin_factor_x = 1,
		 .bin_factor_y = 1,
		 .skip_frames = 0,
		 .fps_options = {
			{
				.fps = 30,
				.pixels_per_line = 0x1200, /* 1936*/
				.lines_per_frame = 0x0C6A, /* 2486 */
			},
			{
			}
		},
                .mipi_freq = 495000, /*990M data rate*/
	},
	
	{
                 .desc = "array_setup_1636x1096_30fps",
                 .width = 1636,
                 .height = 1096,
                 .used = 0,
                 .regs = array_setup_1636x1096_30fps,
                 .bin_factor_x = 1,
                 .bin_factor_y = 1,
                 .skip_frames = 0,
                 .fps_options = {
                        {
                                .fps = 30,
                                .pixels_per_line = 0x1230, /* 1936*/
                                .lines_per_frame = 0x0898, /* 2486 */
                        },
                        {
                        }
                },
                .mipi_freq = 230000, /*990M data rate*/
        },

	{
		 .desc = "array_setup_1336x1096_30fps",
		 .width = 1336,
		 .height = 1096,
		 .used = 0,
		 .regs = array_setup_1336x1096_30fps,
		 .bin_factor_x = 1,
		 .bin_factor_y = 1,
		 .skip_frames = 0,
		 .fps_options = {
			{
				.fps = 30,
				.pixels_per_line = 0x1200, /* 1936*/
				.lines_per_frame = 0x0C6A, /* 2486 */
			},
			{
			}
		},
                .mipi_freq = 495000, /*990M data rate*/
	},
	
	{
		 .desc = "array_setup_2104x1560_30fps",
		 .width = 2104,
		 .height = 1560,
		 .used = 0,
		 .regs = array_setup_2104x1560_30fps,
		 .bin_factor_x = 1,
		 .bin_factor_y = 1,
		 .skip_frames = 0,
		 .fps_options = {
			{
				.fps = 30,
				.pixels_per_line = 0x1200, /* 1936*/
				.lines_per_frame = 0xC6E, /* 2486 */
			},
			{
			}
		},
                .mipi_freq = 495000, /*990M data rate*/
	},
};

static int
ar1335_read_reg(struct i2c_client *client, u16 len, u16 reg, u16 *val);

static int
ar1335_write_reg(struct i2c_client *client, u16 data_length, u16 reg, u16 val);

#endif
