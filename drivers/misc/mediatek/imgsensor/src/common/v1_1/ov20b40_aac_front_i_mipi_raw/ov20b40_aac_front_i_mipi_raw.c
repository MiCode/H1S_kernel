/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

/*****************************************************************************
 *
 * Filename:
 * ---------
 *     ov20b40_aac_front_i_mipi_raw.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 *
 * Version:  V20240314112457 by GC-S-TEAM
 *

 */
#define PFX "ov20b40_camera_sensor"
#define pr_fmt(fmt) PFX "[%s] " fmt, __func__

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "imgsensor_custom.h"

#include "ov20b40_aac_front_i_mipi_raw.h"
#define LOG_INF(format, args...)    \
    pr_debug(PFX "[%s] " format, __func__, ##args)

static DEFINE_SPINLOCK(imgsensor_drv_lock);

static IMGSENSOR_GET_CUSTOM_INFO_STRUCT custom_info = {
    .sensor_type = SENSOR_FRONT_MAIN,
	.module_head = "_aac",
	.fuse_id = "",
	.fuse_id_len = 16,
	.sn_info = "",
	.sn_info_len = 14,
	.vendor_id = 0,
	.lens_id = 0,
	.sensor_id = 0,
};

static IMGSENSOR_CUSTOM_INFO_ADDR_STRUCT addr_info = {
    .eeprom_i2c_addr = 0xA2,
	.basic_info_flag_add = 0x00,
	.fuse_id_addr = 0x10,
	.sn_flag_addr = 0x1F91,
	.sn_info_addr = 0X1F92,
	.vendor_id_addr = 0x01,
	.lens_id_addr = 0x08,
	.sensor_id_addr = 0x0c,
};

static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = OV20B40_AAC_FRONT_I_SENSOR_ID,
	.checksum_value = 0xe5d32119,
	.pre = { //01_OV20B40_4C1_2592x1944_30fps_DPHY_4Lane_850M_24M
		.pclk = 50000000,
		.linelength = 500,
		.framelength = 3333,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2592,
		.grabwindow_height = 1944,
		.mipi_data_lp2hs_settle_dc = 10,
		.mipi_pixel_rate = 436800000,
		.max_framerate = 300,
	},

	.cap = { //01_OV20B40_4C1_2592x1944_30fps_DPHY_4Lane_850M_24M
		.pclk = 50000000,
		.linelength = 500,
		.framelength = 3333,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2592,
		.grabwindow_height = 1944,
		.mipi_data_lp2hs_settle_dc = 10,
		.mipi_pixel_rate = 436800000,
		.max_framerate = 300,
	},
	.normal_video = { //02_OV20B40_4C1_2592x1464_30fps_DPHY_4Lane_850M_24M
		.pclk = 50000000,
		.linelength = 500,
		.framelength = 3333,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2592,
		.grabwindow_height = 1464,
		.mipi_data_lp2hs_settle_dc = 10,
		.mipi_pixel_rate = 436800000,
		.max_framerate = 300,
	},
	.hs_video = { //03_OV20B40_4C1_V2aH2d_1280x720_120fps_DPHY_4Lane_850M_24M
		.pclk = 50000000,
		.linelength = 500,
		.framelength = 833,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1280,
		.grabwindow_height = 720,
		.mipi_data_lp2hs_settle_dc = 10,
		.mipi_pixel_rate = 436800000,
		.max_framerate = 1200,
	},
	.slim_video = { // 02_OV20B40_4C1_2592x1464_30fps_DPHY_4Lane_850M_24M
		.pclk = 50000000,
		.linelength = 500,
		.framelength = 3333,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2592,
		.grabwindow_height = 1464,
		.mipi_data_lp2hs_settle_dc = 10,
		.mipi_pixel_rate = 436800000,
		.max_framerate = 300,
	},
	.custom1 = { // 01_OV20B40_4C1_2592x1944_30fps_DPHY_4Lane_850M_24M
		.pclk = 50000000,
		.linelength = 500,
		.framelength = 3333,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2592,
		.grabwindow_height = 1944,
		.mipi_data_lp2hs_settle_dc = 10,
		.mipi_pixel_rate = 436800000,
		.max_framerate = 300,
	},
	.custom2 = { // 01_OV20B40_4C1_2592x1944_30fps_DPHY_4Lane_850M_24M
		.pclk = 50000000,
		.linelength = 500,
		.framelength = 3333,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2592,
		.grabwindow_height = 1944,
		.mipi_data_lp2hs_settle_dc = 10,
		.mipi_pixel_rate = 436800000,
		.max_framerate = 300,
	},
	.custom3 = { // 02_OV20B40_4C1_2592x1464_30fps_DPHY_4Lane_850M_24M
		.pclk = 50000000,
		.linelength = 500,
		.framelength = 3333,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2592,
		.grabwindow_height = 1464,
		.mipi_data_lp2hs_settle_dc = 10,
		.mipi_pixel_rate = 436800000,
		.max_framerate = 300,
	},

	.margin = 4,
	.min_shutter = 2,
	.min_gain = 64,/* 1x */
	.max_gain = 15.96875*64,
	.min_gain_iso = 50,
	.exp_step = 1,
	.gain_step = 1,
	.gain_type = 1,
	.max_frame_length = 0xffff,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,	/* isp gain delay frame for AE cycle */
	.ihdr_support = 0,	/* 1, support; 0,not support */
	.ihdr_le_firstline = 0,	/* 1, le first ; 0, se first */
	.temperature_support = 0,/* 1, support; 0, not support */
	.sensor_mode_num = 8,	/* support sensor mode num */
	.cap_delay_frame = 2,	/* enter capture delay frame num */
	.pre_delay_frame = 2,	/* enter preview delay frame num */
	.video_delay_frame = 2,	/* enter video delay frame num */
	.hs_video_delay_frame = 2,
	.slim_video_delay_frame = 2,	/* enter slim video delay frame num */
	.custom1_delay_frame = 2,
	.custom2_delay_frame = 2,
	.custom3_delay_frame = 2,
	.frame_time_delay_frame = 2,
	.isp_driving_current = ISP_DRIVING_4MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO,
    .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_B,
	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.i2c_addr_table = {0x20, 0xff},
	/* record sensor support all write id addr,
	 * only supprt 4 must end with 0xff
	 */
	.i2c_speed = 400,
};

static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_HV_MIRROR,	/* mirrorflip information */
	.sensor_mode = IMGSENSOR_MODE_INIT,
	/* IMGSENSOR_MODE enum value,record current sensor mode,such as:
	 * INIT, Preview, Capture, Video,High Speed Video, Slim Video
	 */
	.shutter = 0x480,
	.gain = 0x40,
	.dummy_pixel = 0,	/* current dummypixel */
	.dummy_line = 0,	/* current dummyline */
	.current_fps = 300,
	.autoflicker_en = KAL_FALSE,
	.test_pattern = KAL_FALSE,
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,
	.ihdr_mode = 0, /* sensor need support LE, SE with HDR feature */
	.i2c_write_id = 0x20, /* record current sensor's i2c write id */
};

/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[8] = {
      { 2592, 1944,   0,   0, 2592, 1944, 2592, 1944, 0, 0, 2592, 1944, 0, 0, 2592, 1944}, /* Preview */
      { 2592, 1944,   0,   0, 2592, 1944, 2592, 1944, 0, 0, 2592, 1944, 0, 0, 2592, 1944}, /* capture */
      { 2592, 1944,   0, 240, 2592, 1464, 2592, 1464, 0, 0, 2592, 1464, 0, 0, 2592, 1464}, /* video */
      { 2592, 1944,  16, 252, 2560, 1440, 1280,  720, 0, 0, 1280,  720, 0, 0, 1280,  720}, /* hs_video */
      { 2592, 1944,   0, 240, 2592, 1464, 2592, 1464, 0, 0, 2592, 1464, 0, 0, 2592, 1464}, /* slim video */
      { 2592, 1944,   0,   0, 2592, 1944, 2592, 1944, 0, 0, 2592, 1944, 0, 0, 2592, 1944}, /* Custom1 */
      { 2592, 1944,   0,   0, 2592, 1944, 2592, 1944, 0, 0, 2592, 1944, 0, 0, 2592, 1944}, /* Custom2 */
      { 2592, 1944,   0, 240, 2592, 1464, 2592, 1464, 0, 0, 2592, 1464, 0, 0, 2592, 1464}  /* Custom3 */
};


static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pu_send_cmd[2] = { 
		(char)((addr >> 8) & 0xff), 
		(char)(addr & 0xff) 
	};

	iReadRegI2C(pu_send_cmd, 2, (u8 *)&get_byte, 1, imgsensor.i2c_write_id);

	return get_byte;
}

void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[4] = {
		(char)((addr >> 8) & 0xff),
		(char)(addr & 0xff),
		(char)((para >> 8) & 0xff),
		(char)(para & 0xff)
	};

	iWriteRegI2C(pu_send_cmd, 4, imgsensor.i2c_write_id);
}

static void write_cmos_sensor_8bit(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[3] = { 
		(char)((addr >> 8) & 0xff), 
		(char)(addr & 0xff), 
		(char)(para & 0xff) 
	};

	iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}


static void table_write_cmos_sensor(kal_uint16 *para, kal_uint32 len)
{
	char puSendCmd[I2C_BUFFER_LEN];
	kal_uint32 tosend = 0, idx = 0;
	kal_uint16 addr = 0, data = 0;

	while (len > idx) {
		addr = para[idx];
		puSendCmd[tosend++] = (char)((addr >> 8) & 0xff);
		puSendCmd[tosend++] = (char)(addr & 0xff);
		data = para[idx + 1];
		puSendCmd[tosend++] = (char)(data & 0xff);
		idx += 2;
#if MULTI_WRITE
		if (tosend >= I2C_BUFFER_LEN || idx == len) {
			iBurstWriteReg_multi(puSendCmd, tosend, imgsensor.i2c_write_id,
					3, imgsensor_info.i2c_speed);
			tosend = 0;
		}
#else
		iWriteRegI2CTiming(puSendCmd, 3, imgsensor.i2c_write_id, imgsensor_info.i2c_speed);
		tosend = 0;
#endif
	}
}

static kal_uint32 return_sensor_id(void)
{
	kal_uint32 sensor_id = 0;

	sensor_id = (read_cmos_sensor(0x300b) << 8) | read_cmos_sensor(0x300c);
	return sensor_id;
}

static void set_dummy(void)
{
	pr_debug("frame length = %d\n", imgsensor.frame_length);	
	write_cmos_sensor_8bit(0x3840, imgsensor.frame_length >> 16);
	write_cmos_sensor_8bit(0x380e, imgsensor.frame_length >> 8);
	write_cmos_sensor_8bit(0x380f, imgsensor.frame_length & 0xFF);
}	/*	set_dummy  */


static void set_max_framerate(kal_uint16 framerate, kal_bool min_framelength_en)
{
	/*kal_int16 dummy_line;*/
	kal_uint32 frame_length = imgsensor.frame_length;

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	if (frame_length >= imgsensor.min_frame_length)
		imgsensor.frame_length = frame_length;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;

	imgsensor.dummy_line =
			imgsensor.frame_length - imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line =
			imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}	/*	set_max_framerate  */


static void set_mirror_flip(kal_uint8 image_mirror)
{
}

static void write_shutter(kal_uint32 shutter)
{
	kal_uint16 realtime_fps = 0;

	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	if (shutter < imgsensor_info.min_shutter)
		shutter = imgsensor_info.min_shutter;

	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ?
		(imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;
	realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
	if (imgsensor.autoflicker_en) {
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
			write_cmos_sensor_8bit(0x3840, imgsensor.frame_length >> 16);
	                write_cmos_sensor_8bit(0x380e, imgsensor.frame_length >> 8);
	                write_cmos_sensor_8bit(0x380f, imgsensor.frame_length & 0xFF);
		}
	} else {
		write_cmos_sensor_8bit(0x3840, imgsensor.frame_length >> 16);
	        write_cmos_sensor_8bit(0x380e, imgsensor.frame_length >> 8);
	        write_cmos_sensor_8bit(0x380f, imgsensor.frame_length & 0xFF);
	}
		  
	write_cmos_sensor_8bit(0x3500, (shutter >> 16) & 0xFF);
	write_cmos_sensor_8bit(0x3501, (shutter >>  8) & 0xFF);
	write_cmos_sensor_8bit(0x3502,  shutter & 0xFF);
	
	pr_debug("shutter =%d, framelength =%d\n",
		shutter, imgsensor.frame_length);
}	/*	write_shutter  */

/*************************************************************************
 * FUNCTION
 *	set_shutter
 *
 * DESCRIPTION
 *	This function set e-shutter of sensor to change exposure time.
 *
 * PARAMETERS
 *	iShutter : exposured lines
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static void set_shutter(kal_uint32 shutter)
{
	unsigned long flags;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	write_shutter(shutter);
} /* set_shutter */


/*************************************************************************
 * FUNCTION
 *	set_shutter_frame_length
 *
 * DESCRIPTION
 *	for frame & 3A sync
 *
 *************************************************************************/
static void set_shutter_frame_length(kal_uint32 shutter,
				     kal_uint16 frame_length)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;
	kal_int32 dummy_line = 0;
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	/* if shutter bigger than frame_length, should extend frame length first*/
	spin_lock(&imgsensor_drv_lock);
	if (frame_length > 1)
		dummy_line = frame_length - imgsensor.frame_length;

	imgsensor.frame_length = imgsensor.frame_length + dummy_line;

	if (shutter > imgsensor.frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter)
			? imgsensor_info.min_shutter : shutter;
	shutter =
	(shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin))
		? (imgsensor_info.max_frame_length - imgsensor_info.margin)
		: shutter;
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
	if (imgsensor.autoflicker_en) {
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
			write_cmos_sensor_8bit(0x3840, imgsensor.frame_length >> 16);
	                write_cmos_sensor_8bit(0x380e, imgsensor.frame_length >> 8);
	                write_cmos_sensor_8bit(0x380f, imgsensor.frame_length & 0xFF);
		}
	} else {
		write_cmos_sensor_8bit(0x3840, imgsensor.frame_length >> 16);
	        write_cmos_sensor_8bit(0x380e, imgsensor.frame_length >> 8);
	        write_cmos_sensor_8bit(0x380f, imgsensor.frame_length & 0xFF);
	}
	
	/* Update Shutter */
	write_cmos_sensor_8bit(0x3500, (shutter >> 16) & 0xFF);
	write_cmos_sensor_8bit(0x3501, (shutter >>  8) & 0xFF);
	write_cmos_sensor_8bit(0x3502,  shutter & 0xFF);
	
	pr_debug(
		"Exit! shutter =%d, framelength =%d/%d, dummy_line=%d,\n",
		shutter, imgsensor.frame_length, frame_length,
		dummy_line);

}	/* set_shutter_frame_length */

/*************************************************************************
 * FUNCTION
 *	set_gain
 *
 * DESCRIPTION
 *	This function is to set global gain to sensor.
 *
 * PARAMETERS
 *	iGain : sensor global gain(base: 0x40)
 *
 * RETURNS
 *	the actually gain set to sensor.
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/

static kal_uint32 gain2reg(const kal_uint16 gain)
{
	kal_uint32 reg_gain = 0x0;

	reg_gain = gain * 4;// need to check whether 1x = 256 (0x100)
	
	return (kal_uint32) reg_gain;
}
static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint32 reg_gain, max_gain = imgsensor_info.max_gain;

	if (gain < imgsensor_info.min_gain || gain > max_gain) {
		pr_debug("Error max gain setting: %d\n", max_gain);

		if (gain < imgsensor_info.min_gain)
			gain = imgsensor_info.min_gain;
		else if (gain > max_gain)
			gain = max_gain;
	}

	reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	pr_debug("gain = %d, reg_gain = 0x%x, max_gain:0x%x\n ",
		gain, reg_gain, max_gain);
	write_cmos_sensor_8bit(0x3508, reg_gain >> 8);
	write_cmos_sensor_8bit(0x3509, reg_gain & 0xFF);

	return gain;
} /* set_gain */

/*************************************************************************
 * FUNCTION
 *	night_mode
 *
 * DESCRIPTION
 *	This function night mode of sensor.
 *
 * PARAMETERS
 *	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 streaming_control(kal_bool enable)
{
	pr_debug("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);
	if (enable) {
		write_cmos_sensor_8bit(0x0100, 0x01);
	} else {
		write_cmos_sensor_8bit(0x0100, 0x00);
	}
	return ERROR_NONE;
}

static kal_uint16 ov20b40_init_addr_data[] = {
	0x36af, 0x00,
	0x1222, 0x00,
	0x0103, 0x01,
	0x1003, 0x01,
	0x1001, 0x04,
	0x5000, 0x55,
	0x5161, 0x14,
	0x5162, 0x00,
	0x5163, 0x6b,
	0x5460, 0x00,
	0x5461, 0x14,
	0x5462, 0x00,
	0x5463, 0x6b,
	0x51a2, 0x83,
	0x51a4, 0x0c,
	0x51a5, 0x0c,
	0x51a6, 0x0c,
	0x51a7, 0x0c,
	0x51a8, 0x0c,
	0x51a9, 0x0c,
	0x51aa, 0x0c,
	0x51ab, 0x0c,
	0x54a4, 0x0c,
	0x54a5, 0x0c,
	0x54a6, 0x0c,
	0x54a7, 0x0c,
	0x54a8, 0x0c,
	0x54a9, 0x0c,
	0x54aa, 0x0c,
	0x54ab, 0x0c,
	0x5008, 0xb0,
	0x3640, 0xa6,
	0x3923, 0x03,
	0x3924, 0x8a,
	0x391b, 0x11,
	0x3641, 0xa2,
	0x3925, 0x00,
	0x3926, 0x03,
	0x391c, 0x00,
	0x3642, 0xa2,
	0x3927, 0x00,
	0x3928, 0x06,
	0x391d, 0x01,
	0x3643, 0xa2,
	0x3929, 0x00,
	0x392a, 0x0b,
	0x391e, 0x00,
	0x3ad2, 0x00,
	0x3ad4, 0x00,
	0x3650, 0xc0,
	0x3651, 0xc0,
	0x3652, 0xc0,
	0x3653, 0xc0,
	0x3654, 0xc0,
	0x3655, 0xc0,
	0x3656, 0xc0,
	0x3657, 0xc0,
	0x3a3e, 0x06,
	0x3a6f, 0x12,
	0x3a55, 0x12,
	0x3a3b, 0xb4,
	0x3a41, 0xb3,
	0x3a35, 0x49,
	0x3a36, 0x50,
	0x3bd0, 0x52,
	0x3bd1, 0x53,
	0x3be0, 0x26,
	0x3a73, 0x21,
	0x3a4f, 0x21,
	0x3a44, 0x21,
	0x3be1, 0x3a,
	0x3a45, 0x42,
	0x3a4e, 0x42,
	0x3a72, 0x42,
	0x3be2, 0x63,
	0x3a51, 0x5e,
	0x3a75, 0x5e,
	0x3a46, 0x63,
	0x3a4a, 0x63,
	0x3be3, 0x71,
	0x3a4b, 0x7b,
	0x3a47, 0x7b,
	0x3a50, 0x7b,
	0x3a74, 0x7b,
	0x3ae0, 0x00,
	0x3ae1, 0x01,
	0x3ae4, 0x00,
	0x3ae5, 0x01,
	0x3ae8, 0x00,
	0x3ae9, 0x01,
	0x3aec, 0x00,
	0x3aed, 0x01,
	0x3af2, 0x00,
	0x3af3, 0x01,
	0x3af6, 0x00,
	0x3af7, 0x01,
	0x3afc, 0x00,
	0x3afd, 0x01,
	0x3b00, 0x00,
	0x3b01, 0x01,
	0x3b04, 0x00,
	0x3b05, 0x01,
	0x3b08, 0x00,
	0x3b09, 0x01,
	0x3b0c, 0x00,
	0x3b0d, 0x01,
	0x3b10, 0x00,
	0x3b11, 0x01,
	0x3b14, 0x00,
	0x3b15, 0x01,
	0x3b18, 0x00,
	0x3b19, 0x01,
	0x3b1c, 0x00,
	0x3b1d, 0x01,
	0x3b20, 0x00,
	0x3b21, 0x01,
	0x3ae2, 0x04,
	0x3ae6, 0x04,
	0x3aea, 0x04,
	0x3aee, 0x04,
	0x3af4, 0x04,
	0x3af8, 0x04,
	0x3afe, 0x04,
	0x3b02, 0x04,
	0x3ae3, 0x1c,
	0x3ae7, 0x1e,
	0x3aeb, 0x20,
	0x3aef, 0x23,
	0x3af5, 0x35,
	0x3af9, 0x40,
	0x3aff, 0x40,
	0x3b03, 0x40,
	0x3b06, 0x04,
	0x3b0a, 0x04,
	0x3b0e, 0x04,
	0x3b12, 0x04,
	0x3b16, 0x04,
	0x3b1a, 0x04,
	0x3b1e, 0x04,
	0x3b22, 0x04,
	0x3b07, 0x1c,
	0x3b0b, 0x1e,
	0x3b0f, 0x20,
	0x3b13, 0x23,
	0x3b17, 0x35,
	0x3b1b, 0x40,
	0x3b1f, 0x40,
	0x3b23, 0x40,
	0x3aa0, 0x00,
	0x3aa1, 0x01,
	0x3457, 0xc8,
	0x3a08, 0x0d,
	0x3a11, 0x21,
	0x3913, 0x0a,
	0x3914, 0x0a,
	0x3915, 0x0a,
	0x3916, 0x0a,
	0x3917, 0x0a,
	0x3918, 0x0a,
	0x3919, 0x0a,
	0x391a, 0x0a,
	0x3a05, 0x0e,
	0x3425, 0xf0,
	0x3422, 0x00,
	0x3658, 0x00,
	0x3659, 0x00,
	0x365a, 0x00,
	0x365b, 0x00,
	0x365c, 0x00,
	0x365d, 0x00,
	0x365e, 0x00,
	0x365f, 0x00,
	0x3900, 0x13,
	0x3901, 0x0c,
	0x3902, 0x0c,
	0x3903, 0x0c,
	0x3904, 0x13,
	0x3905, 0x0c,
	0x3906, 0x0c,
	0x3907, 0x0c,
	0x3908, 0x01,
	0x3909, 0x01,
	0x390a, 0x01,
	0x390b, 0x01,
	0x390c, 0x01,
	0x390d, 0x01,
	0x390e, 0x01,
	0x390f, 0x01,
	0x3624, 0x40,
	0x361f, 0x16,
	0x361e, 0x98,
	0x3638, 0xa9,
	0x3639, 0xa9,
	0x363a, 0xa9,
	0x363b, 0xa9,
	0x363c, 0xa9,
	0x363d, 0xa9,
	0x363e, 0xa9,
	0x363f, 0xa9,
	0x3648, 0x04,
	0x3649, 0x04,
	0x364a, 0x04,
	0x364b, 0x04,
	0x364c, 0x04,
	0x364d, 0x04,
	0x364e, 0x04,
	0x364f, 0x04,
	0x3912, 0xf0,
	0x396c, 0x09,
	0x3948, 0x0b,
	0x3949, 0x0b,
	0x394a, 0x0b,
	0x394b, 0x0b,
	0x3950, 0x1a,
	0x3951, 0x1a,
	0x3952, 0x1a,
	0x3953, 0x1a,
	0x394c, 0x0b,
	0x394d, 0x0b,
	0x394e, 0x0b,
	0x394f, 0x0b,
	0x3954, 0x1a,
	0x3955, 0x1a,
	0x3956, 0x1a,
	0x3957, 0x1a,
	0x3b24, 0x00,
	0x3b25, 0x01,
	0x3b26, 0x04,
	0x3b27, 0x1a,
	0x3601, 0x44,
	0x3a0a, 0x66,
	0x3ac8, 0x01,
	0x3ac9, 0x00,
	0x3aca, 0x00,
	0x3acb, 0x00,
	0x3a78, 0x02,
	0x3a79, 0x63,
	0x3a7c, 0x02,
	0x3a7d, 0x63,
	0x3a80, 0x63,
	0x3a81, 0x02,
	0x3a82, 0xb3,
	0x3a83, 0x6e,
	0x3a86, 0x63,
	0x3a87, 0x02,
	0x3a88, 0xb3,
	0x3a89, 0x6e,
	0x3a8c, 0x76,
	0x3a8d, 0xb1,
	0x3a90, 0x76,
	0x3a91, 0xb1,
	0x4003, 0x40,
	0x3658, 0x00,
	0x3659, 0x00,
	0x365a, 0x00,
	0x365b, 0x00,
	0x365c, 0x00,
	0x365d, 0x00,
	0x365e, 0x00,
	0x365f, 0x00,
	0x3ba4, 0x63,
	0x3ba8, 0xaf,
	0x0300, 0x00,
	0x0301, 0xc8,
	0x0304, 0x02,//0x01,
	0x0305, 0x22,//0xa9, 1092Mbps
	0x0306, 0x04,
	0x0307, 0x01,
	0x0309, 0x13,//0x12,
	0x0320, 0x02,
	0x0328, 0x04,
	0x032a, 0x06,
	0x032b, 0x02,
	0x0360, 0x01,
	0x0361, 0x00,
	0x1216, 0x00,
	0x1217, 0x00,
	0x1218, 0x00,
	0x2a00, 0x00,
	0x2a06, 0x1f,
	0x2a07, 0x40,
	0x3012, 0x41,
	0x3015, 0x10,
	0x3016, 0xb0,
	0x3017, 0xf0,
	0x3018, 0xf0,
	0x3019, 0xd2,
	0x301a, 0xb0,
	0x301e, 0x80,
	0x3025, 0x89,
	0x3026, 0x00,
	0x3027, 0x01,
	0x3029, 0x00,
	0x3030, 0x03,
	0x3044, 0xc2,
	0x3051, 0x60,
	0x3054, 0x02,
	0x3056, 0x80,
	0x3057, 0x80,
	0x3058, 0x80,
	0x3400, 0x1c,
	0x3401, 0x80,
	0x3402, 0xaf,
	0x3403, 0x57,
	0x3404, 0x01,
	0x3405, 0xe3,
	0x3406, 0xaa,
	0x3407, 0x01,
	0x3408, 0xe3,
	0x3409, 0xaa,
	0x340a, 0x02,
	0x340b, 0x2d,
	0x340d, 0x21,
	0x3419, 0x1a,
	0x341a, 0x0b,
	0x341b, 0x65,
	0x3420, 0x00,
	0x3421, 0x00,
	0x3422, 0x00,
	0x3423, 0x10,
	0x3424, 0x00,
	0x3425, 0xf0,
	0x3427, 0x00,
	0x3428, 0x00,
	0x342a, 0x10,
	0x342b, 0x00,
	0x342c, 0x00,
	0x342d, 0x00,
	0x342e, 0x00,
	0x342f, 0x00,
	0x3431, 0x00,
	0x3432, 0x00,
	0x3433, 0x00,
	0x3434, 0x00,
	0x3435, 0x00,
	0x3436, 0x00,
	0x3437, 0x00,
	0x3438, 0x00,
	0x3439, 0x00,
	0x343a, 0x00,
	0x343b, 0x00,
	0x343c, 0x00,
	0x343d, 0x00,
	0x343e, 0x00,
	0x343f, 0x00,
	0x3440, 0x00,
	0x3441, 0x00,
	0x3450, 0x02,
	0x3451, 0x02,
	0x3452, 0x02,
	0x3453, 0x02,
	0x3454, 0x08,
	0x3459, 0x00,
	0x345a, 0x00,
	0x3460, 0x04,
	0x3461, 0x04,
	0x3462, 0x04,
	0x3463, 0x04,
	0x3464, 0x28,
	0x3500, 0x00,
	0x3501, 0x01,
	0x3502, 0x00,
	0x3503, 0xa8,
	0x3504, 0x4c,
	0x3506, 0x07,
	0x3507, 0x00,
	0x3508, 0x01,
	0x3509, 0x00,
	0x350a, 0x01,
	0x350b, 0x00,
	0x350c, 0x00,
	0x3540, 0x00,
	0x3541, 0x00,
	0x3542, 0x80,
	0x3544, 0x48,
	0x3546, 0x07,
	0x3547, 0x00,
	0x3548, 0x01,
	0x3549, 0x00,
	0x354a, 0x01,
	0x354b, 0x00,
	0x354c, 0x00,
	0x3682, 0x00,
	0x368a, 0x2e,
	0x368e, 0x71,
	0x3695, 0x10,
	0x3696, 0xd1,
	0x36a4, 0x00,
	0x3712, 0x50,
	0x3714, 0x22,
	0x3720, 0x08,
	0x3727, 0x05,
	0x3760, 0x02,
	0x3761, 0x24,
	0x3762, 0x02,
	0x3763, 0x02,
	0x3764, 0x02,
	0x3765, 0x2c,
	0x3766, 0x04,
	0x3767, 0x2c,
	0x3768, 0x00,
	0x3769, 0x00,
	0x376b, 0x20,
	0x376e, 0x07,
	0x37b0, 0x00,
	0x37b1, 0xab,
	0x37b2, 0x01,
	0x37b3, 0x82,
	0x37b4, 0x00,
	0x37b5, 0xe4,
	0x37b6, 0x01,
	0x37b7, 0xee,
	0x37c0, 0x02,
	0x3800, 0x00,
	0x3801, 0x00,
	0x3802, 0x00,
	0x3803, 0x0c,
	0x3804, 0x14,
	0x3805, 0x5f,
	0x3806, 0x0f,
	0x3807, 0x43,
	0x3808, 0x0a,
	0x3809, 0x20,
	0x380a, 0x07,
	0x380b, 0x98,
	0x380c, 0x01,
	0x380d, 0xf4,
	0x380e, 0x0d,
	0x380f, 0x05,
	0x3810, 0x00,
	0x3811, 0x08,
	0x3812, 0x00,
	0x3813, 0x02,
	0x3814, 0x11,
	0x3815, 0x11,
	0x3820, 0x02,
	0x3821, 0x16,
	0x3823, 0x04,
	0x3828, 0x00,
	0x382a, 0x80,
	0x3837, 0x0e,
	0x383a, 0x81,
	0x383b, 0x81,
	0x383f, 0x38,
	0x3840, 0x00,
	0x384c, 0x01,
	0x384d, 0xf4,
	0x3860, 0x00,
	0x3894, 0x00,
	0x3d84, 0x04,
	0x3d8c, 0x80,
	0x3d8d, 0xbc,
	0x3d90, 0x80,
	0x3daa, 0x00,
	0x3dac, 0x01,
	0x3dad, 0x68,
	0x3dae, 0x01,
	0x3daf, 0x6b,
	0x4000, 0xe3,
	0x4001, 0x60,
	0x4008, 0x00,
	0x4009, 0x23,
	0x400e, 0x08,
	0x400f, 0x80,
	0x4010, 0x70,
	0x4040, 0x00,
	0x4041, 0x07,
	0x404c, 0x20,
	0x404e, 0x10,
	0x40ba, 0x01,
	0x4308, 0x00,
	0x4500, 0x07,
	0x4501, 0x00,
	0x4503, 0x0f,
	0x4504, 0x80,
	0x4506, 0x01,
	0x450c, 0x00,
	0x4542, 0x00,
	0x4602, 0x00,
	0x460b, 0x07,
	0x4680, 0x11,
	0x4686, 0x00,
	0x4687, 0x00,
	0x4700, 0x07,
	0x4800, 0x64,
	0x4802, 0x00,
	0x4806, 0x40,
	0x4808, 0x05,
	0x480b, 0x10,
	0x480c, 0x80,
	0x480f, 0x32,
	0x4813, 0xe4,
	0x481b, 0x3c,
	0x4826, 0x32,
	0x4837, 0x0e, // 0x12, 1092Mbps
	0x483a, 0xdc,
	0x4850, 0x42,
	0x4860, 0x00,
	0x4861, 0xec,
	0x4883, 0x00,
	0x4885, 0x5f,
	0x4886, 0x02,
	0x4888, 0x10,
	0x4d00, 0x04,
	0x4d01, 0xc6,
	0x4d02, 0xbc,
	0x4d03, 0x3c,
	0x4d04, 0xd2,
	0x4d05, 0x00,
};

static kal_uint16 ov20b40_2592x1944_addr_data[] = {
	0x3419, 0x1a,
	0x341a, 0x0b,
	0x3760, 0x02,
	0x3761, 0x24,
	0x3762, 0x02,
	0x3763, 0x02,
	0x3764, 0x02,
 	0x3802, 0x00,
 	0x3803, 0x0c,
 	0x3806, 0x0f,
 	0x3807, 0x43,
 	0x3808, 0x0a,
 	0x3809, 0x20,
 	0x380a, 0x07,
 	0x380b, 0x98,
 	0x380e, 0x0d,
 	0x380f, 0x05,
 	0x3811, 0x08,
 	0x3813, 0x02,
 	0x3814, 0x11,
 	0x3815, 0x11,
 	0x3820, 0x02,
 	0x3821, 0x16,
 	0x4009, 0x23,
 	0x4041, 0x07,
 	0x4500, 0x07,
};

static kal_uint16 ov20b40_2592x1464_addr_data[] = {
	0x3419, 0x1a,
	0x341a, 0x0b,
	0x3760, 0x02,
	0x3761, 0x24,
	0x3762, 0x02,
	0x3763, 0x02,
	0x3764, 0x02,
 	0x3802, 0x00,
 	0x3803, 0x0c,
 	0x3806, 0x0f,
 	0x3807, 0x43,
 	0x3808, 0x0a,
 	0x3809, 0x20,
 	0x380a, 0x05,
 	0x380b, 0xb8,
 	0x380e, 0x0d,
 	0x380f, 0x05,
 	0x3811, 0x08,
 	0x3813, 0xf2,
 	0x3814, 0x11,
 	0x3815, 0x11,
 	0x3820, 0x02,
 	0x3821, 0x16,
 	0x4009, 0x23,
 	0x4041, 0x07,
 	0x4500, 0x07,
};

static kal_uint16 ov20b40_1280x720_120fps_addr_data[] = {
	0x3419, 0x06,
  	0x341a, 0x83,
  	0x3760, 0x04,
  	0x3761, 0x28,
  	0x3762, 0x04,
  	0x3763, 0x04,
  	0x3764, 0x04,
   	0x3802, 0x02,
   	0x3803, 0x00,
   	0x3806, 0x0d,
   	0x3807, 0x4f,
   	0x3808, 0x05,
   	0x3809, 0x00,
   	0x380a, 0x02,
   	0x380b, 0xd0,
   	0x380e, 0x03,
   	0x380f, 0x41,
   	0x3811, 0x0c,
   	0x3813, 0x02,
   	0x3814, 0x31,
   	0x3815, 0x31,
   	0x3820, 0x03,
   	0x3821, 0x56,
   	0x4009, 0x13,
   	0x4041, 0x03,
   	0x4500, 0x0d,
};

static kal_uint32 ov20b40aac_ana_gain_table[] = {
	1024,
	1088,
	1152,
	1216,
	1280,
	1344,
	1408,
	1472,
	1536,
	1600,
	1664,
	1728,
	1792,
	1856,
	1920,
	1984,
	2048,
	2176,
	2304,
	2432,
	2560,
	2688,
	2816,
	2944,
	3072,
	3200,
	3328,
	3456,
	3584,
	3712,
	3840,
	3968,
	4096,
	4352,
	4608,
	4864,
	5120,
	5376,
	5632,
	5888,
	6144,
	6400,
	6656,
	6912,
	7168,
	7424,
	7680,
	7936,
	8192,
	8704,
	9216,
	9728,
	10240,
	10752,
	11264,
	11776,
	12288,
	12800,
	13312,
	13824,
	14336,
	14848,
	15360,
	15872,
	16352,
};

static void sensor_init(void)
{
	pr_debug("[%s] init_start\n", __func__);
	table_write_cmos_sensor(ov20b40_init_addr_data,
		sizeof(ov20b40_init_addr_data)/sizeof(kal_uint16));
	pr_debug("[%s] init_End\n", __func__);
}	/*	  sensor_init  */

static void preview_setting(void)
{
	pr_debug("%s preview_Start\n", __func__);
	table_write_cmos_sensor(ov20b40_2592x1944_addr_data,
	  sizeof(ov20b40_2592x1944_addr_data)/sizeof(kal_uint16));

	pr_debug("%s preview_End\n", __func__);
}

static void capture_setting(kal_uint16 currefps)
{
	pr_debug("[%s] capture_Start, currefps:%d\n", __func__, currefps);
	table_write_cmos_sensor(ov20b40_2592x1944_addr_data,
		sizeof(ov20b40_2592x1944_addr_data)/sizeof(kal_uint16));
	pr_debug("[%s] capture_End\n", __func__);
}

static void normal_video_setting(kal_uint16 currefps)
{
	pr_debug("[%s] normal_video_Start, currefps:%d\n", __func__, currefps);
	table_write_cmos_sensor(ov20b40_2592x1464_addr_data,
		sizeof(ov20b40_2592x1464_addr_data)/sizeof(kal_uint16));
	pr_debug("[%s] normal_video_End\n", __func__);
}

static void hs_video_setting(void)
{
	pr_debug("[%s] hs_video_Start, 1280x720@120fps\n", __func__);
	table_write_cmos_sensor(ov20b40_1280x720_120fps_addr_data,
		sizeof(ov20b40_1280x720_120fps_addr_data)/sizeof(kal_uint16));
	pr_debug("[%s] hs_video_End\n", __func__);
}

static void slim_video_setting(void)
{
	pr_debug("[%s] slim_video_Start, 2592x1464@30.33fps\n", __func__);
	table_write_cmos_sensor(ov20b40_2592x1464_addr_data,
		sizeof(ov20b40_2592x1464_addr_data)/sizeof(kal_uint16));
	pr_debug("[%s] slim_video_End\n", __func__);
}

static void custom1_setting(void)
{
	pr_debug("[%s] custom1_Start, 2592x1944@30fps\n", __func__);
	table_write_cmos_sensor(ov20b40_2592x1944_addr_data,
		sizeof(ov20b40_2592x1944_addr_data)/sizeof(kal_uint16));
	pr_debug("[%s] custom1_End\n", __func__);
}

static void custom2_setting(void)
{
	pr_debug("[%s] custom2_Start, 2592x1944@30fps\n", __func__);
	table_write_cmos_sensor(ov20b40_2592x1944_addr_data,
		sizeof(ov20b40_2592x1944_addr_data)/sizeof(kal_uint16));
	pr_debug("[%s] custom2_End\n", __func__);
}

static void custom3_setting(void)
{
	pr_debug("[%s] custom3_Start, 2592x1464@30fps\n", __func__);
	table_write_cmos_sensor(ov20b40_2592x1464_addr_data,
		sizeof(ov20b40_2592x1464_addr_data)/sizeof(kal_uint16));
	pr_debug("[%s] custom3_End\n", __func__);
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	pr_debug("enable: %d\n", enable);

	if (enable)
	{
		write_cmos_sensor_8bit(0x3019, 0xf0);
	        write_cmos_sensor_8bit(0x4308, 0x01);
	}
	else
	{
		write_cmos_sensor_8bit(0x3019, 0xd2);
                write_cmos_sensor_8bit(0x4308, 0x00);
	}

	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}
#define OV20B40C_AAC_EEPROM_I2C_ADDR 0xA2
static kal_uint16 read_ov20b40_aac_eeprom_module(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pusendcmd[2] = { (char)(addr >> 8), (char)(addr & 0xFF) };

	iReadRegI2C(pusendcmd, 2, (u8 *) &get_byte, 1, OV20B40C_AAC_EEPROM_I2C_ADDR);

	return get_byte;
}

static kal_uint32 return_vendor_id(void)
{
	kal_uint32 vendor_id = 0;

	vendor_id = read_ov20b40_aac_eeprom_module(0x0001);
	return vendor_id;
}

/*************************************************************************
 * FUNCTION
 *	get_imgsensor_id
 *
 * DESCRIPTION
 *	This function get the sensor ID
 *
 * PARAMETERS
 *	*sensorID : return the sensor ID
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint32 vendor_id = return_vendor_id();


	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = return_sensor_id();
			pr_err("[ov20b40_camera_sensor] vendor_id = 0x%2x", vendor_id);
			if ((*sensor_id == imgsensor_info.sensor_id) && (vendor_id == 0x10)) {
				pr_err("[ov20b40_camera_sensor]get_imgsensor_id:i2c write id: 0x%x, sensor id: 0x%x, vendor_id: 0x%2x\n",
					imgsensor.i2c_write_id, *sensor_id, vendor_id);
				get_custom_info(&addr_info, &custom_info);
				return ERROR_NONE;
			}
			pr_err("[ov20b40_camera_sensor]get_imgsensor_id:Read sensor id fail, write id: 0x%x, id: 0x%x, vendor_id: 0x%2x\n",
				imgsensor.i2c_write_id, *sensor_id, vendor_id);
			retry--;
		} while (retry > 0);
		i++;
		retry = 2;
	}
	if ((*sensor_id != imgsensor_info.sensor_id) || (vendor_id != 0x10)) {
		/* if Sensor ID is not correct,
		 * Must set *sensor_id to 0xFFFFFFFF
		 */
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}


/*************************************************************************
 * FUNCTION
 *	open
 *
 * DESCRIPTION
 *	This function initialize the registers of CMOS sensor
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 open(void)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint16 sensor_id = 0;

	pr_debug("%s +\n", __func__);

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = return_sensor_id();
			if (sensor_id == imgsensor_info.sensor_id) {
				pr_debug("[ov20b40_camera_sensor]open:i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, sensor_id);
				break;
			}
			pr_debug("[ov20b40_camera_sensor]open:Read sensor id fail, write id: 0x%x, id: 0x%x\n",
				imgsensor.i2c_write_id, sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}
	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail sequence write in  */
	sensor_init();

	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en = KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.shutter = 0x480;
	imgsensor.gain = 0x40;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_mode = 0;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);
	pr_debug("%s -\n", __func__);

	return ERROR_NONE;
}

/*************************************************************************
 * FUNCTION
 *	close
 *
 * DESCRIPTION
 *
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 close(void)
{
	pr_debug("E\n");
	/* No Need to implement this function */
	streaming_control(KAL_FALSE);
	return ERROR_NONE;
}


/*************************************************************************
 * FUNCTION
 * preview
 *
 * DESCRIPTION
 *	This function start the sensor preview.
 *
 * PARAMETERS
 *	*image_window : address pointer of pixel numbers in one period of HSYNC
 *  *sensor_config_data : address pointer of line numbers in one period of VSYNC
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("%s E\n", __func__);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	preview_setting();
	set_mirror_flip(imgsensor.mirror);
	return ERROR_NONE;
}

/*************************************************************************
 * FUNCTION
 *	capture
 *
 * DESCRIPTION
 *	This function setup the CMOS sensor in capture MY_OUTPUT mode
 *
 * PARAMETERS
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;

	if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
		pr_debug(
			"Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
			imgsensor.current_fps,
			imgsensor_info.cap.max_framerate / 10);
	imgsensor.pclk = imgsensor_info.cap.pclk;
	imgsensor.line_length = imgsensor_info.cap.linelength;
	imgsensor.frame_length = imgsensor_info.cap.framelength;
	imgsensor.min_frame_length = imgsensor_info.cap.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;

	spin_unlock(&imgsensor_drv_lock);
	capture_setting(imgsensor.current_fps);
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting(imgsensor.current_fps);
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	/*imgsensor.video_mode = KAL_TRUE;*/
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/*	hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("%s. 720P@30FPS\n", __func__);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}

static kal_uint32 custom1(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM1;
	imgsensor.pclk = imgsensor_info.custom1.pclk;
	imgsensor.line_length = imgsensor_info.custom1.linelength;
	imgsensor.frame_length = imgsensor_info.custom1.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom1.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	custom1_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}

static kal_uint32 custom2(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM2;
	imgsensor.pclk = imgsensor_info.custom2.pclk;
	imgsensor.line_length = imgsensor_info.custom2.linelength;
	imgsensor.frame_length = imgsensor_info.custom2.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom2.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	custom2_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}

static kal_uint32 custom3(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM3;
	imgsensor.pclk = imgsensor_info.custom3.pclk;
	imgsensor.line_length = imgsensor_info.custom3.linelength;
	imgsensor.frame_length = imgsensor_info.custom3.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom3.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	custom3_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}

static kal_uint32
get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	pr_debug("E\n");
	sensor_resolution->SensorFullWidth =
		imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight =
		imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth =
		imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight =
		imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth =
		imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight =
		imgsensor_info.normal_video.grabwindow_height;

	sensor_resolution->SensorHighSpeedVideoWidth =
		imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight =
		imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth =
		imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight =
		imgsensor_info.slim_video.grabwindow_height;

	sensor_resolution->SensorCustom1Width =
		imgsensor_info.custom1.grabwindow_width;
	sensor_resolution->SensorCustom1Height =
		imgsensor_info.custom1.grabwindow_height;
	
	sensor_resolution->SensorCustom2Width =
		imgsensor_info.custom2.grabwindow_width;
	sensor_resolution->SensorCustom2Height =
		imgsensor_info.custom2.grabwindow_height;

	sensor_resolution->SensorCustom3Width =
		imgsensor_info.custom3.grabwindow_width;
	sensor_resolution->SensorCustom3Height =
		imgsensor_info.custom3.grabwindow_height;

	return ERROR_NONE;
}

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			   MSDK_SENSOR_INFO_STRUCT *sensor_info,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("scenario_id = %d\n", scenario_id);

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat =
		imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame =
		imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame =
		imgsensor_info.slim_video_delay_frame;
	sensor_info->Custom1DelayFrame = imgsensor_info.custom1_delay_frame;
	sensor_info->Custom2DelayFrame = imgsensor_info.custom2_delay_frame;
	sensor_info->Custom3DelayFrame = imgsensor_info.custom3_delay_frame;


	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;
	sensor_info->AESensorGainDelayFrame =
		imgsensor_info.ae_sensor_gain_delay_frame;
	sensor_info->AEISPGainDelayFrame =
		imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->TEMPERATURE_SUPPORT = imgsensor_info.temperature_support;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0; /* 0 is default 1x */
	sensor_info->SensorHightSampling = 0; /* 0 is default 1x */
	sensor_info->SensorPacketECCOrder = 1;

	sensor_info->FrameTimeDelayFrame =
		imgsensor_info.frame_time_delay_frame;

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.pre.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.cap.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:

		sensor_info->SensorGrabStartX =
			imgsensor_info.normal_video.startx;
		sensor_info->SensorGrabStartY =
			imgsensor_info.normal_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		sensor_info->SensorGrabStartX =
			imgsensor_info.slim_video.startx;
		sensor_info->SensorGrabStartY =
			imgsensor_info.slim_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		sensor_info->SensorGrabStartX = imgsensor_info.custom1.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.custom1.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		sensor_info->SensorGrabStartX = imgsensor_info.custom2.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.custom2.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom2.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		sensor_info->SensorGrabStartX = imgsensor_info.custom3.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.custom3.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom3.mipi_data_lp2hs_settle_dc;

		break;
	default:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
		break;
	}

	return ERROR_NONE;
}	/*	get_info  */


static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("scenario_id = %d\n", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		preview(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		capture(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		normal_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		hs_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		slim_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		custom1(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		custom2(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		custom3(image_window, sensor_config_data);
		break;
	default:
		pr_debug("Error ScenarioId setting");
		preview(image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}

	return ERROR_NONE;
}



static kal_uint32 set_video_mode(UINT16 framerate)
{
	pr_debug("framerate = %d\n ", framerate);
	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	spin_lock(&imgsensor_drv_lock);
	if (enable) /*enable auto flicker*/ {
		imgsensor.autoflicker_en = KAL_TRUE;
		pr_debug("enable! fps = %d", framerate);
	} else {
		 /*Cancel Auto flick*/
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(
		enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
	kal_uint32 frame_length;

//	pr_debug("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		frame_length = imgsensor_info.pre.pclk / framerate * 10
				/ imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.pre.framelength)
		? (frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length =
			imgsensor_info.pre.framelength
			+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if (framerate == 0)
			return ERROR_NONE;
		frame_length = imgsensor_info.normal_video.pclk /
				framerate * 10 /
				imgsensor_info.normal_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.normal_video.framelength)
		? (frame_length - imgsensor_info.normal_video.framelength)
		: 0;
		imgsensor.frame_length =
			imgsensor_info.normal_video.framelength
			+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
	if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
		pr_debug(
			"Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n"
			, framerate, imgsensor_info.cap.max_framerate/10);
		frame_length = imgsensor_info.cap.pclk / framerate * 10
				/ imgsensor_info.cap.linelength;
		spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line =
			(frame_length > imgsensor_info.cap.framelength)
			  ? (frame_length - imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length =
				imgsensor_info.cap.framelength
				+ imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);

		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		frame_length = imgsensor_info.hs_video.pclk / framerate * 10
				/ imgsensor_info.hs_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.hs_video.framelength)
			  ? (frame_length - imgsensor_info.hs_video.framelength)
			  : 0;
		imgsensor.frame_length =
			imgsensor_info.hs_video.framelength
				+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		frame_length = imgsensor_info.slim_video.pclk / framerate * 10
			/ imgsensor_info.slim_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.slim_video.framelength)
			? (frame_length - imgsensor_info.slim_video.framelength)
			: 0;
		imgsensor.frame_length =
			imgsensor_info.slim_video.framelength
			+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		frame_length = imgsensor_info.custom1.pclk / framerate * 10
			/ imgsensor_info.custom1.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.custom1.framelength)
			? (frame_length - imgsensor_info.custom1.framelength)
			: 0;
		imgsensor.frame_length =
			imgsensor_info.custom1.framelength
			+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		frame_length = imgsensor_info.custom2.pclk / framerate * 10
			/ imgsensor_info.custom2.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.custom2.framelength)
			? (frame_length - imgsensor_info.custom2.framelength)
			: 0;
		imgsensor.frame_length =
			imgsensor_info.custom2.framelength
			+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		frame_length = imgsensor_info.custom3.pclk / framerate * 10
			/ imgsensor_info.custom3.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.custom3.framelength)
			? (frame_length - imgsensor_info.custom3.framelength)
			: 0;
		imgsensor.frame_length =
			imgsensor_info.custom3.framelength
			+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	default:  /*coding with  preview scenario by default*/
		frame_length = imgsensor_info.pre.pclk / framerate * 10
			/ imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.pre.framelength)
			? (frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length =
			imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		pr_debug("error scenario_id = %d, we use preview scenario\n",
			scenario_id);
		break;
	}
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(
		enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
	pr_debug("scenario_id = %d\n", scenario_id);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		*framerate = imgsensor_info.pre.max_framerate;
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		*framerate = imgsensor_info.normal_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		*framerate = imgsensor_info.cap.max_framerate;
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		*framerate = imgsensor_info.hs_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		*framerate = imgsensor_info.slim_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		*framerate = imgsensor_info.custom1.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		*framerate = imgsensor_info.custom2.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		*framerate = imgsensor_info.custom3.max_framerate;
		break;
	default:
		break;
	}

	return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
				 UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;
	unsigned long long *feature_data = (unsigned long long *) feature_para;

	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;

	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data
		= (MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	/* pr_debug("feature_id = %d\n", feature_id); */
	switch (feature_id) {
	case SENSOR_FEATURE_GET_AWB_REQ_BY_SCENARIO:
		*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 0;
		break;
	case SENSOR_FEATURE_GET_GAIN_RANGE_BY_SCENARIO:
		*(feature_data + 1) = imgsensor_info.min_gain;
		*(feature_data + 2) = imgsensor_info.max_gain;
		break;
	case SENSOR_FEATURE_GET_BASE_GAIN_ISO_AND_STEP:
		*(feature_data + 0) = imgsensor_info.min_gain_iso;
		*(feature_data + 1) = imgsensor_info.gain_step;
		*(feature_data + 2) = imgsensor_info.gain_type;
		break;
	case SENSOR_FEATURE_GET_MIN_SHUTTER_BY_SCENARIO:
		*(feature_data + 1) = imgsensor_info.min_shutter;
		*(feature_data + 2) = imgsensor_info.exp_step;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ_BY_SCENARIO:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.cap.pclk;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.normal_video.pclk;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.hs_video.pclk;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.slim_video.pclk;
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom1.pclk;
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom2.pclk;
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom3.pclk;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.pre.pclk;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_PERIOD_BY_SCENARIO:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.cap.framelength << 16)
				+ imgsensor_info.cap.linelength;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.normal_video.framelength << 16)
				+ imgsensor_info.normal_video.linelength;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.hs_video.framelength << 16)
				+ imgsensor_info.hs_video.linelength;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.slim_video.framelength << 16)
				+ imgsensor_info.slim_video.linelength;
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom1.framelength << 16)
				+ imgsensor_info.custom1.linelength;
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom2.framelength << 16)
				+ imgsensor_info.custom2.linelength;
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom3.framelength << 16)
				+ imgsensor_info.custom3.linelength;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.pre.framelength << 16)
				+ imgsensor_info.pre.linelength;
			break;
		}
		break;

	case SENSOR_FEATURE_GET_PERIOD:
		*feature_return_para_16++ = imgsensor.line_length;
		*feature_return_para_16 = imgsensor.frame_length;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
		*feature_return_para_32 = imgsensor.pclk;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_ESHUTTER:
		 set_shutter(*feature_data);
		break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
		 /* night_mode((BOOL) *feature_data); */
		break;
	case SENSOR_FEATURE_SET_GAIN:
		set_gain((UINT16) *feature_data);
		break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;
	case SENSOR_FEATURE_SET_REGISTER:
		write_cmos_sensor_8bit(sensor_reg_data->RegAddr,
				    sensor_reg_data->RegData);
		break;
	case SENSOR_FEATURE_GET_REGISTER:
		sensor_reg_data->RegData =
			read_cmos_sensor(sensor_reg_data->RegAddr);
		break;
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
		/*get the lens driver ID from EEPROM
		 * or just return LENS_DRIVER_ID_DO_NOT_CARE
		 * if EEPROM does not exist in camera module.
		 */
		*feature_return_para_32 = LENS_DRIVER_ID_DO_NOT_CARE;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_VIDEO_MODE:
		set_video_mode(*feature_data);
		break;
	case SENSOR_FEATURE_CHECK_SENSOR_ID:
		get_imgsensor_id(feature_return_para_32);
		break;
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
		set_auto_flicker_mode((BOOL)*feature_data_16,
				      *(feature_data_16+1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		 set_max_framerate_by_scenario(
				(enum MSDK_SCENARIO_ID_ENUM)*feature_data,
				*(feature_data+1));
		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		 get_default_framerate_by_scenario(
				(enum MSDK_SCENARIO_ID_ENUM)*(feature_data),
				(MUINT32 *)(uintptr_t)(*(feature_data+1)));
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode((BOOL)*feature_data);
		break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
		/* for factory mode auto testing */
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		pr_debug("current fps :%d\n", (UINT32)*feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.current_fps = *feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_SET_HDR:
		pr_debug("ihdr enable :%d\n", (BOOL)*feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.ihdr_mode = *feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_GET_CROP_INFO:
	#if 0
		pr_debug("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n",
			(UINT32)*feature_data);
	#endif
		wininfo =
	(struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));

		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[1],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[2],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[3],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[4],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[5],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[6],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[7],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;

		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[0],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
		break;
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		pr_debug("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",
			(UINT16)*feature_data,
			(UINT16)*(feature_data+1),
			(UINT16)*(feature_data+2));
		break;
	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
		set_shutter_frame_length((UINT16) (*feature_data),
					(UINT16) (*(feature_data + 1)));
		break;
	case SENSOR_FEATURE_GET_FRAME_CTRL_INFO_BY_SCENARIO:
		/*
		 * 1, if driver support new sw frame sync
		 * set_shutter_frame_length() support third para auto_extend_en
		 */
		*(feature_data + 1) = 1;
		/* margin info by scenario */
		*(feature_data + 2) = imgsensor_info.margin;
		break;
	case SENSOR_FEATURE_SET_HDR_SHUTTER:
		pr_debug("SENSOR_FEATURE_SET_HDR_SHUTTER LE=%d, SE=%d\n",
			(UINT16)*feature_data, (UINT16)*(feature_data+1));
		#if 0
		ihdr_write_shutter((UINT16)*feature_data,
				   (UINT16)*(feature_data+1));
		#endif
		break;
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		pr_debug("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
		streaming_control(KAL_FALSE);
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		pr_debug("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n",
			*feature_data);
		if (*feature_data != 0)
			set_shutter(*feature_data);
		streaming_control(KAL_TRUE);
		break;
	case SENSOR_FEATURE_GET_BINNING_TYPE:
		*feature_return_para_32 = 1; /* BINNING_AVERAGED */
		pr_debug("SENSOR_FEATURE_GET_BINNING_TYPE AE_binning_type:%d,\n",
			*feature_return_para_32);
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
	{
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.cap.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.normal_video.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.hs_video.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.slim_video.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom1.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom2.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom3.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.pre.mipi_pixel_rate;
			break;
		}
	}
break;
	case SENSOR_FEATURE_SET_AWB_GAIN:
	case SENSOR_FEATURE_SET_LSC_TBL:
		break;
	case SENSOR_FEATURE_GET_CUSTOM_INFO:
		if( *feature_para_len < sizeof(IMGSENSOR_GET_CUSTOM_INFO_STRUCT) ){
			LOG_INF("FeatureParaLen is too small %d\n", feature_para_len);
			kfree(feature_para);
			return -EINVAL;
		}
		memcpy((UINT8 *)feature_para, (UINT8 *)&custom_info, sizeof(IMGSENSOR_GET_CUSTOM_INFO_STRUCT));
	break;
	case SENSOR_FEATURE_GET_ANA_GAIN_TABLE:
		if ((void *)(uintptr_t) (*(feature_data + 1)) == NULL) {
			*(feature_data + 0) =
				sizeof(ov20b40aac_ana_gain_table);
		} else {
			memcpy((void *)(uintptr_t) (*(feature_data + 1)),
			(void *)ov20b40aac_ana_gain_table,
			sizeof(ov20b40aac_ana_gain_table));
		}
		break;
	case SENSOR_FEATURE_GET_OFFSET_TO_START_OF_EXPOSURE:
		switch (*feature_data) {
			case 100:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 1200000;
				break;
			case 101:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 3050000;
				break;
			default:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 1200000;
				break;
		}
		break;
	default:
		break;
	}

	return ERROR_NONE;
}

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};
UINT32 OV20B40_AAC_FRONT_I_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc = &sensor_func;
	return ERROR_NONE;
}


