/*
 * Copyright (C) 2018 MediaTek Inc.
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

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>

#include "imgsensor_custom.h"

#include "sc1620_ofilm_front_ii_mipi_raw.h"

#define PFX "sc1620_camera_sensor"
#define LOG_INF(format, args...)    \
	pr_debug(PFX "[%s] " format, __func__, ##args)


/*static kal_uint16 get_eeprom_0x004_value=0;
*/

static DEFINE_SPINLOCK(imgsensor_drv_lock);

static IMGSENSOR_GET_CUSTOM_INFO_STRUCT custom_info = {
    .sensor_type = SENSOR_FRONT_MAIN,
	.module_head = "ofil",
	.fuse_id = "",
	.fuse_id_len = 16,
	.sn_info = "",
	.sn_info_len = 14,
	.vendor_id = 0,
	.lens_id = 0,
	.sensor_id = 0
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
	.sensor_id = SC1620_OFILM_FRONT_II_SENSOR_ID,
	.checksum_value = 0x55e2a82f,
	.pre = {
		.pclk = 108000000,
		.linelength = 1340,
		.framelength = 2686,
		.startx = 0,
		.starty = 0,
                .grabwindow_width = 2304,
                .grabwindow_height = 1728,
		.mipi_data_lp2hs_settle_dc = 14,
		.mipi_pixel_rate = 556800000,
		.max_framerate = 300,
	},
	.cap = {
		.pclk = 108000000,
		.linelength = 1340,
		.framelength = 2686,
		.startx = 0,
		.starty = 0,
                .grabwindow_width = 2304,
                .grabwindow_height = 1728,
		.mipi_data_lp2hs_settle_dc = 14,
		.mipi_pixel_rate = 480000000,
		.max_framerate = 300,
	},
	.normal_video = {
		.pclk = 108000000,
		.linelength = 1340,
		.framelength = 2686,
		.startx = 0,
		.starty = 0,
                .grabwindow_width = 2304,
                .grabwindow_height = 1296,
		.mipi_data_lp2hs_settle_dc = 14,
		.mipi_pixel_rate = 480000000,
		.max_framerate = 300,
	},
	.hs_video = {
                .pclk = 108000000,
		.linelength = 670,
		.framelength = 2686,
		.startx = 0,
		.starty = 0,
                .grabwindow_width = 2304,
                .grabwindow_height = 1728,
		.mipi_data_lp2hs_settle_dc = 14,
		.mipi_pixel_rate = 480000000,
		.max_framerate = 600,
         },
    .slim_video = {
                .pclk = 108000000,
		.linelength = 1340,
		.framelength = 2686,
		.startx = 0,
		.starty = 0,
                .grabwindow_width = 2304,
                .grabwindow_height = 1728,
		.mipi_data_lp2hs_settle_dc = 14,
		.mipi_pixel_rate = 480000000,
		.max_framerate = 300,
     },
    .custom1 = {
                .pclk = 108000000,
		.linelength = 670,
		.framelength = 2686,
		.startx = 0,
		.starty = 0,
                .grabwindow_width = 2304,
                .grabwindow_height = 1728,
		.mipi_data_lp2hs_settle_dc = 14,
		.mipi_pixel_rate = 480000000,
		.max_framerate = 600,
         },
    .custom2 = {
                .pclk = 108000000,
		.linelength = 670,
		.framelength = 2686,
		.startx = 0,
		.starty = 0,
                .grabwindow_width = 2304,
                .grabwindow_height = 1728,
		.mipi_data_lp2hs_settle_dc = 14,
		.mipi_pixel_rate = 480000000,
		.max_framerate = 600,
         },
    .custom3 = {
                .pclk = 108000000,
		.linelength = 670,
		.framelength = 2686,
		.startx = 0,
		.starty = 0,
                .grabwindow_width = 2304,
                .grabwindow_height = 1296,
		.mipi_data_lp2hs_settle_dc = 14,
		.mipi_pixel_rate = 480000000,
		.max_framerate = 600,
         },

     .margin = 8,            //sensor framelength & shutter margin
     .min_shutter = 4,
     .max_frame_length = 0x1FFF0,
     .min_gain = 64,
     .max_gain = 3840,
     .min_gain_iso = 50,
     .gain_step = 2,
     .gain_type = 2,
     .ae_shut_delay_frame = 0,
     .ae_sensor_gain_delay_frame = 0,
     .ae_ispGain_delay_frame = 2,
     .ihdr_support = 0,     // //1, support; 0,not support
     .ihdr_le_firstline = 0, // //1,le first ; 0, se first
     .sensor_mode_num = 8,      //support sensor mode num

     .cap_delay_frame = 3,
     .pre_delay_frame = 3,
     .video_delay_frame = 3,
     .hs_video_delay_frame = 3,
     .slim_video_delay_frame = 3,
     .custom1_delay_frame = 3,
     .custom2_delay_frame = 3,
     .custom3_delay_frame = 3,
     .frame_time_delay_frame = 2,
     .isp_driving_current = ISP_DRIVING_6MA,
     .sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
     .mipi_sensor_type = MIPI_OPHY_NCSI2,
     .mipi_settle_delay_mode =MIPI_SETTLEDELAY_AUTO,//0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
     .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_B,
     .mclk = 24,
     .mipi_lane_num = SENSOR_MIPI_4_LANE,
     .i2c_addr_table = {0x20, 0xff},
     .i2c_speed = 400,
};

static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_HV_MIRROR,
	.sensor_mode = IMGSENSOR_MODE_INIT,
	.shutter = 0x3d0,                   //current shutter   
	.gain = 0x40,                      //current gain     
	.dummy_pixel = 0,
	.dummy_line = 0,
//full size current fps : 24fps for PIP, 30fps for Normal or ZSD
	.current_fps = 300,
	.autoflicker_en = KAL_FALSE,
	.test_pattern = KAL_FALSE,
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,
	.ihdr_en = 0,
    .i2c_write_id = 0x20,
};

/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[8] = {
 { 2304, 1728,	  0,	0, 2304, 1728, 2304, 1728, 0000, 0000, 2304, 1728,    0,	0, 2304, 1728}, // Preview
 { 2304, 1728,	  0,	0, 2304, 1728, 2304, 1728, 0000, 0000, 2304, 1728,    0,	0, 2304, 1728}, // capture  
 { 2304, 1728,	  0,	216, 2304, 1296, 2304, 1296, 0000, 0000, 2304, 1296,    0,	0, 2304, 1296}, // video
 { 2304, 1728,	  0,	0, 2304, 1728, 2304, 1728, 0000, 0000, 2304, 1728,    0,	0, 2304, 1728}, //hight speed video
 { 2304, 1728,	  0,	0, 2304, 1728, 2304, 1728, 0000, 0000, 2304, 1728,    0,	0, 2304, 1728}, // slim video
 { 2304, 1728,	  0,	0, 2304, 1728, 2304, 1728, 0000, 0000, 2304, 1728,    0,	0, 2304, 1728}, //custom1
 { 2304, 1728,	  0,	0, 2304, 1728, 2304, 1728, 0000, 0000, 2304, 1728,    0,	0, 2304, 1728}, //custom2
 { 2304, 1728,	  0,	216, 2304, 1296, 2304, 1296, 0000, 0000, 2304, 1296,    0,	0, 2304, 1296}, //custom3
};


#define I2C_BUFFER_LEN 765
/*
static kal_uint16 sc1620cs_table_write_cmos_sensor(
					kal_uint16 *para, kal_uint32 len)
{
	char puSendCmd[I2C_BUFFER_LEN];
	kal_uint32 tosend, IDX;
	kal_uint16 addr = 0, addr_last = 0, data;

	tosend = 0;
	IDX = 0;
	while (len > IDX) {
		addr = para[IDX];

		{
			puSendCmd[tosend++] = (char)(addr >> 8);
			puSendCmd[tosend++] = (char)(addr & 0xFF);
			data = para[IDX + 1];
			//puSendCmd[tosend++] = (char)(data >> 8);
			puSendCmd[tosend++] = (char)(data & 0xFF);
			IDX += 2;
			addr_last = addr;
		}

		if ((I2C_BUFFER_LEN - tosend) < 4 ||
			len == IDX ||
			addr != addr_last) {
			iBurstWriteReg_multi(puSendCmd, tosend,
				imgsensor.i2c_write_id,
				3, imgsensor_info.i2c_speed);

			tosend = 0;
		}
	}
	return 0;
}
*/

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

	iReadRegI2C(pu_send_cmd, 2, (u8 *)&get_byte, 1, imgsensor.i2c_write_id);

	return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[4] = {(char)(addr >> 8),
		(char)(addr & 0xFF), (char)(para >> 8), (char)(para & 0xFF)};

	iWriteRegI2C(pu_send_cmd, 4, imgsensor.i2c_write_id);
}

static void write_cmos_sensor8(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[4] = {(char)(addr >> 8),
		(char)(addr & 0xFF), (char)(para & 0xFF)};

	iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}

static void set_dummy(void)
{
	LOG_INF("dummyline = %d, dummypixels = %d\n",
		imgsensor.dummy_line, imgsensor.dummy_pixel);
    write_cmos_sensor8(0x326d, (imgsensor.frame_length >> 16) & 0x01);
	write_cmos_sensor8(0x320e, (imgsensor.frame_length >> 8) & 0xff);
	write_cmos_sensor8(0x320f, imgsensor.frame_length & 0xFF);	  
	write_cmos_sensor8(0x320c, imgsensor.line_length >> 8);
	write_cmos_sensor8(0x320d, imgsensor.line_length & 0xFF);


}	/*	set_dummy  */

static kal_uint32 return_sensor_id(void)
{
	return ((read_cmos_sensor(0x3107) << 8) | read_cmos_sensor(0x3108)); //c959
}


static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
	kal_uint32 frame_length = imgsensor.frame_length;

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ?
			frame_length : imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length -
		imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length -
			imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;

	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}	/*	set_max_framerate  */

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

	shutter = (shutter < imgsensor_info.min_shutter) ?
		imgsensor_info.min_shutter : shutter;
	shutter = (shutter >
		(imgsensor_info.max_frame_length - imgsensor_info.margin)) ?
		(imgsensor_info.max_frame_length - imgsensor_info.margin) :
		shutter;
	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk * 10 /
			(imgsensor.line_length * imgsensor.frame_length);
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else{
		write_cmos_sensor8(0x326d, (imgsensor.frame_length >> 16) & 0x01);
		write_cmos_sensor8(0x320e, (imgsensor.frame_length >> 8) & 0xff);
		write_cmos_sensor8(0x320f, imgsensor.frame_length & 0xFF);
		}

	} else{
		// Extend frame length

		// ADD ODIN
		realtime_fps = imgsensor.pclk * 10 /
			(imgsensor.line_length * imgsensor.frame_length);
		if (realtime_fps > 300 && realtime_fps < 320)
			set_max_framerate(300, 0);
		// ADD END
		write_cmos_sensor8(0x326d, (imgsensor.frame_length >> 16) & 0x01);
		write_cmos_sensor8(0x320e, (imgsensor.frame_length >> 8) & 0xff);
		write_cmos_sensor8(0x320f, imgsensor.frame_length & 0xFF);
	}

	// Update Shutter
	//shutter = shutter *2;
	write_cmos_sensor8(0x3e00, (shutter >> 12) & 0x0F);
	write_cmos_sensor8(0x3e01, (shutter >> 4)&0xFF);
	write_cmos_sensor8(0x3e02, (shutter<<4) & 0xF0);

	LOG_INF("shutter =%d, framelength =%d",
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

	LOG_INF("set_shutter");
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	write_shutter(shutter);
}	/*	set_shutter */
#if 0
static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 reg_gain = 0x0000;

	reg_gain = gain / 4 - 16;

	return (kal_uint16)reg_gain;
}
#endif
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
static kal_uint16 set_gain(kal_uint16 gain)
{

	kal_uint16 reg_gain;

	if (gain < BASEGAIN || gain > 60 * BASEGAIN) {

		if (gain < BASEGAIN)
			gain = BASEGAIN;
		else if (gain > 60 * BASEGAIN)
			gain = 60 * BASEGAIN;
	}
    reg_gain = gain >> 1;
	write_cmos_sensor8(0x3e08,(reg_gain>>8));	
	write_cmos_sensor8(0x3e09,(reg_gain&0xFF));	

    spin_lock(&imgsensor_drv_lock); 
    imgsensor.gain = reg_gain; 
    spin_unlock(&imgsensor_drv_lock);
    LOG_INF("gain = %d ,reg_gain = 0x%x ,again1 = 0x%x , again2 = 0x%x\n ", gain, reg_gain,read_cmos_sensor(0x3e08),read_cmos_sensor(0x3e09));

	return gain;

}	/*	set_gain  */

#if 0
static void ihdr_write_shutter_gain(kal_uint16 le,
				kal_uint16 se, kal_uint16 gain)
{
	LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n", le, se, gain);
	if (imgsensor.ihdr_en) {
		spin_lock(&imgsensor_drv_lock);
		if (le > imgsensor.min_frame_length - imgsensor_info.margin)
			imgsensor.frame_length = le + imgsensor_info.margin;
		else
			imgsensor.frame_length = imgsensor.min_frame_length;
		if (imgsensor.frame_length > imgsensor_info.max_frame_length)
			imgsensor.frame_length =
				imgsensor_info.max_frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (le < imgsensor_info.min_shutter)
			le = imgsensor_info.min_shutter;
		if (se < imgsensor_info.min_shutter)
			se = imgsensor_info.min_shutter;
		// Extend frame length first
		write_cmos_sensor(0x0006, imgsensor.frame_length);
		write_cmos_sensor(0x3502, (le << 4) & 0xFF);
		write_cmos_sensor(0x3501, (le >> 4) & 0xFF);
		write_cmos_sensor(0x3500, (le >> 12) & 0x0F);
		write_cmos_sensor(0x3508, (se << 4) & 0xFF);
		write_cmos_sensor(0x3507, (se >> 4) & 0xFF);
		write_cmos_sensor(0x3506, (se >> 12) & 0x0F);
		set_gain(gain);
	}
}
#endif



static void set_mirror_flip(kal_uint8 image_mirror)
{
	LOG_INF("image_mirror = %d", image_mirror);

	switch (image_mirror) {
		case IMAGE_NORMAL:
			write_cmos_sensor8(0x3221,0x00);  
			break;
		case IMAGE_H_MIRROR:
			write_cmos_sensor8(0x3221,0x06);
			break;
		case IMAGE_V_MIRROR:
			write_cmos_sensor8(0x3221,0x60);		
			break;
		case IMAGE_HV_MIRROR:
			write_cmos_sensor8(0x3221,0x66);

		break;
	default:
		LOG_INF("Error image_mirror setting");
		break;
	}

}

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
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/
}    /*    night_mode    */

static void sensor_init(void)
{
    write_cmos_sensor8(0x301f,0x06); 

}

static void preview_setting(void)
{
    write_cmos_sensor8(0x0103,0x01);
    write_cmos_sensor8(0x0100,0x00);
    write_cmos_sensor8(0x36e9,0x80);
    write_cmos_sensor8(0x37f9,0x80);
    write_cmos_sensor8(0x36ea,0x1d);
    write_cmos_sensor8(0x36eb,0x0c);
    write_cmos_sensor8(0x36ec,0x4b);
    write_cmos_sensor8(0x36ed,0x14);
    write_cmos_sensor8(0x37fa,0x1b);
    write_cmos_sensor8(0x37fb,0xb2);
    write_cmos_sensor8(0x37fc,0x10);
    write_cmos_sensor8(0x37fd,0x4a);
    write_cmos_sensor8(0x36e9,0x24);
    write_cmos_sensor8(0x37f9,0x24);
    write_cmos_sensor8(0x301f,0x0d);
    write_cmos_sensor8(0x3200,0x00);
    write_cmos_sensor8(0x3201,0x18);
    write_cmos_sensor8(0x3202,0x00);
    write_cmos_sensor8(0x3203,0x14);
    write_cmos_sensor8(0x3204,0x12);
    write_cmos_sensor8(0x3205,0x27);
    write_cmos_sensor8(0x3206,0x0d);
    write_cmos_sensor8(0x3207,0xa3);
    write_cmos_sensor8(0x3208,0x09);
    write_cmos_sensor8(0x3209,0x00);
    write_cmos_sensor8(0x320a,0x06);
    write_cmos_sensor8(0x320b,0xc0);
    write_cmos_sensor8(0x320c,0x05);
    write_cmos_sensor8(0x320d,0x3c);
    write_cmos_sensor8(0x320e,0x0a);
    write_cmos_sensor8(0x320f,0x7e);
    write_cmos_sensor8(0x3210,0x00);
    write_cmos_sensor8(0x3211,0x04);
    write_cmos_sensor8(0x3212,0x00);
    write_cmos_sensor8(0x3213,0x0a);
    write_cmos_sensor8(0x3250,0x03);
    write_cmos_sensor8(0x325f,0xf1);
    write_cmos_sensor8(0x32d1,0xf0);
    write_cmos_sensor8(0x3301,0x06);
    write_cmos_sensor8(0x3304,0x58);
    write_cmos_sensor8(0x3306,0x60);
    write_cmos_sensor8(0x3308,0x10);
    write_cmos_sensor8(0x3309,0x68);
    write_cmos_sensor8(0x330b,0xc0);
    write_cmos_sensor8(0x330d,0x18);
    write_cmos_sensor8(0x330e,0x28);
    write_cmos_sensor8(0x331e,0x49);
    write_cmos_sensor8(0x331f,0x59);
    write_cmos_sensor8(0x3333,0x10);
    write_cmos_sensor8(0x3334,0x40);
    write_cmos_sensor8(0x3347,0x05);
    write_cmos_sensor8(0x3351,0x03);
    write_cmos_sensor8(0x336c,0x4e);
    write_cmos_sensor8(0x337f,0x13);
    write_cmos_sensor8(0x3399,0x06);
    write_cmos_sensor8(0x33ad,0x1c);
    write_cmos_sensor8(0x33b1,0x80);
    write_cmos_sensor8(0x33b2,0x50);
    write_cmos_sensor8(0x33b3,0x18);
    write_cmos_sensor8(0x3433,0x69);
    write_cmos_sensor8(0x34ae,0x16);
    write_cmos_sensor8(0x34ba,0x01);
    write_cmos_sensor8(0x34bb,0x13);
    write_cmos_sensor8(0x3633,0x46);
    write_cmos_sensor8(0x3637,0x56);
    write_cmos_sensor8(0x363e,0x34);
    write_cmos_sensor8(0x3670,0x48);
    write_cmos_sensor8(0x3690,0x44);
    write_cmos_sensor8(0x3691,0x45);
    write_cmos_sensor8(0x3692,0x66);
    write_cmos_sensor8(0x3698,0x83);
    write_cmos_sensor8(0x3699,0x85);
    write_cmos_sensor8(0x369a,0x87);
    write_cmos_sensor8(0x369b,0x8d);
    write_cmos_sensor8(0x369c,0x80);
    write_cmos_sensor8(0x369d,0xf8);
    write_cmos_sensor8(0x36a0,0xa8);
    write_cmos_sensor8(0x36a1,0xb2);
    write_cmos_sensor8(0x36a2,0x80);
    write_cmos_sensor8(0x36a3,0x88);
    write_cmos_sensor8(0x36a4,0x98);
    write_cmos_sensor8(0x36ae,0xb8);
    write_cmos_sensor8(0x36af,0xf8);
    write_cmos_sensor8(0x36b3,0x08);
    write_cmos_sensor8(0x370f,0x01);
    write_cmos_sensor8(0x3722,0x63);
    write_cmos_sensor8(0x3724,0xb1);
    write_cmos_sensor8(0x3771,0x43);
    write_cmos_sensor8(0x3772,0x43);
    write_cmos_sensor8(0x3773,0x63);
    write_cmos_sensor8(0x377a,0x80);
    write_cmos_sensor8(0x377b,0xb8);
    write_cmos_sensor8(0x3900,0x07);
    write_cmos_sensor8(0x3902,0xf0);
    write_cmos_sensor8(0x3903,0x20);
    write_cmos_sensor8(0x3905,0x8d);
    write_cmos_sensor8(0x3907,0x00);
    write_cmos_sensor8(0x3908,0x40);
    write_cmos_sensor8(0x391a,0x60);
    write_cmos_sensor8(0x391b,0x48);
    write_cmos_sensor8(0x391c,0x2d);
    write_cmos_sensor8(0x391d,0x00);
    write_cmos_sensor8(0x391f,0x41);
    write_cmos_sensor8(0x3921,0x00);
    write_cmos_sensor8(0x3922,0x01);
    write_cmos_sensor8(0x3926,0xe0);
    write_cmos_sensor8(0x397b,0x00);
    write_cmos_sensor8(0x3993,0x00);
    write_cmos_sensor8(0x39c1,0x4c);
    write_cmos_sensor8(0x39dd,0x02);
    write_cmos_sensor8(0x39de,0x02);
    write_cmos_sensor8(0x39e7,0x04);
    write_cmos_sensor8(0x39e8,0x80);
    write_cmos_sensor8(0x39e9,0x80);
    write_cmos_sensor8(0x3e00,0x00);
    write_cmos_sensor8(0x3e01,0xa7);
    write_cmos_sensor8(0x3e02,0x60);
    write_cmos_sensor8(0x3e03,0x03);
    write_cmos_sensor8(0x3e16,0x02);
    write_cmos_sensor8(0x3e17,0x00);
    write_cmos_sensor8(0x3e18,0x02);
    write_cmos_sensor8(0x3e19,0x00);
    write_cmos_sensor8(0x4200,0x0c);
    write_cmos_sensor8(0x4209,0x08);
    write_cmos_sensor8(0x420b,0x08);
    write_cmos_sensor8(0x420d,0x08);
    write_cmos_sensor8(0x420f,0x08);
    write_cmos_sensor8(0x4210,0x06);
    write_cmos_sensor8(0x4211,0x06);
    write_cmos_sensor8(0x4212,0x06);
    write_cmos_sensor8(0x4213,0x0c);
    write_cmos_sensor8(0x4215,0x80);
    write_cmos_sensor8(0x4217,0x98);
    write_cmos_sensor8(0x4219,0xb8);
    write_cmos_sensor8(0x421b,0xf8);
    write_cmos_sensor8(0x421c,0x06);
    write_cmos_sensor8(0x421d,0x0c);
    write_cmos_sensor8(0x421e,0x14);
    write_cmos_sensor8(0x421f,0x18);
    write_cmos_sensor8(0x4269,0x80);
    write_cmos_sensor8(0x426b,0x98);
    write_cmos_sensor8(0x426d,0xb8);
    write_cmos_sensor8(0x426f,0xf8);
    write_cmos_sensor8(0x4272,0x10);
    write_cmos_sensor8(0x4273,0x08);
    write_cmos_sensor8(0x4274,0x00);
    write_cmos_sensor8(0x4275,0x00);
    write_cmos_sensor8(0x4402,0x03);
    write_cmos_sensor8(0x4403,0x0b);
    write_cmos_sensor8(0x4404,0x21);
    write_cmos_sensor8(0x4405,0x2b);
    write_cmos_sensor8(0x440c,0x36);
    write_cmos_sensor8(0x440d,0x36);
    write_cmos_sensor8(0x440e,0x29);
    write_cmos_sensor8(0x440f,0x44);
    write_cmos_sensor8(0x4412,0x01);
    write_cmos_sensor8(0x4424,0x01);
    write_cmos_sensor8(0x442d,0x02);
    write_cmos_sensor8(0x442e,0x1c);
    write_cmos_sensor8(0x4509,0x20);
    write_cmos_sensor8(0x4800,0x24);
    write_cmos_sensor8(0x4837,0x16);
    write_cmos_sensor8(0x5008,0x01);
    write_cmos_sensor8(0x5011,0x40);
    write_cmos_sensor8(0x5501,0xff);
    write_cmos_sensor8(0x5504,0x00);
    write_cmos_sensor8(0x5505,0x0c);
    write_cmos_sensor8(0x5506,0x09);
    write_cmos_sensor8(0x5507,0x13);
    write_cmos_sensor8(0x5508,0x00);
    write_cmos_sensor8(0x5509,0x10);
    write_cmos_sensor8(0x550a,0x06);
    write_cmos_sensor8(0x550b,0xd7);
    write_cmos_sensor8(0x550c,0x11);
    write_cmos_sensor8(0x550d,0x11);
    write_cmos_sensor8(0x550e,0x01);
    write_cmos_sensor8(0x550f,0xb0);
    write_cmos_sensor8(0x5514,0x0c);
    write_cmos_sensor8(0x5784,0x08);
    write_cmos_sensor8(0x5785,0x04);
    write_cmos_sensor8(0x5786,0x02);
    write_cmos_sensor8(0x5787,0x08);
    write_cmos_sensor8(0x5788,0x08);
    write_cmos_sensor8(0x5789,0x08);
    write_cmos_sensor8(0x578a,0x08);
    write_cmos_sensor8(0x578b,0x08);
    write_cmos_sensor8(0x578c,0x08);
    write_cmos_sensor8(0x578d,0x41);
    write_cmos_sensor8(0x5790,0x08);
    write_cmos_sensor8(0x5791,0x08);
    write_cmos_sensor8(0x5792,0x08);
    write_cmos_sensor8(0x5793,0x08);
    write_cmos_sensor8(0x5794,0x08);
    write_cmos_sensor8(0x5795,0x08);
    write_cmos_sensor8(0x5799,0x44);
    write_cmos_sensor8(0x57ac,0x00);
	write_cmos_sensor8(0x57ad,0x00);
    write_cmos_sensor8(0x5ae0,0x00);
}

static void capture_setting(kal_uint16 currefps)
{
    write_cmos_sensor8(0x0103,0x01);
    write_cmos_sensor8(0x0100,0x00);
    write_cmos_sensor8(0x36e9,0x80);
    write_cmos_sensor8(0x37f9,0x80);
    write_cmos_sensor8(0x36ea,0x0a);
    write_cmos_sensor8(0x36eb,0x0c);
    write_cmos_sensor8(0x36ec,0x4b);
    write_cmos_sensor8(0x36ed,0x24);
    write_cmos_sensor8(0x37fa,0x1b);
    write_cmos_sensor8(0x37fb,0xb2);
    write_cmos_sensor8(0x37fc,0x10);
    write_cmos_sensor8(0x37fd,0x4a);
    write_cmos_sensor8(0x36e9,0x24);
    write_cmos_sensor8(0x37f9,0x24);
    write_cmos_sensor8(0x301f,0x07);
    write_cmos_sensor8(0x3200,0x00);
    write_cmos_sensor8(0x3201,0x18);
    write_cmos_sensor8(0x3202,0x00);
    write_cmos_sensor8(0x3203,0x14);
    write_cmos_sensor8(0x3204,0x12);
    write_cmos_sensor8(0x3205,0x27);
    write_cmos_sensor8(0x3206,0x0d);
    write_cmos_sensor8(0x3207,0xa3);
    write_cmos_sensor8(0x3208,0x09);
    write_cmos_sensor8(0x3209,0x00);
    write_cmos_sensor8(0x320a,0x06);
    write_cmos_sensor8(0x320b,0xc0);
    write_cmos_sensor8(0x320c,0x05);
    write_cmos_sensor8(0x320d,0x3c);
    write_cmos_sensor8(0x320e,0x0a);
    write_cmos_sensor8(0x320f,0x7e);
    write_cmos_sensor8(0x3210,0x00);
    write_cmos_sensor8(0x3211,0x04);
    write_cmos_sensor8(0x3212,0x00);
    write_cmos_sensor8(0x3213,0x0a);
    write_cmos_sensor8(0x3250,0x03);
    write_cmos_sensor8(0x325f,0xf1);
    write_cmos_sensor8(0x32d1,0xf0);
    write_cmos_sensor8(0x3301,0x06);
    write_cmos_sensor8(0x3304,0x58);
    write_cmos_sensor8(0x3306,0x60);
    write_cmos_sensor8(0x3308,0x10);
    write_cmos_sensor8(0x3309,0x68);
    write_cmos_sensor8(0x330b,0xc0);
    write_cmos_sensor8(0x330d,0x18);
    write_cmos_sensor8(0x330e,0x28);
    write_cmos_sensor8(0x331e,0x49);
    write_cmos_sensor8(0x331f,0x59);
    write_cmos_sensor8(0x3333,0x10);
    write_cmos_sensor8(0x3334,0x40);
    write_cmos_sensor8(0x3347,0x05);
    write_cmos_sensor8(0x3351,0x03);
    write_cmos_sensor8(0x336c,0x4e);
    write_cmos_sensor8(0x337f,0x13);
    write_cmos_sensor8(0x3399,0x06);
    write_cmos_sensor8(0x33ad,0x1c);
    write_cmos_sensor8(0x33b1,0x80);
    write_cmos_sensor8(0x33b2,0x50);
    write_cmos_sensor8(0x33b3,0x18);
    write_cmos_sensor8(0x3433,0x69);
    write_cmos_sensor8(0x34ae,0x16);
    write_cmos_sensor8(0x34ba,0x01);
    write_cmos_sensor8(0x34bb,0x13);
    write_cmos_sensor8(0x3633,0x46);
    write_cmos_sensor8(0x3637,0x56);
    write_cmos_sensor8(0x363e,0x34);
    write_cmos_sensor8(0x3670,0x48);
    write_cmos_sensor8(0x3690,0x44);
    write_cmos_sensor8(0x3691,0x45);
    write_cmos_sensor8(0x3692,0x66);
    write_cmos_sensor8(0x3698,0x83);
    write_cmos_sensor8(0x3699,0x85);
    write_cmos_sensor8(0x369a,0x87);
    write_cmos_sensor8(0x369b,0x8d);
    write_cmos_sensor8(0x369c,0x80);
    write_cmos_sensor8(0x369d,0xf8);
    write_cmos_sensor8(0x36a0,0xa8);
    write_cmos_sensor8(0x36a1,0xb2);
    write_cmos_sensor8(0x36a2,0x80);
    write_cmos_sensor8(0x36a3,0x88);
    write_cmos_sensor8(0x36a4,0x98);
    write_cmos_sensor8(0x36ae,0xb8);
    write_cmos_sensor8(0x36af,0xf8);
    write_cmos_sensor8(0x36b3,0x08);
    write_cmos_sensor8(0x370f,0x01);
    write_cmos_sensor8(0x3722,0x63);
    write_cmos_sensor8(0x3724,0xb1);
    write_cmos_sensor8(0x3771,0x43);
    write_cmos_sensor8(0x3772,0x43);
    write_cmos_sensor8(0x3773,0x63);
    write_cmos_sensor8(0x377a,0x80);
    write_cmos_sensor8(0x377b,0xb8);
    write_cmos_sensor8(0x3900,0x07);
    write_cmos_sensor8(0x3902,0xf0);
    write_cmos_sensor8(0x3903,0x20);
    write_cmos_sensor8(0x3905,0x8d);
    write_cmos_sensor8(0x3907,0x00);
    write_cmos_sensor8(0x3908,0x40);
    write_cmos_sensor8(0x391a,0x60);
    write_cmos_sensor8(0x391b,0x48);
    write_cmos_sensor8(0x391c,0x2d);
    write_cmos_sensor8(0x391d,0x00);
    write_cmos_sensor8(0x391f,0x41);
    write_cmos_sensor8(0x3921,0x00);
    write_cmos_sensor8(0x3922,0x01);
    write_cmos_sensor8(0x3926,0xe0);
    write_cmos_sensor8(0x397b,0x00);
    write_cmos_sensor8(0x3993,0x00);
    write_cmos_sensor8(0x39c1,0x4c);
    write_cmos_sensor8(0x39dd,0x02);
    write_cmos_sensor8(0x39de,0x02);
    write_cmos_sensor8(0x39e7,0x04);
    write_cmos_sensor8(0x39e8,0x80);
    write_cmos_sensor8(0x39e9,0x80);
    write_cmos_sensor8(0x3e00,0x00);
    write_cmos_sensor8(0x3e01,0xa7);
    write_cmos_sensor8(0x3e02,0x60);
    write_cmos_sensor8(0x3e03,0x03);
    write_cmos_sensor8(0x3e16,0x02);
    write_cmos_sensor8(0x3e17,0x00);
    write_cmos_sensor8(0x3e18,0x02);
    write_cmos_sensor8(0x3e19,0x00);
    write_cmos_sensor8(0x4200,0x0c);
    write_cmos_sensor8(0x4209,0x08);
    write_cmos_sensor8(0x420b,0x08);
    write_cmos_sensor8(0x420d,0x08);
    write_cmos_sensor8(0x420f,0x08);
    write_cmos_sensor8(0x4210,0x06);
    write_cmos_sensor8(0x4211,0x06);
    write_cmos_sensor8(0x4212,0x06);
    write_cmos_sensor8(0x4213,0x0c);
    write_cmos_sensor8(0x4215,0x80);
    write_cmos_sensor8(0x4217,0x98);
    write_cmos_sensor8(0x4219,0xb8);
    write_cmos_sensor8(0x421b,0xf8);
    write_cmos_sensor8(0x421c,0x06);
    write_cmos_sensor8(0x421d,0x0c);
    write_cmos_sensor8(0x421e,0x14);
    write_cmos_sensor8(0x421f,0x18);
    write_cmos_sensor8(0x4269,0x80);
    write_cmos_sensor8(0x426b,0x98);
    write_cmos_sensor8(0x426d,0xb8);
    write_cmos_sensor8(0x426f,0xf8);
    write_cmos_sensor8(0x4272,0x10);
    write_cmos_sensor8(0x4273,0x08);
    write_cmos_sensor8(0x4274,0x00);
    write_cmos_sensor8(0x4275,0x00);
    write_cmos_sensor8(0x4402,0x03);
    write_cmos_sensor8(0x4403,0x0b);
    write_cmos_sensor8(0x4404,0x21);
    write_cmos_sensor8(0x4405,0x2b);
    write_cmos_sensor8(0x440c,0x36);
    write_cmos_sensor8(0x440d,0x36);
    write_cmos_sensor8(0x440e,0x29);
    write_cmos_sensor8(0x440f,0x44);
    write_cmos_sensor8(0x4412,0x01);
    write_cmos_sensor8(0x4424,0x01);
    write_cmos_sensor8(0x442d,0x02);
    write_cmos_sensor8(0x442e,0x1c);
    write_cmos_sensor8(0x4509,0x20);
    write_cmos_sensor8(0x4800,0x24);
    write_cmos_sensor8(0x4837,0x1e);
    write_cmos_sensor8(0x5008,0x01);
    write_cmos_sensor8(0x5011,0x40);
    write_cmos_sensor8(0x5501,0xff);
    write_cmos_sensor8(0x5504,0x00);
    write_cmos_sensor8(0x5505,0x0c);
    write_cmos_sensor8(0x5506,0x09);
    write_cmos_sensor8(0x5507,0x13);
    write_cmos_sensor8(0x5508,0x00);
    write_cmos_sensor8(0x5509,0x10);
    write_cmos_sensor8(0x550a,0x06);
    write_cmos_sensor8(0x550b,0xd7);
    write_cmos_sensor8(0x550c,0x11);
    write_cmos_sensor8(0x550d,0x11);
    write_cmos_sensor8(0x550e,0x01);
    write_cmos_sensor8(0x550f,0xb0);
    write_cmos_sensor8(0x5514,0x0c);
    write_cmos_sensor8(0x5784,0x08);
    write_cmos_sensor8(0x5785,0x04);
    write_cmos_sensor8(0x5786,0x02);
    write_cmos_sensor8(0x5787,0x08);
    write_cmos_sensor8(0x5788,0x08);
    write_cmos_sensor8(0x5789,0x08);
    write_cmos_sensor8(0x578a,0x08);
    write_cmos_sensor8(0x578b,0x08);
    write_cmos_sensor8(0x578c,0x08);
    write_cmos_sensor8(0x578d,0x41);
    write_cmos_sensor8(0x5790,0x08);
    write_cmos_sensor8(0x5791,0x08);
    write_cmos_sensor8(0x5792,0x08);
    write_cmos_sensor8(0x5793,0x08);
    write_cmos_sensor8(0x5794,0x08);
    write_cmos_sensor8(0x5795,0x08);
    write_cmos_sensor8(0x5799,0x44);
    write_cmos_sensor8(0x57ac,0x00);
	write_cmos_sensor8(0x57ad,0x00);
    write_cmos_sensor8(0x5ae0,0x00);
}

static void normal_video_setting(void)
{
    write_cmos_sensor8(0x0103,0x01);
    write_cmos_sensor8(0x0100,0x00);
    write_cmos_sensor8(0x36e9,0x80);
    write_cmos_sensor8(0x37f9,0x80);
    write_cmos_sensor8(0x36ea,0x0a);
    write_cmos_sensor8(0x36eb,0x0c);
    write_cmos_sensor8(0x36ec,0x4b);
    write_cmos_sensor8(0x36ed,0x24);
    write_cmos_sensor8(0x37fa,0x1b);
    write_cmos_sensor8(0x37fb,0xb2);
    write_cmos_sensor8(0x37fc,0x10);
    write_cmos_sensor8(0x37fd,0x4a);
    write_cmos_sensor8(0x36e9,0x24);
    write_cmos_sensor8(0x37f9,0x24);
    write_cmos_sensor8(0x301f,0x09);
    write_cmos_sensor8(0x3200,0x00);
    write_cmos_sensor8(0x3201,0x18);
    write_cmos_sensor8(0x3202,0x01);
    write_cmos_sensor8(0x3203,0xc4);
    write_cmos_sensor8(0x3204,0x12);
    write_cmos_sensor8(0x3205,0x27);
    write_cmos_sensor8(0x3206,0x0b);
    write_cmos_sensor8(0x3207,0xf3);
    write_cmos_sensor8(0x3208,0x09);
    write_cmos_sensor8(0x3209,0x00);
    write_cmos_sensor8(0x320a,0x05);
    write_cmos_sensor8(0x320b,0x10);
    write_cmos_sensor8(0x320c,0x05);
    write_cmos_sensor8(0x320d,0x3c);
    write_cmos_sensor8(0x320e,0x0a);
    write_cmos_sensor8(0x320f,0x7e);
    write_cmos_sensor8(0x3210,0x00);
    write_cmos_sensor8(0x3211,0x04);
    write_cmos_sensor8(0x3212,0x00);
    write_cmos_sensor8(0x3213,0x0a);
    write_cmos_sensor8(0x3250,0x03);
    write_cmos_sensor8(0x325f,0xf1);
    write_cmos_sensor8(0x32d1,0xf0);
    write_cmos_sensor8(0x3301,0x06);
    write_cmos_sensor8(0x3304,0x58);
    write_cmos_sensor8(0x3306,0x60);
    write_cmos_sensor8(0x3308,0x10);
    write_cmos_sensor8(0x3309,0x68);
    write_cmos_sensor8(0x330b,0xc0);
    write_cmos_sensor8(0x330d,0x18);
    write_cmos_sensor8(0x330e,0x28);
    write_cmos_sensor8(0x331e,0x49);
    write_cmos_sensor8(0x331f,0x59);
    write_cmos_sensor8(0x3333,0x10);
    write_cmos_sensor8(0x3334,0x40);
    write_cmos_sensor8(0x3347,0x05);
    write_cmos_sensor8(0x3351,0x03);
    write_cmos_sensor8(0x336c,0x4e);
    write_cmos_sensor8(0x337f,0x13);
    write_cmos_sensor8(0x3399,0x06);
    write_cmos_sensor8(0x33ad,0x1c);
    write_cmos_sensor8(0x33b1,0x80);
    write_cmos_sensor8(0x33b2,0x50);
    write_cmos_sensor8(0x33b3,0x18);
    write_cmos_sensor8(0x3433,0x69);
    write_cmos_sensor8(0x34ae,0x16);
    write_cmos_sensor8(0x34ba,0x01);
    write_cmos_sensor8(0x34bb,0x13);
    write_cmos_sensor8(0x3633,0x46);
    write_cmos_sensor8(0x3637,0x56);
    write_cmos_sensor8(0x363e,0x34);
    write_cmos_sensor8(0x3670,0x48);
    write_cmos_sensor8(0x3690,0x44);
    write_cmos_sensor8(0x3691,0x45);
    write_cmos_sensor8(0x3692,0x66);
    write_cmos_sensor8(0x3698,0x83);
    write_cmos_sensor8(0x3699,0x85);
    write_cmos_sensor8(0x369a,0x87);
    write_cmos_sensor8(0x369b,0x8d);
    write_cmos_sensor8(0x369c,0x80);
    write_cmos_sensor8(0x369d,0xf8);
    write_cmos_sensor8(0x36a0,0xa8);
    write_cmos_sensor8(0x36a1,0xb2);
    write_cmos_sensor8(0x36a2,0x80);
    write_cmos_sensor8(0x36a3,0x88);
    write_cmos_sensor8(0x36a4,0x98);
    write_cmos_sensor8(0x36ae,0xb8);
    write_cmos_sensor8(0x36af,0xf8);
    write_cmos_sensor8(0x36b3,0x08);
    write_cmos_sensor8(0x370f,0x01);
    write_cmos_sensor8(0x3722,0x63);
    write_cmos_sensor8(0x3724,0xb1);
    write_cmos_sensor8(0x3771,0x43);
    write_cmos_sensor8(0x3772,0x43);
    write_cmos_sensor8(0x3773,0x63);
    write_cmos_sensor8(0x377a,0x80);
    write_cmos_sensor8(0x377b,0xb8);
    write_cmos_sensor8(0x3900,0x07);
    write_cmos_sensor8(0x3902,0xf0);
    write_cmos_sensor8(0x3903,0x20);
    write_cmos_sensor8(0x3905,0x8d);
    write_cmos_sensor8(0x3907,0x00);
    write_cmos_sensor8(0x3908,0x40);
    write_cmos_sensor8(0x391a,0x60);
    write_cmos_sensor8(0x391b,0x48);
    write_cmos_sensor8(0x391c,0x2d);
    write_cmos_sensor8(0x391d,0x00);
    write_cmos_sensor8(0x391f,0x41);
    write_cmos_sensor8(0x3921,0x00);
    write_cmos_sensor8(0x3922,0x01);
    write_cmos_sensor8(0x3926,0xe0);
    write_cmos_sensor8(0x397b,0x00);
    write_cmos_sensor8(0x3993,0x00);
    write_cmos_sensor8(0x39c1,0x4c);
    write_cmos_sensor8(0x39dd,0x02);
    write_cmos_sensor8(0x39de,0x02);
    write_cmos_sensor8(0x39e7,0x04);
    write_cmos_sensor8(0x39e8,0x80);
    write_cmos_sensor8(0x39e9,0x80);
    write_cmos_sensor8(0x3e00,0x00);
    write_cmos_sensor8(0x3e01,0xa7);
    write_cmos_sensor8(0x3e02,0x60);
    write_cmos_sensor8(0x3e03,0x03);
    write_cmos_sensor8(0x3e16,0x02);
    write_cmos_sensor8(0x3e17,0x00);
    write_cmos_sensor8(0x3e18,0x02);
    write_cmos_sensor8(0x3e19,0x00);
    write_cmos_sensor8(0x4200,0x0c);
    write_cmos_sensor8(0x4209,0x08);
    write_cmos_sensor8(0x420b,0x08);
    write_cmos_sensor8(0x420d,0x08);
    write_cmos_sensor8(0x420f,0x08);
    write_cmos_sensor8(0x4210,0x06);
    write_cmos_sensor8(0x4211,0x06);
    write_cmos_sensor8(0x4212,0x06);
    write_cmos_sensor8(0x4213,0x0c);
    write_cmos_sensor8(0x4215,0x80);
    write_cmos_sensor8(0x4217,0x98);
    write_cmos_sensor8(0x4219,0xb8);
    write_cmos_sensor8(0x421b,0xf8);
    write_cmos_sensor8(0x421c,0x06);
    write_cmos_sensor8(0x421d,0x0c);
    write_cmos_sensor8(0x421e,0x14);
    write_cmos_sensor8(0x421f,0x18);
    write_cmos_sensor8(0x4269,0x80);
    write_cmos_sensor8(0x426b,0x98);
    write_cmos_sensor8(0x426d,0xb8);
    write_cmos_sensor8(0x426f,0xf8);
    write_cmos_sensor8(0x4272,0x10);
    write_cmos_sensor8(0x4273,0x08);
    write_cmos_sensor8(0x4274,0x00);
    write_cmos_sensor8(0x4275,0x00);
    write_cmos_sensor8(0x4402,0x03);
    write_cmos_sensor8(0x4403,0x0b);
    write_cmos_sensor8(0x4404,0x21);
    write_cmos_sensor8(0x4405,0x2b);
    write_cmos_sensor8(0x440c,0x36);
    write_cmos_sensor8(0x440d,0x36);
    write_cmos_sensor8(0x440e,0x29);
    write_cmos_sensor8(0x440f,0x44);
    write_cmos_sensor8(0x4412,0x01);
    write_cmos_sensor8(0x4424,0x01);
    write_cmos_sensor8(0x442d,0x02);
    write_cmos_sensor8(0x442e,0x1c);
    write_cmos_sensor8(0x4509,0x20);
    write_cmos_sensor8(0x4800,0x24);
    write_cmos_sensor8(0x4837,0x1e);
    write_cmos_sensor8(0x5008,0x01);
    write_cmos_sensor8(0x5011,0x40);
    write_cmos_sensor8(0x5501,0xff);
    write_cmos_sensor8(0x5504,0x00);
    write_cmos_sensor8(0x5505,0x0c);
    write_cmos_sensor8(0x5506,0x09);
    write_cmos_sensor8(0x5507,0x13);
    write_cmos_sensor8(0x5508,0x00);
    write_cmos_sensor8(0x5509,0xe8);
    write_cmos_sensor8(0x550a,0x05);
    write_cmos_sensor8(0x550b,0xff);
    write_cmos_sensor8(0x550c,0x11);
    write_cmos_sensor8(0x550d,0x11);
    write_cmos_sensor8(0x550e,0x01);
    write_cmos_sensor8(0x550f,0xb0);
    write_cmos_sensor8(0x5514,0x0c);
    write_cmos_sensor8(0x5784,0x08);
    write_cmos_sensor8(0x5785,0x04);
    write_cmos_sensor8(0x5786,0x02);
    write_cmos_sensor8(0x5787,0x08);
    write_cmos_sensor8(0x5788,0x08);
    write_cmos_sensor8(0x5789,0x08);
    write_cmos_sensor8(0x578a,0x08);
    write_cmos_sensor8(0x578b,0x08);
    write_cmos_sensor8(0x578c,0x08);
    write_cmos_sensor8(0x578d,0x41);
    write_cmos_sensor8(0x5790,0x08);
    write_cmos_sensor8(0x5791,0x08);
    write_cmos_sensor8(0x5792,0x08);
    write_cmos_sensor8(0x5793,0x08);
    write_cmos_sensor8(0x5794,0x08);
    write_cmos_sensor8(0x5795,0x08);
    write_cmos_sensor8(0x5799,0x44);
    write_cmos_sensor8(0x57ac,0x00);
    write_cmos_sensor8(0x57ad,0x00);
    write_cmos_sensor8(0x5ae0,0x00);
}

static void hs_video_setting(void)
{

    write_cmos_sensor8(0x0103,0x01);
    write_cmos_sensor8(0x0100,0x00);
    write_cmos_sensor8(0x36e9,0x80);
    write_cmos_sensor8(0x37f9,0x80);
    write_cmos_sensor8(0x36ea,0x0a);
    write_cmos_sensor8(0x36eb,0x0c);
    write_cmos_sensor8(0x36ec,0x4b);
    write_cmos_sensor8(0x36ed,0x24);
    write_cmos_sensor8(0x37fa,0x1b);
    write_cmos_sensor8(0x37fb,0xb2);
    write_cmos_sensor8(0x37fc,0x10);
    write_cmos_sensor8(0x37fd,0x4a);
    write_cmos_sensor8(0x36e9,0x24);
    write_cmos_sensor8(0x37f9,0x24);
    write_cmos_sensor8(0x301f,0x06);
    write_cmos_sensor8(0x3200,0x00);
    write_cmos_sensor8(0x3201,0x18);
    write_cmos_sensor8(0x3202,0x00);
    write_cmos_sensor8(0x3203,0x14);
    write_cmos_sensor8(0x3204,0x12);
    write_cmos_sensor8(0x3205,0x27);
    write_cmos_sensor8(0x3206,0x0d);
    write_cmos_sensor8(0x3207,0xa3);
    write_cmos_sensor8(0x3208,0x09);
    write_cmos_sensor8(0x3209,0x00);
    write_cmos_sensor8(0x320a,0x06);
    write_cmos_sensor8(0x320b,0xc0);
    write_cmos_sensor8(0x320c,0x02);
    write_cmos_sensor8(0x320d,0x9e);
    write_cmos_sensor8(0x320e,0x0a);
    write_cmos_sensor8(0x320f,0x7e);
    write_cmos_sensor8(0x3210,0x00);
    write_cmos_sensor8(0x3211,0x04);
    write_cmos_sensor8(0x3212,0x00);
    write_cmos_sensor8(0x3213,0x0a);
    write_cmos_sensor8(0x3250,0x03);
    write_cmos_sensor8(0x325f,0xf1);
    write_cmos_sensor8(0x32d1,0xf0);
    write_cmos_sensor8(0x3301,0x06);
    write_cmos_sensor8(0x3304,0x58);
    write_cmos_sensor8(0x3306,0x60);
    write_cmos_sensor8(0x3308,0x10);
    write_cmos_sensor8(0x3309,0x68);
    write_cmos_sensor8(0x330b,0xc0);
    write_cmos_sensor8(0x330d,0x18);
    write_cmos_sensor8(0x330e,0x28);
    write_cmos_sensor8(0x331e,0x49);
    write_cmos_sensor8(0x331f,0x59);
    write_cmos_sensor8(0x3333,0x10);
    write_cmos_sensor8(0x3334,0x40);
    write_cmos_sensor8(0x3347,0x05);
    write_cmos_sensor8(0x3351,0x03);
    write_cmos_sensor8(0x336c,0x4e);
    write_cmos_sensor8(0x337f,0x13);
    write_cmos_sensor8(0x3399,0x06);
    write_cmos_sensor8(0x33ad,0x1c);
    write_cmos_sensor8(0x33b1,0x80);
    write_cmos_sensor8(0x33b2,0x50);
    write_cmos_sensor8(0x33b3,0x18);
    write_cmos_sensor8(0x3433,0x69);
    write_cmos_sensor8(0x34ae,0x16);
    write_cmos_sensor8(0x34ba,0x01);
    write_cmos_sensor8(0x34bb,0x13);
    write_cmos_sensor8(0x3633,0x46);
    write_cmos_sensor8(0x3637,0x56);
    write_cmos_sensor8(0x363e,0x34);
    write_cmos_sensor8(0x3670,0x48);
    write_cmos_sensor8(0x3690,0x44);
    write_cmos_sensor8(0x3691,0x45);
    write_cmos_sensor8(0x3692,0x66);
    write_cmos_sensor8(0x3698,0x83);
    write_cmos_sensor8(0x3699,0x85);
    write_cmos_sensor8(0x369a,0x87);
    write_cmos_sensor8(0x369b,0x8d);
    write_cmos_sensor8(0x369c,0x80);
    write_cmos_sensor8(0x369d,0xf8);
    write_cmos_sensor8(0x36a0,0xa8);
    write_cmos_sensor8(0x36a1,0xb2);
    write_cmos_sensor8(0x36a2,0x80);
    write_cmos_sensor8(0x36a3,0x88);
    write_cmos_sensor8(0x36a4,0x98);
    write_cmos_sensor8(0x36ae,0xb8);
    write_cmos_sensor8(0x36af,0xf8);
    write_cmos_sensor8(0x36b3,0x08);
    write_cmos_sensor8(0x370f,0x01);
    write_cmos_sensor8(0x3722,0x63);
    write_cmos_sensor8(0x3724,0xb1);
    write_cmos_sensor8(0x3771,0x43);
    write_cmos_sensor8(0x3772,0x43);
    write_cmos_sensor8(0x3773,0x63);
    write_cmos_sensor8(0x377a,0x80);
    write_cmos_sensor8(0x377b,0xb8);
    write_cmos_sensor8(0x3900,0x07);
    write_cmos_sensor8(0x3902,0xf0);
    write_cmos_sensor8(0x3903,0x20);
    write_cmos_sensor8(0x3905,0x8d);
    write_cmos_sensor8(0x3907,0x00);
    write_cmos_sensor8(0x3908,0x40);
    write_cmos_sensor8(0x391a,0x60);
    write_cmos_sensor8(0x391b,0x48);
    write_cmos_sensor8(0x391c,0x2d);
    write_cmos_sensor8(0x391d,0x00);
    write_cmos_sensor8(0x391f,0x41);
    write_cmos_sensor8(0x3921,0x00);
    write_cmos_sensor8(0x3922,0x01);
    write_cmos_sensor8(0x3926,0xe0);
    write_cmos_sensor8(0x397b,0x00);
    write_cmos_sensor8(0x3993,0x00);
    write_cmos_sensor8(0x39c1,0x4c);
    write_cmos_sensor8(0x39dd,0x02);
    write_cmos_sensor8(0x39de,0x02);
    write_cmos_sensor8(0x39e7,0x04);
    write_cmos_sensor8(0x39e8,0x80);
    write_cmos_sensor8(0x39e9,0x80);
    write_cmos_sensor8(0x3e00,0x00);
    write_cmos_sensor8(0x3e01,0xa7);
    write_cmos_sensor8(0x3e02,0x60);
    write_cmos_sensor8(0x3e03,0x03);
    write_cmos_sensor8(0x3e16,0x02);
    write_cmos_sensor8(0x3e17,0x00);
    write_cmos_sensor8(0x3e18,0x02);
    write_cmos_sensor8(0x3e19,0x00);
    write_cmos_sensor8(0x4200,0x0c);
    write_cmos_sensor8(0x4209,0x08);
    write_cmos_sensor8(0x420b,0x08);
    write_cmos_sensor8(0x420d,0x08);
    write_cmos_sensor8(0x420f,0x08);
    write_cmos_sensor8(0x4210,0x06);
    write_cmos_sensor8(0x4211,0x06);
    write_cmos_sensor8(0x4212,0x06);
    write_cmos_sensor8(0x4213,0x0c);
    write_cmos_sensor8(0x4215,0x80);
    write_cmos_sensor8(0x4217,0x98);
    write_cmos_sensor8(0x4219,0xb8);
    write_cmos_sensor8(0x421b,0xf8);
    write_cmos_sensor8(0x421c,0x06);
    write_cmos_sensor8(0x421d,0x0c);
    write_cmos_sensor8(0x421e,0x14);
    write_cmos_sensor8(0x421f,0x18);
    write_cmos_sensor8(0x4269,0x80);
    write_cmos_sensor8(0x426b,0x98);
    write_cmos_sensor8(0x426d,0xb8);
    write_cmos_sensor8(0x426f,0xf8);
    write_cmos_sensor8(0x4272,0x10);
    write_cmos_sensor8(0x4273,0x08);
    write_cmos_sensor8(0x4274,0x00);
    write_cmos_sensor8(0x4275,0x00);
    write_cmos_sensor8(0x4402,0x03);
    write_cmos_sensor8(0x4403,0x0b);
    write_cmos_sensor8(0x4404,0x21);
    write_cmos_sensor8(0x4405,0x2b);
    write_cmos_sensor8(0x440c,0x36);
    write_cmos_sensor8(0x440d,0x36);
    write_cmos_sensor8(0x440e,0x29);
    write_cmos_sensor8(0x440f,0x44);
    write_cmos_sensor8(0x4412,0x01);
    write_cmos_sensor8(0x4424,0x01);
    write_cmos_sensor8(0x442d,0x02);
    write_cmos_sensor8(0x442e,0x1c);
    write_cmos_sensor8(0x4509,0x20);
    write_cmos_sensor8(0x4800,0x24);
    write_cmos_sensor8(0x4837,0x1e);
    write_cmos_sensor8(0x5008,0x01);
    write_cmos_sensor8(0x5011,0x40);
    write_cmos_sensor8(0x5501,0xff);
    write_cmos_sensor8(0x5504,0x00);
    write_cmos_sensor8(0x5505,0x0c);
    write_cmos_sensor8(0x5506,0x09);
    write_cmos_sensor8(0x5507,0x13);
    write_cmos_sensor8(0x5508,0x00);
    write_cmos_sensor8(0x5509,0x10);
    write_cmos_sensor8(0x550a,0x06);
    write_cmos_sensor8(0x550b,0xd7);
    write_cmos_sensor8(0x550c,0x11);
    write_cmos_sensor8(0x550d,0x11);
    write_cmos_sensor8(0x550e,0x01);
    write_cmos_sensor8(0x550f,0xb0);
    write_cmos_sensor8(0x5514,0x0c);
    write_cmos_sensor8(0x5784,0x08);
    write_cmos_sensor8(0x5785,0x04);
    write_cmos_sensor8(0x5786,0x02);
    write_cmos_sensor8(0x5787,0x08);
    write_cmos_sensor8(0x5788,0x08);
    write_cmos_sensor8(0x5789,0x08);
    write_cmos_sensor8(0x578a,0x08);
    write_cmos_sensor8(0x578b,0x08);
    write_cmos_sensor8(0x578c,0x08);
    write_cmos_sensor8(0x578d,0x41);
    write_cmos_sensor8(0x5790,0x08);
    write_cmos_sensor8(0x5791,0x08);
    write_cmos_sensor8(0x5792,0x08);
    write_cmos_sensor8(0x5793,0x08);
    write_cmos_sensor8(0x5794,0x08);
    write_cmos_sensor8(0x5795,0x08);
    write_cmos_sensor8(0x5799,0x44);
    write_cmos_sensor8(0x57ac,0x00);
    write_cmos_sensor8(0x57ad,0x00);
    write_cmos_sensor8(0x5ae0,0x00);
}


static void slim_video_setting(void)
{
    write_cmos_sensor8(0x0103,0x01);
    write_cmos_sensor8(0x0100,0x00);
    write_cmos_sensor8(0x36e9,0x80);
    write_cmos_sensor8(0x37f9,0x80);
    write_cmos_sensor8(0x36ea,0x0a);
    write_cmos_sensor8(0x36eb,0x0c);
    write_cmos_sensor8(0x36ec,0x4b);
    write_cmos_sensor8(0x36ed,0x24);
    write_cmos_sensor8(0x37fa,0x1b);
    write_cmos_sensor8(0x37fb,0xb2);
    write_cmos_sensor8(0x37fc,0x10);
    write_cmos_sensor8(0x37fd,0x4a);
    write_cmos_sensor8(0x36e9,0x24);
    write_cmos_sensor8(0x37f9,0x24);
    write_cmos_sensor8(0x301f,0x07);
    write_cmos_sensor8(0x3200,0x00);
    write_cmos_sensor8(0x3201,0x18);
    write_cmos_sensor8(0x3202,0x00);
    write_cmos_sensor8(0x3203,0x14);
    write_cmos_sensor8(0x3204,0x12);
    write_cmos_sensor8(0x3205,0x27);
    write_cmos_sensor8(0x3206,0x0d);
    write_cmos_sensor8(0x3207,0xa3);
    write_cmos_sensor8(0x3208,0x09);
    write_cmos_sensor8(0x3209,0x00);
    write_cmos_sensor8(0x320a,0x06);
    write_cmos_sensor8(0x320b,0xc0);
    write_cmos_sensor8(0x320c,0x05);
    write_cmos_sensor8(0x320d,0x3c);
    write_cmos_sensor8(0x320e,0x0a);
    write_cmos_sensor8(0x320f,0x7e);
    write_cmos_sensor8(0x3210,0x00);
    write_cmos_sensor8(0x3211,0x04);
    write_cmos_sensor8(0x3212,0x00);
    write_cmos_sensor8(0x3213,0x0a);
    write_cmos_sensor8(0x3250,0x03);
    write_cmos_sensor8(0x325f,0xf1);
    write_cmos_sensor8(0x32d1,0xf0);
    write_cmos_sensor8(0x3301,0x06);
    write_cmos_sensor8(0x3304,0x58);
    write_cmos_sensor8(0x3306,0x60);
    write_cmos_sensor8(0x3308,0x10);
    write_cmos_sensor8(0x3309,0x68);
    write_cmos_sensor8(0x330b,0xc0);
    write_cmos_sensor8(0x330d,0x18);
    write_cmos_sensor8(0x330e,0x28);
    write_cmos_sensor8(0x331e,0x49);
    write_cmos_sensor8(0x331f,0x59);
    write_cmos_sensor8(0x3333,0x10);
    write_cmos_sensor8(0x3334,0x40);
    write_cmos_sensor8(0x3347,0x05);
    write_cmos_sensor8(0x3351,0x03);
    write_cmos_sensor8(0x336c,0x4e);
    write_cmos_sensor8(0x337f,0x13);
    write_cmos_sensor8(0x3399,0x06);
    write_cmos_sensor8(0x33ad,0x1c);
    write_cmos_sensor8(0x33b1,0x80);
    write_cmos_sensor8(0x33b2,0x50);
    write_cmos_sensor8(0x33b3,0x18);
    write_cmos_sensor8(0x3433,0x69);
    write_cmos_sensor8(0x34ae,0x16);
    write_cmos_sensor8(0x34ba,0x01);
    write_cmos_sensor8(0x34bb,0x13);
    write_cmos_sensor8(0x3633,0x46);
    write_cmos_sensor8(0x3637,0x56);
    write_cmos_sensor8(0x363e,0x34);
    write_cmos_sensor8(0x3670,0x48);
    write_cmos_sensor8(0x3690,0x44);
    write_cmos_sensor8(0x3691,0x45);
    write_cmos_sensor8(0x3692,0x66);
    write_cmos_sensor8(0x3698,0x83);
    write_cmos_sensor8(0x3699,0x85);
    write_cmos_sensor8(0x369a,0x87);
    write_cmos_sensor8(0x369b,0x8d);
    write_cmos_sensor8(0x369c,0x80);
    write_cmos_sensor8(0x369d,0xf8);
    write_cmos_sensor8(0x36a0,0xa8);
    write_cmos_sensor8(0x36a1,0xb2);
    write_cmos_sensor8(0x36a2,0x80);
    write_cmos_sensor8(0x36a3,0x88);
    write_cmos_sensor8(0x36a4,0x98);
    write_cmos_sensor8(0x36ae,0xb8);
    write_cmos_sensor8(0x36af,0xf8);
    write_cmos_sensor8(0x36b3,0x08);
    write_cmos_sensor8(0x370f,0x01);
    write_cmos_sensor8(0x3722,0x63);
    write_cmos_sensor8(0x3724,0xb1);
    write_cmos_sensor8(0x3771,0x43);
    write_cmos_sensor8(0x3772,0x43);
    write_cmos_sensor8(0x3773,0x63);
    write_cmos_sensor8(0x377a,0x80);
    write_cmos_sensor8(0x377b,0xb8);
    write_cmos_sensor8(0x3900,0x07);
    write_cmos_sensor8(0x3902,0xf0);
    write_cmos_sensor8(0x3903,0x20);
    write_cmos_sensor8(0x3905,0x8d);
    write_cmos_sensor8(0x3907,0x00);
    write_cmos_sensor8(0x3908,0x40);
    write_cmos_sensor8(0x391a,0x60);
    write_cmos_sensor8(0x391b,0x48);
    write_cmos_sensor8(0x391c,0x2d);
    write_cmos_sensor8(0x391d,0x00);
    write_cmos_sensor8(0x391f,0x41);
    write_cmos_sensor8(0x3921,0x00);
    write_cmos_sensor8(0x3922,0x01);
    write_cmos_sensor8(0x3926,0xe0);
    write_cmos_sensor8(0x397b,0x00);
    write_cmos_sensor8(0x3993,0x00);
    write_cmos_sensor8(0x39c1,0x4c);
    write_cmos_sensor8(0x39dd,0x02);
    write_cmos_sensor8(0x39de,0x02);
    write_cmos_sensor8(0x39e7,0x04);
    write_cmos_sensor8(0x39e8,0x80);
    write_cmos_sensor8(0x39e9,0x80);
    write_cmos_sensor8(0x3e00,0x00);
    write_cmos_sensor8(0x3e01,0xa7);
    write_cmos_sensor8(0x3e02,0x60);
    write_cmos_sensor8(0x3e03,0x03);
    write_cmos_sensor8(0x3e16,0x02);
    write_cmos_sensor8(0x3e17,0x00);
    write_cmos_sensor8(0x3e18,0x02);
    write_cmos_sensor8(0x3e19,0x00);
    write_cmos_sensor8(0x4200,0x0c);
    write_cmos_sensor8(0x4209,0x08);
    write_cmos_sensor8(0x420b,0x08);
    write_cmos_sensor8(0x420d,0x08);
    write_cmos_sensor8(0x420f,0x08);
    write_cmos_sensor8(0x4210,0x06);
    write_cmos_sensor8(0x4211,0x06);
    write_cmos_sensor8(0x4212,0x06);
    write_cmos_sensor8(0x4213,0x0c);
    write_cmos_sensor8(0x4215,0x80);
    write_cmos_sensor8(0x4217,0x98);
    write_cmos_sensor8(0x4219,0xb8);
    write_cmos_sensor8(0x421b,0xf8);
    write_cmos_sensor8(0x421c,0x06);
    write_cmos_sensor8(0x421d,0x0c);
    write_cmos_sensor8(0x421e,0x14);
    write_cmos_sensor8(0x421f,0x18);
    write_cmos_sensor8(0x4269,0x80);
    write_cmos_sensor8(0x426b,0x98);
    write_cmos_sensor8(0x426d,0xb8);
    write_cmos_sensor8(0x426f,0xf8);
    write_cmos_sensor8(0x4272,0x10);
    write_cmos_sensor8(0x4273,0x08);
    write_cmos_sensor8(0x4274,0x00);
    write_cmos_sensor8(0x4275,0x00);
    write_cmos_sensor8(0x4402,0x03);
    write_cmos_sensor8(0x4403,0x0b);
    write_cmos_sensor8(0x4404,0x21);
    write_cmos_sensor8(0x4405,0x2b);
    write_cmos_sensor8(0x440c,0x36);
    write_cmos_sensor8(0x440d,0x36);
    write_cmos_sensor8(0x440e,0x29);
    write_cmos_sensor8(0x440f,0x44);
    write_cmos_sensor8(0x4412,0x01);
    write_cmos_sensor8(0x4424,0x01);
    write_cmos_sensor8(0x442d,0x02);
    write_cmos_sensor8(0x442e,0x1c);
    write_cmos_sensor8(0x4509,0x20);
    write_cmos_sensor8(0x4800,0x24);
    write_cmos_sensor8(0x4837,0x1e);
    write_cmos_sensor8(0x5008,0x01);
    write_cmos_sensor8(0x5011,0x40);
    write_cmos_sensor8(0x5501,0xff);
    write_cmos_sensor8(0x5504,0x00);
    write_cmos_sensor8(0x5505,0x0c);
    write_cmos_sensor8(0x5506,0x09);
    write_cmos_sensor8(0x5507,0x13);
    write_cmos_sensor8(0x5508,0x00);
    write_cmos_sensor8(0x5509,0x10);
    write_cmos_sensor8(0x550a,0x06);
    write_cmos_sensor8(0x550b,0xd7);
    write_cmos_sensor8(0x550c,0x11);
    write_cmos_sensor8(0x550d,0x11);
    write_cmos_sensor8(0x550e,0x01);
    write_cmos_sensor8(0x550f,0xb0);
    write_cmos_sensor8(0x5514,0x0c);
    write_cmos_sensor8(0x5784,0x08);
    write_cmos_sensor8(0x5785,0x04);
    write_cmos_sensor8(0x5786,0x02);
    write_cmos_sensor8(0x5787,0x08);
    write_cmos_sensor8(0x5788,0x08);
    write_cmos_sensor8(0x5789,0x08);
    write_cmos_sensor8(0x578a,0x08);
    write_cmos_sensor8(0x578b,0x08);
    write_cmos_sensor8(0x578c,0x08);
    write_cmos_sensor8(0x578d,0x41);
    write_cmos_sensor8(0x5790,0x08);
    write_cmos_sensor8(0x5791,0x08);
    write_cmos_sensor8(0x5792,0x08);
    write_cmos_sensor8(0x5793,0x08);
    write_cmos_sensor8(0x5794,0x08);
    write_cmos_sensor8(0x5795,0x08);
    write_cmos_sensor8(0x5799,0x44);
    write_cmos_sensor8(0x57ac,0x00);
    write_cmos_sensor8(0x57ad,0x00);
    write_cmos_sensor8(0x5ae0,0x00);
}

static void custom1_setting(void)
{

    write_cmos_sensor8(0x0103,0x01);
    write_cmos_sensor8(0x0100,0x00);
    write_cmos_sensor8(0x36e9,0x80);
    write_cmos_sensor8(0x37f9,0x80);
    write_cmos_sensor8(0x36ea,0x0a);
    write_cmos_sensor8(0x36eb,0x0c);
    write_cmos_sensor8(0x36ec,0x4b);
    write_cmos_sensor8(0x36ed,0x24);
    write_cmos_sensor8(0x37fa,0x1b);
    write_cmos_sensor8(0x37fb,0xb2);
    write_cmos_sensor8(0x37fc,0x10);
    write_cmos_sensor8(0x37fd,0x4a);
    write_cmos_sensor8(0x36e9,0x24);
    write_cmos_sensor8(0x37f9,0x24);
    write_cmos_sensor8(0x301f,0x06);
    write_cmos_sensor8(0x3200,0x00);
    write_cmos_sensor8(0x3201,0x18);
    write_cmos_sensor8(0x3202,0x00);
    write_cmos_sensor8(0x3203,0x14);
    write_cmos_sensor8(0x3204,0x12);
    write_cmos_sensor8(0x3205,0x27);
    write_cmos_sensor8(0x3206,0x0d);
    write_cmos_sensor8(0x3207,0xa3);
    write_cmos_sensor8(0x3208,0x09);
    write_cmos_sensor8(0x3209,0x00);
    write_cmos_sensor8(0x320a,0x06);
    write_cmos_sensor8(0x320b,0xc0);
    write_cmos_sensor8(0x320c,0x02);
    write_cmos_sensor8(0x320d,0x9e);
    write_cmos_sensor8(0x320e,0x0a);
    write_cmos_sensor8(0x320f,0x7e);
    write_cmos_sensor8(0x3210,0x00);
    write_cmos_sensor8(0x3211,0x04);
    write_cmos_sensor8(0x3212,0x00);
    write_cmos_sensor8(0x3213,0x0a);
    write_cmos_sensor8(0x3250,0x03);
    write_cmos_sensor8(0x325f,0xf1);
    write_cmos_sensor8(0x32d1,0xf0);
    write_cmos_sensor8(0x3301,0x06);
    write_cmos_sensor8(0x3304,0x58);
    write_cmos_sensor8(0x3306,0x60);
    write_cmos_sensor8(0x3308,0x10);
    write_cmos_sensor8(0x3309,0x68);
    write_cmos_sensor8(0x330b,0xc0);
    write_cmos_sensor8(0x330d,0x18);
    write_cmos_sensor8(0x330e,0x28);
    write_cmos_sensor8(0x331e,0x49);
    write_cmos_sensor8(0x331f,0x59);
    write_cmos_sensor8(0x3333,0x10);
    write_cmos_sensor8(0x3334,0x40);
    write_cmos_sensor8(0x3347,0x05);
    write_cmos_sensor8(0x3351,0x03);
    write_cmos_sensor8(0x336c,0x4e);
    write_cmos_sensor8(0x337f,0x13);
    write_cmos_sensor8(0x3399,0x06);
    write_cmos_sensor8(0x33ad,0x1c);
    write_cmos_sensor8(0x33b1,0x80);
    write_cmos_sensor8(0x33b2,0x50);
    write_cmos_sensor8(0x33b3,0x18);
    write_cmos_sensor8(0x3433,0x69);
    write_cmos_sensor8(0x34ae,0x16);
    write_cmos_sensor8(0x34ba,0x01);
    write_cmos_sensor8(0x34bb,0x13);
    write_cmos_sensor8(0x3633,0x46);
    write_cmos_sensor8(0x3637,0x56);
    write_cmos_sensor8(0x363e,0x34);
    write_cmos_sensor8(0x3670,0x48);
    write_cmos_sensor8(0x3690,0x44);
    write_cmos_sensor8(0x3691,0x45);
    write_cmos_sensor8(0x3692,0x66);
    write_cmos_sensor8(0x3698,0x83);
    write_cmos_sensor8(0x3699,0x85);
    write_cmos_sensor8(0x369a,0x87);
    write_cmos_sensor8(0x369b,0x8d);
    write_cmos_sensor8(0x369c,0x80);
    write_cmos_sensor8(0x369d,0xf8);
    write_cmos_sensor8(0x36a0,0xa8);
    write_cmos_sensor8(0x36a1,0xb2);
    write_cmos_sensor8(0x36a2,0x80);
    write_cmos_sensor8(0x36a3,0x88);
    write_cmos_sensor8(0x36a4,0x98);
    write_cmos_sensor8(0x36ae,0xb8);
    write_cmos_sensor8(0x36af,0xf8);
    write_cmos_sensor8(0x36b3,0x08);
    write_cmos_sensor8(0x370f,0x01);
    write_cmos_sensor8(0x3722,0x63);
    write_cmos_sensor8(0x3724,0xb1);
    write_cmos_sensor8(0x3771,0x43);
    write_cmos_sensor8(0x3772,0x43);
    write_cmos_sensor8(0x3773,0x63);
    write_cmos_sensor8(0x377a,0x80);
    write_cmos_sensor8(0x377b,0xb8);
    write_cmos_sensor8(0x3900,0x07);
    write_cmos_sensor8(0x3902,0xf0);
    write_cmos_sensor8(0x3903,0x20);
    write_cmos_sensor8(0x3905,0x8d);
    write_cmos_sensor8(0x3907,0x00);
    write_cmos_sensor8(0x3908,0x40);
    write_cmos_sensor8(0x391a,0x60);
    write_cmos_sensor8(0x391b,0x48);
    write_cmos_sensor8(0x391c,0x2d);
    write_cmos_sensor8(0x391d,0x00);
    write_cmos_sensor8(0x391f,0x41);
    write_cmos_sensor8(0x3921,0x00);
    write_cmos_sensor8(0x3922,0x01);
    write_cmos_sensor8(0x3926,0xe0);
    write_cmos_sensor8(0x397b,0x00);
    write_cmos_sensor8(0x3993,0x00);
    write_cmos_sensor8(0x39c1,0x4c);
    write_cmos_sensor8(0x39dd,0x02);
    write_cmos_sensor8(0x39de,0x02);
    write_cmos_sensor8(0x39e7,0x04);
    write_cmos_sensor8(0x39e8,0x80);
    write_cmos_sensor8(0x39e9,0x80);
    write_cmos_sensor8(0x3e00,0x00);
    write_cmos_sensor8(0x3e01,0xa7);
    write_cmos_sensor8(0x3e02,0x60);
    write_cmos_sensor8(0x3e03,0x03);
    write_cmos_sensor8(0x3e16,0x02);
    write_cmos_sensor8(0x3e17,0x00);
    write_cmos_sensor8(0x3e18,0x02);
    write_cmos_sensor8(0x3e19,0x00);
    write_cmos_sensor8(0x4200,0x0c);
    write_cmos_sensor8(0x4209,0x08);
    write_cmos_sensor8(0x420b,0x08);
    write_cmos_sensor8(0x420d,0x08);
    write_cmos_sensor8(0x420f,0x08);
    write_cmos_sensor8(0x4210,0x06);
    write_cmos_sensor8(0x4211,0x06);
    write_cmos_sensor8(0x4212,0x06);
    write_cmos_sensor8(0x4213,0x0c);
    write_cmos_sensor8(0x4215,0x80);
    write_cmos_sensor8(0x4217,0x98);
    write_cmos_sensor8(0x4219,0xb8);
    write_cmos_sensor8(0x421b,0xf8);
    write_cmos_sensor8(0x421c,0x06);
    write_cmos_sensor8(0x421d,0x0c);
    write_cmos_sensor8(0x421e,0x14);
    write_cmos_sensor8(0x421f,0x18);
    write_cmos_sensor8(0x4269,0x80);
    write_cmos_sensor8(0x426b,0x98);
    write_cmos_sensor8(0x426d,0xb8);
    write_cmos_sensor8(0x426f,0xf8);
    write_cmos_sensor8(0x4272,0x10);
    write_cmos_sensor8(0x4273,0x08);
    write_cmos_sensor8(0x4274,0x00);
    write_cmos_sensor8(0x4275,0x00);
    write_cmos_sensor8(0x4402,0x03);
    write_cmos_sensor8(0x4403,0x0b);
    write_cmos_sensor8(0x4404,0x21);
    write_cmos_sensor8(0x4405,0x2b);
    write_cmos_sensor8(0x440c,0x36);
    write_cmos_sensor8(0x440d,0x36);
    write_cmos_sensor8(0x440e,0x29);
    write_cmos_sensor8(0x440f,0x44);
    write_cmos_sensor8(0x4412,0x01);
    write_cmos_sensor8(0x4424,0x01);
    write_cmos_sensor8(0x442d,0x02);
    write_cmos_sensor8(0x442e,0x1c);
    write_cmos_sensor8(0x4509,0x20);
    write_cmos_sensor8(0x4800,0x24);
    write_cmos_sensor8(0x4837,0x1e);
    write_cmos_sensor8(0x5008,0x01);
    write_cmos_sensor8(0x5011,0x40);
    write_cmos_sensor8(0x5501,0xff);
    write_cmos_sensor8(0x5504,0x00);
    write_cmos_sensor8(0x5505,0x0c);
    write_cmos_sensor8(0x5506,0x09);
    write_cmos_sensor8(0x5507,0x13);
    write_cmos_sensor8(0x5508,0x00);
    write_cmos_sensor8(0x5509,0x10);
    write_cmos_sensor8(0x550a,0x06);
    write_cmos_sensor8(0x550b,0xd7);
    write_cmos_sensor8(0x550c,0x11);
    write_cmos_sensor8(0x550d,0x11);
    write_cmos_sensor8(0x550e,0x01);
    write_cmos_sensor8(0x550f,0xb0);
    write_cmos_sensor8(0x5514,0x0c);
    write_cmos_sensor8(0x5784,0x08);
    write_cmos_sensor8(0x5785,0x04);
    write_cmos_sensor8(0x5786,0x02);
    write_cmos_sensor8(0x5787,0x08);
    write_cmos_sensor8(0x5788,0x08);
    write_cmos_sensor8(0x5789,0x08);
    write_cmos_sensor8(0x578a,0x08);
    write_cmos_sensor8(0x578b,0x08);
    write_cmos_sensor8(0x578c,0x08);
    write_cmos_sensor8(0x578d,0x41);
    write_cmos_sensor8(0x5790,0x08);
    write_cmos_sensor8(0x5791,0x08);
    write_cmos_sensor8(0x5792,0x08);
    write_cmos_sensor8(0x5793,0x08);
    write_cmos_sensor8(0x5794,0x08);
    write_cmos_sensor8(0x5795,0x08);
    write_cmos_sensor8(0x5799,0x44);
    write_cmos_sensor8(0x57ac,0x00);
    write_cmos_sensor8(0x57ad,0x00);
    write_cmos_sensor8(0x5ae0,0x00);
}


static void custom2_setting(void)
{

    write_cmos_sensor8(0x0103,0x01);
    write_cmos_sensor8(0x0100,0x00);
    write_cmos_sensor8(0x36e9,0x80);
    write_cmos_sensor8(0x37f9,0x80);
    write_cmos_sensor8(0x36ea,0x0a);
    write_cmos_sensor8(0x36eb,0x0c);
    write_cmos_sensor8(0x36ec,0x4b);
    write_cmos_sensor8(0x36ed,0x24);
    write_cmos_sensor8(0x37fa,0x1b);
    write_cmos_sensor8(0x37fb,0xb2);
    write_cmos_sensor8(0x37fc,0x10);
    write_cmos_sensor8(0x37fd,0x4a);
    write_cmos_sensor8(0x36e9,0x24);
    write_cmos_sensor8(0x37f9,0x24);
    write_cmos_sensor8(0x301f,0x06);
    write_cmos_sensor8(0x3200,0x00);
    write_cmos_sensor8(0x3201,0x18);
    write_cmos_sensor8(0x3202,0x00);
    write_cmos_sensor8(0x3203,0x14);
    write_cmos_sensor8(0x3204,0x12);
    write_cmos_sensor8(0x3205,0x27);
    write_cmos_sensor8(0x3206,0x0d);
    write_cmos_sensor8(0x3207,0xa3);
    write_cmos_sensor8(0x3208,0x09);
    write_cmos_sensor8(0x3209,0x00);
    write_cmos_sensor8(0x320a,0x06);
    write_cmos_sensor8(0x320b,0xc0);
    write_cmos_sensor8(0x320c,0x02);
    write_cmos_sensor8(0x320d,0x9e);
    write_cmos_sensor8(0x320e,0x0a);
    write_cmos_sensor8(0x320f,0x7e);
    write_cmos_sensor8(0x3210,0x00);
    write_cmos_sensor8(0x3211,0x04);
    write_cmos_sensor8(0x3212,0x00);
    write_cmos_sensor8(0x3213,0x0a);
    write_cmos_sensor8(0x3250,0x03);
    write_cmos_sensor8(0x325f,0xf1);
    write_cmos_sensor8(0x32d1,0xf0);
    write_cmos_sensor8(0x3301,0x06);
    write_cmos_sensor8(0x3304,0x58);
    write_cmos_sensor8(0x3306,0x60);
    write_cmos_sensor8(0x3308,0x10);
    write_cmos_sensor8(0x3309,0x68);
    write_cmos_sensor8(0x330b,0xc0);
    write_cmos_sensor8(0x330d,0x18);
    write_cmos_sensor8(0x330e,0x28);
    write_cmos_sensor8(0x331e,0x49);
    write_cmos_sensor8(0x331f,0x59);
    write_cmos_sensor8(0x3333,0x10);
    write_cmos_sensor8(0x3334,0x40);
    write_cmos_sensor8(0x3347,0x05);
    write_cmos_sensor8(0x3351,0x03);
    write_cmos_sensor8(0x336c,0x4e);
    write_cmos_sensor8(0x337f,0x13);
    write_cmos_sensor8(0x3399,0x06);
    write_cmos_sensor8(0x33ad,0x1c);
    write_cmos_sensor8(0x33b1,0x80);
    write_cmos_sensor8(0x33b2,0x50);
    write_cmos_sensor8(0x33b3,0x18);
    write_cmos_sensor8(0x3433,0x69);
    write_cmos_sensor8(0x34ae,0x16);
    write_cmos_sensor8(0x34ba,0x01);
    write_cmos_sensor8(0x34bb,0x13);
    write_cmos_sensor8(0x3633,0x46);
    write_cmos_sensor8(0x3637,0x56);
    write_cmos_sensor8(0x363e,0x34);
    write_cmos_sensor8(0x3670,0x48);
    write_cmos_sensor8(0x3690,0x44);
    write_cmos_sensor8(0x3691,0x45);
    write_cmos_sensor8(0x3692,0x66);
    write_cmos_sensor8(0x3698,0x83);
    write_cmos_sensor8(0x3699,0x85);
    write_cmos_sensor8(0x369a,0x87);
    write_cmos_sensor8(0x369b,0x8d);
    write_cmos_sensor8(0x369c,0x80);
    write_cmos_sensor8(0x369d,0xf8);
    write_cmos_sensor8(0x36a0,0xa8);
    write_cmos_sensor8(0x36a1,0xb2);
    write_cmos_sensor8(0x36a2,0x80);
    write_cmos_sensor8(0x36a3,0x88);
    write_cmos_sensor8(0x36a4,0x98);
    write_cmos_sensor8(0x36ae,0xb8);
    write_cmos_sensor8(0x36af,0xf8);
    write_cmos_sensor8(0x36b3,0x08);
    write_cmos_sensor8(0x370f,0x01);
    write_cmos_sensor8(0x3722,0x63);
    write_cmos_sensor8(0x3724,0xb1);
    write_cmos_sensor8(0x3771,0x43);
    write_cmos_sensor8(0x3772,0x43);
    write_cmos_sensor8(0x3773,0x63);
    write_cmos_sensor8(0x377a,0x80);
    write_cmos_sensor8(0x377b,0xb8);
    write_cmos_sensor8(0x3900,0x07);
    write_cmos_sensor8(0x3902,0xf0);
    write_cmos_sensor8(0x3903,0x20);
    write_cmos_sensor8(0x3905,0x8d);
    write_cmos_sensor8(0x3907,0x00);
    write_cmos_sensor8(0x3908,0x40);
    write_cmos_sensor8(0x391a,0x60);
    write_cmos_sensor8(0x391b,0x48);
    write_cmos_sensor8(0x391c,0x2d);
    write_cmos_sensor8(0x391d,0x00);
    write_cmos_sensor8(0x391f,0x41);
    write_cmos_sensor8(0x3921,0x00);
    write_cmos_sensor8(0x3922,0x01);
    write_cmos_sensor8(0x3926,0xe0);
    write_cmos_sensor8(0x397b,0x00);
    write_cmos_sensor8(0x3993,0x00);
    write_cmos_sensor8(0x39c1,0x4c);
    write_cmos_sensor8(0x39dd,0x02);
    write_cmos_sensor8(0x39de,0x02);
    write_cmos_sensor8(0x39e7,0x04);
    write_cmos_sensor8(0x39e8,0x80);
    write_cmos_sensor8(0x39e9,0x80);
    write_cmos_sensor8(0x3e00,0x00);
    write_cmos_sensor8(0x3e01,0xa7);
    write_cmos_sensor8(0x3e02,0x60);
    write_cmos_sensor8(0x3e03,0x03);
    write_cmos_sensor8(0x3e16,0x02);
    write_cmos_sensor8(0x3e17,0x00);
    write_cmos_sensor8(0x3e18,0x02);
    write_cmos_sensor8(0x3e19,0x00);
    write_cmos_sensor8(0x4200,0x0c);
    write_cmos_sensor8(0x4209,0x08);
    write_cmos_sensor8(0x420b,0x08);
    write_cmos_sensor8(0x420d,0x08);
    write_cmos_sensor8(0x420f,0x08);
    write_cmos_sensor8(0x4210,0x06);
    write_cmos_sensor8(0x4211,0x06);
    write_cmos_sensor8(0x4212,0x06);
    write_cmos_sensor8(0x4213,0x0c);
    write_cmos_sensor8(0x4215,0x80);
    write_cmos_sensor8(0x4217,0x98);
    write_cmos_sensor8(0x4219,0xb8);
    write_cmos_sensor8(0x421b,0xf8);
    write_cmos_sensor8(0x421c,0x06);
    write_cmos_sensor8(0x421d,0x0c);
    write_cmos_sensor8(0x421e,0x14);
    write_cmos_sensor8(0x421f,0x18);
    write_cmos_sensor8(0x4269,0x80);
    write_cmos_sensor8(0x426b,0x98);
    write_cmos_sensor8(0x426d,0xb8);
    write_cmos_sensor8(0x426f,0xf8);
    write_cmos_sensor8(0x4272,0x10);
    write_cmos_sensor8(0x4273,0x08);
    write_cmos_sensor8(0x4274,0x00);
    write_cmos_sensor8(0x4275,0x00);
    write_cmos_sensor8(0x4402,0x03);
    write_cmos_sensor8(0x4403,0x0b);
    write_cmos_sensor8(0x4404,0x21);
    write_cmos_sensor8(0x4405,0x2b);
    write_cmos_sensor8(0x440c,0x36);
    write_cmos_sensor8(0x440d,0x36);
    write_cmos_sensor8(0x440e,0x29);
    write_cmos_sensor8(0x440f,0x44);
    write_cmos_sensor8(0x4412,0x01);
    write_cmos_sensor8(0x4424,0x01);
    write_cmos_sensor8(0x442d,0x02);
    write_cmos_sensor8(0x442e,0x1c);
    write_cmos_sensor8(0x4509,0x20);
    write_cmos_sensor8(0x4800,0x24);
    write_cmos_sensor8(0x4837,0x1e);
    write_cmos_sensor8(0x5008,0x01);
    write_cmos_sensor8(0x5011,0x40);
    write_cmos_sensor8(0x5501,0xff);
    write_cmos_sensor8(0x5504,0x00);
    write_cmos_sensor8(0x5505,0x0c);
    write_cmos_sensor8(0x5506,0x09);
    write_cmos_sensor8(0x5507,0x13);
    write_cmos_sensor8(0x5508,0x00);
    write_cmos_sensor8(0x5509,0x10);
    write_cmos_sensor8(0x550a,0x06);
    write_cmos_sensor8(0x550b,0xd7);
    write_cmos_sensor8(0x550c,0x11);
    write_cmos_sensor8(0x550d,0x11);
    write_cmos_sensor8(0x550e,0x01);
    write_cmos_sensor8(0x550f,0xb0);
    write_cmos_sensor8(0x5514,0x0c);
    write_cmos_sensor8(0x5784,0x08);
    write_cmos_sensor8(0x5785,0x04);
    write_cmos_sensor8(0x5786,0x02);
    write_cmos_sensor8(0x5787,0x08);
    write_cmos_sensor8(0x5788,0x08);
    write_cmos_sensor8(0x5789,0x08);
    write_cmos_sensor8(0x578a,0x08);
    write_cmos_sensor8(0x578b,0x08);
    write_cmos_sensor8(0x578c,0x08);
    write_cmos_sensor8(0x578d,0x41);
    write_cmos_sensor8(0x5790,0x08);
    write_cmos_sensor8(0x5791,0x08);
    write_cmos_sensor8(0x5792,0x08);
    write_cmos_sensor8(0x5793,0x08);
    write_cmos_sensor8(0x5794,0x08);
    write_cmos_sensor8(0x5795,0x08);
    write_cmos_sensor8(0x5799,0x44);
    write_cmos_sensor8(0x57ac,0x00);
    write_cmos_sensor8(0x57ad,0x00);
    write_cmos_sensor8(0x5ae0,0x00);
}


static void custom3_setting(void)
{

    write_cmos_sensor8(0x0103,0x01);
    write_cmos_sensor8(0x0100,0x00);
    write_cmos_sensor8(0x36e9,0x80);
    write_cmos_sensor8(0x37f9,0x80);
    write_cmos_sensor8(0x36ea,0x0a);
    write_cmos_sensor8(0x36eb,0x0c);
    write_cmos_sensor8(0x36ec,0x4b);
    write_cmos_sensor8(0x36ed,0x24);
    write_cmos_sensor8(0x37fa,0x1b);
    write_cmos_sensor8(0x37fb,0xb2);
    write_cmos_sensor8(0x37fc,0x10);
    write_cmos_sensor8(0x37fd,0x4a);
    write_cmos_sensor8(0x36e9,0x24);
    write_cmos_sensor8(0x37f9,0x24);
    write_cmos_sensor8(0x301f,0x08);
    write_cmos_sensor8(0x3200,0x00);
    write_cmos_sensor8(0x3201,0x18);
    write_cmos_sensor8(0x3202,0x01);
    write_cmos_sensor8(0x3203,0xc4);
    write_cmos_sensor8(0x3204,0x12);
    write_cmos_sensor8(0x3205,0x27);
    write_cmos_sensor8(0x3206,0x0b);
    write_cmos_sensor8(0x3207,0xf3);
    write_cmos_sensor8(0x3208,0x09);
    write_cmos_sensor8(0x3209,0x00);
    write_cmos_sensor8(0x320a,0x05);
    write_cmos_sensor8(0x320b,0x10);
    write_cmos_sensor8(0x320c,0x02);
    write_cmos_sensor8(0x320d,0x9e);
    write_cmos_sensor8(0x320e,0x0a);
    write_cmos_sensor8(0x320f,0x7e);
    write_cmos_sensor8(0x3210,0x00);
    write_cmos_sensor8(0x3211,0x04);
    write_cmos_sensor8(0x3212,0x00);
    write_cmos_sensor8(0x3213,0x0a);
    write_cmos_sensor8(0x3250,0x03);
    write_cmos_sensor8(0x325f,0xf1);
    write_cmos_sensor8(0x32d1,0xf0);
    write_cmos_sensor8(0x3301,0x06);
    write_cmos_sensor8(0x3304,0x58);
    write_cmos_sensor8(0x3306,0x60);
    write_cmos_sensor8(0x3308,0x10);
    write_cmos_sensor8(0x3309,0x68);
    write_cmos_sensor8(0x330b,0xc0);
    write_cmos_sensor8(0x330d,0x18);
    write_cmos_sensor8(0x330e,0x28);
    write_cmos_sensor8(0x331e,0x49);
    write_cmos_sensor8(0x331f,0x59);
    write_cmos_sensor8(0x3333,0x10);
    write_cmos_sensor8(0x3334,0x40);
    write_cmos_sensor8(0x3347,0x05);
    write_cmos_sensor8(0x3351,0x03);
    write_cmos_sensor8(0x336c,0x4e);
    write_cmos_sensor8(0x337f,0x13);
    write_cmos_sensor8(0x3399,0x06);
    write_cmos_sensor8(0x33ad,0x1c);
    write_cmos_sensor8(0x33b1,0x80);
    write_cmos_sensor8(0x33b2,0x50);
    write_cmos_sensor8(0x33b3,0x18);
    write_cmos_sensor8(0x3433,0x69);
    write_cmos_sensor8(0x34ae,0x16);
    write_cmos_sensor8(0x34ba,0x01);
    write_cmos_sensor8(0x34bb,0x13);
    write_cmos_sensor8(0x3633,0x46);
    write_cmos_sensor8(0x3637,0x56);
    write_cmos_sensor8(0x363e,0x34);
    write_cmos_sensor8(0x3670,0x48);
    write_cmos_sensor8(0x3690,0x44);
    write_cmos_sensor8(0x3691,0x45);
    write_cmos_sensor8(0x3692,0x66);
    write_cmos_sensor8(0x3698,0x83);
    write_cmos_sensor8(0x3699,0x85);
    write_cmos_sensor8(0x369a,0x87);
    write_cmos_sensor8(0x369b,0x8d);
    write_cmos_sensor8(0x369c,0x80);
    write_cmos_sensor8(0x369d,0xf8);
    write_cmos_sensor8(0x36a0,0xa8);
    write_cmos_sensor8(0x36a1,0xb2);
    write_cmos_sensor8(0x36a2,0x80);
    write_cmos_sensor8(0x36a3,0x88);
    write_cmos_sensor8(0x36a4,0x98);
    write_cmos_sensor8(0x36ae,0xb8);
    write_cmos_sensor8(0x36af,0xf8);
    write_cmos_sensor8(0x36b3,0x08);
    write_cmos_sensor8(0x370f,0x01);
    write_cmos_sensor8(0x3722,0x63);
    write_cmos_sensor8(0x3724,0xb1);
    write_cmos_sensor8(0x3771,0x43);
    write_cmos_sensor8(0x3772,0x43);
    write_cmos_sensor8(0x3773,0x63);
    write_cmos_sensor8(0x377a,0x80);
    write_cmos_sensor8(0x377b,0xb8);
    write_cmos_sensor8(0x3900,0x07);
    write_cmos_sensor8(0x3902,0xf0);
    write_cmos_sensor8(0x3903,0x20);
    write_cmos_sensor8(0x3905,0x8d);
    write_cmos_sensor8(0x3907,0x00);
    write_cmos_sensor8(0x3908,0x40);
    write_cmos_sensor8(0x391a,0x60);
    write_cmos_sensor8(0x391b,0x48);
    write_cmos_sensor8(0x391c,0x2d);
    write_cmos_sensor8(0x391d,0x00);
    write_cmos_sensor8(0x391f,0x41);
    write_cmos_sensor8(0x3921,0x00);
    write_cmos_sensor8(0x3922,0x01);
    write_cmos_sensor8(0x3926,0xe0);
    write_cmos_sensor8(0x397b,0x00);
    write_cmos_sensor8(0x3993,0x00);
    write_cmos_sensor8(0x39c1,0x4c);
    write_cmos_sensor8(0x39dd,0x02);
    write_cmos_sensor8(0x39de,0x02);
    write_cmos_sensor8(0x39e7,0x04);
    write_cmos_sensor8(0x39e8,0x80);
    write_cmos_sensor8(0x39e9,0x80);
    write_cmos_sensor8(0x3e00,0x00);
    write_cmos_sensor8(0x3e01,0xa7);
    write_cmos_sensor8(0x3e02,0x60);
    write_cmos_sensor8(0x3e03,0x03);
    write_cmos_sensor8(0x3e16,0x02);
    write_cmos_sensor8(0x3e17,0x00);
    write_cmos_sensor8(0x3e18,0x02);
    write_cmos_sensor8(0x3e19,0x00);
    write_cmos_sensor8(0x4200,0x0c);
    write_cmos_sensor8(0x4209,0x08);
    write_cmos_sensor8(0x420b,0x08);
    write_cmos_sensor8(0x420d,0x08);
    write_cmos_sensor8(0x420f,0x08);
    write_cmos_sensor8(0x4210,0x06);
    write_cmos_sensor8(0x4211,0x06);
    write_cmos_sensor8(0x4212,0x06);
    write_cmos_sensor8(0x4213,0x0c);
    write_cmos_sensor8(0x4215,0x80);
    write_cmos_sensor8(0x4217,0x98);
    write_cmos_sensor8(0x4219,0xb8);
    write_cmos_sensor8(0x421b,0xf8);
    write_cmos_sensor8(0x421c,0x06);
    write_cmos_sensor8(0x421d,0x0c);
    write_cmos_sensor8(0x421e,0x14);
    write_cmos_sensor8(0x421f,0x18);
    write_cmos_sensor8(0x4269,0x80);
    write_cmos_sensor8(0x426b,0x98);
    write_cmos_sensor8(0x426d,0xb8);
    write_cmos_sensor8(0x426f,0xf8);
    write_cmos_sensor8(0x4272,0x10);
    write_cmos_sensor8(0x4273,0x08);
    write_cmos_sensor8(0x4274,0x00);
    write_cmos_sensor8(0x4275,0x00);
    write_cmos_sensor8(0x4402,0x03);
    write_cmos_sensor8(0x4403,0x0b);
    write_cmos_sensor8(0x4404,0x21);
    write_cmos_sensor8(0x4405,0x2b);
    write_cmos_sensor8(0x440c,0x36);
    write_cmos_sensor8(0x440d,0x36);
    write_cmos_sensor8(0x440e,0x29);
    write_cmos_sensor8(0x440f,0x44);
    write_cmos_sensor8(0x4412,0x01);
    write_cmos_sensor8(0x4424,0x01);
    write_cmos_sensor8(0x442d,0x02);
    write_cmos_sensor8(0x442e,0x1c);
    write_cmos_sensor8(0x4509,0x20);
    write_cmos_sensor8(0x4800,0x24);
    write_cmos_sensor8(0x4837,0x1e);
    write_cmos_sensor8(0x5008,0x01);
    write_cmos_sensor8(0x5011,0x40);
    write_cmos_sensor8(0x5501,0xff);
    write_cmos_sensor8(0x5504,0x00);
    write_cmos_sensor8(0x5505,0x0c);
    write_cmos_sensor8(0x5506,0x09);
    write_cmos_sensor8(0x5507,0x13);
    write_cmos_sensor8(0x5508,0x00);
    write_cmos_sensor8(0x5509,0xe8);
    write_cmos_sensor8(0x550a,0x05);
    write_cmos_sensor8(0x550b,0xff);
    write_cmos_sensor8(0x550c,0x11);
    write_cmos_sensor8(0x550d,0x11);
    write_cmos_sensor8(0x550e,0x01);
    write_cmos_sensor8(0x550f,0xb0);
    write_cmos_sensor8(0x5514,0x0c);
    write_cmos_sensor8(0x5784,0x08);
    write_cmos_sensor8(0x5785,0x04);
    write_cmos_sensor8(0x5786,0x02);
    write_cmos_sensor8(0x5787,0x08);
    write_cmos_sensor8(0x5788,0x08);
    write_cmos_sensor8(0x5789,0x08);
    write_cmos_sensor8(0x578a,0x08);
    write_cmos_sensor8(0x578b,0x08);
    write_cmos_sensor8(0x578c,0x08);
    write_cmos_sensor8(0x578d,0x41);
    write_cmos_sensor8(0x5790,0x08);
    write_cmos_sensor8(0x5791,0x08);
    write_cmos_sensor8(0x5792,0x08);
    write_cmos_sensor8(0x5793,0x08);
    write_cmos_sensor8(0x5794,0x08);
    write_cmos_sensor8(0x5795,0x08);
    write_cmos_sensor8(0x5799,0x44);
    write_cmos_sensor8(0x57ac,0x00);
    write_cmos_sensor8(0x57ad,0x00);
    write_cmos_sensor8(0x5ae0,0x00);
}
/*
//+bug604664,liuidijin.wt,ADD,2021/01/12,3 gong wide camera sc500cs eeprom bring up
#include "cam_cal_define.h"
#include <linux/slab.h>
static struct stCAM_CAL_DATAINFO_STRUCT sc1620cs_eeprom_data ={
	.sensorID= SC1620CS_OFILM_FRONT_II_SENSOR_ID,
	.deviceID = 0x10,
	.dataLength = 0x076A,
	.sensorVendorid = 0x18030000,
	.vendorByte = {1,2,3,4},
	.dataBuffer = NULL,
};
static struct stCAM_CAL_CHECKSUM_STRUCT sc1620cs_checksum[8] =
{
	{MODULE_ITEM,0x0000,0x0000,0x0007,0x0008,0x55},
	{AWB_ITEM,0x0009,0x0009,0x0019,0x001A,0x55},
	{LSC_ITEM,0x001B,0x001B,0x0767,0x0768,0x55},
	{TOTAL_ITEM,0x0000,0x0000,0x0768,0x0769,0x55},
	{MAX_ITEM,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x55},  // this line must haved
};
extern int imgSensorReadEepromData(struct stCAM_CAL_DATAINFO_STRUCT* pData,
	struct stCAM_CAL_CHECKSUM_STRUCT* checkData);
extern int imgSensorSetEepromData(struct stCAM_CAL_DATAINFO_STRUCT* pData);
//-bug604664,liuidijin.wt,ADD,2021/01/12,3 gong wide camera sc500cs eeprom bring up
*/

static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
        //+bug604664,liuidijin.wt,ADD,2021/01/12,3 gong wide camera sc500cs eeprom bring up
//        kal_int32 size = 0;
        //-bug604664,liuidijin.wt,ADD,2021/01/12,3 gong wide camera sc500cs eeprom bring up

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = return_sensor_id();
			if (*sensor_id == imgsensor_info.sensor_id) {
/*				//+bug604664,liuidijin.wt,ADD,2021/01/12,3 gong wide camera sc500cs eeprom bring up
				size = imgSensorReadEepromData(&sc1620cs_eeprom_data,sc1620cs_checksum);
				if((size != sc1620cs_eeprom_data.dataLength) || ((sc1620cs_eeprom_data.sensorVendorid >> 24)!= read_eeprom(0x0001))) {
					pr_err("sc1620cs get eeprom data failed\n");
					if(sc1620cs_eeprom_data.dataBuffer != NULL) {
						kfree(sc1620cs_eeprom_data.dataBuffer);
						sc1620cs_eeprom_data.dataBuffer = NULL;
					}
					//sensor_id = 0xFFFFFFFF;
					//return ERROR_SENSOR_CONNECT_FAIL;
				} else {
					pr_info("get eeprom data success\n");
					imgSensorSetEepromData(&sc1620cs_eeprom_data);
					//bug604664 liuxiangyin.wt, add, 2021/04/15, add flag for the new and old modules compatible
					get_eeprom_0x004_value=read_eeprom(0x004);
				}
*/				//-bug604664,liuidijin.wt,ADD,2021/01/12,3 gong wide camera sc500cs eeprom bring up
                get_custom_info(&addr_info, &custom_info);
				pr_info("[sc1620cs_camera_sensor]get_imgsensor_id:i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, *sensor_id);
				return ERROR_NONE;
			}
			pr_err("[sc1620cs_camera_sensor]get_imgsensor_id:Read sensor id fail, write id: 0x%x, id: 0x%x\n",
				imgsensor.i2c_write_id, *sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
		pr_err("sc1620cs Read id fail,sensor id: 0x%x\n", *sensor_id);
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

	LOG_INF("[open]: PLATFORM:MT6737,MIPI 24LANE\n");
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = return_sensor_id();
			if (sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, sensor_id);
				break;
			}

			retry--;
		} while (retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}
	if (imgsensor_info.sensor_id != sensor_id) {
		LOG_INF("open sensor id fail: 0x%x\n", sensor_id);
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	/* initail sequence write in  */
	sensor_init();

	spin_lock(&imgsensor_drv_lock);
	imgsensor.autoflicker_en = KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_en = 0;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}	/*	open  */
static kal_uint32 close(void)
{
	return ERROR_NONE;
}	/*	close  */


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
	LOG_INF("E");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	set_mirror_flip(imgsensor.mirror);
	return ERROR_NONE;
}	/*	preview   */

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
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;

	if (imgsensor.current_fps == imgsensor_info.cap.max_framerate)	{
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} else {
	 //PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}

	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("Caputre fps:%d\n", imgsensor.current_fps);
	capture_setting(imgsensor.current_fps);
	set_mirror_flip(imgsensor.mirror);
	return ERROR_NONE;

}	/* capture() */
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting();
	set_mirror_flip(imgsensor.mirror);
	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	//imgsensor.video_mode = KAL_TRUE;
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
}    /*    hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
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
}    /*    slim_video     */


static kal_uint32 custom1(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM1;
	imgsensor.pclk = imgsensor_info.custom1.pclk;
	//imgsensor.video_mode = KAL_TRUE;
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
}    /*    hs_video   */
static kal_uint32 custom2(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM2;
	imgsensor.pclk = imgsensor_info.custom2.pclk;
	//imgsensor.video_mode = KAL_TRUE;
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
}    /*    hs_video   */
static kal_uint32 custom3(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM3;
	imgsensor.pclk = imgsensor_info.custom3.pclk;
	//imgsensor.video_mode = KAL_TRUE;
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
}    /*    hs_video   */
static kal_uint32 get_resolution(
		MSDK_SENSOR_RESOLUTION_INFO_STRUCT * sensor_resolution)
{
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
}    /*    get_resolution    */


static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			MSDK_SENSOR_INFO_STRUCT *sensor_info,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */

	sensor_info->SensroInterfaceType =
	imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat =
		imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame =
		imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame =
		imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame =
		imgsensor_info.slim_video_delay_frame;
    sensor_info->Custom1DelayFrame = imgsensor_info.custom1_delay_frame;
	sensor_info->Custom2DelayFrame = imgsensor_info.custom2_delay_frame;
	sensor_info->Custom3DelayFrame = imgsensor_info.custom3_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent =
		imgsensor_info.isp_driving_current;
/* The frame of setting shutter default 0 for TG int */
	sensor_info->AEShutDelayFrame =
		imgsensor_info.ae_shut_delay_frame;
/* The frame of setting sensor gain */
	sensor_info->AESensorGainDelayFrame =
		imgsensor_info.ae_sensor_gain_delay_frame;
	sensor_info->AEISPGainDelayFrame =
		imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine =
		imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum =
		imgsensor_info.sensor_mode_num;

	sensor_info->SensorMIPILaneNumber =
		imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;  // 0 is default 1x
	sensor_info->SensorHightSampling = 0;    // 0 is default 1x
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
	    sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
	    sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;

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
	    sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
	    sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;
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
}    /*    get_info  */


static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		LOG_INF("preview\n");
		preview(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
	case MSDK_SCENARIO_ID_CAMERA_ZSD:
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
		preview(image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d ", framerate);
	// SetVideoMode Function should fix framerate
	if (framerate == 0)
		// Dynamic frame rate
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);

	if ((framerate == 30) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 15) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = 10 * framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps, 1);
	set_dummy();
	return ERROR_NONE;
}


static kal_uint32 set_auto_flicker_mode(kal_bool enable,
			UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d ", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable)
		imgsensor.autoflicker_en = KAL_TRUE;
	else //Cancel Auto flick
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(
			enum MSDK_SCENARIO_ID_ENUM scenario_id,
			MUINT32 framerate)
{
	kal_uint32 frame_length;

/*	LOG_INF("scenario_id = %d, framerate = %d\n",
				scenario_id, framerate);
*/
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
	    frame_length = imgsensor_info.pre.pclk / framerate * 10 /
			imgsensor_info.pre.linelength;
	    spin_lock(&imgsensor_drv_lock);
	    imgsensor.dummy_line = (frame_length >
			imgsensor_info.pre.framelength) ?
			(frame_length - imgsensor_info.pre.framelength) : 0;
	    imgsensor.frame_length = imgsensor_info.pre.framelength +
			imgsensor.dummy_line;
	    imgsensor.min_frame_length = imgsensor.frame_length;
	    spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
	break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if (framerate == 0)
			return ERROR_NONE;
	    frame_length = imgsensor_info.normal_video.pclk /
			framerate * 10 / imgsensor_info.normal_video.linelength;
	    spin_lock(&imgsensor_drv_lock);
	    imgsensor.dummy_line = (frame_length >
			imgsensor_info.normal_video.framelength) ?
		(frame_length - imgsensor_info.normal_video.framelength) : 0;
	    imgsensor.frame_length = imgsensor_info.normal_video.framelength +
			imgsensor.dummy_line;
	    imgsensor.min_frame_length = imgsensor.frame_length;
	    spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
	break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		if (imgsensor.current_fps ==
				imgsensor_info.cap1.max_framerate) {
		frame_length = imgsensor_info.cap1.pclk / framerate * 10 /
				imgsensor_info.cap1.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length >
			imgsensor_info.cap1.framelength) ?
			(frame_length - imgsensor_info.cap1.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.cap1.framelength +
				imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		} else {
			if (imgsensor.current_fps !=
				imgsensor_info.cap.max_framerate)
			LOG_INF("fps %d fps not support,use cap: %d fps!\n",
			framerate, imgsensor_info.cap.max_framerate/10);
			frame_length = imgsensor_info.cap.pclk /
				framerate * 10 / imgsensor_info.cap.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length >
				imgsensor_info.cap.framelength) ?
			(frame_length - imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length =
				imgsensor_info.cap.framelength +
				imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		}
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
	break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
	    frame_length = imgsensor_info.hs_video.pclk /
			framerate * 10 / imgsensor_info.hs_video.linelength;
	    spin_lock(&imgsensor_drv_lock);
	    imgsensor.dummy_line = (frame_length >
			imgsensor_info.hs_video.framelength) ? (frame_length -
			imgsensor_info.hs_video.framelength) : 0;
	    imgsensor.frame_length = imgsensor_info.hs_video.framelength +
			imgsensor.dummy_line;
	    imgsensor.min_frame_length = imgsensor.frame_length;
	    spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
	break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
	    frame_length = imgsensor_info.slim_video.pclk /
			framerate * 10 / imgsensor_info.slim_video.linelength;
	    spin_lock(&imgsensor_drv_lock);
	    imgsensor.dummy_line = (frame_length >
			imgsensor_info.slim_video.framelength) ? (frame_length -
			imgsensor_info.slim_video.framelength) : 0;
	    imgsensor.frame_length =
			imgsensor_info.slim_video.framelength +
			imgsensor.dummy_line;
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
	default:  //coding with  preview scenario by default
	    frame_length = imgsensor_info.pre.pclk / framerate * 10 /
						imgsensor_info.pre.linelength;
	    spin_lock(&imgsensor_drv_lock);
	    imgsensor.dummy_line = (frame_length >
			imgsensor_info.pre.framelength) ?
			(frame_length - imgsensor_info.pre.framelength) : 0;
	    imgsensor.frame_length = imgsensor_info.pre.framelength +
				imgsensor.dummy_line;
	    imgsensor.min_frame_length = imgsensor.frame_length;
	    spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
	    LOG_INF("error scenario_id = %d, we use preview scenario\n",
				scenario_id);
	break;
	}
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(
				enum MSDK_SCENARIO_ID_ENUM scenario_id,
				MUINT32 *framerate)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

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

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("set_test_pattern_mode enable: %d", enable);

	if (enable) {
		write_cmos_sensor8(0x4501, 0xbc);
	} else {
		write_cmos_sensor8(0x4501, 0xb4);
	}
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 streaming_control(kal_bool enable)
{
	pr_debug("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);

	if (enable)
		write_cmos_sensor8(0x0100, 0x01); // stream on
	else
		write_cmos_sensor8(0x0100, 0x00); // stream off

	mdelay(10);
	return ERROR_NONE;
}

static kal_uint32 feature_control(
			MSDK_SENSOR_FEATURE_ENUM feature_id,
			UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;
	INT32 *feature_return_para_i32 = (INT32 *) feature_para;
	unsigned long long *feature_data =
		(unsigned long long *) feature_para;

	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data =
		(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	LOG_INF("feature_id = %d\n", feature_id);
	switch (feature_id) {
	case SENSOR_FEATURE_GET_PERIOD:
	    *feature_return_para_16++ = imgsensor.line_length;
	    *feature_return_para_16 = imgsensor.frame_length;
	    *feature_para_len = 4;
	break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
	    *feature_return_para_32 = imgsensor.pclk;
	    *feature_para_len = 4;
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
		break;
	case SENSOR_FEATURE_GET_BINNING_TYPE:
		switch (*(feature_data + 1)) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*feature_return_para_32 = 1; /*BINNING_NONE*/
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			if (*(feature_data + 2))/* HDR on */
				*feature_return_para_32 = 1;/*BINNING_NONE*/
			else
				*feature_return_para_32 = 1;/*BINNING_AVERAGED*/
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		default:
			*feature_return_para_32 = 1; /*BINNING_AVERAGED*/
			break;
		}
		pr_debug("SENSOR_FEATURE_GET_BINNING_TYPE AE_binning_type:%d\n",
			*feature_return_para_32);

		*feature_para_len = 4;
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
	case SENSOR_FEATURE_SET_ESHUTTER:
	    set_shutter(*feature_data);
	break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
	    night_mode((BOOL) * feature_data);
	break;
	case SENSOR_FEATURE_SET_GAIN:
	    set_gain((UINT16) *feature_data);
	break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
	break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
	break;
	case SENSOR_FEATURE_SET_REGISTER:
	    write_cmos_sensor(sensor_reg_data->RegAddr,
						sensor_reg_data->RegData);
	break;
	case SENSOR_FEATURE_GET_REGISTER:
	    sensor_reg_data->RegData =
				read_cmos_sensor(sensor_reg_data->RegAddr);
	break;
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
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
	    *feature_return_para_32 = imgsensor_info.checksum_value;
	    *feature_para_len = 4;
	break;
	case SENSOR_FEATURE_SET_FRAMERATE:
	    LOG_INF("current fps :%d\n", (UINT32)*feature_data);
	    spin_lock(&imgsensor_drv_lock);
	    imgsensor.current_fps = *feature_data;
	    spin_unlock(&imgsensor_drv_lock);
	break;

	case SENSOR_FEATURE_SET_HDR:
	    LOG_INF("ihdr enable :%d\n", (BOOL)*feature_data);
	    spin_lock(&imgsensor_drv_lock);
	    imgsensor.ihdr_en = (BOOL)*feature_data;
	    spin_unlock(&imgsensor_drv_lock);
	break;
	case SENSOR_FEATURE_GET_CROP_INFO:
	    LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n",
				(UINT32)*feature_data);

	    wininfo = (struct SENSOR_WINSIZE_INFO_STRUCT *)
			(uintptr_t)(*(feature_data+1));

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
	    LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",
			(UINT16)*feature_data, (UINT16)*(feature_data+1),
			(UINT16)*(feature_data+2));
	#if 0
	    ihdr_write_shutter_gain((UINT16)*feature_data,
			(UINT16)*(feature_data+1), (UINT16)*(feature_data+2));
	#endif
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
    case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
    {
        kal_uint32 rate;
    
        switch (*feature_data) {
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            rate = imgsensor_info.cap.mipi_pixel_rate;
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            rate = imgsensor_info.normal_video.mipi_pixel_rate;
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            rate = imgsensor_info.hs_video.mipi_pixel_rate;
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            rate = imgsensor_info.slim_video.mipi_pixel_rate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM1:
            rate = imgsensor_info.custom1.mipi_pixel_rate;
            break; 
        case MSDK_SCENARIO_ID_CUSTOM2:
            rate = imgsensor_info.custom2.mipi_pixel_rate;
            break; 
        case MSDK_SCENARIO_ID_CUSTOM3:
            rate = imgsensor_info.custom3.mipi_pixel_rate;
            break;
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        default:
            rate = imgsensor_info.pre.mipi_pixel_rate;
            break;
        }
        LOG_INF("sc500sc SENSOR_FEATURE_GET_MIPI_PIXEL_RATE");
        *(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = rate;
    }
    break;
	case SENSOR_FEATURE_GET_TEMPERATURE_VALUE:
		*feature_return_para_i32 = 0;
		*feature_para_len = 4;
	break;
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		streaming_control(KAL_FALSE);
	break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		if (*feature_data != 0)
			set_shutter(*feature_data);
		streaming_control(KAL_TRUE);
	break;
	case SENSOR_FEATURE_GET_CUSTOM_INFO:
		if( *feature_para_len < sizeof(IMGSENSOR_GET_CUSTOM_INFO_STRUCT) ){
			LOG_INF("FeatureParaLen is too small %d\n", feature_para_len);
			kfree(feature_para);
			return -EINVAL;
		}
		memcpy((UINT8 *)feature_para, (UINT8 *)&custom_info, sizeof(IMGSENSOR_GET_CUSTOM_INFO_STRUCT));
        break;
	case SENSOR_FEATURE_GET_OFFSET_TO_START_OF_EXPOSURE:
		*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 1200000;
		break;
	default:
	break;
	}

	return ERROR_NONE;
}    /*    feature_control()  */

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 SC1620_OFILM_FRONT_II_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc =  &sensor_func;
	return ERROR_NONE;
}	/*	sc1620_MIPI_RAW_SensorInit	*/

