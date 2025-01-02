// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/kernel.h>
#include "cam_cal_list.h"
#include "eeprom_i2c_common_driver.h"
#include "eeprom_i2c_custom_driver.h"
#include "kd_imgsensor.h"

#define MAX_EEPROM_SIZE_32K 0x8000
#define MAX_EEPROM_SIZE_16K 0x4000
extern unsigned int sc202cs_aac_read_otp_info(struct i2c_client *client,unsigned int addr, unsigned char *data, unsigned int size);
extern unsigned int sc202cs_ofilm_read_otp_info(struct i2c_client *client,unsigned int addr, unsigned char *data, unsigned int size);
struct stCAM_CAL_LIST_STRUCT g_camCalList[] = {
	/* for o17 start */
	{S5KHM6_OFILM_MAIN_I_SENSOR_ID, 0xA2, Common_read_region, MAX_EEPROM_SIZE_32K},
	{S5KHM6_AAC_MAIN_II_SENSOR_ID, 0xA2, Common_read_region},
	{IMX882_AAC_MAIN_I_SENSOR_ID, 0xA2, Common_read_region, MAX_EEPROM_SIZE_32K},
	{IMX882_OFILM_MAIN_II_SENSOR_ID, 0xA2, Common_read_region, MAX_EEPROM_SIZE_32K},
	{SC202PCS_AAC_MACRO_I_SENSOR_ID, 0xA4, Common_read_region},
	{SC202_OFILM_MACRO_II_SENSOR_ID, 0xA4, Common_read_region},
	{GC16B3C_AAC_FRONT_I_SENSOR_ID,0xA2,Common_read_region},
	{SC820CS_AAC_ULTRA_I_SENSOR_ID, 0xA0, Common_read_region},
	{SC820CS_OFILM_ULTRA_II_SENSOR_ID, 0xA0, Common_read_region},
	{SC1620_OFILM_FRONT_II_SENSOR_ID, 0xA2, Common_read_region},
	{OV20B40_AAC_FRONT_I_SENSOR_ID, 0xA2, Common_read_region},
	{OV20B40_OFILM_FRONT_II_SENSOR_ID, 0xA2, Common_read_region},
	/* for o17 end */
	/*Below is commom sensor */
	//{SC202CS_AAC_DEPTH_I_SENSOR_ID, 0x6C, sc202cs_aac_read_otp_info},
	//{SC202CS_OFILM_DEPTH_II_SENSOR_ID, 0x6C, sc202cs_ofilm_read_otp_info},
	{HI1339_SENSOR_ID, 0xB0, Common_read_region},
	{OV13B10LZ_SENSOR_ID, 0xB0, Common_read_region},
	{GC5035_SENSOR_ID,  0x7E, Common_read_region},
	{HI1339SUBOFILM_SENSOR_ID, 0xA2, Common_read_region},
	{HI1339SUBTXD_SENSOR_ID, 0xA2, Common_read_region},
	{OV48B_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{IMX766_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_32K},
	{IMX766DUAL_SENSOR_ID, 0xA2, Common_read_region, MAX_EEPROM_SIZE_16K},
	{S5K3P9SP_SENSOR_ID, 0xA0, Common_read_region},
	{IMX481_SENSOR_ID, 0xA2, Common_read_region},
	{IMX586_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{IMX576_SENSOR_ID, 0xA2, Common_read_region},
	{IMX519_SENSOR_ID, 0xA0, Common_read_region},
	{S5K3M5SX_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{IMX350_SENSOR_ID, 0xA0, Common_read_region},
	{IMX499_SENSOR_ID, 0xA0, Common_read_region},
	/*  ADD before this line */
	{0, 0, 0}       /*end of list */
};

unsigned int cam_cal_get_sensor_list(
	struct stCAM_CAL_LIST_STRUCT **ppCamcalList)
{
	if (ppCamcalList == NULL)
		return 1;

	*ppCamcalList = &g_camCalList[0];
	return 0;
}


