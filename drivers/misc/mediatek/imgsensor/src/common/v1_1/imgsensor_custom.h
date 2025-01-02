#ifndef _IMGSENSOR_CUSTOM_H
#define _IMGSENSOR_CUSTOM_H

#include "imgsensor_i2c.h"

//O17 add fuse id & module info 2024/04/07 start
enum IMGSENSOR_CUSTOM_TYPE {
	SENSOR_REAR_WIDE,
	SENSOR_FRONT_MAIN,
	SENSOR_REAR_ULTRA,
	SENSOR_REAR_MACRO,
	SENSOR_REAR_DEPTH
};

typedef struct {
    u8  eeprom_i2c_addr;
    u16 basic_info_flag_add;
    u16 fuse_id_addr;
    u16 sn_flag_addr;
    u16 sn_info_addr;
    u16 vendor_id_addr;
    u16 lens_id_addr;
    u16 sensor_id_addr;
}IMGSENSOR_CUSTOM_INFO_ADDR_STRUCT;

typedef struct {
	enum IMGSENSOR_CUSTOM_TYPE sensor_type;
	char module_head[5];
	char fuse_id[35];
	u8 fuse_id_len;
	char sn_info[15];
	u8 sn_info_len;
	u16 vendor_id;
	u16 lens_id;
	u16 sensor_id;
}IMGSENSOR_GET_CUSTOM_INFO_STRUCT;

void get_custom_info(IMGSENSOR_CUSTOM_INFO_ADDR_STRUCT* addr_info, IMGSENSOR_GET_CUSTOM_INFO_STRUCT* custom_info);

extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
//O17 add fuse id & module info 2024/04/07 end

#endif