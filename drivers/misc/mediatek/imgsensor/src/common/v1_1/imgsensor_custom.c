#include "imgsensor_custom.h"
#include <linux/i2c.h>

#define PFX "get_custom_info"
#define LOG_INF(format, args...)    \
    pr_err(PFX "[%s] " format, __func__, ##args)

u16 read_eeprom_module(u8 i2c_addr, u16 info_addr){
	u16 get_byte = 0;
	char pusendcmd[2] = { (char)(info_addr >> 8), (char)(info_addr & 0xFF) };

	iReadRegI2C(pusendcmd, 2, (u8 *) &get_byte, 1, i2c_addr);

	return get_byte;
}

void get_custom_info(IMGSENSOR_CUSTOM_INFO_ADDR_STRUCT* addr_info, IMGSENSOR_GET_CUSTOM_INFO_STRUCT* custom_info){
    u8 i2c_addr = addr_info->eeprom_i2c_addr;
	u32 i;
    if(read_eeprom_module(i2c_addr, addr_info->basic_info_flag_add) == 0x01){
        //fuse id
        for (i = 0; i < custom_info->fuse_id_len; i++) {
		    u16 fuse = read_eeprom_module(i2c_addr, addr_info->fuse_id_addr+i);
		    sprintf(custom_info->fuse_id + i*2, "%02x", fuse);
		    LOG_INF("%s %d zry fuse[%d]=0x%2x\n",__func__, __LINE__, i, fuse);
	    }
    	for(; i < 17; i++){
	        strncpy(custom_info->fuse_id + i * 2, "00", 2);
	    }
        //vendor id
        custom_info->vendor_id = read_eeprom_module(i2c_addr, addr_info->vendor_id_addr);
        custom_info->lens_id = read_eeprom_module(i2c_addr, addr_info->lens_id_addr);
        custom_info->sensor_id = read_eeprom_module(i2c_addr, addr_info->sensor_id_addr);
    }else{
        custom_info->fuse_id_len = 0;
        custom_info->vendor_id = 0;
        custom_info->lens_id = 0;
        custom_info->sensor_id = 0;
        LOG_INF("%s %d invalid basic info",__func__, __LINE__);
    }
	//sn info
    if(read_eeprom_module(i2c_addr, addr_info->sn_flag_addr) == 0x01){
	    for (i=0; i < custom_info->sn_info_len; i++) {
		    u16 sn = read_eeprom_module(i2c_addr, addr_info->sn_info_addr+i);
		    sprintf(custom_info->sn_info + i, "%c", sn);
		    pr_err("%s %d zry sn[%d]=0x%x\n",__func__, __LINE__, i, sn);
	    }
	    LOG_INF("%s %d zry sn_info=%s\n",__func__, __LINE__, custom_info->sn_info);
    }else{
        custom_info->sn_info_len = 0;
        LOG_INF("%s %d invalid sn info",__func__, __LINE__);
    }
}