/*
 * Copyright (C) 2016 Xiaomi Inc.
 * Copyright (C) 2021 XiaoMi, Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#ifdef CONFIG_ADB_WRITE_PARAM_FEATURE

#include <linux/device.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include "panel_set_disp_param.h"
#include "ddp_drv.h"
#include "lcm_drv.h"
#include "disp_recovery.h"
#include <linux/notifier.h>
#include <linux/fb_notifier.h>

#define REGFLAG_DELAY           0xFE
#define REGFLAG_END_OF_TABLE    0xFF   /* END OF REGISTERS MARKER */

unsigned char xy_writepoint[16] = {0};

#ifdef CONFIG_BACKLIGHT_SUPPORT_LM36273
extern int hbm_brightness_set(int level);
#else
int hbm_brightness_set(int level) { return 0; }
#endif

static struct LCM_setting_table lcm_dsi_dispparam_cabcguion_command[] = {
		{0x55, 1, {0x01}},
		{REGFLAG_END_OF_TABLE, 0x00, {}}

};

static struct LCM_setting_table lcm_dsi_dispparam_cabcstillon_command[] = {
		{0x55, 1, {0x02}},
		{REGFLAG_END_OF_TABLE, 0x00, {}}

};

static struct LCM_setting_table lcm_dsi_dispparam_cabcmovieon_command[] = {
		{0x55, 1, {0x03}},
		{REGFLAG_END_OF_TABLE, 0x00, {}}

};

static struct LCM_setting_table lcm_dsi_dispparam_cabcoff_command[] = {
		{0x55, 1, {0x00}},
		{REGFLAG_END_OF_TABLE, 0x00, {}}

};

static struct LCM_setting_table lcm_dsi_dispparam_skin_ce_cabcguion_command[] = {
		{0x55, 1, {0x01}},
		{REGFLAG_END_OF_TABLE, 0x00, {}}

};

static struct LCM_setting_table lcm_dsi_dispparam_skin_ce_cabcstillon_command[] = {
		{0x55, 1, {0x02}},
		{REGFLAG_END_OF_TABLE, 0x00, {}}

};

static struct LCM_setting_table lcm_dsi_dispparam_skin_ce_cabcmovieon_command[] = {
		{0x55, 1, {0x03}},
		{REGFLAG_END_OF_TABLE, 0x00, {}}

};

static struct LCM_setting_table lcm_dsi_dispparam_skin_ce_cabcoff_command[] = {
		{0x55, 1, {0x00}},
		{REGFLAG_END_OF_TABLE, 0x00, {}}

};

static struct LCM_setting_table lcm_dsi_dispparam_dimmingon_command[] = {
		{0x53, 1, {0x2C}},
		{REGFLAG_END_OF_TABLE, 0x00, {}}

};

static struct LCM_setting_table lcm_dsi_dispparam_dimmingoff_command[] = {

};

static struct LCM_setting_table lcm_dsi_dispparam_idleon_command[] = {

};

static struct LCM_setting_table lcm_dsi_dispparam_warm_command[] = {

};

static struct LCM_setting_table lcm_dsi_dispparam_cold_command[] = {
		{0xF0, 0x05, {0x55, 0xAA, 0x52, 0x08, 0x02} },
		{REGFLAG_DELAY, 1, {} },
		{0xD1, 0x10, {0x01, 0x23, 0x01, 0x29, 0x01, 0x35, 0x01, 0x42, 0x01, 0x4A, 0x01, 0x5C, 0x01, 0x6E, 0x01, 0x8A} },
		{0xD2, 0x10, {0x01, 0xA7, 0x01, 0xCF, 0x01, 0xF3, 0x02, 0x2E, 0x02, 0x5E, 0x02, 0x5F, 0x02, 0x8C, 0x02, 0xC1} },
		{0xD3, 0x10, {0x02, 0xE5, 0x03, 0x16, 0x03, 0x36, 0x03, 0x60, 0x03, 0x7C, 0x03, 0x9C, 0x03, 0xAE, 0x03, 0xC6} },
		{0xD4, 0x04, {0x03, 0xFC, 0x03, 0xFF} },
		{REGFLAG_DELAY, 1, {} },
		{0xD5, 0x10, {0x01, 0x0D, 0x01, 0x14, 0x01, 0x21, 0x01, 0x2D, 0x01, 0x38, 0x01, 0x4C, 0x01, 0x5E, 0x01, 0x7E} },
		{0xD6, 0x10, {0x01, 0x99, 0x01, 0xC6, 0x01, 0xEB, 0x02, 0x27, 0x02, 0x59, 0x02, 0x5A, 0x02, 0x88, 0x02, 0xBE} },
		{0xD7, 0x10, {0x02, 0xE2, 0x03, 0x13, 0x03, 0x34, 0x03, 0x5F, 0x03, 0x79, 0x03, 0x99, 0x03, 0xAD, 0x03, 0xC8} },
		{0xD8, 0x04, {0x03, 0xEF, 0x03, 0xFF} },
		{REGFLAG_DELAY, 1, {} },
		{0xD9, 0x10, {0x00, 0x00, 0x00, 0x2D, 0x00, 0x63, 0x00, 0x8D, 0x00, 0xA3, 0x00, 0xCE, 0x00, 0xF2, 0x01, 0x26} },
		{0xDD, 0x10, {0x01, 0x55, 0x01, 0x91, 0x01, 0xC3, 0x02, 0x0F, 0x02, 0x48, 0x02, 0x4A, 0x02, 0x7C, 0x02, 0xB4} },
		{0xDE, 0x10, {0x02, 0xD9, 0x03, 0x0E, 0x03, 0x33, 0x03, 0x6F, 0x03, 0x93, 0x03, 0xD7, 0x03, 0xDE, 0x03, 0xE0} },
		{0xDF, 0x04, {0x03, 0xEC, 0x03, 0xFF} },
		{REGFLAG_DELAY, 1, {} },
		{0xE0, 0x10, {0x01, 0x23, 0x01, 0x29, 0x01, 0x35, 0x01, 0x42, 0x01, 0x4A, 0x01, 0x5C, 0x01, 0x6E, 0x01, 0x8A} },
		{0xE1, 0x10, {0x01, 0xA7, 0x01, 0xCF, 0x01, 0xF3, 0x02, 0x2E, 0x02, 0x5E, 0x02, 0x5F, 0x02, 0x8C, 0x02, 0xC1} },
		{0xE2, 0x10, {0x02, 0xE5, 0x03, 0x16, 0x03, 0x36, 0x03, 0x60, 0x03, 0x7C, 0x03, 0x9C, 0x03, 0xAE, 0x03, 0xC6} },
		{0xE3, 0x04, {0x03, 0xFC, 0x03, 0xFF} },
		{REGFLAG_DELAY, 1, {} },
		{0xE4, 0x10, {0x01, 0x0D, 0x01, 0x14, 0x01, 0x21, 0x01, 0x2D, 0x01, 0x38, 0x01, 0x4C, 0x01, 0x5E, 0x01, 0x7E} },
		{0xE5, 0x10, {0x01, 0x99, 0x01, 0xC6, 0x01, 0xEB, 0x02, 0x27, 0x02, 0x59, 0x02, 0x5A, 0x02, 0x88, 0x02, 0xBE} },
		{0xE6, 0x10, {0x02, 0xE2, 0x03, 0x13, 0x03, 0x34, 0x03, 0x5F, 0x03, 0x79, 0x03, 0x99, 0x03, 0xAD, 0x03, 0xC8} },
		{0xE7, 0x04, {0x03, 0xEF, 0x03, 0xFF} },
		{REGFLAG_DELAY, 1, {} },
		{0xE8, 0x10, {0x00, 0x00, 0x00, 0x2D, 0x00, 0x63, 0x00, 0x8D, 0x00, 0xA3, 0x00, 0xCE, 0x00, 0xF2, 0x01, 0x26} },
		{0xE9, 0x10, {0x01, 0x55, 0x01, 0x91, 0x01, 0xC3, 0x02, 0x0F, 0x02, 0x48, 0x02, 0x4A, 0x02, 0x7C, 0x02, 0xB4} },
		{0xEA, 0x10, {0x02, 0xD9, 0x03, 0x0E, 0x03, 0x33, 0x03, 0x6F, 0x03, 0x93, 0x03, 0xD7, 0x03, 0xDE, 0x03, 0xE0} },
		{0xEB, 0x04, {0x03, 0xEC, 0x03, 0xFF} },
		{REGFLAG_DELAY, 1, {} },
		{0xF0, 0x05, {0x55, 0xAA, 0x52, 0x00, 0x00}},
		{REGFLAG_END_OF_TABLE, 0x00, {}}


};

static struct LCM_setting_table lcm_dsi_dispparam_papermode_command[] = {

};

static struct LCM_setting_table lcm_dsi_dispparam_papermode1_command[] = {

};

static struct LCM_setting_table lcm_dsi_dispparam_papermode2_command[] = {

};

static struct LCM_setting_table lcm_dsi_dispparam_papermode3_command[] = {

};

static struct LCM_setting_table lcm_dsi_dispparam_papermode4_command[] = {

};

static struct LCM_setting_table lcm_dsi_dispparam_papermode5_command[] = {

};

static struct LCM_setting_table lcm_dsi_dispparam_papermode6_command[] = {

};

static struct LCM_setting_table lcm_dsi_dispparam_papermode7_command[] = {

};

static struct LCM_setting_table lcm_dsi_dispparam_default_command[] = {

};

static struct LCM_setting_table lcm_dsi_dispparam_ceon_command[] = {
		{0xF0, 5, {0x55, 0xAA, 0x52, 0x08, 0x00} },
		{0xCE, 0x0C, {0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20} },
		{0xF0, 5, {0x55, 0xAA, 0x52, 0x00, 0x00} },
		{0x55, 1, {0x80}},
		{REGFLAG_END_OF_TABLE, 0x00, {}}

};

static struct LCM_setting_table lcm_dsi_dispparam_cecabc_on_command[] = {
		{0xF0, 5, {0x55, 0xAA, 0x52, 0x08, 0x00} },
		{0xCE, 0x0C, {0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20} },
		{0xF0, 5, {0x55, 0xAA, 0x52, 0x00, 0x00} },
		{0x55, 1, {0x81}},
		{REGFLAG_END_OF_TABLE, 0x00, {}}

};

static void value2str(char *pbuf)
{
	int i = 0;
	int param_nb = 0;
	int write_len = 0;
	unsigned char *pTemp = &xy_writepoint[0];

	/* pbuf - buffer size must >= 256 */
	for (i = 0; i < 2; i++) {
		write_len = scnprintf(pTemp, 8, "p%d=%d", param_nb, pbuf[i]);
		pTemp += write_len;
		param_nb++;
	}
	pr_debug("read %s  from panel\n", xy_writepoint);
}

static void lcm_dsi_dispparam_xy_writepoint(enum panel_type panel_type)
{
	struct dsi_cmd_desc read_tab;
	struct dsi_cmd_desc write_tab;
	unsigned int i = 0;
	if (PANEL_SUMSUNG_FT3418 == panel_type) {
		//switch F0 write 5A A5
		write_tab.dtype = 0xF0;
		write_tab.dlen = 1;
		write_tab.payload = vmalloc(2 * sizeof(unsigned char));
		write_tab.payload[0] = 0x5A;
		write_tab.payload[1] = 0xA5;
		write_tab.vc = 0;
		write_tab.link_state = 1;

		//switch B0 write 11 A1
		write_tab.dtype = 0xB0;
		write_tab.dlen = 1;
		write_tab.payload = vmalloc(2 * sizeof(unsigned char));
		write_tab.payload[0] = 0x11;
		write_tab.payload[1] = 0xA1;
		write_tab.vc = 0;
		write_tab.link_state = 1;

		/*read xy writepoint info*/
		memset(&read_tab, 0, sizeof(struct dsi_cmd_desc));
		read_tab.dtype = 0xA1;
		read_tab.payload = kmalloc(2 * sizeof(unsigned char), GFP_KERNEL);
		memset(read_tab.payload, 0, 2);
		read_tab.dlen = 1;

		do_lcm_vdo_lp_write_without_lock(&write_tab, 1);
		do_lcm_vdo_lp_read_without_lock(&read_tab, 1);

		read_tab.dtype = 0xA1;
		read_tab.payload++;
		read_tab.dlen = 1;
		do_lcm_vdo_lp_read_without_lock(&read_tab, 1);

		read_tab.payload--;
		pr_debug("[%s]read xy writepoint from panel\n", __func__);
		for (i = 0; i < 2; i++)
			pr_debug("[%s]0x%x\n",  __func__, read_tab.payload[i]);

		//switch F0 write 5A A5
		write_tab.dtype = 0xF0;
		write_tab.dlen = 1;
		write_tab.payload = vmalloc(2 * sizeof(unsigned char));
		write_tab.payload[0] = 0x5A;
		write_tab.payload[1] = 0xA5;
		write_tab.vc = 0;
		write_tab.link_state = 1;

		//switch B0 write 11 A1
		write_tab.dtype = 0xB0;
		write_tab.dlen = 1;
		write_tab.payload = vmalloc(2 * sizeof(unsigned char));
		write_tab.payload[0] = 0x11;
		write_tab.payload[1] = 0xA1;
		write_tab.vc = 0;
		write_tab.link_state = 1;
		do_lcm_vdo_lp_write_without_lock(&write_tab, 1);

		value2str(read_tab.payload);

		kfree(read_tab.payload);
		vfree(write_tab.payload);
	}
	return ;
}


int panel_disp_backlight_send_lock(unsigned int level)
{
	struct dsi_cmd_desc backlight_set;
	static int Compensation_level=0x00;

	if(level >=0 && level <= 31 && Compensation_level!=0x1D){

		Compensation_level = 0x1D;
		backlight_set.payload = vmalloc(sizeof(unsigned char));
		backlight_set.vc = 0;
		backlight_set.dlen = 1;
		backlight_set.link_state = 1;
		backlight_set.dtype = 0xFE;
		backlight_set.payload[0] = 0x20;
		do_lcm_vdo_lp_brief_write_without_lock(&backlight_set,1);

		backlight_set.dtype = 0xB8;
		backlight_set.payload[0] = 0x1D;
		do_lcm_vdo_lp_brief_write_without_lock(&backlight_set,1);

		backlight_set.dtype = 0xB9;
		backlight_set.payload[0] = 0x1D;
		do_lcm_vdo_lp_brief_write_without_lock(&backlight_set,1);

		backlight_set.dtype = 0xBA;
		backlight_set.payload[0] = 0x1D;
		do_lcm_vdo_lp_brief_write_without_lock(&backlight_set,1);

		backlight_set.dtype = 0xFE;
		backlight_set.payload[0] = 0x00;
		do_lcm_vdo_lp_brief_write_without_lock(&backlight_set,1);
	}
	else if(level > 31 && level <= 509 && Compensation_level!=0x12){

		Compensation_level = 0x12;
		backlight_set.payload = vmalloc(sizeof(unsigned char));
		backlight_set.vc = 0;
		backlight_set.dlen = 1;
		backlight_set.link_state = 1;
		backlight_set.dtype = 0xFE;
		backlight_set.payload[0] = 0x20;
		do_lcm_vdo_lp_brief_write_without_lock(&backlight_set,1);

		backlight_set.dtype = 0xB8;
		backlight_set.payload[0] = 0x12;
		do_lcm_vdo_lp_brief_write_without_lock(&backlight_set,1);

		backlight_set.dtype = 0xB9;
		backlight_set.payload[0] = 0x12;
		do_lcm_vdo_lp_brief_write_without_lock(&backlight_set,1);

		backlight_set.dtype = 0xBA;
		backlight_set.payload[0] = 0x12;
		do_lcm_vdo_lp_brief_write_without_lock(&backlight_set,1);

		backlight_set.dtype = 0xFE;
		backlight_set.payload[0] = 0x00;
		do_lcm_vdo_lp_brief_write_without_lock(&backlight_set,1);
	}
	else if(level >=510 && level <= 767 && Compensation_level!=0x0D){

		Compensation_level = 0x0D;
		backlight_set.payload = vmalloc(sizeof(unsigned char));
		backlight_set.vc = 0;
		backlight_set.dlen = 1;
		backlight_set.link_state = 1;
		backlight_set.dtype = 0xFE;
		backlight_set.payload[0] = 0x20;
		do_lcm_vdo_lp_brief_write_without_lock(&backlight_set,1);

		backlight_set.dtype = 0xB8;
		backlight_set.payload[0] = 0x0D;
		do_lcm_vdo_lp_brief_write_without_lock(&backlight_set,1);

		backlight_set.dtype = 0xB9;
		backlight_set.payload[0] = 0x0D;
		do_lcm_vdo_lp_brief_write_without_lock(&backlight_set,1);

		backlight_set.dtype = 0xBA;
		backlight_set.payload[0] = 0x0D;
		do_lcm_vdo_lp_brief_write_without_lock(&backlight_set,1);
		
		backlight_set.dtype = 0xFE;
		backlight_set.payload[0] = 0x00;
		do_lcm_vdo_lp_brief_write_without_lock(&backlight_set,1);

		pr_debug("normal3 %d",level);
	}
	else if(level >=768 && level <= 1023 && Compensation_level!=0x0B){

		Compensation_level = 0x0B;
		backlight_set.payload = vmalloc(sizeof(unsigned char));
		backlight_set.vc = 0;
		backlight_set.dlen = 1;
		backlight_set.link_state = 1;
		backlight_set.dtype = 0xFE;
		backlight_set.payload[0] = 0x20;
		do_lcm_vdo_lp_brief_write_without_lock(&backlight_set,1);

		backlight_set.dtype = 0xB8;
		backlight_set.payload[0] = 0x0B;
		do_lcm_vdo_lp_brief_write_without_lock(&backlight_set,1);

		backlight_set.dtype = 0xB9;
		backlight_set.payload[0] = 0x0B;
		do_lcm_vdo_lp_brief_write_without_lock(&backlight_set,1);
		
		backlight_set.dtype = 0xBA;
		backlight_set.payload[0] = 0x0B;
		do_lcm_vdo_lp_brief_write_without_lock(&backlight_set,1);
		
		backlight_set.dtype = 0xFE;
		backlight_set.payload[0] = 0x00;
		do_lcm_vdo_lp_brief_write_without_lock(&backlight_set,1);

		pr_debug("normal3 %d",level);
	}
	else if(level >=1024 && level <= 1279 && Compensation_level!=0x0A){

		Compensation_level = 0x0A;
		backlight_set.payload = vmalloc(sizeof(unsigned char));
		backlight_set.vc = 0;
		backlight_set.dlen = 1;
		backlight_set.link_state = 1;
		backlight_set.dtype = 0xFE;
		backlight_set.payload[0] = 0x20;
		do_lcm_vdo_lp_brief_write_without_lock(&backlight_set,1);
		
		backlight_set.dtype = 0xB8;
		backlight_set.payload[0] = 0x0A;
		do_lcm_vdo_lp_brief_write_without_lock(&backlight_set,1);
		
		backlight_set.dtype = 0xB9;
		backlight_set.payload[0] = 0x0A;
		do_lcm_vdo_lp_brief_write_without_lock(&backlight_set,1);
		
		backlight_set.dtype = 0xBA;
		backlight_set.payload[0] = 0x0A;
		do_lcm_vdo_lp_brief_write_without_lock(&backlight_set,1);
		
		backlight_set.dtype = 0xFE;
		backlight_set.payload[0] = 0x00;
		do_lcm_vdo_lp_brief_write_without_lock(&backlight_set,1);

		pr_debug("[LCM]Normal4_backlight: level = %d\n", level);
	}
	else if(level >=1280 && level <= 1535 && Compensation_level!=0x09){

		Compensation_level = 0x09;
		backlight_set.payload = vmalloc(sizeof(unsigned char));
		backlight_set.vc = 0;
		backlight_set.dlen = 1;
		backlight_set.link_state = 1;
		backlight_set.dtype = 0xFE;
		backlight_set.payload[0] = 0x20;
		do_lcm_vdo_lp_brief_write_without_lock(&backlight_set,1);
		
		backlight_set.dtype = 0xB8;
		backlight_set.payload[0] = 0x09;
		do_lcm_vdo_lp_brief_write_without_lock(&backlight_set,1);
		
		backlight_set.dtype = 0xB9;
		backlight_set.payload[0] = 0x09;
		do_lcm_vdo_lp_brief_write_without_lock(&backlight_set,1);
		
		backlight_set.dtype = 0xBA;
		backlight_set.payload[0] = 0x09;
		do_lcm_vdo_lp_brief_write_without_lock(&backlight_set,1);
		
		backlight_set.dtype = 0xFE;
		backlight_set.payload[0] = 0x00;
		do_lcm_vdo_lp_brief_write_without_lock(&backlight_set,1);

		pr_debug("[Normal5_backlight: level = %d\n",level);
	}
	else if(level >=1536 && Compensation_level!=0x08){

		Compensation_level = 0x08;
		backlight_set.payload = vmalloc(sizeof(unsigned char));
		backlight_set.vc = 0;
		backlight_set.dlen = 1;
		backlight_set.link_state = 1;
		backlight_set.dtype = 0xFE;
		backlight_set.payload[0] = 0x20;
		do_lcm_vdo_lp_brief_write_without_lock(&backlight_set,1);
		
		backlight_set.dtype = 0xB8;
		backlight_set.payload[0] = 0x08;
		do_lcm_vdo_lp_brief_write_without_lock(&backlight_set,1);
		
		backlight_set.dtype = 0xB9;
		backlight_set.payload[0] = 0x08;
		do_lcm_vdo_lp_brief_write_without_lock(&backlight_set,1);
		
		backlight_set.dtype = 0xBA;
		backlight_set.payload[0] = 0x08;
		do_lcm_vdo_lp_brief_write_without_lock(&backlight_set,1);
		
		backlight_set.dtype = 0xFE;
		backlight_set.payload[0] = 0x00;
		do_lcm_vdo_lp_brief_write_without_lock(&backlight_set,1);

		pr_debug("[LCM]Normal6_backlight: level = %d lcd_bl_en = %d\n",level);
	}
	vfree(backlight_set.payload);
	return 0;
}

int panel_disp_param_send_lock(enum panel_type panel_type,unsigned int param, send_cmd p_func)
{
	struct dsi_cmd_desc dimming_hbm_off;
	struct dsi_cmd_desc dimming_hbm_on;
	struct dsi_cmd_desc hbm_on;
	struct dsi_cmd_desc cmd_normal_tab[6];
	struct dsi_cmd_desc cmd_p3_tab[8];
	struct dsi_cmd_desc cmd_srgb_tab[8];
	struct dsi_cmd_desc doze_brightness_set;
	struct fb_drm_notify_data g_notify_data;

	unsigned int tmp;
	int i = 0;
	int event = FB_DRM_BLANK_POWERDOWN;
	send_cmd push_table;
	push_table = p_func;
	g_notify_data.data = &event;
	pr_debug("[%s]panel_type = %d,param = 0x%x\n",__func__,panel_type,param);

	tmp = param & 0x0000000F;

	switch (tmp) {
	case DISPPARAM_WARM:		/* warm */
		pr_debug("warm\n");
		push_table(lcm_dsi_dispparam_warm_command,
			sizeof(lcm_dsi_dispparam_warm_command) / sizeof(struct LCM_setting_table), 1);
		break;
	case DISPPARAM_DEFAULT:		/* normal */
		pr_debug("default\n");
		push_table(lcm_dsi_dispparam_default_command,
			sizeof(lcm_dsi_dispparam_default_command) / sizeof(struct LCM_setting_table), 1);
		break;
	case DISPPARAM_COLD:		/* cold */
		pr_debug("cold\n");
		push_table(lcm_dsi_dispparam_cold_command,
			sizeof(lcm_dsi_dispparam_cold_command) / sizeof(struct LCM_setting_table), 1);
		break;
	case DISPPARAM_PAPERMODE8:
		pr_debug("paper mode\n");
		push_table(lcm_dsi_dispparam_papermode_command,
			sizeof(lcm_dsi_dispparam_papermode_command) / sizeof(struct LCM_setting_table), 1);
		break;
	case DISPPARAM_PAPERMODE1:
		pr_debug("paper mode1\n");
		push_table(lcm_dsi_dispparam_papermode1_command,
			sizeof(lcm_dsi_dispparam_papermode1_command) / sizeof(struct LCM_setting_table), 1);
		break;
	case DISPPARAM_PAPERMODE2:
		pr_debug("paper mode2\n");
		push_table(lcm_dsi_dispparam_papermode2_command,
			sizeof(lcm_dsi_dispparam_papermode2_command) / sizeof(struct LCM_setting_table), 1);
		break;
	case DISPPARAM_PAPERMODE3:
		pr_debug("paper mode3\n");
		push_table(lcm_dsi_dispparam_papermode3_command,
			sizeof(lcm_dsi_dispparam_papermode3_command) / sizeof(struct LCM_setting_table), 1);
		break;
	case DISPPARAM_PAPERMODE4:
		pr_debug("paper mode4\n");
		push_table(lcm_dsi_dispparam_papermode4_command,
			sizeof(lcm_dsi_dispparam_papermode4_command) / sizeof(struct LCM_setting_table), 1);
		break;
	case DISPPARAM_PAPERMODE5:
		pr_debug("paper mode5\n");
		push_table(lcm_dsi_dispparam_papermode5_command,
			sizeof(lcm_dsi_dispparam_papermode5_command) / sizeof(struct LCM_setting_table), 1);
		break;
	case DISPPARAM_PAPERMODE6:
		pr_debug("paper mode6\n");
		push_table(lcm_dsi_dispparam_papermode6_command,
			sizeof(lcm_dsi_dispparam_papermode6_command) / sizeof(struct LCM_setting_table), 1);
		break;
	case DISPPARAM_PAPERMODE7:
		pr_debug("paper mode7\n");
		push_table(lcm_dsi_dispparam_papermode7_command,
			sizeof(lcm_dsi_dispparam_papermode7_command) / sizeof(struct LCM_setting_table), 1);
		break;
	case DISPPARAM_WHITEPOINT_XY:
		lcm_dsi_dispparam_xy_writepoint(panel_type);
		break;
	default:
		break;
	}

	tmp = param & 0x000000F0;
	switch (tmp) {
	case DISPPARAM_CE_ON:
		pr_debug("ce on\n");
		push_table(lcm_dsi_dispparam_ceon_command,
			sizeof(lcm_dsi_dispparam_ceon_command) / sizeof(struct LCM_setting_table), 1);
		break;
	case DISPPARAM_CE_OFF:
		pr_debug("ce&cabc on\n");
		push_table(lcm_dsi_dispparam_cecabc_on_command,
			sizeof(lcm_dsi_dispparam_cecabc_on_command) / sizeof(struct LCM_setting_table), 1);
		break;
	default:
		break;
	}

	tmp = param & 0x00000F00;
	switch (tmp) {
	case DISPPARAM_CABCUI_ON:
		pr_debug("cabc on\n");
		push_table(lcm_dsi_dispparam_cabcguion_command,
			sizeof(lcm_dsi_dispparam_cabcguion_command) / sizeof(struct LCM_setting_table), 1);
		break;
	case DISPPARAM_CABCSTILL_ON:
		pr_debug("cabc still on\n");
		push_table(lcm_dsi_dispparam_cabcstillon_command,
			sizeof(lcm_dsi_dispparam_cabcstillon_command) / sizeof(struct LCM_setting_table), 1);
		break;
	case DISPPARAM_CABCMOVIE_ON:
		pr_debug("cabc movie on\n");
		push_table(lcm_dsi_dispparam_cabcmovieon_command,
			sizeof(lcm_dsi_dispparam_cabcmovieon_command) / sizeof(struct LCM_setting_table), 1);
		break;
	case DISPPARAM_CABC_OFF:
		pr_debug("cabc movie on\n");
		push_table(lcm_dsi_dispparam_cabcoff_command,
			sizeof(lcm_dsi_dispparam_cabcoff_command) / sizeof(struct LCM_setting_table), 1);
		break;
	case DISPPARAM_SKIN_CE_CABCUI_ON:
		pr_debug("cabc on\n");
		push_table(lcm_dsi_dispparam_skin_ce_cabcguion_command,
			sizeof(lcm_dsi_dispparam_skin_ce_cabcguion_command) / sizeof(struct LCM_setting_table), 1);
		break;
	case DISPPARAM_SKIN_CE_CABCSTILL_ON:
		pr_debug("cabc still on\n");
		push_table(lcm_dsi_dispparam_skin_ce_cabcstillon_command,
			sizeof(lcm_dsi_dispparam_skin_ce_cabcstillon_command) / sizeof(struct LCM_setting_table), 1);
		break;
	case DISPPARAM_SKIN_CE_CABCMOVIE_ON:
		pr_debug("cabc movie on\n");
		push_table(lcm_dsi_dispparam_skin_ce_cabcmovieon_command,
			sizeof(lcm_dsi_dispparam_skin_ce_cabcmovieon_command) / sizeof(struct LCM_setting_table), 1);
		break;
	case DISPPARAM_SKIN_CE_CABC_OFF:
		pr_debug("cabc movie on\n");
		push_table(lcm_dsi_dispparam_skin_ce_cabcoff_command,
			sizeof(lcm_dsi_dispparam_skin_ce_cabcoff_command) / sizeof(struct LCM_setting_table), 1);
		break;
	case DISPPARAM_DIMMING_OFF:
		pr_debug("dimming on\n");
		push_table(lcm_dsi_dispparam_dimmingoff_command,
			sizeof(lcm_dsi_dispparam_dimmingoff_command) / sizeof(struct LCM_setting_table), 1);
		break;
	case DISPPARAM_DIMMING:
		pr_debug("smart contrast on\n");
		push_table(lcm_dsi_dispparam_dimmingon_command,
			sizeof(lcm_dsi_dispparam_dimmingon_command) / sizeof(struct LCM_setting_table), 1);
		break;
	default:
		break;
	}

	tmp = param & 0x000F0000;
	switch (tmp) {
	case 0xA0000:
		pr_debug("idle on\n");
		push_table(lcm_dsi_dispparam_idleon_command,
			sizeof(lcm_dsi_dispparam_idleon_command) / sizeof(struct LCM_setting_table), 1);
		break;
	case DISPPARAM_HBM_ON:
		pr_debug("hbm l1 on\n");
		if(panel_type == PANEL_SUMSUNG_FT3418)
		{
		dimming_hbm_on.payload = vmalloc(sizeof(unsigned char));
		dimming_hbm_on.vc = 0;
		dimming_hbm_on.dlen = 1;
		dimming_hbm_on.link_state = 1;
		dimming_hbm_on.dtype = 0x53;
		dimming_hbm_on.payload[0] = 0xE8;
		do_lcm_vdo_lp_write_without_lock(&dimming_hbm_on,1);

		hbm_on.payload = vmalloc(2 * sizeof(unsigned char));
		hbm_on.vc = 0;
		hbm_on.dlen = 2;
		hbm_on.link_state = 1;
		hbm_on.dtype = 0x51;
		hbm_on.payload[0] = 0x07;
		hbm_on.payload[1] = 0xFF;

		do_lcm_vdo_lp_write_without_lock(&hbm_on,1);

		}
		else if(panel_type == PANEL_VISIONOX)
		{
		dimming_hbm_on.payload = vmalloc(sizeof(unsigned char));
		dimming_hbm_on.vc = 0;
		dimming_hbm_on.dlen = 1;
		dimming_hbm_on.link_state = 1;
		dimming_hbm_on.dtype = 0xFE;
		dimming_hbm_on.payload[0] = 0x00;
		do_lcm_vdo_lp_write_without_lock(&dimming_hbm_on,1);

		hbm_on.payload = vmalloc(2 * sizeof(unsigned char));
		hbm_on.vc = 0;
		hbm_on.dlen = 2;
		hbm_on.link_state = 1;
		hbm_on.dtype = 0x51;
		hbm_on.payload[0] = 0x0F;
		hbm_on.payload[1] = 0xFF;

		do_lcm_vdo_lp_write_without_lock(&hbm_on,1);
		}
		vfree(hbm_on.payload);
		vfree(dimming_hbm_on.payload);
		break;
	case DISPPARAM_HBM_OFF:
		pr_debug("hbm off\n");
		if(panel_type == PANEL_SUMSUNG_FT3418)
		{
		dimming_hbm_off.payload = vmalloc(sizeof(unsigned char));
		dimming_hbm_off.vc = 0;
		dimming_hbm_off.dlen = 1;
		dimming_hbm_off.link_state = 1;
		dimming_hbm_off.dtype = 0x53;
		*(dimming_hbm_off.payload) = 0x28;
		do_lcm_vdo_lp_write_without_lock(&dimming_hbm_off,1);

		}
		else if(panel_type == PANEL_VISIONOX)
		{
		dimming_hbm_off.payload = vmalloc(sizeof(unsigned char));
		dimming_hbm_off.vc = 0;
		dimming_hbm_off.dlen = 1;
		dimming_hbm_off.link_state = 1;
		dimming_hbm_off.dtype = 0xFE;
		*(dimming_hbm_off.payload) = 0x00;
		do_lcm_vdo_lp_write_without_lock(&dimming_hbm_off,1);

		}
		vfree(dimming_hbm_off.payload);
		break;
	default:
		break;
	}

	tmp = param & 0x00F00000;
	switch (tmp) {
	case DISPPARAM_CRC_OFF:
		pr_debug("DISPPARAM_NORMALMODE1 CRC off\n");

		for (i = 0; i < 6; i++) {
			cmd_normal_tab[i].payload = vmalloc(6 * sizeof(unsigned char));
			cmd_normal_tab[i].vc = 0;
			cmd_normal_tab[i].link_state = 1;
		}

		cmd_normal_tab[0].dtype = 0xF0;
		cmd_normal_tab[1].dtype = 0x80;
		cmd_normal_tab[2].dtype = 0xB1;
		cmd_normal_tab[3].dtype = 0xB0;
		cmd_normal_tab[4].dtype = 0xB0;
		cmd_normal_tab[5].dtype = 0xF0;

		cmd_normal_tab[0].dlen = 2;
		cmd_normal_tab[1].dlen = 1;
		cmd_normal_tab[2].dlen = 1;
		cmd_normal_tab[3].dlen = 2;
		cmd_normal_tab[4].dlen = 1;
		cmd_normal_tab[5].dlen = 2;

		(cmd_normal_tab[0].payload)[0] = 0x5A;
		(cmd_normal_tab[0].payload)[1] = 0x5A;
		*(cmd_normal_tab[1].payload) = 0x01;
		*(cmd_normal_tab[2].payload) = 0x01;
		(cmd_normal_tab[3].payload)[0] = 0x01;
		(cmd_normal_tab[3].payload)[1] = 0xB8;
		*(cmd_normal_tab[4].payload) = 0x09;
		(cmd_normal_tab[5].payload)[0] = 0xA5;
		(cmd_normal_tab[5].payload)[1] = 0xA5;

		do_lcm_vdo_lp_write_without_lock(cmd_normal_tab, 6);

		for (i = 0; i < 6; i++)
			vfree(cmd_normal_tab[i].payload);

		break;
	case DISPPARAM_P3:
		pr_debug("DISPPARAM_P3 CRC p3\n");

		for (i = 0; i < 8; i++) {
			cmd_p3_tab[i].payload = vmalloc(8 * sizeof(unsigned char));
			cmd_p3_tab[i].vc = 0;
			cmd_p3_tab[i].link_state = 1;
		}

		cmd_p3_tab[0].dtype = 0xF0;
		cmd_p3_tab[1].dtype = 0x80;
		cmd_p3_tab[2].dtype = 0xB1;
		cmd_p3_tab[3].dtype = 0xB0;
		cmd_p3_tab[4].dtype = 0xB1;
		cmd_p3_tab[5].dtype = 0xB0;
		cmd_p3_tab[6].dtype = 0xB0;
		cmd_p3_tab[7].dtype = 0xF0;

		cmd_p3_tab[0].dlen = 2;
		cmd_p3_tab[1].dlen = 1;
		cmd_p3_tab[2].dlen = 1;
		cmd_p3_tab[3].dlen = 2;
		cmd_p3_tab[4].dlen = 21;
		cmd_p3_tab[5].dlen = 2;
		cmd_p3_tab[6].dlen = 1;
		cmd_p3_tab[7].dlen = 2;

		(cmd_p3_tab[0].payload)[0] = 0x5A;
		(cmd_p3_tab[0].payload)[1] = 0x5A;
		(cmd_p3_tab[1].payload)[0] = 0x91;
		(cmd_p3_tab[2].payload)[0] = 0x00;
		(cmd_p3_tab[3].payload)[0] = 0x16;
		(cmd_p3_tab[3].payload)[1] = 0xB1;
		(cmd_p3_tab[4].payload)[0] = 0xD7;
		(cmd_p3_tab[4].payload)[1] = 0x00;
		(cmd_p3_tab[4].payload)[2] = 0x00;
		(cmd_p3_tab[4].payload)[3] = 0x0F;
		(cmd_p3_tab[4].payload)[4] = 0xC9;
		(cmd_p3_tab[4].payload)[5] = 0x02;
		(cmd_p3_tab[4].payload)[6] = 0x0A;
		(cmd_p3_tab[4].payload)[7] = 0x06;
		(cmd_p3_tab[4].payload)[8] = 0xC2;
		(cmd_p3_tab[4].payload)[9] = 0x1D;
		(cmd_p3_tab[4].payload)[10] = 0xF1;
		(cmd_p3_tab[4].payload)[11] = 0xDC;
		(cmd_p3_tab[4].payload)[12] = 0xFC;
		(cmd_p3_tab[4].payload)[13] = 0x00;
		(cmd_p3_tab[4].payload)[14] = 0xE1;
		(cmd_p3_tab[4].payload)[15] = 0xEC;
		(cmd_p3_tab[4].payload)[16] = 0xDA;
		(cmd_p3_tab[4].payload)[17] = 0x03;
		(cmd_p3_tab[4].payload)[18] = 0xFF;
		(cmd_p3_tab[4].payload)[19] = 0xFF;
		(cmd_p3_tab[4].payload)[20] = 0xFF;
		(cmd_p3_tab[5].payload)[0] = 0x01;
		(cmd_p3_tab[5].payload)[1] = 0xB8;
		(cmd_p3_tab[6].payload)[0] = 0x09;
		(cmd_p3_tab[7].payload)[0] = 0xA5;
		(cmd_p3_tab[7].payload)[1] = 0xA5;

		do_lcm_vdo_lp_write_without_lock(cmd_p3_tab, 8);

		for (i = 0; i < 8; i++)
			vfree(cmd_p3_tab[i].payload);

		break;
	case DISPPARAM_SRGB:
		pr_debug("CRC sRGB\n");

		for (i = 0; i < 8; i++) {
			cmd_srgb_tab[i].payload = vmalloc(8 * sizeof(unsigned char));
			cmd_srgb_tab[i].vc = 0;
			cmd_srgb_tab[i].link_state = 1;
		}

		cmd_srgb_tab[0].dtype = 0xF0;
		cmd_srgb_tab[1].dtype = 0x80;
		cmd_srgb_tab[2].dtype = 0xB1;
		cmd_srgb_tab[3].dtype = 0xB0;
		cmd_srgb_tab[4].dtype = 0xB1;
		cmd_srgb_tab[5].dtype = 0xB0;
		cmd_srgb_tab[6].dtype = 0xB8;
		cmd_srgb_tab[7].dtype = 0xF0;

		cmd_srgb_tab[0].dlen = 2;
		cmd_srgb_tab[1].dlen = 1;
		cmd_srgb_tab[2].dlen = 1;
		cmd_srgb_tab[3].dlen = 2;
		cmd_srgb_tab[4].dlen = 21;
		cmd_srgb_tab[5].dlen = 2;
		cmd_srgb_tab[6].dlen = 1;
		cmd_srgb_tab[7].dlen = 2;

		(cmd_srgb_tab[0].payload)[0] = 0x5A;
		(cmd_srgb_tab[0].payload)[1] = 0x5A;
		*(cmd_srgb_tab[1].payload) = 0x90;
		*(cmd_srgb_tab[2].payload) = 0x00;
		(cmd_srgb_tab[3].payload)[0] = 0x01;
		(cmd_srgb_tab[3].payload)[1] = 0xB1;
		(cmd_srgb_tab[4].payload)[0] = 0xAD;
		(cmd_srgb_tab[4].payload)[1] = 0x0F;
		(cmd_srgb_tab[4].payload)[2] = 0x04;
		(cmd_srgb_tab[4].payload)[3] = 0x3D;
		(cmd_srgb_tab[4].payload)[4] = 0xBD;
		(cmd_srgb_tab[4].payload)[5] = 0x14;
		(cmd_srgb_tab[4].payload)[6] = 0x05;
		(cmd_srgb_tab[4].payload)[7] = 0x08;
		(cmd_srgb_tab[4].payload)[8] = 0xA8;
		(cmd_srgb_tab[4].payload)[9] = 0x4B;
		(cmd_srgb_tab[4].payload)[10] = 0xDA;
		(cmd_srgb_tab[4].payload)[11] = 0xD0;
		(cmd_srgb_tab[4].payload)[12] = 0xCB;
		(cmd_srgb_tab[4].payload)[13] = 0x1D;
		(cmd_srgb_tab[4].payload)[14] = 0xCD;
		(cmd_srgb_tab[4].payload)[15] = 0xE5;
		(cmd_srgb_tab[4].payload)[16] = 0xD7;
		(cmd_srgb_tab[4].payload)[17] = 0x1B;
		(cmd_srgb_tab[4].payload)[18] = 0xFF;
		(cmd_srgb_tab[4].payload)[19] = 0xF4;
		(cmd_srgb_tab[4].payload)[20] = 0xE0;
		(cmd_srgb_tab[5].payload)[0] = 0x01;
		(cmd_srgb_tab[5].payload)[1] = 0xB8;
		*(cmd_srgb_tab[6].payload) = 0x09;
		(cmd_srgb_tab[7].payload)[0] = 0xA5;
		(cmd_srgb_tab[7].payload)[1] = 0xA5;

		do_lcm_vdo_lp_write_without_lock(cmd_srgb_tab, 8);

		for (i = 0; i < 8; i++)
			vfree(cmd_srgb_tab[i].payload);

		break;
	case DISPPARAM_DOZE_BRIGHTNESS_HBM:
		pr_debug("DISPPARAM_DOZE_BRIGHTNESS_HBM\n");
		//START tp fb suspend
		printk("-----FTS----primary_display_suspend_early_aod1");
		fb_drm_notifier_call_chain(FB_DRM_EVENT_BLANK, &g_notify_data);

		printk("-----FTS----primary_display_suspend_aod1");
		fb_drm_notifier_call_chain(FB_DRM_EARLY_EVENT_BLANK, &g_notify_data);
		doze_brightness_set.payload = vmalloc(sizeof(unsigned char));
		doze_brightness_set.vc = 0;
		doze_brightness_set.dlen = 1;
		doze_brightness_set.link_state = 1;
		doze_brightness_set.dtype = 0x51;
		doze_brightness_set.payload[0] = 0xFF;
		do_lcm_vdo_lp_write_without_lock(&doze_brightness_set,1);
		pr_debug("In %s the doze_brightness value:%x\n", __func__, DISPPARAM_DOZE_BRIGHTNESS_HBM);
		vfree(doze_brightness_set.payload);
		break;
	case DISPPARAM_DOZE_BRIGHTNESS_LBM:
		pr_debug("DISPPARAM_DOZE_BRIGHTNESS_LBM\n");
		printk("-----FTS----primary_display_suspend_aod1");
		fb_drm_notifier_call_chain(FB_DRM_EVENT_BLANK, &g_notify_data);

		printk("-----FTS----primary_display_suspend_aod1");
		fb_drm_notifier_call_chain(FB_DRM_EARLY_EVENT_BLANK, &g_notify_data);
		doze_brightness_set.payload = vmalloc(sizeof(unsigned char));
		doze_brightness_set.vc = 0;
		doze_brightness_set.dlen = 1;
		doze_brightness_set.link_state = 1;
		doze_brightness_set.dtype = 0x51;
		doze_brightness_set.payload[0] = 0x2F;
		do_lcm_vdo_lp_write_without_lock(&doze_brightness_set,1);

		pr_debug("In %s the doze_brightness value:%x\n", __func__, DISPPARAM_DOZE_BRIGHTNESS_LBM);
		vfree(doze_brightness_set.payload);
		break;
	case DISPPARAM_DOZE_OFF:
		pr_debug("DISPPARAM_DOZE_OFF\n");
		pr_debug("In %s the doze_brightness value:%x\n", __func__, DISPPARAM_DOZE_OFF);
		break;
	default:
		break;
	}

	return 0;
}

#endif
