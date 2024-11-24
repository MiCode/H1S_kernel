// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 MediaTek Inc.
 *
 */

/****************************************************************************
 * LED DRV functions
 ***************************************************************************/
extern struct led_conf_info {
	int level;
	int led_bits;
	int trans_bits;
	int max_level;
	struct led_classdev cdev;
} led_conf_info;

int mtk_leds_register_notifier(struct notifier_block *nb);
int mtk_leds_unregister_notifier(struct notifier_block *nb);
int mt_leds_brightness_set(char *name, int bl_1024);
int mt_leds_max_brightness_set(char *name, int percent, bool enable);


extern int mtkfb_set_backlight_level(unsigned int level);
extern void disp_pq_notify_backlight_changed(int bl_1024);
extern int enable_met_backlight_tag(void);
extern int output_met_backlight_tag(int level);
extern int disp_ccorr_change_backlight(int bl_1024);

