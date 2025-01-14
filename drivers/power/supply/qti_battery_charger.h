/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2021 The Linux Foundation. All rights reserved.
 */
#ifndef __QTI_TYPEC_CLASS_H
#define __QTI_TYPEC_CLASS_H

enum charger_notifier_events {
	/* thermal board temp */
	 THERMAL_BOARD_TEMP = 0,
	 THERMAL_SCENE,
};

enum smart_chg_scene_type {
    SMART_CHG_SCENE_NORMAL              = 0,
    SMART_CHG_SCENE_HUANJI              = 1,
    SMART_CHG_SCENE_PHONE               = 5,
    SMART_CHG_SCENE_NOLIMIT             = 6,
    SMART_CHG_SCENE_CLASS0              = 7,
    SMART_CHG_SCENE_YOUTUBE             = 8,
    SMART_CHG_SCENE_NAVIGATION          = 10,
    SMART_CHG_SCENE_VIDEO               = 11,
    SMART_CHG_SCENE_VIDEOCHAT           = 14,
    SMART_CHG_SCENE_CAMERA              = 15,
    SMART_CHG_SCENE_TGAME               = 18,
    SMART_CHG_SCENE_MGAME               = 19,
    SMART_CHG_SCENE_YUANSHEN            = 20,
    SMART_CHG_SCENE_XINGTIE             = 25,
    SMART_CHG_SCENE_PER_NORMAL          = 50,
    SMART_CHG_SCENE_PER_CLASS0          = 57,
    SMART_CHG_SCENE_PER_YOUTUBE         = 58,
    SMART_CHG_SCENE_PER_VIDEO           = 61,
    SMART_CHG_SCENE_INDEX_MAX,
};

extern struct blocking_notifier_head charger_notifier;
extern int charger_reg_notifier(struct notifier_block *nb);
extern int charger_unreg_notifier(struct notifier_block *nb);
extern int charger_notifier_call_chain(unsigned long event,int val);

#endif /* __QTI_TYPEC_CLASS_H */