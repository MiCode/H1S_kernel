/**
 * Copyright (C) Fourier Semiconductor Inc. 2016-2020. All rights reserved.
 * 2018-10-22 File created.
 */

#if defined(CONFIG_FSM_CODEC)
#include "fsm_public.h"
#include <linux/regmap.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <linux/miscdevice.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include <linux/version.h>

static atomic_t fsm_amp_switch;
static atomic_t fsm_amp_select;

static int fsm_get_scene_index(uint16_t scene)
{
	int index = 0;

	while (scene) {
		scene = (scene >> 1);
		if (scene == 0) {
			break;
		}
		index++;
	}

	return index;
}

//#if LINUX_VERSION_CODE < KERNEL_VERSION(3,16,0)
//static struct snd_soc_codec *snd_soc_kcontrol_codec(
//		struct snd_kcontrol *kcontrol)
//{
//	return snd_kcontrol_chip(kcontrol);
//}
//#endif

int fsm_init_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	int state;

	state = (fsm_get_presets() != NULL) ? 1 : 0;
	pr_info("state:%d", state);
	ucontrol->value.integer.value[0] = state;

	return 0;
}

int fsm_init_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	fsm_init();
	return 0;
}

int fsm_scene_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	fsm_config_t *cfg = fsm_get_config();
	int scene_index;

	if (!cfg) {
		ucontrol->value.integer.value[0] = -1;
		return 0;
	}
	scene_index = fsm_get_scene_index(cfg->next_scene);
	pr_info("scene: %04X, BIT(%d)", cfg->next_scene, scene_index);
	ucontrol->value.integer.value[0] = scene_index;

	return 0;
}

int fsm_scene_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	int next_scene = ucontrol->value.integer.value[0];

	pr_info("next_scene: %d", next_scene);
	fsm_set_scene(next_scene);

	return 0;
}

int fsm_volume_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	fsm_config_t *cfg = fsm_get_config();
	int volume;

	volume = ((cfg != NULL) ? cfg->volume : FSM_VOLUME_MAX);
	ucontrol->value.integer.value[0] = volume;
	pr_info("volume: %ld", ucontrol->value.integer.value[0]);

	return 0;
}

int fsm_volume_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	int volume = ucontrol->value.integer.value[0];

	pr_info("volume: %d", volume);
	fsm_set_volume(volume);

	return 0;
}

int fsm_amp_switch_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	int enable = atomic_read(&fsm_amp_switch);

	ucontrol->value.integer.value[0] = enable;
	pr_info("switch: %s", enable ? "On" : "Off");

	return 0;
}

int fsm_amp_switch_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	int enable = ucontrol->value.integer.value[0];

	pr_info("switch: %s", enable ? "On" : "Off");
	atomic_set(&fsm_amp_switch, enable);
	if (enable) {
		fsm_speaker_onn();
	} else {
		fsm_speaker_off();
	}

	return 0;
}

int fsm_amp_select_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	int mask = atomic_read(&fsm_amp_select);

	pr_info("MASK:%X", mask);
	ucontrol->value.integer.value[0] = mask;

	return 0;
}

int fsm_amp_select_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	int sel_mask = ucontrol->value.integer.value[0];

	pr_info("MASK:%X", sel_mask);
	atomic_set(&fsm_amp_select, sel_mask);
	fsm_set_sel_mask(sel_mask);

	return 0;
}

#ifdef CONFIG_FSM_VBAT_MONITOR
int fsm_vbat_monitor_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	int state = 0;

	fsm_vbat_monitor_state(&state);
	pr_info("State:%s", (state ? "On" : "Off"));
	ucontrol->value.integer.value[0] = state;

	return 0;
}

int fsm_vbat_monitor_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	int state = ucontrol->value.integer.value[0];

	pr_info("set %s", (state ? "On" : "Off"));
	fsm_set_vbat_monitor(!!state);

	return 0;
}
#endif

static int g_amp_scene[FSM_DEV_MAX];

int fsm_left1_scene_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct fsm_dev *fsm_dev;
	int index;

	fsm_dev = fsm_get_fsm_dev_by_position(FSM_POS_LTOP);
	if (fsm_dev == NULL)
		return -EINVAL;

	index = fsm_pos_mask_to_index(fsm_dev->pos_mask);
	ucontrol->value.integer.value[0] = g_amp_scene[index];
	return 0;
}

int fsm_left1_scene_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	int next_scene = ucontrol->value.integer.value[0];
	struct fsm_dev *fsm_dev;
	int index;
	int ret;

	pr_info("spk next_scene: %d", next_scene);
	fsm_dev = fsm_get_fsm_dev_by_position(FSM_POS_LTOP);
	if (fsm_dev == NULL)
		return -EINVAL;

	fsm_mutex_lock();
	index = fsm_pos_mask_to_index(fsm_dev->pos_mask);
	ret = fsm_stub_set_scene(fsm_dev, BIT(next_scene));
	if (!ret)
		g_amp_scene[index] = next_scene;
	fsm_mutex_unlock();

	return 0;
}

int fsm_left2_scene_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct fsm_dev *fsm_dev;
	int index;

	fsm_dev = fsm_get_fsm_dev_by_position(FSM_POS_LBTM);
	if (fsm_dev == NULL)
		return -EINVAL;

	index = fsm_pos_mask_to_index(fsm_dev->pos_mask);
	ucontrol->value.integer.value[0] = g_amp_scene[index];
	return 0;
}

int fsm_left2_scene_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	int next_scene = ucontrol->value.integer.value[0];
	struct fsm_dev *fsm_dev;
	int index;
	int ret;

	pr_info("spk next_scene: %d", next_scene);
	fsm_dev = fsm_get_fsm_dev_by_position(FSM_POS_LBTM);
	if (fsm_dev == NULL)
		return -EINVAL;

	fsm_mutex_lock();
	index = fsm_pos_mask_to_index(fsm_dev->pos_mask);
	ret = fsm_stub_set_scene(fsm_dev, BIT(next_scene));
	if (!ret)
		g_amp_scene[index] = next_scene;
	fsm_mutex_unlock();

	return 0;
}

int fsm_right1_scene_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct fsm_dev *fsm_dev;
	int index;

	fsm_dev = fsm_get_fsm_dev_by_position(FSM_POS_RTOP);
	if (fsm_dev == NULL)
		return -EINVAL;

	index = fsm_pos_mask_to_index(fsm_dev->pos_mask);
	ucontrol->value.integer.value[0] = g_amp_scene[index];
	return 0;
}

int fsm_right1_scene_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	int next_scene = ucontrol->value.integer.value[0];
	struct fsm_dev *fsm_dev;
	int index;
	int ret;

	pr_info("spk next_scene: %d", next_scene);
	fsm_dev = fsm_get_fsm_dev_by_position(FSM_POS_RTOP);
	if (fsm_dev == NULL)
		return -EINVAL;

	fsm_mutex_lock();
	index = fsm_pos_mask_to_index(fsm_dev->pos_mask);
	ret = fsm_stub_set_scene(fsm_dev, BIT(next_scene));
	if (!ret)
		g_amp_scene[index] = next_scene;
	fsm_mutex_unlock();

	return 0;
}

int fsm_right2_scene_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct fsm_dev *fsm_dev;
	int index;

	fsm_dev = fsm_get_fsm_dev_by_position(FSM_POS_RBTM);
	if (fsm_dev == NULL)
		return -EINVAL;

	index = fsm_pos_mask_to_index(fsm_dev->pos_mask);
	ucontrol->value.integer.value[0] = g_amp_scene[index];
	return 0;
}

int fsm_right2_scene_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	int next_scene = ucontrol->value.integer.value[0];
	struct fsm_dev *fsm_dev;
	int index;
	int ret;

	pr_info("spk next_scene: %d", next_scene);
	fsm_dev = fsm_get_fsm_dev_by_position(FSM_POS_RBTM);
	if (fsm_dev == NULL)
		return -EINVAL;

	fsm_mutex_lock();
	index = fsm_pos_mask_to_index(fsm_dev->pos_mask);
	ret = fsm_stub_set_scene(fsm_dev, BIT(next_scene));
	if (!ret)
		g_amp_scene[index] = next_scene;
	fsm_mutex_unlock();

	return 0;
}

static int g_spkx_switch = 0;

int fsm_spkx_switch_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = g_spkx_switch;
	return 0;
}

int fsm_spkx_switch_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	int spkx_switch = ucontrol->value.integer.value[0];
	struct fsm_dev *fsm_dev;
	bool on;
	int ret;

	pr_info("spkx switch: %d", spkx_switch);
	if (g_spkx_switch == spkx_switch)
		return 0;

	g_spkx_switch = spkx_switch;
	if (spkx_switch >= 10) {
		spkx_switch -= 10;
		on = true;
	} else {
		on = false;
	}

	if (spkx_switch >= FSM_DEV_MAX)
		return -EINVAL;

	fsm_dev = fsm_get_fsm_dev_by_position(BIT(spkx_switch));
	if (fsm_dev == NULL)
		return -EINVAL;

	fsm_mutex_lock();
	ret = fsm_stub_amp_switch(fsm_dev, on);
	fsm_mutex_unlock();

	return ret;
}

static const struct snd_kcontrol_new fsm_snd_controls[] =
{
	SOC_SINGLE_EXT("FSM_Init", SND_SOC_NOPM, 0, 1, 0,
			fsm_init_get, fsm_init_put),
	SOC_SINGLE_EXT("FSM_Scene", SND_SOC_NOPM, 0, 17, 0,
			fsm_scene_get, fsm_scene_put),
	SOC_SINGLE_EXT("FSM_Volume", SND_SOC_NOPM, 0, FSM_VOLUME_MAX, 0,
			fsm_volume_get, fsm_volume_put),
	SOC_SINGLE_EXT("FSM_Amp_Switch", SND_SOC_NOPM, 0, 1, 0,
			fsm_amp_switch_get, fsm_amp_switch_put),
	SOC_SINGLE_EXT("FSM_Amp_Select", SND_SOC_NOPM, 0, 0xF, 0,
			fsm_amp_select_get, fsm_amp_select_put),
#ifdef CONFIG_FSM_VBAT_MONITOR
	SOC_SINGLE_EXT("FSM_Vbat_Monitor", SND_SOC_NOPM, 0, 1, 0,
			fsm_vbat_monitor_get, fsm_vbat_monitor_put),
#endif
	SOC_SINGLE_EXT("FSM_SpkL1_Scene", SND_SOC_NOPM, 0, FSM_SCENE_MAX, 0,
			fsm_left1_scene_get, fsm_left1_scene_put),
	SOC_SINGLE_EXT("FSM_SpkR1_Scene", SND_SOC_NOPM, 0, FSM_SCENE_MAX, 0,
			fsm_right1_scene_get, fsm_right1_scene_put),
	SOC_SINGLE_EXT("FSM_SpkL2_Scene", SND_SOC_NOPM, 0, FSM_SCENE_MAX, 0,
			fsm_left2_scene_get, fsm_left2_scene_put),
	SOC_SINGLE_EXT("FSM_SpkR2_Scene", SND_SOC_NOPM, 0, FSM_SCENE_MAX, 0,
			fsm_right2_scene_get, fsm_right2_scene_put),
	SOC_SINGLE_EXT("FSM_SpkX_Switch", SND_SOC_NOPM, 0, 14, 0,
			fsm_spkx_switch_get, fsm_spkx_switch_put),
};

void fsm_add_codec_controls(struct snd_soc_codec *codec)
{
	atomic_set(&fsm_amp_switch, 0);
	atomic_set(&fsm_amp_select, 0xF);
	snd_soc_add_codec_controls(codec, fsm_snd_controls,
			ARRAY_SIZE(fsm_snd_controls));
}
EXPORT_SYMBOL(fsm_add_codec_controls);

void fsm_add_card_controls(struct snd_soc_card *card)
{
	atomic_set(&fsm_amp_switch, 0);
	atomic_set(&fsm_amp_select, 0xF);
	snd_soc_add_card_controls(card, fsm_snd_controls,
			ARRAY_SIZE(fsm_snd_controls));
}
EXPORT_SYMBOL(fsm_add_card_controls);

#endif
