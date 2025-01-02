// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#include <linux/backlight.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <drm/drm_modes.h>
#include <linux/delay.h>
#include <drm/drm_connector.h>
#include <drm/drm_device.h>

#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#include <linux/kconfig.h>
#include <linux/of_gpio.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>

#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../mediatek/mediatek_v2/mtk_panel_ext.h"
#include "../mediatek/mediatek_v2/mtk_log.h"
#include "../mediatek/mediatek_v2/mtk_drm_graphics_base.h"
#endif
extern unsigned int screen_state;
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
#include "../mediatek/mediatek_v2/mtk_corner_pattern/mtk_data_hw_roundedpattern.h"
#endif
#include "../mediatek/mediatek_v2/mi_disp/mi_panel_ext.h"
#include "../mediatek/mediatek_v2/mi_disp/mi_dsi_panel.h"
#include "../mediatek/mediatek_v2/mtk_disp_notify.h"
#define MTE_OFF (0xFFFF)
#define CMD_NUM_MAX 24
#define HBM_MAP_MAX_BRIGHTNESS      4095
#define NORMAL_MAX_BRIGHTNESS       2047

#define REGFLAG_DELAY       0xFFFC
#define REGFLAG_UDELAY  0xFFFB
#define REGFLAG_END_OF_TABLE    0xFFFD
static int curr_bl1;
static int curr_bl2;

struct lcm {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *vci_gpio;
	bool prepared;
	bool enabled;
	struct mutex panel_lock;
	struct mutex lhbm_lock;
	struct mutex flm_lock;
	int pmode_id;
	struct list_head probed_modes;
	const char *panel_info;
	int error;

	enum doze_brightness_state doze_brightness;
};

static const char *panel_name = "panel_name=dsi_panel_o17_38_0c_0a_fhdp_vdo";
static struct mtk_ddic_dsi_msg *cmd_msg = NULL;
static struct drm_panel *this_panel = NULL;
static bool hbm_on = false;
#define lcm_dcs_write_seq(ctx, seq...) \
({\
	const u8 d[] = { seq };\
	BUILD_BUG_ON_MSG(ARRAY_SIZE(d) > 64, "DCS sequence too big for stack");\
	lcm_dcs_write(ctx, d, ARRAY_SIZE(d));\
})

#define lcm_dcs_write_seq_static(ctx, seq...) \
({\
	static const u8 d[] = { seq };\
	lcm_dcs_write(ctx, d, ARRAY_SIZE(d));\
})

static struct LCM_setting_table flat_mode_off[] = {
	{0xF0, 02, {0x5A, 0x5A}},
	{0xB0, 03, {0x00, 0x89, 0xB1}},
	{0xB1, 01, {0x0D}},
	{0xB0, 03, {0x00, 0x8E, 0xB1}},
	{0xB1, 01, {0x0F}},
	{0xB0, 03, {0x00, 0x86, 0xB1}},
	{0xB1, 01, {0x00}},
	{0xB0, 03, {0x00, 0x04, 0xB8}},
	{0xB8, 01, {0x8B}},
	{0xB0, 03, {0x00, 0x07, 0xB8}},
	{0xB8, 01, {0x8B}},
	{0xF0, 02, {0xA5, 0xA5}},
};

static struct LCM_setting_table flat_mode_on[] = {
	{0xF0, 02, {0x5A, 0x5A}},
	{0xB0, 03, {0x00, 0x89, 0xB1}},
	{0xB1, 01, {0x2D}},
	{0xB0, 03, {0x00, 0x8E, 0xB1}},
	{0xB1, 01, {0x0F}},
	{0xB0, 03, {0x00, 0x86, 0xB1}},
	{0xB1, 01, {0x02}},
	{0xB0, 03, {0x00, 0x04, 0xB8}},
	{0xB8, 01, {0x8F}},
	{0xB0, 03, {0x00, 0x07, 0xB8}},
	{0xB8, 01, {0x8F}},
	{0xF0, 02, {0xA5, 0xA5}},
};

static struct LCM_setting_table normal_hbm_on[] = {
	{0x53, 01, {0xE8}},
	{0x51, 02, {0x07, 0xFF}},
	{0xF7, 01, {0x0B}},
};

static struct LCM_setting_table normal_hbm_off[] = {
	{0x53, 01, {0x28}},
	{0xF7, 01, {0x0B}},
};

static struct LCM_setting_table normal_hbm_off_7FF[] = {
	{0x53, 01, {0x28}},
	{0x51, 02, {0x07, 0xFF}},
	{0xF7, 01, {0x0B}},
};

static struct LCM_setting_table normal_hbm_on_001[] = {
	{0x53, 01, {0xE8}},
	{0x51, 02, {0x00, 0x01}},
	{0xF7, 01, {0x0B}},
};

static struct LCM_setting_table local_hbm_normal_white_1200nit[] = {
	{0xF0, 02, {0x5A, 0x5A}},
	{0x60, 01, {0x01}},
	{0x53, 01, {0x24}},
	{0xB0, 03, {0x01, 0x53, 0xB2}},
	{0xB2, 14, {0x00, 0x04, 0x7F, 0xF7, 0xFF, 0x7F, 0xF7, 0xFF, 0x7F, 0xF7, 0xFF, 0x7F, 0xFF, 0xFF}},
	{0xB0, 03, {0x00, 0xB5, 0xB4}},
	{0xB4, 03, {0x01, 0x00, 0x14}},
	{0xB0, 03, {0x01, 0x08, 0xB5}},
	{0xB5, 15, {0x02, 0x00, 0x00, 0x00, 0x00, 0x1B, 0x87, 0xD9, 0x28, 0x08, 0xA1, 0x21, 0xC8, 0x3D, 0x5D}},
	{0xB0, 03, {0x01, 0x4E, 0xB2}},
	{0xB2, 05, {0x03, 0x33, 0xAF, 0x3F, 0xFF}},
	{0xB0, 03, {0x00, 0x19, 0xF4}},
	{0xF4, 01, {0x1C}},
	{0xB0, 03, {0x01, 0x17, 0xB2}},
	{0xB2, 01, {0x01}},
	{0xB0, 03, {0x00, 0xBD, 0xB4}},
	{0xB4, 02, {0x03, 0xFF}},
	{0xB0, 03, {0x00, 0x36, 0xB4}},
	{0xB4, 12, {0x00, 0x40, 0x14, 0x02, 0x80, 0x52, 0x0A, 0x41, 0x48, 0x7F, 0xFF, 0xFF}},
	{0xF7, 01, {0x0B}},
};

static struct LCM_setting_table local_hbm_normal_white_200nit[] = {
	{0xF0, 02, {0x5A, 0x5A}},
	{0x60, 01, {0x01}},
	{0x53, 01, {0x24}},
	{0xB0, 03, {0x01, 0x53, 0xB2}},
	{0xB2, 14, {0x00, 0x04, 0x7F, 0xF7, 0xFF, 0x7F, 0xF7, 0xFF, 0x7F, 0xF7, 0xFF, 0x7F, 0xFF, 0xFF}},
	{0xB0, 03, {0x00, 0xB5, 0xB4}},
	{0xB4, 03, {0x01, 0x00, 0x14}},
	{0xB0, 03, {0x01, 0x08, 0xB5}},
	{0xB5, 15, {0x02, 0x00, 0x00, 0x00, 0x00, 0x1B, 0x87, 0xD9, 0x28, 0x08, 0xA1, 0x21, 0xC8, 0x3D, 0x5D}},
	{0xB0, 03, {0x00, 0x19, 0xF4}},
	{0xF4, 01, {0x1F}},
	{0xB0, 03, {0x01, 0x17, 0xB2}},
	{0xB2, 50, {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xF0, 0xF0, 0x39, 0x00, 0x00, 0x00, 0x00, 0x15, 0x1D, 0x12, 0x61, 0x15, 0x38, 0x24, 0x7D, 0x15, 0x82, 0x5E, 0xC8, 0x16, 0xB9, 0x8C, 0x00, 0x16, 0xF1, 0xC0, 0x3E, 0x2A, 0x52, 0x16, 0xA3, 0x2B, 0xD1, 0x87, 0x2C, 0x3B, 0x27, 0xD4, 0x8D, 0x3F, 0x7A, 0x1D, 0xE9}},
	{0xB0, 03, {0x00, 0x69, 0xB2}},
	{0xB2, 07, {0x03, 0x03, 0x03, 0x03, 0x03, 0x02, 0x01}},
	{0xB0, 03, {0x00, 0x19, 0xB2}},
	{0xB2, 11, {0x0E, 0x41, 0xD4, 0x28, 0x34, 0x92, 0x5F, 0x17, 0xC4, 0x7F, 0xF8}},
	{0xB0, 03, {0x00, 0xBD, 0xB4}},
	{0xB4, 02, {0x03, 0xFF}},
	{0xB0, 03, {0x01, 0x4E, 0xB2}},
	{0xB2, 05, {0x03, 0x33, 0x5F, 0x00, 0xBF}},
	{0xB0, 03, {0x00, 0x36, 0xB4}},
	{0xB4, 12, {0x00, 0x40, 0x14, 0x02, 0x80, 0x52, 0x0A, 0x41, 0x48, 0x7F, 0xFF, 0xFF}},
	{0xF7, 01, {0x0B}},
};

static struct LCM_setting_table local_hbm_normal_green_500nit[] = {
	{0xF0, 02, {0x5A, 0x5A}},
	{0x60, 01, {0x01}},
	{0x53, 01, {0x24}},
	{0xB0, 03, {0x01, 0x53, 0xB2}},
	{0xB2, 14, {0x00, 0x04, 0x7F, 0xF7, 0xFF, 0x7F, 0xF7, 0xFF, 0x7F, 0xF7, 0xFF, 0x7F, 0xFF, 0xFF}},
	{0xB0, 03, {0x00, 0xB5, 0xB4}},
	{0xB4, 03, {0x01, 0x00, 0x14}},
	{0xB0, 03, {0x01, 0x08, 0xB5}},
	{0xB5, 15, {0x02, 0x33, 0xFF, 0x00, 0xFF, 0x1B, 0x87, 0xD9, 0x28, 0x08, 0xA1, 0x21, 0xC8, 0x3D, 0x5D}},
	{0xB0, 03, {0x01, 0x4E, 0xB2}},
	{0xB2, 05, {0x03, 0x33, 0x00, 0x01, 0x00}},
	{0xB0, 03, {0x00, 0x19, 0xF4}},
	{0xF4, 01, {0x1C}},
	{0xB0, 03, {0x01, 0x17, 0xB2}},
	{0xB2, 01, {0x01}},
	{0xB0, 03, {0x00, 0xBD, 0xB4}},
	{0xB4, 02, {0x03, 0xFF}},
	{0xB0, 03, {0x00, 0x36, 0xB4}},
	{0xB4, 12, {0x00, 0x40, 0x14, 0x02, 0x80, 0x52, 0x0A, 0x41, 0x48, 0x7F, 0xFF, 0xFF}},
	{0xF7, 01, {0x0B}},
};

static struct LCM_setting_table local_hbm_normal_off[] = {
	{0xF0, 02, {0x5A, 0x5A}},
	{0x53, 01, {0x28}},
	{0xB0, 03, {0x01, 0x17, 0xB2}},
	{0xB2, 01, {0x00}},
	{0xB0, 03, {0x00, 0x19, 0xF4}},
	{0xF4, 01, {0x0A}},
	{0xB0, 03, {0x01, 0x53, 0xB2}},
	{0xB2, 14, {0x01, 0xC2, 0x00, 0x40, 0x14, 0x02, 0x80, 0x52, 0x0A, 0x41, 0x48, 0x7F, 0xFF, 0xFF}},
	{0xB0, 03, {0x00, 0xB5, 0xB4}},
	{0xB4, 03, {0x00, 0x00, 0x14}},
	{0xF7, 01, {0x0B}},
};

/*sensor add level callback*/
static struct blocking_notifier_head backlight_level_nb;

int backlight_level_register_notifier(struct notifier_block *nb)
{
        pr_err("%s, backlight_level_register_notifier\n", __func__);
        return blocking_notifier_chain_register(&backlight_level_nb, nb);
}
EXPORT_SYMBOL(backlight_level_register_notifier);

int backlight_level_unregister_notifier(struct notifier_block *nb)
{
        pr_err("%s, backlight_level_unregister_notifier\n", __func__);
        return blocking_notifier_chain_unregister(&backlight_level_nb, nb);
}
EXPORT_SYMBOL(backlight_level_unregister_notifier);

int backlight_level_notifier_call_chain(unsigned long val, void *v)
{
        pr_err("%s,backlight_level_notifier_call_chain\n", __func__);
        return blocking_notifier_call_chain(&backlight_level_nb, val, v);
}
EXPORT_SYMBOL(backlight_level_notifier_call_chain);
/*sensor add level callback*/

static ssize_t screen_states_show(struct device *device,
			    struct device_attribute *attr,
			   char *buf)
{
	pr_info("%s, start \n", __func__);
	return snprintf(buf, PAGE_SIZE, "%d\n", screen_state);

}

static ssize_t screen_states_store(struct device *device,
			   struct device_attribute *attr,
			   const char *buf, size_t count)
{
	int value;
	int data;
	pr_info("%s, start \n", __func__);
	if(kstrtoint(buf, 10, &value))
          return -EINVAL;
	screen_state = value;
	if(screen_state == 3)
		data = MTK_DISP_BLANK_LP1;
	else if(screen_state == 2)
		data = MTK_DISP_BLANK_UNBLANK;
	else if(screen_state == 1)
		data = MTK_DISP_BLANK_POWERDOWN;
	mtk_disp_notifier_call_chain(MTK_DISP_EARLY_EVENT_BLANK,
					&data);
	mtk_disp_notifier_call_chain(MTK_DISP_EVENT_BLANK,
					&data);
	pr_info("%s,[lcd_notifier] data=%u \n", __func__, data);
	return count;
}

static DEVICE_ATTR_RW(screen_states);

static struct attribute *o17_attrs[] = {
        &dev_attr_screen_states.attr,
        NULL,
};

static const struct attribute_group o17_attr_group = {
        .attrs = o17_attrs,
};

static inline struct lcm *panel_to_lcm(struct drm_panel *panel)
{
	return container_of(panel, struct lcm, panel);
}

static void lcm_dcs_write(struct lcm *ctx, const void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;
	char *addr;

	if (ctx->error < 0)
		return;

	addr = (char *)data;
	if ((int)*addr < 0xB0)
		ret = mipi_dsi_dcs_write_buffer(dsi, data, len);
	else
		ret = mipi_dsi_generic_write(dsi, data, len);
	if (ret < 0) {
		dev_info(ctx->dev, "error %zd writing seq: %ph\n", ret, data);
		ctx->error = ret;
	}
}

int panel_ddic_send_cmd(struct LCM_setting_table *table,
	unsigned int count, bool block)
{
	int i = 0, j = 0;
	int ret = 0;
	unsigned char temp[25][255] = {0};
	unsigned char cmd = {0};
	struct mtk_ddic_dsi_msg cmd_msg = {
		.channel = 0,
		.flags = 0,//MIPI_DSI_MSG_USE_LPM
		.tx_cmd_num = count,
	};
	if (table == NULL) {
		pr_err("invalid ddic cmd \n");
		return ret;
	}
	if (count == 0 || count > CMD_NUM_MAX) {
		pr_err("cmd count invalid, value:%d \n", count);
		return ret;
	}
	for (i = 0;i < count; i++) {
		memset(temp[i], 0, sizeof(temp[i]));
		/* LCM_setting_table format: {cmd, count, {para_list[]}} */
		cmd = (u8)table[i].cmd;
		temp[i][0] = cmd;
		for (j = 0; j < table[i].count; j++) {
			temp[i][j+1] = table[i].para_list[j];
		}
		cmd_msg.type[i] = table[i].count > 1 ? 0x39 : 0x15;
		cmd_msg.tx_buf[i] = temp[i];
		cmd_msg.tx_len[i] = table[i].count + 1;
	}
	ret = mtk_ddic_dsi_send_cmd(&cmd_msg, block, false);
	if (ret != 0) {
		pr_err("%s: failed to send ddic cmd\n", __func__);
	}
	return ret;
}
#ifdef PANEL_SUPPORT_READBACK
static int lcm_dcs_read(struct lcm *ctx, u8 cmd, void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;

	if (ctx->error < 0)
		return 0;

	ret = mipi_dsi_dcs_read(dsi, cmd, data, len);
	if (ret < 0) {
		dev_info(ctx->dev, "error %d reading dcs seq:(%#x)\n", ret, cmd);
		ctx->error = ret;
	}

	return ret;
}

static void lcm_panel_get_data(struct lcm *ctx)
{
	u8 buffer[3] = {0};
	static int ret;

	if (ret == 0) {
		ret = lcm_dcs_read(ctx,  0x0A, buffer, 1);
		dev_info(ctx->dev, "return %d data(0x%08x) to dsi engine\n",
			 ret, buffer[0] | (buffer[1] << 8));
	}
}
#endif

static void lcm_panel_fhd120_init(struct lcm *ctx)
{
	lcm_dcs_write_seq_static(ctx, 0x11);
	msleep(32);
	/*FFC setting*/
	lcm_dcs_write_seq_static(ctx, 0xFC, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0x14, 0xDF);
	lcm_dcs_write_seq_static(ctx, 0xDF, 0x01);
	lcm_dcs_write_seq_static(ctx, 0xDF, 0x09, 0x30, 0x95, 0x46, 0xE9, 0x46, 0xE9);
	lcm_dcs_write_seq_static(ctx, 0xFC, 0xA5, 0xA5);
	/*11bit Dimming*/
	lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx, 0xB2, 0x01, 0x31);
	lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);
	/*DSC*/
	lcm_dcs_write_seq_static(ctx, 0x9F, 0xA5, 0xA5);
	lcm_dcs_write_seq_static(ctx, 0x9D, 0x01);
	lcm_dcs_write_seq_static(ctx, 0x9E, 0x11,0x00,0x00,0x89,0x30,0x80,0x09,0x60,
			0x04,0x38,0x00,0x28,0x02,0x1C,0x02,0x1C,
			0x02,0x00,0x02,0x0E,0x00,0x20,0x03,0xDD,
			0x00,0x07,0x00,0x0C,0x02,0x77,0x02,0x8B,
			0x18,0x00,0x10,0xF0,0x03,0x0C,0x20,0x00,
			0x06,0x0B,0x0B,0x33,0x0E,0x1C,0x2A,0x38,
			0x46,0x54,0x62,0x69,0x70,0x77,0x79,0x7B,
			0x7D,0x7E,0x01,0x02,0x01,0x00,0x09,0x40,
			0x09,0xBE,0x19,0xFC,0x19,0xFA,0x19,0xF8,
			0x1A,0x38,0x1A,0x78,0x1A,0xB6,0x2A,0xF6,
			0x2B,0x34,0x2B,0x74,0x3B,0x74,0x6B,0xF4,
			0x00);
	lcm_dcs_write_seq_static(ctx, 0x9F, 0x5A, 0x5A);
	/*brightness control*/
	lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx, 0x60, 0x01);
	lcm_dcs_write_seq_static(ctx, 0xF7, 0x0B);
	lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);
	/*Steal Light Repair Code*/
	lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0x40, 0xF2);
	lcm_dcs_write_seq_static(ctx, 0xF2, 0x03);
	lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);
	/*vint voltage control*/
	lcm_dcs_write_seq_static(ctx, 0xFC, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0x11, 0xFE);
	lcm_dcs_write_seq_static(ctx, 0xFE, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xFC, 0xA5, 0xA5);
	/*dimming setting*/
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0x0D, 0xB2);
	lcm_dcs_write_seq_static(ctx, 0xB2, 0x08);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0x0C, 0xB2);
	lcm_dcs_write_seq_static(ctx, 0xB2, 0x20);
	lcm_dcs_write_seq_static(ctx, 0xF7, 0x0B);
	lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);
	/*flat mode on*/
	lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0x89, 0xB1);
	lcm_dcs_write_seq_static(ctx, 0xB1, 0x2D);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0x8E, 0xB1);
	lcm_dcs_write_seq_static(ctx, 0xB1, 0x0F);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0x86, 0xB1);
	lcm_dcs_write_seq_static(ctx, 0xB1, 0x02);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0x04, 0xB8);
	lcm_dcs_write_seq_static(ctx, 0xB8, 0x8F);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0x07, 0xB8);
	lcm_dcs_write_seq_static(ctx, 0xB8, 0x8F);
	lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);
	/*LTPS setting*/
	lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0x1F, 0xC3);
	lcm_dcs_write_seq_static(ctx, 0xC3, 0x2C, 0x26);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0x24, 0xC3);
	lcm_dcs_write_seq_static(ctx, 0xC3, 0x17, 0x23, 0x22);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0x33, 0xC3);
	lcm_dcs_write_seq_static(ctx, 0xC3, 0x1B);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0x6F, 0xC3);
	lcm_dcs_write_seq_static(ctx, 0xC3, 0x38, 0x15);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0x74, 0xC3);
	lcm_dcs_write_seq_static(ctx, 0xC3, 0x1E, 0x24, 0x1E);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0x83, 0xC3);
	lcm_dcs_write_seq_static(ctx, 0xC3, 0x3F);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0xA3, 0xC3);
	lcm_dcs_write_seq_static(ctx, 0xC3, 0x2C, 0x26);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0xA8, 0xC3);
	lcm_dcs_write_seq_static(ctx, 0xC3, 0x17, 0x23, 0x22);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0xB7, 0xC3);
	lcm_dcs_write_seq_static(ctx, 0xC3, 0x1B);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0xF3, 0xC3);
	lcm_dcs_write_seq_static(ctx, 0xC3, 0x1D, 0x1D, 0x1D, 0x1D);
	lcm_dcs_write_seq_static(ctx, 0xF7, 0x0B);
	lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);
	/*Mux setting*/
	lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0x0C, 0xF6);
	lcm_dcs_write_seq_static(ctx, 0xF6, 0x45, 0x20);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0x1F, 0xF6);
	lcm_dcs_write_seq_static(ctx, 0xF6, 0x41, 0x20);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0x32, 0xF6);
	lcm_dcs_write_seq_static(ctx, 0xF6, 0x45, 0x20);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0x2C, 0xF6);
	lcm_dcs_write_seq_static(ctx, 0xF6, 0x00, 0x00, 0x00, 0x48, 0x4A);
	lcm_dcs_write_seq_static(ctx, 0xF7, 0x0B);
	lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);
	/*err fg setting*/
	lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx, 0xFC, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx, 0xED, 0x01, 0xCD, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xE1, 0x93);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0x06, 0xF4);
	lcm_dcs_write_seq_static(ctx, 0xF4, 0x1F);
	lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);
	lcm_dcs_write_seq_static(ctx, 0xFC, 0xA5, 0xA5);

	lcm_dcs_write_seq_static(ctx, 0x53, 0x28);
	lcm_dcs_write_seq_static(ctx, 0x51, 0x00, 0x00);
	msleep(100);
	lcm_dcs_write_seq_static(ctx, 0x9F, 0xA5, 0xA5);
	lcm_dcs_write_seq_static(ctx, 0x29);
}

static void lcm_panel_fhd60_init(struct lcm *ctx)
{
	lcm_dcs_write_seq_static(ctx, 0x11);
	msleep(32);
	/*FFC setting*/
	lcm_dcs_write_seq_static(ctx, 0xFC, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0x14, 0xDF);
	lcm_dcs_write_seq_static(ctx, 0xDF, 0x01);
	lcm_dcs_write_seq_static(ctx, 0xDF, 0x09, 0x30, 0x95, 0x46, 0xE9, 0x46, 0xE9);
	lcm_dcs_write_seq_static(ctx, 0xFC, 0xA5, 0xA5);
	/*11bit Dimming*/
	lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx, 0xB2, 0x01, 0x31);
	lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);
	/*DSC*/
	lcm_dcs_write_seq_static(ctx, 0x9F, 0xA5, 0xA5);
	lcm_dcs_write_seq_static(ctx, 0x9D, 0x01);
	lcm_dcs_write_seq_static(ctx, 0x9E, 0x11,0x00,0x00,0x89,0x30,0x80,0x09,0x60,
			0x04,0x38,0x00,0x28,0x02,0x1C,0x02,0x1C,
			0x02,0x00,0x02,0x0E,0x00,0x20,0x03,0xDD,
			0x00,0x07,0x00,0x0C,0x02,0x77,0x02,0x8B,
			0x18,0x00,0x10,0xF0,0x03,0x0C,0x20,0x00,
			0x06,0x0B,0x0B,0x33,0x0E,0x1C,0x2A,0x38,
			0x46,0x54,0x62,0x69,0x70,0x77,0x79,0x7B,
			0x7D,0x7E,0x01,0x02,0x01,0x00,0x09,0x40,
			0x09,0xBE,0x19,0xFC,0x19,0xFA,0x19,0xF8,
			0x1A,0x38,0x1A,0x78,0x1A,0xB6,0x2A,0xF6,
			0x2B,0x34,0x2B,0x74,0x3B,0x74,0x6B,0xF4,
			0x00);
	lcm_dcs_write_seq_static(ctx, 0x9F, 0x5A, 0x5A);
	/*brightness control*/
	lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx, 0x60, 0x21);
	lcm_dcs_write_seq_static(ctx, 0xF7, 0x0B);
	lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);
	/*Steal Light Repair Code*/
	lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0x40, 0xF2);
	lcm_dcs_write_seq_static(ctx, 0xF2, 0x03);
	lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);
	/*vint voltage control*/
	lcm_dcs_write_seq_static(ctx, 0xFC, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0x11, 0xFE);
	lcm_dcs_write_seq_static(ctx, 0xFE, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xFC, 0xA5, 0xA5);
	/*dimming setting*/
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0x0D, 0xB2);
	lcm_dcs_write_seq_static(ctx, 0xB2, 0x08);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0x0C, 0xB2);
	lcm_dcs_write_seq_static(ctx, 0xB2, 0x20);
	lcm_dcs_write_seq_static(ctx, 0xF7, 0x0B);
	lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);
	/*flat mode on*/
	lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0x89, 0xB1);
	lcm_dcs_write_seq_static(ctx, 0xB1, 0x2D);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0x8E, 0xB1);
	lcm_dcs_write_seq_static(ctx, 0xB1, 0x0F);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0x86, 0xB1);
	lcm_dcs_write_seq_static(ctx, 0xB1, 0x02);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0x04, 0xB8);
	lcm_dcs_write_seq_static(ctx, 0xB8, 0x8F);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0x07, 0xB8);
	lcm_dcs_write_seq_static(ctx, 0xB8, 0x8F);
	lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);
	/*LTPS setting*/
	lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0x1F, 0xC3);
	lcm_dcs_write_seq_static(ctx, 0xC3, 0x2C, 0x26);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0x24, 0xC3);
	lcm_dcs_write_seq_static(ctx, 0xC3, 0x17, 0x23, 0x22);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0x33, 0xC3);
	lcm_dcs_write_seq_static(ctx, 0xC3, 0x1B);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0x6F, 0xC3);
	lcm_dcs_write_seq_static(ctx, 0xC3, 0x38, 0x15);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0x74, 0xC3);
	lcm_dcs_write_seq_static(ctx, 0xC3, 0x1E, 0x24, 0x1E);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0x83, 0xC3);
	lcm_dcs_write_seq_static(ctx, 0xC3, 0x3F);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0xA3, 0xC3);
	lcm_dcs_write_seq_static(ctx, 0xC3, 0x2C, 0x26);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0xA8, 0xC3);
	lcm_dcs_write_seq_static(ctx, 0xC3, 0x17, 0x23, 0x22);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0xB7, 0xC3);
	lcm_dcs_write_seq_static(ctx, 0xC3, 0x1B);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0xF3, 0xC3);
	lcm_dcs_write_seq_static(ctx, 0xC3, 0x1D, 0x1D, 0x1D, 0x1D);
	lcm_dcs_write_seq_static(ctx, 0xF7, 0x0B);
	lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);
	/*Mux setting*/
	lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0x0C, 0xF6);
	lcm_dcs_write_seq_static(ctx, 0xF6, 0x45, 0x20);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0x1F, 0xF6);
	lcm_dcs_write_seq_static(ctx, 0xF6, 0x41, 0x20);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0x32, 0xF6);
	lcm_dcs_write_seq_static(ctx, 0xF6, 0x45, 0x20);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0x2C, 0xF6);
	lcm_dcs_write_seq_static(ctx, 0xF6, 0x00, 0x00, 0x00, 0x48, 0x4A);
	lcm_dcs_write_seq_static(ctx, 0xF7, 0x0B);
	lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);
	/*err fg setting*/
	lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx, 0xFC, 0x5A, 0x5A);
	lcm_dcs_write_seq_static(ctx, 0xED, 0x01, 0xCD, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xE1, 0x93);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0x06, 0xF4);
	lcm_dcs_write_seq_static(ctx, 0xF4, 0x1F);
	lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);
	lcm_dcs_write_seq_static(ctx, 0xFC, 0xA5, 0xA5);

	lcm_dcs_write_seq_static(ctx, 0x53, 0x28);
	lcm_dcs_write_seq_static(ctx, 0x51, 0x00, 0x00);
	msleep(100);
	lcm_dcs_write_seq_static(ctx, 0x9F, 0xA5, 0xA5);
	lcm_dcs_write_seq_static(ctx, 0x29);
}

static void lcm_panel_init(struct lcm *ctx)
{
	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_info(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return;
	}
	/*gpiod_set_value(ctx->reset_gpio, 0);*/
	/*udelay(10 * 1000);*/
	gpiod_set_value(ctx->reset_gpio, 1);
	msleep(10);
	gpiod_set_value(ctx->reset_gpio, 0);
	msleep(10);
	gpiod_set_value(ctx->reset_gpio, 1);
	msleep(20);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	DDPMSG("%s++ pmode_id:%d\n", __func__, ctx->pmode_id);

	switch (ctx->pmode_id) {
	case 0:
		lcm_panel_fhd60_init(ctx);
		break;
	case 1:
		lcm_panel_fhd120_init(ctx);
		break;
	default:
		break;
	}
}

static void push_table(struct lcm *ctx, struct LCM_setting_table *table, unsigned int count)
{
	unsigned int i, j;
	unsigned char temp[255] = {0};
	for (i = 0; i < count; i++) {
		unsigned int cmd;
		cmd = table[i].cmd;
		memset(temp, 0, sizeof(temp));
		switch (cmd) {
		case REGFLAG_DELAY:
			msleep(table[i].count);
			break;
		case REGFLAG_END_OF_TABLE:
			break;
		default:
			temp[0] = cmd;
			for (j = 0; j < table[i].count; j++)
				temp[j+1] = table[i].para_list[j];
			lcm_dcs_write(ctx, temp, table[i].count+1);
		}
	}
}

static int lcm_disable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	char disable_set1[] = {0x9F, 0xA5, 0xA5};
	char disable_set2[] = {0x28};
	char disable_set3[] = {0x10};

	pr_info("%s\n", __func__);
	if (!ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(ctx->backlight);
	}

	cmd_msg->channel = 0;
	cmd_msg->flags = 0;
	cmd_msg->tx_cmd_num = 1;

	cmd_msg->type[0] = 0x39;
	cmd_msg->tx_buf[0] = disable_set1;
	cmd_msg->tx_len[0] = 3;
	mtk_ddic_dsi_send_cmd_no_lock(cmd_msg, false, false);
	cmd_msg->type[0] = 0x05;
	cmd_msg->tx_buf[0] = disable_set2;
	cmd_msg->tx_len[0] = 1;
	mtk_ddic_dsi_send_cmd_no_lock(cmd_msg, false, false);
	msleep(20);
	cmd_msg->type[0] = 0x05;
	cmd_msg->tx_buf[0] = disable_set3;
	cmd_msg->tx_len[0] = 1;
	mtk_ddic_dsi_send_cmd_no_lock(cmd_msg, false, false);
	msleep(150);

	ctx->enabled = false;
	pr_info("%s end\n", __func__);
	return 0;
}

static int lcm_unprepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	pr_info("%s\n", __func__);
	if (!ctx->prepared)
		return 0;

	ctx->error = 0;
	ctx->prepared = false;

	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_info(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	gpiod_set_value(ctx->reset_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
/*	ctx->vddi_gpio =
		devm_gpiod_get(ctx->dev, "vddi", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vddi_gpio)) {
		dev_err(ctx->dev, "%s: cannot get vddi_gpio %ld\n",
			__func__, PTR_ERR(ctx->vddi_gpio));
		return PTR_ERR(ctx->vddi_gpio);
	}
	gpiod_set_value(ctx->vddi_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->vddi_gpio);*/

	msleep(20);

	ctx->vci_gpio =
		devm_gpiod_get(ctx->dev, "vci", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vci_gpio)) {
		dev_err(ctx->dev, "%s: cannot get vci_gpio %ld\n",
			__func__, PTR_ERR(ctx->vci_gpio));
		return PTR_ERR(ctx->vci_gpio);
	}
	gpiod_set_value(ctx->vci_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->vci_gpio);
	udelay(5000);

	return 0;
}

static int lcm_prepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	pr_info("%s\n", __func__);
	if (ctx->prepared)
		return 0;

/*	ctx->vddi_gpio =
		devm_gpiod_get(ctx->dev, "vddi", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vddi_gpio)) {
		dev_err(ctx->dev, "%s: cannot get vddi_gpio %ld\n",
			__func__, PTR_ERR(ctx->vddi_gpio));
		return PTR_ERR(ctx->vddi_gpio);
	}
	gpiod_set_value(ctx->vddi_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->vddi_gpio);*/

	ctx->vci_gpio =
		devm_gpiod_get(ctx->dev, "vci", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vci_gpio)) {
		dev_err(ctx->dev, "%s: cannot get vci_gpio %ld\n",
			__func__, PTR_ERR(ctx->vci_gpio));
		return PTR_ERR(ctx->vci_gpio);
	}
	gpiod_set_value(ctx->vci_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->vci_gpio);

	msleep(20);

	lcm_panel_init(ctx);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	ctx->prepared = true;

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_rst(panel);
#endif
#ifdef PANEL_SUPPORT_READBACK
	lcm_panel_get_data(ctx);
#endif

	return ret;
}

static int lcm_enable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	pr_info("%s\n", __func__);
	if (ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = true;

	return 0;
}


#define PLL_CLOCK_FHD (550)

#define DATA_RATE		1100

#define MODE0_FPS		60
#define MODE0_HFP		140
#define MODE0_HSA		16
#define MODE0_HBP		24
#define MODE0_VFP		2452
#define MODE0_VSA		2
#define MODE0_VBP		10

#define MODE2_FPS		90
#define MODE2_HFP		140
#define MODE2_HSA		16
#define MODE2_HBP		24
#define MODE2_VFP		836
#define MODE2_VSA		2
#define MODE2_VBP		10

#define MODE3_FPS		120
#define MODE3_HFP		140
#define MODE3_HSA		16
#define MODE3_HBP		24
#define MODE3_VFP		20
#define MODE3_VSA		2
#define MODE3_VBP		10
#define HACT_FHDP		1080
#define VACT_FHDP		2400

static struct drm_display_mode fhd_120_mode = {
	.clock		= 367718, //htotal*vtotal*fps
	.hdisplay	= HACT_FHDP,
	.hsync_start	= HACT_FHDP + MODE3_HFP,
	.hsync_end	= HACT_FHDP + MODE3_HFP + MODE3_HSA,
	.htotal		= HACT_FHDP + MODE3_HFP + MODE3_HSA + MODE3_HBP, //1260
	.vdisplay	= VACT_FHDP,
	.vsync_start	= VACT_FHDP + MODE3_VFP,
	.vsync_end	= VACT_FHDP + MODE3_VFP + MODE3_VSA,
	.vtotal		= VACT_FHDP + MODE3_VFP + MODE3_VSA + MODE3_VBP, //2432
};

static struct drm_display_mode fhd_90_mode = {
	.clock		= 368323, //htotal*vtotal*fps
	.hdisplay	= HACT_FHDP,
	.hsync_start	= HACT_FHDP + MODE2_HFP,
	.hsync_end	= HACT_FHDP + MODE2_HFP + MODE2_HSA,
	.htotal		= HACT_FHDP + MODE2_HFP + MODE2_HSA + MODE2_HBP, //1260
	.vdisplay	= VACT_FHDP,
	.vsync_start	= VACT_FHDP + MODE2_VFP,
	.vsync_end	= VACT_FHDP + MODE2_VFP + MODE2_VSA,
	.vtotal		= VACT_FHDP + MODE2_VFP + MODE2_VSA + MODE2_VBP, //3248
};

static struct drm_display_mode fhd_60_mode = {
	.clock		= 367718, //htotal*vtotal*fps
	.hdisplay	= HACT_FHDP,
	.hsync_start	= HACT_FHDP + MODE0_HFP,
	.hsync_end	= HACT_FHDP + MODE0_HFP + MODE0_HSA,
	.htotal		= HACT_FHDP + MODE0_HFP + MODE0_HSA + MODE0_HBP, //1260
	.vdisplay	= VACT_FHDP,
	.vsync_start	= VACT_FHDP + MODE0_VFP,
	.vsync_end	= VACT_FHDP + MODE0_VFP + MODE0_VSA,
	.vtotal		= VACT_FHDP + MODE0_VFP + MODE0_VSA + MODE0_VBP, //4864
};


#if defined(CONFIG_MTK_PANEL_EXT)
static int panel_ext_reset(struct drm_panel *panel, int on)
{
	struct lcm *ctx = panel_to_lcm(panel);

	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_info(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	gpiod_set_value(ctx->reset_gpio, on);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	return 0;
}

static int panel_ata_check(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	unsigned char data[3] = {0, 0, 0};
	unsigned char id[3] = {0x00, 0x80, 0x00};
	ssize_t ret;

	pr_info("%s success\n", __func__);

	ret = mipi_dsi_dcs_read(dsi, 0x4, data, 3);
	if (ret < 0)
		pr_info("%s error\n", __func__);

	DDPINFO("ATA read data %x %x %x\n", data[0], data[1], data[2]);

	if (data[0] == id[0] &&
			data[1] == id[1] &&
			data[2] == id[2])
		return 1;

	DDPINFO("ATA expect read data is %x %x %x\n",
			id[0], id[1], id[2]);

	return 1;
}

static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb,
	void *handle, unsigned int level)
{
	struct lcm *ctx = panel_to_lcm(this_panel);
	char bl_tb0[] = {0x51, 0x03, 0xFF};

	pr_err("lcm backlight level is %u\n", level);
	if (level > HBM_MAP_MAX_BRIGHTNESS) {
		pr_err("[kernel/lcm]%s err level;", __func__);
		return -1;
	} else if (level > NORMAL_MAX_BRIGHTNESS) {
		if(!hbm_on){
			lcm_dcs_write_seq_static(ctx, 0x53, 0xE8);
			hbm_on = true;
		}
		level = level - (NORMAL_MAX_BRIGHTNESS + 1);
	} else {
		if(hbm_on){
			lcm_dcs_write_seq_static(ctx, 0x53, 0x28);
			hbm_on = false;
		}
	}

	bl_tb0[1] = ((level & 0xF00) >> 8) & 0xF;
	bl_tb0[2] = level & 0xFF;

	curr_bl1 = bl_tb0[1];
	curr_bl2 = bl_tb0[2];

	DDPINFO("lcm backlight level is %u\n", level);
	if (!cb)
		return -1;
       /*sensor add level callback*/
        backlight_level_notifier_call_chain((unsigned long)level, NULL);
        //pr_err("%s  level=%d\n", __func__, level);
       /*sensor add level callback*/

	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

	return 0;
}

#ifdef CONFIG_MI_DISP
static unsigned char LCM_AOD_HBM[] = {0x51, 0x01 , 0x18};
static unsigned char LCM_AOD_LBM[] = {0x51, 0x00 , 0x16};
static unsigned char LCM_AOD_TO_NORMAL[] = {0x51, 0, 0};
static int panel_set_doze_brightness(struct drm_panel *panel, int doze_brightness) {
	struct lcm *ctx = NULL;
	int ret = 0;
	if (!panel) {
		pr_err("%s: panel is NULL\n", __func__);
		ret = -1;
		goto err;
	}

	mdelay(100);
	ctx = panel_to_lcm(panel);
	cmd_msg->channel = 0;
	cmd_msg->flags = 0;
	cmd_msg->tx_cmd_num = 1;
	cmd_msg->type[0] = 0x39;
	cmd_msg->tx_len[0] = 3;
	switch (doze_brightness) {
		case DOZE_BRIGHTNESS_HBM:
			cmd_msg->tx_buf[0] = LCM_AOD_HBM;
			break;
		case DOZE_BRIGHTNESS_LBM:
			cmd_msg->tx_buf[0] = LCM_AOD_LBM;
			break;
		case DOZE_TO_NORMAL:
			cmd_msg->tx_buf[0] = LCM_AOD_TO_NORMAL;
			return ret;
			break;
		default:
			pr_err("%s: doze_brightness is invalid\n", __func__);
			ret = -1;
			goto err;
	}
	pr_info("doze_brightness = %d\n", doze_brightness);
	mtk_ddic_dsi_send_cmd(cmd_msg, false, false);
	ctx->doze_brightness = doze_brightness;
err:
	return ret;
}

int panel_get_doze_brightness(struct drm_panel *panel, u32 *brightness) {
	struct lcm *ctx = NULL;
	if (!panel) {
		pr_err("%s: panel is NULL\n", __func__);
		return -1;
	}
	ctx = panel_to_lcm(panel);
	*brightness = ctx->doze_brightness;
	return 0;
}
#endif

static struct mtk_panel_params ext_params_fhd_120 = {
	.data_rate = PLL_CLOCK_FHD * 2,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.mi_esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A,
		.count = 1,
		.para_list[0] = 0x9F,
	},
	.lcm_esd_check_table[1] = {
		.cmd = 0xEE,
		.count = 2,
		.para_list[0] = 0x00,
		.para_list[1] = 0x00,
	},
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.physical_width_um = 69550,
	.physical_height_um = 154560,
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.dsc_params = {
		.enable = 1,
		.ver = 17,
		.slice_mode = 1,
		.rgb_swap = 0,
		.dsc_cfg = 34,
		.rct_on = 1,
		.bit_per_channel = 8,
		.dsc_line_buf_depth = 9,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2400,
		.pic_width = 1080,
		.slice_height = 40,
		.slice_width = 540,
		.chunk_size = 540,
		.xmit_delay = 512,
		.dec_delay = 526,
		.scale_value = 32,
		.increment_interval = 989,
		.decrement_interval = 7,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 631,
		.slice_bpg_offset = 651,
		.initial_offset = 6144,
		.final_offset = 4336,
		.flatness_minqp = 3,
		.flatness_maxqp = 12,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 11,
		.rc_quant_incr_limit1 = 11,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,

	},
	.change_fps_by_vfp_send_cmd = 1,
	.dyn_fps = {
		.switch_en=1,
		.vact_timing_fps = 120,
		.dfps_cmd_table[0] = {0, 3 , {0xF0, 0x5A, 0x5A}},
		.dfps_cmd_table[1] = {0, 2 , {0x60, 0x01}},
		.dfps_cmd_table[2] = {0, 2 , {0xF7, 0x0B}},
  	},
};

static struct mtk_panel_params ext_params_fhd_90 = {
	.data_rate = PLL_CLOCK_FHD * 2,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.mi_esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A,
		.count = 1,
		.para_list[0] = 0x9F,
	},
	.lcm_esd_check_table[1] = {
		.cmd = 0xEE,
		.count = 2,
		.para_list[0] = 0x00,
		.para_list[1] = 0x00,
	},
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.physical_width_um = 69550,
	.physical_height_um = 154560,
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.dsc_params = {
		.enable = 1,
		.ver = 17,
		.slice_mode = 1,
		.rgb_swap = 0,
		.dsc_cfg = 34,
		.rct_on = 1,
		.bit_per_channel = 8,
		.dsc_line_buf_depth = 9,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2400,
		.pic_width = 1080,
		.slice_height = 40,
		.slice_width = 540,
		.chunk_size = 540,
		.xmit_delay = 512,
		.dec_delay = 526,
		.scale_value = 32,
		.increment_interval = 989,
		.decrement_interval = 7,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 631,
		.slice_bpg_offset = 651,
		.initial_offset = 6144,
		.final_offset = 4336,
		.flatness_minqp = 3,
		.flatness_maxqp = 12,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 11,
		.rc_quant_incr_limit1 = 11,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,

	},
	.change_fps_by_vfp_send_cmd = 1,
	.dyn_fps = {
		.switch_en=1,
		.vact_timing_fps = 120,
		.dfps_cmd_table[0] = {0, 3 , {0xF0, 0x5A, 0x5A}},
		.dfps_cmd_table[1] = {0, 3 , {0xF1, 0x5A, 0x5A}},
		.dfps_cmd_table[2] = {0, 2 , {0x60, 0x01}},
		.dfps_cmd_table[3] = {0, 4 , {0xB0, 0x00, 0x0B, 0xF2}},
		.dfps_cmd_table[4] = {0, 3 , {0xF2, 0x03, 0x44}},
		.dfps_cmd_table[5] = {0, 4 , {0xB0, 0x01, 0x9E, 0xB2}},
		.dfps_cmd_table[6] = {0, 15 , {0xB2, 0x0C, 0xB0, 0x0C, 0x60, 0x0C, 0x20, 0x0B, 0x91, 0x0B, 0x02, 0x0A, 0x65, 0x09, 0x47}},
		.dfps_cmd_table[7] = {0, 19 , {0xB3, 0x08, 0x1A, 0x06, 0xFC, 0x05, 0xCF, 0x04, 0xB1, 0x03, 0x84, 0x01, 0xD7, 0x00, 0x1C, 0x00, 0x1C, 0x00, 0x1C}},
		.dfps_cmd_table[8] = {0, 2 , {0xF7, 0x0B}},
		.dfps_cmd_table[9] = {0, 3 , {0xF1, 0xA5,0xA5}},
		.dfps_cmd_table[10] = {0, 3 , {0xF0, 0xA5,0xA5}},
	},
};

static struct mtk_panel_params ext_params_fhd_60 = {
	.data_rate = PLL_CLOCK_FHD * 2,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.mi_esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A,
		.count = 1,
		.para_list[0] = 0x9F,
	},
	.lcm_esd_check_table[1] = {
		.cmd = 0xEE,
		.count = 2,
		.para_list[0] = 0x00,
		.para_list[1] = 0x00,
	},
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.physical_width_um = 69550,
	.physical_height_um = 154560,
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.dsc_params = {
		.enable = 1,
		.ver = 17,
		.slice_mode = 1,
		.rgb_swap = 0,
		.dsc_cfg = 34,
		.rct_on = 1,
		.bit_per_channel = 8,
		.dsc_line_buf_depth = 9,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2400,
		.pic_width = 1080,
		.slice_height = 40,
		.slice_width = 540,
		.chunk_size = 540,
		.xmit_delay = 512,
		.dec_delay = 526,
		.scale_value = 32,
		.increment_interval = 989,
		.decrement_interval = 7,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 631,
		.slice_bpg_offset = 651,
		.initial_offset = 6144,
		.final_offset = 4336,
		.flatness_minqp = 3,
		.flatness_maxqp = 12,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 11,
		.rc_quant_incr_limit1 = 11,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,
	},
	.change_fps_by_vfp_send_cmd = 1,
	.dyn_fps = {
  		.switch_en=1,
		.vact_timing_fps = 120,
  		.dfps_cmd_table[0] = {0, 3 , {0xF0, 0x5A, 0x5A}},
  		.dfps_cmd_table[1] = {0, 2 , {0x60, 0x21}},
  		.dfps_cmd_table[2] = {0, 2 , {0xF7, 0x0B}},
  	},
};

int convert_mode_id_to_pmode_id(struct drm_panel *panel,
	struct drm_connector *connector, unsigned int mode)
{
	struct lcm *ctx = panel_to_lcm(panel);
	struct drm_display_mode *m, *pmode;
	unsigned int i = 0;

	list_for_each_entry(m, &connector->modes, head) {
		if (i == mode)
			break;
		i++;
	}

	if (list_empty(&ctx->probed_modes)) {
		pr_info("ctx->probed_modes is empty\n");
		return -1;
	}

	i = 0;
	list_for_each_entry(pmode, &ctx->probed_modes, head) {
		if (drm_mode_equal(pmode, m))
			return i;
		i++;
	}

	return -1;
}

static int mtk_panel_ext_param_set(struct drm_panel *panel,
			struct drm_connector *connector, unsigned int mode)
{
	struct mtk_panel_ext *ext = find_panel_ext(panel);
	int ret = 0;
	int pmode_id = convert_mode_id_to_pmode_id(panel, connector, mode);

	DDPMSG("%s mode:%d, pmode_id:%d\n", __func__, mode, pmode_id);

	switch (pmode_id) {
	case 0:
		ext->params = &ext_params_fhd_60;
		break;
	case 1:
		ext->params = &ext_params_fhd_120;
		break;
	case 2:
		ext->params = &ext_params_fhd_90;
		break;
	default:
		ret = 1;
	}

	return ret;
}

static int mtk_panel_ext_param_get(struct drm_panel *panel,
	struct drm_connector *connector,
	struct mtk_panel_params **ext_param,
	unsigned int mode)
{
	int ret = 0;
	int pmode_id = convert_mode_id_to_pmode_id(panel, connector, mode);

	DDPMSG("%s mode:%d, pmode_id:%d\n", __func__, mode, pmode_id);

	switch (pmode_id) {
	case 0:
		*ext_param = &ext_params_fhd_60;
		break;
	case 1:
		*ext_param = &ext_params_fhd_120;
		break;
	case 2:
		*ext_param = &ext_params_fhd_90;
		break;
	default:
		ret = 1;
	}

	if (*ext_param)
		pr_info("data_rate:%d\n", (*ext_param)->data_rate);
	else
		pr_info("ext_param is NULL;\n");

	return ret;
}

static void mode_switch_to_fhd120(struct drm_panel *panel,
			enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (stage == BEFORE_DSI_POWERDOWN) {
	} else if (stage == AFTER_DSI_POWERON) {
		lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
		lcm_dcs_write_seq_static(ctx, 0x60, 0x01);
		lcm_dcs_write_seq_static(ctx, 0xF7, 0x0B);
		lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);
	}
}

static void mode_switch_to_fhd60(struct drm_panel *panel,
			enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (stage == BEFORE_DSI_POWERDOWN) {
	} else if (stage == AFTER_DSI_POWERON) {
		lcm_dcs_write_seq_static(ctx, 0xF0, 0x5A, 0x5A);
		lcm_dcs_write_seq_static(ctx, 0x60, 0x21);
		lcm_dcs_write_seq_static(ctx, 0xF7, 0x0B);
		lcm_dcs_write_seq_static(ctx, 0xF0, 0xA5, 0xA5);

	}
}


static int mode_switch(struct drm_panel *panel,
		struct drm_connector *connector, unsigned int cur_mode,
		unsigned int dst_mode, enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	int ret = 0;
	struct lcm *ctx = panel_to_lcm(panel);
	int pmode_id = 0;

	if (cur_mode == dst_mode)
		return ret;

	pmode_id = convert_mode_id_to_pmode_id(panel, connector, dst_mode);
	DDPMSG("%s cur_mode:%d, dst_mode:%d, pmode_id:%d\n", __func__,
		cur_mode, dst_mode, pmode_id);

	switch (pmode_id) {
	case 0:
		mode_switch_to_fhd60(panel, stage);
		break;
	case 1:
		mode_switch_to_fhd120(panel, stage);
		break;
	default:
		ret = 1;
	}

	if (stage == AFTER_DSI_POWERON)
		ctx->pmode_id = pmode_id;

	return ret;
}

static int panel_get_panel_info(struct drm_panel *panel, char *buf)
{
	int count = 0;
	struct lcm *ctx;

	if (!panel) {
		pr_err(": panel is NULL\n", __func__);
		return -EAGAIN;
	}

	ctx = panel_to_lcm(panel);
	count = snprintf(buf, PAGE_SIZE, "%s\n", ctx->panel_info);

	return count;
}

static bool get_lcm_initialized(struct drm_panel *panel)
{
	bool ret = false;
	struct lcm *ctx;
	if (!panel) {
		pr_err("invalid params\n");
		return -EAGAIN;
	}
	ctx = panel_to_lcm(panel);
	ret = ctx->prepared;
	return ret;
}

static int panel_set_gir_on_control(struct drm_panel *panel)
{
	struct lcm *ctx;
	if (!panel) {
		pr_err("invalid params\n");
		return -EAGAIN;
	}
	ctx = panel_to_lcm(panel);
	if (!ctx || !ctx->enabled) {
		pr_err("%s: panel isn't enabled\n", __func__);
		return -1;
	}

	mutex_lock(&ctx->flm_lock);
	panel_ddic_send_cmd(flat_mode_on, ARRAY_SIZE(flat_mode_on),  true);
	mutex_unlock(&ctx->flm_lock);
	return 0;
}

static int panel_set_gir_off_control(struct drm_panel *panel)
{
	struct lcm *ctx;
	if (!panel) {
		pr_err("invalid params\n");
		return -EAGAIN;
	}
	ctx = panel_to_lcm(panel);
	if (!ctx || !ctx->enabled) {
		pr_err("%s: panel isn't enabled\n", __func__);
		return -1;
	}

	mutex_lock(&ctx->flm_lock);
	panel_ddic_send_cmd(flat_mode_off, ARRAY_SIZE(flat_mode_off),  true);
	mutex_unlock(&ctx->flm_lock);

	return 0;
}

static int panel_normal_hbm_control(struct drm_panel *panel, uint32_t level)
{
	struct lcm *ctx;
	if (!panel) {
		pr_err("invalid params\n");
		return -EAGAIN;
	}
	ctx = panel_to_lcm(panel);
	mutex_lock(&ctx->panel_lock);
	if (level == 1) {
		panel_ddic_send_cmd(normal_hbm_on, ARRAY_SIZE(normal_hbm_on),  true);
	} else if (level == 0) {
		panel_ddic_send_cmd(normal_hbm_off, ARRAY_SIZE(normal_hbm_off),  true);
	}
	mutex_unlock(&ctx->panel_lock);
	return 0;
}

static int panel_set_lhbm_fod(struct mtk_dsi *dsi, enum local_hbm_state lhbm_state)
{
	struct lcm *ctx = NULL;
	struct mi_dsi_panel_cfg *mi_cfg = NULL;

	if(!dsi || !dsi->panel) {
		pr_err("%s:panel is NULL\n", __func__);
		return -1;
	}

	mi_cfg = &dsi->mi_cfg;
	ctx = panel_to_lcm(dsi->panel);
	if (!ctx || !ctx->enabled) {
		pr_err("%s: panel isn't enabled\n", __func__);
		return -1;
	}

	pr_info("%s local hbm_state :%d \n", __func__, lhbm_state);

	switch (lhbm_state) {
	case LOCAL_HBM_OFF_TO_NORMAL://0
	case LOCAL_HBM_OFF_TO_NORMAL_BACKLIGHT:
	case LOCAL_HBM_OFF_TO_NORMAL_BACKLIGHT_RESTORE:
		pr_info("LOCAL_HBM_NORMAL off\n");
		mutex_lock(&ctx->lhbm_lock);
		if(hbm_on){
			panel_ddic_send_cmd(local_hbm_normal_off, ARRAY_SIZE(local_hbm_normal_off),  true);
			panel_ddic_send_cmd(normal_hbm_on_001, ARRAY_SIZE(normal_hbm_on_001), true);
		} else {
			panel_ddic_send_cmd(local_hbm_normal_off, ARRAY_SIZE(local_hbm_normal_off),  true);
		}
		mutex_unlock(&ctx->lhbm_lock);
		break;
	case LOCAL_HBM_NORMAL_WHITE_1000NIT://1
	case LOCAL_HBM_HLPM_WHITE_1000NIT:
		pr_info("LOCAL_HBM_NORMAL_WHITE_1000NIT in HBM\n");
		mutex_lock(&ctx->lhbm_lock);
		if(hbm_on){
			panel_ddic_send_cmd(normal_hbm_off_7FF, ARRAY_SIZE(normal_hbm_off_7FF), true);
			panel_ddic_send_cmd(local_hbm_normal_white_1200nit, ARRAY_SIZE(local_hbm_normal_white_1200nit), true);
		} else {
			panel_ddic_send_cmd(local_hbm_normal_white_1200nit, ARRAY_SIZE(local_hbm_normal_white_1200nit), true);
		}
		mutex_unlock(&ctx->lhbm_lock);
		break;
	case LOCAL_HBM_NORMAL_WHITE_750NIT://2
		pr_info("LOCAL_HBM_NORMAL_WHITE_750NIT in HBM\n");
		break;
	case LOCAL_HBM_NORMAL_WHITE_500NIT://3
		pr_info("LOCAL_HBM_NORMAL_WHITE_500NIT in HBM\n");
		break;
	case LOCAL_HBM_NORMAL_WHITE_110NIT://4
	case LOCAL_HBM_HLPM_WHITE_110NIT:
		pr_info("LOCAL_HBM_NORMAL_WHITE_110NIT in HBM\n");
		mutex_lock(&ctx->lhbm_lock);
		panel_ddic_send_cmd(local_hbm_normal_white_200nit, ARRAY_SIZE(local_hbm_normal_white_200nit), true);
		mutex_unlock(&ctx->lhbm_lock);
		break;
	case LOCAL_HBM_NORMAL_GREEN_500NIT://5
		pr_info("LOCAL_HBM_NORMAL_GREEN_500NIT in HBM\n");
		mutex_lock(&ctx->lhbm_lock);
		panel_ddic_send_cmd(local_hbm_normal_green_500nit, ARRAY_SIZE(local_hbm_normal_green_500nit), true);
		mutex_unlock(&ctx->lhbm_lock);
		break;
	default:
		pr_info("invalid local hbm value\n");
		break;
	}

	return 0;

}

static struct LCM_setting_table esd_restore_bl[] = {
	{0x51, 02, {0x03,0xFF}},
};


static void lcm_esd_restore_backlight(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	esd_restore_bl->para_list[0] = curr_bl1;
	esd_restore_bl->para_list[1] = curr_bl2;
	push_table(ctx, esd_restore_bl,
		sizeof(esd_restore_bl) / sizeof(struct LCM_setting_table));
        pr_info("%s, lcm_esd_restore_backlight restore curr_bl1:%x curr_bl2:%x \n", __func__, curr_bl1, curr_bl2);

	pr_info("lcm_esd_restore_backlight \n");
	return;
}

static int panel_get_wp_info(struct drm_panel *panel, char *buf, size_t size)
{
	struct device_node *chosen;
	char *tmp_buf = NULL;
	int tmp_size = 0;
	chosen = of_find_node_by_path("/chosen");
	if (chosen) {
		tmp_buf = (char *)of_get_property(chosen, "wp_info", (int *)&tmp_size);
		if (tmp_size > 0) {
			strncpy(buf, tmp_buf, tmp_size);
			pr_info("[%s]: white_point = %s, size = %d\n", __func__, buf, tmp_size);
		}
	} else {
		pr_info("[%s]:find chosen failed\n", __func__);
	}

	return tmp_size;
}

static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.ata_check = panel_ata_check,
	.ext_param_set = mtk_panel_ext_param_set,
	.ext_param_get = mtk_panel_ext_param_get,
	.mode_switch = mode_switch,
	.get_panel_info = panel_get_panel_info,
	.get_panel_initialized = get_lcm_initialized,
	.set_lhbm_fod = panel_set_lhbm_fod,
	.normal_hbm_control = panel_normal_hbm_control,
	.panel_set_gir_on = panel_set_gir_on_control,
	.panel_set_gir_off = panel_set_gir_off_control,
	.get_wp_info = panel_get_wp_info,
	.set_doze_brightness = panel_set_doze_brightness,
	.get_doze_brightness = panel_get_doze_brightness,
	.LC_esd_restore_backlight = lcm_esd_restore_backlight,
};
#endif

struct panel_desc {
	const struct drm_display_mode *modes;
	unsigned int num_modes;

	unsigned int bpc;

	struct {
		unsigned int width;
		unsigned int height;
	} size;

	struct {
		unsigned int prepare;
		unsigned int enable;
		unsigned int disable;
		unsigned int unprepare;
	} delay;
};

static int lcm_get_modes(struct drm_panel *panel,
			struct drm_connector *connector)
{
	struct drm_display_mode *mode0;
	struct drm_display_mode *mode1;
	struct drm_display_mode *mode2;
	struct lcm *ctx = panel_to_lcm(panel);

	mode0 = drm_mode_duplicate(connector->dev, &fhd_60_mode);
	if (!mode0) {
		dev_info(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
			fhd_60_mode.hdisplay, fhd_60_mode.vdisplay,
			drm_mode_vrefresh(&fhd_60_mode));
		return -ENOMEM;
	}

	drm_mode_set_name(mode0);
	mode0->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode0);
	ctx->pmode_id = 0;
	list_add_tail(&fhd_60_mode.head, &ctx->probed_modes);

	mode1 = drm_mode_duplicate(connector->dev, &fhd_120_mode);
	if (!mode1) {
		dev_info(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
			fhd_120_mode.hdisplay,
			fhd_120_mode.vdisplay,
			drm_mode_vrefresh(&fhd_120_mode));
		return -ENOMEM;
	}

	drm_mode_set_name(mode1);
	mode1->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, mode1);
	list_add_tail(&fhd_120_mode.head, &ctx->probed_modes);

	mode2 = drm_mode_duplicate(connector->dev, &fhd_90_mode);
	if (!mode2) {
		dev_info(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
			fhd_90_mode.hdisplay,
			fhd_90_mode.vdisplay,
			drm_mode_vrefresh(&fhd_90_mode));
		return -ENOMEM;
	}

	drm_mode_set_name(mode2);
	mode2->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, mode2);
	list_add_tail(&fhd_90_mode.head, &ctx->probed_modes);

	return 1;
}

static const struct drm_panel_funcs lcm_drm_funcs = {
	.disable = lcm_disable,
	.unprepare = lcm_unprepare,
	.prepare = lcm_prepare,
	.enable = lcm_enable,
	.get_modes = lcm_get_modes,
};

static int lcm_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct lcm *ctx;
	struct device_node *backlight;
	int ret;
	struct device_node *dsi_node, *remote_node = NULL, *endpoint = NULL;
	pr_info("%s+ start\n", __func__);

	dsi_node = of_get_parent(dev->of_node);
	if (dsi_node) {
		endpoint = of_graph_get_next_endpoint(dsi_node, NULL);
		if (endpoint) {
			remote_node = of_graph_get_remote_port_parent(endpoint);
			if (!remote_node) {
				pr_info("No panel connected,skip probe lcm\n");
				return -ENODEV;
			}
			pr_info("device node name:%s\n", remote_node->name);
		}
	}
	if (remote_node != dev->of_node) {
		pr_info("%s+ skip probe due to not current lcm\n", __func__);
		return -ENODEV;
	}

	ctx = devm_kzalloc(dev, sizeof(struct lcm), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST
			  | MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_EOT_PACKET | MIPI_DSI_CLOCK_NON_CONTINUOUS;

	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		ctx->backlight = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!ctx->backlight)
			return -EPROBE_DEFER;
	}

	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset-gpios %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	devm_gpiod_put(dev, ctx->reset_gpio);

/*	ctx->vddi_gpio =
		devm_gpiod_get(ctx->dev, "vddi", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vddi_gpio)) {
		dev_err(ctx->dev, "%s: cannot get vddi_gpio %ld\n",
			__func__, PTR_ERR(ctx->vddi_gpio));
		return PTR_ERR(ctx->vddi_gpio);
	}
	devm_gpiod_put(ctx->dev, ctx->vddi_gpio);*/

	ctx->vci_gpio =
		devm_gpiod_get(ctx->dev, "vci", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vci_gpio)) {
		dev_err(ctx->dev, "%s: cannot get vci_gpio %ld\n",
			__func__, PTR_ERR(ctx->vci_gpio));
		return PTR_ERR(ctx->vci_gpio);
	}
	devm_gpiod_put(ctx->dev, ctx->vci_gpio);
#ifdef CONFIG_FACTORY_BUILD
	pr_info("Factory build no need ESD function!\n");
#else
	ext_params_fhd_120.err_flag_irq_gpio = of_get_named_gpio_flags(
		dev->of_node, "mi,esd-err-irq-gpio",
		0, (enum of_gpio_flags *)&(ext_params_fhd_120.err_flag_irq_flags));
#endif
#ifndef CONFIG_MTK_DISP_NO_LK
	ctx->prepared = true;
	ctx->enabled = true;
#endif

	drm_panel_init(&ctx->panel, dev, &lcm_drm_funcs, DRM_MODE_CONNECTOR_DSI);
	ctx->panel.dev = dev;
	ctx->panel.funcs = &lcm_drm_funcs;

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&ctx->panel);

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_handle_reg(&ctx->panel);
	ret = mtk_panel_ext_create(dev, &ext_params_fhd_120, &ext_funcs, &ctx->panel);
	if (ret < 0)
		return ret;
#endif
	ctx->panel_info = panel_name;
	INIT_LIST_HEAD(&ctx->probed_modes);
	this_panel = &ctx->panel;

	cmd_msg = vmalloc(sizeof(struct mtk_ddic_dsi_msg));
	if (cmd_msg == NULL) {
		ret= -ENOMEM;
		pr_err("fail to vmalloc for cmd_msg\n");
	} else {
		memset(cmd_msg, 0, sizeof(struct mtk_ddic_dsi_msg));
        }

	ret = sysfs_create_group(&dev->kobj, &o17_attr_group);
	if (ret)
		return ret;

	pr_info("%s-end\n", __func__);

	return ret;
}

static int lcm_remove(struct mipi_dsi_device *dsi)
{
	struct lcm *ctx = mipi_dsi_get_drvdata(dsi);
#if defined(CONFIG_MTK_PANEL_EXT)
	struct mtk_panel_ctx *ext_ctx = find_panel_ctx(&ctx->panel);
#endif
	vfree(cmd_msg);
	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_detach(ext_ctx);
	mtk_panel_remove(ext_ctx);
#endif

	return 0;
}

static const struct of_device_id lcm_of_match[] = {
	{ .compatible = "dsi_panel_o17_38_0c_0a_fhdp_vdo,lcm", },
	{ }
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "dsi_panel_o17_38_0c_0a_fhdp_vdo",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

module_mipi_dsi_driver(lcm_driver);

MODULE_AUTHOR("MEDIATEK");
MODULE_DESCRIPTION("Samsung ANA6705 AMOLED CMD LCD Panel Driver");
MODULE_LICENSE("GPL v2");
