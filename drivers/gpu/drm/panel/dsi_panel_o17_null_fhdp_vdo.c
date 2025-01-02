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

#define MTE_OFF (0xFFFF)

struct lcm {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	bool prepared;
	bool enabled;
	int pmode_id;
	struct list_head probed_modes;
	const char *panel_info;
	int error;
};

static const char *panel_name = "panel_name=dsi_panel_o17_null_fhdp_vdo";

static inline struct lcm *panel_to_lcm(struct drm_panel *panel)
{
	return container_of(panel, struct lcm, panel);
}

static void lcm_panel_init(struct lcm *ctx)
{

}

static int lcm_disable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (!ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = false;

	return 0;
}

static int lcm_unprepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (!ctx->prepared)
		return 0;

	ctx->error = 0;
	ctx->prepared = false;

	return 0;
}

static int lcm_prepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	pr_info("%s\n", __func__);
	if (ctx->prepared)
		return 0;

	lcm_panel_init(ctx);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	ctx->prepared = true;

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_rst(panel);
#endif
	return ret;
}

static int lcm_enable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = true;

	return 0;
}


#define PLL_CLOCK_FHD (585)

#define DATA_RATE		1170

#define MODE0_FPS		60
#define MODE0_HFP		180
#define MODE0_HSA		16
#define MODE0_HBP		24
#define MODE0_VFP		2452
#define MODE0_VSA		2
#define MODE0_VBP		10

#define MODE3_FPS		120
#define MODE3_HFP		180
#define MODE3_HSA		16
#define MODE3_HBP		24
#define MODE3_VFP		20
#define MODE3_VSA		2
#define MODE3_VBP		10
#define HACT_FHDP		1080
#define VACT_FHDP		2400

static struct drm_display_mode fhd_120_mode = {
	.clock		= 379392, //htotal*vtotal*fps
	.hdisplay	= HACT_FHDP,
	.hsync_start	= HACT_FHDP + MODE3_HFP,
	.hsync_end	= HACT_FHDP + MODE3_HFP + MODE3_HSA,
	.htotal		= HACT_FHDP + MODE3_HFP + MODE3_HSA + MODE3_HBP, //1300
	.vdisplay	= VACT_FHDP,
	.vsync_start	= VACT_FHDP + MODE3_VFP,
	.vsync_end	= VACT_FHDP + MODE3_VFP + MODE3_VSA,
	.vtotal		= VACT_FHDP + MODE3_VFP + MODE3_VSA + MODE3_VBP, //2432
};

static struct drm_display_mode fhd_60_mode = {
	.clock		= 379392, //htotal*vtotal*fps
	.hdisplay	= HACT_FHDP,
	.hsync_start	= HACT_FHDP + MODE0_HFP,
	.hsync_end	= HACT_FHDP + MODE0_HFP + MODE0_HSA,
	.htotal		= HACT_FHDP + MODE0_HFP + MODE0_HSA + MODE0_HBP, //1300
	.vdisplay	= VACT_FHDP,
	.vsync_start	= VACT_FHDP + MODE0_VFP,
	.vsync_end	= VACT_FHDP + MODE0_VFP + MODE0_VSA,
	.vtotal		= VACT_FHDP + MODE0_VFP + MODE0_VSA + MODE0_VBP, //4864
};


#if defined(CONFIG_MTK_PANEL_EXT)
static int panel_ext_reset(struct drm_panel *panel, int on)
{
	return 0;
}


static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb,
	void *handle, unsigned int level)
{
	char bl_tb0[] = {0x51, 0x03, 0xFF};

	if((level > 0) && (level <= 8))
		level = 8;

	bl_tb0[1] = ((level & 0xF00) >> 8) & 0xF;
	bl_tb0[2] = level & 0xFF;

	DDPINFO("null_panel backlight level is %u\n", level);
	if (!cb)
		return -1;

	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

	return 0;
}

static struct mtk_panel_params ext_params_fhd_120 = {
	.data_rate = PLL_CLOCK_FHD * 2,
	.cust_esd_check = 0,
	.esd_check_enable = 0,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0a,
		.count = 1,
		.para_list[0] = 0x9c,
	},
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
};

static struct mtk_panel_params ext_params_fhd_60 = {
	.data_rate = PLL_CLOCK_FHD * 2,
	.cust_esd_check = 0,
	.esd_check_enable = 0,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0a,
		.count = 1,
		.para_list[0] = 0x9c,
	},
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

}

static void mode_switch_to_fhd60(struct drm_panel *panel,
			enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{

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

static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.ext_param_set = mtk_panel_ext_param_set,
	.ext_param_get = mtk_panel_ext_param_get,
	.mode_switch = mode_switch,
	.get_panel_info = panel_get_panel_info,
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


	connector->display_info.width_mm = 70;
	connector->display_info.height_mm = 152;

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
	ret = mtk_panel_ext_create(dev, &ext_params_fhd_60, &ext_funcs, &ctx->panel);
	if (ret < 0)
		return ret;
#endif
	ctx->panel_info = panel_name;
	INIT_LIST_HEAD(&ctx->probed_modes);

	pr_info("%s-end\n", __func__);

	return ret;
}

static int lcm_remove(struct mipi_dsi_device *dsi)
{
	struct lcm *ctx = mipi_dsi_get_drvdata(dsi);
#if defined(CONFIG_MTK_PANEL_EXT)
	struct mtk_panel_ctx *ext_ctx = find_panel_ctx(&ctx->panel);
#endif

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_detach(ext_ctx);
	mtk_panel_remove(ext_ctx);
#endif

	return 0;
}

static const struct of_device_id lcm_of_match[] = {
	{ .compatible = "dsi_panel_o17_null_fhdp_vdo,lcm", },
	{ }
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "dsi_panel_o17_null_fhdp_vdo",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

module_mipi_dsi_driver(lcm_driver);

MODULE_AUTHOR("MEDIATEK");
MODULE_DESCRIPTION("Samsung ANA6705 AMOLED CMD LCD Panel Driver");
MODULE_LICENSE("GPL v2");
