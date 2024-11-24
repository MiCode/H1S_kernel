/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
*/

#include <drm/drmP.h>
#include <linux/clk.h>
#include <linux/component.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/soc/mediatek/mtk-cmdq.h>

#include "mtk_drm_crtc.h"
#include "mtk_drm_ddp_comp.h"
#include "mtk_drm_drv.h"
#include "mtk_disp_ccorr.h"
#include "mtk_disp_color.h"
#include "mtk_log.h"
#include "mtk_dump.h"
#include "mtk_drm_helper.h"

#ifdef CONFIG_LEDS_MTK_DISP
#include <mtk_leds_drv.h>
#include <leds-mtk-disp.h>
#elif defined CONFIG_LEDS_MTK_PWM
#include <mtk_leds_drv.h>
#include <leds-mtk-pwm.h>
#else
#define mt_leds_brightness_set(x, y) do { } while (0)
#endif

#define DISP_REG_CCORR_EN (0x000)
#define DISP_REG_CCORR_INTEN                     (0x008)
#define DISP_REG_CCORR_INTSTA                    (0x00C)
#define DISP_REG_CCORR_CFG (0x020)
#define DISP_REG_CCORR_SIZE (0x030)
#define DISP_REG_CCORR_COLOR_OFFSET_0	(0x100)
#define DISP_REG_CCORR_COLOR_OFFSET_1	(0x104)
#define DISP_REG_CCORR_COLOR_OFFSET_2	(0x108)
#define CCORR_COLOR_OFFSET_MASK	(0x3FFFFFF)

#define DISP_REG_CCORR_SHADOW (0x0A0)
#define CCORR_READ_WORKING		BIT(0)
#define CCORR_BYPASS_SHADOW		BIT(2)

#define CCORR_12BIT_MASK				0x0fff
#define CCORR_13BIT_MASK				0x1fff

#define CCORR_REG(idx) (idx * 4 + 0x80)
#define CCORR_CLIP(val, min, max) ((val >= max) ? \
	max : ((val <= min) ? min : val))

static unsigned int g_ccorr_relay_value[DISP_CCORR_TOTAL];
#define index_of_ccorr(module) ((module == DDP_COMPONENT_CCORR0) ? 0 : 1)

static bool bypass_color0, bypass_color1;

static atomic_t g_ccorr_is_clock_on[DISP_CCORR_TOTAL] = {
	ATOMIC_INIT(0), ATOMIC_INIT(0) };

static struct DRM_DISP_CCORR_COEF_T *g_disp_ccorr_coef[DISP_CCORR_TOTAL] = {
	NULL };
static int g_ccorr_color_matrix[3][3] = {
	{1024, 0, 0},
	{0, 1024, 0},
	{0, 0, 1024} };
static int g_ccorr_prev_matrix[3][3] = {
	{1024, 0, 0},
	{0, 1024, 0},
	{0, 0, 1024} };
static int g_rgb_matrix[3][3] = {
	{1024, 0, 0},
	{0, 1024, 0},
	{0, 0, 1024} };

#ifdef CONFIG_MTK_SLD_SUPPORT
static DEFINE_SPINLOCK(g_sld_bl_change_lock);
static bool sld_end_flag;
static int sld_tmp_bl = 240;
// sld_backligt_value should equals with led default backlight
static int sld_backligt_value = 400;
static int sld_old_backligt_value;
static struct DISP_SLD_PARAM g_sld_param = {
.sld_ccorr_table ={
	0x3, 0x4, 0x4, 0x4, 0x5, 0x6, 0x6, 0x6, 0x7, 0x8, 0x8, 0x8, 0x9, 0xA, 0xA,
	0xA, 0xB, 0xC, 0xC, 0xC, 0xD, 0xE, 0xE, 0xE, 0xF, 0x10, 0x10, 0x10, 0x11,
	0x12, 0x12, 0x12, 0x13, 0x14, 0x14, 0x14, 0x15, 0x16, 0x16, 0x16, 0x17, 0x18,
	0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25,
	0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2E, 0x30, 0x31, 0x32, 0x34, 0x36,
	0x37, 0x38, 0x3A, 0x3C, 0x3D, 0x3E, 0x40, 0x42, 0x43, 0x44, 0x46, 0x48, 0x49,
	0x4B, 0x4D, 0x4F, 0x51, 0x53, 0x55, 0x57, 0x59, 0x5B, 0x5D, 0x5F, 0x61, 0x63,
	0x65, 0x67, 0x69, 0x6B, 0x6D, 0x6F, 0x71, 0x74, 0x76, 0x78, 0x7B, 0x7E, 0x80,
	0x82, 0x85, 0x88, 0x8A, 0x8C, 0x8F, 0x92, 0x94, 0x96, 0x99, 0x9C, 0x9E, 0xA0,
	0xA3, 0xA6, 0xAA, 0xAD, 0xB0, 0xB3, 0xB6, 0xBA, 0xBD, 0xC0, 0xC4, 0xC7, 0xCA,
	0xCD, 0xD0, 0xD4, 0xD7, 0xDA, 0xDE, 0xE1, 0xE4, 0xE8, 0xEC, 0xF1, 0xF5, 0xF9,
	0xFE, 0x102, 0x106, 0x10A, 0x10E, 0x113, 0x117, 0x11B, 0x120, 0x124, 0x128, 0x12C,
	0x130, 0x135, 0x139, 0x13E, 0x144, 0x14A, 0x14F, 0x154, 0x15A, 0x160, 0x165, 0x16A,
	0x170, 0x176, 0x17B, 0x180, 0x186, 0x18C, 0x191, 0x196, 0x19C, 0x1A2, 0x1A7, 0x1AE,
	0x1B5, 0x1BC, 0x1C3, 0x1CA, 0x1D1, 0x1D8, 0x1DF, 0x1E6, 0x1ED, 0x1F4, 0x1FB, 0x202,
	0x209, 0x210, 0x217, 0x21E, 0x225, 0x22C, 0x233, 0x23C, 0x246, 0x24F, 0x258, 0x262,
	0x26B, 0x274, 0x27E, 0x287, 0x291, 0x29A, 0x2A3, 0x2AD, 0x2B6, 0x2BF, 0x2C9, 0x2D2,
	0x2DB, 0x2E5, 0x2EE, 0x2FC, 0x30B, 0x319, 0x327, 0x336, 0x344, 0x353, 0x361, 0x36F,
	0x37E, 0x38C, 0x39A, 0x3A9, 0x3B7, 0x3C6, 0x3D4, 0x3E2, 0x3F1, 0x3FF,
},
.sld_bl = 240,
};

static bool g_sld_enable;
static bool g_sld_enable_pre;

static int g_sld_matrix[3][3] = {
	{1024, 0, 0},
	{0, 1024, 0},
	{0, 0, 1024} };
#endif

static struct DRM_DISP_CCORR_COEF_T g_multiply_matrix_coef;
static int g_disp_ccorr_without_gamma;

static DECLARE_WAIT_QUEUE_HEAD(g_ccorr_get_irq_wq);
//static DEFINE_SPINLOCK(g_ccorr_get_irq_lock);
static DEFINE_SPINLOCK(g_ccorr_clock_lock);
static atomic_t g_ccorr_get_irq = ATOMIC_INIT(0);

/* FOR TRANSITION */
static DEFINE_SPINLOCK(g_pq_bl_change_lock);
static int g_old_pq_backlight;
static int g_pq_backlight;
static int g_pq_backlight_db;
static atomic_t g_ccorr_is_init_valid = ATOMIC_INIT(0);

static DEFINE_MUTEX(g_ccorr_global_lock);
// For color conversion bug fix
static bool need_offset;
#define OFFSET_VALUE (1024)

/* TODO */
/* static ddp_module_notify g_ccorr_ddp_notify; */

// It's a work around for no comp assigned in functions.
static struct mtk_ddp_comp *default_comp;
static struct mtk_ddp_comp *ccorr1_default_comp;

static int disp_ccorr_write_coef_reg(struct mtk_ddp_comp *comp,
	struct cmdq_pkt *handle, int lock);
/* static void ccorr_dump_reg(void); */

enum CCORR_IOCTL_CMD {
	SET_CCORR = 0,
	SET_INTERRUPT,
	BYPASS_CCORR,
};

struct mtk_disp_ccorr_data {
	bool support_shadow;
};

struct mtk_disp_ccorr {
	struct mtk_ddp_comp ddp_comp;
	struct drm_crtc *crtc;
	const struct mtk_disp_ccorr_data *data;
};

static inline struct mtk_disp_ccorr *comp_to_ccorr(struct mtk_ddp_comp *comp)
{
	return container_of(comp, struct mtk_disp_ccorr, ddp_comp);
}

static void disp_ccorr_multiply_3x3(unsigned int ccorrCoef[3][3],
	int color_matrix[3][3], unsigned int resultCoef[3][3])
{
	int temp_Result;
	int signedCcorrCoef[3][3];
	int i, j;

	/* convert unsigned 12 bit ccorr coefficient to signed 12 bit format */
	for (i = 0; i < 3; i += 1) {
		for (j = 0; j < 3; j += 1) {
			if (ccorrCoef[i][j] > 2047) {
				signedCcorrCoef[i][j] =
					(int)ccorrCoef[i][j] - 4096;
			} else {
				signedCcorrCoef[i][j] =
					(int)ccorrCoef[i][j];
			}
		}
	}

	for (i = 0; i < 3; i += 1) {
		DDPINFO("signedCcorrCoef[%d][0-2] = {%d, %d, %d}\n", i,
			signedCcorrCoef[i][0],
			signedCcorrCoef[i][1],
			signedCcorrCoef[i][2]);
	}

	temp_Result = (int)((signedCcorrCoef[0][0]*color_matrix[0][0] +
		signedCcorrCoef[0][1]*color_matrix[1][0] +
		signedCcorrCoef[0][2]*color_matrix[2][0]) / 1024);
	resultCoef[0][0] = CCORR_CLIP(temp_Result, -2048, 2047) & 0xFFF;

	temp_Result = (int)((signedCcorrCoef[0][0]*color_matrix[0][1] +
		signedCcorrCoef[0][1]*color_matrix[1][1] +
		signedCcorrCoef[0][2]*color_matrix[2][1]) / 1024);
	resultCoef[0][1] = CCORR_CLIP(temp_Result, -2048, 2047) & 0xFFF;

	temp_Result = (int)((signedCcorrCoef[0][0]*color_matrix[0][2] +
		signedCcorrCoef[0][1]*color_matrix[1][2] +
		signedCcorrCoef[0][2]*color_matrix[2][2]) / 1024);
	resultCoef[0][2] = CCORR_CLIP(temp_Result, -2048, 2047) & 0xFFF;

	temp_Result = (int)((signedCcorrCoef[1][0]*color_matrix[0][0] +
		signedCcorrCoef[1][1]*color_matrix[1][0] +
		signedCcorrCoef[1][2]*color_matrix[2][0]) / 1024);
	resultCoef[1][0] = CCORR_CLIP(temp_Result, -2048, 2047) & 0xFFF;

	temp_Result = (int)((signedCcorrCoef[1][0]*color_matrix[0][1] +
		signedCcorrCoef[1][1]*color_matrix[1][1] +
		signedCcorrCoef[1][2]*color_matrix[2][1]) / 1024);
	resultCoef[1][1] = CCORR_CLIP(temp_Result, -2048, 2047) & 0xFFF;

	temp_Result = (int)((signedCcorrCoef[1][0]*color_matrix[0][2] +
		signedCcorrCoef[1][1]*color_matrix[1][2] +
		signedCcorrCoef[1][2]*color_matrix[2][2]) / 1024);
	resultCoef[1][2] = CCORR_CLIP(temp_Result, -2048, 2047) & 0xFFF;

	temp_Result = (int)((signedCcorrCoef[2][0]*color_matrix[0][0] +
		signedCcorrCoef[2][1]*color_matrix[1][0] +
		signedCcorrCoef[2][2]*color_matrix[2][0]) / 1024);
	resultCoef[2][0] = CCORR_CLIP(temp_Result, -2048, 2047) & 0xFFF;

	temp_Result = (int)((signedCcorrCoef[2][0]*color_matrix[0][1] +
		signedCcorrCoef[2][1]*color_matrix[1][1] +
		signedCcorrCoef[2][2]*color_matrix[2][1]) / 1024);
	resultCoef[2][1] = CCORR_CLIP(temp_Result, -2048, 2047) & 0xFFF;

	temp_Result = (int)((signedCcorrCoef[2][0]*color_matrix[0][2] +
		signedCcorrCoef[2][1]*color_matrix[1][2] +
		signedCcorrCoef[2][2]*color_matrix[2][2]) / 1024);
	resultCoef[2][2] = CCORR_CLIP(temp_Result, -2048, 2047) & 0xFFF;

	for (i = 0; i < 3; i += 1) {
		DDPINFO("resultCoef[%d][0-2] = {0x%x, 0x%x, 0x%x}\n", i,
			resultCoef[i][0],
			resultCoef[i][1],
			resultCoef[i][2]);
	}
}

static int disp_ccorr_color_matrix_to_dispsys(struct drm_device *dev)
{
	int ret = 0;
	struct mtk_drm_private *private = dev->dev_private;

	// All Support 3*4 matrix on drm architecture
	ret = mtk_drm_helper_set_opt_by_name(private->helper_opt,
		"MTK_DRM_OPT_PQ_34_COLOR_MATRIX", 1);

	return ret;
}

static int disp_ccorr_write_coef_reg(struct mtk_ddp_comp *comp,
	struct cmdq_pkt *handle, int lock)
{
	struct DRM_DISP_CCORR_COEF_T *ccorr, *multiply_matrix;
	int ret = 0;
	int id = index_of_ccorr(comp->id);
	unsigned int temp_matrix[3][3];
	unsigned int cfg_val;
	int i, j;
#ifdef CONFIG_MTK_SLD_SUPPORT
	unsigned int temp_sld_matrix[3][3];
#endif

	if (lock)
		mutex_lock(&g_ccorr_global_lock);

	ccorr = g_disp_ccorr_coef[id];
	if (ccorr == NULL) {
		DDPINFO("%s: [%d] is not initialized\n", __func__, id);
		ret = -EFAULT;
		goto ccorr_write_coef_unlock;
	}

	//if (id == 0) {
		multiply_matrix = &g_multiply_matrix_coef;
		disp_ccorr_multiply_3x3(ccorr->coef, g_ccorr_color_matrix,
			temp_matrix);
#ifdef CONFIG_MTK_SLD_SUPPORT
		disp_ccorr_multiply_3x3(temp_matrix, g_rgb_matrix,
			temp_sld_matrix);
		disp_ccorr_multiply_3x3(temp_sld_matrix, g_sld_matrix,
			multiply_matrix->coef);
#else
		disp_ccorr_multiply_3x3(temp_matrix, g_rgb_matrix,
			multiply_matrix->coef);
#endif

		ccorr = multiply_matrix;

		ccorr->offset[0] = g_disp_ccorr_coef[id]->offset[0];
		ccorr->offset[1] = g_disp_ccorr_coef[id]->offset[1];
		ccorr->offset[2] = g_disp_ccorr_coef[id]->offset[2];
	//}

#if defined(CONFIG_MACH_MT6885) || defined(CONFIG_MACH_MT6873) \
	|| defined(CONFIG_MACH_MT6893) || defined(CONFIG_MACH_MT6853) \
	|| defined(CONFIG_MACH_MT6833) || defined(CONFIG_MACH_MT6877) \
	|| defined(CONFIG_MACH_MT6781)
	// For 6885 need to left shift one bit
	for (i = 0; i < 3; i++)
		for (j = 0; j < 3; j++)
			ccorr->coef[i][j] = ccorr->coef[i][j]<<1;
#endif

	if (handle == NULL) {
		/* use CPU to write */
		writel(1, comp->regs + DISP_REG_CCORR_EN);
		cfg_val = 0x2 | g_ccorr_relay_value[id] |
			     (g_disp_ccorr_without_gamma << 2);
		writel(cfg_val, comp->regs + DISP_REG_CCORR_CFG);
		writel(((ccorr->coef[0][0] & CCORR_13BIT_MASK) << 16) |
			(ccorr->coef[0][1] & CCORR_13BIT_MASK),
			comp->regs + CCORR_REG(0));
		writel(((ccorr->coef[0][2] & CCORR_13BIT_MASK) << 16) |
			(ccorr->coef[1][0] & CCORR_13BIT_MASK),
			comp->regs + CCORR_REG(1));
		writel(((ccorr->coef[1][1] & CCORR_13BIT_MASK) << 16) |
			(ccorr->coef[1][2] & CCORR_13BIT_MASK),
			comp->regs + CCORR_REG(2));
		writel(((ccorr->coef[2][0] & CCORR_12BIT_MASK) << 16) |
			(ccorr->coef[2][1] & CCORR_13BIT_MASK),
			comp->regs + CCORR_REG(3));
		writel(((ccorr->coef[2][2] & CCORR_13BIT_MASK) << 16),
			comp->regs + CCORR_REG(4));
		/* Ccorr Offset */
		writel(((ccorr->offset[0] & CCORR_COLOR_OFFSET_MASK) |
			(0x1 << 31)),
			comp->regs + DISP_REG_CCORR_COLOR_OFFSET_0);
		writel(((ccorr->offset[1] & CCORR_COLOR_OFFSET_MASK)),
			comp->regs + DISP_REG_CCORR_COLOR_OFFSET_1);
		writel(((ccorr->offset[2] & CCORR_COLOR_OFFSET_MASK)),
			comp->regs + DISP_REG_CCORR_COLOR_OFFSET_2);
	} else {
		/* use CMDQ to write */

		cfg_val = 0x2 | g_ccorr_relay_value[index_of_ccorr(comp->id)] |
			     (g_disp_ccorr_without_gamma << 2 | (0x1 << 10));

		cmdq_pkt_write(handle, comp->cmdq_base,
			comp->regs_pa + DISP_REG_CCORR_CFG, cfg_val, ~0);

		cmdq_pkt_write(handle, comp->cmdq_base,
			comp->regs_pa + CCORR_REG(0),
			((ccorr->coef[0][0] & CCORR_13BIT_MASK) << 16) |
			(ccorr->coef[0][1] & CCORR_13BIT_MASK), ~0);
		cmdq_pkt_write(handle, comp->cmdq_base,
			comp->regs_pa + CCORR_REG(1),
			((ccorr->coef[0][2] & CCORR_13BIT_MASK) << 16) |
			(ccorr->coef[1][0] & CCORR_13BIT_MASK), ~0);
		cmdq_pkt_write(handle, comp->cmdq_base,
			comp->regs_pa + CCORR_REG(2),
			((ccorr->coef[1][1] & CCORR_13BIT_MASK) << 16) |
			(ccorr->coef[1][2] & CCORR_13BIT_MASK), ~0);
		cmdq_pkt_write(handle, comp->cmdq_base,
			comp->regs_pa + CCORR_REG(3),
			((ccorr->coef[2][0] & CCORR_13BIT_MASK) << 16) |
			(ccorr->coef[2][1] & CCORR_13BIT_MASK), ~0);
		cmdq_pkt_write(handle, comp->cmdq_base,
			comp->regs_pa + CCORR_REG(4),
			((ccorr->coef[2][2] & CCORR_13BIT_MASK) << 16), ~0);
		/* Ccorr Offset */
		cmdq_pkt_write(handle, comp->cmdq_base,
			comp->regs_pa + DISP_REG_CCORR_COLOR_OFFSET_0,
			(ccorr->offset[0] & CCORR_COLOR_OFFSET_MASK) |
			(0x1 << 31), ~0);
		cmdq_pkt_write(handle, comp->cmdq_base,
			comp->regs_pa + DISP_REG_CCORR_COLOR_OFFSET_1,
			(ccorr->offset[1] & CCORR_COLOR_OFFSET_MASK), ~0);
		cmdq_pkt_write(handle, comp->cmdq_base,
			comp->regs_pa + DISP_REG_CCORR_COLOR_OFFSET_2,
			(ccorr->offset[2] & CCORR_COLOR_OFFSET_MASK), ~0);
	}

	DDPINFO("%s: finish\n", __func__);
ccorr_write_coef_unlock:
	if (lock)
		mutex_unlock(&g_ccorr_global_lock);

	return ret;
}

void disp_ccorr_on_end_of_frame(struct mtk_ddp_comp *comp)
{
	unsigned int intsta;
	unsigned long flags;
	int index = index_of_ccorr(comp->id);
	DDPDBG("%s @ %d......... [IRQ] spin_trylock_irqsave ++ ",
		  __func__, __LINE__);
	if (spin_trylock_irqsave(&g_ccorr_clock_lock, flags)) {
		DDPDBG("%s @ %d......... spin_trylock_irqsave -- ",
			__func__, __LINE__);
		if (atomic_read(&g_ccorr_is_clock_on[index]) != 1) {
			DDPINFO("%s: clock is off. enabled:%d\n", __func__, 0);

			spin_unlock_irqrestore(&g_ccorr_clock_lock, flags);
			DDPDBG("%s @ %d......... spin_unlock_irqrestore -- ",
				__func__, __LINE__);
			return;
		}
		intsta = readl(comp->regs + DISP_REG_CCORR_INTSTA);
		DDPINFO("%s: intsta: 0x%x", __func__, intsta);

		if (intsta & 0x2) {	/* End of frame */
			// Clear irq
			writel(intsta & ~0x3, comp->regs
				+ DISP_REG_CCORR_INTSTA);

			atomic_set(&g_ccorr_get_irq, 1);

			wake_up_interruptible(&g_ccorr_get_irq_wq);
		}
		DDPDBG("%s @ %d......... [IRQ] spin_unlock_irqrestore ++ ",
			__func__, __LINE__);
		spin_unlock_irqrestore(&g_ccorr_clock_lock, flags);
		DDPDBG("%s @ %d......... [IRQ] spin_unlock_irqrestore -- ",
			__func__, __LINE__);
	} else {
		DDPINFO("%s @ %d......... Failed to spin_trylock_irqsave -- ",
			__func__, __LINE__);
	}
}

static void disp_ccorr_set_interrupt(struct mtk_ddp_comp *comp,
					int enabled)
{
	if (default_comp == NULL)
		default_comp = comp;

	if (!enabled && (g_old_pq_backlight != g_pq_backlight))
		g_old_pq_backlight = g_pq_backlight;
	else
		mtk_crtc_user_cmd(&(comp->mtk_crtc->base), comp,
			SET_INTERRUPT, &enabled);
}

/* change backlight when sld enable */
int disp_ccorr_change_backlight(int bl)
{
#ifdef CONFIG_MTK_SLD_SUPPORT

	int backligt_value = bl;
	unsigned long flags;

	DDPINFO("%s sld_backligt_value: %d\n", __func__, sld_backligt_value);

	spin_lock_irqsave(&g_sld_bl_change_lock, flags);
	sld_old_backligt_value = sld_backligt_value;
	sld_backligt_value = bl;
	spin_unlock_irqrestore(&g_sld_bl_change_lock, flags);

	if (g_sld_enable) {
		if (default_comp != NULL &&
			g_ccorr_relay_value[index_of_ccorr(default_comp->id)] != 1) {
			disp_ccorr_set_interrupt(default_comp, 1);

			if (default_comp != NULL &&
					default_comp->mtk_crtc != NULL)
				mtk_crtc_check_trigger(default_comp->mtk_crtc, false,
					true);
		}
		if(backligt_value < g_sld_param.sld_bl)
			backligt_value = g_sld_param.sld_bl;
	}

	return backligt_value;
#else
	return bl;
#endif
}
EXPORT_SYMBOL_GPL(disp_ccorr_change_backlight);

/* set color matrix by 10bit backlight when subdued light display enable */
void disp_ccorr_sld_impl(struct mtk_ddp_comp *comp)
{
#ifdef CONFIG_MTK_SLD_SUPPORT
	int i;
	unsigned long flags;

	if (g_sld_enable) {
		// stop set color matrix when g_sld_matrix equals with target or tmp_bl over sld_bl
		if ((sld_tmp_bl >= g_sld_param.sld_bl - 1)||
			((g_sld_matrix[0][0] == g_sld_param.sld_ccorr_table[sld_tmp_bl])
				 && sld_tmp_bl == sld_backligt_value)) {
			// keep tmp_bl smaller than sld_bl to prevent ccorr_table out of bounds
			if(sld_tmp_bl >= g_sld_param.sld_bl - 1)
				sld_tmp_bl = g_sld_param.sld_bl - 2;
			// set backlight to sld_bl when sld enable
			if((!g_sld_enable_pre && g_sld_enable)
					&& (sld_backligt_value < g_sld_param.sld_bl)) {
				mt_leds_brightness_set("lcd-backlight", g_sld_param.sld_bl);
				g_sld_enable_pre = true;
			}
			sld_end_flag = true;
			disp_ccorr_set_interrupt(comp, 0);
			return;
		} else if (sld_tmp_bl < sld_backligt_value)
			sld_tmp_bl ++;
		else if (sld_tmp_bl > sld_backligt_value)
			sld_tmp_bl --;

		for (i = 0; i < 3; i += 1)
			g_sld_matrix[i][i] = g_sld_param.sld_ccorr_table[sld_tmp_bl];
		sld_end_flag = false;
	} else {
		for (i = 0; i < 3; i += 1)
			g_sld_matrix[i][i] = 1024;
		sld_end_flag = true;
	}
	spin_lock_irqsave(&g_sld_bl_change_lock, flags);
	if (comp->mtk_crtc->is_dual_pipe) {
		disp_ccorr_write_coef_reg(default_comp, NULL, 0);
		disp_ccorr_write_coef_reg(ccorr1_default_comp, NULL, 0);
	} else
		disp_ccorr_write_coef_reg(comp, NULL, 0);
	spin_unlock_irqrestore(&g_sld_bl_change_lock, flags);
	if (comp->mtk_crtc != NULL)
		mtk_crtc_check_trigger(comp->mtk_crtc, false, false);

#endif
}


static void disp_ccorr_clear_irq_only(struct mtk_ddp_comp *comp)
{
	unsigned int intsta;
	unsigned long flags;
	int index = index_of_ccorr(comp->id);

	DDPDBG("%s @ %d......... spin_trylock_irqsave ++ ",
		__func__, __LINE__);
	if (spin_trylock_irqsave(&g_ccorr_clock_lock, flags)) {
		DDPDBG("%s @ %d......... spin_trylock_irqsave -- ",
			__func__, __LINE__);
		if (atomic_read(&g_ccorr_is_clock_on[index]) != 1) {
			DDPINFO("%s: clock is off. enabled:%d\n", __func__, 0);

			spin_unlock_irqrestore(&g_ccorr_clock_lock, flags);
			DDPDBG("%s @ %d......... spin_unlock_irqrestore -- ",
				__func__, __LINE__);
			return;
		}
		intsta = readl(comp->regs + DISP_REG_CCORR_INTSTA);

		DDPINFO("%s: intsta: 0x%x\n", __func__, intsta);

		if (intsta & 0x2) { /* End of frame */
			writel(intsta & ~0x3, comp->regs
					+ DISP_REG_CCORR_INTSTA);
		}
		spin_unlock_irqrestore(&g_ccorr_clock_lock, flags);
		DDPDBG("%s @ %d......... spin_unlock_irqrestore -- ",
			__func__, __LINE__);
	} else {
		DDPINFO("%s @ %d......... Failed to spin_trylock_irqsave -- ",
			__func__, __LINE__);
	}


	/* disable interrupt */
	//disp_ccorr_set_interrupt(comp, 0);

	DDPDBG("%s @ %d......... spin_trylock_irqsave ++ ",
		__func__, __LINE__);
	if (spin_trylock_irqsave(&g_ccorr_clock_lock, flags)) {
		DDPDBG("%s @ %d......... spin_trylock_irqsave -- ",
			__func__, __LINE__);
		if (atomic_read(&g_ccorr_is_clock_on[index]) != 1) {
			DDPINFO("%s: clock is off. enabled:%d\n", __func__, 0);

			spin_unlock_irqrestore(&g_ccorr_clock_lock, flags);
			DDPDBG("%s @ %d......... spin_unlock_irqrestore -- ",
				__func__, __LINE__);
			return;
		}

		{
			/* Disable output frame end interrupt */
			writel(0x0, comp->regs + DISP_REG_CCORR_INTEN);
			DDPINFO("%s: Interrupt disabled\n", __func__);
		}
			spin_unlock_irqrestore(&g_ccorr_clock_lock, flags);
			DDPDBG("%s @ %d......... spin_unlock_irqrestore -- ",
				__func__, __LINE__);
	} else {
		DDPINFO("%s @ %d......... Failed to spin_trylock_irqsave -- ",
			__func__, __LINE__);
	}


}

static irqreturn_t mtk_disp_ccorr_irq_handler(int irq, void *dev_id)
{
	struct mtk_disp_ccorr *priv = dev_id;
	struct mtk_ddp_comp *ccorr = &priv->ddp_comp;

	disp_ccorr_on_end_of_frame(ccorr);
#ifdef CONFIG_MTK_SLD_SUPPORT
	if(g_sld_enable || (g_sld_enable_pre && !g_sld_enable))
		return IRQ_WAKE_THREAD;
#endif
	return IRQ_HANDLED;
}

static irqreturn_t mtk_disp_ccorr_irq_thread(int irq, void *dev_id)
{
	struct mtk_disp_ccorr *priv = dev_id;
	struct mtk_ddp_comp *ccorr = &priv->ddp_comp;

	disp_ccorr_sld_impl(ccorr);

	return IRQ_HANDLED;
}

static int disp_ccorr_wait_irq(struct drm_device *dev, unsigned long timeout)
{
	int ret = 0;

	if (atomic_read(&g_ccorr_get_irq) == 0) {
		DDPDBG("%s: wait_event_interruptible ++ ", __func__);
		ret = wait_event_interruptible(g_ccorr_get_irq_wq,
			atomic_read(&g_ccorr_get_irq) == 1);
		DDPDBG("%s: wait_event_interruptible -- ", __func__);
		DDPINFO("%s: get_irq = 1, waken up", __func__);
		DDPINFO("%s: get_irq = 1, ret = %d", __func__, ret);
	} else {
		/* If g_ccorr_get_irq is already set, */
		/* means PQService was delayed */
		DDPINFO("%s: get_irq = 0", __func__);
	}

	atomic_set(&g_ccorr_get_irq, 0);

	return ret;
}

static int disp_pq_copy_backlight_to_user(int *backlight)
{
	unsigned long flags;
	int ret = 0;

	/* We assume only one thread will call this function */
	spin_lock_irqsave(&g_pq_bl_change_lock, flags);
	g_pq_backlight_db = g_pq_backlight;
	spin_unlock_irqrestore(&g_pq_bl_change_lock, flags);

	memcpy(backlight, &g_pq_backlight_db, sizeof(int));

	DDPINFO("%s: %d\n", __func__, ret);

	return ret;
}

void disp_pq_notify_backlight_changed(int bl_1024)
{
	unsigned long flags;

	spin_lock_irqsave(&g_pq_bl_change_lock, flags);
	g_old_pq_backlight = g_pq_backlight;
	g_pq_backlight = bl_1024;
	spin_unlock_irqrestore(&g_pq_bl_change_lock, flags);

	if (atomic_read(&g_ccorr_is_init_valid) != 1)
		return;

	DDPINFO("%s: %d\n", __func__, bl_1024);

	if (m_new_pq_persist_property[DISP_PQ_CCORR_SILKY_BRIGHTNESS]) {
		if (default_comp != NULL &&
			g_ccorr_relay_value[index_of_ccorr(default_comp->id)] != 1) {
			disp_ccorr_set_interrupt(default_comp, 1);

			if (default_comp != NULL &&
					default_comp->mtk_crtc != NULL)
				mtk_crtc_check_trigger(default_comp->mtk_crtc, false,
					true);

			DDPINFO("%s: trigger refresh when backlight changed", __func__);
		}
	} else {
		if (default_comp != NULL && (g_old_pq_backlight == 0 || bl_1024 == 0)) {
			disp_ccorr_set_interrupt(default_comp, 1);

			if (default_comp != NULL &&
					default_comp->mtk_crtc != NULL)
				mtk_crtc_check_trigger(default_comp->mtk_crtc, false,
					true);

			DDPINFO("%s: trigger refresh when backlight ON/Off", __func__);
		}
	}
}

static int disp_ccorr_set_coef(
	const struct DRM_DISP_CCORR_COEF_T *user_color_corr,
	struct mtk_ddp_comp *comp,
	struct cmdq_pkt *handle)
{
	int ret = 0;
	struct DRM_DISP_CCORR_COEF_T *ccorr, *old_ccorr;
	int id = index_of_ccorr(comp->id);

	ccorr = kmalloc(sizeof(struct DRM_DISP_CCORR_COEF_T), GFP_KERNEL);
	if (ccorr == NULL) {
		DDPPR_ERR("%s: no memory\n", __func__);
		return -EFAULT;
	}

	if (user_color_corr == NULL) {
		ret = -EFAULT;
		kfree(ccorr);
	} else {
		memcpy(ccorr, user_color_corr,
			sizeof(struct DRM_DISP_CCORR_COEF_T));

		if (id >= 0 && id < DISP_CCORR_TOTAL) {
			mutex_lock(&g_ccorr_global_lock);

			old_ccorr = g_disp_ccorr_coef[id];
			g_disp_ccorr_coef[id] = ccorr;
			if ((g_disp_ccorr_coef[id]->offset[0] == 0) &&
				(g_disp_ccorr_coef[id]->offset[1] == 0) &&
				(g_disp_ccorr_coef[id]->offset[2] == 0) &&
				need_offset) {
				DDPINFO("%s:need change offset", __func__);
				g_disp_ccorr_coef[id]->offset[0] =
						(OFFSET_VALUE << 1) << 14;
				g_disp_ccorr_coef[id]->offset[1] =
						(OFFSET_VALUE << 1) << 14;
				g_disp_ccorr_coef[id]->offset[2] =
						(OFFSET_VALUE << 1) << 14;
			}
			DDPINFO("%s: Set module(%d) coef", __func__, id);
			ret = disp_ccorr_write_coef_reg(comp, handle, 0);

			mutex_unlock(&g_ccorr_global_lock);

			if (old_ccorr != NULL)
				kfree(old_ccorr);

			mtk_crtc_check_trigger(comp->mtk_crtc, false, false);
		} else {
			DDPPR_ERR("%s: invalid ID = %d\n", __func__, id);
			ret = -EFAULT;
			kfree(ccorr);
		}
	}

	return ret;
}

static int mtk_disp_ccorr_set_interrupt(struct mtk_ddp_comp *comp, void *data)
{
	int enabled = *((int *)data);
	unsigned long flags;
	int index = index_of_ccorr(comp->id);
	int ret = 0;

	DDPDBG("%s @ %d......... spin_lock_irqsave ++ %d\n", __func__, __LINE__, index);
	spin_lock_irqsave(&g_ccorr_clock_lock, flags);
	DDPDBG("%s @ %d......... spin_lock_irqsave -- ",
		__func__, __LINE__);
	if (atomic_read(&g_ccorr_is_clock_on[index]) != 1) {
		DDPINFO("%s: clock is off. enabled:%d\n",
			__func__, enabled);

		spin_unlock_irqrestore(&g_ccorr_clock_lock, flags);
		DDPDBG("%s @ %d......... spin_unlock_irqrestore -- ",
			__func__, __LINE__);
		return ret;
	}

	if (enabled || g_old_pq_backlight != g_pq_backlight) {
		if (readl(comp->regs + DISP_REG_CCORR_EN) == 0) {
			/* Print error message */
			DDPINFO("[WARNING] DISP_REG_CCORR_EN not enabled!\n");
		}
		/* Enable output frame end interrupt */
		writel(0x2, comp->regs + DISP_REG_CCORR_INTEN);
		DDPINFO("%s: Interrupt enabled\n", __func__);
	} else {
		/* Disable output frame end interrupt when sld end*/
#ifdef CONFIG_MTK_SLD_SUPPORT
		if (sld_end_flag)
#endif
			writel(0x0, comp->regs + DISP_REG_CCORR_INTEN);
		DDPINFO("%s: Interrupt disabled\n", __func__);
	}
	spin_unlock_irqrestore(&g_ccorr_clock_lock, flags);
	DDPDBG("%s @ %d......... spin_unlock_irqrestore -- ",
		__func__, __LINE__);
	return ret;
}
int disp_ccorr_set_color_matrix(struct mtk_ddp_comp *comp,
	struct cmdq_pkt *handle, int32_t matrix[16], int32_t hint, bool fte_flag)
{
	int ret = 0;
	int i, j;
	int ccorr_without_gamma = 0;
	bool need_refresh = false;
	bool identity_matrix = true;
	int id = index_of_ccorr(comp->id);
	struct mtk_drm_crtc *mtk_crtc = comp->mtk_crtc;
	struct drm_crtc *crtc = &mtk_crtc->base;
	struct mtk_drm_private *priv = crtc->dev->dev_private;

	if (handle == NULL) {
		DDPPR_ERR("%s: cmdq can not be NULL\n", __func__);
		return -EFAULT;
	}

	if (g_disp_ccorr_coef[id] == NULL) {
		DDPINFO("%s: can't set matrix for NULL coef", __func__);
		return -EFAULT;
	}

	mutex_lock(&g_ccorr_global_lock);

	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			/* Copy Color Matrix */
			g_ccorr_color_matrix[i][j] = matrix[j*4 + i];

			/* early jump out */
			if (ccorr_without_gamma == 1)
				continue;

			if (i == j && g_ccorr_color_matrix[i][j] != 1024) {
				ccorr_without_gamma = 1;
				identity_matrix = false;
			} else if (i != j && g_ccorr_color_matrix[i][j] != 0) {
				ccorr_without_gamma = 1;
				identity_matrix = false;
			}
		}
	}

	// hint: 0: identity matrix; 1: arbitraty matrix
	// fte_flag: true: gpu overlay && hwc not identity matrix
	// arbitraty matrix maybe identity matrix or color transform matrix;
	// only when set identity matrix and not gpu overlay, open display color
	DDPINFO("hint: %d, identity: %d, fte_flag: %d, bypass: color0:%d color1:%d",
		hint, identity_matrix, fte_flag, bypass_color0, bypass_color1);
	if (((hint == 0) || ((hint == 1) && identity_matrix)) && (!fte_flag)) {
		if (id == 0) {
			if (bypass_color0 == true) {
				struct mtk_ddp_comp *comp_color0 =
					priv->ddp_comp[DDP_COMPONENT_COLOR0];
				ddp_color_bypass_color(comp_color0, false, handle);
				bypass_color0 = false;
			}
		} else if (id == 1) {
			if (bypass_color1 == true) {
				struct mtk_ddp_comp *comp_color1 =
					priv->ddp_comp[DDP_COMPONENT_COLOR1];
				ddp_color_bypass_color(comp_color1, false, handle);
				bypass_color1 = false;
			}
		} else {
			DDPINFO("%s, id is invalid!\n", __func__);
		}
	} else {
		if (id == 0) {
			if (bypass_color0 == false) {
				struct mtk_ddp_comp *comp_color0 =
					priv->ddp_comp[DDP_COMPONENT_COLOR0];
				ddp_color_bypass_color(comp_color0, true, handle);
				bypass_color0 = true;
			}
		} else if (id == 1) {
			if (bypass_color1 == false) {
				struct mtk_ddp_comp *comp_color1 =
					priv->ddp_comp[DDP_COMPONENT_COLOR1];
				ddp_color_bypass_color(comp_color1, true, handle);
				bypass_color1 = true;
			}
		} else {
			DDPINFO("%s, id is invalid!\n", __func__);
		}
	}

#if defined(CONFIG_MACH_MT6885) || defined(CONFIG_MACH_MT6873) \
	|| defined(CONFIG_MACH_MT6893) || defined(CONFIG_MACH_MT6853) \
	|| defined(CONFIG_MACH_MT6833) || defined(CONFIG_MACH_MT6877) \
	|| defined(CONFIG_MACH_MT6781)
	// offset part
	if ((matrix[12] != 0) || (matrix[13] != 0) || (matrix[14] != 0))
		need_offset = true;
	else
		need_offset = false;

	g_disp_ccorr_coef[id]->offset[0] = (matrix[12] << 1) << 14;
	g_disp_ccorr_coef[id]->offset[1] = (matrix[13] << 1) << 14;
	g_disp_ccorr_coef[id]->offset[2] = (matrix[14] << 1) << 14;
#endif

	for (i = 0; i < 3; i += 1) {
		DDPDBG("g_ccorr_color_matrix[%d][0-2] = {%d, %d, %d}\n",
				i,
				g_ccorr_color_matrix[i][0],
				g_ccorr_color_matrix[i][1],
				g_ccorr_color_matrix[i][2]);
	}

	DDPDBG("g_ccorr_color_matrix offset {%d, %d, %d}, hint: %d\n",
		g_disp_ccorr_coef[id]->offset[0],
		g_disp_ccorr_coef[id]->offset[1],
		g_disp_ccorr_coef[id]->offset[2], hint);

	g_disp_ccorr_without_gamma = ccorr_without_gamma;

	disp_ccorr_write_coef_reg(comp, handle, 0);

	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			if (g_ccorr_prev_matrix[i][j]
				!= g_ccorr_color_matrix[i][j]) {
				/* refresh when matrix changed */
				need_refresh = true;
			}
			/* Copy Color Matrix */
			g_ccorr_prev_matrix[i][j] = g_ccorr_color_matrix[i][j];
		}
	}

	for (i = 0; i < 3; i += 1) {
		DDPINFO("g_ccorr_color_matrix[%d][0-2] = {%d, %d, %d}\n",
				i,
				g_ccorr_color_matrix[i][0],
				g_ccorr_color_matrix[i][1],
				g_ccorr_color_matrix[i][2]);
	}

	DDPINFO("g_disp_ccorr_without_gamma: [%d], need_refresh: [%d]\n",
		g_disp_ccorr_without_gamma, need_refresh);

	mutex_unlock(&g_ccorr_global_lock);

	if (need_refresh == true && comp->mtk_crtc != NULL)
		mtk_crtc_check_trigger(comp->mtk_crtc, false, false);

	return ret;
}

int disp_ccorr_set_RGB_Gain(struct mtk_ddp_comp *comp,
	struct cmdq_pkt *handle,
	int r, int g, int b)
{
	int ret;

	mutex_lock(&g_ccorr_global_lock);
	g_rgb_matrix[0][0] = r;
	g_rgb_matrix[1][1] = g;
	g_rgb_matrix[2][2] = b;

	DDPINFO("%s: r[%d], g[%d], b[%d]", __func__, r, g, b);
	ret = disp_ccorr_write_coef_reg(comp, NULL, 0);
	mutex_unlock(&g_ccorr_global_lock);

	return ret;
}

int mtk_drm_ioctl_set_ccorr(struct drm_device *dev, void *data,
		struct drm_file *file_priv)
{
	struct mtk_drm_private *private = dev->dev_private;
	struct mtk_ddp_comp *comp = private->ddp_comp[DDP_COMPONENT_CCORR0];
	struct drm_crtc *crtc = private->crtc[0];

	if (m_new_pq_persist_property[DISP_PQ_CCORR_SILKY_BRIGHTNESS]) {
		int ret;
		struct DRM_DISP_CCORR_COEF_T *ccorr_config = data;

		ret = mtk_crtc_user_cmd(crtc, comp, SET_CCORR, data);

		if ((ccorr_config->silky_bright_flag) == 1 &&
			ccorr_config->FinalBacklight != 0) {
			DDPINFO("brightness = %d, silky_bright_flag = %d",
				ccorr_config->FinalBacklight,
				ccorr_config->silky_bright_flag);
			mt_leds_brightness_set("lcd-backlight",
				ccorr_config->FinalBacklight);
		}

		mtk_crtc_check_trigger(comp->mtk_crtc, false, true);

		return ret;
	} else {
		return mtk_crtc_user_cmd(crtc, comp, SET_CCORR, data);
	}
}

int mtk_drm_ioctl_enable_sld(struct drm_device *dev, void *data,
		struct drm_file *file_priv)
{
	int ret = 0;
	struct mtk_drm_private *private = dev->dev_private;
	struct mtk_ddp_comp *comp = private->ddp_comp[DDP_COMPONENT_CCORR0];
#ifdef CONFIG_MTK_SLD_SUPPORT
	unsigned long flags;

	spin_lock_irqsave(&g_sld_bl_change_lock, flags);
	g_sld_enable_pre = g_sld_enable;
	g_sld_enable = *((bool *)data);
	spin_unlock_irqrestore(&g_sld_bl_change_lock, flags);

	/* set sld_backlight_value by leds API when sld disable */
	/* or set tmp_bl to leds backlight when sld enable */
	if(!g_sld_enable)
		mt_leds_brightness_set("lcd-backlight", sld_backligt_value);
	else
		sld_tmp_bl = sld_backligt_value;
#endif
	disp_ccorr_set_interrupt(comp, 1);
	return ret;

}

int mtk_drm_ioctl_set_sld_param(struct drm_device *dev, void *data,
		struct drm_file *file_priv)
{
	int ret = 0;
	struct mtk_drm_private *private = dev->dev_private;
	struct mtk_ddp_comp *comp = private->ddp_comp[DDP_COMPONENT_CCORR0];

#ifdef CONFIG_MTK_SLD_SUPPORT
	memcpy(&g_sld_param, data, sizeof(struct DISP_SLD_PARAM));
#endif
	mtk_crtc_check_trigger(comp->mtk_crtc, false, true);
	disp_ccorr_set_interrupt(comp, 1);
	return ret;

}

int mtk_drm_ioctl_ccorr_eventctl(struct drm_device *dev, void *data,
		struct drm_file *file_priv)
{
	struct mtk_drm_private *private = dev->dev_private;
	struct mtk_ddp_comp *comp = private->ddp_comp[DDP_COMPONENT_CCORR0];
	int ret = 0;
	/* TODO: dual pipe */
	int *enabled = data;

	if (enabled || g_old_pq_backlight != g_pq_backlight)
		mtk_crtc_check_trigger(comp->mtk_crtc, false, true);

	//mtk_crtc_user_cmd(crtc, comp, EVENTCTL, data);
	DDPINFO("ccorr_eventctl, enabled = %d\n", *enabled);
	disp_ccorr_set_interrupt(comp, *enabled);

	return ret;
}

int mtk_drm_ioctl_ccorr_get_irq(struct drm_device *dev, void *data,
		struct drm_file *file_priv)
{
	int ret = 0;

	atomic_set(&g_ccorr_is_init_valid, 1);

	disp_ccorr_wait_irq(dev, 60);

	if (disp_pq_copy_backlight_to_user((int *) data) < 0) {
		DDPPR_ERR("%s: failed", __func__);
		ret = -EFAULT;
	}

	return ret;
}

int mtk_drm_ioctl_support_color_matrix(struct drm_device *dev, void *data,
		struct drm_file *file_priv)
{
	int ret = 0;
	struct DISP_COLOR_TRANSFORM *color_transform;
	bool support_matrix = true;
	int i;

	if (data == NULL) {
		support_matrix = false;
		ret = -EFAULT;
		DDPINFO("unsupported matrix");
		return ret;
	}

	color_transform = data;

#if defined(CONFIG_MACH_MT6885) || defined(CONFIG_MACH_MT6873) \
	|| defined(CONFIG_MACH_MT6893) || defined(CONFIG_MACH_MT6853) \
	|| defined(CONFIG_MACH_MT6833) || defined(CONFIG_MACH_MT6877) \
	 || defined(CONFIG_MACH_MT6781)
	// Support matrix:
	// AOSP is 4x3 matrix. Offset is located at 4th row (not zero)

	for (i = 0 ; i < 3; i++) {
		if (color_transform->matrix[i][3] != 0) {
			for (i = 0 ; i < 4; i++) {
				DDPINFO("unsupported:[%d][0-3]:[%d %d %d %d]",
					i,
					color_transform->matrix[i][0],
					color_transform->matrix[i][1],
					color_transform->matrix[i][2],
					color_transform->matrix[i][3]);
			}
			support_matrix = false;
			ret = -EFAULT;
			return ret;
		}
	}
	if (support_matrix)
		ret = 0; //Zero: support color matrix.
#else
	// 3x3 support
	for (i = 0 ; i < 3; i++) {
		if (color_transform->matrix[3][i] != 0 ||
			color_transform->matrix[i][3] != 0) {
			DDPINFO("unsupported matrix");
			ret = -EFAULT;
		}
	}
#endif

	return ret;
}


static void mtk_ccorr_config(struct mtk_ddp_comp *comp,
			     struct mtk_ddp_config *cfg,
			     struct cmdq_pkt *handle)
{
	unsigned int width;

	if (comp->mtk_crtc->is_dual_pipe)
		width = cfg->w / 2;
	else
		width = cfg->w;

	DDPINFO("%s\n", __func__);

	cmdq_pkt_write(handle, comp->cmdq_base,
		       comp->regs_pa + DISP_REG_CCORR_SIZE,
		       (width << 16) | cfg->h, ~0);
}

static void mtk_ccorr_start(struct mtk_ddp_comp *comp, struct cmdq_pkt *handle)
{
	DDPINFO("%s\n", __func__);
	disp_ccorr_write_coef_reg(comp, handle, 1);

	cmdq_pkt_write(handle, comp->cmdq_base,
		       comp->regs_pa + DISP_REG_CCORR_EN, 0x1, 0x1);
}

static void mtk_ccorr_bypass(struct mtk_ddp_comp *comp, int bypass,
	struct cmdq_pkt *handle)
{
	if (g_ccorr_relay_value[index_of_ccorr(comp->id)] != bypass) {
		DDPINFO("%s\n", __func__);
		cmdq_pkt_write(handle, comp->cmdq_base,
			       comp->regs_pa + DISP_REG_CCORR_CFG, bypass, 0x1);
		g_ccorr_relay_value[index_of_ccorr(comp->id)] = bypass;
	}
}

static int mtk_ccorr_user_cmd(struct mtk_ddp_comp *comp,
	struct cmdq_pkt *handle, unsigned int cmd, void *data)
{

	DDPINFO("%s: cmd: %d\n", __func__, cmd);

	switch (cmd) {
	case SET_CCORR:
	{
		struct DRM_DISP_CCORR_COEF_T *config = data;

		if (disp_ccorr_set_coef(config,
			comp, handle) < 0) {
			DDPPR_ERR("DISP_IOCTL_SET_CCORR: failed\n");
			return -EFAULT;
		}
		if (comp->mtk_crtc->is_dual_pipe) {
			struct mtk_drm_crtc *mtk_crtc = comp->mtk_crtc;
			struct drm_crtc *crtc = &mtk_crtc->base;
			struct mtk_drm_private *priv = crtc->dev->dev_private;
			struct mtk_ddp_comp *comp_ccorr1 = priv->ddp_comp[DDP_COMPONENT_CCORR1];

			if (disp_ccorr_set_coef(config, comp_ccorr1, handle) < 0) {
				DDPPR_ERR("DISP_IOCTL_SET_CCORR: failed\n");
				return -EFAULT;
			}
		}
	}
	break;

	case SET_INTERRUPT:
	{
		mtk_disp_ccorr_set_interrupt(comp, data);
		if (comp->mtk_crtc->is_dual_pipe) {
			struct mtk_drm_crtc *mtk_crtc = comp->mtk_crtc;
			struct drm_crtc *crtc = &mtk_crtc->base;
			struct mtk_drm_private *priv = crtc->dev->dev_private;
			struct mtk_ddp_comp *comp_ccorr1 = priv->ddp_comp[DDP_COMPONENT_CCORR1];

			mtk_disp_ccorr_set_interrupt(comp_ccorr1, data);
		}
	}
	break;

	case BYPASS_CCORR:
	{
		int *value = data;

		mtk_ccorr_bypass(comp, *value, handle);
	}
	break;

	default:
		DDPPR_ERR("%s: error cmd: %d\n", __func__, cmd);
		return -EINVAL;
	}
	return 0;
}

struct ccorr_backup {
	unsigned int REG_CCORR_CFG;
};
static struct ccorr_backup g_ccorr_backup;

static void ddp_ccorr_backup(struct mtk_ddp_comp *comp)
{
	g_ccorr_backup.REG_CCORR_CFG =
		readl(comp->regs + DISP_REG_CCORR_CFG);
}

static void ddp_ccorr_restore(struct mtk_ddp_comp *comp)
{
	writel(g_ccorr_backup.REG_CCORR_CFG, comp->regs + DISP_REG_CCORR_CFG);
}

static void mtk_ccorr_prepare(struct mtk_ddp_comp *comp)
{
#if defined(CONFIG_DRM_MTK_SHADOW_REGISTER_SUPPORT)
	struct mtk_disp_ccorr *ccorr = comp_to_ccorr(comp);
#endif

	DDPINFO("%s\n", __func__);

	mtk_ddp_comp_clk_prepare(comp);
	atomic_set(&g_ccorr_is_clock_on[index_of_ccorr(comp->id)], 1);

#if defined(CONFIG_DRM_MTK_SHADOW_REGISTER_SUPPORT)
	if (ccorr->data->support_shadow) {
		/* Enable shadow register and read shadow register */
		mtk_ddp_write_mask_cpu(comp, 0x0,
			DISP_REG_CCORR_SHADOW, CCORR_BYPASS_SHADOW);
	} else {
		/* Bypass shadow register and read shadow register */
		mtk_ddp_write_mask_cpu(comp, CCORR_BYPASS_SHADOW,
			DISP_REG_CCORR_SHADOW, CCORR_BYPASS_SHADOW);
	}
#else
#if defined(CONFIG_MACH_MT6873) || defined(CONFIG_MACH_MT6853) \
	|| defined(CONFIG_MACH_MT6833) || defined(CONFIG_MACH_MT6877) \
	|| defined(CONFIG_MACH_MT6781)
	/* Bypass shadow register and read shadow register */
	mtk_ddp_write_mask_cpu(comp, CCORR_BYPASS_SHADOW,
		DISP_REG_CCORR_SHADOW, CCORR_BYPASS_SHADOW);
#endif
#endif
	ddp_ccorr_restore(comp);
}

static void mtk_ccorr_unprepare(struct mtk_ddp_comp *comp)
{
	unsigned long flags;

	disp_ccorr_clear_irq_only(comp);

	DDPINFO("%s @ %d......... spin_lock_irqsave ++ ", __func__, __LINE__);
	spin_lock_irqsave(&g_ccorr_clock_lock, flags);
	DDPINFO("%s @ %d......... spin_lock_irqsave -- ", __func__, __LINE__);
	atomic_set(&g_ccorr_is_clock_on[index_of_ccorr(comp->id)], 0);
	spin_unlock_irqrestore(&g_ccorr_clock_lock, flags);
	DDPDBG("%s @ %d......... spin_unlock_irqrestore ", __func__, __LINE__);
	wake_up_interruptible(&g_ccorr_get_irq_wq); // wake up who's waiting isr
	ddp_ccorr_backup(comp);
	mtk_ddp_comp_clk_unprepare(comp);

	DDPINFO("%s\n", __func__);

}

static const struct mtk_ddp_comp_funcs mtk_disp_ccorr_funcs = {
	.config = mtk_ccorr_config,
	.start = mtk_ccorr_start,
	.bypass = mtk_ccorr_bypass,
	.user_cmd = mtk_ccorr_user_cmd,
	.prepare = mtk_ccorr_prepare,
	.unprepare = mtk_ccorr_unprepare,
};

static int mtk_disp_ccorr_bind(struct device *dev, struct device *master,
			       void *data)
{
	struct mtk_disp_ccorr *priv = dev_get_drvdata(dev);
	struct drm_device *drm_dev = data;
	int ret;

	DDPINFO("%s\n", __func__);

	ret = mtk_ddp_comp_register(drm_dev, &priv->ddp_comp);
	if (ret < 0) {
		dev_err(dev, "Failed to register component %s: %d\n",
			dev->of_node->full_name, ret);
		return ret;
	}

	disp_ccorr_color_matrix_to_dispsys(drm_dev);
	return 0;
}

static void mtk_disp_ccorr_unbind(struct device *dev, struct device *master,
				  void *data)
{
	struct mtk_disp_ccorr *priv = dev_get_drvdata(dev);
	struct drm_device *drm_dev = data;

	mtk_ddp_comp_unregister(drm_dev, &priv->ddp_comp);
}

static const struct component_ops mtk_disp_ccorr_component_ops = {
	.bind	= mtk_disp_ccorr_bind,
	.unbind = mtk_disp_ccorr_unbind,
};

void mtk_ccorr_dump(struct mtk_ddp_comp *comp)
{
	void __iomem *baddr = comp->regs;

	DDPDUMP("== %s REGS ==\n", mtk_dump_comp_str(comp));
	mtk_cust_dump_reg(baddr, 0x0, 0x20, 0x30, -1);
	mtk_cust_dump_reg(baddr, 0x24, 0x28, -1, -1);
}

static int mtk_disp_ccorr_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_disp_ccorr *priv;
	enum mtk_ddp_comp_id comp_id;
	int irq;
	int ret;

	DDPINFO("%s+\n", __func__);

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	comp_id = mtk_ddp_comp_get_id(dev->of_node, MTK_DISP_CCORR);
	if ((int)comp_id < 0) {
		DDPPR_ERR("Failed to identify by alias: %d\n", comp_id);
		return comp_id;
	}

	if (!default_comp && comp_id == DDP_COMPONENT_CCORR0)
		default_comp = &priv->ddp_comp;

	if (!ccorr1_default_comp && comp_id == DDP_COMPONENT_CCORR1)
		ccorr1_default_comp = &priv->ddp_comp;

	ret = mtk_ddp_comp_init(dev, dev->of_node, &priv->ddp_comp, comp_id,
				&mtk_disp_ccorr_funcs);
	if (ret != 0) {
		DDPPR_ERR("Failed to initialize component: %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, priv);

	ret = devm_request_threaded_irq(dev, irq, mtk_disp_ccorr_irq_handler,
					mtk_disp_ccorr_irq_thread,
					IRQF_TRIGGER_NONE | IRQF_SHARED,
					dev_name(dev), priv);

	pm_runtime_enable(dev);

	ret = component_add(dev, &mtk_disp_ccorr_component_ops);
	if (ret != 0) {
		dev_err(dev, "Failed to add component: %d\n", ret);
		pm_runtime_disable(dev);
	}
	DDPINFO("%s-\n", __func__);

	return ret;
}

static int mtk_disp_ccorr_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &mtk_disp_ccorr_component_ops);

	pm_runtime_disable(&pdev->dev);
	return 0;
}

static const struct mtk_disp_ccorr_data mt6779_ccorr_driver_data = {
	.support_shadow = false,
};

static const struct mtk_disp_ccorr_data mt6885_ccorr_driver_data = {
	.support_shadow = false,
};

static const struct mtk_disp_ccorr_data mt6873_ccorr_driver_data = {
	.support_shadow = false,
};

static const struct mtk_disp_ccorr_data mt6853_ccorr_driver_data = {
	.support_shadow = false,
};

static const struct mtk_disp_ccorr_data mt6877_ccorr_driver_data = {
	.support_shadow = false,
};

static const struct mtk_disp_ccorr_data mt6833_ccorr_driver_data = {
	.support_shadow = false,
};

static const struct mtk_disp_ccorr_data mt6781_ccorr_driver_data = {
	.support_shadow = false,
};

static const struct of_device_id mtk_disp_ccorr_driver_dt_match[] = {
	{ .compatible = "mediatek,mt6779-disp-ccorr",
	  .data = &mt6779_ccorr_driver_data},
	{ .compatible = "mediatek,mt6885-disp-ccorr",
	  .data = &mt6885_ccorr_driver_data},
	{ .compatible = "mediatek,mt6873-disp-ccorr",
	  .data = &mt6873_ccorr_driver_data},
	{ .compatible = "mediatek,mt6853-disp-ccorr",
	  .data = &mt6853_ccorr_driver_data},
	{ .compatible = "mediatek,mt6877-disp-ccorr",
	  .data = &mt6877_ccorr_driver_data},
	{ .compatible = "mediatek,mt6833-disp-ccorr",
	  .data = &mt6833_ccorr_driver_data},
	{ .compatible = "mediatek,mt6781-disp-ccorr",
	  .data = &mt6781_ccorr_driver_data},
	{},
};

MODULE_DEVICE_TABLE(of, mtk_disp_ccorr_driver_dt_match);

struct platform_driver mtk_disp_ccorr_driver = {
	.probe = mtk_disp_ccorr_probe,
	.remove = mtk_disp_ccorr_remove,
	.driver = {

			.name = "mediatek-disp-ccorr",
			.owner = THIS_MODULE,
			.of_match_table = mtk_disp_ccorr_driver_dt_match,
		},
};

void disp_ccorr_set_bypass(struct drm_crtc *crtc, int bypass)
{
	int ret;

	if (g_ccorr_relay_value[index_of_ccorr(default_comp->id)] == bypass)
		return;
	ret = mtk_crtc_user_cmd(crtc, default_comp, BYPASS_CCORR, &bypass);
	if (default_comp->mtk_crtc->is_dual_pipe)
		ret = mtk_crtc_user_cmd(crtc, ccorr1_default_comp, BYPASS_CCORR, &bypass);
	DDPFUNC("ret = %d", ret);
}
