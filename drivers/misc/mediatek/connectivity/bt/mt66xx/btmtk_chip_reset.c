/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */


#include "btmtk_main.h"
#include "btmtk_woble.h"

void btmtk_reset_waker(struct work_struct *work)
{
	struct btmtk_dev *bdev = container_of(work, struct btmtk_dev, reset_waker);
	struct btmtk_cif_state *cif_state = NULL;
	struct btmtk_main_info *bmain_info = btmtk_get_main_info();
	int cif_event = 0, err = 0;

	cif_event = HIF_EVENT_SUBSYS_RESET;
	if (BTMTK_CIF_IS_NULL(bdev, cif_event)) {
		/* Error */
		BTMTK_WARN("%s priv setting is NULL", __func__);
		goto Finish;
	}

	while (!bdev->bt_cfg.support_dongle_reset) {
		BTMTK_ERR("%s chip_reset is not support", __func__);
		msleep(2000);
	}

	cif_state = &bdev->cif_state[cif_event];

	/* Set Entering state */
	btmtk_set_chip_state((void *)bdev, cif_state->ops_enter);

	BTMTK_INFO("%s: Receive a byte (0xFF)", __func__);
	/* read interrupt EP15 CR */

	bdev->subsys_reset = 1;
	bdev->sco_num = 0;

	if (bmain_info->whole_reset_flag == 0) {
		if (bmain_info->hif_hook.subsys_reset)
			err = bmain_info->hif_hook.subsys_reset(bdev);
		else
			BTMTK_INFO("%s: Not support subsys chip reset", __func__);
	} else {
		err = -1;
		BTMTK_INFO("%s: whole_reset_flag is %d", __func__, bmain_info->whole_reset_flag);
	}

	if (err) {
		/* L0.5 reset failed, do whole chip reset */
		/* We will add support dongle reset flag, reading from bt.cfg */
		bdev->subsys_reset = 0;
		/* TODO: need to confirm with usb host when suspend fail, to do chip reset,
		 * because usb3.0 need to toggle reset pin after hub_event unfreeze,
		 * otherwise, it will not occur disconnect on Capy Platform. When Mstar
		 * chip has usb3.0 port, we will use Mstar platform to do comparison
		 * test, then found the final solution.
		 */
		/* msleep(2000); */
		if (bmain_info->hif_hook.whole_reset)
			bmain_info->hif_hook.whole_reset(bdev);
		else
			BTMTK_INFO("%s: Not support whole chip reset", __func__);
		bmain_info->whole_reset_flag = 0;
		goto Finish;
	}

	/* It's a test code for stress test (whole chip reset & L0.5 reset) */
#if 0
	if (bdev->bt_cfg.support_dongle_reset == 0) {
		err = btmtk_cif_subsys_reset(bdev);
		if (err) {
			/* L0.5 reset failed, do whole chip reset */
			if (main_info.hif_hook->whole_reset)
				main_info.hif_hook.whole_reset(bdev);
			goto Finish;
		}
	} else {
		/* L0.5 reset failed, do whole chip reset */
		/* TODO: need to confirm with usb host when suspend fail, to do chip reset,
		 * because usb3.0 need to toggle reset pin after hub_event unfreeze,
		 * otherwise, it will not occur disconnect on Capy Platform. When Mstar
		 * chip has usb3.0 port, we will use Mstar platform to do comparison
		 * test, then found the final solution.
		 */
		/* msleep(2000); */
		if (main_info.hif_hook->whole_reset)
			main_info.hif_hook.whole_reset(bdev);
		/* btmtk_send_hw_err_to_host(bdev); */
		goto Finish;
	}
#endif

	bmain_info->reset_stack_flag = HW_ERR_CODE_CHIP_RESET;
	bdev->subsys_reset = 0;

	err = btmtk_cap_init(bdev);
	if (err < 0) {
		BTMTK_ERR("btmtk init failed!");
		goto Finish;
	}

	err = btmtk_load_rom_patch(bdev);
	if (err < 0) {
		BTMTK_ERR("btmtk load rom patch failed!");
		goto Finish;
	}
	btmtk_send_hw_err_to_host(bdev);
	btmtk_woble_wake_unlock(bdev);

Finish:
	bmain_info->hif_hook.chip_reset_notify(bdev);

	/* Set End/Error state */
	if (err < 0)
		btmtk_set_chip_state((void *)bdev, cif_state->ops_error);
	else
		btmtk_set_chip_state((void *)bdev, cif_state->ops_end);
}

