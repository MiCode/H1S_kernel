// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2023 AWINIC Technology CO., LTD.All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of  the GNU General  Public License as published by the Free
 * Software Foundation; either version 2 of the  License, or (at your option)
 * any later version.
 ******************************************************************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/printk.h>
#include <linux/device.h>
#include <linux/slab.h>
#include "lens_info.h"
#include "AW8601xAF.h"

#define AW8601xAF_SLAVE_ADDR		(0x18) /* 0x0c: 0x18 >> 1 */
#define AW8601xAF_DRIVER_VERSION	"v1.2.0"

static struct i2c_client *g_pstAF_I2Cclient;
static int *g_pAF_Opened;
static spinlock_t *g_pAF_SpinLock;
static unsigned char g_motor_type;

static unsigned long g_u4CurrPosition;
static unsigned long g_u4AF_MACRO = 1023;
static unsigned long g_u4AF_INF;

/*******************************************************************************
 * I2c read/write
 ******************************************************************************/
static int AW8601xAF_WriteRegs(unsigned char a_uAddr, unsigned char *a_uData, unsigned int len)
{
	int ret = 0;
	unsigned char *buf = NULL;

	buf = kmalloc(len + 1, GFP_KERNEL);
	if (buf == NULL) {
		AW_LOGE("allocate memory error.");
		return -ENOMEM;
	}
	memset(buf, 0, len + 1);
	buf[0] = a_uAddr;
	memcpy(&buf[1], a_uData, len);

	g_pstAF_I2Cclient->addr = (AW8601xAF_SLAVE_ADDR >> 1);
	ret = i2c_master_send(g_pstAF_I2Cclient, buf, len + 1);
	if (ret < 0)
		AW_LOGE("Send data err, ret = %d.", ret);

	kfree(buf);
	buf = NULL;

	return ret;
}

static int AW8601xAF_ReadRegs(unsigned char a_uAddr, unsigned char *a_puData, unsigned int data_len)
{
	int ret = 0;
	struct i2c_msg msg[2];
	int msg_num = 0;

	msg_num = ARRAY_SIZE(msg);

	g_pstAF_I2Cclient->addr = (AW8601xAF_SLAVE_ADDR >> 1);

	msg[0].addr = g_pstAF_I2Cclient->addr;
	msg[0].flags = 0;
	msg[0].len = sizeof(unsigned char);
	msg[0].buf = &a_uAddr;

	msg[1].addr = g_pstAF_I2Cclient->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = data_len;
	msg[1].buf = a_puData;

	ret = i2c_transfer(g_pstAF_I2Cclient->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0) {
		AW_LOGE("i2c transfer error, ret: %d.", ret);
		return ret;
	} else if (ret != msg_num) {
		AW_LOGE("i2c transfer error(size error), ret: %d.", ret);
		return -EAGAIN;
	}

	return AW_SUCCESS;
}

static inline int AW8601xAF_SoftReset(void)
{
	uint8_t val = 0;
	int ret = 0;

	val = AW_SHUTDOWN;
	ret = AW8601xAF_WriteRegs(AW_REG_CONTROL, &val, AW_DATA_BYTE_1);
	if (ret < 0) {
		AW_LOGE("Shut down error, ret: %d.", ret);
		return -EAGAIN;
	}
	val = AW_WAKEUP;
	ret = AW8601xAF_WriteRegs(AW_REG_CONTROL, &val, AW_DATA_BYTE_1);
	if (ret < 0) {
		AW_LOGE("Wake up error, ret: %d.", ret);
		return -EAGAIN;
	}

	usleep_range(AW_RESET_DELAY_MIN, AW_RESET_DELAY_MAX);

	return ret;
}

static inline int AW8601xAF_CfgCurrentGears(unsigned char chip_id)
{
	uint8_t gears = 0;

	if (chip_id == AW86017_CHIPID) {
		/* the maximum output current gear is defined by product requirements */
		gears = AW86017_CURRENT_120MA;
		AW_LOGD("Current gears config 150mA, val: 0x%02x", gears);
		AW8601xAF_WriteRegs(AW_REG_IMAX, &gears, AW_DATA_BYTE_1);
	} else {
		AW_LOGD("This chip is not support current gears config.");
	}

	AW_LOGD("Current gears config OK, 0x%02x", gears);

	return AW_SUCCESS;
}

static inline int AW8601xAF_Middle_to_Buttom(void)
{
	uint8_t datavalue = 0;
	int ret = 0;

	AW_LOGI("Start.");
	datavalue = 0x5A;
	ret |= AW8601xAF_WriteRegs(0x11, &datavalue, AW_DATA_BYTE_1);

	ret |= AW8601xAF_ReadRegs(0x22, &datavalue, AW_DATA_BYTE_1);
	ret |= AW8601xAF_WriteRegs(0x13, &datavalue, AW_DATA_BYTE_1);

	ret |= AW8601xAF_ReadRegs(0x23, &datavalue, AW_DATA_BYTE_1);
	datavalue = datavalue & 0xFE;
	ret |= AW8601xAF_WriteRegs(0x14, &datavalue, AW_DATA_BYTE_1);

	ret |= AW8601xAF_ReadRegs(0x24, &datavalue, AW_DATA_BYTE_1);
	datavalue = datavalue & 0xFE;
	ret |= AW8601xAF_WriteRegs(0x15, &datavalue, AW_DATA_BYTE_1);

	ret |= AW8601xAF_ReadRegs(0x25, &datavalue, AW_DATA_BYTE_1);
	datavalue = datavalue & 0x3E;
	ret |= AW8601xAF_WriteRegs(0x16, &datavalue, AW_DATA_BYTE_1);

	ret |= AW8601xAF_ReadRegs(0x26, &datavalue, AW_DATA_BYTE_1);
	datavalue = datavalue & 0x1F;
	ret |= AW8601xAF_WriteRegs(0x17, &datavalue, AW_DATA_BYTE_1);

	datavalue = 0x80;
	ret |= AW8601xAF_WriteRegs(0x12, &datavalue, AW_DATA_BYTE_1);

	AW_LOGI("End.");
	return ret;
}

static inline int AW8601xAF_SetAlgoMode(void)
{
	uint8_t mode_cmd[] = { 0xed, 0xab };
	uint8_t chip_id = 0;
	uint8_t algo_mode = 0;
	uint8_t div = 0;
	uint8_t vrct = 0;
	uint8_t type = 0;
	uint8_t ctl_val = 0;
	uint8_t algo_cfg[3] = { 0 };
	int ret = 0;

	AW_LOGI("Start.");
	/* enter advance mode */
	ret = AW8601xAF_WriteRegs(mode_cmd[0], &mode_cmd[1], AW_DATA_BYTE_1);
	if (ret < 0) {
		AW_LOGE("Enter advance mode failed, ret: %d.", ret);
		return -EAGAIN;
	}

	/* Identification IC */
	ret = AW8601xAF_ReadRegs(AW_REG_CHIP_ID, &chip_id, AW_DATA_BYTE_1);
	if (ret < 0) {
		AW_LOGE("Read chipid error, ret: %d.", ret);
		return -EAGAIN;
	}

	switch (chip_id) {
	case AW8601_CHIPID: /* Mid-mounted motor */
		type = AW_MID_MOUNTED_MOTOR;
		algo_mode = AW8601_ALGO_MODE;
		div = (AW8601_DIV_H << 2) | AW8601_DIV_L;
		vrct = AW8601_VRCT;
		break;
	case AW86016_CHIPID: /* Mid-mounted motor */
		type = AW_MID_MOUNTED_MOTOR;
		algo_mode = AW86016_ALGO_MODE;
		div = (AW86016_DIV_H << 2) | AW86016_DIV_L;
		vrct = AW86016_VRCT;
		break;
	case AW86014_CHIPID: /* Bottom motor */
		type = AW_BOTTOM_MOTOR;
		algo_mode = AW86014_ALGO_MODE;
		div = AW86014_DIV;
		vrct = AW86014_VRCT;
		break;
	case AW86017_CHIPID: /* Bottom motor */
		type = AW_BOTTOM_MOTOR;
		algo_mode = AW86017_ALGO_MODE;
		div = AW86017_DIV;
		vrct = AW86017_VRCT;
		break;
	default:
		AW_LOGE("Chip id match failed, Chip ID: 0x%02x", chip_id);
		break;
	}

	ret = AW8601xAF_SoftReset();
	if (ret < 0) {
		AW_LOGE("Soft reset error, ret: %d.", ret);
		return -EAGAIN;
	}

	if (type == AW_BOTTOM_MOTOR) {
		algo_cfg[0] = (AW86014_RING << 7) | algo_mode;
		algo_cfg[1] = div;
		algo_cfg[2] = vrct;

		ret = AW8601xAF_WriteRegs(AW_REG_ALG_MODE, algo_cfg, AW_DATA_BYTE_3);
		if (ret < 0) {
			AW_LOGE("Set algo_cfg error, ret: %d", ret);
			return -EAGAIN;
		}
	} else if (type == AW_MID_MOUNTED_MOTOR) {
		ret = AW8601xAF_ReadRegs(AW_REG_CONTROL, &ctl_val, AW_DATA_BYTE_1);
		if (ret < 0) {
			AW_LOGE("Read ctl_val error, ret: %d", ret);
			return -EAGAIN;
		}
		ctl_val &= 0xfd; /* clear RING flag */
		ctl_val |= (AW86016_RING << 1);
		ret = AW8601xAF_WriteRegs(AW_REG_CONTROL, &ctl_val, AW_DATA_BYTE_1);
		if (ret < 0) {
			AW_LOGE("Set ctl_val error, ret: %d", ret);
			return -EAGAIN;
		}
		algo_cfg[0] = (algo_mode << 6) | (div >> 2);
		algo_cfg[1] = (div << 6) | vrct;

		ret = AW8601xAF_WriteRegs(AW_REG_ALG_MODE, algo_cfg, AW_DATA_BYTE_2);
		if (ret < 0) {
			AW_LOGE("Set algo_cfg error, ret: %d", ret);
			return -EAGAIN;
		}
	}

#if AW_CURRENT_GEARS_CFG_EN
	AW8601xAF_CfgCurrentGears(chip_id);
#endif
	g_motor_type = type;

	AW_LOGI("End.");

	return AW_SUCCESS;
}

static inline int AW8601xAF_InitDrv(void)
{
	int ret = 0;

	AW_LOGI("Start.");

	if (*g_pAF_Opened == 1) {
		ret = AW8601xAF_SetAlgoMode();
		if (ret < 0) {
			AW_LOGE("Set algo mode error.");
			return -EAGAIN;
		}

		spin_lock(g_pAF_SpinLock);
		*g_pAF_Opened = 2;
		spin_unlock(g_pAF_SpinLock);
	} else if (*g_pAF_Opened == 0) {
		AW_LOGE("Please open device.");
		return -EAGAIN;
	}

	AW_LOGI("Init drv OK!.");

	return AW_SUCCESS;
}

static int AW8601xAF_SetPos(unsigned long a_u4Position)
{
	int ret = 0;
	unsigned char pos[3] = { 0 };

	AW_LOGI("Start.");

	pos[0] = AW_REG_CODE_H; /* AW_REG_CODE_H: 0x03 */
	pos[1] = (unsigned char)((a_u4Position >> 8) & 0x03);
	pos[2] = (unsigned char)(a_u4Position & 0xff);

	AW_LOGI("Target Position = 0x%04lx(%d).", a_u4Position, a_u4Position);

	g_pstAF_I2Cclient->addr = (AW8601xAF_SLAVE_ADDR >> 1); /* 0x18 >> 1 */
	ret = i2c_master_send(g_pstAF_I2Cclient, &pos[0], AW_DATA_BYTE_3);
	if (ret < 0) {
		AW_LOGE("Set position err, ret = %d.", ret);
		return ret;
	}

	AW_LOGI("End.");

	return AW_SUCCESS;
}

static inline int AW8601xAF_MoveVCM(unsigned long a_u4Position)
{
	AW_LOGI("Start.");

	if ((a_u4Position > g_u4AF_MACRO) || (a_u4Position < g_u4AF_INF)) {
		AW_LOGE("Target position out of range.");
		return -EINVAL;
	}

	if (*g_pAF_Opened == 1) {
		spin_lock(g_pAF_SpinLock);
		*g_pAF_Opened = 2;
		spin_unlock(g_pAF_SpinLock);
	} else if (*g_pAF_Opened == 0) {
		AW_LOGE("Please open device.");
		return -EIO;
	}

	AW_LOGD("*g_pAF_Opened = %d.", *g_pAF_Opened);

	if (*g_pAF_Opened == 2) {
		if (AW8601xAF_SetPos(a_u4Position) == 0) {
			spin_lock(g_pAF_SpinLock);
			g_u4CurrPosition = a_u4Position;
			spin_unlock(g_pAF_SpinLock);
		} else {
			AW_LOGE("Move vcm error!.");
			return -EAGAIN;
		}
	}
#ifdef AW8601xAF_Middle_to_Buttom
	ret = AW8601xAF_Middle_to_Buttom();
	if (ret < 0)
		AW_LOGE("change to buttom mode error, ret: %d", ret);
#endif
	AW_LOGI("End.");

	return AW_SUCCESS;
}

static inline int AW8601xAF_SetMacro(unsigned long a_u4Param)
{
	AW_LOGI("Start.");

	if ((a_u4Param > AW_LIMITPOS_MAX) || (a_u4Param < AW_LIMITPOS_MIN)) {
		AW_LOGE("a_u4Param out of range.");
		return -EINVAL;
	}

	spin_lock(g_pAF_SpinLock);
	g_u4AF_MACRO = a_u4Param;
	spin_unlock(g_pAF_SpinLock);

	AW_LOGD("g_u4AF_MACRO = 0x%04lx.", g_u4AF_MACRO);

	return AW_SUCCESS;
}

static inline int AW8601xAF_SetInf(unsigned long a_u4Param)
{
	AW_LOGI("Start.");

	if ((a_u4Param > AW_LIMITPOS_MAX) || (a_u4Param < AW_LIMITPOS_MIN)) {
		AW_LOGE("a_u4Param out of range.");
		return -EINVAL;
	}

	spin_lock(g_pAF_SpinLock);
	g_u4AF_INF = a_u4Param;
	spin_unlock(g_pAF_SpinLock);

	AW_LOGD("g_u4AF_INF = 0x%04lx.", g_u4AF_INF);

	return AW_SUCCESS;
}

static inline int AW8601xAF_GetVCMInfo(__user struct stAF_MotorInfo *pstMotorInfo)
{
	struct stAF_MotorInfo stMotorInfo;

	AW_LOGI("Start.");

	stMotorInfo.u4MacroPosition = g_u4AF_MACRO;
	stMotorInfo.u4InfPosition = g_u4AF_INF;
	stMotorInfo.u4CurrentPosition = g_u4CurrPosition;
	stMotorInfo.bIsSupportSR = 1;

	stMotorInfo.bIsMotorMoving = 1;

	if (*g_pAF_Opened >= 1)
		stMotorInfo.bIsMotorOpen = 1;
	else
		stMotorInfo.bIsMotorOpen = 0;

	if (copy_to_user(pstMotorInfo, &stMotorInfo, sizeof(struct stAF_MotorInfo)))
		AW_LOGE("Get VCM information error!.");

	return AW_SUCCESS;
}

long AW8601xAF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command, unsigned long a_u4Param)
{
	long ret = 0;

	AW_LOGI("Start.");

	switch (a_u4Command) {
	case AFIOC_G_MOTORINFO:
		ret = AW8601xAF_GetVCMInfo((__user struct stAF_MotorInfo *) (a_u4Param));
		break;
	case AFIOC_T_MOVETO:
		ret = AW8601xAF_MoveVCM(a_u4Param);
		break;
	case AFIOC_T_SETMACROPOS:
		ret = AW8601xAF_SetMacro(a_u4Param);
		break;
	case AFIOC_T_SETINFPOS:
		ret = AW8601xAF_SetInf(a_u4Param);
		break;
	default:
		AW_LOGE("NO CMD!.");
		ret = -EPERM;
		break;
	}

	AW_LOGI("End.");

	return ret;
}

int AW8601xAF_Release(struct inode *a_pInode, struct file *a_pstFile)
{
	int ret = 0;

	AW_LOGI("Start.");

	if (*g_pAF_Opened == 2) {
		unsigned long pos;

		if (g_motor_type == AW_MID_MOUNTED_MOTOR)
			pos = (unsigned long)((AW_MID_INIT_POS_H << 8) | AW_MID_INIT_POS_L);
		if (g_motor_type == AW_BOTTOM_MOTOR) {
			pos = (unsigned long)((AW_BOTTOM_INIT_POS_H << 8) | AW_BOTTOM_INIT_POS_L);
#ifdef RELEASE_FADE_OUT
			/* Fade out processing */
			if (g_u4CurrPosition > (pos + 600)) {
				AW8601xAF_SetPos(pos + 600);
				usleep_range(5000, 5500);
			}
			if (g_u4CurrPosition > (pos + 400)) {
				AW8601xAF_SetPos(pos + 400);
				usleep_range(5000, 5500);
			}
			if (g_u4CurrPosition > (pos + 200)) {
				AW8601xAF_SetPos(pos + 200);
				usleep_range(5000, 5500);
			}
			if (g_u4CurrPosition > (pos + 100)) {
				AW8601xAF_SetPos(pos + 100);
				usleep_range(5000, 5500);
			}
			if (g_u4CurrPosition > (pos + 50)) {
				AW8601xAF_SetPos(pos + 50);
				usleep_range(5000, 5500);
			}
#endif
		}

		ret = AW8601xAF_SetPos(pos);
		if (ret != AW_SUCCESS)
			AW_LOGE("Set AF target position failed, ret: %d", ret);
	}

	if (*g_pAF_Opened) {
		spin_lock(g_pAF_SpinLock);
		*g_pAF_Opened = 0;
		spin_unlock(g_pAF_SpinLock);

		AW_LOGI("VCM free.");
	}

	AW_LOGI("End.");

	return AW_SUCCESS;
}

int AW8601xAF_PowerDown(struct i2c_client *pstAF_I2Cclient, int *pAF_Opened)
{
	unsigned char mode_cmd[] = { 0xed, 0xab };
	uint8_t pd_cmd[] = { 0x02, 0x01 };
	int ret = 0;

	AW_LOGI("Start.");

	g_pstAF_I2Cclient = pstAF_I2Cclient;
	g_pAF_Opened = pAF_Opened;

	if (*g_pAF_Opened > 0)
		*g_pAF_Opened = 0;
	/* enter advance mode */
	ret = AW8601xAF_WriteRegs(mode_cmd[0], &mode_cmd[1], AW_DATA_BYTE_1);
	if (ret < 0) {
		AW_LOGE("Enter advance mode error, ret: %d.", ret);
		return -EAGAIN;
	}

	/* enter PD mode, PD = 1; */
	ret = AW8601xAF_WriteRegs(pd_cmd[0], &pd_cmd[1], AW_DATA_BYTE_1);
	if (ret < 0) {
		AW_LOGE("Set PD mode error, ret: %d.", ret);
		return -EAGAIN;
	}

	AW_LOGI("Set PD mode OK.");

	return AW_SUCCESS;
}

int AW8601xAF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient, spinlock_t *pAF_SpinLock, int *pAF_Opened)
{
	int ret = 0;
	int loop = 0;

	AW_LOGI("Start.");

	if ((!pstAF_I2Cclient) || (!pAF_SpinLock) || (!pAF_Opened)) {
		AW_LOGE("Pointer is NULL");
		return -EINVAL;
	}

	g_pstAF_I2Cclient = pstAF_I2Cclient;
	g_pAF_SpinLock = pAF_SpinLock;
	g_pAF_Opened = pAF_Opened;

	do {
		ret = AW8601xAF_InitDrv();
		if (ret == AW_SUCCESS)
			break;
		AW_LOGD("InitDrv failed, ret: %d, loop: %d", ret, loop);
		usleep_range(AW_INIT_ERROR_DELAY_MIN, AW_INIT_ERROR_DELAY_MAX);
	} while ((++loop) < AW_EOOR_LOOP);
	if (loop >= AW_EOOR_LOOP) {
		AW_LOGE("InitDrv error.");
		return AW_ERROR;
	}

	AW_LOGI("End.");

	return 1;
}

int AW8601xAF_GetFileName(uint8_t *pFileName)
{
	#if SUPPORT_GETTING_LENS_FOLDER_NAME
	char FilePath[256];
	char *FileString;

	sprintf(FilePath, "%s", __FILE__);
	FileString = strrchr(FilePath, '/');
	*FileString = '\0';
	FileString = (strrchr(FilePath, '/') + 1);
	strncpy(pFileName, FileString, AF_MOTOR_NAME);
	AW_LOGI("FileName : %s", pFileName);
	#else
	pFileName[0] = '\0';
	#endif
	return 1;
}
MODULE_AUTHOR("<zhangzetao@awinic.com>");
MODULE_DESCRIPTION("AW8601xAF VCM Driver");
MODULE_LICENSE("GPL v2");
