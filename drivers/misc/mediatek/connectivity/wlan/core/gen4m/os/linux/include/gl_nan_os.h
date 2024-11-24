/* SPDX-License-Identifier: GPL-2.0 */

/*
 * Copyright (c) 2020 MediaTek Inc.
 */
/*
 * Id:
 * //Department/DaVinci/TRUNK/MT6620_5931_WiFi_Driver/os/linux/
 * include/gl_p2p_os.h#28
 */

/*! \file   gl_nan_os.h
 *    \brief  List the external reference to OS for nan GLUE Layer.
 *
 *    In this file we define the data structure - GLUE_INFO_T to store
 *    those objects
 *    we acquired from OS - e.g. TIMER, SPINLOCK, NET DEVICE ... . And all the
 *    external reference (header file, extern func() ..) to OS for GLUE Layer
 *    should also list down here.
 */

#ifndef _GL_NAN_OS_H
#define _GL_NAN_OS_H

/*******************************************************************************
 *                         C O M P I L E R   F L A G S
 *******************************************************************************
 */
#if CFG_SUPPORT_NAN

/*******************************************************************************
 *                    E X T E R N A L   R E F E R E N C E S
 *******************************************************************************
 */

/*******************************************************************************
 *                    E X T E R N A L   V A R I A B L E
 *******************************************************************************
 */

/*******************************************************************************
 *                              C O N S T A N T S
 *******************************************************************************
 */

/*******************************************************************************
 *                                 M A C R O S
 *******************************************************************************
 */

/*******************************************************************************
 *                             D A T A   T Y P E S
 *******************************************************************************
 */

/*******************************************************************************
 *                            P U B L I C   D A T A
 *******************************************************************************
 */

/*******************************************************************************
 *                           P R I V A T E   D A T A
 *******************************************************************************
 */

/*******************************************************************************
 *                  F U N C T I O N   D E C L A R A T I O N S
 *******************************************************************************
 */

struct _GL_NAN_INFO_T {
	struct net_device *prDevHandler;
	unsigned char fgBMCFilterSet;
	/* struct net_device *prRoleDevHandler; */
};

/*******************************************************************************
 *                            P U B L I C   D A T A
 *******************************************************************************
 */

/*******************************************************************************
 *                           P R I V A T E   D A T A
 *******************************************************************************
 */
int mtk_nan_wext_get_priv(IN struct net_device *prDev,
			  IN struct iw_request_info *info,
			  IN OUT union iwreq_data *wrqu, IN OUT char *extra);

unsigned char nanLaunch(struct GLUE_INFO *prGlueInfo);

unsigned char nanRemove(struct GLUE_INFO *prGlueInfo);
void nanSetSuspendMode(struct GLUE_INFO *prGlueInfo, unsigned char fgEnable);

unsigned char glRegisterNAN(struct GLUE_INFO *prGlueInfo,
	const char *prDevName);

int glSetupNAN(struct GLUE_INFO *prGlueInfo, struct wireless_dev *prNanWdev,
	       struct net_device *prNanDev, int u4Idx);

unsigned char glUnregisterNAN(struct GLUE_INFO *prGlueInfo);

unsigned char nanNetRegister(struct GLUE_INFO *prGlueInfo,
		       unsigned char fgIsRtnlLockAcquired);

unsigned char nanNetUnregister(struct GLUE_INFO *prGlueInfo,
			 unsigned char fgIsRtnlLockAcquired);

unsigned char nanAllocInfo(IN struct GLUE_INFO *prGlueInfo, uint8_t ucRoleIdx);
unsigned char nanFreeInfo(struct GLUE_INFO *prGlueInfo, uint8_t ucRoleIdx);

/* VOID p2pSetSuspendMode(P_GLUE_INFO_T prGlueInfo, BOOLEAN fgEnable); */
unsigned char glNanCreateWirelessDevice(struct GLUE_INFO *prGlueInfo);
void nanSetMulticastListWorkQueueWrapper(struct GLUE_INFO *prGlueInfo);

/* VOID p2pUpdateChannelTableByDomain(P_GLUE_INFO_T prGlueInfo); */
#endif
#endif
