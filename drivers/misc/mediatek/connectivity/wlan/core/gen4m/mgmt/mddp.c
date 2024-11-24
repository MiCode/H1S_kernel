/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */
/*
** Id: @(#) mddp.c@@
*/

/*! \file   mddp.c
*    \brief  Main routines for modem direct path handling
*
*    This file contains the support routines of modem direct path operation.
*/


/*******************************************************************************
*                         C O M P I L E R   F L A G S
********************************************************************************
*/

/*******************************************************************************
*                    E X T E R N A L   R E F E R E N C E S
********************************************************************************
*/
#include "precomp.h"

#if CFG_MTK_MDDP_SUPPORT

#include "gl_os.h"
#include "mddp_export.h"
#include "mddp.h"

/*******************************************************************************
*                              C O N S T A N T S
********************************************************************************
*/

/*******************************************************************************
*                   F U N C T I O N   D E C L A R A T I O N S
********************************************************************************
*/

/*******************************************************************************
*                            P U B L I C   D A T A
********************************************************************************
*/
enum ENUM_MDDPW_DRV_INFO_STATUS {
	MDDPW_DRV_INFO_STATUS_ON_START   = 0,
	MDDPW_DRV_INFO_STATUS_ON_END     = 1,
	MDDPW_DRV_INFO_STATUS_OFF_START  = 2,
	MDDPW_DRV_INFO_STATUS_OFF_END    = 3,
	MDDPW_DRV_INFO_STATUS_ON_END_QOS = 4,
};

enum ENUM_MDDPW_MD_INFO {
	MDDPW_MD_INFO_RESET_IND     = 1,
	MDDPW_MD_INFO_DRV_EXCEPTION = 2,
};

/* MDDPW_MD_INFO_DRV_EXCEPTION */
struct wsvc_md_event_exception_t {
	uint32_t u4RstReason;
	uint32_t u4RstFlag;
	uint32_t u4Line;
	char pucFuncName[64];
};

struct mddpw_drv_handle_t gMddpWFunc = {
	.notify_md_info = mddpMdNotifyInfo,
};

struct mddp_drv_conf_t gMddpDrvConf = {
	.app_type = MDDP_APP_TYPE_WH,
};

struct mddp_drv_handle_t gMddpFunc = {
	.change_state = mddpChangeState,
};

/*******************************************************************************
*                           P R I V A T E   D A T A
********************************************************************************
*/
#define MAC_ADDR_LEN            6

struct mddp_txd_t {
	uint8_t version;
	uint8_t wlan_idx;
	uint8_t sta_idx;
	uint8_t nw_if_name[8];
	uint8_t sta_mode;
	uint8_t bss_id;
	uint8_t wmmset;
	uint8_t aucMacAddr[MAC_ADDR_LEN];
	uint8_t local_mac[MAC_ADDR_LEN];
	uint8_t txd_length;
	uint8_t txd[0];
} __packed;

struct tag_bootmode {
	u32 size;
	u32 tag;
	u32 bootmode;
	u32 boottype;
};

enum BOOTMODE {
	NORMAL_BOOT = 0,
	META_BOOT = 1,
	RECOVERY_BOOT = 2,
	SW_REBOOT = 3,
	FACTORY_BOOT = 4,
	ADVMETA_BOOT = 5,
	ATE_FACTORY_BOOT = 6,
	ALARM_BOOT = 7,
	KERNEL_POWER_OFF_CHARGING_BOOT = 8,
	LOW_POWER_OFF_CHARGING_BOOT = 9,
	FASTBOOT = 99,
	DOWNLOAD_BOOT = 100,
	UNKNOWN_BOOT
};

enum BOOTMODE g_wifi_boot_mode = NORMAL_BOOT;
u_int8_t g_fgMddpEnabled = TRUE;
struct MDDP_SETTINGS g_rSettings;

struct mddpw_net_stat_ext_t stats;

/*******************************************************************************
*                              F U N C T I O N S
********************************************************************************
*/

static bool wait_for_md_on_complete(void);
static bool wait_for_md_off_complete(void);
static void save_mddp_stats(void);

static void mddpRdFunc(struct MDDP_SETTINGS *prSettings, uint32_t *pu4Val)
{
	if (!prSettings->u4SyncAddr)
		return;

	wf_ioremap_read(prSettings->u4SyncAddr, pu4Val);
}

static void mddpSetFuncV1(struct MDDP_SETTINGS *prSettings, uint32_t u4Bit)
{
	uint32_t u4Value = 0;

	if (!prSettings->u4SyncAddr || u4Bit == 0)
		return;

	wf_ioremap_read(prSettings->u4SyncAddr, &u4Value);
	u4Value |= u4Bit;
	wf_ioremap_write(prSettings->u4SyncAddr, u4Value);
}

static void mddpClrFuncV1(struct MDDP_SETTINGS *prSettings, uint32_t u4Bit)
{
	uint32_t u4Value = 0;

	if (!prSettings->u4SyncAddr || u4Bit == 0)
		return;

	wf_ioremap_read(prSettings->u4SyncAddr, &u4Value);
	u4Value &= ~u4Bit;
	wf_ioremap_write(prSettings->u4SyncAddr, u4Value);
}

static void mddpSetFuncV2(struct MDDP_SETTINGS *prSettings, uint32_t u4Bit)
{
	if (!prSettings->u4SyncSetAddr || u4Bit == 0)
		return;

	wf_ioremap_write(prSettings->u4SyncSetAddr, u4Bit);
}

static void mddpClrFuncV2(struct MDDP_SETTINGS *prSettings, uint32_t u4Bit)
{
	if (!prSettings->u4SyncClrAddr || u4Bit == 0)
		return;

	wf_ioremap_write(prSettings->u4SyncClrAddr, u4Bit);
}

static int32_t mddpRegisterCb(void)
{
	int32_t ret = 0;

	switch (g_wifi_boot_mode) {
	case RECOVERY_BOOT:
		g_fgMddpEnabled = FALSE;
		break;
	default:
		g_fgMddpEnabled = TRUE;
		break;
	}
	gMddpFunc.wifi_handle = &gMddpWFunc;

	ret = mddp_drv_attach(&gMddpDrvConf, &gMddpFunc);

	DBGLOG(INIT, INFO, "mddp_drv_attach ret: %d, g_fgMddpEnabled: %d\n",
			ret, g_fgMddpEnabled);

	kalMemZero(&stats, sizeof(struct mddpw_net_stat_ext_t));

	return ret;
}

static void mddpUnregisterCb(void)
{
	DBGLOG(INIT, INFO, "mddp_drv_detach\n");
	mddp_drv_detach(&gMddpDrvConf, &gMddpFunc);
	gMddpFunc.wifi_handle = NULL;
}

int32_t mddpGetMdStats(IN struct net_device *prDev)
{
	struct NETDEV_PRIVATE_GLUE_INFO *prNetDevPrivate;
	struct net_device_stats *prStats;
	struct GLUE_INFO *prGlueInfo;
	struct mddpw_net_stat_ext_t mddpNetStats;
	int32_t ret;
	uint8_t i = 0;

	if (!mddpIsSupportMcifWifi() || !mddpIsSupportMddpWh())
		return 0;

	if (!gMddpWFunc.get_net_stat_ext)
		return 0;

	prNetDevPrivate = (struct NETDEV_PRIVATE_GLUE_INFO *)
			netdev_priv(prDev);
	prStats = &prNetDevPrivate->stats;
	prGlueInfo = prNetDevPrivate->prGlueInfo;

	if (!prGlueInfo || (prGlueInfo->u4ReadyFlag == 0) ||
			!prGlueInfo->prAdapter)
		return 0;

	if (!prNetDevPrivate->ucMddpSupport)
		return 0;

	ret = gMddpWFunc.get_net_stat_ext(&mddpNetStats);
	if (ret != 0) {
		DBGLOG(INIT, ERROR, "get_net_stat fail, ret: %d.\n", ret);
		return 0;
	}
	for (i = 0; i < NW_IF_NUM_MAX; i++) {
		struct mddpw_net_stat_elem_ext_t *element, *prev;

		element = &mddpNetStats.ifs[0][i];
		prev = &stats.ifs[0][i];
		if (kalStrnCmp(element->nw_if_name, prDev->name,
				IFNAMSIZ) != 0)
			continue;

		prStats->rx_packets +=
			element->rx_packets + prev->rx_packets;
		prStats->tx_packets +=
			element->tx_packets + prev->tx_packets;
		prStats->rx_bytes +=
			element->rx_bytes + prev->rx_bytes;
		prStats->tx_bytes +=
			element->tx_bytes + prev->tx_bytes;
		prStats->rx_errors +=
			element->rx_errors + prev->rx_errors;
		prStats->tx_errors +=
			element->tx_errors + prev->tx_errors;
		prStats->rx_dropped +=
			element->rx_dropped + prev->rx_dropped;
		prStats->tx_dropped +=
			element->tx_dropped + prev->tx_dropped;
	}

	return 0;
}

static bool mddpIsSsnSent(struct ADAPTER *prAdapter,
			  uint8_t *prReorderBuf, uint16_t u2SSN)
{
	uint8_t ucSent = 0;
	uint16_t u2Idx = u2SSN / 8;
	uint8_t ucBit = u2SSN % 8;

	ucSent = (prReorderBuf[u2Idx] & BIT(ucBit)) != 0;
	prReorderBuf[u2Idx] &= ~(BIT(ucBit));

	return ucSent;
}

static void mddpGetRxReorderBuffer(struct ADAPTER *prAdapter,
				   struct SW_RFB *prSwRfb,
				   struct mddpw_md_virtual_buf_t **prMdBuf,
				   struct mddpw_ap_virtual_buf_t **prApBuf)
{
	struct mddpw_md_reorder_sync_table_t *prMdTable = NULL;
	struct mddpw_ap_reorder_sync_table_t *prApTable = NULL;
	uint8_t ucStaRecIdx = prSwRfb->ucStaRecIdx;
	uint8_t ucTid = prSwRfb->ucTid;
	int32_t u4Idx = 0;

	if (gMddpWFunc.get_md_rx_reorder_buf &&
	    gMddpWFunc.get_ap_rx_reorder_buf) {
		if (!gMddpWFunc.get_md_rx_reorder_buf(&prMdTable) &&
		    !gMddpWFunc.get_ap_rx_reorder_buf(&prApTable)) {
			u4Idx = prMdTable->reorder_info[ucStaRecIdx].buf_idx;
			*prMdBuf = &prMdTable->virtual_buf[u4Idx][ucTid];
			*prApBuf = &prApTable->virtual_buf[u4Idx][ucTid];
		}
	}
}

void mddpUpdateReorderQueParm(struct ADAPTER *prAdapter,
			      struct RX_BA_ENTRY *prReorderQueParm,
			      struct SW_RFB *prSwRfb)
{
	struct mddpw_md_virtual_buf_t *prMdBuf = NULL;
	struct mddpw_ap_virtual_buf_t *prApBuf = NULL;
	uint16_t u2SSN = prReorderQueParm->u2WinStart, u2Idx;

	mddpGetRxReorderBuffer(prAdapter, prSwRfb, &prMdBuf, &prApBuf);
	if (!prMdBuf || !prApBuf) {
		DBGLOG(QM, ERROR, "Can't get reorder buffer.\n");
		return;
	}

	for (u2Idx = 0; u2Idx < prReorderQueParm->u2WinSize; u2Idx++) {
		if (prReorderQueParm->u2WinStart == prSwRfb->u2SSN ||
		    !mddpIsSsnSent(prAdapter, prMdBuf->virtual_buf, u2SSN))
			break;

		prReorderQueParm->u2WinStart =
			(u2SSN % MAX_SEQ_NO_COUNT);
		prReorderQueParm->u2WinEnd =
			(((prReorderQueParm->u2WinStart) +
			  (prReorderQueParm->u2WinSize) - 1) %
			 MAX_SEQ_NO_COUNT);
		u2SSN = (u2SSN + 1) % MAX_SEQ_NO_COUNT;
		DBGLOG(QM, TRACE,
			"Update reorder window: SSN: %d, start: %d, end: %d.\n",
			u2SSN,
			prReorderQueParm->u2WinStart,
			prReorderQueParm->u2WinEnd);
	}

	prApBuf->start_idx = prReorderQueParm->u2WinStart;
	prApBuf->end_idx = prReorderQueParm->u2WinEnd;
}

int32_t mddpNotifyDrvTxd(IN struct ADAPTER *prAdapter,
	IN struct STA_RECORD *prStaRec,
	IN uint8_t fgActivate)
{
	struct mddpw_drv_notify_info_t *prNotifyInfo;
	struct mddpw_drv_info_t *prDrvInfo;
	struct mddp_txd_t *prMddpTxd;
	struct BSS_INFO *prBssInfo = (struct BSS_INFO *) NULL;
	struct net_device *prNetdev;
	struct NETDEV_PRIVATE_GLUE_INFO *prNetDevPrivate;
	uint32_t u32BufSize = 0;
	uint8_t *buff = NULL;
	int32_t ret = 0;

	if (!gMddpWFunc.notify_drv_info) {
		DBGLOG(NIC, ERROR, "notify_drv_info callback NOT exist.\n");
		ret = -1;
		goto exit;
	}
	if (!prStaRec) {
		DBGLOG(NIC, ERROR, "sta NOT valid\n");
		ret = -1;
		goto exit;
	}
	if (prStaRec->ucBssIndex >= MAX_BSSID_NUM) {
		DBGLOG(NIC, ERROR, "sta bssid NOT valid: %d.\n",
				prStaRec->ucBssIndex);
		ret = -1;
		goto exit;
	}
	if (fgActivate && !prStaRec->aprTxDescTemplate[0]) {
		DBGLOG(NIC, INFO,
			"sta[%d]'s TXD NOT generated done, maybe wait.\n",
			prStaRec->ucBssIndex);
		ret = -1;
		goto exit;
	}

	prBssInfo = prAdapter->aprBssInfo[prStaRec->ucBssIndex];
	prNetdev = (struct net_device *) wlanGetNetInterfaceByBssIdx(
			prAdapter->prGlueInfo, prStaRec->ucBssIndex);
	prNetDevPrivate = (struct NETDEV_PRIVATE_GLUE_INFO *)
			netdev_priv(prNetdev);

	if (!prNetDevPrivate->ucMddpSupport) {
		goto exit;
	}

	u32BufSize = (sizeof(struct mddpw_drv_notify_info_t) +
			sizeof(struct mddpw_drv_info_t) +
			sizeof(struct mddp_txd_t) +
			NIC_TX_DESC_LONG_FORMAT_LENGTH);
	buff = kalMemAlloc(u32BufSize, VIR_MEM_TYPE);

	if (buff == NULL) {
		DBGLOG(NIC, ERROR, "buffer allocation failed.\n");
		ret = -1;
		goto exit;
	}
	prNotifyInfo = (struct mddpw_drv_notify_info_t *) buff;
	prNotifyInfo->version = 0;
	prNotifyInfo->buf_len = sizeof(struct mddpw_drv_info_t) +
			sizeof(struct mddp_txd_t) +
			NIC_TX_DESC_LONG_FORMAT_LENGTH;
	prNotifyInfo->info_num = 1;
	prDrvInfo = (struct mddpw_drv_info_t *) &(prNotifyInfo->buf[0]);
	prDrvInfo->info_id = 3; /* MDDPW_DRV_INFO_TXD; */
	prDrvInfo->info_len = (sizeof(struct mddpw_txd_t) +
			NIC_TX_DESC_LONG_FORMAT_LENGTH);
	prMddpTxd = (struct mddp_txd_t *) &(prDrvInfo->info[0]);
	prMddpTxd->version = 1;
	prMddpTxd->sta_idx = prStaRec->ucIndex;
	prMddpTxd->wlan_idx = prStaRec->ucWlanIndex;
	prMddpTxd->sta_mode = prStaRec->eStaType;
	prMddpTxd->bss_id = prStaRec->ucBssIndex;
	/* TODO: Create a new msg for DMASHDL BMP */
	prMddpTxd->wmmset = prBssInfo->ucWmmQueSet % 2;
	kalMemCopy(prMddpTxd->nw_if_name, prNetdev->name,
			sizeof(prMddpTxd->nw_if_name));
	kalMemCopy(prMddpTxd->aucMacAddr, prStaRec->aucMacAddr, MAC_ADDR_LEN);
	kalMemCopy(prMddpTxd->local_mac,
		   prBssInfo->aucOwnMacAddr, MAC_ADDR_LEN);
	if (fgActivate) {
		prMddpTxd->txd_length = NIC_TX_DESC_LONG_FORMAT_LENGTH;
		kalMemCopy(prMddpTxd->txd, prStaRec->aprTxDescTemplate[0],
				prMddpTxd->txd_length);
	} else {
		prMddpTxd->txd_length = 0;
	}

	ret = gMddpWFunc.notify_drv_info(prNotifyInfo);

#define TEMP_LOG_TEMPLATE "ver:%d,idx:%d,w_idx:%d,mod:%d,bss:%d,wmm:%d," \
		"name:%s,act:%d,ret:%d"
	DBGLOG(NIC, INFO, TEMP_LOG_TEMPLATE,
		prMddpTxd->version,
		prMddpTxd->sta_idx,
		prMddpTxd->wlan_idx,
		prMddpTxd->sta_mode,
		prMddpTxd->bss_id,
		prMddpTxd->wmmset,
		prMddpTxd->nw_if_name,
		fgActivate,
		ret);
#undef TEMP_LOG_TEMPLATE

exit:
	if (buff)
		kalMemFree(buff, VIR_MEM_TYPE, u32BufSize);
	return ret;
}

int32_t mddpNotifyWifiStatus(IN enum ENUM_MDDPW_DRV_INFO_STATUS status)
{
	struct mddpw_drv_notify_info_t *prNotifyInfo;
	struct mddpw_drv_info_t *prDrvInfo;
	uint32_t u32BufSize = 0;
	uint8_t *buff = NULL;
	int32_t ret = 0, feature = 0;

	if (gMddpWFunc.get_mddp_feature)
		feature = gMddpWFunc.get_mddp_feature();

	if (gMddpWFunc.notify_drv_info) {
		int32_t ret;

		u32BufSize = (sizeof(struct mddpw_drv_notify_info_t) +
			sizeof(struct mddpw_drv_info_t) + sizeof(bool));
		buff = kalMemAlloc(u32BufSize, VIR_MEM_TYPE);

		if (buff == NULL) {
			DBGLOG(NIC, ERROR, "Can't allocate buffer.\n");
			return -1;
		}
		prNotifyInfo = (struct mddpw_drv_notify_info_t *) buff;
		prNotifyInfo->version = 0;
		prNotifyInfo->buf_len = sizeof(struct mddpw_drv_info_t) +
				sizeof(bool);
		prNotifyInfo->info_num = 1;
		prDrvInfo = (struct mddpw_drv_info_t *) &(prNotifyInfo->buf[0]);
		prDrvInfo->info_id = MDDPW_DRV_INFO_NOTIFY_WIFI_ONOFF;
		prDrvInfo->info_len = WIFI_ONOFF_NOTIFICATION_LEN;
		prDrvInfo->info[0] = status;

		ret = gMddpWFunc.notify_drv_info(prNotifyInfo);
		DBGLOG(INIT, INFO, "power: %d, ret: %d, feature:%d.\n",
		       status, ret, feature);
		kalMemFree(buff, VIR_MEM_TYPE, u32BufSize);
	} else {
		DBGLOG(INIT, ERROR, "notify_drv_info is NULL.\n");
		ret = -1;
	}

	return ret;
}

static bool mddpIsCasanFWload(void)
{
	struct GLUE_INFO *prGlueInfo = NULL;
	struct ADAPTER *prAdapter = NULL;
	bool ret = FALSE;

	if (gPrDev == NULL) {
		DBGLOG(INIT, ERROR, "gPrDev is NULL.\n");
		goto exit;
	}

	prGlueInfo = *((struct GLUE_INFO **)netdev_priv(gPrDev));
	if (prGlueInfo == NULL) {
		DBGLOG(INIT, ERROR, "prGlueInfo is NULL.\n");
		goto exit;
	}

	prAdapter = prGlueInfo->prAdapter;
	if (prAdapter == NULL) {
		DBGLOG(INIT, ERROR, "prAdapter is NULL.\n");
		goto exit;
	}

	if (prAdapter->u4CasanLoadType == 1)
		ret = TRUE;

exit:
	return ret;
}

int32_t mddpNotifyDrvOwnTimeoutTime(void)
{
	struct mddpw_drv_notify_info_t *prNotifyInfo;
	struct mddpw_drv_info_t *prDrvInfo;
	int32_t ret = 0;
	uint32_t u32BufSize = 0;
	uint32_t u32DrvOwnTimeoutTime = LP_OWN_BACK_FAILED_LOG_SKIP_MS;
	uint8_t *buff = NULL;

	DBGLOG(INIT, INFO, "MD notify Drv Own Timeout time.\n");

	if (!gMddpWFunc.notify_drv_info) {
		DBGLOG(NIC, ERROR, "notify_drv_info callback NOT exist.\n");
		ret = -1;
		goto exit;
	}

	if (mddpIsCasanFWload() == TRUE)
		u32DrvOwnTimeoutTime = LP_OWN_BACK_FAILED_LOG_SKIP_CASAN_MS;
	else
		goto exit;

	u32BufSize = (sizeof(struct mddpw_drv_notify_info_t) +
			sizeof(struct mddpw_drv_info_t) + sizeof(uint32_t));

	buff = kalMemAlloc(u32BufSize, VIR_MEM_TYPE);

	if (buff == NULL) {
		DBGLOG(NIC, ERROR, "buffer allocation failed.\n");
		ret = -ENODEV;
		goto exit;
	}

	prNotifyInfo = (struct mddpw_drv_notify_info_t *) buff;
	prNotifyInfo->version = 0;
	prNotifyInfo->buf_len = sizeof(struct mddpw_drv_info_t) +
			sizeof(uint32_t);
	prNotifyInfo->info_num = 1;
	prDrvInfo = (struct mddpw_drv_info_t *) &(prNotifyInfo->buf[0]);
	prDrvInfo->info_id = 5; /* WSVC_DRVINFO_DRVOWN_TIME_SET */
	prDrvInfo->info_len = sizeof(uint32_t);

	kalMemCopy((uint32_t *) &(prDrvInfo->info[0]), &u32DrvOwnTimeoutTime,
			sizeof(uint32_t));

	ret = gMddpWFunc.notify_drv_info(prNotifyInfo);

exit:
	if (buff)
		kalMemFree(buff, VIR_MEM_TYPE, u32BufSize);

	DBGLOG(INIT, INFO, "ret: %d, timeout: %d.\n",
				   ret, u32DrvOwnTimeoutTime);
	return ret;
}

void mddpNotifyWifiOnStart(void)
{
	if (!mddpIsSupportMcifWifi())
		return;

	mtk_ccci_register_md_state_cb(&mddpMdStateChangedCb);

	mddpNotifyWifiStatus(MDDPW_DRV_INFO_STATUS_ON_START);
}

int32_t mddpNotifyWifiOnEnd(void)
{
	int32_t ret = 0;

	if (!mddpIsSupportMcifWifi())
		return ret;

	/* Notify Driver own timeout time before Wi-Fi on end */
	mddpNotifyDrvOwnTimeoutTime();

	if (g_rSettings.rOps.set)
		g_rSettings.rOps.set(&g_rSettings, g_rSettings.u4WifiOnBit);

	if (g_rSettings.rOps.clr)
		g_rSettings.rOps.clr(&g_rSettings, g_rSettings.u4MdOnBit);
#if (CFG_SUPPORT_CONNAC2X == 0)
	ret = mddpNotifyWifiStatus(MDDPW_DRV_INFO_STATUS_ON_END);
#else
	ret = mddpNotifyWifiStatus(MDDPW_DRV_INFO_STATUS_ON_END_QOS);
#endif
	if (ret == 0)
		ret = wait_for_md_on_complete() ?
				WLAN_STATUS_SUCCESS :
				WLAN_STATUS_FAILURE;
	return ret;
}

void mddpNotifyWifiOffStart(void)
{
	int32_t ret;

	if (!mddpIsSupportMcifWifi())
		return;

	mddpSetMDFwOwn();

	mtk_ccci_register_md_state_cb(NULL);

	DBGLOG(INIT, INFO, "md off start.\n");
	if (g_rSettings.rOps.set)
		g_rSettings.rOps.set(&g_rSettings, g_rSettings.u4MdOffBit);

	ret = mddpNotifyWifiStatus(MDDPW_DRV_INFO_STATUS_OFF_START);
	if (ret == 0)
		wait_for_md_off_complete();
}

void mddpNotifyWifiOffEnd(void)
{
	if (!mddpIsSupportMcifWifi())
		return;

	if (g_rSettings.rOps.clr) {
		g_rSettings.rOps.clr(
			&g_rSettings,
			g_rSettings.u4WifiOnBit | g_rSettings.u4MdInitBit);
	}

	mddpNotifyWifiStatus(MDDPW_DRV_INFO_STATUS_OFF_END);
}

void mddpNotifyWifiReset(void)
{
	if (!mddpIsSupportMcifWifi())
		return;

	if (g_rSettings.rOps.clr)
		g_rSettings.rOps.clr(&g_rSettings, g_rSettings.u4WifiOnBit);
}

int32_t mddpMdNotifyInfo(struct mddpw_md_notify_info_t *prMdInfo)
{
	struct GLUE_INFO *prGlueInfo = NULL;
	struct ADAPTER *prAdapter = NULL;
	int32_t ret = 0;
	u_int8_t fgHalted = kalIsHalted();

	DBGLOG(INIT, INFO, "MD notify mddpMdNotifyInfo.\n");

	if (gPrDev == NULL) {
		DBGLOG(INIT, ERROR, "gPrDev is NULL.\n");
		ret = -ENODEV;
		goto exit;
	}

	prGlueInfo = *((struct GLUE_INFO **)netdev_priv(gPrDev));
	if (prGlueInfo == NULL) {
		DBGLOG(INIT, ERROR, "prGlueInfo is NULL.\n");
		ret = -ENODEV;
		goto exit;
	}
	prAdapter = prGlueInfo->prAdapter;
	if (prAdapter == NULL) {
		DBGLOG(INIT, ERROR, "prAdapter is NULL.\n");
		ret = -ENODEV;
		goto exit;
	}

	if (fgHalted || !prGlueInfo->u4ReadyFlag) {
		DBGLOG(INIT, INFO,
			"Skip update info. to MD, fgHalted: %d, u4ReadyFlag: %d\n",
			fgHalted, prGlueInfo->u4ReadyFlag);
		ret = -ENODEV;
		goto exit;
	}

	if (prMdInfo->info_type == MDDPW_MD_INFO_RESET_IND) {
		uint32_t i;
		struct BSS_INFO *prSapBssInfo = (struct BSS_INFO *) NULL;
		struct BSS_INFO *prP2pBssInfo = (struct BSS_INFO *) NULL;
		int32_t ret;

		save_mddp_stats();
		mddpNotifyWifiOnStart();
		ret = mddpNotifyWifiOnEnd();
		if (ret != WLAN_STATUS_SUCCESS) {
			DBGLOG(INIT, INFO, "mddpNotifyWifiOnEnd failed.\n");
			return 0;
		}

		/* Notify STA's TXD to MD */
		for (i = 0; i < KAL_AIS_NUM; i++) {
			struct BSS_INFO *prAisBssInfo = aisGetAisBssInfo(
					prAdapter,
					i);

			if (prAisBssInfo && prAisBssInfo->eConnectionState ==
					MEDIA_STATE_CONNECTED)
				mddpNotifyDrvTxd(prAdapter,
						prAisBssInfo->prStaRecOfAP,
						TRUE);
		}
		/* Notify SAP clients' TXD to MD */
		prP2pBssInfo = cnmGetSapBssInfo(prAdapter);
		if (prP2pBssInfo) {
			struct LINK *prClientList;
			struct STA_RECORD *prCurrStaRec;

			prClientList = &prP2pBssInfo->rStaRecOfClientList;
			LINK_FOR_EACH_ENTRY(prCurrStaRec, prClientList,
					rLinkEntry, struct STA_RECORD) {
				if (!prCurrStaRec)
					break;
				mddpNotifyDrvTxd(prAdapter,
						prCurrStaRec,
						TRUE);
			}
		}
		prSapBssInfo = cnmGetOtherSapBssInfo(prAdapter, prP2pBssInfo);
		if (prSapBssInfo) {
			struct LINK *prClientList;
			struct STA_RECORD *prCurrStaRec;

			prClientList = &prSapBssInfo->rStaRecOfClientList;
			LINK_FOR_EACH_ENTRY(prCurrStaRec, prClientList,
					rLinkEntry, struct STA_RECORD) {
				if (!prCurrStaRec)
					break;
				mddpNotifyDrvTxd(prAdapter,
						prCurrStaRec,
						TRUE);
			}
		}
	} else if (prMdInfo->info_type == MDDPW_MD_INFO_DRV_EXCEPTION) {
		struct wsvc_md_event_exception_t *event;

		if (prMdInfo->buf_len != sizeof(
				struct wsvc_md_event_exception_t)) {
			DBGLOG(INIT, ERROR,
				"Invalid args from MD, expect %u but %u\n",
				sizeof(struct wsvc_md_event_exception_t),
				prMdInfo->buf_len);
			ret = -EINVAL;
			goto exit;
		}
		event = (struct wsvc_md_event_exception_t *)
				&(prMdInfo->buf[1]);
		DBGLOG(INIT, WARN, "reason: %d, flag: %d, line: %d, func: %s\n",
				event->u4RstReason,
				event->u4RstFlag,
				event->u4Line,
				event->pucFuncName);
		glSetRstReason(RST_MDDP_MD_TRIGGER_EXCEPTION);
		GL_RESET_TRIGGER(prAdapter, event->u4RstFlag
			| RST_FLAG_DO_CORE_DUMP);
	} else {
		DBGLOG(INIT, ERROR, "unknown MD info type: %d\n",
			prMdInfo->info_type);
		ret = -ENODEV;
		goto exit;
	}

exit:
	return ret;
}

int32_t mddpChangeState(enum mddp_state_e event, void *buf, uint32_t *buf_len)
{
	struct GLUE_INFO *prGlueInfo = NULL;
	struct ADAPTER *prAdapter = NULL;
	u_int8_t fgHalted = kalIsHalted();

	if (gPrDev == NULL) {
		DBGLOG(INIT, ERROR, "gPrDev is NULL.\n");
		return 0;
	}

	prGlueInfo = *((struct GLUE_INFO **)netdev_priv(gPrDev));
	if (prGlueInfo == NULL) {
		DBGLOG(INIT, ERROR, "prGlueInfo is NULL.\n");
		return 0;
	}

	if (fgHalted || !prGlueInfo->u4ReadyFlag) {
		DBGLOG(INIT, ERROR, "fgHalted: %d, u4ReadyFlag: %d\n",
				fgHalted, prGlueInfo->u4ReadyFlag);
		return 0;
	}

	prAdapter = prGlueInfo->prAdapter;
	if (prAdapter == NULL) {
		DBGLOG(INIT, ERROR, "prAdapter is NULL.\n");
		return 0;
	}

	switch (event) {
	case MDDP_STATE_ENABLING:
		break;

	case MDDP_STATE_ACTIVATING:
		break;

	case MDDP_STATE_ACTIVATED:
		DBGLOG(INIT, INFO, "Mddp activated.\n");
		prAdapter->fgMddpActivated = true;
		break;

	case MDDP_STATE_DEACTIVATING:
		break;

	case MDDP_STATE_DEACTIVATED:
		DBGLOG(INIT, INFO, "Mddp deactivated.\n");
		prAdapter->fgMddpActivated = false;
		break;

	case MDDP_STATE_DISABLING:
	case MDDP_STATE_UNINIT:
	case MDDP_STATE_CNT:
	case MDDP_STATE_DUMMY:
	default:
		break;
	}

	return 0;

}

static bool wait_for_md_off_complete(void)
{
	uint32_t u4Value = 0;
	uint32_t u4StartTime, u4CurTime;
	bool fgTimeout = false;
	uint32_t u4MDOffTimeoutTime = MD_ON_OFF_TIMEOUT;

	if (mddpIsCasanFWload() == TRUE)
		u4MDOffTimeoutTime = MD_ON_OFF_TIMEOUT_CASAN;

	u4StartTime = kalGetTimeTick();

	do {
		if (g_rSettings.rOps.rd)
			g_rSettings.rOps.rd(&g_rSettings, &u4Value);

		if ((u4Value & g_rSettings.u4MdOffBit) == 0) {
			DBGLOG(INIT, INFO, "md off end.\n");
			break;
		}

		u4CurTime = kalGetTimeTick();
		if (CHECK_FOR_TIMEOUT(u4CurTime, u4StartTime,
				u4MDOffTimeoutTime)) {
			DBGLOG(INIT, ERROR, "wait for md off timeout\n");
			fgTimeout = true;
			break;
		}

		kalMsleep(CFG_RESPONSE_POLLING_DELAY);
	} while (TRUE);

	return !fgTimeout;
}

static bool wait_for_md_on_complete(void)
{
	uint32_t u4Value = 0;
	uint32_t u4StartTime, u4CurTime;
	bool fgCompletion = false;
	struct GLUE_INFO *prGlueInfo = NULL;
	uint32_t u4MDOnTimeoutTime = MD_ON_OFF_TIMEOUT;

	u4StartTime = kalGetTimeTick();
	prGlueInfo = *((struct GLUE_INFO **) netdev_priv(gPrDev));
	if (!prGlueInfo) {
		DBGLOG(INIT, ERROR, "prGlueInfo is NULL.\n");
		return false;
	}

	if (mddpIsCasanFWload() == TRUE)
		u4MDOnTimeoutTime = MD_ON_OFF_TIMEOUT_CASAN;

	do {
		if (g_rSettings.rOps.rd)
			g_rSettings.rOps.rd(&g_rSettings, &u4Value);

		if ((u4Value & g_rSettings.u4MdOnBit) > 0) {
			DBGLOG(INIT, INFO, "md on end.\n");
			fgCompletion = true;
			break;
		} else if (!prGlueInfo->u4ReadyFlag) {
			DBGLOG(INIT, WARN, "Skip waiting due to ready flag.\n");
			fgCompletion = false;
			break;
		}

		u4CurTime = kalGetTimeTick();
		if (CHECK_FOR_TIMEOUT(u4CurTime, u4StartTime,
				u4MDOnTimeoutTime)) {
			DBGLOG(INIT, ERROR, "wait for md on timeout\n");
			fgCompletion = false;
			break;
		}

		kalMsleep(CFG_RESPONSE_POLLING_DELAY);
	} while (TRUE);

	return fgCompletion;
}

void setMddpSupportRegister(IN struct ADAPTER *prAdapter)
{
	struct mt66xx_chip_info *prChipInfo;
#if (CFG_SUPPORT_CONNAC2X == 0)
	uint32_t u4Val = 0;
#endif

	prChipInfo = prAdapter->chip_info;

	if (prChipInfo->isSupportMddpAOR) {
		g_rSettings.rOps.rd = mddpRdFunc;
		g_rSettings.rOps.set = mddpSetFuncV2;
		g_rSettings.rOps.clr = mddpClrFuncV2;
		g_rSettings.u4SyncAddr = MD_AOR_RD_CR_ADDR;
		g_rSettings.u4SyncSetAddr = MD_AOR_SET_CR_ADDR;
		g_rSettings.u4SyncClrAddr = MD_AOR_CLR_CR_ADDR;
		g_rSettings.u4MdInitBit = MD_AOR_MD_INIT_BIT;
		g_rSettings.u4MdOnBit = MD_AOR_MD_RDY_BIT;
		g_rSettings.u4MdOffBit = MD_AOR_MD_OFF_BIT;
		g_rSettings.u4WifiOnBit = MD_AOR_WIFI_ON_BIT;
	} else {
		g_rSettings.rOps.rd = mddpRdFunc;
		g_rSettings.rOps.set = mddpSetFuncV1;
		g_rSettings.rOps.clr = mddpClrFuncV1;
		g_rSettings.u4SyncAddr = MD_STATUS_SYNC_CR;
		g_rSettings.u4MdOnBit = MD_STATUS_ON_SYNC_BIT;
		g_rSettings.u4MdOffBit = MD_STATUS_OFF_SYNC_BIT;
	}

#if (CFG_SUPPORT_CONNAC2X == 0)
	HAL_MCR_RD(prAdapter, MDDP_SUPPORT_CR, &u4Val);
	if (g_fgMddpEnabled)
		u4Val |= MDDP_SUPPORT_CR_BIT;
	else
		u4Val &= ~MDDP_SUPPORT_CR_BIT;
	HAL_MCR_WR(prAdapter, MDDP_SUPPORT_CR, u4Val);
#endif
}

void mddpInit(void)
{
	struct device_node *np_chosen;
	struct tag_bootmode *tag = NULL;

	np_chosen = of_find_node_by_path("/chosen");
	if (!np_chosen)
		np_chosen = of_find_node_by_path("/chosen@0");

	if (!np_chosen)
		return;

	tag = (struct tag_bootmode *) of_get_property(np_chosen, "atag,boot",
			NULL);

	if (!tag)
		return;

	DBGLOG(INIT, INFO, "bootmode: 0x%x\n", tag->bootmode);
	g_wifi_boot_mode = tag->bootmode;

	mddpRegisterCb();
}

void mddpUninit(void)
{
	mddpUnregisterCb();
}

static void notifyMdCrash2FW(void)
{
	struct GLUE_INFO *prGlueInfo = NULL;

	if (gPrDev == NULL) {
		DBGLOG(INIT, ERROR, "gPrDev is NULL.\n");
		return;
	}

	prGlueInfo = *((struct GLUE_INFO **) netdev_priv(gPrDev));
	if (!prGlueInfo || !prGlueInfo->u4ReadyFlag) {
		DBGLOG(INIT, ERROR, "Invalid drv state.\n");
		return;
	}

	/*
	 * Set MD FW Own before Notify FW MD crash
	 * Reason: MD cannt set FW own itself when MD crash
	 */
	mddpSetMDFwOwn();

	kalSetMdCrashEvent(prGlueInfo);
}

void  mddpMdStateChangedCb(enum MD_STATE old_state,
		enum MD_STATE new_state)
{
	DBGLOG(INIT, TRACE, "old_state: %d, new_state: %d.\n",
			old_state, new_state);

	switch (new_state) {
	case GATED: /* MD off */
	case EXCEPTION: /* MD crash */
		notifyMdCrash2FW();
		break;
	default:
		break;
	}
}

static void save_mddp_stats(void)
{
	struct mddpw_net_stat_ext_t temp;
	uint8_t i = 0;
	int32_t ret;

	if (!gMddpWFunc.get_net_stat_ext)
		return;

	ret = gMddpWFunc.get_net_stat_ext(&temp);
	if (ret != 0) {
		DBGLOG(INIT, ERROR, "get_net_stat fail, ret: %d.\n", ret);
		return;
	}

	for (i = 0; i < NW_IF_NUM_MAX; i++) {
		struct mddpw_net_stat_elem_ext_t *element, *curr;

		element = &temp.ifs[0][i];
		curr = &stats.ifs[0][i];

		curr->rx_packets += element->rx_packets;
		curr->tx_packets += element->tx_packets;
		curr->rx_bytes += element->rx_bytes;
		curr->tx_bytes += element->tx_bytes;
		curr->rx_errors += element->rx_errors;
		curr->tx_errors += element->tx_errors;
		curr->rx_dropped += element->rx_dropped;
		curr->tx_dropped += element->tx_dropped;
	}
}

void mddpSetMDFwOwn(void)
{
	wf_ioremap_write(MD_LPCTL_ADDR, MDDP_LPCR_MD_SET_FW_OWN);
	DBGLOG(INIT, INFO, "Set MD Fw Own.\n");
}

bool mddpIsSupportMcifWifi(void)
{
	if (!gMddpWFunc.get_mddp_feature)
		return false;

	return (gMddpWFunc.get_mddp_feature() & MDDP_FEATURE_MCIF_WIFI) != 0;
}

bool mddpIsSupportMddpWh(void)
{
	if (!gMddpWFunc.get_mddp_feature)
		return false;

	return (gMddpWFunc.get_mddp_feature() & MDDP_FEATURE_MDDP_WH) != 0;
}

#endif /* CFG_MTK_MDDP_SUPPORT */
