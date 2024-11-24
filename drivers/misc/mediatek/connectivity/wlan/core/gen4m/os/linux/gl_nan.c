/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MediaTek Inc.
 */
/*
 ** Id: @(#) gl_nan.c@@
 */

/*! \file   gl_nan.c
 *    \brief  Main routines of Linux driver interface for Wi-Fi Aware
 *
 *    This file contains the main routines of Linux driver for MediaTek Inc.
 *    802.11 Wireless LAN Adapters.
 */

/*******************************************************************************
 *                         C O M P I L E R   F L A G S
 *******************************************************************************
 */

/*******************************************************************************
 *                    E X T E R N A L   R E F E R E N C E S
 *******************************************************************************
 */

#include <linux/poll.h>

#include <linux/kmod.h>

#include "precomp.h"
#include "debug.h"
#include "gl_os.h"
#include "gl_wext.h"
#include "wlan_lib.h"

#include "gl_cfg80211.h"
#include "gl_vendor.h"
#include "nan/nan_sec.h"

/*******************************************************************************
 *                              C O N S T A N T S
 *******************************************************************************
 */

#define NAN_INF_NAME "nan%d"

#if 0
#define RUNNING_P2P_MODE 0
#define RUNNING_AP_MODE 1
#define RUNNING_DUAL_AP_MODE 2
#endif

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

struct wireless_dev *g_aprNanRoleWdev[NAN_BSS_INDEX_NUM];
struct _GL_NAN_INFO_T g_aprNanMultiDev[NAN_BSS_INDEX_NUM];

static unsigned char *nifname = NAN_INF_NAME;

#if CFG_ENABLE_WIFI_DIRECT_CFG_80211
#endif

static const struct iw_priv_args rNANIwPrivTable[] = {
	{ IOCTL_GET_DRIVER, IW_PRIV_TYPE_CHAR | 2000, IW_PRIV_TYPE_CHAR | 2000,
	  "driver" },
};

#if 0
const struct iw_handler_def mtk_p2p_wext_handler_def = {
	.num_standard =
		(__u16) sizeof(rP2PIwStandardHandler) / sizeof(iw_handler),
	/* .num_private = */
	/*(__u16)sizeof(rP2PIwPrivHandler)/sizeof(iw_handler), */
	.num_private_args =
		(__u16) sizeof(rP2PIwPrivTable) / sizeof(struct iw_priv_args),
	.standard = rP2PIwStandardHandler,
	/* .private            = rP2PIwPrivHandler, */
	.private_args = rP2PIwPrivTable,
#if CFG_SUPPORT_P2P_RSSI_QUERY
	.get_wireless_stats = mtk_p2p_wext_get_wireless_stats,
#else
	.get_wireless_stats = NULL,
#endif
};
#endif

/*******************************************************************************
 *                                 M A C R O S
 *******************************************************************************
 */

/*******************************************************************************
 *                   F U N C T I O N   D E C L A R A T I O N S
 *******************************************************************************
 */

/* Net Device Hooks */
static int nanOpen(IN struct net_device *prDev);

static int nanStop(IN struct net_device *prDev);

static struct net_device_stats *nanGetStats(IN struct net_device *prDev);

static void nanSetMulticastList(IN struct net_device *prDev);

static netdev_tx_t nanHardStartXmit(IN struct sk_buff *prSkb,
				    IN struct net_device *prDev);

static int nanDoIOCTL(struct net_device *prDev, struct ifreq *prIFReq,
		      int i4Cmd);

/*----------------------------------------------------------------------------*/
/*!
 * \brief A function for prDev->init
 *
 * \param[in] prDev      Pointer to struct net_device.
 *
 * \retval 0         The execution of wlanInit succeeds.
 * \retval -ENXIO    No such device.
 */
/*----------------------------------------------------------------------------*/
static int
nanInit(struct net_device *prDev) {
	if (!prDev)
		return -ENXIO;

	return 0; /* success */
}

/*----------------------------------------------------------------------------*/
/*!
 * \brief A function for prDev->uninit
 *
 * \param[in] prDev      Pointer to struct net_device.
 *
 * \return (none)
 */
/*----------------------------------------------------------------------------*/
static void
nanUninit(IN struct net_device *prDev) {}
const struct net_device_ops nan_netdev_ops = {
	.ndo_open = nanOpen,
	.ndo_stop = nanStop,
	.ndo_set_rx_mode = nanSetMulticastList,
	.ndo_get_stats = nanGetStats,
	.ndo_do_ioctl = nanDoIOCTL,
	.ndo_start_xmit = nanHardStartXmit,
	.ndo_select_queue = wlanSelectQueue,
	.ndo_init = nanInit,
	.ndo_uninit = nanUninit,
};

/*******************************************************************************
 *                              F U N C T I O N S
 *******************************************************************************
 */

/*----------------------------------------------------------------------------*/
/*!
 * \brief Allocate memory for NAN_INFO,
 *
 * \param[in] prGlueInfo      Pointer to glue info
 *
 * \return   TRUE
 *           FALSE
 */
/*----------------------------------------------------------------------------*/
unsigned char
nanAllocInfo(IN struct GLUE_INFO *prGlueInfo, uint8_t ucRoleIdx)
{
	struct ADAPTER *prAdapter = NULL;
	struct WIFI_VAR *prWifiVar = NULL;
	uint8_t ucIdx = 0;

	if (!prGlueInfo) {
		DBGLOG(NAN, ERROR, "prGlueInfo error\n");
		return FALSE;
	}

	prAdapter = prGlueInfo->prAdapter;
	prWifiVar = &(prAdapter->rWifiVar);

	if (!prAdapter) {
		DBGLOG(NAN, ERROR, "prAdapter error!\n");
		return FALSE;
	}
	if (!prWifiVar) {
		DBGLOG(NAN, ERROR, "prWifiVar error!\n");
		return FALSE;
	}


	if (prGlueInfo->aprNANDevInfo[ucRoleIdx] == NULL) {
		/* alloc memory for NANDEV info */
		prGlueInfo->aprNANDevInfo[ucRoleIdx] = kalMemAlloc(
			sizeof(struct _GL_NAN_INFO_T), VIR_MEM_TYPE);
		if (prGlueInfo->aprNANDevInfo[ucRoleIdx]) {
			kalMemZero(prGlueInfo->aprNANDevInfo[ucRoleIdx],
				   sizeof(struct _GL_NAN_INFO_T));
		} else {
			DBGLOG(NAN, INFO, "alloc aprNANDevInfo fail\n");
			goto err_alloc;
		}

		for (ucIdx = 0; ucIdx < NAN_BSS_INDEX_NUM; ucIdx++) {
			prWifiVar->aprNanSpecificBssInfo[ucIdx] =
				kalMemAlloc(
				sizeof(struct _NAN_SPECIFIC_BSS_INFO_T),
				VIR_MEM_TYPE);
		}
	}

	for (ucIdx = 0; ucIdx < NAN_BSS_INDEX_NUM; ucIdx++) {
		/* chk if alloc successful or not */
		if (prGlueInfo->aprNANDevInfo[ucRoleIdx] &&
		    prWifiVar->aprNanSpecificBssInfo[ucIdx])
			continue;

		DBGLOG(NAN, ERROR, "[fail!]NANAllocInfo :fail\n");
		goto err_alloc;
	}

	return TRUE;

err_alloc:
	for (ucIdx = 0; ucIdx < NAN_BSS_INDEX_NUM; ucIdx++) {
		if (prWifiVar->aprNanSpecificBssInfo[ucIdx]) {
			kalMemFree(prWifiVar->aprNanSpecificBssInfo[ucIdx],
				VIR_MEM_TYPE, sizeof(_NAN_SPECIFIC_BSS_INFO_T));

			prWifiVar->aprNanSpecificBssInfo[ucIdx] = NULL;
		}
	}

	if (prGlueInfo->aprNANDevInfo[ucRoleIdx]) {
		kalMemFree(prGlueInfo->aprNANDevInfo, VIR_MEM_TYPE,
		   sizeof(struct _GL_NAN_INFO_T));

		prGlueInfo->aprNANDevInfo[ucRoleIdx] = NULL;
	}

	return FALSE;
}

/*----------------------------------------------------------------------------*/
/*!
 * \brief Free memory for NAN_INFO,
 *
 * \param[in] prGlueInfo      Pointer to glue info
 *	[in] ucIdx	     The BSS with the idx will be freed.
 *			     "ucIdx == 0xff" will free all BSSs.
 *			     Only has meaning for "CFG_ENABLE_UNIFY_WIPHY == 1"
 *
 * \return   TRUE
 *           FALSE
 */
/*----------------------------------------------------------------------------*/
unsigned char
nanFreeInfo(struct GLUE_INFO *prGlueInfo, uint8_t ucRoleIdx)
{
	struct ADAPTER *prAdapter = prGlueInfo->prAdapter;

	if (!prGlueInfo) {
		DBGLOG(NAN, ERROR, "prGlueInfo error\n");
		return FALSE;
	}
	if (!prAdapter) {
		DBGLOG(NAN, ERROR, "prAdapter error!\n");
		return FALSE;
	}

	if (prGlueInfo->aprNANDevInfo[ucRoleIdx] != NULL) {
		kalMemFree(prGlueInfo->prAdapter->rWifiVar
				   .aprNanSpecificBssInfo[ucRoleIdx],
			   VIR_MEM_TYPE, sizeof(_NAN_SPECIFIC_BSS_INFO_T));
		prGlueInfo->prAdapter->rWifiVar
			.aprNanSpecificBssInfo[ucRoleIdx] = NULL;
		kalMemFree(prGlueInfo->aprNANDevInfo[ucRoleIdx], VIR_MEM_TYPE,
			   sizeof(struct _GL_NAN_INFO_T *));
		prGlueInfo->aprNANDevInfo[ucRoleIdx] = NULL;
	}

	return TRUE;
}

unsigned char
nanNetRegister(struct GLUE_INFO *prGlueInfo,
	    unsigned char fgIsRtnlLockAcquired)
{
	unsigned char fgDoRegister = FALSE;
	unsigned char fgRollbackRtnlLock = FALSE;
	unsigned char ret;
	enum NAN_BSS_ROLE_INDEX eRole = NAN_BSS_INDEX_BAND0;

	GLUE_SPIN_LOCK_DECLARATION();

	if (!prGlueInfo) {
		DBGLOG(NAN, ERROR, "prGlueInfo error\n");
		return FALSE;
	}
	if (!prGlueInfo->prAdapter) {
		DBGLOG(NAN, ERROR, "prGlueInfo->prAdapter error\n");
		return FALSE;
	}

	GLUE_ACQUIRE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_NET_DEV);
	if (prGlueInfo->prAdapter->rNanNetRegState ==
	    ENUM_NET_REG_STATE_UNREGISTERED) {
		prGlueInfo->prAdapter->rNanNetRegState =
			ENUM_NET_REG_STATE_REGISTERING;
		fgDoRegister = TRUE;
	}
	GLUE_RELEASE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_NET_DEV);

	if (!fgDoRegister)
		return TRUE;

	if (fgIsRtnlLockAcquired && rtnl_is_locked()) {
		fgRollbackRtnlLock = TRUE;
		rtnl_unlock();
	}

	ret = TRUE;
	/* net device initialize */
	netif_carrier_off(
		prGlueInfo->aprNANDevInfo[eRole]->prDevHandler);
	netif_tx_stop_all_queues(
		prGlueInfo->aprNANDevInfo[eRole]->prDevHandler);

	/* register for net device */
	if (register_netdev(
		    prGlueInfo->aprNANDevInfo[eRole]->prDevHandler) <
	    0) {
		DBGLOG(INIT, WARN,
		       "unable to register netdevice for nan\n");
		/* trunk doesn't do free_netdev here */
		free_netdev(
			prGlueInfo->aprNANDevInfo[eRole]->prDevHandler);

		ret = FALSE;
	} else {
		prGlueInfo->prAdapter->rNanNetRegState =
			ENUM_NET_REG_STATE_REGISTERED;

#if CFG_SUPPORT_NAN_CARRIER_ON_INIT
		rtnl_lock();
		dev_change_flags(
			prGlueInfo->aprNANDevInfo[eRole]->prDevHandler,
			prGlueInfo->aprNANDevInfo[eRole]
					->prDevHandler->flags |
				IFF_UP
#if KERNEL_VERSION(5, 4, 0) <= CFG80211_VERSION_CODE
				, NULL
#endif
				);
		rtnl_unlock();

		netif_carrier_on(
			prGlueInfo->aprNANDevInfo[eRole]->prDevHandler);
#endif
	}

	if (fgRollbackRtnlLock)
		rtnl_lock();

	return ret;
}

unsigned char
nanNetUnregister(struct GLUE_INFO *prGlueInfo,
	    unsigned char fgIsRtnlLockAcquired)
{
	unsigned char fgDoUnregister = FALSE;
	unsigned char fgRollbackRtnlLock = FALSE;
	struct ADAPTER *prAdapter = NULL;
	struct _GL_NAN_INFO_T *prNANInfo = NULL;
	uint8_t ucIdx = NAN_BSS_INDEX_BAND0;

	GLUE_SPIN_LOCK_DECLARATION();

	prAdapter = prGlueInfo->prAdapter;

	if (!prGlueInfo) {
		DBGLOG(NAN, ERROR, "prGlueInfo error\n");
		return FALSE;
	}
	if (!prAdapter) {
		DBGLOG(NAN, ERROR, "prAdapter error\n");
		return FALSE;
	}

	GLUE_ACQUIRE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_NET_DEV);
	if (prAdapter->rNanNetRegState == ENUM_NET_REG_STATE_REGISTERED) {
		prAdapter->rNanNetRegState = ENUM_NET_REG_STATE_UNREGISTERING;
		fgDoUnregister = TRUE;
	}
	GLUE_RELEASE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_NET_DEV);

	if (!fgDoUnregister)
		return TRUE;

	if (fgIsRtnlLockAcquired && rtnl_is_locked())
		fgRollbackRtnlLock = TRUE;

	prNANInfo = prGlueInfo->aprNANDevInfo[ucIdx];
	if (prNANInfo == NULL)
		return FALSE;

#if CFG_ENABLE_UNIFY_WIPHY
	{
		/* don't unregister the dev that share with the AIS */
		uint32_t u4Idx = 0;

		for (u4Idx = 0; u4Idx < KAL_AIS_NUM; u4Idx++) {

			if (gprWdev[u4Idx] &&
			    (prNANInfo->prDevHandler ==
			     gprWdev[u4Idx]->netdev))
				return FALSE;
		}
	}
#endif

	if (netif_carrier_ok(prNANInfo->prDevHandler))
		netif_carrier_off(prNANInfo->prDevHandler);

	netif_tx_stop_all_queues(prNANInfo->prDevHandler);

	if (fgRollbackRtnlLock)
		rtnl_unlock();

	unregister_netdev(prNANInfo->prDevHandler);
	DBGLOG(INIT, INFO, "unregister nandev\n");
	if (fgRollbackRtnlLock)
		rtnl_lock();

	prGlueInfo->prAdapter->rNanNetRegState =
		ENUM_NET_REG_STATE_UNREGISTERED;

	return TRUE;
}

/*----------------------------------------------------------------------------*/
/*!
 * \brief Setup the NAN device information
 *
 * \param[in] prGlueInfo      Pointer to glue info
 *       [in] prNANWdev       Pointer to the wireless device
 *       [in] prNANDev        Pointer to the net device
 *       [in] u4Idx           The nan Role index
 *
 * \return    0	Success
 *           -1	Failure
 */
/*----------------------------------------------------------------------------*/
int
glSetupNAN(struct GLUE_INFO *prGlueInfo, struct wireless_dev *prNanWdev,
	   struct net_device *prNanDev, int u4Idx)
{
	struct ADAPTER *prAdapter = NULL;
	struct _GL_NAN_INFO_T *prNANInfo = NULL;
	struct GL_HIF_INFO *prHif = NULL;
	uint8_t ucBssIndex;
	struct NETDEV_PRIVATE_GLUE_INFO *prNetDevPriv = NULL;

	DBGLOG(INIT, INFO, "setup the nan dev\n");

	if ((prGlueInfo == NULL) || (prNanWdev == NULL) ||
	    (prNanWdev->wiphy == NULL) || (prNanDev == NULL)) {
		DBGLOG(INIT, ERROR, "parameter is NULL!!\n");
		return -1;
	}

	prHif = &prGlueInfo->rHifInfo;
	prAdapter = prGlueInfo->prAdapter;

	if ((prAdapter == NULL) || (prHif == NULL)) {
		DBGLOG(INIT, ERROR, "prAdapter/prHif is NULL!!\n");
		return -1;
	}

	/*0. allocate naninfo */
	if (nanAllocInfo(prGlueInfo, u4Idx) != TRUE) {
		DBGLOG(INIT, WARN, "Allocate memory for nan FAILED\n");
		return -1;
	}

	prNANInfo = prGlueInfo->aprNANDevInfo[u4Idx];

	if (!prAdapter->fgEnable5GBand)
		prNanWdev->wiphy->bands[BAND_5G] = NULL;
	/* setup netdev */
	/* Point to shared glue structure */
	prNetDevPriv = (struct NETDEV_PRIVATE_GLUE_INFO *)netdev_priv(prNanDev);
	prNetDevPriv->prGlueInfo = prGlueInfo;

	/* set ucNAN for NAN function device */
	prNanWdev->iftype = NL80211_IFTYPE_P2P_GO;

	prNetDevPriv->ucIsNan = TRUE;
	/* register callback functions */
	prNanDev->needed_headroom += NIC_TX_HEAD_ROOM;
	prNanDev->netdev_ops = &nan_netdev_ops;

#if defined(_HIF_SDIO)
#if (MTK_WCN_HIF_SDIO == 0)
	SET_NETDEV_DEV(prNanDev, &(prHif->func->dev));
#endif
#endif
	prNanDev->ieee80211_ptr = prNanWdev;
	prNanWdev->netdev = prNanDev;
#if CFG_TCP_IP_CHKSUM_OFFLOAD
	/* set HW checksum offload */
	if (prAdapter->fgIsSupportCsumOffload) {
		prNanDev->features =
			NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM | NETIF_F_RXCSUM;
	}
#endif /* CFG_TCP_IP_CHKSUM_OFFLOAD */

	kalResetStats(prNanDev);

	/* finish
	 * bind netdev pointer to netdev index
	 */
	prNANInfo->prDevHandler = prNanDev;
	DBGLOG(INIT, INFO, "setup the nan dev\n");

	for (u4Idx = 0; u4Idx < NAN_BSS_INDEX_NUM; u4Idx++) {

		ucBssIndex = nanDevInit(prGlueInfo->prAdapter, u4Idx);

		if (ucBssIndex == MAX_BSS_INDEX) {
			DBGLOG(INIT, ERROR, "No BSS can be used!!\n");
			nanFreeInfo(prGlueInfo, u4Idx);
			return -1;
		}
		if (u4Idx == NAN_BSS_INDEX_BAND0) {
			prNetDevPriv->ucBssIdx = ucBssIndex;
		}
		wlanBindBssIdxToNetInterface(prGlueInfo, ucBssIndex,
					(void *)prNANInfo->prDevHandler);
	}
	return 0;
}

/*----------------------------------------------------------------------------*/
/*!
 * \brief This function is to set multicast list and set rx mode.
 *
 * \param[in] prDev  Pointer to struct net_device
 *
 * \return (none)
 */
/*----------------------------------------------------------------------------*/
void
mtk_nan_wext_set_Multicastlist(struct GLUE_INFO *prGlueInfo)
{
	uint32_t u4SetInfoLen = 0;
	uint32_t u4McCount;
	uint32_t u4PacketFilter = 0;
	uint8_t ucRoleIdx = 0;
	struct net_device *prDev;

	prDev = g_aprNanMultiDev[ucRoleIdx].prDevHandler;

	prGlueInfo =
		(prDev != NULL)
			? *((struct GLUE_INFO **)netdev_priv(prDev))
			: NULL;
	if (g_aprNanMultiDev[ucRoleIdx].fgBMCFilterSet == FALSE)
		return;

	if (!prDev) {
		DBGLOG(NAN, ERROR, "prDev error!\n");
		return;
	}
	if (!prGlueInfo) {
		DBGLOG(NAN, ERROR, "prGlueInfo error!\n");
		return;
	}

	if (!prDev || !prGlueInfo) {
		DBGLOG(INIT, WARN,
		       " abnormal dev or skb: prDev(0x%p), prGlueInfo(0x%p)\n",
		       prDev, prGlueInfo);
		return;
	}

	if (prDev->flags & IFF_PROMISC)
		u4PacketFilter |= PARAM_PACKET_FILTER_PROMISCUOUS;

	if (prDev->flags & IFF_BROADCAST)
		u4PacketFilter |= PARAM_PACKET_FILTER_BROADCAST;
	u4McCount = netdev_mc_count(prDev);

	if (prDev->flags & IFF_MULTICAST) {
		if ((prDev->flags & IFF_ALLMULTI) ||
		    (u4McCount > MAX_NUM_GROUP_ADDR))
			u4PacketFilter |=
				PARAM_PACKET_FILTER_ALL_MULTICAST;
		else
			u4PacketFilter |= PARAM_PACKET_FILTER_MULTICAST;
	}

	if (u4PacketFilter & PARAM_PACKET_FILTER_MULTICAST) {
		/* Prepare multicast address list */
		struct netdev_hw_addr *ha;
		uint8_t *prMCAddrList = NULL;
		uint32_t i = 0;

		prMCAddrList = kalMemAlloc(
			MAX_NUM_GROUP_ADDR * ETH_ALEN, VIR_MEM_TYPE);

		netdev_for_each_mc_addr(ha, prDev) {
			if (i < MAX_NUM_GROUP_ADDR) {
				kalMemCopy(
					(prMCAddrList + i * ETH_ALEN),
					GET_ADDR(ha), ETH_ALEN);
				DBGLOG(NAN, INFO,
				       "SEt Multicast Address List "
				       MACSTR "\n",
				       MAC2STR(GET_ADDR(ha)));
				i++;
			}
		}
		if (i >= MAX_NUM_GROUP_ADDR)
			return;

		wlanoidSetNANMulticastList(
			prGlueInfo->prAdapter,
			wlanGetBssIdxByNetInterface(prGlueInfo, prDev),
			prMCAddrList, (i * ETH_ALEN), &u4SetInfoLen);

		kalMemFree(prMCAddrList, VIR_MEM_TYPE,
			   MAX_NUM_GROUP_ADDR * ETH_ALEN);
	}
	g_aprNanMultiDev[ucRoleIdx].fgBMCFilterSet = FALSE;
} /* end of p2pSetMulticastList() */

void
nanSetMulticastListWorkQueueWrapper(struct GLUE_INFO *prGlueInfo)
{

	if (!prGlueInfo) {
		DBGLOG(INIT, WARN, "abnormal dev or skb: prGlueInfo(0x%p)\n",
		       prGlueInfo);
		return;
	}

	if (prGlueInfo->prAdapter->fgIsNANRegistered)
		mtk_nan_wext_set_Multicastlist(prGlueInfo);
}

/*----------------------------------------------------------------------------*/
/*!
 * \brief Register for cfg80211 for NAN
 *
 * \param[in] prGlueInfo      Pointer to glue info
 *
 * \return   TRUE
 *           FALSE
 */
/*----------------------------------------------------------------------------*/
unsigned char
glRegisterNAN(struct GLUE_INFO *prGlueInfo, const char *prDevName)
{
	struct ADAPTER *prAdapter = NULL;
	uint8_t rMacAddr[6];
	struct wireless_dev *prNanWdev = NULL;
	struct net_device *prNanDev = NULL;
	struct wiphy *prWiphy = NULL;
	const char *prSetDevName;
#if (CFG_ENABLE_UNIFY_WIPHY == 0)
	struct GL_HIF_INFO *prHif = NULL;
	struct device *prDev;
#endif
	struct _GL_NAN_INFO_T *prNANInfo = (struct _GL_NAN_INFO_T *)NULL;
	enum NAN_BSS_ROLE_INDEX eRole = NAN_BSS_INDEX_BAND0;

	if (!prGlueInfo) {
		DBGLOG(NAN, ERROR, "prGlueInfo error!\n");
		return FALSE;
	}

	prAdapter = prGlueInfo->prAdapter;

	glNanCreateWirelessDevice(prGlueInfo);
	if (!g_aprNanRoleWdev[eRole]) {
		DBGLOG(INIT, ERROR, "gprNanWdev is NULL\n");
		return FALSE;
	}

	DBGLOG(INIT, INFO, "gprNanWdev\n");
	prNanWdev = g_aprNanRoleWdev[eRole];
	prWiphy = prNanWdev->wiphy;
	memset(prNanWdev, 0, sizeof(struct wireless_dev));
	prNanWdev->wiphy = prWiphy;

	prSetDevName = prDevName;
/* allocate netdev */
#if KERNEL_VERSION(3, 17, 0) <= CFG80211_VERSION_CODE
	prNanDev = alloc_netdev_mq(
		sizeof(struct NETDEV_PRIVATE_GLUE_INFO), prSetDevName,
		NET_NAME_PREDICTABLE, ether_setup, CFG_MAX_TXQ_NUM);
#else
	prNanDev = alloc_netdev_mq(
		sizeof(struct NETDEV_PRIVATE_GLUE_INFO), prSetDevName,
		ether_setup, CFG_MAX_TXQ_NUM);
#endif
	if (!prNanDev) {
		DBGLOG(INIT, WARN, "unable to allocate ndev for nan\n");
		goto err_alloc_netdev;
	}

	g_aprNanMultiDev[eRole].prDevHandler = prNanDev;
	g_aprNanMultiDev[eRole].fgBMCFilterSet = FALSE;

	/* fill hardware address */
	COPY_MAC_ADDR(rMacAddr, prAdapter->rMyMacAddr);
	rMacAddr[0] |= 0x2;
	if (random_mac_addr_keep_oui(rMacAddr) == -1)
		DBGLOG(INIT, ERROR, "unable to get random mac for nan\n");

	/* change to local administrated address */
	rMacAddr[0] ^= (eRole + 1) << 3;
	kalMemCopy(prNanDev->dev_addr, rMacAddr, ETH_ALEN);
	kalMemCopy(prNanDev->perm_addr, prNanDev->dev_addr, ETH_ALEN);

	if (glSetupNAN(prGlueInfo, prNanWdev, prNanDev, eRole) != 0) {
		DBGLOG(INIT, WARN, "glSetupnan FAILED\n");
		free_netdev(prNanDev);
		return FALSE;
	}

	/* initialize NAN Data Engine */

	prNANInfo = prAdapter->prGlueInfo->aprNANDevInfo[NAN_BSS_INDEX_BAND0];
	nanDataEngineInit(prAdapter, prNANInfo->prDevHandler->dev_addr);

	/* initialize NAN Ranging Engine */
	nanRangingEngineInit(prAdapter);

	/* initialize NAN Security Engine */
	nan_sec_wpa_supplicant_start();
	/*
	 * Send request to CNM module
	 *	- If DBDC is going to be enabled/disabled, the request will
	 *	  be held to wait for DBDC switch done.
	 */
	nanDevSendEnableRequestToCnm(prAdapter);

	prAdapter->rNanNetRegState = ENUM_NET_REG_STATE_UNREGISTERED;

	return TRUE;
err_alloc_netdev:
	return FALSE;
} /* end of glRegisterNAN() */

#if CFG_ENABLE_UNIFY_WIPHY
unsigned char
glNanCreateWirelessDevice(struct GLUE_INFO *prGlueInfo)
{
	/* whsu, KAL_AIS_NUM at gprWdev */
	struct wiphy *prWiphy = wlanGetWiphy();
	struct wireless_dev *prWdev = NULL;
	enum NAN_BSS_ROLE_INDEX eRole = NAN_BSS_INDEX_BAND0;

	if (!prWiphy) {
		DBGLOG(NAN, ERROR, "unable to allocate wiphy for NAN\n");
		return FALSE;
	}

	prWdev = kzalloc(sizeof(struct wireless_dev), GFP_KERNEL);
	if (!prWdev) {
		DBGLOG(NAN, ERROR, "allocate p2p wdev fail, no memory\n");
		return FALSE;
	}

	/* set priv as pointer to glue structure */
	prWdev->wiphy = prWiphy;

	g_aprNanRoleWdev[eRole] = prWdev;
	DBGLOG(NAN, INFO, "glNanCreateWirelessDevice (%x) %d\n",
	       g_aprNanRoleWdev[eRole]->wiphy, eRole);

	return TRUE;
}
#endif
/*----------------------------------------------------------------------------*/
/*!
 * \brief Unregister Net Device for NAN
 *
 * \param[in] prGlueInfo      Pointer to glue info
 *	[in] ucIdx	     The BSS with the idx will be freed.
 *			     "ucIdx == 0xff" will free all BSSs.
 *			     Only has meaning for "CFG_ENABLE_UNIFY_WIPHY == 1"
 *
 * \return   TRUE
 *           FALSE
 */
/*----------------------------------------------------------------------------*/
unsigned char
glUnregisterNAN(struct GLUE_INFO *prGlueInfo)
{
	struct ADAPTER *prAdapter;
	struct _GL_NAN_INFO_T *prNANInfo = NULL;
	uint8_t ucIdx = NAN_BSS_INDEX_BAND0;

	if (!prGlueInfo) {
		DBGLOG(NAN, ERROR, "prGlueInfo error!\n");
		return FALSE;
	}

	prAdapter = prGlueInfo->prAdapter;

	if (prAdapter->fgIsNanSendRequestToCnm)
		nanDevSendAbortRequestToCnm(prAdapter);

	nanDevDisableRequest(prAdapter);

	/* uninitialize NAN Data Engine */
	nanDataEngineUninit(prAdapter);

	/* uninitialize NAN Data Engine */
	nanRangingEngineUninit(prAdapter);
	/* uninitialize NAN SEC Engine */
	nan_sec_hostapd_deinit();
	/* Clear pending cipher suite */
	nanSecFlushCipherList();

	/* 4 <1> Uninit NAN dev FSM
	 * Uninit NAN device FSM
	 * only do nanDevFsmUninit, when unregister all nan device
	 */
	nanDevFsmUninit(prGlueInfo->prAdapter, ucIdx);

	/* 4 <3> Free Wiphy & netdev */
	prNANInfo = prGlueInfo->aprNANDevInfo[ucIdx];
	if (prNANInfo == NULL)
		return TRUE;

	{
		/* don't unregister the dev that share with the AIS */
		uint32_t u4Idx = 0;

		for (u4Idx = 0; u4Idx < KAL_AIS_NUM; u4Idx++) {

			if (gprWdev[u4Idx] &&
			    prNANInfo->prDevHandler ==
				    gprWdev[u4Idx]->netdev) {
				free_netdev(prNANInfo->prDevHandler);
				prNANInfo->prDevHandler = NULL;
			}
		}
	}
	/* 4 <4> Free P2P internal memory */
	if (!nanFreeInfo(prGlueInfo, ucIdx)) {
		DBGLOG(INIT, ERROR, "nanFreeInfo FAILED\n");
		return FALSE;
	}

	return TRUE;
} /* end of glUnregisterP2P() */

/*----------------------------------------------------------------------------*/
/*!
 * \brief
 *       run nan init procedure, glue register nan and set nan registered flag
 *
 * \retval 1     Success
 */
/*----------------------------------------------------------------------------*/
unsigned char
nanLaunch(struct GLUE_INFO *prGlueInfo)
{
	if (prGlueInfo->prAdapter->fgIsNANRegistered == TRUE) {
		DBGLOG(NAN, INFO, "NAN is already registered\n");
		return FALSE;
	}

	if (!glRegisterNAN(prGlueInfo, nifname)) {
		DBGLOG(NAN, ERROR, "Launch failed\n");
		return FALSE;
	}

	prGlueInfo->prAdapter->fgIsNANRegistered = TRUE;

	DBGLOG(NAN, INFO, "Launch success, fgIsNANRegistered TRUE\n");
	return TRUE;
}

/*----------------------------------------------------------------------------*/
/*!
 * \brief
 *       run NAN exit procedure, glue unregister NAN and set NAN registered flag
 *
 * \retval 1     Success
 */
/*----------------------------------------------------------------------------*/
unsigned char
nanRemove(struct GLUE_INFO *prGlueInfo)
{
	uint8_t ucIdx = NAN_BSS_INDEX_BAND0;

	if (prGlueInfo->prAdapter->fgIsNANRegistered == FALSE) {
		DBGLOG(NAN, INFO, "nan is not registered\n");
		return FALSE;
	}

	DBGLOG(NAN, INFO, "fgIsNANRegistered FALSE\n");
	prGlueInfo->prAdapter->fgIsNANRegistered = FALSE;

	glUnregisterNAN(prGlueInfo);

	/* Release nan wdev. */

	if (g_aprNanRoleWdev[ucIdx] == NULL)
		return TRUE;

#if CFG_ENABLE_UNIFY_WIPHY
{
	/* don't unregister the dev that share with the AIS */
	uint32_t u4Idx = 0;

	for (u4Idx = 0; u4Idx < KAL_AIS_NUM; u4Idx++) {
		if (gprWdev[u4Idx] &&
		    g_aprNanRoleWdev[ucIdx] == gprWdev[u4Idx]) {
			/* This is AIS/AP Interface */
			g_aprNanRoleWdev[ucIdx] = NULL;
			continue;
		}
	}
}
#endif

	DBGLOG(INIT, INFO, "Unregister g_aprNanRoleWdev[%d]\n", ucIdx);

	kfree(g_aprNanRoleWdev[ucIdx]);
	g_aprNanRoleWdev[ucIdx] = NULL;
	return TRUE;
}
void
nanSetSuspendMode(struct GLUE_INFO *prGlueInfo, unsigned char fgEnable)
{
	struct net_device *prDev = NULL;

	if (!prGlueInfo)
		return;

	if (!prGlueInfo->prAdapter->fgIsNANRegistered) {
		DBGLOG(NAN, INFO, "%s: NAN is not enabled, SKIP!\n", __func__);
		return;
	}

	prDev = prGlueInfo->aprNANDevInfo[0]->prDevHandler;
	if (!prDev) {
		DBGLOG(NAN, INFO, "%s: NAN  dev is not available, SKIP!\n",
		       __func__);
		return;
	}

	kalSetNetAddressFromInterface(prGlueInfo, prDev, fgEnable);
	wlanNotifyFwSuspend(prGlueInfo, prDev, fgEnable);
}

/* Net Device Hooks */
/*----------------------------------------------------------------------------*/
/*!
 * \brief A function for net_device open (ifup)
 *
 * \param[in] prDev      Pointer to struct net_device.
 *
 * \retval 0     The execution succeeds.
 * \retval < 0   The execution failed.
 */
/*----------------------------------------------------------------------------*/
static int
nanOpen(IN struct net_device *prDev)
{
	/* P_GLUE_INFO_T prGlueInfo = NULL; */
	/* P_ADAPTER_T prAdapter = NULL; */
	/* P_MSG_P2P_FUNCTION_SWITCH_T prFuncSwitch; */

	if (!prDev) {
		DBGLOG(NAN, ERROR, "prDev error!\n");
		return -1;
	}

	/* 2. carrier on & start TX queue */
	/*DFS todo 20161220_DFS*/

	netif_tx_start_all_queues(prDev);

	return 0; /* success */
} /* end of p2pOpen() */

/*----------------------------------------------------------------------------*/
/*!
 * \brief A function for net_device stop (ifdown)
 *
 * \param[in] prDev      Pointer to struct net_device.
 *
 * \retval 0     The execution succeeds.
 * \retval < 0   The execution failed.
 */
/*----------------------------------------------------------------------------*/
static int
nanStop(IN struct net_device *prDev)
{
	struct GLUE_INFO *prGlueInfo = NULL;
	struct ADAPTER *prAdapter = NULL;
	struct _GL_NAN_INFO_T *prNANGlueDevInfo = (struct _GL_NAN_INFO_T *)NULL;
	/* P_MSG_P2P_FUNCTION_SWITCH_T prFuncSwitch; */

	if (!prDev) {
		DBGLOG(NAN, ERROR, "prDev error!\n");
		return -EFAULT;
	}

	prGlueInfo = *((struct GLUE_INFO **)netdev_priv(prDev));
	if (!prGlueInfo) {
		DBGLOG(NAN, ERROR, "prGlueInfo error!\n");
		return -EFAULT;
	}

	prAdapter = prGlueInfo->prAdapter;
	if (!prAdapter) {
		DBGLOG(NAN, ERROR, "prAdapter error!\n");
		return -EFAULT;
	}

	if (!prAdapter->fgIsNANRegistered) {
		DBGLOG(INIT, WARN, "fgIsNANRegistered == 0, and return\n");
		return -EFAULT;
	}

	prNANGlueDevInfo = prGlueInfo->aprNANDevInfo[0];
	if (!prNANGlueDevInfo) {
		DBGLOG(NAN, ERROR, "prNANGlueDevInfo error!\n");
		return -EFAULT;
	}

	/* 0. Do the scan done and set parameter to abort if the scan pending
	 * DBGLOG(INIT, INFO, "p2pStop and ucRoleIdx = %u\n", ucRoleIdx);
	 * TODO flush the scan request
	 * 1. stop TX queue
	 * 3. stop queue and turn off carrier
	 * prGlueInfo->prP2PInfo[0]->eState = PARAM_MEDIA_STATE_DISCONNECTED;
	 */

	netif_tx_stop_all_queues(prDev);
	if (netif_carrier_ok(prDev))
		netif_carrier_off(prDev);

	return 0;
} /* end of p2pStop() */

/*----------------------------------------------------------------------------*/
/*!
 * \brief A method of struct net_device, to get the network interface
 *        statistical information.
 *
 * Whenever an application needs to get statistics for the interface, this
 * method is called.
 * This happens, for example, when ifconfig or netstat -i is run.
 *
 * \param[in] prDev      Pointer to struct net_device.
 *
 * \return net_device_stats buffer pointer.
 */
/*----------------------------------------------------------------------------*/
struct net_device_stats *
nanGetStats(IN struct net_device *prDev)
{
	return (struct net_device_stats *)kalGetStats(prDev);
} /* end of nanGetStats() */

static void
nanSetMulticastList(IN struct net_device *prDev)
{
	struct GLUE_INFO *prGlueInfo = (struct GLUE_INFO *)NULL;
	uint8_t ucRoleIdx = NAN_BSS_INDEX_BAND0;

	prGlueInfo = (prDev != NULL)
			     ? *((struct GLUE_INFO **)netdev_priv(prDev))
			     : NULL;

	if (!prDev || !prGlueInfo) {
		DBGLOG(NAN, WARN,
		       " abnormal dev or skb: prDev(0x%p), prGlueInfo(0x%p)\n",
		       prDev, prGlueInfo);
		return;
	}
	/* TO-DO MulticastList Support */
	if (g_aprNanMultiDev[ucRoleIdx].fgBMCFilterSet == FALSE) {
		g_aprNanMultiDev[ucRoleIdx].fgBMCFilterSet = TRUE;
		/* Mark HALT, notify main thread to
		 * finish current job
		 */
		set_bit(GLUE_FLAG_NAN_MULTICAST_BIT,
			&prGlueInfo->ulFlag);
		/* wake up main thread */
		wake_up_interruptible(&prGlueInfo->waitq);
	}
}

/*----------------------------------------------------------------------------*/
/*!
 * \brief This function is TX entry point of NET DEVICE.
 *
 * \param[in] prSkb  Pointer of the sk_buff to be sent
 * \param[in] prDev  Pointer to struct net_device
 *
 * \retval NETDEV_TX_OK - on success.
 * \retval NETDEV_TX_BUSY - on failure, packet will be discarded
 *                          by upper layer.
 */
/*----------------------------------------------------------------------------*/
netdev_tx_t
nanHardStartXmit(IN struct sk_buff *prSkb, IN struct net_device *prDev)
{
	struct NETDEV_PRIVATE_GLUE_INFO *prNetDevPrivate =
		(struct NETDEV_PRIVATE_GLUE_INFO *)NULL;
	struct GLUE_INFO *prGlueInfo = NULL;
	uint8_t ucBssIndex;
	struct TX_PACKET_INFO prTxPktInfo;
	struct STA_RECORD *prStaRec;

	if (!prSkb) {
		DBGLOG(NAN, ERROR, "prSkb error!\n");
		return NETDEV_TX_BUSY;
	}
	if (!prDev) {
		DBGLOG(NAN, ERROR, "prDev error!\n");
		return NETDEV_TX_BUSY;
	}

	prNetDevPrivate = (struct NETDEV_PRIVATE_GLUE_INFO *)netdev_priv(prDev);
	prGlueInfo = prNetDevPrivate->prGlueInfo;
	ucBssIndex = prNetDevPrivate->ucBssIdx;

#if (CFG_SUPPORT_DBDC == 1)
	if (kalQoSFrameClassifierAndPacketInfo(
			prGlueInfo, prSkb, &prTxPktInfo)) {

		if (IS_BMCAST_MAC_ADDR(prTxPktInfo.aucEthDestAddr)) {
			DBGLOG(NAN, LOUD, "TX with DA = BMCAST\n");
		} else {
			prStaRec = nanGetStaRecByNDI(prGlueInfo->prAdapter,
				prTxPktInfo.aucEthDestAddr);
			if (prStaRec != NULL) {
				ucBssIndex = prStaRec->ucBssIndex;
				DBGLOG(NAN, LOUD, "Starec bssIndex:%d\n",
							ucBssIndex);
			}
		}
	}
#endif

	kalResetPacket(prGlueInfo, (void *)prSkb);

	kalHardStartXmit(prSkb, prDev, prGlueInfo, ucBssIndex);

	return NETDEV_TX_OK;
} /* end of p2pHardStartXmit() */

/*----------------------------------------------------------------------------*/
/*!
 * \brief A method of struct net_device, a primary SOCKET interface to configure
 *        the interface lively. Handle an ioctl call on one of our devices.
 *        Everything Linux ioctl specific is done here. Then we pass the
 *        contents of the ifr->data to the request message handler.
 *
 * \param[in] prDev      Linux kernel netdevice
 *
 * \param[in] prIFReq    Our private ioctl request structure, typed for the
 *                       generic struct ifreq so we can use ptr to function
 *
 * \param[in] cmd        Command ID
 *
 * \retval WLAN_STATUS_SUCCESS The IOCTL command is executed successfully.
 * \retval OTHER The execution of IOCTL command is failed.
 */
/*----------------------------------------------------------------------------*/

int
nanDoIOCTL(struct net_device *prDev, struct ifreq *prIfReq, int i4Cmd)
{
	struct GLUE_INFO *prGlueInfo = NULL;
	int ret = 0;
	/* char *prExtraBuf = NULL; */
	/* UINT_32 u4ExtraSize = 0; */
	struct iwreq *prIwReq = (struct iwreq *)prIfReq;
	struct iw_request_info rIwReqInfo;
	/* fill rIwReqInfo */
	rIwReqInfo.cmd = (__u16)i4Cmd;
	rIwReqInfo.flags = 0;

	if (!prDev) {
		DBGLOG(NAN, ERROR, "prDev error!\n");
		return -EFAULT;
	}
	if (!prIfReq) {
		DBGLOG(NAN, ERROR, "prIfReq error!\n");
		return -EFAULT;
	}


	DBGLOG(NAN, ERROR, "NANDoIOCTL In %x %x\n", i4Cmd, SIOCDEVPRIVATE);

	prGlueInfo = *((struct GLUE_INFO **)netdev_priv(prDev));
	if (!prGlueInfo) {
		DBGLOG(NAN, ERROR, "prGlueInfo is NULL\n");
		return -EFAULT;
	}

	if (prGlueInfo->u4ReadyFlag == 0) {
		DBGLOG(NAN, ERROR, "Adapter is not ready\n");
		return -EINVAL;
	}

	if (i4Cmd == IOCTL_GET_DRIVER)
		ret = priv_support_driver_cmd(prDev, prIfReq, i4Cmd);
	else if (i4Cmd == SIOCGIWPRIV)
		ret = mtk_nan_wext_get_priv(prDev, &rIwReqInfo, &(prIwReq->u),
					    NULL);
#ifdef CFG_ANDROID_AOSP_PRIV_CMD
	else if (i4Cmd == SIOCDEVPRIVATE + 1)
		ret = android_private_support_driver_cmd(prDev, prIfReq, i4Cmd);
#endif
	else {
		DBGLOG(INIT, WARN, "Unexpected ioctl command: 0x%04x\n", i4Cmd);
		ret = -1;
	}

#if 0
	/* fill rIwReqInfo */
	rIwReqInfo.cmd = (__u16) i4Cmd;
	rIwReqInfo.flags = 0;

	switch (i4Cmd) {
	case SIOCSIWENCODEEXT:
		/* Set Encryption Material after 4-way handshaking is done */
		if (prIwReq->u.encoding.pointer) {
			u4ExtraSize = prIwReq->u.encoding.length;
			prExtraBuf = kalMemAlloc(u4ExtraSize, VIR_MEM_TYPE);

			if (!prExtraBuf) {
				ret = -ENOMEM;
				break;
			}

			if (copy_from_user(prExtraBuf,
					   prIwReq->u.encoding.pointer,
					   prIwReq->u.encoding.length))
				ret = -EFAULT;
		} else if (prIwReq->u.encoding.length != 0) {
			ret = -EINVAL;
			break;
		}

		if (ret == 0)
			ret = mtk_p2p_wext_set_key(prDev,
						   &rIwReqInfo,
						   &(prIwReq->u), prExtraBuf);

		kalMemFree(prExtraBuf, VIR_MEM_TYPE, u4ExtraSize);
		prExtraBuf = NULL;
		break;

	case SIOCSIWMLME:
		/* IW_MLME_DISASSOC used for disconnection */
		if (prIwReq->u.data.length != sizeof(struct iw_mlme)) {
			DBGLOG(INIT, INFO, "MLME buffer strange:%d\n",
			       prIwReq->u.data.length);
			ret = -EINVAL;
			break;
		}

		if (!prIwReq->u.data.pointer) {
			ret = -EINVAL;
			break;
		}

		prExtraBuf = kalMemAlloc(sizeof(struct iw_mlme), VIR_MEM_TYPE);
		if (!prExtraBuf) {
			ret = -ENOMEM;
			break;
		}

		if (copy_from_user(prExtraBuf,
				   prIwReq->u.data.pointer,
				   sizeof(struct iw_mlme)))
			ret = -EFAULT;
		else
			ret = mtk_p2p_wext_mlme_handler(prDev, &rIwReqInfo,
							&(prIwReq->u),
							prExtraBuf);

		kalMemFree(prExtraBuf, VIR_MEM_TYPE, sizeof(struct iw_mlme));
		prExtraBuf = NULL;
		break;

	case SIOCGIWPRIV:
		/* This ioctl is used to list all IW privilege ioctls */
		ret = mtk_p2p_wext_get_priv(prDev, &rIwReqInfo,
					    &(prIwReq->u), NULL);
		break;

	case SIOCGIWSCAN:
		ret = mtk_p2p_wext_discovery_results(prDev, &rIwReqInfo,
						     &(prIwReq->u), NULL);
		break;

	case SIOCSIWAUTH:
		ret = mtk_p2p_wext_set_auth(prDev, &rIwReqInfo,
						   &(prIwReq->u), NULL);
		break;

	case IOC_P2P_CFG_DEVICE:
	case IOC_P2P_PROVISION_COMPLETE:
	case IOC_P2P_START_STOP_DISCOVERY:
	case IOC_P2P_DISCOVERY_RESULTS:
	case IOC_P2P_WSC_BEACON_PROBE_RSP_IE:
	case IOC_P2P_CONNECT_DISCONNECT:
	case IOC_P2P_PASSWORD_READY:
	case IOC_P2P_GET_STRUCT:
	case IOC_P2P_SET_STRUCT:
	case IOC_P2P_GET_REQ_DEVICE_INFO:
#if 0
		ret = rP2PIwPrivHandler[i4Cmd - SIOCIWFIRSTPRIV](prDev,
				&rIwReqInfo,
				&(prIwReq->u),
				(char *) &(prIwReq->u));
#endif
		break;
#if CFG_SUPPORT_P2P_RSSI_QUERY
	case SIOCGIWSTATS:
		ret = mtk_p2p_wext_get_rssi(prDev, &rIwReqInfo,
					    &(prIwReq->u), NULL);
		break;
#endif
	default:
		ret = -ENOTTY;
	}
#endif /* 0 */

	return ret;
} /* end of p2pDoIOCTL() */

/*----------------------------------------------------------------------------*/
/*!
 * \brief To report the private supported IOCTLs table to user space.
 *
 * \param[in] prDev Net device requested.
 * \param[out] prIfReq Pointer to ifreq structure, content is copied back to
 *                  user space buffer in gl_iwpriv_table.
 *
 * \retval 0 For success.
 * \retval -E2BIG For user's buffer size is too small.
 * \retval -EFAULT For fail.
 *
 */
/*----------------------------------------------------------------------------*/
int
mtk_nan_wext_get_priv(IN struct net_device *prDev,
		      IN struct iw_request_info *info,
		      IN OUT union iwreq_data *wrqu, IN OUT char *extra)
{
	struct iw_point *prData = (struct iw_point *)&wrqu->data;
	uint16_t u2BufferSize = 0;

	if (!prDev) {
		DBGLOG(NAN, ERROR, "prDev error!\n");
		return -EFAULT;
	}

	u2BufferSize = prData->length;

	/* update our private table size */
	prData->length =
		(__u16)sizeof(rNANIwPrivTable) / sizeof(struct iw_priv_args);

	if (u2BufferSize < prData->length)
		return -E2BIG;

	if (prData->length) {
		if (copy_to_user(prData->pointer, rNANIwPrivTable,
				 sizeof(rNANIwPrivTable)))
			return -EFAULT;
	}

	return 0;
} /* end of mtk_p2p_wext_get_priv() */
