/* SPDX-License-Identifier: GPL-2.0 */

/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#include "wpa_supp/FourWayHandShake.h"
#include "wpa_supp/src/ap/wpa_auth_glue.h"

/* #include "wifi_var.h" */

#include "nan/nan_sec.h"
#include "nan/nan_data_engine.h"

/*******************************************************************************
 *                         C O M P I L E R   F L A G S
 *******************************************************************************
 */

/*******************************************************************************
 *                    E X T E R N A L   R E F E R E N C E S
 *******************************************************************************
 */

/*******************************************************************************
 *                              C O N S T A N T S
 *******************************************************************************
 */

/*******************************************************************************
 *                             D A T A   T Y P E S
 *******************************************************************************
 */
struct _NAN_SEC_CTX {
	struct wpa_supplicant rNanWpaSupp;
	struct hostapd_data rNanHapdData;

	struct QUE rNanSecCipherList;
	uint8_t *pu1CsidAttrBuf;
	uint32_t u4CsidAttrLen;
};

/*******************************************************************************
 *                            P U B L I C   D A T A
 *******************************************************************************
 */

/* struct _NAN_NDP_SUDO        g_rNanNdpSudo[MAX_NDP_NUM]; */
struct _NAN_SEC_CTX g_rNanSecCtx;

struct wpa_supplicant *g_prNanWpaSupp = &g_rNanSecCtx.rNanWpaSupp;
struct hostapd_data *g_prNanHapdData = &g_rNanSecCtx.rNanHapdData;

struct wpa_sm g_arNanWpaSm[NAN_MAX_SUPPORT_NDL_NUM * NAN_MAX_SUPPORT_NDP_NUM];
struct wpa_sm_ctx g_rNanWpaSmCtx;

/* struct sta_info             g_arNanStaInfo[MAX_NDP_NUM]; */
struct wpa_state_machine
	g_arNanWpaAuthSm[NAN_MAX_SUPPORT_NDL_NUM * NAN_MAX_SUPPORT_NDP_NUM];
struct wpa_authenticator g_rNanWpaAuth;

/*******************************************************************************
 *                           P R I V A T E   D A T A
 *******************************************************************************
 */
uint8_t g_aucNanSecAttrBuffer[NAN_IE_BUF_MAX_SIZE];

/*******************************************************************************
 *                                 M A C R O S
 *******************************************************************************
 */

/*******************************************************************************
 *                   F U N C T I O N   D E C L A R A T I O N S
 *******************************************************************************
 */

/*******************************************************************************
 *                              F U N C T I O N S
 *******************************************************************************
 */

/************************************************
 *               Set Key Related
 ************************************************
 */
uint32_t
nan_sec_wlanSetAddKey(IN struct ADAPTER *prAdapter, IN void *pvSetBuffer,
		      IN uint32_t u4SetBufferLen) {

	struct CMD_802_11_KEY *prCmdFWKey;
	struct CMD_INFO *prCmdInfo;
	struct CMD_802_11_KEY *prCmdKey;
	uint8_t ucCmdSeqNum;
	struct BSS_INFO *prBssInfo;
	struct STA_RECORD *prStaRec = NULL;
	unsigned char fgNoHandshakeSec = FALSE;
	struct mt66xx_chip_info *prChipInfo;
	uint16_t cmd_size;

	prCmdFWKey = (struct CMD_802_11_KEY *)pvSetBuffer;
	DEBUGFUNC("wlanSetAddKey");
	DBGLOG(REQ, LOUD, "\n");

	if (!prAdapter) {
		DBGLOG(NAN, ERROR, "prAdapter error!\n");
		return WLAN_STATUS_FAILURE;
	}
	if (!pvSetBuffer) {
		DBGLOG(NAN, ERROR, "pvSetBuffer error!\n");
		return WLAN_STATUS_FAILURE;
	}

	DBGLOG(RSN, INFO, "wlanoidSetFWAddKey\n");

	prBssInfo = GET_BSS_INFO_BY_INDEX(prAdapter, prCmdFWKey->ucBssIdx);

	if (!prBssInfo) {
		DBGLOG(REQ, INFO, "BSS Info not exist !!\n");
		return WLAN_STATUS_SUCCESS;
	}

/*        Tx  Rx   KeyType addr
 * STA, GC:
 * case1:  1    1    0    BC addr (no sta record of AP at this moment)  WEP,
 * notice: tx at default key setting WEP key now save to BSS_INFO
 * case2:  0    1    0    BSSID (sta record of AP)     RSN BC key
 * case3:  1    1    1    AP addr (sta record of AP)   RSN STA key
 *
 * GO:
 * case1:  1    1    0    BSSID (no sta record)     WEP -- Not support
 * case2:  1    0    0    BSSID (no sta record)     RSN BC key
 * case3:  1    1    1    STA addr                  STA key
 */

	/* ucKeyType; */
	if (prCmdFWKey->ucKeyType) {
		prStaRec =
			cnmGetStaRecByAddress(prAdapter, prBssInfo->ucBssIndex,
					      prCmdFWKey->aucPeerAddr);
		if (!prStaRec) { /* Already disconnected ? */
			DBGLOG(NAN, INFO,
			       "[wlan] No sta_rec, bssIdx:%d, PeerAddr:" MACSTR
			       "\n",
			       prBssInfo->ucBssIndex,
			       MAC2STR(prCmdFWKey->aucPeerAddr));

			return WLAN_STATUS_SUCCESS;
		}
	}

	prChipInfo = prAdapter->chip_info;

	if (!prChipInfo) {
		DBGLOG(NAN, ERROR, "prChipInfo error!\n");
		return WLAN_STATUS_FAILURE;
	}
	cmd_size = prChipInfo->u2CmdTxHdrSize + sizeof(struct CMD_802_11_KEY);

	prCmdInfo = cmdBufAllocateCmdInfo(prAdapter, cmd_size);

	if (!prCmdInfo) {
		DBGLOG(INIT, ERROR, "Allocate CMD_INFO_T ==> FAILED.\n");
		return WLAN_STATUS_FAILURE;
	}
	/* increase command sequence number */
	ucCmdSeqNum = nicIncreaseCmdSeqNum(prAdapter);
	DBGLOG(REQ, INFO, "ucCmdSeqNum = %d\n", ucCmdSeqNum);

	/* compose CMD_802_11_KEY cmd pkt */
	prCmdInfo->eCmdType = COMMAND_TYPE_NETWORK_IOCTL;
	prCmdInfo->u2InfoBufLen = cmd_size;
#if CFG_SUPPORT_REPLAY_DETECTION
	prCmdInfo->pfCmdDoneHandler = nicCmdEventSetAddKey;
	prCmdInfo->pfCmdTimeoutHandler = nicOidCmdTimeoutSetAddKey;
#else
	prCmdInfo->pfCmdDoneHandler = NULL;
	prCmdInfo->pfCmdTimeoutHandler = NULL;
#endif
	prCmdInfo->fgIsOid = FALSE;
	prCmdInfo->ucCID = CMD_ID_ADD_REMOVE_KEY;
	prCmdInfo->fgSetQuery = TRUE;
	prCmdInfo->fgNeedResp = FALSE;
	prCmdInfo->ucCmdSeqNum = ucCmdSeqNum;
	prCmdInfo->u4SetInfoLen = u4SetBufferLen;
	prCmdInfo->pvInformationBuffer = pvSetBuffer;
	prCmdInfo->u4InformationBufferLength = u4SetBufferLen;

	NIC_FILL_CMD_TX_HDR(prAdapter, prCmdInfo->pucInfoBuffer,
			    prCmdInfo->u2InfoBufLen, prCmdInfo->ucCID,
			    CMD_PACKET_TYPE_ID, &prCmdInfo->ucCmdSeqNum,
			    prCmdInfo->fgSetQuery, &prCmdKey, FALSE, 0,
			    S2D_INDEX_CMD_H2N);

	/* Setup WIFI_CMD_T */
	kalMemZero(prCmdKey, sizeof(struct CMD_802_11_KEY));

	prCmdKey->ucAddRemove = 1; /* Add */

	prCmdKey->ucTxKey = prCmdFWKey->ucTxKey;
	prCmdKey->ucKeyType = prCmdFWKey->ucKeyType;
	prCmdKey->ucIsAuthenticator = prCmdFWKey->ucIsAuthenticator;
	prCmdKey->ucAlgorithmId = prCmdFWKey->ucAlgorithmId;

	prCmdKey->ucBssIdx = prCmdFWKey->ucBssIdx;
	prCmdKey->ucKeyId = prCmdFWKey->ucKeyId;

	/* Note: the key length may not correct for WPA-None */
	prCmdKey->ucKeyLen = prCmdFWKey->ucKeyLen;

	kalMemCopy(prCmdKey->aucKeyMaterial, prCmdFWKey->aucKeyMaterial,
		   prCmdKey->ucKeyLen);

	if (prStaRec) {
		if (prCmdKey->ucKeyType) { /* RSN STA */
			struct WLAN_TABLE *prWtbl;

			prWtbl = prAdapter->rWifiVar.arWtbl;
			prWtbl[prStaRec->ucWlanIndex].ucKeyId =
				prCmdKey->ucKeyId;

			prCmdKey->ucWlanIndex =
				prStaRec->ucWlanIndex;
			prStaRec->fgTransmitKeyExist =
				TRUE; /* wait for CMD Done ? */
			kalMemCopy(prCmdKey->aucPeerAddr,
				   prCmdFWKey->aucPeerAddr,
				   MAC_ADDR_LEN);
		} else {
			DBGLOG(NAN, ERROR,
				"ucKeyType error!\n");
			return WLAN_STATUS_FAILURE;
		}
	} else { /* Overwrite the old one for AP and STA WEP */
		if (prBssInfo->prStaRecOfAP) {
			prCmdKey->ucWlanIndex =
				secPrivacySeekForBcEntry(
					prAdapter,
					prBssInfo->ucBssIndex,
					prBssInfo->prStaRecOfAP
						->aucMacAddr,
					prBssInfo->prStaRecOfAP
						->ucIndex,
					prCmdKey->ucAlgorithmId,
					prCmdKey->ucKeyId);

			kalMemCopy(prCmdKey->aucPeerAddr,
				   prBssInfo->prStaRecOfAP
					   ->aucMacAddr,
				   MAC_ADDR_LEN);
		} else {
			prCmdKey->ucWlanIndex =
				secPrivacySeekForBcEntry(
					prAdapter,
					prBssInfo->ucBssIndex,
					prBssInfo
						->aucOwnMacAddr,
					STA_REC_INDEX_NOT_FOUND,
					prCmdKey->ucAlgorithmId,
					prCmdKey->ucKeyId);
			kalMemCopy(prCmdKey->aucPeerAddr,
				   prBssInfo->aucOwnMacAddr,
				   MAC_ADDR_LEN);
		}
		if (fgNoHandshakeSec) { /* WEP: STA and AP */
			prBssInfo->wepkeyWlanIdx =
				prCmdKey->ucWlanIndex;
			prBssInfo->wepkeyUsed
				[prCmdKey->ucKeyId] = TRUE;
		} else if (
			!prBssInfo
				 ->prStaRecOfAP) {
			/* AP WPA/RSN */
			prBssInfo->ucBMCWlanIndexS
				[prCmdKey->ucKeyId] =
				prCmdKey->ucWlanIndex;
			prBssInfo->ucBMCWlanIndexSUsed
				[prCmdKey->ucKeyId] = TRUE;
		} else {
			/* STA WPA/RSN, should not have tx
			 * but no sta record
			 */
			prBssInfo->ucBMCWlanIndexS
				[prCmdKey->ucKeyId] =
				prCmdKey->ucWlanIndex;
			prBssInfo->ucBMCWlanIndexSUsed
				[prCmdKey->ucKeyId] = TRUE;
			DBGLOG(RSN, INFO,
			       "BMCWlanIndex kid = %d, index = %d\n",
			       prCmdKey->ucKeyId,
			       prCmdKey->ucWlanIndex);
		}
		if (prCmdKey->ucTxKey) {
			prBssInfo->fgBcDefaultKeyExist = TRUE;
			prBssInfo->ucBcDefaultKeyIdx =
				prCmdKey->ucKeyId;
		}
	}

#if 1 /* DBG */
	DBGLOG(NAN, INFO, "Add key cmd to wlan index %d:",
	       prCmdKey->ucWlanIndex);
	DBGLOG(NAN, INFO, "(BSS = %d) " MACSTR "\n", prCmdKey->ucBssIdx,
	       MAC2STR(prCmdKey->aucPeerAddr));
	DBGLOG(NAN, INFO, "Tx = %d type = %d Auth = %d\n", prCmdKey->ucTxKey,
	       prCmdKey->ucKeyType, prCmdKey->ucIsAuthenticator);
	DBGLOG(NAN, INFO, "cipher = %d keyid = %d keylen = %d\n",
	       prCmdKey->ucAlgorithmId, prCmdKey->ucKeyId, prCmdKey->ucKeyLen);
	DBGLOG_MEM8(RSN, INFO, prCmdKey->aucKeyMaterial, prCmdKey->ucKeyLen);

	DBGLOG(NAN, INFO, "wepkeyUsed = %d\n",
	       prBssInfo->wepkeyUsed[prCmdKey->ucKeyId]);
	DBGLOG(NAN, INFO, "wepkeyWlanIdx = %d:", prBssInfo->wepkeyWlanIdx);
	DBGLOG(NAN, INFO, "ucBMCWlanIndexSUsed = %d\n",
	       prBssInfo->ucBMCWlanIndexSUsed[prCmdKey->ucKeyId]);
	DBGLOG(NAN, INFO, "ucBMCWlanIndexS = %d:",
	       prBssInfo->ucBMCWlanIndexS[prCmdKey->ucKeyId]);
#endif

	/* insert into prCmdQueue */
	kalEnqueueCommand(prAdapter->prGlueInfo, (struct QUE_ENTRY *)prCmdInfo);

	/* wakeup txServiceThread later */
	GLUE_SET_EVENT(prAdapter->prGlueInfo);
	return WLAN_STATUS_PENDING;
}

uint32_t
nan_sec_wlanSetRemoveKey(IN struct ADAPTER *prAdapter, IN void *pvSetBuffer,
			 IN uint32_t u4SetBufferLen) {
	struct GLUE_INFO *prGlueInfo;
	struct CMD_INFO *prCmdInfo;
	/* P_PARAM_REMOVE_KEY_T prRemovedKey; */
	struct CMD_802_11_KEY *prCmdKey;
	struct CMD_802_11_KEY *prCmdFWKey;
	uint8_t ucCmdSeqNum;
	struct WLAN_TABLE *prWlanTable;
	struct STA_RECORD *prStaRec = NULL;
	struct BSS_INFO *prBssInfo;
	/* UINT_8 i = 0; */
	unsigned char fgRemoveWepKey = FALSE;
	uint32_t ucRemoveBCKeyAtIdx = WTBL_RESERVED_ENTRY;
	uint32_t u4KeyIndex;
	struct mt66xx_chip_info *prChipInfo;
	uint16_t cmd_size;

	prCmdFWKey = (struct CMD_802_11_KEY *)pvSetBuffer;
	DEBUGFUNC("wlanoidSetRemoveKey");
	DBGLOG(RSN, INFO, "wlanSetRemoveKeybyFW\n");
	if (!prAdapter) {
		DBGLOG(NAN, ERROR, "prAdapter error!\n");
		return WLAN_STATUS_FAILURE;
	}

	if (u4SetBufferLen < sizeof(struct PARAM_REMOVE_KEY))
		return WLAN_STATUS_INVALID_LENGTH;

	if (prAdapter->rAcpiState == ACPI_STATE_D3) {
		DBGLOG(REQ, WARN,
		       "Fail in set remove key! (Adapter not ready). ACPI=D%d, Radio=%d\n",
		       prAdapter->rAcpiState, prAdapter->fgIsRadioOff);
		return WLAN_STATUS_ADAPTER_NOT_READY;
	}

	prGlueInfo = prAdapter->prGlueInfo;
	prBssInfo = GET_BSS_INFO_BY_INDEX(prAdapter, prCmdFWKey->ucBssIdx);
	if (!prBssInfo) {
		DBGLOG(NAN, ERROR, "prBssInfo error!\n");
		return WLAN_STATUS_FAILURE;
	}
	u4KeyIndex = prCmdFWKey->ucKeyId;
#if CFG_SUPPORT_802_11W
	if (u4KeyIndex >= MAX_KEY_NUM + 2) {
		DBGLOG(NAN, ERROR, "u4KeyIndex is over!\n");
		return WLAN_STATUS_FAILURE;
	}
#else
/* ASSERT(prCmdKey->ucKeyId < MAX_KEY_NUM); */
#endif

	if (u4KeyIndex >= 4) {
		DBGLOG(RSN, INFO, "Remove bip key Index : 0x%08lx\n",
		       u4KeyIndex);
		return WLAN_STATUS_SUCCESS;
	}

	/* Clean up the Tx key flag */
	if (prCmdFWKey->ucKeyType) {
		prStaRec =
			cnmGetStaRecByAddress(prAdapter, prCmdFWKey->ucBssIdx,
					      prCmdFWKey->aucPeerAddr);
		if (!prStaRec)
			return WLAN_STATUS_SUCCESS;
	} else {
		if (u4KeyIndex == prBssInfo->ucBcDefaultKeyIdx)
			prBssInfo->fgBcDefaultKeyExist = FALSE;
	}

	if (!prStaRec) {
		if (prBssInfo->wepkeyUsed[u4KeyIndex] == TRUE)
			fgRemoveWepKey = TRUE;

		if (fgRemoveWepKey) {
			DBGLOG(RSN, INFO, "Remove wep key id = %d", u4KeyIndex);
			prBssInfo->wepkeyUsed[u4KeyIndex] = FALSE;
			if (prBssInfo->fgBcDefaultKeyExist &&
			    prBssInfo->ucBcDefaultKeyIdx == u4KeyIndex) {
				prBssInfo->fgBcDefaultKeyExist = FALSE;
				prBssInfo->ucBcDefaultKeyIdx = 0xff;
			}
			if (prBssInfo->wepkeyWlanIdx >= WTBL_SIZE) {
				DBGLOG(NAN, ERROR, "wepkeyWlanIdx is over!\n");
				return WLAN_STATUS_FAILURE;
			}
			ucRemoveBCKeyAtIdx = prBssInfo->wepkeyWlanIdx;
		} else {
			DBGLOG(RSN, INFO, "Remove group key id = %d",
			       u4KeyIndex);

			if (prBssInfo->ucBMCWlanIndexSUsed[u4KeyIndex]) {

				if (prBssInfo->fgBcDefaultKeyExist &&
				    prBssInfo->ucBcDefaultKeyIdx ==
					    u4KeyIndex) {
					prBssInfo->fgBcDefaultKeyExist = FALSE;
					prBssInfo->ucBcDefaultKeyIdx = 0xff;
				}
				if (u4KeyIndex != 0) {
					if (prBssInfo->
						ucBMCWlanIndexS[u4KeyIndex] >=
						WTBL_SIZE) {
						DBGLOG(NAN, ERROR,
							"ucBMCWlanIndexS is over\n");
						return WLAN_STATUS_FAILURE;
					}
				}
				ucRemoveBCKeyAtIdx =
					prBssInfo->ucBMCWlanIndexS[u4KeyIndex];
			}
		}

		DBGLOG(RSN, INFO, "ucRemoveBCKeyAtIdx = %d",
		       ucRemoveBCKeyAtIdx);

		if (ucRemoveBCKeyAtIdx >= WTBL_SIZE)
			return WLAN_STATUS_SUCCESS;
	}

	prChipInfo = prAdapter->chip_info;

	if (!prChipInfo) {
		DBGLOG(NAN, ERROR, "prChipInfo error!\n");
		return WLAN_STATUS_FAILURE;
	}
	cmd_size = prChipInfo->u2CmdTxHdrSize + sizeof(struct CMD_802_11_KEY);

	prCmdInfo = cmdBufAllocateCmdInfo(prAdapter, cmd_size);

	if (!prCmdInfo) {
		DBGLOG(INIT, ERROR, "Allocate CMD_INFO_T ==> FAILED.\n");
		return WLAN_STATUS_FAILURE;
	}

	prWlanTable = prAdapter->rWifiVar.arWtbl;
	prBssInfo = GET_BSS_INFO_BY_INDEX(prAdapter, prCmdFWKey->ucBssIdx);

	/* increase command sequence number */
	ucCmdSeqNum = nicIncreaseCmdSeqNum(prAdapter);

	/* compose CMD_802_11_KEY cmd pkt */
	prCmdInfo->eCmdType = COMMAND_TYPE_NETWORK_IOCTL;
	/* prCmdInfo->ucBssIndex = prRemovedKey->ucBssIdx; */
	prCmdInfo->u2InfoBufLen = cmd_size;
	prCmdInfo->pfCmdDoneHandler = NULL;
	prCmdInfo->pfCmdTimeoutHandler = NULL;
	prCmdInfo->fgIsOid = FALSE;
	prCmdInfo->ucCID = CMD_ID_ADD_REMOVE_KEY;
	prCmdInfo->fgSetQuery = TRUE;
	prCmdInfo->fgNeedResp = FALSE;
	/* prCmdInfo->fgDriverDomainMCR = FALSE; */
	prCmdInfo->ucCmdSeqNum = ucCmdSeqNum;
	prCmdInfo->u4SetInfoLen = sizeof(struct PARAM_REMOVE_KEY);
	prCmdInfo->pvInformationBuffer = pvSetBuffer;
	prCmdInfo->u4InformationBufferLength = u4SetBufferLen;
	/* Setup WIFI_CMD_T */

	NIC_FILL_CMD_TX_HDR(prAdapter, prCmdInfo->pucInfoBuffer,
			    prCmdInfo->u2InfoBufLen, prCmdInfo->ucCID,
			    CMD_PACKET_TYPE_ID, &prCmdInfo->ucCmdSeqNum,
			    prCmdInfo->fgSetQuery, &prCmdKey, FALSE, 0,
			    S2D_INDEX_CMD_H2N);

	kalMemZero((uint8_t *)prCmdKey, sizeof(struct CMD_802_11_KEY));

	prCmdKey->ucAddRemove = 0; /* Remove */
	prCmdKey->ucKeyId = (uint8_t)u4KeyIndex;
	kalMemCopy(prCmdKey->aucPeerAddr, (uint8_t *)prCmdFWKey->aucPeerAddr,
		   MAC_ADDR_LEN);
	prCmdKey->ucBssIdx = prCmdFWKey->ucBssIdx;

	if (prStaRec) {
		prCmdKey->ucKeyType = 1;
		prCmdKey->ucWlanIndex = prStaRec->ucWlanIndex;
		prStaRec->fgTransmitKeyExist = FALSE;
	} else if (ucRemoveBCKeyAtIdx < WTBL_SIZE) {
		prCmdKey->ucWlanIndex = ucRemoveBCKeyAtIdx;
	} else {
		DBGLOG(NAN, ERROR,
			"prStaRec is null or ucRemoveBCKeyAtIdx >= WTBL_SIZE!\n");
		return WLAN_STATUS_FAILURE;
	}

	/* insert into prCmdQueue */
	kalEnqueueCommand(prGlueInfo, (struct QUE_ENTRY *)prCmdInfo);

	/* wakeup txServiceThread later */
	GLUE_SET_EVENT(prGlueInfo);

	return WLAN_STATUS_PENDING;
}

int
nan_sec_wpas_setkey_glue(bool fgIsAp, u8 u1BssIdx, enum wpa_alg alg,
			 const u8 *addr, int key_idx, const u8 *key,
			 size_t key_len) {
	struct CMD_802_11_KEY rCmdkey;
	struct CMD_802_11_KEY *prCmdkey = &rCmdkey;
	struct STA_RECORD *prStaRec = NULL;
	uint8_t aucBCAddr[] = BC_MAC_ADDR;
	struct BSS_INFO *prBssInfo = NULL;
	int status = 0;
	/* UINT_8 ucEntry = WTBL_RESERVED_ENTRY; */
	unsigned char fgIsBC = FALSE;

	/* TODO_CJ: every NAN should be STA and currently no GTK */

	DBGLOG(NAN, INFO,
	       "[%s]Enter, fgIsAp:%d, u1BssIdx:%d, alg:%d, key_idx:%d, key_len:%d\n",
	       __func__, fgIsAp, u1BssIdx, alg, key_idx,
	       key_len); /* dump outside */

	/* _wpa_hexdump_ram(MSG_INFO, "addr", addr, 6, 1, 0); */

	/* _wpa_hexdump_ram(MSG_INFO, "key", key, key_len, 1, 0); */

	if (kalMemCmp(addr, aucBCAddr, ETH_ALEN) == 0) {
		DBGLOG(NAN, INFO, "[%s] broadcast addr", __func__);
		fgIsBC = TRUE;
	}

	prBssInfo = GET_BSS_INFO_BY_INDEX(g_prAdapter, u1BssIdx);
	prStaRec = cnmGetStaRecByAddress(g_prAdapter, prBssInfo->ucBssIndex,
					 (uint8_t *)addr);

	/* Compose the common add key structure */
	kalMemZero(&rCmdkey, sizeof(struct CMD_802_11_KEY));

	rCmdkey.ucAddRemove = key_len ? 1 : 0;
	rCmdkey.ucTxKey = fgIsBC ? 0 : 1;
	rCmdkey.ucKeyType = fgIsBC ? 0 : 1;

	if (fgIsAp) { /* AP */
		rCmdkey.ucIsAuthenticator = TRUE;

		if (fgIsBC) {
			kalMemCopy(&rCmdkey.aucPeerAddr,
				   prBssInfo->aucOwnMacAddr,
				   MAC_ADDR_LEN); /* Own AP */
		} else {
			kalMemCopy(&rCmdkey.aucPeerAddr, addr,
				   MAC_ADDR_LEN); /* Remote STA */
		}
	} else { /* STA */
		rCmdkey.ucIsAuthenticator = FALSE;

		if (fgIsBC) {
			kalMemCopy(&rCmdkey.aucPeerAddr, prBssInfo->aucBSSID,
				   MAC_ADDR_LEN); /* Remote AP */
		} else {
			kalMemCopy(&rCmdkey.aucPeerAddr, addr,
				   MAC_ADDR_LEN); /* Remote AP */
		}
	}

	rCmdkey.ucBssIdx = u1BssIdx;

	if (alg == WPA_ALG_CCMP)
		rCmdkey.ucAlgorithmId = CIPHER_SUITE_CCMP;
	/* else if (alg == WPA_ALG_GCMP_256) */
	/* rCmdkey.ucAlgorithmId = CIPHER_SUITE_GCMP_256; */
	else if (alg == WPA_ALG_TKIP)
		rCmdkey.ucAlgorithmId = CIPHER_SUITE_TKIP;
	else if (alg == WPA_ALG_IGTK) {
		rCmdkey.ucAlgorithmId = CIPHER_SUITE_BIP;
		kalMemSet(&rCmdkey.aucPeerAddr, 0, MAC_ADDR_LEN);
	} else {
		DBGLOG(NAN, ERROR,
		       "Not support the alg=%d, reset to WPA_ALG_CCMP\n", alg);
		alg = WPA_ALG_CCMP;
		rCmdkey.ucAlgorithmId = CIPHER_SUITE_CCMP;
	}
	rCmdkey.ucKeyId = key_idx;
	rCmdkey.ucKeyLen = key_len;

	if (!rCmdkey.ucKeyType) {
		if (prBssInfo->ucBMCWlanIndex >= MAX_WTBL_ENTRY_NUM) {
			DBGLOG(NAN, INFO,
			       "[%s] WARN! Unknown ucBMCWlanIndex:%d, u1BssIdx:%d",
			       __func__, prBssInfo->ucBMCWlanIndex, u1BssIdx);
			rCmdkey.ucWlanIndex = (MAX_WTBL_ENTRY_NUM - 1);
			/* ASSERT(FALSE); */
		} else {
			rCmdkey.ucWlanIndex = prBssInfo->ucBMCWlanIndex;
		}
	} else {
		if (prStaRec != NULL)
			rCmdkey.ucWlanIndex = prStaRec->ucWlanIndex;
	}

	if (key != NULL) {
		if ((key_len == 32) &&
		    (rCmdkey.ucAlgorithmId == CIPHER_SUITE_TKIP) &&
		    (rCmdkey.ucIsAuthenticator == FALSE)) {
			/* Do this like driver do : mtk_cfg80211_add_key */
			kalMemCopy(&rCmdkey.aucKeyMaterial, key, 16);
			kalMemCopy(&rCmdkey.aucKeyMaterial[24], key + 16, 8);
			kalMemCopy(&rCmdkey.aucKeyMaterial[16], key + 24, 8);
		} else {
			kalMemCopy(&rCmdkey.aucKeyMaterial, key, key_len);
		}
	}

	/* End of Compose the common add key structure */

	/* Add Key */
	/* dumpCmdKey(prCmdkey); */
	if (prCmdkey->ucAddRemove) {
		if (prCmdkey->ucWlanIndex >= MAX_WTBL_ENTRY_NUM) {
			/* DBGLOG(RSN, ERROR, ("Wrong wlan index\n")); */
			DBGLOG(NAN, ERROR, "ucWlanIndex is over!\n");
			status = -1;
		} else {
			/* phase1: driver cmd trigger it */
#if 0
			nicPrivacySetKeyEntry(
				prCmdkey, prCmdkey->ucWlanIndex,
				prStaRec);
			dumpCmdKey(prCmdkey);
				_wpas_evt_cfg80211_add_key(prCmdkey);
#endif
			nan_sec_wlanSetAddKey(g_prAdapter, prCmdkey,
					      sizeof(struct CMD_802_11_KEY));
		}
	} else { /* Remove Key */
#if 0
		DBGLOG(RSN, INFO, ("[%s] Remove key\n", __func__));
		dumpCmdKey(prCmdkey);
		_wpas_evt_cfg80211_add_key(prCmdkey);
#endif
		nan_sec_wlanSetRemoveKey(g_prAdapter, prCmdkey,
					 sizeof(struct CMD_802_11_KEY));
	}
	return status;
}

int
nan_sec_wpa_supplicant_set_key(void *_wpa_s, enum wpa_alg alg, const u8 *addr,
			       int key_idx, int set_tx, const u8 *seq,
			       size_t seq_len, const u8 *key, size_t key_len) {
	struct wpa_supplicant *prWpa_s = (struct wpa_supplicant *)_wpa_s;

	DBGLOG(NAN, INFO, "[%s] Enter\n", __func__);

	nan_sec_wpas_setkey_glue(FALSE, prWpa_s->u1BssIdx, alg, addr, key_idx,
				 key, key_len);
	return 0;
}

int
nan_sec_hostapd_wpa_auth_set_key(void *ctx, int vlan_id, enum wpa_alg alg,
				 const u8 *addr, int idx, u8 *key,
				 size_t key_len) {
	/* struct hostapd_data *hapd = (struct hostapd_data *) ctx; */

	DBGLOG(NAN, INFO, "[%s] Enter\n", __func__);
#if 0 /* set key after NDP alloc sta_rec */
	nan_sec_wpas_setkey_glue(TRUE, hapd->u1BssIdx, alg,
							 addr, idx,
						 key, key_len);
#endif
	return 0;
}

/************************************************
 *               Tx Related
 ************************************************
 */
int
nan_sec_wpa_eapol_key_mic(const u8 *key, size_t key_len, u32 cipher,
			  const u8 *buf, size_t len, u8 *mic) {
	u8 hash[NAN_SHA384_MAC_LEN];

	DBGLOG(NAN, INFO, "[%s] Enter\n", __func__);

	DBGLOG(NAN, INFO, "[%s] KCK len:%d\n", __func__, key_len);
	dumpMemory8((uint8_t *)key, key_len);

	DBGLOG(NAN, INFO, "[%s] BUF_len:%d\n", __func__, len);
	dumpMemory8((uint8_t *)buf, len);

	if (cipher == NAN_CIPHER_SUITE_ID_NCS_SK_GCM_256) {
		if (hmac_sha384(key, key_len, buf, len, hash)) {
			DBGLOG(NAN, INFO, "[%s] ERROR! hmac_sha384() failed",
			       __func__);
			return WLAN_STATUS_FAILURE;
		}
		os_memcpy(mic, hash, NCS_SK_256_MIC_LEN);

		DBGLOG(NAN, INFO, "[%s] Result MIC:\n", __func__);
		dumpMemory8(mic, NCS_SK_256_MIC_LEN);
	} else {
		/* NAN_CIPHER_SUITE_ID_NCS_SK_CCM_128 */
		if (hmac_sha256(key, key_len, buf, len, hash)) {
			DBGLOG(NAN, INFO, "[%s] ERROR! hmac_sha256() failed",
			       __func__);
			return WLAN_STATUS_FAILURE;
		}
		os_memcpy(mic, hash, NCS_SK_128_MIC_LEN);

		DBGLOG(NAN, INFO, "[%s] Result MIC:\n", __func__);
		dumpMemory8(mic, NCS_SK_128_MIC_LEN);
	}

	return WLAN_STATUS_SUCCESS;
}

int
nan_sec_wpa_supplicant_send_2_of_4(struct wpa_sm *sm, const unsigned char *dst,
				   const struct wpa_eapol_key *key, int ver,
				   const u8 *nonce, const u8 *wpa_ie,
				   size_t wpa_ie_len, struct wpa_ptk *ptk) {
	size_t mic_len, hdrlen; /*rlen*/
	struct wpa_eapol_key *reply;
	struct wpa_eapol_key_192 *reply192;
	u8 *key_mic; /* *rbuf */
	/* u8 *rsn_ie_buf = NULL; */

	struct _NAN_SEC_KDE_ATTR_HDR *prNanSecKdeAttrHdr = NULL;
	uint32_t u4TotalLen = 0;

	DBGLOG(NAN, INFO, "[%s] Enter\n", __func__);

#if 0
	if (wpa_ie == NULL) {
		wpa_msg(sm->ctx->msg_ctx, MSG_WARNING,
				"WPA: No wpa_ie set-cannot generate msg 2/4");
		return -1;
	}

	wpa_hexdump(MSG_DEBUG, "WPA: WPA IE for msg 2/4", wpa_ie, wpa_ie_len);
#endif

	mic_len = wpa_mic_len(sm->key_mgmt);
	hdrlen = mic_len == 24 ? sizeof(*reply192) : sizeof(*reply);
#if 0
	rbuf = wpa_sm_alloc_eapol(sm, IEEE802_1X_TYPE_EAPOL_KEY,
			NULL, hdrlen + wpa_ie_len,
			&rlen, (void *) &reply);

	if (rbuf == NULL) {
		os_free(rsn_ie_buf);
		return -1;
	}
#endif

	u4TotalLen = sizeof(struct _NAN_SEC_KDE_ATTR_HDR) + hdrlen;

	sm->pu1TmpKdeAttrBuf = os_zalloc(u4TotalLen);
	sm->u4TmpKdeAttrLen = u4TotalLen;

	if (sm->pu1TmpKdeAttrBuf == NULL)
		return -1;

	prNanSecKdeAttrHdr =
		(struct _NAN_SEC_KDE_ATTR_HDR *)sm->pu1TmpKdeAttrBuf;
	prNanSecKdeAttrHdr->u1AttrId = NAN_ATTR_ID_SHARED_KEY_DESCRIPTOR;
	prNanSecKdeAttrHdr->u2AttrLen = hdrlen + 1; /*1:publishId*/
	prNanSecKdeAttrHdr->u1PublishId =
		((struct _NAN_NDP_INSTANCE_T *)(sm->pvNdp))->ucPublishId;

	reply = (struct wpa_eapol_key *)(sm->pu1TmpKdeAttrBuf +
					 sizeof(struct _NAN_SEC_KDE_ATTR_HDR));

	reply192 = (struct wpa_eapol_key_192 *)reply;

	reply->type =
		(sm->proto == WPA_PROTO_RSN || sm->proto == WPA_PROTO_OSEN)
			? EAPOL_KEY_TYPE_RSN
			: EAPOL_KEY_TYPE_WPA;
	WPA_PUT_BE16(reply->key_info,
				 WPA_KEY_INFO_KEY_TYPE | WPA_KEY_INFO_MIC);
#if 0
	if (sm->proto == WPA_PROTO_RSN || sm->proto == WPA_PROTO_OSEN)
		WPA_PUT_BE16(reply->key_length, 0);
	else
		os_memcpy(reply->key_length, key->key_length, 2);
#endif

	WPA_PUT_BE16(reply->key_length, 0);

	os_memcpy(reply->replay_counter, key->replay_counter,
		  WPA_REPLAY_COUNTER_LEN);
	wpa_hexdump(MSG_DEBUG, "WPA: Replay Counter", reply->replay_counter,
		    WPA_REPLAY_COUNTER_LEN);

	key_mic = reply192->key_mic; /* same offset for reply and reply192 */

#if 0
	if (mic_len == 24) {
		WPA_PUT_BE16(reply192->key_data_length, wpa_ie_len);
		os_memcpy(reply192 + 1, wpa_ie, wpa_ie_len);
	} else {
		WPA_PUT_BE16(reply->key_data_length, wpa_ie_len);
		os_memcpy(reply + 1, wpa_ie, wpa_ie_len);
	}
	os_free(rsn_ie_buf);
#endif

	os_memcpy(reply->key_nonce, nonce, WPA_NONCE_LEN);

	wpa_dbg(sm->ctx->msg_ctx, MSG_DEBUG, "WPA: Sending EAPOL-Key 2/4\n");

	nanSecDumpEapolKey(reply);
#if 0
	wpa_eapol_key_send_wpa(sm, ptk->kck,
		   ptk->kck_len, ver, dst, ETH_P_EAPOL,
		   rbuf, rlen, key_mic);
#endif

	sm->u1CurMsg = NAN_SEC_M2;
	nanSecMicCalStaSmStep(sm);

	return 0;
}

int
nan_sec_wpa_supplicant_send_4_of_4(struct wpa_sm *sm, const unsigned char *dst,
				   const struct wpa_eapol_key *key, u16 ver,
				   u16 key_info, struct wpa_ptk *ptk) {
	size_t mic_len, hdrlen; /*rlen*/
	struct wpa_eapol_key *reply;
	struct wpa_eapol_key_192 *reply192;
	u8 *key_mic; /* *rbuf, */

	struct _NAN_SEC_KDE_ATTR_HDR *prNanSecKdeAttrHdr = NULL;
	uint32_t u4TotalLen = 0;

	DBGLOG(NAN, INFO, "[%s] Enter\n", __func__);

	mic_len = wpa_mic_len(sm->key_mgmt);
	hdrlen = mic_len == 24 ? sizeof(*reply192) : sizeof(*reply);
#if 0
	rbuf = wpa_sm_alloc_eapol(sm, IEEE802_1X_TYPE_EAPOL_KEY, NULL,
			  hdrlen, &rlen, (void *) &reply);

	if (rbuf == NULL)
		return -1;
#endif
	u4TotalLen = sizeof(struct _NAN_SEC_KDE_ATTR_HDR) + hdrlen;

	sm->pu1TmpKdeAttrBuf = os_zalloc(u4TotalLen);
	sm->u4TmpKdeAttrLen = u4TotalLen;

	if (sm->pu1TmpKdeAttrBuf == NULL)
		return -1;

	prNanSecKdeAttrHdr =
		(struct _NAN_SEC_KDE_ATTR_HDR *)sm->pu1TmpKdeAttrBuf;
	prNanSecKdeAttrHdr->u1AttrId = NAN_ATTR_ID_SHARED_KEY_DESCRIPTOR;
	prNanSecKdeAttrHdr->u2AttrLen = hdrlen + 1; /* 1:publishId */
	prNanSecKdeAttrHdr->u1PublishId =
		((struct _NAN_NDP_INSTANCE_T *)(sm->pvNdp))->ucPublishId;

	reply = (struct wpa_eapol_key *)(sm->pu1TmpKdeAttrBuf +
					 sizeof(struct _NAN_SEC_KDE_ATTR_HDR));

	reply192 = (struct wpa_eapol_key_192 *)reply;

	reply->type =
		(sm->proto == WPA_PROTO_RSN || sm->proto == WPA_PROTO_OSEN)
			? EAPOL_KEY_TYPE_RSN
			: EAPOL_KEY_TYPE_WPA;
	key_info &= WPA_KEY_INFO_SECURE;
	key_info |= WPA_KEY_INFO_KEY_TYPE | WPA_KEY_INFO_MIC;
	key_info |= WPA_KEY_INFO_INSTALL;
	WPA_PUT_BE16(reply->key_info, key_info);

#if 0
	if (sm->proto == WPA_PROTO_RSN || sm->proto == WPA_PROTO_OSEN)
		WPA_PUT_BE16(reply->key_length, 0);
	else
		os_memcpy(reply->key_length, key->key_length, 2);
#endif
	WPA_PUT_BE16(reply->key_length, 0);

	os_memcpy(reply->replay_counter, key->replay_counter,
		  WPA_REPLAY_COUNTER_LEN);

	key_mic = reply192->key_mic; /* same offset for reply and reply192 */
	if (mic_len == 24)
		WPA_PUT_BE16(reply192->key_data_length, 0);
	else
		WPA_PUT_BE16(reply->key_data_length, 0);

	wpa_dbg(sm->ctx->msg_ctx, MSG_DEBUG, "WPA: Sending EAPOL-Key 4/4");
#if 0
	wpa_eapol_key_send_wpa(sm,
		   ptk->kck, ptk->kck_len, ver, dst, ETH_P_EAPOL,
		   rbuf, rlen, key_mic);
#endif

	sm->u1CurMsg = NAN_SEC_M4;
	nanSecMicCalStaSmStep(sm);

	return 0;
}

int
nan_sec_wpa_send_eapol(
	struct wpa_authenticator *wpa_auth, /* AP: KDE compose, MIC, and send */
	struct wpa_state_machine *sm, int key_info, const u8 *key_rsc,
	const u8 *nonce, const u8 *kde, size_t kde_len, int keyidx, int encr,
	int force_version) {
	/* struct ieee802_1x_hdr *hdr; */
	struct wpa_eapol_key *key;
	struct wpa_eapol_key_192 *key192;
	size_t mic_len, keyhdrlen; /* len */
	int alg;
	int key_data_len, pad_len = 0;
	u8 *buf, *pos;
	int version, pairwise;
	int i;
	u8 *key_data;

	struct _NAN_SEC_KDE_ATTR_HDR *prNanSecKdeAttrHdr = NULL;
	uint32_t u4TotalLen = 0;

	DBGLOG(NAN, INFO, "[%s] Enter\n", __func__);

	mic_len = wpa_mic_len(sm->wpa_key_mgmt);
	keyhdrlen = mic_len == 24 ? sizeof(*key192) : sizeof(*key);

/* len = sizeof(struct ieee802_1x_hdr) + keyhdrlen; */
#if 0
	if (force_version)
		version = force_version;
	else if (sm->wpa_key_mgmt == WPA_KEY_MGMT_OSEN ||
			 wpa_key_mgmt_suite_b(sm->wpa_key_mgmt))
		version = WPA_KEY_INFO_TYPE_AKM_DEFINED;
	else if (wpa_use_aes_cmac(sm))
		version = WPA_KEY_INFO_TYPE_AES_128_CMAC;
	else if (sm->pairwise != WPA_CIPHER_TKIP)
		version = WPA_KEY_INFO_TYPE_HMAC_SHA1_AES;
	else
		version = WPA_KEY_INFO_TYPE_HMAC_MD5_RC4;
#endif

	version = WPA_KEY_INFO_TYPE_AKM_DEFINED;

	pairwise = !!(key_info & WPA_KEY_INFO_KEY_TYPE);

	key_data_len = kde_len;

	if ((version == WPA_KEY_INFO_TYPE_HMAC_SHA1_AES ||
	     sm->wpa_key_mgmt == WPA_KEY_MGMT_OSEN ||
	     wpa_key_mgmt_suite_b(sm->wpa_key_mgmt) ||
	     version == WPA_KEY_INFO_TYPE_AES_128_CMAC) &&
	    encr) {
		pad_len = key_data_len % 8;
		if (pad_len)
			pad_len = 8 - pad_len;
		key_data_len += pad_len + 8;
	}
#if 0
	len += key_data_len;

	hdr = os_zalloc(len);
	if (hdr == NULL)
		return;
#endif

	u4TotalLen =
		sizeof(struct _NAN_SEC_KDE_ATTR_HDR) + keyhdrlen + key_data_len;

	sm->pu1TmpKdeAttrBuf = os_zalloc(u4TotalLen);
	sm->u4TmpKdeAttrLen = u4TotalLen;

	if (sm->pu1TmpKdeAttrBuf == NULL)
		return WLAN_STATUS_FAILURE;

	prNanSecKdeAttrHdr =
		(struct _NAN_SEC_KDE_ATTR_HDR *)sm->pu1TmpKdeAttrBuf;
	prNanSecKdeAttrHdr->u1AttrId = NAN_ATTR_ID_SHARED_KEY_DESCRIPTOR;
	prNanSecKdeAttrHdr->u2AttrLen =
		keyhdrlen + key_data_len + 1; /*1:publishId*/
	prNanSecKdeAttrHdr->u1PublishId =
		((struct _NAN_NDP_INSTANCE_T *)(sm->pvNdp))->ucPublishId;

	DBGLOG(NAN, INFO, "[%s] u4TotalLen:%d\n", __func__, u4TotalLen);

	key = (struct wpa_eapol_key *)(sm->pu1TmpKdeAttrBuf +
				       sizeof(struct _NAN_SEC_KDE_ATTR_HDR));

	key192 = (struct wpa_eapol_key_192 *)key;

	key_data = ((u8 *)key) + keyhdrlen;
#if 0
	hdr->version = wpa_auth->conf.eapol_version;
	hdr->type = IEEE802_1X_TYPE_EAPOL_KEY;
	hdr->length = host_to_be16(len  - sizeof(*hdr));
	key = (struct wpa_eapol_key *) (hdr + 1);
	key192 = (struct wpa_eapol_key_192 *) (hdr + 1);
	key_data = ((u8 *) (hdr + 1)) + keyhdrlen;
#endif

	key->type = sm->wpa == WPA_VERSION_WPA2 ? EAPOL_KEY_TYPE_RSN
						: EAPOL_KEY_TYPE_WPA;
	key_info |= version;
	if (encr && sm->wpa == WPA_VERSION_WPA2)
		key_info |= WPA_KEY_INFO_ENCR_KEY_DATA;
	if (sm->wpa != WPA_VERSION_WPA2)
		key_info |= keyidx << WPA_KEY_INFO_KEY_INDEX_SHIFT;
	WPA_PUT_BE16(key->key_info, key_info);

	alg = pairwise ? sm->pairwise : wpa_auth->conf.wpa_group;
	/* WPA_PUT_BE16(key->key_length, wpa_cipher_key_len(alg)); */
	WPA_PUT_BE16(key->key_length, 0);

	if (key_info & WPA_KEY_INFO_SMK_MESSAGE)
		WPA_PUT_BE16(key->key_length, 0);

	/* FIX: STSL: what to use as key_replay_counter? */
	for (i = RSNA_MAX_EAPOL_RETRIES - 1; i > 0; i--) {
		sm->key_replay[i].valid = sm->key_replay[i - 1].valid;
		os_memcpy(sm->key_replay[i].counter,
			  sm->key_replay[i - 1].counter,
			  WPA_REPLAY_COUNTER_LEN);
	}
	inc_byte_array(sm->key_replay[0].counter, WPA_REPLAY_COUNTER_LEN);
	os_memcpy(key->replay_counter, sm->key_replay[0].counter,
		  WPA_REPLAY_COUNTER_LEN);
	wpa_hexdump(MSG_DEBUG, "WPA: Replay Counter", key->replay_counter,
		    WPA_REPLAY_COUNTER_LEN);
	sm->key_replay[0].valid = TRUE;

	if (nonce)
		os_memcpy(key->key_nonce, nonce, WPA_NONCE_LEN);

	if (key_rsc)
		os_memcpy(key->key_rsc, key_rsc, WPA_KEY_RSC_LEN);

	if (kde && !encr) {
		os_memcpy(key_data, kde, kde_len);
		if (mic_len == 24)
			WPA_PUT_BE16(key192->key_data_length, kde_len);
		else
			WPA_PUT_BE16(key->key_data_length, kde_len);
	} else if (encr && kde) {
		buf = os_zalloc(key_data_len);
		if (buf == NULL) {
			/* os_free(hdr); */
			return WLAN_STATUS_FAILURE;
		}
		pos = buf;
		os_memcpy(pos, kde, kde_len);
		pos += kde_len;

		if (pad_len)
			*pos++ = 0xdd;

		wpa_hexdump_key(MSG_DEBUG, "Plaintext EAPOL-Key Key Data", buf,
				key_data_len);
		if (version == WPA_KEY_INFO_TYPE_HMAC_SHA1_AES ||
		    sm->wpa_key_mgmt == WPA_KEY_MGMT_OSEN ||
		    wpa_key_mgmt_suite_b(sm->wpa_key_mgmt) ||
		    version == WPA_KEY_INFO_TYPE_AES_128_CMAC) {
			if (aes_wrap(sm->PTK.kek, sm->PTK.kek_len,
				     (key_data_len - 8) / 8, buf, key_data)) {
				/* os_free(hdr); */
				os_free(buf);
				return WLAN_STATUS_FAILURE;
			}
			if (mic_len == 24)
				WPA_PUT_BE16(key192->key_data_length,
					     key_data_len);
			else
				WPA_PUT_BE16(key->key_data_length,
					     key_data_len);
#ifndef CONFIG_NO_RC4
		} else if (sm->PTK.kek_len == 16) {
			u8 ek[32];

			os_memcpy(key->key_iv,
				  sm->group->Counter + WPA_NONCE_LEN - 16, 16);
			inc_byte_array(sm->group->Counter, WPA_NONCE_LEN);
			os_memcpy(ek, key->key_iv, 16);
			os_memcpy(ek + 16, sm->PTK.kek, sm->PTK.kek_len);
			os_memcpy(key_data, buf, key_data_len);
			rc4_skip(ek, 32, 256, key_data, key_data_len);
			if (mic_len == 24)
				WPA_PUT_BE16(key192->key_data_length,
					     key_data_len);
			else
				WPA_PUT_BE16(key->key_data_length,
					     key_data_len);
#endif /* CONFIG_NO_RC4 */
		} else {
			/* os_free(hdr); */
			os_free(buf);
			return WLAN_STATUS_FAILURE;
		}
		os_free(buf);
	}

	if (key_info & WPA_KEY_INFO_MIC) {
		/* u8 *key_mic; */

		if (!sm->PTK_valid) {
			wpa_auth_logger(
				wpa_auth, sm->addr, LOGGER_DEBUG,
				"PTK not valid when sending EAPOL-Key frame");
			/* os_free(hdr); */
			return WLAN_STATUS_FAILURE;
		}

		/* same offset for key and key192 */
#if 0
		key_mic = key192->key_mic;

		wpa_eapol_key_mic_wpa(sm->PTK.kck, sm->PTK.kck_len,
			  sm->wpa_key_mgmt, version,
			  (u8 *) hdr, len, key_mic);
#endif
	}

	wpa_auth_set_eapol(sm->wpa_auth, sm->addr, WPA_EAPOL_inc_EapolFramesTx,
			   1);

#if (ENABLE_SEC_UT_LOG == 1)
	nanSecDumpEapolKey(key);
#endif
#if 0
	wpa_auth_send_eapol(wpa_auth, sm->addr, (u8 *) hdr, len,
		    sm->pairwise_set);
#endif
	nanSecMicCalApSmStep(sm);
#if 0
	os_free(hdr);
#endif

	return WLAN_STATUS_SUCCESS;
}

/************************************************
 *               Rx Related
 ************************************************
 */
int
nan_sec_wpa_verify_key_mic(int akmp, struct wpa_ptk *PTK, u8 *data,
			   size_t data_len, struct wpa_state_machine *sm) /*AP*/
{
	/* struct ieee802_1x_hdr *hdr; */
	struct wpa_eapol_key_192 *key192;
	u16 key_info;
	int ret = 0;
	u8 mic[WPA_EAPOL_KEY_MIC_MAX_LEN];
	size_t mic_len = wpa_mic_len(akmp);
	u32 cipher;
	/* UINT_8  u1RxMsg = 0; */

	struct _NAN_SEC_KDE_ATTR_HDR *hdr;

	DBGLOG(NAN, INFO, "[%s] Enter\n", __func__);

	if (data_len < sizeof(*hdr) + sizeof(struct wpa_eapol_key)) {
		DBGLOG(NAN, ERROR,
		       "[%s] ERROR! size mis-match, data_len:%d, hdr+key:%d",
		       __func__, data_len,
		       sizeof(*hdr) + sizeof(struct wpa_eapol_key));
		return -1;
	}

	/* hdr = (struct ieee802_1x_hdr *) data; */
	/* hdr = (struct _NAN_SEC_KDE_ATTR_HDR *) data; */
	hdr = (struct _NAN_SEC_KDE_ATTR_HDR *)sm->pu1GetRxMsgKdeBuf;
	key192 = (struct wpa_eapol_key_192 *)(hdr + 1);
	key_info = WPA_GET_BE16(key192->key_info);
	os_memcpy(mic, key192->key_mic, mic_len);
	os_memset(key192->key_mic, 0, mic_len);

	/* M2, M4 */
	if (mic_len == 24)
		cipher = NAN_CIPHER_SUITE_ID_NCS_SK_GCM_256;
	else
		cipher = NAN_CIPHER_SUITE_ID_NCS_SK_CCM_128;

	ret = nan_sec_wpa_eapol_key_mic(PTK->kck, PTK->kck_len, cipher,
					sm->pu1GetRxMsgBodyBuf,
					sm->u4GetRxMsgBodyLen, key192->key_mic);
	if (os_memcmp_const(mic, key192->key_mic, mic_len) != 0) {
		DBGLOG(NAN, WARN, "[%s] WARN! MIC mis-match, remote MIC:\n",
		       __func__);
		dumpMemory8(mic, mic_len);
		return WLAN_STATUS_FAILURE;
	}

	os_memcpy(key192->key_mic, mic, mic_len);
	return ret;
}

uint32_t
nan_sec_wpa_sm_rx_eapol(struct wpa_sm *sm, const u8 *src_addr) {
	size_t plen, data_len, key_data_len;
	/* const struct ieee802_1x_hdr *hdr; */
	struct _NAN_SEC_KDE_ATTR_HDR *prNanSecKdeHdr;
	struct wpa_eapol_key *key;
	struct wpa_eapol_key_192 *key192;
	u16 key_info, ver;
	u8 *tmp = NULL;
	int ret = WLAN_STATUS_FAILURE;
	struct wpa_peerkey *peerkey = NULL;
	u8 *key_data;
	size_t mic_len, keyhdrlen;

	u8 *buf = sm->pu1GetRxMsgKdeBuf;
	size_t len = sm->u4GetRxMsgKdeLen;

	mic_len = wpa_mic_len(sm->key_mgmt);
	keyhdrlen = mic_len == 24 ? sizeof(*key192) : sizeof(*key);

	DBGLOG(NAN, INFO, "[%s] Enter\n", __func__);

	if (len < sizeof(*prNanSecKdeHdr) + keyhdrlen) {
		DBGLOG(NAN, INFO,
		       "[%s] Quit1. len:%d, sizeof(*prNanSecKdeHdr):%d, keyhdrlen:%d\n",
		       __func__, len, sizeof(*prNanSecKdeHdr), keyhdrlen);

		return WLAN_STATUS_FAILURE;
	}

	/* hdr = (const struct ieee802_1x_hdr *) buf; */
	/* plen = be_to_host16(hdr->length); */
	/* data_len = plen + sizeof(*hdr); */

	prNanSecKdeHdr = (struct _NAN_SEC_KDE_ATTR_HDR *)buf;
	plen = prNanSecKdeHdr->u2AttrLen;
	data_len = plen + sizeof(*prNanSecKdeHdr);

#if 0
	DBGLOG(NAN, INFO, "[%s] IEEE 802.1X RX: version=%d type=%d length=%lu",
		   __func__, hdr->version, hdr->type, (unsigned long) plen);

	if (hdr->version < EAPOL_VERSION)
		/* TODO: backwards compatibility */

		if (hdr->type != IEEE802_1X_TYPE_EAPOL_KEY) {
			/*
			*wpa_dbg(sm->ctx->msg_ctx, MSG_DEBUG,
			*	"WPA: EAPOL frame (type %u) discarded,
			*	not a Key frame", hdr->type);
			*/
			ret = 0;
			goto out;
		}
#endif

	if (prNanSecKdeHdr->u1AttrId != NAN_ATTR_ID_SHARED_KEY_DESCRIPTOR) {
		DBGLOG(NAN, INFO,
		       "[%s] Error! attribute id is not for KDE:0x%x\n",
		       __func__, prNanSecKdeHdr->u1AttrId);
		goto out;
	}

	wpa_hexdump(MSG_MSGDUMP, "WPA: RX EAPOL-Key", buf, len);

#if 0
	if (plen > len - (sizeof(*prNanSecKdeHdr)+1) || plen < keyhdrlen) {
		/* Publish ID */
		/* wpa_dbg(sm->ctx->msg_ctx, MSG_DEBUG,
		 *		"WPA: EAPOL frame payload size %lu
		 *		invalid (frame size %lu)",
		 *		(unsigned long) plen, (unsigned long) len);
		 */
		ret = WLAN_STATUS_INVALID_LENGTH;
		goto out;
	}
#endif
	if (data_len < len) {
		wpa_dbg(sm->ctx->msg_ctx, MSG_DEBUG,
			"WPA: ignoring %lu bytes after the IEEE 802.1X data",
			(unsigned long)len - data_len);
	}

	/* Make a copy of the frame since we need to modify the buffer during
	 * MAC validation and Key Data decryption.
	 */
	/* tmp = os_malloc(data_len); */
	tmp = buf; /* In NAN, directly edit the buffer content from NDP */
	if (tmp == NULL)
		goto out;
#if 0
	os_memcpy(tmp, buf, data_len);

	key = (struct wpa_eapol_key *) (tmp +
			sizeof(struct ieee802_1x_hdr));

	key192 = (struct wpa_eapol_key_192 *)
		(tmp + sizeof(struct ieee802_1x_hdr));

#endif
	key = (struct wpa_eapol_key *)(tmp +
				       sizeof(struct _NAN_SEC_KDE_ATTR_HDR));
	key192 = (struct wpa_eapol_key_192
			  *)(tmp + sizeof(struct _NAN_SEC_KDE_ATTR_HDR));

	nanSecDumpEapolKey(key);

	if (mic_len == 24)
		key_data = (u8 *)(key192 + 1);
	else
		key_data = (u8 *)(key + 1);

	if (key->type != EAPOL_KEY_TYPE_WPA &&
	    key->type != EAPOL_KEY_TYPE_RSN) {
		wpa_dbg(sm->ctx->msg_ctx, MSG_DEBUG,
			"WPA: EAPOL-Key type (%d) unknown, discarded",
			key->type);
		ret = WLAN_STATUS_INVALID_DATA;
		goto out;
	}

	if (mic_len == 24)
		key_data_len = WPA_GET_BE16(key192->key_data_length);
	else
		key_data_len = WPA_GET_BE16(key->key_data_length);
#if 0
	wpa_eapol_key_dump(sm, key, key_data_len,
		key192->key_mic, mic_len);
#endif

	if (key_data_len > plen - keyhdrlen) {
		DBGLOG(NAN, INFO,
		       "[%s] Quit2. key_data_len:%d, plen:%d, keyhdrlen:%d\n",
		       __func__, key_data_len, plen, keyhdrlen);
		goto out;
	}

	eapol_sm_notify_lower_layer_success(sm->eapol, 0);
	key_info = WPA_GET_BE16(key->key_info);
	ver = key_info & WPA_KEY_INFO_TYPE_MASK;

#if 0
	if (ver != WPA_KEY_INFO_TYPE_HMAC_MD5_RC4 &&
#if defined(CONFIG_IEEE80211R) || defined(CONFIG_IEEE80211W)
		ver != WPA_KEY_INFO_TYPE_AES_128_CMAC &&
#endif /* CONFIG_IEEE80211R || CONFIG_IEEE80211W */
		ver != WPA_KEY_INFO_TYPE_HMAC_SHA1_AES &&
		!wpa_key_mgmt_suite_b(sm->key_mgmt) &&
		sm->key_mgmt != WPA_KEY_MGMT_OSEN) {
		wpa_msg(sm->ctx->msg_ctx, MSG_DEBUG,
			"WPA: Unsupported EAPOL-Key descriptor version %d",
			ver);
		goto out;
	}

	if (sm->key_mgmt == WPA_KEY_MGMT_OSEN &&
		ver != WPA_KEY_INFO_TYPE_AKM_DEFINED) {
		wpa_msg(sm->ctx->msg_ctx, MSG_DEBUG,
			"OSEN: Unsupported EAPOL-Key descriptor version %d",
			ver);
		goto out;
	}

	if (wpa_key_mgmt_suite_b(sm->key_mgmt) &&
		ver != WPA_KEY_INFO_TYPE_AKM_DEFINED) {
		wpa_msg(sm->ctx->msg_ctx, MSG_DEBUG,
			"RSN: Unsupported EAPOL-Key descriptor version %d (expected AKM defined = 0)",
			ver);
		goto out;
	}

#ifdef CONFIG_IEEE80211W
	if (wpa_key_mgmt_sha256(sm->key_mgmt)) {
		if (ver != WPA_KEY_INFO_TYPE_AES_128_CMAC &&
			sm->key_mgmt != WPA_KEY_MGMT_OSEN &&
			!wpa_key_mgmt_suite_b(sm->key_mgmt)) {
			goto out;
		}
	} else
#endif /* CONFIG_IEEE80211W */
#endif

#if 0
		if (sm->pairwise_cipher == WPA_CIPHER_CCMP &&
			!wpa_key_mgmt_suite_b(sm->key_mgmt) &&
			ver != WPA_KEY_INFO_TYPE_HMAC_SHA1_AES) {
			/* wpa_msg(sm->ctx->msg_ctx, MSG_DEBUG,
			 *	       "WPA: CCMP is used, but EAPOL-Key
			 *	       descriptor version (%d) is not 2", ver);
			 */
			if (sm->group_cipher != WPA_CIPHER_CCMP &&
				!(key_info & WPA_KEY_INFO_KEY_TYPE)) {
			/* Earlier versions of IEEE 802.11i did not
			 * explicitly
			 * require version 2 descriptor for all EAPOL-Key
			 * packets, so allow group keys to use version 1 if
			 * CCMP is not used for them.
			 */
			/* wpa_msg(sm->ctx->msg_ctx, MSG_DEBUG,
			 *      "WPA: Backwards compatibility:
			 *      allow invalid version for
			 *		non-CCMP group keys");
			 */
			} else if (ver == WPA_KEY_INFO_TYPE_AES_128_CMAC) {
				wpa_msg(sm->ctx->msg_ctx, MSG_DEBUG,
					"WPA: Interoperability workaround: allow incorrect (should have been HMAC-SHA1), but stronger (is AES-128-CMAC), descriptor version to be used");
			} else
				goto out;
		} else if (sm->pairwise_cipher == WPA_CIPHER_GCMP &&
				   !wpa_key_mgmt_suite_b(sm->key_mgmt) &&
				   ver != WPA_KEY_INFO_TYPE_HMAC_SHA1_AES) {
			/* wpa_msg(sm->ctx->msg_ctx, MSG_DEBUG,
			 *		"WPA: GCMP is used, but EAPOL-Key
			 *		descriptor version (%d) is not 2", ver);
			 */
			goto out;
		}

	if (!peerkey && sm->rx_replay_counter_set &&
		os_memcmp(key->replay_counter, sm->rx_replay_counter,
				  WPA_REPLAY_COUNTER_LEN) <= 0) {
		/* wpa_msg(sm->ctx->msg_ctx, MSG_WARNING,
		 *		"WPA: EAPOL-Key Replay Counter did not
		 *      increase - dropping packet");
		 */
		goto out;
	}
#endif

	if (!(key_info & (WPA_KEY_INFO_ACK | WPA_KEY_INFO_SMK_MESSAGE))) {
		wpa_msg(sm->ctx->msg_ctx, MSG_DEBUG,
			"WPA: No Ack bit in key_info");
		goto out;
	}

	if (key_info & WPA_KEY_INFO_REQUEST) {
		wpa_msg(sm->ctx->msg_ctx, MSG_DEBUG,
			"WPA: EAPOL-Key with Request bit - dropped");
		goto out;
	}

	/* MIC verification: zero MIC body */
	if ((key_info & WPA_KEY_INFO_MIC) && !peerkey &&
	    wpa_supplicant_verify_eapol_key_mic(sm, key192, ver, tmp,
						data_len)) {
		DBGLOG(NAN, INFO, "[%s] Quit3. MIC error!\n", __func__);
		goto out;
	}

	if ((sm->proto == WPA_PROTO_RSN || sm->proto == WPA_PROTO_OSEN) &&
	    (key_info & WPA_KEY_INFO_ENCR_KEY_DATA)) {
		if (wpa_supplicant_decrypt_key_data(sm, key, ver, key_data,
						    &key_data_len)) {
			DBGLOG(NAN, INFO, "[%s] Quit4. decrypt data error!\n",
			       __func__);
			goto out;
		}
	}

	if (key_info & WPA_KEY_INFO_KEY_TYPE) {
		if (key_info & WPA_KEY_INFO_KEY_INDEX_MASK) {
			/* wpa_msg(sm->ctx->msg_ctx, MSG_WARNING,
			 *		"WPA: Ignored EAPOL-Key (Pairwise) with
			 *		non-zero key index");
			 */
			DBGLOG(NAN, INFO, "[%s] Quit5. non-zero key index\n",
			       __func__);
			goto out;
		}
		if (key_info & WPA_KEY_INFO_MIC) {
			/* 3/4 4-Way Handshake */
			wpa_supplicant_process_3_of_4(sm, key, ver, key_data,
						      key_data_len);
		} else {
			/* 1/4 4-Way Handshake */
			wpa_supplicant_process_1_of_4(sm, src_addr, key, ver,
						      key_data, key_data_len);
		}
	} else {
		if (key_info & WPA_KEY_INFO_MIC) {
			/* 1/2 Group Key Handshake */
			wpa_supplicant_process_1_of_2(
				sm, src_addr, key,
				key_data, key_data_len, ver);
		} else {
			/* wpa_msg(sm->ctx->msg_ctx, MSG_WARNING,
			 *		"WPA: EAPOL-Key (Group) without
			 *      Mic bit - dropped");
			 */
			DBGLOG(NAN, INFO, "[%s] Quit6. Group without Mic bit\n",
			       __func__);
		}
	}

	ret = WLAN_STATUS_SUCCESS;

out:
	/*bin_clear_free(tmp, data_len);*/
	return ret;
}

uint32_t
nan_sec_wpa_receive(struct wpa_authenticator *wpa_auth, /* AP */
		    struct wpa_state_machine *sm, u8 *data, size_t data_len) {
	/* struct ieee802_1x_hdr *hdr; */
	struct wpa_eapol_key *key;
	struct wpa_eapol_key_192 *key192;
	u16 key_info, key_data_length;
	enum { PAIRWISE_2,
	       PAIRWISE_4,
	       GROUP_2,
	       REQUEST,
	       SMK_M1,
	       SMK_M3,
	       SMK_ERROR } msg;
	char *msgtxt;
	struct wpa_eapol_ie_parse kde;
	/* int ft; */
	/* const u8 *eapol_key_ie, *key_data; */
	/* size_t eapol_key_ie_len, keyhdrlen, mic_len; */
	const u8 *key_data;
	size_t keyhdrlen, mic_len;

	struct _NAN_SEC_KDE_ATTR_HDR *hdr;

	u8 zero_nonce[WPA_NONCE_LEN] = { 0 };

	if (wpa_auth == NULL || !wpa_auth->conf.wpa || sm == NULL) {
		DBGLOG(NAN, WARN, "[%s] Lacking of condition, exit", __func__);
		return WLAN_STATUS_FAILURE;
	}

/* TODO_CJ:remove IE */
#if 0
	kde.rsn_ie = NULL;
	kde.rsn_ie_len = 0;
	kde.wpa_ie = NULL;
	kde.wpa_ie_len = 0;
	kde.mac_addr = NULL;
#endif

	mic_len = wpa_mic_len(sm->wpa_key_mgmt);
	keyhdrlen = mic_len == 24 ? sizeof(*key192) : sizeof(*key);

	if (data_len < sizeof(*hdr) + keyhdrlen)
		return WLAN_STATUS_FAILURE;

	/* hdr = (struct ieee802_1x_hdr *) data; */
	hdr = (struct _NAN_SEC_KDE_ATTR_HDR *)data;
	key = (struct wpa_eapol_key *)(hdr + 1);
	key192 = (struct wpa_eapol_key_192 *)(hdr + 1);
	key_info = WPA_GET_BE16(key->key_info);
	if (mic_len == 24) {
		key_data = (const u8 *)(key192 + 1);
		key_data_length = WPA_GET_BE16(key192->key_data_length);
	} else {
		key_data = (const u8 *)(key + 1);
		key_data_length = WPA_GET_BE16(key->key_data_length);
	}
	if (key_data_length > data_len - sizeof(*hdr) - keyhdrlen) {
		DBGLOG(NAN, INFO,
		       "[%s] Quit1. key_data_length:%d, data_len:%d, sizeof(*hdr):%d, keyhdrlen:%d\n",
		       __func__, key_data_length, data_len, sizeof(*hdr),
		       sizeof(*hdr));
		return WLAN_STATUS_FAILURE;
	}

	if (sm->wpa == WPA_VERSION_WPA2) {
		if (key->type == EAPOL_KEY_TYPE_WPA) {
			/* Some deployed station implementations seem to send
			 * msg 4/4 with incorrect type value in WPA2 mode.
			 */
		} else if (key->type != EAPOL_KEY_TYPE_RSN) {
			DBGLOG(NAN, INFO, "[%s] Quit2. key->type:%d\n",
			       __func__, key->type);
			return WLAN_STATUS_FAILURE;
		}
	} else {
		if (key->type != EAPOL_KEY_TYPE_WPA) {
			DBGLOG(NAN, INFO, "[%s] Quit3. key->type:%d\n",
			       __func__, key->type);
			return WLAN_STATUS_FAILURE;
		}
	}

	wpa_hexdump(MSG_DEBUG, "WPA: Received Key Nonce", key->key_nonce,
		    WPA_NONCE_LEN);
	DBGLOG(NAN, INFO, "[%s] Received Key Nonce\n", __func__);
	dumpMemory8(key->key_nonce, WPA_NONCE_LEN);

	wpa_hexdump(MSG_DEBUG, "WPA: Received Replay Counter",
		    key->replay_counter, WPA_REPLAY_COUNTER_LEN);
	DBGLOG(NAN, INFO, "[%s] Received Replay Counter\n", __func__);
	dumpMemory8(key->replay_counter, WPA_REPLAY_COUNTER_LEN);

	/* FIX: verify that the EAPOL-Key frame was encrypted if pairwise keys
	 *		are set
	 */
#if 0
	if ((key_info & (WPA_KEY_INFO_SMK_MESSAGE | WPA_KEY_INFO_REQUEST)) ==
		(WPA_KEY_INFO_SMK_MESSAGE | WPA_KEY_INFO_REQUEST)) {
		if (key_info & WPA_KEY_INFO_ERROR) {
			msg = SMK_ERROR;
			msgtxt = "SMK Error";
		} else {
			msg = SMK_M1;
			msgtxt = "SMK M1";
		}
	} else if (key_info & WPA_KEY_INFO_SMK_MESSAGE) {
		msg = SMK_M3;
		msgtxt = "SMK M3";
	} else if (key_info & WPA_KEY_INFO_REQUEST) {
		msg = REQUEST;
		msgtxt = "Request";
	} else if (!(key_info & WPA_KEY_INFO_KEY_TYPE)) {
		msg = GROUP_2;
		msgtxt = "2/2 Group";
	} else if (key_data_length == 0) {
		msg = PAIRWISE_4;
		msgtxt = "4/4 Pairwise";
	} else {
		msg = PAIRWISE_2;
		msgtxt = "2/4 Pairwise";
	}
#endif
	if (os_memcmp(zero_nonce, key->key_nonce, WPA_NONCE_LEN)) {
		msg = PAIRWISE_2;
		msgtxt = "2/4 Pairwise";
		DBGLOG(NAN, INFO, "[%s] Judge as M2\n", __func__);
	} else {
		msg = PAIRWISE_4;
		msgtxt = "4/4 Pairwise";
		DBGLOG(NAN, INFO, "[%s] Judge as M4\n", __func__);
	}

#if 0 /* Skip eapol version check for NAN special case */
	/* TODO: key_info type validation for PeerKey */
	if (msg == REQUEST || msg == PAIRWISE_2 || msg == PAIRWISE_4 ||
		msg == GROUP_2) {
		u16 ver = key_info & WPA_KEY_INFO_TYPE_MASK;

		if (sm->pairwise == WPA_CIPHER_CCMP ||
			sm->pairwise == WPA_CIPHER_GCMP) {
			if (wpa_use_aes_cmac(sm) &&
				sm->wpa_key_mgmt != WPA_KEY_MGMT_OSEN &&
				!wpa_key_mgmt_suite_b(sm->wpa_key_mgmt) &&
				ver != WPA_KEY_INFO_TYPE_AES_128_CMAC) {
				/* wpa_auth_logger(wpa_auth, sm->addr,
				 *	LOGGER_WARNING,
				 *	"advertised support for
				 *	AES-128-CMAC, but did not use it");
				 */
				return;
			}

			if (!wpa_use_aes_cmac(sm) &&
				ver != WPA_KEY_INFO_TYPE_HMAC_SHA1_AES) {
				/* wpa_auth_logger(wpa_auth, sm->addr,
				 *		LOGGER_WARNING,
				 *		"did not use HMAC-SHA1-AES
				 *		with CCMP/GCMP");
				 */
				return;
			}
		}

		if (wpa_key_mgmt_suite_b(sm->wpa_key_mgmt) &&
			ver != WPA_KEY_INFO_TYPE_AKM_DEFINED) {
			wpa_auth_logger(wpa_auth, sm->addr, LOGGER_WARNING,
				"did not use EAPOL-Key descriptor version 0 as required for AKM-defined cases");
			return;
		}
	}
#endif

	if (key_info & WPA_KEY_INFO_REQUEST) {
		if (sm->req_replay_counter_used &&
		    os_memcmp(key->replay_counter, sm->req_replay_counter,
			      WPA_REPLAY_COUNTER_LEN) <= 0) {
			/* wpa_auth_logger(wpa_auth, sm->addr, LOGGER_WARNING,
			 *		"received EAPOL-Key request with
			 *		replayed counter");
			 */
			DBGLOG(NAN, INFO, "[%s] Quit4. replayed counter\n",
			       __func__);

			return WLAN_STATUS_FAILURE;
		}
	}

	if (!(key_info & WPA_KEY_INFO_REQUEST) &&
	    !wpa_replay_counter_valid(sm->key_replay, key->replay_counter)) {
		int i;

		if (msg == PAIRWISE_2 &&
		    wpa_replay_counter_valid(sm->prev_key_replay,
					     key->replay_counter) &&
		    sm->wpa_ptk_state == WPA_PTK_PTKINITNEGOTIATING &&
		    os_memcmp(sm->SNonce, key->key_nonce, WPA_NONCE_LEN) != 0) {
			/* Some supplicant implementations (e.g., Windows XP
			 * WZC) update SNonce for each EAPOL-Key 2/4. This
			 * breaks the workaround on accepting any of the
			 * pending requests, so allow the SNonce to be updated
			 * even if we have already sent out EAPOL-Key 3/4.
			 */
			/* wpa_auth_vlogger(wpa_auth, sm->addr, LOGGER_DEBUG,
			 *	 "Process SNonce update from STA
			 *	 based on retransmitted EAPOL-Key 1/4");
			 */
			sm->update_snonce = 1;
			os_memcpy(sm->alt_SNonce, sm->SNonce, WPA_NONCE_LEN);
			sm->alt_snonce_valid = TRUE;
			os_memcpy(sm->alt_replay_counter,
				  sm->key_replay[0].counter,
				  WPA_REPLAY_COUNTER_LEN);
			goto continue_processing;
		}

		if (msg == PAIRWISE_4 && sm->alt_snonce_valid &&
		    sm->wpa_ptk_state == WPA_PTK_PTKINITNEGOTIATING &&
		    os_memcmp(key->replay_counter, sm->alt_replay_counter,
			      WPA_REPLAY_COUNTER_LEN) == 0) {
			/* Supplicant may still be using the old SNonce since
			 * there was two EAPOL-Key 2/4 messages and they had
			 * different SNonce values.
			 */
			wpa_auth_vlogger(
				wpa_auth, sm->addr, LOGGER_DEBUG,
				"Try to process received EAPOL-Key 4/4 based on old Replay Counter and SNonce from an earlier EAPOL-Key 1/4");
			goto continue_processing;
		}

		if (msg == PAIRWISE_2 &&
		    wpa_replay_counter_valid(sm->prev_key_replay,
					     key->replay_counter) &&
		    sm->wpa_ptk_state == WPA_PTK_PTKINITNEGOTIATING) {
			/* wpa_auth_vlogger(wpa_auth, sm->addr, LOGGER_DEBUG,
			 *	 "ignore retransmitted EAPOL-Key %s -
			 *	 SNonce did not change", msgtxt);
			 */
		} else {
			/* wpa_auth_vlogger(wpa_auth, sm->addr, LOGGER_DEBUG,
			 *	 "received EAPOL-Key %s with
			 *	 unexpected replay counter", msgtxt);
			 */
		}
		for (i = 0; i < RSNA_MAX_EAPOL_RETRIES; i++) {
			if (!sm->key_replay[i].valid)
				break;
			wpa_hexdump(MSG_DEBUG, "pending replay counter",
				    sm->key_replay[i].counter,
				    WPA_REPLAY_COUNTER_LEN);
		}
		wpa_hexdump(MSG_DEBUG, "received replay counter",
			    key->replay_counter, WPA_REPLAY_COUNTER_LEN);

		DBGLOG(NAN, INFO, "[%s] Quit5. replay counter invalid\n",
		       __func__);

		return WLAN_STATUS_FAILURE;
	}

continue_processing:
	switch (msg) {
	case PAIRWISE_2:
		if (sm->wpa_ptk_state != WPA_PTK_PTKSTART &&
		    sm->wpa_ptk_state != WPA_PTK_PTKCALCNEGOTIATING &&
		    (!sm->update_snonce ||
		     sm->wpa_ptk_state != WPA_PTK_PTKINITNEGOTIATING)) {
			/* wpa_auth_vlogger(wpa_auth, sm->addr, LOGGER_INFO,
			 *	 "received EAPOL-Key msg 2/4 in
			 *	 invalid state (%d) - dropped",
			 *	 sm->wpa_ptk_state);
			 */
			DBGLOG(NAN, INFO,
			       "[%s] Quit6. invalid wpa_ptk_state:%d\n",
			       __func__, sm->wpa_ptk_state);

			return WLAN_STATUS_FAILURE;
		}
#if 0
		random_add_randomness(key->key_nonce, WPA_NONCE_LEN);
		if (sm->group->reject_4way_hs_for_entropy) {
			/* The system did not have enough entropy to generate
			 * strong random numbers. Reject the first 4-way
			 * handshake(s) and collect some entropy based on the
			 * information from it. Once enough entropy is
			 * available, the next atempt will trigger GMK/Key
			 * Counter update and the station will be allowed to
			 * continue.
			 */

			random_mark_pool_ready();
			/* wpa_sta_disconnect(wpa_auth, sm->addr); */
			/* TODO_CJ:terminate NDP */
			return WLAN_STATUS_FAILURE;
		}
#endif
#if 0
		if (wpa_parse_kde_ies(key_data, key_data_length, &kde) < 0) {
			/* wpa_auth_vlogger(wpa_auth, sm->addr, LOGGER_INFO,
			 *	 "received EAPOL-Key msg 2/4 with
			 *	 invalid Key Data contents");
			 */
			return WLAN_STATUS_FAILURE;
		}
		if (kde.rsn_ie) {
			eapol_key_ie = kde.rsn_ie;
			eapol_key_ie_len = kde.rsn_ie_len;
		} else {
			eapol_key_ie = kde.wpa_ie;
			eapol_key_ie_len = kde.wpa_ie_len;
		}
		ft = sm->wpa == WPA_VERSION_WPA2 &&
			 wpa_key_mgmt_ft(sm->wpa_key_mgmt);
		if (sm->wpa_ie == NULL ||
			wpa_compare_rsn_ie(ft,
					   sm->wpa_ie, sm->wpa_ie_len,
					   eapol_key_ie, eapol_key_ie_len)) {
			/* wpa_auth_logger(wpa_auth, sm->addr, LOGGER_INFO,
			 *	"WPA IE from (Re)AssocReq did not
			 *	match with msg 2/4");
			 */
			if (sm->wpa_ie) {
				wpa_hexdump(MSG_DEBUG, "WPA IE in AssocReq",
					sm->wpa_ie, sm->wpa_ie_len);
			}
			wpa_hexdump(MSG_DEBUG, "WPA IE in msg 2/4",
					eapol_key_ie, eapol_key_ie_len);
			/* MLME-DEAUTHENTICATE.request */
			/* wpa_sta_disconnect(wpa_auth, sm->addr); */
			/* TODO_CJ: NDP terminate */
			return WLAN_STATUS_FAILURE;
		}
#endif
		break;
	case PAIRWISE_4:
		if (sm->wpa_ptk_state != WPA_PTK_PTKINITNEGOTIATING ||
		    !sm->PTK_valid) {
			/* wpa_auth_vlogger(wpa_auth, sm->addr, LOGGER_INFO,
			 *	 "received EAPOL-Key msg 4/4 in
			 *	 invalid state (%d) - dropped",
			 *	 sm->wpa_ptk_state);
			 */
			DBGLOG(NAN, INFO,
			       "[%s] Quit7. invalid wpa_ptk_state:%d\n",
			       __func__, sm->wpa_ptk_state);

			return WLAN_STATUS_FAILURE;
		}
		break;
	case GROUP_2:
		if (sm->wpa_ptk_group_state != WPA_PTK_GROUP_REKEYNEGOTIATING ||
		    !sm->PTK_valid) {
			/* wpa_auth_vlogger(wpa_auth, sm->addr, LOGGER_INFO,
			 *		 "received EAPOL-Key msg 2/2 in
			 *		 invalid state (%d) - dropped",
			 *		 sm->wpa_ptk_group_state);
			 */
			DBGLOG(NAN, INFO,
			       "[%s] Quit8. invalid wpa_ptk_state:%d\n",
			       __func__, sm->wpa_ptk_state);

			return WLAN_STATUS_FAILURE;
		}
		break;
	case SMK_M1:
	case SMK_M3:
	case SMK_ERROR:
		return WLAN_STATUS_FAILURE;
		/* STSL disabled - ignore SMK messages */
	case REQUEST:
		break;
	}

	wpa_auth_vlogger(wpa_auth, sm->addr, LOGGER_DEBUG,
			 "received EAPOL-Key frame (%s)", msgtxt);

	if (key_info & WPA_KEY_INFO_ACK) {
		wpa_auth_logger(wpa_auth, sm->addr, LOGGER_INFO,
				"received invalid EAPOL-Key: Key Ack set");
		return WLAN_STATUS_FAILURE;
	}

	if (!(key_info & WPA_KEY_INFO_MIC)) {
		wpa_auth_logger(wpa_auth, sm->addr, LOGGER_INFO,
				"received invalid EAPOL-Key: Key MIC not set");
		return WLAN_STATUS_FAILURE;
	}

	sm->MICVerified = FALSE;
	if (sm->PTK_valid && !sm->update_snonce) {
		if (nan_sec_wpa_verify_key_mic(sm->wpa_key_mgmt, &sm->PTK, data,
					       data_len, sm)) {
			wpa_auth_logger(wpa_auth, sm->addr, LOGGER_INFO,
					"received EAPOL-Key with invalid MIC");
			return WLAN_STATUS_FAILURE;
		}
		sm->MICVerified = TRUE;
		eloop_cancel_timeout(wpa_send_eapol_timeout, wpa_auth, sm);
		sm->pending_1_of_4_timeout = 0;
	}

	if (key_info & WPA_KEY_INFO_REQUEST) {
		if (sm->MICVerified) {
			sm->req_replay_counter_used = 1;
			os_memcpy(sm->req_replay_counter, key->replay_counter,
				  WPA_REPLAY_COUNTER_LEN);
		} else {
			/* wpa_auth_logger(wpa_auth, sm->addr, LOGGER_INFO,
			 *	"received EAPOL-Key request with
			 *	invalid MIC");
			 */
			DBGLOG(NAN, INFO, "[%s] Quit9. invalid MIC\n",
			       __func__);

			return WLAN_STATUS_FAILURE;
		}

		/* TODO: should decrypt key data field if encryption was used;
		 * even though MAC address KDE is not normally encrypted,
		 * supplicant is allowed to encrypt it.
		 */
#if 0
		if (msg == SMK_ERROR) {
			return;
		}
#endif
		if (key_info & WPA_KEY_INFO_ERROR) {
			if (wpa_receive_error_report(
				    wpa_auth, sm,
				    !(key_info & WPA_KEY_INFO_KEY_TYPE)) > 0) {

				DBGLOG(NAN, INFO,
				       "[%s] Quit10. STA entry was removed\n",
				       __func__);
				return WLAN_STATUS_FAILURE;
				/* STA entry was removed */
			}
		} else if (key_info & WPA_KEY_INFO_KEY_TYPE) {
			/* wpa_auth_logger(wpa_auth, sm->addr, LOGGER_INFO,
			 *		"received EAPOL-Key Request for new
			 *		4-Way Handshake");
			 */
			wpa_request_new_ptk(sm);
		} else if (key_data_length > 0 &&
			   wpa_parse_kde_ies(key_data, key_data_length, &kde) ==
				   0 &&
			   kde.mac_addr) {
		} else {
			/* wpa_auth_logger(wpa_auth, sm->addr, LOGGER_INFO,
			 *		"received EAPOL-Key Request for GTK
			 *		rekeying");
			 */
			eloop_cancel_timeout(wpa_rekey_gtk, wpa_auth, NULL);
			wpa_rekey_gtk(wpa_auth, NULL);
		}
	} else {
		/* Do not allow the same key replay counter to be reused. */
		wpa_replay_counter_mark_invalid(sm->key_replay,
						key->replay_counter);

		if (msg == PAIRWISE_2) {
			/* Maintain a copy of the pending EAPOL-Key frames in
			 * case the EAPOL-Key frame was retransmitted. This is
			 * needed to allow EAPOL-Key msg 2/4 reply to another
			 * pending msg 1/4 to update the SNonce to work around
			 * unexpected supplicant behavior.
			 */
			os_memcpy(sm->prev_key_replay, sm->key_replay,
				  sizeof(sm->key_replay));
		} else {
			os_memset(sm->prev_key_replay, 0,
				  sizeof(sm->prev_key_replay));
		}

		/* Make sure old valid counters are not accepted anymore and
		 * do not get copied again.
		 */
		wpa_replay_counter_mark_invalid(sm->key_replay, NULL);
	}

	os_free(sm->last_rx_eapol_key);
	sm->last_rx_eapol_key = os_malloc(data_len);
	if (sm->last_rx_eapol_key == NULL)
		return WLAN_STATUS_FAILURE;
	os_memcpy(sm->last_rx_eapol_key, data, data_len);
	sm->last_rx_eapol_key_len = data_len;

	sm->rx_eapol_key_secure = !!(key_info & WPA_KEY_INFO_SECURE);
	sm->EAPOLKeyReceived = TRUE;
	sm->EAPOLKeyPairwise = !!(key_info & WPA_KEY_INFO_KEY_TYPE);
	sm->EAPOLKeyRequest = !!(key_info & WPA_KEY_INFO_REQUEST);
	os_memcpy(sm->SNonce, key->key_nonce, WPA_NONCE_LEN);
	wpa_sm_step(sm);

	return WLAN_STATUS_SUCCESS;
}
/************************************************
 *               STA Init Related
 ************************************************
 */
struct wpa_sm *
nan_sec_wpa_sm_init(struct wpa_sm_ctx *ctx, struct _NAN_NDP_INSTANCE_T *prNdp) {
	struct wpa_sm *sm;

	DBGLOG(NAN, INFO, "[%s] Enter\n", __func__);

	sm = prNdp->prResponderSecSmInfo;

	if (sm == NULL)
		return NULL;

	sm->renew_snonce = 1;
	sm->ctx = ctx;

	sm->dot11RSNAConfigPMKLifetime = 43200;
	sm->dot11RSNAConfigPMKReauthThreshold = 70;
	sm->dot11RSNAConfigSATimeout = 60;

	/* hard-code first, should from driver.
	 * NAN: set from nanSecSetCipherType()
	 */
#if 0
	sm->key_mgmt = WPA_KEY_MGMT_PSK;
	sm->pairwise_cipher = WPA_CIPHER_CCMP;
	sm->group_cipher = WPA_CIPHER_CCMP;
	sm->proto = WPA_PROTO_RSN;
	sm->mgmt_group_cipher = WPA_CIPHER_AES_128_CMAC;
#endif

	return sm;
}

int
nan_sec_wpa_supplicant_init_wpa(struct wpa_supplicant *wpa_s) {
	struct wpa_sm_ctx *ctx;

	ctx = &g_rNanWpaSmCtx; /* call-back should okay for keeping only one */
	os_memset(ctx, 0, sizeof(struct wpa_sm_ctx));

	ctx->ctx = wpa_s;
	ctx->msg_ctx = wpa_s;
	ctx->set_state = _wpa_supplicant_set_state;
	ctx->get_state = _wpa_supplicant_get_state;
	ctx->deauthenticate = _wpa_supplicant_deauthenticate;
	ctx->set_key = nan_sec_wpa_supplicant_set_key;
	ctx->get_bssid = wpa_supplicant_get_bssid;
	ctx->cancel_auth_timeout = _wpa_supplicant_cancel_auth_timeout;

#if 0
	wpa_s->wpa = wpa_sm_init(ctx);    /*TODO_CJ: Per NDP link*/
	if (wpa_s->wpa == NULL) {
		DBGLOG(NAN, ERROR, "Failed to initialize WPA state machine");
		os_free(ctx);
		return -1;
	}
#endif
	return 0;
}

void
nan_sec_wpa_supplicant_init_iface(void) {
	/* wpa init */
	if (nan_sec_wpa_supplicant_init_wpa(g_prNanWpaSupp) < 0)
		return;

	/* update settings */
	wpa_supplicant_set_state(g_prNanWpaSupp, WPA_DISCONNECTED);

	/* wpa_supplicant_set_bssid() */
	/* wpa_supplicant_set_ownmac() */
}

/************************************************
*               AP Init Related                 *
*************************************************/
void
nan_sec_hostapd_wpa_auth_conf(struct hostapd_bss_config *conf,
			      struct hostapd_config *iconf,
			      struct wpa_auth_config *wconf) {
	os_memset(wconf, 0, sizeof(*wconf));
	wconf->wpa = 2;
	wconf->wpa_key_mgmt = WPA_KEY_MGMT_PSK;
	wconf->wpa_pairwise = WPA_CIPHER_CCMP;
	wconf->wpa_group = WPA_CIPHER_CCMP;
	wconf->wpa_group_rekey = 600;
	/* wconf->wpa_strict_rekey = conf->wpa_strict_rekey; */
	wconf->wpa_gmk_rekey = 86400;
	/* wconf->wpa_ptk_rekey = conf->wpa_ptk_rekey; */
	wconf->rsn_pairwise = 0x10; /*CCMP*/
	wconf->eapol_version = EAPOL_VERSION;
}

struct wpa_authenticator *
nan_sec_wpa_init(const u8 *addr, struct wpa_auth_config *conf,
		 struct wpa_auth_callbacks *cb, int u1BssIdx) {
	struct wpa_authenticator *wpa_auth;

	DBGLOG(NAN, INFO, "[%s] Enter\n", __func__);

	wpa_auth = &g_rNanWpaAuth;
	os_memset(wpa_auth, 0, sizeof(struct wpa_authenticator));

	os_memcpy(wpa_auth->addr, addr, ETH_ALEN);
	os_memcpy(&wpa_auth->conf, conf, sizeof(*conf));
	os_memcpy(&wpa_auth->cb, cb, sizeof(*cb));

#if 1 /* TODO_CJ: GTK remove? */
	wpa_auth->group = wpa_group_init(wpa_auth, 0, 1);
	if (wpa_auth->group == NULL) {
		os_free(wpa_auth->wpa_ie);
		os_free(wpa_auth);
		return NULL;
	}

#if 0 /* Disable GTK timer until SPEC update */
	if (wpa_auth->conf.wpa_gmk_rekey) {
		eloop_register_timeout(wpa_auth->conf.wpa_gmk_rekey, 0,
					   wpa_rekey_gmk, wpa_auth, NULL);
	}

	if (wpa_auth->conf.wpa_group_rekey) {
		eloop_register_timeout(wpa_auth->conf.wpa_group_rekey, 0,
					   wpa_rekey_gtk, wpa_auth, NULL);
	}
#endif

#endif

	return wpa_auth;
}

int
nan_sec_hostapd_setup_wpa(struct hostapd_data *hapd) {
	struct wpa_auth_config _conf;
	struct wpa_auth_callbacks cb;

	nan_sec_hostapd_wpa_auth_conf(hapd->conf, hapd->iconf, &_conf);
	if (hapd->iface->drv_flags & WPA_DRIVER_FLAGS_EAPOL_TX_STATUS)
		_conf.tx_status = 1;
	if (hapd->iface->drv_flags & WPA_DRIVER_FLAGS_AP_MLME)
		_conf.ap_mlme = 1;
	os_memset(&cb, 0, sizeof(cb));
	cb.ctx = hapd;

	/* cb.disconnect = hostapd_wpa_auth_disconnect; */
	/* TODO_CJ: whether need to terminate? */

	cb.get_psk = hostapd_wpa_auth_get_psk;
	cb.set_key = nan_sec_hostapd_wpa_auth_set_key;
	cb.get_seqnum =
		hostapd_wpa_auth_get_seqnum;
		/* TODO_CJ: consider remove for GTK */
	cb.for_each_sta =
		hostapd_wpa_auth_for_each_sta;
		/* TODO_CJ: consider remove for GTK */

	hapd->wpa_auth =
		nan_sec_wpa_init(hapd->own_addr, &_conf, &cb, hapd->u1BssIdx);
	if (hapd->wpa_auth == NULL) {
		DBGLOG(NAN, ERROR, "WPA initialization failed.");
		return -1;
	}

	hapd->wpa_auth->u1BssIdx = hapd->u1BssIdx;

	DBGLOG(NAN, INFO, "[%s] hapd->wpa_auth->u1BssIdx:%d, hapd->u1BssIdx:%d",
	       __func__, hapd->wpa_auth->u1BssIdx, hapd->u1BssIdx);

	return 0;
}

int
nan_sec_hostapd_setup_bss(struct hostapd_data *hapd) {
	hostapd_wpa_auth_hapd_data_alloc(
		hapd, nanGetSpecificBssInfo(g_prAdapter, NAN_BSS_INDEX_BAND0)
			      ->ucBssIndex);
	nan_sec_hostapd_setup_wpa(hapd);
	wpa_init_keys(hapd->wpa_auth); /* TODO_CJ: Need GTK init? */

	return 0;
}

void
nan_sec_hostapd_init(void) {
	/* nan_sec_ap_sta_init(); */

	nan_sec_hostapd_setup_bss(g_prNanHapdData);
}
void
nan_sec_hostapd_deinit(void) {
	hostapd_deinit_wpa(g_prNanHapdData);
}

int
nan_sec_hostapd_notif_assoc(struct hostapd_data *hapd, const u8 *addr,
			    const u8 *req_ies, size_t req_ies_len,
			    int reassoc) {
	struct sta_info *sta = NULL;
	int new_assoc; /* res */
	struct ieee802_11_elems elems;
	const u8 *ie;
	size_t ielen;
	/* u16 reason = WLAN_REASON_UNSPECIFIED; */
	/* u16 status = WLAN_STATUS_SUCCESS; */
	const u8 *p2p_dev_addr = NULL;

	if (addr == NULL) {
		/* This could potentially happen with unexpected event from the
		 * driver wrapper. This was seen at least in one case where the
		 * driver ended up being set to station mode while hostapd was
		 * running, so better make sure we stop processing such an
		 * event here.
		 */
		DBGLOG(NAN, INFO,
		       "hostapd_notif_assoc: Skip event with no address");
		return -1;
	}
	random_add_randomness(addr, ETH_ALEN);

	hostapd_logger(hapd, addr, HOSTAPD_MODULE_IEEE80211, HOSTAPD_LEVEL_INFO,
		       "associated");

	ieee802_11_parse_elems(req_ies, req_ies_len, &elems, 0);
	if (elems.wps_ie) {
		ie = elems.wps_ie - 2;
		ielen = elems.wps_ie_len + 2;
		DBGLOG(NAN, INFO, "STA included WPS IE in (Re)AssocReq");
	} else if (elems.rsn_ie) {
		ie = elems.rsn_ie - 2;
		ielen = elems.rsn_ie_len + 2;
		DBGLOG(NAN, INFO, "STA included RSN IE in (Re)AssocReq");
	} else if (elems.wpa_ie) {
		ie = elems.wpa_ie - 2;
		ielen = elems.wpa_ie_len + 2;
		DBGLOG(NAN, INFO, "STA included WPA IE in (Re)AssocReq");
	} else {
		ie = NULL;
		ielen = 0;
		DBGLOG(NAN, INFO,
		       "STA did not include WPS/RSN/WPA IE in (Re)AssocReq");
	}

	/* sta = ap_get_sta(hapd, addr); */
	/*TODO_CJ: */
	if (sta == NULL) {
		/* wpa fail event */
		DBGLOG(NAN, ERROR,
		       "[%s] ERROR! corresponding sta_info is not found!",
		       __func__);
		return -1;
	}
	sta->flags &= ~(WLAN_STA_WPS | WLAN_STA_MAYBE_WPS | WLAN_STA_WPS2);

	if (hapd->conf->wpa) {
		if (ie == NULL || ielen == 0) {

			DBGLOG(NAN, INFO, "No WPA/RSN IE from STA");
			return -1;
		}

		if (sta->wpa_sm == NULL)
			sta->wpa_sm = wpa_auth_sta_init(
				hapd->wpa_auth, sta->addr, p2p_dev_addr);
		if (sta->wpa_sm == NULL) {
			DBGLOG(NAN, ERROR,
			       "Failed to initialize WPA state machine");
			return -1;
		}
#if 0 /* No IE , and key mgmt assignment not here */
		res = wpa_validate_wpa_ie(hapd->wpa_auth, sta->wpa_sm,
					  ie, ielen,
					  elems.mdie, elems.mdie_len);
		/* res	= WPA_IE_OK; */
		/* The IE is from driver */
		if (res != WPA_IE_OK) {
			DBGLOG(NAN, INFO,
				   "WPA/RSN information element rejected? (res %u)",
				   res);
			wpa_hexdump(MSG_DEBUG, "IE", ie, ielen);
			if (res == WPA_INVALID_GROUP) {
				reason = WLAN_REASON_GROUP_CIPHER_NOT_VALID;
				status = WLAN_STATUS_GROUP_CIPHER_NOT_VALID;
			} else if (res == WPA_INVALID_PAIRWISE) {
				reason = WLAN_REASON_PAIRWISE_CIPHER_NOT_VALID;
				status = WLAN_STATUS_PAIRWISE_CIPHER_NOT_VALID;
			} else if (res == WPA_INVALID_AKMP) {
				reason = WLAN_REASON_AKMP_NOT_VALID;
				status = WLAN_STATUS_AKMP_NOT_VALID;
			} else {
				reason = WLAN_REASON_INVALID_IE;
				status = WLAN_STATUS_INVALID_IE;
			}
			goto fail;
		}
#endif
	}

	new_assoc = (sta->flags & WLAN_STA_ASSOC) == 0;
	sta->flags |= WLAN_STA_AUTH | WLAN_STA_ASSOC;
	sta->flags &= ~WLAN_STA_WNM_SLEEP_MODE;

	wpa_auth_sm_event(sta->wpa_sm, WPA_ASSOC);

	/* hostapd_new_assoc_sta(hapd, sta, !new_assoc); */
	/* remove WPS related logic */
	wpa_auth_sta_associated(hapd->wpa_auth, sta->wpa_sm);

	/* ieee802_1x_notify_port_enabled(sta->eapol_sm, 1); */

	return 0;

	/* fail: */
#if 0
	wpas_evt_notify_send_deauth((P_UINT_8)addr,
		hapd->own_addr, reason, TRUE, hapd->u1BssIdx);
	return -1;
#endif
}

/************************************************
 *               Total Init Related
 ************************************************
 */
void
nan_sec_wpa_supplicant_start(void) {
	struct NETDEV_PRIVATE_GLUE_INFO *prNetDevPrivate = NULL;

	DBGLOG(NAN, INFO, "[%s] Enter\n", __func__);

	/* Get prAdapter */
	prNetDevPrivate =
		(struct NETDEV_PRIVATE_GLUE_INFO *)netdev_priv(gPrDev);
	if (prNetDevPrivate != NULL)
		g_prAdapter = prNetDevPrivate->prGlueInfo->prAdapter;

	/* CTX */
	kalMemZero(&g_rNanSecCtx, sizeof(struct _NAN_SEC_CTX));

	/* STA */
	nan_sec_wpa_supplicant_init_iface();

	/* AP */
	nan_sec_hostapd_init();

#if (CFG_NAN_SEC_UT == 1)
	/* UT */
	nanSecUtMain();
#endif
}

/************************************************
 *               Export API Related
 ************************************************
 */
uint32_t
nanSecGetNdpCsidAttr(IN struct _NAN_NDP_INSTANCE_T *prNdp,
		     OUT uint32_t *pu4CsidAttrLen,
		     OUT uint8_t **ppu1CsidAttrBuf) {
	struct _NAN_SEC_CSID_ATTR_HDR *prCsidAttrHdr = NULL;
	struct _NAN_SEC_CSID_ATTR_LIST *prCsidAttrListHdr = NULL;
	uint32_t u4TotalLen = 0;
	uint8_t *pucBuf;

	DBGLOG(NAN, INFO, "[%s] Enter\n", __func__);

	u4TotalLen = sizeof(struct _NAN_SEC_CSID_ATTR_HDR) +
		     sizeof(struct _NAN_SEC_CSID_ATTR_LIST);

	pucBuf = g_aucNanSecAttrBuffer;
	kalMemZero(pucBuf, NAN_IE_BUF_MAX_SIZE);

	prCsidAttrHdr = (struct _NAN_SEC_CSID_ATTR_HDR *)pucBuf;
	prCsidAttrHdr->u1AttrId = NAN_ATTR_ID_CIPHER_SUITE_INFO;
	prCsidAttrHdr->u2AttrLen =
		sizeof(struct _NAN_SEC_CSID_ATTR_LIST) + 1;
	/* Capabilities */
	prCsidAttrHdr->u1Cap = 0;

	/* Fill-in static Cipher list */
	prCsidAttrListHdr =
		(struct _NAN_SEC_CSID_ATTR_LIST
			 *)(pucBuf + sizeof(struct _NAN_SEC_CSID_ATTR_HDR));
	prCsidAttrListHdr->u1CipherType = prNdp->ucCipherType;
	prCsidAttrListHdr->u1PublishId = prNdp->ucPublishId;

	*ppu1CsidAttrBuf = pucBuf;
	*pu4CsidAttrLen = u4TotalLen;

	DBGLOG(NAN, INFO,
	       "[%s] output, *ppu1CsidAttrBuf:0x%p,  *pu4CsidAttrLen:%d\n",
	       __func__, *ppu1CsidAttrBuf, *pu4CsidAttrLen);
#if (ENABLE_SEC_UT_LOG == 1)
	dumpMemory8((uint8_t *)prCsidAttrHdr, u4TotalLen);
#endif

	return 0;
}

uint32_t
nanSecGetNdpScidAttr(IN struct _NAN_NDP_INSTANCE_T *prNdp,
		     OUT uint32_t *pu4ScidAttrLen,
		     OUT uint8_t **ppu1ScidAttrBuf) {
	struct _NAN_SEC_SCID_ATTR_HDR *prScidAttrHdr = NULL;
	struct _NAN_SEC_SCID_ATTR_ENTRY *pr1ScidAttrListHdr = NULL;
	uint32_t u4TotalLen = 0;
	uint8_t *pu1ScidPtr = NULL;
	uint8_t *pucBuf;

	DBGLOG(NAN, INFO, "[%s] Enter\n", __func__);

	u4TotalLen = sizeof(struct _NAN_SEC_SCID_ATTR_HDR) +
		     sizeof(struct _NAN_SEC_SCID_ATTR_ENTRY) +
		     sizeof(prNdp->au1Scid);

	pucBuf = g_aucNanSecAttrBuffer;
	kalMemZero(pucBuf, NAN_IE_BUF_MAX_SIZE);

	prScidAttrHdr = (struct _NAN_SEC_SCID_ATTR_HDR *)pucBuf;
	prScidAttrHdr->u1AttrId = NAN_ATTR_ID_SECURITY_CONTEXT_INFO;
	prScidAttrHdr->u2AttrLen =
		u4TotalLen - sizeof(struct _NAN_SEC_SCID_ATTR_HDR);

	pr1ScidAttrListHdr =
		(struct _NAN_SEC_SCID_ATTR_ENTRY
			 *)(pucBuf + sizeof(struct _NAN_SEC_SCID_ATTR_HDR));
	pr1ScidAttrListHdr->u2ScidLen = sizeof(prNdp->au1Scid);
	pr1ScidAttrListHdr->u1ScidType = 1; /* PMKID */
	pr1ScidAttrListHdr->u1PublishId = prNdp->ucPublishId;

	pu1ScidPtr = &pr1ScidAttrListHdr->u1PublishId + 1;
	kalMemCopy(pu1ScidPtr, prNdp->au1Scid, sizeof(prNdp->au1Scid));

	*ppu1ScidAttrBuf = pucBuf;
	*pu4ScidAttrLen = u4TotalLen;

	DBGLOG(NAN, INFO,
	       "[%s] output, *ppu1ScidAttrBuf:0x%p,  *pu4ScidAttrLen:%d\n",
	       __func__, *ppu1ScidAttrBuf, *pu4ScidAttrLen);

#if (ENABLE_SEC_UT_LOG == 1)
	dumpMemory8((uint8_t *)prScidAttrHdr, u4TotalLen);
#endif

	return 0;
}

uint32_t
nanSecGetCsidAttr(uint32_t *pu4CsidAttrLen, uint8_t **ppu1CsidAttrBuf) {
	struct _NAN_SEC_CSID_ATTR_HDR *prCsidAttrHdr = NULL;
	struct _NAN_SEC_CSID_ATTR_LIST *prCsidAttrListHdr = NULL;
	uint32_t u4TotalLen = 0;
	uint32_t u4CipherListLen = 0;
	struct _NAN_SEC_CIPHER_ENTRY *prCipherEntry = NULL;

	DBGLOG(NAN, INFO, "[%s] Enter\n", __func__);

	u4CipherListLen = sizeof(struct _NAN_SEC_CSID_ATTR_LIST) *
			  (g_rNanSecCtx.rNanSecCipherList.u4NumElem);
	u4TotalLen = sizeof(struct _NAN_SEC_CSID_ATTR_HDR) + u4CipherListLen;

#if (ENABLE_SEC_UT_LOG == 1)
	DBGLOG(NAN, INFO,
	       "[%s] len_ATTR_LIST:%d, len_ATTR_HDR:%d, u4NumElem:%d, u4CipherListLen:%d, u4TotalLen:%d\n",
	       __func__, sizeof(struct _NAN_SEC_CSID_ATTR_LIST),
	       sizeof(struct _NAN_SEC_CSID_ATTR_HDR),
	       (g_rNanSecCtx.rNanSecCipherList.u4NumElem), u4CipherListLen,
	       u4TotalLen);
#endif

	/* g_rNanSecCtx.pu1CsidAttrBuf = os_zalloc(u4TotalLen); */
	kalMemZero(g_aucNanSecAttrBuffer, NAN_IE_BUF_MAX_SIZE);
	g_rNanSecCtx.pu1CsidAttrBuf = g_aucNanSecAttrBuffer;

	if (g_rNanSecCtx.pu1CsidAttrBuf == NULL) {
		DBGLOG(NAN, ERROR,
		       "[%s] ERROR! os_zalloc failed for pu1CsidAttrBuf\n",
		       __func__);
		return -1;
	}
	g_rNanSecCtx.u4CsidAttrLen = u4TotalLen;

	prCsidAttrHdr =
		(struct _NAN_SEC_CSID_ATTR_HDR *)g_rNanSecCtx.pu1CsidAttrBuf;
	prCsidAttrHdr->u1AttrId = NAN_ATTR_ID_CIPHER_SUITE_INFO;
	prCsidAttrHdr->u2AttrLen = u4CipherListLen + 1;
	/* Capabilities */
	prCsidAttrHdr->u1Cap = 0;

	/* Fill-in static Cipher list */
	prCsidAttrListHdr = (struct _NAN_SEC_CSID_ATTR_LIST
				     *)(g_rNanSecCtx.pu1CsidAttrBuf +
					sizeof(struct _NAN_SEC_CSID_ATTR_HDR));
	prCipherEntry = (struct _NAN_SEC_CIPHER_ENTRY *)QUEUE_GET_HEAD(
		&g_rNanSecCtx.rNanSecCipherList);

	while (prCipherEntry != NULL) {
		prCsidAttrListHdr->u1CipherType = prCipherEntry->u4CipherType;
		prCsidAttrListHdr->u1PublishId = prCipherEntry->u2PublishId;

		prCsidAttrListHdr =
			prCsidAttrListHdr +
			1; /* sizeof(struct _NAN_SEC_CSID_ATTR_LIST) */
		prCipherEntry =
			(struct _NAN_SEC_CIPHER_ENTRY *)QUEUE_GET_NEXT_ENTRY(
				&prCipherEntry->rQueEntry);
	}

	*ppu1CsidAttrBuf = g_rNanSecCtx.pu1CsidAttrBuf;
	*pu4CsidAttrLen = g_rNanSecCtx.u4CsidAttrLen;

	DBGLOG(NAN, INFO,
	       "[%s] output, *ppu1CsidAttrBuf:0x%p,  *pu4CsidAttrLen:%d\n",
	       __func__, *ppu1CsidAttrBuf, *pu4CsidAttrLen);

#if (ENABLE_SEC_UT_LOG == 1)
	dumpMemory8((uint8_t *)prCsidAttrHdr, u4TotalLen);
#endif

	return 0;
}

uint32_t
nanSecInsertCipherList(IN uint32_t u4CipherType, IN uint16_t u2PublishId) {
	struct _NAN_SEC_CIPHER_ENTRY *prCipherEntry = NULL;

	DBGLOG(NAN, INFO, "[%s] Enter, u4CipherType:0x%x, u2PublishId:0x%x\n",
	       __func__, u4CipherType, u2PublishId);

	if (u4CipherType == 0)
		return WLAN_STATUS_NOT_ACCEPTED;

	/* Duplicate case handling */
	prCipherEntry = (struct _NAN_SEC_CIPHER_ENTRY *)QUEUE_GET_HEAD(
		&g_rNanSecCtx.rNanSecCipherList);

	while (prCipherEntry != NULL) {
		if (prCipherEntry->u2PublishId == u2PublishId) {
			DBGLOG(NAN, INFO,
			       "[%s] Find duplicate, old u4CipherType:0x%x, old u2PublishId:0x%x\n",
			       __func__, prCipherEntry->u4CipherType,
			       prCipherEntry->u2PublishId);

			prCipherEntry->u4CipherType = u4CipherType;
			return 0;
		}

		prCipherEntry =
			(struct _NAN_SEC_CIPHER_ENTRY *)QUEUE_GET_NEXT_ENTRY(
				&prCipherEntry->rQueEntry);
	}

	/* Insert the new one */
	prCipherEntry = (struct _NAN_SEC_CIPHER_ENTRY *)os_zalloc(
		sizeof(struct _NAN_SEC_CIPHER_ENTRY));
	if (prCipherEntry != NULL) {
		prCipherEntry->u4CipherType = u4CipherType;
		prCipherEntry->u2PublishId = u2PublishId;

		QUEUE_INSERT_TAIL(&g_rNanSecCtx.rNanSecCipherList,
				  &prCipherEntry->rQueEntry);
	} else {
		DBGLOG(NAN, ERROR, "[%s] os_zalloc failed for prCipherEntry\n",
		       __func__);
	}

	return WLAN_STATUS_SUCCESS;
}

uint32_t
nanSecFlushCipherList(void) {
	struct _NAN_SEC_CIPHER_ENTRY *prCipherEntry = NULL;

	DBGLOG(NAN, INFO, "[%s] Enter\n", __func__);

	while (QUEUE_IS_NOT_EMPTY(&g_rNanSecCtx.rNanSecCipherList)) {
		QUEUE_REMOVE_HEAD(&g_rNanSecCtx.rNanSecCipherList,
				  prCipherEntry,
				  struct _NAN_SEC_CIPHER_ENTRY *);
		if (prCipherEntry != NULL) {
			os_free(prCipherEntry);
			prCipherEntry = NULL;
		} else {
			DBGLOG(NAN, WARN,
			       "[%s] rNanSecCipherList is not empty but dequeue nothing, num:%d\n",
			       __func__,
			       g_rNanSecCtx.rNanSecCipherList.u4NumElem);
			return -1;
		}
	}

	return 0;
}

uint32_t
nanSecSetCipherType(IN struct _NAN_NDP_INSTANCE_T *prNdp,
		    IN uint32_t u4CipherType) {
	/* UINT_8  i; */
	int32_t i4TmpKeyMgmt = 0, i4TmpCipher = 0, i4TmpProto = 0,
	       i4TmpAuthAlg = 0, i4TmpKeyInfo = 0;

	DBGLOG(NAN, INFO, "[%s] Enter, eNDPRole:%d, u4CipherType:%d\n",
	       __func__, prNdp->eNDPRole, u4CipherType);

	/* Select chipher suit */
	switch (u4CipherType) {
	case NAN_CIPHER_SUITE_ID_NCS_SK_CCM_128:
		i4TmpKeyMgmt = WPA_KEY_MGMT_PSK_SHA256;
		i4TmpCipher = WPA_CIPHER_CCMP;
		i4TmpProto = WPA_PROTO_RSN;
		i4TmpAuthAlg = WPA_AUTH_ALG_OPEN;
		/* Seems not necessary */
		i4TmpKeyInfo =
			WPA_KEY_INFO_TYPE_AES_128_CMAC;
		break;

	case NAN_CIPHER_SUITE_ID_NCS_SK_GCM_256:
		i4TmpKeyMgmt = WPA_KEY_MGMT_PSK_SHA384;
		i4TmpCipher = WPA_CIPHER_GCMP;
		i4TmpProto = WPA_PROTO_RSN;
		i4TmpAuthAlg = WPA_AUTH_ALG_OPEN;
		/* Seems not necessary */
		i4TmpKeyInfo =
			WPA_KEY_INFO_TYPE_AES_128_CMAC;
		break;

	default:
		u4CipherType = NAN_CIPHER_SUITE_ID_NCS_SK_CCM_128;
		i4TmpKeyMgmt = WPA_KEY_MGMT_PSK_SHA256;
		i4TmpCipher = WPA_CIPHER_CCMP;
		i4TmpProto = WPA_PROTO_RSN;
		i4TmpAuthAlg = WPA_AUTH_ALG_OPEN;
		/* Seems not necessary */
		i4TmpKeyInfo =
			WPA_KEY_INFO_TYPE_AES_128_CMAC;

		break;
	}

	/* Assign into state machine */
	if (prNdp->eNDPRole ==
	    NAN_PROTOCOL_INITIATOR) {
	    /* TODO: integrate with nan_base defines */
		prNdp->prInitiatorSecSmInfo->u4SelCipherType = u4CipherType;
		prNdp->prInitiatorSecSmInfo->wpa_key_mgmt = i4TmpKeyMgmt;
		prNdp->prInitiatorSecSmInfo->pairwise = i4TmpCipher;
		prNdp->prInitiatorSecSmInfo->wpa = WPA_VERSION_WPA2;
	} else {
		prNdp->prResponderSecSmInfo->u4SelCipherType = u4CipherType;
		prNdp->prResponderSecSmInfo->key_mgmt = i4TmpKeyMgmt;
		prNdp->prResponderSecSmInfo->pairwise_cipher = i4TmpCipher;
		prNdp->prResponderSecSmInfo->group_cipher =
			i4TmpCipher; /* TODO_CJ: GTK remove? */
		prNdp->prResponderSecSmInfo->proto = i4TmpProto;
	}

	return 0;
}

uint32_t
nanSecSetPmk(IN struct _NAN_NDP_INSTANCE_T *prNdp, IN uint32_t u4PmkLen,
	     IN uint8_t *pu1Pmk) {
	DBGLOG(NAN, INFO, "[%s] Enter, u4PmkLen:%d, eNDPRole:%d\n", __func__,
	       u4PmkLen, prNdp->eNDPRole);

	if (u4PmkLen == PMK_LEN) {
		if (prNdp->eNDPRole == NAN_PROTOCOL_INITIATOR) {
			kalMemZero(prNdp->prInitiatorSecSmInfo->au1Psk,
				   PMK_LEN);
			kalMemCopy(prNdp->prInitiatorSecSmInfo->au1Psk, pu1Pmk,
				   u4PmkLen);
			prNdp->prInitiatorSecSmInfo->u4PskLen = u4PmkLen;
		} else {
			kalMemZero(prNdp->prResponderSecSmInfo->au1Psk,
				   PMK_LEN);
			kalMemCopy(prNdp->prResponderSecSmInfo->au1Psk, pu1Pmk,
				   u4PmkLen);
			prNdp->prResponderSecSmInfo->u4PskLen = u4PmkLen;
		}
	} else {
		/* PKCS5_PBKDF2_HMAC */
	}

	return 0;
}

uint32_t
nanSecNotify4wayBegin(IN struct _NAN_NDP_INSTANCE_T *prNdp) {
	DBGLOG(NAN, INFO, "[%s] Enter, eNDPRole:%d, NDPID:%d\n", __func__,
	       prNdp->eNDPRole, prNdp->ucNDPID);

	if (prNdp->eNDPRole == NAN_PROTOCOL_INITIATOR) {
		nanSecUpdatePmk(prNdp);

		prNdp->prInitiatorSecSmInfo->u1MicCalState =
			NAN_SEC_MIC_CAL_IDLE;
		prNdp->prInitiatorSecSmInfo->wpa_auth = &g_rNanWpaAuth;
		prNdp->prInitiatorSecSmInfo->pvNdp = (void *)prNdp;
		prNdp->prInitiatorSecSmInfo->wpa_auth->pvNdp = (void *)prNdp;

		wpa_auth_sta_init(prNdp->prInitiatorSecSmInfo->wpa_auth,
				  prNdp->prInitiatorSecSmInfo->addr, NULL);

		hostapd_wpa_auth_set_bssid(
			g_prNanHapdData,
			nanGetSpecificBssInfo(g_prAdapter, NAN_BSS_INDEX_BAND0)
				->aucClusterId);

		/* TODO_CJ: concurrent 4-way */
		hostapd_wpa_auth_set_ownmac(g_prNanHapdData,
					    prNdp->aucLocalNDIAddr);
		kalMemCopy(prNdp->prInitiatorSecSmInfo->addr,
			   prNdp->aucPeerNDIAddr, MAC_ADDR_LEN);

		wpa_auth_sta_associated(&g_rNanWpaAuth,
					prNdp->prInitiatorSecSmInfo);
		} else {
		/* NAN_NDP_RESPONDER */
		prNdp->prResponderSecSmInfo->u1MicCalState =
			NAN_SEC_MIC_CAL_IDLE;
		prNdp->prResponderSecSmInfo->pvNdp = (void *)prNdp;

		g_prNanWpaSupp->wpa = prNdp->prResponderSecSmInfo;

		/* wpa_supplicant_set_bssid(
		 *	g_prNanWpaSupp, prNdp->aucPeerNDIAddr);
		 */
		nanSecUpdatePeerNDI(prNdp, prNdp->aucPeerNDIAddr);

		/* TODO_CJ: concurrent 4-way */
		wpa_supplicant_set_ownmac(g_prNanWpaSupp,
					  prNdp->aucLocalNDIAddr);

		nan_sec_wpa_sm_init(&g_rNanWpaSmCtx,
				    prNdp); /* In Trooper, only 1 sta sm */

		nanSecUpdatePmk(prNdp);

		/* Sigma workaround: g_prNanWpaSupp is not assigned yet */
		wpa_supplicant_set_bssid(g_prNanWpaSupp, prNdp->aucPeerNDIAddr);

	}

	wpa_SYSrand_Gen_Rand_Seed(prNdp->aucLocalNDIAddr);

	return 0;
}

uint32_t
nanSecNotify4wayTerminate(IN struct _NAN_NDP_INSTANCE_T *prNdp) {
	DBGLOG(NAN, INFO, "[%s] Enter, eNDPRole:%d, NDPID:%d\n", __func__,
	       prNdp->eNDPRole, prNdp->ucNDPID);

	if (prNdp->eNDPRole == NAN_PROTOCOL_INITIATOR) {
		/* wpa sm back to disconnect */
		prNdp->prInitiatorSecSmInfo->Disconnect = TRUE;
		wpa_sm_step(prNdp->prInitiatorSecSmInfo);

		/* Orignal clean up */
		wpa_auth_sta_deinit(prNdp->prInitiatorSecSmInfo);

		/* NAN clean up */
		nanSecApSmBufReset(prNdp->prInitiatorSecSmInfo);

		/* Keep NDP index info */
		prNdp->prInitiatorSecSmInfo->pvNdp = (void *)prNdp;

		os_free(g_prNanHapdData->conf->ssid.wpa_psk);
		g_prNanHapdData->conf->ssid.wpa_psk = NULL;
	} else { /* NAN_NDP_RESPONDER */
		/* Orignal clean up */
		g_prNanWpaSupp->wpa->rx_replay_counter_set = 0;
		os_memset(g_prNanWpaSupp->wpa->rx_replay_counter, 0,
			  WPA_REPLAY_COUNTER_LEN);
		g_prNanWpaSupp->wpa->msg_3_of_4_ok = 0;

		g_prNanWpaSupp->wpa->ptk_set = 0;
		os_memset(&g_prNanWpaSupp->wpa->ptk, 0,
			  sizeof(g_prNanWpaSupp->wpa->ptk));
		g_prNanWpaSupp->wpa->tptk_set = 0;
		os_memset(&g_prNanWpaSupp->wpa->tptk, 0,
			  sizeof(g_prNanWpaSupp->wpa->tptk));
		os_memset(&g_prNanWpaSupp->wpa->gtk, 0,
			  sizeof(g_prNanWpaSupp->wpa->gtk));

		/* NAN clean up */
		nanSecStaSmBufReset(prNdp->prResponderSecSmInfo);
		kalMemZero(prNdp->prResponderSecSmInfo, sizeof(struct wpa_sm));

		/* Keep NDP index info */
		prNdp->prResponderSecSmInfo->pvNdp = (void *)prNdp;
	}

	/* Common */

	return 0;
}

uint32_t
nanSecTxKdeAttrDone(IN struct _NAN_NDP_INSTANCE_T *prNdp, IN uint8_t u1DstMsg) {
	u8 u1SmCurMsg = 0;
	u8 **ppu1SmTmpKdeAttrBuf = NULL;
	u32 *pu4SmTmpKdeAttrLen = NULL;
	bool *pfgIsTxDone = NULL;

	DBGLOG(NAN, INFO, "[%s] Enter, eNDPRole:%d, u1DstMsg:%d\n", __func__,
	       prNdp->eNDPRole, u1DstMsg);

	if (prNdp->eNDPRole == NAN_PROTOCOL_INITIATOR) {
		u1SmCurMsg = prNdp->prInitiatorSecSmInfo->u1CurMsg;
		ppu1SmTmpKdeAttrBuf =
			&prNdp->prInitiatorSecSmInfo->pu1TmpKdeAttrBuf;
		pu4SmTmpKdeAttrLen =
			&prNdp->prInitiatorSecSmInfo->u4TmpKdeAttrLen;
		pfgIsTxDone = &prNdp->prInitiatorSecSmInfo->fgIsTxDone;
	} else { /* NAN_NDP_RESPONDER */
		u1SmCurMsg = prNdp->prResponderSecSmInfo->u1CurMsg;
		ppu1SmTmpKdeAttrBuf =
			&prNdp->prResponderSecSmInfo->pu1TmpKdeAttrBuf;
		pu4SmTmpKdeAttrLen =
			&prNdp->prResponderSecSmInfo->u4TmpKdeAttrLen;
		pfgIsTxDone = &prNdp->prResponderSecSmInfo->fgIsTxDone;
	}

	if (u1SmCurMsg != u1DstMsg) {
		DBGLOG(NAN, ERROR, "[%s] ERROR! Msg mismatch, u1SmCurMsg:%d",
		       __func__, u1SmCurMsg);
		return -1;
	}

	os_free(*ppu1SmTmpKdeAttrBuf);
	*ppu1SmTmpKdeAttrBuf = NULL;
	*pu4SmTmpKdeAttrLen = 0;
	*pfgIsTxDone = TRUE;

	if (prNdp->eNDPRole == NAN_PROTOCOL_INITIATOR) {
		nanSecMicCalApSmStep(prNdp->prInitiatorSecSmInfo);
	} else { /* NAN_NDP_RESPONDER */
		nanSecMicCalStaSmStep(prNdp->prResponderSecSmInfo);
	}

	*pfgIsTxDone = FALSE;

	return WLAN_STATUS_SUCCESS;
}

uint32_t
nanSecRxKdeAttr(IN struct _NAN_NDP_INSTANCE_T *prNdp, IN uint8_t u1SrcMsg,
		IN uint32_t u4KdeAttrLen, IN uint8_t *pu1KdeAttrBuf,
		IN uint32_t u4RxMsgLen, IN uint8_t *pu1RxMsgBuf) {
	uint32_t rStatus = WLAN_STATUS_SUCCESS;
	u32 cipher;

	DBGLOG(NAN, INFO,
	       "[%s] Enter, eNDPRole:%d, u1SrcMsg:%d, u4KdeAttrLen:%d\n",
	       __func__, prNdp->eNDPRole, u1SrcMsg, u4KdeAttrLen);

#if (ENABLE_SEC_UT_LOG == 1)
	dumpMemory8(pu1KdeAttrBuf, u4KdeAttrLen);
	dumpMemory8(pu1RxMsgBuf, u4RxMsgLen);
#endif

	if (u1SrcMsg == NAN_SEC_END) {
		DBGLOG(NAN, INFO, "[%s] Rcv NDP terminate, return\n", __func__);
		return rStatus;
	}

	if (prNdp->eNDPRole == NAN_PROTOCOL_INITIATOR) {
		/* M2, M4 */
		prNdp->prInitiatorSecSmInfo->pu1GetRxMsgBodyBuf = pu1RxMsgBuf;
		prNdp->prInitiatorSecSmInfo->u4GetRxMsgBodyLen = u4RxMsgLen;

		prNdp->prInitiatorSecSmInfo->pu1GetRxMsgKdeBuf = pu1KdeAttrBuf;
		prNdp->prInitiatorSecSmInfo->u4GetRxMsgKdeLen = u4KdeAttrLen;

		rStatus = nan_sec_wpa_receive(&g_rNanWpaAuth,
					      prNdp->prInitiatorSecSmInfo,
					      pu1KdeAttrBuf, u4KdeAttrLen);

		cipher = prNdp->prInitiatorSecSmInfo->u4SelCipherType;
	} else { /* NAN_NDP_RESPONDER */

		if (u1SrcMsg == NAN_SEC_M1) {
			if (prNdp->prResponderSecSmInfo->pu1GetRxMsgBodyBuf !=
			    NULL)
				os_free(prNdp->prResponderSecSmInfo
						->pu1GetRxMsgBodyBuf);
			prNdp->prResponderSecSmInfo->pu1GetRxMsgBodyBuf =
				os_zalloc(u4RxMsgLen);
			kalMemCopy(
				prNdp->prResponderSecSmInfo->pu1GetRxMsgBodyBuf,
				pu1RxMsgBuf, u4RxMsgLen);
			prNdp->prResponderSecSmInfo->u4GetRxMsgBodyLen =
				u4RxMsgLen;
			prNdp->prResponderSecSmInfo->fgIsAllocRxMsgForM1 = TRUE;

			if (prNdp->prResponderSecSmInfo->pu1GetRxMsgKdeBuf !=
			    NULL)
				os_free(prNdp->prResponderSecSmInfo
						->pu1GetRxMsgKdeBuf);
			prNdp->prResponderSecSmInfo->pu1GetRxMsgKdeBuf =
				os_zalloc(u4KdeAttrLen);
			kalMemCopy(
				prNdp->prResponderSecSmInfo->pu1GetRxMsgKdeBuf,
				pu1KdeAttrBuf, u4KdeAttrLen);
			prNdp->prResponderSecSmInfo->u4GetRxMsgKdeLen =
				u4KdeAttrLen;

			DBGLOG(NAN, INFO,
			       "[%s] prResponderSecSmInfo:0x%p, u4GetRxKdeAttrLen:%d\n",
			       __func__, prNdp->prResponderSecSmInfo,
			       u4KdeAttrLen);
		} else if (u1SrcMsg == NAN_SEC_M3) {
#if 1
			if (prNdp->prResponderSecSmInfo->fgIsAllocRxMsgForM1) {
				os_free(prNdp->prResponderSecSmInfo
						->pu1GetRxMsgBodyBuf);
				os_free(prNdp->prResponderSecSmInfo
						->pu1GetRxMsgKdeBuf);

				prNdp->prResponderSecSmInfo
					->fgIsAllocRxMsgForM1 = FALSE;
			}
#endif
			prNdp->prResponderSecSmInfo->pu1GetRxMsgBodyBuf =
				pu1RxMsgBuf;
			prNdp->prResponderSecSmInfo->u4GetRxMsgBodyLen =
				u4RxMsgLen;

			prNdp->prResponderSecSmInfo->pu1GetRxMsgKdeBuf =
				pu1KdeAttrBuf;
			prNdp->prResponderSecSmInfo->u4GetRxMsgKdeLen =
				u4KdeAttrLen;

			rStatus = nan_sec_wpa_sm_rx_eapol(
				prNdp->prResponderSecSmInfo,
				prNdp->aucPeerNDIAddr);
		}
	}

	return rStatus;
}

uint32_t
nanSecNotifyMsgBodyRdy(IN struct _NAN_NDP_INSTANCE_T *prNdp,
		IN uint8_t u1SrcMsg, IN OUT uint32_t u4TxMsgLen,
		IN OUT uint8_t *pu1TxMsgBuf) {
	u8 u1SmCurMsg = 0;
	u8 **ppu1SmGetMsgBodyBuf = NULL;
	u32 *pu4SmGetMsgBodyLen = NULL;
	uint32_t rStatus = WLAN_STATUS_SUCCESS;

	DBGLOG(NAN, INFO,
	       "[%s] Enter, eNDPRole:%d, u1SrcMsg:%d, u4TxMsgLen:%d\n",
	       __func__, prNdp->eNDPRole, u1SrcMsg, u4TxMsgLen);

	if (prNdp->eNDPRole == NAN_PROTOCOL_INITIATOR) {
		u1SmCurMsg = prNdp->prInitiatorSecSmInfo->u1CurMsg;
		ppu1SmGetMsgBodyBuf =
			&prNdp->prInitiatorSecSmInfo->pu1GetTxMsgBodyBuf;
		pu4SmGetMsgBodyLen =
			&prNdp->prInitiatorSecSmInfo->u4GetTxMsgBodyLen;
	} else { /* NAN_NDP_RESPONDER */
		u1SmCurMsg = prNdp->prResponderSecSmInfo->u1CurMsg;
		ppu1SmGetMsgBodyBuf =
			&prNdp->prResponderSecSmInfo->pu1GetTxMsgBodyBuf;
		pu4SmGetMsgBodyLen =
			&prNdp->prResponderSecSmInfo->u4GetTxMsgBodyLen;
	}

	if (u1SmCurMsg != u1SrcMsg) {
		DBGLOG(NAN, ERROR, "[%s] ERROR! Msg mismatch, u1SmCurMsg:%d",
		       __func__, u1SmCurMsg);
		return -1;
	}

	*ppu1SmGetMsgBodyBuf = pu1TxMsgBuf;
	*pu4SmGetMsgBodyLen = u4TxMsgLen;

	if (u1SrcMsg == NAN_SEC_M1) {
		if (prNdp->prInitiatorSecSmInfo->pu1AuthTokenBuf != NULL)
			os_free(prNdp->prInitiatorSecSmInfo->pu1AuthTokenBuf);

		prNdp->prInitiatorSecSmInfo->pu1AuthTokenBuf =
			os_zalloc(NAN_AUTH_TOKEN_LEN);
		if (prNdp->prInitiatorSecSmInfo->pu1AuthTokenBuf == NULL) {
			DBGLOG(NAN, ERROR,
			       "[%s] os_zalloc failed for pu1AuthTokenBuf\n",
			       __func__);
			return WLAN_STATUS_FAILURE;
		}

		rStatus = nanSecGenAuthToken(
			prNdp->prInitiatorSecSmInfo->u4SelCipherType,
			pu1TxMsgBuf, u4TxMsgLen,
			prNdp->prInitiatorSecSmInfo->pu1AuthTokenBuf);

	} else { /* M2, M3, M4 */
		if (prNdp->eNDPRole == NAN_PROTOCOL_INITIATOR) {

			rStatus = nanSecMicCalApSmStep(
				prNdp->prInitiatorSecSmInfo);
		} else { /* NAN_NDP_RESPONDER */

			/* M1 Auth token pre-calculation */
			if (u1SrcMsg == NAN_SEC_M2) {
				if (prNdp->prResponderSecSmInfo
					    ->fgIsAllocRxMsgForM1 == FALSE) {
					DBGLOG(NAN, ERROR,
					       "[%s] no M1 msg for pu1AuthTokenBuf\n",
					       __func__);
					return WLAN_STATUS_FAILURE;
				}

				if (prNdp->prResponderSecSmInfo
					    ->pu1AuthTokenBuf != NULL)
					os_free(prNdp->prResponderSecSmInfo
							->pu1AuthTokenBuf);

				prNdp->prResponderSecSmInfo->pu1AuthTokenBuf =
					os_zalloc(NAN_AUTH_TOKEN_LEN);
				if (prNdp->prResponderSecSmInfo
					    ->pu1AuthTokenBuf == NULL) {
					DBGLOG(NAN, ERROR,
					       "[%s] os_zalloc failed for pu1AuthTokenBuf\n",
					       __func__);
					return WLAN_STATUS_FAILURE;
				}

				rStatus = nanSecGenAuthToken(
					prNdp->prResponderSecSmInfo
						->u4SelCipherType,
					prNdp->prResponderSecSmInfo
						->pu1GetRxMsgBodyBuf,
					prNdp->prResponderSecSmInfo
						->u4GetRxMsgBodyLen,
					prNdp->prResponderSecSmInfo
						->pu1AuthTokenBuf);
			}

			rStatus = nanSecMicCalStaSmStep(
				prNdp->prResponderSecSmInfo);
		}
	}

	return rStatus;
}

/************************************************
 *               Self-Use API Related
 ************************************************
 */
uint32_t
nanSecUpdatePmk(struct _NAN_NDP_INSTANCE_T *prNdp) {
	DBGLOG(NAN, INFO, "[%s] Enter\n", __func__);

	if (prNdp->eNDPRole == NAN_PROTOCOL_INITIATOR) {
		if (g_prNanHapdData->conf->ssid.wpa_psk == NULL)
			g_prNanHapdData->conf->ssid.wpa_psk =
				os_zalloc(sizeof(struct hostapd_wpa_psk));

		os_memset(g_prNanHapdData->conf->ssid.wpa_psk->psk, 0, PMK_LEN);
		kalMemCopy(g_prNanHapdData->conf->ssid.wpa_psk->psk,
			   prNdp->prInitiatorSecSmInfo->au1Psk,
			   prNdp->prInitiatorSecSmInfo->u4PskLen);
		g_prNanHapdData->conf->ssid.wpa_psk_set = 1;
	} else {
		os_memset(g_prNanWpaSupp->wpa->pmk, 0, PMK_LEN);
		kalMemCopy(g_prNanWpaSupp->wpa->pmk,
			   prNdp->prResponderSecSmInfo->au1Psk,
			   prNdp->prResponderSecSmInfo->u4PskLen);
		g_prNanWpaSupp->wpa->pmk_len =
			prNdp->prResponderSecSmInfo->u4PskLen;
	}

	return 0;
}

uint8_t
nanSecSelPtkKeyId(struct _NAN_NDP_INSTANCE_T *prNdp, uint8_t *pu1PeerAddr) {
	int8_t i1CurMaxPtkKeyId = -1;
	uint8_t i;

	/* DBGLOG(NAN, INFO, "[%s] Enter, u1NdpIdx:%d", __func__, u1NdpIdx); */

	if (prNdp->eNDPRole == NAN_PROTOCOL_INITIATOR) {
		for (i = 0; i < MAX_NDP_NUM; i++) {
			if (!kalMemCmp(g_arNanWpaAuthSm[i].au1RmtAddr,
				       pu1PeerAddr, ETH_ALEN)) {
				if ((g_arNanWpaAuthSm[i].fgPtkKeyIdSet) &&
				    (g_arNanWpaAuthSm[i].u1PtkKeyId >
				     i1CurMaxPtkKeyId)) {
					i1CurMaxPtkKeyId =
						g_arNanWpaAuthSm[i].u1PtkKeyId;
				}
			}
		}
	} else {
		for (i = 0; i < MAX_NDP_NUM; i++) {
			if (!kalMemCmp(g_arNanWpaSm[i].bssid, pu1PeerAddr,
				       ETH_ALEN)) {
				if ((g_arNanWpaSm[i].fgPtkKeyIdSet) &&
				    (g_arNanWpaSm[i].u1PtkKeyId >
				     i1CurMaxPtkKeyId)) {
					i1CurMaxPtkKeyId =
						g_arNanWpaSm[i].u1PtkKeyId;
				}
			}
		}
	}

	if (i1CurMaxPtkKeyId >= NAN_MAX_KEY_ID)
		return -1;
	else
		return (i1CurMaxPtkKeyId + 1);

	return 0;
}

uint32_t
nanSecMicCalStaSmStep(struct wpa_sm *sm) /* Send M2, M4 */
{
	struct wpa_eapol_key_192 *reply;
	uint8_t *pu1Kck = NULL;
	uint8_t u1KckLen = 0;

	uint32_t rStatus = WLAN_STATUS_SUCCESS;

	DBGLOG(NAN, INFO,
	       "[%s] Enter, state:%d, u4TmpKdeAttrLen:%d, u4GetTxMsgBodyLen:%d\n",
	       __func__, sm->u1MicCalState, sm->u4TmpKdeAttrLen,
	       sm->u4GetTxMsgBodyLen);

	if (sm->u1MicCalState == NAN_SEC_MIC_CAL_ERROR) { /*unlock until reset*/
		return WLAN_STATUS_FAILURE;
	}

	switch (sm->u1MicCalState) {
	case NAN_SEC_MIC_CAL_IDLE: {
		sm->u1MicCalState = NAN_SEC_MIC_CAL_WAIT;
		break;
	}

	case NAN_SEC_MIC_CAL_WAIT: {
		DBGLOG(NAN, INFO, "[%s] CAL_MIC_BEGIN\n", __func__);

		reply = (struct wpa_eapol_key_192
				 *)(sm->pu1TmpKdeAttrBuf +
				    sizeof(struct _NAN_SEC_KDE_ATTR_HDR));

		if (sm->u1CurMsg == NAN_SEC_M2) {
			pu1Kck = sm->tptk.kck;
			u1KckLen = sm->tptk.kck_len;
		} else {
			pu1Kck = sm->ptk.kck;
			u1KckLen = sm->ptk.kck_len;
		}

		if (nan_sec_wpa_eapol_key_mic(
			    pu1Kck, u1KckLen, sm->u4SelCipherType,
			    sm->pu1GetTxMsgBodyBuf, sm->u4GetTxMsgBodyLen,
			    reply->key_mic)) {
			DBGLOG(NAN, INFO,
			       "[%s] ERROR! nan_wpa_eapol_key_mic_wpa() failed\n",
			       __func__);
			sm->u1MicCalState = NAN_SEC_MIC_CAL_ERROR;
			return WLAN_STATUS_FAILURE;
		}

		/* Fill-in KDE for NDP */
		kalMemCopy(sm->pu1GetTxMsgKdeBuf, sm->pu1TmpKdeAttrBuf,
			   sm->u4TmpKdeAttrLen);

		sm->u1MicCalState = NAN_SEC_MIC_CAL_DONE;

		/* rStatus = nanNdpNotifySecAttrRdy(sm->u1NdpIdx); */
		/* TODO_CJ */
		rStatus = WLAN_STATUS_SUCCESS; /* notify NDP */

		DBGLOG(NAN, INFO, "[%s] CAL_MIC_DONE\n", __func__);

		break;
	}

	case NAN_SEC_MIC_CAL_DONE: {
		if (!sm->fgIsTxDone) {
			DBGLOG(NAN, INFO,
			       "[%s] Quit this time. Step Done must after TxDone\n",
			       __func__);
			rStatus = WLAN_STATUS_PENDING;
			break;
		}

		os_free(sm->pu1TmpKdeAttrBuf);
		sm->pu1TmpKdeAttrBuf = NULL;
		sm->u4TmpKdeAttrLen = 0;
		sm->u1MicCalState = NAN_SEC_MIC_CAL_IDLE;

		sm->pu1GetTxMsgBodyBuf = NULL;
		sm->u4GetTxMsgBodyLen = 0;
		sm->pu1GetTxMsgKdeBuf = NULL;

		rStatus = WLAN_STATUS_SUCCESS;

		break;
	}

	default:
		break;
	}

	return rStatus;
}

uint32_t
nanSecStaSmBufReset(struct wpa_sm *sm) {
	DBGLOG(NAN, INFO, "[%s] Enter\n", __func__);

	os_free(sm->pu1AuthTokenBuf);
	sm->pu1AuthTokenBuf = NULL;

	os_free(sm->pu1M3MicMaterialBuf);
	sm->pu1M3MicMaterialBuf = NULL;
	sm->u4M3MicMaterialLen = 0;

	/* os_free(sm->pu1GetTxMsgBodyBuf); */
	/* Buf from NDP */
	sm->pu1GetTxMsgBodyBuf = NULL;
	sm->u4GetTxMsgBodyLen = 0;
	sm->pu1GetTxMsgKdeBuf = NULL;

#if 1
	if (sm->fgIsAllocRxMsgForM1) {
		os_free(sm->pu1GetRxMsgBodyBuf);

		os_free(sm->pu1GetRxMsgKdeBuf);

		sm->fgIsAllocRxMsgForM1 = FALSE;
	}
#endif

	sm->pu1GetRxMsgBodyBuf = NULL;
	sm->u4GetRxMsgBodyLen = 0;

	sm->pu1GetRxMsgKdeBuf = NULL;
	sm->u4GetRxMsgKdeLen = 0;

	os_free(sm->pu1TmpKdeAttrBuf);
	sm->pu1TmpKdeAttrBuf = NULL;
	sm->u4TmpKdeAttrLen = 0;

	kalMemZero(sm, sizeof(struct wpa_sm));

	return WLAN_STATUS_SUCCESS;
}

uint32_t
nanSecMicCalApSmStep(struct wpa_state_machine *sm) /* Send M1, M3 */
{
	struct wpa_eapol_key_192 *reply;
	uint32_t rStatus = WLAN_STATUS_SUCCESS;

	DBGLOG(NAN, INFO,
	       "[%s] Enter, state:%d, u4TmpKdeAttrLen:%d, u4GetTxMsgBodyLen:%d\n",
	       __func__, sm->u1MicCalState, sm->u4TmpKdeAttrLen,
	       sm->u4GetTxMsgBodyLen);

	if (sm->u1MicCalState == NAN_SEC_MIC_CAL_ERROR) {
		/* unlock until reset */
		return WLAN_STATUS_FAILURE;
	}

	switch (sm->u1MicCalState) {
	case NAN_SEC_MIC_CAL_IDLE: {
		if (sm->u1CurMsg == NAN_SEC_M1) {
			DBGLOG(NAN, INFO, "[%s] for M1, direct send out\n",
			       __func__);
			sm->u1MicCalState = NAN_SEC_MIC_CAL_DONE;
		} else
			sm->u1MicCalState = NAN_SEC_MIC_CAL_WAIT;
		break;
	}

	case NAN_SEC_MIC_CAL_WAIT: {
		if (sm->pu1TmpKdeAttrBuf == NULL) {
			DBGLOG(NAN, INFO,
			       "[%s] ERROR!! pu1TmpKdeAttrBuf is NULL\n",
			       __func__);
			return WLAN_STATUS_FAILURE;
		}
		reply = (struct wpa_eapol_key_192
				 *)(sm->pu1TmpKdeAttrBuf +
				    sizeof(struct _NAN_SEC_KDE_ATTR_HDR));

		/* Gen (auth token||M3 body) */
		if (sm->pu1M3MicMaterialBuf != NULL)
			os_free(sm->pu1M3MicMaterialBuf);

		rStatus = nanSecGenM3MicMaterial(
			sm->pu1AuthTokenBuf, sm->pu1GetTxMsgBodyBuf,
			sm->u4GetTxMsgBodyLen, &sm->pu1M3MicMaterialBuf,
			&sm->u4M3MicMaterialLen);

		if (rStatus != WLAN_STATUS_SUCCESS)
			return WLAN_STATUS_FAILURE;

		/* MIC calulation */
		if (nan_sec_wpa_eapol_key_mic(
			    sm->PTK.kck, sm->PTK.kck_len, sm->u4SelCipherType,
			    sm->pu1M3MicMaterialBuf, sm->u4M3MicMaterialLen,
			    reply->key_mic)) {
			DBGLOG(NAN, INFO,
			       "[%s] ERROR! nan_wpa_eapol_key_mic_wpa() failed",
			       __func__);
			sm->u1MicCalState = NAN_SEC_MIC_CAL_ERROR;
			return WLAN_STATUS_FAILURE;
		}

		/* Fill-in KDE for NDP */
		kalMemCopy(sm->pu1GetTxMsgKdeBuf, sm->pu1TmpKdeAttrBuf,
			   sm->u4TmpKdeAttrLen);

		sm->u1MicCalState = NAN_SEC_MIC_CAL_DONE;

		/* rStatus = nanNdpNotifySecAttrRdy(sm->u1NdpIdx); */
		/* TODO_CJ: integrate with NDP */

		break;
	}

	case NAN_SEC_MIC_CAL_DONE: {
		if (!sm->fgIsTxDone) {
			DBGLOG(NAN, INFO,
			       "[%s] Quit this time. Step Done must after TxDone\n",
			       __func__);
			rStatus = WLAN_STATUS_PENDING;
			break;
		}

		sm->u1MicCalState = NAN_SEC_MIC_CAL_IDLE;

		os_free(sm->pu1TmpKdeAttrBuf);
		sm->pu1TmpKdeAttrBuf = NULL;
		sm->u4TmpKdeAttrLen = 0;

		sm->pu1GetTxMsgBodyBuf = NULL;
		sm->u4GetTxMsgBodyLen = 0;
		sm->pu1GetTxMsgKdeBuf = NULL;

		os_free(sm->pu1M3MicMaterialBuf);
		sm->pu1M3MicMaterialBuf = NULL;
		sm->u4M3MicMaterialLen = 0;

		rStatus = WLAN_STATUS_SUCCESS;

		break;
	}

	default:
		break;
	}

	return rStatus;
}

uint32_t
nanSecApSmBufReset(struct wpa_state_machine *sm) {
	DBGLOG(NAN, INFO, "[%s] Enter, sm:0x%p\n", __func__, sm);

	DBGLOG(NAN, INFO, "[%s] pu1AuthTokenBuf:0x%p\n", __func__,
	       sm->pu1AuthTokenBuf);
	if (sm->pu1AuthTokenBuf != NULL)
		dumpMemory8(sm->pu1AuthTokenBuf, NAN_AUTH_TOKEN_LEN);

	DBGLOG(NAN, INFO, "[%s] pu1M3MicMaterialBuf:0x%p\n", __func__,
	       sm->pu1M3MicMaterialBuf);
	if (sm->pu1M3MicMaterialBuf != NULL)
		dumpMemory8(sm->pu1M3MicMaterialBuf, sm->u4M3MicMaterialLen);

	DBGLOG(NAN, INFO, "[%s] pu1TmpKdeAttrBuf:0x%p\n", __func__,
	       sm->pu1TmpKdeAttrBuf);
	if (sm->pu1TmpKdeAttrBuf != NULL)
		dumpMemory8(sm->pu1TmpKdeAttrBuf, sm->u4TmpKdeAttrLen);

	os_free(sm->pu1AuthTokenBuf);
	sm->pu1AuthTokenBuf = NULL;

	os_free(sm->pu1M3MicMaterialBuf);
	sm->pu1M3MicMaterialBuf = NULL;
	sm->u4M3MicMaterialLen = 0;

	/* os_free(sm->pu1GetTxMsgBodyBuf); */
	/* Buf from NDP */
	sm->pu1GetTxMsgBodyBuf = NULL;
	sm->u4GetTxMsgBodyLen = 0;
	sm->pu1GetTxMsgKdeBuf = NULL;

	/* os_free(sm->pu1GetRxMsgBodyBuf); */
	/* Buf from NDP */
	sm->pu1GetRxMsgBodyBuf = NULL;
	sm->u4GetRxMsgBodyLen = 0;

	/* os_free(sm->pu1GetRxMsgKdeBuf); */
	/* Buf from NDP */
	sm->pu1GetRxMsgKdeBuf = NULL;
	sm->u4GetRxMsgKdeLen = 0;

	os_free(sm->pu1TmpKdeAttrBuf);
	sm->pu1TmpKdeAttrBuf = NULL;
	sm->u4TmpKdeAttrLen = 0;

	kalMemZero(sm,
		   sizeof(struct wpa_state_machine));
	/* TODO_CJ: better place */

	return WLAN_STATUS_SUCCESS;
}

uint32_t
nanSecGenAuthToken(u32 cipher, const u8 *auth_token_data,
		   size_t auth_token_data_len, u8 *auth_token) {
	u8 hash[SHA384_MAC_LEN];

	DBGLOG(NAN, INFO, "[%s] Enter, cipher:%d\n", __func__, cipher);

	if ((auth_token_data == NULL) || (auth_token_data_len == 0))
		DBGLOG(NAN, INFO, "[%s] ERROR! auth_token_data is NULL\n",
		       __func__);

#if (ENABLE_SEC_UT_LOG == 1)
	dumpMemory8((uint8_t *)auth_token_data, auth_token_data_len);
#endif

	if (cipher == NAN_CIPHER_SUITE_ID_NCS_SK_GCM_256) {
		if (sha384_vector(1, &auth_token_data, &auth_token_data_len,
				  hash)) {
			DBGLOG(NAN, INFO, "[%s] ERROR! sha256_vector() failed",
			       __func__);
			return WLAN_STATUS_FAILURE;
		}
		os_memcpy(auth_token, hash, NAN_AUTH_TOKEN_LEN);

#if (ENABLE_SEC_UT_LOG == 1)
		dumpMemory8((uint8_t *)auth_token, NAN_AUTH_TOKEN_LEN);
#endif
	} else {
		/* NAN_CIPHER_SUITE_ID_NCS_SK_CCM_128 */
		if (sha256_vector(1, &auth_token_data, &auth_token_data_len,
				  hash)) {
			DBGLOG(NAN, INFO, "[%s] ERROR! sha256_vector() failed",
			       __func__);
			return WLAN_STATUS_FAILURE;
		}
		os_memcpy(auth_token, hash, NAN_AUTH_TOKEN_LEN);

#if (ENABLE_SEC_UT_LOG == 1)
		dumpMemory8((uint8_t *)auth_token, NAN_AUTH_TOKEN_LEN);
#endif
	}

	return WLAN_STATUS_SUCCESS;
}

uint32_t
nanSecGenM3MicMaterial(IN uint8_t *pu1AuthTokenBuf, IN const u8 *pu1M3bodyBuf,
		       IN uint32_t u4M3BodyLen,
		       OUT uint8_t **ppu1M3MicMaterialBuf,
		       OUT uint32_t *pu4M3MicMaterialLen) {
	uint32_t u4TotalLen = 0;
	uint8_t *pu1MicMaterialBuf = NULL;

	DBGLOG(NAN, INFO, "[%s] Enter\n", __func__);

	if (pu1AuthTokenBuf == NULL) {
		DBGLOG(NAN, ERROR, "[%s] ERROR! pu1MicMaterialBuf is NULL",
		       __func__);
		return WLAN_STATUS_FAILURE;
	}

	u4TotalLen = u4M3BodyLen + NAN_AUTH_TOKEN_LEN;
	pu1MicMaterialBuf = os_zalloc(u4TotalLen);

	if (pu1MicMaterialBuf == NULL) {
		DBGLOG(NAN, ERROR,
		       "[%s] ERROR! os_zalloc failed for pu1MicMaterialBuf",
		       __func__);
		return WLAN_STATUS_FAILURE;
	}

	*ppu1M3MicMaterialBuf = pu1MicMaterialBuf;
	*pu4M3MicMaterialLen = u4TotalLen;

	kalMemCopy(pu1MicMaterialBuf, pu1AuthTokenBuf, NAN_AUTH_TOKEN_LEN);
	kalMemCopy(pu1MicMaterialBuf + NAN_AUTH_TOKEN_LEN, pu1M3bodyBuf,
		   u4M3BodyLen);

	DBGLOG(NAN, INFO, "[%s] pu1AuthTokenBuf:\n", __func__);
	dumpMemory8(pu1AuthTokenBuf, NAN_AUTH_TOKEN_LEN);

	DBGLOG(NAN, INFO, "[%s] pu1M3bodyBuf:\n", __func__);
	dumpMemory8((uint8_t *)pu1M3bodyBuf, u4M3BodyLen);

	DBGLOG(NAN, INFO, "[%s] pu1MicMaterialBuf:\n", __func__);
	dumpMemory8(pu1MicMaterialBuf, u4TotalLen);

	return WLAN_STATUS_SUCCESS;
}

uint16_t
nanSecCalKdeAttrLenFunc(struct _NAN_NDP_INSTANCE_T *prNdp) {
	if (prNdp->eNDPRole ==
	    NAN_PROTOCOL_INITIATOR) {
		/* TODO: integrate with nan_base defines */
		return prNdp->prInitiatorSecSmInfo->u4TmpKdeAttrLen;
	} else {
		return prNdp->prResponderSecSmInfo->u4TmpKdeAttrLen;
	}
}

void
nanSecAppendKdeAttrFunc(struct _NAN_NDP_INSTANCE_T *prNdp,
			struct MSDU_INFO *prMsduInfo) {
	uint8_t *pu1TmpKdeAttrBuf = NULL;
	uint32_t u4TmpKdeAttrLen = 0;

	uint8_t *pu1MsduKdePos = NULL;

	/* Pivot the KDE beginning ptr */
	pu1MsduKdePos = prMsduInfo->prPacket + prMsduInfo->u2FrameLength;

	if (prNdp->eNDPRole ==
	    NAN_PROTOCOL_INITIATOR) {
	    /* TODO: integrate with nan_base defines */
		pu1TmpKdeAttrBuf =
			prNdp->prInitiatorSecSmInfo->pu1TmpKdeAttrBuf;
		u4TmpKdeAttrLen = prNdp->prInitiatorSecSmInfo->u4TmpKdeAttrLen;

		prNdp->prInitiatorSecSmInfo->pu1GetTxMsgKdeBuf = pu1MsduKdePos;
	} else {
		pu1TmpKdeAttrBuf =
			prNdp->prResponderSecSmInfo->pu1TmpKdeAttrBuf;
		u4TmpKdeAttrLen = prNdp->prResponderSecSmInfo->u4TmpKdeAttrLen;

		prNdp->prResponderSecSmInfo->pu1GetTxMsgKdeBuf = pu1MsduKdePos;
	}

	kalMemCopy(pu1MsduKdePos, pu1TmpKdeAttrBuf, u4TmpKdeAttrLen);
	prMsduInfo->u2FrameLength += u4TmpKdeAttrLen;
}

struct wpa_state_machine *
nanSecGetInitiatorSm(uint8_t u1Index) {
	return &g_arNanWpaAuthSm[u1Index];
}

struct wpa_sm *
nanSecGetResponderSm(uint8_t u1Index) {
	return &g_arNanWpaSm[u1Index];
}

void
nanSecResetTk(struct STA_RECORD *prStaRec) {
	prStaRec->rPmfCfg.fgApplyPmf = FALSE;

	nan_sec_wpas_setkey_glue(FALSE, prStaRec->ucBssIndex, 0,
				 prStaRec->aucMacAddr, 0, NULL, 0);
}

void
nanSecInstallTk(struct _NAN_NDP_INSTANCE_T *prNdp,
		struct STA_RECORD *prStaRec) {
	uint8_t *pu1Tk = NULL;
	uint8_t u1TkLen = 0;
	uint32_t rStatus = WLAN_STATUS_SUCCESS;
	uint32_t u4Cipher;
	enum wpa_alg alg;

	DBGLOG(NAN, INFO, "[%s] Enter, StaIdx:%d, BssIdx:%d\n", __func__,
	       prStaRec->ucIndex, prStaRec->ucBssIndex);

	prStaRec->rPmfCfg.fgApplyPmf = TRUE;

	if (prNdp->eNDPRole == NAN_PROTOCOL_INITIATOR) {
		pu1Tk = prNdp->prInitiatorSecSmInfo->PTK.tk;
		u1TkLen = prNdp->prInitiatorSecSmInfo->PTK.tk_len;
		u4Cipher = prNdp->prInitiatorSecSmInfo->u4SelCipherType;
	} else {
		pu1Tk = prNdp->prResponderSecSmInfo->ptk.tk;
		u1TkLen = prNdp->prResponderSecSmInfo->ptk.tk_len;
		u4Cipher = prNdp->prResponderSecSmInfo->u4SelCipherType;
	}

	dumpMemory8(pu1Tk, u1TkLen);

	/* TODO_CJ: dynamic chiper, dynamic keyID */
	if (u4Cipher == NAN_CIPHER_SUITE_ID_NCS_SK_GCM_256)
		alg = WPA_ALG_GCMP_256;
	else
		alg = WPA_ALG_CCMP;
	rStatus = nan_sec_wpas_setkey_glue(FALSE, prStaRec->ucBssIndex, alg,
					   prStaRec->aucMacAddr, 0, pu1Tk,
					   u1TkLen);

	if (rStatus == WLAN_STATUS_SUCCESS) {
		if (prNdp->eNDPRole == NAN_PROTOCOL_INITIATOR)
			prNdp->prInitiatorSecSmInfo->PTK.installed = 1;
		else
			prNdp->prResponderSecSmInfo->ptk.installed = 1;
	}
}

void
nanSecUnload(void) {
	/* TODO_CJ: counter nan_sec_wpa_supplicant_start() */
}

void
nanSecDumpEapolKey(struct wpa_eapol_key *key) {
	DBGLOG(NAN, INFO, "[%s] Enter\n", __func__);
	DBGLOG(NAN, INFO, "[%s] type:0x%x\n", __func__, key->type);
	DBGLOG(NAN, INFO, "[%s] key_info:0x%x, 0x%x\n", __func__,
	       key->key_info[0], key->key_info[1]);
	DBGLOG(NAN, INFO, "[%s] key_length:0x%x, 0x%x\n", __func__,
	       key->key_length[0], key->key_length[1]);

	DBGLOG(NAN, INFO, "[%s] replay counter:\n", __func__);
	dumpMemory8(key->replay_counter, WPA_REPLAY_COUNTER_LEN);

	DBGLOG(NAN, INFO, "[%s] key_nonce:\n", __func__);
	dumpMemory8(key->key_nonce, WPA_NONCE_LEN);

	DBGLOG(NAN, INFO, "[%s] key_iv:\n", __func__);
	dumpMemory8(key->key_iv, 16);

	DBGLOG(NAN, INFO, "[%s] key_rsc:\n", __func__);
	dumpMemory8(key->key_rsc, WPA_KEY_RSC_LEN);

	DBGLOG(NAN, INFO, "[%s] key_id:\n", __func__);
	dumpMemory8(key->key_id, 8);

	DBGLOG(NAN, INFO, "[%s] key_mic:\n", __func__);
	dumpMemory8(key->key_mic, 16);

	DBGLOG(NAN, INFO, "[%s] key_data_length:0x%x, 0x%x\n", __func__,
	       key->key_data_length[0], key->key_data_length[1]);
}

void
nanSecUpdateAttrCmd(IN struct ADAPTER *prAdapter, uint8_t aucAttrId,
		    uint8_t *aucAttrBuf, uint16_t u2AttrLen) {
	uint32_t rStatus;
	void *prCmdBuffer;
	uint32_t u4CmdBufferLen;
	struct _CMD_EVENT_TLV_COMMOM_T *prTlvCommon = NULL;
	struct _CMD_EVENT_TLV_ELEMENT_T *prTlvElement = NULL;
	struct _NAN_CMD_UPDATE_ATTR_STRUCT *prCmdNanCmdUpdateAttr = NULL;

	DBGLOG(NAN, INFO, "[%s] Enter, aucAttrId:0x%x, aucAttrLen:%d\n",
	       __func__, aucAttrId, u2AttrLen);

	u4CmdBufferLen = sizeof(struct _CMD_EVENT_TLV_COMMOM_T) +
			 sizeof(struct _CMD_EVENT_TLV_ELEMENT_T) +
			 sizeof(struct _NAN_CMD_UPDATE_ATTR_STRUCT);
	prCmdBuffer = cnmMemAlloc(prAdapter, RAM_TYPE_BUF, u4CmdBufferLen);

	if (!prCmdBuffer) {
		DBGLOG(CNM, ERROR, "Memory allocation fail\n");
		return;
	}

	prTlvCommon = (struct _CMD_EVENT_TLV_COMMOM_T *)prCmdBuffer;

	prTlvCommon->u2TotalElementNum = 0;

	rStatus = nicAddNewTlvElement(
		NAN_CMD_UPDATE_ATTR, sizeof(struct _NAN_CMD_UPDATE_ATTR_STRUCT),
		u4CmdBufferLen, prCmdBuffer);

	if (rStatus != WLAN_STATUS_SUCCESS) {
		DBGLOG(TX, ERROR, "Add new Tlv element fail\n");
		cnmMemFree(prAdapter, prCmdBuffer);
		return;
	}

	prTlvElement = nicGetTargetTlvElement(1, prCmdBuffer);

	if (prTlvElement == NULL) {
		DBGLOG(TX, ERROR, "Get target Tlv element fail\n");
		cnmMemFree(prAdapter, prCmdBuffer);
		return;
	}

	prCmdNanCmdUpdateAttr =
		(struct _NAN_CMD_UPDATE_ATTR_STRUCT *)prTlvElement->aucbody;
	prCmdNanCmdUpdateAttr->ucAttrId = aucAttrId;
	prCmdNanCmdUpdateAttr->u2AttrLen = u2AttrLen;
	kalMemCopy(&prCmdNanCmdUpdateAttr->aucAttrBuf[0], aucAttrBuf,
		   u2AttrLen);

	rStatus = wlanSendSetQueryCmd(prAdapter, CMD_ID_NAN_EXT_CMD, TRUE,
				      FALSE, FALSE, NULL, NULL, u4CmdBufferLen,
				      (uint8_t *)prCmdBuffer, NULL, 0);

	cnmMemFree(prAdapter, prCmdBuffer);
}

void
nanSecUpdatePeerNDI(struct _NAN_NDP_INSTANCE_T *prNdp,
		uint8_t *au1PeerNdiAddr) {
	DBGLOG(NAN, INFO, "[%s] Enter, role:%d\n", __func__, prNdp->eNDPRole);

	if (prNdp->eNDPRole == NAN_PROTOCOL_INITIATOR)
		kalMemCopy(prNdp->prInitiatorSecSmInfo->addr, au1PeerNdiAddr,
			   MAC_ADDR_LEN);
}

int32_t
nanSecCompareSA(IN struct ADAPTER *prAdapter,
		IN struct _NAN_NDP_INSTANCE_T *prNdp1,
		IN struct _NAN_NDP_INSTANCE_T *prNdp2) {
	uint32_t au4Rank[2];
	struct _NAN_NDP_INSTANCE_T *aprNdp[2];
	uint32_t u4Idx;

	aprNdp[0] = prNdp1;
	aprNdp[1] = prNdp2;
	for (u4Idx = 0; u4Idx < 2; u4Idx++) {
		au4Rank[u4Idx] = 0;

		if (aprNdp[u4Idx]->fgSecurityRequired == TRUE) {
			au4Rank[u4Idx] += 1;

			if (aprNdp[u4Idx]->ucCipherType ==
			    NAN_CIPHER_SUITE_ID_NCS_SK_GCM_256)
				au4Rank[u4Idx] += 2;
			else if (aprNdp[u4Idx]->ucCipherType ==
				 NAN_CIPHER_SUITE_ID_NCS_SK_CCM_128)
				au4Rank[u4Idx] += 1;
			else
				DBGLOG(NAN, ERROR, "ucCipherType error!\n");
		}
	}

	if (au4Rank[0] < au4Rank[1])
		return -1;
	else if (au4Rank[0] == au4Rank[1])
		return 0;
	else
		return 1;
}

/************************************************
 *               NDP Sudo Related
 ************************************************
 */
uint32_t
nanNdpGetMsgBody(IN uint8_t u1NdpIdx, IN uint8_t u1Msg, IN uint8_t u1MicMode,
		 OUT uint32_t *pu4MsgBodyLen, OUT uint8_t **ppu1MsgBody) {
	return 0;
}

uint32_t
nanNdpNotifySecAttrRdy(IN uint8_t u1NdpIdx) {
	return 0;
}

uint32_t
nanNdpGetNdiAddr(uint8_t u1NdpIdx, uint8_t u1Role, uint8_t *pu1MacAddr) {
	return 0;
}

uint32_t
nanNdpGetPublishId(uint8_t *u1NdpIdx) {
	return 0;
}

uint32_t
nanNdpNotifySecStatus(uint8_t u1NdpIdx, uint8_t u1Status, uint8_t u1Reason,
		      uint8_t u1Msg) {
	return 0;
}

uint8_t
nanNdpGetWlanIdx(uint8_t u1NdpIdx) {
	return 0;
}

#if 0
void testSecCaller(void)
{
	nanSecAllocCsidAttr(NULL, NULL);
	nanSecFreeCsidAttr(NULL);
	nanSecAllocScidAttr(NULL, NULL);
	nanSecFreeScidAttr(NULL);

	nanSecSetCipherType(0, 0);
	nanSecSetPmk(0, 0, NULL);
	nanSecSetScid(0, NULL);

	nanSecNotify4wayBegin(0);
	nanSecNotify4wayTerminate(0);
	nanSecTxKdeAttrDone(0, 0, 0);
	nanSecRxKdeAttr(0, 0, 0, NULL);
	nanSecNotifyMsgBodyRdy(0, 0, 0, 0, NULL);

	nanSecSelPtkKeyId(0, NULL);
	nanSecUpdatePmk(0);

	nan_sec_wpa_eapol_key_mic(NULL, 0, 0,
					  NULL, 0, NULL);
	nan_sec_wpa_supplicant_send_2_of_4(NULL, NULL,
					   NULL,
					   0, NULL,
					   NULL, 0,
					   NULL);

	nan_sec_wpa_supplicant_send_4_of_4(NULL, NULL,
					   NULL,
					   0, 0,
					   NULL);

	nan_sec_wpa_send_eapol(NULL,     /*AP: KDE compose, MIC, and send*/
						   NULL, 0,
						   NULL, NULL,
						   NULL, 0,
						   0, 0, 0);

	nanSecMicCalStaSmStep(NULL);
	nanSecMicCalApSmStep(NULL);

	nanSecStaSmBufReset(NULL);
	nanSecApSmBufReset(NULL);

	nanSecGenAuthToken(NULL, 0, 0,
					   NULL, 0, NULL);
	nanSecGenM3MicMaterial(NULL, NULL, 0,
						   NULL, NULL);
}
#endif

/************************************************
 *               UT Related
 ************************************************
 */
#if (CFG_NAN_SEC_UT == 1)
uint32_t
nanSecUtStaKdeAttr(void) {
	struct wpa_sm *sm = &g_arNanWpaSm[0];
	unsigned char *dst = NULL;
	struct wpa_eapol_key *key = NULL;
	int ver = WPA_KEY_INFO_TYPE_AES_128_CMAC;
	u8 *nonce = NULL;
	u8 *wpa_ie = NULL;
	size_t wpa_ie_len = 0;
	struct wpa_ptk *ptk = NULL;

	uint32_t rStatus = WLAN_STATUS_FAILURE;

	/* Prepare settings */
	g_rNanNdpSudo[0].u1Role = NAN_NDP_RESPONDER;
	nanSecSetCipherType(0, NAN_CIPHER_SUITE_ID_NCS_SK_CCM_128);

	g_rNanNdpSudo[0].u2PublishId = 0x78;

	key = os_zalloc(sizeof(struct wpa_eapol_key));
	key->replay_counter[0] = 0x56;

	random_get_bytes(sm->snonce, WPA_NONCE_LEN);

	/* Compose KDE */
	rStatus = nan_sec_wpa_supplicant_send_2_of_4(
		sm, dst, key, ver, sm->snonce, wpa_ie, wpa_ie_len, ptk);

	/* Dump */
	if (rStatus == WLAN_STATUS_SUCCESS) {
		wpa_hexdump_dbg(MSG_INFO, "Dump STA KDE Attr",
				sm->pu1TmpKdeAttrBuf, sm->u4TmpKdeAttrLen);
	}

	return rStatus;
}

uint32_t
nanSecUtApKdeAttr(void) {
	struct wpa_authenticator *wpa_auth = &g_rNanWpaAuth;
	struct wpa_state_machine *sm = &g_arNanWpaAuthSm[0];
	int key_info = WPA_KEY_INFO_ACK | WPA_KEY_INFO_KEY_TYPE;
	u8 *key_rsc = NULL;
	u8 nonce[WPA_NONCE_LEN];
	u8 *kde = NULL;
	size_t kde_len = 0;
	int keyidx = 1;
	int encr = 0;
	int force_version = 0;

	uint32_t rStatus = WLAN_STATUS_FAILURE;

	/* Prepare settings */
	g_rNanNdpSudo[0].u1Role = NAN_NDP_INITIATOR;
	nanSecSetCipherType(0, NAN_CIPHER_SUITE_ID_NCS_SK_CCM_128);

	g_rNanNdpSudo[0].u2PublishId = 0x78;

	random_get_bytes(nonce, WPA_NONCE_LEN);

	g_arNanWpaAuthSm[0].wpa_auth = &g_rNanWpaAuth;

	/* Compose KDE */
	rStatus = nan_sec_wpa_send_eapol(wpa_auth, sm, key_info, key_rsc, nonce,
					 kde, kde_len, keyidx, encr,
					 force_version);

	if (rStatus == WLAN_STATUS_SUCCESS) {
		wpa_hexdump_dbg(MSG_INFO, "Dump AP KDE Attr",
				sm->pu1TmpKdeAttrBuf, sm->u4TmpKdeAttrLen);
	} else {
		DBGLOG(NAN, INFO, "[%s] gen KDE failed!\n", __func__);
	}
}

void
nanSecUtPbkdf256(void) {
	/* unsigned char passphrase[] = "NAN"; */
	unsigned char passphrase[] = "NAN2";
	unsigned char salt[] = { 0x00, 0x01, 0x2b, 0x9c, 0x45, 0x0f, 0x66,
				 0x71, 0x02, 0x90, 0x4c, 0x12, 0xd0, 0x01 };

	unsigned char *key = os_zalloc(32); /* key_len:32 */

	DBGLOG(NAN, INFO, "[%s] Enter, p_len:%d, s_len:%d\n", __func__,
	       sizeof(passphrase), sizeof(salt));

	dumpMemory8(passphrase, sizeof(passphrase));
	dumpMemory8(salt, sizeof(salt));

	PKCS5_PBKDF2_HMAC((unsigned char *)passphrase, sizeof(passphrase) - 1,
			  (unsigned char *)salt, sizeof(salt), 4096, 32,
			  (unsigned char *)key);

	dumpMemory8(key, 32);
}

uint32_t
nanSecUtMain(void) {
	DBGLOG(NAN, INFO, "[%s] Enter\n", __func__);

	nanSecUtStaKdeAttr();
	nanSecUtApKdeAttr();
	nanSecUtPbkdf256();

	return 0;
}
#endif
