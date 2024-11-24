/* SPDX-License-Identifier: GPL-2.0 */

/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#ifndef _NAN_SEC_H_
#define _NAN_SEC_H_

/*******************************************************************************
 *                         C O M P I L E R   F L A G S
 *******************************************************************************
 */
#if CFG_SUPPORT_NAN

/*******************************************************************************
 *                    E X T E R N A L   R E F E R E N C E S
 *******************************************************************************
 */
#include "nan/nan_base.h"
#include "wpa_supp/src/utils/common.h"

struct wpa_key_replay_counter;
extern int wpa_replay_counter_valid(struct wpa_key_replay_counter *ctr,
				    const u8 *replay_counter);
extern void wpa_replay_counter_mark_invalid(struct wpa_key_replay_counter *ctr,
					    const u8 *replay_counter);
extern void PKCS5_PBKDF2_HMAC(unsigned char *password, size_t plen,
			      unsigned char *salt, size_t slen,
			      const unsigned long iteration_count,
			      const unsigned long key_length,
			      unsigned char *output);

/*******************************************************************************
 *                              C O N S T A N T S
 *******************************************************************************
 */
#define MAX_NDP_NUM 8 /* May integrate with NDP */
#define NAN_MAX_KEY_ID 3
#define NAN_SHA384_MAC_LEN 48
#define NAN_AUTH_TOKEN_LEN 16 /*128bit */
#define MAX_WTBL_ENTRY_NUM 128
#define CFG_NAN_SEC_UT 0
#define NCS_SK_128_MIC_LEN 16
#define NCS_SK_256_MIC_LEN 24

#define ENABLE_SEC_UT_LOG 1

enum NAN_SEC_MIC_CAL_STATE {
	NAN_SEC_MIC_CAL_IDLE = 0,
	NAN_SEC_MIC_CAL_WAIT,
	NAN_SEC_MIC_CAL_DONE,
	NAN_SEC_MIC_CAL_ERROR,
	NAN_SEC_MIC_CAL_STATE_NUM
};

/*******************************************************************************
 *                             D A T A   T Y P E S
 *******************************************************************************
 */
struct _NAN_NDP_SUDO {
	uint8_t u1Role;
	uint8_t u1WtblIdx;
	uint16_t u2PublishId;
	uint8_t au1RemoteAddr[MAC_ADDR_LEN];
} __packed;

struct _NAN_SEC_KDE_ATTR_HDR {
	uint8_t u1AttrId;
	uint16_t u2AttrLen;
	uint8_t u1PublishId;
} __packed;

struct _NAN_SEC_CSID_ATTR_HDR {
	uint8_t u1AttrId;
	uint16_t u2AttrLen;
	uint8_t u1Cap;
} __packed;

struct _NAN_SEC_CSID_ATTR_LIST {
	uint8_t u1CipherType; /* Follow WFA spec */
	uint8_t u1PublishId;
} __packed;

struct _NAN_SEC_SCID_ATTR_HDR {
	uint8_t u1AttrId;
	uint16_t u2AttrLen;
} __packed;

struct _NAN_SEC_SCID_ATTR_ENTRY {
	/* QUE_ENTRY_T rQueEntry; */
	uint16_t u2ScidLen;
	uint8_t u1ScidType;
	uint8_t u1PublishId;
} __packed;

struct _NAN_SEC_CIPHER_ENTRY {
	struct QUE_ENTRY rQueEntry;
	uint32_t u4CipherType;
	uint16_t u2PublishId;
} __packed;

/* ======For other module building pass */
struct wpa_authenticator;
struct wpa_state_machine;
struct wpa_ptk;
struct wpa_eapol_key;
struct wpa_sm;

struct _NAN_NDP_INSTANCE_T;
/*******************************************************************************
 *                            P U B L I C   D A T A
 *******************************************************************************
 */

/*******************************************************************************
 *                           P R I V A T E   D A T A
 *******************************************************************************
 */

/*******************************************************************************
 *                                 M A C R O S
 *******************************************************************************
 */

/*******************************************************************************
 *                   F U N C T I O N   D E C L A R A T I O N S
 *******************************************************************************
 */

/************************************************
 *               Export API Related
 ************************************************
 */
uint32_t nanSecGetCsidAttr(OUT uint32_t *pu4CsidAttrLen,
			   OUT uint8_t **ppu1CsidAttrBuf);
uint32_t nanSecGetNdpScidAttr(IN struct _NAN_NDP_INSTANCE_T *prNdp,
			      OUT uint32_t *pu4ScidAttrLen,
			      OUT uint8_t **ppu1ScidAttrBuf);
uint32_t nanSecGetNdpCsidAttr(IN struct _NAN_NDP_INSTANCE_T *prNdp,
			      OUT uint32_t *pu4CsidAttrLen,
			      OUT uint8_t **ppu1CsidAttrBuf);
uint32_t nanSecSetCipherType(IN struct _NAN_NDP_INSTANCE_T *prNdp,
			     IN uint32_t u4CipherType);
uint32_t nanSecSetPmk(IN struct _NAN_NDP_INSTANCE_T *prNdp,
		IN uint32_t u4PmkLen, IN uint8_t *pu1Pmk);

uint32_t nanSecNotify4wayBegin(IN struct _NAN_NDP_INSTANCE_T *prNdp);
uint32_t nanSecNotify4wayTerminate(IN struct _NAN_NDP_INSTANCE_T *prNdp);
uint32_t nanSecTxKdeAttrDone(IN struct _NAN_NDP_INSTANCE_T *prNdp,
			     IN uint8_t u1DstMsg);
uint32_t nanSecRxKdeAttr(IN struct _NAN_NDP_INSTANCE_T *prNdp,
			 IN uint8_t u1SrcMsg, IN uint32_t u4KdeAttrLen,
			 IN uint8_t *pu1KdeAttrBuf, IN uint32_t u4RxMsgLen,
			 IN uint8_t *pu1RxMsgBuf);

uint32_t nanSecNotifyMsgBodyRdy(IN struct _NAN_NDP_INSTANCE_T *prNdp,
				IN uint8_t u1SrcMsg, IN OUT uint32_t u4TxMsgLen,
				IN OUT uint8_t *pu1TxMsgBuf);

void nan_sec_wpa_supplicant_start(void);
void nan_sec_hostapd_deinit(void);
uint32_t nanSecInsertCipherList(IN uint32_t u4CipherType,
				IN uint16_t u2PublishId);
uint32_t nanSecFlushCipherList(void);
uint16_t nanSecCalKdeAttrLenFunc(struct _NAN_NDP_INSTANCE_T *prNdp);
void nanSecAppendKdeAttrFunc(struct _NAN_NDP_INSTANCE_T *prNdp,
			     struct MSDU_INFO *prMsduInfo);

struct wpa_state_machine *nanSecGetInitiatorSm(uint8_t u1Index);
struct wpa_sm *nanSecGetResponderSm(uint8_t u1Index);
uint32_t nan_sec_wpa_sm_rx_eapol(struct wpa_sm *sm, const u8 *src_addr);

void nanSecResetTk(struct STA_RECORD *prStaRec);
void nanSecInstallTk(struct _NAN_NDP_INSTANCE_T *prNdp,
		     struct STA_RECORD *prStaRec);
void nanSecUpdatePeerNDI(struct _NAN_NDP_INSTANCE_T *prNdp,
			 uint8_t *au1PeerNdiAddr);
int32_t
nanSecCompareSA(IN struct ADAPTER *prAdapter,
		IN struct _NAN_NDP_INSTANCE_T *prNdp1,
		IN struct _NAN_NDP_INSTANCE_T *prNdp2);

/************************************************
 *               NDP Sudo Related
 ************************************************
 */
uint32_t nanNdpNotifySecAttrRdy(IN uint8_t u1NdpIdx);

uint32_t nanNdpGetNdiAddr(uint8_t u1NdpIdx,
	uint8_t u1Role, uint8_t *pu1MacAddr);
uint32_t nanNdpGetPublishId(uint8_t *u1NdpIdx);
uint32_t nanNdpNotifySecStatus(uint8_t u1NdpIdx, uint8_t u1Status,
			       uint8_t u1Reason, uint8_t u1Msg);

uint8_t nanNdpGetWlanIdx(uint8_t u1NdpIdx);

/************************************************
 *               Self-Use API Related
 ************************************************
 */
uint8_t nanSecSelPtkKeyId(struct _NAN_NDP_INSTANCE_T *prNdp,
			 uint8_t *pu1PeerAddr);
uint32_t nanSecUpdatePmk(struct _NAN_NDP_INSTANCE_T *prNdp);

void nanSecUpdateAttrCmd(IN struct ADAPTER *prAdapter, uint8_t aucAttrId,
			 uint8_t *aucAttrBuf, uint16_t aucAttrLen);

/************************************************
 *               Tx Related
 ************************************************
 */
int nan_sec_wpa_eapol_key_mic(const u8 *key, size_t key_len, u32 cipher,
			      const u8 *buf, size_t len, u8 *mic);
int nan_sec_wpa_supplicant_send_2_of_4(struct wpa_sm *sm,
				       const unsigned char *dst,
				       const struct wpa_eapol_key *key, int ver,
				       const u8 *nonce, const u8 *wpa_ie,
				       size_t wpa_ie_len, struct wpa_ptk *ptk);

int nan_sec_wpa_supplicant_send_4_of_4(struct wpa_sm *sm,
				       const unsigned char *dst,
				       const struct wpa_eapol_key *key, u16 ver,
				       u16 key_info, struct wpa_ptk *ptk);

int nan_sec_wpa_send_eapol(
	struct wpa_authenticator *wpa_auth, /*AP: KDE compose, MIC, and send*/
	struct wpa_state_machine *sm, int key_info, const u8 *key_rsc,
	const u8 *nonce, const u8 *kde, size_t kde_len, int keyidx, int encr,
	int force_version);

/************************************************
 *               MIC Related
 ************************************************
 */
uint32_t nanSecMicCalStaSmStep(struct wpa_sm *sm);
uint32_t nanSecMicCalApSmStep(struct wpa_state_machine *sm);

uint32_t nanSecStaSmBufReset(struct wpa_sm *sm);
uint32_t nanSecApSmBufReset(struct wpa_state_machine *sm);

uint32_t nanSecGenAuthToken(u32 cipher, const u8 *auth_token_data,
			    size_t auth_token_data_len, u8 *auth_token);
uint32_t nanSecGenM3MicMaterial(IN uint8_t *pu1AuthTokenBuf,
				IN const u8 *pu1M3bodyBuf,
				IN uint32_t u4M3BodyLen,
				OUT uint8_t **ppu1M3MicMaterialBuf,
				OUT uint32_t *pu4M3MicMaterialLen);

/************************************************
 *               UT Related
 ************************************************
 */
uint32_t nanSecUtMain(void);
void nanSecDumpEapolKey(struct wpa_eapol_key *key);

/*******************************************************************************
 *                              F U N C T I O N S
 *******************************************************************************
 */

#endif
#endif /* _NAN_SEC_H_ */
