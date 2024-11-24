/* SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause */

/*
 * Copyright (c) 2020 MediaTek Inc.
 */
/*
 * WPA/RSN - Shared functions for supplicant and authenticator
 * Copyright (c) 2002-2015, Jouni Malinen <j@w1.fi>
 *
 * This software may be distributed under the terms of the BSD license.
 * See README for more details.
 */

#include "wpa_supp/FourWayHandShake.h"

#include "wpa_supp/src/crypto/aes_wrap.h"
#include "wpa_supp/src/crypto/crypto.h"
#include "wpa_supp/src/crypto/sha1.h"
#include "wpa_supp/src/crypto/sha256.h"
#include "wpa_supp/src/utils/common.h"
/*#include "wpa_supp/src/crypto/md5.h"*/
#include "wpa_supp/src/common/defs.h"
#include "wpa_supp/src/common/ieee802_11_defs.h"
#include "wpa_supp/src/common/wpa_common.h"

#ifdef CFG_SUPPORT_NAN
#include "nan/nan_sec.h"
#endif

static unsigned int
wpa_kck_len(int akmp) {
	if (akmp == WPA_KEY_MGMT_IEEE8021X_SUITE_B_192)
		return 24;
#if CFG_SUPPORT_NAN
	if (akmp == WPA_KEY_MGMT_PSK_SHA384)
		return 24;
#endif
	return 16;
}

static unsigned int
wpa_kek_len(int akmp) {
	if (akmp == WPA_KEY_MGMT_IEEE8021X_SUITE_B_192)
		return 32;
#if CFG_SUPPORT_NAN
	if (akmp == WPA_KEY_MGMT_PSK_SHA384)
		return 32;
#endif
	return 16;
}

unsigned int
wpa_mic_len(int akmp) {
	if (akmp == WPA_KEY_MGMT_IEEE8021X_SUITE_B_192)
		return 24;
#if CFG_SUPPORT_NAN
	if (akmp == WPA_KEY_MGMT_PSK_SHA384)
		return 24;
#endif
	return 16;
}

/**
 * wpa_eapol_key_mic - Calculate EAPOL-Key MIC
 * @key: EAPOL-Key Key Confirmation Key (KCK)
 * @key_len: KCK length in octets
 * @akmp: WPA_KEY_MGMT_* used in key derivation
 * @ver: Key descriptor version (WPA_KEY_INFO_TYPE_*)
 * @buf: Pointer to the beginning of the EAPOL header (version field)
 * @len: Length of the EAPOL frame (from EAPOL header to the end of the frame)
 * @mic: Pointer to the buffer to which the EAPOL-Key MIC is written
 * Returns: 0 on success, -1 on failure
 *
 * Calculate EAPOL-Key MIC for an EAPOL-Key packet. The EAPOL-Key MIC field has
 * to be cleared (all zeroes) when calling this function.
 *
 * Note: 'IEEE Std 802.11i-2004 - 8.5.2 EAPOL-Key frames' has an error in the
 * description of the Key MIC calculation. It includes packet data from the
 * beginning of the EAPOL-Key header, not EAPOL header. This incorrect change
 * happened during final editing of the standard and the correct behavior is
 * defined in the last draft (IEEE 802.11i/D10).
 */
#define SHA384_MAC_LEN 48

int
wpa_eapol_key_mic_wpa(const u8 *key, size_t key_len, int akmp, int ver,
		      const u8 *buf, size_t len, u8 *mic) {
	u8 hash[SHA384_MAC_LEN];

	wpa_printf(MSG_INFO, "[%s] key_len:%d, akmp:%d, ver:%d, len:%d\n",
		   __func__, key_len, akmp, ver, len);

	wpa_hexdump(MSG_DEBUG, "key", key, key_len);
	wpa_hexdump(MSG_DEBUG, "buf", buf, len);

	switch (ver) {
	case WPA_KEY_INFO_TYPE_HMAC_MD5_RC4:
		wpa_printf(MSG_INFO, "[%s] HMAC_MD5_RC4", __func__);
		/*return hmac_md5(key, key_len, buf, len, mic);*/
		return -1;
	case WPA_KEY_INFO_TYPE_HMAC_SHA1_AES:
		if (hmac_sha1(key, key_len, buf, len, hash)) {
			wpa_printf(MSG_INFO, "[%s] hmac_sha1 failed!",
				   __func__);
			return -1;
		}
		/*else*/
		wpa_printf(MSG_INFO, "[%s] hmac_sha1 success!", __func__);
		os_memcpy(mic, hash, 16); /* MD5_MAC_LEN */
		break;
#if defined(CONFIG_IEEE80211R) || defined(CONFIG_IEEE80211W)
	case WPA_KEY_INFO_TYPE_AES_128_CMAC:
		wpa_printf(MSG_INFO, "[%s] omac1_aes_128!", __func__);
		return omac1_aes_128(key, buf, len, mic);
#endif /* CONFIG_IEEE80211R || CONFIG_IEEE80211W */
	case WPA_KEY_INFO_TYPE_AKM_DEFINED:
		switch (akmp) {
		default:
			return -1;
		}
		break;
	default:
		return -1;
	}

	return 0;
}

/**
 * wpa_pmk_to_ptk - Calculate PTK from PMK, addresses, and nonces
 * @pmk: Pairwise master key
 * @pmk_len: Length of PMK
 * @label: Label to use in derivation
 * @addr1: AA or SA
 * @addr2: SA or AA
 * @nonce1: ANonce or SNonce
 * @nonce2: SNonce or ANonce
 * @ptk: Buffer for pairwise transient key
 * @akmp: Negotiated AKM
 * @cipher: Negotiated pairwise cipher
 * Returns: 0 on success, -1 on failure
 *
 * IEEE Std 802.11i-2004 - 8.5.1.2 Pairwise key hierarchy
 * PTK = PRF-X(PMK, "Pairwise key expansion",
 *             Min(AA, SA) || Max(AA, SA) ||
 *             Min(ANonce, SNonce) || Max(ANonce, SNonce))
 *
 * STK = PRF-X(SMK, "Peer key expansion",
 *             Min(MAC_I, MAC_P) || Max(MAC_I, MAC_P) ||
 *             Min(INonce, PNonce) || Max(INonce, PNonce))
 */
int
wpa_pmk_to_ptk(const u8 *pmk, size_t pmk_len, const char *label,
	       const u8 *addr1, const u8 *addr2, const u8 *nonce1,
	       const u8 *nonce2, struct wpa_ptk *ptk, int akmp, int cipher) {
	u8 data[2 * ETH_ALEN + 2 * WPA_NONCE_LEN];
	u8 tmp[WPA_KCK_MAX_LEN + WPA_KEK_MAX_LEN + WPA_TK_MAX_LEN];
	size_t ptk_len;
#if 0
	if (os_memcmp(addr1, addr2, ETH_ALEN) < 0) {
		os_memcpy(data, addr1, ETH_ALEN);
		os_memcpy(data + ETH_ALEN, addr2, ETH_ALEN);
	} else {
		os_memcpy(data, addr2, ETH_ALEN);
		os_memcpy(data + ETH_ALEN, addr1, ETH_ALEN);
	}

	if (os_memcmp(nonce1, nonce2, WPA_NONCE_LEN) < 0) {
		os_memcpy(data + 2 * ETH_ALEN, nonce1, WPA_NONCE_LEN);
		os_memcpy(data + 2 * ETH_ALEN + WPA_NONCE_LEN, nonce2,
				  WPA_NONCE_LEN);
	} else {
		os_memcpy(data + 2 * ETH_ALEN, nonce2, WPA_NONCE_LEN);
		os_memcpy(data + 2 * ETH_ALEN + WPA_NONCE_LEN, nonce1,
				  WPA_NONCE_LEN);
	}
#endif

	ptk->kck_len = wpa_kck_len(akmp);
	ptk->kek_len = wpa_kek_len(akmp);
	/*ptk->tk_len = wpa_cipher_key_len(cipher);*/
	ptk->tk_len = (cipher == NAN_CIPHER_SUITE_ID_NCS_SK_CCM_128) ? 16 : 32;
	ptk_len = ptk->kck_len + ptk->kek_len + ptk->tk_len;

#ifdef CFG_SUPPORT_NAN
	/*NAN => (IAddr1 || RAddr2 || INounce1 || RNounce2)*/

	os_memcpy(data, addr1, ETH_ALEN);
	os_memcpy(data + ETH_ALEN, addr2, ETH_ALEN);
	os_memcpy(data + 2 * ETH_ALEN, nonce1, WPA_NONCE_LEN);
	os_memcpy(data + 2 * ETH_ALEN + WPA_NONCE_LEN, nonce2, WPA_NONCE_LEN);

	wpa_printf(MSG_INFO, "[%s] Input data:\n", __func__);
	dumpMemory8(data, 2 * ETH_ALEN + 2 * WPA_NONCE_LEN);

	wpa_printf(MSG_INFO, "[%s] PMK:\n", __func__);
	dumpMemory8((uint8_t *)pmk, pmk_len);

	wpa_printf(MSG_INFO, "[%s] label: %s\n", __func__, label);

	if (cipher == NAN_CIPHER_SUITE_ID_NCS_SK_GCM_256) {
		wpa_printf(MSG_INFO, "[%s] Use sha384_prf\n", __func__);
		sha384_prf(pmk, pmk_len, label, data, sizeof(data), tmp,
			   ptk_len);

	} else {
		/* NAN_CIPHER_SUITE_ID_NCS_SK_CCM_128 */
		wpa_printf(MSG_INFO, "[%s] Use sha256_prf\n", __func__);
		sha256_prf(pmk, pmk_len, label, data, sizeof(data), tmp,
			   ptk_len);
	}

	wpa_printf(MSG_INFO, "[%s] Output data:\n", __func__);
	dumpMemory8(tmp, ptk_len);

#else
#ifdef CONFIG_SUITEB192
	if (wpa_key_mgmt_sha384(akmp))
		sha384_prf(pmk, pmk_len, label, data, sizeof(data), tmp,
			   ptk_len);
	else
#endif /* CONFIG_SUITEB192 */
#ifdef CONFIG_IEEE80211W
		if (wpa_key_mgmt_sha256(akmp)) {
		wpa_printf(MSG_INFO, "[%s] Use sha256_prf", __func__);
		sha256_prf(pmk, pmk_len, label, data, sizeof(data), tmp,
			   ptk_len);
	} else
#endif /* CONFIG_IEEE80211W */
	{
		wpa_printf(MSG_INFO, "[%s] Use sha1_prf", __func__);
		sha1_prf(pmk, pmk_len, label, data, sizeof(data), tmp, ptk_len);
	}
#endif /*CFG_SUPPORT_NAN*/

	wpa_printf(MSG_DEBUG, "WPA: PTK derivation - A1=" MACSTR " A2=" MACSTR,
		   MAC2STR(addr1), MAC2STR(addr2));
	wpa_hexdump(MSG_DEBUG, "WPA: Nonce1", nonce1, WPA_NONCE_LEN);
	wpa_hexdump(MSG_DEBUG, "WPA: Nonce2", nonce2, WPA_NONCE_LEN);
	wpa_hexdump_key(MSG_DEBUG, "WPA: PMK", pmk, pmk_len);
	wpa_hexdump_key(MSG_DEBUG, "WPA: PTK", tmp, ptk_len);

	os_memcpy(ptk->kck, tmp, ptk->kck_len);
	wpa_hexdump_key(MSG_DEBUG, "WPA: KCK", ptk->kck, ptk->kck_len);
	wpa_printf(MSG_INFO, "[%s] KCK:\n", __func__);
	dumpMemory8(ptk->kck, ptk->kck_len);

	os_memcpy(ptk->kek, tmp + ptk->kck_len, ptk->kek_len);
	wpa_hexdump_key(MSG_DEBUG, "WPA: KEK", ptk->kek, ptk->kek_len);
	wpa_printf(MSG_INFO, "[%s] KEK:\n", __func__);
	dumpMemory8(ptk->kek, ptk->kek_len);

	os_memcpy(ptk->tk, tmp + ptk->kck_len + ptk->kek_len, ptk->tk_len);
	wpa_hexdump_key(MSG_DEBUG, "WPA: TK", ptk->tk, ptk->tk_len);
	wpa_printf(MSG_INFO, "[%s] TK:\n", __func__);
	dumpMemory8(ptk->tk, ptk->tk_len);

	os_memset(tmp, 0, sizeof(tmp));
	return 0;
}

static int
rsn_selector_to_bitfield(const u8 *s) {
	if (RSN_SELECTOR_GET(s) == RSN_CIPHER_SUITE_NONE)
		return WPA_CIPHER_NONE;
	if (RSN_SELECTOR_GET(s) == RSN_CIPHER_SUITE_TKIP)
		return WPA_CIPHER_TKIP;
	if (RSN_SELECTOR_GET(s) == RSN_CIPHER_SUITE_CCMP)
		return WPA_CIPHER_CCMP;
#ifdef CONFIG_IEEE80211W
	if (RSN_SELECTOR_GET(s) == RSN_CIPHER_SUITE_AES_128_CMAC)
		return WPA_CIPHER_AES_128_CMAC;
#endif /* CONFIG_IEEE80211W */
	if (RSN_SELECTOR_GET(s) == RSN_CIPHER_SUITE_GCMP)
		return WPA_CIPHER_GCMP;
	if (RSN_SELECTOR_GET(s) == RSN_CIPHER_SUITE_CCMP_256)
		return WPA_CIPHER_CCMP_256;
	if (RSN_SELECTOR_GET(s) == RSN_CIPHER_SUITE_GCMP_256)
		return WPA_CIPHER_GCMP_256;
	if (RSN_SELECTOR_GET(s) == RSN_CIPHER_SUITE_BIP_GMAC_128)
		return WPA_CIPHER_BIP_GMAC_128;
	if (RSN_SELECTOR_GET(s) == RSN_CIPHER_SUITE_BIP_GMAC_256)
		return WPA_CIPHER_BIP_GMAC_256;
	if (RSN_SELECTOR_GET(s) == RSN_CIPHER_SUITE_BIP_CMAC_256)
		return WPA_CIPHER_BIP_CMAC_256;
	if (RSN_SELECTOR_GET(s) == RSN_CIPHER_SUITE_NO_GROUP_ADDRESSED)
		return WPA_CIPHER_GTK_NOT_USED;
	return 0;
}

static int
rsn_key_mgmt_to_bitfield(const u8 *s) {
	if (RSN_SELECTOR_GET(s) == RSN_AUTH_KEY_MGMT_UNSPEC_802_1X)
		return WPA_KEY_MGMT_IEEE8021X;
	if (RSN_SELECTOR_GET(s) == RSN_AUTH_KEY_MGMT_PSK_OVER_802_1X)
		return WPA_KEY_MGMT_PSK;
#ifdef CONFIG_IEEE80211W
	if (RSN_SELECTOR_GET(s) == RSN_AUTH_KEY_MGMT_802_1X_SHA256)
		return WPA_KEY_MGMT_IEEE8021X_SHA256;
	if (RSN_SELECTOR_GET(s) == RSN_AUTH_KEY_MGMT_PSK_SHA256)
		return WPA_KEY_MGMT_PSK_SHA256;
#endif /* CONFIG_IEEE80211W */
	if (RSN_SELECTOR_GET(s) == RSN_AUTH_KEY_MGMT_802_1X_SUITE_B)
		return WPA_KEY_MGMT_IEEE8021X_SUITE_B;
	if (RSN_SELECTOR_GET(s) == RSN_AUTH_KEY_MGMT_802_1X_SUITE_B_192)
		return WPA_KEY_MGMT_IEEE8021X_SUITE_B_192;
	if (RSN_SELECTOR_GET(s) == RSN_AUTH_KEY_MGMT_OSEN)
		return WPA_KEY_MGMT_OSEN;
	return 0;
}

int
wpa_cipher_valid_group(int cipher) {
	return wpa_cipher_valid_pairwise(cipher) ||
	       cipher == WPA_CIPHER_GTK_NOT_USED;
}

#ifdef CONFIG_IEEE80211W
int
wpa_cipher_valid_mgmt_group(int cipher) {
	return cipher == WPA_CIPHER_AES_128_CMAC ||
	       cipher == WPA_CIPHER_BIP_GMAC_128 ||
	       cipher == WPA_CIPHER_BIP_GMAC_256 ||
	       cipher == WPA_CIPHER_BIP_CMAC_256;
}
#endif /* CONFIG_IEEE80211W */

/**
 * wpa_parse_wpa_ie_rsn - Parse RSN IE
 * @rsn_ie: Buffer containing RSN IE
 * @rsn_ie_len: RSN IE buffer length (including IE number and length octets)
 * @data: Pointer to structure that will be filled in with parsed data
 * Returns: 0 on success, <0 on failure
 */
int
wpa_parse_wpa_ie_rsn(const u8 *rsn_ie, size_t rsn_ie_len,
		     struct wpa_ie_data *data) {
	const u8 *pos;
	int left;
	int i, count;

	os_memset(data, 0, sizeof(*data));
	data->proto = WPA_PROTO_RSN;
	data->pairwise_cipher = WPA_CIPHER_CCMP;
	data->group_cipher = WPA_CIPHER_CCMP;
	data->key_mgmt = WPA_KEY_MGMT_IEEE8021X;
	data->capabilities = 0;
	data->pmkid = NULL;
	data->num_pmkid = 0;
#ifdef CONFIG_IEEE80211W
	data->mgmt_group_cipher = WPA_CIPHER_AES_128_CMAC;
#else  /* CONFIG_IEEE80211W */
	data->mgmt_group_cipher = 0;
#endif /* CONFIG_IEEE80211W */

	if (rsn_ie_len == 0) {
		/* No RSN IE - fail silently */
		return -1;
	}

	if (rsn_ie_len < sizeof(struct rsn_ie_hdr)) {
		wpa_printf(MSG_DEBUG, "%s: ie len too short %lu", __func__,
			   (unsigned long)rsn_ie_len);
		return -1;
	}

	if (rsn_ie_len >= 6 && rsn_ie[1] >= 4 && rsn_ie[1] == rsn_ie_len - 2 &&
	    WPA_GET_BE32(&rsn_ie[2]) == OSEN_IE_VENDOR_TYPE) {
		pos = rsn_ie + 6;
		left = rsn_ie_len - 6;

		data->proto = WPA_PROTO_OSEN;
	} else {
		const struct rsn_ie_hdr *hdr;

		hdr = (const struct rsn_ie_hdr *)rsn_ie;

		if (hdr->elem_id != WLAN_EID_RSN ||
		    hdr->len != rsn_ie_len - 2 ||
		    WPA_GET_LE16(hdr->version) != RSN_VERSION) {
			wpa_printf(MSG_DEBUG,
				   "%s: malformed ie or unknown version",
				   __func__);
			return -2;
		}

		pos = (const u8 *)(hdr + 1);
		left = rsn_ie_len - sizeof(*hdr);
	}

	if (left >= RSN_SELECTOR_LEN) {
		data->group_cipher = rsn_selector_to_bitfield(pos);
		if (!wpa_cipher_valid_group(data->group_cipher)) {
			wpa_printf(MSG_DEBUG, "%s: invalid group cipher 0x%x",
				   __func__, data->group_cipher);
			return -1;
		}
		pos += RSN_SELECTOR_LEN;
		left -= RSN_SELECTOR_LEN;
	} else if (left > 0) {
		wpa_printf(MSG_DEBUG, "%s: ie length mismatch, %u too much",
			   __func__, left);
		return -3;
	}

	if (left >= 2) {
		data->pairwise_cipher = 0;
		count = WPA_GET_LE16(pos);
		pos += 2;
		left -= 2;
		if (count == 0 || count > left / RSN_SELECTOR_LEN) {
			wpa_printf(
				MSG_DEBUG,
				"%s: ie count botch (pairwise), count %u left %u",
				__func__, count, left);
			return -4;
		}
		for (i = 0; i < count; i++) {
			data->pairwise_cipher |= rsn_selector_to_bitfield(pos);
			pos += RSN_SELECTOR_LEN;
			left -= RSN_SELECTOR_LEN;
		}
#ifdef CONFIG_IEEE80211W
		if (data->pairwise_cipher & WPA_CIPHER_AES_128_CMAC) {
			wpa_printf(MSG_DEBUG,
				   "%s: AES-128-CMAC used as pairwise cipher",
				   __func__);
			return -1;
		}
#endif /* CONFIG_IEEE80211W */
	} else if (left == 1) {
		wpa_printf(MSG_DEBUG, "%s: ie too short (for key mgmt)",
			   __func__);
		return -5;
	}

	if (left >= 2) {
		data->key_mgmt = 0;
		count = WPA_GET_LE16(pos);
		pos += 2;
		left -= 2;
		if (count == 0 || count > left / RSN_SELECTOR_LEN) {
			wpa_printf(
				MSG_DEBUG,
				"%s: ie count botch (key mgmt), count %u left %u",
				__func__, count, left);
			return -6;
		}
		for (i = 0; i < count; i++) {
			data->key_mgmt |= rsn_key_mgmt_to_bitfield(pos);
			pos += RSN_SELECTOR_LEN;
			left -= RSN_SELECTOR_LEN;
		}
	} else if (left == 1) {
		wpa_printf(MSG_DEBUG, "%s: ie too short (for capabilities)",
			   __func__);
		return -7;
	}

	if (left >= 2) {
		data->capabilities = WPA_GET_LE16(pos);
		pos += 2;
		left -= 2;
	}

	if (left >= 2) {
		u16 num_pmkid = WPA_GET_LE16(pos);

		pos += 2;
		left -= 2;
		if (num_pmkid > (unsigned int)left / PMKID_LEN) {
			wpa_printf(MSG_DEBUG,
				   "%s: PMKID underflow (num_pmkid=%u left=%d)",
				   __func__, num_pmkid, left);
			data->num_pmkid = 0;
			return -9;
		} /*else */
		data->num_pmkid = num_pmkid;
		data->pmkid = pos;
		pos += data->num_pmkid * PMKID_LEN;
		left -= data->num_pmkid * PMKID_LEN;
	}

#ifdef CONFIG_IEEE80211W
	if (left >= 4) {
		data->mgmt_group_cipher = rsn_selector_to_bitfield(pos);
		if (!wpa_cipher_valid_mgmt_group(data->mgmt_group_cipher)) {
			wpa_printf(
				MSG_DEBUG,
				"%s: Unsupported management group cipher 0x%x",
				__func__, data->mgmt_group_cipher);
			return -10;
		}
		pos += RSN_SELECTOR_LEN;
		left -= RSN_SELECTOR_LEN;
	}
#endif /* CONFIG_IEEE80211W */

	if (left > 0) {
		wpa_hexdump(MSG_DEBUG,
			    "wpa_parse_wpa_ie_rsn: ignore trailing bytes", pos,
			    left);
	}

	return 0;
}

static int
wpa_selector_to_bitfield(const u8 *s) {
	if (RSN_SELECTOR_GET(s) == WPA_CIPHER_SUITE_NONE)
		return WPA_CIPHER_NONE;
	if (RSN_SELECTOR_GET(s) == WPA_CIPHER_SUITE_TKIP)
		return WPA_CIPHER_TKIP;
	if (RSN_SELECTOR_GET(s) == WPA_CIPHER_SUITE_CCMP)
		return WPA_CIPHER_CCMP;
	return 0;
}

static int
wpa_key_mgmt_to_bitfield(const u8 *s) {
	if (RSN_SELECTOR_GET(s) == WPA_AUTH_KEY_MGMT_UNSPEC_802_1X)
		return WPA_KEY_MGMT_IEEE8021X;
	if (RSN_SELECTOR_GET(s) == WPA_AUTH_KEY_MGMT_PSK_OVER_802_1X)
		return WPA_KEY_MGMT_PSK;
	if (RSN_SELECTOR_GET(s) == WPA_AUTH_KEY_MGMT_NONE)
		return WPA_KEY_MGMT_WPA_NONE;
	return 0;
}

int
wpa_parse_wpa_ie_wpa(const u8 *wpa_ie, size_t wpa_ie_len,
		     struct wpa_ie_data *data) {
	const struct wpa_ie_hdr *hdr;
	const u8 *pos;
	int left;
	int i, count;

	os_memset(data, 0, sizeof(*data));
	data->proto = WPA_PROTO_WPA;
	data->pairwise_cipher = WPA_CIPHER_TKIP;
	data->group_cipher = WPA_CIPHER_TKIP;
	data->key_mgmt = WPA_KEY_MGMT_IEEE8021X;
	data->capabilities = 0;
	data->pmkid = NULL;
	data->num_pmkid = 0;
	data->mgmt_group_cipher = 0;

	if (wpa_ie_len < sizeof(struct wpa_ie_hdr)) {
		wpa_printf(MSG_DEBUG, "%s: ie len too short %lu", __func__,
			   (unsigned long)wpa_ie_len);
		return -1;
	}

	hdr = (const struct wpa_ie_hdr *)wpa_ie;

	if (hdr->elem_id != WLAN_EID_VENDOR_SPECIFIC ||
	    hdr->len != wpa_ie_len - 2 ||
	    RSN_SELECTOR_GET(hdr->oui) != WPA_OUI_TYPE ||
	    WPA_GET_LE16(hdr->version) != WPA_VERSION) {
		wpa_printf(MSG_DEBUG, "%s: malformed ie or unknown version",
			   __func__);
		return -2;
	}

	pos = (const u8 *)(hdr + 1);
	left = wpa_ie_len - sizeof(*hdr);

	if (left >= WPA_SELECTOR_LEN) {
		data->group_cipher = wpa_selector_to_bitfield(pos);
		pos += WPA_SELECTOR_LEN;
		left -= WPA_SELECTOR_LEN;
	} else if (left > 0) {
		wpa_printf(MSG_DEBUG, "%s: ie length mismatch, %u too much",
			   __func__, left);
		return -3;
	}

	if (left >= 2) {
		data->pairwise_cipher = 0;
		count = WPA_GET_LE16(pos);
		pos += 2;
		left -= 2;
		if (count == 0 || count > left / WPA_SELECTOR_LEN) {
			wpa_printf(
				MSG_DEBUG,
				"%s: ie count botch (pairwise), count %u left %u",
				__func__, count, left);
			return -4;
		}
		for (i = 0; i < count; i++) {
			data->pairwise_cipher |= wpa_selector_to_bitfield(pos);
			pos += WPA_SELECTOR_LEN;
			left -= WPA_SELECTOR_LEN;
		}
	} else if (left == 1) {
		wpa_printf(MSG_DEBUG, "%s: ie too short (for key mgmt)",
			   __func__);
		return -5;
	}

	if (left >= 2) {
		data->key_mgmt = 0;
		count = WPA_GET_LE16(pos);
		pos += 2;
		left -= 2;
		if (count == 0 || count > left / WPA_SELECTOR_LEN) {
			wpa_printf(
				MSG_DEBUG,
				"%s: ie count botch (key mgmt), count %u left %u",
				__func__, count, left);
			return -6;
		}
		for (i = 0; i < count; i++) {
			data->key_mgmt |= wpa_key_mgmt_to_bitfield(pos);
			pos += WPA_SELECTOR_LEN;
			left -= WPA_SELECTOR_LEN;
		}
	} else if (left == 1) {
		wpa_printf(MSG_DEBUG, "%s: ie too short (for capabilities)",
			   __func__);
		return -7;
	}

	if (left >= 2) {
		data->capabilities = WPA_GET_LE16(pos);
		pos += 2;
		left -= 2;
	}

	if (left > 0) {
		wpa_hexdump(MSG_DEBUG,
			    "wpa_parse_wpa_ie_wpa: ignore trailing bytes", pos,
			    left);
	}

	return 0;
}

/**
 * rsn_pmkid - Calculate PMK identifier
 * @pmk: Pairwise master key
 * @pmk_len: Length of pmk in bytes
 * @aa: Authenticator address
 * @spa: Supplicant address
 * @pmkid: Buffer for PMKID
 * @use_sha256: Whether to use SHA256-based KDF
 *
 * IEEE Std 802.11i-2004 - 8.5.1.2 Pairwise key hierarchy
 * PMKID = HMAC-SHA1-128(PMK, "PMK Name" || AA || SPA)
 */
void
rsn_pmkid(const u8 *pmk, size_t pmk_len, const u8 *aa, const u8 *spa, u8 *pmkid,
	  int use_sha256) {
	char *title = "PMK Name";
	const u8 *addr[3];
	const size_t len[3] = { 8, ETH_ALEN, ETH_ALEN };
	unsigned char hash[SHA256_MAC_LEN];

	addr[0] = (u8 *)title;
	addr[1] = aa;
	addr[2] = spa;

#ifdef CONFIG_IEEE80211W
	if (use_sha256)
		hmac_sha256_vector(pmk, pmk_len, 3, addr, len, hash);
	else
#endif /* CONFIG_IEEE80211W */
		hmac_sha1_vector(pmk, pmk_len, 3, addr, len, hash);
	os_memcpy(pmkid, hash, PMKID_LEN);
}

#ifdef CONFIG_SUITEB
/**
 * rsn_pmkid_suite_b - Calculate PMK identifier for Suite B AKM
 * @kck: Key confirmation key
 * @kck_len: Length of kck in bytes
 * @aa: Authenticator address
 * @spa: Supplicant address
 * @pmkid: Buffer for PMKID
 * Returns: 0 on success, -1 on failure
 *
 * IEEE Std 802.11ac-2013 - 11.6.1.3 Pairwise key hierarchy
 * PMKID = Truncate(HMAC-SHA-256(KCK, "PMK Name" || AA || SPA))
 */
int
rsn_pmkid_suite_b(const u8 *kck, size_t kck_len, const u8 *aa, const u8 *spa,
		  u8 *pmkid) {
	char *title = "PMK Name";
	const u8 *addr[3];
	const size_t len[3] = { 8, ETH_ALEN, ETH_ALEN };
	unsigned char hash[SHA256_MAC_LEN];

	addr[0] = (u8 *)title;
	addr[1] = aa;
	addr[2] = spa;

	if (hmac_sha256_vector(kck, kck_len, 3, addr, len, hash) < 0)
		return -1;
	os_memcpy(pmkid, hash, PMKID_LEN);
	return 0;
}
#endif /* CONFIG_SUITEB */

#ifdef CONFIG_SUITEB192
/**
 * rsn_pmkid_suite_b_192 - Calculate PMK identifier for Suite B AKM
 * @kck: Key confirmation key
 * @kck_len: Length of kck in bytes
 * @aa: Authenticator address
 * @spa: Supplicant address
 * @pmkid: Buffer for PMKID
 * Returns: 0 on success, -1 on failure
 *
 * IEEE Std 802.11ac-2013 - 11.6.1.3 Pairwise key hierarchy
 * PMKID = Truncate(HMAC-SHA-384(KCK, "PMK Name" || AA || SPA))
 */
int
rsn_pmkid_suite_b_192(const u8 *kck, size_t kck_len, const u8 *aa,
		      const u8 *spa, u8 *pmkid) {
	char *title = "PMK Name";
	const u8 *addr[3];
	const size_t len[3] = { 8, ETH_ALEN, ETH_ALEN };
	unsigned char hash[SHA384_MAC_LEN];

	addr[0] = (u8 *)title;
	addr[1] = aa;
	addr[2] = spa;

	if (hmac_sha384_vector(kck, kck_len, 3, addr, len, hash) < 0)
		return -1;
	os_memcpy(pmkid, hash, PMKID_LEN);
	return 0;
}
#endif /* CONFIG_SUITEB192 */

/**
 * wpa_cipher_txt - Convert cipher suite to a text string
 * @cipher: Cipher suite (WPA_CIPHER_* enum)
 * Returns: Pointer to a text string of the cipher suite name
 */
const char *
wpa_cipher_txt(int cipher) {
	switch (cipher) {
	case WPA_CIPHER_NONE:
		return "NONE";
	case WPA_CIPHER_WEP40:
		return "WEP-40";
	case WPA_CIPHER_WEP104:
		return "WEP-104";
	case WPA_CIPHER_TKIP:
		return "TKIP";
	case WPA_CIPHER_CCMP:
		return "CCMP";
	case WPA_CIPHER_CCMP | WPA_CIPHER_TKIP:
		return "CCMP+TKIP";
	case WPA_CIPHER_GCMP:
		return "GCMP";
	case WPA_CIPHER_GCMP_256:
		return "GCMP-256";
	case WPA_CIPHER_CCMP_256:
		return "CCMP-256";
	case WPA_CIPHER_GTK_NOT_USED:
		return "GTK_NOT_USED";
	default:
		return "UNKNOWN";
	}
}

/**
 * wpa_key_mgmt_txt - Convert key management suite to a text string
 * @key_mgmt: Key management suite (WPA_KEY_MGMT_* enum)
 * @proto: WPA/WPA2 version (WPA_PROTO_*)
 * Returns: Pointer to a text string of the key management suite name
 */
const char *
wpa_key_mgmt_txt(int key_mgmt, int proto) {
	switch (key_mgmt) {
	case WPA_KEY_MGMT_IEEE8021X:
		if (proto == (WPA_PROTO_RSN | WPA_PROTO_WPA))
			return "WPA2+WPA/IEEE 802.1X/EAP";
		return proto == WPA_PROTO_RSN ? "WPA2/IEEE 802.1X/EAP"
					      : "WPA/IEEE 802.1X/EAP";
	case WPA_KEY_MGMT_PSK:
		if (proto == (WPA_PROTO_RSN | WPA_PROTO_WPA))
			return "WPA2-PSK+WPA-PSK";
		return proto == WPA_PROTO_RSN ? "WPA2-PSK" : "WPA-PSK";
	case WPA_KEY_MGMT_NONE:
		return "NONE";
	case WPA_KEY_MGMT_IEEE8021X_NO_WPA:
		return "IEEE 802.1X (no WPA)";
#ifdef CONFIG_IEEE80211W
	case WPA_KEY_MGMT_IEEE8021X_SHA256:
		return "WPA2-EAP-SHA256";
	case WPA_KEY_MGMT_PSK_SHA256:
		return "WPA2-PSK-SHA256";
#endif /* CONFIG_IEEE80211W */
	case WPA_KEY_MGMT_WPS:
		return "WPS";
	case WPA_KEY_MGMT_SAE:
		return "SAE";
	case WPA_KEY_MGMT_FT_SAE:
		return "FT-SAE";
	case WPA_KEY_MGMT_OSEN:
		return "OSEN";
	case WPA_KEY_MGMT_IEEE8021X_SUITE_B:
		return "WPA2-EAP-SUITE-B";
	case WPA_KEY_MGMT_IEEE8021X_SUITE_B_192:
		return "WPA2-EAP-SUITE-B-192";
	default:
		return "UNKNOWN";
	}
}

u32
wpa_akm_to_suite(int akm) {
	wpa_printf(MSG_DEBUG, "[WPA][%s]", __func__);

	if (akm & WPA_KEY_MGMT_FT_IEEE8021X)
		return WLAN_AKM_SUITE_FT_8021X;
	if (akm & WPA_KEY_MGMT_FT_PSK)
		return WLAN_AKM_SUITE_FT_PSK;
	if (akm & WPA_KEY_MGMT_IEEE8021X)
		return WLAN_AKM_SUITE_8021X;
	if (akm & WPA_KEY_MGMT_IEEE8021X_SHA256)
		return WLAN_AKM_SUITE_8021X_SHA256;
	if (akm & WPA_KEY_MGMT_IEEE8021X)
		return WLAN_AKM_SUITE_8021X;
	if (akm & WPA_KEY_MGMT_PSK_SHA256)
		return WLAN_AKM_SUITE_PSK_SHA256;
	if (akm & WPA_KEY_MGMT_PSK)
		return WLAN_AKM_SUITE_PSK;
	if (akm & WPA_KEY_MGMT_CCKM)
		return WLAN_AKM_SUITE_CCKM;
	if (akm & WPA_KEY_MGMT_OSEN)
		return WLAN_AKM_SUITE_OSEN;
	if (akm & WPA_KEY_MGMT_IEEE8021X_SUITE_B)
		return WLAN_AKM_SUITE_8021X_SUITE_B;
	if (akm & WPA_KEY_MGMT_IEEE8021X_SUITE_B_192)
		return WLAN_AKM_SUITE_8021X_SUITE_B_192;
	return 0;
}

int
wpa_compare_rsn_ie(int ft_initial_assoc, const u8 *ie1, size_t ie1len,
		   const u8 *ie2, size_t ie2len) {
	if (ie1 == NULL || ie2 == NULL)
		return -1;

	if (ie1len == ie2len && os_memcmp(ie1, ie2, ie1len) == 0)
		return 0; /* identical IEs */

	return -1;
}

int
wpa_cipher_key_len(int cipher) {
	switch (cipher) {
	case WPA_CIPHER_CCMP_256:
	case WPA_CIPHER_GCMP_256:
	case WPA_CIPHER_BIP_GMAC_256:
	case WPA_CIPHER_BIP_CMAC_256:
		return 32;
	case WPA_CIPHER_CCMP:
	case WPA_CIPHER_GCMP:
	case WPA_CIPHER_AES_128_CMAC:
	case WPA_CIPHER_BIP_GMAC_128:
		return 16;
	case WPA_CIPHER_TKIP:
		return 32;
	}

	return 0;
}

int
wpa_cipher_rsc_len(int cipher) {
	switch (cipher) {
	case WPA_CIPHER_CCMP_256:
	case WPA_CIPHER_GCMP_256:
	case WPA_CIPHER_CCMP:
	case WPA_CIPHER_GCMP:
	case WPA_CIPHER_TKIP:
		return 6;
	}

	return 0;
}

int
wpa_cipher_to_alg(int cipher) {
	switch (cipher) {
	case WPA_CIPHER_CCMP_256:
		return WPA_ALG_CCMP_256;
	case WPA_CIPHER_GCMP_256:
		return WPA_ALG_GCMP_256;
	case WPA_CIPHER_CCMP:
		return WPA_ALG_CCMP;
	case WPA_CIPHER_GCMP:
		return WPA_ALG_GCMP;
	case WPA_CIPHER_TKIP:
		return WPA_ALG_TKIP;
	case WPA_CIPHER_AES_128_CMAC:
		return WPA_ALG_IGTK;
	case WPA_CIPHER_BIP_GMAC_128:
		return WPA_ALG_BIP_GMAC_128;
	case WPA_CIPHER_BIP_GMAC_256:
		return WPA_ALG_BIP_GMAC_256;
	case WPA_CIPHER_BIP_CMAC_256:
		return WPA_ALG_BIP_CMAC_256;
	}
	return WPA_ALG_NONE;
}

int
wpa_cipher_valid_pairwise(int cipher) {
	return cipher == WPA_CIPHER_CCMP_256 || cipher == WPA_CIPHER_GCMP_256 ||
	       cipher == WPA_CIPHER_CCMP || cipher == WPA_CIPHER_GCMP ||
	       cipher == WPA_CIPHER_TKIP;
}

u32
wpa_cipher_to_suite(int proto, int cipher) {
	if (cipher & WPA_CIPHER_CCMP_256)
		return RSN_CIPHER_SUITE_CCMP_256;
	if (cipher & WPA_CIPHER_GCMP_256)
		return RSN_CIPHER_SUITE_GCMP_256;
	if (cipher & WPA_CIPHER_CCMP)
		return (proto == WPA_PROTO_RSN ? RSN_CIPHER_SUITE_CCMP
					       : WPA_CIPHER_SUITE_CCMP);
	if (cipher & WPA_CIPHER_GCMP)
		return RSN_CIPHER_SUITE_GCMP;
	if (cipher & WPA_CIPHER_TKIP)
		return (proto == WPA_PROTO_RSN ? RSN_CIPHER_SUITE_TKIP
					       : WPA_CIPHER_SUITE_TKIP);
	if (cipher & WPA_CIPHER_NONE)
		return (proto == WPA_PROTO_RSN ? RSN_CIPHER_SUITE_NONE
					       : WPA_CIPHER_SUITE_NONE);
	if (cipher & WPA_CIPHER_GTK_NOT_USED)
		return RSN_CIPHER_SUITE_NO_GROUP_ADDRESSED;
	if (cipher & WPA_CIPHER_AES_128_CMAC)
		return RSN_CIPHER_SUITE_AES_128_CMAC;
	if (cipher & WPA_CIPHER_BIP_GMAC_128)
		return RSN_CIPHER_SUITE_BIP_GMAC_128;
	if (cipher & WPA_CIPHER_BIP_GMAC_256)
		return RSN_CIPHER_SUITE_BIP_GMAC_256;
	if (cipher & WPA_CIPHER_BIP_CMAC_256)
		return RSN_CIPHER_SUITE_BIP_CMAC_256;
	return 0;
}

int
rsn_cipher_put_suites(u8 *start, int ciphers) {
	u8 *pos = start;

	if (ciphers & WPA_CIPHER_CCMP_256) {
		RSN_SELECTOR_PUT(pos, RSN_CIPHER_SUITE_CCMP_256);
		pos += RSN_SELECTOR_LEN;
	}
	if (ciphers & WPA_CIPHER_GCMP_256) {
		RSN_SELECTOR_PUT(pos, RSN_CIPHER_SUITE_GCMP_256);
		pos += RSN_SELECTOR_LEN;
	}
	if (ciphers & WPA_CIPHER_CCMP) {
		RSN_SELECTOR_PUT(pos, RSN_CIPHER_SUITE_CCMP);
		pos += RSN_SELECTOR_LEN;
	}
	if (ciphers & WPA_CIPHER_GCMP) {
		RSN_SELECTOR_PUT(pos, RSN_CIPHER_SUITE_GCMP);
		pos += RSN_SELECTOR_LEN;
	}
	if (ciphers & WPA_CIPHER_TKIP) {
		RSN_SELECTOR_PUT(pos, RSN_CIPHER_SUITE_TKIP);
		pos += RSN_SELECTOR_LEN;
	}
	if (ciphers & WPA_CIPHER_NONE) {
		RSN_SELECTOR_PUT(pos, RSN_CIPHER_SUITE_NONE);
		pos += RSN_SELECTOR_LEN;
	}

	return (pos - start) / RSN_SELECTOR_LEN;
}

int
wpa_cipher_put_suites(u8 *start, int ciphers) {
	u8 *pos = start;

	if (ciphers & WPA_CIPHER_CCMP) {
		RSN_SELECTOR_PUT(pos, WPA_CIPHER_SUITE_CCMP);
		pos += WPA_SELECTOR_LEN;
	}
	if (ciphers & WPA_CIPHER_TKIP) {
		RSN_SELECTOR_PUT(pos, WPA_CIPHER_SUITE_TKIP);
		pos += WPA_SELECTOR_LEN;
	}
	if (ciphers & WPA_CIPHER_NONE) {
		RSN_SELECTOR_PUT(pos, WPA_CIPHER_SUITE_NONE);
		pos += WPA_SELECTOR_LEN;
	}

	return (pos - start) / RSN_SELECTOR_LEN;
}

int
wpa_pick_pairwise_cipher(int ciphers, int none_allowed) {
	if (ciphers & WPA_CIPHER_CCMP_256)
		return WPA_CIPHER_CCMP_256;
	if (ciphers & WPA_CIPHER_GCMP_256)
		return WPA_CIPHER_GCMP_256;
	if (ciphers & WPA_CIPHER_CCMP)
		return WPA_CIPHER_CCMP;
	if (ciphers & WPA_CIPHER_GCMP)
		return WPA_CIPHER_GCMP;
	if (ciphers & WPA_CIPHER_TKIP)
		return WPA_CIPHER_TKIP;
	if (none_allowed && (ciphers & WPA_CIPHER_NONE))
		return WPA_CIPHER_NONE;
	return -1;
}

int
wpa_pick_group_cipher(int ciphers) {
	if (ciphers & WPA_CIPHER_CCMP_256)
		return WPA_CIPHER_CCMP_256;
	if (ciphers & WPA_CIPHER_GCMP_256)
		return WPA_CIPHER_GCMP_256;
	if (ciphers & WPA_CIPHER_CCMP)
		return WPA_CIPHER_CCMP;
	if (ciphers & WPA_CIPHER_GCMP)
		return WPA_CIPHER_GCMP;
	if (ciphers & WPA_CIPHER_GTK_NOT_USED)
		return WPA_CIPHER_GTK_NOT_USED;
	if (ciphers & WPA_CIPHER_TKIP)
		return WPA_CIPHER_TKIP;
	return -1;
}

int
wpa_parse_cipher(const char *value) {
	int val = 0, last;
	char *start, *end, *buf;

	buf = os_strdup(value);
	if (buf == NULL)
		return -1;
	start = buf;

	while (*start != '\0') {
		while (*start == ' ' || *start == '\t')
			start++;
		if (*start == '\0')
			break;
		end = start;
		while (*end != ' ' && *end != '\t' && *end != '\0')
			end++;
		last = *end == '\0';
		*end = '\0';
		if (os_strcmp(start, "CCMP-256") == 0)
			val |= WPA_CIPHER_CCMP_256;
		else if (os_strcmp(start, "GCMP-256") == 0)
			val |= WPA_CIPHER_GCMP_256;
		else if (os_strcmp(start, "CCMP") == 0)
			val |= WPA_CIPHER_CCMP;
		else if (os_strcmp(start, "GCMP") == 0)
			val |= WPA_CIPHER_GCMP;
		else if (os_strcmp(start, "TKIP") == 0)
			val |= WPA_CIPHER_TKIP;
		else if (os_strcmp(start, "WEP104") == 0)
			val |= WPA_CIPHER_WEP104;
		else if (os_strcmp(start, "WEP40") == 0)
			val |= WPA_CIPHER_WEP40;
		else if (os_strcmp(start, "NONE") == 0)
			val |= WPA_CIPHER_NONE;
		else if (os_strcmp(start, "GTK_NOT_USED") == 0)
			val |= WPA_CIPHER_GTK_NOT_USED;
		else {
			os_free(buf);
			return -1;
		}

		if (last)
			break;
		start = end + 1;
	}
	os_free(buf);

	return val;
}

int
wpa_write_ciphers(char *start, char *end, int ciphers, const char *delim) {
	char *pos = start;
	int ret;

	if (ciphers & WPA_CIPHER_CCMP_256) {
		ret = os_snprintf(pos, end - pos, "%sCCMP-256",
				  pos == start ? "" : delim);
		if (os_snprintf_error(end - pos, ret))
			return -1;
		pos += ret;
	}
	if (ciphers & WPA_CIPHER_GCMP_256) {
		ret = os_snprintf(pos, end - pos, "%sGCMP-256",
				  pos == start ? "" : delim);
		if (os_snprintf_error(end - pos, ret))
			return -1;
		pos += ret;
	}
	if (ciphers & WPA_CIPHER_CCMP) {
		ret = os_snprintf(pos, end - pos, "%sCCMP",
				  pos == start ? "" : delim);
		if (os_snprintf_error(end - pos, ret))
			return -1;
		pos += ret;
	}
	if (ciphers & WPA_CIPHER_GCMP) {
		ret = os_snprintf(pos, end - pos, "%sGCMP",
				  pos == start ? "" : delim);
		if (os_snprintf_error(end - pos, ret))
			return -1;
		pos += ret;
	}
	if (ciphers & WPA_CIPHER_TKIP) {
		ret = os_snprintf(pos, end - pos, "%sTKIP",
				  pos == start ? "" : delim);
		if (os_snprintf_error(end - pos, ret))
			return -1;
		pos += ret;
	}
	if (ciphers & WPA_CIPHER_NONE) {
		ret = os_snprintf(pos, end - pos, "%sNONE",
				  pos == start ? "" : delim);
		if (os_snprintf_error(end - pos, ret))
			return -1;
		pos += ret;
	}

	return pos - start;
}

int
wpa_select_ap_group_cipher(int wpa, int wpa_pairwise, int rsn_pairwise) {
	int pairwise = 0;

	/* Select group cipher based on the enabled pairwise cipher suites */
	if (wpa & 1)
		pairwise |= wpa_pairwise;
	if (wpa & 2)
		pairwise |= rsn_pairwise;

	if (pairwise & WPA_CIPHER_TKIP)
		return WPA_CIPHER_TKIP;
	if ((pairwise & (WPA_CIPHER_CCMP | WPA_CIPHER_GCMP)) == WPA_CIPHER_GCMP)
		return WPA_CIPHER_GCMP;
	if ((pairwise & (WPA_CIPHER_GCMP_256 | WPA_CIPHER_CCMP |
			 WPA_CIPHER_GCMP)) == WPA_CIPHER_GCMP_256)
		return WPA_CIPHER_GCMP_256;
	if ((pairwise & (WPA_CIPHER_CCMP_256 | WPA_CIPHER_CCMP |
			 WPA_CIPHER_GCMP)) == WPA_CIPHER_CCMP_256)
		return WPA_CIPHER_CCMP_256;
	return WPA_CIPHER_CCMP;
}
