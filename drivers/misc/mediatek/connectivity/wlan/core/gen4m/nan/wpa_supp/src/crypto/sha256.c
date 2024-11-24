/* SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause */

/*
 * Copyright (c) 2020 MediaTek Inc.
 */
/*
 * SHA-256 hash implementation and interface functions
 * Copyright (c) 2003-2012, Jouni Malinen <j@w1.fi>
 *
 * This software may be distributed under the terms of the BSD license.
 * See README for more details.
 */

#include "wpa_supp/FourWayHandShake.h"

#include "wpa_supp/src/crypto/crypto.h"
#include "wpa_supp/src/crypto/sha256.h"
#include "wpa_supp/src/utils/common.h"

/**
 * hmac_sha256_vector - HMAC-SHA256 over data vector (RFC 2104)
 * @key: Key for HMAC operations
 * @key_len: Length of the key in bytes
 * @num_elem: Number of elements in the data vector
 * @addr: Pointers to the data areas
 * @len: Lengths of the data blocks
 * @mac: Buffer for the hash (32 bytes)
 * Returns: 0 on success, -1 on failure
 */
int
hmac_sha256_vector(const u8 *key, size_t key_len, size_t num_elem,
		   const u8 *addr[], const size_t *len, u8 *mac) {
	/*unsigned char k_pad[64];*/
	/* padding - key XORd with ipad/opad zalloc for deadbeef*/
	/*unsigned char tk[32];*/
	const u8 *_addr[6];
	size_t _len[6], i;

	unsigned char *k_pad;
	unsigned char *tk;

	int ret = 0;

	k_pad = os_zalloc(64);
	tk = os_zalloc(32);

	if (num_elem > 5) {
		/*
		 * Fixed limit on the number of fragments to avoid having to
		 * allocate memory (which could fail).
		 */
		os_free(k_pad);
		os_free(tk);
		return -1;
	}

	/* if key is longer than 64 bytes reset it to key = SHA256(key) */
	if (key_len > 64) {
		if (sha256_vector(1, &key, &key_len, tk) < 0) {
			os_free(k_pad);
			os_free(tk);
			return -1;
		}
		key = tk;
		key_len = 32;
	}

	/* the HMAC_SHA256 transform looks like:
	 *
	 * SHA256(K XOR opad, SHA256(K XOR ipad, text))
	 *
	 * where K is an n byte key
	 * ipad is the byte 0x36 repeated 64 times
	 * opad is the byte 0x5c repeated 64 times
	 * and text is the data being protected
	 */

	/* start out by storing key in ipad */
	/*os_memset(k_pad, 0, sizeof(k_pad));*/
	os_memset(k_pad, 0, 64);
	os_memcpy(k_pad, key, key_len);
	/* XOR key with ipad values */
	for (i = 0; i < 64; i++)
		k_pad[i] ^= 0x36;

	/* perform inner SHA256 */
	_addr[0] = k_pad;
	_len[0] = 64;
	for (i = 0; i < num_elem; i++) {
		_addr[i + 1] = addr[i];
		_len[i + 1] = len[i];
	}
	if (sha256_vector(1 + num_elem, _addr, _len, mac) < 0) {
		os_free(k_pad);
		os_free(tk);
		return -1;
	}
	/*os_memset(k_pad, 0, sizeof(k_pad));*/
	os_memset(k_pad, 0, 64);
	os_memcpy(k_pad, key, key_len);
	/* XOR key with opad values */
	for (i = 0; i < 64; i++)
		k_pad[i] ^= 0x5c;

	/* perform outer SHA256 */
	_addr[0] = k_pad;
	_len[0] = 64;
	_addr[1] = mac;
	_len[1] = SHA256_MAC_LEN;

	ret = sha256_vector(2, _addr, _len, mac);
	os_free(k_pad);
	os_free(tk);

	return ret;

	/*return sha256_vector(2, _addr, _len, mac);*/
}

/**
 * hmac_sha256 - HMAC-SHA256 over data buffer (RFC 2104)
 * @key: Key for HMAC operations
 * @key_len: Length of the key in bytes
 * @data: Pointers to the data area
 * @data_len: Length of the data area
 * @mac: Buffer for the hash (32 bytes)
 * Returns: 0 on success, -1 on failure
 */
int
hmac_sha256(const u8 *key, size_t key_len, const u8 *data, size_t data_len,
	    u8 *mac) {
	return hmac_sha256_vector(key, key_len, 1, &data, &data_len, mac);
}

void caculate_pmkid(u8 *key, u8 *IMAC, u8 *RMAC, u8 *serviceName, u8 *pmkid)
{
	char *pmkName = "NAN PMK Name";
	struct nan_rdf_sha256_state r_SHA_256_state;
	u8  auc_tk[32];
	u8 aucServiceID[6];
	int pmkIdSrcLen = strlen(pmkName) + 6+6+6;
	u8 *pmkIdSrc = os_malloc(pmkIdSrcLen);
	int i = 0;
	int post = 0;

	for (i = 0; i < strlen(serviceName); i++) {
		if ((serviceName[i] >= 'A') && (serviceName[i] <= 'Z'))
			serviceName[i] =  serviceName[i] + 32;
	}
	nan_rdf_sha256_init(&r_SHA_256_state);
	sha256_process(&r_SHA_256_state, serviceName, strlen(serviceName));
	sha256_done(&r_SHA_256_state, auc_tk);
	os_memcpy(aucServiceID, auc_tk, 6);


	os_memcpy(pmkIdSrc, pmkName, strlen(pmkName));
	post += strlen(pmkName);
	os_memcpy(pmkIdSrc+post, IMAC, 6);
	post += 6;
	os_memcpy(pmkIdSrc+post, RMAC, 6);
	post += 6;
	os_memcpy(pmkIdSrc+post, aucServiceID, 6);
	hmac_sha256(key, 32, pmkIdSrc, pmkIdSrcLen, pmkid);
	os_free(pmkIdSrc);

}

