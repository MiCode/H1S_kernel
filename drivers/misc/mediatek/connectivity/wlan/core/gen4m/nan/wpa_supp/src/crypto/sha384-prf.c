/* SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause */

/*
 * Copyright (c) 2020 MediaTek Inc.
 */
/*
 * SHA384-based KDF (IEEE 802.11ac)
 * Copyright (c) 2003-2015, Jouni Malinen <j@w1.fi>
 *
 * This software may be distributed under the terms of the BSD license.
 * See README for more details.
 */

#include "wpa_supp/FourWayHandShake.h"

#include "wpa_supp/src/crypto/crypto.h"
#include "wpa_supp/src/crypto/sha384.h"
#include "wpa_supp/src/utils/common.h"

/**
 * sha384_prf - SHA384-based Key derivation function (IEEE 802.11ac, 11.6.1.7.2)
 * @key: Key for KDF
 * @key_len: Length of the key in bytes
 * @label: A unique label for each purpose of the PRF
 * @data: Extra data to bind into the key
 * @data_len: Length of the data
 * @buf: Buffer for the generated pseudo-random key
 * @buf_len: Number of bytes of key to generate
 *
 * This function is used to derive new, cryptographically separate keys from a
 * given key.
 */
void
sha384_prf(const u8 *key, size_t key_len, const char *label, const u8 *data,
	   size_t data_len, u8 *buf, size_t buf_len) {
	sha384_prf_bits(key, key_len, label, data, data_len, buf, buf_len * 8);
}

/**
 * sha384_prf_bits - IEEE Std 802.11ac-2013, 11.6.1.7.2 Key derivation function
 * @key: Key for KDF
 * @key_len: Length of the key in bytes
 * @label: A unique label for each purpose of the PRF
 * @data: Extra data to bind into the key
 * @data_len: Length of the data
 * @buf: Buffer for the generated pseudo-random key
 * @buf_len: Number of bits of key to generate
 *
 * This function is used to derive new, cryptographically separate keys from a
 * given key. If the requested buf_len is not divisible by eight, the least
 * significant 1-7 bits of the last octet in the output are not part of the
 * requested output.
 */
void
sha384_prf_bits(const u8 *key, size_t key_len, const char *label,
		const u8 *data, size_t data_len, u8 *buf, size_t buf_len_bits) {
	u16 counter = 1;
	size_t pos, plen;
	u8 hash[SHA384_MAC_LEN];
	const u8 *addr[4];
	size_t len[4];
	u8 counter_le[2], length_le[2];
	size_t buf_len = (buf_len_bits + 7) / 8;

	addr[0] = counter_le;
	len[0] = 2;
	addr[1] = (u8 *)label;
	len[1] = os_strlen(label);
	addr[2] = data;
	len[2] = data_len;
	addr[3] = length_le;
	len[3] = sizeof(length_le);

	WPA_PUT_LE16(length_le, buf_len_bits);
	pos = 0;
	while (pos < buf_len) {
		plen = buf_len - pos;
		WPA_PUT_LE16(counter_le, counter);
		if (plen >= SHA384_MAC_LEN) {
			hmac_sha384_vector(key, key_len, 4, addr, len,
					   &buf[pos]);
			pos += SHA384_MAC_LEN;
		} else {
			hmac_sha384_vector(key, key_len, 4, addr, len, hash);
			os_memcpy(&buf[pos], hash, plen);
			pos += plen;
			break;
		}
		counter++;
	}

	/*
	 * Mask out unused bits in the last octet if it does not use all the
	 * bits.
	 */
	if (buf_len_bits % 8) {
		u8 mask = 0xff << (8 - buf_len_bits % 8);

		buf[pos - 1] &= mask;
	}

	os_memset(hash, 0, sizeof(hash));
}
