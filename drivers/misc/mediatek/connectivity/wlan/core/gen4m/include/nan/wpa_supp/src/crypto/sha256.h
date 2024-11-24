/* SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause */

/*
 * Copyright (c) 2020 MediaTek Inc.
 */
/*
 * SHA256 hash implementation and interface functions
 * Copyright (c) 2003-2014, Jouni Malinen <j@w1.fi>
 *
 * This software may be distributed under the terms of the BSD license.
 * See README for more details.
 */

#ifndef SHA256_H
#define SHA256_H

#define SHA256_MAC_LEN 32

int hmac_sha256_vector(const u8 *key, size_t key_len, size_t num_elem,
		       const u8 *addr[], const size_t *len, u8 *mac);
int hmac_sha256(const u8 *key, size_t key_len, const u8 *data, size_t data_len,
		u8 *mac);
void sha256_prf(const u8 *key, size_t key_len, const char *label,
		const u8 *data, size_t data_len, u8 *buf, size_t buf_len);
void sha256_prf_bits(const u8 *key, size_t key_len, const char *label,
		     const u8 *data, size_t data_len, u8 *buf,
		     size_t buf_len_bits);
void tls_prf_sha256(const u8 *secret, size_t secret_len, const char *label,
		    const u8 *seed, size_t seed_len, u8 *out, size_t outlen);
int hmac_sha256_kdf(const u8 *secret, size_t secret_len, const char *label,
		    const u8 *seed, size_t seed_len, u8 *out, size_t outlen);

#endif /* SHA256_H */
