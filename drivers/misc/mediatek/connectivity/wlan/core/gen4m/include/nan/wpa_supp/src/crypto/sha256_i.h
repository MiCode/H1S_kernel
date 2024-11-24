/* SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause */

/*
 * Copyright (c) 2020 MediaTek Inc.
 */
/*
 * SHA-256 internal definitions
 * Copyright (c) 2003-2011, Jouni Malinen <j@w1.fi>
 *
 * This software may be distributed under the terms of the BSD license.
 * See README for more details.
 */

#ifndef SHA256_I_H
#define SHA256_I_H

#define SHA256_BLOCK_SIZE 64

struct nan_rdf_sha256_state {
	u64 length;
	u32 state[8], curlen;
	u8 buf[SHA256_BLOCK_SIZE];
};

void nan_rdf_sha256_init(struct nan_rdf_sha256_state *md);
int sha256_process(struct nan_rdf_sha256_state *md, const unsigned char *in,
		   unsigned long inlen);
int sha256_done(struct nan_rdf_sha256_state *md, unsigned char *out);

void caculate_pmkid(u8 *key, u8 *IMAC, u8 *RMAC, u8 *serviceName, u8 *pmkid);

#endif /* SHA256_I_H */
