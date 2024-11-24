/* SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause */

/*
 * Copyright (c) 2020 MediaTek Inc.
 */
/*
 * SHA-512 internal definitions
 * Copyright (c) 2015, Pali Rohár <pali.rohar@gmail.com>
 *
 * This software may be distributed under the terms of the BSD license.
 * See README for more details.
 */

#ifndef SHA512_I_H
#define SHA512_I_H

#define SHA512_BLOCK_SIZE 128

struct nan_rdf_sha512_state {
	u64 length, state[8];
	u32 curlen;
	u8 buf[SHA512_BLOCK_SIZE];
};

void sha512_init(struct nan_rdf_sha512_state *md);
int sha512_process(struct nan_rdf_sha512_state *md, const unsigned char *in,
		   unsigned long inlen);
int sha512_done(struct nan_rdf_sha512_state *md, unsigned char *out);

#endif /* SHA512_I_H */
