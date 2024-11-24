/* SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause */

/*
 * Copyright (c) 2020 MediaTek Inc.
 */
/*
 * SHA1 internal definitions
 * Copyright (c) 2003-2005, Jouni Malinen <j@w1.fi>
 *
 * This software may be distributed under the terms of the BSD license.
 * See README for more details.
 */

#ifndef SHA1_I_H
#define SHA1_I_H

/*#include "../../../../ROM/include/mgmt/crypto_i.h"*/

#if 1
struct SHA1Context {
	u32 state[5];
	u32 count[2];
	unsigned char buffer[64];
};
#endif

void SHA1Init(struct SHA1Context *context);
void SHA1Update(struct SHA1Context *context, const void *data, u32 len);
void SHA1Final(unsigned char digest[20], struct SHA1Context *context);
void SHA1Transform(u32 state[5], const unsigned char buffer[64]);

#endif /* SHA1_I_H */
