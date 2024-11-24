/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _NFT_REDIR_H_
#define _NFT_REDIR_H_

struct nft_redir {
	u8			sreg_proto_min;
	u8			sreg_proto_max;
	u16			flags;
};

extern const struct nla_policy nft_redir_policy[];

int nft_redir_init(const struct nft_ctx *ctx,
		   const struct nft_expr *expr,
		   const struct nlattr * const tb[]);

int nft_redir_dump(struct sk_buff *skb, const struct nft_expr *expr);

int nft_redir_validate(const struct nft_ctx *ctx, const struct nft_expr *expr,
		       const struct nft_data **data);

#endif /* _NFT_REDIR_H_ */
