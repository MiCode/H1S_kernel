/*
 * Copyright (c) 2007-2009 Patrick McHardy <kaber@trash.net>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Development of this code funded by Astaro AG (http://www.astaro.com/)
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/skbuff.h>
#include <linux/netlink.h>
#include <linux/vmalloc.h>
#include <linux/rhashtable.h>
#include <linux/netfilter.h>
#include <linux/netfilter/nfnetlink.h>
#include <linux/netfilter/nf_tables.h>
#include <net/netfilter/nf_flow_table.h>
#include <net/netfilter/nf_tables_core.h>
#include <net/netfilter/nf_tables.h>
#include <net/net_namespace.h>
#include <net/netns/generic.h>
#include <net/sock.h>

#define NFT_MODULE_AUTOLOAD_LIMIT (MODULE_NAME_LEN - sizeof("nft-expr-255-"))
#define NFT_SET_MAX_ANONLEN 16

unsigned int nf_tables_net_id __read_mostly;
EXPORT_SYMBOL_GPL(nf_tables_net_id);

static LIST_HEAD(nf_tables_expressions);
static LIST_HEAD(nf_tables_objects);
static LIST_HEAD(nf_tables_flowtables);
static LIST_HEAD(nf_tables_gc_list);
static DEFINE_SPINLOCK(nf_tables_gc_list_lock);
static u64 table_handle;

enum {
	NFT_VALIDATE_SKIP	= 0,
	NFT_VALIDATE_NEED,
	NFT_VALIDATE_DO,
};

static u32 nft_chain_hash(const void *data, u32 len, u32 seed);
static u32 nft_chain_hash_obj(const void *data, u32 len, u32 seed);
static int nft_chain_hash_cmp(struct rhashtable_compare_arg *, const void *);

static const struct rhashtable_params nft_chain_ht_params = {
	.head_offset		= offsetof(struct nft_chain, rhlhead),
	.key_offset		= offsetof(struct nft_chain, name),
	.hashfn			= nft_chain_hash,
	.obj_hashfn		= nft_chain_hash_obj,
	.obj_cmpfn		= nft_chain_hash_cmp,
	.locks_mul		= 1,
	.automatic_shrinking	= true,
};

static void nft_validate_state_update(struct net *net, u8 new_validate_state)
{
	struct nftables_pernet *nft_net = net_generic(net, nf_tables_net_id);

	switch (nft_net->validate_state) {
	case NFT_VALIDATE_SKIP:
		WARN_ON_ONCE(new_validate_state == NFT_VALIDATE_DO);
		break;
	case NFT_VALIDATE_NEED:
		break;
	case NFT_VALIDATE_DO:
		if (new_validate_state == NFT_VALIDATE_NEED)
			return;
	}

	nft_net->validate_state = new_validate_state;
}

static void nft_trans_gc_work(struct work_struct *work);
static DECLARE_WORK(trans_gc_work, nft_trans_gc_work);

static void nft_ctx_init(struct nft_ctx *ctx,
			 struct net *net,
			 const struct sk_buff *skb,
			 const struct nlmsghdr *nlh,
			 u8 family,
			 struct nft_table *table,
			 struct nft_chain *chain,
			 const struct nlattr * const *nla)
{
	ctx->net	= net;
	ctx->family	= family;
	ctx->level	= 0;
	ctx->table	= table;
	ctx->chain	= chain;
	ctx->nla   	= nla;
	ctx->portid	= NETLINK_CB(skb).portid;
	ctx->report	= nlmsg_report(nlh);
	ctx->seq	= nlh->nlmsg_seq;
}

static struct nft_trans *nft_trans_alloc_gfp(const struct nft_ctx *ctx,
					     int msg_type, u32 size, gfp_t gfp)
{
	struct nft_trans *trans;

	trans = kzalloc(sizeof(struct nft_trans) + size, gfp);
	if (trans == NULL)
		return NULL;

	INIT_LIST_HEAD(&trans->list);
	INIT_LIST_HEAD(&trans->binding_list);
	trans->msg_type = msg_type;
	trans->ctx	= *ctx;

	return trans;
}

static struct nft_trans *nft_trans_alloc(const struct nft_ctx *ctx,
					 int msg_type, u32 size)
{
	return nft_trans_alloc_gfp(ctx, msg_type, size, GFP_KERNEL);
}

static void nft_trans_list_del(struct nft_trans *trans)
{
	list_del(&trans->list);
	list_del(&trans->binding_list);
}

static void nft_trans_destroy(struct nft_trans *trans)
{
	nft_trans_list_del(trans);
	kfree(trans);
}

static void __nft_set_trans_bind(const struct nft_ctx *ctx, struct nft_set *set,
				 bool bind)
{
	struct nftables_pernet *nft_net;
	struct net *net = ctx->net;
	struct nft_trans *trans;

	if (!nft_set_is_anonymous(set))
		return;

	nft_net = net_generic(net, nf_tables_net_id);
	list_for_each_entry_reverse(trans, &nft_net->commit_list, list) {
		switch (trans->msg_type) {
		case NFT_MSG_NEWSET:
			if (nft_trans_set(trans) == set)
				nft_trans_set_bound(trans) = bind;
			break;
		case NFT_MSG_NEWSETELEM:
			if (nft_trans_elem_set(trans) == set)
				nft_trans_elem_set_bound(trans) = bind;
			break;
		}
	}
}

static void nft_set_trans_bind(const struct nft_ctx *ctx, struct nft_set *set)
{
	return __nft_set_trans_bind(ctx, set, true);
}

static void nft_set_trans_unbind(const struct nft_ctx *ctx, struct nft_set *set)
{
	return __nft_set_trans_bind(ctx, set, false);
}

static void nft_trans_commit_list_add_tail(struct net *net, struct nft_trans *trans)
{
	struct nftables_pernet *nft_net;

	nft_net = net_generic(net, nf_tables_net_id);
	switch (trans->msg_type) {
	case NFT_MSG_NEWSET:
		if (nft_set_is_anonymous(nft_trans_set(trans)))
			list_add_tail(&trans->binding_list, &nft_net->binding_list);
		break;
	}

	list_add_tail(&trans->list, &nft_net->commit_list);
}

static int nf_tables_register_hook(struct net *net,
				   const struct nft_table *table,
				   struct nft_chain *chain)
{
	const struct nft_base_chain *basechain;
	const struct nf_hook_ops *ops;

	if (table->flags & NFT_TABLE_F_DORMANT ||
	    !nft_is_base_chain(chain))
		return 0;

	basechain = nft_base_chain(chain);
	ops = &basechain->ops;

	if (basechain->type->ops_register)
		return basechain->type->ops_register(net, ops);

	return nf_register_net_hook(net, ops);
}

static void __nf_tables_unregister_hook(struct net *net,
					const struct nft_table *table,
					struct nft_chain *chain,
					bool release_netdev)
{
	const struct nft_base_chain *basechain;
	const struct nf_hook_ops *ops;

	if (table->flags & NFT_TABLE_F_DORMANT ||
	    !nft_is_base_chain(chain))
		return;
	basechain = nft_base_chain(chain);
	ops = &basechain->ops;

	if (basechain->type->ops_unregister)
		return basechain->type->ops_unregister(net, ops);

	nf_unregister_net_hook(net, ops);
	if (release_netdev &&
	    table->family == NFPROTO_NETDEV)
		nft_base_chain(chain)->ops.dev = NULL;
}

static void nf_tables_unregister_hook(struct net *net,
				      const struct nft_table *table,
				      struct nft_chain *chain)
{
	__nf_tables_unregister_hook(net, table, chain, false);
}

static int nft_trans_table_add(struct nft_ctx *ctx, int msg_type)
{
	struct nft_trans *trans;

	trans = nft_trans_alloc(ctx, msg_type, sizeof(struct nft_trans_table));
	if (trans == NULL)
		return -ENOMEM;

	if (msg_type == NFT_MSG_NEWTABLE)
		nft_activate_next(ctx->net, ctx->table);

	nft_trans_commit_list_add_tail(ctx->net, trans);
	return 0;
}

static int nft_deltable(struct nft_ctx *ctx)
{
	int err;

	err = nft_trans_table_add(ctx, NFT_MSG_DELTABLE);
	if (err < 0)
		return err;

	nft_deactivate_next(ctx->net, ctx->table);
	return err;
}

static int nft_trans_chain_add(struct nft_ctx *ctx, int msg_type)
{
	struct nft_trans *trans;

	trans = nft_trans_alloc(ctx, msg_type, sizeof(struct nft_trans_chain));
	if (trans == NULL)
		return -ENOMEM;

	if (msg_type == NFT_MSG_NEWCHAIN)
		nft_activate_next(ctx->net, ctx->chain);

	nft_trans_commit_list_add_tail(ctx->net, trans);
	return 0;
}

static int nft_delchain(struct nft_ctx *ctx)
{
	int err;

	err = nft_trans_chain_add(ctx, NFT_MSG_DELCHAIN);
	if (err < 0)
		return err;

	nft_use_dec(&ctx->table->use);
	nft_deactivate_next(ctx->net, ctx->chain);

	return err;
}

static void nft_rule_expr_activate(const struct nft_ctx *ctx,
				   struct nft_rule *rule)
{
	struct nft_expr *expr;

	expr = nft_expr_first(rule);
	while (expr != nft_expr_last(rule) && expr->ops) {
		if (expr->ops->activate)
			expr->ops->activate(ctx, expr);

		expr = nft_expr_next(expr);
	}
}

static void nft_rule_expr_deactivate(const struct nft_ctx *ctx,
				     struct nft_rule *rule,
				     enum nft_trans_phase phase)
{
	struct nft_expr *expr;

	expr = nft_expr_first(rule);
	while (expr != nft_expr_last(rule) && expr->ops) {
		if (expr->ops->deactivate)
			expr->ops->deactivate(ctx, expr, phase);

		expr = nft_expr_next(expr);
	}
}

static int
nf_tables_delrule_deactivate(struct nft_ctx *ctx, struct nft_rule *rule)
{
	/* You cannot delete the same rule twice */
	if (nft_is_active_next(ctx->net, rule)) {
		nft_deactivate_next(ctx->net, rule);
		nft_use_dec(&ctx->chain->use);
		return 0;
	}
	return -ENOENT;
}

static struct nft_trans *nft_trans_rule_add(struct nft_ctx *ctx, int msg_type,
					    struct nft_rule *rule)
{
	struct nft_trans *trans;

	trans = nft_trans_alloc(ctx, msg_type, sizeof(struct nft_trans_rule));
	if (trans == NULL)
		return NULL;

	if (msg_type == NFT_MSG_NEWRULE && ctx->nla[NFTA_RULE_ID] != NULL) {
		nft_trans_rule_id(trans) =
			ntohl(nla_get_be32(ctx->nla[NFTA_RULE_ID]));
	}
	nft_trans_rule(trans) = rule;
	nft_trans_commit_list_add_tail(ctx->net, trans);

	return trans;
}

static int nft_delrule(struct nft_ctx *ctx, struct nft_rule *rule)
{
	struct nft_trans *trans;
	int err;

	trans = nft_trans_rule_add(ctx, NFT_MSG_DELRULE, rule);
	if (trans == NULL)
		return -ENOMEM;

	err = nf_tables_delrule_deactivate(ctx, rule);
	if (err < 0) {
		nft_trans_destroy(trans);
		return err;
	}
	nft_rule_expr_deactivate(ctx, rule, NFT_TRANS_PREPARE);

	return 0;
}

static int nft_delrule_by_chain(struct nft_ctx *ctx)
{
	struct nft_rule *rule;
	int err;

	list_for_each_entry(rule, &ctx->chain->rules, list) {
		if (!nft_is_active_next(ctx->net, rule))
			continue;

		err = nft_delrule(ctx, rule);
		if (err < 0)
			return err;
	}
	return 0;
}

static int nft_trans_set_add(const struct nft_ctx *ctx, int msg_type,
			     struct nft_set *set)
{
	struct nft_trans *trans;

	trans = nft_trans_alloc(ctx, msg_type, sizeof(struct nft_trans_set));
	if (trans == NULL)
		return -ENOMEM;

	if (msg_type == NFT_MSG_NEWSET && ctx->nla[NFTA_SET_ID] != NULL) {
		nft_trans_set_id(trans) =
			ntohl(nla_get_be32(ctx->nla[NFTA_SET_ID]));
		nft_activate_next(ctx->net, set);
	}
	nft_trans_set(trans) = set;
	nft_trans_commit_list_add_tail(ctx->net, trans);

	return 0;
}

static int nft_mapelem_deactivate(const struct nft_ctx *ctx,
				  struct nft_set *set,
				  const struct nft_set_iter *iter,
				  struct nft_set_elem *elem)
{
	nft_setelem_data_deactivate(ctx->net, set, elem);

	return 0;
}

static void nft_map_deactivate(const struct nft_ctx *ctx, struct nft_set *set)
{
	struct nft_set_iter iter = {
		.genmask	= nft_genmask_next(ctx->net),
		.fn		= nft_mapelem_deactivate,
	};

	set->ops->walk(ctx, set, &iter);
	WARN_ON_ONCE(iter.err);
}

static int nft_delset(const struct nft_ctx *ctx, struct nft_set *set)
{
	int err;

	err = nft_trans_set_add(ctx, NFT_MSG_DELSET, set);
	if (err < 0)
		return err;

	if (set->flags & (NFT_SET_MAP | NFT_SET_OBJECT))
		nft_map_deactivate(ctx, set);

	nft_deactivate_next(ctx->net, set);
	nft_use_dec(&ctx->table->use);

	return err;
}

static int nft_trans_obj_add(struct nft_ctx *ctx, int msg_type,
			     struct nft_object *obj)
{
	struct nft_trans *trans;

	trans = nft_trans_alloc(ctx, msg_type, sizeof(struct nft_trans_obj));
	if (trans == NULL)
		return -ENOMEM;

	if (msg_type == NFT_MSG_NEWOBJ)
		nft_activate_next(ctx->net, obj);

	nft_trans_obj(trans) = obj;
	nft_trans_commit_list_add_tail(ctx->net, trans);

	return 0;
}

static int nft_delobj(struct nft_ctx *ctx, struct nft_object *obj)
{
	int err;

	err = nft_trans_obj_add(ctx, NFT_MSG_DELOBJ, obj);
	if (err < 0)
		return err;

	nft_deactivate_next(ctx->net, obj);
	nft_use_dec(&ctx->table->use);

	return err;
}

static int nft_trans_flowtable_add(struct nft_ctx *ctx, int msg_type,
				   struct nft_flowtable *flowtable)
{
	struct nft_trans *trans;

	trans = nft_trans_alloc(ctx, msg_type,
				sizeof(struct nft_trans_flowtable));
	if (trans == NULL)
		return -ENOMEM;

	if (msg_type == NFT_MSG_NEWFLOWTABLE)
		nft_activate_next(ctx->net, flowtable);

	nft_trans_flowtable(trans) = flowtable;
	nft_trans_commit_list_add_tail(ctx->net, trans);

	return 0;
}

static int nft_delflowtable(struct nft_ctx *ctx,
			    struct nft_flowtable *flowtable)
{
	int err;

	err = nft_trans_flowtable_add(ctx, NFT_MSG_DELFLOWTABLE, flowtable);
	if (err < 0)
		return err;

	nft_deactivate_next(ctx->net, flowtable);
	nft_use_dec(&ctx->table->use);

	return err;
}

/*
 * Tables
 */

static struct nft_table *nft_table_lookup(const struct net *net,
					  const struct nlattr *nla,
					  u8 family, u8 genmask)
{
	struct nftables_pernet *nft_net;
	struct nft_table *table;

	if (nla == NULL)
		return ERR_PTR(-EINVAL);

	nft_net = net_generic(net, nf_tables_net_id);
	list_for_each_entry_rcu(table, &nft_net->tables, list) {
		if (!nla_strcmp(nla, table->name) &&
		    table->family == family &&
		    nft_active_genmask(table, genmask))
			return table;
	}

	return ERR_PTR(-ENOENT);
}

static struct nft_table *nft_table_lookup_byhandle(const struct net *net,
						   const struct nlattr *nla,
						   int family, u8 genmask)
{
	struct nftables_pernet *nft_net;
	struct nft_table *table;

	nft_net = net_generic(net, nf_tables_net_id);
	list_for_each_entry(table, &nft_net->tables, list) {
		if (be64_to_cpu(nla_get_be64(nla)) == table->handle &&
		    table->family == family &&
		    nft_active_genmask(table, genmask))
			return table;
	}

	return ERR_PTR(-ENOENT);
}

static inline u64 nf_tables_alloc_handle(struct nft_table *table)
{
	return ++table->hgenerator;
}

static const struct nft_chain_type *chain_type[NFPROTO_NUMPROTO][NFT_CHAIN_T_MAX];

static const struct nft_chain_type *
__nft_chain_type_get(u8 family, enum nft_chain_types type)
{
	if (family >= NFPROTO_NUMPROTO ||
	    type >= NFT_CHAIN_T_MAX)
		return NULL;

	return chain_type[family][type];
}

static const struct nft_chain_type *
__nf_tables_chain_type_lookup(const struct nlattr *nla, u8 family)
{
	const struct nft_chain_type *type;
	int i;

	for (i = 0; i < NFT_CHAIN_T_MAX; i++) {
		type = __nft_chain_type_get(family, i);
		if (!type)
			continue;
		if (!nla_strcmp(nla, type->name))
			return type;
	}
	return NULL;
}

/*
 * Loading a module requires dropping mutex that guards the transaction.
 * A different client might race to start a new transaction meanwhile. Zap the
 * list of pending transaction and then restore it once the mutex is grabbed
 * again. Users of this function return EAGAIN which implicitly triggers the
 * transaction abort path to clean up the list of pending transactions.
 */
#ifdef CONFIG_MODULES
static void nft_request_module(struct net *net, const char *fmt, ...)
{
	char module_name[MODULE_NAME_LEN];
	struct nftables_pernet *nft_net;
	LIST_HEAD(commit_list);
	va_list args;
	int ret;

	nft_net = net_generic(net, nf_tables_net_id);
	list_splice_init(&nft_net->commit_list, &commit_list);

	va_start(args, fmt);
	ret = vsnprintf(module_name, MODULE_NAME_LEN, fmt, args);
	va_end(args);
	if (ret >= MODULE_NAME_LEN)
		return;

	mutex_unlock(&nft_net->commit_mutex);
	request_module("%s", module_name);
	mutex_lock(&nft_net->commit_mutex);

	WARN_ON_ONCE(!list_empty(&nft_net->commit_list));
	list_splice(&commit_list, &nft_net->commit_list);
}
#endif

static void lockdep_nfnl_nft_mutex_not_held(void)
{
#ifdef CONFIG_PROVE_LOCKING
	if (debug_locks)
		WARN_ON_ONCE(lockdep_nfnl_is_held(NFNL_SUBSYS_NFTABLES));
#endif
}

static const struct nft_chain_type *
nf_tables_chain_type_lookup(struct net *net, const struct nlattr *nla,
			    u8 family, bool autoload)
{
	const struct nft_chain_type *type;

	type = __nf_tables_chain_type_lookup(nla, family);
	if (type != NULL)
		return type;

	lockdep_nfnl_nft_mutex_not_held();
#ifdef CONFIG_MODULES
	if (autoload) {
		nft_request_module(net, "nft-chain-%u-%.*s", family,
				   nla_len(nla), (const char *)nla_data(nla));
		type = __nf_tables_chain_type_lookup(nla, family);
		if (type != NULL)
			return ERR_PTR(-EAGAIN);
	}
#endif
	return ERR_PTR(-ENOENT);
}

static __be16 nft_base_seq(const struct net *net)
{
	struct nftables_pernet *nft_net = net_generic(net, nf_tables_net_id);

	return htons(nft_net->base_seq & 0xffff);
}

static const struct nla_policy nft_table_policy[NFTA_TABLE_MAX + 1] = {
	[NFTA_TABLE_NAME]	= { .type = NLA_STRING,
				    .len = NFT_TABLE_MAXNAMELEN - 1 },
	[NFTA_TABLE_FLAGS]	= { .type = NLA_U32 },
	[NFTA_TABLE_HANDLE]	= { .type = NLA_U64 },
};

static int nf_tables_fill_table_info(struct sk_buff *skb, struct net *net,
				     u32 portid, u32 seq, int event, u32 flags,
				     int family, const struct nft_table *table)
{
	struct nlmsghdr *nlh;

	event = nfnl_msg_type(NFNL_SUBSYS_NFTABLES, event);
	nlh = nfnl_msg_put(skb, portid, seq, event, flags, family,
			   NFNETLINK_V0, nft_base_seq(net));
	if (!nlh)
		goto nla_put_failure;

	if (nla_put_string(skb, NFTA_TABLE_NAME, table->name) ||
	    nla_put_be32(skb, NFTA_TABLE_FLAGS,
			 htonl(table->flags & NFT_TABLE_F_MASK)) ||
	    nla_put_be32(skb, NFTA_TABLE_USE, htonl(table->use)) ||
	    nla_put_be64(skb, NFTA_TABLE_HANDLE, cpu_to_be64(table->handle),
			 NFTA_TABLE_PAD))
		goto nla_put_failure;

	nlmsg_end(skb, nlh);
	return 0;

nla_put_failure:
	nlmsg_trim(skb, nlh);
	return -1;
}

static void nf_tables_table_notify(const struct nft_ctx *ctx, int event)
{
	struct sk_buff *skb;
	int err;

	if (!ctx->report &&
	    !nfnetlink_has_listeners(ctx->net, NFNLGRP_NFTABLES))
		return;

	skb = nlmsg_new(NLMSG_GOODSIZE, GFP_KERNEL);
	if (skb == NULL)
		goto err;

	err = nf_tables_fill_table_info(skb, ctx->net, ctx->portid, ctx->seq,
					event, 0, ctx->family, ctx->table);
	if (err < 0) {
		kfree_skb(skb);
		goto err;
	}

	nfnetlink_send(skb, ctx->net, ctx->portid, NFNLGRP_NFTABLES,
		       ctx->report, GFP_KERNEL);
	return;
err:
	nfnetlink_set_err(ctx->net, ctx->portid, NFNLGRP_NFTABLES, -ENOBUFS);
}

static int nf_tables_dump_tables(struct sk_buff *skb,
				 struct netlink_callback *cb)
{
	const struct nfgenmsg *nfmsg = nlmsg_data(cb->nlh);
	struct nftables_pernet *nft_net;
	const struct nft_table *table;
	unsigned int idx = 0, s_idx = cb->args[0];
	struct net *net = sock_net(skb->sk);
	int family = nfmsg->nfgen_family;

	rcu_read_lock();
	nft_net = net_generic(net, nf_tables_net_id);
	cb->seq = nft_net->base_seq;

	list_for_each_entry_rcu(table, &nft_net->tables, list) {
		if (family != NFPROTO_UNSPEC && family != table->family)
			continue;

		if (idx < s_idx)
			goto cont;
		if (idx > s_idx)
			memset(&cb->args[1], 0,
			       sizeof(cb->args) - sizeof(cb->args[0]));
		if (!nft_is_active(net, table))
			continue;
		if (nf_tables_fill_table_info(skb, net,
					      NETLINK_CB(cb->skb).portid,
					      cb->nlh->nlmsg_seq,
					      NFT_MSG_NEWTABLE, NLM_F_MULTI,
					      table->family, table) < 0)
			goto done;

		nl_dump_check_consistent(cb, nlmsg_hdr(skb));
cont:
		idx++;
	}
done:
	rcu_read_unlock();
	cb->args[0] = idx;
	return skb->len;
}

static int nft_netlink_dump_start_rcu(struct sock *nlsk, struct sk_buff *skb,
				      const struct nlmsghdr *nlh,
				      struct netlink_dump_control *c)
{
	int err;

	if (!try_module_get(THIS_MODULE))
		return -EINVAL;

	rcu_read_unlock();
	err = netlink_dump_start(nlsk, skb, nlh, c);
	rcu_read_lock();
	module_put(THIS_MODULE);

	return err;
}

/* called with rcu_read_lock held */
static int nf_tables_gettable(struct net *net, struct sock *nlsk,
			      struct sk_buff *skb, const struct nlmsghdr *nlh,
			      const struct nlattr * const nla[],
			      struct netlink_ext_ack *extack)
{
	const struct nfgenmsg *nfmsg = nlmsg_data(nlh);
	u8 genmask = nft_genmask_cur(net);
	const struct nft_table *table;
	struct sk_buff *skb2;
	int family = nfmsg->nfgen_family;
	int err;

	if (nlh->nlmsg_flags & NLM_F_DUMP) {
		struct netlink_dump_control c = {
			.dump = nf_tables_dump_tables,
			.module = THIS_MODULE,
		};

		return nft_netlink_dump_start_rcu(nlsk, skb, nlh, &c);
	}

	table = nft_table_lookup(net, nla[NFTA_TABLE_NAME], family, genmask);
	if (IS_ERR(table)) {
		NL_SET_BAD_ATTR(extack, nla[NFTA_TABLE_NAME]);
		return PTR_ERR(table);
	}

	skb2 = alloc_skb(NLMSG_GOODSIZE, GFP_ATOMIC);
	if (!skb2)
		return -ENOMEM;

	err = nf_tables_fill_table_info(skb2, net, NETLINK_CB(skb).portid,
					nlh->nlmsg_seq, NFT_MSG_NEWTABLE, 0,
					family, table);
	if (err < 0)
		goto err_fill_table_info;

	return nfnetlink_unicast(skb2, net, NETLINK_CB(skb).portid);

err_fill_table_info:
	kfree_skb(skb2);
	return err;
}

static void nft_table_disable(struct net *net, struct nft_table *table, u32 cnt)
{
	struct nft_chain *chain;
	u32 i = 0;

	list_for_each_entry(chain, &table->chains, list) {
		if (!nft_is_active_next(net, chain))
			continue;
		if (!nft_is_base_chain(chain))
			continue;

		if (cnt && i++ == cnt)
			break;

		nf_tables_unregister_hook(net, table, chain);
	}
}

static int nf_tables_table_enable(struct net *net, struct nft_table *table)
{
	struct nft_chain *chain;
	int err, i = 0;

	list_for_each_entry(chain, &table->chains, list) {
		if (!nft_is_active_next(net, chain))
			continue;
		if (!nft_is_base_chain(chain))
			continue;

		err = nf_tables_register_hook(net, table, chain);
		if (err < 0)
			goto err;

		i++;
	}
	return 0;
err:
	if (i)
		nft_table_disable(net, table, i);
	return err;
}

static void nf_tables_table_disable(struct net *net, struct nft_table *table)
{
	table->flags &= ~NFT_TABLE_F_DORMANT;
	nft_table_disable(net, table, 0);
	table->flags |= NFT_TABLE_F_DORMANT;
}

#define __NFT_TABLE_F_INTERNAL		(NFT_TABLE_F_MASK + 1)
#define __NFT_TABLE_F_WAS_DORMANT	(__NFT_TABLE_F_INTERNAL << 0)
#define __NFT_TABLE_F_WAS_AWAKEN	(__NFT_TABLE_F_INTERNAL << 1)
#define __NFT_TABLE_F_UPDATE		(__NFT_TABLE_F_WAS_DORMANT | \
					 __NFT_TABLE_F_WAS_AWAKEN)

static bool nft_table_pending_update(const struct nft_ctx *ctx)
{
	struct nftables_pernet *nft_net = net_generic(ctx->net, nf_tables_net_id);
	struct nft_trans *trans;

	if (ctx->table->flags & __NFT_TABLE_F_UPDATE)
		return true;

	list_for_each_entry(trans, &nft_net->commit_list, list) {
		if (trans->ctx.table == ctx->table &&
		    trans->msg_type == NFT_MSG_DELCHAIN &&
		    nft_is_base_chain(trans->ctx.chain))
			return true;
	}

	return false;
}

static int nf_tables_updtable(struct nft_ctx *ctx)
{
	struct nft_trans *trans;
	u32 flags;
	int ret;

	if (!ctx->nla[NFTA_TABLE_FLAGS])
		return 0;

	flags = ntohl(nla_get_be32(ctx->nla[NFTA_TABLE_FLAGS]));
	if (flags & ~NFT_TABLE_F_DORMANT)
		return -EINVAL;

	if (flags == (ctx->table->flags & NFT_TABLE_F_MASK))
		return 0;

	/* No dormant off/on/off/on games in single transaction */
	if (nft_table_pending_update(ctx))
		return -EINVAL;

	trans = nft_trans_alloc(ctx, NFT_MSG_NEWTABLE,
				sizeof(struct nft_trans_table));
	if (trans == NULL)
		return -ENOMEM;

	if ((flags & NFT_TABLE_F_DORMANT) &&
	    !(ctx->table->flags & NFT_TABLE_F_DORMANT)) {
		ctx->table->flags |= NFT_TABLE_F_DORMANT;
		if (!(ctx->table->flags & __NFT_TABLE_F_UPDATE))
			ctx->table->flags |= __NFT_TABLE_F_WAS_AWAKEN;
	} else if (!(flags & NFT_TABLE_F_DORMANT) &&
		   ctx->table->flags & NFT_TABLE_F_DORMANT) {
		ctx->table->flags &= ~NFT_TABLE_F_DORMANT;
		if (!(ctx->table->flags & __NFT_TABLE_F_UPDATE)) {
			ret = nf_tables_table_enable(ctx->net, ctx->table);
			if (ret < 0)
				goto err_register_hooks;

			ctx->table->flags |= __NFT_TABLE_F_WAS_DORMANT;
		}
	}

	nft_trans_table_update(trans) = true;
	nft_trans_commit_list_add_tail(ctx->net, trans);

	return 0;

err_register_hooks:
	ctx->table->flags |= NFT_TABLE_F_DORMANT;
	nft_trans_destroy(trans);
	return ret;
}

static u32 nft_chain_hash(const void *data, u32 len, u32 seed)
{
	const char *name = data;

	return jhash(name, strlen(name), seed);
}

static u32 nft_chain_hash_obj(const void *data, u32 len, u32 seed)
{
	const struct nft_chain *chain = data;

	return nft_chain_hash(chain->name, 0, seed);
}

static int nft_chain_hash_cmp(struct rhashtable_compare_arg *arg,
			      const void *ptr)
{
	const struct nft_chain *chain = ptr;
	const char *name = arg->key;

	return strcmp(chain->name, name);
}

static bool nft_supported_family(u8 family)
{
	return false
#ifdef CONFIG_NF_TABLES_INET
		|| family == NFPROTO_INET
#endif
#ifdef CONFIG_NF_TABLES_IPV4
		|| family == NFPROTO_IPV4
#endif
#ifdef CONFIG_NF_TABLES_ARP
		|| family == NFPROTO_ARP
#endif
#ifdef CONFIG_NF_TABLES_NETDEV
		|| family == NFPROTO_NETDEV
#endif
#if IS_ENABLED(CONFIG_NF_TABLES_BRIDGE)
		|| family == NFPROTO_BRIDGE
#endif
#ifdef CONFIG_NF_TABLES_IPV6
		|| family == NFPROTO_IPV6
#endif
		;
}

static int nf_tables_newtable(struct net *net, struct sock *nlsk,
			      struct sk_buff *skb, const struct nlmsghdr *nlh,
			      const struct nlattr * const nla[],
			      struct netlink_ext_ack *extack)
{
	struct nftables_pernet *nft_net = net_generic(net, nf_tables_net_id);
	const struct nfgenmsg *nfmsg = nlmsg_data(nlh);
	u8 genmask = nft_genmask_next(net);
	int family = nfmsg->nfgen_family;
	const struct nlattr *attr;
	struct nft_table *table;
	u32 flags = 0;
	struct nft_ctx ctx;
	int err;

	if (!nft_supported_family(family))
		return -EOPNOTSUPP;

	lockdep_assert_held(&nft_net->commit_mutex);
	attr = nla[NFTA_TABLE_NAME];
	table = nft_table_lookup(net, attr, family, genmask);
	if (IS_ERR(table)) {
		if (PTR_ERR(table) != -ENOENT)
			return PTR_ERR(table);
	} else {
		if (nlh->nlmsg_flags & NLM_F_EXCL) {
			NL_SET_BAD_ATTR(extack, attr);
			return -EEXIST;
		}
		if (nlh->nlmsg_flags & NLM_F_REPLACE)
			return -EOPNOTSUPP;

		nft_ctx_init(&ctx, net, skb, nlh, family, table, NULL, nla);
		return nf_tables_updtable(&ctx);
	}

	if (nla[NFTA_TABLE_FLAGS]) {
		flags = ntohl(nla_get_be32(nla[NFTA_TABLE_FLAGS]));
		if (flags & ~NFT_TABLE_F_DORMANT)
			return -EINVAL;
	}

	err = -ENOMEM;
	table = kzalloc(sizeof(*table), GFP_KERNEL);
	if (table == NULL)
		goto err_kzalloc;

	table->name = nla_strdup(attr, GFP_KERNEL);
	if (table->name == NULL)
		goto err_strdup;

	err = rhltable_init(&table->chains_ht, &nft_chain_ht_params);
	if (err)
		goto err_chain_ht;

	INIT_LIST_HEAD(&table->chains);
	INIT_LIST_HEAD(&table->sets);
	INIT_LIST_HEAD(&table->objects);
	INIT_LIST_HEAD(&table->flowtables);
	table->family = family;
	table->flags = flags;
	table->handle = ++table_handle;

	nft_ctx_init(&ctx, net, skb, nlh, family, table, NULL, nla);
	err = nft_trans_table_add(&ctx, NFT_MSG_NEWTABLE);
	if (err < 0)
		goto err_trans;

	list_add_tail_rcu(&table->list, &nft_net->tables);
	return 0;
err_trans:
	rhltable_destroy(&table->chains_ht);
err_chain_ht:
	kfree(table->name);
err_strdup:
	kfree(table);
err_kzalloc:
	return err;
}

static int nft_flush_table(struct nft_ctx *ctx)
{
	struct nft_flowtable *flowtable, *nft;
	struct nft_chain *chain, *nc;
	struct nft_object *obj, *ne;
	struct nft_set *set, *ns;
	int err;

	list_for_each_entry(chain, &ctx->table->chains, list) {
		if (!nft_is_active_next(ctx->net, chain))
			continue;

		ctx->chain = chain;

		err = nft_delrule_by_chain(ctx);
		if (err < 0)
			goto out;
	}

	list_for_each_entry_safe(set, ns, &ctx->table->sets, list) {
		if (!nft_is_active_next(ctx->net, set))
			continue;

		if (nft_set_is_anonymous(set))
			continue;

		err = nft_delset(ctx, set);
		if (err < 0)
			goto out;
	}

	list_for_each_entry_safe(flowtable, nft, &ctx->table->flowtables, list) {
		if (!nft_is_active_next(ctx->net, flowtable))
			continue;

		err = nft_delflowtable(ctx, flowtable);
		if (err < 0)
			goto out;
	}

	list_for_each_entry_safe(obj, ne, &ctx->table->objects, list) {
		if (!nft_is_active_next(ctx->net, obj))
			continue;

		err = nft_delobj(ctx, obj);
		if (err < 0)
			goto out;
	}

	list_for_each_entry_safe(chain, nc, &ctx->table->chains, list) {
		if (!nft_is_active_next(ctx->net, chain))
			continue;

		ctx->chain = chain;

		err = nft_delchain(ctx);
		if (err < 0)
			goto out;
	}

	err = nft_deltable(ctx);
out:
	return err;
}

static int nft_flush(struct nft_ctx *ctx, int family)
{
	struct nftables_pernet *nft_net = net_generic(ctx->net, nf_tables_net_id);
	struct nft_table *table, *nt;
	const struct nlattr * const *nla = ctx->nla;
	int err = 0;

	list_for_each_entry_safe(table, nt, &nft_net->tables, list) {
		if (family != AF_UNSPEC && table->family != family)
			continue;

		ctx->family = table->family;

		if (!nft_is_active_next(ctx->net, table))
			continue;

		if (nla[NFTA_TABLE_NAME] &&
		    nla_strcmp(nla[NFTA_TABLE_NAME], table->name) != 0)
			continue;

		ctx->table = table;

		err = nft_flush_table(ctx);
		if (err < 0)
			goto out;
	}
out:
	return err;
}

static int nf_tables_deltable(struct net *net, struct sock *nlsk,
			      struct sk_buff *skb, const struct nlmsghdr *nlh,
			      const struct nlattr * const nla[],
			      struct netlink_ext_ack *extack)
{
	const struct nfgenmsg *nfmsg = nlmsg_data(nlh);
	u8 genmask = nft_genmask_next(net);
	int family = nfmsg->nfgen_family;
	const struct nlattr *attr;
	struct nft_table *table;
	struct nft_ctx ctx;

	nft_ctx_init(&ctx, net, skb, nlh, 0, NULL, NULL, nla);
	if (family == AF_UNSPEC ||
	    (!nla[NFTA_TABLE_NAME] && !nla[NFTA_TABLE_HANDLE]))
		return nft_flush(&ctx, family);

	if (nla[NFTA_TABLE_HANDLE]) {
		attr = nla[NFTA_TABLE_HANDLE];
		table = nft_table_lookup_byhandle(net, attr, family, genmask);
	} else {
		attr = nla[NFTA_TABLE_NAME];
		table = nft_table_lookup(net, attr, family, genmask);
	}

	if (IS_ERR(table)) {
		NL_SET_BAD_ATTR(extack, attr);
		return PTR_ERR(table);
	}

	if (nlh->nlmsg_flags & NLM_F_NONREC &&
	    table->use > 0)
		return -EBUSY;

	ctx.family = family;
	ctx.table = table;

	return nft_flush_table(&ctx);
}

static void nf_tables_table_destroy(struct nft_ctx *ctx)
{
	if (WARN_ON(ctx->table->use > 0))
		return;

	rhltable_destroy(&ctx->table->chains_ht);
	kfree(ctx->table->name);
	kfree(ctx->table);
}

void nft_register_chain_type(const struct nft_chain_type *ctype)
{
	nfnl_lock(NFNL_SUBSYS_NFTABLES);
	if (WARN_ON(__nft_chain_type_get(ctype->family, ctype->type))) {
		nfnl_unlock(NFNL_SUBSYS_NFTABLES);
		return;
	}
	chain_type[ctype->family][ctype->type] = ctype;
	nfnl_unlock(NFNL_SUBSYS_NFTABLES);
}
EXPORT_SYMBOL_GPL(nft_register_chain_type);

void nft_unregister_chain_type(const struct nft_chain_type *ctype)
{
	nfnl_lock(NFNL_SUBSYS_NFTABLES);
	chain_type[ctype->family][ctype->type] = NULL;
	nfnl_unlock(NFNL_SUBSYS_NFTABLES);
}
EXPORT_SYMBOL_GPL(nft_unregister_chain_type);

/*
 * Chains
 */

static struct nft_chain *
nft_chain_lookup_byhandle(const struct nft_table *table, u64 handle, u8 genmask)
{
	struct nft_chain *chain;

	list_for_each_entry(chain, &table->chains, list) {
		if (chain->handle == handle &&
		    nft_active_genmask(chain, genmask))
			return chain;
	}

	return ERR_PTR(-ENOENT);
}

static bool lockdep_commit_lock_is_held(struct net *net)
{
#ifdef CONFIG_PROVE_LOCKING
	struct nftables_pernet *nft_net = net_generic(net, nf_tables_net_id);

	return lockdep_is_held(&nft_net->commit_mutex);
#else
	return true;
#endif
}

static struct nft_chain *nft_chain_lookup(struct net *net,
					  struct nft_table *table,
					  const struct nlattr *nla, u8 genmask)
{
	char search[NFT_CHAIN_MAXNAMELEN + 1];
	struct rhlist_head *tmp, *list;
	struct nft_chain *chain;

	if (nla == NULL)
		return ERR_PTR(-EINVAL);

	nla_strlcpy(search, nla, sizeof(search));

	WARN_ON(!rcu_read_lock_held() &&
		!lockdep_commit_lock_is_held(net));

	chain = ERR_PTR(-ENOENT);
	rcu_read_lock();
	list = rhltable_lookup(&table->chains_ht, search, nft_chain_ht_params);
	if (!list)
		goto out_unlock;

	rhl_for_each_entry_rcu(chain, tmp, list, rhlhead) {
		if (nft_active_genmask(chain, genmask))
			goto out_unlock;
	}
	chain = ERR_PTR(-ENOENT);
out_unlock:
	rcu_read_unlock();
	return chain;
}

static const struct nla_policy nft_chain_policy[NFTA_CHAIN_MAX + 1] = {
	[NFTA_CHAIN_TABLE]	= { .type = NLA_STRING,
				    .len = NFT_TABLE_MAXNAMELEN - 1 },
	[NFTA_CHAIN_HANDLE]	= { .type = NLA_U64 },
	[NFTA_CHAIN_NAME]	= { .type = NLA_STRING,
				    .len = NFT_CHAIN_MAXNAMELEN - 1 },
	[NFTA_CHAIN_HOOK]	= { .type = NLA_NESTED },
	[NFTA_CHAIN_POLICY]	= { .type = NLA_U32 },
	[NFTA_CHAIN_TYPE]	= { .type = NLA_STRING,
				    .len = NFT_MODULE_AUTOLOAD_LIMIT },
	[NFTA_CHAIN_COUNTERS]	= { .type = NLA_NESTED },
};

static const struct nla_policy nft_hook_policy[NFTA_HOOK_MAX + 1] = {
	[NFTA_HOOK_HOOKNUM]	= { .type = NLA_U32 },
	[NFTA_HOOK_PRIORITY]	= { .type = NLA_U32 },
	[NFTA_HOOK_DEV]		= { .type = NLA_STRING,
				    .len = IFNAMSIZ - 1 },
};

static int nft_dump_stats(struct sk_buff *skb, struct nft_stats __percpu *stats)
{
	struct nft_stats *cpu_stats, total;
	struct nlattr *nest;
	unsigned int seq;
	u64 pkts, bytes;
	int cpu;

	if (!stats)
		return 0;

	memset(&total, 0, sizeof(total));
	for_each_possible_cpu(cpu) {
		cpu_stats = per_cpu_ptr(stats, cpu);
		do {
			seq = u64_stats_fetch_begin_irq(&cpu_stats->syncp);
			pkts = cpu_stats->pkts;
			bytes = cpu_stats->bytes;
		} while (u64_stats_fetch_retry_irq(&cpu_stats->syncp, seq));
		total.pkts += pkts;
		total.bytes += bytes;
	}
	nest = nla_nest_start(skb, NFTA_CHAIN_COUNTERS);
	if (nest == NULL)
		goto nla_put_failure;

	if (nla_put_be64(skb, NFTA_COUNTER_PACKETS, cpu_to_be64(total.pkts),
			 NFTA_COUNTER_PAD) ||
	    nla_put_be64(skb, NFTA_COUNTER_BYTES, cpu_to_be64(total.bytes),
			 NFTA_COUNTER_PAD))
		goto nla_put_failure;

	nla_nest_end(skb, nest);
	return 0;

nla_put_failure:
	return -ENOSPC;
}

static int nf_tables_fill_chain_info(struct sk_buff *skb, struct net *net,
				     u32 portid, u32 seq, int event, u32 flags,
				     int family, const struct nft_table *table,
				     const struct nft_chain *chain)
{
	struct nlmsghdr *nlh;

	event = nfnl_msg_type(NFNL_SUBSYS_NFTABLES, event);
	nlh = nfnl_msg_put(skb, portid, seq, event, flags, family,
			   NFNETLINK_V0, nft_base_seq(net));
	if (!nlh)
		goto nla_put_failure;

	if (nla_put_string(skb, NFTA_CHAIN_TABLE, table->name))
		goto nla_put_failure;
	if (nla_put_be64(skb, NFTA_CHAIN_HANDLE, cpu_to_be64(chain->handle),
			 NFTA_CHAIN_PAD))
		goto nla_put_failure;
	if (nla_put_string(skb, NFTA_CHAIN_NAME, chain->name))
		goto nla_put_failure;

	if (nft_is_base_chain(chain)) {
		const struct nft_base_chain *basechain = nft_base_chain(chain);
		const struct nf_hook_ops *ops = &basechain->ops;
		struct nft_stats __percpu *stats;
		struct nlattr *nest;

		nest = nla_nest_start(skb, NFTA_CHAIN_HOOK);
		if (nest == NULL)
			goto nla_put_failure;
		if (nla_put_be32(skb, NFTA_HOOK_HOOKNUM, htonl(ops->hooknum)))
			goto nla_put_failure;
		if (nla_put_be32(skb, NFTA_HOOK_PRIORITY, htonl(ops->priority)))
			goto nla_put_failure;
		if (basechain->dev_name[0] &&
		    nla_put_string(skb, NFTA_HOOK_DEV, basechain->dev_name))
			goto nla_put_failure;
		nla_nest_end(skb, nest);

		if (nla_put_be32(skb, NFTA_CHAIN_POLICY,
				 htonl(basechain->policy)))
			goto nla_put_failure;

		if (nla_put_string(skb, NFTA_CHAIN_TYPE, basechain->type->name))
			goto nla_put_failure;

		stats = rcu_dereference_check(basechain->stats,
					      lockdep_commit_lock_is_held(net));
		if (nft_dump_stats(skb, stats))
			goto nla_put_failure;
	}

	if (nla_put_be32(skb, NFTA_CHAIN_USE, htonl(chain->use)))
		goto nla_put_failure;

	nlmsg_end(skb, nlh);
	return 0;

nla_put_failure:
	nlmsg_trim(skb, nlh);
	return -1;
}

static void nf_tables_chain_notify(const struct nft_ctx *ctx, int event)
{
	struct sk_buff *skb;
	int err;

	if (!ctx->report &&
	    !nfnetlink_has_listeners(ctx->net, NFNLGRP_NFTABLES))
		return;

	skb = nlmsg_new(NLMSG_GOODSIZE, GFP_KERNEL);
	if (skb == NULL)
		goto err;

	err = nf_tables_fill_chain_info(skb, ctx->net, ctx->portid, ctx->seq,
					event, 0, ctx->family, ctx->table,
					ctx->chain);
	if (err < 0) {
		kfree_skb(skb);
		goto err;
	}

	nfnetlink_send(skb, ctx->net, ctx->portid, NFNLGRP_NFTABLES,
		       ctx->report, GFP_KERNEL);
	return;
err:
	nfnetlink_set_err(ctx->net, ctx->portid, NFNLGRP_NFTABLES, -ENOBUFS);
}

static int nf_tables_dump_chains(struct sk_buff *skb,
				 struct netlink_callback *cb)
{
	const struct nfgenmsg *nfmsg = nlmsg_data(cb->nlh);
	const struct nft_table *table;
	const struct nft_chain *chain;
	unsigned int idx = 0, s_idx = cb->args[0];
	struct net *net = sock_net(skb->sk);
	int family = nfmsg->nfgen_family;
	struct nftables_pernet *nft_net;

	rcu_read_lock();
	nft_net = net_generic(net, nf_tables_net_id);
	cb->seq = nft_net->base_seq;

	list_for_each_entry_rcu(table, &nft_net->tables, list) {
		if (family != NFPROTO_UNSPEC && family != table->family)
			continue;

		list_for_each_entry_rcu(chain, &table->chains, list) {
			if (idx < s_idx)
				goto cont;
			if (idx > s_idx)
				memset(&cb->args[1], 0,
				       sizeof(cb->args) - sizeof(cb->args[0]));
			if (!nft_is_active(net, chain))
				continue;
			if (nf_tables_fill_chain_info(skb, net,
						      NETLINK_CB(cb->skb).portid,
						      cb->nlh->nlmsg_seq,
						      NFT_MSG_NEWCHAIN,
						      NLM_F_MULTI,
						      table->family, table,
						      chain) < 0)
				goto done;

			nl_dump_check_consistent(cb, nlmsg_hdr(skb));
cont:
			idx++;
		}
	}
done:
	rcu_read_unlock();
	cb->args[0] = idx;
	return skb->len;
}

/* called with rcu_read_lock held */
static int nf_tables_getchain(struct net *net, struct sock *nlsk,
			      struct sk_buff *skb, const struct nlmsghdr *nlh,
			      const struct nlattr * const nla[],
			      struct netlink_ext_ack *extack)
{
	const struct nfgenmsg *nfmsg = nlmsg_data(nlh);
	u8 genmask = nft_genmask_cur(net);
	const struct nft_chain *chain;
	struct nft_table *table;
	struct sk_buff *skb2;
	int family = nfmsg->nfgen_family;
	int err;

	if (nlh->nlmsg_flags & NLM_F_DUMP) {
		struct netlink_dump_control c = {
			.dump = nf_tables_dump_chains,
			.module = THIS_MODULE,
		};

		return nft_netlink_dump_start_rcu(nlsk, skb, nlh, &c);
	}

	table = nft_table_lookup(net, nla[NFTA_CHAIN_TABLE], family, genmask);
	if (IS_ERR(table)) {
		NL_SET_BAD_ATTR(extack, nla[NFTA_CHAIN_TABLE]);
		return PTR_ERR(table);
	}

	chain = nft_chain_lookup(net, table, nla[NFTA_CHAIN_NAME], genmask);
	if (IS_ERR(chain)) {
		NL_SET_BAD_ATTR(extack, nla[NFTA_CHAIN_NAME]);
		return PTR_ERR(chain);
	}

	skb2 = alloc_skb(NLMSG_GOODSIZE, GFP_ATOMIC);
	if (!skb2)
		return -ENOMEM;

	err = nf_tables_fill_chain_info(skb2, net, NETLINK_CB(skb).portid,
					nlh->nlmsg_seq, NFT_MSG_NEWCHAIN, 0,
					family, table, chain);
	if (err < 0)
		goto err_fill_chain_info;

	return nfnetlink_unicast(skb2, net, NETLINK_CB(skb).portid);

err_fill_chain_info:
	kfree_skb(skb2);
	return err;
}

static const struct nla_policy nft_counter_policy[NFTA_COUNTER_MAX + 1] = {
	[NFTA_COUNTER_PACKETS]	= { .type = NLA_U64 },
	[NFTA_COUNTER_BYTES]	= { .type = NLA_U64 },
};

static struct nft_stats __percpu *nft_stats_alloc(const struct nlattr *attr)
{
	struct nlattr *tb[NFTA_COUNTER_MAX+1];
	struct nft_stats __percpu *newstats;
	struct nft_stats *stats;
	int err;

	err = nla_parse_nested(tb, NFTA_COUNTER_MAX, attr, nft_counter_policy,
			       NULL);
	if (err < 0)
		return ERR_PTR(err);

	if (!tb[NFTA_COUNTER_BYTES] || !tb[NFTA_COUNTER_PACKETS])
		return ERR_PTR(-EINVAL);

	newstats = netdev_alloc_pcpu_stats(struct nft_stats);
	if (newstats == NULL)
		return ERR_PTR(-ENOMEM);

	/* Restore old counters on this cpu, no problem. Per-cpu statistics
	 * are not exposed to userspace.
	 */
	preempt_disable();
	stats = this_cpu_ptr(newstats);
	stats->bytes = be64_to_cpu(nla_get_be64(tb[NFTA_COUNTER_BYTES]));
	stats->pkts = be64_to_cpu(nla_get_be64(tb[NFTA_COUNTER_PACKETS]));
	preempt_enable();

	return newstats;
}

static void nft_chain_stats_replace(struct net *net,
				    struct nft_base_chain *chain,
				    struct nft_stats __percpu *newstats)
{
	struct nft_stats __percpu *oldstats;

	if (newstats == NULL)
		return;

	if (rcu_access_pointer(chain->stats)) {
		oldstats = rcu_dereference_protected(chain->stats,
					lockdep_commit_lock_is_held(net));
		rcu_assign_pointer(chain->stats, newstats);
		synchronize_rcu();
		free_percpu(oldstats);
	} else {
		rcu_assign_pointer(chain->stats, newstats);
		static_branch_inc(&nft_counters_enabled);
	}
}

static void nf_tables_chain_free_chain_rules(struct nft_chain *chain)
{
	struct nft_rule **g0 = rcu_dereference_raw(chain->rules_gen_0);
	struct nft_rule **g1 = rcu_dereference_raw(chain->rules_gen_1);

	if (g0 != g1)
		kvfree(g1);
	kvfree(g0);

	/* should be NULL either via abort or via successful commit */
	WARN_ON_ONCE(chain->rules_next);
	kvfree(chain->rules_next);
}

static void nf_tables_chain_destroy(struct nft_ctx *ctx)
{
	struct nft_chain *chain = ctx->chain;

	if (WARN_ON(chain->use > 0))
		return;

	/* no concurrent access possible anymore */
	nf_tables_chain_free_chain_rules(chain);

	if (nft_is_base_chain(chain)) {
		struct nft_base_chain *basechain = nft_base_chain(chain);

		module_put(basechain->type->owner);
		if (rcu_access_pointer(basechain->stats)) {
			static_branch_dec(&nft_counters_enabled);
			free_percpu(rcu_dereference_raw(basechain->stats));
		}
		kfree(chain->name);
		kfree(basechain);
	} else {
		kfree(chain->name);
		kfree(chain);
	}
}

struct nft_chain_hook {
	u32				num;
	s32				priority;
	const struct nft_chain_type	*type;
	struct net_device		*dev;
};

static int nft_chain_parse_hook(struct net *net,
				const struct nlattr * const nla[],
				struct nft_chain_hook *hook, u8 family,
				bool autoload)
{
	struct nftables_pernet *nft_net = net_generic(net, nf_tables_net_id);
	struct nlattr *ha[NFTA_HOOK_MAX + 1];
	const struct nft_chain_type *type;
	struct net_device *dev;
	int err;

	lockdep_assert_held(&nft_net->commit_mutex);
	lockdep_nfnl_nft_mutex_not_held();

	err = nla_parse_nested(ha, NFTA_HOOK_MAX, nla[NFTA_CHAIN_HOOK],
			       nft_hook_policy, NULL);
	if (err < 0)
		return err;

	if (ha[NFTA_HOOK_HOOKNUM] == NULL ||
	    ha[NFTA_HOOK_PRIORITY] == NULL)
		return -EINVAL;

	hook->num = ntohl(nla_get_be32(ha[NFTA_HOOK_HOOKNUM]));
	hook->priority = ntohl(nla_get_be32(ha[NFTA_HOOK_PRIORITY]));

	type = __nft_chain_type_get(family, NFT_CHAIN_T_DEFAULT);
	if (!type)
		return -EOPNOTSUPP;

	if (nla[NFTA_CHAIN_TYPE]) {
		type = nf_tables_chain_type_lookup(net, nla[NFTA_CHAIN_TYPE],
						   family, autoload);
		if (IS_ERR(type))
			return PTR_ERR(type);
	}
	if (hook->num > NF_MAX_HOOKS || !(type->hook_mask & (1 << hook->num)))
		return -EOPNOTSUPP;

	if (type->type == NFT_CHAIN_T_NAT &&
	    hook->priority <= NF_IP_PRI_CONNTRACK)
		return -EOPNOTSUPP;

	if (!try_module_get(type->owner))
		return -ENOENT;

	hook->type = type;

	hook->dev = NULL;
	if (family == NFPROTO_NETDEV) {
		char ifname[IFNAMSIZ];

		if (!ha[NFTA_HOOK_DEV]) {
			module_put(type->owner);
			return -EOPNOTSUPP;
		}

		nla_strlcpy(ifname, ha[NFTA_HOOK_DEV], IFNAMSIZ);
		dev = __dev_get_by_name(net, ifname);
		if (!dev) {
			module_put(type->owner);
			return -ENOENT;
		}
		hook->dev = dev;
	} else if (ha[NFTA_HOOK_DEV]) {
		module_put(type->owner);
		return -EOPNOTSUPP;
	}

	return 0;
}

static void nft_chain_release_hook(struct nft_chain_hook *hook)
{
	module_put(hook->type->owner);
}

struct nft_rules_old {
	struct rcu_head h;
	struct nft_rule **start;
};

static struct nft_rule **nf_tables_chain_alloc_rules(const struct nft_chain *chain,
						     unsigned int alloc)
{
	if (alloc > INT_MAX)
		return NULL;

	alloc += 1;	/* NULL, ends rules */
	if (sizeof(struct nft_rule *) > INT_MAX / alloc)
		return NULL;

	alloc *= sizeof(struct nft_rule *);
	alloc += sizeof(struct nft_rules_old);

	return kvmalloc(alloc, GFP_KERNEL);
}

static int nf_tables_addchain(struct nft_ctx *ctx, u8 family, u8 genmask,
			      u8 policy)
{
	const struct nlattr * const *nla = ctx->nla;
	struct nft_table *table = ctx->table;
	struct nft_base_chain *basechain;
	struct nft_stats __percpu *stats;
	struct net *net = ctx->net;
	struct nft_chain *chain;
	struct nft_rule **rules;
	int err;

	if (nla[NFTA_CHAIN_HOOK]) {
		struct nft_chain_hook hook;
		struct nf_hook_ops *ops;

		if (table->flags & __NFT_TABLE_F_UPDATE)
			return -EINVAL;

		err = nft_chain_parse_hook(net, nla, &hook, family, true);
		if (err < 0)
			return err;

		basechain = kzalloc(sizeof(*basechain), GFP_KERNEL);
		if (basechain == NULL) {
			nft_chain_release_hook(&hook);
			return -ENOMEM;
		}

		if (hook.dev != NULL)
			strncpy(basechain->dev_name, hook.dev->name, IFNAMSIZ);

		if (nla[NFTA_CHAIN_COUNTERS]) {
			stats = nft_stats_alloc(nla[NFTA_CHAIN_COUNTERS]);
			if (IS_ERR(stats)) {
				nft_chain_release_hook(&hook);
				kfree(basechain);
				return PTR_ERR(stats);
			}
			rcu_assign_pointer(basechain->stats, stats);
			static_branch_inc(&nft_counters_enabled);
		}

		basechain->type = hook.type;
		chain = &basechain->chain;

		ops		= &basechain->ops;
		ops->pf		= family;
		ops->hooknum	= hook.num;
		ops->priority	= hook.priority;
		ops->priv	= chain;
		ops->hook	= hook.type->hooks[ops->hooknum];
		ops->dev	= hook.dev;

		chain->flags |= NFT_BASE_CHAIN;
		basechain->policy = policy;
	} else {
		chain = kzalloc(sizeof(*chain), GFP_KERNEL);
		if (chain == NULL)
			return -ENOMEM;
	}
	ctx->chain = chain;

	INIT_LIST_HEAD(&chain->rules);
	chain->handle = nf_tables_alloc_handle(table);
	chain->table = table;
	chain->name = nla_strdup(nla[NFTA_CHAIN_NAME], GFP_KERNEL);
	if (!chain->name) {
		err = -ENOMEM;
		goto err1;
	}

	rules = nf_tables_chain_alloc_rules(chain, 0);
	if (!rules) {
		err = -ENOMEM;
		goto err1;
	}

	*rules = NULL;
	rcu_assign_pointer(chain->rules_gen_0, rules);
	rcu_assign_pointer(chain->rules_gen_1, rules);

	err = nf_tables_register_hook(net, table, chain);
	if (err < 0)
		goto err1;

	if (!nft_use_inc(&table->use)) {
		err = -EMFILE;
		goto err_use;
	}

	err = rhltable_insert_key(&table->chains_ht, chain->name,
				  &chain->rhlhead, nft_chain_ht_params);
	if (err)
		goto err2;

	err = nft_trans_chain_add(ctx, NFT_MSG_NEWCHAIN);
	if (err < 0) {
		rhltable_remove(&table->chains_ht, &chain->rhlhead,
				nft_chain_ht_params);
		goto err2;
	}

	list_add_tail_rcu(&chain->list, &table->chains);

	return 0;
err2:
	nft_use_dec_restore(&table->use);
err_use:
	nf_tables_unregister_hook(net, table, chain);
err1:
	nf_tables_chain_destroy(ctx);

	return err;
}

static int nf_tables_updchain(struct nft_ctx *ctx, u8 genmask, u8 policy)
{
	const struct nlattr * const *nla = ctx->nla;
	struct nft_table *table = ctx->table;
	struct nft_chain *chain = ctx->chain;
	struct nft_base_chain *basechain;
	struct nft_stats *stats = NULL;
	struct nft_chain_hook hook;
	struct nf_hook_ops *ops;
	struct nft_trans *trans;
	int err;

	if (nla[NFTA_CHAIN_HOOK]) {
		if (!nft_is_base_chain(chain))
			return -EBUSY;

		err = nft_chain_parse_hook(ctx->net, nla, &hook, ctx->family,
					   false);
		if (err < 0)
			return err;

		basechain = nft_base_chain(chain);
		if (basechain->type != hook.type) {
			nft_chain_release_hook(&hook);
			return -EBUSY;
		}

		ops = &basechain->ops;
		if (ops->hooknum != hook.num ||
		    ops->priority != hook.priority ||
		    ops->dev != hook.dev) {
			nft_chain_release_hook(&hook);
			return -EBUSY;
		}
		nft_chain_release_hook(&hook);
	}

	if (nla[NFTA_CHAIN_HANDLE] &&
	    nla[NFTA_CHAIN_NAME]) {
		struct nft_chain *chain2;

		chain2 = nft_chain_lookup(ctx->net, table,
					  nla[NFTA_CHAIN_NAME], genmask);
		if (!IS_ERR(chain2))
			return -EEXIST;
	}

	if (nla[NFTA_CHAIN_COUNTERS]) {
		if (!nft_is_base_chain(chain))
			return -EOPNOTSUPP;

		stats = nft_stats_alloc(nla[NFTA_CHAIN_COUNTERS]);
		if (IS_ERR(stats))
			return PTR_ERR(stats);
	}

	err = -ENOMEM;
	trans = nft_trans_alloc(ctx, NFT_MSG_NEWCHAIN,
				sizeof(struct nft_trans_chain));
	if (trans == NULL)
		goto err;

	nft_trans_chain_stats(trans) = stats;
	nft_trans_chain_update(trans) = true;

	if (nla[NFTA_CHAIN_POLICY])
		nft_trans_chain_policy(trans) = policy;
	else
		nft_trans_chain_policy(trans) = -1;

	if (nla[NFTA_CHAIN_HANDLE] &&
	    nla[NFTA_CHAIN_NAME]) {
		struct nftables_pernet *nft_net = net_generic(ctx->net, nf_tables_net_id);
		struct nft_trans *tmp;
		char *name;

		err = -ENOMEM;
		name = nla_strdup(nla[NFTA_CHAIN_NAME], GFP_KERNEL);
		if (!name)
			goto err;

		err = -EEXIST;
		list_for_each_entry(tmp, &nft_net->commit_list, list) {
			if (tmp->msg_type == NFT_MSG_NEWCHAIN &&
			    tmp->ctx.table == table &&
			    nft_trans_chain_update(tmp) &&
			    nft_trans_chain_name(tmp) &&
			    strcmp(name, nft_trans_chain_name(tmp)) == 0) {
				kfree(name);
				goto err;
			}
		}

		nft_trans_chain_name(trans) = name;
	}
	nft_trans_commit_list_add_tail(ctx->net, trans);

	return 0;
err:
	free_percpu(stats);
	kfree(trans);
	return err;
}

static int nf_tables_newchain(struct net *net, struct sock *nlsk,
			      struct sk_buff *skb, const struct nlmsghdr *nlh,
			      const struct nlattr * const nla[],
			      struct netlink_ext_ack *extack)
{
	struct nftables_pernet *nft_net = net_generic(net, nf_tables_net_id);
	const struct nfgenmsg *nfmsg = nlmsg_data(nlh);
	u8 genmask = nft_genmask_next(net);
	int family = nfmsg->nfgen_family;
	const struct nlattr *attr;
	struct nft_table *table;
	struct nft_chain *chain;
	u8 policy = NF_ACCEPT;
	struct nft_ctx ctx;
	u64 handle = 0;

	lockdep_assert_held(&nft_net->commit_mutex);

	table = nft_table_lookup(net, nla[NFTA_CHAIN_TABLE], family, genmask);
	if (IS_ERR(table)) {
		NL_SET_BAD_ATTR(extack, nla[NFTA_CHAIN_TABLE]);
		return PTR_ERR(table);
	}

	chain = NULL;
	attr = nla[NFTA_CHAIN_NAME];

	if (nla[NFTA_CHAIN_HANDLE]) {
		handle = be64_to_cpu(nla_get_be64(nla[NFTA_CHAIN_HANDLE]));
		chain = nft_chain_lookup_byhandle(table, handle, genmask);
		if (IS_ERR(chain)) {
			NL_SET_BAD_ATTR(extack, nla[NFTA_CHAIN_HANDLE]);
			return PTR_ERR(chain);
		}
		attr = nla[NFTA_CHAIN_HANDLE];
	} else {
		chain = nft_chain_lookup(net, table, attr, genmask);
		if (IS_ERR(chain)) {
			if (PTR_ERR(chain) != -ENOENT) {
				NL_SET_BAD_ATTR(extack, attr);
				return PTR_ERR(chain);
			}
			chain = NULL;
		}
	}

	if (nla[NFTA_CHAIN_POLICY]) {
		if (chain != NULL &&
		    !nft_is_base_chain(chain)) {
			NL_SET_BAD_ATTR(extack, nla[NFTA_CHAIN_POLICY]);
			return -EOPNOTSUPP;
		}

		if (chain == NULL &&
		    nla[NFTA_CHAIN_HOOK] == NULL) {
			NL_SET_BAD_ATTR(extack, nla[NFTA_CHAIN_POLICY]);
			return -EOPNOTSUPP;
		}

		policy = ntohl(nla_get_be32(nla[NFTA_CHAIN_POLICY]));
		switch (policy) {
		case NF_DROP:
		case NF_ACCEPT:
			break;
		default:
			return -EINVAL;
		}
	}

	nft_ctx_init(&ctx, net, skb, nlh, family, table, chain, nla);

	if (chain != NULL) {
		if (nlh->nlmsg_flags & NLM_F_EXCL) {
			NL_SET_BAD_ATTR(extack, attr);
			return -EEXIST;
		}
		if (nlh->nlmsg_flags & NLM_F_REPLACE)
			return -EOPNOTSUPP;

		return nf_tables_updchain(&ctx, genmask, policy);
	}

	return nf_tables_addchain(&ctx, family, genmask, policy);
}

static int nf_tables_delchain(struct net *net, struct sock *nlsk,
			      struct sk_buff *skb, const struct nlmsghdr *nlh,
			      const struct nlattr * const nla[],
			      struct netlink_ext_ack *extack)
{
	const struct nfgenmsg *nfmsg = nlmsg_data(nlh);
	u8 genmask = nft_genmask_next(net);
	int family = nfmsg->nfgen_family;
	const struct nlattr *attr;
	struct nft_table *table;
	struct nft_chain *chain;
	struct nft_rule *rule;
	struct nft_ctx ctx;
	u64 handle;
	u32 use;
	int err;

	table = nft_table_lookup(net, nla[NFTA_CHAIN_TABLE], family, genmask);
	if (IS_ERR(table)) {
		NL_SET_BAD_ATTR(extack, nla[NFTA_CHAIN_TABLE]);
		return PTR_ERR(table);
	}

	if (nla[NFTA_CHAIN_HANDLE]) {
		attr = nla[NFTA_CHAIN_HANDLE];
		handle = be64_to_cpu(nla_get_be64(attr));
		chain = nft_chain_lookup_byhandle(table, handle, genmask);
	} else {
		attr = nla[NFTA_CHAIN_NAME];
		chain = nft_chain_lookup(net, table, attr, genmask);
	}
	if (IS_ERR(chain)) {
		NL_SET_BAD_ATTR(extack, attr);
		return PTR_ERR(chain);
	}

	if (nlh->nlmsg_flags & NLM_F_NONREC &&
	    chain->use > 0)
		return -EBUSY;

	nft_ctx_init(&ctx, net, skb, nlh, family, table, chain, nla);

	use = chain->use;
	list_for_each_entry(rule, &chain->rules, list) {
		if (!nft_is_active_next(net, rule))
			continue;
		use--;

		err = nft_delrule(&ctx, rule);
		if (err < 0)
			return err;
	}

	/* There are rules and elements that are still holding references to us,
	 * we cannot do a recursive removal in this case.
	 */
	if (use > 0) {
		NL_SET_BAD_ATTR(extack, attr);
		return -EBUSY;
	}

	return nft_delchain(&ctx);
}

/*
 * Expressions
 */

/**
 *	nft_register_expr - register nf_tables expr type
 *	@ops: expr type
 *
 *	Registers the expr type for use with nf_tables. Returns zero on
 *	success or a negative errno code otherwise.
 */
int nft_register_expr(struct nft_expr_type *type)
{
	nfnl_lock(NFNL_SUBSYS_NFTABLES);
	if (type->family == NFPROTO_UNSPEC)
		list_add_tail_rcu(&type->list, &nf_tables_expressions);
	else
		list_add_rcu(&type->list, &nf_tables_expressions);
	nfnl_unlock(NFNL_SUBSYS_NFTABLES);
	return 0;
}
EXPORT_SYMBOL_GPL(nft_register_expr);

/**
 *	nft_unregister_expr - unregister nf_tables expr type
 *	@ops: expr type
 *
 * 	Unregisters the expr typefor use with nf_tables.
 */
void nft_unregister_expr(struct nft_expr_type *type)
{
	nfnl_lock(NFNL_SUBSYS_NFTABLES);
	list_del_rcu(&type->list);
	nfnl_unlock(NFNL_SUBSYS_NFTABLES);
}
EXPORT_SYMBOL_GPL(nft_unregister_expr);

static const struct nft_expr_type *__nft_expr_type_get(u8 family,
						       struct nlattr *nla)
{
	const struct nft_expr_type *type, *candidate = NULL;

	list_for_each_entry_rcu(type, &nf_tables_expressions, list) {
		if (!nla_strcmp(nla, type->name)) {
			if (!type->family && !candidate)
				candidate = type;
			else if (type->family == family)
				candidate = type;
		}
	}
	return candidate;
}

static const struct nft_expr_type *nft_expr_type_get(struct net *net,
						     u8 family,
						     struct nlattr *nla)
{
	const struct nft_expr_type *type;

	if (nla == NULL)
		return ERR_PTR(-EINVAL);

	rcu_read_lock();
	type = __nft_expr_type_get(family, nla);
	if (type != NULL && try_module_get(type->owner)) {
		rcu_read_unlock();
		return type;
	}
	rcu_read_unlock();

	lockdep_nfnl_nft_mutex_not_held();
#ifdef CONFIG_MODULES
	if (type == NULL) {
		nft_request_module(net, "nft-expr-%u-%.*s", family,
				   nla_len(nla), (char *)nla_data(nla));
		if (__nft_expr_type_get(family, nla))
			return ERR_PTR(-EAGAIN);

		nft_request_module(net, "nft-expr-%.*s",
				   nla_len(nla), (char *)nla_data(nla));
		if (__nft_expr_type_get(family, nla))
			return ERR_PTR(-EAGAIN);
	}
#endif
	return ERR_PTR(-ENOENT);
}

static const struct nla_policy nft_expr_policy[NFTA_EXPR_MAX + 1] = {
	[NFTA_EXPR_NAME]	= { .type = NLA_STRING,
				    .len = NFT_MODULE_AUTOLOAD_LIMIT },
	[NFTA_EXPR_DATA]	= { .type = NLA_NESTED },
};

static int nf_tables_fill_expr_info(struct sk_buff *skb,
				    const struct nft_expr *expr)
{
	if (nla_put_string(skb, NFTA_EXPR_NAME, expr->ops->type->name))
		goto nla_put_failure;

	if (expr->ops->dump) {
		struct nlattr *data = nla_nest_start(skb, NFTA_EXPR_DATA);
		if (data == NULL)
			goto nla_put_failure;
		if (expr->ops->dump(skb, expr) < 0)
			goto nla_put_failure;
		nla_nest_end(skb, data);
	}

	return skb->len;

nla_put_failure:
	return -1;
};

int nft_expr_dump(struct sk_buff *skb, unsigned int attr,
		  const struct nft_expr *expr)
{
	struct nlattr *nest;

	nest = nla_nest_start(skb, attr);
	if (!nest)
		goto nla_put_failure;
	if (nf_tables_fill_expr_info(skb, expr) < 0)
		goto nla_put_failure;
	nla_nest_end(skb, nest);
	return 0;

nla_put_failure:
	return -1;
}

struct nft_expr_info {
	const struct nft_expr_ops	*ops;
	struct nlattr			*tb[NFT_EXPR_MAXATTR + 1];
};

static int nf_tables_expr_parse(const struct nft_ctx *ctx,
				const struct nlattr *nla,
				struct nft_expr_info *info)
{
	const struct nft_expr_type *type;
	const struct nft_expr_ops *ops;
	struct nlattr *tb[NFTA_EXPR_MAX + 1];
	int err;

	err = nla_parse_nested(tb, NFTA_EXPR_MAX, nla, nft_expr_policy, NULL);
	if (err < 0)
		return err;

	type = nft_expr_type_get(ctx->net, ctx->family, tb[NFTA_EXPR_NAME]);
	if (IS_ERR(type))
		return PTR_ERR(type);

	if (tb[NFTA_EXPR_DATA]) {
		err = nla_parse_nested(info->tb, type->maxattr,
				       tb[NFTA_EXPR_DATA], type->policy, NULL);
		if (err < 0)
			goto err1;
	} else
		memset(info->tb, 0, sizeof(info->tb[0]) * (type->maxattr + 1));

	if (type->select_ops != NULL) {
		ops = type->select_ops(ctx,
				       (const struct nlattr * const *)info->tb);
		if (IS_ERR(ops)) {
			err = PTR_ERR(ops);
			goto err1;
		}
	} else
		ops = type->ops;

	info->ops = ops;
	return 0;

err1:
	module_put(type->owner);
	return err;
}

static int nf_tables_newexpr(const struct nft_ctx *ctx,
			     const struct nft_expr_info *info,
			     struct nft_expr *expr)
{
	const struct nft_expr_ops *ops = info->ops;
	int err;

	expr->ops = ops;
	if (ops->init) {
		err = ops->init(ctx, expr, (const struct nlattr **)info->tb);
		if (err < 0)
			goto err1;
	}

	return 0;
err1:
	expr->ops = NULL;
	return err;
}

static void nf_tables_expr_destroy(const struct nft_ctx *ctx,
				   struct nft_expr *expr)
{
	const struct nft_expr_type *type = expr->ops->type;

	if (expr->ops->destroy)
		expr->ops->destroy(ctx, expr);
	module_put(type->owner);
}

struct nft_expr *nft_expr_init(const struct nft_ctx *ctx,
			       const struct nlattr *nla)
{
	struct nft_expr_info info;
	struct nft_expr *expr;
	struct module *owner;
	int err;

	err = nf_tables_expr_parse(ctx, nla, &info);
	if (err < 0)
		goto err_expr_parse;

	err = -EOPNOTSUPP;
	if (!(info.ops->type->flags & NFT_EXPR_STATEFUL))
		goto err_expr_stateful;

	err = -ENOMEM;
	expr = kzalloc(info.ops->size, GFP_KERNEL);
	if (expr == NULL)
		goto err_expr_stateful;

	err = nf_tables_newexpr(ctx, &info, expr);
	if (err < 0)
		goto err_expr_new;

	return expr;
err_expr_new:
	kfree(expr);
err_expr_stateful:
	owner = info.ops->type->owner;
	if (info.ops->type->release_ops)
		info.ops->type->release_ops(info.ops);

	module_put(owner);
err_expr_parse:
	return ERR_PTR(err);
}

void nft_expr_destroy(const struct nft_ctx *ctx, struct nft_expr *expr)
{
	nf_tables_expr_destroy(ctx, expr);
	kfree(expr);
}

/*
 * Rules
 */

static struct nft_rule *__nft_rule_lookup(const struct nft_chain *chain,
					  u64 handle)
{
	struct nft_rule *rule;

	// FIXME: this sucks
	list_for_each_entry_rcu(rule, &chain->rules, list) {
		if (handle == rule->handle)
			return rule;
	}

	return ERR_PTR(-ENOENT);
}

static struct nft_rule *nft_rule_lookup(const struct nft_chain *chain,
					const struct nlattr *nla)
{
	if (nla == NULL)
		return ERR_PTR(-EINVAL);

	return __nft_rule_lookup(chain, be64_to_cpu(nla_get_be64(nla)));
}

static const struct nla_policy nft_rule_policy[NFTA_RULE_MAX + 1] = {
	[NFTA_RULE_TABLE]	= { .type = NLA_STRING,
				    .len = NFT_TABLE_MAXNAMELEN - 1 },
	[NFTA_RULE_CHAIN]	= { .type = NLA_STRING,
				    .len = NFT_CHAIN_MAXNAMELEN - 1 },
	[NFTA_RULE_HANDLE]	= { .type = NLA_U64 },
	[NFTA_RULE_EXPRESSIONS]	= { .type = NLA_NESTED },
	[NFTA_RULE_COMPAT]	= { .type = NLA_NESTED },
	[NFTA_RULE_POSITION]	= { .type = NLA_U64 },
	[NFTA_RULE_USERDATA]	= { .type = NLA_BINARY,
				    .len = NFT_USERDATA_MAXLEN },
	[NFTA_RULE_ID]		= { .type = NLA_U32 },
};

static int nf_tables_fill_rule_info(struct sk_buff *skb, struct net *net,
				    u32 portid, u32 seq, int event,
				    u32 flags, int family,
				    const struct nft_table *table,
				    const struct nft_chain *chain,
				    const struct nft_rule *rule)
{
	struct nlmsghdr *nlh;
	const struct nft_expr *expr, *next;
	struct nlattr *list;
	const struct nft_rule *prule;
	u16 type = nfnl_msg_type(NFNL_SUBSYS_NFTABLES, event);

	nlh = nfnl_msg_put(skb, portid, seq, type, flags, family, NFNETLINK_V0,
			   nft_base_seq(net));
	if (!nlh)
		goto nla_put_failure;

	if (nla_put_string(skb, NFTA_RULE_TABLE, table->name))
		goto nla_put_failure;
	if (nla_put_string(skb, NFTA_RULE_CHAIN, chain->name))
		goto nla_put_failure;
	if (nla_put_be64(skb, NFTA_RULE_HANDLE, cpu_to_be64(rule->handle),
			 NFTA_RULE_PAD))
		goto nla_put_failure;

	if ((event != NFT_MSG_DELRULE) && (rule->list.prev != &chain->rules)) {
		prule = list_prev_entry(rule, list);
		if (nla_put_be64(skb, NFTA_RULE_POSITION,
				 cpu_to_be64(prule->handle),
				 NFTA_RULE_PAD))
			goto nla_put_failure;
	}

	list = nla_nest_start(skb, NFTA_RULE_EXPRESSIONS);
	if (list == NULL)
		goto nla_put_failure;
	nft_rule_for_each_expr(expr, next, rule) {
		if (nft_expr_dump(skb, NFTA_LIST_ELEM, expr) < 0)
			goto nla_put_failure;
	}
	nla_nest_end(skb, list);

	if (rule->udata) {
		struct nft_userdata *udata = nft_userdata(rule);
		if (nla_put(skb, NFTA_RULE_USERDATA, udata->len + 1,
			    udata->data) < 0)
			goto nla_put_failure;
	}

	nlmsg_end(skb, nlh);
	return 0;

nla_put_failure:
	nlmsg_trim(skb, nlh);
	return -1;
}

static void nf_tables_rule_notify(const struct nft_ctx *ctx,
				  const struct nft_rule *rule, int event)
{
	struct sk_buff *skb;
	int err;

	if (!ctx->report &&
	    !nfnetlink_has_listeners(ctx->net, NFNLGRP_NFTABLES))
		return;

	skb = nlmsg_new(NLMSG_GOODSIZE, GFP_KERNEL);
	if (skb == NULL)
		goto err;

	err = nf_tables_fill_rule_info(skb, ctx->net, ctx->portid, ctx->seq,
				       event, 0, ctx->family, ctx->table,
				       ctx->chain, rule);
	if (err < 0) {
		kfree_skb(skb);
		goto err;
	}

	nfnetlink_send(skb, ctx->net, ctx->portid, NFNLGRP_NFTABLES,
		       ctx->report, GFP_KERNEL);
	return;
err:
	nfnetlink_set_err(ctx->net, ctx->portid, NFNLGRP_NFTABLES, -ENOBUFS);
}

struct nft_rule_dump_ctx {
	char *table;
	char *chain;
};

static int nf_tables_dump_rules(struct sk_buff *skb,
				struct netlink_callback *cb)
{
	const struct nfgenmsg *nfmsg = nlmsg_data(cb->nlh);
	const struct nft_rule_dump_ctx *ctx = cb->data;
	const struct nft_table *table;
	const struct nft_chain *chain;
	const struct nft_rule *rule;
	unsigned int idx = 0, s_idx = cb->args[0];
	struct net *net = sock_net(skb->sk);
	int family = nfmsg->nfgen_family;
	struct nftables_pernet *nft_net;

	rcu_read_lock();
	nft_net = net_generic(net, nf_tables_net_id);
	cb->seq = nft_net->base_seq;

	list_for_each_entry_rcu(table, &nft_net->tables, list) {
		if (family != NFPROTO_UNSPEC && family != table->family)
			continue;

		if (ctx && ctx->table && strcmp(ctx->table, table->name) != 0)
			continue;

		list_for_each_entry_rcu(chain, &table->chains, list) {
			if (ctx && ctx->chain &&
			    strcmp(ctx->chain, chain->name) != 0)
				continue;

			list_for_each_entry_rcu(rule, &chain->rules, list) {
				if (!nft_is_active(net, rule))
					goto cont;
				if (idx < s_idx)
					goto cont;
				if (idx > s_idx)
					memset(&cb->args[1], 0,
					       sizeof(cb->args) - sizeof(cb->args[0]));
				if (nf_tables_fill_rule_info(skb, net, NETLINK_CB(cb->skb).portid,
							      cb->nlh->nlmsg_seq,
							      NFT_MSG_NEWRULE,
							      NLM_F_MULTI | NLM_F_APPEND,
							      table->family,
							      table, chain, rule) < 0)
					goto done;

				nl_dump_check_consistent(cb, nlmsg_hdr(skb));
cont:
				idx++;
			}
		}
	}
done:
	rcu_read_unlock();

	cb->args[0] = idx;
	return skb->len;
}

static int nf_tables_dump_rules_start(struct netlink_callback *cb)
{
	const struct nlattr * const *nla = cb->data;
	struct nft_rule_dump_ctx *ctx = NULL;

	if (nla[NFTA_RULE_TABLE] || nla[NFTA_RULE_CHAIN]) {
		ctx = kzalloc(sizeof(*ctx), GFP_ATOMIC);
		if (!ctx)
			return -ENOMEM;

		if (nla[NFTA_RULE_TABLE]) {
			ctx->table = nla_strdup(nla[NFTA_RULE_TABLE],
							GFP_ATOMIC);
			if (!ctx->table) {
				kfree(ctx);
				return -ENOMEM;
			}
		}
		if (nla[NFTA_RULE_CHAIN]) {
			ctx->chain = nla_strdup(nla[NFTA_RULE_CHAIN],
						GFP_ATOMIC);
			if (!ctx->chain) {
				kfree(ctx->table);
				kfree(ctx);
				return -ENOMEM;
			}
		}
	}

	cb->data = ctx;
	return 0;
}

static int nf_tables_dump_rules_done(struct netlink_callback *cb)
{
	struct nft_rule_dump_ctx *ctx = cb->data;

	if (ctx) {
		kfree(ctx->table);
		kfree(ctx->chain);
		kfree(ctx);
	}
	return 0;
}

/* called with rcu_read_lock held */
static int nf_tables_getrule(struct net *net, struct sock *nlsk,
			     struct sk_buff *skb, const struct nlmsghdr *nlh,
			     const struct nlattr * const nla[],
			     struct netlink_ext_ack *extack)
{
	const struct nfgenmsg *nfmsg = nlmsg_data(nlh);
	u8 genmask = nft_genmask_cur(net);
	const struct nft_chain *chain;
	const struct nft_rule *rule;
	struct nft_table *table;
	struct sk_buff *skb2;
	int family = nfmsg->nfgen_family;
	int err;

	if (nlh->nlmsg_flags & NLM_F_DUMP) {
		struct netlink_dump_control c = {
			.start= nf_tables_dump_rules_start,
			.dump = nf_tables_dump_rules,
			.done = nf_tables_dump_rules_done,
			.module = THIS_MODULE,
			.data = (void *)nla,
		};

		return nft_netlink_dump_start_rcu(nlsk, skb, nlh, &c);
	}

	table = nft_table_lookup(net, nla[NFTA_RULE_TABLE], family, genmask);
	if (IS_ERR(table)) {
		NL_SET_BAD_ATTR(extack, nla[NFTA_RULE_TABLE]);
		return PTR_ERR(table);
	}

	chain = nft_chain_lookup(net, table, nla[NFTA_RULE_CHAIN], genmask);
	if (IS_ERR(chain)) {
		NL_SET_BAD_ATTR(extack, nla[NFTA_RULE_CHAIN]);
		return PTR_ERR(chain);
	}

	rule = nft_rule_lookup(chain, nla[NFTA_RULE_HANDLE]);
	if (IS_ERR(rule)) {
		NL_SET_BAD_ATTR(extack, nla[NFTA_RULE_HANDLE]);
		return PTR_ERR(rule);
	}

	skb2 = alloc_skb(NLMSG_GOODSIZE, GFP_ATOMIC);
	if (!skb2)
		return -ENOMEM;

	err = nf_tables_fill_rule_info(skb2, net, NETLINK_CB(skb).portid,
				       nlh->nlmsg_seq, NFT_MSG_NEWRULE, 0,
				       family, table, chain, rule);
	if (err < 0)
		goto err_fill_rule_info;

	return nfnetlink_unicast(skb2, net, NETLINK_CB(skb).portid);

err_fill_rule_info:
	kfree_skb(skb2);
	return err;
}

static void nf_tables_rule_destroy(const struct nft_ctx *ctx,
				   struct nft_rule *rule)
{
	struct nft_expr *expr, *next;

	/*
	 * Careful: some expressions might not be initialized in case this
	 * is called on error from nf_tables_newrule().
	 */
	expr = nft_expr_first(rule);
	while (expr != nft_expr_last(rule) && expr->ops) {
		next = nft_expr_next(expr);
		nf_tables_expr_destroy(ctx, expr);
		expr = next;
	}
	kfree(rule);
}

static void nf_tables_rule_release(const struct nft_ctx *ctx,
				   struct nft_rule *rule)
{
	nft_rule_expr_deactivate(ctx, rule, NFT_TRANS_RELEASE);
	nf_tables_rule_destroy(ctx, rule);
}

/** nft_chain_validate - loop detection and hook validation
 *
 * @ctx: context containing call depth and base chain
 * @chain: chain to validate
 *
 * Walk through the rules of the given chain and chase all jumps/gotos
 * and set lookups until either the jump limit is hit or all reachable
 * chains have been validated.
 */
int nft_chain_validate(const struct nft_ctx *ctx, const struct nft_chain *chain)
{
	struct nft_expr *expr, *last;
	const struct nft_data *data;
	struct nft_rule *rule;
	int err;

	if (ctx->level == NFT_JUMP_STACK_SIZE)
		return -EMLINK;

	list_for_each_entry(rule, &chain->rules, list) {
		if (!nft_is_active_next(ctx->net, rule))
			continue;

		nft_rule_for_each_expr(expr, last, rule) {
			if (!expr->ops->validate)
				continue;

			/* This may call nft_chain_validate() recursively,
			 * callers that do so must increment ctx->level.
			 */
			err = expr->ops->validate(ctx, expr, &data);
			if (err < 0)
				return err;
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(nft_chain_validate);

static int nft_table_validate(struct net *net, const struct nft_table *table)
{
	struct nft_chain *chain;
	struct nft_ctx ctx = {
		.net	= net,
		.family	= table->family,
	};
	int err;

	list_for_each_entry(chain, &table->chains, list) {
		if (!nft_is_base_chain(chain))
			continue;

		ctx.chain = chain;
		err = nft_chain_validate(&ctx, chain);
		if (err < 0)
			return err;

		cond_resched();
	}

	return 0;
}

#define NFT_RULE_MAXEXPRS	128

static int nf_tables_newrule(struct net *net, struct sock *nlsk,
			     struct sk_buff *skb, const struct nlmsghdr *nlh,
			     const struct nlattr * const nla[],
			     struct netlink_ext_ack *extack)
{
	struct nftables_pernet *nft_net = net_generic(net, nf_tables_net_id);
	const struct nfgenmsg *nfmsg = nlmsg_data(nlh);
	u8 genmask = nft_genmask_next(net);
	struct nft_expr_info *info = NULL;
	int family = nfmsg->nfgen_family;
	struct nft_table *table;
	struct nft_chain *chain;
	struct nft_rule *rule, *old_rule = NULL;
	struct nft_userdata *udata;
	struct nft_trans *trans = NULL;
	struct nft_expr *expr;
	struct nft_ctx ctx;
	struct nlattr *tmp;
	unsigned int size, i, n, ulen = 0, usize = 0;
	int err, rem;
	u64 handle, pos_handle;

	lockdep_assert_held(&nft_net->commit_mutex);

	table = nft_table_lookup(net, nla[NFTA_RULE_TABLE], family, genmask);
	if (IS_ERR(table)) {
		NL_SET_BAD_ATTR(extack, nla[NFTA_RULE_TABLE]);
		return PTR_ERR(table);
	}

	chain = nft_chain_lookup(net, table, nla[NFTA_RULE_CHAIN], genmask);
	if (IS_ERR(chain)) {
		NL_SET_BAD_ATTR(extack, nla[NFTA_RULE_CHAIN]);
		return PTR_ERR(chain);
	}

	if (nla[NFTA_RULE_HANDLE]) {
		handle = be64_to_cpu(nla_get_be64(nla[NFTA_RULE_HANDLE]));
		rule = __nft_rule_lookup(chain, handle);
		if (IS_ERR(rule)) {
			NL_SET_BAD_ATTR(extack, nla[NFTA_RULE_HANDLE]);
			return PTR_ERR(rule);
		}

		if (nlh->nlmsg_flags & NLM_F_EXCL) {
			NL_SET_BAD_ATTR(extack, nla[NFTA_RULE_HANDLE]);
			return -EEXIST;
		}
		if (nlh->nlmsg_flags & NLM_F_REPLACE)
			old_rule = rule;
		else
			return -EOPNOTSUPP;
	} else {
		if (!(nlh->nlmsg_flags & NLM_F_CREATE) ||
		    nlh->nlmsg_flags & NLM_F_REPLACE)
			return -EINVAL;
		handle = nf_tables_alloc_handle(table);

		if (nla[NFTA_RULE_POSITION]) {
			pos_handle = be64_to_cpu(nla_get_be64(nla[NFTA_RULE_POSITION]));
			old_rule = __nft_rule_lookup(chain, pos_handle);
			if (IS_ERR(old_rule)) {
				NL_SET_BAD_ATTR(extack, nla[NFTA_RULE_POSITION]);
				return PTR_ERR(old_rule);
			}
		}
	}

	nft_ctx_init(&ctx, net, skb, nlh, family, table, chain, nla);

	n = 0;
	size = 0;
	if (nla[NFTA_RULE_EXPRESSIONS]) {
		info = kvmalloc_array(NFT_RULE_MAXEXPRS,
				      sizeof(struct nft_expr_info),
				      GFP_KERNEL);
		if (!info)
			return -ENOMEM;

		nla_for_each_nested(tmp, nla[NFTA_RULE_EXPRESSIONS], rem) {
			err = -EINVAL;
			if (nla_type(tmp) != NFTA_LIST_ELEM)
				goto err1;
			if (n == NFT_RULE_MAXEXPRS)
				goto err1;
			err = nf_tables_expr_parse(&ctx, tmp, &info[n]);
			if (err < 0)
				goto err1;
			size += info[n].ops->size;
			n++;
		}
	}
	/* Check for overflow of dlen field */
	err = -EFBIG;
	if (size >= 1 << 12)
		goto err1;

	if (nla[NFTA_RULE_USERDATA]) {
		ulen = nla_len(nla[NFTA_RULE_USERDATA]);
		if (ulen > 0)
			usize = sizeof(struct nft_userdata) + ulen;
	}

	err = -ENOMEM;
	rule = kzalloc(sizeof(*rule) + size + usize, GFP_KERNEL);
	if (rule == NULL)
		goto err1;

	nft_activate_next(net, rule);

	rule->handle = handle;
	rule->dlen   = size;
	rule->udata  = ulen ? 1 : 0;

	if (ulen) {
		udata = nft_userdata(rule);
		udata->len = ulen - 1;
		nla_memcpy(udata->data, nla[NFTA_RULE_USERDATA], ulen);
	}

	expr = nft_expr_first(rule);
	for (i = 0; i < n; i++) {
		err = nf_tables_newexpr(&ctx, &info[i], expr);
		if (err < 0)
			goto err2;

		if (info[i].ops->validate)
			nft_validate_state_update(net, NFT_VALIDATE_NEED);

		info[i].ops = NULL;
		expr = nft_expr_next(expr);
	}

	if (!nft_use_inc(&chain->use)) {
		err = -EMFILE;
		goto err2;
	}

	if (nlh->nlmsg_flags & NLM_F_REPLACE) {
		trans = nft_trans_rule_add(&ctx, NFT_MSG_NEWRULE, rule);
		if (trans == NULL) {
			err = -ENOMEM;
			goto err_destroy_flow_rule;
		}
		err = nft_delrule(&ctx, old_rule);
		if (err < 0) {
			nft_trans_destroy(trans);
			goto err_destroy_flow_rule;
		}

		list_add_tail_rcu(&rule->list, &old_rule->list);
	} else {
		if (nft_trans_rule_add(&ctx, NFT_MSG_NEWRULE, rule) == NULL) {
			err = -ENOMEM;
			goto err_destroy_flow_rule;
		}

		if (nlh->nlmsg_flags & NLM_F_APPEND) {
			if (old_rule)
				list_add_rcu(&rule->list, &old_rule->list);
			else
				list_add_tail_rcu(&rule->list, &chain->rules);
		 } else {
			if (old_rule)
				list_add_tail_rcu(&rule->list, &old_rule->list);
			else
				list_add_rcu(&rule->list, &chain->rules);
		}
	}
	kvfree(info);

	if (nft_net->validate_state == NFT_VALIDATE_DO)
		return nft_table_validate(net, table);

	return 0;

err_destroy_flow_rule:
	nft_use_dec_restore(&chain->use);
err2:
	nft_rule_expr_deactivate(&ctx, rule, NFT_TRANS_PREPARE_ERROR);
	nf_tables_rule_destroy(&ctx, rule);
err1:
	for (i = 0; i < n; i++) {
		if (info[i].ops) {
			module_put(info[i].ops->type->owner);
			if (info[i].ops->type->release_ops)
				info[i].ops->type->release_ops(info[i].ops);
		}
	}
	kvfree(info);
	return err;
}

static struct nft_rule *nft_rule_lookup_byid(const struct net *net,
					     const struct nft_chain *chain,
					     const struct nlattr *nla)
{
	struct nftables_pernet *nft_net = net_generic(net, nf_tables_net_id);
	u32 id = ntohl(nla_get_be32(nla));
	struct nft_trans *trans;

	list_for_each_entry(trans, &nft_net->commit_list, list) {
		struct nft_rule *rule = nft_trans_rule(trans);

		if (trans->msg_type == NFT_MSG_NEWRULE &&
		    trans->ctx.chain == chain &&
		    id == nft_trans_rule_id(trans))
			return rule;
	}
	return ERR_PTR(-ENOENT);
}

static int nf_tables_delrule(struct net *net, struct sock *nlsk,
			     struct sk_buff *skb, const struct nlmsghdr *nlh,
			     const struct nlattr * const nla[],
			     struct netlink_ext_ack *extack)
{
	const struct nfgenmsg *nfmsg = nlmsg_data(nlh);
	u8 genmask = nft_genmask_next(net);
	struct nft_table *table;
	struct nft_chain *chain = NULL;
	struct nft_rule *rule;
	int family = nfmsg->nfgen_family, err = 0;
	struct nft_ctx ctx;

	table = nft_table_lookup(net, nla[NFTA_RULE_TABLE], family, genmask);
	if (IS_ERR(table)) {
		NL_SET_BAD_ATTR(extack, nla[NFTA_RULE_TABLE]);
		return PTR_ERR(table);
	}

	if (nla[NFTA_RULE_CHAIN]) {
		chain = nft_chain_lookup(net, table, nla[NFTA_RULE_CHAIN],
					 genmask);
		if (IS_ERR(chain)) {
			NL_SET_BAD_ATTR(extack, nla[NFTA_RULE_CHAIN]);
			return PTR_ERR(chain);
		}
	}

	nft_ctx_init(&ctx, net, skb, nlh, family, table, chain, nla);

	if (chain) {
		if (nla[NFTA_RULE_HANDLE]) {
			rule = nft_rule_lookup(chain, nla[NFTA_RULE_HANDLE]);
			if (IS_ERR(rule)) {
				NL_SET_BAD_ATTR(extack, nla[NFTA_RULE_HANDLE]);
				return PTR_ERR(rule);
			}

			err = nft_delrule(&ctx, rule);
		} else if (nla[NFTA_RULE_ID]) {
			rule = nft_rule_lookup_byid(net, chain, nla[NFTA_RULE_ID]);
			if (IS_ERR(rule)) {
				NL_SET_BAD_ATTR(extack, nla[NFTA_RULE_ID]);
				return PTR_ERR(rule);
			}

			err = nft_delrule(&ctx, rule);
		} else {
			err = nft_delrule_by_chain(&ctx);
		}
	} else {
		list_for_each_entry(chain, &table->chains, list) {
			if (!nft_is_active_next(net, chain))
				continue;

			ctx.chain = chain;
			err = nft_delrule_by_chain(&ctx);
			if (err < 0)
				break;
		}
	}

	return err;
}

/*
 * Sets
 */

static LIST_HEAD(nf_tables_set_types);

int nft_register_set(struct nft_set_type *type)
{
	nfnl_lock(NFNL_SUBSYS_NFTABLES);
	list_add_tail_rcu(&type->list, &nf_tables_set_types);
	nfnl_unlock(NFNL_SUBSYS_NFTABLES);
	return 0;
}
EXPORT_SYMBOL_GPL(nft_register_set);

void nft_unregister_set(struct nft_set_type *type)
{
	nfnl_lock(NFNL_SUBSYS_NFTABLES);
	list_del_rcu(&type->list);
	nfnl_unlock(NFNL_SUBSYS_NFTABLES);
}
EXPORT_SYMBOL_GPL(nft_unregister_set);

#define NFT_SET_FEATURES	(NFT_SET_INTERVAL | NFT_SET_MAP | \
				 NFT_SET_TIMEOUT | NFT_SET_OBJECT | \
				 NFT_SET_EVAL)

static bool nft_set_ops_candidate(const struct nft_set_type *type, u32 flags)
{
	return (flags & type->features) == (flags & NFT_SET_FEATURES);
}

/*
 * Select a set implementation based on the data characteristics and the
 * given policy. The total memory use might not be known if no size is
 * given, in that case the amount of memory per element is used.
 */
static const struct nft_set_ops *
nft_select_set_ops(const struct nft_ctx *ctx,
		   const struct nlattr * const nla[],
		   const struct nft_set_desc *desc,
		   enum nft_set_policies policy)
{
	struct nftables_pernet *nft_net = net_generic(ctx->net, nf_tables_net_id);
	const struct nft_set_ops *ops, *bops;
	struct nft_set_estimate est, best;
	const struct nft_set_type *type;
	u32 flags = 0;

	lockdep_assert_held(&nft_net->commit_mutex);
	lockdep_nfnl_nft_mutex_not_held();
#ifdef CONFIG_MODULES
	if (list_empty(&nf_tables_set_types)) {
		nft_request_module(ctx->net, "nft-set");
		if (!list_empty(&nf_tables_set_types))
			return ERR_PTR(-EAGAIN);
	}
#endif
	if (nla[NFTA_SET_FLAGS] != NULL)
		flags = ntohl(nla_get_be32(nla[NFTA_SET_FLAGS]));

	bops	    = NULL;
	best.size   = ~0;
	best.lookup = ~0;
	best.space  = ~0;

	list_for_each_entry(type, &nf_tables_set_types, list) {
		ops = &type->ops;

		if (!nft_set_ops_candidate(type, flags))
			continue;
		if (!ops->estimate(desc, flags, &est))
			continue;

		switch (policy) {
		case NFT_SET_POL_PERFORMANCE:
			if (est.lookup < best.lookup)
				break;
			if (est.lookup == best.lookup &&
			    est.space < best.space)
				break;
			continue;
		case NFT_SET_POL_MEMORY:
			if (!desc->size) {
				if (est.space < best.space)
					break;
				if (est.space == best.space &&
				    est.lookup < best.lookup)
					break;
			} else if (est.size < best.size || !bops) {
				break;
			}
			continue;
		default:
			break;
		}

		if (!try_module_get(type->owner))
			continue;
		if (bops != NULL)
			module_put(to_set_type(bops)->owner);

		bops = ops;
		best = est;
	}

	if (bops != NULL)
		return bops;

	return ERR_PTR(-EOPNOTSUPP);
}

static const struct nla_policy nft_set_policy[NFTA_SET_MAX + 1] = {
	[NFTA_SET_TABLE]		= { .type = NLA_STRING,
					    .len = NFT_TABLE_MAXNAMELEN - 1 },
	[NFTA_SET_NAME]			= { .type = NLA_STRING,
					    .len = NFT_SET_MAXNAMELEN - 1 },
	[NFTA_SET_FLAGS]		= { .type = NLA_U32 },
	[NFTA_SET_KEY_TYPE]		= { .type = NLA_U32 },
	[NFTA_SET_KEY_LEN]		= { .type = NLA_U32 },
	[NFTA_SET_DATA_TYPE]		= { .type = NLA_U32 },
	[NFTA_SET_DATA_LEN]		= { .type = NLA_U32 },
	[NFTA_SET_POLICY]		= { .type = NLA_U32 },
	[NFTA_SET_DESC]			= { .type = NLA_NESTED },
	[NFTA_SET_ID]			= { .type = NLA_U32 },
	[NFTA_SET_TIMEOUT]		= { .type = NLA_U64 },
	[NFTA_SET_GC_INTERVAL]		= { .type = NLA_U32 },
	[NFTA_SET_USERDATA]		= { .type = NLA_BINARY,
					    .len  = NFT_USERDATA_MAXLEN },
	[NFTA_SET_OBJ_TYPE]		= { .type = NLA_U32 },
	[NFTA_SET_HANDLE]		= { .type = NLA_U64 },
};

static const struct nla_policy nft_set_desc_policy[NFTA_SET_DESC_MAX + 1] = {
	[NFTA_SET_DESC_SIZE]		= { .type = NLA_U32 },
};

static int nft_ctx_init_from_setattr(struct nft_ctx *ctx, struct net *net,
				     const struct sk_buff *skb,
				     const struct nlmsghdr *nlh,
				     const struct nlattr * const nla[],
				     struct netlink_ext_ack *extack,
				     u8 genmask)
{
	const struct nfgenmsg *nfmsg = nlmsg_data(nlh);
	int family = nfmsg->nfgen_family;
	struct nft_table *table = NULL;

	if (nla[NFTA_SET_TABLE] != NULL) {
		table = nft_table_lookup(net, nla[NFTA_SET_TABLE], family,
					 genmask);
		if (IS_ERR(table)) {
			NL_SET_BAD_ATTR(extack, nla[NFTA_SET_TABLE]);
			return PTR_ERR(table);
		}
	}

	nft_ctx_init(ctx, net, skb, nlh, family, table, NULL, nla);
	return 0;
}

static struct nft_set *nft_set_lookup(const struct nft_table *table,
				      const struct nlattr *nla, u8 genmask)
{
	struct nft_set *set;

	if (nla == NULL)
		return ERR_PTR(-EINVAL);

	list_for_each_entry_rcu(set, &table->sets, list) {
		if (!nla_strcmp(nla, set->name) &&
		    nft_active_genmask(set, genmask))
			return set;
	}
	return ERR_PTR(-ENOENT);
}

static struct nft_set *nft_set_lookup_byhandle(const struct nft_table *table,
					       const struct nlattr *nla,
					       u8 genmask)
{
	struct nft_set *set;

	list_for_each_entry(set, &table->sets, list) {
		if (be64_to_cpu(nla_get_be64(nla)) == set->handle &&
		    nft_active_genmask(set, genmask))
			return set;
	}
	return ERR_PTR(-ENOENT);
}

static struct nft_set *nft_set_lookup_byid(const struct net *net,
					   const struct nft_table *table,
					   const struct nlattr *nla, u8 genmask)
{
	struct nftables_pernet *nft_net = net_generic(net, nf_tables_net_id);
	struct nft_trans *trans;
	u32 id = ntohl(nla_get_be32(nla));

	list_for_each_entry(trans, &nft_net->commit_list, list) {
		if (trans->msg_type == NFT_MSG_NEWSET) {
			struct nft_set *set = nft_trans_set(trans);

			if (id == nft_trans_set_id(trans) &&
			    set->table == table &&
			    nft_active_genmask(set, genmask))
				return set;
		}
	}
	return ERR_PTR(-ENOENT);
}

struct nft_set *nft_set_lookup_global(const struct net *net,
				      const struct nft_table *table,
				      const struct nlattr *nla_set_name,
				      const struct nlattr *nla_set_id,
				      u8 genmask)
{
	struct nft_set *set;

	set = nft_set_lookup(table, nla_set_name, genmask);
	if (IS_ERR(set)) {
		if (!nla_set_id)
			return set;

		set = nft_set_lookup_byid(net, table, nla_set_id, genmask);
	}
	return set;
}
EXPORT_SYMBOL_GPL(nft_set_lookup_global);

static int nf_tables_set_alloc_name(struct nft_ctx *ctx, struct nft_set *set,
				    const char *name)
{
	const struct nft_set *i;
	const char *p;
	unsigned long *inuse;
	unsigned int n = 0, min = 0;

	p = strchr(name, '%');
	if (p != NULL) {
		if (p[1] != 'd' || strchr(p + 2, '%'))
			return -EINVAL;

		if (strnlen(name, NFT_SET_MAX_ANONLEN) >= NFT_SET_MAX_ANONLEN)
			return -EINVAL;

		inuse = (unsigned long *)get_zeroed_page(GFP_KERNEL);
		if (inuse == NULL)
			return -ENOMEM;
cont:
		list_for_each_entry(i, &ctx->table->sets, list) {
			int tmp;

			if (!nft_is_active_next(ctx->net, i))
				continue;
			if (!sscanf(i->name, name, &tmp))
				continue;
			if (tmp < min || tmp >= min + BITS_PER_BYTE * PAGE_SIZE)
				continue;

			set_bit(tmp - min, inuse);
		}

		n = find_first_zero_bit(inuse, BITS_PER_BYTE * PAGE_SIZE);
		if (n >= BITS_PER_BYTE * PAGE_SIZE) {
			min += BITS_PER_BYTE * PAGE_SIZE;
			memset(inuse, 0, PAGE_SIZE);
			goto cont;
		}
		free_page((unsigned long)inuse);
	}

	set->name = kasprintf(GFP_KERNEL, name, min + n);
	if (!set->name)
		return -ENOMEM;

	list_for_each_entry(i, &ctx->table->sets, list) {
		if (!nft_is_active_next(ctx->net, i))
			continue;
		if (!strcmp(set->name, i->name)) {
			kfree(set->name);
			return -ENFILE;
		}
	}
	return 0;
}

int nf_msecs_to_jiffies64(const struct nlattr *nla, u64 *result)
{
	u64 ms = be64_to_cpu(nla_get_be64(nla));
	u64 max = (u64)(~((u64)0));

	max = div_u64(max, NSEC_PER_MSEC);
	if (ms >= max)
		return -ERANGE;

	ms *= NSEC_PER_MSEC;
	*result = nsecs_to_jiffies64(ms) ? : !!ms;
	return 0;
}

__be64 nf_jiffies64_to_msecs(u64 input)
{
	u64 ms = jiffies64_to_nsecs(input);

	return cpu_to_be64(div_u64(ms, NSEC_PER_MSEC));
}

static int nf_tables_fill_set(struct sk_buff *skb, const struct nft_ctx *ctx,
			      const struct nft_set *set, u16 event, u16 flags)
{
	struct nlmsghdr *nlh;
	struct nlattr *desc;
	u32 portid = ctx->portid;
	u32 seq = ctx->seq;

	event = nfnl_msg_type(NFNL_SUBSYS_NFTABLES, event);
	nlh = nfnl_msg_put(skb, portid, seq, event, flags, ctx->family,
			   NFNETLINK_V0, nft_base_seq(ctx->net));
	if (!nlh)
		goto nla_put_failure;

	if (nla_put_string(skb, NFTA_SET_TABLE, ctx->table->name))
		goto nla_put_failure;
	if (nla_put_string(skb, NFTA_SET_NAME, set->name))
		goto nla_put_failure;
	if (nla_put_be64(skb, NFTA_SET_HANDLE, cpu_to_be64(set->handle),
			 NFTA_SET_PAD))
		goto nla_put_failure;
	if (set->flags != 0)
		if (nla_put_be32(skb, NFTA_SET_FLAGS, htonl(set->flags)))
			goto nla_put_failure;

	if (nla_put_be32(skb, NFTA_SET_KEY_TYPE, htonl(set->ktype)))
		goto nla_put_failure;
	if (nla_put_be32(skb, NFTA_SET_KEY_LEN, htonl(set->klen)))
		goto nla_put_failure;
	if (set->flags & NFT_SET_MAP) {
		if (nla_put_be32(skb, NFTA_SET_DATA_TYPE, htonl(set->dtype)))
			goto nla_put_failure;
		if (nla_put_be32(skb, NFTA_SET_DATA_LEN, htonl(set->dlen)))
			goto nla_put_failure;
	}
	if (set->flags & NFT_SET_OBJECT &&
	    nla_put_be32(skb, NFTA_SET_OBJ_TYPE, htonl(set->objtype)))
		goto nla_put_failure;

	if (set->timeout &&
	    nla_put_be64(skb, NFTA_SET_TIMEOUT,
			 nf_jiffies64_to_msecs(set->timeout),
			 NFTA_SET_PAD))
		goto nla_put_failure;
	if (set->gc_int &&
	    nla_put_be32(skb, NFTA_SET_GC_INTERVAL, htonl(set->gc_int)))
		goto nla_put_failure;

	if (set->policy != NFT_SET_POL_PERFORMANCE) {
		if (nla_put_be32(skb, NFTA_SET_POLICY, htonl(set->policy)))
			goto nla_put_failure;
	}

	if (set->udata &&
	    nla_put(skb, NFTA_SET_USERDATA, set->udlen, set->udata))
		goto nla_put_failure;

	desc = nla_nest_start(skb, NFTA_SET_DESC);
	if (desc == NULL)
		goto nla_put_failure;
	if (set->size &&
	    nla_put_be32(skb, NFTA_SET_DESC_SIZE, htonl(set->size)))
		goto nla_put_failure;
	nla_nest_end(skb, desc);

	nlmsg_end(skb, nlh);
	return 0;

nla_put_failure:
	nlmsg_trim(skb, nlh);
	return -1;
}

static void nf_tables_set_notify(const struct nft_ctx *ctx,
				 const struct nft_set *set, int event,
			         gfp_t gfp_flags)
{
	struct sk_buff *skb;
	u32 portid = ctx->portid;
	int err;

	if (!ctx->report &&
	    !nfnetlink_has_listeners(ctx->net, NFNLGRP_NFTABLES))
		return;

	skb = nlmsg_new(NLMSG_GOODSIZE, gfp_flags);
	if (skb == NULL)
		goto err;

	err = nf_tables_fill_set(skb, ctx, set, event, 0);
	if (err < 0) {
		kfree_skb(skb);
		goto err;
	}

	nfnetlink_send(skb, ctx->net, portid, NFNLGRP_NFTABLES, ctx->report,
		       gfp_flags);
	return;
err:
	nfnetlink_set_err(ctx->net, portid, NFNLGRP_NFTABLES, -ENOBUFS);
}

static int nf_tables_dump_sets(struct sk_buff *skb, struct netlink_callback *cb)
{
	const struct nft_set *set;
	unsigned int idx, s_idx = cb->args[0];
	struct nft_table *table, *cur_table = (struct nft_table *)cb->args[2];
	struct net *net = sock_net(skb->sk);
	struct nft_ctx *ctx = cb->data, ctx_set;
	struct nftables_pernet *nft_net;

	if (cb->args[1])
		return skb->len;

	rcu_read_lock();
	nft_net = net_generic(net, nf_tables_net_id);
	cb->seq = nft_net->base_seq;

	list_for_each_entry_rcu(table, &nft_net->tables, list) {
		if (ctx->family != NFPROTO_UNSPEC &&
		    ctx->family != table->family)
			continue;

		if (ctx->table && ctx->table != table)
			continue;

		if (cur_table) {
			if (cur_table != table)
				continue;

			cur_table = NULL;
		}
		idx = 0;
		list_for_each_entry_rcu(set, &table->sets, list) {
			if (idx < s_idx)
				goto cont;
			if (!nft_is_active(net, set))
				goto cont;

			ctx_set = *ctx;
			ctx_set.table = table;
			ctx_set.family = table->family;

			if (nf_tables_fill_set(skb, &ctx_set, set,
					       NFT_MSG_NEWSET,
					       NLM_F_MULTI) < 0) {
				cb->args[0] = idx;
				cb->args[2] = (unsigned long) table;
				goto done;
			}
			nl_dump_check_consistent(cb, nlmsg_hdr(skb));
cont:
			idx++;
		}
		if (s_idx)
			s_idx = 0;
	}
	cb->args[1] = 1;
done:
	rcu_read_unlock();
	return skb->len;
}

static int nf_tables_dump_sets_start(struct netlink_callback *cb)
{
	struct nft_ctx *ctx_dump = NULL;

	ctx_dump = kmemdup(cb->data, sizeof(*ctx_dump), GFP_ATOMIC);
	if (ctx_dump == NULL)
		return -ENOMEM;

	cb->data = ctx_dump;
	return 0;
}

static int nf_tables_dump_sets_done(struct netlink_callback *cb)
{
	kfree(cb->data);
	return 0;
}

/* called with rcu_read_lock held */
static int nf_tables_getset(struct net *net, struct sock *nlsk,
			    struct sk_buff *skb, const struct nlmsghdr *nlh,
			    const struct nlattr * const nla[],
			    struct netlink_ext_ack *extack)
{
	u8 genmask = nft_genmask_cur(net);
	const struct nft_set *set;
	struct nft_ctx ctx;
	struct sk_buff *skb2;
	const struct nfgenmsg *nfmsg = nlmsg_data(nlh);
	int err;

	/* Verify existence before starting dump */
	err = nft_ctx_init_from_setattr(&ctx, net, skb, nlh, nla, extack,
					genmask);
	if (err < 0)
		return err;

	if (nlh->nlmsg_flags & NLM_F_DUMP) {
		struct netlink_dump_control c = {
			.start = nf_tables_dump_sets_start,
			.dump = nf_tables_dump_sets,
			.done = nf_tables_dump_sets_done,
			.data = &ctx,
			.module = THIS_MODULE,
		};

		return nft_netlink_dump_start_rcu(nlsk, skb, nlh, &c);
	}

	/* Only accept unspec with dump */
	if (nfmsg->nfgen_family == NFPROTO_UNSPEC)
		return -EAFNOSUPPORT;
	if (!nla[NFTA_SET_TABLE])
		return -EINVAL;

	set = nft_set_lookup(ctx.table, nla[NFTA_SET_NAME], genmask);
	if (IS_ERR(set))
		return PTR_ERR(set);

	skb2 = alloc_skb(NLMSG_GOODSIZE, GFP_ATOMIC);
	if (skb2 == NULL)
		return -ENOMEM;

	err = nf_tables_fill_set(skb2, &ctx, set, NFT_MSG_NEWSET, 0);
	if (err < 0)
		goto err_fill_set_info;

	return nfnetlink_unicast(skb2, net, NETLINK_CB(skb).portid);

err_fill_set_info:
	kfree_skb(skb2);
	return err;
}

static int nf_tables_set_desc_parse(const struct nft_ctx *ctx,
				    struct nft_set_desc *desc,
				    const struct nlattr *nla)
{
	struct nlattr *da[NFTA_SET_DESC_MAX + 1];
	int err;

	err = nla_parse_nested(da, NFTA_SET_DESC_MAX, nla,
			       nft_set_desc_policy, NULL);
	if (err < 0)
		return err;

	if (da[NFTA_SET_DESC_SIZE] != NULL)
		desc->size = ntohl(nla_get_be32(da[NFTA_SET_DESC_SIZE]));

	return 0;
}

static int nf_tables_newset(struct net *net, struct sock *nlsk,
			    struct sk_buff *skb, const struct nlmsghdr *nlh,
			    const struct nlattr * const nla[],
			    struct netlink_ext_ack *extack)
{
	const struct nfgenmsg *nfmsg = nlmsg_data(nlh);
	u8 genmask = nft_genmask_next(net);
	int family = nfmsg->nfgen_family;
	const struct nft_set_ops *ops;
	struct nft_table *table;
	struct nft_set *set;
	struct nft_ctx ctx;
	char *name;
	u64 size;
	u64 timeout;
	u32 ktype, dtype, flags, policy, gc_int, objtype;
	struct nft_set_desc desc;
	unsigned char *udata;
	u16 udlen;
	int err;

	if (nla[NFTA_SET_TABLE] == NULL ||
	    nla[NFTA_SET_NAME] == NULL ||
	    nla[NFTA_SET_KEY_LEN] == NULL ||
	    nla[NFTA_SET_ID] == NULL)
		return -EINVAL;

	memset(&desc, 0, sizeof(desc));

	ktype = NFT_DATA_VALUE;
	if (nla[NFTA_SET_KEY_TYPE] != NULL) {
		ktype = ntohl(nla_get_be32(nla[NFTA_SET_KEY_TYPE]));
		if ((ktype & NFT_DATA_RESERVED_MASK) == NFT_DATA_RESERVED_MASK)
			return -EINVAL;
	}

	desc.klen = ntohl(nla_get_be32(nla[NFTA_SET_KEY_LEN]));
	if (desc.klen == 0 || desc.klen > NFT_DATA_VALUE_MAXLEN)
		return -EINVAL;

	flags = 0;
	if (nla[NFTA_SET_FLAGS] != NULL) {
		flags = ntohl(nla_get_be32(nla[NFTA_SET_FLAGS]));
		if (flags & ~(NFT_SET_ANONYMOUS | NFT_SET_CONSTANT |
			      NFT_SET_INTERVAL | NFT_SET_TIMEOUT |
			      NFT_SET_MAP | NFT_SET_EVAL |
			      NFT_SET_OBJECT))
			return -EOPNOTSUPP;
		/* Only one of these operations is supported */
		if ((flags & (NFT_SET_MAP | NFT_SET_OBJECT)) ==
			     (NFT_SET_MAP | NFT_SET_OBJECT))
			return -EOPNOTSUPP;
		if ((flags & (NFT_SET_EVAL | NFT_SET_OBJECT)) ==
			     (NFT_SET_EVAL | NFT_SET_OBJECT))
			return -EOPNOTSUPP;
		if ((flags & (NFT_SET_ANONYMOUS | NFT_SET_TIMEOUT | NFT_SET_EVAL)) ==
			     (NFT_SET_ANONYMOUS | NFT_SET_TIMEOUT))
			return -EOPNOTSUPP;
		if ((flags & (NFT_SET_CONSTANT | NFT_SET_TIMEOUT)) ==
			     (NFT_SET_CONSTANT | NFT_SET_TIMEOUT))
			return -EOPNOTSUPP;
	}

	dtype = 0;
	if (nla[NFTA_SET_DATA_TYPE] != NULL) {
		if (!(flags & NFT_SET_MAP))
			return -EINVAL;

		dtype = ntohl(nla_get_be32(nla[NFTA_SET_DATA_TYPE]));
		if ((dtype & NFT_DATA_RESERVED_MASK) == NFT_DATA_RESERVED_MASK &&
		    dtype != NFT_DATA_VERDICT)
			return -EINVAL;

		if (dtype != NFT_DATA_VERDICT) {
			if (nla[NFTA_SET_DATA_LEN] == NULL)
				return -EINVAL;
			desc.dlen = ntohl(nla_get_be32(nla[NFTA_SET_DATA_LEN]));
			if (desc.dlen == 0 || desc.dlen > NFT_DATA_VALUE_MAXLEN)
				return -EINVAL;
		} else
			desc.dlen = sizeof(struct nft_verdict);
	} else if (flags & NFT_SET_MAP)
		return -EINVAL;

	if (nla[NFTA_SET_OBJ_TYPE] != NULL) {
		if (!(flags & NFT_SET_OBJECT))
			return -EINVAL;

		objtype = ntohl(nla_get_be32(nla[NFTA_SET_OBJ_TYPE]));
		if (objtype == NFT_OBJECT_UNSPEC ||
		    objtype > NFT_OBJECT_MAX)
			return -EOPNOTSUPP;
	} else if (flags & NFT_SET_OBJECT)
		return -EINVAL;
	else
		objtype = NFT_OBJECT_UNSPEC;

	timeout = 0;
	if (nla[NFTA_SET_TIMEOUT] != NULL) {
		if (!(flags & NFT_SET_TIMEOUT))
			return -EINVAL;

		if (flags & NFT_SET_ANONYMOUS)
			return -EOPNOTSUPP;

		err = nf_msecs_to_jiffies64(nla[NFTA_SET_TIMEOUT], &timeout);
		if (err)
			return err;
	}
	gc_int = 0;
	if (nla[NFTA_SET_GC_INTERVAL] != NULL) {
		if (!(flags & NFT_SET_TIMEOUT))
			return -EINVAL;

		if (flags & NFT_SET_ANONYMOUS)
			return -EOPNOTSUPP;

		gc_int = ntohl(nla_get_be32(nla[NFTA_SET_GC_INTERVAL]));
	}

	policy = NFT_SET_POL_PERFORMANCE;
	if (nla[NFTA_SET_POLICY] != NULL)
		policy = ntohl(nla_get_be32(nla[NFTA_SET_POLICY]));

	if (nla[NFTA_SET_DESC] != NULL) {
		err = nf_tables_set_desc_parse(&ctx, &desc, nla[NFTA_SET_DESC]);
		if (err < 0)
			return err;
	}

	table = nft_table_lookup(net, nla[NFTA_SET_TABLE], family, genmask);
	if (IS_ERR(table)) {
		NL_SET_BAD_ATTR(extack, nla[NFTA_SET_TABLE]);
		return PTR_ERR(table);
	}

	nft_ctx_init(&ctx, net, skb, nlh, family, table, NULL, nla);

	set = nft_set_lookup(table, nla[NFTA_SET_NAME], genmask);
	if (IS_ERR(set)) {
		if (PTR_ERR(set) != -ENOENT) {
			NL_SET_BAD_ATTR(extack, nla[NFTA_SET_NAME]);
			return PTR_ERR(set);
		}
	} else {
		if (nlh->nlmsg_flags & NLM_F_EXCL) {
			NL_SET_BAD_ATTR(extack, nla[NFTA_SET_NAME]);
			return -EEXIST;
		}
		if (nlh->nlmsg_flags & NLM_F_REPLACE)
			return -EOPNOTSUPP;

		return 0;
	}

	if (!(nlh->nlmsg_flags & NLM_F_CREATE))
		return -ENOENT;

	ops = nft_select_set_ops(&ctx, nla, &desc, policy);
	if (IS_ERR(ops))
		return PTR_ERR(ops);

	udlen = 0;
	if (nla[NFTA_SET_USERDATA])
		udlen = nla_len(nla[NFTA_SET_USERDATA]);

	size = 0;
	if (ops->privsize != NULL)
		size = ops->privsize(nla, &desc);

	if (!nft_use_inc(&table->use)) {
		err = -EMFILE;
		goto err1;
	}

	set = kvzalloc(sizeof(*set) + size + udlen, GFP_KERNEL);
	if (!set) {
		err = -ENOMEM;
		goto err_alloc;
	}

	name = nla_strdup(nla[NFTA_SET_NAME], GFP_KERNEL);
	if (!name) {
		err = -ENOMEM;
		goto err2;
	}

	err = nf_tables_set_alloc_name(&ctx, set, name);
	kfree(name);
	if (err < 0)
		goto err2;

	udata = NULL;
	if (udlen) {
		udata = set->data + size;
		nla_memcpy(udata, nla[NFTA_SET_USERDATA], udlen);
	}

	INIT_LIST_HEAD(&set->bindings);
	refcount_set(&set->refs, 1);
	set->table = table;
	write_pnet(&set->net, net);
	set->ops   = ops;
	set->ktype = ktype;
	set->klen  = desc.klen;
	set->dtype = dtype;
	set->objtype = objtype;
	set->dlen  = desc.dlen;
	set->flags = flags;
	set->size  = desc.size;
	set->policy = policy;
	set->udlen  = udlen;
	set->udata  = udata;
	set->timeout = timeout;
	set->gc_int = gc_int;
	set->handle = nf_tables_alloc_handle(table);

	err = ops->init(set, &desc, nla);
	if (err < 0)
		goto err3;

	err = nft_trans_set_add(&ctx, NFT_MSG_NEWSET, set);
	if (err < 0)
		goto err4;

	list_add_tail_rcu(&set->list, &table->sets);

	return 0;

err4:
	ops->destroy(&ctx, set);
err3:
	kfree(set->name);
err2:
	kvfree(set);
err_alloc:
	nft_use_dec_restore(&table->use);
err1:
	module_put(to_set_type(ops)->owner);
	return err;
}

static void nft_set_put(struct nft_set *set)
{
	if (refcount_dec_and_test(&set->refs)) {
		kfree(set->name);
		kvfree(set);
	}
}

static void nft_set_destroy(const struct nft_ctx *ctx, struct nft_set *set)
{
	if (WARN_ON(set->use > 0))
		return;

	set->ops->destroy(ctx, set);
	module_put(to_set_type(set->ops)->owner);
	nft_set_put(set);
}

static int nf_tables_delset(struct net *net, struct sock *nlsk,
			    struct sk_buff *skb, const struct nlmsghdr *nlh,
			    const struct nlattr * const nla[],
			    struct netlink_ext_ack *extack)
{
	const struct nfgenmsg *nfmsg = nlmsg_data(nlh);
	u8 genmask = nft_genmask_next(net);
	const struct nlattr *attr;
	struct nft_set *set;
	struct nft_ctx ctx;
	int err;

	if (nfmsg->nfgen_family == NFPROTO_UNSPEC)
		return -EAFNOSUPPORT;
	if (nla[NFTA_SET_TABLE] == NULL)
		return -EINVAL;

	err = nft_ctx_init_from_setattr(&ctx, net, skb, nlh, nla, extack,
					genmask);
	if (err < 0)
		return err;

	if (nla[NFTA_SET_HANDLE]) {
		attr = nla[NFTA_SET_HANDLE];
		set = nft_set_lookup_byhandle(ctx.table, attr, genmask);
	} else {
		attr = nla[NFTA_SET_NAME];
		set = nft_set_lookup(ctx.table, attr, genmask);
	}

	if (IS_ERR(set)) {
		NL_SET_BAD_ATTR(extack, attr);
		return PTR_ERR(set);
	}
	if (set->use ||
	    (nlh->nlmsg_flags & NLM_F_NONREC && atomic_read(&set->nelems) > 0)) {
		NL_SET_BAD_ATTR(extack, attr);
		return -EBUSY;
	}

	return nft_delset(&ctx, set);
}

static int nft_validate_register_store(const struct nft_ctx *ctx,
				       enum nft_registers reg,
				       const struct nft_data *data,
				       enum nft_data_types type,
				       unsigned int len);

static int nf_tables_bind_check_setelem(const struct nft_ctx *ctx,
					struct nft_set *set,
					const struct nft_set_iter *iter,
					struct nft_set_elem *elem)
{
	const struct nft_set_ext *ext = nft_set_elem_ext(set, elem->priv);
	enum nft_registers dreg;

	dreg = nft_type_to_reg(set->dtype);
	return nft_validate_register_store(ctx, dreg, nft_set_ext_data(ext),
					   set->dtype == NFT_DATA_VERDICT ?
					   NFT_DATA_VERDICT : NFT_DATA_VALUE,
					   set->dlen);
}

int nf_tables_bind_set(const struct nft_ctx *ctx, struct nft_set *set,
		       struct nft_set_binding *binding)
{
	struct nft_set_binding *i;
	struct nft_set_iter iter;

	if (!list_empty(&set->bindings) && nft_set_is_anonymous(set))
		return -EBUSY;

	if (binding->flags & NFT_SET_MAP) {
		/* If the set is already bound to the same chain all
		 * jumps are already validated for that chain.
		 */
		list_for_each_entry(i, &set->bindings, list) {
			if (i->flags & NFT_SET_MAP &&
			    i->chain == binding->chain)
				goto bind;
		}

		iter.genmask	= nft_genmask_next(ctx->net);
		iter.skip 	= 0;
		iter.count	= 0;
		iter.err	= 0;
		iter.fn		= nf_tables_bind_check_setelem;

		set->ops->walk(ctx, set, &iter);
		if (iter.err < 0)
			return iter.err;
	}
bind:
	if (!nft_use_inc(&set->use))
		return -EMFILE;

	binding->chain = ctx->chain;
	list_add_tail_rcu(&binding->list, &set->bindings);
	nft_set_trans_bind(ctx, set);

	return 0;
}
EXPORT_SYMBOL_GPL(nf_tables_bind_set);

void nf_tables_unbind_set(const struct nft_ctx *ctx, struct nft_set *set,
			  struct nft_set_binding *binding, bool event)
{
	list_del_rcu(&binding->list);

	if (list_empty(&set->bindings) && nft_set_is_anonymous(set)) {
		list_del_rcu(&set->list);
		set->dead = 1;
		if (event)
			nf_tables_set_notify(ctx, set, NFT_MSG_DELSET,
					     GFP_KERNEL);
	}
}
EXPORT_SYMBOL_GPL(nf_tables_unbind_set);

static void nft_setelem_data_activate(const struct net *net,
				      const struct nft_set *set,
				      struct nft_set_elem *elem);

static int nft_mapelem_activate(const struct nft_ctx *ctx,
				struct nft_set *set,
				const struct nft_set_iter *iter,
				struct nft_set_elem *elem)
{
	nft_setelem_data_activate(ctx->net, set, elem);

	return 0;
}

static void nft_map_activate(const struct nft_ctx *ctx, struct nft_set *set)
{
	struct nft_set_iter iter = {
		.genmask	= nft_genmask_next(ctx->net),
		.fn		= nft_mapelem_activate,
	};

	set->ops->walk(ctx, set, &iter);
	WARN_ON_ONCE(iter.err);
}

void nf_tables_activate_set(const struct nft_ctx *ctx, struct nft_set *set)
{
	if (nft_set_is_anonymous(set)) {
		if (set->flags & (NFT_SET_MAP | NFT_SET_OBJECT))
			nft_map_activate(ctx, set);

		nft_clear(ctx->net, set);
	}

	nft_use_inc_restore(&set->use);
}
EXPORT_SYMBOL_GPL(nf_tables_activate_set);

void nf_tables_deactivate_set(const struct nft_ctx *ctx, struct nft_set *set,
			      struct nft_set_binding *binding,
			      enum nft_trans_phase phase)
{
	switch (phase) {
	case NFT_TRANS_PREPARE_ERROR:
		nft_set_trans_unbind(ctx, set);
		if (nft_set_is_anonymous(set))
			nft_deactivate_next(ctx->net, set);
		else
			list_del_rcu(&binding->list);

		nft_use_dec(&set->use);
		break;
	case NFT_TRANS_PREPARE:
		if (nft_set_is_anonymous(set)) {
			if (set->flags & (NFT_SET_MAP | NFT_SET_OBJECT))
				nft_map_deactivate(ctx, set);

			nft_deactivate_next(ctx->net, set);
		}
		nft_use_dec(&set->use);
		return;
	case NFT_TRANS_ABORT:
	case NFT_TRANS_RELEASE:
		if (nft_set_is_anonymous(set) &&
		    set->flags & (NFT_SET_MAP | NFT_SET_OBJECT))
			nft_map_deactivate(ctx, set);

		nft_use_dec(&set->use);
		/* fall through */
	default:
		nf_tables_unbind_set(ctx, set, binding,
				     phase == NFT_TRANS_COMMIT);
	}
}
EXPORT_SYMBOL_GPL(nf_tables_deactivate_set);

void nf_tables_destroy_set(const struct nft_ctx *ctx, struct nft_set *set)
{
	if (list_empty(&set->bindings) && nft_set_is_anonymous(set))
		nft_set_destroy(ctx, set);
}
EXPORT_SYMBOL_GPL(nf_tables_destroy_set);

const struct nft_set_ext_type nft_set_ext_types[] = {
	[NFT_SET_EXT_KEY]		= {
		.align	= __alignof__(u32),
	},
	[NFT_SET_EXT_DATA]		= {
		.align	= __alignof__(u32),
	},
	[NFT_SET_EXT_EXPR]		= {
		.align	= __alignof__(struct nft_expr),
	},
	[NFT_SET_EXT_OBJREF]		= {
		.len	= sizeof(struct nft_object *),
		.align	= __alignof__(struct nft_object *),
	},
	[NFT_SET_EXT_FLAGS]		= {
		.len	= sizeof(u8),
		.align	= __alignof__(u8),
	},
	[NFT_SET_EXT_TIMEOUT]		= {
		.len	= sizeof(u64),
		.align	= __alignof__(u64),
	},
	[NFT_SET_EXT_EXPIRATION]	= {
		.len	= sizeof(u64),
		.align	= __alignof__(u64),
	},
	[NFT_SET_EXT_USERDATA]		= {
		.len	= sizeof(struct nft_userdata),
		.align	= __alignof__(struct nft_userdata),
	},
};
EXPORT_SYMBOL_GPL(nft_set_ext_types);

/*
 * Set elements
 */

static const struct nla_policy nft_set_elem_policy[NFTA_SET_ELEM_MAX + 1] = {
	[NFTA_SET_ELEM_KEY]		= { .type = NLA_NESTED },
	[NFTA_SET_ELEM_DATA]		= { .type = NLA_NESTED },
	[NFTA_SET_ELEM_FLAGS]		= { .type = NLA_U32 },
	[NFTA_SET_ELEM_TIMEOUT]		= { .type = NLA_U64 },
	[NFTA_SET_ELEM_USERDATA]	= { .type = NLA_BINARY,
					    .len = NFT_USERDATA_MAXLEN },
	[NFTA_SET_ELEM_EXPR]		= { .type = NLA_NESTED },
	[NFTA_SET_ELEM_OBJREF]		= { .type = NLA_STRING,
					    .len = NFT_OBJ_MAXNAMELEN - 1 },
};

static const struct nla_policy nft_set_elem_list_policy[NFTA_SET_ELEM_LIST_MAX + 1] = {
	[NFTA_SET_ELEM_LIST_TABLE]	= { .type = NLA_STRING,
					    .len = NFT_TABLE_MAXNAMELEN - 1 },
	[NFTA_SET_ELEM_LIST_SET]	= { .type = NLA_STRING,
					    .len = NFT_SET_MAXNAMELEN - 1 },
	[NFTA_SET_ELEM_LIST_ELEMENTS]	= { .type = NLA_NESTED },
	[NFTA_SET_ELEM_LIST_SET_ID]	= { .type = NLA_U32 },
};

static int nft_ctx_init_from_elemattr(struct nft_ctx *ctx, struct net *net,
				      const struct sk_buff *skb,
				      const struct nlmsghdr *nlh,
				      const struct nlattr * const nla[],
				      struct netlink_ext_ack *extack,
				      u8 genmask)
{
	const struct nfgenmsg *nfmsg = nlmsg_data(nlh);
	int family = nfmsg->nfgen_family;
	struct nft_table *table;

	table = nft_table_lookup(net, nla[NFTA_SET_ELEM_LIST_TABLE], family,
				 genmask);
	if (IS_ERR(table)) {
		NL_SET_BAD_ATTR(extack, nla[NFTA_SET_ELEM_LIST_TABLE]);
		return PTR_ERR(table);
	}

	nft_ctx_init(ctx, net, skb, nlh, family, table, NULL, nla);
	return 0;
}

static int nf_tables_fill_setelem(struct sk_buff *skb,
				  const struct nft_set *set,
				  const struct nft_set_elem *elem)
{
	const struct nft_set_ext *ext = nft_set_elem_ext(set, elem->priv);
	unsigned char *b = skb_tail_pointer(skb);
	struct nlattr *nest;

	nest = nla_nest_start(skb, NFTA_LIST_ELEM);
	if (nest == NULL)
		goto nla_put_failure;

	if (nft_data_dump(skb, NFTA_SET_ELEM_KEY, nft_set_ext_key(ext),
			  NFT_DATA_VALUE, set->klen) < 0)
		goto nla_put_failure;

	if (nft_set_ext_exists(ext, NFT_SET_EXT_DATA) &&
	    nft_data_dump(skb, NFTA_SET_ELEM_DATA, nft_set_ext_data(ext),
			  nft_set_datatype(set), set->dlen) < 0)
		goto nla_put_failure;

	if (nft_set_ext_exists(ext, NFT_SET_EXT_EXPR) &&
	    nft_expr_dump(skb, NFTA_SET_ELEM_EXPR, nft_set_ext_expr(ext)) < 0)
		goto nla_put_failure;

	if (nft_set_ext_exists(ext, NFT_SET_EXT_OBJREF) &&
	    nla_put_string(skb, NFTA_SET_ELEM_OBJREF,
			   (*nft_set_ext_obj(ext))->name) < 0)
		goto nla_put_failure;

	if (nft_set_ext_exists(ext, NFT_SET_EXT_FLAGS) &&
	    nla_put_be32(skb, NFTA_SET_ELEM_FLAGS,
		         htonl(*nft_set_ext_flags(ext))))
		goto nla_put_failure;

	if (nft_set_ext_exists(ext, NFT_SET_EXT_TIMEOUT) &&
	    nla_put_be64(skb, NFTA_SET_ELEM_TIMEOUT,
			 nf_jiffies64_to_msecs(*nft_set_ext_timeout(ext)),
			 NFTA_SET_ELEM_PAD))
		goto nla_put_failure;

	if (nft_set_ext_exists(ext, NFT_SET_EXT_EXPIRATION)) {
		u64 expires, now = get_jiffies_64();

		expires = *nft_set_ext_expiration(ext);
		if (time_before64(now, expires))
			expires -= now;
		else
			expires = 0;

		if (nla_put_be64(skb, NFTA_SET_ELEM_EXPIRATION,
				 nf_jiffies64_to_msecs(expires),
				 NFTA_SET_ELEM_PAD))
			goto nla_put_failure;
	}

	if (nft_set_ext_exists(ext, NFT_SET_EXT_USERDATA)) {
		struct nft_userdata *udata;

		udata = nft_set_ext_userdata(ext);
		if (nla_put(skb, NFTA_SET_ELEM_USERDATA,
			    udata->len + 1, udata->data))
			goto nla_put_failure;
	}

	nla_nest_end(skb, nest);
	return 0;

nla_put_failure:
	nlmsg_trim(skb, b);
	return -EMSGSIZE;
}

struct nft_set_dump_args {
	const struct netlink_callback	*cb;
	struct nft_set_iter		iter;
	struct sk_buff			*skb;
};

static int nf_tables_dump_setelem(const struct nft_ctx *ctx,
				  struct nft_set *set,
				  const struct nft_set_iter *iter,
				  struct nft_set_elem *elem)
{
	const struct nft_set_ext *ext = nft_set_elem_ext(set, elem->priv);
	struct nft_set_dump_args *args;

	if (nft_set_elem_expired(ext) || nft_set_elem_is_dead(ext))
		return 0;

	args = container_of(iter, struct nft_set_dump_args, iter);
	return nf_tables_fill_setelem(args->skb, set, elem);
}

struct nft_set_dump_ctx {
	const struct nft_set	*set;
	struct nft_ctx		ctx;
};

static int nf_tables_dump_set(struct sk_buff *skb, struct netlink_callback *cb)
{
	struct nft_set_dump_ctx *dump_ctx = cb->data;
	struct net *net = sock_net(skb->sk);
	struct nftables_pernet *nft_net;
	struct nft_table *table;
	struct nft_set *set;
	struct nft_set_dump_args args;
	bool set_found = false;
	struct nlmsghdr *nlh;
	struct nlattr *nest;
	u32 portid, seq;
	int event;

	rcu_read_lock();
	nft_net = net_generic(net, nf_tables_net_id);
	list_for_each_entry_rcu(table, &nft_net->tables, list) {
		if (dump_ctx->ctx.family != NFPROTO_UNSPEC &&
		    dump_ctx->ctx.family != table->family)
			continue;

		if (table != dump_ctx->ctx.table)
			continue;

		list_for_each_entry_rcu(set, &table->sets, list) {
			if (set == dump_ctx->set) {
				set_found = true;
				break;
			}
		}
		break;
	}

	if (!set_found) {
		rcu_read_unlock();
		return -ENOENT;
	}

	event  = nfnl_msg_type(NFNL_SUBSYS_NFTABLES, NFT_MSG_NEWSETELEM);
	portid = NETLINK_CB(cb->skb).portid;
	seq    = cb->nlh->nlmsg_seq;

	nlh = nfnl_msg_put(skb, portid, seq, event, NLM_F_MULTI,
			   table->family, NFNETLINK_V0, nft_base_seq(net));
	if (!nlh)
		goto nla_put_failure;

	if (nla_put_string(skb, NFTA_SET_ELEM_LIST_TABLE, table->name))
		goto nla_put_failure;
	if (nla_put_string(skb, NFTA_SET_ELEM_LIST_SET, set->name))
		goto nla_put_failure;

	nest = nla_nest_start(skb, NFTA_SET_ELEM_LIST_ELEMENTS);
	if (nest == NULL)
		goto nla_put_failure;

	args.cb			= cb;
	args.skb		= skb;
	args.iter.genmask	= nft_genmask_cur(net);
	args.iter.skip		= cb->args[0];
	args.iter.count		= 0;
	args.iter.err		= 0;
	args.iter.fn		= nf_tables_dump_setelem;
	set->ops->walk(&dump_ctx->ctx, set, &args.iter);
	rcu_read_unlock();

	nla_nest_end(skb, nest);
	nlmsg_end(skb, nlh);

	if (args.iter.err && args.iter.err != -EMSGSIZE)
		return args.iter.err;
	if (args.iter.count == cb->args[0])
		return 0;

	cb->args[0] = args.iter.count;
	return skb->len;

nla_put_failure:
	rcu_read_unlock();
	return -ENOSPC;
}

static int nf_tables_dump_set_start(struct netlink_callback *cb)
{
	struct nft_set_dump_ctx *dump_ctx = cb->data;

	cb->data = kmemdup(dump_ctx, sizeof(*dump_ctx), GFP_ATOMIC);

	return cb->data ? 0 : -ENOMEM;
}

static int nf_tables_dump_set_done(struct netlink_callback *cb)
{
	kfree(cb->data);
	return 0;
}

static int nf_tables_fill_setelem_info(struct sk_buff *skb,
				       const struct nft_ctx *ctx, u32 seq,
				       u32 portid, int event, u16 flags,
				       const struct nft_set *set,
				       const struct nft_set_elem *elem)
{
	struct nlmsghdr *nlh;
	struct nlattr *nest;
	int err;

	event = nfnl_msg_type(NFNL_SUBSYS_NFTABLES, event);
	nlh = nfnl_msg_put(skb, portid, seq, event, flags, ctx->family,
			   NFNETLINK_V0, nft_base_seq(ctx->net));
	if (!nlh)
		goto nla_put_failure;

	if (nla_put_string(skb, NFTA_SET_TABLE, ctx->table->name))
		goto nla_put_failure;
	if (nla_put_string(skb, NFTA_SET_NAME, set->name))
		goto nla_put_failure;

	nest = nla_nest_start(skb, NFTA_SET_ELEM_LIST_ELEMENTS);
	if (nest == NULL)
		goto nla_put_failure;

	err = nf_tables_fill_setelem(skb, set, elem);
	if (err < 0)
		goto nla_put_failure;

	nla_nest_end(skb, nest);

	nlmsg_end(skb, nlh);
	return 0;

nla_put_failure:
	nlmsg_trim(skb, nlh);
	return -1;
}

static int nft_setelem_parse_flags(const struct nft_set *set,
				   const struct nlattr *attr, u32 *flags)
{
	if (attr == NULL)
		return 0;

	*flags = ntohl(nla_get_be32(attr));
	if (*flags & ~NFT_SET_ELEM_INTERVAL_END)
		return -EINVAL;
	if (!(set->flags & NFT_SET_INTERVAL) &&
	    *flags & NFT_SET_ELEM_INTERVAL_END)
		return -EINVAL;

	return 0;
}

static int nft_setelem_parse_key(struct nft_ctx *ctx, struct nft_set *set,
				 struct nft_data *key, struct nlattr *attr)
{
	struct nft_data_desc desc;
	int err;

	err = nft_data_init(ctx, key, NFT_DATA_VALUE_MAXLEN, &desc, attr);
	if (err < 0)
		return err;

	if (desc.type != NFT_DATA_VALUE || desc.len != set->klen) {
		nft_data_release(key, desc.type);
		return -EINVAL;
	}

	return 0;
}

static int nft_setelem_parse_data(struct nft_ctx *ctx, struct nft_set *set,
				  struct nft_data_desc *desc,
				  struct nft_data *data,
				  struct nlattr *attr)
{
	u32 dtype;
	int err;

	err = nft_data_init(ctx, data, NFT_DATA_VALUE_MAXLEN, desc, attr);
	if (err < 0)
		return err;

	if (set->dtype == NFT_DATA_VERDICT)
		dtype = NFT_DATA_VERDICT;
	else
		dtype = NFT_DATA_VALUE;

	if (dtype != desc->type ||
	    set->dlen != desc->len) {
		nft_data_release(data, desc->type);
		return -EINVAL;
	}

	return 0;
}

static int nft_get_set_elem(struct nft_ctx *ctx, struct nft_set *set,
			    const struct nlattr *attr)
{
	struct nlattr *nla[NFTA_SET_ELEM_MAX + 1];
	struct nft_set_elem elem;
	struct sk_buff *skb;
	uint32_t flags = 0;
	void *priv;
	int err;

	err = nla_parse_nested(nla, NFTA_SET_ELEM_MAX, attr,
			       nft_set_elem_policy, NULL);
	if (err < 0)
		return err;

	if (!nla[NFTA_SET_ELEM_KEY])
		return -EINVAL;

	err = nft_setelem_parse_flags(set, nla[NFTA_SET_ELEM_FLAGS], &flags);
	if (err < 0)
		return err;

	err = nft_setelem_parse_key(ctx, set, &elem.key.val,
				    nla[NFTA_SET_ELEM_KEY]);
	if (err < 0)
		return err;

	priv = set->ops->get(ctx->net, set, &elem, flags);
	if (IS_ERR(priv))
		return PTR_ERR(priv);

	elem.priv = priv;

	err = -ENOMEM;
	skb = nlmsg_new(NLMSG_GOODSIZE, GFP_ATOMIC);
	if (skb == NULL)
		return err;

	err = nf_tables_fill_setelem_info(skb, ctx, ctx->seq, ctx->portid,
					  NFT_MSG_NEWSETELEM, 0, set, &elem);
	if (err < 0)
		goto err_fill_setelem;

	return nfnetlink_unicast(skb, ctx->net, ctx->portid);

err_fill_setelem:
	kfree_skb(skb);
	return err;
}

/* called with rcu_read_lock held */
static int nf_tables_getsetelem(struct net *net, struct sock *nlsk,
				struct sk_buff *skb, const struct nlmsghdr *nlh,
				const struct nlattr * const nla[],
				struct netlink_ext_ack *extack)
{
	u8 genmask = nft_genmask_cur(net);
	struct nft_set *set;
	struct nlattr *attr;
	struct nft_ctx ctx;
	int rem, err = 0;

	err = nft_ctx_init_from_elemattr(&ctx, net, skb, nlh, nla, extack,
					 genmask);
	if (err < 0)
		return err;

	set = nft_set_lookup(ctx.table, nla[NFTA_SET_ELEM_LIST_SET], genmask);
	if (IS_ERR(set))
		return PTR_ERR(set);

	if (nlh->nlmsg_flags & NLM_F_DUMP) {
		struct netlink_dump_control c = {
			.start = nf_tables_dump_set_start,
			.dump = nf_tables_dump_set,
			.done = nf_tables_dump_set_done,
			.module = THIS_MODULE,
		};
		struct nft_set_dump_ctx dump_ctx = {
			.set = set,
			.ctx = ctx,
		};

		c.data = &dump_ctx;
		return nft_netlink_dump_start_rcu(nlsk, skb, nlh, &c);
	}

	if (!nla[NFTA_SET_ELEM_LIST_ELEMENTS])
		return -EINVAL;

	nla_for_each_nested(attr, nla[NFTA_SET_ELEM_LIST_ELEMENTS], rem) {
		err = nft_get_set_elem(&ctx, set, attr);
		if (err < 0) {
			NL_SET_BAD_ATTR(extack, attr);
			break;
		}
	}

	return err;
}

static void nf_tables_setelem_notify(const struct nft_ctx *ctx,
				     const struct nft_set *set,
				     const struct nft_set_elem *elem,
				     int event, u16 flags)
{
	struct net *net = ctx->net;
	u32 portid = ctx->portid;
	struct sk_buff *skb;
	int err;

	if (!ctx->report && !nfnetlink_has_listeners(net, NFNLGRP_NFTABLES))
		return;

	skb = nlmsg_new(NLMSG_GOODSIZE, GFP_KERNEL);
	if (skb == NULL)
		goto err;

	err = nf_tables_fill_setelem_info(skb, ctx, 0, portid, event, flags,
					  set, elem);
	if (err < 0) {
		kfree_skb(skb);
		goto err;
	}

	nfnetlink_send(skb, net, portid, NFNLGRP_NFTABLES, ctx->report,
		       GFP_KERNEL);
	return;
err:
	nfnetlink_set_err(net, portid, NFNLGRP_NFTABLES, -ENOBUFS);
}

static struct nft_trans *nft_trans_elem_alloc(struct nft_ctx *ctx,
					      int msg_type,
					      struct nft_set *set)
{
	struct nft_trans *trans;

	trans = nft_trans_alloc(ctx, msg_type, sizeof(struct nft_trans_elem));
	if (trans == NULL)
		return NULL;

	nft_trans_elem_set(trans) = set;
	return trans;
}

void *nft_set_elem_init(const struct nft_set *set,
			const struct nft_set_ext_tmpl *tmpl,
			const u32 *key, const u32 *data,
			u64 timeout, gfp_t gfp)
{
	struct nft_set_ext *ext;
	void *elem;

	elem = kzalloc(set->ops->elemsize + tmpl->len, gfp);
	if (elem == NULL)
		return NULL;

	ext = nft_set_elem_ext(set, elem);
	nft_set_ext_init(ext, tmpl);

	memcpy(nft_set_ext_key(ext), key, set->klen);
	if (nft_set_ext_exists(ext, NFT_SET_EXT_DATA))
		memcpy(nft_set_ext_data(ext), data, set->dlen);
	if (nft_set_ext_exists(ext, NFT_SET_EXT_EXPIRATION))
		*nft_set_ext_expiration(ext) =
			get_jiffies_64() + timeout;
	if (nft_set_ext_exists(ext, NFT_SET_EXT_TIMEOUT))
		*nft_set_ext_timeout(ext) = timeout;

	return elem;
}

/* Drop references and destroy. Called from gc, dynset and abort path. */
void nft_set_elem_destroy(const struct nft_set *set, void *elem,
			  bool destroy_expr)
{
	struct nft_set_ext *ext = nft_set_elem_ext(set, elem);
	struct nft_ctx ctx = {
		.net	= read_pnet(&set->net),
		.family	= set->table->family,
	};

	nft_data_release(nft_set_ext_key(ext), NFT_DATA_VALUE);
	if (nft_set_ext_exists(ext, NFT_SET_EXT_DATA))
		nft_data_release(nft_set_ext_data(ext), set->dtype);
	if (destroy_expr && nft_set_ext_exists(ext, NFT_SET_EXT_EXPR)) {
		struct nft_expr *expr = nft_set_ext_expr(ext);

		if (expr->ops->destroy_clone) {
			expr->ops->destroy_clone(&ctx, expr);
			module_put(expr->ops->type->owner);
		} else {
			nf_tables_expr_destroy(&ctx, expr);
		}
	}
	if (nft_set_ext_exists(ext, NFT_SET_EXT_OBJREF))
		nft_use_dec(&(*nft_set_ext_obj(ext))->use);
	kfree(elem);
}
EXPORT_SYMBOL_GPL(nft_set_elem_destroy);

/* Destroy element. References have been already dropped in the preparation
 * path via nft_setelem_data_deactivate().
 */
void nf_tables_set_elem_destroy(const struct nft_ctx *ctx,
				const struct nft_set *set, void *elem)
{
	struct nft_set_ext *ext = nft_set_elem_ext(set, elem);

	if (nft_set_ext_exists(ext, NFT_SET_EXT_EXPR))
		nf_tables_expr_destroy(ctx, nft_set_ext_expr(ext));
	kfree(elem);
}
EXPORT_SYMBOL_GPL(nf_tables_set_elem_destroy);

static int nft_add_set_elem(struct nft_ctx *ctx, struct nft_set *set,
			    const struct nlattr *attr, u32 nlmsg_flags)
{
	struct nlattr *nla[NFTA_SET_ELEM_MAX + 1];
	u8 genmask = nft_genmask_next(ctx->net);
	struct nft_set_ext_tmpl tmpl;
	struct nft_set_ext *ext, *ext2;
	struct nft_set_elem elem;
	struct nft_set_binding *binding;
	struct nft_object *obj = NULL;
	struct nft_userdata *udata;
	struct nft_data_desc desc;
	enum nft_registers dreg;
	struct nft_trans *trans;
	u32 flags = 0;
	u64 timeout;
	u8 ulen;
	int err;

	err = nla_parse_nested(nla, NFTA_SET_ELEM_MAX, attr,
			       nft_set_elem_policy, NULL);
	if (err < 0)
		return err;

	if (nla[NFTA_SET_ELEM_KEY] == NULL)
		return -EINVAL;

	nft_set_ext_prepare(&tmpl);

	err = nft_setelem_parse_flags(set, nla[NFTA_SET_ELEM_FLAGS], &flags);
	if (err < 0)
		return err;
	if (flags != 0)
		nft_set_ext_add(&tmpl, NFT_SET_EXT_FLAGS);

	if (set->flags & NFT_SET_MAP) {
		if (nla[NFTA_SET_ELEM_DATA] == NULL &&
		    !(flags & NFT_SET_ELEM_INTERVAL_END))
			return -EINVAL;
	} else {
		if (nla[NFTA_SET_ELEM_DATA] != NULL)
			return -EINVAL;
	}

	if (set->flags & NFT_SET_OBJECT) {
		if (!nla[NFTA_SET_ELEM_OBJREF] &&
		    !(flags & NFT_SET_ELEM_INTERVAL_END))
			return -EINVAL;
	} else {
		if (nla[NFTA_SET_ELEM_OBJREF])
			return -EINVAL;
	}

	if ((flags & NFT_SET_ELEM_INTERVAL_END) &&
	     (nla[NFTA_SET_ELEM_DATA] ||
	      nla[NFTA_SET_ELEM_OBJREF] ||
	      nla[NFTA_SET_ELEM_TIMEOUT] ||
	      nla[NFTA_SET_ELEM_EXPIRATION] ||
	      nla[NFTA_SET_ELEM_USERDATA] ||
	      nla[NFTA_SET_ELEM_EXPR]))
		return -EINVAL;

	timeout = 0;
	if (nla[NFTA_SET_ELEM_TIMEOUT] != NULL) {
		if (!(set->flags & NFT_SET_TIMEOUT))
			return -EINVAL;
		err = nf_msecs_to_jiffies64(nla[NFTA_SET_ELEM_TIMEOUT],
					    &timeout);
		if (err)
			return err;
	} else if (set->flags & NFT_SET_TIMEOUT) {
		timeout = set->timeout;
	}

	err = nft_setelem_parse_key(ctx, set, &elem.key.val,
				    nla[NFTA_SET_ELEM_KEY]);
	if (err < 0)
		goto err1;

	nft_set_ext_add_length(&tmpl, NFT_SET_EXT_KEY, set->klen);
	if (timeout > 0) {
		nft_set_ext_add(&tmpl, NFT_SET_EXT_EXPIRATION);
		if (timeout != set->timeout)
			nft_set_ext_add(&tmpl, NFT_SET_EXT_TIMEOUT);
	}

	if (nla[NFTA_SET_ELEM_OBJREF] != NULL) {
		obj = nft_obj_lookup(ctx->table, nla[NFTA_SET_ELEM_OBJREF],
				     set->objtype, genmask);
		if (IS_ERR(obj)) {
			err = PTR_ERR(obj);
			obj = NULL;
			goto err2;
		}

		if (!nft_use_inc(&obj->use)) {
			err = -EMFILE;
			obj = NULL;
			goto err2;
		}

		nft_set_ext_add(&tmpl, NFT_SET_EXT_OBJREF);
	}

	if (nla[NFTA_SET_ELEM_DATA] != NULL) {
		err = nft_setelem_parse_data(ctx, set, &desc, &elem.data.val,
					     nla[NFTA_SET_ELEM_DATA]);
		if (err < 0)
			goto err2;

		dreg = nft_type_to_reg(set->dtype);
		list_for_each_entry(binding, &set->bindings, list) {
			struct nft_ctx bind_ctx = {
				.net	= ctx->net,
				.family	= ctx->family,
				.table	= ctx->table,
				.chain	= (struct nft_chain *)binding->chain,
			};

			if (!(binding->flags & NFT_SET_MAP))
				continue;

			err = nft_validate_register_store(&bind_ctx, dreg,
							  &elem.data.val,
							  desc.type, desc.len);
			if (err < 0)
				goto err3;

			if (desc.type == NFT_DATA_VERDICT &&
			    (elem.data.val.verdict.code == NFT_GOTO ||
			     elem.data.val.verdict.code == NFT_JUMP))
				nft_validate_state_update(ctx->net,
							  NFT_VALIDATE_NEED);
		}

		nft_set_ext_add_length(&tmpl, NFT_SET_EXT_DATA, desc.len);
	}

	/* The full maximum length of userdata can exceed the maximum
	 * offset value (U8_MAX) for following extensions, therefor it
	 * must be the last extension added.
	 */
	ulen = 0;
	if (nla[NFTA_SET_ELEM_USERDATA] != NULL) {
		ulen = nla_len(nla[NFTA_SET_ELEM_USERDATA]);
		if (ulen > 0)
			nft_set_ext_add_length(&tmpl, NFT_SET_EXT_USERDATA,
					       ulen);
	}

	err = -ENOMEM;
	elem.priv = nft_set_elem_init(set, &tmpl, elem.key.val.data,
				      elem.data.val.data,
				      timeout, GFP_KERNEL);
	if (elem.priv == NULL)
		goto err3;

	ext = nft_set_elem_ext(set, elem.priv);
	if (flags)
		*nft_set_ext_flags(ext) = flags;
	if (ulen > 0) {
		udata = nft_set_ext_userdata(ext);
		udata->len = ulen - 1;
		nla_memcpy(&udata->data, nla[NFTA_SET_ELEM_USERDATA], ulen);
	}
	if (obj)
		*nft_set_ext_obj(ext) = obj;

	trans = nft_trans_elem_alloc(ctx, NFT_MSG_NEWSETELEM, set);
	if (trans == NULL)
		goto err4;

	ext->genmask = nft_genmask_cur(ctx->net);

	err = set->ops->insert(ctx->net, set, &elem, &ext2);
	if (err) {
		if (err == -EEXIST) {
			if (nft_set_ext_exists(ext, NFT_SET_EXT_DATA) ^
			    nft_set_ext_exists(ext2, NFT_SET_EXT_DATA) ||
			    nft_set_ext_exists(ext, NFT_SET_EXT_OBJREF) ^
			    nft_set_ext_exists(ext2, NFT_SET_EXT_OBJREF)) {
				err = -EBUSY;
				goto err5;
			}
			if ((nft_set_ext_exists(ext, NFT_SET_EXT_DATA) &&
			     nft_set_ext_exists(ext2, NFT_SET_EXT_DATA) &&
			     memcmp(nft_set_ext_data(ext),
				    nft_set_ext_data(ext2), set->dlen) != 0) ||
			    (nft_set_ext_exists(ext, NFT_SET_EXT_OBJREF) &&
			     nft_set_ext_exists(ext2, NFT_SET_EXT_OBJREF) &&
			     *nft_set_ext_obj(ext) != *nft_set_ext_obj(ext2)))
				err = -EBUSY;
			else if (!(nlmsg_flags & NLM_F_EXCL))
				err = 0;
		}
		goto err5;
	}

	if (set->size &&
	    !atomic_add_unless(&set->nelems, 1, set->size + set->ndeact)) {
		err = -ENFILE;
		goto err6;
	}

	nft_trans_elem(trans) = elem;
	nft_trans_commit_list_add_tail(ctx->net, trans);
	return 0;

err6:
	set->ops->remove(ctx->net, set, &elem);
err5:
	kfree(trans);
err4:
	kfree(elem.priv);
err3:
	if (nla[NFTA_SET_ELEM_DATA] != NULL)
		nft_data_release(&elem.data.val, desc.type);
err2:
	if (obj)
		nft_use_dec_restore(&obj->use);

	nft_data_release(&elem.key.val, NFT_DATA_VALUE);
err1:
	return err;
}

static int nf_tables_newsetelem(struct net *net, struct sock *nlsk,
				struct sk_buff *skb, const struct nlmsghdr *nlh,
				const struct nlattr * const nla[],
				struct netlink_ext_ack *extack)
{
	struct nftables_pernet *nft_net = net_generic(net, nf_tables_net_id);
	u8 genmask = nft_genmask_next(net);
	const struct nlattr *attr;
	struct nft_set *set;
	struct nft_ctx ctx;
	int rem, err;

	if (nla[NFTA_SET_ELEM_LIST_ELEMENTS] == NULL)
		return -EINVAL;

	err = nft_ctx_init_from_elemattr(&ctx, net, skb, nlh, nla, extack,
					 genmask);
	if (err < 0)
		return err;

	set = nft_set_lookup_global(net, ctx.table, nla[NFTA_SET_ELEM_LIST_SET],
				    nla[NFTA_SET_ELEM_LIST_SET_ID], genmask);
	if (IS_ERR(set))
		return PTR_ERR(set);

	if (!list_empty(&set->bindings) &&
	    (set->flags & (NFT_SET_CONSTANT | NFT_SET_ANONYMOUS)))
		return -EBUSY;

	nla_for_each_nested(attr, nla[NFTA_SET_ELEM_LIST_ELEMENTS], rem) {
		err = nft_add_set_elem(&ctx, set, attr, nlh->nlmsg_flags);
		if (err < 0) {
			NL_SET_BAD_ATTR(extack, attr);
			return err;
		}
	}

	if (nft_net->validate_state == NFT_VALIDATE_DO)
		return nft_table_validate(net, ctx.table);

	return 0;
}

/**
 *	nft_data_hold - hold a nft_data item
 *
 *	@data: struct nft_data to release
 *	@type: type of data
 *
 *	Hold a nft_data item. NFT_DATA_VALUE types can be silently discarded,
 *	NFT_DATA_VERDICT bumps the reference to chains in case of NFT_JUMP and
 *	NFT_GOTO verdicts. This function must be called on active data objects
 *	from the second phase of the commit protocol.
 */
void nft_data_hold(const struct nft_data *data, enum nft_data_types type)
{
	struct nft_chain *chain;

	if (type == NFT_DATA_VERDICT) {
		switch (data->verdict.code) {
		case NFT_JUMP:
		case NFT_GOTO:
			chain = data->verdict.chain;
			nft_use_inc_restore(&chain->use);
			break;
		}
	}
}

static void nft_setelem_data_activate(const struct net *net,
				      const struct nft_set *set,
				      struct nft_set_elem *elem)
{
	const struct nft_set_ext *ext = nft_set_elem_ext(set, elem->priv);

	if (nft_set_ext_exists(ext, NFT_SET_EXT_DATA))
		nft_data_hold(nft_set_ext_data(ext), set->dtype);
	if (nft_set_ext_exists(ext, NFT_SET_EXT_OBJREF))
		nft_use_inc_restore(&(*nft_set_ext_obj(ext))->use);
}

void nft_setelem_data_deactivate(const struct net *net,
				 const struct nft_set *set,
				 struct nft_set_elem *elem)
{
	const struct nft_set_ext *ext = nft_set_elem_ext(set, elem->priv);

	if (nft_set_ext_exists(ext, NFT_SET_EXT_DATA))
		nft_data_release(nft_set_ext_data(ext), set->dtype);
	if (nft_set_ext_exists(ext, NFT_SET_EXT_OBJREF))
		nft_use_dec(&(*nft_set_ext_obj(ext))->use);
}
EXPORT_SYMBOL_GPL(nft_setelem_data_deactivate);

static int nft_del_setelem(struct nft_ctx *ctx, struct nft_set *set,
			   const struct nlattr *attr)
{
	struct nlattr *nla[NFTA_SET_ELEM_MAX + 1];
	struct nft_set_ext_tmpl tmpl;
	struct nft_set_elem elem;
	struct nft_set_ext *ext;
	struct nft_trans *trans;
	u32 flags = 0;
	void *priv;
	int err;

	err = nla_parse_nested(nla, NFTA_SET_ELEM_MAX, attr,
			       nft_set_elem_policy, NULL);
	if (err < 0)
		return err;

	if (nla[NFTA_SET_ELEM_KEY] == NULL)
		return -EINVAL;

	nft_set_ext_prepare(&tmpl);

	err = nft_setelem_parse_flags(set, nla[NFTA_SET_ELEM_FLAGS], &flags);
	if (err < 0)
		return err;
	if (flags != 0)
		nft_set_ext_add(&tmpl, NFT_SET_EXT_FLAGS);

	err = nft_setelem_parse_key(ctx, set, &elem.key.val,
				    nla[NFTA_SET_ELEM_KEY]);
	if (err < 0)
		return err;

	nft_set_ext_add_length(&tmpl, NFT_SET_EXT_KEY, set->klen);

	err = -ENOMEM;
	elem.priv = nft_set_elem_init(set, &tmpl, elem.key.val.data, NULL, 0,
				      GFP_KERNEL);
	if (elem.priv == NULL)
		goto fail_elem;

	ext = nft_set_elem_ext(set, elem.priv);
	if (flags)
		*nft_set_ext_flags(ext) = flags;

	trans = nft_trans_elem_alloc(ctx, NFT_MSG_DELSETELEM, set);
	if (trans == NULL)
		goto fail_trans;

	priv = set->ops->deactivate(ctx->net, set, &elem);
	if (priv == NULL) {
		err = -ENOENT;
		goto fail_ops;
	}
	kfree(elem.priv);
	elem.priv = priv;

	nft_setelem_data_deactivate(ctx->net, set, &elem);

	nft_trans_elem(trans) = elem;
	nft_trans_commit_list_add_tail(ctx->net, trans);
	return 0;

fail_ops:
	kfree(trans);
fail_trans:
	kfree(elem.priv);
fail_elem:
	nft_data_release(&elem.key.val, NFT_DATA_VALUE);
	return err;
}

static int nft_flush_set(const struct nft_ctx *ctx,
			 struct nft_set *set,
			 const struct nft_set_iter *iter,
			 struct nft_set_elem *elem)
{
	struct nft_trans *trans;
	int err;

	trans = nft_trans_alloc_gfp(ctx, NFT_MSG_DELSETELEM,
				    sizeof(struct nft_trans_elem), GFP_ATOMIC);
	if (!trans)
		return -ENOMEM;

	if (!set->ops->flush(ctx->net, set, elem->priv)) {
		err = -ENOENT;
		goto err1;
	}
	set->ndeact++;

	nft_setelem_data_deactivate(ctx->net, set, elem);
	nft_trans_elem_set(trans) = set;
	nft_trans_elem(trans) = *elem;
	nft_trans_commit_list_add_tail(ctx->net, trans);

	return 0;
err1:
	kfree(trans);
	return err;
}

static int nf_tables_delsetelem(struct net *net, struct sock *nlsk,
				struct sk_buff *skb, const struct nlmsghdr *nlh,
				const struct nlattr * const nla[],
				struct netlink_ext_ack *extack)
{
	u8 genmask = nft_genmask_next(net);
	const struct nlattr *attr;
	struct nft_set *set;
	struct nft_ctx ctx;
	int rem, err = 0;

	err = nft_ctx_init_from_elemattr(&ctx, net, skb, nlh, nla, extack,
					 genmask);
	if (err < 0)
		return err;

	set = nft_set_lookup(ctx.table, nla[NFTA_SET_ELEM_LIST_SET], genmask);
	if (IS_ERR(set))
		return PTR_ERR(set);

	if (nft_set_is_anonymous(set))
		return -EOPNOTSUPP;

	if (!list_empty(&set->bindings) && (set->flags & NFT_SET_CONSTANT))
		return -EBUSY;

	if (nla[NFTA_SET_ELEM_LIST_ELEMENTS] == NULL) {
		struct nft_set_iter iter = {
			.genmask	= genmask,
			.fn		= nft_flush_set,
		};
		set->ops->walk(&ctx, set, &iter);

		return iter.err;
	}

	nla_for_each_nested(attr, nla[NFTA_SET_ELEM_LIST_ELEMENTS], rem) {
		err = nft_del_setelem(&ctx, set, attr);
		if (err < 0) {
			NL_SET_BAD_ATTR(extack, attr);
			break;
		}
		set->ndeact++;
	}
	return err;
}

/*
 * Stateful objects
 */

/**
 *	nft_register_obj- register nf_tables stateful object type
 *	@obj: object type
 *
 *	Registers the object type for use with nf_tables. Returns zero on
 *	success or a negative errno code otherwise.
 */
int nft_register_obj(struct nft_object_type *obj_type)
{
	if (obj_type->type == NFT_OBJECT_UNSPEC)
		return -EINVAL;

	nfnl_lock(NFNL_SUBSYS_NFTABLES);
	list_add_rcu(&obj_type->list, &nf_tables_objects);
	nfnl_unlock(NFNL_SUBSYS_NFTABLES);
	return 0;
}
EXPORT_SYMBOL_GPL(nft_register_obj);

/**
 *	nft_unregister_obj - unregister nf_tables object type
 *	@obj: object type
 *
 * 	Unregisters the object type for use with nf_tables.
 */
void nft_unregister_obj(struct nft_object_type *obj_type)
{
	nfnl_lock(NFNL_SUBSYS_NFTABLES);
	list_del_rcu(&obj_type->list);
	nfnl_unlock(NFNL_SUBSYS_NFTABLES);
}
EXPORT_SYMBOL_GPL(nft_unregister_obj);

struct nft_object *nft_obj_lookup(const struct nft_table *table,
				  const struct nlattr *nla, u32 objtype,
				  u8 genmask)
{
	struct nft_object *obj;

	list_for_each_entry_rcu(obj, &table->objects, list) {
		if (!nla_strcmp(nla, obj->name) &&
		    objtype == obj->ops->type->type &&
		    nft_active_genmask(obj, genmask))
			return obj;
	}
	return ERR_PTR(-ENOENT);
}
EXPORT_SYMBOL_GPL(nft_obj_lookup);

static struct nft_object *nft_obj_lookup_byhandle(const struct nft_table *table,
						  const struct nlattr *nla,
						  u32 objtype, u8 genmask)
{
	struct nft_object *obj;

	list_for_each_entry(obj, &table->objects, list) {
		if (be64_to_cpu(nla_get_be64(nla)) == obj->handle &&
		    objtype == obj->ops->type->type &&
		    nft_active_genmask(obj, genmask))
			return obj;
	}
	return ERR_PTR(-ENOENT);
}

static const struct nla_policy nft_obj_policy[NFTA_OBJ_MAX + 1] = {
	[NFTA_OBJ_TABLE]	= { .type = NLA_STRING,
				    .len = NFT_TABLE_MAXNAMELEN - 1 },
	[NFTA_OBJ_NAME]		= { .type = NLA_STRING,
				    .len = NFT_OBJ_MAXNAMELEN - 1 },
	[NFTA_OBJ_TYPE]		= { .type = NLA_U32 },
	[NFTA_OBJ_DATA]		= { .type = NLA_NESTED },
	[NFTA_OBJ_HANDLE]	= { .type = NLA_U64},
};

static struct nft_object *nft_obj_init(const struct nft_ctx *ctx,
				       const struct nft_object_type *type,
				       const struct nlattr *attr)
{
	struct nlattr **tb;
	const struct nft_object_ops *ops;
	struct nft_object *obj;
	int err = -ENOMEM;

	tb = kmalloc_array(type->maxattr + 1, sizeof(*tb), GFP_KERNEL);
	if (!tb)
		goto err1;

	if (attr) {
		err = nla_parse_nested(tb, type->maxattr, attr, type->policy,
				       NULL);
		if (err < 0)
			goto err2;
	} else {
		memset(tb, 0, sizeof(tb[0]) * (type->maxattr + 1));
	}

	if (type->select_ops) {
		ops = type->select_ops(ctx, (const struct nlattr * const *)tb);
		if (IS_ERR(ops)) {
			err = PTR_ERR(ops);
			goto err2;
		}
	} else {
		ops = type->ops;
	}

	err = -ENOMEM;
	obj = kzalloc(sizeof(*obj) + ops->size, GFP_KERNEL);
	if (!obj)
		goto err2;

	err = ops->init(ctx, (const struct nlattr * const *)tb, obj);
	if (err < 0)
		goto err3;

	obj->ops = ops;

	kfree(tb);
	return obj;
err3:
	kfree(obj);
err2:
	kfree(tb);
err1:
	return ERR_PTR(err);
}

static int nft_object_dump(struct sk_buff *skb, unsigned int attr,
			   struct nft_object *obj, bool reset)
{
	struct nlattr *nest;

	nest = nla_nest_start(skb, attr);
	if (!nest)
		goto nla_put_failure;
	if (obj->ops->dump(skb, obj, reset) < 0)
		goto nla_put_failure;
	nla_nest_end(skb, nest);
	return 0;

nla_put_failure:
	return -1;
}

static const struct nft_object_type *__nft_obj_type_get(u32 objtype)
{
	const struct nft_object_type *type;

	list_for_each_entry(type, &nf_tables_objects, list) {
		if (objtype == type->type)
			return type;
	}
	return NULL;
}

static const struct nft_object_type *
nft_obj_type_get(struct net *net, u32 objtype)
{
	const struct nft_object_type *type;

	type = __nft_obj_type_get(objtype);
	if (type != NULL && try_module_get(type->owner))
		return type;

	lockdep_nfnl_nft_mutex_not_held();
#ifdef CONFIG_MODULES
	if (type == NULL) {
		nft_request_module(net, "nft-obj-%u", objtype);
		if (__nft_obj_type_get(objtype))
			return ERR_PTR(-EAGAIN);
	}
#endif
	return ERR_PTR(-ENOENT);
}

static int nf_tables_newobj(struct net *net, struct sock *nlsk,
			    struct sk_buff *skb, const struct nlmsghdr *nlh,
			    const struct nlattr * const nla[],
			    struct netlink_ext_ack *extack)
{
	const struct nfgenmsg *nfmsg = nlmsg_data(nlh);
	const struct nft_object_type *type;
	u8 genmask = nft_genmask_next(net);
	int family = nfmsg->nfgen_family;
	struct nft_table *table;
	struct nft_object *obj;
	struct nft_ctx ctx;
	u32 objtype;
	int err;

	if (!nla[NFTA_OBJ_TYPE] ||
	    !nla[NFTA_OBJ_NAME] ||
	    !nla[NFTA_OBJ_DATA])
		return -EINVAL;

	table = nft_table_lookup(net, nla[NFTA_OBJ_TABLE], family, genmask);
	if (IS_ERR(table)) {
		NL_SET_BAD_ATTR(extack, nla[NFTA_OBJ_TABLE]);
		return PTR_ERR(table);
	}

	objtype = ntohl(nla_get_be32(nla[NFTA_OBJ_TYPE]));
	obj = nft_obj_lookup(table, nla[NFTA_OBJ_NAME], objtype, genmask);
	if (IS_ERR(obj)) {
		err = PTR_ERR(obj);
		if (err != -ENOENT) {
			NL_SET_BAD_ATTR(extack, nla[NFTA_OBJ_NAME]);
			return err;
		}
	} else {
		if (nlh->nlmsg_flags & NLM_F_EXCL) {
			NL_SET_BAD_ATTR(extack, nla[NFTA_OBJ_NAME]);
			return -EEXIST;
		}
		return 0;
	}

	nft_ctx_init(&ctx, net, skb, nlh, family, table, NULL, nla);

	if (!nft_use_inc(&table->use))
		return -EMFILE;

	type = nft_obj_type_get(net, objtype);
	if (IS_ERR(type)) {
		err = PTR_ERR(type);
		goto err_type;
	}

	obj = nft_obj_init(&ctx, type, nla[NFTA_OBJ_DATA]);
	if (IS_ERR(obj)) {
		err = PTR_ERR(obj);
		goto err1;
	}
	obj->table = table;
	obj->handle = nf_tables_alloc_handle(table);

	obj->name = nla_strdup(nla[NFTA_OBJ_NAME], GFP_KERNEL);
	if (!obj->name) {
		err = -ENOMEM;
		goto err2;
	}

	err = nft_trans_obj_add(&ctx, NFT_MSG_NEWOBJ, obj);
	if (err < 0)
		goto err3;

	list_add_tail_rcu(&obj->list, &table->objects);

	return 0;
err3:
	kfree(obj->name);
err2:
	if (obj->ops->destroy)
		obj->ops->destroy(&ctx, obj);
	kfree(obj);
err1:
	module_put(type->owner);
err_type:
	nft_use_dec_restore(&table->use);

	return err;
}

static int nf_tables_fill_obj_info(struct sk_buff *skb, struct net *net,
				   u32 portid, u32 seq, int event, u32 flags,
				   int family, const struct nft_table *table,
				   struct nft_object *obj, bool reset)
{
	struct nlmsghdr *nlh;

	event = nfnl_msg_type(NFNL_SUBSYS_NFTABLES, event);
	nlh = nfnl_msg_put(skb, portid, seq, event, flags, family,
			   NFNETLINK_V0, nft_base_seq(net));
	if (!nlh)
		goto nla_put_failure;

	if (nla_put_string(skb, NFTA_OBJ_TABLE, table->name) ||
	    nla_put_string(skb, NFTA_OBJ_NAME, obj->name) ||
	    nla_put_be32(skb, NFTA_OBJ_TYPE, htonl(obj->ops->type->type)) ||
	    nla_put_be32(skb, NFTA_OBJ_USE, htonl(obj->use)) ||
	    nft_object_dump(skb, NFTA_OBJ_DATA, obj, reset) ||
	    nla_put_be64(skb, NFTA_OBJ_HANDLE, cpu_to_be64(obj->handle),
			 NFTA_OBJ_PAD))
		goto nla_put_failure;

	nlmsg_end(skb, nlh);
	return 0;

nla_put_failure:
	nlmsg_trim(skb, nlh);
	return -1;
}

struct nft_obj_filter {
	char		*table;
	u32		type;
};

static int nf_tables_dump_obj(struct sk_buff *skb, struct netlink_callback *cb)
{
	const struct nfgenmsg *nfmsg = nlmsg_data(cb->nlh);
	const struct nft_table *table;
	unsigned int idx = 0, s_idx = cb->args[0];
	struct nft_obj_filter *filter = cb->data;
	struct net *net = sock_net(skb->sk);
	int family = nfmsg->nfgen_family;
	struct nftables_pernet *nft_net;
	struct nft_object *obj;
	bool reset = false;

	if (NFNL_MSG_TYPE(cb->nlh->nlmsg_type) == NFT_MSG_GETOBJ_RESET)
		reset = true;

	rcu_read_lock();
	nft_net = net_generic(net, nf_tables_net_id);
	cb->seq = nft_net->base_seq;

	list_for_each_entry_rcu(table, &nft_net->tables, list) {
		if (family != NFPROTO_UNSPEC && family != table->family)
			continue;

		list_for_each_entry_rcu(obj, &table->objects, list) {
			if (!nft_is_active(net, obj))
				goto cont;
			if (idx < s_idx)
				goto cont;
			if (idx > s_idx)
				memset(&cb->args[1], 0,
				       sizeof(cb->args) - sizeof(cb->args[0]));
			if (filter && filter->table &&
			    strcmp(filter->table, table->name))
				goto cont;
			if (filter &&
			    filter->type != NFT_OBJECT_UNSPEC &&
			    obj->ops->type->type != filter->type)
				goto cont;

			if (nf_tables_fill_obj_info(skb, net, NETLINK_CB(cb->skb).portid,
						    cb->nlh->nlmsg_seq,
						    NFT_MSG_NEWOBJ,
						    NLM_F_MULTI | NLM_F_APPEND,
						    table->family, table,
						    obj, reset) < 0)
				goto done;

			nl_dump_check_consistent(cb, nlmsg_hdr(skb));
cont:
			idx++;
		}
	}
done:
	rcu_read_unlock();

	cb->args[0] = idx;
	return skb->len;
}

static int nf_tables_dump_obj_start(struct netlink_callback *cb)
{
	const struct nlattr * const *nla = cb->data;
	struct nft_obj_filter *filter = NULL;

	if (nla[NFTA_OBJ_TABLE] || nla[NFTA_OBJ_TYPE]) {
		filter = kzalloc(sizeof(*filter), GFP_ATOMIC);
		if (!filter)
			return -ENOMEM;

		if (nla[NFTA_OBJ_TABLE]) {
			filter->table = nla_strdup(nla[NFTA_OBJ_TABLE], GFP_ATOMIC);
			if (!filter->table) {
				kfree(filter);
				return -ENOMEM;
			}
		}

		if (nla[NFTA_OBJ_TYPE])
			filter->type = ntohl(nla_get_be32(nla[NFTA_OBJ_TYPE]));
	}

	cb->data = filter;
	return 0;
}

static int nf_tables_dump_obj_done(struct netlink_callback *cb)
{
	struct nft_obj_filter *filter = cb->data;

	if (filter) {
		kfree(filter->table);
		kfree(filter);
	}

	return 0;
}

/* called with rcu_read_lock held */
static int nf_tables_getobj(struct net *net, struct sock *nlsk,
			    struct sk_buff *skb, const struct nlmsghdr *nlh,
			    const struct nlattr * const nla[],
			    struct netlink_ext_ack *extack)
{
	const struct nfgenmsg *nfmsg = nlmsg_data(nlh);
	u8 genmask = nft_genmask_cur(net);
	int family = nfmsg->nfgen_family;
	const struct nft_table *table;
	struct nft_object *obj;
	struct sk_buff *skb2;
	bool reset = false;
	u32 objtype;
	int err;

	if (nlh->nlmsg_flags & NLM_F_DUMP) {
		struct netlink_dump_control c = {
			.start = nf_tables_dump_obj_start,
			.dump = nf_tables_dump_obj,
			.done = nf_tables_dump_obj_done,
			.module = THIS_MODULE,
			.data = (void *)nla,
		};

		return nft_netlink_dump_start_rcu(nlsk, skb, nlh, &c);
	}

	if (!nla[NFTA_OBJ_NAME] ||
	    !nla[NFTA_OBJ_TYPE])
		return -EINVAL;

	table = nft_table_lookup(net, nla[NFTA_OBJ_TABLE], family, genmask);
	if (IS_ERR(table)) {
		NL_SET_BAD_ATTR(extack, nla[NFTA_OBJ_TABLE]);
		return PTR_ERR(table);
	}

	objtype = ntohl(nla_get_be32(nla[NFTA_OBJ_TYPE]));
	obj = nft_obj_lookup(table, nla[NFTA_OBJ_NAME], objtype, genmask);
	if (IS_ERR(obj)) {
		NL_SET_BAD_ATTR(extack, nla[NFTA_OBJ_NAME]);
		return PTR_ERR(obj);
	}

	skb2 = alloc_skb(NLMSG_GOODSIZE, GFP_ATOMIC);
	if (!skb2)
		return -ENOMEM;

	if (NFNL_MSG_TYPE(nlh->nlmsg_type) == NFT_MSG_GETOBJ_RESET)
		reset = true;

	err = nf_tables_fill_obj_info(skb2, net, NETLINK_CB(skb).portid,
				      nlh->nlmsg_seq, NFT_MSG_NEWOBJ, 0,
				      family, table, obj, reset);
	if (err < 0)
		goto err_fill_obj_info;

	return nfnetlink_unicast(skb2, net, NETLINK_CB(skb).portid);

err_fill_obj_info:
	kfree_skb(skb2);
	return err;
}

static void nft_obj_destroy(const struct nft_ctx *ctx, struct nft_object *obj)
{
	if (obj->ops->destroy)
		obj->ops->destroy(ctx, obj);

	module_put(obj->ops->type->owner);
	kfree(obj->name);
	kfree(obj);
}

static int nf_tables_delobj(struct net *net, struct sock *nlsk,
			    struct sk_buff *skb, const struct nlmsghdr *nlh,
			    const struct nlattr * const nla[],
			    struct netlink_ext_ack *extack)
{
	const struct nfgenmsg *nfmsg = nlmsg_data(nlh);
	u8 genmask = nft_genmask_next(net);
	int family = nfmsg->nfgen_family;
	const struct nlattr *attr;
	struct nft_table *table;
	struct nft_object *obj;
	struct nft_ctx ctx;
	u32 objtype;

	if (!nla[NFTA_OBJ_TYPE] ||
	    (!nla[NFTA_OBJ_NAME] && !nla[NFTA_OBJ_HANDLE]))
		return -EINVAL;

	table = nft_table_lookup(net, nla[NFTA_OBJ_TABLE], family, genmask);
	if (IS_ERR(table)) {
		NL_SET_BAD_ATTR(extack, nla[NFTA_OBJ_TABLE]);
		return PTR_ERR(table);
	}

	objtype = ntohl(nla_get_be32(nla[NFTA_OBJ_TYPE]));
	if (nla[NFTA_OBJ_HANDLE]) {
		attr = nla[NFTA_OBJ_HANDLE];
		obj = nft_obj_lookup_byhandle(table, attr, objtype, genmask);
	} else {
		attr = nla[NFTA_OBJ_NAME];
		obj = nft_obj_lookup(table, attr, objtype, genmask);
	}

	if (IS_ERR(obj)) {
		NL_SET_BAD_ATTR(extack, attr);
		return PTR_ERR(obj);
	}
	if (obj->use > 0) {
		NL_SET_BAD_ATTR(extack, attr);
		return -EBUSY;
	}

	nft_ctx_init(&ctx, net, skb, nlh, family, table, NULL, nla);

	return nft_delobj(&ctx, obj);
}

void nft_obj_notify(struct net *net, struct nft_table *table,
		    struct nft_object *obj, u32 portid, u32 seq, int event,
		    int family, int report, gfp_t gfp)
{
	struct sk_buff *skb;
	int err;

	if (!report &&
	    !nfnetlink_has_listeners(net, NFNLGRP_NFTABLES))
		return;

	skb = nlmsg_new(NLMSG_GOODSIZE, gfp);
	if (skb == NULL)
		goto err;

	err = nf_tables_fill_obj_info(skb, net, portid, seq, event, 0, family,
				      table, obj, false);
	if (err < 0) {
		kfree_skb(skb);
		goto err;
	}

	nfnetlink_send(skb, net, portid, NFNLGRP_NFTABLES, report, gfp);
	return;
err:
	nfnetlink_set_err(net, portid, NFNLGRP_NFTABLES, -ENOBUFS);
}
EXPORT_SYMBOL_GPL(nft_obj_notify);

static void nf_tables_obj_notify(const struct nft_ctx *ctx,
				 struct nft_object *obj, int event)
{
	nft_obj_notify(ctx->net, ctx->table, obj, ctx->portid, ctx->seq, event,
		       ctx->family, ctx->report, GFP_KERNEL);
}

/*
 * Flow tables
 */
void nft_register_flowtable_type(struct nf_flowtable_type *type)
{
	nfnl_lock(NFNL_SUBSYS_NFTABLES);
	list_add_tail_rcu(&type->list, &nf_tables_flowtables);
	nfnl_unlock(NFNL_SUBSYS_NFTABLES);
}
EXPORT_SYMBOL_GPL(nft_register_flowtable_type);

void nft_unregister_flowtable_type(struct nf_flowtable_type *type)
{
	nfnl_lock(NFNL_SUBSYS_NFTABLES);
	list_del_rcu(&type->list);
	nfnl_unlock(NFNL_SUBSYS_NFTABLES);
}
EXPORT_SYMBOL_GPL(nft_unregister_flowtable_type);

static const struct nla_policy nft_flowtable_policy[NFTA_FLOWTABLE_MAX + 1] = {
	[NFTA_FLOWTABLE_TABLE]		= { .type = NLA_STRING,
					    .len = NFT_NAME_MAXLEN - 1 },
	[NFTA_FLOWTABLE_NAME]		= { .type = NLA_STRING,
					    .len = NFT_NAME_MAXLEN - 1 },
	[NFTA_FLOWTABLE_HOOK]		= { .type = NLA_NESTED },
	[NFTA_FLOWTABLE_HANDLE]		= { .type = NLA_U64 },
};

struct nft_flowtable *nft_flowtable_lookup(const struct nft_table *table,
					   const struct nlattr *nla, u8 genmask)
{
	struct nft_flowtable *flowtable;

	list_for_each_entry_rcu(flowtable, &table->flowtables, list) {
		if (!nla_strcmp(nla, flowtable->name) &&
		    nft_active_genmask(flowtable, genmask))
			return flowtable;
	}
	return ERR_PTR(-ENOENT);
}
EXPORT_SYMBOL_GPL(nft_flowtable_lookup);

void nf_tables_deactivate_flowtable(const struct nft_ctx *ctx,
				    struct nft_flowtable *flowtable,
				    enum nft_trans_phase phase)
{
	switch (phase) {
	case NFT_TRANS_PREPARE_ERROR:
	case NFT_TRANS_PREPARE:
	case NFT_TRANS_ABORT:
	case NFT_TRANS_RELEASE:
		nft_use_dec(&flowtable->use);
		/* fall through */
	default:
		return;
	}
}
EXPORT_SYMBOL_GPL(nf_tables_deactivate_flowtable);

static struct nft_flowtable *
nft_flowtable_lookup_byhandle(const struct nft_table *table,
			      const struct nlattr *nla, u8 genmask)
{
       struct nft_flowtable *flowtable;

       list_for_each_entry(flowtable, &table->flowtables, list) {
               if (be64_to_cpu(nla_get_be64(nla)) == flowtable->handle &&
                   nft_active_genmask(flowtable, genmask))
                       return flowtable;
       }
       return ERR_PTR(-ENOENT);
}

static int nf_tables_parse_devices(const struct nft_ctx *ctx,
				   const struct nlattr *attr,
				   struct net_device *dev_array[], int *len)
{
	const struct nlattr *tmp;
	struct net_device *dev;
	char ifname[IFNAMSIZ];
	int rem, n = 0, err;

	nla_for_each_nested(tmp, attr, rem) {
		if (nla_type(tmp) != NFTA_DEVICE_NAME) {
			err = -EINVAL;
			goto err1;
		}

		nla_strlcpy(ifname, tmp, IFNAMSIZ);
		dev = __dev_get_by_name(ctx->net, ifname);
		if (!dev) {
			err = -ENOENT;
			goto err1;
		}

		dev_array[n++] = dev;
		if (n == NFT_FLOWTABLE_DEVICE_MAX) {
			err = -EFBIG;
			goto err1;
		}
	}
	if (!len)
		return -EINVAL;

	err = 0;
err1:
	*len = n;
	return err;
}

static const struct nla_policy nft_flowtable_hook_policy[NFTA_FLOWTABLE_HOOK_MAX + 1] = {
	[NFTA_FLOWTABLE_HOOK_NUM]	= { .type = NLA_U32 },
	[NFTA_FLOWTABLE_HOOK_PRIORITY]	= { .type = NLA_U32 },
	[NFTA_FLOWTABLE_HOOK_DEVS]	= { .type = NLA_NESTED },
};

static int nf_tables_flowtable_parse_hook(const struct nft_ctx *ctx,
					  const struct nlattr *attr,
					  struct nft_flowtable *flowtable)
{
	struct net_device *dev_array[NFT_FLOWTABLE_DEVICE_MAX];
	struct nlattr *tb[NFTA_FLOWTABLE_HOOK_MAX + 1];
	struct nf_hook_ops *ops;
	int hooknum, priority;
	int err, n = 0, i;

	err = nla_parse_nested(tb, NFTA_FLOWTABLE_HOOK_MAX, attr,
			       nft_flowtable_hook_policy, NULL);
	if (err < 0)
		return err;

	if (!tb[NFTA_FLOWTABLE_HOOK_NUM] ||
	    !tb[NFTA_FLOWTABLE_HOOK_PRIORITY] ||
	    !tb[NFTA_FLOWTABLE_HOOK_DEVS])
		return -EINVAL;

	hooknum = ntohl(nla_get_be32(tb[NFTA_FLOWTABLE_HOOK_NUM]));
	if (hooknum != NF_NETDEV_INGRESS)
		return -EINVAL;

	priority = ntohl(nla_get_be32(tb[NFTA_FLOWTABLE_HOOK_PRIORITY]));

	err = nf_tables_parse_devices(ctx, tb[NFTA_FLOWTABLE_HOOK_DEVS],
				      dev_array, &n);
	if (err < 0)
		return err;

	ops = kcalloc(n, sizeof(struct nf_hook_ops), GFP_KERNEL);
	if (!ops)
		return -ENOMEM;

	flowtable->hooknum	= hooknum;
	flowtable->priority	= priority;
	flowtable->ops		= ops;
	flowtable->ops_len	= n;

	for (i = 0; i < n; i++) {
		flowtable->ops[i].pf		= NFPROTO_NETDEV;
		flowtable->ops[i].hooknum	= hooknum;
		flowtable->ops[i].priority	= priority;
		flowtable->ops[i].priv		= &flowtable->data;
		flowtable->ops[i].hook		= flowtable->data.type->hook;
		flowtable->ops[i].dev		= dev_array[i];
	}

	return err;
}

/* call under rcu_read_lock */
static const struct nf_flowtable_type *__nft_flowtable_type_get(u8 family)
{
	const struct nf_flowtable_type *type;

	list_for_each_entry_rcu(type, &nf_tables_flowtables, list) {
		if (family == type->family)
			return type;
	}
	return NULL;
}

static const struct nf_flowtable_type *
nft_flowtable_type_get(struct net *net, u8 family)
{
	const struct nf_flowtable_type *type;

	rcu_read_lock();
	type = __nft_flowtable_type_get(family);
	if (type != NULL && try_module_get(type->owner)) {
		rcu_read_unlock();
		return type;
	}
	rcu_read_unlock();

	lockdep_nfnl_nft_mutex_not_held();
#ifdef CONFIG_MODULES
	if (type == NULL) {
		nft_request_module(net, "nf-flowtable-%u", family);
		if (__nft_flowtable_type_get(family))
			return ERR_PTR(-EAGAIN);
	}
#endif
	return ERR_PTR(-ENOENT);
}

static void __nft_unregister_flowtable_net_hooks(struct net *net,
						 struct nft_flowtable *flowtable,
						 bool release_netdev)
{
	int i;

	for (i = 0; i < flowtable->ops_len; i++) {
		if (!flowtable->ops[i].dev)
			continue;

		nf_unregister_net_hook(net, &flowtable->ops[i]);
		if (release_netdev)
			flowtable->ops[i].dev = NULL;
	}
}

static void nft_unregister_flowtable_net_hooks(struct net *net,
					       struct nft_flowtable *flowtable)
{
	__nft_unregister_flowtable_net_hooks(net, flowtable, false);
}

static int nf_tables_newflowtable(struct net *net, struct sock *nlsk,
				  struct sk_buff *skb,
				  const struct nlmsghdr *nlh,
				  const struct nlattr * const nla[],
				  struct netlink_ext_ack *extack)
{
	const struct nfgenmsg *nfmsg = nlmsg_data(nlh);
	const struct nf_flowtable_type *type;
	struct nft_flowtable *flowtable, *ft;
	u8 genmask = nft_genmask_next(net);
	int family = nfmsg->nfgen_family;
	struct nft_table *table;
	struct nft_ctx ctx;
	int err, i, k;

	if (!nla[NFTA_FLOWTABLE_TABLE] ||
	    !nla[NFTA_FLOWTABLE_NAME] ||
	    !nla[NFTA_FLOWTABLE_HOOK])
		return -EINVAL;

	table = nft_table_lookup(net, nla[NFTA_FLOWTABLE_TABLE], family,
				 genmask);
	if (IS_ERR(table)) {
		NL_SET_BAD_ATTR(extack, nla[NFTA_FLOWTABLE_TABLE]);
		return PTR_ERR(table);
	}

	flowtable = nft_flowtable_lookup(table, nla[NFTA_FLOWTABLE_NAME],
					 genmask);
	if (IS_ERR(flowtable)) {
		err = PTR_ERR(flowtable);
		if (err != -ENOENT) {
			NL_SET_BAD_ATTR(extack, nla[NFTA_FLOWTABLE_NAME]);
			return err;
		}
	} else {
		if (nlh->nlmsg_flags & NLM_F_EXCL) {
			NL_SET_BAD_ATTR(extack, nla[NFTA_FLOWTABLE_NAME]);
			return -EEXIST;
		}

		return 0;
	}

	nft_ctx_init(&ctx, net, skb, nlh, family, table, NULL, nla);

	if (!nft_use_inc(&table->use))
		return -EMFILE;

	flowtable = kzalloc(sizeof(*flowtable), GFP_KERNEL);
	if (!flowtable) {
		err = -ENOMEM;
		goto flowtable_alloc;
	}

	flowtable->table = table;
	flowtable->handle = nf_tables_alloc_handle(table);

	flowtable->name = nla_strdup(nla[NFTA_FLOWTABLE_NAME], GFP_KERNEL);
	if (!flowtable->name) {
		err = -ENOMEM;
		goto err1;
	}

	type = nft_flowtable_type_get(net, family);
	if (IS_ERR(type)) {
		err = PTR_ERR(type);
		goto err2;
	}

	flowtable->data.type = type;
	err = type->init(&flowtable->data);
	if (err < 0)
		goto err3;

	err = nf_tables_flowtable_parse_hook(&ctx, nla[NFTA_FLOWTABLE_HOOK],
					     flowtable);
	if (err < 0)
		goto err4;

	for (i = 0; i < flowtable->ops_len; i++) {
		if (!flowtable->ops[i].dev)
			continue;

		list_for_each_entry(ft, &table->flowtables, list) {
			if (!nft_is_active_next(net, ft))
				continue;

			for (k = 0; k < ft->ops_len; k++) {
				if (!ft->ops[k].dev)
					continue;

				if (flowtable->ops[i].dev == ft->ops[k].dev &&
				    flowtable->ops[i].pf == ft->ops[k].pf) {
					err = -EBUSY;
					goto err5;
				}
			}
		}

		err = nf_register_net_hook(net, &flowtable->ops[i]);
		if (err < 0)
			goto err5;
	}

	err = nft_trans_flowtable_add(&ctx, NFT_MSG_NEWFLOWTABLE, flowtable);
	if (err < 0)
		goto err6;

	list_add_tail_rcu(&flowtable->list, &table->flowtables);

	return 0;
err6:
	i = flowtable->ops_len;
err5:
	for (k = i - 1; k >= 0; k--)
		nf_unregister_net_hook(net, &flowtable->ops[k]);

	kfree(flowtable->ops);
err4:
	flowtable->data.type->free(&flowtable->data);
err3:
	module_put(type->owner);
err2:
	kfree(flowtable->name);
err1:
	kfree(flowtable);
flowtable_alloc:
	nft_use_dec_restore(&table->use);

	return err;
}

static int nf_tables_delflowtable(struct net *net, struct sock *nlsk,
				  struct sk_buff *skb,
				  const struct nlmsghdr *nlh,
				  const struct nlattr * const nla[],
				  struct netlink_ext_ack *extack)
{
	const struct nfgenmsg *nfmsg = nlmsg_data(nlh);
	u8 genmask = nft_genmask_next(net);
	int family = nfmsg->nfgen_family;
	struct nft_flowtable *flowtable;
	const struct nlattr *attr;
	struct nft_table *table;
	struct nft_ctx ctx;

	if (!nla[NFTA_FLOWTABLE_TABLE] ||
	    (!nla[NFTA_FLOWTABLE_NAME] &&
	     !nla[NFTA_FLOWTABLE_HANDLE]))
		return -EINVAL;

	table = nft_table_lookup(net, nla[NFTA_FLOWTABLE_TABLE], family,
				 genmask);
	if (IS_ERR(table)) {
		NL_SET_BAD_ATTR(extack, nla[NFTA_FLOWTABLE_TABLE]);
		return PTR_ERR(table);
	}

	if (nla[NFTA_FLOWTABLE_HANDLE]) {
		attr = nla[NFTA_FLOWTABLE_HANDLE];
		flowtable = nft_flowtable_lookup_byhandle(table, attr, genmask);
	} else {
		attr = nla[NFTA_FLOWTABLE_NAME];
		flowtable = nft_flowtable_lookup(table, attr, genmask);
	}

	if (IS_ERR(flowtable)) {
		NL_SET_BAD_ATTR(extack, attr);
		return PTR_ERR(flowtable);
	}
	if (flowtable->use > 0) {
		NL_SET_BAD_ATTR(extack, attr);
		return -EBUSY;
	}

	nft_ctx_init(&ctx, net, skb, nlh, family, table, NULL, nla);

	return nft_delflowtable(&ctx, flowtable);
}

static int nf_tables_fill_flowtable_info(struct sk_buff *skb, struct net *net,
					 u32 portid, u32 seq, int event,
					 u32 flags, int family,
					 struct nft_flowtable *flowtable)
{
	struct nlattr *nest, *nest_devs;
	struct nlmsghdr *nlh;
	int i;

	event = nfnl_msg_type(NFNL_SUBSYS_NFTABLES, event);
	nlh = nfnl_msg_put(skb, portid, seq, event, flags, family,
			   NFNETLINK_V0, nft_base_seq(net));
	if (!nlh)
		goto nla_put_failure;

	if (nla_put_string(skb, NFTA_FLOWTABLE_TABLE, flowtable->table->name) ||
	    nla_put_string(skb, NFTA_FLOWTABLE_NAME, flowtable->name) ||
	    nla_put_be32(skb, NFTA_FLOWTABLE_USE, htonl(flowtable->use)) ||
	    nla_put_be64(skb, NFTA_FLOWTABLE_HANDLE, cpu_to_be64(flowtable->handle),
			 NFTA_FLOWTABLE_PAD))
		goto nla_put_failure;

	nest = nla_nest_start(skb, NFTA_FLOWTABLE_HOOK);
	if (!nest)
		goto nla_put_failure;
	if (nla_put_be32(skb, NFTA_FLOWTABLE_HOOK_NUM, htonl(flowtable->hooknum)) ||
	    nla_put_be32(skb, NFTA_FLOWTABLE_HOOK_PRIORITY, htonl(flowtable->priority)))
		goto nla_put_failure;

	nest_devs = nla_nest_start(skb, NFTA_FLOWTABLE_HOOK_DEVS);
	if (!nest_devs)
		goto nla_put_failure;

	for (i = 0; i < flowtable->ops_len; i++) {
		const struct net_device *dev = READ_ONCE(flowtable->ops[i].dev);

		if (dev &&
		    nla_put_string(skb, NFTA_DEVICE_NAME, dev->name))
			goto nla_put_failure;
	}
	nla_nest_end(skb, nest_devs);
	nla_nest_end(skb, nest);

	nlmsg_end(skb, nlh);
	return 0;

nla_put_failure:
	nlmsg_trim(skb, nlh);
	return -1;
}

struct nft_flowtable_filter {
	char		*table;
};

static int nf_tables_dump_flowtable(struct sk_buff *skb,
				    struct netlink_callback *cb)
{
	const struct nfgenmsg *nfmsg = nlmsg_data(cb->nlh);
	struct nft_flowtable_filter *filter = cb->data;
	unsigned int idx = 0, s_idx = cb->args[0];
	struct net *net = sock_net(skb->sk);
	int family = nfmsg->nfgen_family;
	struct nft_flowtable *flowtable;
	struct nftables_pernet *nft_net;
	const struct nft_table *table;

	rcu_read_lock();
	nft_net = net_generic(net, nf_tables_net_id);
	cb->seq = nft_net->base_seq;

	list_for_each_entry_rcu(table, &nft_net->tables, list) {
		if (family != NFPROTO_UNSPEC && family != table->family)
			continue;

		list_for_each_entry_rcu(flowtable, &table->flowtables, list) {
			if (!nft_is_active(net, flowtable))
				goto cont;
			if (idx < s_idx)
				goto cont;
			if (idx > s_idx)
				memset(&cb->args[1], 0,
				       sizeof(cb->args) - sizeof(cb->args[0]));
			if (filter && filter->table &&
			    strcmp(filter->table, table->name))
				goto cont;

			if (nf_tables_fill_flowtable_info(skb, net, NETLINK_CB(cb->skb).portid,
							  cb->nlh->nlmsg_seq,
							  NFT_MSG_NEWFLOWTABLE,
							  NLM_F_MULTI | NLM_F_APPEND,
							  table->family, flowtable) < 0)
				goto done;

			nl_dump_check_consistent(cb, nlmsg_hdr(skb));
cont:
			idx++;
		}
	}
done:
	rcu_read_unlock();

	cb->args[0] = idx;
	return skb->len;
}

static int nf_tables_dump_flowtable_start(struct netlink_callback *cb)
{
	const struct nlattr * const *nla = cb->data;
	struct nft_flowtable_filter *filter = NULL;

	if (nla[NFTA_FLOWTABLE_TABLE]) {
		filter = kzalloc(sizeof(*filter), GFP_ATOMIC);
		if (!filter)
			return -ENOMEM;

		filter->table = nla_strdup(nla[NFTA_FLOWTABLE_TABLE],
					   GFP_ATOMIC);
		if (!filter->table) {
			kfree(filter);
			return -ENOMEM;
		}
	}

	cb->data = filter;
	return 0;
}

static int nf_tables_dump_flowtable_done(struct netlink_callback *cb)
{
	struct nft_flowtable_filter *filter = cb->data;

	if (!filter)
		return 0;

	kfree(filter->table);
	kfree(filter);

	return 0;
}

/* called with rcu_read_lock held */
static int nf_tables_getflowtable(struct net *net, struct sock *nlsk,
				  struct sk_buff *skb,
				  const struct nlmsghdr *nlh,
				  const struct nlattr * const nla[],
				  struct netlink_ext_ack *extack)
{
	const struct nfgenmsg *nfmsg = nlmsg_data(nlh);
	u8 genmask = nft_genmask_cur(net);
	int family = nfmsg->nfgen_family;
	struct nft_flowtable *flowtable;
	const struct nft_table *table;
	struct sk_buff *skb2;
	int err;

	if (nlh->nlmsg_flags & NLM_F_DUMP) {
		struct netlink_dump_control c = {
			.start = nf_tables_dump_flowtable_start,
			.dump = nf_tables_dump_flowtable,
			.done = nf_tables_dump_flowtable_done,
			.module = THIS_MODULE,
			.data = (void *)nla,
		};

		return nft_netlink_dump_start_rcu(nlsk, skb, nlh, &c);
	}

	if (!nla[NFTA_FLOWTABLE_NAME])
		return -EINVAL;

	table = nft_table_lookup(net, nla[NFTA_FLOWTABLE_TABLE], family,
				 genmask);
	if (IS_ERR(table))
		return PTR_ERR(table);

	flowtable = nft_flowtable_lookup(table, nla[NFTA_FLOWTABLE_NAME],
					 genmask);
	if (IS_ERR(flowtable))
		return PTR_ERR(flowtable);

	skb2 = alloc_skb(NLMSG_GOODSIZE, GFP_ATOMIC);
	if (!skb2)
		return -ENOMEM;

	err = nf_tables_fill_flowtable_info(skb2, net, NETLINK_CB(skb).portid,
					    nlh->nlmsg_seq,
					    NFT_MSG_NEWFLOWTABLE, 0, family,
					    flowtable);
	if (err < 0)
		goto err_fill_flowtable_info;

	return nfnetlink_unicast(skb2, net, NETLINK_CB(skb).portid);

err_fill_flowtable_info:
	kfree_skb(skb2);
	return err;
}

static void nf_tables_flowtable_notify(struct nft_ctx *ctx,
				       struct nft_flowtable *flowtable,
				       int event)
{
	struct sk_buff *skb;
	int err;

	if (ctx->report &&
	    !nfnetlink_has_listeners(ctx->net, NFNLGRP_NFTABLES))
		return;

	skb = nlmsg_new(NLMSG_GOODSIZE, GFP_KERNEL);
	if (skb == NULL)
		goto err;

	err = nf_tables_fill_flowtable_info(skb, ctx->net, ctx->portid,
					    ctx->seq, event, 0,
					    ctx->family, flowtable);
	if (err < 0) {
		kfree_skb(skb);
		goto err;
	}

	nfnetlink_send(skb, ctx->net, ctx->portid, NFNLGRP_NFTABLES,
		       ctx->report, GFP_KERNEL);
	return;
err:
	nfnetlink_set_err(ctx->net, ctx->portid, NFNLGRP_NFTABLES, -ENOBUFS);
}

static void nf_tables_flowtable_destroy(struct nft_flowtable *flowtable)
{
	kfree(flowtable->ops);
	kfree(flowtable->name);
	flowtable->data.type->free(&flowtable->data);
	module_put(flowtable->data.type->owner);
	kfree(flowtable);
}

static int nf_tables_fill_gen_info(struct sk_buff *skb, struct net *net,
				   u32 portid, u32 seq)
{
	struct nftables_pernet *nft_net = net_generic(net, nf_tables_net_id);
	struct nlmsghdr *nlh;
	char buf[TASK_COMM_LEN];
	int event = nfnl_msg_type(NFNL_SUBSYS_NFTABLES, NFT_MSG_NEWGEN);

	nlh = nfnl_msg_put(skb, portid, seq, event, 0, AF_UNSPEC,
			   NFNETLINK_V0, nft_base_seq(net));
	if (!nlh)
		goto nla_put_failure;

	if (nla_put_be32(skb, NFTA_GEN_ID, htonl(nft_net->base_seq)) ||
	    nla_put_be32(skb, NFTA_GEN_PROC_PID, htonl(task_pid_nr(current))) ||
	    nla_put_string(skb, NFTA_GEN_PROC_NAME, get_task_comm(buf, current)))
		goto nla_put_failure;

	nlmsg_end(skb, nlh);
	return 0;

nla_put_failure:
	nlmsg_trim(skb, nlh);
	return -EMSGSIZE;
}

static void nft_flowtable_event(unsigned long event, struct net_device *dev,
				struct nft_flowtable *flowtable)
{
	int i;

	for (i = 0; i < flowtable->ops_len; i++) {
		if (flowtable->ops[i].dev != dev)
			continue;

		nf_unregister_net_hook(dev_net(dev), &flowtable->ops[i]);
		flowtable->ops[i].dev = NULL;
		break;
	}
}

static int nf_tables_flowtable_event(struct notifier_block *this,
				     unsigned long event, void *ptr)
{
	struct net_device *dev = netdev_notifier_info_to_dev(ptr);
	struct nft_flowtable *flowtable;
	struct nftables_pernet *nft_net;
	struct nft_table *table;
	struct net *net;

	if (event != NETDEV_UNREGISTER)
		return 0;

	net = dev_net(dev);
	nft_net = net_generic(net, nf_tables_net_id);
	mutex_lock(&nft_net->commit_mutex);
	list_for_each_entry(table, &nft_net->tables, list) {
		list_for_each_entry(flowtable, &table->flowtables, list) {
			nft_flowtable_event(event, dev, flowtable);
		}
	}
	mutex_unlock(&nft_net->commit_mutex);

	return NOTIFY_DONE;
}

static struct notifier_block nf_tables_flowtable_notifier = {
	.notifier_call	= nf_tables_flowtable_event,
};

static void nf_tables_gen_notify(struct net *net, struct sk_buff *skb,
				 int event)
{
	struct nlmsghdr *nlh = nlmsg_hdr(skb);
	struct sk_buff *skb2;
	int err;

	if (nlmsg_report(nlh) &&
	    !nfnetlink_has_listeners(net, NFNLGRP_NFTABLES))
		return;

	skb2 = nlmsg_new(NLMSG_GOODSIZE, GFP_KERNEL);
	if (skb2 == NULL)
		goto err;

	err = nf_tables_fill_gen_info(skb2, net, NETLINK_CB(skb).portid,
				      nlh->nlmsg_seq);
	if (err < 0) {
		kfree_skb(skb2);
		goto err;
	}

	nfnetlink_send(skb2, net, NETLINK_CB(skb).portid, NFNLGRP_NFTABLES,
		       nlmsg_report(nlh), GFP_KERNEL);
	return;
err:
	nfnetlink_set_err(net, NETLINK_CB(skb).portid, NFNLGRP_NFTABLES,
			  -ENOBUFS);
}

static int nf_tables_getgen(struct net *net, struct sock *nlsk,
			    struct sk_buff *skb, const struct nlmsghdr *nlh,
			    const struct nlattr * const nla[],
			    struct netlink_ext_ack *extack)
{
	struct sk_buff *skb2;
	int err;

	skb2 = alloc_skb(NLMSG_GOODSIZE, GFP_ATOMIC);
	if (skb2 == NULL)
		return -ENOMEM;

	err = nf_tables_fill_gen_info(skb2, net, NETLINK_CB(skb).portid,
				      nlh->nlmsg_seq);
	if (err < 0)
		goto err_fill_gen_info;

	return nfnetlink_unicast(skb2, net, NETLINK_CB(skb).portid);

err_fill_gen_info:
	kfree_skb(skb2);
	return err;
}

static const struct nfnl_callback nf_tables_cb[NFT_MSG_MAX] = {
	[NFT_MSG_NEWTABLE] = {
		.call_batch	= nf_tables_newtable,
		.attr_count	= NFTA_TABLE_MAX,
		.policy		= nft_table_policy,
	},
	[NFT_MSG_GETTABLE] = {
		.call_rcu	= nf_tables_gettable,
		.attr_count	= NFTA_TABLE_MAX,
		.policy		= nft_table_policy,
	},
	[NFT_MSG_DELTABLE] = {
		.call_batch	= nf_tables_deltable,
		.attr_count	= NFTA_TABLE_MAX,
		.policy		= nft_table_policy,
	},
	[NFT_MSG_NEWCHAIN] = {
		.call_batch	= nf_tables_newchain,
		.attr_count	= NFTA_CHAIN_MAX,
		.policy		= nft_chain_policy,
	},
	[NFT_MSG_GETCHAIN] = {
		.call_rcu	= nf_tables_getchain,
		.attr_count	= NFTA_CHAIN_MAX,
		.policy		= nft_chain_policy,
	},
	[NFT_MSG_DELCHAIN] = {
		.call_batch	= nf_tables_delchain,
		.attr_count	= NFTA_CHAIN_MAX,
		.policy		= nft_chain_policy,
	},
	[NFT_MSG_NEWRULE] = {
		.call_batch	= nf_tables_newrule,
		.attr_count	= NFTA_RULE_MAX,
		.policy		= nft_rule_policy,
	},
	[NFT_MSG_GETRULE] = {
		.call_rcu	= nf_tables_getrule,
		.attr_count	= NFTA_RULE_MAX,
		.policy		= nft_rule_policy,
	},
	[NFT_MSG_DELRULE] = {
		.call_batch	= nf_tables_delrule,
		.attr_count	= NFTA_RULE_MAX,
		.policy		= nft_rule_policy,
	},
	[NFT_MSG_NEWSET] = {
		.call_batch	= nf_tables_newset,
		.attr_count	= NFTA_SET_MAX,
		.policy		= nft_set_policy,
	},
	[NFT_MSG_GETSET] = {
		.call_rcu	= nf_tables_getset,
		.attr_count	= NFTA_SET_MAX,
		.policy		= nft_set_policy,
	},
	[NFT_MSG_DELSET] = {
		.call_batch	= nf_tables_delset,
		.attr_count	= NFTA_SET_MAX,
		.policy		= nft_set_policy,
	},
	[NFT_MSG_NEWSETELEM] = {
		.call_batch	= nf_tables_newsetelem,
		.attr_count	= NFTA_SET_ELEM_LIST_MAX,
		.policy		= nft_set_elem_list_policy,
	},
	[NFT_MSG_GETSETELEM] = {
		.call_rcu	= nf_tables_getsetelem,
		.attr_count	= NFTA_SET_ELEM_LIST_MAX,
		.policy		= nft_set_elem_list_policy,
	},
	[NFT_MSG_DELSETELEM] = {
		.call_batch	= nf_tables_delsetelem,
		.attr_count	= NFTA_SET_ELEM_LIST_MAX,
		.policy		= nft_set_elem_list_policy,
	},
	[NFT_MSG_GETGEN] = {
		.call_rcu	= nf_tables_getgen,
	},
	[NFT_MSG_NEWOBJ] = {
		.call_batch	= nf_tables_newobj,
		.attr_count	= NFTA_OBJ_MAX,
		.policy		= nft_obj_policy,
	},
	[NFT_MSG_GETOBJ] = {
		.call_rcu	= nf_tables_getobj,
		.attr_count	= NFTA_OBJ_MAX,
		.policy		= nft_obj_policy,
	},
	[NFT_MSG_DELOBJ] = {
		.call_batch	= nf_tables_delobj,
		.attr_count	= NFTA_OBJ_MAX,
		.policy		= nft_obj_policy,
	},
	[NFT_MSG_GETOBJ_RESET] = {
		.call_rcu	= nf_tables_getobj,
		.attr_count	= NFTA_OBJ_MAX,
		.policy		= nft_obj_policy,
	},
	[NFT_MSG_NEWFLOWTABLE] = {
		.call_batch	= nf_tables_newflowtable,
		.attr_count	= NFTA_FLOWTABLE_MAX,
		.policy		= nft_flowtable_policy,
	},
	[NFT_MSG_GETFLOWTABLE] = {
		.call_rcu	= nf_tables_getflowtable,
		.attr_count	= NFTA_FLOWTABLE_MAX,
		.policy		= nft_flowtable_policy,
	},
	[NFT_MSG_DELFLOWTABLE] = {
		.call_batch	= nf_tables_delflowtable,
		.attr_count	= NFTA_FLOWTABLE_MAX,
		.policy		= nft_flowtable_policy,
	},
};

static int nf_tables_validate(struct net *net)
{
	struct nftables_pernet *nft_net = net_generic(net, nf_tables_net_id);
	struct nft_table *table;

	switch (nft_net->validate_state) {
	case NFT_VALIDATE_SKIP:
		break;
	case NFT_VALIDATE_NEED:
		nft_validate_state_update(net, NFT_VALIDATE_DO);
		/* fall through */
	case NFT_VALIDATE_DO:
		list_for_each_entry(table, &nft_net->tables, list) {
			if (nft_table_validate(net, table) < 0)
				return -EAGAIN;
		}

		nft_validate_state_update(net, NFT_VALIDATE_SKIP);
		break;
	}

	return 0;
}

static void nft_chain_commit_update(struct nft_trans *trans)
{
	struct nft_base_chain *basechain;

	if (nft_trans_chain_name(trans)) {
		rhltable_remove(&trans->ctx.table->chains_ht,
				&trans->ctx.chain->rhlhead,
				nft_chain_ht_params);
		swap(trans->ctx.chain->name, nft_trans_chain_name(trans));
		rhltable_insert_key(&trans->ctx.table->chains_ht,
				    trans->ctx.chain->name,
				    &trans->ctx.chain->rhlhead,
				    nft_chain_ht_params);
	}

	if (!nft_is_base_chain(trans->ctx.chain))
		return;

	basechain = nft_base_chain(trans->ctx.chain);
	nft_chain_stats_replace(trans->ctx.net, basechain,
				nft_trans_chain_stats(trans));

	switch (nft_trans_chain_policy(trans)) {
	case NF_DROP:
	case NF_ACCEPT:
		basechain->policy = nft_trans_chain_policy(trans);
		break;
	}
}

static void nft_commit_release(struct nft_trans *trans)
{
	switch (trans->msg_type) {
	case NFT_MSG_DELTABLE:
		nf_tables_table_destroy(&trans->ctx);
		break;
	case NFT_MSG_NEWCHAIN:
		kfree(nft_trans_chain_name(trans));
		break;
	case NFT_MSG_DELCHAIN:
		nf_tables_chain_destroy(&trans->ctx);
		break;
	case NFT_MSG_DELRULE:
		nf_tables_rule_destroy(&trans->ctx, nft_trans_rule(trans));
		break;
	case NFT_MSG_DELSET:
		nft_set_destroy(&trans->ctx, nft_trans_set(trans));
		break;
	case NFT_MSG_DELSETELEM:
		nf_tables_set_elem_destroy(&trans->ctx,
					   nft_trans_elem_set(trans),
					   nft_trans_elem(trans).priv);
		break;
	case NFT_MSG_DELOBJ:
		nft_obj_destroy(&trans->ctx, nft_trans_obj(trans));
		break;
	case NFT_MSG_DELFLOWTABLE:
		nf_tables_flowtable_destroy(nft_trans_flowtable(trans));
		break;
	}
	kfree(trans);
}

static void nf_tables_commit_release(struct net *net)
{
	struct nftables_pernet *nft_net = net_generic(net, nf_tables_net_id);
	struct nft_trans *trans, *next;

	if (list_empty(&nft_net->commit_list))
		return;

	synchronize_rcu();

	list_for_each_entry_safe(trans, next, &nft_net->commit_list, list) {
		nft_trans_list_del(trans);
		nft_commit_release(trans);
	}
}

static int nf_tables_commit_chain_prepare(struct net *net, struct nft_chain *chain)
{
	struct nft_rule *rule;
	unsigned int alloc = 0;
	int i;

	/* already handled or inactive chain? */
	if (chain->rules_next || !nft_is_active_next(net, chain))
		return 0;

	rule = list_entry(&chain->rules, struct nft_rule, list);
	i = 0;

	list_for_each_entry_continue(rule, &chain->rules, list) {
		if (nft_is_active_next(net, rule))
			alloc++;
	}

	chain->rules_next = nf_tables_chain_alloc_rules(chain, alloc);
	if (!chain->rules_next)
		return -ENOMEM;

	list_for_each_entry_continue(rule, &chain->rules, list) {
		if (nft_is_active_next(net, rule))
			chain->rules_next[i++] = rule;
	}

	chain->rules_next[i] = NULL;
	return 0;
}

static void nf_tables_commit_chain_prepare_cancel(struct net *net)
{
	struct nftables_pernet *nft_net = net_generic(net, nf_tables_net_id);
	struct nft_trans *trans, *next;

	list_for_each_entry_safe(trans, next, &nft_net->commit_list, list) {
		struct nft_chain *chain = trans->ctx.chain;

		if (trans->msg_type == NFT_MSG_NEWRULE ||
		    trans->msg_type == NFT_MSG_DELRULE) {
			kvfree(chain->rules_next);
			chain->rules_next = NULL;
		}
	}
}

static void __nf_tables_commit_chain_free_rules_old(struct rcu_head *h)
{
	struct nft_rules_old *o = container_of(h, struct nft_rules_old, h);

	kvfree(o->start);
}

static void nf_tables_commit_chain_free_rules_old(struct nft_rule **rules)
{
	struct nft_rule **r = rules;
	struct nft_rules_old *old;

	while (*r)
		r++;

	r++;	/* rcu_head is after end marker */
	old = (void *) r;
	old->start = rules;

	call_rcu(&old->h, __nf_tables_commit_chain_free_rules_old);
}

static void nf_tables_commit_chain(struct net *net, struct nft_chain *chain)
{
	struct nft_rule **g0, **g1;
	bool next_genbit;

	next_genbit = nft_gencursor_next(net);

	g0 = rcu_dereference_protected(chain->rules_gen_0,
				       lockdep_commit_lock_is_held(net));
	g1 = rcu_dereference_protected(chain->rules_gen_1,
				       lockdep_commit_lock_is_held(net));

	/* No changes to this chain? */
	if (chain->rules_next == NULL) {
		/* chain had no change in last or next generation */
		if (g0 == g1)
			return;
		/*
		 * chain had no change in this generation; make sure next
		 * one uses same rules as current generation.
		 */
		if (next_genbit) {
			rcu_assign_pointer(chain->rules_gen_1, g0);
			nf_tables_commit_chain_free_rules_old(g1);
		} else {
			rcu_assign_pointer(chain->rules_gen_0, g1);
			nf_tables_commit_chain_free_rules_old(g0);
		}

		return;
	}

	if (next_genbit)
		rcu_assign_pointer(chain->rules_gen_1, chain->rules_next);
	else
		rcu_assign_pointer(chain->rules_gen_0, chain->rules_next);

	chain->rules_next = NULL;

	if (g0 == g1)
		return;

	if (next_genbit)
		nf_tables_commit_chain_free_rules_old(g1);
	else
		nf_tables_commit_chain_free_rules_old(g0);
}

static void nft_chain_del(struct nft_chain *chain)
{
	struct nft_table *table = chain->table;

	WARN_ON_ONCE(rhltable_remove(&table->chains_ht, &chain->rhlhead,
				     nft_chain_ht_params));
	list_del_rcu(&chain->list);
}

static void nft_trans_gc_setelem_remove(struct nft_ctx *ctx,
					struct nft_trans_gc *trans)
{
	void **priv = trans->priv;
	unsigned int i;

	for (i = 0; i < trans->count; i++) {
		struct nft_set_elem elem = {
			.priv = priv[i],
		};

		nft_setelem_data_deactivate(ctx->net, trans->set, &elem);
		trans->set->ops->remove(trans->net, trans->set, &elem);
	}
}

void nft_trans_gc_destroy(struct nft_trans_gc *trans)
{
	nft_set_put(trans->set);
	put_net(trans->net);
	kfree(trans);
}
EXPORT_SYMBOL_GPL(nft_trans_gc_destroy);

static void nft_trans_gc_trans_free(struct rcu_head *rcu)
{
	struct nft_set_elem elem = {};
	struct nft_trans_gc *trans;
	struct nft_ctx ctx = {};
	unsigned int i;

	trans = container_of(rcu, struct nft_trans_gc, rcu);
	ctx.net = read_pnet(&trans->set->net);

	for (i = 0; i < trans->count; i++) {
		elem.priv = trans->priv[i];
		atomic_dec(&trans->set->nelems);

		nf_tables_set_elem_destroy(&ctx, trans->set, elem.priv);
       }

	nft_trans_gc_destroy(trans);
}

static bool nft_trans_gc_work_done(struct nft_trans_gc *trans)
{
	struct nftables_pernet *nft_net;
	struct nft_ctx ctx = {};

	nft_net = net_generic(trans->net, nf_tables_net_id);

	mutex_lock(&nft_net->commit_mutex);

	/* Check for race with transaction, otherwise this batch refers to
	 * stale objects that might not be there anymore. Skip transaction if
	 * set has been destroyed from control plane transaction in case gc
	 * worker loses race.
	 */
	if (READ_ONCE(nft_net->gc_seq) != trans->seq || trans->set->dead) {
		mutex_unlock(&nft_net->commit_mutex);
		return false;
	}

	ctx.net = trans->net;
	ctx.table = trans->set->table;

	nft_trans_gc_setelem_remove(&ctx, trans);
	mutex_unlock(&nft_net->commit_mutex);

	return true;
}

static void nft_trans_gc_work(struct work_struct *work)
{
	struct nft_trans_gc *trans, *next;
	LIST_HEAD(trans_gc_list);

	spin_lock(&nf_tables_gc_list_lock);
	list_splice_init(&nf_tables_gc_list, &trans_gc_list);
	spin_unlock(&nf_tables_gc_list_lock);

	list_for_each_entry_safe(trans, next, &trans_gc_list, list) {
		list_del(&trans->list);
		if (!nft_trans_gc_work_done(trans)) {
			nft_trans_gc_destroy(trans);
			continue;
		}
		call_rcu(&trans->rcu, nft_trans_gc_trans_free);
	}
}

struct nft_trans_gc *nft_trans_gc_alloc(struct nft_set *set,
					unsigned int gc_seq, gfp_t gfp)
{
	struct net *net = read_pnet(&set->net);
	struct nft_trans_gc *trans;

	trans = kzalloc(sizeof(*trans), gfp);
	if (!trans)
		return NULL;

	trans->net = maybe_get_net(net);
	if (!trans->net) {
		kfree(trans);
		return NULL;
	}

	refcount_inc(&set->refs);
	trans->set = set;
	trans->seq = gc_seq;

	return trans;
}
EXPORT_SYMBOL_GPL(nft_trans_gc_alloc);

void nft_trans_gc_elem_add(struct nft_trans_gc *trans, void *priv)
{
	trans->priv[trans->count++] = priv;
}
EXPORT_SYMBOL_GPL(nft_trans_gc_elem_add);

static void nft_trans_gc_queue_work(struct nft_trans_gc *trans)
{
	spin_lock(&nf_tables_gc_list_lock);
	list_add_tail(&trans->list, &nf_tables_gc_list);
	spin_unlock(&nf_tables_gc_list_lock);

	schedule_work(&trans_gc_work);
}

static int nft_trans_gc_space(struct nft_trans_gc *trans)
{
	return NFT_TRANS_GC_BATCHCOUNT - trans->count;
}

struct nft_trans_gc *nft_trans_gc_queue_async(struct nft_trans_gc *gc,
					      unsigned int gc_seq, gfp_t gfp)
{
	struct nft_set *set;

	if (nft_trans_gc_space(gc))
		return gc;

	set = gc->set;
	nft_trans_gc_queue_work(gc);

	return nft_trans_gc_alloc(set, gc_seq, gfp);
}
EXPORT_SYMBOL_GPL(nft_trans_gc_queue_async);

void nft_trans_gc_queue_async_done(struct nft_trans_gc *trans)
{
	if (trans->count == 0) {
		nft_trans_gc_destroy(trans);
		return;
	}

	nft_trans_gc_queue_work(trans);
}
EXPORT_SYMBOL_GPL(nft_trans_gc_queue_async_done);

struct nft_trans_gc *nft_trans_gc_queue_sync(struct nft_trans_gc *gc, gfp_t gfp)
{
	struct nft_set *set;

	if (WARN_ON_ONCE(!lockdep_commit_lock_is_held(gc->net)))
		return NULL;

	if (nft_trans_gc_space(gc))
		return gc;

	set = gc->set;
	call_rcu(&gc->rcu, nft_trans_gc_trans_free);

	return nft_trans_gc_alloc(set, 0, gfp);
}
EXPORT_SYMBOL_GPL(nft_trans_gc_queue_sync);

void nft_trans_gc_queue_sync_done(struct nft_trans_gc *trans)
{
	WARN_ON_ONCE(!lockdep_commit_lock_is_held(trans->net));

	if (trans->count == 0) {
		nft_trans_gc_destroy(trans);
		return;
	}

	call_rcu(&trans->rcu, nft_trans_gc_trans_free);
}
EXPORT_SYMBOL_GPL(nft_trans_gc_queue_sync_done);

static unsigned int nft_gc_seq_begin(struct nftables_pernet *nft_net)
{
	unsigned int gc_seq;

	/* Bump gc counter, it becomes odd, this is the busy mark. */
	gc_seq = READ_ONCE(nft_net->gc_seq);
	WRITE_ONCE(nft_net->gc_seq, ++gc_seq);

	return gc_seq;
}

static void nft_gc_seq_end(struct nftables_pernet *nft_net, unsigned int gc_seq)
{
	WRITE_ONCE(nft_net->gc_seq, ++gc_seq);
}

static int nf_tables_commit(struct net *net, struct sk_buff *skb)
{
	struct nftables_pernet *nft_net = net_generic(net, nf_tables_net_id);
	struct nft_trans *trans, *next;
	struct nft_trans_elem *te;
	struct nft_chain *chain;
	struct nft_table *table;
	unsigned int gc_seq;

	list_for_each_entry(trans, &nft_net->binding_list, binding_list) {
		switch (trans->msg_type) {
		case NFT_MSG_NEWSET:
			if (nft_set_is_anonymous(nft_trans_set(trans)) &&
			    !nft_trans_set_bound(trans)) {
				pr_warn_once("nftables ruleset with unbound set\n");
				return -EINVAL;
			}
			break;
		}
	}

	/* 0. Validate ruleset, otherwise roll back for error reporting. */
	if (nf_tables_validate(net) < 0)
		return -EAGAIN;

	/* 1.  Allocate space for next generation rules_gen_X[] */
	list_for_each_entry_safe(trans, next, &nft_net->commit_list, list) {
		int ret;

		if (trans->msg_type == NFT_MSG_NEWRULE ||
		    trans->msg_type == NFT_MSG_DELRULE) {
			chain = trans->ctx.chain;

			ret = nf_tables_commit_chain_prepare(net, chain);
			if (ret < 0) {
				nf_tables_commit_chain_prepare_cancel(net);
				return ret;
			}
		}
	}

	/* step 2.  Make rules_gen_X visible to packet path */
	list_for_each_entry(table, &nft_net->tables, list) {
		list_for_each_entry(chain, &table->chains, list)
			nf_tables_commit_chain(net, chain);
	}

	/*
	 * Bump generation counter, invalidate any dump in progress.
	 * Cannot fail after this point.
	 */
	while (++nft_net->base_seq == 0)
		;

	gc_seq = nft_gc_seq_begin(nft_net);

	/* step 3. Start new generation, rules_gen_X now in use. */
	net->nft.gencursor = nft_gencursor_next(net);

	list_for_each_entry_safe(trans, next, &nft_net->commit_list, list) {
		switch (trans->msg_type) {
		case NFT_MSG_NEWTABLE:
			if (nft_trans_table_update(trans)) {
				if (!(trans->ctx.table->flags & __NFT_TABLE_F_UPDATE)) {
					nft_trans_destroy(trans);
					break;
				}
				if (trans->ctx.table->flags & NFT_TABLE_F_DORMANT)
					nf_tables_table_disable(net, trans->ctx.table);

				trans->ctx.table->flags &= ~__NFT_TABLE_F_UPDATE;
			} else {
				nft_clear(net, trans->ctx.table);
			}
			nf_tables_table_notify(&trans->ctx, NFT_MSG_NEWTABLE);
			nft_trans_destroy(trans);
			break;
		case NFT_MSG_DELTABLE:
			list_del_rcu(&trans->ctx.table->list);
			nf_tables_table_notify(&trans->ctx, NFT_MSG_DELTABLE);
			break;
		case NFT_MSG_NEWCHAIN:
			if (nft_trans_chain_update(trans)) {
				nft_chain_commit_update(trans);
				nf_tables_chain_notify(&trans->ctx, NFT_MSG_NEWCHAIN);
				/* trans destroyed after rcu grace period */
			} else {
				nft_clear(net, trans->ctx.chain);
				nf_tables_chain_notify(&trans->ctx, NFT_MSG_NEWCHAIN);
				nft_trans_destroy(trans);
			}
			break;
		case NFT_MSG_DELCHAIN:
			nft_chain_del(trans->ctx.chain);
			nf_tables_chain_notify(&trans->ctx, NFT_MSG_DELCHAIN);
			nf_tables_unregister_hook(trans->ctx.net,
						  trans->ctx.table,
						  trans->ctx.chain);
			break;
		case NFT_MSG_NEWRULE:
			nft_clear(trans->ctx.net, nft_trans_rule(trans));
			nf_tables_rule_notify(&trans->ctx,
					      nft_trans_rule(trans),
					      NFT_MSG_NEWRULE);
			nft_trans_destroy(trans);
			break;
		case NFT_MSG_DELRULE:
			list_del_rcu(&nft_trans_rule(trans)->list);
			nf_tables_rule_notify(&trans->ctx,
					      nft_trans_rule(trans),
					      NFT_MSG_DELRULE);
			nft_rule_expr_deactivate(&trans->ctx,
						 nft_trans_rule(trans),
						 NFT_TRANS_COMMIT);
			break;
		case NFT_MSG_NEWSET:
			nft_clear(net, nft_trans_set(trans));
			/* This avoids hitting -EBUSY when deleting the table
			 * from the transaction.
			 */
			if (nft_set_is_anonymous(nft_trans_set(trans)) &&
			    !list_empty(&nft_trans_set(trans)->bindings))
				nft_use_dec(&trans->ctx.table->use);

			nf_tables_set_notify(&trans->ctx, nft_trans_set(trans),
					     NFT_MSG_NEWSET, GFP_KERNEL);
			nft_trans_destroy(trans);
			break;
		case NFT_MSG_DELSET:
			nft_trans_set(trans)->dead = 1;
			list_del_rcu(&nft_trans_set(trans)->list);
			nf_tables_set_notify(&trans->ctx, nft_trans_set(trans),
					     NFT_MSG_DELSET, GFP_KERNEL);
			break;
		case NFT_MSG_NEWSETELEM:
			te = (struct nft_trans_elem *)trans->data;

			te->set->ops->activate(net, te->set, &te->elem);
			nf_tables_setelem_notify(&trans->ctx, te->set,
						 &te->elem,
						 NFT_MSG_NEWSETELEM, 0);
			nft_trans_destroy(trans);
			break;
		case NFT_MSG_DELSETELEM:
			te = (struct nft_trans_elem *)trans->data;

			nf_tables_setelem_notify(&trans->ctx, te->set,
						 &te->elem,
						 NFT_MSG_DELSETELEM, 0);
			te->set->ops->remove(net, te->set, &te->elem);
			atomic_dec(&te->set->nelems);
			te->set->ndeact--;
			break;
		case NFT_MSG_NEWOBJ:
			nft_clear(net, nft_trans_obj(trans));
			nf_tables_obj_notify(&trans->ctx, nft_trans_obj(trans),
					     NFT_MSG_NEWOBJ);
			nft_trans_destroy(trans);
			break;
		case NFT_MSG_DELOBJ:
			list_del_rcu(&nft_trans_obj(trans)->list);
			nf_tables_obj_notify(&trans->ctx, nft_trans_obj(trans),
					     NFT_MSG_DELOBJ);
			break;
		case NFT_MSG_NEWFLOWTABLE:
			nft_clear(net, nft_trans_flowtable(trans));
			nf_tables_flowtable_notify(&trans->ctx,
						   nft_trans_flowtable(trans),
						   NFT_MSG_NEWFLOWTABLE);
			nft_trans_destroy(trans);
			break;
		case NFT_MSG_DELFLOWTABLE:
			list_del_rcu(&nft_trans_flowtable(trans)->list);
			nf_tables_flowtable_notify(&trans->ctx,
						   nft_trans_flowtable(trans),
						   NFT_MSG_DELFLOWTABLE);
			nft_unregister_flowtable_net_hooks(net,
					nft_trans_flowtable(trans));
			break;
		}
	}

	nf_tables_commit_release(net);
	nf_tables_gen_notify(net, skb, NFT_MSG_NEWGEN);

	nft_gc_seq_end(nft_net, gc_seq);
	mutex_unlock(&nft_net->commit_mutex);

	return 0;
}

static void nf_tables_abort_release(struct nft_trans *trans)
{
	switch (trans->msg_type) {
	case NFT_MSG_NEWTABLE:
		nf_tables_table_destroy(&trans->ctx);
		break;
	case NFT_MSG_NEWCHAIN:
		nf_tables_chain_destroy(&trans->ctx);
		break;
	case NFT_MSG_NEWRULE:
		nf_tables_rule_destroy(&trans->ctx, nft_trans_rule(trans));
		break;
	case NFT_MSG_NEWSET:
		nft_set_destroy(&trans->ctx, nft_trans_set(trans));
		break;
	case NFT_MSG_NEWSETELEM:
		nft_set_elem_destroy(nft_trans_elem_set(trans),
				     nft_trans_elem(trans).priv, true);
		break;
	case NFT_MSG_NEWOBJ:
		nft_obj_destroy(&trans->ctx, nft_trans_obj(trans));
		break;
	case NFT_MSG_NEWFLOWTABLE:
		nf_tables_flowtable_destroy(nft_trans_flowtable(trans));
		break;
	}
	kfree(trans);
}

static int __nf_tables_abort(struct net *net)
{
	struct nftables_pernet *nft_net = net_generic(net, nf_tables_net_id);
	struct nft_trans *trans, *next;
	struct nft_trans_elem *te;

	list_for_each_entry_safe_reverse(trans, next, &nft_net->commit_list,
					 list) {
		switch (trans->msg_type) {
		case NFT_MSG_NEWTABLE:
			if (nft_trans_table_update(trans)) {
				if (!(trans->ctx.table->flags & __NFT_TABLE_F_UPDATE)) {
					nft_trans_destroy(trans);
					break;
				}
				if (trans->ctx.table->flags & __NFT_TABLE_F_WAS_DORMANT) {
					nf_tables_table_disable(net, trans->ctx.table);
					trans->ctx.table->flags |= NFT_TABLE_F_DORMANT;
				} else if (trans->ctx.table->flags & __NFT_TABLE_F_WAS_AWAKEN) {
					trans->ctx.table->flags &= ~NFT_TABLE_F_DORMANT;
				}
				trans->ctx.table->flags &= ~__NFT_TABLE_F_UPDATE;
				nft_trans_destroy(trans);
			} else {
				list_del_rcu(&trans->ctx.table->list);
			}
			break;
		case NFT_MSG_DELTABLE:
			nft_clear(trans->ctx.net, trans->ctx.table);
			nft_trans_destroy(trans);
			break;
		case NFT_MSG_NEWCHAIN:
			if (nft_trans_chain_update(trans)) {
				free_percpu(nft_trans_chain_stats(trans));
				kfree(nft_trans_chain_name(trans));
				nft_trans_destroy(trans);
			} else {
				nft_use_dec_restore(&trans->ctx.table->use);
				nft_chain_del(trans->ctx.chain);
				nf_tables_unregister_hook(trans->ctx.net,
							  trans->ctx.table,
							  trans->ctx.chain);
			}
			break;
		case NFT_MSG_DELCHAIN:
			nft_use_inc_restore(&trans->ctx.table->use);
			nft_clear(trans->ctx.net, trans->ctx.chain);
			nft_trans_destroy(trans);
			break;
		case NFT_MSG_NEWRULE:
			nft_use_dec_restore(&trans->ctx.chain->use);
			list_del_rcu(&nft_trans_rule(trans)->list);
			nft_rule_expr_deactivate(&trans->ctx,
						 nft_trans_rule(trans),
						 NFT_TRANS_ABORT);
			break;
		case NFT_MSG_DELRULE:
			nft_use_inc_restore(&trans->ctx.chain->use);
			nft_clear(trans->ctx.net, nft_trans_rule(trans));
			nft_rule_expr_activate(&trans->ctx, nft_trans_rule(trans));
			nft_trans_destroy(trans);
			break;
		case NFT_MSG_NEWSET:
			nft_use_dec_restore(&trans->ctx.table->use);
			if (nft_trans_set_bound(trans)) {
				nft_trans_destroy(trans);
				break;
			}
			nft_trans_set(trans)->dead = 1;
			list_del_rcu(&nft_trans_set(trans)->list);
			break;
		case NFT_MSG_DELSET:
			nft_use_inc_restore(&trans->ctx.table->use);
			nft_clear(trans->ctx.net, nft_trans_set(trans));
			if (nft_trans_set(trans)->flags & (NFT_SET_MAP | NFT_SET_OBJECT))
				nft_map_activate(&trans->ctx, nft_trans_set(trans));
			nft_trans_destroy(trans);
			break;
		case NFT_MSG_NEWSETELEM:
			if (nft_trans_elem_set_bound(trans)) {
				nft_trans_destroy(trans);
				break;
			}
			te = (struct nft_trans_elem *)trans->data;
			te->set->ops->remove(net, te->set, &te->elem);
			atomic_dec(&te->set->nelems);
			break;
		case NFT_MSG_DELSETELEM:
			te = (struct nft_trans_elem *)trans->data;

			nft_setelem_data_activate(net, te->set, &te->elem);
			te->set->ops->activate(net, te->set, &te->elem);
			te->set->ndeact--;

			nft_trans_destroy(trans);
			break;
		case NFT_MSG_NEWOBJ:
			nft_use_dec_restore(&trans->ctx.table->use);
			list_del_rcu(&nft_trans_obj(trans)->list);
			break;
		case NFT_MSG_DELOBJ:
			nft_use_inc_restore(&trans->ctx.table->use);
			nft_clear(trans->ctx.net, nft_trans_obj(trans));
			nft_trans_destroy(trans);
			break;
		case NFT_MSG_NEWFLOWTABLE:
			nft_use_dec_restore(&trans->ctx.table->use);
			list_del_rcu(&nft_trans_flowtable(trans)->list);
			nft_unregister_flowtable_net_hooks(net,
					nft_trans_flowtable(trans));
			break;
		case NFT_MSG_DELFLOWTABLE:
			nft_use_inc_restore(&trans->ctx.table->use);
			nft_clear(trans->ctx.net, nft_trans_flowtable(trans));
			nft_trans_destroy(trans);
			break;
		}
	}

	synchronize_rcu();

	list_for_each_entry_safe_reverse(trans, next,
					 &nft_net->commit_list, list) {
		nft_trans_list_del(trans);
		nf_tables_abort_release(trans);
	}

	return 0;
}

static int nf_tables_abort(struct net *net, struct sk_buff *skb)
{
	struct nftables_pernet *nft_net = net_generic(net, nf_tables_net_id);
	unsigned int gc_seq;
	int ret;

	gc_seq = nft_gc_seq_begin(nft_net);
	ret = __nf_tables_abort(net);
	nft_gc_seq_end(nft_net, gc_seq);

	mutex_unlock(&nft_net->commit_mutex);

	return ret;
}

static bool nf_tables_valid_genid(struct net *net, u32 genid)
{
	struct nftables_pernet *nft_net = net_generic(net, nf_tables_net_id);
	bool genid_ok;

	mutex_lock(&nft_net->commit_mutex);
	nft_net->tstamp = get_jiffies_64();

	genid_ok = genid == 0 || nft_net->base_seq == genid;
	if (!genid_ok)
		mutex_unlock(&nft_net->commit_mutex);

	/* else, commit mutex has to be released by commit or abort function */
	return genid_ok;
}

static const struct nfnetlink_subsystem nf_tables_subsys = {
	.name		= "nf_tables",
	.subsys_id	= NFNL_SUBSYS_NFTABLES,
	.cb_count	= NFT_MSG_MAX,
	.cb		= nf_tables_cb,
	.commit		= nf_tables_commit,
	.abort		= nf_tables_abort,
	.valid_genid	= nf_tables_valid_genid,
	.owner		= THIS_MODULE,
};

int nft_chain_validate_dependency(const struct nft_chain *chain,
				  enum nft_chain_types type)
{
	const struct nft_base_chain *basechain;

	if (nft_is_base_chain(chain)) {
		basechain = nft_base_chain(chain);
		if (basechain->type->type != type)
			return -EOPNOTSUPP;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(nft_chain_validate_dependency);

int nft_chain_validate_hooks(const struct nft_chain *chain,
			     unsigned int hook_flags)
{
	struct nft_base_chain *basechain;

	if (nft_is_base_chain(chain)) {
		basechain = nft_base_chain(chain);

		if ((1 << basechain->ops.hooknum) & hook_flags)
			return 0;

		return -EOPNOTSUPP;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(nft_chain_validate_hooks);

/**
 *	nft_parse_u32_check - fetch u32 attribute and check for maximum value
 *
 *	@attr: netlink attribute to fetch value from
 *	@max: maximum value to be stored in dest
 *	@dest: pointer to the variable
 *
 *	Parse, check and store a given u32 netlink attribute into variable.
 *	This function returns -ERANGE if the value goes over maximum value.
 *	Otherwise a 0 is returned and the attribute value is stored in the
 *	destination variable.
 */
int nft_parse_u32_check(const struct nlattr *attr, int max, u32 *dest)
{
	u32 val;

	val = ntohl(nla_get_be32(attr));
	if (val > max)
		return -ERANGE;

	*dest = val;
	return 0;
}
EXPORT_SYMBOL_GPL(nft_parse_u32_check);

static int nft_parse_register(const struct nlattr *attr, u32 *preg)
{
	unsigned int reg;

	reg = ntohl(nla_get_be32(attr));
	switch (reg) {
	case NFT_REG_VERDICT...NFT_REG_4:
		*preg = reg * NFT_REG_SIZE / NFT_REG32_SIZE;
		break;
	case NFT_REG32_00...NFT_REG32_15:
		*preg = reg + NFT_REG_SIZE / NFT_REG32_SIZE - NFT_REG32_00;
		break;
	default:
		return -ERANGE;
	}

	return 0;
}

/**
 *	nft_dump_register - dump a register value to a netlink attribute
 *
 *	@skb: socket buffer
 *	@attr: attribute number
 *	@reg: register number
 *
 *	Construct a netlink attribute containing the register number. For
 *	compatibility reasons, register numbers being a multiple of 4 are
 *	translated to the corresponding 128 bit register numbers.
 */
int nft_dump_register(struct sk_buff *skb, unsigned int attr, unsigned int reg)
{
	if (reg % (NFT_REG_SIZE / NFT_REG32_SIZE) == 0)
		reg = reg / (NFT_REG_SIZE / NFT_REG32_SIZE);
	else
		reg = reg - NFT_REG_SIZE / NFT_REG32_SIZE + NFT_REG32_00;

	return nla_put_be32(skb, attr, htonl(reg));
}
EXPORT_SYMBOL_GPL(nft_dump_register);

/**
 *	nft_validate_register_load - validate a load from a register
 *
 *	@reg: the register number
 *	@len: the length of the data
 *
 * 	Validate that the input register is one of the general purpose
 * 	registers and that the length of the load is within the bounds.
 */
static int nft_validate_register_load(enum nft_registers reg, unsigned int len)
{
	if (reg < NFT_REG_1 * NFT_REG_SIZE / NFT_REG32_SIZE)
		return -EINVAL;
	if (len == 0)
		return -EINVAL;
	if (reg * NFT_REG32_SIZE + len > FIELD_SIZEOF(struct nft_regs, data))
		return -ERANGE;

	return 0;
}

int nft_parse_register_load(const struct nlattr *attr, u8 *sreg, u32 len)
{
	u32 reg;
	int err;

	err = nft_parse_register(attr, &reg);
	if (err < 0)
		return err;

	err = nft_validate_register_load(reg, len);
	if (err < 0)
		return err;

	*sreg = reg;
	return 0;
}
EXPORT_SYMBOL_GPL(nft_parse_register_load);

/**
 *	nft_validate_register_store - validate an expressions' register store
 *
 *	@ctx: context of the expression performing the load
 * 	@reg: the destination register number
 * 	@data: the data to load
 * 	@type: the data type
 * 	@len: the length of the data
 *
 * 	Validate that a data load uses the appropriate data type for
 * 	the destination register and the length is within the bounds.
 * 	A value of NULL for the data means that its runtime gathered
 * 	data.
 */
static int nft_validate_register_store(const struct nft_ctx *ctx,
				       enum nft_registers reg,
				       const struct nft_data *data,
				       enum nft_data_types type,
				       unsigned int len)
{
	int err;

	switch (reg) {
	case NFT_REG_VERDICT:
		if (type != NFT_DATA_VERDICT)
			return -EINVAL;

		if (data != NULL &&
		    (data->verdict.code == NFT_GOTO ||
		     data->verdict.code == NFT_JUMP)) {
			err = nft_chain_validate(ctx, data->verdict.chain);
			if (err < 0)
				return err;
		}

		return 0;
	default:
		if (type != NFT_DATA_VALUE)
			return -EINVAL;

		if (reg < NFT_REG_1 * NFT_REG_SIZE / NFT_REG32_SIZE)
			return -EINVAL;
		if (len == 0)
			return -EINVAL;
		if (reg * NFT_REG32_SIZE + len >
		    FIELD_SIZEOF(struct nft_regs, data))
			return -ERANGE;

		return 0;
	}
}

int nft_parse_register_store(const struct nft_ctx *ctx,
			     const struct nlattr *attr, u8 *dreg,
			     const struct nft_data *data,
			     enum nft_data_types type, unsigned int len)
{
	int err;
	u32 reg;

	err = nft_parse_register(attr, &reg);
	if (err < 0)
		return err;

	err = nft_validate_register_store(ctx, reg, data, type, len);
	if (err < 0)
		return err;

	*dreg = reg;
	return 0;
}
EXPORT_SYMBOL_GPL(nft_parse_register_store);

static const struct nla_policy nft_verdict_policy[NFTA_VERDICT_MAX + 1] = {
	[NFTA_VERDICT_CODE]	= { .type = NLA_U32 },
	[NFTA_VERDICT_CHAIN]	= { .type = NLA_STRING,
				    .len = NFT_CHAIN_MAXNAMELEN - 1 },
};

static int nft_verdict_init(const struct nft_ctx *ctx, struct nft_data *data,
			    struct nft_data_desc *desc, const struct nlattr *nla)
{
	u8 genmask = nft_genmask_next(ctx->net);
	struct nlattr *tb[NFTA_VERDICT_MAX + 1];
	struct nft_chain *chain;
	int err;

	err = nla_parse_nested(tb, NFTA_VERDICT_MAX, nla, nft_verdict_policy,
			       NULL);
	if (err < 0)
		return err;

	if (!tb[NFTA_VERDICT_CODE])
		return -EINVAL;

	/* zero padding hole for memcmp */
	memset(data, 0, sizeof(*data));
	data->verdict.code = ntohl(nla_get_be32(tb[NFTA_VERDICT_CODE]));

	switch (data->verdict.code) {
	case NF_ACCEPT:
	case NF_DROP:
	case NF_QUEUE:
		break;
	case NFT_CONTINUE:
	case NFT_BREAK:
	case NFT_RETURN:
		break;
	case NFT_JUMP:
	case NFT_GOTO:
		if (!tb[NFTA_VERDICT_CHAIN])
			return -EINVAL;
		chain = nft_chain_lookup(ctx->net, ctx->table,
					 tb[NFTA_VERDICT_CHAIN], genmask);
		if (IS_ERR(chain))
			return PTR_ERR(chain);
		if (nft_is_base_chain(chain))
			return -EOPNOTSUPP;
		if (!nft_use_inc(&chain->use))
			return -EMFILE;

		data->verdict.chain = chain;
		break;
	default:
		return -EINVAL;
	}

	desc->len = sizeof(data->verdict);
	desc->type = NFT_DATA_VERDICT;
	return 0;
}

static void nft_verdict_uninit(const struct nft_data *data)
{
	struct nft_chain *chain;

	switch (data->verdict.code) {
	case NFT_JUMP:
	case NFT_GOTO:
		chain = data->verdict.chain;
		nft_use_dec(&chain->use);
		break;
	}
}

int nft_verdict_dump(struct sk_buff *skb, int type, const struct nft_verdict *v)
{
	struct nlattr *nest;

	nest = nla_nest_start(skb, type);
	if (!nest)
		goto nla_put_failure;

	if (nla_put_be32(skb, NFTA_VERDICT_CODE, htonl(v->code)))
		goto nla_put_failure;

	switch (v->code) {
	case NFT_JUMP:
	case NFT_GOTO:
		if (nla_put_string(skb, NFTA_VERDICT_CHAIN,
				   v->chain->name))
			goto nla_put_failure;
	}
	nla_nest_end(skb, nest);
	return 0;

nla_put_failure:
	return -1;
}

static int nft_value_init(const struct nft_ctx *ctx,
			  struct nft_data *data, unsigned int size,
			  struct nft_data_desc *desc, const struct nlattr *nla)
{
	unsigned int len;

	len = nla_len(nla);
	if (len == 0)
		return -EINVAL;
	if (len > size)
		return -EOVERFLOW;

	nla_memcpy(data->data, nla, len);
	desc->type = NFT_DATA_VALUE;
	desc->len  = len;
	return 0;
}

static int nft_value_dump(struct sk_buff *skb, const struct nft_data *data,
			  unsigned int len)
{
	return nla_put(skb, NFTA_DATA_VALUE, len, data->data);
}

static const struct nla_policy nft_data_policy[NFTA_DATA_MAX + 1] = {
	[NFTA_DATA_VALUE]	= { .type = NLA_BINARY },
	[NFTA_DATA_VERDICT]	= { .type = NLA_NESTED },
};

/**
 *	nft_data_init - parse nf_tables data netlink attributes
 *
 *	@ctx: context of the expression using the data
 *	@data: destination struct nft_data
 *	@size: maximum data length
 *	@desc: data description
 *	@nla: netlink attribute containing data
 *
 *	Parse the netlink data attributes and initialize a struct nft_data.
 *	The type and length of data are returned in the data description.
 *
 *	The caller can indicate that it only wants to accept data of type
 *	NFT_DATA_VALUE by passing NULL for the ctx argument.
 */
int nft_data_init(const struct nft_ctx *ctx,
		  struct nft_data *data, unsigned int size,
		  struct nft_data_desc *desc, const struct nlattr *nla)
{
	struct nlattr *tb[NFTA_DATA_MAX + 1];
	int err;

	err = nla_parse_nested(tb, NFTA_DATA_MAX, nla, nft_data_policy, NULL);
	if (err < 0)
		return err;

	if (tb[NFTA_DATA_VALUE])
		return nft_value_init(ctx, data, size, desc,
				      tb[NFTA_DATA_VALUE]);
	if (tb[NFTA_DATA_VERDICT] && ctx != NULL)
		return nft_verdict_init(ctx, data, desc, tb[NFTA_DATA_VERDICT]);
	return -EINVAL;
}
EXPORT_SYMBOL_GPL(nft_data_init);

/**
 *	nft_data_release - release a nft_data item
 *
 *	@data: struct nft_data to release
 *	@type: type of data
 *
 *	Release a nft_data item. NFT_DATA_VALUE types can be silently discarded,
 *	all others need to be released by calling this function.
 */
void nft_data_release(const struct nft_data *data, enum nft_data_types type)
{
	if (type < NFT_DATA_VERDICT)
		return;
	switch (type) {
	case NFT_DATA_VERDICT:
		return nft_verdict_uninit(data);
	default:
		WARN_ON(1);
	}
}
EXPORT_SYMBOL_GPL(nft_data_release);

int nft_data_dump(struct sk_buff *skb, int attr, const struct nft_data *data,
		  enum nft_data_types type, unsigned int len)
{
	struct nlattr *nest;
	int err;

	nest = nla_nest_start(skb, attr);
	if (nest == NULL)
		return -1;

	switch (type) {
	case NFT_DATA_VALUE:
		err = nft_value_dump(skb, data, len);
		break;
	case NFT_DATA_VERDICT:
		err = nft_verdict_dump(skb, NFTA_DATA_VERDICT, &data->verdict);
		break;
	default:
		err = -EINVAL;
		WARN_ON(1);
	}

	nla_nest_end(skb, nest);
	return err;
}
EXPORT_SYMBOL_GPL(nft_data_dump);

int __nft_release_basechain(struct nft_ctx *ctx)
{
	struct nft_rule *rule, *nr;

	if (WARN_ON(!nft_is_base_chain(ctx->chain)))
		return 0;

	nf_tables_unregister_hook(ctx->net, ctx->chain->table, ctx->chain);
	list_for_each_entry_safe(rule, nr, &ctx->chain->rules, list) {
		list_del(&rule->list);
		nft_use_dec(&ctx->chain->use);
		nf_tables_rule_release(ctx, rule);
	}
	nft_chain_del(ctx->chain);
	nft_use_dec(&ctx->table->use);
	nf_tables_chain_destroy(ctx);

	return 0;
}
EXPORT_SYMBOL_GPL(__nft_release_basechain);

static void __nft_release_table(struct net *net, struct nft_table *table)
{
	struct nft_flowtable *flowtable, *nf;
	struct nft_chain *chain, *nc;
	struct nft_object *obj, *ne;
	struct nft_rule *rule, *nr;
	struct nft_set *set, *ns;
	struct nft_ctx ctx = {
		.net	= net,
		.family	= NFPROTO_NETDEV,
	};

	ctx.family = table->family;

	list_for_each_entry(chain, &table->chains, list)
		__nf_tables_unregister_hook(net, table, chain, true);
	list_for_each_entry(flowtable, &table->flowtables, list)
		__nft_unregister_flowtable_net_hooks(net, flowtable, true);
	/* No packets are walking on these chains anymore. */
	ctx.table = table;
	list_for_each_entry(chain, &table->chains, list) {
		ctx.chain = chain;
		list_for_each_entry_safe(rule, nr, &chain->rules, list) {
			list_del(&rule->list);
			nft_use_dec(&chain->use);
			nf_tables_rule_release(&ctx, rule);
		}
	}
	list_for_each_entry_safe(flowtable, nf, &table->flowtables, list) {
		list_del(&flowtable->list);
		nft_use_dec(&table->use);
		nf_tables_flowtable_destroy(flowtable);
	}
	list_for_each_entry_safe(set, ns, &table->sets, list) {
		list_del(&set->list);
		nft_use_dec(&table->use);
		if (set->flags & (NFT_SET_MAP | NFT_SET_OBJECT))
			nft_map_deactivate(&ctx, set);

		nft_set_destroy(&ctx, set);
	}
	list_for_each_entry_safe(obj, ne, &table->objects, list) {
		list_del(&obj->list);
		nft_use_dec(&table->use);
		nft_obj_destroy(&ctx, obj);
	}
	list_for_each_entry_safe(chain, nc, &table->chains, list) {
		ctx.chain = chain;
		nft_chain_del(chain);
		nft_use_dec(&table->use);
		nf_tables_chain_destroy(&ctx);
	}
	list_del(&table->list);
	nf_tables_table_destroy(&ctx);
}

static void __nft_release_tables(struct net *net)
{
	struct nftables_pernet *nft_net = net_generic(net, nf_tables_net_id);
	struct nft_table *table, *nt;

	list_for_each_entry_safe(table, nt, &nft_net->tables, list)
		__nft_release_table(net, table);
}

static int __net_init nf_tables_init_net(struct net *net)
{
	struct nftables_pernet *nft_net = net_generic(net, nf_tables_net_id);

	INIT_LIST_HEAD(&nft_net->tables);
	INIT_LIST_HEAD(&nft_net->commit_list);
	INIT_LIST_HEAD(&nft_net->binding_list);
	mutex_init(&nft_net->commit_mutex);
	nft_net->base_seq = 1;
	nft_net->validate_state = NFT_VALIDATE_SKIP;
	nft_net->gc_seq = 0;

	return 0;
}

static void __net_exit nf_tables_exit_net(struct net *net)
{
	struct nftables_pernet *nft_net = net_generic(net, nf_tables_net_id);
	unsigned int gc_seq;

	mutex_lock(&nft_net->commit_mutex);

	gc_seq = nft_gc_seq_begin(nft_net);

	if (!list_empty(&nft_net->commit_list))
		__nf_tables_abort(net);

	__nft_release_tables(net);

	nft_gc_seq_end(nft_net, gc_seq);

	mutex_unlock(&nft_net->commit_mutex);
	WARN_ON_ONCE(!list_empty(&nft_net->tables));
}

static void nf_tables_exit_batch(struct list_head *net_exit_list)
{
	flush_work(&trans_gc_work);
}

static struct pernet_operations nf_tables_net_ops = {
	.init	= nf_tables_init_net,
	.exit	= nf_tables_exit_net,
	.exit_batch = nf_tables_exit_batch,
	.id	= &nf_tables_net_id,
	.size	= sizeof(struct nftables_pernet),
};

static int __init nf_tables_module_init(void)
{
	int err;

	err = register_pernet_subsys(&nf_tables_net_ops);
	if (err < 0)
		return err;

	err = nft_chain_filter_init();
	if (err < 0)
		goto err1;

	err = nf_tables_core_module_init();
	if (err < 0)
		goto err2;

	err = register_netdevice_notifier(&nf_tables_flowtable_notifier);
	if (err < 0)
		goto err3;

	/* must be last */
	err = nfnetlink_subsys_register(&nf_tables_subsys);
	if (err < 0)
		goto err4;

	return err;
err4:
	unregister_netdevice_notifier(&nf_tables_flowtable_notifier);
err3:
	nf_tables_core_module_exit();
err2:
	nft_chain_filter_fini();
err1:
	unregister_pernet_subsys(&nf_tables_net_ops);
	return err;
}

static void __exit nf_tables_module_exit(void)
{
	nfnetlink_subsys_unregister(&nf_tables_subsys);
	unregister_netdevice_notifier(&nf_tables_flowtable_notifier);
	nft_chain_filter_fini();
	unregister_pernet_subsys(&nf_tables_net_ops);
	cancel_work_sync(&trans_gc_work);
	rcu_barrier();
	nf_tables_core_module_exit();
}

module_init(nf_tables_module_init);
module_exit(nf_tables_module_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Patrick McHardy <kaber@trash.net>");
MODULE_ALIAS_NFNL_SUBSYS(NFNL_SUBSYS_NFTABLES);
