/* SPDX-License-Identifier: GPL-2.0 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM blk_mq

#define TRACE_INCLUDE_PATH trace/hooks

#if !defined(_TRACE_HOOK_BLK_MQ_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_HOOK_BLK_MQ_H

#include <trace/hooks/vendor_hooks.h>

struct blk_mq_tag_set;
struct blk_mq_hw_ctx;


DECLARE_HOOK(android_vh_blk_mq_all_tag_iter,
	TP_PROTO(bool *skip, struct blk_mq_tags *tags, busy_tag_iter_fn *fn,
		 void *priv),
	TP_ARGS(skip, tags, fn, priv));

DECLARE_HOOK(android_vh_blk_mq_queue_tag_busy_iter,
	TP_PROTO(bool *skip, struct blk_mq_hw_ctx *hctx, busy_iter_fn * fn,
		 void *priv),
	TP_ARGS(skip, hctx, fn, priv));

DECLARE_HOOK(android_vh_blk_mq_free_tags,
	TP_PROTO(bool *skip, struct blk_mq_tags *tags),
	TP_ARGS(skip, tags));

#endif /* _TRACE_HOOK_BLK_MQ_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
