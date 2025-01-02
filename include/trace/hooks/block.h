/* SPDX-License-Identifier: GPL-2.0 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM block

#define TRACE_INCLUDE_PATH trace/hooks

#if !defined(_TRACE_HOOK_BLOCK_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_HOOK_BLOCK_H

#include <linux/tracepoint.h>
#include <trace/hooks/vendor_hooks.h>

#if defined(__GENKSYMS__) || !defined(CONFIG_BLOCK)
struct blk_mq_tags;
struct blk_mq_alloc_data;
struct blk_mq_tag_set;
struct blk_mq_hw_ctx;
#else
/* struct blk_mq_tags */
#include <../block/blk-mq-tag.h>
/* struct blk_mq_alloc_data */
#include <../block/blk-mq.h>
/* struct blk_mq_tag_set struct blk_mq_hw_ctx*/
#include <linux/blk-mq.h>
#endif /* __GENKSYMS__ */
struct bio;
struct request_queue;
struct request;
struct blk_plug;
struct blk_flush_queue;

DECLARE_HOOK(android_vh_blk_alloc_rqs,
	TP_PROTO(size_t *rq_size, struct blk_mq_tag_set *set,
		struct blk_mq_tags *tags),
	TP_ARGS(rq_size, set, tags));

DECLARE_HOOK(android_vh_blk_rq_ctx_init,
	TP_PROTO(struct request *rq, struct blk_mq_tags *tags,
		struct blk_mq_alloc_data *data, u64 alloc_time_ns),
	TP_ARGS(rq, tags, data, alloc_time_ns));

DECLARE_HOOK(android_vh_bio_free,
	TP_PROTO(struct bio *bio),
	TP_ARGS(bio));

DECLARE_RESTRICTED_HOOK(android_rvh_internal_blk_mq_alloc_request,
	TP_PROTO(bool *skip, int *tag, struct blk_mq_alloc_data *data),
	TP_ARGS(skip, tag, data), 1);

DECLARE_HOOK(android_vh_internal_blk_mq_free_request,
	TP_PROTO(bool *skip, struct request *rq, struct blk_mq_hw_ctx *hctx),
	TP_ARGS(skip, rq, hctx));

DECLARE_HOOK(android_vh_blk_mq_complete_request,
	TP_PROTO(bool *skip, struct request *rq),
	TP_ARGS(skip, rq));

DECLARE_HOOK(android_vh_blk_mq_add_to_requeue_list,
	TP_PROTO(bool *skip, struct request *rq, bool kick_requeue_list),
	TP_ARGS(skip, rq, kick_requeue_list));

DECLARE_HOOK(android_vh_blk_mq_get_driver_tag,
	TP_PROTO(struct request *rq),
	TP_ARGS(rq));

DECLARE_RESTRICTED_HOOK(android_rvh_blk_mq_delay_run_hw_queue,
	TP_PROTO(bool *skip, struct blk_mq_hw_ctx *hctx, bool async),
	TP_ARGS(skip, hctx, async), 1);

DECLARE_HOOK(android_vh_blk_mq_run_hw_queue,
	TP_PROTO(bool *need_run, struct blk_mq_hw_ctx *hctx),
	TP_ARGS(need_run, hctx));

DECLARE_HOOK(android_vh_blk_mq_insert_request,
	TP_PROTO(bool *skip, struct blk_mq_hw_ctx *hctx, struct request *rq),
	TP_ARGS(skip, hctx, rq));

DECLARE_RESTRICTED_HOOK(android_rvh_blk_mq_alloc_rq_map,
	TP_PROTO(bool *skip, struct blk_mq_tags **tags,
		struct blk_mq_tag_set *set, int node, unsigned int flags),
	TP_ARGS(skip, tags, set, node, flags), 1);

DECLARE_HOOK(android_vh_blk_mq_hctx_notify_dead,
	TP_PROTO(bool *skip, struct blk_mq_hw_ctx *hctx),
	TP_ARGS(skip, hctx));

DECLARE_RESTRICTED_HOOK(android_rvh_blk_mq_init_allocated_queue,
	TP_PROTO(struct request_queue *q),
	TP_ARGS(q), 1);

DECLARE_HOOK(android_vh_blk_mq_exit_queue,
	TP_PROTO(struct request_queue *q),
	TP_ARGS(q));

DECLARE_HOOK(android_vh_blk_mq_alloc_tag_set,
	TP_PROTO(struct blk_mq_tag_set *set),
	TP_ARGS(set));

DECLARE_HOOK(android_vh_blk_mq_update_nr_requests,
	TP_PROTO(bool *skip, struct request_queue *q),
	TP_ARGS(skip, q));

DECLARE_RESTRICTED_HOOK(android_rvh_blk_allocated_queue_init,
	TP_PROTO(bool *skip, struct request_queue *q),
	TP_ARGS(skip, q), 1);

DECLARE_RESTRICTED_HOOK(android_rvh_blk_flush_plug_list,
	TP_PROTO(struct blk_plug *plug, bool from_schedule),
	TP_ARGS(plug, from_schedule), 1);

DECLARE_HOOK(android_vh_blk_alloc_flush_queue,
	TP_PROTO(bool *skip, int cmd_size, int flags, int node,
		 struct blk_flush_queue *fq),
	TP_ARGS(skip, cmd_size, flags, node, fq));

DECLARE_HOOK(android_vh_blk_mq_sched_insert_request,
	TP_PROTO(bool *skip, struct request *rq),
	TP_ARGS(skip, rq));

#endif /* _TRACE_HOOK_BLOCK_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
