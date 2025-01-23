/* SPDX-License-Identifier: GPL-2.0 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM dmabuf

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH trace/hooks

#if !defined(_TRACE_HOOK_DMABUF_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_HOOK_DMABUF_H

#include <trace/hooks/vendor_hooks.h>

DECLARE_HOOK(android_vh_dmabuf_page_pool_free_bypass,
		TP_PROTO(struct page *page, bool *bypass),
		TP_ARGS(page, bypass));
#endif /* _TRACE_HOOK_DMABUF_H */
/* This part must be outside protection */
#include <trace/define_trace.h>

