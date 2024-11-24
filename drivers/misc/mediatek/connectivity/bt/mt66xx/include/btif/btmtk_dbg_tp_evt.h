/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM connsys_trace_event

#if !defined(_TRACE_BTMTK_DBG_EVENTS_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_BTMTK_DBG_EVENTS_H

#include <linux/tracepoint.h>

#define __GET_ENTRY_16_BYTES_LEN_BUFFER(__entry_data, __entry_data_len, data, data_len)	\
	do {																				\
		unsigned int data_sz = (data_len <= 16) ? (data_len) : (16);					\
		char buf_str[32] = {""};														\
		int i = 0;																		\
		if (data_sz > 0 && data != NULL) {												\
			if (snprintf(__entry_data, __entry_data_len, "%s", "") < 0)					\
				pr_info("snprintf error in btmtk_dbg_tp_evt.h");											\
			for (i = 0; i < data_sz; i++) {												\
				if (snprintf(buf_str, sizeof(buf_str), "%02x", data[i]) > 0)			\
					strncat(__entry_data, buf_str, strlen(buf_str));					\
			}																			\
		} else {																		\
			if (snprintf(__entry_data, __entry_data_len, "%s", "null") < 0)				\
				pr_info("snprintf error in btmtk_dbg_tp_evt.h");											\
		}																				\
	} while (0)

#define __GET_ENTRY_STRING(__entry_data, __entry_data_len, data)					\
	do {																			\
		if (strlen(data) == 0 || data == NULL){										\
			if (snprintf(__entry_data, __entry_data_len, "%s", "null") < 0)			\
				pr_info("snprintf error in btmtk_dbg_tp_evt.h");										\
		} else {																	\
			if (snprintf(__entry_data, __entry_data_len, "%s", data) < 0)			\
				pr_info("snprintf error in btmtk_dbg_tp_evt.h");										\
		}																			\
	} while (0)

TRACE_EVENT(bt_evt,

	TP_PROTO(unsigned int pkt_action,
		unsigned int parameter,
		unsigned int data_len,
		char *data),

	TP_ARGS(pkt_action, parameter, data_len, data),

	TP_STRUCT__entry(
		__field(unsigned int, pkt_action)
		__field(unsigned int, parameter)
		__field(unsigned int, data_len)
		__dynamic_array(char, data, 256)
	),

	TP_fast_assign(
		__entry->pkt_action = pkt_action;
		__entry->parameter = parameter;
		__entry->data_len = data_len;
		__GET_ENTRY_16_BYTES_LEN_BUFFER(__get_str(data), __get_dynamic_array_len(data), data, data_len);
	),

	TP_printk("%d,%d,%d,%s",
		__entry->pkt_action, __entry->parameter, __entry->data_len, __get_str(data))
);

#endif /* _TRACE_BTMTK_DBG_EVENTS_H */

/* This part must be outside protection */
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH ./
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE btmtk_dbg_tp_evt
#include <trace/define_trace.h>


