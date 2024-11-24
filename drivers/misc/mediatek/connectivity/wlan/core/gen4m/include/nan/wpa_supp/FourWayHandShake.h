/* SPDX-License-Identifier: GPL-2.0 */

/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#ifndef FOURWAYHANDSHAKE_H
#define FOURWAYHANDSHAKE_H

#if 0
#include <stdarg.h>    /*for va_list*/
#include <stddef.h>    /*for NULL*/
#include <string.h>    /*for memcpy*/
#include <sys/time.h>  /*for os_get_reltime*/
#include <sys/types.h> /*for off_t in stdio.h*/
#include <unistd.h>    /*for os_get_reltime*/
#endif

#include "../include/precomp.h"

#include "src/ap/wpa_auth.h"
#include "src/ap/wpa_auth_i.h"
#include "src/common/defs.h"
#include "src/common/eapol_common.h"
#include "src/crypto/sha1.h"
#include "src/rsn_supp/wpa_ie.h"
#include "src/utils/common.h"
#include "src/utils/state_machine.h"
/*#include "src/ap/wpa_auth_ie.h"*/
#include "src/crypto/aes_wrap.h"
#include "src/crypto/crypto.h"

#include "src/eapol_supp/eapol_supp_sm.h"
#include "src/rsn_supp/wpa_i.h"
#include "src/utils/list.h"

/*WPA STA add*/
#include "src/common/ieee802_11_defs.h"
#include "wpa_supplicant/wpas_glue.h"

/*For WPS*/
/*#include "src/wps/wps.h"*/
/*#include "src/wps/wps_i.h"*/
/*#include "src/wps/wps_dev_attr.h"*/
#include "src/crypto/sha256.h"
#include "src/crypto/sha384.h"
/*#include "src/utils/trace.h"*/
/*#include "src/utils/base64.h"*/
/*#include "src/utils/uuid.h"*/
#include "src/crypto/sha256_i.h"
#include "src/crypto/sha384_i.h"
/*#include "src/crypto/tls.h"*/
#include "src/crypto/random.h"

extern unsigned char g_EnableHostPrintWpa;

extern struct net_device *gPrDev;
extern struct ADAPTER *g_prAdapter;

/*typedef long os_time_t;*/

struct os_time {
	long sec;
	long usec;
};

struct os_reltime {
	long sec;
	long usec;
};

#if 0 /*/*/
struct in_addr {
	u32 s_addr;
};
#endif

/**
 * eloop_timeout_handler - eloop timeout event callback type
 * @eloop_ctx: Registered callback context data (eloop_data)
 * @sock_ctx: Registered callback context data (user_data)
 */
typedef void (*eloop_timeout_handler)(void *eloop_data, void *user_ctx);

struct _WPAS_TIMER_T {
	struct TIMER rTimer;
	eloop_timeout_handler rHandler;
	void *pvEloopData;
	void *pvUserData;
};

enum { MSG_EXCESSIVE,
	MSG_MSGDUMP,
	MSG_DEBUG,
	MSG_INFO,
	MSG_WARNING,
	MSG_ERROR };

#define ATTR_NOINLINE __attribute__((noinline))

#define ETH_ALEN 6
#define WPA_NONCE_LEN 32
#define PMK_LEN 32
#define MAX_STA_NUM 10

#define WLAN_REASON_UNSPECIFIED 1
#define WLAN_REASON_PREV_AUTH_NOT_VALID 2
#define WLAN_REASON_IE_IN_4WAY_DIFFERS 17

#define WLAN_EID_SUPP_RATES 1
#define WLAN_EID_RSN 48
#define WLAN_EID_EXT_SUPP_RATES 50
#define WLAN_EID_MOBILITY_DOMAIN 54
#define WLAN_EID_FAST_BSS_TRANSITION 55
#define WLAN_EID_TIMEOUT_INTERVAL 56
#define WLAN_EID_LINK_ID 101
#define WLAN_EID_EXT_CAPAB 127
#define WLAN_EID_VENDOR_SPECIFIC 221

#define SHA256_MAC_LEN 32
#define SHA384_MAC_LEN 48

#define WLAN_TIMEOUT_REASSOC_DEADLINE 1
#define WLAN_TIMEOUT_KEY_LIFETIME 2

#define WPAS_TIMER_MAX_NUM 10
#define WPAS_TIMER_NOT_FOUND 0xFF

/* Reason codes (IEEE 802.11-2007, 7.3.1.7, Table 7-22) */
#define WLAN_REASON_UNSPECIFIED 1
#define WLAN_REASON_PREV_AUTH_NOT_VALID 2
#define WLAN_REASON_DEAUTH_LEAVING 3
#define WLAN_REASON_DISASSOC_DUE_TO_INACTIVITY 4
#define WLAN_REASON_DISASSOC_AP_BUSY 5
#define WLAN_REASON_CLASS2_FRAME_FROM_NONAUTH_STA 6
#define WLAN_REASON_CLASS3_FRAME_FROM_NONASSOC_STA 7
#define WLAN_REASON_DISASSOC_STA_HAS_LEFT 8
#define WLAN_REASON_STA_REQ_ASSOC_WITHOUT_AUTH 9
/* IEEE 802.11h */
#define WLAN_REASON_PWR_CAPABILITY_NOT_VALID 10
#define WLAN_REASON_SUPPORTED_CHANNEL_NOT_VALID 11
/* IEEE 802.11i */
#define WLAN_REASON_INVALID_IE 13
#define WLAN_REASON_MICHAEL_MIC_FAILURE 14
#define WLAN_REASON_4WAY_HANDSHAKE_TIMEOUT 15
#define WLAN_REASON_GROUP_KEY_UPDATE_TIMEOUT 16
#define WLAN_REASON_IE_IN_4WAY_DIFFERS 17
#define WLAN_REASON_GROUP_CIPHER_NOT_VALID 18
#define WLAN_REASON_PAIRWISE_CIPHER_NOT_VALID 19
#define WLAN_REASON_AKMP_NOT_VALID 20
#define WLAN_REASON_UNSUPPORTED_RSN_IE_VERSION 21
#define WLAN_REASON_INVALID_RSN_IE_CAPAB 22
#define WLAN_REASON_IEEE_802_1X_AUTH_FAILED 23
#define WLAN_REASON_CIPHER_SUITE_REJECTED 24
#define WLAN_REASON_TDLS_TEARDOWN_UNREACHABLE 25
#define WLAN_REASON_TDLS_TEARDOWN_UNSPECIFIED 26

#define HOSTAPD_MODULE_IEEE80211 0x00000001
#define HOSTAPD_MODULE_IEEE8021X 0x00000002
#define HOSTAPD_MODULE_RADIUS 0x00000004
#define HOSTAPD_MODULE_WPA 0x00000008
#define HOSTAPD_MODULE_DRIVER 0x00000010
#define HOSTAPD_MODULE_IAPP 0x00000020
#define HOSTAPD_MODULE_MLME 0x00000040

#define WPA_SYS_RAND_MAX 0xffffffff

enum hostapd_logger_level {
	HOSTAPD_LEVEL_DEBUG_VERBOSE = 0,
	HOSTAPD_LEVEL_DEBUG = 1,
	HOSTAPD_LEVEL_INFO = 2,
	HOSTAPD_LEVEL_NOTICE = 3,
	HOSTAPD_LEVEL_WARNING = 4
};

/**
 * enum report_event - WPA/WPS event types
 */
enum _ENUM_REPORT_EVT_T {
	REPORT_EV_M2D,
	REPORT_EV_FAIL,
	REPORT_EV_SUCCESS,
	REPORT_EV_PWD_AUTH_FAIL
};

#define WPA_PRINT_LEVEL MSG_MSGDUMP
#define DISABLE_FOR_DEADBEEF 1 /*Disable dump print causing stack overflow*/
#define WAIT_DRV_CMD_BY_POLLING 0
#define ENABLE_MEM_ALLOC_CNT 0
#define ENABLE_WPA_PRINT 4
#define CONFIG_NO_RC4 1

/*Port from wpa cmd/evt */
#define RMT_RSN_IE_SAVE_NUM 2
#define MAX_RSN_IE_LEN 100 /*26 256*/

struct eapol_authenticator;

#define HOST_PRINT(...)
#define wpa_parse_kde_ies(...) 0
#define wpa_add_kde(...) 0

#if (ENABLE_WPA_PRINT == 1)
void wpa_printf(int level, const char *fmt, ...);
void wpa_printf_time(const char *title);
void wpa_hexdump(int level, const char *title, const void *buf, size_t len);
void wpa_hexdump_key(int level, const char *title, const void *buf, size_t len);
void wpa_hexdump_buf(int level, const char *title, const struct wpabuf *buf);
void wpa_hexdump_buf_key(int level, const char *title,
			 const struct wpabuf *buf);
void wpa_hexdump_ascii(int level, const char *title, const void *buf,
		       size_t len);
void wpa_hexdump_ascii_key(int level, const char *title, const void *buf,
			   size_t len);
void wpa_hexdump_long(int level, const char *title, const void *buf,
		      size_t len);
void wpa_msg(void *ctx, int level, const char *fmt, ...);
#define wpa_dbg(args...) wpa_msg(args)
void hostapd_logger(void *ctx, const u8 *addr, unsigned int module, int level,
		    const char *fmt, ...);
void wpa_auth_vlogger(struct wpa_authenticator *wpa_auth, const u8 *addr,
		      int level, const char *fmt, ...);
void wpa_auth_logger(struct wpa_authenticator *wpa_auth, const u8 *addr,
		     int level, const char *txt);
void eapol_auth_vlogger(struct eapol_authenticator *eapol, const u8 *addr,
			int level, const char *fmt, ...);
void eapol_auth_logger(struct eapol_authenticator *eapol, const u8 *addr,
		       int level, const char *txt);
#elif (ENABLE_WPA_PRINT == 2)
void wpa_printf_main(const char *fmt, ...);

#define wpa_printf(level, ...) wpa_##level##_printf(__VA_ARGS__)

#define wpa_MSG_ERROR_printf(...) wpa_printf_main(__VA_ARGS__)
#define wpa_MSG_WARNING_printf(...) wpa_printf_main(__VA_ARGS__)
#define wpa_MSG_INFO_printf(...) wpa_printf_main(__VA_ARGS__)
#define wpa_MSG_DEBUG_printf(...)     /*wpa_printf_main(__VA_ARGS__)*/
#define wpa_MSG_MSGDUMP_printf(...)   /*wpa_printf_main(__VA_ARGS__)*/
#define wpa_MSG_EXCESSIVE_printf(...) /*wpa_printf_main(__VA_ARGS__)*/
#define wpa_level_printf(...)	 /*wpa_printf_main(__VA_ARGS__)*/

void wpa_hexdump_dbg(int level, const char *title, const void *buf, size_t len);

#define wpa_printf_time(...)

#define wpa_hexdump(...)
#define wpa_hexdump_key(...)
#define wpa_hexdump_buf(l, t, b)
#define wpa_hexdump_buf_key(l, t, b)
#define wpa_hexdump_ascii_key(...)
#define wpa_hexdump_ascii(...)
#define wpa_hexdump_long(...)
#define wpa_dbg(...)
#define wpa_msg(args...)
#define hostapd_logger(...)
#define wpa_auth_vlogger(...)
#define wpa_auth_logger(...)
#define eapol_auth_vlogger(...)
#define eapol_auth_logger(...)

#elif (ENABLE_WPA_PRINT == 3)
#define HOST_PRINT_WPA(_Fmt) hemPrint _Fmt

#define wpa_printf(level, ...)                                                 \
	do {                                                                   \
		if (g_EnableHostPrintWpa)                                      \
			wpa_##level##_printf(__VA_ARGS__);                     \
	} while (0)

#define wpa_MSG_ERROR_printf(...) HOST_PRINT_WPA((__VA_ARGS__))
#define wpa_MSG_WARNING_printf(...) HOST_PRINT_WPA((__VA_ARGS__))
#define wpa_MSG_INFO_printf(...) HOST_PRINT_WPA((__VA_ARGS__))
#define wpa_MSG_DEBUG_printf(...)     /*HOST_PRINT_WPA((__VA_ARGS__))*/
#define wpa_MSG_MSGDUMP_printf(...)   /*HOST_PRINT_WPA((__VA_ARGS__))*/
#define wpa_MSG_EXCESSIVE_printf(...) /*HOST_PRINT_WPA((__VA_ARGS__))*/
#define wpa_level_printf(...)	 /*HOST_PRINT_WPA((__VA_ARGS__))*/

void wpa_hexdump_dbg(int level, const char *title, const void *buf, size_t len);

#define wpa_printf_time(...)

#define wpa_hexdump(...)
#define wpa_hexdump_key(...)
#define wpa_hexdump_buf(l, t, b)
#define wpa_hexdump_buf_key(l, t, b)
#define wpa_hexdump_ascii_key(...)
#define wpa_hexdump_ascii(...)
#define wpa_hexdump_long(...)

/*#define wpa_dbg(...)*/
#define wpa_dbg(ctx, level, ...) HOST_PRINT_WPA((__VA_ARGS__))

/*#define wpa_msg(args..*/.)
#define wpa_msg(ctx, level, ...) HOST_PRINT_WPA((__VA_ARGS__))

/*#define hostapd_logger(...)*/
#define hostapd_logger(ctx, addr, module, level, ...)                          \
	HOST_PRINT_WPA((__VA_ARGS__))

/*#define wpa_auth_vlogger(...)*/
#define wpa_auth_vlogger(wpa_auth, addr, level, ...)                           \
	HOST_PRINT_WPA((__VA_ARGS__))

/*#define wpa_auth_logger(...)*/
#define wpa_auth_logger(wpa_auth, addr, level, ...)                            \
	HOST_PRINT_WPA((__VA_ARGS__))

#define eapol_auth_vlogger(...)
#define eapol_auth_logger(...)

#elif (ENABLE_WPA_PRINT == 4)
#define wpa_printf(level, ...)                                                 \
	do {                                                                   \
		if (g_EnableHostPrintWpa)                                      \
			wpa_##level##_printf(__VA_ARGS__);                     \
	} while (0)

#define NAN_SEC_DETAIL_LOG 1
#define wpa_MSG_ERROR_printf(...) DBGLOG(NAN, ERROR, __VA_ARGS__)
#define wpa_MSG_WARNING_printf(...) DBGLOG(NAN, WARN, __VA_ARGS__)
#define wpa_MSG_INFO_printf(...) DBGLOG(NAN, INFO, __VA_ARGS__)
#if NAN_SEC_DETAIL_LOG
#define wpa_MSG_DEBUG_printf(...) DBGLOG(NAN, WARN, __VA_ARGS__)
#define wpa_MSG_MSGDUMP_printf(...) DBGLOG(NAN, WARN, __VA_ARGS__)
#define wpa_MSG_EXCESSIVE_printf(...) DBGLOG(NAN, WARN, __VA_ARGS__)
#define wpa_level_printf(...) DBGLOG(NAN, WARN, __VA_ARGS__)
#else
#define wpa_MSG_DEBUG_printf(...)
#define wpa_MSG_MSGDUMP_printf(...)
#define wpa_MSG_EXCESSIVE_printf(...)
#define wpa_level_printf(...)
#endif

#define wpa_printf_time(...)
#if NAN_SEC_DETAIL_LOG
#define wpa_hexdump(LEVEL, TITLE, BUF, LEN)                                    \
	nanUtilDump(NULL, TITLE, (uint8_t *)BUF, LEN)
#else
#define wpa_hexdump(...)
#endif
#define wpa_hexdump_key(...)
#define wpa_hexdump_buf(l, t, b)
#define wpa_hexdump_buf_key(l, t, b)
#define wpa_hexdump_ascii_key(...)
#define wpa_hexdump_ascii(...)
#define wpa_hexdump_long(...)
/*#define wpa_dbg(...)*/
#define wpa_dbg(ctx, level, ...) DBGLOG(NAN, INFO, __VA_ARGS__)
/*#define wpa_msg(...)*/
#define wpa_msg(ctx, level, ...) DBGLOG(NAN, INFO, __VA_ARGS__)
#define hostapd_logger(...)
/*#define wpa_auth_vlogger(...)*/
#define wpa_auth_vlogger(wpa_auth, addr, level, ...)                           \
	DBGLOG(NAN, INFO, __VA_ARGS__)
/*#define wpa_auth_logger(...)*/
#define wpa_auth_logger(wpa_auth, addr, level, ...)                            \
	DBGLOG(NAN, INFO, __VA_ARGS__)

#define eapol_auth_vlogger(...)
#define eapol_auth_logger(...)

#else
#define wpa_printf(...)

/*#define wpa_printf(level, ...)  DBGLOG(MEM, WARN, (__VA_ARGS__))*/
/*wpa_printf(MSG_DEBUG, "[%s] u1BssIdx:%d", __func__, prCmdStartAp->u1BssIdx);*/
/*DBGLOG(MEM, WARN, ("[%s] Warning! Try to free a NULL pointer\n", __func__));*/

#define wpa_printf_time(...)
#define wpa_hexdump(...)
#define wpa_hexdump_key(...)
#define wpa_hexdump_buf(l, t, b)
#define wpa_hexdump_buf_key(l, t, b)
#define wpa_hexdump_ascii_key(...)
#define wpa_hexdump_ascii(...)
#define wpa_hexdump_long(...)
#define wpa_dbg(...)
#define wpa_msg(...)
#define hostapd_logger(...)
#define wpa_auth_vlogger(...)
#define wpa_auth_logger(...)
#define eapol_auth_vlogger(...)
#define eapol_auth_logger(...)
#endif

/*/#define htonl(x)    (x)*/
#define abort()
#define printf(x, y)
#define rand() (0)

#define in_range(c, lo, up) ((int)c >= lo && (int)c <= up)
/*/#define isprint(c)           in_range(c, 0x20, 0x7f)*/

/*#define bswap_16(a) ((((u16)(a) << 8) & 0xff00) | (((u16)(a) >> 8) & 0xff))*/

#define os_time_before(a, b)                                                   \
	((a)->sec < (b)->sec || ((a)->sec == (b)->sec && (a)->usec < (b)->usec))

int eap_register_methods(void);

int eloop_register_timeout(unsigned int secs, unsigned int usecs,
			   eloop_timeout_handler handler, void *eloop_data,
			   void *user_data);

int eloop_cancel_timeout(eloop_timeout_handler handler, void *eloop_data,
			 void *user_data);

int os_get_time(struct os_time *t);

void *os_memcpy(void *dest, const void *src, size_t n);

void *os_memset(void *s, int c, size_t n);

int os_memcmp(const void *s1, const void *s2, size_t n);

int os_memcmp_const(const void *a, const void *b, size_t len);

void *os_memmove(void *dest, const void *src, size_t n);

/*void * _os_malloc(size_t size);*/

int os_snprintf(char *str, size_t size, const char *format, ...);

size_t os_strlen(const char *s);

void *_os_malloc(size_t size, const char *func, int line);

void _os_free(void *ptr, const char *func, int line);

void *_os_zalloc(size_t size, const char *func, int line);

void *os_zalloc_TCM(size_t size);

char *os_strdup(const char *s);

unsigned long os_random(void);

int os_get_random(unsigned char *buf, size_t len);

char *os_strchr(const char *s, int c);

char *os_strstr(const char *haystack, const char *needle);

void *os_realloc(void *ptr, size_t new_size, size_t old_size);

int os_strncmp(const char *s1, const char *s2, size_t n);

int os_strcmp(const char *s1, const char *s2);

/*/int atoi(const char *s);*/

int os_snprintf_error(size_t size, int res);

int os_get_reltime(struct os_reltime *t);

int os_reltime_before(struct os_reltime *a, struct os_reltime *b);

void os_reltime_sub(struct os_reltime *a, struct os_reltime *b,
		    struct os_reltime *res);
#if 0
int os_reltime_expired(struct os_reltime *now,
					   struct os_reltime *ts,
					   os_time_t timeout_secs);
#endif
void *os_calloc(size_t nmemb, size_t size);

int os_strcasecmp(const char *s1, const char *s2);

void wpa_SYSrand_Gen_Rand_Seed(uint8_t *aucOwnMacAddr);

/*void * tls_init(const struct tls_config *conf);*/

/*void tls_deinit(void *ssl_ctx);*/

#define os_malloc(_S) _os_malloc(_S, __func__, __LINE__)
#define os_zalloc(_S) _os_zalloc(_S, __func__, __LINE__)
#define os_free(_V) _os_free(_V, __func__, __LINE__)

#define dup_binstr(_S, _L) _dup_binstr(_S, _L)

#define wpabuf_alloc(_S) _wpabuf_alloc(_S)
#define wpabuf_free(_V) _wpabuf_free(_V)
#define wpabuf_alloc_copy(_D, _L) _wpabuf_alloc_copy(_D, _L)
#define wpabuf_dup(_D) _wpabuf_dup(_D)
#define wpabuf_zeropad(_D, _L) _wpabuf_zeropad(_D, _L)
#define eap_msg_alloc(_V, _T, _P, _C, _I) _eap_msg_alloc(_V, _T, _P, _C, _I)

#endif /*FOURWAYHANDSHAKE_H*/
