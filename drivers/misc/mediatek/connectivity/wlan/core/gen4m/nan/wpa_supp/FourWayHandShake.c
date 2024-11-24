/* SPDX-License-Identifier: GPL-2.0 */

/*
 * Copyright (c) 2020 MediaTek Inc.
 */
#include "wpa_supp/FourWayHandShake.h"
/*#include "wifi_var.h"*/

struct _WPAS_TIMER_T g_arWpasTimer[WPAS_TIMER_MAX_NUM];

int wpa_debug_show_keys = 1;

uint32_t g_u4Randomseed;

uint32_t g_u4OsMallocCnt;

unsigned char g_EnableHostPrintWpa = TRUE;

struct ADAPTER *g_prAdapter;

void
wpas_timeoutCb(struct ADAPTER *prAdapter, unsigned long u4TimerIdx) {
	wpa_printf(MSG_INFO, "[%s] Enter, u4TimerIdx:%d\n", __func__,
		   u4TimerIdx);
	g_arWpasTimer[u4TimerIdx].rHandler(
		g_arWpasTimer[u4TimerIdx].pvEloopData,
		g_arWpasTimer[u4TimerIdx].pvUserData);
}

uint32_t
wpas_convertToMs(IN unsigned int secs, IN unsigned int usecs) {
	return (SEC_TO_MSEC(secs) + USEC_TO_MSEC(usecs));
}

uint8_t
wpas_getCorrespondingTimer(IN eloop_timeout_handler handler) {
	uint8_t i;

	/*====== Search if current timer exists for this handler*/
	for (i = 0; i < WPAS_TIMER_MAX_NUM; i++) {
		if (g_arWpasTimer[i].rHandler == handler)
			return i;
	}

	return WPAS_TIMER_NOT_FOUND;
}

uint8_t
wpas_regCorrespondingTimer(IN eloop_timeout_handler handler,
			   IN void *eloop_data, IN void *user_data) {
	uint8_t i;

	/*====== Reserve an available timer for this handler*/
	for (i = 0; i < WPAS_TIMER_MAX_NUM; i++) {
		if (g_arWpasTimer[i].rHandler == NULL) {
			g_arWpasTimer[i].rHandler = handler;
			g_arWpasTimer[i].pvEloopData = eloop_data;
			g_arWpasTimer[i].pvUserData = user_data;

			wpa_printf(
				MSG_INFO,
				"[%s] i:%d, rHandler:0x%p, eloop_data:0x%p, user_data:0x%p\n",
				__func__, i, g_arWpasTimer[i].rHandler,
				g_arWpasTimer[i].pvEloopData,
				g_arWpasTimer[i].pvUserData);

			cnmTimerInitTimer(g_prAdapter, &g_arWpasTimer[i].rTimer,
					  (PFN_MGMT_TIMEOUT_FUNC)wpas_timeoutCb,
					  (unsigned long)i);
			return i;
		}
	}

	return WPAS_TIMER_NOT_FOUND;
}

int
eloop_register_timeout(unsigned int secs, unsigned int usecs,
		       eloop_timeout_handler handler, void *eloop_data,
		       void *user_data) {
	uint32_t u4TimeoutMs = 0;
	uint8_t u1TimerIdx = WPAS_TIMER_NOT_FOUND;
	int8_t i1Status = 0;

	u4TimeoutMs = wpas_convertToMs(secs, usecs);
	u1TimerIdx = wpas_getCorrespondingTimer(handler);

	if (u1TimerIdx != WPAS_TIMER_NOT_FOUND) {
		cnmTimerStartTimer(g_prAdapter,
				   &g_arWpasTimer[u1TimerIdx].rTimer,
				   u4TimeoutMs);
		i1Status = 1;
	} else {
		u1TimerIdx = wpas_regCorrespondingTimer(handler, eloop_data,
							user_data);
		if (u1TimerIdx != WPAS_TIMER_NOT_FOUND) {
			cnmTimerStartTimer(g_prAdapter,
					   &g_arWpasTimer[u1TimerIdx].rTimer,
					   u4TimeoutMs);
			i1Status = 0;
		} else {
			/*warning: no avaialbe timer*/
			i1Status = -1;
		}
	}
	return i1Status;
}

int
eloop_cancel_timeout(eloop_timeout_handler handler, void *eloop_data,
		     void *user_data) {
	uint8_t u1TimerIdx = WPAS_TIMER_NOT_FOUND;

	u1TimerIdx = wpas_getCorrespondingTimer(handler);

	if (u1TimerIdx != WPAS_TIMER_NOT_FOUND) {
		cnmTimerStopTimer(g_prAdapter,
				  &g_arWpasTimer[u1TimerIdx].rTimer);
		g_arWpasTimer[u1TimerIdx].rHandler = NULL;
		g_arWpasTimer[u1TimerIdx].pvEloopData = NULL;
		g_arWpasTimer[u1TimerIdx].pvUserData = NULL;
		return 0;
	} else {
		return -1;
	}
}

void *
os_memcpy(void *dest, const void *src, size_t n) {

	if (src == NULL) {
		/*DBGLOG(MEM, WARN, */
		/* ("[%s] Warning! Try to copy a NULL pointer\n", */
		/* __func__)); */
		return NULL;
	}

	return kalMemCopy(dest, src, n);
}

void *
os_memset(void *s, int c, size_t n) {
	if (s == NULL) {
		/*DBGLOG(MEM, WARN, */
		/* ("[%s] Warning! Try to set a NULL pointer\n", */
		/* __func__)); */
		return NULL;
	}

	return kalMemSet(s, c, n);
}

int
os_memcmp(const void *s1, const void *s2, size_t n) {
	if (s1 == NULL || s2 == NULL) {
		/*DBGLOG(MEM, WARN, */
		/* ("[%s] Warning! Try to cmp a NULL pointer\n", */
		/* __func__)); */
		return 0;
	}
	return kalMemCmp(s1, s2, n);
}

int
os_memcmp_const(const void *a, const void *b, size_t len) {
	return kalMemCmp(a, b, len);
}

void *
os_memmove(void *dest, const void *src, size_t n) {
	return kalMemMove(dest, src, n);
}

int
os_get_time(struct os_time *t) {
	uint32_t u4CurTime = SYSTIME_TO_USEC(kalGetTimeTick());

	t->sec = u4CurTime / USEC_PER_SEC;
	t->usec = u4CurTime % USEC_PER_SEC;
	return 0;
}

/*void * _os_malloc(size_t size)
*{
*    void * n = cnmMemAlloc(RAM_TYPE_BUF, size); //or RAM_TYPE_TCM
*    g_u4OsMallocCnt++;
*    wpa_printf(MSG_INFO, "os_malloc@[%s][LINE:%d][CNT:%d] len:%d,
*	 addr:0x%p", __func__, __LINE__, g_u4OsMallocCnt, size, n);
*    return n;
*}
*/

int
os_snprintf(char *str, size_t size, const char *format, ...) {
	va_list ap;
	int ret = 0;

	/* See http://www.ijs.si/software/snprintf/ for portable
	 * implementation of snprintf.
	 */

	va_start(ap, format);
	ret = vsnprintf(str, size, format, ap);
	/*ret = rpl_vsnprintf(str, size, format, ap);*/
	va_end(ap);
	if (size > 0)
		str[size - 1] = '\0';
	return ret;
}

size_t
os_strlen(const char *s) {
	const char *p = s;

	while (*p)
		p++;
	return p - s;
}

void
_os_free(void *ptr, const char *func, int line) {
#if 0
	DBGLOG(NAN, INFO, "[TEST][free] ptr:%p, %s:%d\n", ptr, func, line);
#endif

	if (ptr == NULL) {
		/*DBGLOG(MEM, WARN, */
		/* ("[%s] Warning! Try to free a NULL pointer\n", */
		/*__func__));*/
	} else {
		cnmMemFree(g_prAdapter, ptr);
		g_u4OsMallocCnt--;
		ptr = NULL;
	}
}

void *
_os_malloc(size_t size, const char *func, int line) {
	void *n = cnmMemAlloc(g_prAdapter, RAM_TYPE_BUF, size);

#if 0
	DBGLOG(NAN, INFO, "[TEST][malloc] ptr:%p, %s:%d\n", n, func, line);
#endif

	return n;
}

void *
_os_zalloc(size_t size, const char *func, int line) {
	void *n = _os_malloc(size, func, line);

	if (n)
		return os_memset(n, 0, size);
	return NULL;
}

void *
os_zalloc_TCM(size_t size) {
	void *n = cnmMemAlloc(g_prAdapter, RAM_TYPE_BUF, size);

	if (n)
		return os_memset(n, 0, size);
	return NULL;
}

char *
os_strdup(const char *s) {
	char *res;
	size_t len;

	if (s == NULL)
		return NULL;
	len = os_strlen(s);
	res = os_malloc(len + 1);
	if (res)
		os_memcpy(res, s, len + 1);
	return res;
}

/*Must after CMD_ID_ACTIVATE_CTRL for set own MAC*/
void
wpa_SYSrand_Gen_Rand_Seed(uint8_t *aucOwnMacAddr) {
	uint32_t u4CurrentTick = kalGetTimeTick();
	uint32_t u4OwnMacAddr =
		((aucOwnMacAddr[5]) | (aucOwnMacAddr[4] << 8) |
		 (aucOwnMacAddr[3] << 16) | (aucOwnMacAddr[2] << 24));

	g_u4Randomseed = u4CurrentTick ^ u4OwnMacAddr;

	/*wpa_printf(MSG_DEBUG, "[%s] New RandSeed:0x%lx", */
	/* __func__, g_u4Randomseed);*/
}

/*Ref: SYSrand_Get_Rand()*/
uint32_t
wpa_SYSrand_Get_Rand(void) /* reentrant */
{
	uint32_t result;
	uint32_t next_seed = g_u4Randomseed;

	next_seed = (next_seed * 1103515245) + 12345; /* permutate seed */
	result = next_seed & 0xfffc0000;	      /* use only top 14 bits */

	next_seed = (next_seed * 1103515245) + 12345;       /* permutate seed */
	result = result + ((next_seed & 0xffe00000) >> 14); /* top 11 bits */

	next_seed = (next_seed * 1103515245) + 12345; /* permutate seed */
	result = result +
		 ((next_seed & 0xfe000000) >> (25)); /* use only top 7 bits */

	g_u4Randomseed = next_seed;

	/*wpa_printf(MSG_DEBUG, "[%s] next_seed:0x%lx, result:0x%lx",*/
	/* __func__, g_u4Randomseed, result);*/

	return (result & WPA_SYS_RAND_MAX);
}

/*Ref: Ant_Wrap_MTK_Get_Rand_Num()*/
int
os_get_random(unsigned char *buf, size_t len) {
	uint8_t nLoop;
	uint32_t randValue;

	while (len > 0) {
		randValue = wpa_SYSrand_Get_Rand();

		for (nLoop = 0; (nLoop < 4) && (len > 0); nLoop++, len--) {
			*buf++ = (uint8_t)(randValue & 0xFF);
			randValue >>= 8;
		}
	}

	return 0;
}

unsigned long
os_random(void) {
	return wpa_SYSrand_Get_Rand();
}

char *
os_strchr(const char *s, int c) {
	while (*s) {
		if (*s == c)
			return (char *)s;
		s++;
	}
	return NULL;
}

char *
os_strstr(const char *haystack, const char *needle) {
	size_t len = os_strlen(needle);

	while (*haystack) {
		if (os_strncmp(haystack, needle, len) == 0)
			return (char *)haystack;
		haystack++;
	}

	return NULL;
}

void *
os_realloc(void *ptr, size_t new_size, size_t old_size) {

	size_t copy_len;
	void *n;

	if (ptr == NULL)
		return os_malloc(new_size);

	n = os_malloc(new_size);
	if (n == NULL)
		return NULL;
	copy_len = old_size;
	if (copy_len > new_size)
		copy_len = new_size;
	os_memcpy(n, ptr, copy_len);
	os_free(ptr);
	return n;
}

int
os_strncmp(const char *s1, const char *s2, size_t n) {
	if (n == 0)
		return 0;

	while (*s1 == *s2) {
		if (*s1 == '\0')
			break;
		s1++;
		s2++;
		n--;
		if (n == 0)
			return 0;
	}

	return *s1 - *s2;
}

int
os_strcmp(const char *s1, const char *s2) {
	while (*s1 == *s2) {
		if (*s1 == '\0')
			break;
		s1++;
		s2++;
	}

	return *s1 - *s2;
}

#if 0 /*/*/
int atoi(const char *s)
{
	int sum = 0, i;

	for (i = 0; s[i] != '\0'; i++)
		sum = sum * 10 + s[i] - '0';
	return sum;
}
#endif

int
os_snprintf_error(size_t size, int res) {
	return res < 0 || (unsigned int)res >= size;
}

int
os_get_reltime(struct os_reltime *t) {
	uint32_t u4CurTime = SYSTIME_TO_USEC(kalGetTimeTick());

	t->sec = u4CurTime / USEC_PER_SEC;
	t->usec = u4CurTime % USEC_PER_SEC;
	return 0;
}

int
os_reltime_before(struct os_reltime *a, struct os_reltime *b) {
	return (a->sec < b->sec) || (a->sec == b->sec && a->usec < b->usec);
}

void
os_reltime_sub(struct os_reltime *a, struct os_reltime *b,
	       struct os_reltime *res) {
	res->sec = a->sec - b->sec;
	res->usec = a->usec - b->usec;
	if (res->usec < 0) {
		res->sec--;
		res->usec += 1000000;
	}
}
#if 0
int os_reltime_expired(struct os_reltime *now,
					   struct os_reltime *ts,
					   os_time_t timeout_secs)
{
	struct os_reltime age;

	os_reltime_sub(now, ts, &age);
	return (age.sec > timeout_secs) ||
		   (age.sec == timeout_secs && age.usec > 0);
}
#endif
void *
os_calloc(size_t nmemb, size_t size) {
	if (size && nmemb > (~(size_t)0) / size)
		return NULL;
	return os_zalloc(nmemb * size);
}

int
os_strcasecmp(const char *s1, const char *s2) {
	/*
	 * Ignoring case is not required for main functionality, so just use
	 * the case sensitive version of the function.
	 */
	return os_strcmp(s1, s2);
}

void
dumpCmdKey(struct CMD_802_11_KEY *prCmdKey) {}

#if (ENABLE_WPA_PRINT == 1)

/**
 * wpa_printf - conditional printf
 * @level: priority level (MSG_*) of the message
 * @fmt: printf format string, followed by optional arguments
 *
 * This function is used to print conditional debugging and error messages. The
 * output may be directed to stdout, stderr, and/or syslog based on
 * configuration.
 *
 * Note: New line '\n' is added to the end of the text when printing to stdout.
 */
void
wpa_printf(int level, const char *fmt, ...) {
	char acOutBuf[DBG_BUF_SZ];
	uint16_t u2OutBufLen = 0;

	if (level >= WPA_PRINT_LEVEL) {
		va_list ap;

		va_start(ap, fmt);

		u2OutBufLen = rpl_vsnprintf(acOutBuf, DBG_BUF_SZ, fmt, ap);

		if (u2OutBufLen < (DBG_BUF_SZ - 1)) {
			/*always new line since the end of msg is usually */
			/* without \n in wpa supp codebase */
			if (u2OutBufLen < (DBG_BUF_SZ - 1) &&
			    acOutBuf[u2OutBufLen - 1] == '\n') {
				acOutBuf[u2OutBufLen - 1] = '\r';
				acOutBuf[u2OutBufLen] = '\n';
				acOutBuf[u2OutBufLen + 1] = 0; /*terminal*/
				u2OutBufLen++;
			} else if (u2OutBufLen < (DBG_BUF_SZ - 2)) {
				acOutBuf[u2OutBufLen] = '\r';
				acOutBuf[u2OutBufLen + 1] = '\n';
				acOutBuf[u2OutBufLen + 2] = 0;
				u2OutBufLen += 2;
			}
		}

		wifi_UART_PutBytes(acOutBuf, u2OutBufLen);

		va_end(ap);
	}
}

void
wpa_printf_time(const char *title) {
#if (DISABLE_FOR_DEADBEEF == 0)
	uint32_t u4CurTime = SYSTIME_TO_USEC(kalGetTimeTick());

	wpa_printf(MSG_DEBUG, "******(%5u.%06u) %s", (u4CurTime / USEC_PER_SEC),
		   (u4CurTime % USEC_PER_SEC), title);
#endif
}

void
_wpa_hexdump(int level, const char *title, const u8 *buf, size_t len, int show,
	     int type) /*type: 0=%d, 1=%c, 2=%ld*/
{
#if (DISABLE_FOR_DEADBEEF == 0)
	uint8_t i;
	UINT_8 u1Round = len / 5;
	uint8_t u1Rest = len % 5;

	uint32_t *u4Buf = (uint32_t *)buf;

	if (level >= WPA_PRINT_LEVEL && show) {
		/*====== Title print*/
		wpa_printf(level, "%s:", title);

		/*====== Dump print*/
		if (buf == NULL || len == 0) {
			wpa_printf(level, "NULL\n", title);
			return;
		} else if (len) {
			for (i = 0; i < u1Round; i++) {
				if (type == 1) {
					wpa_printf(
						level,
						"[%02d]:%c, [%02d]:%c, [%02d]:%c, [%02d]:%c, [%02d]:%c,",
						i * 5 + 0, buf[i * 5 + 0],
						i * 5 + 1, buf[i * 5 + 1],
						i * 5 + 2, buf[i * 5 + 2],
						i * 5 + 3, buf[i * 5 + 3],
						i * 5 + 4, buf[i * 5 + 4]);

				} else if (type == 2) {
					wpa_printf(
						level,
						"[%02d]:%ld, [%02d]:%ld, [%02d]:%ld, [%02d]:%ld, [%02d]:%ld,",
						i * 5 + 0, u4Buf[i * 5 + 0],
						i * 5 + 1, u4Buf[i * 5 + 1],
						i * 5 + 2, u4Buf[i * 5 + 2],
						i * 5 + 3, u4Buf[i * 5 + 3],
						i * 5 + 4, u4Buf[i * 5 + 4]);
				} else {
					wpa_printf(
						level,
						"[%02d]:0x%02x, [%02d]:0x%02x, [%02d]:0x%02x, [%02d]:0x%02x, [%02d]:0x%02x,",
						i * 5 + 0, buf[i * 5 + 0],
						i * 5 + 1, buf[i * 5 + 1],
						i * 5 + 2, buf[i * 5 + 2],
						i * 5 + 3, buf[i * 5 + 3],
						i * 5 + 4, buf[i * 5 + 4]);
				}
			}

			if (u1Rest) {
				for (i = 0; i < u1Rest; i++) {
					if (type == 1)
						wpa_printf(
							level, "[%02d]:%c",
							u1Round * 5 + i,
							buf[u1Round * 5 + i]);
					else if (type == 2)
						wpa_printf(
							level, "[%02d]:%ld",
							u1Round * 5 + i,
							u4Buf[u1Round * 5 + i]);
					else
						wpa_printf(
							level, "[%02d]:0x%02x",
							u1Round * 5 + i,
							buf[u1Round * 5 + i]);
				}
			}
		}
	}
#endif
}

/**
 * wpa_hexdump - conditional hex dump
 * @level: priority level (MSG_*) of the message
 * @title: title of for the message
 * @buf: data buffer to be dumped
 * @len: length of the buf
 *
 * This function is used to print conditional debugging and error messages. The
 * output may be directed to stdout, stderr, and/or syslog based on
 * configuration. The contents of buf is printed out has hex dump.
 */
void
wpa_hexdump(int level, const char *title, const void *buf, size_t len) {
	_wpa_hexdump(level, title, buf, len, 1, 0);
}

void
wpa_hexdump_key(int level, const char *title, const void *buf, size_t len) {
	_wpa_hexdump(level, title, buf, len, wpa_debug_show_keys, 0);
}

void
wpa_hexdump_buf(int level, const char *title, const struct wpabuf *buf) {
	wpa_hexdump(level, title, buf ? wpabuf_head(buf) : NULL,
		    buf ? wpabuf_len(buf) : 0);
}

void
wpa_hexdump_buf_key(int level, const char *title, const struct wpabuf *buf) {
	wpa_hexdump_key(level, title, buf ? wpabuf_head(buf) : NULL,
			buf ? wpabuf_len(buf) : 0);
}

void
wpa_hexdump_ascii(int level, const char *title, const void *buf, size_t len) {
	_wpa_hexdump(level, title, buf, len, 1, 1);
}

void
wpa_hexdump_ascii_key(int level, const char *title, const void *buf,
		      size_t len) {
	_wpa_hexdump(level, title, buf, len, wpa_debug_show_keys, 1);
}
void
wpa_hexdump_long(int level, const char *title, const void *buf, size_t len) {
	_wpa_hexdump(level, title, buf, len, 1, 2);
}

void
wpa_msg(void *ctx, int level, const char *fmt, ...) {
	wpa_printf(level, fmt);
}

void
hostapd_logger(void *ctx, const u8 *addr, unsigned int module, int level,
	       const char *fmt, ...) {
	if (level >= WPA_PRINT_LEVEL) {
		if (addr)
			wpa_printf(MSG_DEBUG, "hostapd_logger: STA " MACSTR,
				   MAC2STR(addr));

		if (module)
			wpa_printf(MSG_DEBUG, "hostapd_logger: module:%d ",
				   module);

		wpa_printf(level, fmt);
	}
}

void
wpa_auth_vlogger(struct wpa_authenticator *wpa_auth, const u8 *addr, int level,
		 const char *fmt, ...) {
	wpa_printf(MSG_DEBUG, fmt);
}

void
wpa_auth_logger(struct wpa_authenticator *wpa_auth, const u8 *addr, int level,
		const char *txt) {
	wpa_printf(MSG_DEBUG, "%s", txt);
}

void
eapol_auth_vlogger(struct eapol_authenticator *eapol, const u8 *addr, int level,
		   const char *fmt, ...) {
	wpa_printf(MSG_DEBUG, fmt);
}

void
eapol_auth_logger(struct eapol_authenticator *eapol, const u8 *addr, int level,
		  const char *txt) {
	wpa_printf(MSG_DEBUG, "%s", txt);
}

#elif (ENABLE_WPA_PRINT == 2)

void
wpa_printf_main(const char *fmt, ...) {
	char acOutBuf[DBG_BUF_SZ];
	uint16_t u2OutBufLen = 0;

	va_list ap;

	va_start(ap, fmt);

	u2OutBufLen = rpl_vsnprintf(acOutBuf, DBG_BUF_SZ, fmt, ap);

	if (u2OutBufLen < (DBG_BUF_SZ - 1)) {
		/*always new line since the end of msg is usually */
		/* without \n in wpa supp codebase */
		if (u2OutBufLen < (DBG_BUF_SZ - 1) &&
		    acOutBuf[u2OutBufLen - 1] == '\n') {
			acOutBuf[u2OutBufLen - 1] = '\r';
			acOutBuf[u2OutBufLen] = '\n';
			acOutBuf[u2OutBufLen + 1] = 0; /*terminal*/
			u2OutBufLen++;
		} else if (u2OutBufLen < (DBG_BUF_SZ - 2)) {
			acOutBuf[u2OutBufLen] = '\r';
			acOutBuf[u2OutBufLen + 1] = '\n';
			acOutBuf[u2OutBufLen + 2] = 0;
			u2OutBufLen += 2;
		}
	}

	wifi_UART_PutBytes(acOutBuf, u2OutBufLen);

	va_end(ap);
}

void
_wpa_hexdump(int level, const char *title, const u8 *buf, size_t len, int show,
	     int type) /*type: 0=%d, 1=%c, 2=%ld*/
{
	/*#if(DISABLE_FOR_DEADBEEF==0)*/
	uint8_t i;
	uint8_t u1Round = len / 5;
	uint8_t u1Rest = len % 5;

	uint32_t *u4Buf = (uint32_t *)buf;

	if (level >= WPA_PRINT_LEVEL && show) {
		/*====== Title print*/
		wpa_printf(level, "%s:", title);

		/*====== Dump print*/
		if (buf == NULL || len == 0) {
			wpa_printf(level, "NULL\n", title);
			return;
		} else if (len) {
			for (i = 0; i < u1Round; i++) {
				if (type == 1) {
					wpa_printf(
						level,
						"[%02d]:%c, [%02d]:%c, [%02d]:%c, [%02d]:%c, [%02d]:%c,",
						i * 5 + 0, buf[i * 5 + 0],
						i * 5 + 1, buf[i * 5 + 1],
						i * 5 + 2, buf[i * 5 + 2],
						i * 5 + 3, buf[i * 5 + 3],
						i * 5 + 4, buf[i * 5 + 4]);

				} else if (type == 2) {
					wpa_printf(
						level,
						"[%02d]:%ld, [%02d]:%ld, [%02d]:%ld, [%02d]:%ld, [%02d]:%ld,",
						i * 5 + 0, u4Buf[i * 5 + 0],
						i * 5 + 1, u4Buf[i * 5 + 1],
						i * 5 + 2, u4Buf[i * 5 + 2],
						i * 5 + 3, u4Buf[i * 5 + 3],
						i * 5 + 4, u4Buf[i * 5 + 4]);
				} else {
					wpa_printf(
						level,
						"[%02d]:0x%02x, [%02d]:0x%02x, [%02d]:0x%02x, [%02d]:0x%02x, [%02d]:0x%02x,",
						i * 5 + 0, buf[i * 5 + 0],
						i * 5 + 1, buf[i * 5 + 1],
						i * 5 + 2, buf[i * 5 + 2],
						i * 5 + 3, buf[i * 5 + 3],
						i * 5 + 4, buf[i * 5 + 4]);
				}
			}

			if (u1Rest) {
				for (i = 0; i < u1Rest; i++) {
					if (type == 1)
						wpa_printf(
							level, "[%02d]:%c",
							u1Round * 5 + i,
							buf[u1Round * 5 + i]);
					else if (type == 2)
						wpa_printf(
							level, "[%02d]:%ld",
							u1Round * 5 + i,
							u4Buf[u1Round * 5 + i]);
					else
						wpa_printf(
							level, "[%02d]:0x%02x",
							u1Round * 5 + i,
							buf[u1Round * 5 + i]);
				}
			}
		}
	}
	/*#endif*/
}

void
wpa_hexdump_dbg(int level, const char *title, const void *buf, size_t len) {
	_wpa_hexdump(level, title, buf, len, 1, 0);
}
#elif (ENABLE_WPA_PRINT == 3)
/*void wpa_msg(void *ctx, int level, const char *fmt, ...)
*{
*    wpa_printf(level, fmt);
*}
*/
#endif

struct wpabuf *
_wpabuf_alloc(size_t len) {
	return NULL;
}

void *
wpabuf_put(struct wpabuf *buf, size_t len) {
	return NULL;
}

int
wpa_supplicant_parse_ies_wpa(const u8 *buf, size_t len,
			     struct wpa_eapol_ie_parse *ie) {
	return 0;
}
