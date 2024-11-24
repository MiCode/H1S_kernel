/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 - 2021 MediaTek Inc.
 */


#ifndef _GPS_H_
#define _GPS_H_

#include "wmt_core.h"
#include "wmt_dev.h"
#include "osal.h"
#include "mtk_wcn_consys_hw.h"

#if defined(CONFIG_MACH_MT6765) || defined(CONFIG_MACH_MT6761) || defined(CONFIG_MACH_MT6779) \
|| defined(CONFIG_MACH_MT6768) || defined(CONFIG_MACH_MT6785) || defined(CONFIG_MACH_MT6873) \
|| defined(CONFIG_MACH_MT6853) || defined(CONFIG_MACH_MT6833) || defined(CONFIG_MACH_MT6781)
#define GPS_FWCTL_SUPPORT
#endif

#ifdef GPS_FWCTL_SUPPORT
/* Disable: #define GPS_FWCTL_IOCTL_SUPPORT */

#define GPS_HW_SUSPEND_SUPPORT
#endif /* GPS_FWCTL_SUPPORT */

enum gps_ctrl_status_enum {
	GPS_CLOSED,
	GPS_OPENED,
	GPS_SUSPENDED,
	GPS_RESET_START,
	GPS_RESET_DONE,
	GPS_CTRL_STATUS_NUM
};

enum gps_reference_count_cmd {
	FWLOG_CTRL_INNER = 0,
	HANDLE_DESENSE,
	GPS_FWCTL_READY,
};

#ifdef GPS_FWCTL_SUPPORT
#define GPS_FWCTL_OPCODE_ENTER_SLEEP_MODE (2)
#define GPS_FWCTL_OPCODE_EXIT_SLEEP_MODE  (3)
#define GPS_FWCTL_OPCODE_ENTER_STOP_MODE  (4)
#define GPS_FWCTL_OPCODE_EXIT_STOP_MODE   (5)
#define GPS_FWCTL_OPCODE_LOG_CFG          (6)
#define GPS_FWCTL_OPCODE_LOOPBACK_TEST    (7)
#define GPS2_FWCTL_OPCODE_ENTER_SLEEP_MODE (0x12)
#define GPS2_FWCTL_OPCODE_EXIT_SLEEP_MODE  (0x13)
#define GPS2_FWCTL_OPCODE_ENTER_STOP_MODE (0x14)
#define GPS2_FWCTL_OPCODE_EXIT_STOP_MODE  (0x15)

#endif

enum gps_data_link_id_enum {
	GPS_DATA_LINK_ID0	= 0,
	GPS_DATA_LINK_ID1	= 1,
	GPS_DATA_LINK_NUM	= 2,
};
extern int gps_stp_get_reserved_memory_lk(struct device *dev);

extern void GPS_reference_count(enum gps_reference_count_cmd cmd, bool flag, enum gps_data_link_id_enum user);

extern phys_addr_t gConEmiPhyBase;

extern int mtk_wcn_stpgps_drv_init(void);
extern void mtk_wcn_stpgps_drv_exit(void);
#ifdef CONFIG_MTK_GPS_EMI
extern int mtk_gps_emi_init(void);
extern void mtk_gps_emi_exit(void);
#endif
#ifdef CONFIG_MTK_CONNSYS_DEDICATED_LOG_PATH
extern int mtk_gps_fw_log_init(void);
extern void mtk_gps_fw_log_exit(void);
void GPS_fwlog_ctrl(bool on);
#endif

#ifdef MTK_GENERIC_HAL
/* stp_chrdev_gps2 */
extern struct wakeup_source *gps2_wake_lock_ptr;
extern const char gps2_wake_lock_name[];
extern struct semaphore wr_mtx2, rd_mtx2, status_mtx2;
extern const struct file_operations GPS2_fops;
#else
#ifdef CONFIG_GPSL5_SUPPORT
/* stp_chrdev_gps2 */
extern struct wakeup_source *gps2_wake_lock_ptr;
extern const char gps2_wake_lock_name[];
extern struct semaphore wr_mtx2, rd_mtx2, status_mtx2;
extern const struct file_operations GPS2_fops;
#endif
#endif

extern struct semaphore fwctl_mtx;

#ifdef CONFIG_MTK_CONNSYS_DEDICATED_LOG_PATH
extern bool fgGps_fwlog_on;
#endif

#ifdef GPS_FWCTL_SUPPORT
extern bool fgGps_fwctl_ready;
#endif

extern struct semaphore lna_mtx;

#endif
