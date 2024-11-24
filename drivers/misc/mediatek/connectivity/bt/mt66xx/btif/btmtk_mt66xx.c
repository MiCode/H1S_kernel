/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */


#include <linux/kthread.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of.h>
#include <linux/rtc.h>

#include "btmtk_define.h"
#include "btmtk_chip_if.h"
#include "btmtk_main.h"
#include "btmtk_dbg_tp_evt_if.h"
#include "conninfra.h"
#include "connsys_debug_utility.h"
#include "connfem_api.h"


/*******************************************************************************
*                                 M A C R O S
********************************************************************************
*/

/*******************************************************************************
*                              C O N S T A N T S
********************************************************************************
*/
#define PATCH_FILE_NUM			2
#define WMT_CMD_HDR_LEN			(4)

#define EMI_BGFSYS_MCU_START    	0
#define EMI_BGFSYS_MCU_SIZE     	0x100000 /* 1024*1024 */
#define EMI_BT_START            	(EMI_BGFSYS_MCU_START + EMI_BGFSYS_MCU_SIZE)
#define EMI_BT_SIZE             	0x129400 /* 1189*1024 */
#define EMI_DATETIME_LEN		16 // 14 bytes(ex: "20190821054545") + end of string
#define EMI_COREDUMP_MCU_DATE_OFFSET	0xE0
#define EMI_COREDUMP_BT_DATE_OFFSET	0xF0

/*******************************************************************************
*                             D A T A   T Y P E S
********************************************************************************
*/
struct fw_patch_emi_hdr {
	uint8_t date_time[EMI_DATETIME_LEN];
	uint8_t plat[4];
	uint16_t hw_ver;
	uint16_t sw_ver;
	uint8_t emi_addr[4];
	uint32_t subsys_id;
	uint8_t reserved[14];
	uint16_t crc;
};

#if (CUSTOMER_FW_UPDATE == 1)
struct fwp_info {
	bool result;
	uint8_t status;
	uint8_t datetime[PATCH_FILE_NUM][EMI_DATETIME_LEN];
	uint8_t update_time[EMI_DATETIME_LEN];
};
#else
struct fwp_info{
	bool result;
	uint8_t datetime[PATCH_FILE_NUM][EMI_DATETIME_LEN];
};
#endif

enum FWP_LOAD_FROM {
	FWP_DIR_DEFAULT,
	FWP_DIR_UPDATE,
};

enum FWP_CHECK_STATUS {
	FWP_CHECK_SUCCESS,
	FWP_NO_PATCH,
	FWP_CRC_ERROR,
	FWP_ERROR,
	FWP_OLDER_DATETIME,
	FWP_FUNCTION_DISABLE,
};

/*******************************************************************************
*                            P U B L I C   D A T A
********************************************************************************
*/
static const uint8_t WMT_OVER_HCI_CMD_HDR[] = { 0x01, 0x6F, 0xFC, 0x00 };

#if (CUSTOMER_FW_UPDATE == 1)
static bool g_fwp_update_enable = FALSE;
uint8_t g_fwp_names[PATCH_FILE_NUM][2][FW_NAME_LEN] = {};
#else
uint8_t g_fwp_names[PATCH_FILE_NUM][1][FW_NAME_LEN] = {};
#endif
static struct fwp_info g_fwp_info;

/*******************************************************************************
*                           P R I V A T E   D A T A
********************************************************************************
*/

extern bool g_bt_trace_pt;
extern struct btmtk_dev *g_sbdev;
extern struct bt_dbg_st g_bt_dbg_st;

struct bt_base_addr bt_reg;
/*******************************************************************************
*                  F U N C T I O N   D E C L A R A T I O N S
********************************************************************************
*/
#if (CUSTOMER_FW_UPDATE == 1)
static uint16_t fwp_checksume16(uint8_t *pData, uint64_t len) {
    int32_t sum = 0;

    while (len > 1) {
        sum += *((uint16_t*)pData);
        pData = pData + 2;

        if (sum & 0x80000000) {
            sum = (sum & 0xFFFF) + (sum >> 16);
        }

        len -= 2;
    }

    if (len) {
        sum += *((uint8_t*)pData);
    }

    while (sum >> 16) {
        sum = (sum & 0xFFFF) + (sum >> 16);
    }
    return ~sum;
}

static enum FWP_CHECK_STATUS fwp_check_patch (
		uint8_t *patch_name,
		uint8_t *patch_datetime)
{
	uint8_t *p_buf = NULL;
	struct fw_patch_emi_hdr *p_patch_hdr = NULL;
	uint32_t patch_size;
	uint8_t header_szie = sizeof(struct fw_patch_emi_hdr);
	uint16_t crc;
	int32_t ret = FWP_CHECK_SUCCESS;

	/*  Read Firmware patch content */
	if(btmtk_load_code_from_bin(&p_buf, patch_name, NULL, &patch_size, 2) == -1) {
		ret = FWP_NO_PATCH;
		goto done;
	} else if(patch_size < header_szie) {
		BTMTK_ERR("%s: patch size %u error", __func__, patch_size);
		ret = FWP_ERROR;
		goto done;
	}

	/* Check patch header information */
	p_patch_hdr = (struct fw_patch_emi_hdr *)p_buf;
	strncpy(patch_datetime, p_patch_hdr->date_time, EMI_DATETIME_LEN);
	patch_datetime[EMI_DATETIME_LEN - 2] = '\0'; // 14 bytes actually

	/* Caculate crc from body patch */
	crc = fwp_checksume16(p_buf + header_szie, patch_size - header_szie);
	if(crc != p_patch_hdr->crc) {
		ret = FWP_CRC_ERROR;
	}
done:
	BTMTK_INFO("%s: patch_name[%s], ret[%d]", __func__, patch_name, ret);
	if (p_buf)
		vfree(p_buf);
	return ret;
}

static enum FWP_LOAD_FROM fwp_preload_patch(struct fwp_info *info)
{
	uint8_t i = 0;
	bool result = FWP_DIR_UPDATE;
	uint8_t status = FWP_CHECK_SUCCESS;
	uint8_t time_d[EMI_DATETIME_LEN], time_u[EMI_DATETIME_LEN];

	if(g_fwp_update_enable) {
		for(i = 0; i < PATCH_FILE_NUM; i++) {
			fwp_check_patch(g_fwp_names[i][FWP_DIR_DEFAULT], time_d);
			status = fwp_check_patch(g_fwp_names[i][FWP_DIR_UPDATE], time_u);
			if(status != FWP_CHECK_SUCCESS) {
				// if there is any error on update patch
				result = FWP_DIR_DEFAULT;
				goto done;
			} else {
				BTMTK_INFO("%s: %s, datetime[%s]", __func__, g_fwp_names[i][FWP_DIR_DEFAULT], time_d);
				BTMTK_INFO("%s: %s: datetime[%s]", __func__, g_fwp_names[i][FWP_DIR_UPDATE], time_u);
				if(strcmp(time_u, time_d) < 0) {
					// if any update patch datetime is older
					status = FWP_OLDER_DATETIME;
					result = FWP_DIR_DEFAULT;
					goto done;
				}
			}
		}
	} else {
		result = FWP_DIR_DEFAULT;
		status = FWP_FUNCTION_DISABLE;
	}
done:
	info->result = result;
	info->status = status;
	return result;
}

static void fwp_update_info(struct fwp_info *info) {
	struct timespec64 time;
	struct rtc_time tm;
	unsigned long local_time;
	int i;

	ktime_get_real_ts64(&time);
	local_time = (uint32_t)(time.tv_nsec/1000 - (sys_tz.tz_minuteswest * 60));
	rtc_time64_to_tm(local_time, &tm);
	if (snprintf(info->update_time, EMI_DATETIME_LEN, "%04d%02d%02d%02d%02d%02d",
			tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec) < 0) {
		BTMTK_INFO("%s: snprintf error", __func__);
		return;
	}
	info->update_time[EMI_DATETIME_LEN - 2] = '\0'; // 14 bytes actually

	BTMTK_INFO("%s: result=%s, status=%d, datetime=%s, update_time=%s", __func__,
		info->result == FWP_DIR_DEFAULT ? "FWP_DIR_DEFAULT" : "FWP_DIR_UPDATE",
		info->status, info->datetime[0], info->update_time);
}

void fwp_if_set_update_enable(int par)
{
	/* 0: disable, 1: enable*/
	g_fwp_update_enable = (par == 0) ? FALSE : TRUE;
	BTMTK_INFO("%s: set fw patch update function = %s", __func__, (par == 0) ? "DISABLE" : "ENABLE");
}

int fwp_if_get_update_info(char *buf, int max_len)
{
	// mapping to FWP_CHECK_STATUS
	char* status_info[] ={
		"none", "no new patch", "crc error", "error", "older version", "function disable"
	};

	/* return update information */
	if (snprintf(buf, max_len - 1, "result=%s\nstatus=%s\nversion=%s\nupdate_time=%s",
			g_fwp_info.result == FWP_DIR_DEFAULT ? "FAIL" : "PASS",
			status_info[g_fwp_info.status],
			g_fwp_info.datetime[0],
			g_fwp_info.update_time) < 0) {
		BTMTK_INFO("%s: snprintf error", __func__);
		return 0;
	}
	buf[strlen(buf)] = '\0';
	BTMTK_INFO("%s: %s, %d", __func__, buf, strlen(buf));
	return strlen(buf) + 1;
}
#endif

int fwp_if_get_datetime(char *buf, int max_len)
{
	#define VER_STR_LEN 60
	int i = 0, ret_len = 0;
	bool fwp_from = g_fwp_info.result;
	char *tmp = buf;

	if (PATCH_FILE_NUM * VER_STR_LEN > max_len)
		return 0;
	/* write datetime information of each patch */
	for (i = 0; i < PATCH_FILE_NUM; i++) {
		if (snprintf(tmp, VER_STR_LEN, "%s: %s\n", g_fwp_names[i][fwp_from], g_fwp_info.datetime[i]) < 0) {
			BTMTK_INFO("%s: snprintf error", __func__);
			return 0;
		}
		ret_len += strlen(tmp);
		tmp = tmp + strlen(tmp);
	}
	buf[ret_len] = '\0';
	BTMTK_INFO("%s: %s, %d", __func__, buf, strlen(buf));
	return ret_len + 1;
}

int fwp_if_get_bt_patch_path(char *buf, int max_len)
{
	#undef VER_STR_LEN
	#define VER_STR_LEN	100
	bool fwp_from = g_fwp_info.result;
	int ret = 0;

	if (VER_STR_LEN > max_len)
		return 0;
	if (fwp_from == FWP_DIR_DEFAULT) {
		ret = snprintf(buf, VER_STR_LEN, "/vendor/firmware/%s", g_fwp_names[1][fwp_from]);
	} else {
#if (CUSTOMER_FW_UPDATE == 1)
		ret = snprintf(buf, VER_STR_LEN, "/data/vendor/firmware/update/%s", g_fwp_names[1][fwp_from]);
#else
		ret = snprintf(buf, VER_STR_LEN, "");
#endif
	}

	if (ret < 0) {
		BTMTK_INFO("%s: snprintf error", __func__);
		return 0;
	}
	BTMTK_INFO("%s: %s, %d", __func__, buf, strlen(buf));
	return strlen(buf) + 1;
}

/* bgfsys_cal_data_backup
 *
 *    Backup BT calibration data
 *
 * Arguments:
 *    [IN] start_addr   - offset to the SYSRAM
 *    [IN] cal_data     - data buffer to backup calibration data
 *    [IN] data_len     - data length
 *
 * Return Value:
 *    N/A
 *
 */
static void bgfsys_cal_data_backup(
	uint32_t start_addr,
	uint8_t *cal_data,
	uint16_t data_len)
{
	uint32_t start_offset;

	/*
	 * The calibration data start address and ready bit address returned in WMT RF cal event
	 * should be 0x7C05XXXX from F/W's point of view (locate at ConnInfra sysram region),
	 * treat the low 3 bytes as offset to our remapped virtual base.
	 */
	start_offset = start_addr & 0x00000FFF;
	if (start_offset + data_len > 0x1000) {
		BTMTK_ERR("Error sysram offset address start=[0x%08x], len = [%d]",
								start_addr, data_len);
		return;
	}

	if (!conninfra_reg_readable()) {
		int32_t ret = conninfra_is_bus_hang();
		if (ret > 0)
			BTMTK_ERR("%s: conninfra bus is hang, needs reset", __func__);
		else
			BTMTK_ERR("%s: conninfra not readable, but not bus hang ret = %d", __func__, ret);
		return;
	}

	memcpy_fromio(cal_data, (const volatile void *)(CON_REG_INFRA_SYS_ADDR + start_offset), data_len);
}

/* bgfsys_cal_data_restore
 *
 *    Restore BT calibration data to SYSRAM, after restore success,
 *    driver needs to write a special pattern '0x5AD02EA5' to ready address to
 *    info FW not to do calibration this time
 *
 * Arguments:
 *    [IN] start_addr   - offset to the SYSRAM
 *    [IN] ready_addr   - readya_address to write 0x5AD02EA5
 *    [IN] cal_data     - data buffer of calibration data
 *    [IN] data_len     - data length
 *
 * Return Value:
 *    N/A
 *
 */
static void bgfsys_cal_data_restore(uint32_t start_addr,
					 	uint32_t ready_addr,
						uint8_t *cal_data,
						uint16_t data_len)
{
	uint32_t start_offset, ready_offset;
	uint32_t ready_status = 0;

	start_offset = start_addr & 0x00000FFF;
	ready_offset = ready_addr & 0x00000FFF;

	if (start_offset > 0x1000 || ready_offset > 0x1000) {
		BTMTK_ERR("Error sysram offset address start=[0x%08x], ready=[0x%08x]",
							start_addr, ready_addr);
		return;
	}

	if (!conninfra_reg_readable()) {
		int32_t ret = conninfra_is_bus_hang();
		if (ret > 0)
			BTMTK_ERR("%s: conninfra bus is hang, needs reset", __func__);
		else
			BTMTK_ERR("%s: conninfra not readable, but not bus hang ret = %d", __func__, ret);
		return;
	}

	memcpy_toio((volatile void *)(CON_REG_INFRA_SYS_ADDR + start_offset), cal_data, data_len);
	/* Firmware will not do calibration again when BT func on */
	REG_WRITEL(CON_REG_INFRA_SYS_ADDR + ready_offset, CAL_READY_BIT_PATTERN);
	ready_status = REG_READL(CON_REG_INFRA_SYS_ADDR + ready_offset);
	BTMTK_DBG("Ready pattern after restore cal=[0x%08x]", ready_status);
}

/* __download_patch_to_emi
 *
 *    Download(copy) FW to EMI
 *
 * Arguments:
 *    [IN] patch_name   - FW bin filename
 *    [IN] emi_start    - offset to EMI address for BT / MCU
 *    [IN] emi_size     - EMI region
 *
 * Return Value:
 *     0 if success, otherwise error code
 *
 */
static int32_t __download_patch_to_emi(
		uint8_t *patch_name,
		uint32_t emi_start,
		uint32_t emi_size,
#if SUPPORT_COREDUMP
		phys_addr_t fwdate_offset,
#endif
		uint8_t *datetime)
{
	int32_t ret = 0;
	struct fw_patch_emi_hdr *p_patch_hdr = NULL;
	uint8_t *p_buf = NULL, *p_img = NULL;
	uint32_t patch_size = 0;
	uint16_t hw_ver = 0;
	uint16_t sw_ver = 0;
	uint32_t subsys_id = 0;
	uint32_t patch_emi_offset = 0;
	phys_addr_t emi_ap_phy_base;
	uint8_t *remap_addr = NULL;

	BTMTK_INFO("%s: load binary [%s]", __func__, patch_name);
	/*  Read Firmware patch content */
	btmtk_load_code_from_bin(&p_img, patch_name, NULL, &patch_size, 10);
	if (p_img == NULL || patch_size < sizeof(struct fw_patch_emi_hdr)) {
		BTMTK_ERR("patch size %u error", patch_size);
		ret = -EINVAL;
		goto done;
	}

	/* Patch binary format:
	 * |<-EMI header: 48 Bytes->|<-BT/MCU header: 48 Bytes ->||<-patch body: X Bytes ----->|
	 */
	/* Check patch header information */
	p_buf = p_img;
	p_patch_hdr = (struct fw_patch_emi_hdr *)p_buf;


	hw_ver = p_patch_hdr->hw_ver;
	sw_ver = p_patch_hdr->sw_ver;
	subsys_id = p_patch_hdr->subsys_id;
	strncpy(datetime, p_patch_hdr->date_time, sizeof(p_patch_hdr->date_time));
	datetime[sizeof(p_patch_hdr->date_time) - 2] = '\0'; // 14 bytes actually
	BTMTK_INFO(
		"[Patch]BTime=%s,HVer=0x%04x,SVer=0x%04x,Plat=%c%c%c%c,Addr=0x%02x%02x%02x%02x,Type=%x",
		datetime,
		((hw_ver & 0x00ff) << 8) | ((hw_ver & 0xff00) >> 8),
		((sw_ver & 0x00ff) << 8) | ((sw_ver & 0xff00) >> 8),
		p_patch_hdr->plat[0], p_patch_hdr->plat[1],
		p_patch_hdr->plat[2], p_patch_hdr->plat[3],
		p_patch_hdr->emi_addr[3], p_patch_hdr->emi_addr[2],
		p_patch_hdr->emi_addr[1], p_patch_hdr->emi_addr[0],
		subsys_id);

	/* Remove EMI header:
	 * |<-BT/MCU header: 48 Bytes ->||<-patch body: X Bytes ----->|
	 */
	patch_size -= sizeof(struct fw_patch_emi_hdr);
	p_buf += sizeof(struct fw_patch_emi_hdr);

	/*
	 * The EMI entry address given in patch header should be 0xFXXXXXXX
	 * from F/W's point of view, treat the middle 2 bytes as offset,
	 * the actual phy base should be dynamically allocated and provided
	 * by conninfra driver.
	 *
	 */
	patch_emi_offset = (p_patch_hdr->emi_addr[2] << 16) |
		(p_patch_hdr->emi_addr[1] << 8);

	conninfra_get_phy_addr(&emi_ap_phy_base, NULL);

	//if ((patch_emi_offset >= emi_start) &&
	//    (patch_emi_offset + patch_size < emi_start + emi_size)) {
		remap_addr = ioremap(emi_ap_phy_base + patch_emi_offset, patch_size);
		BTMTK_INFO("[Patch] emi_ap_phy_base[0x%p], remap_addr[0x%08x]", emi_ap_phy_base, *remap_addr);
		BTMTK_INFO("[Patch] patch_emi_offset[0x%08x], patch_size[0x%08x]", patch_emi_offset, patch_size);

		if (remap_addr) {
			memcpy_toio(remap_addr, p_buf, patch_size);
			iounmap(remap_addr);
		} else {
			BTMTK_ERR("ioremap fail!");
			ret = -EFAULT;
		}
	//} else {
	//	BTMTK_ERR("emi_start =0x%x size=0x%x", emi_start, emi_size);
	//	BTMTK_ERR("Patch overflow on EMI, offset=0x%x size=0x%x",
	//			  patch_emi_offset, patch_size);
	//	ret = -EINVAL;
	//}

#if SUPPORT_COREDUMP
	remap_addr = ioremap(emi_ap_phy_base + fwdate_offset, sizeof(p_patch_hdr->date_time));
	if (remap_addr) {
		memcpy_toio(remap_addr, p_patch_hdr->date_time, sizeof(p_patch_hdr->date_time));
		iounmap(remap_addr);
	} else
		BTMTK_ERR("ioremap coredump data field fail");
#endif

done:
	if (p_img)
		vfree(p_img);
	return ret;
}

/* bgfsys_mcu_rom_patch_dl
 *
 *    Download BGF MCU rom patch
 *
 * Arguments:
 *    N/A
 *
 * Return Value:
 *     0 if success, otherwise error code
 *
 */
static int32_t bgfsys_mcu_rom_patch_dl(struct fwp_info *info)
{
	int32_t ret = 0;
#if SUPPORT_COREDUMP
	phys_addr_t fwdate_offset = connsys_coredump_get_start_offset(CONN_DEBUG_TYPE_BT) +
				    EMI_COREDUMP_MCU_DATE_OFFSET;
#endif

	ret = bgfsys_check_conninfra_ready();
	if (ret)
		return ret;

	return __download_patch_to_emi(g_fwp_names[0][info->result],
				       EMI_BGFSYS_MCU_START,
				       EMI_BGFSYS_MCU_SIZE,
#if SUPPORT_COREDUMP
				       fwdate_offset,
#endif
				       info->datetime[0]);
}

/* bgfsys_mcu_rom_patch_dl
 *
 *    Download BT ram patch
 *
 * Arguments:
 *    N/A
 *
 * Return Value:
 *     0 if success, otherwise error code
 *
 */
static int32_t bgfsys_bt_ram_code_dl(struct fwp_info *info)
{
#if SUPPORT_COREDUMP
	phys_addr_t fwdate_offset = connsys_coredump_get_start_offset(CONN_DEBUG_TYPE_BT) +
				    EMI_COREDUMP_BT_DATE_OFFSET;
#endif

	return __download_patch_to_emi(g_fwp_names[1][info->result],
				       EMI_BT_START,
				       EMI_BT_SIZE,
#if SUPPORT_COREDUMP
				       fwdate_offset,
#endif
				       info->datetime[1]);
}

int32_t bgfsys_bt_patch_dl(void)
{
	int32_t ret = -1;
	ret = bgfsys_mcu_rom_patch_dl(&g_fwp_info);
	if (ret)
		return ret;

	return bgfsys_bt_ram_code_dl(&g_fwp_info);
}

/* bt_hw_and_mcu_on
 *
 *    BT HW / MCU / HAL poweron/init flow
 *
 * Arguments:
 *    N/A
 *
 * Return Value:
 *     0 if success, otherwise error code
 *
 */
static int32_t bt_hw_and_mcu_on(void)
{
	int ret = -1;
	enum FWP_LOAD_FROM fwp_from = FWP_DIR_DEFAULT;

	memset(&g_fwp_info, '\0', sizeof(struct fwp_info));
#if (CUSTOMER_FW_UPDATE == 1)
	fwp_from = fwp_preload_patch(&g_fwp_info);
#else
	g_fwp_info.result = fwp_from;
#endif

	/*
	 * Firmware patch download (ROM patch, RAM code...)
	 * start MCU to enter idle loop after patch ready
	 */
	ret = btmtk_load_rom_patch(g_sbdev);
	if (ret)
		goto power_on_error;

#if (CUSTOMER_FW_UPDATE == 1)
	fwp_update_info(&g_fwp_info);
#endif

	/* BGFSYS hardware power on */
	if (bgfsys_power_on()) {
		BTMTK_ERR("BGFSYS power on failed!");
		ret = -EIO;
		goto power_on_error;
	}

	/*reset sw_irq*/
	//bgfsys_ack_sw_irq_reset();
	//bgfsys_ack_sw_irq_fwlog();

	/* Register all needed IRQs by MCU */
	ret = bt_request_irq(BGF2AP_BTIF_WAKEUP_IRQ);
	if (ret)
		goto request_irq_error;

	bt_disable_irq(BGF2AP_BTIF_WAKEUP_IRQ);

	ret = bt_request_irq(BGF2AP_SW_IRQ);
	if (ret)
		goto request_irq_error2;

	bt_disable_irq(BGF2AP_SW_IRQ);

	if (BT_SSPM_TIMER) {
		ret = bt_request_irq(BT_CONN2AP_SW_IRQ);
		if (ret)
			goto bus_operate_error;
		bt_disable_irq(BT_CONN2AP_SW_IRQ);
	}

	btmtk_reset_init();

	if (btmtk_wcn_btif_open()) {
		ret = -EIO;
		goto bus_operate_error;
	}
	return 0;


bus_operate_error:
	bt_free_irq(BGF2AP_SW_IRQ);

request_irq_error2:
	bt_free_irq(BGF2AP_BTIF_WAKEUP_IRQ);

request_irq_error:
power_on_error:
	bgfsys_power_off();
	return ret;
}

/* bt_hw_and_mcu_off
 *
 *    BT HW / MCU / HAL poweron/deinit flow
 *
 * Arguments:
 *    N/A
 *
 * Return Value:
 *    N/A
 *
 */
static void bt_hw_and_mcu_off(void)
{
	BTMTK_INFO("%s", __func__);
	/* Close hardware bus interface */
	btmtk_wcn_btif_close();

	bt_disable_irq(BGF2AP_SW_IRQ);
	bt_disable_irq(BGF2AP_BTIF_WAKEUP_IRQ);

	/* Free all registered IRQs */
	bt_free_irq(BGF2AP_SW_IRQ);
	bt_free_irq(BGF2AP_BTIF_WAKEUP_IRQ);

	if (BT_SSPM_TIMER) {
		bt_disable_irq(BT_CONN2AP_SW_IRQ);
		bt_free_irq(BT_CONN2AP_SW_IRQ);
	}
	/* BGFSYS hardware power off */
	bgfsys_power_off();
}

uint8_t *_internal_evt_result(u_int8_t wmt_evt_result)
{

	if (wmt_evt_result == WMT_EVT_SUCCESS)
		return "WMT_EVT_SUCCESS";
	else if (wmt_evt_result == WMT_EVT_FAIL)
		return "WMT_EVT_FAIL";
	else if (wmt_evt_result == WMT_EVT_INVALID)
		return "WMT_EVT_INVALID";
	else
		return "WMT_EVT_SKIP";
}

/* _send_wmt_power_cmd
 *
 *    Send BT func on/off command to FW
 *
 * Arguments:
 *    [IN] hdev     - hci_device as control structure during BT life cycle
 *    [IN] is_on    - indicate current action is On (TRUE) or Off (FALSE)
 *
 * Return Value:
 *     0 if success, otherwise -EIO
 *
 */
static int32_t _send_wmt_power_cmd(struct hci_dev *hdev, u_int8_t is_on)
{
	struct btmtk_dev *bdev = hci_get_drvdata(hdev);
	struct wmt_pkt wmt_cmd;
	uint8_t buffer[HCI_MAX_FRAME_SIZE];
	uint16_t param_len, pkt_len;
	struct btmtk_btif_dev *cif_dev = (struct btmtk_btif_dev *)g_sbdev->cif_dev;
	struct bt_internal_cmd *p_inter_cmd = &cif_dev->internal_cmd;
	int ret;

	BTMTK_INFO("[InternalCmd] %s", __func__);

	down(&cif_dev->internal_cmd_sem);
	cif_dev->event_intercept = TRUE;

	wmt_cmd.hdr.dir = WMT_PKT_DIR_HOST_TO_CHIP;

	/* Support Connac 2.0 */
	wmt_cmd.hdr.opcode = WMT_OPCODE_FUNC_CTRL;
	wmt_cmd.params.u.func_ctrl_cmd.subsys = 0;
	wmt_cmd.params.u.func_ctrl_cmd.on = is_on ? 1 : 0;
	param_len = sizeof(wmt_cmd.params.u.func_ctrl_cmd);

	wmt_cmd.hdr.param_len[0] = (uint8_t)(param_len & 0xFF);
	wmt_cmd.hdr.param_len[1] = (uint8_t)((param_len >> 8) & 0xFF);

	pkt_len = HCI_CMD_HDR_LEN + WMT_CMD_HDR_LEN + param_len;
	memcpy(buffer, WMT_OVER_HCI_CMD_HDR, HCI_CMD_HDR_LEN);
	buffer[3] = WMT_CMD_HDR_LEN + param_len;
	memcpy(buffer + HCI_CMD_HDR_LEN, &wmt_cmd, WMT_CMD_HDR_LEN + param_len);

	p_inter_cmd->waiting_event = 0xE4;
	p_inter_cmd->pending_cmd_opcode = 0xFC6F;
	p_inter_cmd->wmt_opcode = WMT_OPCODE_FUNC_CTRL;
	p_inter_cmd->result = WMT_EVT_INVALID;

	ret = btmtk_main_send_cmd(bdev, buffer, pkt_len, NULL, 0, 0, 0, BTMTK_TX_WAIT_VND_EVT);
	if (ret <= 0 && is_on) {
		BTMTK_ERR("%s: Unable to get event in time, start dump and reset!", __func__);
		bt_trigger_reset();
	}

	ret = (p_inter_cmd->result == WMT_EVT_SUCCESS) ? 0 : -EIO;
	cif_dev->event_intercept = FALSE;
	up(&cif_dev->internal_cmd_sem);
	BTMTK_INFO("[InternalCmd] %s done, result = %s", __func__, _internal_evt_result(p_inter_cmd->result));
	return ret;
}

/* _send_wmt_get_cal_data_cmd
 *
 *    Send query calibration data command to FW and wait for response (event)
 *    to get calibration data (for backup calibration data purpose)
 *
 *
 * Arguments:
 *    [IN]  hdev     - hci_device as control structure during BT life cycle
 *    [OUT] p_start_addr - start offset to SYSRAM that stores calibration data
 *    [OUT] p_ready_addr - ready address for indicating restore calibration data
 *                         successfully
 *    [OUT] p_data_len   - length of calibration data
 *
 * Return Value:
 *     0 if success, otherwise -EIO
 *
 */
static int32_t _send_wmt_get_cal_data_cmd(
	struct hci_dev *hdev,
	uint32_t *p_start_addr,
	uint32_t *p_ready_addr,
	uint16_t *p_data_len
)
{
	struct btmtk_dev *bdev = hci_get_drvdata(hdev);
	struct wmt_pkt wmt_cmd;
	uint8_t buffer[HCI_MAX_FRAME_SIZE];
	uint16_t param_len, pkt_len;
	struct btmtk_btif_dev *cif_dev = (struct btmtk_btif_dev *)g_sbdev->cif_dev;
	struct bt_internal_cmd *p_inter_cmd = &cif_dev->internal_cmd;
	int ret;

	BTMTK_INFO("[InternalCmd] %s", __func__);

	down(&cif_dev->internal_cmd_sem);
	cif_dev->event_intercept = TRUE;

	wmt_cmd.hdr.dir = WMT_PKT_DIR_HOST_TO_CHIP;
	wmt_cmd.hdr.opcode = WMT_OPCODE_RF_CAL;
	wmt_cmd.params.u.rf_cal_cmd.subop = 0x03;
	param_len = sizeof(wmt_cmd.params.u.rf_cal_cmd);


	wmt_cmd.hdr.param_len[0] = (uint8_t)(param_len & 0xFF);
	wmt_cmd.hdr.param_len[1] = (uint8_t)((param_len >> 8) & 0xFF);

	pkt_len = HCI_CMD_HDR_LEN + WMT_CMD_HDR_LEN + param_len;
	memcpy(buffer, WMT_OVER_HCI_CMD_HDR, HCI_CMD_HDR_LEN);
	buffer[3] = WMT_CMD_HDR_LEN + param_len;
	memcpy(buffer + HCI_CMD_HDR_LEN, &wmt_cmd, WMT_CMD_HDR_LEN + param_len);

	/* Save the necessary information to internal_cmd struct */
	p_inter_cmd->waiting_event = 0xE4;
	p_inter_cmd->pending_cmd_opcode = 0xFC6F;
	p_inter_cmd->wmt_opcode = WMT_OPCODE_RF_CAL;
	p_inter_cmd->result = WMT_EVT_INVALID;

	ret = btmtk_main_send_cmd(bdev, buffer, pkt_len, NULL, 0, 0, 0, BTMTK_TX_WAIT_VND_EVT);

	if (ret <= 0) {
		BTMTK_ERR("Unable to get calibration event in time, start dump and reset!");
		// TODO: FW request dump & reset, need apply to all internal cmdå
		bt_trigger_reset();
		return -1;
	}

	*p_start_addr = (p_inter_cmd->wmt_event_params.u.rf_cal_evt.start_addr[3] << 24) |
			(p_inter_cmd->wmt_event_params.u.rf_cal_evt.start_addr[2] << 16) |
			(p_inter_cmd->wmt_event_params.u.rf_cal_evt.start_addr[1] << 8) |
			(p_inter_cmd->wmt_event_params.u.rf_cal_evt.start_addr[0]);
	*p_ready_addr = (p_inter_cmd->wmt_event_params.u.rf_cal_evt.ready_addr[3] << 24) |
			(p_inter_cmd->wmt_event_params.u.rf_cal_evt.ready_addr[2] << 16) |
			(p_inter_cmd->wmt_event_params.u.rf_cal_evt.ready_addr[1] << 8) |
			(p_inter_cmd->wmt_event_params.u.rf_cal_evt.ready_addr[0]);
	*p_data_len = (p_inter_cmd->wmt_event_params.u.rf_cal_evt.data_len[1] << 8) |
		      (p_inter_cmd->wmt_event_params.u.rf_cal_evt.data_len[0]);

	if (p_inter_cmd->result == WMT_EVT_SUCCESS)
		ret = 0;
	else {
		uint32_t offset = *p_start_addr & 0x00000FFF;
		uint8_t *data = NULL;

		if(offset > 0x1000)
			BTMTK_ERR("Error calibration offset (%d)", offset);
		else {
			data = (uint8_t *)(CON_REG_INFRA_SYS_ADDR + offset);

			BTMTK_ERR("Calibration fail, dump calibration data");
			if(*p_data_len < BT_CR_DUMP_BUF_SIZE)
				bt_dump_memory8(data, *p_data_len);
			else
				BTMTK_ERR("get wrong calibration length [%d]", *p_data_len);
		}
		ret = -EIO;
	}

	/* Whether succeed or not, no one is waiting, reset the flag here */
	cif_dev->event_intercept = FALSE;
	up(&cif_dev->internal_cmd_sem);
	BTMTK_INFO("[InternalCmd] %s done, result = %s", __func__, _internal_evt_result(p_inter_cmd->result));
	return ret;
}

int btmtk_inttrx_DynamicAdjustTxPower_cb(uint8_t *buf, int len)
{
	struct btmtk_btif_dev *cif_dev = (struct btmtk_btif_dev *)g_sbdev->cif_dev;

	// do nothing if not support
	if (*(buf + 5) != HCI_EVT_CC_STATUS_SUCCESS) {
		BTMTK_INFO("%s: status error[0x%02x]!", __func__, *(buf + 5));
		goto callback;
	}

	if (*(buf + 6) == HCI_CMD_DY_ADJ_PWR_QUERY) {
		/* handle command complete(query) */
		// store dbm value
		cif_dev->dy_pwr.dy_max_dbm = *(buf + 7);
		cif_dev->dy_pwr.dy_min_dbm = *(buf + 8);
		cif_dev->dy_pwr.lp_bdy_dbm = *(buf + 9);
	} else {
		/* handle command complete(set) */
		cif_dev->dy_pwr.fw_sel_dbm = *(buf + 7); //store the dbm value fw select
	}

callback:
	BTMTK_INFO("%s: mode[%d], lp_cur_lv[%d], dy_max_dbm[%d], dy_min_dbm[%d], lp_bdy_dbm[%d], fw_sel_dbm[%d]",
		__func__, *(buf + 6),
		cif_dev->dy_pwr.lp_cur_lv,
		cif_dev->dy_pwr.dy_max_dbm,
		cif_dev->dy_pwr.dy_min_dbm,
		cif_dev->dy_pwr.lp_bdy_dbm,
		cif_dev->dy_pwr.fw_sel_dbm);

	// run callback function
	if (cif_dev->dy_pwr.cb) {
		cif_dev->dy_pwr.cb(buf, len);
		cif_dev->dy_pwr.cb = NULL;
	}

	return 0;
}

int btmtk_inttrx_DynamicAdjustTxPower(uint8_t mode, int8_t req_val, BT_RX_EVT_HANDLER_CB cb, bool is_blocking)
{
	struct btmtk_btif_dev *cif_dev = (struct btmtk_btif_dev *)g_sbdev->cif_dev;
	uint8_t cmd_query[5] = {0x01, 0x2D, 0xFC, 0x01, 0x01};
	uint8_t cmd_set[6] = {0x01, 0x2D, 0xFC, 0x02, 0x02, (uint8_t)req_val};
	int8_t set_val;
	/*
	Query
		Cmd: 01 2D FC 01 01
		Evt: 04 0E 08 01 2D FC SS 01 XX YY ZZ
		SS: Status
		XX: Dynamic range Max dBm
		YY: Dynamic range Min dBm
		ZZ: Low power region boundary dBm
	Set
		Cmd: 01 2D FC 02 02 RR
		Evt: 04 0E 06 01 2D FC SS 02 XX
		RR: Requested TX power upper limitation dBm
		SS: Status
		XX: Selected TX power upper limitation dBm
	*/

	if (!bt_pwrctrl_support())
		return 1;

	/* register call back */
	cif_dev->dy_pwr.cb = cb;

	BTMTK_INFO("%s: mode[%d], lp_cur_lv[%d], dy_max_dbm[%d], dy_min_dbm[%d], lp_bdy_dbm[%d], fw_sel_dbm[%d], req_val[%d]",
		__func__, mode,
		cif_dev->dy_pwr.lp_cur_lv,
		cif_dev->dy_pwr.dy_max_dbm,
		cif_dev->dy_pwr.dy_min_dbm,
		cif_dev->dy_pwr.lp_bdy_dbm,
		cif_dev->dy_pwr.fw_sel_dbm,
		req_val);

	if (mode == HCI_CMD_DY_ADJ_PWR_QUERY) {
		// send cmd
		btmtk_btif_internal_trx(cmd_query,
								sizeof(cmd_query),
								btmtk_inttrx_DynamicAdjustTxPower_cb,
								FALSE,
								is_blocking);
	} else {
		// do not send if not support
		if (cif_dev->dy_pwr.dy_max_dbm <= cif_dev->dy_pwr.dy_min_dbm) {
			BTMTK_INFO("%s: invalid dbm range, skip set cmd", __func__);
			return 1;
		}

		/* value select flow */
		if (req_val >= cif_dev->dy_pwr.dy_min_dbm) {
			set_val = req_val;
			// check max limitation
			if (set_val > cif_dev->dy_pwr.dy_max_dbm)
				set_val = cif_dev->dy_pwr.dy_max_dbm;
			// power throttling limitation
			if (cif_dev->dy_pwr.lp_cur_lv >= CONN_PWR_THR_LV_4) { //TODO_PWRCTRL
				set_val = cif_dev->dy_pwr.lp_bdy_dbm;
				// lp_bdy_dbm may be larger than dy_max_dbm, check again
				if (set_val > cif_dev->dy_pwr.dy_max_dbm)
					set_val = cif_dev->dy_pwr.dy_max_dbm;
			}
		} else {
			BTMTK_INFO("%s: invalid dbm value, skip set cmd", __func__);
			return 1;
		}

		BTMTK_INFO("%s: set_val[%d]", __func__, set_val);
		cmd_set[5] = set_val;

		btmtk_btif_internal_trx(cmd_set,
								sizeof(cmd_set),
								btmtk_inttrx_DynamicAdjustTxPower_cb,
								FALSE,
								is_blocking);
	}

	return 0;
}



/* btmtk_intcmd_wmt_power_on
 *
 *    Send BT func on
 *
 * Arguments:
 *    [IN] hdev     - hci_device as control structure during BT life cycle
 *
 * Return Value:
 *     0 if success, otherwise -EIO
 *
 */
int32_t btmtk_intcmd_wmt_power_on(struct hci_dev *hdev)
{
	//struct btmtk_dev *bdev = hci_get_drvdata(hdev);
	struct btmtk_btif_dev *cif_dev = (struct btmtk_btif_dev *)g_sbdev->cif_dev;
	int ret = 0;

	ret = _send_wmt_power_cmd(hdev, TRUE);
	if (!ret) {
		cif_dev->bt_state = FUNC_ON;
		bt_notify_state();
	} else {
		if (++cif_dev->rst_count > 3) {
			conninfra_trigger_whole_chip_rst(CONNDRV_TYPE_BT,
					"power on fail more than 3 times");
			return RET_PWRON_WHOLE_CHIP_RESET;
		}
		bt_dump_cpupcr(10, 5);
		bt_dump_bgfsys_debug_cr();
	}

	return ret;
}

int32_t btmtk_intcmd_wmt_send_antenna_cmd(struct hci_dev *hdev)
{
	#define BT_FW_CFG_FILE	"BT_FW.cfg"
	#define BT_FW_CFG_TAG	"[driver_antenna]"

	struct btmtk_btif_dev *cif_dev = (struct btmtk_btif_dev *)g_sbdev->cif_dev;
	struct bt_internal_cmd *p_inter_cmd = &cif_dev->internal_cmd;
	uint32_t i = 0, len = 0, ret = 0;
	uint8_t *p_img = NULL;
	uint8_t *ptr = NULL, *pRaw = NULL, findTag[32] = {0};
	uint8_t cmd[32] = {0};
	long val = 0;
	uint8_t cmd_header[] =  {0x01, 0x6F, 0xFC, 0x00, 0x01, 0x55, 0x03, 0x00, 0x00};

	BTMTK_DBG("%s: load config [%s]", __func__, BT_FW_CFG_FILE);
	btmtk_load_code_from_bin(&p_img, BT_FW_CFG_FILE, NULL, &len, 10);
	if (p_img == NULL) {
		BTMTK_WARN("%s: get config file fail!", __func__);
		return 0;
	}

	/* find tag: [BT_FW_CFG_TAG][CONNAC20_CHIPID] */
	if (snprintf(findTag, sizeof(findTag), "%s[%d] ", BT_FW_CFG_TAG, CONNAC20_CHIPID) < 0) {
		BTMTK_ERR("%s: snprintf error", __func__);
		ret = -1;
		goto done;
	}

	p_img[len - 1] = 0;
	ptr = strstr(p_img, findTag);
	if (ptr == NULL) {
		BTMTK_WARN("%s: ptr is NULL, do not get corresponding tag. Ignore antenna setting", __func__);
		goto done;
	}

	memcpy(cmd, cmd_header, sizeof(cmd_header));

	/*
	 * command and event example
	 *  0  1  2  3  4  5  6  7  8  9  A
	 * 01 6F FC PP 01 55 LL LL 00 MM NN XX XX XX
	 * PP : WMT length = LL + 4
	 * LL LL : length = NN + 3
	 * MM : ant swap mode
	 * NN : ant pin num
	 * XX XX : NN bytes data
	 * 02 55 02 00 00 SS
	 * SS : status
	 */

	/* parse parameter */
	ptr += (int)strlen(findTag);

	/* find line feed */
	pRaw = ptr;
	while(*pRaw != '\r' && *pRaw != '\n' && pRaw < ptr + len)
		pRaw++;
	*pRaw = 0;

	len = sizeof(cmd_header);
	pRaw = ptr;
	/* separate by space to get paramter */
	for (i = 0; ; i++) {
		ptr = strsep((char **)&pRaw, " ");
		if (ptr != NULL && osal_strtol(ptr, 16, &val) == 0)
			cmd[len++] = val;
		else
			break;
	}

	/* check input size and also boundary check */
	if (cmd[10] != (i - 2) || cmd[10] > 25) {
		BTMTK_ERR("input antenna parameter length incorrect len = %d", cmd[10]);
		ret = -1;
		goto done;
	}

	/* we only allocate 32 bytes cmd buffer, only 25 pins are allowed,
	 * so total length won't more than a byte, cmd[7] is not neccessary to assign value */
	cmd[6] = cmd[10] + 3;
	cmd[3] = cmd[6] + 4;

	BTMTK_DBG_RAW(cmd, len, "%s: Send: ", __func__);

	down(&cif_dev->internal_cmd_sem);
	cif_dev->event_intercept = TRUE;
	p_inter_cmd->waiting_event = 0xE4;
	p_inter_cmd->pending_cmd_opcode = 0xFC6F;
	p_inter_cmd->wmt_opcode = WMT_OPCODE_ANT_EFEM;
	p_inter_cmd->result = WMT_EVT_INVALID;

	btmtk_main_send_cmd(g_sbdev, cmd, len, NULL, 0, 0, 0, BTMTK_TX_WAIT_VND_EVT);

	cif_dev->event_intercept  = FALSE;
	up(&cif_dev->internal_cmd_sem);
	BTMTK_INFO("[InternalCmd] %s done, result = %s", __func__, _internal_evt_result(p_inter_cmd->result));
	ret = p_inter_cmd->result;

done:
	if (p_img)
		vfree(p_img);
	return ret;
}

/* btmtk_intcmd_wmt_power_off
 *
 *    Send BT func off (cmd won't send during reset flow)
 *
 * Arguments:
 *    [IN] hdev     - hci_device as control structure during BT life cycle
 *
 * Return Value:
 *     0 if success, otherwise -EIO
 *
 */
int32_t btmtk_intcmd_wmt_power_off(struct hci_dev *hdev)
{
	//struct btmtk_dev *bdev = hci_get_drvdata(hdev);
	struct btmtk_btif_dev *cif_dev = (struct btmtk_btif_dev *)g_sbdev->cif_dev;
	int32_t ret = 0;

	/* Do not send CMD to connsys while connsys is resetting */
	if (cif_dev->bt_state == FUNC_OFF) {
		BTMTK_INFO("already at off state, skip command");
		return 0;
	} else if (cif_dev->bt_state != RESET_START)
		ret = _send_wmt_power_cmd(hdev, FALSE);

	BTMTK_DBG("%s: Done", __func__);
	return ret;
}

/* btmtk_intcmd_wmt_calibration
 *
 *    Check calibration cache and send query calibration data command if
 *    cache is not available
 *
 *
 * Arguments:
 *    [IN]  hdev     - hci_device as control structure during BT life cycle
 *
 * Return Value:
 *    N/A
 *
 */
int32_t btmtk_intcmd_wmt_calibration(struct hci_dev *hdev)
{
	//struct btmtk_dev *bdev = hci_get_drvdata(hdev);
	uint32_t cal_data_start_addr = 0;
	uint32_t cal_data_ready_addr = 0;
	uint16_t cal_data_len = 0;
	uint8_t *p_cal_data = NULL;
	int ret = 0;
	struct btmtk_btif_dev *cif_dev = (struct btmtk_btif_dev *)g_sbdev->cif_dev;

	if (cif_dev->cal_data.p_cache_buf) {
		BTMTK_DBG("calibration cache has data, no need to recal");
		return 0;
	}

	/*
	 * In case that we did not have a successful pre-calibration and no
	 * cached data to restore, the first BT func on will trigger calibration,
	 * but it is most likely to be failed due to Wi-Fi is off.
	 *
	 * If BT func on return success and we get here, anyway try to backup
	 * the calibration data, any failure during backup should be ignored.
	 */

	/* Get calibration data reference for backup */
	ret = _send_wmt_get_cal_data_cmd(hdev, &cal_data_start_addr,
				    &cal_data_ready_addr,
				    &cal_data_len);
	if (ret)
		BTMTK_ERR("Get cal data ref failed!");
	else {
		BTMTK_DBG(
			"Get cal data ref: saddr(0x%08x) raddr(0x%08x) len(%d)",
			cal_data_start_addr, cal_data_ready_addr, cal_data_len);

		/* Allocate a cache buffer to backup the calibration data in driver */
		if (cal_data_len) {
			p_cal_data = kmalloc(cal_data_len, GFP_KERNEL);
			if (p_cal_data) {
				bgfsys_cal_data_backup(cal_data_start_addr, p_cal_data, cal_data_len);
				cif_dev->cal_data.p_cache_buf = p_cal_data;
				cif_dev->cal_data.cache_len = cal_data_len;
				cif_dev->cal_data.start_addr = cal_data_start_addr;
				cif_dev->cal_data.ready_addr = cal_data_ready_addr;
			} else
				BTMTK_ERR("Failed to allocate cache buffer for backup!");
		} else
			BTMTK_ERR(
				"Abnormal cal data length! something error with F/W!");
	}
	return ret;
}


/* btmtk_intcmd_query_thermal
 *
 *    Send query thermal (a-die) command to FW
 *
 * Arguments:
 *    [IN] hdev     - hci_device as control structure during BT life cycle
 *
 * Return Value:
 *    Thermal value
 *
 */
int32_t btmtk_intcmd_query_thermal(void)
{
	uint8_t cmd[] = { 0x01, 0x91, 0xFD, 0x00 };
	struct btmtk_btif_dev *cif_dev = (struct btmtk_btif_dev *)g_sbdev->cif_dev;
	struct bt_internal_cmd *p_inter_cmd = &cif_dev->internal_cmd;
	struct wmt_pkt_param *evt = &p_inter_cmd->wmt_event_params;
	uint8_t *evtbuf = NULL;
	int32_t ret = 0;
	int32_t thermal = 0;
	/* To-Do, for event check */
	/* u8 event[] = { 0x04, 0x0E, 0x08, 0x01, 0x91, 0xFD, 0x00, 0x00, 0x00, 0x00, 0x00 }; */

	BTMTK_INFO("[InternalCmd] %s", __func__);

	down(&cif_dev->internal_cmd_sem);
	cif_dev->event_intercept = TRUE;

	p_inter_cmd->waiting_event = 0xE4;
	p_inter_cmd->pending_cmd_opcode = 0xFD91;
	p_inter_cmd->wmt_opcode = WMT_OPCODE_RF_CAL;
	p_inter_cmd->result = WMT_EVT_INVALID;

	ret = btmtk_main_send_cmd(g_sbdev, cmd, sizeof(cmd),
		NULL, 0, 0, 0, BTMTK_TX_WAIT_VND_EVT);

	if (ret <= 0) {
		BTMTK_ERR("Unable to send thermal cmd");
		return -1;
	}

	if (p_inter_cmd->result == WMT_EVT_SUCCESS) {
		evtbuf = &evt->u.evt_buf[6];
		thermal =  evtbuf[3] << 24 |
			   evtbuf[2] << 16 |
			   evtbuf[1] << 8 |
			   evtbuf[0];
	}

	cif_dev->event_intercept = FALSE;
	up(&cif_dev->internal_cmd_sem);
	BTMTK_INFO("[InternalCmd] %s done, result = %s, thermal = %d",
		__func__,  _internal_evt_result(p_inter_cmd->result), thermal);
	return thermal;
}

/* btmtk_intcmd_wmt_blank_status
 *
 *    Send blank status to FW
 *
 * Arguments:
 *    [IN] hdev     - hci_device as control structure during BT life cycle
 *
 * Return Value:
 *    N/A
 *
 */
int32_t btmtk_intcmd_wmt_blank_status(struct hci_dev *hdev, int32_t blank)
{
	struct btmtk_dev *bdev = hci_get_drvdata(hdev);
	struct btmtk_btif_dev *cif_dev = (struct btmtk_btif_dev *)g_sbdev->cif_dev;
	struct bt_internal_cmd *p_inter_cmd = &cif_dev->internal_cmd;
	uint8_t cmd[] =  { 0x01, 0x6F, 0xFC, 0x06, 0x01, 0xF0, 0x02, 0x00, 0x03, 0x00 };
	/* uint8_t evt[] = {0x04, 0xE4, 0x06, 0x02, 0xF0, 0x02, 0x00, 0x03, 0x00}; */

	BTMTK_INFO("[InternalCmd] %s", __func__);

	down(&cif_dev->internal_cmd_sem);
	cif_dev->event_intercept = TRUE;

	// wmt_blank_state: 0(screen off) / 1(screen on)
	cmd[9] = blank;
	cif_dev->event_intercept = TRUE;
	p_inter_cmd->waiting_event = 0xE4;
	p_inter_cmd->pending_cmd_opcode = 0xFC6F;
	p_inter_cmd->wmt_opcode = WMT_OPCODE_0XF0;
	p_inter_cmd->result = WMT_EVT_INVALID;

	btmtk_main_send_cmd(bdev, cmd, sizeof(cmd), NULL, 0, 0, 0, BTMTK_TX_WAIT_VND_EVT);

	cif_dev->event_intercept = FALSE;
	up(&cif_dev->internal_cmd_sem);
	BTMTK_INFO("[InternalCmd] %s done, result = %s", __func__, _internal_evt_result(p_inter_cmd->result));
	return 0;
}

/* btmtk_intcmd_wmt_utc_sync
 *
 *    Send time sync command to FW to synchronize time
 *
 * Arguments:
 *    [IN] hdev     - hci_device as control structure during BT life cycle
 *
 * Return Value:
 *    N/A
 *
 */
int32_t btmtk_intcmd_wmt_utc_sync(void)
{
	struct btmtk_btif_dev *cif_dev = (struct btmtk_btif_dev *)g_sbdev->cif_dev;
	struct bt_internal_cmd *p_inter_cmd = &cif_dev->internal_cmd;
	uint8_t cmd[] =  {0x01, 0x6F, 0xFC, 0x01, 0x0C,
			  0xF0, 0x09, 0x00, 0x02,
			  0x00, 0x00, 0x00, 0x00,	/* UTC time second unit */
			  0x00, 0x00, 0x00, 0x00};	/* UTC time microsecond unit*/
	uint32_t sec, usec;
	/* uint8_t evt[] = {0x04, 0xE4, 0x06, 0x02, 0xF0, 0x02, 0x00, 0x02, 0x00}; */

	BTMTK_INFO("[InternalCmd] %s", __func__);

	down(&cif_dev->internal_cmd_sem);
	cif_dev->event_intercept = TRUE;

	connsys_log_get_utc_time(&sec, &usec);
	memcpy(cmd + 9, &sec, sizeof(uint32_t));
	memcpy(cmd + 9 + sizeof(uint32_t), &usec, sizeof(uint32_t));

	p_inter_cmd->waiting_event = 0xE4;
	p_inter_cmd->pending_cmd_opcode = 0xFC6F;
	p_inter_cmd->wmt_opcode = WMT_OPCODE_0XF0;
	p_inter_cmd->result = WMT_EVT_INVALID;

	btmtk_main_send_cmd(g_sbdev, cmd, sizeof(cmd),
		NULL, 0, 0, 0, BTMTK_TX_WAIT_VND_EVT);

	cif_dev->event_intercept = FALSE;
	up(&cif_dev->internal_cmd_sem);
	BTMTK_INFO("[InternalCmd] %s done, result = %s", __func__, _internal_evt_result(p_inter_cmd->result));
	return 0;
}

/**
 * \brief: btmtk_intcmd_set_fw_log
 *
 * \details
 *   Send an MTK vendor specific command to configure Firmware Picus log:
 *   - enable, disable, or set it to a particular level.
 *
 *   The command format is conventional as below:
 *
 *      5D FC = TCI_MTK_DEBUG_VERSION_INFO
 *      04 = Command Length
 *      01 00 01 XX = Refer to table
 *      XX = Log Level
 *
 *   If configured to return a vendor specific event, the event format is as below:
 *
 *      FF = HCI_MTK_TRACE_EVENT
 *      08 = Event Length
 *      FE = TCI_HOST_DRIVER_LOG_EVENT
 *      5D FC = TCI_MTK_DEBUG_VERSION_INFO
 *      00 = Success
 *      01 00 01 XX = Refer to table
 *
 * +----------------------------+-----------------------+--------------+--------------------+
 * | Return Type                | Op Code               | Log Type     | Log Level          |
 * +----------------------------+-----------------------+--------------+--------------------+
 * | 1 byte                     | 1 byte                | 1 byte       | 1 byte             |
 * +----------------------------+-----------------------+--------------+--------------------+
 * | 0x00 Command Complete      | 0x00 Config Picus Log | 0x00 Via HCI | 0x00 Disable       |
 * | 0x01 Vendor Specific Event |                       | 0x01 Via EMI | 0x01 Low Power     |
 * | 0x02 No Event              |                       |              | 0x02 SQC (Default) |
 * |                            |                       |              | 0x03 Debug (Full)  |
 * +----------------------------+-----------------------+--------------+--------------------+
 *
 * \param
 *  @flag: F/W log level
 *
 * \return
 *  0: success; nagtive: fail
 */
int32_t btmtk_intcmd_set_fw_log(uint8_t flag)
{
	int32_t ret;
	uint8_t HCI_CMD_FW_LOG_DEBUG[] = {0x01, 0x5d, 0xfc, 0x04, 0x02, 0x00, 0x01, 0xff};
	struct btmtk_btif_dev *cif_dev = (struct btmtk_btif_dev *)g_sbdev->cif_dev;

	if (cif_dev->rst_level != RESET_LEVEL_NONE) {
		BTMTK_WARN("Resetting, skip %s [%d]", __func__, flag);
		return -1;
	}

	HCI_CMD_FW_LOG_DEBUG[7] = flag;
	BTMTK_DBG("hci_cmd: %02x %02x %02x %02x %02x %02x %02x %02x\n",
			HCI_CMD_FW_LOG_DEBUG[0], HCI_CMD_FW_LOG_DEBUG[1],
			HCI_CMD_FW_LOG_DEBUG[2], HCI_CMD_FW_LOG_DEBUG[3],
			HCI_CMD_FW_LOG_DEBUG[4], HCI_CMD_FW_LOG_DEBUG[5],
			HCI_CMD_FW_LOG_DEBUG[6], HCI_CMD_FW_LOG_DEBUG[7]);

	down(&cif_dev->internal_cmd_sem);
	ret = btmtk_main_send_cmd(g_sbdev, HCI_CMD_FW_LOG_DEBUG,
				  sizeof(HCI_CMD_FW_LOG_DEBUG),
				  NULL, 0, 0, 0, BTMTK_TX_SKIP_VENDOR_EVT);
	up(&cif_dev->internal_cmd_sem);

	if (ret < 0) {
		BTMTK_ERR("Send F/W log cmd failed!\n");
		return ret;
	}
	return 0;
}

/* btmtk_intcmd_send_connfem_cmd
 *
 *    --
 *
 * Arguments:
 *    void
 *
 * Return Value:
 *    N/A
 *
 */
int32_t btmtk_intcmd_send_connfem_cmd(void)
{
	struct connfem_epaelna_fem_info fem_info;
	struct connfem_epaelna_pin_info pin_info;
	struct connfem_epaelna_flags_bt bt_flag;
	struct btmtk_btif_dev *cif_dev = (struct btmtk_btif_dev *)g_sbdev->cif_dev;
	struct bt_internal_cmd *p_inter_cmd = &cif_dev->internal_cmd;

	uint8_t *cmd = NULL;
	uint8_t cmd_header[] = {0x01, 0x6F, 0xFC, 0x00, 0x01, 0x55, 0x00, 0x00,
				0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint32_t cmd_len = 0, i = 0, offset = 0;
	const uint32_t pin_struct_size = sizeof(struct connfem_epaelna_pin);

	BTMTK_INFO("[InternalCmd] %s", __func__);

	/* Get data from connfem_api */
	connfem_epaelna_get_fem_info(&fem_info);
	connfem_epaelna_get_pin_info(&pin_info);
	connfem_epaelna_get_flags(CONNFEM_SUBSYS_BT, &bt_flag);

	if (fem_info.part[CONNFEM_PORT_BT].vid == 0 &&
	    fem_info.part[CONNFEM_PORT_BT].pid == 0) {
		BTMTK_INFO("CONNFEM BTvid/pid == 0, ignore");
		return 0;
	}

        /*
         * command and event example
         *  0  1  2      3  4  5  6  7  8  9  A  B  C  D
         * 01 6F FC length 01 55 LL LL 01 XX XX XX XX NN YYYYYY ..  YYYYYY AA BB BB
         * lengthL : LL + 4
         * LLLL : length = 1 + 4 + 1 + 3*num + 3 (only 1 byte length valid,
         *					  value 251 should be maxium)
         * XXXXXXXX : 4 byte,  efem ID
         * NN : 1 byte, total efem number
         * YYYYYY: 3 byte * number, u1AntSelNo,    u1FemPin,     u1Polarity;
         * AA : bt flag
         * BBBB : 2.4G part = VID + PID
	 *
         * RX: 04 E4 06 02 55 02 00 01 SS (SS : status)
        */
	cmd_len = sizeof(cmd_header) + pin_info.count * pin_struct_size + 3;
	cmd = vmalloc(cmd_len);
	if (!cmd) {
		BTMTK_ERR("unable to allocate confem command");
		return -1;
	}

	memcpy(cmd, cmd_header, sizeof(cmd_header));

	/* assign WMT over HCI command length */
	cmd[3] = cmd_len - 4;

	/* assign payload length */
	cmd[6] = cmd_len - 8;

	/* assign femid */
	memcpy(&cmd[9], &fem_info.id, sizeof(fem_info.id));
	offset = sizeof(cmd_header);

	/* assign pin count */
	cmd[offset-1] = pin_info.count;

	/* assign pin mapping info */
	for (i = 0; i < pin_info.count; i++) {
		memcpy(&cmd[offset], &pin_info.pin[i], pin_struct_size);
		offset += pin_struct_size;
	}

	/* config priority: epa_elna > elna > epa > bypass */
	cmd[offset++] = (bt_flag.epa_elna) ? 3 :
			(bt_flag.epa) ? 2:
			(bt_flag.elna) ? 1: 0;

	cmd[offset++] = fem_info.part[CONNFEM_PORT_BT].vid;
	cmd[offset++] = fem_info.part[CONNFEM_PORT_BT].pid;

	BTMTK_INFO_RAW(cmd, offset, "%s: Send: ", __func__);

	down(&cif_dev->internal_cmd_sem);
	cif_dev->event_intercept = TRUE;
	p_inter_cmd->waiting_event = 0xE4;
	p_inter_cmd->pending_cmd_opcode = 0xFC6F;
	p_inter_cmd->wmt_opcode = WMT_OPCODE_ANT_EFEM;
	p_inter_cmd->result = WMT_EVT_INVALID;

	btmtk_main_send_cmd(g_sbdev, cmd, cmd_len, NULL, 0, 0, 0, BTMTK_TX_WAIT_VND_EVT);

	cif_dev->event_intercept  = FALSE;
	vfree(cmd);
	up(&cif_dev->internal_cmd_sem);
	BTMTK_INFO("[InternalCmd] %s done, result = %s", __func__, _internal_evt_result(p_inter_cmd->result));
	return p_inter_cmd->result;
}

/* btmtk_set_power_on
 *
 *    BT On flow including all steps exclude BT func on commaond
 *
 *
 * Arguments:
 *    [IN]  hdev     - hci_device as control structure during BT life cycle
 *    [IN]  for_precal - power on in pre-calibation flow, and don't
 *			 call coninfra_pwr_on if the value is TRUE
 *
 * Return Value:
 *     0 if success, otherwise error code
 *
 */
int32_t btmtk_set_power_on(struct hci_dev *hdev, u_int8_t for_precal)
{
	int ret;
	int sch_ret = -1;
	struct sched_param sch_param;
	struct btmtk_dev *bdev = hci_get_drvdata(hdev);
	struct btmtk_btif_dev *cif_dev = (struct btmtk_btif_dev *)g_sbdev->cif_dev;

	if (g_bt_trace_pt)
		bt_dbg_tp_evt(TP_ACT_PWR_ON, 0, 0, NULL);
	/*
	 *  1. ConnInfra hardware power on (Must be the first step)
	 *
	 *  We will block in this step if conninfra driver find that the pre-calibration
	 *  has not been performed, conninfra driver should guarantee to trigger
	 *  pre-calibration procedure first:
	 *    - call BT/Wi-Fi registered pwr_on_cb and do_cal_cb
	 *  then return from this API after 2.4G calibration done.
	 */
	bt_pwrctrl_pre_on();
	if (!for_precal)
	{
		if (conninfra_pwr_on(CONNDRV_TYPE_BT)) {
			BTMTK_ERR("ConnInfra power on failed!");
			ret = -EIO;
			goto conninfra_error;
		}
	}

	BTMTK_DBG("%s: wait halt_sem...", __func__);
	down(&cif_dev->halt_sem);
	BTMTK_DBG("%s: wait halt_sem finish...", __func__);

	/* record current bt state for restoring orginal state after pre-cal */
	cif_dev->bt_precal_state = cif_dev->bt_state;

	/* state check before power on */
	if (cif_dev->bt_state == FUNC_ON) {
		BTMTK_INFO("BT is already on, skip");
		up(&cif_dev->halt_sem);
		return 0;
	} else if (cif_dev->bt_state != FUNC_OFF) {
		BTMTK_ERR("BT is not at off state, uanble to on [%d]!",
							cif_dev->bt_state);
		up(&cif_dev->halt_sem);
		return -EIO;
	}
	BTMTK_INFO("%s: CONNAC20_CHIPID[%d], bdev[0x%p], bt_state[%d]!",
			__func__, CONNAC20_CHIPID, bdev, cif_dev->bt_state);

	cif_dev->bt_state = TURNING_ON;

	/* 2. MD coex ccif on */
	bgfsys_ccif_on();

	/* 3. clear coredump */
#if SUPPORT_COREDUMP
	if (!cif_dev->coredump_handle)
		BTMTK_ERR("Coredump handle is NULL\n");
	else
		connsys_coredump_clean(cif_dev->coredump_handle);
#endif

	/* 4. BGFSYS hardware and MCU bring up, BT RAM code ready */
	ret = bt_hw_and_mcu_on();
	if (ret) {
		BTMTK_ERR("BT hardware and MCU on failed!");
		goto mcu_error;
	}


	/* 5. initialize bdev variables */
	if (bdev->evt_skb)
		kfree_skb(bdev->evt_skb);
	bdev->evt_skb = NULL;

	cif_dev->event_intercept = FALSE;
	cif_dev->internal_cmd.waiting_event = 0x00;
	cif_dev->internal_cmd.pending_cmd_opcode = 0x0000;
	cif_dev->internal_cmd.wmt_opcode = 0x00;
	cif_dev->internal_cmd.result = WMT_EVT_INVALID;

	cif_dev->rx_ind = FALSE;
	cif_dev->psm.state = PSM_ST_SLEEP;
	cif_dev->psm.sleep_flag = FALSE;
	cif_dev->psm.wakeup_flag = FALSE;
	cif_dev->psm.result = 0;
	cif_dev->psm.force_on = FALSE;

#if (USE_DEVICE_NODE == 1)
	btmtk_rx_flush();
#endif

	cif_dev->internal_trx.cb = NULL;
	spin_lock_init(&cif_dev->internal_trx.lock);

#if SUPPORT_BT_THREAD
	/* 6. Create TX thread with PS state machine */
	skb_queue_purge(&cif_dev->tx_queue);
	init_waitqueue_head(&cif_dev->tx_waitq);
	cif_dev->tx_thread = kthread_create(btmtk_tx_thread, bdev, "bt_tx_ps");
	if (IS_ERR(cif_dev->tx_thread)) {
		BTMTK_DBG("btmtk_tx_thread failed to start!");
		ret = PTR_ERR(cif_dev->tx_thread);
		goto thread_create_error;
	}

	if(g_bt_dbg_st.rt_thd_enable) {
		sch_param.sched_priority = MAX_RT_PRIO - 20;
		sch_ret = sched_setscheduler(cif_dev->tx_thread, SCHED_FIFO, &sch_param);
		BTMTK_INFO("sch_ret = %d", sch_ret);
		if (sch_ret != 0)
			BTMTK_INFO("set RT thread failed");
		else
			BTMTK_INFO("set RT thread succeed");
	}
	wake_up_process(cif_dev->tx_thread);
#endif

	/*
	 * 7. Calibration data restore
	 * If we have a cache of calibration data, restore it to Firmware sysram,
	 * otherwise, BT func on will trigger calibration again.
	 */
	if (cif_dev->cal_data.p_cache_buf)
		bgfsys_cal_data_restore(cif_dev->cal_data.start_addr,
					cif_dev->cal_data.ready_addr,
					cif_dev->cal_data.p_cache_buf,
					cif_dev->cal_data.cache_len);

#if SUPPORT_COREDUMP
	if (!cif_dev->coredump_handle)
		BTMTK_ERR("Coredump handle is NULL\n");
	else
		connsys_coredump_setup_dump_region(cif_dev->coredump_handle);
#endif

	bt_enable_irq(BGF2AP_SW_IRQ);
	if (BT_SSPM_TIMER)
		bt_enable_irq(BT_CONN2AP_SW_IRQ);

	/* 8. init cmd queue and workqueue */
#if (DRIVER_CMD_CHECK == 1)
	cmd_list_initialize();
	cmd_workqueue_init();
	cif_dev->cmd_timeout_check = FALSE;
#endif
	dump_queue_initialize();

	/* 9. init reset variable */
	cif_dev->rst_level = RESET_LEVEL_NONE;
	cif_dev->rst_count = 0;
	cif_dev->rst_flag = FALSE;

	/* 9.5 send connfem command before BT on */
	ret = btmtk_intcmd_send_connfem_cmd();
	if (ret) {
		BTMTK_ERR("btmtk_send_confem_cmd fail");
		//goto wmt_power_on_error;
	}

	/* 9.6 send antenna command before BT on */
	ret = btmtk_intcmd_wmt_send_antenna_cmd(hdev);
	if (ret) {
		BTMTK_ERR("btmtk_send_wmt_antenna_cmd fail");
		//goto wmt_power_on_error;
	}

	/* 10. send WMT cmd to set BT on */
	ret = btmtk_intcmd_wmt_power_on(hdev);
	up(&cif_dev->halt_sem);

	/* case of fw trigger reset during power on cmd,
	   directly return since reset thread will perform turn off */
	if (cif_dev->rst_flag != FALSE) {
		BTMTK_INFO("%s: wait rst_flag", __func__);
		wait_event_interruptible(cif_dev->rst_onoff_waitq, cif_dev->rst_flag == FALSE);
		BTMTK_INFO("%s: wait rst_flag done", __func__);
		return -EIO;
	}
	if (cif_dev->bt_state == FUNC_OFF || cif_dev->bt_state == RESET_START)
		return -EIO;
	/* case of whole chip reset */
	if (ret == RET_PWRON_WHOLE_CHIP_RESET)
		return -EIO;
	else if (ret) {
		BTMTK_ERR("btmtk_intcmd_wmt_power_on fail");
		goto wmt_power_on_error;
	}

	/* Set utc_sync & blank_state cmd */
	btmtk_intcmd_wmt_utc_sync();
	btmtk_intcmd_wmt_blank_status(hdev, cif_dev->blank_state);

	/* Set bt to sleep mode */
	btmtk_set_sleep(hdev, TRUE);

	return 0;

wmt_power_on_error:
	wake_up_interruptible(&cif_dev->tx_waitq);
	kthread_stop(cif_dev->tx_thread);
	cif_dev->tx_thread = NULL;
#if (DRIVER_CMD_CHECK == 1)
	cmd_workqueue_exit();
	cmd_list_destory();
	cif_dev->cmd_timeout_check = FALSE;
#endif

thread_create_error:
	bt_hw_and_mcu_off();

mcu_error:
	bgfsys_ccif_off();
	if (!for_precal) {
		conninfra_pwr_off(CONNDRV_TYPE_BT);
		bt_pwrctrl_post_off();
	}
	up(&cif_dev->halt_sem);

conninfra_error:
	cif_dev->bt_state = FUNC_OFF;
	return ret;
}

/* btmtk_set_power_off
 *
 *    BT off flow
 *
 *
 * Arguments:
 *    [IN]  hdev     - hci_device as control structure during BT life cycle
 *    [IN]  for_precal - power on in pre-calibation flow, and don't
 *			 call coninfra_pwr_off if the value is TRUE
 *
 * Return Value:
 *    N/A
 *
 */
int32_t btmtk_set_power_off(struct hci_dev *hdev, u_int8_t for_precal)
{
	//struct btmtk_dev *bdev = hci_get_drvdata(hdev);
	struct btmtk_btif_dev *cif_dev = (struct btmtk_btif_dev *)g_sbdev->cif_dev;

	BTMTK_INFO("%s", __func__);
	if (g_bt_trace_pt)
		bt_dbg_tp_evt(TP_ACT_PWR_OFF, 0, 0, NULL);

	down(&cif_dev->halt_sem);

	if (cif_dev->bt_state == FUNC_OFF) {
		BTMTK_WARN("Alread in off state, skip");
		up(&cif_dev->halt_sem);
		return 0;
	/* Do not power off while chip reset thread is doing coredump */
	} else if ((cif_dev->bt_state == FUNC_ON || cif_dev->bt_state == TURNING_ON)
		   && cif_dev->rst_level != RESET_LEVEL_NONE) {
		BTMTK_WARN("BT is coredump, ignore stack power off");
		up(&cif_dev->halt_sem);
		return 0;
	}

	/* 1. Send WMT cmd to set BT off */
	btmtk_intcmd_wmt_power_off(hdev);

	if (cif_dev->rst_flag != FALSE) {
		up(&cif_dev->halt_sem);
		BTMTK_INFO("%s: wait rst_flag", __func__);
		wait_event_interruptible(cif_dev->rst_onoff_waitq, cif_dev->rst_flag == FALSE);
		BTMTK_INFO("%s: wait rst_flag done", __func__);
		return 0; // directly return since reset thread will perform turn off
	}

	/* flush fw log in EMI */
	connsys_log_irq_handler(CONN_DEBUG_TYPE_BT);

	/* 2. Stop TX thread */
#if SUPPORT_BT_THREAD
	if (cif_dev->tx_thread) {
		wake_up_interruptible(&cif_dev->tx_waitq);
		kthread_stop(cif_dev->tx_thread);
	}
	cif_dev->tx_thread = NULL;
	skb_queue_purge(&cif_dev->tx_queue);
#endif

	/* 3. BGFSYS hardware and MCU shut down */
	bt_hw_and_mcu_off();
	BTMTK_DBG("BT hardware and MCU off!");

	/* 4. MD Coex ccif */
	bgfsys_ccif_off();

	/* 5. close cmd queue and workqueue */
#if (DRIVER_CMD_CHECK == 1)
	cmd_workqueue_exit();
	cmd_list_destory();
	cif_dev->cmd_timeout_check = FALSE;
#endif

	cif_dev->bt_state = FUNC_OFF;

	up(&cif_dev->halt_sem);
	/* 6. ConnInfra hardware power off */
	if (!for_precal)
		conninfra_pwr_off(CONNDRV_TYPE_BT);
	bt_pwrctrl_post_off();

	/* Delay sending HW error to stack if it's whole chip reset,
	 * we have to wait conninfra power on then send message or
	 * stack triggers BT on will fail bcz conninfra is not power on
	 */
	if (cif_dev->rst_level != RESET_LEVEL_0)
		bt_notify_state();

	BTMTK_INFO("ConnInfra power off!");

	return 0;
}


/* btmtk_set_sleep
 *
 *    Request driver to enter sleep mode (FW own), and restore the state from
 *    btmtk_set_wakeup (this api makes driver stay at wakeup all the time)
 *
 *
 * Arguments:
 *    [IN]  hdev     - hci_device as control structure during BT life cycle
 *
 * Return Value:
 *    N/A
 *
 */
int32_t btmtk_set_sleep(struct hci_dev *hdev, uint8_t need_wait)
{
	//struct btmtk_dev *bdev = hci_get_drvdata(hdev);
	struct btmtk_btif_dev *cif_dev = (struct btmtk_btif_dev *)g_sbdev->cif_dev;

#if SUPPORT_BT_THREAD
	cif_dev->psm.sleep_flag = TRUE;
	wake_up_interruptible(&cif_dev->tx_waitq);

	if (!need_wait)
		return 0;

	if (!wait_for_completion_timeout(&cif_dev->psm.comp, msecs_to_jiffies(1000))) {
		BTMTK_ERR("[PSM]Timeout for BGFSYS to enter sleep!");
		cif_dev->psm.sleep_flag = FALSE;
		return -1;
	}

	BTMTK_DBG("[PSM]sleep return %s, sleep(%d), wakeup(%d)",
		      (cif_dev->psm.result == 0) ? "success" : "failure",
		      cif_dev->psm.sleep_flag, cif_dev->psm.wakeup_flag);

	return 0;
#else
	BTMTK_ERR("%s: [PSM] Doesn't support non-thread mode !", __func__);
	return -1;
#endif


}

/* btmtk_set_wakeup
 *
 *    Force BT driver/FW awake all the time
 *
 *
 * Arguments:
 *    [IN]  hdev     - hci_device as control structure during BT life cycle
 *
 * Return Value:
 *    N/A
 *
 */
int32_t btmtk_set_wakeup(struct hci_dev *hdev, uint8_t need_wait)
{
	struct btmtk_btif_dev *cif_dev = (struct btmtk_btif_dev *)g_sbdev->cif_dev;

#if SUPPORT_BT_THREAD
	cif_dev->psm.wakeup_flag = TRUE;
	wake_up_interruptible(&cif_dev->tx_waitq);

	if (!need_wait)
		return 0;

	if (!wait_for_completion_timeout(&cif_dev->psm.comp, msecs_to_jiffies(1000))) {
		BTMTK_ERR("[PSM] Timeout for BGFSYS to enter wakeup!");
		cif_dev->psm.wakeup_flag = FALSE;
		return -1;
	}

	BTMTK_INFO("[PSM] wakeup return %s, sleep(%d), wakeup(%d)",
			(cif_dev->psm.result == 0) ? "success" : "failure",
			cif_dev->psm.sleep_flag, cif_dev->psm.wakeup_flag);
	return 0;
#else
	BTMTK_ERR("%s: [PSM] Doesn't support non-thread mode !", __func__);
	return -1;
#endif
}
