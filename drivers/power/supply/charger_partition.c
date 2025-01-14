#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/workqueue.h>
#include <linux/err.h>
#include <linux/blkdev.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/printk.h>
#include <linux/spinlock.h>
#include <linux/reboot.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/efi.h>
#include <linux/rcupdate.h>
#include <linux/of.h>
#include <scsi/scsi_device.h>
#include <scsi/scsi_cmnd.h>
#include <scsi/scsi_host.h>
#include <linux/completion.h>
#include "../../scsi/sd.h"
#include "charger_partition.h"
//#include "hwid.h"

// extern struct completion ufs_xiaomi_comp;

struct ChargerPartition {
        struct device *dev;
	struct scsi_device *sdev;
        struct delayed_work charger_partition_work;

	int part_info_part_number;
	char *part_info_part_name;

	bool is_charger_partition_rdy;
};
static struct ChargerPartition *charger_partition;
static void* rw_buf;

typedef struct part {
	sector_t part_start;
	sector_t part_size;
} partinfo;
static partinfo part_info = { 0 };

static int charger_scsi_read_partition(struct scsi_device *sdev, void *buf, uint32_t lba, uint32_t blocks)
{
	uint8_t cdb[16];
	int ret = 0;
	struct scsi_sense_hdr sshdr = {};
	const struct scsi_exec_args exec_args = {
		.sshdr = &sshdr,
	};
	unsigned long flags = 0;

	spin_lock_irqsave(sdev->host->host_lock, flags);
	ret = scsi_device_get(sdev);
	if (!ret && !scsi_device_online(sdev)) {
		ret = -ENODEV;
		scsi_device_put(sdev);
		pr_err("[ChgPartition] get device fail\n");
	}
	spin_unlock_irqrestore(sdev->host->host_lock, flags);
	if (ret) {
        pr_err("[ChgPartition] failed to get scsi device\n");
		return ret;
    }
	sdev->host->eh_noresume = 1;

	// Fill in the CDB with SCSI command structure
	memset (cdb, 0, sizeof(cdb));
	cdb[0] = READ_10;				// Command
	cdb[1] = 0;
	cdb[2] = (lba >> 24) & 0xff;	// LBA
 	cdb[3] = (lba >> 16) & 0xff;
 	cdb[4] = (lba >> 8) & 0xff;
	cdb[5] = (lba) & 0xff;
	cdb[6] = 0;						// Group Number
	cdb[7] = (blocks >> 8) & 0xff;	// Transfer Len
	cdb[8] = (blocks) & 0xff;
	cdb[9] = 0;						// Control
	ret = scsi_execute_cmd(sdev, cdb, REQ_OP_DRV_IN, buf, (blocks * PART_BLOCK_SIZE), msecs_to_jiffies(15000), 3, &exec_args);
	if (ret) {
		pr_err("[ChgPartition] read error %d\n", ret);
	}
	scsi_device_put(sdev);
	sdev->host->eh_noresume = 0;
	return ret;
}

static int charger_scsi_write_partition(struct scsi_device *sdev, void *buf, uint32_t lba, uint32_t blocks)
{
	uint8_t cdb[16];
	int ret = 0;
	struct scsi_sense_hdr sshdr = {};
	const struct scsi_exec_args exec_args = {
		.sshdr = &sshdr,
	};
	unsigned long flags = 0;

	spin_lock_irqsave(sdev->host->host_lock, flags);
	ret = scsi_device_get(sdev);
	if (!ret && !scsi_device_online(sdev)) {
		ret = -ENODEV;
		scsi_device_put(sdev);
		pr_err("[ChgPartition] get device fail\n");
	}
	spin_unlock_irqrestore(sdev->host->host_lock, flags);

	if (ret) {
        pr_err("[ChgPartition] get scsi fail\n");
		return ret;
    }
	sdev->host->eh_noresume = 1;

	// Fill in the CDB with SCSI command structure
	memset (cdb, 0, sizeof(cdb));
	cdb[0] = WRITE_10;				// Command
	cdb[1] = 0;
	cdb[2] = (lba >> 24) & 0xff;	// LBA
	cdb[3] = (lba >> 16) & 0xff;
	cdb[4] = (lba >> 8) & 0xff;
	cdb[5] = (lba) & 0xff;
	cdb[6] = 0;						// Group Number
	cdb[7] = (blocks >> 8) & 0xff;	// Transfer Len
	cdb[8] = (blocks) & 0xff;
	cdb[9] = 0;					// Control
	ret = scsi_execute_cmd(sdev, cdb, REQ_OP_DRV_OUT, buf, (blocks * PART_BLOCK_SIZE), msecs_to_jiffies(15000), 3, &exec_args);
	if (ret) {
		pr_err("[ChgPartition] write error %d\n", ret);
	}
	scsi_device_put(sdev);
	sdev->host->eh_noresume = 0;
	return ret;
}

int charger_partition_alloc(u8 charger_partition_host_type, u8 charger_partition_info_type, uint32_t size)
{
	int ret = 0;

	if (!charger_partition->is_charger_partition_rdy) {
		pr_err("[ChgPartition] charger partition not rdy, can't do rw!");
		return -1;
	}
	if (charger_partition_host_type >= CHARGER_PARTITION_HOST_LAST) {
		pr_err("[ChgPartition] charger_partition_host_type not support!");
		return -1;
	}
	if (charger_partition_info_type >= CHARGER_PARTITION_INFO_LAST) {
		pr_err("[ChgPartition] charger_partition_info_type not support!");
		return -1;
	}
	if (size >= CHARGER_PARTITION_RWSIZE) {
		pr_err("[ChgPartition] read %u size not support!", size);
		return -1;
	}

	rw_buf = kzalloc(CHARGER_PARTITION_RWSIZE, GFP_KERNEL);
	if (!rw_buf) {
		pr_err("[ChgPartition] malloc buf error!");
		return -1;
	}

	/* check if avaliable */
	ret = charger_scsi_read_partition(charger_partition->sdev, rw_buf, (part_info.part_start + CHARGER_PARTITION_HEADER), CHARGER_PARTITION_RWSIZE / PART_BLOCK_SIZE);
	if (ret) {
		pr_err("[ChgPartition] charger read error %d\n", ret);
		kfree(rw_buf);
		return -1;
	}
	if (0 == ((charger_partition_header *)rw_buf)->avaliable) {
		pr_err("[ChgPartition] not avaliable, can't do rw now!\n");
		kfree(rw_buf);
		return -1;
	}
	((charger_partition_header *)rw_buf)->avaliable = 0;
	ret = charger_scsi_write_partition(charger_partition->sdev, (void *)rw_buf, (part_info.part_start + CHARGER_PARTITION_HEADER), CHARGER_PARTITION_RWSIZE / PART_BLOCK_SIZE);
	if (ret) {
		pr_err("[ChgPartition] charger write error %d\n", ret);
		kfree(rw_buf);
		return -1;
	}

	return ret;
}
EXPORT_SYMBOL(charger_partition_alloc);

int charger_partition_dealloc(u8 charger_partition_host_type, u8 charger_partition_info_type, uint32_t size)
{
	int ret = 0;

	if (!charger_partition->is_charger_partition_rdy) {
		pr_err("[ChgPartition] charger_partition not rdy, can't do rw!");
		return -1;
	}
	if (charger_partition_host_type >= CHARGER_PARTITION_HOST_LAST) {
		pr_err("[ChgPartition] charger_partition_host_type not support!");
		return -1;
	}
	if (charger_partition_info_type >= CHARGER_PARTITION_INFO_LAST) {
		pr_err("[ChgPartition] charger_partition_info_type not support!");
		return -1;
	}
	if (size >= CHARGER_PARTITION_RWSIZE) {
		pr_err("[ChgPartition] read %u size not support!", size);
		return -1;
	}

	memset(rw_buf, 0, CHARGER_PARTITION_RWSIZE);
	ret = charger_scsi_read_partition(charger_partition->sdev, rw_buf, (part_info.part_start + CHARGER_PARTITION_HEADER), CHARGER_PARTITION_RWSIZE / PART_BLOCK_SIZE);
	if (ret) {
		pr_err("[ChgPartition] charger read error %d\n", ret);
		kfree(rw_buf);
		return -1;
	}
	((charger_partition_header *)rw_buf)->avaliable = 1;
	ret = charger_scsi_write_partition(charger_partition->sdev, (void *)rw_buf, (part_info.part_start + CHARGER_PARTITION_HEADER), CHARGER_PARTITION_RWSIZE / PART_BLOCK_SIZE);
	if (ret) {
		pr_err("[ChgPartition] charger write error %d\n", ret);
		kfree(rw_buf);
		return -1;
	}

	kfree(rw_buf);
	return ret;
}
EXPORT_SYMBOL(charger_partition_dealloc);

void *charger_partition_read(u8 charger_partition_host_type, u8 charger_partition_info_type, uint32_t size)
{
	int ret = 0;

	if (!charger_partition->is_charger_partition_rdy) {
		pr_err("[ChgPartition] charger partition not rdy, can't do read!");
		return NULL;
	}
	if (!rw_buf) {
		pr_err("[ChgPartition] rw_buf null, please alloc first!");
		return NULL;
	}
	if (charger_partition_host_type >= CHARGER_PARTITION_HOST_LAST) {
		pr_err("[ChgPartition] charger_partition_host_type not support!");
		return NULL;
	}
	if (charger_partition_info_type >= CHARGER_PARTITION_INFO_LAST) {
		pr_err("[ChgPartition] charger_partition_info_type not support!");
		return NULL;
	}
	if (size >= CHARGER_PARTITION_RWSIZE) {
		pr_err("[ChgPartition] read %u size not support!", size);
		return NULL;
	}

	memset(rw_buf, 0, CHARGER_PARTITION_RWSIZE);
	ret = charger_scsi_read_partition(charger_partition->sdev, rw_buf, (part_info.part_start + charger_partition_info_type), CHARGER_PARTITION_RWSIZE / PART_BLOCK_SIZE);
	if (ret) {
		pr_err("[ChgPartition] charger read error %d\n", ret);
		return NULL;
	}
	return rw_buf;
}
EXPORT_SYMBOL(charger_partition_read);

int charger_partition_write(u8 charger_partition_host_type, u8 charger_partition_info_type, void *buf, uint32_t size)
{
	int ret = 0;

	if (!charger_partition->is_charger_partition_rdy) {
		pr_err("[ChgPartition] charger partition not rdy, can't do read!");
		return -1;
	}
	if (!rw_buf) {
		pr_err("[ChgPartition] rw_buf null, please alloc first!");
		return -1;
	}
	if (charger_partition_host_type >= CHARGER_PARTITION_HOST_LAST) {
		pr_err("[ChgPartition] charger_partition_host_type not support!");
		return -1;
	}
	if (charger_partition_info_type >= CHARGER_PARTITION_INFO_LAST) {
		pr_err("[ChgPartition] charger_partition_info_type not support!");
		return -1;
	}
	if (size >= CHARGER_PARTITION_RWSIZE) {
		pr_err("[ChgPartition] read %u size not support!", size);
		return -1;
	}

	memset(rw_buf, 0, CHARGER_PARTITION_RWSIZE);
	ret = charger_scsi_read_partition(charger_partition->sdev, rw_buf, (part_info.part_start + charger_partition_info_type), CHARGER_PARTITION_RWSIZE / PART_BLOCK_SIZE);
	if (ret) {
		pr_err("[ChgPartition] charger read error %d\n", ret);
		return -1;
	}
	memcpy(rw_buf, buf, size);

	ret = charger_scsi_write_partition(charger_partition->sdev, rw_buf, (part_info.part_start + charger_partition_info_type), CHARGER_PARTITION_RWSIZE / PART_BLOCK_SIZE);
	if (ret) {
		pr_err("[ChgPartition] charger write error %d\n", ret);
		return -1;
	}

	return ret;
}
EXPORT_SYMBOL(charger_partition_write);

int charger_partition_get_prop(enum charger_partition_prop_list prop, u32 *val)
{
	int ret = 0;
	charger_partition_info_1 *info_1 = NULL;
	charger_partition_info_2 *info_2 = NULL;

	switch (prop) {
	case CHARGER_PARTITION_PROP_TEST:
		ret = charger_partition_alloc(CHARGER_PARTITION_HOST_KERNEL, CHARGER_PARTITION_INFO_1, sizeof(charger_partition_info_1));
		if(ret < 0) {
			pr_err("[ChgPartition] failed to alloc\n");
			return -1;
		}

		info_1 = (charger_partition_info_1 *)charger_partition_read(CHARGER_PARTITION_HOST_KERNEL, CHARGER_PARTITION_INFO_1, sizeof(charger_partition_info_1));
		if(!info_1) {
			pr_err("[ChgPartition] failed to read\n");
			return -1;
		}
		*val = info_1->test;
		pr_info("[ChgPartition] ret: %d, info_1->test: %u, val: %u\n", ret, info_1->test, *val);

		ret = charger_partition_dealloc(CHARGER_PARTITION_HOST_KERNEL, CHARGER_PARTITION_INFO_1, sizeof(charger_partition_info_1));
		if(ret < 0) {
			pr_err("[ChgPartition] failed to dealloc\n");
			return -1;
		}
		break;
	case CHARGER_PARTITION_PROP_POWER_OFF_MODE:
		ret = charger_partition_alloc(CHARGER_PARTITION_HOST_KERNEL, CHARGER_PARTITION_INFO_1, sizeof(charger_partition_info_1));
		if(ret < 0) {
			pr_err("[ChgPartition] failed to alloc\n");
			return -1;
		}

		info_1 = (charger_partition_info_1 *)charger_partition_read(CHARGER_PARTITION_HOST_KERNEL, CHARGER_PARTITION_INFO_1, sizeof(charger_partition_info_1));
		if(!info_1) {
			pr_err("[ChgPartition] failed to read\n");
			return -1;
		}
		*val = info_1->power_off_mode;
		pr_info("[ChgPartition] ret: %d, info_1->power_off_mode: %u, val: %u\n", ret, info_1->power_off_mode, *val);

		ret = charger_partition_dealloc(CHARGER_PARTITION_HOST_KERNEL, CHARGER_PARTITION_INFO_1, sizeof(charger_partition_info_1));
		if(ret < 0) {
			pr_err("[ChgPartition] failed to dealloc\n");
			return -1;
		}
		break;
	case CHARGER_PARTITION_PROP_EU_MODE:
		ret = charger_partition_alloc(CHARGER_PARTITION_HOST_KERNEL, CHARGER_PARTITION_INFO_2, sizeof(charger_partition_info_2));
		if(ret < 0) {
			pr_err("[ChgPartition] failed to alloc\n");
			return -1;
		}

		info_2 = (charger_partition_info_2 *)charger_partition_read(CHARGER_PARTITION_HOST_KERNEL, CHARGER_PARTITION_INFO_2, sizeof(charger_partition_info_2));
		if(!info_2) {
			pr_err("[ChgPartition] failed to read\n");
			return -1;
		}
		*val = info_2->eu_mode;
		pr_info("[ChgPartition] ret: %d, info_2->eu_mode: %u, val: %u\n", ret, info_2->eu_mode, *val);

		ret = charger_partition_dealloc(CHARGER_PARTITION_HOST_KERNEL, CHARGER_PARTITION_INFO_2, sizeof(charger_partition_info_2));
		if(ret < 0) {
			pr_err("[ChgPartition] failed to dealloc\n");
			return -1;
		}
		break;
	default:
		break;
	}

	return ret;
}
EXPORT_SYMBOL(charger_partition_get_prop);

int charger_partition_set_prop(enum charger_partition_prop_list prop, u32 val)
{
	int ret = 0;
	charger_partition_info_1 info_1 = {.power_off_mode = 2, .zero_speed_mode = 2, .test = val, .reserved = 0};
	charger_partition_info_2 info_2 = {.eu_mode = 0, .test = val, .reserved = 0};

	switch (prop) {
	case CHARGER_PARTITION_PROP_TEST:
		ret = charger_partition_alloc(CHARGER_PARTITION_HOST_KERNEL, CHARGER_PARTITION_INFO_1, sizeof(charger_partition_info_1));
		if(ret < 0) {
			pr_err("[ChgPartition] failed to alloc\n");
			return -1;
		}

		info_1.power_off_mode = 2;
		info_1.zero_speed_mode = 2;
		info_1.test = val;
		info_1.reserved = 0;
		ret = charger_partition_write(CHARGER_PARTITION_HOST_KERNEL, CHARGER_PARTITION_INFO_1, (void *)&info_1, sizeof(charger_partition_info_1));
		if(ret < 0) {
			pr_err("[ChgPartition] failed to write\n");
			return -1;
		}
		pr_info("[ChgPartition] ret: %d, info_1.test: %u\n", ret, info_1.test);

		ret = charger_partition_dealloc(CHARGER_PARTITION_HOST_KERNEL, CHARGER_PARTITION_INFO_1, sizeof(charger_partition_info_1));
		if(ret < 0) {
			pr_err("[ChgPartition] failed to dealloc\n");
			return -1;
		}
		break;
	case CHARGER_PARTITION_PROP_POWER_OFF_MODE:
		ret = charger_partition_alloc(CHARGER_PARTITION_HOST_KERNEL, CHARGER_PARTITION_INFO_1, sizeof(charger_partition_info_1));
		if(ret < 0) {
			pr_err("[ChgPartition] failed to alloc\n");
			return -1;
		}

		info_1.power_off_mode = val;
		info_1.zero_speed_mode = 2;
		info_1.test = 0x34567890;
		info_1.reserved = 0;
		ret = charger_partition_write(CHARGER_PARTITION_HOST_KERNEL, CHARGER_PARTITION_INFO_1, (void *)&info_1, sizeof(charger_partition_info_1));
		if(ret < 0) {
			pr_err("[ChgPartition] failed to write\n");
			return -1;
		}
		pr_info("[ChgPartition] ret: %d, info_1.power_off_mode: %u\n", ret, info_1.power_off_mode);

		ret = charger_partition_dealloc(CHARGER_PARTITION_HOST_KERNEL, CHARGER_PARTITION_INFO_1, sizeof(charger_partition_info_1));
		if(ret < 0) {
			pr_err("[ChgPartition] failed to dealloc\n");
			return -1;
		}
		break;
	case CHARGER_PARTITION_PROP_EU_MODE:
		ret = charger_partition_alloc(CHARGER_PARTITION_HOST_KERNEL, CHARGER_PARTITION_INFO_2, sizeof(charger_partition_info_2));
		if(ret < 0) {
			pr_err("[ChgPartition] failed to alloc\n");
			return -1;
		}

		info_2.eu_mode = val;
		info_2.test = 0x34567890;
		info_2.reserved = 0;
		ret = charger_partition_write(CHARGER_PARTITION_HOST_KERNEL, CHARGER_PARTITION_INFO_2, (void *)&info_2, sizeof(charger_partition_info_2));
		if(ret < 0) {
			pr_err("[ChgPartition] failed to write\n");
			return -1;
		}
		pr_info("[ChgPartition] ret: %d, info_2.eu_mode: %u\n", ret, info_2.eu_mode);

		ret = charger_partition_dealloc(CHARGER_PARTITION_HOST_KERNEL, CHARGER_PARTITION_INFO_2, sizeof(charger_partition_info_2));
		if(ret < 0) {
			pr_err("[ChgPartition] failed to dealloc\n");
			return -1;
		}
		break;
	default:
		break;
	}

	return ret;
}
EXPORT_SYMBOL(charger_partition_set_prop);

static bool look_up_scsi_device(int lun)
{
	struct Scsi_Host *shost;

	shost = scsi_host_lookup(0);
	if (!shost)
		return false;

	charger_partition->sdev = scsi_device_lookup(shost, 0, 0, lun);
	if (!charger_partition->sdev)
		return false;

	scsi_host_put(shost);
	return true;
}

static bool check_device_is_correct(void)
{
	if (strncmp(charger_partition->sdev->host->hostt->proc_name, UFSHCD, strlen(UFSHCD))) {
		/*check if the device is ufs. If not, just return directly.*/
                pr_err("[ChgPartition] proc name is not ufshcd, name: %s", charger_partition->sdev->host->hostt->proc_name);
		return false;
	}

	return true;
}

static bool get_charger_partition_info(void)
{
	struct scsi_disk *sdkp = NULL;
	struct scsi_device *sdev = charger_partition->sdev;
	int part_number = charger_partition->part_info_part_number;
	struct block_device *part;

	if (!sdev->sdev_gendev.driver_data) {
		pr_err("[ChgPartition] scsi disk is null\n");
		return false;
	}

	sdkp = (struct scsi_disk *)sdev->sdev_gendev.driver_data;
	if (!sdkp->disk) {
		pr_err("[ChgPartition] gendisk is null\n");
		return false;
	}

	charger_partition->part_info_part_name = sdkp->disk->disk_name;
	pr_err("[ChgPartition] partion: %s part_number:%d\n",
			charger_partition->part_info_part_name, part_number);

	part = xa_load(&sdkp->disk->part_tbl, part_number);
	if (!part) {
		pr_err("[ChgPartition] device is null\n");
		return false;
	}

	if (strncmp(part->bd_meta_info->volname, "charger", sizeof("charger"))) {
		pr_err("[ChgPartition] this is not charger partition\n");
		return false;
	}

	part_info.part_start = part->bd_start_sect * PART_SECTOR_SIZE / PART_BLOCK_SIZE;
	part_info.part_size = bdev_nr_sectors(part) * PART_SECTOR_SIZE / PART_BLOCK_SIZE;

	pr_err("[ChgPartition] partion: %s start %llu(block) size %llu(block)\n",
			charger_partition->part_info_part_name, part_info.part_start, part_info.part_size);
	return true;
}

int check_charger_partition_header(void)
{
	int ret = 0;
	charger_partition_header *header = NULL;

	header = kzalloc(CHARGER_PARTITION_RWSIZE, GFP_KERNEL);
	if(!header){
		pr_err("[ChgPartition] malloc buf error!");
		return -1;
	}

	ret = charger_scsi_read_partition(charger_partition->sdev, (void *)header, (part_info.part_start + CHARGER_PARTITION_HEADER), CHARGER_PARTITION_RWSIZE / PART_BLOCK_SIZE);
	if (ret) {
		pr_err("[ChgPartition] charger read error %d\n", ret);
		kfree(header);
		return -1;
	}
	pr_err("[ChgPartition] magic:0x%0x, version:%d\n", header->magic, header->version);

	if(header->magic != CHARGER_PARTITION_MAGIC) {
		pr_err("[ChgPartition] magic error, set to default!\n");
		header->magic = CHARGER_PARTITION_MAGIC;
	}
	header->initialized = 1;
	header->avaliable = 1;

	ret = charger_scsi_write_partition(charger_partition->sdev, (void *)header, (part_info.part_start + CHARGER_PARTITION_HEADER), CHARGER_PARTITION_RWSIZE / PART_BLOCK_SIZE);
	if (ret) {
		pr_err("[ChgPartition] charger write error %d\n", ret);
		kfree(header);
		return -1;
	}

	ret = charger_scsi_read_partition(charger_partition->sdev, (void *)header, (part_info.part_start + CHARGER_PARTITION_HEADER), CHARGER_PARTITION_RWSIZE / PART_BLOCK_SIZE);
	if (ret) {
		pr_err("[ChgPartition] charger read error %d\n", ret);
	} else {
		pr_err("[ChgPartition] magic:0x%0x, version:%d, initialized:%d, available:%d\n", header->magic, header->version, header->initialized, header->avaliable);
	}

	pr_err("[ChgPartition] initiablized ok\n");

	kfree(header);
	return ret;
}

int get_charger_partition_info_1(void)
{
	int ret = 0;
	charger_partition_info_1 *info_1 = NULL;

	ret = charger_partition_alloc(CHARGER_PARTITION_HOST_KERNEL, CHARGER_PARTITION_INFO_1, sizeof(charger_partition_info_1));
	if(ret < 0) {
		pr_err("[ChgPartition] failed to alloc\n");
		return -1;
	}

	info_1 = (charger_partition_info_1 *)charger_partition_read(CHARGER_PARTITION_HOST_KERNEL, CHARGER_PARTITION_INFO_1, sizeof(charger_partition_info_1));
	if(!info_1) {
		pr_err("[ChgPartition] failed to read\n");
		ret = charger_partition_dealloc(CHARGER_PARTITION_HOST_KERNEL, CHARGER_PARTITION_INFO_1, sizeof(charger_partition_info_1));
		if(ret < 0) {
			pr_err("[ChgPartition] failed to dealloc\n");
			return -1;
		}
		return -1;
	}
	pr_err("[ChgPartition] ret:%d test:0x%0x zero_speed_mode:%u power_off_mode:%u\n", ret, info_1->test, info_1->zero_speed_mode, info_1->power_off_mode);

	ret = charger_partition_dealloc(CHARGER_PARTITION_HOST_KERNEL, CHARGER_PARTITION_INFO_1, sizeof(charger_partition_info_1));
	if(ret < 0) {
		pr_err("[ChgPartition] failed to dealloc\n");
		return -1;
	}
	return 0;
}

int set_charger_partition_info_1(void)
{
	int ret = 0;
	charger_partition_info_1 info_1 = {.power_off_mode = 2, .zero_speed_mode = 2, .test = 0x23456789, .reserved = 0};

	ret = charger_partition_alloc(CHARGER_PARTITION_HOST_KERNEL, CHARGER_PARTITION_INFO_1, sizeof(charger_partition_info_1));
	if(ret < 0) {
		pr_err("[ChgPartition] failed to alloc\n");
		return -1;
	}

	ret = charger_partition_write(CHARGER_PARTITION_HOST_KERNEL, CHARGER_PARTITION_INFO_1, (void *)&info_1, sizeof(charger_partition_info_1));
	if(ret < 0) {
		pr_err("[ChgPartition] failed to write\n");
		ret = charger_partition_dealloc(CHARGER_PARTITION_HOST_KERNEL, CHARGER_PARTITION_INFO_1, sizeof(charger_partition_info_1));
		if(ret < 0) {
			pr_err("[ChgPartition] failed to dealloc\n");
			return -1;
		}
		return -1;
	}

	ret = charger_partition_dealloc(CHARGER_PARTITION_HOST_KERNEL, CHARGER_PARTITION_INFO_1, sizeof(charger_partition_info_1));
	if(ret < 0) {
		pr_err("[ChgPartition] failed to dealloc\n");
		return -1;
	}
	return 0;
}

static void charger_partition_work(struct work_struct *work)
{
    // int ret = 0;
	int lun = 0;
#if defined(CONFIG_MI_O11_EMPTY_BATTERY)
	static int retry;
#endif
#if 0
	// char HWC[16]={0};
	uint32_t hw_country_ver = 0;

	hw_country_ver = get_hw_country_version();
	pr_err("[ChgPartition] get hw_country_ver: %u\n", hw_country_ver);
	if ((uint32_t)CountryCN == hw_country_ver) {
		charger_partition->part_info_part_number = 22;
	} else if ((uint32_t)CountryGlobal == hw_country_ver) {
		charger_partition->part_info_part_number = 22;
	} else if ((uint32_t)CountryIndia == hw_country_ver) {
		charger_partition->part_info_part_number = 22;
	} else {
		charger_partition->part_info_part_number = 22;
	}
#endif
	charger_partition->part_info_part_number = 23;

	//1. find charger partition
    for (lun = 0; lun < 6; lun++) {
		/*find charger partition scsi device*/
		if (!look_up_scsi_device(lun)) {
            pr_err("[ChgPartition] not find, continue...\n");
			continue;
		}

		/*check device is correct*/
		if (!check_device_is_correct()){
            pr_err("[ChgPartition] not find finally, won't read charger partition!!!\n");
			return;
        }

		if (get_charger_partition_info()) {
			pr_err("[ChgPartition] get partition info ok\n");
			charger_partition->is_charger_partition_rdy = true;
			check_charger_partition_header();
			/*reset info_1: power_off_mode and zero_speed_mode*/
			get_charger_partition_info_1();
			set_charger_partition_info_1();
			get_charger_partition_info_1();

			break;
		}
	}

#if defined(CONFIG_MI_O11_EMPTY_BATTERY)
	if (!charger_partition->is_charger_partition_rdy && retry++ < CHARGER_PARTITION_RETRY_TIMES) {
		pr_err("[ChgPartition] charger partition not ready, retry: %d/%d\n", retry, CHARGER_PARTITION_RETRY_TIMES);
		schedule_delayed_work(&charger_partition->charger_partition_work, msecs_to_jiffies(CHARGER_WORK_DELAY_MS));
	}
	if (retry == CHARGER_PARTITION_RETRY_TIMES) {
		if (look_up_scsi_device(0)) {
			if (check_device_is_correct()) {
				if (get_charger_partition_info()) {
					pr_err("[ChgPartition] get partition info ok\n");
					check_charger_partition_header();
					charger_partition->is_charger_partition_rdy = true;
					/*reset info_1: power_off_mode and zero_speed_mode*/
					get_charger_partition_info_1();
					set_charger_partition_info_1();
					get_charger_partition_info_1();
				}
			}
		}
	}
#endif
}

static int charger_partition_probe(struct platform_device *pdev)
{
    charger_partition = devm_kzalloc(&pdev->dev, sizeof(struct ChargerPartition), GFP_KERNEL);
	if (!charger_partition) {
		pr_err("[ChgPartition] out of memory\n");
		return -ENOMEM;
	}
	charger_partition->dev = &pdev->dev;
	platform_set_drvdata(pdev, charger_partition);

	INIT_DELAYED_WORK(&charger_partition->charger_partition_work, charger_partition_work);
	schedule_delayed_work(&charger_partition->charger_partition_work, msecs_to_jiffies(CHARGER_WORK_DELAY_MS));
	return 0;
}

static int charger_partition_remove(struct platform_device *pdev)
{
    cancel_delayed_work(&charger_partition->charger_partition_work);
	kfree(charger_partition);
	return 0;
}

static const struct of_device_id match_table[] = {
	{.compatible = "xiaomi,charger_partition"},
	{},
};

static struct platform_driver charger_partition_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "charger_partition",
		.of_match_table = match_table,
	},
	.probe = charger_partition_probe,
	.remove = charger_partition_remove,
};

static int __init charger_partition_init(void)
{
	return platform_driver_register(&charger_partition_driver);
}
module_init(charger_partition_init);

static void __exit charger_partition_exit(void)
{
	platform_driver_unregister(&charger_partition_driver);
}
module_exit(charger_partition_exit);

MODULE_DESCRIPTION("smart charge driver");
MODULE_AUTHOR("liluting@xiaomi.com");
MODULE_LICENSE("GPL v2");