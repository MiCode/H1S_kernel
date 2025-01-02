/*===========================================================================
*
=============================================================================
EDIT HISTORY
when           who     what, where, why
--------       ---     -----------------------------------------------------------
03/25/2020           Inital Release
=============================================================================*/
/*---------------------------------------------------------------------------
* Include Files
* -------------------------------------------------------------------------*/
#define pr_fmt(fmt)	"[ds28e16] %s: " fmt, __func__
#include <linux/slab.h>		/* kfree() */
#include <linux/module.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/power_supply.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/gpio/consumer.h>
#include <linux/regmap.h>
#include <linux/random.h>
#include "ds28e30.h"
#include "battery_auth_class.h"
//common define
#define ds_info	pr_err
#define ds_dbg	pr_err
#define ds_err	pr_err
#define ds_log	pr_err
// int OWSkipROM(void);
int pagenumber;
struct mutex ds_cmd_lock;
//maxim define
unsigned short CRC16;
const short oddparity[16] =
    { 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0 };
unsigned char last_result_byte = RESULT_SUCCESS;
//define system-level publick key, authority public key  and certificate constant variables
unsigned char SystemPublicKeyX[32];
unsigned char SystemPublicKeyY[32];
unsigned char AuthorityPublicKey_X[32];
unsigned char AuthorityPublicKey_Y[32];
unsigned char Page_Certificate_R[32];
unsigned char Page_Certificate_S[32];
unsigned char Certificate_Constant[16];
unsigned char Expected_CID[2];
unsigned char Expected_MAN_ID[2];
unsigned char challenge[32] =
    { 55, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00,
	00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 66
};

int auth_ANON = 1;
int auth_BDCONST = 1;

//define testing item result
#define Family_Code_Result    0
#define Custom_ID_Result 1	//custom ID is special for each mobile maker
#define Unique_ID_Result  2
#define MAN_ID_Result  3
#define Status_Result     4
#define Page0_Result  5
#define Page1_Result  6
#define Page2_Result  7
#define Page3_Result  8
#define CounterValue_Result 9
#define Verification_Signature_Result  10
#define Verification_Certificate_Result  11
#define Program_Page0_Result  12
#define Program_Page1_Result  13
#define Program_Page2_Result  14
#define Program_Page3_Result  15
#define DecreasingCounterValue_Result 16
#define Device_publickey_Result  17
#define Device_certificate_Result  18
//mi add
// unsigned char flag_mi_romid = 0;
unsigned char flag_mi_page0_data = 0;
unsigned char flag_mi_page1_data = 0;
unsigned char flag_mi_counter = 0;
unsigned char flag_mi_auth_result = 0;
unsigned char mi_romid[8] = { 0x00 };
unsigned char mi_status[12] = { 0x00 };	//0,1,2,3,4,5,6,7,28,29,36
unsigned char mi_page0_data[32] = { 0x00 };
unsigned char mi_page1_data[32] = { 0x00 };
unsigned char mi_counter[16] = { 0x00 };
int mi_auth_result = 0x00;
unsigned int attr_trytimes = 1;
struct ds_data {
	struct platform_device *pdev;
	struct device *dev;
	const char *auth_name;
	struct auth_device *auth_dev;
};
static struct ds_data *g_info;
#define ONE_WIRE_CONFIG_IN			gpiod_direction_input(g_info->auth_dev->gpiod)
#define ONE_WIRE_OUT_HIGH			gpiod_direction_output(g_info->auth_dev->gpiod, 1)
#define ONE_WIRE_OUT_LOW			gpiod_direction_output(g_info->auth_dev->gpiod, 0)
#define ONE_WIRE_GPIO_READ			gpiod_get_raw_value(g_info->auth_dev->gpiod)
#define ONE_WIRE_CONFIG_OUT		ONE_WIRE_CONFIG_IN
/* write/read ops */
static inline void Delay_ns(unsigned int T)
{
	u64 pre, last;

	pre = ktime_get_boottime_ns();
	while (1) {
		last = ktime_get_boottime_ns();
		if (last - pre >= T)
			break;
	}
}

static void Delay_us(unsigned int T)
{
	Delay_ns(T * 1000);
}
static unsigned char ow_reset(void)
{
	unsigned char presence = 0xFF;
	unsigned long flags;
	raw_spin_lock_irqsave(&g_info->auth_dev->io_lock, flags);
	ONE_WIRE_CONFIG_OUT;
	ONE_WIRE_OUT_LOW;
	Delay_us(48);		// 48
	ONE_WIRE_OUT_HIGH;
	ONE_WIRE_CONFIG_IN;
	Delay_us(7);
	/*presence = (unsigned char)((readl_relaxed(g_onewire_data->gpio_din_addr) \
	   & ONE_WIRE_GPIO_ADDR) >> ONE_WIRE_GPIO_OFFSET); // Read */
	presence = ONE_WIRE_GPIO_READ;
	Delay_us(50);
	raw_spin_unlock_irqrestore(&g_info->auth_dev->io_lock, flags);
	return presence;
}
static unsigned char read_bit(void)
{
	unsigned int vamm;
	ONE_WIRE_CONFIG_OUT;
	ONE_WIRE_OUT_LOW;
	//Delay_us(1);
	ONE_WIRE_CONFIG_IN;
	Delay_ns(100);
	//Delay_ns(500);
	vamm = ONE_WIRE_GPIO_READ;
	//vamm = readl_relaxed(g_onewire_data->gpio_din_addr); // Read
	Delay_us(5);
	ONE_WIRE_OUT_HIGH;
	ONE_WIRE_CONFIG_OUT;
	Delay_us(6);
	//return ((unsigned char)((vamm & ONE_WIRE_GPIO_ADDR) >> ONE_WIRE_GPIO_OFFSET));
	return vamm;
}
void write_bit(char bitval)
{
	ONE_WIRE_OUT_LOW;
	Delay_ns(200);
	if (bitval != 0)
		ONE_WIRE_OUT_HIGH;
	Delay_us(6);
	ONE_WIRE_OUT_HIGH;
	Delay_us(6);
}
static unsigned char read_byte(void)
{
	unsigned char i;
	unsigned char value = 0;
	unsigned long flags;
	raw_spin_lock_irqsave(&g_info->auth_dev->io_lock, flags);
	for (i = 0; i < 8; i++) {
		if (read_bit())
			value |= 0x01 << i;	// reads byte in, one byte at a time and then shifts it left
	}
	raw_spin_unlock_irqrestore(&g_info->auth_dev->io_lock, flags);
	return value;
}
static void write_byte(char val)
{
	unsigned char i;
	unsigned char temp;
	unsigned long flags;
	raw_spin_lock_irqsave(&g_info->auth_dev->io_lock, flags);
	ONE_WIRE_CONFIG_OUT;
	// writes byte, one bit at a time
	for (i = 0; i < 8; i++) {
		temp = val >> i;	// shifts val right ‘i’ spaces
		temp &= 0x01;	// copy that bit to temp
		write_bit(temp);	// write bit in temp into
	}
	raw_spin_unlock_irqrestore(&g_info->auth_dev->io_lock, flags);
}
unsigned char crc_low_first(unsigned char *ptr, unsigned char len)
{
	unsigned char i;
	unsigned char crc = 0x00;
	while (len--) {
		crc ^= *ptr++;
		for (i = 0; i < 8; ++i) {
			if (crc & 0x01)
				crc = (crc >> 1) ^ 0x8c;
			else
				crc = (crc >> 1);
		}
	}
	return (crc);
}
short Read_RomID(unsigned char *RomID)
{
	unsigned char i;
	unsigned char crc = 0x00;
	// if (flag_mi_romid == 2) {
	//      memcpy(RomID, mi_romid, 8);
	//      return DS_TRUE;
	// }
	if ((ow_reset()) != 0) {
		ds_err("Failed to reset ds28e30!\n");
		ow_reset();
		return ERROR_NO_DEVICE;
	}
	// ds_dbg("Ready to write 0x33 to maxim IC!\n");
	write_byte(CMD_READ_ROM);
	Delay_us(10);
	for (i = 0; i < 8; i++)
		RomID[i] = read_byte();
	ds_dbg("RomID = %02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x\n",
	       RomID[0], RomID[1], RomID[2], RomID[3],
	       RomID[4], RomID[5], RomID[6], RomID[7]);
	crc = crc_low_first(RomID, 7);
	// ds_dbg("crc_low_first = %02x\n", crc);
	if (crc == RomID[7]) {
		// if (flag_mi_status == 0)
		//      flag_mi_romid = 1;
		// else
		//      flag_mi_romid = 2;
		memcpy(mi_romid, RomID, 8);
		return DS_TRUE;
	} else {
		ow_reset();
		return DS_FALSE;
	}
}
unsigned short docrc16(unsigned short data)
{
	data = (data ^ (CRC16 & 0xff)) & 0xff;
	CRC16 >>= 8;
	if (oddparity[data & 0xf] ^ oddparity[data >> 4])
		CRC16 ^= 0xc001;
	data <<= 6;
	CRC16 ^= data;
	data <<= 1;
	CRC16 ^= data;
	return CRC16;
}
//---------------------------------------------------------------------------
/// @internal
///
/// Sent/receive standard flow command 
///
/// @param[in] write_buf
/// Buffer with write contents (preable payload)
/// @param[in] write_len
/// Total length of data to write in 'write_buf'
/// @param[in] delay_ms
/// Delay in milliseconds after command/preable.  If == 0 then can use 
/// repeated-start to re-access the device for read of result byte. 
/// @param[in] expect_read_len
/// Expected result read length 
/// @param[out] read_buf
/// Buffer to hold data read from device. It must be at least 255 bytes long. 
/// @param[out] read_len
/// Pointer to an integer to contain the length of data read and placed in read_buf
/// Preloaded with expected read length for 1-Wire mode. If (0) but expected_read=TRUE
/// then the first byte read is the length of data to read. 
///
///  @return
///  TRUE - command successful @n
///  FALSE - command failed
///
/// @endinternal
///
int standard_cmd_flow(unsigned char *write_buf, int write_len,
		      int delay_ms, int expect_read_len,
		      unsigned char *read_buf, int *read_len)
{
	unsigned char pkt[256] = { 0 };
	int pkt_len = 0;
	int i;

	mutex_lock(&ds_cmd_lock);
	// Reset/presence
	// Rom COMMAND (set from select options)
	// if((OWSkipROM() == 0))     return DS_FALSE;
	if ((ow_reset()) != 0) {
		ds_err("Failed to reset ds28e30!\n");
		ow_reset();
		mutex_unlock(&ds_cmd_lock);
		return ERROR_NO_DEVICE;
	}
	write_byte(CMD_SKIP_ROM);
	// set result byte to no response
	last_result_byte = RESULT_FAIL_NONE;
	// Construct write block, start with XPC command
	pkt[pkt_len++] = CMD_START;
	// Add length
	pkt[pkt_len++] = write_len;
	// write (first byte will be sub-command)
	memcpy(&pkt[pkt_len], write_buf, write_len);
	pkt_len += write_len;
	//send packet to DS28E30
	for (i = 0; i < pkt_len; i++)
		write_byte(pkt[i]);
	// read two CRC bytes
	pkt[pkt_len++] = read_byte();
	pkt[pkt_len++] = read_byte();
	// check CRC16
	CRC16 = 0;
	for (i = 0; i < pkt_len; i++)
		docrc16(pkt[i]);
	if( mi_romid[0] !=0 ) {		//?????
		if (CRC16 != 0xB001) {
			ow_reset();
			ds_info("standard_cmd_flow: 1 crc error!\n");
			mutex_unlock(&ds_cmd_lock);
			return DS_FALSE;
		}
	}		//?????
	if (delay_ms > 0) {
		// Send release byte, start strong pull-up
		write_byte(0xAA);
		// optional delay
		Delay_us(1000 * delay_ms);
	}
	// read FF and the length byte
	pkt[0] = read_byte();
	pkt[1] = read_byte();
	*read_len = pkt[1];
	// make sure there is a valid length
	if (*read_len != RESULT_FAIL_NONE) {
		// read packet
		for (i = 0; i < *read_len + 2; i++)
			read_buf[i] = read_byte();
		// check CRC16
		CRC16 = 0;
		docrc16(*read_len);
		for (i = 0; i < (*read_len + 2); i++)
			docrc16(read_buf[i]);
		if (CRC16 != 0xB001) {
			ds_info("standard_cmd_flow: 2 crc error!\n");
			mutex_unlock(&ds_cmd_lock);
			return DS_FALSE;
		}
		if (expect_read_len != *read_len) {
			ds_info("standard_cmd_flow: 2 len error!\n");
			mutex_unlock(&ds_cmd_lock);
			return DS_FALSE;
		}
	} else {
		mutex_unlock(&ds_cmd_lock);
		return DS_FALSE;
	}
	mutex_unlock(&ds_cmd_lock);
	return DS_TRUE;
}

//--------------------------------------------------------------------------
/// 'Read Memory' command
///
/// @param[in] pg
/// page number to read
/// @param[out] data
/// buffer length must be at least 32 bytes to hold memory read
///
///  @return
///  DS_TRUE - command successful @n
///  DS_FALSE - command failed
///
int ds28e30_cmd_readMemory(int pg, unsigned char *data)
{
	unsigned char write_buf[10];
	int write_len;
	unsigned char read_buf[255];
	int read_len;
	/*
	   Reset
	   Presence Pulse
	   <ROM Select>
	   TX: XPC Command (66h)
	   TX: Length byte 2d
	   TX: XPC sub-command 69h (Read Memory)
	   TX: Parameter (page)
	   RX: CRC16 (inverted of XPC command, length, sub-command, and parameter)
	   TX: Release Byte
	   <Delay TBD>
	   RX: Dummy Byte
	   RX: Length (33d)
	   RX: Result Byte
	   RX: Read page data (32d bytes)
	   RX: CRC16 (inverted, length byte, result byte, and page data)
	   Reset or send XPC command (66h) for a new sequence
	 */
	// construct the write buffer
	write_len = 0;
	write_buf[write_len++] = CMD_READ_MEM;
	write_buf[write_len++] = pg;
	// preload read_len with expected length
	read_len = 33;
	// default failure mode
	last_result_byte = RESULT_FAIL_NONE;
	ds_err("%s enter\n", __func__);
	// if(ds28e30_standard_cmd_flow(write_buf, DELAY_DS28E30_EE_READ_TRM, read_buf, &read_len, write_len))
	if (standard_cmd_flow
	    (write_buf, write_len, DELAY_DS28E30_EE_READ_TRM, read_len,
	     read_buf, &read_len)) {
		// get result byte
		last_result_byte = read_buf[0];
		// check result
		if (read_len == 33) {
			if (read_buf[0] == RESULT_SUCCESS) {
				memcpy(data, &read_buf[1], 32);
				if (pg == 0) {
					flag_mi_page0_data = 1;
					memcpy(mi_page0_data, data, 16);
				}
				if (pg == 1) {
					flag_mi_page1_data = 1;
					memcpy(mi_page1_data, data, 16);
				}
				if (pg == 106) {
					flag_mi_counter = 1;
					memcpy(mi_counter, data, 16);
				}
				return DS_TRUE;
			}
		}
	}
	return DS_FALSE;
}

////////////////////////////////////////////////////////////////////////////////
/*
* retry interface for read page data
*/
////////////////////////////////////////////////////////////////////////////////
int ds28e30_get_page_data_retry(int page, unsigned char *data)
{
	int i;
	ds_err("%s enter\n", __func__);
	if (page >= MAX_PAGENUM)
		return DS_FALSE;
	for (i = 0; i < GET_USER_MEMORY_RETRY; i++) {
		if (ds28e30_cmd_readMemory(page, data) ==
		    DS_TRUE) {
			ds_dbg("mi_counter data:\n");
			/*
			ds_dbg("%02x %02x %02x %02x %02x %02x %02x %02x",
			       mi_counter[0], mi_counter[1],
			       mi_counter[2], mi_counter[3],
			       mi_counter[4], mi_counter[5],
			       mi_counter[6], mi_counter[7]);
			ds_dbg("%02x %02x %02x %02x %02x %02x %02x %02x",
			       mi_counter[8], mi_counter[9],
			       mi_counter[10], mi_counter[11],
			       mi_counter[12], mi_counter[13],
			       mi_counter[14], mi_counter[15]);
			*/
			return DS_TRUE;
		}
	}
	return DS_FALSE;
}

static int ds28e30_ds_cycle_count_read(struct auth_device *auth_dev, u32 * cycle_count)
{
	int ret;
	unsigned char pagedata[32] = { 0x00 };
	ds_info("%s enter\n", __func__);
	ret = ds28e30_get_page_data_retry(DC_PAGE, pagedata);
	if (ret == DS_TRUE) {
		*cycle_count = (pagedata[2] << 16) + (pagedata[1] << 8)
		    + pagedata[0];
		ds_info("pagedata[2]=%d, pagedata[1]=%d, pagedata[0]=%d\n", pagedata[2], pagedata[1], pagedata[0]);
		ds_info("cycle_count0=%d\n", *cycle_count);
		*cycle_count = DC_INIT_VALUE - *cycle_count;
		ds_info("cycle_count1=%d\n", *cycle_count);
	} else {
		*cycle_count = 0;
	}
	return 0;
}

int DS28E30_cmd_decrementCounter(void)
{
	unsigned char write_buf[255];
	unsigned char read_buf[255];
	int write_len = 0;
	int read_len = 1;
	last_result_byte = RESULT_FAIL_NONE;
	/*
	   ?<Start, device address write>
	   ?TX: Decrement Counter Command
	   ?<Stop>
	   ?<Delay>
	   ?<Start, device address read>
	   ?RX: Length (SMBus) [always 1]
	   ?RX: Result byte
	   ?<Stop>
	 */
	write_buf[write_len++] = 1;
	write_buf[write_len++] = CMD_DECREMENT_CNT;
	if (standard_cmd_flow
	    (write_buf, write_len, DELAY_DS28E30_EE_WRITE_TWM + 50,
	     read_len, read_buf, &read_len)) {
		if (read_len == 1) {
			last_result_byte = read_buf[0];
			if (read_buf[0] == RESULT_SUCCESS)
				return DS_TRUE;
		}
	}
	ow_reset();
	return DS_FALSE;
}
static int ds28e30_ds_cycle_count_write(struct auth_device * auth_dev, u32 set_cycle_count, u32 get_cycle)
{
	int ret = 0;
	int i = set_cycle_count - get_cycle;
	if (i > 0) {
		while(i--) {
		    ret = DS28E30_cmd_decrementCounter();
		}
	}
	return ret;
}

struct auth_ops ds28e30_auth_ops = {
	.set_cycle_count = ds28e30_ds_cycle_count_write,
	.get_cycle_count = ds28e30_ds_cycle_count_read,
};
static int ds28e30_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct ds_data *info;
	ds_err("%s enter\n", __func__);
	info =
	    devm_kzalloc(&(pdev->dev), sizeof(struct ds_data), GFP_KERNEL);
	if (!info) {
		ds_err("%s alloc mem fail\n", __func__);
		return -ENOMEM;
	}
	if ((!pdev->dev.of_node
	     || !of_device_is_available(pdev->dev.of_node)))
		return -ENODEV;
	info->dev = &(pdev->dev);
	info->pdev = pdev;
	platform_set_drvdata(pdev, info);

	ret = of_property_read_string(pdev->dev.of_node,
		"auth_name", &info->auth_name);
	if (ret < 0) {
		ds_info("%s can not find auth name(%d)\n", __func__, ret);
		info->auth_name = "third_supplier";
	}
	info->auth_dev = auth_device_register(info->auth_name, NULL, info,
					      &ds28e30_auth_ops);
	if (IS_ERR_OR_NULL(info->auth_dev)) {
		ds_err("%s failed to register auth device\n", __func__);
		return PTR_ERR(info->auth_dev);
	}
	mutex_init(&ds_cmd_lock);
	g_info = info;

	return 0;
}
static int ds28e30_remove(struct platform_device *pdev)
{
	mutex_destroy(&ds_cmd_lock);
	return 0;
}
static const struct of_device_id ds28e30_of_ids[] = {
	{.compatible = "maxim,ds28e30"},
	{},
};
static struct platform_driver ds28e30_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "maxim,ds28e30",
		   .of_match_table = ds28e30_of_ids,
		   },
	.probe = ds28e30_probe,
	.remove = ds28e30_remove,
};
static int __init ds28e30_init(void)
{
	ds_err("%s enter\n", __func__);
	return platform_driver_register(&ds28e30_driver);
}
static void __exit ds28e30_exit(void)
{
	ds_err("%s enter\n", __func__);
	platform_driver_unregister(&ds28e30_driver);
}
module_init(ds28e30_init);
module_exit(ds28e30_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("LC inc.");
