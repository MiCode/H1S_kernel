/*
 *  w1_slg.c - Aisinochip SLG driver
 *
 * Copyright (c) Shanghai Aisinochip Electronics Techology Co., Ltd.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/crc16.h>
#include <linux/uaccess.h>
#include <linux/scatterlist.h>
#include <linux/of_gpio.h>
#include <linux/random.h>
#include <linux/gpio.h>
#include <crypto/akcipher.h>
#include <crypto/rng.h>
#include <crypto/algapi.h>
#include <crypto/hash.h>
#include <linux/gpio/consumer.h>
#include "battery_auth_class.h"

#define CRC16_INIT      0
//#define W1_FAMILY_SLG               0xAC
#define SLG_DEBUG
#ifdef  SLG_DEBUG
#define DEBUG     printk
#define DEBUG_BYTES printf_hex
#else
#define DEBUG(fmt, ...)
#define DEBUG_BYTES
#endif
#define READ_PAGE_MAX               6
#define WRITE_PAGE_MAX              8
#define SLG_MEMORY_WRITE            0xDA
#define SLG_MEMORY_READ             0xCA
#define SLG_ECDSA_SIGN              0x2A
#define SLG_DC_DECREASE             0x24
#define SLG_DC_GET                  0xC0
#define SLG_ROMID_GET               0xB2
#define SLG_MANDI_GET               0x4F
#define SLG_POWEROFF_MODE           0x58
#define SLG_ECDSA_DISABLE           0xFC
#define SLG_CHIP_KILL               0xFE
#define RESPONSE_SUCCESS            0xAA
#define W1_BUS_ERROR                -201
#define COMMAND_ERROR               -202
#define PARAM_ERROR                 -203
#define RECV_CRC_ERROR              -204
#define RECV_LENGTH_ERROR           -205
#define SLG_NOT_EXIST               -210
#define SHASH_ALLOC_FAILED          -211
#define AKCIPHER_ALLOC_FAILED       -212
#define AKCIPHER_REQUEST_FAILED     -213
#define ECC_SIGN_WAIT_MAX           80
#define ECC_VERIFY_WAIT_MAX         150
#define MEMORY_WRITE_WAIT           100
#define GENERAL_WAIT_MAX            10
#define DC_INIT_VALUE				0x1FFFF
//struct w1_slave *slg_slave = NULL;
static int cmd_need_wait;
struct mutex slg_cmd_lock;
struct result_data_struct {
	unsigned char result_buf[128];
	int result_len;
	int status;
};
//static struct result_data_struct result_data;
struct slg_data {
	struct platform_device *pdev;
	struct device *dev;
	const char *auth_name;
	struct auth_device *auth_dev;
	uint8_t batt_id;
};
static struct slg_data *g_info;
#define WRITE_ONE_OUTPUT_L              (400L)
#define WRITE_ONE_OUTPUT_H              16
#define WRITE_ZERO_OUTPUT_L             (8500L)
#define WRITE_ZERO_OUTPUT_H             (5000L)
#define READ_BIT_OUTPUT_L               (400L)
#define READ_BIT_OUTPUT_H               (2000L)
#define READ_BIT_END                    12
#define W1_BUS_RESET_DELAY              (70)
#define W1_BUS_RESET_COUNT              3
#define W1_SKIP_ROM		0xCC
static int pullup_duration;
static const int DEVICE_CYCLE_T = 2;
static bool is_error = false;
static u8 w1_gpio_set_pullup(int delay)
{
	if (delay) {
		pullup_duration = delay;
	} else {
		if (pullup_duration) {
			/*
			 * This will OVERRIDE open drain emulation and force-pull
			 * the line high for some time.
			 */
			gpiod_set_raw_value(g_info->auth_dev->gpiod, 1);
			msleep(pullup_duration);
			/*
			 * This will simply set the line as input since we are doing
			 * open drain emulation in the GPIO library.
			 */
			gpiod_set_value(g_info->auth_dev->gpiod, 1);
		}
		pullup_duration = 0;
	}
	return 0;
}
static void w1_pre_write(void)
{
	if (pullup_duration) {
		w1_gpio_set_pullup(pullup_duration);
	}
}
static void w1_post_write(void)
{
	if (pullup_duration) {
		msleep(pullup_duration);
		pullup_duration = 0;
	}
}
static void w1_gpio_write_bit(u8 bit)
{
	gpiod_set_value(g_info->auth_dev->gpiod, bit);
}
static u8 w1_gpio_read_bit(void)
{
	return gpiod_get_value(g_info->auth_dev->gpiod) ? 1 : 0;
}
/*write/read ops*/
static void w1_write_bit(u8 bit)
{
	unsigned long flags = 0;
	u64 t;
	raw_spin_lock_irqsave(&g_info->auth_dev->io_lock, flags);
	if (bit) {
		//dev->bus_master->write_bit(dev->bus_master->data, 0);
		w1_gpio_write_bit(0);
		// t = ktime_get_ns();
		// t += WRITE_ONE_OUTPUT_L;
		// while (ktime_get_ns() < t);
		udelay(1);
		//dev->bus_master->write_bit(dev->bus_master->data, 1);
		w1_gpio_write_bit(1);
		raw_spin_unlock_irqrestore(&g_info->auth_dev->io_lock, flags);
		udelay(WRITE_ONE_OUTPUT_H);
	} else {
		//dev->bus_master->write_bit(dev->bus_master->data, 0);
		w1_gpio_write_bit(0);
		//keep to low
		t = ktime_get_ns();
		t += WRITE_ZERO_OUTPUT_L;
		while (ktime_get_ns() < t);
		//dev->bus_master->write_bit(dev->bus_master->data, 1);
		w1_gpio_write_bit(1);
		raw_spin_unlock_irqrestore(&g_info->auth_dev->io_lock, flags);
		//delay
		t = ktime_get_ns();
		t += WRITE_ZERO_OUTPUT_H;
		while (ktime_get_ns() < t);
	}
}
static u8 w1_read_bit(void)
{
	int result;
	unsigned long flags = 0;
	u64 t;
	/* sample timing is critical here */
	raw_spin_lock_irqsave(&g_info->auth_dev->io_lock, flags);
	//dev->bus_master->write_bit(dev->bus_master->data, 0);
	w1_gpio_write_bit(0);
	// t = ktime_get_ns();
	// t += READ_BIT_OUTPUT_L;
	// while (ktime_get_ns() < t);
	udelay(1);
	//dev->bus_master->write_bit(dev->bus_master->data, 1);
	w1_gpio_write_bit(1);
	t = ktime_get_ns();
	t += READ_BIT_OUTPUT_H;
	while (ktime_get_ns() < t);
	//result = dev->bus_master->read_bit(dev->bus_master->data);
	result = w1_gpio_read_bit();
	raw_spin_unlock_irqrestore(&g_info->auth_dev->io_lock, flags);
	udelay(READ_BIT_END);
	return result & 0x1;
}
static u8 w1_touch_bit(int bit)
{
	if (bit)
		return w1_read_bit();
	else {
		w1_write_bit(0);
		return 0;
	}
}
static void w1_write_8(u8 byte)
{
	int i;
	for (i = 0; i < 8; ++i) {
		if (i == 7)
			w1_pre_write();
		w1_touch_bit((byte >> i) & 0x1);
	}
	w1_post_write();
}
static void w1_write_block(const u8 * buf, int len)
{
	int i;
	for (i = 0; i < len; ++i)
		w1_write_8(buf[i]);	/* calls w1_pre_write */
	w1_post_write();
}
static u8 w1_read_8(void)
{
	int i;
	int res = 0;
	for (i = 0; i < 8; ++i)
		res |= (w1_touch_bit(1) << i);
	return res;
}
static u8 w1_read_block(u8 * buf, int len)
{
	int i;
	int ret;
	for (i = 0; i < len; ++i)
		buf[i] = w1_read_8();
	ret = len;
	return ret;
}
static void wait_slave_release(void)
{
	int counter;
	int result;
	for (counter = 0; counter < 200; counter++) {
		//result = dev->bus_master->read_bit(dev->bus_master->data) & 0x1;
		result = w1_gpio_read_bit() & 0x1;
		if (result == 1) {
			break;
		}
		udelay(1);
	}
}
static int w1_reset_bus(void)
{
	int result = 1;
	int counter = 0;
	int retry = W1_BUS_RESET_COUNT;
	int poweroff = 0;
	unsigned long flags = 0;
RESET_AGAIN:
	raw_spin_lock_irqsave(&g_info->auth_dev->io_lock, flags);
	//dev->bus_master->write_bit(dev->bus_master->data, 0);
	w1_gpio_write_bit(0);
	//delay 70us for 62.5kbps
	udelay(W1_BUS_RESET_DELAY);
	//dev->bus_master->write_bit(dev->bus_master->data, 1);
	w1_gpio_write_bit(1);
	for (counter = 0; counter < 100; counter++) {
		//result = dev->bus_master->read_bit(dev->bus_master->data) & 0x1;
		result = w1_gpio_read_bit() & 0x1;
		if (result == 0) {
			break;
		}
	}
	raw_spin_unlock_irqrestore(&g_info->auth_dev->io_lock, flags);
	udelay(1);
	if (result == 0) {
		//there is a slave, wait the bus release
		wait_slave_release();
	} else {
		retry--;
		if (retry > 0) {
			msleep(2);
			goto RESET_AGAIN;
		} else {
			if (poweroff == 0) {
				printk
				    ("[slg] slg reset failed, poweroff slg and retry again\n");
				poweroff = 1;
				//dev->bus_master->write_bit(dev->bus_master->data, 0);
				w1_gpio_write_bit(0);
				// delay 30ms let slg poweroff
				msleep(30);
				//dev->bus_master->write_bit(dev->bus_master->data, 1);
				w1_gpio_write_bit(1);
				// delay 10ms let slg poweron
				msleep(10);
				retry = W1_BUS_RESET_COUNT;
				goto RESET_AGAIN;
			}
		}
	}
	return result;
}
static int w1_reset_select_slave(void)
{
	if (w1_reset_bus())
		return -1;
	udelay(100);
	w1_write_8(W1_SKIP_ROM);
	return 0;
}
void *hex2asc(unsigned char *dest, unsigned char *src, unsigned int len)
{
	unsigned int i;
	unsigned char *p;
	p = dest;
	if (len % 2)
		*dest++ = (*src++ & 0x0F) + 0x30;
	for (i = 0; i < (len / 2); i++) {
		*dest++ = ((*src & 0xF0) >> 4) + 0x30;
		*dest++ = (*src++ & 0x0F) + 0x30;
	}
	while (p != dest) {
		if (*p >= 0x3A)
			*p += 7;
		p++;
	}
	return ((unsigned char *) dest);
}
void printf_hex(unsigned char *output, int output_len)
{
	char buffer[1024];
	if (output_len == 0) {
		return;
	} else if (output_len > (sizeof(buffer) / 2)) {
		output_len = (sizeof(buffer) / 2);
	}
	memset(buffer, 0x00, sizeof(buffer));
	hex2asc(buffer, output, output_len << 1);
	DEBUG("w1 data: %s\n", buffer);
}
/*
 * Send and Receive data by W1 bus
 *
 * @param input: Send data buffer
 * @param input_len: Send data length
 * @param output: Received data buffer
 * @param output_len: Received data length
 * @return: 0=Success; others=failure, see Error code
 *
 */
unsigned char buffer_bus[512];
unsigned char recv_bus[512];
static int bus_send_recv(unsigned char *input, int input_len,
			 unsigned char *output, int *output_len)
{
	unsigned short calc_crc;
	unsigned short recv_crc;
	int index, len;
	int ret;
	unsigned short retry = 5;
	mutex_lock(&slg_cmd_lock);
	if (input_len > 500) {
		mutex_unlock(&slg_cmd_lock);
		return PARAM_ERROR;
	}
	/*if(slg_slave == NULL)
	   {
	   return SLG_NOT_EXIST;
	   } */
	memset(buffer_bus, 0, sizeof(buffer_bus));
	memset(recv_bus, 0, sizeof(recv_bus));
	memcpy(buffer_bus, input, input_len);
	index = input_len;
	calc_crc = crc16(CRC16_INIT, buffer_bus, index);
	calc_crc ^= 0xFFFF;
	buffer_bus[index++] = calc_crc >> 8;
	buffer_bus[index++] = calc_crc & 0xFF;
	//mutex_lock(&slg_slave->master->mutex);
RETRY:
	if (w1_reset_select_slave()) {
		DEBUG("w1_reset_select_slave failed\n");
		ret = W1_BUS_ERROR;
		goto END;
	}
	udelay(100);
	//w1_write_block(slg_slave->master, buffer, index);
	w1_write_block(buffer_bus, index);
	mdelay(cmd_need_wait);
	memset(recv_bus, 0, sizeof(recv_bus));
	//w1_read_block(slg_slave->master, recv, 1);
	w1_read_block(recv_bus, 1);
	len = recv_bus[0];
	if (len <= 200) {
		//w1_read_block(slg_slave->master, &recv[1], len + 2);
		w1_read_block(&recv_bus[1], len + 2);
	} else {
		len = 0;
	}
	DEBUG("w1 send data:\n");
	DEBUG_BYTES(buffer_bus, index);
	DEBUG("w1 read length data: %02X\n", recv_bus[0]);
	if (len != 0) {
		DEBUG("w1 recv data:\n");
		DEBUG_BYTES(recv_bus, len + 3);
		calc_crc = crc16(CRC16_INIT, recv_bus, len + 1);
		calc_crc ^= 0xFFFF;
		recv_crc = (recv_bus[len + 1] << 8) + recv_bus[len + 2];
		DEBUG("w1 recv crc %X, clac crc: %X\n", recv_crc,
		      calc_crc);
		if (recv_crc == calc_crc) {
			DEBUG("w1 recv success\n");
			ret = 0;
			memcpy(output, &recv_bus[1], len);
			*output_len = len;
			if (recv_bus[1] == 0x22) {
				ret = RECV_CRC_ERROR;
			}
		} else {
			ret = RECV_CRC_ERROR;
		}
	} else {
		ret = RECV_LENGTH_ERROR;
	}
END:
	if (((ret == RECV_CRC_ERROR) || (ret == RECV_LENGTH_ERROR))
	    && (retry > 0)) {
		printk("[slg] command send failed retry %d\n",
		       5 - retry + 1);
		retry--;
		goto RETRY;
	}
	// mutex_unlock(&slg_slave->master->mutex);
	mutex_unlock(&slg_cmd_lock);
	return ret;
}

/**
 * Write a page of memory data to SLG
 *
 * @param index, page index, value range [0~15]
 * @param page, page data
 * @param pageSize, must be less than 32 bytes, when less than 32 bytes
 *        will be filled with 0xFF to 32 bytes in the function
 *
 * @return 0 success, other failed
 */
int slg_memory_write(int index, const unsigned char *page, int pageSize)
{
    unsigned char send[64];
    unsigned char recv[8];
    int ret;
    int recv_len;
    int writeSize = pageSize < 0x20 ? pageSize : 0x20;
    cmd_need_wait = WRITE_PAGE_MAX;

    if((index < 0) || (index > 15) || (page == NULL) || (pageSize > 0x20))
    {
        return -203;
    }

    memset(send, 0xFF, sizeof(send));
    memset(recv, 0xFF, sizeof(recv));

    send[0] = SLG_MEMORY_WRITE;
    send[1] = 0x21;
    send[2] = index;
    memcpy(&send[3], page, writeSize);

    ret = bus_send_recv(send, 0x23, recv, &recv_len);

    if(ret == 0)
    {
        if(recv[0] != RESPONSE_SUCCESS)
        {
            ret = -recv[0];
        }
    }

    return ret;
}

/*
 * Read a page of memory data from SLG
 *
 * @param index: The memory page index
 * @param output: Memory data buffer
 * @param output_len: Memory data length
 * @return: 0=Success; others=failure, see Error code
 */
int slg_memory_read(int index, unsigned char *output, int *output_len)
{
    unsigned char send[8];
    unsigned char recv[64];
    int ret;
    int recv_len;
    cmd_need_wait = READ_PAGE_MAX;

    *output_len = 0;

    send[0] = SLG_MEMORY_READ;
    send[1] = 1;
    send[2] = index;

    memset(recv, 0xFF, sizeof(recv));
    ret = bus_send_recv(send, 3, recv, &recv_len);

    if(ret == 0)
    {
        if(recv[0] != RESPONSE_SUCCESS)
        {
            ret = -recv[0];
        }
        else
        {
            memcpy(output, &recv[1], recv_len-1);
            *output_len = recv_len-1;
        }
    }

    return ret;
}


/*
 * Get the Decrement Counter
 *
 * @param counter: The initial value of the DC
 * @return: postive=The DC value; negative=failure, see Error code
 */
u32 slg_dc_get(void)
{
	unsigned char send[8];
	unsigned char recv[8];
	u32 ret;
	int recv_len;
	cmd_need_wait = GENERAL_WAIT_MAX;
	send[0] = SLG_DC_GET;
	send[1] = 0x00;
	memset(recv, 0xFF, sizeof(recv));
	ret = bus_send_recv(send, 2, recv, &recv_len);
	if (ret == 0) {
		if (recv[0] != RESPONSE_SUCCESS) {
			ret = -recv[0];
			is_error = true;
			pr_err("slg recv[0] error %x\n", ret);
		} else {
			ret =
			    (recv[1] << 24) + (recv[2] << 16) +
			    (recv[3] << 8) + (recv[4]);
			pr_err("slg dc_get %2x %2x %2x %2x\n", recv[1], recv[2], recv[3], recv[4]);
		}
	} else {
		is_error = true;
		pr_err("slg send_recv_error %x\n", ret);
	}
    
	return ret;
}

/*
 * Do decrease the Decrement Counter
 *
 * @return: 0=Success; others=failure, see Error code
 */
int slg_dc_decrease(void)
{
	unsigned char send[8];
	unsigned char recv[8];
	int ret;
	int recv_len;
	cmd_need_wait = GENERAL_WAIT_MAX;
	send[0] = SLG_DC_DECREASE;
	send[1] = 0x00;
	memset(recv, 0xFF, sizeof(recv));
	ret = bus_send_recv(send, 2, recv, &recv_len);
	if (ret == 0) {
		if (recv[0] != RESPONSE_SUCCESS) {
			ret = -recv[0];
		}
	}
	return ret;
}

int slg_get_cycle_count(struct auth_device *auth_dev, u32 * cycle_count)
{
	int ret;
	int len;
	int get_count;
	uint8_t device_pubkey[32] = {0};

	pr_err("%s \n", __func__);
	ret = slg_memory_read(DEVICE_CYCLE_T, device_pubkey, &len);
	if (ret != 0) {
		pr_err("%s read failed: %d\n", __func__, ret);
		return -1;
	}
	get_count = device_pubkey[30] & 0xff;
	if (get_count == 0xff)
		get_count = 0;

	pr_err("%s get_count:0x%2x\n", __func__, get_count);
	if (get_count >= 29) {
		ret = slg_dc_get();
		if (is_error) {
			is_error = false;
			return -1;
		}
		*cycle_count = DC_INIT_VALUE - ret;
		if (*cycle_count < get_count)
			*cycle_count = get_count;
	} else {
		*cycle_count = get_count;
	}

	pr_err("%s cycle_count:0x%x\n", __func__, *cycle_count);
	return 0;
}

int slg_set_cycle_count(struct auth_device * auth_dev, u32 set_cycle_count, u32 get_cycle)
{
	uint8_t device_pubkey[32] = {0};
	int i = set_cycle_count - get_cycle;
	int ret = 0;
	int len;

	pr_err("%s add_num:%d\n", __func__, i);
	if (i <= 0)
		return 0;

	ret = slg_memory_read(DEVICE_CYCLE_T, device_pubkey, &len);
	if (ret != 0) {
		pr_err("%s read fail\n", __func__);
		return -1;
	}
	pr_err("%s set_cycle_count: %d\n", __func__, set_cycle_count);
	if (set_cycle_count < 30) {
		device_pubkey[30] = set_cycle_count;
		ret = slg_memory_write(DEVICE_CYCLE_T, device_pubkey, len);
		if (ret != 0) {
			pr_err("%s slg_memory_write fail!\n", __func__);
			return -1;
		}
	} else if (set_cycle_count >= 30 && get_cycle <= 29) {
		device_pubkey[30] = 29;
		ret = slg_memory_write(DEVICE_CYCLE_T, device_pubkey, len);
		if (ret != 0) {
			pr_err("%s slg_memory_write fail!\n", __func__);
			return -1;
		}
		ret = slg_dc_get();
		if (is_error) {
			is_error = false;
			return -1;
		}
		get_cycle = DC_INIT_VALUE - ret;
		i = set_cycle_count - get_cycle;
		if (i <= 0)
			return 0;
		while(i--){
			ret = slg_dc_decrease();
			if (ret != 0){
				pr_err("%s %d decrease counter decrease failed: %d\n",
					__func__, __LINE__, ret);
				return -1;
			}
		}
	} else {
		while(i--) {
			ret = slg_dc_decrease();
			if (ret != 0){
				pr_err("%s %d decrease counter decrease failed: %d\n",
					__func__, __LINE__, ret);
				return -1;
			}
		}
	}
    return 0;
}

static int slg_get_ui_soh(struct auth_device *auth_dev, u8 *ui_soh_data, int len)
{
	int ret = 0, length = 0, i = 0, invalid_count = 0;
	uint8_t page_data[32];

	ret = slg_memory_read(DEVICE_CYCLE_T, page_data, &length);
	if (ret != 0) {
		pr_err("%s %d failed: %d\n", __func__, __LINE__, ret);
		return ret;
	}

	memcpy(ui_soh_data, page_data, len);

	for (i = 0; i < len; i++) {
		if (ui_soh_data[i] == 0xff)
			invalid_count++;
	}

	if (invalid_count >= 5) {
		pr_err("%s invalid value, set 0\n", __func__);
		memset(ui_soh_data, 0, len);
	}

	pr_err("%s ui_soh:\n", __func__);
	for (i = 0; i < len; i++) {
		pr_err("%s data[%d]:0x%x\n", __func__, i, ui_soh_data[i]);
	}

	return 0;
}

static int slg_set_ui_soh(struct auth_device *auth_dev, u8 *ui_soh_data, int len, int raw_soh)
{
	int ret = 0, length = 0, i = 0;
	unsigned char buf[128];

	ret = slg_memory_read(DEVICE_CYCLE_T, buf, &length);
	if (ret != 0) {
		pr_err("%s read fail\n", __func__);
		return 0;
	}

	for (i = 0; i < len; i++) {
		buf[i] = ui_soh_data[i];
	}
	buf[16] = raw_soh;

	ret = slg_memory_write(DEVICE_CYCLE_T, buf, 32);
	if (ret == 0)
		pr_err("%s slg_memory_write success!\n", __func__);

	pr_err("%s ui_soh:\n", __func__);
	for (i = 0; i < len; i++) {
		pr_err("%s buf[%d]:0x%x\n", __func__, i, buf[i]);
	}
	return 0;
}

static int slg_get_raw_soh(struct auth_device *auth_dev)
{
	int ret;
	int len;
	int raw_soh;
	uint8_t device_pubkey[32] = {0};

	pr_err("%s \n", __func__);
	ret = slg_memory_read(DEVICE_CYCLE_T, device_pubkey, &len);
	if (ret != 0) {
		pr_err("%s read failed: %d\n", __func__, ret);
		return -1;
	}

	raw_soh = device_pubkey[16] & 0xff;
	if (raw_soh > 100) {
		raw_soh = 100;
		device_pubkey[16] = raw_soh;
		ret = slg_memory_write(DEVICE_CYCLE_T, device_pubkey, len);
		if (ret != 0) {
			pr_err("%s slg_memory_write fail!\n", __func__);
			return -1;
		}
	}
	pr_err("%s raw_soh:0x%2x\n", __func__, raw_soh);
	return raw_soh;
}

static int slg_set_raw_soh(struct auth_device *auth_dev, int raw_soh)
{
	int ret = 0, len = 0;
	uint8_t device_pubkey[32] = {0};

	ret = slg_memory_read(DEVICE_CYCLE_T, device_pubkey, &len);
	if (ret != 0) {
		pr_err("%s read fail\n", __func__);
		return -1;
	}

	device_pubkey[16] = raw_soh;
	ret = slg_memory_write(DEVICE_CYCLE_T, device_pubkey, len);
	if (ret != 0) {
		pr_err("%s slg_memory_write fail!\n", __func__);
		return -1;
	}

	pr_err("%s raw_soh:%d\n", __func__, raw_soh);
	return 0;
}


struct auth_ops slg_auth_ops = {
	.set_cycle_count = slg_set_cycle_count,
	.get_cycle_count = slg_get_cycle_count,
	.get_ui_soh = slg_get_ui_soh,
	.set_ui_soh = slg_set_ui_soh,
	.get_raw_soh = slg_get_raw_soh,
	.set_raw_soh = slg_set_raw_soh,
};
static int slg_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct slg_data *info;
	pr_err("%s enter\n", __func__);
	info =
	    devm_kzalloc(&(pdev->dev), sizeof(struct slg_data),
			 GFP_KERNEL);
	if (!info) {
		pr_err("%s alloc mem fail\n", __func__);
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
		pr_info("%s can not find auth name(%d)\n", __func__, ret);
		info->auth_name = "main_supplier";
	}
	info->auth_dev = auth_device_register(info->auth_name, NULL, info,
					      &slg_auth_ops);
	if (IS_ERR_OR_NULL(info->auth_dev)) {
		pr_err("%s failed to register auth device\n", __func__);
		return PTR_ERR(info->auth_dev);
	}
	mutex_init(&slg_cmd_lock);
	g_info = info;
	return 0;
}
static int slg_remove(struct platform_device *pdev)
{
	mutex_destroy(&slg_cmd_lock);
	return 0;
}
static const struct of_device_id slg_of_ids[] = {
	{.compatible = "acl,slg"},
	{},
};
static struct platform_driver slg_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "acl,slg",
		   .of_match_table = slg_of_ids,
		   },
	.probe = slg_probe,
	.remove = slg_remove,
};
static int __init slg_init(void)
{
	pr_info("%s enter\n", __func__);
	return platform_driver_register(&slg_driver);
}
static void __exit slg_exit(void)
{
	pr_info("%s enter\n", __func__);
	platform_driver_unregister(&slg_driver);
}
module_init(slg_init);
module_exit(slg_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("LC inc.");
