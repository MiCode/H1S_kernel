/*
 * STICK ST1Wire driver
 * Copyright (C) 2020 ST Microelectronics S.A.
 */
#define LOGTAG "sticka101 "
#define pr_fmt(fmt) LOGTAG fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/err.h>
#include "battery_auth_class.h"

#define STICK_ID_NAME             "stick"
#define STICK_CMD_READ            0x06U
#define STICK_CMD_DECREMENT       0x05U
#define STICK_CMD_UPDATE          0x07U
#define STICK_CMD_REPEAT          0x3EU
#define STICK_UI_SOH              0x08U
#define STICK_RAW_SOH             0x09U
#define STICK_CYCLE_T             0x09U
#define STICK_DC_COUNTER          0x05U
#define DC_INIT_VALUE             0x1FFFF
#define INIT_ERR_VALUE            0xFFFFFF
#define INIT_DONE_VALUE           0x55
#define UISOH_MAX_LEN             11

#define GPIO_LOW  0
#define GPIO_HIGH 1
#define STICK_READ                   gpiod_get_raw_value(g_sd->gpiod)
#define STICK_1WIRE_SET_IN           gpiod_direction_input(g_sd->gpiod)
#define STICK_1WIRE_SET_OUT_LOW      gpiod_direction_output(g_sd->gpiod, GPIO_LOW)
#define STICK_1WIRE_SET_OUT_HIGH STICK_1WIRE_SET_IN
#define STICK_1WIRE_SET_LOW STICK_1WIRE_SET_OUT_LOW

/* STWire Status report definitions */
#define ST1WIRE_RECEIVE_TIMEOUT_LOW  -5
#define ST1WIRE_RECEIVE_TIMEOUT_HIGH -4
#define ST1WIRE_ARBITRATION_LOST     -3
#define ST1WIRE_RECEIVE_TIMEOUT      -2
#define ST1WIRE_ACK_ERROR            -1
#define ST1WIRE_OK                    0
/* ---------  STWire timmings in us -------- */
#define IDLE                       100
#define RECEIVE_TIMEOUT            10000
#define SLOW_INTERFRAME_DELAY      1000
#define SLOW_LONG_PULSE            14
#define SLOW_SHORT_PULSE           4
#define SLOW_WAIT_ACK              4
#define SLOW_ACK_PULSE             14
#define SLOW_START_PULSE           4 * (SLOW_LONG_PULSE + SLOW_SHORT_PULSE)	// 4 bit time = 72us
#define SLOW_DELAY_INTER_BYTE      8 * (SLOW_LONG_PULSE + SLOW_SHORT_PULSE)	// 8 bit time = 144us
#define SLOW_WAIT_RX               4
#define SLOW_WAIT_ACK_LOW_TIMEOUT  100
#define SLOW_WAIT_ACK_HIGH_TIMEOUT 100
#define HIBERNATE_CMD              0x02
#define ECHO_CMD                   0x00
#define STICK_RETRY_CNT            100
#define STICK_REPEAT_RETRY         10

struct stick_device {
	struct mutex mutex;
	struct device *dev;
	uint32_t last_counter;
	uint32_t last_cycle;
	uint32_t last_raw_soh;
	uint32_t last_ui_soh;
	uint8_t  ui_soh_bytes[UISOH_MAX_LEN];
	struct gpio_desc *gpiod;
	const char *auth_name;
	struct auth_device *auth_dev;
};

static struct stick_device *g_sd;

static void *hex2asc(unsigned char *dest, unsigned char *src, unsigned int len)
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

static void hex_dump(const char *prefix, unsigned char *output, int output_len)
{
	char buffer[1024];

	if (output_len == 0 || output == NULL) {
		pr_err("%s\n", prefix);
		return;
	} else if (output_len > (sizeof(buffer) / 2))
		output_len = (sizeof(buffer) / 2);
	memset(buffer, 0x00, sizeof(buffer));
	hex2asc(buffer, output, output_len << 1);
	pr_err("%s%s\n", prefix, buffer);
}

static void delayn(int ns)
{
	u64 pre, last;

	pre = ktime_get_boottime_ns();
	while (1) {
		last = ktime_get_boottime_ns();
		if (last - pre >= ns)
			break;
	}
}

static void delayu(int us)
{
	u64 pre, last;

	pre = ktime_get_boottime_ns();
	while (1) {
		last = ktime_get_boottime_ns();
		if (last - pre >= us * 1000)
			break;
	}
}

static void Delay_us_sleep(int T)
{
	if (T <= 10)
		udelay(T);
	else if (T <= 20000)
		usleep_range(T, T);
	else
		msleep(DIV_ROUND_UP(T, 1000));
}

static void stick_write_bit1(void)
{
	//Long Pulse H, short Pulse L
	STICK_1WIRE_SET_OUT_HIGH;
	delayu(SLOW_LONG_PULSE);
	STICK_1WIRE_SET_LOW;
	delayu(SLOW_SHORT_PULSE);
}

//sync bit, to write bit1
static void stick_write_sync_bit(void)
{
	stick_write_bit1();
}

static void stick_write_bit0(void)
{
	//Long Pulse L, short Pulse H
	STICK_1WIRE_SET_OUT_HIGH;
	delayu(SLOW_SHORT_PULSE);
	STICK_1WIRE_SET_LOW;
	delayu(SLOW_LONG_PULSE);
}

//after sending the last bit, set the io to high, waiting for STICK to set IO to low first, and to high later.
static int stick_read_ack_bit(void)
{
	int i = 0;

	//set IO to high
	STICK_1WIRE_SET_OUT_HIGH;
	//set IO from output to input status
	STICK_1WIRE_SET_IN;

	//wait for low
	while (STICK_READ) {
		i++;
		delayu(1);
		if (i >= SLOW_WAIT_ACK_LOW_TIMEOUT) {
			return ST1WIRE_ACK_ERROR;
		}
	}
	// - Wait for a high level on STWire
	i = 0;
	while (!STICK_READ) {
		i++;
		delayu(1);
		if (i >= SLOW_WAIT_ACK_HIGH_TIMEOUT) {
			return ST1WIRE_ACK_ERROR;
		}
	}
	return ST1WIRE_OK;
}

//after send the last bit, the io line is at high, to ack, need set the io to low first and then to high
static int stick_write_ack_bit(void)
{
	int j = 0;
	// wait for line high
	while (!STICK_READ) {
		j++;
		delayu(1);
		//add below to avoid dead loop
		if (j > RECEIVE_TIMEOUT)
			return ST1WIRE_ARBITRATION_LOST;
	}
	delayu(SLOW_WAIT_ACK);
	//set io to output and low
	STICK_1WIRE_SET_OUT_LOW;
	//keep low
	delayu(SLOW_ACK_PULSE);
	//set io to high
	STICK_1WIRE_SET_OUT_HIGH;
	return ST1WIRE_OK;
}

//read a byte and return it, if error return code value, needs to check
static int stick_read_byte(uint8_t *rcv_byte)
{
	int is_ok = 1, ret = ST1WIRE_OK;
	unsigned long flags;
	int i = 0, value = 0;
	uint64_t high_time_ns[8], low_time_ns[8];
	uint64_t high_start[8], high_end[8], low_start[8], low_end[8];

	memset(high_start, 0x00, sizeof(high_start));
	memset(high_end, 0x00, sizeof(high_end));
	memset(low_start, 0x00, sizeof(low_start));
	memset(low_end, 0x00, sizeof(low_end));

	raw_spin_lock_irqsave(&g_sd->auth_dev->io_lock, flags);
	stick_write_sync_bit();
	STICK_1WIRE_SET_IN;
	delayu(SLOW_WAIT_RX);
	for (i = 0; i < 8; i++) {
		high_start[i] = ktime_get_boottime_ns();
		while (STICK_READ) {
			high_end[i] = ktime_get_boottime_ns();
			// 30us timeout
			if (high_end[i] - high_start[i] >= 15000) {
				is_ok = 0;
				break;
			}
		}
		high_time_ns[i] = high_end[i] - high_start[i];
		delayn(50); // skip falling edge stable
		low_start[i] = ktime_get_boottime_ns();
		while (!STICK_READ) {
			low_end[i] = ktime_get_boottime_ns();
			// 30us timeout
			if (low_end[i] - low_start[i] >= 15000) {
				is_ok = 0;
				break;
			}
		}
		low_time_ns[i] = low_end[i] - low_start[i];
		delayn(50); // skip falling edge stable
		if (!is_ok) {
			ret = ST1WIRE_RECEIVE_TIMEOUT;
			break;
		}
	}
	if (is_ok)
		ret = stick_write_ack_bit();
	raw_spin_unlock_irqrestore(&g_sd->auth_dev->io_lock, flags);
	if (is_ok && ret == ST1WIRE_OK) {
		for (i = 0; i < 8; i++) {
			// bigger than 1us is logic one
			if (high_time_ns[i] >= (low_time_ns[i] + 1000))
				value |= 0x01 << (7 - i);
		}
		*rcv_byte = value;
	}

	return ret;
}

static int stick_write_byte(uint8_t byte)
{
	int ret;
	uint32_t i = 0;
	unsigned long flags;

	raw_spin_lock_irqsave(&g_sd->auth_dev->io_lock, flags);
	//send sync bit 1
	stick_write_sync_bit();
	// - Send Byte
	for (i = 0; i < 8; i++) {
		/* Mask each bit value */
		if (byte & (1 << (7 - i))) {
			/* - Send '1' */
			stick_write_bit1();
		} else {
			/* - Send '0' */
			stick_write_bit0();
		}
	}
	/*- Wait for Ack */
	ret = stick_read_ack_bit();
	raw_spin_unlock_irqrestore(&g_sd->auth_dev->io_lock, flags);

	return ret;
}
static int stick_set_frame_start(uint8_t byteval)
{
	int ret = 0;
	unsigned long flags;
	//make sure it is in idle status
	STICK_1WIRE_SET_IN;
	if (STICK_READ == GPIO_LOW) {
		pr_err("stickerr arb lost\n");
		return ST1WIRE_ARBITRATION_LOST;
	}
	raw_spin_lock_irqsave(&g_sd->auth_dev->io_lock, flags);
	 /* - Set bus to low level */
	STICK_1WIRE_SET_OUT_LOW;
	delayu(SLOW_START_PULSE);
	/* - Set bus back to high level */
	STICK_1WIRE_SET_OUT_HIGH;
	raw_spin_unlock_irqrestore(&g_sd->auth_dev->io_lock, flags);
	delayu(SLOW_DELAY_INTER_BYTE);
	hex_dump("<- ", &byteval, 1);
	ret = stick_write_byte(byteval);

	return ret;
}

static int stick_send_frame(uint8_t * buf, size_t frame_length)
{
	int i, ret;
	uint8_t ACKvalue = 0x00;

	ret = stick_set_frame_start(frame_length);
	if (ret != ST1WIRE_OK) {
		pr_err("stickerr set frame start failed in send frame\n");
		return ret;
	}
	print_hex_dump(KERN_ERR, LOGTAG"<- ", DUMP_PREFIX_NONE, 16, 1, buf, frame_length, 0);
	/* - Send Frame content */
	for (i = 0; i < frame_length; i++) {
		Delay_us_sleep(SLOW_DELAY_INTER_BYTE);	//8*18= 144us
		ret = stick_write_byte(buf[i]);
		if (ret != ST1WIRE_OK) {
			pr_err("stickerr write byte(%d) failed\n", i);
			break;
		}
	}
	if (i == frame_length)	//all sent in success
	{
		Delay_us_sleep(SLOW_DELAY_INTER_BYTE);	//8*18= 144us
		ret = stick_read_byte(&ACKvalue);
		hex_dump("-> ", &ACKvalue, 1);
		if (ret != ST1WIRE_OK || ACKvalue != 0x20) {
			pr_err("stickerr get ack failed in send frame(%d)\n", ret);
			if (ret != ST1WIRE_OK) {
				Delay_us_sleep(SLOW_INTERFRAME_DELAY);
				return ST1WIRE_ACK_ERROR;
			} else
				pr_err("stickerr ignores invalid ack in send frame\n");
		}
	}

	// Delay in ST1Wire slow to allow STICK Vcc to stabilize
	Delay_us_sleep(SLOW_INTERFRAME_DELAY);
	return ret;
}

static int stick_read_frame(uint8_t *buf, int len)
{
	int i, ret;
	uint8_t rcv_byte = 0x00, frame_length = 0x00;

	ret = stick_set_frame_start(0x00);
	if (ret != ST1WIRE_OK) {
		pr_err("stickerr set frame start failed in get frame\n");
		return ret;
	}
	Delay_us_sleep(SLOW_DELAY_INTER_BYTE);
	ret = stick_read_byte(&rcv_byte);
	hex_dump("-> ", &rcv_byte, 1);
	if (ret != ST1WIRE_OK || rcv_byte != 0x20) {
		pr_err("stickerr get ack failed in get frame(%d)\n", ret);
		if (ret != ST1WIRE_OK) {
			Delay_us_sleep(SLOW_INTERFRAME_DELAY);
			return ST1WIRE_ACK_ERROR;
		} else
			pr_err("stickerr ignores invalid ack in get frame\n");
	}
	Delay_us_sleep(SLOW_DELAY_INTER_BYTE);
	ret = stick_read_byte(&frame_length);
	hex_dump("-> ", &frame_length, 1);
	if (ret != ST1WIRE_OK) {
		pr_err("stickerr get frame length failed(%d)\n", ret);
		Delay_us_sleep(SLOW_INTERFRAME_DELAY);
		return ret;
	}
	for (i = 0; i < len; i++) {
		Delay_us_sleep(SLOW_DELAY_INTER_BYTE);
		ret = stick_read_byte(&buf[i]);
		if (ret != ST1WIRE_OK) {
			pr_err("stickerr read byte(%d) failed(%d)\n", i, ret);
			Delay_us_sleep(SLOW_INTERFRAME_DELAY);
			return ret;
		}
	}
	hex_dump("-> ", buf, len);
	return ST1WIRE_OK;
}

static void stick_cold_reset(void)
{
	// on ST1WIRE we just need to keep the IO low to go below POR
	STICK_1WIRE_SET_OUT_LOW;
	msleep(20);
	STICK_1WIRE_SET_OUT_HIGH;
	msleep(50);
	STICK_1WIRE_SET_IN;
	// Chip is active after cold reset
}

// reverse bits in a byte
static uint8_t reverse(uint8_t b)
{
	b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
	b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
	b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
	return b;
}

// compute CRC16
static uint16_t crypto_Crc16(uint8_t *pbuffer, uint16_t length)
{
	uint16_t Crc = 0xFFFF;

	static const uint16_t CrcTable[] = {
		// Byte Table, 0x1021 Left Shifting Polynomial - sourcer32@gmail.com
		0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
		0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
		0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
		0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
		0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
		0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
		0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
		0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
		0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
		0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
		0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
		0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
		0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
		0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
		0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
		0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
		0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
		0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
		0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
		0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
		0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
		0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
		0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
		0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
		0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
		0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
		0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
		0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
		0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
		0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
		0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
		0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0};

	while (length--) {
		uint8_t v = reverse(*pbuffer++);
		Crc = Crc ^ ((uint16_t)v << 8);
		Crc = (Crc << 8) ^ CrcTable[Crc >> 8]; // Process 8-bits at a time
	}

	return ~(reverse(Crc & 0xFF) << 8 | reverse(Crc >> 8));
}

static int stick_read_repeated(uint8_t *read_buf, int len)
{
	uint8_t ack_buf[32];
	uint16_t repeat_count_read = 0;
	uint16_t repeat_count_write = 0;
	uint16_t crc = 0, rsp_crc = 0;
	int ret = ST1WIRE_ACK_ERROR;
	uint8_t repeat_cmd[3] = { STICK_CMD_REPEAT, 0x28, 0x85 };

	while (repeat_count_read < STICK_REPEAT_RETRY) {
		ret = stick_read_frame(ack_buf, len + 2);
		rsp_crc = (ack_buf[len] << 8) | ack_buf[len + 1];
		crc = crypto_Crc16(&ack_buf[0], len);
		if (crc != rsp_crc || ret != ST1WIRE_OK) {
			pr_err("stickerr crc\n");
			crc = crypto_Crc16(&repeat_cmd[0], 1);
			repeat_cmd[1] = crc >> 8;
			repeat_cmd[2] = crc & 0xFF;
			repeat_count_write = 0;
			Delay_us_sleep(SLOW_DELAY_INTER_BYTE);
			while (repeat_count_write < STICK_REPEAT_RETRY) {
				ret = stick_send_frame(repeat_cmd, 3);
				if (ret == ST1WIRE_OK)
					break;
				repeat_count_write++;
			}
			pr_err("send frame retry(%u)\n", repeat_count_write);
			repeat_count_read++;
		} else {
			pr_err("read frame retry(%u)\n", repeat_count_read);
			memcpy(read_buf, ack_buf, len);
			break;
		}
	}
	if (repeat_count_read >= STICK_REPEAT_RETRY)
		pr_err("stickerr read repeated failed\n");

	return repeat_count_read < STICK_REPEAT_RETRY ? ST1WIRE_OK : ST1WIRE_ACK_ERROR;
}

static int stick_read_zone(uint8_t zone, uint8_t offset, uint8_t *read_buf, uint8_t len)
{
	uint8_t buf[24];
	uint16_t crc = 0;
	uint8_t cmd_buf[9];
	int i = 0, ret = ST1WIRE_ACK_ERROR;

	pr_err("%s enter\n", __func__);
	if (len > 16) {
		pr_err("stickerr invalid read length(%u)\n", len);
		return -EINVAL;
	}
	if (offset >= 16) {
		pr_err("stickerr invalid read offset(%u)\n", offset);
		return -EINVAL;
	}
	mutex_lock(&g_sd->mutex);
	for (i = 0; i < STICK_RETRY_CNT; i++) {
		cmd_buf[0] = STICK_CMD_READ;
		cmd_buf[1] = 0x00;
		cmd_buf[2] = zone;
		cmd_buf[3] = 0x00;
		cmd_buf[4] = offset;
		cmd_buf[5] = 0x00;
		cmd_buf[6] = len;
		crc = crypto_Crc16(&cmd_buf[0], 7);
		cmd_buf[7] = (uint8_t)((crc & 0xFF00) >> 8);
		cmd_buf[8] = (uint8_t)(crc & 0x00FF);

		stick_cold_reset();
		ret = stick_send_frame(cmd_buf, sizeof(cmd_buf));
		if (ret != ST1WIRE_OK) {
			ret = ST1WIRE_ACK_ERROR;
			pr_err("stickerr send read zone frame\n");
			continue;
		}
		msleep(10);
		ret = stick_read_repeated(buf, len + 1);
		if (ret != ST1WIRE_OK) {
			ret = ST1WIRE_ACK_ERROR;
			pr_err("stickerr read zone\n");
			continue;
		}
		// buf[0] is the status byte, always must be 0x00
		memcpy(read_buf, &buf[1], len);
		pr_err("read zone(%u) retry(%d)\n", zone, i);
		break;
	}
	if (i >= STICK_RETRY_CNT)
		pr_err("stickerr read zone failed\n");
	mutex_unlock(&g_sd->mutex);

	return ret;
}

static int stick_update_zone(uint8_t zone, uint8_t offset, uint8_t *write_buf, uint8_t len)
{
	int i, ret = ST1WIRE_ACK_ERROR;
	uint8_t ack;
	uint16_t crc = 0;
	uint8_t cmd_buf[32];

	pr_err("%s enter\n", __func__);
	if (len > 16) {
		pr_err("invalid write length(%u)\n", len);
		return -EINVAL;
	}
	if (offset >= 16) {
		pr_err("invalid write offset(%u)\n", offset);
		return -EINVAL;
	}
	mutex_lock(&g_sd->mutex);
	for (i = 0; i < STICK_RETRY_CNT; i++) {
		cmd_buf[0] = STICK_CMD_UPDATE;
		cmd_buf[1] = 0x00;
		cmd_buf[2] = zone;
		cmd_buf[3] = 0x00;
		cmd_buf[4] = offset;
		memcpy(&cmd_buf[5], write_buf, len);
		crc = crypto_Crc16(&cmd_buf[0], len + 5);
		cmd_buf[5 + len] = (uint8_t)((crc & 0xFF00) >> 8);
		cmd_buf[6 + len] = (uint8_t)(crc & 0x00FF);

		stick_cold_reset();
		ret = stick_send_frame(cmd_buf, len + 7);
		if (ret != ST1WIRE_OK) {
			ret = ST1WIRE_ACK_ERROR;
			pr_err("stickerr send update zone frame\n");
			continue;
		}
		msleep(10);
		ret = stick_read_repeated(&ack, 1);
		if (ret != ST1WIRE_OK) {
			ret = ST1WIRE_ACK_ERROR;
			pr_err("stickerr update zone\n");
			continue;
		}
		pr_err("stick update zone(%u) retry(%d)\n", zone, i);
		break;
	}
	if (i >= STICK_RETRY_CNT)
		pr_err("stickerr update zone failed\n");
	mutex_unlock(&g_sd->mutex);

	return ret;
}

static int stick_read_dc_counter(uint32_t *counter_val)
{
	uint8_t buf[4];
	uint16_t crc = 0;
	uint8_t cmd_buf[9];
	int i = 0, ret = ST1WIRE_ACK_ERROR;

	pr_err("%s enter\n", __func__);
	if (g_sd->last_counter != INIT_ERR_VALUE) {
		*counter_val = g_sd->last_counter;
		pr_err("use last dc counter value(%u)\n", g_sd->last_counter);
		return ST1WIRE_OK;
	}
	mutex_lock(&g_sd->mutex);
	for (i = 0; i < STICK_RETRY_CNT; i++) {
		cmd_buf[0] = STICK_CMD_READ;
		cmd_buf[1] = 0x00;
		cmd_buf[2] = STICK_DC_COUNTER;
		cmd_buf[3] = 0x00;
		cmd_buf[4] = 0x00;
		cmd_buf[5] = 0x00;
		cmd_buf[6] = 0x00;
		crc = crypto_Crc16(&cmd_buf[0], 7);
		cmd_buf[7] = (uint8_t)((crc & 0xFF00) >> 8);
		cmd_buf[8] = (uint8_t)(crc & 0x00FF);

		stick_cold_reset();
		ret = stick_send_frame(cmd_buf, sizeof(cmd_buf));
		if (ret != ST1WIRE_OK) {
			ret = ST1WIRE_ACK_ERROR;
			pr_err("stickerr send read dc frame\n");
			continue;
		}
		msleep(10);
		ret = stick_read_repeated(buf, 4);
		if (ret != ST1WIRE_OK) {
			ret = ST1WIRE_ACK_ERROR;
			pr_err("stickerr read dc counter\n");
			continue;
		}
		// buf[0] is the status byte, always must be 0x00
		*counter_val = (buf[1] << 16) | (buf[2] << 8) | buf[3];
		g_sd->last_counter = *counter_val;
		pr_err("read dc counter retry(%d)(%u)\n", i, *counter_val);
		break;
	}
	if (i >= STICK_RETRY_CNT)
		pr_err("stickerr read dc counter failed\n");
	mutex_unlock(&g_sd->mutex);

	return ret;
}

static int stick_decrease_counter(uint32_t amount, uint32_t *counter_val)
{
	uint16_t crc = 0;
	uint8_t  buf[4], cmd_buf[10];
	int      i, ret = ST1WIRE_ACK_ERROR;
	uint32_t  current_counter, last_new_counter;

	// update initial dc counter value
	last_new_counter = g_sd->last_counter;
	if (last_new_counter == INIT_ERR_VALUE) {
		ret = stick_read_dc_counter(&last_new_counter);
		if (ret != ST1WIRE_OK) {
			ret = ST1WIRE_ACK_ERROR;
			pr_err("stickerr update new dc counter failed\n");
			return -EINVAL;
		}
	}

	for (i = 0; i < STICK_RETRY_CNT; i++) {
		// skip first time recheck
		if (i > 0) {
			ret = stick_read_dc_counter(&current_counter);
			if (ret != ST1WIRE_OK) {
				ret = ST1WIRE_ACK_ERROR;
				pr_err("stickerr read new dc counter failed\n");
				return -EINVAL;
			}
			if (current_counter + amount == last_new_counter) {
				pr_err("decrease new counter retry(%d)(%u)\n", i - 1, *counter_val);
				break;
			}
		}
		cmd_buf[0] = STICK_CMD_DECREMENT;
		cmd_buf[1] = 0x00;
		cmd_buf[2] = STICK_DC_COUNTER;
		cmd_buf[3] = 0x00;
		cmd_buf[4] = 0x00;
		cmd_buf[5] = (uint8_t)((amount & 0xFF0000) >> 16);
		cmd_buf[6] = (uint8_t)((amount & 0x00FF00) >> 8);
		cmd_buf[7] = (uint8_t)(amount & 0x0000FF);
		crc = crypto_Crc16(&cmd_buf[0], 8);
		cmd_buf[8] = (uint8_t)((crc & 0xFF00) >> 8);
		cmd_buf[9] = (uint8_t)(crc & 0x00FF);

		stick_cold_reset();
		ret = stick_send_frame(cmd_buf, sizeof(cmd_buf));
		if (ret != ST1WIRE_OK) {
			ret = ST1WIRE_ACK_ERROR;
			pr_err("stickerr send decrease dc frame\n");
			continue;
		}
		msleep(10);
		ret = stick_read_repeated(buf, 4);
		if (ret != ST1WIRE_OK) {
			ret = ST1WIRE_ACK_ERROR;
			pr_err("stickerr decrease dc counter\n");
			continue;
		}
		*counter_val = (buf[1] << 16) | (buf[2] << 8) | buf[3];
		g_sd->last_counter = *counter_val;
		ret = ST1WIRE_OK;
		pr_err("decrease dc counter retry(%d)(%u)\n", i, *counter_val);
		break;
	}
	if (i >= STICK_RETRY_CNT)
		pr_err("stickerr decrease dc counter failed\n");

	return ret;
}

static int stick_read_cycle(u8 *new_cycle)
{
	if (g_sd->last_cycle != INIT_ERR_VALUE) {
		*new_cycle = g_sd->last_cycle;
		pr_err("use last cycle count(%u)\n", g_sd->last_cycle);
		return 0;
	} else {
		if (stick_read_zone(STICK_CYCLE_T, 15, new_cycle, 1) == ST1WIRE_OK) {
			g_sd->last_cycle = *new_cycle;
			pr_err("use new cycle count(%u)\n", g_sd->last_cycle);
			return 0;
		} else {
			g_sd->last_cycle = INIT_ERR_VALUE;
			pr_err("read new cycle count failed\n");
			return -1;
		}
	}
}

static int stick_write_cycle(u32 new_cycle)
{
	u8 cycle_count = new_cycle;

	if (stick_update_zone(STICK_CYCLE_T, 15, &cycle_count, 1) == ST1WIRE_OK) {
		g_sd->last_cycle = cycle_count;
		pr_err("write cycle count(%u) succeeded\n", cycle_count);
		return 0;
	} else {
		g_sd->last_cycle = INIT_ERR_VALUE;
		pr_err("write cycle count(%u) failed\n", cycle_count);
		return -1;
	}
}

int stick_get_cycle_count(struct auth_device *auth_dev, u32 *cycle_count)
{
	int     ret;
	uint8_t get_count = 0xff;

	pr_err("%s enter\n", __func__);
	ret = stick_read_cycle(&get_count);
	if (ret != ST1WIRE_OK) {
		pr_err("%s read cycle count failed\n", __func__);
		return -1;
	}
	pr_err("%s cycle count: %u\n", __func__, get_count);
	if (get_count == 0xff)
		get_count = 0;
	if (get_count >= 29) {
		ret = stick_read_dc_counter(cycle_count);
		if (ret != ST1WIRE_OK) {
			pr_err("%s read dc err\n", __func__);
			return -1;
		}
		*cycle_count = DC_INIT_VALUE - *cycle_count;
		pr_err("%s cycle_count:0x%2x\n", __func__, *cycle_count);
		if (*cycle_count < get_count)
			*cycle_count = get_count;
	} else {
		*cycle_count = get_count;
	}

	pr_err("%s cycle_count:0x%x\n", __func__, *cycle_count);
	return 0;
}

int stick_set_cycle_count(struct auth_device * auth_dev, u32 set_cycle_count, u32 get_cycle)
{
	u32 cycle_count;
	u32 counter_val;
	int ret = 0;
	int i = set_cycle_count - get_cycle;

	pr_err("%s add_num:%d\n", __func__, i);
	if (i <= 0)
		return 0;

	pr_err("%s set_cycle_count: %d\n", __func__, set_cycle_count);
	if (set_cycle_count < 30) {
		ret = stick_write_cycle(set_cycle_count);
		if (ret != ST1WIRE_OK) {
			pr_err("%s random_update_write fail!\n", __func__);
			return -1;
		}
	} else if (set_cycle_count >= 30 && get_cycle <= 29) {
		ret = stick_write_cycle(29);
		if (ret != ST1WIRE_OK) {
			pr_err("%s random_update_write fail!\n", __func__);
			return -1;
		}
		ret = stick_read_dc_counter(&cycle_count);
		if (ret != ST1WIRE_OK) {
			pr_err("%s read dc err\n", __func__);
			return -1;
		}
		pr_err("%s cycle_count: %d\n", __func__, cycle_count);
		get_cycle = DC_INIT_VALUE - cycle_count;
		i = set_cycle_count - get_cycle;
		if (i <= 0)
			return 0;

		ret = stick_decrease_counter(i, &counter_val);
		pr_err("%s counter_val: %u\n", __func__, counter_val);
		if (ret != ST1WIRE_OK){
			pr_err("%s %d decrease counter decrease failed: %d\n",
				__func__, __LINE__, ret);
			return -1;
		}
	} else {
		ret = stick_decrease_counter(i, &counter_val);
		pr_err("%s counter_val: %u\n", __func__, counter_val);
		if (ret != ST1WIRE_OK){
			pr_err("%s %d decrease counter decrease failed: %d\n",
				__func__, __LINE__, ret);
			return -1;
		}
	}
	return 0;
}

static int stick_get_ui_soh(struct auth_device *auth_dev, u8 *ui_soh_data, int len)
{
	if (len > UISOH_MAX_LEN) {
		pr_err("read invalid ui soh len(%d)\n", len);
		return -1;
	}

	if (g_sd->last_ui_soh != INIT_ERR_VALUE) {
		pr_err("use last ui soh\n");
		memcpy(ui_soh_data, g_sd->ui_soh_bytes, UISOH_MAX_LEN);
		print_hex_dump(KERN_ERR, LOGTAG"last ui soh <- ",
			DUMP_PREFIX_NONE, 16, 1, g_sd->ui_soh_bytes, UISOH_MAX_LEN, 0);
		return 0;
	} else {
		if (stick_read_zone(STICK_UI_SOH, 0, ui_soh_data, UISOH_MAX_LEN) == ST1WIRE_OK) {
			g_sd->last_ui_soh = INIT_DONE_VALUE;
			memcpy(g_sd->ui_soh_bytes, ui_soh_data, UISOH_MAX_LEN);
			print_hex_dump(KERN_ERR, LOGTAG"new ui soh <- ",
				DUMP_PREFIX_NONE, 16, 1, g_sd->ui_soh_bytes, UISOH_MAX_LEN, 0);
			return 0;
		} else {
			g_sd->last_ui_soh = INIT_ERR_VALUE;
			pr_err("read new ui soh failed\n");
			return -1;
		}
	}
}

static int stick_set_ui_soh(struct auth_device *auth_dev, u8 *ui_soh_data, int len, int raw_soh)
{
	u8 new_raw_soh = raw_soh;

	if (len > UISOH_MAX_LEN) {
		pr_err("write invalid ui soh len(%d)\n", len);
		return -1;
	}

	if (stick_update_zone(STICK_UI_SOH, 0, ui_soh_data, UISOH_MAX_LEN) == ST1WIRE_OK) {
		g_sd->last_ui_soh = INIT_DONE_VALUE;
		memcpy(g_sd->ui_soh_bytes, ui_soh_data, UISOH_MAX_LEN);
		pr_err("update ui soh succeeded\n");
		if (stick_update_zone(STICK_RAW_SOH, 0, &new_raw_soh, 1) == ST1WIRE_OK) {
			g_sd->last_raw_soh = new_raw_soh;
			pr_err("update raw soh(%u) succeeded\n", new_raw_soh);
			return 0;
		} else {
			g_sd->last_raw_soh = INIT_ERR_VALUE;
			pr_err("update raw soh failed\n");
			return -1;
		}
	} else {
		g_sd->last_ui_soh = INIT_ERR_VALUE;
		pr_err("update ui soh failed\n");
		return -1;
	}
}

static int stick_set_raw_soh(struct auth_device *auth_dev, int raw_soh)
{
	u8 new_raw_soh = raw_soh;

	if (stick_update_zone(STICK_RAW_SOH, 0, &new_raw_soh, 1) == ST1WIRE_OK) {
		g_sd->last_raw_soh = new_raw_soh;
		pr_err("write new raw soh(%u) succeeded\n", new_raw_soh);
		return 0;
	} else {
		g_sd->last_raw_soh = INIT_ERR_VALUE;
		pr_err("write new raw soh(%u) failed\n", new_raw_soh);
		return -1;
	}
}

static int stick_get_raw_soh(struct auth_device *auth_dev)
{
	u8  raw_soh;
	u32 new_raw_soh = INIT_ERR_VALUE;

	if (g_sd->last_raw_soh != INIT_ERR_VALUE) {
		new_raw_soh = g_sd->last_raw_soh;
		pr_err("last raw soh(%u)\n", g_sd->last_raw_soh);
	} else {
		if (stick_read_zone(STICK_RAW_SOH, 0, &raw_soh, 1) == ST1WIRE_OK) {
			g_sd->last_raw_soh = new_raw_soh = raw_soh;
			pr_err("read new raw soh(%u) succeeded\n", raw_soh);
		} else {
			g_sd->last_raw_soh = new_raw_soh = INIT_ERR_VALUE;
			pr_err("read new raw soh failed\n");
		}
	}

	if (new_raw_soh > 100) {
		if (stick_set_raw_soh(auth_dev, 100) == ST1WIRE_OK) {
			g_sd->last_raw_soh = 100;
			pr_err("update new raw soh to 100 succeeded\n");
			return 100;
		} else {
			g_sd->last_raw_soh = INIT_ERR_VALUE;
			pr_err("update new raw soh to 100 failed\n");
			return -1;
		}
	} else if (new_raw_soh == INIT_ERR_VALUE) {
		g_sd->last_raw_soh = INIT_ERR_VALUE;
		pr_err("invalid raw soh\n");
		return -1;
	} else {
		g_sd->last_raw_soh = new_raw_soh;
		pr_err("use last raw soh(%u)\n", g_sd->last_raw_soh);
		return new_raw_soh;
	}
}

static struct auth_ops st_auth_ops = {
	.get_cycle_count = stick_get_cycle_count,
	.set_cycle_count = stick_set_cycle_count,
	.get_ui_soh      = stick_get_ui_soh,
	.set_ui_soh      = stick_set_ui_soh,
	.get_raw_soh     = stick_get_raw_soh,
	.set_raw_soh     = stick_set_raw_soh,
};

static ssize_t cycle_soh_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	uint8_t cycle_soh[16] = { 0x00 };

	if (stick_read_zone(STICK_CYCLE_T, 0x00, cycle_soh, 16) == ST1WIRE_OK) {
		len += snprintf(buf + len, PAGE_SIZE, "raw soh(%u) cycle count(%u)\n", cycle_soh[0], cycle_soh[15]);
		if (stick_read_zone(STICK_UI_SOH, 0x00, cycle_soh, 11) == ST1WIRE_OK) {
			len += snprintf(buf + len, PAGE_SIZE, "ui soh: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
					cycle_soh[0], cycle_soh[1], cycle_soh[2], cycle_soh[3], cycle_soh[4],
					cycle_soh[5], cycle_soh[6], cycle_soh[7], cycle_soh[8], cycle_soh[9], cycle_soh[10]);
			return len;
		} else {
			len += snprintf(buf + len, PAGE_SIZE, "read ui soh failed\n");
			return len;
		}
	} else
		return snprintf(buf, PAGE_SIZE, "read raw soh/cycle count failed\n");
}

static ssize_t cycle_soh_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	char    *tx_ptr;
	char    *hexstr;
	char    txbuf[64];
	size_t  i, len = 0;
	uint8_t raw_soh, cycle_count;
	ulong   ui_soh[16];
	uint8_t ui_soh_byte[16];

	// set raw soh to 100: echo "0 100" > /sys/bus/platform/devices/st_stick/cycle_soh
	// set cycle count to 200: echo "1 200" > /sys/bus/platform/devices/st_stick/cycle_soh
	// set 11 bytes ui soh and raw soh, the last byte(1A) is the new raw soh
	// echo "2 00 01 02 03 04 05 06 07 08 09 0A 1A" > /sys/bus/platform/devices/st_stick/cycle_soh
	if (buf[0] == '0' || buf[0] == '1' || buf[0] == '2') {
		if (buf[0] == '0') {
			sscanf(&buf[2], "%u\n", &raw_soh);
			if (stick_update_zone(STICK_RAW_SOH, 0, &raw_soh, 1) == ST1WIRE_OK) {
				pr_err("store raw soh(%u) succeeded\n", raw_soh); 
				return size;
			} else {
				pr_err("store raw soh(%u) failed\n", raw_soh);
				return -EINVAL;
			}
		} else if (buf[0] == '1') {
			sscanf(&buf[2], "%u\n", &cycle_count);
			if (stick_update_zone(STICK_CYCLE_T, 15, &cycle_count, 1) == ST1WIRE_OK) {
				pr_err("store cycle count(%u) succeeded\n", cycle_count);
				return size;
			} else {
				pr_err("store cycle count(%u) failed\n", cycle_count);
				return -EINVAL;
			}
		} else {
			tx_ptr = txbuf;
			memset(txbuf, 0x00, sizeof(txbuf));
			strcpy(txbuf, &buf[2]);
			while ((hexstr = strsep(&tx_ptr, " ")) != NULL) {
				if (kstrtoul(hexstr, 16, &ui_soh[len]) < 0) {
					pr_err("parse ui soh params(%u) failed\n", len);
					return -EINVAL;
				}
				len++;
				// max length is 17 = 16(ui soh) + 1(raw soh)
				if (len >= 17)
					break;
			}
			for (i = 0; i < len; i++)
				ui_soh_byte[i] = ui_soh[i];
			// The last byte is new raw soh
			if (stick_update_zone(STICK_UI_SOH, 0, ui_soh_byte, len - 1) == ST1WIRE_OK)
				pr_err("store ui soh succeeded\n");
			else {
				pr_err("store ui soh failed\n");
				return -EINVAL;
			}
			if (stick_update_zone(STICK_RAW_SOH, 0, &ui_soh_byte[len - 1], 1) == ST1WIRE_OK) {
				pr_err("store raw soh(%u) succeeded\n", ui_soh_byte[len - 1]);
				return size;
			} else {
				pr_err("store raw soh(%u) failed\n", ui_soh_byte[len - 1]);
				return -EINVAL;
			}
		}
	} else {
		pr_err("invalid type: %c\n", buf[0]);
		return -EINVAL;
	}
}

static struct device_attribute dev_attr_cycle_soh =
	__ATTR(cycle_soh, S_IRUSR | S_IWUSR,
	cycle_soh_show, cycle_soh_store);

static ssize_t dc_counter_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	uint32_t counter_val;

	if (stick_read_dc_counter(&counter_val) == ST1WIRE_OK) {
		return snprintf(buf, PAGE_SIZE, "%06X\n", counter_val);
	} else
		return snprintf(buf, PAGE_SIZE, "failed\n");
}

static ssize_t dc_counter_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int ret = -EINVAL;
	uint8_t counter = 0;
	uint32_t dc_counter_val;

	sscanf(buf, "%u\n", &counter);
	pr_err("counter value: %d\n", counter);
	if (counter) {
		ret = stick_decrease_counter(counter, &dc_counter_val);
		if (ret == ST1WIRE_OK)
			pr_err("new dc counter: %u\n", dc_counter_val);
	} else
		pr_err("invalid counter\n");

	return (ret == ST1WIRE_OK) ? size : -EINVAL;
}

static struct device_attribute dev_attr_dc_counter =
	__ATTR(dc_counter, S_IRUSR | S_IWUSR,
	dc_counter_show, dc_counter_store);

static int stick_platform_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device *dev = &pdev->dev;
	struct stick_device *sd;

	pr_err("%s\n", __func__);
	sd = (struct stick_device *) devm_kzalloc(dev, sizeof(*sd),
						  GFP_KERNEL);
	if (sd == NULL) {
		pr_err("%s : Cannot allocate memory\n", __func__);
		return -ENOMEM;
	}
	g_sd = sd;
	sd->dev = dev;
	mutex_init(&sd->mutex);
	ret = of_property_read_string(pdev->dev.of_node,
		"auth_name", &sd->auth_name);
	if (ret < 0) {
		pr_info("%s can not find auth name(%d)\n", __func__, ret);
		sd->auth_name = "second_supplier";
	}
	sd->auth_dev = auth_device_register(sd->auth_name, NULL, sd, &st_auth_ops);
	if (IS_ERR_OR_NULL(sd->auth_dev)) {
		pr_err("auth device register fail\n");
		return PTR_ERR(sd->auth_dev);
	}

	dev_set_drvdata(dev, sd);
	sd->gpiod = sd->auth_dev->gpiod;
	g_sd->last_cycle   = INIT_ERR_VALUE;
	g_sd->last_raw_soh = INIT_ERR_VALUE;
	g_sd->last_ui_soh  = INIT_ERR_VALUE;
	g_sd->last_counter = INIT_ERR_VALUE;
	if (device_create_file(&pdev->dev, &dev_attr_cycle_soh))
		pr_err("create file cycle_soh failed\n");
	if (device_create_file(&pdev->dev, &dev_attr_dc_counter))
		pr_err("create file dc_counter failed\n");
	pr_err("%s end\n", __func__);

	return 0;
}
static int stick_platform_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct stick_device *sd;
	pr_err("%s\n", __func__);
	sd = dev_get_drvdata(dev);
	mutex_destroy(&sd->mutex);
	auth_device_unregister(sd->auth_dev);
	devm_kfree(dev, sd);
	dev_set_drvdata(dev, NULL);
	return 0;
}

static const struct of_device_id stick_dev_of_match[] = {
	{ .compatible = "st,stick", },
	{ },
};

MODULE_DEVICE_TABLE(of, stick_dev_of_match);

static struct platform_driver stick_platform_driver = {
	.probe = stick_platform_probe,
	.remove = stick_platform_remove,
	.driver = {
		   .name = STICK_ID_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = stick_dev_of_match,
	},
};

/* module load/unload record keeping */
static int __init stick_dev_init(void)
{
	return platform_driver_register(&stick_platform_driver);
}
module_init(stick_dev_init);

static void __exit stick_dev_exit(void)
{
	pr_info("%s : Unloading STICK driver\n", __func__);
	platform_driver_unregister(&stick_platform_driver);
}
module_exit(stick_dev_exit);
/* API for other kernel module stickapi */
MODULE_LICENSE("GPL");
