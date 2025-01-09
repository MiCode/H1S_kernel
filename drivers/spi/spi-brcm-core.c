//Create by wangyu36@xiaomi.com
#if IS_ENABLED(CONFIG_BRCM_XGBE)
#include <linux/init.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/compat.h>

#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/msm_pcie.h>
#include "spi_common.h"
#include <linux/gpio.h>
#include <linux/gpio/machine.h>
#include <linux/of_gpio.h>

static void acd_init_for_1G(void);
static void acd_init_for_100M(int port);
static int bcm89272_suspend(void);
static int bcm89272_resume(void);

static int g_interrupt_gpio = -1;
static int g_wake_gpio = -1;
static int g_reset_gpio = -1;
static int g_psd_state = -1;

/*------------------------------ sysfs operation start ---------------------------*/
static int aps_spi_read(unsigned char *wr_buf, int wr_len, unsigned char *rd_buf, int rd_len)
{
	struct spi_ioc_transfer xfer[2];
	int status = 0;
	memset(xfer, 0, sizeof (xfer));

	xfer[0].tx_buf = (unsigned long)wr_buf;
	xfer[0].len = wr_len;
	xfer[0].rx_buf = (unsigned long long)NULL;

	xfer[1].tx_buf = (unsigned long long)NULL;
	xfer[1].len = rd_len;
	xfer[1].rx_buf = (unsigned long)rd_buf;

	status = spidev_message_kern(xfer, 2);

	return status;
}

static int spi_read16(unsigned int addr, unsigned short *rd_val)
{
	unsigned char buf[MAX_BUF_SZ];
	unsigned char val[2];
	int len = 0;
	int i, status;

	//opcode
	buf[len++] = SPI_OPCODE_PHYADDR(g_spi_id) | SPI_OPCODE_RD | SPI_OPCODE_NO_INC | SPI_OPCODE_RD_WAIT_2 | SPI_OPCODE_TX_SZ_16;
	buf[len++] = (addr >> 24) & 0xff;
	buf[len++] = (addr >> 16) & 0xff;
	buf[len++] = (addr >> 8) & 0xff;
	buf[len++] = (addr & 0xff);
	/* wait states as per opcode */
	for (i =0; i < ((buf[0] & SPI_OPCODE_RD_WAIT_MASK) >> SPI_OPCODE_RD_WAIT_SHIFT) * 2; i++)
		buf[len++] = 0x0;

	status = aps_spi_read(&buf[0], len, (unsigned char*)val, sizeof(unsigned short));

	*rd_val = ((unsigned int)val[0] << 8UL) | val[1];
	return status > 0 ? 0 : status;
}

static int aps_spi_write(unsigned char *buf, int len)
{
	struct spi_ioc_transfer xfer[1];
	int status = 0;

	memset(xfer, 0, sizeof (xfer));

	xfer[0].tx_buf = (unsigned long long)buf;
	xfer[0].len = len;
	xfer[0].rx_buf = (unsigned long long)NULL;

	status = spidev_message_kern(xfer, 1);

	return status;
}

static int spi_write16(unsigned int addr, unsigned short data)
{
	unsigned char txbuf[MAX_BUF_SZ];
	int len = 0;
	int status;

	//opcode
	txbuf[len++] = SPI_OPCODE_PHYADDR(g_spi_id) | SPI_OPCODE_WR | SPI_OPCODE_NO_INC | SPI_OPCODE_RD_WAIT_0 | SPI_OPCODE_TX_SZ_16;
	txbuf[len++] = (addr >> 24) & 0xff;
	txbuf[len++] = (addr >> 16) & 0xff;
	txbuf[len++] = (addr >> 8) & 0xff;
	txbuf[len++] = (addr & 0xff);
	txbuf[len++] = (data >> 8) & 0xff;
	txbuf[len++] = (data & 0xff);

	status = aps_spi_write(&txbuf[0], len);
	return status > 0 ? 0 : status;
}

static int spi_write8(unsigned int addr, unsigned char data)
{
    unsigned char buf[MAX_BUF_SZ];
    unsigned int len = 0;
    int status;

    //opcode
    buf[len++] = SPI_OPCODE_PHYADDR(g_spi_id) | SPI_OPCODE_WR | SPI_OPCODE_NO_INC | SPI_OPCODE_RD_WAIT_0 | SPI_OPCODE_TX_SZ_8;
    buf[len++] = (addr >> 24) & 0xff;
    buf[len++] = (addr >> 16) & 0xff;
    buf[len++] = (addr >> 8) & 0xff;
    buf[len++] = (addr & 0xff);
    buf[len++] = (data & 0xff);

    status = aps_spi_write(&buf[0], len);
    return status > 0 ? 0 : status;
}

static int spi_write32(unsigned int addr, unsigned int data)
{
	unsigned char buf[MAX_BUF_SZ];
	unsigned int len = 0;
	int status;

	//opcode
	buf[len++] = SPI_OPCODE_PHYADDR(g_spi_id) | SPI_OPCODE_WR | SPI_OPCODE_NO_INC | SPI_OPCODE_RD_WAIT_0 | SPI_OPCODE_TX_SZ_32;
	buf[len++] = (addr >> 24) & 0xff;
	buf[len++] = (addr >> 16) & 0xff;
	buf[len++] = (addr >> 8) & 0xff;
	buf[len++] = (addr & 0xff);
	buf[len++] = (data >> 24) & 0xff;
	buf[len++] = (data >> 16) & 0xff;
	buf[len++] = (data >> 8) & 0xff;
	buf[len++] = (data & 0xff);

	status = aps_spi_write(&buf[0], len);
	return status > 0 ? 0 : status;
}

static int spi_read32(unsigned int addr, unsigned int *rd_val)
{
	unsigned char txbuf[MAX_BUF_SZ];
	unsigned char val[4];
	int len = 0;
	int i, ret = -1;

	//opcode
	txbuf[len++] = SPI_OPCODE_PHYADDR(g_spi_id) | SPI_OPCODE_RD | SPI_OPCODE_NO_INC | SPI_OPCODE_RD_WAIT_2 | SPI_OPCODE_TX_SZ_32;
	txbuf[len++] = (addr >> 24) & 0xff;
	txbuf[len++] = (addr >> 16) & 0xff;
	txbuf[len++] = (addr >> 8) & 0xff;
	txbuf[len++] = (addr & 0xff);
	/* wait states as per opcode */
	for (i = 0; i < ((txbuf[0] & SPI_OPCODE_RD_WAIT_MASK) >> SPI_OPCODE_RD_WAIT_SHIFT) * 2; i++)
	txbuf[len++] = 0x0;
	ret = aps_spi_read(&txbuf[0], len, (unsigned char*)val, sizeof(unsigned int));
	if (ret) {
	printk("aps_spi_read failed ret=%d\n", ret);
	}
	*rd_val = ((unsigned int)val[0] << 24UL)
			| ((unsigned int)val[1] << 16UL)
			| ((unsigned int)val[2] << 8UL)
			| val[3];
	return ret;
}

static int spi_write64(unsigned int addr, unsigned long data)
{
	unsigned char buf[MAX_BUF_SZ];
	unsigned int len = 0;
	int status;

	//opcode
	buf[len++] = SPI_OPCODE_PHYADDR(g_spi_id) | SPI_OPCODE_WR | SPI_OPCODE_NO_INC | SPI_OPCODE_RD_WAIT_0 | SPI_OPCODE_TX_SZ_64;
	buf[len++] = (addr >> 24) & 0xff;
	buf[len++] = (addr >> 16) & 0xff;
	buf[len++] = (addr >> 8) & 0xff;
	buf[len++] = (addr & 0xff);
	buf[len++] = (data >> 56) & 0xff;
	buf[len++] = (data >> 48) & 0xff;
	buf[len++] = (data >> 40) & 0xff;
	buf[len++] = (data >> 32) & 0xff;
	buf[len++] = (data >> 24) & 0xff;
	buf[len++] = (data >> 16) & 0xff;
	buf[len++] = (data >> 8) & 0xff;
	buf[len++] = (data & 0xff);

	status = aps_spi_write(&buf[0], len);
	return status > 0 ? 0 : status;
}

static void acd_init_for_1G(void)
{
	const struct brcm_reg16_config init_1G_port_cfgtab[] = {
		{0x49030000 + 0x1E04, 0x0001}, //
		{0x49030000 + 0x2E0E, 0x0202}, // ACD_EXPC7
		{0x49030000 + 0x2E10, 0x7F50}, // ACD_EXPC8
		{0x49030000 + 0x2E12, 0x2C22}, // ACD_EXPC9
		{0x49030000 + 0x2E14, 0x5252}, // ACD_EXPCA
		{0x49030000 + 0x2E16, 0x0000}, // ACD_EXPCB
		{0x49030000 + 0x2E18, 0x0014}, // ACD_EXPCC
		{0x49030000 + 0x2E1C, 0x1CA3}, // ACD_EXPCE
		{0x49030000 + 0x2E1E, 0x0206}, // ACD_EXPCF
		{0x49030000 + 0x2E20, 0x0010}, // ACD_EXPE0
		{0x49030000 + 0x2E22, 0x0D0D}, // ACD_EXPE1
		{0x49030000 + 0x2E24, 0x0000}, // ACD_EXPE2
		{0x49030000 + 0x2E26, 0x7700}, // ACD_EXPE3
		{0x49030000 + 0x2E28, 0x0000}, // ACD_EXPE4
		{0x49030000 + 0x2E2E, 0x0000}, // ACD_EXPE7
		{0x49030000 + 0x2E3E, 0x409F}, // ACD_EXPEF
		{0x49030000 + 0x2E1A, 0x1129}, // ACD_EXPCD
		{0x49030000 + 0x2E1A, 0x0129}, // ACD_EXPCD
		{0x49030000 + 0x2E20, 0x0000}, // ACD_EXPE0
		{0x49030000 + 0x2E22, 0x0000}, // ACD_EXPE1
		{0x49030000 + 0x2E24, 0x0000}, // ACD_EXPE2
		{0x49030000 + 0x2E26, 0x0000}, // ACD_EXPE3
		{0x49030000 + 0x2E28, 0x0000}, // ACD_EXPE4
		{0x49030000 + 0x2E2E, 0x0000}, // ACD_EXPE7
		{0x49030000 + 0x2E3E, 0x0000}, // ACD_EXPEF
		{0x49030000 + 0x2E20, 0x3619}, // ACD_EXPE0
		{0x49030000 + 0x2E22, 0x343A}, // ACD_EXPE1
		{0x49030000 + 0x2E24, 0x0000}, // ACD_EXPE2
		{0x49030000 + 0x2E26, 0x0000}, // ACD_EXPE3
		{0x49030000 + 0x2E28, 0x8000}, // ACD_EXPE4
		{0x49030000 + 0x2E2A, 0x000E}, // ACD_EXPE5
		{0x49030000 + 0x2E2E, 0x0000}, // ACD_EXPE7
		{0x49030000 + 0x2E32, 0x0400}, // ACD_EXPE9
		{0x49030000 + 0x2E3A, 0x0000}, // ACD_EXPED
		{0x49030000 + 0x2E3E, 0xA2BF}, // ACD_EXPEF
		{0x49030000 + 0x2E1A, 0x1129}, // ACD_EXPCD
		{0x49030000 + 0x2E1A, 0x0129}, // ACD_EXPCD
		{0x49030000 + 0x2E20, 0x0000}, // ACD_EXPE0
		{0x49030000 + 0x2E22, 0x0000}, // ACD_EXPE1
		{0x49030000 + 0x2E24, 0x0000}, // ACD_EXPE2
		{0x49030000 + 0x2E26, 0x0000}, // ACD_EXPE3
		{0x49030000 + 0x2E28, 0x0000}, // ACD_EXPE4
		{0x49030000 + 0x2E2A, 0x0000}, // ACD_EXPE5
		{0x49030000 + 0x2E2E, 0x0000}, // ACD_EXPE7
		{0x49030000 + 0x2E30, 0x0000}, // ACD_EXPE8
		{0x49030000 + 0x2E32, 0x0000}, // ACD_EXPE9
		{0x49030000 + 0x2E3A, 0x0000}, // ACD_EXPED
		{0x49030000 + 0x2E3E, 0x0000}, // ACD_EXPEF
	};
	int status, i, regcount;

	status = spidev_open_kern();
	if (status) {
		goto err_open;
	}

	regcount = sizeof(init_1G_port_cfgtab) / sizeof(struct brcm_reg16_config);
	for (i = 0; i < regcount; i++) {
		status = spi_write16(init_1G_port_cfgtab[i].addr, init_1G_port_cfgtab[i].data);
		if (status) {
			break;
		}
	}

	spidev_release_kern();
err_open:
	return ;
}

static void acd_init_for_100M(int port)
{
	unsigned int base_addr = 0x490F0000 + port * 0x400000;
	const struct brcm_reg16_config init_100M_port_cfgtab[] = {
		{base_addr + 0x254E, 0xA01A},
		{base_addr + 0x2550, 0x0000},
		{base_addr + 0x2552, 0x00EF},
		{base_addr + 0x2558, 0x0200},
		{base_addr + 0x255C, 0x4000},
		{base_addr + 0x255E, 0x3000},
		{base_addr + 0x2560, 0x0015},
		{base_addr + 0x2562, 0x0D0D},
		{base_addr + 0x2564, 0x0000},
		{base_addr + 0x2566, 0x7700},
		{base_addr + 0x2568, 0x0000},
		{base_addr + 0x256E, 0x00A0},
		{base_addr + 0x257E, 0x409F},
		{base_addr + 0x255A, 0x1000},
		{base_addr + 0x255A, 0x0000},
		{base_addr + 0x2560, 0x0000},
		{base_addr + 0x2562, 0x0000},
		{base_addr + 0x2564, 0x0000},
		{base_addr + 0x2566, 0x0000},
		{base_addr + 0x2568, 0x0000},
		{base_addr + 0x256E, 0x0000},
		{base_addr + 0x257E, 0x0000},
		{base_addr + 0x2560, 0x3600},
		{base_addr + 0x2562, 0x343A},
		{base_addr + 0x2564, 0x0000},
		{base_addr + 0x2566, 0x0000},
		{base_addr + 0x2568, 0x8000},
		{base_addr + 0x256A, 0x000E},
		{base_addr + 0x256E, 0x0000},
		{base_addr + 0x2572, 0x0400},
		{base_addr + 0x257A, 0x0000},
		{base_addr + 0x257E, 0xA3BF},
		{base_addr + 0x255A, 0x1000},
		{base_addr + 0x255A, 0x0000},
		{base_addr + 0x2560, 0x0000},
		{base_addr + 0x2562, 0x0000},
		{base_addr + 0x2564, 0x0000},
		{base_addr + 0x2566, 0x0000},
		{base_addr + 0x2568, 0x0000},
		{base_addr + 0x256A, 0x0000},
		{base_addr + 0x256E, 0x0000},
		{base_addr + 0x2570, 0x0000},
		{base_addr + 0x2572, 0x0000},
		{base_addr + 0x257A, 0x0000},
		{base_addr + 0x257E, 0x0000},
		{base_addr + 0x2614, 0x2000},
		{base_addr + 0x2600, 0x8001},
		{base_addr + 0x2602, 0x9428},
	};
	int status, i, regcount;

	status = spidev_open_kern();
	if (status) {
		goto err_open;
	}

	regcount = sizeof(init_100M_port_cfgtab) / sizeof(struct brcm_reg16_config);
	for (i = 0; i < regcount; i++) {
		status = spi_write16(init_100M_port_cfgtab[i].addr, init_100M_port_cfgtab[i].data);
		if (status) {
			break;
		}
	}

	spidev_release_kern();
err_open:
	return ;
}

static int dut_status_show_for_port(struct device_driver *drv, int port)
{
	unsigned short read_data = 0, data;
	unsigned int read_addr, addr;
	int status, dut_status = 0;

	if (msm_pcie_deenumerate(1)) {
		goto err_deenum;
	}
	msleep(100);

	if (port == 0) {
		acd_init_for_1G();
	} else if (port == 3 || port == 5) {
		acd_init_for_100M(port);
	}

	status = spidev_open_kern();
	if (status) {
		dut_status = status == -EBUSY ? BRCM_SPI_OP_BUSY : BRCM_SPI_OP_ERROR;
		goto err_open;
	}

	if (port == 0) {
		read_addr = 0x49032E02, addr = 0x49032E00;
		// write 0x49032E00
		status = spi_read16(addr, &data);
		if (status) {
			goto err_done;
		}
		SET_BIT_ENABLE(data, 10);
		status = spi_write16(addr, data);
		if (status) {
			goto err_done;
		}
		data = 0;
		status = spi_read16(addr, &data);
		if (status) {
			goto err_done;
		}
		SET_BIT_ENABLE(data, 15);
		SET_BIT_ENABLE(data, 9);
		status = spi_write16(addr, data);
		if (status) {
			goto err_done;
		}

		// read 0x49032E02
		msleep(1000);
		status = spi_read16(read_addr, (&read_data));
		if (status) {
			goto err_done;
		}
	} else if (port == 3 || port == 5) {
		addr = 0x490F2540 + port * 0x400000, read_addr = 0x490F2542 + port * 0x400000;

		// write 0x49CF2540
		status = spi_read16(addr, &data);
		if (status) {
			goto err_done;
		}
		SET_BIT_ENABLE(data, 6);
		status = spi_write16(addr, data);
		if (status) {
			goto err_done;
		}
		data = 0;
		status = spi_write16(addr, data);
		if (status) {
			goto err_done;
		}
		status = spi_read16(addr, &data);
		if (status) {
			goto err_done;
		}
		SET_BIT_ENABLE(data, 13);
		status = spi_write16(addr, data);
		if (status) {
			goto err_done;
		}
		status = spi_read16(addr, &data);
		if (status) {
			goto err_done;
		}
		SET_BIT_ENABLE(data, 13);
		SET_BIT_ENABLE(data, 15);
		status = spi_write16(addr, data);
		if (status) {
			goto err_done;
		}

		status = spi_read16(addr, &data);
		if (status) {
			goto err_done;
		}
		SET_BIT_ENABLE(data, 10);
		SET_BIT_ENABLE(data, 13);
		status = spi_write16(addr, data);
		if (status) {
			goto err_done;
		}

		status = spi_read16(addr, &data);
		if (status) {
			goto err_done;
		}
		SET_BIT_ENABLE(data, 10);
		SET_BIT_ENABLE(data, 13);
		SET_BIT_ENABLE(data, 15);
		status = spi_write16(addr, data);
		if (status) {
			goto err_done;
		}

		status = spi_read16(addr, &data);
		if (status) {
			goto err_done;
		}
		SET_BIT_ENABLE(data, 10);
		SET_BIT_ENABLE(data, 15);
		status = spi_write16(addr, data);
		if (status) {
			goto err_done;
		}

		// read 0x4**F2542
		msleep(1000);
		status = spi_read16(read_addr, (&read_data));
		if (status) {
			goto err_done;
		}
	}

	// reset switch
	data = 0xFFFF;
	status = spi_write16(0x4A820024, data);
	if (status) {
		goto err_done;
	}
	dut_status = (read_data >> 12) & 0xF;

err_done:
	spidev_release_kern();
err_open:
        msleep(300);
	if (msm_pcie_enumerate(1)) {
		goto err_done;
	}
err_deenum:
	return dut_status;
}

static int link_status_show_for_port(struct device_driver *drv, int port)
{
	unsigned short read_data;
	unsigned int read_addr = 0x4B000100;
	int status, link_status = 2;

	status = spidev_open_kern();
	if (status) {
		link_status = status == -EBUSY ? BRCM_SPI_OP_BUSY : BRCM_SPI_OP_ERROR;
		goto err_open;
	}
	// read 0x4B000100
	if (spi_read16(read_addr, (&read_data))) {
		goto err_done;
	}
	link_status = GET_BIT(read_data, port);

err_done:
	spidev_release_kern();
err_open:
	return link_status;
}

static int sqi_status_show_for_port(struct device_driver *drv, int port)
{
	unsigned short read_data, data;
	unsigned int read_addr, addr;
	int status = 0, sqi_status = 0;

	status = spidev_open_kern();
	if (status) {
		sqi_status = status == -EBUSY ? BRCM_SPI_OP_BUSY : BRCM_SPI_OP_ERROR;
		goto err_open;
	}

	if (port == 0) {
		read_addr = 0x490300A4, addr = 0x49032016;
		status = spi_read16(addr, &data);
		if (status) {
			goto err_done;
		}
		SET_BIT_DISABLE(data, 13);
		status = spi_write16(addr, data);
		if (status) {
			goto err_done;
		}
	} else if (port == 3 || port == 5) {
		if (port == 3) {
			read_addr = 0x49CF22E8, addr = 0x49CF2050;
		} else {
			read_addr = 0x4A4F22E8, addr = 0x4A4F2050;
		}

		status = spi_read16(addr, &data);
		if (status) {
			goto err_done;
		}
		data = 0x0C30; // enable DSP clock
		status = spi_write16(addr, data);
		if (status) {
			goto err_done;
		}
	} else {
		goto err_done;
	}

	status = spi_read16(read_addr, &read_data);
	if (status) {
		goto err_done;
	}
	sqi_status = (read_data >> 1) & 0x7;

err_done:
	spidev_release_kern();
err_open:
	return sqi_status;
}

static int work_mode_show_store_for_port(struct device_driver *drv, int port, int operation, int set_mode)
{
	unsigned short data;
	unsigned int addr;
	int status, mode = 2;

	if (port == 0) {
		addr = 0x49021068;
	} else if (port == 3) {
		addr = 0x49C21068;
	} else if (port == 5) {
		addr = 0x4A421068;
	}

	status = spidev_open_kern();
	if (status) {
		mode = status == -EBUSY ? BRCM_SPI_OP_BUSY : BRCM_SPI_OP_ERROR;
		goto err_open;
	}

	if (operation == OP_READ) {
		status = spi_read16(addr, &data);
		if (status) {
			goto err_done;
		}
		mode = GET_BIT(data, 14);
	} else if (operation == OP_WRITE) {
		status = spi_read16(addr, &data);
		if (status) {
			goto err_done;
		}
		if (set_mode == 1) {
			SET_BIT_ENABLE(data, 14);
		} else if (set_mode == 0) {
			SET_BIT_DISABLE(data, 14);
		}
		status = spi_write16(addr, data);
		if (status) {
			goto err_done;
		}
	}

err_done:
	spidev_release_kern();
err_open:
	return mode;
}

static void test_tvco_for_1G(struct device_driver *drv)
{
	unsigned short data;
	unsigned int addr;
	int status;

	status = spidev_open_kern();
	if (status) {
		goto err_open;
	}

	// WDT_WdogControl: Disable Watchdog first
	addr = 0x40145008;
	data = 0;
	status = spi_write16(addr, data);
	if (status) {
		goto err_done;
	}

	// AFE_DIG_PLL_TEST
	addr = 0x49038060;
	status = spi_read16(addr, &data);
	if (status) {
		goto err_done;
	}

	SET_BIT_ENABLE(data, 0);
	SET_BIT_ENABLE(data, 1);

	status = spi_write16(addr, data);
	if (status) {
		goto err_done;
	}

	// PMA_PMD_IEEE_CONTROL_REG1
	addr = 0x49020000;
	status = spi_read16(addr, &data);
	if (status) {
		goto err_done;
	}

	SET_BIT_ENABLE(data, 1);
	SET_BIT_ENABLE(data, 6);

	status = spi_write16(addr, data);
	if (status) {
		goto err_done;
	}

err_done:
	spidev_release_kern();
err_open:
	return;
}

static void test_tvco(struct device_driver *drv, int port)
{
	unsigned short data;
	unsigned int addr = 0x4A4F2022; // BRPHY0_GPHY_CORE_SHD1C_01_ Px
	int status;

	status = spidev_open_kern();
	if (status) {
		goto err_open;
	}

	status = spi_read16(addr, &data);
	if (status) {
		goto err_done;
	}

	switch (port) {
	case 1:
		data = 0x415;
		break;
	case 2:
		data = 0x425;
		break;
	case 3:
		data = 0x435;
		break;
	case 4:
		data = 0x445;
		break;
	case 5:
		data = 0x405;
		break;

	default:
		goto err_done;
	}

	status = spi_write16(addr, data);
	if (status) {
		goto err_done;
	}

err_done:
	spidev_release_kern();
err_open:
	return;
}

static void test_for_1G(struct device_driver *drv, int test_mode)
{
	// 1G port only
	unsigned short data;
	int status;

	if (test_mode == TestMode_TVCO) {
		return test_tvco_for_1G(drv);
	}

	status = spidev_open_kern();
	if (status) {
		goto err_open;
	}

	// DSP_TOP_PHSHFT_CONTROL
	data = 0x40;
	status = spi_write16(0x49030A14, data);
	if (status) {
		goto err_done;
	}

	// AUTONEG_IEEE_AUTONEG_BASET1_AN_CONTROL
	status = spi_write16(0x490E0400, 0);
	if (status) {
		goto err_done;
	}

	// LINK_SYNC_CONTROL_A
	data = 0x3;
	status = spi_write16(0x49031E04, data);
	if (status) {
		goto err_done;
	}

	// PMA_PMD_IEEE_BASET1_PMA_PMD_CONTROL
	if (test_mode == TestMode_IB) {
		data = 0x8001;
	} else {
		data = 0xc001;
	}
	status = spi_write16(0x49021068, data);
	if (status) {
		goto err_done;
	}

	if (test_mode != TestMode_IB) {
		// PMA_PMD_IEEE_BASE1000T1_TEST_MODE_CONTROL
		status = spi_read16(0x49021208, &data);
		if (status) {
			goto err_done;
		}

		switch(test_mode) {
		case TestMode_2:
			data = 0x4000;
			break;
		case TestMode_4:
			data = 0x8000;
			break;
		case TestMode_5:
			data = 0xA000;
			break;
		case TestMode_6:
			data = 0xC000;
			break;
		default:
			goto err_done;
		}

		status = spi_write16(0x49021208, data);
		if (status) {
			goto err_done;
		}
	}

err_done:
	spidev_release_kern();
err_open:
	return;
}

static void set_test_mode_for_port(struct device_driver *drv, int port_num, const char* test_mode)
{
	unsigned short data;
	unsigned int addr1, addr2;
	int test_mode_num = 0, status;

	if (test_mode == NULL) {
		return;
	}
	test_mode_num = test_mode[0] - '0';
	if (test_mode_num <= 0 || test_mode_num > 6) {
		if (strncmp(test_mode, "IB", 2) == 0) {
			test_mode_num = TestMode_IB;
		} else if (strncmp(test_mode, "TVCO", 4) == 0) {
			test_mode_num = TestMode_TVCO;
		}
	}

	if (test_mode_num <= 0 || test_mode_num > TestMode_TVCO) {
		return;
	}

	if (port_num > 0 && test_mode_num == TestMode_TVCO) {
		return test_tvco(drv, port_num);
	}

	switch(port_num) {
	case 0:
		return test_for_1G(drv, test_mode_num);
	case 1:
		addr1 = 0x494E0400;
		addr2 = 0x4942106C;
		break;
	case 2:
		addr1 = 0x498E0400;
		addr2 = 0x4982106C;
		break;
	case 3:
		addr1 = 0x49CE0400;
		addr2 = 0x49C2106C;
		break;
	case 4:
		addr1 = 0x4A0E0400;
		addr2 = 0x4A02106C;
		break;
	case 5:
		addr1 = 0x4A4E0400;
		addr2 = 0x4A42106C;
		break;
	default:
		return;
	}

	status = spidev_open_kern();
	if (status) {
		goto err_open;
	}

	status = spi_write16(addr1, 0);
	if (status) {
		goto err_done;
	}

	switch(test_mode_num) {
	case 1:
		data = 0x2000;
		break;
	case 2:
		data = 0x4000;
		break;
	case 4:
		data = 0x8000;
		break;
	case 5:
		data = 0xA000;
		break;
	default:
		goto err_done;
	}

	status = spi_write16(addr2, data);
	if (status) {
		goto err_done;
	}

err_done:
	spidev_release_kern();
err_open:
	return;
}

static int get_normal_test_mode_show_for_port(struct device_driver *drv, int port)
{
	unsigned short read_data;
	unsigned int read_addr;
	int status, test_mode = 0;

	if (port < 0 || port > 5) {
		return test_mode;
	}

	if (port == 0) {
		read_addr = 0x49021208;
	} else {
		read_addr = 0x4902106C + port * 0x400000;
	}
	status = spidev_open_kern();
	if (status) {
		test_mode = status == -EBUSY ? BRCM_SPI_OP_BUSY : BRCM_SPI_OP_ERROR;
		goto err_open;
	}

	status = spi_read16(read_addr, &read_data);
	if (status) {
		goto err_done;
	}

	test_mode = read_data >> 13;
	if (test_mode) {
		goto err_done;
	}

err_done:
	spidev_release_kern();
err_open:
	return test_mode;
}

static int get_tvco_test_mode_show_for_port(struct device_driver *drv, int port)
{
	unsigned short read_data;
	unsigned int read_addr;
	int status, is_tvco = 0;

	if (port < 0 || port > 5) {
		return is_tvco;
	}

	if (port == 0 ) {
		read_addr = 0x49038060;
	} else {
		read_addr = 0x4A4F2022;
	}
	status = spidev_open_kern();
	if (status) {
		is_tvco = status == -EBUSY ? BRCM_SPI_OP_BUSY : BRCM_SPI_OP_ERROR;
		goto err_open;
	}

	status = spi_read16(read_addr, &read_data);
	if (status) {
		goto err_done;
	}

	if ((port == 0 && ((read_data & 0x3) == 0x3)) || (port == 3 && (read_data == 0x435))
		|| (port == 5 && (read_data == 0x405))) {
		is_tvco = 1;
	}

err_done:
	spidev_release_kern();
err_open:
	return is_tvco;
}

static ssize_t phy_bcm89272_port0_dut_status_show(struct device_driver *drv, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "0x%x\n", dut_status_show_for_port(drv, 0));
}
static DRIVER_ATTR_RO(phy_bcm89272_port0_dut_status);

static ssize_t phy_bcm89272_port3_dut_status_show(struct device_driver *drv, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "0x%x\n", dut_status_show_for_port(drv, 3));
}
static DRIVER_ATTR_RO(phy_bcm89272_port3_dut_status);

static ssize_t phy_bcm89272_port0_link_status_show(struct device_driver *drv, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", link_status_show_for_port(drv, 0));
}
static DRIVER_ATTR_RO(phy_bcm89272_port0_link_status);

static ssize_t phy_bcm89272_port0_sqi_status_show(struct device_driver *drv, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", sqi_status_show_for_port(drv, 0));
}
static DRIVER_ATTR_RO(phy_bcm89272_port0_sqi_status);

static ssize_t phy_bcm89272_port0_work_mode_show(struct device_driver *drv, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", work_mode_show_store_for_port(drv, 0, OP_READ, 0));
}

static ssize_t phy_bcm89272_port0_work_mode_store(struct device_driver *drv, const char *buf, size_t count)
{
	work_mode_show_store_for_port(drv, 0, OP_WRITE, buf[0] - '0');
	return count;
}
static DRIVER_ATTR_RW(phy_bcm89272_port0_work_mode);

static ssize_t phy_bcm89272_port3_link_status_show(struct device_driver *drv, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", link_status_show_for_port(drv, 3));
}
static DRIVER_ATTR_RO(phy_bcm89272_port3_link_status);

static ssize_t phy_bcm89272_port3_sqi_status_show(struct device_driver *drv, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", sqi_status_show_for_port(drv, 3));
}
static DRIVER_ATTR_RO(phy_bcm89272_port3_sqi_status);

static ssize_t phy_bcm89272_port3_work_mode_show(struct device_driver *drv, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", work_mode_show_store_for_port(drv, 3, OP_READ, 0));
}

static ssize_t phy_bcm89272_port3_work_mode_store(struct device_driver *drv, const char *buf, size_t count)
{
	work_mode_show_store_for_port(drv, 3, OP_WRITE, buf[0] - '0');
	return count;
}
static DRIVER_ATTR_RW(phy_bcm89272_port3_work_mode);

static ssize_t phy_bcm89272_port5_link_status_show(struct device_driver *drv, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", link_status_show_for_port(drv, 5));
}
static DRIVER_ATTR_RO(phy_bcm89272_port5_link_status);

static ssize_t phy_bcm89272_port5_sqi_status_show(struct device_driver *drv, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", sqi_status_show_for_port(drv, 5));
}
static DRIVER_ATTR_RO(phy_bcm89272_port5_sqi_status);

static ssize_t phy_bcm89272_port5_work_mode_show(struct device_driver *drv, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", work_mode_show_store_for_port(drv, 5, OP_READ, 0));
}

static ssize_t phy_bcm89272_port5_work_mode_store(struct device_driver *drv, const char *buf, size_t count)
{
	work_mode_show_store_for_port(drv, 5, OP_WRITE, buf[0] - '0');
	return count;
}
static DRIVER_ATTR_RW(phy_bcm89272_port5_work_mode);

static ssize_t phy_bcm89272_port5_dut_status_show(struct device_driver *drv, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "0x%x\n", dut_status_show_for_port(drv, 5));
}
static DRIVER_ATTR_RO(phy_bcm89272_port5_dut_status);

static ssize_t phy_bcm89272_PN_polarity_show(struct device_driver *drv, char *buf)
{
	unsigned short read_data;
	unsigned int read_addr = 0x49021202;
	int status, mode = 2;

	status = spidev_open_kern();
	if (status) {
		mode = status == -EBUSY ? BRCM_SPI_OP_BUSY : BRCM_SPI_OP_ERROR;
		goto err_done;
	}

	status = spi_read16(read_addr, &read_data);
	if (status) {
		goto err_done;
	}

	mode = GET_BIT(read_data, 2);

err_done:
	spidev_release_kern();
	return scnprintf(buf, PAGE_SIZE, "%d\n", mode);
}
static DRIVER_ATTR_RO(phy_bcm89272_PN_polarity);

static ssize_t phy_bcm89272_port0_set_test_mode_store(struct device_driver *drv, const char *buf, size_t count)
{
	set_test_mode_for_port(drv, 0, buf);
	return count;
}
static DRIVER_ATTR_WO(phy_bcm89272_port0_set_test_mode);

static ssize_t phy_bcm89272_port3_set_test_mode_store(struct device_driver *drv, const char *buf, size_t count)
{
	set_test_mode_for_port(drv, 3, buf);
	return count;
}
static DRIVER_ATTR_WO(phy_bcm89272_port3_set_test_mode);

static ssize_t phy_bcm89272_port5_set_test_mode_store(struct device_driver *drv, const char *buf, size_t count)
{
	set_test_mode_for_port(drv, 5, buf);
	return count;
}
static DRIVER_ATTR_WO(phy_bcm89272_port5_set_test_mode);

static ssize_t phy_bcm89272_port0_get_normal_test_mode_show(struct device_driver *drv, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", get_normal_test_mode_show_for_port(drv, 0));
}
static DRIVER_ATTR_RO(phy_bcm89272_port0_get_normal_test_mode);

static ssize_t phy_bcm89272_port3_get_normal_test_mode_show(struct device_driver *drv, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", get_normal_test_mode_show_for_port(drv, 3));
}
static DRIVER_ATTR_RO(phy_bcm89272_port3_get_normal_test_mode);

static ssize_t phy_bcm89272_port5_get_normal_test_mode_show(struct device_driver *drv, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", get_normal_test_mode_show_for_port(drv, 5));
}
static DRIVER_ATTR_RO(phy_bcm89272_port5_get_normal_test_mode);

static ssize_t phy_bcm89272_port0_get_tvco_test_mode_show(struct device_driver *drv, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", get_tvco_test_mode_show_for_port(drv, 0));
}
static DRIVER_ATTR_RO(phy_bcm89272_port0_get_tvco_test_mode);

static ssize_t phy_bcm89272_port3_get_tvco_test_mode_show(struct device_driver *drv, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", get_tvco_test_mode_show_for_port(drv, 3));
}
static DRIVER_ATTR_RO(phy_bcm89272_port3_get_tvco_test_mode);

static ssize_t phy_bcm89272_port5_get_tvco_test_mode_show(struct device_driver *drv, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", get_tvco_test_mode_show_for_port(drv, 5));
}
static DRIVER_ATTR_RO(phy_bcm89272_port5_get_tvco_test_mode);

static ssize_t phy_bcm89272_port0_get_ib_test_mode_show(struct device_driver *drv, char *buf)
{
	unsigned short read_data;
	unsigned int read_addr;
	int status, is_ib = 0;

	status = spidev_open_kern();
	if (status) {
		is_ib = status == -EBUSY ? BRCM_SPI_OP_BUSY : BRCM_SPI_OP_ERROR;
		goto err_open;
	}

	read_addr = 0x49030A14;
	status = spi_read16(read_addr, &read_data);
	if (status) {
		goto err_done;
	}
	if (GET_BIT(read_data, 6) != 1) {
		goto err_done;
	}

	read_addr = 0x490E0400;
	status = spi_read16(read_addr, &read_data);
	if (status) {
		goto err_done;
	}
	if (GET_BIT(read_data, 12) != 0) {
		goto err_done;
	}

	read_addr = 0x49031E04;
	status = spi_read16(read_addr, &read_data);
	if (status) {
		goto err_done;
	}
	if (GET_BIT(read_data, 0) != 1 || GET_BIT(read_data, 1) != 1) {
		goto err_done;
	}

	read_addr = 0x49021068;
	status = spi_read16(read_addr, &read_data);
	if (status) {
		goto err_done;
	}
	if (GET_BIT(read_data, 14) != 0) {
		goto err_done;
	}

	is_ib = 1;

err_done:
	spidev_release_kern();
err_open:
	return scnprintf(buf, PAGE_SIZE, "%d\n", is_ib);
}
static DRIVER_ATTR_RO(phy_bcm89272_port0_get_ib_test_mode);

/********************************************************************************************************/

static ssize_t phy_bcm89272_loopback_state_show(struct device_driver *drv, char *buf)
{
	unsigned short read_data;
	unsigned int read_addr = 0x49060000;
	int status, state = 2;

	status = spidev_open_kern();
	if (status) {
		state = status == -EBUSY ? BRCM_SPI_OP_BUSY : BRCM_SPI_OP_ERROR;
		goto err_open;
	}

	status = spi_read16(read_addr, &read_data);
	if (status) {
		goto err_done;
	}

	state = GET_BIT(read_data, 14);

err_done:
	spidev_release_kern();
err_open:
	return scnprintf(buf, PAGE_SIZE, "%d\n", state);
}

static ssize_t phy_bcm89272_loopback_state_store(struct device_driver *drv, const char *buf, size_t count)
{
	unsigned short data;
	unsigned int addr = 0x49060000;
	int status;

	status = spidev_open_kern();
	if (status) {
		goto err_open;
	}

	status = spi_read16(addr, &data);
	if (status) {
		goto err_done;
	}

	if (buf[0] == '1') {
		SET_BIT_ENABLE(data, 14);
	} else if (buf[0] == '0') {
		SET_BIT_DISABLE(data, 14);
	}
	status = spi_write16(addr, data);
	if (status) {
		goto err_done;
	}

err_done:
	spidev_release_kern();
err_open:
	return count;
}
static DRIVER_ATTR_RW(phy_bcm89272_loopback_state);

static ssize_t phy_bcm89272_external_loopback_state_show(struct device_driver *drv, char *buf)
{
	unsigned short read_data;
	unsigned int read_addr = 0x49032650;
	int status, state = 2;

	status = spidev_open_kern();
	if (status) {
		state = status == -EBUSY ? BRCM_SPI_OP_BUSY : BRCM_SPI_OP_ERROR;
		goto err_open;
	}

	status = spi_read16(read_addr, &read_data);
	if (status) {
		goto err_done;
	}

	state = GET_BIT(read_data, 15);

err_done:
	spidev_release_kern();
err_open:
	return scnprintf(buf, PAGE_SIZE, "%d\n", state);
}

static ssize_t phy_bcm89272_external_loopback_state_store(struct device_driver *drv, const char *buf, size_t count)
{
	unsigned short data;
	unsigned int addr = 0x49032650;
	int status;

	status = spidev_open_kern();
	if (status) {
		goto err_open;
	}

	status = spi_read16(addr, &data);
	if (status) {
		goto err_done;
	}

	if (buf[0] == '1') {
		SET_BIT_ENABLE(data, 15);
	} else if (buf[0] == '0') {
		SET_BIT_DISABLE(data, 15);
	}
	status = spi_write16(addr, data);
	if (status) {
		goto err_done;
	}

err_done:
	spidev_release_kern();
err_open:
	return count;
}
static DRIVER_ATTR_RW(phy_bcm89272_external_loopback_state);

static ssize_t phy_bcm89272_enable_port3_5_store(struct device_driver *drv, const char *buf, size_t count)
{
	unsigned short data = 0;
	unsigned int addr = 0x4b000003;
	int status;

	status = spidev_open_kern();
	if (status) {
		goto err_open;
	}

	status = spi_write8(addr, data);
	if (status) {
		goto err_done;
	}

	addr = 0x4b000005;
	status = spi_write8(addr, data);
	if (status) {
		goto err_done;
	}

err_done:
	spidev_release_kern();
err_open:
	return count;
}
static DRIVER_ATTR_WO(phy_bcm89272_enable_port3_5);

static void create_default_vlan_for_port(struct device_driver *drv, unsigned short port, unsigned short vlan, unsigned short pri)
{
	unsigned int data = 0;
	unsigned int base_addr = 0x4b003410;
	int status;

	status = spidev_open_kern();
	if (status) {
		goto err_open;
	}

	data += pri << 13;
	data += vlan;
	status = spi_write16(base_addr + (2*port), data);
	if (status) {
		goto err_done;
	}

err_done:
	spidev_release_kern();
err_open:
	return ;
}

static void config_vlan_for_port(struct device_driver *drv, unsigned short port,
                                unsigned short vlan, unsigned short operation)
{
	unsigned int data;
	int status;

	status = spidev_open_kern();
	if (status) {
		goto err_open;
	}

	// read vlan port mask
	status = spi_write16(0x4b000581, vlan);
	if (status) {
		goto err_done;
	}

	status = spi_write32(0x4b000580, 0x0);
	if (status) {
		goto err_done;
	}

	status = spi_write32(0x4b000580, 0x81);
	if (status) {
		goto err_done;
	}
	msleep(300);
	status = spi_read32(0x4b000583, &data);
	if (BRCM_OP_ADD == operation) {
		SET_BIT_ENABLE(data, port);
	} else if (BRCM_OP_DEL == operation) {
		SET_BIT_DISABLE(data, port);
	}

	// add port to vlan
	status = spi_write8(0x4b003400, 0xe3);
	if (status) {
		goto err_done;
	}

	status = spi_write32(0x4b000583, data);
	if (status) {
		goto err_done;
	}

	status = spi_write16(0x4b000581, vlan);
	if (status) {
		goto err_done;
	}

	status = spi_write32(0x4b000580, 0x0);
	if (status) {
		goto err_done;
	}

	status = spi_write32(0x4b000580, 0x80);
	if (status) {
		goto err_done;
	}

err_done:
	spidev_release_kern();
err_open:
	return ;
}

static void config_static_arl_for_port(struct device_driver *drv, unsigned short port,
                                        unsigned short operation)
{
	int status;

	status = spidev_open_kern();
	if (status) {
		goto err_open;
	}

	if (port == 4) {
		status = spi_write64(0x4b000502, 0x020400000022);
		if (status) {
			goto err_done;
		}
		status = spi_write16(0x4b000508, 0x21);
		if (status) {
			goto err_done;
		}
		status = spi_write64(0x4b000510, 0x21020400000022);
		if (status) {
			goto err_done;
		}
		if (BRCM_OP_ADD == operation) {
			status = spi_write32(0x4b000518, 0x18004);
			if (status) {
				goto err_done;
			}
		} else if (BRCM_OP_DEL == operation) {
			status = spi_write32(0x4b000518, 0x0);
			if (status) {
				goto err_done;
			}
		}
		status = spi_write8(0x4b000500, 0x0);
		if (status) {
			goto err_done;
		}
		status = spi_write8(0x4b000500, 0x80);
		if (status) {
			goto err_done;
		}
		msleep(300);
		status = spi_write64(0x4b000502, 0x020400000022);
		if (status) {
			goto err_done;
		}
		status = spi_write16(0x4b000508, 0x2D);
		if (status) {
			goto err_done;
		}
		status = spi_write64(0x4b000510, 0x2D020400000022);
		if (status) {
			goto err_done;
		}
		if (BRCM_OP_ADD == operation) {
			status = spi_write32(0x4b000518, 0x18004);
			if (status) {
				goto err_done;
			}
		} else if (BRCM_OP_DEL == operation) {
			status = spi_write32(0x4b000518, 0x0);
			if (status) {
				goto err_done;
			}
		}
		status = spi_write8(0x4b000500, 0x0);
		if (status) {
			goto err_done;
		}
		status = spi_write8(0x4b000500, 0x80);
		if (status) {
			goto err_done;
		}
		msleep(300);
		status = spi_write64(0x4b000502, 0x01005E7F0001);
		if (status) {
			goto err_done;
		}
		status = spi_write16(0x4b000508, 0x2D);
		if (status) {
			goto err_done;
		}
		status = spi_write64(0x4b000510, 0x2D01005E7F0001);
		if (status) {
			goto err_done;
		}
		if (BRCM_OP_ADD == operation) {
			status = spi_write32(0x4b000518, 0x18151);
			if (status) {
				goto err_done;
			}
		} else if (BRCM_OP_DEL == operation) {
			status = spi_write32(0x4b000518, 0x18141);
			if (status) {
				goto err_done;
			}
		}
		status = spi_write8(0x4b000500, 0x0);
		if (status) {
			goto err_done;
		}
		status = spi_write8(0x4b000500, 0x80);
		if (status) {
			goto err_done;
		}
	}

err_done:
	spidev_release_kern();
err_open:
	return ;
}

static ssize_t phy_bcm89272_config_port_vlan_membership_store(struct device_driver *drv, const char *buf, size_t count)
{
	unsigned short port = buf[0] - '0';
	if (port < 0 || port > 8) {
		return count;
	}
	switch(port) {
		case 3:
			create_default_vlan_for_port(drv, port, VLAN_33, VLAN_PRI_0);
			config_vlan_for_port(drv, port, VLAN_33, BRCM_OP_ADD);
			break;
		case 4:
			config_vlan_for_port(drv, port, VLAN_33, BRCM_OP_ADD);
			config_vlan_for_port(drv, port, VLAN_45, BRCM_OP_ADD);
			config_static_arl_for_port(drv, port, BRCM_OP_ADD);
			break;
		case 5:
			create_default_vlan_for_port(drv, port, VLAN_33, VLAN_PRI_0);
			config_vlan_for_port(drv, port, VLAN_33, BRCM_OP_ADD);
			break;
		default:
			break;
	}

	return count;
}
static DRIVER_ATTR_WO(phy_bcm89272_config_port_vlan_membership);

static ssize_t phy_bcm89272_delete_port_extern_vlan_membership_store(struct device_driver *drv, const char *buf, size_t count)
{
	unsigned short port = buf[0] - '0';
	if (port < 0 || port > 8) {
		return count;
	}
	switch(port) {
		case 3:
			create_default_vlan_for_port(drv, port, VLAN_1, VLAN_PRI_0);
			config_vlan_for_port(drv, port, VLAN_33, BRCM_OP_DEL);
			break;
		case 4:
			config_vlan_for_port(drv, port, VLAN_33, BRCM_OP_DEL);
			config_vlan_for_port(drv, port, VLAN_45, BRCM_OP_DEL);
			config_static_arl_for_port(drv, port, BRCM_OP_DEL);
			break;
		case 5:
			create_default_vlan_for_port(drv, port, VLAN_1, VLAN_PRI_0);
			config_vlan_for_port(drv, port, VLAN_33, BRCM_OP_DEL);
			break;
		default:
			break;
	}

	return count;
}
static DRIVER_ATTR_WO(phy_bcm89272_delete_port_extern_vlan_membership);

void psd_sleep(void)
{
	const struct brcm_reg16_config psd_sleep_cfgtab[] = {
		{0x49032654, 0x0022},
		{0x494F2054, 0x0062},
		{0x498F2054, 0x0062},
		{0x49CF2054, 0x0062},
		{0x4A0F2054, 0x0062},
		{0x4A4F2054, 0x0062},
		{0x49021068, 0x8001},
		{0x49421068, 0x8000},
		{0x49821068, 0x8000},
		{0x49c21068, 0x8000},
		{0x4a021068, 0x8000},
		{0x4a421068, 0x8000},
		{0x4a4f20fe, 0x1800},
	};
	int status, i, regcount;

	status = spidev_open_kern();
	if (status) {
		return;
	}

	regcount = sizeof(psd_sleep_cfgtab) / sizeof(struct brcm_reg16_config);
	for (i = 0; i < regcount; i++) {
		status = spi_write16(psd_sleep_cfgtab[i].addr, psd_sleep_cfgtab[i].data);
		if (status) {
			goto err_done;
		}
	}

	msleep(25);
	status = spi_write16(0x4903209e, 0x3);
	if (status) {
		goto err_done;
	}

err_done:
	spidev_release_kern();
}

static ssize_t phy_bcm89272_psd_store(struct device_driver *drv, const char *buf, size_t count)
{
	if (buf[0] == '0') {
		psd_sleep();
	} else if (buf[0] == '1') {
		printk(KERN_ERR "phy_bcm89272_psd_store only supports 0/2/3 characters.\n");
	} else if (buf[0] == '2') {
		bcm89272_suspend();
	} else if (buf[0] == '3') {
		bcm89272_resume();
	} else {
		printk(KERN_ERR "phy_bcm89272_psd_store only supports 0/2/3 characters.\n");
	}
	return count;
}
static ssize_t phy_bcm89272_psd_show(struct device_driver *drv, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", g_psd_state);
}
static DRIVER_ATTR_RW(phy_bcm89272_psd);

static ssize_t phy_bcm89272_port4_link_status_show(struct device_driver *drv, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", link_status_show_for_port(drv, 4));
}
static DRIVER_ATTR_RO(phy_bcm89272_port4_link_status);

static ssize_t phy_bcm89272_interrupt_gpio_state_show(struct device_driver *drv, char *buf)
{
	int value = -1;

	value = gpio_get_value(g_interrupt_gpio);

	if (value >= 0) {
		return scnprintf(buf, PAGE_SIZE, "interrupt gpio state: %d\n", value);
	} else {
		return scnprintf(buf, PAGE_SIZE, "error, gpio: %d, state: %d\n", g_interrupt_gpio, value);
	}
}
static DRIVER_ATTR_RO(phy_bcm89272_interrupt_gpio_state);

static ssize_t phy_bcm89272_wake_gpio_state_show(struct device_driver *drv, char *buf)
{
	int value = -1;

	value = gpio_get_value(g_wake_gpio);

	if (value >= 0) {
		return scnprintf(buf, PAGE_SIZE, "gpio state: %d\n", value);
	} else {
		return scnprintf(buf, PAGE_SIZE, "error, gpio124: state: %d\n", value);
	}
}

static ssize_t phy_bcm89272_wake_gpio_state_store(struct device_driver *drv, const char *buf, size_t count)
{
	if (buf[0] == '1') {
		gpio_set_value(g_wake_gpio, 0);
	} else if (buf[0] == '0') {
		gpio_set_value(g_wake_gpio, 1);
	}
	return count;
}
static DRIVER_ATTR_RW(phy_bcm89272_wake_gpio_state);

static ssize_t phy_bcm89272_reset_gpio_state_show(struct device_driver *drv, char *buf)
{
	int value = -1;

	value = gpio_get_value(g_reset_gpio);

	if (value >= 0) {
		return scnprintf(buf, PAGE_SIZE, "gpio state: %d\n", value);
	} else {
		return scnprintf(buf, PAGE_SIZE, "error, gpio179: state: %d\n", value);
	}
}

static ssize_t phy_bcm89272_reset_gpio_state_store(struct device_driver *drv, const char *buf, size_t count)
{
	if (buf[0] == '1') {
		gpio_set_value(g_reset_gpio, 0);
	} else if (buf[0] == '0') {
		gpio_set_value(g_reset_gpio, 1);
	}

	msleep(25);
	return count;
}
static DRIVER_ATTR_RW(phy_bcm89272_reset_gpio_state);

void bcm89272_reset_gpio_set(bool val){
	if (val)
		gpio_set_value(g_reset_gpio, 1);
	else
		gpio_set_value(g_reset_gpio, 0);
}

void bcm89272_wake_gpio_set(bool val){
	if (val)
		gpio_set_value(g_wake_gpio, 1);
	else
		gpio_set_value(g_wake_gpio, 0);
}

void bcm89272_gpio_request(struct spi_device *spi)
{
	int ret = -1;
	if (spi != NULL) {
		if (g_reset_gpio < 0) {
			g_reset_gpio = of_get_named_gpio(spi->dev.of_node, "reset-gpio", 0);
		}

		if (g_wake_gpio < 0) {
			g_wake_gpio = of_get_named_gpio(spi->dev.of_node, "wake-gpio", 0);
		}
		if (g_interrupt_gpio < 0) {
			g_interrupt_gpio = of_get_named_gpio(spi->dev.of_node, "interrupt-gpio", 0);
		}
	}

	if (g_reset_gpio < 0) {
		g_reset_gpio = 480;
		ret = gpio_request(g_reset_gpio, "reset-gpio");
		printk(KERN_ERR "gpio_request reset-gpio 480 ret = %d\n", ret);
	}

	if (g_wake_gpio < 0) {
		g_wake_gpio = 425;
		ret = gpio_request(g_wake_gpio, "wake-gpio");
		printk(KERN_ERR "gpio_request wake-gpio 425 ret = %d\n", ret);
	}


	if (g_interrupt_gpio < 0) {
		g_interrupt_gpio = 378;
		ret = gpio_request(g_interrupt_gpio, "interrupt-gpio");
		ret = gpio_direction_input(g_interrupt_gpio);
		printk(KERN_ERR "gpio_request interrupt-gpio 378 ret = %d\n", ret);
	}
}

void bcm89272_gpio_free(struct spi_device *spi)
{
	if (g_reset_gpio > 0) {
		gpio_free(g_reset_gpio);
		g_reset_gpio = 0;
	}
	if (g_wake_gpio > 0) {
		gpio_free(g_wake_gpio);
		g_wake_gpio = 0;
	}
	if (g_interrupt_gpio > 0) {
		gpio_free(g_interrupt_gpio);
		g_interrupt_gpio = 0;
	}
}

extern int msm_pcie_enumerate(u32 rc_idx);
extern int msm_pcie_deenumerate(u32 rc_idx);

static int bcm89272_suspend(void)
{
	msm_pcie_deenumerate(1);
	msleep(25);
	bcm89272_wake_gpio_set(1);
	msleep(5);
	psd_sleep();
	bcm89272_reset_gpio_set(1);
	g_psd_state = 0;
	printk(KERN_INFO "bcm89272_suspend end.\n");
	return 0;
}

static int bcm89272_resume(void)
{
	int retry = 5;
	int ret = -1;

	bcm89272_wake_gpio_set(0);
	msleep(25);
	bcm89272_reset_gpio_set(0);
	msleep(50);
	while (retry--) {
		ret = msm_pcie_enumerate(1);
		if (ret >= 0)
			break;
		else
			msleep(400);
	}
	if (ret < 0) {
		printk(KERN_INFO "bcm89272_resume ret = %d\n", ret);
		g_psd_state = -2;
	} else {
		g_psd_state = 1;
	}

	printk(KERN_INFO "bcm89272_resume end.\n");
	return 0;
}

int diagnosis_sysfs_init(struct spi_driver spidev_driver) {
	int status;

	status = driver_create_file(&(spidev_driver.driver), &driver_attr_phy_bcm89272_port0_link_status);
	if (status < 0) {
		goto err_done;
	}
	status = driver_create_file(&(spidev_driver.driver), &driver_attr_phy_bcm89272_port0_sqi_status);
	if (status < 0) {
		goto err_done;
	}
	status = driver_create_file(&(spidev_driver.driver), &driver_attr_phy_bcm89272_port0_work_mode);
	if (status < 0) {
		goto err_done;
	}
	status = driver_create_file(&(spidev_driver.driver), &driver_attr_phy_bcm89272_port3_link_status);
	if (status < 0) {
		goto err_done;
	}
	status = driver_create_file(&(spidev_driver.driver), &driver_attr_phy_bcm89272_port3_sqi_status);
	if (status < 0) {
		goto err_done;
	}
	status = driver_create_file(&(spidev_driver.driver), &driver_attr_phy_bcm89272_port3_work_mode);
		if (status < 0) {
		goto err_done;
	}
	status = driver_create_file(&(spidev_driver.driver), &driver_attr_phy_bcm89272_port0_set_test_mode);
	if (status < 0) {
		goto err_done;
	}
	status = driver_create_file(&(spidev_driver.driver), &driver_attr_phy_bcm89272_port3_set_test_mode);
	if (status < 0) {
		goto err_done;
	}
	status = driver_create_file(&(spidev_driver.driver), &driver_attr_phy_bcm89272_port5_set_test_mode);
	if (status < 0) {
		goto err_done;
	}
	status = driver_create_file(&(spidev_driver.driver), &driver_attr_phy_bcm89272_port0_dut_status);
	if (status < 0) {
		goto err_done;
	}
	status = driver_create_file(&(spidev_driver.driver), &driver_attr_phy_bcm89272_port3_dut_status);
	if (status < 0) {
		goto err_done;
	}
	status = driver_create_file(&(spidev_driver.driver), &driver_attr_phy_bcm89272_PN_polarity);
	if (status < 0) {
		goto err_done;
	}
	status = driver_create_file(&(spidev_driver.driver), &driver_attr_phy_bcm89272_loopback_state);
	if (status < 0) {
		goto err_done;
	}
	status = driver_create_file(&(spidev_driver.driver), &driver_attr_phy_bcm89272_external_loopback_state);
	if (status < 0) {
		goto err_done;
	}
	status = driver_create_file(&(spidev_driver.driver), &driver_attr_phy_bcm89272_port0_get_normal_test_mode);
	if (status < 0) {
		goto err_done;
	}
	status = driver_create_file(&(spidev_driver.driver), &driver_attr_phy_bcm89272_port3_get_normal_test_mode);
	if (status < 0) {
		goto err_done;
	}
	status = driver_create_file(&(spidev_driver.driver), &driver_attr_phy_bcm89272_port5_get_normal_test_mode);
	if (status < 0) {
		goto err_done;
	}
	status = driver_create_file(&(spidev_driver.driver), &driver_attr_phy_bcm89272_port0_get_tvco_test_mode);
	if (status < 0) {
		goto err_done;
	}
	status = driver_create_file(&(spidev_driver.driver), &driver_attr_phy_bcm89272_port3_get_tvco_test_mode);
	if (status < 0) {
		goto err_done;
	}
	status = driver_create_file(&(spidev_driver.driver), &driver_attr_phy_bcm89272_port5_get_tvco_test_mode);
	if (status < 0) {
		goto err_done;
	}
	status = driver_create_file(&(spidev_driver.driver), &driver_attr_phy_bcm89272_port0_get_ib_test_mode);
	if (status < 0) {
		goto err_done;
	}
	status = driver_create_file(&(spidev_driver.driver), &driver_attr_phy_bcm89272_enable_port3_5);
	if (status < 0) {
		goto err_done;
	}
	status = driver_create_file(&(spidev_driver.driver), &driver_attr_phy_bcm89272_config_port_vlan_membership);
	if (status < 0) {
		goto err_done;
	}
	status = driver_create_file(&(spidev_driver.driver), &driver_attr_phy_bcm89272_delete_port_extern_vlan_membership);
	if (status < 0) {
		goto err_done;
	}
	status = driver_create_file(&(spidev_driver.driver), &driver_attr_phy_bcm89272_port4_link_status);
	if (status < 0) {
		goto err_done;
	}

	status = driver_create_file(&(spidev_driver.driver), &driver_attr_phy_bcm89272_interrupt_gpio_state);
	if (status < 0) {
		status = -ENOENT;
	}

	status = driver_create_file(&(spidev_driver.driver), &driver_attr_phy_bcm89272_wake_gpio_state);
	if (status < 0) {
		status = -ENOENT;
	}

	status = driver_create_file(&(spidev_driver.driver), &driver_attr_phy_bcm89272_reset_gpio_state);
	if (status < 0) {
		status = -ENOENT;
	}

	status = driver_create_file(&(spidev_driver.driver), &driver_attr_phy_bcm89272_psd);
	if (status < 0) {
		goto err_done;
	}

	status = driver_create_file(&(spidev_driver.driver), &driver_attr_phy_bcm89272_port5_link_status);
	if (status < 0) {
		goto err_done;
	}
	status = driver_create_file(&(spidev_driver.driver), &driver_attr_phy_bcm89272_port5_sqi_status);
	if (status < 0) {
		goto err_done;
	}
	status = driver_create_file(&(spidev_driver.driver), &driver_attr_phy_bcm89272_port5_work_mode);
		if (status < 0) {
		goto err_done;
	}
	status = driver_create_file(&(spidev_driver.driver), &driver_attr_phy_bcm89272_port5_dut_status);
err_done:
	if (status < 0) {
		status = -ENOENT;
	}
	return status;
}

/*---------------------------- sysfs operation end ------------------------------*/
#endif