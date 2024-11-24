/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */
 
//#include <command.h>
//#include <common.h>
//#include <ShareType.h>
//#include <CusConfig.h>
//#include <MsVfs.h>
//#include <MsDebug.h>
//#include "usb_def.h"
//#include <MsSystem.h>
#include <stdio.h>
#include "LD_usbbt.h"
#include "LD_btmtk_usb.h"
//#include "hal_usb.h"

usb_vid_pid array_mtk_vid_pid[] = {
	{0x0E8D, 0x7668, "MTK7668"},	// 7668
	{0x0E8D, 0x76A0, "MTK7662T"},	// 7662T
	{0x0E8D, 0x76A1, "MTK7632T"},	// 7632T
	{0x0E8D, 0x7663, "MTK7663"},	//7663
	{0x0E8D, 0x7961, "MTK7961"},	//7961
};

int max_mtk_wifi_id = (sizeof(array_mtk_vid_pid) / sizeof(array_mtk_vid_pid[0]));
usb_vid_pid *pmtk_wifi = &array_mtk_vid_pid[0];

static mtkbt_dev_t *g_DrvData = NULL;

extern void LDR_Mount(void);
extern UINT32 FAT_getsize(const char* filename);
extern UINT8 FAT_Read(const char* filename, char *buffer,UINT32 filesize);

VOID *os_memcpy(VOID *dst, const VOID *src, UINT32 len)
{
	return x_memcpy(dst, src, len);
}

VOID *os_memmove(VOID *dest, const void *src,UINT32 len)
{
	return x_memcpy(dest, src, len);
}

VOID *os_memset(VOID *s, int c, size_t n)
{
	return x_memset(s,c,n);
}

VOID *os_kzalloc(size_t size, unsigned int flags)
{
	VOID *ptr = x_mem_alloc(size);

	os_memset(ptr, 0, size);
	return ptr;
}

void LD_load_code_from_bin(unsigned char **image, char *bin_name, char *path, mtkbt_dev_t *dev, u32 *code_len)
{
	int size;

	size = FAT_getsize(bin_name);
	if (size == -1){
		usb_debug("Get file size fail\n");
		return;
	}

	*code_len = size;
	*image = x_mem_alloc(size);
	FAT_Read(bin_name, (char *)(*image),size);
	return;
}

static int usb_bt_bulk_msg(
		mtkbt_dev_t *dev,
		u32 epType,
		u8 *data,
		int size,
		int* realsize,
		int timeout /* not used */
)
{
	int ret =0 ;
	if(dev == NULL || dev->idev == NULL || dev->bulk_tx_ep ==  NULL)
	{
		usb_debug("bulk out error 00\n");
		return -1;
	}

	//usb_debug("[usb_bt_bulk_msg]ep_addr:%x\n", dev->bulk_tx_ep->bEndpointAddress);
	//usb_debug("[usb_bt_bulk_msg]ep_maxpkt:%x\n", dev->bulk_tx_ep->wMaxPacketSize);

	if(epType == MTKBT_BULK_TX_EP)
	{
		ret = dev->idev->controller->bulk(dev->bulk_tx_ep, size, data,0);
		*realsize = ret;

		if(ret<0)
		{
			usb_debug("bulk out error 01\n");
			return -1;
		}

		if(*realsize == size)
		{
			//usb_debug("bulk out success 01,size =0x%x\n",size);
			return 0;
		}
		else
		{
			usb_debug("bulk out fail 02,size =0x%x,realsize =0x%x\n",size,*realsize);
		}
	}
	return -1;
}

static int usb_bt_control_msg(
		mtkbt_dev_t *dev,
		u32 epType,
		u8 request,
		u8 requesttype,
		u16 value,
		u16 index,
		u8 *data,
		int data_length,
		int timeout  /* not used */
)
{
	int ret = -1;
	dev_req_t dr;
    dr.bmRequestType = requesttype;
	dr.bRequest = request;
	dr.wValue = value;
	dr.wIndex = index;
	dr.wLength = data_length;
	if(epType == MTKBT_CTRL_TX_EP)
	{
		ret = dev->idev->controller->control(dev->idev, OUT, sizeof (dr), &dr, data_length, data);
	}
	else if (epType == MTKBT_CTRL_RX_EP)
	{
		ret = dev->idev->controller->control(dev->idev, IN, sizeof (dr), &dr, data_length, data);
	}
	else
	{
		usb_debug("control message wrong Type =0x%x\n",epType);
	}

	if (ret < 0)
	{
		usb_debug("Err1(%d)\n", ret);
		return ret;
	}
	return ret;
}

static int usb_bt_interrupt_msg(
		mtkbt_dev_t *dev,
		u32 epType,
		u8 *data,
		int size,
		int* realsize,
		int timeout  /* unit of 1ms */
)
{
	int ret = -1;

	usb_debug("epType = 0x%x\n",epType);

	if(epType == MTKBT_INTR_EP)
	{
		ret = dev->idev->controller->intr(dev->intr_ep, size, realsize,data, 2000);
	}

	usb_debug("realsize=%d reqdata 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n",*realsize,data[0],data[1],data[2],data[3],data[4],data[5]);

	if(ret < 0 )
	{
		usb_debug("Err1(%d)\n", ret);
		return ret;
	}
	//usb_debug("ret = 0x%x\n",ret);
	return ret;
}

static HC_IF usbbt_host_interface =
{
		usb_bt_bulk_msg,
		usb_bt_control_msg,
		usb_bt_interrupt_msg,
};

static void Ldbtusb_diconnect (btusbdev_t *dev)
{
	LD_btmtk_usb_disconnect(g_DrvData);

	if(g_DrvData)
	{
		os_kfree(g_DrvData);
	}
	g_DrvData = NULL;
}

static int Ldbtusb_SetWoble(btusbdev_t *dev)
{
	if(!g_DrvData)
	{
		usb_debug("usb set woble fail ,because no drv data\n");
		return -1;
	}
	else
	{
		LD_btmtk_usb_SetWoble(g_DrvData);
		usb_debug("usb set woble end\n");
	}
	return 0;
}

int Ldbtusb_connect (btusbdev_t *dev, int flag)
{
	int ret = 0;

	// For Mstar
	//struct usb_endpoint_descriptor *ep_desc;
	//struct usb_interface *iface;
	int i;
	//iface = &dev->config.if_desc[0];

	if(g_DrvData == NULL)
	{
		g_DrvData = os_kmalloc(sizeof(mtkbt_dev_t),MTK_GFP_ATOMIC);

		if(!g_DrvData)
		{
			usb_debug("Not enough memory for mtkbt virtual usb device.\n");
			return -1;
		}
		else
		{
			os_memset(g_DrvData,0,sizeof(mtkbt_dev_t));
			g_DrvData->idev = dev;
			g_DrvData->connect = Ldbtusb_connect;
			g_DrvData->disconnect = Ldbtusb_diconnect;
			g_DrvData->SetWoble = Ldbtusb_SetWoble;
		}
	}
	else
	{
			return -1;
	}

	for (i = 1; i <= dev->num_endp; i++) {
		usb_debug("dev->endpoints[%d].type = %d\n", i, dev->endpoints[i].type);
		usb_debug("dev->endpoints[%d].endpoint = %d\n", i, dev->endpoints[i].endpoint);
		usb_debug("dev->endpoints[%d].direction = %d\n", i, dev->endpoints[i].direction);
		if (dev->endpoints[i].type == BULK)
		{
			if (dev->endpoints[i].direction == IN)
			{
				g_DrvData->bulk_rx_ep = &dev->endpoints[i];
			}
			else if (dev->endpoints[i].direction == OUT &&
				dev->endpoints[i].endpoint != 0x01)
			{
				g_DrvData->bulk_tx_ep = &dev->endpoints[i];
			}
			continue;
		}
		if (dev->endpoints[i].type == INTERRUPT &&
			dev->endpoints[i].endpoint != 0x8f)
		{
			g_DrvData->intr_ep = &dev->endpoints[i];
			continue;
		}
	}

	if (!g_DrvData->intr_ep || !g_DrvData->bulk_tx_ep || !g_DrvData->bulk_rx_ep)
	{
		os_kfree(g_DrvData);
		g_DrvData = NULL;
		usb_debug("btmtk_usb_probe end Error 3\n");
		return -1;
	}

	/* Init HostController interface */
	g_DrvData->hci_if = &usbbt_host_interface;

	/* btmtk init */
	ret = LD_btmtk_usb_probe(g_DrvData, flag);

	if (ret != 0)
	{
		usb_debug("usb probe fail\n");
		if(g_DrvData)
		{
		   os_kfree(g_DrvData);
		}
		g_DrvData = NULL;
		return -1;
	}
	else
	{
		usb_debug("usbbt probe success\n");
	}
	return ret;
}

u8 LDbtusb_getWoBTW(void)
{
	return LD_btmtk_usb_getWoBTW();
}
#if 0
static int checkUsbDevicePort(struct usb_device* udev, u16 vendorID, u16 productID, u8 port)
{
	struct usb_device* pdev = NULL;
/*#if defined (CONFIG_USB_PREINIT)
	usb_stop(port);
	if (usb_post_init(port) == 0)
#else
	if (usb_init(port) == 0)
#endif*/
	#if 0
	{
		/* get device */
		//pdev = usb_get_dev_index(0);
		if ((pdev != NULL) && (pdev->descriptor.idVendor == vendorID) && (pdev->descriptor.idProduct == productID))  // MTK 7662
		{
			Printf("OK\n");
			x_memcpy(udev, pdev, sizeof(struct usb_device));
			return 0 ;
		}
	}
	#endif
	return -1;
}
#endif
#if 0
static int findUsbDevice(struct usb_device* udev)
{
	int ret = -1;
	u8 idx = 0;
	u8 i = 0;
	char portNumStr[10] = "\0";
	char* pBTUsbPort = NULL;
	Printf("IN\n");
	if(udev == NULL)
	{
		Printf("udev can not be NULL\n");
		return -1;
	}
    //use the usb poll function replace----lining
    //keys add:find usb port idx
	/*#define BT_USB_PORT "bt_usb_port"
	pBTUsbPort = getenv(BT_USB_PORT);
	if(pBTUsbPort != NULL)
	{
		i = 0;
		// search mtk bt usb port
		idx = atoi(pBTUsbPort);
		usb_debug("find mtk bt usb device from usb prot[%d]\n", idx);
		while (i < max_mtk_wifi_id) {
			ret = checkUsbDevicePort(udev, (pmtk_wifi + i)->vid, (pmtk_wifi + i)->pid, idx);
			if (ret == 0) break;
			i++;
			#if defined(NEW_RC_CON) && (NEW_RC_CON == TRUE)
			usb_debug("fengchen 7668 error");
			return -1;
			#endif
		}
		if(ret == 0)
		{
			return 0;
		}
	}*/
     //keys add:find usb port idx end!!!
	// not find mt bt usb device from given usb port, so poll every usb port.
	/*#if defined(ENABLE_FIFTH_EHC)
	const char u8UsbPortCount = 5;
	#elif defined(ENABLE_FOURTH_EHC)
	const char u8UsbPortCount = 4;
	#elif defined(ENABLE_THIRD_EHC)
	const char u8UsbPortCount = 3;
	#elif defined(ENABLE_SECOND_EHC)
	const char u8UsbPortCount = 2;
	#else
	const char u8UsbPortCount = 1;
	#endif
	for(idx = 0; idx < u8UsbPortCount; idx++)
	{
		i = 0;
		while (i < max_mtk_wifi_id) {
			ret = checkUsbDevicePort(udev, (pmtk_wifi + i)->vid, (pmtk_wifi + i)->pid, idx);
			if (ret == 0) break;
			i++;
		}
		if(ret == 0)
		{
			// set bt_usb_port to store mt bt usb device port
			snprintf(portNumStr, sizeof(portNumStr), "%d", idx);
			setenv(BT_USB_PORT, portNumStr);
			saveenv();
			return 0;
		}
	}
	if(pBTUsbPort != NULL)
	{
		// env BT_USB_PORT is involid, so delete it
		setenv(BT_USB_PORT, NULL);
		saveenv();
	}*/
	Printf("Not find usb device\n");
	return -1;
}
#endif

void do_setMtkBT(usbdev_t *dev)
{
	int ret = 0;
	Printf("IN\n");
	LDR_Mount();  //16566
	// MTK USB controller
	/*ret = findUsbDevice(&udev);
	if (ret != 0)
	{
		Printf("find bt usb device failed\n");
		return -1;
	}*/
	ret = Ldbtusb_connect(dev,0);
	if(ret != 0){
		Printf("connect to bt usb device failed\n");
		return;
	}
	ret = Ldbtusb_SetWoble(NULL);
	if(ret != 0)
	{
		Printf("set bt usb device woble cmd failed\n");
		return;
	}
	Printf("OK\n");
}

int getMtkBTWakeT(void)
{
	int ret = 0;
	#if 0
	struct usb_device udev;
	memset(&udev, 0, sizeof(struct usb_device));
	Printf("IN\n");
	// MTK USB controller
	ret = findUsbDevice(&udev);
	if (ret != 0)
	{
		Printf("find bt usb device failed\n");
		return -1;
	}
	ret = Ldbtusb_connect(&udev, 1);
	if(ret != 0)
	{
		Printf("connect to bt usb device failed\n");
		return -1;
	}

	if(ret != 0)
	{
		Printf("set bt usb device woble cmd failed\n");
		return -1;
	}
	Printf("OK\n");
	#endif
	return ret;
}

