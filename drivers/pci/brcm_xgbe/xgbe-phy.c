/*
 * Broadcom BCM8956X / BCM8957X / BCM8989X 10Gb Ethernet driver
 *
 * Copyright (c) 2023 Broadcom. The term "Broadcom" refers solely to the 
 * Broadcom Inc. subsidiary that distributes the Licensed Product, as defined 
 * below.
 *
 * The following copyright statements and licenses apply to open source software 
 * ("OSS") distributed with the Broadcom Product (the "Licensed Product").
 * The Licensed Product does not necessarily use all the OSS referred to below and 
 * may also only use portions of a given OSS component. 
 *
 * To the extent required under an applicable open source license, Broadcom 
 * will make source code available for applicable OSS upon request. Please send 
 * an inquiry to opensource@broadcom.com including your name, address, the 
 * product name and version, operating system, and the place of purchase.   
 *
 * To the extent the Licensed Product includes OSS, the OSS is typically not 
 * owned by Broadcom. THE OSS IS PROVIDED AS IS WITHOUT WARRANTY OR CONDITION 
 * OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING, WITHOUT LIMITATION, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  
 * To the full extent permitted under applicable law, Broadcom disclaims all 
 * warranties and liability arising from or related to any use of the OSS.
 *
 * To the extent the Licensed Product includes OSS licensed under the GNU 
 * General Public License ("GPL") or the GNU Lesser General Public License 
 * ("LGPL"), the use, copying, distribution and modification of the GPL OSS or 
 * LGPL OSS is governed, respectively, by the GPL or LGPL.  A copy of the GPL 
 * or LGPL license may be found with the applicable OSS.  Additionally, a copy 
 * of the GPL License or LGPL License can be found at 
 * https://www.gnu.org/licenses or obtained by writing to the Free Software 
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 * 
 * This file is available to you under your choice of the following two 
 * licenses:
 *
 * License 1: GPLv2 License
 *
 * Copyright (c) 2023 Broadcom
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * License 2: Modified BSD License
 * 
 * Copyright (c) 2023 Broadcom
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Advanced Micro Devices, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * AMD 10Gb Ethernet driver
 *
 * This file is available to you under your choice of the following two
 * licenses:
 *
 * License 1: GPLv2
 *
 * Copyright (c) 2014-2016 Advanced Micro Devices, Inc.
 *
 * This file is free software; you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or (at
 * your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * This file incorporates work covered by the following copyright and
 * permission notice:
 *     The Synopsys DWC ETHER XGMAC Software Driver and documentation
 *     (hereinafter "Software") is an unsupported proprietary work of Synopsys,
 *     Inc. unless otherwise expressly agreed to in writing between Synopsys
 *     and you.
 *
 *     The Software IS NOT an item of Licensed Software or Licensed Product
 *     under any End User Software License Agreement or Agreement for Licensed
 *     Product with Synopsys or any supplement thereto.  Permission is hereby
 *     granted, free of charge, to any person obtaining a copy of this software
 *     annotated with this license and the Software, to deal in the Software
 *     without restriction, including without limitation the rights to use,
 *     copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 *     of the Software, and to permit persons to whom the Software is furnished
 *     to do so, subject to the following conditions:
 *
 *     The above copyright notice and this permission notice shall be included
 *     in all copies or substantial portions of the Software.
 *
 *     THIS SOFTWARE IS BEING DISTRIBUTED BY SYNOPSYS SOLELY ON AN "AS IS"
 *     BASIS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 *     TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 *     PARTICULAR PURPOSE ARE HEREBY DISCLAIMED. IN NO EVENT SHALL SYNOPSYS
 *     BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *     CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *     SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *     INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *     CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *     ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 *     THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * License 2: Modified BSD
 *
 * Copyright (c) 2014-2016 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Advanced Micro Devices, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * This file incorporates work covered by the following copyright and
 * permission notice:
 *     The Synopsys DWC ETHER XGMAC Software Driver and documentation
 *     (hereinafter "Software") is an unsupported proprietary work of Synopsys,
 *     Inc. unless otherwise expressly agreed to in writing between Synopsys
 *     and you.
 *
 *     The Software IS NOT an item of Licensed Software or Licensed Product
 *     under any End User Software License Agreement or Agreement for Licensed
 *     Product with Synopsys or any supplement thereto.  Permission is hereby
 *     granted, free of charge, to any person obtaining a copy of this software
 *     annotated with this license and the Software, to deal in the Software
 *     without restriction, including without limitation the rights to use,
 *     copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 *     of the Software, and to permit persons to whom the Software is furnished
 *     to do so, subject to the following conditions:
 *
 *     The above copyright notice and this permission notice shall be included
 *     in all copies or substantial portions of the Software.
 *
 *     THIS SOFTWARE IS BEING DISTRIBUTED BY SYNOPSYS SOLELY ON AN "AS IS"
 *     BASIS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 *     TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 *     PARTICULAR PURPOSE ARE HEREBY DISCLAIMED. IN NO EVENT SHALL SYNOPSYS
 *     BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *     CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *     SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *     INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *     CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *     ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 *     THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/kmod.h>
#include <linux/mdio.h>
#include <linux/phy.h>
#include <linux/pci.h>

#include "xgbe.h"
#include "xgbe-common.h"
#if XGBE_SRIOV_PF

#define ARM_CORE_CTL_REG                 0xf0003000

#define PHY_MDIO_REG_ADDR(x , y) (0xf2000000 | ((x) << 17) | ((y << 1)))

#define	OTP_CPU_COMMAND		PHY_MDIO_REG_ADDR(0x1E, 0x8400)
#define	OTP_CPU_WRDATA_H	PHY_MDIO_REG_ADDR(0x1E, 0x8401)
#define	OTP_CPU_WRDATA_L	PHY_MDIO_REG_ADDR(0x1E, 0x8402)
#define	OTP_CONFIG			PHY_MDIO_REG_ADDR(0x1E, 0x8403)
#define	OTP_ADDRESS			PHY_MDIO_REG_ADDR(0x1E, 0x8404)
#define	OTP_STATUS_1		PHY_MDIO_REG_ADDR(0x1E, 0x8405)
#define	OTP_STATUS_0		PHY_MDIO_REG_ADDR(0x1E, 0x8406)
#define	OTP_CPU_RDDATA_H	PHY_MDIO_REG_ADDR(0x1E, 0x8408)
#define	OTP_CPU_RDDATA_L	PHY_MDIO_REG_ADDR(0x1E, 0x8409)

static char eiger_version[4][3] = {"A0\0", "B0\0", "B1\0", "UR\0"};

static int pcie_reg_read( struct xgbe_prv_data* pdata, unsigned int addr, int len, unsigned int* val)
{
	int ret = 0;
    unsigned int ctrl, reg_value;
    volatile int done;
    int timeout = 1000;
    volatile void* virt_addr = (void*)pdata->phy_regs;

    switch(len) {
	    case 1:
            ctrl = 0x10;
            break;
        case 4:
            ctrl = 0x12;
            break;
        default:
        case 2:
            ctrl = 0x11;
            break;
    }

    *((unsigned int *)(virt_addr+4)) = addr;
    wmb();
    *((unsigned int *) virt_addr) = ctrl;
    wmb();
    done = *((unsigned int*)virt_addr);
    rmb();
    if( done==0 ) {
		netif_err(pdata, link, pdata->netdev, "PHY REG Read Error: No HW Present \n");
		ret = -1;
		goto no_hw_avialable;
    }

    done &= 0x80;
    //XGBE_USLEEP_RANGE(10, 30);
	*val = 0;

    while( !done && (--timeout)>0 ) {
      done = (*((unsigned int *) virt_addr) & 0x80);
      rmb();
    }

    if(timeout) {
	    reg_value = *((unsigned int *) (virt_addr+8));
	    switch(len) {
			case 1:
				*val = reg_value & 0xff;
			break;
			case 4:
				*val = reg_value;
			break;
			default:
			case 2:
				*val = reg_value & 0xffff;
			break;
	    }
    } else {
		netif_err(pdata, link, pdata->netdev, "PHY REG Read Error: Failed for addr=0x%x \n",addr);
		ret = -2;
    }
no_hw_avialable:
    return ret;
}

static int pcie_reg_write(struct xgbe_prv_data* pdata, unsigned int addr, unsigned int val, int len )
{
	int ret = 0;
    unsigned int ctrl, reg_value;
    volatile int done;
    int timeout = 1000;
    volatile void* virt_addr = (void*)pdata->phy_regs;

    switch(len) {
        case 1:
            ctrl = 0x14;
			reg_value = val & 0xff;
        break;
        case 4:
            ctrl = 0x16;
			reg_value = val;
        break;
        default:
		case 2:
            ctrl = 0x15;
			reg_value = val & 0xffff;
        break;
    }

    *((unsigned int *) (virt_addr+4)) = addr;
    wmb();
    *((unsigned int *) (virt_addr+8)) = reg_value;
    wmb();
    *((unsigned int *) virt_addr) = ctrl;
    wmb();
    done = *((unsigned int*)virt_addr);
    rmb();
    if( done==0 ) {
		ret = -1;
		netif_err(pdata, link, pdata->netdev, "PHY REG Write Error: No HW Present \n");
		goto skip_to_end;
	}
    done &= 0x80;
    //XGBE_USLEEP_RANGE(10, 30);

    while( !done && (--timeout)>0 ) {
      done = (*((unsigned int *) virt_addr) & 0x80);
      rmb();
    }
    if(timeout == 0) {
		netif_err(pdata, link, pdata->netdev, "PHY REG Write Error: Failed for addr=0x%x val=0x%x\n",addr,val);
		ret = -2;
	}
skip_to_end:
    return ret;
}

static int xgbe_otp_reg_read(struct xgbe_prv_data *pdata, unsigned int otp_reg, unsigned int* otp_value)
{
    if((pdata->dev_id == BCM8989X_PF_ID) && (pdata->bcm8989x_b0)) {
		unsigned int status = 0;
		unsigned int val_h, val_l;
        spin_lock(&pdata->phy_access_lock);
		pcie_reg_write(pdata, OTP_CONFIG, 2, 2);
		pcie_reg_write(pdata, OTP_ADDRESS, otp_reg, 2);
		pcie_reg_write(pdata, OTP_CPU_COMMAND, 0, 2);
		pcie_reg_write(pdata, OTP_CONFIG, 6, 2);
		pcie_reg_read(pdata, OTP_STATUS_0, 2, (unsigned int *)&status);
		pcie_reg_read(pdata,  OTP_CPU_RDDATA_H, 2, &val_h);
		pcie_reg_read(pdata,  OTP_CPU_RDDATA_L, 2, &val_l);
		*otp_value = ((val_h << 16) | val_l);
        spin_unlock(&pdata->phy_access_lock);
    }
    return 0;
}

static int xgbe_otp_reg_write(struct xgbe_prv_data *pdata, unsigned int otp_reg, unsigned int otp_value)
{
    if((pdata->dev_id == BCM8989X_PF_ID) && (pdata->bcm8989x_b0)) {
        spin_lock(&pdata->phy_access_lock);
		pcie_reg_write(pdata, OTP_CPU_COMMAND, 0xA, 2);
		pcie_reg_write(pdata, OTP_ADDRESS, otp_reg, 2);
		pcie_reg_write(pdata, OTP_CPU_WRDATA_H, otp_value >> 16, 2);
		pcie_reg_write(pdata, OTP_CPU_WRDATA_L, otp_value & 0xffff, 2);
		pcie_reg_write(pdata, OTP_CONFIG, 6, 2);
        spin_unlock(&pdata->phy_access_lock);
    }
    return 0;
}

static int xgbe_otp_prog_enable(struct xgbe_prv_data *pdata, unsigned int enable)
{
    if((pdata->dev_id == BCM8989X_PF_ID) && (pdata->bcm8989x_b0)) {
        spin_lock(&pdata->phy_access_lock);
		if(enable) {
			unsigned int status = 0;
			pcie_reg_write(pdata, OTP_CONFIG, 2, 2);
			pcie_reg_write(pdata, OTP_CPU_COMMAND, 2, 2);
			pcie_reg_write(pdata, OTP_CPU_WRDATA_L, 0xF, 2);
			pcie_reg_write(pdata, OTP_CONFIG, 6, 2);
			pcie_reg_write(pdata, OTP_CPU_WRDATA_L, 0x4, 2);
			pcie_reg_write(pdata, OTP_CONFIG, 6, 2);
			pcie_reg_write(pdata, OTP_CPU_WRDATA_L, 0x8, 2);
			pcie_reg_write(pdata, OTP_CONFIG, 6, 2);
			pcie_reg_write(pdata, OTP_CPU_WRDATA_L, 0xD, 2);
			pcie_reg_write(pdata, OTP_CONFIG, 6, 2);
			pcie_reg_read(pdata,  OTP_STATUS_0, 2, &status);
		} else {
			pcie_reg_write(pdata, OTP_CPU_COMMAND, 3, 2);
			pcie_reg_write(pdata, OTP_CONFIG, 6, 2);

			pcie_reg_write(pdata, OTP_CPU_COMMAND, 0xD, 2);
			pcie_reg_write(pdata, OTP_CONFIG, 6, 2);

		}
        spin_unlock(&pdata->phy_access_lock);
    }
    return 0;
}

static void xgbe_phy_exit(struct xgbe_prv_data *pdata)
{
}

static void xgbe_phy_set_autoneg_advertise(struct xgbe_prv_data *pdata, struct ethtool_link_ksettings *dlks)
{
	unsigned int advertised = 0;
	volatile unsigned int reg_value = 0;
	volatile int ret;
	struct ethtool_link_ksettings *slks = &pdata->phy.lks;

    if (ethtool_link_ksettings_test_link_mode(dlks, advertising, 2500baseT_Full))
            advertised |= 0x1;
    if (ethtool_link_ksettings_test_link_mode(dlks, advertising, 5000baseT_Full))
            advertised |= 0x2;
    if (ethtool_link_ksettings_test_link_mode(dlks, advertising, 10000baseT_Full))
            advertised |= 0x4;

	XGBE_LM_COPY(slks, advertising, dlks, advertising);

	spin_lock(&pdata->phy_access_lock);

    ret = pcie_reg_read(pdata, PHY_MDIO_REG_ADDR(0x7, 0x203), 2, (unsigned int *)&reg_value);

	if(!ret) {
		// Set Advertise Registers with 2.5G/5G/10G Speeds
		reg_value &= 0xf8ff;
		reg_value |= (advertised << 8);
		pcie_reg_write(pdata, PHY_MDIO_REG_ADDR(0x7, 0x203), reg_value, 2);
        pcie_reg_read(pdata, PHY_MDIO_REG_ADDR(0x7, 0x202), 2, (unsigned int *)&reg_value);
        pcie_reg_write(pdata, PHY_MDIO_REG_ADDR(0x7, 0x202), reg_value, 2);
	}

	// Enable Auto negotiation and restart
	if(pdata->phy.autoneg) {
		pcie_reg_write(pdata, PHY_MDIO_REG_ADDR(0x7, 0x200), 0x1200, 2);
	}

	spin_unlock(&pdata->phy_access_lock);

	return;
}

static int xgbe_phy_init(struct xgbe_prv_data *pdata)
{
	volatile unsigned int reg_value;
	volatile int ret;
	struct ethtool_link_ksettings *lks = &pdata->phy.lks;
#if XGBE_SRIOV_PF
	if(pdata->dev_id == BCM8989X_PF_ID) {
		spin_lock_init(&pdata->phy_access_lock);

		XGBE_ZERO_SUP(lks);
		pdata->phy.speed = SPEED_UNKNOWN;
		pdata->phy.duplex = DUPLEX_UNKNOWN;
		pdata->phy.autoneg = AUTONEG_ENABLE;
		pdata->phy.pause_autoneg = AUTONEG_ENABLE;

		XGBE_SET_SUP(lks, Autoneg);
		XGBE_SET_SUP(lks, Pause);
		XGBE_SET_SUP(lks, MII);
		XGBE_SET_SUP(lks, 2500baseT_Full);
		XGBE_SET_SUP(lks, 5000baseT_Full);
		XGBE_SET_SUP(lks, 10000baseT_Full);
		XGBE_LM_COPY(lks, advertising, lks, supported);
		spin_lock(&pdata->phy_access_lock);
		ret = pcie_reg_read(pdata, PHY_MDIO_REG_ADDR(1, 3), 2, (unsigned int *)&reg_value);
		if(!ret) {
			if(reg_value & 0x3) {
				pdata->bcm8989x_b0 = (reg_value & 3);  //BCM8989X B0,B1 
			}
			printk("Detected BCM8989X %s Chip \n",eiger_version[pdata->bcm8989x_b0]);
		} else {
			printk("Failed to Read BCM8989X CHIP REV\n");
		}

		ret = pcie_reg_read(pdata, PHY_MDIO_REG_ADDR(0x1E, 0x8129), 2, (unsigned int *)&reg_value);
		printk("BCM8989X-%s Firmware Rev = 0x%x \n", eiger_version[pdata->bcm8989x_b0],reg_value);

#if 0		// Commented out as new firmware is resetting the PCIe block as well
		//Reset Port
		ret = pcie_reg_read(pdata, XGMAC_PHY_IEEE_CTL1, 2, (unsigned int *)&reg_value);
		if(!ret) {
			reg_value |= 0x8000;
			pcie_reg_write(pdata, XGMAC_PHY_IEEE_CTL1, reg_value, 2);
		}
#endif
		spin_unlock(&pdata->phy_access_lock);

		xgbe_phy_set_autoneg_advertise(pdata, lks);
	} else {
        u16 dev_link_sts;
		pdata->phy_link = 1;
		pdata->phy.link = 1;
        pdata->phy.speed = SPEED_1000;
	    pdata->phy.duplex = DUPLEX_FULL;
        pcie_capability_read_word(pdata->pcidev, PCI_EXP_LNKSTA, &dev_link_sts);
        if((dev_link_sts & PCI_EXP_LNKSTA_CLS) == PCI_EXP_LNKSTA_CLS_2_5GB) pdata->phy.speed = SPEED_2500;
        if((dev_link_sts & PCI_EXP_LNKSTA_CLS) == PCI_EXP_LNKSTA_CLS_5_0GB) pdata->phy.speed = SPEED_5000;
	}
#endif
	return 0;
}

static int xgbe_phy_link_status(struct xgbe_prv_data *pdata, int *an_restart)
{
	volatile unsigned int reg_value;
	volatile int ret;

	spin_lock(&pdata->phy_access_lock);
	ret = pcie_reg_read( pdata, PHY_MDIO_REG_ADDR(0x1E, 0x8126), 2, (unsigned int *)&reg_value);
	spin_unlock(&pdata->phy_access_lock);

	/* Link status is latched low, so read once to clear
	 * and then read again to get current state
	 */
	if(!ret  && (reg_value & 0x01)) {
		//Read Success Full and Link Status is UP
		pdata->phy.link = 1;
		netif_carrier_on(pdata->netdev);
		// 0 - 10G , 1 - 5G , 2 - 2.5G
		pdata->phy.speed = ((reg_value & 0x0E) >> 1);
		pdata->phy.duplex = (reg_value  >> 5) & 1;
		pdata->phy.autoneg = (reg_value  >> 4) & 1;
		return 1;
	} else {
		if(!ret) {
			pdata->phy.link = 0;
			pdata->phy.speed = SPEED_UNKNOWN;
			pdata->phy.duplex = DUPLEX_UNKNOWN;
			netif_carrier_off(pdata->netdev);
		}
	}

	return 0;
}

static enum xgbe_mode xgbe_phy_get_mode(struct xgbe_prv_data *pdata,
					int speed)
{
	volatile unsigned int reg_value = 0;
	volatile int ret;
	spin_lock(&pdata->phy_access_lock);
	ret = pcie_reg_read( pdata, PHY_MDIO_REG_ADDR(0x1E, 0x8126), 2, (unsigned int *)&reg_value);
	spin_unlock(&pdata->phy_access_lock);

	/* Get link speed data */
	if(!ret && (reg_value & 1)) {
		 pdata->phy.speed = ((reg_value & 0x0E) >> 1);
		 reg_value = pdata->phy.speed;
    } else {
		reg_value = 0;
	}

	return reg_value;
}

static bool xgbe_phy_valid_speed(struct xgbe_prv_data *pdata, int speed)
{
#if XGBE_SRIOV_PF
	if(pdata->dev_id == BCM8989X_PF_ID) {
		if((speed != SPEED_10000) && (speed != SPEED_5000) && (speed != SPEED_2500)) return false;
		else return true;
	} else {
		if(speed != SPEED_1000) return false;
		else return true;
	}
#else
	// For VF Return True always
	return true;
#endif
}
void xgbe_phy_flash_firmware(struct xgbe_prv_data *pdata, int region, char* buffer, int size)
{
    volatile void* virt_addr_tcm = (void*)(pdata->phy_regs - (pdata->bar2_size - XGMAC_PHY_REGS_OFFSET_FROM_END));
	volatile unsigned int reg_value;
	volatile int ret;
	int count, i;
	int fail = 0;
#if XGBE_SRIOV_PF
	if(pdata->dev_id == BCM8989X_PF_ID) {
		spin_lock(&pdata->phy_access_lock);
		ret = pcie_reg_read(pdata, PHY_MDIO_REG_ADDR(0x1E, 0x8129), 2, (unsigned int *)&reg_value);
		printk("BCM8989X-%s Current Running Firmware is Rev = 0x%x \n", eiger_version[pdata->bcm8989x_b0], reg_value);
		printk("Starting BCM8989X-%s Firmware %s Process .... \n", eiger_version[pdata->bcm8989x_b0], region? "ITCM Download": "Flashing");
		if(pdata->bcm8989x_b0 == 0) {
			if(region == 0) {
				//reset and halt CPU
				printk("\t Reset and Halt CPU .... \n");
				pcie_reg_write( pdata, ARM_CORE_CTL_REG, 0x823, 4);

				//Soft Reset Port
				pcie_reg_write( pdata, PHY_MDIO_REG_ADDR(0x1E, 0x8007), 0x4, 2);
				pcie_reg_write( pdata, PHY_MDIO_REG_ADDR(0x1E, 0x800A), 0x0, 2);
				pcie_reg_write( pdata, PHY_MDIO_REG_ADDR(0x1E, 0x8098), 0x0, 2);
				pcie_reg_write( pdata, PHY_MDIO_REG_ADDR(0x1E, 0x812B), 0x0, 2);
				pcie_reg_write( pdata, PHY_MDIO_REG_ADDR(0x1E, 0x80D6), 0x0, 2);

				pcie_reg_write( pdata, PHY_MDIO_REG_ADDR(1, 0xA819), 0x0, 2);
				pcie_reg_write( pdata, PHY_MDIO_REG_ADDR(1, 0xA81A), 0x0, 2);
				pcie_reg_write( pdata, PHY_MDIO_REG_ADDR(1, 0xA817), 0x38, 2);

				printk("\t Copying Firmware to ITCM.... \n");
				for(i=0; i<size; i+=1) {
					*((char *)(virt_addr_tcm + i)) = *((char *)(buffer + i));
				}

				pcie_reg_write( pdata, PHY_MDIO_REG_ADDR(1, 0xA817), 0x0, 2);

				printk("\t Bringing CPU out of Halt State.... \n");

				//Bring CPU out of halt state
				pcie_reg_write( pdata, ARM_CORE_CTL_REG, 0x802, 2);

				printk("\t Waiting for SPI ROM Loader to Complete.... \n");
				// Wait For SPI ROM Loader to Complete
				count = 30;
				ret = pcie_reg_read(pdata, PHY_MDIO_REG_ADDR(0x1E, 0x812B), 2, (unsigned int *)&reg_value);
				while(!ret && (reg_value != 0x600D) && (count > 0)) {
					XGBE_MSLEEP(1000);
					ret = pcie_reg_read(pdata, PHY_MDIO_REG_ADDR(0x1E, 0x812B), 2, (unsigned int *)&reg_value);
					printk("SPI ROM Status (%d) is 0x%x \n", count, reg_value);
					if(reg_value == 0xdead) {
						printk("SPI ROM Flashing Failed with DEAD \n");
						fail = 1;
						break;
					}
					count--;
				}
				if(count == 0) {
					printk("SPI ROM Loading Failed in 30 seconds \n");
					fail = 1;
				}

				if(fail == 0) {
					//Reset and Restart the System
					pcie_reg_write( pdata, PHY_MDIO_REG_ADDR(0x1E, 0x80D6), 0x2, 2);

					//pcie_reg_write( pdata, ARM_CORE_CTL_REG, 0x23, 2);
					// Replaced above line with mGigAuto API Code
					pcie_reg_write( pdata, PHY_MDIO_REG_ADDR(1, 0xA819), 0x3000, 2);
					pcie_reg_write( pdata, PHY_MDIO_REG_ADDR(1, 0xA81A), 0xf000, 2);
					pcie_reg_write( pdata, PHY_MDIO_REG_ADDR(1, 0xA81B), 0x23, 2);
					pcie_reg_write( pdata, PHY_MDIO_REG_ADDR(1, 0xA81C), 0x0, 2);
					pcie_reg_write( pdata, PHY_MDIO_REG_ADDR(1, 0xA817), 0x9, 2);
					pcie_reg_write( pdata, PHY_MDIO_REG_ADDR(1, 0xA817), 0x0, 2);

					pcie_reg_write( pdata, 0xf23d0008, 0xdc, 2);
					pcie_reg_write( pdata, 0xf23d0008, 0x0, 2);

					//pcie_reg_write( pdata, (ARM_CORE_CTL_REG + 0x90), 0x3, 2);
					// Replaced above line with mGigAuto API Code
					pcie_reg_write( pdata, PHY_MDIO_REG_ADDR(1, 0xA819), 0x3090, 2);
					pcie_reg_write( pdata, PHY_MDIO_REG_ADDR(1, 0xA81A), 0xf000, 2);
					pcie_reg_write( pdata, PHY_MDIO_REG_ADDR(1, 0xA81B), 0x3, 2);
					pcie_reg_write( pdata, PHY_MDIO_REG_ADDR(1, 0xA81C), 0x0, 2);
					pcie_reg_write( pdata, PHY_MDIO_REG_ADDR(1, 0xA817), 0x9, 2);
					pcie_reg_write( pdata, PHY_MDIO_REG_ADDR(1, 0xA817), 0x0, 2);

					XGBE_MSLEEP(2000);
					ret = pcie_reg_read(pdata, PHY_MDIO_REG_ADDR(0x1E, 0x8129), 2, (unsigned int *)&reg_value);
					printk("BCM8989x-A0 Firmware After Update Rev = 0x%x \n", reg_value);
				}
			} else {
                printk("\t Reset and Halt CPU .... \n");
                pcie_reg_write( pdata, ARM_CORE_CTL_REG, 0x823, 4);

                //Soft Reset Port
                pcie_reg_write( pdata, PHY_MDIO_REG_ADDR(0x1E, 0x8007), 0x4, 2);
                pcie_reg_write( pdata, PHY_MDIO_REG_ADDR(0x1E, 0x800A), 0x0, 2);
                pcie_reg_write( pdata, PHY_MDIO_REG_ADDR(0x1E, 0x8098), 0x0, 2);

                pcie_reg_write( pdata, PHY_MDIO_REG_ADDR(1, 0xA819), 0x0, 2);
                pcie_reg_write( pdata, PHY_MDIO_REG_ADDR(1, 0xA81A), 0x0, 2);
                pcie_reg_write( pdata, PHY_MDIO_REG_ADDR(1, 0xA817), 0x38, 2);

                printk("\t Copying Firmware to ITCM.... \n");
                for(i=0; i<size; i+=1) {
                    *((char *)(virt_addr_tcm + i)) = *((char *)(buffer + i));
                }

                pcie_reg_write( pdata, PHY_MDIO_REG_ADDR(1, 0xA817), 0x0, 2);

                printk("\t Bringing CPU out of Halt State.... \n");

                //Bring CPU out of halt state
                pcie_reg_write( pdata, ARM_CORE_CTL_REG, 0x822, 4);
			}
		} else {
			volatile unsigned int bootrom_bypass;
			/* Get BOOTROM bypass mode(0:NOT bypassed, 1:bypassed) */
			pcie_reg_read(pdata, PHY_MDIO_REG_ADDR(0x1E, 0x8087), 2, (unsigned int *)&bootrom_bypass);
			bootrom_bypass &= 0x20;

			if(bootrom_bypass) {
                printk("Bootrom is in Bypassed Mode -- Driver Wont Support this mode\n");
                goto download_complete;
            }

			pcie_reg_write( pdata, PHY_MDIO_REG_ADDR(0x1E, 0x8099), 0x1, 2); /*IOPAD_CFG_SPARE_REG1, set force download bit*/

			pcie_reg_write( pdata, ARM_CORE_CTL_REG, 0x0001202C, 4);             /*ARM_CORE_CRTL_REG, clear ITCM protection*/
			pcie_reg_write( pdata, ARM_CORE_CTL_REG + 0x90, 0x00000003, 4);      /*ARM_MEM_INIT_CRTL_REG, initialize ITCM and DTCM*/

			pcie_reg_write( pdata, PHY_MDIO_REG_ADDR(0x1E, 0x800A), 0x0, 2);    /*CRG_CHIP_WD_TIMER_CTL*/
			pcie_reg_write( pdata, PHY_MDIO_REG_ADDR(0x1E, 0x8098), 0x0, 2);    /*IOPAD_CFG_SPARE_REG0, clear PLL done status*/
			pcie_reg_write( pdata, PHY_MDIO_REG_ADDR(0x1E, 0x8007), 0x4, 2);    /*CRG_CHIP_GLB_RST_CTL, Issue soft reset*/

			printk("Waiting For Bootrom Ready Bit ...\n");
			count = 50;
			ret = pcie_reg_read(pdata, PHY_MDIO_REG_ADDR(0x1E, 0x8198), 2, (unsigned int *)&reg_value);
			while(!ret && (reg_value != 0x1) && (count > 0)) {
				XGBE_MSLEEP(100);
				ret = pcie_reg_read(pdata, PHY_MDIO_REG_ADDR(0x1E, 0x8198), 2, (unsigned int *)&reg_value);
				count--;
			}
			if(count == 0) {
				printk("Failed: Bootrom Ready Bit is not set in 5 seconds\n");
				fail = 1;
			}

			if(fail == 0) {
				pcie_reg_write( pdata, PHY_MDIO_REG_ADDR(1, 0xA81A), 0x2, 2);
				pcie_reg_write( pdata, PHY_MDIO_REG_ADDR(1, 0xA819), 0x0, 2);
				pcie_reg_write( pdata, PHY_MDIO_REG_ADDR(1, 0xA817), 0x38, 2);
				printk("\t Copying Firmware to ITCM.... \n");
				for(i=0; i<size; i+=1) {
					*((char *)(virt_addr_tcm + i)) = *((char *)(buffer + i));
				}

				pcie_reg_write( pdata, PHY_MDIO_REG_ADDR(1, 0xA817), 0x0, 2);
				pcie_reg_write( pdata, PHY_MDIO_REG_ADDR(0x1E, 0x8197), 1, 2);
				if(region) {
					pcie_reg_write( pdata, PHY_MDIO_REG_ADDR(0x1E, 0x8099), 0, 2);
				} else {
					printk("Waiting for Programming to finish ...\n");
					count = 20;
					ret = pcie_reg_read(pdata, PHY_MDIO_REG_ADDR(0x1E, 0x812B), 2, (unsigned int *)&reg_value);
					while(!ret && (reg_value != 0x600D) && (count > 0)) {
						XGBE_MSLEEP(1000);
						ret = pcie_reg_read(pdata, PHY_MDIO_REG_ADDR(0x1E, 0x812B), 2, (unsigned int *)&reg_value);
						printk("SPI ROM Status (%d) is 0x%x \n", count, reg_value);
						if(reg_value == 0xdead) {
							printk("SPI ROM Flashing Failed with DEAD \n");
							fail = 1;
							break;
						}
						count--;
					}
					if(fail == 0) {
						pcie_reg_write( pdata, PHY_MDIO_REG_ADDR(0x1E, 0x8099), 0, 2);
						pcie_reg_write( pdata, PHY_MDIO_REG_ADDR(0x1E, 0x80D5), 1, 2);
					}
				}
			}
		}
download_complete:
		spin_unlock(&pdata->phy_access_lock);
	}
#endif
}

static int xgbe_phy_an_config(struct xgbe_prv_data *pdata)
{
	volatile unsigned int reg;
	int speed = 0;
	int autoneg = 0;
	spin_lock(&pdata->phy_access_lock);
	if(pdata->phy.autoneg == 0) {
		//Set Force Speed
		switch(pdata->phy.speed) {
			case SPEED_10000: speed = 6; break;
			case SPEED_5000: speed = 5; break;
			case SPEED_2500: speed = 4; break;
			default: speed = 4; break;
		}
	}
	else {
		autoneg = 0x1200;
	}


	if(autoneg == 0) {
		printk("%s: Force Speed, %d Speed %d Duplex , Converted Speed %d\n",__func__, pdata->phy.speed, pdata->phy.duplex, speed);

		pcie_reg_write(pdata, PHY_MDIO_REG_ADDR(0x1, 0x0), 0, 2);
		XGBE_USLEEP_RANGE(2000, 5000);
		reg = 0x8000;
		reg |= speed;
		if(pdata->phy.duplex) reg |= 0x4000;

		pcie_reg_write(pdata, PHY_MDIO_REG_ADDR(0x1, 0x834) , reg, 2);
		XGBE_USLEEP_RANGE(2000, 5000);
		pcie_reg_write(pdata, PHY_MDIO_REG_ADDR(0x7, 0x200), 0x200, 2);
		XGBE_USLEEP_RANGE(2000, 5000);
	} else {
		printk("%s: Auto Negotiation Enable\n",__func__);

		pcie_reg_write(pdata, PHY_MDIO_REG_ADDR(0x7, 0x200), 0x1200, 2);
		XGBE_USLEEP_RANGE(2000, 5000);
	}
	spin_unlock(&pdata->phy_access_lock);

	return 0;
}

void xgbe_init_function_ptrs_phy(struct xgbe_phy_if *phy_if)
{
	struct xgbe_phy_impl_if *phy_impl = &phy_if->phy_impl;

	phy_impl->init			= xgbe_phy_init;
	phy_impl->exit			= xgbe_phy_exit;
	phy_impl->link_status		= xgbe_phy_link_status;
	phy_impl->get_mode		= xgbe_phy_get_mode;
	phy_impl->an_config		= xgbe_phy_an_config;
	phy_impl->an_advertising = xgbe_phy_set_autoneg_advertise;
	phy_impl->valid_speed = xgbe_phy_valid_speed;
	phy_impl->flash_firmware = xgbe_phy_flash_firmware;
	phy_impl->reg_read  = pcie_reg_read;
	phy_impl->reg_write = pcie_reg_write;
	phy_impl->otp_reg_read  = xgbe_otp_reg_read;
	phy_impl->otp_reg_write = xgbe_otp_reg_write;
	phy_impl->otp_prog_enable = xgbe_otp_prog_enable;
}

#endif
