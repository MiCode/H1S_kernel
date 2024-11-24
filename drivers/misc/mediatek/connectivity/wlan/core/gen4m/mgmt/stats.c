/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
 * PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*******************************************************************************
 *            C O M P I L E R	 F L A G S
 *******************************************************************************
 */

/*******************************************************************************
 *            E X T E R N A L	R E F E R E N C E S
 *******************************************************************************
 */
#include "precomp.h"

#if (CFG_SUPPORT_STATISTICS == 1)

enum EVENT_TYPE {
	EVENT_RX,
	EVENT_TX,
};
/*******************************************************************************
 *            C O N S T A N T S
 *******************************************************************************
 */

/*******************************************************************************
 *            F U N C T I O N   D E C L A R A T I O N S
 *******************************************************************************
 */

/*******************************************************************************
 *            P U B L I C   D A T A
 *******************************************************************************
 */
/* function pointer array for tx stats*/
static struct STATS_TLV_HDLR_T apfnTxTbl[STATS_TX_TAG_MAX_NUM] = {
	{statsTxGetQueuetLen, statsTxQueueHdlr},
	{statsTxGetPerBssLen, statsTxTlvBss0Hdlr},
	{statsTxGetTimeLen, statsTxTimeHdlr},
};

static struct STATS_TLV_HDLR_T apfnRxTbl[STATS_RX_TAG_MAX_NUM] = {
	{statsGetTlvU8Len, statsRxReorderDropHdlr},
};

static struct STATS_TLV_HDLR_T apfnCgsTbl[STATS_CGS_TAG_MAX_NUM] = {
	{statsGetTlvU8Len, statsCgsB0IdleSlotHdlr},
	{statsCgsGetAirLatLen, statsCgsAirLatHdlr},
};

/*******************************************************************************
 *            P R I V A T E  F U N C T I O N S
 *******************************************************************************
 */

uint32_t u4TotalTx;
uint32_t u4NoDelayTx;
uint32_t u4TotalRx;
uint32_t u4NoDelayRx;

static uint8_t g_ucTxRxFlag;
static uint8_t g_ucTxIpProto;
static uint16_t g_u2TxUdpPort;
static uint32_t g_u4TxDelayThreshold;
static uint8_t g_ucRxIpProto;
static uint16_t g_u2RxUdpPort;
static uint32_t g_u4RxDelayThreshold;

void StatsResetTxRx(void)
{
	u4TotalRx = 0;
	u4TotalTx = 0;
	u4NoDelayRx = 0;
	u4NoDelayTx = 0;
}

uint64_t StatsEnvTimeGet(void)
{
	uint64_t u8Clk;

	u8Clk = sched_clock();	/* unit: naro seconds */

	return (uint64_t) u8Clk;	/* sched_clock *//* jiffies size = 4B */
}

void StatsEnvGetPktDelay(OUT uint8_t *pucTxRxFlag,
	OUT uint8_t *pucTxIpProto, OUT uint16_t *pu2TxUdpPort,
	OUT uint32_t *pu4TxDelayThreshold, OUT uint8_t *pucRxIpProto,
	OUT uint16_t *pu2RxUdpPort, OUT uint32_t *pu4RxDelayThreshold)
{
	*pucTxRxFlag = g_ucTxRxFlag;
	*pucTxIpProto = g_ucTxIpProto;
	*pu2TxUdpPort = g_u2TxUdpPort;
	*pu4TxDelayThreshold = g_u4TxDelayThreshold;
	*pucRxIpProto = g_ucRxIpProto;
	*pu2RxUdpPort = g_u2RxUdpPort;
	*pu4RxDelayThreshold = g_u4RxDelayThreshold;
}

void StatsEnvSetPktDelay(IN uint8_t ucTxOrRx, IN uint8_t ucIpProto,
	IN uint16_t u2UdpPort, uint32_t u4DelayThreshold)
{
#define MODULE_RESET 0
#define MODULE_TX 1
#define MODULE_RX 2

	if (ucTxOrRx == MODULE_TX) {
		g_ucTxRxFlag |= BIT(0);
		g_ucTxIpProto = ucIpProto;
		g_u2TxUdpPort = u2UdpPort;
		g_u4TxDelayThreshold = u4DelayThreshold;
	} else if (ucTxOrRx == MODULE_RX) {
		g_ucTxRxFlag |= BIT(1);
		g_ucRxIpProto = ucIpProto;
		g_u2RxUdpPort = u2UdpPort;
		g_u4RxDelayThreshold = u4DelayThreshold;
	} else if (ucTxOrRx == MODULE_RESET) {
		g_ucTxRxFlag = 0;
		g_ucTxIpProto = 0;
		g_u2TxUdpPort = 0;
		g_u4TxDelayThreshold = 0;
		g_ucRxIpProto = 0;
		g_u2RxUdpPort = 0;
		g_u4RxDelayThreshold = 0;
	}
}

void StatsEnvRxTime2Host(IN struct ADAPTER *prAdapter,
	struct sk_buff *prSkb, struct net_device *prNetDev)
{
	uint8_t *pucEth = prSkb->data;
	uint16_t u2EthType = 0;
	uint8_t ucIpVersion = 0;
	uint8_t ucIpProto = 0;
	uint16_t u2IPID = 0;
	uint16_t u2UdpDstPort = 0;
	uint16_t u2UdpSrcPort = 0;
	uint64_t u8IntTime = 0;
	uint64_t u8RxTime = 0;
	uint32_t u4Delay = 0;
	struct timespec64 tval;
	struct rtc_time tm;

	u2EthType = (pucEth[ETH_TYPE_LEN_OFFSET] << 8)
		| (pucEth[ETH_TYPE_LEN_OFFSET + 1]);
	pucEth += ETH_HLEN;
	u2IPID = pucEth[4] << 8 | pucEth[5];

	DBGLOG(RX, TEMP, "u2IpId=%d rx_packets=%lu\n",
		u2IPID, prNetDev->stats.rx_packets);

	if ((g_ucTxRxFlag & BIT(1)) == 0)
		return;
	if (prSkb->len <= 24 + ETH_HLEN)
		return;
	if (u2EthType != ETH_P_IPV4)
		return;
	ucIpProto = pucEth[9];
	if (g_ucRxIpProto && (ucIpProto != g_ucRxIpProto))
		return;
	ucIpVersion = (pucEth[0] & IPVH_VERSION_MASK) >> IPVH_VERSION_OFFSET;
	if (ucIpVersion != IPVERSION)
		return;
	u2IPID = pucEth[4] << 8 | pucEth[5];
	u8IntTime = GLUE_RX_GET_PKT_INT_TIME(prSkb);
	u4Delay = ((uint32_t)(sched_clock() - u8IntTime))/NSEC_PER_USEC;
	u8RxTime = GLUE_RX_GET_PKT_RX_TIME(prSkb);
	ktime_get_ts64(&tval);
	rtc_time64_to_tm(tval.tv_sec, &tm);

	switch (ucIpProto) {
	case IP_PRO_TCP:
	case IP_PRO_UDP:
		u2UdpSrcPort = (pucEth[20] << 8) | pucEth[21];
		u2UdpDstPort = (pucEth[22] << 8) | pucEth[23];
		if (g_u2RxUdpPort && (u2UdpSrcPort != g_u2RxUdpPort))
			break;
	case IP_PRO_ICMP:
		u4TotalRx++;
		if (g_u4RxDelayThreshold && (u4Delay <= g_u4RxDelayThreshold)) {
			u4NoDelayRx++;
			break;
		}
		DBGLOG(RX, INFO,
	"IPID 0x%04x src %d dst %d UP %d,delay %u us,int2rx %lu us,IntTime %llu,%u/%u,leave at %02d:%02d:%02d.%09ld\n",
			u2IPID, u2UdpSrcPort, u2UdpDstPort,
			((pucEth[1] & IPTOS_PREC_MASK) >> IPTOS_PREC_OFFSET),
			u4Delay,
			((uint32_t)(u8RxTime - u8IntTime))/NSEC_PER_USEC,
			u8IntTime, u4NoDelayRx, u4TotalRx,
			tm.tm_hour, tm.tm_min, tm.tm_sec, tval.tv_nsec);
		break;
	default:
		break;
	}
}

void StatsEnvTxTime2Hif(IN struct ADAPTER *prAdapter,
	IN struct MSDU_INFO *prMsduInfo)
{
	uint64_t u8SysTime, u8SysTimeIn;
	uint32_t u4TimeDiff;
	uint8_t *pucEth;
	uint32_t u4PacketLen;
	uint8_t ucIpVersion = 0;
	uint8_t ucIpProto = 0;
	uint8_t *pucEthBody = NULL;
	uint16_t u2EthType = 0;
	uint8_t *pucAheadBuf = NULL;
	uint16_t u2IPID = 0;
	uint16_t u2UdpDstPort = 0;
	uint16_t u2UdpSrcPort = 0;

	if (prMsduInfo == NULL) {
		DBGLOG(TX, ERROR, "prMsduInfo=NULL");
		return;
	}

	if (prMsduInfo->prPacket == NULL) {
		DBGLOG(TX, ERROR, "prMsduInfo->prPacket=NULL");
		return;
	}

	kalTraceEvent("Move ipid=0x%04x sn=%d",
		GLUE_GET_PKT_IP_ID(prMsduInfo->prPacket),
		GLUE_GET_PKT_SEQ_NO(prMsduInfo->prPacket));

	pucEth = ((struct sk_buff *)prMsduInfo->prPacket)->data;

	if (pucEth == NULL) {
		DBGLOG(TX, ERROR, "pucEth=NULL");
		return;
	}

	u4PacketLen = ((struct sk_buff *)prMsduInfo->prPacket)->len;

	u8SysTime = StatsEnvTimeGet();
	u8SysTimeIn = GLUE_GET_PKT_XTIME(prMsduInfo->prPacket);

	if ((g_ucTxRxFlag & BIT(0)) == 0)
		return;

	if ((u8SysTimeIn == 0) || (u8SysTime <= u8SysTimeIn))
		return;

	/* units of u4TimeDiff is micro seconds (us) */
	if (u4PacketLen < 24 + ETH_HLEN)
		return;
	pucAheadBuf = &pucEth[76];
	u2EthType = (pucAheadBuf[ETH_TYPE_LEN_OFFSET] << 8)
		| (pucAheadBuf[ETH_TYPE_LEN_OFFSET + 1]);
	pucEthBody = &pucAheadBuf[ETH_HLEN];
	if (u2EthType != ETH_P_IPV4)
		return;
	ucIpProto = pucEthBody[9];
	if (g_ucTxIpProto && (ucIpProto != g_ucTxIpProto))
		return;
	ucIpVersion = (pucEthBody[0] & IPVH_VERSION_MASK)
		>> IPVH_VERSION_OFFSET;
	if (ucIpVersion != IPVERSION)
		return;
	u2IPID = pucEthBody[4]<<8 | pucEthBody[5];
	u8SysTime = u8SysTime - u8SysTimeIn;
	u4TimeDiff = (uint32_t) u8SysTime;
	u4TimeDiff = u4TimeDiff / 1000;	/* ns to us */

	switch (ucIpProto) {
	case IP_PRO_TCP:
	case IP_PRO_UDP:
		u2UdpDstPort = (pucEthBody[22] << 8) | pucEthBody[23];
		u2UdpSrcPort = (pucEthBody[20] << 8) | pucEthBody[21];
		if (g_u2TxUdpPort && (u2UdpDstPort != g_u2TxUdpPort))
			break;
	case IP_PRO_ICMP:
		u4TotalTx++;
		if (g_u4TxDelayThreshold
			&& (u4TimeDiff <= g_u4TxDelayThreshold)) {
			u4NoDelayTx++;
			break;
		}
		DBGLOG(TX, INFO,
			"IPID 0x%04x src %d dst %d UP %d,delay %u us,u8SysTimeIn %llu, %u/%u\n",
			u2IPID, u2UdpSrcPort, u2UdpDstPort,
			((pucEthBody[1] & IPTOS_PREC_MASK)
				>> IPTOS_PREC_OFFSET),
			u4TimeDiff, u8SysTimeIn, u4NoDelayTx, u4TotalTx);
		break;
	default:
		break;
	}
}

void statsParseARPInfo(struct sk_buff *skb,
		uint8_t *pucEthBody, uint8_t eventType)
{
	uint16_t u2OpCode = (pucEthBody[6] << 8) | pucEthBody[7];

	switch (eventType) {
	case EVENT_RX:
		GLUE_SET_INDEPENDENT_PKT(skb, TRUE);
		if (u2OpCode == ARP_PRO_REQ)
			DBGLOG(RX, INFO,
				"<RX> Arp Req From IP: " IPV4STR "\n",
				IPV4TOSTR(&pucEthBody[ARP_SENDER_IP_OFFSET]));
		else if (u2OpCode == ARP_PRO_RSP)
			DBGLOG(RX, INFO,
				"<RX> Arp Rsp From IP: " IPV4STR "\n",
				IPV4TOSTR(&pucEthBody[ARP_SENDER_IP_OFFSET]));
		break;
	case EVENT_TX:
		DBGLOG(TX, INFO,
			"ARP %s SRC MAC/IP["
			MACSTR "]/[" IPV4STR "], TAR MAC/IP["
			MACSTR "]/[" IPV4STR "], SeqNo: %d\n",
			u2OpCode == ARP_OPERATION_REQUEST ? "REQ" : "RSP",
			MAC2STR(&pucEthBody[ARP_SENDER_MAC_OFFSET]),
			IPV4TOSTR(&pucEthBody[ARP_SENDER_IP_OFFSET]),
			MAC2STR(&pucEthBody[ARP_TARGET_MAC_OFFSET]),
			IPV4TOSTR(&pucEthBody[ARP_TARGET_IP_OFFSET]),
			GLUE_GET_PKT_SEQ_NO(skb));
		break;
	}
}

void statsParseUDPInfo(struct sk_buff *skb, uint8_t *pucEthBody,
		uint8_t eventType, uint16_t u2IpId)
{
	/* the number of DHCP packets is seldom so we print log here */
	uint8_t *pucUdp = &pucEthBody[20];
	uint8_t *pucBootp = &pucUdp[UDP_HDR_LEN];
	struct BOOTP_PROTOCOL *prBootp = NULL;
	uint16_t u2UdpDstPort;
	uint16_t u2UdpSrcPort;
	uint32_t u4TransID;
	uint32_t u4DhcpMagicCode;
	char buf[50] = {0};

	prBootp = (struct BOOTP_PROTOCOL *) &pucUdp[UDP_HDR_LEN];
	u2UdpDstPort = (pucUdp[2] << 8) | pucUdp[3];
	u2UdpSrcPort = (pucUdp[0] << 8) | pucUdp[1];
	if (u2UdpDstPort == UDP_PORT_DHCPS || u2UdpDstPort == UDP_PORT_DHCPC) {
		WLAN_GET_FIELD_BE32(&prBootp->u4TransId, &u4TransID);
		switch (eventType) {
		case EVENT_RX:
			GLUE_SET_INDEPENDENT_PKT(skb, TRUE);

			WLAN_GET_FIELD_BE32(&prBootp->aucOptions[0],
					    &u4DhcpMagicCode);
			if (u4DhcpMagicCode == DHCP_MAGIC_NUMBER) {
				uint32_t u4Opt;

				WLAN_GET_FIELD_BE32(&prBootp->aucOptions[4],
						    &u4Opt);
				switch (u4Opt & 0xffffff00) {
				case 0x35010100:
					kalSnprintf(buf, 49, "DISCOVERY");
					break;
				case 0x35010200:
					kalSnprintf(buf, 49, "OFFER");
					break;
				case 0x35010300:
					kalSnprintf(buf, 49, "REQUEST");
					break;
				case 0x35010500:
					kalSnprintf(buf, 49, "ACK");
					break;
				case 0x35010600:
					kalSnprintf(buf, 49, "NAK");
					break;
				}
			}
			DBGLOG_LIMITED(RX, INFO,
				"<RX> DHCP: Recv %s IPID 0x%02x, MsgType 0x%x, TransID 0x%04x\n",
				buf, u2IpId, prBootp->aucOptions[6],
				u4TransID);
			break;
		case EVENT_TX:
		{
			uint32_t u4Xid = 0;
			uint32_t u4Opt = 0;

			WLAN_GET_FIELD_BE32(&prBootp->aucOptions[0],
					    &u4DhcpMagicCode);
			if (u4DhcpMagicCode == DHCP_MAGIC_NUMBER) {
				WLAN_GET_FIELD_BE32(&prBootp->u4TransId,
						    &u4Xid);
				WLAN_GET_FIELD_BE32(&prBootp->aucOptions[4],
						    &u4Opt);

				switch (u4Opt & 0xffffff00) {
				case 0x35010100:
					kalSnprintf(buf, 49,
						"client DISCOVERY");
					break;
				case 0x35010200:
					kalSnprintf(buf, 49, "server OFFER");
					break;
				case 0x35010300:
					kalSnprintf(buf, 49, "client REQUEST");
					break;
				case 0x35010500:
					kalSnprintf(buf, 49, "server ACK");
					break;
				case 0x35010600:
					kalSnprintf(buf, 49, "server NAK");
					break;
				}
			}

			DBGLOG_LIMITED(TX, INFO,
				"<TP> DHCP %s, XID[0x%08x] OPT[0x%08x] TYPE[%u], SeqNo: %d\n",
				buf, u4Xid, u4Opt, prBootp->aucOptions[6],
				GLUE_GET_PKT_SEQ_NO(skb));
		}
			break;
		}
	} else if (u2UdpSrcPort == UDP_PORT_DNS ||
			u2UdpDstPort == UDP_PORT_DNS) {
		uint16_t u2TransId =
			(pucBootp[0] << 8) | pucBootp[1];
		if (eventType == EVENT_RX) {
			GLUE_SET_INDEPENDENT_PKT(skb, TRUE);
			DBGLOG_LIMITED(RX, INFO,
				"<RX> DNS: IPID 0x%02x, TransID 0x%04x\n",
				u2IpId, u2TransId);
		} else if (eventType == EVENT_TX) {
			DBGLOG_LIMITED(TX, INFO,
				"<TX> DNS: IPID[0x%02x] TransID[0x%04x] SeqNo[%d]\n",
				u2IpId, u2TransId, GLUE_GET_PKT_SEQ_NO(skb));
		}
	}
}

void statsParseIPV4Info(struct sk_buff *skb,
		uint8_t *pucEthBody, uint8_t eventType)
{
	/* IP header without options */
	uint8_t ucIpProto = pucEthBody[9];
	uint8_t ucIpVersion =
		(pucEthBody[0] & IPVH_VERSION_MASK)
			>> IPVH_VERSION_OFFSET;
	uint16_t u2IpId = pucEthBody[4] << 8 | pucEthBody[5];

	if (ucIpVersion != IPVERSION)
		return;

	GLUE_SET_PKT_IP_ID(skb, u2IpId);
	switch (ucIpProto) {
	case IP_PRO_ICMP:
	{
		/* the number of ICMP packets is seldom so we print log here */
		uint8_t ucIcmpType;
		uint16_t u2IcmpId, u2IcmpSeq;
		uint8_t *pucIcmp = &pucEthBody[20];

		ucIcmpType = pucIcmp[0];
		/* don't log network unreachable packet */
		if (ucIcmpType == 3)
			break;
		u2IcmpId = *(uint16_t *) &pucIcmp[4];
		u2IcmpSeq = *(uint16_t *) &pucIcmp[6];
		switch (eventType) {
		case EVENT_RX:
			GLUE_SET_INDEPENDENT_PKT(skb, TRUE);
			DBGLOG_LIMITED(RX, INFO,
				"<RX> ICMP: Type %d, Id BE 0x%04x, Seq BE 0x%04x\n",
				ucIcmpType, u2IcmpId, u2IcmpSeq);
			break;
		case EVENT_TX:
			DBGLOG_LIMITED(TX, INFO,
				"<TX> ICMP: IPID[0x%04x] Type %d, Id 0x%04x, Seq BE 0x%04x, SeqNo: %d\n",
				u2IpId, ucIcmpType, u2IcmpId, u2IcmpSeq,
				GLUE_GET_PKT_SEQ_NO(skb));
			break;
		}
		break;
	}
	case IP_PRO_UDP:
		statsParseUDPInfo(skb, pucEthBody, eventType, u2IpId);
	}
}

void statsLogData(uint8_t eventType, enum WAKE_DATA_TYPE wakeType)
{
	if (eventType == EVENT_TX)
		wlanLogTxData(wakeType);
	else if (eventType == EVENT_RX)
		wlanLogRxData(wakeType);
}

static void statsParsePktInfo(uint8_t *pucPkt, struct sk_buff *skb,
	uint8_t status, uint8_t eventType)
{
	/* get ethernet protocol */
	uint16_t u2EtherType =
		(pucPkt[ETH_TYPE_LEN_OFFSET] << 8)
			| (pucPkt[ETH_TYPE_LEN_OFFSET + 1]);
	uint8_t *pucEthBody = &pucPkt[ETH_HLEN];

	switch (u2EtherType) {
	case ETH_P_ARP:
	{
		statsLogData(eventType, WLAN_WAKE_ARP);
		statsParseARPInfo(skb, pucEthBody, eventType);
		break;
	}
	case ETH_P_IPV4:
	{
		statsLogData(eventType, WLAN_WAKE_IPV4);
		statsParseIPV4Info(skb, pucEthBody, eventType);
		break;
	}
	case ETH_P_IPV6:
	{
		/* IPv6 header without options */
		uint8_t ucIpv6Proto =
			pucEthBody[IPV6_HDR_PROTOCOL_OFFSET];
		uint8_t ucIpVersion =
			(pucEthBody[0] & IPVH_VERSION_MASK)
				>> IPVH_VERSION_OFFSET;

		if (ucIpVersion != IP_VERSION_6)
			break;

		statsLogData(eventType, WLAN_WAKE_IPV6);
		switch (ucIpv6Proto) {
		case 0x06:/*tcp*/
			switch (eventType) {
			case EVENT_RX:
				DBGLOG(RX, TRACE, "<RX><IPv6> tcp packet\n");
				break;
			case EVENT_TX:
				DBGLOG(TX, TRACE, "<TX><IPv6> tcp packet\n");
				break;
			}
			break;

		case 0x11:/*UDP*/
			switch (eventType) {
			case EVENT_RX:
			{
				uint16_t ucIpv6UDPSrcPort = 0;

				/* IPv6 header without options */
				ucIpv6UDPSrcPort = pucEthBody[IPV6_HDR_LEN];
				ucIpv6UDPSrcPort = ucIpv6UDPSrcPort << 8;
				ucIpv6UDPSrcPort +=
					pucEthBody[IPV6_HDR_LEN + 1];

				switch (ucIpv6UDPSrcPort) {
				case 53:/*dns port*/
					DBGLOG(RX, TRACE,
						"<RX><IPv6> dns packet\n");
					GLUE_SET_INDEPENDENT_PKT(skb, TRUE);
					break;
				case 547:/*dhcp*/
				case 546:
					DBGLOG(RX, INFO,
						"<RX><IPv6> dhcp packet\n");
					GLUE_SET_INDEPENDENT_PKT(skb, TRUE);
					break;
				case 123:/*ntp port*/
					DBGLOG(RX, INFO,
						"<RX><IPv6> ntp packet\n");
					GLUE_SET_INDEPENDENT_PKT(skb, TRUE);
					break;
				default:
					DBGLOG(RX, TRACE,
					"<RX><IPv6> other packet srtport=%u\n",
						ucIpv6UDPSrcPort);
					break;
				}
			}
				break;
			case EVENT_TX:
				DBGLOG(TX, TRACE, "<TX><IPv6> UDP packet\n");
				break;
			}
			break;

		case 0x00:/*IPv6  hop-by-hop*/
			switch (eventType) {
			case EVENT_RX:
				/*need chech detai pakcet type*/
				/*130 mlti listener query*/
				/*143 multi listener report v2*/
				GLUE_SET_INDEPENDENT_PKT(skb, TRUE);

				DBGLOG(RX, INFO,
					"<RX><IPv6> hop-by-hop packet\n");
				break;
			case EVENT_TX:
				DBGLOG(TX, INFO,
					"<TX><IPv6> hop-by-hop packet\n");
				break;
			}
			break;

		case 0x3a:/*ipv6 ICMPV6*/
			switch (eventType) {
			case EVENT_RX:
			{
				uint8_t ucICMPv6Type = 0;

				/* IPv6 header without options */
				ucICMPv6Type = pucEthBody[IPV6_HDR_LEN];
				GLUE_SET_INDEPENDENT_PKT(skb, TRUE);

				switch (ucICMPv6Type) {
				case 0x85: /*ICMPV6_TYPE_ROUTER_SOLICITATION*/
					DBGLOG_LIMITED(RX, INFO,
				"<RX><IPv6> ICMPV6 Router Solicitation\n");
					break;

				case 0x86: /*ICMPV6_TYPE_ROUTER_ADVERTISEMENT*/
					DBGLOG_LIMITED(RX, INFO,
				"<RX><IPv6> ICMPV6 Router Advertisement\n");
					break;

				case ICMPV6_TYPE_NEIGHBOR_SOLICITATION:
					DBGLOG_LIMITED(RX, INFO,
				"<RX><IPv6> ICMPV6 Neighbor Solicitation\n");
					break;

				case ICMPV6_TYPE_NEIGHBOR_ADVERTISEMENT:
					DBGLOG_LIMITED(RX, INFO,
				"<RX><IPv6> ICMPV6 Neighbor Advertisement\n");
					break;
				default:
					DBGLOG_LIMITED(RX, INFO,
						"<RX><IPv6> ICMPV6 type=%u\n",
						ucICMPv6Type);
					break;
				}
			}
				break;
			case EVENT_TX:
				DBGLOG_LIMITED(TX, INFO,
					"<TX><IPv6> ICMPV6 packet\n");
				break;
			}
			break;
		default:
			if (eventType == EVENT_RX)
				DBGLOG(RX, INFO,
				"<RX><IPv6> default protocol=%u\n",
				ucIpv6Proto);
			break;
		}
		break;
	}
	case ETH_P_1X:
	{
		uint8_t *pucEapol = pucEthBody;
		uint8_t ucEapolType = pucEapol[1];
		uint16_t u2KeyInfo = 0;
		uint8_t m = 0;

		statsLogData(eventType, WLAN_WAKE_1X);
		switch (ucEapolType) {
		case 0: /* eap packet */
			switch (eventType) {
			case EVENT_RX:
				DBGLOG(RX, INFO,
					"<RX> EAP Packet: code %d, id %d, type %d\n",
					pucEapol[4], pucEapol[5], pucEapol[7]);
				break;
			case EVENT_TX:
				DBGLOG(TX, INFO,
				       "<TX> EAP Packet: code %d, id %d, type %d, SeqNo: %d\n",
				       pucEapol[4], pucEapol[5], pucEapol[7],
				       GLUE_GET_PKT_SEQ_NO(skb));
				break;
			}
			break;
		case 1: /* eapol start */
			switch (eventType) {
			case EVENT_RX:
				DBGLOG(RX, INFO,
					"<RX> EAPOL: start\n");
				break;
			case EVENT_TX:
				DBGLOG(TX, INFO,
				       "<TX> EAPOL: start, SeqNo: %d\n",
				       GLUE_GET_PKT_SEQ_NO(skb));
				break;
			}
			break;
		case ETH_EAPOL_KEY: /* key */
			WLAN_GET_FIELD_BE16(&pucEapol[5], &u2KeyInfo);
			switch (eventType) {
			case EVENT_RX:
				if ((u2KeyInfo & 0x1100) == 0x0000 ||
					(u2KeyInfo & 0x0008) == 0x0000)
					m = 1;
				else
					m = 3;
				DBGLOG(RX, INFO,
					"<RX> EAPOL: key, M%d, KeyInfo 0x%04x\n",
					m, u2KeyInfo);
				break;
			case EVENT_TX:
				if ((u2KeyInfo & 0xfff0) == 0x0100)
					m = 2;
				else if ((u2KeyInfo & 0xfff0) == 0x0300)
					m = 4;
				DBGLOG(TX, INFO,
				       "<TX> EAPOL: key, M%d, KeyInfo 0x%04x SeqNo: %d\n",
				       m, u2KeyInfo, GLUE_GET_PKT_SEQ_NO(skb));
				break;
			}
			break;
		}
		break;
	}
#if CFG_SUPPORT_WAPI
	case ETH_WPI_1X:
	{
		uint8_t ucSubType = pucEthBody[3]; /* sub type filed*/
		uint16_t u2Length = *(uint16_t *)&pucEthBody[6];
		uint16_t u2Seq = *(uint16_t *)&pucEthBody[8];

		statsLogData(eventType, WLAN_WAKE_1X);
		switch (eventType) {
		case EVENT_RX:
			DBGLOG(RX, INFO,
				"<RX> WAPI: subType %d, Len %d, Seq %d\n",
				ucSubType, u2Length, u2Seq);
			break;
		case EVENT_TX:
			DBGLOG(TX, INFO,
			       "<TX> WAPI: subType %d, Len %d, Seq %d, SeqNo: %d\n",
			       ucSubType, u2Length, u2Seq,
			       GLUE_GET_PKT_SEQ_NO(skb));
			break;
		}
		break;
	}
#endif
	case ETH_PRO_TDLS:
		statsLogData(eventType, WLAN_WAKE_TDLS);
		switch (eventType) {
		case EVENT_RX:
			DBGLOG(RX, INFO,
				"<RX> TDLS type %d, category %d, Action %d, Token %d\n",
				pucEthBody[0], pucEthBody[1],
				pucEthBody[2], pucEthBody[3]);
			break;
		case EVENT_TX:
			DBGLOG(TX, INFO,
				"<TX> TDLS type %d, category %d, Action %d, Token %d\n",
				pucEthBody[0], pucEthBody[1],
				pucEthBody[2], pucEthBody[3]);
			break;
		}
		break;
	default:
		statsLogData(eventType, WLAN_WAKE_OTHER);
		break;
	}
}
/*----------------------------------------------------------------------------*/
/*! \brief  This routine is called to display rx packet information.
 *
 * \param[in] pPkt			Pointer to the packet
 * \param[out] None
 *
 * \retval None
 */
/*----------------------------------------------------------------------------*/
void StatsRxPktInfoDisplay(struct SW_RFB *prSwRfb)
{
	uint8_t *pPkt = NULL;
	struct sk_buff *skb = NULL;

	if (prSwRfb->u2PacketLen <= ETHER_HEADER_LEN)
		return;

	pPkt = prSwRfb->pvHeader;
	if (!pPkt)
		return;

	skb = (struct sk_buff *)(prSwRfb->pvPacket);
	if (!skb)
		return;

	statsParsePktInfo(pPkt, skb, 0, EVENT_RX);

	DBGLOG(RX, TEMP, "RxPkt p=%p ipid=%d\n",
		prSwRfb, GLUE_GET_PKT_IP_ID(skb));
	kalTraceEvent("RxPkt p=%p ipid=0x%04x",
		prSwRfb, GLUE_GET_PKT_IP_ID(skb));
}

/*----------------------------------------------------------------------------*/
/*! \brief  This routine is called to display tx packet information.
 *
 * \param[in] pPkt			Pointer to the packet
 * \param[out] None
 *
 * \retval None
 */
/*----------------------------------------------------------------------------*/
void StatsTxPktInfoDisplay(struct sk_buff *prSkb)
{
	uint8_t *pPkt;

	pPkt = prSkb->data;
	statsParsePktInfo(pPkt, prSkb, 0, EVENT_TX);
}

uint32_t
statsGetTlvU2Len(void)
{
	return (uint32_t)(sizeof(uint16_t) + sizeof(struct STATS_TRX_TLV_T));
}

uint32_t
statsGetTlvU4Len(void)
{
	return (uint32_t)(sizeof(uint32_t) + sizeof(struct STATS_TRX_TLV_T));
}

uint32_t
statsGetTlvU8Len(void)
{
	return (uint32_t)(sizeof(uint64_t) + sizeof(struct STATS_TRX_TLV_T));
}

uint32_t
statsTxGetQueuetLen(void)
{
	return (uint32_t)(sizeof(struct STATS_TX_QUEUE_STAT_T) +
		sizeof(struct STATS_TRX_TLV_T));
}

uint32_t
statsTxGetPerBssLen(void)
{
	return (uint32_t)(sizeof(struct STATS_TX_PER_BSS_STAT_T) +
		sizeof(struct STATS_TRX_TLV_T));
}

uint32_t
statsTxGetTimeLen(void)
{
	return (uint32_t)(sizeof(struct STATS_TX_TIME_STAT_T) +
		sizeof(struct STATS_TRX_TLV_T));
}

uint32_t
statsCgsGetAirLatLen(void)
{
	return (uint32_t)(sizeof(struct STATS_CGS_LAT_STAT_T) +
		sizeof(struct STATS_TRX_TLV_T));
}

uint32_t
statsTxGetTlvStatTotalLen(void)
{
	uint32_t u4TlvLen = 0;
	uint32_t u4TlvIdx = 0;

	for (u4TlvIdx = 0; u4TlvIdx < STATS_TX_TAG_MAX_NUM; u4TlvIdx++) {
		if (apfnTxTbl[u4TlvIdx].pfnTlvGetLen)
			u4TlvLen += apfnTxTbl[u4TlvIdx].pfnTlvGetLen();
	}
	DBGLOG(TX, TRACE, "%s=%u\n", __func__, u4TlvLen);
	return u4TlvLen;
}

uint32_t
statsRxGetTlvStatTotalLen(void)
{
	uint32_t u4TlvLen = 0;
	uint32_t u4TlvIdx = 0;

	for (u4TlvIdx = 0; u4TlvIdx < STATS_RX_TAG_MAX_NUM; u4TlvIdx++) {
		if (apfnRxTbl[u4TlvIdx].pfnTlvGetLen)
			u4TlvLen += apfnRxTbl[u4TlvIdx].pfnTlvGetLen();
	}
	DBGLOG(RX, TRACE, "%s=%u\n", __func__, u4TlvLen);
	return u4TlvLen;
}

uint32_t
statsCgsGetTlvStatTotalLen(void)
{
	uint32_t u4TlvLen = 0;
	uint32_t u4TlvIdx = 0;

	for (u4TlvIdx = 0; u4TlvIdx < STATS_CGS_TAG_MAX_NUM; u4TlvIdx++) {
		if (apfnCgsTbl[u4TlvIdx].pfnTlvGetLen)
			u4TlvLen += apfnCgsTbl[u4TlvIdx].pfnTlvGetLen();
	}
	DBGLOG(TX, TRACE, "%s=%u\n", __func__, u4TlvLen);
	return u4TlvLen;
}

void
statsTxQueueHdlr(struct GLUE_INFO *prGlueInfo,
	struct STATS_TRX_TLV_T *prTlvList, uint32_t u4TlvLen)
{
	struct ADAPTER *prAdapter;
	struct BUS_INFO *prBusInfo;
	struct PLE_TOP_CR *prCr;
	struct CMD_ACCESS_REG rCmdAccessReg;
	struct STATS_TRX_TLV_T *prStatTlv = prTlvList;
	struct STATS_TX_QUEUE_STAT_T *prQueueStat;
	uint32_t u4MsduTokenUsed = 0;
	uint32_t u4BufLen = 0;
	uint32_t rStatus;

	prQueueStat = (struct STATS_TX_QUEUE_STAT_T *)(
		&prStatTlv->aucBuffer[0]);
	/* MSDU token */
	prAdapter = prGlueInfo->prAdapter;
	u4MsduTokenUsed = prGlueInfo->rHifInfo.rTokenInfo.u4UsedCnt;
	prQueueStat->u4MsduTokenUsed = u4MsduTokenUsed;
	prQueueStat->u4MsduTokenRsvd = HIF_TX_MSDU_TOKEN_NUM - u4MsduTokenUsed;

	/* ple hif */
	prBusInfo = prAdapter->chip_info->bus_info;
	prCr = prBusInfo->prPleTopCr;
	rCmdAccessReg.u4Address = prCr->rHifPgInfo.u4Addr;
	rCmdAccessReg.u4Data = 0;

	rStatus = kalIoctl(prGlueInfo, wlanoidQueryMcrRead,
			&rCmdAccessReg, sizeof(rCmdAccessReg),
			TRUE, TRUE, TRUE, &u4BufLen);
	prQueueStat->u4PleHifUsed = ((rCmdAccessReg.u4Data &
		prCr->rHifPgInfoHifSrcCnt.u4Mask) >>
		prCr->rHifPgInfoHifSrcCnt.u4Shift);
	prQueueStat->u4PleHifRsvd = ((rCmdAccessReg.u4Data &
		prCr->rHifPgInfoHifRsvCnt.u4Mask) >>
		prCr->rHifPgInfoHifRsvCnt.u4Shift);

	prStatTlv->u4Tag = STATS_TX_TAG_QUEUE;
	prStatTlv->u4Len = u4TlvLen;
	prTlvList += u4TlvLen;
	DBGLOG(TX, TRACE, "len=%u Msdu=[%u/%u] PLE Hif=[0x%03x/0x%03x]\n",
		u4TlvLen, prQueueStat->u4MsduTokenUsed,
		prQueueStat->u4MsduTokenRsvd,
		prQueueStat->u4PleHifUsed, prQueueStat->u4PleHifRsvd);
}

void
statsTxTlvBss0Hdlr(struct GLUE_INFO *prGlueInfo,
	struct STATS_TRX_TLV_T *prTlvList, uint32_t u4TlvLen)
{
	struct PARAM_GET_LINK_QUALITY_INFO rParam;
	struct WIFI_LINK_QUALITY_INFO rLinkQualityInfo;
	struct STATS_TRX_TLV_T *prStatTlv = prTlvList;
	struct STATS_TX_PER_BSS_STAT_T *prBssStat;
	uint32_t u4BufLen;
	int32_t i4Status;
	uint64_t u8Retry = 0;
	uint64_t u8RtsFail = 0;
	uint64_t u8AckFail = 0;

	prBssStat = (struct STATS_TX_PER_BSS_STAT_T *)(
		&prStatTlv->aucBuffer[0]);
	rParam.ucBssIdx = 0; /* prNetDevPrivate->ucBssIdx; */
	rParam.prLinkQualityInfo = &rLinkQualityInfo;
	i4Status = kalIoctl(prGlueInfo, wlanoidGetLinkQualityInfo,
		 &rParam, sizeof(struct PARAM_GET_LINK_QUALITY_INFO),
		 TRUE, FALSE, FALSE, &u4BufLen);
	if (i4Status != WLAN_STATUS_SUCCESS)
		DBGLOG(REQ, ERROR, "wlanoidGetLinkQualityInfo error\n");
	else {
		if (kalGetMediaStateIndicated(prGlueInfo,
			AIS_DEFAULT_INDEX) == MEDIA_STATE_CONNECTED) {
			u8Retry = rLinkQualityInfo.u8TxRetryCount;
			u8RtsFail = rLinkQualityInfo.u8TxRtsFailCount;
			u8AckFail = rLinkQualityInfo.u8TxAckFailCount;
		} else {
			DBGLOG(TX, TRACE, "Bss0 not connected yet.\n");
		}
	}
	prBssStat->u8Retry = u8Retry;
	prBssStat->u8RtsFail = u8RtsFail;
	prBssStat->u8AckFail = u8AckFail;
	prStatTlv->u4Tag = STATS_TX_TAG_BSS0;
	prStatTlv->u4Len = u4TlvLen;
	prTlvList += u4TlvLen;
	DBGLOG(TX, TRACE, "Bss0 len=%u retry=%lu RtsFail=%lu AckFail=%lu\n",
		u4TlvLen, prBssStat->u8Retry, prBssStat->u8RtsFail,
		prBssStat->u8AckFail);
}

void
statsTxTimeHdlr(struct GLUE_INFO *prGlueInfo,
	struct STATS_TRX_TLV_T *prTlvList, uint32_t u4TlvLen)
{
	struct STATS_TRX_TLV_T *prStatTlv = prTlvList;
	struct STATS_TX_TIME_STAT_T *prTimeStat;
#if CFG_SUPPORT_TX_LATENCY_STATS
	struct ADAPTER *prAdapter = prGlueInfo->prAdapter;
	struct TX_LATENCY_STATS *stats;
	uint32_t au4Success[TX_TIME_CAT_NUM];
	uint32_t au4Fail[TX_TIME_CAT_NUM];
	uint8_t i;
#endif

	prTimeStat = (struct STATS_TX_TIME_STAT_T *)(
		&prStatTlv->aucBuffer[0]);
	kalMemZero(prTimeStat->au4Success, sizeof(uint32_t) * TX_TIME_CAT_NUM);
	kalMemZero(prTimeStat->au4Fail, sizeof(uint32_t) * TX_TIME_CAT_NUM);

#if CFG_SUPPORT_TX_LATENCY_STATS
	stats = &prAdapter->rMsduReportStats.rCounting;
	for (i = 0; i < TX_TIME_CAT_NUM; i++) {
		au4Success[i] = GLUE_GET_REF_CNT(stats->au4ConnsysLatency[i]);
		au4Fail[i] = GLUE_GET_REF_CNT(stats->au4FailConnsysLatency[i]);
	}
	kalMemCopy(prTimeStat->au4Success, au4Success,
		sizeof(uint32_t) * TX_TIME_CAT_NUM);
	kalMemCopy(prTimeStat->au4Fail, au4Fail,
		sizeof(uint32_t) * TX_TIME_CAT_NUM);
#else
	DBGLOG(TX, INFO, "tx latency not support.\n");
#endif
	prStatTlv->u4Tag = STATS_TX_TAG_TIME;
	prStatTlv->u4Len = u4TlvLen;
	prTlvList += u4TlvLen;
	DBGLOG(TX, TRACE,
		"Time len=%u success=%u/%u/%u/%u/%u fail=%u/%u/%u/%u/%u\n",
		u4TlvLen, prTimeStat->au4Success[0], prTimeStat->au4Success[1],
		prTimeStat->au4Success[2], prTimeStat->au4Success[3],
		prTimeStat->au4Success[4], prTimeStat->au4Fail[0],
		prTimeStat->au4Fail[1], prTimeStat->au4Fail[2],
		prTimeStat->au4Fail[3], prTimeStat->au4Fail[4]);
}

void
statsRxReorderDropHdlr(struct GLUE_INFO *prGlueInfo,
	struct STATS_TRX_TLV_T *prTlvList, uint32_t u4TlvLen)
{
	struct ADAPTER *prAdapter = prGlueInfo->prAdapter;
	struct STATS_TRX_TLV_T *prStatTlv = prTlvList;
	uint64_t *pu8RxReorderDrop = (uint64_t *)(&prStatTlv->aucBuffer[0]);

	*pu8RxReorderDrop = RX_GET_CNT(&prAdapter->rRxCtrl,
		RX_REORDER_BEHIND_DROP_COUNT);
	prStatTlv->u4Tag = STATS_RX_TAG_REORDER_DROP;
	prStatTlv->u4Len = u4TlvLen;
	prTlvList += u4TlvLen;
	DBGLOG(RX, TRACE, "ReorderDrop len=%u val=%lu\n", u4TlvLen,
		*pu8RxReorderDrop);
}

void
statsCgsB0IdleSlotHdlr(struct GLUE_INFO *prGlueInfo,
	struct STATS_TRX_TLV_T *prTlvList, uint32_t u4TlvLen)
{
	struct ADAPTER *prAdapter = prGlueInfo->prAdapter;
	struct WIFI_LINK_QUALITY_INFO *prLinkQualityInfo;
	struct STATS_TRX_TLV_T *prStatTlv = prTlvList;
	uint64_t *pu8B0IdleSlot = (uint64_t *)(&prStatTlv->aucBuffer[0]);

	prLinkQualityInfo = &(prAdapter->rLinkQualityInfo);
	if (prLinkQualityInfo)
		*pu8B0IdleSlot = prLinkQualityInfo->u8IdleSlotCount;
	prStatTlv->u4Tag = STATS_CGS_TAG_B0_IDLE_SLOT;
	prStatTlv->u4Len = u4TlvLen;
	prTlvList += u4TlvLen;
	DBGLOG(TX, TRACE, "B0IdleSlot len=%u val=%lu\n", u4TlvLen,
		*pu8B0IdleSlot);
}

void
statsCgsAirLatHdlr(struct GLUE_INFO *prGlueInfo,
	struct STATS_TRX_TLV_T *prTlvList, uint32_t u4TlvLen)
{
	struct STATS_TRX_TLV_T *prStatTlv = prTlvList;
	struct STATS_CGS_LAT_STAT_T *prAirLat;
#if CFG_SUPPORT_LLS
	union {
		struct CMD_GET_STATS_LLS cmd;
		struct EVENT_STATS_LLS_TX_LATENCY latency;
	} query = {0};
	uint32_t u4QueryBufLen;
	uint32_t u4QueryInfoLen;
	uint32_t rStatus = WLAN_STATUS_SUCCESS;
#endif

	prAirLat = (struct STATS_CGS_LAT_STAT_T *)(&prStatTlv->aucBuffer[0]);
	kalMemZero(prAirLat->au4AirLatLvl, sizeof(uint32_t) * AIR_LAT_LVL_NUM);
	kalMemZero(prAirLat->au4AirLatMpdu, sizeof(uint32_t) * AIR_LAT_CAT_NUM);

#if CFG_SUPPORT_LLS
	u4QueryBufLen = sizeof(query);
	u4QueryInfoLen = sizeof(query.cmd);

	kalMemZero(&query, sizeof(query));
	query.cmd.u4Tag = STATS_LLS_TAG_PPDU_LATENCY;

	rStatus = kalIoctl(prGlueInfo,
			wlanQueryLinkStats,
			&query,
			u4QueryBufLen,
			TRUE,
			TRUE,
			TRUE,
			&u4QueryInfoLen);
	DBGLOG(REQ, INFO, "kalIoctl=%x, %u bytes",
				rStatus, u4QueryInfoLen);

	if (rStatus == WLAN_STATUS_SUCCESS &&
		u4QueryInfoLen == sizeof(struct EVENT_STATS_LLS_TX_LATENCY)) {
		DBGLOG(REQ, INFO, "query.latency=%u/%u/%u/%u; %u/%u/%u/%u/%u",
			query.latency.arLatencyLevel[0],
			query.latency.arLatencyLevel[1],
			query.latency.arLatencyLevel[2],
			query.latency.arLatencyLevel[3],
			query.latency.arLatencyMpduCntPerLevel[0],
			query.latency.arLatencyMpduCntPerLevel[1],
			query.latency.arLatencyMpduCntPerLevel[2],
			query.latency.arLatencyMpduCntPerLevel[3],
			query.latency.arLatencyMpduCntPerLevel[4]);
		kalMemCopy(prAirLat->au4AirLatLvl,
			query.latency.arLatencyLevel,
			sizeof(uint32_t) * AIR_LAT_LVL_NUM);
		kalMemCopy(prAirLat->au4AirLatMpdu,
			query.latency.arLatencyMpduCntPerLevel,
			sizeof(uint32_t) * AIR_LAT_CAT_NUM);
	} else if (rStatus != WLAN_STATUS_SUCCESS) {
		DBGLOG(REQ, WARN, "wlanQueryLinkStats return fail\n");
	} else {
		DBGLOG(REQ, WARN, "wlanQueryLinkStats return len unexpected\n");
	}
#else
	DBGLOG(TX, INFO, "LLS not support.\n");
#endif
	prStatTlv->u4Tag = STATS_CGS_TAG_AIR_LAT;
	prStatTlv->u4Len = u4TlvLen;
	prTlvList += u4TlvLen;
	DBGLOG(TX, TRACE, "AirLat len=%u lvl=%u/%u/%u/%u cnt=%u/%u/%u/%u/%u\n",
		u4TlvLen, prAirLat->au4AirLatLvl[0], prAirLat->au4AirLatLvl[1],
		prAirLat->au4AirLatLvl[2], prAirLat->au4AirLatLvl[3],
		prAirLat->au4AirLatMpdu[0], prAirLat->au4AirLatMpdu[1],
		prAirLat->au4AirLatMpdu[2], prAirLat->au4AirLatMpdu[3],
		prAirLat->au4AirLatMpdu[4]);
}

void
statsGetTxInfoHdlr(struct GLUE_INFO *prGlueInfo,
	struct STATS_TRX_TLV_T *paucTxTlvList)
{
	uint32_t u4TlvIdx = 0;
	uint32_t u4TlvLen = 0;
	struct STATS_TRX_TLV_T *prTlvList = paucTxTlvList;

	for (u4TlvIdx = 0; u4TlvIdx < STATS_TX_TAG_MAX_NUM; u4TlvIdx++) {
		u4TlvLen = apfnTxTbl[u4TlvIdx].pfnTlvGetLen();
		apfnTxTbl[u4TlvIdx].pfnStstsTlvHdl(prGlueInfo,
			prTlvList, u4TlvLen);
	}
}

void
statsGetRxInfoHdlr(struct GLUE_INFO *prGlueInfo,
	struct STATS_TRX_TLV_T *paucRxTlvList)
{
	uint32_t u4TlvIdx = 0;
	uint32_t u4TlvLen = 0;
	struct STATS_TRX_TLV_T *prTlvList = paucRxTlvList;

	for (u4TlvIdx = 0; u4TlvIdx < STATS_RX_TAG_MAX_NUM; u4TlvIdx++) {
		u4TlvLen = apfnRxTbl[u4TlvIdx].pfnTlvGetLen();
		apfnRxTbl[u4TlvIdx].pfnStstsTlvHdl(prGlueInfo,
			prTlvList, u4TlvLen);
	}
}

void
statsGetCgsInfoHdlr(struct GLUE_INFO *prGlueInfo,
	struct STATS_TRX_TLV_T *paucCgsTlvList)
{
	uint32_t u4TlvIdx = 0;
	uint32_t u4TlvLen = 0;
	struct STATS_TRX_TLV_T *prTlvList = paucCgsTlvList;

	for (u4TlvIdx = 0; u4TlvIdx < STATS_CGS_TAG_MAX_NUM; u4TlvIdx++) {
		u4TlvLen = apfnCgsTbl[u4TlvIdx].pfnTlvGetLen();
		apfnCgsTbl[u4TlvIdx].pfnStstsTlvHdl(prGlueInfo,
			prTlvList, u4TlvLen);
	}
}

#endif /* CFG_SUPPORT_STATISTICS */

/* End of stats.c */
