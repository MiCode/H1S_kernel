/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */


/******************************************************************************
*                         C O M P I L E R   F L A G S
*******************************************************************************
*/

/******************************************************************************
*                    E X T E R N A L   R E F E R E N C E S
*******************************************************************************
*/
#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <syslog.h>
#include <termios.h>
#include <time.h>
#include <sys/time.h>
#include <sys/poll.h>
#include <sys/param.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/uio.h>
#include <linux/serial.h> /* struct serial_struct  */

/******************************************************************************
*                              C O N S T A N T S
*******************************************************************************
*/
#ifndef N_MTKSTP
#define N_MTKSTP    (15 + 1)  /* MediaTek WCN Serial Transport Protocol */
#endif

#define HCIUARTSETPROTO _IOW('U', 200, int)
#define HCIUARTSETBAUD _IOW('U', 201, int)
#define HCIUARTGETBAUD _IOW('U', 202, int)
#define HCIUARTSETSTP _IOW('U', 203, int)
#define HCIUARTLOADPATCH _IOW('U', 204, int)
#define HCIUARTSETWAKEUP _IOW('U', 205, int)

#define CUST_COMBO_WMT_DEV "/dev/stpwmt"
#define CUST_COMBO_STP_DEV "/dev/ttyUSB0"
#define CUST_COMBO_PATCH_PATH "/etc/firmware" //-- for ALPS


#define CUST_BAUDRATE_DFT (115200)

#define CUST_MULTI_PATCH (1)

#ifdef CFG_MTK_SOC_CONSYS_SUPPORT
#define CUST_MTK_SOC_CONSYS (1)
#else
#define CUST_MTK_SOC_CONSYS (0)
#endif

typedef enum {
    STP_MIN = 0x0,
    STP_UART_FULL = 0x1,
    STP_UART_MAND = 0x2,
    STP_BTIF_FULL = 0x3,
    STP_SDIO = 0x4,
    STP_MAX = 0x5,
}STP_MODE;

#define MAX_CMD_LEN (NAME_MAX+1)

typedef enum {
    UART_DISABLE_FC = 0, /*NO flow control*/
    UART_MTK_SW_FC = 1,  /*MTK SW Flow Control, differs from Linux Flow Control*/
    UART_LINUX_FC = 2,   /*Linux SW Flow Control*/
    UART_HW_FC = 3,      /*HW Flow Control*/
} STP_UART_FC;

#ifdef ANDROID
typedef struct 
{
    const char *key;
    const char *defValue;
    char value[PROPERTY_VALUE_MAX];
    
}SYS_PROPERTY;

#endif

typedef struct {
    STP_UART_FC fc;
    int parity;
    int stop_bit;
} STP_UART_CONFIG;

typedef struct {
    STP_MODE eStpMode;
    char *pPatchPath;
    char *pPatchName;
    char *gStpDev;
    int iBaudrate;
    STP_UART_CONFIG sUartConfig;
}STP_PARAMS_CONFIG, *P_STP_PARAMS_CONFIG;


#if CUST_MULTI_PATCH
typedef struct {
    int dowloadSeq;
    char addRess[4];
    char patchName[256];
}STP_PATCH_INFO,*P_STP_PATCH_INFO;
#endif

typedef struct {
    const char *pCfgItem;
    char cfgItemValue[NAME_MAX + 1];
}CHIP_ANT_MODE_INFO, *P_CHIP_ANT_MODE_INFO;


typedef struct {
    int chipId; 
    STP_MODE stpMode;
    CHIP_ANT_MODE_INFO antMode;
}CHIP_MODE_INFO, *P_CHIP_MODE_INFO;
#ifdef ANDROID
#ifndef WMT_PLAT_APEX
CHIP_MODE_INFO gChipModeInfo[] = {
    {0x6620, STP_UART_FULL, {"mt6620.defAnt", "mt6620_ant_m3.cfg"}},
    {0x6628, STP_UART_FULL, {"mt6628.defAnt", "mt6628_ant_m1.cfg"}},
    {0x6630, STP_UART_FULL, {"mt6630.defAnt", "mt6630_ant_m1.cfg"}},
};
#else
CHIP_MODE_INFO gChipModeInfo[] = {
    {0x6620, STP_UART_FULL, {"mt6620.defAnt", "WMT.cfg"}},
    {0x6628, STP_UART_FULL, {"mt6628.defAnt", "WMT.cfg"}},
    {0x6630, STP_UART_FULL, {"mt6630.defAnt", "WMT.cfg"}},
};
#endif
#else
CHIP_MODE_INFO gChipModeInfo[] = {
    {0x6620, STP_UART_FULL, {"mt6620.defAnt", "WMT.cfg"}},
    {0x6628, STP_UART_FULL, {"mt6628.defAnt", "WMT.cfg"}},
    {0x6630, STP_UART_FULL, {"mt6630.defAnt", "WMT.cfg"}},
};
#endif
/******************************************************************************
*                             D A T A   T Y P E S
*******************************************************************************
*/
struct cmd_hdr{
    char *pCmd;
    int (*hdr_func)(P_STP_PARAMS_CONFIG pStpParamsConfig);
};

struct speed_map {
    unsigned int baud;
    speed_t      speed;
};

/******************************************************************************
*                                 M A C R O S
*******************************************************************************
*/
#define INIT_CMD(c, e, s) {.cmd= c, .cmd_sz=sizeof(c), .evt=e, .evt_sz=sizeof(e), .str=s}

/******************************************************************************
*                   F U N C T I O N   D E C L A R A T I O N S
*******************************************************************************
*/
static int set_speed(int fd, struct termios *ti, int speed);
int setup_uart_param (int hComPort, int iBaudrate, STP_UART_CONFIG *stp_uart);

int cmd_hdr_baud (P_STP_PARAMS_CONFIG pStpParamsConfig, int baudrate);
/*int cmd_hdr_baud_115k (P_STP_PARAMS_CONFIG pStpParamsConfig);
int cmd_hdr_baud_921k (P_STP_PARAMS_CONFIG pStpParamsConfig);
int cmd_hdr_baud_2kk (P_STP_PARAMS_CONFIG pStpParamsConfig);
int cmd_hdr_baud_2_5kk (P_STP_PARAMS_CONFIG pStpParamsConfig);
int cmd_hdr_baud_3kk (P_STP_PARAMS_CONFIG pStpParamsConfig);
int cmd_hdr_baud_3_2kk (P_STP_PARAMS_CONFIG pStpParamsConfig);
int cmd_hdr_baud_3_25kk (P_STP_PARAMS_CONFIG pStpParamsConfig);
int cmd_hdr_baud_3_5kk (P_STP_PARAMS_CONFIG pStpParamsConfig);
int cmd_hdr_baud_4kk (P_STP_PARAMS_CONFIG pStpParamsConfig);*/
int cmd_hdr_stp_open (P_STP_PARAMS_CONFIG pStpParamsConfig);
int cmd_hdr_stp_close (P_STP_PARAMS_CONFIG pStpParamsConfig);
int cmd_hdr_stp_rst(P_STP_PARAMS_CONFIG pStpParamsConfig);
int cmd_hdr_sch_patch (P_STP_PARAMS_CONFIG pStpParamsConfig);
static int wmt_cfg_item_parser(char *pItem);
static speed_t get_speed (int baudrate);


/******************************************************************************
*                            P U B L I C   D A T A
*******************************************************************************
*/

/******************************************************************************
*                           P R I V A T E   D A T A
*******************************************************************************
*/
static struct speed_map speeds[] = {
    {115200,    B115200},
    {921600,    B921600},
    {1000000,    B1000000},
    {1152000,    B1152000},
    {2000000,    B2000000},
    {2500000,    B2500000},
    {3000000,    B3000000},
    {3500000,    B3500000},
    {4000000,    B4000000},
};

static STP_UART_CONFIG g_stp_uart_config;

static volatile sig_atomic_t __io_canceled = 0;
static char gPatchName[NAME_MAX+1]= {0};
static char gPatchFolder[NAME_MAX+1]= {0};
static char gStpDev[NAME_MAX+1]= {0};
static int gStpMode = -1;
static char gWmtCfgName[NAME_MAX+1] = {0};
static int gWmtFd = -1;
static int gWmtBtFd = -1;
static int gTtyFd = -1;
static char gCmdStr[MAX_CMD_LEN]= {0};
static char gRespStr[MAX_CMD_LEN]= {0};
static int gFmMode = 2; /* 1: i2c, 2: comm I/F */
static const char *gUartName = NULL;

#if CUST_MULTI_PATCH
static unsigned int gPatchNum = 0;
static unsigned int gDwonSeq = 0; 
static P_STP_PATCH_INFO pStpPatchInfo = NULL;
static STP_PATCH_INFO gStpPatchInfo;
#endif
/******************************************************************************
*                              F U N C T I O N S
*******************************************************************************
*/

/* Used as host uart param setup callback */
int setup_uart_param (
    int hComPort,
    int iBaudrate,
    STP_UART_CONFIG *stpUartConfig)
{
    struct termios ti;
    int  fd;
    printf("setup_uart_param begin\n");
    if(!stpUartConfig){
        perror("Invalid stpUartConfig");
        return -2;
    }

    printf("setup_uart_param %d %d\n", iBaudrate, stpUartConfig->fc);

    fd = hComPort;
    if (fd < 0) {
        perror("Invalid serial port");
        return -2;
    }

    tcflush(fd, TCIOFLUSH);

    if (tcgetattr(fd, &ti) < 0) {
        perror("Can't get port settings");
        return -3;
    }

    cfmakeraw(&ti);

    printf("ti.c_cflag = 0x%08x\n", ti.c_cflag);
    ti.c_cflag |= CLOCAL;
    printf("CLOCAL = 0x%x\n", CLOCAL);
    printf("(ori)ti.c_iflag = 0x%08x\n", ti.c_iflag);
    printf("(ori)ti.c_cflag = 0x%08x\n", ti.c_cflag);
    printf("stpUartConfig->fc= %d (0:none,sw,hw,linux)\n", stpUartConfig->fc);

    if(stpUartConfig->fc == UART_DISABLE_FC){
        ti.c_cflag &= ~CRTSCTS;
        ti.c_iflag &= ~(0x80000000);
    } else if(stpUartConfig->fc == UART_MTK_SW_FC){
        ti.c_cflag &= ~CRTSCTS;
        ti.c_iflag |= 0x80000000; /*MTK Software FC*/
    } else if(stpUartConfig->fc == UART_HW_FC){ 
        printf("stpUartConfig->fc is UART_HW_FC\n");
        ti.c_cflag |= CRTSCTS;      /*RTS, Enable*/
        ti.c_iflag &= ~(0x80000000);
    } else if(stpUartConfig->fc == UART_LINUX_FC){
        ti.c_iflag |= (IXON | IXOFF | IXANY); /*Linux Software FC*/
        ti.c_cflag &= ~CRTSCTS;
        ti.c_iflag &= ~(0x80000000);
    }else {
        ti.c_cflag &= ~CRTSCTS;
        ti.c_iflag &= ~(0x80000000);
    }

    printf("c_c CRTSCTS = 0x%16x\n", CRTSCTS);
    printf("c_i IXON = 0x%08x\n", IXON);
    printf("c_i IXOFF = 0x%08x\n", IXOFF);
    printf("c_i IXANY = 0x%08x\n", IXANY);
    printf("(aft)ti.c_iflag = 0x%08x\n", ti.c_iflag);
    printf("(aft)ti.c_cflag = 0x%08x\n\n", ti.c_cflag);

    if (tcsetattr(fd, TCSANOW, &ti) < 0) {
        perror("Can't set port settings");
        return -4;
    }

    /* Set baudrate */
    if (set_speed(fd, &ti, iBaudrate) < 0) {
        perror("Can't set initial baud rate");
        return -5;
    }

    tcflush(fd, TCIOFLUSH);
    printf("%s done \n\n", __func__);
    return 0;
}

static speed_t get_speed (int baudrate)
{
    unsigned int idx;
    for (idx = 0; idx < sizeof(speeds)/sizeof(speeds[0]); idx++) {
        if (baudrate == (int)speeds[idx].baud) {
            return speeds[idx].speed;
        }
    }
    return CBAUDEX;
}

int set_speed(int fd, struct termios *ti, int speed)
{
    struct serial_struct ss;
    int baudenum = get_speed(speed);

    if (speed != CBAUDEX) {
        //printf("%s: standard baudrate: %d -> 0x%08x\n", __FUNCTION__, speed, baudenum);
        if ((ioctl(fd, TIOCGSERIAL, &ss)) < 0) {
            printf("%s: BAUD: error to get the serial_struct info:%s\n", __FUNCTION__, strerror(errno));
            return -1;
        }
#ifdef ANDROID
        ss.flags &= ~ASYNC_SPD_CUST;
#if defined(SERIAL_STRUCT_EXT) /*modified in serial_struct.h*/
        memset(ss.reserved, 0x00, sizeof(ss.reserved));
#endif
        ss.flags |= (1 << 13);    /*set UPFLOWLATENCY flat to tty, or serial_core will reset tty->low_latency to 0*/
        /*set standard buadrate setting*/
        if ((ioctl(fd, TIOCSSERIAL, &ss)) < 0) {
            printf("%s: BAUD: error to set serial_struct:%s\n", __FUNCTION__, strerror(errno));
            return -2;
        }
#endif
        cfsetospeed(ti, baudenum);
        cfsetispeed(ti, baudenum);
        return tcsetattr(fd, TCSANOW, ti);
    }
    else {
        printf("%s: unsupported non-standard baudrate: %d -> 0x%08x\n", __FUNCTION__, speed, baudenum);
        return -3;
    }
}

int cmd_hdr_baud (P_STP_PARAMS_CONFIG pStpParamsConfig, int baudrate) {
    STP_UART_CONFIG *gStpUartConfig = &pStpParamsConfig->sUartConfig;
    return (gTtyFd != -1) ? setup_uart_param(gTtyFd, baudrate, gStpUartConfig) : -1;
}

/*int cmd_hdr_baud_115k (P_STP_PARAMS_CONFIG pStpParamsConfig) {
    STP_UART_CONFIG *gStpUartConfig = &pStpParamsConfig->sUartConfig;
    return (gTtyFd != -1) ? setup_uart_param(gTtyFd, 115200, gStpUartConfig) : -1;
}

int cmd_hdr_baud_921k (P_STP_PARAMS_CONFIG pStpParamsConfig) {
    STP_UART_CONFIG *gStpUartConfig = &pStpParamsConfig->sUartConfig;
    return (gTtyFd != -1) ? setup_uart_param(gTtyFd, 921600, gStpUartConfig) : -1;
}

int cmd_hdr_baud_2kk (P_STP_PARAMS_CONFIG pStpParamsConfig) {
    STP_UART_CONFIG *gStpUartConfig = &pStpParamsConfig->sUartConfig;
    return (gTtyFd != -1) ? setup_uart_param(gTtyFd, 2000000, gStpUartConfig) : -1;
}

int cmd_hdr_baud_2_5kk (P_STP_PARAMS_CONFIG pStpParamsConfig) {
    STP_UART_CONFIG *gStpUartConfig = &pStpParamsConfig->sUartConfig;
    return (gTtyFd != -1) ? setup_uart_param(gTtyFd, 2500000, gStpUartConfig) : -1;
}

int cmd_hdr_baud_3kk (P_STP_PARAMS_CONFIG pStpParamsConfig) {
    STP_UART_CONFIG *gStpUartConfig = &pStpParamsConfig->sUartConfig;
    return (gTtyFd != -1) ? setup_uart_param(gTtyFd, 3000000, gStpUartConfig) : -1;
}

int cmd_hdr_baud_3_2kk (P_STP_PARAMS_CONFIG pStpParamsConfig) {
    STP_UART_CONFIG *gStpUartConfig = &pStpParamsConfig->sUartConfig;
    return (gTtyFd != -1) ? setup_uart_param(gTtyFd, 3200000, gStpUartConfig) : -1;
}

int cmd_hdr_baud_3_25kk (P_STP_PARAMS_CONFIG pStpParamsConfig) {
    STP_UART_CONFIG *gStpUartConfig = &pStpParamsConfig->sUartConfig;
    return (gTtyFd != -1) ? setup_uart_param(gTtyFd, 3250000, gStpUartConfig) : -1;
}

int cmd_hdr_baud_3_5kk (P_STP_PARAMS_CONFIG pStpParamsConfig) {
    STP_UART_CONFIG *gStpUartConfig = &pStpParamsConfig->sUartConfig;
    return (gTtyFd != -1) ? setup_uart_param(gTtyFd, 3500000, gStpUartConfig) : -1;
}

int cmd_hdr_baud_4kk (P_STP_PARAMS_CONFIG pStpParamsConfig) {
    STP_UART_CONFIG *gStpUartConfig = &pStpParamsConfig->sUartConfig;
    return (gTtyFd != -1) ? setup_uart_param(gTtyFd, 4000000, gStpUartConfig) : -1;
}*/

#define N_MTK        (15+1)
int main(int argc, char *argv[])
{
    printf("build date %s time %s\n",__DATE__,__TIME__);
    FILE *fscript = 0;
    int ld = 0;
    unsigned char cmd[2] = {0};
    struct pollfd fds;
    int fd_num = 0;
    int err = 0;
    int opt;
    int baudrate = 0;
    int chang_baud_rate = 0;
    int flow_control =0;
    char *tty_path = "/dev/ttyUSB0";

    STP_PARAMS_CONFIG sStpParaConfig;
    memset(&sStpParaConfig, 0, sizeof(sStpParaConfig));
    sStpParaConfig.sUartConfig.fc = flow_control;

    while ((opt = getopt(argc, argv, "c:f:p:")) != -1) {
        switch (opt) {
            /* debug */
            case 'c':
                printf("baudrate = %d\n", baudrate);
                baudrate = atoi(optarg);
                printf("baudrate = %d\n", baudrate);
                chang_baud_rate = 1;
                printf("baudrate = %d\n", chang_baud_rate);
                break;
            /* command Usage */
            case 'f':
                flow_control = atoi(optarg);
                printf("flow_control = %d\n", flow_control);
                sStpParaConfig.sUartConfig.fc = flow_control;
                break;
            case 'p':
            /* command Usage */
            tty_path = optarg;
            printf("Log path is %s\n", tty_path);
            case '?':
        default:
                printf("uart_launcher -c baudrate\n");
        }
    }

    /* open ttyUSB */
    printf("Running...\n");
    gTtyFd = open(tty_path, O_RDWR | O_NOCTTY | O_NONBLOCK);
    printf("open done ttyfd %d\n", gTtyFd);
    if (gTtyFd < 0) {
        printf("ttyfd %d, error\n", gTtyFd);
        return 0;
    }

    ld = N_MTK;
    if (ioctl(gTtyFd, TIOCSETD, &ld) < 0) {
        printf("set TIOCSETD N_MTK error\n");
        return 0;
    }

restart:
    /* Set default Baud rate */
    sStpParaConfig.iBaudrate = CUST_BAUDRATE_DFT;
    printf("set baudtate = %d\n", CUST_BAUDRATE_DFT);
    cmd_hdr_baud(&sStpParaConfig, CUST_BAUDRATE_DFT);
    fds.fd = gTtyFd;
    fds.events = POLLIN;
    ++fd_num;

    /*if (ioctl(gTtyFd, HCIUARTSETSTP, NULL) < 0) {
        printf("set HCIUARTSETSTP error\n");
        return 0;
    }*/

    ld = N_MTK;

    /* chang baud rate */
    if (chang_baud_rate) {
        if (ioctl(gTtyFd, HCIUARTSETBAUD, NULL) < 0) {
            printf("set HCIUARTSETBAUD error\n");
            return 0;
        }

        sStpParaConfig.iBaudrate = baudrate;
        printf("set baudtate %d\n", baudrate);
        cmd_hdr_baud(&sStpParaConfig, baudrate);
    }
    //fds.fd = gTtyFd;
    //fds.events = POLLERR | POLLHUP;

    /*if (ioctl(gTtyFd, HCIUARTGETBAUD, NULL) < 0) {
        printf("set HCIUARTSETBAUD error\n");
        return 0;
    }*/
    if (ioctl(gTtyFd, HCIUARTSETWAKEUP, NULL) < 0) {
        printf("set HCIUARTSETWAKEUP error\n");
        return 0;
    }

    if (ioctl(gTtyFd, HCIUARTLOADPATCH, NULL) < 0) {
        printf("set HCIUARTLOADPATCH error\n");
        return 0;
    }

    /*write(gTtyFd, cmd , sizeof(cmd));*/

    while (1) {
        err = poll(&fds, fd_num, 2000);
        if (err < 0) {
            if (errno == EINTR) {
                continue;
            }
            else {
                printf("poll error:%d errno:%d, %s\n", err, errno, strerror(errno));
                break;
            }
        } else if (!err) {
            if (fds.revents & POLLIN) {
                goto restart;
            } else {
                continue;
            }
        }
        goto restart;
       /*if (fds.revents & POLLIN) {
       }*/
    }
}
