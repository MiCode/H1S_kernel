#ifndef __XGBE_USR_OPT__
#define __XGBE_USR_OPT__

#if !defined PF_DRIVER && !defined VF_DRIVER
    // If No Options are selected , PF_DRIVER & NO_ELI as Build Option
    #define PF_DRIVER   1
    #define NO_ELI      1
#endif

// Uncomment Macros based on the need to build driver with the Kernel.

//#define PF_DRIVER		1
//#define NO_ELI		1
//#define VF_DRIVER     1
//#define BRCM_IPC	    0

#if defined PF_DRIVER && defined VF_DRIVER
    #error "Select Either PF_DRIVER or VF_DRIVER not Both"
#endif

//Define below to 1 to toggle PCIe PERST Pin on ROCK5B (RK3588)
//Toggle is necessary for BCM8956x/BCM8957x to support PCIe Link Awareness
//after power cycle of the switch

//Change Function "rk_pcie_reset_ep" in xgbe-drv.c on the chip/Hardware
#define ROCK5B_PERST_TOGGLE 0

//XGBE has 12 Queues , with 9,10 Queues un-unused. 
//First 8 Queues are used for Tagged packets and respective priorities
//11 for untagged, 12 for Multicast/Broadcast.
//The queue sizes should be multiple of 256 bytes. 
//The default Queue sizes are 1-8 : 6656 (6.5K) , 9-10: 0, 11-12: 6K

//To Support Jumbo Frames on Untagged change macros
//XGBE_TAGGED_QUEUE_LEN to 5888
//XGBE_UNTAGGED_QUEUE_LEN to 9216

#define XGBE_TAGGED_QUEUE_LEN   6656
#define XGBE_UNTAGGED_QUEUE_LEN 6144

#if (((XGBE_TAGGED_QUEUE_LEN * 8) + (XGBE_UNTAGGED_QUEUE_LEN * 2)) > (64 * 1024))
    #error "Total Queue Size Should not Exceed 64K"
#endif

#ifdef PF_DRIVER
#define XGBE_SRIOV_PF 1
#else
#define XGBE_SRIOV_PF 0
#endif

#ifdef VF_DRIVER
#define XGBE_SRIOV_VF 1
#else
#define XGBE_SRIOV_VF 0
#endif

/* ELI Enable macro.
 *
 * ELI_ENABLE 1 -> supports ELI mode
 * ELI_ENABLE 0 -> supports Non-ELI mode
 * Non-ELI mode supports only PF Driver
 *
 * While compiling VF driver set this macro to 0
 */

#ifdef VF_DRIVER
#define ELI_ENABLE		0
#define BRCM_BCMUTIL    0
#define XGBE_AER_SUPPORT      0
#endif

#ifdef PF_DRIVER

#ifdef NO_ELI
#define ELI_ENABLE	0
#else
#define ELI_ENABLE	1
#endif

#ifdef NO_AER
#define XGBE_AER_SUPPORT      0
#else
#define XGBE_AER_SUPPORT      1
#endif

#define BRCM_BCMUTIL    1

#endif


/* Broadcom Flexible header support */

#ifdef FLEX_HEADER
#define BRCM_FH	1
#else
#define BRCM_FH	0
#endif

#endif
