###########################################################################
# $RCSfile: Makefile,v $
# $Revision: #2 $
# $Date: 2009/04/08 $
# $Author: allen.kao $
#
# Description:
#         Leave-level makefile to build the subcomponent of driver library.
#
#         Specify the source files to be compile in SRC.
#############################################################################

THIS_COMPONENT = usb3

ifeq "$(UBOOT_LIBRARY)" "y"

include $(TOPDIR)/config.mk

CPPFLAGS += -I$(TOPDIR)/board/$(BOARDDIR)/drv_lib/drv_inc -I$(TOPDIR)/board/$(BOARDDIR)/drv_lib/inc
CFLAGS += -I$(TOPDIR)/board/$(BOARDDIR)/drv_lib/drv_inc -I$(TOPDIR)/board/$(BOARDDIR)/drv_lib/inc
CPPFLAGS += -I$(TOPDIR)/board/$(BOARDDIR)/include -I$(OSAI_INC)
CFLAGS += -I$(TOPDIR)/board/$(BOARDDIR)/include -I$(OSAI_INC)

LIB	= lib$(THIS_COMPONENT).a


OBJS	:= LD_btmtk_usb.o LD_usbbt.o 

$(LIB):	$(OBJS) $(SOBJS)
	$(AR) crv $@ $^

clean:
	rm -f $(SOBJS) $(OBJS)

distclean:	clean
	rm -f $(LIB) core *.bak .depend

#########################################################################

.depend:	Makefile $(SOBJS:.o=.S) $(OBJS:.o=.c)
		$(CC) -M $(CPPFLAGS) $(SOBJS:.o=.S) $(OBJS:.o=.c) > $@

-include .depend

else # UBOOT_LIBRARY

ifdef LINUX_DRV_ROOT
export DRV_ROOT = $(LINUX_DRV_ROOT)
else
export DRV_ROOT = $(TARGET_OPEN_ROOT)
endif


SRC  = LD_btmtk_usb.c LD_usbbt.c

ifeq "$(USRDRV)" "true"
ifeq "$(BUILD_LINUX_LOADER)" ""
SRC += kcu_graphic.c
endif
endif

OBJ =

SUB_COMPONENTS =

OPTIONAL_SUB_COMPONENTS =

DEFINES +=

CC_INC += -I$(KERNEL_ROOT)/$(KERNEL_VER)/include -I$(DRV_ROOT)/usb3/libpayload_usb/


#############################################################################
#
# Include the makefile common to all components
#

include $(DRV_ROOT)/driver.mak


endif # UBOOT_LIBRARY

