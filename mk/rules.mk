TOOLCHAINTARGET_DIR= $(SRCROOT)/toolchain/install/$(TOOLCHAINTARGET)
TOOLCHAINBIN= $(PWD)/$(SRCROOT)/toolchain/install/bin
export CC= $(TOOLCHAINBIN)/$(TOOLCHAINTARGET)-gcc
export AS= $(TOOLCHAINBIN)/$(TOOLCHAINTARGET)-as
export LD= $(TOOLCHAINBIN)/$(TOOLCHAINTARGET)-ld
export AR= $(TOOLCHAINBIN)/$(TOOLCHAINTARGET)-ar
export NM= $(TOOLCHAINBIN)/$(TOOLCHAINTARGET)-nm
export OBJCOPY= $(TOOLCHAINBIN)/$(TOOLCHAINTARGET)-objcopy
export RANLIB= $(TOOLCHAINBIN)/$(TOOLCHAINTARGET)-ranlib
export STRIP=$(TOOLCHAINBIN)/$(TOOLCHAINTARGET)-strip

CP= cp
LN= ln
INSTALL= install

INSTALLINCDIR= $(INSTALLDIR)/usr/include
INSTALLUKINCDIR= $(INSTALLDIR)/usr/include/microkernel

CFLAGS+= -D_DREX_SOURCE -I $(INSTALLDIR)/usr/include \
	-D_DREX_MACHINE=$(MACHINE) -B $(TOOLCHAINTARGET_DIR)
ASFLAGS+= -D_DREX_SOURCE -I $(INSTALLDIR)/usr/include -D_ASSEMBLER \
	-D_DREX_MACHINE=$(MACHINE) -B $(TOOLCHAINTARGET_DIR)

CFLAGS+= -fno-strict-aliasing -fno-delete-null-pointer-checks

ifneq ($(DEBUG)z,z)
CFLAGS+= -D__DEBUG -O0 -g
else
CFLAGS+= -O7 -g
endif

do_all: all

ifneq ($(SUBDIRS_MKINC)z, z)
include $(addsuffix /Makefile.inc, $(SUBDIRS_MKINC))
endif

include $(MAKEDIR)/$(MACHINE)/rules.mk

