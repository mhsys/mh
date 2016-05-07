INSTALLDIR=$(SRCROOT)/dist

ifeq ($(MACHINE),i386)
TOOLCHAINTARGET=i686-elf
endif

TOOLCHAINBIN= $(SRCROOT)/toolchain/install/bin
CC= $(TOOLCHAINBIN)/$(TOOLCHAINTARGET)-gcc
AS= $(TOOLCHAINBIN)/$(TOOLCHAINTARGET)-as
LD= $(TOOLCHAINBIN)/$(TOOLCHAINTARGET)-ld
AR= $(TOOLCHAINBIN)/$(TOOLCHAINTARGET)-ar
OBJCOPY= $(TOOLCHAINBIN)/$(TOOLCHAINTARGET)-objcopy
RANLIB= $(TOOLCHAINBIN)/$(TOOLCHAINTARGET)-ranlib

LN= ln
INSTALL= install

EXTSDIR=$(SRCROOT)/exts
EXTSRCDIR=$(SRCROOT)/exts
INSTALLINCDIR= $(INSTALLDIR)/usr/include
INSTALLUKINCDIR= $(INSTALLDIR)/usr/include/microkernel

CFLAGS+= -D_DREX_SOURCE -I$(INSTALLDIR)/usr/include \
	-fno-builtin -nostdinc -nostdlib -Wall \
	-D_DREX_MACHINE=$(MACHINE)
ASFLAGS+= -D_DREX_SOURCE -I$(INSTALLDIR)/usr/include \
	-fno-builtin -nostdinc -nostdlib -Wall -D_ASSEMBLER \
	-D_DREX_MACHINE=$(MACHINE)

CFLAGS+= -fno-strict-aliasing -fno-delete-null-pointer-checks

ifneq ($(DEBUG)z,z)
CFLAGS+= -D__DEBUG -O0 -g
else
CFLAGS+= -O2 -g
endif

do_all: all

ifneq ($(SUBDIRS_MKINC)z, z)
include $(addsuffix /Makefile.inc, $(SUBDIRS_MKINC))
endif

include $(MAKEDIR)/$(MACHINE)/rules.mk

