export CC= $(TOOLCHAINTARGET)-gcc
export AS= $(TOOLCHAINTARGET)-as
export LD= $(TOOLCHAINTARGET)-ld
export AR= $(TOOLCHAINTARGET)-ar
export NM= $(TOOLCHAINTARGET)-nm
export OBJCOPY= $(TOOLCHAINTARGET)-objcopy
export RANLIB= $(TOOLCHAINTARGET)-ranlib
export STRIP= $(TOOLCHAINTARGET)-strip

CP= cp
LN= ln
INSTALL= install

INSTALLINCDIR= $(INSTALLDIR)/usr/include
INSTALLUKINCDIR= $(INSTALLDIR)/usr/include/microkernel

CFLAGS+= -D_DREX_SOURCE -I $(INSTALLDIR)/usr/include \
	-D_DREX_MACHINE=$(MACHINE)
ASFLAGS+= -D_DREX_SOURCE -I $(INSTALLDIR)/usr/include -D_ASSEMBLER \
	-D_DREX_MACHINE=$(MACHINE)

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

