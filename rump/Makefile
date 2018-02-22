SRCROOT=..
MKDIR=$(SRCROOT)/mk
include $(MKDIR)/mk.conf

BUILDRUMP=buildrump-20170910

SUBDIRS+= librumpmrg

ALL_PREDEP = exts build_rump
EXTS += $(BUILDRUMP)

include $(MKDIR)/sysflags.mk
export HOST_CC=cc
export EXTRA_CFLAGS=${CFLAGS}
export EXTRA_LDFLAGS=${LDFLAGS}
export MKPIC=no
export RUMP_CURLWP=hypercall
export RUMPUSER_THREADS=fiber

.PHONY: build_rump clean_rump

build_rump:
	$(PWD)/$(EXTSDIR)/$(BUILDRUMP)/buildrump.sh -q -k \
	    -s $(PWD)/$(EXTSDIR)/$(BUILDRUMP)/src \
	    -F CFLAGS='${CFLAGS} -DHOST_NOT_POSIX' \
            -F LDFLAGS='${LDFLAGS}' \
	    -d ${INSTALLDIR}/usr \
	    fullbuild

clean_rump:
	rm -rf $(PWD)/obj $(PWD)/rump


# Add this if you want rump components to be rebuilt every time you
# clean the rest of the source code.
#
#CLEAN_TARGET += clean_rump
#

include $(MKDIR)/exts.mk
include $(MKDIR)/lib.mk
include $(MKDIR)/subdir.mk
include $(MKDIR)/def.mk
