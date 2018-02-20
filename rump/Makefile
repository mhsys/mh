SRCROOT=..
MKDIR=$(SRCROOT)/mk
include $(MKDIR)/mk.conf

BUILDRUMP=buildrump-20170910

SUBDIRS+= librumpmrg

ALL_PREDEP = exts build_rump
EXTS += $(BUILDRUMP)

include $(MKDIR)/sysflags.mk
export HOST_CC=cc
export EXTRA_CFLAGS=-D_FORTIFY_LEVEL=0 ${CFLAGS}
export EXTRA_LDFLAGS=${LDFLAGS}
export MKPIC=no
export RUMP_CURLWP=hypercall
export RUMPUSER_THREADS=fiber

.PHONY: build_rump clean_rump

build_rump:
	$(PWD)/$(EXTSDIR)/$(BUILDRUMP)/buildrump.sh -k\
	    -s $(PWD)/$(EXTSDIR)/$(BUILDRUMP)/src \
	    -F CFLAGS='${CFLAGS} -DHOST_NOT_POSIX -Wno-error=attributes' \
            -F LDFLAGS='${LDFLAGS}' \
            -T tools \
	    -d ${INSTALLDIR}/usr \
            -V MKMAN=no -V MKDOC=no -V MKHTML=no \
	    fullbuild

clean_rump:
	-rm -rf $(PWD)/obj $(PWD)/rump $(PWD)/tools

CLEAN_TARGET += clean_rump

include $(MKDIR)/exts.mk
include $(MKDIR)/subdir.mk
include $(MKDIR)/def.mk
