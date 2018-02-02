SRCROOT=.
MKDIR=$(SRCROOT)/mk
SUBDIRS+= include lib/libuk lib/libmrg lib sys

ifneq ($(TESTS)z,z)
SUBDIRS += tests
endif

SUBDIRS+= ukern

SUBDIRS+= rump

include $(MKDIR)/subdir.mk
include $(MKDIR)/def.mk

.PHONY: headers toolchain
headers: include
	make -C include

toolchain:
	$(MAKE) headers
	(cd toolchain; $(MAKE) populate all)
