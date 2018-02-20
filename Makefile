SRCROOT=.
MKDIR=$(SRCROOT)/mk
include $(MKDIR)/mk.conf


ifneq ($(TESTS)z,z)
SUBDIRS += tests
endif

SUBDIRS+= lib rump sys ukern

INCSUBDIRS= \
	$(SRCROOT)/ukern/src \
	$(SRCROOT)/ukern/$(MACHINE)/src \
	$(SRCROOT)/lib/libuk \
	$(SRCROOT)/lib/libmrg \
	$(SRCROOT)/lib/libsquoze

include $(MKDIR)/inc.mk
include $(MKDIR)/subdir.mk
include $(MKDIR)/def.mk

.PHONY: headers toolchain
headers: include
	make -C include

toolchain:
	$(MAKE) headers
	(cd toolchain; $(MAKE) populate all)
