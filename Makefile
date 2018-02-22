SRCROOT=.
MKDIR=$(SRCROOT)/mk
SUBDIRS+= include lib/libuk lib/libmrg lib rump sys

ifneq ($(TESTS)z,z)
SUBDIRS += tests
endif

SUBDIRS+= ukern

include $(MKDIR)/subdir.mk
include $(MKDIR)/def.mk

headers: include
	make -C include