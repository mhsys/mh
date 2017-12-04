SRCROOT=../..
include $(SRCROOT)/mk/mk.conf

# Microkernel headers
UKINCS= assert.h cdefs.h cdefs_elf.h errno.h exttypes.h types.h	\
	locks.h param.h stdarg.h stdbool.h elf.h
UKINCS+= bus.h cpu.h fixmems.h gfp.h heap.h hwdev.h kern.h pcpu.h	\
	pfndb.h pgalloc.h slab.h structs.h usrdev.h vmap.h
UKINCDIR= uk

# Public
INCS= exttypes.h sys.h
INCDIR= uk
INCSUBDIRS=

include $(MAKEDIR)/ukinc.mk
include $(MAKEDIR)/inc.mk
include $(MAKEDIR)/def.mk
