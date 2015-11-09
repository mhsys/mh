SRCROOT=../..
include $(SRCROOT)/mk/mk.conf

# Microkernel headers
UKINCS= assert.h cdefs.h cdefs_elf.h types.h locks.h param.h stdarg.h	\
	stdbool.h elf.h
UKINCS+= aperture.h fixmems.h gfp.h heap.h kern.h pfndb.h pgalloc.h	\
	slab.h structs.h vmap.h
UKINCDIR= uk

# Public
INCS= sys.h
INCDIR= uk
INCSUBDIRS=

include $(MAKEDIR)/ukinc.mk
include $(MAKEDIR)/inc.mk
include $(MAKEDIR)/def.mk
