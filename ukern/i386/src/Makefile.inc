# boot must be first
TMP= boot.S $(SRCS)
SRCS:= $(TMP)
SRCS+= biosops.S idt.S text.S locks.S pae.S
SRCS+= ioapic.c lapic.c sysboot.c tlb.c pci.c hpet.c

# Used by MI part.
SRCS+= pmap.c machdep.c

#Used to embed init
SRCS+= empty.S

VPATH+= i386/src
