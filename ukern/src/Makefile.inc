VPATH+= src

SRCS+= kern.c sys.c cpu.c bus.c usrdev.c pltdev.c pfndb.c pgalloc.c \
       fixmems.c heap.c vmap.c hwdev.c sysdev.c klogdev.c \
       xcopy.c

CUSTOBJS+= $(OBJDIR)/fixmem.o $(OBJDIR)/structs.o

$(OBJDIR)/fixmem.o: slab.c
	  $(CC) -c $(CFLAGS) -D__SLAB_FIXMEM -o $@ $^

$(OBJDIR)/structs.o: slab.c
	  $(CC) -c $(CFLAGS) -D__SLAB_STRUCTS -o $@ $^
