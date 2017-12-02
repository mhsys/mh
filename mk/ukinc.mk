# Internal microkernel internals
.PHONY: all clean ukincludes ukincdir ukincs
ALL_TARGET+= ukincludes

ifneq ($(UKINCDIR)z,z)
ALL_TARGET+= ukincdir
endif

ifneq ($(UKINCS)z,z)
ALL_TARGET+= ukincs
endif

ukincdir:
	install -d ${INSTALLUKINCDIR}/${UKINCDIR}

ukincs:
	install -m 0644 ${UKINCS} ${INSTALLUKINCDIR}/${UKINCDIR}/ 


ukincludes:
	for dir in $(UKINCSUBDIRS); do (cd $$dir; $(MAKE)) || break 0; done
