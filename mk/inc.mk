.PHONY: all includes clean incdir incs
ALL_TARGET+= includes

ifneq ($(INCDIR)z,z)
ALL_TARGET+= incdir
incdir:
	install -d ${INSTALLINCDIR}/${INCDIR}
else
incdir:
endif

ifneq ($(INCS)z,z)
ALL_TARGET+= incs
incs: incdir
	install -m 0644 ${INCS} ${INSTALLINCDIR}/${INCDIR}/ 
else
incs:
endif

includes: incs
	for dir in $(INCSUBDIRS); do (cd $$dir && $(MAKE) includes incdir incs) || break 0; done
