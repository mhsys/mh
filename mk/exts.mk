.phony: exts clean_exts

exts:
	for i in $(EXTS); do rm -rf $(EXTSRCDIR)/$$i; done
	for i in $(EXTS); do (cd $(EXTSRCDIR) && if [ -f $$i.tar.gz ]; then $(TAR) xzf $$i.tar.gz; elif [ -f $$i.tar.bz2 ]; then $(TAR) xjf $$i.tar.bz2; else $(TAR) xJf $$i.tar.xz; fi; if [ -f $$i.patch ]; then $(PATCH) -p0 < $$i.patch; fi;); done

clean_exts:
	for i in $(EXTS); do rm -rf $(EXTSRCDIR)/$$i; done

ALL_PREDEP+= exts
CLEAN_TARGET+= clean_exts
