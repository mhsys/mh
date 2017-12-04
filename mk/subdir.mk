.PHONY : subdirs clean $(SUBDIRS)
ALL_TARGET+= subdirs
CLEAN_TARGET+= clean_subdirs
INSTALL_TARGET+= install_subdirs

all: subdirs 

subdirs:
	for dir in $(SUBDIRS); do (cd $$dir && $(MAKE) all) || break 0; done

clean_subdirs:
	for dir in $(SUBDIRS); do (cd $$dir && $(MAKE) clean) || break 0; done

install_subdirs: subdirs
	for dir in $(SUBDIRS); do (cd $$dir && $(MAKE) install) || break 0; done
