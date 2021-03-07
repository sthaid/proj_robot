SUBDIRS = realtime/module \
          realtime/user_mode \
          sensors \
          motor_ctlr \
          gpio \
          encoder
  
.PHONY: build clean

build:
	for d in $(SUBDIRS) ; do echo; make -C $$d || exit 1; done

clean:
	for d in $(SUBDIRS) ; do echo; make -C $$d clean || exit 1; done

