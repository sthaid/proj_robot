EXTRA_CFLAGS += -g

KERNEL_ROOT=/lib/modules/$(shell uname -r)/build

gpiotest_module-y := gpiotest.o 

obj-m := gpiotest_module.o

default:
	make -C $(KERNEL_ROOT) M=`pwd`

install:
	sudo make -C $(KERNEL_ROOT) M=`pwd` modules_install
	sudo depmod

clean:
	make -C $(KERNEL_ROOT) M=`pwd` clean

load:
	sudo modprobe gpiotest_module
	
unload:
	sudo rmmod gpiotest_module

reload:
	-sudo rmmod gpiotest_module
	sudo modprobe gpiotest_module
