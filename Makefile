obj-m	:= usbtemp_cyc.o

KERNELDIR ?= /srv/scratch/yua.chen/vm/linux/
PWD       := $(shell pwd)

all:
	$(MAKE) -C $(KERNELDIR) M=$(PWD)

clean:
	rm -f *.o *~ core .depend .*.cmd *.ko *.mod.c 
	rm -f Module.markers Module.symvers modules.order
	rm -rf .tmp_versions