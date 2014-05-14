ifneq ($(KERNELRELEASE),)
# call from kernel build system

obj-m	:= i2c-s3c2440.o

else
KERNELDIR = /work/linux-2.6.32.2
PWD := $(shell pwd)
INSTALLDIR = /nfsroot/rootfs/work

modules:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

modules_install:
	cp *.ko $(INSTALLDIR)
    
clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions

.PHONY: modules modules_install clean 

depend .depend dep:
	$(CC) $(CFLAGS) -M *.c > .depend
endif
