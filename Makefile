CONFIG_T5400_I2C := y
CONFIG_T5400_SPI := y

obj-m := t5400-core.o
ifeq ($(CONFIG_T5400_I2C),y)
obj-m += t5400-i2c.o
endif
ifeq ($(CONFIG_T5400_SPI),y)
obj-m += t5400-spi.o
endif

KDIR ?= /lib/modules/$(shell uname -r)/build

all:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules

clean:
	rm -rf Module.symvers modules.order *.o *.mod* *.ko .*[:word:] .tmp*
