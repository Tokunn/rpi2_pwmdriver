HOME ?= /home/tokunn
ARCH := arm
CROSS_COMPILE := $(HOME)/Program/raspberry/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian/bin/arm-linux-gnueabihf-
KPATH := $(HOME)/Program/raspberry/linux

obj-m := pwmdriver.o

all:
	make ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KPATH) M=$(PWD)  modules

clean:
	make ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KPATH) M=$(PWD) clean
