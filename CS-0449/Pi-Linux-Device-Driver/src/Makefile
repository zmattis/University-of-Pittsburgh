# @author Zachary M. Mattis
# CS 0449
# Pi Linux Device Driver
# July 26, 2017
#
# This Makefile provides the compilation
# instructions for the modules of pi_driver


obj-m := pi_driver.o

KDIR  := /u/SysLab/shared/linux-2.6.23.1
PWD   := $(shell pwd)

default:
	$(MAKE) -C $(KDIR) M=$(PWD) modules
