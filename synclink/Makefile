#
# Makefile for SyncLink Adapter software
#
# Microgate and SyncLink are registered trademarks
# of Microgate Systems Ltd
#

# set these to suit your environment
SRC_PREFIX ?= /home/kerndtr/src/linux
LINUX_VERSION ?= 4.7.5

# Location of the kernel source
# Required to build drivers outside of kernel source tree.
KERNEL_SRC_DIR ?= $(SRC_PREFIX)/linux-$(LINUX_VERSION)

# Destination to install SyncLink utility programs
UTIL_INSTALL_DIR = /usr/local/bin

# device driver base names
DRIVER_NAMES = n_hdlc synclink_gt synclink_usb

# The complete kernel source version string (a.b.c.d)
KERNEL_SRC_VERSION := $(shell ./kernelver.sh ${KERNEL_SRC_DIR})

# First 2 numbers of kernel source version (a.b)
KERNEL_SRC_VERSION_PATCH := $(shell ./kernelver.sh VERSION_PATCH ${KERNEL_SRC_DIR})

# directory containing driver source code
# currently supports 2.6.x, 3.x and 4.x kernels
DRIVER_SRC_DIR := drivers/kernel-2.6

# Uncomment this to add debugging information to programs
DEBUG=1

CFLAGS := -Wall -Wstrict-prototypes -fomit-frame-pointer 
ifdef DEBUG
CFLAGS += -g -O 
else
CFLAGS += -O2
endif

# utility program and sample names
UTILS := mgslutil mgsltest sample_hdlc sample_raw sample_async sample_bisync sample_xsync gpio sethdlc

# list of subdirs so we can call make clean in them
SUB_DIRS := drivers/kernel-2.6 sample-async sample-bisync sample-hdlc sample-raw sample-xsync

all_targets all: utils
utils: $(UTILS)

# verify kernel source directory is setup correctly
# to allow building modules outside of kernel source directory
#
check_kernel_src:
	@if [ ! -d $(KERNEL_SRC_DIR) ]; then \
		echo ""; \
		echo "$(KERNEL_SRC_DIR) does not exist"; \
		echo "The Makefile variable KERNEL_SRC_DIR must be set"; \
		echo "to the directory containing the Linux kernel source."; \
		echo ""; \
		exit -1; \
	fi
ifeq ($(shell expr "$(KERNEL_SRC_VERSION_PATCH)" "<" "2.6"), 1)
	echo "";
	echo "$(KERNEL_SRC_DIR) contains a kernel version";
	echo "earlier than 2.6 and is not supported";
	echo "";
	exit -1;
endif
	@if [ ! -f $(KERNEL_SRC_DIR)/.config ]; then \
		echo ""; \
		echo "$(KERNEL_SRC_DIR)/.config does not exist"; \
		echo "The kernel source directory $(KERNEL_SRC_DIR)" ; \
		echo "must contain a kernel configuration file .config"; \
		echo ""; \
		exit -1; \
	fi
	@if [ ! -f $(KERNEL_SRC_DIR)/Module.symvers ]; then \
		echo ""; \
		echo "$(KERNEL_SRC_DIR)/Module.symvers does not exist"; \
		echo "Run 'make modules' in $(KERNEL_SRC_DIR)" ; \
		echo "to build the main kernel modules"; \
		echo ""; \
		exit -1; \
	fi
	mkdir -p $(DRIVER_SRC_DIR)/linux
	cp -f synclink.h $(DRIVER_SRC_DIR)/linux/synclink.h

# build drivers outside of the kernel source
driver: check_kernel_src
	make -C $(KERNEL_SRC_DIR) SUBDIRS=`pwd`/$(DRIVER_SRC_DIR) modules

# build HDLC sample programs
sample_hdlc:
	make -C sample-hdlc

# build raw mode sample programs
sample_raw:
	make -C sample-raw

# build asynchronous sample programs
sample_async:
	make -C sample-async

# build bisync sample programs
sample_bisync:
	make -C sample-bisync

# build xsync sample programs
sample_xsync:
	make -C sample-xsync

# SyncLink Configuration utility
mgslutil: mgslutil.c synclink.h
	gcc $(CFLAGS) -o mgslutil mgslutil.c

# SyncLink Diagnostics/Sample Application	
mgsltest: mgsltest.c synclink.h
	gcc $(CFLAGS) -o mgsltest mgsltest.c

# sample general purpose I/O program
gpio: gpio.c synclink.h
	gcc $(CFLAGS) -o gpio gpio.c

# sethdlc utility
sethdlc:
	make -C sethdlc-src
	cp sethdlc-src/sethdlc .


# compiled driver installation directory
LIB_MODULES_DIR := /lib/modules/$(KERNEL_SRC_VERSION)
ifeq ($(shell expr "$(KERNEL_SRC_VERSION)" ">=" "2.6.39"), 1)
DRIVER_INSTALL_DIR := $(LIB_MODULES_DIR)/kernel/drivers/tty
else
DRIVER_INSTALL_DIR := $(LIB_MODULES_DIR)/kernel/drivers/char
endif

# compiled driver name suffix
MODULE_SUFFIX := .ko
DRIVER_NAMES := $(addsuffix $(MODULE_SUFFIX), $(DRIVER_NAMES))

#
# install SyncLink utility programs
#
install : utils

	@if [ ! -d $(UTIL_INSTALL_DIR) ]; then \
		echo ' ** ' directory "$(UTIL_INSTALL_DIR) does not exist"; \
		exit -1; \
	fi

	@for f in $(UTILS) ; do \
		if [ -f $$f ] ; then \
	    		cp -av $$f $(UTIL_INSTALL_DIR) ; \
		fi \
	done;

#
# install drivers to lib/modules directory and run depmod
# old versions of drivers are removed
#
driver_install : driver

	@if [ ! -d $(DRIVER_INSTALL_DIR) ]; then \
		echo ' ** ' directory "$(DRIVER_INSTALL_DIR) does not exist"; \
		exit -1; \
	fi

	@for f in $(DRIVER_NAMES) ; do \
		if [ -f $(DRIVER_SRC_DIR)/$$f ] ; then \
			find $(LIB_MODULES_DIR) -name $$f -delete ; \
	    		cp -av $(DRIVER_SRC_DIR)/$$f $(DRIVER_INSTALL_DIR) ; \
		fi \
	done;

	/sbin/depmod -a $(KERNEL_SRC_VERSION) ;

#
# Remove all generated files and backup files
#
CLEAN_TARGETS = *~ core $(UTILS) *.o *.ko *.mod.c *.mod.o .*.o.flags .*o.cmd
clean:
	@for f in $(CLEAN_TARGETS) ; do \
		find . -name "$$f" | xargs rm -f ; \
	done;
	@for d in $(SUB_DIRS) ; do \
	  make -C $$d clean; \
	done;
	@rm -f  sethdlc
	@rm -f  sethdlc-src/sethdlc
	@find . -type l | xargs rm -f
	@rm -rf drivers/kernel-2.4/linux
	@rm -rf drivers/kernel-2.6/linux
