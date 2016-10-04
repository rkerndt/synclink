#
# Makefile for building modules outside of the kernel source tree.
#
# Use command:
#
# make -C /usr/src/linux SUBDIRS=$PWD modules
#
# in the current directory.
#
# For kernel versions 2.5.x and 2.6.x
#

# most of these drivers are already in 2.6 kernels
# replace obj-m with line below to build all modules
#obj-m += n_hdlc.o synclink_gt.o synclink_usb.o
#obj-m += synclink_gt.o synclink_usb.o
obj-m += synclink_usb.o

clean:
	@rm -rf .tmp_versions
	@rm -f Module.symvers
	@rm -f modules.order
	@rm -f *.o *.ko  
