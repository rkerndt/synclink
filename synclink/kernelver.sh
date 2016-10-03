#!/bin/sh

# send version of kernel source (a.b.c.d) in directory /usr/src/linux
# to standard output
#
# optional arguments:
#
# $1   identifier to return only a portion of version (a.b.c.d):
#      VERSION        first component (a)
#      PATCHLEVEL     second component (b)
#      SUBLEVEL       third component (c)
#      VERSION_PATCH  first two components (a.b)
#
# $1 or $2   override to kernel source directory
#            if directory is $1 then no other arguments
#            are permitted and the entire version is returned

LINUX_SRC_DIR=/usr/src/linux

if [ -d $1 ] && [ -f $1/Makefile ]; then
    LINUX_SRC_DIR=$1
fi
if [ -d $2 ] && [ -f $2/Makefile ]; then
    LINUX_SRC_DIR=$2
fi
if [ ! -d $LINUX_SRC_DIR ] || [ ! -f ${LINUX_SRC_DIR}/Makefile ]; then
    exit 1	
fi

#
# Get the version of the kernel in the kernel source directory
#
KVER=`cat $LINUX_SRC_DIR/Makefile      | awk -F "[ \t=]+" "/^[ \t]*VERSION[ \t=]+/ {print \\$2}"`
KPATCH=`cat $LINUX_SRC_DIR/Makefile    | awk -F "[ \t=]+" "/^[ \t]*PATCHLEVEL[ \t]*=[ \t]*/ {print \\$2}"`
KSUB=`cat $LINUX_SRC_DIR/Makefile      | awk -F "[ \t=]+" "/^[ \t]*SUBLEVEL[ \t]*=[ \t]*/ {print \\$2}"`
KEXTRAVER=`make -C $LINUX_SRC_DIR -qp 2> /dev/null | awk -F "[ \t=]+" "/^[ \t]*EXTRAVERSION[ \t]*=[ \t]*/ {print \\$2; exit}"`

if [ -f ${LINUX_SRC_DIR}/include/linux/utsrelease.h ]; then
    KERNELVERSION=`cat $LINUX_SRC_DIR/include/linux/utsrelease.h | awk -F "[\"]+" "/UTS_RELEASE/ {print \\$2}"`
else 
if [ -f ${LINUX_SRC_DIR}/include/generated/utsrelease.h ]; then
    KERNELVERSION=`cat $LINUX_SRC_DIR/include/generated/utsrelease.h | awk -F "[\"]+" "/UTS_RELEASE/ {print \\$2}"`
else
    KERNELVERSION="${KVER}.${KPATCH}.${KSUB}${KEXTRAVER}"
fi
fi

case "$1" in
    "VERSION" ) 
	echo $KVER
	;;
    "PATCHLEVEL" ) 
	echo $KPATCH
	;;
    "SUBLEVEL" )
	echo $KSUB
	;;
    "VERSION_PATCH" )
	echo ${KVER}.${KPATCH}
	;;
    *)
	echo $KERNELVERSION
esac

exit 0

