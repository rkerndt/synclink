#!/bin/sh

# Demonstrate loading synclink device driver and supporting n_hdlc driver
# manually using Linux modprobe utility.
#
# !!! WARNING !!!
# Linux normally loads drivers automatically making this script unnecessary.
# Only use this script if your system does not load drivers automatically OR
# you manually unload and reload drivers for testing.
#
# Synclink drivers load automatically when PCI or USB devices are detected
# and the system automatically creates device nodes.
#
# The n_hdlc line discipline driver loads automatically when an application
# selects the n_hdlc line discipline with ioctl(TIOCSETD).
#
# Set options by modifying script variables below.

# Linux utilities modprobe and lsmod to load and list modules
MODPROBE=/sbin/modprobe
if [ ! -f $MODPROBE ]; then
    MODPROBE=modprobe
fi
LSMOD=/bin/lsmod
if [ ! -f $LSMOD ]; then
    LSMOD=lsmod
fi

#--------------------------------------------------------------------------
# Set variable values specifying driver load options passed to modprobe.
# Do not alter these values unless you understand their purpose.
#--------------------------------------------------------------------------

# n_hdlc line discipline driver options
# This driver provides a formatting layer above the synclink driver.
# It is used by custom serial applications and diagnostics.
#
# N_HDLC_DEBUG_LEVEL
# 0 = off, 1-5 = increasing level of detail
# default if not specified = 0
# Use only when debugging problems. Debugging degrades performance and fills
# the system log.
#
# N_HDLC_MAX_FRAME_SIZE
# Maximum HDLC frame size in bytes.
# Range: 4096 to 65535
# Default if not specified 4096
# For rare applications requiring frames greater than 4096 bytes, set value to
# largest expected frame size. Larger maximum frame size consumes more memory.

#N_HDLC_DEBUG_LEVEL="0"
#N_HDLC_MAX_FRAME_SIZE="8192"


# synclink driver options
#
# SYNCLINK_DEBUG_LEVEL
# 0 = off, 1-5 = increasing level of detail
# default if not specified = 0
# Use only when debugging problems. Debugging degrades performance and fills
# the system log.
#
# SYNCLINK_MAX_FRAME_SIZE
# Maximum HDLC frame size in bytes.
# Range: 4096 to 65535
# Default if not specified 4096
# For rare applications requiring frames greater than 4096 bytes, set value to
# largest expected frame size. Larger maximum frame size consumes more memory.
# Set maximum HDLC frame size. Normally HDLC frames are 4KByte or smaller.
# Set value for each adapter separated by commas.
# Example:"4096,8192" means ttySLG0=4096 and ttySLG1=8192

#SYNCLINK_DEBUG_LEVEL="0"
#SYNCLINK_MAX_FRAME_SIZE="8192,8192"


#--------------------------------------------------------------------------
# Build command lines based on the above configuration
#--------------------------------------------------------------------------

# n_hdlc line discipline driver options

N_HDLC_OPTIONS=""
if [ $N_HDLC_DEBUG_LEVEL ] ; then
    N_HDLC_OPTIONS="${N_HDLC_OPTIONS} debuglevel=${N_HDLC_DEBUG_LEVEL}"
fi
if [ $N_HDLC_MAX_FRAME_SIZE ] ; then
    N_HDLC_OPTIONS="${N_HDLC_OPTIONS} maxframe=${N_HDLC_MAX_FRAME_SIZE}"
fi

# synclink device driver options

SYNCLINK_OPTIONS=""
if [ -f /etc/synclink.conf ] ; then
    . /etc/synclink.conf
    SYNCLINK_OPTIONS="$SCRIPT_ARGS $SYNCLINK_OPTIONS"
fi
if [ $SYNCLINK_DEBUG_LEVEL ] ; then
    SYNCLINK_OPTIONS="${SYNCLINK_OPTIONS} debug_level=${SYNCLINK_DEBUG_LEVEL}"
fi
if [ $SYNCLINK_MAX_FRAME_SIZE ] ; then
    SYNCLINK_OPTIONS="${SYNCLINK_OPTIONS} maxframe=${SYNCLINK_MAX_FRAME_SIZE}"
fi


#--------------------------------------------------------------------------
# load specified driver and create the device nodes
#
# Args:
#	name		driver module name
#	prefix		device name prefix
#	adapters	number of adapters
#--------------------------------------------------------------------------
load_and_mknode() {
    DRIVER=$1
    DEVICE_PREFIX=$2
    NUM_ADAPTERS=$3

    # load driver
    if ! $LSMOD | grep -i "$DRIVER" > /dev/null ; then
	echo "loading $DRIVER" ;
        $MODPROBE $DRIVER $SYNCLINK_OPTIONS || {
    	    echo "Can't load $DRIVER driver."
	    return 1
        }
    else
	echo "$DRIVER already loaded"
	return 0
    fi

    if [ -z $NUM_ADAPTERS ] ; then
	return 0
    fi

    # create device name list using prefix and numbers starting with 0
    DEVNAMES=
    adapter=0;
    while [ $adapter -lt $NUM_ADAPTERS ] ; do
        DEVNAMES="${DEVNAMES} /dev/${DEVICE_PREFIX}${adapter}"
        adapter=$((adapter+1))
    done

    # create/replace device nodes with device major number from
    # /proc/devices and minor numbers starting at 64
    echo "Removing existing device nodes ..."
    rm -f ${DEVNAMES}
        
    echo "Creating new device nodes ..."
    ttymajor=`cat /proc/devices | awk "\\$2==\"$DEVICE_PREFIX\" {print \\$1}"`
    ttyminor=64
    for device in ${DEVNAMES} ; do 
        mknod ${device} c $ttymajor $ttyminor
        ttyminor=$(($ttyminor + 1))
    done

    # set example group/permissions (your required values may be different)
    chgrp root ${DEVNAMES}
    chmod 666  ${DEVNAMES}
}



#--------------------------------------------------------------------------
# load drivers if not already loaded
#--------------------------------------------------------------------------

# manually load n_hdlc line discipline driver
if ! $LSMOD | grep -i "n_hdlc" > /dev/null ; then
    echo "loading n_hdlc" ;
    $MODPROBE n_hdlc $N_HDLC_OPTIONS || {
	echo "Can't load n_hdlc driver."
	exit 1
    }
else
    echo "n_hdlc driver already loaded"
fi

# manually load synclink_gt driver (create device nodes automatically)
load_and_mknode synclink_gt

# manually load synclink_gt driver and create 4 device nodes with prefix ttySLG
#
# Use this only if your system does not automatically create device nodes
# when the driver is loaded. Normally device nodes are created automatically
# even when manually loading the driver with modprobe.
#
#load_and_mknode synclink_gt ttySLG 4

exit 0;





