#!/bin/sh

LSMOD=/sbin/lsmod
RMMOD=/sbin/rmmod

# script to unload the SyncLink device driver and 
# HDLC line discipline driver
#
# $Id: unload-drivers.sh,v 1.5 2006-07-07 14:36:00 paulkf Exp $

( $LSMOD | grep -i "synclink_gt">/dev/null) 	&& ${RMMOD} synclink_gt
( $LSMOD | grep -i "n_hdlc"	>/dev/null)	&& ${RMMOD} n_hdlc
