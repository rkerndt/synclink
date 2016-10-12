#
# Makefile for SyncLink Adapter software
#
# $Id: Makefile,v 1.1.1.1 2006-07-13 19:20:55 paulkf Exp $
#
# Written by Paul Fulghum paulkf@microgate.com
# for Microgate Corporation
#
# Microgate and SyncLink are registered trademarks
# of Microgate Corporation
#

# Uncomment this to add debugging information to programs
#
#DEBUG=1

CFLAGS := -Wall -Wstrict-prototypes -fomit-frame-pointer 
ifdef DEBUG
CFLAGS += -g -O 
else
CFLAGS += -O2
endif

all : send-async receive-async

send-async: send-async.c ../synclink.h
	$(CC) $(CFLAGS) -I ../ -o send-async send-async.c

receive-async: receive-async.c ../synclink.h
	$(CC) $(CFLAGS) -I ../ -o receive-async receive-async.c


# Remove all generated files and backup files
#
CLEAN_TARGETS = *~ core send-async receive-async data *.o
clean:
	@for f in $(CLEAN_TARGETS) ; do \
		find . -name "$$f" | xargs rm -f ; \
	done;

