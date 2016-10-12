#
# Makefile for SyncLink Adapter software
#
# $Id: Makefile,v 1.1.1.1 2006-06-28 16:26:46 paulkf Exp $
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

all : send-raw receive-raw

send-raw: send-raw.c ../synclink.h
	$(CC) $(CFLAGS) -I ../ -o send-raw send-raw.c

receive-raw: receive-raw.c ../synclink.h
	$(CC) $(CFLAGS) -I ../ -o receive-raw receive-raw.c


# Remove all generated files and backup files
#
CLEAN_TARGETS = *~ core send-raw receive-raw rawdata *.o
clean:
	@for f in $(CLEAN_TARGETS) ; do \
		find . -name "$$f" | xargs rm -f ; \
	done;

