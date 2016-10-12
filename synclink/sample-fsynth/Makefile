#
# Makefile for SyncLink Adapter software
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

all : fsynth

fsynth: fsynth.c ../synclink.h
	$(CC) $(CFLAGS) -I ../ -o fsynth fsynth.c


# Remove all generated files and backup files
#
CLEAN_TARGETS = *~ core fsynth *.o
clean:
	@for f in $(CLEAN_TARGETS) ; do \
		find . -name "$$f" | xargs rm -f ; \
	done;

