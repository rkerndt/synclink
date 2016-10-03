# Uncomment this to add debugging information to programs
#
#DEBUG=1

CFLAGS := -Wall -Wstrict-prototypes -fomit-frame-pointer 
ifdef DEBUG
CFLAGS += -g -O 
else
CFLAGS += -O2
endif

all : send-xsync receive-xsync

send-xsync: send-xsync.c ../synclink.h
	gcc $(CFLAGS) -I ../ -o send-xsync send-xsync.c

receive-xsync: receive-xsync.c ../synclink.h
	gcc $(CFLAGS) -I ../ -o receive-xsync receive-xsync.c


# Remove all generated files and backup files
#
CLEAN_TARGETS = *~ core send-xsync receive-xsync data *.o
clean:
	@for f in $(CLEAN_TARGETS) ; do \
		find . -name "$$f" | xargs rm -f ; \
	done;

