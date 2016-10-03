CC = gcc
CFLAGS = -g -Wall -W -Wno-long-long -pipe

all:		sethdlc

sethdlc:	sethdlc.c
		$(CC) $(CFLAGS) -o $@ $<

install:
		install -C -p -s sethdlc /usr/local/sbin/

clean:
		rm -f sethdlc
