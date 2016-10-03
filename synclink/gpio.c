/*
 * GPIO test program
 *
 * toggle output signal or monitor input signal
 *
 * Specify signals with OUTPUT_SIGNAL and INPUT_SIGNAL macros below.
 *
 * Connect the input signal to the output signal and
 * run one instance of this program in toggle mode and
 * another in wait mode.
 *
 * See Programming.txt for more details
 *
 * $Id: gpio.c,v 1.2 2006-02-22 20:54:15 paulkf Exp $
 *
 * Microgate and SyncLink are registered trademarks
 * of Microgate corporation.
 */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <fcntl.h>
#include <asm/ioctl.h>
#include <linux/types.h>

#include "synclink.h"

/* specify output signal to toggle and input signal to monitor */
#define OUTPUT_SIGNAL 5
#define INPUT_SIGNAL 12

/* calculate bit associated with signal */
#define OUTPUT_BIT (1 << (OUTPUT_SIGNAL - 1))
#define INPUT_BIT  (1 << (INPUT_SIGNAL - 1))

void display_usage(void)
{
	printf("usage: gpio <mode> <devname>\n"
	       "mode = t (toggle output) or w (wait on input)\n"
	       "devname = device name (/dev/ttySLG0 etc)\n");
	exit(0);
}

/* set input/output modes */

void set_direction(int fd)
{
	struct gpio_desc gpio;
	int rc;

	printf("set signal #%d to output, all others to input\n", OUTPUT_SIGNAL);

	gpio.state = 0; /* unused */
	gpio.smask = 0; /* unused */
	gpio.dmask = 0xffffffff; /* set direction of all signals */
	gpio.dir   = OUTPUT_BIT; /* 0=input 1=output */

	if ((rc = ioctl(fd, MGSL_IOCSGPIO, &gpio)) < 0) {
		fprintf(stderr, "ioctl(MGSL_IOCSGPIO) error=%d %s\n",
			errno, strerror(errno));
		exit(1);
	}

	if ((rc = ioctl(fd, MGSL_IOCGGPIO, &gpio)) < 0) {
		fprintf(stderr, "ioctl(MGSL_IOCGGPIO) error=%d %s\n",
			errno, strerror(errno));
		exit(1);
	}

	printf("current GPIO state=%08X direction=%08X\n", gpio.state, gpio.dir);
}

/* toggle output signal */

void do_toggle(int fd)
{
	unsigned int i;
	struct gpio_desc gpio;
	int rc;

	gpio.state = 0;
	gpio.smask = OUTPUT_BIT;
	gpio.dmask = 0; /* unused */
	gpio.dir   = 0; /* unused */

	for (i=1;;i++) {
		printf("%010d set output #%d to %d\n",
		       i, OUTPUT_SIGNAL, gpio.state & OUTPUT_BIT ? 1:0);
		if ((rc = ioctl(fd, MGSL_IOCSGPIO, &gpio)) < 0) {
			fprintf(stderr, "ioctl(MGSL_IOCSGPIO) error=%d %s\n",
				errno, strerror(errno));
			exit(1);
		}
		usleep(500000);
		gpio.state ^= OUTPUT_BIT;
	}
}

/* monitor and report input transitions */

void do_wait(int fd)
{
	unsigned int i;
	struct gpio_desc gpio;
	int rc;

	gpio.state = 0;
	gpio.smask = INPUT_BIT;
	gpio.dmask = 0; /* unused */
	gpio.dir   = 0; /* unused */

	for (i=1 ; ; i++) {
		printf("%010d wait for input #%d = %d\n",
		       i, INPUT_SIGNAL, gpio.state & INPUT_BIT ? 1:0);
		if ((rc = ioctl(fd, MGSL_IOCWAITGPIO, &gpio)) < 0) {
			fprintf(stderr, "ioctl(MGSL_IOCWAITGPIO) error=%d %s\n",
				errno, strerror(errno));
			exit(1);
		}
		gpio.state ^= INPUT_BIT;
	}
}

int main(int argc, char* argv[])
{
	int fd;
	int mode = 0;

	if (argc < 3)
		display_usage();

	if (!strcmp(argv[1],"t"))
		mode = 0;
	else if (!strcmp(argv[1],"w"))
		mode = 1;
	else
		display_usage();

	printf("gpio test mode=%s device=%s\n", mode ? "wait":"toggle", argv[2]);

	if ((fd = open(argv[2], O_RDWR | O_NONBLOCK, 0)) < 0) {
		fprintf(stderr, "open(%s) error=%d %s\n",
			argv[2], errno, strerror(errno));
		return fd;
	}

	set_direction(fd);

	if (mode == 0)
		do_toggle(fd);
	else
		do_wait(fd);

	close(fd);
	return 0;
}

