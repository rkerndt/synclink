/*
 * Microgate Synclink adapter test/sample HDLC application
 *
 * $Id: loopback.c,v 1.2 2001-03-26 17:04:32 ez Exp $
 *
 * Microgate and SyncLink are registered trademarks
 * of Microgate corporation.
 *
 * Original release 01/11/99
 *
 * This code is released under the GNU General Public License (GPL)
 *
 * Demonstrates:
 *
 *	Opening and closing SyncLink device.
 *	Setting the HDLC line discipline
 *	Setting the tty attributes for HDLC communications
 *	Controlling and monitoring modem signals
 *	Sending and receiving HDLC frames.
 *
 * This program sends and received HDLC frames through a
 * SyncLink adapter. If the adapter is set for internal loopback
 * then the program sends and received frames to itself and
 * ignores the modem control signals.
 *
 * If the adapter is not set for internal loopback mode then
 * the program sends a frame to an external device and expects
 * the external device to echo the same frame back. DCD must
 * be active from the external device for the test to work.
 *
 * The external device can be another SyncLink adapter running
 * this program in 'slave' mode connected through a synchronous
 * NULL modem -OR- an external loopback contector can be used.
 *
 * This program does NOT set any of the HDLC parameters.
 * The program mgslutil should be run 1st to set the desired
 * parameters. Examine the source for mgslutil for examples
 * of getting and setting the synclink parameters.
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <asm/ioctl.h>
#include <linux/types.h>
#include <errno.h>
#include <termios.h>
#include <signal.h>
#include <assert.h>

#include "synclink.h"

#ifndef N_HDLC
#define N_HDLC 13
#endif

/*
 * Handler function prototypes for individual options
 */

int set_count(int argc, char* argv[]);
int set_size(int argc, char* argv[]);
int set_slave(int argc, char* argv[]);
int set_master(int argc, char* argv[]);
int set_quiet(int argc, char* argv[]);
int set_timeout(int argc, char* argv[]);

typedef int (*CMD_SET_FUNC)(int argc, char *argv[]);
typedef struct _CMD_TABLE_ENTRY {
	char* cmd;
	CMD_SET_FUNC func;
} CMD_TABLE_ENTRY;

CMD_TABLE_ENTRY cmd_table[] = {
	{"count", set_count},
	{"size", set_size},
	{"slave", set_slave},
	{"master", set_master},
	{"quiet", set_quiet},
	{"timeout", set_timeout}
};

#define CMD_TABLE_COUNT (sizeof(cmd_table)/sizeof(CMD_TABLE_ENTRY))

int parse_cmdline(int argc, char* argv[], char* devname);
int test_device(char *devname);
int display_stats(int fd);
void display_usage(void);

int read_timed(int fd,void *buf, int size, int timeout);
int write_timed(int fd,void *buf, int size, int timeout);
static void signal_handler(int signum);

int	sig_alarm = 0;
int	sig_int  = 0;
int	sig_quit = 0;

char loopback_version[] = "1.2";

/*
 * Testing parameters
 */
char device_name[50];
int count=100;
int frame_size=1024;
int master=1;
int quiet=0;
int timeout=5;


/* main()
 *
 * 	program entry point
 *
 * Arguments:
 *
 * 	argc	count of command line arguments
 * 	argv	array of pointers to command line arguments
 *
 * Return Value:
 *
 * 	0 if success, otherwise error code
 */
int main(int argc, char* argv[])
{
	int rc;

	printf( "\nloopback v%s\n",loopback_version);

	rc = parse_cmdline(argc,argv,device_name);

	if (!rc)
		rc = test_device(device_name);

	return rc;

}	/* end of main() */

/* parse_cmdline()
 *
 * 	parse command line arguments into a device name
 * 	and a device parameters structure
 *
 * Arguments:
 *
 * 	argc		number of command line arguments
 * 	argv		array of pointers to command line arguments
 * 	devname		buffer to hold parsed device name
 *
 * Return Value:
 *
 *	0 if success, otherwise error code
 */
int parse_cmdline(int argc, char* argv[], char* devname)
{
	int rc=0;
	int i;

	if ( argc == 1 || !strcmp(argv[1],"--help") ) {
		display_usage();
		exit(0);
	}

	sprintf(devname,argv[1]);

	argc -= 2;
	argv += 2;

	while(argc) {
		for(i=0;i<CMD_TABLE_COUNT;i++) {
			if (!strcmp(cmd_table[i].cmd,*argv)) {
				rc =(*cmd_table[i].func)(argc,argv);
				if (rc<0)
					return rc;
				break;
			}
		}
		if (i==CMD_TABLE_COUNT) {
			printf("\nInvalid option %s\n",*argv);
			exit(-EINVAL);
		}
		argc -= rc;
		argv += rc;
	}

	return 0;

}	/* end of parse_cmdline() */

/* display_usage()
 *
 * 	output program usage to stdout
 *
 * Arguments:	None
 * Return Value:	None
 */
void display_usage()
{
	printf( "\nloopback, command line utility to test\n"
		"a SyncLink serial adapter by sending and.\n"
		"receiving HDLC frames.\n"
		"usage: loopback devicename [options]\n" );

	printf( "options with [+/-] may be prepended with a\n"
		"+ or - character to enable or disable the option\n"
		"\n-== OPTIONS ==- \n"
		"count             number of test frames (0=infinite)\n"
		"size              size of test frames (2-65535) in bytes\n"
		"master            send frame and wait for frame to be echoed\n"
		"slave             wait for rx frame and echo it\n"
		"quiet             quiet mode (display only error messages\n"
		"timeout           set tx/rx frame timeout in seconds)\n"
		);

}	/* end of display_usage() */

/* test_device()
 *
 * 	test the specified device
 *
 * Arguments:
 *
 * 	devname		device name
 * 	params		pointer to parameters structure
 *
 * Return Value:
 *
 * 	0 if success, otherwise error code
 */
int test_device(char *devname)
{
	int fd=-1;
	int rc=-1;
	int i;
	int loop_count=0;
	static char funcname[] = "test_device()";

	struct termios oldterm;
	struct termios newterm;
	struct termios testterm;
	int hdlc_disc = N_HDLC;	/* HDLC line discipline index */
	int old_disc;		/* original line discipline */
	MGSL_PARAMS params;
	int sigs;
	struct	sigaction sa;
	sigset_t mask;
	pid_t child;

	unsigned char *xmit_buf = malloc(frame_size);
	unsigned char *rcv_buf = malloc(frame_size);

	/* We will call fork() to spawn a new process
	 * to send frames of data at the same time the
	 * parent process is receiving. Create a memory
	 * mapped file to allow child to pass counter
	 * information back to parent process
	 */
	struct counts {
		int rx_ok;
		int rx_timeout;
		int rx_errors;
		int rx_byte_count;
		int tx_ok;
		int tx_timeout;
		int tx_byte_count;
	};
	struct counts *counts;

    	int fdmap, pagesize;

	fdmap = open("/dev/mem", O_RDWR);
	assert(-1!=fdmap);
	pagesize = getpagesize();
	assert(0!=pagesize);
    	counts = mmap((caddr_t)0, pagesize, PROT_READ|PROT_WRITE, MAP_SHARED, fdmap, 0);
	assert(NULL!=counts);

	memset(counts,0,sizeof(struct counts));

	printf("loopback testing %u byte frames on %s\n",
		frame_size, devname );

	if ( !xmit_buf || !rcv_buf )
	{
		printf("memory allocation for transmit/receive buffer failed\n");
		goto cleanup;
	}


	/* install signal handlers */
	sigemptyset(&mask);

	memset( &sa, 0, sizeof(struct sigaction) );
	sa.sa_handler = &signal_handler;
	sa.sa_mask = mask;
	sigaction( SIGALRM, &sa, NULL );
	sigaction( SIGINT, &sa, NULL );
	sigaction( SIGQUIT, &sa, NULL );
	sigaction( SIGTERM, &sa, NULL );

	/* create test frame */
	for(i=0;i<frame_size;i++)
		xmit_buf[i]=(unsigned char)i;

	/* open the specified device */
	/* use O_NONBLOCK so open returns immediately without */
	/* regard to the state of the modem signal DCD */
	fd = open(devname,O_RDWR | O_NONBLOCK,0);
	if (fd < 0) {
		printf("%s(%d):%s open on device %s failed with err=%d %s\n",
			__FILE__,__LINE__,funcname,devname,errno, strerror(errno) );
		goto cleanup;
	}

	/* make ioctl call to get current synclink parameters */
	rc = ioctl(fd,MGSL_IOCGPARAMS,&params);
	if (rc < 0) {
		printf("%s(%d):%s ioctl(MGSL_IOCGPARAMS) on device %s"
			" failed with err=%d %s\n",
			__FILE__,__LINE__,funcname,devname,errno, strerror(errno) );
		goto cleanup;
	}

	/* get current line discipline */
	rc = ioctl(fd, TIOCGETD, &old_disc);
	if(rc < 0) {
		printf("ERROR, can't get line discipline error=%d %s\n",
				errno, strerror(errno) );
		goto cleanup;
	}

	/* set device to N_HDLC line discipline */
	rc = ioctl(fd, TIOCSETD, &hdlc_disc);
	if(rc < 0) {
		printf("ERROR, can't set line discipline error=%d %s\n",
				errno, strerror(errno) );
		goto cleanup;
	}

	/* get and save current port attributes */
	tcgetattr(fd,&oldterm);
	newterm = oldterm;

	cfsetospeed(&newterm,B9600);
	cfsetispeed(&newterm,B9600);

	/* turn off tty processing to allow raw binary HDLC data */
	/* to be exchanged through the tty interface */
	newterm.c_lflag &= ~(ICANON|ECHO|ECHOCTL|ECHONL|ISIG|NOFLSH|IEXTEN);
	newterm.c_oflag &= ~(ONLCR|OPOST|OLCUC|ONOCR|OCRNL|ONLRET);
	newterm.c_iflag &= ~(ICRNL|INPCK|ISTRIP|IUCLC|IXOFF|IXON|IGNCR);

	/* setup control flags to prevent automatic manipulation of the */
	/* modem signals, which is done manually below with ioctl calls */
	newterm.c_cflag &= ~(HUPCL|CRTSCTS);
	newterm.c_cflag |= CLOCAL|CREAD;

	/* setup read to return when at least one byte available */
	newterm.c_cc[VMIN] = 1;
	newterm.c_cc[VTIME] = 0;

	/* set new port attributes */
	tcsetattr(fd,TCSANOW,&newterm);

	/* verify port attributes */
	tcgetattr(fd,&testterm);
	if (testterm.c_iflag != newterm.c_iflag ||
	    testterm.c_oflag != newterm.c_oflag ||
	    testterm.c_lflag != newterm.c_lflag ||
	    testterm.c_cflag != newterm.c_cflag) {
		printf("%s(%d)can't set port attributes\n",__FILE__,__LINE__);
		goto cleanup;
	}

	/* set device to blocking mode for reads and writes */
	fcntl(fd, F_SETFL, fcntl(fd,F_GETFL) & ~O_NONBLOCK);

	printf("Asserting DTR and RTS\n");
	fflush(0);

	sigs = TIOCM_RTS + TIOCM_DTR;
	rc = ioctl(fd, TIOCMBIS, &sigs);
	if(rc < 0) {
		printf("%s(%d)can't assert DTR/RTS error=%d %s\n",
				__FILE__,__LINE__,errno, strerror(errno) );
		goto cleanup;
	}

	if (!params.loopback) {
		int sigs;
		/* synclink is not set for loopback mode */
		/* so wait for DCD active from external device */
		printf("Waiting for DCD active...");
		fflush(0);
		for(;;) {
#if 1
			/* setup timeout for waiting for DCD to go active */
			sig_alarm = 0;
			alarm(1);

			/* use MGSL_IOCWAITEVENT to wait for DCD to go active */
			sigs = MgslEvent_DcdActive;
			rc = ioctl(fd,MGSL_IOCWAITEVENT,&sigs);

			if ( rc == 0 && sigs & MgslEvent_DcdActive ) {
				alarm(0);
				printf("DCD active!\n");
				break;
			}
#else
			/* use standard TTY TIOCMIWAIT to wait for DCD to go active */
			rc = ioctl(fd, TIOCMGET, &sigs);
			if(rc < 0) {
				printf("error getting modem signals=%d %s\n",
						errno, strerror(errno) );
				goto cleanup;
			}
			if (sigs & TIOCM_CD) {
				alarm(0);
				printf("DCD active!\n");
				break;
			}

			/* setup timeout for waiting for DCD to go active */
			sig_alarm = 0;
			alarm(1);
			rc = ioctl(fd, TIOCMIWAIT, TIOCM_CD);
#endif
			/* if call fails, check to see if timeout occurred */
			alarm(0);

			if(rc < 0) {
				if ( errno == EINTR ) {
					if ( sig_alarm ) {
						printf(".");
						fflush(0);
					}
					else if ( sig_quit ) {
						printf("SIGQUIT: DCD failed to go active, may indicate a "
								"cabling or signalling problem, ignoring DCD "
								"and continuing ...\n");
						fflush(0);
						break;
					}
					else if ( sig_int )  {
						printf("SIGINT: exitting\n");
						fflush(0);
						goto cleanup;
					}
				}
				else {
					printf("error waiting for event=%d %s\n",
							errno, strerror(errno) );
					goto cleanup;
				}
			}

		}
	} else {
		printf("Internal loopback enabled, ignoring DCD\n");
	}

	/* work loop - fork to create a new child process that will
	 * transmit a series of frames. This parent process will
	 * retrieve received frames and verify their integrity.
	 * This allows for simultaneous tx/rx operations.
	 */
	child = fork();
	if ( child == -1 ) {
		printf("%s(%d) couldn't fork,rc=%d %s\n",__FILE__,__LINE__,
				errno,strerror(errno));
		goto cleanup;
	}
	else if ( child ) {
		/* path of execution for parent process,
		 * wait for incoming frames
		 */
		int wait_on_child = 1;
		unsigned char last_seq = 0;

		while ( 1 ) {
			unsigned char this_seq;
			rc = read_timed(fd,rcv_buf,frame_size,timeout);
			if ( rc < 0 ) {
				printf("rx: read() failed, rc=%d %s\n",
						errno, strerror(errno));
				break;
			}
			else if ( rc == 0 ) {
				if ( !quiet )
					printf("rx: read timeout\n");
				rc = waitpid(child,NULL,WNOHANG);
				if ( rc <= 0 ) {
					printf("rx: waitpid() failed, rc=%d %s\n",
						errno, strerror(errno));
					break;
				}
				else if ( rc == child ) {
					if ( !quiet ) {
						wait_on_child = 0;
						printf("rx: looks like send child done, exitting\n");
					}
					break;
				}
			}
			else {

				counts->rx_byte_count += rc;

				/* verify received frame */
				if (rc!=frame_size) {
					printf("rx: frame wrong size=%d should be %d\n",rc,frame_size);
					counts->rx_errors++;
					continue;
				}

				/* reconstruct expected transmit frame */
				this_seq = rcv_buf[2];
				memset( xmit_buf, (unsigned char)this_seq, frame_size );
				xmit_buf[0] = (unsigned char)0xff;
				xmit_buf[1] = (unsigned char)0xa5;
				xmit_buf[frame_size-2] = (unsigned char)0xde;
				xmit_buf[frame_size-1] = (unsigned char)0xad;
	
				if (memcmp(xmit_buf,rcv_buf,rc)) {
					printf("rx: bad frame content\n");
					counts->rx_errors++;
					continue;
				}
	
				++last_seq;
	
				if ( last_seq != this_seq ) {
					printf("rx: frame sequence error, expected 0x%x, got 0x%x\n",
							last_seq, this_seq);
				}
	
				++counts->rx_ok;
				if ( !quiet )
					printf("rx: sequence %02x, rx_ok=%d\n",
							this_seq,counts->rx_ok);
			}

			if ( sig_int ) {
				printf("rx: termination request detected, exitting\n");
				break;
			}
		}

		if ( wait_on_child ) {
			printf("rx: signalling child process\n");
			if ( kill(child,SIGINT) < 0 )
				printf("rx: kill SIGINT failed, (%d) %s\n",
						errno,strerror(errno));

			printf("rx: waiting on child process\n");
			rc = waitpid(child,NULL,0);
			if ( rc <= 0 ) {
				printf("rx: waitpid() failed, rc=%d %s\n",
					errno, strerror(errno));
			}
			else if ( rc == child ) {
				if ( !quiet ) {
					wait_on_child = 0;
					printf("rx: looks like send child done, exitting\n");
				}
			}
		}
	}
	else {
		/* path of execution for child process,
		 * transmit a series of frames
		 */
		for (loop_count=1; count==0 || loop_count<=count; ++loop_count) {

			/* prep transmit buffer */
			memset( xmit_buf, (unsigned char)loop_count, frame_size );
			xmit_buf[0] = (unsigned char)0xff;
			xmit_buf[1] = (unsigned char)0xa5;
			xmit_buf[frame_size-2] = (unsigned char)0xde;
			xmit_buf[frame_size-1] = (unsigned char)0xad;

			/* In HDLC mode the driver accepts the whole */
			/* frame or accepts nothing if buffer are full. */
			rc = write_timed(fd,xmit_buf,frame_size,timeout);
			if (!rc) {
				printf("tx: timeout sending frame %d\n",loop_count);
				counts->tx_timeout++;
				continue;
			} else if (rc<0) {
				printf("tx: failed to send frame %d, rc=%d %s\n",
						loop_count,errno, strerror(errno));
				break;
			} else {
				counts->tx_byte_count += rc;
				counts->tx_ok++;
				if ( !quiet )
					printf("tx: send frame %d successful\n",
						loop_count);
			}

			if ( sig_int ) {
				printf("tx: termination request detected, exitting\n");
				break;
			}
		}
		printf("tx: terminating, successfully sent %d frames\n", counts->tx_ok);
		fflush(stdout);
		exit(0);
	}

	/* display results of test */
	printf( "\nMaster test results:\n"
		"transmit frames=%d, transmit bytes=%d, transmit timeouts=%d\n"
		"receive frames=%d, receive bytes=%d, receive timeouts=%d\n"
		"receive errors=%d, lost frames=%d\n\n",
		counts->tx_ok, counts->tx_byte_count, counts->tx_timeout, 
		counts->rx_ok, counts->rx_byte_count, counts->rx_timeout,
		counts->rx_errors, counts->tx_ok - counts->rx_ok - counts->rx_errors );

	display_stats(fd);

	printf("Negating DTR and RTS\n");
	sigs = TIOCM_RTS + TIOCM_DTR;
	rc = ioctl(fd, TIOCMBIC, &sigs);
	if(rc < 0) {
		printf("%s(%d)can't negate DTR/RTS error=%d %s\n",
				__FILE__,__LINE__,errno, strerror(errno) );
		goto cleanup;
	}

	/* restore port attributes */
	tcsetattr(fd,TCSANOW,&oldterm);

	/* restore original line discipline */
	rc = ioctl(fd, TIOCSETD, &old_disc);
	if(rc < 0) {
		printf("can't restore line discipline error=%d %s\n",
				errno, strerror(errno) );
		goto cleanup;
	}

cleanup:
	/* close device */
	if ( !(fd<0) )
		close(fd);

	if ( xmit_buf )
		free(xmit_buf);

	if ( rcv_buf )
		free(rcv_buf);

	return rc;

}	/* end of test_device() */

/*
 * Handler functions for individual options
 */

int set_slave(int argc, char* argv[])
{
	master=0;
	return 1;
}	/* end of set_async() */

int set_master(int argc, char* argv[])
{
	master=1;
	return 1;
}	/* end of set_hdlc() */

int set_size(int argc, char* argv[])
{
	int new_size;

	if (argc<2 || !sscanf(argv[1],"%i",&new_size)) {
		printf("\nsize option requires decimal block size in bytes\n");
		return -EINVAL;
	}
	if (new_size < 2 || new_size > 65535) {
		printf("\nsize option must be in range 2 to 65535\n");
		return -EINVAL;
	}
	frame_size = new_size;
	return 2;
}	/* end of set_size() */

int set_count(int argc, char* argv[])
{
	int new_count;

	if (argc<2 || !sscanf(argv[1],"%i",&new_count)) {
		printf("\ncount option requires decimal count of test iterations (0=infinite)\n");
		return -EINVAL;
	}
	count = new_count;
	return 2;
}	/* end of set_count() */

int set_quiet(int argc, char* argv[])
{
	quiet=1;
	return 1;
}	/* end of set_quiet() */

int set_timeout(int argc, char* argv[])
{
	int new_timeout = 0;

	if (argc<2 || !sscanf(argv[1],"%i",&new_timeout) || new_timeout==0) {
		printf("\ntimeout option requires a non-zero value\n");
		return -EINVAL;
	}
	timeout = new_timeout;
	return 2;
}	/* end of set_timeout() */


int display_stats(int fd)
{
	int rc;
	static char funcname[] = "display_stats()";
	struct mgsl_icount icount;

	/* make ioctl call to get current stats */
	rc = ioctl(fd,MGSL_IOCGSTATS,&icount);
	if (rc < 0) {
		printf("%s(%d):%s ioctl(MGSL_IOCGSTATS)"
			" failed with err=%d %s\n",
			__FILE__,__LINE__,funcname,errno,strerror(errno) );
		return rc;
	}

	printf( "\n*** Statistics ***\n"
		"irqs: DSR:%d CTS:%d DCD:%d RI:%d TX:%d RX:%d\n"
		"async stats:\n"
		"    framing=%d parity=%d overrun=%d breaks=%d bufoverrun=%d\n"
		"hdlc frame stats:\n"
		"    txok=%d txunder=%d txabort=%d txtimeout=%d\n"
		"    rxok=%d rxshort=%d rxlong=%d rxabort=%d rxoverrun=%d rxcrc=%d\n\n",
		icount.dsr,icount.cts, icount.dcd, icount.rng, icount.tx,
		icount.rx,icount.frame,icount.parity,icount.overrun,
		icount.brk,icount.buf_overrun,icount.txok,icount.txunder,
		icount.txabort,icount.txtimeout,icount.rxok,icount.rxshort,icount.rxlong,
		icount.rxabort,icount.rxover,icount.rxcrc);

	return 0;

}	/* end of display_stats() */


/* write_timed()
 *
 * 	make write call to file/device with a timeout
 * 	This call blocks (if file has O_NONBLOCK cleared)
 * 	until write data is accepted or timeout has occurred.
 *
 * Arguments:
 *
 * 	fd		handle to open file descriptor
 * 	buf		pointer to buffer with write data
 * 	size		size write data in bytes
 * 	timeout		timeout in seconds
 *
 * Return Value:
 *
 * 	positive int = number of bytes returned
 * 	0 = write timeout
 * 	negative = read error
 */
int write_timed(int fd,void *buf, int size, int timeout)
{
	struct timeval tv;
	fd_set fds;
	int rc;

	tv.tv_sec  = timeout;
	tv.tv_usec = 0;

	/* use select call to wait for data available */

	FD_ZERO(&fds);
	FD_SET(fd,&fds);
	rc = select(fd+1,NULL,&fds,NULL,&tv);
	if (rc>0)
		rc = write(fd,buf,size);

	return rc;

}	/* end of write_timed() */

/* read_timed()
 *
 * 	make read call to file/device with a timeout
 * 	This call blocks (if file has O_NONBLOCK cleared)
 * 	until read data is available or timeout has occurred.
 *
 * Arguments:
 *
 * 	fd		handle to open file descriptor
 * 	buf		pointer to returned data buffer
 * 	size		size of buffer in bytes
 * 	timeout		timeout in seconds
 *
 * Return Value:
 *
 * 	positive int = number of bytes returned
 * 	0 = read timeout
 * 	negative = read error
 */
int read_timed(int fd,void *buf, int size, int timeout)
{
	struct timeval tv;
	fd_set fds;
	int rc;

	tv.tv_sec  = timeout;
	tv.tv_usec = 0;

	/* use select call to wait for data available */

	FD_ZERO(&fds);
	FD_SET(fd,&fds);
	rc = select(fd+1,&fds,NULL,NULL,&tv);
	if (rc > 0 )
		rc = read(fd,buf,size);

	return rc;

}	/* end of read_timed() */


/*
 *
 * void signal_handler()
 *
 * signal handler for program
 *
 * Notes:
 *
 * We previously set iteration timer to occur every 100ms (or 100,000us).
 * Maintain elapsed time in milliseconds and seconds for application
 * timing purposes
 *
 * input:
 *
 * 	nothing
 *
 * returns:
 *
 * 	nothing
 *
 */

static void signal_handler(int signum)
{
	switch( signum )
	{
		case SIGALRM :
			++sig_alarm;
		break;

		case SIGQUIT :
			++sig_quit;
		break;

		case SIGTERM :
		case SIGINT :
			++sig_int;
		break;
	}
}

