/*
 * Microgate Synclink adapter test/sample application
 *
 * $Id: mgsltest.c,v 1.12 2006-01-18 19:46:03 paulkf Exp $
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
 *	Setting the line discipline (N_HDLC or N_TTY)
 *	Setting the termios settings
 *	Controlling and monitoring modem signals
 *	Sending and receiving data (HDLC frames or asynchronous blocks).
 *
 * This program sends and receives data through a
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
 * Run the mgslutil program to set the desired communications options.
 * Examine the mgslutil source for examples
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
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <asm/ioctl.h>
#include <linux/types.h>
#include <errno.h>
#include <termios.h>
#include <signal.h>

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
int set_crc16ex(int argc, char* argv[]);
int set_crc32ex(int argc, char* argv[]);

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
	{"timeout", set_timeout},
	{"crc16ex", set_crc16ex},
	{"crc32ex", set_crc32ex},
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

char mgsltest_version[] = "$Revision: 1.12 $";

/*
 * Testing parameters 
 */
char device_name[50];
int count=100;
int tx_frame_size=1024;
int rx_frame_size=1024*2;
int master=1;
int quiet=0;
int timeout=5;
int crc_ex = 0;
int hdlc_mode=1;

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

	printf("\nmgsltest %s\n", mgsltest_version);
	
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
	printf( "\nmgsltest, command line utility to test\n"
		"a SyncLink serial adapter by sending and.\n"
		"receiving HDLC frames.\n"
		"usage: mgsltest devicename [options]\n" );
		
	printf( "options with [+/-] may be prepended with a\n"
		"+ or - character to enable or disable the option\n"
		"\n-== OPTIONS ==- \n"
		"count             number of test frames (0=infinite)\n"
		"size              size of test frames (2-65535) in bytes\n"
		"master            send frame and wait for frame to be echoed\n"
		"slave             wait for rx frame and echo it\n"
		"quiet             quiet mode (display only error messages\n"
		"timeout           set tx/rx frame timeout in seconds)\n"
		"hdlc              test in HDLC mode\n"
                "async             test in asynchronous mode\n"
		);
		
}	/* end of display_usage() */

static void trace_block(const unsigned char *data, int count, const char *label)
{
	int i;
	int linecount;
	printf("%s data:\n", label);
	while(count) {
		linecount = (count > 16) ? 16 : count;
		for(i=0; i < linecount; i++)
			printf("%02X ",(unsigned char)data[i]);
		for(;i<17;i++)
			printf("   ");
		for(i=0;i<linecount;i++) {
			if (data[i]>=040 && data[i]<=0176)
				printf("%c",data[i]);
			else
				printf(".");
		}
		printf("\n");
		data  += linecount;
		count -= linecount;
	}
}

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
	int rx_ok=0;
	int rx_timeout=0;
	int rx_errors=0;
	int tx_ok=0;
	int tx_timeout=0;
	int hdlc_disc = N_HDLC;	/* HDLC line discipline index */
	int n_tty_disc = N_TTY; /* asynchronous line discipline index */
	int old_disc;		/* original line discipline */
	MGSL_PARAMS params;
	int sigs;
	struct	sigaction sa;
	sigset_t mask;

	unsigned char *xmit_buf = malloc(tx_frame_size);
	unsigned char *rcv_buf = malloc(rx_frame_size);


	printf("mgsltest testing %u byte packets on %s\n",
		tx_frame_size, devname );


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
			
	/* create test frame */
	for(i=0;i<tx_frame_size;i++)
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
	hdlc_mode = (params.mode == MGSL_MODE_ASYNC) ? 0 : 1;

	/* get current line discipline */
	rc = ioctl(fd, TIOCGETD, &old_disc);
	if(rc < 0) {
		printf("ERROR, can't get line discipline error=%d %s\n",
				errno, strerror(errno) );
		goto cleanup;
	}

	/* set device to N_HDLC (HDLC) or N_TTY (async) line discipline */
	rc = ioctl(fd, TIOCSETD, hdlc_mode ? &hdlc_disc : &n_tty_disc);
	if(rc < 0) {
		printf("ERROR, can't set line discipline error=%d %s\n",
				errno, strerror(errno) );
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

	/* send and receive user specified number of HDLC frames */	
	while( 1 ) {
		if (master) {
			loop_count++;
			if (count) {
				if(loop_count>count)
					break;
			}


			/* prep transmit buffer */
			memset( xmit_buf, (unsigned char)loop_count, tx_frame_size );
			xmit_buf[0] = (unsigned char)0xff;
			xmit_buf[1] = (unsigned char)0xa5;
			xmit_buf[tx_frame_size-2] = (unsigned char)0xde;
			xmit_buf[tx_frame_size-1] = (unsigned char)0xad;
			
			/* reset contents of receive buffer */
			memset(rcv_buf,0,rx_frame_size);

			printf("loop#%d, sending %d bytes...",
				loop_count,tx_frame_size);
			fflush(0);
			
			/* In HDLC mode the driver accepts the whole */
			/* frame or accepts nothing if buffer are full. */
			rc = write_timed(fd,xmit_buf,tx_frame_size,timeout);
			if (!rc) {
				printf("send timeout\n");
				tx_timeout++;
				continue;
			} else if (rc<0) {
				printf("send error=%d %s\n", errno, strerror(errno));
				break;
			}				
			
			tx_ok++;
			printf("send OK...");
			fflush(0);	

			/* 
			 * wait for frame to be echoed. Allow for 2x
			 * specified user timeout, as our transmit has
			 * been accepted by the driver but is still
			 * in transmit when the write() call returns.
			 * Allow time for the frame to be transmitted
			 * and echoed back.
			 */

			rc = read_timed(fd, rcv_buf, hdlc_mode ? rx_frame_size:tx_frame_size, timeout*2);
			if (!rc) {			
				printf("receive timeout\n");
				rx_timeout++;
				continue;
			} else if (rc<0) {
				printf("receive error=%d %s\n",	errno, strerror(errno));
				break;
			}

			/* if running extended crc mode, check the 
			 * rx status byte following the rx frame/crc
			 */
			if ( crc_ex ) {
				i = rc - 1;
				if ( rcv_buf[i] == RX_OK ||  
					rcv_buf[i] == RX_CRC_ERROR ) {
					--rc;
					printf("rx frame %s, crc=",
						rcv_buf[i] == RX_OK ?
							"OK" : "crc error" );
					if ( crc_ex > 2 ) {
						printf("%08lx\n",
							*((ulong*)&rcv_buf[rc-crc_ex]));
						rc -= crc_ex;
					}
					else {
						printf("%04x\n",
							*((unsigned short*)&rcv_buf[rc-crc_ex]));
						rc -= crc_ex;
					}
				}
			}
			
			/* verify received frame */
			if (rc!=tx_frame_size) {
				printf("bad receive size=%d, expected %d\n",rc,tx_frame_size);
				rx_errors++;
			} else if (memcmp(xmit_buf,rcv_buf,rc)) {
				printf("bad receive data\n");
				trace_block(rcv_buf, rc, "rx");
				rx_errors++;
			} else {
				rx_ok++;
				printf("receive OK%c", quiet ? '\r' : '\n' );
			}

		} else {
			/* wait forever for received data */
			rc = read_timed(fd, rcv_buf, hdlc_mode ? rx_frame_size:tx_frame_size, -1);
			if (rc<0) {
				printf("receive error=%d %s\n",	errno, strerror(errno));
				break;
			}
			
			if (rc<2 || rc>tx_frame_size) {
				printf("bad receive size=%d\n",rc);
				rx_errors++;
				continue;
			}
			
			rx_ok++;
			printf("receive OK, size=%d count=%d...",rc,rx_ok);
			fflush(0);
				
			/* echo the frame */
			
			/* In HDLC mode the driver accepts the whole */
			/* frame or accepts nothing if buffer are full. */
			rc = write_timed(fd,rcv_buf,rc,timeout);
			if (!rc) {
				printf("timeout sending echo frame\n");
				tx_timeout++;
				continue;
			} else if (rc<0) {
				printf("error sending echo frame=%d %s\n",
						errno, strerror(errno) );
				break;
			}
					
			tx_ok++;
			printf("echoed send OK%c", quiet ? '\r' : '\n');
		}
	}
	
	/* display results of test */
	if (master) {
		printf( "\nMaster test results:\n"
			"transmit OK=%d, transmit timeouts=%d\n"
			"receive OK=%d, receive timeouts=%d\n"
			"receive errors=%d, lost frames=%d\n\n",
			tx_ok, tx_timeout, rx_ok, rx_timeout,
			rx_errors, tx_ok - rx_ok - rx_errors );
	} else {
		printf( "\nSlave test results:\n"
			"echoed OK=%d, echo timeouts=%d\n"
			"receive OK=%d, receive timeout=%d\n"
			"receive errors=%d\n\n",
			tx_ok, tx_timeout, rx_ok, rx_timeout,
			rx_errors );
	}
	
	display_stats(fd);	

	printf("Negating DTR and RTS\n");
	sigs = TIOCM_RTS + TIOCM_DTR;
	rc = ioctl(fd, TIOCMBIC, &sigs);
	if(rc < 0) {
		printf("%s(%d)can't negate DTR/RTS error=%d %s\n",
				__FILE__,__LINE__,errno, strerror(errno) );
		goto cleanup;
	}
	
	/* restore original line discipline */
	rc = ioctl(fd, TIOCSETD, &old_disc);
	if(rc < 0) {
		printf("can't restore line discipline error=%d %s\n",
				errno, strerror(errno));
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
}

int set_master(int argc, char* argv[])
{
	master=1;
	return 1;
}

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
	tx_frame_size = new_size;
	rx_frame_size = new_size*2;
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

int set_crc16ex(int argc, char* argv[])
{
	crc_ex = 2;
	return 1;
}

int set_crc32ex(int argc, char* argv[])
{
	crc_ex = 4;
	return 1;
}

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
 *      If hdlc_mode is set, the first successful read (a single)
 *      frame is returned immediately. Otherwise async mode is
 *      assumed and repeated reads are made until 'size' bytes
 *      are read.
 * 	
 * Arguments:
 * 
 * 	fd		handle to open file descriptor
 * 	buf		pointer to returned data buffer
 * 	size		size of buffer in bytes
 * 	timeout		timeout in seconds (-1 = wait forever)
 * 	
 * Return Value:
 * 
 * 	positive int = number of bytes returned
 * 	0 = read timeout
 * 	negative = read error
 */
int read_timed(int fd, void *buf, int size, int timeout)
{
	struct timeval tv;
	fd_set fds;
	int rc;
	char *ptr = buf;
	int count = 0;

	while(count < size) {
		if (timeout != -1) {
			/* use select() to wait for read data */

			tv.tv_sec  = timeout;
			tv.tv_usec = 0;
			FD_ZERO(&fds);
			FD_SET(fd, &fds);

			rc = select(fd+1, &fds, NULL, NULL, &tv);
			if (rc <= 0)
				return rc;
		}

		rc = read(fd, ptr, size - count);
		if ((rc <= 0) || hdlc_mode)
			return rc;

		count += rc;
		ptr += rc;
	}
	
	return count;
}

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

		case SIGINT :
			++sig_int;
		break;
	}
}

