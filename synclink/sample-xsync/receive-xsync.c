/*
 * receive extended sync data and write received data to a file
 *
 * This sample demonstrates extended byte synchronous communications using a
 * SyncLink serial card. The companion sample send-xsync.c sends
 * extended sync data. Use both programs to send data between two serial cards
 * connected with a NULL modem (cross over cable) or other serial link.
 *
 * The sample is simple to clearly show basic programming concepts.
 * Use this code to start development of more complex applications.
 *
 * Overview:
 *
 * 1. open serial device (syscall open)
 * 2. configure serial device (syscall ioctl)
 * 3. receive data from serial device (syscall read)
 * 4. write received data to a file
 *
 * For more information about SyncLink specific programming refer to
 * the Programming.txt file included with the SyncLink software package.
 *
 * Microgate and SyncLink are registered trademarks
 * of Microgate corporation.
 *
 * This code is released under the GNU General Public License (GPL)
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
#include <memory.h>
#include <signal.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <termios.h>
#include <errno.h>

#include "synclink.h"

#ifndef N_HDLC
#define N_HDLC 13
#endif

/* handle SIGINT - do nothing */
void sigint_handler(int sigid){}

int main(int argc, char* argv[])
{
	int fd;
	int rc;
	int ldisc = N_HDLC;
	MGSL_PARAMS params;
	FILE *fp = NULL;
	char *devname;
	unsigned char buf[4096];       /* buffer for individual read calls */

	if (argc > 1)
		devname = argv[1];
	else
		devname = "/dev/ttySLG0";

	printf("receive extended sync data on %s\n", devname);

	/* open file to save received data */
	fp = fopen("data", "wb");
	if (fp == NULL) {
		printf("fopen error=%d %s\n", errno, strerror(errno));
		return errno;
	}

	/* open serial device with O_NONBLOCK to ignore DCD */
	fd = open(devname, O_RDWR | O_NONBLOCK, 0);
	if (fd < 0) {
		printf("open error=%d %s\n", errno, strerror(errno));
		return errno;
	}

	/* set N_HDLC line discipline (use for protocols other than async) */
	rc = ioctl(fd, TIOCSETD, &ldisc);
	if(rc < 0) {
		printf("set line discipline error=%d %s\n",
		       errno, strerror(errno));
		return rc;
	}

	/*
	 * set serial port options
	 *
	 * extended sync mode, loopback disabled, NRZ encoding
	 * data clocks sourced from clock input pins
	 * generate 9600 bps clock on auxclk output pin
	 * hardware CRC is not supported for xsync
	 *
	 * Parity generation/checking or CRC generation/checking
	 * must be performed by application if needed.
	 */
	params.mode = MGSL_MODE_XSYNC;
	params.loopback = 0;
	params.flags = HDLC_FLAG_RXC_RXCPIN + HDLC_FLAG_TXC_TXCPIN;
	params.encoding = HDLC_ENCODING_NRZ;
	params.clock_speed = 7372800;
	params.crc_type = HDLC_CRC_NONE;

	rc = ioctl(fd, MGSL_IOCSPARAMS, &params);
	if (rc < 0) {
		printf("ioctl(MGSL_IOCSPARAMS) error=%d %s\n",
		       errno, strerror(errno));
		return rc;
	}

	/* set 8-bit transmit idle pattern */
	rc = ioctl(fd, MGSL_IOCSTXIDLE, HDLC_TXIDLE_CUSTOM_8 | 0xaa);
	if (rc < 0) {
		printf("ioctl(MGSL_IOCSTXIDLE) error=%d %s\n",
		       errno, strerror(errno));
		return rc;
	}

        /*
	 * set extended sync pattern (1 to 4 bytes) for extended sync mode
	 *
	 * sync pattern is contained in least significant bytes of value
	 * most significant byte of sync pattern is oldest (1st sent/detected)
	 */
	rc = ioctl(fd, MGSL_IOCSXSYNC, 0x01020304);
	if (rc < 0) {
		printf("ioctl(MGSL_IOCSXSYNC) error=%d %s\n",
		       errno, strerror(errno));
		return rc;
	}

	/*
	 * set control options for extended sync mode
	 *
	 * xctrl[31:19] reserved, must be zero
	 * xctrl[18:17] extended sync pattern length in bytes
	 *              00 = 1 byte  in xsr[7:0]
	 *              01 = 2 bytes in xsr[15:0]
	 *              10 = 3 bytes in xsr[23:0]
	 *              11 = 4 bytes in xsr[31:0]
	 * xctrl[16]    1 = enable terminal count, 0=disabled
	 * xctrl[15:0]  receive terminal count for fixed length packets
	 *              value is count minus one (0 = 1 byte packet)
	 *              when terminal count is reached, receiver
	 *              automatically returns to hunt mode and receive
	 *              FIFO contents are flushed to DMA buffers with
	 *              end of frame (EOF) status
	 *
	 * 0x00070007 = 4 byte sync pattern, term count of 8 enabled
	 */
	rc = ioctl(fd, MGSL_IOCSXCTRL, 0x00070007);
	if (rc < 0) {
		printf("ioctl(MGSL_IOCSXCTRL) error=%d %s\n",
		       errno, strerror(errno));
		return rc;
	}

	/* set device to blocking mode for reads and writes */
	fcntl(fd, F_SETFL, fcntl(fd,F_GETFL) & ~O_NONBLOCK);

	/* set ctrl-C to interrupt syscall but not exit program */
	printf("Press Ctrl-C to stop program.\n");
	signal(SIGINT, sigint_handler);
	siginterrupt(SIGINT, 1);

	/*
	 * disable and then enable receiver to discard any stale
	 * bytes in receive buffers and to force receiver into
	 * 'hunt mode' where it searches for the next incoming sync pattern.
	 */
	ioctl(fd, MGSL_IOCRXENABLE, 0);
	ioctl(fd, MGSL_IOCRXENABLE, 1);


	/* process received blocks until error or ctrl-C */

	for (;;) {
		/* get fixed 8-byte packet, leading sync is discarded */
		rc = read(fd, buf, sizeof(buf));
		if (rc < 0) {
			printf("read() error=%d %s\n", errno, strerror(errno));
			break;
		}

		/* save received block to file */
		printf("block received size=%d\n", rc);
		rc = fwrite(buf, sizeof(char), rc, fp);
		fflush(fp);
	}

	close(fd);
	fclose(fp);

	return 0;
}


