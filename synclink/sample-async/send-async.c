/*
 * send asynchronous data
 *
 * This sample demonstrates asynchronous communications using a
 * SyncLink serial card. The companion sample receive-async.c receives
 * asynchronous data. Use both programs to send data between two serial cards
 * connected with a NULL modem (cross over cable) or other serial link.
 *
 * The sample is simple to clearly show basic programming concepts.
 * Use this code to start development of more complex applications.
 *
 * Overview:
 *
 * 1. open serial device (syscall open)
 * 2. configure serial device (syscall ioctl and termios library functions)
 * 3. send data to serial device (syscall write)
 *
 * Configuring a SyncLink serial device for asynchronous mode requires
 * both SyncLink specific ioctl calls and standard Linux termios functions.
 *
 * For more information about SyncLink specific programming refer to
 * the Programming.txt file included with the SyncLink software package.
 *
 * For more information about standard termios and system functions refer
 * to publicly available and third party commercial documentation.
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
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <termios.h>
#include <errno.h>

#include "synclink.h"

/*
 * set base clock frequency in cycles per second
 *
 * Data clocks are generated by dividing a fixed base clock by a 16-bit integer.
 * GT family base clock default is 14745600 (14.7456MHz)
 *
 * Other base clocks (up to 33MHz) can be installed
 * at the factory by special order. Non default values require driver
 * configuration with the actual value so correct divisors are used for
 * a specified data rate.
 */
int set_base_clock(int fd, unsigned int freq)
{
	MGSL_PARAMS params;
	int rc;

	/* fields other than mode and clock_speed are ignored */
	params.mode = MGSL_MODE_BASE_CLOCK;
	params.clock_speed = freq;
	rc = ioctl(fd, MGSL_IOCSPARAMS, &params);
	if (rc < 0) {
		printf("set base clock frequency error=%d %s\n",
		       errno, strerror(errno));
	}
	return rc;
}

/*
 * set asynchronous mode and disable internal loopback
 *
 * Other asynchronous configuration options are set using
 * the standard Linux termios functions.
 */
int set_async_mode(int fd)
{
	MGSL_PARAMS params;
	int rc;

	/* get current device parameters */
	rc = ioctl(fd, MGSL_IOCGPARAMS, &params);
	if (rc < 0) {
		printf("ioctl(MGSL_IOCGPARAMS) error=%d %s\n",
		       errno, strerror(errno));
		return rc;
	}

	params.mode = MGSL_MODE_ASYNC;
	params.loopback = 0;

	/* set current device parameters */
	rc = ioctl(fd, MGSL_IOCSPARAMS, &params);
	if (rc < 0) {
		printf("set async mode error=%d %s\n",
		       errno, strerror(errno));
	}
	return rc;
}

/*
 * set asynchronous data rate
 *
 * This function specifies data rates not supported by termios.
 * 
 */
int set_data_rate(int fd, unsigned int data_rate)
{
	MGSL_PARAMS params;
	int rc;

	/* get current device parameters */
	rc = ioctl(fd, MGSL_IOCGPARAMS, &params);
	if (rc < 0) {
		printf("ioctl(MGSL_IOCGPARAMS) error=%d %s\n",
		       errno, strerror(errno));
		return rc;
	}

	params.data_rate = data_rate;

	/* set current device parameters */
	rc = ioctl(fd, MGSL_IOCSPARAMS, &params);
	if (rc < 0) {
		printf("set data rate error=%d %s\n",
		       errno, strerror(errno));
	}
	return rc;
}

int main(int argc, char* argv[])
{
	int fd;
	int rc;
	int sigs;
	int i;
	int ldisc;
	int size = 1024;
	unsigned char buf[1024];
	char *devname;
	struct termios termios;

	if (argc > 1)
		devname = argv[1];
	else
		devname = "/dev/ttySLG0";

	printf("sending %u async bytes on %s\n", size, devname);

	/* open device with O_NONBLOCK to ignore DCD */
	fd = open(devname, O_RDWR | O_NONBLOCK, 0);
	if (fd < 0) {
		printf("open on device %s failed with err=%d %s\n",
			devname, errno, strerror(errno));
		return fd;
	}

	/* set N_TTY (standard async) line discipline */
	ldisc = N_TTY;
	rc = ioctl(fd, TIOCSETD, &ldisc);
	if (rc < 0) {
		printf("set line discipline error=%d %s\n",
		       errno, strerror(errno));
		return rc;
	}

	/* required only if custom base clock (not 14745600) installed */
//	if (set_base_clock(fd, 32000000) < 0)
//		return rc;

	/* use SyncLink specific ioctl to set asynchronous mode */
	if (set_async_mode(fd) < 0)
		return rc;

	/* set async parameters with standard Linux termios functions */

	rc = tcgetattr(fd, &termios);
	if (rc < 0) {
		printf("tcgetattr() error=%d %s\n", errno, strerror(errno));
		return rc;
	}

	termios.c_iflag = 0;
	termios.c_oflag = 0;
	termios.c_cflag = CREAD | CS8 | HUPCL | CLOCAL;
	termios.c_lflag = 0;
	termios.c_cc[VTIME] = 0;
	termios.c_cc[VMIN]  = 1;
	cfsetospeed(&termios, B9600);
	cfsetispeed(&termios, B9600);

	rc = tcsetattr(fd, TCSANOW, &termios);
	if (rc < 0) {
		printf("tcsetattr() error=%d %s\n", errno, strerror(errno));
		return rc;
	}

	/* override termios data rate only if rate not supported by termios */
//	if (set_data_rate(fd, 4000000) < 0)
//		return rc;

	/* initialize send buffer */
	for (i=0; i < size; i++)
		*(buf + i) = (unsigned char)i;

	/* set device to blocking mode for reads and writes */
	fcntl(fd, F_SETFL, fcntl(fd,F_GETFL) & ~O_NONBLOCK);

	printf("Turning on RTS and DTR\n");
	sigs = TIOCM_RTS + TIOCM_DTR;
	rc = ioctl(fd, TIOCMBIS, &sigs);
	if(rc < 0) {
		printf("assert DTR/RTS error=%d %s\n",
		       errno, strerror(errno));
		return rc;
	}

	printf("Sending %d bytes...", size);
	rc = write(fd, buf, size);
	if (rc < 0) {
		printf("write error=%d %s\n", errno, strerror(errno));
		return rc;
	}

	printf("wait for all data sent...");
	/* chose method to wait for all data sent: blocked or polling */
#if 1
	/* block until all data sent */
	rc = tcdrain(fd);
#else
	/* poll until all data sent */
	do {
		rc = ioctl(fd, TIOCOUTQ, &size);
		if (rc < 0) {
			printf("ioctl(TIOCOUTQ) error=%d %s\n",
			       errno, strerror(errno));
			break;
		}
	} while (size);
#endif
	printf("all data sent rc=%d\n", rc);

	printf("Turning off RTS and DTR\n");
	sigs = TIOCM_RTS + TIOCM_DTR;
	rc = ioctl(fd, TIOCMBIC, &sigs);
	if (rc < 0) {
		printf("can't negate DTR/RTS error=%d %s\n",
		       errno, strerror(errno));
		return rc;
	}

	close(fd);

	return 0;
}

