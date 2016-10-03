/*
 * This code programs the frequency synthesizer on the SyncLink GT2e/GT4e
 * PCI express serial adapters and SyncLink USB device to a specified frequency
 * and selects the synthesizer output as the adapter base clock. ONLY the GT2e/GT4e
 * cards and SyncLink USB device have this feature.
 *
 * SyncLink GT family serial adapters have a fixed 14.7456MHz base clock.
 * The serial controller generates data clocks by dividing the base clock
 * by an integer. Only discrete data clock values can be generated
 * exactly from a given base clock frequency. When an exact data clock
 * value is required that cannot be derived by dividing 14.7456MHz by
 * and integer, a different base clock value must be used. One way to
 * do this is special ordering a different fixed base clock which is
 * installed at the factory.
 *
 * The SyncLink GT2e, GT4e and USB serial devices have both
 * a 14.7456MHz fixed base clock and a variable frequency synthesizer.
 * The serial controller can use either as the base clock.
 *
 * The frequency synthesizer is an Integrated Device Technologies (IDT)
 * ICS307-3 device. The reference clock input to the synthesizer is
 * the fixed 14.7456MHz clock.
 * GT2E/GT4E uses synthesizer CLK1 output
 * USB uses synthesizer CLK3 output
 * Refer to the appropriate hardware user's manual for more details.
 *
 * The synthesizer SPI programming interface is connected to the serial
 * controller general purpose I/O signals. The GPIO portion of the serial
 * API is used to program the synthesizer with a 132 bit word. This word
 * is calculated using the IDT Versaclock software available at www.idt.com
 *
 * Contact Microgate for help in producing a programming word for a specific
 * frequency output. Several common frequencies are included in this code.
 * Maximum supported base clock is 82MHz
 *
 * Note:
 * After programming the frequency synthesizer and selecting it
 * as the base clock, each individual port of a card must be configured with
 * the set_base_clock function in the code below.
 *
 * This allows the device driver to correctly calculate divisors
 * for a specified data clock rate. All ports on a card share
 * a common base clock.
 */

#include <stdio.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <asm/ioctl.h>
#include <linux/types.h>

#include "synclink.h"

void display_usage(void)
{
	printf("usage: fsynth <devname>\n"
	       "devname = device name (/dev/ttySLG0 etc)\n");
}

/*
 * set one or more GPIO outputs to a specified value
 *
 * bit   bit mask identifies which GPIO output to set
 * val   new value for outputs identified by bit
 */
void set_gpio(int fd, int bit, int val)
{
	struct gpio_desc gpio;
	int rc;

	gpio.state = (val << bit);
	gpio.smask = (1 << bit);
	gpio.dmask = 0; /* unused */
	gpio.dir   = 0; /* unused */

	if ((rc = ioctl(fd, MGSL_IOCSGPIO, &gpio)) < 0) {
		fprintf(stderr, "ioctl(MGSL_IOCSGPIO) error=%d %s\n",
			errno, strerror(errno));
	}
}

/*
 * set_mux    select between fixed base clock (0) and freq synth (1)
 * set_clk    set SPI CLK pin value
 * set_select set SPI CS pin value
 * set_data   set SPI DI pin value
 */

/* GPIO bit positions for GT2E/GT4E cards */
#define GT4E_MUX  15
#define GT4E_CLK  14
#define GT4E_SEL  13
#define GT4E_DATA 12

/* GPIO bit positions for USB device */
#define USB_MUX   23
#define USB_CLK   22
#define USB_SEL   21
#define USB_DATA  20

/* GPIO bit positions (must be initialized for correct device type) */
int gpio_mux;
int gpio_clk;
int gpio_sel;
int gpio_data;

#define set_mux(fd, val)    set_gpio((fd), gpio_mux, (val))
#define set_clk(fd, val)    set_gpio((fd), gpio_clk, (val))
#define set_select(fd, val) set_gpio((fd), gpio_sel, (val))
#define set_data(fd, val)   set_gpio((fd), gpio_data, (val))

/*
 * Each frequency table entry contains an output frequency
 * and the associated 132 bit synthesizer programming word
 * to produce the frequency.
 *
 * The programming word comes from the Versaclock 2 software
 * parsed into 5 32-bit integers. The final 4 bits are placed
 * in the most significant 4 bits of the final 32-bit integer.
 */

struct freq_table_entry {
	unsigned int freq;    /* output frequency */
	unsigned int data[5]; /* device programming word */
};

/*
 * GT2e/GT4e
 *
 * Base Clock = dedicated 14.7456MHz oscillator
 *
 * ICS307-3:
 * - reference clock = 14.7456MHz oscillator
 * - VDD = 3.3V
 * - CLK1 (pin 8) output drives FPGA fsynth input
 */
struct freq_table_entry gt4e_table[] =
{
	{12288000, {0x29BFDC00, 0x61200000, 0x00000000, 0x0000A5FF, 0xA0000000}},
	{14745600, {0x38003C05, 0x24200000, 0x00000000, 0x000057FF, 0xA0000000}},
	{16000000, {0x280CFC02, 0x64A00000, 0x00000000, 0x000307FD, 0x20000000}},
	{16384000, {0x08001402, 0xA1200000, 0x00000000, 0x0000A5FF, 0xA0000000}},
	{20000000, {0x00001403, 0xE0C00000, 0x00000000, 0x00045E02, 0xF0000000}},
	{24000000, {0x00001405, 0x61400000, 0x00000000, 0x0004D204, 0x30000000}},
	{30000000, {0x20267C05, 0x64C00000, 0x00000000, 0x00050603, 0x30000000}},
	{32000000, {0x21BFDC00, 0x5A400000, 0x00000000, 0x0004D206, 0x30000000}},
	{45056000, {0x08001406, 0xE0200000, 0x00000000, 0x000217FE, 0x20000000}},
	{64000000, {0x21BFDC00, 0x12000000, 0x00000000, 0x000F5E14, 0xF0000000}},
	{0, {0, 0, 0, 0, 0}} /* final entry must have zero freq */
};

/*
 * SyncLink USB
 *
 * Base Clock = ICS307-3 CLK1 (pin 8) output (power up default = 14.7456MHz)
 *
 * ICS307-3:
 * - reference clock = 14.7456MHz xtal
 * - VDD = 3.3V
 * - CLK3 (pin 14) output drives FPGA fsynth input
 * - CLK1 (pin 8)  output drives FPGA base clock input
 *
 * Note: CLK1 and CLK3 outputs must always be driven to prevent floating
 * clock inputs to the FPGA. When calculating programming word with Versaclock,
 * select same output on CLK1 and CLK3 or select CLK1 as multiple of CLK3.
 */
struct freq_table_entry usb_table[] =
{
	{12288000, {0x28401400, 0xE5200000, 0x00000000, 0x00009BFF, 0xA0000000}},
	{14745600, {0x28481401, 0xE5200000, 0x00000000, 0x0000A5FF, 0xA0000000}},
	{16000000, {0x284C1402, 0x64A00000, 0x00000000, 0x000307FD, 0x20000000}},
	{16384000, {0x28501402, 0xE4A00000, 0x00000000, 0x0001F9FE, 0x20000000}},
	{20000000, {0x205C1404, 0x65400000, 0x00000000, 0x00068205, 0xF0000000}},
	{24000000, {0x20641405, 0x65400000, 0x00000000, 0x0004D204, 0x30000000}},
	{30000000, {0x20641405, 0x64C00000, 0x00000000, 0x00050603, 0x30000000}},
	{32000000, {0x206C1406, 0x65400000, 0x00000000, 0x00049E03, 0xF0000000}},
	{45056000, {0x28701406, 0xE4200000, 0x00000000, 0x000217FE, 0x20000000}},
	{64000000, {0x20781400, 0x4D400000, 0x00000000, 0x00049E03, 0xF0000000}},
	{0, {0, 0, 0, 0, 0}} /* final entry must have zero freq */
};

struct freq_table_entry *freq_table;

/*
 * identify device type (USB or PCIe) and select
 * appropriate frequency table and GPIO pin positions
 *
 * return 0 on success or error code
 */
int detect_device_type(int fd)
{
	struct gpio_desc gpio;
	int rc;
	unsigned int usb_mask;

	/* make mask of expected USB GPIO outputs */
	usb_mask = (1 << USB_MUX) + (1 << USB_CLK) + (1 << USB_SEL) + (1 << USB_DATA);

	/* get GPIO direction settings */
	gpio.smask = 0;
	gpio.dmask = 0xffffffff;
	if ((rc = ioctl(fd, MGSL_IOCGGPIO, &gpio)) < 0) {
		fprintf(stderr, "ioctl(MGSL_IOCGGPIO) error=%d %s\n",
			errno, strerror(errno));
		return rc;
	}

	/* select device based on GPIO direction */
	if ((gpio.dir & usb_mask) == usb_mask) {
		printf("USB device detected\n");
		gpio_mux   = USB_MUX;
		gpio_clk   = USB_CLK;
		gpio_sel   = USB_SEL;
		gpio_data  = USB_DATA;
		freq_table = usb_table;
	} else {
		printf("PCIe device detected\n");
		gpio_mux   = GT4E_MUX;
		gpio_clk   = GT4E_CLK;
		gpio_sel   = GT4E_SEL;
		gpio_data  = GT4E_DATA;
		freq_table = gt4e_table;
	}
	return 0;
}

/*
 * configure device driver with new base clock frequency
 * so correct divisors are calculated for a specified data rate
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
		fprintf(stderr, "set base clock frequency error=%d %s\n",
			errno, strerror(errno));
	}
	return rc;
}

/*
 * program frequency synthesizer with data for specified frequency
 * return 0 on success or error code
 */
int set_clock_word(int fd, unsigned int freq)
{
	int i;
	unsigned int dword_val = 0;
	struct freq_table_entry *entry;
	unsigned int *data = NULL;

	/* search for entry for requested output frequency */

	for (entry = freq_table ; entry->freq ; entry++) {
		if (entry->freq == freq) {
			printf("Found programming information for output frequency = %uHz\n", freq);
			data = entry->data;
			break;
		}
	}
	if (!data) {
		printf("No programming information for output frequency = %uHz\n", freq);
		printf("Use Versaclock software to create programming information.\n");
		printf("Operation aborted.\n");
		return -1;
	}

	set_clk(fd, 0);

	/* write 132 bit clock program word */

	for (i = 0 ; i < 132 ; i++) {
		if (!(i % 32))
			dword_val = data[i/32];
		set_data(fd, (dword_val & (1 << 31)) ? 1 : 0);
		set_clk(fd, 1);
		set_clk(fd, 0);
		dword_val <<= 1;
	}

	set_select(fd, 1);
	set_select(fd, 0);

	return 0;
}

int main(int argc, char* argv[])
{
	int fd;
	int rc;
	unsigned int freq;

	/* edit this line to select a frequency in the table */
	freq = 24000000;

	if (argc < 2) {
		display_usage();
		return 1;
	}

	printf("fsynth device=%s\n", argv[1]);

	if ((fd = open(argv[1], O_RDWR | O_NONBLOCK, 0)) < 0) {
		fprintf(stderr, "open error=%d %s\n",
			errno, strerror(errno));
		return fd;
	}

	rc = detect_device_type(fd);
	if (rc)
		return rc;

	/* program frequency synthesizer */
	rc = set_clock_word(fd, freq);
	if (rc)
		return rc;

	/* select frequency synthesizer as the base clock */
	set_mux(fd, 1);

	/*
	 * set_base_clock() must be called for EVERY port on a
	 * multi port card and the value MUST match the frequency of the
	 * synthesizer output configured by set_clock_word()
	 */
	set_base_clock(fd, freq);

	close(fd);
	return 0;
}

