/*
 * Microgate Synclink adapter configuration utility
 *
 * Microgate and SyncLink are registered trademarks
 * of Microgate corporation.
 *
 * This code is released under the GNU General Public License (GPL)
 *
 * This program allows the user to set the SyncLink parameters
 * for serial mode (async/HDLC/raw) and serial options from the command
 * line and shell scripts.
 *
 * This code also serves as an example of how to programmatically
 * get/set/display the SyncLink parameters.
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
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <sys/socket.h>
#include <net/if.h>
#include <errno.h>
#include <stdio.h>
#include <memory.h>

#include "synclink.h"

#define PP_KEEPALIVE    0x01    /* use keepalive protocol */
#define PP_CISCO        0x02    /* use Cisco protocol instead of PPP */
#define SPPPIOCCISCO  (SIOCDEVPRIVATE)
#define SPPPIOCPPP    (SIOCDEVPRIVATE+1)
#define SPPPIOCDEBUG  (SIOCDEVPRIVATE+2)
#define SPPPIOCSFLAGS (SIOCDEVPRIVATE+3)
#define SPPPIOCGFLAGS (SIOCDEVPRIVATE+4)

/*
 * Handler function prototypes for individual options
 */
 
int set_async(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_hdlc(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_raw(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_bisync(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_monosync(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_loopback(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_noloopback(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_noloopback(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_ua7(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_ua15(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_uflag(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_ucrc(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_sharezero(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_nosharezero(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_autocts(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_noautocts(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_autodcd(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_noautodcd(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_autorts(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_noautorts(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_rtsen(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_nortsen(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_rxc(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_rxdpll(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_rxbrg(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_rxtxc(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_txc(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_txdpll(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_txbrg(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_txrxc(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_dplldiv8(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_dplldiv16(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_dplldiv32(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_nrz(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_nrzi(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_nrzb(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_nrzimark(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_bimark(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_bispace(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_bilevel(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_bidiff(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_clock(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_addr(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_crcnone(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_crc16(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_crc32(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_crc16ex(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_crc32ex(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_prenone(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_preone(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_prezero(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_pre10(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_pre01(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_preflag(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_pre8(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_pre16(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_pre32(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_pre64(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_rate(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_databits(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_paritynone(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_parityodd(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_parityeven(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_stopbits(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_idle0(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_idle1(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_idleflag(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_idle01(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_idlemark(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_idlespace(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_idlemarkspace(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int display_stats(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_idle0(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_idle1(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_idle01(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_idleflag(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_idlemark(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_idlespace(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_idlemarkspace(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_idle_custom(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int clearmodcount(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int
*idlemode);
int set_quiet(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_hdlcloop(int argc, char * argv[], char *devname, MGSL_PARAMS* params, int *idlemode);
int set_nohdlcloop(int argc, char * argv[], char *devname, MGSL_PARAMS* params, int *idlemode);
int set_cisco(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_keepalive(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int clear_cisco(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int clear_keepalive(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_ifdisable(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_rs232(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_v35(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int set_rs422(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);

typedef int (*CMD_SET_FUNC)(int argc, char *argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
typedef struct _CMD_TABLE_ENTRY {
	char* cmd;
	CMD_SET_FUNC func;
} CMD_TABLE_ENTRY;

CMD_TABLE_ENTRY cmd_table[] = {
	{"async", set_async},
	{"hdlc", set_hdlc},
	{"raw", set_raw},
	{"bisync", set_bisync},
	{"monosync", set_monosync},
	{"loopback", set_loopback},
	{"+loopback", set_loopback},
	{"-loopback", set_noloopback},
	{"ua7", set_ua7},
	{"ua15", set_ua15},
	{"uflag", set_uflag},
	{"ucrc", set_ucrc},
	{"sharezero", set_sharezero},
	{"+sharezero", set_sharezero},
	{"-sharezero", set_nosharezero},
	{"autocts",set_autocts},
	{"+autocts",set_autocts},
	{"-autocts",set_noautocts},
	{"autodcd",set_autodcd},
	{"+autodcd",set_autodcd},
	{"-autodcd",set_noautodcd},
	{"autorts",set_autorts},
	{"+autorts",set_autorts},
	{"-autorts",set_noautorts},
	{"+rtsen",set_rtsen},
	{"-rtsen",set_nortsen},
	{"hdlcloop",set_hdlcloop},
	{"+hdlcloop",set_hdlcloop},
	{"-hdlcloop",set_nohdlcloop},
	{"rxc",set_rxc},
	{"rxdpll",set_rxdpll},
	{"rxbrg",set_rxbrg},
	{"rxtxc",set_rxtxc},
	{"txc",set_txc},
	{"txdpll",set_txdpll},
	{"txbrg",set_txbrg},
	{"txrxc",set_txrxc},
	{"dplldiv8",set_dplldiv8},
	{"dplldiv16",set_dplldiv16},
	{"dplldiv32",set_dplldiv32},
	{"nrz",set_nrz},
	{"nrzi",set_nrzi},
	{"nrzb",set_nrzb},
	{"nrzimark",set_nrzimark},
	{"bimark",set_bimark},
	{"bispace",set_bispace},
	{"bilevel",set_bilevel},
	{"bidiff",set_bidiff},
	{"clock",set_clock},
	{"addr",set_addr},
	{"crcnone",set_crcnone},
	{"crc16",set_crc16},
	{"crc32",set_crc32},
	{"prenone",set_prenone},
	{"prezero",set_prezero},
	{"preone",set_preone},
	{"pre10",set_pre10},
	{"pre01",set_pre01},
	{"preflag",set_preflag},
	{"pre8",set_pre8},
	{"pre16",set_pre16},
	{"pre32",set_pre32},
	{"pre64",set_pre64},
	{"rate",set_rate},
	{"paritynone",set_paritynone},
	{"parityodd",set_parityodd},
	{"parityeven",set_parityeven},
	{"databits",set_databits},
	{"stopbits",set_stopbits},
	{"stats",display_stats},
	{"idle0",set_idle0},
	{"idle1",set_idle1},
	{"idleflag",set_idleflag},
	{"idle01",set_idle01},
	{"idlemark",set_idlemark},
	{"idlespace",set_idlespace},
	{"idlemarkspace",set_idlemarkspace},
	{"idle",set_idle_custom},
	{"clrmodcount",clearmodcount},
	{"quiet",set_quiet},
	{"cisco",set_cisco},
	{"+cisco",set_cisco},
	{"-cisco",clear_cisco},
	{"keepalive",set_keepalive},
	{"+keepalive",set_keepalive},
	{"-keepalive",clear_keepalive},
	{"crc16ex",set_crc16ex},
	{"crc32ex",set_crc32ex},
	{"ifdisable",set_ifdisable},
	{"rs232",set_rs232},
	{"v35",set_v35},
	{"rs422",set_rs422},
};

#define CMD_TABLE_COUNT (sizeof(cmd_table)/sizeof(CMD_TABLE_ENTRY))

int parse_cmdline(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode);
int get_params(char *devname, MGSL_PARAMS* params, int *idlemode);
int apply_params(char *devname, MGSL_PARAMS* params, int idlemode);
void display_params(char *devname, MGSL_PARAMS* params, int idlemode);
void display_usage(void);
int quiet=0;
int cisco=1;
int keepalive=0;
int syncppp_options_set=0;
int programmable_if=0;
int if_mode = MGSL_INTERFACE_DISABLE;

char mgslutil_version[] = "v1.20";

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
	char devname[100];
	MGSL_PARAMS params;
	MGSL_PARAMS orig_params;
	int idlemode;
	int rc;
	
	if ( argc == 1 || !strcmp(argv[1],"--help") ) {
		display_usage();
		exit(0);
	}
	
	sprintf(devname,argv[1]);

	/* SyncLink PC Card has programmable interface */
	if (strstr(devname,"ttySLP") || strstr(devname,"ttySLG") ||
	    strstr(devname,"ttyUSB"))
		programmable_if = 1;
	
	rc = get_params(devname,&params,&idlemode);
	if (rc<0)
		exit(rc);
	
	argc -= 2;
	argv += 2;
	
	memcpy(&orig_params,&params,sizeof(orig_params));

	rc = parse_cmdline(argc,argv,devname,&params,&idlemode);
	
	if (!quiet) {
		printf( "\nmgslutil %s\n",mgslutil_version);
		display_params(devname,&params,idlemode);
	}
		
	if (!rc && argc)
		rc = apply_params(devname,&params,idlemode);
	
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
 * 	params		buffer to hold parsed device parameters structure
 *
 * Return Value:
 *
 *	0 if success, otherwise error code
 */
int parse_cmdline(int argc, char* argv[], char* devname,
	MGSL_PARAMS* params, int *idlemode)
{
	int rc=0;
	int i;
	
	while(argc) {
		for(i=0;i<CMD_TABLE_COUNT;i++) {
			if (!strcmp(cmd_table[i].cmd,*argv)) {
				rc =(*cmd_table[i].func)(argc,argv,devname,params,idlemode);
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
	printf( "\nmgslutil, command line utility to view and alter\n"
		"device paramters for a SyncLink serial adapter.\n"
		"usage: mgslutil devicename [options]\n" );
		
	printf( "options with [+/-] may be prepended with a\n"
		"+ or - character to enable or disable the option\n"
		"\n-== COMMON OPTIONS ==- \n"
		"stats             display device statistics\n"
		"hdlc              set mode to bit synchronous HDLC\n"
		"async             set mode to asynchronous\n"
		"raw               set mode to raw synchronous\n"
		"\n-== Serial Interface (not supported by all adapters) ==- \n"
		"ifdisable         disable serial interface\n"
		"rs232             select rs232\n"
		"v35               select v.35\n"
		"rs422             select rs422 (RS449/RS530/X.21)\n"
		"[+/-]loopback     set/clear internal loopback mode\n"
		"\n-== HDLC ONLY OPTIONS ==- \n"
		"crcnone           do not append/check CRC on HDLC frames\n"
		"crc16             append/check CRC-16 CCITT on HDLC frames\n"
		"crc32             append/check CRC-32 CCITT on HDLC frames\n"
		"\n"
		"crc16ex           append/check CRC-16 CCITT on HDLC frames,\n"
		"crc32ex           append/check CRC-32 CCITT on HDLC frames\n"
		"                  (return crc info on all received frames)\n"
		"\ntransmit underrun options\n"
		"   ua7            send 7-bit abort\n"
		"   ua15           send 15-bit abort\n"
		"   uflag          send closing flag\n"
		"   ucrc           send accumulated crc + closing flag\n"
		"\ntransmit preamble pattern/length options\n"
		"   prenone        disable preamble\n"
		"   prezero        all zeros\n"
		"   preone         all ones\n"
		"   preflag        all flags\n"
		"   pre10          alternating 1/0\n"
		"   pre01          alternating 0/1\n"
		"   pre8           8 bits\n"
		"   pre16          16 bits\n"
		"   pre32          32 bits\n"
		"   pre64          64 bits\n"
		"\ntransmit idle mode options\n"
		"   idle0          continuous zeros\n"
		"   idle1          continuous ones\n"
		"   idleflag       continuous flags\n"
		"   idle01         alternating 0/1\n"
		"   idlemark       continuous mark\n"
		"   idlespace      continuous space\n"
		"   idlemarkspace  alternating mark/space\n"
		"   idle <x>       custom 8 or 16 bit hex idle\n"
		"\nHDLC tx and rx data clock options\n"
		"   rxc            rx clock comes from rxclock pin\n"
		"   rxdpll         rx clock comes from DPLL\n"
		"   rxbrg          rx clock comes from BRG\n"
		"   rxtxc          rx clock comes from txclock pin\n"
		"   txc            tx clock comes from txclock pin\n"
		"   txdpll         tx clock comes from DPLL\n"
		"   txbrg          tx clock comes from BRG\n"
		"   txrxc          tx clock comes from rxclock pin\n"
		"   dplldiv8       DPLL bit rate = reference clock divided by 8\n"
		"   dplldiv16      DPLL bit rate = reference clock divided by 16\n"
		"   dplldiv32      DPLL bit rate = reference clock divided by 32\n\n"
		"[+/-]sharezero    opening/closing flags share 0 in send frames\n"
		"[+/-]autocts      only send frames while CTS active\n"
		"[+/-]autodcd      can only receive while DCD active\n"
		"[+/-]autorts      assert RTS before sending frame, drop after sent\n"
		"[+/-]rtsen        RTS enables/disables RS-485/422 drivers\n"
		"[+/-]hdlcloop     run as HDLC/SDLC LoopMode Secondary Station\n\n"
		"\nSerial encoding options:\n"
		"   nrz            NRZ\n"
		"   nrzi           NRZI SPACE\n"
		"   nrzb           NRZB\n"
		"   nrzimark       NRZI MARK\n"
		"   bimark         BIPHASE MARK\n"
		"   bispace        BIPHASE SPACE\n"
		"   bilevel        BIPHASE LEVEL\n"
		"   bidiff         DIFFERENTIAL BIPHASE LEVEL\n\n"
		"clock <rate>      generate clock on AUXCLK pin at <rate>\n"
		"                  0 = disabled\n"
		"addr <byte>       set 8 bit receive HDLC addr to <byte>\n"
		"                  value is hexadecimal. FF=accept all\n"
		"\n-== ASYNC ONLY OPTIONS ==- \n"
		"rate <speed>      set async data rate to speed (decimal)\n"
		"paritynone        disable parity\n"
		"parityodd         use odd parity\n"
		"parityeven        use even parity\n"
		"databits <n>      async character has <n> data bits (5-8)\n"
		"stopbits <n>      async character has <n> stop bits (1-2)\n"
		"\n-== SYNCPPP/CISCO OPTIONS ==-\n"
		"[+/-]cisco        enable/disable Cisco HDLC mode (disabled = PPP mode)\n"
		"                  if not specified, defaults to Cisco HDLC\n"
		"[+/-]keepalive    enable/disable PPP/Cisco HDLC keepalive\n"
		"                  if not specified, defaults to disabled\n");
		
		
}	/* end of display_usage() */

/* display_params()
 * 
 * 	display the specified device parameters to stdout
 * 	
 * Arguments:
 * 
 * 	devname		device name
 * 	params		pointer to parameters structure
 *	idlemode	transmit idle mode
 * 	
 * Return Value:
 * 
 * 	None
 */
void display_params(char *devname, MGSL_PARAMS* params, int idlemode)
{
	char *str,*str2;
	char strbuf[255];
	static char unknown_str[] = "unknown";
	static char enabled_str[] = "enabled";
	static char disabled_str[] = "disabled";
	
	printf("*** %s Settings ***\n", devname);	

	if (programmable_if) {
		if (strstr(devname,"ttySLP")) {
			programmable_if = 1;
			switch(if_mode & MGSL_INTERFACE_MASK) {
			case MGSL_INTERFACE_DISABLE: str = "Disabled"; break;
			case MGSL_INTERFACE_RS232:   str = "RS-232 "; break;
			case MGSL_INTERFACE_V35:     str = "V.35"; break;
			case MGSL_INTERFACE_RS422:   str = "RS-422 (RS-449/RS-530/X.21)"; break;
			default:	             str = unknown_str;
			}
			printf("serial interface=%s\n", str);
		}

		if (strstr(devname,"ttySLG")) {
			printf("RTS driver control %s\n",
			       (if_mode & MGSL_INTERFACE_RTS_EN) ? enabled_str : disabled_str);
		}
	}

	switch(params->mode) {
	case MGSL_MODE_ASYNC:	 str = "Async"; break;
	case MGSL_MODE_HDLC:	 str = "HDLC "; break;
	case MGSL_MODE_RAW:	 str = "Raw  "; break;
	case MGSL_MODE_BISYNC:   str = "Bisync"; break;
	case MGSL_MODE_MONOSYNC: str = "Monosync"; break;
	default:		 str = unknown_str;
	}
	
	printf("mode=%s\n", str);
	printf("loopback=%s\n", params->loopback ? enabled_str : disabled_str);
	
	printf("flags=");

	if (params->flags & HDLC_FLAG_UNDERRUN_ABORT15)
		printf("UNDERRUN_ABORT15 ");
	else if (params->flags & HDLC_FLAG_UNDERRUN_FLAG)
		printf("UNDERRUN_FLAG ");
	else if (params->flags & HDLC_FLAG_UNDERRUN_CRC)
		printf("UNDERRUN_CRC ");
	else
		printf("UNDERRUN_ABORT7 ");
	
	if (params->flags & HDLC_FLAG_SHARE_ZERO)
		printf("SHARE_ZERO ");
	
	if (params->flags & HDLC_FLAG_AUTO_CTS)
		printf("AUTO_CTS ");
	
	if (params->flags & HDLC_FLAG_AUTO_DCD)
		printf("AUTO_DCD ");
	
	if (params->flags & HDLC_FLAG_AUTO_RTS)
		printf("AUTO_RTS ");
	
	if (params->flags & HDLC_FLAG_RXC_DPLL)
		printf("RXC_DPLL ");
	else if (params->flags & HDLC_FLAG_RXC_BRG)
		printf("RXC_BRG ");
	else if ( params->flags & HDLC_FLAG_RXC_TXCPIN )
		printf("RXC_TXCPIN ");
	else
		printf("RXC_RXCPIN ");

	if (params->flags & HDLC_FLAG_TXC_DPLL)
		printf("TXC_DPLL ");
	else if (params->flags & HDLC_FLAG_TXC_BRG)
		printf("TXC_BRG ");
	else if (params->flags & HDLC_FLAG_TXC_RXCPIN)
		printf("TXC_RXCPIN ");
	else 
		printf("TXC_TXCPIN ");	

	if (params->flags & HDLC_FLAG_DPLL_DIV8)
		printf("DPLL_DIV8 ");
	if (params->flags & HDLC_FLAG_DPLL_DIV16)
		printf("DPLL_DIV16 ");
	printf("\n");
	
	switch(params->encoding) {
	case HDLC_ENCODING_NRZ:			str="NRZ"; break;
	case HDLC_ENCODING_NRZB:		str="NRZB"; break;
	case HDLC_ENCODING_NRZI_MARK:		str="NRZI MARK"; break;
	case HDLC_ENCODING_NRZI_SPACE:		str="NRZI SPACE"; break;
	case HDLC_ENCODING_BIPHASE_MARK:	str="BIPHASE MARK"; break;
	case HDLC_ENCODING_BIPHASE_SPACE:	str="BIPHASE SPACE"; break;
	case HDLC_ENCODING_BIPHASE_LEVEL:	str="BIPHASE LEVEL"; break;
	case HDLC_ENCODING_DIFF_BIPHASE_LEVEL:	str="BIPHASE LEVEL"; break;
	default:		str = unknown_str;
	}
	printf("encoding=%s\n",str);
	
	if ( !params->clock_speed )
		printf("aux clock output=disabled\n");
	else
		printf("aux clock output=%ld\n",params->clock_speed);
		
	printf("addr filter=%02X\n",params->addr_filter);
	
	switch(params->crc_type & HDLC_CRC_MASK ) {
	case HDLC_CRC_NONE:	str="none"; break;
	case HDLC_CRC_16_CCITT:	str="CRC-16 CCITT"; break;
	case HDLC_CRC_32_CCITT:	str="CRC-32 CCITT"; break;
	default:		str = unknown_str;
	}
	printf("crc_type=%s%s\n",str, 
			params->crc_type & HDLC_CRC_RETURN_EX ? " (extended)" : "");
	
	if (params->preamble == HDLC_PREAMBLE_PATTERN_NONE)
		printf("preamble=disabled\n");
	else {
		switch(params->preamble_length) {
		case HDLC_PREAMBLE_LENGTH_8BITS:	str2="8 bits"; break;
		case HDLC_PREAMBLE_LENGTH_16BITS:	str2="16 bits"; break;
		case HDLC_PREAMBLE_LENGTH_32BITS:	str2="32 bits"; break;
		case HDLC_PREAMBLE_LENGTH_64BITS:	str2="64 bits"; break;
		default:		str2 = unknown_str;
		}
	
		switch(params->preamble) {
		case HDLC_PREAMBLE_PATTERN_ZEROS:	str="all zeros"; break;
		case HDLC_PREAMBLE_PATTERN_FLAGS:	str="all flags"; break;
		case HDLC_PREAMBLE_PATTERN_10:		str="alternating 10"; break;
		case HDLC_PREAMBLE_PATTERN_01:		str="alternating 01"; break;
		case HDLC_PREAMBLE_PATTERN_ONES:	str="all ones"; break;
		default:		str = unknown_str;
		}
		printf("preamble=%s, %s\n",str,str2);
	}

	if (idlemode & HDLC_TXIDLE_CUSTOM_16) {
		sprintf(strbuf, "custom 16 bit=%04X (hex)", idlemode & 0xffff);
		str = strbuf;
	} else if (idlemode & HDLC_TXIDLE_CUSTOM_8) {
		sprintf(strbuf, "custom 8 bit=%02X (hex)", idlemode & 0xff);
		str = strbuf;
	} else {
		switch(idlemode){
		case HDLC_TXIDLE_FLAGS:		str="continuous flags";break;
		case HDLC_TXIDLE_ONES:		str="continuous ones";break;
		case HDLC_TXIDLE_ZEROS:		str="continuous zeros";break;
		case HDLC_TXIDLE_MARK:		str="continuous mark";break;
		case HDLC_TXIDLE_SPACE:		str="continuous space";break;
		case HDLC_TXIDLE_ALT_ZEROS_ONES:str="alternating 0/1";break;
		case HDLC_TXIDLE_ALT_MARK_SPACE:str="alternating mark/space";break;
		default:			str="unknown";
		}
	}
	
	printf("transmit idle mode=%s\n",str);
	
	switch(params->parity) {
	case ASYNC_PARITY_NONE:		str="N"; break;
	case ASYNC_PARITY_EVEN:		str="E"; break;
	case ASYNC_PARITY_ODD:		str="O"; break;
	case ASYNC_PARITY_SPACE:	str="S"; break;
	default:		str = unknown_str;
	}
	
	printf("async speed=%ld format=%s-%d-%d\n\n",params->data_rate,
		str, params->data_bits, params->stop_bits);
		
	fflush(0);
	
}	/* end of display_params() */

/* get_params()
 * 
 * 	get the current device parameters for the specified device
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
int get_params(char *devname, MGSL_PARAMS* params, int *idlemode)
{
	int fd,rc;
	static char funcname[] = "get_params()";
	
	/* open the specified device */
	fd = open(devname,O_NONBLOCK,0);
	if (fd < 0) {
		printf("%s(%d):%s open on device %s failed with err=%d %s\n",
			__FILE__,__LINE__,funcname,devname,errno, strerror(errno));
		return fd;
	}
	
	/* make ioctl call to get current parameters */
	rc = ioctl(fd,MGSL_IOCGPARAMS,params);
	if (rc < 0) {
		printf("%s(%d):%s ioctl(MGSL_IOCGPARAMS) on device %s"
			" failed with err=%d %s\n",
			__FILE__,__LINE__,funcname,devname,errno, strerror(errno));
		return rc;
	}
	
	/* make ioctl call to get current tx idle mode */
	rc = ioctl(fd,MGSL_IOCGTXIDLE,idlemode);
	if (rc < 0) {
		printf("%s(%d):%s ioctl(MGSL_IOCGPARAMS) on device %s"
			" failed with err=%d %s\n",
			__FILE__,__LINE__,funcname,devname,errno,strerror(errno));
		return rc;
	}
	
	if (programmable_if) {
		rc = ioctl(fd, MGSL_IOCGIF, &if_mode);
		if (rc < 0) {
			printf("%s(%d):%s ioctl(MGSL_IOCGIF) on device %s"
			       " failed with err=%d %s\n",
			       __FILE__,__LINE__,funcname,devname,errno,strerror(errno));
			return rc;
		}
	}

	close(fd);
	return 0;
	
}	/* end of get_params() */

/* apply_params()
 * 
 * 	apply a device parameters structure to the specified device
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
int apply_params(char *devname, MGSL_PARAMS* params, int idlemode)
{
	int fd,rc;
	static char funcname[] = "apply_params()";
	struct ifreq ifr;
	int flags;
	int sock;
	int devnum;
	int portnum;

	/* open the specified device */
	fd = open(devname,O_NONBLOCK,0);
	if (fd < 0) {
		printf("%s(%d):%s open on device %s failed with err=%d %s\n",
			__FILE__,__LINE__,funcname,devname,errno, strerror(errno));
		return fd;
	}
	
	/* make ioctl call to set current parameters */
	rc = ioctl(fd,MGSL_IOCSPARAMS,params);
	if (rc < 0) {
		printf("%s(%d):%s ioctl(MGSL_IOCSPARAMS) on device %s"
			" failed with err=%d %s\n",
			__FILE__,__LINE__,funcname,devname,errno, strerror(errno));
		return rc;
	}
	
	/* make ioctl call to get current parameters */
	rc = ioctl(fd,MGSL_IOCSTXIDLE,idlemode);
	if (rc < 0) {
		printf("%s(%d):%s ioctl(MGSL_IOCSTXIDLE) on device %s"
			" failed with err=%d %s\n",
			__FILE__,__LINE__,funcname,devname,errno, strerror(errno));
		return rc;
	}

	if (programmable_if) {
		/* select serial interface type (not supported by all adapters) */
		ioctl(fd,MGSL_IOCSIF,if_mode);
	}

	close(fd);

	if ((sock = socket(PF_INET,SOCK_DGRAM,0)) < 0) {
		perror("Cant open socket\n");
		return 0;
	}
	do {
		portnum = -1;
		if ( sscanf(devname,"/dev/ttySLM%dp%d",&devnum,&portnum) == 2 ) {
			sprintf(ifr.ifr_ifrn.ifrn_name, "mgslm%dp%d", devnum,portnum);
			break;
		}
		else if (sscanf(devname,"/dev/ttySL%d",&devnum) == 1 ) {
			sprintf(ifr.ifr_ifrn.ifrn_name, "mgsl%d", devnum);
			break;
		}
		else if (sscanf(devname,"/dev/ttySLS%d",&devnum) == 1 ) {
			sprintf(ifr.ifr_ifrn.ifrn_name, "mgsls%d", devnum);
			break;
		}
		else if (sscanf(devname,"/dev/ttySLP%d",&devnum) == 1 ) {
			sprintf(ifr.ifr_ifrn.ifrn_name, "mgslp%d", devnum);
			break;
		}
		else if (sscanf(devname,"/dev/ttySLG%d",&devnum) == 1 ) {
			sprintf(ifr.ifr_ifrn.ifrn_name, "mgslg%d", devnum);
			break;
		}
		else if (sscanf(devname,"/dev/ttyUSB%d",&devnum) == 1 ) {
			sprintf(ifr.ifr_ifrn.ifrn_name, "mgslu%d", devnum);
			break;
		}
		else {
			perror("Cant parse device number\n");
			return 0;
		}
	} while ( 0 );

	if (!quiet)
	  printf("Configuring network device %s\n", ifr.ifr_ifrn.ifrn_name);

	/* try old style IOCTL (kernel version < 2.2.16) */
	rc = ioctl(sock,cisco ? SPPPIOCCISCO : SPPPIOCPPP,&ifr);
	if (rc != 0) {
	  if (!quiet && syncppp_options_set) {
	    printf("SYNCPPP support is not present.\n"
		   "SYNCPPP support is *NOT* necessary when using the pppd program for PPP!\n"
		   "SYNCPPP is used for Cisco HDLC and as an alternative to pppd for PPP.\n"
		   "See README.Cisco file of SyncLink software distribution for details.\n");
	  }
	  return 0;
	}

	if (!quiet)
	  printf("SYNCPPP Mode      %s\n", cisco ? "Cisco HDLC" : "PPP");

	/* SYNCPPP support exists, try new style IOCTL call */
	ifr.ifr_data = (char *)&flags;
	rc = ioctl(sock,SPPPIOCGFLAGS,&ifr);
	if (rc == 0) {
		if (cisco)
			flags |= PP_CISCO;
		else
			flags &= ~PP_CISCO;
		if (keepalive)
			flags |= PP_KEEPALIVE;
		else
			flags &= ~PP_KEEPALIVE;
		rc = ioctl(sock,SPPPIOCSFLAGS,&ifr);
		if (rc == 0 && !quiet) {
			printf("SYNCPPP keepalive %s\n", keepalive ? "enabled" : "disabled");
		}
	} else {
	  if (!quiet && syncppp_options_set) {
	    printf("SYNCPPP set/get flags IOCTL call not supported.\n"
		   "The keepalive setting cannot be controlled.\n"
		   "set/get flags IOCTL call is part of kernel 2.2.16 and later\n");
	  }
	}

	return 0;
	
}	/* end of apply_params() */

/*
 * Handler functions for individual options
 */
 
int set_async(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->mode = MGSL_MODE_ASYNC;
	return 1;
}	/* end of set_async() */

int set_hdlc(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->mode = MGSL_MODE_HDLC;
	return 1;
}	/* end of set_hdlc() */

int set_raw(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->mode = MGSL_MODE_RAW;
	return 1;
}	/* end of set_raw() */

int set_bisync(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->mode = MGSL_MODE_BISYNC;
	return 1;
}	/* end of set_bisync() */

int set_monosync(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->mode = MGSL_MODE_MONOSYNC;
	return 1;
}	/* end of set_monosync() */

int set_loopback(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->loopback = 1;
	return 1;
}	/* end of set_loopback() */

int set_noloopback(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->loopback = 0;
	return 1;
}	/* end of set_noloopback() */

int set_ua7(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->flags &= ~(HDLC_FLAG_UNDERRUN_ABORT15+
				HDLC_FLAG_UNDERRUN_FLAG +
				HDLC_FLAG_UNDERRUN_CRC);
	return 1;
}	/* end of set_ua7() */

int set_ua15(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->flags &= ~(HDLC_FLAG_UNDERRUN_ABORT15+
				HDLC_FLAG_UNDERRUN_FLAG +
				HDLC_FLAG_UNDERRUN_CRC);
	params->flags |= HDLC_FLAG_UNDERRUN_ABORT15;
	return 1;
}	/* end of set_ua15() */

int set_uflag(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->flags &= ~(HDLC_FLAG_UNDERRUN_ABORT15+
				HDLC_FLAG_UNDERRUN_FLAG +
				HDLC_FLAG_UNDERRUN_CRC);
	params->flags |= HDLC_FLAG_UNDERRUN_FLAG;
	return 1;
}	/* end of set_uflag() */

int set_ucrc(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->flags &= ~(HDLC_FLAG_UNDERRUN_ABORT15+
				HDLC_FLAG_UNDERRUN_FLAG +
				HDLC_FLAG_UNDERRUN_CRC);
	params->flags |= HDLC_FLAG_UNDERRUN_CRC;
	return 1;
}	/* end of set_ucrc() */

int set_sharezero(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->flags |= HDLC_FLAG_SHARE_ZERO;
	return 1;
}	/* end of set_sharezero() */

int set_nosharezero(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->flags &= ~HDLC_FLAG_SHARE_ZERO;
	return 1;
}	/* end of set_nosharezero() */

int set_autocts(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->flags += HDLC_FLAG_AUTO_CTS;
	return 1;
}	/* end of set_autocts() */

int set_noautocts(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->flags &= ~HDLC_FLAG_AUTO_CTS;
	return 1;
}	/* end of set_noautocts() */

int set_autodcd(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->flags += HDLC_FLAG_AUTO_DCD;
	return 1;
}	/* end of set_autodcd() */

int set_noautodcd(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->flags &= ~HDLC_FLAG_AUTO_DCD;
	return 1;
}	/* end of set_noautodcd() */

int set_autorts(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->flags += HDLC_FLAG_AUTO_RTS;
	return 1;
}	/* end of set_autorts() */

int set_noautorts(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->flags &= ~HDLC_FLAG_AUTO_RTS;
	return 1;
}	/* end of set_noautorts() */

int set_rtsen(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	if_mode |= MGSL_INTERFACE_RTS_EN;
	return 1;
}	/* end of set_rtsen() */

int set_nortsen(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	if_mode &= ~MGSL_INTERFACE_RTS_EN;
	return 1;
}	/* end of set_nortsen() */

int set_rxc(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->flags &= ~(HDLC_FLAG_RXC_DPLL+HDLC_FLAG_RXC_BRG+HDLC_FLAG_RXC_TXCPIN);
	return 1;
}	/* end of set_rxc() */

int set_rxdpll(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->flags &= ~(HDLC_FLAG_RXC_DPLL+HDLC_FLAG_RXC_BRG+HDLC_FLAG_RXC_TXCPIN);
	params->flags |= HDLC_FLAG_RXC_DPLL;
	return 1;
}	/* end of set_rxdpll() */

int set_rxbrg(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->flags &= ~(HDLC_FLAG_RXC_DPLL+HDLC_FLAG_RXC_BRG+HDLC_FLAG_RXC_TXCPIN);
	params->flags |= HDLC_FLAG_RXC_BRG;
	return 1;
}	/* end of set_rxbrg() */

int set_rxtxc(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->flags &= ~(HDLC_FLAG_RXC_DPLL+HDLC_FLAG_RXC_BRG+HDLC_FLAG_RXC_TXCPIN);
	params->flags |= HDLC_FLAG_RXC_TXCPIN;
	return 1;
}	/* end of set_rxbrg() */

int set_txc(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->flags &= ~(HDLC_FLAG_TXC_DPLL+HDLC_FLAG_TXC_BRG+HDLC_FLAG_TXC_RXCPIN);
	return 1;
}	/* end of set_txc() */

int set_txdpll(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->flags &= ~(HDLC_FLAG_TXC_DPLL+HDLC_FLAG_TXC_BRG+HDLC_FLAG_TXC_RXCPIN);
	params->flags |= HDLC_FLAG_TXC_DPLL;
	return 1;
}	/* end of set_txdpll() */

int set_txbrg(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->flags &= ~(HDLC_FLAG_TXC_DPLL+HDLC_FLAG_TXC_BRG+HDLC_FLAG_TXC_RXCPIN);
	params->flags |= HDLC_FLAG_TXC_BRG;
	return 1;
}	/* end of set_txbrg() */

int set_txrxc(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->flags &= ~(HDLC_FLAG_TXC_DPLL+HDLC_FLAG_TXC_BRG+HDLC_FLAG_TXC_RXCPIN);
	params->flags |= HDLC_FLAG_TXC_RXCPIN;
	return 1;
}	/* end of set_txbrg() */

int set_dplldiv8(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->flags &= ~(HDLC_FLAG_DPLL_DIV8+HDLC_FLAG_DPLL_DIV16);
	params->flags |= HDLC_FLAG_DPLL_DIV8;
	return 1;
}	/* end of set_dplldiv8() */

int set_dplldiv16(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->flags &= ~(HDLC_FLAG_DPLL_DIV8+HDLC_FLAG_DPLL_DIV16);
	params->flags |= HDLC_FLAG_DPLL_DIV16;
	return 1;
}	/* end of set_dplldiv16() */

int set_dplldiv32(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->flags &= ~(HDLC_FLAG_DPLL_DIV8+HDLC_FLAG_DPLL_DIV16);
	return 1;
}	/* end of set_dplldiv32() */

int set_nrz(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->encoding = HDLC_ENCODING_NRZ;
	return 1;
}	/* end of set_nrzi() */

int set_nrzi(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->encoding = HDLC_ENCODING_NRZI_SPACE;
	return 1;
}	/* end of set_nrzi() */

int set_nrzb(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->encoding = HDLC_ENCODING_NRZB;
	return 1;
}	/* end of set_nrzb() */

int set_nrzimark(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->encoding = HDLC_ENCODING_NRZI_MARK;
	return 1;
}	/* end of set_nrzimark() */

int set_bimark(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->encoding = HDLC_ENCODING_BIPHASE_MARK;
	return 1;
}	/* end of set_bimark() */

int set_bispace(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->encoding = HDLC_ENCODING_BIPHASE_SPACE;
	return 1;
}	/* end of set_bispace() */

int set_bilevel(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->encoding = HDLC_ENCODING_BIPHASE_LEVEL;
	return 1;
}	/* end of set_bilevel() */

int set_bidiff(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->encoding = HDLC_ENCODING_DIFF_BIPHASE_LEVEL;
	return 1;
}	/* end of set_bidiff() */

int set_clock(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	int new_clock;
	
	if (argc<2 || !sscanf(argv[1],"%i",&new_clock)) {
		printf("\nclock option requires decimal bit rate\n");
		return -EINVAL;
	}
	
	params->clock_speed = new_clock;
	return 2;
}	/* end of set_clock() */

int set_addr(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	unsigned short new_addr;
	
	if (argc<2 || !sscanf(argv[1],"%hx",&new_addr)) {
		printf("\naddr option requires 8-bit hex address\n");
		return -EINVAL;
	}
	
	params->addr_filter = (unsigned char)new_addr;
	return 2;
}	/* end of set_addr() */

int set_crcnone(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->crc_type = HDLC_CRC_NONE;
	return 1;
}	/* end of set_crcnone() */

int set_crc16(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->crc_type = HDLC_CRC_16_CCITT;
	return 1;
}	/* end of set_crc16() */

int set_crc16ex(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->crc_type = HDLC_CRC_16_CCITT | HDLC_CRC_RETURN_EX;
	return 1;
}	/* end of set_crc16ex() */


int set_crc32(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->crc_type = HDLC_CRC_32_CCITT;
	return 1;
}	/* end of set_crc32() */

int set_crc32ex(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->crc_type = HDLC_CRC_32_CCITT | HDLC_CRC_RETURN_EX;
	return 1;
}	/* end of set_crc32ex() */

int set_prenone(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->preamble = HDLC_PREAMBLE_PATTERN_NONE;
	return 1;
}	/* end of set_prenone() */

int set_preone(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->preamble = HDLC_PREAMBLE_PATTERN_ONES;
	return 1;
}	/* end of set_preone() */

int set_prezero(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->preamble = HDLC_PREAMBLE_PATTERN_ZEROS;
	return 1;
}	/* end of set_prezero() */

int set_pre10(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->preamble = HDLC_PREAMBLE_PATTERN_10;
	return 1;
}	/* end of set_pre10() */

int set_pre01(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->preamble = HDLC_PREAMBLE_PATTERN_01;
	return 1;
}	/* end of set_pre01() */

int set_preflag(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->preamble = HDLC_PREAMBLE_PATTERN_FLAGS;
	return 1;
}	/* end of set_preflag() */

int set_pre8(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->preamble_length = HDLC_PREAMBLE_LENGTH_8BITS;
	return 1;
}	/* end of set_pre8() */

int set_pre16(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->preamble_length = HDLC_PREAMBLE_LENGTH_16BITS;
	return 1;
}	/* end of set_pre16() */

int set_pre32(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->preamble_length = HDLC_PREAMBLE_LENGTH_32BITS;
	return 1;
}	/* end of set_pre32() */

int set_pre64(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->preamble_length = HDLC_PREAMBLE_LENGTH_64BITS;
	return 1;
}	/* end of set_pre64() */

int set_rate(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	int new_rate;
	
	if (argc<2 || !sscanf(argv[1],"%i",&new_rate)) {
		printf("\nrate option requires decimal bit rate\n");
		return -EINVAL;
	}
	
	params->data_rate = new_rate;
	return 2;
}	/* end of set_rate() */

int set_databits(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	int new_databits;
	
	if (argc<2 || !sscanf(argv[1],"%i",&new_databits) ||
	    new_databits < 5 || new_databits >8) {
		printf("\ndatabits option requires argument (5-8)\n");
		return -EINVAL;
	}
	
	params->data_bits = new_databits;
	return 2;
}	/* end of set_databits() */

int set_paritynone(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->parity = ASYNC_PARITY_NONE;
	return 1;
}	/* end of set_paritynone() */

int set_parityodd(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->parity = ASYNC_PARITY_ODD;
	return 1;
}	/* end of set_parityodd() */

int set_parityeven(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->parity = ASYNC_PARITY_EVEN;
	return 1;
}	/* end of set_parityeven() */

int set_stopbits(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	int new_stopbits;
	
	if (argc<2 || !sscanf(argv[1],"%i",&new_stopbits) ||
	    new_stopbits < 1 || new_stopbits >2) {
		printf("\nstopbits option requires argument (0-1)\n");
		return -EINVAL;
	}
	
	params->stop_bits = new_stopbits;
	return 2;
}	/* end of set_stopbits() */

int set_hdlcloop(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->flags += HDLC_FLAG_HDLC_LOOPMODE;
	return 1;
}	/* end of set_hdlcloop() */

int set_nohdlcloop(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	params->flags &= ~HDLC_FLAG_HDLC_LOOPMODE;
	return 1;
}	/* end of set_nohdlcloop() */

int display_stats(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	int fd,rc;
	static char funcname[] = "display_stats()";
	struct mgsl_icount icount;
	
	/* open the specified device */
	fd = open(devname,O_NONBLOCK,0);
	if (fd < 0) {
		printf("%s(%d):%s open on device %s failed with err=%d %s\n",
			__FILE__,__LINE__,funcname,devname,errno, strerror(errno));
		return 1;
	}
	
	/* make ioctl call to get current parameters */
	rc = ioctl(fd,MGSL_IOCGSTATS,&icount);
	if (rc < 0) {
		printf("%s(%d):%s ioctl(MGSL_IOCGSTATS) on device %s"
			" failed with err=%d %s\n",
			__FILE__,__LINE__,funcname,devname,errno, strerror(errno));
		return 1;
	}
	
	/* close device */
	close(fd);
	
	printf( "\n*** %s Statistics ***\n"
		"irqs: DSR:%d CTS:%d DCD:%d RI:%d TX:%d RX:%d\n"
		"async stats:\n"
		"    framing=%d parity=%d overrun=%d breaks=%d bufoverrun=%d\n"
		"hdlc/raw frame stats:\n"
		"    txok=%d txunder=%d txabort=%d txtimeout=%d\n"
		"    rxok=%d rxshort=%d rxlong=%d rxabort=%d rxoverrun=%d rxcrc=%d\n\n",
		devname,icount.dsr,icount.cts, icount.dcd, icount.rng, icount.tx,
		icount.rx,icount.frame,icount.parity,icount.overrun,
		icount.brk,icount.buf_overrun,icount.txok,icount.txunder,
		icount.txabort,icount.txtimeout,icount.rxok,icount.rxshort,icount.rxlong,
		icount.rxabort,icount.rxover,icount.rxcrc);
	
	return 1;
}	/* end of display_stats() */

int set_idle0(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	*idlemode=HDLC_TXIDLE_ZEROS;
	return 1;
}	/* end of set_idle0() */

int set_idle1(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	*idlemode=HDLC_TXIDLE_ONES;
	return 1;
}	/* end of set_idle1() */

int set_idle01(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	*idlemode=HDLC_TXIDLE_ALT_ZEROS_ONES;
	return 1;
}	/* end of set_idle01() */

int set_idleflag(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	*idlemode=HDLC_TXIDLE_FLAGS;
	return 1;
}	/* end of set_idleflags() */

int set_idlemark(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	*idlemode=HDLC_TXIDLE_MARK;
	return 1;
}	/* end of set_idlemark() */

int set_idlespace(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	*idlemode=HDLC_TXIDLE_SPACE;
	return 1;
}	/* end of set_idlespace() */

int set_idlemarkspace(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	*idlemode=HDLC_TXIDLE_ALT_MARK_SPACE;
	return 1;
}	/* end of set_idlemarkspace() */

int set_idle_custom(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	int new_idle;

	if (argc < 2 || !sscanf(argv[1], "%x", &new_idle)) {
		printf("\nidle option requires 8 or 16 bit hex argument\n");
		return -EINVAL;
	}

	if (new_idle > 255)
		*idlemode = HDLC_TXIDLE_CUSTOM_16 | (new_idle & 0xffff);
	else
		*idlemode = HDLC_TXIDLE_CUSTOM_8 | (new_idle & 0xff);

	return 2;
}	/* end of set_idle_custom() */

int set_quiet(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	quiet=1;
	return 1;
}	/* end of set_quiet() */

int clearmodcount(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	int fd,rc;
	static char funcname[] = "clearmodcount()";
	
	/* open the specified device */
	fd = open(devname,O_NONBLOCK,0);
	if (fd < 0) {
		printf("%s(%d):%s open on device %s failed with err=%d %s\n",
			__FILE__,__LINE__,funcname,devname,errno, strerror(errno));
		return 1;
	}
	
	/* make ioctl call to get current parameters */
	rc = ioctl(fd,MGSL_IOCCLRMODCOUNT,0);
	if (rc < 0) {
		printf("%s(%d):%s ioctl(MGSL_IOCCLRMODCOUNT) on device %s"
			" failed with err=%d %s\n",
			__FILE__,__LINE__,funcname,devname,errno, strerror(errno));
		return 1;
	}
	
	/* close device */
	close(fd);
	
	return 1;
}	/* end of clearmodcount() */

int set_cisco(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	cisco = 1;
	syncppp_options_set = 1;
	return 1;
}
int clear_cisco(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	cisco = 0;
	syncppp_options_set = 1;
	return 1;
}
int set_keepalive(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	keepalive = 1;
	syncppp_options_set = 1;
	return 1;
}
int clear_keepalive(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	keepalive = 0;
	syncppp_options_set = 1;
	return 1;
}

int set_ifdisable(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	if_mode=(if_mode & ~MGSL_INTERFACE_MASK) | MGSL_INTERFACE_DISABLE;
	return 1;
}

int set_rs232(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	if_mode=(if_mode & ~MGSL_INTERFACE_MASK) | MGSL_INTERFACE_RS232;
	return 1;
}

int set_v35(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	if_mode=(if_mode & ~MGSL_INTERFACE_MASK) | MGSL_INTERFACE_V35;
	return 1;
}

int set_rs422(int argc, char* argv[], char* devname, MGSL_PARAMS* params, int *idlemode)
{
	if_mode=(if_mode & ~MGSL_INTERFACE_MASK) | MGSL_INTERFACE_RS422;
	return 1;
}

