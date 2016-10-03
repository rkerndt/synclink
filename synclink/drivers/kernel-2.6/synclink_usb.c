/*
 * SyncLink USB driver
 *
 * SyncLink USB is a USB to sync/async serial converter device
 * made by Microgate Systems, Ltd (www.microgate.com)
 *
 * It uses the FTDI 2232H USB IC, Microgate FPGA based serial controller
 * and Sipex 3508 multiprotocol transceiver. An IDT ICS307-3 frequency
 * synthesizer supports arbitrary clock generation. GPIO pins from the FPGA
 * are available for custom applications.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/usb.h>
#include <linux/serial.h>
#include <linux/usb/serial.h>
#include <linux/hdlc.h>

#include "linux/synclink.h"

#define VERSION(ver,rel,seq) (((ver)<<16) | ((rel)<<8) | (seq))

#if LINUX_VERSION_CODE < VERSION(2,6,33)
#include <linux/autoconf.h>
#else
#include <generated/autoconf.h>
#endif

#if defined(CONFIG_HDLC) || defined(CONFIG_HDLC_MODULE)
#define SYNCLINK_GENERIC_HDLC 1
#else
#define SYNCLINK_GENERIC_HDLC 0
#endif

#if LINUX_VERSION_CODE < VERSION(3,7,0)
#define tty_cflags(tty) ((tty)->termios->c_cflag)
#else
#define tty_cflags(tty) ((tty)->termios.c_cflag)
#endif

#if LINUX_VERSION_CODE < VERSION(3,9,0)
#define slusb_insert_flip_char(a,b,c) tty_insert_flip_char((a),(b),(c))
#define slusb_flip_buffer_push(a) tty_flip_buffer_push((a))
#else
#define slusb_insert_flip_char(a,b,c) tty_insert_flip_char((a)->port,(b),(c))
#define slusb_flip_buffer_push(a) tty_flip_buffer_push((a)->port)
#endif

#if LINUX_VERSION_CODE >= VERSION(2,6,35)
#define usb_buffer_alloc(a,b,c,d) usb_alloc_coherent(a,b,c,d)
#define usb_buffer_free(a,b,c,d)  usb_free_coherent(a,b,c,d)
#endif

/*
 * module identification
 */
static const char slusb_driver_name[] = "SyncLink USB";
MODULE_AUTHOR("Paul Fulghum <paulkf@microgate.com>");
MODULE_DESCRIPTION("SyncLink USB Driver");
MODULE_LICENSE("GPL");

/*
 * module options
 */
static int debug_level = 0;
static int maxframe = 4096;
module_param(debug_level, int, 0);
module_param(maxframe, int, 0);
MODULE_PARM_DESC(debug_level, "Debug output: 0=off 1-5=increasing detail");
MODULE_PARM_DESC(maxframe, "Maximum frame size (4096 to 65535)");

#define POLL_BUFFER_SIZE 4096

struct slusb_tbuf {
	unsigned int count;
	struct slusb_tbuf *next;
	unsigned char buf[1];
};

/* serial input signal events */
struct _input_signal_events {
	int ri_up;
	int ri_down;
	int dsr_up;
	int dsr_down;
	int dcd_up;
	int dcd_down;
	int cts_up;
	int cts_down;
};

/*
 * conditional wait facility
 */
struct cond_wait {
	struct cond_wait *next;
	wait_queue_head_t q;
	wait_queue_t wait;
	unsigned int data;
};
static void init_cond_wait(struct cond_wait *w, unsigned int data);
static void add_cond_wait(struct cond_wait **head, struct cond_wait *w);
static void remove_cond_wait(struct cond_wait **head, struct cond_wait *w);
static void flush_cond_wait(struct cond_wait **head);

/*
 * private per port data
 */
struct slusb_info {

	struct kref kref;
	struct usb_serial_port *port;
	int flags; /* some ASYNC_xxxx flags are supported */

	/* device members */

	spinlock_t lock;    /* spinlock for synchronizing with poll routine */
	spinlock_t netlock; /* generic HDLC synchronization */
	bool net_open;
	bool tty_open;
	int dosyncppp;
#if SYNCLINK_GENERIC_HDLC
	struct net_device *netdev;
#endif

	unsigned int port_number;

	char x_char;

	atomic_t urb_count;
	wait_queue_head_t urb_wait;

	struct work_struct poll_work;
	unsigned int pending_work;
	bool work_running;

	bool gpio_present;

	bool test_mode; /* true during startup test */

	int timeout;

	unsigned short port_index;

	/* saved copy of last register write/read values */
	unsigned char reg_last_write[256];
	unsigned char reg_last_read[256];
	bool reg_updated[256];
	wait_queue_head_t reg_wait;

	/*
	 * circular USB in channel buffer
	 * - must hold max size frame
	 * - must be size of poll buffer + max size packet
	 */
	unsigned char inbuf[20000];
	unsigned int  inbuf_count;
	unsigned int  inbuf_get_index;
	unsigned int  inbuf_put_index;

	/* circular USB receive data buffer */
	unsigned char rbuf[32000];
	unsigned int  rbuf_get_index;
	unsigned int  rbuf_put_index;

	/* USB transmit buffers */
	unsigned int tbuf_count;
	unsigned int tbuf_index;
	struct slusb_tbuf *tbuf_head;
	struct slusb_tbuf *tbuf_tail;

	/* rx bulk synchronous transfer buffer */
	unsigned char rx_bulk_buf[4096];

	/* temporary copy buffer */
	unsigned char temp_buf[256];

	/* out channel buffer */
	unsigned char outbuf[4096];
	unsigned int outbuf_count;

	unsigned char *tmp_rbuf;
	unsigned int tmp_rbuf_count;

	/* bulk receive polling state */
	bool poll_active;
	bool poll_stop;
	wait_queue_head_t poll_wait;
	struct urb *poll_urb;
	unsigned char *poll_buf;

	/* current bulk pipes */
	unsigned int out_pipe;
	unsigned int in_pipe;
	unsigned short in_max_size;
	unsigned short out_max_size;

	/* serial channel bulk pipes */
	unsigned int serial_out_pipe;
	unsigned int serial_in_pipe;
	unsigned short serial_in_max_size;
	unsigned short serial_out_max_size;

	/* service channel bulk pipes */
	unsigned int service_out_pipe;
	unsigned int service_in_pipe;
	unsigned short service_in_max_size;
	unsigned short service_out_max_size;

	unsigned long bus_speed;

	bool is_high_speed;

	unsigned int gpio_state;
	struct cond_wait *gpio_wait_q;

	MGSL_PARAMS params;       /* communications parameters */
	u32 idle_mode;
	u32 max_frame_size;       /* as set by device config */

	unsigned int rbuf_fill_level;
	unsigned int rx_pio;
	unsigned char rx_pio_buf[256];
	unsigned int  rx_pio_count;
	unsigned int if_mode;
	unsigned int base_clock;
	unsigned int xsync;
	unsigned int xctrl;
	unsigned int jcr_value;

	/* device status */

	bool rx_enabled;
	bool rx_restart;

	bool tx_enabled;
	bool tx_active;

	unsigned char signals;    /* serial signal states */
	int init_error;  /* initialization error */

	unsigned char *tx_buf;
	int tx_count;

	char flag_buf[MAX_ASYNC_BUFFER_SIZE];
	char char_buf[MAX_ASYNC_BUFFER_SIZE];
	bool drop_rts_on_tx_done;

	unsigned int read_status_mask;
	unsigned int ignore_status_mask;

	struct _input_signal_events input_signal_events;
	struct mgsl_icount icount;

	wait_queue_head_t status_event_wait_q;
	wait_queue_head_t event_wait_q;
	struct timer_list tx_timer;

	/* service port specific */
	struct device *dev;
	struct usb_interface *usb_if;

	/* debug buffers */
	unsigned char stat1[2];
	unsigned char stat2[2];
	int count1;
	int count2;
	unsigned char buf1[512];
	unsigned char buf2[512];
};

/*
 * bulk URB context
 * info   associate device information
 * func   URB completion handler
 * buf    transfer buffer
 */
struct slusb_bulk_context {
	struct slusb_info *info;
	void (*func)(struct slusb_info *);
	unsigned char buf[4096];
};

static MGSL_PARAMS slusb_default_params = {
	.mode            = MGSL_MODE_HDLC,
	.loopback        = 0,
	.flags           = HDLC_FLAG_UNDERRUN_ABORT15,
	.encoding        = HDLC_ENCODING_NRZI_SPACE,
	.clock_speed     = 0,
	.addr_filter     = 0xff,
	.crc_type        = HDLC_CRC_16_CCITT,
	.preamble_length = HDLC_PREAMBLE_LENGTH_8BITS,
	.preamble        = HDLC_PREAMBLE_PATTERN_NONE,
	.data_rate       = 9600,
	.data_bits       = 8,
	.stop_bits       = 1,
	.parity          = ASYNC_PARITY_NONE
};

static bool slusb_test_register(struct slusb_info *info);
static bool slusb_test_loopback(struct slusb_info *info);

#define MICROGATE_USB_VID 0x2618
#define SYNCLINK_USB_PID  0x00B0

/* port indices used for control messages */
#define SERIAL_PORT_INDEX  1
#define SERVICE_PORT_INDEX 2


/* serial interface registration info */

static struct usb_device_id slusb_serial_id_table [] = {
	{USB_DEVICE(MICROGATE_USB_VID, SYNCLINK_USB_PID)},
	{} /* terminating entry */
};
MODULE_DEVICE_TABLE(usb, slusb_serial_id_table);

#if LINUX_VERSION_CODE < VERSION(3,5,0)
static struct usb_driver slusb_serial_usb_driver = {
	.name =		"synclink_usb_serial",
	.probe =	usb_serial_probe,
	.disconnect =	usb_serial_disconnect,
	.suspend =	usb_serial_suspend,
	.resume =	usb_serial_resume,
	.id_table =	slusb_serial_id_table,
	.no_dynamic_id = 1,
};
#endif

/* service interface registration info */

static struct usb_device_id slusb_service_id_table [] = {
	{USB_DEVICE(MICROGATE_USB_VID, SYNCLINK_USB_PID)},
	{} /* terminating entry */
};
MODULE_DEVICE_TABLE(usb, slusb_service_id_table);

/* USB core layer functions for service port */
static int  slusb_service_probe(struct usb_interface *usb_if,
			const struct usb_device_id *id);
static void slusb_service_disconnect(struct usb_interface *usb_if);

static struct usb_driver slusb_service_usb_driver = {
	.name =		"synclink_usb_service",
	.probe =	slusb_service_probe,
	.disconnect =	slusb_service_disconnect,
	.id_table =	slusb_service_id_table,
	.no_dynamic_id = 1,
};

/* Constants for read urb and write urb */
#define BUFSZ 512

#define SLUSB_MAX_FILL_LEVEL 256
#define SLUSB_MAX_TBUF_COUNT 10

/* usb-serial layer functions */
static int  slusb_serial_probe(struct usb_serial *serial,
			      const struct usb_device_id *id);
static void slusb_serial_disconnect(struct usb_serial *serial);
static int  slusb_port_probe(struct usb_serial_port *port);
static int  slusb_port_remove(struct usb_serial_port *port);
static int  slusb_suspend(struct usb_serial *serial, pm_message_t message);
static int  slusb_resume(struct usb_serial *serial);

/* tty functions */
static int  slusb_open(struct tty_struct *tty, struct usb_serial_port *port);
static void slusb_close(struct usb_serial_port *port);
static void slusb_dtr_rts(struct usb_serial_port *port, int on);
static int  slusb_carrier_raised(struct usb_serial_port *port);
static int  slusb_write(struct tty_struct *tty, struct usb_serial_port *port,
			const unsigned char *buf, int count);
static int  slusb_write_room(struct tty_struct *tty);
static int  slusb_chars_in_buffer(struct tty_struct *tty);
static void slusb_set_termios(struct tty_struct *tty,
			      struct usb_serial_port *port,
			      struct ktermios *old);
#if LINUX_VERSION_CODE < VERSION(2,6,39)
static int  slusb_tiocmget(struct tty_struct *tty, struct file *file);
static int  slusb_tiocmset(struct tty_struct *tty, struct file *file,
			   unsigned int set, unsigned int clear);
static int  slusb_ioctl(struct tty_struct *tty, struct file *file,
			unsigned int cmd, unsigned long arg);
#else
static int  slusb_tiocmget(struct tty_struct *tty);
static int  slusb_tiocmset(struct tty_struct *tty,
			   unsigned int set, unsigned int clear);
static int  slusb_ioctl(struct tty_struct *tty,
			unsigned int cmd, unsigned long arg);
#endif

static void slusb_break_ctl(struct tty_struct *tty, int break_state);
static void slusb_throttle(struct tty_struct *tty);
static void slusb_unthrottle(struct tty_struct *tty);

#if SYNCLINK_GENERIC_HDLC
/* generic HDLC functions */
static int  hdlcdev_init(struct slusb_info *info);
static void hdlcdev_exit(struct slusb_info *info);
static int  hdlcdev_attach(struct net_device *dev, unsigned short encoding,
			   unsigned short parity);
static void hdlcdev_rx(struct slusb_info *info, char *buf, int size);
static void hdlcdev_tx_done(struct slusb_info *info);

#if LINUX_VERSION_CODE > VERSION(2,6,26)
#define hdlc_stats(dev) (&(dev)->stats)
#endif
#define dev_to_port(D) (dev_to_hdlc(D)->priv)
#endif

/* usb-serial layer registration data */

static struct usb_serial_driver slusb_serial_driver = {
	.driver = {.owner = THIS_MODULE, .name =  "slusb",},
	.description =		slusb_driver_name,
#if LINUX_VERSION_CODE < VERSION(3,5,0)
	.usb_driver =		&slusb_serial_usb_driver,
#endif
	.id_table =		slusb_serial_id_table,
	.num_ports =		1,
	.probe =		slusb_serial_probe,
	.disconnect =           slusb_serial_disconnect,
	.port_probe =		slusb_port_probe,
	.port_remove =		slusb_port_remove,
	.suspend =              slusb_suspend,
	.resume =               slusb_resume,
	.open =			slusb_open,
	.close =		slusb_close,
	.carrier_raised =       slusb_carrier_raised,
	.dtr_rts =		slusb_dtr_rts,
	.throttle =		slusb_throttle,
	.unthrottle =		slusb_unthrottle,
	.write =		slusb_write,
	.write_room =		slusb_write_room,
	.chars_in_buffer =	slusb_chars_in_buffer,
	.tiocmget =             slusb_tiocmget,
	.tiocmset =             slusb_tiocmset,
	.ioctl =		slusb_ioctl,
	.set_termios =		slusb_set_termios,
	.break_ctl =		slusb_break_ctl,
};

/* NULL terminated list of usb-serial driver structures for this driver */
static struct usb_serial_driver * const slusb_serial_driver_table[] = {
	&slusb_serial_driver, NULL
};

/* USB to serial controller packet operation codes */

#define OP_MASK          0xe0
#define OP_WRITE_REG     0x00
#define OP_READ_REG      0x20
#define OP_SERIAL_STATUS 0x40
#define OP_GPIO_STATUS   0x60
#define OP_DATA          0x80
#define OP_SIZE_MASK     0x18
#define OP_SIZE_BYTE     0x00
#define OP_SIZE_SHORT    0x08
#define OP_SIZE_WORD     0x18
#define OP_EOM           0x08
#define OP_PORT_MASK     0x07

/* serial controller register macros */

#define GSR   0x00 /* global status */
#define JCR   0x04 /* JTAG control */
#define IODR  0x08 /* GPIO direction */
#define IOER  0x0c /* GPIO interrupt enable */
#define IOVR  0x10 /* GPIO value */
#define IOSR  0x14 /* GPIO interrupt status */
#define UCR   0x18 /* USB control register */
#define XSR   0x40 /* extended sync pattern */
#define XCR   0x44 /* extended control */
#define TDR   0x80 /* tx data */
#define RDR   0x80 /* rx data */
#define TCR   0x82 /* tx control */
#define TIR   0x84 /* tx idle */
#define TPR   0x85 /* tx preamble */
#define RCR   0x86 /* rx control */
#define VCR   0x88 /* V.24 control */
#define CCR   0x89 /* clock control */
#define BDR   0x8a /* baud divisor */
#define SCR   0x8c /* serial control */
#define SSR   0x8e /* serial status */
#define RFCR  0x90 /* rx FIFO control register */
#define TFCR  0x94 /* tx FIFO control register */

#define MASK_PARITY  BIT1
#define MASK_FRAMING BIT0
#define MASK_BREAK   BIT14
#define MASK_OVERRUN BIT4

#define RXIDLE      BIT14
#define RXBREAK     BIT14
#define IRQ_TXDATA  BIT13
#define IRQ_TXIDLE  BIT12
#define IRQ_TXUNDER BIT11 /* HDLC */
#define IRQ_RXDATA  BIT10
#define IRQ_RXIDLE  BIT9  /* HDLC */
#define IRQ_RXBREAK BIT9  /* async */
#define IRQ_RXOVER  BIT8
#define IRQ_DSR     BIT7
#define IRQ_CTS     BIT6
#define IRQ_DCD     BIT5
#define IRQ_RI      BIT4
#define IRQ_ALL     0x3ff0
#define IRQ_MASTER  BIT0

/* FT2232H vendor control URB codes */
#define FTDI_SET_LATENCY   9
#define FTDI_SET_MODE      11
#define FTDI_READ_EEPROM   0x90
#define FTDI_WRITE_EEPROM  0x91
#define FTDI_ERASE_EEPROM  0x92

/* FTDI channel modes */
#define FTDI_MODE_AFIFO 0
#define FTDI_MODE_SFIFO (0x40 << 8)
#define FTDI_MODE_MPSSE (0x02 << 8)

/* FTDI mode change delay in milliseconds */
#define FTDI_MODE_DELAY 50

static void slusb_usb_prom_cmd(struct slusb_info *info,
			       const unsigned char *buf,
			       unsigned int count, struct tty_struct *tty);

/*
 * serial controller register access functions
 *
 * register access via USB packets is expensive in latency and throughput
 *
 * - queue multiple writes and send as a single USB packet
 * - reads use USB request/response and require sleeping
 * - use last written values stored in port data instead of reads
 * - controller sends status indications so reads are seldom needed
 *
 * slusb_outx         write register
 * slusb_outx_queue   queue write register request
 * slusb_outx_saved   return last register write value
 * slusb_inx          read register (requires sleeping/no spinlocks)
 * slusb_inx_queue    queue read register request
 *
 * x = b (byte), w (word/u16), or l (long/u32)
 */
//static u8   slusb_inb(struct slusb_info *info, u8 addr);
//static void slusb_inb_queue(struct slusb_info *info, u8 addr);
static u8   slusb_outb_saved(struct slusb_info *info, u8 addr);
//static void slusb_outb(struct slusb_info *info, u8 addr, u8 data);
static void slusb_outb_queue(struct slusb_info *info, u8 addr, u8 data);

static u16  slusb_inw(struct slusb_info *info, u8 addr);
static void slusb_inw_queue(struct slusb_info *info, u8 addr);
static u16  slusb_outw_saved(struct slusb_info *info, u8 addr);
static void slusb_outw(struct slusb_info *info, u8 addr, u16 data);
static void slusb_outw_queue(struct slusb_info *info, u8 addr, u16 data);

static u32  slusb_inl(struct slusb_info *info, u8 addr);
static void slusb_inl_queue(struct slusb_info *info, u8 addr);
static u32  slusb_outl_saved(struct slusb_info *info, u8 addr);
static void slusb_outl(struct slusb_info *info, u8 addr, u32 data);
static void slusb_outl_queue(struct slusb_info *info, u8 addr, u32 data);


/*
 * usb_set_mux    select between fixed base clock (0) and freq synth (1)
 * usb_set_clk    set SPI CLK pin value
 * usb_set_select set SPI CS pin value
 * usb_set_data   set SPI DI pin value
 */
static void slusb_set_gpio_bit(struct slusb_info *info, int bit, int val);
#define slusb_set_mux(info, val)    slusb_set_gpio_bit((info), 23, (val))
#define slusb_set_clk(info, val)    slusb_set_gpio_bit((info), 22, (val))
#define slusb_set_select(info, val) slusb_set_gpio_bit((info), 21, (val))
#define slusb_set_data(info, val)   slusb_set_gpio_bit((info), 20, (val))

static unsigned int slusb_default_clock_word[5] = {0x08401401, 0xE1200000, 0x00000000, 0x0000A5FF, 0xA0000000};
static void slusb_set_clock_word(struct slusb_info *info, unsigned int *data);

void reset_fpga(struct slusb_info *info);

/* enable/disable indication packets (similar to interrupts) */
static void slusb_irq_on_queue(struct slusb_info *info, unsigned short mask);
static void slusb_irq_off_queue(struct slusb_info *info, unsigned short mask);
static void slusb_irq_off(struct slusb_info *info, unsigned short mask);

#define SLUSB_WORK_RX 1
#define SLUSB_WORK_TX 2
static void slusb_work(struct work_struct *work);

/* transmit functions */
static int  slusb_tx_enable(struct slusb_info *info, int enable);
static void slusb_tx_start(struct slusb_info *info);
static void slusb_tx_stop_queue(struct slusb_info *info);
static void slusb_tx_stop(struct slusb_info *info);
static void slusb_tx_flush(struct slusb_info *info);
static bool slusb_tx(struct slusb_info *info,
		     const char *buf, unsigned int size);

/* receive functions */
static int  slusb_rx_enable(struct slusb_info *info, int enable);
static void slusb_rx_start_queue(struct slusb_info *info);
static void slusb_rx_start(struct slusb_info *info);
static void slusb_rx_stop_queue(struct slusb_info *info);
static void slusb_rx_stop(struct slusb_info *info);
static void slusb_rx(struct slusb_info *info,
		     unsigned char *data, char *flags, int count);

static void slusb_set_signals_queue(struct slusb_info *info);
static void slusb_set_signals(struct slusb_info *info);

/* OUT channel buffer */
static bool slusb_outbuf_put(struct slusb_info *info, unsigned char *buf, unsigned int size);
static void slusb_outbuf_flush(struct slusb_info *info, void (*func)(struct slusb_info *));
static void slusb_outbuf_reset(struct slusb_info *info);

/* IN channel buffer */
static void         slusb_inbuf_reset(struct slusb_info *info);
static unsigned int slusb_inbuf_get(struct slusb_info *info, char *buf, unsigned int size);
static unsigned int slusb_inbuf_put(struct slusb_info *info, char *buf, unsigned int size);
static unsigned int slusb_inbuf_get_inc(struct slusb_info *info, unsigned int count);
static unsigned int slusb_inbuf_process(struct slusb_info *info);
static unsigned int slusb_process_read_reg(struct slusb_info *info);
static unsigned int slusb_process_rx_data(struct slusb_info *info);

/* receive data buffer */
static void         slusb_rbuf_reset(struct slusb_info *info);
static unsigned int slusb_rbuf_count(struct slusb_info *info);
static unsigned int slusb_rbuf_free_space(struct slusb_info *info);
static unsigned int slusb_rbuf_put(struct slusb_info *info, char *buf, unsigned int size);
static unsigned int slusb_rbuf_get(struct slusb_info *info, char *buf, unsigned int size);

static void slusb_process_ssr(struct slusb_info *info, unsigned short ssr);
static void slusb_process_serial_status(struct slusb_info *info, unsigned short status);

/* USB IN pipe polling routines */
static bool slusb_poll_start(struct slusb_info *info);
static bool slusb_poll_stop(struct slusb_info *info);
static int  slusb_poll(struct slusb_info *info);
static void slusb_poll_complete(struct urb *urb);

/* configuration functions */
static int  slusb_set_interface(struct slusb_info *info, int if_mode);
static void slusb_enable_loopback(struct slusb_info *info);
static void slusb_set_vcr(struct slusb_info *info);
static void slusb_tx_set_idle_queue(struct slusb_info *info);
static void slusb_tx_set_idle(struct slusb_info *info);
static void slusb_set_rate(struct slusb_info *info, u32 rate);
static void slusb_program_hw(struct slusb_info *info);
static void slusb_change_params(struct slusb_info *info);
static void slusb_async_mode(struct slusb_info *info);
static void slusb_sync_mode(struct slusb_info *info);
static void slusb_select_serial_pipes(struct slusb_info *info);
static void slusb_select_service_pipes(struct slusb_info *info);

static void slusb_shutdown(struct slusb_info *info);

/* configuration ioctl handlers */
static int slusb_get_params(struct slusb_info *info, MGSL_PARAMS __user *user_params);
static int slusb_set_params(struct slusb_info *info, MGSL_PARAMS __user *new_params);
static int slusb_get_txidle(struct slusb_info *info, int __user *idle_mode);
static int slusb_set_txidle(struct slusb_info *info, int idle_mode);

/* extended sync ioctl handlers */
static int slusb_get_xsync(struct slusb_info *info, int __user *if_mode);
static int slusb_set_xsync(struct slusb_info *info, int if_mode);
static int slusb_get_xctrl(struct slusb_info *info, int __user *if_mode);
static int slusb_set_xctrl(struct slusb_info *info, int if_mode);

static void slusb_tx_timeout(unsigned long context);

#define DBGINFO(fmt) if (debug_level >= DEBUG_LEVEL_INFO) printk fmt
#define DBGERR(fmt)  if (debug_level >= DEBUG_LEVEL_ERROR) printk fmt
#define DBGWORK(fmt) if (debug_level >= DEBUG_LEVEL_BH) printk fmt
#define DBGPOLL(fmt) if (debug_level >= DEBUG_LEVEL_ISR) printk fmt
//#define DBGDATA(info, buf, size, label) if (debug_level >= DEBUG_LEVEL_DATA) trace_block((info), (buf), (size), (label))
#define DBGDATA(info, buf, size, label)


#define slusb_dev_name(info) dev_name(&info->port->dev)

static void trace_block(struct slusb_info *info, const char *data, int count, const char *label)
{
	int i;
	int linecount;
	printk("%s %s data count=%d:\n", slusb_dev_name(info), label, count);
	while(count) {
		linecount = (count > 16) ? 16 : count;
		for(i=0; i < linecount; i++)
			printk("%02X ",(unsigned char)data[i]);
		for(;i<17;i++)
			printk("   ");
		for(i=0;i<linecount;i++) {
			if (data[i]>=040 && data[i]<=0176)
				printk("%c",data[i]);
			else
				printk(".");
		}
		printk("\n");
		data  += linecount;
		count -= linecount;
	}
}

/*
 * init service USB interface with serial USB interface information
 */
static void slusb_associate_if(struct usb_interface *serial_if,
			       struct usb_interface *service_if)
{
	struct usb_device *usb_dev;
	struct usb_serial *serial;
	struct usb_serial_port *port;
	struct usb_endpoint_descriptor *ep;
	struct slusb_info *info;
	unsigned int i;

	serial = usb_get_intfdata(serial_if);
	if (!serial)
		return;
	port = serial->port[0];
	if (!port)
		return;
	info = usb_get_serial_port_data(port);
	if (!info)
		return;
	usb_dev = interface_to_usbdev(serial_if);
	usb_set_intfdata(service_if, serial);

	for (i = 0; i < service_if->cur_altsetting->desc.bNumEndpoints; i++) {
		ep = &service_if->cur_altsetting->endpoint[i].desc;
		if (usb_endpoint_dir_in(ep)) {
			info->service_in_max_size =
				le16_to_cpu(ep->wMaxPacketSize);
			info->service_in_pipe =
				usb_rcvbulkpipe(usb_dev, ep->bEndpointAddress);
		} else {
			info->service_out_max_size =
				le16_to_cpu(ep->wMaxPacketSize);
			info->service_out_pipe =
				usb_sndbulkpipe(usb_dev, ep->bEndpointAddress);
		}
	}
}

/*
 * called by USB core for USB service interface when device inserted
 *
 * interface 0 = serial port
 * interface 1 = service port connected to FPGA PROM
 */
int slusb_service_probe(struct usb_interface *usb_if, const struct usb_device_id *usb_id)
{
	struct usb_device *usb_dev = interface_to_usbdev(usb_if);
	struct usb_interface *serial_usb_if;
	unsigned int i;

	if (usb_if->cur_altsetting->desc.bInterfaceNumber == 0) {
		/* serial port probe is handled by USB serial layer */
		return -ENODEV;
	}

	DBGINFO(("%s probe service port interface\n", __func__));

	/* search for serial usb interface */
	for (i = 0 ; i < USB_MAXINTERFACES ; i++) {
		serial_usb_if = usb_dev->config->interface[i];
		if (!serial_usb_if)
			continue;
		if (serial_usb_if->cur_altsetting->desc.bInterfaceNumber == 0) {
			slusb_associate_if(serial_usb_if, usb_if);
			break;
		}
	}
	return 0;
}

/*
 * called by USB core for service interface when
 * device is unplugged or driver is unloading
 */
void slusb_service_disconnect(struct usb_interface *usb_if)
{
	if (usb_if->cur_altsetting->desc.bInterfaceNumber != 0) {
		DBGINFO(("%s service port interface\n", __func__));
	} else {
		DBGINFO(("ERROR, %s called for serial port\n", __func__));
	}
	return;
}

/* called by usb-serial for each USB interface when device inserted */
static int slusb_serial_probe(struct usb_serial *serial,
			      const struct usb_device_id *id)
{
	usb_set_serial_data(serial, (void *)id->driver_info);
	return 0;
}

/* called by usb-serial for each USB interface when device unplugged */
static void slusb_serial_disconnect(struct usb_serial *serial)
{
}

/*
 * called by usb-serial for each USB interface when suspending device
 * return 0 to allow suspend or error to reject
 */
static int slusb_suspend(struct usb_serial *serial, pm_message_t message)
{
	struct usb_serial_port *port;
	struct slusb_info *info;

	port = serial->port[0];
	if (!port)
		goto accept_suspend;

	info = usb_get_serial_port_data(port);
	if (!info)
		goto accept_suspend;

	if (info->net_open || info->tty_open) {
		DBGINFO(("%s reject\n", __func__));
		return -EBUSY;
	}

accept_suspend:
	DBGINFO(("%s accept\n", __func__));
	return 0;
}

/*
 * called by usb-serial for each USB interface when resuming device
 * return 0 or error
 */
static int slusb_resume(struct usb_serial *serial)
{
	struct usb_serial_port *port;
	struct slusb_info *info;

	port = serial->port[0];
	if (!port)
		goto accept_resume;

	info = usb_get_serial_port_data(port);
	if (!info)
		goto accept_resume;

	/* TODO - reinit now or wait for open? is this interrupt context? */
	/* TODO - is there any reason to reject resume? */

accept_resume:
	DBGINFO(("%s accept\n", __func__));
	return 0;
}

/*
 * toggle FPGA hardware reset input
 *
 * FPGA reset is active low and tied to pin 52 of
 * FTDI device and a pull up resistor.
 *
 * pin 52 function depends on channel B mode:
 * 245 FIFO mode = TXE# output (driven low, FPGA reset on)
 * MPSSE mode    = GPIO tristate (pulled high, FPGA reset off)
 *
 * reset pulse width requirement is 10ns, which is guaranteed by
 * the time to send URBs to toggle the signal
 */
static void slusb_reset_fpga(struct slusb_info *info)
{
	struct usb_serial_port *port = info->port;
	struct usb_device *dev = port->serial->dev;
	int rc;

	/* select Channel B 245 FIFO mode - force FPGA reset (TXE#) active low */
	rc = usb_control_msg(dev, usb_sndctrlpipe(dev, 0), FTDI_SET_MODE,
			     USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			     FTDI_MODE_AFIFO, SERVICE_PORT_INDEX, NULL, 0, 5000);
	if (rc < 0)
		dev_err(&port->dev, "%s FIFO mode error=%i\n", __func__, rc);

	mdelay(FTDI_MODE_DELAY);

	/* select Channel B MPSSE mode - force FPGA reset (pulled up) inactive high */
	rc = usb_control_msg(dev, usb_sndctrlpipe(dev, 0), FTDI_SET_MODE,
			     USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			     FTDI_MODE_MPSSE, SERVICE_PORT_INDEX, NULL, 0, 5000);
	if (rc < 0)
		dev_err(&port->dev, "%s MPSSE mode error=%i\n", __func__, rc);

	mdelay(FTDI_MODE_DELAY);
}

/*
 * one time hardware initialization
 *
 * must be called without spinlocks held in user context
 */
static bool slusb_hardware_init(struct slusb_info *info)
{
	struct usb_serial_port *port = info->port;
	struct usb_device *dev = port->serial->dev;
	int rc;
	unsigned int i;
	unsigned int count;

	/* set Channel A (serial port) latency to max (255 msec) */
	rc = usb_control_msg(dev, usb_sndctrlpipe(dev, 0), FTDI_SET_LATENCY,
			     USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			     255, SERIAL_PORT_INDEX, NULL, 0, 5000);
	if (rc < 0)
		dev_err(&port->dev, "set CH A latency timer error=%i\n", rc);

	/* set Channel B (service port) latency to max (255 msec) */
	rc = usb_control_msg(dev, usb_sndctrlpipe(dev, 0), FTDI_SET_LATENCY,
			     USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			     255, SERVICE_PORT_INDEX, NULL, 0, 5000);
	if (rc < 0)
		dev_err(&port->dev, "set CH B latency timer error=%i\n", rc);

	slusb_reset_fpga(info);

	/* select Channel A - 245 async FIFO mode (reset channel) */
	rc = usb_control_msg(dev, usb_sndctrlpipe(dev, 0), FTDI_SET_MODE,
			     USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			     FTDI_MODE_AFIFO, SERIAL_PORT_INDEX, NULL, 0, 5000);
	if (rc < 0)
		dev_err(&port->dev, "CH.A AFIFO mode error=%i\n", rc);

	mdelay(FTDI_MODE_DELAY);

	/* select Channel A - sync FIFO mode */
	rc = usb_control_msg(dev, usb_sndctrlpipe(dev, 0), FTDI_SET_MODE,
			     USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			     FTDI_MODE_SFIFO, SERIAL_PORT_INDEX, NULL, 0, 5000);
	if (rc < 0)
		dev_err(&port->dev, "CH.A SFIFO mode error=%i\n", rc);

	mdelay(FTDI_MODE_DELAY);

	/* flush IN pipe after mode switch */
	for (i=0 ; i < 10 ; i++) {
		char buf[10];
		rc = usb_bulk_msg(dev, info->in_pipe, buf,
				  sizeof(buf), &count, 500);
		if (rc)
			break;
		if (count > 1)
			count -= 2; /* remove status */
		if (!count)
			break;
	}

	if (count) {
		DBGERR(("(%s) IN channel won't flush\n", slusb_dev_name(info)));
		return false;
	}

	/* verify access to serial controller */
	if (!slusb_test_register(info))
		return false;

	/* set frequency synthesizer for default rate */
	slusb_set_clock_word(info, slusb_default_clock_word);
	/* select frequency synthesizer CLK3 output for serial controller */
	slusb_set_mux(info, 0);

	/* verify operation of serial controller */
	if (!slusb_test_loopback(info))
		return false;

	/* set initialization success bit in JCR to turn on green LED */
	slusb_outl(info, JCR, info->jcr_value | (1 << 31));

	/* program Sipex interface */
	slusb_set_interface(info, info->if_mode);

	return true;
}

static void slusb_info_release(struct kref *k)
{
	struct slusb_info *info = container_of(k, struct slusb_info, kref);
	if (info->tmp_rbuf)
		kfree(info->tmp_rbuf);
	if (info->poll_buf)
		usb_buffer_free(info->port->serial->dev, POLL_BUFFER_SIZE,
				info->poll_buf, info->poll_urb->transfer_dma);
	if (info->poll_urb)
		usb_free_urb(info->poll_urb);
	kfree(info);
}

/*
 * called by usb-serial core for each serial port of
 * USB device when inserted
 */
static int slusb_port_probe(struct usb_serial_port *port)
{
	struct slusb_info    *info;
	struct usb_device    *usb_dev = port->serial->dev;
	struct usb_interface *usb_if  = port->serial->interface;
	struct usb_endpoint_descriptor *ep_desc;
	unsigned int count;
	unsigned int i;

	info = kzalloc(sizeof(struct slusb_info), GFP_KERNEL);
	if (!info) {
		DBGERR(("%s kzalloc slusb_info failed\n", __func__));
		dev_err(&port->dev, "%s: kzalloc(%Zd) failed.\n",
			__func__, sizeof(struct slusb_info));
		return -ENOMEM;
	}

	INIT_WORK(&info->poll_work, slusb_work);

	spin_lock_init(&info->lock);
	spin_lock_init(&info->netlock);

	init_waitqueue_head(&info->urb_wait);
	init_waitqueue_head(&info->reg_wait);
	init_waitqueue_head(&info->poll_wait);
	init_waitqueue_head(&info->status_event_wait_q);
	init_waitqueue_head(&info->event_wait_q);

	info->max_frame_size = maxframe;
	info->base_clock = 14745600;
	info->rbuf_fill_level = SLUSB_MAX_FILL_LEVEL;
	memcpy(&info->params, &slusb_default_params, sizeof(MGSL_PARAMS));
	info->idle_mode = HDLC_TXIDLE_FLAGS;

	setup_timer(&info->tx_timer, slusb_tx_timeout, (unsigned long)info);

	info->flags = 0;

	info->tmp_rbuf = kmalloc(info->max_frame_size + 5, GFP_KERNEL);
	if (!info->tmp_rbuf) {
		DBGERR(("%s kmalloc tmp_rbuf failed\n", __func__));
		slusb_info_release(&info->kref);
		return -ENOMEM;
	}

	info->poll_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!info->poll_urb) {
		DBGERR(("%s usb_alloc_urb poll_urb failed\n", __func__));
		slusb_info_release(&info->kref);
		return -ENOMEM;
	}

	kref_init(&info->kref);

	/* associate private data with USB port structure */
	info->port = port;
	usb_set_serial_port_data(port, info);

	count = usb_if->cur_altsetting->desc.bNumEndpoints;

	for (i = 0; i < count; i++) {
		ep_desc = &usb_if->cur_altsetting->endpoint[i].desc;
		if (usb_endpoint_dir_in(ep_desc)) {
			info->serial_in_max_size =
				le16_to_cpu(ep_desc->wMaxPacketSize);
			info->serial_in_pipe =
				usb_rcvbulkpipe(usb_dev, ep_desc->bEndpointAddress);
		} else {
			info->serial_out_max_size =
				le16_to_cpu(ep_desc->wMaxPacketSize);
			info->serial_out_pipe =
				usb_sndbulkpipe(usb_dev, ep_desc->bEndpointAddress);
		}
	}

	/* allocate memory for poll buffer */
	info->poll_buf = usb_buffer_alloc(info->port->serial->dev, POLL_BUFFER_SIZE,
					  GFP_ATOMIC, &info->poll_urb->transfer_dma);
	if (!info->poll_buf) {
		DBGERR(("%s usb_buffer_alloc info->poll_buf failed\n", __func__));
		dev_err(&port->dev, "%s: usb_buffer_alloc(%d) failed.\n",
			__func__, POLL_BUFFER_SIZE);
		return -ENOMEM;
	}

	slusb_select_serial_pipes(info);

	if (!slusb_hardware_init(info)) {
		DBGERR(("%s slusb_hardware_init failed\n", __func__));
		usb_set_serial_port_data(port, NULL);
		kref_put(&info->kref, slusb_info_release);
		return -ENODEV;
	}

#if SYNCLINK_GENERIC_HDLC
	hdlcdev_init(info);
#endif
	return 0;
}

/*
 * called by usb-serial layer for each port when device is disconnected
 */
static int slusb_port_remove(struct usb_serial_port *port)
{
	struct slusb_info *info = usb_get_serial_port_data(port);
	if (!info) {
		dev_err(&port->dev, "%s: slusb_info is null", __func__);
		return 0;
	}
	DBGINFO(("%s %s\n", __func__, slusb_dev_name(info)));
#if SYNCLINK_GENERIC_HDLC
	hdlcdev_exit(info);
#endif
	kref_put(&info->kref, slusb_info_release);
	return 0;
}

/*
 * called on completion of OUT channel bulk packet
 */
static void slusb_out_bulk_complete(struct urb *urb)
{
	struct slusb_bulk_context *context = urb->context;
	struct slusb_info *info = context->info;

	if (urb->status) {
		dev_err(&info->port->dev, "%s URB error=%08x\n",
			__func__, urb->status);
	}

	if (context->func)
		context->func(info);

	kfree(context);
	usb_free_urb(urb);

	if (atomic_dec_and_test(&info->urb_count))
		wake_up_interruptible(&info->urb_wait);
}

/*
 * do bulk transfer on out pipe
 * return 0 on success or error code
 */
int slusb_out_bulk(struct slusb_info *info, unsigned char *buf,
		   unsigned int size, void (*func)(struct slusb_info *))
{
	struct usb_device *dev = info->port->serial->dev;
	struct urb *urb;
	struct slusb_bulk_context *context;
	int rc;

	if (size > sizeof(context->buf)) {
		dev_err(&info->port->dev, "%s size too large\n", __func__);
		return -EINVAL;
	}

	urb = usb_alloc_urb(0, GFP_ATOMIC);
	if (!urb) {
		dev_err(&info->port->dev, "%s usb_alloc_urb fail\n", __func__);
		return -ENOMEM;
	}

	context = kmalloc(sizeof(*context), GFP_ATOMIC);
	if (!context) {
		usb_free_urb(urb);
		dev_err(&info->port->dev, "%s kmalloc fail\n", __func__);
		return -ENOMEM;
	}
	context->info = info;
	context->func = func;
	memcpy(context->buf, buf, size);

	usb_fill_bulk_urb(urb, dev, info->out_pipe, context->buf, size,
			  slusb_out_bulk_complete, context);

	atomic_inc(&info->urb_count);

	rc = usb_submit_urb(urb, GFP_ATOMIC);
	if (rc) {
		dev_err(&info->port->dev, "%s usb_submit_urb error=%d\n",
			__func__, rc);
		kfree(context);
		usb_free_urb(urb);
		if (atomic_dec_and_test(&info->urb_count))
			wake_up_interruptible(&info->urb_wait);
	}

	return rc;
}

/*
 * called on completion of rx poll URB in interrupt context
 */
static void slusb_poll_complete(struct urb *urb)
{
	struct usb_serial_port *port = urb->context;
	struct slusb_info *info = usb_get_serial_port_data(port);
	unsigned char *buf = urb->transfer_buffer;
	unsigned int count = urb->actual_length;
	unsigned int packet_size;
	unsigned long flags;

	spin_lock_irqsave(&info->lock, flags);

	if (urb->status) {
		DBGPOLL(("%s %s URB error=%08x\n",
			 slusb_dev_name(info), __func__, urb->status));
		info->poll_stop = true;
	}
	if (info->poll_stop) {
		DBGPOLL(("%s %s poll stopped\n",
			 slusb_dev_name(info), __func__));
		info->poll_active = false;
		wake_up_interruptible(&info->poll_wait);
		spin_unlock_irqrestore(&info->lock, flags);
		return;
	}

	if (count > 2) {
		DBGPOLL(("%s poll transfer size=%d\n",
			 slusb_dev_name(info), count));
	}

	/*
	 * - transfer buffer contains one or more bulk packets
	 * - each packet starts with 2 byte FTDI status
	 * - all packets except last packet are max packet size
	 */

	while (count > 2) {

		/* process one packet */
		if (count > info->in_max_size)
			packet_size = info->in_max_size;
		else
			packet_size = count;

		memcpy(info->stat2, info->stat1, 2);
		memcpy(info->stat1, buf, 2);

		/* discard leading FTDI status */
		packet_size -= 2;
		count       -= 2;
		buf         += 2;

		DBGPOLL(("%s USB packet size=%d\n",
			 slusb_dev_name(info), packet_size));

		if (info->params.mode == MGSL_MODE_MSC_PROM) {
			/* pass service data directly to user */
			slusb_rbuf_put(info, buf, packet_size);
			info->pending_work |= SLUSB_WORK_RX;
			DBGPOLL(("%s %s rx work pending\n",
				 slusb_dev_name(info), __func__));
		} else {
			/* store serial data for later parsing */
			slusb_inbuf_put(info, buf, packet_size);
			/* TODO start debug */
			memcpy(info->buf2, info->buf1, info->count1);
			info->count2 = info->count1;
			memcpy(info->buf1, buf, packet_size);
			info->count1 = packet_size;
			/* TODO end debug */
		}

		/* move past current packet */
		count -= packet_size;
		buf   += packet_size;
	}

	/* restart poll */
	if (slusb_poll(info)) {
		info->poll_active = false;
		wake_up_interruptible(&info->poll_wait);
	}

	/* parse complete serial protocol packets */
	do {
		count = slusb_inbuf_process(info);
	} while (count);

	if (info->pending_work && !info->work_running) {
		DBGPOLL(("%s work queued\n", slusb_dev_name(info)));
		schedule_work(&info->poll_work);
	}

	spin_unlock_irqrestore(&info->lock, flags);
}

/*
 * poll bulk in pipe for data
 */
static int slusb_poll(struct slusb_info *info)
{
	int rc;
	info->poll_active = true;
	rc = usb_submit_urb(info->poll_urb, GFP_ATOMIC);
	if (rc) {
		dev_err(&info->port->dev, "%s usb_submit_urb error=%08x\n",
			__func__, rc);
		info->poll_active = false;
	}
	return rc;
}

/*
 * start polling for bulk rx packets
 * return TRUE on success, otherwise FALSE
 */
static bool slusb_poll_start(struct slusb_info *info)
{
	DBGINFO(("%s %s\n", slusb_dev_name(info), __func__));
	if (!info->poll_active) {
		info->poll_stop = false;
		slusb_inbuf_reset(info);
		if (slusb_poll(info))
			return false;
	}
	return true;
}

/*
 * stop polling for bulk rx packets and wait for completion
 * return TRUE on success, otherwise FALSE
 */
static bool slusb_poll_stop(struct slusb_info *info)
{
	int rc;
	DBGINFO(("%s %s\n", slusb_dev_name(info), __func__));
	if (!info->poll_active)
		return true;
	info->poll_stop = true;
	rc = wait_event_interruptible_timeout(info->poll_wait,
					      !info->poll_active, 1*HZ);
	if (!rc) {
		printk(KERN_ERR "%s timeout waiting for poll stop\n", __func__);
		return false;
	}
	return true;
}

/*
 *  set transmit idle mode
 */
static void slusb_tx_set_idle_queue(struct slusb_info *info)
{
	unsigned char val;
	unsigned short tcr;

	/* if preamble enabled (tcr[6] == 1) then tx idle size = 8 bits
	 * else tcr[5:4] = tx idle size: 00 = 8 bits, 01 = 16 bits
	 */
	tcr = slusb_outw_saved(info, TCR);
	if (info->idle_mode & HDLC_TXIDLE_CUSTOM_16) {
		/* disable preamble, set idle size to 16 bits */
		tcr = (tcr & ~(BIT6 + BIT5)) | BIT4;
		/* MSB of 16 bit idle specified in tx preamble register (TPR) */
		slusb_outb_queue(info, TPR, (unsigned char)((info->idle_mode >> 8) & 0xff));
	} else if (!(tcr & BIT6)) {
		/* preamble is disabled, set idle size to 8 bits */
		tcr &= ~(BIT5 + BIT4);
	}
	slusb_outb_queue(info, TCR, tcr);

	if (info->idle_mode & (HDLC_TXIDLE_CUSTOM_8 | HDLC_TXIDLE_CUSTOM_16)) {
		/* LSB of custom tx idle specified in tx idle register */
		val = (unsigned char)(info->idle_mode & 0xff);
	} else {
		/* standard 8 bit idle patterns */
		switch(info->idle_mode)
		{
		case HDLC_TXIDLE_FLAGS:          val = 0x7e; break;
		case HDLC_TXIDLE_ALT_ZEROS_ONES:
		case HDLC_TXIDLE_ALT_MARK_SPACE: val = 0xaa; break;
		case HDLC_TXIDLE_ZEROS:
		case HDLC_TXIDLE_SPACE:          val = 0x00; break;
		default:                         val = 0xff;
		}
	}

	slusb_outb_queue(info, TIR, val);
}

static void slusb_tx_set_idle(struct slusb_info *info)
{
	slusb_tx_set_idle_queue(info);
	slusb_outbuf_flush(info, NULL);
}

/*
 *  set baud rate generator to specified rate
 */
static void slusb_set_rate(struct slusb_info *info, u32 rate)
{
	unsigned int div;
	unsigned int osc = info->base_clock;

	/* div=(osc/rate)-1 (round up to next slowest rate) */
	if (rate) {
		div = osc/rate;
		if (!(osc % rate) && div)
			div--;
		slusb_outw_queue(info, BDR, (unsigned short)div);
	}
}

/*
 * enable internal loopback
 * TxCLK and RxCLK are generated from BRG
 * and TxD is looped back to RxD internally.
 */
static void slusb_enable_loopback(struct slusb_info *info)
{
	unsigned short val;

	/* SCR (serial control) BIT2=looopback enable */
	val = slusb_outw_saved(info, SCR) | BIT2;
	slusb_outw_queue(info, SCR, val);

	if (info->params.mode != MGSL_MODE_ASYNC) {
		/* CCR (clock control)
		 * 07..05  tx clock source (010 = BRG)
		 * 04..02  rx clock source (010 = BRG)
		 * 01      auxclk enable   (0 = disable)
		 * 00      BRG enable      (1 = enable)
		 *
		 * 0100 1001
		 */
		slusb_outb_queue(info, CCR, 0x49);

		/* set speed if available, otherwise use default */
		if (info->params.clock_speed)
			slusb_set_rate(info, info->params.clock_speed);
		else
			slusb_set_rate(info, 1843200);
	}
}

static void slusb_select_serial_pipes(struct slusb_info *info)
{
	info->in_pipe      = info->serial_in_pipe;
	info->in_max_size  = info->serial_in_max_size;
	info->out_pipe     = info->serial_out_pipe;
	info->out_max_size = info->serial_out_max_size;
	usb_fill_bulk_urb(info->poll_urb,
			  info->port->serial->dev, info->in_pipe,
			  info->poll_buf, POLL_BUFFER_SIZE,
			  slusb_poll_complete, info->port);
	info->poll_urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
}

static void slusb_select_service_pipes(struct slusb_info *info)
{
	info->in_pipe      = info->service_in_pipe;
	info->in_max_size  = info->service_in_max_size;
	info->out_pipe     = info->service_out_pipe;
	info->out_max_size = info->service_out_max_size;
	usb_fill_bulk_urb(info->poll_urb,
			  info->port->serial->dev, info->in_pipe,
			  info->poll_buf, POLL_BUFFER_SIZE,
			  slusb_poll_complete, info->port);
	info->poll_urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
}

/*
 * setup hardware for programming the serial controller PROM
 */
static void slusb_prom_mode(struct slusb_info *info)
{
	slusb_select_service_pipes(info);
	slusb_poll_start(info);
}

/*
 * program hardware for asynchronous mode
 */
static void slusb_async_mode(struct slusb_info *info)
{
	unsigned short val;

	slusb_poll_start(info);

	slusb_irq_off_queue(info, IRQ_ALL | IRQ_MASTER);
	slusb_tx_stop_queue(info);
	slusb_rx_stop_queue(info);

	/* TCR (tx control)
	 *
	 * 15..13  mode, 010=async
	 * 12..10  encoding, 000=NRZ
	 * 09      parity enable
	 * 08      1=odd parity, 0=even parity
	 * 07      RTS driver control 1=enabled
	 * 06      1=break enable
	 * 05..04  character length
	 *         00=5 bits
	 *         01=6 bits
	 *         10=7 bits
	 *         11=8 bits
	 * 03      0=1 stop bit, 1=2 stop bits
	 * 02      reset
	 * 01      enable
	 * 00      auto-CTS enable
	 */
	val = 0x4000;

	if (info->if_mode & MGSL_INTERFACE_RTS_EN)
		val |= BIT7;

	if (info->params.parity != ASYNC_PARITY_NONE) {
		val |= BIT9;
		if (info->params.parity == ASYNC_PARITY_ODD)
			val |= BIT8;
	}

	switch (info->params.data_bits) {
	case 6:
		val |= BIT4;
		break;
	case 7:
		val |= BIT5;
		break;
	case 8:
		val |= BIT5 + BIT4;
		break;
	}

	if (info->params.stop_bits != 1)
		val |= BIT3;

	if (info->params.flags & HDLC_FLAG_AUTO_CTS)
		val |= BIT0;

	slusb_outw_queue(info, TCR, val);

	/* RCR (rx control)
	 *
	 * 15..13  mode, 010=async
	 * 12..10  encoding, 000=NRZ
	 * 09      parity enable
	 * 08      1=odd parity, 0=even parity
	 * 07..06  reserved, must be 0
	 * 05..04  character length
	 *         00=5 bits
	 *         01=6 bits
	 *         10=7 bits
	 *         11=8 bits
	 * 03      reserved, must be zero
	 * 02      reset
	 * 01      enable
	 * 00      auto-DCD enable
	 */
	val = 0x4000;

	if (info->params.parity != ASYNC_PARITY_NONE) {
		val |= BIT9;
		if (info->params.parity == ASYNC_PARITY_ODD)
			val |= BIT8;
	}

	switch (info->params.data_bits) {
	case 6:
		val |= BIT4;
		break;
	case 7:
		val |= BIT5;
		break;
	case 8:
		val |= BIT5 + BIT4;
		break;
	}

	if (info->params.flags & HDLC_FLAG_AUTO_DCD)
		val |= BIT0;

	slusb_outw_queue(info, RCR, val);

	if (info->params.data_rate == 0) {
		/* isochronous mode */

		/* CCR (clock control)
		*
		* 07..05  tx clock source
		* 04..02  rx clock source
		* 01      auxclk enable
		* 00      BRG enable
		*/
		val = 0;

		if (info->params.flags & HDLC_FLAG_TXC_BRG) {
			// when RxC source is DPLL, BRG generates 16X DPLL
			// reference clock, so take TxC from BRG/16 to get
			// transmit clock at actual data rate
			// Note: When x8 sampling is selected TxC is BRG/8
			if (info->params.flags & HDLC_FLAG_RXC_DPLL)
				val |= BIT6 + BIT5;	/* 011, txclk = BRG/16 */
			else
				val |= BIT6;	/* 010, txclk = BRG */
		}
		else if (info->params.flags & HDLC_FLAG_TXC_DPLL)
			val |= BIT7;	/* 100, txclk = DPLL Input */
		else if (info->params.flags & HDLC_FLAG_TXC_RXCPIN)
			val |= BIT5;	/* 001, txclk = RXC Input */

		if (info->params.flags & HDLC_FLAG_RXC_BRG)
			val |= BIT3;	/* 010, rxclk = BRG */
		else if (info->params.flags & HDLC_FLAG_RXC_DPLL)
			val |= BIT4;	/* 100, rxclk = DPLL */
		else if (info->params.flags & HDLC_FLAG_RXC_TXCPIN)
			val |= BIT2;	/* 001, rxclk = TXC Input */

		if (info->params.clock_speed)
			val |= BIT1 + BIT0;

		slusb_outb_queue(info, CCR, (unsigned char)val);
	} else {
		/* asynchronous */

		/* CCR (clock control)
		*
		* 07..05  011 = tx clock source is BRG/16
		* 04..02  010 = rx clock source is BRG
		* 01      0 = auxclk disabled
		* 00      1 = BRG enabled
		*
		* 0110 1001
		*/
		slusb_outb_queue(info, CCR, 0x69);
	}

	slusb_set_vcr(info);
	slusb_irq_on_queue(info, IRQ_RXBREAK | IRQ_RXOVER);

	/* SCR (serial control)
	 *
	 * 15  1=tx req on FIFO half empty
	 * 14  1=rx req on FIFO half full
	 * 13  tx data  IRQ enable
	 * 12  tx idle  IRQ enable
	 * 11  rx break on IRQ enable
	 * 10  rx data  IRQ enable
	 * 09  rx break off IRQ enable
	 * 08  overrun  IRQ enable
	 * 07  DSR      IRQ enable
	 * 06  CTS      IRQ enable
	 * 05  DCD      IRQ enable
	 * 04  RI       IRQ enable
	 * 03  0=16x sampling, 1=8x sampling
	 * 02  1=txd->rxd internal loopback enable
	 * 01  0=full duplex, 1=half duplex
	 * 00  1=master IRQ enable
	 */
	val = BIT15 + BIT14 + BIT7 + BIT6 + BIT5 + BIT0;
	switch(info->if_mode & MGSL_INTERFACE_MASK) {
	case MGSL_INTERFACE_RS232:
	case MGSL_INTERFACE_V35:
		val |= BIT4; /* enable RI IRQ */
	}
	if (info->params.loopback)
		val |= BIT2;
	/* JCR[9] : 1 = RS485 half duplex mode feature available */
	if ((info->jcr_value & BIT9) &&
	    info->if_mode & MGSL_INTERFACE_HALF_DUPLEX) {
		val |= BIT1;
	}

	if (info->params.data_rate == 0) {
		/* isochronous */
		if (info->params.clock_speed)
			slusb_set_rate(info, info->params.clock_speed);
	} else {
		/* asynchronous */

		/* JCR[8] : 1 = x8 async mode feature available */
		if ( (info->jcr_value & BIT8) &&
		     ((info->base_clock < (info->params.data_rate * 16)) ||
		      (info->base_clock % (info->params.data_rate * 16))) ) {
			/* use 8x sampling */
			val |= BIT3;
			slusb_set_rate(info, info->params.data_rate * 8);
		} else {
			/* use 16x sampling */
			slusb_set_rate(info, info->params.data_rate * 16);
		}
	}

	slusb_outw_queue(info, SCR, val);

	/* get initial serial signal states */
	slusb_inw_queue(info, SSR);

	if (info->params.loopback)
		slusb_enable_loopback(info);

	slusb_outbuf_flush(info, NULL);
}

/*
 * program hardware for synchronous (HDLC/raw/monosync/bisync) mode
 */
void slusb_sync_mode(struct slusb_info *info)
{
	unsigned short val;

	slusb_poll_start(info);

	slusb_irq_off_queue(info, IRQ_ALL | IRQ_MASTER);
	slusb_tx_stop_queue(info);
	slusb_rx_stop_queue(info);

	/* TCR (tx control)
	 *
	 * 15..13  mode, 000=HDLC 001=raw 010=async 011=monosync 100=bisync
	 * 12..10  encoding
	 * 09      CRC enable
	 * 08      CRC32
	 * 07      RTS driver control 1=enabled
	 * 06      preamble enable
	 * 05..04  preamble length
	 * 03      share open/close flag
	 * 02      reset
	 * 01      enable
	 * 00      auto-CTS enable
	 */
	val = BIT2;

	switch(info->params.mode) {
	case MGSL_MODE_XSYNC:
		val |= BIT15 + BIT13;
		break;
	case MGSL_MODE_MONOSYNC:
		val |= BIT14 + BIT13;
		break;
	case MGSL_MODE_BISYNC:
		val |= BIT15;
		break;
	case MGSL_MODE_RAW:
		val |= BIT13;
		break;
	}
	if (info->if_mode & MGSL_INTERFACE_RTS_EN)
		val |= BIT7;

	switch(info->params.encoding) {
	case HDLC_ENCODING_NRZB:
		val |= BIT10;
		break;
	case HDLC_ENCODING_NRZI_MARK:
		val |= BIT11;
		break;
	case HDLC_ENCODING_NRZI:
		val |= BIT11 + BIT10;
		break;
	case HDLC_ENCODING_BIPHASE_MARK:
		val |= BIT12;
		break;
	case HDLC_ENCODING_BIPHASE_SPACE:
		val |= BIT12 + BIT10;
		break;
	case HDLC_ENCODING_BIPHASE_LEVEL:
		val |= BIT12 + BIT11;
		break;
	case HDLC_ENCODING_DIFF_BIPHASE_LEVEL:
		val |= BIT12 + BIT11 + BIT10;
		break;
	}

	switch (info->params.crc_type & HDLC_CRC_MASK) {
	case HDLC_CRC_16_CCITT:
		val |= BIT9;
		break;
	case HDLC_CRC_32_CCITT:
		val |= BIT9 + BIT8;
		break;
	}

	if (info->params.preamble != HDLC_PREAMBLE_PATTERN_NONE)
		val |= BIT6;

	switch (info->params.preamble_length) {
	case HDLC_PREAMBLE_LENGTH_16BITS:
		val |= BIT5;
		break;
	case HDLC_PREAMBLE_LENGTH_32BITS:
		val |= BIT4;
		break;
	case HDLC_PREAMBLE_LENGTH_64BITS:
		val |= BIT5 + BIT4;
		break;
	}

	if (info->params.flags & HDLC_FLAG_AUTO_CTS)
		val |= BIT0;

	slusb_outw_queue(info, TCR, val);

	/* TPR (transmit preamble) */

	switch (info->params.preamble) {
	case HDLC_PREAMBLE_PATTERN_ONES:
		val = 0xff;
		break;
	case HDLC_PREAMBLE_PATTERN_ZEROS:
		val = 0x00;
		break;
	case HDLC_PREAMBLE_PATTERN_10:
		val = 0x55;
		break;
	case HDLC_PREAMBLE_PATTERN_01:
		val = 0xaa;
		break;
	default:
		val = 0x7e;
	}

	slusb_outb_queue(info, TPR, (unsigned char)val);


	/* RCR (rx control)
	 *
	 * 15..13  mode
	 *         000=HDLC/SDLC
	 *         001=raw bit synchronous
	 *         010=asynchronous/isochronous
	 *         011=monosync byte synchronous
	 *         100=bisync byte synchronous
	 *         101=xsync byte synchronous
	 * 12..10  encoding
	 * 09      CRC enable
	 * 08      CRC32
	 * 07..03  reserved, must be 0
	 * 02      reset
	 * 01      enable
	 * 00      auto-DCD enable
	 */
	val = 0;

	switch (info->params.mode) {
	case MGSL_MODE_XSYNC:
		val |= BIT15 + BIT13;
		break;
	case MGSL_MODE_MONOSYNC:
		val |= BIT14 + BIT13;
		break;
	case MGSL_MODE_BISYNC:
		val |= BIT15;
		break;
	case MGSL_MODE_RAW:
		val |= BIT13;
		break;
	}

	switch(info->params.encoding) {
	case HDLC_ENCODING_NRZB:
		val |= BIT10;
		break;
	case HDLC_ENCODING_NRZI_MARK:
		val |= BIT11;
		break;
	case HDLC_ENCODING_NRZI:
		val |= BIT11 + BIT10;
		break;
	case HDLC_ENCODING_BIPHASE_MARK:
		val |= BIT12;
		break;
	case HDLC_ENCODING_BIPHASE_SPACE:
		val |= BIT12 + BIT10;
		break;
	case HDLC_ENCODING_BIPHASE_LEVEL:
		val |= BIT12 + BIT11;
		break;
	case HDLC_ENCODING_DIFF_BIPHASE_LEVEL:
		val |= BIT12 + BIT11 + BIT10;
		break;
	}

	switch (info->params.crc_type & HDLC_CRC_MASK) {
	case HDLC_CRC_16_CCITT:
		val |= BIT9;
		break;
	case HDLC_CRC_32_CCITT:
		val |= BIT9 + BIT8;
		break;
	}

	if (info->params.flags & HDLC_FLAG_AUTO_DCD)
		val |= BIT0;

	slusb_outw_queue(info, RCR, val);

	/* CCR (clock control)
	 *
	 * 07..05  tx clock source
	 * 04..02  rx clock source
	 * 01      auxclk enable
	 * 00      BRG enable
	 */
	val = 0;

	if (info->params.flags & HDLC_FLAG_TXC_BRG) {
		// when RxC source is DPLL, BRG generates 16X DPLL
		// reference clock, so take TxC from BRG/16 to get
		// transmit clock at actual data rate
		// Note: When x8 sampling is selected TxC is BRG/8
		if (info->params.flags & HDLC_FLAG_RXC_DPLL)
			val |= BIT6 + BIT5;	/* 011, txclk = BRG/16 */
		else {
			if (info->params.flags & HDLC_FLAG_TXC_INV)
				val |= BIT7 + BIT6 + BIT5; /* 111, txclk = inverted BRG */
			else
				val |= BIT6;	/* 010, txclk = BRG */
		}
	} else if (info->params.flags & HDLC_FLAG_TXC_DPLL)
		val |= BIT7;	/* 100, txclk = DPLL Input */
	else if (info->params.flags & HDLC_FLAG_TXC_RXCPIN) {
		if (info->params.flags & HDLC_FLAG_TXC_INV)
			val |= BIT7 + BIT6;	/* 110, txclk = inverted RXC Input */
		else
			val |= BIT5;	/* 001, txclk = RXC Input */
	} else {
		/* 000 = TXC input */
		if (info->params.flags & HDLC_FLAG_TXC_INV)
			val |= BIT7 + BIT5;	/* 101, txclk = inverted TXC Input */
	}

	if (info->params.flags & HDLC_FLAG_RXC_BRG) {
		if (info->params.flags & HDLC_FLAG_RXC_INV)
			val |= BIT4 + BIT3 + BIT2;	/* 111, rxclk = inverted BRG */
		else
			val |= BIT3; /* 010, rxclk = BRG */
	} else if (info->params.flags & HDLC_FLAG_RXC_DPLL)
		val |= BIT4;	/* 100, rxclk = DPLL */
	else if (info->params.flags & HDLC_FLAG_RXC_TXCPIN) {
		if (info->params.flags & HDLC_FLAG_RXC_INV)
			val |= BIT4 + BIT3;	/* 110, rxclk = inverted TXC Input */
		else
			val |= BIT2;	/* 001, rxclk = TXC Input */
	} else {
		/* 000 = RXC input */
		if (info->params.flags & HDLC_FLAG_RXC_INV)
			val |= BIT4 + BIT2;	/* 101, rxclk = inverted RXC Input */
	}

	if (info->params.clock_speed)
		val |= BIT1 + BIT0;

	slusb_outb_queue(info, CCR, (unsigned char)val);

	if (info->params.flags & (HDLC_FLAG_TXC_DPLL + HDLC_FLAG_RXC_DPLL)) {
		// program DPLL mode
		switch(info->params.encoding) {
		case HDLC_ENCODING_BIPHASE_MARK:
		case HDLC_ENCODING_BIPHASE_SPACE:
			val = BIT7;
			break;
		case HDLC_ENCODING_BIPHASE_LEVEL:
		case HDLC_ENCODING_DIFF_BIPHASE_LEVEL:
			val = BIT7 + BIT6;
			break;
		default:
			val = BIT6; // NRZ encodings
		}
		val |= slusb_outw_saved(info, RCR);
		slusb_outw_queue(info, RCR, val);

		if (info->params.flags & HDLC_FLAG_DPLL_DIV8) {
			// DPLL requires a 8X reference clock from BRG
			slusb_set_rate(info, info->params.clock_speed * 8);
		} else {
			// DPLL requires a 16X reference clock from BRG
			slusb_set_rate(info, info->params.clock_speed * 16);
		}
	} else
		slusb_set_rate(info, info->params.clock_speed);

	slusb_tx_set_idle_queue(info);
	slusb_set_vcr(info);

	/* SCR (serial control)
	 *
	 * 15  1=tx req on FIFO half empty
	 * 14  1=rx req on FIFO half full
	 * 13  tx data  IRQ enable
	 * 12  tx idle  IRQ enable
	 * 11  underrun IRQ enable
	 * 10  rx data  IRQ enable
	 * 09  rx idle  IRQ enable
	 * 08  overrun  IRQ enable
	 * 07  DSR      IRQ enable
	 * 06  CTS      IRQ enable
	 * 05  DCD      IRQ enable
	 * 04  RI       IRQ enable
	 * 03  DPLL Sample Rate: 0=x16 1=x8
	 * 02  1=txd->rxd internal loopback enable
	 * 01  0=full duplex, 1=half duplex
	 * 00  1=master IRQ enable
	 */
	val = BIT15 + BIT14 + BIT7 + BIT6 + BIT5 + BIT0;
	switch(info->if_mode & MGSL_INTERFACE_MASK) {
	case MGSL_INTERFACE_RS232:
	case MGSL_INTERFACE_V35:
		val |= BIT4; /* enable RI IRQ */
	}
	/* JCR[9] : 1 = RS485 half duplex mode feature available */
	if ((info->jcr_value & BIT9) &&
	    info->if_mode & MGSL_INTERFACE_HALF_DUPLEX) {
		val |= BIT1;
	}
	if (info->params.flags & HDLC_FLAG_DPLL_DIV8)
		val |= BIT3;
	slusb_outw_queue(info, SCR, val);

	/* get initial serial signal states */
	slusb_inw_queue(info, SSR);

	if (info->params.loopback)
		slusb_enable_loopback(info);

	slusb_outbuf_flush(info, NULL);
}

/*
 * shutdown hardware when device closed
 */
static void slusb_shutdown(struct slusb_info *info)
{
	struct tty_struct *tty = tty_port_tty_get(&info->port->port);
	unsigned long flags;
	int rc;

	DBGINFO(("%s shutdown\n", slusb_dev_name(info)));

	slusb_poll_stop(info);
	cancel_work_sync(&info->poll_work);

	wake_up_interruptible(&info->status_event_wait_q);
	wake_up_interruptible(&info->event_wait_q);
	del_timer_sync(&info->tx_timer);

	spin_lock_irqsave(&info->lock, flags);

	slusb_tx_stop(info);
	slusb_rx_stop(info);
	slusb_irq_off(info, IRQ_ALL | IRQ_MASTER);

	if (!tty || tty_cflags(tty) & HUPCL) {
		info->signals &= ~(SerialSignal_DTR + SerialSignal_RTS);
		slusb_set_signals(info);
	}
	if (tty)
		tty_kref_put(tty);

	flush_cond_wait(&info->gpio_wait_q);

	slusb_inbuf_reset(info);
	slusb_rbuf_reset(info);
	slusb_outbuf_reset(info);

	spin_unlock_irqrestore(&info->lock, flags);

	/* wait until all URBs complete */
	rc = wait_event_interruptible_timeout(info->urb_wait,
					      !atomic_read(&info->urb_count), 5*HZ);
	if (!rc)
		printk(KERN_ERR "%s URB count timeout\n", __func__);
}

static void slusb_program_hw(struct slusb_info *info)
{
	unsigned long flags;
	struct tty_struct *tty = tty_port_tty_get(&info->port->port);

	spin_lock_irqsave(&info->lock, flags);

	switch (info->params.mode) {
	case MGSL_MODE_ASYNC:
		slusb_async_mode(info);
		break;
	case MGSL_MODE_MSC_PROM:
		slusb_prom_mode(info);
		break;
	default:
		slusb_sync_mode(info);
		break;
	}

	if (info->params.mode != MGSL_MODE_MSC_PROM) {
		slusb_set_signals(info);
		if (info->net_open ||
		    (tty && tty_cflags(tty) & CREAD))
			slusb_rx_start(info);
	}

	if (tty)
		tty_kref_put(tty);

	spin_unlock_irqrestore(&info->lock, flags);
}

/*
 * reconfigure adapter based on new parameters
 */
static void slusb_change_params(struct slusb_info *info)
{
	unsigned cflag;
	int bits_per_char;
	struct tty_struct *tty = tty_port_tty_get(&info->port->port);

#if LINUX_VERSION_CODE < VERSION(3,7,0)
	if (!tty || !tty->termios)
		return;
#else
	if (!tty)
		return;
#endif

	DBGINFO(("%s change_params\n", slusb_dev_name(info)));

	cflag = tty_cflags(tty);

	/* turn on DTR+RTS unless B0 rate (hangup) specified */
	if (cflag & CBAUD)
		info->signals |= SerialSignal_RTS + SerialSignal_DTR;
	else
		info->signals &= ~(SerialSignal_RTS + SerialSignal_DTR);

	switch (cflag & CSIZE) {
	case CS5:
		info->params.data_bits = 5;
		break;
	case CS6:
		info->params.data_bits = 6;
		break;
	case CS7:
		info->params.data_bits = 7;
		break;
	case CS8:
		info->params.data_bits = 8;
		break;
	default:
		info->params.data_bits = 7;
	}

	if (cflag & CSTOPB)
		info->params.stop_bits = 2;
	else
		info->params.stop_bits = 1;

	if (cflag & PARENB) {
		if (cflag & PARODD)
			info->params.parity = ASYNC_PARITY_ODD;
		else
			info->params.parity = ASYNC_PARITY_EVEN;
	} else
		info->params.parity = ASYNC_PARITY_NONE;

	/* calculate jiffies to transmit full FIFO (32 bytes) */
	bits_per_char = info->params.data_bits +
			info->params.stop_bits + 1;

	info->params.data_rate = tty_get_baud_rate(tty);

	if (info->params.data_rate) {
		info->timeout = (32*HZ*bits_per_char) /
				info->params.data_rate;
	}
	info->timeout += HZ/50;		/* Add .02 seconds of slop */

	if (cflag & CRTSCTS)
		info->flags |= ASYNC_CTS_FLOW;
	else
		info->flags &= ~ASYNC_CTS_FLOW;

	if (cflag & CLOCAL)
		info->flags &= ~ASYNC_CHECK_CD;
	else
		info->flags |= ASYNC_CHECK_CD;

	/* process tty input control flags */

	info->read_status_mask = IRQ_RXOVER;
	if (I_INPCK(tty))
		info->read_status_mask |= MASK_PARITY | MASK_FRAMING;
	if (I_BRKINT(tty) || I_PARMRK(tty))
		info->read_status_mask |= MASK_BREAK;
	if (I_IGNPAR(tty))
		info->ignore_status_mask |= MASK_PARITY | MASK_FRAMING;
	if (I_IGNBRK(tty)) {
		info->ignore_status_mask |= MASK_BREAK;
		/* If ignoring parity and break indicators, ignore
		 * overruns too.  (For real raw support).
		 */
		if (I_IGNPAR(tty))
			info->ignore_status_mask |= MASK_OVERRUN;
	}

	slusb_program_hw(info);

	tty_kref_put(tty);
}

/*
 * called on first open of port to perform per port initialization
 */
static int slusb_open(struct tty_struct *tty, struct usb_serial_port *port)
{
	struct slusb_info *info = usb_get_serial_port_data(port);
	DBGINFO(("%s %s\n", __func__, slusb_dev_name(info)));
	info->tty_open = true;
	memset(&info->icount, 0, sizeof(info->icount));
	slusb_change_params(info);
	return 0;
}

/*
 * called by upper layer to poll state of DCD input
 */
static int slusb_carrier_raised(struct usb_serial_port *port)
{
	struct slusb_info *info = usb_get_serial_port_data(port);
	int rc;

	if (info->signals & SerialSignal_DCD)
		rc = 1;
	else
		rc = 0;
	DBGINFO(("%s %s rc=%d\n", __func__, slusb_dev_name(info), rc));
	return rc;
}

/*
 * called by upper layer to set state of DTR and RTS outputs
 */
static void slusb_dtr_rts(struct usb_serial_port *port, int on)
{
	struct slusb_info *info = usb_get_serial_port_data(port);

	DBGINFO(("%s %s on=%d\n", __func__, slusb_dev_name(info), on));

#if LINUX_VERSION_CODE < VERSION(3,9,0)
	/* note: this 3.9 change was backported to Ubuntu kernels 3.5.7.7+ */
	mutex_lock(&port->serial->disc_mutex);
	if (port->serial->disconnected) {
		mutex_unlock(&port->serial->disc_mutex);
		return;
	}
#endif

	if (on) {
		if (!(info->signals & SerialSignal_DTR) ||
		    !(info->signals & SerialSignal_RTS)) {
			info->signals |= SerialSignal_DTR + SerialSignal_RTS;
			slusb_set_signals(info);
		}
	} else {
		if (info->signals & (SerialSignal_DTR | SerialSignal_RTS)) {
			info->signals &= ~(SerialSignal_DTR | SerialSignal_RTS);
			slusb_set_signals(info);
		}
	}

#if LINUX_VERSION_CODE < VERSION(3,9,0)
	mutex_unlock(&port->serial->disc_mutex);
#endif
}

/*
 * called when last open file descriptor closed
 */
static void slusb_close(struct usb_serial_port *port)
{
	struct slusb_info *info = usb_get_serial_port_data(port);
	unsigned long flags;

	DBGINFO(("%s %s\n", __func__, slusb_dev_name(info)));
	slusb_shutdown(info);
	info->tty_open = false;

	/* restore normal operation if in PROM modes */
	if ((info->params.mode == MGSL_MODE_USB_PROM) ||
	    (info->params.mode == MGSL_MODE_MSC_PROM)) {
		spin_lock_irqsave(&info->lock, flags);
		memcpy(&info->params, &slusb_default_params,
		       sizeof(MGSL_PARAMS));
		slusb_select_serial_pipes(info);
		spin_unlock_irqrestore(&info->lock, flags);
	}
}

static int slusb_write(struct tty_struct *tty, struct usb_serial_port *port,
		       const unsigned char *buf, int count)
{
	struct slusb_info *info = usb_get_serial_port_data(port);
	int rc = 0;
	unsigned long flags;

	if (count > info->max_frame_size)
		rc = -EIO;
	else if (info->params.mode == MGSL_MODE_USB_PROM) {
		/* USB interface IC PROM mode */
		slusb_usb_prom_cmd(info, buf, count, tty);
		return rc;
	} else if (info->params.mode == MGSL_MODE_MSC_PROM) {
		/* serial controller PROM mode (buf is MPSSE command) */
		spin_lock_irqsave(&info->lock, flags);
		slusb_out_bulk(info, (unsigned char*)buf, count, NULL);
		rc = count;
		spin_unlock_irqrestore(&info->lock, flags);
	}
	else if (count && !tty->stopped && !tty->hw_stopped) {
		/* normal communications mode */
		spin_lock_irqsave(&info->lock, flags);
		if (slusb_tx(info, buf, count))
			rc = count;
		spin_unlock_irqrestore(&info->lock, flags);
	}

	DBGINFO(("%s write count=%d rc=%d\n", slusb_dev_name(info), count, rc));
	return rc;
}

/*
 * return number of bytes available for write calls
 */
static int slusb_write_room(struct tty_struct *tty)
{
	struct usb_serial_port *port = tty->driver_data;
	struct slusb_info *info = usb_get_serial_port_data(port);

	if (info->tbuf_count < SLUSB_MAX_TBUF_COUNT)
		return info->max_frame_size;
	else
		return 0;
}

static int slusb_chars_in_buffer(struct tty_struct *tty)
{
	struct usb_serial_port *port = tty->driver_data;
	struct slusb_info *info = usb_get_serial_port_data(port);
	struct slusb_tbuf *tbuf;
	int count;
	unsigned long flags;

	spin_lock_irqsave(&info->lock, flags);
	count = 0;
	tbuf = info->tbuf_head;
	while (tbuf) {
		count += tbuf->count;
		tbuf = tbuf->next;
	}
	spin_unlock_irqrestore(&info->lock, flags);
	return count;
}

static void slusb_break_ctl(struct tty_struct *tty, int break_state)
{
	struct usb_serial_port *port = tty->driver_data;
	struct slusb_info *info = usb_get_serial_port_data(port);
	unsigned short value;
	unsigned long flags;

	DBGINFO(("%s set_break(%d)\n", slusb_dev_name(info), break_state));

	spin_lock_irqsave(&info->lock,flags);

	value = slusb_outw_saved(info, TCR);
	if (break_state == -1)
		value |= BIT6; /* enable break*/
	else
		value &= ~BIT6; /* disable break */
	slusb_outw(info, TCR, value);

	spin_unlock_irqrestore(&info->lock,flags);
}

static void slusb_set_termios(struct tty_struct *tty,
			      struct usb_serial_port *port,
			      struct ktermios *old_termios)
{
	struct slusb_info *info = usb_get_serial_port_data(port);
	unsigned long flags;

	DBGINFO(("%s set_termios\n", tty->driver->name));

	slusb_change_params(info);

	/* Handle transition to B0 status */
	if (old_termios->c_cflag & CBAUD &&
	    !(tty_cflags(tty) & CBAUD)) {
		info->signals &= ~(SerialSignal_RTS + SerialSignal_DTR);
		spin_lock_irqsave(&info->lock,flags);
		slusb_set_signals(info);
		spin_unlock_irqrestore(&info->lock,flags);
	}

	/* Handle transition away from B0 status */
	if (!(old_termios->c_cflag & CBAUD) &&
	    tty_cflags(tty) & CBAUD) {
		info->signals |= SerialSignal_DTR;
		if (!(tty_cflags(tty) & CRTSCTS) ||
		    !test_bit(TTY_THROTTLED, &tty->flags)) {
			info->signals |= SerialSignal_RTS;
		}
		spin_lock_irqsave(&info->lock,flags);
		slusb_set_signals(info);
		spin_unlock_irqrestore(&info->lock,flags);
	}

	/* Handle turning off CRTSCTS */
	if (old_termios->c_cflag & CRTSCTS &&
	    !(tty_cflags(tty) & CRTSCTS)) {
		tty->hw_stopped = 0;
	}
}

/*
 * get modem input states ioctl handler
 */
#if LINUX_VERSION_CODE < VERSION(2,6,39)
static int slusb_tiocmget(struct tty_struct *tty, struct file *file)
#else
static int slusb_tiocmget(struct tty_struct *tty)
#endif
{
	struct usb_serial_port *port = tty->driver_data;
	struct slusb_info *info = usb_get_serial_port_data(port);
	int rc;

	/* update SSR and info->signals */
	slusb_inw(info, SSR);

	rc = ((info->signals & SerialSignal_RTS) ? TIOCM_RTS:0) +
		((info->signals & SerialSignal_DTR) ? TIOCM_DTR:0) +
		((info->signals & SerialSignal_DCD) ? TIOCM_CAR:0) +
		((info->signals & SerialSignal_RI)  ? TIOCM_RNG:0) +
		((info->signals & SerialSignal_DSR) ? TIOCM_DSR:0) +
		((info->signals & SerialSignal_CTS) ? TIOCM_CTS:0);

	DBGINFO(("%s tiocmget value=%08X\n", slusb_dev_name(info), rc));
	return rc;
}

/*
 * set modem output states ioctl handler
 */
#if LINUX_VERSION_CODE < VERSION(2,6,39)
static int slusb_tiocmset(struct tty_struct *tty, struct file *file,
			  unsigned int set, unsigned int clear)
#else
static int slusb_tiocmset(struct tty_struct *tty,
			  unsigned int set, unsigned int clear)
#endif
{
	struct usb_serial_port *port = tty->driver_data;
	struct slusb_info *info = usb_get_serial_port_data(port);
	unsigned long flags;

	DBGINFO(("%s tiocmset(%x,%x)\n", slusb_dev_name(info), set, clear));

	if (set & TIOCM_RTS)
		info->signals |= SerialSignal_RTS;
	if (set & TIOCM_DTR)
		info->signals |= SerialSignal_DTR;
	if (clear & TIOCM_RTS)
		info->signals &= ~SerialSignal_RTS;
	if (clear & TIOCM_DTR)
		info->signals &= ~SerialSignal_DTR;

	spin_lock_irqsave(&info->lock, flags);
	slusb_set_signals(info);
	spin_unlock_irqrestore(&info->lock, flags);
	return 0;
}

/*
 * ioctl TIOCMIWAIT handler
 */
static int slusb_tiocmiwait(struct slusb_info *info, int arg)
{
	unsigned long flags;
	int rc;
	struct mgsl_icount cprev, cnow;
	DECLARE_WAITQUEUE(wait, current);

	/* save current irq counts */
	spin_lock_irqsave(&info->lock,flags);
	cprev = info->icount;
	add_wait_queue(&info->status_event_wait_q, &wait);
	set_current_state(TASK_INTERRUPTIBLE);
	spin_unlock_irqrestore(&info->lock,flags);

	for(;;) {
		schedule();
		if (signal_pending(current)) {
			rc = -ERESTARTSYS;
			break;
		}

		/* get new irq counts */
		spin_lock_irqsave(&info->lock,flags);
		cnow = info->icount;
		set_current_state(TASK_INTERRUPTIBLE);
		spin_unlock_irqrestore(&info->lock,flags);

		/* if no change, wait aborted for some reason */
		if (cnow.rng == cprev.rng && cnow.dsr == cprev.dsr &&
		    cnow.dcd == cprev.dcd && cnow.cts == cprev.cts) {
			rc = -EIO;
			break;
		}

		/* check for change in caller specified modem input */
		if ((arg & TIOCM_RNG && cnow.rng != cprev.rng) ||
		    (arg & TIOCM_DSR && cnow.dsr != cprev.dsr) ||
		    (arg & TIOCM_CD  && cnow.dcd != cprev.dcd) ||
		    (arg & TIOCM_CTS && cnow.cts != cprev.cts)) {
			rc = 0;
			break;
		}

		cprev = cnow;
	}
	remove_wait_queue(&info->status_event_wait_q, &wait);
	set_current_state(TASK_RUNNING);
	return rc;
}

static int slusb_get_xsync(struct slusb_info *info, int __user *xsync)
{
	DBGINFO(("%s slusb_get_xsync=%x\n", slusb_dev_name(info), info->xsync));
	if (put_user(info->xsync, xsync))
		return -EFAULT;
	return 0;
}

/*
 * set extended sync pattern (1 to 4 bytes) for extended sync mode
 *
 * sync pattern is contained in least significant bytes of value
 * most significant byte of sync pattern is oldest (1st sent/detected)
 */
static int slusb_set_xsync(struct slusb_info *info, int xsync)
{
	unsigned long flags;

	DBGINFO(("%s slusb_set_xsync=%x)\n", slusb_dev_name(info), xsync));

	spin_lock_irqsave(&info->lock, flags);
	info->xsync = xsync;
	slusb_outl(info, XSR, xsync);
	spin_unlock_irqrestore(&info->lock, flags);

	return 0;
}

static int slusb_get_xctrl(struct slusb_info *info, int __user *xctrl)
{
	DBGINFO(("%s get_xctrl=%x\n", slusb_dev_name(info), info->xctrl));

	if (put_user(info->xctrl, xctrl))
		return -EFAULT;

	return 0;
}

/*
 * set extended control options
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
 */
static int slusb_set_xctrl(struct slusb_info *info, int xctrl)
{
	unsigned long flags;

	DBGINFO(("%s set_xctrl=%x)\n", slusb_dev_name(info), xctrl));

	spin_lock_irqsave(&info->lock, flags);
	info->xctrl = xctrl;
	slusb_outl(info, XCR, xctrl);
	spin_unlock_irqrestore(&info->lock, flags);

	return 0;
}

static int slusb_get_params(struct slusb_info *info, MGSL_PARAMS __user *user_params)
{
	DBGINFO(("%s get_params\n", slusb_dev_name(info)));
	if (copy_to_user(user_params, &info->params, sizeof(MGSL_PARAMS)))
		return -EFAULT;
	return 0;
}

static int slusb_set_params(struct slusb_info *info, MGSL_PARAMS __user *new_params)
{
	unsigned long flags;
	MGSL_PARAMS tmp_params;

	DBGINFO(("%s set_params\n", slusb_dev_name(info)));
	if (copy_from_user(&tmp_params, new_params, sizeof(MGSL_PARAMS)))
		return -EFAULT;

	if ((tmp_params.mode == MGSL_MODE_USB_PROM) ||
	    (tmp_params.mode == MGSL_MODE_MSC_PROM)) {
		slusb_shutdown(info);
		spin_lock_irqsave(&info->lock, flags);
		slusb_select_service_pipes(info);
		info->params.mode = tmp_params.mode;
		slusb_poll(info);
		spin_unlock_irqrestore(&info->lock, flags);
		return 0;
	}

	spin_lock_irqsave(&info->lock, flags);
	if (tmp_params.mode == MGSL_MODE_BASE_CLOCK)
		info->base_clock = tmp_params.clock_speed;
	else
		memcpy(&info->params, &tmp_params, sizeof(MGSL_PARAMS));
	slusb_select_serial_pipes(info);
	spin_unlock_irqrestore(&info->lock, flags);

	slusb_program_hw(info);

	return 0;
}

static int slusb_get_txidle(struct slusb_info *info, int __user *idle_mode)
{
	DBGINFO(("%s get_txidle=%d\n", slusb_dev_name(info), info->idle_mode));
	if (put_user(info->idle_mode, idle_mode))
		return -EFAULT;
	return 0;
}

static int slusb_set_txidle(struct slusb_info *info, int idle_mode)
{
	unsigned long flags;
	DBGINFO(("%s set_txidle(%d)\n", slusb_dev_name(info), idle_mode));
	spin_lock_irqsave(&info->lock, flags);
	info->idle_mode = idle_mode;
	if (info->params.mode != MGSL_MODE_ASYNC)
		slusb_tx_set_idle(info);
	spin_unlock_irqrestore(&info->lock, flags);
	return 0;
}

static int slusb_tx_enable(struct slusb_info *info, int enable)
{
	unsigned long flags;

	DBGINFO(("%s tx_enable(%d)\n", slusb_dev_name(info), enable));

	spin_lock_irqsave(&info->lock, flags);
	if (enable) {
		if (!info->tx_enabled)
			slusb_tx_start(info);
	} else {
		if (info->tx_enabled)
			slusb_tx_stop(info);
	}
	spin_unlock_irqrestore(&info->lock, flags);

	return 0;
}

/*
 * abort transmit HDLC frame
 */
static int slusb_tx_abort(struct slusb_info *info)
{
	unsigned long flags;
	DBGINFO(("%s tx_abort\n", slusb_dev_name(info)));
	spin_lock_irqsave(&info->lock, flags);
	slusb_tx_flush(info);
	spin_unlock_irqrestore(&info->lock, flags);
	return 0;
}

static void slusb_rx_stop_queue(struct slusb_info *info)
{
	unsigned short val;

	/* disable receive data packet mode */
	slusb_outl_queue(info, UCR, 0);

	/* disable (bit 1) and reset (bit 2) receiver */
	val = slusb_outw_saved(info, RCR) & ~BIT1;
	slusb_outw_queue(info, RCR, (unsigned short)(val | BIT2));
	slusb_outw_queue(info, RCR, val);

	slusb_irq_off_queue(info, IRQ_RXOVER + IRQ_RXDATA + IRQ_RXIDLE);

	/* clear pending rx indications */
	slusb_outw_queue(info, SSR, IRQ_RXIDLE + IRQ_RXOVER);

	info->rx_enabled = false;
	info->rx_restart = false;
}

static void slusb_rx_stop(struct slusb_info *info)
{
	slusb_rx_stop_queue(info);
	slusb_outbuf_flush(info, NULL);
}

static void slusb_rx_start_queue(struct slusb_info *info)
{
	unsigned short val;

	slusb_rbuf_reset(info);
	info->rx_pio_count = 0;

	if (info->rx_pio) {
		/* disable receive packet mode */
		slusb_outl_queue(info, UCR, 0);
		/* rx request when rx FIFO not empty */
		val = slusb_outw_saved(info, SCR) & ~BIT14;
		slusb_outw_queue(info, SCR, val);
		slusb_irq_on_queue(info, IRQ_RXOVER + IRQ_RXDATA);
	} else {
		if (info->params.mode == MGSL_MODE_ASYNC) {
			/* enable receive packet mode + save status bytes */
			slusb_outl_queue(info, UCR, 0x101);
		} else {
			/* enable receive packet mode */
			slusb_outl_queue(info, UCR, 0x100);
		}
		/* rx request when rx FIFO half full */
		val = slusb_outw_saved(info, SCR) | BIT14;
		slusb_outw_queue(info, SCR, val);
		slusb_irq_on_queue(info, IRQ_RXOVER);
	}

	/* enable receiver */
	val = slusb_outw_saved(info, RCR) | BIT1;
	slusb_outw_queue(info, RCR, val);

	info->rx_restart = false;
	info->rx_enabled = true;
}

static void slusb_rx_start(struct slusb_info *info)
{
	slusb_rx_start_queue(info);
	slusb_outbuf_flush(info, NULL);
}

static int slusb_rx_enable(struct slusb_info *info, int enable)
{
	unsigned long flags;
	unsigned int rbuf_fill_level;

	DBGINFO(("%s rx_enable(%08x)\n", slusb_dev_name(info), enable));
	spin_lock_irqsave(&info->lock,flags);

	/*
	 * enable[31..16] = receive buffer fill level
	 * 0      = noop (leave fill level unchanged)
	 * 1..127 = PIO mode, 1 byte returned in each USB packet
	 * 128+   = packet mode, 128 bytes returned in each USB packet
	 *
	 * return data to application when level reached
	 */
	rbuf_fill_level = ((unsigned int)enable) >> 16;
	if (rbuf_fill_level) {
		if (rbuf_fill_level > SLUSB_MAX_FILL_LEVEL) {
			spin_unlock_irqrestore(&info->lock, flags);
			return -EINVAL;
		}
		if (rbuf_fill_level < 128)
			info->rx_pio = 1;
		else
			info->rx_pio = 0;
		DBGINFO(("fill level=%d PIO=%d\n", rbuf_fill_level, info->rx_pio));
		info->rbuf_fill_level = rbuf_fill_level;
		slusb_rx_stop_queue(info); /* restart receiver to use new fill level */
	}

	/*
	 * enable[1..0] = receiver enable command
	 * 0 = disable
	 * 1 = enable
	 * 2 = enable or force hunt mode if already enabled
	 */
	enable &= 3;
	if (enable) {
		if (!info->rx_enabled)
			slusb_rx_start_queue(info);
		else if (enable == 2) {
			/* force hunt mode (write 1 to RCR[3]) */
			slusb_outw_queue(info, RCR, slusb_outw_saved(info, RCR) | BIT3);
		}
	} else {
		if (info->rx_enabled)
			slusb_rx_stop_queue(info);
	}

	/* send queued commands */
	slusb_outbuf_flush(info, NULL);

	spin_unlock_irqrestore(&info->lock,flags);
	return 0;
}

static int slusb_get_stats(struct slusb_info *info, struct mgsl_icount __user *user_icount)
{
	DBGINFO(("%s get_stats\n", slusb_dev_name(info)));
	if (!user_icount) {
		memset(&info->icount, 0, sizeof(info->icount));
	} else {
		if (copy_to_user(user_icount, &info->icount, sizeof(struct mgsl_icount)))
			return -EFAULT;
	}
	return 0;
}

/*
 *  wait for specified event to occur
 */
static int slusb_wait_event(struct slusb_info *info, int __user *mask_ptr)
{
	unsigned long flags;
	int s;
	int rc=0;
	struct mgsl_icount cprev;
	struct mgsl_icount cnow;
	int events;
	int mask;
	struct _input_signal_events newsigs;
	struct _input_signal_events oldsigs;
	DECLARE_WAITQUEUE(wait, current);

	if (get_user(mask, mask_ptr))
		return -EFAULT;

	DBGINFO(("%s wait_mgsl_event(%d)\n", slusb_dev_name(info), mask));

	/* update status and info->signals */
	slusb_inw(info, SSR);

	spin_lock_irqsave(&info->lock, flags);

	/* return immediately if state matches requested events */
	s = info->signals;

	events = mask &
		( ((s & SerialSignal_DSR) ? MgslEvent_DsrActive:MgslEvent_DsrInactive) +
		  ((s & SerialSignal_DCD) ? MgslEvent_DcdActive:MgslEvent_DcdInactive) +
		  ((s & SerialSignal_CTS) ? MgslEvent_CtsActive:MgslEvent_CtsInactive) +
		  ((s & SerialSignal_RI)  ? MgslEvent_RiActive :MgslEvent_RiInactive) );
	if (events) {
		spin_unlock_irqrestore(&info->lock, flags);
		goto exit;
	}

	/* save current irq counts */
	cprev = info->icount;
	oldsigs = info->input_signal_events;

	/* enable hunt and idle irqs if needed */
	if (mask & (MgslEvent_ExitHuntMode+MgslEvent_IdleReceived)) {
		unsigned short val = slusb_outw_saved(info, SCR);
		if (!(val & IRQ_RXIDLE))
			slusb_outw(info, SCR, (unsigned short)(val | IRQ_RXIDLE));
	}

	set_current_state(TASK_INTERRUPTIBLE);
	add_wait_queue(&info->event_wait_q, &wait);

	spin_unlock_irqrestore(&info->lock, flags);

	for(;;) {
		schedule();
		if (signal_pending(current)) {
			rc = -ERESTARTSYS;
			break;
		}

		/* get current irq counts */
		spin_lock_irqsave(&info->lock, flags);
		cnow = info->icount;
		newsigs = info->input_signal_events;
		set_current_state(TASK_INTERRUPTIBLE);
		spin_unlock_irqrestore(&info->lock, flags);

		/* if no change, wait aborted for some reason */
		if (newsigs.dsr_up   == oldsigs.dsr_up   &&
		    newsigs.dsr_down == oldsigs.dsr_down &&
		    newsigs.dcd_up   == oldsigs.dcd_up   &&
		    newsigs.dcd_down == oldsigs.dcd_down &&
		    newsigs.cts_up   == oldsigs.cts_up   &&
		    newsigs.cts_down == oldsigs.cts_down &&
		    newsigs.ri_up    == oldsigs.ri_up    &&
		    newsigs.ri_down  == oldsigs.ri_down  &&
		    cnow.exithunt    == cprev.exithunt   &&
		    cnow.rxidle      == cprev.rxidle) {
			rc = -EIO;
			break;
		}

		events = mask &
			( (newsigs.dsr_up   != oldsigs.dsr_up   ? MgslEvent_DsrActive:0)   +
			  (newsigs.dsr_down != oldsigs.dsr_down ? MgslEvent_DsrInactive:0) +
			  (newsigs.dcd_up   != oldsigs.dcd_up   ? MgslEvent_DcdActive:0)   +
			  (newsigs.dcd_down != oldsigs.dcd_down ? MgslEvent_DcdInactive:0) +
			  (newsigs.cts_up   != oldsigs.cts_up   ? MgslEvent_CtsActive:0)   +
			  (newsigs.cts_down != oldsigs.cts_down ? MgslEvent_CtsInactive:0) +
			  (newsigs.ri_up    != oldsigs.ri_up    ? MgslEvent_RiActive:0)    +
			  (newsigs.ri_down  != oldsigs.ri_down  ? MgslEvent_RiInactive:0)  +
			  (cnow.exithunt    != cprev.exithunt   ? MgslEvent_ExitHuntMode:0) +
			  (cnow.rxidle      != cprev.rxidle     ? MgslEvent_IdleReceived:0) );
		if (events)
			break;

		cprev = cnow;
		oldsigs = newsigs;
	}

	remove_wait_queue(&info->event_wait_q, &wait);
	set_current_state(TASK_RUNNING);


	if (mask & (MgslEvent_ExitHuntMode + MgslEvent_IdleReceived)) {
		spin_lock_irqsave(&info->lock, flags);
		if (!waitqueue_active(&info->event_wait_q)) {
			/* disable enable exit hunt mode/idle rcvd IRQs */
			slusb_outw(info, SCR,
				(unsigned short)(slusb_outw_saved(info, SCR) & ~IRQ_RXIDLE));
		}
		spin_unlock_irqrestore(&info->lock, flags);
	}
exit:
	if (rc == 0)
		rc = put_user(events, mask_ptr);
	return rc;
}

static int slusb_get_interface(struct slusb_info *info, int __user *if_mode)
{
	DBGINFO(("%s slusb_get_interface=%x\n", slusb_dev_name(info), info->if_mode));
	if (put_user(info->if_mode, if_mode))
		return -EFAULT;
	return 0;
}

/*
 * map Sipex 3508 configuration signals to XCR register bits
 */
#define SIPEX_LOOPBACK (1 << 24)
#define SIPEX_TERM     (1 << 23)
#define SIPEX_RESERVED (7 << 20)
#define SIPEX_V35      (6 << 20)
#define SIPEX_EIA530   (5 << 20)
#define SIPEX_RS232    (4 << 20)
#define SIPEX_EIA530A  (3 << 20)
#define SIPEX_RS449    (2 << 20)
#define SIPEX_X21      (1 << 20)
#define SIPEX_DISABLED (0)

static int slusb_set_interface(struct slusb_info *info, int if_mode)
{
	unsigned long flags;
	unsigned int value;

	DBGINFO(("%s set_interface=%x\n", slusb_dev_name(info), if_mode));

	spin_lock_irqsave(&info->lock, flags);

	info->if_mode = if_mode;

	/*
	 * XCR
	 * [31:25]  reserved, must be zero
	 * [24]     Sipex loopback (1=enabled, 0=disabled)
	 * [23]     Sipex term off (1=no termination, 0=120 Ohm)
	 * [22:20]  Sipex Mode
	 *          111 = reserved
	 *          110 = V.35
	 *          101 = EIA-530
	 *          100 = RS-232
	 *          011 = EIA-530A
	 *          010 = RS-449
	 *          001 = X.21
	 *          000 = Shutdown (disabled)
	 * [19]     idle select (0=sync, 1=idle pattern)
	 * [18:17]  sync size
	 * [16]     terminal count enable
	 * [15:00]  terminal count minus 1
	 */
	value = slusb_outl_saved(info, XCR) & 0xfe0fffff;
	switch(info->if_mode & MGSL_INTERFACE_MASK) {
	case MGSL_INTERFACE_RS232:
		value |= SIPEX_RS232;
		break;
	case MGSL_INTERFACE_V35:
		value |= SIPEX_V35;
		break;
	case MGSL_INTERFACE_RS422:
		value |= SIPEX_EIA530;
		break;
	case MGSL_INTERFACE_RS530A:
		value |= SIPEX_EIA530A;
		break;
	default:
		value |= SIPEX_DISABLED;
		break;
	}
	if (!(info->if_mode & MGSL_INTERFACE_TERM_OFF))
		value |= SIPEX_TERM;
	slusb_outl_queue(info, XCR, value);

	/* TCR (tx control) 07  1=RTS driver control */
	value = slusb_outw_saved(info, TCR);
	if (info->if_mode & MGSL_INTERFACE_RTS_EN)
		value |= BIT7;
	else
		value &= ~BIT7;
	slusb_outw(info, TCR, (unsigned short)value);

	spin_unlock_irqrestore(&info->lock, flags);

	return 0;
}

/*
 * set general purpose IO pin state and direction
 *
 * user_gpio fields:
 * state   each bit indicates a pin state
 * smask   set bit indicates pin state to set
 * dir     each bit indicates a pin direction (0=input, 1=output)
 * dmask   set bit indicates pin direction to set
 */
static int slusb_set_gpio(struct slusb_info *info, struct gpio_desc __user *user_gpio)
{
	unsigned long flags;
	struct gpio_desc gpio;
	__u32 data;
	bool flush = false;

	if (copy_from_user(&gpio, user_gpio, sizeof(gpio)))
		return -EFAULT;

	DBGINFO(("%s set_gpio state=%08x smask=%08x dir=%08x dmask=%08x\n",
		 slusb_dev_name(info), gpio.state, gpio.smask,
		 gpio.dir, gpio.dmask));

	spin_lock_irqsave(&info->lock, flags);
	if (gpio.dmask) {
		data = slusb_outl_saved(info, IODR);
		data |= gpio.dmask & gpio.dir;
		data &= ~(gpio.dmask & ~gpio.dir);
		slusb_outl_queue(info, IODR, data);
		flush = true;
	}
	if (gpio.smask) {
		data = slusb_outl_saved(info, IOVR);
		data |= gpio.smask & gpio.state;
		data &= ~(gpio.smask & ~gpio.state);
		slusb_outl_queue(info, IOVR, data);
		flush = true;
	}
	if (flush)
		slusb_outbuf_flush(info, NULL);
	spin_unlock_irqrestore(&info->lock, flags);

	return 0;
}

/*
 * get general purpose IO pin state and direction
 */
static int slusb_get_gpio(struct slusb_info *info, struct gpio_desc __user *user_gpio)
{
	struct gpio_desc gpio;

	if (!info->gpio_present)
		return -EINVAL;

	gpio.state = slusb_inl(info, IOVR);
	gpio.smask = 0xffffffff;
	gpio.dir   = slusb_inl(info, IODR);
	gpio.dmask = 0xffffffff;

	if (copy_to_user(user_gpio, &gpio, sizeof(gpio)))
		return -EFAULT;
	DBGINFO(("%s get_gpio state=%08x dir=%08x\n",
		 slusb_dev_name(info), gpio.state, gpio.dir));
	return 0;
}

/*
 * conditional wait facility
 */
static void init_cond_wait(struct cond_wait *w, unsigned int data)
{
	init_waitqueue_head(&w->q);
	init_waitqueue_entry(&w->wait, current);
	w->data = data;
}

static void add_cond_wait(struct cond_wait **head, struct cond_wait *w)
{
	set_current_state(TASK_INTERRUPTIBLE);
	add_wait_queue(&w->q, &w->wait);
	w->next = *head;
	*head = w;
}

static void remove_cond_wait(struct cond_wait **head, struct cond_wait *cw)
{
	struct cond_wait *w, *prev;
	remove_wait_queue(&cw->q, &cw->wait);
	set_current_state(TASK_RUNNING);
	for (w = *head, prev = NULL ; w != NULL ; prev = w, w = w->next) {
		if (w == cw) {
			if (prev != NULL)
				prev->next = w->next;
			else
				*head = w->next;
			break;
		}
	}
}

static void flush_cond_wait(struct cond_wait **head)
{
	while (*head != NULL) {
		wake_up_interruptible(&(*head)->q);
		*head = (*head)->next;
	}
}

/*
 * wait for general purpose I/O pin(s) to enter specified state
 *
 * user_gpio fields:
 * state - bit indicates target pin state
 * smask - set bit indicates watched pin
 *
 * The wait ends when at least one watched pin enters the specified
 * state. When 0 (no error) is returned, user_gpio->state is set to the
 * state of all GPIO pins when the wait ends.
 *
 * Note: Each pin may be a dedicated input, dedicated output, or
 * configurable input/output. The number and configuration of pins
 * varies with the specific adapter model. Only input pins (dedicated
 * or configured) can be monitored with this function.
 */
static int slusb_wait_gpio(struct slusb_info *info, struct gpio_desc __user *user_gpio)
{
	unsigned long flags;
	int rc = 0;
	struct gpio_desc gpio;
	struct cond_wait wait;
	u32 state;

	if (!info->gpio_present)
		return -EINVAL;
	if (copy_from_user(&gpio, user_gpio, sizeof(gpio)))
		return -EFAULT;
	DBGINFO(("%s wait_gpio() state=%08x smask=%08x\n",
		 slusb_dev_name(info), gpio.state, gpio.smask));
	/* ignore output pins identified by set IODR bit */
	if ((gpio.smask &= ~slusb_outl_saved(info, IODR)) == 0)
		return -EINVAL;
	init_cond_wait(&wait, gpio.smask);

	/* enable interrupts for watched pins */
	spin_lock_irqsave(&info->lock, flags);
	slusb_outl(info, IOER, slusb_outl_saved(info, IOER) | gpio.smask);
	spin_unlock_irqrestore(&info->lock, flags);

	/* read current pin states */
	state = slusb_inl(info, IOVR);

	spin_lock_irqsave(&info->lock, flags);

	/* update states with lock to catch post read gpio status events */
	state = le32_to_cpu(*((unsigned int *)(&info->reg_last_read[IOVR])));

	if (gpio.smask & ~(state ^ gpio.state)) {
		/* already in target state */
		gpio.state = state;
	} else {
		/* wait for target state */
		add_cond_wait(&info->gpio_wait_q, &wait);
		spin_unlock_irqrestore(&info->lock, flags);
		schedule();
		if (signal_pending(current))
			rc = -ERESTARTSYS;
		else
			gpio.state = wait.data;
		spin_lock_irqsave(&info->lock, flags);
		remove_cond_wait(&info->gpio_wait_q, &wait);
	}

	/* disable all GPIO interrupts if no waiting processes */
	if (info->gpio_wait_q == NULL)
		slusb_outl(info, IOER, 0);
	spin_unlock_irqrestore(&info->lock, flags);

	if ((rc == 0) && copy_to_user(user_gpio, &gpio, sizeof(gpio)))
		rc = -EFAULT;
	DBGINFO(("%s wait_gpio() rc=%d\n", slusb_dev_name(info), rc));
	return rc;
}

static void slusb_set_gpio_bit(struct slusb_info *info, int bit, int val)
{
	unsigned int data;
	unsigned long flags;

	spin_lock_irqsave(&info->lock, flags);
	data = slusb_outl_saved(info, IOVR);
	data |= val << bit;
	data &= ~(val << bit);
	slusb_outl(info, IOVR, data);
	spin_unlock_irqrestore(&info->lock, flags);
}

/*
 * program frequency synthesizer with data for specified output frequency
 */
void slusb_set_clock_word(struct slusb_info *info, unsigned int *data)
{
	int i;
	unsigned int dword_val = 0;

	slusb_set_clk(info, 0);

	/* write 132 bit clock program word */

	for (i = 0 ; i < 132 ; i++) {
		if (!(i % 32))
			dword_val = data[i/32];
		slusb_set_data(info, (dword_val & (1 << 31)) ? 1 : 0);
		slusb_set_clk(info, 1);
		slusb_set_clk(info, 0);
		dword_val <<= 1;
	}

	/* clock word loaded on low to high select */
	slusb_set_select(info, 1);
	slusb_set_select(info, 0);
}

/*
 * Service an IOCTL request
 *
 * Arguments
 *
 *	tty	pointer to tty instance data
 *	file	pointer to associated file object for device
 *	cmd	IOCTL command code
 *	arg	command argument/context
 *
 * Return 0 if success, otherwise error code
 */
#if LINUX_VERSION_CODE < VERSION(2,6,39)
static int slusb_ioctl(struct tty_struct *tty, struct file *file,
		 unsigned int cmd, unsigned long arg)
#else
static int slusb_ioctl(struct tty_struct *tty, unsigned int cmd, unsigned long arg)
#endif
{
	struct usb_serial_port *port = tty->driver_data;
	struct slusb_info      *info = usb_get_serial_port_data(port);
	struct mgsl_icount cnow;	/* kernel counter temps */
	struct serial_icounter_struct __user *p_cuser;	/* user space */
	unsigned long flags;
	void __user *argp = (void __user *)arg;
	int ret;

	DBGINFO(("%s ioctl() cmd=%08X\n", slusb_dev_name(info), cmd));

	switch (cmd) {
	case MGSL_IOCGPARAMS:
		ret = slusb_get_params(info, argp);
		break;
	case MGSL_IOCSPARAMS:
		ret = slusb_set_params(info, argp);
		break;
	case MGSL_IOCGTXIDLE:
		ret = slusb_get_txidle(info, argp);
		break;
	case MGSL_IOCSTXIDLE:
		ret = slusb_set_txidle(info, (int)arg);
		break;
	case MGSL_IOCTXENABLE:
		ret = slusb_tx_enable(info, (int)arg);
		break;
	case MGSL_IOCRXENABLE:
		ret = slusb_rx_enable(info, (int)arg);
		break;
	case MGSL_IOCTXABORT:
		ret = slusb_tx_abort(info);
		break;
	case MGSL_IOCGSTATS:
		ret = slusb_get_stats(info, argp);
		break;
	case MGSL_IOCWAITEVENT:
		ret = slusb_wait_event(info, argp);
		break;
	case TIOCMIWAIT:
		ret = slusb_tiocmiwait(info, (int)arg);
		break;
	case MGSL_IOCGIF:
		ret = slusb_get_interface(info, argp);
		break;
	case MGSL_IOCSIF:
		ret = slusb_set_interface(info, (int)arg);
		break;
	case MGSL_IOCSGPIO:
		ret = slusb_set_gpio(info, argp);
		break;
	case MGSL_IOCGGPIO:
		ret = slusb_get_gpio(info, argp);
		break;
	case MGSL_IOCWAITGPIO:
		ret = slusb_wait_gpio(info, argp);
		break;
	case TIOCGICOUNT:
		spin_lock_irqsave(&info->lock, flags);
		cnow = info->icount;
		spin_unlock_irqrestore(&info->lock, flags);
		p_cuser = argp;
		if (put_user(cnow.cts, &p_cuser->cts) ||
		    put_user(cnow.dsr, &p_cuser->dsr) ||
		    put_user(cnow.rng, &p_cuser->rng) ||
		    put_user(cnow.dcd, &p_cuser->dcd) ||
		    put_user(cnow.rx, &p_cuser->rx) ||
		    put_user(cnow.tx, &p_cuser->tx) ||
		    put_user(cnow.frame, &p_cuser->frame) ||
		    put_user(cnow.overrun, &p_cuser->overrun) ||
		    put_user(cnow.parity, &p_cuser->parity) ||
		    put_user(cnow.brk, &p_cuser->brk) ||
		    put_user(cnow.buf_overrun, &p_cuser->buf_overrun))
			ret = -EFAULT;
		ret = 0;
		break;
	case MGSL_IOCGXSYNC:
		ret = slusb_get_xsync(info, argp);
		break;
	case MGSL_IOCSXSYNC:
		ret = slusb_set_xsync(info,(int)arg);
		break;
	case MGSL_IOCGXCTRL:
		ret = slusb_get_xctrl(info, argp);
		break;
	case MGSL_IOCSXCTRL:
		ret = slusb_set_xctrl(info,(int)arg);
		break;
	default:
		ret = -ENOIOCTLCMD;
	}
	return ret;
}

/*
 * set V.24 Control Register based on current configuration
 */
static void slusb_set_vcr(struct slusb_info *info)
{
	unsigned char val = 0;

	/* VCR (V.24 control)
	 *
	 * 7:5  reserved, must be zero
	 * 4    bit order: 0=LSB first 1=MSB first
	 * 3    DTR: 0=off 1=on
	 * 2    RTS: 0=off 1=on
	 * 1    LL:  0=off 1=on
	 * 0    RL:  0=off 1=on
	 */

	if (info->if_mode & MGSL_INTERFACE_MSB_FIRST)
		val |= BIT4;
	if (info->signals & SerialSignal_DTR)
		val |= BIT3;
	if (info->signals & SerialSignal_RTS)
		val |= BIT2;
	if (info->if_mode & MGSL_INTERFACE_LL)
		val |= BIT1;
	if (info->if_mode & MGSL_INTERFACE_RL)
		val |= BIT0;
	slusb_outb_queue(info, VCR, val);
}

/*
 * queue USB packet to set serial outputs
 */
static void slusb_set_signals_queue(struct slusb_info *info)
{
	unsigned char val = slusb_outb_saved(info, VCR);
	if (info->signals & SerialSignal_DTR)
		val |= BIT3;
	else
		val &= ~BIT3;
	if (info->signals & SerialSignal_RTS)
		val |= BIT2;
	else
		val &= ~BIT2;
	slusb_outb_queue(info, VCR, val);
}

/*
 * set state of serial outputs
 */
static void slusb_set_signals(struct slusb_info *info)
{
	slusb_set_signals_queue(info);
	slusb_outbuf_flush(info, NULL);
}

void slusb_irq_on_queue(struct slusb_info *info, unsigned short mask)
{
	unsigned short old_scr = slusb_outw_saved(info, SCR);
	unsigned short new_scr = old_scr | mask;

	if (new_scr != old_scr)
		slusb_outw_queue(info, SCR, new_scr);
}

void slusb_irq_off_queue(struct slusb_info *info, unsigned short mask)
{
	unsigned short old_scr = slusb_outw_saved(info, SCR);
	unsigned short new_scr = old_scr & ~mask;

	if (new_scr != old_scr)
		slusb_outw_queue(info, SCR, new_scr);
}

void slusb_irq_off(struct slusb_info *info, unsigned short mask)
{
	unsigned short old_scr = slusb_outw_saved(info, SCR);
	unsigned short new_scr = old_scr & ~mask;

	if (new_scr != old_scr)
		slusb_outw(info, SCR, new_scr);
}


/*
 * IN channel buffer overview
 *
 * - driver and device exchange packets (data/status/command/response)
 * - poll data is not aligned to packet boundaries
 * - inbuf stores poll data until complete packets are received
 * - complete packets are copied from inbuf to function specific buffers
 * - inbuf operations are synchronized with info->lock
 */

void slusb_inbuf_reset(struct slusb_info *info)
{
	info->inbuf_put_index = 0;
	info->inbuf_get_index = 0;
	info->inbuf_count = 0;
}

/*
 * get data from circular IN channel buffer
 * return number of bytes returned (0 or size)
 */
unsigned int slusb_inbuf_get(struct slusb_info *info, char *buf, unsigned int size)
{
	unsigned char *inbuf;
	unsigned int inbuf_size;
	unsigned int get_index;
	unsigned int tail_count;

	if (info->inbuf_count < size)
		return 0;

	inbuf      = info->inbuf;
	inbuf_size = sizeof(info->inbuf);
	get_index  = info->inbuf_get_index;

	/* calculate space from get index to end of buffer */
	tail_count = inbuf_size - get_index;

	if (tail_count < size) {
		/* wrap around two step copy */
		memcpy(buf, &inbuf[get_index], tail_count);
		memcpy(buf + tail_count, inbuf, size - tail_count);
	} else
		memcpy(buf, &inbuf[get_index], size);

	get_index += size;
	if (get_index >= inbuf_size)
		get_index -= inbuf_size;

	/* update buffer state */
	info->inbuf_get_index = get_index;
	info->inbuf_count -= size;

	return size;
}

/*
 * add data to circular IN channel buffer
 * return number of bytes added (0 or size)
 */
unsigned int slusb_inbuf_put(struct slusb_info *info, char *buf, unsigned int size)
{
	unsigned char *inbuf;
	unsigned int inbuf_size;
	unsigned int put_index;
	unsigned int tail_count;

	if ((sizeof(info->inbuf) - info->inbuf_count) < size) {
		DBGERR(("slusb_inbuf_put full, data discarded size=%d", size));
		return 0;
	}

	inbuf      = info->inbuf;
	inbuf_size = sizeof(info->inbuf);
	put_index  = info->inbuf_put_index;

//	TRACE_DATA(info, TraceLevel_Detail, "slusb_inbuf_put", buf, size);

	/* calculate space from put index to end of buffer */
	tail_count = inbuf_size - put_index;

	if (tail_count < size) {
		/* wrap around two step copy */
		memcpy(&inbuf[put_index], buf, tail_count);
		memcpy(inbuf, buf + tail_count, size - tail_count);
	} else
		memcpy(&inbuf[put_index], buf, size);

	put_index += size;
	if (put_index >= inbuf_size)
		put_index -= inbuf_size;

	/* update buffer state */
	info->inbuf_put_index = put_index;
	info->inbuf_count += size;

	return size;
}

/*
 * increment get pointer by count and decrement rbuf_count by count
 * return new get index
 */
unsigned int slusb_inbuf_get_inc(struct slusb_info *info, unsigned int count)
{
	info->inbuf_get_index += count;
	if (info->inbuf_get_index >= sizeof(info->inbuf))
		info->inbuf_get_index -= sizeof(info->inbuf);
	info->inbuf_count -= count;
	return info->inbuf_get_index;
}

static void slusb_process_gpio_status(struct slusb_info *info,
				      unsigned int changed, unsigned int state)
{
	struct cond_wait *w, *prev;

	/* wake processes waiting for specific transitions */
	for (w = info->gpio_wait_q, prev = NULL ; w != NULL ; w = w->next) {
		if (w->data & changed) {
			w->data = state;
			wake_up_interruptible(&w->q);
			if (prev != NULL)
				prev->next = w->next;
			else
				info->gpio_wait_q = w->next;
		} else
			prev = w;
	}

	/* acknowledge status indication */
	slusb_outl(info, IOSR, changed);
}

static void slusb_invalid_op(struct slusb_info *info)
{
	int count;
	int tail_count;
	int head_count;

	tail_count = sizeof(info->inbuf) - info->inbuf_get_index;
	count = info->inbuf_count;

	printk("%s invalid op, inbuf get=%d put=%d count=%d\n",
		slusb_dev_name(info), info->inbuf_get_index,
		info->inbuf_put_index, count);

	printk("%s invalid op, stat1=%02x%02x stat2=%02x%02x\n",
		slusb_dev_name(info), info->stat1[1], info->stat1[0],
		info->stat2[1], info->stat2[0]);

	trace_block(info, info->buf2, info->count2, "insert 2");
	trace_block(info, info->buf1, info->count1, "insert 1");

	head_count = info->inbuf_get_index;
	if (head_count > 128)
		head_count = 128;
	trace_block(info, &info->inbuf[info->inbuf_get_index - head_count],
		    head_count, "inbuf head");

	if (count > 256)
		count = 256;

	if (count > tail_count) {
		trace_block(info, &info->inbuf[info->inbuf_get_index],
			    tail_count, "inbuf");
		trace_block(info, info->inbuf,
			    count - tail_count, "inbuf");
	} else {
		trace_block(info, &info->inbuf[info->inbuf_get_index],
			    count, "inbuf");
	}

	info->poll_stop = true;
	slusb_inbuf_reset(info);
}

/*
 * process one complete packet (if available) from bulk IN pipe buffer
 * return number of bytes processed
 */
static unsigned int slusb_inbuf_process(struct slusb_info *info)
{
	unsigned char  temp[9]; /* max serial or GPIO status size = 9 */
	unsigned int   iosr;    /* gpio status */
	unsigned int   iovr;    /* gpio values */
	unsigned short ssr;     /* serial status */

	if (!info->inbuf_count)
		return 0;

	switch (info->inbuf[info->inbuf_get_index]) {
	case OP_READ_REG + OP_SIZE_BYTE:
	case OP_READ_REG + OP_SIZE_SHORT:
	case OP_READ_REG + OP_SIZE_WORD:
		return slusb_process_read_reg(info);

	case OP_SERIAL_STATUS:
		if (info->inbuf_count < 3)
			return 0; /* incomplete */
		slusb_inbuf_get(info, temp, 3);

		/* save last read value and signal update */
		memcpy(&info->reg_last_read[SSR], &temp[1], 2);
		info->reg_updated[SSR]   = true;
		info->reg_updated[SSR+1] = true;
		wake_up_interruptible(&info->reg_wait);

		memcpy(&ssr, &temp[1], 2);
		ssr = le16_to_cpu(ssr);
		slusb_process_serial_status(info, ssr);
		return 3;

	case OP_GPIO_STATUS:
		if (info->inbuf_count < 9)
			return 0; /* incomplete */
		slusb_inbuf_get(info, temp, 9);

		/* save last read values and signal update */
		memcpy(&info->reg_last_read[IOSR], &temp[1], sizeof(iosr));
		info->reg_updated[IOSR]   = true;
		info->reg_updated[IOSR+1] = true;
		info->reg_updated[IOSR+2] = true;
		info->reg_updated[IOSR+3] = true;
		memcpy(&info->reg_last_read[IOVR], &temp[5], sizeof(iovr));
		info->reg_updated[IOVR]   = true;
		info->reg_updated[IOVR+1] = true;
		info->reg_updated[IOVR+2] = true;
		info->reg_updated[IOVR+3] = true;
		wake_up_interruptible(&info->reg_wait);

		memcpy(&iosr, &temp[1], sizeof(iosr));
		memcpy(&iovr, &temp[5], sizeof(iovr));
		iosr = le32_to_cpu(iosr);
		iovr = le32_to_cpu(iovr);
		slusb_process_gpio_status(info, iosr, iovr);
		return 9;

	case OP_DATA:
		return slusb_process_rx_data(info);
	}

	slusb_invalid_op(info);
	return 0;
}

/*
 * receive data packet buffer (rbuf) overview
 *
 * the device transfers received serial data to the driver
 * in receive data packets via the input bulk pipe
 *
 * receive data packet format:
 * - one byte opcode
 * - one byte count (data + status)
 * - serial data (1 to 255 bytes)
 * - one byte status
 *
 * HDLC frames may span multiple receive data packets
 *
 * complete receive data packets are placed in the receive
 * data packet buffer with the opcode removed
 */

void slusb_rbuf_reset(struct slusb_info *info)
{
	info->rbuf_put_index = 0;
	info->rbuf_get_index = 0;
}

unsigned int slusb_rbuf_count(struct slusb_info *info)
{
	unsigned int rbuf_size = sizeof(info->rbuf);
	unsigned int put_index = info->rbuf_put_index;
	unsigned int get_index = info->rbuf_get_index;

	if (get_index > put_index)
		return rbuf_size - (get_index - put_index);
	else
		return put_index - get_index;
}

unsigned int slusb_rbuf_free_space(struct slusb_info *info)
{
	unsigned int rbuf_size = sizeof(info->rbuf);
	unsigned int put_index = info->rbuf_put_index;
	unsigned int get_index = info->rbuf_get_index;

	if (get_index > put_index)
		return get_index - put_index;
	else
		return rbuf_size - (put_index - get_index);
}

/*
 * add data to circular receive data packet buffer
 * return number of bytes added (0 or size)
 */
unsigned int slusb_rbuf_put(struct slusb_info *info, char *buf, unsigned int size)
{
	unsigned char *rbuf;
	unsigned int rbuf_size;
	unsigned int put_index;
	unsigned int tail_count;

	DBGPOLL(("%s %s size=%d\n", slusb_dev_name(info), __func__, size));

	if (slusb_rbuf_free_space(info) < size) {
		DBGERR(("%s %s full, data discarded size=%d",
			slusb_dev_name(info), __func__, size));
		return 0;
	}

	rbuf      = info->rbuf;
	rbuf_size = sizeof(info->rbuf);
	put_index = info->rbuf_put_index;

	/* calculate space from put index to end of buffer */
	tail_count = rbuf_size - put_index;

	if (tail_count < size) {
		/* wrap around two step copy */
		memcpy(&rbuf[put_index], buf, tail_count);
		memcpy(rbuf, buf + tail_count, size - tail_count);
	} else
		memcpy(&rbuf[put_index], buf, size);

	put_index += size;
	if (put_index >= rbuf_size)
		put_index -= rbuf_size;

	/* update buffer state */
	info->rbuf_put_index = put_index;

	return size;
}

/*
 * get data from circular receive buffer
 * return number of bytes returned (0 or size)
 */
unsigned int slusb_rbuf_get(struct slusb_info *info, char *buf, unsigned int size)
{
	unsigned char *rbuf;
	unsigned int rbuf_size;
	unsigned int get_index;
	unsigned int tail_count;

	if (slusb_rbuf_count(info) < size)
		return 0;

	rbuf      = info->rbuf;
	rbuf_size = sizeof(info->rbuf);
	get_index = info->rbuf_get_index;

	/* calculate space from get index to end of buffer */
	tail_count = rbuf_size - get_index;

	if (tail_count < size) {
		/* wrap around two step copy */
		memcpy(buf, &rbuf[get_index], tail_count);
		memcpy(buf + tail_count, rbuf, size - tail_count);
	} else
		memcpy(buf, &rbuf[get_index], size);

	get_index += size;
	if (get_index >= rbuf_size)
		get_index -= rbuf_size;

	/* update buffer state */
	info->rbuf_get_index = get_index;

	return size;
}

/*
 * pass receive data up to network or tty layer
 */
static void slusb_rx(struct slusb_info *info,
		     unsigned char *data, char *flags, int count)
{
	struct tty_struct *tty;
	struct tty_ldisc *ld;

	DBGDATA(info, data, count, "rx");

#if SYNCLINK_GENERIC_HDLC
	if (info->net_open) {
		hdlcdev_rx(info, data, count);
		return;
	}
#endif

	tty = tty_port_tty_get(&info->port->port);
	if (!tty) {
		dev_err(&info->port->dev, "%s tty is null\n", __func__);
		return;
	}

	ld = tty_ldisc_ref(tty);
	if (!ld) {
		dev_err(&info->port->dev, "%s ldisc is null\n", __func__);
		tty_kref_put(tty);
		return;
	}

#if LINUX_VERSION_CODE < VERSION(2,6,27)
	if (ld->receive_buf)
		ld->receive_buf(tty, data, flags, count);
#else
	if (ld->ops->receive_buf)
		ld->ops->receive_buf(tty, data, flags, count);
#endif

	tty_ldisc_deref(ld);
	tty_kref_put(tty);
}

/*
 * process one receive HDLC/SDLC frame if available
 * return true if frame processed, otherwise false
 */
static bool slusb_rx_frame(struct slusb_info *info)
{
	unsigned int  crc_size;
	unsigned char *buf;
	unsigned char *rbuf;
	unsigned int  get_index;
	unsigned int  rbuf_count;
	unsigned int  status_index;
	unsigned char packet_size;
	unsigned char packet_status;
	unsigned int  frame_size;
	unsigned char frame_addr;
	unsigned int  tail_count;
	unsigned int  data_size;
	unsigned int  copy_size;
	bool          rc = false;

	switch (info->params.crc_type & HDLC_CRC_MASK) {
	case HDLC_CRC_16_CCITT:
		crc_size = 2;
		break;
	case HDLC_CRC_32_CCITT:
		crc_size = 4;
		break;
	default:
		crc_size = 0;
	}

check_again:

	frame_size = 0;
	frame_addr = 0xff;

	rbuf_count = slusb_rbuf_count(info);
	if (!rbuf_count)
		goto cleanup;

	/*
	 * search for last packet of frame:
	 *
	 * - rbuf contains one or more packets
	 * - packet = 1 byte count (data+status), data, 1 byte status
	 * - a frame may span multiple packets
	 * - last packet of frame has BIT2 set in status
	 */
	rbuf      = info->rbuf;
	get_index = info->rbuf_get_index;

	/* frame address is first byte of first packet */
	if (get_index + 1 < sizeof(info->rbuf))
		frame_addr = rbuf[get_index + 1];
	else
		frame_addr = rbuf[0];

	for (;;) {
		/* get packet size (data + 1 byte status) */
		packet_size = rbuf[get_index];
		data_size   = packet_size - 1;
		frame_size += data_size;

		/* get packet status */
		status_index = get_index + packet_size;
		if (status_index >= sizeof(info->rbuf))
			status_index -= sizeof(info->rbuf);
		packet_status = rbuf[status_index];

		/* move past current packet */
		get_index += packet_size + 1;
		if (get_index >= sizeof(info->rbuf))
			get_index -= sizeof(info->rbuf);
		rbuf_count -= packet_size + 1;

		if (packet_status & BIT2)
			break; /* complete frame found*/
		if (!rbuf_count)
			goto cleanup; /* no complete frames available */
	}

	/* frame available, get_index and rbuf_count hold new values after frame */

	/*
	 * packet status format
	 *
	 * [7:6]  reserved
	 * [5:3]  residue
	 * [2]    eof (end of frame)
	 * [1]    CRC error
	 * [0]    abort
	 */

	/* ignore CRC bit if not using CRC (bit is undefined) */
	if ((info->params.crc_type & HDLC_CRC_MASK) == HDLC_CRC_NONE)
		packet_status &= ~BIT1;

	if ((frame_addr != 0xff) &&
	    (info->params.addr_filter != 0xff) &&
	    (frame_addr != info->params.addr_filter)) {
		DBGWORK(("%s address mismatch, discard rx frame\n",
		       slusb_dev_name(info)));
		info->rbuf_get_index = get_index;
		goto check_again;
	}

	if ((frame_size < (2 + crc_size)) || (packet_status & BIT0)) {
		DBGWORK(("%s discard short rx frame\n", slusb_dev_name(info)));
		info->icount.rxshort++;
		info->rbuf_get_index = get_index;
		goto check_again;
	}

	if (packet_status & BIT1) {
		DBGWORK(("%s rx CRC error\n", slusb_dev_name(info)));
		info->icount.rxcrc++;
		if (!(info->params.crc_type & HDLC_CRC_RETURN_EX)) {
			info->rbuf_get_index = get_index;
			goto check_again;
		}
	}

	DBGWORK(("%s rx frame status=%02X size=%d\n",
	       slusb_dev_name(info), packet_status, frame_size - crc_size));

	if (!frame_size) {
#if SYNCLINK_GENERIC_HDLC
		struct net_device_stats *stats = hdlc_stats(info->netdev);
		stats->rx_errors++;
		stats->rx_frame_errors++;
#endif
		/* discard frame */
		info->rbuf_get_index = get_index;
		goto check_again;
	}

	if (!(info->params.crc_type & HDLC_CRC_RETURN_EX)) {
		frame_size -= crc_size;
		crc_size = 0;
	}

	if (frame_size > info->max_frame_size + crc_size) {
		info->icount.rxlong++;
		/* frame to big, discard */
		info->rbuf_get_index = get_index;
		goto check_again;
	}

	/* copy frame data to contiguous buffer */
	buf = info->tmp_rbuf;
	copy_size = frame_size;

	get_index  = info->rbuf_get_index;

	do {
		/* get packet size (data + 1 status byte) */
		packet_size = rbuf[get_index];
		data_size   = packet_size - 1;

		/* get packet status */
		status_index = get_index + packet_size;
		if (status_index >= sizeof(info->rbuf))
			status_index -= sizeof(info->rbuf);
		packet_status = rbuf[status_index];

		/* adjust data_size for CRC if needed */
		if (copy_size < data_size)
			data_size = copy_size;
		copy_size -= data_size;

		/* move past packet size to data */
		get_index++;
		if (get_index >= sizeof(info->rbuf))
			get_index -= sizeof(info->rbuf);

		if (data_size) {
			/* copy packet data to buffer */
			tail_count = sizeof(info->rbuf) - get_index;
			if (tail_count < data_size) {
				memcpy(buf, &rbuf[get_index], tail_count);
				memcpy(&buf[tail_count], rbuf, data_size - tail_count);
			} else
				memcpy(buf, &rbuf[get_index], data_size);
			buf += data_size;
		}

		/* move past current packet */
		get_index += packet_size;
		if (get_index >= sizeof(info->rbuf))
			get_index -= sizeof(info->rbuf);

	} while (!(packet_status & BIT2));

	/* update buffer state */
	info->rbuf_get_index = get_index;

	rc = true;

	/* pass frame to network or tty layer */
	slusb_rx(info, info->tmp_rbuf, info->flag_buf, frame_size);

cleanup:
	return rc;
}

/*
 * process one complete receive buffer in
 * raw/bisync/monosync/xsync synchronous modes
 * return true if data processed, otherwise false
 */
bool slusb_rx_buf(struct slusb_info *info)
{
	unsigned char *buf;
	unsigned char *rbuf;
	unsigned int  get_index;
	unsigned int  rbuf_count;
	unsigned int  status_index;
	unsigned char packet_size;
	unsigned char packet_status;
	unsigned int  tail_count;
	unsigned int  data_size;
	bool          rc = false;

check_again:

	rbuf_count = slusb_rbuf_count(info);
	if (!rbuf_count)
		goto cleanup;

	/*
	 * - usb_rbuf contains one or more packets
	 * - packet = 1 byte count (data+status), data, 1 byte status
	 */
	rbuf       = info->rbuf;
	get_index  = info->rbuf_get_index;

	/* get packet size (data + 1 byte status) */
	packet_size = rbuf[get_index];
	data_size   = packet_size - 1;

	/*
	 * get packet status
	 *
	 * [7:6]  reserved
	 * [5:3]  residue
	 * [2]    eof (end of frame)
	 * [1]    CRC error
	 * [0]    abort
	 */
	status_index = get_index + packet_size;
	if (status_index >= sizeof(info->rbuf))
		status_index -= sizeof(info->rbuf);
	packet_status = rbuf[status_index];

	if ((info->params.mode != MGSL_MODE_RAW) &&
	    (packet_status & (BIT5+BIT4+BIT3))) {
		/* discard residue in byte synchronous modes */
		data_size--;
		if (!data_size) {
			/* no usable data, discard packet */
			info->rbuf_get_index = get_index;
			goto check_again;
		}
	}

	/* copy data to contiguous buffer */

	buf        = info->tmp_rbuf;
	get_index  = info->rbuf_get_index;

	/* move past packet size to data */
	get_index++;
	if (get_index >= sizeof(info->rbuf))
		get_index -= sizeof(info->rbuf);

	tail_count = sizeof(info->rbuf) - get_index;
	if (tail_count < data_size) {
		memcpy(buf, &rbuf[get_index], tail_count);
		memcpy(&buf[tail_count], rbuf, data_size - tail_count);
	} else
		memcpy(buf, &rbuf[get_index], data_size);

	/* move past current packet */
	get_index += packet_size;
	if (get_index >= sizeof(info->rbuf))
		get_index -= sizeof(info->rbuf);
	info->rbuf_get_index = get_index;

	rc = true;

	/* pass data to network or tty layer */
	slusb_rx(info, info->tmp_rbuf, info->flag_buf, data_size);

cleanup:
	return rc;
}

static void slusb_dsr_change(struct slusb_info *info)
{
	DBGPOLL(("%s dsr changed to %s\n", slusb_dev_name(info),
		info->signals & SerialSignal_DSR ? "ON" : "OFF"));

	if (info->signals & SerialSignal_DSR)
		info->input_signal_events.dsr_up++;
	else
		info->input_signal_events.dsr_down++;
	info->icount.dsr++;
	wake_up_interruptible(&info->status_event_wait_q);
	wake_up_interruptible(&info->event_wait_q);
}

static void slusb_cts_change(struct slusb_info *info)
{
	DBGPOLL(("%s cts changed to %s\n", slusb_dev_name(info),
		info->signals & SerialSignal_CTS ? "ON" : "OFF"));

	/* device specific stats and notification */
	if (info->signals & SerialSignal_CTS)
		info->input_signal_events.cts_up++;
	else
		info->input_signal_events.cts_down++;
	info->icount.cts++;
	wake_up_interruptible(&info->status_event_wait_q);
	wake_up_interruptible(&info->event_wait_q);

	/* tty notification */
	if (info->flags & ASYNC_CTS_FLOW) {
		struct tty_struct *tty = tty_port_tty_get(&info->port->port);
		if (tty) {
			if (tty->hw_stopped) {
				if (info->signals & SerialSignal_CTS) {
					tty->hw_stopped = 0;
					info->pending_work |= SLUSB_WORK_TX;
					DBGPOLL(("%s %s tx work pending\n",
						 slusb_dev_name(info), __func__));
					return;
				}
			} else {
				if (!(info->signals & SerialSignal_CTS))
					tty->hw_stopped = 1;
			}
			tty_kref_put(tty);
		}
	}
}

static void slusb_dcd_change(struct slusb_info *info)
{
	DBGPOLL(("%s dcd changed to %s\n", slusb_dev_name(info),
		info->signals & SerialSignal_DCD ? "ON" : "OFF"));

	/* device specific stats and notification */
	if (info->signals & SerialSignal_DCD)
		info->input_signal_events.dcd_up++;
	else
		info->input_signal_events.dcd_down++;
	info->icount.dcd++;
	wake_up_interruptible(&info->status_event_wait_q);
	wake_up_interruptible(&info->event_wait_q);

#if SYNCLINK_GENERIC_HDLC
	/* generic HDLC network notification */
	if (info->net_open) {
		if (info->signals & SerialSignal_DCD)
			netif_carrier_on(info->netdev);
		else
			netif_carrier_off(info->netdev);
		return;
	}
#endif
	/* tty notification */
	if (info->flags & ASYNC_CHECK_CD) {
		if (info->signals & SerialSignal_DCD) {
			wake_up_interruptible(&info->port->port.open_wait);
		} else {
			struct tty_struct *tty;
			tty = tty_port_tty_get(&info->port->port);
			if (tty) {
				tty_hangup(tty);
				tty_kref_put(tty);
			}
		}
	}
}

static void slusb_ri_change(struct slusb_info *info)
{
	DBGPOLL(("%s ri changed to %s\n", slusb_dev_name(info),
		info->signals & SerialSignal_RI ? "ON" : "OFF"));

	if (info->signals & SerialSignal_RI)
		info->input_signal_events.ri_up++;
	else
		info->input_signal_events.ri_down++;
	info->icount.rng++;
	wake_up_interruptible(&info->status_event_wait_q);
	wake_up_interruptible(&info->event_wait_q);
}

/*
 * process receive break indication in asynchronous mode
 */
static void slusb_rx_break(struct slusb_info *info)
{
	struct tty_struct *tty = tty_port_tty_get(&info->port->port);

	info->icount.brk++;

	/* process break if tty control allows */
	if (tty && !(RXBREAK & info->ignore_status_mask) &&
	    (info->read_status_mask & MASK_BREAK)) {
		slusb_insert_flip_char(tty, 0, TTY_BREAK);
		slusb_flip_buffer_push(tty);
		if (info->flags & ASYNC_SAK)
			do_SAK(tty);
	}

	if (tty)
		tty_kref_put(tty);

	usb_serial_handle_break(info->port);
}

/*
 * process SSR (Serial Status Register) read value
 *
 * Usually a serial status indication is generated by a change
 * in serial signal states, which is handled by usb_process_serial_status().
 *
 * On initial configuration, an SSR read is issued to determine the
 * initial state. The initial state does not generate a status
 * indication, so process states accordingly.
 */
static void slusb_process_ssr(struct slusb_info *info, unsigned short ssr)
{
	unsigned char changed;

	/* save current signal values */
	changed = info->signals;

	/* calculate new signal values based on SSR */

	info->signals &= SerialSignal_DTR + SerialSignal_RTS;

	if (ssr & BIT3)
		info->signals |= SerialSignal_DSR;
	if (ssr & BIT2)
		info->signals |= SerialSignal_CTS;
	if (ssr & BIT1)
		info->signals |= SerialSignal_DCD;
	if (ssr & BIT0)
		info->signals |= SerialSignal_RI;


	/* calculate changed values */
	changed ^= info->signals;

	/* report changes in serial inputs that did not trigger indication */
	if (changed & SerialSignal_DSR)
		slusb_dsr_change(info);
	if (changed & SerialSignal_CTS)
		slusb_cts_change(info);
	if (changed & SerialSignal_DCD)
		slusb_dcd_change(info);
	if (changed & SerialSignal_RI)
		slusb_ri_change(info);
}

/*
 * process transmit end of message/frame
 */
static void slusb_tx_eom(struct slusb_info *info, unsigned short status)
{
	struct tty_struct *tty;

	DBGPOLL(("%s txeom status=%04x\n", slusb_dev_name(info), status));

	slusb_irq_off_queue(info, IRQ_TXDATA + IRQ_TXIDLE + IRQ_TXUNDER);

	/* transmit underrun requires toggling tx reset bit */
	if (status & IRQ_TXUNDER) {
		unsigned short val = slusb_outw_saved(info, TCR);
		slusb_outw_queue(info, TCR, (unsigned short)(val | BIT2));
		slusb_outw_queue(info, TCR, val);
	}

	if (!info->tx_active)
		return;

	/* track synchronous mode statistics */
	if (info->params.mode != MGSL_MODE_ASYNC) {
		if (status & IRQ_TXUNDER)
			info->icount.txunder++;
		else if (status & IRQ_TXIDLE)
			info->icount.txok++;
	}

	info->tx_active = false;
	del_timer(&info->tx_timer);

	if ((info->params.mode != MGSL_MODE_ASYNC) &&
	    info->drop_rts_on_tx_done) {
		info->signals &= ~SerialSignal_RTS;
		info->drop_rts_on_tx_done = false;
		slusb_set_signals_queue(info);
	}

#if SYNCLINK_GENERIC_HDLC
	if (info->net_open) {
		hdlcdev_tx_done(info);
		return;
	}
#endif
	tty = tty_port_tty_get(&info->port->port);
	if (tty) {
		if (tty->stopped || tty->hw_stopped) {
			/* tx is stopped, do nothing */
			tty_kref_put(tty);
			return;
		}
		tty_kref_put(tty);
	}
	/* tx is not stopped, signal send completion */
	info->pending_work |= SLUSB_WORK_TX;
	DBGPOLL(("%s %s tx work pending\n", slusb_dev_name(info), __func__));
}

/*
 * process received asynchronous data
 * buf contains interleaved data and status bytes
 */
void slusb_rx_async(struct slusb_info *info, unsigned char *buf, unsigned int size)
{
	unsigned int i;
	unsigned char data;
	unsigned char status;
	int tty_status;
	bool chars_inserted = false;
	struct tty_struct *tty = tty_port_tty_get(&info->port->port);

	for (i=0 ; i < size ; i += 2) {
		data   = buf[i];
		status = buf[i+1];
		if (status & BIT0) {
			info->icount.frame++;
			tty_status = TTY_FRAME;
		} else if (status & BIT1) {
			info->icount.parity++;
			tty_status = TTY_PARITY;
		} else
			tty_status = 0;
		if (tty && !(status & info->ignore_status_mask)) {
			slusb_insert_flip_char(tty, data, tty_status);
			chars_inserted = true;
		}
	}

	if (tty) {
		if (chars_inserted)
			slusb_flip_buffer_push(tty);
		tty_kref_put(tty);
	}
}

/*
 * process received data in PIO mode
 *
 * add data to rx_pio_buf until fill level reached or EOF status set,
 * then copy accumulated data and final status to rbuf
 */
void slusb_rx_pio(struct slusb_info *info, unsigned char data, unsigned char status)
{
	unsigned int rbuf_packet_size;

	info->rx_pio_buf[info->rx_pio_count++] = data;
	DBGPOLL(("slusb_rx_pio() data=%02X status=%02X count=%d\n",
		 data, status, info->rx_pio_count));

	if ((info->rx_pio_count < info->rbuf_fill_level) && !(status & BIT2))
		return;

	/* fill level reached or EOF */
	DBGPOLL(("PIO packet complete\n"));

	/*
	 * create rbuf packet in temp_buf
	 *
	 * [0]     count of data and status
	 * [1..n]  n bytes of data
	 * [n+1]   final status
	 */
	rbuf_packet_size  = info->rx_pio_count + 2;
	info->temp_buf[0] = info->rx_pio_count + 1;
	memcpy(&info->temp_buf[1], info->rx_pio_buf, info->rx_pio_count);
	info->temp_buf[info->rx_pio_count + 1] = status;

	/* add rbuf packet to queue if space available */
	if (slusb_rbuf_free_space(info) < rbuf_packet_size) {
		/* insufficient space in receive data buffer */
		DBGERR(("%s rx packet count=%d, rbuf full\n",
			slusb_dev_name(info), rbuf_packet_size));
		slusb_rx_stop_queue(info);
		info->icount.rxover++;
		info->rx_restart = TRUE;
	} else {
		slusb_rbuf_put(info, info->temp_buf, rbuf_packet_size);
	}

	/* schedule processing of receive data */
	if (!info->test_mode) {
		info->pending_work |= SLUSB_WORK_RX;
		DBGPOLL(("%s rx packet count=%d rx work pending\n",
			 slusb_dev_name(info), rbuf_packet_size));
	}

	/* clear PIO buffer */
	info->rx_pio_count = 0;
}

/*
 * process receive data packet from device
 * return number of bytes processed
 */
static unsigned int slusb_process_rx_data(struct slusb_info *info)
{
	unsigned int op_size;
	unsigned int size_index;
	unsigned int packet_size;

	if (info->inbuf_count < 4) {
		/* opcode + packet size + data + status = 4 byte min */
		return 0;
	}

	/* packet size is byte following op code */
	size_index = info->inbuf_get_index + 1;
	if (size_index >= sizeof(info->inbuf))
		size_index -= sizeof(info->inbuf);
	packet_size = info->inbuf[size_index];
	if (packet_size == 0)
		packet_size = 256;

	/* operation size = op + size + data */
	op_size = 2 + packet_size;

	if (info->inbuf_count < op_size) {
		/* receive data packet incomplete */
		return 0;
	}

	if (info->params.mode == MGSL_MODE_ASYNC) {
		/* move past op + size to data */
		slusb_inbuf_get_inc(info, 2);
		/* copy data to temporary buffer */
		slusb_inbuf_get(info, info->temp_buf, packet_size);
		slusb_rx_async(info, info->temp_buf, packet_size);
	} else {
		/* move past op to size + data */
		slusb_inbuf_get_inc(info, 1);
		/* copy size + data to temporary buffer */
		slusb_inbuf_get(info, info->temp_buf, packet_size + 1);
		
		if ((slusb_rbuf_free_space(info) < packet_size + 1) || info->rx_restart) {
			/* insufficient space in receive data buffer */
			DBGERR(("%s rx packet count=%d, rbuf full\n",
				 slusb_dev_name(info), packet_size + 1));
			if (!info->rx_restart) {
				slusb_rx_stop_queue(info);
				info->icount.rxover++;
				info->rx_restart = TRUE;
			}
		} else {
			/* copy temporary buffer to receive data buffer */
			slusb_rbuf_put(info, info->temp_buf, packet_size + 1);
		}
		if (!info->test_mode) {
			info->pending_work |= SLUSB_WORK_RX;
			DBGPOLL(("%s rx packet count=%d rx work pending\n",
				 slusb_dev_name(info), packet_size + 1));
		}
	}

	return op_size;
}

/*
 * send data from tbuf to serial controller
 */
void slusb_tx_data(struct slusb_info *info)
{
	struct slusb_tbuf *tbuf;
	unsigned int packet_size;
	unsigned int data_size;
	unsigned int fifo_space = 4096;
	unsigned int buf_space = sizeof(info->outbuf) - info->outbuf_count;
	bool allow_eom;

	tbuf = info->tbuf_head;

	/* send data until FIFO full or all sent */

	while (fifo_space && tbuf) {

		if (buf_space < 3) {
			/* send pending data to clear outbuf space */
			slusb_outbuf_flush(info, NULL);
			buf_space = sizeof(info->outbuf);
		}

		/* max packet payload size = 256 bytes (8 bit count field) */
		if (tbuf->count < 256)
			data_size = tbuf->count;
		else
			data_size = 256;

		/* limit data to fit in hardware FIFO */
		if (data_size > fifo_space)
			data_size = fifo_space;

		/* limit data to fit in outbuf */
		if (data_size > buf_space - 2)
			data_size = buf_space - 2;

		/* packet = opcode + count field + data */
		packet_size = 2 + data_size;

		if ((info->params.mode == MGSL_MODE_HDLC) ||
		    (info->params.mode == MGSL_MODE_ASYNC))
			allow_eom = true;
		else
			allow_eom = false;

		/* set opcode field */
		if (!allow_eom  || (tbuf->count - data_size)) {
			/* first or intermediate packet of frame */
			info->outbuf[info->outbuf_count++] = OP_DATA;
		} else {
			/* last packet of frame, set EOM */
			info->outbuf[info->outbuf_count++] = OP_DATA + OP_EOM;
		}

		/* set count field */
		info->outbuf[info->outbuf_count++] = (unsigned char)data_size;

		/* copy data to out buffer */
		memcpy(&(info->outbuf[info->outbuf_count]),
		       &tbuf->buf[info->tbuf_index], data_size);
		info->outbuf_count += data_size;

		/* update transmit data buffer state */
		info->tbuf_index += data_size;
		tbuf->count -= data_size;

		if (!tbuf->count) {
			/* buffer complete, free it and get next */
			info->tbuf_head = tbuf->next;
			if (!info->tbuf_head)
				info->tbuf_tail = NULL;
			kfree(tbuf);
			info->tbuf_count--;
			tbuf = info->tbuf_head;
			info->tbuf_index = 0;
		}

		fifo_space -= data_size;
		buf_space -= packet_size;
	}

	if (info->outbuf_count) {
		/* send pending data */
		slusb_outbuf_flush(info, NULL);
	}
}

/*
 * process serial status packet
 */
void slusb_process_serial_status(struct slusb_info *info, unsigned short status)
{
	bool tx_data_irq;

	DBGINFO(("%s serial status=%04x\n", slusb_dev_name(info), status));


	if ((slusb_outw_saved(info, SCR) & IRQ_TXDATA) &&
	    (status & IRQ_TXDATA))
		tx_data_irq = true;
	else
		tx_data_irq = false;

	if (status & IRQ_TXUNDER) {
		slusb_tx_eom(info, status);
	} else if ((status & IRQ_TXIDLE) && !info->tbuf_count) {
		/* transmit is idle, no more data to send */
		slusb_tx_eom(info, status);
	} else if (tx_data_irq) {
		/* device is ready for more data */
		if (info->tbuf_head) {
			DBGPOLL(("%s load tx data\n",
				 slusb_dev_name(info)));
			slusb_tx_data(info);
		}
		if (!info->tbuf_head) {
			slusb_irq_off_queue(info, IRQ_TXDATA);
			DBGPOLL(("%s no more data, disable tx IRQ\n",
				 slusb_dev_name(info)));
		}
		if (info->tbuf_count < SLUSB_MAX_TBUF_COUNT) {
			info->pending_work |= SLUSB_WORK_TX;
			DBGPOLL(("%s tx work pending\n",
				 slusb_dev_name(info)));
		}
	}

	if (status & IRQ_RXOVER) {
		info->icount.rxover++;
		slusb_rx_start(info);
	} else if (info->rx_pio && (status & IRQ_RXDATA)) {
		/* PIO mode, read next receive byte */
		slusb_inw_queue(info, RDR);
	}

	if (info->params.mode == MGSL_MODE_ASYNC) {
		/* asynchronous mode */
		if ((status & IRQ_RXBREAK) && (status & RXBREAK))
			slusb_rx_break(info);
	} else {
		/* synchronous mode (HDLC/raw/bisync/monosync) */
		if (status & IRQ_RXIDLE) {
			if (status & RXIDLE)
				info->icount.rxidle++;
			else
				info->icount.exithunt++;
			wake_up_interruptible(&info->event_wait_q);
		}
	}

	/* update serial input states */
	info->signals &= SerialSignal_DTR + SerialSignal_RTS;
	if (status & BIT3)
		info->signals |= SerialSignal_DSR;
	if (status & BIT2)
		info->signals |= SerialSignal_CTS;
	if (status & BIT1)
		info->signals |= SerialSignal_DCD;
	if (status & BIT0)
		info->signals |= SerialSignal_RI;

	/* report changes in serial inputs */
	if (status & IRQ_DSR)
		slusb_dsr_change(info);
	if (status & IRQ_CTS)
		slusb_cts_change(info);
	if (status & IRQ_DCD)
		slusb_dcd_change(info);
	if (status & IRQ_RI)
		slusb_ri_change(info);

	/* acknowledge status indication */
	slusb_outw(info, SSR, status);
}

/*
 * process read register response from inbuf
 * return number of bytes processed
 */
static unsigned int slusb_process_read_reg(struct slusb_info *info)
{
	unsigned int packet_size;
	unsigned int reg_size;
	unsigned char buf[6]; /* max read reg packet size = 6 bytes */
	unsigned char addr;

	switch(info->inbuf[info->inbuf_get_index] & OP_SIZE_MASK) {
	case OP_SIZE_BYTE:
		reg_size = 1;
		break;
	case OP_SIZE_SHORT:
		reg_size = 2;
		break;
	case OP_SIZE_WORD:
		reg_size = 4;
		break;
	default:
		/* illegal register size */
//		usb_invalid_op(info);
		return 0;
	}

	/* packet size = opcode + addr + data */
	packet_size = 2 + reg_size;

	if (info->inbuf_count < packet_size) {
		/* read register response packet incomplete */
		return 0;
	}

	slusb_inbuf_get(info, buf, packet_size);

	/* perform register specific processing */

	if (info->rx_pio && (buf[1] == RDR)) {
		if (info->params.mode == MGSL_MODE_ASYNC)
			slusb_rx_async(info, &buf[2], 2);
		else
			slusb_rx_pio(info, buf[2], buf[3]);
	}

	if (buf[1] == SSR) {
		slusb_process_ssr(info, *((unsigned short *)&buf[2]));
	}

	/* save register read response information */

	addr = buf[1];

	switch(reg_size) {
	case 4:
		info->reg_last_read[addr + 3] = buf[5];
		info->reg_updated[addr + 3]   = true;
		info->reg_last_read[addr + 2] = buf[4];
		info->reg_updated[addr + 2]   = true;
		/* fall through */
	case 2:
		info->reg_last_read[addr + 1] = buf[3];
		info->reg_updated[addr + 1]   = true;
	}
	info->reg_last_read[addr] = buf[2];
	info->reg_updated[addr] = true;

	/* signal read register values have been updated */
	wake_up_interruptible(&info->reg_wait);

	return packet_size;
}

/*
 * flush OUT channel buffer to device
 */
void slusb_outbuf_flush(struct slusb_info *info, void (*func)(struct slusb_info*))
{
	if (info->outbuf_count) {
		slusb_out_bulk(info, info->outbuf, info->outbuf_count, func);
		info->outbuf_count = 0;
	}
}

/*
 * add data to OUT channel buffer
 *
 * return TRUE if data added, otherwise FALSE
 *
 * notes:
 * - flush OUT channel buffer if space needed for new data
 * - all data added or no data added
 */
bool slusb_outbuf_put(struct slusb_info *info, unsigned char *buf, unsigned int size)
{
	if (size > sizeof(info->outbuf)) {
		printk(KERN_INFO "%s size=%d too big\n", __func__, size);
		return false;
	}

	/* flush register write buffer if not enough space for command */
	if ((sizeof(info->outbuf) - info->outbuf_count) < size) {
		printk(KERN_INFO "%s flushing to make room\n", __func__);
		slusb_outbuf_flush(info, NULL);
	}

	memcpy(&(info->outbuf[info->outbuf_count]), buf, size);
	info->outbuf_count += size;

	return true;
}

/*
 * reset OUT channel buffer, discarding contents
 */
void slusb_outbuf_reset(struct slusb_info *info)
{
	info->outbuf_count = 0;
}

/*
 * FPGA serial controller register access functions
 */

/*
 * used as wait_event() condition for signalling register read complete
 */
bool slusb_reg_read_complete(struct slusb_info *info, unsigned char addr, unsigned char size)
{
	switch(size) {
	case 4:
		if (info->reg_updated[addr]   && info->reg_updated[addr+1] &&
		    info->reg_updated[addr+2] && info->reg_updated[addr+3])
			return true;
		break;
	case 2:
		if (info->reg_updated[addr]   && info->reg_updated[addr+1])
			return true;
		break;
	default:
		if (info->reg_updated[addr])
			return true;
	}
	return false;
}

/*
 * wait for register to be updated with a response
 * return true for success, otherwise false
 */
bool slusb_read_wait(struct slusb_info *info, unsigned char addr, unsigned char size)
{
	long rc;
	if (!info->poll_active)
		return false;
	rc = wait_event_interruptible_timeout(info->reg_wait,
					      slusb_reg_read_complete(info, addr, size), 1*HZ);
	if (rc)
		return true;
	return false;
}

/*
 * calculate register address based on base address and port index
 * base 0x00 to 0x3F = global (no port index)
 * base 0x40 to 0x7F = per port, 16 bytes per port
 * base 0x80 to 0xFF = per port, 32 bytes per port
 * used by register access functions below
 */
#define CALC_REGADDR() \
	if (addr >= 0x80) \
		addr += (info->port_number) * 32; \
	else if (addr >= 0x40)	\
		addr += (info->port_number) * 16;

#if 0
static void slusb_inb_queue(struct slusb_info *info, u8 addr)
{
	unsigned char cmd[2];
	CALC_REGADDR();
	cmd[0] = OP_READ_REG + OP_SIZE_BYTE;
	cmd[1] = addr;
	info->reg_updated[addr] = 0;
	slusb_outbuf_put(info, cmd, sizeof(cmd));
}
#endif

#if 0
static u8 slusb_inb(struct slusb_info *info, u8 addr)
{
	unsigned long flags;
	spin_lock_irqsave(&info->lock, flags);
	slusb_inb_queue(info, addr);
	slusb_outbuf_flush(info, NULL);
	spin_unlock_irqrestore(&info->lock, flags);
	if (slusb_read_wait(info, addr, 1))
		return info->reg_last_read[addr];
	return 0;
}
#endif

static u8 slusb_outb_saved(struct slusb_info *info, u8 addr)
{
	CALC_REGADDR();
	return info->reg_last_write[addr];
}

void slusb_outb_queue(struct slusb_info *info, u8 addr, u8 data)
{
	unsigned char cmd[3];
	CALC_REGADDR();
	cmd[0] = OP_WRITE_REG + OP_SIZE_BYTE;
	cmd[1] = addr;
	cmd[2] = data;
	info->reg_last_write[addr] = data;
	slusb_outbuf_put(info, cmd, sizeof(cmd));
}

#if 0
static void slusb_outb(struct slusb_info *info, u8 addr, u8 data)
{
	slusb_outb_queue(info, addr, data);
	slusb_outbuf_flush(info, NULL);
}
#endif

static void slusb_inw_queue(struct slusb_info *info, u8 addr)
{
	unsigned char cmd[2];
	CALC_REGADDR();
	cmd[0] = OP_READ_REG + OP_SIZE_SHORT;
	cmd[1] = addr;
	memset(&info->reg_updated[addr], 0, sizeof(u16));
	slusb_outbuf_put(info, cmd, sizeof(cmd));
}

static u16 slusb_inw(struct slusb_info *info, u8 addr)
{
	unsigned long flags;
	u16 data;
	spin_lock_irqsave(&info->lock, flags);
	slusb_inw_queue(info, addr);
	slusb_outbuf_flush(info, NULL);
	spin_unlock_irqrestore(&info->lock, flags);
	if (!slusb_read_wait(info, addr, 2))
		return 0;
	memcpy(&data, &info->reg_last_read[addr], sizeof(data));
	data = le16_to_cpu(data);
	return data;
}

u16 slusb_outw_saved(struct slusb_info *info, u8 addr)
{
	u16 data;
	CALC_REGADDR();
	memcpy(&data, &info->reg_last_write[addr], sizeof(data));
	data = le16_to_cpu(data);
	return data;
}

static void slusb_outw_queue(struct slusb_info *info, u8 addr, u16 data)
{
	unsigned char cmd[4];
	CALC_REGADDR();
	data = cpu_to_le16(data);
	cmd[0] = OP_WRITE_REG + OP_SIZE_SHORT;
	cmd[1] = addr;
	memcpy(&cmd[2], &data, sizeof(data));
	memcpy(&info->reg_last_write[addr], &data, sizeof(data));
	slusb_outbuf_put(info, cmd, sizeof(cmd));
}

static void slusb_outw(struct slusb_info *info, u8 addr, u16 data)
{
	slusb_outw_queue(info, addr, data);
	slusb_outbuf_flush(info, NULL);
}

static void slusb_inl_queue(struct slusb_info *info, u8 addr)
{
	unsigned char cmd[2];
	CALC_REGADDR();
	cmd[0] = OP_READ_REG + OP_SIZE_WORD;
	cmd[1] = addr;
	memset(&info->reg_updated[addr], 0, sizeof(u32));
	slusb_outbuf_put(info, cmd, sizeof(cmd));
}

static u32 slusb_inl(struct slusb_info *info, u8 addr)
{
	unsigned long flags;
	u32 data;
	spin_lock_irqsave(&info->lock, flags);
	slusb_inl_queue(info, addr);
	slusb_outbuf_flush(info, NULL);
	spin_unlock_irqrestore(&info->lock, flags);
	if (!slusb_read_wait(info, addr, 4))
		return 0;
	memcpy(&data, &info->reg_last_read[addr], sizeof(data));
	data = le32_to_cpu(data);
	return data;
}

static u32 slusb_outl_saved(struct slusb_info *info, u8 addr)
{
	u32 data;
	CALC_REGADDR();
	memcpy(&data, &info->reg_last_write[addr], sizeof(data));
	data = le32_to_cpu(data);
	return data;
}

static void slusb_outl_queue(struct slusb_info *info, u8 addr, u32 data)
{
	u8 cmd[6];
	CALC_REGADDR();
	data = cpu_to_le32(data);
	cmd[0] = OP_WRITE_REG + OP_SIZE_WORD;
	cmd[1] = addr;
	memcpy(&cmd[2], &data, sizeof(data));
	memcpy(&info->reg_last_write[addr], &data, sizeof(data));
	slusb_outbuf_put(info, cmd, sizeof(cmd));
}

static void slusb_outl(struct slusb_info *info, u8 addr, u32 data)
{
	slusb_outl_queue(info, addr, data);
	slusb_outbuf_flush(info, NULL);
}

/*
 * test register access
 * return true on success otherwise false
 */
static bool slusb_test_register(struct slusb_info *info)
{
	static u16 patterns[] =
		{0x0000, 0xffff, 0xaaaa, 0x5555, 0x6969, 0x9696};
	static unsigned int count = sizeof(patterns)/sizeof(patterns[0]);
	unsigned int i;
	bool rc = true;

	slusb_poll_start(info);

	for (i=0 ; i < count ; i++) {
		slusb_outw(info, TIR, patterns[i]);
		slusb_outw(info, BDR, patterns[(i+1)%count]);
		if ((slusb_inw(info, TIR) != patterns[i]) ||
		    (slusb_inw(info, BDR) != patterns[(i+1)%count])) {
			rc = false;
			break;
		}
	}

	/* store feature bits */
	info->jcr_value = slusb_inl(info, JCR);
	DBGINFO(("%s %s JCR = %08X\n", slusb_dev_name(info),
		 __func__, info->jcr_value));
	if (info->jcr_value & BIT5)
		info->gpio_present = true;

	slusb_poll_stop(info);

	if (!rc) {
		dev_err(&info->port->dev, "%s %s fail\n",
			slusb_dev_name(info), __func__);
	}

	return rc;
}

/*
 * loopback small HDLC frame to test serial controller
 * return true on success otherwise false
 */
bool slusb_test_loopback(struct slusb_info *info)
{
	unsigned char buf[20];
	unsigned int  i;
	bool rc;
	unsigned long timeout;
	MGSL_PARAMS params;

	/* initialize transmit buffer */
	for (i=0; i < sizeof(buf) ; i++)
		buf[i] = (unsigned char)i;

	/* save current settings */
	memcpy(&params, &info->params, sizeof(params));

	info->params.mode = MGSL_MODE_HDLC;
	info->params.clock_speed = 921600;
	info->params.loopback = 1;
	info->test_mode = true;

	/* initialize hardware and send data */
	slusb_sync_mode(info);
	slusb_rx_start(info);
	slusb_tx(info, buf, sizeof(buf));

	/* wait for receive complete */
	for (timeout = 100; timeout; --timeout) {
		msleep_interruptible(10);
		if (slusb_rbuf_count(info))
			break;
	}

	/* rbuf should contain byte count + data + 2 bytes CRC + byte status */
	if (slusb_rbuf_count(info) == (sizeof(buf) + 4) &&
	    !memcmp(buf, &info->rbuf[1], sizeof(buf))) {
		rc = true;
	} else {
		rc = false;
		dev_err(&info->port->dev, "%s %s fail\n",
			slusb_dev_name(info), __func__);
	}

	/* restore original settings */
	memcpy(&info->params, &params, sizeof(info->params));
	info->test_mode = false;

	slusb_tx_stop(info);
	slusb_rx_stop(info);
	slusb_poll_stop(info);

	return rc;
}

static void slusb_send_xchar(struct slusb_info *info, char ch)
{
	unsigned long flags;

	DBGINFO(("%s %s %02x\n", slusb_dev_name(info), __func__, ch));

	if (!ch)
		return;

	spin_lock_irqsave(&info->lock, flags);
	// TODO - allocate and queue tbuf
//	info->tbuf[info->tbuf_count++] = ch;
	if (!info->tx_enabled)
		slusb_tx_start(info);
	spin_unlock_irqrestore(&info->lock, flags);
}

/*
 * signal remote device to throttle send data (our receive data)
 */
static void slusb_throttle(struct tty_struct *tty)
{
	struct usb_serial_port *port = tty->driver_data;
	struct slusb_info *info = usb_get_serial_port_data(port);
	unsigned long flags;

	DBGINFO(("%s %s\n", slusb_dev_name(info), __func__));

	if (I_IXOFF(tty))
		slusb_send_xchar(info, STOP_CHAR(tty));
	if (tty_cflags(tty) & CRTSCTS) {
		spin_lock_irqsave(&info->lock,flags);
		info->signals &= ~SerialSignal_RTS;
		slusb_set_signals(info);
		spin_unlock_irqrestore(&info->lock,flags);
	}
}

/*
 * signal remote device to stop throttling send data (our receive data)
 */
static void slusb_unthrottle(struct tty_struct *tty)
{
	struct usb_serial_port *port = tty->driver_data;
	struct slusb_info *info = usb_get_serial_port_data(port);
	unsigned long flags;

	DBGINFO(("%s %s\n", slusb_dev_name(info), __func__));

	if (I_IXOFF(tty))
		slusb_send_xchar(info, START_CHAR(tty));
	if (tty_cflags(tty) & CRTSCTS) {
		spin_lock_irqsave(&info->lock, flags);
		info->signals |= SerialSignal_RTS;
		slusb_set_signals(info);
		spin_unlock_irqrestore(&info->lock, flags);
	}
}

/*
 * start serial transmitter
 */
static void slusb_tx_start(struct slusb_info *info)
{
	if (info->tx_active) {
		/* transmitter is already enabled and sending data */
		if (info->tbuf_count &&
		    !(slusb_outw_saved(info, SCR) & IRQ_TXDATA)) {
			/* enable tx data indications to process new data */
			slusb_irq_on_queue(info, IRQ_TXDATA);
			slusb_outbuf_flush(info, NULL);
		}
		return;
	}

	if (!info->tx_enabled) {
		unsigned short val;
		/* set enable bit and clear reset bit */
		val = (slusb_outw_saved(info, TCR) | BIT1) & ~BIT2;
		slusb_outw_queue(info, TCR, val);
		info->tx_enabled = true;
	}

	if (!info->tbuf_head) {
		slusb_outbuf_flush(info, NULL);
		return;
	}

	/*
	 * tbuf contains data to send:
	 * - assert RTS if needed
	 * - enable transmitter indications
	 * - data is sent to device when processing IRQ_TXDATA
	 */

	if ((info->params.flags & HDLC_FLAG_AUTO_RTS) &&
	    !(info->signals & SerialSignal_RTS)) {
		info->signals |= SerialSignal_RTS;
		slusb_set_signals_queue(info);
		info->drop_rts_on_tx_done = true;
	} else
		info->drop_rts_on_tx_done = false;

	if (info->params.mode == MGSL_MODE_ASYNC)
		slusb_irq_on_queue(info, IRQ_TXIDLE + IRQ_TXDATA);
	else
		slusb_irq_on_queue(info, IRQ_TXIDLE + IRQ_TXDATA + IRQ_TXUNDER);

	info->tx_active = true;
	slusb_outbuf_flush(info, NULL);
}

/*
 * discard buffer transmit data
 */
static void slusb_tx_flush(struct slusb_info *info)
{
	struct slusb_tbuf *tbuf;

	while (info->tbuf_head) {
		tbuf = info->tbuf_head;
		info->tbuf_head = tbuf->next;
		kfree(tbuf);
	}
	info->tbuf_tail = NULL;
	info->tbuf_index = 0;
	info->tbuf_count = 0;
}

/*
 * stop serial transmitter
 */
static void slusb_tx_stop_queue(struct slusb_info *info)
{
	unsigned short val;

	del_timer(&info->tx_timer);

	/* clear enable bit and set reset bit */
	val = (slusb_outw_saved(info, TCR) & ~BIT1) | BIT2;
	slusb_outw_queue(info, TCR, val);

	slusb_irq_off_queue(info, IRQ_TXDATA + IRQ_TXIDLE + IRQ_TXUNDER);

	/* clear tx idle and underrun status bit */
	slusb_outw_queue(info, SSR, (unsigned short)(IRQ_TXIDLE + IRQ_TXUNDER));

	info->tx_enabled = false;
	info->tx_active = false;

	slusb_tx_flush(info);
}

static void slusb_tx_stop(struct slusb_info *info)
{
	slusb_tx_stop_queue(info);
	slusb_outbuf_flush(info, NULL);
}

static void slusb_update_tx_timer(struct slusb_info *info)
{
	if (info->params.mode == MGSL_MODE_HDLC) {
		int timeout  = 5000;
		mod_timer(&info->tx_timer, jiffies + msecs_to_jiffies(timeout));
	}
}

/*
 * load data into transmit buffer and start transmitter if needed
 * return true if data accepted, otherwise false (buffers full)
 */
static bool slusb_tx(struct slusb_info *info,
		     const char *buf, unsigned int count)
{
	struct slusb_tbuf *tbuf;

	if (info->tbuf_count > SLUSB_MAX_TBUF_COUNT)
		return false;

	DBGDATA(info, buf, size, "tx");

	tbuf = kmalloc(sizeof(*tbuf) + count, GFP_ATOMIC);
	if (!tbuf)
		return false;
	memcpy(tbuf->buf, buf, count);
	tbuf->count = count;
	tbuf->next  = NULL;

	if (!info->tbuf_head) {
		info->tbuf_index = 0;
		info->tbuf_head = tbuf;
	} else {
		info->tbuf_tail->next = tbuf;
	}
	info->tbuf_tail = tbuf;
	info->tbuf_count++;

	slusb_tx_start(info);
	slusb_update_tx_timer(info);
	return true;
}

/*
 * transmit timeout handler
 */
static void slusb_tx_timeout(unsigned long context)
{
	struct slusb_info *info = (struct slusb_info*)context;
	unsigned long flags;

	DBGINFO(("%s tx_timeout\n", slusb_dev_name(info)));

	if (info->tx_active && info->params.mode == MGSL_MODE_HDLC)
		info->icount.txtimeout++;

	spin_lock_irqsave(&info->lock, flags);
	slusb_tx_stop(info);
	if (info->tty_open) {
		info->pending_work |= SLUSB_WORK_TX;
		DBGPOLL(("%s %s tx work pending\n",
			 slusb_dev_name(info), __func__));
		schedule_work(&info->poll_work);
	}
	spin_unlock_irqrestore(&info->lock, flags);

#if SYNCLINK_GENERIC_HDLC
	if (info->net_open)
		hdlcdev_tx_done(info);
#endif
}

/*
 * send all data in receive buffer directly to tty layer
 * this is used only when programming MSC PROM
 */
static void slusb_rx_prom(struct slusb_info *info)
{
	unsigned int count;

	while ((count = slusb_rbuf_count(info))) {
		if (count > info->max_frame_size)
			count = info->max_frame_size;
		slusb_rbuf_get(info, info->tmp_rbuf, count);
		slusb_rx(info, info->tmp_rbuf, info->flag_buf, count);
	}
}

/*
 * process work queued from interrupt context
 */
static void slusb_work(struct work_struct *work)
{
	struct slusb_info *info = container_of(work, struct slusb_info, poll_work);
	struct tty_struct *tty;
	unsigned long flags;

	spin_lock_irqsave(&info->lock, flags);
	info->work_running = true;

	for (;;) {
		if (info->pending_work & SLUSB_WORK_RX) {
			info->pending_work &= ~SLUSB_WORK_RX;
			DBGWORK(("%s %s rx\n", slusb_dev_name(info), __func__));
			spin_unlock_irqrestore(&info->lock, flags);
			if (info->params.mode == MGSL_MODE_MSC_PROM)
				slusb_rx_prom(info);
			else if (info->params.mode == MGSL_MODE_HDLC)
				while(slusb_rx_frame(info));
			else
				while(slusb_rx_buf(info));
			spin_lock_irqsave(&info->lock, flags);
			if (info->rx_restart)
				slusb_rx_start(info);
		} else if (info->pending_work & SLUSB_WORK_TX) {
			info->pending_work &= ~SLUSB_WORK_TX;
			DBGWORK(("%s %s tx\n", slusb_dev_name(info), __func__));
			tty = tty_port_tty_get(&info->port->port);
			if (tty) {
				spin_unlock_irqrestore(&info->lock, flags);
				tty_wakeup(tty);
				spin_lock_irqsave(&info->lock, flags);
				tty_kref_put(tty);
			}
		} else {
			info->work_running = false;
			spin_unlock_irqrestore(&info->lock, flags);
			return;
		}
	}
}

static void slusb_usb_prom_read(struct slusb_info *info, const unsigned char *buf,
				unsigned int count, struct tty_struct *tty)
{
	int rc;
	unsigned short addr;
	unsigned char rbuf[2];
	struct usb_device *dev = info->port->serial->dev;

	memcpy(&addr, &buf[1], 2);
	addr = cpu_to_le16(addr);

	DBGINFO(("%s addr=%02x", __func__, addr));
	rc = usb_control_msg(dev, usb_rcvctrlpipe(dev, 0), FTDI_READ_EEPROM,
			     USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			     0, addr, rbuf, 2, 5000);
	if (rc < 0) {
		dev_err(&info->port->dev, "%s error=%i\n", __func__, rc);
	} else {
		/* return PROM word value to read() */
		slusb_insert_flip_char(tty, rbuf[0], 0);
		slusb_insert_flip_char(tty, rbuf[1], 0);
		slusb_flip_buffer_push(tty);
	}
}

static void slusb_usb_prom_write(struct slusb_info *info,
				 const unsigned char *buf, unsigned int count)
{
	int rc;
	unsigned short addr;
	unsigned short value;
	struct usb_device *dev = info->port->serial->dev;

	memcpy(&addr, &buf[1], 2);
	addr = cpu_to_le16(addr);

	memcpy(&value, &buf[3], 2);
	value = cpu_to_le16(value);

	DBGINFO(("%s addr=%02x value=%02x", __func__, addr, value));
	rc = usb_control_msg(dev, usb_sndctrlpipe(dev, 0), FTDI_WRITE_EEPROM,
			     USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			     value, addr, NULL, 0, 5000);
	if (rc < 0)
		dev_err(&info->port->dev, "%s error=%i\n", __func__, rc);
}

static void slusb_usb_prom_erase(struct slusb_info *info)
{
	int rc;
	struct usb_device *dev = info->port->serial->dev;

	DBGINFO(("%s\n", __func__));
	rc = usb_control_msg(dev, usb_sndctrlpipe(dev, 0), FTDI_ERASE_EEPROM,
			     USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			     0, 0, NULL, 0, 5000);
	if (rc < 0)
		dev_err(&info->port->dev, "%s error=%i\n", __func__, rc);
}

/*
 * process write() for FTDI PROM mode
 *
 * this mode uses control endpoint 0 to send and receive data
 * containing FTDI PROM commands:
 *
 * FTDI_READ_EEPROM (read 16-bit word)
 *   byte[0]   = opcode
 *   byte[1:2] = 16-bit word address
 *   result is placed in the receive buffer for read()
 *
 * FTDI_WRITE_EEPROM (write 16-bit word)
 *   byte[0]   = opcode
 *   byte[1:2] = 16-bit word address
 *   byte[3:4] = 16-bit word value
 *
 * FTDI_ERASE_EEPROM
 *   byte[0]   = opcode
 */
static void slusb_usb_prom_cmd(struct slusb_info *info, const unsigned char *buf,
			       unsigned int count, struct tty_struct *tty)
{
	switch (buf[0]) {

	case FTDI_READ_EEPROM:
		if (info->tbuf_count == 3)
			slusb_usb_prom_read(info, buf, count, tty);
		break;

	case FTDI_WRITE_EEPROM:
		if (info->tbuf_count == 5)
			slusb_usb_prom_write(info, buf, count);
		break;

	case FTDI_ERASE_EEPROM:
		if (info->tbuf_count == 1)
			slusb_usb_prom_erase(info);
		break;
	}

	tty_wakeup(tty);
}

#if SYNCLINK_GENERIC_HDLC

/**
 * called by generic HDLC layer when protocol selected (PPP, frame relay, etc.)
 * set encoding and frame check sequence (FCS) options
 *
 * dev       pointer to network device structure
 * encoding  serial encoding setting
 * parity    FCS setting
 *
 * returns 0 if success, otherwise error code
 */
static int hdlcdev_attach(struct net_device *dev, unsigned short encoding,
			  unsigned short parity)
{
	struct slusb_info *info = dev_to_port(dev);
	unsigned char  new_encoding;
	unsigned short new_crctype;

	if (info->tty_open)
		return -EBUSY;

	DBGINFO(("%s hdlcdev_attach\n", slusb_dev_name(info)));

	switch (encoding) {
	case ENCODING_NRZ:
		new_encoding = HDLC_ENCODING_NRZ;
		break;
	case ENCODING_NRZI:
		new_encoding = HDLC_ENCODING_NRZI_SPACE;
		break;
	case ENCODING_FM_MARK:
		new_encoding = HDLC_ENCODING_BIPHASE_MARK;
		break;
	case ENCODING_FM_SPACE:
		new_encoding = HDLC_ENCODING_BIPHASE_SPACE;
		break;
	case ENCODING_MANCHESTER:
		new_encoding = HDLC_ENCODING_BIPHASE_LEVEL;
		break;
	default:
		return -EINVAL;
	}

	switch (parity) {
	case PARITY_NONE:
		new_crctype = HDLC_CRC_NONE;
		break;
	case PARITY_CRC16_PR1_CCITT:
		new_crctype = HDLC_CRC_16_CCITT;
		break;
	case PARITY_CRC32_PR1_CCITT:
		new_crctype = HDLC_CRC_32_CCITT;
		break;
	default:
		return -EINVAL;
	}

	info->params.encoding = new_encoding;
	info->params.crc_type = new_crctype;

	/* if network interface up, reprogram hardware */
	if (info->net_open)
		slusb_program_hw(info);

	return 0;
}

/**
 * called by generic HDLC layer to send frame
 *
 * skb  socket buffer containing HDLC frame
 * dev  pointer to network device structure
 *
 * returns 0 if success, otherwise error code
 */
static int hdlcdev_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct slusb_info *info = dev_to_port(dev);
	struct net_device_stats *stats = hdlc_stats(dev);
	unsigned long flags;

	DBGINFO(("%s hdlc_xmit\n", dev->name));

	if (!skb->len)
		return 0;

	/* stop sending until this frame completes */
	netif_stop_queue(dev);

	/* update network statistics */
	stats->tx_packets++;
	stats->tx_bytes += skb->len;

	/* save start time for transmit timeout detection */
	dev->trans_start = jiffies;

	spin_lock_irqsave(&info->lock, flags);
	slusb_tx(info, skb->data, skb->len);
	spin_unlock_irqrestore(&info->lock, flags);

	/* done with socket buffer, so free it */
	dev_kfree_skb(skb);

	return 0;
}

/**
 * called by network layer when interface enabled
 * claim resources and initialize hardware
 *
 * dev  pointer to network device structure
 *
 * returns 0 if success, otherwise error code
 */
static int hdlcdev_open(struct net_device *dev)
{
	struct slusb_info *info = dev_to_port(dev);
	int rc;
	unsigned long flags;

	if (!try_module_get(THIS_MODULE))
		return -EBUSY;

	DBGINFO(("%s hdlcdev_open\n", dev->name));

	/* generic HDLC layer open processing */
	if ((rc = hdlc_open(dev)))
		return rc;

	/* arbitrate between network and tty opens */
	spin_lock_irqsave(&info->netlock, flags);
	if (info->tty_open || info->net_open) {
		DBGINFO(("%s hdlc_open busy\n", dev->name));
		spin_unlock_irqrestore(&info->netlock, flags);
		return -EBUSY;
	}
	info->net_open = true;
	spin_unlock_irqrestore(&info->netlock, flags);

	/* assert DTR and RTS, apply hardware settings */
	info->signals |= SerialSignal_RTS + SerialSignal_DTR;
	slusb_program_hw(info);

	/* enable network layer transmit */
	dev->trans_start = jiffies;
	netif_start_queue(dev);

	/* inform generic HDLC layer of current DCD status */
#if LINUX_VERSION_CODE < VERSION(2,6,18)
	hdlc_set_carrier(info->signals & SerialSignal_DCD, dev);
#else
	if (info->signals & SerialSignal_DCD)
		netif_carrier_on(dev);
	else
		netif_carrier_off(dev);
#endif
	return 0;
}

/**
 * called by network layer when interface is disabled
 * shutdown hardware and release resources
 *
 * dev  pointer to network device structure
 *
 * returns 0 if success, otherwise error code
 */
static int hdlcdev_close(struct net_device *dev)
{
	struct slusb_info *info = dev_to_port(dev);
	unsigned long flags;

	DBGINFO(("%s %s\n", __func__, dev->name));
	netif_stop_queue(dev);
	slusb_shutdown(info);
	hdlc_close(dev);

	spin_lock_irqsave(&info->netlock, flags);
	info->net_open = false;
	spin_unlock_irqrestore(&info->netlock, flags);

	module_put(THIS_MODULE);
	return 0;
}

/**
 * called by network layer to process IOCTL call to network device
 *
 * dev  pointer to network device structure
 * ifr  pointer to network interface request structure
 * cmd  IOCTL command code
 *
 * returns 0 if success, otherwise error code
 */
static int hdlcdev_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{
	const size_t size = sizeof(sync_serial_settings);
	sync_serial_settings new_line;
	sync_serial_settings __user *line = ifr->ifr_settings.ifs_ifsu.sync;
	struct slusb_info *info = dev_to_port(dev);
	unsigned int flags;

	DBGINFO(("%s hdlcdev_ioctl\n", dev->name));

	/* return error if TTY interface open */
	if (info->tty_open)
		return -EBUSY;

	if (cmd != SIOCWANDEV)
		return hdlc_ioctl(dev, ifr, cmd);

	switch(ifr->ifr_settings.type) {
	case IF_GET_IFACE: /* return current sync_serial_settings */

		ifr->ifr_settings.type = IF_IFACE_SYNC_SERIAL;
		if (ifr->ifr_settings.size < size) {
			ifr->ifr_settings.size = size; /* data size wanted */
			return -ENOBUFS;
		}

		flags = info->params.flags & (HDLC_FLAG_RXC_RXCPIN | HDLC_FLAG_RXC_DPLL |
					      HDLC_FLAG_RXC_BRG    | HDLC_FLAG_RXC_TXCPIN |
					      HDLC_FLAG_TXC_TXCPIN | HDLC_FLAG_TXC_DPLL |
					      HDLC_FLAG_TXC_BRG    | HDLC_FLAG_TXC_RXCPIN);

		switch (flags){
		case (HDLC_FLAG_RXC_RXCPIN | HDLC_FLAG_TXC_TXCPIN):
			new_line.clock_type = CLOCK_EXT;
			break;
		case (HDLC_FLAG_RXC_BRG | HDLC_FLAG_TXC_BRG):
			new_line.clock_type = CLOCK_INT;
			break;
		case (HDLC_FLAG_RXC_RXCPIN | HDLC_FLAG_TXC_BRG):
			new_line.clock_type = CLOCK_TXINT;
			break;
		case (HDLC_FLAG_RXC_RXCPIN | HDLC_FLAG_TXC_RXCPIN):
			new_line.clock_type = CLOCK_TXFROMRX;
			break;
		default:
			new_line.clock_type = CLOCK_DEFAULT;
		}

		new_line.clock_rate = info->params.clock_speed;
		new_line.loopback   = info->params.loopback ? 1:0;

		if (copy_to_user(line, &new_line, size))
			return -EFAULT;
		return 0;

	case IF_IFACE_SYNC_SERIAL: /* set sync_serial_settings */

		if(!capable(CAP_NET_ADMIN))
			return -EPERM;
		if (copy_from_user(&new_line, line, size))
			return -EFAULT;

		switch (new_line.clock_type) {
		case CLOCK_EXT:
			flags = HDLC_FLAG_RXC_RXCPIN | HDLC_FLAG_TXC_TXCPIN;
			break;
		case CLOCK_TXFROMRX:
			flags = HDLC_FLAG_RXC_RXCPIN | HDLC_FLAG_TXC_RXCPIN;
			break;
		case CLOCK_INT:
			flags = HDLC_FLAG_RXC_BRG | HDLC_FLAG_TXC_BRG;
			break;
		case CLOCK_TXINT:
			flags = HDLC_FLAG_RXC_RXCPIN | HDLC_FLAG_TXC_BRG;
			break;
		case CLOCK_DEFAULT:
			flags = info->params.flags &
				(HDLC_FLAG_RXC_RXCPIN | HDLC_FLAG_RXC_DPLL |
				 HDLC_FLAG_RXC_BRG    | HDLC_FLAG_RXC_TXCPIN |
				 HDLC_FLAG_TXC_TXCPIN | HDLC_FLAG_TXC_DPLL |
				 HDLC_FLAG_TXC_BRG    | HDLC_FLAG_TXC_RXCPIN);
			break;
		default:
			return -EINVAL;
		}

		if (new_line.loopback != 0 && new_line.loopback != 1)
			return -EINVAL;

		info->params.flags &= ~(HDLC_FLAG_RXC_RXCPIN | HDLC_FLAG_RXC_DPLL |
					HDLC_FLAG_RXC_BRG    | HDLC_FLAG_RXC_TXCPIN |
					HDLC_FLAG_TXC_TXCPIN | HDLC_FLAG_TXC_DPLL |
					HDLC_FLAG_TXC_BRG    | HDLC_FLAG_TXC_RXCPIN);
		info->params.flags |= flags;

		info->params.loopback = new_line.loopback;

		if (flags & (HDLC_FLAG_RXC_BRG | HDLC_FLAG_TXC_BRG))
			info->params.clock_speed = new_line.clock_rate;
		else
			info->params.clock_speed = 0;

		/* if network interface up, reprogram hardware */
		if (info->net_open)
			slusb_program_hw(info);
		return 0;

	default:
		return hdlc_ioctl(dev, ifr, cmd);
	}
}

/**
 * called by network layer when transmit timeout is detected
 *
 * dev  pointer to network device structure
 */
static void hdlcdev_tx_timeout(struct net_device *dev)
{
	struct slusb_info *info = dev_to_port(dev);
	struct net_device_stats *stats = hdlc_stats(dev);
	unsigned long flags;

	DBGINFO(("%s hdlcdev_tx_timeout\n", dev->name));

	stats->tx_errors++;
	stats->tx_aborted_errors++;

	spin_lock_irqsave(&info->lock, flags);
	slusb_tx_stop(info);
	spin_unlock_irqrestore(&info->lock, flags);

	netif_wake_queue(dev);
}

/**
 * called by device driver when transmit completes
 * reenable network layer transmit if stopped
 *
 * info  pointer to device instance information
 */
static void hdlcdev_tx_done(struct slusb_info *info)
{
	if (netif_queue_stopped(info->netdev))
		netif_wake_queue(info->netdev);
}

/**
 * called by device driver when frame received
 * pass frame to network layer
 *
 * info  pointer to device instance information
 * buf   pointer to buffer contianing frame data
 * size  count of data bytes in buf
 */
static void hdlcdev_rx(struct slusb_info *info, char *buf, int size)
{
	struct sk_buff *skb = dev_alloc_skb(size);
	struct net_device *dev = info->netdev;
	struct net_device_stats *stats = hdlc_stats(dev);

	DBGINFO(("%s hdlcdev_rx\n", dev->name));

	if (skb == NULL) {
		DBGERR(("%s: can't alloc skb, drop packet\n", dev->name));
		stats->rx_dropped++;
		return;
	}

	memcpy(skb_put(skb, size),buf,size);

	skb->protocol = hdlc_type_trans(skb, info->netdev);

	stats->rx_packets++;
	stats->rx_bytes += size;

	netif_rx(skb);

	info->netdev->last_rx = jiffies;
}

#if LINUX_VERSION_CODE > VERSION(2,6,30)
static const struct net_device_ops hdlcdev_ops = {
	.ndo_open       = hdlcdev_open,
	.ndo_stop       = hdlcdev_close,
	.ndo_change_mtu = hdlc_change_mtu,
	.ndo_start_xmit = hdlc_start_xmit,
	.ndo_do_ioctl   = hdlcdev_ioctl,
	.ndo_tx_timeout = hdlcdev_tx_timeout,
};
#endif

/**
 * called by device driver when adding device instance
 * do generic HDLC initialization
 *
 * info  pointer to device instance information
 *
 * returns 0 if success, otherwise error code
 */
static int hdlcdev_init(struct slusb_info *info)
{
	int rc;
	struct net_device *dev;
	hdlc_device *hdlc;

	/* allocate and initialize network and HDLC layer objects */

	if (!(dev = alloc_hdlcdev(info))) {
		printk(KERN_ERR "%s hdlc device alloc failure\n", slusb_dev_name(info));
		return -ENOMEM;
	}

	/* network layer callbacks and settings */
#if LINUX_VERSION_CODE > VERSION(2,6,30)
	dev->netdev_ops	    = &hdlcdev_ops;
#else
	dev->do_ioctl       = hdlcdev_ioctl;
	dev->open           = hdlcdev_open;
	dev->stop           = hdlcdev_close;
	dev->tx_timeout     = hdlcdev_tx_timeout;
#endif
	dev->watchdog_timeo = 10*HZ;
	dev->tx_queue_len   = 50;

	/* generic HDLC layer callbacks and settings */
	hdlc         = dev_to_hdlc(dev);
	hdlc->attach = hdlcdev_attach;
	hdlc->xmit   = hdlcdev_xmit;

	/* register objects with HDLC layer */
	if ((rc = register_hdlc_device(dev))) {
		printk(KERN_WARNING "%s:unable to register hdlc device\n",__FILE__);
		free_netdev(dev);
		return rc;
	}

	info->netdev = dev;
	return 0;
}

/**
 * called by device driver when removing device instance
 * do generic HDLC cleanup
 *
 * info  pointer to device instance information
 */
static void hdlcdev_exit(struct slusb_info *info)
{
	unregister_hdlc_device(info->netdev);
	free_netdev(info->netdev);
	info->netdev = NULL;
}

#endif /* ifdef CONFIG_HDLC */

static int __init slusb_init(void)
{
	int rc;

	rc = usb_register(&slusb_service_usb_driver);
	if (rc) {
		printk(KERN_ERR"%s usb_register(service) error=%d\n",
		       slusb_driver_name, rc);
		return rc;
	}

#if LINUX_VERSION_CODE < VERSION(3,4,0)
	rc = usb_serial_register(&slusb_serial_driver);
	if (rc) {
		printk(KERN_ERR"%s usb_serial_register error=%d\n",
		       slusb_driver_name, rc);
		return rc;
	}
	rc = usb_register(&slusb_serial_usb_driver);
	if (rc) {
		printk(KERN_ERR"%s usb_register(serial) error=%d\n",
		       slusb_driver_name, rc);
		usb_serial_deregister(&slusb_serial_driver);
		return rc;
	}
#elif LINUX_VERSION_CODE < VERSION(3,5,0)
	rc = usb_serial_register_drivers(&slusb_serial_usb_driver,
					 slusb_serial_driver_table);
	if (rc) {
		printk(KERN_ERR"%s usb_serial_register_drivers error=%d\n",
		       slusb_driver_name, rc);
		return rc;
	}
#else
	rc = usb_serial_register_drivers(slusb_serial_driver_table,
					 KBUILD_MODNAME, slusb_serial_id_table);
	if (rc) {
		printk(KERN_ERR"%s usb_serial_register_drivers error=%d\n",
		       slusb_driver_name, rc);
		return rc;
	}
#endif

	printk(KERN_INFO"%s loaded\n", slusb_driver_name);
	return 0;
}

static void __exit slusb_exit(void)
{
	usb_deregister(&slusb_service_usb_driver);

#if LINUX_VERSION_CODE < VERSION(3,4,0)
	usb_deregister(&slusb_serial_usb_driver);
	usb_serial_deregister(&slusb_serial_driver);
#elif LINUX_VERSION_CODE < VERSION(3,5,0)
	usb_serial_deregister_drivers(&slusb_serial_usb_driver,
				      slusb_serial_driver_table);
#else
	usb_serial_deregister_drivers(slusb_serial_driver_table);
#endif

	printk(KERN_INFO"%s unloaded\n", slusb_driver_name);
}

module_init(slusb_init);
module_exit(slusb_exit);
