// SPDX-License-Identifier: GPL-2.0+
/*
*  LPC USB-10 ports Serial Converters
*
*  Copyright 2018 NXP
*  All rights reserved.
*
*  Shamelessly based on DIGI driver
*
*
*/

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>
#include <linux/usb.h>
#include <linux/wait.h>
#include <linux/sched/signal.h>
#include <linux/usb/serial.h>
#include <linux/types.h>

/* Defines */

#define DRIVER_AUTHOR "Hui Song <hui.song_1@nxp.com>"
#define DRIVER_DESC "NXP usb-10 Serial Converter driver"

/* when the number of the have send data  is greater than the macro below */
/* we should enquire the device whether can send data */
#define LPC_MAX_SEND_NUM        512

/* port output buffer length -- must be <= transfer buffer length - 3 */
/* so we can be sure to send the full buffer in one urb */
#define LPC_OUT_BUF_SIZE		128


/* retry timeout while sleeping */
#define LPC_RETRY_TIMEOUT		(HZ/10)

/* timeout while waiting for tty output to drain in close */
/* this delay is used twice in close, so the total delay could */
/* be twice this value */
#define LPC_CLOSE_TIMEOUT		(5*HZ)


/* LPC USB Defines */

/* ids */
#define LPC_VENDOR_ID			0x1FC9
#define LPC_J2_ID			0xEA60

/* commands
 * "INB": can be used on the in-band endpoint
 * "OOB": can be used on the out-of-band endpoint
 */
#define LPC_CMD_SET_BAUD_RATE			       0	/* INB, OOB */
#define LPC_CMD_SET_WORD_SIZE			       1	/* INB, OOB */
#define LPC_CMD_SET_PARITY			       2	/* INB, OOB */
#define LPC_CMD_SET_STOP_BITS			       3	/* INB, OOB */
#define LPC_CMD_SET_INPUT_FLOW_CONTROL		       4	/* INB, OOB */
#define LPC_CMD_SET_OUTPUT_FLOW_CONTROL	               5	/* INB, OOB */
#define LPC_CMD_SET_DTR_SIGNAL			       6	/* INB, OOB */
#define LPC_CMD_SET_RTS_SIGNAL			       7	/* INB, OOB */
#define LPC_CMD_READ_INPUT_SIGNALS		       8	/*      OOB */
#define LPC_CMD_IFLUSH_FIFO			       9	/*      OOB */
#define LPC_CMD_RECEIVE_ENABLE			       10	/* INB, OOB */
#define LPC_CMD_BREAK_CONTROL			       11	/* INB, OOB */
#define LPC_CMD_LOCAL_LOOPBACK			       12	/* INB, OOB */
#define LPC_CMD_TRANSMIT_IDLE			       13	/* INB, OOB */
#define LPC_CMD_READ_UART_REGISTER		       14	/*      OOB */
#define LPC_CMD_WRITE_UART_REGISTER		       15	/* INB, OOB */
#define LPC_CMD_AND_UART_REGISTER		       16	/* INB, OOB */
#define LPC_CMD_OR_UART_REGISTER		       17	/* INB, OOB */
#define LPC_CMD_SEND_DATA			       18	/* INB      */
#define LPC_CMD_RECEIVE_DATA			       19	/* INB      */
#define LPC_CMD_RECEIVE_DISABLE		               20	/* INB      */
#define LPC_CMD_GET_PORT_TYPE			       21	/*      OOB */
#define LPC_CMD_TRANSMIT_FULL                  	       22   /*      OOB */

/* baud rates */
#define LPC_BAUD_50				0
#define LPC_BAUD_75				1
#define LPC_BAUD_110				2
#define LPC_BAUD_150				3
#define LPC_BAUD_200				4
#define LPC_BAUD_300				5
#define LPC_BAUD_600				6
#define LPC_BAUD_1200				7
#define LPC_BAUD_1800				8
#define LPC_BAUD_2400				9
#define LPC_BAUD_4800				10
#define LPC_BAUD_7200				11
#define LPC_BAUD_9600				12
#define LPC_BAUD_14400				13
#define LPC_BAUD_19200				14
#define LPC_BAUD_28800				15
#define LPC_BAUD_38400				16
#define LPC_BAUD_57600				17
#define LPC_BAUD_76800				18
#define LPC_BAUD_115200			    	19
#define LPC_BAUD_153600			    	20
#define LPC_BAUD_230400			    	21
#define LPC_BAUD_460800			    	22
#define LPC_BAUD_921600			    	23
#define LPC_BAUD_1000000		    	24
#define LPC_BAUD_2000000		        25


/* arguments */
#define LPC_WORD_SIZE_5			0
#define LPC_WORD_SIZE_6			1
#define LPC_WORD_SIZE_7			2
#define LPC_WORD_SIZE_8			3

#define LPC_PARITY_NONE			0
#define LPC_PARITY_ODD			1
#define LPC_PARITY_EVEN			2
#define LPC_PARITY_MARK			3
#define LPC_PARITY_SPACE		4

#define LPC_STOP_BITS_1			0
#define LPC_STOP_BITS_2			1

#define LPC_INPUT_FLOW_CONTROL_XON_XOFF	    1
#define LPC_INPUT_FLOW_CONTROL_RTS	    2
#define LPC_INPUT_FLOW_CONTROL_DTR	    4

#define LPC_OUTPUT_FLOW_CONTROL_XON_XOFF       	1
#define LPC_OUTPUT_FLOW_CONTROL_CTS	        2
#define LPC_OUTPUT_FLOW_CONTROL_DSR	        4

#define LPC_DTR_INACTIVE			    0
#define LPC_DTR_ACTIVE				    1
#define LPC_DTR_INPUT_FLOW_CONTROL		2

#define LPC_RTS_INACTIVE			    0
#define LPC_RTS_ACTIVE				    1
#define LPC_RTS_INPUT_FLOW_CONTROL		2
#define LPC_RTS_TOGGLE				    3

#define LPC_FLUSH_TX				1
#define LPC_FLUSH_RX				2
#define LPC_RESUME_TX				4 /* clears xoff condition */

#define LPC_TRANSMIT_NOT_IDLE			0
#define LPC_TRANSMIT_IDLE			    1

#define LPC_DISABLE				0
#define LPC_ENABLE				1

#define LPC_DEASSERT				0
#define LPC_ASSERT				    1

/* in band status codes */
#define LPC_OVERRUN_ERROR			4
#define LPC_PARITY_ERROR			8
#define LPC_FRAMING_ERROR			16
#define LPC_BREAK_ERROR			    32

/* out of band status */
#define LPC_NO_ERROR				    0
#define LPC_BAD_FIRST_PARAMETER		    1
#define LPC_BAD_SECOND_PARAMETER		2
#define LPC_INVALID_LINE			    3
#define LPC_INVALID_OPCODE			    4

/* input signals */
#define LPC_READ_INPUT_SIGNALS_SLOT		1
#define LPC_READ_INPUT_SIGNALS_ERR		2
#define LPC_READ_INPUT_SIGNALS_BUSY		4
#define LPC_READ_INPUT_SIGNALS_PE		8
#define LPC_READ_INPUT_SIGNALS_CTS		16
#define LPC_READ_INPUT_SIGNALS_DSR		32
#define LPC_READ_INPUT_SIGNALS_RI		64
#define LPC_READ_INPUT_SIGNALS_DCD		128

/* usb serial port configuration information */
#define NUM_PORTS 			10
#define NUM_BULK_IN			11
#define NUM_BULK_OUT        11
#define REAL_NUM_BULK_IN    2
#define REAL_NUM_BULK_OUT   2
/* Structures */

struct lpc_serial {
    spinlock_t                 lpcs_serial_lock;
    struct usb_serial_port     *lpcs_oob_port;	        /* out-of-band port */
    int                        lpcs_oob_port_num;			/* index of out-of-band port */
    int                        lpcs_device_started;
};

struct lpc_port {
    spinlock_t                        lpcp_port_lock;
    bool                              is_loopback;
    int                               lpcp_port_num;
    int                               lpcp_out_buf_len;
    unsigned char                     lpcp_out_buf[LPC_OUT_BUF_SIZE];
    int                               lpcp_write_urb_in_use;
    unsigned int                      lpcp_modem_signals;
    int                               lpcp_transmit_idle;
    wait_queue_head_t                 lpcp_transmit_idle_wait;
    wait_queue_head_t                 lpcp_transmit_is_full;
    int                               lpcp_threshold_num;
    unsigned long long                lpcp_download_num;
    unsigned long long                lpcp_upload_num;
    int                               lpcp_throttled;
    int                               lpcp_throttle_restart;
    wait_queue_head_t                 lpcp_flush_wait;
    wait_queue_head_t                 lpcp_close_wait;	/* wait queue for close */
    struct work_struct                lpcp_wakeup_work;
    struct usb_serial_port            *lpcp_port;
};


/* Local Function Declarations */

static void lpc_wakeup_write_lock(struct work_struct *work);
static int  lpc_write_oob_command(struct usb_serial_port *port,unsigned char *buf, int count, int interruptible);
static int  lpc_write_inb_command(struct usb_serial_port *port,unsigned char *buf, int count, unsigned long timeout);
static int  lpc_set_modem_signals(struct usb_serial_port *port,unsigned int modem_signals, int interruptible);
static int  lpc_transmit_idle(struct usb_serial_port *port,unsigned long timeout);
static void lpc_rx_throttle(struct tty_struct *tty);
static void lpc_rx_unthrottle(struct tty_struct *tty);
static void lpc_set_termios(struct tty_struct *tty,struct usb_serial_port *port, struct ktermios *old_termios);
static void lpc_break_ctl(struct tty_struct *tty, int break_state);
static int  lpc_tiocmget(struct tty_struct *tty);
static int  lpc_tiocmset(struct tty_struct *tty, unsigned int set,unsigned int clear);
static int  lpc_write(struct tty_struct *tty, struct usb_serial_port *port,const unsigned char *buf, int count);
static void lpc_write_bulk_callback(struct urb *urb);
static int  lpc_write_room(struct tty_struct *tty);
static int  lpc_chars_in_buffer(struct tty_struct *tty);
static int  lpc_open(struct tty_struct *tty, struct usb_serial_port *port);
static void lpc_close(struct usb_serial_port *port);
static void lpc_dtr_rts(struct usb_serial_port *port, int on);
static int  lpc_startup_device(struct usb_serial *serial);
static int  lpc_startup(struct usb_serial *serial);
static void lpc_disconnect(struct usb_serial *serial);
static void lpc_release(struct usb_serial *serial);
static int  lpc_port_probe(struct usb_serial_port *port);
static int  lpc_port_remove(struct usb_serial_port *port);
static void lpc_read_bulk_callback(struct urb *urb);
static int  lpc_read_inb_callback(struct urb *urb);
static int  lpc_read_oob_callback(struct urb *urb);
static int  lpc_probe(struct usb_serial *serial,const struct usb_device_id *id);
static int  lpc_calc_num_ports(struct usb_serial *serial,struct usb_serial_endpoints *epds);


static const struct usb_device_id id_table_10[] = {
    { USB_DEVICE(LPC_VENDOR_ID, LPC_J2_ID) },
    { }						/* Terminating entry */
};
MODULE_DEVICE_TABLE(usb, id_table_10);


/* device info needed for the lpc serial converter */

static struct usb_serial_driver lpc_10_port_driver = {
    .driver = {
        .owner =		THIS_MODULE,
        .name =			"LPC54XXX",
    },
    .description =			"LPC54XXX 10 port USB adapter",
    .id_table =			    id_table_10,
    .num_ports =			NUM_PORTS,
    .num_bulk_in =			NUM_BULK_IN,
    .num_bulk_out =			NUM_BULK_OUT,
    .bulk_in_size  =        256,
    .bulk_out_size =        256,
    .probe =                lpc_probe,
    .calc_num_ports =       lpc_calc_num_ports,
    .open =				    lpc_open,
    .close =			    lpc_close,
    .dtr_rts =			    lpc_dtr_rts,
    .write =			    lpc_write,
    .write_room =			lpc_write_room,
    .write_bulk_callback = 	lpc_write_bulk_callback,
    .read_bulk_callback =	lpc_read_bulk_callback,
    .chars_in_buffer =		lpc_chars_in_buffer,
    .throttle =			    lpc_rx_throttle,
    .unthrottle =			lpc_rx_unthrottle,
    .set_termios =			lpc_set_termios,
    .break_ctl =			lpc_break_ctl,
    .tiocmget =			    lpc_tiocmget,
    .tiocmset =			    lpc_tiocmset,
    .attach =			    lpc_startup,
    .disconnect =			lpc_disconnect,
    .release =			    lpc_release,
    .port_probe =			lpc_port_probe,
    .port_remove =			lpc_port_remove,
};

static struct usb_serial_driver * const serial_drivers[] = {
    &lpc_10_port_driver, NULL
};

/* Functions */
static ssize_t loopback_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    bool val;
    unsigned long flags = 0;

    struct usb_serial_port *port = to_usb_serial_port(dev);
    struct lpc_port *priv = usb_get_serial_port_data(port);

    spin_lock_irqsave(&priv->lpcp_port_lock,flags);
    val = priv->is_loopback;
    spin_unlock_irqrestore(&priv->lpcp_port_lock, flags);


    return sprintf(buf, "%u\n", val);
}

static ssize_t loopback_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    bool val;
    int ret;
    int index = 0;
    unsigned char oob_command[4];
    unsigned long flags = 0;

    struct usb_serial_port *port = to_usb_serial_port(dev);
    struct lpc_port *priv = usb_get_serial_port_data(port);

    index =  port->port_number;

    ret = strtobool(buf, &val);
    if (ret < 0)
        return ret;

    spin_lock_irqsave(&priv->lpcp_port_lock,flags);
    priv->is_loopback = val;
    spin_unlock_irqrestore(&priv->lpcp_port_lock, flags);
    dev_warn(&port->dev, "-----loopback is = %d\n", val);
    /* decide the current port whether in loopback mode */
    oob_command[0] = LPC_CMD_LOCAL_LOOPBACK;
    oob_command[1] = index;
    oob_command[2] = val;
    oob_command[3] = 0;

    ret = lpc_write_oob_command(port, oob_command, 4, 1);
    if(ret)
    {
        dev_warn(&port->dev, "lpc_write: write oob failed, ret=%d\n", ret);
        return ret;
    }

    return count;
}
static DEVICE_ATTR_RW(loopback);


static ssize_t port_number_show(struct device *dev,struct device_attribute *attr, char *buf)
{
     struct usb_serial_port *port = to_usb_serial_port(dev);

     return sprintf(buf, "%u\n", port->port_number);
}
static DEVICE_ATTR_RO(port_number);
static ssize_t send_bytes_show(struct device *dev,struct device_attribute *attr, char *buf)
{
     unsigned long long send_num = 0;
     unsigned long flags = 0;
     struct usb_serial_port *port = to_usb_serial_port(dev);
     struct lpc_port *priv = usb_get_serial_port_data(port);

     spin_lock_irqsave(&priv->lpcp_port_lock,flags);
     send_num = priv->lpcp_download_num;
     spin_unlock_irqrestore(&priv->lpcp_port_lock, flags);

     return sprintf(buf, "%llu\n", send_num);
}
static DEVICE_ATTR_RO(send_bytes);
static ssize_t recv_bytes_show(struct device *dev,struct device_attribute *attr, char *buf)
{
     unsigned long long recv_num = 0;
     unsigned long flags = 0;
     struct usb_serial_port *port = to_usb_serial_port(dev);
     struct lpc_port *priv = usb_get_serial_port_data(port);

     spin_lock_irqsave(&priv->lpcp_port_lock, flags);
     recv_num = priv->lpcp_upload_num;
     spin_unlock_irqrestore(&priv->lpcp_port_lock, flags);

     return sprintf(buf, "%llu\n", recv_num);
}
static DEVICE_ATTR_RO(recv_bytes);
static struct attribute *usb_serial_port_attrs[] = {
     &dev_attr_port_number.attr,
     &dev_attr_send_bytes.attr,
     &dev_attr_recv_bytes.attr,
     &dev_attr_loopback.attr,
     NULL
};
ATTRIBUTE_GROUPS(usb_serial_port);




static void set_endpoints(struct usb_serial *serial,struct usb_serial_endpoints *epds)
{
    struct usb_endpoint_descriptor *epd_bulk_in_data  = epds->bulk_in[0];
    struct usb_endpoint_descriptor *epd_bulk_in_oob   = epds->bulk_in[1];
    struct usb_endpoint_descriptor *epd_bulk_out_data = epds->bulk_out[0];
    struct usb_endpoint_descriptor *epd_bulk_out_oob  = epds->bulk_out[1];
    unsigned int i;

    for (i = 1; i < epds->num_bulk_in; ++i) {
        if(i != (epds->num_bulk_in - 1))
        {
            epds->bulk_in[i]  = epd_bulk_in_data;
            epds->bulk_out[i] = epd_bulk_out_data;
        }
        else
        {
            epds->bulk_in[i]  = epd_bulk_in_oob;
            epds->bulk_out[i] = epd_bulk_out_oob;
        }
    }
}

static int lpc_probe(struct usb_serial *serial,const struct usb_device_id *id)
{
    struct usb_serial_driver *driver = serial->type;
    driver->num_bulk_in  = REAL_NUM_BULK_IN;
    driver->num_bulk_out = REAL_NUM_BULK_OUT;

    return 0;
}

static int lpc_calc_num_ports(struct usb_serial *serial,struct usb_serial_endpoints *epds)
{
    struct usb_serial_driver *driver = serial->type;
    int num_ports;

    driver->num_bulk_in  = NUM_BULK_IN;
    driver->num_bulk_out = NUM_BULK_OUT;
    epds->num_bulk_in    = driver->num_bulk_in;
    epds->num_bulk_out   = driver->num_bulk_out;
    num_ports            = driver->num_ports;
    set_endpoints(serial,epds);

    return num_ports;
}
/*
 *  Cond Wait Interruptible Timeout Irqrestore
 *
 *  Do spin_unlock_irqrestore and interruptible_sleep_on_timeout
 *  so that wake ups are not lost if they occur between the unlock
 *  and the sleep.  In other words, spin_unlock_irqrestore and
 *  interruptible_sleep_on_timeout are "atomic" with respect to
 *  wake ups.  This is used to implement condition variables.
 *
 *  interruptible_sleep_on_timeout is deprecated and has been replaced
 *  with the equivalent code.
 */

static long cond_wait_interruptible_timeout_irqrestore(
    wait_queue_head_t *q, long timeout,
    spinlock_t *lock, unsigned long flags)
__releases(lock)
{
    DEFINE_WAIT(wait);

    prepare_to_wait(q, &wait, TASK_INTERRUPTIBLE);
    spin_unlock_irqrestore(lock, flags);
    timeout = schedule_timeout(timeout);
    finish_wait(q, &wait);

    return timeout;
}

/*
 *  lpc Wakeup Write
 *
 *  Wake up port, line discipline, and tty processes sleeping
 *  on writes.
 */

static void lpc_wakeup_write_lock(struct work_struct *work)
{
    struct lpc_port *priv =
            container_of(work, struct lpc_port, lpcp_wakeup_work);
    struct usb_serial_port *port = priv->lpcp_port;
    unsigned long flags;

    spin_lock_irqsave(&priv->lpcp_port_lock, flags);
    tty_port_tty_wakeup(&port->port);
    spin_unlock_irqrestore(&priv->lpcp_port_lock, flags);
}

/*
 *  lpc Write OOB Command
 *
 *  Write commands on the out of band port.  Commands are 4
 *  bytes each, multiple commands can be sent at once, and
 *  no command will be split across USB packets.  Returns 0
 *  if successful, -EINTR if interrupted while sleeping and
 *  the interruptible flag is true, or a negative error
 *  returned by usb_submit_urb.
 */

static int lpc_write_oob_command(struct usb_serial_port *port,
    unsigned char *buf, int count, int interruptible)
{
    int ret = 0;
    int len;
    struct usb_serial_port *oob_port = (struct usb_serial_port *)((struct lpc_serial *)(usb_get_serial_data(port->serial)))->lpcs_oob_port;
    struct lpc_port *oob_priv = usb_get_serial_port_data(oob_port);
    unsigned long flags = 0;

    dev_dbg(&port->dev,
        "lpc_write_oob_command: TOP: port=%d, count=%d\n",
        oob_priv->lpcp_port_num, count);

    spin_lock_irqsave(&oob_priv->lpcp_port_lock, flags);
    while (count > 0) {
        while (oob_priv->lpcp_write_urb_in_use) {
            cond_wait_interruptible_timeout_irqrestore(
                &oob_port->write_wait, LPC_RETRY_TIMEOUT,
                &oob_priv->lpcp_port_lock, flags);
            if (interruptible && signal_pending(current))
                return -EINTR;
            spin_lock_irqsave(&oob_priv->lpcp_port_lock, flags);
        }

        /* len must be a multiple of 4, so commands are not split */
        len = min(count, oob_port->bulk_out_size);
        if (len > 4)
            len &= ~3;
        memcpy(oob_port->write_urb->transfer_buffer, buf, len);
        oob_port->write_urb->transfer_buffer_length = len;
        ret = usb_submit_urb(oob_port->write_urb, GFP_ATOMIC);
        if (ret == 0) {
            oob_priv->lpcp_write_urb_in_use = 1;
            count -= len;
            buf += len;
        }
    }
    spin_unlock_irqrestore(&oob_priv->lpcp_port_lock, flags);
    if (ret)
        dev_err(&port->dev, "%s: usb_submit_urb failed, ret=%d\n",
            __func__, ret);
    return ret;

}


/*
 *  lpc Write In Band Command
 *
 *  Write commands on the given port.  Commands are 4
 *  bytes each, multiple commands can be sent at once, and
 *  no command will be split across USB packets.  If timeout
 *  is non-zero, write in band command will return after
 *  waiting unsuccessfully for the URB status to clear for
 *  timeout ticks.  Returns 0 if successful, or a negative
 *  error returned by lpc_write.
 */

static int lpc_write_inb_command(struct usb_serial_port *port,
    unsigned char *buf, int count, unsigned long timeout)
{
    int ret = 0;
    int len;

    int index =  port->port_number;
    struct lpc_port *priv = usb_get_serial_port_data(port);
    unsigned char *data = port->write_urb->transfer_buffer;
    unsigned long flags = 0;

    dev_dbg(&port->dev, "lpc_write_inb_command: TOP: port=%d, count=%d\n",
        priv->lpcp_port_num, count);

    if (timeout)
        timeout += jiffies;
    else
        timeout = ULONG_MAX;

    spin_lock_irqsave(&priv->lpcp_port_lock, flags);
    while (count > 0 && ret == 0) {
        while (priv->lpcp_write_urb_in_use &&
               time_before(jiffies, timeout)) {
            cond_wait_interruptible_timeout_irqrestore(
                &port->write_wait, LPC_RETRY_TIMEOUT,
                &priv->lpcp_port_lock, flags);
            if (signal_pending(current))
                return -EINTR;
            spin_lock_irqsave(&priv->lpcp_port_lock, flags);
        }

        /* len must be a multiple of 4 and small enough to */
        /* guarantee the write will send buffered data first, */
        /* so commands are in order with data and not split */
        len = min(count, port->bulk_out_size-3-priv->lpcp_out_buf_len);
        if (len > 4)
            len &= ~3;

        /* write any buffered data first */
        if (priv->lpcp_out_buf_len > 0) {
            data[0] = LPC_CMD_SEND_DATA;

            data[1] =  index;
            data[2] = priv->lpcp_out_buf_len;
            memcpy(data + 3, priv->lpcp_out_buf,
                priv->lpcp_out_buf_len);
            memcpy(data + 3 + priv->lpcp_out_buf_len, buf, len);
            port->write_urb->transfer_buffer_length
                = priv->lpcp_out_buf_len + 3 + len;
        } else {
            memcpy(data, buf, len);
            port->write_urb->transfer_buffer_length = len;
        }

        ret = usb_submit_urb(port->write_urb, GFP_ATOMIC);
        if (ret == 0) {
            priv->lpcp_write_urb_in_use = 1;
            priv->lpcp_out_buf_len = 0;
            count -= len;
            buf += len;
        }

    }
    spin_unlock_irqrestore(&priv->lpcp_port_lock, flags);

    if (ret)
        dev_err(&port->dev,
            "%s: usb_submit_urb failed, ret=%d, port=%d\n",
            __func__, ret, priv->lpcp_port_num);
    return ret;
}


/*
 *  lpc Set Modem Signals
 *
 *  Sets or clears DTR and RTS on the port, according to the
 *  modem_signals argument.  Use TIOCM_DTR and TIOCM_RTS flags
 *  for the modem_signals argument.  Returns 0 if successful,
 *  -EINTR if interrupted while sleeping, or a non-zero error
 *  returned by usb_submit_urb.
 */

static int lpc_set_modem_signals(struct usb_serial_port *port,
    unsigned int modem_signals, int interruptible)
{

    int ret;
    struct lpc_port *port_priv = usb_get_serial_port_data(port);
    struct usb_serial_port *oob_port = (struct usb_serial_port *) ((struct lpc_serial *)(usb_get_serial_data(port->serial)))->lpcs_oob_port;
    struct lpc_port *oob_priv = usb_get_serial_port_data(oob_port);
    unsigned char *data = oob_port->write_urb->transfer_buffer;
    unsigned long flags = 0;


    dev_dbg(&port->dev,
        "lpc_set_modem_signals: TOP: port=%d, modem_signals=0x%x\n",
        port_priv->lpcp_port_num, modem_signals);

    spin_lock_irqsave(&oob_priv->lpcp_port_lock, flags);
    spin_lock(&port_priv->lpcp_port_lock);

    while (oob_priv->lpcp_write_urb_in_use) {
        spin_unlock(&port_priv->lpcp_port_lock);
        cond_wait_interruptible_timeout_irqrestore(
            &oob_port->write_wait, LPC_RETRY_TIMEOUT,
            &oob_priv->lpcp_port_lock, flags);
        if (interruptible && signal_pending(current))
            return -EINTR;
        spin_lock_irqsave(&oob_priv->lpcp_port_lock, flags);
        spin_lock(&port_priv->lpcp_port_lock);
    }
    data[0] = LPC_CMD_SET_DTR_SIGNAL;
    data[1] = port_priv->lpcp_port_num;
    data[2] = (modem_signals & TIOCM_DTR) ?
                    LPC_DTR_ACTIVE : LPC_DTR_INACTIVE;
    data[3] = 0;
    data[4] = LPC_CMD_SET_RTS_SIGNAL;
    data[5] = port_priv->lpcp_port_num;
    data[6] = (modem_signals & TIOCM_RTS) ?
                    LPC_RTS_ACTIVE : LPC_RTS_INACTIVE;
    data[7] = 0;

    oob_port->write_urb->transfer_buffer_length = 8;

    ret = usb_submit_urb(oob_port->write_urb, GFP_ATOMIC);
    if (ret == 0) {
        oob_priv->lpcp_write_urb_in_use = 1;
        port_priv->lpcp_modem_signals =
            (port_priv->lpcp_modem_signals&~(TIOCM_DTR|TIOCM_RTS))
            | (modem_signals&(TIOCM_DTR|TIOCM_RTS));
    }
    spin_unlock(&port_priv->lpcp_port_lock);
    spin_unlock_irqrestore(&oob_priv->lpcp_port_lock, flags);
    if (ret)
        dev_err(&port->dev, "%s: usb_submit_urb failed, ret=%d\n",
            __func__, ret);
    return ret;
}

/*
 *  lpc Transmit Idle
 *
 *  lpc transmit idle waits, up to timeout ticks, for the transmitter
 *  to go idle.  It returns 0 if successful or a negative error.
 *
 *  There are race conditions here if more than one process is calling
 *  lpc_transmit_idle on the same port at the same time.  However, this
 *  is only called from close, and only one process can be in close on a
 *  port at a time, so its ok.
 */

static int lpc_transmit_idle(struct usb_serial_port *port,
    unsigned long timeout)
{
    int ret;
    unsigned char buf[2];
    struct lpc_port *priv = usb_get_serial_port_data(port);
    unsigned long flags = 0;

    spin_lock_irqsave(&priv->lpcp_port_lock, flags);
    priv->lpcp_transmit_idle = 0;
    spin_unlock_irqrestore(&priv->lpcp_port_lock, flags);

    buf[0] = LPC_CMD_TRANSMIT_IDLE;
    buf[1] = 0;

    timeout += jiffies;

    ret = lpc_write_inb_command(port, buf, 2, timeout - jiffies);
    if (ret != 0)
        return ret;

    spin_lock_irqsave(&priv->lpcp_port_lock, flags);

    while (time_before(jiffies, timeout) && !priv->lpcp_transmit_idle) {
        cond_wait_interruptible_timeout_irqrestore(
            &priv->lpcp_transmit_idle_wait, LPC_RETRY_TIMEOUT,
            &priv->lpcp_port_lock, flags);
        if (signal_pending(current))
            return -EINTR;
        spin_lock_irqsave(&priv->lpcp_port_lock, flags);
    }
    priv->lpcp_transmit_idle = 0;
    spin_unlock_irqrestore(&priv->lpcp_port_lock, flags);
    return 0;

}


static void lpc_rx_throttle(struct tty_struct *tty)
{
    unsigned long flags;
    struct usb_serial_port *port = tty->driver_data;
    struct lpc_port *priv = usb_get_serial_port_data(port);

    /* stop receiving characters by not resubmitting the read urb */
    spin_lock_irqsave(&priv->lpcp_port_lock, flags);
    priv->lpcp_throttled = 1;
    priv->lpcp_throttle_restart = 0;
    spin_unlock_irqrestore(&priv->lpcp_port_lock, flags);
}


static void lpc_rx_unthrottle(struct tty_struct *tty)
{
    int ret = 0;
    unsigned long flags;
    struct usb_serial_port *port = tty->driver_data;
    struct lpc_port *priv = usb_get_serial_port_data(port);

    spin_lock_irqsave(&priv->lpcp_port_lock, flags);

    /* restart read chain */
    if (priv->lpcp_throttle_restart){

         ret = usb_submit_urb(port->read_urb, GFP_ATOMIC);

    }
    /* turn throttle off */
    priv->lpcp_throttled = 0;
    priv->lpcp_throttle_restart = 0;

    spin_unlock_irqrestore(&priv->lpcp_port_lock, flags);
    if (ret)
        dev_err(&port->dev,
            "%s: usb_submit_urb failed, ret=%d, port=%d\n",
            __func__, ret, priv->lpcp_port_num);
}


static void lpc_set_termios(struct tty_struct *tty,
        struct usb_serial_port *port, struct ktermios *old_termios)
{
    struct lpc_port *priv = usb_get_serial_port_data(port);
    struct device *dev = &port->dev;
    unsigned int iflag = tty->termios.c_iflag;
    unsigned int cflag = tty->termios.c_cflag;
    unsigned int old_iflag = old_termios->c_iflag;
    unsigned int old_cflag = old_termios->c_cflag;
    unsigned char buf[32];
    unsigned int modem_signals;
    int arg, ret;
    int i = 0;
    speed_t baud;

    dev_dbg(dev,
        "lpc_set_termios: TOP: port=%d, iflag=0x%x, old_iflag=0x%x, cflag=0x%x, old_cflag=0x%x\n",
        priv->lpcp_port_num, iflag, old_iflag, cflag, old_cflag);

    /* set baud rate */
    baud = tty_get_baud_rate(tty);
    if (baud != tty_termios_baud_rate(old_termios)) {
        arg = -1;

        /* reassert DTR and (maybe) RTS on transition from B0 */
        if ((old_cflag & CBAUD) == B0) {
            /* don't set RTS if using hardware flow control */
            /* and throttling input */
            modem_signals = TIOCM_DTR;
            if (!C_CRTSCTS(tty) || !tty_throttled(tty))
                modem_signals |= TIOCM_RTS;
            lpc_set_modem_signals(port, modem_signals, 1);
        }
        switch (baud) {
        /* drop DTR and RTS on transition to B0 */
        case 0: lpc_set_modem_signals(port, 0, 1); break;
        case 50: arg = LPC_BAUD_50; break;
        case 75: arg = LPC_BAUD_75; break;
        case 110: arg = LPC_BAUD_110; break;
        case 150: arg = LPC_BAUD_150; break;
        case 200: arg = LPC_BAUD_200; break;
        case 300: arg = LPC_BAUD_300; break;
        case 600: arg = LPC_BAUD_600; break;
        case 1200: arg = LPC_BAUD_1200; break;
        case 1800: arg = LPC_BAUD_1800; break;
        case 2400: arg = LPC_BAUD_2400; break;
        case 4800: arg = LPC_BAUD_4800; break;
        case 9600: arg = LPC_BAUD_9600; break;
        case 19200: arg = LPC_BAUD_19200; break;
        case 38400: arg = LPC_BAUD_38400; break;
        case 57600: arg = LPC_BAUD_57600; break;
        case 115200: arg = LPC_BAUD_115200; break;
        case 230400: arg = LPC_BAUD_230400; break;
        case 460800: arg = LPC_BAUD_460800; break;
	case 921600:  arg = LPC_BAUD_921600; break;
        case 1000000: arg = LPC_BAUD_1000000; break;
        case 2000000: arg = LPC_BAUD_2000000; break;

        default:
            arg = LPC_BAUD_9600;
            baud = 9600;
            break;
        }
        if (arg != -1) {
            buf[i++] = LPC_CMD_SET_BAUD_RATE;
            buf[i++] = priv->lpcp_port_num;
            buf[i++] = arg;
            buf[i++] = 0;
        }
    }
    /* set parity */
    tty->termios.c_cflag &= ~CMSPAR;

    if ((cflag&(PARENB|PARODD)) != (old_cflag&(PARENB|PARODD))) {
        if (cflag&PARENB) {
            if (cflag&PARODD)
                arg = LPC_PARITY_ODD;
            else
                arg = LPC_PARITY_EVEN;
        } else {
            arg = LPC_PARITY_NONE;
        }
        buf[i++] = LPC_CMD_SET_PARITY;
        buf[i++] = priv->lpcp_port_num;
        buf[i++] = arg;
        buf[i++] = 0;
    }
    /* set word size */
    if ((cflag&CSIZE) != (old_cflag&CSIZE)) {
        arg = -1;
        switch (cflag&CSIZE) {
        case CS5: arg = LPC_WORD_SIZE_5; break;
        case CS6: arg = LPC_WORD_SIZE_6; break;
        case CS7: arg = LPC_WORD_SIZE_7; break;
        case CS8: arg = LPC_WORD_SIZE_8; break;
        default:
            dev_dbg(dev,
                "lpc_set_termios: can't handle word size %d\n",
                (cflag&CSIZE));
            break;
        }

        if (arg != -1) {
            buf[i++] = LPC_CMD_SET_WORD_SIZE;
            buf[i++] = priv->lpcp_port_num;
            buf[i++] = arg;
            buf[i++] = 0;
        }

    }

    /* set stop bits */
    if ((cflag&CSTOPB) != (old_cflag&CSTOPB)) {

        if ((cflag&CSTOPB))
            arg = LPC_STOP_BITS_2;
        else
            arg = LPC_STOP_BITS_1;

        buf[i++] = LPC_CMD_SET_STOP_BITS;
        buf[i++] = priv->lpcp_port_num;
        buf[i++] = arg;
        buf[i++] = 0;

    }

    /* set input flow control */
    if ((iflag&IXOFF) != (old_iflag&IXOFF)
        || (cflag&CRTSCTS) != (old_cflag&CRTSCTS)) {
        arg = 0;
        if (iflag&IXOFF)
            arg |= LPC_INPUT_FLOW_CONTROL_XON_XOFF;
        else
            arg &= ~LPC_INPUT_FLOW_CONTROL_XON_XOFF;

        if (cflag&CRTSCTS) {
            arg |= LPC_INPUT_FLOW_CONTROL_RTS;

            /* On USB-4 it is necessary to assert RTS prior */
            /* to selecting RTS input flow control.  */
            buf[i++] = LPC_CMD_SET_RTS_SIGNAL;
            buf[i++] = priv->lpcp_port_num;
            buf[i++] = LPC_RTS_ACTIVE;
            buf[i++] = 0;

        } else {
            arg &= ~LPC_INPUT_FLOW_CONTROL_RTS;
        }
        buf[i++] = LPC_CMD_SET_INPUT_FLOW_CONTROL;
        buf[i++] = priv->lpcp_port_num;
        buf[i++] = arg;
        buf[i++] = 0;
    }

    /* set output flow control */
    if ((iflag & IXON) != (old_iflag & IXON)
        || (cflag & CRTSCTS) != (old_cflag & CRTSCTS)) {
        arg = 0;
        if (iflag & IXON)
            arg |= LPC_OUTPUT_FLOW_CONTROL_XON_XOFF;
        else
            arg &= ~LPC_OUTPUT_FLOW_CONTROL_XON_XOFF;

        if (cflag & CRTSCTS) {
            arg |= LPC_OUTPUT_FLOW_CONTROL_CTS;
        } else {
            arg &= ~LPC_OUTPUT_FLOW_CONTROL_CTS;
        }

        buf[i++] = LPC_CMD_SET_OUTPUT_FLOW_CONTROL;
        buf[i++] = priv->lpcp_port_num;
        buf[i++] = arg;
        buf[i++] = 0;
    }

    /* set receive enable/disable */
    if ((cflag & CREAD) != (old_cflag & CREAD)) {
        if (cflag & CREAD)
            arg = LPC_ENABLE;
        else
            arg = LPC_DISABLE;

        buf[i++] = LPC_CMD_RECEIVE_ENABLE;
        buf[i++] = priv->lpcp_port_num;
        buf[i++] = arg;
        buf[i++] = 0;
    }
    ret = lpc_write_oob_command(port, buf, i, 1);
    if (ret != 0)
        dev_dbg(dev, "lpc_set_termios: write oob failed, ret=%d\n", ret);
    tty_encode_baud_rate(tty, baud, baud);
}


static void lpc_break_ctl(struct tty_struct *tty, int break_state)
{
    struct usb_serial_port *port = tty->driver_data;
    unsigned char buf[4];

    buf[0] = LPC_CMD_BREAK_CONTROL;
    buf[1] = 2;				/* length */
    buf[2] = break_state ? 1 : 0;
    buf[3] = 0;				/* pad */
    lpc_write_inb_command(port, buf, 4, 0);
}


static int lpc_tiocmget(struct tty_struct *tty)
{
    struct usb_serial_port *port = tty->driver_data;
    struct lpc_port *priv = usb_get_serial_port_data(port);
    unsigned int val;
    unsigned long flags;

    spin_lock_irqsave(&priv->lpcp_port_lock, flags);
    val = priv->lpcp_modem_signals;
    spin_unlock_irqrestore(&priv->lpcp_port_lock, flags);
    return val;
}


static int lpc_tiocmset(struct tty_struct *tty,
                    unsigned int set, unsigned int clear)
{
    struct usb_serial_port *port = tty->driver_data;
    struct lpc_port *priv = usb_get_serial_port_data(port);
    unsigned int val;
    unsigned long flags;

    spin_lock_irqsave(&priv->lpcp_port_lock, flags);
    val = (priv->lpcp_modem_signals & ~clear) | set;
    spin_unlock_irqrestore(&priv->lpcp_port_lock, flags);
    return lpc_set_modem_signals(port, val, 1);
}

static int lpc_write(struct tty_struct *tty, struct usb_serial_port *port,
                    const unsigned char *buf, int count)
{

    int ret, data_len, new_len;
    int index =  port->port_number;
    struct lpc_port *priv = usb_get_serial_port_data(port);
    unsigned char *data = port->write_urb->transfer_buffer;
    unsigned long flags = 0;
    unsigned char oob_command[4];

    dev_dbg(&port->dev,
        "lpc_write: TOP: port=%d, count=%d, in_interrupt=%ld\n",
        priv->lpcp_port_num, count, in_interrupt());

    /* copy user data (which can sleep) before getting spin lock */
    count = min(count, port->bulk_out_size-3);
    count = min(253, count);

    /* be sure only one write proceeds at a time */
    /* there are races on the port private buffer */
    spin_lock_irqsave(&priv->lpcp_port_lock, flags);

    /* wait for urb status clear to submit another urb */
    if (priv->lpcp_write_urb_in_use) {
        /* buffer data if count is 1 (probably put_char) if possible */
        if (count == 1 && priv->lpcp_out_buf_len < LPC_OUT_BUF_SIZE) {
            priv->lpcp_out_buf[priv->lpcp_out_buf_len++] = *buf;
            new_len = 1;
        } else {
            new_len = 0;
        }
        spin_unlock_irqrestore(&priv->lpcp_port_lock, flags);
        return new_len;
    }

    /* be sure current port can receive data */
    if(priv->lpcp_threshold_num > LPC_MAX_SEND_NUM){
        oob_command[0] = LPC_CMD_TRANSMIT_FULL;
        oob_command[1] = index;
        oob_command[2] = 0;
        oob_command[3] = 0;
        spin_unlock_irqrestore(&priv->lpcp_port_lock, flags);
        ret = lpc_write_oob_command(port, oob_command, 4, 1);

        if(ret)
        {
            dev_warn(&port->dev, "lpc_write: write oob failed, ret=%d\n", ret);
            return ret;
        }

        spin_lock_irqsave(&priv->lpcp_port_lock, flags);

        cond_wait_interruptible_timeout_irqrestore(&priv->lpcp_transmit_is_full,
                                                   LPC_RETRY_TIMEOUT,&priv->lpcp_port_lock, flags);
        if (signal_pending(current))
            return -EINTR;
        spin_lock_irqsave(&priv->lpcp_port_lock, flags);
        priv->lpcp_threshold_num = 0;

    }


    /* allow space for any buffered data and for new data, up to */
    /* transfer buffer size - 3 (for command, channel, length bytes) */
    new_len = min(count, port->bulk_out_size-3-priv->lpcp_out_buf_len);
    data_len = new_len + priv->lpcp_out_buf_len;
    priv->lpcp_threshold_num += data_len;
    priv->lpcp_download_num    += data_len;

    if (data_len == 0) {
        spin_unlock_irqrestore(&priv->lpcp_port_lock, flags);
        return 0;
    }

    port->write_urb->transfer_buffer_length = data_len+3;

    *data++ = LPC_CMD_SEND_DATA;
    *data++ = (unsigned char)index;
    *data++ = data_len;

    /* copy in buffered data first */
    memcpy(data, priv->lpcp_out_buf, priv->lpcp_out_buf_len);
    data += priv->lpcp_out_buf_len;

    /* copy in new data */
    memcpy(data, buf, new_len);

    ret = usb_submit_urb(port->write_urb, GFP_ATOMIC);
    if (ret == 0) {
        priv->lpcp_write_urb_in_use = 1;
        ret = new_len;
        priv->lpcp_out_buf_len = 0;
    }

    /* return length of new data written, or error */
    spin_unlock_irqrestore(&priv->lpcp_port_lock, flags);
    if (ret < 0)
        dev_err_console(port,
            "%s: usb_submit_urb failed, ret=%d, port=%d\n",
            __func__, ret, priv->lpcp_port_num);
    dev_dbg(&port->dev, "lpc_write: returning %d\n", ret);
    return ret;

}

static void lpc_write_bulk_callback(struct urb *urb)
{

    struct usb_serial_port *port = urb->context;
    int index = port->port_number ;
    struct usb_serial *serial;
    struct lpc_port *priv;
    struct lpc_serial *serial_priv;
    unsigned long flags;
    int ret = 0;
    int status = urb->status;

    /* port and serial sanity check */
    if (port == NULL || (priv = usb_get_serial_port_data(port)) == NULL) {
        pr_err("%s: port or port->private is NULL, status=%d\n",
            __func__, status);
        return;
    }
    serial = port->serial;
    if (serial == NULL || (serial_priv = usb_get_serial_data(serial)) == NULL) {
        dev_err(&port->dev,
            "%s: serial or serial->private is NULL, status=%d\n",
            __func__, status);
        return;
    }

    /* handle oob callback */
    if (priv->lpcp_port_num == serial_priv->lpcs_oob_port_num) {
        dev_dbg(&port->dev, "lpc_write_bulk_callback: oob callback\n");
        spin_lock_irqsave(&priv->lpcp_port_lock, flags);
        priv->lpcp_write_urb_in_use = 0;
        wake_up_interruptible(&port->write_wait);
        spin_unlock_irqrestore(&priv->lpcp_port_lock, flags);
        return;
    }

    /* try to send any buffered data on this port */
    spin_lock_irqsave(&priv->lpcp_port_lock, flags);
    priv->lpcp_write_urb_in_use = 0;
    if (priv->lpcp_out_buf_len > 0) {
        *((unsigned char *)(port->write_urb->transfer_buffer))
            = (unsigned char)LPC_CMD_SEND_DATA;
        *((unsigned char *)(port->write_urb->transfer_buffer) + 1)
            = (unsigned char)index;
        *((unsigned char *)(port->write_urb->transfer_buffer) + 2)
            = (unsigned char)priv->lpcp_out_buf_len;
        port->write_urb->transfer_buffer_length =
                        priv->lpcp_out_buf_len + 3;
        memcpy(port->write_urb->transfer_buffer + 3, priv->lpcp_out_buf,
            priv->lpcp_out_buf_len);
        ret = usb_submit_urb(port->write_urb, GFP_ATOMIC);
        if (ret == 0) {
            priv->lpcp_write_urb_in_use = 1;
            priv->lpcp_out_buf_len = 0;
        }
    }
    /* wake up processes sleeping on writes immediately */
    tty_port_tty_wakeup(&port->port);
    /* also queue up a wakeup at scheduler time, in case we */
    /* lost the race in write_chan(). */
    schedule_work(&priv->lpcp_wakeup_work);

    spin_unlock_irqrestore(&priv->lpcp_port_lock, flags);
    if (ret && ret != -EPERM)
        dev_err_console(port,
            "%s: usb_submit_urb failed, ret=%d, port=%d\n",
            __func__, ret, priv->lpcp_port_num);
}

static int lpc_write_room(struct tty_struct *tty)
{
    struct usb_serial_port *port = tty->driver_data;
    struct lpc_port *priv = usb_get_serial_port_data(port);
    int room;
    unsigned long flags = 0;

    spin_lock_irqsave(&priv->lpcp_port_lock, flags);

    if (priv->lpcp_write_urb_in_use)
        room = 0;
    else
        room = port->bulk_out_size - 3 - priv->lpcp_out_buf_len;

    spin_unlock_irqrestore(&priv->lpcp_port_lock, flags);
    dev_dbg(&port->dev, "lpc_write_room: port=%d, room=%d\n", priv->lpcp_port_num, room);
    return room;

}

static int lpc_chars_in_buffer(struct tty_struct *tty)
{
    struct usb_serial_port *port = tty->driver_data;
    struct lpc_port *priv = usb_get_serial_port_data(port);

    if (priv->lpcp_write_urb_in_use) {
        dev_dbg(&port->dev, "lpc_chars_in_buffer: port=%d, chars=%d\n",
            priv->lpcp_port_num, port->bulk_out_size - 3);
        /* return(port->bulk_out_size - 2); */
        return 256;
    } else {
        dev_dbg(&port->dev, "lpc_chars_in_buffer: port=%d, chars=%d\n",
            priv->lpcp_port_num, priv->lpcp_out_buf_len);
        return priv->lpcp_out_buf_len;
    }

}

static void lpc_dtr_rts(struct usb_serial_port *port, int on)
{
    /* Adjust DTR and RTS */
    lpc_set_modem_signals(port, on * (TIOCM_DTR|TIOCM_RTS), 1);
}

static int lpc_open(struct tty_struct *tty, struct usb_serial_port *port)
{
    int ret;
    unsigned char buf[8];
    struct lpc_port *priv = usb_get_serial_port_data(port);
    struct ktermios not_termios;
    /* be sure the device is started up */
    if (lpc_startup_device(port->serial) != 0)
        return -ENXIO;

    /* read modem signals automatically whenever they change */
    buf[0] = LPC_CMD_READ_INPUT_SIGNALS;
    buf[1] = priv->lpcp_port_num;
    buf[2] = LPC_ENABLE;
    buf[3] = 0;

    /* flush fifos */
    buf[4] = LPC_CMD_IFLUSH_FIFO;
    buf[5] = priv->lpcp_port_num;
    buf[6] = LPC_FLUSH_TX | LPC_FLUSH_RX;
    buf[7] = 0;

    ret = lpc_write_oob_command(port, buf, 8, 1);
    if (ret != 0)
        dev_dbg(&port->dev, "lpc_open: write oob failed, ret=%d\n", ret);

    /* set termios settings */
    if (tty) {
        not_termios.c_cflag = ~tty->termios.c_cflag;
        not_termios.c_iflag = ~tty->termios.c_iflag;
        lpc_set_termios(tty, port, &not_termios);
    }
    return 0;
}


static void lpc_close(struct usb_serial_port *port)
{
    DEFINE_WAIT(wait);
    int ret;
    unsigned char buf[32];
    struct lpc_port *priv = usb_get_serial_port_data(port);

    mutex_lock(&port->serial->disc_mutex);
    /* if disconnected, just clear flags */
    if (port->serial->disconnected)
        goto exit;

    /* FIXME: Transmit idle belongs in the wait_unti_sent path */
    lpc_transmit_idle(port, LPC_CLOSE_TIMEOUT);

    /* disable input flow control */
    buf[0] = LPC_CMD_SET_INPUT_FLOW_CONTROL;
    buf[1] = priv->lpcp_port_num;
    buf[2] = LPC_DISABLE;
    buf[3] = 0;

    /* disable output flow control */
    buf[4] = LPC_CMD_SET_OUTPUT_FLOW_CONTROL;
    buf[5] = priv->lpcp_port_num;
    buf[6] = LPC_DISABLE;
    buf[7] = 0;

    /* disable reading modem signals automatically */
    buf[8] = LPC_CMD_READ_INPUT_SIGNALS;
    buf[9] = priv->lpcp_port_num;
    buf[10] = LPC_DISABLE;
    buf[11] = 0;

    /* disable receive */
    buf[12] = LPC_CMD_RECEIVE_ENABLE;
    buf[13] = priv->lpcp_port_num;
    buf[14] = LPC_DISABLE;
    buf[15] = 0;

    /* flush fifos */
    buf[16] = LPC_CMD_IFLUSH_FIFO;
    buf[17] = priv->lpcp_port_num;
    buf[18] = LPC_FLUSH_TX | LPC_FLUSH_RX;
    buf[19] = 0;

    ret = lpc_write_oob_command(port, buf, 20, 0);
    if (ret != 0)
        dev_dbg(&port->dev, "lpc_close: write oob failed, ret=%d\n",
                                    ret);
    /* wait for final commands on oob port to complete */
    prepare_to_wait(&priv->lpcp_flush_wait, &wait,
            TASK_INTERRUPTIBLE);
    schedule_timeout(LPC_CLOSE_TIMEOUT);
    finish_wait(&priv->lpcp_flush_wait, &wait);

    /* shutdown any outstanding bulk writes */
    usb_kill_urb(port->write_urb);
exit:
    spin_lock_irq(&priv->lpcp_port_lock);
    priv->lpcp_write_urb_in_use = 0;
    wake_up_interruptible(&priv->lpcp_close_wait);
    spin_unlock_irq(&priv->lpcp_port_lock);
    mutex_unlock(&port->serial->disc_mutex);
}


/*
 *  lpc Startup Device
 *
 *  Starts reads on all ports.  Must be called AFTER startup, with
 *  urbs initialized.  Returns 0 if successful, non-zero error otherwise.
 */
static int lpc_startup_device(struct usb_serial *serial)
{
    int i, ret = 0;
    struct lpc_serial *serial_priv = usb_get_serial_data(serial);
    struct usb_serial_port *port;
    /* be sure this happens exactly once */
    spin_lock(&serial_priv->lpcs_serial_lock);
    if (serial_priv->lpcs_device_started) {
        spin_unlock(&serial_priv->lpcs_serial_lock);
        return 0;
    }
    serial_priv->lpcs_device_started = 1;
    spin_unlock(&serial_priv->lpcs_serial_lock);
    /* start reading from each bulk in endpoint for the device */
    /* set USB_DISABLE_SPD flag for write bulk urbs */
    for (i = 0; i < serial->type->num_ports + 1; i++) {
        port = serial->port[i];
        ret = usb_submit_urb(port->read_urb, GFP_KERNEL);
        if (ret != 0) {
            dev_err(&port->dev,
                "%s: usb_submit_urb failed, ret=%d, port=%d\n",
                __func__, ret, i);
            break;
        }
    }
    return ret;
}

static int lpc_port_init(struct usb_serial_port *port, unsigned port_num)
{
    struct lpc_port *priv;

    priv = kzalloc(sizeof(*priv), GFP_KERNEL);
    if (!priv)
        return -ENOMEM;
    spin_lock_init(&priv->lpcp_port_lock);
    priv->lpcp_port_num = port_num;
    init_waitqueue_head(&priv->lpcp_transmit_idle_wait);
    init_waitqueue_head(&priv->lpcp_transmit_is_full);
    init_waitqueue_head(&priv->lpcp_flush_wait);
    init_waitqueue_head(&priv->lpcp_close_wait);
    INIT_WORK(&priv->lpcp_wakeup_work, lpc_wakeup_write_lock);
    priv->lpcp_port = port;

    init_waitqueue_head(&port->write_wait);

    usb_set_serial_port_data(port, priv);

    return 0;
}
/* add recv_number  and send_number  attribute to the device ttyUSB */
static void lpc_add_attribute(struct usb_serial *serial)
{
    int i;
    struct usb_serial_port *port;

    for(i = 0; i < serial->num_ports;i++){
        port = serial->port[i];
        port->dev.groups = usb_serial_port_groups;

    }

}

static int lpc_startup(struct usb_serial *serial)
{
    struct lpc_serial *serial_priv;
    int ret;

    lpc_add_attribute(serial);
    serial_priv = kzalloc(sizeof(*serial_priv), GFP_KERNEL);
    if (!serial_priv)
        return -ENOMEM;

    spin_lock_init(&serial_priv->lpcs_serial_lock);
    serial_priv->lpcs_oob_port_num = serial->type->num_ports;
    serial_priv->lpcs_oob_port = serial->port[serial_priv->lpcs_oob_port_num];

    ret = lpc_port_init(serial_priv->lpcs_oob_port,
                        serial_priv->lpcs_oob_port_num);
    if (ret) {
        kfree(serial_priv);
        return ret;
    }

    usb_set_serial_data(serial, serial_priv);

    return 0;
}


static void lpc_disconnect(struct usb_serial *serial)
{
    int i;

    /* stop reads and writes on all ports */
    for (i = 0; i < serial->type->num_ports + 1; i++) {
        usb_kill_urb(serial->port[i]->read_urb);
        usb_kill_urb(serial->port[i]->write_urb);
    }
}


static void lpc_release(struct usb_serial *serial)
{
    struct lpc_serial *serial_priv;
    struct lpc_port *priv;

    serial_priv = usb_get_serial_data(serial);

    priv = usb_get_serial_port_data(serial_priv->lpcs_oob_port);
    kfree(priv);

    kfree(serial_priv);
}

static int lpc_port_probe(struct usb_serial_port *port)
{
    return lpc_port_init(port, port->port_number);
}

static int lpc_port_remove(struct usb_serial_port *port)
{
    struct lpc_port *priv;

    priv = usb_get_serial_port_data(port);
    kfree(priv);

    return 0;
}

static void lpc_read_bulk_callback(struct urb *urb)
{
    struct usb_serial_port *port = urb->context;
    struct lpc_port *priv;
    struct lpc_serial *serial_priv;
    int ret;
    int status = urb->status;

    /* port sanity check, do not resubmit if port is not valid */
    if (port == NULL)
        return;
    priv = usb_get_serial_port_data(port);
    if (priv == NULL) {
        dev_err(&port->dev, "%s: port->private is NULL, status=%d\n",
            __func__, status);
        return;
    }
    if (port->serial == NULL ||
        (serial_priv = usb_get_serial_data(port->serial)) == NULL) {
        dev_err(&port->dev, "%s: serial is bad or serial->private "
            "is NULL, status=%d\n", __func__, status);
        return;
    }

    /* do not resubmit urb if it has any status error */
    if (status) {
        dev_err(&port->dev,
            "%s: nonzero read bulk status: status=%d, port=%d\n",
            __func__, status, priv->lpcp_port_num);
        return;
    }

    /* handle oob or inb callback, do not resubmit if error */
    if (priv->lpcp_port_num == serial_priv->lpcs_oob_port_num) {
        if (lpc_read_oob_callback(urb) != 0)
            return;
    } else {
        if (lpc_read_inb_callback(urb) != 0)
            return;
    }

    /* continue read */
    ret = usb_submit_urb(urb, GFP_ATOMIC);
    if (ret != 0 && ret != -EPERM) {
        dev_err(&port->dev,
            "%s: failed resubmitting urb, ret=%d, port=%d\n",
            __func__, ret, priv->lpcp_port_num);
    }

}

/*
 *  lpc Read INB Callback
 *
 *  lpc Read INB Callback handles reads on the in band ports, sending
 *  the data on to the tty subsystem.  When called we know port and
 *  port->private are not NULL and port->serial has been validated.
 *  It returns 0 if successful, 1 if successful but the port is
 *  throttled, and -1 if the sanity checks failed.
 */

static int lpc_read_inb_callback(struct urb *urb)
{
    struct usb_serial_port *port = urb->context;
    struct usb_serial_port *tmp  = port;
    struct lpc_port *priv = NULL;
    struct usb_serial* serial;
    unsigned char *buf = urb->transfer_buffer;
    unsigned long flags;
    int opcode;
    int len;
    int index;
    unsigned char *data;
    int tty_flag, throttled;

    /* short/multiple packet check */
    if (urb->actual_length < 2) {
        dev_warn(&port->dev, "short packet received\n");
        return -1;
    }

    opcode = buf[0];
    index  = buf[1];
    len    = buf[2];
    serial = port->serial;
    port   = serial->port[index];
    priv   = usb_get_serial_port_data(port);

    if (urb->actual_length != len + 3) {
        dev_err(&port->dev, "malformed packet received: port=%d, opcode=%d, len=%d, actual_length=%u\n",
            priv->lpcp_port_num, opcode, len, urb->actual_length);
        return -1;
    }

    if (opcode == LPC_CMD_RECEIVE_DATA && len < 1) {
        dev_err(&port->dev, "malformed data packet received\n");
        return -1;
    }

    spin_lock_irqsave(&priv->lpcp_port_lock, flags);

    /* check for throttle; if set, do not resubmit read urb */
    /* indicate the read chain needs to be restarted on unthrottle */
    throttled = priv->lpcp_throttled;
    if (throttled){
        priv->lpcp_throttle_restart = 1;
    /*swap the urb and urb->context*/
        tmp->read_urb  = port->read_urb;
        tmp->read_urb->context =  tmp;
        urb->context   = port;
        port->read_urb = urb;
    }
    /* receive data */
    if (opcode == LPC_CMD_RECEIVE_DATA) {

        data = &buf[3];

        /* get flag from port_status */
        tty_flag = 0;

        if (len > 0) {
            priv->lpcp_upload_num += len;
            tty_insert_flip_string_fixed_flag(&port->port, data,
                    tty_flag, len);
            tty_flip_buffer_push(&port->port);
        }
    }
    spin_unlock_irqrestore(&priv->lpcp_port_lock, flags);

    if (opcode == LPC_CMD_RECEIVE_DISABLE)
        dev_dbg(&port->dev, "%s: got RECEIVE_DISABLE\n", __func__);
    else if (opcode != LPC_CMD_RECEIVE_DATA)
        dev_dbg(&port->dev, "%s: unknown opcode: %d\n", __func__, opcode);

    return throttled ? 1 : 0;

}


/*
 *  lpc Read OOB Callback
 *
 *  lpc Read OOB Callback handles reads on the out of band port.
 *  When called we know port and port->private are not NULL and
 *  the port->serial is valid.  It returns 0 if successful, and
 *  -1 if the sanity checks failed.
 */

static int lpc_read_oob_callback(struct urb *urb)
{

    struct usb_serial_port *port = urb->context;
    struct usb_serial *serial = port->serial;
    struct tty_struct *tty;
    struct lpc_port *priv = usb_get_serial_port_data(port);
    unsigned char *buf = urb->transfer_buffer;
    int opcode, channel, status, val;
    unsigned long flags;
    int i;
    unsigned int rts;

    if (urb->actual_length < 4)
        return -1;

    /* handle each oob command */
    for (i = 0; i < urb->actual_length; i += 4) {
        opcode = buf[i];
        channel = buf[i + 1];
        status = buf[i + 2];
        val = buf[i + 3];

        dev_dbg(&port->dev, "lpc_read_oob_callback:length=%d, opcode=%d, channel=%d, status=%d, val=%d\n",
            urb->actual_length,opcode, channel, status, val);

        if (status != 0 || channel >= serial->type->num_ports)
            continue;

        port = serial->port[channel];

        priv = usb_get_serial_port_data(port);
        if (priv == NULL)
            return -1;

        /* check whether receive buffer is full  */
        if (opcode == LPC_CMD_TRANSMIT_FULL) {

            if(status == 0){
                wake_up_interruptible(&priv->lpcp_transmit_is_full);
            }

            return 0;
        }

        tty = tty_port_tty_get(&port->port);

        rts = 0;
        if (tty)
            rts = C_CRTSCTS(tty);

        if (tty && opcode == LPC_CMD_READ_INPUT_SIGNALS) {
            spin_lock_irqsave(&priv->lpcp_port_lock, flags);
            /* convert from lpc flags to termiox flags */
            if (val & LPC_READ_INPUT_SIGNALS_CTS) {
                priv->lpcp_modem_signals |= TIOCM_CTS;
                /* port must be open to use tty struct */
                if (rts)
                    tty_port_tty_wakeup(&port->port);
            } else {
                priv->lpcp_modem_signals &= ~TIOCM_CTS;
                /* port must be open to use tty struct */
            }
            if (val & LPC_READ_INPUT_SIGNALS_DSR)
                priv->lpcp_modem_signals |= TIOCM_DSR;
            else
                priv->lpcp_modem_signals &= ~TIOCM_DSR;
            if (val & LPC_READ_INPUT_SIGNALS_RI)
                priv->lpcp_modem_signals |= TIOCM_RI;
            else
                priv->lpcp_modem_signals &= ~TIOCM_RI;
            if (val & LPC_READ_INPUT_SIGNALS_DCD)
                priv->lpcp_modem_signals |= TIOCM_CD;
            else
                priv->lpcp_modem_signals &= ~TIOCM_CD;

            spin_unlock_irqrestore(&priv->lpcp_port_lock, flags);
        } else if (opcode == LPC_CMD_TRANSMIT_IDLE) {

            spin_lock_irqsave(&priv->lpcp_port_lock, flags);
            priv->lpcp_transmit_idle = 1;
            wake_up_interruptible(&priv->lpcp_transmit_idle_wait);
            spin_unlock_irqrestore(&priv->lpcp_port_lock, flags);
        } else if (opcode == LPC_CMD_IFLUSH_FIFO) {
            wake_up_interruptible(&priv->lpcp_flush_wait);
        }
        tty_kref_put(tty);

    }
    return 0;

}

module_usb_serial_driver(serial_drivers, id_table_10);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
