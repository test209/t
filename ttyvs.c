// SPDX-License-Identifier: GPL-2.0
/*
 * Serial port null modem emulation driver
 *
 * Copyright (c) 2020, Rishi Gupta <gupt21@gmail.com>
 */

/*
 * Virtual multi-port serial card:
 *
 * This driver implements a virtual multi-port serial card in such a
 * way that the card can have 0 to N number of virtual serial ports
 * (tty devices). These devices can be used using standard termios
 * and Linux/Posix APIs.
 *
 * DT bindings: Documentation/devicetree/bindings/serial/ttyvs.yaml
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/errno.h>
#include <linux/init.h>
#include <linux/idr.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/sched.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/configfs.h>

/* Pin out configurations definitions */
#define VS_CON_CTS    0x0001
#define VS_CON_DCD    0x0002
#define VS_CON_DSR    0x0004
#define VS_CON_RI     0x0008

/* Modem control register definitions */
#define VS_MCR_DTR    0x0001
#define VS_MCR_RTS    0x0002
#define VS_MCR_LOOP   0x0004

/* Modem status register definitions */
#define VS_MSR_CTS    0x0008
#define VS_MSR_DCD    0x0010
#define VS_MSR_RI     0x0020
#define VS_MSR_DSR    0x0040

/* UART frame structure definitions */
#define VS_CRTSCTS       0x0001
#define VS_XON           0x0002
#define VS_NONE          0x0004
#define VS_DATA_5        0x0008
#define VS_DATA_6        0x0010
#define VS_DATA_7        0x0020
#define VS_DATA_8        0x0040
#define VS_PARITY_NONE   0x0080
#define VS_PARITY_ODD    0x0100
#define VS_PARITY_EVEN   0x0200
#define VS_PARITY_MARK   0x0400
#define VS_PARITY_SPACE  0x0800
#define VS_STOP_1        0x1000
#define VS_STOP_2        0x2000

/* Constants for the device type (odevtyp) */
#define VS_SNM 0x0001
#define VS_CNM 0x0002
#define VS_SLB 0x0003
#define VS_CLB 0x0004

/* Attributes associated with a configfs item (folder/device) */
struct vs_cfs_dev_info {
	struct config_group grp;
	char *devtype;
	int ownidx;
	int peeridx;
	u8 ortsmap;
	u8 odtrmap;
	u8 odtratopn;
	u8 prtsmap;
	u8 pdtrmap;
	u8 pdtratopn;
};

/* Represents a virtual tty device in this virtual card */
struct vs_dev {
	/* index for this device in tty core */
	unsigned int own_index;
	/* index of the device to which this device is connected */
	unsigned int peer_index;
	/* shadow modem status register */
	int msr_reg;
	/* shadow modem control register */
	int mcr_reg;
	/* rts line connections for this device */
	int rts_mappings;
	/* dtr line connections for this device */
	int dtr_mappings;
	int set_odtr_at_open;
	int set_pdtr_at_open;
	int odevtyp;
	/* mutual exclusion at device level */
	struct mutex lock;
	int is_break_on;
	/* currently active baudrate */
	int baud;
	int uart_frame;
	int waiting_msr_chg;
	int tx_paused;
	int faulty_cable;
	struct tty_struct *own_tty;
	struct tty_struct *peer_tty;
	struct serial_struct serial;
	struct async_icount icount;
	struct device *device;
};

/*
 * Index radix tree based database of all devices managed by
 * this driver.
 */
static DEFINE_IDR(db);

/* Used to create and destroy devices atomically/serially */
static DEFINE_MUTEX(card_lock);

/* Describes this driver */
static struct tty_driver *ttyvs_driver;

/* Maximum number of devices supported by this driver */
static ushort max_num_vs_devs = 64;

/*
 * Notifies tty core that a framing/parity/overrun error has happend
 * while receiving data on serial port. When frame or parity error
 * happens, -7 (randomly selected number by this driver) is sent as
 * byte that got corrupted to tty core. For emulation purpose 0 can
 * not be taken as corrupted byte because parity and break both will
 * have same sequence (octal \377 \0 \0) and therefore application
 * will not be able to differentiate between these two.
 *
 * This is also used for asserting/de-asserting ring event on line and
 * notifies tty core when a break condition has been detected on line.
 *
 * 1. Emulate framing error:
 * $ echo "1" > /sys/devices/virtual/tty/ttyvs0/event
 *
 * 2. Emulate parity error:
 * $ echo "2" > /sys/devices/virtual/tty/ttyvs0/event
 *
 * 3. Emulate overrun error:
 * $ echo "3" > /sys/devices/virtual/tty/ttyvs0/event
 *
 * 4. Emulate ring indicator (set RI signal):
 * $ echo "4" > /sys/devices/virtual/tty/ttyvs0/event
 *
 * 5. Emulate ring indicator (unset RI signal):
 * $ echo "5" > /sys/devices/virtual/tty/ttyvs0/event
 *
 * 6. Emulate break received:
 * $ echo "6" > /sys/devices/virtual/tty/ttyvs0/event
 *
 * 7. Emulate cable is faulty (data sent but not received):
 * $ echo "7" > /sys/devices/virtual/tty/ttyvs0/event
 *
 * 8. Remove faulty cable condition:
 * $ echo "8" > /sys/devices/virtual/tty/ttyvs0/event
 */
static ssize_t event_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret, push = 1;
	struct vs_dev *local_vsdev = dev_get_drvdata(dev);
	struct tty_struct *tty_to_write = local_vsdev->own_tty;

	if (!buf || (count <= 0))
		return -EINVAL;

	/*
	 * Ensure required structure has been allocated, initialized and
	 * port has been opened.
	 */
	if (!tty_to_write || (tty_to_write->port == NULL)
			|| (tty_to_write->port->count <= 0))
		return -EIO;
	if (!test_bit(ASYNCB_INITIALIZED, &tty_to_write->port->flags))
		return -EIO;

	mutex_lock(&local_vsdev->lock);

	switch (buf[0]) {
	case '1':
		ret = tty_insert_flip_char(tty_to_write->port, -7, TTY_FRAME);
		if (ret < 0)
			goto fail;
		local_vsdev->icount.frame++;
		break;
	case '2':
		ret = tty_insert_flip_char(tty_to_write->port, -7, TTY_PARITY);
		if (ret < 0)
			goto fail;
		local_vsdev->icount.parity++;
		break;
	case '3':
		ret = tty_insert_flip_char(tty_to_write->port, 0, TTY_OVERRUN);
		if (ret < 0)
			goto fail;
		local_vsdev->icount.overrun++;
		break;
	case '4':
		local_vsdev->msr_reg |= VS_MSR_RI;
		local_vsdev->icount.rng++;
		push = -1;
		break;
	case '5':
		local_vsdev->msr_reg &= ~VS_MSR_RI;
		local_vsdev->icount.rng++;
		push = -1;
		break;
	case '6':
		ret = tty_insert_flip_char(tty_to_write->port, 0, TTY_BREAK);
		if (ret < 0)
			goto fail;
		local_vsdev->icount.brk++;
		break;
	case '7':
		local_vsdev->faulty_cable = 1;
		push = -1;
		break;
	case '8':
		local_vsdev->faulty_cable = 0;
		push = -1;
		break;
	default:
		mutex_unlock(&local_vsdev->lock);
		return -EINVAL;
	}

	if (push)
		tty_flip_buffer_push(tty_to_write->port);
	ret = count;

fail:
	mutex_unlock(&local_vsdev->lock);
	return ret;
}
static DEVICE_ATTR_WO(event);

static struct attribute *ttyvs_attrs[] = {
	&dev_attr_event.attr,
	NULL,
};
ATTRIBUTE_GROUPS(ttyvs);

/*
 * Checks if the given serial port has received its carrier detect
 * line raised or not. Return 1 if the carrier is raised otherwise 0.
 */
static int vs_port_carrier_raised(struct tty_port *port)
{
	struct vs_dev *local_vsdev = idr_find(&db, port->tty->index);

	return (local_vsdev->msr_reg & VS_MSR_DCD) ? 1 : 0;
}

/* Shutdown the given serial port */
static void vs_port_shutdown(struct tty_port *port)
{
	pr_debug("shutting down the port!\n");
}

/*
 * Invoked when tty is going to be destroyed and driver should
 * release resources.
 */
static void vs_port_destruct(struct tty_port *port)
{
	pr_debug("destroying the port!\n");
}

/* Activate the given serial port as opposed to shutdown */
static int vs_port_activate(struct tty_port *port, struct tty_struct *tty)
{
	return 0;
}

static const struct tty_port_operations vs_port_ops = {
	.carrier_raised = vs_port_carrier_raised,
	.shutdown       = vs_port_shutdown,
	.activate       = vs_port_activate,
	.destruct       = vs_port_destruct,
};

/*
 * Update modem control and status registers according to the bit
 * mask(s) provided. The RTS and DTR values can be set only if the
 * current handshaking state of the tty device allows direct control
 * of the modem control lines. The pin mappings are honoured.
 *
 * Caller holds lock of thegiven virtual tty device.
 */
static int vs_update_modem_lines(struct tty_struct *tty,
			unsigned int set, unsigned int clear)
{
	int ctsint = 0;
	int dcdint = 0;
	int dsrint = 0;
	int rngint = 0;
	int mcr_ctrl_reg = 0;
	int wakeup_blocked_open = 0;
	int rts_mappings, dtr_mappings, msr_state_reg;
	struct async_icount *evicount;
	struct vs_dev *vsdev, *local_vsdev, *remote_vsdev;

	local_vsdev = idr_find(&db, tty->index);

	/* Read modify write MSR register */
	if (tty->index != local_vsdev->peer_index) {
		remote_vsdev = idr_find(&db, local_vsdev->peer_index);
		msr_state_reg = remote_vsdev->msr_reg;
		vsdev = remote_vsdev;
	} else {
		msr_state_reg = local_vsdev->msr_reg;
		vsdev = local_vsdev;
	}

	rts_mappings = local_vsdev->rts_mappings;
	dtr_mappings = local_vsdev->dtr_mappings;

	if (set & TIOCM_RTS) {
		mcr_ctrl_reg |= VS_MCR_RTS;
		if ((rts_mappings & VS_CON_CTS) == VS_CON_CTS) {
			msr_state_reg |= VS_MSR_CTS;
			ctsint++;
		}
		if ((rts_mappings & VS_CON_DCD) == VS_CON_DCD) {
			msr_state_reg |= VS_MSR_DCD;
			dcdint++;
			wakeup_blocked_open = 1;
		}
		if ((rts_mappings & VS_CON_DSR) == VS_CON_DSR) {
			msr_state_reg |= VS_MSR_DSR;
			dsrint++;
		}
		if ((rts_mappings & VS_CON_RI) == VS_CON_RI) {
			msr_state_reg |= VS_MSR_RI;
			rngint++;
		}
	}

	if (set & TIOCM_DTR) {
		mcr_ctrl_reg |= VS_MCR_DTR;
		if ((dtr_mappings & VS_CON_CTS) == VS_CON_CTS) {
			msr_state_reg |= VS_MSR_CTS;
			ctsint++;
		}
		if ((dtr_mappings & VS_CON_DCD) == VS_CON_DCD) {
			msr_state_reg |= VS_MSR_DCD;
			dcdint++;
			wakeup_blocked_open = 1;
		}
		if ((dtr_mappings & VS_CON_DSR) == VS_CON_DSR) {
			msr_state_reg |= VS_MSR_DSR;
			dsrint++;
		}
		if ((dtr_mappings & VS_CON_RI) == VS_CON_RI) {
			msr_state_reg |= VS_MSR_RI;
			rngint++;
		}
	}

	if (clear & TIOCM_RTS) {
		mcr_ctrl_reg &= ~VS_MCR_RTS;
		if ((rts_mappings & VS_CON_CTS) == VS_CON_CTS) {
			msr_state_reg &= ~VS_MSR_CTS;
			ctsint++;
		}
		if ((rts_mappings & VS_CON_DCD) == VS_CON_DCD) {
			msr_state_reg &= ~VS_MSR_DCD;
			dcdint++;
		}
		if ((rts_mappings & VS_CON_DSR) == VS_CON_DSR) {
			msr_state_reg &= ~VS_MSR_DSR;
			dsrint++;
		}
		if ((rts_mappings & VS_CON_RI) == VS_CON_RI) {
			msr_state_reg &= ~VS_MSR_RI;
			rngint++;
		}
	}

	if (clear & TIOCM_DTR) {
		mcr_ctrl_reg &= ~VS_MCR_DTR;
		if ((dtr_mappings & VS_CON_CTS) == VS_CON_CTS) {
			msr_state_reg &= ~VS_MSR_CTS;
			ctsint++;
		}
		if ((dtr_mappings & VS_CON_DCD) == VS_CON_DCD) {
			msr_state_reg &= ~VS_MSR_DCD;
			dcdint++;
		}
		if ((dtr_mappings & VS_CON_DSR) == VS_CON_DSR) {
			msr_state_reg &= ~VS_MSR_DSR;
			dsrint++;
		}
		if ((dtr_mappings & VS_CON_RI) == VS_CON_RI) {
			msr_state_reg &= ~VS_MSR_RI;
			rngint++;
		}
	}

	local_vsdev->mcr_reg = mcr_ctrl_reg;
	vsdev->msr_reg = msr_state_reg;

	evicount = &vsdev->icount;
	evicount->cts += ctsint;
	evicount->dsr += dsrint;
	evicount->dcd += dcdint;
	evicount->rng += rngint;

	if (vsdev->own_tty && vsdev->own_tty->port) {
		/* Wake up process blocked on TIOCMIWAIT ioctl */
		if ((vsdev->waiting_msr_chg == 1) &&
				(vsdev->own_tty->port->count > 0)) {
			wake_up_interruptible(
					&vsdev->own_tty->port->delta_msr_wait);
		}

		/* Wake up application blocked on carrier detect signal */
		if ((wakeup_blocked_open == 1) &&
				(vsdev->own_tty->port->blocked_open > 0)) {
			wake_up_interruptible(&vsdev->own_tty->port->open_wait);
		}
	}

	return 0;
}

/*
 * Invoked when user space process opens a serial port. The tty core
 * calls this to install tty and initialize the required resources.
 */
static int vs_install(struct tty_driver *drv, struct tty_struct *tty)
{
	int ret;
	struct tty_port *port;

	port = kcalloc(1, sizeof(struct tty_port), GFP_KERNEL);
	if (!port)
		return -ENOMEM;

	/* First initialize and then set port operations */
	tty_port_init(port);
	port->ops = &vs_port_ops;

	ret = tty_port_install(port, drv, tty);
	if (ret) {
		kfree(port);
		return ret;
	}

	return 0;
}

/*
 * Invoked when there exist no user process or tty is to be
 * released explicitly for whatever reason.
 */
static void vs_cleanup(struct tty_struct *tty)
{
	tty_port_put(tty->port);
}

/*
 * Called when open system call is called on virtual tty device node.
 * The tty core allocates 'struct tty_struct' for this device and
 * set up various resources, sets up line discipline and call this
 * function. For first time allocation happens and from next time
 * onwards only re-opening happens.
 *
 * The tty core finds the tty driver serving this device node and the
 * index of this tty device as registered by this driver with tty core.
 * From this inded we retrieve the virtual tty device to work on.
 *
 * If the same serial port is opened more than once, the tty structure
 * passed to this function will be same but filp structure will be
 * different every time. Caller holds tty lock.
 *
 * This driver does not set CLOCAL by default. This means that the
 * open() system call will block until it find its carrier detect
 * line raised. Application should use O_NONBLOCK/O_NDELAY flag if
 * it does not want to wait for DCD line change.
 */
static int vs_open(struct tty_struct *tty, struct file *filp)
{
	int ret;
	struct vs_dev *remote_vsdev;
	struct vs_dev *local_vsdev = idr_find(&db, tty->index);

	local_vsdev->own_tty = tty;

	/*
	 * If this device is one end of a null modem connection,
	 * provide its address to remote end.
	 */
	if (tty->index != local_vsdev->peer_index) {
		remote_vsdev = idr_find(&db, local_vsdev->peer_index);
		remote_vsdev->peer_tty = tty;
	}

	memset(&local_vsdev->serial, 0, sizeof(struct serial_struct));
	memset(&local_vsdev->icount, 0, sizeof(struct async_icount));

	/*
	 * Handle DTR raising logic ourselve instead of tty_port helpers
	 * doing it. Locking virtual tty is not required here.
	 */
	if (local_vsdev->set_odtr_at_open == 1)
		vs_update_modem_lines(tty, TIOCM_DTR | TIOCM_RTS, 0);

	/* Associate tty with port and do port level opening. */
	ret = tty_port_open(tty->port, tty, filp);
	if (ret)
		return ret;

	tty->port->close_delay  = 0;
	tty->port->closing_wait = ASYNC_CLOSING_WAIT_NONE;
	tty->port->drain_delay  = 0;

	return ret;
}

/*
 * Invoked by tty layer when release() is called on the file pointer
 * that was previously created with a call to open().
 */
static void vs_close(struct tty_struct *tty, struct file *filp)
{
	if (test_bit(TTY_IO_ERROR, &tty->flags))
		return;

	if (tty && filp && tty->port && (tty->port->count > 0))
		tty_port_close(tty->port, tty, filp);

	if (tty && C_HUPCL(tty) && tty->port && (tty->port->count < 1))
		vs_update_modem_lines(tty, 0, TIOCM_DTR | TIOCM_RTS);
}

/*
 * Invoked when write() system call is invoked on device node.
 * This function constructs evry byte as per the current uart
 * frame settings. Finally, the data is inserted into the tty
 * buffer of the receiver tty device.
 */
static int vs_write(struct tty_struct *tty,
			const unsigned char *buf, int count)
{
	int x;
	unsigned char *data = NULL;
	struct tty_struct *tty_to_write = NULL;
	struct vs_dev *rx_vsdev = NULL;
	struct vs_dev *tx_vsdev = idr_find(&db, tty->index);

	if (tx_vsdev->tx_paused || !tty || tty->stopped
			|| (count < 1) || !buf || tty->hw_stopped)
		return 0;

	if (tx_vsdev->is_break_on == 1) {
		pr_debug("break condition is on!\n");
		return -EIO;
	}

	if (tx_vsdev->faulty_cable == 1)
		return count;

	if (tty->index != tx_vsdev->peer_index) {
		/* Null modem */
		tty_to_write = tx_vsdev->peer_tty;
		rx_vsdev = idr_find(&db, tx_vsdev->peer_index);

		if ((tx_vsdev->baud != rx_vsdev->baud) ||
			(tx_vsdev->uart_frame != rx_vsdev->uart_frame)) {
			/*
			 * Emulate data sent but not received due to
			 * mismatched baudrate/framing.
			 */
			pr_debug("mismatched serial port settings!\n");
			tx_vsdev->icount.tx++;
			return count;
		}
	} else {
		/* Loop back */
		tty_to_write = tty;
		rx_vsdev = tx_vsdev;
	}

	if (tty_to_write) {
		if ((tty_to_write->termios.c_cflag & CSIZE) == CS8) {
			data = (unsigned char *)buf;
		} else {
			data = kcalloc(count, sizeof(char), GFP_KERNEL);
			if (!data)
				return -ENOMEM;

			/* Emulate correct number of data bits */
			switch (tty_to_write->termios.c_cflag & CSIZE) {
			case CS7:
				for (x = 0; x < count; x++)
					data[x] = buf[x] & 0x7F;
				break;
			case CS6:
				for (x = 0; x < count; x++)
					data[x] = buf[x] & 0x3F;
				break;
			case CS5:
				for (x = 0; x < count; x++)
					data[x] = buf[x] & 0x1F;
				break;
			default:
				data = (unsigned char *)buf;
			}
		}

		tty_insert_flip_string(tty_to_write->port, data, count);
		tty_flip_buffer_push(tty_to_write->port);
		tx_vsdev->icount.tx++;
		rx_vsdev->icount.rx++;

		if (data != buf)
			kfree(data);
	} else {
		/*
		 * Other end is still not opened, emulate transmission from
		 * local end but don't make other end receive it as is the
		 * case in real world.
		 */
		tx_vsdev->icount.tx++;
	}

	return count;
}

/* Invoked by tty core to transmit single data byte. */
static int vs_put_char(struct tty_struct *tty, unsigned char ch)
{
	unsigned char data;
	struct tty_struct *tty_to_write;
	struct vs_dev *rx_vsdev;
	struct vs_dev *tx_vsdev = idr_find(&db, tty->index);

	if (tx_vsdev->tx_paused || !tty || tty->stopped || tty->hw_stopped)
		return 0;

	if (tx_vsdev->is_break_on == 1)
		return -EIO;

	if (tx_vsdev->faulty_cable == 1)
		return 1;

	if (tty->index != tx_vsdev->peer_index) {
		tty_to_write = tx_vsdev->peer_tty;
		rx_vsdev = idr_find(&db, tx_vsdev->peer_index);
		if ((tx_vsdev->baud != rx_vsdev->baud) ||
			(tx_vsdev->uart_frame != rx_vsdev->uart_frame)) {
			tx_vsdev->icount.tx++;
			return 1;
		}
	} else {
		tty_to_write = tty;
		rx_vsdev = tx_vsdev;
	}

	if (tty_to_write != NULL) {
		switch (tty_to_write->termios.c_cflag & CSIZE) {
		case CS8:
			data = ch;
			break;
		case CS7:
			data = ch & 0x7F;
			break;
		case CS6:
			data = ch & 0x3F;
			break;
		case CS5:
			data = ch & 0x1F;
			break;
		default:
			data = ch;
		}
		tty_insert_flip_string(tty_to_write->port, &data, 1);
		tty_flip_buffer_push(tty_to_write->port);
		tx_vsdev->icount.tx++;
		rx_vsdev->icount.rx++;
	} else {
		tx_vsdev->icount.tx++;
	}

	return 1;
}

/*
 * Flush the data out of serial port. This driver immediately
 * pushes data into receiver's tty buffer hence do nothing here.
 */
static void vs_flush_chars(struct tty_struct *tty)
{
	pr_debug("flushing the chars!\n");
}

/*
 * Discard the internal output buffer for this tty device. Typically
 * it may be called when executing IOCTL TCOFLUSH, closing the
 * serial port, when break is received in input stream (flushing
 * is configured) or when hangup occurs.
 *
 * On the other hand, when TCIFLUSH IOCTL is invoked, tty flip buffer
 * and line discipline queue gets emptied without involvement of tty
 * driver. The driver is generally expected not to keep data but send
 * it to tty layer as soon as possible when it receives data.
 *
 * As this driver immediately pushes data into receiver's tty buffer
 * hence do nothing here.
 */
static void vs_flush_buffer(struct tty_struct *tty)
{
	pr_debug("flushing the buffer!\n");
}

/* Provides information as a repsonse to TIOCGSERIAL IOCTL */
static int vs_get_serinfo(struct tty_struct *tty, unsigned long arg)
{
	int ret;
	struct serial_struct info;
	struct vs_dev *local_vsdev = idr_find(&db, tty->index);
	struct serial_struct serial = local_vsdev->serial;

	if (!arg)
		return -EFAULT;

	memset(&info, 0, sizeof(info));

	info.type		    = PORT_UNKNOWN;
	info.line		    = serial.line;
	info.port		    = tty->index;
	info.irq			= 0;
	info.flags		    = tty->port->flags;
	info.xmit_fifo_size = 0;
	info.baud_base	    = 0;
	info.close_delay	= tty->port->close_delay;
	info.closing_wait   = tty->port->closing_wait;
	info.custom_divisor = 0;
	info.hub6		    = 0;
	info.io_type		= SERIAL_IO_MEM;

	ret = copy_to_user((void __user *)arg, &info,
				sizeof(struct serial_struct));

	return ret ? -EFAULT : 0;
}

/* Returns number of bytes that can be queued to this device now */
static int vs_write_room(struct tty_struct *tty)
{
	struct vs_dev *tx_vsdev = idr_find(&db, tty->index);

	if (tx_vsdev->tx_paused || !tty ||
			tty->stopped || tty->hw_stopped)
		return 0;

	return 2048;
}

/*
 * Invoked when serial terminal settings are chaged. The old_termios
 * contains currently active settings and tty->termios contains new
 * settings to be applied.
 */
static void vs_set_termios(struct tty_struct *tty,
				struct ktermios *old_termios)
{
	u32 baud;
	int uart_frame_settings;
	unsigned int mask = TIOCM_DTR;
	struct vs_dev *local_vsdev = idr_find(&db, tty->index);

	mutex_lock(&local_vsdev->lock);

	/*
	 * Typically B0 is used to terminate the connection.
	 * Drop RTS and DTR.
	 */
	if ((tty->termios.c_cflag & CBAUD) == B0) {
		vs_update_modem_lines(tty, 0, TIOCM_DTR | TIOCM_RTS);
		mutex_unlock(&local_vsdev->lock);
		return;
	}

	/* If coming out of B0, raise DTR and RTS. This might get
	 * overridden in next steps. Applications like minicom when
	 * opens a serial port, may drop speed to B0 and then back
	 * to normal speed again.
	 */
	if (!old_termios || (old_termios->c_cflag & CBAUD) == B0) {
		if (!(tty->termios.c_cflag & CRTSCTS) ||
				!test_bit(TTY_THROTTLED, &tty->flags)) {
			mask |= TIOCM_RTS;
			vs_update_modem_lines(tty, mask, 0);
		}
	}

	baud = tty_get_baud_rate(tty);
	if (!baud)
		baud = 9600;

	tty_encode_baud_rate(tty, baud, baud);

	local_vsdev->baud = baud;

	uart_frame_settings = 0;
	if (tty->termios.c_cflag & CRTSCTS) {
		uart_frame_settings |= VS_CRTSCTS;
	} else if ((tty->termios.c_iflag & IXON) ||
					(tty->termios.c_iflag & IXOFF)) {
		uart_frame_settings |= VS_XON;
	} else {
		uart_frame_settings |= VS_NONE;
	}

	switch (tty->termios.c_cflag & CSIZE) {
	case CS8:
		uart_frame_settings |= VS_DATA_8;
		break;
	case CS7:
		uart_frame_settings |= VS_DATA_7;
		break;
	case CS6:
		uart_frame_settings |= VS_DATA_6;
		break;
	case CS5:
		uart_frame_settings |= VS_DATA_5;
		break;
	default:
		uart_frame_settings |= VS_DATA_8;
	}

	if (tty->termios.c_cflag & CSTOPB)
		uart_frame_settings |= VS_STOP_2;
	else
		uart_frame_settings |= VS_STOP_1;

	if (tty->termios.c_cflag & PARENB) {
		if (tty->termios.c_cflag & CMSPAR) {
			if (tty->termios.c_cflag & PARODD)
				uart_frame_settings |= VS_PARITY_MARK;
			else
				uart_frame_settings |= VS_PARITY_SPACE;
		} else {
			if (tty->termios.c_cflag & PARODD)
				uart_frame_settings |= VS_PARITY_ODD;
			else
				uart_frame_settings |= VS_PARITY_EVEN;
		}
	} else {
		uart_frame_settings |= VS_PARITY_NONE;
	}

	local_vsdev->uart_frame = uart_frame_settings;

	mutex_unlock(&local_vsdev->lock);
}

/*
 * Returns the number of bytes in device's output queue. This is
 * invoked when TIOCOUTQ IOCTL is executed or by tty core as and
 * when required. Because we all push all data into receiver's
 * end tty buffer, always return 0 here.
 */
static int vs_chars_in_buffer(struct tty_struct *tty)
{
	return 0;
}

/*
 * Based on the number od interrupts check if any of the signal
 * line has changed.
 */
static int vs_check_msr_delta(struct tty_struct *tty,
		struct vs_dev *local_vsdev, unsigned long mask,
		struct async_icount *prev)
{
	int delta;
	struct async_icount now;

	/*
	 * Use tty-port initialised flag to detect all hangups
	 * including the disconnect(device destroy) event.
	 */
	if (!test_bit(ASYNCB_INITIALIZED, &tty->port->flags))
		return 1;

	mutex_lock(&local_vsdev->lock);
	now = local_vsdev->icount;
	mutex_unlock(&local_vsdev->lock);
	delta = ((mask & TIOCM_RNG && prev->rng != now.rng) ||
			 (mask & TIOCM_DSR && prev->dsr != now.dsr) ||
			 (mask & TIOCM_CAR && prev->dcd != now.dcd) ||
			 (mask & TIOCM_CTS && prev->cts != now.cts));

	*prev = now;
	return delta;
}

/* Sleeps until at-least one of the modem lines changes */
static int vs_wait_change(struct tty_struct *tty, unsigned long mask)
{
	int ret;
	struct async_icount prev;
	struct vs_dev *local_vsdev = idr_find(&db, tty->index);

	mutex_lock(&local_vsdev->lock);

	local_vsdev->waiting_msr_chg = 1;
	prev = local_vsdev->icount;

	mutex_unlock(&local_vsdev->lock);

	ret = wait_event_interruptible(tty->port->delta_msr_wait,
			vs_check_msr_delta(tty, local_vsdev, mask, &prev));

	local_vsdev->waiting_msr_chg = 0;

	if (!ret && !test_bit(ASYNCB_INITIALIZED, &tty->port->flags))
		ret = -EIO;

	return ret;
}

/* Execute IOCTL commands */
static int vs_ioctl(struct tty_struct *tty,
				unsigned int cmd, unsigned long arg)
{
	switch (cmd) {
	case TIOCGSERIAL:
		return vs_get_serinfo(tty, arg);
	case TIOCMIWAIT:
		return vs_wait_change(tty, arg);
	}

	return -ENOIOCTLCMD;
}

/*
 * Invoked when tty layer's input buffers are about to get full.
 *
 * When using RTS/CTS flow control, when RTS line is de-asserted,
 * interrupt will be generated in hardware. The interrupt handler
 * will raise a flag to indicate transmission should be stopped.
 * This is achieved in this driver through tx_paused variable.
 */
static void vs_throttle(struct tty_struct *tty)
{
	struct vs_dev *local_vsdev = idr_find(&db, tty->index);
	struct vs_dev *remote_vsdev = idr_find(&db, local_vsdev->peer_index);

	if (tty->termios.c_cflag & CRTSCTS) {
		mutex_lock(&local_vsdev->lock);
		remote_vsdev->tx_paused = 1;
		vs_update_modem_lines(tty, 0, TIOCM_RTS);
		mutex_unlock(&local_vsdev->lock);
	} else if ((tty->termios.c_iflag & IXON) ||
				(tty->termios.c_iflag & IXOFF)) {
		vs_put_char(tty, STOP_CHAR(tty));
	} else {
		/* do nothing */
	}
}

/*
 * Invoked when the tty layer's input buffers have been emptied out,
 * and it now can accept more data. Throttle/Unthrottle is about
 * notifying remote end to start or stop data as per the currently
 * active flow control. On the other hand, Start/Stop is about what
 * action to take at local end itself to start or stop data as per
 * the currently active flow control.
 */
static void vs_unthrottle(struct tty_struct *tty)
{
	struct vs_dev *local_vsdev = idr_find(&db, tty->index);
	struct vs_dev *remote_vsdev = idr_find(&db, local_vsdev->peer_index);

	if (tty->termios.c_cflag & CRTSCTS) {
		/* hardware (RTS/CTS) flow control */
		mutex_lock(&local_vsdev->lock);
		remote_vsdev->tx_paused = 0;
		vs_update_modem_lines(tty, TIOCM_RTS, 0);
		mutex_unlock(&local_vsdev->lock);

		if (remote_vsdev->own_tty && remote_vsdev->own_tty->port)
			tty_port_tty_wakeup(remote_vsdev->own_tty->port);
	} else if ((tty->termios.c_iflag & IXON) ||
				(tty->termios.c_iflag & IXOFF)) {
		/* software flow control */
		vs_put_char(tty, START_CHAR(tty));
	} else {
		/* do nothing */
	}
}

/*
 * Invoked when this driver should stop sending data for example
 * as a part of flow control mechanism.
 *
 * Line discipline n_tty calls this function if this device uses
 * software flow control and an XOFF character is received from
 * other end.
 */
static void vs_stop(struct tty_struct *tty)
{
	struct vs_dev *local_vsdev = idr_find(&db, tty->index);

	mutex_lock(&local_vsdev->lock);
	local_vsdev->tx_paused = 1;
	mutex_unlock(&local_vsdev->lock);
}

/*
 * Invoked when this driver should start sending data for example
 * as a part of flow control mechanism.
 *
 * Line discipline n_tty calls this function if this device uses
 * software flow control and an XON character is received from
 * other end.
 */
static void vs_start(struct tty_struct *tty)
{
	struct vs_dev *local_vsdev = idr_find(&db, tty->index);

	mutex_lock(&local_vsdev->lock);
	local_vsdev->tx_paused = 0;
	mutex_unlock(&local_vsdev->lock);

	if (tty && tty->port)
		tty_port_tty_wakeup(tty->port);
}

/*
 * Obtain the modem status bits for the given tty device. Invoked
 * typically when TIOCMGET IOCTL is executed on the given
 * tty device.
 */
static int vs_tiocmget(struct tty_struct *tty)
{
	int status, msr_reg, mcr_reg;
	struct vs_dev *local_vsdev = idr_find(&db, tty->index);

	mutex_lock(&local_vsdev->lock);
	mcr_reg = local_vsdev->mcr_reg;
	msr_reg = local_vsdev->msr_reg;
	mutex_unlock(&local_vsdev->lock);

	status = ((mcr_reg & VS_MCR_DTR)  ? TIOCM_DTR  : 0) |
			 ((mcr_reg & VS_MCR_RTS)  ? TIOCM_RTS  : 0) |
			 ((mcr_reg & VS_MCR_LOOP) ? TIOCM_LOOP : 0) |
			 ((msr_reg & VS_MSR_DCD)  ? TIOCM_CAR  : 0) |
			 ((msr_reg & VS_MSR_RI)   ? TIOCM_RI   : 0) |
			 ((msr_reg & VS_MSR_CTS)  ? TIOCM_CTS  : 0) |
			 ((msr_reg & VS_MSR_DSR)  ? TIOCM_DSR  : 0);

	return status;
}

/*
 * Set the modem status bits. Invoked typically when TIOCMSET IOCTL
 * is executed on the given tty device.
 */
static int vs_tiocmset(struct tty_struct *tty,
				unsigned int set, unsigned int clear)
{
	int ret;
	struct vs_dev *local_vsdev = idr_find(&db, tty->index);

	mutex_lock(&local_vsdev->lock);
	ret = vs_update_modem_lines(tty, set, clear);
	mutex_unlock(&local_vsdev->lock);

	return ret;
}

/*
 * Unconditionally assert/de-assert break condition of the given
 * tty device.
 */
static int vs_break_ctl(struct tty_struct *tty, int break_state)
{
	struct tty_struct *tty_to_write;
	struct vs_dev *brk_rx_vsdev;
	struct vs_dev *brk_tx_vsdev = idr_find(&db, tty->index);

	if (tty->index != brk_tx_vsdev->peer_index) {
		tty_to_write = brk_tx_vsdev->peer_tty;
		brk_rx_vsdev = idr_find(&db, brk_tx_vsdev->peer_index);
	} else {
		tty_to_write = tty;
		brk_rx_vsdev = brk_tx_vsdev;
	}

	mutex_lock(&brk_tx_vsdev->lock);

	if (break_state != 0) {
		if (brk_tx_vsdev->is_break_on == 1)
			return 0;

		brk_tx_vsdev->is_break_on = 1;
		if (tty_to_write != NULL) {
			tty_insert_flip_char(tty_to_write->port, 0, TTY_BREAK);
			tty_flip_buffer_push(tty_to_write->port);
			brk_rx_vsdev->icount.brk++;
		}
	} else {
		brk_tx_vsdev->is_break_on = 0;
	}

	mutex_unlock(&brk_tx_vsdev->lock);
	return 0;
}

/*
 * Invoked by tty layer to inform this driver that it should hangup
 * the tty device (lower modem control lines after last process
 * using tty devices closes the device or exited).
 *
 * Drop DTR/RTS if HUPCL is set. This causes any attached modem to
 * hang up the line.
 *
 * On the receiving end, if CLOCAL bit is set, DCD will be ignored
 * otherwise SIGHUP may be generated to indicate a line disconnect
 * event.
 */
static void vs_hangup(struct tty_struct *tty)
{
	struct vs_dev *local_vsdev = idr_find(&db, tty->index);

	mutex_lock(&local_vsdev->lock);

	/* Drops reference to tty */
	tty_port_hangup(tty->port);

	if (tty && C_HUPCL(tty))
		vs_update_modem_lines(tty, 0, TIOCM_DTR | TIOCM_RTS);

	mutex_unlock(&local_vsdev->lock);
	pr_debug("hanged up!\n");
}

/*
 * Return number of interrupts as response to TIOCGICOUNT IOCTL.
 * Both 1->0 and 0->1 transitions are counted, except for RI;
 * where only 0->1 transitions are accounted.
 */
static int vs_get_icount(struct tty_struct *tty,
				struct serial_icounter_struct *icount)
{
	struct async_icount cnow;
	struct vs_dev *local_vsdev = idr_find(&db, tty->index);

	mutex_lock(&local_vsdev->lock);
	cnow = local_vsdev->icount;
	mutex_unlock(&local_vsdev->lock);

	icount->cts = cnow.cts;
	icount->dsr = cnow.dsr;
	icount->rng = cnow.rng;
	icount->dcd = cnow.dcd;
	icount->tx = cnow.tx;
	icount->rx = cnow.rx;
	icount->frame = cnow.frame;
	icount->parity = cnow.parity;
	icount->overrun = cnow.overrun;
	icount->brk = cnow.brk;
	icount->buf_overrun = cnow.buf_overrun;

	return 0;
}

/*
 * Invoked by tty layer to execute TCIOFF and TCION IOCTL commands
 * generally because user space process called tcflow() function.
 * It send a high priority character to the tty device end even if
 * stopped.
 *
 * If this function (send_xchar) is defined by tty device driver,
 * tty core will call this function. If it is not specified then
 * tty core will first instruct this driver to start transmission
 * (start()) and then invoke write() of this driver passing character
 * to be written and then it will call stop() of this driver.
 */
static void vs_send_xchar(struct tty_struct *tty, char ch)
{
	int was_paused;
	struct vs_dev *local_vsdev = idr_find(&db, tty->index);

	was_paused = local_vsdev->tx_paused;
	if (was_paused)
		local_vsdev->tx_paused = 0;

	vs_put_char(tty, ch);
	if (was_paused)
		local_vsdev->tx_paused = 1;
}

/*
 * Invoked by tty core in response to tcdrain() call. As this driver
 * drains on write() itself, we return immediately from here.
 */
static void vs_wait_until_sent(struct tty_struct *tty, int timeout)
{
	pr_debug("returned wait until sent!\n");
}

/*
 * Unregister tty device specified by minor number ownidx
 * and remove sysfs files associate with it. Caller must
 * hold card lock. First tty must be released and then port.
 *
 * It is common to reset environment before launching new test
 * suite during automated testing. To support this we allow
 * removing devices even when it was created using DT as of
 * now till we find any valid reason not do so.
 */
static void vs_unreg_one_dev(int ownidx, struct vs_dev *vsdev)
{
	struct tty_struct *tty;

	//sysfs_remove_group(&vsdev->device->kobj, &vs_info_attr_grp); TODO

	if (vsdev->own_tty && vsdev->own_tty->port) {
		tty = tty_port_tty_get(vsdev->own_tty->port);
		if (tty) {
			tty_vhangup(tty);
			tty_kref_put(tty);
		}
	}

	tty_unregister_device(ttyvs_driver, ownidx);
}

/*
 * Destroy a virtual tty device specified by the given index.
 * Whether IDR id will be freed or not is specified by the
 * caller through free_idr.
 */
static int vs_del_specific_devs(int ownidx, int free_idr)
{
	struct vs_dev *vsdev1, *vsdev2;

	/*
	 * If user just created configfs item but did not populated valid
	 * index, device will not exist, so bail out early.
	 */
	vsdev1 = idr_find(&db, ownidx);
	if (!vsdev1)
		return 0;

	vs_unreg_one_dev(ownidx, vsdev1);

	/* If this device is part of a null modem, delete peer also */
	if (vsdev1->own_index != vsdev1->peer_index) {
		vsdev2 = idr_find(&db, vsdev1->peer_index);
		if (vsdev2) {
			vs_unreg_one_dev(vsdev2->own_index, vsdev2);
			if (free_idr)
				idr_remove(&db, vsdev2->own_index);
			kfree(vsdev2);
		}
	}

	if (free_idr)
		idr_remove(&db, ownidx);
	kfree(vsdev1);

	return 0;
}

/*
 * Destroy all tty devices created, mark all the indexes as
 * available for allocation; reset IDR for re-use.
 */
static void vs_del_all_devs(void)
{
	int x;
	struct vs_dev *vsdev;

	mutex_lock(&card_lock);

	idr_for_each_entry(&db, vsdev, x)
		vs_del_specific_devs(vsdev->own_index, 0);

	idr_destroy(&db);

	mutex_unlock(&card_lock);
}

/*
 * Allocate per device private data (vsdev) for this driver, register
 * with tty core and create custom sysfs nodes for emulating serial
 * port events. Caller should hold card lock.
 */
static int vs_alloc_reg_one_dev(int oidx, int pidx, int rtsmap,
			int dtrmap, int dtropn)
{
	int ret, id;
	struct vs_dev *vsdev;
	struct device *dev;

	/* Allocate and init virtual tty device's private data */
	vsdev = kcalloc(1, sizeof(struct vs_dev), GFP_KERNEL);
	if (!vsdev)
		return -ENOMEM;

	id = idr_alloc(&db, vsdev, oidx, oidx + 1, GFP_KERNEL);
	if (id < 0) {
		ret = id;
		goto fail_id;
	}

	vsdev->own_tty = NULL;
	vsdev->peer_tty = NULL;
	vsdev->own_index = oidx;
	vsdev->peer_index =  pidx;
	vsdev->rts_mappings = rtsmap;
	vsdev->dtr_mappings = dtrmap;
	vsdev->set_odtr_at_open = dtropn;
	vsdev->msr_reg = 0;
	vsdev->mcr_reg = 0;
	vsdev->waiting_msr_chg = 0;
	vsdev->tx_paused = 0;
	vsdev->faulty_cable = 0;
	mutex_init(&vsdev->lock);

	/* Register with tty core with specific minor number */
	dev = tty_register_device(ttyvs_driver, oidx, NULL);
	if (!dev) {
		ret = -ENOMEM;
		goto fail_reg;
	}

	vsdev->device = dev;
	dev_set_drvdata(dev, vsdev);

	/* Create custom sysfs files for this device for events 
	ret = sysfs_create_group(&dev->kobj, &vs_info_attr_grp);
	if (ret)
		goto fail_sysfs;*/ //TODO

	return 0;

fail_sysfs:
	tty_unregister_device(ttyvs_driver, oidx);
fail_reg:
	idr_remove(&db, id);
fail_id:
	kfree(vsdev);

	return ret;
}

/*
 * Extract pin mappings from local to remote tty devices.
 * The map contains bits setted by user. Returns 0 on success
 * or negative error code on error. The *mapping will contain
 * pin connections (bit map as used by this driver) when this
 * function returns.
 */
static int vs_extract_pin_mapping(int usrval, int *mapping)
{
	if (usrval > (VS_CON_CTS | VS_CON_DCD | VS_CON_DSR | VS_CON_RI))
		return -EINVAL;

	/* No pin connections by-default */
	*mapping = 0;

	if ((usrval & VS_CON_CTS) == VS_CON_CTS)
		*mapping |= VS_CON_CTS;

	if ((usrval & VS_CON_DCD) == VS_CON_DCD)
		*mapping |= VS_CON_DCD;

	if ((usrval & VS_CON_DSR) == VS_CON_DSR)
		*mapping |= VS_CON_DSR;

	if ((usrval & VS_CON_RI) == VS_CON_RI)
		*mapping |= VS_CON_RI;

	return 0;
}

/*
 * The devtyp is 1 for null modem and 0 for loop-back. We extract
 * user supplied information, validate it and convert it as
 * required by this driver to create a device.
 */
static int vs_extract_dev_param_cfs(const struct vs_cfs_dev_info *di,
			unsigned int *idx, int *rtsmap, int *dtrmap,
			int *dtratopen, int devtyp)
{
	int ret;

	if (devtyp) {
		if (di->peeridx >= max_num_vs_devs)
			return -EINVAL;

		*idx = di->peeridx;

		ret = vs_extract_pin_mapping(di->prtsmap, rtsmap);
		if (ret)
			return ret;

		ret = vs_extract_pin_mapping(di->pdtrmap, rtsmap);
		if (ret)
			return ret;

		*dtratopen = di->pdtratopn ? 1 : 0;
	} else {
		if (di->ownidx >= max_num_vs_devs)
			return -EINVAL;

		*idx = di->ownidx;

		ret = vs_extract_pin_mapping(di->ortsmap, rtsmap);
		if (ret)
			return ret;

		ret = vs_extract_pin_mapping(di->odtrmap, rtsmap);
		if (ret)
			return ret;

		*dtratopen = di->odtratopn  ? 1 : 0;
	}

	return 0;
}

/* Converts pin mappings from dt node to this driver specific bit map */
static int vs_parse_dt_get_map(const struct device_node *np,
				const char *prop, int *mapping)
{
	int x, ret, num_map;
	int val[4];

	/*
	 * If the RTS/DTR pin is unconnected (property doesn't exist)
	 * set mapping to 0 and return success.
	 */
	ret = of_property_count_u32_elems(np, prop);
	if (ret < 0) {
		if (ret == -EINVAL) {
			*mapping = 0;
			return 0;
		}
		return ret;
	}

	/*
	 * A given pin can be connected to 1,6,8,9 pins. Therefore if
	 * more then 4 mappings are defined in DT, ignore it.
	 */
	num_map = ret;
	if (ret > 4)
		num_map = 4;

	ret = of_property_read_u32_array(np, prop, val, num_map);
	if (ret < 0)
		return ret;

	*mapping = 0;
	for (x = 0; x < num_map; x++) {
		switch (val[x]) {
		case 8:
			*mapping |= VS_CON_CTS;
			break;
		case 1:
			*mapping |= VS_CON_DCD;
			break;
		case 6:
			*mapping |= VS_CON_DSR;
			break;
		case 9:
			*mapping |= VS_CON_RI;
			break;
		default:
			return -EINVAL;
		}
	}

	return 0;
}

/*
 * Extract index of device, RTS mappings, DTR mappings and
 * whether to assert DTR at device open or not from dt node.
 */
static int vs_extract_dev_param_dt(const struct device_node *np,
			unsigned int *idx, int *rtsmap, int *dtrmap,
			int *dtratopen, int exclude)
{
	int ret;

	ret = of_property_read_u32(np, "dev-num", idx);
	if (ret)
		return ret;

	if (*idx >= max_num_vs_devs)
		return -EINVAL;

	ret = vs_parse_dt_get_map(np, "rtsmap", rtsmap);
	if (ret)
		return ret;

	ret = vs_parse_dt_get_map(np, "dtrmap", dtrmap);
	if (ret)
		return ret;

	*dtratopen = of_property_read_bool(np,
						"set-dtr-at-open") ? 1 : 0;

	return 0;
}

/*
 * Create a loop-back style device:
 *
 * 0. Information about device parameters can come through either
 *    configFS node or device-tree node.
 * 1. Decide index to use; the number is specified by user. If the
 *    given index is used already through error.
 * 2. Extract RTS and DTR mappings. A pin can map to pin numbers
 *    1,6,8,9 only or might be un-connected. Through error if
 *    invalid mapping is given.
 * 3. Find if DTR should be asserted when tty device is opened or
 *    not.
 * 4. Allocate and initialize 'struct vs_dev' instance with info
 *    from steps 1,2 & 3.
 * 5. Register one tty device with tty core and associate this tty
 *    device with vsdev instance from step 4.
 * 6. Create custom sysfs nodes to emulate serial port events for
 *    this device.
 */
static int vs_add_lb(const struct vs_cfs_dev_info *di,
				const struct device_node *np)
{
	int ret, rtsmap, dtrmap, dtratopen;
	unsigned int idx;

	mutex_lock(&card_lock);

	if (di) {
		ret = vs_extract_dev_param_cfs(di, &idx, &rtsmap,
					&dtrmap, &dtratopen, 0);
	} else {
		ret = vs_extract_dev_param_dt(np, &idx, &rtsmap,
					&dtrmap, &dtratopen, -1);
	}
	if (ret)
		goto fail;

	ret = vs_alloc_reg_one_dev(idx, idx, rtsmap, dtrmap, dtratopen);
	if (ret)
		goto fail;

fail:
	mutex_unlock(&card_lock);
	return ret;
}

/*
 * Create a null-modem style pair of devices:
 *
 * Steps are same as for creating loop-back style device except,
 * we create both the devices on success or none of them on error.
 */
static int vs_add_nm(const struct vs_cfs_dev_info *di,
				const struct device_node *np1,
				const struct device_node *np2)
{
	int ret, rtsmap1, dtrmap1, dtratopen1;
	int rtsmap2, dtrmap2, dtratopen2;
	unsigned int idx1, idx2;

	mutex_lock(&card_lock);

	if (di) {
		ret = vs_extract_dev_param_cfs(di, &idx1, &rtsmap1, &dtrmap1,
				&dtratopen1, 0);
		if (ret)
			goto out;

		ret = vs_extract_dev_param_cfs(di, &idx2, &rtsmap2, &dtrmap2,
				&dtratopen2, 1);
	} else {
		ret = vs_extract_dev_param_dt(np1, &idx1, &rtsmap1,
					&dtrmap1, &dtratopen1, -1);
		if (ret)
			goto out;

		ret = vs_extract_dev_param_dt(np2, &idx2, &rtsmap2,
					&dtrmap2, &dtratopen2, idx1);
	}
	if (ret)
		goto out;

	ret = vs_alloc_reg_one_dev(idx1, idx2, rtsmap1, dtrmap1, dtratopen1);
	if (ret)
		goto out;

	ret = vs_alloc_reg_one_dev(idx2, idx1, rtsmap2, dtrmap2, dtratopen2);
	if (ret)
		vs_del_specific_devs(idx1, 1);

out:
	mutex_unlock(&card_lock);
	return ret;
}

static const struct tty_operations vs_serial_ops = {
	.install	     = vs_install,
	.cleanup	     = vs_cleanup,
	.open	         = vs_open,
	.close	         = vs_close,
	.write	         = vs_write,
	.put_char	     = vs_put_char,
	.flush_chars     = vs_flush_chars,
	.write_room      = vs_write_room,
	.chars_in_buffer = vs_chars_in_buffer,
	.ioctl	         = vs_ioctl,
	.set_termios     = vs_set_termios,
	.throttle	     = vs_throttle,
	.unthrottle      = vs_unthrottle,
	.stop	         = vs_stop,
	.start	         = vs_start,
	.hangup	         = vs_hangup,
	.break_ctl       = vs_break_ctl,
	.flush_buffer    = vs_flush_buffer,
	.wait_until_sent = vs_wait_until_sent,
	.send_xchar      = vs_send_xchar,
	.tiocmget	     = vs_tiocmget,
	.tiocmset	     = vs_tiocmset,
	.get_icount      = vs_get_icount,
};

static int vs_register_with_tty_core(void)
{
	int ret;

	/* Initialize and register this driver with tty core */
	ttyvs_driver = tty_alloc_driver(max_num_vs_devs, 0);
	if (IS_ERR(ttyvs_driver))
		return PTR_ERR(ttyvs_driver);

	ttyvs_driver->owner = THIS_MODULE;
	ttyvs_driver->driver_name = "ttyvs";
	ttyvs_driver->name = "ttyvs";
	ttyvs_driver->major = 0;
	ttyvs_driver->minor_start = 0;
	ttyvs_driver->type = TTY_DRIVER_TYPE_SERIAL;
	ttyvs_driver->subtype = SERIAL_TYPE_NORMAL;
	ttyvs_driver->flags = TTY_DRIVER_REAL_RAW
				| TTY_DRIVER_RESET_TERMIOS
				| TTY_DRIVER_DYNAMIC_DEV;
	ttyvs_driver->init_termios = tty_std_termios;
	ttyvs_driver->init_termios.c_cflag = B9600 | CS8 | CREAD | HUPCL;
	ttyvs_driver->init_termios.c_ispeed = 9600;
	ttyvs_driver->init_termios.c_ospeed = 9600;

	tty_set_operations(ttyvs_driver, &vs_serial_ops);

	ret = tty_register_driver(ttyvs_driver);
	if (ret)
		put_tty_driver(ttyvs_driver);

	return ret;
}

/*
 * Information passed through device tree is given more preference
 * then through module params. This parses all device nodes and
 * creates loop-back and null-modem ttyvsX devices in the process.
 */
static int ttyvs_device_probe(struct platform_device *pdev)
{
	int ret;
	u32 max_num;
	struct device_node *child, *peer_node;
	phandle peer;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;

	/*
	 * We register with tty core again only if maximum number of
	 * devices registered during module_init is changed by device
	 * tree.
	 */
	max_num = 0;
	ret = of_property_read_u32(np, "max-num-vs-devs", &max_num);
	if (!ret && (max_num != max_num_vs_devs)) {
		tty_unregister_driver(ttyvs_driver);
		put_tty_driver(ttyvs_driver);

		max_num_vs_devs = max_num;
		ret = vs_register_with_tty_core();
		if (ret)
			return ret;
	}

	/*
	 * If we fail to create any device emit error log and move to
	 * the next dt node.
	 */
	for_each_available_child_of_node(np, child) {
		if (of_node_test_and_set_flag(child, OF_POPULATED))
			continue;

		if (of_property_read_u32(child, "peer-dev", &peer)) {
			ret = vs_add_lb(NULL, child);
			if (ret) {
				pr_err("can't create lb %s %d\n",
						child->name, ret);
				continue;
			}
		} else {
			peer_node = of_find_node_by_phandle(peer);
			if (peer_node) {
				of_node_set_flag(peer_node, OF_POPULATED);
				ret = vs_add_nm(NULL, child, peer_node);
				if (ret) {
					pr_err("can't create nm %s <-> %s %d\n",
						child->name, peer_node->name,
						ret);
					continue;
				}
			} else {
				pr_err("can't find peer for %s %d\n",
						child->name, ret);
			}
		}
	}

	return 0;
}

static inline struct vs_cfs_dev_info *to_vs_dinfo(
				struct config_item *item)
{
	return container_of(to_config_group(item),
				struct vs_cfs_dev_info, grp);
}

#define VS_DEV_ATTR_WR_U8(_name) \
static ssize_t vs_dev_##_name##_store(struct config_item *item, \
		const char *page, size_t len) \
{                                     \
	u8 val;                           \
	int ret;                          \
	ret = kstrtou8(page, 0, &val);    \
	if (ret)                          \
		return ret;                   \
	to_vs_dinfo(item)->_name = val;   \
	return len;                       \
}                                     \
static ssize_t vs_dev_##_name##_show(struct config_item *item, \
		char *buf)                    \
{                                     \
	return snprintf(buf, PAGE_SIZE, "%u\n", to_vs_dinfo(item)->_name); \
}

#define VS_DEV_ATTR_WR_U16(_name)     \
static ssize_t vs_dev_##_name##_store(struct config_item *item, \
		const char *page, size_t len) \
{                                     \
	u16 val;                          \
	int ret;                          \
	ret = kstrtou16(page, 0, &val);   \
	if (ret)                          \
		return ret;                   \
	to_vs_dinfo(item)->_name = val;   \
	return len;                       \
}                                     \
static ssize_t vs_dev_##_name##_show(struct config_item *item, \
		char *buf)                     \
{                                      \
	return snprintf(buf, PAGE_SIZE, "%u\n", to_vs_dinfo(item)->_name); \
}

#define VS_DEV_ATTR_WR_STR(_name) \
static ssize_t vs_dev_##_name##_store(struct config_item *item, \
		const char *page, size_t len)     \
{                                         \
	char *devtype;                        \
	devtype = kstrdup(page, GFP_KERNEL);  \
	if (!devtype)                         \
		return -ENOMEM;                   \
	if (devtype[len - 1] == '\n')         \
		devtype[len - 1] = '\0';          \
	to_vs_dinfo(item)->devtype = devtype; \
	return len;                           \
}                                         \
static ssize_t vs_dev_##_name##_show(struct config_item *item, \
		char *buf)                        \
{                                         \
	return snprintf(buf, PAGE_SIZE, "%s\n", to_vs_dinfo(item)->devtype); \
}

/*
 * Once all parameters for the device has been set, this finally
 * creates the device.
 */
static ssize_t vs_dev_create_store(struct config_item *item,
		const char *page, size_t len)
{
	u8 val;
	int ret;
	struct vs_cfs_dev_info *di;

	ret = kstrtou8(page, 0, &val);
	if (ret)
		return ret;

	/* User must write 1 to this node create device */
	if (val != 1)
		return -EINVAL;

	di = to_vs_dinfo(item);

	/* devtype must be defined to proceed further */
	if (!di->devtype)
		return -EINVAL;

	if (strncmp(di->devtype, "lb", 2) == 0)
		ret = vs_add_lb(di, NULL);
	else if (strncmp(di->devtype, "nm", 2) == 0)
		ret = vs_add_nm(di, NULL, NULL);
	else
		return -EINVAL;

	if (ret)
		return ret;
	return len;
}

VS_DEV_ATTR_WR_STR(devtype)
VS_DEV_ATTR_WR_U16(ownidx)
VS_DEV_ATTR_WR_U16(peeridx)
VS_DEV_ATTR_WR_U8(ortsmap)
VS_DEV_ATTR_WR_U8(odtrmap)
VS_DEV_ATTR_WR_U8(odtratopn)
VS_DEV_ATTR_WR_U8(prtsmap)
VS_DEV_ATTR_WR_U8(pdtrmap)
VS_DEV_ATTR_WR_U8(pdtratopn)

CONFIGFS_ATTR(vs_dev_, devtype);
CONFIGFS_ATTR(vs_dev_, ownidx);
CONFIGFS_ATTR(vs_dev_, ortsmap);
CONFIGFS_ATTR(vs_dev_, odtrmap);
CONFIGFS_ATTR(vs_dev_, odtratopn);
CONFIGFS_ATTR(vs_dev_, peeridx);
CONFIGFS_ATTR(vs_dev_, prtsmap);
CONFIGFS_ATTR(vs_dev_, pdtrmap);
CONFIGFS_ATTR(vs_dev_, pdtratopn);
CONFIGFS_ATTR_WO(vs_dev_, create);

static struct configfs_attribute *vs_dev_attrs[] = {
	&vs_dev_attr_devtype,
	&vs_dev_attr_ownidx,
	&vs_dev_attr_ortsmap,
	&vs_dev_attr_odtrmap,
	&vs_dev_attr_odtratopn,
	&vs_dev_attr_peeridx,
	&vs_dev_attr_prtsmap,
	&vs_dev_attr_pdtrmap,
	&vs_dev_attr_pdtratopn,
	&vs_dev_attr_create,
	NULL,
};

static const struct config_item_type vs_cfs_root_type = {
	.ct_attrs = vs_dev_attrs,
	.ct_owner = THIS_MODULE,
};

static struct config_group *vs_cfs_grp_make(
			struct config_group *group,
			const char *name)
{
	struct vs_cfs_dev_info *di;

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di)
		return ERR_PTR(-ENOMEM);

	config_group_init_type_name(&di->grp, name, &vs_cfs_root_type);

	return &di->grp;
}

static void vs_cfs_grp_drop(struct config_group *group,
				struct config_item *item)
{
	struct vs_cfs_dev_info *di = to_vs_dinfo(item);

	mutex_lock(&card_lock);
	vs_del_specific_devs(di->ownidx, 1);
	mutex_unlock(&card_lock);

	kfree(di);
	config_item_put(item);
}

static struct configfs_group_operations vs_cfs_grp_ops = {
	.make_group     = &vs_cfs_grp_make,
	.drop_item      = &vs_cfs_grp_drop,
};

static const struct config_item_type vs_cfs_grp_type = {
	.ct_group_ops = &vs_cfs_grp_ops,
	.ct_owner = THIS_MODULE,
};

struct configfs_subsystem vs_cfs_subsys = {
	.su_group = {
		.cg_item = {
			.ci_namebuf = "ttyvs",
			.ci_type = &vs_cfs_grp_type,
		},
	},
	.su_mutex = __MUTEX_INITIALIZER(vs_cfs_subsys.su_mutex),
};

static const struct of_device_id ttyvs_dev_match_tbl[] = {
	{ .compatible = "ttyvs,virtual-uart-card" },
	{ }
};
MODULE_DEVICE_TABLE(of, ttyvs_dev_match_tbl);

static struct platform_driver ttyvs_platform_drv = {
	.probe		= ttyvs_device_probe,
	.driver		= {
		.name	= "ttyvs",
		.of_match_table = ttyvs_dev_match_tbl,
		.dev_groups	= ttyvs_groups,
	},
};

static int __init ttyvs_init(void)
{
	int ret;

	config_group_init(&vs_cfs_subsys.su_group);

	ret = configfs_register_subsystem(&vs_cfs_subsys);
	if (ret)
		return ret;

	ret = vs_register_with_tty_core();
	if (ret)
		goto fail_drv;

	/* Register as platform driver to handle device tree nodes */
	ret = platform_driver_register(&ttyvs_platform_drv);
	if (ret)
		goto fail_plat;

	pr_info("serial port null modem emulation driver\n");
	return 0;

fail_plat:
	tty_unregister_driver(ttyvs_driver);
	put_tty_driver(ttyvs_driver);

fail_drv:
	configfs_unregister_subsystem(&vs_cfs_subsys);

	return ret;
}

static void __exit ttyvs_exit(void)
{
	vs_del_all_devs();

	configfs_unregister_subsystem(&vs_cfs_subsys);
	platform_driver_unregister(&ttyvs_platform_drv);

	tty_unregister_driver(ttyvs_driver);
	put_tty_driver(ttyvs_driver);
}

module_init(ttyvs_init);
module_exit(ttyvs_exit);

/*
 * By default this driver supports upto 64 virtual devices. This
 * can be overridden through max_num_vs_devs module parameter or
 * through max-num-vs-devs device tree property.
 */
module_param(max_num_vs_devs, ushort, 0);
MODULE_PARM_DESC(max_num_vs_devs,
		"Maximum virtual tty devices to be supported");

MODULE_AUTHOR("Rishi Gupta <gupt21@gmail.com>");
MODULE_DESCRIPTION("Serial port null modem emulation driver");
MODULE_LICENSE("GPL v2");
