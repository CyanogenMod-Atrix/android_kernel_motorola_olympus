//#define AES1750_DEBUG 1
/*
 * drivers/input/aes1750.c - driver for AuthenTec aes1750
 *
 * Copyright (C) 2010 Motorola, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/* DESCRIPTION:
   This file implements the driver for the AuthenTec AES1750 fingerprint
   sensor.  The sensor supports fingerprint imaging as well as
   navigation control.  To this end, the driver acts as three devices:
      1. A SPI slave device to access the SPI bus.
      2. A miscellaneous device to provide file-operations to user-mode.
      3. An input device to provide navigation events to user-mode.
   This driver provides low-level, packet-based access to the sensor
   for a single user-mode application.

   HARDWARE INTERFACES:
   The sensor interfaces to the host via the SPI bus and an interrupt
   GPIO.  The sensor acts as a SPI slave device.  There is no explict
   reset GPIO.

   CONTEXTS:
   The driver code runs in the following contexts:
      1. An interrupt service routine (ISR).  The ISR performs no work
	 except to signal the IST (described next).
      2. An interrupt service thread (IST) that performs SPI reads
	 via calls to spi_sync().  This call blocks on a SPI bus master
	 thread (described next).
      3. SPI bus master thread that performs the actual SPI transaction.
	 This thread is used to sychronize the various SPI devices so
	 that only one device uses the SPI bus at a time.  This thread
	 is implemented in the standard SPI bus master code (e.g. spi.c).
      4. From user mode, via the file-operations: open, release, read,
	 write, and ioctl.
      5. From the kernel driver core via probe, remove, and optionally
	 suspend, and remove.
      6. Optionally from the early-suspend core via early-suspend and
	 early-resume.

    SEMAPHORES:
    The driver uses two semaphores:
      1. ist_semaphore: This semaphore signals that the IST should
	 read data.  It is signalled by the ISR.  The IST waits on it.
      2. read_semaphore: This semaphore signals that there is
	 received data available for the user-mode application.  It
	 is signalled by the IST.  The read file-operation,
	 aes1750_read(), waits on it.

    SUSPEND/RESUME:
    The driver supports early-suspend and early-resume via the compile
    flag, CONFIG_HAS_EARLYSUSPEND.  If this flag is not defined, then
    the driver supports the standard suspend and resume functions.

    When the driver receives a suspend notification, it will notify
    the application of the suspend by sending a signal (SIGUSR1).
    The driver will then reset the sensor, placing it in low-power mode.
    On resume, the driver will notify the application of the resume
    by sending a signal (SIGUSR2).  It is the job of the application
    to re-enable the sensor after a resume.

    SPI OPERATIONS:
    The SPI bus is, at all times, a full-duplex bus.  This complicates
    access to the device since writing data to the sensor simultaneously
    reads data from the sensor.  Likewise, whenever reading from the
    sensor it is necessary to send benign (no-op) data back.

    If the sensor is in a mode that allows it to send unsolicited data,
    then any data sent to the sensor must be carefully coordinated.
    Specifically, it must be buffered until the next receive.  The driver
    design uses a set of tx buffers to ensure that any pending transmit
    data is sent during a read (see TX BUFFERS below).

    Reads and writes to the sensor are performed in blocks (as packets
    or sets of packets) rather than as a stream of characters.  This
    improves performance by transferring blocks of data at a time rather
    than many single-byte transfers.  Block transfers, however, complicate
    the interface between driver and user-mode application.  It is always
    necessary for the application to inform the driver of the size of the
    next incoming block before the sensor is ready with the data.  This
    is done via an IOCTL, AES1750_IOCTL_WR_RX_LEN.  This IOCTL tells the
    IST how many bytes to read during the next SPI access.  Setting the
    value of this IOCTL to 0 will prevent any more data from being read.
    Note that this can effectively shutdown the sensor by masking future
    sensor interrupts (see below).

    INTERRUPTS:
    The sensor's interrupt is configured to trigger on the rising edge.
    The sensor will generate interrupts very quickly (as fast as every
    millisecond).  Configuring the interrupt as level-sensitive would
    drain performance during imaging by scheduling many more ISRs than
    can be handled.

    However, triggering on the rising edge can has its pitfalls as well.
    If an interrupt fires, an ISR is scheduled, but the sensor is not
    serviced and the interrupt is not lowered, then the processor will no
    longer receive any more interrupts.  The only resolution is to reset
    the sensor which will automatically lower the interrupt.

    TX BUFFERS:
    Because the driver runs in multiple contexts and because calls to
    spi_sync() block, it is necessary to use multiple tx buffers.
    There are three tx buffers:
       1. tx_buf: This is the 'official' tx buffer and contains any
	  pending, outgoing data.  The length of valid data in
	  tx_buf is tx_len.
       2. write_tx_buf: This is a temporary copy of tx_buf used when
	  performing a write to the sensor.  When performing a write
	  the tx_buf is locked, copied to write_tx_buf, cleared, and
	  then unlocked.  After the write is complete, write_tx_buf is
	  reinitialized with no-op bytes.
       3. read_tx_buf: This is a temporary copy of tx_buf used when
	  performing a read from the sensor.  When performing a read
	  the tx_buf is locked, copied to read_tx_buf, cleared, and
	  then unlocked.  After the read is complete, read_tx_buf is
	  reinitialized with no-op bytes.

    RX MESSAGES:
    rx_msgs is a list of messages received from the sensor and
    available to be read by the user-mode application via the read
    file-operation, aes1750_read().  This list contains spi_messages.
    They are only allocated by aes1750_rx() and are typically
    released by aes1750_read().  They will also be released for
    various clean-up operations (e.g. aes1750_remove()).

    NAVIGATION:
    The sensor can send navigation events and mimic various devices:
    a dpad, trackball, mouse, etc.  The driver implements a pair of
    IOCTLs to tunnel input events from the user-mode application to
    the kernel.

    DEBUGGING:
    Go to /sys/module/aes1750/parameters and echo 1 > trace to enable
    calls to aes1750_trace.  You can also change max_bytes and number
    of bytes per line at run time.
    Most control packets are under 128 bytes,
    resets are under 256 bytes, and swipes under 600 bytes.

    PITFALLS:
    Since the calls to spi_sync() block, it is necessary to set
    state data at the appropriate times as well as to lock accesses to
    data structures.  The sensor is capable of processing data and
    signalling an response interrupt very quickly.  As a result, there
    are often context-switches at the tail end of calls to spi_sync()
    before it has returned to its caller.

    CORE FUNCTIONS:
    aes1750_rx()
    aes1750_tx()
    aes1750_thread()
    aes1750_read()
    aes1750_write()
*/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/freezer.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/signal.h>
#include <linux/spi/spi.h>
#include <mach/gpio.h>

/* Identifiers */
#define AES_SPI_DRIVER_NAME ("aes1750")
#define AES_MISC_DRIVER_NAME AES_SPI_DRIVER_NAME
#define AES_INPUT_DRIVER_NAME AES_SPI_DRIVER_NAME

/* States */
/* The device has not been initialized, or it has been uninitialized. */
#define AES1750_STATE_DISABLED (0)
/* The device is initialized, but not opened. */
#define AES1750_STATE_CLOSED   (1)
/* The device is open, but rx_len has not been set. */
#define AES1750_STATE_IDLE     (2)
/* The device is open, rx_len is set, and data is NOT being transmitted. */
#define AES1750_STATE_READY    (3)
/* The device is open, rx_len is set, and data is being transmitted. */
#define AES1750_STATE_ACTIVE   (4)

/* Buffer sizes */
#define AES1750_MAX_BUFFER_SIZE (1024)
#define AES1750_RESET_BUFFER_SIZE (261)
#define AES1750_IDLE_BUFFER_SIZE (2)

#define AES1750_NOP (0xFF)

/* Hardware config */
#define AES1750_IRQ_FLAGS (IRQF_DISABLED|IRQF_TRIGGER_RISING)

/* IOCTLs */
#define AES1750_IOCTL_MAGIC ('x')

/* Signals */
#define AES1750_SIGNAL_SUSPEND	(SIGUSR1)
#define AES1750_SIGNAL_RESUME	(SIGUSR2)

/* Read and write the rx_len setting.  This setting is used to tell the
   interrupt service routine how many bytes to read at the next interrupt. */
#define AES1750_IOCTL_RD_RX_LEN _IOR(AES1750_IOCTL_MAGIC, 1, __u32)
#define AES1750_IOCTL_WR_RX_LEN _IOW(AES1750_IOCTL_MAGIC, 1, __u32)

/* Read the number of received messages available.
   This is the number of calls to read() that will return data immediately. */
#define AES1750_IOCTL_RD_NUM_RX_MSGS _IOR(AES1750_IOCTL_MAGIC, 2, __u32)

/* Read the size (in bytes) of the next available received message.
   This is the maximum number of bytes to be returned by the next call to
   read().  If no received messages are availabl, returns 0. */
#define AES1750_IOCTL_RD_NUM_RX_MSG_LEN _IOR(AES1750_IOCTL_MAGIC, 3, __u32)

/* Call input_report_key(code, value).
   The high 32-bits is the code.  The low 32-bits is the value. */
#define AES1750_IOCTL_INPUT_KEY _IOW(AES1750_IOCTL_MAGIC, 4, __u64)

/* Call input_report_rel(x, y).
   The high 32-bits is the x.  The low 32-bits is the y. */
#define AES1750_IOCTL_INPUT_REL _IOW(AES1750_IOCTL_MAGIC, 5, __u64)

/* This ioctl tells the driver that a swipe occurred (any type of swipe) */
#define AES1750_IOCTL_SWIPE _IOW(AES1750_IOCTL_MAGIC, 6, __u64)

/* This is an ioctl with which am2server queries the driver about
suspend/resume state after receiving a signal */
#define AUTH_IOCTL_OS_SUSPENDSTATE _IOR(AES1750_IOCTL_MAGIC, 7, __u32)

#define AES1750_NIBBLE_TO_ASCII(n) ((n) > 9 ? (n)-10+'a' : (n)+'0')

#define aes1750_err(fmt,args...) pr_err("%s: "fmt, __func__, ##args)
#define aes1750_info(fmt,args...) pr_info("%s: "fmt, __func__, ##args)
#define aes1750_warn(fmt,args...) pr_warning("%s: "fmt, __func__, ##args)
#define aes1750_dbg(fmt,args...) pr_debug("%s: "fmt, __func__, ##args)

#define MODULE_NAME "aes1750"

#ifdef AES1750_DEBUG
/* Trace tx/rx activity */
static unsigned trace_txrx = 0;
module_param(trace_txrx, uint, 0664);
#define aes1750_trace_txrx(fmt,args...) if (trace_txrx) pr_info("%s: "fmt, __func__, ##args)

/* Trace upper layer read/write and ioctl requests */
static unsigned trace_request = 0;
module_param(trace_request, uint, 0664);
#define aes1750_trace_request(fmt,args...) if (trace_request) pr_info("%s: "fmt, __func__, ##args)

#define aes1750_print_buffer_txrx(buf,len) aes1750_print_buffer(__func__, trace_txrx, buf, len)
#define aes1750_print_buffer_request(buf,len) aes1750_print_buffer(__func__, trace_request, buf, len)

/* aes1750_print_buffer constants and macros */
static unsigned bytes_per_line = 256;
module_param(bytes_per_line, uint, 0664);
static unsigned max_bytes = 256;
module_param(max_bytes, uint, 0664);
#else
#define aes1750_trace_txrx(fmt,args...)
#define aes1750_trace_request(fmt,args...)
#define aes1750_print_buffer_txrx(buf,len)
#define aes1750_print_buffer_request(buf,len)
#endif

/* Logs the value of the interrupt pin. */
#define AES1750_LOG_GPIO_INT(f) \
{\
	int value;\
	value = gpio_get_value(aes1750->gpio_int);\
	aes1750_dbg(f "aes1750_int: %d\n", value);\
}

struct aes1750 {
	struct spi_device *spi;
	struct input_dev *input_dev;
	struct early_suspend early_suspend;

	/* Used to identify when the device is opened. */
	struct file *file;
	pid_t pid;

	spinlock_t lock;

	int gpio_int;
	struct semaphore ist_semaphore;
	struct semaphore read_semaphore;

	struct task_struct *thread;

	int state;

	int rx_len;
	struct list_head rx_msgs;

	int tx_len;
	unsigned char *tx_buf;

	unsigned char *read_tx_buf;
	unsigned char *write_tx_buf;

	unsigned char *reset_command;
	int reset_command_len;

	unsigned char idle_command[AES1750_IDLE_BUFFER_SIZE];

	atomic_t is_suspended;

	int swipe_value;

} *_aes1750;

static int aes1750_rx(struct aes1750 *aes1750);
static int aes1750_tx(struct aes1750 *aes1750);

static int aes1750_thread(void *data);
static irqreturn_t aes1750_irq(int irq, void *data);

static void aes1750_send_user_signal(struct aes1750 *aes1750, int signal);
static int aes1750_flush(struct aes1750 *aes1750);
static int aes1750_reset(struct aes1750 *aes1750);

static void aes1750_set_rx_len(struct aes1750 *aes1750, int rx_len);
static void aes1750_flush_rx_msgs(struct aes1750 *aes1750);
static void aes1750_uninit(struct aes1750 *aes1750);

static int aes1750_open(struct inode *, struct file *);
static int aes1750_release(struct inode *, struct file *);
static ssize_t aes1750_read(struct file *, char __user *, size_t, loff_t *);
static ssize_t aes1750_write(struct file *, const char __user *, size_t,
			     loff_t *);
static long aes1750_ioctl(struct file *, unsigned int, unsigned long);

static int __devinit aes1750_probe(struct spi_device *spi);
static int __devexit aes1750_remove(struct spi_device *spi);
static int aes1750_suspend(struct spi_device *spi, pm_message_t mesg);
static int aes1750_resume(struct spi_device *spi);

#if defined(CONFIG_HAS_EARLYSUSPEND)
static void aes1750_early_suspend(struct early_suspend *handler);
static void aes1750_late_resume(struct early_suspend *handler);
#endif

int __devinit aes1750_init_module(void);
void __devexit aes1750_cleanup_module(void);

static struct spi_driver aes1750_spi_driver = {
	.driver = {
		   .name = AES_SPI_DRIVER_NAME,
		   .bus = &spi_bus_type,
		   .owner = THIS_MODULE},
	.probe = aes1750_probe,
	.remove = __exit_p(aes1750_remove),
#if !defined(CONFIG_HAS_EARLYSUSPEND)
	.suspend = aes1750_suspend,
	.resume = aes1750_resume,
#endif
};

const static struct file_operations aes1750_fops = {
	.owner = THIS_MODULE,
	.open = aes1750_open,
	.release = aes1750_release,
	.read = aes1750_read,
	.write = aes1750_write,
	.unlocked_ioctl = aes1750_ioctl,
};

static struct miscdevice aes1750_misc_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = AES_MISC_DRIVER_NAME,
	.fops = &aes1750_fops,
};

static unsigned short int aes1750_keycodes[] = {
	KEY_LEFT,
	KEY_RIGHT,
	KEY_UP,
	KEY_DOWN,
	KEY_ENTER,
	BTN_LEFT,
	BTN_RIGHT,
	BTN_FINGERPRINT,
};

#ifdef AES1750_DEBUG
static void aes1750_print_buffer(const char *caller, unsigned trace_flag,
	char *buffer, int len)
{
	int i;
	/* Terminating zero allocated by sizeof(KERN_DEBUG) */
	/* 3 characters per byte */
	/* sizeof("\n") includes an extra terminating zero byte that is not
	   needed. */
	char tmp[3 * bytes_per_line + sizeof("\n")];
	char *p = tmp;
	unsigned part = 1;
	int length = len;
	int nparts;

	if (!trace_flag)
		return;
	if (!buffer || !len)
		return;

	if (len > max_bytes)
		len = max_bytes;

	nparts = (int)(len / bytes_per_line) + (len % bytes_per_line ? 1 : 0);
	for (i = 0; i < len; i++) {
		/* Header */
		if (i % bytes_per_line == 0) {
			/* Start a new line */
			p = tmp;
			*p = 0;
		}

		/* Body */
		*p++ = AES1750_NIBBLE_TO_ASCII(buffer[i] >> 4);
		*p++ = AES1750_NIBBLE_TO_ASCII(buffer[i] & 0x0F);
		*p++ = ' ';

		/* Footer */
		if (((i + 1) % bytes_per_line == 0) || (i + 1 == len)) {
			*p = '\0';
			if (nparts > 1)
				printk(KERN_INFO "%s: len %d, part %d of %d: %s%s\n",
					caller, length, part++, nparts, tmp,
					(i+1 == len && length > max_bytes) ? "..." : "");
			else
				printk(KERN_INFO "%s: len %d: %s%s\n", caller, length, tmp,
					length > max_bytes ? "..." : "");
		}
	}
}
#endif

/* aes1750_rx reads a block of size rx_len, sending any buffered data
   in tx_buf, then appends the data to the rx_msgs list.  This function
   is used by the IST.
*/
static int aes1750_rx(struct aes1750 *aes1750)
{
	int status;
	struct spi_message *m;
	struct spi_transfer *t;
	int tx_len = 0;

	/* Allocate the receive message, transfer, and buffer. */
	m = kzalloc(sizeof(struct spi_message) +
		    sizeof(struct spi_transfer) + aes1750->rx_len,
		    GFP_KERNEL);
	if (!m) {
		aes1750_err("failed to allocate spi_message\n");
		return -ENOMEM;
	}

	t = (struct spi_transfer *)(m + 1);

	spin_lock(&aes1750->lock);

	INIT_LIST_HEAD(&m->transfers);
	t->tx_buf = aes1750->read_tx_buf;
	t->len = aes1750->rx_len;
	t->rx_buf = (void *)(t + 1);
	t->bits_per_word = 8;

	/* Copy any buffered data to the output buffer. */
	if (aes1750->tx_len > aes1750->rx_len)
		aes1750_warn("cannot send buffered data. rx_len: %d, tx_len: %d\n",
			aes1750->rx_len, aes1750->tx_len);
	else if (aes1750->tx_len) {
		memcpy(aes1750->read_tx_buf, aes1750->tx_buf, aes1750->tx_len);

		aes1750_trace_txrx("sending buffered data.\n");
		aes1750_print_buffer_txrx(aes1750->read_tx_buf, aes1750->tx_len);

		tx_len = aes1750->tx_len;
		aes1750->tx_len = 0;
	}

	spin_unlock(&aes1750->lock);

	spi_message_add_tail(t, m);

	status = spi_sync(aes1750->spi, m);
	/*
	aes1750_trace_txrx(
		 "spi_sync rx: m: %p, t: %p, tx: %p, rx: %p, len: %d, "
		 "status: %d\n",
		 m, t, t->tx_buf, t->rx_buf, t->len, status);
	*/
	if (status)
		aes1750_err("spi_sync failed: %d\n", status);
	aes1750_print_buffer_txrx(t->rx_buf, t->len);

	/* Queue the message. */
	if (!status) {
		spin_lock(&aes1750->lock);
		list_add_tail(&m->queue, &aes1750->rx_msgs);
		spin_unlock(&aes1750->lock);
	}

	/* Reset the read_tx_buf now that we sent the data. */
	if (tx_len)
		memset(aes1750->read_tx_buf, AES1750_NOP, tx_len);

	return status;
}

/* aes1750_tx sends any buffered data in tx_buf, ignoring any data
   that is received by the sensor.  This function is used by aes1750_write().
*/
static int aes1750_tx(struct aes1750 *aes1750)
{
	int status;
	struct spi_message *m;
	struct spi_transfer *t;

	/* Allocate the message, transfer, and buffer. */
	m = kzalloc(sizeof(struct spi_message) + sizeof(struct spi_transfer),
		    GFP_KERNEL);
	if (!m) {
		aes1750_err("failed to allocate spi_message\n");
		return -ENOMEM;
	}

	t = (struct spi_transfer *)(m + 1);

	spin_lock(&aes1750->lock);

	INIT_LIST_HEAD(&m->transfers);
	t->tx_buf = aes1750->write_tx_buf;
	t->rx_buf = NULL;
	t->len = aes1750->tx_len;
	t->bits_per_word = 8;

	memcpy(aes1750->write_tx_buf, aes1750->tx_buf, t->len);
	aes1750->tx_len = 0;

	spin_unlock(&aes1750->lock);

	spi_message_add_tail(t, m);

	status = spi_sync(aes1750->spi, m);
	/*
	aes1750_trace_txrx(
		"spi_sync tx: m: %p, t: %p, tx: %p, rx: %p, len: %d, "
		"status: %d\n", m, t, t->tx_buf, t->rx_buf, t->len, status);
	*/
	if (status)
		aes1750_err("spi_sync failed: %d\n", status);
	aes1750_print_buffer_txrx((char *)t->tx_buf, t->len);

	kfree(m);

	return status;
}

/* This IST waits for a signal from the ISR that there is data to read.
   If rx_len is non-zero, the IST reads the appropriate block, and
   signals aes1750_read() that there is data ready.
*/
static int aes1750_thread(void *data)
{
	struct aes1750 *aes1750 = data;

	aes1750_trace_txrx("%d:%d started\n", current->pid, current->tgid);

	while (1) {
		int status;

		aes1750_trace_txrx("before sema gpio_int = %d\n",
			gpio_get_value(aes1750->gpio_int));

		/*aes1750_dbg("aes1750_thread: down(ist) %d\n",
			aes1750->ist_semaphore.count);*/
		status = down_interruptible(&aes1750->ist_semaphore);
		/*aes1750_dbg("aes1750_thread: down(ist) done %d\n",
			aes1750->ist_semaphore.count);*/
		if (status) {
			try_to_freeze();
			continue;
		}

		aes1750_trace_txrx("after sema gpio_int = %d\n",
			gpio_get_value(aes1750->gpio_int));

		if (kthread_should_stop()) {
			aes1750_trace_txrx("stopping the thread\n");
			break;
		}

		if (aes1750->state == AES1750_STATE_DISABLED) {
			aes1750_trace_txrx("stopping the thread\n");
			break;
		} else if (aes1750->state != AES1750_STATE_READY &&
			   aes1750->state != AES1750_STATE_ACTIVE) {
			aes1750_warn("ignoring interrupt while not ready or active\n");
			aes1750_flush(aes1750);
			continue;
		}

		aes1750_rx(aes1750);

		aes1750_trace_txrx("up(read) %d\n", aes1750->read_semaphore.count);
		up(&aes1750->read_semaphore);
		/*aes1750_dbg("aes1750_thread: up(read) done %d\n",
			aes1750->read_semaphore.count);*/
	}

	return 0;
}

/* The ISR signals the IST that there is data to read and then returns. */
static irqreturn_t aes1750_irq(int irq, void *data)
{
	struct aes1750 *aes1750 = data;
	int value = gpio_get_value(aes1750->gpio_int);

	aes1750_trace_txrx("value = %d\n", value);

	if (!value) {
		aes1750_warn("ignoring irq with value of 0\n");
		return IRQ_HANDLED;
	}

	aes1750_trace_txrx("up(ist) %d\n", aes1750->ist_semaphore.count);
	up(&aes1750->ist_semaphore);
	/*aes1750_dbg("aes1750_irq: up(ist) done %d\n",
		aes1750->ist_semaphore.count);*/

	return IRQ_HANDLED;
}

static int aes1750_reset(struct aes1750 *aes1750)
{
	int status;
	struct spi_message *m;
	struct spi_transfer *t;

	aes1750_info("enter\n");

	/* Allocate the message and transfer. */
	m = kzalloc(sizeof(struct spi_message) +
		    sizeof(struct spi_transfer), GFP_KERNEL);
	if (!m) {
		aes1750_err("failed to allocate spi_message\n");
		return -ENOMEM;
	}

	t = (struct spi_transfer *)(m + 1);

	INIT_LIST_HEAD(&m->transfers);
	t->rx_buf = NULL;
	/*
	t->tx_buf = aes1750->reset_command;
	t->len = aes1750->reset_command_len;
	*/
	/* Send Idle command instead to go into LP state */
	t->tx_buf = aes1750->idle_command;
	t->len = AES1750_IDLE_BUFFER_SIZE;
	t->bits_per_word = 8;

	spi_message_add_tail(t, m);

	status = spi_sync(aes1750->spi, m);
	/*
	aes1750_trace_txrx("spi_sync tx: m: %p, t: %p, tx: %p, rx: %p, len: %d, "
		 "status: %d\n",
		 m, t, t->tx_buf, t->rx_buf, t->len, status);
	*/
	if (status)
		aes1750_err("spi_sync failed: %d\n", status);
	aes1750_print_buffer_txrx((char *)t->tx_buf, t->len);

	kfree(m);

	return status;
}

/* Sends a signal specificed by signal down to the PID which
   currently holds the device open.
*/
static void aes1750_send_user_signal(struct aes1750 *aes1750, int signal)
{
	if (aes1750->pid) {
		struct task_struct *p;

		aes1750_info("pid = %d, signal = %d\n",
			aes1750->pid, signal);

		p = find_task_by_vpid(aes1750->pid);
		if (p) {
			int rc;
			rc = send_sig(signal, p, 0);
			if (rc)
				aes1750_err("send_sig failed: %d\n", rc);
		} else {
			aes1750_err("find_task_by_pid(%d) failed\n", aes1750->pid);
		}
	} else {
		aes1750_err("no pid registered for signal %d\n", signal);
	}
}

static int aes1750_flush(struct aes1750 *aes1750)
{
	int status = 0;
	struct spi_message *m;
	struct spi_transfer *t;
	char *rx_buf;
	char *tx_buf;
	/* Read 'len' sized blocks until the interrupt is clear. */
	int len = 16;
	size_t m_len = sizeof(struct spi_message) +
		       sizeof(struct spi_transfer) +
		       2 * len;

	aes1750_trace_txrx("enter\n");

	/* Allocate the receive message, transfer, and buffers. */
	m = kzalloc(m_len, GFP_KERNEL);
	if (!m) {
		aes1750_err("failed to allocate spi_message\n");
		return -ENOMEM;
	}

	while (gpio_get_value(aes1750->gpio_int)) {
		memset(m, 0, m_len);

		t = (struct spi_transfer *)(m + 1);
		rx_buf = (void *)(t + 1);
		tx_buf = rx_buf + len;

		INIT_LIST_HEAD(&m->transfers);
		t->rx_buf = rx_buf;
		t->tx_buf = tx_buf;
		t->len = len;
		t->bits_per_word = 8;

		memset(tx_buf, AES1750_NOP, len);

		spi_message_add_tail(t, m);

		status = spi_sync(aes1750->spi, m);
		/*
		aes1750_trace_txrx("spi_sync rx: m: %p, t: %p, tx: %p, rx: %p, "
			"len: %d, status: %d\n",
			 m, t, t->tx_buf, t->rx_buf, t->len, status);
		*/
		if (status)
			aes1750_err("spi_sync failed: %d\n", status);
		aes1750_print_buffer_txrx(t->rx_buf, t->len);
	}

	kfree(m);

	return status;
}

static void aes1750_set_rx_len(struct aes1750 *aes1750, int rx_len)
{
	spin_lock(&aes1750->lock);
	aes1750->rx_len = rx_len;
	aes1750->state = (aes1750->rx_len ? AES1750_STATE_READY :
					    AES1750_STATE_IDLE);
	spin_unlock(&aes1750->lock);
}

static void aes1750_flush_rx_msgs(struct aes1750 *aes1750)
{
	struct spi_message *m;
	struct spi_message *next;

	aes1750_trace_txrx("enter\n");

	if (list_empty(&aes1750->rx_msgs))
		return;

	list_for_each_entry_safe(m, next, &aes1750->rx_msgs, queue); {
		spin_unlock(&aes1750->lock);
		kfree(m);
		spin_lock(&aes1750->lock);
	}
	INIT_LIST_HEAD(&aes1750->rx_msgs);
}

static int aes1750_open(struct inode *inode, struct file *file)
{
	struct aes1750 *aes1750 = _aes1750;

	file->private_data = aes1750;

	aes1750_trace_request("%d:%d\n", current->pid, current->tgid);

	if (aes1750->state != AES1750_STATE_CLOSED) {
		aes1750_err("cannot open, state = %d\n", aes1750->state);
		return -EBUSY;
	}

	spin_lock(&aes1750->lock);
	aes1750->file = file;
	aes1750->pid = current->pid;
	aes1750->state = AES1750_STATE_IDLE;
	aes1750->rx_len = 0;
	aes1750_flush_rx_msgs(aes1750);

	memset(aes1750->tx_buf, AES1750_NOP, AES1750_MAX_BUFFER_SIZE);
	memset(aes1750->read_tx_buf, AES1750_NOP, AES1750_MAX_BUFFER_SIZE);
	memset(aes1750->write_tx_buf, AES1750_NOP, AES1750_MAX_BUFFER_SIZE);
	aes1750->tx_len = 0;

	spin_unlock(&aes1750->lock);

	return 0;
}

static int aes1750_release(struct inode *inode, struct file *file)
{
	struct aes1750 *aes1750 = file->private_data;

	aes1750_trace_request("%d:%d\n", current->pid, current->tgid);

	/* Reset the sensor so that it is in low-power mode. */
	aes1750_reset(aes1750);

	spin_lock(&aes1750->lock);
	aes1750->file = NULL;
	aes1750->pid = 0;
	aes1750->state = AES1750_STATE_CLOSED;
	aes1750->rx_len = 0;
	aes1750_flush_rx_msgs(aes1750);

	memset(aes1750->tx_buf, AES1750_NOP, AES1750_MAX_BUFFER_SIZE);
	memset(aes1750->read_tx_buf, AES1750_NOP, AES1750_MAX_BUFFER_SIZE);
	memset(aes1750->write_tx_buf, AES1750_NOP, AES1750_MAX_BUFFER_SIZE);
	aes1750->tx_len = 0;

	spin_unlock(&aes1750->lock);

	return 0;
}

/* aes1750_read() blocks until a message is available.  Once a message is
   available, it returns all of the bytes from the message.
*/
static ssize_t aes1750_read(struct file *file, char __user *buf, size_t count,
			    loff_t *f_pos)
{
	struct aes1750 *aes1750 = file->private_data;
	int status;
	struct spi_message *m;
	struct spi_transfer *t;
	int len;

	aes1750_trace_request("%d:%d count: %d\n",
		current->pid, current->tgid, count);

	/* Wait for a message to be available. */
	/*aes1750_dbg("aes1750_read: down(read) %d\n",
		aes1750->read_semaphore.count);*/
	status = down_interruptible(&aes1750->read_semaphore);
	/*aes1750_dbg("aes1750_read: down(read) done %d\n",
		aes1750->read_semaphore.count);*/
	if (status) {
		/* When am2server is suspended down_interruptible will return EINTR
		 * which is normal, so don't complain */
		if (!(status == -EINTR && atomic_read(&aes1750->is_suspended)))
			aes1750_err("down_interruptible failed: status = %d\n", status);
		return status;
	}

	/* Make sure a message is really available. */
	spin_lock(&aes1750->lock);
	if (list_empty(&aes1750->rx_msgs)) {
		aes1750_err("rx_msgs is empty\n");
		spin_unlock(&aes1750->lock);
		return -EFAULT;
	}

	/* Grab the first message. */
	m = list_entry(aes1750->rx_msgs.next, struct spi_message, queue);
	list_del(aes1750->rx_msgs.next);
	spin_unlock(&aes1750->lock);
	if (!m) {
		aes1750_err("message list_entry failed\n");
		return -EFAULT;
	}

	/* Copy the data from each transaction in the message. */
	len = 0;
	list_for_each_entry(t, &m->transfers, transfer_list) {
		if (t && t->rx_buf && t->len) {
			int tmplen;

			tmplen = min(count - len, t->len);

			aes1750_trace_request("m: %p, t: %p, tx: %p, rx: %p, "
				"t->len: %d, tmplen: %d\n",
				m, t, t->tx_buf, t->rx_buf, t->len, tmplen);

			//aes1750_print_buffer_request(t->rx_buf, tmplen);
			status = copy_to_user(buf, t->rx_buf, tmplen);
			if (status != 0) {
				aes1750_err("copy_to_user failed: %d\n", status);
				status = -EFAULT;
				break;
			}

			len += tmplen;
			if (len >= count)
				break;
		}
	}

	kfree(m);

	if (!status)
		status = len;

	aes1750_trace_request("done, status: %d.\n", status);
	return status;
}

/* If we are expecting a response from the sensor, aes1750_write() buffers
   the data in tx_buf, otherwise it sends it to the sensor.
*/
static ssize_t aes1750_write(struct file *file, const char __user *buf,
			     size_t count, loff_t *f_pos)
{
	struct aes1750 *aes1750 = file->private_data;
	int status;

	aes1750_trace_request("%d:%d count: %d\n",
		current->pid, current->tgid, count);

	/* Make sure there is enough buffer space available. */
	if (AES1750_MAX_BUFFER_SIZE - aes1750->tx_len < count) {
		aes1750_err("not enough space left in tx_buf\n");
		return -ENOMEM;
	}

	/* Copy the data to tx_buf. */
	status = copy_from_user(&aes1750->tx_buf[aes1750->tx_len], buf, count);
	if (status != 0) {
		aes1750_err("copy_from_user failed: %d\n", status);
		return -EFAULT;
	}
	//aes1750_print_buffer_request(&aes1750->tx_buf[aes1750->tx_len], count);

	spin_lock(&aes1750->lock);
	aes1750->tx_len += count;
	spin_unlock(&aes1750->lock);

	/* Send or buffer depending on the driver's state. */
	if (aes1750->state == AES1750_STATE_ACTIVE) {
		/* A transfer is active.
		   Buffer the data to send during the next receive. */
		aes1750_trace_request("buffering data\n");
		status = count;
	} else {
		spin_lock(&aes1750->lock);
		if (aes1750->rx_len)
			aes1750->state = AES1750_STATE_ACTIVE;
		spin_unlock(&aes1750->lock);

		/* A transfer is not active.
		   Go ahead and transmit it now.
		   Signal the thread to transmit. */
		status = aes1750_tx(aes1750);
		if (!status)
			status = count;
	}

	aes1750_trace_request("done, status: %d.\n", status);
	return status;
}

static long aes1750_ioctl(struct file *file, 
	unsigned int cmd, unsigned long arg)
{
	int status = 0;

	struct aes1750 *aes1750 = file->private_data;

	aes1750_trace_request("%d:%d cmd: %08x, arg: %08lx\n",
		current->pid, current->tgid, cmd, arg);

	/* Check type and command number */
	if (_IOC_TYPE(cmd) != AES1750_IOCTL_MAGIC) {
		aes1750_err("invalid magic number: cmd: %08x, arg: %08lx\n", cmd, arg);
		return -ENOTTY;
	}

	/* Check access direction once here; don't repeat below.
	 * IOC_DIR is from the user perspective, while access_ok is
	 * from the kernel perspective; so they look reversed.
	 */
	if (_IOC_DIR(cmd) & _IOC_READ) {
		aes1750_trace_request("check user read buffer\n");

		status = !access_ok(VERIFY_WRITE, (void __user *)arg,
				    _IOC_SIZE(cmd));
		if (status)
			aes1750_err("user read buffer invalid: "
				"cmd: %08x, arg: %08lx\n", cmd, arg);
	}

	if (status == 0 && _IOC_DIR(cmd) & _IOC_WRITE) {
		aes1750_trace_request("check user write buffer\n");

		status = !access_ok(VERIFY_READ, (void __user *)arg,
				    _IOC_SIZE(cmd));

		if (status)
			aes1750_err("user write buffer invalid: "
				"cmd: %08x, arg: %08lx\n", cmd, arg);
	}

	if (status) {
		aes1750_err("invalid user buffer: cmd: %08x, arg: %08lx\n",
			cmd, arg);
		return -EFAULT;
	}

	switch (cmd) {
	case AUTH_IOCTL_OS_SUSPENDSTATE: {
		u32 tmp = atomic_read(&aes1750->is_suspended);
		aes1750_info("AUTH_IOCTL_OS_SUSPENDSTATE: %d\n", tmp);
		status = __put_user(tmp, (__u32 __user *) arg);
		if (status != 0) {
			aes1750_warn("AUTH_IOCTL_OS_SUSPENDSTATE: __put_user error %d\n",
				status);
		}
		break;
	}
	case AES1750_IOCTL_SWIPE:{
		aes1750_trace_request("AES1750_IOCTL_SWIPE\n");
		if (aes1750->input_dev) {
			input_report_key(aes1750->input_dev, BTN_FINGERPRINT,
			    aes1750->swipe_value);
			aes1750->swipe_value = !aes1750->swipe_value;
			input_sync(aes1750->input_dev);
		}
		break;
	}
	case AES1750_IOCTL_RD_RX_LEN:{
		aes1750_trace_request("AES1750_IOCTL_RD_RX_LEN: %d\n", aes1750->rx_len);

		spin_lock(&aes1750->lock);
		status =
		    __put_user(aes1750->rx_len, (__u32 __user *) arg);
		spin_unlock(&aes1750->lock);

		break;
	}
	case AES1750_IOCTL_WR_RX_LEN:{
		u32 tmp;


		status = __get_user(tmp, (u32 __user *) arg);
		if (status == 0) {
			if (tmp > AES1750_MAX_BUFFER_SIZE) {
				aes1750_err("AES1750_IOCTL_WR_RX_LEN: Error: %d "
					"exceeds max buffer size\n", tmp);

				status = -EINVAL;
				break;
			} else  {
				aes1750_trace_request("AES1750_IOCTL_WR_RX_LEN: %d\n", tmp);
			}

			aes1750_set_rx_len(aes1750, tmp);
			status = 0;
		}

		break;
	}
	case AES1750_IOCTL_RD_NUM_RX_MSGS:{
		struct spi_message *m;
		int i = 0;

		spin_lock(&aes1750->lock);
		list_for_each_entry(m, &aes1750->rx_msgs, queue); {
			i++;
		}

		spin_unlock(&aes1750->lock);

		aes1750_trace_request("AES1750_IOCTL_RD_NUM_RX_MSGS: %d\n", i);

		status = __put_user(i, (__u32 __user *) arg);
		break;
	}
	case AES1750_IOCTL_RD_NUM_RX_MSG_LEN:{
		struct spi_message *m;
		int empty = 1;
		int len = 0;

		aes1750_trace_request("AES1750_IOCTL_RD_NUM_RX_MSG_LEN\n");

		spin_lock(&aes1750->lock);
		empty = list_empty(&aes1750->rx_msgs);

		if (!empty) {
			m = list_entry(aes1750->rx_msgs.next,
			       struct spi_message, queue);
			if (m) {
				struct spi_transfer *t;

				list_for_each_entry(t, &m->transfers,
						    transfer_list) {
					len += t->len;
					aes1750_trace_request("AES1750_IOCTL_RD_NUM_RX_MSG_LEN "
						"m: %p, t: %p, tx: %p, rx: %p, "
						"t->len: %d, len: %d\n",
						m, t, t->tx_buf, t->rx_buf,
						t->len, len);
				}
			} else {
				aes1750_err("AES1750_IOCTL_RD_NUM_RX_MSG_LEN: "
					"Error: get message failed\n");
			}
		}
		spin_unlock(&aes1750->lock);

		aes1750_trace_request("len: %d\n", len);
		status = __put_user(len, (__u32 __user *) arg);
		break;
	}
	case AES1750_IOCTL_INPUT_KEY:{
		u64 tmp;

		status = copy_from_user(&tmp, (const void *)arg,
					sizeof(tmp));
		if (status == 0) {
			unsigned int code;
			int value;

			code = tmp >> 32;
			value = tmp & 0xFFFFFFFF;

			aes1750_trace_request("AES1750_IOCTL_INPUT_KEY: "
				"code: %d, value: %d\n",
				code, value);

			if (aes1750->input_dev) {
				input_report_key(aes1750->input_dev, code, value);
				input_sync(aes1750->input_dev);
			} else {
				aes1750_warn("AES1750_IOCTL_INPUT_KEY: ignored\n");
			}
			status = 0;
		} else {
			aes1750_err("AES1750_IOCTL_INPUT_KEY: Error: get user failed\n");
		}

		break;
	}
	case AES1750_IOCTL_INPUT_REL:{
		u64 tmp;

		status = copy_from_user(&tmp, (const void *)arg,
					sizeof(tmp));
		if (status == 0) {
			int rel_x;
			int rel_y;

			rel_x = tmp >> 32;
			rel_y = tmp & 0xFFFFFFFF;

			aes1750_trace_request("AES1750_IOCTL_INPUT_KEY: "
				"rel_x: %d, rel_y: %d\n",
				rel_x, rel_y);

			if (aes1750->input_dev) {
				input_report_rel(aes1750->input_dev, REL_X, rel_x);
				input_report_rel(aes1750->input_dev, REL_Y, rel_y);
				input_report_key(aes1750->input_dev, BTN_FINGERPRINT,
					aes1750->swipe_value);
				aes1750->swipe_value = !aes1750->swipe_value;
				input_sync(aes1750->input_dev);
			} else {
				aes1750_warn("AES1750_IOCTL_INPUT_KEY: ignored\n");
			}

			status = 0;
		} else {
			aes1750_err("AES1750_IOCTL_INPUT_REL: Error: get user failed\n");
		}

		break;
	}
	default:{
		aes1750_err("Error: invalid ioctl 0x%x\n", cmd);
		status = -EFAULT;
	}
	}

	/* aes1750_dbg("aes1750_ioctl: status: %d\n", status); */
	return status;
}

static void aes1750_uninit(struct aes1750 *aes1750)
{
	aes1750_info("enter\n");

	/* Variables */
	spin_lock(&aes1750->lock);
	aes1750->file = NULL;
	aes1750->state = AES1750_STATE_DISABLED;

	aes1750->rx_len = 0;
	aes1750_flush_rx_msgs(aes1750);

	aes1750->tx_len = 0;
	spin_unlock(&aes1750->lock);

#if defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&aes1750->early_suspend);
#endif

	/* Thread */
	/*kthread_stop(aes1750->thread);*/
	aes1750_info("up(ist) %d\n", aes1750->ist_semaphore.count);
	up(&aes1750->ist_semaphore);
	aes1750_info("up(ist) done %d\n", aes1750->ist_semaphore.count);

	/* Input */
	if (aes1750->input_dev) {
		input_unregister_device(aes1750->input_dev);
		input_free_device(aes1750->input_dev);
	}

	/* IRQ */
	free_irq(aes1750->spi->irq, aes1750);

	/* GPIO */
	gpio_free(aes1750->gpio_int);

	/* Buffers */
	kfree(aes1750->tx_buf);
	aes1750->tx_buf = 0;

	kfree(aes1750->read_tx_buf);
	aes1750->read_tx_buf = 0;

	kfree(aes1750->write_tx_buf);
	aes1750->write_tx_buf = 0;

	kfree(aes1750->reset_command);
	aes1750->reset_command = 0;
	aes1750->reset_command_len = 0;

	/* Device */
	misc_deregister(&aes1750_misc_dev);

	kfree(aes1750);
	_aes1750 = NULL;
}

static int __devinit aes1750_probe(struct spi_device *spi)
{
	int status;
	struct aes1750 *aes1750;
	int i;

	aes1750_info("%d:%d spi: %p\n", current->pid, current->tgid, spi);

	if (!spi) {
		aes1750_err("invalid spi device\n");
		return -ENODEV;
	}
	aes1750_info("mode: %d speed: %d bpw: %d cs: %d irq: %d\n",
		 spi->mode, spi->max_speed_hz, spi->bits_per_word,
		 spi->chip_select, spi->irq);

	if (!spi->dev.platform_data) {
		aes1750_err("no platform data\n");
		return -ENODEV;
	}

	/* Allocate driver data */
	_aes1750 = aes1750 = kzalloc(sizeof(*aes1750), GFP_KERNEL);
	if (!aes1750) {
		aes1750_err("failed to allocate aes1750 struct\n");
		return -ENOMEM;
    }

	/* Initialize the driver data */
	aes1750->spi = spi;

	aes1750->file = NULL;
	aes1750->pid = 0;
	aes1750->state = AES1750_STATE_CLOSED;

	aes1750->rx_len = 0;
	aes1750->tx_len = 0;
	aes1750->swipe_value = 1;

	spin_lock_init(&aes1750->lock);
	sema_init(&aes1750->ist_semaphore, 0);
	sema_init(&aes1750->read_semaphore, 0);

	INIT_LIST_HEAD(&aes1750->rx_msgs);

	/* Device */
	misc_register(&aes1750_misc_dev);

	/* Buffers */
	aes1750->tx_buf = kmalloc(AES1750_MAX_BUFFER_SIZE, GFP_KERNEL);
	if (!aes1750->tx_buf) {
		aes1750_err("failed to allocate tx_buf\n");
		status = -ENOMEM;
		goto error;
	}
	memset(aes1750->tx_buf, AES1750_NOP, AES1750_MAX_BUFFER_SIZE);

	aes1750->read_tx_buf = kmalloc(AES1750_MAX_BUFFER_SIZE, GFP_KERNEL);
	if (!aes1750->read_tx_buf) {
		aes1750_err("failed to allocate read_tx_buf\n");
		status = -ENOMEM;
		goto error;
	}
	memset(aes1750->read_tx_buf, AES1750_NOP, AES1750_MAX_BUFFER_SIZE);

	aes1750->write_tx_buf = kmalloc(AES1750_MAX_BUFFER_SIZE, GFP_KERNEL);
	if (!aes1750->write_tx_buf) {
		aes1750_err("failed to allocate write_tx_buf\n");
		status = -ENOMEM;
		goto error;
	}
	memset(aes1750->write_tx_buf, AES1750_NOP, AES1750_MAX_BUFFER_SIZE);

	aes1750->reset_command_len = AES1750_RESET_BUFFER_SIZE;
	aes1750->reset_command = kzalloc(aes1750->reset_command_len,
		GFP_KERNEL);
	if (!aes1750->reset_command) {
		aes1750_err("failed to allocate reset_command\n");
		status = -ENOMEM;
		goto error;
	}
	aes1750->reset_command[0] = AES1750_NOP;

	/* Fill in IDLE command */
	aes1750->idle_command[0] = 0x0D;
	aes1750->idle_command[1] = 0x0D;

	/*
	NOTE:- Platform data is changed to tegra_spi_platform_data which does not have
	member to pass GPIO to be used as interrupt line, We do reverse mapping of irq to
	obtain GPIO
	*/
	if (aes1750->spi->irq)
		aes1750->gpio_int = irq_to_gpio(aes1750->spi->irq);
	aes1750_info("gpio_int: %d\n", aes1750->gpio_int);

	status = gpio_request(aes1750->gpio_int, "aes_int");
	if (status < 0) {
		aes1750_err("gpio_request failed: %d\n", status);
		goto error;
	}

	status = gpio_direction_input(aes1750->gpio_int);
	if (status < 0) {
		aes1750_err("gpio_direction_input failed: %d\n", status);
		goto error;
	}

	/* IRQ */
	if (!aes1750->spi->irq)
		aes1750->spi->irq = gpio_to_irq(aes1750->gpio_int);
	aes1750_info("irq: %d\n", aes1750->spi->irq);

	status =
	    request_irq(aes1750->spi->irq, aes1750_irq, AES1750_IRQ_FLAGS,
			"aes_int", aes1750);
	if (status < 0) {
		aes1750_err("request_irq failed: %d\n", status);
		goto error;
	}

	/* Input */
	aes1750->input_dev = input_allocate_device();
	if (!aes1750->input_dev) {
		aes1750_err("input_allocate_device failed\n");
		return -ENODEV;
	}

	aes1750->input_dev->name = AES_INPUT_DRIVER_NAME;
	aes1750->input_dev->phys = AES_INPUT_DRIVER_NAME;
	aes1750->input_dev->id.bustype = BUS_HOST;
	aes1750->input_dev->id.vendor = 0x0001;
	aes1750->input_dev->id.product = 0x3344;
	aes1750->input_dev->id.version = 0xae70;
	aes1750->input_dev->evbit[0] = BIT(EV_KEY) | BIT(EV_REL);
	aes1750->input_dev->keycode = aes1750_keycodes;
	aes1750->input_dev->keycodesize = sizeof(aes1750_keycodes[0]);
	aes1750->input_dev->keycodemax =
	    sizeof(aes1750_keycodes) / sizeof(aes1750_keycodes[0]);
	for (i = 0; i < aes1750->input_dev->keycodemax; i++)
		set_bit(aes1750_keycodes[i], aes1750->input_dev->keybit);

	status = input_register_device(aes1750->input_dev);
	if (status < 0) {
		aes1750_err("input_register_device failed: %d\n", status);
		input_free_device(aes1750->input_dev);
		return -ENODEV;
	}

	/* Thread */
	aes1750->thread =
	    kthread_create(aes1750_thread, aes1750, "%s", "aes1750");
	if (IS_ERR(aes1750->thread)) {
		aes1750_err("failed to create thread\n");
		status = -ENODEV;
		goto error;
	}
	aes1750_info("thread: %p\n", aes1750->thread);
	wake_up_process(aes1750->thread);

	spi_set_drvdata(spi, aes1750);
	atomic_set(&aes1750->is_suspended, 0);

#if defined(CONFIG_HAS_EARLYSUSPEND)
	aes1750->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 100;
	aes1750->early_suspend.suspend = aes1750_early_suspend;
	aes1750->early_suspend.resume = aes1750_late_resume;
	/* If the device is suspended, then register_early_suspend will
	   trigger an immediate notification. */
	register_early_suspend(&aes1750->early_suspend);
#endif

	/* Reset the sensor so it's in low power mode */
	aes1750_reset(aes1750);
    aes1750_info("probe successful\n");
	return 0;

error:
	aes1750_uninit(aes1750);
	return status;
}

static int __exit aes1750_remove(struct spi_device *spi)
{
	struct aes1750 *aes1750 = spi_get_drvdata(spi);

	aes1750_info("%d:%d\n", current->pid, current->tgid);

	/* Reset the sensor so that it is in low-power mode. */
	aes1750_reset(aes1750);

	aes1750_uninit(aes1750);

	return 0;
}

static int aes1750_suspend(struct spi_device *spi, pm_message_t mesg)
{
	struct aes1750 *aes1750 = spi_get_drvdata(spi);

	aes1750_info("%d:%d\n", current->pid, current->tgid);

	/* Set is_suspended to 1 and notify am2server right away so that it
	 * doesn't try to send/read any more data */
	disable_irq(aes1750->spi->irq);
	atomic_set(&aes1750->is_suspended, 1);
	aes1750_send_user_signal(aes1750, AES1750_SIGNAL_SUSPEND);

	/* Reset the sensor so that it is in low-power mode. */
	aes1750_reset(aes1750);
	aes1750_info("finished\n");

	return 0;
}

static int aes1750_resume(struct spi_device *spi)
{
	struct aes1750 *aes1750 = spi_get_drvdata(spi);

	aes1750_info("started\n");

	atomic_set(&aes1750->is_suspended, 0);
	enable_irq(aes1750->spi->irq);
	aes1750_send_user_signal(aes1750, AES1750_SIGNAL_RESUME);
	aes1750_info("finished\n");

	return 0;
}

#if defined(CONFIG_HAS_EARLYSUSPEND)
static void aes1750_early_suspend(struct early_suspend *handler)
{
	struct aes1750 *aes1750;

	aes1750 = container_of(handler, struct aes1750,
			early_suspend);

	aes1750_info("%d:%d\n", current->pid, current->tgid);

	aes1750_suspend(aes1750->spi, PMSG_SUSPEND);
}

static void aes1750_late_resume(struct early_suspend *handler)
{
	struct aes1750 *aes1750;

	aes1750 = container_of(handler, struct aes1750,
			early_suspend);

	aes1750_info("started\n");
	aes1750_resume(aes1750->spi);
	aes1750_info("finished\n");
}
#endif

int __devinit aes1750_init_module(void)
{
	int status;

	/*printk(KERN_DEBUG "%d:%d aes1750_init_module\n",
		current->pid, current->tgid);*/
	status = spi_register_driver(&aes1750_spi_driver);
	if (status)
		printk(KERN_ERR "spi_register_driver failed\n");

	return 0;
}

void __devexit aes1750_cleanup_module(void)
{
	/*printk(KERN_DEBUG "%d:%d aes1750_cleanup_module\n",
		current->pid, current->tgid);*/

	spi_unregister_driver(&aes1750_spi_driver);
	return;
}

device_initcall(aes1750_init_module);
module_exit(aes1750_cleanup_module);

MODULE_ALIAS("aes1750");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Motorola, Inc.");
MODULE_DESCRIPTION("aes1750 driver");

