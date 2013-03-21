#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/hardirq.h>
#include <linux/wakelock.h>
#include <linux/wait.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/circ_buf.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <linux/spi/spi.h>
#include <linux/mdm_ctrl.h>
#include "spi_tty.h"
#include "spi_dbg.h"
#include "spi_msg.h"

#define SPI_TTY_MAJOR		66
#define SPI_TTY_MINORS		1

#define USE_SPINLOCK 1
#if USE_SPINLOCK
#define LOCK_T          spinlock_t
#define CREATELOCK(_l)  spin_lock_init(&(_l))
#define DELETELOCK(_l)
#define LOCK(_l)        spin_lock(&(_l))
#define UNLOCK(_l)      spin_unlock(&(_l))
#define ATOMIC(_l,_f)   spin_lock_irqsave(&(_l),(_f))
#define UNATOMIC(_l,_f) spin_unlock_irqrestore(&(_l),(_f))
#else
#define LOCK_T          struct mutex
#define CREATELOCK(_l)  mutex_init(&(_l))
#define DELETELOCK(_l)
#define LOCK(_l)        mutex_lock(&(_l))
#define UNLOCK(_l)      mutex_unlock(&(_l))
#define ATOMIC(_l,_f)   local_irq_save((_f))
#define UNATOMIC(_l,_f) local_irq_restore((_f))
#endif

#define SPI_TTY_BUF_SIZE (128*1024) // Size should be 2's Power
#define MIN(x,y) (x < y ? x : y)
#define MAX(x,y) (x > y ? x : y)
// 2s wakelock timeout, because everything is done in kthread, we give it a little bit more time
#define SPI_TTY_WAKE_LOCK_TIMEOUT (2*HZ)
#define SPI_TTY_FORCE_FULL_TRANSACTION 1

struct spi_tty_s {
	struct tty_struct			*tty;
	int					open_count;
	LOCK_T 					port_lock;
	wait_queue_head_t                       write_wait;
	u8                                      write_buf_full;
	u8					dtr;
	struct circ_buf				*write_buf;
	u8					throttle;
	// when need read, null data packet will be created if no data to send
	// this is for DTR change and MRDY request case
	u8					tx_null;
	struct mutex 				work_lock;
	struct work_struct 			write_work;
	struct workqueue_struct			*work_queue;
	struct wake_lock			wakelock;
};

#if SPI_IPC_DEBUG
unsigned int spi_tty_debug_mask = 0;
module_param(spi_tty_debug_mask, uint, S_IRUGO|S_IWUSR);
#endif
unsigned long tx_size;
module_param(tx_size, ulong, S_IRUGO|S_IWUSR);
unsigned long tx_time;
module_param(tx_time, ulong, S_IRUGO|S_IWUSR);
unsigned long tx_count;
module_param(tx_count, ulong, S_IRUGO|S_IWUSR);
unsigned long write_count;
module_param(write_count, ulong, S_IRUGO|S_IWUSR);

struct spi_message spi_big_msg;
struct spi_transfer spi_big_trans;
static struct spi_tty_s *spi_tty_gbl = NULL;
static struct spi_tty_device *spi_tty_dev = NULL;

void spi_ipc_buf_dump1(const char *header, const u8 *buf, int len, int in_ascii)
{
        int i;
        int c;

        u8 dbg_buf[256];

        if (len <= 0)
                return;

        for (i = 0, c = 0; (i < len) && (c < (256 - 3)); i++) {
                if (in_ascii && isprint(buf[i])) {
                        sprintf(&dbg_buf[c], "%c ", buf[i]);
                        c += 2;
                }else{
                        sprintf(&dbg_buf[c], "%02x ", buf[i]);
                        c += 3;
                }
        }
        dbg_buf[c] = 0;
        SPI_IPC_INFO("%s%s\n", header, dbg_buf);
}

static void spi_tty_handle_data(void *context, u8 *buf, u32 count)
{
	int cnt, crc, len;
	struct spi_tty_s *spi_tty;
	spi_msg_header header;

	SPI_IPC_INFO("%s, context=%p, buf=%p, count=%d\n", __func__, context, buf, count);

	if (!context || !buf || !count) {
		pr_err("%s - Invalid param!\n", __func__);
		return;
	}

	spi_ipc_buf_dump("rx header: ", buf, SPI_MSG_HEADER_LEN);

	spi_tty = (struct spi_tty_s *) context;
	spi_msg_get_header(buf, &header);
	crc = spi_msg_cal_crc(&header);

	// validate data
	if (header.type != 1 || header.len > SPI_MTU || header.fcs != crc) {
		pr_err("Invalid data receicved!\n");
		return;
	}

	if (spi_tty->open_count > 0 && header.len > 0) {
		len = header.len;
		spi_ipc_buf_dump_ascii("rx data: ", buf+SPI_MSG_HEADER_LEN, (len>16?16:len));
		SPI_IPC_INFO("insert data to tty\n");
		buf += SPI_MSG_HEADER_LEN;
		do {
			cnt = tty_buffer_request_room(spi_tty->tty, len);
			if (cnt == 0)
				break;
			cnt = tty_insert_flip_string(spi_tty->tty, buf, cnt);
			tty_flip_buffer_push(spi_tty->tty);
			len -= cnt;
			buf += cnt;
		}while(len > 0);
	}
}

static void spi_tty_buf_clear(struct circ_buf *cb)
{
	cb->head = cb->tail = 0;
}

static struct circ_buf *spi_tty_buf_alloc(void)
{
	struct circ_buf *cb;

	cb = kmalloc(sizeof(struct circ_buf), GFP_KERNEL);
	if (!cb)
		return NULL;
	cb->buf = kmalloc(SPI_TTY_BUF_SIZE, GFP_KERNEL);
	if (!cb->buf) {
		kfree(cb);
		return NULL;
	}

	spi_tty_buf_clear(cb);

	return cb;
}

static void spi_tty_buf_free(struct circ_buf *cb)
{
	kfree(cb->buf);
	kfree(cb);
}

static int spi_tty_buf_room_avail(struct circ_buf *cb)
{
	return CIRC_SPACE(cb->head, cb->tail, SPI_TTY_BUF_SIZE);
}

static int spi_tty_buf_data_avail(struct circ_buf *cb)
{
	return CIRC_CNT(cb->head, cb->tail, SPI_TTY_BUF_SIZE);
}

static int spi_tty_buf_put(struct circ_buf *cb, const char *buf, int count)
{
	int c, ret = 0;

	while (1) {
		c = CIRC_SPACE_TO_END(cb->head, cb->tail, SPI_TTY_BUF_SIZE);
		if (count < c)
			c = count;
		if (c <= 0)
			break;
		memcpy(cb->buf + cb->head, buf, c);
		cb->head = (cb->head + c) & (SPI_TTY_BUF_SIZE-1);
		buf += c;
		count -= c;
		ret += c;
	}

	return ret;
}

static int spi_tty_buf_get(struct circ_buf *cb, char *buf, int count)
{
	int c, ret = 0;

	while (1) {
		c = CIRC_CNT_TO_END(cb->head, cb->tail, SPI_TTY_BUF_SIZE);
		if (count < c)
			c = count;
		if (c <= 0)
			break;
		memcpy(buf, cb->buf + cb->tail, c);
		cb->tail = (cb->tail + c) & (SPI_TTY_BUF_SIZE-1);
		buf += c;
		count -= c;
		ret += c;
	}

	return ret;
}

static void spi_tty_write_worker(struct work_struct *work)
{
	int c;
	int crc;
	unsigned long flags;
	unsigned long start_t = 0;
	spi_msg_header header;
	struct spi_tty_s *spi_tty =
		container_of(work, struct spi_tty_s, write_work);

	start_t = jiffies;
	SPI_IPC_INFO("%s\n", __func__);

	mutex_lock(&(spi_tty->work_lock));
	spin_lock_irqsave(&(spi_tty->port_lock), flags);

	while(((c = spi_tty_buf_data_avail(spi_tty->write_buf))
		|| (spi_tty->tx_null))
		&& (!spi_tty->throttle)
		&& (!(spi_tty_dev && spi_tty_dev->peer_is_dead)))
	{
		SPI_IPC_INFO("%s: %d outgoing bytes\n", __func__, c);
		if (spi_tty->tx_null)
			spi_tty->tx_null = 0;

		// initiate spi_big_trans
		memset((char*)spi_big_trans.tx_buf, 0x0,
			SPI_TRANSACTION_LEN * 2);
		spi_big_msg.actual_length = 0;

		c = MIN(c, SPI_MTU);
		spi_tty_buf_get(spi_tty->write_buf,
			(char*)spi_big_trans.tx_buf + SPI_MSG_HEADER_LEN,
			c);

		if (spi_tty->tty && spi_tty->open_count)
			tty_wakeup(spi_tty->tty);
		spin_unlock_irqrestore(&(spi_tty->port_lock), flags);

		header.type = 1;
		header.len = c;
		header.dtr = spi_tty->dtr;
		crc = spi_msg_cal_crc(&header);
		header.fcs = crc;
		spi_msg_set_header((u8*)spi_big_trans.tx_buf, &header);
#if SPI_TTY_FORCE_FULL_TRANSACTION
		spi_big_trans.len = SPI_TRANSACTION_LEN;
#else
		spi_big_trans.len = c + SPI_MSG_HEADER_LEN;
#endif
		spi_big_trans.speed_hz = SPI_SPEED_HZ;
		spi_big_trans.bits_per_word = 32;
		spi_ipc_buf_dump("tx header: ", spi_big_trans.tx_buf, SPI_MSG_HEADER_LEN);
		spi_ipc_buf_dump_ascii("tx data: ", spi_big_trans.tx_buf + SPI_MSG_HEADER_LEN, (c>16?16:c));
		if (spi_tty_dev)
			spi_sync(spi_tty_dev->spi, &spi_big_msg);
		else
			pr_warning("%s: dropping data: no spi device "
				   "registered", __func__);

		if (spi_big_msg.actual_length == SPI_TRANSACTION_LEN) {
			tx_count++;
			tx_size += spi_big_trans.len - SPI_MSG_HEADER_LEN;
			spi_tty_handle_data(spi_tty, spi_big_trans.rx_buf, SPI_TRANSACTION_LEN);
		}else {
			pr_err("%s: spi data transfer failed\n", __func__);
		}

		// wake up writes wait on queue
		wake_up_interruptible(&spi_tty->write_wait);

		spin_lock_irqsave(&(spi_tty->port_lock), flags);
	}

	spin_unlock_irqrestore(&(spi_tty->port_lock), flags);
	mutex_unlock(&(spi_tty->work_lock));
	SPI_IPC_INFO("%s: done\n", __func__);
	tx_time += jiffies_to_msecs(jiffies - start_t);
}

static int spi_tty_write_cache(struct tty_struct *tty,
		      const unsigned char *buffer, int count)
{
	unsigned long flags;
	int ret;
	struct spi_tty_s *spi_tty = tty->driver_data;

	spin_lock_irqsave(&spi_tty->port_lock, flags);

	if (spi_tty->throttle == 1) {
		// SPI is symmetrical bus, so stop write means stop receive
		ret = 0;
	}else{
		if (!in_interrupt()
			&& spi_tty_buf_room_avail(spi_tty->write_buf) < count) {
			// no enough room, wait
			spin_unlock_irqrestore(&spi_tty->port_lock, flags);
			SPI_IPC_INFO("No write room, put write wait...\n");
			spi_tty->write_buf_full = 1;
			wait_event_interruptible_timeout(spi_tty->write_wait,
						spi_tty->write_buf_full == 0, 2*HZ);
			SPI_IPC_INFO("Write wait up from sleep\n");
			spin_lock_irqsave(&spi_tty->port_lock, flags);
		}

		ret = spi_tty_buf_put(spi_tty->write_buf, buffer, count);
	}

	// wake lock to prevent suspend
	wake_lock_timeout(&spi_tty->wakelock, SPI_TTY_WAKE_LOCK_TIMEOUT);

	spin_unlock_irqrestore(&spi_tty->port_lock, flags);

	return ret;
}

static void spi_tty_handle_mrdy(void *context)
{
	unsigned long flags;
	struct spi_tty_s *spi_tty = context;

	if (!context) {
		return;
	}

	spin_lock_irqsave(&spi_tty->port_lock, flags);

	spi_tty->tx_null = 1;

	// wake lock to prevent suspend
	wake_lock_timeout(&spi_tty->wakelock, SPI_TTY_WAKE_LOCK_TIMEOUT);

	spin_unlock_irqrestore(&spi_tty->port_lock, flags);

	queue_work(spi_tty->work_queue, &spi_tty->write_work);
}

static int spi_tty_tiocmset(struct tty_struct *tty, unsigned int set,
			    unsigned int clear)
{
	unsigned long flags;
	struct spi_tty_s *spi_tty = tty->driver_data;

	spin_lock_irqsave(&spi_tty->port_lock, flags);

	if (set & TIOCM_DTR) {
		SPI_IPC_INFO("set DTR\n");
		spi_tty->dtr = 1;
	}

	if (clear & TIOCM_DTR) {
		SPI_IPC_INFO("clear DTR\n");
		spi_tty->dtr = 0;
	}

	spi_tty->tx_null = 1;

	spin_unlock_irqrestore(&spi_tty->port_lock, flags);

	queue_work(spi_tty->work_queue, &spi_tty->write_work);

	return 0;
}

static int spi_tty_write_room(struct tty_struct *tty)
{
	unsigned long flags;
	struct spi_tty_s *spi_tty = tty->driver_data;
	int room = 0;

	SPI_IPC_INFO("%s\n", __func__);

	spin_lock_irqsave(&spi_tty->port_lock, flags);

	if (spi_tty->open_count) {
		if (spi_tty->throttle == 1)
			room = 0;
		else
			room = spi_tty_buf_room_avail(spi_tty->write_buf);
	}

	spin_unlock_irqrestore(&spi_tty->port_lock, flags);

	SPI_IPC_INFO("room=%d\n", room);

	return room;
}

static int spi_tty_write(struct tty_struct *tty, const unsigned char *buffer,
			 int count)
{
	unsigned long flags;
	struct spi_tty_s *spi_tty = tty->driver_data;
	int retval = 0;

	if (!spi_tty || (spi_tty_dev && spi_tty_dev->peer_is_dead))
		return -ENODEV;

	//spi_slave_dump_hex(buffer, count);
	if (count > SPI_MTU) {
		pr_err("%s: data length over SPI MTU!\n", __func__);
		return 0;
	}

	spin_lock_irqsave(&spi_tty->port_lock, flags);

	if (!spi_tty->open_count) {
		pr_err("%s: device not opened!\n", __func__);
		spin_unlock_irqrestore(&spi_tty->port_lock, flags);
		return -EIO;
	}

	spin_unlock_irqrestore(&spi_tty->port_lock, flags);

	retval = spi_tty_write_cache(tty, buffer, count);
	write_count++;
	queue_work(spi_tty->work_queue, &spi_tty->write_work);
	SPI_IPC_INFO("%s exit, retval=%d\n", __func__, retval);
	return retval;
}

static void spi_tty_set_termios(struct tty_struct *tty,
				struct ktermios *old_termios)
{
}

static int spi_tty_tiocmget(struct tty_struct *tty)
{
	return 0;
}

static int spi_tty_ioctl(struct tty_struct *tty,
			 unsigned int cmd, unsigned long arg)
{
	return -ENOIOCTLCMD;
}

/* will be called by PPP from irq, so that only spinlock can be used */
static int spi_tty_open(struct tty_struct *tty, struct file *file)
{
	unsigned long flags;
	int index;
	struct spi_tty_s *spi_tty;

	SPI_IPC_INFO("%s, tty=%p\n", __func__, tty);

	spi_tty = spi_tty_gbl;

	spin_lock_irqsave(&spi_tty->port_lock, flags);

	index = tty->index;
	tty->driver_data = spi_tty;
	tty->low_latency = 1;
	spi_tty->tty = tty;
	++spi_tty->open_count;

	spin_unlock_irqrestore(&spi_tty->port_lock, flags);

	return 0;
}

static void do_close(struct spi_tty_s *spi_tty)
{
	unsigned long flags;

	SPI_IPC_INFO("%s\n", __func__);
	spin_lock_irqsave(&spi_tty->port_lock, flags);

	if (!spi_tty->open_count) {
		goto exit;
	}

	--spi_tty->open_count;
	if (spi_tty->open_count <= 0) {
		/* The port is being closed by the last user. */
		spi_tty->tty = NULL;
	}
exit:
	spin_unlock_irqrestore(&spi_tty->port_lock, flags);
}

static void spi_tty_close(struct tty_struct *tty, struct file *file)
{
	struct spi_tty_s *spi_tty = tty->driver_data;

	SPI_IPC_INFO("%s\n", __func__);
	if (spi_tty)
		do_close(spi_tty);
}

static struct tty_operations serial_ops = {
	.open = spi_tty_open,
	.close = spi_tty_close,
	.write = spi_tty_write,
	.write_room = spi_tty_write_room,
	.set_termios = spi_tty_set_termios,
	.ioctl = spi_tty_ioctl,
	.tiocmget = spi_tty_tiocmget,
	.tiocmset = spi_tty_tiocmset,
	.throttle = NULL,
	.unthrottle = NULL,
};

/* It's safe to call this before or after spi_tty_init() */
int spi_tty_register_device(struct spi_tty_device *device)
{
	if (spi_tty_dev)
		return -EBUSY;

	/* Populate the caller's callbacks */
	device->callback_context = spi_tty_gbl;
	device->data_callback = spi_tty_handle_data;
	device->mrdy_callback = spi_tty_handle_mrdy;

	spi_tty_dev = device;
	SPI_IPC_INFO("%s: registered device 0x%p\n", __func__, device);
	
	return 0;
}

static struct tty_driver *spi_tty_driver;

static int __init spi_tty_init(void)
{
	int retval;
	struct spi_tty_s *spi_tty;

	SPI_IPC_INFO("%s\n", __func__);

	tx_size = 0L;
	tx_time = 0L;
	tx_count = 0L;
	write_count = 0L;

	spi_tty_gbl = kmalloc(sizeof(*spi_tty), GFP_KERNEL);
	if (spi_tty_gbl == NULL) {
		pr_err("%s: Cannot malloc mem!\n", __func__);
		return -ENOMEM;
	}

	memset(spi_tty_gbl, 0x0, sizeof(*spi_tty));
	spi_tty = spi_tty_gbl;

	spi_tty->write_buf = spi_tty_buf_alloc();
	if (!spi_tty->write_buf) {
		kfree(spi_tty_gbl);
		pr_err("failed to malloc spi_tty write buf!\n");
		return -ENOMEM;
	}

	spi_tty->throttle = 0;
	spi_tty->open_count = 0;
	spi_tty->tx_null = 0;
	spin_lock_init(&spi_tty->port_lock);
	mutex_init(&spi_tty->work_lock);
	INIT_WORK(&spi_tty->write_work, spi_tty_write_worker);
	wake_lock_init(&spi_tty->wakelock, WAKE_LOCK_SUSPEND, "spi_tty_wakelock");

	init_waitqueue_head(&spi_tty->write_wait);
	spi_tty->write_buf_full = 0;

	spi_tty->work_queue = create_singlethread_workqueue("spi_tty_wq");
	if (spi_tty->work_queue  == NULL) {
		kfree(spi_tty);
		kfree(spi_big_trans.tx_buf);
		pr_err("Failed to create work queue\n");
		return -ESRCH;
	}

	spi_message_init(&spi_big_msg);

	spi_big_trans.tx_buf = kmalloc(SPI_TRANSACTION_LEN*2, GFP_KERNEL);
	if (!spi_big_trans.tx_buf) {
		kfree(spi_tty);
		pr_err("%s: Cannot malloc mem!\n", __func__);
		return -ENOMEM;
	}

	spi_big_trans.rx_buf =
			(char*)spi_big_trans.tx_buf + SPI_TRANSACTION_LEN;
	spi_message_add_tail(&spi_big_trans, &spi_big_msg);

	spi_tty_driver = alloc_tty_driver(SPI_TTY_MINORS);
	if (!spi_tty_driver)
		return -ENOMEM;

	spi_tty_driver->owner = THIS_MODULE;
	spi_tty_driver->driver_name = "spi_modem";
	spi_tty_driver->name = "ttySPI";
	spi_tty_driver->major = SPI_TTY_MAJOR;
	spi_tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
	spi_tty_driver->subtype = SERIAL_TYPE_NORMAL;
	spi_tty_driver->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
	spi_tty_driver->init_termios = tty_std_termios;
	spi_tty_driver->init_termios.c_cflag =
		B9600 | CS8 | CREAD | HUPCL | CLOCAL;
	tty_set_operations(spi_tty_driver, &serial_ops);

	retval = tty_register_driver(spi_tty_driver);
	if (retval) {
		pr_err("failed to register spi_tty tty driver");
		put_tty_driver(spi_tty_driver);
		return retval;
	}

	tty_register_device(spi_tty_driver, 0, NULL);

	/* 
	 * If spi_tty_register_device() was called before spi_tty_init(), fill
	 * in the blanks.  Yeah, it's ugly.
	 */
	if (spi_tty_dev)
		spi_tty_dev->callback_context = spi_tty;

	return retval;
}

static void __exit spi_tty_exit(void)
{
	struct spi_tty_s *spi_tty;

	tty_unregister_device(spi_tty_driver, 0);
	tty_unregister_driver(spi_tty_driver);

	/* shut down all of the timers and free the memory */
	spi_tty = spi_tty_gbl;
	if (spi_tty) {
		/* close the port */
		while (spi_tty->open_count)
			do_close(spi_tty);

		spi_tty_buf_free(spi_tty->write_buf);
		wake_lock_destroy(&spi_tty->wakelock);

		//TODO: flush can destory work queue
		kfree(spi_tty);
		spi_tty_gbl = NULL;
	}
}

module_init(spi_tty_init);
module_exit(spi_tty_exit);

