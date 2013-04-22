/*
 * Copyright (C) 2010 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/cpcap_wdog.h>
#include <linux/miscdevice.h>
#include <linux/spi/spi.h>
#include <linux/spi/cpcap.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/spi/cpcap-regbits.h>
#include <linux/timer.h>
#include <linux/workqueue.h>

#define CPCAP_WDT_TIMEOUT	        80
#define CPCAP_WDT_KICK_INTERVAL         10
#define CPCAP_WDT_RAMWRITE_RETRIES	5
#define CPCAP_WDT_MAX_TIMER		0xFFFF

struct cpcap_wdt {
	struct cpcap_device *cpcap;
	struct device *dev;
	unsigned long is_active;
#ifdef CONFIG_CPCAP_WATCHDOG_KERNEL_SPACE
	unsigned long kick_interval;  /* interval in jiffies */
	struct timer_list kick_timer;
	struct workqueue_struct *working_queue;
	struct work_struct kick_work;
#endif
};

static int cpcap_wdt_open(struct inode *inode, struct file *file);
static ssize_t cpcap_wdt_write(struct file *file, const char __user *data,
                size_t len, loff_t *ppos);
static long cpcap_wdt_ioctl(struct file *file, unsigned int cmd,
                                                unsigned long arg);
static int cpcap_wdt_release(struct inode *inode, struct file *file);
static int cpcap_wdt_start(struct cpcap_wdt *wdt);
static int cpcap_wdt_stop(struct cpcap_wdt *wdt);
static int cpcap_wdt_set_timeout(struct cpcap_wdt *wdt, unsigned short int timeout);
static int cpcap_wdt_get_timeout(struct cpcap_wdt *wdt, unsigned short int *timeout);
static int cpcap_wdt_get_timer(struct cpcap_wdt *wdt, unsigned short int *timer);
static int cpcap_wdt_ping(struct cpcap_wdt *wdt);
#ifdef CONFIG_CPCAP_WATCHDOG_KERNEL_SPACE
static void cpcap_wdt_handle_kicktimer(unsigned long data);
static void kick_wdt_work(struct work_struct *work);
static void cpcap_wdt_panic(enum cpcap_irqs irq, void *data);
#endif
#ifdef CONFIG_PM
static int cpcap_wdt_suspend(struct platform_device *pdev, pm_message_t message);
static int cpcap_wdt_resume(struct platform_device *pdev);
#endif

static struct cpcap_wdt *wdt_ptr;

static const struct file_operations wdt_fops = {
        .owner          = THIS_MODULE,
        .llseek         = no_llseek,
        .write          = cpcap_wdt_write,
        .unlocked_ioctl = cpcap_wdt_ioctl,
        .open           = cpcap_wdt_open,
        .release        = cpcap_wdt_release,
};

struct miscdevice cpcap_wdt_miscdev = {
	.minor  = WATCHDOG_MINOR,
	.name   = "watchdog",
	.fops   = &wdt_fops,
};

static struct watchdog_info cpcap_wdt_info = {
	.options        = WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING,
	.identity       = "cpcap watchdog",
};

static int cpcap_wdt_set_timeout(struct cpcap_wdt *wdt, unsigned short int timeout)
{
	int i, retval=-EFAULT;
	unsigned short readval=0;

	if (!wdt)
		return -EINVAL;

	if ((timeout < 0) || (timeout > CPCAP_WDT_MAX_TIMER))
		return -EINVAL;

	for(i=0; i<CPCAP_WDT_RAMWRITE_RETRIES; i++){
		cpcap_uc_set_wdt_timeout(wdt->cpcap, timeout);
		cpcap_uc_get_wdt_timeout(wdt->cpcap, &readval);
		if (readval == timeout){
			retval=0;
			break;
		}
		dev_err(wdt->dev, "failure to set timeout; retrying\n");
		msleep(1);
	}

	/* Ping needed for change to take effect */
	cpcap_wdt_ping(wdt);

	return retval;
}

static int cpcap_wdt_get_timeout(struct cpcap_wdt *wdt, unsigned short int *timeout)
{
	if (!wdt || !timeout)
		return -EINVAL;

	return(cpcap_uc_get_wdt_timeout(wdt->cpcap, timeout));
}

static int cpcap_wdt_get_timer(struct cpcap_wdt *wdt, unsigned short int *timer)
{
	if (!wdt || !timer)
		return -EINVAL;

	return(cpcap_uc_get_wdt_timer(wdt->cpcap, timer));
}

static int cpcap_wdt_ping(struct cpcap_wdt *wdt)
{

	//dev_dbg (wdt->dev, "%s()\n", __func__);
	dev_info (wdt->dev, "%s()\n", __func__);
	if (!wdt)
		return -EINVAL;

	/* secondary macro6 is used to kick the watchdog */
	return (cpcap_uc_start(wdt->cpcap, CPCAP_BANK_SECONDARY, CPCAP_MACRO_6));
}

static int cpcap_wdt_start(struct cpcap_wdt *wdt)
{
	int retval;
	static bool first_run=true;

	if (!wdt)
		return -EINVAL;

	/* only 1 client allowed */
	if (test_and_set_bit(0, &wdt->is_active))
		return -EBUSY;

	if (first_run)
	{
		retval=cpcap_wdt_set_timeout(wdt, CPCAP_WDT_TIMEOUT);
		if (retval)
			return retval;
		first_run=false;
	}

#ifdef CONFIG_CPCAP_WATCHDOG_KERNEL_SPACE
	dev_info(wdt->dev, "%s: starting kernel watchdog timer\n", __func__);
	if (timer_pending(&wdt->kick_timer))
		del_timer(&wdt->kick_timer);

	(wdt->kick_timer).expires = jiffies + wdt->kick_interval;
	add_timer(&wdt->kick_timer);
#endif
	/* Pinging wdog before it's started is safe and ensures
	   that the timeout value is reset to full value */
	cpcap_wdt_ping(wdt);

	/* primary macro14 is the main watchdog process */
	retval = cpcap_uc_start(wdt->cpcap, CPCAP_BANK_PRIMARY, CPCAP_MACRO_14);
	if (retval)
		dev_err(wdt->dev, "%s: failed to start macro: %d\n",
			__func__, retval);

	return (retval);
}

static int cpcap_wdt_stop(struct cpcap_wdt *wdt)
{
	int retval;

	if (!wdt)
		return -EINVAL;

#ifdef CONFIG_CPCAP_WATCHDOG_KERNEL_SPACE
	dev_info(wdt->dev, "%s: stopping kernel watchdog timer\n", __func__);

	del_timer(&wdt->kick_timer);
#endif
	clear_bit(0, &wdt->is_active);

	retval = cpcap_uc_stop(wdt->cpcap, CPCAP_BANK_PRIMARY, CPCAP_MACRO_14);
	if (retval)
		dev_err(wdt->dev, "%s: failed to stop macro: %d\n",
			__func__, retval);
	return (cpcap_uc_stop(wdt->cpcap, CPCAP_BANK_PRIMARY, CPCAP_MACRO_14));
}


static int cpcap_wdt_open(struct inode *inode, struct file *file)
{
	struct cpcap_wdt *wdt = wdt_ptr;
	int retval;

	if (!file)
		return -EINVAL;

	file->private_data = wdt;


	retval = cpcap_wdt_start(wdt);
	if (retval)
		return retval;

	return nonseekable_open(inode, file);
}

static int cpcap_wdt_release(struct inode *inode, struct file *file)
{
	struct cpcap_wdt *wdt = file->private_data;

	if (!file || !inode)
		return -EINVAL;

	/*
	 *  Shut off the timer unless NOWAYOUT is defined.
	 */
#ifndef CONFIG_WATCHDOG_NOWAYOUT
	cpcap_wdt_stop(wdt);
#else
	dev_err(wdt->dev, "unexpected close, not stopping!\n");
#endif
        return 0;
}


static ssize_t cpcap_wdt_write(struct file *file, const char __user *data,
                size_t len, loff_t *ppos)
{
	struct cpcap_wdt *wdt = file->private_data;

	if (len) {
		cpcap_wdt_ping(wdt);
	}

	return len;
}

static long cpcap_wdt_ioctl(struct file *file, unsigned int cmd,
                                                unsigned long arg)

{
	struct cpcap_wdt *wdt = file->private_data;
	int __user *data = (int __user *)arg;
        unsigned short int wdt_data;
	int ret = 0;

	dev_info (wdt->dev, "%s: ioctl cmd: 0x%x", __func__, cmd);

	switch (cmd) {
	case WDIOC_GETSUPPORT:
		ret = copy_to_user(data, &cpcap_wdt_info,
						sizeof(cpcap_wdt_info));
		if (ret)
			ret = -EFAULT;
		break;

	case WDIOC_GETSTATUS:
	case WDIOC_GETBOOTSTATUS:
		ret = put_user(0, data);
		break;

	case WDIOC_KEEPALIVE:
		cpcap_wdt_ping(wdt);
		break;

	case WDIOC_SETTIMEOUT:
		if (get_user(wdt_data, data)){
			ret = -EFAULT;
			break;
		}
		if (cpcap_wdt_set_timeout(wdt, wdt_data)){
			ret = -EINVAL;
			break;
		}
		/* fall through and return the timeout */
	case WDIOC_GETTIMEOUT:
		if (cpcap_wdt_get_timeout(wdt, &wdt_data)){
			ret = -EFAULT;
			break;
		}
		ret = put_user(wdt_data, data);
		break;
	case WDIOC_GETTIMELEFT:
		if (cpcap_wdt_get_timer(wdt, &wdt_data)){
			ret = -EFAULT;
			break;
		}
		ret = put_user(wdt_data, data);
		break;
#ifdef CONFIG_CPCAP_WATCHDOG_KERNEL_SPACE
	case WDIOC_SET_KICK_INTERVAL:
		if (get_user(wdt_data, data)){
			ret = -EFAULT;
			break;
		}
		wdt->kick_interval = wdt_data * HZ;
		ret = put_user(wdt->kick_interval/HZ, data);
		break;
#endif
	default:
		ret = -ENOTTY;
	}

	return ret;
}

#ifdef CONFIG_CPCAP_WATCHDOG_KERNEL_SPACE
static void cpcap_wdt_handle_kicktimer(unsigned long data){

	struct cpcap_wdt *wdt = (struct cpcap_wdt *)data;

	if (!wdt)
		return;

	/* queue the work to kick the watchdog */
	queue_work(wdt->working_queue, &wdt->kick_work);

	/* schedule the next watchdog kick */
	(wdt->kick_timer).expires = jiffies + wdt->kick_interval;
	add_timer(&wdt->kick_timer);

}

static void kick_wdt_work(struct work_struct *work)
{
	struct cpcap_wdt *wdt = container_of(work, struct cpcap_wdt, kick_work);

	if (!wdt)
		return;

	if (cpcap_wdt_ping(wdt))
	{
		dev_err(wdt->dev, "unable to kick watchdog\n");
	}
}

static void cpcap_wdt_panic(enum cpcap_irqs irq, void *data)
{
	struct cpcap_wdt *wdt = (struct cpcap_wdt *) data;

	/* Clearing IRQ within 500ms delays CPCAP restart by 1 timeout period */
	cpcap_irq_clear(wdt->cpcap, CPCAP_IRQ_UC_PRIMACRO_14);

	dev_err(wdt->dev, "host watchdog timeout.  PANIC!\n");

	/* now panic and reboot the system */
	//BUG();
}
#endif

static int __devinit cpcap_wdt_probe(struct platform_device *pdev)
{
	struct cpcap_wdt *wdt;
	struct cpcap_platform_data *data;
	int ret = 0;

	if (pdev->dev.platform_data == NULL) {
		dev_err(&pdev->dev, "no platform_data\n");
		return -EINVAL;
	}

	wdt = kzalloc(sizeof(*wdt), GFP_KERNEL);
	if (!wdt)
	{
		ret=-ENOMEM;
		goto err_ret;
	}

	wdt_ptr = wdt;
	wdt->cpcap = pdev->dev.platform_data;
	data = wdt->cpcap->spi->controller_data;
	if (data->wdt_disable == 1){
		dev_err(&pdev->dev, "watchdog disabled for debugging\n");
		ret=-ECANCELED;
		goto err_mem;
	}

	platform_set_drvdata(pdev, wdt);

	wdt->dev = &pdev->dev;

	clear_bit(0, &wdt->is_active);

#ifdef CONFIG_CPCAP_WATCHDOG_KERNEL_SPACE
	wdt->kick_interval = CPCAP_WDT_KICK_INTERVAL * HZ;

	/* Setup timer to kick watchdog */
	init_timer(&(wdt->kick_timer));
	(wdt->kick_timer).function = cpcap_wdt_handle_kicktimer;
	(wdt->kick_timer).data = (unsigned long) wdt;

	/* Init working queue */
	wdt->working_queue = create_singlethread_workqueue("cpcap_wdog_wq");
	if (!wdt->working_queue)
	{
		dev_err(wdt->dev, "cannot create work queue\n");
		ret = -ENOMEM;
		goto err_mem;
	}
	INIT_WORK(&wdt->kick_work, kick_wdt_work);

	/* Register for host watchdog interupt */
	ret = cpcap_irq_clear(wdt->cpcap, CPCAP_IRQ_UC_PRIMACRO_14);
	ret |= cpcap_irq_register(wdt->cpcap, CPCAP_IRQ_UC_PRIMACRO_14, cpcap_wdt_panic, wdt);
	if (ret)
	{
		dev_err(wdt->dev, "cannot register MACRO 14 IRQ\n");
		goto err_wq;
	}
#endif

	ret = misc_register(&cpcap_wdt_miscdev);
	if (ret)
	{
		dev_err(wdt->dev, "cannot register miscdev on minor=%d (err=%d)\n",
						WATCHDOG_MINOR, ret);
		goto err_irq;
	}

#ifdef CONFIG_CPCAP_WATCHDOG_KERNEL_SPACE
	ret = cpcap_wdt_start(wdt);
	if (ret)
	{
		dev_err(wdt->dev, "cannot start watchdog task\n");
		goto err_reg;
	}
	dev_info(&pdev->dev, "watchdog started from kernel\n");
#else
	dev_info(&pdev->dev, "watchdog framework started for user space control\n");
#endif

	return 0;

#ifdef CONFIG_CPCAP_WATCHDOG_KERNEL_SPACE
err_reg:
	misc_deregister(&cpcap_wdt_miscdev);
#endif

err_irq:
#ifdef CONFIG_CPCAP_WATCHDOG_KERNEL_SPACE
	cpcap_irq_free(wdt->cpcap, CPCAP_IRQ_UC_PRIMACRO_14);
err_wq:
	destroy_workqueue(wdt->working_queue);
#endif
err_mem:
	kfree(wdt);
err_ret:
	return ret;
}

static int __devexit cpcap_wdt_remove(struct platform_device *pdev)
{
	struct cpcap_wdt *wdt;

	wdt = platform_get_drvdata(pdev);

	misc_deregister(&cpcap_wdt_miscdev);
	cpcap_irq_free(wdt->cpcap, CPCAP_IRQ_UC_PRIMACRO_14);
#ifdef CONFIG_CPCAP_WATCHDOG_KERNEL_SPACE
	destroy_workqueue(wdt->working_queue);
#endif
	kfree(wdt);

	return 0;
}

#ifdef CONFIG_PM

static int cpcap_wdt_suspend(struct platform_device *pdev, pm_message_t message)
{
#ifdef CONFIG_CPCAP_WATCHDOG_KERNEL_SPACE
	struct cpcap_wdt *wdt = platform_get_drvdata(pdev);
	//dev_dbg(wdt->dev, "suspend\n");
	dev_info(wdt->dev, "suspend\n");
	return (cpcap_wdt_stop(wdt));
#else
	return 0;
#endif
}

static int cpcap_wdt_resume(struct platform_device *pdev)
{
#ifdef CONFIG_CPCAP_WATCHDOG_KERNEL_SPACE
	struct cpcap_wdt *wdt = platform_get_drvdata(pdev);
	//dev_dbg(wdt->dev, "resume\n");
	dev_info(wdt->dev, "resume\n");
	return (cpcap_wdt_start(wdt));
#else
	return 0;
#endif
}

#else
#define cpcap_wdt_suspend NULL
#define cpcap_wdt_resume  NULL
#endif

static struct platform_driver cpcap_wdt_driver = {
	.driver = {
		.name = "cpcap_wdt",
	},
	.probe = cpcap_wdt_probe,
	.remove = __devexit_p(cpcap_wdt_remove),
	.suspend = cpcap_wdt_suspend,
	.resume = cpcap_wdt_resume,
};

static int __init cpcap_wdt_init(void)
{
	return platform_driver_register(&cpcap_wdt_driver);
}
module_init(cpcap_wdt_init);

static void __exit cpcap_wdt_exit(void)
{
	platform_driver_unregister(&cpcap_wdt_driver);
}
module_exit(cpcap_wdt_exit);

MODULE_ALIAS("platform:cpcap_wdt");
MODULE_DESCRIPTION("CPCAP RTC driver");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");
