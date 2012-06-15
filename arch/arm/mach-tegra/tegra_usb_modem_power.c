/*
 * arch/arm/mach-tegra/tegra_usb_modem_power.c
 *
 * Copyright (c) 2011-2012, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/usb.h>
#include <linux/err.h>
#include <linux/pm_runtime.h>
#include <linux/suspend.h>
#include <linux/slab.h>
#include <linux/wakelock.h>
#include <linux/pm_qos_params.h>
#include <mach/tegra_usb_modem_power.h>

#define BOOST_CPU_FREQ_MIN	1200000
#define BOOST_CPU_FREQ_TIMEOUT	5000

#define WAKELOCK_TIMEOUT_FOR_USB_ENUM		(HZ * 10)
#define WAKELOCK_TIMEOUT_FOR_REMOTE_WAKE	(HZ)

struct tegra_usb_modem {
	struct tegra_usb_modem_power_platform_data *pdata;
	unsigned int wake_cnt;	/* remote wakeup counter */
	unsigned int wake_irq;	/* remote wakeup irq */
	unsigned int boot_irq;	/* modem boot irq */
	struct mutex lock;
	struct wake_lock wake_lock;	/* modem wake lock */
	unsigned int vid;	/* modem vendor id */
	unsigned int pid;	/* modem product id */
	struct usb_device *udev;	/* modem usb device */
	struct usb_device *parent;	/* parent device */
	struct usb_interface *intf;	/* first modem usb interface */
	struct workqueue_struct *wq;	/* modem workqueue */
	struct delayed_work recovery_work;	/* modem recovery work */
	struct pm_qos_request_list cpu_boost_req; /* min CPU freq request */
	struct work_struct cpu_boost_work;	/* CPU freq boost work */
	struct delayed_work cpu_unboost_work;	/* CPU freq unboost work */
	const struct tegra_modem_operations *ops;	/* modem operations */
	unsigned int capability;	/* modem capability */
	int system_suspend;	/* system suspend flag */
	struct notifier_block pm_notifier;	/* pm event notifier */
	struct notifier_block usb_notifier;	/* usb event notifier */
	int sysfs_file_created;
	int short_autosuspend_enabled;
};

static struct platform_device *hc = NULL;	/* USB host controller */
static struct mutex hc_lock;
static const struct platform_device *hc_device;
static const struct tegra_usb_platform_data *hc_pdata;

/* supported modems */
static const struct usb_device_id modem_list[] = {
	{USB_DEVICE(0x1983, 0x0310),	/* Icera 450 rev1 */
	 .driver_info = TEGRA_MODEM_AUTOSUSPEND,
	 },
	{USB_DEVICE(0x1983, 0x0321),	/* Icera 450 rev2 */
	 .driver_info = TEGRA_MODEM_AUTOSUSPEND,
	 },
	{USB_DEVICE(0x1983, 0x0327),	/* Icera 450 5AE */
	 .driver_info = TEGRA_MODEM_AUTOSUSPEND,
	 },
	{}
};

static void cpu_freq_unboost(struct work_struct *ws)
{
	struct tegra_usb_modem *modem = container_of(ws, struct tegra_usb_modem,
						     cpu_unboost_work.work);

	pm_qos_update_request(&modem->cpu_boost_req, PM_QOS_DEFAULT_VALUE);
}

static void cpu_freq_boost(struct work_struct *ws)
{
	struct tegra_usb_modem *modem = container_of(ws, struct tegra_usb_modem,
						     cpu_boost_work);

	cancel_delayed_work_sync(&modem->cpu_unboost_work);
	pm_qos_update_request(&modem->cpu_boost_req, BOOST_CPU_FREQ_MIN);
	queue_delayed_work(modem->wq, &modem->cpu_unboost_work,
			      msecs_to_jiffies(BOOST_CPU_FREQ_TIMEOUT));
}

static irqreturn_t tegra_usb_modem_wake_thread(int irq, void *data)
{
	struct tegra_usb_modem *modem = (struct tegra_usb_modem *)data;

	mutex_lock(&modem->lock);
	if (modem->udev && modem->udev->state != USB_STATE_NOTATTACHED) {
		pr_info("modem wake (%u)\n", ++(modem->wake_cnt));

		if (!modem->system_suspend) {
			wake_lock_timeout(&modem->wake_lock,
					  WAKELOCK_TIMEOUT_FOR_REMOTE_WAKE);

			usb_lock_device(modem->udev);
			if (usb_autopm_get_interface(modem->intf) == 0)
				usb_autopm_put_interface_async(modem->intf);
			usb_unlock_device(modem->udev);
		}
#ifdef CONFIG_PM
		if (modem->capability & TEGRA_MODEM_AUTOSUSPEND &&
		    modem->short_autosuspend_enabled) {
			pm_runtime_set_autosuspend_delay(&modem->udev->dev,
					modem->pdata->autosuspend_delay);
			modem->short_autosuspend_enabled = 0;
		}
#endif
	}
	mutex_unlock(&modem->lock);

	return IRQ_HANDLED;
}

static irqreturn_t tegra_usb_modem_boot_thread(int irq, void *data)
{
	struct tegra_usb_modem *modem = (struct tegra_usb_modem *)data;

	if (gpio_get_value(modem->pdata->boot_gpio))
		pr_info("BB_RST_OUT high\n");
	else
		pr_info("BB_RST_OUT low\n");

	/* hold wait lock to complete the enumeration */
	wake_lock_timeout(&modem->wake_lock, WAKELOCK_TIMEOUT_FOR_USB_ENUM);

	/* boost CPU freq */
	if (!work_pending(&modem->cpu_boost_work))
		queue_work(modem->wq, &modem->cpu_boost_work);

	/* USB disconnect maybe on going... */
	mutex_lock(&modem->lock);
	if (modem->udev && modem->udev->state != USB_STATE_NOTATTACHED)
		pr_warn("Device is not disconnected!\n");
	mutex_unlock(&modem->lock);

	return IRQ_HANDLED;
}

static void tegra_usb_modem_recovery(struct work_struct *ws)
{
	struct tegra_usb_modem *modem = container_of(ws, struct tegra_usb_modem,
						     recovery_work.work);

	mutex_lock(&modem->lock);
	if (!modem->udev) {	/* assume modem crashed */
		if (modem->ops && modem->ops->reset)
			modem->ops->reset();
	}
	mutex_unlock(&modem->lock);
}

static void device_add_handler(struct tegra_usb_modem *modem,
			       struct usb_device *udev)
{
	const struct usb_device_descriptor *desc = &udev->descriptor;
	struct usb_interface *intf = usb_ifnum_to_if(udev, 0);
	const struct usb_device_id *id = usb_match_id(intf, modem_list);

	if (id) {
		/* hold wakelock to ensure ril has enough time to restart */
		wake_lock_timeout(&modem->wake_lock,
				  WAKELOCK_TIMEOUT_FOR_USB_ENUM);

		pr_info("Add device %d <%s %s>\n", udev->devnum,
			udev->manufacturer, udev->product);

		mutex_lock(&modem->lock);
		modem->udev = udev;
		modem->parent = udev->parent;
		modem->intf = intf;
		modem->vid = desc->idVendor;
		modem->pid = desc->idProduct;
		modem->wake_cnt = 0;
		modem->capability = id->driver_info;
		mutex_unlock(&modem->lock);

		pr_info("persist_enabled: %u\n", udev->persist_enabled);

#ifdef CONFIG_PM
		if (modem->capability & TEGRA_MODEM_AUTOSUSPEND) {
			pm_runtime_set_autosuspend_delay(&udev->dev,
					modem->pdata->autosuspend_delay);
			modem->short_autosuspend_enabled = 0;
			usb_enable_autosuspend(udev);
			pr_info("enable autosuspend for %s %s\n",
				udev->manufacturer, udev->product);
		}
#endif
	}
}

static void device_remove_handler(struct tegra_usb_modem *modem,
				  struct usb_device *udev)
{
	const struct usb_device_descriptor *desc = &udev->descriptor;

	if (desc->idVendor == modem->vid && desc->idProduct == modem->pid) {
		pr_info("Remove device %d <%s %s>\n", udev->devnum,
			udev->manufacturer, udev->product);

		mutex_lock(&modem->lock);
		modem->udev = NULL;
		modem->intf = NULL;
		modem->vid = 0;
		mutex_unlock(&modem->lock);

		if (modem->capability & TEGRA_MODEM_RECOVERY)
			queue_delayed_work(modem->wq,
					   &modem->recovery_work, HZ * 10);
	}
}

static int mdm_usb_notifier(struct notifier_block *notifier,
			    unsigned long usb_event, void *udev)
{
	struct tegra_usb_modem *modem =
	    container_of(notifier, struct tegra_usb_modem, usb_notifier);

	switch (usb_event) {
	case USB_DEVICE_ADD:
		device_add_handler(modem, udev);
		break;
	case USB_DEVICE_REMOVE:
		device_remove_handler(modem, udev);
		break;
	}
	return NOTIFY_OK;
}

static int mdm_pm_notifier(struct notifier_block *notifier,
			   unsigned long pm_event, void *unused)
{
	struct tegra_usb_modem *modem =
	    container_of(notifier, struct tegra_usb_modem, pm_notifier);

	mutex_lock(&modem->lock);
	if (!modem->udev) {
		mutex_unlock(&modem->lock);
		return NOTIFY_DONE;
	}

	pr_info("%s: event %ld\n", __func__, pm_event);
	switch (pm_event) {
	case PM_SUSPEND_PREPARE:
		if (wake_lock_active(&modem->wake_lock)) {
			pr_warn("%s: wakelock was active, aborting suspend\n",
				__func__);
			mutex_unlock(&modem->lock);
			return NOTIFY_STOP;
		}

		modem->system_suspend = 1;
#ifdef CONFIG_PM
		if (modem->capability & TEGRA_MODEM_AUTOSUSPEND &&
		    modem->udev &&
		    modem->udev->state != USB_STATE_NOTATTACHED) {
			pm_runtime_set_autosuspend_delay(&modem->udev->dev,
					modem->pdata->short_autosuspend_delay);
			modem->short_autosuspend_enabled = 1;
		}
#endif
		mutex_unlock(&modem->lock);
		return NOTIFY_OK;
	case PM_POST_SUSPEND:
		modem->system_suspend = 0;
		mutex_unlock(&modem->lock);
		return NOTIFY_OK;
	}

	mutex_unlock(&modem->lock);
	return NOTIFY_DONE;
}

static int mdm_request_wakeable_irq(struct tegra_usb_modem *modem,
				    irq_handler_t thread_fn,
				    unsigned int irq_gpio,
				    unsigned long irq_flags,
				    const char *label, unsigned int *irq)
{
	int ret;

	ret = gpio_request(irq_gpio, label);
	if (ret)
		return ret;

	tegra_gpio_enable(irq_gpio);

	/* enable IRQ for GPIO */
	*irq = gpio_to_irq(irq_gpio);

	/* request threaded irq for GPIO */
	ret = request_threaded_irq(*irq, NULL, thread_fn, irq_flags, label,
				   modem);
	if (ret)
		return ret;

	ret = enable_irq_wake(*irq);
	if (ret) {
		free_irq(*irq, modem);
		return ret;
	}

	return ret;
}

/* load USB host controller */
static struct platform_device *tegra_usb_null_ulpi_host_register(void)
{
	struct platform_device *pdev;
	int val;

	pdev = platform_device_alloc(hc_device->name, hc_device->id);
	if (!pdev)
		return NULL;

	val = platform_device_add_resources(pdev, hc_device->resource,
					    hc_device->num_resources);
	if (val)
		goto error;

	pdev->dev.dma_mask = hc_device->dev.dma_mask;
	pdev->dev.coherent_dma_mask = hc_device->dev.coherent_dma_mask;

	val = platform_device_add_data(pdev, hc_pdata,
				       sizeof(struct tegra_usb_platform_data));
	if (val)
		goto error;

	val = platform_device_add(pdev);
	if (val)
		goto error;

	return pdev;

error:
	pr_err("%s: err %d\n", __func__, val);
	platform_device_put(pdev);
	return NULL;
}

/* unload USB host controller */
static void tegra_usb_null_ulpi_host_unregister(struct platform_device *pdev)
{
	platform_device_unregister(pdev);
}

static ssize_t show_usb_host(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", (hc) ? 1 : 0);
}

static ssize_t load_unload_usb_host(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	int host;

	if (sscanf(buf, "%d", &host) != 1 || host < 0 || host > 1)
		return -EINVAL;

	pr_info("%s USB host\n", (host) ? "load" : "unload");

	mutex_lock(&hc_lock);
	if (host) {
		if (!hc)
			hc = tegra_usb_null_ulpi_host_register();
	} else {
		if (hc) {
			tegra_usb_null_ulpi_host_unregister(hc);
			hc = NULL;
		}
	}
	mutex_unlock(&hc_lock);

	return count;
}

static DEVICE_ATTR(load_host, 0666, show_usb_host, load_unload_usb_host);

static int mdm_init(struct tegra_usb_modem *modem, struct platform_device *pdev)
{
	struct tegra_usb_modem_power_platform_data *pdata =
	    pdev->dev.platform_data;
	int ret = 0;

	modem->pdata = pdata;

	hc_device = pdata->tegra_ehci_device;
	hc_pdata = pdata->tegra_ehci_pdata;
	mutex_init(&hc_lock);

	/* get modem operations from platform data */
	modem->ops = (const struct tegra_modem_operations *)pdata->ops;

	if (modem->ops) {
		/* modem init */
		if (modem->ops->init) {
			ret = modem->ops->init();
			if (ret)
				return ret;
		}

		/* start modem */
		if (modem->ops->start)
			modem->ops->start();
	}

	/* create sysfs node to load/unload host controller */
	ret = device_create_file(&pdev->dev, &dev_attr_load_host);
	if (ret) {
		dev_err(&pdev->dev, "can't create sysfs file\n");
		goto error;
	}
	modem->sysfs_file_created = 1;

	mutex_init(&(modem->lock));
	wake_lock_init(&modem->wake_lock, WAKE_LOCK_SUSPEND, "mdm_lock");

	/* create work queue platform_driver_registe */
	modem->wq = create_workqueue("tegra_usb_mdm_queue");
	INIT_DELAYED_WORK(&modem->recovery_work, tegra_usb_modem_recovery);

	INIT_WORK(&modem->cpu_boost_work, cpu_freq_boost);
	INIT_DELAYED_WORK(&modem->cpu_unboost_work, cpu_freq_unboost);

	pm_qos_add_request(&modem->cpu_boost_req, PM_QOS_CPU_FREQ_MIN,
			   PM_QOS_DEFAULT_VALUE);

	/* request remote wakeup irq from platform data */
	ret = mdm_request_wakeable_irq(modem,
				       tegra_usb_modem_wake_thread,
				       pdata->wake_gpio,
				       pdata->wake_irq_flags,
				       "mdm_wake", &modem->wake_irq);
	if (ret) {
		dev_err(&pdev->dev, "request wake irq error\n");
		goto error;
	}

	/* request boot irq from platform data */
	ret = mdm_request_wakeable_irq(modem,
				       tegra_usb_modem_boot_thread,
				       pdata->boot_gpio,
				       pdata->boot_irq_flags,
				       "mdm_boot", &modem->boot_irq);
	if (ret) {
		dev_err(&pdev->dev, "request boot irq error\n");
		goto error;
	}

	modem->pm_notifier.notifier_call = mdm_pm_notifier;
	modem->usb_notifier.notifier_call = mdm_usb_notifier;

	usb_register_notify(&modem->usb_notifier);
	register_pm_notifier(&modem->pm_notifier);

	return ret;
error:
	if (modem->sysfs_file_created)
		device_remove_file(&pdev->dev, &dev_attr_load_host);

	if (modem->wake_irq) {
		disable_irq_wake(modem->wake_irq);
		free_irq(modem->wake_irq, modem);
	}

	if (modem->boot_irq) {
		disable_irq_wake(modem->boot_irq);
		free_irq(modem->boot_irq, modem);
	}

	return ret;
}

static int tegra_usb_modem_probe(struct platform_device *pdev)
{
	struct tegra_usb_modem_power_platform_data *pdata =
	    pdev->dev.platform_data;
	struct tegra_usb_modem *modem;
	int ret = 0;

	if (!pdata) {
		dev_dbg(&pdev->dev, "platform_data not available\n");
		return -EINVAL;
	}

	modem = kzalloc(sizeof(struct tegra_usb_modem), GFP_KERNEL);
	if (!modem) {
		dev_dbg(&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	ret = mdm_init(modem, pdev);
	if (ret) {
		kfree(modem);
		return ret;
	}

	dev_set_drvdata(&pdev->dev, modem);

	return ret;
}

static int __exit tegra_usb_modem_remove(struct platform_device *pdev)
{
	struct tegra_usb_modem *modem = platform_get_drvdata(pdev);

	unregister_pm_notifier(&modem->pm_notifier);
	usb_unregister_notify(&modem->usb_notifier);

	if (modem->wake_irq) {
		disable_irq_wake(modem->wake_irq);
		free_irq(modem->wake_irq, modem);
	}

	if (modem->boot_irq) {
		disable_irq_wake(modem->boot_irq);
		free_irq(modem->boot_irq, modem);
	}

	if (modem->sysfs_file_created)
		device_remove_file(&pdev->dev, &dev_attr_load_host);

	cancel_work_sync(&modem->cpu_boost_work);
	cancel_delayed_work_sync(&modem->cpu_unboost_work);
	destroy_workqueue(modem->wq);

	pm_qos_remove_request(&modem->cpu_boost_req);

	kfree(modem);
	return 0;
}

#ifdef CONFIG_PM
static int tegra_usb_modem_suspend(struct platform_device *pdev,
				   pm_message_t state)
{
	struct tegra_usb_modem *modem = platform_get_drvdata(pdev);

	/* send L3 hint to modem */
	if (modem->ops && modem->ops->suspend)
		modem->ops->suspend();
	return 0;
}

static int tegra_usb_modem_resume(struct platform_device *pdev)
{
	struct tegra_usb_modem *modem = platform_get_drvdata(pdev);

	/* send L3->L0 hint to modem */
	if (modem->ops && modem->ops->resume)
		modem->ops->resume();
	return 0;
}
#endif

static struct platform_driver tegra_usb_modem_power_driver = {
	.driver = {
		   .name = "tegra_usb_modem_power",
		   .owner = THIS_MODULE,
		   },
	.probe = tegra_usb_modem_probe,
	.remove = __exit_p(tegra_usb_modem_remove),
#ifdef CONFIG_PM
	.suspend = tegra_usb_modem_suspend,
	.resume = tegra_usb_modem_resume,
#endif
};

static int __init tegra_usb_modem_power_init(void)
{
	return platform_driver_register(&tegra_usb_modem_power_driver);
}

subsys_initcall(tegra_usb_modem_power_init);

static void __exit tegra_usb_modem_power_exit(void)
{
	platform_driver_unregister(&tegra_usb_modem_power_driver);
}

module_exit(tegra_usb_modem_power_exit);

MODULE_DESCRIPTION("Tegra usb modem power management driver");
MODULE_LICENSE("GPL");
