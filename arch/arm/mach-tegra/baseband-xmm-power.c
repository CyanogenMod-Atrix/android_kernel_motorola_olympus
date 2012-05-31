/*
 * arch/arm/mach-tegra/baseband-xmm-power.c
 *
 * Copyright (C) 2011 NVIDIA Corporation
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/wakelock.h>
#include <linux/spinlock.h>
#include <linux/usb.h>
#include <linux/pm_runtime.h>
#include <linux/suspend.h>
#include <linux/pm_qos_params.h>
#include <mach/usb_phy.h>
#include <linux/regulator/consumer.h>
#include "board.h"
#include "board-enterprise.h"
#include "devices.h"
#include "gpio-names.h"
#include "baseband-xmm-power.h"

MODULE_LICENSE("GPL");

unsigned long modem_ver = XMM_MODEM_VER_1130;
EXPORT_SYMBOL(modem_ver);

unsigned long modem_flash;
EXPORT_SYMBOL(modem_flash);

unsigned long modem_pm = 1;
EXPORT_SYMBOL(modem_pm);

unsigned long enum_delay_ms = 1000; /* ignored if !modem_flash */

module_param(modem_ver, ulong, 0644);
MODULE_PARM_DESC(modem_ver,
	"baseband xmm power - modem software version");
module_param(modem_flash, ulong, 0644);
MODULE_PARM_DESC(modem_flash,
	"baseband xmm power - modem flash (1 = flash, 0 = flashless)");
module_param(modem_pm, ulong, 0644);
MODULE_PARM_DESC(modem_pm,
	"baseband xmm power - modem power management (1 = pm, 0 = no pm)");
module_param(enum_delay_ms, ulong, 0644);
MODULE_PARM_DESC(enum_delay_ms,
	"baseband xmm power - delay in ms between modem on and enumeration");

static struct usb_device_id xmm_pm_ids[] = {
	{ USB_DEVICE(VENDOR_ID, PRODUCT_ID),
	.driver_info = 0 },
	{}
};


static struct gpio tegra_baseband_gpios[] = {
	{ -1, GPIOF_OUT_INIT_LOW,  "BB_RSTn" },
	{ -1, GPIOF_OUT_INIT_LOW,  "BB_ON"   },
	{ -1, GPIOF_OUT_INIT_LOW,  "IPC_BB_WAKE" },
	{ -1, GPIOF_IN,            "IPC_AP_WAKE" },
	{ -1, GPIOF_OUT_INIT_HIGH, "IPC_HSIC_ACTIVE" },
	{ -1, GPIOF_IN,            "IPC_HSIC_SUS_REQ" },
};

static enum {
	IPC_AP_WAKE_UNINIT,
	IPC_AP_WAKE_IRQ_READY,
	IPC_AP_WAKE_INIT1,
	IPC_AP_WAKE_INIT2,
	IPC_AP_WAKE_L,
	IPC_AP_WAKE_H,
} ipc_ap_wake_state;

static enum baseband_xmm_powerstate_t baseband_xmm_powerstate;
static struct workqueue_struct *workqueue;
static struct work_struct init1_work;
static struct work_struct init2_work;
static struct work_struct L2_resume_work;
static struct work_struct autopm_resume_work;
static bool register_hsic_device;
static struct wake_lock wakelock;
static struct usb_device *usbdev;
static bool CP_initiated_L2toL0;
static bool modem_power_on;
static int power_onoff;
static int reenable_autosuspend;
static bool wakeup_pending;
static bool modem_sleep_flag;
static spinlock_t xmm_lock;
static DEFINE_MUTEX(xmm_onoff_mutex);
static bool system_suspending;
static struct regulator *enterprise_hsic_reg;
static bool _hsic_reg_status;
static struct pm_qos_request_list boost_cpu_freq_req;
static struct delayed_work pm_qos_work;
#define BOOST_CPU_FREQ_MIN	1500000

/* driver specific data - same structure is used for flashless
 * & flashed modem drivers i.e. baseband-xmm-power2.c
 */
struct xmm_power_data xmm_power_drv_data;
EXPORT_SYMBOL(xmm_power_drv_data);

static void xmm_power_L2_resume(void);
static int xmm_power_driver_handle_resume(
			struct baseband_power_platform_data *data);

static int tegra_baseband_rail_on(void)
{
	int ret;
	struct board_info bi;
	tegra_get_board_info(&bi);

	/* only applicable to enterprise */
	if (bi.board_id != BOARD_E1197)
		return 0;

	if (_hsic_reg_status == true)
		return 0;

	if (enterprise_hsic_reg == NULL) {
		enterprise_hsic_reg = regulator_get(NULL, "avdd_hsic");
		if (IS_ERR_OR_NULL(enterprise_hsic_reg)) {
			pr_err("xmm: could not get regulator vddio_hsic\n");
			enterprise_hsic_reg = NULL;
			return PTR_ERR(enterprise_hsic_reg);
		}
	}
	ret = regulator_enable(enterprise_hsic_reg);
	if (ret < 0) {
		pr_err("xmm: failed to enable regulator\n");
		return ret;
	}
	_hsic_reg_status = true;
	return 0;
}

static int tegra_baseband_rail_off(void)
{
	int ret;
	struct board_info bi;
	tegra_get_board_info(&bi);

	/* only applicable to enterprise */
	if (bi.board_id != BOARD_E1197)
		return 0;

	if (_hsic_reg_status == false)
		return 0;

	if (IS_ERR_OR_NULL(enterprise_hsic_reg)) {
		pr_err("xmm: unbalanced disable on vddio_hsic regulator\n");
		enterprise_hsic_reg = NULL;
		return PTR_ERR(enterprise_hsic_reg);
	}
	ret = regulator_disable(enterprise_hsic_reg);
	if (ret < 0) {
		pr_err("xmm: failed to disable regulator\n");
		return ret;
	}
	_hsic_reg_status = false;
	return 0;
}

static int baseband_modem_power_on(struct baseband_power_platform_data *data)
{
	/* set IPC_HSIC_ACTIVE active */
	gpio_set_value(data->modem.xmm.ipc_hsic_active, 1);

	/* reset / power on sequence */
	msleep(40);
	gpio_set_value(data->modem.xmm.bb_rst, 1);
	mdelay(1);
	gpio_set_value(data->modem.xmm.bb_on, 1);
	udelay(70);
	gpio_set_value(data->modem.xmm.bb_on, 0);

	pr_debug("%s: pm qos request CPU 1.5GHz\n", __func__);
	pm_qos_update_request(&boost_cpu_freq_req, (s32)BOOST_CPU_FREQ_MIN);
	/* Device enumeration should happen in 1 sec however in any case
	 * we want to request it back to normal so schedule work to restore
	 * CPU freq after 2 seconds */
	schedule_delayed_work(&pm_qos_work, msecs_to_jiffies(2000));

	return 0;
}

static int xmm_power_on(struct platform_device *device)
{
	struct baseband_power_platform_data *pdata =
			device->dev.platform_data;
	struct xmm_power_data *data = &xmm_power_drv_data;
	int ret;

	pr_debug("%s {\n", __func__);

	/* check for platform data */
	if (!pdata) {
		pr_err("%s: !pdata\n", __func__);
		return -EINVAL;
	}
	if (baseband_xmm_powerstate != BBXMM_PS_UNINIT)
		return -EINVAL;

	tegra_baseband_rail_on();

	/* reset the state machine */
	baseband_xmm_powerstate = BBXMM_PS_INIT;
	modem_sleep_flag = false;
	ipc_ap_wake_state = IPC_AP_WAKE_INIT2;

	pr_debug("%s wake_st(%d) modem version %lu\n", __func__,
				ipc_ap_wake_state, modem_ver);

	/* register usb host controller */
	if (!modem_flash) {
		pr_debug("%s - %d\n", __func__, __LINE__);
		/* register usb host controller only once */
		if (register_hsic_device) {
			pr_debug("%s: register usb host controller\n",
				__func__);
			modem_power_on = true;
			if (pdata->hsic_register)
				data->hsic_device = pdata->hsic_register();
			else
				pr_err("%s: hsic_register is missing\n",
					__func__);
			register_hsic_device = false;
		} else {
			/* register usb host controller */
			if (pdata->hsic_register)
				data->hsic_device = pdata->hsic_register();
			/* turn on modem */
			pr_debug("%s call baseband_modem_power_on\n", __func__);
			baseband_modem_power_on(pdata);
		}
	}
	ret = enable_irq_wake(gpio_to_irq(pdata->modem.xmm.ipc_ap_wake));
	if (ret < 0)
		pr_err("%s: enable_irq_wake error\n", __func__);
	pr_debug("%s }\n", __func__);

	return 0;
}

static int xmm_power_off(struct platform_device *device)
{
	struct baseband_power_platform_data *pdata =
			device->dev.platform_data;
	struct xmm_power_data *data = &xmm_power_drv_data;

	int ret;
	unsigned long flags;

	pr_debug("%s {\n", __func__);

	if (baseband_xmm_powerstate == BBXMM_PS_UNINIT)
		return -EINVAL;
	/* check for device / platform data */
	if (!device) {
		pr_err("%s: !device\n", __func__);
		return -EINVAL;
	}
	if (!pdata) {
		pr_err("%s: !pdata\n", __func__);
		return -EINVAL;
	}

	ipc_ap_wake_state = IPC_AP_WAKE_UNINIT;
	ret = disable_irq_wake(gpio_to_irq(pdata->modem.xmm.ipc_ap_wake));
	if (ret < 0)
		pr_err("%s: disable_irq_wake error\n", __func__);

	/* unregister usb host controller */
	if (pdata->hsic_unregister)
		pdata->hsic_unregister(data->hsic_device);
	else
		pr_err("%s: hsic_unregister is missing\n", __func__);


	/* set IPC_HSIC_ACTIVE low */
	gpio_set_value(pdata->modem.xmm.ipc_hsic_active, 0);

	/* wait 20 ms */
	mdelay(20);

	/* drive bb_rst low */
	gpio_set_value(pdata->modem.xmm.bb_rst, 0);
	mdelay(1);

	baseband_xmm_powerstate = BBXMM_PS_UNINIT;
	modem_sleep_flag = false;
	CP_initiated_L2toL0 = false;
	spin_lock_irqsave(&xmm_lock, flags);
	wakeup_pending = false;
	system_suspending = false;
	spin_unlock_irqrestore(&xmm_lock, flags);
	/* start registration process once again on xmm on */
	register_hsic_device = true;

	tegra_baseband_rail_off();
	pr_debug("%s }\n", __func__);

	return 0;
}

static ssize_t xmm_onoff(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int pwr;
	int size;
	struct platform_device *device = to_platform_device(dev);

	mutex_lock(&xmm_onoff_mutex);

	pr_debug("%s\n", __func__);

	/* check input */
	if (buf == NULL) {
		pr_err("%s: buf NULL\n", __func__);
		mutex_unlock(&xmm_onoff_mutex);
		return -EINVAL;
	}
	pr_debug("%s: count=%d\n", __func__, count);

	/* parse input */
	size = sscanf(buf, "%d", &pwr);
	if (size != 1) {
		pr_err("%s: size=%d -EINVAL\n", __func__, size);
		mutex_unlock(&xmm_onoff_mutex);
		return -EINVAL;
	}

	if (power_onoff == pwr) {
		pr_err("%s: Ignored, due to same CP power state(%d)\n",
						__func__, power_onoff);
		mutex_unlock(&xmm_onoff_mutex);
		return -EINVAL;
	}
	power_onoff = pwr;
	pr_debug("%s power_onoff=%d\n", __func__, power_onoff);

	if (power_onoff == 0)
		xmm_power_off(device);
	else if (power_onoff == 1)
		xmm_power_on(device);

	mutex_unlock(&xmm_onoff_mutex);

	return count;
}

static void pm_qos_worker(struct work_struct *work)
{
	pr_debug("%s - pm qos CPU back to normal\n", __func__);
	pm_qos_update_request(&boost_cpu_freq_req,
			(s32)PM_QOS_CPU_FREQ_MIN_DEFAULT_VALUE);
}

static DEVICE_ATTR(xmm_onoff, S_IRUSR | S_IWUSR | S_IRGRP,
		NULL, xmm_onoff);


void baseband_xmm_set_power_status(unsigned int status)
{
	struct baseband_power_platform_data *data = xmm_power_drv_data.pdata;
	int value = 0;
	unsigned long flags;

	if (baseband_xmm_powerstate == status)
		return;
	pr_debug("%s\n", __func__);
	switch (status) {
	case BBXMM_PS_L0:
		if (modem_sleep_flag) {
			pr_info("%s Resume from L3 without calling resume"
						"function\n",  __func__);
			xmm_power_driver_handle_resume(data);
		}
		pr_info("L0\n");
		baseband_xmm_powerstate = status;
		if (!wake_lock_active(&wakelock))
			wake_lock(&wakelock);
		value = gpio_get_value(data->modem.xmm.ipc_hsic_active);
		pr_debug("before L0 ipc_hsic_active=%d\n", value);
		if (!value) {
			pr_debug("before L0 gpio set ipc_hsic_active=1 ->\n");
			gpio_set_value(data->modem.xmm.ipc_hsic_active, 1);
		}
		if (modem_power_on) {
			modem_power_on = false;
			baseband_modem_power_on(data);
		}
		pr_debug("gpio host active high->\n");
		break;
	case BBXMM_PS_L2:
		pr_info("L2\n");
		baseband_xmm_powerstate = status;
		spin_lock_irqsave(&xmm_lock, flags);
		if (wakeup_pending) {
			spin_unlock_irqrestore(&xmm_lock, flags);
			xmm_power_L2_resume();
		 } else {
			spin_unlock_irqrestore(&xmm_lock, flags);
			if (wake_lock_active(&wakelock))
				wake_unlock(&wakelock);
			modem_sleep_flag = true;
		}
		break;
	case BBXMM_PS_L3:
		if (baseband_xmm_powerstate == BBXMM_PS_L2TOL0) {
			if (!data->modem.xmm.ipc_ap_wake) {
				spin_lock_irqsave(&xmm_lock, flags);
				wakeup_pending = true;
				spin_unlock_irqrestore(&xmm_lock, flags);
				pr_info("%s: L2 race condition-CP wakeup"
						" pending\n", __func__);
			}
		}
		pr_info("L3\n");
		/* system is going to suspend */
		if (baseband_xmm_powerstate == BBXMM_PS_L2)
			tegra_baseband_rail_off();

		baseband_xmm_powerstate = status;
		spin_lock_irqsave(&xmm_lock, flags);
		system_suspending = false;
		spin_unlock_irqrestore(&xmm_lock, flags);
		if (wake_lock_active(&wakelock)) {
			pr_info("%s: releasing wakelock before L3\n",
				__func__);
			wake_unlock(&wakelock);
		}
		gpio_set_value(data->modem.xmm.ipc_hsic_active, 0);
		pr_debug("gpio host active low->\n");
		break;
	case BBXMM_PS_L2TOL0:
		spin_lock_irqsave(&xmm_lock, flags);
		system_suspending = false;
		wakeup_pending = false;
		spin_unlock_irqrestore(&xmm_lock, flags);
		/* do this only from L2 state */
		if (baseband_xmm_powerstate == BBXMM_PS_L2) {
			baseband_xmm_powerstate = status;
			pr_debug("BB XMM POWER STATE = %d\n", status);
			xmm_power_L2_resume();
		}
		baseband_xmm_powerstate = status;
		break;
	case BBXMM_PS_L3TOL0:
		/* poweron rail for L3 -> L0 (system resume) */
		pr_debug("L3 -> L0, turning on power rail.\n");
		tegra_baseband_rail_on();
		baseband_xmm_powerstate = status;
		break;
	default:
		baseband_xmm_powerstate = status;
		break;
	}
	pr_debug("BB XMM POWER STATE = %d\n", status);
}
EXPORT_SYMBOL_GPL(baseband_xmm_set_power_status);

irqreturn_t xmm_power_ipc_ap_wake_irq(int irq, void *dev_id)
{
	struct baseband_power_platform_data *data = xmm_power_drv_data.pdata;
	int value;

	value = gpio_get_value(data->modem.xmm.ipc_ap_wake);
	pr_debug("%s g(%d), wake_st(%d)\n", __func__, value, ipc_ap_wake_state);

	/* modem initialization/bootup part*/
	if (unlikely(ipc_ap_wake_state < IPC_AP_WAKE_IRQ_READY)) {
		pr_err("%s - spurious irq\n", __func__);
		return IRQ_HANDLED;
	} else if (ipc_ap_wake_state == IPC_AP_WAKE_IRQ_READY) {
		if (!value) {
			pr_debug("%s - IPC_AP_WAKE_INIT1"
					" - got falling edge\n", __func__);
			/* go to IPC_AP_WAKE_INIT1 state */
			ipc_ap_wake_state = IPC_AP_WAKE_INIT1;
			queue_work(workqueue, &init1_work);
		} else
			pr_debug("%s - IPC_AP_WAKE_INIT1"
				" - wait for falling edge\n", __func__);
		return IRQ_HANDLED;
	} else if (ipc_ap_wake_state == IPC_AP_WAKE_INIT1) {
		if (!value) {
			pr_debug("%s - IPC_AP_WAKE_INIT2"
				" - wait for rising edge\n", __func__);
		} else {
			pr_debug("%s - IPC_AP_WAKE_INIT2"
					" - got rising edge\n",	__func__);
			/* go to IPC_AP_WAKE_INIT2 state */
			ipc_ap_wake_state = IPC_AP_WAKE_INIT2;
			queue_work(workqueue, &init2_work);
		}
		return IRQ_HANDLED;
	}

	/* modem wakeup part */
	if (!value) {
		pr_debug("%s - falling\n", __func__);
		/* First check it a CP ack or CP wake  */
		value = gpio_get_value(data->modem.xmm.ipc_bb_wake);
		if (value) {
			pr_debug("cp ack for bb_wake\n");
			ipc_ap_wake_state = IPC_AP_WAKE_L;
			return IRQ_HANDLED;
		}
		spin_lock(&xmm_lock);
		wakeup_pending = true;
		if (system_suspending) {
			spin_unlock(&xmm_lock);
			pr_info("Set wakeup_pending = 1 in system_"
					" suspending!!!\n");
		} else {
			if ((baseband_xmm_powerstate == BBXMM_PS_L3) ||
				(baseband_xmm_powerstate == BBXMM_PS_L3TOL0)) {
				spin_unlock(&xmm_lock);
				pr_info(" CP L3 -> L0\n");
			} else if (baseband_xmm_powerstate == BBXMM_PS_L2) {
				CP_initiated_L2toL0 = true;
				spin_unlock(&xmm_lock);
				baseband_xmm_set_power_status(BBXMM_PS_L2TOL0);
			} else {
				CP_initiated_L2toL0 = true;
				spin_unlock(&xmm_lock);
			}
		}
		/* save gpio state */
		ipc_ap_wake_state = IPC_AP_WAKE_L;
	} else {
		pr_debug("%s - rising\n", __func__);
		value = gpio_get_value(data->modem.xmm.ipc_hsic_active);
		if (!value) {
			pr_info("host active low: ignore request\n");
			ipc_ap_wake_state = IPC_AP_WAKE_H;
			return IRQ_HANDLED;
		}
		value = gpio_get_value(data->modem.xmm.ipc_bb_wake);
		if (value) {
			/* Clear the slave wakeup request */
			gpio_set_value(data->modem.xmm.ipc_bb_wake, 0);
			pr_debug("gpio slave wakeup done ->\n");
		}
		if (reenable_autosuspend && usbdev) {
			reenable_autosuspend = false;
			queue_work(workqueue, &autopm_resume_work);
		}
		modem_sleep_flag = false;
		baseband_xmm_set_power_status(BBXMM_PS_L0);
		/* save gpio state */
		ipc_ap_wake_state = IPC_AP_WAKE_H;
	}

	return IRQ_HANDLED;
}
EXPORT_SYMBOL(xmm_power_ipc_ap_wake_irq);

static void xmm_power_init1_work(struct work_struct *work)
{
	struct baseband_power_platform_data *pdata = xmm_power_drv_data.pdata;
	int value;

	pr_debug("%s {\n", __func__);

	/* check if IPC_HSIC_ACTIVE high */
	value = gpio_get_value(pdata->modem.xmm.ipc_hsic_active);
	if (value != 1) {
		pr_err("%s - expected IPC_HSIC_ACTIVE high!\n", __func__);
		return;
	}

	/* wait 100 ms */
	mdelay(100);

	/* set IPC_HSIC_ACTIVE low */
	gpio_set_value(pdata->modem.xmm.ipc_hsic_active, 0);

	/* wait 10 ms */
	mdelay(10);

	/* set IPC_HSIC_ACTIVE high */
	gpio_set_value(pdata->modem.xmm.ipc_hsic_active, 1);

	/* wait 20 ms */
	mdelay(20);

	pr_debug("%s }\n", __func__);
}

static void xmm_power_init2_work(struct work_struct *work)
{
	struct baseband_power_platform_data *pdata = xmm_power_drv_data.pdata;

	pr_debug("%s\n", __func__);

	/* check input */
	if (!pdata)
		return;

	/* register usb host controller only once */
	if (register_hsic_device) {
		if (pdata->hsic_register)
			xmm_power_drv_data.hsic_device = pdata->hsic_register();
		else
			pr_err("%s: hsic_register is missing\n", __func__);
		register_hsic_device = false;
	}

}

static void xmm_power_autopm_resume(struct work_struct *work)
{
	struct usb_interface *intf;

	pr_debug("%s\n", __func__);
	if (usbdev) {
		usb_lock_device(usbdev);
		intf = usb_ifnum_to_if(usbdev, 0);
		if (!intf) {
			usb_unlock_device(usbdev);
			return;
		}
		if (usb_autopm_get_interface(intf) == 0)
			usb_autopm_put_interface(intf);
		usb_unlock_device(usbdev);
	}
}


/* Do the work for AP/CP initiated L2->L0 */
static void xmm_power_L2_resume(void)
{
	struct baseband_power_platform_data *pdata = xmm_power_drv_data.pdata;
	int value;
	int delay = 10000; /* maxmum delay in msec */
	unsigned long flags;

	pr_debug("%s\n", __func__);

	if (!pdata)
		return;

	/* claim the wakelock here to avoid any system suspend */
	if (!wake_lock_active(&wakelock))
		wake_lock(&wakelock);
	modem_sleep_flag = false;
	spin_lock_irqsave(&xmm_lock, flags);
	wakeup_pending = false;
	spin_unlock_irqrestore(&xmm_lock, flags);

	if (CP_initiated_L2toL0)  {
		pr_info("CP L2->L0\n");
		CP_initiated_L2toL0 = false;
		queue_work(workqueue, &L2_resume_work);
	} else {
		/* set the slave wakeup request */
		pr_info("AP L2->L0\n");
		value = gpio_get_value(pdata->modem.xmm.ipc_ap_wake);
		if (value) {
			pr_debug("waiting for host wakeup from CP...\n");
			/* wake bb */
			gpio_set_value(pdata->modem.xmm.ipc_bb_wake, 1);
			do {
				mdelay(1);
				value = gpio_get_value(
						pdata->modem.xmm.ipc_ap_wake);
				delay--;
			} while ((value) && (delay));
			if (delay)
				pr_debug("gpio host wakeup low <-\n");
			else
				pr_info("!!AP L2->L0 Failed\n");
		} else {
			pr_info("CP already ready\n");
		}
	}
}

/* Do the work for CP initiated L2->L0 */
static void xmm_power_L2_resume_work(struct work_struct *work)
{
	struct usb_interface *intf;

	pr_debug("%s {\n", __func__);

	if (!usbdev)
		return;
	usb_lock_device(usbdev);
	intf = usb_ifnum_to_if(usbdev, 0);
	if (usb_autopm_get_interface(intf) == 0)
		usb_autopm_put_interface(intf);
	usb_unlock_device(usbdev);

	pr_debug("} %s\n", __func__);
}

static void xmm_power_reset_on(struct baseband_power_platform_data *pdata)
{
	/* reset / power on sequence */
	gpio_set_value(pdata->modem.xmm.bb_rst, 0);
	msleep(40);
	gpio_set_value(pdata->modem.xmm.bb_rst, 1);
	mdelay(1);
	gpio_set_value(pdata->modem.xmm.bb_on, 1);
	udelay(70);
	gpio_set_value(pdata->modem.xmm.bb_on, 0);
}


static void xmm_power_work_func(struct work_struct *work)
{
	struct xmm_power_data *data =
			container_of(work, struct xmm_power_data, work);
	struct baseband_power_platform_data *pdata = data->pdata;

	pr_debug("%s\n", __func__);

	switch (data->state) {
	case BBXMM_WORK_UNINIT:
		pr_debug("BBXMM_WORK_UNINIT\n");
		break;
	case BBXMM_WORK_INIT:
		pr_debug("BBXMM_WORK_INIT\n");
		/* go to next state */
		data->state = (modem_flash && !modem_pm)
			? BBXMM_WORK_INIT_FLASH_STEP1
			: (modem_flash && modem_pm)
			? BBXMM_WORK_INIT_FLASH_PM_STEP1
			: (!modem_flash && modem_pm)
			? BBXMM_WORK_INIT_FLASHLESS_PM_STEP1
			: BBXMM_WORK_UNINIT;
		pr_debug("Go to next state %d\n", data->state);
		queue_work(workqueue, work);
		break;
	case BBXMM_WORK_INIT_FLASH_STEP1:
		pr_debug("BBXMM_WORK_INIT_FLASH_STEP1\n");
		/* register usb host controller */
		pr_debug("%s: register usb host controller\n", __func__);
		if (pdata->hsic_register)
			data->hsic_device = pdata->hsic_register();
		else
			pr_err("%s: hsic_register is missing\n", __func__);
		break;
	case BBXMM_WORK_INIT_FLASH_PM_STEP1:
		pr_debug("BBXMM_WORK_INIT_FLASH_PM_STEP1\n");
		pr_debug("%s: ipc_hsic_active -> 0\n", __func__);
		gpio_set_value(pdata->modem.xmm.ipc_hsic_active, 1);
		/* reset / power on sequence */
		xmm_power_reset_on(pdata);
		/* set power status as on */
		power_onoff = 1;
		gpio_set_value(pdata->modem.xmm.ipc_hsic_active, 0);

		/* expecting init2 performs register hsic to enumerate modem
		 * software directly.
		 */
		break;
	case BBXMM_WORK_INIT_FLASHLESS_PM_STEP1:
		pr_debug("BBXMM_WORK_INIT_FLASHLESS_PM_STEP1\n");
		pr_info("%s: flashless is not supported here\n", __func__);
		break;
	default:
		break;
	}
}

static void xmm_device_add_handler(struct usb_device *udev)
{
	struct usb_interface *intf = usb_ifnum_to_if(udev, 0);
	const struct usb_device_id *id;

	if (intf == NULL)
		return;

	id = usb_match_id(intf, xmm_pm_ids);

	if (id) {
		pr_debug("persist_enabled: %u\n", udev->persist_enabled);
		pr_info("Add device %d <%s %s>\n", udev->devnum,
			udev->manufacturer, udev->product);
		usbdev = udev;
		usb_enable_autosuspend(udev);
		pr_info("enable autosuspend\n");
	}
}

static void xmm_device_remove_handler(struct usb_device *udev)
{
	if (usbdev == udev) {
		pr_info("Remove device %d <%s %s>\n", udev->devnum,
			udev->manufacturer, udev->product);
		usbdev = 0;
	}

}

static int usb_xmm_notify(struct notifier_block *self, unsigned long action,
			void *blob)
{
	switch (action) {
	case USB_DEVICE_ADD:
		xmm_device_add_handler(blob);
		break;
	case USB_DEVICE_REMOVE:
		xmm_device_remove_handler(blob);
		break;
	}

	return NOTIFY_OK;
}


static struct notifier_block usb_xmm_nb = {
	.notifier_call = usb_xmm_notify,
};

static int xmm_power_pm_notifier_event(struct notifier_block *this,
					unsigned long event, void *ptr)
{
	struct baseband_power_platform_data *pdata = xmm_power_drv_data.pdata;
	unsigned long flags;

	if (!pdata)
		return NOTIFY_DONE;

	pr_debug("%s: event %ld\n", __func__, event);
	switch (event) {
	case PM_SUSPEND_PREPARE:
		pr_debug("%s : PM_SUSPEND_PREPARE\n", __func__);
		if (wake_lock_active(&wakelock)) {
			pr_info("%s: wakelock was active, aborting suspend\n",
					__func__);
			return NOTIFY_STOP;
		}

		spin_lock_irqsave(&xmm_lock, flags);
		if (wakeup_pending) {
			wakeup_pending = false;
			spin_unlock_irqrestore(&xmm_lock, flags);
			pr_info("%s : XMM busy : Abort system suspend\n",
				 __func__);
			return NOTIFY_STOP;
		}
		system_suspending = true;
		spin_unlock_irqrestore(&xmm_lock, flags);
		return NOTIFY_OK;
	case PM_POST_SUSPEND:
		pr_debug("%s : PM_POST_SUSPEND\n", __func__);
		spin_lock_irqsave(&xmm_lock, flags);
		system_suspending = false;
		if (wakeup_pending &&
			(baseband_xmm_powerstate == BBXMM_PS_L2)) {
			wakeup_pending = false;
			spin_unlock_irqrestore(&xmm_lock, flags);
			pr_info("%s : Service Pending CP wakeup\n", __func__);
			CP_initiated_L2toL0 = true;
			baseband_xmm_set_power_status(BBXMM_PS_L2TOL0);
			return NOTIFY_OK;
		}
		wakeup_pending = false;
		spin_unlock_irqrestore(&xmm_lock, flags);
		return NOTIFY_OK;
	}
	return NOTIFY_DONE;
}

static struct notifier_block xmm_power_pm_notifier = {
	.notifier_call = xmm_power_pm_notifier_event,
};


static int xmm_power_driver_probe(struct platform_device *device)
{
	struct baseband_power_platform_data *pdata = device->dev.platform_data;
	struct device *dev = &device->dev;
	unsigned long flags;
	int err;

	pr_debug("%s\n", __func__);
	pr_debug("[XMM] enum_delay_ms=%ld\n", enum_delay_ms);

	/* check for platform data */
	if (!pdata)
		return -ENODEV;

	/* check if supported modem */
	if (pdata->baseband_type != BASEBAND_XMM) {
		pr_err("unsuppported modem\n");
		return -ENODEV;
	}

	/* save platform data */
	xmm_power_drv_data.pdata = pdata;

	/* create device file */
	err = device_create_file(dev, &dev_attr_xmm_onoff);
	if (err < 0) {
		pr_err("%s - device_create_file failed\n", __func__);
		return -ENODEV;
	}

	/* init wake lock */
	wake_lock_init(&wakelock, WAKE_LOCK_SUSPEND, "baseband_xmm_power");

	/* init spin lock */
	spin_lock_init(&xmm_lock);
	/* request baseband gpio(s) */
	tegra_baseband_gpios[0].gpio = pdata->modem.xmm.bb_rst;
	tegra_baseband_gpios[1].gpio = pdata->modem.xmm.bb_on;
	tegra_baseband_gpios[2].gpio = pdata->modem.xmm.ipc_bb_wake;
	tegra_baseband_gpios[3].gpio = pdata->modem.xmm.ipc_ap_wake;
	tegra_baseband_gpios[4].gpio = pdata->modem.xmm.ipc_hsic_active;
	tegra_baseband_gpios[5].gpio = pdata->modem.xmm.ipc_hsic_sus_req;
	err = gpio_request_array(tegra_baseband_gpios,
				ARRAY_SIZE(tegra_baseband_gpios));
	if (err < 0) {
		pr_err("%s - request gpio(s) failed\n", __func__);
		return -ENODEV;
	}

	/* request baseband irq(s) */
	if (modem_flash && modem_pm) {
		pr_debug("%s: request_irq IPC_AP_WAKE_IRQ\n", __func__);
		ipc_ap_wake_state = IPC_AP_WAKE_UNINIT;
		err = request_threaded_irq(
				gpio_to_irq(pdata->modem.xmm.ipc_ap_wake),
				NULL, xmm_power_ipc_ap_wake_irq,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				"IPC_AP_WAKE_IRQ", NULL);
		if (err < 0) {
			pr_err("%s - request irq IPC_AP_WAKE_IRQ failed\n",
				__func__);
			return err;
		}
		err = enable_irq_wake(gpio_to_irq(
					pdata->modem.xmm.ipc_ap_wake));
		if (err < 0)
			pr_err("%s: enable_irq_wake error\n", __func__);

		pr_debug("%s: AP_WAKE_INIT1\n", __func__);
		/* ver 1130 or later starts in INIT1 state */
		ipc_ap_wake_state = IPC_AP_WAKE_INIT1;
	}

	/* init work queue */
	workqueue = create_singlethread_workqueue("xmm_power_wq");
	if (!workqueue) {
		pr_err("cannot create workqueue\n");
		return -ENOMEM;
	}

	INIT_WORK(&xmm_power_drv_data.work, xmm_power_work_func);
	xmm_power_drv_data.state = BBXMM_WORK_INIT;
	queue_work(workqueue, &xmm_power_drv_data.work);

	/* init work objects */
	INIT_WORK(&init1_work, xmm_power_init1_work);
	INIT_WORK(&init2_work, xmm_power_init2_work);
	INIT_WORK(&L2_resume_work, xmm_power_L2_resume_work);
	INIT_WORK(&autopm_resume_work, xmm_power_autopm_resume);

	/* init state variables */
	register_hsic_device = true;
	CP_initiated_L2toL0 = false;
	baseband_xmm_powerstate = BBXMM_PS_UNINIT;
	spin_lock_irqsave(&xmm_lock, flags);
	wakeup_pending = false;
	system_suspending = false;
	spin_unlock_irqrestore(&xmm_lock, flags);

	usb_register_notify(&usb_xmm_nb);
	register_pm_notifier(&xmm_power_pm_notifier);

	pr_debug("%s }\n", __func__);
	return 0;
}

static int xmm_power_driver_remove(struct platform_device *device)
{
	struct baseband_power_platform_data *pdata = device->dev.platform_data;
	struct xmm_power_data *data = &xmm_power_drv_data;
	struct device *dev = &device->dev;

	pr_debug("%s\n", __func__);

	/* check for platform data */
	if (!pdata)
		return 0;

	unregister_pm_notifier(&xmm_power_pm_notifier);
	usb_unregister_notify(&usb_xmm_nb);

	/* free baseband irq(s) */
	if (modem_flash && modem_pm) {
		free_irq(gpio_to_irq(pdata->modem.xmm.ipc_ap_wake), NULL);
	}

	/* free baseband gpio(s) */
	gpio_free_array(tegra_baseband_gpios,
		ARRAY_SIZE(tegra_baseband_gpios));

	/* destroy wake lock */
	wake_lock_destroy(&wakelock);

	/* delete device file */
	device_remove_file(dev, &dev_attr_xmm_onoff);

	/* unregister usb host controller */
	if (pdata->hsic_unregister)
		pdata->hsic_unregister(data->hsic_device);
	else
		pr_err("%s: hsic_unregister is missing\n", __func__);

	return 0;
}

static int xmm_power_driver_handle_resume(
			struct baseband_power_platform_data *pdata)
{
	int value;
	int delay = 1000; /* maxmum delay in msec */
	unsigned long flags;

	pr_debug("%s\n", __func__);
	if (!pdata)
		return 0;

	/* check if modem is on */
	if (power_onoff == 0) {
		pr_debug("%s - flight mode - nop\n", __func__);
		return 0;
	}

	modem_sleep_flag = false;
	spin_lock_irqsave(&xmm_lock, flags);
	wakeup_pending = false;
	spin_unlock_irqrestore(&xmm_lock, flags);

	/* L3->L0 */
	baseband_xmm_set_power_status(BBXMM_PS_L3TOL0);
	value = gpio_get_value(pdata->modem.xmm.ipc_ap_wake);
	if (value) {
		pr_info("AP L3 -> L0\n");
		pr_debug("waiting for host wakeup...\n");
		/* wake bb */
		gpio_set_value(pdata->modem.xmm.ipc_bb_wake, 1);
		do {
			mdelay(1);
			value = gpio_get_value(pdata->modem.xmm.ipc_ap_wake);
			delay--;
		} while ((value) && (delay));
		if (delay)
			pr_debug("gpio host wakeup low <-\n");
		else
			pr_info("!!AP L3->L0 Failed\n");

	} else {
		pr_info("CP L3 -> L0\n");
	}
	reenable_autosuspend = true;

	return 0;

}


#ifdef CONFIG_PM
static int xmm_power_driver_suspend(struct device *dev)
{
	pr_debug("%s\n", __func__);
	return 0;
}

static int xmm_power_driver_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct baseband_power_platform_data *pdata = pdev->dev.platform_data;

	pr_debug("%s\n", __func__);
	xmm_power_driver_handle_resume(pdata);

	return 0;
}

static int xmm_power_suspend_noirq(struct device *dev)
{
	unsigned long flags;

	pr_debug("%s\n", __func__);
	spin_lock_irqsave(&xmm_lock, flags);
	system_suspending = false;
	if (wakeup_pending) {
		wakeup_pending = false;
		spin_unlock_irqrestore(&xmm_lock, flags);
		pr_info("%s:**Abort Suspend: reason CP WAKEUP**\n", __func__);
		return -EBUSY;
	}
	spin_unlock_irqrestore(&xmm_lock, flags);
	return 0;
}

static int xmm_power_resume_noirq(struct device *dev)
{
	pr_debug("%s\n", __func__);
	return 0;
}

static const struct dev_pm_ops xmm_power_dev_pm_ops = {
	.suspend_noirq = xmm_power_suspend_noirq,
	.resume_noirq = xmm_power_resume_noirq,
	.suspend = xmm_power_driver_suspend,
	.resume = xmm_power_driver_resume,
};
#endif

static void xmm_power_driver_shutdown(struct platform_device *device)
{
	struct baseband_power_platform_data *pdata = device->dev.platform_data;

	pr_debug("%s\n", __func__);
	disable_irq(gpio_to_irq(pdata->modem.xmm.ipc_ap_wake));
	/* bb_on is already down, to make sure set 0 again */
	gpio_set_value(pdata->modem.xmm.bb_on, 0);
	gpio_set_value(pdata->modem.xmm.bb_rst, 0);
	return;
}

static struct platform_driver baseband_power_driver = {
	.probe = xmm_power_driver_probe,
	.remove = xmm_power_driver_remove,
	.shutdown = xmm_power_driver_shutdown,
	.driver = {
		.name = "baseband_xmm_power",
#ifdef CONFIG_PM
		.pm   = &xmm_power_dev_pm_ops,
#endif
	},
};

static int __init xmm_power_init(void)
{
	pr_debug("%s\n", __func__);

	INIT_DELAYED_WORK(&pm_qos_work, pm_qos_worker);
	pm_qos_add_request(&boost_cpu_freq_req, PM_QOS_CPU_FREQ_MIN,
			(s32)PM_QOS_CPU_FREQ_MIN_DEFAULT_VALUE);

	return platform_driver_register(&baseband_power_driver);
}

static void __exit xmm_power_exit(void)
{
	pr_debug("%s\n", __func__);
	platform_driver_unregister(&baseband_power_driver);

	pm_qos_remove_request(&boost_cpu_freq_req);
}

module_init(xmm_power_init)
module_exit(xmm_power_exit)
