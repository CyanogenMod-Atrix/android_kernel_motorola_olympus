/*
 * drivers/misc/bcm4329_rfkill.c
 *
 * Copyright (c) 2010, NVIDIA Corporation.
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

#include <linux/err.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/rfkill.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/irq.h>
#include <linux/serial_core.h>
#include <linux/wakelock.h>

struct bcm4329_rfkill_data {
	int gpio_reset;
	int gpio_shutdown;
	int gpio_wake;
	int gpio_host_wake;
	int delay;
	struct clk *bt_32k_clk;
};

static struct bcm4329_rfkill_data *bcm4329_rfkill;
static struct rfkill *bt_rfkill;

static int bcm4329_bt_rfkill_set_power(void *data, bool blocked)
{
	/*
	 * check if BT gpio_shutdown line status and current request are same.
	 * If same, then return, else perform requested operation.
	 */
	if (gpio_get_value(bcm4329_rfkill->gpio_shutdown) && !blocked)
		return 0;

	if (blocked) {
		if (bcm4329_rfkill->gpio_shutdown)
			gpio_direction_output(bcm4329_rfkill->gpio_shutdown, 0);
		if (bcm4329_rfkill->gpio_reset)
			gpio_direction_output(bcm4329_rfkill->gpio_reset, 0);
		if (bcm4329_rfkill->bt_32k_clk)
			clk_disable(bcm4329_rfkill->bt_32k_clk);
	} else {
		if (bcm4329_rfkill->bt_32k_clk)
			clk_enable(bcm4329_rfkill->bt_32k_clk);
		if (bcm4329_rfkill->gpio_shutdown)
		{
			gpio_direction_output(bcm4329_rfkill->gpio_shutdown, 0);
			msleep(100);
			gpio_direction_output(bcm4329_rfkill->gpio_shutdown, 1);
			msleep(100);
		}
		if (bcm4329_rfkill->gpio_reset)
		{
			gpio_direction_output(bcm4329_rfkill->gpio_reset, 0);
			msleep(100);
			gpio_direction_output(bcm4329_rfkill->gpio_reset, 1);
			msleep(100);
		}
	}

	return 0;
}

static const struct rfkill_ops bcm4329_bt_rfkill_ops = {
	.set_block = bcm4329_bt_rfkill_set_power,
};

struct bcm_bt_lpm {
	int wake;
	int host_wake;
	bool rx_wake_lock_taken;

	struct hrtimer enter_lpm_timer;
	ktime_t enter_lpm_delay;

	struct uart_port *uport;

	struct wake_lock wake_lock;
	char wake_lock_name[100];
} bt_lpm;

static void set_wake_locked(int wake)
{
	bt_lpm.wake = wake;

	if (!wake)
		wake_unlock(&bt_lpm.wake_lock);

	gpio_set_value(bcm4329_rfkill->gpio_wake, wake);
}

static enum hrtimer_restart enter_lpm(struct hrtimer *timer)
{
	unsigned long flags;
	spin_lock_irqsave(&bt_lpm.uport->lock, flags);
	set_wake_locked(0);
	spin_unlock_irqrestore(&bt_lpm.uport->lock, flags);
	pr_info("%s: gpio_wake set low\n",__func__);

	return HRTIMER_NORESTART;
}

void bcm_bt_lpm_exit_lpm_locked(struct uart_port *uport)
{
	bt_lpm.uport = uport;


	hrtimer_try_to_cancel(&bt_lpm.enter_lpm_timer);

	set_wake_locked(1);

	hrtimer_start(&bt_lpm.enter_lpm_timer, bt_lpm.enter_lpm_delay,
		HRTIMER_MODE_REL);

}
EXPORT_SYMBOL(bcm_bt_lpm_exit_lpm_locked);

void bcm_bt_rx_done_locked(struct uart_port *uport)
{


	if (bt_lpm.host_wake) {
		/* Release wake in 500 ms so that higher layers can take it */
		wake_lock_timeout(&bt_lpm.wake_lock, HZ/2);
		bt_lpm.rx_wake_lock_taken = true;
	}

}
EXPORT_SYMBOL(bcm_bt_rx_done_locked);

static void update_host_wake_locked(int host_wake)
{
	if (host_wake == bt_lpm.host_wake)
		return;

	bt_lpm.host_wake = host_wake;

	if (host_wake) {
		bt_lpm.rx_wake_lock_taken = false;
		wake_lock(&bt_lpm.wake_lock);
	} else if (!bt_lpm.rx_wake_lock_taken) {
		/* Failsafe timeout of wakelock.
		   If the host wake pin is asserted and no data is sent,
		   when its deasserted we will enter this path */
		wake_lock_timeout(&bt_lpm.wake_lock, HZ/2);
	}

}

static irqreturn_t host_wake_isr(int irq, void *dev)
{
	int host_wake;
	unsigned long flags;

	host_wake = gpio_get_value(bcm4329_rfkill->gpio_host_wake);
	irq_set_irq_type(irq, host_wake ? IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH);
//	irq_set_irq_type(irq, IRQF_TRIGGER_FALLING);

	if (!bt_lpm.uport) {
		bt_lpm.host_wake = host_wake;
		return IRQ_HANDLED;
	}

	spin_lock_irqsave(&bt_lpm.uport->lock, flags);
	update_host_wake_locked(host_wake);
	spin_unlock_irqrestore(&bt_lpm.uport->lock, flags);

	return IRQ_HANDLED;
}

static int bcm4329_bt_lpm_init(struct platform_device *pdev)
{
	int irq;
	int ret;
	int rc;

	tegra_gpio_enable(bcm4329_rfkill->gpio_wake);
	rc = gpio_request(bcm4329_rfkill->gpio_wake, "bcm4329_wake_gpio");
	pr_info("%s: bcm4329_wake_gpio (%d) request: rc=%d\n",__func__, bcm4329_rfkill->gpio_wake, rc);
	if (unlikely(rc)) {
		tegra_gpio_disable(bcm4329_rfkill->gpio_wake);
		return rc;
	}
	tegra_gpio_enable(bcm4329_rfkill->gpio_host_wake);
	rc = gpio_request(bcm4329_rfkill->gpio_host_wake,
				"bcm4329_host_wake_gpio");
	pr_info("%s: bcm4329_host_wake_gpio (%d) request: rc=%d\n",__func__, bcm4329_rfkill->gpio_host_wake, rc);
	if (unlikely(rc)) {
		tegra_gpio_disable(bcm4329_rfkill->gpio_wake);
		tegra_gpio_disable(bcm4329_rfkill->gpio_host_wake);
		gpio_free(bcm4329_rfkill->gpio_wake);
		return rc;
	}

	hrtimer_init(&bt_lpm.enter_lpm_timer, CLOCK_MONOTONIC,
					HRTIMER_MODE_REL);
	bt_lpm.enter_lpm_delay = ktime_set(1, 0);  /* 1 sec */
	bt_lpm.enter_lpm_timer.function = enter_lpm;

	bt_lpm.host_wake = 0;

	irq = gpio_to_irq(bcm4329_rfkill->gpio_host_wake);
	ret = request_irq(irq, host_wake_isr, IRQF_TRIGGER_HIGH,
//	ret = request_irq(irq, host_wake_isr, IRQF_TRIGGER_FALLING,
				"bt host_wake", NULL);
	if (ret) {
		pr_info("%s: request_irq problem, ret=%d\n",__func__, ret);
		tegra_gpio_disable(bcm4329_rfkill->gpio_wake);
		tegra_gpio_disable(bcm4329_rfkill->gpio_host_wake);

		gpio_free(bcm4329_rfkill->gpio_wake);
		gpio_free(bcm4329_rfkill->gpio_host_wake);
		return ret;
	}

	ret = irq_set_irq_wake(irq, 1);
	if (ret) {
		pr_info("%s: irq_set_irq_wake problem, ret=%d\n",__func__, ret);
		tegra_gpio_disable(bcm4329_rfkill->gpio_wake);
		tegra_gpio_disable(bcm4329_rfkill->gpio_host_wake);

		gpio_free(bcm4329_rfkill->gpio_wake);
		gpio_free(bcm4329_rfkill->gpio_host_wake);
		return ret;
	}

	gpio_direction_output(bcm4329_rfkill->gpio_wake, 0);
	gpio_set_value(bcm4329_rfkill->gpio_wake, 0);
	gpio_direction_input(bcm4329_rfkill->gpio_host_wake);

	snprintf(bt_lpm.wake_lock_name, sizeof(bt_lpm.wake_lock_name),
			"BTLowPower");
	wake_lock_init(&bt_lpm.wake_lock, WAKE_LOCK_SUSPEND,
			 bt_lpm.wake_lock_name);
	return 0;
}

static int bcm4329_rfkill_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret;
	bool enable = false;  /* off */
	bool default_sw_block_state;

	bcm4329_rfkill = kzalloc(sizeof(*bcm4329_rfkill), GFP_KERNEL);
	if (!bcm4329_rfkill)
		return -ENOMEM;

	bcm4329_rfkill->bt_32k_clk = clk_get(&pdev->dev, "bcm4329_32k_clk");
	bcm4329_rfkill->bt_32k_clk = clk_get(&pdev->dev, "blink");
	if (IS_ERR(bcm4329_rfkill->bt_32k_clk)) {
		pr_warn("%s: can't find bcm4329_32k_clk.\
				assuming 32k clock to chip\n", __func__);
		bcm4329_rfkill->bt_32k_clk = NULL;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_IO,
						"bcm4329_nreset_gpio");
	if (res) {
		bcm4329_rfkill->gpio_reset = res->start;
		printk(KERN_INFO "%s: gpio_reset: %d \n", __func__, bcm4329_rfkill->gpio_reset);
		tegra_gpio_enable(bcm4329_rfkill->gpio_reset);
		ret = gpio_request(bcm4329_rfkill->gpio_reset,
						"bcm4329_nreset_gpio");
	} else {
		pr_warn("%s : can't find reset gpio.\n", __func__);
		bcm4329_rfkill->gpio_reset = 0;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_IO,
						"bcm4329_nshutdown_gpio");
	if (res) {
		bcm4329_rfkill->gpio_shutdown = res->start;
		printk(KERN_INFO "%s: gpio_shutdown: %d \n", __func__, bcm4329_rfkill->gpio_shutdown);
		tegra_gpio_enable(bcm4329_rfkill->gpio_shutdown);
		ret = gpio_request(bcm4329_rfkill->gpio_shutdown,
						"bcm4329_nshutdown_gpio");
	} else {
		pr_warn("%s : can't find shutdown gpio.\n", __func__);
		bcm4329_rfkill->gpio_shutdown = 0;
	}

	/* make sure at-least one of the GPIO is defined */
	if (!bcm4329_rfkill->gpio_reset && !bcm4329_rfkill->gpio_shutdown)
		goto free_bcm_res;

	if (bcm4329_rfkill->bt_32k_clk && enable)
		clk_enable(bcm4329_rfkill->bt_32k_clk);
	if (bcm4329_rfkill->gpio_shutdown)
		gpio_direction_output(bcm4329_rfkill->gpio_shutdown, enable);
	if (bcm4329_rfkill->gpio_reset)
		gpio_direction_output(bcm4329_rfkill->gpio_reset, enable);

	bt_rfkill = rfkill_alloc("bcm4329 Bluetooth", &pdev->dev,
				RFKILL_TYPE_BLUETOOTH, &bcm4329_bt_rfkill_ops,
				NULL);

	if (unlikely(!bt_rfkill))
		goto free_bcm_res;

	default_sw_block_state = !enable;
	rfkill_set_states(bt_rfkill, default_sw_block_state, false);

	ret = rfkill_register(bt_rfkill);

	if (unlikely(ret)) {
		rfkill_destroy(bt_rfkill);
		goto free_bcm_res;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_IO,
						"bcm4329_wake_gpio");
	if (res) {
		bcm4329_rfkill->gpio_wake = res->start;
	} else {
		pr_warn("%s : can't find wake gpio.\n", __func__);
		bcm4329_rfkill->gpio_wake = 0;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_IO,
						"bcm4329_host_wake_gpio");
	if (res) {
		bcm4329_rfkill->gpio_host_wake = res->start;
	} else {
		pr_warn("%s : can't find host wake gpio.\n", __func__);
		bcm4329_rfkill->gpio_host_wake = 0;
	}

	ret = bcm4329_bt_lpm_init(pdev);

	if (unlikely(ret)) {
		goto free_lpm_res;
	}

	return 0;

free_lpm_res:
	if (bcm4329_rfkill->gpio_wake)
		gpio_free(bcm4329_rfkill->gpio_wake);
	if (bcm4329_rfkill->gpio_host_wake)
		gpio_free(bcm4329_rfkill->gpio_host_wake);

free_bcm_res:
	if (bcm4329_rfkill->gpio_shutdown)
		gpio_free(bcm4329_rfkill->gpio_shutdown);
	if (bcm4329_rfkill->gpio_reset)
		gpio_free(bcm4329_rfkill->gpio_reset);
	if (bcm4329_rfkill->bt_32k_clk && enable)
		clk_disable(bcm4329_rfkill->bt_32k_clk);
	if (bcm4329_rfkill->bt_32k_clk)
		clk_put(bcm4329_rfkill->bt_32k_clk);
	kfree(bcm4329_rfkill);
	return -ENODEV;
}

static int bcm4329_rfkill_remove(struct platform_device *pdev)
{
	struct rfkill *bt_rfkill = platform_get_drvdata(pdev);

	if (bcm4329_rfkill->bt_32k_clk)
		clk_put(bcm4329_rfkill->bt_32k_clk);
	rfkill_unregister(bt_rfkill);
	rfkill_destroy(bt_rfkill);
	if (bcm4329_rfkill->gpio_shutdown)
		gpio_free(bcm4329_rfkill->gpio_shutdown);
	if (bcm4329_rfkill->gpio_reset)
		gpio_free(bcm4329_rfkill->gpio_reset);

	if (bcm4329_rfkill->gpio_wake)
		gpio_free(bcm4329_rfkill->gpio_wake);
	if (bcm4329_rfkill->gpio_host_wake)
		gpio_free(bcm4329_rfkill->gpio_host_wake);

	kfree(bcm4329_rfkill);

	return 0;
}

static struct platform_driver bcm4329_rfkill_driver = {
	.probe = bcm4329_rfkill_probe,
	.remove = bcm4329_rfkill_remove,
	.driver = {
		   .name = "bcm4329_rfkill",
		   .owner = THIS_MODULE,
	},
};

static int __init bcm4329_rfkill_init(void)
{
	return platform_driver_register(&bcm4329_rfkill_driver);
}

static void __exit bcm4329_rfkill_exit(void)
{
	platform_driver_unregister(&bcm4329_rfkill_driver);
}

module_init(bcm4329_rfkill_init);
module_exit(bcm4329_rfkill_exit);

MODULE_DESCRIPTION("BCM4329 rfkill");
MODULE_AUTHOR("NVIDIA");
MODULE_LICENSE("GPL");

#if defined CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#include <linux/delay.h>

#define GET_BT_POWER	  (bcm4329_rfkill ? gpio_get_value(bcm4329_rfkill->gpio_shutdown) : 0)
#define GET_BT_RESET	  (bcm4329_rfkill ? gpio_get_value(bcm4329_rfkill->gpio_reset) : 0)
#define GET_BT_WAKE	  (bcm4329_rfkill ? gpio_get_value(bcm4329_rfkill->gpio_wake) : -1)
#define GET_BT_HOST_WAKE  (bcm4329_rfkill ? gpio_get_value(bcm4329_rfkill->gpio_host_wake) : -1)

#define SET_BT_POWER(a)	  { if (bcm4329_rfkill) gpio_set_value(bcm4329_rfkill->gpio_shutdown, a); msleep(100); }
#define SET_BT_RESET(a)   { if (bcm4329_rfkill) gpio_set_value(bcm4329_rfkill->gpio_reset, a); msleep(100); }
#define SET_BT_WAKE(a)	  { if (bcm4329_rfkill) gpio_set_value(bcm4329_rfkill->gpio_wake, a); msleep(100); }

static char power_buf[1];
// write '1' to turn on power, '0' to turn it off
// write 'w' to set BT WAKE to 0,  'W' to set BT WAKE to 1
static ssize_t bcm4329_power_write(struct file *file, char __user *userbuf, size_t count, loff_t *ppos)
{
	int r;
	if(copy_from_user(power_buf, userbuf, 1)) {return(-EFAULT);}
	if('w' == *power_buf) {
		SET_BT_WAKE(0);
		printk(KERN_INFO "BLUETOOTH: BT_WAKE: low\n");
		return 0;
	}
	if('W' == *power_buf) {
		SET_BT_WAKE(1);
		printk(KERN_INFO "BLUETOOTH: BT_WAKE: high\n");
		return 0;
	}
	r = GET_BT_POWER;
	if('1' == *power_buf) {
		if(! r) {
			//rfkill_set_states(bt_rfkill, false, false);
			SET_BT_POWER(1);  // turn power on to BT module
			SET_BT_RESET(1);  // take BT out of reset
			printk(KERN_INFO "BLUETOOTH: power ON\n");
		}
	} else {
		if(r) {
			//rfkill_set_states(bt_rfkill, true, false);
			SET_BT_RESET(0);  // put BT into reset
			SET_BT_POWER(0);  // turn off internal BT VREG
			printk(KERN_INFO "BLUETOOTH: power OFF\n");
		}
	}
	return 0;
}

static ssize_t bcm4329_power_read(struct file *file, char __user *userbuf, size_t count, loff_t *ppos)
{
  char *str;
  int len;
  int r;

  r = GET_BT_POWER;
  if(r) {
	str = "1 (Bluetooth power is ON)\n";
  } else {
	str = "0 (Bluetooth power is OFF)\n";
  }

  len = strlen(str);
  if (count < len) return -EINVAL;
  return simple_read_from_buffer(userbuf, count, ppos, str, len);
}

static ssize_t bcm4329_status_read(struct file *file, char __user *userbuf, size_t count, loff_t *ppos)
{
  char str[64*5];
  char line[64];
  int len;

  str[0]=0;

  sprintf(line,"%d = BT_RESET\n", GET_BT_RESET);	strcat(str,line);
  sprintf(line,"%d = BT_POWER\n", GET_BT_POWER);	strcat(str,line);
  sprintf(line,"%d = BT_EXTWAKE\n", GET_BT_WAKE);	strcat(str,line);
  sprintf(line,"%d = BT_HOST_WAKE\n", GET_BT_HOST_WAKE); strcat(str,line);

  len = strlen(str);
  if (count < len) return -EINVAL;
  return simple_read_from_buffer(userbuf, count, ppos, str, len);
}

static const struct file_operations bcm4329_power_fops = {
	.read	= bcm4329_power_read,
	.write	= bcm4329_power_write,
};

static const struct file_operations bcm4329_status_fops = {
	.read	= bcm4329_status_read,
};

static struct dentry *bcm4329_root;

static int __init bcm4329_debugfs_init(void)
{
	struct dentry *direntry;

	bcm4329_root = debugfs_create_dir("bcm4329-bt", NULL);
	if (!bcm4329_root)
		return -ENODEV;

	direntry = debugfs_create_file("status", S_IRUGO,
			bcm4329_root, NULL, &bcm4329_status_fops);
	if (!direntry) {
		debugfs_remove_recursive(bcm4329_root);
		bcm4329_root = NULL;
		return -ENODEV;
	}

	direntry = debugfs_create_file("power", S_IRUGO | S_IWUGO,
			bcm4329_root, NULL, &bcm4329_power_fops);
	if(!direntry) {
		debugfs_remove_recursive(bcm4329_root);
		bcm4329_root = NULL;
		return -ENODEV;
	}

	return 0;
}

void bcm4329_debugfs_remove(void)
{
	if (bcm4329_root)
		debugfs_remove_recursive(bcm4329_root);
}

subsys_initcall(bcm4329_debugfs_init);

#endif /* if defined CONFIG_DEBUG_FS */
