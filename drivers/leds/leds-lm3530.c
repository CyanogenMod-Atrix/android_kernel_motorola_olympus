/*
 * Copyright (C) 2008-2009 Motorola, Inc.
 * Author: Alina Yakovleva <qvdh43@motorola.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2 
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */

// Linux  driver for    LM3530 display backlight

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <linux/clk.h>
#include <linux/irq.h>
#include <linux/sysfs.h>
#include <linux/hwmon-sysfs.h>
#include <linux/gpio_event.h>
#include <linux/leds.h>
#include <linux/mutex.h>
#include <linux/debugfs.h>
#include <linux/list.h>
#include <linux/leds-lm3530.h>
#ifdef  CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#define MODULE_NAME "leds_lm3530"

static unsigned trip_points[] = {0x02, 0xBD, 0xFE, 0xFF};
module_param_array(trip_points, uint, NULL, 0644);
static unsigned resistor_value = 0x30;
module_param(resistor_value, uint, 0644);

static unsigned ramp_time = 200000;

module_param(ramp_time, uint, 0644);
enum {
    TRACE_SUSPEND = 0x1,
    TRACE_ALS = 0x2,
    TRACE_BRIGHTNESS = 0x4,
    TRACE_WRITE = 0x8,
    TRACE_EVENT = 0x10,
};
unsigned do_trace = TRACE_ALS | TRACE_SUSPEND | TRACE_BRIGHTNESS | TRACE_WRITE;
module_param(do_trace, uint, 0644);

#define printk_write(fmt,args...) if (do_trace & TRACE_WRITE) printk(KERN_INFO fmt, ##args)
#define printk_br(fmt,args...) if (do_trace & TRACE_BRIGHTNESS) printk(KERN_INFO fmt, ##args)
#define printk_als(fmt,args...) if (do_trace & TRACE_ALS) printk(KERN_INFO fmt, ##args)
#define printk_suspend(fmt,args...) if (do_trace & TRACE_SUSPEND) printk(KERN_INFO fmt, ##args)
#define printk_event(fmt,args...) if (do_trace & TRACE_EVENT) printk(KERN_INFO fmt, ##args)

struct lm3530_data {
    uint16_t addr;
    struct i2c_client *client;
    struct lm3530_platform_data *pdata;
    struct led_classdev led_dev;
    struct led_classdev led_dev_nr;
    struct early_suspend early_suspend;
    unsigned initialized;
    int revision;
    atomic_t als_zone;     // Current ALS zone
    atomic_t bright_zone;  // Current brightness zone, diff. from ALS
    atomic_t in_suspend;   // Whether the driver is in TCMD SUSPEND mode
    unsigned bvalue;       // Current brightness register value
    unsigned saved_bvalue; // Brightness before TCMD SUSPEND
#ifdef CONFIG_LM3530_ALS
    struct input_dev *idev;
    struct work_struct work;
#endif
};

#ifdef CONFIG_LM3530_ALS
static void lm3530_send_als_event (struct lm3530_data *driver_data, int zone);
static void lm3530_work_func (struct work_struct *work);
static irqreturn_t lm3530_irq_handler (int irq, void *dev_id);
#endif
static const char *lm3530_reg_name (int reg);
static int lm3530_configure (struct lm3530_data *driver_data);
static int lm3530_write_reg (struct i2c_client *client, 
    unsigned reg, uint8_t value, const char *caller);
static int lm3530_read_reg (struct i2c_client *client,
    unsigned reg, uint8_t *value);
static int lm3530_set_ramp (struct i2c_client *client, 
    unsigned int on, unsigned int nsteps, unsigned int *rtime);
static int lm3530_probe(struct i2c_client *client, 
    const struct i2c_device_id *id);
static int lm3530_setup (struct lm3530_data *driver_data);
static int lm3530_remove (struct i2c_client *client);
static void lm3530_brightness_set(struct led_classdev *led_cdev,
                enum led_brightness value);
#ifdef CONFIG_PM
#ifdef CONFIG_HAS_EARLYSUSPEND
static void lm3530_early_suspend (struct early_suspend *h);
static void lm3530_late_resume (struct early_suspend *h);
#endif
static int lm3530_suspend (struct i2c_client *client, pm_message_t mesg);
static int lm3530_resume (struct i2c_client *client);
#endif

static const struct i2c_device_id lm3530_id[] = {
    { LM3530_NAME, 0 },
    { }
};

/* This is the I2C driver that will be inserted */
static struct i2c_driver lm3530_driver =
{
    .driver = {
        .name   = LM3530_NAME,
    },
    .id_table = lm3530_id,
    .probe = lm3530_probe,
    .remove  = lm3530_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
    .suspend    = lm3530_suspend,
    .resume     = lm3530_resume,
#endif
};

static DEFINE_MUTEX(lm3530_mutex);

struct lm3530_reg {
    const char *name;
    uint8_t reg;
} lm3530_regs[] = {
    { "GEN_CONFIG",           LM3530_GEN_CONFIG_REG },
    { "ALS_CONFIG",           LM3530_ALS_CONFIG_REG },
    { "VERSION_REG",          LM3530_VERSION_REG },
    { "BRIGHTNESS_RAMP_RATE", LM3530_RAMP_REG },
    { "ALS_ZONE_REG",         LM3530_ALS_ZONE_REG },
    { "ALS_RESISTOR_SELECT",  LM3530_ALS_RESISTOR_SELECT },
    { "BRIGHTNESS_CTRL_REG",  LM3530_BRIGHTNESS_CTRL_REG },
    { "ALS_ZB0_REG",          LM3530_ALS_ZB0_REG },
    { "ALS_ZB1_REG",          LM3530_ALS_ZB1_REG },
    { "ALS_ZB2_REG",          LM3530_ALS_ZB2_REG },
    { "ALS_ZB3_REG",          LM3530_ALS_ZB3_REG },
    { "ALS_Z0T_REG",          LM3530_ALS_Z0T_REG },
    { "ALS_Z1T_REG",          LM3530_ALS_Z1T_REG },
    { "ALS_Z2T_REG",          LM3530_ALS_Z2T_REG },
    { "ALS_Z3T_REG",          LM3530_ALS_Z3T_REG },
    { "ALS_Z4T_REG",          LM3530_ALS_Z4T_REG },
};

#if 0
int lm3530_register_als_callback(als_cb func, uint32_t cookie)
{
    struct als_callback *c;

    //printk (KERN_INFO "%s: enter\n", __FUNCTION__);
    c = kzalloc (sizeof (struct als_callback), GFP_KERNEL);
    if (c == NULL) {
        printk (KERN_ERR "%s: unable to register ALS callback: kzalloc\n",
            __FUNCTION__);
        return -ENOMEM;
    }
    c->cb = func;
    c->cookie = cookie;
    mutex_lock (&als_cb_mutex);
    list_add (&c->entry, &als_callbacks);
    mutex_unlock (&als_cb_mutex);
    return 0;
}
EXPORT_SYMBOL(lm3530_register_als_callback);

void lm3530_unregister_als_callback (als_cb func)
{
    struct als_callback *c;

    if (!lm3530_data.initialized) {
        printk (KERN_ERR "%s: not initialized\n", __FUNCTION__);
        return;
    }
    printk (KERN_INFO "%s: enter\n", __FUNCTION__);
    mutex_lock (&als_cb_mutex);
    list_for_each_entry(c, &als_callbacks, entry) {
        if (c->cb == func) {
            list_del (&c->entry);
            mutex_unlock (&als_cb_mutex);
            return;
        }
    }
    mutex_unlock (&als_cb_mutex);
    printk (KERN_ERR "%s: callback 0x%x not found\n", 
        __FUNCTION__, (unsigned int)func);
}
EXPORT_SYMBOL(lm3530_unregister_als_callback);
#endif

static int lm3530_read_reg (struct i2c_client *client, 
    unsigned reg, uint8_t *value)
{
    uint8_t buf[1];
    int ret = 0;

    if (!value)
        return -EINVAL;
    buf[0] = reg;
    ret = i2c_master_send (client, buf, 1);
    if (ret > 0) {
        msleep_interruptible (1);
        ret = i2c_master_recv (client, buf, 1);
        if (ret > 0)
            *value = buf[0];
    }
    return ret;
}

static int lm3530_write_reg (struct i2c_client *client, 
    unsigned reg, uint8_t value, const char *caller)
{
    uint8_t buf[2] = {reg, value};
    int ret = 0;

    printk_write ("%s: writing 0x%X to reg 0x%X (%s) at addr 0x%X\n",
        caller, buf[1], buf[0], lm3530_reg_name (reg), client->addr);
    ret = i2c_master_send (client, buf, 2);
    if (ret < 0)
        printk (KERN_ERR "%s: i2c_master_send error %d\n",
            caller, ret);
    return ret;
}

static unsigned dim_values[] = {0x2E, 0x30, 0x50, 0x50, 0x50};
module_param_array(dim_values, uint, NULL, 0644);

/* Convert slider value into LM3530 register value */
static uint8_t lm3530_convert_value (unsigned value, unsigned zone)
{
    if (!value)
        return 0;

    return value/2;
}

static void lm3530_brightness_set (struct led_classdev *led_cdev,
                enum led_brightness value)
{
    int ret, nsteps;
    unsigned int total_time;
    unsigned bright_zone;
    unsigned bvalue;
    unsigned do_ramp = 1;
    struct lm3530_data *driver_data =
        container_of(led_cdev, struct lm3530_data, led_dev);
    struct i2c_client *client = driver_data->client;

    printk_br ("%s: %s, 0x%x (%d)\n", __FUNCTION__, 
        led_cdev->name, value, value);
    if (!driver_data->initialized) {
        printk (KERN_ERR "%s: not initialized\n", __FUNCTION__);
        return;
    }
    if (strstr (led_cdev->name, "nr"))
        do_ramp = 0;

    mutex_lock (&lm3530_mutex);

    /* Calculate brightness value for each zone relative to its cap */
    bvalue = lm3530_convert_value (value, bright_zone);

    /* Calculate number of steps for ramping */
    nsteps = bvalue - driver_data->bvalue;
    if (nsteps < 0)
        nsteps = nsteps * (-1);

#if 0
    lm3530_set_ramp (client, do_ramp, nsteps, &total_time);

    printk_br ("%s: 0x%x => 0x%x, %d steps, ramp time %dus\n",
        __FUNCTION__,
        driver_data->bvalue, bvalue, nsteps, total_time);
#endif

    ret = lm3530_write_reg (client,
        LM3530_BRIGHTNESS_CTRL_REG, bvalue, __FUNCTION__);
    driver_data->bvalue = bvalue;

    mutex_unlock (&lm3530_mutex);
}

#ifdef CONFIG_LM3530_ALS
static int lm3530_als_open (struct inode *inode, struct file *file)
{
    return 0;
}

static int lm3530_als_release (struct inode *inode, struct file *file)
{
    return 0;
}

#define CMD_LEN 5
static ssize_t lm3530_als_write (struct file *fp, const char __user *buf, 
    size_t count, loff_t *pos)
{
#if 0
    unsigned char cmd[CMD_LEN];
    int len;
    uint8_t value;
    unsigned old_zone;

    if (count < 1)
        return 0;

    len = count > CMD_LEN-1 ? CMD_LEN-1 : count;

    if (copy_from_user (cmd, buf, len))
        return -EFAULT;

    if (lm3530_data.revision <= 1)
        return -EFAULT;

    cmd[len] = '\0';
    if (cmd[len-1] == '\n') {
        cmd[len-1] = '\0';
        len--;
    }
    if (!strcmp (cmd, "1")) {
        printk (KERN_INFO "%s: enabling ALS\n", __FUNCTION__);
        value = CONFIG_VALUE | 0x80;
        mutex_lock (&lm3530_mutex);
        atomic_set (&lm3530_data.use_als, 1);
        /* No need to change ALS zone; interrupt handler will do it */
        lm3530_write_reg (LM3530_CONFIG_REG, value, __FUNCTION__);
        mutex_unlock (&lm3530_mutex);
    } else if (!strcmp (cmd, "0")) {
        printk (KERN_INFO "%s: disabling ALS\n", __FUNCTION__);
        value = CONFIG_VALUE_NO_ALS;
        mutex_lock (&lm3530_mutex);
        old_zone = atomic_read (&lm3530_data.als_zone);
        lm3530_write_reg (LM3530_CONFIG_REG, value, __FUNCTION__);
        atomic_set (&lm3530_data.use_als, 0);
        atomic_set (&lm3530_data.als_zone, ALS_NO_ZONE);
        mutex_unlock (&lm3530_mutex);
        if (atomic_read (&lm3530_data.bright_zone) < 2) {
            atomic_set (&lm3530_data.bright_zone, ALS_NO_ZONE);
            printk_als ("%s: ALS canceled; changing brightness\n",
                __FUNCTION__);
            /* Adjust brightness */
            lm3530_brightness_set (&lm3530_led, -1);
        } else {
            atomic_set (&lm3530_data.bright_zone, ALS_NO_ZONE);
        }
        lm3530_call_als_callbacks (old_zone, 0);
        lm3530_send_als_event (0);
#ifndef CONFIG_MACH_OLYMPUS
    } else if (!strcmp (cmd, "e")) {
        lm3530_disable_esd = 1;
    } else if (!strcmp (cmd, "E")) {
        lm3530_disable_esd = 0;
    } else if (!strcmp (cmd, "p")) {
        pwm_value = 0;
    } else if (!strcmp (cmd, "P")) {
        pwm_value = 1;
#endif
    } else {
        printk (KERN_ERR "%s: invalid command %s\n", __FUNCTION__, cmd);
        return -EFAULT;
    }
    
    return count;
#endif
    return 0;
}

static ssize_t lm3530_als_read (struct file *file, char __user *buf, 
    size_t count, loff_t *ppos)
{
#if 0
    char z[20];

    if (file->private_data)
        return 0;

    if (!atomic_read (&lm3530_data.use_als)) {
        sprintf (z, "%d\n", ALS_NO_ZONE);
    } else {
        sprintf (z, "%d %d\n", 
            atomic_read (&lm3530_data.als_zone), 
            atomic_read (&lm3530_data.bright_zone));
    }
    if (copy_to_user (buf, z, strlen (z)))
        return -EFAULT;

    file->private_data = (void *)1;
    return strlen (z);
#endif
    return 0;
}

static const struct file_operations als_fops = {
    .owner      = THIS_MODULE,
    .read       = lm3530_als_read,
    .write      = lm3530_als_write,
    .open       = lm3530_als_open,
    .release    = lm3530_als_release,
};

static struct miscdevice als_miscdev = {
    .minor      = MISC_DYNAMIC_MINOR,
    .name       = "als",
    .fops       = &als_fops,
};
#endif

static ssize_t lm3530_suspend_show (struct device *dev,
                      struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev->parent, struct i2c_client,
						 dev);
	struct lm3530_data *driver_data = i2c_get_clientdata(client);

    sprintf (buf, "%d\n", atomic_read (&driver_data->in_suspend));
    return strlen(buf)+1;
}

static ssize_t lm3530_suspend_store (struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t size)
{
    unsigned value = 0;
	struct i2c_client *client = container_of(dev->parent, struct i2c_client,
						 dev);
	struct lm3530_data *driver_data = i2c_get_clientdata(client);

    if (!buf || size == 0) {
        printk (KERN_ERR "%s: invalid command\n", __FUNCTION__);
        return -EINVAL;
    }

    sscanf (buf, "%d", &value);
    if (value) {
        printk (KERN_INFO "%s: going into TCMD SUSPEND mode\n",
            __FUNCTION__);
        atomic_set (&driver_data->in_suspend, 1);
        driver_data->saved_bvalue = driver_data->led_dev.brightness;
        driver_data->led_dev.brightness = 255;
    } else {
        printk (KERN_INFO "%s: exiting TCMD SUSPEND mode\n",
            __FUNCTION__);
        atomic_set (&driver_data->in_suspend, 0);
        driver_data->led_dev.brightness = driver_data->saved_bvalue;
    }
    /* Adjust brightness */
    lm3530_brightness_set (&driver_data->led_dev, 255);
    return size;
}
static DEVICE_ATTR(suspend, 0644, lm3530_suspend_show, lm3530_suspend_store);

/* This function is called by i2c_probe */
static int lm3530_probe (struct i2c_client *client,
    const struct i2c_device_id *id)
{
    int ret = 0;
#ifdef CONFIG_LM3530_ALS
    unsigned long request_flags =  IRQF_TRIGGER_LOW;
#endif
    struct lm3530_platform_data *pdata = client->dev.platform_data;
    struct lm3530_data *driver_data;

    printk (KERN_INFO "%s: enter, I2C address = 0x%x, flags = 0x%x\n", 
        __FUNCTION__, client->addr, client->flags);

    if (pdata == NULL) {
        printk (KERN_ERR "%s: platform data required\n", __FUNCTION__);
        return -EINVAL;
    }
    /* We should be able to read and write byte data */
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        printk (KERN_ERR "%s: I2C_FUNC_I2C not supported\n",
            __FUNCTION__);
        return -ENOTSUPP;
    }
    if (pdata->als_config && !client->irq) {
        printk (KERN_ERR "%s: ALS polling not supported, irq is not provided\n",
            __FUNCTION__);
        return -ENOTSUPP;
    }
    driver_data = kzalloc(sizeof(struct lm3530_data), GFP_KERNEL);
    if (driver_data == NULL) {
        printk (KERN_ERR "%s: kzalloc failed\n", __FUNCTION__);
        return -ENOMEM;
    }
    memset (driver_data, 0, sizeof (*driver_data));

    driver_data->client = client;
    driver_data->pdata = pdata;

    i2c_set_clientdata (client, driver_data);

    /* Initialize chip */
    if (pdata->init) {
        pdata->init();
    }
    if ((ret = lm3530_setup (driver_data)) < 0) {
        goto setup_failed;
    }

    /* Register LED class */
    driver_data->led_dev.name = LM3530_LED_NAME;
    driver_data->led_dev.brightness_set = lm3530_brightness_set;
    ret = led_classdev_register (&client->adapter->dev, &driver_data->led_dev);
    if (ret) {
        printk (KERN_ERR "%s: led_classdev_register %s failed: %d\n",
           __FUNCTION__, LM3530_LED_NAME, ret);
        goto led_register_failed;
    } 

    /* Register LED class for no ramping */
    driver_data->led_dev_nr.name = LM3530_LED_NAME_NORAMP;
    driver_data->led_dev_nr.brightness_set = lm3530_brightness_set;
    ret = led_classdev_register (&client->adapter->dev, 
        &driver_data->led_dev_nr);
    if (ret) {
        printk (KERN_ERR "%s: led_classdev_register %s failed: %d\n",
           __FUNCTION__, LM3530_LED_NAME_NORAMP, ret);
        goto led_nr_register_failed;
    } 

    atomic_set (&driver_data->in_suspend, 0);
    ret = device_create_file (driver_data->led_dev.dev, &dev_attr_suspend);
    if (ret) {
      printk (KERN_ERR "%s: unable to create suspend device file for %s: %d\n",
            __FUNCTION__, LM3530_LED_NAME, ret);
        goto device_create_file_failed;
    }
    dev_set_drvdata (driver_data->led_dev.dev, &driver_data->led_dev);

#ifdef CONFIG_LEDS_LM3530_ALS
    /* Initialize interrupts */
    INIT_WORK(&driver_data->work, lm3530_work_func);
    ret = request_irq (client->irq, lm3530_irq_handler, request_flags, 
        LM3530_NAME, driver_data);

    if (ret == 0) {
        ret = set_irq_wake (client->irq, 1);
    } else {
        printk (KERN_ERR "request_irq %d for lm3530 failed: %d\n", 
            client->irq, ret);
        ret = -EINVAL;
        goto request_irq_failed;
    }

    if ((ret = misc_register (&als_miscdev))) {
        printk (KERN_ERR "%s: misc_register failed, error %d\n",
            __FUNCTION__, ret);
        goto misc_register_failed;
    }

    driver_data->idev = input_allocate_device();
    if (driver_data->idev == NULL) {
      printk (KERN_ERR "%s: unable to allocate input device file for als\n",
            __FUNCTION__);
        goto input_allocate_device_failed;
    }
    driver_data->idev->name = "als";
    input_set_capability(driver_data->idev, EV_MSC, MSC_RAW);
    input_set_capability(driver_data->idev, EV_LED, LED_MISC);
    ret = input_register_device (driver_data->idev);
    if (ret) {
      printk (KERN_ERR "%s: unable to register input device file for als: %d\n",
            __FUNCTION__, ret);
        goto input_register_device_failed;
    }
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
    driver_data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 5;
    driver_data->early_suspend.suspend = lm3530_early_suspend,
    driver_data->early_suspend.resume = lm3530_late_resume,
    register_early_suspend (&driver_data->early_suspend);
#endif

    driver_data->led_dev.brightness = 255;
    driver_data->led_dev_nr.brightness = 255;
    lm3530_write_reg (client, LM3530_BRIGHTNESS_CTRL_REG, 0xFF, __FUNCTION__);
    driver_data->initialized = 1;

    return 0;

#ifdef CONFIG_LM3530_ALS
input_register_device_failed:
    input_free_device (driver_data->idev);
input_allocate_device_failed: 
    misc_deregister (&als_miscdev);
misc_register_failed:
    free_irq (client->irq, driver_data);
request_irq_failed:
#endif
    device_remove_file (driver_data->led_dev.dev, &dev_attr_suspend);
device_create_file_failed:
    led_classdev_unregister(&driver_data->led_dev_nr);
led_nr_register_failed:
    led_classdev_unregister(&driver_data->led_dev);
led_register_failed:
setup_failed:
    kfree (driver_data);
    return ret;
}

#ifdef CONFIG_LM3530_ALS
static irqreturn_t lm3530_irq_handler (int irq, void *dev_id)
{
    struct lm3530_data *driver_data = dev_id;

    pr_debug ("%s: got an interrupt %d\n", __FUNCTION__, irq);

    disable_irq (irq);
    schedule_work (&driver_data->work);

    return IRQ_HANDLED;
}

static void lm3530_send_als_event (struct lm3530_data *driver_data, int zone)
{
    //input_event (lm3530_data.idev, EV_MSC, MSC_RAW, light_value);
    input_event (driver_data->idev, EV_LED, LED_MISC, zone);
    input_sync (driver_data->idev);
}

static void lm3530_work_func (struct work_struct *work)
{
    struct lm3530_data *driver_data =
	    container_of(work, struct lm3530_data, work);
    enable_irq (driver_data->client->irq);
}
#endif

/* This function calculates ramp step time so that total ramp time is
 * equal to ramp_time defined currently at 200ms
 */
static int lm3530_set_ramp (struct i2c_client *client, 
    unsigned int on, unsigned int nsteps, unsigned int *rtime)
{
    int ret, i = 0;
    uint8_t value = 0;
    unsigned int total_time = 0;
    /* Ramp times in microseconds */
    unsigned int lm3530_ramp[] = {8, 4096, 8192, 16384};
    int nramp = sizeof (lm3530_ramp) / sizeof (lm3530_ramp[0]);

    if (on) {
        /* Calculate the closest possible ramp time */
        for (i = 0; i < nramp; i++) {
            total_time = nsteps * lm3530_ramp[i];
            if (total_time >= ramp_time)
                break;
        }
        if (i > 0 && total_time > ramp_time) {
            i--;
            total_time = nsteps * lm3530_ramp[i];
        }
    } 
    value = i | (i << 2);
     
#if 0
    printk (KERN_ERR "%s: ramp = %s, ramp step = %d us (total = %d us)\n",
        __FUNCTION__, on ? "on" : "off", lm3530_ramp[i], total_time);
#endif
    if (rtime)
        *rtime = total_time;
    ret = lm3530_write_reg (client, LM3530_RAMP_REG, value, __FUNCTION__);
    return ret;
}

static int lm3530_configure (struct lm3530_data *driver_data)
{
    int ret = 0;

    lm3530_write_reg (driver_data->client,
        LM3530_GEN_CONFIG_REG, driver_data->pdata->gen_config, __FUNCTION__);

    lm3530_write_reg (driver_data->client,
        LM3530_ALS_CONFIG_REG, driver_data->pdata->als_config, __FUNCTION__);

    return ret;
}

static int lm3530_setup (struct lm3530_data *driver_data)
{
    int ret;
    uint8_t value;

    /* Read revision number */
    ret = lm3530_read_reg (driver_data->client, LM3530_VERSION_REG, &value);
    if (ret < 0) {
        printk (KERN_ERR "%s: unable to read from chip: %d\n",
            __FUNCTION__, ret);
        return ret;
    }
    switch (value & 0x03) {
        case 0x01: driver_data->revision = 2; break;
        case 0x00: driver_data->revision = 1; break;
        default: driver_data->revision = 2; break; 
    }
    /* revision is going to be an index to lm3530_ramp array */
    printk (KERN_INFO "%s: revision %d (0x%X)\n", 
        __FUNCTION__, driver_data->revision, value);

    ret = lm3530_configure (driver_data);
    if (ret < 0)
        return ret;

    //hrtimer_init (&lm3530_data.timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    //lm3530_data.timer.function = lm3530_timer_func;
    //hrtimer_start(&lm3530_data.timer, ktime_set(2, 0), HRTIMER_MODE_REL);

    return ret;
}

static int lm3530_remove (struct i2c_client *client)
{
    struct lm3530_data *driver_data = i2c_get_clientdata(client);

#ifdef CONFIG_LM3530_ALS
    input_unregister_device (driver_data->idev);
    input_free_device (driver_data->idev);
    misc_deregister (&als_miscdev);
    free_irq (client->irq, driver_data);
#endif
    device_remove_file (driver_data->led_dev.dev, &dev_attr_suspend);
    led_classdev_unregister(&driver_data->led_dev_nr);
    led_classdev_unregister(&driver_data->led_dev);
    kfree (driver_data);
    return 0;
}

static int lm3530_suspend (struct i2c_client *client, pm_message_t mesg)
{
    struct lm3530_data *driver_data = i2c_get_clientdata(client);
    printk_suspend ("%s: called with pm message %d\n", 
        __FUNCTION__, mesg.event);

    led_classdev_suspend (&driver_data->led_dev);
    if (driver_data->pdata->power_off) {
        driver_data->pdata->power_off();
    }
    return 0;
}

static int lm3530_resume (struct i2c_client *client)
{
    struct lm3530_data *driver_data = i2c_get_clientdata(client);
    printk_suspend ("%s: resuming\n", __FUNCTION__);
    if (driver_data->pdata->power_on) {
        driver_data->pdata->power_on();
    }
    mutex_lock (&lm3530_mutex);
    lm3530_configure (driver_data);
    mutex_unlock (&lm3530_mutex);
    led_classdev_resume (&driver_data->led_dev);

    printk_suspend ("%s: driver resumed\n", __FUNCTION__);

    return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void lm3530_early_suspend (struct early_suspend *handler)
{
    struct lm3530_data *driver_data;

	driver_data = container_of(handler, struct lm3530_data, early_suspend);
    lm3530_suspend (driver_data->client, PMSG_SUSPEND);
}

static void lm3530_late_resume (struct early_suspend *handler)
{
	struct lm3530_data *driver_data;

	driver_data = container_of(handler, struct lm3530_data, early_suspend);

    lm3530_resume (driver_data->client);
}
#endif


static int __devinit lm3530_init (void)
{
    int ret;

    printk (KERN_INFO "%s: enter\n", __FUNCTION__);
    ret = i2c_add_driver (&lm3530_driver);
    if (ret) {
        printk (KERN_ERR "%s: i2c_add_driver failed, error %d\n", 
            __FUNCTION__, ret);
    }

    return ret;
}

static void __exit lm3530_exit(void)
{
    i2c_del_driver (&lm3530_driver);
}

static const char *lm3530_reg_name (int reg) 
{
    unsigned reg_count;
    int i;

    reg_count = sizeof(lm3530_regs) / sizeof(lm3530_regs[0]);
    for (i = 0; i < reg_count; i++) {
        if (reg == lm3530_regs[i].reg) {
            return lm3530_regs[i].name;
        }
    }
    return "UNKNOWN";
}

module_init(lm3530_init);
module_exit(lm3530_exit);

MODULE_DESCRIPTION("LM3530 DISPLAY BACKLIGHT DRIVER");
MODULE_AUTHOR("Alina Yakovleva, Motorola, qvdh43@motorola.com");
MODULE_LICENSE("GPL v2");
