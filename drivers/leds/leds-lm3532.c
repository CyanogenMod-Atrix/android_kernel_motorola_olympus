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
#include <linux/leds-lm3532.h>
#ifdef  CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#define MODULE_NAME "leds_lm3532"

#define LM3532_INIT_REQUESTS 8

unsigned btn_bl_tied = 0;
module_param(btn_bl_tied, uint, 0664);

unsigned trace_suspend = 1;
module_param(trace_suspend, uint, 0664);
#define printk_suspend(fmt,args...) if (trace_suspend) printk(KERN_INFO fmt, ##args)

unsigned trace_request = 0;
/* Initial number of requests logged */
unsigned trace_request_initial = LM3532_INIT_REQUESTS;
module_param(trace_request, uint, 0664);
#define printk_request(fmt,args...) if (trace_request || trace_request_initial) {printk(KERN_INFO fmt, ##args); if (trace_request_initial) trace_request_initial--;}

#ifdef LM3532_DEBUG
unsigned trace_brightness = 0;
module_param(trace_brightness, uint, 0664);
#define printk_br(fmt,args...) if (trace_brightness) printk(KERN_INFO fmt, ##args)

unsigned trace_write = 0;
module_param(trace_write, uint, 0664);
#define printk_write(fmt,args...) if (trace_write) printk(KERN_INFO fmt, ##args)
#else
#define printk_br(fmt,args...)
#define printk_write(fmt,args...)
#endif


struct lm3532_data {
	uint16_t addr;
    struct i2c_client *client;
    struct input_dev *idev;
    struct lm3532_platform_data *pdata;
    struct led_classdev led_dev;
    struct led_classdev led_dev_tcmd;
    struct led_classdev led_dev_nr;
    struct led_classdev button_led;
    struct led_classdev button_led_tcmd;
    struct led_classdev webtop_led;
    struct early_suspend early_suspend;
    unsigned initialized;
	unsigned in_webtop_mode;
    uint8_t revision;
    uint8_t enable_reg;
	atomic_t suspended;
    atomic_t in_suspend;   // Whether the driver is in TCMD SUSPEND mode
    unsigned bvalue;       // Current brightness register value
    unsigned saved_bvalue; // Brightness before TCMD SUSPEND
    struct work_struct work;
};

static DEFINE_MUTEX(lm3532_mutex);

struct lm3532_reg {
    const char *name;
    uint8_t reg;
} lm3532_r0_regs[] = {
    {"OUTPUT_CFG_REG",    LM3532_OUTPUT_CFG_REG},
    {"START_UP_RAMP_REG", LM3532_START_UP_RAMP_REG},
    {"RUN_TIME_RAMP_REG", LM3532_RUN_TIME_RAMP_REG},
    {"CTRL_A_PWM_REG",    LM3532_CTRL_A_PWM_REG},
    {"CTRL_B_PWM_REG",    LM3532_CTRL_B_PWM_REG},
    {"CTRL_C_PWM_REG",    LM3532_CTRL_C_PWM_REG},
    {"CTRL_A_BR_CFG_REG", LM3532_CTRL_A_BR_CFG_REG},
    {"CTRL_B_BR_CFG_REG", LM3532_R0_CTRL_B_BR_CFG_REG},
    {"CTRL_C_BR_CFG_REG", LM3532_R0_CTRL_C_BR_CFG_REG},
    {"ENABLE_REG",        LM3532_R0_ENABLE_REG},
    {"ALS1_RES_SEL_REG",  LM3532_ALS1_RES_SEL_REG},
    {"ALS2_RES_SEL_REG",  LM3532_ALS2_RES_SEL_REG},
    {"ALS_CFG_REG",       LM3532_R0_ALS_CFG_REG},
    {"ALS_ZONE_REG",      LM3532_R0_ALS_ZONE_REG},
    {"ALS_BR_ZONE_REG",   LM3532_R0_ALS_BR_ZONE_REG},
    {"ALS_UP_ZONE_REG",   LM3532_R0_ALS_UP_ZONE_REG},
    {"IND_BLINK1_REG",    LM3532_R0_IND_BLINK1_REG},
    {"IND_BLINK2_REG",    LM3532_R0_IND_BLINK2_REG},
    {"ZB1_REG",           LM3532_ZB1_REG},
    {"ZB2_REG",           LM3532_ZB2_REG},
    {"ZB3_REG",           LM3532_ZB3_REG},
    {"ZB4_REG",           LM3532_ZB4_REG},
    {"CTRL_A_ZT1_REG",    LM3532_CTRL_A_ZT1_REG},
    {"CTRL_A_ZT2_REG",    LM3532_CTRL_A_ZT2_REG},
    {"CTRL_A_ZT3_REG",    LM3532_CTRL_A_ZT3_REG},
    {"CTRL_A_ZT4_REG",    LM3532_CTRL_A_ZT4_REG},
    {"CTRL_A_ZT5_REG",    LM3532_CTRL_A_ZT5_REG},
    {"CTRL_B_ZT1_REG",    LM3532_CTRL_B_ZT1_REG},
    {"CTRL_B_ZT2_REG",    LM3532_CTRL_B_ZT2_REG},
    {"CTRL_B_ZT3_REG",    LM3532_CTRL_B_ZT3_REG},
    {"CTRL_B_ZT4_REG",    LM3532_CTRL_B_ZT4_REG},
    {"CTRL_B_ZT5_REG",    LM3532_CTRL_B_ZT5_REG},
    {"CTRL_C_ZT1_REG",    LM3532_CTRL_C_ZT1_REG},
    {"CTRL_C_ZT2_REG",    LM3532_CTRL_C_ZT2_REG},
    {"CTRL_C_ZT3_REG",    LM3532_CTRL_C_ZT3_REG},
    {"CTRL_C_ZT4_REG",    LM3532_CTRL_C_ZT4_REG},
    {"CTRL_C_ZT5_REG",    LM3532_CTRL_C_ZT5_REG},
    {"VERSION_REG",       LM3532_VERSION_REG},
};

struct lm3532_reg lm3532_regs[] = {
    {"OUTPUT_CFG_REG",    LM3532_OUTPUT_CFG_REG},
    {"START_UP_RAMP_REG", LM3532_START_UP_RAMP_REG},
    {"RUN_TIME_RAMP_REG", LM3532_RUN_TIME_RAMP_REG},
    {"CTRL_A_PWM_REG",    LM3532_CTRL_A_PWM_REG},
    {"CTRL_B_PWM_REG",    LM3532_CTRL_B_PWM_REG},
    {"CTRL_C_PWM_REG",    LM3532_CTRL_C_PWM_REG},
    {"CTRL_A_BR_CFG_REG", LM3532_CTRL_A_BR_CFG_REG},
    {"CTRL_A_FS_CURR_REG",LM3532_CTRL_A_FS_CURR_REG},
    {"CTRL_B_BR_CFG_REG", LM3532_CTRL_B_BR_CFG_REG},
    {"CTRL_B_FS_CURR_REG",LM3532_CTRL_B_FS_CURR_REG},
    {"CTRL_C_BR_CFG_REG", LM3532_CTRL_C_BR_CFG_REG},
    {"CTRL_C_FS_CURR_REG",LM3532_CTRL_C_FS_CURR_REG},
    {"ENABLE_REG",        LM3532_ENABLE_REG},
    {"FEEDBACK_ENABLE_REG", LM3532_FEEDBACK_ENABLE_REG},
    {"ALS1_RES_SEL_REG",  LM3532_ALS1_RES_SEL_REG},
    {"ALS2_RES_SEL_REG",  LM3532_ALS2_RES_SEL_REG},
    {"ALS_CFG_REG",       LM3532_ALS_CFG_REG},
    {"ALS_ZONE_REG",      LM3532_ALS_ZONE_REG},
    {"ALS_BR_ZONE_REG",   LM3532_ALS_BR_ZONE_REG},
    {"ALS_UP_ZONE_REG",   LM3532_ALS_UP_ZONE_REG},
    {"ZB1_REG",           LM3532_ZB1_REG},
    {"ZB2_REG",           LM3532_ZB2_REG},
    {"ZB3_REG",           LM3532_ZB3_REG},
    {"ZB4_REG",           LM3532_ZB4_REG},
    {"CTRL_A_ZT1_REG",    LM3532_CTRL_A_ZT1_REG},
    {"CTRL_A_ZT2_REG",    LM3532_CTRL_A_ZT2_REG},
    {"CTRL_A_ZT3_REG",    LM3532_CTRL_A_ZT3_REG},
    {"CTRL_A_ZT4_REG",    LM3532_CTRL_A_ZT4_REG},
    {"CTRL_A_ZT5_REG",    LM3532_CTRL_A_ZT5_REG},
    {"CTRL_B_ZT1_REG",    LM3532_CTRL_B_ZT1_REG},
    {"CTRL_B_ZT2_REG",    LM3532_CTRL_B_ZT2_REG},
    {"CTRL_B_ZT3_REG",    LM3532_CTRL_B_ZT3_REG},
    {"CTRL_B_ZT4_REG",    LM3532_CTRL_B_ZT4_REG},
    {"CTRL_B_ZT5_REG",    LM3532_CTRL_B_ZT5_REG},
    {"CTRL_C_ZT1_REG",    LM3532_CTRL_C_ZT1_REG},
    {"CTRL_C_ZT2_REG",    LM3532_CTRL_C_ZT2_REG},
    {"CTRL_C_ZT3_REG",    LM3532_CTRL_C_ZT3_REG},
    {"CTRL_C_ZT4_REG",    LM3532_CTRL_C_ZT4_REG},
    {"CTRL_C_ZT5_REG",    LM3532_CTRL_C_ZT5_REG},
    {"VERSION_REG",       LM3532_VERSION_REG},
};

#ifdef LM3532_DEBUG
static const char *lm3532_reg_name (struct lm3532_data *driver_data, int reg)
{
    unsigned reg_count;
    int i;

	if (driver_data->revision == 0xF6) {
        reg_count = sizeof(lm3532_r0_regs) / sizeof(lm3532_r0_regs[0]);
        for (i = 0; i < reg_count; i++) {
        	if (reg == lm3532_r0_regs[i].reg) {
                return lm3532_r0_regs[i].name;
            }
        }
    } else {
        reg_count = sizeof(lm3532_regs) / sizeof(lm3532_regs[0]);
        for (i = 0; i < reg_count; i++) {
            if (reg == lm3532_regs[i].reg) {
                return lm3532_regs[i].name;
            }
        }
    }
    return "UNKNOWN";
}
#endif

static int lm3532_read_reg (struct i2c_client *client,
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

static int lm3532_write_reg (struct i2c_client *client,
    unsigned reg, uint8_t value, const char *caller)
{
    uint8_t buf[2] = {reg, value};
    int ret = 0;
#ifdef LM3532_DEBUG
	struct lm3532_data *driver_data = i2c_get_clientdata(client);

    printk_write ("%s: reg 0x%X (%s) = 0x%x\n",
        caller, buf[0], lm3532_reg_name (driver_data, reg), buf[1]);
#endif
    ret = i2c_master_send (client, buf, 2);
    if (ret < 0)
        printk (KERN_ERR "%s: i2c_master_send error %d\n",
            caller, ret);
    return ret;
}

/* This function calculates ramp step time so that total ramp time is
 * equal to ramp_time defined currently at 200ms
 */
static int lm3532_set_ramp (struct lm3532_data *driver_data,
    unsigned int on, unsigned int nsteps, unsigned int *rtime)
{
    int ret, i = 0;
    uint8_t value = 0;
    unsigned int total_time = 0;
    /* Ramp times in microseconds */
    unsigned int lm3532_ramp[] = {8, 1000, 2000, 4000, 8000, 16000, 32000, 64000};
    int nramp = sizeof (lm3532_ramp) / sizeof (lm3532_ramp[0]);
    unsigned ramp_time = driver_data->pdata->ramp_time * 1000;

    if (on) {
        /* Calculate the closest possible ramp time */
        for (i = 0; i < nramp; i++) {
            total_time = nsteps * lm3532_ramp[i];
            if (total_time >= ramp_time)
                break;
        }
        if (i > 0 && total_time > ramp_time) {
            i--;
            total_time = nsteps * lm3532_ramp[i];
        }
        value = i | (i << 3);
    } else {
        value = 0;
    }

    if (rtime)
        *rtime = total_time;
    ret = lm3532_write_reg (driver_data->client, LM3532_RUN_TIME_RAMP_REG,
        value, __func__);
    ret = lm3532_write_reg (driver_data->client, LM3532_START_UP_RAMP_REG,
        value, __func__);
    return ret;
}

static void lm3532_do_button_brtns_set (struct lm3532_data *driver_data,
	enum led_brightness value)
{
    struct i2c_client *client = driver_data->client;

	if (!(driver_data->pdata->flags & LM3532_CONFIG_BUTTON_BL))
		return;
    if (driver_data->revision == 0xF6) {
        lm3532_write_reg (client, LM3532_CTRL_A_ZT2_REG, value, __func__);
    } else {
        lm3532_write_reg (client, LM3532_CTRL_B_ZT1_REG, value, __func__);
    }
}

static void lm3532_button_brightness_set (struct led_classdev *led_cdev,
                enum led_brightness value)
{
    struct lm3532_data *driver_data;
    struct i2c_client *client;
	unsigned is_tcmd = 0;

    printk_request ("%s: %s, 0x%x (%d)\n", __func__,
        led_cdev->name, value, value);
	if (strstr(led_cdev->name, "tcmd"))
		is_tcmd = 1;
	if (is_tcmd)
		driver_data =
			container_of(led_cdev, struct lm3532_data, button_led_tcmd);
	else
		driver_data = container_of(led_cdev, struct lm3532_data, button_led);
	client = driver_data->client;

    if (!driver_data->initialized) {
        printk (KERN_ERR "%s: not initialized\n", __func__);
        return;
    }

	if (!is_tcmd && btn_bl_tied) {
		printk_request("%s: request denied, flags=0x%x\n",
			__func__, driver_data->pdata->flags);
		return;
	}

    mutex_lock(&lm3532_mutex);
	if (value && !is_tcmd && driver_data->in_webtop_mode) {
		printk_request("%s: request denied - in webtop mode\n", __func__);
		mutex_unlock(&lm3532_mutex);
		return;
	}

	lm3532_do_button_brtns_set(driver_data, value);
    mutex_unlock(&lm3532_mutex);
}

static void lm3532_webtop_brightness_set (struct led_classdev *led_cdev,
                enum led_brightness value)
{
    struct lm3532_data *driver_data;
    struct i2c_client *client;

	driver_data = container_of(led_cdev, struct lm3532_data, webtop_led);
	client = driver_data->client;

    printk_request ("%s: %s, 0x%x(%d), webtop=%d\n", __func__,
        led_cdev->name, value, value, driver_data->in_webtop_mode);

    if (!driver_data->initialized) {
        printk (KERN_ERR "%s: not initialized\n", __func__);
        return;
    }
    mutex_lock (&lm3532_mutex);
	if (value == 0) {
		/* We go into normal mode */
		driver_data->in_webtop_mode = 0;
		/* Restore screen backlight */
		lm3532_write_reg (client,
			LM3532_CTRL_A_ZT1_REG, driver_data->bvalue, __func__);
		/* Restore button backlight */
		if (btn_bl_tied)
			lm3532_do_button_brtns_set(driver_data,
				driver_data->bvalue ? 255 : 0);
		else
			lm3532_do_button_brtns_set(driver_data,
				driver_data->button_led.brightness);
	} else {
		/* We go into webtop mode */
		if (value == 1)
			value = 0; /* Turn the backlight off in webtop mode */
		driver_data->in_webtop_mode = 1;
		lm3532_write_reg (client, LM3532_CTRL_A_ZT1_REG, value, __func__);
		/* Turn off button backlight */
		lm3532_do_button_brtns_set(driver_data, 0);
	}

    mutex_unlock (&lm3532_mutex);
}

static void lm3532_brightness_set (struct led_classdev *led_cdev,
                enum led_brightness value)
{
    int nsteps = 0;
    unsigned int total_time = 0;
    unsigned do_ramp = 1;
	unsigned is_tcmd = 0;
    struct lm3532_data *driver_data;
    struct i2c_client *client;

	if (strstr(led_cdev->name, "tcmd"))
		is_tcmd = 1;

	if (is_tcmd)
		driver_data =
			container_of(led_cdev, struct lm3532_data, led_dev_tcmd);
	else
		driver_data = container_of(led_cdev, struct lm3532_data, led_dev);
	client = driver_data->client;

    printk_request ("%s: %s, 0x%x(%d), webtop=%d\n", __func__,
        led_cdev->name, value, value, driver_data->in_webtop_mode);

    if (!driver_data->initialized) {
        printk (KERN_ERR "%s: not initialized\n", __func__);
        return;
    }
    if (strstr (led_cdev->name, "nr"))
        do_ramp = 0;

    if (driver_data->pdata->ramp_time == 0)
        do_ramp = 0;

    mutex_lock (&lm3532_mutex);

    /* Calculate number of steps for ramping */
    nsteps = value - driver_data->bvalue;
    if (nsteps < 0)
        nsteps = -nsteps;

	if (!driver_data->in_webtop_mode) {
		if (do_ramp) {
			lm3532_set_ramp (driver_data, do_ramp, nsteps, &total_time);
			printk_br ("%s: 0x%x => 0x%x, %d steps, RT=%dus\n", __func__,
				driver_data->bvalue, value, nsteps, total_time);
		}

		lm3532_write_reg (client, LM3532_CTRL_A_ZT1_REG, value, __func__);
		if (!is_tcmd && btn_bl_tied) {
			lm3532_do_button_brtns_set(driver_data, value ? 255 : 0);
		}
	}
    driver_data->bvalue = value;

    mutex_unlock (&lm3532_mutex);
}

static ssize_t lm3532_suspend_show (struct device *dev,
                      struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev->parent, struct i2c_client,
						 dev);
	struct lm3532_data *driver_data = i2c_get_clientdata(client);

    sprintf (buf, "%d\n", atomic_read (&driver_data->in_suspend));
    return strlen(buf)+1;
}

static ssize_t lm3532_suspend_store (struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t size)
{
    unsigned value = 0;
	struct i2c_client *client = container_of(dev->parent, struct i2c_client,
						 dev);
	struct lm3532_data *driver_data = i2c_get_clientdata(client);

    if (!buf || size == 0) {
        printk (KERN_ERR "%s: invalid command\n", __func__);
        return -EINVAL;
    }

    sscanf (buf, "%d", &value);
    if (value) {
        printk (KERN_INFO "%s: going into TCMD SUSPEND mode\n",
            __func__);
        atomic_set (&driver_data->in_suspend, 1);
        driver_data->saved_bvalue = driver_data->led_dev.brightness;
        driver_data->led_dev.brightness = 255;
    } else {
        printk (KERN_INFO "%s: exiting TCMD SUSPEND mode\n",
            __func__);
        atomic_set (&driver_data->in_suspend, 0);
        driver_data->led_dev.brightness = driver_data->saved_bvalue;
    }
    /* Adjust brightness */
    lm3532_brightness_set (&driver_data->led_dev, 255);
    return size;
}
static DEVICE_ATTR(suspend, 0664, lm3532_suspend_show, lm3532_suspend_store);

static ssize_t lm3532_registers_show (struct device *dev,
                      struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev->parent,
		struct i2c_client, dev);
	struct lm3532_data *driver_data = i2c_get_clientdata(client);
	int reg_count = sizeof(lm3532_regs) / sizeof(lm3532_regs[0]);
    int i, n = 0;
    uint8_t value;

	if (atomic_read(&driver_data->suspended)) {
		printk(KERN_INFO "%s: can't read: driver suspended\n",
			__func__);
		n += scnprintf(buf + n, PAGE_SIZE - n,
			"Unable to read LM3532 registers: driver suspended\n");
	} else {
		printk(KERN_INFO "%s: reading registers\n", __func__);
		for (i = 0, n = 0; i < reg_count; i++) {
			lm3532_read_reg(client, lm3532_regs[i].reg, &value);
			n += scnprintf(buf + n, PAGE_SIZE - n,
				"%-20s (0x%x) = 0x%02X\n",
				lm3532_regs[i].name, lm3532_regs[i].reg, value);
		}
	}

	return n;
}

static ssize_t lm3532_registers_store (struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t size)
{
	struct i2c_client *client = container_of(dev->parent, struct i2c_client,
						 dev);
	struct lm3532_data *driver_data = i2c_get_clientdata(client);
	unsigned reg;
	unsigned value;

	if (atomic_read(&driver_data->suspended)) {
		printk(KERN_INFO "%s: can't write: driver suspended\n",
			__func__);
		return -ENODEV;
	}
	sscanf(buf, "%x %x", &reg, &value);
	if (value > 0xFF)
		return -EINVAL;
	printk(KERN_INFO "%s: writing reg 0x%x = 0x%x\n", __func__, reg, value);
    mutex_lock(&lm3532_mutex);
	lm3532_write_reg(client, reg, (uint8_t)value, __func__);
	if (reg == LM3532_CTRL_A_FS_CURR_REG)
		driver_data->pdata->ctrl_a_fs_current = (uint8_t)value;
    mutex_unlock(&lm3532_mutex);

    return size;
}
static DEVICE_ATTR(registers, 0664, lm3532_registers_show, lm3532_registers_store);

static ssize_t lm3532_pwm_show (struct device *dev,
                      struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev->parent,
		struct i2c_client, dev);
	struct lm3532_data *driver_data = i2c_get_clientdata(client);
    uint8_t value;
	uint8_t pwm_mask = 0x4;
	int ret;

	if (atomic_read(&driver_data->suspended)) {
		printk(KERN_INFO "%s: unable to read pwm: driver suspended\n",
			__func__);
		return -ENODEV;
	}
    mutex_lock(&lm3532_mutex);
    ret = lm3532_read_reg(client, LM3532_CTRL_A_PWM_REG, &value);
	mutex_unlock(&lm3532_mutex);
    if (ret < 0) {
        printk(KERN_ERR "%s: unable to read PWM register: %d\n",
            __func__, ret);
        return ret;
    }
    sprintf (buf, "%d\n", (value & pwm_mask)>>2);
    return strlen(buf)+1;
}

static ssize_t lm3532_pwm_store (struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t size)
{
	struct i2c_client *client = container_of(dev->parent, struct i2c_client,
						 dev);
	struct lm3532_data *driver_data = i2c_get_clientdata(client);
	unsigned pwm_value;
	uint8_t value;
	uint8_t pwm_mask = 0x4;
	int ret;

	if (atomic_read(&driver_data->suspended)) {
		printk(KERN_INFO "%s: unable to write pwm: driver suspended\n",
			__func__);
		return -ENODEV;
	}
	sscanf(buf, "%d", &pwm_value);
	if (pwm_value != 0 && pwm_value != 1) {
		printk(KERN_ERR "%s: invalid value %d, should be 0 or 1\n",
			__func__, pwm_value);
		return -EINVAL;
	}
	pwm_value = pwm_value << 2; /* Bit 2 is PWM enable bit */
    mutex_lock(&lm3532_mutex);
    ret = lm3532_read_reg(client, LM3532_CTRL_A_PWM_REG, &value);
    if (ret < 0) {
        printk(KERN_ERR "%s: unable to read PWM register: %d\n",
            __func__, ret);
		mutex_unlock(&lm3532_mutex);
        return ret;
    }
	/* If PWM value is the same as requested do nothing */
	if ((value & pwm_mask) == pwm_value) {
		printk(KERN_INFO "%s: %d, PWM value is as requested; nothing to do\n",
			__func__, pwm_value>>2);
	} else {
		/* Now we just need to change the PWM enable bit to the opposite of
		 * what it was */
		printk(KERN_INFO "%s: %s PWM; CTRL_A_PWM = 0x%x\n",
			__func__, pwm_value ? "enabling" : "disabling", value^pwm_mask);
		lm3532_write_reg(client, LM3532_CTRL_A_PWM_REG, value^pwm_mask,
			__func__);
	}
    mutex_unlock(&lm3532_mutex);

    return size;
}
static DEVICE_ATTR(pwm, 0664, lm3532_pwm_show, lm3532_pwm_store);

static irqreturn_t lm3532_irq_handler (int irq, void *dev_id)
{
    struct lm3532_data *driver_data = dev_id;

    pr_debug ("%s: got an interrupt %d\n", __func__, irq);

    disable_irq (irq);
    schedule_work (&driver_data->work);

    return IRQ_HANDLED;
}

static void lm3532_work_func (struct work_struct *work)
{
    struct lm3532_data *driver_data =
	    container_of(work, struct lm3532_data, work);

    enable_irq (driver_data->client->irq);
}

static int lm3532_suspend (struct i2c_client *client, pm_message_t mesg)
{
    struct lm3532_data *driver_data = i2c_get_clientdata(client);
    printk_suspend ("%s: called with pm message %d\n",
        __func__, mesg.event);

	atomic_set(&driver_data->suspended, 1);
    led_classdev_suspend (&driver_data->led_dev);
    if (driver_data->pdata->flags & LM3532_HAS_WEBTOP)
		led_classdev_suspend (&driver_data->webtop_led);
    if (driver_data->pdata->flags & LM3532_CONFIG_BUTTON_BL)
        led_classdev_suspend (&driver_data->button_led);
    if (driver_data->pdata->power_off) {
        driver_data->pdata->power_off();
    }
    return 0;
}

static int lm3532_configure (struct lm3532_data *driver_data)
{
    int ret = 0;
    static uint8_t r0_fs_current[] = {
        LM3532_R0_5mA_FS_CURRENT,       /* 0x00000 */
        LM3532_R0_5mA_FS_CURRENT,       /* 0x00001 */
        LM3532_R0_8mA_FS_CURRENT,       /* 0x00010 */
        LM3532_R0_8mA_FS_CURRENT,       /* 0x00011 */
        LM3532_R0_8mA_FS_CURRENT,       /* 0x00100 */
        LM3532_R0_8mA_FS_CURRENT,       /* 0x00101 */
        LM3532_R0_8mA_FS_CURRENT,       /* 0x00110 */
        LM3532_R0_12mA_FS_CURRENT,      /* 0x00111 */
        LM3532_R0_12mA_FS_CURRENT,      /* 0x01000 */
        LM3532_R0_12mA_FS_CURRENT,      /* 0x01001 */
        LM3532_R0_12mA_FS_CURRENT,      /* 0x01010 */
        LM3532_R0_15mA_FS_CURRENT,      /* 0x01011 */
        LM3532_R0_15mA_FS_CURRENT,      /* 0x01100 */
        LM3532_R0_15mA_FS_CURRENT,      /* 0x01101 */
        LM3532_R0_15mA_FS_CURRENT,      /* 0x01110 */
        LM3532_R0_19mA_FS_CURRENT,      /* 0x01111 */
        LM3532_R0_19mA_FS_CURRENT,      /* 0x10000 */
        LM3532_R0_19mA_FS_CURRENT,      /* 0x10001 */
        LM3532_R0_19mA_FS_CURRENT,      /* 0x10010 */
        LM3532_R0_19mA_FS_CURRENT,      /* 0x10011 */
        LM3532_R0_22mA_FS_CURRENT,      /* 0x10100 */
        LM3532_R0_22mA_FS_CURRENT,      /* 0x10101 */
        LM3532_R0_22mA_FS_CURRENT,      /* 0x10110 */
        LM3532_R0_22mA_FS_CURRENT,      /* 0x10111 */
        LM3532_R0_22mA_FS_CURRENT,      /* 0x11000 */
        LM3532_R0_26mA_FS_CURRENT,      /* 0x11001 */
        LM3532_R0_26mA_FS_CURRENT,      /* 0x11010 */
        LM3532_R0_26mA_FS_CURRENT,      /* 0x11011 */
        LM3532_R0_26mA_FS_CURRENT,      /* 0x11100 */
        LM3532_R0_29mA_FS_CURRENT,      /* 0x11101 */
        LM3532_R0_29mA_FS_CURRENT,      /* 0x11110 */
        LM3532_R0_29mA_FS_CURRENT,      /* 0x11111 */
    };
    if (driver_data->revision == 0xF6) {
        /* Revision 0 */
        uint8_t ctrl_a_fs_current =
            r0_fs_current[driver_data->pdata->ctrl_a_fs_current];
		/* Map ILED1 to CTRL A, ILED2 to CTRL B, ILED3 to CTRL C */
		lm3532_write_reg (driver_data->client,
			LM3532_OUTPUT_CFG_REG, 0x34, __func__);
        lm3532_write_reg (driver_data->client,
            LM3532_CTRL_A_BR_CFG_REG,
                LM3532_I2C_CONTROL | ctrl_a_fs_current |
                driver_data->pdata->ctrl_a_mapping_mode, __func__);

        if (driver_data->pdata->flags & LM3532_CONFIG_BUTTON_BL) {
            uint8_t ctrl_b_fs_current =
                r0_fs_current[driver_data->pdata->ctrl_b_fs_current];
            lm3532_write_reg (driver_data->client,
                LM3532_R0_CTRL_B_BR_CFG_REG,
                    LM3532_I2C_CONTROL | ctrl_b_fs_current |
                    driver_data->pdata->ctrl_b_mapping_mode | 0x4, __func__);
        }
		ret = lm3532_write_reg (driver_data->client,
			LM3532_CTRL_A_ZT2_REG, 0, __func__);
    } else {
        /* Revision 1 and above */
		lm3532_write_reg (driver_data->client,
			LM3532_FEEDBACK_ENABLE_REG, 0x01, __func__);
		lm3532_write_reg (driver_data->client,
			LM3532_CTRL_A_PWM_REG, driver_data->pdata->ctrl_a_pwm, __func__);
		/* Map ILED1 to CTRL A, ILED2 to CTRL B, ILED3 to CTRL B */
		lm3532_write_reg (driver_data->client,
			LM3532_OUTPUT_CFG_REG, 0x14, __func__);
        lm3532_write_reg (driver_data->client,
            LM3532_CTRL_A_BR_CFG_REG,
                LM3532_I2C_CONTROL | driver_data->pdata->ctrl_a_mapping_mode,
                __func__);

        lm3532_write_reg (driver_data->client,
            LM3532_CTRL_A_FS_CURR_REG , driver_data->pdata->ctrl_a_fs_current,
                __func__);

        if (driver_data->pdata->flags & LM3532_CONFIG_BUTTON_BL) {
            lm3532_write_reg (driver_data->client,
                LM3532_CTRL_B_BR_CFG_REG,
                    LM3532_I2C_CONTROL |
                    driver_data->pdata->ctrl_b_mapping_mode, __func__);
            lm3532_write_reg (driver_data->client,
                LM3532_CTRL_B_FS_CURR_REG,
                driver_data->pdata->ctrl_b_fs_current,
                __func__);
        }
		ret = lm3532_write_reg (driver_data->client,
			LM3532_CTRL_B_ZT1_REG, 0, __func__);
    }
    if (driver_data->pdata->ramp_time == 0) {
		lm3532_write_reg (driver_data->client,
			LM3532_START_UP_RAMP_REG , 0x0, __func__);
		lm3532_write_reg (driver_data->client,
			LM3532_RUN_TIME_RAMP_REG , 0x0, __func__);
	}

    return ret;
}

static int lm3532_enable (struct lm3532_data *driver_data)
{
    if (driver_data->pdata->flags & LM3532_CONFIG_BUTTON_BL) {
        lm3532_write_reg (driver_data->client,
            driver_data->enable_reg,
            LM3532_CTRLA_ENABLE|LM3532_CTRLB_ENABLE|LM3532_CTRLC_ENABLE,
			__func__);
    } else {
        lm3532_write_reg (driver_data->client,
            driver_data->enable_reg, LM3532_CTRLA_ENABLE, __func__);
    }
    return 0;
}

static int lm3532_resume (struct i2c_client *client)
{
    struct lm3532_data *driver_data = i2c_get_clientdata(client);
    printk_suspend ("%s: resuming\n", __func__);
    if (driver_data->pdata->power_on) {
        driver_data->pdata->power_on();
    }
    mutex_lock (&lm3532_mutex);
    lm3532_configure (driver_data);
    lm3532_enable (driver_data);
    mutex_unlock (&lm3532_mutex);
	atomic_set(&driver_data->suspended, 0);
	trace_request_initial = LM3532_INIT_REQUESTS;
    led_classdev_resume (&driver_data->led_dev);
    if (driver_data->pdata->flags & LM3532_HAS_WEBTOP)
		led_classdev_resume (&driver_data->webtop_led);
    if (driver_data->pdata->flags & LM3532_CONFIG_BUTTON_BL)
        led_classdev_resume (&driver_data->button_led);

    printk_suspend ("%s: driver resumed\n", __func__);

    return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void lm3532_early_suspend (struct early_suspend *handler)
{
    struct lm3532_data *driver_data;

	driver_data = container_of(handler, struct lm3532_data, early_suspend);
    lm3532_suspend (driver_data->client, PMSG_SUSPEND);
}

static void lm3532_late_resume (struct early_suspend *handler)
{
	struct lm3532_data *driver_data;

	driver_data = container_of(handler, struct lm3532_data, early_suspend);

    lm3532_resume (driver_data->client);
}
#endif


static int lm3532_setup (struct lm3532_data *driver_data)
{
    int ret;
    uint8_t value;

    /* Read revision number, need to write first to get correct value */
    lm3532_write_reg (driver_data->client, LM3532_VERSION_REG, 0, __func__);
    ret = lm3532_read_reg (driver_data->client, LM3532_VERSION_REG, &value);
    if (ret < 0) {
        printk (KERN_ERR "%s: unable to read from chip: %d\n",
            __func__, ret);
        return ret;
    }
    driver_data->revision = value;
    printk (KERN_INFO "%s: revision 0x%X\n", __func__, driver_data->revision);
    if (value == 0xF6) {
        driver_data->enable_reg = LM3532_R0_ENABLE_REG;
    } else {
        driver_data->enable_reg = LM3532_ENABLE_REG;
    }

    ret = lm3532_configure (driver_data);
    driver_data->bvalue = 255;

    return ret;
}

/* This function is called by i2c_probe */
static int lm3532_probe (struct i2c_client *client,
    const struct i2c_device_id *id)
{
    int ret = 0;
    struct lm3532_platform_data *pdata = client->dev.platform_data;
    struct lm3532_data *driver_data;

    printk (KERN_INFO "%s: enter, I2C address = 0x%x, flags = 0x%x\n",
        __func__, client->addr, client->flags);

    if (pdata == NULL) {
        printk (KERN_ERR "%s: platform data required\n", __func__);
        return -EINVAL;
    }
    /* We should be able to read and write byte data */
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        printk (KERN_ERR "%s: I2C_FUNC_I2C not supported\n",
            __func__);
        return -ENOTSUPP;
    }
    driver_data = kzalloc(sizeof(struct lm3532_data), GFP_KERNEL);
    if (driver_data == NULL) {
        printk (KERN_ERR "%s: kzalloc failed\n", __func__);
        return -ENOMEM;
    }
    memset (driver_data, 0, sizeof (*driver_data));

    driver_data->client = client;
    driver_data->pdata = pdata;
    if (driver_data->pdata->ctrl_a_fs_current > 0xFF)
        driver_data->pdata->ctrl_a_fs_current = 0xFF;
    if (driver_data->pdata->ctrl_b_fs_current > 0xFF)
        driver_data->pdata->ctrl_b_fs_current = 0xFF;

    i2c_set_clientdata (client, driver_data);

    /* Initialize chip */
    if (pdata->init) {
        pdata->init();
    }
    if (pdata->power_on) {
        pdata->power_on();
    }
    if ((ret = lm3532_setup (driver_data)) < 0) {
        goto setup_failed;
    }

    /* Register LED class */
    driver_data->led_dev.name = LM3532_LED_NAME;
    driver_data->led_dev.brightness_set = lm3532_brightness_set;
    ret = led_classdev_register (&client->dev, &driver_data->led_dev);
    if (ret) {
        printk (KERN_ERR "%s: led_classdev_register %s failed: %d\n",
           __func__, LM3532_LED_NAME, ret);
        goto led_register_failed;
    }

    /* Register LED class for TCMD */
    driver_data->led_dev_tcmd.name = LM3532_LED_NAME_TCMD;
    driver_data->led_dev_tcmd.brightness_set = lm3532_brightness_set;
    ret = led_classdev_register (&client->dev, &driver_data->led_dev_tcmd);
    if (ret) {
        printk (KERN_ERR "%s: led_classdev_register %s failed: %d\n",
           __func__, LM3532_LED_NAME_TCMD, ret);
        goto led_tcmd_register_failed;
    }

    /* Register LED class for no ramping */
    if (driver_data->pdata->ramp_time != 0) {
        driver_data->led_dev_nr.name = LM3532_LED_NAME_NORAMP;
        driver_data->led_dev_nr.brightness_set = lm3532_brightness_set;
        ret = led_classdev_register (&client->dev, &driver_data->led_dev_nr);
        if (ret) {
            printk (KERN_ERR "%s: led_classdev_register %s failed: %d\n",
               __func__, LM3532_LED_NAME_NORAMP, ret);
            goto led_nr_register_failed;
        }
    }

    if (driver_data->pdata->flags & LM3532_CONFIG_BUTTON_BL) {
        /* Register LED class for button backlight */
        driver_data->button_led.name = LM3532_BUTTON_LED_NAME;
        driver_data->button_led.brightness_set = lm3532_button_brightness_set;
        ret = led_classdev_register (&client->dev, &driver_data->button_led);
        if (ret) {
            printk (KERN_ERR "%s: led_classdev_register %s failed: %d\n",
               __func__, LM3532_BUTTON_LED_NAME, ret);
            goto button_led_register_failed;
        }
        /* Register LED class for button backlight TCMD */
        driver_data->button_led_tcmd.name = LM3532_BUTTON_LED_NAME_TCMD;
        driver_data->button_led_tcmd.brightness_set =
			lm3532_button_brightness_set;
        ret = led_classdev_register (&client->dev,
            &driver_data->button_led_tcmd);
        if (ret) {
            printk (KERN_ERR "%s: led_classdev_register %s failed: %d\n",
               __func__, LM3532_BUTTON_LED_NAME_TCMD, ret);
            goto button_led_tcmd_register_failed;
        }
    }

    if (driver_data->pdata->flags & LM3532_HAS_WEBTOP) {
        /* Register LED class for webtop backlight */
        driver_data->webtop_led.name = LM3532_LED_NAME_WEBTOP;
        driver_data->webtop_led.brightness_set = lm3532_webtop_brightness_set;
        ret = led_classdev_register (&client->dev, &driver_data->webtop_led);
        if (ret) {
            printk (KERN_ERR "%s: led_classdev_register %s failed: %d\n",
               __func__, LM3532_LED_NAME_WEBTOP, ret);
            goto webtop_led_register_failed;
        }
    }

    atomic_set(&driver_data->in_suspend, 0);
    atomic_set(&driver_data->suspended, 0);
    ret = device_create_file(driver_data->led_dev.dev, &dev_attr_suspend);
    if (ret) {
      printk (KERN_ERR "%s: unable to create suspend device file for %s: %d\n",
            __func__, LM3532_LED_NAME, ret);
        goto device_create_file_failed;
    }

    ret = device_create_file(driver_data->led_dev.dev, &dev_attr_registers);
    if (ret) {
      printk (KERN_ERR "%s: unable to create registers device file for %s: %d\n",
            __func__, LM3532_LED_NAME, ret);
        goto device_create_file_failed1;
    }

    ret = device_create_file(driver_data->led_dev.dev, &dev_attr_pwm);
    if (ret) {
      printk (KERN_ERR "%s: unable to create pwm device file for %s: %d\n",
            __func__, LM3532_LED_NAME, ret);
        goto device_create_file_failed2;
    }
    dev_set_drvdata (&client->dev, driver_data);

    /* Initialize interrupts */
    if (driver_data->pdata->flags & LM3532_CONFIG_ALS) {
        unsigned long request_flags =  IRQF_TRIGGER_LOW;

        INIT_WORK(&driver_data->work, lm3532_work_func);
        ret = request_irq (client->irq, lm3532_irq_handler, request_flags,
            LM3532_NAME, driver_data);

        if (ret == 0) {
            ret = set_irq_wake (client->irq, 1);
        } else {
            printk (KERN_ERR "request_irq %d for lm3532 failed: %d\n",
                client->irq, ret);
            ret = -EINVAL;
            goto request_irq_failed;
        }
        driver_data->idev = input_allocate_device();
        if (driver_data->idev == NULL) {
          printk (KERN_ERR "%s: unable to allocate input device file for als\n",
                __func__);
            goto input_allocate_device_failed;
        }
        driver_data->idev->name = "als";
        input_set_capability(driver_data->idev, EV_MSC, MSC_RAW);
        input_set_capability(driver_data->idev, EV_LED, LED_MISC);
        ret = input_register_device (driver_data->idev);
        if (ret) {
          printk (KERN_ERR "%s: unable to register input device file for als: %d\n",
                __func__, ret);
            goto input_register_device_failed;
        }
    }

#ifdef CONFIG_HAS_EARLYSUSPEND
    driver_data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 5;
    driver_data->early_suspend.suspend = lm3532_early_suspend,
    driver_data->early_suspend.resume = lm3532_late_resume,
    register_early_suspend (&driver_data->early_suspend);
#endif

    lm3532_enable(driver_data);

	/* Initialize brightness to whatever bootloader wrote into the register */
	lm3532_read_reg(client, LM3532_CTRL_A_ZT1_REG,
		(uint8_t *)&driver_data->led_dev.brightness);
    driver_data->led_dev_nr.brightness = driver_data->led_dev.brightness;
    driver_data->webtop_led.brightness = 0;

    if (driver_data->pdata->flags & LM3532_CONFIG_BUTTON_BL) {
        lm3532_write_reg (client, LM3532_CTRL_A_ZT2_REG, 0x0, __func__);
		driver_data->button_led.brightness = 0;
	}

	if (driver_data->pdata->flags & LM3532_DISP_BTN_TIED)
		btn_bl_tied = 1;

    driver_data->initialized = 1;

    return 0;

input_register_device_failed:
    input_free_device (driver_data->idev);
input_allocate_device_failed:
    free_irq (client->irq, driver_data);
request_irq_failed:
    device_remove_file (driver_data->led_dev.dev, &dev_attr_pwm);
device_create_file_failed2:
    device_remove_file (driver_data->led_dev.dev, &dev_attr_registers);
device_create_file_failed1:
    device_remove_file (driver_data->led_dev.dev, &dev_attr_suspend);
device_create_file_failed:
    if (driver_data->pdata->flags & LM3532_HAS_WEBTOP)
        led_classdev_unregister(&driver_data->webtop_led);
webtop_led_register_failed:
    if (driver_data->pdata->flags & LM3532_CONFIG_BUTTON_BL)
        led_classdev_unregister(&driver_data->button_led_tcmd);
button_led_tcmd_register_failed:
    if (driver_data->pdata->flags & LM3532_CONFIG_BUTTON_BL)
        led_classdev_unregister(&driver_data->button_led);
button_led_register_failed:
    if (driver_data->pdata->ramp_time != 0)
        led_classdev_unregister(&driver_data->led_dev_nr);
led_nr_register_failed:
    led_classdev_unregister(&driver_data->led_dev_tcmd);
led_tcmd_register_failed:
    led_classdev_unregister(&driver_data->led_dev);
led_register_failed:
setup_failed:
    kfree (driver_data);
    return ret;
}

static int lm3532_remove (struct i2c_client *client)
{
    struct lm3532_data *driver_data = i2c_get_clientdata(client);

    if (driver_data->pdata->flags & LM3532_CONFIG_ALS) {
        input_unregister_device (driver_data->idev);
        input_free_device (driver_data->idev);
        free_irq (client->irq, driver_data);
    }
    device_remove_file (driver_data->led_dev.dev, &dev_attr_suspend);
    device_remove_file (driver_data->led_dev.dev, &dev_attr_registers);
    device_remove_file (driver_data->led_dev.dev, &dev_attr_pwm);
    if (driver_data->pdata->ramp_time != 0)
        led_classdev_unregister(&driver_data->led_dev_nr);
    led_classdev_unregister(&driver_data->led_dev);
    if (driver_data->pdata->flags & LM3532_CONFIG_BUTTON_BL) {
        led_classdev_unregister(&driver_data->button_led);
        led_classdev_unregister(&driver_data->button_led_tcmd);
	}
    if (driver_data->pdata->flags & LM3532_HAS_WEBTOP)
        led_classdev_unregister(&driver_data->webtop_led);
    kfree (driver_data);
    return 0;
}

static const struct i2c_device_id lm3532_id[] = {
    { LM3532_NAME, 0 },
    { }
};

/* This is the I2C driver that will be inserted */
static struct i2c_driver lm3532_driver =
{
    .driver = {
        .name   = LM3532_NAME,
    },
    .id_table = lm3532_id,
    .probe = lm3532_probe,
    .remove  = lm3532_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
    .suspend    = lm3532_suspend,
    .resume     = lm3532_resume,
#endif
};

static int __devinit lm3532_init (void)
{
    int ret;

    printk (KERN_INFO "%s: enter\n", __func__);
    ret = i2c_add_driver (&lm3532_driver);
    if (ret) {
        printk (KERN_ERR "%s: i2c_add_driver failed, error %d\n",
            __func__, ret);
    }

    return ret;
}

static void __exit lm3532_exit(void)
{
    i2c_del_driver (&lm3532_driver);
}

module_init(lm3532_init);
module_exit(lm3532_exit);

MODULE_DESCRIPTION("LM3532 DISPLAY BACKLIGHT DRIVER");
MODULE_AUTHOR("Alina Yakovleva, Motorola, qvdh43@motorola.com");
MODULE_LICENSE("GPL v2");
