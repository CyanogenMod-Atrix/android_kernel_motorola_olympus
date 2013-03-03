/*
 * Copyright (C) 2009-2010 Motorola, Inc.
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

#include <linux/isl29030.h>

#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/leds.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>

struct isl29030_data {
	struct input_dev *idev;
	struct input_dev *adev;
	struct i2c_client *client;
	struct work_struct wq;
	struct workqueue_struct *working_queue;
	struct isl29030_platform_data *pdata;
	struct mutex lock;    /* Mutex to lock read and write */
	struct mutex bit_lock; /* Mutex to lock single bit function */
	struct early_suspend early_suspend;
	unsigned int lux_level;
	atomic_t prox_enabled;
	atomic_t als_enabled;
	atomic_t prox_near;
	atomic_t prox_reported_near;
	atomic_t als_needs_enable_flag;
};

struct isl29030_data *isl29030_misc_data;
#ifndef USE_DYNAMIC_MEMORY_ALLOCATION
struct isl29030_data GL_isl;
#endif

struct isl29030_reg {
	const char *name;
	u8 reg;
} isl29030_regs[] = {
	{ "CHIP_ID",		ISL29030_CHIPID },
	{ "CONFIGURE",		ISL29030_CONFIGURE },
	{ "INTERRUPT",		ISL29030_INTERRUPT },
	{ "PROX_LT",		ISL29030_PROX_LT },
	{ "PROX_HT",		ISL29030_PROX_HT },
	{ "ALS_IR_TH1",		ISL29030_ALSIR_TH1 },
	{ "ALS_IR_TH2",		ISL29030_ALSIR_TH2 },
	{ "ALS_IR_TH3",		ISL29030_ALSIR_TH3 },
	{ "PROX_DATA",		ISL29030_PROX_DATA },
	{ "ALS_IR_DT1",		ISL29030_ALSIR_DT1 },
	{ "ALS_IR_DT2",		ISL29030_ALSIR_DT2 },
	{ "ENABLE",		ISL29030_TEST1 },
	{ "DISABLE",		ISL29030_TEST2 },
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void isl29030_early_suspend(struct early_suspend *handler);
static void isl29030_late_resume(struct early_suspend *handler);
#endif

#define ISL29030_DBG_REPORT_INPUT	0x00000001
#define ISL29030_DBG_POWER_ON_OFF	0x00000002
#define ISL29030_DBG_ENABLE_DISABLE	0x00000004
#define ISL29030_DBG_MISC_IOCTL		0x00000008
#define ISL29030_DBG_SUSPEND_RESUME	0x00000010

static uint32_t irqStatus;
module_param_named(prox_int, irqStatus, uint, 0444);

static u32 isl29030_debug = 0x00000000;
module_param_named(als_debug, isl29030_debug, uint, 0664);

static int isl29030_read_reg(struct isl29030_data *isl, u8 reg,
			     u8 *value)
{
	int error = 0;
	int i = 0;
	u8 dest_buffer;

	if (!value) {
		pr_err("%s: invalid value pointer\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&isl->lock);

	do {
		dest_buffer = reg;
		error = i2c_master_send(isl->client, &dest_buffer, 1);
		if (error == 1) {
			error = i2c_master_recv(isl->client,
				&dest_buffer, LD_ISL29030_ALLOWED_R_BYTES);
		}
		if (error != LD_ISL29030_ALLOWED_R_BYTES) {
			pr_err("%s: read[%i] failed: %d\n", __func__, i, error);
			msleep_interruptible(LD_ISL29030_I2C_RETRY_DELAY);
		}
	} while ((error != LD_ISL29030_ALLOWED_R_BYTES) &&
			((++i) < LD_ISL29030_MAX_RW_RETRIES));

	if (error == LD_ISL29030_ALLOWED_R_BYTES) {
		error = 0;
		*value = dest_buffer;
	}
	mutex_unlock(&isl->lock);

	return error;
}

static int isl29030_write_reg(struct isl29030_data *isl,
			      u8 reg,
			      u8 value)
{
	u8 buf[LD_ISL29030_ALLOWED_W_BYTES] = { reg, value };
	int bytes;
	int i = 0;

	mutex_lock(&isl->lock);

	do {
		bytes = i2c_master_send(isl->client, buf,
					LD_ISL29030_ALLOWED_W_BYTES);

		if (bytes != LD_ISL29030_ALLOWED_W_BYTES) {
			pr_err("%s: write %d failed: %d\n", __func__, i, bytes);
			msleep_interruptible(LD_ISL29030_I2C_RETRY_DELAY);
		}
	} while ((bytes != (LD_ISL29030_ALLOWED_W_BYTES))
		 && ((++i) < LD_ISL29030_MAX_RW_RETRIES));

	mutex_unlock(&isl->lock);
	if (bytes != LD_ISL29030_ALLOWED_W_BYTES) {
		pr_err("%s: i2c_master_send error\n", __func__);
		return -EINVAL;
	}
	return 0;
}

static int isl29030_set_bit(struct isl29030_data *isl,
				   u8 reg,
				   u8 bit_mask,
				   u8 value)
{
	int ret;
	u8 reg_val;
	mutex_lock(&isl->bit_lock);

	ret = isl29030_read_reg(isl, reg, &reg_val);

	if (ret != 0) {
		pr_err("%s:Unable to read register 0x%x: %d\n",
			__func__, reg, ret);
		ret = -EFAULT;
	} else {

		if (value)
			reg_val |= (u8)bit_mask;
		else
			reg_val &= (u8)~bit_mask;

		ret = isl29030_write_reg(isl, reg, reg_val);

		if (ret != 0) {
			pr_err("%s:Unable to write register 0x%x: %d\n",
				__func__, reg, ret);
			ret = -EFAULT;
		}
	}
	mutex_unlock(&isl->bit_lock);

	return ret;
}

static int isl29030_set_als_enable(struct isl29030_data *isl,
				   unsigned int bit_value)
{
	if (isl29030_debug & ISL29030_DBG_ENABLE_DISABLE)
		pr_info("%s: bit = %d prox_near = %d als_needs_enable = %d\n",
			__func__, bit_value, atomic_read(&isl->prox_near),
			atomic_read(&isl->als_needs_enable_flag));
	if (bit_value && (atomic_read(&isl->prox_near))) {
		atomic_set(&isl->als_needs_enable_flag, 1);
		return 0;
	}
	atomic_set(&isl->als_needs_enable_flag, 0);

	return isl29030_set_bit(isl, ISL29030_CONFIGURE,
		ISL29030_CNF_ALS_EN_MASK, bit_value);
}

static int isl29030_set_als_range(struct isl29030_data *isl,
				  unsigned int bit_value)
{
	return isl29030_set_bit(isl, ISL29030_CONFIGURE,
		ISL29030_CNF_ALS_RANGE_MASK, bit_value);
}

static int isl29030_clear_als_flag(struct isl29030_data *isl)
{
	return isl29030_set_bit(isl, ISL29030_INTERRUPT,
		ISL29030_ALS_FLAG_MASK, 0);
}

static int isl29030_clear_prox_flag(struct isl29030_data *isl)
{
	return isl29030_set_bit(isl, ISL29030_INTERRUPT,
		ISL29030_PROX_FLAG_MASK, 0);
}

static int isl29030_clear_prox_and_als_flags(struct isl29030_data *isl)
{
	return isl29030_set_bit(isl, ISL29030_INTERRUPT,
		ISL29030_ALS_FLAG_MASK | ISL29030_PROX_FLAG_MASK, 0);
}

static int isl29030_set_prox_enable(struct isl29030_data *isl,
				    unsigned int bit_value)
{
	int err;
	if (isl29030_debug & ISL29030_DBG_ENABLE_DISABLE)
		pr_info("%s: bit = %d prox_near = %d als_needs_enable = %d\n",
			__func__, bit_value, atomic_read(&isl->prox_near),
			atomic_read(&isl->als_needs_enable_flag));

	err = isl29030_set_bit(isl, ISL29030_CONFIGURE,
		ISL29030_CNF_PROX_EN_MASK,
		bit_value);

	if (!err && !bit_value) {
		atomic_set(&isl->prox_near, 0);
		if (atomic_read(&isl->als_needs_enable_flag))
			return isl29030_set_als_enable(isl, 1);
	}
	return err;
}

static int ld_isl29030_init_registers(struct isl29030_data *isl)
{
	/* as per intersil recommendations */
	if (isl29030_write_reg(isl, ISL29030_CONFIGURE, 0) ||
		isl29030_write_reg(isl, ISL29030_TEST2, 0x29) ||
		isl29030_write_reg(isl, ISL29030_TEST1, 0) ||
		isl29030_write_reg(isl, ISL29030_TEST2, 0)) {

		pr_err("%s:Register initialization failed\n", __func__);
		return -EINVAL;
	}
	msleep(2);

	if (isl29030_write_reg(isl, ISL29030_CONFIGURE,
			isl->pdata->configure) ||
		isl29030_write_reg(isl, ISL29030_INTERRUPT,
			isl->pdata->interrupt_cntrl) ||
		isl29030_write_reg(isl, ISL29030_PROX_LT,
			isl->pdata->prox_lower_threshold) ||
		isl29030_write_reg(isl, ISL29030_PROX_HT,
			isl->pdata->prox_higher_threshold) ||
		isl29030_write_reg(isl, ISL29030_ALSIR_TH1, 0xFF) ||
		isl29030_write_reg(isl, ISL29030_ALSIR_TH2, 0xFF) ||
		isl29030_write_reg(isl, ISL29030_ALSIR_TH3, 0xFF)) {
		pr_err("%s:Register initialization failed\n", __func__);
		return -EINVAL;
	}
	return 0;
}

irqreturn_t ld_isl29030_irq_handler(int irq, void *dev)
{
	struct isl29030_data *isl = dev;

	if (isl->pdata->getIrqStatus)
		irqStatus = isl->pdata->getIrqStatus();
	else
		pr_err("%s: getIrqStatus() is not defined.\n", __func__);

	disable_irq_nosync(isl->client->irq);
	queue_work(isl->working_queue, &isl->wq);
	enable_irq(isl->client->irq);

	return IRQ_HANDLED;
}

static int isl29030_read_adj_als(struct isl29030_data *isl,
				 unsigned int *raw_als_count)
{
	int lens_adj_lux = -1;
	int ret;
	unsigned int als_read_data = 0;
	unsigned char als_lower;
	unsigned char als_upper;

	ret = isl29030_read_reg(isl, ISL29030_ALSIR_DT1, &als_lower);
	if (ret != 0) {
		pr_err("%s:Unable to read ISL29030_ALSIR_DT1 register: %d\n",
			__func__, ret);
		return -EFAULT;
	}
	ret = isl29030_read_reg(isl, ISL29030_ALSIR_DT2, &als_upper);
	if (ret != 0) {
		pr_err("%s:Unable to read ISL29030_ALSIR_DT2 register: %d\n",
			__func__, ret);
		return -EFAULT;
	}

	als_read_data = (als_upper << 8);
	als_read_data |= als_lower;
	if (raw_als_count)
		*raw_als_count = als_read_data;
	else
		pr_err("%s: ERROR ptr NULL\n", __func__);

	if (isl29030_debug & ISL29030_DBG_REPORT_INPUT)
		pr_info("%s:Data read from ALS 0x%X\n",
		__func__, als_read_data);

	lens_adj_lux = (isl->lux_level * als_read_data) /
		(isl->pdata->lens_percent_t * 41);

	return lens_adj_lux;
}

static int isl29030_switch_als_range_and_thresholds(struct isl29030_data *isl,
						    unsigned int raw_als_count)
{
	unsigned int zone_size, als_low, als_high;
	unsigned int switch_range = 0;

	/* turn off ALS since we're going to be reconfiguring it */
	if (isl29030_set_als_enable(isl, 0))
		return -EFAULT;

	/* if we're in the highest low-lux range, switch to high lux
		or if in lowest high-lux range, switch to low lux*/
	if ((isl->lux_level == ISL29030_LOW_LUX_RANGE) &&
		(raw_als_count >= ISL29030_LOW_TO_HIGH_COUNTS)) {
		if (isl29030_debug & ISL29030_DBG_REPORT_INPUT)
			pr_info("%s switching to high lux range\n", __func__);
		isl->lux_level	= ISL29030_HIGH_LUX_RANGE;
		switch_range	= 1;
		raw_als_count	= raw_als_count * ISL29030_LOW_LUX_RANGE
			/ ISL29030_HIGH_LUX_RANGE;

	} else if ((isl->lux_level == ISL29030_HIGH_LUX_RANGE) &&
		(raw_als_count <= ISL29030_HIGH_TO_LOW_COUNTS)) {
		if (isl29030_debug & ISL29030_DBG_REPORT_INPUT)
			pr_info("%s switching to low lux range\n", __func__);
		isl->lux_level	= ISL29030_LOW_LUX_RANGE;
		switch_range	= 1;
		raw_als_count	= raw_als_count * ISL29030_HIGH_LUX_RANGE
			/ ISL29030_LOW_LUX_RANGE;
	}

	zone_size = 1;
	als_low = ((raw_als_count > zone_size) ? raw_als_count - zone_size : 0);
	if (raw_als_count <= 0xFFE) {
		als_high = raw_als_count + zone_size;
		als_high = ((als_high > 0xFFE) ? 0xFFE : als_high);
	} else {
		als_high = 0xFFF;
	}

	/* reconfigure if needed */
	if (switch_range) {
		isl29030_set_als_range(isl,
			(isl->lux_level == ISL29030_LOW_LUX_RANGE) ? 0 : 1);
	}

	if (isl29030_write_reg(isl, ISL29030_ALSIR_TH1, als_low & 0x0FF) ||
		isl29030_write_reg(isl, ISL29030_ALSIR_TH2,
			((als_low & 0xF00) >> 8) | ((als_high & 0x00F) << 4)) ||
		isl29030_write_reg(isl, ISL29030_ALSIR_TH3,
			(als_high & 0xFF0) >> 4)) {
		pr_err("%s couldn't set als thresholds\n", __func__);
		return -EFAULT;
	}

	if (isl29030_set_als_enable(isl, 1))
		return -EFAULT;

	return 0;
}

static int isl29030_report_prox(struct isl29030_data *isl, int force_report)
{
	int ret = 0;
	u8 als_prox_int;
	int prox_near = 0;
	u8 config_reg = 0;

	ret = isl29030_read_reg(isl, ISL29030_INTERRUPT, &als_prox_int);
	if (ret != 0) {
		pr_err("%s:Unable to read interrupt register: %d\n",
			__func__, ret);
		return 1;
	}

	if (als_prox_int & ISL29030_PROX_FLAG_MASK) {
		atomic_set(&isl->prox_near, 1);
		prox_near = 1;
		if (isl29030_debug & ISL29030_DBG_REPORT_INPUT)
			pr_info("%s:Prox near - disabling als if "
				"active due to part limitation\n",
				__func__);
		isl29030_set_als_enable(isl, 0);
	} else {
		atomic_set(&isl->prox_near, 0);
		ret = isl29030_read_reg(isl, ISL29030_CONFIGURE, &config_reg);
		if (ret != 0) {
			pr_err("%s:Unable to read config "
				"register: %d\n",
				__func__, ret);
		} else if ((atomic_read(&isl->als_enabled) == 1) &&
			((config_reg & ISL29030_CNF_ALS_EN_MASK) == 0)) {
			isl29030_set_als_enable(isl, 1);
			if (isl29030_debug & ISL29030_DBG_REPORT_INPUT)
				pr_info("%s: re-enabling als_en bit "
					"in config. reg\n",
					__func__);
		}
	}

	/* Don't report the prox state if it hasn't changed. */
	if (force_report ||
		(prox_near != atomic_read(&isl->prox_reported_near))) {
		atomic_set(&isl->prox_reported_near, prox_near);
		input_report_abs(isl->idev, ABS_DISTANCE,
			prox_near ? PROXIMITY_NEAR : PROXIMITY_FAR);
		input_sync(isl->idev);
	}

	return !prox_near;
}

static void isl29030_report_als(struct isl29030_data *isl)
{
	unsigned int raw_als_data = 0;
	int lux_val;

	lux_val = isl29030_read_adj_als(isl, &raw_als_data);
	isl29030_switch_als_range_and_thresholds(isl, raw_als_data);

	if (lux_val >= 0) {
		input_event(isl->adev, EV_LED, LED_MISC,
			((lux_val > 1) ? lux_val : 2));
		input_sync(isl->adev);
	}
}

static void isl29030_report_input(struct isl29030_data *isl)
{
	int ret = 0;
	int clear_prox = 1;
	u8 als_prox_int;

	ret = isl29030_read_reg(isl, ISL29030_INTERRUPT, &als_prox_int);
	if (ret != 0) {
		pr_err("%s:Unable to read interrupt register: %d\n",
			__func__, ret);
		return;
	}

	if (als_prox_int & ISL29030_ALS_FLAG_MASK)
		isl29030_report_als(isl);

	if (atomic_read(&isl->prox_enabled) == 1)
		clear_prox = isl29030_report_prox(isl, 0);

	if (clear_prox)
		isl29030_clear_prox_and_als_flags(isl);
	else
		isl29030_clear_als_flag(isl);
}

static unsigned int isl29030_get_avg_noise_floor(struct isl29030_data *isl)
{
	int err = -EINVAL;
	unsigned int i, sum, avg;
	u8 prox_data;

	unsigned int num_samples =
		isl->pdata->num_samples_for_noise_floor;

	/* turn off PROX_EN */
	isl29030_set_prox_enable(isl, 0);

	msleep(2);

	avg = 0;
	sum = 0;

	for (i = 0; i < num_samples; i++) {
		/* turn on PROX_EN */
		err = isl29030_set_prox_enable(isl, 1);
		if (err) {
			pr_err("%s:Unable to turn on PROX_EN with error: %d\n",
				__func__, err);
			break;
		}
		msleep(2); /* sleep for a bit before reading PROX_DATA */
		err = isl29030_read_reg(isl, ISL29030_PROX_DATA, &prox_data);
		if (err) {
			pr_err("%s:Unable to read prox data: %d\n",
				__func__, err);
			break;
		}
		/* turn back off */
		err = isl29030_set_prox_enable(isl, 0);
		if (err) {
			pr_err("%s:Unable to turn off PROX_EN with error: %d\n",
				__func__, err);
			break;
		}
		msleep(2);
		sum += prox_data;
		pr_err("%s: prox data sample %d is 0x%x\n",
			__func__, i, prox_data);
	}

	if (!err)
		avg = sum / num_samples;

	if (isl29030_debug & ISL29030_DBG_POWER_ON_OFF)
		pr_info("%s: average is 0x%x ", __func__, avg);
	return avg;
}

static int isl29030_set_prox_thresholds(struct isl29030_data *isl)
{
	unsigned int prox_ht, prox_lt, offset;
	int err = 0;
	unsigned int avg_noise_floor = isl29030_get_avg_noise_floor(isl);

	if ((avg_noise_floor >
		(isl->pdata->crosstalk_vs_covered_threshold)) ||
		(avg_noise_floor == 0)) {
		offset = isl->pdata->default_prox_noise_floor;
	} else {
		offset = avg_noise_floor;
	}
	prox_lt = offset + isl->pdata->prox_lower_threshold;
	prox_ht = offset + isl->pdata->prox_higher_threshold;

	/* check for overflow beyond 1 byte */
	if ((prox_lt > 0xFF) || (prox_ht > 0xFF)) {
		pr_err("%s: noise adjusted proximity thresholds are 0x%x "
			"and 0x%x, overflowing 8 bits, using defaults\n",
			__func__, prox_lt, prox_ht);
		prox_lt  = isl->pdata->prox_lower_threshold;
		prox_ht = isl->pdata->prox_higher_threshold;
	}

	err = isl29030_write_reg(isl, ISL29030_PROX_LT, (u8)prox_lt);
	if (err)
		pr_err("%s:Unable to write PROX_LT: %d\n", __func__, err);
	else {
		err = isl29030_write_reg(isl, ISL29030_PROX_HT, (u8)prox_ht);
		if (err)
			pr_err("%s:Unable to write PROX_HT: %d\n",
				__func__, err);
	}
	return err;
}

static void isl29030_device_power_off(struct isl29030_data *isl)
{
	int err;
	u8 config_reg;
	err = isl29030_set_prox_enable(isl, 0);
	isl29030_clear_prox_flag(isl);

	if (err)
		pr_err("%s:Unable to turn off prox: %d\n", __func__, err);

	isl29030_clear_als_flag(isl);

	err = isl29030_read_reg(isl, ISL29030_CONFIGURE, &config_reg);

	if (err)
		pr_err("%s: unable to read config reg: %d\n", __func__, err);

	if ((atomic_read(&isl->als_enabled) == 1) &&
		((config_reg & ISL29030_CNF_ALS_EN_MASK) == 0)) {
		isl29030_set_als_enable(isl, 1);
		if (isl29030_debug & ISL29030_DBG_POWER_ON_OFF)
			pr_info("%s: re-enabling als_en bit in config. reg\n",
				__func__);

	}

	return;
}

static int isl29030_device_power_on(struct isl29030_data *isl)
{
	int err;

	if (isl29030_debug & ISL29030_DBG_POWER_ON_OFF)
		pr_info("%s\n", __func__);

	err = isl29030_set_prox_thresholds(isl);
	if (err)
		pr_err("%s:Unable to set prox thresholds: %d\n",
			__func__, err);
	else{
		err = isl29030_set_prox_enable(isl, 1);

		if (err)
			pr_err("%s:Unable to turn on prox: %d\n",
				__func__, err);
	}
	return err;
}

int isl29030_enable(struct isl29030_data *isl)
{
	int err;

	if (!atomic_cmpxchg(&isl->prox_enabled, 0, 1)) {
		err = isl29030_device_power_on(isl);
		if (err) {
			atomic_set(&isl->prox_enabled, 0);
			return err;
		}
		/* only enable irq if not already done so */
		if (atomic_read(&isl->als_enabled) == 0) {
			if (isl29030_debug & ISL29030_DBG_ENABLE_DISABLE)
				pr_info("%s: als is not enabled, "
					"enabling IRQ for prox\n", __func__);
			enable_irq(isl->client->irq);
		}
		isl29030_report_prox(isl, 1);
	}

	return 0;
}

int isl29030_disable(struct isl29030_data *isl)
{
	int ret = 0;

	if (atomic_cmpxchg(&isl->prox_enabled, 1, 0)) {
		isl29030_device_power_off(isl);
		if (atomic_read(&isl->als_enabled) == 0) {
			if (isl29030_debug & ISL29030_DBG_ENABLE_DISABLE)
				pr_info("%s: ALS is not enabled, "
					"disabling IRQ\n", __func__);

			disable_irq_nosync(isl->client->irq);
			ret = cancel_work_sync(&isl->wq);
			if (ret) {
				pr_err("%s: Not Suspending\n", __func__);
				enable_irq(isl->client->irq);
				return -EBUSY;
			}
		}

	}

	return 0;
}

static int isl29030_misc_open(struct inode *inode, struct file *file)
{
	int err;

	err = nonseekable_open(inode, file);
	if (err < 0)
		return err;

	file->private_data = isl29030_misc_data;

	return 0;
}

static long isl29030_misc_ioctl(struct file *file,
			       unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	u8 enable;
	struct isl29030_data *isl = file->private_data;

	if (isl29030_debug & ISL29030_DBG_MISC_IOCTL)
		pr_info("%s: cmd = %d\n", __func__, cmd);

	switch (cmd) {
	case ISL29030_IOCTL_SET_ENABLE:
		if (copy_from_user(&enable, argp, 1))
			return -EFAULT;
		if (enable > 1)
			return -EINVAL;

		if (enable != 0)
			isl29030_enable(isl);
		else
			isl29030_disable(isl);

		break;

	case ISL29030_IOCTL_GET_ENABLE:
		enable = atomic_read(&isl->prox_enabled);
		if (copy_to_user(argp, &enable, 1))
			return -EINVAL;

		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static const struct file_operations isl29030_misc_fops = {
	.owner = THIS_MODULE,
	.open = isl29030_misc_open,
	.unlocked_ioctl = isl29030_misc_ioctl,
};

static struct miscdevice isl29030_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = FOPS_ISL29030_NAME,
	.fops = &isl29030_misc_fops,
};

static ssize_t ld_isl29030_registers_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct isl29030_data *als_ir_data = i2c_get_clientdata(client);
	unsigned i, n, reg_count;
	u8 value;

	reg_count = sizeof(isl29030_regs) / sizeof(isl29030_regs[0]);
	for (i = 0, n = 0; i < reg_count; i++) {
		isl29030_read_reg(als_ir_data, isl29030_regs[i].reg, &value);
		n += scnprintf(buf + n, PAGE_SIZE - n,
			"%-20s = 0x%02X\n",
			isl29030_regs[i].name,
			value);
	}

	return n;
}

static ssize_t ld_isl29030_registers_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct isl29030_data *als_ir_data = i2c_get_clientdata(client);
	unsigned i, reg_count, value;
	int error;
	char name[30];

	if (count >= 30) {
		pr_err("%s:input too long\n", __func__);
		return -EFAULT;
	}

	if (sscanf(buf, "%30s %x", name, &value) != 2) {
		pr_err("%s:unable to parse input\n", __func__);
		return -EFAULT;
	}

	reg_count = sizeof(isl29030_regs) / sizeof(isl29030_regs[0]);
	for (i = 0; i < reg_count; i++) {
		if (!strcmp(name, isl29030_regs[i].name)) {
			switch (isl29030_regs[i].reg) {

			case ISL29030_PROX_LT:
				als_ir_data->pdata->
					prox_lower_threshold = value;
			break;

			case ISL29030_PROX_HT:
				als_ir_data->pdata->
					prox_higher_threshold = value;
			break;

			case ISL29030_TEST1:
				isl29030_enable(als_ir_data);
				return count;
			break;

			case ISL29030_TEST2:
				isl29030_disable(als_ir_data);
				return count;
			break;
			}
			error = isl29030_write_reg(als_ir_data,
				isl29030_regs[i].reg,
				value);
			if (error) {
				pr_err("%s:Failed to write register %s\n",
					__func__, name);
				return -EFAULT;
			}
			return count;
		}
	}

	pr_err("%s:no such register %s\n", __func__, name);
	return -EFAULT;
}

static DEVICE_ATTR(registers, 0644, ld_isl29030_registers_show,
		   ld_isl29030_registers_store);

void ld_isl29030_work_queue(struct work_struct *work)
{
	struct isl29030_data *isl =
		container_of(work, struct isl29030_data, wq);

	isl29030_report_input(isl);
}

static int ld_isl29030_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	struct isl29030_platform_data *pdata = client->dev.platform_data;
	struct isl29030_data *isl;
	int error = 0;

	if (pdata == NULL) {
		pr_err("%s: platform data required\n", __func__);
		return -ENODEV;
	}

	client->irq = pdata->irq;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s:I2C_FUNC_I2C not supported\n", __func__);
		return -ENODEV;
	}

#ifdef USE_DYNAMIC_MEMORY_ALLOCATION
	isl = kzalloc(sizeof(struct isl29030_data), GFP_KERNEL);
	if (isl == NULL) {
		error = -ENOMEM;
		goto err_alloc_data_failed;
	}
#else
	isl = &(GL_isl);
#endif
	isl->client = client;
	isl->pdata = pdata;

	if (isl->pdata->init) {
		error = isl->pdata->init();
		if (error < 0) {
			pr_err("%s:init failed: %d\n", __func__, error);
			goto error_init_failed;
		}
	}

	isl->idev = input_allocate_device();
	if (!isl->idev) {
		error = -ENOMEM;
		pr_err("%s: input device allocate failed: %d\n", __func__,
			error);
		goto error_input_allocate_failed_ir;
	}

	isl->idev->name = "proximity";
	input_set_capability(isl->idev, EV_ABS, ABS_DISTANCE);

	isl->adev = input_allocate_device();
	if (!isl->adev) {
		error = -ENOMEM;
		pr_err("%s: input device allocate failed: %d\n", __func__,
			error);
		goto error_input_allocate_failed_als;
	}

	isl->adev->name = "als";
	input_set_capability(isl->adev, EV_LED, LED_MISC);

	error = misc_register(&isl29030_misc_device);
	if (error < 0) {
		pr_err("%s: isl29030 register failed\n", __func__);
		goto error_misc_register_failed;
	}

	isl->lux_level = ISL29030_HIGH_LUX_RANGE;

	atomic_set(&isl->prox_enabled, 0);
	atomic_set(&isl->als_enabled, 1);
	atomic_set(&isl->prox_near, 0);
	atomic_set(&isl->als_needs_enable_flag, 0);

	isl->working_queue = create_singlethread_workqueue("als_wq");
	if (!isl->working_queue) {
		pr_err("%s: Cannot create work queue\n", __func__);
		error = -ENOMEM;
		goto error_create_wq_failed;
	}

	INIT_WORK(&isl->wq, ld_isl29030_work_queue);

	error = request_irq(client->irq,
		ld_isl29030_irq_handler,
		(IRQF_DISABLED | IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING),
		LD_ISL29030_NAME, isl);
	if (error != 0) {
		pr_err("%s: irq request failed: %d\n", __func__, error);
		error = -ENODEV;
		goto err_req_irq_failed;
	}
	enable_irq_wake(client->irq);

	i2c_set_clientdata(client, isl);

	mutex_init(&isl->lock);

	mutex_init(&isl->bit_lock);

	error = input_register_device(isl->idev);
	if (error) {
		pr_err("%s: input device register failed:%d\n", __func__,
			error);
		goto error_input_register_failed_ir;
	}

	error = input_register_device(isl->adev);
	if (error) {
		pr_err("%s: input device register failed:%d\n", __func__,
			error);
		goto error_input_register_failed_als;
	}

	if (isl->pdata->power_on) {
		error = isl->pdata->power_on();
		if (error < 0) {
			pr_err("%s:power_on failed: %d\n", __func__, error);
			goto error_power_on_failed;
		}
	}

	error = ld_isl29030_init_registers(isl);
	if (error < 0) {
		pr_err("%s: Register Initialization failed: %d\n",
			__func__, error);
		error = -ENODEV;
		goto err_reg_init_failed;
	}

	error = device_create_file(&client->dev, &dev_attr_registers);
	if (error < 0) {
		pr_err("%s:File device creation failed: %d\n", __func__, error);
		error = -ENODEV;
		goto err_create_registers_file_failed;
	}

	irqStatus = 0;
#ifdef CONFIG_HAS_EARLYSUSPEND
	isl->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	isl->early_suspend.suspend = isl29030_early_suspend;
	isl->early_suspend.resume = isl29030_late_resume;
	register_early_suspend(&isl->early_suspend);
#endif
	isl29030_misc_data = isl;
	isl29030_report_input(isl);

	return 0;

err_create_registers_file_failed:
err_reg_init_failed:
	if (isl->pdata->power_off)
		isl->pdata->power_off();
error_power_on_failed:
	input_unregister_device(isl->adev);
	isl->adev = NULL;
error_input_register_failed_als:
	input_unregister_device(isl->idev);
	isl->idev = NULL;
error_input_register_failed_ir:
	mutex_destroy(&isl->lock);
	mutex_destroy(&isl->bit_lock);
	i2c_set_clientdata(client, NULL);
	free_irq(isl->client->irq, isl);
err_req_irq_failed:
	destroy_workqueue(isl->working_queue);
error_create_wq_failed:
	misc_deregister(&isl29030_misc_device);
error_misc_register_failed:
	input_free_device(isl->adev);
error_input_allocate_failed_als:
	input_free_device(isl->idev);
error_input_allocate_failed_ir:
	if (isl->pdata->exit)
		isl->pdata->exit();
error_init_failed:
#ifdef USE_DYNAMIC_MEMORY_ALLOCATION
	kfree(isl);
err_alloc_data_failed:
#endif
	return error;
}

static int ld_isl29030_remove(struct i2c_client *client)
{
	struct isl29030_data *isl = i2c_get_clientdata(client);

	input_unregister_device(isl->idev);
	input_unregister_device(isl->adev);

	free_irq(isl->client->irq, isl);

	if (isl->working_queue)
		destroy_workqueue(isl->working_queue);

	if (isl->pdata->power_off)
		isl->pdata->power_off();
	if (isl->pdata->exit)
		isl->pdata->exit();

	kfree(isl);
	return 0;
}

static int isl29030_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct isl29030_data *isl = i2c_get_clientdata(client);
	int ret;

	if (isl29030_debug & ISL29030_DBG_SUSPEND_RESUME)
		pr_info("%s: Suspending\n", __func__);

	if (atomic_cmpxchg(&isl->als_enabled, 1, 0)) {

		ret = isl29030_set_als_enable(isl, 0);
		isl29030_clear_als_flag(isl);

		if (ret) {
			pr_err("%s:Unable to turn off ALS_EN: %d\n",
				__func__, ret);
			atomic_set(&isl->als_enabled, 1);
			return ret;
		}
		if (isl29030_debug & ISL29030_DBG_SUSPEND_RESUME)
			pr_info("%s: turned off ALS\n", __func__);

		if (atomic_read(&isl->prox_enabled) == 0) {
			if (isl29030_debug & ISL29030_DBG_SUSPEND_RESUME)
				pr_info("%s: prox is not enabled, "
					"disabling IRQ\n", __func__);

			disable_irq_nosync(isl->client->irq);
			ret = cancel_work_sync(&isl->wq);
			if (ret) {
				pr_err("%s: Not Suspending\n", __func__);
				enable_irq(isl->client->irq);
				return -EBUSY;
			}
		}
	} else {
		if (isl29030_debug & ISL29030_DBG_SUSPEND_RESUME)
			pr_info("%s: ALS already disabled\n", __func__);
	}
	return 0;
}

static int isl29030_resume(struct i2c_client *client)
{
	struct isl29030_data *isl = i2c_get_clientdata(client);
	int ret;

	if (isl29030_debug & ISL29030_DBG_SUSPEND_RESUME)
		pr_info("%s: Resuming\n", __func__);

	if (!atomic_cmpxchg(&isl->als_enabled, 0, 1)) {
		ret = isl29030_set_als_enable(isl, 1);
		if (ret) {
			pr_err("%s:Unable to turn on ALS: %d\n",
				__func__, ret);
			atomic_set(&isl->als_enabled, 0);
			return ret;
		}
		/* Allow the ALS sensor to read the zone */
		msleep(100);
		isl29030_report_als(isl);
		isl29030_clear_als_flag(isl);

		if (atomic_read(&isl->prox_enabled) == 0) {
			if (isl29030_debug & ISL29030_DBG_SUSPEND_RESUME)
				pr_info("%s: PROX not enabled so turn on IRQ "
					"for ALS benefit\n", __func__);
			enable_irq(isl->client->irq);
		}
	} else {
		if (isl29030_debug & ISL29030_DBG_SUSPEND_RESUME)
			pr_info("%s: ALS already activated\n", __func__);
	}

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void isl29030_early_suspend(struct early_suspend *handler)
{
	struct isl29030_data *isl;

	isl = container_of(handler, struct isl29030_data,
		early_suspend);
	isl29030_suspend(isl->client, PMSG_SUSPEND);
}

static void isl29030_late_resume(struct early_suspend *handler)
{
	struct isl29030_data *isl;

	isl = container_of(handler, struct isl29030_data,
		early_suspend);
	isl29030_resume(isl->client);
}
#endif

static const struct i2c_device_id isl29030_id[] = {
	{LD_ISL29030_NAME, 0},
	{}
};

static struct i2c_driver ld_isl29030_i2c_driver = {
	.probe		= ld_isl29030_probe,
	.remove		= ld_isl29030_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= isl29030_suspend,
	.resume		= isl29030_resume,
#endif
	.id_table	= isl29030_id,
	.driver = {
		.name = LD_ISL29030_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init ld_isl29030_init(void)
{
	return i2c_add_driver(&ld_isl29030_i2c_driver);
}

static void __exit ld_isl29030_exit(void)
{
	i2c_del_driver(&ld_isl29030_i2c_driver);
}

module_init(ld_isl29030_init);
module_exit(ld_isl29030_exit);

MODULE_DESCRIPTION("ALS and Proximity driver for ISL29030");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");
