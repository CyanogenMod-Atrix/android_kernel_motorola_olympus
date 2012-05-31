/* Copyright (C) 2011-2012 NVIDIA Corporation.
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
/* This is a NVC kernel driver for a focuser device called
 * ad5816.
 */
/* Implementation
 * --------------
 * The board level details about the device need to be provided in the board
 * file with the <device>_platform_data structure.
 * Standard among NVC kernel drivers in this structure is:
 * .cfg = Use the NVC_CFG_ defines that are in nvc.h.
 *  Descriptions of the configuration options are with the defines.
 *      This value is typically 0.
 * .num = The number of the instance of the device.  This should start at 1 and
 *      and increment for each device on the board.  This number will be
 *      appended to the MISC driver name, Example: /dev/focuser.1
 *      If not used or 0, then nothing is appended to the name.
 * .sync = If there is a need to synchronize two devices, then this value is
 *       the number of the device instance (.num above) this device is to
 *       sync to.  For example:
 *       Device 1 platform entries =
 *       .num = 1,
 *       .sync = 2,
 *       Device 2 platfrom entries =
 *       .num = 2,
 *       .sync = 1,
 *       The above example sync's device 1 and 2.
 *       To disable sync, set .sync = 0.  Note that the .num = 0 device is not
 *       allowed to be synced to.
 *       This is typically used for stereo applications.
 * .dev_name = The MISC driver name the device registers as.  If not used,
 *       then the part number of the device is used for the driver name.
 *       If using the NVC user driver then use the name found in this
 *       driver under _default_pdata.
 * .gpio_count = The ARRAY_SIZE of the nvc_gpio_pdata table.
 * .gpio = A pointer to the nvc_gpio_pdata structure's platform GPIO data.
 *       The GPIO mechanism works by cross referencing the .gpio_type key
 *       among the nvc_gpio_pdata GPIO data and the driver's nvc_gpio_init
 *       GPIO data to build a GPIO table the driver can use.  The GPIO's
 *       defined in the device header file's _gpio_type enum are the
 *       gpio_type keys for the nvc_gpio_pdata and nvc_gpio_init structures.
 *       These need to be present in the board file's nvc_gpio_pdata
 *       structure for the GPIO's that are used.
 *       The driver's GPIO logic uses assert/deassert throughout until the
 *       low level _gpio_wr/rd calls where the .assert_high is used to
 *       convert the value to the correct signal level.
 *       See the GPIO notes in nvc.h for additional information.
 *
 * The following is specific to NVC kernel focus drivers:
 * .nvc = Pointer to the nvc_focus_nvc structure.  This structure needs to
 *      be defined and populated if overriding the driver defaults.
 * .cap = Pointer to the nvc_focus_cap structure.  This structure needs to
 *      be defined and populated if overriding the driver defaults.
 *
 * The following is specific to this NVC kernel focus driver:
 * .info = Pointer to the ad5816_pdata_info structure.  This structure does
 *       not need to be defined and populated unless overriding ROM data.
 *
 * Power Requirements:
 * The device's header file defines the voltage regulators needed with the
 * enumeration <device>_vreg.  The order these are enumerated is the order
 * the regulators will be enabled when powering on the device.  When the
 * device is powered off the regulators are disabled in descending order.
 * The <device>_vregs table in this driver uses the nvc_regulator_init
 * structure to define the regulator ID strings that go with the regulators
 * defined with <device>_vreg.  These regulator ID strings (or supply names)
 * will be used in the regulator_get function in the _vreg_init function.
 * The board power file and <device>_vregs regulator ID strings must match.
 */

#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/list.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <media/ad5816.h>

#define AD5816_ID				0x04
#define AD5816_FOCAL_LENGTH	 (4.570f)
#define AD5816_FNUMBER			(2.8f)
#define AD5816_ACTUATOR_RANGE	680
#define AD5816_SETTLETIME		110
#define AD5816_FOCUS_MACRO		810
#define AD5816_FOCUS_INFINITY	50 /* Exact value needs to be decided */
#define AD5816_POS_LOW_DEFAULT  220
#define AD5816_POS_HIGH_DEFAULT 900
/* Need to decide exact value of VCM_THRESHOLD and its use */
/* define AD5816_VCM_THRESHOLD	20 */

static u8 ad5816_ids[] = {
	0x04,
};

static struct nvc_gpio_init ad5816_gpios[] = {
	{ AD5816_GPIO_RESET, GPIOF_OUT_INIT_LOW, "reset", false, true, },
	{ AD5816_GPIO_I2CMUX, 0, "i2c_mux", 0, false},
	{ AD5816_GPIO_GP1, 0, "gp1", 0, false},
	{ AD5816_GPIO_GP2, 0, "gp2", 0, false},
	{ AD5816_GPIO_GP3, 0, "gp3", 0, false},
};

static struct nvc_regulator_init ad5816_vregs[] = {
	{ AD5816_VREG_VDD, "vdd"},
	{ AD5816_VREG_VDD_AF, "vdd_af"},
	{ AD5816_VREG_VDD_I2C, "vdd_i2c"},
};

struct ad5816_info {
	atomic_t in_use;
	struct i2c_client *i2c_client;
	struct ad5816_platform_data *pdata;
	struct miscdevice miscdev;
	struct list_head list;
	struct nvc_gpio gpio[ARRAY_SIZE(ad5816_gpios)];
	struct nvc_regulator vreg[ARRAY_SIZE(ad5816_vregs)];
	int pwr_api;
	int pwr_dev;
	int id_minor;
	u32 pos;
	u8 s_mode;
	bool reset_flag;
	struct ad5816_info *s_info;
	struct nvc_focus_nvc nvc;
	struct nvc_focus_cap cap;
	struct ad5816_pdata_info config;
};

/**
 * The following are default values
 */

static struct ad5816_pdata_info ad5816_default_info = {
	.pos_low = AD5816_POS_LOW_DEFAULT,
	.pos_high = AD5816_POS_HIGH_DEFAULT,
};

static struct nvc_focus_cap ad5816_default_cap = {
	.version = NVC_FOCUS_CAP_VER2,
	.actuator_range = AD5816_ACTUATOR_RANGE,
	.settle_time = AD5816_SETTLETIME,
	.focus_macro = AD5816_FOCUS_MACRO,
	.focus_infinity = AD5816_FOCUS_INFINITY,
};

static struct nvc_focus_nvc ad5816_default_nvc = {
	.focal_length = AD5816_FOCAL_LENGTH,
	.fnumber = AD5816_FNUMBER,
};

static struct ad5816_platform_data ad5816_default_pdata = {
	.cfg = 0,
	.num = 0,
	.sync = 0,
	.dev_name = "focuser",
};
static LIST_HEAD(ad5816_info_list);
static DEFINE_SPINLOCK(ad5816_spinlock);

static int ad5816_i2c_rd8(struct ad5816_info *info, u8 addr, u8 reg, u8 *val)
{
	struct i2c_msg msg[2];
	u8 buf[2];
	buf[0] = reg;
	if (addr) {
		msg[0].addr = addr;
		msg[1].addr = addr;
	} else {
		msg[0].addr = info->i2c_client->addr;
		msg[1].addr = info->i2c_client->addr;
	}
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &buf[0];
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = &buf[1];
	*val = 0;
	if (i2c_transfer(info->i2c_client->adapter, msg, 2) != 2)
		return -EIO;
	*val = buf[1];
	return 0;
}

static int ad5816_i2c_wr8(struct ad5816_info *info, u8 reg, u8 val)
{
	struct i2c_msg msg;
	u8 buf[2];
	buf[0] = reg;
	buf[1] = val;
	msg.addr = info->i2c_client->addr;
	msg.flags = 0;
	msg.len = 2;
	msg.buf = &buf[0];
	if (i2c_transfer(info->i2c_client->adapter, &msg, 1) != 1)
		return -EIO;
	return 0;
}

static int ad5816_i2c_rd16(struct ad5816_info *info, u8 reg, u16 *val)
{
	struct i2c_msg msg[2];
	u8 buf[3];
	buf[0] = reg;
	msg[0].addr = info->i2c_client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &buf[0];
	msg[1].addr = info->i2c_client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 2;
	msg[1].buf = &buf[1];
	if (i2c_transfer(info->i2c_client->adapter, msg, 2) != 2)
		return -EIO;
	*val = (((u16)buf[1] << 8) | (u16)buf[2]);
	return 0;
}

static int ad5816_i2c_wr16(struct ad5816_info *info, u8 reg, u16 val)
{
	struct i2c_msg msg;
	u8 buf[3];
	buf[0] = reg;
	buf[1] = (u8)(val >> 8);
	buf[2] = (u8)(val & 0xff);
	msg.addr = info->i2c_client->addr;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = &buf[0];
	if (i2c_transfer(info->i2c_client->adapter, &msg, 1) != 1)
		return -EIO;
	return 0;
}

static int ad5816_gpio_wr(struct ad5816_info *info,
						enum ad5816_gpio_types i,
						int val) /* val: 0=deassert, 1=assert */
{
	int err = -EINVAL;
	if (info->gpio[i].valid) {
		if (val)
			val = 1;
		if (!info->gpio[i].active_high)
			val = !val;
		val &= 1;
		err = val;
		gpio_set_value_cansleep(info->gpio[i].gpio, val);
		dev_dbg(&info->i2c_client->dev, "%s %u %d\n", __func__, info->gpio[i].gpio, val);
	}
	return err; /* return value written or error */
}

static int ad5816_gpio_reset(struct ad5816_info *info, int val)
{
	int err = 0;

	if (val) {
		if (!info->reset_flag) {
			info->reset_flag = true;
			err = ad5816_gpio_wr(info, AD5816_GPIO_RESET, 1);
			if (err < 0)
				return 0; /* flag no reset */

			mdelay(1);
			ad5816_gpio_wr(info, AD5816_GPIO_RESET, 0);
			mdelay(10); /* startup delay needs to be modified*/
			err = 1; /* flag that a reset was done */
		}
	} else {
		info->reset_flag = false;
	}
	return err;
}

static void ad5816_gpio_able(struct ad5816_info *info, int val)
{
	/**
	* This is a feature that allows driver to control GPIOs
	* that may be needed for the board (not the device).
	* */
	if (val) {
		ad5816_gpio_wr(info, AD5816_GPIO_GP1, val);
		ad5816_gpio_wr(info, AD5816_GPIO_GP2, val);
		ad5816_gpio_wr(info, AD5816_GPIO_GP3, val);
	} else {
		ad5816_gpio_wr(info, AD5816_GPIO_GP3, val);
		ad5816_gpio_wr(info, AD5816_GPIO_GP2, val);
		ad5816_gpio_wr(info, AD5816_GPIO_GP1, val);
	}
}
static void ad5816_gpio_exit(struct ad5816_info *info)
{
	unsigned i;
	for (i = 0; i <= ARRAY_SIZE(ad5816_gpios); i++) {
		if (info->gpio[i].flag && info->gpio[i].own) {
			gpio_free(info->gpio[i].gpio);
			info->gpio[i].own = false;
		}
	}
}

static void ad5816_gpio_init(struct ad5816_info *info)
{
	char label[32];
	unsigned long flags;
	unsigned type;
	unsigned i;
	unsigned j;
	int err;
	for (i = 0; i < ARRAY_SIZE(ad5816_gpios); i++)
		info->gpio[i].flag = false;

	if (!info->pdata->gpio_count || !info->pdata->gpio)
		return;

	for (i = 0; i < ARRAY_SIZE(ad5816_gpios); i++) {
		type = ad5816_gpios[i].gpio_type;

		for (j = 0; j < info->pdata->gpio_count; j++) {
			if (type == info->pdata->gpio[j].gpio_type)
				break;
		}

		if (j == info->pdata->gpio_count)
			continue;
		info->gpio[type].gpio = info->pdata->gpio[j].gpio;
		info->gpio[type].flag = true;

		if (ad5816_gpios[i].use_flags) {
			flags = ad5816_gpios[i].flags;
			info->gpio[type].active_high = ad5816_gpios[i].active_high;
		} else {
			info->gpio[type].active_high = info->pdata->gpio[j].active_high;
			if (info->gpio[type].active_high)
				flags = GPIOF_OUT_INIT_LOW;
			else
				flags = GPIOF_OUT_INIT_HIGH;
		}

		if (!info->pdata->gpio[j].init_en)
			continue;
		snprintf(label, sizeof(label), "ad5816_%u_%s",
				info->pdata->num, ad5816_gpios[i].label);
		err = gpio_request_one(info->gpio[type].gpio, flags, label);
		if (err) {
			dev_err(&info->i2c_client->dev, "%s ERR %s %u\n",
					__func__, label, info->gpio[type].gpio);
		} else {
			info->gpio[type].own = true;
			dev_dbg(&info->i2c_client->dev, "%s %s %u\n",
					__func__, label, info->gpio[type].gpio);
		}
	}
}

static int ad5816_vreg_dis(struct ad5816_info *info,
			enum ad5816_vreg i)
{
	int err = 0;
	if (info->vreg[i].vreg_flag && (info->vreg[i].vreg != NULL)) {
		err = regulator_disable(info->vreg[i].vreg);
		if (!err)
			dev_dbg(&info->i2c_client->dev, "%s: %s\n",
					__func__, info->vreg[i].vreg_name);
		else
			dev_err(&info->i2c_client->dev, "%s %s ERR\n",
					__func__, info->vreg[i].vreg_name);
	}
	info->vreg[i].vreg_flag = false;
	return err;
}

static int ad5816_vreg_dis_all(struct ad5816_info *info)
{
	unsigned i;
	int err = 0;
	for (i = ARRAY_SIZE(ad5816_vregs); i > 0; i--)
		err |= ad5816_vreg_dis(info, (i - 1));
	return err;
}

static int ad5816_vreg_en(struct ad5816_info *info,
							enum ad5816_vreg i)
{
	int err = 0;
	if (!info->vreg[i].vreg_flag && (info->vreg[i].vreg != NULL)) {
		err = regulator_enable(info->vreg[i].vreg);

		if (!err) {
			dev_dbg(&info->i2c_client->dev, "%s: %s\n",
					__func__, info->vreg[i].vreg_name);
			info->vreg[i].vreg_flag = true;
			err = 1; /* flag regulator state change */
		} else {
			dev_err(&info->i2c_client->dev, "%s %s ERR\n",
					__func__, info->vreg[i].vreg_name);
		}

	}
	return err;
}

static int ad5816_vreg_en_all(struct ad5816_info *info)
{
	unsigned i;
	int err = 0;
	for (i = 0; i < ARRAY_SIZE(ad5816_vregs); i++)
		err |= ad5816_vreg_en(info, i);
	return err;
}

static void ad5816_vreg_exit(struct ad5816_info *info)
{
	unsigned i;
	for (i = 0; i < ARRAY_SIZE(ad5816_vregs); i++) {
		regulator_put(info->vreg[i].vreg);
		info->vreg[i].vreg = NULL;
	}
}

static int ad5816_vreg_init(struct ad5816_info *info)
{
	unsigned i;
	unsigned j;
	int err = 0;
	for (i = 0; i < ARRAY_SIZE(ad5816_vregs); i++) {
		j = ad5816_vregs[i].vreg_num;
		info->vreg[j].vreg_name = ad5816_vregs[i].vreg_name;
		info->vreg[j].vreg_flag = false;
		info->vreg[j].vreg = regulator_get(&info->i2c_client->dev,
								info->vreg[j].vreg_name);
		if (IS_ERR_OR_NULL(info->vreg[j].vreg)) {
			dev_dbg(&info->i2c_client->dev, "%s %s ERR: %d\n",
					__func__, info->vreg[j].vreg_name,
					(int)info->vreg[j].vreg);
			err |= PTR_ERR(info->vreg[j].vreg);
			info->vreg[j].vreg = NULL;
		} else {
			dev_dbg(&info->i2c_client->dev, "%s: %s\n",
					__func__, info->vreg[j].vreg_name);
		}
	}
	return err;
}

void ad5816_set_power_down(struct ad5816_info *info)
{
	int err;
	u16 data = 0x0000;
	err = ad5816_i2c_wr16(info, VCM_CODE_MSB, data);
	if (err)
		dev_err(&info->i2c_client->dev, " %s: failed \n",
			__func__);
}

void ad5816_set_arc_mode(struct ad5816_info *info)
{
	int err = 0;
	/* set ARC enable */
	err = ad5816_i2c_wr8(info, CONTROL, 0x02);
	if (err)
		dev_err(&info->i2c_client->dev,
		"%s: CONTROL reg write failed \n", __func__);

	/* set the ARC RES2 */
	err = ad5816_i2c_wr8(info, MODE, 0x01);
	if (err)
		dev_err(&info->i2c_client->dev,
		"%s: MODE reg write failed \n", __func__);

	/* set the VCM_FREQ to 12.8mS */
	err = ad5816_i2c_wr8(info, VCM_FREQ, 0x80);
	if (err)
		dev_err(&info->i2c_client->dev,
		"%s: VCM_FREQ reg write failed \n", __func__);
}

static int ad5816_pm_wr(struct ad5816_info *info, int pwr)
{
	int err = 0;

	if ((info->pdata->cfg & (NVC_CFG_OFF2STDBY | NVC_CFG_BOOT_INIT)) &&
		(pwr == NVC_PWR_OFF ||
		pwr == NVC_PWR_STDBY_OFF))
			pwr = NVC_PWR_STDBY;

	if (pwr == info->pwr_dev)
		return 0;

	switch (pwr)
	{
		case NVC_PWR_OFF_FORCE:
		case NVC_PWR_OFF:
			err = ad5816_vreg_dis_all(info);
			ad5816_gpio_able(info, 0);
			ad5816_gpio_reset(info, 0);
			break;
		case NVC_PWR_STDBY_OFF:
		case NVC_PWR_STDBY:
			err = ad5816_vreg_en_all(info);
			ad5816_gpio_able(info, 1);
			ad5816_gpio_reset(info, 1);
			break;
		case NVC_PWR_COMM:
		case NVC_PWR_ON:
			err = ad5816_vreg_en_all(info);
			ad5816_gpio_able(info, 1);
			ad5816_gpio_reset(info, 1);
			break;
		default:
			err = -EINVAL;
			break;
	}

	if (err < 0) {
		dev_err(&info->i2c_client->dev, "%s err %d\n", __func__, err);
		pwr = NVC_PWR_ERR;
	}

	info->pwr_dev = pwr;
	dev_dbg(&info->i2c_client->dev, "%s pwr_dev=%d\n", __func__, info->pwr_dev);

	if (err > 0)
		return 0;

	return err;
}
static int ad5816_pm_wr_s(struct ad5816_info *info, int pwr)
{
	int err1 = 0;
	int err2 = 0;
	if ((info->s_mode == NVC_SYNC_OFF) ||
		(info->s_mode == NVC_SYNC_MASTER) ||
		(info->s_mode == NVC_SYNC_STEREO))
		err1 = ad5816_pm_wr(info, pwr);
	if ((info->s_mode == NVC_SYNC_SLAVE) ||
		(info->s_mode == NVC_SYNC_STEREO))
		err2 = ad5816_pm_wr(info->s_info, pwr);
	return err1 | err2;
}

static int ad5816_pm_api_wr(struct ad5816_info *info, int pwr)
{
	int err = 0;
	if (!pwr || (pwr > NVC_PWR_ON))
		return 0;
	if (pwr > info->pwr_dev) {
		err = ad5816_pm_wr_s(info, pwr);
	}
	if (!err) {
		info->pwr_api = pwr;
	} else
		info->pwr_api = NVC_PWR_ERR;
	if (info->pdata->cfg & NVC_CFG_NOERR)
		return 0;
	return err;
}

static int ad5816_pm_dev_wr(struct ad5816_info *info, int pwr)
{
	if (pwr < info->pwr_api)
		pwr = info->pwr_api;
	return ad5816_pm_wr(info, pwr);
}

static void ad5816_pm_exit(struct ad5816_info *info)
{
	ad5816_pm_wr(info, NVC_PWR_OFF_FORCE);
	ad5816_vreg_exit(info);
	ad5816_gpio_exit(info);
}
static void ad5816_pm_init(struct ad5816_info *info)
{
	ad5816_gpio_init(info);
	ad5816_vreg_init(info);
}

static int ad5816_reset(struct ad5816_info *info, u32 level)
{
	int err;
	if (level == NVC_RESET_SOFT) {
		err = ad5816_pm_wr(info, NVC_PWR_COMM);
		err |= ad5816_i2c_wr8(info, CONTROL, 0x01); /* SW reset */
	} else {
		err = ad5816_pm_wr(info, NVC_PWR_OFF_FORCE);
	}
	err |= ad5816_pm_wr(info, info->pwr_api);
	return err;
}

static int ad5816_dev_id(struct ad5816_info *info)
{
	u16 val = 0;
	unsigned i;
	int err;
	ad5816_pm_dev_wr(info, NVC_PWR_COMM);
	err = ad5816_i2c_rd16(info, IC_INFO, &val);
	if (!err) {
		dev_dbg(&info->i2c_client->dev, "%s found devId: %x\n", __func__, val);
		info->id_minor = 0;
		val = val & 0xff;
		for (i = 0; i < ARRAY_SIZE(ad5816_ids); i++) {
			if (val == ad5816_ids[i]) {
				info->id_minor = val;
				break;
			}
		}
		if (!info->id_minor) {
			err = -ENODEV;
			dev_dbg(&info->i2c_client->dev, "%s No devId match\n", __func__);
		}
	}
	ad5816_pm_dev_wr(info, NVC_PWR_OFF);
	return err;
}

static void ad5816_sts_rd(struct ad5816_info *info)
{
	/**
	* Device specific code for status
	*
	* TODO: Ad5816 has support to get status for over/under
	* voltage conditions but currently this feature is not
	* required.
	*/
}
/**
 * Below are device specific functions.
 */

static int ad5816_position_rd(struct ad5816_info *info, unsigned *position)
{

	u16 pos = 0;
	u8 t1 = 0;
	int err = 0;

	err = ad5816_i2c_rd8(info, 0, VCM_CODE_MSB, &t1);
	pos = t1 & 0x03;
	err  = ad5816_i2c_rd8(info, 0, VCM_CODE_LSB, &t1);
	pos = (pos << 8) | t1;
	if(pos)
		*position = pos - info->config.pos_low;
	else
		*position = info->config.pos_low;

	return 0;
}

static int ad5816_position_wr(struct ad5816_info *info, unsigned position)
{
	position = position + info->config.pos_low;

	if(position > info->config.pos_high)
		position = info->config.pos_high;

	u16 data = position & 0x03ff;

	return ad5816_i2c_wr16(info, VCM_CODE_MSB, data);
}

static int ad5816_param_rd(struct ad5816_info *info, unsigned long arg)
{
	struct nvc_param params;
	const void *data_ptr;
	u32 data_size = 0;
	u32 position;
	int err;
	if (copy_from_user(&params,
		(const void __user *)arg,
		sizeof(struct nvc_param))) {
		dev_err(&info->i2c_client->dev, "%s %d copy_from_user err\n", __func__, __LINE__);
		return -EFAULT;
	}
	if (info->s_mode == NVC_SYNC_SLAVE)
		info = info->s_info;
	switch (params.param) {
		case NVC_PARAM_LOCUS:
			ad5816_pm_dev_wr(info, NVC_PWR_COMM);
			err = ad5816_position_rd(info, &position);
			if (err && !(info->pdata->cfg & NVC_CFG_NOERR))
				return err;
			data_ptr = &position;
			data_size = sizeof(position);
			ad5816_pm_dev_wr(info, NVC_PWR_STDBY);
			dev_dbg(&info->i2c_client->dev, "%s LOCUS: %d\n",
					__func__, position);
			break;
		case NVC_PARAM_FOCAL_LEN:
			info->nvc.focal_length = AD5816_FOCAL_LENGTH;
			data_ptr = &info->nvc.focal_length;
			data_size = sizeof(info->nvc.focal_length);
			break;
		case NVC_PARAM_MAX_APERTURE:
			data_ptr = &info->nvc.max_aperature;
			data_size = sizeof(info->nvc.max_aperature);
			dev_dbg(&info->i2c_client->dev, "%s MAX_APERTURE: %x\n",
					__func__, info->nvc.max_aperature);
			break;
		case NVC_PARAM_FNUMBER:
			data_ptr = &info->nvc.fnumber;
			data_size = sizeof(info->nvc.fnumber);
			dev_dbg(&info->i2c_client->dev, "%s FNUMBER: %u\n",
					__func__, info->nvc.fnumber);
			break;
		case NVC_PARAM_CAPS:
			data_ptr = &info->cap;
			/* there are different sizes depending on the version */
			/* send back just what's requested or our max size */
			if (params.sizeofvalue < sizeof(info->cap))
				data_size = params.sizeofvalue;
			else
				data_size = sizeof(info->cap);
			dev_err(&info->i2c_client->dev, "%s CAPS\n", __func__);
			break;
		case NVC_PARAM_STS:
			/*data_ptr = &info->sts;
			data_size = sizeof(info->sts);*/
			dev_dbg(&info->i2c_client->dev, "%s \n", __func__);
			break;
		case NVC_PARAM_STEREO:
			data_ptr = &info->s_mode;
			data_size = sizeof(info->s_mode);
			dev_err(&info->i2c_client->dev, "%s STEREO: %d\n", __func__, info->s_mode);
			break;
		default:
			dev_err(&info->i2c_client->dev, "%s unsupported parameter: %d\n",
					__func__, params.param);
			return -EINVAL;
	}
	if (params.sizeofvalue < data_size) {
		dev_err(&info->i2c_client->dev,
			"%s data size mismatch %d != %d Param: %d\n",
			__func__, params.sizeofvalue, data_size, params.param);
		return -EINVAL;
	}
	if (copy_to_user((void __user *)params.p_value, data_ptr, data_size)) {
		dev_err(&info->i2c_client->dev, "%s copy_to_user err line %d\n",
				__func__, __LINE__);
		return -EFAULT;
	}
	return 0;
}

static int ad5816_param_wr_s(struct ad5816_info *info,
								struct nvc_param *params,
								u32 u32val)
{
	struct nvc_focus_cap cap;
	u8 u8val;
	int err = 0;
	u8val = (u8)u32val;
	switch (params->param) {
		case NVC_PARAM_LOCUS:
			dev_dbg(&info->i2c_client->dev, "%s LOCUS: %u\n", __func__, u32val);
			err = ad5816_position_wr(info, u32val);
			return err;
		case NVC_PARAM_RESET:
			err = ad5816_reset(info, u32val);
			dev_dbg(&info->i2c_client->dev, "%s RESET: %d\n", __func__, err);
			return err;
		case NVC_PARAM_SELF_TEST:
			err = 0;
			dev_dbg(&info->i2c_client->dev, "%s SELF_TEST: %d\n", __func__, err);
			return err;
		default:
			dev_dbg(&info->i2c_client->dev,
				"%s unsupported parameter: %d\n",
					__func__, params->param);
			return -EINVAL;
	}
}

static int ad5816_param_wr(struct ad5816_info *info, unsigned long arg)
{
	struct nvc_param params;
	u8 u8val;
	u32 u32val;
	int err = 0;
	if (copy_from_user(&params, (const void __user *)arg,
		sizeof(struct nvc_param))) {
		dev_err(&info->i2c_client->dev, "%s copy_from_user err line %d\n",
				__func__, __LINE__);
		return -EFAULT;
	}
	if (copy_from_user(&u32val, (const void __user *)params.p_value, sizeof(u32val))) {
		dev_err(&info->i2c_client->dev, "%s %d copy_from_user err\n", __func__, __LINE__);
		return -EFAULT;
	}
	u8val = (u8)u32val;
	/* parameters independent of sync mode */
	switch (params.param) {
		case NVC_PARAM_STEREO:
			dev_dbg(&info->i2c_client->dev, "%s STEREO: %d\n", __func__, u8val);
			if (u8val == info->s_mode)
				return 0;
			switch (u8val) {
				case NVC_SYNC_OFF:
					info->s_mode = u8val;
					ad5816_gpio_wr(info, AD5816_GPIO_I2CMUX, 0);
					if (info->s_info != NULL) {
						info->s_info->s_mode = u8val;
						ad5816_pm_wr(info->s_info, NVC_PWR_OFF);
					}
					break;
				case NVC_SYNC_MASTER:
					info->s_mode = u8val;
					ad5816_gpio_wr(info, AD5816_GPIO_I2CMUX, 0);
					if (info->s_info != NULL)
						info->s_info->s_mode = u8val;
					break;
				case NVC_SYNC_SLAVE:
					if (info->s_info != NULL) {
						/* default slave lens position */
						err = ad5816_position_wr(info->s_info,
								info->s_info->cap.focus_infinity);
						if (!err) {
							info->s_mode = u8val;
							info->s_info->s_mode = u8val;
							ad5816_gpio_wr(info,
							AD5816_GPIO_I2CMUX, 0);
						}
						else {
							if (info->s_mode != NVC_SYNC_STEREO)
								ad5816_pm_wr(info->s_info,
								NVC_PWR_OFF);
								err = -EIO;
						}
					} else {
						err = -EINVAL;
					}
					break;
				case NVC_SYNC_STEREO:
					if (info->s_info != NULL) {
						/* sync power */
						info->s_info->pwr_api = info->pwr_api;
						/* move slave lens to master position */
						err = ad5816_position_wr(info->s_info, info->pos);
						if (!err) {
							info->s_mode = u8val;
							info->s_info->s_mode = u8val;
							ad5816_gpio_wr(info, AD5816_GPIO_I2CMUX, 1);
						}
						else {
							if (info->s_mode != NVC_SYNC_SLAVE)
							ad5816_pm_wr(info->s_info, NVC_PWR_OFF);
							err = -EIO;
						}
					} else {
						err = -EINVAL;
					}
					break;
				default:
					err = -EINVAL;
			}
			if (info->pdata->cfg & NVC_CFG_NOERR)
				return 0;
			return err;
		default:
			/* parameters dependent on sync mode */
			switch (info->s_mode) {
				case NVC_SYNC_OFF:
				case NVC_SYNC_MASTER:
					return ad5816_param_wr_s(info, &params, u32val);
				case NVC_SYNC_SLAVE:
					return ad5816_param_wr_s(info->s_info, &params, u32val);
				case NVC_SYNC_STEREO:
					err = ad5816_param_wr_s(info, &params, u32val);
					if (!(info->pdata->cfg & NVC_CFG_SYNC_I2C_MUX))
						err |= ad5816_param_wr_s(info->s_info,
								&params,
								u32val);
					return err;
				default:
					dev_err(&info->i2c_client->dev, "%s %d internal err\n",
							__func__, __LINE__);
					return -EINVAL;
			}
		}
}

static long ad5816_ioctl(struct file *file,
							unsigned int cmd,
							unsigned long arg)
{
	struct ad5816_info *info = file->private_data;
	int pwr;
	int err = 0;
	switch (cmd) {
		case NVC_IOCTL_PARAM_WR:
			err = ad5816_param_wr(info, arg);
			return err;
		case NVC_IOCTL_PARAM_RD:
			err = ad5816_param_rd(info, arg);
			return err;
		case NVC_IOCTL_PWR_WR:
			/* This is a Guaranteed Level of Service (GLOS) call */
			pwr = (int)arg * 2;
			dev_dbg(&info->i2c_client->dev, "%s PWR_WR: %d\n",
					__func__, pwr);
			err = ad5816_pm_api_wr(info, pwr);
			return err;
		case NVC_IOCTL_PWR_RD:
			if (info->s_mode == NVC_SYNC_SLAVE)
				pwr = info->s_info->pwr_api / 2;
			else
				pwr = info->pwr_api / 2;
			dev_dbg(&info->i2c_client->dev, "%s PWR_RD: %d\n",
					__func__, pwr);
			if (copy_to_user((void __user *)arg, (const void *)&pwr, sizeof(pwr))) {
				dev_err(&info->i2c_client->dev, "%s copy_to_user err line %d\n",
						__func__, __LINE__);
				return -EFAULT;
			}
			return 0;
		default:
			dev_dbg(&info->i2c_client->dev, "%s unsupported ioctl: %x\n", __func__, cmd);
	}
	return -EINVAL;
}


static void ad5816_sdata_init(struct ad5816_info *info)
{
	/* set defaults */
	memcpy(&info->config, &ad5816_default_info, sizeof(info->config));
	memcpy(&info->nvc, &ad5816_default_nvc, sizeof(info->nvc));
	memcpy(&info->cap, &ad5816_default_cap, sizeof(info->cap));

	info->config.settle_time = AD5816_SETTLETIME;
	info->config.focal_length = AD5816_FOCAL_LENGTH;
	info->config.fnumber = AD5816_FNUMBER;
	info->config.pos_low = AD5816_POS_LOW_DEFAULT;
	info->config.pos_high = AD5816_POS_HIGH_DEFAULT;

	/* set to proper value */
	info->cap.actuator_range = info->config.pos_high - info->config.pos_low;

	/* set overrides if any */
	if (info->pdata->nvc) {
		if (info->pdata->nvc->fnumber)
			info->nvc.fnumber = info->pdata->nvc->fnumber;
		if (info->pdata->nvc->focal_length)
			info->nvc.focal_length = info->pdata->nvc->focal_length;
		if (info->pdata->nvc->max_aperature)
			info->nvc.max_aperature = info->pdata->nvc->max_aperature;
	}

	if (info->pdata->cap) {
		if (info->pdata->cap->actuator_range)
			info->cap.actuator_range = info->pdata->cap->actuator_range;
		if (info->pdata->cap->settle_time)
			info->cap.settle_time = info->pdata->cap->settle_time;
		if (info->pdata->cap->focus_macro)
			info->cap.focus_macro = info->pdata->cap->focus_macro;
		if (info->pdata->cap->focus_hyper)
			info->cap.focus_hyper = info->pdata->cap->focus_hyper;
		if (info->pdata->cap->focus_infinity)
			info->cap.focus_infinity = info->pdata->cap->focus_infinity;
	}
}

static int ad5816_sync_en(unsigned num, unsigned sync)
{
	struct ad5816_info *master = NULL;
	struct ad5816_info *slave = NULL;
	struct ad5816_info *pos = NULL;
	rcu_read_lock();
	list_for_each_entry_rcu(pos, &ad5816_info_list, list) {
		if (pos->pdata->num == num) {
			master = pos;
			break;
		}
	}
	pos = NULL;
	list_for_each_entry_rcu(pos, &ad5816_info_list, list) {
		if (pos->pdata->num == sync) {
			slave = pos;
			break;
		}
	}
	rcu_read_unlock();
	if (master != NULL)
		master->s_info = NULL;
	if (slave != NULL)
		slave->s_info = NULL;
	if (!sync)
		return 0; /* no err if sync disabled */
	if (num == sync)
		return -EINVAL; /* err if sync instance is itself */
	if ((master != NULL) && (slave != NULL)) {
		master->s_info = slave;
		slave->s_info = master;
	}
	return 0;
}

static int ad5816_sync_dis(struct ad5816_info *info)
{
	if (info->s_info != NULL) {
		info->s_info->s_mode = 0;
		info->s_info->s_info = NULL;
		info->s_mode = 0;
		info->s_info = NULL;
		return 0;
	}
	return -EINVAL;
}

static int ad5816_open(struct inode *inode, struct file *file)
{
	struct ad5816_info *info = NULL;
	struct ad5816_info *pos = NULL;
	int err;
	rcu_read_lock();
	list_for_each_entry_rcu(pos, &ad5816_info_list, list) {
		if (pos->miscdev.minor == iminor(inode)) {
			info = pos;
			break;
		}
	}
	rcu_read_unlock();
	if (!info)
		return -ENODEV;
	err = ad5816_sync_en(info->pdata->num, info->pdata->sync);
	if (err == -EINVAL)
		dev_err(&info->i2c_client->dev, "%s err: invalid num (%u) and sync (%u) instance\n",
		__func__, info->pdata->num, info->pdata->sync);
	if (atomic_xchg(&info->in_use, 1))
		return -EBUSY;
	if (info->s_info != NULL) {
		if (atomic_xchg(&info->s_info->in_use, 1))
			return -EBUSY;
	}
	file->private_data = info;
	ad5816_pm_dev_wr(info, NVC_PWR_ON);
	/* set ARC Mode to ensure faster focus */
	ad5816_set_arc_mode(info);
	dev_dbg(&info->i2c_client->dev, "%s\n", __func__);

	return 0;
}

static int ad5816_release(struct inode *inode, struct file *file)
{
	struct ad5816_info *info = file->private_data;
	dev_dbg(&info->i2c_client->dev, "%s\n", __func__);
	ad5816_pm_wr_s(info, NVC_PWR_OFF);
	file->private_data = NULL;
	WARN_ON(!atomic_xchg(&info->in_use, 0));
	if (info->s_info != NULL)
		WARN_ON(!atomic_xchg(&info->s_info->in_use, 0));
	ad5816_sync_dis(info);
	return 0;
}

static const struct file_operations ad5816_fileops = {
	.owner = THIS_MODULE,
	.open = ad5816_open,
	.unlocked_ioctl = ad5816_ioctl,
	.release = ad5816_release,
};

static void ad5816_del(struct ad5816_info *info)
{
	ad5816_pm_exit(info);
	if ((info->s_mode == NVC_SYNC_SLAVE) ||
		(info->s_mode == NVC_SYNC_STEREO))
		ad5816_pm_exit(info->s_info);

	ad5816_sync_dis(info);
	spin_lock(&ad5816_spinlock);
	list_del_rcu(&info->list);
	spin_unlock(&ad5816_spinlock);
	synchronize_rcu();
}

static int ad5816_remove(struct i2c_client *client)
{
	struct ad5816_info *info = i2c_get_clientdata(client);
	dev_dbg(&info->i2c_client->dev, "%s\n", __func__);
	misc_deregister(&info->miscdev);
	ad5816_del(info);
	return 0;
}

static int ad5816_probe(
		struct i2c_client *client,
		const struct i2c_device_id *id)
{
	pr_info("ad5816: probing focuser.\n");
	struct ad5816_info *info;
	char dname[16];
	int err;
	dev_dbg(&client->dev, "%s\n", __func__);
	info = devm_kzalloc(&client->dev, sizeof(*info), GFP_KERNEL);
	if (info == NULL) {
		dev_err(&client->dev, "%s: kzalloc error\n", __func__);
		return -ENOMEM;
	}
	info->i2c_client = client;
	if (client->dev.platform_data) {
		info->pdata = client->dev.platform_data;
	} else {
		info->pdata = &ad5816_default_pdata;
		dev_dbg(&client->dev,"%s No platform data.  Using defaults.\n", __func__);
	}

	i2c_set_clientdata(client, info);
	INIT_LIST_HEAD(&info->list);
	spin_lock(&ad5816_spinlock);
	list_add_rcu(&info->list, &ad5816_info_list);
	spin_unlock(&ad5816_spinlock);
	ad5816_pm_init(info);
	ad5816_sdata_init(info);

	err = ad5816_dev_id(info);
	if (err < 0) {
		dev_err(&client->dev, "%s device not found\n", __func__);
		ad5816_pm_wr(info, NVC_PWR_OFF);
		if (info->pdata->cfg & NVC_CFG_NODEV) {
			ad5816_del(info);
			return -ENODEV;
		}
	} else {
		dev_dbg(&client->dev, "%s device found\n", __func__);
		if (info->pdata->cfg & NVC_CFG_BOOT_INIT) {
			/* initial move causes full initialization */
			ad5816_pm_dev_wr(info, NVC_PWR_ON);
			ad5816_position_wr(info, info->cap.focus_infinity);
			ad5816_pm_dev_wr(info, NVC_PWR_OFF);
		}
	}

	if (info->pdata->dev_name != 0)
		strcpy(dname, info->pdata->dev_name);
	else
		strcpy(dname, "ad5816");

	if (info->pdata->num)
		snprintf(dname, sizeof(dname), "%s.%u", dname, info->pdata->num);

	info->miscdev.name = dname;
	info->miscdev.fops = &ad5816_fileops;
	info->miscdev.minor = MISC_DYNAMIC_MINOR;
	if (misc_register(&info->miscdev)) {
		dev_err(&client->dev, "%s unable to register misc device %s\n",
			__func__, dname);
		ad5816_del(info);
		return -ENODEV;
	}

	return 0;
}


static const struct i2c_device_id ad5816_id[] = {
	{ "ad5816", 0 },
	{ },
};


MODULE_DEVICE_TABLE(i2c, ad5816_id);

static struct i2c_driver ad5816_i2c_driver = {
	.driver = {
		.name = "ad5816",
		.owner = THIS_MODULE,
	},
	.id_table = ad5816_id,
	.probe = ad5816_probe,
	.remove = ad5816_remove,
};

static int __init ad5816_init(void)
{
	return i2c_add_driver(&ad5816_i2c_driver);
}

static void __exit ad5816_exit(void)
{
	i2c_del_driver(&ad5816_i2c_driver);
}

module_init(ad5816_init);
module_exit(ad5816_exit);
MODULE_LICENSE("GPL");
