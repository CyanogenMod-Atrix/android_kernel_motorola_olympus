/*
 * arch/arm/mach-tegra/tegra3_thermal.c
 *
 * Copyright (C) 2010-2012 NVIDIA Corporation.
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
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/thermal.h>
#include <mach/thermal.h>
#include <mach/edp.h>
#include <linux/slab.h>
#include <linux/suspend.h>

#include "clock.h"
#include "cpu-tegra.h"
#include "dvfs.h"

static struct tegra_thermal_data *therm;
static LIST_HEAD(tegra_therm_list);
static DEFINE_MUTEX(tegra_therm_mutex);

static struct balanced_throttle *throttle_list;
static int throttle_list_size;

#ifdef CONFIG_TEGRA_EDP_LIMITS
static long edp_thermal_zone_val;
#endif

#ifdef CONFIG_TEGRA_SKIN_THROTTLE
static int skin_devs_bitmap;
static struct therm_est_subdevice *skin_devs[THERMAL_DEVICE_MAX];
static int skin_devs_count;
#endif
static bool tegra_thermal_suspend;

#ifdef CONFIG_DEBUG_FS
static struct dentry *thermal_debugfs_root;
#endif

static inline long dev2tj(struct tegra_thermal_device *dev,
				long dev_temp)
{
	return dev_temp + dev->offset;
}

static inline long tj2dev(struct tegra_thermal_device *dev,
				long tj_temp)
{
	return tj_temp - dev->offset;
}

static int tegra_thermal_get_temp_unlocked(long *tj_temp, bool offsetted)
{
	struct tegra_thermal_device *dev = NULL;
	int ret = 0;

#if defined(CONFIG_TEGRA_EDP_LIMITS) || defined(CONFIG_TEGRA_THERMAL_THROTTLE)
	list_for_each_entry(dev, &tegra_therm_list, node)
		if (dev->id == therm->throttle_edp_device_id)
			break;
#endif

	if (dev) {
		dev->get_temp(dev->data, tj_temp);
		if (offsetted)
			*tj_temp = dev2tj(dev, *tj_temp);
	} else {
		ret = -1;
	}

	return ret;
}

#ifdef CONFIG_TEGRA_THERMAL_THROTTLE

static int tegra_thermal_zone_bind(struct thermal_zone_device *thz,
				struct thermal_cooling_device *cdevice) {

	struct balanced_throttle *bthrot = cdevice->devdata;
	struct tegra_thermal_device *device = thz->devdata;

	if ((bthrot->id == BALANCED_THROTTLE_ID_TJ) &&
		(device->id == therm->throttle_edp_device_id))
		return thermal_zone_bind_cooling_device(thz, 0, cdevice);

#ifdef CONFIG_TEGRA_SKIN_THROTTLE
	if ((bthrot->id == BALANCED_THROTTLE_ID_SKIN) &&
		(device->id == therm->skin_device_id))
		return thermal_zone_bind_cooling_device(thz, 0, cdevice);
#endif

	return 0;
}

static int tegra_thermal_zone_unbind(struct thermal_zone_device *thz,
				struct thermal_cooling_device *cdevice) {
	struct balanced_throttle *bthrot = cdevice->devdata;
	struct tegra_thermal_device *device = thz->devdata;

	if ((bthrot->id == BALANCED_THROTTLE_ID_TJ) &&
		(device->id == therm->throttle_edp_device_id))
		return thermal_zone_unbind_cooling_device(thz, 0, cdevice);

#ifdef CONFIG_TEGRA_SKIN_THROTTLE
	if ((bthrot->id == BALANCED_THROTTLE_ID_SKIN) &&
		(device->id == therm->skin_device_id))
		return thermal_zone_unbind_cooling_device(thz, 0, cdevice);
#endif

	return 0;
}

static int tegra_thermal_zone_get_temp(struct thermal_zone_device *thz,
					unsigned long *temp)
{
	struct tegra_thermal_device *device = thz->devdata;

	if (!tegra_thermal_suspend)
		device->get_temp(device->data, temp);

	return 0;
}

static int tegra_thermal_zone_get_trip_type(
			struct thermal_zone_device *thermal,
			int trip,
			enum thermal_trip_type *type) {
	if (trip != 0)
		return -EINVAL;

	*type = THERMAL_TRIP_PASSIVE;

	return 0;
}

static int tegra_thermal_zone_get_trip_temp(struct thermal_zone_device *thz,
					int trip,
					unsigned long *temp) {
	struct tegra_thermal_device *device = thz->devdata;

	if (trip != 0)
		return -EINVAL;

	if (device->id == therm->throttle_edp_device_id)
		*temp = therm->temp_throttle;
#ifdef CONFIG_TEGRA_SKIN_THROTTLE
	else if (device->id == therm->skin_device_id)
		*temp = therm->temp_throttle_skin;
#endif
	else
		return -EINVAL;

	return 0;
}

static struct thermal_zone_device_ops tegra_thermal_zone_ops = {
	.bind = tegra_thermal_zone_bind,
	.unbind = tegra_thermal_zone_unbind,
	.get_temp = tegra_thermal_zone_get_temp,
	.get_trip_type = tegra_thermal_zone_get_trip_type,
	.get_trip_temp = tegra_thermal_zone_get_trip_temp,
};
#endif

static int tegra_thermal_pm_notify(struct notifier_block *nb,
				unsigned long event, void *data)
{
	switch (event) {
	case PM_SUSPEND_PREPARE:
		tegra_thermal_suspend = true;
		break;
	case PM_POST_SUSPEND:
		tegra_thermal_suspend = false;
		break;
	}

	return NOTIFY_OK;
};

static struct notifier_block tegra_thermal_nb = {
	.notifier_call = tegra_thermal_pm_notify,
};

static void tegra_thermal_alert_unlocked(void *data)
{
	struct tegra_thermal_device *device = data;
	long temp_tj;
	long lo_limit_throttle_tj, hi_limit_throttle_tj;
	long lo_limit_edp_tj = 0, hi_limit_edp_tj = 0;
	long temp_low_dev, temp_low_tj;
	int lo_limit_tj = 0, hi_limit_tj = 0;
#ifdef CONFIG_TEGRA_EDP_LIMITS
	const struct tegra_edp_limits *z;
	int zones_sz;
	int i;
#endif

#ifdef CONFIG_TEGRA_THERMAL_THROTTLE
	if (device->thz) {
		if ((!device->thz->passive) && (!tegra_thermal_suspend))
			thermal_zone_device_update(device->thz);
	}
#endif

	/* Convert all temps to tj and then do all work/logic in terms of
	   tj in order to avoid confusion */
	if (tegra_thermal_get_temp_unlocked(&temp_tj, true))
		return;
	device->get_temp_low(device, &temp_low_dev);
	temp_low_tj = dev2tj(device, temp_low_dev);

	lo_limit_throttle_tj = temp_low_tj;
	hi_limit_throttle_tj = dev2tj(device, therm->temp_shutdown);

#ifdef CONFIG_TEGRA_THERMAL_THROTTLE
	hi_limit_throttle_tj = dev2tj(device, therm->temp_throttle);

	if (temp_tj > dev2tj(device, therm->temp_throttle)) {
		lo_limit_throttle_tj = dev2tj(device, therm->temp_throttle);
		hi_limit_throttle_tj = dev2tj(device, therm->temp_shutdown);
	}
#endif

#ifdef CONFIG_TEGRA_EDP_LIMITS
	tegra_get_cpu_edp_limits(&z, &zones_sz);

/* edp table based off of tdiode measurements */
#define EDP_TEMP_TJ(_index) (z[_index].temperature * 1000 + therm->edp_offset)

	if (temp_tj < EDP_TEMP_TJ(0)) {
		lo_limit_edp_tj = temp_low_tj;
		hi_limit_edp_tj = EDP_TEMP_TJ(0);
	} else if (temp_tj >= EDP_TEMP_TJ(zones_sz-1)) {
		lo_limit_edp_tj = EDP_TEMP_TJ(zones_sz-1) -
					therm->hysteresis_edp;
		hi_limit_edp_tj = dev2tj(device, therm->temp_shutdown);
	} else {
		for (i = 0; (i + 1) < zones_sz; i++) {
			if ((temp_tj >= EDP_TEMP_TJ(i)) &&
				(temp_tj < EDP_TEMP_TJ(i+1))) {
				lo_limit_edp_tj = EDP_TEMP_TJ(i) -
							therm->hysteresis_edp;
				hi_limit_edp_tj = EDP_TEMP_TJ(i+1);
				break;
			}
		}
	}
#undef EDP_TEMP_TJ
#else
	lo_limit_edp_tj = temp_low_tj;
	hi_limit_edp_tj = dev2tj(device, therm->temp_shutdown);
#endif

	/* Get smallest window size */
	lo_limit_tj = max(lo_limit_throttle_tj, lo_limit_edp_tj);
	hi_limit_tj = min(hi_limit_throttle_tj, hi_limit_edp_tj);

	device->set_limits(device->data,
				tj2dev(device, lo_limit_tj),
				tj2dev(device, hi_limit_tj));

#ifdef CONFIG_TEGRA_EDP_LIMITS
	/* inform edp governor */
	if (edp_thermal_zone_val != temp_tj) {
		long temp_edp = (dev2tj(device, temp_tj) - therm->edp_offset) / 1000;
		tegra_edp_update_thermal_zone(temp_edp);
		edp_thermal_zone_val = temp_edp;
	}
#endif
}

#ifdef CONFIG_TEGRA_THERMAL_THROTTLE
/* Make sure this function remains stateless */
static void tegra_thermal_alert(void *data)
{
	mutex_lock(&tegra_therm_mutex);
	tegra_thermal_alert_unlocked(data);
	mutex_unlock(&tegra_therm_mutex);
}
#endif

#ifdef CONFIG_TEGRA_SKIN_THROTTLE
static void tegra_skin_thermal_alert(void *data)
{
	struct tegra_thermal_device *dev = data;

	if (!dev->thz->passive)
		thermal_zone_device_update(dev->thz);
}

static int tegra_skin_device_register(struct tegra_thermal_device *device)
{
	int i;
	struct therm_est_subdevice *skin_dev =
		kzalloc(sizeof(struct therm_est_subdevice), GFP_KERNEL);

	for (i = 0; i < therm->skin_devs_size; i++) {
		if (therm->skin_devs[i].id == device->id) {
			memcpy(skin_dev->coeffs,
				therm->skin_devs[i].coeffs,
				sizeof(skin_devs[i]->coeffs));
			break;
		}
	}

	skin_dev->dev_data = device->data;
	skin_dev->get_temp = device->get_temp;

	skin_devs[skin_devs_count++] = skin_dev;

	/* Create skin thermal device */
	if (skin_devs_count == therm->skin_devs_size) {
		struct tegra_thermal_device *thermal_skin_device;
		struct therm_estimator *skin_estimator;

		skin_estimator = therm_est_register(
					skin_devs,
					skin_devs_count,
					therm->skin_temp_offset,
					therm->skin_period);
		thermal_skin_device = kzalloc(sizeof(struct tegra_thermal_device),
							GFP_KERNEL);
		thermal_skin_device->name = "skin_pred";
		thermal_skin_device->id = THERMAL_DEVICE_ID_SKIN;
		thermal_skin_device->data = skin_estimator;
		thermal_skin_device->get_temp =
			(int (*)(void *, long *)) therm_est_get_temp;
		thermal_skin_device->set_limits =
			(int (*)(void *, long, long)) therm_est_set_limits;
		thermal_skin_device->set_alert =
			(int (*)(void *, void (*)(void *), void *))
				therm_est_set_alert;

		tegra_thermal_device_register(thermal_skin_device);
	}

	return 0;
}
#endif

int tegra_thermal_device_register(struct tegra_thermal_device *device)
{
	struct tegra_thermal_device *dev;
#ifdef CONFIG_TEGRA_THERMAL_THROTTLE
	struct thermal_zone_device *thz;
	int t1 = 0, t2 = 0, pdelay = 0;
	bool create_thz = false;
#endif

	mutex_lock(&tegra_therm_mutex);
	list_for_each_entry(dev, &tegra_therm_list, node) {
		if (dev->id == device->id) {
			mutex_unlock(&tegra_therm_mutex);
			return -EINVAL;
		}
	}

#ifdef CONFIG_TEGRA_THERMAL_THROTTLE
	if (device->id == therm->throttle_edp_device_id) {
		t1 = therm->tc1;
		t2 = therm->tc2;
		pdelay = therm->passive_delay;
		create_thz = true;
	}
#endif
#ifdef CONFIG_TEGRA_SKIN_THROTTLE
	if (device->id == therm->skin_device_id) {
		t1 = 0;
		t2 = 1;
		pdelay = 5000;
		create_thz = true;
	}
#endif

#ifdef CONFIG_TEGRA_THERMAL_THROTTLE
	if (create_thz) {
		thz = thermal_zone_device_register(
						device->name,
						1, /* trips */
						device,
						&tegra_thermal_zone_ops,
						t1, /* dT/dt */
						t2, /* throttle */
						pdelay,
						0); /* polling delay */
		if (IS_ERR_OR_NULL(thz))
			return -ENODEV;

		device->thz = thz;
	}
#endif

	list_add(&device->node, &tegra_therm_list);
	mutex_unlock(&tegra_therm_mutex);

	if (device->id == therm->shutdown_device_id) {
		device->set_shutdown_temp(device->data, therm->temp_shutdown);
	}

#ifdef CONFIG_TEGRA_SKIN_THROTTLE
	if (device->id == therm->skin_device_id) {
		if (create_thz)
			device->set_alert(device->data,
				tegra_skin_thermal_alert,
				device);
		device->set_limits(device->data, 0, therm->temp_throttle_skin);
	}
#endif

#ifdef CONFIG_TEGRA_THERMAL_THROTTLE
	if (device->id == therm->throttle_edp_device_id) {
		device->set_alert(device->data, tegra_thermal_alert, device);

		/* initialize limits */
		tegra_thermal_alert(device);
	}
#endif

#ifdef CONFIG_TEGRA_SKIN_THROTTLE
	if ((therm->skin_device_id == THERMAL_DEVICE_ID_SKIN) &&
		device->id && skin_devs_bitmap)
		tegra_skin_device_register(device);
#endif

	register_pm_notifier(&tegra_thermal_nb);
	return 0;
}

/* This needs to be inialized later hand */
static int __init throttle_list_init(void)
{
	int i;
	for (i = 0; i < throttle_list_size; i++)
		if (balanced_throttle_register(&throttle_list[i]))
			return -ENODEV;

	return 0;
}
late_initcall(throttle_list_init);

int __init tegra_thermal_init(struct tegra_thermal_data *data,
				struct balanced_throttle *tlist,
				int tlist_size)
{
	therm = data;
#ifdef CONFIG_DEBUG_FS
	thermal_debugfs_root = debugfs_create_dir("tegra_thermal", 0);
#endif

#ifdef CONFIG_TEGRA_SKIN_THROTTLE
	{
		int i;
		for (i = 0; i < therm->skin_devs_size; i++)
			skin_devs_bitmap |= therm->skin_devs[i].id;
	}
#endif

	throttle_list = tlist;
	throttle_list_size = tlist_size;

	return 0;
}

int tegra_thermal_exit(void)
{
#ifdef CONFIG_TEGRA_THERMAL_THROTTLE
	struct tegra_thermal_device *dev;
	mutex_lock(&tegra_therm_mutex);
	list_for_each_entry(dev, &tegra_therm_list, node) {
		thermal_zone_device_unregister(dev->thz);
	}
	mutex_unlock(&tegra_therm_mutex);
#endif

	return 0;
}

#ifdef CONFIG_DEBUG_FS
static int tegra_thermal_temp_tj_get(void *data, u64 *val)
{
	long temp_tj;

	mutex_lock(&tegra_therm_mutex);
	if (tegra_thermal_get_temp_unlocked(&temp_tj, false))
		temp_tj = -1;
	mutex_unlock(&tegra_therm_mutex);

	*val = (u64)temp_tj;

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(temp_tj_fops,
			tegra_thermal_temp_tj_get,
			NULL,
			"%llu\n");

static int __init temp_tj_debug_init(void)
{
	debugfs_create_file("temp_tj", 0644, thermal_debugfs_root,
		NULL, &temp_tj_fops);
	return 0;
}
late_initcall(temp_tj_debug_init);


#define TEGRA_THERM_DEBUGFS(_name, _device_id, throttle, shutdown) \
	static int tegra_thermal_##_name##_set(void *data, u64 val) \
	{ \
		struct tegra_thermal_device *dev; \
		mutex_lock(&tegra_therm_mutex); \
		therm->_name = val; \
		list_for_each_entry(dev, &tegra_therm_list, node) \
			if (dev->id == therm->_device_id) \
				break; \
		if (dev) { \
			if (throttle) \
				tegra_thermal_alert_unlocked(dev); \
			if (shutdown) \
				dev->set_shutdown_temp(dev->data, \
							therm->temp_shutdown); \
		} \
		mutex_unlock(&tegra_therm_mutex); \
		return 0; \
	} \
	static int tegra_thermal_##_name##_get(void *data, u64 *val) \
	{ \
		*val = (u64)therm->_name; \
		return 0; \
	} \
	DEFINE_SIMPLE_ATTRIBUTE(_name##_fops, \
				tegra_thermal_##_name##_get, \
				tegra_thermal_##_name##_set, \
				"%llu\n"); \
	static int __init _name##_debug_init(void) \
	{ \
		debugfs_create_file(#_name, 0644, thermal_debugfs_root, \
			NULL, &_name##_fops); \
		return 0; \
	} \
	late_initcall(_name##_debug_init);


TEGRA_THERM_DEBUGFS(temp_shutdown, shutdown_device_id, false, true);
#ifdef CONFIG_TEGRA_THERMAL_THROTTLE
TEGRA_THERM_DEBUGFS(temp_throttle, throttle_edp_device_id, true, false);
#endif
#ifdef CONFIG_TEGRA_SKIN_THROTTLE
TEGRA_THERM_DEBUGFS(temp_throttle_skin, skin_device_id, false, false);
#endif

#ifdef CONFIG_TEGRA_THERMAL_THROTTLE
#define THERM_DEBUGFS(_name) \
	static int tegra_thermal_##_name##_set(void *data, u64 val) \
	{ \
		struct tegra_thermal_device *dev; \
		mutex_lock(&tegra_therm_mutex); \
		list_for_each_entry(dev, &tegra_therm_list, node) \
			if (dev->id == therm->throttle_edp_device_id) \
				break; \
		if (dev) \
			dev->thz->_name = val; \
		mutex_unlock(&tegra_therm_mutex); \
		return 0; \
	} \
	static int tegra_thermal_##_name##_get(void *data, u64 *val) \
	{ \
		struct tegra_thermal_device *dev; \
		mutex_lock(&tegra_therm_mutex); \
		list_for_each_entry(dev, &tegra_therm_list, node) \
			if (dev->id == therm->throttle_edp_device_id) \
				break; \
		if (dev) \
			*val = (u64)dev->thz->_name; \
		mutex_unlock(&tegra_therm_mutex); \
		return 0; \
	} \
	DEFINE_SIMPLE_ATTRIBUTE(_name##_fops, \
			tegra_thermal_##_name##_get, \
			tegra_thermal_##_name##_set, \
			"%llu\n"); \
	static int __init _name##_debug_init(void) \
	{ \
		debugfs_create_file(#_name, 0644, thermal_debugfs_root, \
			NULL, &_name##_fops); \
		return 0; \
	} \
	late_initcall(_name##_debug_init);


THERM_DEBUGFS(tc1);
THERM_DEBUGFS(tc2);
THERM_DEBUGFS(passive_delay);
#endif
#endif
