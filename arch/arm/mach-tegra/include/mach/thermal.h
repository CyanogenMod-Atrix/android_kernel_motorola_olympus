/*
 * arch/arm/mach-tegra/thermal.h
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

#ifndef __MACH_THERMAL_H
#define __MACH_THERMAL_H

#include <linux/therm_est.h>

enum thermal_device_id {
	THERMAL_DEVICE_ID_NULL = 0x0,
	THERMAL_DEVICE_ID_NCT_EXT = 0x1,
	THERMAL_DEVICE_ID_NCT_INT = 0x2,
	THERMAL_DEVICE_ID_TSENSOR = 0x4,
	THERMAL_DEVICE_ID_SKIN = 0x8,
};

#define THERMAL_DEVICE_MAX	(4)

enum balanced_throttle_id {
	BALANCED_THROTTLE_ID_TJ,
	BALANCED_THROTTLE_ID_SKIN,
};

struct skin_therm_est_subdevice {
	enum thermal_device_id id;
	long coeffs[HIST_LEN];
};

/* All units in millicelsius */
struct tegra_thermal_data {
	enum thermal_device_id shutdown_device_id;
	long temp_shutdown;
#if defined(CONFIG_TEGRA_EDP_LIMITS) || defined(CONFIG_TEGRA_THERMAL_THROTTLE)
	enum thermal_device_id throttle_edp_device_id;
#endif
#ifdef CONFIG_TEGRA_EDP_LIMITS
	long edp_offset;
	long hysteresis_edp;
#endif
#ifdef CONFIG_TEGRA_THERMAL_THROTTLE
	long temp_throttle;
	int tc1;
	int tc2;
	long passive_delay;
#endif
#ifdef CONFIG_TEGRA_SKIN_THROTTLE
	enum thermal_device_id skin_device_id;
	long temp_throttle_skin;
	int tc1_skin;
	int tc2_skin;
	int passive_delay_skin;

	long skin_temp_offset;
	long skin_period;
	int skin_devs_size;
	struct skin_therm_est_subdevice skin_devs[];
#endif
};

struct tegra_thermal_device {
	char *name;
	enum thermal_device_id id;
	void *data;
	long offset;
	int (*get_temp) (void *, long *);
	int (*get_temp_low)(void *, long *);
	int (*set_limits) (void *, long, long);
	int (*set_alert)(void *, void (*)(void *), void *);
	int (*set_shutdown_temp)(void *, long);
#ifdef CONFIG_TEGRA_THERMAL_THROTTLE
	struct thermal_zone_device *thz;
#endif
	struct list_head node;
};

struct throttle_table {
	unsigned int cpu_freq;
	int core_cap_level;
};

#define MAX_THROT_TABLE_SIZE	(32)

struct balanced_throttle {
	enum balanced_throttle_id id;

	int is_throttling;
	int throttle_index;
	struct thermal_cooling_device *cdev;

	struct list_head node;

	int throt_tab_size;
	struct throttle_table throt_tab[MAX_THROT_TABLE_SIZE];
};

#ifdef CONFIG_TEGRA_THERMAL_THROTTLE
int balanced_throttle_register(struct balanced_throttle *bthrot);
#else
static inline int balanced_throttle_register(struct balanced_throttle *bthrot)
{ return 0; }
#endif

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
int tegra_thermal_init(struct tegra_thermal_data *data,
				struct balanced_throttle *throttle_list,
				int throttle_list_size);
int tegra_thermal_device_register(struct tegra_thermal_device *device);
int tegra_thermal_exit(void);
#else
static inline int tegra_thermal_init(struct tegra_thermal_data *data,
					struct balanced_throttle throttle_list,
					int throttle_list_size);
{ return 0; }
static int tegra_thermal_device_register(struct tegra_thermal_device *device)
{ return 0; }
static inline int tegra_thermal_exit(void)
{ return 0; }
#endif

#endif	/* __MACH_THERMAL_H */
