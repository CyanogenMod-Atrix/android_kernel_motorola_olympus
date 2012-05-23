/*
 * arch/arm/mach-tegra/tegra3_throttle.c
 *
 * Copyright (c) 2011, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
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

#include <linux/kernel.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/thermal.h>

#include "clock.h"
#include "cpu-tegra.h"
#include "dvfs.h"

/* tegra throttling require frequencies in the table to be in ascending order */
static struct cpufreq_frequency_table *cpu_freq_table;

static struct {
	unsigned int cpu_freq;
	int core_cap_level;
} throttle_table[] = {
	{      0, 1000 },	/* placeholder for cpu floor rate */
	{ 640000, 1000 },
	{ 640000, 1000 },
	{ 640000, 1000 },
	{ 640000, 1000 },
	{ 640000, 1000 },
	{ 760000, 1000 },
	{ 760000, 1050 },
	{1000000, 1050 },
	{1000000, 1100 },
};

static int is_throttling;
static int throttle_index;
static struct thermal_cooling_device *cdev;

static unsigned int clip_to_table(unsigned int cpu_freq)
{
	int i;

	for (i = 0; cpu_freq_table[i].frequency != CPUFREQ_TABLE_END; i++) {
		if (cpu_freq_table[i].frequency > cpu_freq)
			break;
	}
	i = (i == 0) ? 0 : i-1;
	return cpu_freq_table[i].frequency;
}

unsigned int tegra_throttle_governor_speed(unsigned int requested_speed)
{
	return is_throttling ?
		min(requested_speed, throttle_table[throttle_index].cpu_freq) :
		requested_speed;
}

bool tegra_is_throttling(void)
{
	return is_throttling;
}

static int
tegra_throttle_get_max_state(struct thermal_cooling_device *cdev,
				unsigned long *max_state)
{
	*max_state = ARRAY_SIZE(throttle_table);
	return 0;
}

static int
tegra_throttle_get_cur_state(struct thermal_cooling_device *cdev,
				unsigned long *cur_state)
{
	*cur_state = is_throttling ?
			(ARRAY_SIZE(throttle_table) - throttle_index) :
			0;

	return 0;
}

static int
tegra_throttle_set_cur_state(struct thermal_cooling_device *cdev,
				unsigned long cur_state)
{
	int core_level;

	if (cur_state == 0) {
		/* restore speed requested by governor */
		if (is_throttling) {
			tegra_dvfs_core_cap_enable(false);
			is_throttling = false;
		}

		tegra_cpu_set_speed_cap(NULL);
	} else {
		if (!is_throttling) {
			tegra_dvfs_core_cap_enable(true);
			is_throttling = true;
		}

		throttle_index = ARRAY_SIZE(throttle_table) - cur_state;
		core_level = throttle_table[throttle_index].core_cap_level;
		tegra_dvfs_core_cap_level_set(core_level);

		tegra_cpu_set_speed_cap(NULL);
	}


	return 0;
}

struct thermal_cooling_device_ops tegra_throttle_cooling_ops = {
	.get_max_state = tegra_throttle_get_max_state,
	.get_cur_state = tegra_throttle_get_cur_state,
	.set_cur_state = tegra_throttle_set_cur_state,
};

int __init tegra_throttle_init(struct mutex *cpu_lock)
{
	int i;
	struct tegra_cpufreq_table_data *table_data =
		tegra_cpufreq_table_get();
	if (IS_ERR_OR_NULL(table_data))
		return -EINVAL;

	cpu_freq_table = table_data->freq_table;
	throttle_table[0].cpu_freq =
		cpu_freq_table[table_data->throttle_lowest_index].frequency;

	for (i = 0; i < ARRAY_SIZE(throttle_table); i++) {
		unsigned int cpu_freq = throttle_table[i].cpu_freq;
		throttle_table[i].cpu_freq = clip_to_table(cpu_freq);
	}

	cdev = thermal_cooling_device_register("Throttle", NULL,
						&tegra_throttle_cooling_ops);

	if (IS_ERR(cdev)) {
		cdev = NULL;
		return -ENODEV;
	}

	return 0;
}

void tegra_throttle_exit(void)
{
	if (cdev) {
		thermal_cooling_device_unregister(cdev);
		cdev = NULL;
	}
}

#ifdef CONFIG_DEBUG_FS
static int table_show(struct seq_file *s, void *data)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(throttle_table); i++)
		seq_printf(s, "[%d] = %7u %4d\n",
			i, throttle_table[i].cpu_freq,
			throttle_table[i].core_cap_level);
	return 0;
}

static int table_open(struct inode *inode, struct file *file)
{
	return single_open(file, table_show, inode->i_private);
}

static ssize_t table_write(struct file *file,
	const char __user *userbuf, size_t count, loff_t *ppos)
{
	char buf[80];
	int table_idx;
	unsigned int cpu_freq;
	int core_cap_level;

	if (sizeof(buf) <= count)
		return -EINVAL;

	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;

	/* terminate buffer and trim - white spaces may be appended
	 *  at the end when invoked from shell command line */
	buf[count] = '\0';
	strim(buf);

	if (sscanf(buf, "[%d] = %u %d",
		   &table_idx, &cpu_freq, &core_cap_level) != 3)
		return -1;

	if ((table_idx < 0) || (table_idx >= ARRAY_SIZE(throttle_table)))
		return -EINVAL;

	/* round new settings before updating table */
	throttle_table[table_idx].cpu_freq = clip_to_table(cpu_freq);
	throttle_table[table_idx].core_cap_level = (core_cap_level / 50) * 50;

	return count;
}

static const struct file_operations table_fops = {
	.open		= table_open,
	.read		= seq_read,
	.write		= table_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};


int __init tegra_throttle_debug_init(struct dentry *cpu_tegra_debugfs_root)
{
	if (!debugfs_create_file("throttle_table", 0644, cpu_tegra_debugfs_root,
				 NULL, &table_fops))
		return -ENOMEM;

	return 0;
}
#endif /* CONFIG_DEBUG_FS */

