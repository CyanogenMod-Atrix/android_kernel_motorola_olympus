/*
 * arch/arm/mach-tegra/cpu-tegra2.c
 *
 * CPU auto-hotplug for Tegra2 CPUs
 *
 * Copyright (c) 2012, NVIDIA Corporation.
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

#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/cpu.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/pm_qos_params.h>

#include "pm.h"
#include "cpu-tegra.h"
#include "clock.h"

//#define INITIAL_STATE		TEGRA_HP_DISABLED
#define INITIAL_STATE		TEGRA_HP_IDLE
#define DELAY_MS			0

#define CPU1_ON_PENDING_MS  4000
#define CPU1_OFF_PENDING_MS 4000

static u64 last_change_time_hi = 0;
static u64 last_change_time_lo = 0;

static struct mutex *tegra2_cpu_lock;

static struct workqueue_struct *hotplug_wq;
static struct delayed_work hotplug_work;

static unsigned long delay = 0;
static unsigned long up_delay = 0;
static unsigned long down_delay = 0;
static bool pending = false;

static unsigned long top_freq;
static unsigned long bottom_freq;

static struct clk *cpu_clk;

enum {
	TEGRA_HP_DISABLED = 0,
	TEGRA_HP_IDLE,
	TEGRA_HP_DOWN,
	TEGRA_HP_UP,
};
static int hp_state;

u64 last_change_time(void)
{
	return last_change_time_hi;
}

void tegra2_enable_autoplug(void)
{
	mutex_lock(tegra2_cpu_lock);
	hp_state = TEGRA_HP_IDLE;
	mutex_unlock(tegra2_cpu_lock);
}

void tegra2_disable_autoplug(void)
{
	mutex_lock(tegra2_cpu_lock);
	hp_state = TEGRA_HP_DISABLED;

	/* check if CPU-1 is offline before leaving from here.
	 * If it was, bring it online as it was before enabling hot-plug
	 * for Tegra2 */
	if (!cpu_online(1))
		cpu_up(1);

	mutex_unlock(tegra2_cpu_lock);
}

static void tegra2_auto_hotplug_work_func(struct work_struct *work)
{
	bool up = false, good_time = false;
	u64 now = jiffies;
//	pr_info("%s before mutex, now  - %llu,  last_change_time - %llu,  down_delay - %lu, up_delay - %lu", __func__, now, last_change_time, msecs_to_jiffies(CPU1_OFF_PENDING_MS), msecs_to_jiffies(CPU1_ON_PENDING_MS));
//	mutex_lock(tegra2_cpu_lock);
	switch (hp_state) {

	case TEGRA_HP_DISABLED:
		mutex_unlock(tegra2_cpu_lock);
		return;

	case TEGRA_HP_IDLE:
		pr_info("%s TEGRA_HP_IDLE", __func__);
		break;
	case TEGRA_HP_DOWN:
		if (cpu_online(1)) {
			pr_info("%s TEGRA_HP_DOWN", __func__);
			if ((now - last_change_time_hi) >= down_delay)
				good_time = true;
			}
		break;
	case TEGRA_HP_UP:
		if (!cpu_online(1)) {
			pr_info("%s TEGRA_HP_UP", __func__);
			up = true;
			if ((now - last_change_time_lo) >= up_delay)
				good_time = true;
			}
		break;

	default:
		pr_err("%s: invalid tegra hotplug state %d\n",
		       __func__, hp_state);
	}
	pending = false;
//	pr_info("%s before switch", __func__);
	if (good_time) {
		pr_info("%s good time", __func__);
		if (up) {
			//cpu_up(1);
			last_change_time_hi = now;
			last_change_time_lo = now;
		} else {
			//cpu_down(1);
			last_change_time_hi = now;
			last_change_time_lo = now;
		}
	}
//	mutex_unlock(tegra2_cpu_lock);
//	pr_info("%s after switch", __func__);
}

void tegra2_auto_hotplug_governor(unsigned int cpu_freq, bool suspend)
{
//	u64 now = jiffies;

	switch (hp_state) {

	case TEGRA_HP_DISABLED:
		//pr_info("%s TEGRA_HP_DISABLED", __func__);
		break;

	case TEGRA_HP_IDLE:
		if (cpu_freq > top_freq) {
			    hp_state = TEGRA_HP_UP;
				queue_delayed_work(hotplug_wq, &hotplug_work, up_delay);
//				last_change_time_hi = now;
				pr_info("%s going up", __func__);
		} else if (cpu_freq <= bottom_freq) {
				hp_state = TEGRA_HP_DOWN;
				queue_delayed_work(hotplug_wq, &hotplug_work, down_delay);
//				last_change_time_lo = now;
				pr_info("%s going down", __func__);
		}
		break;

	case TEGRA_HP_DOWN:
		if (cpu_freq > top_freq) {
				hp_state = TEGRA_HP_UP;
				queue_delayed_work(hotplug_wq, &hotplug_work, up_delay);
//				last_change_time_hi = now;
				pr_info("%s going up", __func__);
		} else if (cpu_freq > bottom_freq) {
			hp_state = TEGRA_HP_IDLE;
		}

		break;

	case TEGRA_HP_UP:
		if (cpu_freq <= bottom_freq) {
				hp_state = TEGRA_HP_DOWN;
				queue_delayed_work(hotplug_wq, &hotplug_work, down_delay);
//				last_change_time_lo = now;
				pr_info("%s going down", __func__);
		} else if (cpu_freq <= top_freq) {
			hp_state = TEGRA_HP_IDLE;
		}
		break;

	default:
		pr_err("%s: invalid tegra hotplug state %d\n",
		       __func__, hp_state);
		BUG();
	}
	//pr_info("%s exit", __func__);
}

int tegra2_auto_hotplug_init(struct mutex *cpu_lock)
{
	pr_info("%s", __func__);
	/*
	 * Not bound to the issuer CPU (=> high-priority), has rescue worker
	 * task, single-threaded, freezable.
	 */
	hotplug_wq = alloc_workqueue("cpu-tegra2",
		WQ_NON_REENTRANT | WQ_UNBOUND | WQ_RESCUER | WQ_FREEZABLE, 1);
	if (!hotplug_wq)
		return -ENOMEM;

	INIT_DELAYED_WORK(&hotplug_work, tegra2_auto_hotplug_work_func);

	cpu_clk = clk_get_sys(NULL, "cpu");
	if (IS_ERR(cpu_clk))
		return -ENOENT;

	/* top frequency = 80% of max CPU frequency */
	top_freq = clk_get_max_rate(cpu_clk)/1000;
	top_freq = (top_freq * 8)/10;

	/* bottom frequency = 250 MHz */
//	bottom_freq = 250000;

//	/* bottom frequency = 1/2 of max CPU frequency */
	bottom_freq = (clk_get_max_rate(cpu_clk)/1000)/2;

	if (top_freq < bottom_freq) {
		printk(KERN_ERR "%s: invalid range for CPU hotplug, top: %lu, bottom: %lu\n", __func__, top_freq, bottom_freq);
		return -1;
	}

	delay = msecs_to_jiffies(DELAY_MS);
	up_delay = msecs_to_jiffies(CPU1_ON_PENDING_MS);
	down_delay = msecs_to_jiffies(CPU1_OFF_PENDING_MS);

	tegra2_cpu_lock = cpu_lock;
	hp_state = INITIAL_STATE;
	pr_info("Tegra auto-hotplug initialized: %s\n",
		(hp_state == TEGRA_HP_DISABLED) ? "disabled" : "enabled");

	return 0;
}

void tegra2_auto_hotplug_exit(void)
{
	pr_info("%s", __func__);
	destroy_workqueue(hotplug_wq);
}
