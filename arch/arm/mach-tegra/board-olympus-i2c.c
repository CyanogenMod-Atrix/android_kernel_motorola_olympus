/*
 * arch/arm/mach-tegra/board-olympus-sdhci.c
 *
 * Copyright (C) 2010 Google, Inc.
 * Copyright (C) 2011-2012 NVIDIA Corporation.
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
#include <linux/clk.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/i2c.h>
#include <linux/i2c-tegra.h>

#include <asm/mach/time.h>
#include <asm/mach-types.h>

#include <mach/clk.h>
#include <mach/gpio.h>
#include <mach/io.h>
#include <mach/iomap.h>
#include <mach/irqs.h>

#include "clock.h"
#include "devices.h"
#include "gpio-names.h"
#include "pm.h"
#include "board.h"
#include "hwrev.h"
#include "board-olympus.h"

static struct tegra_i2c_platform_data olympus_i2c1_platform_data = {
	.adapter_nr	= 0,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.scl_gpio	= {TEGRA_GPIO_PC4, 0},
	.sda_gpio	= {TEGRA_GPIO_PC5, 0},
	.arb_recovery = arb_lost_recovery,
	.slave_addr = 0xFC,
};

static struct tegra_i2c_platform_data olympus_i2c2_platform_data = {
	.adapter_nr	= 1,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
};

static struct tegra_i2c_platform_data olympus_i2c3_platform_data = {
	.adapter_nr	= 2,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.scl_gpio	= {TEGRA_GPIO_PBB2, 0},
	.sda_gpio	= {TEGRA_GPIO_PBB3, 0},
	.arb_recovery = arb_lost_recovery,
	.slave_addr = 0xFC,
};

static struct tegra_i2c_platform_data olympus_dvc_platform_data = {
	.adapter_nr	= 3,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.is_dvc		= true,
	.scl_gpio	= {TEGRA_GPIO_PZ6, 0},
	.sda_gpio	= {TEGRA_GPIO_PZ7, 0},
	.arb_recovery = arb_lost_recovery,
};

void olympus_i2c_init(void)
{
	tegra_i2c_device1.dev.platform_data = &olympus_i2c1_platform_data;
	tegra_i2c_device2.dev.platform_data = &olympus_i2c2_platform_data;
	tegra_i2c_device3.dev.platform_data = &olympus_i2c3_platform_data;
	tegra_i2c_device4.dev.platform_data = &olympus_dvc_platform_data;

	platform_device_register(&tegra_i2c_device1);
	platform_device_register(&tegra_i2c_device2);
	platform_device_register(&tegra_i2c_device3);
	platform_device_register(&tegra_i2c_device4);
}
