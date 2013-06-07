/*
 * arch/arm/mach-tegra/board-olympus-bluetooth.c
 *
 * Copyright 2013: Olympus Kernel Project
 * <http://forum.xda-developers.com/showthread.php?t=2016837>
 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/hrtimer.h>
#include <linux/irq.h>
#include <linux/rfkill.h>
#include <linux/platform_device.h>
#include <linux/serial_core.h>
#include <linux/wakelock.h>
#include <asm/mach-types.h>

#include "gpio-names.h"

static struct resource olympus_bcm4329_rfkill_resources[] = {
	{
		.name	= "bcm4329_nreset_gpio",
		.start	= 0,
		.end	= 0,
		.flags	= IORESOURCE_IO,
	},
	{
		.name	= "bcm4329_nshutdown_gpio",
		.start	= TEGRA_GPIO_PU0,
		.end	= TEGRA_GPIO_PU0,
		.flags	= IORESOURCE_IO,
	},
	{
		.name	= "bcm4329_wake_gpio",
		.start	= TEGRA_GPIO_PU1,
		.end	= TEGRA_GPIO_PU1,
		.flags	= IORESOURCE_IO,
	},
	{
		.name	= "bcm4329_host_wake_gpio",
		.start	= TEGRA_GPIO_PU6,
		.end	= TEGRA_GPIO_PU6,
		.flags	= IORESOURCE_IO,
	},
};

static struct platform_device olympus_bcm4329_rfkill_device = {
	.name		= "bcm4329_rfkill",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(olympus_bcm4329_rfkill_resources),
	.resource	= olympus_bcm4329_rfkill_resources,
};

void __init olympus_bt_rfkill(void)
{
	olympus_bcm4329_rfkill_resources[0].start =
	olympus_bcm4329_rfkill_resources[0].end = TEGRA_GPIO_PU4;

	tegra_gpio_enable (TEGRA_GPIO_PU4);
	printk("%s: registering bcm4329_rfkill device...\n", __func__);

	platform_device_register(&olympus_bcm4329_rfkill_device);
	return;
}
