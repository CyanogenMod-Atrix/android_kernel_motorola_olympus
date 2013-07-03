/*
 * arch/arm/mach-tegra/board-olympus-device.c
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

#include <linux/kernel.h>
#include <linux/init.h>

#include "devices.h"
#include "board-olympus.h"

static struct platform_device *olympus_devices[] __initdata = {
	//&tegra_pmu_device,
	//&tegra_wdt_device,
   	//&tegra_pwfm1_device,
};

void __init olympus_devices_init()
{

	platform_add_devices(olympus_devices, ARRAY_SIZE(olympus_devices));

}

