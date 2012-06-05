/*
 * arch/arm/mach-tegra/wakeups-t2.h
 *
 * Declarations of Tegra 2 LP0 wakeup sources
 *
 * Copyright (c) 2010, NVIDIA Corporation.
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

#ifndef __MACH_TEGRA_WAKEUPS_T2_H
#define __MACH_TEGRA_WAKEUPS_T2_H

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
#error "Tegra 2 wakeup sources valid only for CONFIG_ARCH_TEGRA_2x_SOC"
#endif

int tegra_irq_to_wake(int irq);
int tegra_wake_to_irq(int wake);

#define TEGRA_WAKE_GPIO_PO5	0
#define TEGRA_WAKE_GPIO_PV3	1
#define TEGRA_WAKE_GPIO_PL1	2
#define TEGRA_WAKE_GPIO_PB6	3
#define TEGRA_WAKE_GPIO_PN7	4
#define TEGRA_WAKE_GPIO_PA0	5
#define TEGRA_WAKE_GPIO_PU5	6
#define TEGRA_WAKE_GPIO_PU6	7
#define TEGRA_WAKE_GPIO_PC7	8
#define TEGRA_WAKE_GPIO_PS2	9
#define TEGRA_WAKE_GPIO_PAA1	10
#define TEGRA_WAKE_GPIO_PW3	11
#define TEGRA_WAKE_GPIO_PW2	12
#define TEGRA_WAKE_GPIO_PY6	13
#define TEGRA_WAKE_GPIO_PV6	14
#define TEGRA_WAKE_GPIO_PJ7	15
#define TEGRA_WAKE_RTC_ALARM	16
#define TEGRA_WAKE_KBC_EVENT	17
#define TEGRA_WAKE_PWR_INT	18
#define TEGRA_WAKE_USB1_VBUS	19
#define TEGRA_WAKE_USB3_VBUS	20
#define TEGRA_WAKE_USB1_ID	21
#define TEGRA_WAKE_USB3_ID	22
#define TEGRA_WAKE_GPIO_PI5	23
#define TEGRA_WAKE_GPIO_PV2	24
#define TEGRA_WAKE_GPIO_PS4	25
#define TEGRA_WAKE_GPIO_PS5	26
#define TEGRA_WAKE_GPIO_PS0	27
#define TEGRA_WAKE_GPIO_PQ6	28
#define TEGRA_WAKE_GPIO_PQ7	29
#define TEGRA_WAKE_GPIO_PN2	30

#endif
