/*
 * arch/arm/mach-tegra/board-olympus.h
 *
 * Copyright (C) 2010 Google, Inc.
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

#ifndef _MACH_TEGRA_BOARD_OLYMPUS_H
#define _MACH_TEGRA_BOARD_OLYMPUS_H

void olympus_pinmux_init(void);
int olympus_keypad_init(void);
void olympus_i2c_init(void);
int olympus_panel_init(void);
int olympus_wlan_init(void);

#define	BACKLIGHT_DEV		0
#define	TOUCHSCREEN_DEV		1

#define SERIAL_NUMBER_STRING_LEN 17

#define OLYMPUS_TOUCH_GPIO_RESET	TEGRA_GPIO_PF4
#define OLYMPUS_TOUCH_GPIO_INTR		TEGRA_GPIO_PF5

#ifndef PROX_INT_GPIO
#define	PROX_INT_GPIO	TEGRA_GPIO_PE1
#endif

#define UART_IPC_OLYMPUS	3
#endif
