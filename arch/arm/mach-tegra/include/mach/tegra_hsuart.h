/*
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
 */

#ifndef __MACH_TEGRA_HSUART_H
#define __MACH_TEGRA_HSUART_H

#include <linux/serial_core.h>
#include <linux/clk.h>

struct uart_clk_parent {
	const char	*name;
	struct clk	*parent_clk;
	unsigned long	fixed_clk_rate;
};

struct tegra_hsuart_platform_data {
	void (*wake_peer)(struct uart_port *);
	struct uart_clk_parent *parent_clk_list;
	int parent_clk_count;
	void (*exit_lpm_cb)(struct uart_port *);
	void (*rx_done_cb)(struct uart_port *);
};

int tegra_uart_is_tx_empty(struct uart_port *);
void tegra_uart_request_clock_on(struct uart_port *);
void tegra_uart_set_mctrl(struct uart_port *, unsigned int);
void tegra_uart_request_clock_off(struct uart_port *uport);

#endif
