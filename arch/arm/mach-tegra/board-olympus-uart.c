/*
 * arch/arm/mach-tegra/board-olympus-uart.c
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
#include <linux/clk.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/pda_power.h>
#include <linux/regulator/machine.h>
#include <linux/reboot.h>
#include <linux/serial_8250.h>

#include <asm/mach/time.h>
#include <asm/mach-types.h>

#include <mach/clk.h>
#include <mach/gpio.h>
#include <mach/io.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/i2s.h>
#include <mach/i2s.h>
#include <mach/tegra_hsuart.h>

#include "clock.h"
#include "devices.h"
#include "gpio-names.h"
#include "pm.h"
#include "board.h"
#include "hwrev.h"
#include "board-olympus.h"

/*
 * UART configuration for Olympus:
 *   UARTA - whisper (/dev/ttyHS0)
 *   UARTB - serial debug console (/dev/tty0)
 *   UARTC - bluetooth (/dev/ttyHS2)
 *   UARTD - modem; configured in board-olympus-modem.c (/dev/ttyHS3)
 */

extern void bcm_bt_rx_done_locked(struct uart_port *uport);
extern void bcm_bt_lpm_exit_lpm_locked(struct uart_port *uport);

static struct plat_serial8250_port debug_uart_platform_data[] = {
	{
		.membase	= IO_ADDRESS(TEGRA_UARTB_BASE),
		.mapbase	= TEGRA_UARTB_BASE,
		.irq		= INT_UARTB,
		.flags		= UPF_BOOT_AUTOCONF | UPF_FIXED_TYPE,
		.type           = PORT_TEGRA,
		.iotype		= UPIO_MEM,
		.regshift	= 2,
		.uartclk	= 216000000,
	}, {
		.flags		= 0,
	}
};

static struct platform_device debug_uart = {
	.name = "serial8250",
	.id = PLAT8250_DEV_PLATFORM,
	.dev = {
		.platform_data = debug_uart_platform_data,
	},
};

static struct platform_device *olympus_uart_devices[] __initdata = {
	&tegra_uarta_device,
	&tegra_uartb_device,
	&tegra_uartc_device,
//	&tegra_uartd_device,
};

struct uart_clk_parent uart_parent_clk[] = {
	[0] = {.name = "pll_p"},
	[1] = {.name = "pll_m"},
	[2] = {.name = "clk_m"},
};

static void __init uart_debug_init(void)
{
	unsigned long rate;
	struct clk *c;

	pr_info("Selecting UARTB as the debug console\n");
	olympus_uart_devices[1] = &debug_uart;
	debug_uart_clk = clk_get_sys("serial8250.0", "uartb");
	debug_uart_port_base = ((struct plat_serial8250_port *)(
							debug_uartb_device.dev.platform_data))->mapbase;

		/* Clock enable for the debug channel */
		if (!IS_ERR_OR_NULL(debug_uart_clk)) {
		rate = debug_uart_platform_data[0].uartclk;
			pr_info("The debug console clock name is %s\n",
						debug_uart_clk->name);
			c = tegra_get_clock_by_name("pll_p");
			if (IS_ERR_OR_NULL(c))
				pr_err("Not getting the parent clock pll_p\n");
			else
				clk_set_parent(debug_uart_clk, c);

			clk_enable(debug_uart_clk);
			clk_set_rate(debug_uart_clk, rate);
		} else {
			pr_err("Not getting the clock %s for debug console\n",
						debug_uart_clk->name);
	}
}

static struct tegra_hsuart_platform_data olympus_uart_pdata[4];

void __init olympus_uart_init(void)
{
	int i;
	struct clk *c;

	for (i = 0; i < ARRAY_SIZE(uart_parent_clk); ++i) {
		c = tegra_get_clock_by_name(uart_parent_clk[i].name);
		if (IS_ERR_OR_NULL(c)) {
			pr_err("Not able to get the clock for %s\n",
						uart_parent_clk[i].name);
			continue;
		}
		uart_parent_clk[i].parent_clk = c;
		uart_parent_clk[i].fixed_clk_rate = clk_get_rate(c);
	}
	olympus_uart_pdata[0].parent_clk_list = uart_parent_clk;
	olympus_uart_pdata[0].parent_clk_count = ARRAY_SIZE(uart_parent_clk);

	olympus_uart_pdata[1].parent_clk_list = uart_parent_clk;
	olympus_uart_pdata[1].parent_clk_count = ARRAY_SIZE(uart_parent_clk);

	olympus_uart_pdata[2].parent_clk_list = uart_parent_clk;
	olympus_uart_pdata[2].parent_clk_count = ARRAY_SIZE(uart_parent_clk);
	olympus_uart_pdata[2].exit_lpm_cb = bcm_bt_lpm_exit_lpm_locked;
	olympus_uart_pdata[2].rx_done_cb = bcm_bt_rx_done_locked;

	olympus_uart_pdata[3].parent_clk_list = uart_parent_clk;
	olympus_uart_pdata[3].parent_clk_count = ARRAY_SIZE(uart_parent_clk);

	tegra_uarta_device.dev.platform_data = &olympus_uart_pdata[0];
	tegra_uartb_device.dev.platform_data = &olympus_uart_pdata[1];
	tegra_uartc_device.dev.platform_data = &olympus_uart_pdata[2];
	tegra_uartd_device.dev.platform_data = &olympus_uart_pdata[3];

	if (!is_tegra_debug_uartport_hs())
		uart_debug_init();

	platform_add_devices(olympus_uart_devices,
				ARRAY_SIZE(olympus_uart_devices));
}
