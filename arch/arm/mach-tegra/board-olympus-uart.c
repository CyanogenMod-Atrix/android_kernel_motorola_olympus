/*
 * arch/arm/mach-tegra/board-mot.c
 *
 * Copyright (C) 2010 Google, Inc.
 * Copyright (C) 2010 NVIDIA Corporation
 * Copyright (C) 2010 Motorola, Inc.
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
#include <linux/fsl_devices.h>
#include <linux/gpio.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/platform_device.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/pda_power.h>
#include <linux/regulator/machine.h>
#include <linux/reboot.h>
#include <linux/serial_8250.h>
#include <linux/i2c.h>
#include <linux/i2c-tegra.h>
#include <linux/spi-tegra.h>
#include <linux/spi/spi.h>
#include <linux/spi/cpcap.h>
#include <linux/tegra_uart.h>
#include <linux/nvhost.h>

#include <linux/usb/composite.h>
#include <linux/usb/gadget.h>
#include <linux/usb/f_accessory.h>
#include <linux/fsl_devices.h>

#include <asm/mach/time.h>
#include <asm/mach-types.h>

#include <mach/clk.h>
#include <mach/gpio.h>
#include <mach/io.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/i2s.h>
#include <mach/kbc.h>
#include <mach/nand.h>
#include <mach/pinmux.h>
#include <mach/sdhci.h>
#include <mach/w1.h>
#include <mach/usb_phy.h>
#include <mach/olympus_usb.h>
#include <mach/nvmap.h>

#include "clock.h"
#include "devices.h"
#include "gpio-names.h"
#include "pm.h"
#include "board.h"
#include "hwrev.h"
#include "board-olympus.h"
#include <linux/mmc/host.h>
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
        &tegra_uartd_device,
	&tegra_uarte_device,
};

struct uart_clk_parent uart_parent_clk[] = {
	[0] = {.name = "pll_p"},
	[1] = {.name = "pll_m"},
	[2] = {.name = "clk_m"},
};

static struct tegra_uart_platform_data olympus_uart_pdata;

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

static void __init olympus_uart_init(void)
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
	olympus_uart_pdata.parent_clk_list = uart_parent_clk;
	olympus_uart_pdata.parent_clk_count = ARRAY_SIZE(uart_parent_clk);

	tegra_uarta_device.dev.platform_data = &olympus_uart_pdata;
	tegra_uartb_device.dev.platform_data = &olympus_uart_pdata;
	tegra_uartc_device.dev.platform_data = &olympus_uart_pdata;
        tegra_uartd_device.dev.platform_data = &olympus_uart_pdata;

	if (!is_tegra_debug_uartport_hs())
		uart_debug_init();

	platform_add_devices(olympus_uart_devices,
				ARRAY_SIZE(olympus_uart_devices));
}

