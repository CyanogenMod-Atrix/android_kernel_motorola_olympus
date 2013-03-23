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

#ifdef CONFIG_USB_G_ANDROID
#include <linux/usb/android_composite.h>
#endif

/*
 * SPI
 */
#if 0
static struct tegra_spi_platform_data olympus_spi_pdata = {
	.is_dma_based		= true,
	.max_dma_buffer		= (16 * 1024),
	.is_clkon_always	= false,
	.max_rate		= 100000000,
};
/*
struct spi_clk_parent spi_parent_clk[] = {
	[0] = {.name = "pll_p"},
#ifndef CONFIG_TEGRA_PLLM_RESTRICTED
	[1] = {.name = "pll_m"},
	[2] = {.name = "clk_m"},
#else
	[1] = {.name = "clk_m"},
#endif
};*/

static void __init olympus_spi_init(void)
{
//	int i;
//	struct clk *clk;
/*
	printk("this board spi init\n");
        for (i = 0; i < ARRAY_SIZE(spi_parent_clk); ++i) {
      		clk = tegra_get_clock_by_name(spi_parent_clk[i].name);
		if (IS_ERR_OR_NULL(clk)) {
			pr_err("Not able to get the clock for %s\n", spi_parent_clk[i].name);
			continue;
                }
		spi_parent_clk[i].parent_clk = clk;
		spi_parent_clk[i].fixed_clk_rate = clk_get_rate(clk);
	}
	olympus_spi_pdata.parent_clk_list = spi_parent_clk;
	olympus_spi_pdata.parent_clk_count = ARRAY_SIZE(spi_parent_clk);
*/
	tegra_spi_device1.dev.platform_data = &olympus_spi_pdata;
	platform_device_register(&tegra_spi_device1);

	tegra_spi_device2.dev.platform_data = &olympus_spi_pdata;
	platform_device_register(&tegra_spi_device2);

	tegra_spi_device3.dev.platform_data = &olympus_spi_pdata;
	platform_device_register(&tegra_spi_device3);

	tegra_spi_device3.dev.platform_data = &olympus_spi_pdata;
	platform_device_register(&tegra_spi_device4);

}
#endif


