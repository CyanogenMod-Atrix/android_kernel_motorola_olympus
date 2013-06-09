/*
 * arch/arm/mach-tegra/board-olympus-sdhci.c
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
#include <linux/reboot.h>
#include <linux/spi-tegra.h>
#include <linux/spi/spi.h>
#include <linux/spi/cpcap.h>
#include <linux/nvhost.h>

#include <asm/mach/time.h>
#include <asm/mach-types.h>

#include <mach/clk.h>
#include <mach/gpio.h>
#include <mach/io.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/i2s.h>

#include "clock.h"
#include "devices.h"
#include "gpio-names.h"
#include "pm.h"
#include "board.h"
#include "hwrev.h"
#include "board-olympus.h"

/*
 * SPI
 */

static struct tegra_spi_platform_data olympus_spi_pdata = {
	.is_dma_based		= true,
	.max_dma_buffer		= (16 * 1024),
	.is_clkon_always	= false,
	.max_rate			= 100000000,
};

static struct tegra_spi_platform_data olympus_spi_cpcap_pdata = {
	.is_dma_based		= true,
	.max_dma_buffer		= (16 * 1024),
	.is_clkon_always	= true,
	.max_rate			= 100000000,
};

struct spi_clk_parent spi_parent_clk[] = {
	[0] = {.name = "pll_p"},
#ifndef CONFIG_TEGRA_PLLM_RESTRICTED
	[1] = {.name = "pll_m"},
	[2] = {.name = "clk_m"},
#else
	[1] = {.name = "clk_m"},
#endif
};

void __init olympus_spi_init(void)
{

	int i;
	struct clk *clk;

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

	tegra_spi_device1.dev.platform_data = &olympus_spi_pdata;
	tegra_spi_device2.dev.platform_data = &olympus_spi_cpcap_pdata;
	tegra_spi_device3.dev.platform_data = &olympus_spi_pdata;
	tegra_spi_device4.dev.platform_data = &olympus_spi_pdata;

//	platform_device_register(&tegra_spi_slave_device1);
	platform_device_register(&tegra_spi_device2);
	platform_device_register(&tegra_spi_device3);
	platform_device_register(&tegra_spi_device4);

}



