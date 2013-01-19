/*
 * arch/arm/mach-tegra/board-olympus-panel.c
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

#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/i2c.h>
#include <linux/resource.h>
#include <linux/platform_device.h>
#include <linux/earlysuspend.h>
#include <linux/delay.h>
#include <linux/keyreset.h>
#include <linux/input.h>

#include <asm/mach-types.h>
#include <mach/clk.h>
#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/nvmap.h>
#include <mach/dc.h>
#include <mach/fb.h>
#include <mach/tegra_fb.h>

#include "board.h"
#include "board-olympus.h"
#include "gpio-names.h"

/* Framebuffer */
static struct resource fb_resource[] = {
	[0] = {
		.start  = INT_DISPLAY_GENERAL,
		.end    = INT_DISPLAY_GENERAL,
		.flags  = IORESOURCE_IRQ,
	},
	[1] = {
		.start	= TEGRA_DISPLAY_BASE,
		.end	= TEGRA_DISPLAY_BASE + TEGRA_DISPLAY_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
	[2] = {
		.start	= 0x1c03a000,
		.end	= 0x1c03a000 + 0x500000 - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct tegra_fb_lcd_data tegra_fb_lcd_platform_data = {
	.lcd_xres	= 480,
	.lcd_yres	= 854,
	.fb_xres	= 480,
	.fb_yres	= 854,
	.bits_per_pixel	= 16,
};

static struct platform_device tegra_fb_device = {
	.name 		= "tegrafb",
	.id		= 0,
	.resource	= fb_resource,
	.num_resources 	= ARRAY_SIZE(fb_resource),
	.dev = {
		.platform_data = &tegra_fb_lcd_platform_data,
	},
};

static struct nvmap_platform_carveout olympus_carveouts[] = {
	[0] = {
		.name		= "iram",
		.usage_mask	= NVMAP_HEAP_CARVEOUT_IRAM,
		.base		= TEGRA_IRAM_BASE,
		.size		= TEGRA_IRAM_SIZE,
		.buddy_size	= 0,
	},
	[1] = {
		.name		= "generic-0",
		.usage_mask	= NVMAP_HEAP_CARVEOUT_GENERIC,
             .base       = 0,    /* Filled in by olympus_panel_init() */
             .size       = 0,    /* Filled in by olympus_panel_init() */
		.buddy_size	= SZ_32K,
	},
};

static struct nvmap_platform_data olympus_nvmap_data = {
	.carveouts	= olympus_carveouts,
	.nr_carveouts	= ARRAY_SIZE(olympus_carveouts),
};

static struct platform_device olympus_nvmap_device = {
	.name	= "tegra-nvmap",
	.id	= -1,
	.dev	= {
		.platform_data = &olympus_nvmap_data,
	},
};

static struct platform_device *olympus_gfx_devices[] __initdata = {
	&olympus_nvmap_device,
};

int __init olympus_panel_init(void) {

	int err;

	olympus_carveouts[1].base = tegra_carveout_start;
	olympus_carveouts[1].size = tegra_carveout_size;

	err = platform_add_devices(olympus_gfx_devices,
				   ARRAY_SIZE(olympus_gfx_devices));

	return platform_device_register(&tegra_fb_device);
}
