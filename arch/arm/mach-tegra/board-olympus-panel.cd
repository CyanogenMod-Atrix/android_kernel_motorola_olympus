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
#include <linux/nvhost.h>

#include <asm/mach-types.h>
#include <mach/clk.h>
#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/nvmap.h>
#include <mach/dc.h>
#include <mach/fb.h>

#include "board.h"
#include "board-olympus.h"
#include "gpio-names.h"

#define LCD_POWER_GPIO TEGRA_GPIO_PF7
#define HDMI_POWER_GPIO TEGRA_GPIO_PF6
#define HDMI_HPD_GPIO TEGRA_GPIO_PN7

/* Display Controller */

static int olympus_panel_enable(void)
{
    gpio_set_value(LCD_POWER_GPIO, 1);
    return 0;
}

static int olympus_panel_disable(void)
{
    gpio_set_value(LCD_POWER_GPIO, 0);
    return 0;
}

static struct resource olympus_disp1_resources[] = {
	{
		.name	= "irq",
		.start	= INT_DISPLAY_GENERAL,
		.end	= INT_DISPLAY_GENERAL,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "regs",
		.start	= TEGRA_DISPLAY_BASE,
		.end	= TEGRA_DISPLAY_BASE + TEGRA_DISPLAY_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "fbmem",
		/* .start and .end to be filled in later */
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource olympus_disp2_resources[] = {
	{
		.name	= "irq",
		.start	= INT_DISPLAY_B_GENERAL,
		.end	= INT_DISPLAY_B_GENERAL,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "regs",
		.start	= TEGRA_DISPLAY2_BASE,
		.end	= TEGRA_DISPLAY2_BASE + TEGRA_DISPLAY2_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "fbmem",
		/* .start and .end to be filled in later */
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "hdmi_regs",
		.start	= TEGRA_HDMI_BASE,
		.end	= TEGRA_HDMI_BASE + TEGRA_HDMI_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct tegra_dc_mode olympus_panel_modes[] = {
	{
		.pclk = 38333333,
		.h_ref_to_sync = 4,
		.v_ref_to_sync = 1,
		.h_sync_width = 16,
		.v_sync_width = 1,
		.h_back_porch = 32,
		.v_back_porch = 1,
		.h_active = 540,
		.v_active = 960,
		.h_front_porch = 32,
		.v_front_porch = 2,
		.stereo_mode = 0,
	},
};

static struct tegra_dc_out_pin olympus_dc_out_pins[] = {
	{
		.name = TEGRA_DC_OUT_PIN_PIXEL_CLOCK,
		.pol = TEGRA_DC_OUT_PIN_POL_LOW,
	},
};

static struct tegra_fb_data olympus_fb_data = {
	.win		= 0,
	.xres		= 960,
	.yres		= 540,
	.bits_per_pixel	= -1,
	.flags		= TEGRA_FB_FLIP_ON_PROBE,
};

static struct tegra_dc_out olympus_disp1_out = {
	.type = TEGRA_DC_OUT_RGB,

	.align = TEGRA_DC_ALIGN_MSB,
	.order = TEGRA_DC_ORDER_RED_BLUE,
	.depth = 24,

	.height = 91, /* mm */
	.width = 51, /* mm */

	.modes = olympus_panel_modes,
	.n_modes = ARRAY_SIZE(olympus_panel_modes),

	.out_pins = olympus_dc_out_pins,
	.n_out_pins = ARRAY_SIZE(olympus_dc_out_pins),

	.enable = olympus_panel_enable,
	.disable = olympus_panel_disable,
};

static struct tegra_dc_platform_data olympus_disp1_pdata = {
	.flags		= TEGRA_DC_FLAG_ENABLED,
	.emc_clk_rate	= 300000000,
	.default_out	= &olympus_disp1_out,
	.fb		= &olympus_fb_data,
};

static struct nvhost_device olympus_disp1_device = {
	.name		= "tegradc",
	.id		= 0,
	.resource	= olympus_disp1_resources,
	.num_resources	= ARRAY_SIZE(olympus_disp1_resources),
	.dev = {
		.platform_data = &olympus_disp1_pdata,
	},
};

static int olympus_hdmi_init(void)
{
	tegra_gpio_enable(HDMI_POWER_GPIO);
	gpio_request(HDMI_POWER_GPIO, "hdmi_5v_en");
	gpio_direction_output(HDMI_POWER_GPIO, 1);

	tegra_gpio_enable(HDMI_HPD_GPIO);
	gpio_request(HDMI_HPD_GPIO, "hdmi_hpd");
	gpio_direction_input(HDMI_HPD_GPIO);


	return 0;
}

static struct tegra_dc_out olympus_disp2_out = {
	.type = TEGRA_DC_OUT_HDMI,
	.flags = TEGRA_DC_OUT_HOTPLUG_HIGH,

	.dcc_bus = 1,
	.hotplug_gpio = TEGRA_GPIO_PN7,

	.align = TEGRA_DC_ALIGN_MSB,
	.order = TEGRA_DC_ORDER_RED_BLUE,
};

static struct tegra_fb_data olympus_disp2_fb_data = {
	.win		= 0,
	.xres		= 1280,
	.yres		= 720,
	.bits_per_pixel	= 32,
};

static struct tegra_dc_platform_data olympus_disp2_pdata = {
	.flags		= 0,
	.emc_clk_rate	= ULONG_MAX,
	.default_out	= &olympus_disp2_out,
	.fb		= &olympus_disp2_fb_data,
};

static struct nvhost_device olympus_disp2_device = {
	.name		= "tegradc",
	.id		= 1,
	.resource	= olympus_disp2_resources,
	.num_resources	= ARRAY_SIZE(olympus_disp2_resources),
	.dev = {
		.platform_data = &olympus_disp2_pdata,
	},
};

static struct nvmap_platform_carveout olympus_carveouts[] = {
	[0] = NVMAP_HEAP_CARVEOUT_IRAM_INIT,
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

#ifdef CONFIG_HAS_EARLYSUSPEND
/* put early_suspend/late_resume handlers here for the display in order
 * to keep the code out of the display driver, keeping it closer to upstream
 */
struct early_suspend olympus_panel_early_suspender;

static void olympus_panel_early_suspend(struct early_suspend *h)
{
	if (num_registered_fb > 0)
		fb_blank(registered_fb[0], FB_BLANK_POWERDOWN);
}

static void olympus_panel_late_resume(struct early_suspend *h)
{
	if (num_registered_fb > 0)
		fb_blank(registered_fb[0], FB_BLANK_UNBLANK);
}
#endif

int __init olympus_panel_init(void)
{
	struct resource *res;
	int err;

	printk(KERN_INFO "pICS_%s: Starting...",__func__);

	olympus_hdmi_init();

#ifdef CONFIG_HAS_EARLYSUSPEND
	olympus_panel_early_suspender.suspend = olympus_panel_early_suspend;
	olympus_panel_early_suspender.resume = olympus_panel_late_resume;
	olympus_panel_early_suspender.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	register_early_suspend(&olympus_panel_early_suspender);
#endif

	olympus_carveouts[1].base = tegra_carveout_start;
	olympus_carveouts[1].size = tegra_carveout_size;

	err = platform_add_devices(olympus_gfx_devices,
				   ARRAY_SIZE(olympus_gfx_devices));

	res = nvhost_get_resource_byname(&olympus_disp1_device,
		IORESOURCE_MEM, "fbmem");
	res->start = tegra_fb_start;
	res->end = tegra_fb_start + tegra_fb_size - 1;

	res = nvhost_get_resource_byname(&olympus_disp2_device,
		IORESOURCE_MEM, "fbmem");
	res->start = tegra_fb2_start;
	res->end = tegra_fb2_start + tegra_fb2_size - 1;

	tegra_move_framebuffer(tegra_fb_start, tegra_bootloader_fb_start,
		min(tegra_fb_size, tegra_bootloader_fb_size));

	if (!err)
		err = nvhost_device_register(&olympus_disp1_device);

	if (!err)
		err = nvhost_device_register(&olympus_disp2_device);

	printk(KERN_INFO "pICS_%s: Ending...",__func__);

	return err;

}

