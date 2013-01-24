/*
 * arch/arm/mach-tegra/board-olympus-panel.c
 *
 * Copyright (c) 2010-2011, NVIDIA Corporation.
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

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/resource.h>
#include <asm/mach-types.h>
#include <linux/platform_device.h>
#include <linux/earlysuspend.h>
#include <linux/pwm_backlight.h>
#include <linux/nvhost.h>
#include <mach/nvmap.h>
#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/dc.h>
#include <mach/fb.h>

#include "devices.h"
#include "gpio-names.h"
#include "board.h"
#include "board-olympus.h"

/*panel power on sequence timing*/
#define olympus_pnl_to_lvds_ms	0
#define olympus_lvds_to_bl_ms	200

#define OLYMPUS_LVDS_SHUTDOWN 35 // Is this right?
#define OLYMPUS_EN_VDD_PANEL 47
#define OLYMPUS_HDMI_ENB 46 /* unconfirmed */ // Does betelgeuse have HDMI enbl?
#define OLYMPUS_HDMI_HPD 111 /* 1=HDMI plug detected */

#ifdef CONFIG_TEGRA_DC
static struct regulator *olympus_hdmi_reg = NULL;
static struct regulator *olympus_hdmi_pll = NULL;
#endif
/*
static int olympus_disp1_check_fb(struct device *dev, struct fb_info *info);
*/
static int olympus_panel_enable(void)
{
	gpio_set_value(OLYMPUS_EN_VDD_PANEL, 1);
	mdelay(olympus_pnl_to_lvds_ms);
	gpio_set_value(OLYMPUS_LVDS_SHUTDOWN, 1);
	mdelay(olympus_lvds_to_bl_ms);
	return 0;
}

static int olympus_panel_disable(void)
{
	gpio_set_value(OLYMPUS_LVDS_SHUTDOWN, 0);
	gpio_set_value(OLYMPUS_EN_VDD_PANEL, 0);
	return 0;
}

static int olympus_hdmi_enable(void)
{
	if (!olympus_hdmi_reg) {
		olympus_hdmi_reg = regulator_get(NULL, "vhdmi"); /* LD07 */
		if (IS_ERR_OR_NULL(olympus_hdmi_reg)) {
			pr_err("hdmi: couldn't get regulator vhdmi\n");
			olympus_hdmi_reg = NULL;
			return PTR_ERR(olympus_hdmi_reg);
		}
	}
	regulator_enable(olympus_hdmi_reg);

	if (!olympus_hdmi_pll) {
		olympus_hdmi_pll = regulator_get(NULL, "avdd_hdmi_pll"); /* LD08 */
		if (IS_ERR_OR_NULL(olympus_hdmi_pll)) {
			pr_err("hdmi: couldn't get regulator avdd_hdmi_pll\n");
			olympus_hdmi_pll = NULL;
			regulator_disable(olympus_hdmi_reg);
			olympus_hdmi_reg = NULL;
			return PTR_ERR(olympus_hdmi_pll);
		}
	}
	regulator_enable(olympus_hdmi_pll);
	return 0;
}

static int olympus_hdmi_disable(void)
{
	regulator_disable(olympus_hdmi_reg);
	regulator_disable(olympus_hdmi_pll);
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
		.flags	= IORESOURCE_MEM,
	},
	{
		.name = "dsi_regs",
		.start = TEGRA_DSI_BASE,
		.end = TEGRA_DSI_BASE + TEGRA_DSI_SIZE-1,
		.flags = IORESOURCE_MEM,
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
		.end	= TEGRA_DISPLAY2_BASE + TEGRA_DISPLAY2_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "fbmem",
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "hdmi_regs",
		.start	= TEGRA_HDMI_BASE,
		.end	= TEGRA_HDMI_BASE + TEGRA_HDMI_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct tegra_dc_mode olympus_panel_modes[] = {
	{
		/*.pclk = 38333333,
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
		.stereo_mode = 0,*/
		.pclk = 12200000,
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
	},
};

static struct tegra_fb_data olympus_fb_data = {
	.win		= 0,
	.xres		= 960,
	.yres		= 540,
	.bits_per_pixel	= 32,
	.flags		= TEGRA_FB_FLIP_ON_PROBE,
};

static struct tegra_fb_data olympus_hdmi_fb_data = {
	.win		= 0,
	.xres		= 1280,
	.yres		= 720,
	.bits_per_pixel	= 32,
	.flags		= TEGRA_FB_FLIP_ON_PROBE,
};

static struct tegra_dsi_out olympus_dsi_out = {
	.n_data_lanes = 2,
	.pixel_format = TEGRA_DSI_PIXEL_FORMAT_24BIT_P,
	.refresh_rate = 60,
	.virtual_channel = TEGRA_DSI_VIRTUAL_CHANNEL_0,
	.panel_reset = 0,
//	.power_saving_suspend = true,
	.power_saving_suspend = false,
/*	.dsi_init_cmd = &dsi_init_cmd, 	
	.n_init_cmd = ARRAY_SIZE(dsi_init_cmd),
	.dsi_early_suspend_cmd = &dsi_early_suspend_cmd,    				
	.n_early_suspend_cmd = ARRAY_SIZE(dsi_early_suspend_cmd),    
	.dsi_late_resume_cmd = &dsi_late_resume_cmd,	
	.n_late_resume_cmd = ARRAY_SIZE(dsi_late_resume_cmd),
	.dsi_suspend_cmd = &dsi_suspend_cmd,
	.n_suspend_cmd = ARRAY_SIZE(dsi_suspend_cmd),*/
	.video_data_type = TEGRA_DSI_VIDEO_TYPE_COMMAND_MODE,
	.video_clock_mode = TEGRA_DSI_VIDEO_CLOCK_TX_ONLY,
	.lp_cmd_mode_freq_khz = 10000, 
	/* TODO: Get the vender recommended freq */
	.lp_read_cmd_mode_freq_khz = 230000,
	.te_polarity_low = true,

};

static struct tegra_dc_out olympus_disp1_out = {
//	.type		= TEGRA_DC_OUT_RGB,
	.type		= TEGRA_DC_OUT_DSI,

	.align		= TEGRA_DC_ALIGN_MSB,
	.order		= TEGRA_DC_ORDER_RED_BLUE,
//	.depth		= 18,
//	.dither		= TEGRA_DC_ORDERED_DITHER,

	.height = 91, /* mm */
	.width = 51, /* mm */

	.modes	 	= olympus_panel_modes,
	.n_modes 	= ARRAY_SIZE(olympus_panel_modes),

	.dsi		= &olympus_dsi_out,

	.enable		= olympus_panel_enable,
	.disable	= olympus_panel_disable,
};

static struct tegra_dc_out olympus_disp2_out = {
	.type		= TEGRA_DC_OUT_HDMI,
	.flags		= TEGRA_DC_OUT_HOTPLUG_HIGH,

	.dcc_bus	= 1,
	.hotplug_gpio	= OLYMPUS_HDMI_HPD,

	.max_pixclock	= KHZ2PICOS(148500),

	.align		= TEGRA_DC_ALIGN_MSB,
	.order		= TEGRA_DC_ORDER_RED_BLUE,

	.enable		= olympus_hdmi_enable,
	.disable	= olympus_hdmi_disable,
};

static struct tegra_dc_platform_data olympus_disp1_pdata = {
	.flags		= TEGRA_DC_FLAG_ENABLED,
	.default_out	= &olympus_disp1_out,
	.emc_clk_rate	= 300000000,
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

static struct tegra_dc_platform_data olympus_disp2_pdata = {
	.flags		= 0,
	.default_out	= &olympus_disp2_out,
	.fb		= &olympus_hdmi_fb_data,
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

#ifdef CONFIG_HAS_EARLYSUSPEND
/* put early_suspend/late_resume handlers here for the display in order
 * to keep the code out of the display driver, keeping it closer to upstream
 */
struct early_suspend olympus_panel_early_suspender;

static void olympus_panel_early_suspend(struct early_suspend *h)
{
	/* power down LCD, add use a black screen for HDMI */
	if (num_registered_fb > 0)
		fb_blank(registered_fb[0], FB_BLANK_POWERDOWN);
	if (num_registered_fb > 1)
		fb_blank(registered_fb[1], FB_BLANK_NORMAL);
#if 0
#ifdef CONFIG_TEGRA_CONVSERVATIVE_GOV_ON_EARLYSUPSEND
	cpufreq_save_default_governor();
	cpufreq_set_conservative_governor();
	cpufreq_set_conservative_governor_param("up_threshold",
			SET_CONSERVATIVE_GOVERNOR_UP_THRESHOLD);

	cpufreq_set_conservative_governor_param("down_threshold",
			SET_CONSERVATIVE_GOVERNOR_DOWN_THRESHOLD);

	cpufreq_set_conservative_governor_param("freq_step",
		SET_CONSERVATIVE_GOVERNOR_FREQ_STEP);
#endif
#endif
}

static void olympus_panel_late_resume(struct early_suspend *h)
{
	unsigned i;
#if 0
#ifdef CONFIG_TEGRA_CONVSERVATIVE_GOV_ON_EARLYSUPSEND
	cpufreq_restore_default_governor();
#endif
#endif
	for (i = 0; i < num_registered_fb; i++)
		fb_blank(registered_fb[i], FB_BLANK_UNBLANK);
}
#endif

int __init olympus_panel_init(void)
{
	int err;
	struct resource __maybe_unused *res;

	gpio_request(OLYMPUS_EN_VDD_PANEL, "pnl_pwr_enb");
	gpio_direction_output(OLYMPUS_EN_VDD_PANEL, 1);
	tegra_gpio_enable(OLYMPUS_EN_VDD_PANEL);

	gpio_request(OLYMPUS_LVDS_SHUTDOWN, "lvds_shdn");
	gpio_direction_output(OLYMPUS_LVDS_SHUTDOWN, 1);
	tegra_gpio_enable(OLYMPUS_LVDS_SHUTDOWN);

	tegra_gpio_enable(OLYMPUS_HDMI_ENB);
	gpio_request(OLYMPUS_HDMI_ENB, "hdmi_5v_en");
	gpio_direction_output(OLYMPUS_HDMI_ENB, 1);

	tegra_gpio_enable(OLYMPUS_HDMI_HPD);
	gpio_request(OLYMPUS_HDMI_HPD, "hdmi_hpd");
	gpio_direction_input(OLYMPUS_HDMI_HPD);

#ifdef CONFIG_HAS_EARLYSUSPEND
	olympus_panel_early_suspender.suspend = olympus_panel_early_suspend;
	olympus_panel_early_suspender.resume = olympus_panel_late_resume;
	olympus_panel_early_suspender.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	register_early_suspend(&olympus_panel_early_suspender);
#endif

#if defined(CONFIG_TEGRA_GRHOST) && defined(CONFIG_TEGRA_DC)
	res = nvhost_get_resource_byname(&olympus_disp1_device,
		IORESOURCE_MEM, "fbmem");
	res->start = tegra_fb_start;
	res->end = tegra_fb_start + tegra_fb_size - 1;

	res = nvhost_get_resource_byname(&olympus_disp2_device,
		IORESOURCE_MEM, "fbmem");
	res->start = tegra_fb2_start;
	res->end = tegra_fb2_start + tegra_fb2_size - 1;
#endif

	/* Copy the bootloader fb to the fb. */
	tegra_move_framebuffer(tegra_fb_start, tegra_bootloader_fb_start,
		min(tegra_fb_size, tegra_bootloader_fb_size));

	nvhost_device_register(&nvavp_device);

#if defined(CONFIG_TEGRA_GRHOST) && defined(CONFIG_TEGRA_DC)
//	if (!err)
		err = nvhost_device_register(&olympus_disp1_device);

	if (!err)
		err = nvhost_device_register(&olympus_disp2_device);
#endif

	return err;
}


