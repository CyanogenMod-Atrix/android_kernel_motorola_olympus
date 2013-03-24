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
#include <linux/regulator/consumer.h>
#include <linux/platform_device.h>
#include <linux/earlysuspend.h>
#include <linux/delay.h>
#include <linux/keyreset.h>
#include <linux/input.h>
#include <linux/nvhost.h>
#include <linux/pwm_backlight.h>
#include <linux/tegra_pwm_bl.h>

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
#include "tegra2_host1x_devices.h"

#define HDMI_HPD_GPIO 111

static struct regulator *olympus_dsi_reg = NULL;   			
static struct regulator *olympus_SW5 = NULL; 

static struct regulator *olympus_hdmi_reg = NULL;
static struct regulator *olympus_hdmi_pll = NULL;

static struct platform_tegra_pwm_backlight_data olympus_disp1_backlight_data = {
	.which_dc = 0,
	.which_pwm = 1,
	.gpio_conf_to_sfio = 177,
	.dft_brightness	= 77,
	.max_brightness	= 256,
	.period	= 0x1F,
	.clk_div = 3,
	.clk_select = 2,
};

static struct platform_device olympus_disp1_backlight_device = {
	.name	= "tegra-pwm-bl",
	.id	= -1,
	.dev	= {
		.platform_data = &olympus_disp1_backlight_data,
	},
};

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
		.pclk = 27000000,
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

static u8 qhd_smd_cmdF0[]={0xf0, 0x5a, 0x5a};
static u8 qhd_smd_cmdF1[]={0xf1, 0x5a, 0x5a};
static u8 qhd_smd_cmdD0[]={0xd0, 0x8e};
static u8 qhd_smd_cmdD2a_es4[]={0xd2, 0x04, 0x53};
static u8 qhd_smd_cmdD2b_es4[]={0xd2, 0x05, 0x53};
static u8 qhd_smd_cmd55_es2[]={0x55, 0x01};
static u8 qhd_smd_cmdB5_es4[]={0xb5, 0x03, 0x7f, 0x0a, 0x80, 0xff, 0x00};
static u8 qhd_smd_cmdB7_es4[]={0xb7, 0x7a, 0xf7, 0x4d, 0x91, 0x90, 0xb3, 0xff, 0x80, 0x6d, 0x01};
static u8 qhd_smd_cmdF4_es4[]={0xf4, 0x00, 0xbb, 0x46, 0x53, 0x0c, 0x49, 0x74, 0x29, 0x12, 0x15, 0x37, 0x37, 0x04};

static u8 qhd_smd_cmdF8_es4[]={0xf8, 0x0a, 0x04, 0x10, 0x2a, 0x35, 0x35, 0x35, 0x35, 0x21, 0x1a};
static u8 gamma_r_F9[]={0xf9, 0x04};
static u8 gamma_r_FA_es4[]={0xfa, 0x08, 0x1c, 0x1b, 0x0f, 0x0f, 0x0a, 0x1e, 0x22, 0x27, 0x26, 0x07, 0x0d};
static u8 gamma_r_FB_es4[]={0xfb, 0x08, 0x3c, 0x27, 0x0f, 0x0f, 0x0a, 0x1e, 0x26, 0x31, 0x2f, 0x07, 0x0b};
static u8 gamma_g_F9[]={0xf9, 0x02};
static u8 gamma_g_FA_es4[]={0xfa, 0x30, 0x14, 0x0f, 0x00, 0x06, 0x02, 0x1e, 0x22, 0x27, 0x27, 0x08, 0x10};
static u8 gamma_g_FB_es4[]={0xfb, 0x30, 0x35, 0x0f, 0x00, 0x0a, 0x02, 0x1c, 0x23, 0x31, 0x2f, 0x08, 0x0e};
static u8 gamma_b_F9[]={0xf9, 0x01};
static u8 gamma_b_FA_es4[]={0xfa, 0x12, 0x1b, 0x26, 0x0e, 0x12, 0x0b, 0x1e, 0x22, 0x27, 0x27, 0x06, 0x0c};
static u8 gamma_b_FB_es4[]={0xfb, 0x12, 0x3b, 0x2c, 0x12, 0x12, 0x0e, 0x1e, 0x26, 0x31, 0x2f, 0x06, 0x0d};
static u8 gamma_w_F9[]={0xf9, 0x20};
static u8 gamma_w_FA_es4[]={0xfa, 0x37, 0x1b, 0x09, 0x01, 0x06, 0x04, 0x19, 0x19, 0x22, 0x24, 0x04, 0x15};
static u8 gamma_w_FB_es4[]={0xfb, 0x37, 0x3b, 0x17, 0x01, 0x0a, 0x04, 0x19, 0x1d, 0x2c, 0x2c, 0x04, 0x13};
static u8 qhd_smd_cmd53_es2[]={0x53, 0x2c};
static u8 qhd_smd_cmd35[]={0x35, 0x00};
static u8 qhd_smd_cmdC3_es4[]={0xc3, 0x01, 0x4e};

static struct tegra_dsi_cmd dsi_olympus_init_cmd[]= {
	DSI_DLY_MS(10),
	DSI_CMD_LONG(0x39, qhd_smd_cmdF0),
	DSI_CMD_LONG(0x39, qhd_smd_cmdF1),
	DSI_CMD_LONG(0x39, qhd_smd_cmdD0),
	DSI_CMD_LONG(0x39, qhd_smd_cmdD2a_es4),
	DSI_CMD_LONG(0x39, qhd_smd_cmdD2b_es4),
	DSI_CMD_LONG(0x39, qhd_smd_cmd55_es2),
	
	DSI_CMD_SHORT(0x05, 0x11, 0x00),
	DSI_DLY_MS(120),

	DSI_CMD_LONG(0x39, qhd_smd_cmdB5_es4),
	DSI_CMD_LONG(0x39, qhd_smd_cmdB7_es4),
	DSI_CMD_LONG(0x39, qhd_smd_cmdF4_es4),
	DSI_CMD_LONG(0x39, qhd_smd_cmdF8_es4),
	DSI_CMD_LONG(0x39, gamma_r_F9),
	DSI_CMD_LONG(0x39, gamma_r_FA_es4),
	DSI_CMD_LONG(0x39, gamma_r_FB_es4),
	DSI_CMD_LONG(0x39, gamma_g_F9),
	DSI_CMD_LONG(0x39, gamma_g_FA_es4),
	DSI_CMD_LONG(0x39, gamma_g_FB_es4),
	DSI_CMD_LONG(0x39, gamma_b_F9),
	DSI_CMD_LONG(0x39, gamma_b_FA_es4),
	DSI_CMD_LONG(0x39, gamma_b_FB_es4),
	DSI_CMD_LONG(0x39, gamma_w_F9),
	DSI_CMD_LONG(0x39, gamma_w_FA_es4),
	DSI_CMD_LONG(0x39, gamma_w_FB_es4),
	DSI_CMD_LONG(0x39, qhd_smd_cmd53_es2),
	DSI_CMD_LONG(0x39, qhd_smd_cmd35),
	DSI_CMD_LONG(0x39, qhd_smd_cmdC3_es4),

	DSI_CMD_SHORT(0x05, 0x29, 0x00),
	DSI_DLY_MS(20),
};

/*
static struct tegra_dsi_cmd dsi_olympus_init_cmd[]= {
	DSI_CMD_SHORT(0x05, 0x11, 0x00),
	DSI_DLY_MS(150), 
	DSI_CMD_SHORT(0x05, 0x29, 0x00),
	DSI_DLY_MS(20), 
};
*/
static struct tegra_dsi_cmd dsi_suspend_cmd[] = {
	DSI_CMD_SHORT(0x05, 0x28, 0x00),
	DSI_DLY_MS(20), 
	DSI_CMD_SHORT(0x05, 0x10, 0x00),
	DSI_DLY_MS(120),
};

static int olympus_panel_enable(void)
{
	int ret;
	printk(KERN_INFO "pICS_%s: level 1...",__func__);

	if (!olympus_dsi_reg) {
		olympus_dsi_reg = regulator_get(NULL, "vcsi");
		if (IS_ERR_OR_NULL(olympus_dsi_reg)) {
			pr_err("Ninja: dsi: Could not get regulator vcsi\n");
				olympus_dsi_reg = NULL;
				return PTR_ERR(olympus_dsi_reg);
		}
		printk(KERN_INFO "pICS_%s: level 2...",__func__);
		ret = regulator_enable(olympus_dsi_reg);
		if (ret < 0) {
			printk(KERN_ERR
				"Ninja: DSI regulator vcsi could not be enabled\n");
			return ret;
		}
	}

	printk(KERN_INFO "pICS_%s: level 3...",__func__);
	if (!olympus_SW5) {
		olympus_SW5 = regulator_get(NULL, "sw5"); /* SW5 */
		if (IS_ERR_OR_NULL(olympus_SW5)) {
			pr_err("dsi: couldn't get regulator sw5\n");
			olympus_SW5 = NULL;
			return PTR_ERR(olympus_SW5);
		}
		printk(KERN_INFO "pICS_%s: level 4...",__func__);
        	ret = regulator_enable(olympus_SW5);
		if (ret < 0) {
			printk(KERN_ERR
				"Ninja: DSI regulator SW5 could not be enabled\n");
			return ret;
		}
        }
	printk(KERN_INFO "pICS_%s: level 5...",__func__);
	gpio_set_value(47,1);
	mdelay(5);
	gpio_set_value(35, 1);	

	return 0;
}

static int olympus_panel_disable(void)
{
	int ret;

	printk(KERN_INFO "pICS_%s: Starting...",__func__);
	gpio_set_value(35, 0);
	gpio_set_value(47, 0);	

	printk(KERN_INFO "pICS_%s: level 1...",__func__);
	
	if (olympus_dsi_reg) {
		ret = regulator_disable(olympus_dsi_reg);
		if (ret < 0) {
			printk(KERN_ERR
				"Ninja: DSI regulator vcsi could not be disabled\n");
			return ret;
		}
		regulator_put(olympus_dsi_reg);  
		olympus_dsi_reg = NULL;
	}
	printk(KERN_INFO "pICS_%s: level 2...",__func__);
	if (olympus_SW5) {
       		ret = regulator_disable(olympus_SW5);
		if (ret < 0) {
			printk(KERN_ERR
				"Ninja: DSI regulator SW5 could not be disabled\n");
			return ret;
		}
		regulator_put(olympus_SW5);
		olympus_dsi_reg = NULL;
	}
	printk(KERN_INFO "pICS_%s: level 3...",__func__);

	return 0;
}

static struct tegra_dsi_out olympus_dsi_out = {
	.n_data_lanes = 2,
	.pixel_format = TEGRA_DSI_PIXEL_FORMAT_24BIT_P, // 3
	.video_data_type = TEGRA_DSI_VIDEO_TYPE_VIDEO_MODE, //nn
	.video_burst_mode = TEGRA_DSI_VIDEO_NONE_BURST_MODE, //nn
//	.refresh_rate = 60,
	.refresh_rate = 64, //nn
//	.rated_refresh_rate = 0,
//	.panel_reset = 0,
	.panel_reset = 1, //nn
	.panel_reset_timeout_msec = 202, //nn
	.virtual_channel = TEGRA_DSI_VIRTUAL_CHANNEL_0, // 0
	.dsi_instance = 0,
	.chip_id = 0,
	.chip_rev = 0,
//	.panel_has_frame_buffer = 1,
//	.panel_has_frame_buffer = 0,  
//	.dsi_init_cmd = dsi_init_cmd, 	/*init cmd*/
//	.n_init_cmd = ARRAY_SIZE(dsi_init_cmd),
	.dsi_init_cmd = dsi_olympus_init_cmd, 	/*init cmd*/
	.n_init_cmd = ARRAY_SIZE(dsi_olympus_init_cmd),
	.dsi_suspend_cmd = dsi_suspend_cmd,
	.n_suspend_cmd = ARRAY_SIZE(dsi_suspend_cmd),		
	.video_data_type = TEGRA_DSI_VIDEO_TYPE_COMMAND_MODE,
	.video_clock_mode = TEGRA_DSI_VIDEO_CLOCK_TX_ONLY, //nn
//	.video_clock_mode = TEGRA_DSI_VIDEO_CLOCK_CONTINUOUS,
	.power_saving_suspend = 1, //nn
	.phy_timing = { //nn
		.t_hsdexit_ns = 6,
		.t_hstrail_ns = 7,
		.t_hsprepare_ns = 4,
		.t_datzero_ns = 9,
		.t_clktrail_ns = 4,
		.t_clkpost_ns = 10,
		.t_clkzero_ns = 13,
		//.t_tlpx_ns =  2,
	},
//	.lp_cmd_mode_freq_khz = 10000, 
	.lp_cmd_mode_freq_khz = 229500, //nn
	/* TODO: Get the vender recommended freq */
//	.lp_read_cmd_mode_freq_khz = 230000,
// 	.te_polarity_low = true,
};

static struct tegra_dc_out olympus_disp1_out = {
	.type		= TEGRA_DC_OUT_DSI, //2

	.align		= TEGRA_DC_ALIGN_MSB, //0
	.order		= TEGRA_DC_ORDER_RED_BLUE, //0

	.height		= 91, /* mm */
	.width 		= 51, /* mm */

	.modes 		= olympus_panel_modes,
	.n_modes 	= ARRAY_SIZE(olympus_panel_modes),

	.dsi		= &olympus_dsi_out,

	.enable		= &olympus_panel_enable,
	.disable	= &olympus_panel_disable,
};

static struct tegra_fb_data olympus_fb_data = {
	.win		= 0,
	.xres		= 540,
	.yres		= 960,
	.bits_per_pixel	= 32,
	.flags		= TEGRA_FB_FLIP_ON_PROBE,
};

static struct tegra_dc_platform_data olympus_disp1_pdata = {
	.flags		= TEGRA_DC_FLAG_ENABLED,
//	.emc_clk_rate	= 300000000,
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

static int olympus_panel_setup_dc(void)
{
	tegra_gpio_enable(47);	
	gpio_request(47, "disp_5v_en");
	gpio_direction_output(47, 0);

	tegra_gpio_enable(35);
	gpio_request(35, "disp_reset_n");
	gpio_direction_output(35, 0);

	tegra_gpio_enable(46);
	gpio_request(46, "hdmi_5v_en");
	gpio_direction_output(47, 1);

	return 0;
}

static int olympus_hdmi_enable(void)
{
	if (!olympus_hdmi_reg) {
		olympus_hdmi_reg = regulator_get(NULL, "vhdmi"); /* LD011 */
		if (IS_ERR_OR_NULL(olympus_hdmi_reg)) {
			pr_err("hdmi: couldn't get regulator vhdmi\n");
			olympus_hdmi_reg = NULL;
			return PTR_ERR(olympus_hdmi_reg);
		}
	}
	regulator_enable(olympus_hdmi_reg);

	/* Need to also change avdd_hdmi_pll regulator */
	if (!olympus_hdmi_pll) {
		olympus_hdmi_pll = regulator_get(NULL, "avdd_hdmi_pll"); /* LD06 */
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
	if (olympus_hdmi_reg)
		regulator_disable(olympus_hdmi_reg);
	if (olympus_hdmi_pll)
		regulator_disable(olympus_hdmi_pll);
	return 0;
}

static struct tegra_dc_out olympus_disp2_out = {
	.type = TEGRA_DC_OUT_HDMI,
	.flags = TEGRA_DC_OUT_HOTPLUG_HIGH,

	.dcc_bus = 1,
	.hotplug_gpio = TEGRA_GPIO_PN7,

	.max_pixclock	= KHZ2PICOS(148500),

	.align = TEGRA_DC_ALIGN_MSB,
	.order = TEGRA_DC_ORDER_RED_BLUE,

	.enable		= olympus_hdmi_enable,
	.disable	= olympus_hdmi_disable,
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
#if defined(CONFIG_TEGRA_NVMAP)
	&olympus_nvmap_device,
#endif
	&olympus_disp1_backlight_device,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
/* put early_suspend/late_resume handlers here for the display in order
 * to keep the code out of the display driver, keeping it closer to upstream
 */
struct early_suspend olympus_panel_early_suspender;

static void olympus_panel_early_suspend(struct early_suspend *h)
{
	int i;
	for (i = 0; i < num_registered_fb; i++)
		fb_blank(registered_fb[i], FB_BLANK_POWERDOWN);
#ifdef CONFIG_TEGRA_CONVSERVATIVE_GOV_ON_EARLYSUPSEND
	cpufreq_save_default_governor();
	cpufreq_set_conservative_governor();
	cpufreq_set_conservative_governor_param(
		SET_CONSERVATIVE_GOVERNOR_UP_THRESHOLD,
		SET_CONSERVATIVE_GOVERNOR_DOWN_THRESHOLD);
#endif
}

static void olympus_panel_late_resume(struct early_suspend *h)
{
	int i;
#ifdef CONFIG_TEGRA_CONVSERVATIVE_GOV_ON_EARLYSUPSEND
	cpufreq_restore_default_governor();
#endif
	for (i = 0; i < num_registered_fb; i++)
		fb_blank(registered_fb[i], FB_BLANK_UNBLANK);
}
#endif

int __init olympus_panel_init(void)
{
	struct resource *res;
	int err;

	//printk(KERN_INFO "pICS_%s: Starting...",__func__);

	tegra_gpio_enable(111);
	gpio_request(111, "hdmi_hpd");
	gpio_direction_input(111);

	olympus_panel_setup_dc();

#ifdef CONFIG_HAS_EARLYSUSPEND
	olympus_panel_early_suspender.suspend = olympus_panel_early_suspend;
	olympus_panel_early_suspender.resume = olympus_panel_late_resume;
	olympus_panel_early_suspender.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	register_early_suspend(&olympus_panel_early_suspender);
#endif

#if defined(CONFIG_TEGRA_NVMAP)
	olympus_carveouts[1].base = tegra_carveout_start;
	olympus_carveouts[1].size = tegra_carveout_size;
#endif

#ifdef CONFIG_TEGRA_GRHOST
	err = tegra2_register_host1x_devices();
	if (err)
		return err;
#endif

	err = platform_add_devices(olympus_gfx_devices,
				   ARRAY_SIZE(olympus_gfx_devices));

#if defined(CONFIG_TEGRA_GRHOST) && defined(CONFIG_TEGRA_DC)
	res = nvhost_get_resource_byname(&olympus_disp1_device,
		IORESOURCE_MEM, "fbmem");
	res->start = tegra_fb_start;
	res->end = tegra_fb_start + tegra_fb_size - 1;

	if (tegra_bootloader_fb_start)
		tegra_move_framebuffer(tegra_fb_start,tegra_bootloader_fb_start,
			min(tegra_fb_size, tegra_bootloader_fb_size));

	res = nvhost_get_resource_byname(&olympus_disp2_device,
		IORESOURCE_MEM, "fbmem");
	res->start = tegra_fb2_start;
	res->end = tegra_fb2_start + tegra_fb2_size - 1;

	if (!err)
		err = nvhost_device_register(&olympus_disp1_device);

	if (!err)
		err = nvhost_device_register(&olympus_disp2_device);
#endif

	//printk(KERN_INFO "pICS_%s: Ending...",__func__);

	return err;

}



