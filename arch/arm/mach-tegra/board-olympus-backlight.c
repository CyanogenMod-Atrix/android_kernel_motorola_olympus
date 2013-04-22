/*
 * arch/arm/mach-tegra/board-olympus-backlight.c
 *
 * ...
 *
 * Copyright (c) 2009-2013, ...
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

#include <linux/resource.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/i2c-tegra.h>
#include <asm/mach-types.h>
#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/pinmux.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/leds-lm3530.h>
#include <linux/leds-lm3532.h>
#include <linux/isl29030.h>

#include "board-olympus.h"
#include "gpio-names.h"
#include "devices.h"
#include "board.h"
#include "hwrev.h"

#define TEGRA_BACKLIGHT_EN_GPIO 	32 /* TEGRA_GPIO_P?? */
#define TEGRA_KEY_BACKLIGHT_EN_GPIO 	47 /* TEGRA_GPIO_P?? */

static int disp_backlight_init(void)
{
    int ret;

	tegra_gpio_enable(TEGRA_BACKLIGHT_EN_GPIO);
	tegra_gpio_enable(TEGRA_KEY_BACKLIGHT_EN_GPIO);

	if ((ret = gpio_request(TEGRA_BACKLIGHT_EN_GPIO, "backlight_en"))) {
		pr_err("%s: gpio_request(%d, backlight_en) failed: %d\n",
			__func__, TEGRA_BACKLIGHT_EN_GPIO, ret);
		return ret;
	} else {
		pr_info("%s: gpio_request(%d, backlight_en) success!\n",
			__func__, TEGRA_BACKLIGHT_EN_GPIO);
	}
	if ((ret = gpio_direction_output(TEGRA_BACKLIGHT_EN_GPIO, 1))) {
		pr_err("%s: gpio_direction_output(backlight_en) failed: %d\n",
			__func__, ret);
		return ret;
	}
	if ((ret = gpio_request(TEGRA_KEY_BACKLIGHT_EN_GPIO,
			"key_backlight_en"))) {
		pr_err("%s: gpio_request(%d, key_backlight_en) failed: %d\n",
			__func__, TEGRA_KEY_BACKLIGHT_EN_GPIO, ret);
		return ret;
	} else {
		pr_info("%s: gpio_request(%d, key_backlight_en) success!\n",
			__func__, TEGRA_KEY_BACKLIGHT_EN_GPIO);
	}
	if ((ret = gpio_direction_output(TEGRA_KEY_BACKLIGHT_EN_GPIO, 1))) {
		pr_err("%s: gpio_direction_output(key_backlight_en) failed: %d\n",
			__func__, ret);
		return ret;
	}

    return 0;
}

static int disp_backlight_power_on(void)
{
    pr_info("%s: display backlight is powered on\n", __func__);
    gpio_set_value(TEGRA_BACKLIGHT_EN_GPIO, 1);
    gpio_set_value(TEGRA_KEY_BACKLIGHT_EN_GPIO, 1);
    return 0;
}

static int disp_backlight_power_off(void)
{
    pr_info("%s: display backlight is powered off\n", __func__);
    gpio_set_value(TEGRA_BACKLIGHT_EN_GPIO, 0);
    gpio_set_value(TEGRA_KEY_BACKLIGHT_EN_GPIO, 0);
    return 0;
}

struct lm3530_platform_data lm3530_pdata = {
    .init = disp_backlight_init,
    .power_on = disp_backlight_power_on,
    .power_off = disp_backlight_power_off,

    .ramp_time = 0,   /* Ramp time in milliseconds */
    .gen_config = 
        LM3530_26mA_FS_CURRENT | LM3530_LINEAR_MAPPING | LM3530_I2C_ENABLE,
    .als_config = 0,  /* We don't use ALS from this chip */
};

struct lm3532_platform_data lm3532_pdata = {
    .flags = LM3532_CONFIG_BUTTON_BL | LM3532_HAS_WEBTOP,
    .init = disp_backlight_init,
    .power_on = disp_backlight_power_on,
    .power_off = disp_backlight_power_off,

    .ramp_time = 0,   /* Ramp time in milliseconds */
    .ctrl_a_fs_current = LM3532_26p6mA_FS_CURRENT,
    .ctrl_b_fs_current = LM3532_8p2mA_FS_CURRENT,
    .ctrl_a_mapping_mode = LM3532_LINEAR_MAPPING,
    .ctrl_b_mapping_mode = LM3532_LINEAR_MAPPING,
	.ctrl_a_pwm = 0x82,
};

extern int MotorolaBootDispArgGet(unsigned int *arg);

static struct i2c_board_info olympus_i2c1_backlight_info[] = {
	 { /* Display backlight */
		I2C_BOARD_INFO(LM3532_NAME, LM3532_I2C_ADDR),
		.platform_data = &lm3532_pdata,
		/*.irq = ..., */
	},
};

void __init olympus_backlight_init(void)
{
	unsigned int disp_type = 0;
	int ret;

#ifdef CONFIG_LEDS_DISP_BTN_TIED
	lm3532_pdata.flags |= LM3532_DISP_BTN_TIED;
#endif
	if ((ret = MotorolaBootDispArgGet(&disp_type))) {
		pr_err("\n%s: unable to read display type: %d\n", __func__, ret);
		return;
	}
	if (disp_type & 0x100) {
		pr_info("\n%s: 0x%x ES2 display; will enable PWM in LM3532\n",
			__func__, disp_type);
		lm3532_pdata.ctrl_a_pwm = 0x86;
	} else {
		pr_info("\n%s: 0x%x ES1 display; will NOT enable PWM in LM3532\n",
			__func__, disp_type);
	}

	printk("%s: registering i2c1 device... backlight\n", __func__);
	printk("bus 0: %d device\n", 1);
	i2c_register_board_info(0, olympus_i2c1_backlight_info, 1);
}

