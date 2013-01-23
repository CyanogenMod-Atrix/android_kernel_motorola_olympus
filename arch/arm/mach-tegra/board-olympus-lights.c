#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/io.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/setup.h>

#include <mach/iomap.h>
#include <mach/irqs.h>
#include <linux/gpio.h>
#include <linux/leds-lm3530.h>
#include <linux/leds-lm3532.h>

#include "gpio-names.h"
#include "board.h"
#include "hwrev.h"

#include "board-olympus.h"

#define TEGRA_BACKLIGHT_EN_GPIO 32 /* TEGRA_GPIO_PE0 */
#define TEGRA_KEY_BACKLIGHT_EN_GPIO 47 /* TEGRA_GPIO_PE0 */

static int disp_backlight_init(void)
{
    int ret;
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
    return 0;
}

static int disp_backlight_power_off(void)
{
    pr_info("%s: display backlight is powered off\n", __func__);
    gpio_set_value(TEGRA_BACKLIGHT_EN_GPIO, 0);
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

void mot_setup_lights(struct i2c_board_info *info)
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
}
