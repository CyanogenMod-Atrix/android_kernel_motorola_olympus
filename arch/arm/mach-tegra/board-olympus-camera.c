#include <linux/delay.h>
#include <linux/i2c.h>
#include <mach/gpio.h>
#include <media/ov5650.h>
#include <media/dw9714l.h>
#include <media/soc380.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <asm/mach-types.h>
#include <linux/gpio.h>
#include <linux/leds-lm3554.h>
#include "gpio-names.h"
#include "cpu-tegra.h"
#include "board-olympus.h"

#define POWER_ID_SENSOR		0x1
#define POWER_ID_FOCUSER	0x2
#define POWER_ID_FLASH		0x4

static struct i2c_board_info *devices = NULL;
static unsigned ndevices = 0;

struct cam_pwr_ctl {
	unsigned rs_gpio;
	unsigned rs_on;
	unsigned pd_gpio;
	unsigned pd_on;
	unsigned flash_rs;
	unsigned flash_on;
	unsigned pwr_mask;
	struct mutex pwr_lock;
};

static struct cam_pwr_ctl front_cam;
static struct cam_pwr_ctl rear_cam;

static struct regulator *reg_avdd_cam1; /* LDO9 */
static struct regulator *reg_vdd_mipi;  /* LDO17 */

static int olympus_camera_sensors_init(void)
{
	if (rear_cam.pd_gpio) {
		tegra_gpio_enable(rear_cam.pd_gpio);
		gpio_request(rear_cam.pd_gpio, "camera1_powerdown");
		gpio_direction_output(rear_cam.pd_gpio, rear_cam.pd_on);
		gpio_export(rear_cam.pd_gpio, false);
	}

	if (rear_cam.rs_gpio) {
		tegra_gpio_enable(rear_cam.rs_gpio);
		gpio_request(rear_cam.rs_gpio, "camera1_reset");
		gpio_direction_output(rear_cam.rs_gpio, !rear_cam.rs_on);
		gpio_export(rear_cam.rs_gpio, false);
	}

	if (front_cam.pd_gpio) {
		tegra_gpio_enable(front_cam.pd_gpio);
		gpio_request(front_cam.pd_gpio, "camera2_powerdown");
		gpio_direction_output(front_cam.pd_gpio, !front_cam.pd_on);
		gpio_export(front_cam.pd_gpio, false);
	}

	if (front_cam.rs_gpio) {
		tegra_gpio_enable(front_cam.rs_gpio);
		gpio_request(front_cam.rs_gpio, "camera2_reset");
		gpio_direction_output(front_cam.rs_gpio, !front_cam.rs_on);
		gpio_export(front_cam.rs_gpio, false);
	}

	if (rear_cam.flash_rs) {
		tegra_gpio_enable(rear_cam.flash_rs);
		gpio_request(rear_cam.flash_rs, "camera_rear_cam.flash_rs");
		gpio_direction_output(rear_cam.flash_rs, rear_cam.flash_on);
		gpio_export(rear_cam.flash_rs, false);
	}

	mutex_init(&rear_cam.pwr_lock);
	mutex_init(&front_cam.pwr_lock);

	if (rear_cam.pd_gpio) {
		gpio_set_value(rear_cam.pd_gpio, !rear_cam.pd_on);
	}

	return 0;
}

static int olympus_rear_cam_power_on(unsigned power_id)
{

	pr_info("%s: (mask=%x) ++++\n", __func__, rear_cam.pwr_mask);

	mutex_lock(&rear_cam.pwr_lock);

	if (!rear_cam.pwr_mask) {

		if (!reg_avdd_cam1) {
			reg_avdd_cam1 = regulator_get(NULL, "vcsi");
			if (IS_ERR_OR_NULL(reg_avdd_cam1)) {
				pr_err("%s: vcsi failed\n", __func__);
				reg_avdd_cam1 = NULL;
				return PTR_ERR(reg_avdd_cam1);
			}
			regulator_enable(reg_avdd_cam1);
			mdelay(20);
			pr_info("%s: regulator enabled(vcsi)\n", __func__);
		}

		if (!reg_vdd_mipi) {
			reg_vdd_mipi = regulator_get(NULL, "vcam");
			if (IS_ERR_OR_NULL(reg_vdd_mipi)) {
				pr_err("%s: vcam failed\n", __func__);
				reg_vdd_mipi = NULL;
				return PTR_ERR(reg_vdd_mipi);
			}
			regulator_enable(reg_vdd_mipi);
			mdelay(5);
			pr_info("%s: regulator enabled(vcam)\n", __func__);
		}

		if (rear_cam.pd_gpio) {
			gpio_set_value(rear_cam.pd_gpio, rear_cam.pd_on);
			mdelay(5);
			pr_info("%s: gpio %u set %u\n", __func__, rear_cam.pd_gpio, rear_cam.pd_on);
		}

		if (rear_cam.rs_gpio) {
			gpio_set_value(rear_cam.rs_gpio, rear_cam.rs_on);
			mdelay(10);
			gpio_set_value(rear_cam.rs_gpio, !rear_cam.rs_on);
			mdelay(5);
			gpio_set_value(rear_cam.rs_gpio, rear_cam.rs_on);
			mdelay(20);
		}
	} else {
		pr_info("%s: pwr_mask=%x, skipping\n", __func__, rear_cam.pwr_mask);
	}

	rear_cam.pwr_mask |= power_id;
	mutex_unlock(&rear_cam.pwr_lock);

	pr_info("%s: (mask=%x) ----\n", __func__, rear_cam.pwr_mask);

	return 0;
}

static int olympus_rear_cam_power_off(unsigned power_id)
{
	pr_info("%s: (mask=%x) ++++\n", __func__, rear_cam.pwr_mask);

	mutex_lock(&rear_cam.pwr_lock);
	rear_cam.pwr_mask &= ~power_id;

	if (!rear_cam.pwr_mask) {
		if (rear_cam.pd_gpio)
			gpio_set_value(rear_cam.pd_gpio, !rear_cam.pd_on);
		if (rear_cam.rs_gpio)
			gpio_set_value(rear_cam.rs_gpio, !rear_cam.rs_on);

		if (reg_avdd_cam1) {
			regulator_disable(reg_avdd_cam1);
			regulator_put(reg_avdd_cam1);
			reg_avdd_cam1 = NULL;
		}

		if (reg_vdd_mipi) {
			regulator_disable(reg_vdd_mipi);
			regulator_put(reg_vdd_mipi);
			reg_vdd_mipi = NULL;
		}
	} else {
		pr_info("%s: pwr_mask=%x, skipping\n", __func__, rear_cam.pwr_mask);
	}

	mutex_unlock(&rear_cam.pwr_lock);

	pr_info("%s: (mask=%x) ----\n", __func__, rear_cam.pwr_mask);
	return 0;
}

static int olympus_front_cam_power_on(void)
{
	if (!reg_avdd_cam1) {
		reg_avdd_cam1 = regulator_get(NULL, "vdd_cam1");
		if (IS_ERR_OR_NULL(reg_avdd_cam1)) {
			pr_err("%s: vdd_cam1 failed\n", __func__);
			reg_avdd_cam1 = NULL;
			return PTR_ERR(reg_avdd_cam1);
		}
		regulator_enable(reg_avdd_cam1);
		mdelay(20);
		pr_info("%s: regulator enabled(reg_avdd_cam1)\n", __func__);
	}

	if (!reg_vdd_mipi) {
		reg_vdd_mipi = regulator_get(NULL, "vcam");
		if (IS_ERR_OR_NULL(reg_vdd_mipi)) {
			pr_err("%s: vcam failed\n", __func__);
			reg_vdd_mipi = NULL;
			return PTR_ERR(reg_vdd_mipi);
		}
		regulator_enable(reg_vdd_mipi);
		mdelay(5);
		pr_info("%s: regulator enabled(vcam)\n", __func__);
	}

	if (front_cam.pd_gpio) {
		gpio_set_value(front_cam.pd_gpio, front_cam.pd_on);
		mdelay(5);
		pr_info("%s: gpio %u set %u\n", __func__, front_cam.pd_gpio, front_cam.pd_on);
	}

	if (front_cam.rs_gpio) {
		gpio_set_value(front_cam.rs_gpio, front_cam.rs_on);
		mdelay(10);
		gpio_set_value(front_cam.rs_gpio, !front_cam.rs_on);
		mdelay(5);
		gpio_set_value(front_cam.rs_gpio, front_cam.rs_on);
		mdelay(20);
	}

	return 0;
}

static int olympus_front_cam_power_off(void)
{
	if (front_cam.pd_gpio)
		gpio_set_value(front_cam.pd_gpio, !front_cam.pd_on);
	if (front_cam.rs_gpio)
		gpio_set_value(front_cam.rs_gpio, !front_cam.rs_on);

	if (reg_vdd_mipi) {
		regulator_disable(reg_vdd_mipi);
		regulator_put(reg_vdd_mipi);
		reg_vdd_mipi = NULL;
	}

	if (reg_avdd_cam1) {
		regulator_disable(reg_avdd_cam1);
		regulator_put(reg_avdd_cam1);
		reg_avdd_cam1 = NULL;
	}

	return 0;
}

struct ov5650_platform_data olympus_ov5650_data = {
	.power_on = olympus_rear_cam_power_on,
	.power_off = olympus_rear_cam_power_off,
	.power_id = POWER_ID_SENSOR,
};

struct soc380_platform_data olympus_soc380_data = {
	.power_on = olympus_front_cam_power_on,
	.power_off = olympus_front_cam_power_off,
};

struct dw9714l_platform_data focuser_dw9714l_pdata = {
	.power_id = POWER_ID_FOCUSER,
	.power_on = olympus_rear_cam_power_on,
	.power_off = olympus_rear_cam_power_off,
};

static struct lm3554_platform_data flash_lm3554_data = {
	.flags	= 0x1,
	.torch_brightness_def = 0xa0,
	.flash_brightness_def = 0x78,
	.flash_duration_def = 0x28,
	.config_reg_1_def = 0xe0,
	.config_reg_2_def = 0xf0,
	.vin_monitor_def = 0x01,
	.gpio_reg_def = 0x0,
/*
	.power_id = POWER_ID_FLASH,
	.power_on = olympus_rear_cam_power_on,
	.power_off = olympus_rear_cam_power_off,
*/
};

static struct i2c_board_info olympus_i2c_cam_board_info[] = {
	{	/* rear imager */
		I2C_BOARD_INFO("ov5650", 0x36),		// 0x6C
		.platform_data = &olympus_ov5650_data,
	},
	{	/* rear AF module */
		I2C_BOARD_INFO("dw9714l", 0x0C),	// 0x18
		.platform_data = &focuser_dw9714l_pdata,
	},
	{	/* front imager */
		I2C_BOARD_INFO("soc380", 0x3D),		// 0x7A
		.platform_data = &olympus_soc380_data,
	},
	{	/* Flash LED */
		I2C_BOARD_INFO(LM3554_NAME, 0x53), // 0xA6
		.platform_data = &flash_lm3554_data,
	},
};

static void olympus_camera_set(void)
{
		devices = olympus_i2c_cam_board_info;
		ndevices = ARRAY_SIZE(olympus_i2c_cam_board_info);

		front_cam.pd_gpio 	= TEGRA_GPIO_PBB5;
		front_cam.pd_on 	= 0;
		front_cam.rs_gpio 	= TEGRA_GPIO_PL4;
		front_cam.rs_on 	= 1;

		rear_cam.pd_gpio 	= TEGRA_GPIO_PBB1;
		rear_cam.pd_on 		= 0;
		rear_cam.rs_gpio 	= TEGRA_GPIO_PD2;
		rear_cam.rs_on 		= 1;
		rear_cam.flash_rs	= TEGRA_GPIO_PT3; // BB4 on old Olympus
		rear_cam.flash_on	= 1;
}

void __init olympus_camera_init(void)
{
	olympus_camera_set();
	olympus_camera_sensors_init();
	if (ndevices)
		i2c_register_board_info(2, devices, ndevices);

}

