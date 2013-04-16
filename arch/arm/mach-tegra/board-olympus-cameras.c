#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <media/ov5650.h>
#include <media/dw9714l.h>
#include <media/soc380.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <asm/mach-types.h>
#include <asm/bootinfo.h>

#include "gpio-names.h"
#include "board-olympus.h"
#include "cpu-tegra.h"

#define POWER_ID_FOCUSER	0x2

#define CAMERA1_PWDN_GPIO		TEGRA_GPIO_PBB1
#define CAMERA1_RESET_GPIO		TEGRA_GPIO_PD2
#define CAMERA2_PWDN_GPIO		TEGRA_GPIO_PBB5
#define CAMERA2_RESET_GPIO		TEGRA_GPIO_PL4
//#define CAMERA_AF_PD_GPIO		TEGRA_GPIO_PT3
#define CAMERA_FLASH_EN1_GPIO		TEGRA_GPIO_PT3
//#define CAMERA_FLASH_EN2_GPIO		TEGRA_GPIO_PA0

static struct regulator *reg_avdd_cam1;
static struct regulator *reg_vdd_af; 
static struct regulator *reg_vdd_mipi;
static struct regulator *reg_vddio_vi;

static struct mutex cam1_pwr_lock;
static struct mutex cam2_pwr_lock;

static int camera_init(void)
{
	gpio_request(CAMERA1_PWDN_GPIO, "camera1_powerdown");
	gpio_direction_output(CAMERA1_PWDN_GPIO, 0);
	gpio_export(CAMERA1_PWDN_GPIO, false);

	gpio_request(CAMERA1_RESET_GPIO, "camera1_reset");
	gpio_direction_output(CAMERA1_RESET_GPIO, 0);
	gpio_export(CAMERA1_RESET_GPIO, false);

	gpio_request(CAMERA2_PWDN_GPIO, "camera2_powerdown");
	gpio_direction_output(CAMERA2_PWDN_GPIO, 0);
	gpio_export(CAMERA2_PWDN_GPIO, false);

	gpio_request(CAMERA2_RESET_GPIO, "camera2_reset");
	gpio_direction_output(CAMERA2_RESET_GPIO, 0);
	gpio_export(CAMERA2_RESET_GPIO, false);

/*	gpio_request(CAMERA_AF_PD_GPIO, "camera_autofocus");
	gpio_direction_output(CAMERA_AF_PD_GPIO, 0);
	gpio_export(CAMERA_AF_PD_GPIO, false);*/

	gpio_request(CAMERA_FLASH_EN1_GPIO, "camera_flash_en1");
	gpio_direction_output(CAMERA_FLASH_EN1_GPIO, 0);
	gpio_export(CAMERA_FLASH_EN1_GPIO, false);

/*	gpio_request(CAMERA_FLASH_EN2_GPIO, "camera_flash_en2");
	gpio_direction_output(CAMERA_FLASH_EN2_GPIO, 0);
	gpio_export(CAMERA_FLASH_EN2_GPIO, false);*/

	gpio_set_value(CAMERA1_PWDN_GPIO, 1);
	mdelay(5);

	mutex_init(&cam1_pwr_lock);
	mutex_init(&cam2_pwr_lock);

	return 0;
}

static int olympus_ov5650_power_on(void)
{

	mutex_lock(&cam1_pwr_lock);

	gpio_set_value(CAMERA1_PWDN_GPIO, 0);

	if (!reg_avdd_cam1) {
		reg_avdd_cam1 = regulator_get(NULL, "vcsi");
		if (IS_ERR_OR_NULL(reg_avdd_cam1)) {
			pr_err("olympus_ov5650_power_on: reg_avdd_cam1 failed\n");
			reg_avdd_cam1 = NULL;
			return PTR_ERR(reg_avdd_cam1);
		}
		regulator_enable(reg_avdd_cam1);
	}
	mdelay(5);

	if (!reg_vdd_mipi) {
		reg_vdd_mipi = regulator_get(NULL, "vcam");
		if (IS_ERR_OR_NULL(reg_vdd_mipi)) {
			pr_err("olympus_ov5650_power_on: vddio_mipi failed\n");
			reg_vdd_mipi = NULL;
			return PTR_ERR(reg_vdd_mipi);
		}
		regulator_enable(reg_vdd_mipi);
	}
	mdelay(5);

/*	if (!reg_vdd_af) {
		reg_vdd_af = regulator_get(NULL, "v28_af_baf");
		if (IS_ERR_OR_NULL(reg_vdd_af)) {
			pr_err("olympus_ov5650_power_on: vdd_vcore_af failed\n");
			reg_vdd_af = NULL;
			return PTR_ERR(reg_vdd_af);
		}
		regulator_enable(reg_vdd_af);
	}
	mdelay(5);*/

	gpio_set_value(CAMERA1_PWDN_GPIO, 1);
	mdelay(5);
	gpio_set_value(CAMERA1_RESET_GPIO, 1);
	mdelay(10);
	gpio_set_value(CAMERA1_RESET_GPIO, 0);
	mdelay(5);
	gpio_set_value(CAMERA1_RESET_GPIO, 1);
	mdelay(20);
//	gpio_set_value(CAMERA_AF_PD_GPIO, 1);

	mutex_unlock(&cam1_pwr_lock);
	return 0;
}

static int olympus_ov5650_power_off(void)
{
	mutex_lock(&cam1_pwr_lock);

//	gpio_set_value(CAMERA_AF_PD_GPIO, 0);
	gpio_set_value(CAMERA1_PWDN_GPIO, 1);
	gpio_set_value(CAMERA1_RESET_GPIO, 0);

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

/*	if (reg_vdd_af) {
		regulator_disable(reg_vdd_af);
		regulator_put(reg_vdd_af);
		reg_vdd_af = NULL;
	}*/

	mutex_unlock(&cam1_pwr_lock);

	return 0;
}

static int olympus_soc380_power_on(void)
{
	mutex_lock(&cam2_pwr_lock);

	gpio_set_value(CAMERA2_PWDN_GPIO, 0);

	if (!reg_vddio_vi) {
		reg_vddio_vi = regulator_get(NULL, "vdd_cam1");
		if (IS_ERR_OR_NULL(reg_vddio_vi)) {
			pr_err("olympus_soc380_power_on: vddio_vi failed\n");
			reg_vddio_vi = NULL;
			return PTR_ERR(reg_vddio_vi);
		}
		regulator_set_voltage(reg_vddio_vi, 1800*1000, 1800*1000);
		mdelay(5);
		regulator_enable(reg_vddio_vi);
	}

	if (!reg_avdd_cam1) {
		reg_avdd_cam1 = regulator_get(NULL, "vcam");
		if (IS_ERR_OR_NULL(reg_avdd_cam1)) {
			pr_err("olympus_soc380_power_on: vdd_cam1 failed\n");
			reg_avdd_cam1 = NULL;
			return PTR_ERR(reg_avdd_cam1);
		}
		regulator_enable(reg_avdd_cam1);
	}
	mdelay(5);

	gpio_set_value(CAMERA2_RESET_GPIO, 1);
	mdelay(10);
	gpio_set_value(CAMERA2_RESET_GPIO, 0);
	mdelay(5);
	gpio_set_value(CAMERA2_RESET_GPIO, 1);
	mdelay(20);

	mutex_unlock(&cam2_pwr_lock);
	return 0;

}

static int olympus_soc380_power_off(void)
{
	mutex_lock(&cam2_pwr_lock);

	gpio_set_value(CAMERA2_PWDN_GPIO, 1);
	gpio_set_value(CAMERA2_RESET_GPIO, 0);

	if (reg_avdd_cam1) {
		regulator_disable(reg_avdd_cam1);
		regulator_put(reg_avdd_cam1);
		reg_avdd_cam1 = NULL;
	}
	if (reg_vddio_vi) {
		regulator_disable(reg_vddio_vi);
		regulator_put(reg_vddio_vi);
		reg_vddio_vi = NULL;
	}

	mutex_unlock(&cam2_pwr_lock);

	return 0;
}

struct ov5650_platform_data olympus_ov5650_data = {
	.power_on = olympus_ov5650_power_on,
	.power_off = olympus_ov5650_power_off,
};

struct soc380_platform_data olympus_soc380_data = {
	.power_on = olympus_soc380_power_on,
	.power_off = olympus_soc380_power_off,
};

struct dw9714l_platform_data focuser_dw9714l_pdata = {
        .power_id = POWER_ID_FOCUSER,
        //.power_on = olympus_rear_cam_power_on,
        //.power_off = olympus_rear_cam_power_off,
};

static struct i2c_board_info olympus_i2c3_board_info[] = {
	{
		I2C_BOARD_INFO("ov5650", 0x36),
		.platform_data = &olympus_ov5650_data,
	},
	//{       /* rear AF module */
        //        I2C_BOARD_INFO("dw9714l", 0x0C),        // 0x18
        //        .platform_data = &focuser_dw9714l_pdata,
        //},
	{
		I2C_BOARD_INFO("soc380", 0x3D),   //0x3c
		.platform_data = &olympus_soc380_data,
	},
};

void __init olympus_cameras_init(void)
{
	camera_init();
	
	printk("bus 2: %d devices\n", ARRAY_SIZE(olympus_i2c3_board_info));

	i2c_register_board_info(2, olympus_i2c3_board_info,
				ARRAY_SIZE(olympus_i2c3_board_info));

}

