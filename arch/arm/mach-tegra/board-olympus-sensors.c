#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/akm8975.h>
#include <linux/adt7461.h>
#include <linux/isl29030.h>
#include <linux/bu52014hfv.h>
#include <linux/kxtf9.h>
#include <media/ov5650.h>
#include <media/soc380.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/vib-gpio.h>
#include <asm/mach-types.h>
#include <asm/bootinfo.h>

#include "gpio-names.h"
#include "board-olympus.h"
#include "cpu-tegra.h"

#define TEGRA_PROX_INT_GPIO		TEGRA_GPIO_PE1
#define TEGRA_HF_NORTH_GPIO		TEGRA_GPIO_PS2
#define TEGRA_HF_SOUTH_GPIO		TEGRA_GPIO_PS0
#define TEGRA_HF_KICKSTAND_GPIO		TEGRA_GPIO_PW3
#define TEGRA_VIBRATOR_GPIO		TEGRA_GPIO_PD0
#define TEGRA_KXTF9_INT_GPIO		TEGRA_GPIO_PV3
#define TEGRA_L3G4200D_IRQ_GPIO		TEGRA_GPIO_PH2
#define TEGRA_AKM8975_IRQ_GPIO		TEGRA_GPIO_PE2
#define TEGRA_AKM8975_RESET_GPIO	TEGRA_GPIO_PK5
#define TEGRA_ADT7461_IRQ_GPIO		TEGRA_GPIO_PE5
#define PWRUP_BAREBOARD            	0x00100000 /* Bit 20 */

#define CAMERA1_PWDN_GPIO		TEGRA_GPIO_PBB1
#define CAMERA1_RESET_GPIO		TEGRA_GPIO_PD2
#define CAMERA2_PWDN_GPIO		TEGRA_GPIO_PBB5
#define CAMERA2_RESET_GPIO		TEGRA_GPIO_PL4
#define CAMERA_AF_PD_GPIO		TEGRA_GPIO_PT3
#define CAMERA_FLASH_EN1_GPIO		TEGRA_GPIO_PBB4
//#define CAMERA_FLASH_EN2_GPIO		TEGRA_GPIO_PA0

static struct regulator *reg_avdd_cam1;
static struct regulator *reg_vdd_af; 
static struct regulator *reg_vdd_mipi;
static struct regulator *reg_vddio_vi;

static int olympus_camera_init(void)
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

	gpio_request(CAMERA_AF_PD_GPIO, "camera_autofocus");
	gpio_direction_output(CAMERA_AF_PD_GPIO, 0);
	gpio_export(CAMERA_AF_PD_GPIO, false);

	gpio_request(CAMERA_FLASH_EN1_GPIO, "camera_flash_en1");
	gpio_direction_output(CAMERA_FLASH_EN1_GPIO, 0);
	gpio_export(CAMERA_FLASH_EN1_GPIO, false);

/*	gpio_request(CAMERA_FLASH_EN2_GPIO, "camera_flash_en2");
	gpio_direction_output(CAMERA_FLASH_EN2_GPIO, 0);
	gpio_export(CAMERA_FLASH_EN2_GPIO, false);*/

	gpio_set_value(CAMERA1_PWDN_GPIO, 1);
	mdelay(5);

	return 0;
}

static int olympus_ov5650_power_on(void)
{
	gpio_set_value(CAMERA1_PWDN_GPIO, 0);

	if (!reg_avdd_cam1) {
		reg_avdd_cam1 = regulator_get(NULL, "vio");
		if (IS_ERR_OR_NULL(reg_avdd_cam1)) {
			pr_err("olympus_ov5650_power_on: reg_avdd_cam1 failed\n");
			reg_avdd_cam1 = NULL;
			return PTR_ERR(reg_avdd_cam1);
		}
		regulator_enable(reg_avdd_cam1);
	}
	mdelay(5);

	if (!reg_vdd_mipi) {
		reg_vdd_mipi = regulator_get(NULL, "vhvio");
		if (IS_ERR_OR_NULL(reg_vdd_mipi)) {
			pr_err("olympus_ov5650_power_on: vddio_mipi failed\n");
			reg_vdd_mipi = NULL;
			return PTR_ERR(reg_vdd_mipi);
		}
		regulator_enable(reg_vdd_mipi);
	}
	mdelay(5);

	if (!reg_vdd_af) {
		reg_vdd_af = regulator_get(NULL, "v28_af_baf");
		if (IS_ERR_OR_NULL(reg_vdd_af)) {
			pr_err("olympus_ov5650_power_on: vdd_vcore_af failed\n");
			reg_vdd_af = NULL;
			return PTR_ERR(reg_vdd_af);
		}
		regulator_enable(reg_vdd_af);
	}
	mdelay(5);

	gpio_set_value(CAMERA1_RESET_GPIO, 1);
	mdelay(10);
	gpio_set_value(CAMERA1_RESET_GPIO, 0);
	mdelay(5);
	gpio_set_value(CAMERA1_RESET_GPIO, 1);
	mdelay(20);
	gpio_set_value(CAMERA_AF_PD_GPIO, 1);

	return 0;
}

static int olympus_ov5650_power_off(void)
{
	gpio_set_value(CAMERA_AF_PD_GPIO, 0);
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

	if (reg_vdd_af) {
		regulator_disable(reg_vdd_af);
		regulator_put(reg_vdd_af);
		reg_vdd_af = NULL;
	}

	return 0;
}

static int olympus_soc380_power_on(void)
{
	gpio_set_value(CAMERA2_PWDN_GPIO, 0);

	if (!reg_vddio_vi) {
		reg_vddio_vi = regulator_get(NULL, "vio");
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
		reg_avdd_cam1 = regulator_get(NULL, "vhvio");
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

	return 0;

}

static int olympus_soc380_power_off(void)
{
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
/*
 * Vibrator
 */
static struct regulator *tegra_vibrator_regulator;
static int tegra_vibrator_initialization(void)
{
	struct regulator *reg;
	reg = regulator_get(NULL, "vvib");
	if (IS_ERR(reg))
        {
                printk("VIB_GPIO:vvib regulator open error \n");
		return PTR_ERR(reg);
        }
	tegra_vibrator_regulator = reg;

	return 0;
}

static void tegra_vibrator_exit(void)
{
	regulator_put(tegra_vibrator_regulator);
}

static int tegra_vibrator_power_on(void)
{
	regulator_set_voltage(tegra_vibrator_regulator,
			3000000, 3000000);
	return regulator_enable(tegra_vibrator_regulator);
}

static int tegra_vibrator_power_off(void)
{
	if (tegra_vibrator_regulator)
		return regulator_disable(tegra_vibrator_regulator);
	return 0;
}

static int isl29030_getIrqStatus(void)
{
	int	status = -1;

	status = gpio_get_value(TEGRA_PROX_INT_GPIO);
	return status;
}

static struct vib_gpio_platform_data tegra_vib_gpio_data = {
	.gpio = TEGRA_VIBRATOR_GPIO,
	.max_timeout = 15000,
	.active_low = 0,
	.initial_vibrate = 0,

	.init = tegra_vibrator_initialization,
	.exit = tegra_vibrator_exit,
	.power_on = tegra_vibrator_power_on,
	.power_off = tegra_vibrator_power_off,
};

static struct platform_device tegra_vib_gpio = {
	.name           = "vib-gpio",
	.id             = -1,
	.dev            = {
		.platform_data  = &tegra_vib_gpio_data,
	},
};

static struct platform_device tegra_tmon = {
	.name           = "tegra_tmon",
	.id             = -1,
};

static struct bu52014hfv_platform_data bu52014hfv_platform_data = {
	.docked_north_gpio = TEGRA_HF_NORTH_GPIO,
	.docked_south_gpio = TEGRA_HF_SOUTH_GPIO,
	.kickstand_gpio = TEGRA_HF_KICKSTAND_GPIO,
	.north_is_desk = 1,
	.set_switch_func = cpcap_set_dock_switch,
};


/*
 * Accelerometer
 */
static struct regulator *kxtf9_regulator;
static int kxtf9_initialization(void)
{
	struct regulator *reg;
	reg = regulator_get(NULL, "vhvio_kxtf9");
	if (IS_ERR(reg))
		return PTR_ERR(reg);
	kxtf9_regulator = reg;
	return 0;
}

static void kxtf9_exit(void)
{
	regulator_put(kxtf9_regulator);
}

static int kxtf9_power_on(void)
{
	return regulator_enable(kxtf9_regulator);
}

static int kxtf9_power_off(void)
{
	if (kxtf9_regulator)
		return regulator_disable(kxtf9_regulator);
	return 0;
}

struct kxtf9_platform_data kxtf9_data = {
	.init = kxtf9_initialization,
	.exit = kxtf9_exit,
	.power_on = kxtf9_power_on,
	.power_off = kxtf9_power_off,

	.min_interval	= 2,
	.poll_interval	= 200,

	.g_range	= KXTF9_G_8G,

	.axis_map_x	= 0,
	.axis_map_y	= 1,
	.axis_map_z	= 2,

	.negate_x	= 1,
	.negate_y	= 1,
	.negate_z	= 0,

	.data_odr_init		= ODR25,
	.ctrl_reg1_init		= RES_12BIT | KXTF9_G_2G | TPE | WUFE | TDTE,
	.int_ctrl_init		= IEA | IEN,
	.tilt_timer_init	= 0x03,
	.engine_odr_init	= OTP12_5 | OWUF50 | OTDT400,
	.wuf_timer_init		= 0x0A,
	.wuf_thresh_init	= 0x20,
	.tdt_timer_init		= 0x78,
	.tdt_h_thresh_init	= 0xB6,
	.tdt_l_thresh_init	= 0x1A,
	.tdt_tap_timer_init	= 0xA2,
	.tdt_total_timer_init	= 0x24,
	.tdt_latency_timer_init	= 0x28,
	.tdt_window_timer_init	= 0xA0,

	.gpio = TEGRA_KXTF9_INT_GPIO,
	.gesture = 0,
	.sensitivity_low = {
		  0x50, 0xFF, 0xFF, 0x32, 0x09, 0x0A, 0xA0,
	},
	.sensitivity_medium = {
		  0x50, 0xFF, 0x68, 0xA3, 0x09, 0x0A, 0xA0,
	},
	.sensitivity_high = {
		  0x78, 0xB6, 0x1A, 0xA2, 0x24, 0x28, 0xA0,
	},
};

static void __init kxtf9_init(void)
{
#ifdef CONFIG_ARM_OF
	struct device_node *node;
	const void *prop;
	int len = 0;

	node = of_find_node_by_path(DT_PATH_ACCELEROMETER);
	if (node) {
		prop = of_get_property(node,
				DT_PROP_ACCELEROMETER_AXIS_MAP_X, &len);
		if (prop && len)
			kxtf9_data.axis_map_x = *(u8 *)prop;
		prop = of_get_property(node,
				DT_PROP_ACCELEROMETER_AXIS_MAP_Y, &len);
		if (prop && len)
			kxtf9_data.axis_map_y = *(u8 *)prop;
		prop = of_get_property(node,
				DT_PROP_ACCELEROMETER_AXIS_MAP_Z, &len);
		if (prop && len)
			kxtf9_data.axis_map_z = *(u8 *)prop;
		prop = of_get_property(node,
				DT_PROP_ACCELEROMETER_NEGATE_X, &len);
		if (prop && len)
			kxtf9_data.negate_x = *(u8 *)prop;
		prop = of_get_property(node,
				DT_PROP_ACCELEROMETER_NEGATE_Y, &len);
		if (prop && len)
			kxtf9_data.negate_y = *(u8 *)prop;
		prop = of_get_property(node,
				DT_PROP_ACCELEROMETER_NEGATE_Z, &len);
		if (prop && len)
			kxtf9_data.negate_z = *(u8 *)prop;
		prop = of_get_property(node,
				DT_PROP_ACCELEROMETER_SENS_LOW, &len);
		if (prop && len)
				memcpy(kxtf9_data.sensitivity_low,
						(u8 *)prop, len);
		prop = of_get_property(node,
				DT_PROP_ACCELEROMETER_SENS_MEDIUM, &len);
		if (prop && len)
				memcpy(kxtf9_data.sensitivity_medium,
						(u8 *)prop, len);
		prop = of_get_property(node,
				DT_PROP_ACCELEROMETER_SENS_HIGH, &len);
		if (prop && len)
				memcpy(kxtf9_data.sensitivity_high,
						(u8 *)prop, len);
		of_node_put(node);
	}
#endif
	tegra_gpio_enable(TEGRA_KXTF9_INT_GPIO);
	gpio_request(TEGRA_KXTF9_INT_GPIO, "kxtf9 accelerometer int");
	gpio_direction_input(TEGRA_KXTF9_INT_GPIO);
}

struct platform_device kxtf9_platform_device = {
	.name = "kxtf9",
	.id = -1,
	.dev = {
		.platform_data = &kxtf9_data,
	},
};


/*
 * Compass
 */

static void __init tegra_akm8975_init(void)
{
	tegra_gpio_enable(TEGRA_AKM8975_IRQ_GPIO);

	tegra_gpio_enable(TEGRA_AKM8975_RESET_GPIO);
	gpio_request(TEGRA_AKM8975_RESET_GPIO, "akm8975 reset");
	gpio_direction_output(TEGRA_AKM8975_RESET_GPIO, 1);
}

static struct regulator *akm8975_regulator;

static int akm8975_init(void)
{
	struct regulator *reg;
	int err = 0;

	printk (KERN_INFO "%s: Doing stuff\n", __func__);

	if (!akm8975_regulator) {
		reg = regulator_get(NULL, "vhvio");
//		reg = regulator_get(NULL, "vcc");
		if (IS_ERR(reg)) {
			printk (KERN_INFO "%s: Regulator error\n", __func__);
			err = PTR_ERR(reg); 
		} else {
			printk (KERN_INFO "%s: Regulator request OK\n", __func__);
			akm8975_regulator = reg;
		}
	}

	gpio_set_value(TEGRA_AKM8975_RESET_GPIO, 1);

	gpio_request(TEGRA_AKM8975_IRQ_GPIO, "akm8975_irq");
	gpio_direction_input(TEGRA_AKM8975_IRQ_GPIO);

	return err;
}

static void akm8975_exit(void)
{
	gpio_set_value(TEGRA_AKM8975_RESET_GPIO, 0);
}

static int akm8975_power_on(void)
{
	int ret;

	printk (KERN_INFO "%s: Doing stuff\n", __func__);

	ret = 0;
	if (akm8975_regulator) {
		ret = regulator_enable(akm8975_regulator);
		printk (KERN_INFO "%s: ret = %d\n", __func__, ret);
		return ret;
	}
	return -ENXIO;
}

static int akm8975_power_off(void)
{
	if (akm8975_regulator)
		return regulator_disable(akm8975_regulator);
	return -ENXIO;
}

struct akm8975_platform_data akm8975_data = {
	.init      = akm8975_init,
	.exit      = akm8975_exit,
	.power_on  = akm8975_power_on,
	.power_off = akm8975_power_off,
};

struct platform_device akm8975_platform_device = {
	.name = "akm8975",
	.id   = 0,
	.dev  = {
		.platform_data = &akm8975_data,
	},
};

/*
 * Hall Effect Sensor
 */

static struct platform_device ap20_hall_effect_dock = {
	.name	= BU52014HFV_MODULE_NAME,
	.id	= -1,
	.dev	= {
		.platform_data  = &bu52014hfv_platform_data,
	},
};

static void tegra_vibrator_init(void)
{
	tegra_gpio_enable(tegra_vib_gpio_data.gpio);
        if( gpio_request(tegra_vib_gpio_data.gpio, "vib_en") < 0) {
		printk (KERN_INFO "%s: Error requesting gpio 'vib_en' %u\n", __func__, tegra_vib_gpio_data.gpio);
	};
        gpio_direction_output(tegra_vib_gpio_data.gpio, 0);
}


/*
 * ALS/Proximity Sensor
 */
struct isl29030_platform_data isl29030_als_ir_data_Olympus = {
/*
	NOTE: Original values
	.configure = 0x6c,
	.interrupt_cntrl = 0x40,
	.prox_lower_threshold = 0x1e,
	.prox_higher_threshold = 0x32,
	.als_ir_low_threshold = 0x00,
	.als_ir_high_low_threshold = 0x00,
	.als_ir_high_threshold = 0x45,
	.lens_percent_t = 100,
*/
	.init = NULL,
	.exit = NULL,
	.power_on = NULL,
	.power_off = NULL,
	.configure = 0x66,
	.interrupt_cntrl = 0x20,
	.prox_lower_threshold = 0x0A,
	.prox_higher_threshold = 0x14,
	.crosstalk_vs_covered_threshold = 0xB4,
	.default_prox_noise_floor = 0x96,
	.num_samples_for_noise_floor = 0x05,
	.lens_percent_t = 10,
	.irq = 0,
	.getIrqStatus = isl29030_getIrqStatus,
	.gpio_intr = TEGRA_PROX_INT_GPIO,
};

static struct platform_device isl29030_als_ir = {
	.name	= LD_ISL29030_NAME,
	.id	= -1,
};
static void __init isl29030_init(void)
{
	tegra_gpio_enable(TEGRA_PROX_INT_GPIO); 
	isl29030_als_ir_data_Olympus.irq = gpio_to_irq(TEGRA_PROX_INT_GPIO);
	isl29030_als_ir.dev.platform_data = &(isl29030_als_ir_data_Olympus);
	if( gpio_request(TEGRA_PROX_INT_GPIO, "isl29030_proximity_int") < 0) {
		printk (KERN_INFO "%s: Error requesting gpio 'isl29030_proximity_int' %u\n", __func__, TEGRA_PROX_INT_GPIO);
	};
	gpio_direction_input(TEGRA_PROX_INT_GPIO);
}

/*
static int isl29030_power_on(void)
{
	return 0;
}

static int isl29030_power_off(void)
{
	return 0;
}*/

static struct platform_device *tegra_sensors[] __initdata = {
	&isl29030_als_ir,
	&kxtf9_platform_device,
	&akm8975_platform_device,
	&ap20_hall_effect_dock,
	&tegra_vib_gpio,
	&tegra_tmon,
};

static int aes1750_interrupt = TEGRA_GPIO_PM5;

static struct spi_board_info aes1750_spi_device __initdata = {
    .modalias = "aes1750",
    .bus_num = 1,
    .chip_select = 2,
    .mode = SPI_MODE_1 | SPI_CS_HIGH,
    .max_speed_hz = 15000000,
    .controller_data = NULL,
    .platform_data = &aes1750_interrupt,
    .irq = 0,
};
/*
static struct regulator *tegra_l3g4200d_regulator=NULL;

static void tegra_l3g4200d_exit(void)
{
        if (tegra_l3g4200d_regulator)
                regulator_put(tegra_l3g4200d_regulator);

        gpio_free(TEGRA_L3G4200D_IRQ_GPIO);
}
static int tegra_l3g4200d_power_on(void)
{
        if (tegra_l3g4200d_regulator)
                return regulator_enable(tegra_l3g4200d_regulator);
        return 0;
}
static int tegra_l3g4200d_power_off(void)
{
        if (tegra_l3g4200d_regulator)
                return regulator_disable(tegra_l3g4200d_regulator);
        return 0;
}
struct l3g4200d_platform_data tegra_gyro_pdata = {
        .poll_interval = 200,
        .min_interval = 0,

        .g_range = 0,

        .ctrl_reg_1 = 0xbf,
        .ctrl_reg_2 = 0x00,
        .ctrl_reg_3 = 0x00,
        .ctrl_reg_4 = 0x00,
        .ctrl_reg_5 = 0x00,
        .int_config = 0x00,
        .int_source = 0x00,
        .int_th_x_h = 0x00,
        .int_th_x_l = 0x00,
        .int_th_y_h = 0x00,
        .int_th_y_l = 0x00,
        .int_th_z_h = 0x00,
        .int_th_z_l = 0x00,
        .int_duration = 0x00,

        .axis_map_x = 0,
        .axis_map_y = 0,
        .axis_map_z = 0,

        .negate_x = 0,
        .negate_y = 0,
        .negate_z = 0,

        .exit = tegra_l3g4200d_exit,
        .power_on = tegra_l3g4200d_power_on,
        .power_off = tegra_l3g4200d_power_off,

};
*/

struct adt7461_platform_data olympus_adt7461_pdata = {
	.supported_hwrev = true,
	.ext_range = false,
	.therm2 = true,
	.conv_rate = 5,
	.offset = 0,
	.hysteresis = 0,
	.shutdown_ext_limit = 115,
	.shutdown_local_limit = 120,
	.throttling_ext_limit = 90,
	.alarm_fn = tegra_throttling_enable,
	.irq_gpio = TEGRA_ADT7461_IRQ_GPIO,
};

static struct i2c_board_info olympus_i2c3_board_info[] = {
	{
		I2C_BOARD_INFO("ov5650", 0x36),
		.platform_data = &olympus_ov5650_data,
	},
	{
		I2C_BOARD_INFO("soc380", 0x3C),
		.platform_data = &olympus_soc380_data,
	},
};

static struct i2c_board_info __initdata olympus_i2c_bus4_board_info[] = {
	{
		I2C_BOARD_INFO("akm8975", 0x0C),
		.platform_data = &akm8975_data,
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PE2),
	},
	{
		I2C_BOARD_INFO("kxtf9", 0x0F),
		.platform_data = &kxtf9_data,
	},
};

void __init mot_sensors_init(void)
{

	olympus_camera_init();
	kxtf9_init();
	tegra_akm8975_init();

	tegra_gpio_enable(TEGRA_GPIO_PE5); 
	if( gpio_request(TEGRA_GPIO_PE5, "adt7461") < 0) {
		printk (KERN_INFO "%s: Error requesting gpio 'adt7461' %u\n", __func__, TEGRA_GPIO_PE5);
	};
	gpio_direction_input(TEGRA_GPIO_PE5);

	tegra_vibrator_init();

	isl29030_init();
	
	platform_add_devices(tegra_sensors, ARRAY_SIZE(tegra_sensors));

        aes1750_spi_device.irq = gpio_to_irq(aes1750_interrupt);
        spi_register_board_info(&aes1750_spi_device,sizeof(aes1750_spi_device));

	printk("bus 2: %d devices\n", ARRAY_SIZE(olympus_i2c_bus4_board_info));
	printk("bus 3: %d devices\n", ARRAY_SIZE(olympus_i2c_bus4_board_info));

	i2c_register_board_info(3, olympus_i2c_bus4_board_info, 
				ARRAY_SIZE(olympus_i2c_bus4_board_info));

	i2c_register_board_info(2, olympus_i2c3_board_info,
		ARRAY_SIZE(olympus_i2c3_board_info));
}

