#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/spi-tegra.h>
#include <linux/akm8975.h>
#include <linux/adt7461.h>
#include <linux/isl29030.h>
#include <linux/bu52014hfv.h>
#include <linux/kxtf9.h>
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

static void olympus_vibrator_init(void)
{
	tegra_gpio_enable(tegra_vib_gpio_data.gpio);
        if( gpio_request(tegra_vib_gpio_data.gpio, "vib_en") < 0) {
		printk (KERN_INFO "%s: Error requesting gpio 'vib_en' %u\n", __func__, tegra_vib_gpio_data.gpio);
	};
        gpio_direction_output(tegra_vib_gpio_data.gpio, 0);
}

/*
 * Temp sensor
 */

static void olympus_adt7461_init(void)
{
	tegra_gpio_enable(TEGRA_ADT7461_IRQ_GPIO);
	gpio_request(TEGRA_ADT7461_IRQ_GPIO, "adt7461");
	gpio_direction_input(TEGRA_ADT7461_IRQ_GPIO);
}

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
	//.irq_gpio = TEGRA_ADT7461_IRQ_GPIO,
};

/*
 * Accelerometer
 */
static struct regulator *kxtf9_regulator;
static int kxtf9_initialization(void)
{
	struct regulator *reg;
	reg = regulator_get(NULL, "vhvio");
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

	.axis_map_x	= 1,
	.axis_map_y	= 0,
	.axis_map_z	= 2,

	.negate_x	= 1,
	.negate_y	= 1,
	.negate_z	= 1,

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

static void __init olympus_kxtf9_init(void)
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

static void __init olympus_akm8975_init(void)
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
		if (IS_ERR(reg)) {
			printk (KERN_INFO "%s: Regulator error\n", __func__);
			err = PTR_ERR(reg); 
		} else {
			printk (KERN_INFO "%s: Regulator request OK\n", __func__);
			akm8975_regulator = reg;
		}
	}

	gpio_request(TEGRA_AKM8975_IRQ_GPIO, "akm8975_irq");
	gpio_direction_input(TEGRA_AKM8975_IRQ_GPIO);

	return err;
}

static void akm8975_exit(void)
{
	return;
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

static struct bu52014hfv_platform_data bu52014hfv_platform_data = {
	.docked_north_gpio = TEGRA_HF_NORTH_GPIO,
	.docked_south_gpio = TEGRA_HF_SOUTH_GPIO,
	.kickstand_gpio = TEGRA_HF_KICKSTAND_GPIO,
	.north_is_desk = 1,
	.set_switch_func = cpcap_set_dock_switch,
};

static struct platform_device ap20_hall_effect_dock = {
	.name	= BU52014HFV_MODULE_NAME,
	.id	= -1,
	.dev	= {
		.platform_data  = &bu52014hfv_platform_data,
	},
};

/*
 * ALS/Proximity Sensor
 */

struct isl29030_platform_data olympus_isl29030_pdata = {
	.configure = 0x66,
	.interrupt_cntrl = 0x20,
	.prox_lower_threshold = 0x0A,
	.prox_higher_threshold = 0x14,
	.crosstalk_vs_covered_threshold = 0xB4,
	.default_prox_noise_floor = 0x96,
	.num_samples_for_noise_floor = 0x05,
	.lens_percent_t = 10,
	.irq = 0,
};

static struct platform_device isl29030_als_ir = {
	.name	= LD_ISL29030_NAME,
	.id	= -1,
};
static void __init olympus_isl29030_init(void)
{
	tegra_gpio_enable(TEGRA_PROX_INT_GPIO); 

	olympus_isl29030_pdata.irq = gpio_to_irq(TEGRA_PROX_INT_GPIO);
	isl29030_als_ir.dev.platform_data = &(olympus_isl29030_pdata);

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
}
*/
static struct platform_device *olympus_sensors[] __initdata = {
	&isl29030_als_ir,
	&kxtf9_platform_device,
	&akm8975_platform_device,
	&ap20_hall_effect_dock,
	&tegra_vib_gpio,
	&tegra_tmon,
};

static int aes1750_interrupt = TEGRA_GPIO_PM5;

static struct tegra_spi_platform_data aes1750_spi_slave_platform_data = {
    .is_dma_based = true,
    .is_clkon_always = false,
};

static struct tegra_spi_device_controller_data aes1750_spi_controller_data = {
    .is_hw_based_cs = 0,
};

static struct spi_board_info aes1750_spi_device __initdata = {
		.modalias = "aes1750",
		.bus_num = 1,
		.chip_select = 2,
		.mode = SPI_MODE_1,
		.max_speed_hz = 15000000,
		.controller_data = &aes1750_spi_controller_data,
		.platform_data = &aes1750_spi_slave_platform_data,
		.irq = 0,
};

static struct i2c_board_info olympus_i2c1_board_info[] = {
	{
		/*  ISL 29030 (prox/ALS) driver */
		I2C_BOARD_INFO(LD_ISL29030_NAME, 0x44),
		.platform_data = &olympus_isl29030_pdata,
		.irq = 180,
	},
};

static struct i2c_board_info olympus_i2c4_board_info[] = {
	{
		I2C_BOARD_INFO("adt7461", 0x4C),
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PE5),
		.platform_data = &olympus_adt7461_pdata,
	},
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

void __init olympus_sensors_init(void)
{
	olympus_kxtf9_init();

	olympus_adt7461_init();	

	olympus_akm8975_init();

	olympus_vibrator_init();

	olympus_isl29030_init();
	
	platform_add_devices(olympus_sensors, ARRAY_SIZE(olympus_sensors));

	printk("bus 0: %d device\n", 1);
	printk("bus 3: %d devices\n", ARRAY_SIZE(olympus_i2c4_board_info));

	i2c_register_board_info(0, olympus_i2c1_board_info, 
				ARRAY_SIZE(olympus_i2c1_board_info));

	i2c_register_board_info(3, olympus_i2c4_board_info, 
				ARRAY_SIZE(olympus_i2c4_board_info));

        aes1750_spi_device.irq = gpio_to_irq(aes1750_interrupt);
        spi_register_board_info(&aes1750_spi_device,
					sizeof(aes1750_spi_device));

}

