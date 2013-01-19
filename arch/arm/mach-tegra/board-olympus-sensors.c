#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/i2c/akm8975.h>
#include <linux/isl29030.h>
#include <linux/kxtf9.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/vib-gpio.h>
#include <asm/mach-types.h>
#include <asm/bootinfo.h>

#include "gpio-names.h"
#include "board-olympus.h"

#define TEGRA_PROX_INT_GPIO			TEGRA_GPIO_PE1
#define TEGRA_HF_NORTH_GPIO			TEGRA_GPIO_PS2
#define TEGRA_HF_SOUTH_GPIO			TEGRA_GPIO_PS0
#define TEGRA_HF_KICKSTAND_GPIO		TEGRA_GPIO_PW3
#define TEGRA_VIBRATOR_GPIO			TEGRA_GPIO_PD0
#define TEGRA_KXTF9_INT_GPIO		TEGRA_GPIO_PV3
#define TEGRA_L3G4200D_IRQ_GPIO		TEGRA_GPIO_PH2

#define TEGRA_AKM8975_RESET_GPIO	TEGRA_GPIO_PK5
#define PWRUP_BAREBOARD             0x00100000 /* Bit 20 */


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
                printk("VIB_GPIO:vvib regilator open error \n");
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

	status = gpio_get_value(PROX_INT_GPIO);
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


#ifdef CONFIG_VIB_PWM

static int tegra_lvibrator_initialization(void)
{
	struct regulator *reg;

	reg = regulator_get(NULL, "vvib");
       if (IS_ERR(reg))
        {
		printk("VIB_GPIO:vvib regilator open error \n");
		return PTR_ERR(reg);
        }
	tegra_vibrator_regulator = reg;
	return 0;
}

static void tegra_lvibrator_exit(void)
{
	if(hOdmPwm != NULL)
	{
		NvOdmPwmClose(hOdmPwm);
	}
	regulator_put(tegra_vibrator_regulator);
}

static int tegra_lvibrator_power_on(int freq, int duty_cycle)
{
	NvU32 ReqPeriod, RetPeriod;
	printk("tegra_lvibrator_power_on\n");

        if (hOdmPwm)
        {
            NvOdmPwmClose(hOdmPwm);
            hOdmPwm = NULL;
        }

        hOdmPwm = NvOdmPwmOpen();
	if (!hOdmPwm)
	{
		printk("pwm vib:  NvOdmPwmOpen failed\n");
		return NV_FALSE;
	}
	ReqPeriod = freq;
	NvOdmPwmConfig(hOdmPwm, NvOdmPwmOutputId_PWM1, NvOdmPwmMode_Enable,
		duty_cycle, &ReqPeriod, &RetPeriod);

	printk("Requested %d, ReturnPeriod=%d \n", ReqPeriod, RetPeriod);
	regulator_set_voltage(tegra_vibrator_regulator, 3000000, 3000000);
	return regulator_enable(tegra_vibrator_regulator);
}

static int tegra_lvibrator_power_off(void)
{
	NvU32 ReqPeriod, RetPeriod;
	ReqPeriod = 0;

        if (hOdmPwm != NULL)
        {
            NvOdmPwmConfig(hOdmPwm, NvOdmPwmOutputId_PWM1, NvOdmPwmMode_Disable,
                0, &ReqPeriod, &RetPeriod);

            NvOdmPwmClose(hOdmPwm);
            hOdmPwm = NULL;
        }

	if (tegra_vibrator_regulator)
		return regulator_disable(tegra_vibrator_regulator);
}
static struct vib_pwm_platform_data vib_pwm_data = {
	.gpio = TEGRA_VIBRATOR_GPIO,
	.max_timeout = 15000,
	.active_low = 0,
	.initial_vibrate = 0,
	.freq = 160,  //freq 160hz-Will be remove when pattern is implemented
	.duty_cycle = 0x00320000,  //default duty_cycle 50
	.init = tegra_lvibrator_initialization,
	.exit = tegra_lvibrator_exit,
	.power_on = tegra_lvibrator_power_on,
	.power_off = tegra_lvibrator_power_off,
	.device_name = "vibrator",
};

static struct platform_device tegra_vib_pwm = {
	.name = VIB_PWM_NAME,
	.id = -1,
	.dev = {
		.platform_data = &vib_pwm_data,
	},
};
#endif


static struct platform_device tegra_tmon = {
	.name           = "tegra_tmon",
	.id             = -1,
};


/*
static struct bu52014hfv_platform_data bu52014hfv_platform_data = {
	.docked_north_gpio = TEGRA_HF_NORTH_GPIO,
	.docked_south_gpio = TEGRA_HF_SOUTH_GPIO,
	.kickstand_gpio = TEGRA_HF_KICKSTAND_GPIO,
	.north_is_desk = 1,
	.set_switch_func = cpcap_set_dock_switch,
};
*/

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
	gpio_request(kxtf9_data.gpio, "kxtf9 accelerometer int");
	gpio_direction_input(kxtf9_data.gpio);
//	omap_cfg_reg(AF9_34XX_GPIO22_DOWN);
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
#ifdef TEGRA_AKM8975_RESET_GPIO
	gpio_request(TEGRA_AKM8975_RESET_GPIO, "akm8975 reset");
	gpio_direction_output(TEGRA_AKM8975_RESET_GPIO, 1);
#endif
}

static struct regulator *akm8975_regulator;

static int akm8975_init(void)
{
	struct regulator *reg;
	int err = 0;

	if (!akm8975_regulator) {
		reg = regulator_get(NULL, "vhvio");
		if (IS_ERR(reg))
			err = PTR_ERR(reg);
		else
			akm8975_regulator = reg;
	}

#ifdef TEGRA_AKM8975_RESET_GPIO
	gpio_set_value(TEGRA_AKM8975_RESET_GPIO, 1);
#endif
	return err;
}

static void akm8975_exit(void)
{
#ifdef TEGRA_AKM8975_RESET_GPIO
	gpio_set_value(TEGRA_AKM8975_RESET_GPIO, 0);
#endif
}

static int akm8975_power_on(void)
{
	if (akm8975_regulator)
		return regulator_enable(akm8975_regulator);
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
/*
static struct platform_device ap20_hall_effect_dock = {
	.name	= BU52014HFV_MODULE_NAME,
	.id	= -1,
	.dev	= {
		.platform_data  = &bu52014hfv_platform_data,
	},
};
*/
static void tegra_vibrator_init(void)
{
        if( gpio_request(tegra_vib_gpio_data.gpio, "vib_en") < 0) return;
        gpio_direction_output(tegra_vib_gpio_data.gpio, 0);
//	omap_cfg_reg(Y4_34XX_GPIO181);
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
	.gpio_intr = PROX_INT_GPIO,
};

static struct platform_device isl29030_als_ir = {
	.name	= LD_ISL29030_NAME,
	.id	= -1,
};
static void __init isl29030_init(void)
{
	isl29030_als_ir_data_Olympus.irq = gpio_to_irq(PROX_INT_GPIO);
	isl29030_als_ir.dev.platform_data = &(isl29030_als_ir_data_Olympus);
	gpio_request(PROX_INT_GPIO, "isl29030_proximity_int");
	gpio_direction_input(PROX_INT_GPIO);
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
/*	&ap20_hall_effect_dock,*/
	&tegra_vib_gpio,
	&tegra_tmon,
};

static int aes1750_interrupt = TEGRA_GPIO_PM5;

static struct spi_board_info aes1750_spi_device __initdata = {
    .modalias = "aes1750",
    .bus_num = 1,
    .chip_select = 2,
    .mode = SPI_MODE_1,
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
void __init mot_sensors_init(void)
{
	kxtf9_init();
	tegra_akm8975_init();

	tegra_vibrator_init();
	if(!(bi_powerup_reason() & PWRUP_BAREBOARD)) {
		isl29030_init();
	}

	platform_add_devices(tegra_sensors, ARRAY_SIZE(tegra_sensors));

        aes1750_spi_device.irq = gpio_to_irq(aes1750_interrupt);
        spi_register_board_info(&aes1750_spi_device,sizeof(aes1750_spi_device));
}
