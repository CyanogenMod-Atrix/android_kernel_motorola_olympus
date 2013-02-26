#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#if defined(CONFIG_SENSORS_AK8975)
#include <linux/akm8975.h>
#endif
#include <linux/isl29030.h>
#if defined(CONFIG_MPU_SENSORS_MPU3050)
#include <linux/mpu.h>
#endif
#include <linux/nct1008.h>  
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/vib-gpio.h>
#include <asm/mach-types.h>
#include <asm/bootinfo.h>

#include "gpio-names.h"
#include "board-olympus.h"

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

#if defined(CONFIG_MPU_SENSORS_MPU3050)
static struct ext_slave_platform_data mpu3050_kxtf9_data = {
	.type		= EXT_SLAVE_TYPE_ACCEL,
	.irq		= TEGRA_GPIO_TO_IRQ(MPU_ACCEL_IRQ_GPIO),
	.address	= MPU_ACCEL_ADDR,
	.adapt_num	= MPU_ACCEL_BUS_NUM, 
	.bus		= EXT_SLAVE_BUS_SECONDARY,
	.orientation	= MPU_ACCEL_ORIENTATION,
};

/*static struct ext_slave_descr {*/

static struct ext_slave_platform_data mpu_compass_data = {
        .address        = MPU_COMPASS_ADDR,
        .irq            = TEGRA_GPIO_TO_IRQ(MPU_COMPASS_IRQ_GPIO),
        .adapt_num      = MPU_COMPASS_BUS_NUM,
        .bus            = EXT_SLAVE_BUS_PRIMARY,
        .orientation    = MPU_COMPASS_ORIENTATION,
};

#endif

static struct nct1008_platform_data olympus_nct1008_pdata = {
	.supported_hwrev = true,
	.ext_range = false,
	.conv_rate = 0x08,
	.offset = 0,
	.hysteresis = 0,
	.shutdown_ext_limit = 115, //115,85
	.shutdown_local_limit = 115, //120,85
	.throttling_ext_limit = 90, //90,70
};

static void olympus_nct1008_init(void)
{
       printk(KERN_ERR"olympus_nct1008_init\n");  //wangbing
	tegra_gpio_enable(SENSOR_TEMP_IRQ_GPIO);
	gpio_request(SENSOR_TEMP_IRQ_GPIO, "temp_alert");
	gpio_direction_input(SENSOR_TEMP_IRQ_GPIO);
}

static void tegra_vibrator_init(void)
{
        if( gpio_request(tegra_vib_gpio_data.gpio, "vib_en") < 0) return;
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
	isl29030_als_ir_data_Olympus.irq = gpio_to_irq(TEGRA_PROX_INT_GPIO);
	isl29030_als_ir.dev.platform_data = &(isl29030_als_ir_data_Olympus);
	gpio_request(TEGRA_PROX_INT_GPIO, "isl29030_proximity_int");
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
//	&kxtf9_platform_device,
//	&akm8975_platform_device,
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

static struct i2c_board_info __initdata olympus_i2c_bus4_board_info[] = {
	{
		I2C_BOARD_INFO(MPU_COMPASS_NAME, MPU_COMPASS_ADDR),
#if defined(CONFIG_MPU_SENSORS_MPU3050)
		.platform_data = &mpu_compass_data,
#endif
		.irq = TEGRA_GPIO_TO_IRQ(MPU_COMPASS_IRQ_GPIO),
	},
	{
		I2C_BOARD_INFO(MPU_ACCEL_NAME, MPU_ACCEL_ADDR),
#if defined(CONFIG_MPU_SENSORS_MPU3050)
		.platform_data = &mpu3050_kxtf9_data,
#endif
	},
	{
		I2C_BOARD_INFO(SENSOR_TEMP_NAME, SENSOR_TEMP_ADDR),
		.platform_data = &olympus_nct1008_pdata,
		.irq = TEGRA_GPIO_TO_IRQ(SENSOR_TEMP_IRQ_GPIO),
	},	
};

static void olympus_mpuirq_init(void)
{
        int ret = 0;

        pr_info("*** MPU START *** mpuirq_init...\n");

        /* MPU-IRQ assignment */
/*        tegra_gpio_enable(MPU_GYRO_IRQ_GPIO);
        ret = gpio_request(MPU_GYRO_IRQ_GPIO, MPU_GYRO_NAME);
        if (ret < 0) {
                pr_err("%s: gpio_request gyro_irq failed %d\n", __func__, ret);
        }

        ret = gpio_direction_input(MPU_GYRO_IRQ_GPIO);
        if (ret < 0) {
                pr_err("%s: gpio_direction_input gyro_irq failed %d\n", __func__, ret);
        }*/

        /* ACCEL-IRQ assignment */
        tegra_gpio_enable(MPU_ACCEL_IRQ_GPIO);
        ret = gpio_request(MPU_ACCEL_IRQ_GPIO, MPU_ACCEL_NAME);
        if (ret < 0) {
                pr_err("%s: gpio_request accel_irq1 failed %d\n", __func__, ret);
        }

        ret = gpio_direction_input(MPU_ACCEL_IRQ_GPIO);
        if (ret < 0) {
                pr_err("%s: gpio_direction_input accel_irq1 failed %d\n", __func__, ret);
        }

        /* COMPASS-IRQ assignment */
        tegra_gpio_enable(MPU_COMPASS_IRQ_GPIO);
        ret = gpio_request(MPU_COMPASS_IRQ_GPIO, MPU_COMPASS_NAME);
        if (ret < 0) {
                pr_err("%s: gpio_request compass_irq failed %d\n", __func__, ret);
        }

        ret = gpio_direction_input(MPU_COMPASS_IRQ_GPIO);
        if (ret < 0) {
                pr_err("%s: gpio_direction_input compass_irq failed %d\n", __func__, ret);
        }
        pr_info("*** MPU END *** mpuirq_init...\n");

}

void __init mot_sensors_init(void)
{
	olympus_mpuirq_init();

	printk("bus 3: %d devices\n", ARRAY_SIZE(olympus_i2c_bus4_board_info));
	i2c_register_board_info(3, olympus_i2c_bus4_board_info, 
				ARRAY_SIZE(olympus_i2c_bus4_board_info));

//	kxtf9_init();
//	tegra_akm8975_init();

	olympus_nct1008_init();
	tegra_vibrator_init();
	if(!(bi_powerup_reason() & PWRUP_BAREBOARD)) {
		isl29030_init();
	}

	platform_add_devices(tegra_sensors, ARRAY_SIZE(tegra_sensors));

        aes1750_spi_device.irq = gpio_to_irq(aes1750_interrupt);
        spi_register_board_info(&aes1750_spi_device,sizeof(aes1750_spi_device));
}
