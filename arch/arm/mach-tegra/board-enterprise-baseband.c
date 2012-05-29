/*
 * arch/arm/mach-tegra/board-enterprise-baseband.c
 *
 * Copyright (c) 2011, NVIDIA Corporation.
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
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/err.h>
#include <linux/wakelock.h>
#include <linux/platform_data/tegra_usb.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/pinmux.h>
#include <mach/usb_phy.h>
#include <mach/tegra_usb_modem_power.h>
#include "devices.h"
#include "gpio-names.h"

/* Tegra3 BB GPIO */
#define MODEM_PWR_ON    TEGRA_GPIO_PE0
#define MODEM_RESET     TEGRA_GPIO_PE1
#define BB_RST_OUT      TEGRA_GPIO_PV1

/* Icera BB GPIO */
#define AP2MDM_ACK      TEGRA_GPIO_PE3
#define MDM2AP_ACK      TEGRA_GPIO_PU5
#define AP2MDM_ACK2     TEGRA_GPIO_PE2
#define MDM2AP_ACK2     TEGRA_GPIO_PV0

/* ULPI GPIO */
#define ULPI_STP	TEGRA_GPIO_PY3
#define ULPI_DIR	TEGRA_GPIO_PY1
#define ULPI_D0		TEGRA_GPIO_PO1
#define ULPI_D1		TEGRA_GPIO_PO2

static struct wake_lock mdm_wake_lock;

static struct gpio modem_gpios[] = {
	{MODEM_PWR_ON, GPIOF_OUT_INIT_LOW, "MODEM PWR ON"},
	{MODEM_RESET, GPIOF_IN, "MODEM RESET"},
	{BB_RST_OUT, GPIOF_IN, "BB RST OUT"},
	{MDM2AP_ACK, GPIOF_IN, "MDM2AP_ACK"},
	{AP2MDM_ACK2, GPIOF_OUT_INIT_HIGH, "AP2MDM ACK2"},
	{AP2MDM_ACK, GPIOF_OUT_INIT_LOW, "AP2MDM ACK"},
	{ULPI_STP, GPIOF_IN, "ULPI_STP"},
	{ULPI_DIR, GPIOF_OUT_INIT_LOW, "ULPI_DIR"},
	{ULPI_D0, GPIOF_OUT_INIT_LOW, "ULPI_D0"},
	{ULPI_D1, GPIOF_OUT_INIT_LOW, "ULPI_D1"},
};

static void baseband_phy_init(void);
static void baseband_phy_on(void);
static void baseband_pre_phy_off(void);
static void baseband_post_phy_off(void);
static bool ap2mdm_ack_gpio_off = false;

static struct tegra_usb_phy_platform_ops ulpi_null_plat_ops = {
	.init = baseband_phy_init,
	.pre_phy_off = baseband_pre_phy_off,
	.post_phy_off = baseband_post_phy_off,
	.post_phy_on = baseband_phy_on,
};

static struct tegra_usb_platform_data tegra_ehci2_ulpi_null_pdata = {
	.port_otg = false,
	.has_hostpc = true,
	.phy_intf = TEGRA_USB_PHY_INTF_ULPI_NULL,
	.op_mode	= TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.vbus_gpio = -1,
		.vbus_reg = NULL,
		.hot_plug = false,
		.remote_wakeup_supported = false,
		.power_off_on_suspend = true,
	},
	.u_cfg.ulpi = {
		.shadow_clk_delay = 10,
		.clock_out_delay = 1,
		.data_trimmer = 1,
		.stpdirnxt_trimmer = 1,
		.dir_trimmer = 1,
		.clk = NULL,
	},
	.ops = &ulpi_null_plat_ops,
};

static int __init tegra_null_ulpi_init(void)
{
	tegra_ehci2_device.dev.platform_data = &tegra_ehci2_ulpi_null_pdata;
	platform_device_register(&tegra_ehci2_device);
	return 0;
}

static irqreturn_t mdm_start_thread(int irq, void *data)
{
	if (gpio_get_value(BB_RST_OUT)) {
		pr_info("BB_RST_OUT high\n");
	} else {
		pr_info("BB_RST_OUT low\n");
	}

	/* hold wait lock to complete the enumeration */
	wake_lock_timeout(&mdm_wake_lock, HZ * 10);

	return IRQ_HANDLED;
}

static void baseband_phy_init(void)
{
	static bool phy_init = false;

	if (!phy_init) {
		/* set AP2MDM_ACK2 low */
		gpio_set_value(AP2MDM_ACK2, 0);
		phy_init = true;
	}
	pr_info("%s\n", __func__);
}

static inline void null_phy_set_tristate(bool enable)
{
	int tristate = (enable) ? TEGRA_TRI_TRISTATE : TEGRA_TRI_NORMAL;

	tegra_pinmux_set_tristate(TEGRA_PINGROUP_ULPI_DATA0, tristate);
	tegra_pinmux_set_tristate(TEGRA_PINGROUP_ULPI_DATA1, tristate);
	tegra_pinmux_set_tristate(TEGRA_PINGROUP_ULPI_DATA2, tristate);
	tegra_pinmux_set_tristate(TEGRA_PINGROUP_ULPI_DATA3, tristate);
	tegra_pinmux_set_tristate(TEGRA_PINGROUP_ULPI_DATA4, tristate);
	tegra_pinmux_set_tristate(TEGRA_PINGROUP_ULPI_DATA5, tristate);
	tegra_pinmux_set_tristate(TEGRA_PINGROUP_ULPI_DATA6, tristate);
	tegra_pinmux_set_tristate(TEGRA_PINGROUP_ULPI_DATA7, tristate);
	tegra_pinmux_set_tristate(TEGRA_PINGROUP_ULPI_NXT, tristate);

	if (enable)
		tegra_pinmux_set_tristate(TEGRA_PINGROUP_ULPI_DIR, tristate);
}


static void baseband_post_phy_off(void)
{
	null_phy_set_tristate(true);
}


static void baseband_pre_phy_off(void)
{
	/* set AP2MDM_ACK2 high */
	gpio_set_value(AP2MDM_ACK2, 1);
	ap2mdm_ack_gpio_off = true;
}

static void baseband_phy_on(void)
{
	if (ap2mdm_ack_gpio_off) {

		/* driving linestate using GPIO */
		gpio_set_value(ULPI_D0, 0);
		gpio_set_value(ULPI_D1, 0);

		/* remove ULPI tristate */
		null_phy_set_tristate(false);

		gpio_set_value(AP2MDM_ACK2, 0);

		if (gpio_is_valid(MDM2AP_ACK2)) {
			int retry = 20000;
			while (retry) {
				/* poll phy_restore_gpio high */
				if (gpio_get_value(MDM2AP_ACK2))
					break;
				retry--;
			}

			if (retry == 0)
				pr_info("phy_restore_gpio timeout\n");
		}
		ap2mdm_ack_gpio_off = false;
	}
}

static void baseband_start(void)
{
	/*
	 *  Leave baseband powered OFF.
	 *  User-space daemons will take care of powering it up.
	 */
	pr_info("%s\n", __func__);
	gpio_set_value(MODEM_PWR_ON, 0);
}

static void baseband_reset(void)
{
	/* Initiate power cycle on baseband sub system */
	pr_info("%s\n", __func__);
	gpio_set_value(MODEM_PWR_ON, 0);
	mdelay(200);
	gpio_set_value(MODEM_PWR_ON, 1);
}

static int baseband_init(void)
{
	int irq;
	int ret;

	ret = gpio_request_array(modem_gpios, ARRAY_SIZE(modem_gpios));
	if (ret)
		return ret;

	/* enable pull-up for ULPI STP */
	tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_ULPI_STP,
				    TEGRA_PUPD_PULL_UP);

	/* enable pull-up for MDM2AP_ACK2 */
	tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_GPIO_PV0,
				    TEGRA_PUPD_PULL_UP);

	tegra_gpio_enable(MODEM_PWR_ON);
	tegra_gpio_enable(MODEM_RESET);
	tegra_gpio_enable(AP2MDM_ACK2);
	tegra_gpio_enable(BB_RST_OUT);
	tegra_gpio_enable(AP2MDM_ACK);
	tegra_gpio_enable(MDM2AP_ACK);
	tegra_gpio_enable(TEGRA_GPIO_PY3);
	tegra_gpio_enable(TEGRA_GPIO_PO1);
	tegra_gpio_enable(TEGRA_GPIO_PO2);

	/* export GPIO for user space access through sysfs */
	gpio_export(MODEM_PWR_ON, false);

	/* phy init */
	tegra_null_ulpi_init();

	wake_lock_init(&mdm_wake_lock, WAKE_LOCK_SUSPEND, "mdm_lock");

	/* enable IRQ for BB_RST_OUT */
	irq = gpio_to_irq(BB_RST_OUT);

	ret = request_threaded_irq(irq, NULL, mdm_start_thread,
				   IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				   "mdm_start", NULL);
	if (ret < 0) {
		pr_err("%s: request_threaded_irq error\n", __func__);
		return ret;
	}

	ret = enable_irq_wake(irq);
	if (ret) {
		pr_err("%s: enable_irq_wake error\n", __func__);
		free_irq(irq, NULL);
		return ret;
	}

	return 0;
}

static const struct tegra_modem_operations baseband_operations = {
	.init = baseband_init,
	.start = baseband_start,
	.reset = baseband_reset,
};

static struct tegra_usb_modem_power_platform_data baseband_pdata = {
	.ops = &baseband_operations,
	.wake_gpio = MDM2AP_ACK2,
	.flags = IRQF_TRIGGER_FALLING,
};

static struct platform_device icera_baseband_device = {
	.name = "tegra_usb_modem_power",
	.id = -1,
	.dev = {
		.platform_data = &baseband_pdata,
	},
};

int __init enterprise_modem_init(void)
{
	platform_device_register(&icera_baseband_device);
	return 0;
}
