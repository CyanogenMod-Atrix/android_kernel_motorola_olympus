/*
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/fsl_devices.h>
#include <linux/gpio.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/platform_device.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/pda_power.h>
#include <linux/regulator/machine.h>
#include <linux/reboot.h>
#include <linux/spi/cpcap.h>

#include <linux/usb/composite.h>
#include <linux/usb/gadget.h>
#include <linux/usb/f_accessory.h>
#include <linux/fsl_devices.h>

#include <asm/mach/time.h>
#include <asm/mach-types.h>

#include <mach/clk.h>
#include <mach/gpio.h>
#include <mach/io.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/usb_phy.h>
#include <mach/olympus_usb.h>

#include "clock.h"
#include "devices.h"
#include "gpio-names.h"
#include "pm.h"
#include "board.h"
#include "hwrev.h"
#include "board-olympus.h"

#ifdef CONFIG_USB_G_ANDROID
#include <linux/usb/android_composite.h>
#endif

#define BOOT_MODE_MAX_LEN 30
static char boot_mode[BOOT_MODE_MAX_LEN + 1];
int __init board_boot_mode_init(char *s)
{
	strncpy(boot_mode, s, BOOT_MODE_MAX_LEN);
	boot_mode[BOOT_MODE_MAX_LEN] = '\0';
	printk(KERN_INFO "boot_mode=%s\n", boot_mode);
	return 1;
}
__setup("androidboot.mode=", board_boot_mode_init);

#define USB_MANUFACTURER_NAME	"Motorola"
#define USB_PRODUCT_NAME	"Atrix 4G"
#define BLUE_PID		0x0CD9
#define USB_VENDOR_ID		0x0BB4

static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id	= 0x22b8,
	.product_id	= 0x7081,
//	.version	= 0x0100,
//	.product_name		= "Atrix 4G",
	.manufacturer_name	= "Motorola",
//	.serial_number = "0000",
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_all),
	.functions = usb_functions_all,
//	.fserial_init_string = "tty:modem,tty:autobot,tty:serial,tty:autobot",
//	.usb_id_pin_gpio = TEGRA_GPIO_PS2,
//	.RndisDisableMPDecision = true,
	.nluns = 2,
};

static struct android_usb_platform_data android_usb_bp_pdata = {
	.vendor_id = 0x22b8,
	.product_id = 0x7094,
	.manufacturer_name = "Motorola",
	.serial_number = "0000",
	.num_products = ARRAY_SIZE(bp_usb_products),
	.products = bp_usb_products,
	.num_functions = ARRAY_SIZE(bp_usb_functions_all),
	.functions = bp_usb_functions_all,

};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id		= -1,
	.dev		= {
		.platform_data = &android_usb_pdata,
	},
};

static struct usb_mass_storage_platform_data tegra_usb_fsg_platform = {
	.vendor = "Motorola",
	.product = "Mass Storage",
	.nluns = 2,   /* one for external sd and one for eMMC */
//	.bulk_size = 16384,
};

static struct platform_device tegra_usb_fsg_device = {
	.name = "usb_mass_storage",
	.id = -1,
	.dev = {
		.platform_data = &tegra_usb_fsg_platform,
	},
};

static struct usb_ether_platform_data rndis_pdata = {
	/* ethaddr is filled by board_serialno_setup */
	.vendorID	= 0x22b8,
	.vendorDescr	= "Motorola",
};

static struct platform_device rndis_device = {
	.name	= "rndis",
	.id	= -1,
	.dev	= {
		.platform_data = &rndis_pdata,
	},
};

/* OTG transceiver */
static struct resource cpcap_otg_resources[] = {
	[0] = {
		.start  = TEGRA_USB_BASE,
		.end    = TEGRA_USB_BASE + TEGRA_USB_SIZE - 1,
		.flags  = IORESOURCE_MEM,
	},
};

static struct platform_device cpcap_otg_device = {
	.name = "cpcap-otg",
	.id   = -1,
	.resource = cpcap_otg_resources,
	.num_resources = ARRAY_SIZE(cpcap_otg_resources),
	.dev = {
		.platform_data = &tegra_ehci1_device,
	},
};

static struct tegra_usb_platform_data tegra_udc_pdata = {
	.port_otg = true,
	.has_hostpc = false,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode = TEGRA_USB_OPMODE_DEVICE,
	.u_data.dev = {
		//.vbus_pmu_irq = -1,
		.vbus_gpio = TEGRA_GPIO_PV6,
		.charging_supported = true,
		.remote_wakeup_supported = false,
	},
	.u_cfg.utmi = {
		.hssync_start_delay = 9,
		.idle_wait_delay = 17,
		.elastic_limit = 16,
		.term_range_adj = 6,
		.xcvr_setup = 15,
		.xcvr_lsfslew = 1,
		.xcvr_lsrslew = 1,
	},
};

static struct tegra_usb_platform_data tegra_ehci1_utmi_pdata = {
	.port_otg = true,
	.has_hostpc = false,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode = TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.vbus_gpio = -1,
//		.vbus_reg = NULL,
		.vbus_reg = "vusb",
		.hot_plug = true,
		.remote_wakeup_supported = false,
		.power_off_on_suspend = true,
	},
	.u_cfg.utmi = {
		.hssync_start_delay = 9,
		.elastic_limit = 16,
		.idle_wait_delay = 17,
		.term_range_adj = 6,
		.xcvr_setup = 8,
		.xcvr_lsfslew = 2,
		.xcvr_lsrslew = 2,
	},
};

static void ulpi_link_platform_open(void)
{
	int reset_gpio = TEGRA_GPIO_PG2;

	gpio_request(reset_gpio, "ulpi_phy_reset");
	gpio_direction_output(reset_gpio, 0);
	tegra_gpio_enable(reset_gpio);

	gpio_direction_output(reset_gpio, 0);
	msleep(5);
	gpio_direction_output(reset_gpio, 1);
}

static struct tegra_usb_phy_platform_ops ulpi_link_plat_ops = {
	.open = ulpi_link_platform_open,
};

static struct tegra_usb_platform_data tegra_ehci2_ulpi_link_pdata = {
	.port_otg = false,
	.has_hostpc = false,
	.phy_intf = TEGRA_USB_PHY_INTF_ULPI_LINK,
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
		.data_trimmer = 4,
		.stpdirnxt_trimmer = 4,
		.dir_trimmer = 4,
		.clk = "cdev2",
	},
	.ops = &ulpi_link_plat_ops,
};

static struct tegra_usb_platform_data tegra_ehci3_utmi_pdata = {
	.port_otg = false,
	.has_hostpc = false,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode	= TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.vbus_gpio = -1,
		.vbus_reg = NULL,
		.hot_plug = true,
		.remote_wakeup_supported = false,
		.power_off_on_suspend = true,
	},
	.u_cfg.utmi = {
		.hssync_start_delay = 9,
		.elastic_limit = 16,
		.idle_wait_delay = 17,
		.term_range_adj = 6,
		.xcvr_setup = 8,
		.xcvr_lsfslew = 2,
		.xcvr_lsrslew = 2,
	},
};

static struct tegra_usb_otg_data cpcap_otg_pdata = {
	.ehci_device = &tegra_ehci1_device,
	.ehci_pdata = &tegra_ehci1_utmi_pdata,
};

static char *usb_serial_num;

static char *olympus_dev = "MB860";

static int __init olympus_usb_serial_num_setup(char *options)
{
    usb_serial_num = options;
    pr_info("%s: usb_serial_num: %s\n", __func__, usb_serial_num);
    return 0;
}
__setup("androidboot.serialno=", olympus_usb_serial_num_setup);

void olympus_usb_init(void)
{
	char serial[17];

	/* OTG should be the first to be registered */
	cpcap_otg_device.dev.platform_data = &cpcap_otg_pdata;
	cpcap_device_register(&cpcap_otg_device);

	snprintf(serial, sizeof(serial), "037c7148423ff097");

	tegra_ehci1_device.dev.platform_data = &tegra_ehci1_utmi_pdata;

	tegra_udc_device.dev.platform_data = &tegra_udc_pdata;
	platform_device_register(&tegra_udc_device);

	if (usb_serial_num)
		android_usb_pdata.serial_number = kstrdup(usb_serial_num, GFP_KERNEL);
	else
		android_usb_pdata.serial_number = kstrdup(serial, GFP_KERNEL);

	android_usb_pdata.product_name = olympus_dev;
	android_usb_bp_pdata.product_name = olympus_dev;
	tegra_usb_fsg_platform.product = olympus_dev;

	if (!strncmp(boot_mode, "bp-tools", BOOT_MODE_MAX_LEN)) {
		android_usb_device.dev.platform_data = &android_usb_bp_pdata;
/*		acm_pdata.num_inst = 4;
		acm_pdata.use_iads = 1;*/
	}

//	tegra_ehci2_device.dev.platform_data = &tegra_ehci2_ulpi_link_pdata;
//	platform_device_register(&tegra_ehci2_device);

	tegra_ehci3_device.dev.platform_data = &tegra_ehci3_utmi_pdata;
	platform_device_register(&tegra_ehci3_device);

	platform_device_register(&android_usb_device);
	platform_device_register(&tegra_usb_fsg_device);
	platform_device_register(&rndis_device);

}
