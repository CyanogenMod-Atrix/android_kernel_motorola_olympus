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
#include <linux/platform_device.h>
#include <linux/platform_data/tegra_usb.h>
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

#include "clock.h"
#include "devices.h"
#include "gpio-names.h"
#include "pm.h"
#include "board.h"
#include "hwrev.h"
#include "board-olympus.h"

static struct device_pid olympus_android_pid[MAX_DEVICE_TYPE_NUM] = {
	{"mtp,usbnet",		0x4361},
	{"mtp,usbnet,adb",	0x4362},
	{"usbnet",		0x7083},
	{"usbnet,adb",		0x7082},
	{"ptp",			0x4367},
	{"ptp,adb",		0x4368},
	{"rndis",		0x7091},
	{"rndis,adb",		0x7092},
	{"cdrom",		0x4340},
	{"mass_storage",	0x4365},
	{"mass_storage,adb",	0x4366}, 
	{"cdrom2",		0x41ce},
	{}
};

static struct android_usb_platform_data android_usb_pdata = {
	.vendor         = "Motorola",
	.product_name   = "MB860",
	.android_pid    = olympus_android_pid,
	.nluns                  = 2,
	.cdrom_lun_num          = 0,
};

static struct platform_device android_usb_platform_device = {
	.name   = "android_gadget",
	.id     = -1,
	.dev    = {
		.platform_data = &android_usb_pdata,
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
		//.vbus_gpio = TEGRA_GPIO_PV6,
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

static int __init olympus_usb_serial_num_setup(char *options)
{
    usb_serial_num = options;
    pr_info("%s: usb_serial_num: %s\n", __func__, usb_serial_num);
    return 0;
}
__setup("androidboot.serialno=", olympus_usb_serial_num_setup);

void olympus_usb_init(void)
{
	/* OTG should be the first to be registered */
	cpcap_otg_device.dev.platform_data = &cpcap_otg_pdata;
	cpcap_device_register(&cpcap_otg_device);

//	tegra_ehci1_device.dev.platform_data = &tegra_ehci1_utmi_pdata;

	tegra_udc_device.dev.platform_data = &tegra_udc_pdata;
	platform_device_register(&tegra_udc_device);

	if (usb_serial_num)
		snprintf(android_usb_pdata.device_serial, 
			sizeof(android_usb_pdata.device_serial), usb_serial_num);
	else
		snprintf(android_usb_pdata.device_serial, 
			sizeof(android_usb_pdata.device_serial), "037c7148423ff097");

	tegra_ehci3_device.dev.platform_data = &tegra_ehci3_utmi_pdata;
	platform_device_register(&tegra_ehci3_device);

	platform_device_register(&android_usb_platform_device);

}
