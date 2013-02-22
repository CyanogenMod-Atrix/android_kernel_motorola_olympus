#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/usb/android_composite.h>

#include <asm/mach-types.h>

#include "board-olympus.h"


/* All configurations related to USB */
 
#include <linux/console.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/sysfs.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include <linux/i2c-tegra.h>
#include <linux/mfd/tps6586x.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/setup.h>

#include <mach/io.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/gpio.h>
#include <mach/clk.h>
#include <mach/usb_phy.h>
#include <mach/system.h>

#include <linux/usb/f_accessory.h>

#include "board.h"
#include "clock.h"
#include "board-olympus.h"
#include "devices.h"
#include "gpio-names.h"
#include "fuse.h"

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

#define SERIAL_NUMBER_LENGTH 16
static char usb_serial_num[SERIAL_NUMBER_LENGTH + 1];
static int __init mot_usb_serial_num_setup(char *options)
{
	strncpy(usb_serial_num, options, SERIAL_NUMBER_LENGTH);
	usb_serial_num[SERIAL_NUMBER_LENGTH] = '\0';
	printk(KERN_INFO "usb_serial_num=%s\n", usb_serial_num);
	return 1;
}
__setup("androidboot.serialno=", mot_usb_serial_num_setup);

static int mot_boot_recovery = 0;
static int __init mot_bm_recovery_setup(char *options)
{
       mot_boot_recovery = 1;
       return 1;
}
__setup("rec", mot_bm_recovery_setup);

#define PRODUCT_TYPE_MAX_LEN 4
static char product_type[PRODUCT_TYPE_MAX_LEN + 1] = "cw";
static int __init stingray_product_type_parse(char *s)
{
	strncpy(product_type, s, PRODUCT_TYPE_MAX_LEN);
	product_type[PRODUCT_TYPE_MAX_LEN] = '\0';
	printk(KERN_INFO "product_type=%s\n", product_type);

	return 1;
}
__setup("product_type=", stingray_product_type_parse);

static struct tegra_utmip_config utmi_phy_config[] = {
	[0] = {
			.hssync_start_delay = 9,
			.idle_wait_delay = 17,
			.elastic_limit = 16,
			.term_range_adj = 6,
			.xcvr_setup = 15,
			.xcvr_lsfslew = 2,
			.xcvr_lsrslew = 2,
		},
	[1] = {
			.hssync_start_delay = 9,
			.idle_wait_delay = 17,
			.elastic_limit = 16,
			.term_range_adj = 6,
			.xcvr_setup = 8,
			.xcvr_lsfslew = 2,
			.xcvr_lsrslew = 2,
		},
};


static struct tegra_ulpi_config ulpi_phy_config = {
	.reset_gpio = TEGRA_GPIO_PG2,
	.clk = "cdev2",
};

static struct tegra_ehci_platform_data tegra_ehci_pdata[] = {
	[0] = {
		.phy_config = &utmi_phy_config[0],
		.operating_mode = TEGRA_USB_OTG,
		.power_down_on_bus_suspend = 0,
	},
	[1] = {
		.phy_config = &ulpi_phy_config,
		.operating_mode = TEGRA_USB_HOST,
		.power_down_on_bus_suspend = 1,
	},
	[2] = {
		.phy_config = &utmi_phy_config[1],
		.operating_mode = TEGRA_USB_HOST,
		.power_down_on_bus_suspend = 1,
	},
};

static void olympus_ehci_init(void)
{
	int factorycable = !strncmp(boot_mode, "factorycable",
			BOOT_MODE_MAX_LEN);
			
	tegra_otg_device.dev.platform_data = &tegra_ehci_pdata[0];
	platform_device_register(&tegra_otg_device);
	
	tegra_udc_device.dev.platform_data = &tegra_udc_pdata;
	
	tegra_ehci2_device.dev.platform_data = &tegra_ehci_pdata[1];
	tegra_ehci3_device.dev.platform_data = &tegra_ehci_pdata[2];
	
	platform_device_register(&tegra_otg_device);
	
	if (!(factorycable && mot_boot_recovery))
		platform_device_register(&tegra_udc_device);

	if (stingray_hw_has_cdma())
		platform_device_register(&tegra_ehci2_device);

	platform_device_register(&tegra_ehci3_device);
}
}

int __init olympus_usb_init(void)
{
	olympus_ehci_init();
	return 0;
}


