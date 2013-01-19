#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/usb/android_composite.h>

#include <asm/mach-types.h>

#include "board-olympus.h"

#define BOOT_MODE_MAX_LEN 30

static char boot_mode[BOOT_MODE_MAX_LEN+1];
int __init board_boot_mode_init(char *s)
{
	strncpy(boot_mode, s, BOOT_MODE_MAX_LEN);
	printk(KERN_INFO "boot_mode=%s\n", boot_mode);
	return 1;
}
__setup("androidboot.mode=", board_boot_mode_init);

static char *usb_functions[] = { "usb_mass_storage" };
static char *usb_functions_adb[] = { "usb_mass_storage", "adb" };

static struct android_usb_product usb_products[] = {
	{
		.product_id     = 0x7093,
		.num_functions  = ARRAY_SIZE(usb_functions),
		.functions      = usb_functions,
	},
	{
		.product_id     = 0x7094,
		.num_functions  = ARRAY_SIZE(usb_functions_adb),
		.functions      = usb_functions_adb,
	},
};

static struct android_usb_platform_data andusb_plat = {
	.vendor_id = 0x22b8,
	.product_id = 0x7081,
	.manufacturer_name = "Motorola",
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_adb),
	.functions = usb_functions_adb,
};

static struct android_usb_platform_data andusb_plat_bp = {
	.vendor_id = 0x22b8,
	.product_id = 0x7094,
	.manufacturer_name = "Motorola",
	.serial_number = "0000",
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_adb),
	.functions = usb_functions_adb,
};

static struct platform_device tegra_android_device = {
	.name = "android_usb",
	.id = -1,
	.dev = {
		.platform_data = &andusb_plat,
	},
};

static struct usb_mass_storage_platform_data tegra_usb_fsg_platform = {
	.vendor = "Motorola",
	.product = "Mass Storage",
	.nluns = 2,   /* one for external sd and one for eMMC */
	.bulk_size = 16384,
};
static struct platform_device tegra_usb_fsg_device = {
	.name = "usb_mass_storage",
	.id = -1,
	.dev = {
		.platform_data = &tegra_usb_fsg_platform,
	},
};

static struct acm_platform_data acm_pdata = {
	/* Modify num_inst at runtime depending on boot_mode */
	.num_inst       = 1,
};

static struct platform_device acm_device = {
	.name	= "acm",
	.id	= -1,
	.dev	= {
		.platform_data = &acm_pdata,
	},
};

static char *usb_serial_num;

static char *olympus_dev = "MB860";

static int __init mot_usb_serial_num_setup(char *options)
{
    usb_serial_num = options;
    pr_info("%s: usb_serial_num: %s\n", __func__, usb_serial_num);
    return 0;
}
__setup("androidboot.serialno=", mot_usb_serial_num_setup);

void tegra_get_serial_number(void)
{
}

void __init olympus_usb_gadget_init(void)
{
	char serial[17];

	snprintf(serial, sizeof(serial), "037c7148423ff097");

	printk(KERN_INFO "pICS_%s: serial = %s",__func__, serial);
	tegra_get_serial_number();

	if (usb_serial_num)
		andusb_plat.serial_number = kstrdup(usb_serial_num, GFP_KERNEL);
	else
		andusb_plat.serial_number = kstrdup(serial, GFP_KERNEL);

	andusb_plat.product_name = olympus_dev;
	andusb_plat_bp.product_name = olympus_dev;
	tegra_usb_fsg_platform.product = olympus_dev;

	if (!strncmp(boot_mode, "bp-tools", BOOT_MODE_MAX_LEN)) {
		tegra_android_device.dev.platform_data = &andusb_plat_bp;
		acm_pdata.num_inst = 4;
		/*acm_pdata.use_iads = 1;*/
	}

	platform_device_register(&acm_device);
	platform_device_register(&tegra_usb_fsg_device);
	platform_device_register(&tegra_android_device);
}
