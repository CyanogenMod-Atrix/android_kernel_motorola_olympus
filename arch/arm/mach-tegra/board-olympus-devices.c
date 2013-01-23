/*
 * arch/arm/mach-tegra/board-nvodm.c
 *
 * Converts data from ODM query library into platform data
 *
 * Copyright (c) 2009-2010, NVIDIA Corporation.
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
#include <linux/serial_8250.h>
#include <linux/spi-tegra.h>
#include <linux/spi/spi.h>
#include <linux/usb/f_accessory.h>
#include <linux/nvhost.h>

#include <asm/mach/time.h>
#include <asm/mach-types.h>

#include <mach/clk.h>
#include <mach/gpio.h>
#include <mach/io.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/i2s.h>
#include <mach/kbc.h>
#include <mach/nand.h>
#include <mach/pinmux.h>
#include <mach/sdhci.h>
#include <mach/w1.h>
#include <mach/usb_phy.h>
#include <mach/olympus_usb.h>
#include <mach/nvmap.h>

#include "clock.h"
#include "devices.h"
#include "gpio-names.h"
#include "pm.h"
#include "board.h"
#include "hwrev.h"
#include "board-olympus.h"
#include <linux/mmc/host.h>

#ifdef CONFIG_USB_G_ANDROID
#include <linux/usb/android_composite.h>
#endif

#define BT_RESET 0
#define BT_SHUTDOWN 1

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

static struct plat_serial8250_port debug_uart_platform_data[] = {
	{
		.membase	= IO_ADDRESS(TEGRA_UARTB_BASE),
		.mapbase	= TEGRA_UARTB_BASE,
		.irq		= INT_UARTB,
		.flags		= UPF_BOOT_AUTOCONF,
		.iotype		= UPIO_MEM,
		.regshift	= 2,
		.uartclk	= 0, /* filled in by init */
	}, {
		.flags		= 0
	}
};

static struct platform_device debug_uart = {
	.name = "serial8250",
	.id = PLAT8250_DEV_PLATFORM,
	.dev = {
		.platform_data = debug_uart_platform_data,
	},
};

/* 
 * SDHCI init
 */

extern struct tegra_nand_platform tegra_nand_plat;

static struct tegra_sdhci_platform_data olympus_sdhci_platform[] = {
	[0] = { /* SDHCI 1 - WIFI*/
		.mmc_data = {
			.built_in = 1,
		},
		.wp_gpio = -1,
		.cd_gpio = -1,
		.power_gpio = -1,
		.max_clk_limit = 50000000,
	},
	[1] = {
		
	},
	[2] = {
		.mmc_data = {
			.built_in = 0,
			.card_present = 0,
		},
		.wp_gpio = -1,
		.cd_gpio = 69,
		.power_gpio = -1,
		.max_clk_limit = 50000000,
	},
	[3] = {
		.mmc_data = {
			.built_in = 1,
		},
		.wp_gpio = -1,
		.cd_gpio = -1,
		.power_gpio = -1,
		.is_8bit = 1,
		.max_clk_limit = 50000000,
	},
};

static const char tegra_sdio_ext_reg_str[] = "vsdio_ext";
int tegra_sdhci_boot_device = -1;

static void __init olympus_sdhci_init(void)
{
	int i;

	printk(KERN_INFO "pICS_%s: Starting...",__func__);
	
	tegra_sdhci_device1.dev.platform_data = &olympus_sdhci_platform[0];
	tegra_sdhci_device3.dev.platform_data = &olympus_sdhci_platform[2];
	tegra_sdhci_device4.dev.platform_data = &olympus_sdhci_platform[3];

	/* Olympus P3+, Etna P2+, Etna S3+, Daytona and Sunfire
	   can handle shutting down the external SD card. */
	if ( (HWREV_TYPE_IS_FINAL(system_rev) || (HWREV_TYPE_IS_PORTABLE(system_rev) && (HWREV_REV(system_rev) >= HWREV_REV_3)))) 			{
	/*	olympus_sdhci_platform[2].regulator_str = (char *)tegra_sdio_ext_reg_str;*/
		}

		/* check if an "MBR" partition was parsed from the tegra partition
		 * command line, and store it in sdhci.3's offset field */

	for (i=0; i<tegra_nand_plat.nr_parts; i++) {
		if (strcmp("mbr", tegra_nand_plat.parts[i].name))
			continue;
		olympus_sdhci_platform[3].startoffset = tegra_nand_plat.parts[i].offset;
		printk(KERN_INFO "pICS_%s: tegra_sdhci_boot_device plat->offset = 0x%llx ",__func__, tegra_nand_plat.parts[i].offset);		
		}

	platform_device_register(&tegra_sdhci_device4); 
	platform_device_register(&tegra_sdhci_device1);
	platform_device_register(&tegra_sdhci_device3);
}

static void tegra_system_power_off(void)
{
	struct regulator *regulator = regulator_get(NULL, "soc_main");

	if (!IS_ERR(regulator)) {
		int rc;
		regulator_enable(regulator);
		rc = regulator_disable(regulator);
		pr_err("%s: regulator_disable returned %d\n", __func__, rc);
	} else {
		pr_err("%s: regulator_get returned %ld\n", __func__,
		       PTR_ERR(regulator));
	}
	local_irq_disable();
	while (1) {
		dsb();
		__asm__ ("wfi");
	}
}

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
#if 0
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


/* OTG gadget device */
static struct tegra_utmi_config udc_phy_config = {
	.hssync_start_delay = 9,
	.idle_wait_delay = 17,
	.elastic_limit = 16,
	.term_range_adj = 6,
	.xcvr_setup = 15,
	.xcvr_lsfslew = 1,
	.xcvr_lsrslew = 1,
};

static struct fsl_usb2_platform_data tegra_udc_pdata = {
	.operating_mode	= FSL_USB2_DR_DEVICE,
	.phy_mode	= FSL_USB2_PHY_UTMI,
	.phy_config	= &udc_phy_config,
};
#endif
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
		.vbus_pmu_irq = INT_EXTERNAL_PMU,
		.vbus_gpio = -1,
		.charging_supported = false,
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
		.vbus_gpio = 154,
//		.vbus_gpio = -1,
		.vbus_reg = NULL,
//		.vbus_reg = "vusb",
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

static char *olympus_dev = "MB860";

static int __init mot_usb_serial_num_setup(char *options)
{
    usb_serial_num = options;
    pr_info("%s: usb_serial_num: %s\n", __func__, usb_serial_num);
    return 0;
}
__setup("androidboot.serialno=", mot_usb_serial_num_setup);

static void olympus_usb_init(void)
{
	char serial[17];

	/* OTG should be the first to be registered */
	cpcap_otg_device.dev.platform_data = &cpcap_otg_pdata;
	platform_device_register(&cpcap_otg_device);

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

/*	android_usb_pdata.serial_number = "0000";
	android_usb_pdata.products[0].product_id =
	android_usb_pdata.product_id;
*/

	tegra_ehci3_device.dev.platform_data = &tegra_ehci3_utmi_pdata;
	platform_device_register(&tegra_ehci3_device);
	
	platform_device_register(&tegra_usb_fsg_device);
//	platform_device_register(&rndis_device);
	platform_device_register(&android_usb_device);
}

/*
 * SPI
 */

static struct spi_board_info tegra_spi_slave_device[] = {
	{
		.modalias = "mdm6600_spi",
		.bus_num = 0,
		.chip_select = 0,
		.max_speed_hz = 26000000,
		.mode = SPI_MODE_0,
	},
};

static struct tegra_spi_platform_data olympus_spi_pdata = {
	.is_dma_based		= true,
	.max_dma_buffer		= (16 * 1024),
	.is_clkon_always	= false,
	.max_rate		= 100000000,
};

struct spi_clk_parent spi_parent_clk[] = {
	[0] = {.name = "pll_p"},
#ifndef CONFIG_TEGRA_PLLM_RESTRICTED
	[1] = {.name = "pll_m"},
	[2] = {.name = "clk_m"},
#else
	[1] = {.name = "clk_m"},
#endif
};

static void __init olympus_spi_init(void)
{
	int i;
	struct clk *clk;

	printk("this board spi init\n");
        for (i = 0; i < ARRAY_SIZE(spi_parent_clk); ++i) {
      		clk = tegra_get_clock_by_name(spi_parent_clk[i].name);					
		if (IS_ERR_OR_NULL(clk)) {
			pr_err("Not able to get the clock for %s\n", spi_parent_clk[i].name);
			continue;
                }
		spi_parent_clk[i].parent_clk = clk;
		spi_parent_clk[i].fixed_clk_rate = clk_get_rate(clk);
	}
	olympus_spi_pdata.parent_clk_list = spi_parent_clk;
	olympus_spi_pdata.parent_clk_count = ARRAY_SIZE(spi_parent_clk);	

	spi_register_board_info(tegra_spi_slave_device, ARRAY_SIZE(tegra_spi_slave_device));
	platform_device_register(&tegra_spi_slave_device1);

	tegra_spi_device2.dev.platform_data = &olympus_spi_pdata;
	platform_device_register(&tegra_spi_device2);
	
	tegra_spi_device3.dev.platform_data = &olympus_spi_pdata;
	platform_device_register(&tegra_spi_device3);

}

static struct nvmap_platform_carveout olympus_carveouts[] = {
	[0] = NVMAP_HEAP_CARVEOUT_IRAM_INIT,
	[1] = {
		.name		= "generic-0",
		.usage_mask	= NVMAP_HEAP_CARVEOUT_GENERIC,
		.size 		= SZ_128M + SZ_64M,
		.buddy_size	= SZ_32K,
	},
};

static struct nvmap_platform_data olympus_nvmap_data = {
	.carveouts	= olympus_carveouts,
	.nr_carveouts	= ARRAY_SIZE(olympus_carveouts),
};

static struct platform_device olympus_nvmap_device = {
	.name	= "tegra-nvmap",
	.id	= -1,
	.dev	= {
		.platform_data = &olympus_nvmap_data,
	},
};

static struct resource tegra_grhost_resources[] = {
	[0] = {
		.name = "host1x",
		.start = TEGRA_HOST1X_BASE,
		.end = TEGRA_HOST1X_BASE + TEGRA_HOST1X_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.name = "display",
		.start = TEGRA_DISPLAY_BASE,
		.end = TEGRA_DISPLAY_BASE + TEGRA_DISPLAY_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[2] = {
		.name = "display2",
		.start = TEGRA_DISPLAY2_BASE,
		.end = TEGRA_DISPLAY2_BASE + TEGRA_DISPLAY2_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[3] = {
		.name = "vi",
		.start = TEGRA_VI_BASE,
		.end = TEGRA_VI_BASE + TEGRA_VI_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[4] = {
		.name = "isp",
		.start = TEGRA_ISP_BASE,
		.end = TEGRA_ISP_BASE + TEGRA_ISP_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[5] = {
		.name = "mpe",
		.start = TEGRA_MPE_BASE,
		.end = TEGRA_MPE_BASE + TEGRA_MPE_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[6] = {
		.name = "syncpt_thresh",
		.start = INT_SYNCPT_THRESH_BASE,
		.end = INT_SYNCPT_THRESH_BASE + INT_SYNCPT_THRESH_NR - 1,
		.flags = IORESOURCE_IRQ,
	},
	[7] = {
		.name = "host1x_mpcore_general",
		.start = INT_HOST1X_MPCORE_GENERAL,
		.end = INT_HOST1X_MPCORE_GENERAL,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device olympus_grhost_device = {
	.name = "tegra_grhost",
	.id = -1,
	.resource = tegra_grhost_resources,
	.num_resources = ARRAY_SIZE(tegra_grhost_resources),
};

static struct platform_device *olympus_devices[] __initdata = {
	&olympus_nvmap_device,
	&olympus_grhost_device,
	&tegra_gart_device,
	&debug_uart,
	&tegra_uarta_device,
	&tegra_uartc_device,
	&tegra_uartd_device,
	&tegra_uarte_device,
	&tegra_w1_device,
	&tegra_aes_device,
	&tegra_avp_device,
};

static int tegra_reboot_notify(struct notifier_block *nb,
				unsigned long event, void *data)
{
	printk(KERN_INFO "pICS_%s: event = [%lu]",__func__, event);
	switch (event) {
	case SYS_RESTART:
	case SYS_HALT:
	case SYS_POWER_OFF:
		/* USB power rail must be enabled during boot */
		/*NvOdmEnableUsbPhyPowerRail(1);*/
		return NOTIFY_OK;
	}
	return NOTIFY_DONE;
}

static struct notifier_block tegra_reboot_nb = {
	.notifier_call = tegra_reboot_notify,
	.next = NULL,
	.priority = 0
};

static void olympus_reboot_init(void)
{
	int rc = register_reboot_notifier(&tegra_reboot_nb);
	if (rc)
		pr_err("%s: failed to regsiter platform reboot notifier\n",
			__func__);
}

void __init olympus_devices_init()
{
	struct clk *clk;

	clk = tegra_get_clock_by_name("uartb");
	debug_uart_platform_data[0].uartclk = clk_get_rate(clk);
	olympus_spi_init();

	olympus_carveouts[1].base = tegra_carveout_start;
	olympus_carveouts[1].size = tegra_carveout_size;

	platform_add_devices(olympus_devices, ARRAY_SIZE(olympus_devices));

	printk(KERN_INFO "pICS_%s: olympus_sdhci_init();\n",__func__);
	olympus_sdhci_init();
	olympus_usb_init();

	pm_power_off = tegra_system_power_off;
	
	olympus_reboot_init();
}

