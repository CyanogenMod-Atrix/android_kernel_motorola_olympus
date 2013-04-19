/*
 * arch/arm/mach-tegra/board-mot.c
 *
 * Copyright (C) 2010 Google, Inc.
 * Copyright (C) 2010 NVIDIA Corporation
 * Copyright (C) 2010 Motorola, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/serial_8250.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/pda_power.h>
#include <linux/io.h>
#include <linux/spi/cpcap.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/memblock.h>
#include <linux/console.h>
#include <linux/mdm_ctrl.h>

#include <asm/bootinfo.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/setup.h>

#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/w1.h>
#include <linux/gpio.h>
#include <linux/cpcap-accy.h>
#include <linux/reboot.h>

#include "clock.h"
#include "gpio-names.h"
#include "pm.h"

#include "board.h"
#include "devices.h"
#include "hwrev.h"

#include "board-olympus.h"

#define PWRUP_FACTORY_CABLE         0x00000020 /* Bit 5  */
#define PWRUP_INVALID               0xFFFFFFFF
#define PWRUP_BAREBOARD             0x00100000 /* Bit 20 */

#define UART_IPC_OLYMPUS	3

static char oly_unused_pins_p3[] = {
        TEGRA_GPIO_PO1,
        TEGRA_GPIO_PO2,
        TEGRA_GPIO_PO3,
        TEGRA_GPIO_PO4,
        TEGRA_GPIO_PO5,
        TEGRA_GPIO_PO6,
        TEGRA_GPIO_PO7,
        TEGRA_GPIO_PO0,
        TEGRA_GPIO_PY0,
        TEGRA_GPIO_PY1,
        TEGRA_GPIO_PY2,
        TEGRA_GPIO_PY3,
        TEGRA_GPIO_PC1,        
        TEGRA_GPIO_PN5,
        TEGRA_GPIO_PN6,
        TEGRA_GPIO_PW1,
        TEGRA_GPIO_PB3,
        TEGRA_GPIO_PJ3,
        TEGRA_GPIO_PE5,
        TEGRA_GPIO_PE6,
        TEGRA_GPIO_PE7,
        TEGRA_GPIO_PF0,
        TEGRA_GPIO_PM6,
        TEGRA_GPIO_PM7,
        TEGRA_GPIO_PT4,
        TEGRA_GPIO_PL5,
        TEGRA_GPIO_PL6,
        TEGRA_GPIO_PL7,
        TEGRA_GPIO_PT2,
        TEGRA_GPIO_PD6,
        TEGRA_GPIO_PD7,
        TEGRA_GPIO_PR3,
        TEGRA_GPIO_PR4,
        TEGRA_GPIO_PR5,
        TEGRA_GPIO_PR6,
        TEGRA_GPIO_PR7,
        TEGRA_GPIO_PS0,
        TEGRA_GPIO_PS1,
        TEGRA_GPIO_PS2,
        TEGRA_GPIO_PQ3,
        TEGRA_GPIO_PQ4,
        TEGRA_GPIO_PQ5,
        TEGRA_GPIO_PBB0,
        TEGRA_GPIO_PZ5,
        /* TEGRA_GPIO_PK5, AKM8975_RESET_GPIO */
        TEGRA_GPIO_PK6,
        TEGRA_GPIO_PW5,
        TEGRA_GPIO_PD3,
        TEGRA_GPIO_PI7,
        TEGRA_GPIO_PJ0,
        TEGRA_GPIO_PJ2,
        TEGRA_GPIO_PK3,
        TEGRA_GPIO_PK4,
        TEGRA_GPIO_PK2,
        TEGRA_GPIO_PG0,
        TEGRA_GPIO_PG1,
        TEGRA_GPIO_PG2,
        TEGRA_GPIO_PG3,
        TEGRA_GPIO_PG4,
        TEGRA_GPIO_PG5,
        TEGRA_GPIO_PG6,
        TEGRA_GPIO_PG7,
        TEGRA_GPIO_PH0,
        TEGRA_GPIO_PH1,
        TEGRA_GPIO_PH2,
        TEGRA_GPIO_PH3,
        TEGRA_GPIO_PI0,
        TEGRA_GPIO_PI4,
        TEGRA_GPIO_PT5,
        TEGRA_GPIO_PT6,
        TEGRA_GPIO_PC7,
};

static char oly_unused_pins_p2[] = {
        TEGRA_GPIO_PO1,
        TEGRA_GPIO_PO2,
        TEGRA_GPIO_PO3,
        TEGRA_GPIO_PO4,
        TEGRA_GPIO_PO5,
        TEGRA_GPIO_PO6,
        TEGRA_GPIO_PO7,
        TEGRA_GPIO_PO0,
        TEGRA_GPIO_PY0,
        TEGRA_GPIO_PY1,
        TEGRA_GPIO_PY2,
        TEGRA_GPIO_PY3,
        TEGRA_GPIO_PC1,        
        TEGRA_GPIO_PN5,
        TEGRA_GPIO_PN6,
        TEGRA_GPIO_PW1,
        TEGRA_GPIO_PB3,
        TEGRA_GPIO_PJ3,
        TEGRA_GPIO_PE7,
        TEGRA_GPIO_PF0,
        TEGRA_GPIO_PM6,
        TEGRA_GPIO_PM7,
        TEGRA_GPIO_PT4,
        TEGRA_GPIO_PL5,
        TEGRA_GPIO_PL6,
        TEGRA_GPIO_PL7,
        TEGRA_GPIO_PT2,
        TEGRA_GPIO_PD6,
        TEGRA_GPIO_PD7,
        TEGRA_GPIO_PR3,
        TEGRA_GPIO_PR4,
        TEGRA_GPIO_PR5,
        TEGRA_GPIO_PR6,
        TEGRA_GPIO_PR7,
        TEGRA_GPIO_PS0,
        TEGRA_GPIO_PS1,
        TEGRA_GPIO_PS2,
        TEGRA_GPIO_PQ3,
        TEGRA_GPIO_PQ4,
        TEGRA_GPIO_PQ5,
        TEGRA_GPIO_PBB0,
        TEGRA_GPIO_PZ5,
        TEGRA_GPIO_PK5,
        TEGRA_GPIO_PK6,
        TEGRA_GPIO_PW5,
        TEGRA_GPIO_PD3,
        TEGRA_GPIO_PI7,
        TEGRA_GPIO_PJ0,
        TEGRA_GPIO_PJ2,
        TEGRA_GPIO_PK3,
        TEGRA_GPIO_PK4,
        TEGRA_GPIO_PK2,
        TEGRA_GPIO_PG0,
        TEGRA_GPIO_PG1,
        TEGRA_GPIO_PG2,
        TEGRA_GPIO_PG3,
        TEGRA_GPIO_PG4,
        TEGRA_GPIO_PG5,
        TEGRA_GPIO_PG6,
        TEGRA_GPIO_PG7,
        TEGRA_GPIO_PH0,
        TEGRA_GPIO_PH1,
        TEGRA_GPIO_PH2,
        TEGRA_GPIO_PH3,
        TEGRA_GPIO_PI0,
        TEGRA_GPIO_PI4,
        TEGRA_GPIO_PT5,
        TEGRA_GPIO_PT6,
        TEGRA_GPIO_PC7,
        TEGRA_GPIO_PD1,
};

static char oly_unused_pins_p1[] = {
        TEGRA_GPIO_PO1,
        TEGRA_GPIO_PO2,
        TEGRA_GPIO_PO3,
        TEGRA_GPIO_PO4,
        TEGRA_GPIO_PO5,
        TEGRA_GPIO_PO6,
        TEGRA_GPIO_PO7,
        TEGRA_GPIO_PO0,
        TEGRA_GPIO_PY0,
        TEGRA_GPIO_PY1,
        TEGRA_GPIO_PY2,
        TEGRA_GPIO_PY3,        
        TEGRA_GPIO_PN5,
        TEGRA_GPIO_PN6,
        TEGRA_GPIO_PW1,
        TEGRA_GPIO_PB3,
        TEGRA_GPIO_PJ3,
        TEGRA_GPIO_PF0,
        TEGRA_GPIO_PM6,
        TEGRA_GPIO_PM7,
        TEGRA_GPIO_PL5,
        TEGRA_GPIO_PL6,
        TEGRA_GPIO_PL7,
        TEGRA_GPIO_PT2,
        TEGRA_GPIO_PD6,
        TEGRA_GPIO_PD7,
        TEGRA_GPIO_PR3,
        TEGRA_GPIO_PR4,
        TEGRA_GPIO_PR5,
        TEGRA_GPIO_PR6,
        TEGRA_GPIO_PR7,
        TEGRA_GPIO_PS1,
        TEGRA_GPIO_PQ3,
        TEGRA_GPIO_PQ4,
        TEGRA_GPIO_PQ5,
        TEGRA_GPIO_PBB0,
        TEGRA_GPIO_PZ5,
        TEGRA_GPIO_PK5,
        TEGRA_GPIO_PK6,
        TEGRA_GPIO_PW5,
        TEGRA_GPIO_PD3,
        TEGRA_GPIO_PI7,
        TEGRA_GPIO_PJ0,
        TEGRA_GPIO_PJ2,
        TEGRA_GPIO_PK3,
        TEGRA_GPIO_PK4,
        TEGRA_GPIO_PK2,
        TEGRA_GPIO_PG0,
        TEGRA_GPIO_PG1,
        TEGRA_GPIO_PG2,
        TEGRA_GPIO_PG3,
        TEGRA_GPIO_PG4,
        TEGRA_GPIO_PG5,
        TEGRA_GPIO_PG6,
        TEGRA_GPIO_PG7,
        TEGRA_GPIO_PH0,
        TEGRA_GPIO_PH1,
        TEGRA_GPIO_PH2,
        TEGRA_GPIO_PH3,
        TEGRA_GPIO_PI0,
        TEGRA_GPIO_PI4,
        TEGRA_GPIO_PT5,
        TEGRA_GPIO_PT6,
        TEGRA_GPIO_PC7,
        TEGRA_GPIO_PV7,
        TEGRA_GPIO_PD1,
};

static __initdata struct tegra_clk_init_table olympus_clk_init_table[] = {
	/* name		parent		rate		enabled */  
	{"sbc1",	"pll_c",	60000000,	true},
	{"sbc2",	"pll_c",	60000000,	true},
	{"pwm",		"clk_32k",	32768,		false},
	{"kbc",		"clk_32k",	32768,		true},
	{"sdmmc2",	"pll_p",	25000000,	false},
	{"i2s1",	"pll_a_out0",	0,		false},
	{"spdif_out",	"pll_a_out0",	0,		false},
	{ NULL,		NULL,		0,		0},
};

static struct resource olympus_bcm4329_rfkill_resources[] = {
	{
		.name	= "bcm4329_nreset_gpio",
		.start	= 0,
		.end	= 0,
		.flags	= IORESOURCE_IO,
	},
	{
		.name	= "bcm4329_nshutdown_gpio",
		.start	= TEGRA_GPIO_PU0,
		.end	= TEGRA_GPIO_PU0,
		.flags	= IORESOURCE_IO,
	},
	{
		.name	= "bcm4329_wake_gpio",
		.start	= TEGRA_GPIO_PU1,
		.end	= TEGRA_GPIO_PU1,
		.flags	= IORESOURCE_IO,
	},
	{
		.name	= "bcm4329_host_wake_gpio",
		.start	= TEGRA_GPIO_PU6,
		.end	= TEGRA_GPIO_PU6,
		.flags	= IORESOURCE_IO,
	},
};

static struct platform_device olympus_bcm4329_rfkill_device = {
	.name		= "bcm4329_rfkill",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(olympus_bcm4329_rfkill_resources),
	.resource	= olympus_bcm4329_rfkill_resources,
};

static noinline void __init olympus_bt_rfkill(void)
{
	olympus_bcm4329_rfkill_resources[0].start =
		olympus_bcm4329_rfkill_resources[0].end = TEGRA_GPIO_PU4;
	printk("%s: registering bcm4329_rfkill device...\n", __func__);

	platform_device_register(&olympus_bcm4329_rfkill_device);
	return;
}

static struct tegra_w1_timings tegra_w1_platform_timings = {
        .tsu = 0x1,
        .trelease = 0xf,
        .trdv = 0xf,
        .tlow0 = 0x3c,
        .tlow1 = 0x1,
        .tslot = 0x77,

        .tpdl = 0x3c,
        .tpdh = 0x1e,
        .trstl = 0x1df,
        .trsth = 0x1df,

        .rdsclk = 0x7,
        .psclk = 0x50,
};

static struct tegra_w1_platform_data tegra_w1_pdata = {
        .clk_id = NULL,
        .timings = &tegra_w1_platform_timings,
};

static void olympus_w1_init(void)
{
        tegra_w1_device.dev.platform_data = &tegra_w1_pdata;
        platform_device_register(&tegra_w1_device);
}

#if 0
static int cpcap_usb_connected_probe(struct platform_device *pdev)
{
/*	struct cpcap_accy_platform_data *pdata = pdev->dev.platform_data;*/

	int nr_gpio;
	int ret;

	nr_gpio = 174;

	tegra_gpio_enable(nr_gpio);
    	ret = gpio_request(nr_gpio, "usb_data_en");

    	printk(KERN_INFO "pICS_%s: gpio_request(nr_gpio=%i, 'usb_data_en');\n",__func__, nr_gpio);

	gpio_direction_output(nr_gpio, 0);	
	gpio_set_value(nr_gpio, 1);
#if 0
	platform_set_drvdata(pdev, pdata);

	/* when the phone is the host do not start the gadget driver */
	if((pdata->accy == CPCAP_ACCY_USB) || (pdata->accy == CPCAP_ACCY_FACTORY)) {
		tegra_otg_set_mode(0);
		android_usb_set_connected(1, pdata->accy);
	}
	if(pdata->accy == CPCAP_ACCY_USB_DEVICE) {
		tegra_otg_set_mode(1);
	}

//	mdm_ctrl_set_usb_ipc(true);
#endif
	return 0;
}

static int cpcap_usb_connected_remove(struct platform_device *pdev)
{
/*	struct cpcap_accy_platform_data *pdata = pdev->dev.platform_data;*/

	int nr_gpio;

//	mdm_ctrl_set_usb_ipc(false);
	
	nr_gpio = 174;
	gpio_set_value(nr_gpio, 0);
printk(KERN_INFO "pICS_%s: nr_gpio=%i, 'usb_data_en';\n",__func__, nr_gpio);
	gpio_free(nr_gpio);
	
	tegra_gpio_disable(nr_gpio);
/*
	if((pdata->accy == CPCAP_ACCY_USB) || (pdata->accy == CPCAP_ACCY_FACTORY))
		android_usb_set_connected(0, pdata->accy);

	tegra_otg_set_mode(2);
*/
        return 0;
}

struct platform_driver cpcap_usb_connected_driver = {
        .probe          = cpcap_usb_connected_probe,
        .remove         = cpcap_usb_connected_remove,
        .driver         = {
                .name   = "cpcap_usb_connected",
                .owner  = THIS_MODULE,
    },
};
#endif

static int config_unused_pins(char *pins, int num)
{
        int i, ret = 0;
        
        pr_info("%s: ENTRY\n", __func__);

        for (i = 0; i < num; i++) {
                ret = gpio_request(pins[i], "unused");
                if (ret) {
                        printk(KERN_ERR "%s: Error (%d) - gpio_reqest failed for unused GPIO %d\n", __func__,ret, pins[i]);
                } else {                
                        ret = gpio_direction_output(pins[i], 1);
                        if (ret) {
                                printk(KERN_ERR "%s: Error (%d)- gpio_direction failed for unused GPIO %d\n", __func__,ret, pins[i]);
                        }
                }
        }
        
        pr_info("%s: EXIT\n", __func__);

        return ret;
}

static void __init tegra_olympus_init(void)
{
	
	tegra_clk_init_from_table(olympus_clk_init_table);

	tegra_ram_console_debug_init();

	olympus_pinmux_init();

	olympus_i2c_init();

	olympus_uart_init();

	olympus_spi_init();

	olympus_audio_init();

	olympus_devices_init();

	olympus_power_init();
	
	olympus_sdhci_init();

	olympus_tcmd_init();

	olympus_sensors_init();

	olympus_sec_init();

	olympus_backlight_init();

	olympus_panel_init();

	olympus_kbc_init();

	olympus_touch_init();

	olympus_bt_rfkill();

	olympus_usb_init();

	olympus_camera_init();

//	olympus_emc_init();

	olympus_modem_init();
	olympus_wlan_init();

//	platform_driver_register(&cpcap_usb_connected_driver);

	olympus_w1_init();
	
	//tegra_setup_bluesleep();

	/* Configure SPDIF_OUT as GPIO by default, it can be later controlled
	   as needed. When SPDIF_OUT is enabled and if HDMI is connected, it
	   can interefere with CPCAP ID pin, as SPDIF_OUT and ID are coupled.
	*/

	tegra_gpio_enable(TEGRA_GPIO_PD4);
	gpio_request(TEGRA_GPIO_PD4, "spdif_enable");
	gpio_direction_output(TEGRA_GPIO_PD4, 0);
	gpio_export(TEGRA_GPIO_PD4, false);
/*	if ((HWREV_TYPE_IS_PORTABLE(system_rev) || HWREV_TYPE_IS_FINAL(system_rev)))
		{
			if (HWREV_REV(system_rev) >= HWREV_REV_1 && HWREV_REV(system_rev) < HWREV_REV_2)
			{
				printk("%s: Olympus P1\n", __func__);
				// Olympus P1
				config_unused_pins(oly_unused_pins_p1, ARRAY_SIZE(oly_unused_pins_p1));
			}
			else if (HWREV_REV(system_rev) >= HWREV_REV_2 && HWREV_REV(system_rev) < HWREV_REV_3)
			{
				printk("%s: Olympus P2\n", __func__);
				// Olympus P2
				config_unused_pins(oly_unused_pins_p2, ARRAY_SIZE(oly_unused_pins_p2));
			}
			else if (HWREV_REV(system_rev) >= HWREV_REV_3 || HWREV_TYPE_IS_FINAL(system_rev))
			{
				printk("%s: Olympus P3 and newer\n", __func__);
			// Olympus P3 and newer
				config_unused_pins(oly_unused_pins_p3, ARRAY_SIZE(oly_unused_pins_p3));
			}
		}*/
	tegra_release_bootloader_fb();	
}

static void __init olympus_fixup(struct machine_desc *desc, struct tag *tags,
                 char **cmdline, struct meminfo *mi)
{
	struct tag *t;
	int i;

	/*
	 * Dump some key ATAGs
	 */
	for (t=tags; t->hdr.size; t = tag_next(t)) {
		switch (t->hdr.tag) {
		case ATAG_WLAN_MAC:        // 57464d41 parsed in board-mot-wlan.c
		case ATAG_BLDEBUG:         // 41000811 same, in board-mot-misc.c
		case ATAG_POWERUP_REASON:  // F1000401 ex: 0x4000, parsed after... ignore
			break;
		case ATAG_CORE:     // 54410001
			printk("%s: atag_core hdr.size=%d\n", __func__, t->hdr.size);
			break;
		case ATAG_CMDLINE:
			printk("%s: atag_cmdline=\"%s\"\n", __func__, t->u.cmdline.cmdline);
			break;
		case ATAG_REVISION: // 54410007
			printk("%s: atag_revision=0x%x\n", __func__, t->u.revision.rev);
			break;
		case ATAG_SERIAL:   // 54410006
			printk("%s: atag_serial=%x%x\n", __func__, t->u.serialnr.low, t->u.serialnr.high);
			break;
		case ATAG_INITRD2:  // 54420005
			printk("%s: atag_initrd2=0x%x size=0x%x\n", __func__, t->u.initrd.start, t->u.initrd.size);
			break;
		case ATAG_MEM:
			printk("%s: atag_mem.start=0x%x, mem.size=0x%x\n", __func__, t->u.mem.start, t->u.mem.size);
			break;
#ifdef CONFIG_MACH_OLYMPUS
		case ATAG_MOTOROLA: // 41000810
			printk("%s: atag_moto allow_fb=%d\n", __func__, t->u.motorola.allow_fb_open);
			break;
#endif
		case ATAG_NVIDIA_TEGRA: // 41000801
			printk("%s: atag_tegra=0x%X\n", __func__, t->u.tegra.bootarg_key);
			break;
		default:
			printk("%s: ATAG %X\n", __func__, t->hdr.tag);
		}
	}
	/*
	 * Dump memory nodes
	 */
	for (i=0; i<mi->nr_banks; i++) {
		printk("%s: bank[%d]=%lx@%lx\n", __func__, i, mi->bank[i].size, (long unsigned int)(mi->bank[i].start));
	}
}

int __init olympus_protected_aperture_init(void)
{
	tegra_protected_aperture_init(tegra_grhost_aperture);
	return 0;
}
late_initcall(olympus_protected_aperture_init);

void __init tegra_olympus_reserve(void)
{
//	long ret;

	if (memblock_reserve(0x0, 4096) < 0)
		pr_warn("Cannot reserve first 4K of memory for safety\n");

	tegra_reserve(SZ_128M + SZ_64M, SZ_8M, SZ_8M);
	tegra_ram_console_debug_reserve(SZ_1M);

}

MACHINE_START(OLYMPUS, "Olympus")

    .boot_params  = 0x00000100,
    .fixup        = olympus_fixup,
    .map_io       = tegra_map_common_io,
    .reserve	  = tegra_olympus_reserve,
    .init_early	  = tegra_init_early,
    .init_irq     = tegra_init_irq,
    .timer        = &tegra_timer,
    .init_machine = tegra_olympus_init,

MACHINE_END

