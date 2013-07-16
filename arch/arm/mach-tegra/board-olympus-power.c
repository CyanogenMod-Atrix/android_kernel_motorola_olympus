/*
 * arch/arm/mach-tegra/board-olympus-power.c
 *
 * Copyright 2013: Olympus Kernel Project
 * <http://forum.xda-developers.com/showthread.php?t=2016837>
 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds-ld-cpcap.h>
#include <linux/clk.h>
#include <linux/cpcap-accy.h>
#include <linux/cpcap_audio_platform_data.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/mdm_ctrl.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/machine.h>
#include <linux/reboot.h>
#include <linux/rtc.h>
#include <linux/spi/cpcap.h>
#include <linux/spi/cpcap-regbits.h>
#include <linux/spi/spi.h>
#include <linux/spi-tegra.h>

#include <asm/mach-types.h>
#include <mach/iomap.h>
#include "apbio.h"
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/setup.h>
#include <asm/bootinfo.h>
#include <mach/io.h>
#include <mach/system.h>
#include <asm/cacheflush.h>
#include <linux/cpu.h>

#include <mach/iomap.h>
#include <mach/irqs.h>

#include "gpio-names.h"
#include "board.h"
#include "hwrev.h"
#include "pm.h"
#include <mach/io.h>
#include <mach/iomap.h>
#include "apbio.h"
#include "pm-irq.h"
#include "fuse.h"
#include "wakeups-t2.h"

#include "board-olympus.h"

#define PWRUP_FACTORY_CABLE         0x00000020 /* Bit 5  */
#define PWRUP_INVALID               0xFFFFFFFF

#define PMC_CTRL		0x0
#define PMC_CTRL_INTR_LOW	(1 << 17)

static int disable_rtc_alarms(struct device *dev, void *data)
{
	return (rtc_alarm_irq_enable((struct rtc_device *)dev, 0));
}

void olympus_pm_restart(char mode, const char *cmd)
{
	disable_nonboot_cpus();
	flush_cache_all();
	outer_disable();
	arm_machine_restart(mode,cmd);
}
/*
void tegra_system_power_off(void)
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
}*/

void olympus_system_power_off(void)
{
	/* If there's external power, let's restart instead ...
	   except for the case when phone was powered on with factory cable
	   and thus has to stay powered off after Turn-Off TCMD INKVSSW-994 */
	if (cpcap_misc_is_ext_power() &&
	   !((bi_powerup_reason() & PWRUP_FACTORY_CABLE) &&
	     (bi_powerup_reason() != PWRUP_INVALID)) )
	{
		pr_info("%s: external power detected: rebooting\n", __func__);
		olympus_pm_restart(0, "");
		while(1);
	}

	printk(KERN_ERR "%s: powering down system\n", __func__);

	/* Disable RTC alarms to prevent unwanted powerups */
	class_for_each_device(rtc_class, NULL, NULL, disable_rtc_alarms);

	/* Disable powercut detection before power off */
	cpcap_disable_powercut();

	/* We need to set the WDI bit low to power down normally */
	if (machine_is_olympus())
	{
		if (HWREV_TYPE_IS_PORTABLE(system_rev) &&
		    HWREV_REV(system_rev) >= HWREV_REV_1 &&
		    HWREV_REV(system_rev) <= HWREV_REV_1C )
		{
			/* Olympus P1 */
			gpio_request(TEGRA_GPIO_PT4, "P1 WDI");
			gpio_direction_output(TEGRA_GPIO_PT4, 1);
			gpio_set_value(TEGRA_GPIO_PT4, 0);
		}
		else
		{
			/* Olympus Mortable, P0, P2 and later */
			gpio_request(TEGRA_GPIO_PV7, "P2 WDI");
			gpio_direction_output(TEGRA_GPIO_PV7, 1);
			gpio_set_value(TEGRA_GPIO_PV7, 0);
		}
	}
	else
	{
		printk(KERN_ERR "Could not poweroff.  Unknown hardware "
				"revision: 0x%x\n", system_rev);
	}

	mdelay(500);
	printk("Power-off failed (Factory cable inserted?), rebooting\r\n");
	olympus_pm_restart(0, "");
}

static struct cpcap_device *cpcap_di;

static int cpcap_validity_reboot(struct notifier_block *this,
				 unsigned long code, void *cmd)
{
	int ret = -1;
	int result = NOTIFY_DONE;
	char *mode = cmd;

	switch (code) {
		case SYS_RESTART:
		case SYS_HALT:
		case SYS_POWER_OFF:
			result = NOTIFY_OK;
	}

	return result;
}

static struct notifier_block validity_reboot_notifier = {
	.notifier_call = cpcap_validity_reboot,
};

static int cpcap_validity_probe(struct platform_device *pdev)
{
	int err = 0;

	if (pdev->dev.platform_data == NULL) {
		dev_err(&pdev->dev, "no platform_data\n");
		return -EINVAL;
	}

	cpcap_di = pdev->dev.platform_data;

#ifdef CONFIG_BOOTINFO
	if (bi_powerup_reason() != PU_REASON_CHARGER) {
		/* Set Kpanic bit, which will be cleared at normal reboot */
		cpcap_regacc_write(cpcap_di, CPCAP_REG_VAL1,
				   CPCAP_BIT_AP_KERNEL_PANIC,
				   CPCAP_BIT_AP_KERNEL_PANIC);
	}
#endif

	register_reboot_notifier(&validity_reboot_notifier);

#ifdef CONFIG_MFD_CPCAP_SOFTRESET
	/* Enable workaround to allow soft resets to work */
	cpcap_regacc_write(cpcap_di, CPCAP_REG_PGC,
			   CPCAP_BIT_SYS_RST_MODE, CPCAP_BIT_SYS_RST_MODE);
	err = cpcap_uc_start(cpcap_di,CPCAP_BANK_PRIMARY, CPCAP_MACRO_15);
	dev_info(&pdev->dev, "Started macro 15: %d\n", err);
#endif

	return err;
}

static int cpcap_validity_remove(struct platform_device *pdev)
{
	unregister_reboot_notifier(&validity_reboot_notifier);
	cpcap_di = NULL;

	return 0;
}

static struct platform_driver cpcap_validity_driver = {
	.probe = cpcap_validity_probe,
	.remove = cpcap_validity_remove,
	.driver = {
		.name = "cpcap_validity",
		.owner  = THIS_MODULE,
	},
};

static struct platform_device cpcap_validity_device = {
	.name   = "cpcap_validity",
	.id     = -1,
	.dev    = {
		.platform_data  = NULL,
	},
};

static struct platform_device cpcap_3mm5_device = {
	.name   = "cpcap_3mm5",
	.id     = -1,
	.dev    = {
		.platform_data  = NULL,
	},
};

static struct platform_device cpcap_batt_device = {
	.name   = "cpcap_battery",
        .id     = -1,
        .dev    = {
                .platform_data  = NULL,
        },
};

static struct platform_device cpcap_audio_device = {
	.name   = "cpcap_audio",
	.id     = -1,
	.dev    = {
		.platform_data = NULL,
	},
};

static struct platform_device cpcap_usb_det_device = {
	.name           = "cpcap_usb_det",
	.id             = -1,
	.dev    = {
		.platform_data  = NULL,
	},
};

static struct platform_device cpcap_wdt_device = {
	.name           = "cpcap_wdt",
	.id             = -1,
	.dev.platform_data = NULL,
};

struct platform_device cpcap_disp_button_led = {
	.name = LD_DISP_BUTTON_DEV,
	.id	= -1,
	.dev = {
		.platform_data  = NULL,
	},
};

struct platform_device cpcap_rgb_led = {
	.name = LD_MSG_IND_DEV,
	.id	= -1,
	.dev = {
		.platform_data  = NULL,
	},
};

static struct platform_device *cpcap_devices[] = {
	&cpcap_validity_device,
	&cpcap_rgb_led,
	&cpcap_disp_button_led,
	&cpcap_3mm5_device,
	&cpcap_audio_device,
	&cpcap_usb_det_device,
	&cpcap_batt_device,
	/*
#ifdef CONFIG_CPCAP_WATCHDOG
	&cpcap_wdt_device,
#endif*/
};

static int is_olympus_ge_p0(struct cpcap_device *cpcap)
{
//	printk(KERN_INFO "pICS_%s: testing...\n",__func__);
	return 1;
}

static int is_olympus_ge_p3(struct cpcap_device *cpcap)
{
//	printk(KERN_INFO "pICS_%s: testing...\n",__func__);
	if (HWREV_TYPE_IS_FINAL(system_rev) ||
		(HWREV_TYPE_IS_PORTABLE(system_rev) &&
		 (HWREV_REV(system_rev) >= HWREV_REV_3))) {
		return 1;
	}
	return 0;
}

enum cpcap_revision cpcap_get_revision(struct cpcap_device *cpcap)
{
	unsigned short value;

//	printk(KERN_INFO "pICS_%s: testing...\n",__func__);

	/* Code taken from drivers/mfd/cpcap_core.c, since the revision value
	   is not initialized until after the registers are initialized, which
	   will happen after the trgra_cpcap_spi_init table is used. */
	(void)cpcap_regacc_read(cpcap, CPCAP_REG_VERSC1, &value);
	return (enum cpcap_revision)(((value >> 3) & 0x0007) |
						((value << 3) & 0x0038));
}

int is_cpcap_eq_3_1(struct cpcap_device *cpcap)
{
//	printk(KERN_INFO "pICS_%s: testing...\n",__func__);
	return cpcap_get_revision(cpcap) == CPCAP_REVISION_3_1;
}

struct cpcap_spi_init_data tegra_cpcap_spi_init[] = {
	/* Set SW1 to AMS/AMS 1.025v. */
	{CPCAP_REG_S1C1,      0x4822, NULL             },
	/* Set SW2 to AMS/AMS 1.2v. */
	{CPCAP_REG_S2C1,      0x4830, NULL             }, 
	/* Set SW3 to AMS/AMS 1.8V for version 3.1 only. */
	{CPCAP_REG_S3C,       0x0445, is_cpcap_eq_3_1  },
	/* Set SW3 to Pulse Skip/PFM. */
	{CPCAP_REG_S3C,       0x043d, NULL             },
	/* Set SW4 to AMS/AMS 1.2v for version 3.1 only. */
	{CPCAP_REG_S4C1,      0x4830, is_cpcap_eq_3_1  },
	/* Set SW4 to PFM/PFM 1.2v. */
	{CPCAP_REG_S4C1,      0x4930, NULL             },
	/* Set SW4 down to 0.95v when secondary standby is asserted. */
	{CPCAP_REG_S4C2,      0x301c, NULL             },
	/* Set SW5 to On/Off. */
	{CPCAP_REG_S5C,       0x0020, NULL             },
	/* Set SW6 to Off/Off. */
	{CPCAP_REG_S6C,       0x0000, NULL             },
	/* Set VCAM to Off/Off. */
	{CPCAP_REG_VCAMC,     0x0030, NULL             },
	/* Set VCSI to AMS/Off 1.2v. */
	{CPCAP_REG_VCSIC,     0x0007, NULL             },
	{CPCAP_REG_VDACC,     0x0000, NULL             },
	{CPCAP_REG_VDIGC,     0x0000, NULL             },
	{CPCAP_REG_VFUSEC,    0x0000, NULL             },
	/* Set VHVIO to On/LP. */
	{CPCAP_REG_VHVIOC,    0x0002, NULL             },
	/* Set VSDIO to On/LP. */
	{CPCAP_REG_VSDIOC,    0x003A, NULL             },
	/* Set VPLL to On/Off. */
	{CPCAP_REG_VPLLC,     0x0019, NULL             },
	{CPCAP_REG_VRF1C,     0x0002, NULL             },
	{CPCAP_REG_VRF2C,     0x0000, NULL             },
	{CPCAP_REG_VRFREFC,   0x0000, NULL             },
	/* Set VWLAN1 to off */
	{CPCAP_REG_VWLAN1C,   0x0000, NULL             },
	{CPCAP_REG_VWLAN1C,   0x0000, NULL             },
	{CPCAP_REG_VWLAN1C,   0x0000, is_olympus_ge_p3 },
	{CPCAP_REG_VWLAN1C,   0x0000, NULL             },
	/* Set VWLAN1 to AMS/AMS 1.8v */
	{CPCAP_REG_VWLAN1C,   0x0005, NULL             },
	/* Set VWLAN2 to On/LP 3.3v. */
	{CPCAP_REG_VWLAN2C,   0x0089, is_olympus_ge_p3 },
	{CPCAP_REG_VWLAN2C,   0x0089, NULL             },
	/* Set VWLAN2 to On/On 3.3v */
	{CPCAP_REG_VWLAN2C,   0x008d, NULL             },
	/* Set VSIMCARD to AMS/Off 2.9v. */
	{CPCAP_REG_VSIMC,     0x1e08, NULL             },
	/* Set to off 3.0v. */
	{CPCAP_REG_VVIBC,     0x000C, NULL             },
	/* Set VUSB to On/On */
	{CPCAP_REG_VUSBC,     0x004C, NULL             },
	{CPCAP_REG_VUSBINT1C, 0x0000, NULL             },
	{CPCAP_REG_USBC1,     0x1201, NULL             },
	{CPCAP_REG_USBC2,     0xC058, NULL             },
	{CPCAP_REG_USBC3,     0x7DFF, NULL             },
	/* one wire level shifter */
	{CPCAP_REG_OWDC,      0x0003, NULL             },
	/* power cut is enabled, the timer is set to 312.5 ms */
	{CPCAP_REG_PC1,       0x010A, NULL             },
	/* power cut counter is enabled to prevent ambulance mode */
	{CPCAP_REG_PC2,       0x0150, NULL             },
	/* Enable coin cell charger and set charger voltage to 3.0 V
	   Enable coulomb counter, enable dithering and set integration
	   period to 250 mS*/
	{CPCAP_REG_CCCC2,     0x002B, NULL             },
    /* Set ADC_CLK to 3 MHZ
       Disable leakage currents into channels between ADC
	   conversions */
	{CPCAP_REG_ADCC1,     0x9000, NULL             },
	/* Disable TS_REF
	   Enable coin cell charger input to A/D
	   Ignore ADTRIG signal
	   THERMBIAS pin is open circuit
	   Use B+ for ADC channel 4, Bank 0
	   Enable BATDETB comparator
	   Do not apply calibration offsets to ADC current readings */
	{CPCAP_REG_ADCC2,     0x4136, NULL             },
	/* Clear UC Control 1 */
	{CPCAP_REG_UCC1,      0x0000, NULL             },
};

struct cpcap_leds tegra_cpcap_leds = {
	.button_led = {
		.button_reg = CPCAP_REG_KLC,
		.button_mask = 0x03FF,
		.button_on = 0xFFFF,
		.button_off = 0x0000,
		.regulator = "sw5",  /* set to NULL below for products with button LED on B+ */
	},
	.rgb_led = {
		.rgb_on = 0x0053,
		.regulator = "sw5",  /* set to NULL below for products with RGB LED on B+ */
		.regulator_macro_controlled = false,
	},
};

extern struct platform_device cpcap_disp_button_led;
extern struct platform_device cpcap_rgb_led;

struct cpcap_mode_value *cpcap_regulator_mode_values[] = {
	[CPCAP_SW1] = (struct cpcap_mode_value []) {
		/* AMS/AMS Primary control via Macro */
		{0x6800, NULL }
	},
	[CPCAP_SW2] = (struct cpcap_mode_value []) {
		/* AMS/AMS Secondary control via Macro */
		{0x4804, NULL }
	},
	[CPCAP_SW3] = (struct cpcap_mode_value []) {
		/* AMS/AMS Secondary Standby */
		{0x0040, is_cpcap_eq_3_1  },
		/* Pulse Skip/PFM Secondary Standby */
		{0x043c, NULL             },
	},
	[CPCAP_SW4] = (struct cpcap_mode_value []) {
		/* AMS/AMS Secondary Standby */
		{ 0x0800, is_cpcap_eq_3_1  },
		/* PFM/PFM Secondary Standby */
		{ 0x4909, NULL             },
	},
	[CPCAP_SW5] = (struct cpcap_mode_value []) {
		{ 0x0020, NULL             },
		{ CPCAP_REG_OFF_MODE_SEC | 0x0020, NULL             },
		{ CPCAP_REG_OFF_MODE_SEC | 0x0020, is_olympus_ge_p0 },
		{ CPCAP_REG_OFF_MODE_SEC | 0x0020, NULL             },
		/* On/Off */
		{ 0x0020, NULL             },
	},
	[CPCAP_VCAM] = (struct cpcap_mode_value []) {
		/* AMS/Off Secondary Standby */
		{0x0007, NULL }
	},
	[CPCAP_VCSI] = (struct cpcap_mode_value []) {
		/* AMS/Off Secondary Standby */
		{0x0007, NULL }
	},
	[CPCAP_VDAC] = (struct cpcap_mode_value []) {
		/* Off/Off */
		{0x0000, NULL }
	},
	[CPCAP_VDIG] = (struct cpcap_mode_value []) {
		/* Off/Off */
		{0x0000, NULL }
	},
	[CPCAP_VFUSE] = (struct cpcap_mode_value []) {
		/* Off/Off */
		{0x0000, NULL }
	},
	[CPCAP_VHVIO] = (struct cpcap_mode_value []) {
		/* On/LP  Secondary Standby */
		{0x0002, NULL }
	},
	[CPCAP_VSDIO] = (struct cpcap_mode_value []) {
		/* On/LP  Secondary Standby */
		{0x0002, NULL }
	},
	[CPCAP_VPLL] = (struct cpcap_mode_value []) {
		/* On/Off Secondary Standby */
		{0x0001, NULL }
	},
	[CPCAP_VRF1] = (struct cpcap_mode_value []) {
		/* Off/Off */
		{0x0000, NULL }
	},
	[CPCAP_VRF2] = (struct cpcap_mode_value []) {
		/* Off/Off */
		{0x0000, NULL }
	},
	[CPCAP_VRFREF] = (struct cpcap_mode_value []) {
		/* Off/Off */
		{0x0000, NULL  }
	},
	[CPCAP_VWLAN1] = (struct cpcap_mode_value []) {
		/* Off/Off */
		{0x0000, NULL             },
		{0x0000, NULL             },
		{0x0000, is_olympus_ge_p3 },
		{0x0000, NULL             },
		/* AMS/AMS. */
		{0x0005, NULL             },
	},
	[CPCAP_VWLAN2] = (struct cpcap_mode_value []) {
		/* On/LP 3.3v Secondary Standby (external pass) */
		{0x0009, is_olympus_ge_p3 },
		{0x0009, NULL             },
		/* On/On 3.3v (external pass) */
		{0x000D, NULL             },
	},
	[CPCAP_VSIM] = (struct cpcap_mode_value []) {
		/* Off/Off */
		{0x0000, NULL }
	},
	[CPCAP_VSIMCARD] = (struct cpcap_mode_value []) {
		/* AMS/Off Secondary Standby */
		{0x1E00, NULL }
	},
	[CPCAP_VVIB] = (struct cpcap_mode_value []) {
		/* On */
		{0x0001, NULL }
	},
	[CPCAP_VUSB] = (struct cpcap_mode_value []) {
		/* On/On */
		{0x000C, NULL }
	},
	[CPCAP_VAUDIO] = (struct cpcap_mode_value []) {
		/* On/LP Secondary Standby */
		{0x0005, NULL }
	},
};

struct cpcap_mode_value *cpcap_regulator_off_mode_values[] = {
	[CPCAP_SW1] = (struct cpcap_mode_value []) {
		{0x0000, NULL }
	},
	[CPCAP_SW2] = (struct cpcap_mode_value []) {
		{0x0000, NULL }
	},
	[CPCAP_SW3] = (struct cpcap_mode_value []) {
		{0x0000, NULL }
	},
	[CPCAP_SW4] = (struct cpcap_mode_value []) {
		{0x0000, NULL }
	},
	[CPCAP_SW5] = (struct cpcap_mode_value []) {
		{0x0000, NULL }
	},
	[CPCAP_VCAM] = (struct cpcap_mode_value []) {
		{0x0000, NULL }
	},
	[CPCAP_VCSI] = (struct cpcap_mode_value []) {
		{0x0000, NULL }
	},
	[CPCAP_VDAC] = (struct cpcap_mode_value []) {
		{0x0000, NULL }
	},
	[CPCAP_VDIG] = (struct cpcap_mode_value []) {
		{0x0000, NULL }
	},
	[CPCAP_VFUSE] = (struct cpcap_mode_value []) {
		{0x0000, NULL }
	},
	[CPCAP_VHVIO] = (struct cpcap_mode_value []) {
		{0x0000, NULL }
	},
	[CPCAP_VSDIO] = (struct cpcap_mode_value []) {
		{0x0000, NULL }
	},
	[CPCAP_VPLL] = (struct cpcap_mode_value []) {
		{0x0000, NULL }
	},
	[CPCAP_VRF1] = (struct cpcap_mode_value []) {
		{0x0000, NULL }
	},
	[CPCAP_VRF2] = (struct cpcap_mode_value []) {
		{0x0000, NULL }
	},
	[CPCAP_VRFREF] = (struct cpcap_mode_value []) {
		{0x0000, NULL }
	},
	[CPCAP_VWLAN1] = (struct cpcap_mode_value []) {
		{0x0000, NULL }
	},
	[CPCAP_VWLAN2] = (struct cpcap_mode_value []) {
		/* Turn off only once sec standby is entered. */
		{0x0004, is_olympus_ge_p3 },
		{0x0004, NULL             },
		{0x0000, NULL             },
	},
	[CPCAP_VSIM] = (struct cpcap_mode_value []) {
		{0x0000, NULL }
	},
	[CPCAP_VSIMCARD] = (struct cpcap_mode_value []) {
		{0x0000, NULL }
	},
	[CPCAP_VVIB] = (struct cpcap_mode_value []) {
		{0x0000, NULL }
	},
	[CPCAP_VUSB] = (struct cpcap_mode_value []) {
		{0x0000, NULL }
	},
	[CPCAP_VAUDIO] = (struct cpcap_mode_value []) {
		{0x0000, NULL }
	},
};

#define REGULATOR_CONSUMER(name, device) { .supply = name, .dev = device, }
#define REGULATOR_CONSUMER_BY_DEVICE(name, device) \
	{ .supply = name, .dev = device, }

struct regulator_consumer_supply cpcap_sw1_consumers[] = {
	REGULATOR_CONSUMER("vdd_cpu", NULL), /* core */
};

struct regulator_consumer_supply cpcap_sw2_consumers[] = {
	REGULATOR_CONSUMER("vdd_core", NULL),/* core */
};

struct regulator_consumer_supply cpcap_sw3_consumers[] = {
	REGULATOR_CONSUMER("sw3", NULL ),/* VIO */
	REGULATOR_SUPPLY("vddio_sys", NULL),
};

struct regulator_consumer_supply cpcap_sw4_consumers[] = {
	REGULATOR_CONSUMER("vdd_aon", NULL),/* core */
};

struct regulator_consumer_supply cpcap_sw5_consumers[] = {
	REGULATOR_SUPPLY("sw5", "button-backlight"),
	REGULATOR_SUPPLY("sw5", "notification-led"),
	REGULATOR_SUPPLY("sw5", NULL),
};

struct regulator_consumer_supply cpcap_vcam_consumers[] = {
	REGULATOR_CONSUMER("vcam", NULL /* cpcap_cam_device */),
	REGULATOR_CONSUMER("vdd_cam1", NULL),
};

struct regulator_consumer_supply cpcap_vhvio_consumers[] = {
	REGULATOR_CONSUMER("vhvio", NULL /* lighting_driver */),
	REGULATOR_CONSUMER("vddio_mipi", NULL /* Camera */),
	REGULATOR_CONSUMER("avdd_hdmi_pll", NULL /* HDMI */),
//	REGULATOR_CONSUMER("vhvio", NULL /* lighting_driver */),
//	REGULATOR_CONSUMER("vhvio", NULL /* magnetometer */),
//	REGULATOR_CONSUMER("vhvio", NULL /* light sensor */),
//	REGULATOR_CONSUMER("vhvio", NULL /* display */),
};

struct regulator_consumer_supply cpcap_vsdio_consumers[] = {
	REGULATOR_CONSUMER("vsdio", NULL),

};

struct regulator_consumer_supply cpcap_vpll_consumers[] = {
	REGULATOR_CONSUMER("vpll", NULL),
};

struct regulator_consumer_supply cpcap_vcsi_consumers[] = {
	REGULATOR_CONSUMER("vcsi", NULL),
};

struct regulator_consumer_supply cpcap_vwlan1_consumers[] = {
	REGULATOR_CONSUMER("vwlan1", NULL),
};

struct regulator_consumer_supply cpcap_vwlan2_consumers[] = {
	REGULATOR_CONSUMER("vwlan2", NULL),
//	REGULATOR_CONSUMER("vddio_sdmmc", "sdhci-tegra.0"),
	/* Powers the tegra usb block, cannot be named vusb, since
	   this name already exists in regulator-cpcap.c. */
	REGULATOR_CONSUMER("avdd_usb", NULL), /* usb */
	REGULATOR_CONSUMER("vusb_modem_flash", NULL),
	REGULATOR_CONSUMER("vusb_modem_ipc", NULL),
	REGULATOR_CONSUMER("avdd_hdmi", NULL),
	REGULATOR_CONSUMER("vdd_vcore_temp", NULL),
};

struct regulator_consumer_supply cpcap_vsimcard_consumers[] = {
	REGULATOR_CONSUMER("vsimcard", NULL /* sd slot */),
};

struct regulator_consumer_supply cpcap_vvib_consumers[] = {
	REGULATOR_CONSUMER("vvib", NULL /* vibrator */),
};

struct regulator_consumer_supply cpcap_vusb_consumers[] = {
	REGULATOR_CONSUMER_BY_DEVICE("vusb", &cpcap_usb_det_device.dev),
};

struct regulator_consumer_supply cpcap_vaudio_consumers[] = {
	REGULATOR_CONSUMER("vaudio", NULL /* mic opamp */),
};

static struct regulator_init_data cpcap_regulator[CPCAP_NUM_REGULATORS] = {
	[CPCAP_SW1] = {
		.constraints = {
			.min_uV			= 750000,
			.max_uV			= 1475000,
			.valid_ops_mask		= REGULATOR_CHANGE_STATUS |
                                                  REGULATOR_CHANGE_VOLTAGE,
       //     .always_on		= 1,
		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_sw1_consumers),
		.consumer_supplies	= cpcap_sw1_consumers,
	},
	[CPCAP_SW2] = {
		.constraints = {
			.min_uV			= 900000,
			.max_uV			= 1475000,
			.valid_ops_mask		= REGULATOR_CHANGE_STATUS |
                                                  REGULATOR_CHANGE_VOLTAGE,
          //  .always_on		= 1,
		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_sw2_consumers),
		.consumer_supplies	= cpcap_sw2_consumers,
	},
	[CPCAP_SW3] = {
		.constraints = {
			.min_uV			= 1350000,
			.max_uV			= 1875000,
			.valid_ops_mask		= REGULATOR_CHANGE_STATUS |
                                                 REGULATOR_CHANGE_VOLTAGE,
        //    .always_on		= 1,
		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_sw3_consumers),
		.consumer_supplies	= cpcap_sw3_consumers,
	},
	[CPCAP_SW4] = {
		.constraints = {
			.min_uV			= 900000,
			.max_uV			= 1475000,
			.valid_ops_mask		= REGULATOR_CHANGE_STATUS |
                                                  REGULATOR_CHANGE_VOLTAGE,
         //   .always_on		= 1,
		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_sw4_consumers),
		.consumer_supplies	= cpcap_sw4_consumers,
	},
	[CPCAP_SW5] = {
		.constraints = {
			.min_uV			= 5050000,
			.max_uV			= 5050000,
			.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
			.apply_uV		= 1,
		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_sw5_consumers),
		.consumer_supplies	= cpcap_sw5_consumers,
	},
	[CPCAP_VCAM] = {
		.constraints = {
			.min_uV			= 2600000,
			.max_uV			= 2900000,
			.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_vcam_consumers),
		.consumer_supplies	= cpcap_vcam_consumers,
	},
	[CPCAP_VCSI] = {
		.constraints = {
			.min_uV			= 1200000,
			.max_uV			= 1200000,
			.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
			.boot_on		= 1,
			.apply_uV		= 1,
		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_vcsi_consumers),
		.consumer_supplies	= cpcap_vcsi_consumers,
	},
	[CPCAP_VDAC] = {
		.constraints = {
			.min_uV			= 1200000,
			.max_uV			= 2500000,
			.valid_ops_mask		= 0,
		},
	},
	[CPCAP_VDIG] = {
		.constraints = {
			.min_uV			= 1200000,
			.max_uV			= 1875000,
			.valid_ops_mask		= 0,
		},
	},
	[CPCAP_VFUSE] = {
		.constraints = {
			.min_uV			= 1500000,
			.max_uV			= 3150000,
			.valid_ops_mask		= 0,
		},
	},
	[CPCAP_VHVIO] = {
		.constraints = {
			.min_uV			= 2775000,
			.max_uV			= 2775000,
			.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
			.always_on		= 1,
			.apply_uV		= 1,
		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_vhvio_consumers),
		.consumer_supplies	= cpcap_vhvio_consumers,
	},
	[CPCAP_VSDIO] = {
		.constraints = {
			.min_uV			= 1500000,
			.max_uV			= 3000000,
			.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
         //   .always_on		= 1,
		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_vsdio_consumers),
		.consumer_supplies	= cpcap_vsdio_consumers,
	},
	[CPCAP_VPLL] = {
		.constraints = {
			.min_uV			= 1800000,
			.max_uV			= 1800000,
			.valid_ops_mask		= 0,
			.apply_uV		= 1,
       //     .always_on		= 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(cpcap_vpll_consumers),
		.consumer_supplies = cpcap_vpll_consumers,
	},
	[CPCAP_VRF1] = {
		.constraints = {
			.min_uV			= 2500000,
			.max_uV			= 2775000,
			.valid_ops_mask		= 0,
		},
	},
	[CPCAP_VRF2] = {
		.constraints = {
			.min_uV			= 2775000,
			.max_uV			= 2775000,
			.valid_ops_mask		= 0,
		},
	},
	[CPCAP_VRFREF] = {
		.constraints = {
			.min_uV			= 2500000,
			.max_uV			= 2775000,
			.valid_ops_mask		= 0,
		},
	},
	[CPCAP_VWLAN1] = {
		.constraints = {
			.min_uV			= 1800000,
			.max_uV			= 1900000,
			.valid_ops_mask		= 0,
		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_vwlan1_consumers),
		.consumer_supplies	= cpcap_vwlan1_consumers,
	},
	[CPCAP_VWLAN2] = {
		.constraints = {
			.min_uV			= 2775000,
			.max_uV			= 3300000,
			.valid_ops_mask		= (REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS),
			.always_on		= 1,  /* Reinitialized based on hwrev in olympus_setup_power() */
		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_vwlan2_consumers),
		.consumer_supplies	= cpcap_vwlan2_consumers,
	},
	[CPCAP_VSIM] = {
		.constraints = {
			.min_uV			= 1800000,
			.max_uV			= 2900000,
			.valid_ops_mask		= 0,
		},
	},
	[CPCAP_VSIMCARD] = {
		.constraints = {
			.min_uV			= 1800000,
			.max_uV			= 2900000,
			.valid_ops_mask		= 0,
		//	.always_on		= 1,
		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_vsimcard_consumers),
		.consumer_supplies	= cpcap_vsimcard_consumers,
	},
	[CPCAP_VVIB] = {
		.constraints = {
			.min_uV			= 1300000,
			.max_uV			= 3000000,
			.valid_ops_mask		= (REGULATOR_CHANGE_VOLTAGE |
						   REGULATOR_CHANGE_STATUS),
		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_vvib_consumers),
		.consumer_supplies	= cpcap_vvib_consumers,
	},
	[CPCAP_VUSB] = {
		.constraints = {
			.min_uV			= 3300000,
			.max_uV			= 3300000,
			.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
			.apply_uV		= 1,
		//	.always_on		= 1,
		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_vusb_consumers),
		.consumer_supplies	= cpcap_vusb_consumers,
	},
	[CPCAP_VAUDIO] = {
		.constraints = {
			.min_uV			= 2775000,
			.max_uV			= 2775000,
			.valid_modes_mask	= (REGULATOR_MODE_NORMAL |
								  REGULATOR_MODE_STANDBY |
								  REGULATOR_MODE_IDLE),
			.valid_ops_mask		= REGULATOR_CHANGE_MODE,
			.always_on		= 1,
			.apply_uV		= 1,
		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_vaudio_consumers),
		.consumer_supplies	= cpcap_vaudio_consumers,
	},
};

/* ADC conversion delays for battery V and I measurments taken in and out of TX burst  */
static struct cpcap_adc_ato cpcap_adc_ato = {
	.ato_in 		= 0x0300,
	.atox_in 		= 0x0000,
	.adc_ps_factor_in 	= 0x0200,
	.atox_ps_factor_in 	= 0x0000,
	.ato_out 		= 0x0780,
	.atox_out 		= 0x0000,
	.adc_ps_factor_out 	= 0x0600,
	.atox_ps_factor_out 	= 0x0000,
};

struct cpcap_platform_data tegra_cpcap_data =
{
	.init = tegra_cpcap_spi_init,
	.init_len = ARRAY_SIZE(tegra_cpcap_spi_init),
	.leds = &tegra_cpcap_leds,
	.regulator_mode_values = cpcap_regulator_mode_values,
	.regulator_off_mode_values = cpcap_regulator_off_mode_values,
	.regulator_init = cpcap_regulator,
	.adc_ato = &cpcap_adc_ato,
	.wdt_disable = 0,
	.usb_changed = NULL,
	.hwcfg = {
		(CPCAP_HWCFG0_SEC_STBY_SW3 |
		 CPCAP_HWCFG0_SEC_STBY_SW4 |
		 CPCAP_HWCFG0_SEC_STBY_VAUDIO |
		 CPCAP_HWCFG0_SEC_STBY_VCAM |
		 CPCAP_HWCFG0_SEC_STBY_VCSI |
		 CPCAP_HWCFG0_SEC_STBY_VHVIO |
		 CPCAP_HWCFG0_SEC_STBY_VPLL |
		 CPCAP_HWCFG0_SEC_STBY_VSDIO),
		(CPCAP_HWCFG1_SEC_STBY_VWLAN1 |    /* WLAN1 may be reset in olympus_setup_power(). */
		 CPCAP_HWCFG1_SEC_STBY_VSIMCARD)},
	.spdif_gpio = TEGRA_GPIO_PD4,
	.uartmux = 1,
	.usbmux_gpio = TEGRA_GPIO_PV6,
};

struct regulator_consumer_supply fixed_sdio_en_consumers[] = {
	REGULATOR_SUPPLY("vddio_sd_slot", "sdhci-tegra.2"),
};

static struct regulator_init_data fixed_sdio_regulator = {
	.constraints = {
		.min_uV = 2800000,
		.max_uV = 2800000,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = ARRAY_SIZE(fixed_sdio_en_consumers),
	.consumer_supplies = fixed_sdio_en_consumers
};

static struct fixed_voltage_config fixed_sdio_config = {
	.supply_name = "sdio_en",
	.microvolts = 2800000,
	.gpio = TEGRA_GPIO_PF3,
	.enable_high = 1,
	.enabled_at_boot = 0,		/* Needs to be enabled on older oly & etna below */
	.init_data = &fixed_sdio_regulator,
};

static struct platform_device fixed_regulator_devices[] = {
	{
		.name = "reg-fixed-voltage",
		.id = 1,
		.dev = {
			.platform_data = &fixed_sdio_config,
		},
	},
};

struct tegra_spi_device_controller_data olympus_spi_tegra_data = {
	.is_hw_based_cs = 0,
};

struct spi_board_info tegra_spi_devices[] __initdata = {
    {
        .modalias = "cpcap",
        .bus_num = 1,
        .chip_select = 0,
        .mode = SPI_MODE_0 | SPI_CS_HIGH,
        .max_speed_hz = 8000000,
        .platform_data = &tegra_cpcap_data,
        .controller_data = &olympus_spi_tegra_data,
        .irq = INT_EXTERNAL_PMU,
    },

};
extern void get_gpio_settings(void);
extern void	pinmux_show(void);

static void olympus_board_suspend(int lp_state, enum suspend_stage stg)
{
	int rc,ret;
	if ((lp_state == TEGRA_SUSPEND_LP1) && (stg == TEGRA_SUSPEND_BEFORE_CPU))
		tegra_console_uart_suspend();
	if ((lp_state == TEGRA_SUSPEND_LP0) && (stg == TEGRA_SUSPEND_BEFORE_CPU))
			{
				printk(KERN_INFO "%s: entering...\n", __func__);

				tegra_pinmux_set_tristate(5 /*TEGRA_PINGROUP_CDEV*/, TEGRA_TRI_TRISTATE);
				tegra_pinmux_set_tristate(8 /*TEGRA_PINGROUP_CSUS*/, TEGRA_TRI_TRISTATE);
				tegra_pinmux_set_tristate(9 /*TEGRA_PINGROUP_DAP1*/, TEGRA_TRI_TRISTATE);
				tegra_pinmux_set_tristate(10/*TEGRA_PINGROUP_DAP2*/, TEGRA_TRI_TRISTATE);
				tegra_pinmux_set_tristate(11/*TEGRA_PINGROUP_DAP3*/, TEGRA_TRI_TRISTATE);
				tegra_pinmux_set_tristate(12/*TEGRA_PINGROUP_DAP4*/, TEGRA_TRI_TRISTATE);
				tegra_pinmux_set_tristate(22 /*TEGRA_PINGROUP_GMC*/, TEGRA_TRI_TRISTATE);
				tegra_pinmux_set_tristate(30/*TEGRA_PINGROUP_IRRX*/, TEGRA_TRI_TRISTATE);
				tegra_pinmux_set_tristate(31/*TEGRA_PINGROUP_IRTX*/, TEGRA_TRI_TRISTATE);
				tegra_pinmux_set_tristate(49 /*TEGRA_PINGROUP_LD2*/, TEGRA_TRI_TRISTATE);
				tegra_pinmux_set_tristate(51 /*TEGRA_PINGROUP_LD4*/, TEGRA_TRI_TRISTATE);
				tegra_pinmux_set_tristate(58 /*TEGRA_PINGROUP_LDI*/, TEGRA_TRI_TRISTATE);
				tegra_pinmux_set_tristate(61/*TEGRA_PINGROUP_LHP2*/, TEGRA_TRI_TRISTATE);
				tegra_pinmux_set_tristate(74/*TEGRA_PINGROUP_LSPI*/, TEGRA_TRI_TRISTATE);
				tegra_pinmux_set_tristate(76/*TEGRA_PINGROUP_LVP1*/, TEGRA_TRI_TRISTATE);
				tegra_pinmux_set_tristate(77 /*TEGRA_PINGROUP_LVS*/, TEGRA_TRI_TRISTATE);
				tegra_pinmux_set_tristate(104/*TEGRA_PINGROUP_UCA*/, TEGRA_TRI_TRISTATE);
				tegra_pinmux_set_tristate(105/*TEGRA_PINGROUP_UCB*/, TEGRA_TRI_TRISTATE);
				tegra_pinmux_set_pullupdown(108/*TEGRA_PINGROUP_DDRC*/, TEGRA_PUPD_PULL_UP);

				printk(KERN_INFO "%s: TEGRA_GPIO_PM2 = 0",__func__);
				gpio_set_value(TEGRA_GPIO_PM2, 0);

				printk(KERN_INFO "%s: TEGRA_AKM8975_IRQ_GPIO",__func__);
				tegra_gpio_disable(TEGRA_AKM8975_IRQ_GPIO);

				printk(KERN_INFO "%s: TEGRA_GPIO_PT3 = 0",__func__);
				gpio_set_value(TEGRA_GPIO_PT3, 0);

				printk(KERN_INFO "%s: TEGRA_GPIO_PM5 disabling",__func__);
				tegra_gpio_disable(TEGRA_GPIO_PM5);

//				printk(KERN_INFO "%s: TEGRA_GPIO_PU1 = 0",__func__);
//				gpio_set_value(TEGRA_GPIO_PU1, 0);

//				printk(KERN_INFO "%s: disable_irq_wake(gpio_to_irq(TEGRA_GPIO_PU6))",__func__);
//				ret = irq_set_irq_wake(gpio_to_irq(TEGRA_GPIO_PU6), 0);
//				if (ret) pr_info("%s: irq_set_irq_wake problem, ret=%d\n",__func__, ret);

//				printk(KERN_INFO "%s: TEGRA_GPIO_PU6 = 0",__func__);
//				gpio_set_value(TEGRA_GPIO_PU6, 0);

//				printk(KERN_INFO "%s: disable_irq_wake(gpio_to_irq(TEGRA_GPIO_PV2))",__func__);

//				ret = irq_set_irq_wake(gpio_to_irq(TEGRA_GPIO_PV2), 0);
//				if (ret) pr_info("%s: irq_set_irq_wake problem, ret=%d\n",__func__, ret);

//				printk(KERN_INFO "%s: TEGRA_GPIO_PU0 = 0",__func__);
//				gpio_set_value(TEGRA_GPIO_PU0, 0);

//				printk(KERN_INFO "%s: TEGRA_GPIO_PU4 = 0",__func__);
//				gpio_set_value(TEGRA_GPIO_PU4, 0);

				//pinmux_show();
				get_gpio_settings();
				printk(KERN_INFO "%s: exiting...\n", __func__);
			};

};

static void olympus_board_resume(int lp_state, enum resume_stage stg)
{
	int rc;
	if ((lp_state == TEGRA_SUSPEND_LP1) && (stg == TEGRA_RESUME_AFTER_CPU))
		tegra_console_uart_resume();
	if ((lp_state == TEGRA_SUSPEND_LP0) && (stg == TEGRA_RESUME_AFTER_CPU)) {
		printk(KERN_INFO "%s: entering...\n", __func__);

		tegra_pinmux_set_tristate(5 /*TEGRA_PINGROUP_CDEV*/, TEGRA_TRI_NORMAL);
		tegra_pinmux_set_tristate(8 /*TEGRA_PINGROUP_CSUS*/, TEGRA_TRI_NORMAL);
		tegra_pinmux_set_tristate(9 /*TEGRA_PINGROUP_DAP1*/, TEGRA_TRI_NORMAL);
		tegra_pinmux_set_tristate(10/*TEGRA_PINGROUP_DAP2*/, TEGRA_TRI_NORMAL);
		tegra_pinmux_set_tristate(11/*TEGRA_PINGROUP_DAP3*/, TEGRA_TRI_NORMAL);
		tegra_pinmux_set_tristate(12/*TEGRA_PINGROUP_DAP4*/, TEGRA_TRI_NORMAL);
		tegra_pinmux_set_tristate(22 /*TEGRA_PINGROUP_GMC*/, TEGRA_TRI_NORMAL);
		tegra_pinmux_set_tristate(30/*TEGRA_PINGROUP_IRRX*/, TEGRA_TRI_NORMAL);
		tegra_pinmux_set_tristate(31/*TEGRA_PINGROUP_IRTX*/, TEGRA_TRI_NORMAL);
		tegra_pinmux_set_tristate(49 /*TEGRA_PINGROUP_LD2*/, TEGRA_TRI_NORMAL);
		tegra_pinmux_set_tristate(51 /*TEGRA_PINGROUP_LD4*/, TEGRA_TRI_NORMAL);
		tegra_pinmux_set_tristate(58 /*TEGRA_PINGROUP_LDI*/, TEGRA_TRI_NORMAL);
		tegra_pinmux_set_tristate(61/*TEGRA_PINGROUP_LHP2*/, TEGRA_TRI_NORMAL);
		tegra_pinmux_set_tristate(74/*TEGRA_PINGROUP_LSPI*/, TEGRA_TRI_NORMAL);
		tegra_pinmux_set_tristate(76/*TEGRA_PINGROUP_LVP1*/, TEGRA_TRI_NORMAL);
		tegra_pinmux_set_tristate(77 /*TEGRA_PINGROUP_LVS*/, TEGRA_TRI_NORMAL);
		tegra_pinmux_set_tristate(104/*TEGRA_PINGROUP_UCA*/, TEGRA_TRI_NORMAL);
		tegra_pinmux_set_tristate(105/*TEGRA_PINGROUP_UCB*/, TEGRA_TRI_NORMAL);
		tegra_pinmux_set_pullupdown(108/*TEGRA_PINGROUP_DDRC*/, TEGRA_PUPD_NORMAL);

		printk(KERN_INFO "%s: TEGRA_GPIO_PM2 = 1",__func__);
		gpio_set_value(TEGRA_GPIO_PM2, 1);

		printk(KERN_INFO "%s: TEGRA_AKM8975_IRQ_GPIO",__func__);
		tegra_gpio_enable(TEGRA_AKM8975_IRQ_GPIO);

		printk(KERN_INFO "%s: TEGRA_GPIO_PT3 = 1",__func__);
		gpio_set_value(TEGRA_GPIO_PT3, 1);

		printk(KERN_INFO "%s: TEGRA_GPIO_PM5 disabling",__func__);
		tegra_gpio_enable(TEGRA_GPIO_PM5);

/*		printk(KERN_INFO "%s: TEGRA_GPIO_PU1 = 1",__func__);
		gpio_set_value(TEGRA_GPIO_PU1, 1);

		printk(KERN_INFO "%s: TEGRA_GPIO_PU6 = 1",__func__);
		gpio_set_value(TEGRA_GPIO_PU6, 1);
		printk(KERN_INFO "%s: enable_irq_wake(gpio_to_irq(TEGRA_GPIO_PU6))",__func__);
		enable_irq_wake(gpio_to_irq(TEGRA_GPIO_PU6));

		printk(KERN_INFO "%s: enable_irq_wake(gpio_to_irq(TEGRA_GPIO_PV2))",__func__);
		enable_irq_wake(gpio_to_irq(TEGRA_GPIO_PV2));

		printk(KERN_INFO "%s: enable_irq_wake(gpio_to_irq(TEGRA_GPIO_PI5))",__func__);
		enable_irq_wake(gpio_to_irq(TEGRA_GPIO_PI5));

		for (rc=0;rc<255;rc++) printk("gpio = %d => irq = %d\n", rc, gpio_to_irq(rc));*/

		printk(KERN_INFO "%s: exiting...\n", __func__);
	}
};

static struct tegra_suspend_platform_data olympus_suspend_data = {
	.cpu_timer 		= 800,
	.cpu_off_timer	= 600,
	.suspend_mode	= TEGRA_SUSPEND_LP0,
	.core_timer		= 1842,
	.core_off_timer = 31,
	.corereq_high	= true,
	.sysclkreq_high	= true,
	.combined_req 	= false,
	.board_suspend	= olympus_board_suspend,
	.board_resume 	= olympus_board_resume,

/*	.cpu_timer	= 2000,
	.cpu_off_timer	= 0,
	.suspend_mode	= TEGRA_SUSPEND_LP0,
	.core_timer	= 0x7e7e,
	.core_off_timer = 0,
	.corereq_high	= true,
	.sysclkreq_high	= true,*/
};

void __init olympus_suspend_init(void)
{

/*	enable_irq_wake(wakepad_irq[2]);
	enable_irq_wake(wakepad_irq[5]);
	enable_irq_wake(wakepad_irq[6]);
	enable_irq_wake(wakepad_irq[7]);
	enable_irq_wake(wakepad_irq[17]);
	enable_irq_wake(wakepad_irq[18]);
	enable_irq_wake(wakepad_irq[24]);

	tegra_suspend_platform.wake_low 	= 4;		//0x0000004
	tegra_suspend_platform.wake_high 	= 393248;	//0x0060020
	tegra_suspend_platform.wake_any 	= 16777408;	//0x10000C0
	tegra_suspend_platform.wake_enb 	= 17170660;	//0x10600E4

	*/

	tegra_init_suspend(&olympus_suspend_data);
}

static void get_cpcap_audio_data(void)
{
        static struct cpcap_audio_pdata data;
        cpcap_audio_device.dev.platform_data = (void *)&data;

        printk("CPCAP audio init\n");
        data.voice_type = VOICE_TYPE_QC;
        data.stereo_loudspeaker = 0;
        data.mic3 = 1;
}

void __init olympus_power_init(void)
{
	unsigned int i;
	int error;

	void __iomem *chip_id = IO_ADDRESS(TEGRA_APB_MISC_BASE) + 0x804;
	u32 minor;

	tegra_chip_uid();

	minor = (readl(chip_id) >> 16) & 0xf;
	/* A03 (but not A03p) chips do not support LP0 */
	if (minor == 3 && !(tegra_spare_fuse(18) || tegra_spare_fuse(19))) {
		printk(KERN_INFO "%s: this SoC does not support LP0, switching to LP1\n", __func__);
		olympus_suspend_data.suspend_mode = TEGRA_SUSPEND_LP1;
	}

	/* CPCAP standby lines connected to CPCAP GPIOs on Etna P1B & Olympus P2 */
	if ( HWREV_TYPE_IS_FINAL(system_rev) ||
	     (machine_is_etna() &&
	      HWREV_TYPE_IS_PORTABLE(system_rev) &&
	       (HWREV_REV(system_rev)  >= HWREV_REV_1B))  ||
	     (machine_is_olympus() &&
	       HWREV_TYPE_IS_PORTABLE(system_rev) &&
	       (HWREV_REV(system_rev)  >= HWREV_REV_2))) {
		tegra_cpcap_data.hwcfg[1] |= CPCAP_HWCFG1_STBY_GPIO;
	}

	/* For Olympus P3 the following is done:
	 * 1. VWLAN2 is  shutdown in standby by the CPCAP uC.
	 * 2. VWLAN1 is shutdown all of the time.
	 */
	if (HWREV_TYPE_IS_FINAL(system_rev) ||
		(HWREV_TYPE_IS_PORTABLE(system_rev) &&
		 (HWREV_REV(system_rev) >= HWREV_REV_3))) {
		pr_info("Detected P3 Olympus hardware.\n");
		tegra_cpcap_data.hwcfg[1] |= CPCAP_HWCFG1_SEC_STBY_VWLAN2;
		tegra_cpcap_data.hwcfg[1] &= ~CPCAP_HWCFG1_SEC_STBY_VWLAN1;
		cpcap_regulator[CPCAP_VWLAN2].constraints.always_on = 0;
	} else {
		/* Currently only Olympus P3 or greater can handle turning off the
		   external SD card. */
		fixed_sdio_config.enabled_at_boot = 1;
		tegra_gpio_enable(43);		
		gpio_request(43, "sdio_en");
		gpio_direction_output(43,1);
	}

	/* For all machine types, disable watchdog when HWREV is debug, brassboard or mortable */
	if (HWREV_TYPE_IS_DEBUG(system_rev) || HWREV_TYPE_IS_BRASSBOARD(system_rev) ||
	    HWREV_TYPE_IS_MORTABLE(system_rev) ){
		tegra_cpcap_data.wdt_disable = 1;
	}

	spi_register_board_info(tegra_spi_devices, ARRAY_SIZE(tegra_spi_devices));

	for (i = 0; i < sizeof(fixed_regulator_devices)/sizeof(fixed_regulator_devices[0]); i++) {
		error = platform_device_register(&fixed_regulator_devices[i]);
		pr_info("Registered reg-fixed-voltage: %d result: %d\n", i, error);
	}

	get_cpcap_audio_data();

	for (i = 0; i < ARRAY_SIZE(cpcap_devices); i++)
		cpcap_device_register(cpcap_devices[i]);

	(void) cpcap_driver_register(&cpcap_validity_driver);

	olympus_suspend_init();

	pm_power_off = olympus_system_power_off;
	arm_pm_restart = olympus_pm_restart;
}
