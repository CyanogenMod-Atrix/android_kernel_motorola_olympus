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

#include <asm/bootinfo.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/setup.h>

#include <mach/iomap.h>
#include <mach/irqs.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/gpio.h>
#include <linux/cpcap-accy.h>
#include <linux/reboot.h>

#include "clock.h"
#include "gpio-names.h"
#include "pm.h"

#include <linux/qtouch_obp_ts.h>
#if 0
#include <linux/isl29030.h>

#include <linux/leds-lm3530.h>
#include <linux/leds-lm3532.h>
#endif
#include "board.h"
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
	/* TEGRA_GPIO_PT2, ICS uses it */
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
        /*TEGRA_GPIO_PT2, ICS uses it */
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
	/* TEGRA_GPIO_PT2, ICS uses it */
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
#if 0
	{ "usb3",	"clk_m",	26000000,	false},
{ "dvc",	"clk_m",	2888888,	true},
{ "i2c3",	"clk_m",	2888888,	true},
{ "i2c2",	"clk_m",	787878,		true},
{ "i2c1",	"clk_m",	2888888,	true},
{ "owr",	"clk_m",	1000000,	false},
{ "kfuse",	"clk_m",	26000000,	true},
{ "i2s2",	"clk_m",	26000000,	false},
{ "timer",	"clk_m",	26000000,	true},
{ "pll_u",	"clk_m",	480000000,	true},
{ "pll_d",	"clk_m",	459000000,	true},
{ "pll_d_out0",	"pll_d",	229500000,	true},
{ "disp1",	"pll_d_out0",	229500000,	true},
{ "pll_p",	"clk_m",	216000000,	true},
{ "host1x",	"pll_p",	108000000,	true},
{ "uartb",	"pll_p",	216000000,	true},
{ "uartd",	"pll_p",	216000000,	true},
{ "csite",	"pll_p",	144000000,	true},
{ "sdmmc3",	"pll_p",	48000000,	false},
{ "sdmmc2",	"pll_p",	24000000,	false},
{ "sdmmc1",	"pll_p",	48000000,	false},
{ "pll_p_out3",	"pll_p",	72000000,	true},
{ "pll_p_out1",	"pll_p",	28800000,	true},
{ "pll_a",	"pll_p_out1",	56448000,	false},
{ "pll_a_out0",	"pll_a",	11289600,	false},
{ "pll_c",	"clk_m",	600000000,	true},
{ "mpe",	"pll_c",	300000000,	true},
{ "epp",	"pll_c",	300000000,	true},
{ "vi",		"pll_c",	100000000,	true},
{ "2d",		"pll_c",	300000000,	true},
{ "3d",		"pll_c",	300000000,	true},
{ "cop",	"sclk",		80000000,	true},
{ "pll_m",	"clk_m",	600000000,	true},
{ "emc",	"pll_m",	600000000,	true},
{ "uartc",	"pll_m",	600000000,	true},
{ "rtc",	"clk_32k",	32768,	true},
	{ "pll_p_out4",	"pll_p",	108000000,	true},	
	{ "sdmmc4",	"pll_p",	48000000,	true},
	{ "pll_c_out1",	"pll_c",	80000000,	true},
	{ "sclk",	"pll_c_out1",	80000000,	true},
	{ "hclk",	"sclk",		80000000,	true},
	{ "pclk",	"hclk",		40000000,	true},
	{ "apbdma",	"pclk",		40000000,	true},
	{ "blink",	"clk_32k",	32768,		false},
	{ "pwm",	"clk_32k",	32768,		false},
	{ "kbc",	"clk_32k",	32768,		true},
	{ "sdmmc2",	"pll_p",	25000000,	false},
	{ "i2s2",	"pll_a_out0",	0,		false},
	{ "spi",	"clk_m",	26000000,	true},
#endif
//	{"usbd",	"clk_m",	26000000,	true},
	{"sbc1",	"pll_c",	60000000,	true},
	{"sbc2",	"pll_c",	60000000,	true},
//	{"pwm",		"clk_32k",	32768,		true},
	{"pwm",		"clk_32k",	32768,		false},
	{"kbc",		"clk_32k",	32768,		true},
	{"sdmmc2",	"pll_p",	25000000,	false},
//	{"i2s1",	"pll_a_out0",	0,		true},
	{"i2s1",	"pll_a_out0",	0,		false},
//	{"pll_a_out0",	"pll_a",	11289600,	true}, //added
	{"spdif_out",	"pll_a_out0",	0,		false},
	{ NULL,		NULL,		0,		0},
};

static noinline void __init tegra_setup_bluesleep(void)
{
       struct platform_device *pDev = NULL;
       struct resource *res;

       pDev = platform_device_alloc("bluesleep", 0);
       if (!pDev) {
               pr_err("unable to allocate platform device for bluesleep");
               goto fail;
       }

       res = kzalloc(sizeof(struct resource)*3, GFP_KERNEL);
       if (!res) {
               pr_err("unable to allocate resource for bluesleep\n");
               goto fail;
       }

       res[0].name   = "gpio_host_wake";
       res[0].start  = TEGRA_GPIO_PU6;
       res[0].end    = TEGRA_GPIO_PU6;
       res[0].flags  = IORESOURCE_IO;

       res[1].name   = "gpio_ext_wake";
       res[1].start  = TEGRA_GPIO_PU1;
       res[1].end    = TEGRA_GPIO_PU1;
       res[1].flags  = IORESOURCE_IO;

       res[2].name   = "host_wake";
       res[2].start  = gpio_to_irq(TEGRA_GPIO_PU6);
       res[2].end    = gpio_to_irq(TEGRA_GPIO_PU6);
       res[2].flags  = IORESOURCE_IRQ;

       if (platform_device_add_resources(pDev, res, 3)) {
               pr_err("unable to add resources to bluesleep device\n");
               goto fail;
       }

       if (platform_device_add(pDev)) {
               pr_err("unable to add bluesleep device\n");
               goto fail;
       }

fail:
       if (pDev)
               return;
}

static struct platform_device tegra_w1_device = {
    .name          = "tegra_w1",
    .id            = -1,
};

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

#define GPIO_BANK(x)		((x) >> 5)
#define GPIO_PORT(x)		(((x) >> 3) & 0x3)
#define GPIO_BIT(x)		((x) & 0x7)

#define GPIO_REG(x)		(IO_TO_VIRT(TEGRA_GPIO_BASE) +	\
				 GPIO_BANK(x) * 0x80 +		\
				 GPIO_PORT(x) * 4)

#define GPIO_CNF(x)		(GPIO_REG(x) + 0x00)
#define GPIO_OE(x)		(GPIO_REG(x) + 0x10)
#define GPIO_OUT(x)		(GPIO_REG(x) + 0X20)
#define GPIO_IN(x)		(GPIO_REG(x) + 0x30)
#define GPIO_INT_STA(x)		(GPIO_REG(x) + 0x40)
#define GPIO_INT_ENB(x)		(GPIO_REG(x) + 0x50)
#define GPIO_INT_LVL(x)		(GPIO_REG(x) + 0x60)
#define GPIO_INT_CLR(x)		(GPIO_REG(x) + 0x70)

#define GPIO_MSK_CNF(x)		(GPIO_REG(x) + 0x800)
#define GPIO_MSK_OE(x)		(GPIO_REG(x) + 0x810)
#define GPIO_MSK_OUT(x)		(GPIO_REG(x) + 0X820)
#define GPIO_MSK_INT_STA(x)	(GPIO_REG(x) + 0x840)
#define GPIO_MSK_INT_ENB(x)	(GPIO_REG(x) + 0x850)
#define GPIO_MSK_INT_LVL(x)	(GPIO_REG(x) + 0x860)

#define GPIO_INT_LVL_MASK		0x010101
#define GPIO_INT_LVL_EDGE_RISING	0x000101
#define GPIO_INT_LVL_EDGE_FALLING	0x000100
#define GPIO_INT_LVL_EDGE_BOTH		0x010100
#define GPIO_INT_LVL_LEVEL_HIGH		0x000001
#define GPIO_INT_LVL_LEVEL_LOW		0x000000
#if 0
struct tegra_gpio_bank {
	int bank;
	int irq;
	spinlock_t lvl_lock[4];
#ifdef CONFIG_PM
	u32 cnf[4];
	u32 out[4];
	u32 oe[4];
	u32 int_enb[4];
	u32 int_lvl[4];
#endif
};

static struct tegra_gpio_bank tegra_gpio_banks[] = {
	{.bank = 0, .irq = INT_GPIO1, .cnf = {0x01,0x08,0x82,0xfe}, .oe = {0x01,0x08,0x82,0xfc}, .out = {0x00,0x08,0x82,0xc8}},
	{.bank = 1, .irq = INT_GPIO2, .cnf = {0xe9,0xcb,0xff,0x0f}, .oe = {0xe9,0xcb,0xff,0x0f}, .out = {0xe9,0x81,0xff,0x0f}},
	{.bank = 2, .irq = INT_GPIO3, .cnf = {0xb1,0x0d,0x5c,0xe9}, .oe = {0x91,0x0d,0x5c,0xe9}, .out = {0x91,0x0d,0x5c,0xe9}},
	{.bank = 3, .irq = INT_GPIO4, .cnf = {0xc7,0x60,0xff,0x00}, .oe = {0xc4,0x60,0xff,0x00}, .out = {0xc0,0x60,0xff,0x00}},
	{.bank = 4, .irq = INT_GPIO5, .cnf = {0x38,0xf8,0x07,0x7d}, .oe = {0x38,0xf8,0x07,0x7c}, .out = {0x38,0xf8,0x07,0x74}},
	{.bank = 5, .irq = INT_GPIO6, .cnf = {0x00,0xc5,0x22,0x00}, .oe = {0x00,0xc1,0x22,0x00}, .out = {0x00,0x80,0x22,0x00}},
	{.bank = 6, .irq = INT_GPIO7, .cnf = {0x0f,0x20,0x00,0x03}, .oe = {0x0f,0x20,0x00,0x0f}, .out = {0x0f,0x20,0x00,0x0f}},
};
#endif
static int tegra_gpio_compose(int bank, int port, int bit)
{
	return (bank << 5) | ((port & 0x3) << 3) | (bit & 0x7);
}

static void read_gpio(void)
{
	int i,j;
	for (i = 0; i < 7; i++) {
		for (j = 0; j < 4; j++) {
			int gpio = tegra_gpio_compose(i, j, 0);
			printk(KERN_INFO "pICS_%s: %d:%d %02x %02x %02x %02x %02x %02x %06x\n",__func__,
			       i, j,
			       __raw_readl(GPIO_CNF(gpio)),
			       __raw_readl(GPIO_OE(gpio)),
			       __raw_readl(GPIO_OUT(gpio)),
			       __raw_readl(GPIO_IN(gpio)),
			       __raw_readl(GPIO_INT_STA(gpio)),
			       __raw_readl(GPIO_INT_ENB(gpio)),
			       __raw_readl(GPIO_INT_LVL(gpio)));
		}
	}
}
#if 0
static void write_gpio(void)
{
	int i,j;
	for (i = 0; i < 7; i++) {
		for (j = 0; j < 4; j++) {
			int gpio = tegra_gpio_compose(i, j, 0);
			printk(KERN_INFO "pICS_%s: %d:%d %02x %02x %02x %02x %02x %02x %06x\n",__func__,
			       i, j,
			       __raw_readl(GPIO_CNF(gpio)),
			       __raw_readl(GPIO_OE(gpio)),
			       __raw_readl(GPIO_OUT(gpio)),
			       __raw_readl(GPIO_IN(gpio)),
			       __raw_readl(GPIO_INT_STA(gpio)),
			       __raw_readl(GPIO_INT_ENB(gpio)),
			       __raw_readl(GPIO_INT_LVL(gpio)));
		}
	}
}
#endif

static void __init tegra_mot_init(void)
{
/*	struct clk *clk;*/
	
	tegra_clk_init_from_table(olympus_clk_init_table);

	tegra_ram_console_debug_init();

	olympus_pinmux_init();

	olympus_devices_init();

	olympus_audio_init();

	olympus_power_init();
	
	mot_tcmd_init();

	olympus_panel_init();

	olympus_i2c_init();

	olympus_keypad_init();

	if( (bi_powerup_reason() & PWRUP_FACTORY_CABLE) &&
	    (bi_powerup_reason() != PWRUP_INVALID) ){
#ifdef NEED_FACT_BUSY_HINT
		FactoryBusyHint(); //factory workaround no longer needed
#endif
	}

if (1==0) olympus_emc_init();

	platform_device_register(&tegra_w1_device);
	
//	mot_modem_init();
	olympus_wlan_init();

	mot_sensors_init();

	pm_power_off = mot_system_power_off;
	tegra_setup_bluesleep();

	/* Configure SPDIF_OUT as GPIO by default, it can be later controlled
	   as needed. When SPDIF_OUT is enabled and if HDMI is connected, it
	   can interefere with CPCAP ID pin, as SPDIF_OUT and ID are coupled.
	*/

	tegra_gpio_enable(TEGRA_GPIO_PD4);
	gpio_request(TEGRA_GPIO_PD4, "spdif_enable");
	gpio_direction_output(TEGRA_GPIO_PD4, 0);
	gpio_export(TEGRA_GPIO_PD4, false);
	if (1==0)
	if ((HWREV_TYPE_IS_PORTABLE(system_rev) || HWREV_TYPE_IS_FINAL(system_rev)))
		{
			if (HWREV_REV(system_rev) >= HWREV_REV_1 && HWREV_REV(system_rev) < HWREV_REV_2)
			{
				// Olympus P1
				config_unused_pins(oly_unused_pins_p1, ARRAY_SIZE(oly_unused_pins_p1));
			}
			else if (HWREV_REV(system_rev) >= HWREV_REV_2 && HWREV_REV(system_rev) < HWREV_REV_3)
			{
				// Olympus P2
				config_unused_pins(oly_unused_pins_p2, ARRAY_SIZE(oly_unused_pins_p2));
			}
			else if (HWREV_REV(system_rev) >= HWREV_REV_3 || HWREV_TYPE_IS_FINAL(system_rev))
			{
			// Olympus P3 and newer
				config_unused_pins(oly_unused_pins_p3, ARRAY_SIZE(oly_unused_pins_p3));
			}
		}
	
	tegra_release_bootloader_fb();
	if (1==0) read_gpio();	
}

static void __init mot_fixup(struct machine_desc *desc, struct tag *tags,
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

	tegra_reserve(SZ_128M+SZ_64M, SZ_8M, SZ_16M);
	tegra_ram_console_debug_reserve(SZ_1M);

}

MACHINE_START(OLYMPUS, "Olympus")

    .boot_params  = 0x00000100,
    .fixup        = mot_fixup,
    .map_io       = tegra_map_common_io,
    .reserve	  = tegra_olympus_reserve,
    .init_early	  = tegra_init_early,
    .init_irq     = tegra_init_irq,
    .timer        = &tegra_timer,
    .init_machine = tegra_mot_init,

MACHINE_END

