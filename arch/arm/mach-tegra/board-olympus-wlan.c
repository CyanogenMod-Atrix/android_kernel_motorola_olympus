/*
 * arch/arm/mach-tegra/board-mot-wlan.c
 *
 * Board file with wlan specific functions and data structrures
 *
 * Copyright (c) 2010, Motorola Corporation.
 *
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/pm.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <asm/mach-types.h>
#include <linux/io.h>
#include <asm/mach/arch.h>
#include <asm/setup.h>
#include <linux/skbuff.h>
#include <linux/wlan_plat.h>
#include <mach/sdhci.h>

#include "board.h"
#include "board-olympus.h"
#include "gpio-names.h"

#define WLAN_RESET_GPIO 	TEGRA_GPIO_PU2
#define WLAN_REG_ON_GPIO 	TEGRA_GPIO_PU3
#define WLAN_IRQ_GPIO 		TEGRA_GPIO_PU5

#define PREALLOC_WLAN_NUMBER_OF_SECTIONS        4
#define PREALLOC_WLAN_NUMBER_OF_BUFFERS         160
#define PREALLOC_WLAN_SECTION_HEADER            24

#define WLAN_SECTION_SIZE_0     (PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_1     (PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_2     (PREALLOC_WLAN_NUMBER_OF_BUFFERS * 512)
#define WLAN_SECTION_SIZE_3     (PREALLOC_WLAN_NUMBER_OF_BUFFERS * 1024)

#define WLAN_SKB_BUF_NUM        16

static bool wlan_ctrl_ready = false;

static struct sk_buff *wlan_static_skb[WLAN_SKB_BUF_NUM];
char mot_wlan_mac[6] = {0x00, 0x90, 0xC3, 0x00, 0x00, 0x00};

struct wifi_mem_prealloc_struct {
	void *mem_ptr;
	unsigned long size;
};

static struct wifi_mem_prealloc_struct
	wifi_mem_array[PREALLOC_WLAN_NUMBER_OF_SECTIONS] = {
	{ NULL, (WLAN_SECTION_SIZE_0 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_1 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_2 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_3 + PREALLOC_WLAN_SECTION_HEADER) }
};

#define WLC_CNTRY_BUF_SZ        4
 struct cntry_locales_custom {
	char iso_abbrev[WLC_CNTRY_BUF_SZ];
	char ccode[WLC_CNTRY_BUF_SZ];
	int  regrev;
 };

/* Table should be filled out based on custom regulatory requirement */
/* products can declare tables of variable length as needed */
/* This is shared by olympus, daoytona and etna */
 static struct cntry_locales_custom olympus_locales_table[] = {
	{"",   "XY", 4},/* universal if Counry code is unknown or empty */
	{"US", "US", 69},/* input ISO "US" to : US regrev 69 */
	{"CA", "US", 69},/* input ISO "CA" to : US regrev 69 */
	{"EU", "EU", 5},/* European union countries */
	{"AT", "EU", 5},
	{"BE", "EU", 5},
	{"BG", "EU", 5},
	{"CY", "EU", 5},
	{"CZ", "EU", 5},
	{"DK", "EU", 5},
	{"EE", "EU", 5},
	{"FI", "EU", 5},
	{"FR", "EU", 5},
	{"DE", "EU", 5},
	{"GR", "EU", 5},
	{"HU", "EU", 5},
	{"IE", "EU", 5},
	{"IT", "EU", 5},
	{"LV", "EU", 5},
	{"LI", "EU", 5},
	{"LT", "EU", 5},
	{"LU", "EU", 5},
	{"MT", "EU", 5},
	{"NL", "EU", 5},
	{"PL", "EU", 5},
	{"PT", "EU", 5},
	{"RO", "EU", 5},
	{"SK", "EU", 5},
	{"SI", "EU", 5},
	{"ES", "EU", 5},
	{"SE", "EU", 5},
	{"GB", "EU", 5},/* input ISO "GB" to : EU regrev 05 */
	{"KR", "XY", 3},
	{"AU", "XY", 3},
	{"CN", "XY", 3},/* input ISO "CN" to : XY regrev 03 */
	{"TW", "XY", 3},
	{"AR", "XY", 3},
	{"MX", "XY", 3}
 };

static struct cntry_locales_custom *mot_locales_table_ptr;
static int mot_locales_table_size;

static void *mot_wifi_mem_prealloc(int section, unsigned long size)
{
	if (section == PREALLOC_WLAN_NUMBER_OF_SECTIONS)
		return wlan_static_skb;
	if ((section < 0) || (section > PREALLOC_WLAN_NUMBER_OF_SECTIONS))
		return NULL;
	if (wifi_mem_array[section].size < size)
		return NULL;
	return wifi_mem_array[section].mem_ptr;
}

 int __init mot_init_wifi_mem(void)
 {
	int i;
	for (i = 0; (i < WLAN_SKB_BUF_NUM); i++) {
		if (i < (WLAN_SKB_BUF_NUM/2))
			wlan_static_skb[i] = dev_alloc_skb(4096);
		else
			wlan_static_skb[i] = dev_alloc_skb(8192);
		}
	for (i = 0; (i < PREALLOC_WLAN_NUMBER_OF_SECTIONS); i++) {
		wifi_mem_array[i].mem_ptr = kmalloc(wifi_mem_array[i].size,
			GFP_KERNEL);
		if (wifi_mem_array[i].mem_ptr == NULL)
			return -ENOMEM;
	}
	return 0;
 }

static struct resource mot_wifi_resources[] = {
	[0] = {
		.name	= "bcm4329_wlan_irq",
		.start	= TEGRA_GPIO_TO_IRQ(WLAN_IRQ_GPIO),
		.end	= TEGRA_GPIO_TO_IRQ(WLAN_IRQ_GPIO),
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_LOWLEVEL
			| IORESOURCE_IRQ_SHAREABLE,
	},
 };

 static int mot_wifi_cd; /* WIFI virtual 'card detect' status */
 static void (*wifi_status_cb)(int card_present, void *dev_id);
 static void *wifi_status_cb_devid;

 int mot_wifi_set_carddetect(int val)
 {
	pr_debug("%s: %d\n", __func__, val);
	mot_wifi_cd = val;
	sdhci_tegra_wlan_detect();
	if (wifi_status_cb)
		wifi_status_cb(val, wifi_status_cb_devid);
	else
		pr_warning("%s: Nobody to notify\n", __func__);
	return 0;
 }

 int mot_wifi_power(int on)
 {
	pr_debug("%s: %d\n", __func__, on);
	gpio_set_value(WLAN_REG_ON_GPIO, on);
	mdelay(100);
	gpio_set_value(WLAN_RESET_GPIO, on);
	mdelay(100);
	return 0;
 }

 static void *mot_wifi_get_country_code(char *my_iso)
 {
	int i;
	if (!my_iso)
		return NULL;

	for (i = 0; i < mot_locales_table_size; i++)
		if (strcmp(my_iso, mot_locales_table_ptr[i].iso_abbrev) == 0)
			return &mot_locales_table_ptr[i];
	/* return default configuration if no match found */
	return &mot_locales_table_ptr[0];
 }

 int mot_wifi_reset(int on)
 {
	pr_debug("%s:\n", __func__);
	gpio_set_value(WLAN_RESET_GPIO, on);
	mdelay(100);
	return 0;
 }

 int mot_wifi_get_mac_addr(unsigned char *buf)
 {
	if (!buf)
		return -EINVAL;
	pr_debug("%s\n", __func__);
	memcpy(buf, mot_wlan_mac, sizeof(mot_wlan_mac));
	return 0;
 }

 static struct wifi_platform_data mot_wifi_control = {
	.set_power     = mot_wifi_power,
	.set_reset      = mot_wifi_reset,
	.set_carddetect = mot_wifi_set_carddetect,
	.mem_prealloc   = mot_wifi_mem_prealloc,
	.get_mac_addr   = mot_wifi_get_mac_addr,
	.get_country_code = mot_wifi_get_country_code,
 };

 static struct platform_device mot_wifi_device = {
	.name           = "bcm4329_wlan",
	.id             = 1,
	.num_resources  = ARRAY_SIZE(mot_wifi_resources),
	.resource       = mot_wifi_resources,
	.dev            = {
	.platform_data = &mot_wifi_control,
	},
 };

 static int __init mot_wlan_gpio_init(void)
 {
	int ret = 0;
	pr_debug("%s Enter\n", __func__);

	tegra_gpio_enable(WLAN_RESET_GPIO);
	ret = gpio_request(WLAN_RESET_GPIO, "wlan_reset_pin");
	if (ret)
		pr_err("%s: %d gpio_reqest reset\n", __func__, ret);
	else
		ret = gpio_direction_output(WLAN_RESET_GPIO, 0);
	if (ret) {
		pr_err("%s: Err %d gpio_direction reset\n", __func__, ret);
		return -1;
	}

	tegra_gpio_enable(WLAN_REG_ON_GPIO);
	ret = gpio_request(WLAN_REG_ON_GPIO, "wlan_reg_on_pin");
	if (ret)
		pr_err("%s: Err %d gpio_reqest reg\n", __func__, ret);
	 else
		ret = gpio_direction_output(WLAN_REG_ON_GPIO, 0);
	if (ret) {
		pr_err("%s: Err %d gpio_direction reg\n", __func__, ret);
		return -1;
	}

	tegra_gpio_enable(WLAN_IRQ_GPIO);
	ret = gpio_request(WLAN_IRQ_GPIO, "wlan_irq_pin");
	if (ret)
		pr_err("%s: Error (%d) - gpio_reqest irq\n", __func__, ret);
	else
		ret = gpio_direction_input(WLAN_IRQ_GPIO);
	if (ret) {
		pr_err("%s: Err %d gpio_direction irq\n", __func__, ret);
		return -1;
	}
	return 0;
 }

 int __init mot_wlan_init(void)
 {
	int ret;
	pr_debug("%s: start\n", __func__);
	mot_wlan_gpio_init();
	mot_init_wifi_mem();
	mot_locales_table_ptr = olympus_locales_table;
	mot_locales_table_size = ARRAY_SIZE(olympus_locales_table);
	ret = platform_device_register(&mot_wifi_device);
	wlan_ctrl_ready = (ret == 0);
	return ret;
 }

#ifdef CONFIG_WIFI_CONTROL_EXPORT

 void bcm_wlan_power_on(int mode)
 {
	if (0 == wlan_ctrl_ready) {
		pr_err("%s WLAN control not ready\n", __func__);
		return;
	}
	gpio_set_value(WLAN_REG_ON_GPIO, 0x1);
	msleep_interruptible(100);
	gpio_set_value(WLAN_RESET_GPIO, 0x1);
	msleep_interruptible(100);
	if (1 == mode){
		sdhci_tegra_wlan_detect();
		msleep_interruptible(100);
	}
 }
 EXPORT_SYMBOL(bcm_wlan_power_on);

 void bcm_wlan_power_off(int mode)
 {
	if (0 == wlan_ctrl_ready) {
		pr_err("%s WLAN control not ready\n", __func__);
		return;
	}
	gpio_set_value(WLAN_RESET_GPIO, 0x0);
	msleep_interruptible(100);
	gpio_set_value(WLAN_REG_ON_GPIO, 0x0);
	msleep_interruptible(100);
	if (1 == mode){
		sdhci_tegra_wlan_detect();
		msleep_interruptible(100);
	}
 }
 EXPORT_SYMBOL(bcm_wlan_power_off);

 int bcm_wlan_get_irq(void)
 {
	return gpio_to_irq(WLAN_IRQ_GPIO);
 }
 EXPORT_SYMBOL(bcm_wlan_get_irq);

 char *bcm_wlan_mac = mot_wlan_mac;
 EXPORT_SYMBOL(bcm_wlan_mac);

#endif /*  CONFIG_WIFI_CONTROL_EXPORT */

 /*
  * Parse the WLAN MAC ATAG
  */
 static int __init parse_tag_wlan_mac(const struct tag *tag)
 {
	const struct tag_wlan_mac *wlan_mac_tag = &tag->u.wlan_mac;

	pr_info("%s: WLAN MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", __func__,
	        wlan_mac_tag->addr[0], wlan_mac_tag->addr[1],
		wlan_mac_tag->addr[2], wlan_mac_tag->addr[3],
		wlan_mac_tag->addr[4], wlan_mac_tag->addr[5]);

	memcpy(mot_wlan_mac, wlan_mac_tag->addr, sizeof(mot_wlan_mac));

	return 0;
 }
 __tagtable(ATAG_WLAN_MAC, parse_tag_wlan_mac);
