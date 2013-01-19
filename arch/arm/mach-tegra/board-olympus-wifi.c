/* linux/arch/arm/mach-msm/board-stingray-wifi.c
*/
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <asm/mach-types.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/setup.h>
#include <linux/if.h>
#include <linux/skbuff.h>
#include <linux/wlan_plat.h>
#include <mach/sdhci.h>

#include <linux/random.h>
#include <linux/jiffies.h>

#include "board-olympus.h"
#include "gpio-names.h"

#define OLYMPUS_WLAN_IRQ	TEGRA_GPIO_PU5
#define OLYMPUS_WLAN_RST	TEGRA_GPIO_PU2

#define ATAG_OLYMPUS_MAC	0x57464d41
#define ATAG_OLYMPUS_MAC_DEBUG

#define PREALLOC_WLAN_NUMBER_OF_SECTIONS	4
#define PREALLOC_WLAN_NUMBER_OF_BUFFERS		160
#define PREALLOC_WLAN_SECTION_HEADER		24

#define WLAN_SECTION_SIZE_0	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_1	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_2	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 512)
#define WLAN_SECTION_SIZE_3	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 1024)

#define WLAN_SKB_BUF_NUM	16

static struct sk_buff *wlan_static_skb[WLAN_SKB_BUF_NUM];

typedef struct wifi_mem_prealloc_struct {
	void *mem_ptr;
	unsigned long size;
} wifi_mem_prealloc_t;

static wifi_mem_prealloc_t wifi_mem_array[PREALLOC_WLAN_NUMBER_OF_SECTIONS] = {
	{ NULL, (WLAN_SECTION_SIZE_0 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_1 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_2 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_3 + PREALLOC_WLAN_SECTION_HEADER) }
};

static void *olympus_wifi_mem_prealloc(int section, unsigned long size)
{
	if (section == PREALLOC_WLAN_NUMBER_OF_SECTIONS)
		return wlan_static_skb;
	if ((section < 0) || (section > PREALLOC_WLAN_NUMBER_OF_SECTIONS))
		return NULL;
	if (wifi_mem_array[section].size < size)
		return NULL;
	return wifi_mem_array[section].mem_ptr;
}

int __init olympus_init_wifi_mem(void)
{
	int i;

	for(i=0;( i < WLAN_SKB_BUF_NUM );i++) {
		if (i < (WLAN_SKB_BUF_NUM/2))
			wlan_static_skb[i] = dev_alloc_skb(4096);
		else
			wlan_static_skb[i] = dev_alloc_skb(8192);
	}
	for(i=0;( i < PREALLOC_WLAN_NUMBER_OF_SECTIONS );i++) {
		wifi_mem_array[i].mem_ptr = kmalloc(wifi_mem_array[i].size,
							GFP_KERNEL);
		if (wifi_mem_array[i].mem_ptr == NULL)
			return -ENOMEM;
	}
	return 0;
}

static struct resource olympus_wifi_resources[] = {
	[0] = {
		.name		= "bcm4329_wlan_irq",
		.start		= TEGRA_GPIO_TO_IRQ(OLYMPUS_WLAN_IRQ),
		.end		= TEGRA_GPIO_TO_IRQ(OLYMPUS_WLAN_IRQ),
		.flags          = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL | IORESOURCE_IRQ_SHAREABLE,
	},
};

/* BCM4329 returns wrong sdio_vsn(1) when we read cccr,
 * we use predefined value (sdio_vsn=2) here to initial sdio driver well
  */
static struct embedded_sdio_data olympus_wifi_emb_data = {
	.cccr	= {
		.sdio_vsn       = 2,
		.multi_block    = 1,
		.low_speed      = 0,
		.wide_bus       = 0,
		.high_power     = 1,
		.high_speed     = 1,
	},
};

static int olympus_wifi_cd = 0; /* WIFI virtual 'card detect' status */
static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_devid;

static int olympus_wifi_status_register(
		void (*callback)(int card_present, void *dev_id),
		void *dev_id)
{
	if (wifi_status_cb)
		return -EAGAIN;
	wifi_status_cb = callback;
	wifi_status_cb_devid = dev_id;
	return 0;
}

static unsigned int olympus_wifi_status(struct device *dev)
{
	return olympus_wifi_cd;
}
/*STI-OLY this needs to be verified, I assume it should work but someone should check it*/
struct tegra_sdhci_platform_data olympus_wifi_data = {
	.cd_gpio = -1,
	.wp_gpio = -1,
	.power_gpio = -1,
	.is_8bit = 0,
	.mmc_data = {
		/*STI-OLY sets VDD voltage 1.65 - 1.95 */
		.ocr_mask		= MMC_VDD_165_195,
		.built_in		= 1,
		.status			= olympus_wifi_status,
		.card_present		= 0,
		.register_status_notify	= olympus_wifi_status_register,
		.embedded_sdio		= &olympus_wifi_emb_data,
	},
};

static int olympus_wifi_set_carddetect(int val)
{
	pr_debug("%s: %d\n", __func__, val);
	olympus_wifi_cd = val;
	if (wifi_status_cb) {
		wifi_status_cb(val, wifi_status_cb_devid);
	} else
		pr_warning("%s: Nobody to notify\n", __func__);
	return 0;
}

static int olympus_wifi_power_state;

static int olympus_wifi_power(int on)
{
	pr_debug("%s: %d\n", __func__, on);

	msleep(100);
	change_power_brcm_4329(on);
	msleep(300);
	gpio_set_value(OLYMPUS_WLAN_RST, on);
	msleep(200);

	olympus_wifi_power_state = on;
	return 0;
}

static int olympus_wifi_reset_state;

static int olympus_wifi_reset(int on)
{
	pr_debug("%s: do nothing\n", __func__);
	olympus_wifi_reset_state = on;
	return 0;
}

static unsigned char olympus_mac_addr[IFHWADDRLEN] = { 0,0x90,0xc3,0,0,0 };

static int __init parse_tag_wlan_mac(const struct tag *tag)
{
	unsigned char *dptr = (unsigned char *)(&tag->u);
	unsigned size;
#ifdef ATAG_OLYMPUS_MAC_DEBUG
	unsigned i;
#endif

	size = min((tag->hdr.size - 2) * sizeof(__u32), (unsigned)IFHWADDRLEN);
#ifdef ATAG_OLYMPUS_MAC_DEBUG
	printk("WiFi MAC Addr [%d] = 0x%x\n", tag->hdr.size, tag->hdr.tag);
	for(i=0;(i < size);i++) {
		printk(" %02x", dptr[i]);
	}
	printk("\n");
#endif
	memcpy(olympus_mac_addr, dptr, size);
	return 0;
}

__tagtable(ATAG_OLYMPUS_MAC, parse_tag_wlan_mac);

static int olympus_wifi_get_mac_addr(unsigned char *buf)
{
	uint rand_mac;

	if (!buf)
		return -EINVAL;

	if ((olympus_mac_addr[4] == 0) && (olympus_mac_addr[5] == 0)) {
		srandom32((uint)jiffies);
		rand_mac = random32();
		olympus_mac_addr[3] = (unsigned char)rand_mac;
		olympus_mac_addr[4] = (unsigned char)(rand_mac >> 8);
		olympus_mac_addr[5] = (unsigned char)(rand_mac >> 16);
	}
	memcpy(buf, olympus_mac_addr, IFHWADDRLEN);
	return 0;
}

/* Customized Locale table : OPTIONAL feature */
#define WLC_CNTRY_BUF_SZ	4
typedef struct cntry_locales_custom {
	char iso_abbrev[WLC_CNTRY_BUF_SZ];
	char custom_locale[WLC_CNTRY_BUF_SZ];
	int  custom_locale_rev;
} cntry_locales_custom_t;

static cntry_locales_custom_t olympus_wifi_translate_custom_table[] = {
/* Table should be filled out based on custom platform regulatory requirement */
	{"",   "XY", 4},  /* universal */
	{"US", "US", 69}, /* input ISO "US" to : US regrev 69 */
	{"CA", "US", 69}, /* input ISO "CA" to : US regrev 69 */
	{"EU", "EU", 5},  /* European union countries */
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
	{"GB", "EU", 5},  /* input ISO "GB" to : EU regrev 05 */
	{"IL", "IL", 0},
	{"CH", "CH", 0},
	{"TR", "TR", 0},
	{"NO", "NO", 0},
	{"KR", "XY", 3},
	{"AU", "XY", 3},
	{"CN", "XY", 3},  /* input ISO "CN" to : XY regrev 03 */
	{"TW", "XY", 3},
	{"AR", "XY", 3},
	{"MX", "XY", 3}
};

static void *olympus_wifi_get_country_code(char *ccode)
{
	int size = ARRAY_SIZE(olympus_wifi_translate_custom_table);
	int i;

	if (!ccode)
		return NULL;

	for (i = 0; i < size; i++)
		if (strcmp(ccode, olympus_wifi_translate_custom_table[i].iso_abbrev) == 0)
			return &olympus_wifi_translate_custom_table[i];
	return &olympus_wifi_translate_custom_table[0];
}

static struct wifi_platform_data olympus_wifi_control = {
	.set_power      = olympus_wifi_power,
	.set_reset      = olympus_wifi_reset,
	.set_carddetect = olympus_wifi_set_carddetect,
	.mem_prealloc	= olympus_wifi_mem_prealloc,
	.get_mac_addr	= olympus_wifi_get_mac_addr,
	.get_country_code = olympus_wifi_get_country_code,
};

static struct platform_device olympus_wifi_device = {
        .name           = "bcm4329_wlan",
        .id             = 1,
        .num_resources  = ARRAY_SIZE(olympus_wifi_resources),
        .resource       = olympus_wifi_resources,
        .dev            = {
                .platform_data = &olympus_wifi_control,
        },
};

static void __init olympus_wlan_gpio(void)
{
	tegra_gpio_enable(OLYMPUS_WLAN_RST);
	gpio_request(OLYMPUS_WLAN_RST, "wlan_rst");
	gpio_direction_output(OLYMPUS_WLAN_RST, 0);

/*STI-OLY didn't add WLAN_REG_ON_GPIO from 2.6.32.9 */

	tegra_gpio_enable(OLYMPUS_WLAN_IRQ);
	gpio_request(OLYMPUS_WLAN_IRQ, "wlan_irq");
	gpio_direction_input(OLYMPUS_WLAN_IRQ);
}

int __init olympus_wlan_init(void)
{
	pr_debug("%s: start\n", __func__);
	olympus_wlan_gpio();
	olympus_init_wifi_mem();
	return platform_device_register(&olympus_wifi_device);
}
