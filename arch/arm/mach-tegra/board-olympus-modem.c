#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/qtouch_obp_ts.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach/mdm_ctrl.h>

#include <linux/spi/spi.h>
#include <linux/spi-tegra.h>
#include <linux/spi/mdm6600_spi.h>

#include "gpio-names.h"
#include "board-olympus.h"


/*
 * Modem class control driver agent for MDM6600.
 */
void (*mot_mdm_ctrl_agent_change)(int, int);

int mot_mdm_ctrl_agent_register(void (*change)(int, int))
{
	mot_mdm_ctrl_agent_change = change;

	return 0;
}

static struct mdm6600_agent_platform_data mdm6600_agent_platform_data = {
	.mdm_ctrl_agent_register = mot_mdm_ctrl_agent_register
};

static struct platform_device mdm6600_agent_platform_device = {
	.name = "mdm6600_ctrl",
	.id = -1,
	.dev = {
		.platform_data = &mdm6600_agent_platform_data,
	},
};

static int __init mot_mdm6600_agent_init(void)
{
	return platform_device_register(&mdm6600_agent_platform_device);
}


/*
 * Legacy modem control driver for MDM6x00
 */
struct mdm_ctrl_peer_entry
{
	void (*startup)(void*);
	void (*shutdown)(void*);
	void* context;
};

#define MDM_CTRL_MAX_PEERS 8
static spinlock_t mdm_ctrl_peer_lock;
static struct mdm_ctrl_peer_entry mdm_ctrl_peer[MDM_CTRL_MAX_PEERS];
static unsigned int mdm_ctrl_peers = 0;
static bool mdm_ctrl_state = true;

int mot_mdm_ctrl_peer_register(void (*peer_startup)(void*),
                               void (*peer_shutdown)(void*),
                               void* peer_context)
{
	unsigned long flags;

	if (mdm_ctrl_peers >= MDM_CTRL_MAX_PEERS)
		return -ENOMEM;

	spin_lock_irqsave(&mdm_ctrl_peer_lock, flags);

	mdm_ctrl_peer[mdm_ctrl_peers].startup = peer_startup;
	mdm_ctrl_peer[mdm_ctrl_peers].shutdown = peer_shutdown;
	mdm_ctrl_peer[mdm_ctrl_peers].context = peer_context;
	mdm_ctrl_peers++;

	spin_unlock_irqrestore(&mdm_ctrl_peer_lock, flags);

	return 0;
}

static void mot_on_bp_startup(void)
{
	unsigned long flags;
	int i;

	if (mdm_ctrl_state)
		return;

	spin_lock_irqsave(&mdm_ctrl_peer_lock, flags);

	for (i = 0; i < mdm_ctrl_peers; i++) {
		if (mdm_ctrl_peer[i].startup)
			mdm_ctrl_peer[i].startup(mdm_ctrl_peer[i].context);
	}

	mdm_ctrl_state = true;

	spin_unlock_irqrestore(&mdm_ctrl_peer_lock, flags);
}

static void mot_on_bp_shutdown(void)
{
	unsigned long flags;
	int i;

	if (!mdm_ctrl_state)
		return;

	spin_lock_irqsave(&mdm_ctrl_peer_lock, flags);

	for (i = 0; i < mdm_ctrl_peers; i++) {
		if (mdm_ctrl_peer[i].shutdown)
			mdm_ctrl_peer[i].shutdown(mdm_ctrl_peer[i].context);
	}

	mdm_ctrl_state = false;

	spin_unlock_irqrestore(&mdm_ctrl_peer_lock, flags);
}

/* Some hacky glue between mdm_ctrl and the mdm6600 modem class driver. */
static void mot_on_bp_change(int state, int status)
{
	unsigned long flags;

	spin_lock_irqsave(&mdm_ctrl_peer_lock, flags);

	if (mot_mdm_ctrl_agent_change)
		mot_mdm_ctrl_agent_change(state, status);

	spin_unlock_irqrestore(&mdm_ctrl_peer_lock, flags);
}


#define AP_STATUS0_GPIO TEGRA_GPIO_PL0
#define AP_STATUS1_GPIO TEGRA_GPIO_PL3
#define AP_STATUS2_GPIO TEGRA_GPIO_PD5
#define BP_STATUS0_GPIO TEGRA_GPIO_PM0
#define BP_STATUS1_GPIO TEGRA_GPIO_PM1
#define BP_STATUS2_GPIO TEGRA_GPIO_PT0
#define BP_RESIN_GPIO   TEGRA_GPIO_PV1
#define BP_PSHOLD_GPIO  TEGRA_GPIO_PV1
#define BP_RESOUT_GPIO  TEGRA_GPIO_PV2
#define BP_BYPASSS_GPIO TEGRA_GPIO_PE4
#define BP_PWRON_GPIO   TEGRA_GPIO_PV0
#define BP_FLASH1_GPIO  TEGRA_GPIO_PF1
#define BP_FLASH2_GPIO  TEGRA_GPIO_PA0

static struct mdm_ctrl_platform_data mdm_ctrl_platform_data;
static struct platform_device mdm_ctrl_platform_device = {
	.name = MDM_CTRL_MODULE_NAME,
	.id = -1,
	.dev = {
		.platform_data = &mdm_ctrl_platform_data,
	},
};

static const char mdm_ctrl_usb_regulator[] = "vusb_modem_flash";

static int __init mot_mdm_ctrl_init(void)
{
	int value;

	spin_lock_init(&mdm_ctrl_peer_lock);

	mdm_ctrl_platform_data.on_bp_startup = mot_on_bp_startup;
	mdm_ctrl_platform_data.on_bp_shutdown = mot_on_bp_shutdown;
	mdm_ctrl_platform_data.on_bp_change = mot_on_bp_change;

	if ((HWREV_TYPE_IS_FINAL(system_rev) ||
			(HWREV_TYPE_IS_PORTABLE(system_rev) &&
			(HWREV_REV(system_rev) >= HWREV_REV_3))))
		mdm_ctrl_platform_data.usb_regulator =
					mdm_ctrl_usb_regulator;
	else
		/* BP_RESOUT floats on P2 and older Olympus hardware */
		mdm_ctrl_platform_data.bp_resout_quirk = true;

	mdm_ctrl_platform_data.ap_status0_gpio = AP_STATUS0_GPIO;
	mdm_ctrl_platform_data.ap_status1_gpio = AP_STATUS1_GPIO;
	mdm_ctrl_platform_data.ap_status2_gpio = AP_STATUS2_GPIO;
	mdm_ctrl_platform_data.bp_status0_gpio = BP_STATUS0_GPIO;
	mdm_ctrl_platform_data.bp_status1_gpio = BP_STATUS1_GPIO;
	mdm_ctrl_platform_data.bp_status2_gpio = BP_STATUS2_GPIO;
	mdm_ctrl_platform_data.bp_pshold_gpio = MDM_GPIO_INVALID;
	mdm_ctrl_platform_data.bp_resin_gpio = BP_RESIN_GPIO;
	mdm_ctrl_platform_data.bp_resout_gpio = BP_RESOUT_GPIO;
	mdm_ctrl_platform_data.bp_bypass_gpio = BP_BYPASSS_GPIO;
	mdm_ctrl_platform_data.bp_pwron_gpio = BP_PWRON_GPIO;
	mdm_ctrl_platform_data.bp_flash_en1_gpio = BP_FLASH1_GPIO;
	mdm_ctrl_platform_data.bp_flash_en2_gpio = BP_FLASH2_GPIO;

	mdm_ctrl_platform_data.bp_status0_gpio_irq_type =
					IRQ_TYPE_EDGE_BOTH;
	mdm_ctrl_platform_data.bp_status1_gpio_irq_type =
					IRQ_TYPE_EDGE_BOTH;
	mdm_ctrl_platform_data.bp_status2_gpio_irq_type =
					IRQ_TYPE_EDGE_BOTH;

	/*
	 * Tegra doesn't support edge triggering on GPIOs that can wake
	 * the system from deep sleep.  If the BP goes down while AP is
	 * sleeping, the AP won't notice.  So we must level trigger and
	 * toggle it in the driver.  Setting it to "high" will cause
	 * the interrupt to fire immediately so that the driver's state
	 * is accurate.
	 */
	mdm_ctrl_platform_data.bp_resout_gpio_irq_type =
					IRQ_TYPE_LEVEL_HIGH;

	gpio_request(AP_STATUS0_GPIO, "AP Status 0");
	value = gpio_get_value(AP_STATUS0_GPIO);
	gpio_direction_output(AP_STATUS0_GPIO, value);

	gpio_request(AP_STATUS1_GPIO, "AP Status 1");
	value = gpio_get_value(AP_STATUS1_GPIO);
	gpio_direction_output(AP_STATUS1_GPIO, value);

	gpio_request(AP_STATUS2_GPIO, "AP Status 2");
	value = gpio_get_value(AP_STATUS2_GPIO);
	gpio_direction_output(AP_STATUS2_GPIO, value);

	gpio_request(BP_STATUS0_GPIO, "BP Status 0");
	gpio_direction_input(BP_STATUS0_GPIO);

	gpio_request(BP_STATUS1_GPIO, "BP Status 1");
	gpio_direction_input(BP_STATUS1_GPIO);

	gpio_request(BP_STATUS2_GPIO, "BP Status 2");
	gpio_direction_input(BP_STATUS2_GPIO);

	gpio_request(BP_RESIN_GPIO, "BP Reset");
	value = gpio_get_value(BP_RESIN_GPIO);
	gpio_direction_output(BP_RESIN_GPIO, value);

	gpio_request(BP_RESOUT_GPIO, "BP Reset Output");
	gpio_direction_input(BP_RESOUT_GPIO);

	gpio_request(BP_PWRON_GPIO, "BP Power On");
	value = gpio_get_value(BP_PWRON_GPIO);
	gpio_direction_output(BP_PWRON_GPIO, value);

	return platform_device_register(&mdm_ctrl_platform_device);
}


#define MDM6600_HOST_WAKE_GPIO TEGRA_GPIO_PL1
#define MDM6600_PEER_WAKE_GPIO TEGRA_GPIO_PF2

/*
 * MDM6600 SPI IPC link configuration
 */
struct mdm6600_spi_platform_data mdm6600_spi_platform_data = {
	.gpio_mrdy = MDM6600_HOST_WAKE_GPIO,
	.gpio_srdy = MDM6600_PEER_WAKE_GPIO,
#ifdef CONFIG_MDM_CTRL
	.peer_register = mot_mdm_ctrl_peer_register,
#endif
};

struct spi_board_info tegra_spi_mdm6600_device[] __initdata = {
{
	.modalias = "mdm6600_spi",
	.bus_num = 0,
	.chip_select = 0,
	.mode = SPI_MODE_0 | SPI_CS_HIGH,
	.max_speed_hz = 26000000,
	.platform_data = &mdm6600_spi_platform_data,
	.irq = 0,
    },
};

static int __init mot_setup_mdm6600_spi_ipc(void)
{
	return spi_register_board_info(tegra_spi_mdm6600_device,
				ARRAY_SIZE(tegra_spi_mdm6600_device));
}

/*
 * MDM6600 USB IPC link configuration
 */

static struct resource mdm6600_resources[] = {
	[0] = {
		.flags = IORESOURCE_IRQ,
		.start = 0,
		.end   = 0,
	},
};

static struct platform_device mdm6600_usb_platform_device = {
	.name = "mdm6600_modem",
	.id   = -1,
	.resource = mdm6600_resources,
	.num_resources = ARRAY_SIZE(mdm6600_resources),
};

static int __init mot_setup_mdm6600_usb_ipc(int irq)
{
	if (irq) {
		gpio_request(irq, "mdm6600_usb_wakeup");
		gpio_direction_input(irq);
		mdm6600_resources[0].start = TEGRA_GPIO_TO_IRQ(irq);
		mdm6600_resources[0].end = TEGRA_GPIO_TO_IRQ(irq);
	}
	return platform_device_register(&mdm6600_usb_platform_device);
}

/*
 * Interim Wrigley host wake support.
 */
#include <linux/wakelock.h>

#define WRIGLEY_HOST_WAKE_GPIO TEGRA_GPIO_PC7

int __init mot_modem_init(void)
{
	char bp_ctrl_bus[40] = "UART";
	char bp_data_bus[20] = "only";

	if ( !(HWREV_TYPE_IS_MORTABLE(system_rev) && HWREV_REV(system_rev) <= HWREV_REV_1) ) {
		strcat(bp_ctrl_bus, " (with mdm_ctrl)");
		mot_mdm_ctrl_init();
		mot_mdm6600_agent_init();
	} else
		strcat(bp_ctrl_bus, " (NO mdm_ctrl)");

	strcpy(bp_data_bus, "and SPI");
	mot_setup_mdm6600_spi_ipc();
	mot_setup_mdm6600_usb_ipc(0);

	/* All hardware at least has MDM6x00 at the moment. */
	printk(KERN_INFO "%s: MDM6x00 on %s %s\n", __func__,
				bp_ctrl_bus, bp_data_bus);

	return 0;
}
