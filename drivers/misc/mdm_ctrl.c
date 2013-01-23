/*
     Copyright (C) 2009 Motorola, Inc.

     This program is free software; you can redistribute it and/or modify
     it under the terms of the GNU General Public License version 2 as
     published by the Free Software Foundation.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program; if not, write to the Free Software
     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
     02111-1307  USA
*/

#define DEBUG 1
#define DEBUG_FORCE_STATUS_LOGS 0

#include <linux/io.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/mdm_ctrl.h>
#include <linux/platform_device.h>
#include <linux/notifier.h>
#include <linux/poll.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/debugfs.h>
#include <linux/wakelock.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/regulator/consumer.h>

#include <mach/mdm_ctrl.h>


#define MDM_CTRL_STATE_DISABLED	0
#define MDM_CTRL_STATE_OFF	1
#define MDM_CTRL_STATE_ON	2

#define BP_STARTUP_POLL_INTERVAL    8000 /* ms */
#define BP_SHUTDOWN_POLL_INTERVAL   5000 /* ms */
#define BP_STATUS_POLL_INTERVAL     50   /* ms */
#define BP_STATUS_DEBOUNCE_INTERVAL 5    /* ms */
#define BP_PWRON_HOLD_DELAY         350  /* ms */
#define BP_RESET_HOLD_DELAY         500  /* ms */
#define BP_RESTART_ATTEMPTS	    3

#define MDM_CTRL_BP_STATUS_DONTCARE -2

#define AP_EXPECTS_BP(s) \
	((s == MDM_CTRL_AP_STATUS_NO_BYPASS) || \
	 (s == MDM_CTRL_AP_STATUS_NO_BYPASS_USB_IPC) || \
	 (s == MDM_CTRL_AP_STATUS_FLASH_MODE))
#define BP_UP(s) \
	(s == MDM_CTRL_BP_STATUS_AWAKE || \
	 s == MDM_CTRL_BP_STATUS_ASLEEP)
#define BP_READY(s) \
	(s == MDM_CTRL_BP_STATUS_AWAKE || \
	s == MDM_CTRL_BP_STATUS_RDL)


/* structure to keep track of gpio, irq, and irq enabled info */
struct gpioinfo {
	unsigned int gpio;
	unsigned int irq_type;
	int irq;
	int irq_enabled;
	int irq_enabled_mask;		/* Mask for each client */
	int irq_fired_mask;		/* Mask for each client */
	int irq_level_rising_mask;	/* Mask for each client */
	int irq_level_falling_mask;	/* Mask for each client */
	struct work_struct work;
};

struct clientinfo {
	int open;
	int mask;
	int bp_status;
	bool usb_ipc_enabled;
	wait_queue_head_t wq;
};
#define MAX_CLIENTS 4

/* BP Interface GPIO List*/
enum bpgpio {
	/* Interruptable */
	BP_RESOUT = 0,
	BP_STATUS0,
	BP_STATUS1,
	BP_STATUS2,

	BP_PWRON,

	AP_STATUS0,
	AP_STATUS1,
	AP_STATUS2,

	BP_PSHOLD,
	BP_RESIN,
	BP_BYPASS,
	BP_FLASH1,
	BP_FLASH2,

	GPIO_COUNT
};

/* Internal representation of the mdm_ctrl driver. */
struct mdm_ctrl_info {
	struct gpioinfo gpios[GPIO_COUNT];
	struct clientinfo clients[MAX_CLIENTS];
	bool bp_status_trigger_enabled;
	int bp_status_triggered_mask;
	struct mutex lock;
	struct workqueue_struct *working_queue;
	struct work_struct restart_work;
	struct wake_lock wakelock;
	struct regulator *usb_regulator;
	atomic_t state;
	atomic_t pu_state;
	bool kernel_is_atomic;
	bool usb_ipc_count;
	bool force_status_logs;
	bool bp_resout_quirk;
	void (*on_bp_startup)(void);
	void (*on_bp_shutdown)(void);
	void (*on_bp_change)(int, int);
};

/* Driver operational structure */
static struct mdm_ctrl_info mdm_ctrl;

/* The detailed wait condition used in mdm_ctrl_read().  Returns 0 only
 * when an irq has fired, otherwise always returns -1. */
static int wait_condition(void)
{
	int ret = -1;
	short i = 0;

	if (mdm_ctrl.bp_status_triggered_mask)
		return 0;

	for (i = 0; i < MDM_GPIO_INTERRUPT_COUNT; i++) {
		if (mdm_ctrl.gpios[i].irq_fired_mask) {
			ret = 0;
			break;
		}
	}

	return ret;
}

/* Frees all active interrupts and cleans up the gpio data structure */
static void clear_gpio_data(void)
{
	int i;
	for (i = 0; i < GPIO_COUNT; i++) {
		if (mdm_ctrl.gpios[i].irq != 0)
			free_irq(mdm_ctrl.gpios[i].irq, NULL);
		mdm_ctrl.gpios[i].irq_enabled_mask = 0;
		mdm_ctrl.gpios[i].irq_fired_mask = 0;
		if (mdm_ctrl.gpios[i].gpio != MDM_GPIO_INVALID)
			gpio_free(mdm_ctrl.gpios[i].gpio);
	}
	mdm_ctrl.bp_status_triggered_mask = 0;
}

static void mdm_ctrl_msleep(unsigned int ms)
{
	if (mdm_ctrl.kernel_is_atomic)
		mdelay(ms);
	else
		msleep(ms);
}

/* Get the BP->AP status */
int get_bp_status(void)
{
	int status[3] = { 0 };

	if (mdm_ctrl.gpios[BP_STATUS0].gpio != MDM_GPIO_INVALID)
		status[0] =
			gpio_get_value(mdm_ctrl.gpios[BP_STATUS0].gpio);
	if (mdm_ctrl.gpios[BP_STATUS1].gpio != MDM_GPIO_INVALID)
		status[1] =
			gpio_get_value(mdm_ctrl.gpios[BP_STATUS1].gpio);
	if (mdm_ctrl.gpios[BP_STATUS2].gpio != MDM_GPIO_INVALID)
		status[2] =
			gpio_get_value(mdm_ctrl.gpios[BP_STATUS2].gpio);

	return ((status[2] << 2) | (status[1] << 1) | status[0]);
}

static char *str_bp_status(int status)
{
	static char * const bp_status_string[8] = {
		"off",
		"panic wait",
		"core dump",
		"flash mode",
		"awake",
		"asleep",
		"shutdown acknowledge",
		"panic"
	};

	if (status < 0 || status > 7)
		return "";

	return bp_status_string[status];
}

static void mdm_ctrl_set_gpio_irq(int index, int enable)
{
	if (mdm_ctrl.gpios[index].gpio == MDM_GPIO_INVALID ||
	    mdm_ctrl.gpios[index].irq < 0 ||
	    mdm_ctrl.gpios[index].irq_enabled == enable)
		return;

	if (enable)
		enable_irq(mdm_ctrl.gpios[index].irq);
	else {
		disable_irq_nosync(mdm_ctrl.gpios[index].irq);
		if (!mdm_ctrl.kernel_is_atomic)
			synchronize_irq(mdm_ctrl.gpios[index].irq);
	}

	mdm_ctrl.gpios[index].irq_enabled = enable;
}

static void set_bp_status_irq(int enable)
{
	mdm_ctrl_set_gpio_irq(BP_STATUS0, enable);
	mdm_ctrl_set_gpio_irq(BP_STATUS1, enable);
	mdm_ctrl_set_gpio_irq(BP_STATUS2, enable);
}

/* Poll the BP for a particular status for the specified number of msec.
 * Returns:
 *   The final BP status on success or timeout
 *   -ESHUTDOWN if the BP shutdown without acknowledgement */
static int poll_for_bp_status(int status,
                              unsigned int interval_ms)
{
	int polls = interval_ms / BP_STATUS_POLL_INTERVAL;
	int i;
	int bp_status = 0;
	int bp_status_prev = -1;
	int bp_state;
	int is_shutdown = 0;

	/*
	 * This logic is a little bit crazy because of some odd behavior seen
	 * in varios BP releases.  We need to handle shutdown specially, since
	 * the real shutdown (BP_RESOUT going low) can occur some time after
	 * we receive the ACK.  Other times the shutdown happens so fast that
	 * we see BP_RESOUT before we notice the shutdown ACK.  So for shutdown
	 * we need to watch for both.  The BP team has asked that we make it
	 * clear in the logs how graceful the startup and shutdown was.
	 */
	for (i = 0; i < polls; i++) {
		bp_status = get_bp_status();
		if (bp_status != bp_status_prev) {  /* debounce */
			bp_status_prev = bp_status;
			mdm_ctrl_msleep(BP_STATUS_DEBOUNCE_INTERVAL);
			continue;
		}

		bp_state = atomic_read(&mdm_ctrl.state);

		if (bp_status == status)
			break;
		else if (!is_shutdown &&
		         status == MDM_CTRL_BP_STATUS_SHUTDOWN_ACK &&
			 bp_state == MDM_CTRL_STATE_OFF) {
			i = polls - 3;  /* check few more times */
			is_shutdown = 1;
		} else
			mdm_ctrl_msleep(BP_STATUS_POLL_INTERVAL);
	}

	if (is_shutdown &&
	    bp_status != status) {
		pr_info("%s: modem has powered down without acknowledge\n",
		        __func__);
		bp_status = -ESHUTDOWN;
	}

	return bp_status;
}

/* Poll the BP for a particular state for the specified number of msec.
 * Returns:
 *   -ETIMEDOUT if the BP did not completely shutdown as expected */
static int poll_for_bp_state(int state,
                             unsigned int interval_ms)
{
	int polls = interval_ms / BP_STATUS_POLL_INTERVAL;
	int i;
	int bp_state;
	int ret = -ETIMEDOUT;

	for (i = 0; i < polls; i++) {
		bp_state = atomic_read(&mdm_ctrl.state);

		if (bp_state == state) {
			ret = 0;
			break;
		} else
			mdm_ctrl_msleep(BP_STATUS_POLL_INTERVAL);
	}

	return ret;
}

/* Get the AP->BP status */
static int get_ap_status(void)
{
	int status[3] = { 0 };

	if (mdm_ctrl.gpios[AP_STATUS0].gpio != MDM_GPIO_INVALID)
		status[0] =
			gpio_get_value(mdm_ctrl.gpios[AP_STATUS0].gpio);
	if (mdm_ctrl.gpios[AP_STATUS1].gpio != MDM_GPIO_INVALID)
		status[1] =
			gpio_get_value(mdm_ctrl.gpios[AP_STATUS1].gpio);
	if (mdm_ctrl.gpios[AP_STATUS2].gpio != MDM_GPIO_INVALID)
		status[2] =
			gpio_get_value(mdm_ctrl.gpios[AP_STATUS2].gpio);

	return ((status[2] << 2) | (status[1] << 1) | status[0]);
}

/* Map the requested status based on the requested USB IPC value.
 * If any of the open clients have requested ipc usb, then it
 * must remain enabled.  This means mapping MDM_CTRL_AP_STATUS_NO_BYPASS
 * to MDM_CTRL_AP_STATUS_NO_BYPASS_USB_IPC. */
static int map_ap_status(int status)
{
	unsigned int i;

	switch (status) {
	case MDM_CTRL_AP_STATUS_NO_BYPASS_USB_IPC:
		status = MDM_CTRL_AP_STATUS_NO_BYPASS_USB_IPC;
		break;

	case MDM_CTRL_AP_STATUS_NO_BYPASS:
		/* If the kernel has requested USB IPC, keep it on. */
		if (mdm_ctrl.usb_ipc_count) {
			status = MDM_CTRL_AP_STATUS_NO_BYPASS_USB_IPC;
		} else {
			/* If any other file handles have requested USB IPC,
			   keep it on. */
			for (i = 0; i < MAX_CLIENTS; i++)
				if ((mdm_ctrl.clients[i].open) &&
					(mdm_ctrl.clients[i].usb_ipc_enabled)) {
					status = MDM_CTRL_AP_STATUS_NO_BYPASS_USB_IPC;
					break;
				}
		}
		break;
	}
	return status;
}

/* Set the AP->BP status. */
static void set_ap_status(int status)
{
	static int last_status = MDM_CTRL_AP_STATUS_MAX;

	pr_info("mdm_ctrl: %s(%x)\n", __func__, status);
	status = map_ap_status(status);
	if (status != last_status) {
		if (mdm_ctrl.gpios[AP_STATUS0].gpio != MDM_GPIO_INVALID)
			gpio_set_value(mdm_ctrl.gpios[AP_STATUS0].gpio,
						   (status & 0x1));
		if (mdm_ctrl.gpios[AP_STATUS1].gpio != MDM_GPIO_INVALID)
			gpio_set_value(mdm_ctrl.gpios[AP_STATUS1].gpio,
						   ((status >> 1) & 0x1));
		if (mdm_ctrl.gpios[AP_STATUS2].gpio != MDM_GPIO_INVALID)
			gpio_set_value(mdm_ctrl.gpios[AP_STATUS2].gpio,
						   ((status >> 2) & 0x1));
		last_status = status;
	}
}

static char *str_ap_status(int status)
{
	static char * const ap_status_strings[MDM_CTRL_AP_STATUS_MAX] = {
		/* MDM_CTRL_AP_STATUS_UNKNOWN */
		"unknown",
		/* MDM_CTRL_AP_STATUS_DATA_BYPASS */
		"up: data bypass",
		/* MDM_CTRL_AP_STATUS_FULL_BYPASS */
		"up: full bypass",
		/* MDM_CTRL_AP_STATUS_NO_BYPASS */
		"up: usb ipc disabled",
		/* MDM_CTRL_AP_STATUS_BP_SHUTDOWN */
		"shutdown request",
		/* MDM_CTRL_AP_STATUS_NO_BYPASS_USB_IPC */
		"up: usb ipc enabled",
		/* MDM_CTRL_AP_STATUS_FLASH_MODE */
		"flash mode",
		/* MDM_CTRL_AP_STATUS_BP_PANIC_ACK */
		"panic acknowledge"
	};

	if (status >= 0 || status < MDM_CTRL_AP_STATUS_MAX)
		return ap_status_strings[status];
	return "";
}

static void bp_update_state(void)
{
	if (((mdm_ctrl.gpios[BP_RESOUT].irq_type & IRQ_TYPE_EDGE_BOTH) &&
	     gpio_get_value(mdm_ctrl.gpios[BP_RESOUT].gpio)) ||
	    mdm_ctrl.gpios[BP_RESOUT].irq_type & IRQ_TYPE_LEVEL_HIGH) {
		atomic_set(&mdm_ctrl.state, MDM_CTRL_STATE_ON);
		pr_info("%s: modem control state ON\n", __func__);
		if (mdm_ctrl.on_bp_startup)
			mdm_ctrl.on_bp_startup();
	} else {
		atomic_set(&mdm_ctrl.state, MDM_CTRL_STATE_OFF);
		pr_info("%s: modem control state OFF\n", __func__);
		if (mdm_ctrl.on_bp_shutdown)
			mdm_ctrl.on_bp_shutdown();
	}
}

static void bp_set_flash_pins(int value)
{
	/* Enable the USB regulator if it is defined. */
	if (mdm_ctrl.usb_regulator != NULL) {
		regulator_enable(mdm_ctrl.usb_regulator);
	}

	if (mdm_ctrl.gpios[BP_FLASH1].gpio != MDM_GPIO_INVALID) {
		gpio_request(mdm_ctrl.gpios[BP_FLASH1].gpio,
					 "BP Flash Mode 1");
		gpio_direction_output(mdm_ctrl.gpios[BP_FLASH1].gpio,
							  value&0x01);
	}
	if (mdm_ctrl.gpios[BP_FLASH2].gpio != MDM_GPIO_INVALID) {
		gpio_request(mdm_ctrl.gpios[BP_FLASH2].gpio,
					 "BP Flash Mode 2");
		gpio_direction_output(mdm_ctrl.gpios[BP_FLASH2].gpio,
							  (value>>1)&0x01);
	}
}

static int bp_startup(int bp_mode)
{
	int bp_status = 0;
	int pu_result = -1;
	int poll_status = MDM_CTRL_BP_STATUS_AWAKE;

	pr_info("%s: powering up modem...\n", __func__);

	/* Check to see if the modem is already powered on */
	if (atomic_read(&mdm_ctrl.state) != MDM_CTRL_STATE_OFF) {
		pr_info("%s: modem already powered on\n", __func__);
		return 0;
	}

	/* Re-enable the BP_RESOUT on bad hardware */
	if (mdm_ctrl.bp_resout_quirk) {
		pr_info("%s: re-enable BP_RESOUT irq\n", __func__);
		mdm_ctrl.gpios[BP_RESOUT].irq_type = IRQ_TYPE_LEVEL_HIGH;
		enable_irq(mdm_ctrl.gpios[BP_RESOUT].irq);
	}

	bp_set_flash_pins(bp_mode == MDM_CTRL_BP_PU_MODE_FLASH ? 0x03 : 0x00);

	if (bp_mode == MDM_CTRL_BP_PU_MODE_FLASH)
		poll_status = MDM_CTRL_BP_STATUS_RDL;

	/* Request BP startup */
	set_ap_status(MDM_CTRL_AP_STATUS_NO_BYPASS);

	/* Take BP out of reset if we are holding it */
	if (mdm_ctrl.gpios[BP_RESIN].gpio != MDM_GPIO_INVALID)
		gpio_set_value(mdm_ctrl.gpios[BP_RESIN].gpio, 0);

	/* Tell the BP to look at AP status */
	gpio_set_value(mdm_ctrl.gpios[BP_PWRON].gpio, 1);
	mdm_ctrl_msleep(BP_PWRON_HOLD_DELAY);
	gpio_set_value(mdm_ctrl.gpios[BP_PWRON].gpio, 0);

	/* Wait for the modem to acknowledge powerup */
	bp_status = poll_for_bp_status(poll_status,
	                               BP_STARTUP_POLL_INTERVAL);

	if (bp_status == MDM_CTRL_BP_STATUS_AWAKE) {
		pr_info("%s: modem powerup in normal mode\n", __func__);
                pu_result = bp_status;
        }
        else if (bp_status == MDM_CTRL_BP_STATUS_RDL) {
                pr_info("%s: modem powerup in flash mode\n", __func__);
                pu_result = bp_status;
        }

	return (pu_result == -1) ? 0 : -1;
}

/* Shutdown the BP using gradually more drastic methods:
 *   1) request shutdown via AP status lines and wait for response
 *   2) force BP power IC to cut power to BP
 *
 * The function returns:
 *   0 on success
 *   -ESHUTDOWN if the BP shutdown while we were expecting an acknowledge
 *   -ENOTRECOVERABLE if the BP panicked during the shutdown process
 *   -ETIMEDOUT if the BP never responded and did not appear to shut down
 */
static int bp_shutdown(int force)
{
	int bp_status = 0;
	int pd_result = -ETIMEDOUT;

	/* Check to see if the modem is already powered down */
	if (!force && atomic_read(&mdm_ctrl.state) != MDM_CTRL_STATE_ON) {
		pr_info("%s: modem already powered down\n", __func__);
		return 0;
	}

	/* Disable hardware interrupts for status changes. */
	set_bp_status_irq(0);

	/* Disable client interrupts for BP_RESOUT and BP_STATUS so that the
	 * panic daemon doesn't think the BP has gone down unexpectedly. */
	mdm_ctrl.gpios[BP_RESOUT].irq_enabled = 0;
	mdm_ctrl.bp_status_trigger_enabled = false;

	/* If the BP is not up, it's not going to respond to polite requests */
	if (!BP_UP(get_bp_status()))
		goto force_shutdown;

	/* We need to call these hooks now so that any stuck interrupts don't
	 * interfere with the shutdown process.  It is safe to call them more
	 * than once. */
	if (mdm_ctrl.on_bp_shutdown)
		mdm_ctrl.on_bp_shutdown();

	pr_info("%s: powering down modem...\n", __func__);

	/* Request BP shutdown */
	set_ap_status(MDM_CTRL_AP_STATUS_BP_SHUTDOWN);

	/* Holding PWRON should lock the BP in a loop so that we can see the ACK */
	gpio_set_value(mdm_ctrl.gpios[BP_PWRON].gpio, 1);

	/* Wait for the modem to send the ACK */
	bp_status = poll_for_bp_status(MDM_CTRL_BP_STATUS_SHUTDOWN_ACK,
	                               BP_SHUTDOWN_POLL_INTERVAL);

	gpio_set_value(mdm_ctrl.gpios[BP_PWRON].gpio, 0);

	if (bp_status == MDM_CTRL_BP_STATUS_SHUTDOWN_ACK) {
		pr_info("%s: modem power down acknowledged\n", __func__);
		/* Wait for the modem to finish powerdown */
		pd_result = poll_for_bp_state(MDM_CTRL_STATE_OFF,
		                              BP_SHUTDOWN_POLL_INTERVAL);
		if (pd_result == 0)
			pr_info("%s: modem power down complete\n", __func__);
	} else if (bp_status == MDM_CTRL_BP_STATUS_PANIC ||
	           bp_status == MDM_CTRL_BP_STATUS_PANIC_WAIT) {
		pr_info("%s: modem has panicked\n", __func__);
		pd_result = ENOTRECOVERABLE;
	} else if (bp_status == -ESHUTDOWN)
		pd_result = ESHUTDOWN;

	if (pd_result < 0)
		pr_warning("%s: modem did not complete power down\n", __func__);

	/* If it didn't power down, force PMIC to cut power by holding
	 * BP_RESIN (actually PM_RESET_N via an inverter) and wait for
	 * BP_RESOUT to go low.  Skip this if we are atomic so that we
	 * don't get bitten by the watchdog. */
force_shutdown:
	if (mdm_ctrl.kernel_is_atomic)
		pr_warning("%s: kernel is atomic; giving up\n", __func__);
	else if (pd_result < 0) {
		pr_info("%s: cutting power to modem...\n", __func__);
		gpio_set_value(mdm_ctrl.gpios[BP_RESIN].gpio, 1);

		pd_result = poll_for_bp_state(MDM_CTRL_STATE_OFF,
		                              BP_SHUTDOWN_POLL_INTERVAL);

		gpio_set_value(mdm_ctrl.gpios[BP_RESIN].gpio, 0);

		/* The unpowered state of the status lines are all zero. */
		if (pd_result == 0)
			pr_info("%s: modem has powered down\n", __func__);
		else
			pr_err("%s: modem failed to power down\n", __func__);
	}

	/* Interrupts were disabled, so force a change callback. */
	if (mdm_ctrl.on_bp_change)
		mdm_ctrl.on_bp_change(atomic_read(&mdm_ctrl.state), bp_status);

	/* Re-enable client interrupts for BP_RESOUT and BP_STATUS. */
	mdm_ctrl.bp_status_trigger_enabled = true;
	mdm_ctrl.gpios[BP_RESOUT].irq_enabled = 1;

	/* Re-enable hardware interrupts for status changes. */
	set_bp_status_irq(1);

	/* Make errno's negative. */
	if (pd_result > 0)
		pd_result *= -1;

	return pd_result;
}

/* Checks to see if the client should be notified of this irq */
static int should_notify_client(struct gpioinfo *gpio, int ci)
{
	int ret = 0;
	int gpio_value = MDM_CTRL_GPIO_LOW;
	int mask = mdm_ctrl.clients[ci].mask;

	if (gpio == &mdm_ctrl.gpios[BP_RESOUT] &&
	    atomic_read(&mdm_ctrl.state) == MDM_CTRL_STATE_ON)
		gpio_value = MDM_CTRL_GPIO_HIGH;
	else
		gpio_value = gpio_get_value(gpio->gpio);

	if (gpio->irq_enabled && (gpio->irq_enabled_mask & mask))
		switch (gpio_value) {
		case MDM_CTRL_GPIO_HIGH:
			if (gpio->irq_level_rising_mask & mask)
				ret = 1;
			break;
		case MDM_CTRL_GPIO_LOW:
			if (gpio->irq_level_falling_mask & mask)
				ret = 1;
			break;
		}

	return ret;
}

static void bp_restart_work(struct work_struct *work)
{
	int attempt = 0;
	int bp_status = get_bp_status();

	while (attempt < BP_RESTART_ATTEMPTS && !BP_READY(bp_status))
	{
		pr_info("%s: forcing modem restart (attempt %d of %d)...\n",
			__func__, attempt, BP_RESTART_ATTEMPTS);
		bp_shutdown(1);
                bp_startup(atomic_read(&mdm_ctrl.pu_state));
		bp_status = get_bp_status();
		attempt++;
		pr_debug("%s: modem status=%d\n", __func__, bp_status);
	}

	if (!BP_READY(bp_status))
	{
		pr_err("%s: failed to restart modem after %d attempts: "
		       "status=%d\n", __func__, attempt, bp_status);
	}
}

static void bp_irq(struct work_struct *work)
{
	struct gpioinfo *gpio = container_of(work, struct gpioinfo, work);
	int i;
	int mask;
	bool wake_client;
	int bp_status = get_bp_status();
	static int bp_status_prev = -1;

	if (mdm_ctrl.force_status_logs)
		pr_emerg("%s: modem status: %s (%d -> %d)\n", __func__,
		         str_bp_status(bp_status), bp_status_prev, bp_status);

	/* Only log the BP status if there was a status change that was not
	 * for an awake <-> asleep transition. */
	else if ((!(bp_status == MDM_CTRL_BP_STATUS_AWAKE ||
	            bp_status == MDM_CTRL_BP_STATUS_ASLEEP) ||
	          !(bp_status_prev == MDM_CTRL_BP_STATUS_AWAKE ||
	            bp_status_prev == MDM_CTRL_BP_STATUS_ASLEEP)) &&
		 (bp_status != bp_status_prev))
		pr_info("%s: modem status: %s (%d -> %d)\n", __func__,
		        str_bp_status(bp_status), bp_status_prev, bp_status);

	mutex_lock(&mdm_ctrl.lock);

	/* mdm_ctrl.state is updated in the real IRQ handler */
	if (gpio == &mdm_ctrl.gpios[BP_RESOUT]) {
		if (atomic_read(&mdm_ctrl.state) == MDM_CTRL_STATE_ON) {
			pr_info("%s: modem has powered on\n", __func__);
			set_bp_status_irq(1);
		} else {
			pr_info("%s: modem has powered off\n", __func__);
			set_bp_status_irq(0);
		}
	}

	/* Search through clients, waking those that are waiting on this irq */
	for (i = 0; i < MAX_CLIENTS; i++)
		if (mdm_ctrl.clients[i].open) {
			mask = mdm_ctrl.clients[i].mask;
			wake_client = false;
			if (!(gpio->irq_fired_mask & mask) &&
			    (should_notify_client(gpio, i))) {
				gpio->irq_fired_mask |= mask;
				wake_up_interruptible(
					&mdm_ctrl.clients[i].wq);
				wake_client = true;
			}
			if (mdm_ctrl.bp_status_trigger_enabled &&
			    bp_status != bp_status_prev &&
			    bp_status == mdm_ctrl.clients[i].bp_status) {
				mdm_ctrl.bp_status_triggered_mask |= mask;
				wake_client = true;
			}
			if (wake_client)
				wake_up_interruptible(
					&mdm_ctrl.clients[i].wq);
		}

	/* Toggle the type if level-triggered (pseudo-edge). */
	if (gpio->irq_type & IRQ_TYPE_LEVEL_HIGH) {
		gpio->irq_type &= ~IRQ_TYPE_LEVEL_HIGH;
		gpio->irq_type |= IRQ_TYPE_LEVEL_LOW;
		irq_set_irq_type(gpio->irq, gpio->irq_type);
	} else if (gpio->irq_type & IRQ_TYPE_LEVEL_LOW) {
		gpio->irq_type &= ~IRQ_TYPE_LEVEL_LOW;
		gpio->irq_type |= IRQ_TYPE_LEVEL_HIGH;
		irq_set_irq_type(gpio->irq, gpio->irq_type);

		/* ...unless it's BP_RESOUT on buggy HW... */
		if (mdm_ctrl.bp_resout_quirk &&
		    gpio == &mdm_ctrl.gpios[BP_RESOUT]) {
			gpio->irq_type = IRQ_TYPE_NONE;
			/* Seems to be the only way to avoid stuck interrupt. */
			irq_set_irq_type(gpio->irq, IRQ_TYPE_EDGE_RISING);
			pr_debug("%s: bad BP_RESOUT signal; clobber IRQ %d\n",
				__func__, gpio->irq);
		}
	}

	mutex_unlock(&mdm_ctrl.lock);

	if (mdm_ctrl.on_bp_change)
		mdm_ctrl.on_bp_change(atomic_read(&mdm_ctrl.state), bp_status);

	bp_status_prev = bp_status;

	if (gpio->irq_type != IRQ_TYPE_NONE)
		enable_irq(gpio->irq);
}



/* When an interrupt is fired the 'irq_fired_mask' bit for the GPIO's IRQ
 * that fired is set and all listening users are told to wake up. */
irqreturn_t irq_handler(int irq, void *data)
{
	struct gpioinfo *gpio = (struct gpioinfo *)data;

	disable_irq_nosync(irq);

	/* If BP_RESOUT is edge triggered, our only chance to know the state
	 * is for a short period after the transition.  Once the BP is
	 * completely powered-off, the GPIO floats.  Otherwise, use the
	 * current level trigger type to deduce the new state. */
	if (gpio == &mdm_ctrl.gpios[BP_RESOUT] &&
	    atomic_read(&mdm_ctrl.state) != MDM_CTRL_STATE_DISABLED)
		bp_update_state();

	queue_work(mdm_ctrl.working_queue, &gpio->work);

	return IRQ_HANDLED;
}

int mdm_ctrl_startup(int pu_mode)
{
	return bp_startup(pu_mode);
}
EXPORT_SYMBOL(mdm_ctrl_startup);

int mdm_ctrl_shutdown(void)
{
	return bp_shutdown(0);
}
EXPORT_SYMBOL(mdm_ctrl_shutdown);

int mdm_ctrl_get_bp_state(void)
{
	return atomic_read(&mdm_ctrl.state);
}
EXPORT_SYMBOL(mdm_ctrl_get_bp_state);

int mdm_ctrl_get_bp_status(void)
{
	return get_bp_status();
}
EXPORT_SYMBOL(mdm_ctrl_get_bp_status);

void mdm_ctrl_set_ap_status(int status)
{
	pr_info("%s: status=%d.\n", __func__, status);
	set_ap_status(status);
}
EXPORT_SYMBOL(mdm_ctrl_set_ap_status);

void mdm_ctrl_set_usb_ipc(bool on)
{
	mutex_lock(&mdm_ctrl.lock);
	if (on)
		mdm_ctrl.usb_ipc_count++;
	else if (mdm_ctrl.usb_ipc_count)
		mdm_ctrl.usb_ipc_count--;
	set_ap_status(mdm_ctrl.usb_ipc_count ?
				  MDM_CTRL_AP_STATUS_NO_BYPASS_USB_IPC :
				  MDM_CTRL_AP_STATUS_NO_BYPASS);
	mutex_unlock(&mdm_ctrl.lock);
	pr_info("%s: usb ipc count is now: %d\n",
			__func__, mdm_ctrl.usb_ipc_count);
}
EXPORT_SYMBOL(mdm_ctrl_set_usb_ipc);

void mdm_ctrl_dump_log(void)
{
	/* Toggle BP_PWRON to trigger BP log dump */
	gpio_set_value(mdm_ctrl.gpios[BP_PWRON].gpio, 1);
	mdm_ctrl_msleep(100);
	gpio_set_value(mdm_ctrl.gpios[BP_PWRON].gpio, 0);

	/* Arbitrary delay */
	mdm_ctrl_msleep(500);
}
EXPORT_SYMBOL(mdm_ctrl_dump_log);

/* Called when a user opens a connection to the bp interface device.
 * Sets an open flag to indicate that no further connections to this
 * device are allowed. */
static int mdm_ctrl_open(struct inode *inode, struct file *filp)
{
	int ci = 0;
	int ret = 0;

	mutex_lock(&mdm_ctrl.lock);

	/* Check for an open client */
	for (ci = 0; ci < MAX_CLIENTS; ci++)
		if (!mdm_ctrl.clients[ci].open) {
			mdm_ctrl.clients[ci].open = 1;
			break;
		}

	if (ci >= MAX_CLIENTS) {
		pr_info("%s: Device is busy.\n", __func__);
		ret = -EBUSY;
	} else
		/* Store client structure on filp */
		filp->private_data = &mdm_ctrl.clients[ci];

	mutex_unlock(&mdm_ctrl.lock);

	return ret;
}

/* Called when a user releases their connection to the bp interface
 * device. Clears the flag thus allowing another user connection to
 * this device. */
static int mdm_ctrl_release(struct inode *inode, struct file *filp)
{
	struct clientinfo *cinfo = (struct clientinfo *)filp->private_data;

	mutex_lock(&mdm_ctrl.lock);

	/* Release the client structure */
	cinfo->open = 0;
	cinfo->usb_ipc_enabled = false;
	filp->private_data = 0;

	mutex_unlock(&mdm_ctrl.lock);

	return 0;
}

/* Set the user on the wait queue and tell the user that data may be
 * available soon. */
static unsigned int mdm_ctrl_poll(struct file *filp, poll_table *wait)
{
	unsigned int mask = 0;
	struct clientinfo *cinfo = (struct clientinfo *)filp->private_data;

	poll_wait(filp, &(cinfo->wq), wait);

	if (wait_condition() == 0)
		mask |= POLLIN | POLLRDNORM;

	return mask;
}

/* Returns a 1 byte bitmask containing the irq "fire status" of all
 * the BP Interface GPIOs that are capable of experiencing an interrupt.
 * Will return a value of 1 for a particular GPIO only if that GPIO's
 * IRQ is enabled and has fired.  At this point the 'fired' flag
 * for that GPIO is cleared. */
static ssize_t mdm_ctrl_read(struct file *filp, char __user * buff,
				  size_t count, loff_t *f_pos)
{
	ssize_t ret = -EINVAL;
	char gpioData[MDM_GPIO_BYTE_COUNT];
	int i;
	struct clientinfo *cinfo;
	int mask;

	mutex_lock(&mdm_ctrl.lock);

	cinfo = (struct clientinfo *)filp->private_data;
	if (!cinfo) {
		pr_info("%s: File pointer invalid.\n", __func__);
		return -EBADF;
	}
	mask = cinfo->mask;

	if (cinfo->open) {
		/* Inform calling process to sleep until the interrupt for
		 * any GPIO is fired */
		wait_event_interruptible(cinfo->wq, (wait_condition() == 0));

		ret = 0;
		if (*f_pos >= MDM_GPIO_BYTE_COUNT)
			goto out;

		memset(gpioData, 0, MDM_GPIO_BYTE_COUNT);

		/* Process the list of active interrupts, returning those
		 * that have fired */
		for (i = 0; i < MDM_GPIO_INTERRUPT_COUNT; i++) {
			/* GPIO interrupt was enabled and fired */
			if (mdm_ctrl.gpios[i].irq_enabled_mask & mask
				&& mdm_ctrl.gpios[i].irq_fired_mask & mask) {
				mdm_ctrl.gpios[i].irq_fired_mask &= ~mask;
				gpioData[0] |= 1 << i;
			}
		}
		gpioData[1] = -1;
		if (mdm_ctrl.bp_status_triggered_mask == mask) {
			if (cinfo->bp_status >= 0)
				gpioData[1] = (char)cinfo->bp_status;
			mdm_ctrl.bp_status_triggered_mask &= ~mask;
		}

		/* Restrict the copy to the amount specified */
		if (count > MDM_GPIO_BYTE_COUNT)
			count = MDM_GPIO_BYTE_COUNT;

		/* Copy the interrupt information to the buffer */
		if (copy_to_user(buff, gpioData, count)) {
			ret = -EFAULT;
			goto out;
		}
		*f_pos += count;
		ret = count;
	}

out:
	mutex_unlock(&mdm_ctrl.lock);

	return ret;
}

/* Modify the client information for enabling / disabling an irq */
static void set_interrupt(struct gpioinfo *gpio, struct clientinfo *cinfo, int enable)
{
	int mask = cinfo->mask;
	if (enable == 0) {
		gpio->irq_enabled_mask &= ~mask;
		gpio->irq_fired_mask &= ~mask;
		gpio->irq_level_rising_mask &= ~mask;
		gpio->irq_level_falling_mask &= ~mask;
	} else {
		gpio->irq_enabled_mask |= mask;
		if (enable == MDM_CTRL_IRQ_RISING)
			gpio->irq_level_rising_mask |= mask;
		else if (enable == MDM_CTRL_IRQ_FALLING)
			gpio->irq_level_falling_mask |= mask;
	}
}

/* Set the ipc enabled status of based on the provided state. */
static void ioctl_set_usb_state(int status, struct clientinfo *cinfo)
{
	switch (status) {
	case MDM_CTRL_AP_STATUS_NO_BYPASS_USB_IPC:
		cinfo->usb_ipc_enabled = true;
		break;

	case MDM_CTRL_AP_STATUS_NO_BYPASS:
		cinfo->usb_ipc_enabled = false;
		break;
	}
}

/* Performs the appropriate action for the specified IOCTL command,
 * returning a failure code only if the IOCTL command was not recognized */
static long mdm_ctrl_ioctl(struct file *filp,
				   unsigned int cmd, unsigned long data)
{
	int ret = -EINVAL;
	int *dataParam = (int *)data;
	int value = -1;
	int intParam = -1;
	struct clientinfo *cinfo = (struct clientinfo *)filp->private_data;

	if (!cinfo) {
		pr_info("%s: file pointer invalid\n", __func__);
		return -EBADF;
	}

	mutex_lock(&mdm_ctrl.lock);

	pr_debug("%s: cmd=%d\n", __func__, _IOC_NR(cmd));
	switch (cmd) {
	/* Common */
	case MDM_CTRL_IOCTL_SET_BP_PWRON:
		ret = get_user(value, dataParam);
		if (ret != 0)
			break;

		gpio_set_value(mdm_ctrl.gpios[BP_PWRON].gpio, value);
		break;

	case MDM_CTRL_IOCTL_GET_BP_RESOUT:
		ret = put_user(atomic_read(&mdm_ctrl.state) == MDM_CTRL_STATE_ON,
			       dataParam);
		break;

	case MDM_CTRL_IOCTL_SET_INT_BP_RESOUT:
		ret = get_user(intParam, dataParam);
		if (ret != 0)
			break;

		set_interrupt(&mdm_ctrl.gpios[BP_RESOUT],
			      cinfo, intParam);
		break;

	/* MDM6600 */
	case MDM_CTRL_IOCTL_SET_AP_STATUS:
		ret = get_user(value, dataParam);
		if (ret != 0)
			break;

		ioctl_set_usb_state(value, cinfo);
		set_ap_status(value);
		break;

	case MDM_CTRL_IOCTL_GET_BP_STATUS:
		value = get_bp_status();
		ret = put_user(value, dataParam);
		break;

	case MDM_CTRL_IOCTL_SET_BP_RESIN:
		if (mdm_ctrl.gpios[BP_RESIN].gpio == MDM_GPIO_INVALID)
			break;

		ret = get_user(value, dataParam);
		if (ret != 0)
			break;

		gpio_set_value(mdm_ctrl.gpios[BP_RESIN].gpio, value);
		break;

	case MDM_CTRL_IOCTL_SET_BP_BYPASS:
		if (mdm_ctrl.gpios[BP_BYPASS].gpio == MDM_GPIO_INVALID)
			break;

		ret = get_user(value, dataParam);
		if (ret != 0)
			break;

		gpio_request(mdm_ctrl.gpios[BP_BYPASS].gpio, "BP Bypass");
		gpio_direction_output(mdm_ctrl.gpios[BP_BYPASS].gpio, value);
		/* We do not release this GPIO once requested. */
		break;

	case MDM_CTRL_IOCTL_SET_BP_FLASH_MODE:
		ret = get_user(value, dataParam);
		if (ret != 0)
			break;
		bp_set_flash_pins(value);
		/* We do not release these GPIOs once requested. */
		break;

	case MDM_CTRL_IOCTL_SET_INT_BP_STATUS:
		ret = get_user(value, dataParam);
		if (ret != 0)
			break;

		if (value < 0) {
			mdm_ctrl.bp_status_triggered_mask &= ~cinfo->mask;
			cinfo->bp_status = -1;
		} else
			cinfo->bp_status = value;
		break;

	case MDM_CTRL_IOCTL_SET_BP_PSHOLD:
		if (mdm_ctrl.gpios[BP_PSHOLD].gpio == MDM_GPIO_INVALID)
			break;

		ret = get_user(value, dataParam);
		if (ret != 0)
			break;

		gpio_set_value(mdm_ctrl.gpios[BP_PSHOLD].gpio, value);
		break;

	/* Legacy support: set all flash pins to same value */
	case MDM_CTRL_IOCTL_SET_BP_FLASH_EN:
		ret = get_user(value, dataParam);
		if (ret != 0)
			break;
		bp_set_flash_pins(value != 0 ? 0x03 : 0x00);
		/* We do not release these GPIOs once requested. */
		break;

	/* High-level interfaces */
	case MDM_CTRL_IOCTL_BP_STARTUP:
                ret = get_user(intParam, dataParam);
                if(ret == 0) {
        		pr_info("%s: MDM_CTRL_IOCTL_BP_STARTUP\n", __func__);
	        	ret = bp_startup(intParam);
                        if (ret == 0) {
                                atomic_set(&mdm_ctrl.pu_state, intParam);
                                put_user(ret, dataParam);
                        }
                }
		break;

	case MDM_CTRL_IOCTL_BP_SHUTDOWN:
		pr_info("%s: MDM_CTRL_IOCTL_BP_SHUTDOWN\n", __func__);
		ret = bp_shutdown(0);
		break;

	default:
		pr_err("%s: Unknown IO control command received: %d\n",
		       __func__, cmd);
		break;
	}

	mutex_unlock(&mdm_ctrl.lock);

	return ret;
}

/* Character device file operation function pointers */
static const struct file_operations mdm_ctrl_fops = {
	.owner = THIS_MODULE,
	.open = mdm_ctrl_open,
	.release = mdm_ctrl_release,
	.poll = mdm_ctrl_poll,
	.read = mdm_ctrl_read,
	.unlocked_ioctl = mdm_ctrl_ioctl,
};

static struct miscdevice mdm_ctrl_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = MDM_CTRL_MODULE_NAME,
	.fops = &mdm_ctrl_fops,
};

/* Initializes the driver and creates the bp driver interface device
 * (/dev/mdm_ctrl). */
static int __devinit mdm_ctrl_probe(struct platform_device *pdev)
{
	int ret = 0;
	int i;
	struct mdm_ctrl_platform_data *pdata = pdev->dev.platform_data;
	int ap_status, bp_status;

	/* Initialize the client information */
	memset(&mdm_ctrl.clients, 0,
		sizeof(mdm_ctrl.clients));
	for (i = 0; i < MAX_CLIENTS; i++) {
		init_waitqueue_head(&mdm_ctrl.clients[i].wq);
		mdm_ctrl.clients[i].mask = 1 << i;
		mdm_ctrl.clients[i].bp_status = -1;
	}

	/* Need to take the lock since interrupts are configured and enabled
	   sequentially, which might trigger bp_irq() before we are ready. */
	mutex_init(&mdm_ctrl.lock);
	mutex_lock(&mdm_ctrl.lock);

	/* If level-triggered, this will be updated as soon as the interrupt
	 * gets enabled below. */
	atomic_set(&mdm_ctrl.state, MDM_CTRL_STATE_OFF);

	/* Init working queue */
	mdm_ctrl.working_queue =
	    create_singlethread_workqueue("mdm_ctrl_wq");
	if (!mdm_ctrl.working_queue) {
		pr_err("%s: failed to create work queue\n", __func__);
		ret = -ENOMEM;
		goto err_exit;
	}

#if DEBUG_FORCE_STATUS_LOGS
	mdm_ctrl.force_status_logs = true;
#endif

	/* Get the usb regulator if one was defined. */
	if (pdata->usb_regulator != NULL) {
		mdm_ctrl.usb_regulator = regulator_get(NULL, pdata->usb_regulator);
		if (IS_ERR(mdm_ctrl.usb_regulator)) {
			pr_info("%s: Unable to obtain usb regulator for flash mode: %s regulator_get returned: %ld\n",
					__func__, pdata->usb_regulator, PTR_ERR(mdm_ctrl.usb_regulator));
			mdm_ctrl.usb_regulator = NULL;
		}
	}

	/* Initialize the GPIOs */
	memset(&mdm_ctrl.gpios, 0, sizeof(mdm_ctrl.gpios));
	for (i = 0; i < GPIO_COUNT; i++) {
		mdm_ctrl.gpios[i].gpio = MDM_GPIO_INVALID;
		mdm_ctrl.gpios[i].irq = -1;
	}

	/* Common */
	mdm_ctrl.gpios[BP_PWRON].gpio = pdata->bp_pwron_gpio;
	INIT_WORK(&mdm_ctrl.gpios[BP_PWRON].work, bp_irq);

	mdm_ctrl.gpios[BP_RESOUT].gpio = pdata->bp_resout_gpio;
	mdm_ctrl.gpios[BP_RESOUT].irq_type = pdata->bp_resout_gpio_irq_type;
	INIT_WORK(&mdm_ctrl.gpios[BP_RESOUT].work, bp_irq);

	/* Tegra */
	mdm_ctrl.gpios[BP_STATUS0].gpio = pdata->bp_status0_gpio;
	mdm_ctrl.gpios[BP_STATUS0].irq_type = pdata->bp_status0_gpio_irq_type;
	INIT_WORK(&mdm_ctrl.gpios[BP_STATUS0].work, bp_irq);

	mdm_ctrl.gpios[BP_STATUS1].gpio = pdata->bp_status1_gpio;
	mdm_ctrl.gpios[BP_STATUS1].irq_type = pdata->bp_status1_gpio_irq_type;
	INIT_WORK(&mdm_ctrl.gpios[BP_STATUS1].work, bp_irq);

	mdm_ctrl.gpios[BP_STATUS2].gpio = pdata->bp_status2_gpio;
	mdm_ctrl.gpios[BP_STATUS2].irq_type = pdata->bp_status2_gpio_irq_type;
	INIT_WORK(&mdm_ctrl.gpios[BP_STATUS2].work, bp_irq);

	mdm_ctrl.gpios[AP_STATUS0].gpio = pdata->ap_status0_gpio;
	mdm_ctrl.gpios[AP_STATUS1].gpio = pdata->ap_status1_gpio;
	mdm_ctrl.gpios[AP_STATUS2].gpio = pdata->ap_status2_gpio;

	mdm_ctrl.gpios[BP_PSHOLD].gpio = pdata->bp_pshold_gpio;
	mdm_ctrl.gpios[BP_RESIN].gpio = pdata->bp_resin_gpio;
	mdm_ctrl.gpios[BP_BYPASS].gpio = pdata->bp_bypass_gpio;

	mdm_ctrl.gpios[BP_FLASH1].gpio = pdata->bp_flash_en1_gpio;
	mdm_ctrl.gpios[BP_FLASH2].gpio = pdata->bp_flash_en2_gpio;

	/* Setup all interrupts for input GPIOs */
	for (i = 0; i < MDM_GPIO_INTERRUPT_COUNT; i++) {
		if (mdm_ctrl.gpios[i].gpio == MDM_GPIO_INVALID)
			continue;

		mdm_ctrl.gpios[i].irq =
			gpio_to_irq(mdm_ctrl.gpios[i].gpio);

		ret = request_irq(mdm_ctrl.gpios[i].irq,
		                  irq_handler,
		                  mdm_ctrl.gpios[i].irq_type | IRQF_DISABLED,
		                  MDM_CTRL_MODULE_NAME,
		                  &mdm_ctrl.gpios[i]);
		if (ret < 0) {
			pr_err("%s: failed to reqeust IRQ %d\n",
			       __func__, mdm_ctrl.gpios[i].irq);
			goto err_clear_gpio;
		}

		enable_irq_wake(mdm_ctrl.gpios[i].irq);
		mdm_ctrl.gpios[i].irq_enabled = 1;
	}

	mdm_ctrl.bp_status_trigger_enabled = true;

	INIT_WORK(&mdm_ctrl.restart_work, bp_restart_work);

	mdm_ctrl.bp_resout_quirk = pdata->bp_resout_quirk;

	mdm_ctrl.on_bp_startup = pdata->on_bp_startup;
	mdm_ctrl.on_bp_shutdown = pdata->on_bp_shutdown;
	mdm_ctrl.on_bp_change = pdata->on_bp_change;

	ret = misc_register(&mdm_ctrl_misc_device);
	if (ret < 0) {
		pr_err("%s: misc device registration failed\n", __func__);
		goto err_unregister;
	}

	ap_status = get_ap_status();
	bp_status = get_bp_status();
	pr_info("%s: requested status: %s (%d)\n",
		__func__, str_ap_status(ap_status), ap_status);
	pr_info("%s: modem status: %s (%d)\n",
		__func__, str_bp_status(bp_status), bp_status);

	/* ensure that flash mode pins are set correctly */
	if(ap_status == MDM_CTRL_AP_STATUS_FLASH_MODE) {
		pr_info("%s: Ensure flash mode pins are set correctly\n",
				__func__);
		bp_set_flash_pins(0x03);
	}

	/* update bp power up state. used to track whether bp should power up
	   in normal or flash mode */
	if (ap_status == MDM_CTRL_AP_STATUS_FLASH_MODE)
		atomic_set(&mdm_ctrl.pu_state, MDM_CTRL_BP_PU_MODE_FLASH);
	else
		atomic_set(&mdm_ctrl.pu_state, MDM_CTRL_BP_PU_MODE_NORMAL);

	mutex_unlock(&mdm_ctrl.lock);

	if (AP_EXPECTS_BP(ap_status)) {
		/* First just try sending a synchronous startup request. */
		if (!BP_READY(bp_status)) {
                        bp_startup(atomic_read(&mdm_ctrl.pu_state));
			bp_status = get_bp_status();
		}

		/* Next try restarting it a few times, but do it in the background */
		if (!BP_READY(bp_status))
			queue_work(mdm_ctrl.working_queue, &mdm_ctrl.restart_work);
		else
			atomic_set(&mdm_ctrl.state, MDM_CTRL_STATE_ON);
	}

	return ret;

	/* Handling for error conditions */
err_unregister:
	misc_deregister(&mdm_ctrl_misc_device);
err_clear_gpio:
	clear_gpio_data();
	destroy_workqueue(mdm_ctrl.working_queue);
err_exit:
	atomic_set(&mdm_ctrl.state, MDM_CTRL_STATE_DISABLED);

	return ret;
}

/* Destroys the bp driver interface device and clears up any pending IRQs */
static int __devexit mdm_ctrl_driver_remove(struct platform_device *pdev)
{
	atomic_set(&mdm_ctrl.state, MDM_CTRL_STATE_DISABLED);

	misc_deregister(&mdm_ctrl_misc_device);

	/* Free any GPIO IRQs requested but not yet freed */
	clear_gpio_data();

	destroy_workqueue(mdm_ctrl.working_queue);

	pr_info("%s: unloaded mdm_ctrl device driver\n", __func__);
	return 0;
}

/* Initiate modem power down */
static void __devexit mdm_ctrl_driver_shutdown(struct platform_device *pdev)
{
	pr_info("%s: shutdown requested by OS\n", __func__);
	bp_shutdown(0);
}

#if defined(CONFIG_PM)
/* This is a hack to work around an AP20 quirk where waking from LP0 can cause
 * us to miss a short pulse on BP_RESOUT, even though it is level-triggered. */
#define WAKE_REASON_ADDR 0x7000e414
#define WAKE_REASON_MASK 0x01000000	/* PV2 */
static int mdm_ctrl_resume(struct device *dev)
{
	unsigned long wake_reason = readl(IO_ADDRESS(WAKE_REASON_ADDR));

	pr_info("%s: wake reason: 0x%08lX\n", __func__, wake_reason);
	if((wake_reason & WAKE_REASON_MASK) &&
	   atomic_read(&mdm_ctrl.state) != MDM_CTRL_STATE_DISABLED) {
		wake_lock_timeout(&mdm_ctrl.wakelock, (HZ * 1));
		disable_irq(mdm_ctrl.gpios[BP_RESOUT].irq);
		bp_update_state();
		bp_irq(&mdm_ctrl.gpios[BP_RESOUT].work);
		/* Try to force the scheduler to run the ugly way. */
		run_local_timers();
	}

	return 0;
}

static struct dev_pm_ops mdm_ctrl_pm_ops = {
	.resume = mdm_ctrl_resume,
};
#define mdm_ctrl_pm &mdm_ctrl_pm_ops
#else
#define mdm_ctrl_pm NULL
#endif

static struct platform_driver mdm_ctrl_driver = {
	.probe = mdm_ctrl_probe,
	.remove = __devexit_p(mdm_ctrl_driver_remove),
	.shutdown = __devexit_p(mdm_ctrl_driver_shutdown),
	.driver = {
		   .name = MDM_CTRL_MODULE_NAME,
		   .owner = THIS_MODULE,
		   .pm = mdm_ctrl_pm,
		   },
};

static int mdm_ctrl_dbg_get(void *data, u64 *val)
{
	if (val)
		*val = (mdm_ctrl.force_status_logs) ? 1 : 0;

	return 0;
}

static int mdm_ctrl_dbg_set(void *data, u64 val)
{
	mdm_ctrl.force_status_logs = (val != 0);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(mdm_ctrl_dbg_fops, mdm_ctrl_dbg_get,
                        mdm_ctrl_dbg_set, "%llu\n");

/* Force BP shutdown on panic */
static int mdm_ctrl_panic(struct notifier_block *this, unsigned long event,
			  void *ptr)
{
	int bp_status;
	int ret;

	if (atomic_read(&mdm_ctrl.state) == MDM_CTRL_STATE_DISABLED)
		return -ENODEV;

	bp_status = get_bp_status();
	pr_info("%s: modem status: %s (%d)\n",
		__func__, str_bp_status(bp_status), bp_status);

	if (atomic_read(&mdm_ctrl.state) == MDM_CTRL_STATE_OFF)
		return -ESHUTDOWN;

	/* Assume we are atomic...there is no reliable way to determine this
	 * from a driver.  :( */
	mdm_ctrl.kernel_is_atomic = 1;

	pr_info("%s: shutting down modem\n", __func__);
	ret = bp_shutdown(1);

	mdm_ctrl.kernel_is_atomic = 0;
	return ret;
}

static struct notifier_block mdm_ctrl_panic_notifier = {
	.notifier_call	= mdm_ctrl_panic,
};

static int __init mdm_ctrl_os_init(void)
{
	struct dentry *root;

	root = debugfs_create_dir(MDM_CTRL_MODULE_NAME, NULL);
	if (root)
		debugfs_create_file("log_bp_status", 0644, root, NULL,
		                    &mdm_ctrl_dbg_fops);

	atomic_notifier_chain_register(&panic_notifier_list,
	                               &mdm_ctrl_panic_notifier);

	wake_lock_init(&mdm_ctrl.wakelock, WAKE_LOCK_SUSPEND,
	               MDM_CTRL_MODULE_NAME" wakelock");

	mdm_ctrl.usb_regulator = NULL;
	return platform_driver_register(&mdm_ctrl_driver);
}

static void __exit mdm_ctrl_os_exit(void)
{
	platform_driver_unregister(&mdm_ctrl_driver);
	wake_lock_destroy(&mdm_ctrl.wakelock);
}

module_init(mdm_ctrl_os_init);
module_exit(mdm_ctrl_os_exit);

MODULE_AUTHOR("Motorola");
MODULE_DESCRIPTION("Modem Control Driver");
MODULE_VERSION("1.1.3");
MODULE_LICENSE("GPL");
