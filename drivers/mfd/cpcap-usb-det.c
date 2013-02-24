/*
 * Copyright (C) 2007 - 2011 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/errno.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/switch.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/usb/otg.h>

#include <linux/spi/cpcap.h>
#include <linux/spi/cpcap-regbits.h>
#include <linux/spi/spi.h>

#include <linux/cpcap-accy.h>

#define CPCAP_SENSE4_LS		8
#define CPCAP_BIT_DP_S_LS	(CPCAP_BIT_DP_S << CPCAP_SENSE4_LS)
#define CPCAP_BIT_DM_S_LS	(CPCAP_BIT_DM_S << CPCAP_SENSE4_LS)


#define SENSE_USB           (CPCAP_BIT_ID_FLOAT_S  | \
			     CPCAP_BIT_CHRGCURR1_S | \
			     CPCAP_BIT_VBUSVLD_S   | \
			     CPCAP_BIT_SESSVLD_S)

#define SENSE_USB_FLASH     (CPCAP_BIT_CHRGCURR1_S | \
			     CPCAP_BIT_VBUSVLD_S   | \
			     CPCAP_BIT_SESSVLD_S)

/* Tegra has Fact_Comp, so AP20 controls DP, therefore host can't pull down
 * DP, and it will show High, therefore add DP sense bit to mask.
 */
#define SENSE_FACTORY       (CPCAP_BIT_ID_FLOAT_S  | \
			     CPCAP_BIT_ID_GROUND_S | \
			     CPCAP_BIT_CHRGCURR1_S | \
			     CPCAP_BIT_VBUSVLD_S   | \
			     CPCAP_BIT_SESSVLD_S   | \
			     CPCAP_BIT_DP_S_LS)

/* This Sense mask is needed because on TI the CHRGCURR1 sense bit
 * is not always set. In Factory Mode the comparator follows the
 * Charge current only.
 */
#define SENSE_FACTORY_COM   (CPCAP_BIT_ID_FLOAT_S  | \
			     CPCAP_BIT_ID_GROUND_S | \
			     CPCAP_BIT_VBUSVLD_S   | \
			     CPCAP_BIT_SESSVLD_S   | \
			     CPCAP_BIT_DP_S_LS)

#define SENSE_CHARGER_FLOAT (CPCAP_BIT_ID_FLOAT_S  | \
			     CPCAP_BIT_CHRGCURR1_S | \
			     CPCAP_BIT_VBUSVLD_S   | \
			     CPCAP_BIT_SESSVLD_S   | \
			     CPCAP_BIT_SE1_S       | \
			     CPCAP_BIT_DM_S_LS     | \
			     CPCAP_BIT_DP_S_LS)

#define SENSE_CHARGER       (CPCAP_BIT_CHRGCURR1_S | \
			     CPCAP_BIT_VBUSVLD_S   | \
			     CPCAP_BIT_SESSVLD_S   | \
			     CPCAP_BIT_SE1_S       | \
			     CPCAP_BIT_DM_S_LS     | \
			     CPCAP_BIT_DP_S_LS)

#define SENSE_WHISPER_PPD   (CPCAP_BIT_CHRGCURR1_S | \
			     CPCAP_BIT_DP_S_LS)

#define SENSE_WHISPER_PPD_NO_DP   (CPCAP_BIT_CHRGCURR1_S)

#define SENSE_WHISPER_SPD   SENSE_CHARGER

/* Since phone is host, no one to pull DP down, therefore it will be high
 * and needs to be added to sense mask
 */
#define SENSE_WHISPER_SMART (CPCAP_BIT_ID_GROUND_S | \
			     CPCAP_BIT_VBUSVLD_S   | \
			     CPCAP_BIT_SESSVLD_S   | \
			     CPCAP_BIT_CHRGCURR1_S | \
			     CPCAP_BIT_DP_S_LS)

#define ADC_AUDIO_THRES     0x12C

enum cpcap_det_state {
	CONFIG,
	SAMPLE_1,
	SAMPLE_2,
	IDENTIFY,
	USB,
	FACTORY,
	CHARGER,
	USB_DEVICE,
	WHISPER_PPD,
	WHISPER_SPD,
	WHISPER_SMART_DOCK,
};

static const char *accy_names[9] = {"USB", "FACTORY", "CHARGER", "USB_DEVICE", "WHISPER_PPD",
				"WHISPER_SPD", "WHISPER_SMART_DOCK", "NONE", "UNKNOWN"};

static const char *state_names[11] = {"CONFIG", "SAMPLE_1", "SAMPLE_2", "IDENTIFY", "USB", "FACTORY",
				"CHARGER", "USB_DEVICE", "WHISPER_PPD", "WHISPER_SPD", "WHISPER_SMART_DOCK"};

static const char *irq_names[23] = {"NA","NA","NA","NA","NA","NA","NA","NA","NA","NA","NA",
					"CPCAP_IRQ_VBUSOV",
					"CPCAP_IRQ_RVRS_CHRG",
					"CPCAP_IRQ_CHRG_DET",
					"CPCAP_IRQ_IDFLOAT",
					"CPCAP_IRQ_IDGND",
					"CPCAP_IRQ_SE1",
					"CPCAP_IRQ_SESSEND",
					"CPCAP_IRQ_SESSVLD",
					"CPCAP_IRQ_VBUSVLD",
					"CPCAP_IRQ_CHRG_CURR1",
					"CPCAP_IRQ_CHRG_CURR2",
					"CPCAP_IRQ_RVRS_MODE"};

enum {
	NO_DOCK,
	DESK_DOCK,
	CAR_DOCK,
	MOBILE_DOCK,
	HD_DOCK,
	MAX_DOCK,
};

enum {
	EXT_NO_DOCK,
	EXT_DESK_DOCK,
	EXT_CAR_DOCK,
	EXT_MOBILE_DOCK,
	EXT_HD_DOCK,
};

enum {
	NO_DEVICE,
	HEADSET_WITH_MIC,
	HEADSET_WITHOUT_MIC,
	SPDIF_AUDIO_OUT = 0x4,
};

enum {
	AUTH_NOT_STARTED,
	AUTH_IN_PROGRESS,
	AUTH_FAILED,
	AUTH_PASSED,
};

struct cpcap_usb_det_data {
	struct cpcap_device *cpcap;
	struct delayed_work work;
	struct workqueue_struct *wq;
	struct cpcap_whisper_pdata *pdata;
	unsigned short sense;
	unsigned short prev_sense;
	enum cpcap_det_state state;
	enum cpcap_accy usb_accy;
	struct platform_device *usb_dev;
	struct platform_device *usb_connected_dev;
	struct platform_device *charger_connected_dev;
	struct regulator *regulator;
	struct wake_lock wake_lock;
	unsigned char is_vusb_enabled;
	struct switch_dev wsdev;
	struct switch_dev sdsdev;
	struct switch_dev emusdev;
	struct switch_dev dsdev;
	struct switch_dev edsdev;
	unsigned char audio;
	unsigned char uartmux;
	short whisper_auth;
	bool hall_effect_connected;
	enum cpcap_irqs irq;
	struct otg_transceiver *otg;
};

static const char *accy_devices[] = {
	"cpcap_usb_charger",
	"cpcap_factory",
	"cpcap_charger",
	"cpcap_usb_device",
	"cpcap_whisper_ppd",
	"cpcap_whisper_spd",
	"cpcap_whisper_smart_dock",
};

static DEFINE_MUTEX(switch_access);

/* Expects values from 0 to 2: 0=no_log, 1=basic_log, 2=max_log */
static int cpcap_usb_det_debug = 1;
module_param(cpcap_usb_det_debug, int, S_IRUGO | S_IWUSR | S_IWGRP);

static ssize_t dock_print_name(struct switch_dev *switch_dev, char *buf)
{
	switch (switch_get_state(switch_dev)) {
	case NO_DOCK:
		return sprintf(buf, "None\n");
	case DESK_DOCK:
		return sprintf(buf, "DESK\n");
	case CAR_DOCK:
		return sprintf(buf, "CAR\n");
	}

	return -EINVAL;
}

static ssize_t ext_dock_print_name(struct switch_dev *switch_dev, char *buf)
{
	switch (switch_get_state(switch_dev)) {
	case EXT_NO_DOCK:
		return sprintf(buf, "None\n");
	case EXT_DESK_DOCK:
		return sprintf(buf, "DESK\n");
	case EXT_CAR_DOCK:
		return sprintf(buf, "CAR\n");
	case EXT_MOBILE_DOCK:
		return sprintf(buf, "MOBILE\n");
	case EXT_HD_DOCK:
		return sprintf(buf, "HD\n");
	}

	return -EINVAL;
}

static ssize_t emu_audio_print_name(struct switch_dev *sdev, char *buf)
{
        switch (switch_get_state(sdev)) {
        case NO_DEVICE:
                return sprintf(buf, "No Device\n");
        case HEADSET_WITH_MIC:
                return sprintf(buf, "Mono out\n");
        case HEADSET_WITHOUT_MIC:
                return sprintf(buf, "Stereo out\n");
        case SPDIF_AUDIO_OUT:
                return sprintf(buf, "SPDIF audio out\n");
        }

        return -EINVAL;
}

void whisper_toggle_audio_switch_for_spdif(struct cpcap_device *cpcap, bool state)
{
	struct cpcap_usb_det_data *data = cpcap->accydata;
        if (data) {
		if (state)
			switch_set_state(&data->emusdev, SPDIF_AUDIO_OUT);
		else
			switch_set_state(&data->emusdev, NO_DEVICE);
	}
}

void cpcap_accy_set_dock_switch(struct cpcap_device *cpcap, int state, bool is_hall_effect)
{
    struct cpcap_usb_det_data *data = cpcap->accydata;

    if (data->regulator != NULL) {
		mutex_lock(&switch_access);

		/* if request from whisper docks, then can override hall_effect docks
		   OR if request from hall_effect, then can only change state if either
		   no-dock connected or if a hall_effect dock was already connected
		*/
		if (!is_hall_effect || (is_hall_effect && (switch_get_state(&data->dsdev)
				== NO_DOCK || data->hall_effect_connected))) {
			if (!is_hall_effect) {
				/* whisper resets previous dock states */
				switch_set_state(&data->edsdev, EXT_NO_DOCK);
				switch_set_state(&data->dsdev, NO_DOCK);
			}

			switch (state) {
			case DESK_DOCK:
				switch_set_state(&data->edsdev, EXT_DESK_DOCK);
				switch_set_state(&data->dsdev, DESK_DOCK);
				data->hall_effect_connected = is_hall_effect;
				break;
			case CAR_DOCK:
				switch_set_state(&data->edsdev, EXT_CAR_DOCK);
				switch_set_state(&data->dsdev, CAR_DOCK);
				data->hall_effect_connected = is_hall_effect;
				break;
			case HD_DOCK:
				switch_set_state(&data->edsdev, EXT_HD_DOCK);
				switch_set_state(&data->dsdev, DESK_DOCK);
				data->hall_effect_connected = is_hall_effect;
				break;
			case MOBILE_DOCK:
				switch_set_state(&data->edsdev, EXT_MOBILE_DOCK);
				switch_set_state(&data->dsdev, DESK_DOCK);
				data->hall_effect_connected = is_hall_effect;
				break;
			default:
				switch_set_state(&data->edsdev, EXT_NO_DOCK);
				switch_set_state(&data->dsdev, NO_DOCK);
				data->hall_effect_connected = false;
				break;
			}
		}
		mutex_unlock(&switch_access);
    }
}

static void vusb_enable(struct cpcap_usb_det_data *data)
{
	if (!data->is_vusb_enabled) {
		wake_lock(&data->wake_lock);
		regulator_enable(data->regulator);
		data->is_vusb_enabled = 1;
	}
}

static void vusb_disable(struct cpcap_usb_det_data *data)
{
	if (data->is_vusb_enabled) {
		wake_unlock(&data->wake_lock);
		regulator_disable(data->regulator);
		data->is_vusb_enabled = 0;
	}
}

static void dump_sense_bits(struct cpcap_usb_det_data *data)
{
	if (CPCAP_BIT_CHRGCURR1_S & data->sense)
		pr_info("cpcap_usb_det: SenseBit = CPCAP_BIT_CHRGCURR1_S\n");
	if (CPCAP_BIT_DM_S_LS & data->sense)
		pr_info("cpcap_usb_det: SenseBit = CPCAP_BIT_DM_S_LS\n");
	if (CPCAP_BIT_DP_S_LS & data->sense)
		pr_info("cpcap_usb_det: SenseBit = CPCAP_BIT_DP_S_LS)\n");
	if (CPCAP_BIT_ID_FLOAT_S & data->sense)
		pr_info("cpcap_usb_det: SenseBit = CPCAP_BIT_ID_FLOAT_S\n");
	if (CPCAP_BIT_ID_GROUND_S & data->sense)
		pr_info("cpcap_usb_det: SenseBit = CPCAP_BIT_ID_GROUND_S\n");
	if (CPCAP_BIT_SE1_S & data->sense)
		pr_info("cpcap_usb_det: SenseBit = CPCAP_BIT_SE1_S\n");
	if (CPCAP_BIT_SESSVLD_S & data->sense)
		pr_info("cpcap_usb_det: SenseBit = CPCAP_BIT_SESSVLD_S\n");
	if (CPCAP_BIT_VBUSVLD_S & data->sense)
		pr_info("cpcap_usb_det: SenseBit = CPCAP_BIT_VBUSVLD_S\n");

	if (SENSE_CHARGER == data->sense)
		pr_info("cpcap_usb_det: Sense Pattern = SENSE_CHARGER\n");
	if (SENSE_CHARGER_FLOAT == data->sense)
		pr_info("cpcap_usb_det: Sense Pattern = SENSE_CHARGER_FLOAT\n");
	if (SENSE_FACTORY == data->sense)
		pr_info("cpcap_usb_det: Sense Pattern = SENSE_FACTORY\n");
	if (SENSE_FACTORY_COM == data->sense)
		pr_info("cpcap_usb_det: Sense Pattern = SENSE_FACTORY_COM\n");
	if (SENSE_USB == data->sense)
		pr_info("cpcap_usb_det: Sense Pattern = SENSE_USB\n");
	if (SENSE_USB_FLASH == data->sense)
		pr_info("cpcap_usb_det: Sense Pattern = SENSE_USB_FLASH\n");
	if (SENSE_WHISPER_PPD == data->sense)
		pr_info("cpcap_usb_det: Sense Pattern = SENSE_WHISPER_PPD\n");
	if (SENSE_WHISPER_PPD_NO_DP == data->sense)
		pr_info("cpcap_usb_det: Sense Patt=SENSE_WHISPER_PPD_NO_DP\n");
	if (SENSE_WHISPER_SMART == data->sense)
		pr_info("cpcap_usb_det: Sense Pattern = SENSE_WHISPER_SMART\n");
	if (SENSE_WHISPER_SPD == data->sense)
		pr_info("cpcap_usb_det: Sense Pattern = SENSE_WHISPER_SPD\n");
}

static int get_sense(struct cpcap_usb_det_data *data)
{
	int retval = -EFAULT;
	unsigned short value;
	struct cpcap_device *cpcap;

	if (!data)
		return -EFAULT;

	cpcap = data->cpcap;

	retval = cpcap_regacc_read(cpcap, CPCAP_REG_INTS1, &value);
	if (retval)
		return retval;

	/* Clear ASAP after read. */
	retval = cpcap_regacc_write(cpcap, CPCAP_REG_INT1,
				     (CPCAP_BIT_CHRG_DET_I |
				      CPCAP_BIT_ID_FLOAT_I |
				      CPCAP_BIT_ID_GROUND_I),
				     (CPCAP_BIT_CHRG_DET_I |
				      CPCAP_BIT_ID_FLOAT_I |
				      CPCAP_BIT_ID_GROUND_I));
	if (retval)
		return retval;

	data->sense = value & (CPCAP_BIT_ID_FLOAT_S |
			       CPCAP_BIT_ID_GROUND_S);

	retval = cpcap_regacc_read(cpcap, CPCAP_REG_INTS2, &value);
	if (retval)
		return retval;

	/* Clear ASAP after read. */
	retval = cpcap_regacc_write(cpcap, CPCAP_REG_INT2,
				    (CPCAP_BIT_CHRGCURR1_I |
				     CPCAP_BIT_VBUSVLD_I |
				     CPCAP_BIT_SESSVLD_I |
				     CPCAP_BIT_SE1_I),
				    (CPCAP_BIT_CHRGCURR1_I |
				     CPCAP_BIT_VBUSVLD_I |
				     CPCAP_BIT_SESSVLD_I |
				     CPCAP_BIT_SE1_I));
	if (retval)
		return retval;

	data->sense |= value & (CPCAP_BIT_CHRGCURR1_S |
				CPCAP_BIT_VBUSVLD_S |
				CPCAP_BIT_SESSVLD_S |
				CPCAP_BIT_SE1_S);

	retval = cpcap_regacc_read(cpcap, CPCAP_REG_INTS4, &value);
	if (retval)
		return retval;

	/* Clear ASAP after read. */
	retval = cpcap_regacc_write(cpcap, CPCAP_REG_INT4,
				     (CPCAP_BIT_DP_I |
				      CPCAP_BIT_DM_I),
				     (CPCAP_BIT_DP_I |
				      CPCAP_BIT_DM_I));
	if (retval)
		return retval;

	data->sense |= (value & (CPCAP_BIT_DP_S |
			       CPCAP_BIT_DM_S)) << CPCAP_SENSE4_LS;

	if (cpcap_usb_det_debug && data->state > SAMPLE_2) {
		pr_info("cpcap_usb_det: SenseBits = 0x%04x\n", data->sense);
		if (cpcap_usb_det_debug > 1) {
		    dump_sense_bits(data);
		}
	}

	return 0;
}

static int configure_hardware(struct cpcap_usb_det_data *data, enum cpcap_accy accy)
{
	int retval;

	retval = cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC1,
				    CPCAP_BIT_DP150KPU,
				    (CPCAP_BIT_DP150KPU | CPCAP_BIT_DP1K5PU |
				     CPCAP_BIT_DM1K5PU | CPCAP_BIT_DPPD |
				     CPCAP_BIT_DMPD));

	switch (accy) {
	case CPCAP_ACCY_USB:
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC1, 0,
					     CPCAP_BIT_VBUSPD);
		gpio_set_value(data->pdata->data_gpio, 1);
		if (data->otg)
			raw_notifier_call_chain((void *)&data->otg->notifier.head,
						     USB_EVENT_VBUS, NULL);
		break;
	case CPCAP_ACCY_USB_DEVICE:
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC1, 0,
					     CPCAP_BIT_VBUSPD);
		gpio_set_value(data->pdata->data_gpio, 1);
		if (data->otg)
			raw_notifier_call_chain((void *)&data->otg->notifier.head,
						     USB_EVENT_ID, NULL);
		break;
	case CPCAP_ACCY_FACTORY:
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC1, 0,
					     CPCAP_BIT_VBUSPD);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC2,
					     CPCAP_BIT_USBXCVREN,
					     CPCAP_BIT_USBXCVREN);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC3, 0,
					     CPCAP_BIT_VBUSSTBY_EN);

		if ((data->cpcap->vendor == CPCAP_VENDOR_ST) &&
			(data->cpcap->revision == CPCAP_REVISION_2_0))
				vusb_enable(data);
		break;

	case CPCAP_ACCY_CHARGER:
		/* Disable Reverse Mode */
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_CRM,
					     0, CPCAP_BIT_RVRSMODE);
		/* Enable VBus PullDown */
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC1,
					     CPCAP_BIT_VBUSPD,
					     CPCAP_BIT_VBUSPD);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC3, 0,
					     CPCAP_BIT_VBUSSTBY_EN);
		break;

	case CPCAP_ACCY_WHISPER_PPD:
		/* Remove VBus PullDown */
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC1, 0,
					     CPCAP_BIT_VBUSPD);
		/* Enable Reverse Mode */
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_CRM,
					     CPCAP_BIT_RVRSMODE,
					     CPCAP_BIT_RVRSMODE);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC3, CPCAP_BIT_VBUSSTBY_EN,
					     CPCAP_BIT_VBUSSTBY_EN);
		break;

	case CPCAP_ACCY_UNKNOWN:
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC1, 0,
					     CPCAP_BIT_VBUSPD);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC1, 0,
					     (CPCAP_BIT_VBUSPD |
					      CPCAP_BIT_ID100KPU));
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC2, CPCAP_BIT_USBSUSPEND,
					      CPCAP_BIT_UARTMUX1 | CPCAP_BIT_UARTMUX0 |
					      CPCAP_BIT_EMUMODE2 |
					      CPCAP_BIT_EMUMODE1 |
					      CPCAP_BIT_EMUMODE0 |
					      CPCAP_BIT_USBSUSPEND);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_VUSBC, 0,
					     CPCAP_BIT_VBUS_SWITCH);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_CRM, 0,
					     CPCAP_BIT_RVRSMODE);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC3, CPCAP_BIT_VBUSSTBY_EN,
					     CPCAP_BIT_VBUSSTBY_EN);
		data->whisper_auth = AUTH_NOT_STARTED;
		tegra_gpio_enable(data->cpcap->spdif_gpio);
		break;

	case CPCAP_ACCY_WHISPER_SMART_DOCK:
		/* Enable VBus PullDown */
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC1,
					     CPCAP_BIT_VBUSPD,
					     CPCAP_BIT_VBUSPD);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC3, 0,
					     CPCAP_BIT_VBUSSTBY_EN);
		break;

	case CPCAP_ACCY_NONE:
	default:
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_CRM, 0,
					     CPCAP_BIT_RVRSMODE);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_VUSBC, 0,
					     CPCAP_BIT_VBUS_SWITCH);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC1,
					     CPCAP_BIT_VBUSPD,
					     CPCAP_BIT_VBUSPD);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC2, CPCAP_BIT_USBSUSPEND,
					     CPCAP_BIT_USBXCVREN | CPCAP_BIT_USBSUSPEND);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC3, CPCAP_BIT_VBUSSTBY_EN,
					     CPCAP_BIT_VBUSSTBY_EN);
		data->whisper_auth = AUTH_NOT_STARTED;
		tegra_gpio_enable(data->cpcap->spdif_gpio);
		break;
	}

	if (retval != 0)
		retval = -EFAULT;

	return retval;
}

static inline bool vbus_valid_adc_check(struct cpcap_usb_det_data *data)
{
	struct cpcap_adc_request req;
	int ret;

	req.format = CPCAP_ADC_FORMAT_CONVERTED;
	req.timing = CPCAP_ADC_TIMING_IMM;
	req.type = CPCAP_ADC_TYPE_BANK_0;

	ret = cpcap_adc_sync_read(data->cpcap, &req);

	if (cpcap_usb_det_debug)
		pr_info("cpcap_usb_det: BATTP=%d, VBUS=%d, BATTI=%d, CHG_ISENSE=%d, ret=%d, status=%d\n",
			req.result[CPCAP_ADC_BATTP], req.result[CPCAP_ADC_VBUS], req.result[CPCAP_ADC_BATTI_ADC],
			req.result[CPCAP_ADC_CHG_ISENSE], ret, req.status);

	return ((req.result[CPCAP_ADC_CHG_ISENSE] < 100) &&
		(req.result[CPCAP_ADC_VBUS] < (req.result[CPCAP_ADC_BATTP]+100))) ? false : true;
}

static void whisper_audio_check(struct cpcap_usb_det_data *data)
{
	struct cpcap_adc_request req;
	int ret;
	unsigned short value;

	cpcap_regacc_read(data->cpcap, CPCAP_REG_USBC1, &value);
	value &= CPCAP_BIT_ID100KPU;

	/* When IDPUCNTRL is High, the 220K ID PU is disconnected
	 * and a 5uA current source powerd from the VUSB reg is switched on.
	 * With this current src, the ID pin is routed to ADC to determine
	 * the resister is connected to the pin.
	 */
	cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC1, CPCAP_BIT_IDPUCNTRL,
			   (CPCAP_BIT_ID100KPU | CPCAP_BIT_IDPUCNTRL));

	msleep(200);

	req.format = CPCAP_ADC_FORMAT_RAW;
	req.timing = CPCAP_ADC_TIMING_IMM;
	req.type = CPCAP_ADC_TYPE_BANK_0;

	ret = cpcap_adc_sync_read(data->cpcap, &req);

	cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC1, value,
			   (CPCAP_BIT_ID100KPU | CPCAP_BIT_IDPUCNTRL));

	data->audio = (req.result[CPCAP_ADC_USB_ID] > ADC_AUDIO_THRES) ? 1 : 0;

	if (data->audio == 1)
		switch_set_state(&data->emusdev, HEADSET_WITHOUT_MIC);
	else
		switch_set_state(&data->emusdev, NO_DEVICE);

	if (cpcap_usb_det_debug)
		pr_info("cpcap_usb_det: ADC_USB_ID result=0x%X, ret=%d, status=%d, Audio Cable %s Present\n",
			req.result[CPCAP_ADC_USB_ID], ret, req.status, (data->audio ? "IS" : "NOT"));
}

static void notify_accy(struct cpcap_usb_det_data *data, enum cpcap_accy accy)
{
	if (cpcap_usb_det_debug > 1)
		pr_info("cpcap_usb_det %s: accy=%s\n", __func__, accy_names[accy]);

	dev_info(&data->cpcap->spi->dev, "notify_accy: accy=%d\n", accy);

	if ((data->usb_accy != CPCAP_ACCY_NONE) && (data->usb_dev != NULL)) {
		if (!((data->usb_accy == CPCAP_ACCY_USB_DEVICE) && (accy == CPCAP_ACCY_CHARGER))) {
			platform_device_del(data->usb_dev);
			data->usb_dev = NULL;
		}
	}

	configure_hardware(data, accy);
	/* SMART dock needs to load USB Host stack */
	if (accy == CPCAP_ACCY_WHISPER_SMART_DOCK)
		data->usb_accy = CPCAP_ACCY_USB_DEVICE;
	else
		data->usb_accy = accy;

	if (accy != CPCAP_ACCY_NONE) {
		/* SMART dock needs to charge */
		if (!data->usb_dev) {
			if (accy == CPCAP_ACCY_WHISPER_SMART_DOCK)
				data->usb_dev = platform_device_alloc(accy_devices[CPCAP_ACCY_CHARGER], -1);
			else
				data->usb_dev = platform_device_alloc(accy_devices[accy], -1);
			if (data->usb_dev) {
				data->usb_dev->dev.platform_data = data->cpcap;
				platform_device_add(data->usb_dev);
			}
		}
	} else
		vusb_disable(data);

	if ((accy == CPCAP_ACCY_USB) || (accy == CPCAP_ACCY_FACTORY)
		|| (accy == CPCAP_ACCY_USB_DEVICE)
		|| (accy == CPCAP_ACCY_WHISPER_SMART_DOCK)) {
		if (!data->usb_connected_dev) {
			data->usb_connected_dev =
			    platform_device_alloc("cpcap_usb_connected", -1);
			platform_device_add_data(data->usb_connected_dev,
					&data->usb_accy, sizeof(data->usb_accy));
			platform_device_add(data->usb_connected_dev);
		}
	} else if (data->usb_connected_dev) {
		platform_device_del(data->usb_connected_dev);
		data->usb_connected_dev = NULL;
	}

	if (accy == CPCAP_ACCY_CHARGER) {
		if (!data->charger_connected_dev) {
			data->charger_connected_dev =
			    platform_device_alloc("cpcap_charger_connected", -1);
			platform_device_add(data->charger_connected_dev);
		}
	} else if (data->charger_connected_dev) {
			platform_device_del(data->charger_connected_dev);
			data->charger_connected_dev = NULL;
	}
}

static void notify_whisper_switch(struct cpcap_usb_det_data *data, enum cpcap_accy accy)
{
	if (accy == CPCAP_ACCY_CHARGER || accy == CPCAP_ACCY_WHISPER_PPD) {
		/* Set switch for whisper PPDs and Chargers, which are like whisper SPDs */
		switch_set_state(&data->wsdev, 1);
	} else if (accy != CPCAP_ACCY_CHARGER && accy != CPCAP_ACCY_WHISPER_PPD) {
		switch_set_state(&data->wsdev, 0);
		mutex_lock(&switch_access);
		if (!(data->hall_effect_connected)) {
			switch_set_state(&data->dsdev, NO_DOCK);
			switch_set_state(&data->edsdev, EXT_NO_DOCK);
		}
		mutex_unlock(&switch_access);
		switch_set_state(&data->emusdev, NO_DEVICE);
	}
}

static inline short ichrg_bits(struct cpcap_device *cpcap)
{
	unsigned short value = 0;
	unsigned short ICHRG_BITS_MASK = CPCAP_BIT_ICHRG3 |
					 CPCAP_BIT_ICHRG2 |
					 CPCAP_BIT_ICHRG1 |
					 CPCAP_BIT_ICHRG0;
	cpcap_regacc_read(cpcap, CPCAP_REG_CRM, &value);
	return (value&ICHRG_BITS_MASK);
}

static void detection_work(struct work_struct *work)
{
	bool   isVBusValid = false;
	struct cpcap_usb_det_data *data =
		container_of(work, struct cpcap_usb_det_data, work.work);

	if (cpcap_usb_det_debug > 1) {
		pr_info("cpcap_usb_det: in detection_work\n");
		dump_sense_bits(data);
	}

	if (cpcap_usb_det_debug && data->state > SAMPLE_2)
		pr_info("cpcap_usb_det: state %s\n",state_names[data->state]);

	switch (data->state) {
	case CONFIG:
		vusb_enable(data);
		cpcap_irq_mask(data->cpcap, CPCAP_IRQ_CHRG_DET);
		cpcap_irq_mask(data->cpcap, CPCAP_IRQ_CHRG_CURR1);
		cpcap_irq_mask(data->cpcap, CPCAP_IRQ_SE1);
		cpcap_irq_mask(data->cpcap, CPCAP_IRQ_IDGND);
		cpcap_irq_mask(data->cpcap, CPCAP_IRQ_VBUSVLD);
		cpcap_irq_mask(data->cpcap, CPCAP_IRQ_IDFLOAT);
		cpcap_irq_mask(data->cpcap, CPCAP_IRQ_DPI);
		cpcap_irq_mask(data->cpcap, CPCAP_IRQ_DMI);
		cpcap_irq_mask(data->cpcap, CPCAP_IRQ_SESSVLD);

		configure_hardware(data, CPCAP_ACCY_UNKNOWN);
		notify_whisper_switch(data, CPCAP_ACCY_UNKNOWN);

		data->state = SAMPLE_1;
		queue_delayed_work(data->wq, &data->work, msecs_to_jiffies(11));
		break;

	case SAMPLE_1:
		get_sense(data);
		data->state = SAMPLE_2;
		queue_delayed_work(data->wq, &data->work, msecs_to_jiffies(100));
		break;

	case SAMPLE_2:
		data->prev_sense = data->sense;
		get_sense(data);

		if (data->prev_sense != data->sense) {
			/* Stay in this state */
			data->state = SAMPLE_2;
			queue_delayed_work(data->wq, &data->work, msecs_to_jiffies(100));
		} else if (!(data->sense & CPCAP_BIT_SE1_S) &&
			   (data->sense & CPCAP_BIT_ID_FLOAT_S) &&
			   !(data->sense & CPCAP_BIT_ID_GROUND_S) &&
			   !(data->sense & CPCAP_BIT_SESSVLD_S)) {
			/* cable may not be fully inserted: wait a bit more & try again... */
			if (cpcap_usb_det_debug > 1)
				pr_info("cpcap_usb_det: SAMPLE_2 cable may not be fully inserted\n");
			data->state = IDENTIFY;
			queue_delayed_work(data->wq, &data->work, msecs_to_jiffies(100));
		} else {
			/* cable connected: try to identify what was connected... */
			if (cpcap_usb_det_debug > 1)
				pr_info("cpcap_usb_det: cable connected.\n");
			data->state = IDENTIFY;
			queue_delayed_work(data->wq, &data->work, 0);
		}
		break;

	case IDENTIFY:
		get_sense(data);
		data->state = CONFIG;

		if ((data->sense == SENSE_USB) ||
		    (data->sense == SENSE_USB_FLASH)) {
			if (cpcap_usb_det_debug)
				pr_info("cpcap_usb_det: USB or USB_FLASH\n");
			notify_accy(data, CPCAP_ACCY_USB);

			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_DET);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_CURR1);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_SE1);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDGND);

			/* Special handling of USB cable undetect. */
			data->state = USB;
		} else if (data->sense == SENSE_FACTORY ||
			data->sense == SENSE_FACTORY_COM) {
			if (cpcap_usb_det_debug)
				pr_info("cpcap_usb_det: FACTORY Cable\n");
			notify_accy(data, CPCAP_ACCY_FACTORY);

			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_SE1);

			/* Special handling of factory cable undetect. */
			data->state = FACTORY;
		} else if (((data->sense | CPCAP_BIT_VBUSVLD_S) ==
			    SENSE_CHARGER_FLOAT) ||
			   ((data->sense | CPCAP_BIT_VBUSVLD_S) ==
			    SENSE_CHARGER) ||
			   (data->sense == SENSE_WHISPER_SPD)) {
			if (cpcap_usb_det_debug)
				pr_info("cpcap_usb_det: CHARGER or WHISPER_SPD\n");

			notify_accy(data, CPCAP_ACCY_CHARGER);

			/* Power removal from phone/SPD is triggered by CURR1 */
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_CURR1);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_DET);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_VBUSVLD);

			/* Charger removal can also be detected by loss of SE1 while not a Whisper Device */
			/* TODO: need for whisper_auth get/set to be in cs?*/
			if (data->whisper_auth == AUTH_NOT_STARTED || data->whisper_auth == AUTH_FAILED)
				cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_SE1);

			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDGND);

			/* Special handling of CHARGER/SPD undetect. */
			data->state = CHARGER;
			notify_whisper_switch(data, CPCAP_ACCY_CHARGER);
		} else if (data->sense == SENSE_WHISPER_SMART) {

			if (cpcap_usb_det_debug)
				pr_info("cpcap_usb_det: WHISPER_SMART_DOCK\n");

			notify_accy(data, CPCAP_ACCY_WHISPER_SMART_DOCK);

			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDGND);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_CURR1);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_DET);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_VBUSVLD);

			data->state = WHISPER_SMART_DOCK;
			switch_set_state(&data->sdsdev, 1);
		} else if (data->sense == SENSE_WHISPER_PPD ||
				   data->sense == SENSE_WHISPER_PPD_NO_DP) {

			if (cpcap_usb_det_debug)
				pr_info("cpcap_usb_det: WHISPER_PPD\n");

			/* mask VBUSVLD, CHGDET, SESSVLD as Reverse Mode enable may raise these */
			cpcap_irq_mask(data->cpcap, CPCAP_IRQ_VBUSVLD);
			cpcap_irq_mask(data->cpcap, CPCAP_IRQ_CHRG_DET);
			cpcap_irq_mask(data->cpcap, CPCAP_IRQ_SESSVLD);

			notify_accy(data, CPCAP_ACCY_WHISPER_PPD);

			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDFLOAT);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDGND);

			/* Special handling of WHISPER_PPD undetect */
			data->state = WHISPER_PPD;
			notify_whisper_switch(data, CPCAP_ACCY_WHISPER_PPD);
		} else if ((vbus_valid_adc_check(data)) &&
			(data->usb_accy == CPCAP_ACCY_NONE)) {
			/* partially inserted cable - restart detection */
			data->state = CONFIG;
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_DET);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_DPI);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_DMI);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_SE1);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDGND);
		} else {
			if (cpcap_usb_det_debug)
				pr_info("cpcap_usb_det: ACCY_NONE\n");
			notify_accy(data, CPCAP_ACCY_NONE);
			notify_whisper_switch(data, CPCAP_ACCY_NONE);

			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_DET);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_CURR1);

			/* When a charger is unpowered by unplugging from the
			 * wall, VBUS voltage will drop below CHRG_DET (3.5V)
			 * until the ICHRG bits are cleared.  Once ICHRG is
			 * cleared, VBUS will rise above CHRG_DET, but below
			 * VBUSVLD (4.4V) briefly as it decays.  If the charger
			 * is re-powered while VBUS is within this window, the
			 * VBUSVLD interrupt is needed to trigger charger
			 * detection.
			 *
			 * VBUSVLD must be masked before going into suspend.
			 * See cpcap_usb_det_suspend() for details.
			 */
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_VBUSVLD);

			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDFLOAT);
		}
		break;

	case USB:
		get_sense(data);

		if ((data->sense & CPCAP_BIT_SE1_S) ||
		    (data->sense & CPCAP_BIT_ID_GROUND_S) ||
		    (!(vbus_valid_adc_check(data)))) {
			data->state = CONFIG;
			queue_delayed_work(data->wq, &data->work, 0);
		} else {
			data->state = USB;

			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_DET);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_CURR1);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_SE1);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDGND);
		}
		break;

	case FACTORY:
		get_sense(data);

		/* The removal of a factory cable can only be detected if a
		 * charger is attached.
		 */
		if (data->sense & CPCAP_BIT_SE1_S) {
#ifdef CONFIG_TTA_CHARGER
			enable_tta();
#endif
			data->state = CONFIG;
			queue_delayed_work(data->wq, &data->work, 0);
		} else {
			data->state = FACTORY;
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_SE1);
		}
		break;

	case CHARGER:
	case WHISPER_SPD:
		get_sense(data);
		isVBusValid = vbus_valid_adc_check(data);

		/* If the battery voltage is above the set charge voltage VCHRG[3:0] in CPCAP & ICHRG
		 * is set, CHRGCURR1 will be 0.  Do not undetect charger in this case. */
		if (!isVBusValid && !(data->sense & CPCAP_BIT_ID_FLOAT_S) &&
		     (data->whisper_auth == AUTH_PASSED)) {
			if (!(data->sense & CPCAP_BIT_SESSVLD_S)) {
			if (cpcap_usb_det_debug)
				pr_info("cpcap_usb_det: WHISPER_SPD power removed\n");

			/* mask everything while we enable PPD configuration */
			cpcap_irq_mask(data->cpcap, CPCAP_IRQ_CHRG_DET);
			cpcap_irq_mask(data->cpcap, CPCAP_IRQ_CHRG_CURR1);
			cpcap_irq_mask(data->cpcap, CPCAP_IRQ_SE1);
			cpcap_irq_mask(data->cpcap, CPCAP_IRQ_IDGND);
			cpcap_irq_mask(data->cpcap, CPCAP_IRQ_VBUSVLD);
			cpcap_irq_mask(data->cpcap, CPCAP_IRQ_IDFLOAT);
			cpcap_irq_mask(data->cpcap, CPCAP_IRQ_SESSVLD);

			/* config for PPD (VBUS power on), notify for PPD and set state to PPD */
			notify_accy(data, CPCAP_ACCY_WHISPER_PPD);

			data->state = WHISPER_PPD;

			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDFLOAT);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDGND);
			} else {
				if (cpcap_usb_det_debug)
					pr_info("cpcap_usb_det: Waiting for VBUS to drain before switching on RVRS CHRG\n");
			}
		} else if (!isVBusValid) {
			if (cpcap_usb_det_debug)
				pr_info("cpcap_usb_det: CHARGER or WHISPER_SPD detached [whisper_auth=%d]\n",
						(data->whisper_auth));

			cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC2, CPCAP_BIT_USBSUSPEND,
							 CPCAP_BIT_UARTMUX1 | CPCAP_BIT_UARTMUX0 |
							 CPCAP_BIT_EMUMODE2 |
							 CPCAP_BIT_EMUMODE1 |
							 CPCAP_BIT_EMUMODE0 |
							 CPCAP_BIT_USBSUSPEND);
			data->state = CONFIG;
			queue_delayed_work(data->wq, &data->work, 0);
		} else {
			if (!(data->sense & CPCAP_BIT_ID_GROUND_S) && data->irq == CPCAP_IRQ_IDGND
					&& data->whisper_auth == AUTH_PASSED) {
				if (cpcap_usb_det_debug > 1)
					pr_info("cpcap_usb_det: WHISPER_SPD checking for audio cable state\n");
				whisper_audio_check(data);
			} else {
				if (cpcap_usb_det_debug && (data->whisper_auth != AUTH_PASSED))
					pr_info("cpcap_usb_det: WHISPER_SPD not authenticated, ignoring change\n");
			}

			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_CURR1);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_DET);

			if ((data->whisper_auth == AUTH_PASSED || data->whisper_auth == AUTH_IN_PROGRESS) &&
			    isVBusValid && !(data->sense & CPCAP_BIT_CHRGCURR1_S)) {
				if (cpcap_usb_det_debug > 1)
					pr_info("cpcap_usb_det: WHISPER_SPD Unmasking VBUSVLD Interrupt\n");
				cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_VBUSVLD);
			} else if (isVBusValid && (data->sense & CPCAP_BIT_CHRGCURR1_S)) {
				if (cpcap_usb_det_debug > 1)
					pr_info("cpcap_usb_det: WHISPER_SPD Msking VBUSVLD Interrupt\n");
				cpcap_irq_mask(data->cpcap, CPCAP_IRQ_VBUSVLD);
			}

			/* Charger removal can also be detected by loss of SE1 while not a Whisper Device */
			/* TODO: need for whisper_auth get/set to be in cs?*/
			if (data->whisper_auth == AUTH_NOT_STARTED || data->whisper_auth == AUTH_FAILED)
				cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_SE1);
			else
				cpcap_irq_mask(data->cpcap, CPCAP_IRQ_SE1);

			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDGND);
		}
		break;

	case WHISPER_PPD:
		get_sense(data);
		if (data->sense & CPCAP_BIT_ID_FLOAT_S) {
			if (cpcap_usb_det_debug)
				pr_info("cpcap_usb_det: WHISPER_PPD detached\n");
			data->state = CONFIG;
			queue_delayed_work(data->wq, &data->work, 0);
		} else if (!(data->sense & CPCAP_BIT_ID_GROUND_S) && (vbus_valid_adc_check(data)) &&
				(data->sense & CPCAP_BIT_SESSVLD_S)) {
			if (cpcap_usb_det_debug)
				pr_info("cpcap_usb_det: WHISPER_PPD power connected\n");

			/* config for CHRGR/SPD, notify for CHRGR/SPD, set state to SPD */
			/* mask everything while we enable SPD configuration */
			cpcap_irq_mask(data->cpcap, CPCAP_IRQ_CHRG_DET);
			cpcap_irq_mask(data->cpcap, CPCAP_IRQ_CHRG_CURR1);
			cpcap_irq_mask(data->cpcap, CPCAP_IRQ_SE1);
			cpcap_irq_mask(data->cpcap, CPCAP_IRQ_IDGND);
			cpcap_irq_mask(data->cpcap, CPCAP_IRQ_VBUSVLD);
			cpcap_irq_mask(data->cpcap, CPCAP_IRQ_IDFLOAT);

			/* config for CHRGR/SPD(RVRS_CHRG off), notify for CHRGR/SPD, set state to SPD */
			notify_accy(data, CPCAP_ACCY_CHARGER);
			data->state = CHARGER;
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_CURR1);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDGND);
		} else {
			if (data->whisper_auth == AUTH_FAILED && data->usb_accy == CPCAP_ACCY_WHISPER_PPD) {
				notify_accy(data, CPCAP_ACCY_NONE); /*disable reverse charge if not whisper*/
			}

			if (!(data->sense & CPCAP_BIT_ID_GROUND_S) && data->irq == CPCAP_IRQ_IDGND
				&& data->whisper_auth == AUTH_PASSED) {
				if (cpcap_usb_det_debug > 1)
					pr_info("cpcap_usb_det: WHISPER_PPD checking for audio cable state\n");
				whisper_audio_check(data);
			} else {
				if (cpcap_usb_det_debug && (data->whisper_auth != AUTH_PASSED))
					pr_info("cpcap_usb_det: WHISPER_PPD not authenticated, ignoring change\n");
			}
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDFLOAT);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDGND);
		}
		break;

	case WHISPER_SMART_DOCK:
		get_sense(data);
		isVBusValid = vbus_valid_adc_check(data);

		if (data->whisper_auth == AUTH_FAILED && data->usb_accy == CPCAP_ACCY_USB_DEVICE
				&& isVBusValid) {
			notify_accy(data, CPCAP_ACCY_CHARGER); /*can only charge when auth failed*/
		}

		if (!(data->sense & CPCAP_BIT_ID_GROUND_S) && isVBusValid && data->irq == CPCAP_IRQ_IDGND) {
			if (cpcap_usb_det_debug)
				pr_info("cpcap_usb_det: WHISPER_SMART_DOCK ID pin released for SPDIF \n");
			data->state = WHISPER_SMART_DOCK;
			cpcap_irq_mask(data->cpcap, CPCAP_IRQ_IDGND);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_CURR1);
		} else if (!isVBusValid) {
			if (cpcap_usb_det_debug)
				pr_info("cpcap_usb_det: WHISPER_SMART_DOCK Removed\n");
			data->state = CONFIG;
			switch_set_state(&data->sdsdev, 0);
			queue_delayed_work(data->wq, &data->work, 0);
		} else {
			data->state = WHISPER_SMART_DOCK;
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_CURR1);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_DET);

			if (isVBusValid && !(data->sense & CPCAP_BIT_CHRGCURR1_S)) {
				if (cpcap_usb_det_debug > 1)
					pr_info("cpcap_usb_det: WHISPER_SMART Unmasking VBUSVLD Interrupt\n");
				cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_VBUSVLD);
			} else if (isVBusValid && (data->sense & CPCAP_BIT_CHRGCURR1_S)) {
				if (cpcap_usb_det_debug > 1)
					pr_info("cpcap_usb_det: WHISPER_SMART Msking VBUSVLD Interrupt\n");
				cpcap_irq_mask(data->cpcap, CPCAP_IRQ_VBUSVLD);
			}
		}
		break;

	default:
		/* This shouldn't happen.  Need to reset state machine. */
		vusb_disable(data);
		data->state = CONFIG;
		queue_delayed_work(data->wq, &data->work, 0);
		break;
	}
}

static void int_handler(enum cpcap_irqs int_event, void *data)
{
	struct cpcap_usb_det_data *usb_det_data = data;
	usb_det_data->irq = int_event;
	if (cpcap_usb_det_debug) {
		if ((int_event >= 0 && int_event <= 22))
			pr_info("cpcap_usb_det: irq=%s\n", irq_names[int_event]);
		else
			pr_info("cpcap_usb_det: irq=%d\n", int_event);
	}
	queue_delayed_work(usb_det_data->wq, &(usb_det_data->work), 0);
}

int cpcap_accy_whisper(struct cpcap_device *cpcap, unsigned long cmd)
{
	struct cpcap_usb_det_data *data = cpcap->accydata;
	int retval = -EAGAIN;
	unsigned short value;
	int dock_type = NO_DOCK;

	if (cpcap_usb_det_debug)
	pr_info("cpcap_usb_det %s: ioctl cmd = 0x%08lx\n", __func__, cmd);

	if (!data)
		return -ENODEV;

	/* Can only change settings if not debouncing and is whisper device */
	if (data->state == CHARGER || data->state == WHISPER_PPD
				|| data->state == WHISPER_SMART_DOCK) {
		if (cmd & CPCAP_WHISPER_ENABLE_UART) {
			if (data->state != WHISPER_SMART_DOCK) {
				cpcap_irq_mask(data->cpcap, CPCAP_IRQ_SE1);
				data->whisper_auth = AUTH_IN_PROGRESS;
				retval = cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC2,
					((data->uartmux << 8) |
					CPCAP_BIT_EMUMODE0),
					CPCAP_BIT_UARTMUX1 | CPCAP_BIT_UARTMUX0 |
					CPCAP_BIT_EMUMODE2 | CPCAP_BIT_EMUMODE1 |
					CPCAP_BIT_EMUMODE0 | CPCAP_BIT_USBSUSPEND);
				if (cpcap_usb_det_debug)
					pr_info("cpcap_usb_det: UART Mode enabled, retval = %d\n", retval);
			} else
				retval = -ENOTTY; /* uart_enable only supported for basic docks */

		} else {
			value = (cmd & CPCAP_WHISPER_MODE_PU) ? CPCAP_BIT_ID100KPU : 0;
			if (value != 0) {
				cpcap_irq_mask(data->cpcap, CPCAP_IRQ_SE1);
				dock_type = ((cmd >> 27)%MAX_DOCK);
				cpcap_accy_set_dock_switch(data->cpcap, dock_type, false);
				data->whisper_auth = AUTH_PASSED;
				if (data->state != WHISPER_SMART_DOCK) {
					tegra_gpio_enable(data->cpcap->spdif_gpio);
					retval = cpcap_regacc_write(cpcap, CPCAP_REG_USBC2, CPCAP_BIT_USBSUSPEND,
						 CPCAP_BIT_UARTMUX1 | CPCAP_BIT_UARTMUX0 |
						 CPCAP_BIT_EMUMODE2 |
						 CPCAP_BIT_EMUMODE1 |
						 CPCAP_BIT_EMUMODE0 |
						 CPCAP_BIT_USBSUSPEND);
					if (cpcap_usb_det_debug)
						pr_info("cpcap_usb_det: UART Mode disabled, retval=%d\n", retval);
					whisper_audio_check(data);
					retval = cpcap_regacc_write(cpcap, CPCAP_REG_USBC1,
						value, CPCAP_BIT_ID100KPU);
				} else if (data->state == WHISPER_SMART_DOCK) {
					tegra_gpio_disable(data->cpcap->spdif_gpio);
					retval = 0;
				}
				data->whisper_auth = AUTH_PASSED;

				if (data->state == CHARGER || data->state == WHISPER_SMART_DOCK)
					cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_VBUSVLD);

			} else {
				/* HE docks will fail authentication so don't clear
				 * the switches when one is connected */
				if (!data->hall_effect_connected) {
					switch_set_state(&data->dsdev, NO_DOCK);
					switch_set_state(&data->edsdev, EXT_NO_DOCK);
				}
				data->whisper_auth = AUTH_FAILED;
				if (data->state != WHISPER_SMART_DOCK) {
					retval = cpcap_regacc_write(cpcap, CPCAP_REG_USBC2, CPCAP_BIT_USBSUSPEND,
						 CPCAP_BIT_UARTMUX1 | CPCAP_BIT_UARTMUX0 |
						 CPCAP_BIT_EMUMODE2 |
						 CPCAP_BIT_EMUMODE1 |
						 CPCAP_BIT_EMUMODE0 |
						 CPCAP_BIT_USBSUSPEND);
					if (cpcap_usb_det_debug)
						pr_info("cpcap_usb_det: revert back to USB Mode, retval=%d, whisper_auth=%d\n",
							retval, data->whisper_auth);
					if (data->state == WHISPER_PPD)
						queue_delayed_work(data->wq, &data->work, msecs_to_jiffies(11));
				} else {
					queue_delayed_work(data->wq, &data->work, msecs_to_jiffies(11));
					retval = 0;
				}
				if (data->state == CHARGER) {
					/* clear old SE1 interrupt bits before unmasking */
					cpcap_regacc_write(data->cpcap, CPCAP_REG_INT2, CPCAP_BIT_SE1_I, CPCAP_BIT_SE1_I);
					cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_SE1);
				}
			}
		}
	}
	return retval;
}

static int cpcap_usb_det_probe(struct platform_device *pdev)
{
	int retval;
	struct cpcap_usb_det_data *data;

	if (pdev->dev.platform_data == NULL) {
		dev_err(&pdev->dev, "no platform_data\n");
		return -EINVAL;
	}

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->cpcap = pdev->dev.platform_data;
	data->state = CONFIG;
	data->wq = create_singlethread_workqueue("cpcap_accy");
	INIT_DELAYED_WORK(&data->work, detection_work);
	data->usb_accy = CPCAP_ACCY_NONE;
	wake_lock_init(&data->wake_lock, WAKE_LOCK_SUSPEND, "usb");
	data->wsdev.name = "whisper";
	data->sdsdev.name = "smartdock";
	data->emusdev.name = "emuconn";
	data->emusdev.print_name = emu_audio_print_name;
	data->dsdev.name = "dock";
	data->edsdev.name = "extdock";
	data->dsdev.print_name = dock_print_name;
	data->edsdev.print_name = ext_dock_print_name;
	switch_dev_register(&data->wsdev);
	switch_dev_register(&data->sdsdev);
	switch_dev_register(&data->emusdev);
	switch_dev_register(&data->dsdev);
	switch_dev_register(&data->edsdev);
	data->uartmux = 1;
	data->whisper_auth = AUTH_NOT_STARTED;
	data->hall_effect_connected = false;
	data->irq = CPCAP_IRQ__START;

	platform_set_drvdata(pdev, data);

	data->regulator = regulator_get(&pdev->dev, "vusb");
	if (IS_ERR(data->regulator)) {
		dev_err(&pdev->dev, "Could not get regulator for cpcap_usb\n");
		retval = PTR_ERR(data->regulator);
		goto free_mem;
	}
	regulator_set_voltage(data->regulator, 3300000, 3300000);

	/* Clear the interrupts so they are in a known state when starting detection. */
	retval = cpcap_irq_clear(data->cpcap, CPCAP_IRQ_CHRG_DET);
	retval |= cpcap_irq_clear(data->cpcap, CPCAP_IRQ_CHRG_CURR1);
	retval |= cpcap_irq_clear(data->cpcap, CPCAP_IRQ_SE1);
	retval |= cpcap_irq_clear(data->cpcap, CPCAP_IRQ_IDGND);
	retval |= cpcap_irq_clear(data->cpcap, CPCAP_IRQ_VBUSVLD);
	retval |= cpcap_irq_clear(data->cpcap, CPCAP_IRQ_IDFLOAT);
	retval |= cpcap_irq_clear(data->cpcap, CPCAP_IRQ_DPI);
	retval |= cpcap_irq_clear(data->cpcap, CPCAP_IRQ_DMI);
	retval |= cpcap_irq_clear(data->cpcap, CPCAP_IRQ_SESSVLD);

	/* Register the interrupt handler, please be aware this will enable the
	   interrupts. */
	retval |= cpcap_irq_register(data->cpcap, CPCAP_IRQ_CHRG_DET,
				    int_handler, data);
	retval |= cpcap_irq_register(data->cpcap, CPCAP_IRQ_CHRG_CURR1,
				     int_handler, data);
	retval |= cpcap_irq_register(data->cpcap, CPCAP_IRQ_SE1,
				     int_handler, data);
	retval |= cpcap_irq_register(data->cpcap, CPCAP_IRQ_IDGND,
				     int_handler, data);
	retval |= cpcap_irq_register(data->cpcap, CPCAP_IRQ_VBUSVLD,
				     int_handler, data);
	retval |= cpcap_irq_register(data->cpcap, CPCAP_IRQ_IDFLOAT,
				     int_handler, data);
	retval |= cpcap_irq_register(data->cpcap, CPCAP_IRQ_DPI,
				     int_handler, data);
	retval |= cpcap_irq_register(data->cpcap, CPCAP_IRQ_DMI,
				     int_handler, data);
	retval |= cpcap_irq_register(data->cpcap, CPCAP_IRQ_SESSVLD,
				     int_handler, data);

	if (retval != 0) {
		dev_err(&pdev->dev, "Initialization Error\n");
		retval = -ENODEV;
		goto free_irqs;
	}

#ifdef CONFIG_USB_CPCAP_OTG
	data->otg = otg_get_transceiver();
#endif
	data->cpcap->accydata = data;
	dev_info(&pdev->dev, "CPCAP USB detection device probed\n");

	/* Schedule initial detection.  This is done in case an interrupt has
	   already scheduled the work, to keep it from running more than once. */
	queue_delayed_work(data->wq, &(data->work), 0);

	return 0;

free_irqs:
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_VBUSVLD);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_IDGND);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_SE1);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_CHRG_CURR1);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_CHRG_DET);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_IDFLOAT);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_DPI);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_DMI);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_SESSVLD);

	regulator_put(data->regulator);
free_mem:
	switch_dev_unregister(&data->wsdev);
	switch_dev_unregister(&data->sdsdev);
	switch_dev_unregister(&data->emusdev);
	switch_dev_unregister(&data->dsdev);
	switch_dev_unregister(&data->edsdev);
	wake_lock_destroy(&data->wake_lock);
	kfree(data);

	return retval;
}

static int __exit cpcap_usb_det_remove(struct platform_device *pdev)
{
	struct cpcap_usb_det_data *data = platform_get_drvdata(pdev);
	if (cpcap_usb_det_debug > 1)
		pr_info("cpcap_usb_det %s:\n", __func__);

	cpcap_irq_free(data->cpcap, CPCAP_IRQ_CHRG_DET);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_CHRG_CURR1);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_SE1);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_IDGND);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_VBUSVLD);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_IDFLOAT);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_DPI);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_DMI);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_SESSVLD);

	configure_hardware(data, CPCAP_ACCY_NONE);
	cancel_delayed_work_sync(&data->work);
	destroy_workqueue(data->wq);

	if ((data->usb_accy != CPCAP_ACCY_NONE) && (data->usb_dev != NULL))
		platform_device_del(data->usb_dev);

	switch_dev_unregister(&data->wsdev);
	switch_dev_unregister(&data->sdsdev);
	switch_dev_unregister(&data->emusdev);
	switch_dev_unregister(&data->dsdev);
	switch_dev_unregister(&data->edsdev);

	gpio_set_value(data->pdata->data_gpio, 1);

	vusb_disable(data);
	regulator_put(data->regulator);

#ifdef CONFIG_USB_CPCAP_OTG
	if (data->otg)
		otg_put_transceiver(data->otg);
#endif

	wake_lock_destroy(&data->wake_lock);

	kfree(data);
	return 0;
}

#ifdef CONFIG_PM
static int cpcap_usb_det_suspend(struct platform_device *pdev,
				 pm_message_t state)
{
	struct cpcap_usb_det_data *data = platform_get_drvdata(pdev);
	if (cpcap_usb_det_debug > 1)
		pr_info("cpcap_usb_det %s:\n", __func__);

	/* VBUSVLD cannot be unmasked when entering suspend. If left
	 * unmasked, a false interrupt will be received, keeping the
	 * device out of suspend. The interrupt does not need to be
	 * unmasked when resuming from suspend since the use case
	 * for having the interrupt unmasked is over.
	 */
	cpcap_irq_mask(data->cpcap, CPCAP_IRQ_VBUSVLD);

	return 0;
}
#else
#define cpcap_usb_det_suspend NULL
#endif

static struct platform_driver cpcap_usb_det_driver = {
	.probe		= cpcap_usb_det_probe,
	.remove		= __exit_p(cpcap_usb_det_remove),
	.suspend	= cpcap_usb_det_suspend,
	.driver		= {
		.name	= "cpcap_usb_det",
		.owner	= THIS_MODULE,
	},
};

static int __init cpcap_usb_det_init(void)
{
	return platform_driver_register(&cpcap_usb_det_driver);
}
/* The CPCAP USB detection driver must be started later to give the MUSB
 * driver time to complete its initialization. */
late_initcall(cpcap_usb_det_init);

static void __exit cpcap_usb_det_exit(void)
{
	platform_driver_unregister(&cpcap_usb_det_driver);
}
module_exit(cpcap_usb_det_exit);

MODULE_ALIAS("platform:cpcap_usb_det");
MODULE_DESCRIPTION("CPCAP USB detection driver");
MODULE_LICENSE("GPL");
