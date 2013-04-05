/*
 * Copyright (C) 2010 HTC, Inc.
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
#ifndef __ASM_ARCH_OLYMPUS_OLYMPUS_USB_H
#define __ASM_ARCH_OLYMPUS_OLYMPUS_USB_H

#ifdef ERROR
#undef ERROR
#endif
#include <linux/usb/android_composite.h>
#include <linux/usb/f_accessory.h>

static char *usb_functions_mtp_un[] = {
		"mtp",
		"usbnet",
};

static char *usb_functions_mtp_un_adb[] = {
		"mtp",
		"usbnet",
		"adb",
};

static char *usb_functions_un[] = {
		"usbnet",
};

static char *usb_functions_un_adb[] = {
		"usbnet",
		"adb",
};

static char *usb_functions_ptp[] = {
		"ptp",
};

static char *usb_functions_ptp_adb[] = {
		"ptp",
		"adb",
};

static char *usb_functions_rn[] = {
	"rndis",
};

static char *usb_functions_rn_adb[] = {
	"rndis",
	"adb",
};

static char *usb_functions_cd[] = {
	"cdrom",
};

static char *usb_functions_ms[] = {
	"mass_storage",
};

static char *usb_functions_ms_adb[] = {
	"mass_storage",
	"adb",
};

static char *usb_functions_cd2[] = {
	"cdrom2",
};

static char *usb_functions_all[] = {
	"mtp",
	"usbnet",
	"ptp",
	"rndis",
	"mass_storage",
	"adb",
	"cdrom",
	"cdrom2",
};

static struct android_usb_product usb_products[] = {
	{
		.product_id     = 0x4361,
		.num_functions  = ARRAY_SIZE(usb_functions_mtp_un),
		.functions      = usb_functions_mtp_un,
	},
	{
		.product_id     = 0x4362,
		.num_functions  = ARRAY_SIZE(usb_functions_mtp_un_adb),
		.functions      = usb_functions_mtp_un_adb,
	},
	{
		.product_id     = 0x7083,
		.num_functions  = ARRAY_SIZE(usb_functions_un),
		.functions      = usb_functions_un,
	},
	{
		.product_id     = 0x7082,
		.num_functions  = ARRAY_SIZE(usb_functions_un_adb),
		.functions      = usb_functions_un_adb,
	},
	{
		.product_id		= 0x4367,
		.num_functions	= ARRAY_SIZE(usb_functions_ptp),
		.functions		= usb_functions_ptp,
	},
	{
		.product_id		= 0x4368,
		.num_functions	= ARRAY_SIZE(usb_functions_ptp_adb),
		.functions		= usb_functions_ptp_adb,
	},
	{
		.product_id		= 0x7091,
		.num_functions	= ARRAY_SIZE(usb_functions_rn),
		.functions		= usb_functions_rn,
	},
	{
		.product_id		= 0x7092,
		.num_functions	= ARRAY_SIZE(usb_functions_rn_adb),
		.functions		= usb_functions_rn_adb,
	},
	{
		.product_id		= 0x4340,
		.num_functions	= ARRAY_SIZE(usb_functions_cd),
		.functions		= usb_functions_cd,
	},
	{
		.product_id		= 0x4365,
		.num_functions	= ARRAY_SIZE(usb_functions_ms),
		.functions		= usb_functions_ms,
	},
	{
		.product_id		= 0x4366,
		.num_functions	= ARRAY_SIZE(usb_functions_ms_adb),
		.functions		= usb_functions_ms_adb,
	},
	{
		.product_id		= 0x41ce,
		.num_functions	= ARRAY_SIZE(usb_functions_cd2),
		.functions		= usb_functions_cd2,
	},
};

#endif

