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

static char *usb_functions_ums[] = {
	"usb_mass_storage",
};

static char *usb_functions_ums_adb[] = {
	"usb_mass_storage",
	"adb",
};

static char *usb_functions_all[] = {
	"acm",
	"usbnet",
	"mtp",
	"rndis",
	"usb_mass_storage",
	"adb",
};

static char *usb_functions_phone_portal[] = {
	"acm",
	"usbnet",
	"mtp",
};

static char *usb_functions_phone_portal_adb[] = {
	"acm",
	"usbnet",
	"mtp",
	"adb",
};

static char *usb_functions_mtp[] = {
	"mtp",
};

static char *usb_functions_mtp_adb[] = {
	"mtp",
	"adb",
};

static char *usb_functions_rndis[] = {
	"rndis",
};

static char *usb_functions_rndis_adb[] = {
	"rndis",
	"adb",
};

static char *bp_usb_functions_bp[] = { "acm", "usbnet" };
static char *bp_usb_functions_bp_adb[] = { "acm", "usbnet", "adb" };
static char *bp_usb_functions_rndis_bp[] = { "rndis", "acm", "usbnet" };
static char *bp_usb_functions_all[] = {
	"rndis",
	"acm",
	"usbnet",
	"adb"
};

static struct android_usb_product usb_products[] = {
	{
		.product_id     = 0x708a,
		.num_functions  = ARRAY_SIZE(usb_functions_phone_portal),
		.functions      = usb_functions_phone_portal,
	},
	{
		.product_id     = 0x708b,
		.num_functions  = ARRAY_SIZE(usb_functions_phone_portal_adb),
		.functions      = usb_functions_phone_portal_adb,
	},
	{
		.product_id     = 0x7088,
		.num_functions  = ARRAY_SIZE(usb_functions_mtp),
		.functions      = usb_functions_mtp,
	},
	{
		.product_id     = 0x7089,
		.num_functions  = ARRAY_SIZE(usb_functions_mtp_adb),
		.functions      = usb_functions_mtp_adb,
	},
	{
		.product_id	= 0x7086,
		.num_functions	= ARRAY_SIZE(usb_functions_ums),
		.functions	= usb_functions_ums,
	},
	{
		.product_id	= 0x7087,
		.num_functions	= ARRAY_SIZE(usb_functions_ums_adb),
		.functions	= usb_functions_ums_adb,
	},
	{
		.product_id	= 0x7091,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis),
		.functions	= usb_functions_rndis,
	},
	{
		.product_id	= 0x7092,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_adb),
		.functions	= usb_functions_rndis_adb,
	},
};

static struct android_usb_product bp_usb_products[] = {
	{
		.product_id     = 0x7093,
		.num_functions  = ARRAY_SIZE(bp_usb_functions_bp),
		.functions      = bp_usb_functions_bp,
	},
	{
		.product_id     = 0x7094,
		.num_functions  = ARRAY_SIZE(bp_usb_functions_bp_adb),
		.functions      = bp_usb_functions_bp_adb,
	},
	{
		.product_id     = 0x7095,
		.num_functions  = ARRAY_SIZE(bp_usb_functions_rndis_bp),
		.functions      = bp_usb_functions_rndis_bp,
	},
	{
		.product_id     = 0x7096,
		.num_functions  = ARRAY_SIZE(bp_usb_functions_all),
		.functions      = bp_usb_functions_all,
	},
};

#endif

