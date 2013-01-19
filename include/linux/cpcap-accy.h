/*
 *  * Copyright (C) 2010 Motorola, Inc.
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

#ifndef __CPCAP_ACCY_H__
#define __CPCAP_ACCY_H__

enum cpcap_accy {
	CPCAP_ACCY_USB,		/* USB Host connected */
	CPCAP_ACCY_FACTORY,
	CPCAP_ACCY_CHARGER,
	CPCAP_ACCY_USB_DEVICE,  /* USB Device connected */
	CPCAP_ACCY_WHISPER_PPD,
	CPCAP_ACCY_WHISPER_SPD,
	CPCAP_ACCY_WHISPER_SMART_DOCK,
	CPCAP_ACCY_NONE,

	/* Used while debouncing the accessory. */
	CPCAP_ACCY_UNKNOWN,
};

struct cpcap_accy_platform_data {
	enum cpcap_accy accy;
};

#endif  /* __CPCAP_ACCY_H__ */
