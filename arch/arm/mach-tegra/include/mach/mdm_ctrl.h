/*
 * Copyright (C) 2009 Motorola, Inc.
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

#ifndef _LINUX_MACH_MDM_CTRL_H__
#define _LINUX_MACH_MDM_CTRL_H__

#define MDM_CTRL_MODULE_NAME "mdm_ctrl"

/* The number of bytes needed to return interrupt fire status.  First byte
 * contains the GPIO mask and the second byte the BP status */
#define MDM_GPIO_BYTE_COUNT		2
/* The number of GPIOs that can experience interrupts */
#define MDM_GPIO_INTERRUPT_COUNT	4

#define MDM_GPIO_INVALID		~0

struct mdm_ctrl_platform_data {
	unsigned int ap_status0_gpio;
	unsigned int ap_status1_gpio;
	unsigned int ap_status2_gpio;
	unsigned int bp_status0_gpio;
	unsigned int bp_status0_gpio_irq_type;
	unsigned int bp_status1_gpio;
	unsigned int bp_status1_gpio_irq_type;
	unsigned int bp_status2_gpio;
	unsigned int bp_status2_gpio_irq_type;
	unsigned int bp_pshold_gpio;
	unsigned int bp_resin_gpio;
	unsigned int bp_resout_gpio;
	unsigned int bp_resout_gpio_irq_type;
	unsigned int bp_bypass_gpio;
	unsigned int bp_pwron_gpio;
	unsigned int bp_flash_en1_gpio;
	unsigned int bp_flash_en2_gpio;
	const char *usb_regulator;
	bool bp_resout_quirk;
	void (*on_bp_startup)(void);
	void (*on_bp_shutdown)(void);
	void (*on_bp_change)(int, int);
};

typedef int (*mdm_ctrl_peer_register_t)(void (*peer_startup)(void*),
	                                void (*peer_shutdown)(void*),
	                                void* context);


int mdm_ctrl_force_shutdown(void);


struct mdm6600_agent_platform_data {
	int (*mdm_ctrl_agent_register)(void (*change)(int, int));
};

#endif  /* _LINUX_MACH_MDM_CTRL_H__ */

