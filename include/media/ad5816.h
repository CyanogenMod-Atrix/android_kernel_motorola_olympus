/*
 * Copyright (C) 2011-2012 NVIDIA Corporation.
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

#ifndef __AD5816_H__
#define __AD5816_H__

#include <media/nvc_focus.h>
#include <media/nvc.h>

typedef enum {
	AD5816_VREG_VDD = 0,
	AD5816_VREG_VDD_AF,
	AD5816_VREG_VDD_I2C
} ad5816_vreg;

typedef enum {
	AD5816_GPIO_RESET = 0,
	AD5816_GPIO_I2CMUX,
	AD5816_GPIO_GP1,
	AD5816_GPIO_GP2,
	AD5816_GPIO_GP3
} ad5816_gpio_types;


struct ad5816_platform_data {
	int cfg;
	int num;
	int sync;
	const char *dev_name;
	struct nvc_focus_nvc (*nvc);
	struct nvc_focus_cap (*cap);
	struct ad5816_pdata_info (*info);
	int gpio_count;
	struct nvc_gpio_pdata *gpio;
};

struct ad5816_pdata_info {
	float focal_length;
	float fnumber;
	__u32 settle_time;
	__s16 pos_low;
	__s16 pos_high;
	__s16 limit_low;
	__s16 limit_high;
	int move_timeoutms;
	__u32 focus_hyper_ratio;
	__u32 focus_hyper_div;
};

// Register Definitions

#define IC_INFO			0x00
#define IC_VERSION		0x01
#define CONTROL			0x02
#define VCM_CODE_MSB	0x03
#define VCM_CODE_LSB	0x04
#define STATUS			0x05
#define MODE			0x06
#define VCM_FREQ		0x07
#define VCM_THRESHOLD	0x08


#endif
/* __AD5816_H__ */
