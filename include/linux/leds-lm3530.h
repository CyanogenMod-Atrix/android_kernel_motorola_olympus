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

#ifndef __LEDS_LM3530_H__
#define __LEDS_LM3530_H__

#include <linux/ioctl.h>

#define LM3530_NAME            "lm3530"
#define LM3530_LED_NAME        "lcd-backlight"
#define LM3530_LED_NAME_NORAMP "lcd-backlight-nr"
#define LM3530_I2C_ADDR        0x38

/*****************************************************************************
 *  LM3530 registers
 *****************************************************************************/
#define LM3530_GEN_CONFIG_REG       0x10
#define LM3530_ALS_CONFIG_REG       0x20
#define LM3530_RAMP_REG             0x30
#define LM3530_ALS_ZONE_REG		    0x40
#define LM3530_ALS_RESISTOR_SELECT	0x41
#define LM3530_BRIGHTNESS_CTRL_REG	0xA0
#define LM3530_ALS_ZB0_REG		    0x60
#define LM3530_ALS_ZB1_REG		    0x61
#define LM3530_ALS_ZB2_REG		    0x62
#define LM3530_ALS_ZB3_REG		    0x63
#define LM3530_ALS_Z0T_REG		    0x70
#define LM3530_ALS_Z1T_REG		    0x71
#define LM3530_ALS_Z2T_REG		    0x72
#define LM3530_ALS_Z3T_REG	    	0x73
#define LM3530_ALS_Z4T_REG    		0x74
#define LM3530_VERSION_REG          0xCC

/* Gen Config values */
#define LM3530_PWM_ACTIVE_LOW   0x40
#define LM3530_PWM_ENABLE       0x20
#define LM3530_29mA_FS_CURRENT  0x1C
#define LM3530_26mA_FS_CURRENT  0x18
#define LM3530_22mA_FS_CURRENT  0x14
#define LM3530_19mA_FS_CURRENT  0x10
#define LM3530_15mA_FS_CURRENT  0x0C
#define LM3530_12mA_FS_CURRENT  0x08
#define LM3530_8mA_FS_CURRENT   0x04
#define LM3530_LINEAR_MAPPING   0x02
#define LM3530_I2C_ENABLE       0x01

/* ALS Config values */
#define LM3530_ALS_ENABLED      0x08
#define LM3530_ALS_ENABLED_AUTO 0x18
#define LM3530_ALS_AVG_16ms     0x00
#define LM3530_ALS_AVG_32ms     0x01
#define LM3530_ALS_AVG_64ms     0x02
#define LM3530_ALS_AVG_128ms    0x03
#define LM3530_ALS_AVG_256ms    0x04
#define LM3530_ALS_AVG_512ms    0x05
#define LM3530_ALS_AVG_1024ms   0x06
#define LM3530_ALS_AVG_2048ms   0x07

#ifdef __KERNEL__
struct lm3530_platform_data {
    int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);

    unsigned ramp_time;      /* Ramp time */

    u8 gen_config;
    u8 als_config;
	u8 zone_boundary[4];
    u8 als_resistor;
};
#endif



#endif /* __LEDS_LM3530_H__ */
