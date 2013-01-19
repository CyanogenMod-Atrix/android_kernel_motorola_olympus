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

#ifndef __LEDS_LM3532_H__
#define __LEDS_LM3532_H__

#include <linux/ioctl.h>

#define LM3532_NAME                 "lm3532"
#define LM3532_LED_NAME             "lcd-backlight"
#define LM3532_LED_NAME_NORAMP      "lcd-backlight-nr"
#define LM3532_LED_NAME_TCMD        "lcd-backlight-tcmd"
#define LM3532_LED_NAME_WEBTOP      "lcd-backlight-webtop"
#define LM3532_BUTTON_LED_NAME      "button-backlight"
#define LM3532_BUTTON_LED_NAME_TCMD "button-backlight-tcmd"
#define LM3532_I2C_ADDR        0x38

/*****************************************************************************
 *  LM3532 registers
 *****************************************************************************/
#define LM3532_OUTPUT_CFG_REG       0x10
#define LM3532_START_UP_RAMP_REG    0x11
#define LM3532_RUN_TIME_RAMP_REG    0x12
#define LM3532_CTRL_A_PWM_REG       0x13
#define LM3532_CTRL_B_PWM_REG       0x14
#define LM3532_CTRL_C_PWM_REG       0x15
#define LM3532_CTRL_A_BR_CFG_REG    0x16
#define LM3532_CTRL_A_FS_CURR_REG   0x17

#define LM3532_R0_CTRL_B_BR_CFG_REG 0x17
#define LM3532_CTRL_B_BR_CFG_REG    0x18
#define LM3532_CTRL_B_FS_CURR_REG   0x19

#define LM3532_R0_CTRL_C_BR_CFG_REG 0x18
#define LM3532_CTRL_C_BR_CFG_REG    0x1A
#define LM3532_CTRL_C_FS_CURR_REG   0x1B

#define LM3532_FEEDBACK_ENABLE_REG  0x1C
#define LM3532_R0_ENABLE_REG        0x19
#define LM3532_ENABLE_REG           0x1D

#define LM3532_ALS1_RES_SEL_REG     0x20
#define LM3532_ALS2_RES_SEL_REG     0x21

#define LM3532_ALS_DOWN_DELAY_REG   0x22

#define LM3532_R0_ALS_CFG_REG       0x22
#define LM3532_ALS_CFG_REG          0x23

#define LM3532_R0_ALS_ZONE_REG      0x23
#define LM3532_ALS_ZONE_REG         0x24

#define LM3532_R0_ALS_BR_ZONE_REG   0x24
#define LM3532_ALS_BR_ZONE_REG      0x25

#define LM3532_R0_ALS_UP_ZONE_REG   0x25
#define LM3532_ALS_UP_ZONE_REG      0x26

#define LM3532_ADC_REG              0x27
#define LM3532_ADC_AVG_REG          0x28

#define LM3532_R0_IND_BLINK1_REG    0x30
#define LM3532_R0_IND_BLINK2_REG    0x31

#define LM3532_ZB1_REG              0x60
#define LM3532_ZB2_REG              0x61
#define LM3532_ZB3_REG              0x62
#define LM3532_ZB4_REG              0x63

#define LM3532_CTRL_A_ZT1_REG       0x70
#define LM3532_CTRL_A_ZT2_REG       0x71
#define LM3532_CTRL_A_ZT3_REG       0x72
#define LM3532_CTRL_A_ZT4_REG       0x73
#define LM3532_CTRL_A_ZT5_REG       0x74
#define LM3532_CTRL_B_ZT1_REG       0x75
#define LM3532_CTRL_B_ZT2_REG       0x76
#define LM3532_CTRL_B_ZT3_REG       0x77
#define LM3532_CTRL_B_ZT4_REG       0x78
#define LM3532_CTRL_B_ZT5_REG       0x79
#define LM3532_CTRL_C_ZT1_REG       0x7A
#define LM3532_CTRL_C_ZT2_REG       0x7B
#define LM3532_CTRL_C_ZT3_REG       0x7C
#define LM3532_CTRL_C_ZT4_REG       0x7D
#define LM3532_CTRL_C_ZT5_REG       0x7E
#define LM3532_VERSION_REG          0xCC

/* Brightness config values */
#define LM3532_I2C_CONTROL      0x01
#define LM3532_ALS_CONTROL      0x00
#define LM3532_LINEAR_MAPPING   0x02
#define LM3532_EXP_MAPPING      0x00

#define LM3532_R0_29mA_FS_CURRENT  0xE0
#define LM3532_R0_26mA_FS_CURRENT  0xC0
#define LM3532_R0_22mA_FS_CURRENT  0xA0
#define LM3532_R0_19mA_FS_CURRENT  0x80
#define LM3532_R0_15mA_FS_CURRENT  0x60
#define LM3532_R0_12mA_FS_CURRENT  0x40
#define LM3532_R0_8mA_FS_CURRENT   0x20
#define LM3532_R0_5mA_FS_CURRENT   0x00

#define LM3532_5mA_FS_CURRENT      0x00
#define LM3532_5p8mA_FS_CURRENT    0x01
#define LM3532_6p6mA_FS_CURRENT    0x02
#define LM3532_7p4mA_FS_CURRENT    0x03
#define LM3532_8p2mA_FS_CURRENT    0x04
#define LM3532_9mA_FS_CURRENT      0x05
#define LM3532_9p8mA_FS_CURRENT    0x06
#define LM3532_10p6mA_FS_CURRENT   0x07
#define LM3532_11p4mA_FS_CURRENT   0x08
#define LM3532_12p2mA_FS_CURRENT   0x09
#define LM3532_13mA_FS_CURRENT     0x0A
#define LM3532_13p8mA_FS_CURRENT   0x0B
#define LM3532_14p6mA_FS_CURRENT   0x0C
#define LM3532_15p4mA_FS_CURRENT   0x0D
#define LM3532_16p2mA_FS_CURRENT   0x0E
#define LM3532_17mA_FS_CURRENT     0x0F
#define LM3532_17p8mA_FS_CURRENT   0x10
#define LM3532_18p6mA_FS_CURRENT   0x11
#define LM3532_19p4mA_FS_CURRENT   0x12
#define LM3532_20p2mA_FS_CURRENT   0x13
#define LM3532_21mA_FS_CURRENT     0x14
#define LM3532_21p8mA_FS_CURRENT   0x15
#define LM3532_22p6mA_FS_CURRENT   0x16
#define LM3532_23p4mA_FS_CURRENT   0x17
#define LM3532_24p2mA_FS_CURRENT   0x18
#define LM3532_25mA_FS_CURRENT     0x19
#define LM3532_25p8mA_FS_CURRENT   0x1A
#define LM3532_26p6mA_FS_CURRENT   0x1B
#define LM3532_27p4mA_FS_CURRENT   0x1C
#define LM3532_28p2mA_FS_CURRENT   0x1D
#define LM3532_29mA_FS_CURRENT     0x1E
#define LM3532_29p8mA_FS_CURRENT   0x1F

/* Enable register values */
#define LM3532_CTRLA_ENABLE     0x01
#define LM3532_CTRLB_ENABLE     0x02
#define LM3532_CTRLC_ENABLE     0x04
#define LM3532_IND_ENABLE       0x08
#define LM3532_IND_BLINK_ENABLE 0x10

/* Flags */
#define LM3532_CONFIG_BUTTON_BL 0x1
#define LM3532_CONFIG_ALS       0x2
#define LM3532_DISP_BTN_TIED    0x4
#define LM3532_HAS_WEBTOP       0x8

#ifdef __KERNEL__

struct lm3532_platform_data {
    int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);

    unsigned flags;
    unsigned ramp_time;          /* Ramp time if ramping in the driver */
    unsigned ctrl_a_fs_current;  /* control a full scale current */
    unsigned ctrl_b_fs_current;  /* control b full scale current */
    unsigned ctrl_c_fs_current;  /* control c full scale current */
    unsigned ctrl_a_mapping_mode; /* Control A Mapping mode (linear/exp) */
    unsigned ctrl_b_mapping_mode; /* Control B Mapping mode (linear/exp) */
    unsigned ctrl_c_mapping_mode; /* Control C Mapping mode (linear/exp) */
	unsigned ctrl_a_pwm;
};
#endif



#endif /* __LEDS_LM3532_H__ */
