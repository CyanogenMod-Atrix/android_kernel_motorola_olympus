#ifndef __MACH_TEGRA_BOARD_OLYMPUS_H
#define __MACH_TEGRA_BOARD_OLYMPUS_H
#include <linux/i2c.h>
/*#include <linux/i2c/akm8975.h>*/
#include "hwrev.h"
/*
extern struct kxtf9_platform_data kxtf9_data;
extern struct akm8975_platform_data akm8975_data;
extern struct isl29030_platform_data isl29030_als_ir_data_Olympus;
extern struct lm3532_platform_data lm3532_pdata;
extern struct qtouch_ts_platform_data ts_platform_olympus_p_1_43;
extern struct cpcap_platform_data tegra_cpcap_data;
extern struct cpcap_leds tegra_cpcap_leds;
extern struct platform_driver cpcap_usb_connected_driver;
extern struct l3g4200d_platform_data tegra_gyro_pdata;*/

void olympus_pinmux_init(void);
int olympus_panel_init(void);
void olympus_emc_init(void);
void olympus_devices_init(void);
void olympus_power_init(void);
void olympus_keypad_init(void);
void olympus_i2c_init(void);
void olympus_usb_gadget_init(void);
extern void mot_setup_lights(struct i2c_board_info *info);
extern void mot_setup_touch(struct i2c_board_info *info);

extern int mot_mdm_ctrl_shutdown(void);
extern int mot_mdm_ctrl_peer_register(void (*)(void*),
                                      void (*)(void*),
                                      void*);
extern int __init mot_wlan_init(void);
extern int __init mot_modem_init(void);

extern void __init mot_hdmi_init(void);

extern void __init mot_sensors_init(void);

extern int __init mot_nvodmcam_init(void);

extern void mot_system_power_off(void);
extern void mot_sec_init(void);
extern void mot_tcmd_init(void);

extern void tegra_otg_set_mode(int);
extern void sdhci_tegra_wlan_detect(void);

extern void mot_keymap_update_init(void);

extern void cpcap_set_dock_switch(int state);

#define	BACKLIGHT_DEV		0
#define	TOUCHSCREEN_DEV		1

#define SERIAL_NUMBER_STRING_LEN 17

#define TOUCH_GPIO_RESET	TEGRA_GPIO_PF4
#define TOUCH_GPIO_INTR		TEGRA_GPIO_PF5

#ifndef PROX_INT_GPIO
#define	PROX_INT_GPIO	TEGRA_GPIO_PE1
#endif

#endif
