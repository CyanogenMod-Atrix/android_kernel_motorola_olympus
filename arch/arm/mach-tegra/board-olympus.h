#ifndef __MACH_TEGRA_BOARD_OLYMPUS_H
#define __MACH_TEGRA_BOARD_OLYMPUS_H
#include <linux/i2c.h>
#include "hwrev.h"

extern int __init olympus_kbc_init(void);
extern void __init olympus_pinmux_init(void);
extern int __init olympus_panel_init(void);
extern void __init olympus_emc_init(void);
extern void __init olympus_devices_init(void);
extern void __init olympus_power_init(void);
extern void __init olympus_keypad_init(void);
extern void __init olympus_i2c_init(void);
extern void __init olympus_audio_init(void);
extern void __init olympus_usb_gadget_init(void);

extern int mot_mdm_ctrl_shutdown(void);
extern int mot_mdm_ctrl_peer_register(void (*)(void*),
                                      void (*)(void*),
                                      void*);
extern int __init olympus_wlan_init(void);
extern int __init mot_modem_init(void);

extern void __init mot_hdmi_init(void);

extern void __init mot_sensors_init(void);

extern int __init mot_nvodmcam_init(void);

extern void mot_system_power_off(void);
extern void mot_sec_init(void);
extern void mot_tcmd_init(void);

extern void tegra_otg_set_mode(int);
extern void sdhci_tegra_wlan_detect(void);

extern int mot_keymap_update_init(void);

extern void cpcap_set_dock_switch(int state);

#define	BACKLIGHT	0
#define	TOUCHSCREEN	1
#define	PROX		2

#define SERIAL_NUMBER_STRING_LEN 17

#define TOUCH_GPIO_RESET	TEGRA_GPIO_PF4
#define TOUCH_GPIO_INTR		TEGRA_GPIO_PF5

//#define MPU_GYRO_NAME           "mpu3050"
//#define MPU_GYRO_IRQ_GPIO       TEGRA_GPIO_PX1
//#define MPU_GYRO_ADDR           0x68
//#define MPU_GYRO_BUS_NUM        3
//#define MPU_GYRO_ORIENTATION    { 1, 0, 0, 0, 1, 0, 0, 0, 1 }
/*
#define MPU_ACCEL_NAME		"kxtf9"
#define MPU_ACCEL_IRQ_GPIO	TEGRA_GPIO_PV3
#define MPU_ACCEL_ADDR		0x0F
#define MPU_ACCEL_BUS_NUM       3
#define MPU_ACCEL_ORIENTATION   { 0, 1, 0, -1, 0, 0, 0, 0, 1 }

#define MPU_COMPASS_NAME        "akm8975"
#define MPU_COMPASS_IRQ_GPIO    TEGRA_GPIO_PE2
#define MPU_COMPASS_RESET_GPIO	TEGRA_GPIO_PK5
#define MPU_COMPASS_ADDR        0x0C
#define MPU_COMPASS_BUS_NUM     3
#define MPU_COMPASS_ORIENTATION { 0, 1, 0, -1, 0, 0, 0, 0, 1 }

#define SENSOR_TEMP_NAME        "nct1008"
#define SENSOR_TEMP_IRQ_GPIO    TEGRA_GPIO_PD1
#define SENSOR_TEMP_RESET_GPIO	TEGRA_GPIO_PV7
#define SENSOR_TEMP_ADDR        0x4C
#define SENSOR_TEMP_BUS_NUM     3

#define TEGRA_PROX_INT_GPIO			TEGRA_GPIO_PE1
#define TEGRA_HF_NORTH_GPIO			TEGRA_GPIO_PS2
#define TEGRA_HF_SOUTH_GPIO			TEGRA_GPIO_PS0
#define TEGRA_HF_KICKSTAND_GPIO			TEGRA_GPIO_PW3
#define TEGRA_VIBRATOR_GPIO			TEGRA_GPIO_PD0
#define TEGRA_L3G4200D_IRQ_GPIO			TEGRA_GPIO_PH2
*/
#endif

