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
extern void __init olympus_touch_init(void);
extern void __init olympus_backlight_init(void);
extern void __init olympus_sensors_init(void);
extern void __init olympus_cameras_init(void);
extern void __init olympus_usb_gadget_init(void);

extern int mot_mdm_ctrl_shutdown(void);
extern int mot_mdm_ctrl_peer_register(void (*)(void*),
                                      void (*)(void*),
                                      void*);
extern int __init olympus_wlan_init(void);
extern int __init mot_modem_init(void);

extern void __init mot_hdmi_init(void);

extern void mot_system_power_off(void);
extern void mot_sec_init(void);
extern void mot_tcmd_init(void);

extern void tegra_otg_set_mode(int);
extern void sdhci_tegra_wlan_detect(void);

extern int mot_keymap_update_init(void);

extern void cpcap_set_dock_switch(int state);

#define SERIAL_NUMBER_STRING_LEN 17

#define TOUCH_GPIO_RESET	TEGRA_GPIO_PF4
#define TOUCH_GPIO_INTR		TEGRA_GPIO_PF5

#endif

