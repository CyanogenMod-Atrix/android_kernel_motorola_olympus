/*
 * arch/arm/mach-tegra/board-olympus.h
 *
 * Copyright 2013: Olympus Kernel Project
 * <http://forum.xda-developers.com/showthread.php?t=2016837>
 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef __MACH_TEGRA_BOARD_OLYMPUS_H
#define __MACH_TEGRA_BOARD_OLYMPUS_H
#include <linux/i2c.h>
#include "hwrev.h"

extern unsigned int s_MotorolaDispInfo;

extern int __init olympus_kbc_init(void);
extern void __init olympus_pinmux_init(void);
extern void __init olympus_audio_init(void);
extern int __init olympus_panel_init(void);
extern void __init olympus_emc_init(void);
extern void __init olympus_devices_init(void);
extern void __init olympus_power_init(void);
extern void __init olympus_keypad_init(void);
extern void __init olympus_touch_init(void);
extern void __init olympus_backlight_init(void);
extern void __init olympus_sensors_init(void);
extern void __init olympus_camera_init(void);
extern void __init olympus_usb_gadget_init(void);
extern void __init olympus_clks_init(void);

extern int olympus_mdm_ctrl_shutdown(void);
extern int olympus_mdm_ctrl_peer_register(void (*)(void*),
                                      void (*)(void*),
                                      void*);
extern int __init olympus_wlan_init(void);

extern int __init olympus_modem_init(void);

extern void __init olympus_hdmi_init(void);

extern void olympus_system_power_off(void);
extern void olympus_sec_init(void);
extern void olympus_tcmd_init(void);

extern void tegra_otg_set_mode(int);

int olympus_wifi_status_register(
		void (*sdhcicallback)(int card_present, void *dev_id),
		void *dev_id);

extern int olympus_keymap_update_init(void);

extern void cpcap_set_dock_switch(int state);

void __init olympus_usb_init(void);

void __init olympus_uart_init(void);

void __init olympus_sdhci_init(void);

void __init olympus_i2c_init(void);

void __init olympus_spi_init(void);

void tegra_system_power_off(void);

#define SERIAL_NUMBER_STRING_LEN 17

#define TEGRA_PROX_INT_GPIO		TEGRA_GPIO_PE1
#define TEGRA_HF_NORTH_GPIO		TEGRA_GPIO_PS2
#define TEGRA_HF_SOUTH_GPIO		TEGRA_GPIO_PS0
#define TEGRA_HF_KICKSTAND_GPIO		TEGRA_GPIO_PW3
#define TEGRA_VIBRATOR_GPIO		TEGRA_GPIO_PD0
#define TEGRA_KXTF9_INT_GPIO		TEGRA_GPIO_PV3
#define TEGRA_L3G4200D_IRQ_GPIO		TEGRA_GPIO_PH2
#define TEGRA_AKM8975_IRQ_GPIO		TEGRA_GPIO_PE2
#define TEGRA_AKM8975_RESET_GPIO	TEGRA_GPIO_PK5
#define TEGRA_ADT7461_IRQ_GPIO		TEGRA_GPIO_PD1 //TEGRA_GPIO_PE5
#define PWRUP_BAREBOARD            	0x00100000 /* Bit 20 */

#define TOUCH_GPIO_RESET	TEGRA_GPIO_PF4
#define TOUCH_GPIO_INTR		TEGRA_GPIO_PF5

#define OLYMPUS_EXT_SDCARD_DETECT	TEGRA_GPIO_PI5
#define MMC_OCR_1V8_MASK    0x8

#endif

