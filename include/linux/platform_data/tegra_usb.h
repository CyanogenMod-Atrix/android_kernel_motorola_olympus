/*
 * Copyright (C) 2010 Google, Inc.
 * Copyright (c) 2010-2012, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _TEGRA_USB_H_
#define _TEGRA_USB_H_

/**
 * defines operation mode of the USB controller
 */
enum tegra_usb_operation_mode {
	TEGRA_USB_OPMODE_DEVICE,
	TEGRA_USB_OPMODE_HOST,
};

/**
 * defines the various phy interface mode supported by controller
 */
enum tegra_usb_phy_interface {
	TEGRA_USB_PHY_INTF_UTMI = 0,
	TEGRA_USB_PHY_INTF_ULPI_LINK = 1,
	TEGRA_USB_PHY_INTF_ULPI_NULL = 2,
	TEGRA_USB_PHY_INTF_HSIC = 3,
	TEGRA_USB_PHY_INTF_ICUSB = 4,
};

/**
 * configuration structure for setting up utmi phy
 */
struct tegra_utmi_config {
	u8 hssync_start_delay;
	u8 elastic_limit;
	u8 idle_wait_delay;
	u8 term_range_adj;
	u8 xcvr_setup;
	u8 xcvr_lsfslew;
	u8 xcvr_lsrslew;
	signed char xcvr_setup_offset;
	u8 xcvr_use_lsb;
	u8 xcvr_use_fuses;
};

/**
 * configuration structure for setting up ulpi phy
 */
struct tegra_ulpi_config {
	u8 shadow_clk_delay;
	u8 clock_out_delay;
	u8 data_trimmer;
	u8 stpdirnxt_trimmer;
	u8 dir_trimmer;
	const char *clk;
	int phy_restore_gpio;
};

/**
 * configuration structure for setting up hsic phy
 */
struct tegra_hsic_config {
	u8 sync_start_delay;
	u8 idle_wait_delay;
	u8 term_range_adj;
	u8 elastic_underrun_limit;
	u8 elastic_overrun_limit;
};

/**
 * Platform specific operations that will be controlled
 * during the phy operations.
 */
struct tegra_usb_phy_platform_ops {
	void (*open)(void);
	void (*init)(void);
	void (*pre_suspend)(void);
	void (*post_suspend)(void);
	void (*pre_resume)(void);
	void (*post_resume)(void);
	void (*pre_phy_off)(void);
	void (*post_phy_off)(void);
	void (*pre_phy_on)(void);
	void (*post_phy_on)(void);
	void (*port_power)(void);
	void (*close)(void);
};

/**
 * defines structure for platform dependent device parameters
 */
struct tegra_usb_dev_mode_data {
	int vbus_pmu_irq;
	int vbus_gpio;
	bool charging_supported;
	bool remote_wakeup_supported;
};

/**
 * defines structure for platform dependent host parameters
 */
struct tegra_usb_host_mode_data {
	int vbus_gpio;
	const char *vbus_reg;
	bool hot_plug;
	bool remote_wakeup_supported;
	bool power_off_on_suspend;
};

/**
 * defines structure for usb platform data
 */
struct tegra_usb_platform_data {
	bool port_otg;
	bool has_hostpc;
	bool builtin_host_disabled;
	enum tegra_usb_phy_interface phy_intf;
	enum tegra_usb_operation_mode op_mode;

	union {
		struct tegra_usb_dev_mode_data dev;
		struct tegra_usb_host_mode_data host;
	} u_data;

	union {
		struct tegra_utmi_config utmi;
		struct tegra_ulpi_config ulpi;
		struct tegra_hsic_config hsic;
	} u_cfg;

	struct tegra_usb_phy_platform_ops *ops;
};

/**
 * defines structure for platform dependent OTG parameters
 */
struct tegra_usb_otg_data {
	struct platform_device *ehci_device;
	struct tegra_usb_platform_data *ehci_pdata;
};

#endif /* _TEGRA_USB_H_ */
