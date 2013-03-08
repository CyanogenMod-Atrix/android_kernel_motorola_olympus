/*
 * Copyright (C) 2010-2011 NVIDIA, Inc.
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


#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/device.h>

#include <mach/clk.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/pinmux.h>
#include <mach/iomap.h>
#include <mach/io.h>
#include <mach/kbc.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#define OLYMPUS_ROW_COUNT	2
#define OLYMPUS_COL_COUNT	3

static const u32 olympus_keymap[] = {
/*	KEY(0, 0, KEY_POWER),
	KEY(0, 1, KEY_VOLUMEUP),
	KEY(1, 0, KEY_HOME),
	KEY(1, 1, KEY_BACK),
	KEY(2, 0, KEY_VOLUMEDOWN),
	KEY(2, 1, KEY_MENU),
	KEY(3, 0, KEY_RESERVED),
	KEY(3, 1, KEY_RESERVED),*/

	KEY(0, 0, 115),
	KEY(1, 0, 114),
	KEY(2, 0, 152),
	KEY(0, 1, 211),
	KEY(1, 1, 212),
	KEY(2, 1, 217),
	KEY(0, 2, 139),
	KEY(1, 2, 102),
	KEY(2, 2, 158),
};

static const struct matrix_keymap_data olympus_keymap_data = {
	.keymap = olympus_keymap,
	.keymap_size = ARRAY_SIZE(olympus_keymap),
};

static struct tegra_kbc_wake_key olympus_wake_cfg[] = {
	[0] = {
		.row = 0,
		.col = 0,
	},
	[1] = {
		.row = 1,
		.col = 0,
	},
	[2] = {
		.row = 1,
		.col = 1,
	},
	[3] = {
		.row = 2,
		//.col = 0,
	},
	[4] = {
		.row = 2,
		.col = 1,
	},
};

static struct tegra_kbc_platform_data olympus_kbc_platform_data = {
	.debounce_cnt = 10,
	.repeat_cnt = 1024,
	.wake_cnt = 5,
	.wake_cfg = &olympus_wake_cfg[0],
	.keymap_data = &olympus_keymap_data,
	.use_fn_map = false,
	.wakeup = true,
#ifdef CONFIG_ANDROID
	.disable_ev_rep = true,
#endif
};

static struct resource olympus_kbc_resources[] = {
	[0] = {
		.start = TEGRA_KBC_BASE,
		.end   = TEGRA_KBC_BASE + TEGRA_KBC_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = INT_KBC,
		.end   = INT_KBC,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device olympus_kbc_device = {
	.name = "tegra-kbc",
	.id = -1,
	.dev = {
		.platform_data = &olympus_kbc_platform_data,
	},
	.resource = olympus_kbc_resources,
	.num_resources = ARRAY_SIZE(olympus_kbc_resources),
};

int __init olympus_kbc_init(void)
{
	struct tegra_kbc_platform_data *data = &olympus_kbc_platform_data;
	int i;

	pr_info("KBC: olympus_kbc_init\n");
	for (i = 0; i < OLYMPUS_ROW_COUNT; i++) {
		data->pin_cfg[i].num = i;
		data->pin_cfg[i].is_row = true;
		data->pin_cfg[i].en = true;
	}
	for (i = 0; i < OLYMPUS_COL_COUNT; i++) {
		data->pin_cfg[i + 16].num = i;
		data->pin_cfg[i + 16].en = true;
	}

	platform_device_register(&olympus_kbc_device);
	return 0;
}
