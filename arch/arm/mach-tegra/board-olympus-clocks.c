/*
 * Copyright (C) 2010 Google, Inc.
 * Copyright (C) 2010 NVIDIA Corporation
 * Copyright (C) 2010 Motorola, Inc.
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

#include "clock.h"

static __initdata struct tegra_clk_init_table olympus_clk_init_table[] = {
	/* name		parent		rate		enabled */
	{"sbc1",	"pll_c",	60000000,	true},
	{"sbc2",	"pll_c",	60000000,	true},
	{"pwm",		"clk_32k",	32768,		false},
	{"kbc",		"clk_32k",	32768,		true},
	{"sdmmc2",	"pll_p",	25000000,	false},
	{"i2s1",	"pll_a_out0",	0,		false},
	{"spdif_out",	"pll_a_out0",	0,		false},
	{ NULL,		NULL,		0,		0},
};

void __init olympus_clks_init(void)
{
	tegra_clk_init_from_table(olympus_clk_init_table);
}
