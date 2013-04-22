/*
 * arch/arm/mach-tegra/board-olympus-clocks.c
 *
 * ...
 *
 * Copyright (c) 2009-2013, ...
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
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
