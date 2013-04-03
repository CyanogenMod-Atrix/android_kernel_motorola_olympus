/*
 * arch/arm/mach-tegra/board-olympus-i2c.c
 *
 * Copyright (C) 2010 Google, Inc.
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

#include <linux/resource.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/of.h>
#include <linux/spi/cpcap.h>
#include <linux/cpcap_audio_platform_data.h>

#include <asm/mach-types.h>
#include <asm/io.h>

#include <mach/iomap.h>
#include <mach/i2s.h>
#include <mach/spdif.h>
#include <mach/audio.h>
#include <mach/io.h>

#include "board-olympus.h"
#include "gpio-names.h"
#include "devices.h"
#include "board.h"
#include "hwrev.h"

static struct platform_device olympus_codec_cpcap = {
		.name   = "cpcap_audio",
        .id             = 0,
};

static struct platform_device *olympus_audio_devices[] __initdata = {
	&tegra_i2s_device1,
	&tegra_i2s_device2,
	&tegra_das_device,
	&tegra_pcm_device,
	&tegra_spdif_device,
	&olympus_codec_cpcap,
	&spdif_dit_device,
};

void __init olympus_audio_init(void)
{

	platform_add_devices(olympus_audio_devices, ARRAY_SIZE(olympus_audio_devices));

}

