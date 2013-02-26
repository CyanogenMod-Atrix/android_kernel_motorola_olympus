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
#include <linux/i2c.h>
#include <linux/i2c-tegra.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/spi/cpcap.h>

#include <asm/mach-types.h>
#include <asm/io.h>

#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/pinmux.h>
#include <mach/i2s.h>
#include <mach/spdif.h>
#include <mach/audio.h>
#include <mach/cpcap_audio.h>
#include <mach/io.h>

#include "board-olympus.h"
#include "gpio-names.h"
#include "devices.h"
#include "board.h"
#include "hwrev.h"

/* CPCAP is i2s master; tegra_audio_pdata.master == false */
static void init_dac2(bool bluetooth);

static struct platform_device cpcap_audio_device = {
	.name   = "cpcap_audio",
	.id     = -1,
	.dev    = {
//		.platform_data = &cpcap_audio_pdata,
	},
};

/* This is the CPCAP Stereo DAC interface. */
static struct tegra_audio_platform_data tegra_audio_pdata = {
	.i2s_master	= false, /* CPCAP Stereo DAC */
	.dsp_master	= false, /* Don't care */
	.dma_on		= true,  /* use dma by default */
	.i2s_clk_rate	= 24000000,
	.dap_clk	= "cdev1",
	.audio_sync_clk = "audio_2x",
	.mode		= I2S_BIT_FORMAT_I2S,
	.fifo_fmt	= I2S_FIFO_PACKED,
	.bit_size	= I2S_BIT_SIZE_16,
	.i2s_bus_width = 32, /* Using Packed 16 bit data, the dma is 32 bit. */
	.dsp_bus_width = 16, /* When using DSP mode (unused), this should be 16 bit. */
	.mask		= TEGRA_AUDIO_ENABLE_TX,
};

/* Connected to CPCAP CODEC - Switchable to Bluetooth Audio. */
static struct tegra_audio_platform_data tegra_audio2_pdata = {
	.i2s_master	= false, /* CPCAP CODEC */
	.dsp_master	= true,  /* Bluetooth */
	.dsp_master_clk = 8000,  /* Bluetooth audio speed */
	.dma_on		= true,  /* use dma by default */
	.i2s_clk_rate	= 2000000, /* BCM4329 max bitclock is 2048000 Hz */
	.dap_clk	= "cdev1",
	.audio_sync_clk = "audio_2x",
	.mode		= I2S_BIT_FORMAT_DSP, /* Using COCEC in network mode */
	.fifo_fmt	= I2S_FIFO_16_LSB,
	.bit_size	= I2S_BIT_SIZE_16,
	.i2s_bus_width = 16, /* Capturing a single timeslot, mono 16 bits */
	.dsp_bus_width = 16,
	.mask		= TEGRA_AUDIO_ENABLE_TX | TEGRA_AUDIO_ENABLE_RX,
};

static struct tegra_audio_platform_data tegra_spdif_pdata = {
	.dma_on		= true,  /* use dma by default */
	.i2s_clk_rate	= 5644800,
	.mode		= SPDIF_BIT_MODE_MODE16BIT,
	.fifo_fmt	= 1,
};

static void *das_base = IO_ADDRESS(TEGRA_APB_MISC_BASE);

static inline void das_writel(unsigned long value, unsigned long offset)
{
	writel(value, das_base + offset);
}

#define APB_MISC_DAS_DAP_CTRL_SEL_0             0xc00
#define APB_MISC_DAS_DAC_INPUT_DATA_CLK_SEL_0   0xc40

static void init_dac1(void)
{
	bool master = tegra_audio_pdata.i2s_master;
	/* DAC1 -> DAP1 */
	das_writel((!master)<<31, APB_MISC_DAS_DAP_CTRL_SEL_0);
	das_writel(0, APB_MISC_DAS_DAC_INPUT_DATA_CLK_SEL_0);
}

static void init_dac2(bool bluetooth)
{
	if (!bluetooth) {
		/* DAC2 -> DAP2 for CPCAP CODEC */
		bool master = tegra_audio2_pdata.i2s_master;
		das_writel((!master)<<31 | 1, APB_MISC_DAS_DAP_CTRL_SEL_0 + 4);
		das_writel(1<<28 | 1<<24 | 1,
				APB_MISC_DAS_DAC_INPUT_DATA_CLK_SEL_0 + 4);
	} else {
		/* DAC2 -> DAP4 for Bluetooth Voice */
		bool master = tegra_audio2_pdata.dsp_master;
		das_writel((!master)<<31 | 1, APB_MISC_DAS_DAP_CTRL_SEL_0 + 12);
		das_writel(3<<28 | 3<<24 | 3,
				APB_MISC_DAS_DAC_INPUT_DATA_CLK_SEL_0 + 4);
	}
}

void __init olympus_audio_init(void)
{
	if (1==0) {
	init_dac1();
	init_dac2(false);

	tegra_i2s_device1.dev.platform_data = &tegra_audio_pdata;
	tegra_i2s_device2.dev.platform_data = &tegra_audio2_pdata;
	}
	cpcap_device_register(&cpcap_audio_device);
	if (1==0) tegra_spdif_device.dev.platform_data = &tegra_spdif_pdata;

}

