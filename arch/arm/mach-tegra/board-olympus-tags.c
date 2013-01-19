/*
 * arch/arm/mach-tegra/lge/star/star_bootloader_parse.c
 *
 * Based on 
 * arch/arm/mach-tegra/board-shuttle.c
 *
 * Copyright (C) 2011 Eduardo José Tagle <ejtagle@tutopia.com>
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>

#include <asm/bootinfo.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/setup.h>

#include <mach/io.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/sdhci.h>
#include <mach/gpio.h>
#include <mach/clk.h>

#include "board.h"
#include "board-olympus.h"
#include "clock.h"
#include "gpio-names.h"
#include "devices.h"


/* NVidia bootloader tags and parsing routines */
#define ATAG_NVIDIA		0x41000801

#define ATAG_NVIDIA_RM			0x1
#define ATAG_NVIDIA_DISPLAY		0x2
#define ATAG_NVIDIA_FRAMEBUFFER		0x3
#define ATAG_NVIDIA_CHIPSHMOO		0x4
#define ATAG_NVIDIA_CHIPSHMOOPHYS	0x5
#define ATAG_NVIDIA_CARVEOUT		0x6
#define ATAG_NVIDIA_WARMBOOT		0x7

#define ATAG_NVIDIA_PRESERVED_MEM_0	0x10000
#define ATAG_NVIDIA_PRESERVED_MEM_N	3
#define ATAG_NVIDIA_FORCE_32		0x7fffffff

/**
 * Resource Manager boot args.
 *
 * Nothing here yet.
 */
struct NVBOOTARGS_Rm
{
	u32 	reserved;
};

/**
 * Carveout boot args, which define the physical memory location of the GPU
 * carved-out memory region(s).
 */
struct NVBOOTARGS_Carveout
{
	void* 	base;
	u32 	size;
};

/**
 * Warmbootloader boot args. This structure only contains
 * a mem handle key to preserve the warm bootloader
 * across the bootloader->os transition
 */
struct NVBOOTARGS_Warmboot
{
	/* The key used for accessing the preserved memory handle */
	u32 	MemHandleKey;
};

/**
 * PreservedMemHandle boot args, indexed by ATAG_NVIDIA_PRESERVED_MEM_0 + n.
 * This allows physical memory allocations (e.g., for framebuffers) to persist
 * between the bootloader and operating system.  Only carveout and IRAM
 * allocations may be preserved with this interface.
 */
struct NVBOOTARGS_PreservedMemHandle
{
	u32 	Address;
	u32   	Size;
};

/**
 * Display boot args.
 *
 * The bootloader may have a splash screen. This will flag which controller
 * and device was used for the splash screen so the device will not be
 * reinitialized (which causes visual artifacts).
 */
struct NVBOOTARGS_Display
{
	/* which controller is initialized */
	u32 	Controller;

	/* index into the ODM device list of the boot display device */
	u32 	DisplayDeviceIndex;

	/* set to != 0 if the display has been initialized */
	u8 		bEnabled;
};

/**
 * Framebuffer boot args
 *
 * A framebuffer may be shared between the bootloader and the
 * operating system display driver.  When this key is present,
 * a preserved memory handle for the framebuffer must also
 * be present, to ensure that no display corruption occurs
 * during the transition.
 */
struct NVBOOTARGS_Framebuffer
{
	/*  The key used for accessing the preserved memory handle */
	u32 	MemHandleKey;
	/*  Total memory size of the framebuffer */
	u32 	Size;
	/*  Color format of the framebuffer, cast to a U32  */
	u32 	ColorFormat;
	/*  Width of the framebuffer, in pixels  */
	u16 	Width;
	/*  Height of each surface in the framebuffer, in pixels  */
	u16 	Height;
	/*  Pitch of a framebuffer scanline, in bytes  */
	u16 	Pitch;
	/*  Surface layout of the framebuffer, cast to a U8 */
	u8  	SurfaceLayout;
	/*  Number of contiguous surfaces of the same height in the
	    framebuffer, if multi-buffering.  Each surface is
	    assumed to begin at Pitch * Height bytes from the
	    previous surface.  */
	u8  	NumSurfaces;
	/* Flags for future expandability.
	   Current allowable flags are:
	   zero - default
	   NV_BOOT_ARGS_FB_FLAG_TEARING_EFFECT - use a tearing effect signal in
	   combination with a trigger from the display software to generate
	   a frame of pixels for the display device. */
	u32 	Flags;
#define NVBOOTARG_FB_FLAG_TEARING_EFFECT (0x1)

};

/**
 * Chip characterization shmoo data
 */
struct NVBOOTARGS_ChipShmoo
{
	/* The key used for accessing the preserved memory handle of packed
	   characterization tables  */
	u32 	MemHandleKey;

	/* Offset and size of each unit in the packed buffer */
	u32 	CoreShmooVoltagesListOffset;
	u32 	CoreShmooVoltagesListSize;

	u32 	CoreScaledLimitsListOffset;
	u32 	CoreScaledLimitsListSize;

	u32 	OscDoublerListOffset;
	u32 	OscDoublerListSize;

	u32 	SKUedLimitsOffset;
	u32 	SKUedLimitsSize;

	u32 	CpuShmooVoltagesListOffset;
	u32 	CpuShmooVoltagesListSize;

	u32 	CpuScaledLimitsOffset;
	u32 	CpuScaledLimitsSize;

	/* Misc characterization settings */
	u16 	CoreCorner;
	u16 	CpuCorner;
	u32 	Dqsib;
	u32 	SvopLowVoltage;
	u32 	SvopLowSetting;
	u32 	SvopHighSetting;
};

/**
 * Chip characterization shmoo data indexed by NvBootArgKey_ChipShmooPhys
 */
struct NVBOOTARGS_ChipShmooPhys
{
	u32 	PhysShmooPtr;
	u32 	Size;
};


/**
 * OS-agnostic bootarg structure.
 */
struct NVBOOTARGS
{
	struct NVBOOTARGS_Rm 					RmArgs;
	struct NVBOOTARGS_Display 				DisplayArgs;
	struct NVBOOTARGS_Framebuffer 			FramebufferArgs;
	struct NVBOOTARGS_ChipShmoo 			ChipShmooArgs;
	struct NVBOOTARGS_ChipShmooPhys			ChipShmooPhysArgs;
	struct NVBOOTARGS_Warmboot 				WarmbootArgs;
	struct NVBOOTARGS_PreservedMemHandle 	MemHandleArgs[ATAG_NVIDIA_PRESERVED_MEM_N];
};

static __initdata struct NVBOOTARGS NvBootArgs = { {0}, {0}, {0}, {0}, {0}, {0}, {{0}} }; 

static int __init get_cfg_from_tags(void)
{
	/* If the bootloader framebuffer is found, use it */
	if (tegra_bootloader_fb_start == 0 && tegra_bootloader_fb_size == 0 &&
			NvBootArgs.FramebufferArgs.MemHandleKey >= ATAG_NVIDIA_PRESERVED_MEM_0 &&
			NvBootArgs.FramebufferArgs.MemHandleKey <  (ATAG_NVIDIA_PRESERVED_MEM_0+ATAG_NVIDIA_PRESERVED_MEM_N) &&
			NvBootArgs.FramebufferArgs.Size != 0 &&
			NvBootArgs.MemHandleArgs[NvBootArgs.FramebufferArgs.MemHandleKey - ATAG_NVIDIA_PRESERVED_MEM_0].Size != 0) 
	{
		/* Got the bootloader framebuffer address and size. Store it */
		tegra_bootloader_fb_start = NvBootArgs.MemHandleArgs[NvBootArgs.FramebufferArgs.MemHandleKey - ATAG_NVIDIA_PRESERVED_MEM_0].Address;
		tegra_bootloader_fb_size  = NvBootArgs.MemHandleArgs[NvBootArgs.FramebufferArgs.MemHandleKey - ATAG_NVIDIA_PRESERVED_MEM_0].Size;

		pr_info("Nvidia ATAG: framebuffer: %lu @ 0x%08lx\n",tegra_bootloader_fb_size,tegra_bootloader_fb_start);
	}

	/* If the LP0 vector is found, use it */
	if (tegra_lp0_vec_start == 0 && tegra_lp0_vec_size == 0 &&
			NvBootArgs.WarmbootArgs.MemHandleKey >= ATAG_NVIDIA_PRESERVED_MEM_0 &&
			NvBootArgs.WarmbootArgs.MemHandleKey <  (ATAG_NVIDIA_PRESERVED_MEM_0+ATAG_NVIDIA_PRESERVED_MEM_N) &&
			NvBootArgs.MemHandleArgs[NvBootArgs.WarmbootArgs.MemHandleKey - ATAG_NVIDIA_PRESERVED_MEM_0].Size != 0) 
	{
		/* Got the Warmboot block address and size. Store it */
		tegra_lp0_vec_start = NvBootArgs.MemHandleArgs[NvBootArgs.WarmbootArgs.MemHandleKey - ATAG_NVIDIA_PRESERVED_MEM_0].Address;
		tegra_lp0_vec_size  = NvBootArgs.MemHandleArgs[NvBootArgs.WarmbootArgs.MemHandleKey - ATAG_NVIDIA_PRESERVED_MEM_0].Size;

		pr_info("Nvidia ATAG: LP0: %lu @ 0x%08lx\n",tegra_lp0_vec_size,tegra_lp0_vec_start);

	}

	return 0;
}

static int __init parse_tag_nvidia(const struct tag *tag)
{
	const char *addr = (const char *)&tag->hdr + sizeof(struct tag_header);
	const struct tag_nvidia_tegra *nvtag = (const struct tag_nvidia_tegra*)addr;

	if (nvtag->bootarg_key >= ATAG_NVIDIA_PRESERVED_MEM_0 &&
			nvtag->bootarg_key <  (ATAG_NVIDIA_PRESERVED_MEM_0+ATAG_NVIDIA_PRESERVED_MEM_N) )
	{
		int Index = nvtag->bootarg_key - ATAG_NVIDIA_PRESERVED_MEM_0;

		struct NVBOOTARGS_PreservedMemHandle *dst = 
			&NvBootArgs.MemHandleArgs[Index];
		const struct NVBOOTARGS_PreservedMemHandle *src = 
			(const struct NVBOOTARGS_PreservedMemHandle *) nvtag->bootarg;

		if (nvtag->bootarg_len != sizeof(*dst)) {
			pr_err("Unexpected preserved memory handle tag length (expected: %d, got: %d!\n",
					sizeof(*dst), nvtag->bootarg_len);
		} else {

			pr_debug("Preserved memhandle: 0x%08x, address: 0x%08x, size: %d\n",
					nvtag->bootarg_key, src->Address, src->Size);

			memcpy(dst,src,sizeof(*dst));
		}
		return get_cfg_from_tags();
	}

	switch (nvtag->bootarg_key) {
		case ATAG_NVIDIA_FRAMEBUFFER:
			{
				struct NVBOOTARGS_Framebuffer *dst = 
					&NvBootArgs.FramebufferArgs;
				const struct NVBOOTARGS_Framebuffer *src = 
					(const struct NVBOOTARGS_Framebuffer *)nvtag->bootarg;

				if (nvtag->bootarg_len != sizeof(*dst)) {
					pr_err("Unexpected framebuffer tag length (expected: %d, got: %d!\n",
							sizeof(*dst), nvtag->bootarg_len);
				} else {
					pr_debug("Framebuffer tag with 0x%08x handle, size: %d\n",
							src->MemHandleKey,src->Size);
					memcpy(dst,src,sizeof(*dst));
				}
				return get_cfg_from_tags();
			}
		case ATAG_NVIDIA_WARMBOOT:
			{
				struct NVBOOTARGS_Warmboot *dst = 
					&NvBootArgs.WarmbootArgs;
				const struct NVBOOTARGS_Warmboot *src =
					(const struct NVBOOTARGS_Warmboot *)nvtag->bootarg;

				if (nvtag->bootarg_len != sizeof(*dst)) {
					pr_err("Unexpected Warnboot tag length (expected: %d, got: %d!\n",
							sizeof(*dst), nvtag->bootarg_len);
				} else {
					pr_debug("Found a warmboot tag with handle 0x%08x!\n", src->MemHandleKey);
					memcpy(dst,src,sizeof(*dst));
				}
				return get_cfg_from_tags();
			}

		default:
			pr_info("Ignoring irrelevant nvidia tag 0x%04x!\n", nvtag->bootarg_key);
			break;
	} 
	return 0;
}
__tagtable(ATAG_NVIDIA, parse_tag_nvidia);

/*
 * Some global queries for the framebuffer, display, and backlight drivers.
 */
static unsigned int s_MotorolaDispInfo = 0;
static unsigned int s_MotorolaFBInfo = 1;

unsigned short bootloader_ver_major = 0;
unsigned short bootloader_ver_minor = 0;
unsigned short uboot_ver_major = 0;
unsigned short uboot_ver_minor = 0;

unsigned char lpddr2_mr[12];

int MotorolaBootFBArgGet(unsigned int *arg)
{
    *arg = s_MotorolaFBInfo;
    return 0;
}

int MotorolaBootDispArgGet(unsigned int *arg)
{
    if(s_MotorolaDispInfo)
    {
        *arg = s_MotorolaDispInfo;
        return 0;
    }

    return -1;
}

/*
 * Parse the Motorola-specific ATAG
 */
static int __init parse_tag_motorola(const struct tag *tag)
{
    const struct tag_motorola *moto_tag = &tag->u.motorola;
    int i = 0;

    s_MotorolaDispInfo = moto_tag->panel_size;
	
    s_MotorolaFBInfo = moto_tag->allow_fb_open;
 
/*   mot_sec_platform_data.fl_factory = moto_tag->in_factory;*/

    bootloader_ver_major = moto_tag->bl_ver_major;
    bootloader_ver_minor = moto_tag->bl_ver_minor;
    uboot_ver_major = moto_tag->uboot_ver_major;
    uboot_ver_minor = moto_tag->uboot_ver_minor;
#ifdef CONFIG_BOOTINFO
    bi_set_cid_recover_boot(moto_tag->cid_suspend_boot);
#endif
    pr_info("%s: bootloader v%d.%d\n", __func__, bootloader_ver_major, bootloader_ver_minor);
    pr_info("%s: uboot v%d.%d\n", __func__, uboot_ver_major, uboot_ver_minor);
    pr_info("%s: panel_size: 0x%x\n", __func__, s_MotorolaDispInfo);
    pr_info("%s: allow_fb_open: %x\n", __func__, s_MotorolaFBInfo);
/*    pr_info("%s: factory: %d\n", __func__, mot_sec_platform_data.fl_factory);*/
    pr_info("%s: cid_suspend_boot: %u\n", __func__, (unsigned)moto_tag->cid_suspend_boot);

    /*
     *	Dump memory information
     */
     /* FIXME:  Add eMMC support */
	for (i = 0; i < 12; i++) {
		lpddr2_mr[i] = moto_tag->at_lpddr2_mr[i];
		pr_info("%s: LPDDR2 MR%d:     0x%04X (0x%04X)\n", __func__, i,
			lpddr2_mr[i],
			moto_tag->at_lpddr2_mr[i]);
	}

    return 0;
}
__tagtable(ATAG_MOTOROLA, parse_tag_motorola);

/*
 * Parse the Motorola boot loader ATAG 41000811
 */
static int __init parse_tag_bldebug(const struct tag *t)
{
	pr_info("%s: powerup reason regs: INTS1=0x%4.4x INT2=0x%4.4x INTS2=0x%4.4x INT3=0x%4.4x "
		"PC2=0x%4.4x MEMA=0x%4.4x ACCY=%d UBOOT=%d\n", __func__, t->u.bldebug.ints1,
		t->u.bldebug.int2, t->u.bldebug.ints2, t->u.bldebug.int3, t->u.bldebug.pc2,
		t->u.bldebug.mema, t->u.bldebug.accy, t->u.bldebug.uboot);
	return 0;
}
__tagtable(ATAG_BLDEBUG, parse_tag_bldebug);

/*
  * Parse the WLAN MAC ATAG
  */

char mot_wlan_mac[6] = {0x00, 0x90, 0xC3, 0x00, 0x00, 0x00};

 static int __init parse_tag_wlan_mac(const struct tag *tag)
 {
	const struct tag_wlan_mac *wlan_mac_tag = &tag->u.wlan_mac;

	pr_info("%s: WLAN MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", __func__,
	        wlan_mac_tag->addr[0], wlan_mac_tag->addr[1],
		wlan_mac_tag->addr[2], wlan_mac_tag->addr[3],
		wlan_mac_tag->addr[4], wlan_mac_tag->addr[5]);

	memcpy(mot_wlan_mac, wlan_mac_tag->addr, sizeof(mot_wlan_mac));

	return 0;
 }
 __tagtable(ATAG_WLAN_MAC, parse_tag_wlan_mac);
