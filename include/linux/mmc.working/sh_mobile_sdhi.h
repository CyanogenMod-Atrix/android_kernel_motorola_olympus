/*
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

#ifndef __SH_MOBILE_SDHI_H__
#define __SH_MOBILE_SDHI_H__

#include <linux/types.h>

struct sh_mobile_sdhi_info {
	int dma_slave_tx;
	int dma_slave_rx;
	unsigned long tmio_flags;
	unsigned long tmio_caps;
	u32 tmio_ocr_mask;	/* available MMC voltages */
	void (*set_pwr)(struct platform_device *pdev, int state);
	int (*get_cd)(struct platform_device *pdev);
};

#endif /* __SH_MOBILE_SDHI_H__ */
