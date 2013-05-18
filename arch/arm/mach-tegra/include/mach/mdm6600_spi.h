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

#ifndef _MACH_MDM6600_SPI_H_
#define _MACH_MDM6600_SPI_H_

#ifdef CONFIG_MDM_CTRL
#include <mach/mdm_ctrl.h>
#endif

#define SPI_GPIO_SRDY_NAME "spi_gpio_srdy"
#define SPI_GPIO_MRDY_NAME "spi_gpio_mrdy"
#define MRDY_IRQ_NAME "spi_mrdy"

struct mdm6600_spi_platform_data
{
	int gpio_mrdy;
	int gpio_srdy;
#ifdef CONFIG_MDM_CTRL
	mdm_ctrl_peer_register_t peer_register;
#endif
};

#endif /* _MACH_MDM6600_SPI_H_ */
