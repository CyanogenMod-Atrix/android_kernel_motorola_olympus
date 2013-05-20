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

#ifndef SPI_TTY_H
#define SPI_TTY_H

struct spi_tty_device
{
	struct spi_device *spi;
	bool peer_is_dead;

	void *callback_context;
	void (*data_callback)(void *context, u8 *buf, u32 len);
	void (*mrdy_callback)(void *context);
};

int spi_tty_register_device(struct spi_tty_device *device);

#endif
