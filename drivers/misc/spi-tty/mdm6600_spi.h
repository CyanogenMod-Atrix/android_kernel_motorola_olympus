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

#ifndef _MDM6600_SPI_H_
#define _MDM6600_SPI_H_

struct mdm6600_spi_device
{
	struct spi_device *spi;

	struct spi_tty_device spi_tty;

	/* MRDY is used by the SPI master:
	 * a) as a response to a SPI slave initiated transaction, telling the
	 *    slave that the master is ready and will start the clock.
	 * b) that the master is requesting the slave to prepare for a data
	 *    transaction.
	 *
	 * SRDY is used by the SPI slave:
	 * a) to notify the master that the slave requests a data transaction.
	 * b) as a response to a SPI master initiated transaction, telling the
	 *    master that the slave is ready and the clock can be started.
	 */
	int mrdy_gpio;
	int mrdy_irq;
	int srdy_gpio;
};

#endif /* _MDM6600_SPI_H_ */
