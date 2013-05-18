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

#ifndef _SPI_MSG_H
#define _SPI_MSG_H

#define SPI_TRANSACTION_LEN 16256
#define SPI_MSG_HEADER_LEN 16
#define SPI_MTU (SPI_TRANSACTION_LEN - SPI_MSG_HEADER_LEN)
#define SPI_SPEED_HZ 26000000

typedef struct spi_msg_header_s
{
	u32 type;
	u32 len;
	u32 dtr;
	u32 fcs;
}spi_msg_header;

u32 spi_msg_cal_crc(spi_msg_header *header);
void spi_msg_set_header(u8 *buf, spi_msg_header *header);
void spi_msg_get_header(u8 *buf, spi_msg_header *header);

#endif // _SPI_MSG_H
