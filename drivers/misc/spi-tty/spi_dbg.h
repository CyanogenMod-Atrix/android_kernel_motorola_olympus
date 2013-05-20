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

#ifndef __SPI_DBG_H
#define __SPI_DBG_H

#define SPI_IPC_DEBUG 1

#if SPI_IPC_DEBUG
#include <linux/sched.h>
#include <linux/ctype.h>

extern unsigned int spi_tty_debug_mask;

#define SPI_IPC_INFO(fmt, args...) do { \
        if (spi_tty_debug_mask == 1) \
                printk(KERN_INFO "[%d] "fmt, current->pid, ## args); \
        }while(0)

#define SPI_IPC_ERROR(fmt, args...) do { \
        if (spi_tty_debug_mask == 1) \
                pr_err("[%d] "fmt, current->pid, ## args); \
        }while(0)

#define spi_ipc_buf_dump(header, buf, len) do { \
	if(spi_tty_debug_mask) \
		spi_ipc_buf_dump1(header, buf, len, 0); \
	}while(0)

#define spi_ipc_buf_dump_ascii(header, buf, len) do { \
	if(spi_tty_debug_mask) \
		spi_ipc_buf_dump1(header, buf, len, 1); \
	}while(0)

#else
#define SPI_IPC_INFO(fmt, args...) do{}while(0)
#define SPI_IPC_ERROR(fmt, args...) pr_error(fmt, ## args);
#define spi_ipc_buf_dump(x, y, z) do{}while(0)
#define spi_ipc_buf_dump_ascii(x, y, z) do{}while(0)
#endif

extern unsigned long spi_ipc_txed_byte;
extern unsigned long spi_ipc_txed_time;
extern unsigned long spi_ipc_txed_count;
extern unsigned long spi_ipc_write_count;

extern void spi_ipc_buf_dump1(const char *header, const u8 *buf, int len, int in_ascii);

#endif //__SPI_DBG_H
