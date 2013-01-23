#ifndef __SPI_DBG_H
#define __SPI_DBG_H

#define SPI_IPC_DEBUG 1

#if SPI_IPC_DEBUG
#include <linux/sched.h>
#include <linux/ctype.h>

extern unsigned int debug_mask;

#define SPI_IPC_INFO(fmt, args...) do { \
        if (debug_mask == 1) \
                printk(KERN_INFO "[%d] "fmt, current->pid, ## args); \
        }while(0)

#define SPI_IPC_ERROR(fmt, args...) do { \
        if (debug_mask == 1) \
                pr_err("[%d] "fmt, current->pid, ## args); \
        }while(0)

#define spi_ipc_buf_dump(header, buf, len) do { \
	if(debug_mask) \
		spi_ipc_buf_dump1(header, buf, len, 0); \
	}while(0)

#define spi_ipc_buf_dump_ascii(header, buf, len) do { \
	if(debug_mask) \
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
