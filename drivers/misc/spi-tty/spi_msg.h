#ifndef _SPI_MSG_H
#define _SPI_MSG_H

#define SPI_TRANSACTION_LEN 16256
#define SPI_MSG_HEADER_LEN 16
#define SPI_MTU (SPI_TRANSACTION_LEN - SPI_MSG_HEADER_LEN)

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
