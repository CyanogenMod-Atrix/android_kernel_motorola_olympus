#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/module.h>
#include "spi_msg.h"
#include "spi_dbg.h"

// SPI transfer protocol
struct spi_msg_header{
	u32 type;
	u32 len;
	u32 dtr;
	u32 fcs;
};

void spi_msg_set_type(u8 *buf, u32 type)
{
	buf[3] = (type>>24)&0xff;
	buf[2] = (type>>16)&0xff;
	buf[1] = (type>>8)&0xff;
	buf[0] = (type)&0xff;
}

void spi_msg_get_type(u8 *buf, u32 *type)
{
	(*type) = (buf[3] << 24)&0xff000000;
	(*type) |= (buf[2] << 16)&0xff0000;
	(*type) |= (buf[1] << 8)&0xff00;
	(*type) |= (buf[0])&0xff;
}

void spi_msg_set_len(u8 *buf, u32 len)
{
	buf[7] = (len>>24)&0xff;
	buf[6] = (len>>16)&0xff;
	buf[5] = (len>>8)&0xff;
	buf[4] = (len)&0xff;
}

void spi_msg_get_len(u8 *buf, u32 *len)
{
	(*len) = (buf[7] << 24)&0xff000000;
	(*len) |= (buf[6] << 16)&0xff0000;
	(*len) |= (buf[5] << 8)&0xff00;
	(*len) |= (buf[4])&0xff;
}

void spi_msg_set_dtr(u8 *buf, u32 dtr)
{
	buf[11] = (dtr>>24)&0xff;
	buf[10] = (dtr>>16)&0xff;
	buf[9] = (dtr>>8)&0xff;
	buf[8] = (dtr)&0xff;
}

void spi_msg_get_dtr(u8 *buf, u32 *dtr)
{
	(*dtr) = (buf[11] << 24)&0xff000000;
	(*dtr) |= (buf[10] << 16)&0xff0000;
	(*dtr) |= (buf[9] << 8)&0xff00;
	(*dtr) |= (buf[8])&0xff;
}

void spi_msg_set_fcs(u8 *buf, u32 fcs)
{
	buf[15] = (fcs>>24)&0xff;
	buf[14] = (fcs>>16)&0xff;
	buf[13] = (fcs>>8)&0xff;
	buf[12] = (fcs)&0xff;
}

void spi_msg_get_fcs(u8 *buf, u32 *fcs)
{
	(*fcs) = (buf[15] << 24)&0xff000000;
	(*fcs) |= (buf[14] << 16)&0xff0000;
	(*fcs) |= (buf[13] << 8)&0xff00;
	(*fcs) |= (buf[12])&0xff;
}

u32 spi_msg_cal_crc(spi_msg_header *header)
{
	u32 fcs = 0;

	fcs += header->type;
	fcs += header->len;
	fcs += header->dtr;

	return fcs;
}

void spi_msg_set_header(u8 *buf, spi_msg_header *header)
{
	spi_msg_set_type(buf, header->type);
	spi_msg_set_len(buf, header->len);
	spi_msg_set_dtr(buf, header->dtr);
	spi_msg_set_fcs(buf, header->fcs);
}

void spi_msg_get_header(u8 *buf, spi_msg_header *header)
{
	spi_msg_get_type(buf, &(header->type));
	spi_msg_get_len(buf, &(header->len));
	spi_msg_get_dtr(buf, &(header->dtr));
	spi_msg_get_fcs(buf, &(header->fcs));
}

