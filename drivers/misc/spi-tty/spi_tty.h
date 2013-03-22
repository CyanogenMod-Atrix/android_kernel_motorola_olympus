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
