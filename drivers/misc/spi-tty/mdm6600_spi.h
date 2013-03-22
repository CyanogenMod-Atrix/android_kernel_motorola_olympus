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
