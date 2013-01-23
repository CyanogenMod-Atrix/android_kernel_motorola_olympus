#ifndef MDM6600_SPI_H_
#define MDM6600_SPI_H_

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

struct mdm6600_spi_device
{
	struct spi_slave_device *spi;
/* mrdy is used by spi master to:
     a) As as a response for SPI slave initiated transaction, tell slave that master is ready and
         will shift clock out;
      b) Master is request slave to prepare a data transaction;
    srdy is used by slave to:
     a) Notify master that slave requests for a data transaction;
     b) As as a response for SPI master initiated transaction, tell master that slave is ready and
         clock can be shifted in;
*/
	int mrdy_irq;
	u8 wait_for_mrdy;
	void (*callback)(void *context, u8 *buf, u32 len);	/* spi slave data callback */
	void (*handle_master_mrdy)(void *context);
	void *cb_context;		/* spi slave data callback context */
	/* srdy is used by slave to:
	     a) Notify master that slave requests for a data transaction;
	     b) As as a response for SPI master initiated transaction, tell master that slave is ready and
	         clock can be shifted in;
	*/
	void (*active_slave_srdy)(void *spi);
	void (*deactive_slave_srdy)(void *spi);
	void *hs_gpio; /* handshake gpio context */
	int mrdy;
	int srdy;
	bool peer_is_dead;
};

extern struct mdm6600_spi_device mdm6600_spi_dev;

#endif //MDM6600_SPI_H_
