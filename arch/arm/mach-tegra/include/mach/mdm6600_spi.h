#ifndef _MACH_MDM6600_SPI_H_
#define _MACH_MDM6600_SPI_H_

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

#endif /* _MACH_MDM6600_SPI_H_ */
