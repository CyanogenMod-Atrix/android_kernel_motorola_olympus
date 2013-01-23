#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/leds-cpcap-display.h>
#include <linux/leds-cpcap-button.h>
#include <linux/leds-ld-cpcap.h>
#include <linux/platform_device.h>
#include <linux/regulator/machine.h>
#include <linux/uaccess.h>
#include <linux/reboot.h>
#include <linux/notifier.h>
#include <linux/delay.h>
#include <linux/spi/spi_slave.h>
#include <linux/spi/mdm6600_spi.h>
#include "spi_dbg.h"

struct mdm6600_spi_device mdm6600_spi_dev = {0};

static unsigned int mrdy_irq_count = 0;
module_param(mrdy_irq_count, uint, S_IRUGO|S_IWUSR);
static unsigned int mrdy_irq_spur_count = 0;
module_param(mrdy_irq_spur_count, uint, S_IRUGO|S_IWUSR);
static unsigned int trigger_level= 0;
module_param(trigger_level, uint, S_IRUGO|S_IWUSR);

static void mdm6600_spi_control_srdy_gpio(struct mdm6600_spi_device *spi, int value)
{
	gpio_set_value(spi->srdy, value);
	if (gpio_get_value(spi->srdy) != value)
		pr_warning("%s: GPIO discrepancy: should be %d\n", __func__, value);
}

static irqreturn_t mdm6600_spi_mrdy_irq_handler(int irq, void *dev_id)
{
	int value;
	struct mdm6600_spi_device *spi = (struct mdm6600_spi_device *)dev_id;

        value = gpio_get_value(spi->mrdy);
        if (value == 0) {
                mrdy_irq_count++;
                // mrdy notify slave to start new transaction
                if (spi->handle_master_mrdy) {
                        spi->handle_master_mrdy(spi->cb_context);
                }
        }

	// This is a hack solution for AP20 wakeup by gpio
	// PL1 can wakeup AP20 but falling edge irq before low level is missed
	// If use low level trigger, we will get many irq, so here we
	// toggle trigger level in irq so that we will only get one time triggered
	trigger_level = (value == 0?IRQF_TRIGGER_HIGH:IRQF_TRIGGER_LOW);
	irq_set_irq_type(spi->mrdy_irq, trigger_level);

	return IRQ_HANDLED;
}

static void mdm6600_active_slave_srdy(void *ptr)
{
	struct mdm6600_spi_device	*spi = ptr;

	mdm6600_spi_control_srdy_gpio(spi, 0);
}

static void mdm6600_deactive_slave_srdy(void *ptr)
{
	struct mdm6600_spi_device	*spi = ptr;

	mdm6600_spi_control_srdy_gpio(spi, 1);
}

static void mdm6600_spi_slave_config_gpio(struct mdm6600_spi_device *spi)
{
	int err;

	err = gpio_request(spi->srdy, SPI_GPIO_SRDY_NAME);
	if ( err) {
		SPI_IPC_ERROR("%s: gpio_request for srdy failed\n", __func__);
		goto err_request_srdy;
	}

	err = gpio_direction_output(spi->srdy, 1);
	if ( err )	{
		SPI_IPC_ERROR("%s: gpio_direction_output srdy failed\n", __func__);
		goto err_set_srdy;
	}

	gpio_set_value(spi->srdy, 1);

	err = gpio_request(spi->mrdy, SPI_GPIO_MRDY_NAME);
	if ( err )	{
		pr_err("%s: gpio_request mrdy failed\n", __func__);
		goto err_request_mrdy;
	}

	err = gpio_direction_input(spi->mrdy);
	if ( err )	{
		pr_err("%s: gpio_direction_input mrdy failed\n", __func__);
		goto err_set_mrdy;
	}

	spi->mrdy_irq = gpio_to_irq(spi->mrdy);
	printk("%s: INTR irq: %d, current value: %d\n",
			__func__, spi->mrdy_irq, gpio_get_value(spi->mrdy));

	trigger_level = IRQF_TRIGGER_LOW;
	err = request_irq(spi->mrdy_irq,
			mdm6600_spi_mrdy_irq_handler,
			trigger_level,
			MRDY_IRQ_NAME,
			spi);
	if (err != 0)
	{
		SPI_IPC_ERROR("%s: request_irq (%d) failed\n", __func__,
		       spi->mrdy_irq);
		goto err_request_irq;
	}
	goto end;

err_request_irq:
err_set_mrdy:
	gpio_free(spi->mrdy);
err_request_mrdy:
err_set_srdy:
	gpio_free(spi->srdy);
err_request_srdy:
end:
	return;
}

static void mdm6600_spi_handle_peer_startup(void *context)
{
	struct mdm6600_spi_device *spi = (struct mdm6600_spi_device *)context;

	printk("%s: enabling IRQ %d\n", __func__, spi->mrdy_irq);
	spi->peer_is_dead = false;
	enable_irq(spi->mrdy_irq);
}

static void mdm6600_spi_handle_peer_shutdown(void *context)
{
	struct mdm6600_spi_device *spi = (struct mdm6600_spi_device *)context;

	printk("%s: disabling IRQ %d\n", __func__, spi->mrdy_irq);
	spi->peer_is_dead = true;
	disable_irq_nosync(spi->mrdy_irq);
}

static int __devinit mdm6600_spi_probe(struct spi_slave_device *spi)
{
	struct mdm6600_spi_platform_data *plat;

	plat = (struct mdm6600_spi_platform_data *)spi->dev.platform_data;

	SPI_IPC_INFO("%s, spi dev=%p, srdy=%d, mrdy=%d\n", __func__,
			spi, plat->gpio_srdy, plat->gpio_mrdy);

	spi->controller_data = &mdm6600_spi_dev;
	mdm6600_spi_dev.srdy = plat->gpio_srdy;
	mdm6600_spi_dev.mrdy = plat->gpio_mrdy;
	mdm6600_spi_dev.spi = spi;
	mdm6600_spi_dev.active_slave_srdy = mdm6600_active_slave_srdy;
	mdm6600_spi_dev.deactive_slave_srdy = mdm6600_deactive_slave_srdy;
	mdm6600_spi_slave_config_gpio(&mdm6600_spi_dev);

#ifdef CONFIG_MDM_CTRL
	if (plat->peer_register)
		plat->peer_register(mdm6600_spi_handle_peer_startup,
		                    mdm6600_spi_handle_peer_shutdown,
				    &mdm6600_spi_dev);
#endif
	return 0;
}

static int mdm6600_spi_suspend(struct spi_slave_device *dev, pm_message_t mesg)
{
	SPI_IPC_INFO("%s\n", __func__);
        return 0;
}

static int mdm6600_spi_resume(struct spi_slave_device *dev)
{
	SPI_IPC_INFO("%s\n", __func__);
        return 0;
}

static int __devexit mdm6600_spi_remove(struct spi_slave_device *spi)
{
	return 0;
}

static struct spi_slave_driver mdm6600_spi_driver = {
	.driver = {
		   .name = "mdm6600_spi",
		   .bus = &spi_slave_bus_type,
		   .owner = THIS_MODULE,
		   },
	.probe = mdm6600_spi_probe,
        .suspend = mdm6600_spi_suspend,
        .resume = mdm6600_spi_resume,
	.remove = __devexit_p(mdm6600_spi_remove),
};

static int __init mdm6600_spi_init(void)
{
	SPI_IPC_INFO("%s\n", __func__);
	return spi_slave_register_driver(&mdm6600_spi_driver);
}

static void mdm6600_spi_shutdown(void)
{
	spi_slave_unregister_driver(&mdm6600_spi_driver);
}

subsys_initcall(mdm6600_spi_init);
module_exit(mdm6600_spi_shutdown);
