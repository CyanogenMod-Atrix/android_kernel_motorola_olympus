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
#include <linux/spi/spi.h>

#include "spi_tty.h"
#include "spi_dbg.h"
#include "mdm6600_spi.h"

#include <mach/mdm6600_spi.h>

#ifdef CONFIG_SPI_SLAVE_TEGRA
#include <mach/spi.h>
#endif

#define NVODM_PORT(x) ((x) - 'a')

static struct mdm6600_spi_device mdm6600_spi_dev = {0};

static unsigned int mrdy_irq_count = 0;
module_param(mrdy_irq_count, uint, S_IRUGO|S_IWUSR);
static unsigned int mrdy_irq_spur_count = 0;
module_param(mrdy_irq_spur_count, uint, S_IRUGO|S_IWUSR);
static unsigned int trigger_level= 0;
module_param(trigger_level, uint, S_IRUGO|S_IWUSR);

static int mdm6600_spi_control_srdy_gpio(struct mdm6600_spi_device *spi_dev,
					 int value)
{
	gpio_set_value(spi_dev->srdy_gpio, value);
	if (gpio_get_value(spi_dev->srdy_gpio) != value) {
		pr_warning("%s: GPIO discrepancy: should be %d\n", __func__, value);
		return -EIO;
	}

	return 0;
}

static irqreturn_t mdm6600_spi_mrdy_irq_handler(int irq, void *data)
{
	int value;
	struct mdm6600_spi_device *spi_dev = (struct mdm6600_spi_device *)data;
	struct spi_tty_device *spi_tty = &spi_dev->spi_tty;

        value = gpio_get_value(spi_dev->mrdy_gpio);
        if (value == 0) {
                mrdy_irq_count++;
                if (spi_tty->mrdy_callback) {
                        spi_tty->mrdy_callback(spi_tty->callback_context);
                }
        }

	/* Pseudo edge-trigger workaround for AP20 wake interrupts */
	trigger_level = (value == 0 ? IRQF_TRIGGER_HIGH : IRQF_TRIGGER_LOW);
	irq_set_irq_type(spi_dev->mrdy_irq, trigger_level);

	return IRQ_HANDLED;
}

static int mdm6600_assert_slave_srdy(void *ptr)
{
	struct mdm6600_spi_device *spi_dev = ptr;

	return mdm6600_spi_control_srdy_gpio(spi_dev, 0);
}

static int mdm6600_deassert_slave_srdy(void *ptr)
{
	struct mdm6600_spi_device *spi_dev = ptr;

	return mdm6600_spi_control_srdy_gpio(spi_dev, 1);
}

static void mdm6600_spi_slave_config_gpio(struct mdm6600_spi_device *spi_dev)
{
	int err;

	err = gpio_request(spi_dev->srdy_gpio, SPI_GPIO_SRDY_NAME);
	if ( err) {
		SPI_IPC_ERROR("%s: gpio_request for srdy failed\n", __func__);
		goto err_request_srdy;
	}

	err = gpio_direction_output(spi_dev->srdy_gpio, 1);
	if ( err )	{
		SPI_IPC_ERROR("%s: gpio_direction_output srdy failed\n", __func__);
		goto err_set_srdy;
	}

	gpio_set_value(spi_dev->srdy_gpio, 1);

	err = gpio_request(spi_dev->mrdy_gpio, SPI_GPIO_MRDY_NAME);
	if ( err )	{
		pr_err("%s: gpio_request mrdy failed\n", __func__);
		goto err_request_mrdy;
	}

	err = gpio_direction_input(spi_dev->mrdy_gpio);
	if ( err )	{
		pr_err("%s: gpio_direction_input mrdy failed\n", __func__);
		goto err_set_mrdy;
	}

	spi_dev->mrdy_irq = gpio_to_irq(spi_dev->mrdy_gpio);
	printk("%s: INTR irq: %d, current value: %d\n", __func__,
			spi_dev->mrdy_irq,
			gpio_get_value(spi_dev->mrdy_gpio));

	trigger_level = IRQF_TRIGGER_LOW;
	irq_set_irq_wake(spi_dev->mrdy_irq, 1);
	err = request_irq(spi_dev->mrdy_irq,
			mdm6600_spi_mrdy_irq_handler,
			trigger_level,
			MRDY_IRQ_NAME,
			spi_dev);
	if (err != 0) {
		SPI_IPC_ERROR("%s: request_irq (%d) failed\n", __func__,
		       spi_dev->mrdy_irq);
		goto err_request_irq;
	}

	err = spi_tegra_register_ready_callback(spi_dev->spi,
		mdm6600_assert_slave_srdy, spi_dev);
	if (err < 0) {
		pr_err("spi_tegra_register_ready_callback() failed\n");
		goto err_request_irq;
	}

	err = spi_tegra_register_done_callback(spi_dev->spi,
		mdm6600_deassert_slave_srdy, spi_dev);
	if (err < 0) {
		pr_err("spi_tegra_register_done_callback() failed\n");
		goto err_request_irq;
	}

	return;

err_request_irq:
err_set_mrdy:
	gpio_free(spi_dev->mrdy_gpio);
err_request_mrdy:
err_set_srdy:
	gpio_free(spi_dev->srdy_gpio);
err_request_srdy:
	return;
}

static void mdm6600_spi_handle_peer_startup(void *context)
{
	struct mdm6600_spi_device *spi_dev =
			(struct mdm6600_spi_device *)context;

	printk("%s: enabling IRQ %d\n", __func__, spi_dev->mrdy_irq);
	spi_dev->spi_tty.peer_is_dead = false;
	enable_irq(spi_dev->mrdy_irq);
}

static void mdm6600_spi_handle_peer_shutdown(void *context)
{
	struct mdm6600_spi_device *spi_dev =
			(struct mdm6600_spi_device *)context;

	printk("%s: disabling IRQ %d\n", __func__, spi_dev->mrdy_irq);
	spi_dev->spi_tty.peer_is_dead = true;
	disable_irq_nosync(spi_dev->mrdy_irq);
}

static int __devinit mdm6600_spi_probe(struct spi_device *spi)
{
	struct mdm6600_spi_platform_data *plat;
	struct mdm6600_spi_device *spi_dev = &mdm6600_spi_dev;

	plat = (struct mdm6600_spi_platform_data *)spi->dev.platform_data;

	SPI_IPC_INFO("%s, spi dev=%p, srdy=%d, mrdy=%d\n", __func__,
			spi, plat->gpio_srdy, plat->gpio_mrdy);

	spi->dev.platform_data = spi_dev;
	spi_dev->srdy_gpio = plat->gpio_srdy;
	spi_dev->mrdy_gpio = plat->gpio_mrdy;
	spi_dev->spi = spi;
	spi_dev->spi_tty.spi = spi;

	mdm6600_spi_slave_config_gpio(spi_dev);

#ifdef CONFIG_MDM_CTRL
	if (plat->peer_register)
		plat->peer_register(mdm6600_spi_handle_peer_startup,
		                    mdm6600_spi_handle_peer_shutdown,
				    &mdm6600_spi_dev);
#endif
	return spi_tty_register_device(&spi_dev->spi_tty);
}

static int mdm6600_spi_suspend(struct spi_device *dev, pm_message_t mesg)
{
	SPI_IPC_INFO("%s\n", __func__);
        return 0;
}

static int mdm6600_spi_resume(struct spi_device *dev)
{
	SPI_IPC_INFO("%s\n", __func__);
        return 0;
}

static int __devexit mdm6600_spi_remove(struct spi_device *spi)
{
	return 0;
}

static struct spi_driver mdm6600_spi_driver = {
	.driver = {
		   .name = "mdm6600_spi",
		   .bus = &spi_bus_type,
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
	return spi_register_driver(&mdm6600_spi_driver);
}

static void mdm6600_spi_shutdown(void)
{
	spi_unregister_driver(&mdm6600_spi_driver);
}

module_init(mdm6600_spi_init);
module_exit(mdm6600_spi_shutdown);
