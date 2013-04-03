
/*
 *	tcmd_driver.c
 *
 * Copyright (c) 2010 Motorola
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/tcmd_driver.h>

#define NAME "tcmd_driver"

static struct tcmd_driver_platform_data *tcmd_misc_data;

static int tcmd_misc_open(struct inode *inode, struct file *file)
{
	int err = nonseekable_open(inode, file);
	if (err < 0)
		return err;

	return 0;
}

static long tcmd_misc_ioctl( struct file *file,
				unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int gpio_enum = -1, gpio = -1, irq = -1, gpio_state = -1;

	if (copy_from_user(&gpio_enum, argp, sizeof(int)))
		return -EFAULT;
	pr_info("tcmd_driver gpio enum = %d.\n", gpio_enum);

	if ((gpio_enum < 0) || (gpio_enum >= (tcmd_misc_data->size)))
		return -EINVAL;

	gpio = tcmd_misc_data->gpio_list[gpio_enum];
	if (gpio < 0)
		return -EINVAL;
	pr_info("tcmd_driver gpio = %d.\n", gpio);

	switch (cmd) {
	case TCMD_IOCTL_MASK_INT:
		irq = gpio_to_irq(gpio);
		if (irq < 0)
			return -EINVAL;
		pr_info("tcmd mask interrupt: gpio = %d, irq = %d.\n",
						gpio, irq);
		disable_irq(irq);
		break;
	case TCMD_IOCTL_UNMASK_INT:
		irq = gpio_to_irq(gpio);
		if (irq < 0)
			return -EINVAL;
		pr_info("tcmd unmask interrupt: gpio = %d, irq = %d\n",
						gpio, irq);
		enable_irq(irq);
		break;
	case TCMD_IOCTL_READ_INT:
		gpio_state = gpio_get_value(gpio);
		pr_info("tcmd interrupt state: gpio = %d -> %d.\n",
						gpio, gpio_state);
		if (copy_to_user(argp, &gpio_state, sizeof(int)))
			return -EFAULT;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct file_operations tcmd_misc_fops = {
	.owner = THIS_MODULE,
	.open = tcmd_misc_open,
	.unlocked_ioctl = tcmd_misc_ioctl,
};

static struct miscdevice tcmd_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = NAME,
	.fops = &tcmd_misc_fops,
};

static int __devinit tcmd_probe(struct platform_device *pdev)
{
	int error = misc_register(&tcmd_misc_device);
	if (error < 0) {
		pr_err("%s: tcmd misc register failed!\n", __func__);
		return error;
	}

	if ((!pdev) && (!pdev->dev.platform_data)) {
		pr_err("%s: tcmd driver: platform_device is not available!\n",
				__func__);
		return -EBUSY;
	}
	tcmd_misc_data = (struct tcmd_driver_platform_data *)
		pdev->dev.platform_data;

	return 0;
}

static struct platform_driver tcmd_driver = {
	.probe = tcmd_probe,
	.driver = {
		.name = "tcmd_driver",
	},
};

int tcmd_init(void)
{
	return platform_driver_register(&tcmd_driver);
}

static void __exit tcmd_exit(void)
{
	misc_deregister(&tcmd_misc_device);
}

module_init(tcmd_init);
module_exit(tcmd_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Motorola");
MODULE_DESCRIPTION("tcmd");
