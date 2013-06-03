/*
     Copyright (C) 2010 Motorola, Inc.

     This program is free software; you can redistribute it and/or modify
     it under the terms of the GNU General Public License version 2 as
     published by the Free Software Foundation.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program; if not, write to the Free Software
     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
     02111-1307  USA
*/
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kobject.h>
#include <linux/radio_ctrl/radio_class.h>
#include <linux/mdm_ctrl.h>

#include <mach/mdm_ctrl.h>


static const char *mdmctrl = "mdm6600_ctrl";

static const char *bp_status[8] = {
	[MDM_CTRL_BP_STATUS_PANIC] = "panic",
	[MDM_CTRL_BP_STATUS_PANIC_WAIT] = "panic busy wait",
	[MDM_CTRL_BP_STATUS_CORE_DUMP] = "qc dload",
	[MDM_CTRL_BP_STATUS_RDL] = "ram downloader",
	[MDM_CTRL_BP_STATUS_AWAKE] = "awake",
	[MDM_CTRL_BP_STATUS_ASLEEP] = "asleep",
	[MDM_CTRL_BP_STATUS_SHUTDOWN_ACK] = "shutdown ack",
	[MDM_CTRL_BP_STATUS_UNKNOWN] = "undefined",
};

static const char *bp_power_state[] = {
	"disabled",
	"off",
	"on",
};

#define BP_STATUS_MAX_LENGTH        32
#define BP_COMMAND_MAX_LENGTH       32

static DEFINE_MUTEX(mdm_ctrl_info_lock);

static struct workqueue_struct *working_queue;
static struct work_struct bp_change_work;

static struct radio_dev radio_cdev;

static unsigned int bp_power_up_mode = MDM_CTRL_BP_PU_MODE_NORMAL;

static void __devexit mdm6600_shutdown(struct platform_device *pdev);

static const char *bp_status_string(unsigned int stat)
{
	if (stat < ARRAY_SIZE(bp_status))
		return bp_status[stat];
	else
		return "status out of range";
}

static const char *bp_power_state_string(unsigned int stat)
{
	if (stat < ARRAY_SIZE(bp_power_state))
		return bp_power_state[stat];
	else
		return "status out of range";
}

static ssize_t mdm_status_show(struct radio_dev *dev, char *buff)
{
	ssize_t status = 0;
	status = snprintf(buff, BP_STATUS_MAX_LENGTH, "%s\n",
			  bp_status_string(mdm_ctrl_get_bp_status()));

	return status;
}

static ssize_t mdm_power_show(struct radio_dev *rdev, char *buff)
{
	ssize_t status = 0;
	status = snprintf(buff, BP_STATUS_MAX_LENGTH, "%s\n",
			  bp_power_state_string(mdm_ctrl_get_bp_state()));

	return status;
}

static ssize_t mdm_user_command(struct radio_dev *rdev, char *post_strip)
{

	pr_info("%s: user command = %s\n", mdmctrl, post_strip);

	if (strcmp(post_strip, "shutdown") == 0)
		mdm6600_shutdown(NULL);
	else if (strcmp(post_strip, "powerup") == 0)
		mdm_ctrl_startup(bp_power_up_mode);
	else if (strcmp(post_strip, "bootmode_normal") == 0)
		bp_power_up_mode = MDM_CTRL_BP_PU_MODE_NORMAL;
	else if (strcmp(post_strip, "bootmode_flash") == 0)
		bp_power_up_mode = MDM_CTRL_BP_PU_MODE_FLASH;
	else if (strcmp(post_strip, "dump_log") == 0)
		mdm_ctrl_dump_log();
	else
		return -EINVAL;

	return 0;
}

static void bp_change_worker(struct work_struct *work)
{
	kobject_uevent(&radio_cdev.dev->kobj, KOBJ_CHANGE);
}

static void on_bp_change(int state, int status)
{
	pr_debug("%s: %d, %d\n", __func__, state, status);
	queue_work(working_queue, &bp_change_work);
}

static struct radio_dev radio_cdev = {
	.name = "mdm6600",
	.power_status = mdm_power_show,
	.status = mdm_status_show,
	.command = mdm_user_command,
};

static int __devinit mdm6600_probe(struct platform_device *pdev)
{
	struct mdm6600_agent_platform_data *pdata = pdev->dev.platform_data;

	dev_info(&pdev->dev, "mdm6600_probe");

	pr_debug("%s: radio_cdev = %p\n", __func__, &radio_cdev);

	working_queue = create_singlethread_workqueue("mdm6600_agent_wq");
	if (!working_queue) {
		dev_err(&pdev->dev, "Cannot create work queue.");
		return -1;
	}
	INIT_WORK(&bp_change_work, bp_change_worker);

	if (radio_dev_register(&radio_cdev)) {
		pr_err("%s: failed to register mdm6600_ctrl_agent device\n",
			__func__);
		goto fail;
	}

	/*
	 * Register for callbacks for BP events.
	 */
	if (pdata->mdm_ctrl_agent_register)
		pdata->mdm_ctrl_agent_register(on_bp_change);

	on_bp_change(0, 0);

	return 0;

fail:
	destroy_workqueue(working_queue);

	return -1;
}

static int __devexit mdm6600_remove(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "cleanup\n");

	radio_dev_unregister(&radio_cdev);

	if (working_queue)
		destroy_workqueue(working_queue);

	return 0;
}

static void __devexit mdm6600_shutdown(struct platform_device *pdev)
{
	pr_info("%s: Shutting down modem.", mdmctrl);
	mdm_ctrl_shutdown();
}

static struct platform_driver mdm6x00_ctrl_driver = {
	.probe = mdm6600_probe,
	.remove = __devexit_p(mdm6600_remove),
	.shutdown = __devexit_p(mdm6600_shutdown),
	.driver = {
		.name = "mdm6600_ctrl",
		.owner = THIS_MODULE,
	},
};

static int __init mdm6600_ctrl_init(void)
{
	printk(KERN_DEBUG "mdm6600_ctrl_init\n");
	return platform_driver_register(&mdm6x00_ctrl_driver);
}

static void __exit mdm6600_ctrl_exit(void)
{
	printk(KERN_DEBUG "mdm6600_ctrl_exit\n");
	platform_driver_unregister(&mdm6x00_ctrl_driver);
}

module_init(mdm6600_ctrl_init);
module_exit(mdm6600_ctrl_exit);

MODULE_AUTHOR("Motorola");
MODULE_DESCRIPTION("MDM6X00 Control Driver Agent");
MODULE_VERSION("1.1.4");
MODULE_LICENSE("GPL");
