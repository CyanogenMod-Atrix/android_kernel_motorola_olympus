/*
 * Copyright (C) 2009-2010 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#ifndef _LINUX_MDM_CTRL_H__
#define _LINUX_MDM_CTRL_H__

#ifdef __KERNEL__

/* Exported APIs */
extern int mdm_ctrl_startup(int pu_mode);
extern int mdm_ctrl_shutdown(void);
extern int mdm_ctrl_get_bp_state(void);
extern int mdm_ctrl_get_bp_status(void);
extern void mdm_ctrl_set_ap_status(int status);
extern void mdm_ctrl_dump_log(void);

#endif /* __KERNEL__ */


/* Bit mask of interruptible gpios */
enum {
	MASK_BP_RESOUT		= 0x0001,
};

enum {
	MDM_CTRL_BP_MODE_INVALID = 0,
	MDM_CTRL_BP_MODE_NORMAL,
	MDM_CTRL_BP_MODE_FLASH,
	MDM_CTRL_BP_MODE_UNKNOWN,
};

#define MDM_CTRL_MAGIC 0xEF

/* List of IOCtl commands */
#define MDM_CTRL_IOCTL_GET_BP_READY_AP		_IOR(MDM_CTRL_MAGIC, 0, int)
#define MDM_CTRL_IOCTL_GET_BP_READY2_AP		_IOR(MDM_CTRL_MAGIC, 1, int)
#define MDM_CTRL_IOCTL_GET_BP_RESOUT		_IOR(MDM_CTRL_MAGIC, 2, int)
#define MDM_CTRL_IOCTL_GET_BP_STATUS		_IOR(MDM_CTRL_MAGIC, 3, int)

#define MDM_CTRL_IOCTL_SET_BP_PWRON		_IOW(MDM_CTRL_MAGIC, 100, int)
#define MDM_CTRL_IOCTL_SET_BP_PSHOLD		_IOW(MDM_CTRL_MAGIC, 101, int)
#define MDM_CTRL_IOCTL_SET_BP_FLASH_EN		_IOW(MDM_CTRL_MAGIC, 102, int)
#define MDM_CTRL_IOCTL_SET_AP_STATUS		_IOW(MDM_CTRL_MAGIC, 103, int)
#define MDM_CTRL_IOCTL_SET_BP_RESIN		_IOW(MDM_CTRL_MAGIC, 104, int)
#define MDM_CTRL_IOCTL_SET_BP_BYPASS		_IOW(MDM_CTRL_MAGIC, 105, int)
#define MDM_CTRL_IOCTL_SET_BP_FLASH_MODE	_IOW(MDM_CTRL_MAGIC, 106, int)

#define MDM_CTRL_IOCTL_SET_INT_BP_READY_AP	_IOW(MDM_CTRL_MAGIC, 200, int)
#define MDM_CTRL_IOCTL_SET_INT_BP_READY2_AP	_IOW(MDM_CTRL_MAGIC, 201, int)
#define MDM_CTRL_IOCTL_SET_INT_BP_RESOUT	_IOW(MDM_CTRL_MAGIC, 202, int)
#define MDM_CTRL_IOCTL_SET_INT_BP_STATUS	_IOW(MDM_CTRL_MAGIC, 203, int)

#define MDM_CTRL_IOCTL_BP_STARTUP		_IOW(MDM_CTRL_MAGIC, 300, int)
#define MDM_CTRL_IOCTL_BP_SHUTDOWN		_IO(MDM_CTRL_MAGIC, 301)
#define MDM_CTRL_IOCTL_BP_RESET			_IO(MDM_CTRL_MAGIC, 302)

/* List of Interrupt Options */
#define MDM_CTRL_IRQ_RISING	0x01
#define MDM_CTRL_IRQ_FALLING	0x02

/* List of GPIO output pull options */
#define MDM_CTRL_GPIO_HIGH	1
#define MDM_CTRL_GPIO_LOW	0

/* BP status values */
#define MDM_CTRL_BP_STATUS_UNKNOWN	0x0
#define MDM_CTRL_BP_STATUS_PANIC_WAIT	0x1
#define MDM_CTRL_BP_STATUS_CORE_DUMP	0x2
#define MDM_CTRL_BP_STATUS_RDL		0x3
#define MDM_CTRL_BP_STATUS_AWAKE	0x4
#define MDM_CTRL_BP_STATUS_ASLEEP	0x5
#define MDM_CTRL_BP_STATUS_SHUTDOWN_ACK	0x6
#define MDM_CTRL_BP_STATUS_PANIC	0x7

/* AP status values */
#define MDM_CTRL_AP_STATUS_UNKNOWN	         0x0
#define MDM_CTRL_AP_STATUS_DATA_BYPASS       0x1
#define MDM_CTRL_AP_STATUS_FULL_BYPASS       0x2
#define MDM_CTRL_AP_STATUS_NO_BYPASS         0x3
#define MDM_CTRL_AP_STATUS_BP_SHUTDOWN       0x4
#define MDM_CTRL_AP_STATUS_NO_BYPASS_USB_IPC 0x5
#define MDM_CTRL_AP_STATUS_FLASH_MODE        0x6
#define MDM_CTRL_AP_STATUS_BP_PANIC_ACK      0x7
/* The maximum number of AP status values. */
#define MDM_CTRL_AP_STATUS_MAX               8

/* BP powerup modes */
#define MDM_CTRL_BP_PU_MODE_NORMAL	0x1
#define MDM_CTRL_BP_PU_MODE_FLASH	0x2

/* BP flash modes */
#define MDM_CTRL_BP_FLASH_MODE_NONE	0x0
#define MDM_CTRL_BP_FLASH_MODE_RSD	0x1
#define MDM_CTRL_BP_FLASH_MODE_QCOM	0x2

/* Exported APIs */
extern int mdm_ctrl_get_bp_status(void);
extern void mdm_ctrl_set_usb_ipc(bool on);

#endif  /* _LINUX_MDM_CTRL_H__ */
