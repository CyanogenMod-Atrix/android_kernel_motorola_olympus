/*
 * drivers/input/touchscreen/qtouch_obp_ts.c - driver for Quantum touch IC
 *
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009 Motorola, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Derived from the Motorola OBP touch driver.
 *
 */
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/earlysuspend.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/qtouch_obp_ts.h>
#include <linux/wakelock.h>
#include <linux/slab.h>
#include <asm/io.h>
#include <asm/uaccess.h>

/* Alternative resume methods */
/* use soft reset instead of power config */
//#define CONFIG_XMEGAT_USE_RESET_TO_RESUME 1
/* do hard reset before soft reset */
//#define CONFIG_XMEGAT_DO_HARD_RESET  1

/*
#define	USE_NVODM_STUFF	1
*/

#define IGNORE_CHECKSUM_MISMATCH

struct axis_map 
{
	int	key;
	int	x;
	int	y;
};

#define _BITMAP_LEN			BITS_TO_LONGS(QTM_OBP_MAX_OBJECT_NUM)
#define _NUM_FINGERS		10
struct qtouch_ts_data 
{
	struct i2c_client		*client;
	/* Use this mutex to protect access to i2c */
	struct mutex				i2c_lock;
	struct input_dev		*input_dev;
	struct work_struct		init_work;
	struct work_struct		work;
	struct qtouch_ts_platform_data	*pdata;
	struct coordinate_map		finger_data[_NUM_FINGERS];
	struct coordinate_map		prev_finger_data[_NUM_FINGERS];
	struct early_suspend		early_suspend;

	struct qtm_object		obj_tbl[QTM_OBP_MAX_OBJECT_NUM];
	struct	wake_lock		wLock;
	char	modeOfOperation;
	char	irqStatus;
	char	dlStatus;
	char	XferStatus;
	char	suspendMode;
	char	selfStatus;
	struct  selftest	selfStatusReport;
	bool	multiMode;

	unsigned long			obj_map[_BITMAP_LEN];

	uint32_t			last_keystate;
	uint32_t			eeprom_checksum;
	uint8_t			    checksum_cnt;
	int					x_delta;
	int					y_delta;

	bool				hw_init;	/* Flag indicating that hw initalization is in place */
	/* Note: The message buffer is reused for reading different messages.
	 * MUST enforce that there is no concurrent access to msg_buf. */
	uint8_t				msg_buf[20];
	int				msg_size;
	int				irqRest;
	int				irqInt;
	int					i2cNormAddr;
	int					i2cBLAddr;

	uint8_t				cal_check_flag;
	unsigned long		cal_timer;
};

struct qtm_id_info qtm_info;

struct qtouch_ts_data *tsGl;
struct qtouch_buttons	*buttons[BUTTON_MAX+1];

#ifdef CONFIG_HAS_EARLYSUSPEND
static void qtouch_ts_early_suspend(struct early_suspend *handler);
static void qtouch_ts_late_resume(struct early_suspend *handler);
#endif

static int qtouch_hw_init(struct qtouch_ts_data *ts);

extern unsigned int	MotorolaBootDispArgGet(int *);

static struct workqueue_struct *qtouch_ts_wq;
static irqreturn_t qtouch_ts_irq_handler(int irq, void *dev_id);

static int ignore_keyarray_touches = 0;
#define KEYARRAY_IGNORE_TIME (msecs_to_jiffies(100))

static	int	need2check4IC_problem;

static uint32_t qtouch_tsdebug;
module_param_named(tsdebug, qtouch_tsdebug, uint, 0664);

#define QTOUCH_INFO(args...) {qtouch_printk(qtouch_tsdebug, args);}
#define QTOUCH_INFO2(args...) {qtouch_printk((qtouch_tsdebug&2), args);}
#define QTOUCH_INFO4(args...) {qtouch_printk((qtouch_tsdebug&4), args);}
#define QTOUCH_ERR(args...) {qtouch_printk(0xff,args);}
static void qtouch_printk (int, char *, ...);

static	int	qtouch_ioctl_open(struct inode *inode, struct file *filp);
static	long qtouch_ioctl_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
static	int qtouch_ioctl_write(struct file *flip, const char __user *buf, size_t count, loff_t *f_pos );
static int qtouch_read(struct qtouch_ts_data *ts, void *buf, int buf_sz);

static uint8_t calibrate_chip(struct qtouch_ts_data *ts);
static uint8_t check_chip_calibration(struct qtouch_ts_data *ts);

struct	file_operations	qtouch_fops =
{
	.owner		= THIS_MODULE,
	.open		= qtouch_ioctl_open,
	.unlocked_ioctl	= qtouch_ioctl_ioctl,
	.write		= qtouch_ioctl_write,
};

static struct miscdevice qtouch_pf_driver = 
{
	.minor = MISC_DYNAMIC_MINOR,
	.name = "touchpad",
	.fops = &qtouch_fops,
};

static	char	dumpStr[100];
static void	qtouch_dump_block(const char *fn, char *block, int blSize)
{
	int	i;
	
	strlcpy(dumpStr,fn,20);
	strcat(dumpStr,": ");
	/* dump 20 digits per line. */
	for ( i = 0; i < blSize; i++ )
	{
		char tmp[5];
		if ( i > 0 && !(i % 20) )
		{
			strcat(dumpStr,"\n");
			QTOUCH_INFO(dumpStr);
			strlcpy(dumpStr,fn, 20);
			strcat(dumpStr,": ");
		}
			
		sprintf(tmp,"%02X ", block[i]);
		strcat(dumpStr,tmp);
	}
	strcat(dumpStr,"\n");
	QTOUCH_INFO(dumpStr);
	return;
}

static void clean_i2c(struct qtouch_ts_data *ts)
{
	int		ret;
	char	buf[5];
	char	clean;
	int		dCount = 0;

	QTOUCH_INFO("%s: Enter...\n",__func__);
	clean = FALSE;
	while ( !clean )
	{
		ret = qtouch_read(ts, (char *)buf, 1);
		if ( ret < 1 )
			clean = TRUE;
		if ( buf[0] == 0xff )
			clean = TRUE;
		dCount++;
	}
	QTOUCH_INFO("%s: Exit...\n",__func__);
	return;
}

static void qtouch_enable_irq(int irq)
{
	if ( !tsGl->irqStatus )
	{
		QTOUCH_INFO("%s: Enabling IRQ %d.\n", __func__, irq);
		enable_irq(irq);
		tsGl->irqStatus = TRUE;
	}
	else
		QTOUCH_INFO("%s: IRQ should be enabled by now.\n", __func__);
}

static void qtouch_disable_irq(int irq)
{
	if ( tsGl->irqStatus )
	{
		QTOUCH_INFO("%s: Disabling IRQ %d.\n", __func__, irq);
		tsGl->irqStatus = FALSE;
		disable_irq(irq);
	}
	else
		QTOUCH_INFO("%s: IRQ should be disabled by now.\n", __func__);
		
}

static irqreturn_t qtouch_ts_irq_handler(int irq, void *dev_id)
{
	struct qtouch_ts_data *ts = dev_id;

	QTOUCH_INFO("%s: Enter...\n",__func__);

	queue_work(qtouch_ts_wq, &ts->work);

	QTOUCH_INFO("%s: Exit...\n",__func__);
	return 0; 
}

static int qtouch_write(struct qtouch_ts_data *ts, void *buf, int buf_sz)
{
	int retries = 10;
	int ret;

	mutex_lock(&ts->i2c_lock);

	do {
		ret = i2c_master_send(ts->client, (char *)buf, buf_sz);
	} while ((ret < buf_sz) && (--retries > 0));

	if (ret < 0)
	{
		QTOUCH_INFO("%s: Error while trying to write %d bytes\n", __func__, buf_sz);
	}
	else if (ret != buf_sz) 
	{
		QTOUCH_INFO("%s: Write %d bytes, expected %d\n", __func__, ret, buf_sz);
		ret = -EIO;
	}

	mutex_unlock(&ts->i2c_lock);

	return ret;
}

static int qtouch_set_addr(struct qtouch_ts_data *ts, uint16_t addr)
{
	int ret;

	/* Note: addr on the wire is LSB first */
	ret = qtouch_write(ts, (char *)&addr, sizeof(uint16_t));
	if (ret < 0)
		QTOUCH_INFO("%s: Can't send obp addr 0x%4x\n", __func__, addr);

	return ret >= 0 ? 0 : ret;
}

static int qtouch_read(struct qtouch_ts_data *ts, void *buf, int buf_sz)
{
	int retries = 10;
	int ret;

	mutex_lock(&ts->i2c_lock);

	QTOUCH_INFO("%s: Read %d bytes into address 0x%X\n", __func__, buf_sz, buf );
	do 
	{
		memset(buf, 0, buf_sz);
		ret = i2c_master_recv(ts->client, (char *)buf, buf_sz);
	} while ((ret < 0) && (--retries > 0));

	if (ret < 0)
	{
		QTOUCH_ERR("%s: Error while trying to read %d bytes\n", __func__,
			buf_sz);
	}
	else if (ret != buf_sz) 
	{
		QTOUCH_INFO("%s: Read %d bytes, expected %d\n", __func__,
			ret, buf_sz);
		ret = -EIO;
	}
	else
		QTOUCH_INFO("%s: Read %d bytes\n", __func__, buf_sz );

	mutex_unlock(&ts->i2c_lock);

	return ret >= 0 ? 0 : ret;
}

static int qtouch_read_addr(struct qtouch_ts_data *ts, uint16_t addr,
			    void *buf, int buf_sz)
{
	int ret;

	ret = qtouch_set_addr(ts, addr);
	if (ret != 0)
		return ret;

	return qtouch_read(ts, buf, buf_sz);
}

static struct qtm_obj_message *qtouch_read_msg(struct qtouch_ts_data *ts)
{
	int ret;

	ret = qtouch_read(ts, ts->msg_buf, ts->msg_size);
	if (!ret)
		return (struct qtm_obj_message *)ts->msg_buf;
	return NULL;
}

static int qtouch_write_addr(struct qtouch_ts_data *ts, uint16_t addr,
			     void *buf, int buf_sz)
{
	int ret;
	uint8_t write_buf[128];

	if (buf_sz + sizeof(uint16_t) > sizeof(write_buf)) 
	{
		QTOUCH_ERR("%s: Buffer too large (%d)\n", __func__, buf_sz);
		return -EINVAL;
	}

	memcpy(write_buf, (void *)&addr, sizeof(addr));
	memcpy((void *)write_buf + sizeof(addr), buf, buf_sz);

	ret = qtouch_write(ts, write_buf, buf_sz + sizeof(addr));

	if (ret < 0) 
	{
		QTOUCH_ERR("%s: Could not write %d bytes.\n", __func__, buf_sz);
		return ret;
	}

	return 0;
}

static uint16_t calc_csum(uint16_t curr_sum, void *_buf, int buf_sz)
{
	uint8_t *buf = _buf;
	uint32_t new_sum;
	int i;

	while (buf_sz-- > 0) 
	{
		new_sum = (((uint32_t) curr_sum) << 8) | *(buf++);
		for (i = 0; i < 8; ++i) 
		{
			if (new_sum & 0x800000)
				new_sum ^= 0x800500;
			new_sum <<= 1;
		}
		curr_sum = ((uint32_t) new_sum >> 8) & 0xffff;
	}

	return curr_sum;
}

static inline struct qtm_object *find_obj(struct qtouch_ts_data *ts, int id)
{
	return &ts->obj_tbl[id];
}

static struct qtm_object *create_obj(struct qtouch_ts_data *ts,
				     struct qtm_obj_entry *entry)
{
	struct qtm_object *obj;

	obj = &ts->obj_tbl[entry->type];
	memcpy(&obj->entry, entry, sizeof(*entry));
	set_bit(entry->type, ts->obj_map);

	return obj;
}

static struct qtm_object *find_object_rid(struct qtouch_ts_data *ts, int rid)
{
	int i;

	for_each_set_bit(i, ts->obj_map, QTM_OBP_MAX_OBJECT_NUM) {
		struct qtm_object *obj = &ts->obj_tbl[i];

		if ((rid >= obj->report_id_min) && (rid <= obj->report_id_max))
			return obj;
	}

	return NULL;
}

static int qtouch_force_reset(struct qtouch_ts_data *ts, uint8_t sw_reset)
{
	struct qtm_object *obj;
	uint16_t addr;
	uint8_t val;
	int ret;

	if (ts->pdata->hw_reset && !sw_reset) 
	{
		QTOUCH_INFO("%s: Forcing HW reset\n", __func__);
		QTOUCH_INFO("%s: Taking device out of reset\n", __func__);
		ts->pdata->hw_reset();
	}
	else if (sw_reset) 
	{
		QTOUCH_INFO("%s: Forcing SW reset\n", __func__);
		obj = find_obj(ts, QTM_OBJ_GEN_CMD_PROC);
		addr = obj->entry.addr + offsetof(struct qtm_gen_cmd_proc, reset);
		val = 1;
		ret = qtouch_write_addr(ts, addr, &val, 1);
		if (ret)
			QTOUCH_ERR("%s: Unable to send the reset msg\n", __func__);
	}
	return 0;
}

static int qtouch_force_calibration(struct qtouch_ts_data *ts)
{
	struct qtm_object *obj;
	uint16_t addr;
	uint8_t val;
	int ret;

	QTOUCH_INFO("%s: Forcing calibration\n", __func__);

	obj = find_obj(ts, QTM_OBJ_GEN_CMD_PROC);

	addr = obj->entry.addr + offsetof(struct qtm_gen_cmd_proc, calibrate);
	val = 1;
	ret = qtouch_write_addr(ts, addr, &val, 1);
	if (ret)
		QTOUCH_ERR("%s: Unable to send the calibrate message\n", __func__);
	return ret;
}

#if 0
static int qtouch_force_backupnv(struct qtouch_ts_data *ts)
{
	struct qtm_object *obj;
	uint16_t addr;
	uint8_t val;
	int ret;

	QTOUCH_INFO("%s: Forcing Backup NV\n", __func__);

	obj = find_obj(ts, QTM_OBJ_GEN_CMD_PROC);

	addr = obj->entry.addr + offsetof(struct qtm_gen_cmd_proc, backupnv);
	val = 0x55;
	ret = qtouch_write_addr(ts, addr, &val, 1);
	if (ret)
		QTOUCH_ERR("%s: Unable to send the Backup NV request message\n",
					__func__);
	return ret;
}
#endif

static int	mode_codes[5] = 
{	
	/* 16x14 */ 224,
	/* 17x13 */ 221,
	/* 18x12 */ 216,
	/* 19x11 */ 209,
	/* 20x10 */ 200,
};

static int qtouch_get_channels_count(struct qtouch_ts_data *ts)
{
	struct qtm_object *obj;
	struct qtm_spt_cte_config_cfg	msg;
	int ret;

	QTOUCH_INFO("%s: Getting number of channels\n", __func__);

	obj = find_obj(ts, QTM_OBJ_SPT_CTE_CONFIG);
	ret = qtouch_read_addr(ts, obj->entry.addr, &msg, sizeof(msg) );
	if (!ret)
	{
		QTOUCH_INFO("%s: CTECONFIG result: mode = 0x%02X\n", __func__, msg.mode);
		if ( msg.mode >= 0 && msg.mode < 5 )
			ret = mode_codes[msg.mode];
		else
			ret = -1;
	}
	return ret;
}

static int qtouch_get_sens_data(struct qtouch_ts_data *ts, struct qtm_debug_diagnostics *dbgMsg, char mode )
{
	struct qtm_object *obj;
	struct	qtm_gen_cmd_proc	msg;
	uint16_t addr;
	int ret;

	QTOUCH_INFO("%s: Getting diagnostics\n", __func__);

	obj = find_obj(ts, QTM_OBJ_GEN_CMD_PROC);
	addr = obj->entry.addr;
	if ( mode == 0 )
	{
		msg.diagnostic = QTOUCH_DIAG_REFERENCE_MODE;
	}
	else
	{
		msg.diagnostic = QTOUCH_DIAG_PAGE_DOWN;
	}
	ret = qtouch_write_addr(ts, addr, &(msg), sizeof(msg));
	if (ret)
	{
		QTOUCH_ERR("%s: Unable to initiate diagnostics\n", __func__);
	}

	/* Get the diagnostics information */
	obj = find_obj(ts, QTM_OBJ_DEBUG_DIAGNOSTICS_T37);
	ret = qtouch_read_addr(ts, obj->entry.addr, dbgMsg, sizeof(*dbgMsg) );
	if (!ret)
	{
		QTOUCH_INFO("%s: DEBUG_DIAG result: mode = 0x%02X, page:\n", __func__, dbgMsg->mode, dbgMsg->page);
	}
	return ret;
}


static int qtouch_force_selftest(struct qtouch_ts_data *ts, char mode)
{
	struct qtm_object *obj;
	struct	qtm_spt_self_test_rslt	msg;
	uint16_t addr;
	int	i;
	int ret;

	QTOUCH_INFO("%s: Forcing self-test\n", __func__);

	obj = find_obj(ts, QTM_OBJ_SPT_SELF_TEST);

	addr = obj->entry.addr;
	if ( mode )
	{
		ts->pdata->self_test_cfg.ctrl = 0x03;
		ts->pdata->self_test_cfg.command = 0xFE;
		ts->selfStatus = QTOUCH_SELFTEST_INPROGRESS;
		memset((char *) &(ts->selfStatusReport), 0, sizeof(struct selftest));
	}
	else
	{
		ts->pdata->self_test_cfg.ctrl = 0x03;
		ts->pdata->self_test_cfg.command = 0xFE;
		ts->selfStatus = QTOUCH_SELFTEST_DONE;
	}
	ret = qtouch_write_addr(ts, addr, &(ts->pdata->self_test_cfg), sizeof(ts->pdata->self_test_cfg));
	if (ret)
	{
		QTOUCH_ERR("%s: Unable to send the request for self test\n", __func__);
	}
	else
	{
		obj = find_obj(ts, QTM_OBJ_GEN_MSG_PROC);
		ret = qtouch_read_addr(ts, obj->entry.addr, &msg, sizeof(msg) );
		if (!ret)
		{
			QTOUCH_INFO("%s: SELFTEST result: status = 0x%02X\n", __func__, msg.status);
			ts->selfStatusReport.status = msg.status;
			for ( i = 0; i < 5; i++ )
				ts->selfStatusReport.info[i] = msg.info[i];
		}
	}
	return ret;
}

static int qtouch_force_bootloaderMode(struct qtouch_ts_data *ts)
{
	struct qtm_object *obj;
	uint16_t addr;
	uint8_t val;
	int ret;

	QTOUCH_INFO("%s: Forcing bootloader mode\n", __func__);

	obj = find_obj(ts, QTM_OBJ_GEN_CMD_PROC);

	addr = obj->entry.addr + offsetof(struct qtm_gen_cmd_proc, reset);
	val = 0xA5;
	ret = qtouch_write_addr(ts, addr, &val, 1);
	if (ret)
		QTOUCH_ERR("%s: Unable to send the force bootloader message\n", __func__);
	return ret;
}

#undef min
#define min(a, b) (((a) < (b)) ? (a) : (b))
static int qtouch_power_config(struct qtouch_ts_data *ts, int on)
{
	struct qtm_gen_power_cfg pwr_cfg;
	struct qtm_object *obj;

	if (!on) 
	{
		/* go to standby mode */
		pwr_cfg.idle_acq_int = 0;
		pwr_cfg.active_acq_int = 0;
	} 
	else 
	{
		pwr_cfg.idle_acq_int = ts->pdata->power_cfg.idle_acq_int;
		pwr_cfg.active_acq_int = ts->pdata->power_cfg.active_acq_int;
	}

	pwr_cfg.active_idle_to = ts->pdata->power_cfg.active_idle_to;

	obj = find_obj(ts, QTM_OBJ_GEN_PWR_CONF);
	return qtouch_write_addr(ts, obj->entry.addr, &pwr_cfg,
				 min(sizeof(pwr_cfg), obj->entry.size));
}

/* Apply the configuration provided in the platform_data to the hardware */
static int qtouch_hw_init(struct qtouch_ts_data *ts)
{
	struct qtm_object *obj;
	int i;
	int ret;
	uint16_t adj_addr;

	QTOUCH_ERR("%s: Doing hw init\n", __func__);
	ts->hw_init = TRUE;

	/* take the IC out of suspend */
	qtouch_power_config(ts, TRUE);

	/* configure the acquisition object. */
	obj = find_obj(ts, QTM_OBJ_GEN_ACQUIRE_CONF);
	ret = qtouch_write_addr(ts, obj->entry.addr, &ts->pdata->acquire_cfg,
				min(sizeof(ts->pdata->acquire_cfg),
				    obj->entry.size));
	if (ret != 0) 
	{
		QTOUCH_ERR("%s: Can't write acquisition config\n", __func__);
		goto failed2write;
	}

	/* The multitouch and keyarray objects have very similar memory
	 * layout, but are just different enough where we basically have to
	 * repeat the same code */

	/* configure the multi-touch object. */
	obj = find_obj(ts, QTM_OBJ_TOUCH_MULTI);
	if (obj && obj->entry.num_inst > 0) 
	{
		struct qtm_touch_multi_cfg cfg;
		memcpy(&cfg, &ts->pdata->multi_touch_cfg, sizeof(cfg));
		if (ts->pdata->flags & QTOUCH_USE_MULTITOUCH)
			cfg.ctrl |= (1 << 1) | (1 << 0); /* reporten | enable */
		else
			cfg.ctrl = 0;
		ret = qtouch_write_addr(ts, obj->entry.addr, &cfg,
					min(sizeof(cfg), obj->entry.size));
		if (ret != 0) 
		{
			QTOUCH_ERR("%s: Can't write multi-touch config\n", __func__);
			goto failed2write;
		}
	}

	/* configure the key-array object. */
	obj = find_obj(ts, QTM_OBJ_TOUCH_KEYARRAY);
	if (obj && obj->entry.num_inst > 0) 
	{
		struct qtm_touch_keyarray_cfg cfg;
		for (i = 0; i < obj->entry.num_inst; i++) 
		{
			if (i > (ts->pdata->key_array.num_keys - 1)) 
			{
				QTOUCH_INFO("%s: No entry key instance.\n", __func__);
				memset(&cfg, 0, sizeof(cfg));
			} 
			else if (ts->pdata->flags & QTOUCH_USE_KEYARRAY) 
			{
				memcpy(&cfg, &ts->pdata->key_array.cfg[i], sizeof(cfg));
				cfg.ctrl |= (1 << 1) | (1 << 0); /* reporten | enable */
			}
			else
				memset(&cfg, 0, sizeof(cfg));

			adj_addr = obj->entry.addr + ((obj->entry.size + 1) * i);
			ret = qtouch_write_addr(ts, adj_addr, &cfg, min(sizeof(cfg), obj->entry.size));
			if (ret != 0) 
			{
				QTOUCH_ERR("%s: Can't write keyarray config\n", __func__);
				goto failed2write;
			}
		}
	}

	/* configure the signal filter */
	obj = find_obj(ts, QTM_OBJ_PROCG_SIG_FILTER);
	if (obj && obj->entry.num_inst > 0) 
	{
		ret = qtouch_write_addr(ts, obj->entry.addr,
					&ts->pdata->sig_filter_cfg,
					min(sizeof(ts->pdata->sig_filter_cfg),
					    obj->entry.size));
		if (ret != 0) 
		{
			QTOUCH_ERR("%s: Can't write signal filter config\n", __func__);
			goto failed2write;
		}
	}

	/* configure the linearization table */
	obj = find_obj(ts, QTM_OBJ_PROCI_LINEAR_TBL);
	if (obj && obj->entry.num_inst > 0) 
	{
		ret = qtouch_write_addr(ts, obj->entry.addr,
					&ts->pdata->linear_tbl_cfg,
					min(sizeof(ts->pdata->linear_tbl_cfg),
					    obj->entry.size));
		if (ret != 0) 
		{
			QTOUCH_ERR("%s: Can't write linear table config\n", __func__);
			goto failed2write;
		}
	}

	/* Motorola - BEGIN - qa2113 - 11/18/2009 - XmegaT firmware 1.5 support */
	/* configure the comms configuration */
	obj = find_obj(ts, QTM_OBJ_SPT_COM_CONFIG);
	if (obj && obj->entry.num_inst > 0) 
	{
		ret = qtouch_write_addr(ts, obj->entry.addr,
					&ts->pdata->comms_config_cfg,
					min(sizeof(ts->pdata->comms_config_cfg),
					    obj->entry.size));
		if (ret != 0) 
		{
			QTOUCH_ERR("%s: Can't write the comms configuration config\n", __func__);
			goto failed2write;
		}
	}
	/* Motorola - END - qa2113 - 11/18/2009 - XmegaT firmware 1.5 support */

	/* configure the GPIO PWM support */
	obj = find_obj(ts, QTM_OBJ_SPT_GPIO_PWM);
	if (obj && obj->entry.num_inst > 0) 
	{
		ret = qtouch_write_addr(ts, obj->entry.addr,
					&ts->pdata->gpio_pwm_cfg,
					min(sizeof(ts->pdata->gpio_pwm_cfg),
					    obj->entry.size));
		if (ret != 0) 
		{
			QTOUCH_ERR("%s: Can't write the GPIO PWM config\n", __func__);
			goto failed2write;
		}
	}

	/* configure the grip suppression table */
	obj = find_obj(ts, QTM_OBJ_PROCI_GRIPFACESUPPRESSION);
	if (obj && obj->entry.num_inst > 0) 
	{
		ret = qtouch_write_addr(ts, obj->entry.addr,
					&ts->pdata->grip_suppression_cfg,
					min(sizeof(ts->pdata->grip_suppression_cfg),
					    obj->entry.size));
		if (ret != 0) 
		{
			QTOUCH_ERR("%s: Can't write the grip suppression config\n", __func__);
			goto failed2write;
		}
	}

	/* configure noise suppression */
	obj = find_obj(ts, QTM_OBJ_PROCG_NOISE_SUPPRESSION);
	if (obj && obj->entry.num_inst > 0) 
	{
		ret = qtouch_write_addr(ts, obj->entry.addr,
					&ts->pdata->noise_suppression_cfg,
					min(sizeof(ts->pdata->noise_suppression_cfg),
					    obj->entry.size));
		if (ret != 0) 
		{
			QTOUCH_ERR("%s: Can't write the noise suppression config\n", __func__);
			goto failed2write;
		}
	}

	/* configure the touch proximity sensor */
	obj = find_obj(ts, QTM_OBJ_TOUCH_PROXIMITY);
	if (obj && obj->entry.num_inst > 0) 
	{
		ret = qtouch_write_addr(ts, obj->entry.addr,
					&ts->pdata->touch_proximity_cfg,
					min(sizeof(ts->pdata->touch_proximity_cfg),
					    obj->entry.size));
		if (ret != 0) 
		{
			QTOUCH_ERR("%s: Can't write the touch proximity config\n", __func__);
			goto failed2write;
		}
	}
	
	/* configure the one touch gesture processor */
	obj = find_obj(ts, QTM_OBJ_PROCI_ONE_TOUCH_GESTURE_PROC);
	if (obj && obj->entry.num_inst > 0) 
	{
		ret = qtouch_write_addr(ts, obj->entry.addr,
					&ts->pdata->one_touch_gesture_proc_cfg,
					min(sizeof(ts->pdata->one_touch_gesture_proc_cfg),
					    obj->entry.size));
		if (ret != 0) 
		{
			QTOUCH_ERR("%s: Can't write the one touch gesture processor config\n", __func__);
			goto failed2write;
		}
	}

	/* configure self test */
	obj = find_obj(ts, QTM_OBJ_SPT_SELF_TEST);
	if (obj && obj->entry.num_inst > 0) 
	{
		ret = qtouch_write_addr(ts, obj->entry.addr,
					&ts->pdata->self_test_cfg,
					min(sizeof(ts->pdata->self_test_cfg),
					    obj->entry.size));
		if (ret != 0) 
		{
			QTOUCH_ERR("%s: Can't write the self test config\n", __func__);
			goto failed2write;
		}
	}

	/* configure the two touch gesture processor */
	obj = find_obj(ts, QTM_OBJ_PROCI_TWO_TOUCH_GESTURE_PROC);
	if (obj && obj->entry.num_inst > 0) 
	{
		ret = qtouch_write_addr(ts, obj->entry.addr,
					&ts->pdata->two_touch_gesture_proc_cfg,
					min(sizeof(ts->pdata->two_touch_gesture_proc_cfg),
					    obj->entry.size));
		if (ret != 0) 
		{
			QTOUCH_ERR("%s: Can't write the two touch gesture processor config\n", __func__);
			goto failed2write;
		}
	}

	/* configure the capacitive touch engine  */
	obj = find_obj(ts, QTM_OBJ_SPT_CTE_CONFIG);
	if (obj && obj->entry.num_inst > 0) 
	{
		ret = qtouch_write_addr(ts, obj->entry.addr,
					&ts->pdata->cte_config_cfg,
					min(sizeof(ts->pdata->cte_config_cfg),
					    obj->entry.size));
		if (ret != 0) 
		{
			QTOUCH_ERR("%s: Can't write the capacitive touch engine config\n", __func__);
			goto failed2write;
		}
	}

	/* configure the noise suppression table */
	obj = find_obj(ts, QTM_OBJ_NOISESUPPRESSION_1);
	if (obj && obj->entry.num_inst > 0) 
	{
		ret = qtouch_write_addr(ts, obj->entry.addr,
					&ts->pdata->noise1_suppression_cfg,
					min(sizeof(ts->pdata->noise1_suppression_cfg),
					    obj->entry.size));
		if (ret != 0) 
		{
			QTOUCH_ERR("%s: Can't write the noise suppression config\n", __func__);
			goto failed2write;
		}
	}

	ret = qtouch_force_calibration(ts);
	if (ret != 0) 
	{
		QTOUCH_ERR("%s: Unable to recalibrate after reset\n", __func__);
		goto failed2write;
	}

	/* Write the settings into nvram, if needed */
	if (ts->pdata->flags & QTOUCH_CFG_BACKUPNV) 
	{
		uint8_t val;
		uint16_t addr;

		obj = find_obj(ts, QTM_OBJ_GEN_CMD_PROC);
		addr = obj->entry.addr + offsetof(struct qtm_gen_cmd_proc,
						  backupnv);
		val = 0x55;
		ret = qtouch_write_addr(ts, addr, &val, 1);
		if (ret != 0) 
		{
			QTOUCH_ERR("%s: Can't backup nvram settings\n", __func__);
			goto failed2write;
		}
		/* Since the IC does not indicate that has completed the
		backup place a hard wait here.  If we communicate with the
		IC during backup the EEPROM may be corrupted */

		msleep(500);
	}

	/* If debugging, read back and print all settings */
	if (qtouch_tsdebug) 
	{
		int object;
		int size;
		uint8_t *data_buff;
		int byte;
		int msg_bytes;
		int msg_location;
		char *msg;

		msg = kmalloc(1024, GFP_KERNEL);
		if (msg != NULL) 
		{
			for (object = 7; object < QTM_OBP_MAX_OBJECT_NUM; object++)
			{
				size = ts->obj_tbl[object].entry.size * ts->obj_tbl[object].entry.num_inst;
				if (size != 0) 
				{
					data_buff = kmalloc(size, GFP_KERNEL);
					if (data_buff == NULL) 
					{
						QTOUCH_ERR("%s: Object %d: Malloc failed\n", __func__, object);
						continue;
					}

					qtouch_read_addr(ts,
					                 ts->obj_tbl[object].entry.addr,
					                 (void *)data_buff, size);

					msg_location = sprintf(msg, "%s: Object %d:", __func__, object);
					for (byte=0; byte < size; byte++) 
					{
						msg_bytes = snprintf((msg + msg_location),
						                    (1024 - msg_location),
						                    " 0x%02x",
						                    *(data_buff + byte));
						msg_location += msg_bytes;
						if (msg_location >= 1024)
							break;
					}
					if (msg_location < 1024) 
					{
						QTOUCH_INFO("%s\n", msg);
					} 
					else 
					{
						QTOUCH_INFO("%s:  Object %d: String overflow\n", __func__, object);
					}
	
					kfree (data_buff);
				}
			}
			kfree (msg);
		}
		qtouch_set_addr(ts, ts->obj_tbl[QTM_OBJ_GEN_MSG_PROC].entry.addr);
	}

	/* reset the address pointer */
	ret = qtouch_set_addr(ts, ts->obj_tbl[QTM_OBJ_GEN_MSG_PROC].entry.addr);
	if (ret != 0) 
	{
		QTOUCH_ERR("%s: Unable to reset address pointer after reset\n", __func__);
		goto failed2write;
	}
failed2write:
	clean_i2c(ts);
	ts->hw_init = FALSE;
	return 0;
}

static int do_cmd_selftest_msg(struct qtouch_ts_data *ts, struct qtm_object *obj, void *_msg)
{
	struct qtm_spt_self_test_rslt *msg = _msg;
	int	i;
	
	ts->selfStatusReport.status = msg->status;
	for ( i = 0; i < 5; i++ )
		ts->selfStatusReport.info[i] = msg->info[i];
	ts->selfStatus = QTOUCH_SELFTEST_DONE;
	return 0;
}

/* Handles a message from the command processor object. */
static int do_cmd_proc_msg(struct qtouch_ts_data *ts, struct qtm_object *obj, void *_msg)
{
	struct qtm_cmd_proc_msg *msg = _msg;
	int ret = 0;
	int hw_reset = 0;
	uint32_t checksum = 0x0;

	if (msg->status & QTM_CMD_PROC_STATUS_RESET) 
	{
		QTOUCH_INFO("%s:EEPROM checksum is 0x%02X%02X%02X cnt %i\n",
				__func__, msg->checksum[2], msg->checksum[1],
				msg->checksum[0], ts->checksum_cnt);

		checksum = (msg->checksum[0] |
			(msg->checksum[1] << BITS_PER_BYTE) |
			(msg->checksum[2] << (BITS_PER_BYTE << 1)));

		checksum = __le32_to_cpu(checksum);
		if (checksum != ts->eeprom_checksum)
		{
			if (ts->checksum_cnt > 2) 
			{
				/* Assume the checksum is what it is, cannot
				disable the touch screen so set the checksum*/
				ts->eeprom_checksum = checksum;
				ts->checksum_cnt = 0;
			} 
			else 
			{
				ret = qtouch_hw_init(ts);
				if (ret != 0)
					QTOUCH_ERR("%s:Cannot init the touch IC\n",
						   __func__);
				hw_reset = 1;
				ts->checksum_cnt++;
			}
		}

		calibrate_chip(ts);

		QTOUCH_INFO("%s: Reset done.\n", __func__);
	}

	if (msg->status & QTM_CMD_PROC_STATUS_CAL) 
	{
		ts->cal_check_flag = 1;
		ts->cal_timer = 0;
		QTOUCH_INFO("%s: Self-calibration started.\n", __func__);
	}


	if (msg->status & QTM_CMD_PROC_STATUS_OFL)
		QTOUCH_ERR("%s: Acquisition cycle length overflow\n", __func__);

	if (msg->status & QTM_CMD_PROC_STATUS_SIGERR)
		QTOUCH_ERR("%s: Acquisition error\n", __func__);

	if (msg->status & QTM_CMD_PROC_STATUS_CFGERR) 
	{
		QTOUCH_ERR("%s: Configuration error\n", __func__);
	}
	/* Check the EEPROM checksum.  An ESD event may cause
	the checksum to change during operation so we need to
	reprogram the EEPROM and reset the IC */
	if (ts->pdata->flags & QTOUCH_EEPROM_CHECKSUM) 
	{
		QTOUCH_INFO("%s:EEPROM checksum is 0x%02X%02X%02X  cnt %i \n",
			__func__, msg->checksum[2], msg->checksum[1],
			msg->checksum[0], ts->checksum_cnt);

		checksum = (msg->checksum[0] |
			(msg->checksum[1] << BITS_PER_BYTE) |
			(msg->checksum[2] << (BITS_PER_BYTE << 1)));

		if (checksum != ts->eeprom_checksum) 
		{
			if (qtouch_tsdebug)
				pr_info("%s:EEPROM checksum is 0x%08X cnt %i\n",
					__func__, checksum, ts->checksum_cnt);

			if (ts->checksum_cnt > 2) 
			{
				/* Assume the checksum is what it is, cannot
				disable the touch screen so set the checksum*/
				ts->eeprom_checksum = checksum;
				ts->checksum_cnt = 0;
			}
			else 
			{
				if (!hw_reset) 
				{
					qtouch_force_reset(ts, FALSE);
					ts->checksum_cnt++;
				}
			}
		}
	}
	return ret;
}

/* Handles a message from a multi-touch object. */
static int do_touch_multi_msg(struct qtouch_ts_data *ts, struct qtm_object *obj,
			      void *_msg)
{
	struct qtm_touch_multi_msg *msg = _msg;
	int i;
	int x;
	int y;
	int pressure;
	int width;
	uint32_t finger;
	int down;
	int btnFlag;
	int stuckFinger;
	int num_fingers_down;

	QTOUCH_INFO4("%s: dump of arrived multitouch message: \n", __func__);
	QTOUCH_INFO4("\treport_id: %d\n",msg->report_id);
	QTOUCH_INFO4("\tstatus: 0x%x\n",msg->status);
	QTOUCH_INFO4("\txpos_msb: %d\n",msg->xpos_msb);
	QTOUCH_INFO4("\typos_msb: %d\n",msg->ypos_msb);
	QTOUCH_INFO4("\txypos_lsb: %d\n",msg->xypos_lsb);
	QTOUCH_INFO4("\ttouch_area: %d\n",msg->touch_area);
	QTOUCH_INFO4("\ttouch_amp: %d\n",msg->touch_amp);
	QTOUCH_INFO4("\ttouch_vect: 0x%x\n",msg->touch_vect);

	QTOUCH_INFO4("\tObject info: report_id_min: %d\n", obj->report_id_min);
	finger = msg->report_id - obj->report_id_min;
	QTOUCH_INFO4("\tFinger: %d\n", finger);
	if (finger >= ts->pdata->multi_touch_cfg.num_touch)
		return 0;

	/* x/y are 10bit values, with bottom 2 bits inside the xypos_lsb */
	x = (msg->xpos_msb << 2) | ((msg->xypos_lsb >> 6) & 0x3);
	y = (msg->ypos_msb << 2) | ((msg->xypos_lsb >> 2) & 0x3);
	width = msg->touch_area;
	pressure = msg->touch_amp;

	down = !(msg->status & (QTM_TOUCH_MULTI_STATUS_RELEASE |
		 QTM_TOUCH_MULTI_STATUS_SUPPRESS));

	/*
	 * IC may report old finger 2 as new finger 2 after lift and down of a
	 * finger 1. That is a condition under which we have to generate a
	 * liftoff for the finger 2. Some strange bug on IC.
	 */
	stuckFinger = FALSE;
	if (need2check4IC_problem)
	{
		if (finger == 1 && down  &&
			ts->finger_data[finger].x_data == x &&
			ts->finger_data[finger].y_data == y)
		{
			QTOUCH_ERR("%s: Detected stuck finger!\n",
					__func__);
			stuckFinger = TRUE;
		}
	}

	if (ts->pdata->flags & QTOUCH_FLIP_X)
		x = (ts->pdata->abs_max_x-1)-x;

	if (ts->pdata->flags & QTOUCH_FLIP_Y)
		y = (ts->pdata->abs_max_y-1)-y;

	if (ts->pdata->flags & QTOUCH_SWAP_XY)
		swap(x, y);


	if (finger >= _NUM_FINGERS) 
	{
		QTOUCH_ERR("%s: Invalid finger number %dd\n", __func__, finger);
		return 1;
	}

	QTOUCH_INFO2("%s: stat=%02x, f=%d x=%d y=%d p=%d w=%d, down=%d\n", __func__,
			msg->status, finger, x, y, pressure, width, down);

	/* The chip may report erroneous points way
	beyond what a user could possibly perform so we filter
	these out */
	if (ts->finger_data[finger].down &&
		(abs(ts->finger_data[finger].x_data - x) > ts->x_delta ||
		abs(ts->finger_data[finger].y_data - y) > ts->y_delta))
	{
		down = 0;
		QTOUCH_INFO2("%s: x0 %i x1 %i y0 %i y1 %i\n",
				__func__,
				ts->finger_data[finger].x_data, x,
				ts->finger_data[finger].y_data, y);
	} 
	else 
	{
		ts->finger_data[finger].x_data = x;
		ts->finger_data[finger].y_data = y;
		ts->finger_data[finger].w_data = width;
		ts->finger_data[finger].vector = msg->touch_vect;
		ts->finger_data[finger].down = 1;
	}

	/* The touch IC will not give back a pressure of zero
	   so send a 0 when a liftoff is produced */
	ts->finger_data[finger].down = down;
	if (!down) 
	{
		ts->finger_data[finger].z_data = 0;
		/* this will cause check to be performed on the next point */
		if (finger == 0)
			need2check4IC_problem = TRUE;
		else
			need2check4IC_problem = FALSE;
	} 
	else 
	{
		ts->finger_data[finger].z_data = msg->touch_amp;
		/*
		 * No need to do check on the next point, since it's only
		 * important after a lift
		 */
		need2check4IC_problem = FALSE;
	}

	/* check to make sure user is not pressing the buttons area */
	btnFlag = -1;
	if ( ts->pdata->buttons_count > 0 )
	{
		for ( i = 0; i < _NUM_FINGERS && btnFlag < 0; i++ )
		{
			/* All buttons have the same Y value */
			if ( ts->finger_data[i].y_data > buttons[0]->minY )
				btnFlag = i;
		}
		QTOUCH_INFO4("%s: btnFlg = %d\n", __func__, btnFlag);
	}
#if 0
	if ( btnFlag < 0 )
	{
		if ( ts->multiMode )
		{
			for (i = 0;
				i < ts->pdata->multi_touch_cfg.num_touch;
				i++)
			{
				if ( ts->finger_data[i].down == 0
					&& ts->finger_data[i].z_data == 0
					&& ts->finger_data[i].w_data == 0
					&& ts->finger_data[i].x_data == 0
					&& ts->finger_data[i].y_data == 0  )
					continue;
				if ( !ts->suspendMode )
				{
					input_report_abs(ts->input_dev, ABS_MT_PRESSURE,
							ts->finger_data[i].z_data);
					input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,
							 ts->finger_data[i].w_data);
					input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
							 ts->finger_data[i].x_data);
					input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
							 ts->finger_data[i].y_data);
					input_report_abs(ts->input_dev, ABS_MT_ORIENTATION,
							 ts->finger_data[i].vector);
					input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID,
							 i);
					input_mt_sync(ts->input_dev);
				}
			}
			input_sync(ts->input_dev);
		}
		else
		{
			QTOUCH_INFO4("%s: Single touch: x=%d, y=%d\n",
					__func__,ts->finger_data[0].x_data,ts->finger_data[0].y_data); 
			if ( !ts->suspendMode )
			{
				/* In single mode, only 1st finger is used */
				input_report_abs(ts->input_dev, ABS_X,
						 ts->finger_data[0].x_data);
				input_report_abs(ts->input_dev, ABS_Y,
						 ts->finger_data[0].y_data);
				input_report_abs(ts->input_dev, ABS_PRESSURE,
						 ts->finger_data[0].z_data);
				input_report_abs(ts->input_dev, ABS_TOOL_WIDTH,
						 ts->finger_data[0].w_data);
				if (ts->finger_data[0].down == 0)
					input_report_key(ts->input_dev, BTN_TOUCH, 0 );
				else
					input_report_key(ts->input_dev, BTN_TOUCH, 1 );
				input_sync(ts->input_dev);
			}
		}
	}
	else
	{
/* 
 * Check if user finger is just moving into the button range or not 
 * If it is, generate a "liftoff" messages first 
*/
		int needLiftOff = FALSE;
		if ( ts->pdata->buttons_count > 0 )
		{
			for ( i = 0; i < _NUM_FINGERS && needLiftOff == FALSE; i++ )
			{
				/* is previous point Y less then top of the button? */
				if ( ts->prev_finger_data[i].y_data < buttons[0]->minY )
					needLiftOff = TRUE;
			}
		}
		QTOUCH_INFO4("%s: needLiftOff= %s\n", __func__, (needLiftOff ? "Yes" : "No"));
		if ( needLiftOff )
		{
			for (i = 0; i < ts->pdata->multi_touch_cfg.num_touch; i++) 
			{
				if ( !ts->suspendMode )
				{
					input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
					input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR,
							 ts->prev_finger_data[i].w_data);
					input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
							 ts->prev_finger_data[i].x_data);
					input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
							 ts->prev_finger_data[i].y_data);
					input_mt_sync(ts->input_dev);
				}
			}
			input_sync(ts->input_dev);
		}
		/* Generate appropriate key based on the range if X for each button */
		QTOUCH_INFO("%s: Generating button\n", __func__ );
		for ( i = 0; i < ts->pdata->buttons_count; i++ )
		{
			QTOUCH_INFO("%s: Checking against button %d\n", __func__, i);
			if ( ts->finger_data[btnFlag].x_data >= buttons[i]->minX && 
			  ts->finger_data[btnFlag].x_data <= buttons[i]->maxX ) 
			{
				/* If this button was already pressed, don't send anything more */
				QTOUCH_INFO("%s: Button[%d] is %d\n", __func__, i, 
					ts->finger_data[btnFlag].down);
				if ( ts->finger_data[btnFlag].down == 1 )
				{
					/* This is a press */
					if ( buttons[i]->pressed == 0 )
					{
						buttons[i]->pressed = 1;
						input_report_key (ts->input_dev, buttons[i]->key, 1);
						QTOUCH_INFO("%s: Sent %d key\n", __func__, buttons[i]->key);
					}
				}
				else
				{
					/* This is a release */
					if ( buttons[i]->pressed == 1 )
					{
						buttons[i]->pressed = 0;
						input_report_key (ts->input_dev, buttons[i]->key,0);
						QTOUCH_INFO("%s: Sent %d key\n", __func__, buttons[i]->key);
					}
				}
				input_sync(ts->input_dev);
			}
		}
	}
#endif
	if (stuckFinger)
	{
		if (!ts->suspendMode)
		{
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE,
				 ts->finger_data[i].z_data);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,
				 ts->finger_data[i].w_data);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
				 ts->finger_data[i].x_data);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
				 ts->finger_data[i].y_data);
			input_report_abs(ts->input_dev, ABS_MT_ORIENTATION,
				 ts->finger_data[i].vector);
			input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID,
				 i);
			input_mt_sync(ts->input_dev);
			ts->finger_data[1].x_data = 0;
			ts->finger_data[1].y_data = 0;
			ts->finger_data[1].w_data = 0;
		}
	}
#if 0
	if (!down) 
	{
		memset(&ts->finger_data[finger],
				0,
				sizeof(struct coordinate_map));
	}

	for (i = 0; i < ts->pdata->multi_touch_cfg.num_touch; i++) 
		memcpy(&(ts->prev_finger_data[i]),
				&(ts->finger_data[i]),
				sizeof(ts->finger_data[i]));
#endif
	if (down) {
		ts->finger_data[finger].x_data = x;
		ts->finger_data[finger].y_data = y;
		ts->finger_data[finger].w_data = width;
		ts->finger_data[finger].z_data = pressure;
		ts->finger_data[finger].vector = msg->touch_vect;
		ts->finger_data[finger].down = 1;
	} else {
		memset(&ts->finger_data[finger], 0,
			sizeof(struct coordinate_map));
	}

	num_fingers_down = 0;
	for (i = 0; i < ts->pdata->multi_touch_cfg.num_touch; i++) {
		if (ts->finger_data[i].down == 0)
			continue;
		input_report_abs(ts->input_dev, ABS_MT_PRESSURE,
				 ts->finger_data[i].z_data);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,
				 ts->finger_data[i].w_data);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
				 ts->finger_data[i].x_data);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
				 ts->finger_data[i].y_data);
		input_report_abs(ts->input_dev, ABS_MT_ORIENTATION,
				 ts->finger_data[i].vector);
		input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID,
				 i);
		input_mt_sync(ts->input_dev);
		num_fingers_down++;
	}
	if (num_fingers_down == 0)
		input_mt_sync(ts->input_dev);
	input_sync(ts->input_dev);

	QTOUCH_INFO("%s: exit...\n", __func__);
	return 0;
}

/* Handles a message from a keyarray object. */
static int do_touch_keyarray_msg(struct qtouch_ts_data *ts,
				 struct qtm_object *obj, void *_msg)
{
	struct qtm_touch_keyarray_msg *msg = _msg;
	int i;

	/* nothing changed.. odd. */
	if (ts->last_keystate == msg->keystate)
		return 0;

	for (i = 0; i < ts->pdata->key_array.num_keys; ++i) {
		struct qtouch_key *key = &ts->pdata->key_array.keys[i];
		uint32_t bit = 1 << (key->channel & 0x1f);
		if ((msg->keystate & bit) != (ts->last_keystate & bit)) {
			if (msg->keystate & bit) {
				if (ignore_keyarray_touches)
					return 0;
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 4);
			} else {
				input_report_abs(ts->input_dev,
						 ABS_MT_TOUCH_MAJOR, 0);
			}
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 4);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
					 ts->pdata->key_array.keys[i].x_coord);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
					 ts->pdata->key_array.keys[i].y_coord);
			input_mt_sync(ts->input_dev);
			input_sync(ts->input_dev);
		}
	}

	QTOUCH_INFO2("%s: key state changed 0x%08x -> 0x%08x\n", __func__,
			ts->last_keystate, msg->keystate);

	/* update our internal state */
	ts->last_keystate = msg->keystate;

	return 0;
}
static int do_cmd_gen_msg_t5(struct qtouch_ts_data *ts,
				 struct qtm_object *obj, void *_msg)
{
	int ret = 0;
	struct	qtm_cmd_proc_msg	msg;
	struct qtm_object *object;

	QTOUCH_INFO("%s: enter...\n", __func__);

	/* Read the T5 message out */
	object = find_obj(ts, QTM_OBJ_GEN_MSG_PROC);
	ret = qtouch_read_addr(ts, object->entry.addr, &msg, sizeof(msg) );
	if (!ret)
	{
		qtouch_dump_block(__func__, (char *) (&msg), sizeof(msg));
	}
	QTOUCH_INFO("%s: exit...\n", __func__);

	return ret;
}

static int qtouch_handle_msg(struct qtouch_ts_data *ts, struct qtm_object *obj,
			     struct qtm_obj_message *msg)
{
	int ret = 0;

	/* These are all the known objects that we know how to handle. */
	QTOUCH_INFO("%s: the object type = %d\n", __func__, obj->entry.type);
	switch (obj->entry.type) 
	{
	case QTM_OBJ_GEN_MSG_PROC:
		ret = do_cmd_gen_msg_t5(ts, obj, msg);
		break;

	case QTM_OBJ_SPT_SELF_TEST:
		ret = do_cmd_selftest_msg(ts, obj, msg);
		break;

	case QTM_OBJ_GEN_CMD_PROC:
		ret = do_cmd_proc_msg(ts, obj, msg);
		break;

	case QTM_OBJ_TOUCH_MULTI:
		ret = do_touch_multi_msg(ts, obj, msg);
		break;

	case QTM_OBJ_TOUCH_KEYARRAY:
		ret = do_touch_keyarray_msg(ts, obj, msg);
		break;

	default:
		/* probably not fatal? */
		ret = 0;
		QTOUCH_INFO("%s: No handler defined for message from object "
			"type %d, report_id %d\n", __func__, obj->entry.type,
			msg->report_id);
	}

	return ret;
}

static int	qtouch_set_bootloader_mode(struct qtouch_ts_data *ts)
{
	int		ret;
	char	data[5];
	
	/* need to read 1 byte from i2c */
	ts->client->addr = XMEGAT_BL_I2C_ADDR;
	ts->i2cBLAddr = XMEGAT_BL_I2C_ADDR;
	ret = qtouch_read(ts, data, 1);
	if (ret  < 0)
	{
		ts->client->addr = XMEGAT_BL_I2C_ADDR_ALT;
		ts->i2cBLAddr = XMEGAT_BL_I2C_ADDR_ALT;
		ret = qtouch_write(ts, data, 1);
		if (ret < 0)
		{
			QTOUCH_INFO("%s: BT client error %d\n",
					__func__, ret);
			QTOUCH_INFO("%s: touch device is probably missing.\n",
					__func__);
			ts->modeOfOperation = QTOUCH_MODE_UNKNOWN;
			return QTOUCH_MODE_UNKNOWN;
		}
	}
	else
	{
		QTOUCH_INFO("%s: qtouch_write (bootloader client) data: 0x%x\n",
				__func__, data[0]);
		QTOUCH_INFO("%s: entering bootloader mode\n",__func__);
		ts->modeOfOperation = QTOUCH_MODE_BOOTLOADER;
		/* wait until we get 0x8n */
		while ( (data[0] & 0xF0) != 0x80 )
		{
			QTOUCH_INFO("%s: Sending Unblock command (0xDC,0xAA)\n",__func__);
			data[0] = 0xDC;
			data[1] = 0xAA;
			ret = qtouch_write(ts, data, 2);
			if ( ret < 0 )
			{
				QTOUCH_INFO("%s: BT unlock: 0xDC,0xAA)\n",
						__func__);
				ts->modeOfOperation = QTOUCH_MODE_UNKNOWN;
				return QTOUCH_MODE_UNKNOWN;
			}
			msleep(30);
			ret = qtouch_read(ts, data, 1);
			if ( ret < 0 )
			{
				QTOUCH_INFO("%s: error waiting for 0x8n (%d)\n",
						__func__, ret);
				ts->modeOfOperation = QTOUCH_MODE_UNKNOWN;
				return QTOUCH_MODE_UNKNOWN;
			}
			msleep(30);
		}
	}
	return QTOUCH_MODE_BOOTLOADER;
}

static	int	qtouch_reset_read_ptr(struct qtouch_ts_data *ts)
{
	struct qtm_object *obj;
	int err;

	obj = find_obj(ts, QTM_OBJ_GEN_MSG_PROC);
	err = qtouch_set_addr(ts, obj->entry.addr);
	return err;
}
	
static int qtouch_process_info_block(struct qtouch_ts_data *ts)
{
	uint16_t our_csum = 0x0;
	uint16_t their_csum = 0x00;
	uint8_t	checksum[3];
	uint8_t report_id;
	uint16_t addr;
	int err;
	int i;

	/* query the device and get the info block. */
	err = qtouch_read_addr(ts, QTM_OBP_ID_INFO_ADDR, &qtm_info,
			       sizeof(qtm_info));
	if (0 != err) 
	{
		QTOUCH_ERR("%s: Cannot read info object block\n", __func__);
		/* Check if the device is in bootloader mode */
		goto err_read_info_block;
	}
	our_csum = calc_csum(our_csum, &qtm_info, sizeof(qtm_info));

	/* TODO: Add a version/family/variant check? */

	if (0 == qtm_info.num_objs ) 
	{
		QTOUCH_ERR("%s: Device (0x%x/0x%x/0x%x/0x%x) does not export any "
		       "objects.\n", __func__, qtm_info.family_id,
		       qtm_info.variant_id, qtm_info.version, qtm_info.build);
		err = -ENODEV;
		goto err_no_objects;
	}
	else
	{
		qtouch_dump_block(__func__, (char *) (&qtm_info), sizeof(qtm_info));
		QTOUCH_INFO("%s: Dumping information from Id info block:\n", __func__ );
		QTOUCH_INFO("================================\n");
		QTOUCH_INFO("\t Family ID: %d\n", qtm_info.family_id);
		QTOUCH_INFO("\t Variant ID: %d\n", qtm_info.variant_id);
		QTOUCH_INFO("\t Version: %d\n", qtm_info.version);
		QTOUCH_INFO("\t Build: %d\n", qtm_info.build);
		QTOUCH_INFO("\t Matrix X size: %d\n", qtm_info.matrix_x_size);
		QTOUCH_INFO("\t Matrix Y size: %d\n", qtm_info.matrix_y_size);
		QTOUCH_INFO("\t Number of objects to read: %d\n",qtm_info.num_objs);
		QTOUCH_INFO("================================\n");
	}
	addr = QTM_OBP_ID_INFO_ADDR + sizeof(qtm_info);
	report_id = 1;

	/* Clear the object table */
	for (i = 0; i < QTM_OBP_MAX_OBJECT_NUM; ++i) 
	{
		ts->obj_tbl[i].entry.type = 0;
		ts->obj_tbl[i].entry.addr = 0;
		ts->obj_tbl[i].entry.size = 0;
		ts->obj_tbl[i].entry.num_inst = 0;
		ts->obj_tbl[i].entry.num_rids = 0;
		ts->obj_tbl[i].report_id_min = 0;
		ts->obj_tbl[i].report_id_max = 0;
	}

	/* read out the object entries table */
	for (i = 0; i < qtm_info.num_objs; ++i) 
	{
		struct qtm_object *obj;
		struct qtm_obj_entry entry;

		err = qtouch_read_addr(ts, addr, &entry, sizeof(entry));
		if (err != 0) 
		{
			QTOUCH_ERR("%s: Can't read object (%d) entry.\n", __func__, i);
			err = -EIO;
			goto err_read_entry;
		}
		our_csum = calc_csum(our_csum, &entry, sizeof(entry));
		addr += sizeof(entry);

		entry.size++;
		entry.num_inst++;

		qtouch_dump_block(__func__, (char *) (&entry), sizeof(entry));
		QTOUCH_INFO("%s: Object %d @ 0x%04x (%d) insts %d rep_ids %d\n",
			__func__, entry.type, entry.addr, entry.size,
			entry.num_inst, entry.num_rids);

		if (entry.type >= QTM_OBP_MAX_OBJECT_NUM) 
		{
			QTOUCH_INFO("%s: Unknown object type (%d) encountered\n", __func__, entry.type);
			/* Not fatal */
			continue;
		}

		/* save the message_procesor msg_size for easy reference. */
		if (entry.type == QTM_OBJ_GEN_MSG_PROC)
			ts->msg_size = entry.size;

		obj = create_obj(ts, &entry);
		/* set the report_id range that the object is responsible for */
		if ((obj->entry.num_rids * obj->entry.num_inst) != 0) 
		{
			obj->report_id_min = report_id;
			report_id += obj->entry.num_rids * obj->entry.num_inst;
			obj->report_id_max = report_id - 1;
		}
	}

	if (!ts->msg_size) 
	{
		QTOUCH_ERR("%s: Message processing object not found. Bailing.\n",
		       __func__);
		err = -ENODEV;
		goto err_no_msg_proc;
	}

	/* verify that some basic objects are present. These objects are
	 * assumed to be present by the rest of the driver, so fail out now
	 * if the firmware is busted. */
	if (!find_obj(ts, QTM_OBJ_GEN_PWR_CONF) ||
	    !find_obj(ts, QTM_OBJ_GEN_ACQUIRE_CONF) ||
	    !find_obj(ts, QTM_OBJ_GEN_MSG_PROC) ||
	    !find_obj(ts, QTM_OBJ_GEN_CMD_PROC)) 
	{
		QTOUCH_ERR("%s: Required objects are missing\n", __func__);
		err = -ENOENT;
		goto err_missing_objs;
	}

	err = qtouch_read_addr(ts, addr, &checksum, sizeof(checksum));
	if (err != 0) 
	{
		QTOUCH_ERR("%s: Unable to read remote checksum\n", __func__);
		err = -ENODEV;
		goto err_no_checksum;
	}

	/* FIXME: The algorithm described in the datasheet doesn't seem to
	 * match what the touch firmware is doing on the other side. We
	 * always get mismatches! */
	their_csum = (checksum[0] |
		(checksum[1] << BITS_PER_BYTE) |
		(checksum[2] << (BITS_PER_BYTE << 1)));

	their_csum = __le32_to_cpu(their_csum);

	if (our_csum != their_csum) 
	{
		QTOUCH_ERR("%s: Checksum mismatch (0x%04x != 0x%04x)\n", __func__, our_csum, their_csum);
#ifndef IGNORE_CHECKSUM_MISMATCH
		err = -ENODEV;
		goto err_bad_checksum;
#endif
	}

	QTOUCH_INFO("%s: %s found. family 0x%x, variant 0x%x, ver 0x%x, "
		"build 0x%x, matrix %dx%d, %d objects.\n", __func__,
		QTOUCH_TS_NAME, qtm_info.family_id, qtm_info.variant_id,
		qtm_info.version, qtm_info.build, qtm_info.matrix_x_size,
		qtm_info.matrix_y_size, qtm_info.num_objs);

	ts->eeprom_checksum = ts->pdata->nv_checksum;

	return 0;

err_no_checksum:
err_missing_objs:
err_no_msg_proc:
err_read_entry:
err_no_objects:
err_read_info_block:
	return err;
}

static void qtouch_ts_work_func(struct work_struct *work)
{
	struct qtouch_ts_data *ts = container_of(work, struct qtouch_ts_data, work);
	struct qtm_obj_message *msg;
	struct qtm_object *obj;
	int ret;
	bool	keepGoing = TRUE;
#ifdef USE_NVODM_STUFF
    NvU32 pinValue = 0;
#else
	unsigned char pinValue = 0;
#endif

	QTOUCH_INFO("%s: Enter....\n", __func__);
	/* We need to read all the messages from i2c as long as interrup is low */
	if ( ts->hw_init == TRUE )
	{
		/* Hardware is being initialized. Just return */
		QTOUCH_INFO("%s: Hardware is being initalized. Exit....\n", __func__);
		return;
	}
	if ( ts->modeOfOperation == QTOUCH_MODE_BOOTLOADER )
	{
		char	buf[5];
		ret = qtouch_read(ts, buf, 1);
		msleep(30);
		if (ret < 0)
		{
			/* Receive failed. Reset status  */
/* 
 * ASSUMPTION: we fail to read because there is no client on 0x5F 
 * address because IC has finished reset after the firmware.
 * So, we can switch the address to 0x11 and to NORMAL mode
 * */
			QTOUCH_ERR("%s: qtouch_reqd failed(BOOTLOADER mode)\n",
					__func__);
			ts->dlStatus = QTOUCH_BL_WAITING_FOR_NOTHING;
			ts->modeOfOperation = QTOUCH_MODE_NORMAL;
			ts->client->addr = ts->i2cNormAddr;
		}
		else
		{
			QTOUCH_INFO("%s: buf[0] = 0x%x\n", __FUNCTION__, buf[0]);
			/* Determine which code we got and set status appropriately */
			if ( (buf[0] & 0xF0) == 0xC0 )
			{
				ts->dlStatus = QTOUCH_BL_WAITING_FOR_COMMAND;
			}
			else if ( (buf[0] & 0xF0) == 0x80 )
			{
				if ( ts->dlStatus == QTOUCH_BL_GOT_BAD_CRC )
					ts->dlStatus = QTOUCH_BL_WAITING_AFTER_BAD_CRC;
				else
					ts->dlStatus = QTOUCH_BL_WAITING_FOR_DATA;
			}
			else if ( buf[0] == 0x02 )
			{
				ts->dlStatus = QTOUCH_BL_WAITING_FOR_CRC;
			}
			else if ( buf[0] == 0x04 )
			{
				ts->dlStatus = QTOUCH_BL_WAITING_AFTER_GOOD_CRC;
			}
			else if ( buf[0] == 0x03 )
			{
				/* We got bad CRC on the record */
				ts->dlStatus = QTOUCH_BL_GOT_BAD_CRC;
			}
		}
	}
	else
	{
		while (keepGoing)
		{
			QTOUCH_INFO("%s: interrupt pin is %s\n", __func__, (pinValue)?"high":"low");
			QTOUCH_INFO("%s: current i2c address: 0x%02X\n",__func__, ts->client->addr);
			msg = qtouch_read_msg(ts);
			if (msg == NULL) 
			{
				QTOUCH_ERR("%s: Cannot read message\n", __func__);
				keepGoing = FALSE;
			}
			else
			{
				if ( msg->report_id == 0xff )
				{
					keepGoing = FALSE;
					clean_i2c(ts);
				}
				else
				{
					if (ts->cal_check_flag)
						check_chip_calibration(ts);

					obj = find_object_rid(ts, msg->report_id);
					if (!obj) 
					{
						QTOUCH_ERR("%s: Unknown object for report_id %d\n", __func__,
							   msg->report_id);
						keepGoing = FALSE;
						clean_i2c(ts);
					}
					else
					{
						ret = qtouch_handle_msg(ts, obj, msg);
						if (ret != 0) 
						{
							QTOUCH_ERR("%s: Unable to process message for obj %d, "
								   "report_id %d\n", __func__, obj->entry.type,
								   msg->report_id);
							keepGoing = FALSE;
						}
					}
				}
			}
			if ( keepGoing == TRUE )
			{

				pinValue = gpio_get_value(tsGl->pdata->gpio_intr);

				QTOUCH_INFO("%s:  GPIO is %s", __func__, 
						(pinValue == 1) ? "HIGH\n":"LOW\n" );
				if ( pinValue )
					keepGoing = FALSE;
			}
		}
	}
	QTOUCH_INFO("%s: Exit....\n", __func__);
}



static ssize_t qtouch_irq_status(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev,
	                                         struct i2c_client, dev);
	struct qtouch_ts_data *ts = i2c_get_clientdata(client);
	return sprintf(buf, "%u\n", ts->irqStatus);
}

static ssize_t qtouch_irq_enable(struct device *dev,
                                 struct device_attribute *attr,
                                 const char *buf, size_t size)
{
	struct i2c_client *client = container_of(dev,
	                                         struct i2c_client, dev);
	struct qtouch_ts_data *ts = i2c_get_clientdata(client);
	int err = 0;
	unsigned long value;

	if (size > 2)
		return -EINVAL;

	err = strict_strtoul(buf, 10, &value);
	if (err != 0)
		return err;

	switch (value) {
	case 0:
		qtouch_disable_irq(ts->irqInt);
		err = size;
		break;
	case 1:
		qtouch_enable_irq(ts->irqInt);
		qtouch_force_reset(ts, FALSE);
		qtouch_force_calibration(ts);
		err = size;
		break;
	default:
		pr_info("qtouch_irq_enable failed -> irqStatus = %d\n", 
					ts->irqStatus);
		err = -EINVAL;
		break;
	}

	return err;
}

static DEVICE_ATTR(irqStatus, 0664, qtouch_irq_status, qtouch_irq_enable);

static ssize_t qtouch_update_i2c_addr(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev,
									struct i2c_client, dev);
	struct qtouch_ts_data *ts = i2c_get_clientdata(client);

	return sprintf(buf, "%02X\n", ts->client->addr);

}

static DEVICE_ATTR(i2cAddr, 0444, qtouch_update_i2c_addr, NULL);

static ssize_t qtouch_update_status(struct device *dev,
				    struct device_attribute *attr, char *buf)
{

	/* means no need for upgrade */
	return sprintf(buf, "%u\n", 254);
}

static DEVICE_ATTR(update_status, 0444, qtouch_update_status, NULL);

static ssize_t qtouch_fw_version(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev,
						 struct i2c_client, dev);
	struct qtouch_ts_data *ts = i2c_get_clientdata(client);
	/* Make sure that IC can accept the config information */
	if ( ts->modeOfOperation == QTOUCH_MODE_NORMAL )
	{
		qtouch_process_info_block(ts);
	}
	return sprintf(buf, "0x%X%X\n", qtm_info.version, qtm_info.build);
}

static DEVICE_ATTR(fw_version, 0444, qtouch_fw_version, NULL);

static int qtouch_ts_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct qtouch_ts_platform_data *pdata = client->dev.platform_data;
	struct qtouch_ts_data *ts;
	struct qtm_object *obj;
	int err;
	int i;
	int	tmpDbg;

	tmpDbg = qtouch_tsdebug;
	qtouch_tsdebug = 0xFF;
	QTOUCH_INFO("%s: Enter...\n", __func__);

#ifdef CONFIG_XMEGAT_USE_RESET_TO_RESUME
	QTOUCH_INFO("%s: Using reset on resume\n", __func__);
#ifdef CONFIG_XMEGAT_DO_HARD_RESET
	QTOUCH_INFO("%s: Using HARD reset on resume\n", __func__);
#endif
#endif
/* do hard reset before soft reset */
/* #define CONFIG_XMEGAT_DO_HARD_RESET */

	need2check4IC_problem = FALSE;
	if (pdata == NULL) 
	{
		QTOUCH_ERR("%s: platform data required\n", __func__);
		qtouch_tsdebug = tmpDbg;
		return -ENODEV;
	} 
	else if (!client->irq) 
	{
		QTOUCH_ERR("%s: polling mode currently not supported\n", __func__);
		qtouch_tsdebug = tmpDbg;
		return -ENODEV;
	} 
	else if (!pdata->hw_reset) 
	{
		QTOUCH_ERR("%s: Must supply a hw reset function\n", __func__);
		qtouch_tsdebug = tmpDbg;
		return -ENODEV;
	}

	/* Get buttons information from the pdata */
	QTOUCH_ERR("%s: Initializing %d buttons\n", __func__, pdata->buttons_count);
	for ( i = 0; i < pdata->buttons_count; i++ )
	{
		buttons[i] = &(pdata->buttons[i]);
		QTOUCH_ERR("%s: Button %d: minX: %d, maxX: %d, key: %d\n", __func__, 
			i, buttons[i]->minX, buttons[i]->maxX, buttons[i]->key );
	}

	err = misc_register(&qtouch_pf_driver);
	QTOUCH_ERR("%s: misc_register(pf) returns %d\n", __func__, err);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
	{
		QTOUCH_ERR("%s: need I2C_FUNC_I2C\n", __func__);
		qtouch_tsdebug = tmpDbg;
		return -ENODEV;
	}

	ts = kzalloc(sizeof(struct qtouch_ts_data), GFP_KERNEL);
	if (ts == NULL) 
	{
		err = -ENOMEM;
		goto err_alloc_data_failed;
	}

	/* Should only be used by IOCTL functions */
	tsGl = ts;

	mutex_init(&ts->i2c_lock);
	qtouch_ts_wq = create_singlethread_workqueue("qtouch_obp_ts_wq");
	if (qtouch_ts_wq == NULL) 
	{
		QTOUCH_ERR("%s: No memory for qtouch_ts_wq\n", __func__);
		return -ENOMEM;
	}

	INIT_WORK(&ts->work, qtouch_ts_work_func);

	ts->hw_init = FALSE;
	ts->modeOfOperation = QTOUCH_MODE_NORMAL;
	ts->dlStatus = OBP_DL_WAITING_FOR_NOTHING;
	ts->multiMode = TRUE;
	ts->pdata = pdata;
	ts->client = client;
	ts->suspendMode = false;
	ts->XferStatus = QTOUCH_FM_DOWNLOAD_NOT_STARTED | QTOUCH_CFG_DOWNLOAD_NOT_STARTED;
	ts->selfStatus = QTOUCH_CNTRL_UNKNOWN;
	wake_lock_init(&(ts->wLock), WAKE_LOCK_SUSPEND, QTOUCH_WAKE_NAME);

	i2c_set_clientdata(client, ts);
	ts->checksum_cnt = 0;
	ts->x_delta = ts->pdata->x_delta;
	ts->y_delta = ts->pdata->y_delta;

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) 
	{
		QTOUCH_ERR("%s: failed to alloc input device\n", __func__);
		err = -ENOMEM;
		goto err_alloc_input_dev;
	}
	ts->input_dev->name = QTOUCH_TS_NAME;
	input_set_drvdata(ts->input_dev, ts);
/*
	ts->msg_buf = kmalloc(ts->msg_size, GFP_KERNEL);
	if (ts->msg_buf == NULL) 
	{
		QTOUCH_ERR("%s: Cannot allocate msg_buf\n", __func__);
		err = -ENOMEM;
		goto err_alloc_msg_buf;
	}
	else
		QTOUCH_ERR("%s: allocated msg_buf @ 0x%X\n", __func__, ts->msg_buf);
*/

	ts->client->addr = XMEGAT_I2C_ADDR;
	ts->i2cNormAddr = XMEGAT_I2C_ADDR;

	qtouch_force_reset(ts, FALSE);

	err = qtouch_process_info_block(ts);
	if (err != 0)
	{
/*
 * Try alternative i2c address
 * 
*/
		ts->client->addr = XMEGAT_I2C_ADDR_ALT;
		ts->i2cNormAddr = XMEGAT_I2C_ADDR_ALT;
		err = qtouch_process_info_block(ts);
		if (err != 0)
		{
/* 
 * Failed reading information block from the IC. See if we are in 
 * BOOTLOADER mode
 */
			if ( qtouch_set_bootloader_mode(ts) != QTOUCH_MODE_BOOTLOADER )
				goto err_process_info_block;
		}
	}

	if ( ts->modeOfOperation == QTOUCH_MODE_NORMAL )
	{
		/* Point the address pointer to the message processor.
		 * Must do this before enabling interrupts */
		QTOUCH_ERR("%s: setting QTM_OBJ_GEN_MSG_PROC\n", __func__);
		err = qtouch_reset_read_ptr(ts);
		if (err != 0) 
		{
			QTOUCH_ERR("%s: Can't to set addr to msg processor\n", __func__);
			goto err_rst_addr_msg_proc;
		}
		/* Before enabling interrupt, clean up i2c, just in case.... */
		clean_i2c(ts);
	}
	else
		QTOUCH_ERR("%s: switched into BOOTLOADER mode\n", __func__);

	/* register the harwdare assisted virtual keys, if any */
	obj = find_obj(ts, QTM_OBJ_TOUCH_KEYARRAY);
	if (obj && (obj->entry.num_inst > 0) &&
	    (pdata->flags & QTOUCH_USE_KEYARRAY)) 
	{
		for (i = 0; i < pdata->key_array.num_keys; ++i)
			input_set_capability(ts->input_dev, EV_KEY,
					     pdata->key_array.keys[i].code);
	}

	set_bit(EV_SYN, ts->input_dev->evbit);
	/* register the software virtual keys, if any are provided */
	for (i = 0; i < pdata->vkeys.count; ++i)
		input_set_capability(ts->input_dev, EV_KEY,
		     pdata->vkeys.keys[i].code);

	for ( i=0; i < pdata->buttons_count; i++ )
		input_set_capability(ts->input_dev, EV_KEY,
				buttons[i]->key);

	obj = find_obj(ts, QTM_OBJ_TOUCH_MULTI);
	if (obj && obj->entry.num_inst > 0) 
	{
		if ( ts->multiMode )
		{
			/* multi touch */
			QTOUCH_ERR("%s: MultiTouch config X: (%d,%d), Y: (%d,%d), TOUCH MAJOR: (%d,%d), WIDTH: (%d,%d)\n",
					__func__,
					pdata->abs_min_x,pdata->abs_max_x,
					pdata->abs_min_y,pdata->abs_max_y,
					pdata->abs_min_p, pdata->abs_max_p,
					pdata->abs_min_w, pdata->abs_max_w);
			input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X,
				pdata->abs_min_x, pdata->abs_max_x,
				pdata->fuzz_x, 0);
			input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y,
				pdata->abs_min_y, pdata->abs_max_y,
				pdata->fuzz_y, 0);
			input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR,
				pdata->abs_min_p, pdata->abs_max_p,
				pdata->fuzz_p, 0);
			input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR,
				pdata->abs_min_w, pdata->abs_max_w,
				pdata->fuzz_w, 0);
			set_bit(EV_ABS, ts->input_dev->evbit);
			set_bit(EV_KEY, ts->input_dev->keybit);
			set_bit(EV_SYN, ts->input_dev->keybit);
			set_bit(BTN_TOUCH, ts->input_dev->keybit);
			set_bit(ABS_X, ts->input_dev->keybit);
			set_bit(ABS_Y, ts->input_dev->keybit);
		}
		else
		{
			/* Legacy support for testing only */
			QTOUCH_ERR("%s: Single touch config: X: (%d,%d), Y: (%d,%d), PRESSURE: (%d,%d), WIDTH: (%d,%d)\n",
					__func__,
					pdata->abs_min_x,pdata->abs_max_x,
					pdata->abs_min_y,pdata->abs_max_y,
					pdata->abs_min_p, pdata->abs_max_p,
					pdata->abs_min_w, pdata->abs_max_w);

			set_bit(EV_SYN, ts->input_dev->evbit);
			set_bit(EV_KEY, ts->input_dev->evbit);
			set_bit(BTN_TOUCH, ts->input_dev->keybit);
			set_bit(KEY_HOME, ts->input_dev->keybit);
			set_bit(KEY_BACK, ts->input_dev->keybit);
			set_bit(KEY_MENU, ts->input_dev->keybit);
			set_bit(BTN_2, ts->input_dev->keybit);
			set_bit(EV_ABS, ts->input_dev->evbit);
	
			input_set_capability(ts->input_dev, EV_KEY, BTN_TOUCH);
			input_set_capability(ts->input_dev, EV_KEY, BTN_2);
			input_set_abs_params(ts->input_dev, ABS_X,
				pdata->abs_min_x, pdata->abs_max_x,
				pdata->fuzz_x, 0);
			input_set_abs_params(ts->input_dev, ABS_HAT0X,
				pdata->abs_min_x, pdata->abs_max_x,
				pdata->fuzz_x, 0);
			input_set_abs_params(ts->input_dev, ABS_Y,
				pdata->abs_min_y, pdata->abs_max_y,
				pdata->fuzz_y, 0);
			input_set_abs_params(ts->input_dev, ABS_HAT0Y,
				pdata->abs_min_x, pdata->abs_max_x,
				pdata->fuzz_x, 0);
			input_set_abs_params(ts->input_dev, ABS_PRESSURE,
				pdata->abs_min_p, pdata->abs_max_p,
				pdata->fuzz_p, 0);
			input_set_abs_params(ts->input_dev, ABS_TOOL_WIDTH,
				pdata->abs_min_w, pdata->abs_max_w,
				pdata->fuzz_w, 0);
		}
	}

	memset(&ts->finger_data[0], 0,
		(sizeof(struct coordinate_map) * _NUM_FINGERS));

	err = input_register_device(ts->input_dev);
	if (err != 0) 
	{
		QTOUCH_ERR("%s: Cannot register input device \"%s\"\n", __func__,
		       ts->input_dev->name);
		goto err_input_register_dev;
	}

	QTOUCH_INFO("%s: gpio_request(reset)\n", __func__);
	err = gpio_request(pdata->gpio_reset, QTOUCH_RST_NAME);
	if ( err )	{
		pr_err("%s: gpio_request(reset) failed\n", __func__);
		goto err_request_irq;
	}

	QTOUCH_INFO("%s: gpio_direction_output(reset)\n", __func__);
	err = gpio_direction_output(pdata->gpio_reset, 1);
	if ( err )	{
		pr_err("%s: gpio_direction_input(reset) failed\n", __func__);
		goto err_request_irq;
	}

	QTOUCH_INFO("%s: gpio_request(interrupt)\n", __func__);
	err = gpio_request(pdata->gpio_intr, QTOUCH_INT_NAME);
	if ( err )	{
		pr_err("%s: gpio_request(interrupt) failed\n", __func__);
		goto err_request_irq;
	}

	QTOUCH_INFO("%s: gpio_direction_input(interrupt)\n", __func__);
	err = gpio_direction_input(pdata->gpio_intr);
	if ( err )	{
		pr_err("%s: gpio_direction_input(interrupt) failed\n", __func__);
		goto err_request_irq;
	}
	ts->irqInt = gpio_to_irq(pdata->gpio_intr);
	pr_info("%s: INTR irq: %d, current value: %d\n",
			__func__, ts->irqInt, gpio_get_value(pdata->gpio_intr));

	err = request_irq(ts->irqInt, 
						qtouch_ts_irq_handler,
						IRQF_TRIGGER_FALLING,
						QTOUCH_INT_NAME, 
						ts);
	if (err != 0) 
	{
		QTOUCH_ERR("%s: request_irq (%d) failed\n", __func__,
		       ts->irqInt);
		goto err_request_irq;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1;
	ts->early_suspend.suspend = qtouch_ts_early_suspend;
	ts->early_suspend.resume = qtouch_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	ts->irqStatus = TRUE;
	qtouch_tsdebug = tmpDbg;

	err = device_create_file(&ts->client->dev, &dev_attr_irqStatus);
	if (err != 0) {
		pr_err("%s:File device creation failed: %d\n", __func__, err);
		err = -ENODEV;
		goto err_create_file_failed;
	}

	err = device_create_file(&ts->client->dev, &dev_attr_update_status);
	if (err != 0) {
		pr_err("%s:File device creation failed: %d\n", __func__, err);
		err = -ENODEV;
		goto err_create_update_status_failed;
	}

	err = device_create_file(&ts->client->dev, &dev_attr_fw_version);
	if (err != 0) {
		pr_err("%s:File device creation failed: %d\n", __func__, err);
		err = -ENODEV;
		goto err_create_fw_version_file_failed;
	}

	err = device_create_file(&ts->client->dev, &dev_attr_i2cAddr);
	if (err != 0) {
		pr_err("%s:File device creation failed: %d\n", __func__, err);
		err = -ENODEV;
		goto err_create_i2cAddr_failed;
	}

	ts->cal_check_flag = 0;
	ts->cal_timer = 0;

	return 0;

err_create_i2cAddr_failed:
	device_remove_file(&ts->client->dev, &dev_attr_fw_version);
err_create_fw_version_file_failed:
	device_remove_file(&ts->client->dev, &dev_attr_update_status);
err_create_update_status_failed:
	device_remove_file(&ts->client->dev, &dev_attr_irqStatus);
err_create_file_failed:
	free_irq(ts->client->irq, ts);
err_request_irq:
	input_unregister_device(ts->input_dev);

err_input_register_dev:
err_rst_addr_msg_proc:
err_process_info_block:
	input_free_device(ts->input_dev);

err_alloc_input_dev:
	mutex_destroy(&ts->i2c_lock);
	i2c_set_clientdata(client, NULL);
	wake_lock_destroy(&(ts->wLock));
	kfree(ts);

err_alloc_data_failed:
/*
	ts->modeOfOperation = QTOUCH_MODE_BOOTLOADER;
	qtouch_tsdebug = tmpDbg;
*/
	return err;
}

static int qtouch_ts_remove(struct i2c_client *client)
{
	struct qtouch_ts_data *ts = i2c_get_clientdata(client);

	unregister_early_suspend(&ts->early_suspend);
	free_irq(ts->irqInt, ts);
	input_unregister_device(ts->input_dev);
	input_free_device(ts->input_dev);
	i2c_set_clientdata(client, NULL);
	wake_lock_destroy(&(ts->wLock));
	kfree(ts);
	return 0;
}

static int qtouch_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct qtouch_ts_data *ts = i2c_get_clientdata(client);
	int ret;

	QTOUCH_INFO("%s: Suspending\n", __func__);

	/* Note, this may block and suspend timer may expire, causing panic */
	cancel_work_sync(&ts->work);

	ret = qtouch_power_config(ts, FALSE);
	if (ret < 0)
		QTOUCH_ERR("%s: Cannot write power config\n", __func__);

	/* Need some delay. Seems that if suspend/resup happens too fast, TS
	 * gets locked up and only reset helps
	 */
	msleep(10);
	return 0;
}

static int qtouch_ts_resume(struct i2c_client *client)
{
	struct qtouch_ts_data *ts = i2c_get_clientdata(client);
	int ret;
	int i;

	QTOUCH_INFO4("%s: Resuming\n", __func__);

	/*
	* If we were suspended while a touch was happening
	*  we need to tell the upper layers so they do not hang
	* waiting on the liftoff that will not come.
	*/
	for (i = 0; i < ts->pdata->multi_touch_cfg.num_touch; i++)
	{
		QTOUCH_INFO4("%s: Finger %i down state %i\n",
			__func__, i, ts->finger_data[i].down);
		if (ts->finger_data[i].down == 0)
			continue;
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_mt_sync(ts->input_dev);
		memset(&ts->finger_data[i], 0,
			sizeof(struct coordinate_map));
	}
	input_sync(ts->input_dev);

#ifdef CONFIG_XMEGAT_USE_RESET_TO_RESUME
/*
 * hard reset does not generate reset message, so there
 * will be no re-calibration after the hard reset.
 * soft reset does, and does not require configuration reload.
 * so we prefer to do just soft reset, since it makes resume faster
 * and avoids mucking with interrupts.
 * not clear if there is anything that hard reset actually solves
 */

#ifdef CONFIG_XMEGAT_DO_HARD_RESET
	qtouch_force_reset(ts, FALSE);
	/* fall through here, to do power/calibration */

#else /* CONFIG_XMEGAT_DO_HARD_RESET */
	qtouch_force_reset(ts, TRUE);
	/*
	 * nothing more to do, qtouch_hw_init from control msg will do the rest
	 */
	return 0;
#endif /* CONFIG_XMEGAT_DO_HARD_RESET */
#endif /* CONFIG_XMEGAT_USE_RESET_TO_RESUME */

	ret = qtouch_power_config(ts, TRUE);
	if (ret < 0) 
	{
		QTOUCH_ERR("%s: Cannot write power config\n", __func__);
		return -EIO;
	}

	ret = qtouch_force_calibration(ts);
	if (ret != 0) {
		pr_err("%s: Unable to write to calibrate\n", __func__);
		return -EIO;
	}

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void qtouch_ts_early_suspend(struct early_suspend *handler)
{
	struct qtouch_ts_data *ts;

	ts = container_of(handler, struct qtouch_ts_data, early_suspend);
	qtouch_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void qtouch_ts_late_resume(struct early_suspend *handler)
{
	struct qtouch_ts_data *ts;

	ts = container_of(handler, struct qtouch_ts_data, early_suspend);
	qtouch_ts_resume(ts->client);
}
#endif

/* file operation functions */
/**
 * Open device
 *
 * @param inode
 * @param filp
 * @return 0
 */
static	int	qtouch_ioctl_open(struct inode *inode, struct file *filp)
{
	int	rc;
	QTOUCH_INFO("%s: entering\n", __func__);

	if ( tsGl->modeOfOperation == QTOUCH_MODE_NORMAL ||
			tsGl->modeOfOperation == QTOUCH_MODE_BOOTLOADER )
		rc = 0;
	else
		rc = -1;

	QTOUCH_INFO("%s: exiting\n", __func__);
	return rc;
}

unsigned char	kernelBuffer[sizeof(struct qtim_ioctl_data)];
/**
 * Ioctl implementation
 *
 * @param node File in /proc
 * @param f Kernel level file structure
 * @param cmd Ioctl command
 * @param arg Ioctl argument
 * @return 0 in success, or negative error code
 */
static	QTM_ALL_OBJECTS qtmAllObjects;
static long qtouch_ioctl_ioctl(struct file *filp,
		unsigned int code, unsigned long arg)
{
	IOCTL_DATA	*usrData;
	struct qtm_object *obj;
	struct coordinate_map		*pointPtr;
/*
	struct qtm_id_info	qtmId;
*/
	int	rc = 0;
	unsigned long	bCount;
	int	i;
	uint8_t	*fields;
	char *raw;
	char	need2send = FALSE;
	unsigned int	cmd;

	QTOUCH_INFO("%s: entering\n", __func__);
	if ( tsGl->modeOfOperation == QTOUCH_MODE_UNKNOWN )
		return -1;

	bCount = copy_from_user(kernelBuffer, (char *)arg,
			(unsigned long)sizeof(IOCTL_DATA));
	if ( bCount == 0 )
	{
		usrData = (IOCTL_DATA *)kernelBuffer;
	}
	else
		return -1;

	cmd = usrData->cmd;
	switch (cmd) 
	{
	case QTOUCH_IOCTL_GET_DEBUG:
		usrData->data.debug = qtouch_tsdebug;
		break;
	case QTOUCH_IOCTL_SET_DEBUG:
		QTOUCH_INFO("%s: Setting debug to %d\n", __func__,
				usrData->data.debug);
		qtouch_tsdebug = usrData->data.debug;
		usrData->data.debug = qtouch_tsdebug;
		break;
	case QTOUCH_IOCTL_GET_VERSION:
		/* Make sure that IC can accept the config information */
		if ( tsGl->modeOfOperation != QTOUCH_MODE_NORMAL )
			return -1;
		rc = qtouch_process_info_block(tsGl);
		QTOUCH_INFO("%s: Dumping information from Id info block:\n",
				__func__);
		QTOUCH_INFO("================================\n");
		QTOUCH_INFO("\t Family ID: %d\n", qtm_info.family_id);
		usrData->data.ioctl_version.FamilyId = qtm_info.family_id;
		QTOUCH_INFO("\t Variant ID: %d\n", qtm_info.variant_id);
		usrData->data.ioctl_version.VariantId = qtm_info.variant_id;
		QTOUCH_INFO("\t Version: %d\n", qtm_info.version);
		usrData->data.ioctl_version.Version = qtm_info.version;
		QTOUCH_INFO("\t Build: %d\n", qtm_info.build);
		usrData->data.ioctl_version.Build = qtm_info.build;
		usrData->data.ioctl_version.Mode = tsGl->modeOfOperation;
		QTOUCH_INFO("\t Mode: %d\n", usrData->data.ioctl_version.Mode);
		QTOUCH_INFO("================================\n");
		break;
	case QTOUCH_IOCTL_MODE:
		QTOUCH_INFO("%s: Doing mode operations\n", __func__);
		switch ( usrData->data.mode.op )
		{
		case QTOUCH_MODE_NORMAL:
			/* Make sure that IC can accept the config information */
			if ( tsGl->modeOfOperation == QTOUCH_MODE_UNKNOWN )
				return -1;
			QTOUCH_INFO("%s: Setting mode to NORMAL\n", __func__);
			if ( tsGl->modeOfOperation != QTOUCH_MODE_NORMAL )
			{
				/* Resetting the hardware should put the device into normal mode */
				tsGl->client->addr = tsGl->i2cNormAddr;
				rc = qtouch_force_reset(tsGl, FALSE);
				qtouch_force_calibration(tsGl);
				tsGl->modeOfOperation = QTOUCH_MODE_NORMAL;
			}
			break;
		case QTOUCH_MODE_BOOTLOADER:
			if ( tsGl->modeOfOperation == QTOUCH_MODE_UNKNOWN )
				return -1;
			QTOUCH_INFO("%s: Setting mode to BOOTLOADER\n",
					__func__);
			if ( tsGl->modeOfOperation != QTOUCH_MODE_BOOTLOADER )
			{
				char	data[3];

				/* Need to disable interrupt, so we can do i2c read in this function */
				/* Setting device and all needed flags to bootloader mode */
				/* Make sure power is not 0 */
				qtouch_power_config(tsGl, TRUE);
				/* Write 0xA5 to command processor object */
				qtouch_force_bootloaderMode(tsGl);
				/* artificial delay to make sure the device is in the right state */
				msleep(50);
				/* Make sure the device is in bootloader mode */
				tsGl->client->addr = tsGl->i2cBLAddr;
				rc = qtouch_read(tsGl, data, 1);
				if( (data[0] & 0xC0 ) == 0xC0 )
				{
					QTOUCH_INFO("%s: mode is BOOTLOADER. Ready to unlock to device for firmware download.\n", __func__);
					/* unlock the device */
					data[0] = 0xDC;
					data[1] = 0xAA;
					rc = qtouch_write(tsGl, data, 2);
					msleep(30);
					rc = qtouch_read(tsGl, data, 1);
					if( (data[0] & 0x80 ) == 0x80 )
					{
						/* Device should be in the right state waiting for the data */
						tsGl->dlStatus = OBP_DL_WAITING_FOR_DATA;
						tsGl->modeOfOperation = QTOUCH_MODE_BOOTLOADER;
					}
				}
				else
					QTOUCH_INFO("%s: did not receive confirmation of the BOOTLOADER mode. Got: 0x%02X\n", __func__, data[0] );
			}
			break;
		case QTOUCH_MODE_GET:
			QTOUCH_INFO("%s: Responding to mode query\n", __func__);
		default:
			break;
		}
		usrData->data.mode.op = tsGl->modeOfOperation;
		usrData->data.mode.dl = tsGl->dlStatus;
		break;
	case QTOUCH_IOCTL_CNTRL:
		if ( tsGl->modeOfOperation != QTOUCH_MODE_NORMAL )
			return -1;
		switch ( usrData->data.info )
		{
		case QTOUCH_CNTRL_RESET:
			qtouch_force_reset(tsGl, FALSE);
			rc = qtouch_process_info_block(tsGl);
			break;
		case QTOUCH_CNTRL_CALIBRATE:
			rc = qtouch_force_calibration(tsGl);
			break;
		case QTOUCH_CNTRL_SELFTEST_ON:
			rc = qtouch_force_selftest(tsGl, true);
			memcpy((char *)&(usrData->data.selftst), (char *) &(tsGl->selfStatusReport), sizeof(struct selftest));
			break;
		case QTOUCH_CNTRL_SELFTEST_OFF:
			rc = qtouch_force_selftest(tsGl, false);
			memcpy((char *)&(usrData->data.selftst), (char *) &(tsGl->selfStatusReport), sizeof(struct selftest));
			break;
		case QTOUCH_CNTRL_SELFTEST_GET:
			memcpy((char *)&(usrData->data.selftst), (char *) &(tsGl->selfStatusReport), sizeof(struct selftest));
			break;
		default:
			rc = -1;
			break;
		}
		break;
	case QTOUCH_IOCTL_IRQ:
		if ( tsGl->modeOfOperation != QTOUCH_MODE_NORMAL )
			return -1;
		switch ( usrData->data.irq )
		{
		case QTOUCH_IRQ_ON:
			qtouch_enable_irq(tsGl->irqInt);
			qtouch_force_reset(tsGl, FALSE);
			qtouch_force_calibration(tsGl);
			break;
		case QTOUCH_IRQ_OFF:
			qtouch_disable_irq(tsGl->irqInt);
			break;
		case QTOUCH_IRQ_GET:
			break;
		default:
			break;
		}
		rc = 0;
		usrData->data.irq = tsGl->irqStatus;
		break;
	case QTOUCH_IOCTL_GET_REGISTER:
		if ( tsGl->modeOfOperation != QTOUCH_MODE_NORMAL )
			return -1;
		rc = qtouch_read_addr(tsGl, usrData->data.reg.reg, &(usrData->data.reg.vals[0]),sizeof(usrData->data.reg.vals[0]));
		break;
	case QTOUCH_IOCTL_SET_REGISTER:
		if ( tsGl->modeOfOperation != QTOUCH_MODE_NORMAL )
			return -1;
		rc = qtouch_write_addr(tsGl, usrData->data.reg.reg, &(usrData->data.reg.value), 1);
		break;
	case QTOUCH_IOCTL_SET_OBJECT:
		if ( tsGl->modeOfOperation != QTOUCH_MODE_NORMAL )
			return -1;
		fields = (char *) &qtmAllObjects;
		memset(fields, 0, sizeof(qtmAllObjects));
		need2send = FALSE;
		switch (usrData->type)
		{
		case QTM_OBJ_GEN_PWR_CONF:
			QTOUCH_INFO("%s: Got QTM_OBJ_GEN_PWR_CONF object:\n", __func__);
			/* Each bit represents a field. Max of 32 fields */
			for ( i = 0; i < sizeof(struct qtm_gen_power_cfg); i++ )
			{
				if ( usrData->data.qtm_t7.changed & (1<<i) )
				{
					/* This byte has changed based on configuration */
					fields[i] = usrData->data.qtm_t7.data.rawData[i];
					need2send = TRUE;
				}
			}
			memcpy((char *)&(tsGl->pdata->power_cfg),(char *)fields,sizeof(struct qtm_gen_power_cfg));

			QTOUCH_INFO("\tidle_acq_int: %d\n", tsGl->pdata->power_cfg.idle_acq_int);
			QTOUCH_INFO("\tactive_acq_int: %d\n", tsGl->pdata->power_cfg.active_acq_int);
			QTOUCH_INFO("\tactive_idle_to: %d\n", tsGl->pdata->power_cfg.active_idle_to);
			break;
		case QTM_OBJ_GEN_ACQUIRE_CONF:
			QTOUCH_INFO("%s: Got QTM_OBJ_GEN_ACQUIRE_CONF object\n", __func__);
			/* Each bit represents a field. Max of 32 fields */
			for ( i = 0; i < sizeof(struct qtm_gen_acquire_cfg); i++ )
			{
				if ( usrData->data.qtm_t8.changed & (1<<i) )
				{
					/* This byte has changed based on configuration */
					fields[i] = usrData->data.qtm_t8.data.rawData[i];
					need2send = TRUE;
				}
			}
			memcpy((char *)&(tsGl->pdata->acquire_cfg),(char *)fields,sizeof(struct qtm_gen_acquire_cfg));
			break;
		case QTM_OBJ_TOUCH_MULTI:
			QTOUCH_INFO("%s: Got QTM_OBJ_TOUCH_MULTI object\n", __func__);
			/* Each bit represents a field. Max of 32 fields */
			for ( i = 0; i < sizeof(struct qtm_touch_multi_cfg); i++ )
			{
				if ( usrData->data.qtm_t9.changed & (1<<i) )
				{
					/* This byte has changed based on configuration */
					fields[i] = usrData->data.qtm_t9.data.rawData[i];
					need2send = TRUE;
				}
			}
			memcpy((char *)&(tsGl->pdata->multi_touch_cfg),(char *)fields,sizeof(struct qtm_touch_multi_cfg));
			break;
		case QTM_OBJ_TOUCH_KEYARRAY:
			QTOUCH_INFO("%s: Got QTM_OBJ_TOUCH_KEYARRAY object\n", __func__);
			/* Each bit represents a field. Max of 32 fields */
			for ( i = 0; i < sizeof(struct qtm_touch_keyarray_cfg); i++ )
			{
				if ( usrData->data.qtm_t15.changed & (1<<i) )
				{
					/* This byte has changed based on configuration */
					fields[i] = usrData->data.qtm_t15.data.rawData[i];
					need2send = TRUE;
				}
			}
			memcpy((char *)&(tsGl->pdata->key_array.cfg),(char *)fields,sizeof(struct qtm_touch_keyarray_cfg));
			break;
		case QTM_OBJ_SPT_GPIO_PWM:
			QTOUCH_INFO("%s: Got QTM_OBJ_SPT_GPIO_PWM object\n", __func__);
			/* Each bit represents a field. Max of 32 fields */
			for ( i = 0; i < sizeof(struct qtm_spt_gpio_pwm_cfg); i++ )
			{
				if ( usrData->data.qtm_t19.changed & (1<<i) )
				{
					/* This byte has changed based on configuration */
					fields[i] = usrData->data.qtm_t19.data.rawData[i];
					need2send = TRUE;
				}
			}
			memcpy((char *)&(tsGl->pdata->gpio_pwm_cfg),(char *)fields,sizeof(struct qtm_spt_gpio_pwm_cfg));
			break;
		case QTM_OBJ_PROCI_GRIPFACESUPPRESSION:
			QTOUCH_INFO("%s: Got QTM_OBJ_PROCI_GRIPFACESUPPRESSION object\n", __func__);
			/* Each bit represents a field. Max of 32 fields */
			for ( i = 0; i < sizeof(struct qtm_proci_grip_suppression_cfg); i++ )
			{
				if ( usrData->data.qtm_t20.changed & (1<<i) )
				{
					/* This byte has changed based on configuration */
					fields[i] = usrData->data.qtm_t20.data.rawData[i];
					need2send = TRUE;
				}
			}
			memcpy((char *)&(tsGl->pdata->grip_suppression_cfg),(char *)fields,sizeof(struct qtm_proci_grip_suppression_cfg));
			break;
		case QTM_OBJ_PROCG_NOISE_SUPPRESSION:
			QTOUCH_INFO("%s: Got QTM_OBJ_PROCG_NOISE_SUPPRESSION object\n", __func__);
			/* Each bit represents a field. Max of 32 fields */
			for ( i = 0; i < sizeof(struct qtm_procg_noise_suppression_cfg); i++ )
			{
				if ( usrData->data.qtm_t22.changed & (1<<i) )
				{
					/* This byte has changed based on configuration */
					fields[i] = usrData->data.qtm_t22.data.rawData[i];
					need2send = TRUE;
				}
			}
			memcpy((char *)&(tsGl->pdata->noise_suppression_cfg),(char *)fields,sizeof(struct qtm_procg_noise_suppression_cfg));
			break;
		case QTM_OBJ_TOUCH_PROXIMITY:
			QTOUCH_INFO("%s: Got QTM_OBJ_TOUCH_PROXIMITY object\n", __func__);
			/* Each bit represents a field. Max of 32 fields */
			for ( i = 0; i < sizeof(struct qtm_touch_proximity_cfg); i++ )
			{
				if ( usrData->data.qtm_t23.changed & (1<<i) )
				{
					/* This byte has changed based on configuration */
					fields[i] = usrData->data.qtm_t23.data.rawData[i];
					need2send = TRUE;
				}
			}
			memcpy((char *)&(tsGl->pdata->touch_proximity_cfg),(char *)fields,sizeof(struct qtm_touch_proximity_cfg));
			break;
		case QTM_OBJ_PROCI_ONE_TOUCH_GESTURE_PROC:
			QTOUCH_INFO("%s: Got QTM_OBJ_PROCI_ONE_TOUCH_GESTURE_PROC object\n", __func__);
			/* Each bit represents a field. Max of 32 fields */
			for ( i = 0; i < sizeof(struct qtm_proci_one_touch_gesture_proc_cfg); i++ )
			{
				if ( usrData->data.qtm_t24.changed & (1<<i) )
				{
					/* This byte has changed based on configuration */
					fields[i] = usrData->data.qtm_t24.data.rawData[i];
					need2send = TRUE;
				}
			}
			memcpy((char *)&(tsGl->pdata->one_touch_gesture_proc_cfg),(char *)fields,sizeof(struct qtm_proci_one_touch_gesture_proc_cfg));
			break;
		case QTM_OBJ_SPT_SELF_TEST:
			QTOUCH_INFO("%s: Got QTM_OBJ_SPT_SELF_TEST object\n", __func__);
			/* Each bit represents a field. Max of 32 fields */
			for ( i = 0; i < sizeof(struct qtm_spt_self_test_cfg); i++ )
			{
				if ( usrData->data.qtm_t25.changed & (1<<i) )
				{
					/* This byte has changed based on configuration */
					fields[i] = usrData->data.qtm_t25.data.rawData[i];
					need2send = TRUE;
				}
			}
			memcpy((char *)&(tsGl->pdata->self_test_cfg),(char *)fields,sizeof(struct qtm_spt_self_test_cfg));
			break;
		case QTM_OBJ_PROCI_TWO_TOUCH_GESTURE_PROC:
			QTOUCH_INFO("%s: Got QTM_OBJ_PROCI_TWO_TOUCH_GESTURE_PROC object\n", __func__);
			/* Each bit represents a field. Max of 32 fields */
			for ( i = 0; i < sizeof(struct qtm_proci_two_touch_gesture_proc_cfg); i++ )
			{
				if ( usrData->data.qtm_t27.changed & (1<<i) )
				{
					/* This byte has changed based on configuration */
					fields[i] = usrData->data.qtm_t27.data.rawData[i];
					need2send = TRUE;
				}
			}
			memcpy((char *)&(tsGl->pdata->two_touch_gesture_proc_cfg),(char *)fields,sizeof(struct qtm_proci_two_touch_gesture_proc_cfg));
			break;
		case QTM_OBJ_SPT_CTE_CONFIG:
			QTOUCH_INFO("%s: Got QTM_OBJ_SPT_CTE_CONFIG object\n", __func__);
			/* Make sure that IC can accept the config information */
			if ( tsGl->modeOfOperation != QTOUCH_MODE_NORMAL )
				return -1;
			/* Each bit represents a field. Max of 32 fields */
			for ( i = 0; i < sizeof(struct qtm_spt_cte_config_cfg); i++ )
			{
				if ( usrData->data.qtm_t28.changed & (1<<i) )
				{
					/* This byte has changed based on configuration */
					fields[i] = usrData->data.qtm_t28.data.rawData[i];
					need2send = TRUE;
				}
			}
			memcpy((char *)&(tsGl->pdata->cte_config_cfg),(char *)fields,sizeof(struct qtm_spt_cte_config_cfg));
			break;
		default:
			rc = -4;
			break;
		}
		if ( need2send )
		{
			obj = find_obj(tsGl, usrData->type);
			rc = qtouch_write_addr(tsGl, obj->entry.addr, fields, i);
			if (rc != 0) 
			{
				QTOUCH_ERR("%s: Can't write config object\n", __func__);
				rc = -5;
			}
			else
				QTOUCH_INFO("%s: wrote config object!\n", __func__);
		}
		break;
	case QTOUCH_IOCTL_PUSH_CONFIG:
		QTOUCH_INFO("%s: Pushing configuration to the IC\n", __func__);
		/* Make sure that IC can accept the config information */
		if ( tsGl->modeOfOperation != QTOUCH_MODE_NORMAL )
			return -1;

		qtouch_force_reset(tsGl, FALSE);
		qtouch_force_calibration(tsGl);
		rc = qtouch_process_info_block(tsGl);
		break;
	case QTOUCH_IOCTL_GET_CONFIG:
		QTOUCH_INFO("%s: Getting configuration element %d\n", __func__, usrData->data.objId);
		/* Make sure that IC can accept the config information */
		if ( tsGl->modeOfOperation != QTOUCH_MODE_NORMAL )
			return -1;
		raw = (char *)&(usrData->data);
		rc = qtouch_read_addr(tsGl, 
						tsGl->obj_tbl[usrData->data.objId].entry.addr, 
						(void *)raw, 
						tsGl->obj_tbl[usrData->data.objId].entry.size);
		break;
	case QTOUCH_IOCTL_DL_GET_STATUS:
		QTOUCH_INFO("%s: current download status: %d\n", __func__, tsGl->dlStatus);
		/* Make sure that IC can accept the config information */
		if ( tsGl->modeOfOperation != QTOUCH_MODE_NORMAL )
			return -1;
		rc = tsGl->dlStatus;
		break;
	case QTOUCH_IOCTL_SUSPEND:
		/* Make sure that IC can accept the config information */
		if ( tsGl->modeOfOperation != QTOUCH_MODE_NORMAL )
			return -1;
		switch ( usrData->data.suspend )
		{
		case QTOUCH_SUSPEND_ON:
			if ( tsGl->suspendMode != true )
			{
				qtouch_ts_suspend(tsGl->client, PMSG_SUSPEND);
				tsGl->suspendMode = true;
			}
			break;
		case QTOUCH_SUSPEND_OFF:
			if ( tsGl->suspendMode != false )
			{
				qtouch_ts_resume(tsGl->client);
				tsGl->suspendMode = false;
			}
			break;
		case QTOUCH_SUSPEND_GET:
			break;
		default:
			break;
		}
		rc = 0;
		usrData->data.suspend = tsGl->suspendMode;
		QTOUCH_INFO("%s: current suspend status: %d\n", __func__, tsGl->suspendMode);
		break;
	case QTOUCH_IOCTL_GET_POINT:
		/* Make sure that IC can accept the config information */
		if ( tsGl->modeOfOperation != QTOUCH_MODE_NORMAL )
			return -1;
		QTOUCH_INFO("%s: X: %d, Y: %d, W: %d, Z: %d\n", __func__,
			tsGl->finger_data[0].x_data,
			tsGl->finger_data[0].y_data,
			tsGl->finger_data[0].w_data,
			tsGl->finger_data[0].z_data );

		pointPtr = &(tsGl->finger_data[0]);
		usrData->data.currPoints[0].x_data = pointPtr->x_data;
		usrData->data.currPoints[0].y_data = pointPtr->y_data;
		usrData->data.currPoints[0].w_data = pointPtr->w_data;
		usrData->data.currPoints[0].z_data = pointPtr->z_data;
		break;
	case QTOUCH_IOCTL_SET_CMD_STATUS:
		tsGl->XferStatus = usrData->data.info;	
	case QTOUCH_IOCTL_GET_CMD_STATUS:
		usrData->data.info = tsGl->XferStatus;
		break;
	case QTOUCH_IOCTL_GET_SENS:
		/* Make sure that IC can accept the config information */
		if ( tsGl->modeOfOperation != QTOUCH_MODE_NORMAL )
			return -1;
		switch ( usrData->data.info )
		{
		case QTOUCH_SENSITIVITY_CHCOUNT:
			usrData->data.info = qtouch_get_channels_count(tsGl);
			QTOUCH_INFO("%s: SENS->CHCOUNT = %d channels\n", __func__, usrData->data.info);
			break;
		case QTOUCH_SENSITIVITY_GETREF:
			QTOUCH_INFO("%s: got SENS->GETREF request!\n", __func__);
			rc = qtouch_get_sens_data(tsGl, &(usrData->data.sens), 0 );
			break;
		case QTOUCH_SENSITIVITY_NEXTPAGE:
			QTOUCH_INFO("%s: got SENS->NEXTPAGE request!\n", __func__);
			rc = qtouch_get_sens_data(tsGl, &(usrData->data.sens), 1 );
			break;
		case QTOUCH_SENSITIVITY_UNKNOWN:
		default:
			rc = -1;
			QTOUCH_INFO("%s: got unknown sensitivity request!\n", __func__);
			break;
		}
		break;
	default:
		rc = -1;
		break;
	}

	/* Local space should be populated with requested information. Copy that to the user space and get out */
	if ( rc >= 0 )
	{
		bCount = copy_to_user((char *)arg, (char *)usrData, (unsigned long)sizeof(*usrData));
		if ( bCount != 0 )
			rc = -EFAULT;
		else
		{
			rc = 0;
			QTOUCH_INFO("%s: Copied %d bytes to user space\n",__func__, sizeof(*usrData));
		}
	}
	rc = qtouch_reset_read_ptr(tsGl);
	if (rc != 0) 
	{
		QTOUCH_ERR("%s: Can't to set addr to msg processor\n", __func__);
	}
	QTOUCH_INFO("%s: exiting. RC = %d\n", __func__, rc);
	return rc;
}

/**
 * Write to device
 *
 * @param flip
 * @param buf
 * @param size
 * @param pos
 * @return number of bytes written to the device
 */


static int qtouch_ioctl_write(struct file *flip, const char __user *buf, size_t count, loff_t *f_pos )
{
	unsigned long	bCount;
	int	retval = 0;
	int	i;

	QTOUCH_INFO("%s: Enter....\n",__func__);
	if ( tsGl->modeOfOperation == QTOUCH_MODE_BOOTLOADER )
	{
		tsGl->dlStatus = QTOUCH_BL_WAITING_FOR_COMMAND;
		bCount = copy_from_user(kernelBuffer, buf, (unsigned long)count); 
		QTOUCH_INFO("%s: copy_from_user() returned %d\n", __func__,(int)bCount);
		tsGl->client->addr = XMEGAT_BL_I2C_ADDR;
		do
		{
			QTOUCH_INFO("%s: record to IC: size=%d, record data:\n",__func__,count); 
			for ( i = 0; i < count; i++)
			{
				if ( !(i%20) )
					QTOUCH_INFO("\n%s: ", __func__);
				QTOUCH_INFO("%02x ", kernelBuffer[i]);
			}
			QTOUCH_INFO("\n");
			retval = qtouch_write(tsGl, kernelBuffer, count);

			if (retval < 0)
			{
				QTOUCH_ERR("%s: write failed: %d\n", __func__,retval);
				tsGl->dlStatus = QTOUCH_BL_WAITING_FAILED;
			}
			else
			{
				QTOUCH_INFO("%s: qtouch_write(): rc= %d\n",
						__func__, retval);
				tsGl->dlStatus = QTOUCH_BL_WAITING_FOR_COMMAND;
				retval = count;
			}
			msleep(30);
		}
		while (retval < 0);
	}
	else
	{
		QTOUCH_INFO("%s: Device is not in BOOTLOADER mode\n", __func__);
		retval = -1;
	}
	return retval;
}

static uint8_t calibrate_chip(struct qtouch_ts_data *ts)
{
	uint8_t atchcalst, atchcalsthr, tchautocal;
	struct qtm_object *obj;
	int ret;

	ts->cal_timer = 0;

	atchcalst = ts->pdata->acquire_cfg.atch_cal_suspend_time;
	atchcalsthr = ts->pdata->acquire_cfg.atch_cal_suspend_thres;
	tchautocal = ts->pdata->acquire_cfg.touch_autocal;
	ts->pdata->acquire_cfg.atch_cal_suspend_time = 0;
	ts->pdata->acquire_cfg.atch_cal_suspend_thres = 0;
	ts->pdata->acquire_cfg.touch_autocal = 5;

	if (qtouch_tsdebug & 16)
		pr_info("%s: issue cal. calst=%d, calsthr=%d\n",
			__func__, atchcalst, atchcalsthr);

	/* configure the acquisition object. */
	obj = find_obj(ts, QTM_OBJ_GEN_ACQUIRE_CONF);
	ret = qtouch_write_addr(ts, obj->entry.addr, &ts->pdata->acquire_cfg,
				min(sizeof(ts->pdata->acquire_cfg),
				    obj->entry.size));
	if (ret != 0) {
		pr_err("%s: Can't write acquisition config\n", __func__);
		return ret;
	}

	ts->pdata->acquire_cfg.atch_cal_suspend_time = atchcalst;
	ts->pdata->acquire_cfg.atch_cal_suspend_thres = atchcalsthr;
	ts->pdata->acquire_cfg.touch_autocal = tchautocal;

	ret = qtouch_force_calibration(ts);
	if (ret != 0) {
		pr_err("%s: Unable to write to calibrate\n", __func__);
		return ret;
	}

	/* Reset the address pointer after forcing a calibration */
	ret = qtouch_set_addr(ts, ts->obj_tbl[QTM_OBJ_GEN_MSG_PROC].entry.addr);
	if (ret != 0) {
		pr_err("%s: Unable to reset addr pointer after calibration\n",
		       __func__);
		return ret;
	}

	return 0;
}

static uint8_t check_chip_calibration(struct qtouch_ts_data *ts)
{
	uint8_t val, num_x_lines, num_y_lines;
	uint16_t bytes_to_read, maskl, maskh;
	struct qtm_object *obj;
	int ret, try_ctr = 0;
	uint8_t *data_buffer = NULL;
	uint8_t i;
	uint8_t *diag_ptr = NULL;
	int touch = 0;
	int atouch = 0;
	uint16_t addr;
	unsigned long current_jiffies;

	obj = find_obj(ts, QTM_OBJ_GEN_CMD_PROC);
	addr = obj->entry.addr + offsetof(struct qtm_gen_cmd_proc, diagnostic);
	val = 0xF3;
	ret = qtouch_write_addr(ts, addr, &val, 1);
	if (ret) {
		pr_err("%s: Unable to send the diagnostic message\n", __func__);
		goto err_diag_data;
	}

	data_buffer = kmalloc(QTM_OBP_DIAG_BUFFER_SIZE, GFP_KERNEL);
	if (data_buffer == NULL) {
		pr_err("%s: Unable to allocate memory for diag buffer\n",
			 __func__);
		goto err_diag_data;
	}

	msleep(QTM_OBP_SLEEP_WAIT_FOR_DIAG_DATA);
	memset(data_buffer, 0xFF, QTM_OBP_DIAG_BUFFER_SIZE);

	obj = find_obj(ts, QTM_OBJ_DEBUG_DIAGNOSTICS_T37);
	while (!((*data_buffer == 0xF3) && (*(data_buffer + 1) == 0x00))) {
		if (try_ctr > QTM_OBP_NUM_DIAG_READ_RETRIES) {
			pr_err("%s: Failed to get Diag block response\n",
				__func__);
			ret = -1;
			goto err_diag_data;
		}

		try_ctr++;
		msleep(QTM_OBP_SLEEP_WAIT_FOR_DIAG_DATA);
		ret = qtouch_read_addr(ts, obj->entry.addr, data_buffer, 2);

		if (ret != 0) {
			pr_err("%s: Cannot read diag block header\n", __func__);
			goto err_diag_data;
		}
	}

	/* Always read 82 bytes; (20 * 2 * 2) + 2 */
	num_x_lines = ts->pdata->multi_touch_cfg.x_size;
	num_y_lines = ts->pdata->multi_touch_cfg.y_size;
	bytes_to_read = (QTM_OBP_DIAG_NUM_X_LINES * 2 * 2) + 2;

	if (bytes_to_read > QTM_OBP_DIAG_BUFFER_SIZE) {
		pr_err("%s: data_buffer not enough\n", __func__);
		ret = -1;
		goto err_diag_data;
	}

	if (qtouch_tsdebug & 16)
		pr_info("%s: Reading diag info. xsize=%d, \
			ysize=%d, read_size = %d\n",
			__func__, num_x_lines, num_y_lines, bytes_to_read);

	ret = qtouch_read_addr(ts, obj->entry.addr, data_buffer,
		bytes_to_read);

	if (ret != 0) {
		pr_err("%s: Cannot read diag block data\n", __func__);
		goto err_diag_data;
	}

	if (qtouch_tsdebug & 32) {
		int msg_bytes;
		int msg_location;
		char *msg;
		int index;

		msg = kmalloc(1024, GFP_KERNEL);
		if (msg != NULL) {
			msg_location = sprintf(msg, "%s: Diag data:",
				__func__);
			for (index = 0; index < QTM_OBP_DIAG_BUFFER_SIZE;
				index++) {
				msg_bytes = snprintf((msg + msg_location),
					(1024 - msg_location),
					" 0x%02x",
					*(data_buffer + index));
				msg_location += msg_bytes;
				if (msg_location >= 1024)
					break;
			}
			if (msg_location < 1024) {
				pr_info("%s\n", msg);
			} else {
				pr_info("%s:  Diag data: String overflow\n",
				__func__);
			}

			kfree(msg);
		}
	}

	diag_ptr = data_buffer + 2;
	maskl = ((1 << num_y_lines) - 1) & (0x00FF);
	maskh = (((1 << num_y_lines) - 1) & (0xFF00)) >> 8;

	if ((*data_buffer == 0xF3) && (*(data_buffer + 1) == 0)) {
		for (i = 0; i < num_x_lines; i++) {
			touch += hweight8((uint8_t)(*diag_ptr & maskl));
			touch += hweight8((uint8_t)(*(diag_ptr + 1) & maskh));
			atouch += hweight8(
				(uint8_t)(*(diag_ptr
				+ (QTM_OBP_DIAG_NUM_X_LINES<<1)) & maskl));
			atouch += hweight8(
				(uint8_t)(*(diag_ptr
				+ (QTM_OBP_DIAG_NUM_X_LINES<<1) + 1) & maskh));
			diag_ptr += 2;
		}
	} else {
		pr_err("%s: Diag block header not correct\n", __func__);
		ret = -1;
		goto err_diag_data;
	}

	if (qtouch_tsdebug & 16)
		pr_info("%s: Processed diag data. touch=%d, atouch=%d\n",
			__func__, touch, atouch);

	obj = find_obj(ts, QTM_OBJ_GEN_CMD_PROC);
	addr = obj->entry.addr + offsetof(struct qtm_gen_cmd_proc, diagnostic);
	val = 0x01;
	ret = qtouch_write_addr(ts, addr, &val, 1);
	if (ret) {
		pr_err("%s: Unable to send pageup diagnostic message\n",
			__func__);
		goto err_diag_data;
	}

	if (touch && (atouch == 0)) {
		current_jiffies = jiffies;

		if (qtouch_tsdebug & 16)
			pr_info("%s: cal_timer=%lu curr_jiffies=%lu\n",
				__func__, ts->cal_timer, current_jiffies);

		if (ts->cal_timer == 0) {
			ts->cal_timer = current_jiffies;
		} else if ((((long)current_jiffies -
				(long)ts->cal_timer) * 1000 / HZ) >=
				QTM_OBP_DIAG_CALIB_WAIT_MS) {

			ts->cal_check_flag = 0;
			ts->cal_timer = 0;
			obj = find_obj(ts, QTM_OBJ_GEN_ACQUIRE_CONF);
			ret = qtouch_write_addr(ts, obj->entry.addr,
						&ts->pdata->acquire_cfg,
						min(sizeof(
							ts->pdata->acquire_cfg),
						obj->entry.size));
			if (ret != 0) {
				pr_err("%s: Can't write acquisition config\n",
					__func__);
				goto err_diag_data;
			}
		}
	} else if (0 < atouch) {
		calibrate_chip(ts);
	} else {
		if (qtouch_tsdebug & 16)
			pr_info("%s: cannot decide.  Wait for next data\n",
				__func__);
	}

err_diag_data:
	if (data_buffer != NULL)
		kfree(data_buffer);

	ret = qtouch_set_addr(ts, ts->obj_tbl[QTM_OBJ_GEN_MSG_PROC].entry.addr);
	if (ret != 0) {
		pr_err("%s: Unable to reset address pointer to Gen Msg\n",
		       __func__);
	}

	return ret;
}

/******** init ********/
static const struct i2c_device_id qtouch_ts_id[] = 
{
	{ QTOUCH_TS_NAME, 0 },
	{ }
};

static struct i2c_driver qtouch_ts_driver = 
{
	.probe		= qtouch_ts_probe,
	.remove		= qtouch_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= qtouch_ts_suspend,
	.resume		= qtouch_ts_resume,
#endif
	.id_table	= qtouch_ts_id,
	.driver = 
	{
		.name	= QTOUCH_TS_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __devinit qtouch_ts_init(void)
{
	qtouch_ts_wq = NULL;
/*
	qtouch_ts_wq = create_singlethread_workqueue("qtouch_obp_ts_wq");
	if (qtouch_ts_wq == NULL) 
	{
		QTOUCH_ERR("%s: No memory for qtouch_ts_wq\n", __func__);
		return -ENOMEM;
	}
*/
	qtouch_tsdebug = 0x00;
	return i2c_add_driver(&qtouch_ts_driver);
}

static void __exit qtouch_ts_exit(void)
{
	i2c_del_driver(&qtouch_ts_driver);
	if (qtouch_ts_wq)
		destroy_workqueue(qtouch_ts_wq);
}

/*!
 * @brief Logs data
 *
 * This function is called as a replacement to printk
 *
 * @param fmt Text of message to log
 */

static void qtouch_printk (int dbg, char *fmt, ...)
{
	if ( dbg )
	{
		static va_list args;
		va_start(args, fmt);
		vprintk(fmt, args);
		va_end(args);
	}
}

module_init(qtouch_ts_init);
module_exit(qtouch_ts_exit);

MODULE_AUTHOR("Dima Zavin <dima@android.com>");
MODULE_DESCRIPTION("Quantum OBP Touchscreen Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("2.0");
