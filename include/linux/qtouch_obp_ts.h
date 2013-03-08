/*
 * include/linux/qtouch_obp_ts.h - platform/protocol data for Quantum touch IC
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

#ifndef _LINUX_QTOUCH_OBP_TS_H
#define _LINUX_QTOUCH_OBP_TS_H

#ifndef FALSE
#define	FALSE	0
#define	TRUE	(!FALSE)
#endif

#define OFF	FALSE
#define	ON	(!OFF)

/* define device name for data transfers to IC */
#define QTOUCH_PF_NAME   "touchpad"

/* Zeppelin Specific IOCTLs start from 40 */
#define QTOUCH_IOCTL_SET_MESSAGE_PTR		40
#define QTOUCH_IOCTL_GET_TOUCH_SENS_DATA		41
#define QTOUCH_IOCTL_SAVE_CONFIG                 42
#define QTOUCH_IOCTL_CAL_STATUS                  43
#define QTOUCH_IOCTL_GET_BAD_CRC_COUNT           44

/* Bootloader statuses */
#define	QTOUCH_BL_WAITING_FOR_NOTHING	0
#define	QTOUCH_BL_WAITING_FOR_COMMAND	1
#define	QTOUCH_BL_WAITING_FOR_DATA		2
#define QTOUCH_BL_WAITING_FOR_CRC		3
#define QTOUCH_BL_GOT_BAD_CRC			4
#define QTOUCH_BL_WAITING_AFTER_BAD_CRC	5
#define QTOUCH_BL_WAITING_AFTER_GOOD_CRC	6
#define QTOUCH_BL_WAITING_FAILED			7

/* Status of driver. */
#define QTOUCH_MODE_UNKNOWN				-1
#define QTOUCH_MODE_GET					0
#define QTOUCH_MODE_NORMAL				1
#define	QTOUCH_MODE_BOOTLOADER			2
#define	QTOUCH_MODE_OBP					3

/* Control function */
#define 	QTOUCH_CNTRL_UNKNOWN		0
#define 	QTOUCH_CNTRL_RESET			1
#define		QTOUCH_CNTRL_CALIBRATE		2
#define		QTOUCH_CNTRL_SELFTEST_ON	3
#define		QTOUCH_CNTRL_SELFTEST_OFF	4
#define		QTOUCH_CNTRL_SELFTEST_GET	5

/* Status of SELFTEST */
#define		QTOUCH_SELFTEST_UNKNOWN		0
#define		QTOUCH_SELFTEST_INPROGRESS	1
#define		QTOUCH_SELFTEST_DONE		2

/* Status of SENSITIVITY */
#define		QTOUCH_SENSITIVITY_UNKNOWN	0
#define		QTOUCH_SENSITIVITY_CHCOUNT	1
#define		QTOUCH_SENSITIVITY_GETREF	2
#define		QTOUCH_SENSITIVITY_NEXTPAGE	3

#define		QTOUCH_DIAG_PAGE_UP			0x01
#define		QTOUCH_DIAG_PAGE_DOWN		0x02
#define		QTOUCH_DIAG_DELTAS_MODE		0x10
#define		QTOUCH_DIAG_REFERENCE_MODE	0x11
#define		QTOUCH_DIAG_CTE_MODE		0x31
/* Status of IRQ */
#define	QTOUCH_IRQ_UNKNOWN		-1
#define QTOUCH_IRQ_OFF			0
#define QTOUCH_IRQ_ON			1
#define QTOUCH_IRQ_GET			2

/* Status of SUSPEND */
#define	QTOUCH_SUSPEND_UNKNOWN		-1
#define QTOUCH_SUSPEND_OFF			0
#define QTOUCH_SUSPEND_ON			1
#define QTOUCH_SUSPEND_GET			2

/* Status of FIRMAWARE transfers */
#define	QTOUCH_FM_DOWNLOAD_SUCCESS		0x00
#define QTOUCH_FM_DOWNLOAD_STILL_ACTIVE	0x01
#define	QTOUCH_FM_DOWNLOAD_FAILED		0x02
#define	QTOUCH_FM_DOWNLOAD_NOT_STARTED	0x03
#define	QTOUCH_FM_DOWNLOAD_GET			0x0F

/* Status of CONFIGURATION transfers */
#define	QTOUCH_CFG_DOWNLOAD_SUCCESS		0x00
#define QTOUCH_CFG_DOWNLOAD_STILL_ACTIVE	0x10
#define	QTOUCH_CFG_DOWNLOAD_FAILED		0x20
#define	QTOUCH_CFG_DOWNLOAD_NOT_STARTED	0x30
#define	QTOUCH_CFG_DOWNLOAD_GET			0xF0

#define NUMBER_OF_FINGERS		4

/* States for upgrading firmware. Bootloader (BL) states*/
#define BOOTLOADER_STATE_NOT_FOUND              0
#define BOOTLOADER_STATE_NOT_FOUND_SEARCH_ADDR1 1
#define BOOTLOADER_STATE_NOT_FOUND_SEARCH_ADDR2 2
#define BOOTLOADER_STATE_FOUND_CHECK_VER        3
#define BOOTLOADER_STATE_APPCRC_FAIL            4
#define BOOTLOADER_STATE_WAIT_UNLOCK_CMD        5
#define BOOTLOADER_STATE_READ_BEFORE_WAIT_FRAME 6
#define BOOTLOADER_STATE_WAIT_FRAME             7
#define BOOTLOADER_STATE_SENDING_FRAME_DATA     8
#define BOOTLOADER_STATE_WAIT_CRC_CHECK         9
#define BOOTLOADER_STATE_WAIT_CRC_RESULT        10
#define BOOTLOADER_STATE_WAIT_CHANGE_LINE       11

/* Bootloader statuses */
#define	OBP_DL_WAITING_FOR_NOTHING	0
#define	OBP_DL_WAITING_FOR_COMMAND	1
#define	OBP_DL_WAITING_FOR_DATA		2
#define OBP_DL_WAITING_FOR_CRC		3
#define OBP_DL_GOT_BAD_CRC			4
#define OBP_DL_WAITING_AFTER_BAD_CRC	5
#define OBP_DL_WAITING_AFTER_GOOD_CRC	6
#define OBP_DL_WAITING_FAILED			7

#define QTOUCH_TS_NAME "qtouch-obp-ts"

#define XMEGAT_I2C_ADDR			0x4A
#define XMEGAT_I2C_ADDR_ALT		0x4B
#define XMEGAT_BL_I2C_ADDR		0x24
#define XMEGAT_BL_I2C_ADDR_ALT	0x25
#define XMEGAT_INT_N_SIGNAL	41

#define XMEGAT_GPIO_PORT_RESET	5
#define XMEGAT_GPIO_PIN_RESET	4
#define XMEGAT_GPIO_PORT_INTR	5
#define XMEGAT_GPIO_PIN_INTR	5

#define	QTOUCH_RST_NAME			"ts_rst"
#define	QTOUCH_INT_NAME			"ts_intr"
#define	QTOUCH_WAKE_NAME		"ts_wake"

#define QTM_OBP_ID_INFO_ADDR		0

#define QTM_OBP_BOOT_CMD_MASK		0xC0
#define QTM_OBP_BOOT_VERSION_MASK	0x3F
#define QTM_OBP_BOOT_WAIT_FOR_DATA	0x80
#define QTM_OBP_BOOT_WAIT_ON_BOOT_CMD	0xC0
#define QTM_OBP_BOOT_CRC_CHECK		0x02
#define QTM_OBP_BOOT_CRC_FAIL		0x03
#define QTM_OBP_BOOT_CRC_PASSED		0x04

#define QTM_OBP_SLEEP_WAIT_FOR_BOOT	100
#define QTM_OBP_SLEEP_WAIT_FOR_RESET	500
#define QTM_OBP_SLEEP_WAIT_FOR_BACKUP	500
#define QTM_OBP_SLEEP_WAIT_FOR_CKSUM	1000
#define QTM_OBP_SLEEP_RESET_HOLD	20
#define QTM_OBP_SLEEP_WAIT_FOR_HW_RESET	40
#define QTM_OBP_SLEEP_WAIT_FOR_DIAG_DATA	5

#define QTM_OBP_FAMILY_XMEGAT	0x80
#define QTM_OBP_NUM_DIAG_READ_RETRIES	10
#define QTM_OBP_DIAG_BUFFER_SIZE	128
#define QTM_OBP_DIAG_NUM_X_LINES	20
#define QTM_OBP_DIAG_CALIB_WAIT_MS	1500

enum {
	QTM_OBJ_RESERVED0		= 0,
	QTM_OBJ_RESERVED1		= 1,
	QTM_OBJ_DBG_DELTAS		= 2,
	QTM_OBJ_DBG_REFS		= 3,
	QTM_OBJ_DBG_SIGS		= 4,
	QTM_OBJ_GEN_MSG_PROC		= 5,
	QTM_OBJ_GEN_CMD_PROC		= 6,
	QTM_OBJ_GEN_PWR_CONF		= 7,
	QTM_OBJ_GEN_ACQUIRE_CONF	= 8,
	QTM_OBJ_TOUCH_MULTI		= 9,
	QTM_OBJ_TOUCH_SINGLE		= 10,
	QTM_OBJ_TOUCH_XSLIDER		= 11,
	QTM_OBJ_TOUCH_SLIDER		= 12,
	QTM_OBJ_TOUCH_XWHEEL		= 13,
	QTM_OBJ_TOUCH_YWHEEL		= 14,
	QTM_OBJ_TOUCH_KEYARRAY		= 15,
	QTM_OBJ_PROCG_SIG_FILTER	= 16,
	QTM_OBJ_PROCI_LINEAR_TBL	= 17,
	QTM_OBJ_SPT_COM_CONFIG		= 18,
	QTM_OBJ_SPT_GPIO_PWM		= 19,
	QTM_OBJ_PROCI_GRIPFACESUPPRESSION = 20,
	QTM_OBJ_RESERVED3		= 21,
	QTM_OBJ_PROCG_NOISE_SUPPRESSION	= 22,
	QTM_OBJ_TOUCH_PROXIMITY		= 23,
	QTM_OBJ_PROCI_ONE_TOUCH_GESTURE_PROC = 24,
	QTM_OBJ_SPT_SELF_TEST		= 25,
	QTM_OBJ_DEBUG_CTE_RANGE		= 26,
	QTM_OBJ_PROCI_TWO_TOUCH_GESTURE_PROC = 27,
	QTM_OBJ_SPT_CTE_CONFIG		= 28,
	QTM_OBJ_GPI					= 29,
	QTM_OBJ_GATE				= 30,
	QTM_OBJ_KEYSET				= 31,
	QTM_OBJ_XSLIDERSET			= 32,

	QTM_OBJ_NOISESUPPRESSION_1	= 36,
	QTM_OBJ_DEBUG_DIAGNOSTICS_T37	= 37,

	/* Max number of objects currently defined */
	QTM_OBP_MAX_OBJECT_NUM = QTM_OBJ_DEBUG_DIAGNOSTICS_T37 + 1,
};

/* OBP structures as defined by the wire protocol. */

/* Note: Not all the structures below need an explicit packed attribute since
 * many of them just contain uint8_t's. However, the protocol is defined in
 * such a way that the structures may expand in the future, with
 * potential multi-byte fields. Thus, we will mark them all as packed to
 * minimize silly bugs in the future.
 */

/* part of the info block */
struct qtm_id_info 
{
	uint8_t			family_id;
	uint8_t			variant_id;
	uint8_t			version;
	uint8_t			build;
	uint8_t			matrix_x_size;
	uint8_t			matrix_y_size;
	uint8_t			num_objs;
} __attribute__ ((packed));

/* an entry in the ote table */
struct qtm_obj_entry 
{
	uint8_t			type;
	uint16_t		addr;
	uint8_t			size;
	uint8_t			num_inst;
	uint8_t			num_rids;
} __attribute__ ((packed));

struct qtm_object 
{
	struct qtm_obj_entry		entry;
	uint8_t				report_id_min;
	uint8_t				report_id_max;
};


/*******************************/
/*********** messages **********/
/*******************************/

/* generic message received from the message_processor object. size/buffer
 * defined at runtime after reading the info block */
struct qtm_obj_message 
{
	uint8_t			report_id;
	uint8_t			msg[0];
} __attribute__ ((packed));

/* status message sent by the command processor - T6 */
#define QTM_CMD_PROC_STATUS_RESET	(1 << 7)
#define QTM_CMD_PROC_STATUS_OFL		(1 << 6)
#define QTM_CMD_PROC_STATUS_SIGERR	(1 << 5)
#define QTM_CMD_PROC_STATUS_CAL		(1 << 4)
#define QTM_CMD_PROC_STATUS_CFGERR	(1 << 3)
struct qtm_cmd_proc_msg 
{
	uint8_t			report_id;
	uint8_t			status;
	uint8_t			checksum[3];
} __attribute__ ((packed));

/* status message sent by the mutlitouch touch object - T9*/
#define QTM_TOUCH_MULTI_STATUS_TOUCH		(1 << 7)
#define QTM_TOUCH_MULTI_STATUS_PRESS		(1 << 6)
#define QTM_TOUCH_MULTI_STATUS_RELEASE		(1 << 5)
#define QTM_TOUCH_MULTI_STATUS_MOVE		(1 << 4)
#define QTM_TOUCH_MULTI_STATUS_VECTOR		(1 << 3)
#define QTM_TOUCH_MULTI_STATUS_AMPLITUDE	(1 << 2)
struct qtm_touch_multi_msg 
{
	uint8_t			report_id;
	uint8_t			status;
	uint8_t			xpos_msb;
	uint8_t			ypos_msb;
	uint8_t			xypos_lsb;
	uint8_t			touch_area;
	uint8_t			touch_amp;
	uint8_t			touch_vect;
} __attribute__ ((packed));

/* status message sent by the keyarray touch object - T15 */
#define QTM_TOUCH_KEYARRAY_STATUS_TOUCH		(1 << 7)
struct qtm_touch_keyarray_msg 
{
	uint8_t			report_id;
	uint8_t			status;
	uint32_t		keystate;
} __attribute__ ((packed));

/*******************************/
/**** configuration objects ****/
/*******************************/

/* GEN_COMMANDPROCESSOR_T6 */
struct qtm_gen_cmd_proc 
{
	uint8_t			reset;
	uint8_t			backupnv;
	uint8_t			calibrate;
	uint8_t			reportall;
	uint8_t			debugctrl;
	uint8_t			diagnostic;
} __attribute__ ((packed));

/* GEN_POWERCONFIG_T7 */
struct qtm_gen_power_cfg 
{
	uint8_t			idle_acq_int;      /* in ms */
	uint8_t			active_acq_int;    /* in ms */
	uint8_t			active_idle_to;    /* in 200ms */
} __attribute__ ((packed));

/* GEN_ACQUIRECONFIG_T8 */
struct qtm_gen_acquire_cfg 
{
	uint8_t			charge_time;       /* in 250ns */
	uint8_t			atouch_drift;      /* in 200ms */
	uint8_t			touch_drift;       /* in 200ms */
	uint8_t			drift_susp;        /* in 200ms */
	uint8_t			touch_autocal;     /* in 200ms */
	uint8_t			sync;
	uint8_t			atch_cal_suspend_time;
	uint8_t			atch_cal_suspend_thres;
} __attribute__ ((packed));

/* TOUCH_MULTITOUCHSCREEN_T9 */
struct qtm_touch_multi_cfg 
{
	uint8_t			ctrl;
	uint8_t			x_origin;
	uint8_t			y_origin;
	uint8_t			x_size;
	uint8_t			y_size;
	uint8_t			aks_cfg;
	uint8_t			burst_len;
	uint8_t			tch_det_thr;
	uint8_t			tch_det_int;
	uint8_t			orient;
	uint8_t			mrg_to;
	uint8_t			mov_hyst_init;
	uint8_t			mov_hyst_next;
	uint8_t			mov_filter;
	uint8_t			num_touch;
	uint8_t			merge_hyst;
	uint8_t			merge_thresh;
	uint8_t			amp_hyst;
	uint16_t		x_res;
	uint16_t		y_res;
	uint8_t			x_low_clip;
	uint8_t			x_high_clip;
	uint8_t			y_low_clip;
	uint8_t			y_high_clip;
	uint8_t			x_edge_ctrl;
	uint8_t			x_edge_dist;
	uint8_t			y_edge_ctrl;
	uint8_t			y_edge_dist;
} __attribute__ ((packed));

/* TOUCH_KEYARRAY_T15 */
struct qtm_touch_keyarray_cfg 
{
	uint8_t			ctrl;
	uint8_t			x_origin;
	uint8_t			y_origin;
	uint8_t			x_size;
	uint8_t			y_size;
	uint8_t			aks_cfg;
	uint8_t			burst_len;
	uint8_t			tch_det_thr;
	uint8_t			tch_det_int;
	uint8_t			rsvd1;
	uint8_t			rsvd2;
} __attribute__ ((packed));

/* PROCG_SIGNALFILTER_T16 */
struct qtm_procg_sig_filter_cfg 
{
	uint8_t			slew;
	uint8_t			median;
	uint8_t			iir;
} __attribute__ ((packed));

/* PROCI_LINEARIZATIONTABLE_T17 */
struct qtm_proci_linear_tbl_cfg 
{
	uint8_t			ctrl;
	uint16_t		x_offset;
	uint8_t			x_segment[16];
	uint16_t		y_offset;
	uint8_t			y_segment[16];
} __attribute__ ((packed));

/* Motorola - BEGIN - qa2113 - 11/18/2009 - XmegaT firmware 1.5 support */
/* SPT_COMMSCONFIG _T18 */
struct spt_comms_config_cfg 
{
	uint8_t			ctrl;
	uint8_t			command;
} __attribute__ ((packed));
/* Motorola - END - qa2113 - 11/18/2009 - XmegaT firmware 1.5 support */

/* SPT_GPIOPWM_T19*/
struct qtm_spt_gpio_pwm_cfg 
{
	uint8_t			ctrl;
	uint8_t			report_mask;
	uint8_t			pin_direction;
	uint8_t			internal_pullup;
	uint8_t			output_value;
	uint8_t			wake_on_change;
	uint8_t			pwm_enable;
	uint8_t			pwm_period;
	uint8_t			duty_cycle_0;
	uint8_t			duty_cycle_1;
	uint8_t			duty_cycle_2;
	uint8_t			duty_cycle_3;
	/* Motorola - BEGIN - qa2113 - 11/18/2009 - XmegaT firmware 1.5 support */
	uint8_t			trigger_0;
	uint8_t			trigger_1;
	uint8_t			trigger_2;
	uint8_t			trigger_3;
	/* Motorola - END - qa2113 - 11/18/2009 - XmegaT firmware 1.5 support */
} __attribute__ ((packed));

/* PROCI_GRIPFACESUPPRESSION_T20 */
struct qtm_proci_grip_suppression_cfg 
{
	uint8_t			ctrl;
	uint8_t			xlogrip;
	uint8_t			xhigrip;
	uint8_t			ylogrip;
	uint8_t			yhigrip;
	uint8_t			maxtchs;
	uint8_t			reserve0;
	uint8_t			szthr1;
	uint8_t			szthr2;
	uint8_t			shpthr1;
	uint8_t			shpthr2;
	uint8_t			supextto;
} __attribute__ ((packed));

/* PROCG_NOISESUPPRESSION_T22 */
struct qtm_procg_noise_suppression_cfg 
{
	uint8_t			ctrl;
	uint8_t			outlier_filter_len;
	uint8_t			reserve0;
	uint16_t		gcaf_upper_limit;
	uint16_t		gcaf_lower_limit;
	uint8_t			gcaf_low_count;
	uint8_t			noise_threshold;
	uint8_t			reserve1;
	uint8_t			freq_hop_scale;
	uint8_t			burst_freq_0;
	uint8_t			burst_freq_1;
	uint8_t			burst_freq_2;
	uint8_t			burst_freq_3;
	uint8_t			burst_freq_4;
	uint8_t			idle_gcaf_valid;
} __attribute__ ((packed));

/* TOUCH_PROXIMITY_T23 */
struct qtm_touch_proximity_cfg 
{
	uint8_t			ctrl;
	uint8_t			x_origin;
	uint8_t			y_origin;
	uint8_t			x_size;
	uint8_t			y_size;
	uint8_t			reserve0;
	uint8_t			blen;
	uint16_t		tch_thresh;
	uint8_t			tch_detect_int;
	uint8_t			average;
	uint16_t		rate;
} __attribute__ ((packed));

/* PROCI_ONETOUCHGESTUREPROCESSOR_T24 */
struct qtm_proci_one_touch_gesture_proc_cfg 
{
	uint8_t			ctrl;
	uint8_t			numgest;
	uint16_t		gesture_enable;
	uint8_t			pres_proc;
	uint8_t			tap_time_out;
	uint8_t			flick_time_out;
	uint8_t			drag_time_out;
	uint8_t			short_press_time_out;
	uint8_t			long_press_time_out;
	uint8_t			repeat_press_time_out;
	uint16_t		flick_threshold;
	uint16_t		drag_threshold;
	uint16_t		tap_threshold;
	uint16_t		throw_threshold;
} __attribute__ ((packed));

/* SPT_SELFTEST_T25 */
struct qtm_spt_self_test_cfg 
{
	uint8_t			ctrl;
	uint8_t			command;
	uint16_t		high_signal_limit_0;
	uint16_t		low_signal_limit_0;
	uint16_t		high_signal_limit_1;
	uint16_t		low_signal_limit_1;
	uint16_t		high_signal_limit_2;
	uint16_t		low_signal_limit_2;
} __attribute__ ((packed));

struct qtm_spt_self_test_rslt 
{
	uint8_t			ctrl;
	uint8_t			command;
	uint8_t			status;
	uint8_t			info[5];
} __attribute__ ((packed));

/* PROCI_TWOTOUCHGESTUREPROCESSOR_T27 */
struct qtm_proci_two_touch_gesture_proc_cfg 
{
	uint8_t			ctrl;
	uint8_t			numgest;
	uint16_t		gesture_enable;
	uint8_t			rotate_threshold;
	uint16_t		zoom_threshold;
	uint8_t			tcheventto;
} __attribute__ ((packed));

/* SPT_CTECONFIG_T28 */
struct qtm_spt_cte_config_cfg 
{
	uint8_t			ctrl;
	uint8_t			command;
	uint8_t			mode;
	uint8_t			idle_gcaf_depth;
	uint8_t			active_gcaf_depth;
	/* Motorola - BEGIN - qa2113 - 11/18/2009 - XmegaT firmware 1.5 support */
	uint8_t			voltage;
	/* Motorola - BEGIN - qa2113 - 11/18/2009 - XmegaT firmware 1.5 support */
} __attribute__ ((packed));

/* QTM_OBJ_NOISESUPPRESSION_1 */
struct qtm_proci_noise1_suppression_cfg 
{
	uint8_t			ctrl;
	uint8_t			version;
	uint8_t			atchthr;
	uint8_t			duty_cycle;
	uint8_t			drift_thr;
	uint8_t			clamp_thr;
	uint8_t			diff_thr;
	uint8_t			adjustment;
	uint16_t		average;
	uint8_t			temp;
	uint8_t			offset[168];
	uint8_t			bad_chan[11];
	uint8_t			x_short;
} __attribute__ ((packed));

/* QTM_OBJ_DEBUG_DIAGNOSTICS_T37 */
struct qtm_debug_diagnostics
{
	uint8_t			mode;
	uint8_t			page;
	uint8_t			data[128];
} __attribute__ ((packed));

/*******************************/
/******** platform data ********/
/*******************************/

struct vkey 
{
	int     code;
	int     center_x;
	int     center_y;
	int     width;
	int     height;
};

struct virt_keys 
{
	int			count;
	struct vkey		*keys;
};

struct qtouch_key 
{
	uint8_t				channel;
	int				code;
	int				x_coord;
	int				y_coord;
};

struct qtouch_key_array {
	struct qtm_touch_keyarray_cfg	*cfg;
	struct qtouch_key		*keys;
	int				num_keys;
};

#define QTOUCH_FLIP_X		(1 << 0)
#define QTOUCH_FLIP_Y		(1 << 1)
#define QTOUCH_SWAP_XY		(1 << 2)
#define QTOUCH_USE_MULTITOUCH	(1 << 3)
#define QTOUCH_USE_KEYARRAY	(1 << 4)
#define QTOUCH_CFG_BACKUPNV	(1 << 5)
#define QTOUCH_EEPROM_CHECKSUM  (1 << 6)

#define	BUTTON_MAX	4

struct qtouch_buttons
{
	int	minX;
	int	maxX;
	int	minY;
	int	maxY;
	int	key;
	int	exists;
	int	pressed;
};

struct qtouch_ts_platform_data 
{
	uint32_t		flags;
	unsigned long		irqflags;

	int			gpio_reset;
	int			gpio_intr;

	uint32_t		abs_min_x;
	uint32_t		abs_max_x;
	uint32_t		abs_min_y;
	uint32_t		abs_max_y;
	uint32_t		abs_min_p;
	uint32_t		abs_max_p;
	uint32_t		abs_min_w;
	uint32_t		abs_max_w;

	uint32_t		x_delta;
	uint32_t		y_delta;

	uint32_t		nv_checksum;

	uint32_t		fuzz_x;
	uint32_t		fuzz_y;
	uint32_t		fuzz_p;
	uint32_t		fuzz_w;

	int			(*hw_reset)(void);

	/* TODO: allow multiple key arrays */
	struct qtouch_key_array			key_array;
	/* TODO: I don't like current way, but because of lack of time.... */
	int		buttons_count;
	struct	qtouch_buttons	buttons[BUTTON_MAX+1];

	/* object configuration information from board */
	struct qtm_gen_power_cfg			power_cfg;
	struct qtm_gen_acquire_cfg			acquire_cfg;
	struct qtm_touch_multi_cfg			multi_touch_cfg;
	struct qtm_procg_sig_filter_cfg			sig_filter_cfg;
	struct qtm_proci_linear_tbl_cfg			linear_tbl_cfg;
	/* Motorola - BEGIN - qa2113 - 11/18/2009 - XmegaT firmware 1.5 support */
	struct spt_comms_config_cfg			comms_config_cfg;
	/* Motorola - END - qa2113 - 11/18/2009 - XmegaT firmware 1.5 support */
	struct qtm_spt_gpio_pwm_cfg			gpio_pwm_cfg;
	struct qtm_proci_grip_suppression_cfg		grip_suppression_cfg;
	struct qtm_procg_noise_suppression_cfg		noise_suppression_cfg;
	struct qtm_touch_proximity_cfg			touch_proximity_cfg;
	struct qtm_proci_one_touch_gesture_proc_cfg	one_touch_gesture_proc_cfg;
	struct qtm_spt_self_test_cfg			self_test_cfg;
	struct qtm_proci_two_touch_gesture_proc_cfg	two_touch_gesture_proc_cfg;
	struct qtm_spt_cte_config_cfg			cte_config_cfg;
	struct qtm_proci_noise1_suppression_cfg		noise1_suppression_cfg;
	struct qtm_debug_diagnostics			debug_diag;

	struct virt_keys	vkeys;
};

/* IOCTL related structures */

#define QTOUCH_IOCTL_CMD		0x51
enum 
{
/* Implemented */
	QTOUCH_IOCTL_SET_DEBUG		= 0,
	QTOUCH_IOCTL_GET_DEBUG,
	QTOUCH_IOCTL_GET_VERSION,
	QTOUCH_IOCTL_MODE,
	QTOUCH_IOCTL_CNTRL,
	QTOUCH_IOCTL_IRQ,
	QTOUCH_IOCTL_GET_REGISTER,
	QTOUCH_IOCTL_SET_REGISTER,
	QTOUCH_IOCTL_SET_OBJECT,
	QTOUCH_IOCTL_PUSH_CONFIG,
	QTOUCH_IOCTL_GET_CONFIG,
	QTOUCH_IOCTL_DL_GET_STATUS,
	QTOUCH_IOCTL_SUSPEND,
	QTOUCH_IOCTL_GET_POINT,
	QTOUCH_IOCTL_GET_CMD_STATUS,
	QTOUCH_IOCTL_SET_CMD_STATUS,
	QTOUCH_IOCTL_GET_SENS,
/* Not implemented */
	QTOUCH_IOCTL_SET_IN_CALL,
	QTOUCH_IOCTL_SET_NOT_IN_CALL,
	QTOUCH_IOCTL_END_FIRMWARE,
	QTOUCH_IOCTL_DISPLAY_ONE_POINT,
	QTOUCH_IOCTL_DISABLE,
	QTOUCH_IOCTL_ENABLE,
	QTOUCH_IOCTL_READ_REGISTER16,
	QTOUCH_IOCTL_READ_4_REGS,
	QTOUCH_IOCTL_SET_AREA,
	QTOUCH_IOCTL_SET_GMAXX,
	QTOUCH_IOCTL_SET_GMINX,
	QTOUCH_IOCTL_SET_GMAXY,
	QTOUCH_IOCTL_SET_GMINY,

	QTOUCH_IOCTL_LAST,
};

struct coordinate_map 
{
	int x_data;
	int y_data;
	int z_data;
	int w_data;
	int down;
};

typedef struct	a_reg
{
	int	reg;
	int	value;
	int	vals[4];
} A_REG, *A_REG_PTR;

struct qtm_ioctl_version
{
	int	FamilyId;
	int	VariantId;
	int	Version;
	int	Build;
	int	Mode;
};

struct qtm_ioctl_gen_power_cfg 
{
/* Each bit represents a change status of the field */
	uint32_t		changed;	
	union
	{
		struct 	qtm_gen_power_cfg	t7;
		uint8_t	rawData[sizeof(struct qtm_gen_power_cfg)];
	} data;
};

struct qtm_ioctl_gen_acquire_cfg 
{
/* Each bit represents a change status of the field */
	uint32_t		changed;	
	union
	{
		struct	qtm_gen_acquire_cfg	t8;
		uint8_t	rawData[sizeof(struct qtm_gen_acquire_cfg)];
	} data;
};

struct qtm_ioctl_touch_multi_cfg 
{
/* Each bit represents a change status of the field */
	uint32_t		changed;	
	union
	{
		struct 	qtm_touch_multi_cfg	t9;
		uint8_t	rawData[sizeof(struct qtm_touch_multi_cfg)];
	} data;
};

struct qtm_ioctl_touch_keyarray_cfg  
{
/* Each bit represents a change status of the field */
	uint32_t		changed;	
	union
	{
		struct 	qtm_touch_keyarray_cfg 	t15;
		uint8_t	rawData[sizeof(struct qtm_touch_keyarray_cfg)];
	} data;
};

struct qtm_ioctl_spt_gpio_pwm_cfg  
{
/* Each bit represents a change status of the field */
	uint32_t		changed;	
	union
	{
		struct qtm_spt_gpio_pwm_cfg 	t19;
		uint8_t	rawData[sizeof(struct qtm_spt_gpio_pwm_cfg)];
	} data;
};
struct qtm_ioctl_proci_grip_suppression_cfg  
{
/* Each bit represents a change status of the field */
	uint32_t		changed;	
	union
	{
		struct qtm_proci_grip_suppression_cfg 	t20;
		uint8_t	rawData[sizeof(struct qtm_proci_grip_suppression_cfg)];
	} data;
};

struct qtm_ioctl_procg_noise_suppression_cfg
{
/* Each bit represents a change status of the field */
	uint32_t		changed;	
	union
	{
		struct qtm_procg_noise_suppression_cfg 	t22;
		uint8_t	rawData[sizeof(struct qtm_procg_noise_suppression_cfg)];
	} data;
};
struct qtm_ioctl_touch_proximity_cfg
{
/* Each bit represents a change status of the field */
	uint32_t		changed;	
	union
	{
		struct qtm_touch_proximity_cfg 	t23;
		uint8_t	rawData[sizeof(struct qtm_touch_proximity_cfg)];
	} data;
};
struct qtm_ioctl_proci_one_touch_gesture_proc_cfg
{
/* Each bit represents a change status of the field */
	uint32_t		changed;	
	union
	{
		struct qtm_proci_one_touch_gesture_proc_cfg 	t24;
		uint8_t	rawData[sizeof(struct qtm_proci_one_touch_gesture_proc_cfg)];
	} data;
};
struct qtm_ioctl_spt_self_test_cfg
{
/* Each bit represents a change status of the field */
	uint32_t		changed;	
	union
	{
		struct qtm_spt_self_test_cfg 	t25;
		uint8_t	rawData[sizeof(struct qtm_spt_self_test_cfg)];
	} data;
};
struct qtm_ioctl_proci_two_touch_gesture_proc_cfg
{
/* Each bit represents a change status of the field */
	uint32_t		changed;	
	union
	{
		struct qtm_proci_two_touch_gesture_proc_cfg 	t27;
		uint8_t	rawData[sizeof(struct qtm_proci_two_touch_gesture_proc_cfg)];
	} data;
};
struct qtm_ioctl_spt_cte_config_cfg
{
/* Each bit represents a change status of the field */
	uint32_t		changed;	
	union
	{
		struct qtm_spt_cte_config_cfg 	t28;
		uint8_t	rawData[sizeof(struct qtm_spt_cte_config_cfg)];
	} data;
};

struct	op_mode
{
	int	op;
	int	dl;
};

struct	selftest
{
	char	status;
	char	info[5];
};

typedef union 
{
	int								debug;
	int								irq;
	int								objId;
	int								info;
	char							suspend;
	char							blRecord[300];
	struct qtm_debug_diagnostics	sens;
	struct selftest					selftst;
	struct coordinate_map 			currPoints[NUMBER_OF_FINGERS];
	struct op_mode					mode;
	struct a_reg					reg;
	struct qtm_ioctl_version		ioctl_version;
	struct qtm_cmd_proc_msg 		cmd_proc_msg;
	struct qtm_touch_multi_msg		touch_multi_msg;
	struct qtm_touch_keyarray_msg	touch_keyarray_msg;
	struct qtm_gen_cmd_proc 		gen_cmd_proc_t6;
	struct qtm_ioctl_gen_power_cfg 		qtm_t7;
	struct qtm_ioctl_gen_acquire_cfg 	qtm_t8;
	struct qtm_ioctl_touch_multi_cfg 	qtm_t9;
	struct qtm_ioctl_touch_keyarray_cfg 	qtm_t15;
	struct qtm_procg_sig_filter_cfg procg_sig_filter_cfg;
	struct qtm_proci_linear_tbl_cfg proci_linear_tbl_cfg;
	struct spt_comms_config_cfg 	mot_spt_comms_config_cfg;
	struct qtm_ioctl_spt_gpio_pwm_cfg 	qtm_t19;
	struct qtm_ioctl_proci_grip_suppression_cfg 	qtm_t20;
	struct qtm_ioctl_procg_noise_suppression_cfg 	qtm_t22;
	struct qtm_ioctl_touch_proximity_cfg 	qtm_t23;
	struct qtm_ioctl_proci_one_touch_gesture_proc_cfg qtm_t24;
	struct qtm_ioctl_spt_self_test_cfg 	qtm_t25;
	struct qtm_ioctl_proci_two_touch_gesture_proc_cfg 	qtm_t27;
	struct qtm_ioctl_spt_cte_config_cfg 	qtm_t28;
	struct qtm_proci_noise1_suppression_cfg 	proci_noisel_suppression_cfg;
} QTM_ALL_OBJECTS;

typedef struct qtim_ioctl_data
{
	int		cmd;
	int		type;
	QTM_ALL_OBJECTS data;
} IOCTL_DATA;

#endif /* _LINUX_QTOUCH_OBP_TS_H */

