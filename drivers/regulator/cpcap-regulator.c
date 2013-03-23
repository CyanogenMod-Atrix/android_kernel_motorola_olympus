/*
 * Copyright (C) 2009-2011 Motorola, Inc.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>

#include <linux/spi/spi.h>
#include <linux/spi/cpcap.h>
#include <linux/spi/cpcap-regbits.h>

#define CPCAP_REGULATOR(_name, _id) 		\
	{					\
		.name = _name, 			\
		.id = _id,			\
		.ops = &cpcap_regulator_ops,	\
		.type = REGULATOR_VOLTAGE, 	\
		.owner = THIS_MODULE, 		\
	}
static const int sw1_val_tbl[] = {750000, 762500, 775000, 787500, 800000, 812500, 825000, 837500, 850000, 862500, 875000, 887500, 900000, 912500, 925000, 937500, 950000, 962500, 975000, 987500, 1000000, 1012500, 1025000, 1037500, 1050000, 1062500, 1075000, 1087500, 1100000};
static const int sw2_val_tbl[] = {900000, 912500, 925000, 937500, 950000, 962500, 975000, 987500, 1000000, 1012500, 1025000, 1037500, 1050000, 1062500, 1075000, 1087500, 1100000, 1112500, 1125000, 1137500, 1150000, 1162500, 1175000, 1187500, 1200000};
static const int sw3_val_tbl[] = {1350000, 1800000, 1850000, 1875000};
static const int sw4_val_tbl[] = {900000, 912500, 925000, 937500, 950000, 962500, 975000, 987500, 1000000, 1012500, 1025000, 1037500, 1050000, 1062500, 1075000, 1087500, 1100000, 1112500, 1125000, 1137500, 1150000, 1162500, 1175000, 1187500, 1200000};
static const int sw5_val_tbl[] = {0, 5050000};
static const int vcam_val_tbl[] = {2600000, 2700000, 2800000, 2900000};
static const int vcsi_val_tbl[] = {1200000, 1800000};
static const int vdac_val_tbl[] = {1200000, 1500000, 1800000, 2500000};
static const int vdig_val_tbl[] = {1200000, 1350000, 1500000, 1875000};
static const int vfuse_val_tbl[] = {1500000, 1600000, 1700000, 1800000, 1900000,
				    2000000, 2100000, 2200000, 2300000, 2400000,
				    2500000, 2600000, 2700000, 3150000};
static const int vhvio_val_tbl[] = {2775000};
static const int vsdio_val_tbl[] = {1500000, 1600000, 1800000, 2600000,
				    2700000, 2800000, 2900000, 3000000};
static const int vpll_val_tbl[] = {1200000, 1300000, 1400000, 1800000};
static const int vrf1_val_tbl[] = {2775000, 2500000}; /* Yes, this is correct */
static const int vrf2_val_tbl[] = {0, 2775000};
static const int vrfref_val_tbl[] = {2500000, 2775000};
static const int vwlan1_val_tbl[] = {1800000, 1900000};
static const int vwlan2_val_tbl[] = {2775000, 3000000, 3300000, 3300000};
static const int vsim_val_tbl[] = {1800000, 2900000};
static const int vsimcard_val_tbl[] = {1800000, 2900000};
static const int vvib_val_tbl[] = {1300000, 1800000, 2000000, 3000000};
static const int vusb_val_tbl[] = {0, 3300000};
static const int vaudio_val_tbl[] = {0, 2775000};

static struct {
	const enum cpcap_reg reg;
	const enum cpcap_reg assignment_reg;
	const unsigned short assignment_mask;
	unsigned short mode_mask;
	const unsigned short volt_mask;
	const unsigned char volt_shft;
	unsigned short mode_val;
	unsigned short off_mode_val;
	const int val_tbl_sz;
	const int *val_tbl;
	unsigned int mode_cntr;
	const unsigned int volt_trans_time; /* in micro seconds */
	const unsigned int turn_on_time; /* in micro seconds */
        /* Bit difference between lowest value in val_tbl and start of voltage table setting in cpcap.
           Use this for switchers that have many too many voltages to list in val_tbl. */
        const unsigned int bit_offset_from_cpcap_lowest_voltage;
} cpcap_regltr_data[CPCAP_NUM_REGULATORS] = {
	[CPCAP_SW1]      = {CPCAP_REG_S1C1,
			    CPCAP_REG_ASSIGN2,
			    CPCAP_BIT_SW1_SEL,
			    0x6F00,	      /* updated below for ST v3.1 HW*/
			    0x007F,
			    0,
			    0x0000,
			    0x0000,
			    ARRAY_SIZE(sw1_val_tbl),
			    sw1_val_tbl,
			    0,
			    0,
			    1500,
                            0x0c},

	[CPCAP_SW2]      = {CPCAP_REG_S2C1,
			    CPCAP_REG_ASSIGN2,
			    CPCAP_BIT_SW2_SEL,
			    0x6F00,	      /* updated below for ST v3.1 HW*/
			    0x007F,
			    0,
			    0x0000,
			    0x0000,
			    ARRAY_SIZE(sw2_val_tbl),
			    sw2_val_tbl,
			    0,
			    0,
			    1500,
                            0x18},

	[CPCAP_SW3]      = {CPCAP_REG_S3C,
			    CPCAP_REG_ASSIGN2,
			    CPCAP_BIT_SW3_SEL,
				0x0578,		/* updated below for ST v3.1 HW*/
				0x0003,
				0,
				0x0000,
				0x0000,
				ARRAY_SIZE(sw3_val_tbl),
				sw3_val_tbl,
				0,
				0,
				0},

	[CPCAP_SW4]      = {CPCAP_REG_S4C1,
			    CPCAP_REG_ASSIGN2,
			    CPCAP_BIT_SW4_SEL,
			    0x6F00,		/* updated below for ST v3.1 HW*/
			    0x007F,
			    0,
			    0x0000,
			    0x0000,
			    ARRAY_SIZE(sw4_val_tbl),
			    sw4_val_tbl,
			    0,
			    0,
			    1500,
                            0x18},

	[CPCAP_SW5]      = {CPCAP_REG_S5C,
			    CPCAP_REG_ASSIGN2,
			    CPCAP_BIT_SW5_SEL,
			    0x0028,
			    0x0000,
			    0,
			    0x0000,
			    0x0000,
			    ARRAY_SIZE(sw5_val_tbl),
			    sw5_val_tbl,
			    0,
			    0,
			    1500,
                            0},

	[CPCAP_VCAM]     = {CPCAP_REG_VCAMC,
			    CPCAP_REG_ASSIGN2,
			    CPCAP_BIT_VCAM_SEL,
			    0x0087,
			    0x0030,
			    4,
			    0x0000,
			    0x0000,
			    ARRAY_SIZE(vcam_val_tbl),
			    vcam_val_tbl,
			    0,
			    420,
			    1000,
                            0},

	[CPCAP_VCSI]     = {CPCAP_REG_VCSIC,
			    CPCAP_REG_ASSIGN3,
			    CPCAP_BIT_VCSI_SEL,
			    0x0047,
			    0x0010,
			    4,
			    0x0000,
			    0x0000,
			    ARRAY_SIZE(vcsi_val_tbl),
			    vcsi_val_tbl,
			    0,
			    350,
			    1000,
                            0},

	[CPCAP_VDAC]     = {CPCAP_REG_VDACC,
			    CPCAP_REG_ASSIGN3,
			    CPCAP_BIT_VDAC_SEL,
			    0x0087,
			    0x0030,
			    4,
			    0x0000,
			    0x0000,
			    ARRAY_SIZE(vdac_val_tbl),
			    vdac_val_tbl,
			    0,
			    420,
			    1000,
                            0},

	[CPCAP_VDIG]     = {CPCAP_REG_VDIGC,
			    CPCAP_REG_ASSIGN2,
			    CPCAP_BIT_VDIG_SEL,
			    0x0087,
			    0x0030,
			    4,
			    0x0000,
			    0x0000,
			    ARRAY_SIZE(vdig_val_tbl),
			    vdig_val_tbl,
			    0,
			    420,
			    1000,
                            0},

	[CPCAP_VFUSE]    = {CPCAP_REG_VFUSEC,
			    CPCAP_REG_ASSIGN3,
			    CPCAP_BIT_VFUSE_SEL,
			    0x00A0,
			    0x000F,
			    0,
			    0x0000,
			    0x0000,
			    ARRAY_SIZE(vfuse_val_tbl),
			    vfuse_val_tbl,
			    0,
			    420,
			    1000,
                            0},

	[CPCAP_VHVIO]    = {CPCAP_REG_VHVIOC,
			    CPCAP_REG_ASSIGN3,
			    CPCAP_BIT_VHVIO_SEL,
			    0x0017,
			    0x0000,
			    0,
			    0x0000,
			    0x0000,
			    ARRAY_SIZE(vhvio_val_tbl),
			    vhvio_val_tbl,
			    0,
			    0,
			    1000,
                            0},

	[CPCAP_VSDIO]    = {CPCAP_REG_VSDIOC,
			    CPCAP_REG_ASSIGN2,
			    CPCAP_BIT_VSDIO_SEL,
			    0x0087,
			    0x0038,
			    3,
			    0x0000,
			    0x0000,
			    ARRAY_SIZE(vsdio_val_tbl),
			    vsdio_val_tbl,
			    0,
			    420,
			    1000,
                            0},

	[CPCAP_VPLL]     = {CPCAP_REG_VPLLC,
			    CPCAP_REG_ASSIGN3,
			    CPCAP_BIT_VPLL_SEL,
//			    0x0047,
			    0x0048,
			    0x0018,
			    3,
			    0x0000,
			    0x0000,
			    ARRAY_SIZE(vpll_val_tbl),
			    vpll_val_tbl,
			    0,
			    420,
			    100,
                            0},

	[CPCAP_VRF1]     = {CPCAP_REG_VRF1C,
			    CPCAP_REG_ASSIGN3,
			    CPCAP_BIT_VRF1_SEL,
			    0x00AC,
			    0x0002,
			    1,
			    0x0000,
			    0x0000,
			    ARRAY_SIZE(vrf1_val_tbl),
			    vrf1_val_tbl,
			    0,
			    10,
			    1000,
                            0},

	[CPCAP_VRF2]     = {CPCAP_REG_VRF2C,
			    CPCAP_REG_ASSIGN3,
			    CPCAP_BIT_VRF2_SEL,
			    0x0023,
			    0x0008,
			    3,
			    0x0000,
			    0x0000,
			    ARRAY_SIZE(vrf2_val_tbl),
			    vrf2_val_tbl,
			    0,
			    10,
			    1000,
                            0},

	[CPCAP_VRFREF]   = {CPCAP_REG_VRFREFC,
			    CPCAP_REG_ASSIGN3,
			    CPCAP_BIT_VRFREF_SEL,
			    0x0023,
			    0x0008,
			    3,
			    0x0000,
			    0x0000,
			    ARRAY_SIZE(vrfref_val_tbl),
			    vrfref_val_tbl,
			    0,
			    420,
			    100,
                            0},

	[CPCAP_VWLAN1]   = {CPCAP_REG_VWLAN1C,
			    CPCAP_REG_ASSIGN3,
			    CPCAP_BIT_VWLAN1_SEL,
			    0x0047,
			    0x0010,
			    4,
			    0x0000,
			    0x0000,
			    ARRAY_SIZE(vwlan1_val_tbl),
			    vwlan1_val_tbl,
			    0,
			    420,
			    1000,
                            0},

	[CPCAP_VWLAN2]   = {CPCAP_REG_VWLAN2C,
			    CPCAP_REG_ASSIGN3,
			    CPCAP_BIT_VWLAN2_SEL,
			    0x020C,
			    0x00C0,
			    6,
			    0x0000,
			    0x0000,
			    ARRAY_SIZE(vwlan2_val_tbl),
			    vwlan2_val_tbl,
			    0,
			    420,
			    1000,
                            0},

	[CPCAP_VSIM]     = {CPCAP_REG_VSIMC,
			    CPCAP_REG_ASSIGN3,
			    0,		/* VSIM and VSIMCARD are on the same control bit, do not allow secondary control. */
			    0x0023,
			    0x0008,
			    3,
			    0x0000,
			    0x0000,
			    ARRAY_SIZE(vsim_val_tbl),
			    vsim_val_tbl,
			    0,
			    420,
			    1000,
                            0},

	[CPCAP_VSIMCARD] = {CPCAP_REG_VSIMC,
			    CPCAP_REG_ASSIGN3,
			    0,		/* VSIM and VSIMCARD are on the same control bit, do not allow secondary control. */
			    0x1E80,
			    0x0008,
			    3,
			    0x0000,
			    0x0000,
			    ARRAY_SIZE(vsimcard_val_tbl),
			    vsimcard_val_tbl,
			    0,
			    420,
			    1000,
                            0},

	[CPCAP_VVIB]     = {CPCAP_REG_VVIBC,
			    CPCAP_REG_ASSIGN3,
			    CPCAP_BIT_VVIB_SEL,
			    0x0001,
			    0x000C,
			    2,
			    0x0000,
			    0x0000,
			    ARRAY_SIZE(vvib_val_tbl),
			    vvib_val_tbl,
			    0,
			    500,
			    500,
                            0},

	[CPCAP_VUSB]     = {CPCAP_REG_VUSBC,
			    CPCAP_REG_ASSIGN3,
			    CPCAP_BIT_VUSB_SEL,
			    0x011C,
			    0x0040,
			    6,
			    0x0000,
			    0x0000,
			    ARRAY_SIZE(vusb_val_tbl),
			    vusb_val_tbl,
			    0,
			    0,
			    1000,
                            0},

	[CPCAP_VAUDIO]   = {CPCAP_REG_VAUDIOC,
			    CPCAP_REG_ASSIGN4,
			    CPCAP_BIT_VAUDIO_SEL,
			    0x0016,
			    0x0001,
			    0,
			    0x0000,
			    0x0000,
			    ARRAY_SIZE(vaudio_val_tbl),
			    vaudio_val_tbl,
			    0,
			    0,
			    1000,
                            0},
};

static int cpcap_regulator_set_voltage(struct regulator_dev *rdev,
				       int min_uV, int max_uV, unsigned int *selector)
{
	struct cpcap_device *cpcap;
	int regltr_id;
	int retval;
	enum cpcap_reg regnr;
	int i;

	cpcap = rdev_get_drvdata(rdev);

	regltr_id = rdev_get_id(rdev);
	if (regltr_id >= CPCAP_NUM_REGULATORS)
		return -EINVAL;

	regnr = cpcap_regltr_data[regltr_id].reg;

	if (regltr_id == CPCAP_VRF1) {
		if (min_uV > 2500000)
			i = 0;
		else
			i = cpcap_regltr_data[regltr_id].volt_mask;
	} else {
		for (i = 0; i < cpcap_regltr_data[regltr_id].val_tbl_sz; i++)
			if (cpcap_regltr_data[regltr_id].val_tbl[i] >= min_uV)
				break;

		if (i >= cpcap_regltr_data[regltr_id].val_tbl_sz)
			i--;

		*selector = i;

		i <<= cpcap_regltr_data[regltr_id].volt_shft;

                i += cpcap_regltr_data[regltr_id].bit_offset_from_cpcap_lowest_voltage;
	}

	retval = cpcap_regacc_write(cpcap, regnr, i,
				    cpcap_regltr_data[regltr_id].volt_mask);

	if ((cpcap_regltr_data[regltr_id].volt_trans_time) && (retval == 0))
		udelay(cpcap_regltr_data[regltr_id].volt_trans_time);

	return retval;
}

static int cpcap_regulator_get_voltage(struct regulator_dev *rdev)
{
	struct cpcap_device *cpcap;
	int regltr_id;
	unsigned short volt_bits;
	enum cpcap_reg regnr;
	unsigned int shift;

	cpcap = rdev_get_drvdata(rdev);

	regltr_id = rdev_get_id(rdev);
	if (regltr_id >= CPCAP_NUM_REGULATORS)
		return -EINVAL;

	regnr = cpcap_regltr_data[regltr_id].reg;

	if (cpcap_regacc_read(cpcap, regnr, &volt_bits) < 0)
		return -1;

	if (!(volt_bits & cpcap_regltr_data[regltr_id].mode_mask))
		return 0;

	volt_bits &= cpcap_regltr_data[regltr_id].volt_mask;

	volt_bits -= cpcap_regltr_data[regltr_id].bit_offset_from_cpcap_lowest_voltage;

	shift = cpcap_regltr_data[regltr_id].volt_shft;

	return cpcap_regltr_data[regltr_id].val_tbl[volt_bits >> shift];
}

static int cpcap_regulator_enable(struct regulator_dev *rdev)
{
	struct cpcap_device *cpcap = rdev_get_drvdata(rdev);
	int regltr_id;
	int retval;
	enum cpcap_reg regnr;

	regltr_id = rdev_get_id(rdev);
	if (regltr_id >= CPCAP_NUM_REGULATORS)
		return -EINVAL;

	regnr = cpcap_regltr_data[regltr_id].reg;

	retval = cpcap_regacc_write(cpcap, regnr,
				    cpcap_regltr_data[regltr_id].mode_val,
				    cpcap_regltr_data[regltr_id].mode_mask);

	if ((retval == 0) &&
		(cpcap_regltr_data[regltr_id].mode_val & CPCAP_REG_OFF_MODE_SEC)) {
		retval = cpcap_regacc_write(cpcap,
									cpcap_regltr_data[regltr_id].assignment_reg,
									0,
									cpcap_regltr_data[regltr_id].assignment_mask);
	}
	if ((cpcap_regltr_data[regltr_id].turn_on_time) && (retval == 0))
		udelay(cpcap_regltr_data[regltr_id].turn_on_time);

	return retval;
}

static int cpcap_regulator_disable(struct regulator_dev *rdev)
{
	struct cpcap_device *cpcap = rdev_get_drvdata(rdev);
	int regltr_id;
	int retval;
	enum cpcap_reg regnr;

	regltr_id = rdev_get_id(rdev);
	if (regltr_id >= CPCAP_NUM_REGULATORS)
		return -EINVAL;

	regnr = cpcap_regltr_data[regltr_id].reg;

	retval = 0;
	if (cpcap_regltr_data[regltr_id].mode_val & CPCAP_REG_OFF_MODE_SEC) {
		retval = cpcap_regacc_write(cpcap,
									cpcap_regltr_data[regltr_id].assignment_reg,
									cpcap_regltr_data[regltr_id].assignment_mask,
									cpcap_regltr_data[regltr_id].assignment_mask);
	}
	if (retval == 0) {
		retval = cpcap_regacc_write(cpcap, regnr,
									cpcap_regltr_data[regltr_id].off_mode_val,
									cpcap_regltr_data[regltr_id].mode_mask);
	}
	return (retval);
}

static int cpcap_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct cpcap_device *cpcap = rdev_get_drvdata(rdev);
	int regltr_id;
	enum cpcap_reg regnr;
	unsigned short value;

	regltr_id = rdev_get_id(rdev);
	if (regltr_id >= CPCAP_NUM_REGULATORS)
		return -EINVAL;

	regnr = cpcap_regltr_data[regltr_id].reg;

	if (cpcap_regacc_read(cpcap, regnr, &value))
		return -1;

	return (value & cpcap_regltr_data[regltr_id].mode_mask) ? 1 : 0;
}

static int cpcap_regulator_set_mode(struct regulator_dev *rdev,
				    unsigned int mode)
{
	struct cpcap_device *cpcap = rdev_get_drvdata(rdev);
	int regltr_id;
	enum cpcap_reg regnr;
	int ret = 0;

	regltr_id = rdev_get_id(rdev);
	if (regltr_id != CPCAP_VAUDIO)
		return -EINVAL;

	regnr = cpcap_regltr_data[regltr_id].reg;

	if (mode == REGULATOR_MODE_NORMAL) {
		if (cpcap_regltr_data[regltr_id].mode_cntr == 0) {
			ret = cpcap_regacc_write(cpcap, regnr,
						 0,
						 CPCAP_BIT_AUDIO_LOW_PWR);
		}
		if (ret == 0)
			cpcap_regltr_data[regltr_id].mode_cntr++;
	} else if (mode == REGULATOR_MODE_STANDBY) {
		if (cpcap_regltr_data[regltr_id].mode_cntr == 1) {
			ret = cpcap_regacc_write(cpcap, regnr,
						 CPCAP_BIT_AUDIO_LOW_PWR,
						 CPCAP_BIT_AUDIO_LOW_PWR);
		} else if (WARN((cpcap_regltr_data[regltr_id].mode_cntr == 0),
				"Unbalanced modes for supply vaudio\n"))
			ret = -EIO;

		if (ret == 0)
			cpcap_regltr_data[regltr_id].mode_cntr--;
	}else if (mode == REGULATOR_MODE_IDLE ) {
             /*For case of  audio manager restart   set regulator to low power mode and reset counter */
			ret = cpcap_regacc_write(cpcap, regnr,
						 CPCAP_BIT_AUDIO_LOW_PWR,
						 CPCAP_BIT_AUDIO_LOW_PWR);
			cpcap_regltr_data[regltr_id].mode_cntr =0;
         }

	return ret;
}

static struct regulator_ops cpcap_regulator_ops = {
	.set_voltage = cpcap_regulator_set_voltage,
	.get_voltage = cpcap_regulator_get_voltage,
	.enable = cpcap_regulator_enable,
	.disable = cpcap_regulator_disable,
	.is_enabled = cpcap_regulator_is_enabled,
	.set_mode = cpcap_regulator_set_mode,
};

static struct regulator_desc regulators[] = {
	[CPCAP_SW1]      = CPCAP_REGULATOR("sw1", CPCAP_SW1),
	[CPCAP_SW2]      = CPCAP_REGULATOR("sw2", CPCAP_SW2),
	[CPCAP_SW3]      = CPCAP_REGULATOR("sw3", CPCAP_SW3),
	[CPCAP_SW4]      = CPCAP_REGULATOR("sw4", CPCAP_SW4),
	[CPCAP_SW5]      = CPCAP_REGULATOR("sw5", CPCAP_SW5),
	[CPCAP_VCAM]     = CPCAP_REGULATOR("vcam", CPCAP_VCAM),
	[CPCAP_VCSI]     = CPCAP_REGULATOR("vcsi", CPCAP_VCSI),
	[CPCAP_VDAC]     = CPCAP_REGULATOR("vdac", CPCAP_VDAC),
	[CPCAP_VDIG]     = CPCAP_REGULATOR("vdig", CPCAP_VDIG),
	[CPCAP_VFUSE]    = CPCAP_REGULATOR("vfuse", CPCAP_VFUSE),
	[CPCAP_VHVIO]    = CPCAP_REGULATOR("vhvio", CPCAP_VHVIO),
	[CPCAP_VSDIO]    = CPCAP_REGULATOR("vsdio", CPCAP_VSDIO),
	[CPCAP_VPLL]     = CPCAP_REGULATOR("vpll", CPCAP_VPLL),
	[CPCAP_VRF1]     = CPCAP_REGULATOR("vrf1", CPCAP_VRF1),
	[CPCAP_VRF2]     = CPCAP_REGULATOR("vrf2", CPCAP_VRF2),
	[CPCAP_VRFREF]   = CPCAP_REGULATOR("vrfref", CPCAP_VRFREF),
	[CPCAP_VWLAN1]   = CPCAP_REGULATOR("vwlan1", CPCAP_VWLAN1),
	[CPCAP_VWLAN2]   = CPCAP_REGULATOR("vwlan2", CPCAP_VWLAN2),
	[CPCAP_VSIM]     = CPCAP_REGULATOR("vsim", CPCAP_VSIM),
	[CPCAP_VSIMCARD] = CPCAP_REGULATOR("vsimcard", CPCAP_VSIMCARD),
	[CPCAP_VVIB]     = CPCAP_REGULATOR("vvib", CPCAP_VVIB),
	[CPCAP_VUSB]     = CPCAP_REGULATOR("vusb", CPCAP_VUSB),
	[CPCAP_VAUDIO]   = CPCAP_REGULATOR("vaudio", CPCAP_VAUDIO),
};

static struct cpcap_mode_value * __devinit cpcap_regulator_find_init_mode_mask(
				   struct cpcap_device *cpcap,
				   struct cpcap_mode_value *init)
{
	unsigned int i = CPCAP_MODE_VALUE_MAX;

	while ((init != NULL) &&
		   (--i)) {
		if (init->hw_check == NULL)
			return init;
		if (init->hw_check(cpcap))
			return init;
		init++;
	}
	return NULL;
}

static int __devinit cpcap_regulator_probe(struct platform_device *pdev)
{
	struct regulator_dev *rdev;
	struct cpcap_device *cpcap;
	struct cpcap_platform_data *data;
	struct regulator_init_data *init;
	struct cpcap_mode_value *init_mode;
	enum cpcap_regulator_id reg_id;

	/* Already set by core driver */
	cpcap = platform_get_drvdata(pdev);
	data = cpcap->spi->dev.platform_data;
	init = pdev->dev.platform_data;

	reg_id = pdev->id;
	/* SW1, SW2, SW3 & SW4 on ST CPCAP 3.1 are initialized to AMS/AMS mode
	   in board-mot-power.c. Changing the mode bits is disabled here by
	   clearing the four mode bits in the mode_mask below.  This was done to
	   work around a hardware bug in ST CPCAP 3.1 which causes spurious
	   hangs and crashes while the phone is asleep.  This is due to the
	   switchmode power supply getting stuck in a state where the PMOS is
	   fully ON when programmed in low power mode (PFM).  The failure
	   appears random, but setting another buck in close proximity at full
	   load significantly increases occurrence, as verified by STE.	*/

	if (cpcap->vendor == CPCAP_VENDOR_ST &&
	    cpcap->revision == CPCAP_REVISION_3_1 &&
	    reg_id == CPCAP_SW1)
		cpcap_regltr_data[CPCAP_SW1].mode_mask = 0x6000;

	if (cpcap->vendor == CPCAP_VENDOR_ST &&
	    cpcap->revision == CPCAP_REVISION_3_1 &&
	    reg_id == CPCAP_SW2)
		cpcap_regltr_data[CPCAP_SW2].mode_mask = 0x6000;

	if ( cpcap->vendor == CPCAP_VENDOR_ST &&
	     cpcap->revision == CPCAP_REVISION_3_1 &&
	     reg_id == CPCAP_SW3)
		cpcap_regltr_data[CPCAP_SW3].mode_mask = 0x0500;

	if ( cpcap->vendor == CPCAP_VENDOR_ST &&
	     cpcap->revision == CPCAP_REVISION_3_1 &&
	     reg_id == CPCAP_SW4)
		cpcap_regltr_data[CPCAP_SW4].mode_mask = 0x6000;

	init_mode =
		cpcap_regulator_find_init_mode_mask(cpcap,
					data->regulator_mode_values[reg_id]);
	if (init_mode != NULL)
		cpcap_regltr_data[reg_id].mode_val = init_mode->mode;

	init_mode =
		cpcap_regulator_find_init_mode_mask(cpcap,
					data->regulator_off_mode_values[reg_id]);
	if (init_mode != NULL)
		cpcap_regltr_data[reg_id].off_mode_val = init_mode->mode;

	rdev = regulator_register(&regulators[reg_id], &pdev->dev,
				init, cpcap);
	if (IS_ERR(rdev))
		return PTR_ERR(rdev);
	/* this is ok since the cpcap is still reachable from the rdev */
	platform_set_drvdata(pdev, rdev);
/* 
	if (reg_id == CPCAP_SW5) {
		data->regulator_init =
			cpcap->regulator_pdev[CPCAP_VUSB]->dev.platform_data;
		data->regulator_init->supply_regulator_dev =
			rdev_get_dev(rdev);
		platform_device_add(cpcap->regulator_pdev[CPCAP_VUSB]);
	}*/

	return 0;
}

static int __devexit cpcap_regulator_remove(struct platform_device *pdev)
{
	struct regulator_dev *rdev = platform_get_drvdata(pdev);

	regulator_unregister(rdev);

	return 0;
}

static struct platform_driver cpcap_regulator_driver = {
	.driver = {
		.name = "cpcap-regltr",
	},
	.probe = cpcap_regulator_probe,
	.remove = __devexit_p(cpcap_regulator_remove),
};

static int __init cpcap_regulator_init(void)
{
	return platform_driver_register(&cpcap_regulator_driver);
}
subsys_initcall(cpcap_regulator_init);

static void __exit cpcap_regulator_exit(void)
{
	platform_driver_unregister(&cpcap_regulator_driver);
}
module_exit(cpcap_regulator_exit);

MODULE_ALIAS("platform:cpcap-regulator");
MODULE_DESCRIPTION("CPCAP regulator driver");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");
