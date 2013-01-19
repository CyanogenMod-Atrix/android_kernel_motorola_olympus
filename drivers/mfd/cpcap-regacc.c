/*
 * Copyright (C) 2007-2011 Motorola, Inc.
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

#include <linux/ctype.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/spi/spi.h>
#include <linux/spi/cpcap.h>
#include <linux/spi/cpcap-regbits.h>

#define IS_CPCAP(reg) ((reg) >= CPCAP_REG_START && (reg) <= CPCAP_REG_END)
#define CPCAP_SECONDARY_CS    1
#define CPCAP_DEBUG_FLAG_DUMP_PRI  0x00000001
#define CPCAP_DEBUG_FLAG_DUMP_SEC  0x00000002
#define CPCAP_DEBUG_FLAG_LOG_PRI_W 0x00000004
#define CPCAP_DEBUG_FLAG_LOG_PRI_R 0x00000008
#define CPCAP_DEBUG_FLAG_LOG_SEC_W 0x00000010
#define CPCAP_DEBUG_FLAG_LOG_SEC_R 0x00000020

static DEFINE_MUTEX(reg_access);

/*
 * This table contains information about a single register in the power IC.
 * It is used during register access to information such as the register address
 * and the modifiability of each bit in the register.  Special notes for
 * particular elements of this structure follows:
 *
 * constant_mask: A '1' in this mask indicates that the corresponding bit has a
 * 'constant' modifiability, and therefore must never be changed by any register
 * access.
 *
 * It is important to note that any bits that are 'constant' must have
 * synchronized read/write values.  That is to say, when a 'constant' bit is
 * read the value read must be identical to the value that must be written to
 * that bit in order for that bit to be read with the same value.
 *
 * rbw_mask: A '1' in this mask indicates that the corresponding bit (when not
 * being changed) should be written with the current value of that bit.  A '0'
 * in this mask indicates that the corresponding bit (when not being changed)
 * should be written with a value of '0'.
 */
static struct {
	/* Address of the register */
	const unsigned short address;
	/* Constant modifiability mask */
	const unsigned short constant_mask;
	/* Read-before-write mask */
	const unsigned short rbw_mask;
	const char *name;
	unsigned int debug_flags;
} register_info_tbl[CPCAP_NUM_REG_CPCAP] = {
	[CPCAP_REG_INT1]      = {0, 0x0004, 0x0000, "INT1", 		0},
	[CPCAP_REG_INT2]      = {1, 0x0000, 0x0000, "INT2", 		0},
	[CPCAP_REG_INT3]      = {2, 0x0000, 0x0000, "INT3", 		0},
	[CPCAP_REG_INT4]      = {3, 0xFC00, 0x0000, "INT4", 		0},
	[CPCAP_REG_INTM1]     = {4, 0x0004, 0xFFFF, "INTM1", 		0},
	[CPCAP_REG_INTM2]     = {5, 0x0000, 0xFFFF, "INTM2", 		0},
	[CPCAP_REG_INTM3]     = {6, 0x0000, 0xFFFF, "INTM3", 		0},
	[CPCAP_REG_INTM4]     = {7, 0xFC00, 0xFFFF, "INTM4", 		0},
	[CPCAP_REG_INTS1]     = {8, 0xFFFF, 0xFFFF, "INTS1", 		0},
	[CPCAP_REG_INTS2]     = {9, 0xFFFF, 0xFFFF, "INTS2", 		0},
	[CPCAP_REG_INTS3]     = {10, 0xFFFF, 0xFFFF, "INTS3", 		0},
	[CPCAP_REG_INTS4]     = {11, 0xFFFF, 0xFFFF, "INTS4", 		0},
	[CPCAP_REG_ASSIGN1]   = {12, 0x80F8, 0xFFFF, "ASSIGN1",		0},
	[CPCAP_REG_ASSIGN2]   = {13, 0x0000, 0xFFFF, "ASSIGN2",		0},
	[CPCAP_REG_ASSIGN3]   = {14, 0x0004, 0xFFFF, "ASSIGN3",		0},
	[CPCAP_REG_ASSIGN4]   = {15, 0x0068, 0xFFFF, "ASSIGN4",		0},
	[CPCAP_REG_ASSIGN5]   = {16, 0x0000, 0xFFFF, "ASSIGN5",		0},
	[CPCAP_REG_ASSIGN6]   = {17, 0xFC00, 0xFFFF, "ASSIGN6",		0},
	[CPCAP_REG_VERSC1]    = {18, 0xFFFF, 0xFFFF, "VERSC1", 		0},
	[CPCAP_REG_VERSC2]    = {19, 0xFFFF, 0xFFFF, "VERSC2", 		0},
	[CPCAP_REG_MI1]       = {128, 0x0000, 0x0000, "MI1", 		0},
	[CPCAP_REG_MIM1]      = {129, 0x0000, 0xFFFF, "MIM1", 		0},
	[CPCAP_REG_MI2]       = {130, 0x0000, 0xFFFF, "MI2", 		0},
	[CPCAP_REG_MIM2]      = {131, 0xFFFF, 0xFFFF, "MIM2", 		0},
	[CPCAP_REG_UCC1]      = {132, 0xF000, 0xFFFF, "UUC1", 		0},
	[CPCAP_REG_UCC2]      = {133, 0xFC00, 0xFFFF, "UUC2", 		0},
	[CPCAP_REG_PC1]       = {135, 0xFC00, 0xFFFF, "PC1", 		0},
	[CPCAP_REG_PC2]       = {136, 0xFC00, 0xFFFF, "PC2", 		0},
	[CPCAP_REG_BPEOL]     = {137, 0xFE00, 0xFFFF, "BPEOL", 		0},
	[CPCAP_REG_PGC]       = {138, 0xFE00, 0xFFFF, "PCG", 		0},
	[CPCAP_REG_MT1]       = {139, 0x0000, 0x0000, "MT1", 		0},
	[CPCAP_REG_MT2]       = {140, 0x0000, 0x0000, "MT2", 		0},
	[CPCAP_REG_MT3]       = {141, 0x0000, 0x0000, "MT3", 		0},
	[CPCAP_REG_PF]        = {142, 0x0000, 0xFFFF, "PF", 		0},
	[CPCAP_REG_SCC]       = {256, 0xFF00, 0xFFFF, "SCC", 		0},
	[CPCAP_REG_SW1]       = {257, 0xFFFF, 0xFFFF, "SW1", 		0},
	[CPCAP_REG_SW2]       = {258, 0xFC7F, 0xFFFF, "SW2", 		0},
	[CPCAP_REG_UCTM]      = {259, 0xFFFE, 0xFFFF, "UCTM", 		0},
	[CPCAP_REG_TOD1]      = {260, 0xFF00, 0xFFFF, "TOD1", 		0},
	[CPCAP_REG_TOD2]      = {261, 0xFE00, 0xFFFF, "TOD2", 		0},
	[CPCAP_REG_TODA1]     = {262, 0xFF00, 0xFFFF, "TODA1", 		0},
	[CPCAP_REG_TODA2]     = {263, 0xFE00, 0xFFFF, "TODA2", 		0},
	[CPCAP_REG_DAY]       = {264, 0x8000, 0xFFFF, "DAY", 		0},
	[CPCAP_REG_DAYA]      = {265, 0x8000, 0xFFFF, "DAYA", 		0},
	[CPCAP_REG_VAL1]      = {266, 0x0000, 0xFFFF, "VAL1", 		0},
	[CPCAP_REG_VAL2]      = {267, 0x0000, 0xFFFF, "VAL2", 		0},
	[CPCAP_REG_SDVSPLL]   = {384, 0x2488, 0xFFFF, "SDVSPLL", 	0},
	[CPCAP_REG_SI2CC1]    = {385, 0x8000, 0xFFFF, "SI2CC1", 	0},
	[CPCAP_REG_Si2CC2]    = {386, 0xFF00, 0xFFFF, "CI2CC2", 	0},
	[CPCAP_REG_S1C1]      = {387, 0x9080, 0xFFFF, "S1C1",       0},
	[CPCAP_REG_S1C2]      = {388, 0x8080, 0xFFFF, "S1C2",		0},
	[CPCAP_REG_S2C1]      = {389, 0x9080, 0xFFFF, "S2C1",       0},
	[CPCAP_REG_S2C2]      = {390, 0x8080, 0xFFFF, "S2C2",		0},
	[CPCAP_REG_S3C]       = {391, 0xFA84, 0xFFFF, "S3C",		0},
	[CPCAP_REG_S4C1]      = {392, 0x9080, 0xFFFF, "S4C1",       0},
	[CPCAP_REG_S4C2]      = {393, 0x8080, 0xFFFF, "S4C1",		0},
	[CPCAP_REG_S5C]       = {394, 0xFFD7, 0xFFFF, "S5C",		0},
	[CPCAP_REG_S6C]       = {395, 0xFFF4, 0xFFFF, "S6C",		0},
	[CPCAP_REG_VCAMC]     = {396, 0xFF48, 0xFFFF, "VACMC",		0},
	[CPCAP_REG_VCSIC]     = {397, 0xFFA8, 0xFFFF, "VCSIC",		0},
	[CPCAP_REG_VDACC]     = {398, 0xFF48, 0xFFFF, "VDACC",		0},
	[CPCAP_REG_VDIGC]     = {399, 0xFF48, 0xFFFF, "VDIGC",		0},
	[CPCAP_REG_VFUSEC]    = {400, 0xFF50, 0xFFFF, "VFUSEC",		0},
	[CPCAP_REG_VHVIOC]    = {401, 0xFFE8, 0xFFFF, "VHVIO",		0},
	[CPCAP_REG_VSDIOC]    = {402, 0xFF40, 0xFFFF, "VSDIOC",		0},
	[CPCAP_REG_VPLLC]     = {403, 0xFFA4, 0xFFFF, "VPLLC",		0},
	[CPCAP_REG_VRF1C]     = {404, 0xFF50, 0xFFFF, "VRF1C",		0},
	[CPCAP_REG_VRF2C]     = {405, 0xFFD4, 0xFFFF, "VRF2C",		0},
	[CPCAP_REG_VRFREFC]   = {406, 0xFFD4, 0xFFFF, "VRFREFC",	0},
	[CPCAP_REG_VWLAN1C]   = {407, 0xFFA8, 0xFFFF, "VWLAN1C",	0},
	[CPCAP_REG_VWLAN2C]   = {408, 0xFD32, 0xFFFF, "VWLAN2C",	0},
	[CPCAP_REG_VSIMC]     = {409, 0xE154, 0xFFFF, "VSIMC",		0},
	[CPCAP_REG_VVIBC]     = {410, 0xFFF2, 0xFFFF, "VVIBC",		0},
	[CPCAP_REG_VUSBC]     = {411, 0xFEA2, 0xFFFF, "VUSBC",		0},
	[CPCAP_REG_VUSBINT1C] = {412, 0xFFD4, 0xFFFF, "VUSBINT1C",	0},
	[CPCAP_REG_VUSBINT2C] = {413, 0xFFD4, 0xFFFF, "VUSBUNT2C",	0},
	[CPCAP_REG_URT]       = {414, 0xFFFE, 0xFFFF, "URT", 		0},
	[CPCAP_REG_URM1]      = {415, 0x0000, 0xFFFF, "URM1", 		0},
	[CPCAP_REG_URM2]      = {416, 0xFC00, 0xFFFF, "URM2", 		0},
	[CPCAP_REG_VAUDIOC]   = {512, 0xFF88, 0xFFFF, "VAUDIOC", 	0},
	[CPCAP_REG_CC]        = {513, 0x0000, 0xFEDF, "CC", 		0},
	[CPCAP_REG_CDI]       = {514, 0x4000, 0xFFFF, "CDI", 		0},
	[CPCAP_REG_SDAC]      = {515, 0xF000, 0xFCFF, "SDAC", 		0},
	[CPCAP_REG_SDACDI]    = {516, 0xC000, 0xFFFF, "SDACDI", 	0},
	[CPCAP_REG_TXI]       = {517, 0x0000, 0xFFFF, "TXI", 		0},
	[CPCAP_REG_TXMP]      = {518, 0xF000, 0xFFFF, "TXMP", 		0},
	[CPCAP_REG_RXOA]      = {519, 0xF800, 0xFFFF, "RXOA", 		0},
	[CPCAP_REG_RXVC]      = {520, 0x00C3, 0xFFFF, "RXCOA", 		0},
	[CPCAP_REG_RXCOA]     = {521, 0xF800, 0xFFFF, "RXSDOA", 	0},
	[CPCAP_REG_RXSDOA]    = {522, 0xE000, 0xFFFF, "RXSDOA", 	0},
	[CPCAP_REG_RXEPOA]    = {523, 0x8000, 0xFFFF, "RXEPOA", 	0},
	[CPCAP_REG_RXLL]      = {524, 0x0000, 0xFFFF, "RXLL", 		0},
	[CPCAP_REG_A2LA]      = {525, 0xFF00, 0xFFFF, "A2LA", 		0},
	[CPCAP_REG_MIPIS1]    = {526, 0x0000, 0xFFFF, "MIPOS1", 	0},
	[CPCAP_REG_MIPIS2]    = {527, 0xFF00, 0xFFFF, "MIPIS2", 	0},
	[CPCAP_REG_MIPIS3]    = {528, 0xFFFC, 0xFFFF, "MIPIS3", 	0},
	[CPCAP_REG_LVAB]      = {529, 0xFFFC, 0xFFFF, "LVAB", 		0},
	[CPCAP_REG_CCC1]      = {640, 0xFFF0, 0xFFFF, "CCC1", 		0},
	[CPCAP_REG_CRM]       = {641, 0xC000, 0xFFFF, "CRM", 		0},
	[CPCAP_REG_CCCC2]     = {642, 0xFFC0, 0xFFFF, "CCCC2", 		0},
	[CPCAP_REG_CCS1]      = {643, 0x0000, 0xFFFF, "CCS1", 		0},
	[CPCAP_REG_CCS2]      = {644, 0xFF00, 0xFFFF, "CCS2", 		0},
	[CPCAP_REG_CCA1]      = {645, 0x0000, 0xFFFF, "CCA1", 		0},
	[CPCAP_REG_CCA2]      = {646, 0x0000, 0xFFFF, "CCA2", 		0},
	[CPCAP_REG_CCM]       = {647, 0xFC00, 0xFFFF, "CCM", 		0},
	[CPCAP_REG_CCO]       = {648, 0xFC00, 0xFFFF, "CCO", 		0},
	[CPCAP_REG_CCI]       = {649, 0xC000, 0xFFFF, "CCI", 		0},
	[CPCAP_REG_ADCC1]     = {768, 0x0000, 0xFFFF, "ADCC1", 		0},
	[CPCAP_REG_ADCC2]     = {769, 0x0080, 0xFFFF, "ADCC2", 		0},
	[CPCAP_REG_ADCD0]     = {770, 0xFFFF, 0xFFFF, "ADCD0", 		0},
	[CPCAP_REG_ADCD1]     = {771, 0xFFFF, 0xFFFF, "ADCD1", 		0},
	[CPCAP_REG_ADCD2]     = {772, 0xFFFF, 0xFFFF, "ADCD2", 		0},
	[CPCAP_REG_ADCD3]     = {773, 0xFFFF, 0xFFFF, "ADCD3", 		0},
	[CPCAP_REG_ADCD4]     = {774, 0xFFFF, 0xFFFF, "ADCD4", 		0},
	[CPCAP_REG_ADCD5]     = {775, 0xFFFF, 0xFFFF, "ADCD5", 		0},
	[CPCAP_REG_ADCD6]     = {776, 0xFFFF, 0xFFFF, "ADCD6", 		0},
	[CPCAP_REG_ADCD7]     = {777, 0xFFFF, 0xFFFF, "ADCD7", 		0},
	[CPCAP_REG_ADCAL1]    = {778, 0xFFFF, 0xFFFF, "ADCAL1", 	0},
	[CPCAP_REG_ADCAL2]    = {779, 0xFFFF, 0xFFFF, "ADCAL2", 	0},
	[CPCAP_REG_USBC1]     = {896, 0x0000, 0xFFFF, "USBC1", 		0},
	[CPCAP_REG_USBC2]     = {897, 0x0000, 0xFFFF, "USBC2", 		0},
	[CPCAP_REG_USBC3]     = {898, 0x8200, 0xFFFF, "USBC3", 		0},
	[CPCAP_REG_UVIDL]     = {899, 0xFFFF, 0xFFFF, "UVIDL", 		0},
	[CPCAP_REG_UVIDH]     = {900, 0xFFFF, 0xFFFF, "UVIDH", 		0},
	[CPCAP_REG_UPIDL]     = {901, 0xFFFF, 0xFFFF, "UPIDL", 		0},
	[CPCAP_REG_UPIDH]     = {902, 0xFFFF, 0xFFFF, "UPIDH", 		0},
	[CPCAP_REG_UFC1]      = {903, 0xFF80, 0xFFFF, "UFC1", 		0},
	[CPCAP_REG_UFC2]      = {904, 0xFF80, 0xFFFF, "UFC2", 		0},
	[CPCAP_REG_UFC3]      = {905, 0xFF80, 0xFFFF, "UFC3", 		0},
	[CPCAP_REG_UIC1]      = {906, 0xFF64, 0xFFFF, "UIC1", 		0},
	[CPCAP_REG_UIC2]      = {907, 0xFF64, 0xFFFF, "UIC2", 		0},
	[CPCAP_REG_UIC3]      = {908, 0xFF64, 0xFFFF, "UIC3", 		0},
	[CPCAP_REG_USBOTG1]   = {909, 0xFFC0, 0xFFFF, "USBOTG1", 	0},
	[CPCAP_REG_USBOTG2]   = {910, 0xFFC0, 0xFFFF, "USBOTG2", 	0},
	[CPCAP_REG_USBOTG3]   = {911, 0xFFC0, 0xFFFF, "USBPTG3", 	0},
	[CPCAP_REG_UIER1]     = {912, 0xFFE0, 0xFFFF, "UIER1", 		0},
	[CPCAP_REG_UIER2]     = {913, 0xFFE0, 0xFFFF, "UIER2", 		0},
	[CPCAP_REG_UIER3]     = {914, 0xFFE0, 0xFFFF, "UIER3", 		0},
	[CPCAP_REG_UIEF1]     = {915, 0xFFE0, 0xFFFF, "UIEF1", 		0},
	[CPCAP_REG_UIEF2]     = {916, 0xFFE0, 0xFFFF, "UIEF2", 		0},
	[CPCAP_REG_UIEF3]     = {917, 0xFFE0, 0xFFFF, "UIEF3", 		0},
	[CPCAP_REG_UIS]       = {918, 0xFFFF, 0xFFFF, "UIS", 		0},
	[CPCAP_REG_UIL]       = {919, 0xFFFF, 0xFFFF, "UIL", 		0},
	[CPCAP_REG_USBD]      = {920, 0xFFFF, 0xFFFF, "USBD", 		0},
	[CPCAP_REG_SCR1]      = {921, 0xFF00, 0xFFFF, "SCR1", 		0},
	[CPCAP_REG_SCR2]      = {922, 0xFF00, 0xFFFF, "SCD2", 		0},
	[CPCAP_REG_SCR3]      = {923, 0xFF00, 0xFFFF, "SCR3", 		0},
	[CPCAP_REG_VMC]       = {939, 0xFFFE, 0xFFFF, "VMC", 		0},
	[CPCAP_REG_OWDC]      = {940, 0xFFFC, 0xFFFF, "OWDC", 		0},
	[CPCAP_REG_GPIO0]     = {941, 0x0D11, 0x3FFF, "GPIO0", 		0},
	[CPCAP_REG_GPIO1]     = {943, 0x0D11, 0x3FFF, "GPIO1", 		0},
	[CPCAP_REG_GPIO2]     = {945, 0x0D11, 0x3FFF, "GPIO2", 		0},
	[CPCAP_REG_GPIO3]     = {947, 0x0D11, 0x3FFF, "GPIO3", 		0},
	[CPCAP_REG_GPIO4]     = {949, 0x0D11, 0x3FFF, "GPIO4", 		0},
	[CPCAP_REG_GPIO5]     = {951, 0x0C11, 0x3FFF, "GPIO5", 		0},
	[CPCAP_REG_GPIO6]     = {953, 0x0C11, 0x3FFF, "GPIO6", 		0},
	[CPCAP_REG_MDLC]      = {1024, 0x0000, 0xFFFF, "MDLC", 		0},
	[CPCAP_REG_KLC]       = {1025, 0x8000, 0xFFFF, "KLC", 		0},
	[CPCAP_REG_ADLC]      = {1026, 0x8000, 0xFFFF, "ADLC", 		0},
	[CPCAP_REG_REDC]      = {1027, 0xFC00, 0xFFFF, "REDC", 		0},
	[CPCAP_REG_GREENC]    = {1028, 0xFC00, 0xFFFF, "GREENC", 	0},
	[CPCAP_REG_BLUEC]     = {1029, 0xFC00, 0xFFFF, "BLUEC", 	0},
	[CPCAP_REG_CFC]       = {1030, 0xF000, 0xFFFF, "CFC", 		0},
	[CPCAP_REG_ABC]       = {1031, 0xFFC3, 0xFFFF, "ABC", 		0},
	[CPCAP_REG_BLEDC]     = {1032, 0xFC00, 0xFFFF, "BLEDC", 	0},
	[CPCAP_REG_CLEDC]     = {1033, 0xFC00, 0xFFFF, "CLEDC", 	0},
	[CPCAP_REG_OW1C]      = {1152, 0xFF00, 0xFFFF, "OW1C", 		0},
	[CPCAP_REG_OW1D]      = {1153, 0xFF00, 0xFFFF, "OW1D", 		0},
	[CPCAP_REG_OW1I]      = {1154, 0xFFFF, 0xFFFF, "OW1I", 		0},
	[CPCAP_REG_OW1IE]     = {1155, 0xFF00, 0xFFFF, "OW1IE", 	0},
	[CPCAP_REG_OW1]       = {1157, 0xFF00, 0xFFFF, "OW1", 		0},
	[CPCAP_REG_OW2C]      = {1160, 0xFF00, 0xFFFF, "OW2C", 		0},
	[CPCAP_REG_OW2D]      = {1161, 0xFF00, 0xFFFF, "OW2D", 		0},
	[CPCAP_REG_OW2I]      = {1162, 0xFFFF, 0xFFFF, "OW2I", 		0},
	[CPCAP_REG_OW2IE]     = {1163, 0xFF00, 0xFFFF, "OW3IE", 	0},
	[CPCAP_REG_OW2]       = {1165, 0xFF00, 0xFFFF, "OW2", 		0},
	[CPCAP_REG_OW3C]      = {1168, 0xFF00, 0xFFFF, "OW3C", 		0},
	[CPCAP_REG_OW3D]      = {1169, 0xFF00, 0xFFFF, "OW3D", 		0},
	[CPCAP_REG_OW3I]      = {1170, 0xFF00, 0xFFFF, "OW3I", 		0},
	[CPCAP_REG_OW3IE]     = {1171, 0xFF00, 0xFFFF, "OW3IE", 	0},
	[CPCAP_REG_OW3]       = {1173, 0xFF00, 0xFFFF, "OW3", 		0},
	[CPCAP_REG_GCAIC]     = {1174, 0xFF00, 0xFFFF, "GCAIC", 	0},
	[CPCAP_REG_GCAIM]     = {1175, 0xFF00, 0xFFFF, "GCAIM", 	0},
	[CPCAP_REG_LGDIR]     = {1176, 0xFFE0, 0xFFFF, "LGDIR", 	0},
	[CPCAP_REG_LGPU]      = {1177, 0xFFE0, 0xFFFF, "LGPU", 		0},
	[CPCAP_REG_LGPIN]     = {1178, 0xFF00, 0xFFFF, "LGPIN", 	0},
	[CPCAP_REG_LGMASK]    = {1179, 0xFFE0, 0xFFFF, "LGMASK", 	0},
	[CPCAP_REG_LDEB]      = {1180, 0xFF00, 0xFFFF, "LDEB", 		0},
	[CPCAP_REG_LGDET]     = {1181, 0xFF00, 0xFFFF, "LGDET", 	0},
	[CPCAP_REG_LMISC]     = {1182, 0xFF07, 0xFFFF, "LMISC", 	0},
	[CPCAP_REG_LMACE]     = {1183, 0xFFF8, 0xFFFF, "LMACE", 	0},
	[CPCAP_REG_TEST]      = {7936, 0x0000, 0xFFFF, "TEST",		0},
	[CPCAP_REG_ST_TEST1]  = {8002, 0x0000, 0xFFFF, "ST_TEST1", 	0},
};

static unsigned int debug_flags;

#define CPCAP_REGACC_DEBUG_STR_LEN 128
static char debug_str[CPCAP_REGACC_DEBUG_STR_LEN];

struct kparam_string cpcap_regacc_debug_str = {
	.string			= debug_str,
	.maxlen			= CPCAP_REGACC_DEBUG_STR_LEN,
};

static void update_debug_flags(void)
{
	enum cpcap_reg reg_id;

	debug_flags = 0;
	for (reg_id = 0; reg_id < ARRAY_SIZE(register_info_tbl); reg_id++)
		debug_flags |= register_info_tbl[reg_id].debug_flags;
}

static enum cpcap_reg find_reg_id(char *reg_name, size_t len)
{
	enum cpcap_reg reg_id;

	for (reg_id = 0; reg_id < ARRAY_SIZE(register_info_tbl); reg_id++)
		if ((strncasecmp(register_info_tbl[reg_id].name, reg_name, len) == 0) &&
			(strlen(register_info_tbl[reg_id].name) == len))
			return reg_id;
	return CPCAP_NUM_REG_CPCAP;
}

static unsigned int get_reg_name(char *src, char *reg_name)
{
	unsigned int i;

	i = 0;
	while (isalnum(*src))
		reg_name[i++] = *src++;
	reg_name[i] = '\0';
	return i;
}

/*
 * Parse the string used for setting up debugging of registers.  Only symbols
 * can be used, numbers are ignored.  The genral format of the string is:
 *
 * low_reg_name-high_reg_name,single_reg_name =
 *   DUMP_PRI|DUMP_SEC|LOG_PRI_W|LOG_PRI_R|LOG_SEC_W|LOG_SEC_R
 *
 * Where:
 *   low_reg_name    - The name as specified in register_info_tbl[].name for
 *                     the first register to update.
 *   high_reg_name   - The name as specified in register_info_tbl[].name for
 *                     the high register to update.
 *   single_reg_name - A single register name to be updated.
 *
 *   DUMP_PRI  - Specifies the primary register must be dumped when entering
 *               standby.
 *   DUMP_SEC  - Specifies the secondary register must be dumped when entering
 *               standby.
 *   LOG_PRI_W - Specifies writes to the primary register must be logged.
 *   LOG_PRI_R - Specifies reads to the primary register must be logged.
 *   LOG_SEC_W - Specifies writes to the secondary register must be logged.
 *   LOG_SEC_R - Specifies reads to the secondary register must be logged.
 *
 * Register ranges are specified with '-' between the names.  Register sets
 * are seperated by a ','.
 *
 * Any number of the bits may be specified.  If none or 0 are specified, the
 * mask is reset.
 */
int cpcap_regacc_set_debug(const char *kmessage, struct kernel_param *kp)
{
	int err;
	unsigned int i;
	unsigned int mask;
	char *reg;
	char *log_op;
	char reg_name[12];
	enum cpcap_reg start_reg;
	enum cpcap_reg end_reg;
	const struct {
		unsigned int mask;
		unsigned int len;
		char op[11];
	} op_tb[] = {
		{CPCAP_DEBUG_FLAG_DUMP_PRI,  8, "DUMP_PRI"},
		{CPCAP_DEBUG_FLAG_DUMP_SEC,  8, "DUMP_SEC"},
		{CPCAP_DEBUG_FLAG_LOG_PRI_W, 9, "LOG_PRI_W"},
		{CPCAP_DEBUG_FLAG_LOG_PRI_R, 9, "LOG_PRI_R"},
		{CPCAP_DEBUG_FLAG_LOG_SEC_W, 9, "LOG_SEC_W"},
		{CPCAP_DEBUG_FLAG_LOG_SEC_R, 9, "LOG_SEC_R"}
	};

	err = param_set_copystring(kmessage, kp);
	/* Break the string up into the regester id part and the debug flag
	   part. */
	if (err)
		return err;

	reg = debug_str;
	log_op = debug_str;
	while ((*log_op) && (*log_op != '='))
		log_op++;

	*log_op++ = '\0';

	/* Get the debug mask from the debug flag part. */
	mask = 0;
	while ((*log_op) && (!isalnum(*log_op)))
		log_op++;

	if (*log_op != '0') {
		while (*log_op) {
			for (i = 0; i < sizeof(op_tb)/sizeof(op_tb[0]); i++)
				if (strncasecmp(log_op, op_tb[i].op, op_tb[i].len) == 0) {
					mask |= op_tb[i].mask;
					log_op += op_tb[i].len;
				}
			while (isalnum(*log_op))
				log_op++;
			while ((*log_op) && (!isalnum(*log_op)))
				log_op++;
		}
	}

	/* Update the debug table based on the input. */
	while (*reg) {
		while ((*reg) && (!isalnum(*reg)))
			reg++;
		i = get_reg_name(reg, reg_name);
		if (i > 0) {
			reg += i;
			start_reg = find_reg_id(reg_name, i);
			end_reg = start_reg;
			while (isspace(*reg))
				reg++;
			if (*reg == '-') {
				reg++;
				while ((*reg) && (!isalnum(*reg)))
					reg++;
				i = get_reg_name(reg, reg_name);
				if (i > 0) {
					reg += i;
					end_reg = find_reg_id(reg_name, i);
					if (start_reg > end_reg) {
						i = start_reg;
						start_reg = end_reg;
						end_reg = i;
					}
				} else
					while (isalnum(*reg))
						reg++;
			}
			if (start_reg < CPCAP_NUM_REG_CPCAP) {
				if (end_reg >= CPCAP_NUM_REG_CPCAP)
					end_reg = CPCAP_NUM_REG_CPCAP-1;
				for (i = start_reg; i <= end_reg; i++)
					register_info_tbl[i].debug_flags = mask;
			}
		} else
			while (isalnum(*reg))
				reg++;
	}
	update_debug_flags();
	return 0;
}

static int cpcap_spi_access(struct spi_device *spi, u8 *buf,
			    size_t len)
{
	struct spi_message m;
	struct spi_transfer t = {
		.tx_buf = buf,
		.len = len,
		.rx_buf = buf,
		.bits_per_word = 32,
	};

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spi_sync(spi, &m);
}

static int cpcap_config_for_read(struct spi_device *spi, unsigned short reg,
				 unsigned short *data)
{
	int status = -ENOTTY;
	u32 buf32;  /* force buf to be 32bit aligned */
	u8 *buf = (u8 *) &buf32;

	if (spi != NULL) {
#ifdef CONFIG_ARCH_TEGRA
      buf[0] = (reg >> 6) & 0x000000FF;
      buf[1] = (reg << 2) & 0x000000FF;
      buf[2] = 0;
      buf[3] = 0;
#else
		buf[3] = (reg >> 6) & 0x000000FF;
		buf[2] = (reg << 2) & 0x000000FF;
		buf[1] = 0;
		buf[0] = 0;
#endif
		status = cpcap_spi_access(spi, buf, 4);

		if (status == 0)

#ifdef CONFIG_ARCH_TEGRA
      *data = buf[3] | (buf[2] << 8);
#else
		*data = buf[0] | (buf[1] << 8);
#endif

 	}

	return status;
}

static int cpcap_config_for_write(struct spi_device *spi, unsigned short reg,
				  unsigned short data)
{
	int status = -ENOTTY;
	u32 buf32;  /* force buf to be 32bit aligned */
	u8 *buf = (u8 *) &buf32;

	if (spi != NULL) {

#ifdef CONFIG_ARCH_TEGRA
      buf[0] = ((reg >> 6) & 0x000000FF) | 0x80;
      buf[1] = (reg << 2) & 0x000000FF;
      buf[2] = (data >> 8) & 0x000000FF;
      buf[3] = data & 0x000000FF;
#else
		buf[3] = ((reg >> 6) & 0x000000FF) | 0x80;
		buf[2] = (reg << 2) & 0x000000FF;
		buf[1] = (data >> 8) & 0x000000FF;
		buf[0] = data & 0x000000FF;
#endif

		status = cpcap_spi_access(spi, buf, 4);
	}

	return status;
}

int cpcap_regacc_read(struct cpcap_device *cpcap, enum cpcap_reg reg,
		      unsigned short *value_ptr)
{
	int retval = -EINVAL;
	struct spi_device *spi = cpcap->spi;

	if (IS_CPCAP(reg) && (value_ptr != 0)) {
		mutex_lock(&reg_access);

		retval = cpcap_config_for_read(spi, register_info_tbl
				      [reg].address, value_ptr);

		mutex_unlock(&reg_access);
		if (cpcap->spi->chip_select != CPCAP_SECONDARY_CS) {
			if (register_info_tbl[reg].debug_flags&CPCAP_DEBUG_FLAG_LOG_PRI_R)
				printk(KERN_INFO "%s Primary register: %d %s %04x\n",
					__func__, reg, register_info_tbl[reg].name, *value_ptr);
		} else {
			if (register_info_tbl[reg].debug_flags&CPCAP_DEBUG_FLAG_LOG_SEC_R)
				printk(KERN_INFO "%s Secondary register: %d %s %04x\n",
					__func__, reg, register_info_tbl[reg].name, *value_ptr);
		}
	}

	if (retval)
		printk (KERN_ERR "%s: Unable to read from register %d\n",
			__func__, register_info_tbl[reg].address);

	return retval;
}

int cpcap_regacc_write(struct cpcap_device *cpcap,
		       enum cpcap_reg reg,
		       unsigned short value,
		       unsigned short mask)
{
	int retval = -EINVAL;
	unsigned short old_value = 0;
	struct cpcap_platform_data *data;
	struct spi_device *spi = cpcap->spi;

	data = (struct cpcap_platform_data *)spi->controller_data;

	if (IS_CPCAP(reg) &&
	    (mask & register_info_tbl[reg].constant_mask) == 0) {
		mutex_lock(&reg_access);

		value &= mask;

		if ((register_info_tbl[reg].rbw_mask) != 0) {
			retval = cpcap_config_for_read(spi, register_info_tbl
						       [reg].address,
						       &old_value);
			if (retval != 0)
				goto error;
		}

		old_value &= register_info_tbl[reg].rbw_mask;
		old_value &= ~mask;
		value |= old_value;
		retval = cpcap_config_for_write(spi,
						register_info_tbl[reg].address,
						value);
		if (cpcap->spi->chip_select != CPCAP_SECONDARY_CS) {
			if (register_info_tbl[reg].debug_flags&CPCAP_DEBUG_FLAG_LOG_PRI_W)
				printk(KERN_INFO "%s Primary register: %d %s %04x\n",
					__func__, reg, register_info_tbl[reg].name, value);
		} else {
			if (register_info_tbl[reg].debug_flags&CPCAP_DEBUG_FLAG_LOG_SEC_W)
				printk(KERN_INFO "%s Secondary register: %d %s %04x\n",
					__func__, reg, register_info_tbl[reg].name, value);
		}
error:
		mutex_unlock(&reg_access);
	}

	if (retval)
		printk (KERN_ERR "%s: Unable to write 0x%x & 0x%x to register %d\n",
			__func__, value, mask, register_info_tbl[reg].address);

	return retval;
}

int cpcap_regacc_read_secondary(struct cpcap_device *cpcap, enum cpcap_reg reg,
		      unsigned short *value_ptr)
{
	int retval = -EINVAL;
	struct cpcap_device secondary_cpcap;
	struct spi_device secondary_spi;

	/* cpcap & spi structures are duplicated since secondary reads are only
	   performed a couple of times at powerup which doesn't justify completely
	   duplicating the entire CPCAP driver structure or creating static
	   instances of the structures.  Since regacc_read only uses the SPI
	   element in the cpcap_device everything else in it can be safely
	   NULL'ed out
	 */
	memset(&secondary_cpcap, 0, sizeof(struct cpcap_device));
	if (cpcap && cpcap->spi){
		memcpy(&secondary_spi, cpcap->spi, sizeof(struct spi_device));
		secondary_spi.chip_select = CPCAP_SECONDARY_CS;
		secondary_cpcap.spi = &secondary_spi;
		retval = cpcap_regacc_read(&secondary_cpcap, reg, value_ptr);
	}

	if (retval)
		printk (KERN_ERR "%s: Unable to read from SECONDARY register %d\n",
			__func__, register_info_tbl[reg].address);

	return retval;
}

int cpcap_regacc_write_secondary(struct cpcap_device *cpcap,
		       enum cpcap_reg reg,
		       unsigned short value,
		       unsigned short mask)
{
	int retval = -EINVAL;
	struct cpcap_device secondary_cpcap;
	struct spi_device secondary_spi;

	/* cpcap & spi structures are duplicated since secondary writes are only
	   performed a couple of times at powerup which doesn't justify completely
	   duplicating the entire CPCAP driver structure or creating static
	   instances of the structures.  Since regacc_write only uses the SPI
	   element in the cpcap_device everything else in it can be safely
	   NULL'ed out
	 */
	memset(&secondary_cpcap, 0, sizeof(struct cpcap_device));
	if (cpcap && cpcap->spi){
		memcpy(&secondary_spi, cpcap->spi, sizeof(struct spi_device));
		secondary_spi.chip_select = CPCAP_SECONDARY_CS;
		secondary_cpcap.spi = &secondary_spi;
		retval = cpcap_regacc_write(&secondary_cpcap, reg, value, mask);
	}

	if (retval)
		printk (KERN_ERR "%s: Unable to write 0x%x & 0x%x to SECONDARY register %d\n",
			__func__, value, mask, register_info_tbl[reg].address);

	return retval;
}

void cpcap_regacc_dump(struct cpcap_device *cpcap, char *note)
{
	unsigned int i;
	unsigned short reg;
	char reg_str[10];

	if (debug_flags&(CPCAP_DEBUG_FLAG_DUMP_PRI|CPCAP_DEBUG_FLAG_DUMP_SEC)) {
		printk(KERN_INFO "Dumping CPCAP registers - %s:\n", note);
		printk(KERN_INFO "Name       Reg Pri  Sec\n");
		for (i = 0; i < CPCAP_NUM_REG_CPCAP; i++) {
			if (register_info_tbl[i].debug_flags&(CPCAP_DEBUG_FLAG_DUMP_PRI|CPCAP_DEBUG_FLAG_DUMP_SEC)) {
				if (register_info_tbl[i].debug_flags&CPCAP_DEBUG_FLAG_DUMP_PRI)	{
					cpcap_regacc_read(cpcap, i, &reg);
					sprintf(reg_str, "%04x", reg);
				} else {
					sprintf(reg_str, "n/a ");
				}
				if (register_info_tbl[i].debug_flags&CPCAP_DEBUG_FLAG_DUMP_SEC)	{
					cpcap_regacc_read_secondary(cpcap, i, &reg);
					sprintf(&reg_str[4], " %04x", reg);
				} else {
					sprintf(&reg_str[4], " n/a ");
				}
				printk(KERN_INFO "%-10s %03d %s\n",
					   register_info_tbl[i].name, i, reg_str);
			}
		}
	}
}

int cpcap_regacc_init(struct cpcap_device *cpcap)
{
	unsigned short i;
	unsigned short mask;
	unsigned short reg;
	int retval = 0;
	struct cpcap_platform_data *data;
	struct spi_device *spi = cpcap->spi;

	data = (struct cpcap_platform_data *)spi->controller_data;

	i = 0;
	while (i < data->init_len) {
		if ((data->init[i].hw_check == NULL) ||
			(data->init[i].hw_check(cpcap))) {
			reg = data->init[i].reg;
			mask = 0xFFFF;
			mask &= ~(register_info_tbl[reg].constant_mask);

			retval = cpcap_regacc_write(cpcap, reg, data->init[i].data, mask);
			if (retval)
				break;
			do {
				i++;
			} while ((i < data->init_len) &&
					 (reg == data->init[i].reg));
			continue;
		}
		i++;
	}
	update_debug_flags();

	return retval;
}
