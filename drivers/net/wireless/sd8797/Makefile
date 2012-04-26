# File: Makefile
#
# Copyright (C) 2008-2012, Marvell International Ltd. 
#
# This software file (the "File") is distributed by Marvell International 
# Ltd. under the terms of the GNU General Public License Version 2, June 1991 
# (the "License").  You may use, redistribute and/or modify this File in 
# accordance with the terms and conditions of the License, a copy of which 
# is available by writing to the Free Software Foundation, Inc.,
# 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or on the
# worldwide web at http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt.
#
# A copy of the GPL is available in file gpl-2.0.txt accompanying in this 
# deliverables.
#
# THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE 
# IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE 
# ARE EXPRESSLY DISCLAIMED.  The License provides additional details about 
# this warranty disclaimer.

# Enable CFG80211 for STA
ifeq ($(CONFIG_CFG80211),y)
CONFIG_STA_CFG80211=y
else
ifeq ($(CONFIG_CFG80211),m)
CONFIG_STA_CFG80211=y
else
CONFIG_STA_CFG80211=n
endif
endif

# Enable CFG80211 for uAP
ifeq ($(CONFIG_CFG80211),y)
CONFIG_UAP_CFG80211=y
else
ifeq ($(CONFIG_CFG80211),m)
CONFIG_UAP_CFG80211=y
else
CONFIG_UAP_CFG80211=n
endif
endif

#############################################################################
# Compiler Flags
#############################################################################

EXTRA_CFLAGS += -Idrivers/net/wireless/sd8797/mlan
EXTRA_CFLAGS += -DLINUX
EXTRA_CFLAGS += -DFPNUM='"69"'
EXTRA_CFLAGS += -DDEBUG_LEVEL1
EXTRA_CFLAGS += -DPROC_DEBUG
EXTRA_CFLAGS += -DSDIO_MULTI_PORT_TX_AGGR
EXTRA_CFLAGS += -DSDIO_MULTI_PORT_RX_AGGR
EXTRA_CFLAGS += -DSDIO_SUSPEND_RESUME
EXTRA_CFLAGS += -DDFS_TESTING_SUPPORT
EXTRA_CFLAGS += -DMFG_CMD_SUPPORT

ifeq ($(CONFIG_64BIT), y)
	EXTRA_CFLAGS += -DMLAN_64BIT
endif

ifeq ($(CONFIG_STA_SUPPORT),y)
	EXTRA_CFLAGS += -DSTA_SUPPORT
	EXTRA_CFLAGS += -DREASSOCIATION
ifeq ($(CONFIG_STA_WEXT),y)
	EXTRA_CFLAGS += -DSTA_WEXT
endif
ifeq ($(CONFIG_STA_CFG80211),y)
	EXTRA_CFLAGS += -DSTA_CFG80211
endif
else
CONFIG_WIFI_DIRECT_SUPPORT=n
CONFIG_WIFI_DISPLAY_SUPPORT=n
CONFIG_STA_WEXT=n
CONFIG_STA_CFG80211=n
endif

ifeq ($(CONFIG_UAP_SUPPORT),y)
	EXTRA_CFLAGS += -DUAP_SUPPORT
ifeq ($(CONFIG_UAP_WEXT),y)
	EXTRA_CFLAGS += -DUAP_WEXT
endif
ifeq ($(CONFIG_UAP_CFG80211),y)
	EXTRA_CFLAGS += -DUAP_CFG80211
endif
else
CONFIG_WIFI_DIRECT_SUPPORT=n
CONFIG_WIFI_DISPLAY_SUPPORT=n
CONFIG_UAP_WEXT=n
CONFIG_UAP_CFG80211=n
endif

ifeq ($(CONFIG_WIFI_DIRECT_SUPPORT),y)
	EXTRA_CFLAGS += -DWIFI_DIRECT_SUPPORT
endif
ifeq ($(CONFIG_WIFI_DISPLAY_SUPPORT),y)
	EXTRA_CFLAGS += -DWIFI_DISPLAY_SUPPORT
endif

ifeq ($(CONFIG_BIG_ENDIAN),y)
	EXTRA_CFLAGS += -DBIG_ENDIAN_SUPPORT
endif

#############################################################################
# Make Targets
#############################################################################

MOALOBJS =	mlinux/moal_main.o \
		mlinux/moal_ioctl.o \
		mlinux/moal_shim.o \
		mlinux/moal_eth_ioctl.o

MLANOBJS =	mlan/mlan_shim.o mlan/mlan_init.o \
		mlan/mlan_txrx.o \
		mlan/mlan_cmdevt.o mlan/mlan_misc.o \
		mlan/mlan_cfp.o \
		mlan/mlan_module.o

MLANOBJS += mlan/mlan_wmm.o
MLANOBJS += mlan/mlan_sdio.o
MLANOBJS += mlan/mlan_11n_aggr.o
MLANOBJS += mlan/mlan_11n_rxreorder.o
MLANOBJS += mlan/mlan_11n.o
MLANOBJS += mlan/mlan_11d.o
MLANOBJS += mlan/mlan_11h.o
ifeq ($(CONFIG_STA_SUPPORT),y)
MLANOBJS += mlan/mlan_meas.o
MLANOBJS += mlan/mlan_scan.o \
			mlan/mlan_sta_ioctl.o \
			mlan/mlan_sta_rx.o \
			mlan/mlan_sta_tx.o \
			mlan/mlan_sta_event.o \
			mlan/mlan_sta_cmd.o \
			mlan/mlan_sta_cmdresp.o \
			mlan/mlan_join.o
ifeq ($(CONFIG_STA_WEXT),y)
MOALOBJS += mlinux/moal_priv.o \
            mlinux/moal_wext.o
endif
endif
ifeq ($(CONFIG_UAP_SUPPORT),y)
MLANOBJS += mlan/mlan_uap_ioctl.o
MLANOBJS += mlan/mlan_uap_cmdevent.o
MLANOBJS += mlan/mlan_uap_txrx.o
MOALOBJS += mlinux/moal_uap.o
ifeq ($(CONFIG_UAP_WEXT),y)
MOALOBJS += mlinux/moal_uap_priv.o
MOALOBJS += mlinux/moal_uap_wext.o
endif
ifeq ($(CONFIG_STA_WEXT),y)
MOALOBJS += mlinux/moal_wext.o
endif
endif
ifeq ($(CONFIG_STA_CFG80211),y)
MOALOBJS += mlinux/moal_cfg80211.o
MOALOBJS += mlinux/moal_sta_cfg80211.o
endif
ifeq ($(CONFIG_UAP_CFG80211),y)
MOALOBJS += mlinux/moal_cfg80211.o
MOALOBJS += mlinux/moal_uap_cfg80211.o
endif

ifdef CONFIG_PROC_FS
MOALOBJS += mlinux/moal_proc.o
MOALOBJS += mlinux/moal_debug.o
endif

obj-m := mlan.o
mlan-objs := $(MLANOBJS)
MOALOBJS += mlinux/moal_sdio_mmc.o
obj-m += sd8xxx.o
sd8xxx-objs := $(MOALOBJS)

###############################################################
# End of file
