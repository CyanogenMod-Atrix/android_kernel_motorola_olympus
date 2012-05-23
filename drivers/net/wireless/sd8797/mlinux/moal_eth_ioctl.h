/** @file moal_eth_ioctl.h
 *
 * @brief This file contains definition for private IOCTL call.
 *  
 * Copyright (C) 2012, Marvell International Ltd.  
 *
 * This software file (the "File") is distributed by Marvell International 
 * Ltd. under the terms of the GNU General Public License Version 2, June 1991 
 * (the "License").  You may use, redistribute and/or modify this File in 
 * accordance with the terms and conditions of the License, a copy of which 
 * is available by writing to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or on the
 * worldwide web at http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE 
 * ARE EXPRESSLY DISCLAIMED.  The License provides additional details about 
 * this warranty disclaimer.
 *
 */

/********************************************************
Change log:
    01/05/2012: initial version
********************************************************/

#ifndef _WOAL_ETH_PRIV_H_
#define _WOAL_ETH_PRIV_H_

#ifdef WIFI_DIRECT_SUPPORT
/** Private command ID to Host command */
#define	WOAL_WIFIDIRECT_HOST_CMD    (SIOCDEVPRIVATE + 1)
#endif

/** Private command ID to pass mgmt frame */
#define WOAL_MGMT_FRAME_TX          WOAL_MGMT_FRAME_TX_IOCTL

/** Private command ID to pass custom IE list */
#define WOAL_CUSTOM_IE_CFG          (SIOCDEVPRIVATE + 13)

/** Private command ID for Android ICS priv CMDs */
#define	WOAL_ANDROID_PRIV_CMD       (SIOCDEVPRIVATE + 14)

/** Private command ID to get BSS type */
#define WOAL_GET_BSS_TYPE           (SIOCDEVPRIVATE + 15)

int woal_do_ioctl(struct net_device *dev, struct ifreq *req, int cmd);

/*
 * For android private commands, fixed value of ioctl is used.
 * Internally commands are differentiated using strings.
 *
 * application needs to specify "total_len" of data for copy_from_user
 * kernel updates "used_len" during copy_to_user
 */
/** Private command structure from app */
typedef struct _android_wifi_priv_cmd
{
    /** Buffer pointer */
    char *buf;
    /** buffer updated by driver */
    int used_len;
    /** buffer sent by application */
    int total_len;
} android_wifi_priv_cmd;

/** data structure for cmd txratecfg */
typedef struct woal_priv_tx_rate_cfg
{
    /* LG rate: 0, HT rate: 1, VHT rate: 2 */
    t_u32 rate_format;
    /** Rate/MCS index (0xFF: auto) */
    t_u32 rate_index;
} woal_tx_rate_cfg;

typedef struct woal_priv_esuppmode_cfg
{
    /* RSN mode */
    t_u16 rsn_mode;
    /* Pairwise cipher */
    t_u8 pairwise_cipher;
    /* Group cipher */
    t_u8 group_cipher;
} woal_esuppmode_cfg;

mlan_status woal_set_ap_wps_p2p_ie(moal_private * priv, t_u8 * ie, size_t len);

int woal_android_priv_cmd(struct net_device *dev, struct ifreq *req);

#endif /* _WOAL_ETH_PRIV_H_ */
