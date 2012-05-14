/** @file moal_uap_cfg80211.c
  *
  * @brief This file contains the functions for uAP CFG80211. 
  *
  * Copyright (C) 2011-2012, Marvell International Ltd. 
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

#include "moal_cfg80211.h"
#include "moal_uap_cfg80211.h"

/* these 3 function will be called in woal_cfg80211_wifi_direct_ops */
int woal_cfg80211_add_beacon(struct wiphy *wiphy,
                             struct net_device *dev,
                             struct beacon_parameters *params);

int woal_cfg80211_set_beacon(struct wiphy *wiphy,
                             struct net_device *dev,
                             struct beacon_parameters *params);

int woal_cfg80211_del_beacon(struct wiphy *wiphy, struct net_device *dev);

static int woal_uap_cfg80211_scan(struct wiphy *wiphy, struct net_device *dev,
                                  struct cfg80211_scan_request *request);

static int woal_uap_cfg80211_connect(struct wiphy *wiphy,
                                     struct net_device *dev,
                                     struct cfg80211_connect_params *sme);

static int woal_uap_cfg80211_disconnect(struct wiphy *wiphy,
                                        struct net_device *dev,
                                        t_u16 reason_code);

int woal_uap_cfg80211_get_station(struct wiphy *wiphy, struct net_device *dev,
                                  u8 * mac, struct station_info *stainfo);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37) || defined(COMPAT_WIRELESS)
static const struct ieee80211_txrx_stypes
    ieee80211_uap_mgmt_stypes[NUM_NL80211_IFTYPES] = {
    [NL80211_IFTYPE_ADHOC] = {
                              .tx = 0x0000,
                              .rx = 0x0000,
                              },
    [NL80211_IFTYPE_STATION] = {
                                .tx = BIT(IEEE80211_STYPE_ACTION >> 4),
                                .rx = BIT(IEEE80211_STYPE_ACTION >> 4),
                                },
    [NL80211_IFTYPE_AP] = {
                           .tx = 0xffff,
                           .rx = BIT(IEEE80211_STYPE_ASSOC_REQ >> 4) |
                           BIT(IEEE80211_STYPE_REASSOC_REQ >> 4) |
                           BIT(IEEE80211_STYPE_PROBE_REQ >> 4) |
                           BIT(IEEE80211_STYPE_DISASSOC >> 4) |
                           BIT(IEEE80211_STYPE_AUTH >> 4) |
                           BIT(IEEE80211_STYPE_DEAUTH >> 4) |
                           BIT(IEEE80211_STYPE_ACTION >> 4),
                           },
    [NL80211_IFTYPE_AP_VLAN] = {
                                .tx = 0x0000,
                                .rx = 0x0000,
                                },
    [NL80211_IFTYPE_P2P_CLIENT] = {
                                   .tx = 0x0000,
                                   .rx = 0x0000,
                                   },
    [NL80211_IFTYPE_P2P_GO] = {
                               .tx = 0x0000,
                               .rx = 0x0000,
                               },
    [NL80211_IFTYPE_MESH_POINT] = {
                                   .tx = 0x0000,
                                   .rx = 0x0000,
                                   },
};
#endif

/** cfg80211 uAP operations */
static struct cfg80211_ops woal_cfg80211_uap_ops = {
    .add_virtual_intf = woal_cfg80211_add_virtual_intf,
    .del_virtual_intf = woal_cfg80211_del_virtual_intf,
    .set_channel = woal_cfg80211_set_channel,
    .scan = woal_uap_cfg80211_scan,
    .connect = woal_uap_cfg80211_connect,
    .disconnect = woal_uap_cfg80211_disconnect,
    .set_wiphy_params = woal_cfg80211_set_wiphy_params,
    .change_virtual_intf = woal_cfg80211_change_virtual_intf,
    .add_key = woal_cfg80211_add_key,
    .del_key = woal_cfg80211_del_key,
    .set_default_key = woal_cfg80211_set_default_key,
    .add_beacon = woal_cfg80211_add_beacon,
    .set_beacon = woal_cfg80211_set_beacon,
    .del_beacon = woal_cfg80211_del_beacon,
    .get_station = woal_uap_cfg80211_get_station,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37) || defined(COMPAT_WIRELESS)
    .mgmt_frame_register = woal_cfg80211_mgmt_frame_register,
    .mgmt_tx = woal_cfg80211_mgmt_tx,
#endif
};

/********************************************************
				Local Variables
********************************************************/

/********************************************************
				Global Variables
********************************************************/

/********************************************************
				Local Functions
********************************************************/

/********************************************************
				Global Functions
********************************************************/

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,2,0) && !defined(COMPAT_WIRELESS)
/**
 * @brief Verify RSN IE
 *
 * @param rsn_ie          Pointer RSN IE
 * @param sys_config      Pointer to mlan_uap_bss_param structure
 *
 * @return                MTRUE/MFALSE
 */
static t_u8
woal_check_rsn_ie(IEEEtypes_Rsn_t * rsn_ie, mlan_uap_bss_param * sys_config)
{
    int left = 0;
    int count = 0;
    int i = 0;
    wpa_suite_auth_key_mgmt_t *key_mgmt = NULL;
    left = rsn_ie->len + 2;
    if (left < sizeof(IEEEtypes_Rsn_t))
        return MFALSE;
    sys_config->wpa_cfg.group_cipher = 0;
    sys_config->wpa_cfg.pairwise_cipher_wpa2 = 0;
    /* check the group cipher */
    switch (rsn_ie->group_cipher.type) {
    case WPA_CIPHER_TKIP:
        sys_config->wpa_cfg.group_cipher = CIPHER_TKIP;
        break;
    case WPA_CIPHER_AES_CCM:
        sys_config->wpa_cfg.group_cipher = CIPHER_AES_CCMP;
        break;
    default:
        break;
    }
    count = le16_to_cpu(rsn_ie->pairwise_cipher.count);
    for (i = 0; i < count; i++) {
        switch (rsn_ie->pairwise_cipher.list[i].type) {
        case WPA_CIPHER_TKIP:
            sys_config->wpa_cfg.pairwise_cipher_wpa2 |= CIPHER_TKIP;
            break;
        case WPA_CIPHER_AES_CCM:
            sys_config->wpa_cfg.pairwise_cipher_wpa2 |= CIPHER_AES_CCMP;
            break;
        default:
            break;
        }
    }
    left -= sizeof(IEEEtypes_Rsn_t) + (count - 1) * sizeof(wpa_suite);
    if (left < sizeof(wpa_suite_auth_key_mgmt_t))
        return MFALSE;
    key_mgmt =
        (wpa_suite_auth_key_mgmt_t *) ((u8 *) rsn_ie + sizeof(IEEEtypes_Rsn_t) +
                                       (count - 1) * sizeof(wpa_suite));
    count = le16_to_cpu(key_mgmt->count);
    if (left <
        (sizeof(wpa_suite_auth_key_mgmt_t) + (count - 1) * sizeof(wpa_suite)))
        return MFALSE;

    for (i = 0; i < count; i++) {
        switch (key_mgmt->list[i].type) {
        case RSN_AKM_8021X:
            sys_config->key_mgmt = KEY_MGMT_EAP;
            break;
        case RSN_AKM_PSK:
            sys_config->key_mgmt = KEY_MGMT_PSK;
            break;
        }
    }
    return MTRUE;
}

/**
 * @brief Verify WPA IE
 *
 * @param wpa_ie          Pointer WPA IE
 * @param sys_config      Pointer to mlan_uap_bss_param structure
 *
 * @return                MTRUE/MFALSE
 */
static t_u8
woal_check_wpa_ie(IEEEtypes_Wpa_t * wpa_ie, mlan_uap_bss_param * sys_config)
{
    int left = 0;
    int count = 0;
    int i = 0;
    wpa_suite_auth_key_mgmt_t *key_mgmt = NULL;
    left = wpa_ie->len + 2;
    if (left < sizeof(IEEEtypes_Wpa_t))
        return MFALSE;
    sys_config->wpa_cfg.group_cipher = 0;
    sys_config->wpa_cfg.pairwise_cipher_wpa = 0;
    switch (wpa_ie->group_cipher.type) {
    case WPA_CIPHER_TKIP:
        sys_config->wpa_cfg.group_cipher = CIPHER_TKIP;
        break;
    case WPA_CIPHER_AES_CCM:
        sys_config->wpa_cfg.group_cipher = CIPHER_AES_CCMP;
        break;
    default:
        break;
    }
    count = le16_to_cpu(wpa_ie->pairwise_cipher.count);
    for (i = 0; i < count; i++) {
        switch (wpa_ie->pairwise_cipher.list[i].type) {
        case WPA_CIPHER_TKIP:
            sys_config->wpa_cfg.pairwise_cipher_wpa |= CIPHER_TKIP;
            break;
        case WPA_CIPHER_AES_CCM:
            sys_config->wpa_cfg.pairwise_cipher_wpa |= CIPHER_AES_CCMP;
            break;
        default:
            break;
        }
    }
    left -= sizeof(IEEEtypes_Wpa_t) + (count - 1) * sizeof(wpa_suite);
    if (left < sizeof(wpa_suite_auth_key_mgmt_t))
        return MFALSE;
    key_mgmt =
        (wpa_suite_auth_key_mgmt_t *) ((u8 *) wpa_ie + sizeof(IEEEtypes_Wpa_t) +
                                       (count - 1) * sizeof(wpa_suite));
    count = le16_to_cpu(key_mgmt->count);
    if (left <
        (sizeof(wpa_suite_auth_key_mgmt_t) + (count - 1) * sizeof(wpa_suite)))
        return MFALSE;
    for (i = 0; i < count; i++) {
        switch (key_mgmt->list[i].type) {
        case RSN_AKM_8021X:
            sys_config->key_mgmt = KEY_MGMT_EAP;
            break;
        case RSN_AKM_PSK:
            sys_config->key_mgmt = KEY_MGMT_PSK;
            break;
        }
    }
    return MTRUE;
}

/**
 * @brief Find RSN/WPA IES
 *
 * @param ie              Pointer IE buffer
 * @param sys_config      Pointer to mlan_uap_bss_param structure
 *
 * @return                MTRUE/MFALSE
 */
static t_u8
woal_find_wpa_ies(t_u8 * ie, int len, mlan_uap_bss_param * sys_config)
{
    int bytes_left = len;
    t_u8 *pcurrent_ptr = ie;
    t_u16 total_ie_len;
    t_u8 element_len;
    t_u8 wpa2 = 0;
    t_u8 wpa = 0;
    t_u8 ret = MFALSE;
    IEEEtypes_ElementId_e element_id;
    IEEEtypes_VendorSpecific_t *pvendor_ie;
    const t_u8 wpa_oui[4] = { 0x00, 0x50, 0xf2, 0x01 };

    while (bytes_left >= 2) {
        element_id = (IEEEtypes_ElementId_e) (*((t_u8 *) pcurrent_ptr));
        element_len = *((t_u8 *) pcurrent_ptr + 1);
        total_ie_len = element_len + sizeof(IEEEtypes_Header_t);
        if (bytes_left < total_ie_len) {
            PRINTM(MERROR, "InterpretIE: Error in processing IE, "
                   "bytes left < IE length\n");
            bytes_left = 0;
            continue;
        }
        switch (element_id) {
        case RSN_IE:
            wpa2 =
                woal_check_rsn_ie((IEEEtypes_Rsn_t *) pcurrent_ptr, sys_config);
            break;
        case VENDOR_SPECIFIC_221:
            pvendor_ie = (IEEEtypes_VendorSpecific_t *) pcurrent_ptr;
            if (!memcmp(pvendor_ie->vend_hdr.oui, wpa_oui, sizeof(wpa_oui)))
                wpa =
                    woal_check_wpa_ie((IEEEtypes_Wpa_t *) pcurrent_ptr,
                                      sys_config);
            break;
        default:
            break;
        }
        pcurrent_ptr += element_len + 2;
        /* Need to account for IE ID and IE Len */
        bytes_left -= (element_len + 2);
    }
    if (wpa && wpa2) {
        sys_config->protocol = PROTOCOL_WPA | PROTOCOL_WPA2;
        ret = MTRUE;
    } else if (wpa2) {
        sys_config->protocol = PROTOCOL_WPA2;
        ret = MTRUE;
    } else if (wpa) {
        sys_config->protocol = PROTOCOL_WPA;
        ret = MTRUE;
    }
    return ret;
}
#endif

/**
 * @brief initialize AP or GO bss config
 *
 * @param priv            A pointer to moal private structure
 * @param params          A pointer to beacon_parameters structure
 * @return                0 -- success, otherwise fail
 */
static int
woal_cfg80211_beacon_config(moal_private * priv,
                            struct beacon_parameters *params)
{
    int ret = 0;
    mlan_uap_bss_param sys_config;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,2,0) || defined(COMPAT_WIRELESS)
    int i = 0;
#else
    const t_u8 *ssid_ie = NULL;
    struct ieee80211_mgmt *head = NULL;
    t_u16 capab_info = 0;
#endif

    ENTER();

    if (params == NULL) {
        ret = -EFAULT;
        goto done;
    }

    if (priv->bss_type != MLAN_BSS_TYPE_UAP
#ifdef WIFI_DIRECT_SUPPORT
        && priv->bss_type != MLAN_BSS_TYPE_WIFIDIRECT
#endif
        ) {
        ret = -EFAULT;
        goto done;
    }

    /* Initialize the uap bss values which are uploaded from firmware */
    if (MLAN_STATUS_SUCCESS != woal_set_get_sys_config(priv,
                                                       MLAN_ACT_GET,
                                                       MOAL_IOCTL_WAIT,
                                                       &sys_config)) {
        PRINTM(MERROR, "Error getting AP confiruration\n");
        ret = -EFAULT;
        goto done;
    }

    /* Setting the default values */
    sys_config.channel = 6;
    sys_config.preamble_type = 0;

    if (priv->bss_type == MLAN_BSS_TYPE_UAP) {
        if (params->interval)
            sys_config.beacon_period = params->interval;
        if (params->dtim_period)
            sys_config.dtim_period = params->dtim_period;
    }
    if (priv->channel)
        sys_config.channel = priv->channel;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,2,0) || defined(COMPAT_WIRELESS)
    if (!params->ssid || !params->ssid_len) {
        ret = -EINVAL;
        goto done;
    }
    memcpy(sys_config.ssid.ssid, params->ssid,
           MIN(MLAN_MAX_SSID_LENGTH, params->ssid_len));
    sys_config.ssid.ssid_len = MIN(MLAN_MAX_SSID_LENGTH, params->ssid_len);
    if (params->hidden_ssid)
        sys_config.bcast_ssid_ctl = 0;
    else
        sys_config.bcast_ssid_ctl = 1;
    if (params->auth_type == NL80211_AUTHTYPE_SHARED_KEY)
        sys_config.auth_mode = MLAN_AUTH_MODE_SHARED;
    else
        sys_config.auth_mode = MLAN_AUTH_MODE_OPEN;

    for (i = 0; i < params->crypto.n_akm_suites; i++) {
        switch (params->crypto.akm_suites[i]) {
        case WLAN_AKM_SUITE_8021X:
            sys_config.key_mgmt = KEY_MGMT_EAP;
            if ((params->crypto.wpa_versions & NL80211_WPA_VERSION_1) &&
                (params->crypto.wpa_versions & NL80211_WPA_VERSION_2))
                sys_config.protocol = PROTOCOL_WPA | PROTOCOL_WPA2;
            else if (params->crypto.wpa_versions & NL80211_WPA_VERSION_2)
                sys_config.protocol = PROTOCOL_WPA2;
            else if (params->crypto.wpa_versions & NL80211_WPA_VERSION_1)
                sys_config.protocol = PROTOCOL_WPA;
            break;
        case WLAN_AKM_SUITE_PSK:
            sys_config.key_mgmt = KEY_MGMT_PSK;
            if ((params->crypto.wpa_versions & NL80211_WPA_VERSION_1) &&
                (params->crypto.wpa_versions & NL80211_WPA_VERSION_2))
                sys_config.protocol = PROTOCOL_WPA | PROTOCOL_WPA2;
            else if (params->crypto.wpa_versions & NL80211_WPA_VERSION_2)
                sys_config.protocol = PROTOCOL_WPA2;
            else if (params->crypto.wpa_versions & NL80211_WPA_VERSION_1)
                sys_config.protocol = PROTOCOL_WPA;
            break;
        }
    }
    sys_config.wpa_cfg.pairwise_cipher_wpa = 0;
    sys_config.wpa_cfg.pairwise_cipher_wpa2 = 0;
    for (i = 0; i < params->crypto.n_ciphers_pairwise; i++) {
        switch (params->crypto.ciphers_pairwise[i]) {
        case WLAN_CIPHER_SUITE_WEP40:
        case WLAN_CIPHER_SUITE_WEP104:
            break;
        case WLAN_CIPHER_SUITE_TKIP:
            if (params->crypto.wpa_versions & NL80211_WPA_VERSION_1)
                sys_config.wpa_cfg.pairwise_cipher_wpa |= CIPHER_TKIP;
            if (params->crypto.wpa_versions & NL80211_WPA_VERSION_2)
                sys_config.wpa_cfg.pairwise_cipher_wpa2 |= CIPHER_TKIP;
            break;
        case WLAN_CIPHER_SUITE_CCMP:
            if (params->crypto.wpa_versions & NL80211_WPA_VERSION_1)
                sys_config.wpa_cfg.pairwise_cipher_wpa |= CIPHER_AES_CCMP;
            if (params->crypto.wpa_versions & NL80211_WPA_VERSION_2)
                sys_config.wpa_cfg.pairwise_cipher_wpa2 |= CIPHER_AES_CCMP;
            break;
        }
    }
    switch (params->crypto.cipher_group) {
    case WLAN_CIPHER_SUITE_WEP40:
    case WLAN_CIPHER_SUITE_WEP104:
        if ((priv->cipher == WLAN_CIPHER_SUITE_WEP40) ||
            (priv->cipher == WLAN_CIPHER_SUITE_WEP104)) {
            sys_config.protocol = PROTOCOL_STATIC_WEP;
            sys_config.key_mgmt = KEY_MGMT_NONE;
            sys_config.wpa_cfg.length = 0;
            sys_config.wep_cfg.key0.key_index = priv->key_index;
            sys_config.wep_cfg.key0.is_default = 1;
            sys_config.wep_cfg.key0.length = priv->key_len;
            memcpy(sys_config.wep_cfg.key0.key, priv->key_material,
                   priv->key_len);
        }
        break;
    case WLAN_CIPHER_SUITE_TKIP:
        sys_config.wpa_cfg.group_cipher = CIPHER_TKIP;
        break;
    case WLAN_CIPHER_SUITE_CCMP:
        sys_config.wpa_cfg.group_cipher = CIPHER_AES_CCMP;
        break;
    }
#else
    /* Since in Android ICS 4.0.1's wpa_supplicant, there is no way to set ssid
       when GO (AP) starts up, so get it from beacon head parameter TODO: right
       now use hard code 24 -- ieee80211 header lenth, 12 -- fixed element
       length for beacon */
#define BEACON_IE_OFFSET	36
    /* Find SSID in head SSID IE id: 0, right now use hard code */
    ssid_ie = woal_parse_ie_tlv(params->head + BEACON_IE_OFFSET,
                                params->head_len - BEACON_IE_OFFSET, 0);
    if (!ssid_ie) {
        PRINTM(MERROR, "No ssid IE found.\n");
        ret = -EFAULT;
        goto done;
    }
    if (*(ssid_ie + 1) > 32) {
        PRINTM(MERROR, "ssid len error: %d\n", *(ssid_ie + 1));
        ret = -EFAULT;
        goto done;
    }
    memcpy(sys_config.ssid.ssid, ssid_ie + 2, *(ssid_ie + 1));
    sys_config.ssid.ssid_len = *(ssid_ie + 1);
    head = (struct ieee80211_mgmt *) params->head;
    capab_info = le16_to_cpu(head->u.beacon.capab_info);
    PRINTM(MIOCTL, "capab_info=0x%x\n", head->u.beacon.capab_info);
    sys_config.auth_mode = MLAN_AUTH_MODE_OPEN;
        /** For ICS, we don't support OPEN mode */
    if ((priv->cipher == WLAN_CIPHER_SUITE_WEP40) ||
        (priv->cipher == WLAN_CIPHER_SUITE_WEP104)) {
        sys_config.protocol = PROTOCOL_STATIC_WEP;
        sys_config.key_mgmt = KEY_MGMT_NONE;
        sys_config.wpa_cfg.length = 0;
        sys_config.wep_cfg.key0.key_index = priv->key_index;
        sys_config.wep_cfg.key0.is_default = 1;
        sys_config.wep_cfg.key0.length = priv->key_len;
        memcpy(sys_config.wep_cfg.key0.key, priv->key_material, priv->key_len);
    } else {
                /** Get cipher and key_mgmt from RSN/WPA IE */
        if (capab_info & WLAN_CAPABILITY_PRIVACY) {
            if (MFALSE ==
                woal_find_wpa_ies(params->tail, params->tail_len,
                                  &sys_config)) {
                /* hard code setting to wpa2-psk */
                sys_config.protocol = PROTOCOL_WPA2;
                sys_config.key_mgmt = KEY_MGMT_PSK;
                sys_config.wpa_cfg.pairwise_cipher_wpa2 = CIPHER_AES_CCMP;
                sys_config.wpa_cfg.group_cipher = CIPHER_AES_CCMP;
            }
        }
    }
#endif /* COMPAT_WIRELESS */
    /* If the security mode is configured as WEP or WPA-PSK, it will disable
       11n automatically, and if configured as open(off) or wpa2-psk, it will
       automatically enable 11n */
    if ((sys_config.protocol == PROTOCOL_STATIC_WEP) ||
        (sys_config.protocol == PROTOCOL_WPA))
        woal_uap_set_11n_status(&sys_config, MLAN_ACT_DISABLE);
    else
        woal_uap_set_11n_status(&sys_config, MLAN_ACT_ENABLE);
    if (MLAN_STATUS_SUCCESS != woal_set_get_sys_config(priv,
                                                       MLAN_ACT_SET,
                                                       MOAL_IOCTL_WAIT,
                                                       &sys_config)) {
        ret = -EFAULT;
        goto done;
    }
  done:
    LEAVE();
    return ret;
}

static int
woal_mon_open(struct net_device *ndev)
{
    ENTER();
    LEAVE();
    return 0;
}

static int
woal_mon_close(struct net_device *ndev)
{
    ENTER();
    LEAVE();
    return 0;
}

static int
woal_mon_set_mac_address(struct net_device *ndev, void *addr)
{
    ENTER();
    LEAVE();
    return 0;
}

static void
woal_mon_set_multicast_list(struct net_device *ndev)
{
    ENTER();
    LEAVE();
}

static int
woal_mon_hard_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
    int len_rthdr;
    int qos_len = 0;
    int dot11_hdr_len = 24;
    int snap_len = 6;
    unsigned char *pdata;
    unsigned short fc;
    unsigned char src_mac_addr[6];
    unsigned char dst_mac_addr[6];
    struct ieee80211_hdr *dot11_hdr;
    struct ieee80211_radiotap_header *prthdr =
        (struct ieee80211_radiotap_header *) skb->data;
    monitor_iface *mon_if = netdev_priv(ndev);

    if (mon_if == NULL || mon_if->base_ndev == NULL) {
        goto fail;
    }

    ENTER();
    /* check for not even having the fixed radiotap header part */
    if (unlikely(skb->len < sizeof(struct ieee80211_radiotap_header))) {
        PRINTM(MERROR, "Invalid radiotap hdr length,"
               "skb->len: %d\n", skb->len);
        goto fail;              /* too short to be possibly valid */
    }

    /* is it a header version we can trust to find length from? */
    if (unlikely(prthdr->it_version))
        goto fail;              /* only version 0 is supported */

    /* then there must be a radiotap header with a length we can use */
    len_rthdr = ieee80211_get_radiotap_len(skb->data);

    /* does the skb contain enough to deliver on the alleged length? */
    if (unlikely(skb->len < len_rthdr)) {
        PRINTM(MERROR, "Invalid data length," "skb->len: %d\n", skb->len);
        goto fail;              /* skb too short for claimed rt header extent */
    }

    /* Skip the ratiotap header */
    skb_pull(skb, len_rthdr);

    dot11_hdr = (struct ieee80211_hdr *) skb->data;
    fc = le16_to_cpu(dot11_hdr->frame_control);
    if ((fc & IEEE80211_FCTL_FTYPE) == IEEE80211_FTYPE_DATA) {
        /* Check if this ia a Wireless Distribution System (WDS) frame which
           has 4 MAC addresses */
        if (dot11_hdr->frame_control & 0x0080)
            qos_len = 2;
        if ((dot11_hdr->frame_control & 0x0300) == 0x0300)
            dot11_hdr_len += 6;

        memcpy(dst_mac_addr, dot11_hdr->addr1, sizeof(dst_mac_addr));
        memcpy(src_mac_addr, dot11_hdr->addr2, sizeof(src_mac_addr));

        /* Skip the 802.11 header, QoS (if any) and SNAP, but leave spaces for
           for two MAC addresses */
        skb_pull(skb,
                 dot11_hdr_len + qos_len + snap_len - sizeof(src_mac_addr) * 2);
        pdata = (unsigned char *) skb->data;
        memcpy(pdata, dst_mac_addr, sizeof(dst_mac_addr));
        memcpy(pdata + sizeof(dst_mac_addr), src_mac_addr,
               sizeof(src_mac_addr));

        LEAVE();
        return woal_hard_start_xmit(skb, mon_if->base_ndev);
    }

  fail:
    dev_kfree_skb(skb);
    return NETDEV_TX_OK;
}

static const struct net_device_ops woal_cfg80211_mon_if_ops = {
    .ndo_open = woal_mon_open,
    .ndo_start_xmit = woal_mon_hard_start_xmit,
    .ndo_stop = woal_mon_close,
    .ndo_set_mac_address = woal_mon_set_mac_address,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,2,0)
    .ndo_set_rx_mode = woal_mon_set_multicast_list,
#else
    .ndo_set_multicast_list = woal_mon_set_multicast_list,
#endif
};

static void
woal_mon_if_setup(struct net_device *dev)
{
    ENTER();
    ether_setup(dev);
    dev->netdev_ops = &woal_cfg80211_mon_if_ops;
    dev->destructor = free_netdev;
    LEAVE();
}

/**
 * @brief Request the driver to add a monitor interface
 *
 * @param wiphy           A pointer to wiphy structure
 * @param name            Virtual interface name
 * @param flags           Flags for the virtual interface
 * @param params          A pointer to vif_params structure
 * @param new_dev         Netdevice to be passed out
 *
 * @return                A pointer to net_device -- success, otherwise null
 */
static int
woal_cfg80211_add_mon_if(struct wiphy *wiphy, char *name,
                         u32 * flags, struct vif_params *params,
                         struct net_device **new_dev)
{
    int ret;
    struct net_device *ndev;
    monitor_iface *mon_if;
    moal_private *priv;

    ENTER();

    ASSERT_RTNL();

    priv = (moal_private *) woal_get_wiphy_priv(wiphy);

    ndev = alloc_netdev_mq(sizeof(*mon_if), name, woal_mon_if_setup, 1);
    if (!ndev) {
        PRINTM(MFATAL, "Init virtual ethernet device failed\n");
        ret = -EFAULT;
        goto fail;
    }

    dev_net_set(ndev, wiphy_net(wiphy));

    ret = dev_alloc_name(ndev, ndev->name);
    if (ret < 0) {
        PRINTM(MFATAL, "Net device alloc name fail.\n");
        goto fail;
    }

    memcpy(ndev->perm_addr, wiphy->perm_addr, ETH_ALEN);
    memcpy(ndev->dev_addr, ndev->perm_addr, ETH_ALEN);
    SET_NETDEV_DEV(ndev, wiphy_dev(wiphy));

    mon_if = netdev_priv(ndev);
    ndev->ieee80211_ptr = &mon_if->wdev;
    mon_if->wdev.iftype = NL80211_IFTYPE_MONITOR;
    mon_if->wdev.wiphy = wiphy;
    memcpy(mon_if->ifname, ndev->name, IFNAMSIZ);

    ndev->type = ARPHRD_IEEE80211_RADIOTAP;
    ndev->netdev_ops = &woal_cfg80211_mon_if_ops;

    mon_if->priv = priv;
    mon_if->mon_ndev = ndev;
    mon_if->base_ndev = priv->netdev;
    mon_if->radiotap_enabled = 1;
    mon_if->flag = 1;

    ret = register_netdevice(ndev);
    if (ret) {
        PRINTM(MFATAL, "register net_device failed, ret=%d\n", ret);
        goto fail;
    }

    if (new_dev)
        *new_dev = ndev;

  fail:
    if (ret && ndev)
        free_netdev(ndev);
    LEAVE();
    return ret;
}

/**
 * @brief Request the driver to add a virtual interface
 *
 * @param wiphy           A pointer to wiphy structure
 * @param name            Virtual interface name
 * @param type            Virtual interface type
 * @param flags           Flags for the virtual interface
 * @param params          A pointer to vif_params structure
 *
 * @return                A pointer to net_device -- success, otherwise null
 */
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,37) || defined(COMPAT_WIRELESS)
struct net_device *
woal_cfg80211_add_virtual_intf(struct wiphy *wiphy,
                               char *name, enum nl80211_iftype type,
                               u32 * flags, struct vif_params *params)
#else
int
woal_cfg80211_add_virtual_intf(struct wiphy *wiphy,
                               char *name, enum nl80211_iftype type,
                               u32 * flags, struct vif_params *params)
#endif
{
    struct net_device *ndev = NULL;
    int ret = 0;

    ENTER();
    PRINTM(MIOCTL, "add virtual intf: %d\n", type);
    switch (type) {
    case NL80211_IFTYPE_MONITOR:
        ret = woal_cfg80211_add_mon_if(wiphy, name, flags, params, &ndev);
        break;
    default:
        PRINTM(MWARN, "Not supported if type: %d\n", type);
        ret = -EFAULT;
        break;
    }
    LEAVE();
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,37) || defined(COMPAT_WIRELESS)
    if (ret)
        return NULL;
    else
        return ndev;
#else
    return ret;
#endif
}

/**
 * @brief Request the driver to del a virtual interface
 *
 * @param wiphy           A pointer to wiphy structure
 * @param dev             The pointer to net_device
 *
 * @return               0 -- success, otherwise fail
 */
int
woal_cfg80211_del_virtual_intf(struct wiphy *wiphy, struct net_device *dev)
{
    ENTER();

    PRINTM(MIOCTL, "del virtual intf\n");
    ASSERT_RTNL();
    unregister_netdevice(dev);

    LEAVE();
    return 0;
}

static int
woal_uap_cfg80211_scan(struct wiphy *wiphy, struct net_device *dev,
                       struct cfg80211_scan_request *request)
{
    ENTER();

    cfg80211_scan_done(request, MTRUE);

    LEAVE();
    return 0;
}

static int
woal_uap_cfg80211_connect(struct wiphy *wiphy, struct net_device *dev,
                          struct cfg80211_connect_params *sme)
{
    ENTER();
    LEAVE();
    return 0;
}

static int
woal_uap_cfg80211_disconnect(struct wiphy *wiphy, struct net_device *dev,
                             t_u16 reason_code)
{
    ENTER();
    LEAVE();
    return 0;
}

/**
 * @brief initialize AP or GO parameters

 *
 * @param wiphy           A pointer to wiphy structure
 * @param dev             A pointer to net_device structure
 * @param params          A pointer to beacon_parameters structure
 * @return                0 -- success, otherwise fail
 */
int
woal_cfg80211_add_beacon(struct wiphy *wiphy,
                         struct net_device *dev,
                         struct beacon_parameters *params)
{
    moal_private *priv = (moal_private *) woal_get_netdev_priv(dev);
    int ret = 0;

    ENTER();

    PRINTM(MIOCTL, "add beacon\n");
    if (params != NULL) {
        /* bss config */
        if (MLAN_STATUS_SUCCESS != woal_cfg80211_beacon_config(priv, params)) {
            ret = -EFAULT;
            goto done;
        }

        /* set mgmt frame ies */
        if (MLAN_STATUS_SUCCESS != woal_cfg80211_mgmt_frame_ie(priv,
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,2,0) && !defined(COMPAT_WIRELESS)
                                                               params->tail,
                                                               params->tail_len,
                                                               NULL, 0, NULL, 0,
                                                               NULL, 0,
                                                               MGMT_MASK_BEACON
#else
                                                               params->tail,
                                                               params->tail_len,
                                                               params->
                                                               proberesp_ies,
                                                               params->
                                                               proberesp_ies_len,
                                                               params->
                                                               assocresp_ies,
                                                               params->
                                                               assocresp_ies_len,
                                                               NULL, 0,
                                                               MGMT_MASK_BEACON
                                                               |
                                                               MGMT_MASK_PROBE_RESP
                                                               |
                                                               MGMT_MASK_ASSOC_RESP
#endif
            )) {
            ret = -EFAULT;
            goto done;
        }
    }

    /* if the bss is stopped, then start it */
    if (priv->bss_started == MFALSE) {
        if (MLAN_STATUS_SUCCESS !=
            woal_uap_bss_ctrl(priv, MOAL_IOCTL_WAIT, UAP_BSS_START)) {
            ret = -EFAULT;
            goto done;
        }
    }

  done:
    LEAVE();
    return ret;
}

/**
 * @brief set AP or GO parameter
 *
 * @param wiphy           A pointer to wiphy structure
 * @param dev             A pointer to net_device structure
 * @param params          A pointer to beacon_parameters structure
 * @return                0 -- success, otherwise fail
 */
int
woal_cfg80211_set_beacon(struct wiphy *wiphy,
                         struct net_device *dev,
                         struct beacon_parameters *params)
{
    moal_private *priv = (moal_private *) woal_get_netdev_priv(dev);
    int ret = 0;

    ENTER();

    PRINTM(MIOCTL, "set beacon\n");
    if (params != NULL) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,2,0) && !defined(COMPAT_WIRELESS)
        if (params->tail && params->tail_len) {
            if (MLAN_STATUS_SUCCESS !=
                woal_cfg80211_mgmt_frame_ie(priv,
                                            params->tail, params->tail_len,
                                            NULL, 0, NULL, 0, NULL, 0,
                                            MGMT_MASK_BEACON)) {
                ret = -EFAULT;
                goto done;
            }
        }
#else
        if (params->beacon_ies && params->beacon_ies_len) {
            if (MLAN_STATUS_SUCCESS !=
                woal_cfg80211_mgmt_frame_ie(priv, params->tail,
                                            params->tail_len, NULL, 0, NULL, 0,
                                            NULL, 0, MGMT_MASK_BEACON)) {
                ret = -EFAULT;
                goto done;
            }
        }

        if (params->proberesp_ies && params->proberesp_ies_len) {
            if (MLAN_STATUS_SUCCESS !=
                woal_cfg80211_mgmt_frame_ie(priv, NULL, 0,
                                            params->proberesp_ies,
                                            params->proberesp_ies_len, NULL, 0,
                                            NULL, 0, MGMT_MASK_PROBE_RESP)) {
                ret = -EFAULT;
                goto done;
            }
        }

        if (params->assocresp_ies && params->assocresp_ies_len) {
            if (MLAN_STATUS_SUCCESS !=
                woal_cfg80211_mgmt_frame_ie(priv, NULL, 0, NULL, 0,
                                            params->assocresp_ies,
                                            params->assocresp_ies_len, NULL, 0,
                                            MGMT_MASK_ASSOC_RESP)) {
                ret = -EFAULT;
                goto done;
            }
        }
#endif
    }

  done:
    LEAVE();
    return ret;
}

/**
 * @brief reset AP or GO parameters
 *
 * @param wiphy           A pointer to wiphy structure
 * @param dev             A pointer to net_device structure
 *
 * @return                0 -- success, otherwise fail
 */
int
woal_cfg80211_del_beacon(struct wiphy *wiphy, struct net_device *dev)
{
    moal_private *priv = (moal_private *) woal_get_netdev_priv(dev);
    int ret = 0;

    ENTER();

    PRINTM(MIOCTL, "del beacon\n");
    /* if the bss is still running, then stop it */
    if (priv->bss_started == MTRUE) {
        if (MLAN_STATUS_SUCCESS !=
            woal_uap_bss_ctrl(priv, MOAL_IOCTL_WAIT, UAP_BSS_STOP)) {
            ret = -EFAULT;
            goto done;
        }
        if (MLAN_STATUS_SUCCESS !=
            woal_uap_bss_ctrl(priv, MOAL_IOCTL_WAIT, UAP_BSS_RESET)) {
            ret = -EFAULT;
            goto done;
        }
    }

    /* clear mgmt frame ies */
    if (MLAN_STATUS_SUCCESS !=
        woal_cfg80211_mgmt_frame_ie(priv, NULL, 0, NULL, 0, NULL, 0, NULL, 0,
                                    MGMT_MASK_BEACON | MGMT_MASK_PROBE_RESP |
                                    MGMT_MASK_ASSOC_RESP)) {
        ret = -EFAULT;
        goto done;
    }

    priv->cipher = 0;
    priv->key_len = 0;
  done:
    LEAVE();
    return ret;
}

/**
 * @brief Get station info
 *
 * @param wiphy           A pointer to wiphy structure
 * @param dev             A pointer to net_device structure
 * @param mac			  A pointer to station mac address
 * @param stainfo		  A pointer to station_info structure
 *
 * @return                0 -- success, otherwise fail
 */
int
woal_uap_cfg80211_get_station(struct wiphy *wiphy, struct net_device *dev,
                              u8 * mac, struct station_info *stainfo)
{
    moal_private *priv = (moal_private *) woal_get_netdev_priv(dev);
    int ret = -EFAULT;
    int i = 0;
    mlan_ds_get_info *info = NULL;
    mlan_ioctl_req *ioctl_req = NULL;

    ENTER();
    if (priv->media_connected == MFALSE) {
        PRINTM(MINFO, "cfg80211: Media not connected!\n");
        LEAVE();
        return -ENOENT;
    }

    /* Allocate an IOCTL request buffer */
    ioctl_req =
        (mlan_ioctl_req *) woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_get_info));
    if (ioctl_req == NULL) {
        ret = -ENOMEM;
        goto done;
    }

    info = (mlan_ds_get_info *) ioctl_req->pbuf;
    info->sub_command = MLAN_OID_UAP_STA_LIST;
    ioctl_req->req_id = MLAN_IOCTL_GET_INFO;
    ioctl_req->action = MLAN_ACT_GET;

    if (MLAN_STATUS_SUCCESS !=
        woal_request_ioctl(priv, ioctl_req, MOAL_IOCTL_WAIT)) {
        goto done;
    }
    for (i = 0; i < info->param.sta_list.sta_count; i++) {
        if (!memcmp(info->param.sta_list.info[i].mac_address, mac, ETH_ALEN)) {
            PRINTM(MIOCTL,
                   "Get station: %02x:%02x:%02x:%02x:%02x:%02x RSSI=%d\n",
                   mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
                   (int) info->param.sta_list.info[i].rssi);
            stainfo->filled = STATION_INFO_INACTIVE_TIME | STATION_INFO_SIGNAL;
            stainfo->inactive_time = 0;
            stainfo->signal = info->param.sta_list.info[i].rssi;
            ret = 0;
            break;
        }
    }
  done:
    if (ioctl_req)
        kfree(ioctl_req);
    LEAVE();
    return ret;
}

/**
 *  @brief Sets up the CFG802.11 specific HT capability fields
 *  with default values
 *
 *  @param ht_info      A pointer to ieee80211_sta_ht_cap structure
 *  @param ap_cfg       A pointer to mlan_uap_bss_param
 *
 *  @return             N/A
 */
static void
woal_cfg80211_setup_uap_ht_cap(struct ieee80211_sta_ht_cap *ht_info,
                               mlan_uap_bss_param * ap_cfg)
{
    ENTER();

    ht_info->ht_supported = true;
    ht_info->ampdu_factor = ap_cfg->ampdu_param & (MBIT(1) | MBIT(0));
    ht_info->ampdu_density =
        ap_cfg->ampdu_param & (MBIT(4) | MBIT(3) | MBIT(2));

    memset(&ht_info->mcs, 0, sizeof(ht_info->mcs));
    memcpy(ht_info->mcs.rx_mask, ap_cfg->supported_mcs_set,
           sizeof(ht_info->mcs.rx_mask));

    ht_info->cap = 0;
    if (ap_cfg->ht_cap_info & MBIT(1))  /* 20/40 Mhz enable */
        ht_info->cap |= IEEE80211_HT_CAP_SUP_WIDTH_20_40;
    if (ap_cfg->ht_cap_info & MBIT(4))  /* Green field supported */
        ht_info->cap |= IEEE80211_HT_CAP_GRN_FLD;
    if (ap_cfg->ht_cap_info & MBIT(5))  /* Short GI @ 20Mhz supported */
        ht_info->cap |= IEEE80211_HT_CAP_SGI_20;
    if (ap_cfg->ht_cap_info & MBIT(6))  /* Short GI @ 40Mhz supported */
        ht_info->cap |= IEEE80211_HT_CAP_SGI_40;
    if (ap_cfg->ht_cap_info & MBIT(12)) /* DSS/CCK mode in 40MHz enable */
        ht_info->cap |= IEEE80211_HT_CAP_DSSSCCK40;
    ht_info->mcs.tx_params = IEEE80211_HT_MCS_TX_DEFINED;

    LEAVE();
}

/**
 * @brief Initialize the uAP wiphy
 *
 * @param priv            A pointer to moal_private structure
 * @param wait_option     Wait option
 *
 * @return                MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status
woal_cfg80211_uap_init_wiphy(moal_private * priv, t_u8 wait_option)
{
    struct wiphy *wiphy = priv->wdev->wiphy;
    mlan_uap_bss_param ap_cfg;

    ENTER();

    if (MLAN_STATUS_SUCCESS != woal_set_get_sys_config(priv,
                                                       MLAN_ACT_GET,
                                                       wait_option, &ap_cfg)) {
        LEAVE();
        return MLAN_STATUS_FAILURE;
    }

    /* Initialize parameters for 2GHz and 5GHz bands */
    woal_cfg80211_setup_uap_ht_cap(&wiphy->bands[IEEE80211_BAND_2GHZ]->ht_cap,
                                   &ap_cfg);
    if (wiphy->bands[IEEE80211_BAND_5GHZ])
        woal_cfg80211_setup_uap_ht_cap(&wiphy->bands[IEEE80211_BAND_5GHZ]->
                                       ht_cap, &ap_cfg);

    /* Set retry limit count to wiphy */
    wiphy->retry_long = (t_u8) ap_cfg.retry_limit;
    wiphy->retry_short = (t_u8) ap_cfg.retry_limit;
    wiphy->max_scan_ie_len = MAX_IE_SIZE;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37) || defined(COMPAT_WIRELESS)
    wiphy->mgmt_stypes = ieee80211_uap_mgmt_stypes;
#endif
    /* Set RTS threshold to wiphy */
    wiphy->rts_threshold = (t_u32) ap_cfg.rts_threshold;

    /* Set fragment threshold to wiphy */
    wiphy->frag_threshold = (t_u32) ap_cfg.frag_threshold;

    LEAVE();
    return MLAN_STATUS_SUCCESS;
}

/**
 * @brief Register the device with cfg80211
 *
 * @param dev       A pointer to net_device structure
 * @param bss_type  BSS type
 *
 * @return          MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status
woal_register_uap_cfg80211(struct net_device * dev, t_u8 bss_type)
{
    mlan_status ret = MLAN_STATUS_SUCCESS;
    moal_private *priv = (moal_private *) netdev_priv(dev);
    void *wdev_priv = NULL;
    struct wireless_dev *wdev = NULL;
    mlan_fw_info fw_info;

    ENTER();

    /* Allocate wireless device */
    wdev = kzalloc(sizeof(struct wireless_dev), GFP_KERNEL);
    if (!wdev) {
        PRINTM(MERROR, "Could not allocate wireless device\n");
        ret = MLAN_STATUS_FAILURE;
        goto err_wdev;
    }

    /* Allocate wiphy */
    wdev->wiphy = wiphy_new(&woal_cfg80211_uap_ops, sizeof(moal_private *));
    if (!wdev->wiphy) {
        PRINTM(MERROR, "Could not allocate wiphy device\n");
        ret = MLAN_STATUS_FAILURE;
        goto err_wdev;
    }
    if (bss_type == MLAN_BSS_TYPE_UAP) {
        dev_set_name(&wdev->wiphy->dev, dev->name);
        wdev->iftype = NL80211_IFTYPE_AP;
        wdev->wiphy->interface_modes =
            MBIT(NL80211_IFTYPE_AP) | MBIT(NL80211_IFTYPE_STATION) |
            MBIT(NL80211_IFTYPE_MONITOR);
        wdev->wiphy->max_scan_ssids = 10;
    }

    /* Make this wiphy known to this driver only */
    wdev->wiphy->privid = mrvl_wiphy_privid;

    /* Supported bands */
    wdev->wiphy->bands[IEEE80211_BAND_2GHZ] = &cfg80211_band_2ghz;
    if (MLAN_STATUS_SUCCESS ==
        woal_request_get_fw_info(priv, MOAL_CMD_WAIT, &fw_info)) {
        if (fw_info.fw_bands & BAND_A)
            wdev->wiphy->bands[IEEE80211_BAND_5GHZ] = &cfg80211_band_5ghz;
    }

    /* Initialize cipher suits */
    wdev->wiphy->cipher_suites = cfg80211_cipher_suites;
    wdev->wiphy->n_cipher_suites = ARRAY_SIZE(cfg80211_cipher_suites);

    wdev->wiphy->signal_type = CFG80211_SIGNAL_TYPE_MBM;

    /* We are using custom domains */
    wdev->wiphy->flags |= WIPHY_FLAG_CUSTOM_REGULATORY;

    wdev->wiphy->reg_notifier = NULL;   // TODO: woal_cfg80211_reg_notifier;

    /* Set moal_private pointer in wiphy_priv */
    wdev_priv = wiphy_priv(wdev->wiphy);

    *(unsigned long *) wdev_priv = (unsigned long) priv;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,39) || defined(COMPAT_WIRELESS)
    set_wiphy_dev(wdev->wiphy, (struct device *) priv->phandle->hotplug_device);
#endif

    if (wiphy_register(wdev->wiphy) < 0) {
        PRINTM(MERROR, "Wiphy device registration failed!\n");
        ret = MLAN_STATUS_FAILURE;
        goto err_wdev;
    }

    dev_net_set(dev, wiphy_net(wdev->wiphy));
    dev->ieee80211_ptr = wdev;
    SET_NETDEV_DEV(dev, wiphy_dev(wdev->wiphy));
    priv->wdev = wdev;

    if (ret != MLAN_STATUS_SUCCESS) {
        PRINTM(MERROR, "Wiphy device registration failed!\n");
    } else {
        PRINTM(MINFO, "Successfully registered wiphy device\n");
        LEAVE();
        return ret;
    }

    wiphy_unregister(wdev->wiphy);
  err_wdev:
    dev->ieee80211_ptr = NULL;
    if (wdev && wdev->wiphy)
        wiphy_free(wdev->wiphy);
    kfree(wdev);
    LEAVE();
    return ret;
}
