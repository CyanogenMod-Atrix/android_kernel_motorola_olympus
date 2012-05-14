/** @file  moal_eth_ioctl.c
  *
  * @brief This file contains private ioctl functions
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

/************************************************************************
Change log:
    01/05/2012: initial version
************************************************************************/

#include	"moal_main.h"
#include	"moal_eth_ioctl.h"
#include	"mlan_ioctl.h"
#if defined(STA_WEXT) || defined(UAP_WEXT)
#include	"moal_priv.h"
#endif

#if defined(STA_CFG80211) && defined(UAP_CFG80211)
#include	"moal_cfg80211.h"
#endif
#ifdef UAP_SUPPORT
#include    "moal_uap.h"
#endif
/********************************************************
                Local Variables
********************************************************/

/** Marvell private command identifier string */
#define CMD_MARVELL     "MRVL_CMD"

/** Private command: Version */
#define PRIV_CMD_VERSION    "version"
/** Private command: Band cfg */
#define PRIV_CMD_BANDCFG    "bandcfg"
/** Private command: Host cmd */
#define PRIV_CMD_HOSTCMD    "hostcmd"
/** Private command: Custom IE config*/
#define PRIV_CMD_CUSTOMIE   "customie"
/** Private command: HT Tx Cfg */
#define PRIV_CMD_HTTXCFG    "httxcfg"
#define PRIV_CMD_DATARATE   "getdatarate"
#define PRIV_CMD_TXRATECFG  "txratecfg"
#define PRIV_CMD_ESUPPMODE  "esuppmode"
#define PRIV_CMD_PASSPHRASE "passphrase"
#define PRIV_CMD_DEAUTH     "deauth"
#if defined(WIFI_DIRECT_SUPPORT)
#if defined(STA_SUPPORT) && defined(UAP_SUPPORT)
#define PRIV_CMD_BSSROLE    "bssrole"
#endif
#endif
#ifdef STA_SUPPORT
#define PRIV_CMD_SETUSERSCAN    "setuserscan"
#endif
#define PRIV_CMD_DEEPSLEEP      "deepsleep"
#define PRIV_CMD_IPADDR         "ipaddr"
#define PRIV_CMD_WPSSESSION     "wpssession"
#define PRIV_CMD_OTPUSERDATA    "otpuserdata"
#define PRIV_CMD_COUNTRYCODE    "countrycode"
#define PRIV_CMD_TCPACKENH      "tcpackenh"

/** Bands supported in Infra mode */
static t_u8 SupportedInfraBand[] = {
    BAND_B,
    BAND_B | BAND_G, BAND_G,
    BAND_GN, BAND_B | BAND_G | BAND_GN, BAND_G | BAND_GN,
    BAND_A, BAND_B | BAND_A, BAND_B | BAND_G | BAND_A, BAND_G | BAND_A,
    BAND_A | BAND_B | BAND_G | BAND_AN | BAND_GN,
        BAND_A | BAND_G | BAND_AN | BAND_GN, BAND_A | BAND_AN,
};

/** Bands supported in Ad-Hoc mode */
static t_u8 SupportedAdhocBand[] = {
    BAND_B, BAND_B | BAND_G, BAND_G,
    BAND_GN, BAND_B | BAND_G | BAND_GN, BAND_G | BAND_GN,
    BAND_A,
    BAND_AN, BAND_A | BAND_AN,
};

/********************************************************
		Global Variables
********************************************************/
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,29)
#ifdef UAP_SUPPORT
/** Network device handlers for uAP */
extern const struct net_device_ops woal_uap_netdev_ops;
#endif
#ifdef STA_SUPPORT
/** Network device handlers for STA */
extern const struct net_device_ops woal_netdev_ops;
#endif
#endif
extern int cfg80211_wext;

/********************************************************
		Local Functions
********************************************************/
/**
 * @brief Parse a string to extract arguments
 *
 * @param pos           Pointer to the arguments string
 * @param data          Pointer to the arguments buffer
 * @param user_data_len Pointer to the number of arguments extracted
 *
 * @return              MLAN_STATUS_SUCCESS
 *
 * N.B. No boundary check is done on 'data'. The caller must ensure there
 * are enough space for all extracted arguments.
 */
mlan_status
parse_arguments(t_u8 * pos, int *data, int *user_data_len)
{
    unsigned int i, j, k;
    char cdata[10];
    int is_hex = 0;

    memset(cdata, 0, sizeof(cdata));
    for (i = 0, j = 0, k = 0; i <= strlen(pos); i++) {
        if ((k == 0) && (i <= (strlen(pos) - 2))) {
            if ((pos[i] == '0') && (pos[i + 1] == 'x')) {
                is_hex = 1;
                i = i + 2;
            }
        }
        if (pos[i] == '\0') {
            if (is_hex) {
                data[j] = woal_atox(cdata);
                is_hex = 0;
            } else {
                woal_atoi(&data[j], cdata);
            }
            j++;
            (*user_data_len)++;
            k = 0;
            memset(cdata, 0, sizeof(char) * 4);
            break;
        } else if (pos[i] == ' ') {
            if (is_hex) {
                data[j] = woal_atox(cdata);
                is_hex = 0;
            } else {
                woal_atoi(&data[j], cdata);
            }
            j++;
            (*user_data_len)++;
            k = 0;
            memset(cdata, 0, sizeof(char) * 4);
        } else {
            cdata[k] = pos[i];
            k++;
        }
    }

    return MLAN_STATUS_SUCCESS;
}

#if defined(STA_CFG80211) && defined(UAP_CFG80211)
/**
 *  @brief Set wps & p2p ie in AP mode
 *
 *  @param priv         Pointer to priv stucture
 *  @param ie           Pointer to ies data
 *  @param len          Length of data
 *
 *  @return             MLAN_STATUS_SUCCESS -- success, otherwise fail
 */
mlan_status
woal_set_ap_wps_p2p_ie(moal_private * priv, t_u8 * ie, size_t len)
{
    mlan_status ret = MLAN_STATUS_SUCCESS;
    t_u8 *pos = ie;
    t_u32 ie_len;

    ENTER();

    ie_len = len - 2;
    if (ie_len <= 0 || ie_len > MAX_IE_SIZE) {
        PRINTM(MERROR, "IE len error: %d\n", ie_len);
        ret = -EFAULT;
        goto done;
    }

    /* Android cmd format: "SET_AP_WPS_P2P_IE 1" -- beacon IE
       "SET_AP_WPS_P2P_IE 2" -- proberesp IE "SET_AP_WPS_P2P_IE 4" -- assocresp
       IE */
    if (*pos == '1') {
        PRINTM(MIOCTL, "Ignore set beacon ie\n");
        goto done;
    } else if (*pos == '2') {
        /* set the probe resp ies */
        pos += 2;
        if (MLAN_STATUS_SUCCESS != woal_cfg80211_mgmt_frame_ie(priv, NULL,
                                                               0, pos, ie_len,
                                                               NULL, 0, NULL, 0,
                                                               MGMT_MASK_PROBE_RESP))
        {
            PRINTM(MERROR, "Failed to set probe resp ie\n");
            ret = -EFAULT;
            goto done;
        }
    } else if (*pos == '4') {
        /* set the assoc resp ies */
        pos += 2;
        if (MLAN_STATUS_SUCCESS != woal_cfg80211_mgmt_frame_ie(priv, NULL,
                                                               0, NULL, 0, pos,
                                                               ie_len, NULL, 0,
                                                               MGMT_MASK_ASSOC_RESP))
        {
            PRINTM(MERROR, "Failed to set assoc resp ie\n");
            ret = -EFAULT;
            goto done;
        }
    }

  done:
    LEAVE();
    return ret;
}
#endif

/**
 *  @brief Get Driver Version
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int
woal_get_priv_driver_version(moal_private * priv, t_u8 * respbuf,
                             t_u32 respbuflen)
{
    int len = 0, ret = -1;
    char buf[MLAN_MAX_VER_STR_LEN];

    ENTER();

    if (!respbuf) {
        LEAVE();
        return 0;
    }

    memset(buf, 0, sizeof(buf));

    /* Get version string to local buffer */
    woal_get_version(priv->phandle, buf, sizeof(buf) - 1);
    len = strlen(buf);

    if (len) {
        /* Copy back the retrieved version string */
        PRINTM(MINFO, "MOAL VERSION: %s\n", buf);
        ret = MIN(len, (respbuflen - 1));
        memcpy(respbuf, buf, ret);
    } else {
        ret = -1;
        PRINTM(MERROR, "Get version failed!\n");
    }

    LEAVE();
    return ret;
}

/**
 *  @brief Hostcmd interface from application
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int
woal_priv_hostcmd(moal_private * priv, t_u8 * respbuf, t_u32 respbuflen)
{
    int ret = 0;
    t_u8 *data_ptr;
    t_u32 buf_len = 0;
    HostCmd_Header cmd_header;
    mlan_ioctl_req *req = NULL;
    mlan_ds_misc_cfg *misc_cfg = NULL;

    ENTER();

    data_ptr = respbuf + (strlen(CMD_MARVELL) + strlen(PRIV_CMD_HOSTCMD));
    buf_len = *((t_u32 *) data_ptr);
    memcpy(&cmd_header, data_ptr + sizeof(buf_len), sizeof(HostCmd_Header));

    PRINTM(MINFO, "Host command len = %d\n", woal_le16_to_cpu(cmd_header.size));
    if (woal_le16_to_cpu(cmd_header.size) > MLAN_SIZE_OF_CMD_BUFFER) {
        LEAVE();
        return -EINVAL;
    }

    req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
    if (req == NULL) {
        ret = -ENOMEM;
        goto error;
    }
    misc_cfg = (mlan_ds_misc_cfg *) req->pbuf;
    misc_cfg->sub_command = MLAN_OID_MISC_HOST_CMD;
    req->req_id = MLAN_IOCTL_MISC_CFG;
    req->action = MLAN_ACT_SET;
    misc_cfg->param.hostcmd.len = woal_le16_to_cpu(cmd_header.size);
    /* get the whole command */
    memcpy(misc_cfg->param.hostcmd.cmd, data_ptr + sizeof(buf_len),
           misc_cfg->param.hostcmd.len);

    if (MLAN_STATUS_SUCCESS != woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT)) {
        ret = -EFAULT;
        goto error;
    }
    memcpy(data_ptr + sizeof(buf_len), misc_cfg->param.hostcmd.cmd,
           misc_cfg->param.hostcmd.len);
    ret =
        misc_cfg->param.hostcmd.len + sizeof(buf_len) + strlen(CMD_MARVELL) +
        strlen(PRIV_CMD_HOSTCMD);
    memcpy(data_ptr, (t_u8 *) & ret, sizeof(t_u32));

  error:
    if (req)
        kfree(req);

    LEAVE();
    return ret;
}

/**
 *  @brief Custom IE setting
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int
woal_priv_customie(moal_private * priv, t_u8 * respbuf, t_u32 respbuflen)
{
    int ret = 0;
    t_u8 *data_ptr;
    mlan_ioctl_req *ioctl_req = NULL;
    mlan_ds_misc_cfg *misc = NULL;
    mlan_ds_misc_custom_ie *custom_ie = NULL;

    ENTER();
    data_ptr = respbuf + (strlen(CMD_MARVELL) + strlen(PRIV_CMD_CUSTOMIE));

    custom_ie = (mlan_ds_misc_custom_ie *) data_ptr;
    ioctl_req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
    if (ioctl_req == NULL) {
        ret = -ENOMEM;
        goto done;
    }

    misc = (mlan_ds_misc_cfg *) ioctl_req->pbuf;
    misc->sub_command = MLAN_OID_MISC_CUSTOM_IE;
    ioctl_req->req_id = MLAN_IOCTL_MISC_CFG;
    if ((custom_ie->len == 0) ||
        (custom_ie->len == sizeof(custom_ie->ie_data_list[0].ie_index)))
        ioctl_req->action = MLAN_ACT_GET;
    else
        ioctl_req->action = MLAN_ACT_SET;

    memcpy(&misc->param.cust_ie, custom_ie, sizeof(mlan_ds_misc_custom_ie));

    if (MLAN_STATUS_SUCCESS !=
        woal_request_ioctl(priv, ioctl_req, MOAL_IOCTL_WAIT)) {
        ret = -EFAULT;
        goto done;
    }
    custom_ie = (mlan_ds_misc_custom_ie *) data_ptr;
    memcpy(custom_ie, &misc->param.cust_ie, sizeof(mlan_ds_misc_custom_ie));
    ret = sizeof(mlan_ds_misc_custom_ie);
    if (ioctl_req->status_code == MLAN_ERROR_IOCTL_FAIL) {
        /* send a separate error code to indicate error from driver */
        ret = EFAULT;
    }
  done:
    if (ioctl_req) {
        kfree(ioctl_req);
    }
    LEAVE();
    return ret;
}

/**
 *  @brief Set/Get Band and Adhoc-band setting
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int
woal_setget_priv_bandcfg(moal_private * priv, t_u8 * respbuf, t_u32 respbuflen)
{
    int ret = 0;
    unsigned int i;
    int data[4];
    int user_data_len = 0;
    t_u32 infra_band = 0;
    t_u32 adhoc_band = 0;
    t_u32 adhoc_channel = 0;
    t_u32 adhoc_chan_bandwidth = 0;
    mlan_ioctl_req *req = NULL;
    mlan_ds_radio_cfg *radio_cfg = NULL;
    mlan_ds_band_cfg *band_cfg = NULL;

    ENTER();

    if (strlen(respbuf) == (strlen(CMD_MARVELL) + strlen(PRIV_CMD_BANDCFG))) {
        /* GET operation */
        user_data_len = 0;
    } else {
        /* SET operation */
        memset((char *) data, 0, sizeof(data));
        parse_arguments(respbuf + strlen(CMD_MARVELL) +
                        strlen(PRIV_CMD_BANDCFG), data, &user_data_len);
    }

    if (sizeof(int) * user_data_len > sizeof(data)) {
        PRINTM(MERROR, "Too many arguments\n");
        LEAVE();
        return -EINVAL;
    }

    if (user_data_len > 0) {
        if (priv->media_connected == MTRUE) {
            LEAVE();
            return -EOPNOTSUPP;
        }
    }

    req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_radio_cfg));
    if (req == NULL) {
        ret = -ENOMEM;
        goto error;
    }
    radio_cfg = (mlan_ds_radio_cfg *) req->pbuf;
    radio_cfg->sub_command = MLAN_OID_BAND_CFG;
    req->req_id = MLAN_IOCTL_RADIO_CFG;

    if (user_data_len == 0) {
        /* Get config_bands, adhoc_start_band and adhoc_channel values from
           MLAN */
        req->action = MLAN_ACT_GET;
    } else {
        /* To support only <b/bg/bgn/n/aac/gac> */
        infra_band = data[0];
        for (i = 0; i < sizeof(SupportedInfraBand); i++)
            if (infra_band == SupportedInfraBand[i])
                break;
        if (i == sizeof(SupportedInfraBand)) {
            ret = -EINVAL;
            goto error;
        }

        /* Set Adhoc band */
        if (user_data_len >= 2) {
            adhoc_band = data[1];
            for (i = 0; i < sizeof(SupportedAdhocBand); i++)
                if (adhoc_band == SupportedAdhocBand[i])
                    break;
            if (i == sizeof(SupportedAdhocBand)) {
                ret = -EINVAL;
                goto error;
            }
        }

        /* Set Adhoc channel */
        if (user_data_len >= 3) {
            adhoc_channel = data[2];
            if (adhoc_channel == 0) {
                /* Check if specified adhoc channel is non-zero */
                ret = -EINVAL;
                goto error;
            }
        }
        if (user_data_len == 4) {
            if (!(adhoc_band & (BAND_GN | BAND_AN))) {
                PRINTM(MERROR,
                       "11n is not enabled for adhoc, can not set HT/VHT channel bandwidth\n");
                ret = -EINVAL;
                goto error;
            }
            adhoc_chan_bandwidth = data[3];
            /* sanity test */
            if ((adhoc_chan_bandwidth != CHANNEL_BW_20MHZ) &&
                (adhoc_chan_bandwidth != CHANNEL_BW_40MHZ_ABOVE) &&
                (adhoc_chan_bandwidth != CHANNEL_BW_40MHZ_BELOW)
                ) {
                PRINTM(MERROR,
                       "Invalid secondary channel bandwidth, only allowed 0, 1, 3 or 4\n");
                ret = -EINVAL;
                goto error;
            }

        }
        /* Set config_bands and adhoc_start_band values to MLAN */
        req->action = MLAN_ACT_SET;
        radio_cfg->param.band_cfg.config_bands = infra_band;
        radio_cfg->param.band_cfg.adhoc_start_band = adhoc_band;
        radio_cfg->param.band_cfg.adhoc_channel = adhoc_channel;
        radio_cfg->param.band_cfg.sec_chan_offset = adhoc_chan_bandwidth;
    }

    if (MLAN_STATUS_SUCCESS != woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT)) {
        ret = -EFAULT;
        goto error;
    }

    band_cfg = (mlan_ds_band_cfg *) respbuf;

    memcpy(band_cfg, &radio_cfg->param.band_cfg, sizeof(mlan_ds_band_cfg));

    ret = sizeof(mlan_ds_band_cfg);

  error:
    if (req)
        kfree(req);

    LEAVE();
    return ret;
}

/**
 *  @brief Set/Get 11n configurations
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int
woal_setget_priv_httxcfg(moal_private * priv, t_u8 * respbuf, t_u32 respbuflen)
{
    int data[2];
    mlan_ioctl_req *req = NULL;
    mlan_ds_11n_cfg *cfg_11n = NULL;
    int ret = 0;
    int user_data_len = 0;

    ENTER();

    if (strlen(respbuf) == (strlen(CMD_MARVELL) + strlen(PRIV_CMD_HTTXCFG))) {
        /* GET operation */
        user_data_len = 0;
    } else {
        /* SET operation */
        memset((char *) data, 0, sizeof(data));
        parse_arguments(respbuf + strlen(CMD_MARVELL) +
                        strlen(PRIV_CMD_HTTXCFG), data, &user_data_len);
    }

    if (user_data_len > 2) {
        PRINTM(MERROR, "Invalid number of arguments\n");
        ret = -EINVAL;
        goto done;
    }
    req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_11n_cfg));
    if (req == NULL) {
        ret = -ENOMEM;
        goto done;
    }

    cfg_11n = (mlan_ds_11n_cfg *) req->pbuf;
    cfg_11n->sub_command = MLAN_OID_11N_CFG_TX;
    req->req_id = MLAN_IOCTL_11N_CFG;

    if (user_data_len == 0) {
        /* Get 11n tx parameters from MLAN */
        req->action = MLAN_ACT_GET;
        cfg_11n->param.tx_cfg.misc_cfg = BAND_SELECT_BG;
    } else {
        cfg_11n->param.tx_cfg.httxcap = data[0];
        PRINTM(MINFO, "SET: httxcap:0x%x\n", data[0]);
        cfg_11n->param.tx_cfg.misc_cfg = BAND_SELECT_BOTH;
        if (user_data_len == 2) {
            if (data[1] != BAND_SELECT_BG &&
                data[1] != BAND_SELECT_A && data[1] != BAND_SELECT_BOTH) {
                PRINTM(MERROR, "Invalid band selection\n");
                ret = -EINVAL;
                goto done;
            }
            cfg_11n->param.tx_cfg.misc_cfg = data[1];
            PRINTM(MINFO, "SET: httxcap band:0x%x\n", data[1]);
        }
        /* Update 11n tx parameters in MLAN */
        req->action = MLAN_ACT_SET;
    }
    if (MLAN_STATUS_SUCCESS != woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT)) {
        ret = -EFAULT;
        goto done;
    }
    data[0] = cfg_11n->param.tx_cfg.httxcap;

    if (req->action == MLAN_ACT_GET) {
        user_data_len = 1;
        cfg_11n->param.tx_cfg.httxcap = 0;
        cfg_11n->param.tx_cfg.misc_cfg = BAND_SELECT_A;
        if (MLAN_STATUS_SUCCESS !=
            woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT)) {
            ret = -EFAULT;
            goto done;
        }
        if (cfg_11n->param.tx_cfg.httxcap != data[0]) {
            user_data_len = 2;
            data[1] = cfg_11n->param.tx_cfg.httxcap;
            PRINTM(MINFO, "GET: httxcap for 2.4GHz:0x%x\n", data[0]);
            PRINTM(MINFO, "GET: httxcap for 5GHz:0x%x\n", data[1]);
        } else
            PRINTM(MINFO, "GET: httxcap:0x%x\n", data[0]);
    }

    sprintf(respbuf, "0x%x", data[0]);
    ret = strlen(respbuf) + 1;

  done:
    if (req)
        kfree(req);
    LEAVE();
    return ret;

}

/**
 *  @brief Set/Get 11AC configurations
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int
woal_get_priv_datarate(moal_private * priv, t_u8 * respbuf, t_u32 respbuflen)
{
    mlan_ioctl_req *req = NULL;
    mlan_ds_rate *rate = NULL;
    mlan_data_rate *data_rate = NULL;
    int ret = 0;

    ENTER();

    req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_rate));
    if (req == NULL) {
        ret = -ENOMEM;
        goto done;
    }

    rate = (mlan_ds_rate *) req->pbuf;
    rate->sub_command = MLAN_OID_GET_DATA_RATE;
    req->req_id = MLAN_IOCTL_RATE;
    req->action = MLAN_ACT_GET;

    if (MLAN_STATUS_SUCCESS != woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT)) {
        ret = -EFAULT;
        goto done;
    }

    data_rate = (mlan_data_rate *) respbuf;

    memcpy(data_rate, &rate->param.data_rate, sizeof(mlan_data_rate));

    ret = sizeof(mlan_data_rate);

  done:
    if (req)
        kfree(req);
    LEAVE();
    return ret;

}

/**
 *  @brief Set/Get tx rate configurations
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int
woal_setget_priv_txratecfg(moal_private * priv, t_u8 * respbuf,
                           t_u32 respbuflen)
{
    t_u32 data[3];
    mlan_ioctl_req *req = NULL;
    mlan_ds_rate *rate = NULL;
    woal_tx_rate_cfg *ratecfg = NULL;
    int ret = 0;
    int user_data_len = 0;

    ENTER();

    if (strlen(respbuf) == (strlen(CMD_MARVELL) + strlen(PRIV_CMD_TXRATECFG))) {
        /* GET operation */
        user_data_len = 0;
    } else {
        /* SET operation */
        memset((char *) data, 0, sizeof(data));
        parse_arguments(respbuf + strlen(CMD_MARVELL) +
                        strlen(PRIV_CMD_TXRATECFG), data, &user_data_len);
    }

    if (user_data_len >= 4) {
        PRINTM(MERROR, "Invalid number of arguments\n");
        ret = -EINVAL;
        goto done;
    }

    req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_rate));
    if (req == NULL) {
        ret = -ENOMEM;
        goto done;
    }

    req->req_id = MLAN_IOCTL_RATE;
    rate = (mlan_ds_rate *) req->pbuf;
    rate->sub_command = MLAN_OID_RATE_CFG;
    rate->param.rate_cfg.rate_type = MLAN_RATE_INDEX;

    if (user_data_len == 0) {
        /* Get operation */
        req->action = MLAN_ACT_GET;
    } else {
        /* Set operation */
        req->action = MLAN_ACT_SET;
        /* format */
        if ((data[0] != AUTO_RATE) && (data[0] >= 3)) {
            PRINTM(MERROR, "Invalid format selection\n");
            ret = -EINVAL;
            goto done;
        }
        if (data[0] == AUTO_RATE) {
            /* auto */
            rate->param.rate_cfg.is_rate_auto = 1;
        } else {
            /* fixed rate */
            PRINTM(MINFO, "SET: txratefg format: 0x%x\n", data[0]);
            if ((data[0] != AUTO_RATE) && (data[0] > MLAN_RATE_FORMAT_HT)
                ) {
                PRINTM(MERROR, "Invalid format selection\n");
                ret = -EINVAL;
                goto done;
            }
        }

        if ((user_data_len >= 2) && (data[0] != AUTO_RATE)) {
            PRINTM(MINFO, "SET: txratefg index: 0x%x\n", data[1]);
            /* sanity check */
            if (((data[0] == MLAN_RATE_FORMAT_LG) &&
                 (data[1] > MLAN_RATE_INDEX_OFDM7))
                || ((data[0] == MLAN_RATE_FORMAT_HT) && (data[1] != 32) &&
                    (data[1] > 15))
                ) {
                PRINTM(MERROR, "Invalid index selection\n");
                ret = -EINVAL;
                goto done;
            }

            PRINTM(MINFO, "SET: txratefg index: 0x%x\n", data[1]);
            rate->param.rate_cfg.rate = data[1];

            if (data[0] == MLAN_RATE_FORMAT_HT) {
                rate->param.rate_cfg.rate = data[1] + MLAN_RATE_INDEX_MCS0;
            }
        }

    }

    if (MLAN_STATUS_SUCCESS != woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT)) {
        ret = -EFAULT;
        goto done;
    }

    ratecfg = (woal_tx_rate_cfg *) respbuf;
    if (rate->param.rate_cfg.is_rate_auto == MTRUE) {
        ratecfg->rate_format = 0xFF;
    } else {
        /* fixed rate */
        if (rate->param.rate_cfg.rate < MLAN_RATE_INDEX_MCS0) {
            ratecfg->rate_format = MLAN_RATE_FORMAT_LG;
            ratecfg->rate_index = rate->param.rate_cfg.rate;
        } else {
            ratecfg->rate_format = MLAN_RATE_FORMAT_HT;
            ratecfg->rate_index =
                rate->param.rate_cfg.rate - MLAN_RATE_INDEX_MCS0;
        }
    }

    ret = sizeof(woal_tx_rate_cfg);

  done:
    if (req)
        kfree(req);
    LEAVE();
    return ret;

}

/**
 *  @brief Set/Get esupplicant mode configurations
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int
woal_setget_priv_esuppmode(moal_private * priv, t_u8 * respbuf,
                           t_u32 respbuflen)
{
    t_u32 data[3];
    mlan_ioctl_req *req = NULL;
    mlan_ds_sec_cfg *sec = NULL;
    woal_esuppmode_cfg *esupp_mode = NULL;
    int ret = 0;
    int user_data_len = 0;

    ENTER();

    if (strlen(respbuf) == (strlen(CMD_MARVELL) + strlen(PRIV_CMD_ESUPPMODE))) {
        /* GET operation */
        user_data_len = 0;
    } else {
        /* SET operation */
        memset((char *) data, 0, sizeof(data));
        parse_arguments(respbuf + strlen(CMD_MARVELL) +
                        strlen(PRIV_CMD_ESUPPMODE), data, &user_data_len);
    }

    if (user_data_len >= 4 || user_data_len == 1 || user_data_len == 2) {
        PRINTM(MERROR, "Invalid number of arguments\n");
        ret = -EINVAL;
        goto done;
    }

    req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_sec_cfg));
    if (req == NULL) {
        ret = -ENOMEM;
        goto done;
    }

    req->req_id = MLAN_IOCTL_SEC_CFG;
    sec = (mlan_ds_sec_cfg *) req->pbuf;
    sec->sub_command = MLAN_OID_SEC_CFG_ESUPP_MODE;

    if (user_data_len == 0) {
        /* Get operation */
        req->action = MLAN_ACT_GET;
    } else {
        /* Set operation */
        req->action = MLAN_ACT_SET;
        /* RSN mode */
        sec->param.esupp_mode.rsn_mode = data[0];
        /* Pairwise cipher */
        sec->param.esupp_mode.act_paircipher = (data[1] & 0xFF);
        /* Group cipher */
        sec->param.esupp_mode.act_groupcipher = (data[2] & 0xFF);
    }

    if (MLAN_STATUS_SUCCESS != woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT)) {
        ret = -EFAULT;
        goto done;
    }

    esupp_mode = (woal_esuppmode_cfg *) respbuf;
    esupp_mode->rsn_mode = (t_u16) ((sec->param.esupp_mode.rsn_mode) & 0xFFFF);
    esupp_mode->pairwise_cipher =
        (t_u8) ((sec->param.esupp_mode.act_paircipher) & 0xFF);
    esupp_mode->group_cipher =
        (t_u8) ((sec->param.esupp_mode.act_groupcipher) & 0xFF);

    ret = sizeof(woal_esuppmode_cfg);
  done:
    if (req)
        kfree(req);
    LEAVE();
    return ret;

}

/**
 *  @brief Set/Get esupplicant passphrase configurations
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int
woal_setget_priv_passphrase(moal_private * priv, t_u8 * respbuf,
                            t_u32 respbuflen)
{
    mlan_ioctl_req *req = NULL;
    mlan_ds_sec_cfg *sec = NULL;
    int ret = 0, action = -1, i = 0;
    char *begin, *end, *opt;
    t_u16 len = 0;
    t_u8 zero_mac[] = { 0, 0, 0, 0, 0, 0 };
    t_u8 *mac = NULL;

    ENTER();

    if (strlen(respbuf) == (strlen(CMD_MARVELL) + strlen(PRIV_CMD_PASSPHRASE))) {
        PRINTM(MERROR, "No arguments provided\n");
        ret = -EINVAL;
        goto done;
    }

    /* Parse the buf to get the cmd_action */
    begin = respbuf + strlen(CMD_MARVELL) + strlen(PRIV_CMD_PASSPHRASE);
    end = woal_strsep(&begin, ';', '/');
    if (end)
        action = woal_atox(end);
    if (action < 0 || action > 2 || end[1] != '\0') {
        PRINTM(MERROR, "Invalid action argument %s\n", end);
        ret = -EINVAL;
        goto done;
    }

    req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_sec_cfg));
    if (req == NULL) {
        ret = -ENOMEM;
        goto done;
    }

    req->req_id = MLAN_IOCTL_SEC_CFG;
    sec = (mlan_ds_sec_cfg *) req->pbuf;
    sec->sub_command = MLAN_OID_SEC_CFG_PASSPHRASE;
    if (action == 0)
        req->action = MLAN_ACT_GET;
    else
        req->action = MLAN_ACT_SET;

    while (begin) {
        end = woal_strsep(&begin, ';', '/');
        opt = woal_strsep(&end, '=', '/');
        if (!opt || !end || !end[0]) {
            PRINTM(MERROR, "Invalid option\n");
            ret = -EINVAL;
            break;
        } else if (!strnicmp(opt, "ssid", strlen(opt))) {
            if (strlen(end) > MLAN_MAX_SSID_LENGTH) {
                PRINTM(MERROR, "SSID length exceeds max length\n");
                ret = -EFAULT;
                break;
            }
            sec->param.passphrase.ssid.ssid_len = strlen(end);
            strncpy((char *) sec->param.passphrase.ssid.ssid, end, strlen(end));
            PRINTM(MINFO, "ssid=%s, len=%d\n", sec->param.passphrase.ssid.ssid,
                   (int) sec->param.passphrase.ssid.ssid_len);
        } else if (!strnicmp(opt, "bssid", strlen(opt))) {
            woal_mac2u8((t_u8 *) & sec->param.passphrase.bssid, end);
        } else if (!strnicmp(opt, "psk", strlen(opt)) &&
                   req->action == MLAN_ACT_SET) {
            if (strlen(end) != MLAN_PMK_HEXSTR_LENGTH) {
                PRINTM(MERROR, "Invalid PMK length\n");
                ret = -EINVAL;
                break;
            }
            woal_ascii2hex((t_u8 *) (sec->param.passphrase.psk.pmk.pmk), end,
                           MLAN_PMK_HEXSTR_LENGTH / 2);
            sec->param.passphrase.psk_type = MLAN_PSK_PMK;
        } else if (!strnicmp(opt, "passphrase", strlen(opt)) &&
                   req->action == MLAN_ACT_SET) {
            if (strlen(end) < MLAN_MIN_PASSPHRASE_LENGTH ||
                strlen(end) > MLAN_MAX_PASSPHRASE_LENGTH) {
                PRINTM(MERROR, "Invalid length for passphrase\n");
                ret = -EINVAL;
                break;
            }
            sec->param.passphrase.psk_type = MLAN_PSK_PASSPHRASE;
            strncpy(sec->param.passphrase.psk.passphrase.passphrase, end,
                    sizeof(sec->param.passphrase.psk.passphrase.passphrase));
            sec->param.passphrase.psk.passphrase.passphrase_len = strlen(end);
            PRINTM(MINFO, "passphrase=%s, len=%d\n",
                   sec->param.passphrase.psk.passphrase.passphrase,
                   (int) sec->param.passphrase.psk.passphrase.passphrase_len);
        } else {
            PRINTM(MERROR, "Invalid option %s\n", opt);
            ret = -EINVAL;
            break;
        }
    }
    if (ret)
        goto done;

    if (action == 2)
        sec->param.passphrase.psk_type = MLAN_PSK_CLEAR;

    if (MLAN_STATUS_SUCCESS != woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT)) {
        ret = -EFAULT;
        goto done;
    }

    memset(respbuf, 0, respbuflen);
    if (sec->param.passphrase.ssid.ssid_len) {
        len += sprintf(respbuf + len, "ssid:");
        memcpy(respbuf + len, sec->param.passphrase.ssid.ssid,
               sec->param.passphrase.ssid.ssid_len);
        len += sec->param.passphrase.ssid.ssid_len;
        len += sprintf(respbuf + len, " ");
    }
    if (memcmp(&sec->param.passphrase.bssid, zero_mac, sizeof(zero_mac))) {
        mac = (t_u8 *) & sec->param.passphrase.bssid;
        len += sprintf(respbuf + len, "bssid:");
        for (i = 0; i < ETH_ALEN - 1; ++i)
            len += sprintf(respbuf + len, "%02x:", mac[i]);
        len += sprintf(respbuf + len, "%02x ", mac[i]);
    }
    if (sec->param.passphrase.psk_type == MLAN_PSK_PMK) {
        len += sprintf(respbuf + len, "psk:");
        for (i = 0; i < MLAN_MAX_KEY_LENGTH; ++i)
            len +=
                sprintf(respbuf + len, "%02x",
                        sec->param.passphrase.psk.pmk.pmk[i]);
        len += sprintf(respbuf + len, "\n");
    }
    if (sec->param.passphrase.psk_type == MLAN_PSK_PASSPHRASE) {
        len +=
            sprintf(respbuf + len, "passphrase:%s \n",
                    sec->param.passphrase.psk.passphrase.passphrase);
    }

    ret = len;
  done:
    if (req)
        kfree(req);
    LEAVE();
    return ret;

}

/**
 *  @brief Deauthenticate
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int
woal_priv_deauth(moal_private * priv, t_u8 * respbuf, t_u32 respbuflen)
{
    mlan_ioctl_req *req = NULL;
    int ret = 0;
    t_u8 mac[ETH_ALEN];

    ENTER();

    if (strlen(respbuf) > (strlen(CMD_MARVELL) + strlen(PRIV_CMD_DEAUTH))) {
        /* Deauth mentioned BSSID */
        woal_mac2u8(mac,
                    respbuf + strlen(CMD_MARVELL) + strlen(PRIV_CMD_DEAUTH));
        if (MLAN_STATUS_SUCCESS != woal_disconnect(priv, MOAL_IOCTL_WAIT, mac)) {
            ret = -EFAULT;
            goto done;
        }
    } else {
        if (MLAN_STATUS_SUCCESS != woal_disconnect(priv, MOAL_IOCTL_WAIT, NULL))
            ret = -EFAULT;
    }

  done:
    if (req)
        kfree(req);
    LEAVE();
    return ret;
}

#if defined(WIFI_DIRECT_SUPPORT)
#if defined(STA_SUPPORT) && defined(UAP_SUPPORT)
/**
 *  @brief Set/Get BSS role
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int
woal_priv_bssrole(moal_private * priv, t_u8 * respbuf, t_u32 respbuflen)
{
    t_u32 data[1];
    int ret = 0;
    int user_data_len = 0;
    t_u8 action = MLAN_ACT_GET;

    ENTER();

    if (strlen(respbuf) == (strlen(CMD_MARVELL) + strlen(PRIV_CMD_BSSROLE))) {
        /* GET operation */
        user_data_len = 0;
    } else {
        /* SET operation */
        memset((char *) data, 0, sizeof(data));
        parse_arguments(respbuf + strlen(CMD_MARVELL) +
                        strlen(PRIV_CMD_BSSROLE), data, &user_data_len);
    }

    if (user_data_len >= 2) {
        PRINTM(MERROR, "Invalid number of arguments\n");
        ret = -EINVAL;
        goto error;
    }

    if (user_data_len == 0) {
        action = MLAN_ACT_GET;
    } else {
        if ((data[0] != MLAN_BSS_ROLE_STA &&
             data[0] != MLAN_BSS_ROLE_UAP) ||
            priv->bss_type != MLAN_BSS_TYPE_WIFIDIRECT) {
            PRINTM(MWARN, "Invalid BSS role\n");
            ret = -EINVAL;
            goto error;
        }
        if (data[0] == GET_BSS_ROLE(priv)) {
            PRINTM(MWARN, "Already BSS is in desired role\n");
            goto done;
        }
        action = MLAN_ACT_SET;
        /* Reset interface */
        woal_reset_intf(priv, MOAL_IOCTL_WAIT, MFALSE);
    }

    if (MLAN_STATUS_SUCCESS != woal_bss_role_cfg(priv,
                                                 action, MOAL_IOCTL_WAIT,
                                                 (t_u8 *) data)) {
        ret = -EFAULT;
        goto error;
    }

    if (user_data_len) {
        /* Initialize private structures */
        woal_init_priv(priv, MOAL_IOCTL_WAIT);
        /* Enable interfaces */
        netif_device_attach(priv->netdev);
        woal_start_queue(priv->netdev);
    }

  done:
    memset(respbuf, 0, respbuflen);
    respbuf[0] = (t_u8) data[0];
    ret = 1;

  error:
    LEAVE();
    return ret;
}
#endif /* STA_SUPPORT && UAP_SUPPORT */
#endif /* WIFI_DIRECT_SUPPORT && V14_FEATURE */

#ifdef STA_SUPPORT
/**
 *  @brief Set user scan
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int
woal_priv_setuserscan(moal_private * priv, t_u8 * respbuf, t_u32 respbuflen)
{
    wlan_user_scan_cfg scan_cfg;
    int ret = 0;

    ENTER();

    /* Create the scan_cfg structure */
    memset(&scan_cfg, 0, sizeof(scan_cfg));

    /* We expect the scan_cfg structure to be passed in respbuf */
    memcpy((char *) &scan_cfg,
           respbuf + strlen(CMD_MARVELL) + strlen(PRIV_CMD_SETUSERSCAN),
           sizeof(wlan_user_scan_cfg));

    /* Call for scan */
    if (MLAN_STATUS_FAILURE == woal_do_scan(priv, &scan_cfg))
        ret = -EFAULT;

    LEAVE();
    return ret;
}
#endif

/**
 *  @brief Set/Get deep sleep mode configurations
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int
woal_priv_setgetdeepsleep(moal_private * priv, t_u8 * respbuf, t_u32 respbuflen)
{
    t_u32 data[2];
    int ret = 0;
    int user_data_len = 0;

    ENTER();

    if (strlen(respbuf) == (strlen(CMD_MARVELL) + strlen(PRIV_CMD_DEEPSLEEP))) {
        /* GET operation */
        user_data_len = 0;
    } else {
        /* SET operation */
        memset((char *) data, 0, sizeof(data));
        parse_arguments(respbuf + strlen(CMD_MARVELL) +
                        strlen(PRIV_CMD_DEEPSLEEP), data, &user_data_len);
    }

    if (user_data_len >= 3) {
        PRINTM(MERROR, "Invalid number of arguments\n");
        ret = -EINVAL;
        goto done;
    }

    if (user_data_len == 0) {
        if (MLAN_STATUS_SUCCESS != woal_get_deep_sleep(priv, data)) {
            ret = -EFAULT;
            goto done;
        }
        sprintf(respbuf, "%d %d", data[0], data[1]);
        ret = strlen(respbuf) + 1;
    } else {
        if (data[0] == DEEP_SLEEP_OFF) {
            PRINTM(MINFO, "Exit Deep Sleep Mode\n");
            ret = woal_set_deep_sleep(priv, MOAL_IOCTL_WAIT, MFALSE, 0);
            if (ret != MLAN_STATUS_SUCCESS) {
                ret = -EINVAL;
                goto done;
            }
        } else if (data[0] == DEEP_SLEEP_ON) {
            PRINTM(MINFO, "Enter Deep Sleep Mode\n");
            if (user_data_len != 2)
                data[1] = 0;
            ret = woal_set_deep_sleep(priv, MOAL_IOCTL_WAIT, MTRUE, data[1]);
            if (ret != MLAN_STATUS_SUCCESS) {
                ret = -EINVAL;
                goto done;
            }
        } else {
            PRINTM(MERROR, "Unknown option = %u\n", data[0]);
            ret = -EINVAL;
            goto done;
        }
        ret = sprintf(respbuf, "OK\n") + 1;
    }

  done:
    LEAVE();
    return ret;

}

/**
 *  @brief Set/Get IP address configurations
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int
woal_priv_setgetipaddr(moal_private * priv, t_u8 * respbuf, t_u32 respbuflen)
{
    mlan_ioctl_req *req = NULL;
    mlan_ds_misc_cfg *misc = NULL;
    int ret = 0, op_code = 0, data_length = 0, header = 0;

    ENTER();

    header = strlen(CMD_MARVELL) + strlen(PRIV_CMD_IPADDR);
    data_length = strlen(respbuf) - header;

    req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
    if (req == NULL) {
        ret = -ENOMEM;
        goto done;
    }
    misc = (mlan_ds_misc_cfg *) req->pbuf;

    if (data_length < 1) {      /* GET */
        req->action = MLAN_ACT_GET;
    } else {
        /* Make sure we have the operation argument */
        if (data_length > 2 && respbuf[header + 1] != ';') {
            PRINTM(MERROR, "No operation argument. Separate with ';'\n");
            ret = -EINVAL;
            goto done;
        } else {
            respbuf[header + 1] = '\0';
        }
        req->action = MLAN_ACT_SET;

        /* Only one IP is supported in current firmware */
        memset(misc->param.ipaddr_cfg.ip_addr[0], 0, IPADDR_LEN);
        in4_pton(&respbuf[header + 2],
                 MIN((IPADDR_MAX_BUF - 3), (data_length - 2)),
                 misc->param.ipaddr_cfg.ip_addr[0], ' ', NULL);
        misc->param.ipaddr_cfg.ip_addr_num = 1;
        misc->param.ipaddr_cfg.ip_addr_type = IPADDR_TYPE_IPV4;

        if (woal_atoi(&op_code, &respbuf[header]) != MLAN_STATUS_SUCCESS) {
            ret = -EINVAL;
            goto done;
        }
        misc->param.ipaddr_cfg.op_code = (t_u32) op_code;
    }

    req->req_id = MLAN_IOCTL_MISC_CFG;
    misc->sub_command = MLAN_OID_MISC_IP_ADDR;

    if (MLAN_STATUS_SUCCESS != woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT)) {
        ret = -EFAULT;
        goto done;
    }

    if (req->action == MLAN_ACT_GET) {
        snprintf(respbuf, IPADDR_MAX_BUF, "%d;%d.%d.%d.%d",
                 misc->param.ipaddr_cfg.op_code,
                 misc->param.ipaddr_cfg.ip_addr[0][0],
                 misc->param.ipaddr_cfg.ip_addr[0][1],
                 misc->param.ipaddr_cfg.ip_addr[0][2],
                 misc->param.ipaddr_cfg.ip_addr[0][3]);
        ret = IPADDR_MAX_BUF + 1;
    } else {
        ret = sprintf(respbuf, "OK\n") + 1;
    }

  done:
    if (req)
        kfree(req);
    LEAVE();
    return ret;
}

/**
 *  @brief Set/Get WPS session configurations
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int
woal_priv_setwpssession(moal_private * priv, t_u8 * respbuf, t_u32 respbuflen)
{
    mlan_ioctl_req *req = NULL;
    mlan_ds_wps_cfg *pwps = NULL;
    t_u32 data[1];
    int ret = 0;
    int user_data_len = 0;

    ENTER();

    if (strlen(respbuf) == (strlen(CMD_MARVELL) + strlen(PRIV_CMD_WPSSESSION))) {
        /* GET operation */
        user_data_len = 0;
    } else {
        /* SET operation */
        memset((char *) data, 0, sizeof(data));
        parse_arguments(respbuf + strlen(CMD_MARVELL) +
                        strlen(PRIV_CMD_WPSSESSION), data, &user_data_len);
    }

    if (user_data_len != 1) {
        PRINTM(MERROR, "Invalid number of arguments\n");
        ret = -EINVAL;
        goto done;
    }

    req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_wps_cfg));
    if (req == NULL) {
        ret = -ENOMEM;
        goto done;
    }
    pwps = (mlan_ds_wps_cfg *) req->pbuf;

    req->req_id = MLAN_IOCTL_WPS_CFG;
    req->action = MLAN_ACT_SET;
    pwps->sub_command = MLAN_OID_WPS_CFG_SESSION;

    if (data[0] == 1)
        pwps->param.wps_session = MLAN_WPS_CFG_SESSION_START;
    else
        pwps->param.wps_session = MLAN_WPS_CFG_SESSION_END;

    if (MLAN_STATUS_SUCCESS != woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT)) {
        ret = -EFAULT;
        goto done;
    }

    ret = sprintf(respbuf, "OK\n") + 1;
  done:
    if (req)
        kfree(req);
    LEAVE();
    return ret;
}

/**
 *  @brief Get OTP user data
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int
woal_priv_otpuserdata(moal_private * priv, t_u8 * respbuf, t_u32 respbuflen)
{
    int data[1];
    int user_data_len = 0;
    mlan_ioctl_req *req = NULL;
    mlan_ds_misc_cfg *misc = NULL;
    mlan_ds_misc_otp_user_data *otp = NULL;
    int ret = 0;

    ENTER();

    memset((char *) data, 0, sizeof(data));
    parse_arguments(respbuf + strlen(CMD_MARVELL) +
                    strlen(PRIV_CMD_OTPUSERDATA), data, &user_data_len);

    if (user_data_len > 1) {
        PRINTM(MERROR, "Too many arguments\n");
        LEAVE();
        return -EINVAL;
    }

    req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
    if (req == NULL) {
        ret = -ENOMEM;
        goto done;
    }
    req->action = MLAN_ACT_GET;
    req->req_id = MLAN_IOCTL_MISC_CFG;

    misc = (mlan_ds_misc_cfg *) req->pbuf;
    misc->sub_command = MLAN_OID_MISC_OTP_USER_DATA;
    misc->param.otp_user_data.user_data_length = data[0];

    if (MLAN_STATUS_SUCCESS != woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT)) {
        ret = -EFAULT;
        goto done;
    }
    otp = (mlan_ds_misc_otp_user_data *) req->pbuf;

    if (req->action == MLAN_ACT_GET) {
        ret = MIN(otp->user_data_length, data[0]);
        memcpy(respbuf, otp->user_data, ret);
    }

  done:
    if (req)
        kfree(req);
    LEAVE();
    return ret;
}

/**
 *  @brief Set / Get country code
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int
woal_priv_set_get_countrycode(moal_private * priv, t_u8 * respbuf,
                              t_u32 respbuflen)
{
    int ret = 0;
    // char data[COUNTRY_CODE_LEN] = {0, 0, 0};
    int header = 0, data_length = 0;    // wrq->u.data.length;
    mlan_ioctl_req *req = NULL;
    mlan_ds_misc_cfg *pcfg_misc = NULL;
    mlan_ds_misc_country_code *country_code = NULL;

    ENTER();

    header = strlen(CMD_MARVELL) + strlen(PRIV_CMD_COUNTRYCODE);
    data_length = strlen(respbuf) - header;

    if (data_length > COUNTRY_CODE_LEN) {
        PRINTM(MERROR, "Invalid argument!\n");
        ret = -EINVAL;
        goto done;
    }

    /* Allocate an IOCTL request buffer */
    req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
    if (req == NULL) {
        ret = -ENOMEM;
        goto done;
    }

    /* Fill request buffer */
    pcfg_misc = (mlan_ds_misc_cfg *) req->pbuf;
    country_code = &pcfg_misc->param.country_code;
    pcfg_misc->sub_command = MLAN_OID_MISC_COUNTRY_CODE;
    req->req_id = MLAN_IOCTL_MISC_CFG;

    if (data_length <= 1) {
        req->action = MLAN_ACT_GET;
    } else {
        memset(country_code->country_code, 0, COUNTRY_CODE_LEN);
        memcpy(country_code->country_code, respbuf + header, COUNTRY_CODE_LEN);
        req->action = MLAN_ACT_SET;
    }

    /* Send IOCTL request to MLAN */
    if (woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT) != MLAN_STATUS_SUCCESS) {
        ret = -EFAULT;
        goto done;
    }

    if (req->action == MLAN_ACT_GET) {
        ret = data_length = COUNTRY_CODE_LEN;
        memset(respbuf + header, 0, COUNTRY_CODE_LEN);
        memcpy(respbuf, country_code->country_code, COUNTRY_CODE_LEN);
    }

  done:
    if (req)
        kfree(req);

    LEAVE();
    return ret;
}

/**
 *  @brief Set/Get TCP Ack enhancement configurations
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int
woal_priv_setgettcpackenh(moal_private * priv, t_u8 * respbuf, t_u32 respbuflen)
{
    t_u32 data[1];
    int ret = 0;
    int user_data_len = 0;
    struct list_head *link = NULL;
    struct tcp_sess *tcp_sess = NULL;

    ENTER();

    if (strlen(respbuf) == (strlen(CMD_MARVELL) + strlen(PRIV_CMD_TCPACKENH))) {
        /* GET operation */
        user_data_len = 0;
    } else {
        /* SET operation */
        memset((char *) data, 0, sizeof(data));
        parse_arguments(respbuf + strlen(CMD_MARVELL) +
                        strlen(PRIV_CMD_TCPACKENH), data, &user_data_len);
    }

    if (user_data_len >= 2) {
        PRINTM(MERROR, "Invalid number of arguments\n");
        ret = -EINVAL;
        goto done;
    }

    if (user_data_len == 0) {
        /* get operation */
        respbuf[0] = priv->enable_tcp_ack_enh;
    } else {
        /* set operation */
        if (data[0] == MTRUE) {
            PRINTM(MINFO, "Enabling TCP Ack enhancement\n");
            priv->enable_tcp_ack_enh = MTRUE;
        } else if (data[0] == MFALSE) {
            PRINTM(MINFO, "Disabling TCP Ack enhancement\n");
            priv->enable_tcp_ack_enh = MFALSE;
            /* release the tcp sessions if any */
            while (!list_empty(&priv->tcp_sess_queue)) {
                link = priv->tcp_sess_queue.next;
                tcp_sess = list_entry(link, struct tcp_sess, link);
                PRINTM(MINFO,
                       "Disable TCP ACK Enh: release a tcp session in dl. (src: ipaddr 0x%08x, port: %d. dst: ipaddr 0x%08x, port: %d\n",
                       tcp_sess->src_ip_addr, tcp_sess->src_tcp_port,
                       tcp_sess->dst_ip_addr, tcp_sess->dst_tcp_port);
                list_del(link);
                kfree(tcp_sess);
            }
        } else {
            PRINTM(MERROR, "Unknown option = %u\n", data[0]);
            ret = -EINVAL;
            goto done;
        }
        respbuf[0] = priv->enable_tcp_ack_enh;
    }
    ret = 1;

  done:
    LEAVE();
    return ret;

}

/**
 *  @brief Set priv command for Android
 *  @param dev          A pointer to net_device structure
 *  @param req          A pointer to ifreq structure
 *
 *  @return             0 --success, otherwise fail
 */
int
woal_android_priv_cmd(struct net_device *dev, struct ifreq *req)
{
    int ret = 0;
    android_wifi_priv_cmd priv_cmd;
    moal_private *priv = (moal_private *) netdev_priv(dev);
    char *buf = NULL;
    char *pdata;
#ifdef STA_SUPPORT
    int power_mode = 0;
    int band = 0;
    char *pband = NULL;
    mlan_bss_info bss_info;
    mlan_ds_get_signal signal;
    mlan_rate_cfg_t rate;
    t_u8 country_code[COUNTRY_CODE_LEN];
#endif
    int len = 0;

    ENTER();
    if (copy_from_user(&priv_cmd, req->ifr_data, sizeof(android_wifi_priv_cmd))) {
        ret = -EFAULT;
        goto done;
    }
    buf = kzalloc(priv_cmd.total_len, GFP_KERNEL);
    if (!buf) {
        PRINTM(MERROR, "%s: failed to allocate memory\n", __FUNCTION__);
        ret = -ENOMEM;
        goto done;
    }
    if (copy_from_user(buf, priv_cmd.buf, priv_cmd.total_len)) {
        ret = -EFAULT;
        goto done;
    }

    PRINTM(MIOCTL, "Android priv cmd: [%s] on [%s]\n", buf, req->ifr_name);

    if (strncmp(buf, CMD_MARVELL, strlen(CMD_MARVELL)) == 0) {
        /* This command has come from mlanutl app */

        /* Check command */
        if (strnicmp
            (buf + strlen(CMD_MARVELL), PRIV_CMD_VERSION,
             strlen(PRIV_CMD_VERSION)) == 0) {
            /* Get version */
            len = woal_get_priv_driver_version(priv, buf, priv_cmd.total_len);
            goto handled;
        } else
            if (strnicmp
                (buf + strlen(CMD_MARVELL), PRIV_CMD_BANDCFG,
                 strlen(PRIV_CMD_BANDCFG)) == 0) {
            /* Set/Get band configuration */
            len = woal_setget_priv_bandcfg(priv, buf, priv_cmd.total_len);
            goto handled;
        } else
            if (strnicmp
                (buf + strlen(CMD_MARVELL), PRIV_CMD_HOSTCMD,
                 strlen(PRIV_CMD_HOSTCMD)) == 0) {
            /* hostcmd configuration */
            len = woal_priv_hostcmd(priv, buf, priv_cmd.total_len);
            goto handled;
        } else
            if (strnicmp
                (buf + strlen(CMD_MARVELL), PRIV_CMD_HTTXCFG,
                 strlen(PRIV_CMD_HTTXCFG)) == 0) {
            /* Set/Get HT Tx configuration */
            len = woal_setget_priv_httxcfg(priv, buf, priv_cmd.total_len);
            goto handled;
        } else
            if (strnicmp
                (buf + strlen(CMD_MARVELL), PRIV_CMD_DATARATE,
                 strlen(PRIV_CMD_DATARATE)) == 0) {
            /* Get data rate */
            len = woal_get_priv_datarate(priv, buf, priv_cmd.total_len);
            goto handled;
        } else
            if (strnicmp
                (buf + strlen(CMD_MARVELL), PRIV_CMD_TXRATECFG,
                 strlen(PRIV_CMD_TXRATECFG)) == 0) {
            /* Set/Get tx rate cfg */
            len = woal_setget_priv_txratecfg(priv, buf, priv_cmd.total_len);
            goto handled;
        } else
            if (strnicmp
                (buf + strlen(CMD_MARVELL), PRIV_CMD_CUSTOMIE,
                 strlen(PRIV_CMD_CUSTOMIE)) == 0) {
            /* Custom IE configuration */
            len = woal_priv_customie(priv, buf, priv_cmd.total_len);
            goto handled;
        } else
            if (strnicmp
                (buf + strlen(CMD_MARVELL), PRIV_CMD_ESUPPMODE,
                 strlen(PRIV_CMD_ESUPPMODE)) == 0) {
            /* Esupplicant mode configuration */
            len = woal_setget_priv_esuppmode(priv, buf, priv_cmd.total_len);
            goto handled;
        } else
            if (strnicmp
                (buf + strlen(CMD_MARVELL), PRIV_CMD_PASSPHRASE,
                 strlen(PRIV_CMD_PASSPHRASE)) == 0) {
            /* Esupplicant passphrase configuration */
            len = woal_setget_priv_passphrase(priv, buf, priv_cmd.total_len);
            goto handled;
        } else
            if (strnicmp
                (buf + strlen(CMD_MARVELL), PRIV_CMD_DEAUTH,
                 strlen(PRIV_CMD_DEAUTH)) == 0) {
            /* Deauth */
            len = woal_priv_deauth(priv, buf, priv_cmd.total_len);
            goto handled;
#if defined(WIFI_DIRECT_SUPPORT)
#if defined(STA_SUPPORT) && defined(UAP_SUPPORT)
        } else
            if (strnicmp
                (buf + strlen(CMD_MARVELL), PRIV_CMD_BSSROLE,
                 strlen(PRIV_CMD_BSSROLE)) == 0) {
            /* BSS Role */
            len = woal_priv_bssrole(priv, buf, priv_cmd.total_len);
            goto handled;
#endif
#endif
#ifdef STA_SUPPORT
        } else
            if (strnicmp
                (buf + strlen(CMD_MARVELL), PRIV_CMD_SETUSERSCAN,
                 strlen(PRIV_CMD_SETUSERSCAN)) == 0) {
            /* Set user scan */
            len = woal_priv_setuserscan(priv, buf, priv_cmd.total_len);
            goto handled;
#endif
        } else
            if (strnicmp
                (buf + strlen(CMD_MARVELL), PRIV_CMD_DEEPSLEEP,
                 strlen(PRIV_CMD_DEEPSLEEP)) == 0) {
            /* Deep sleep */
            len = woal_priv_setgetdeepsleep(priv, buf, priv_cmd.total_len);
            goto handled;
        } else
            if (strnicmp
                (buf + strlen(CMD_MARVELL), PRIV_CMD_IPADDR,
                 strlen(PRIV_CMD_IPADDR)) == 0) {
            /* IP address */
            len = woal_priv_setgetipaddr(priv, buf, priv_cmd.total_len);
            goto handled;
        } else
            if (strnicmp
                (buf + strlen(CMD_MARVELL), PRIV_CMD_WPSSESSION,
                 strlen(PRIV_CMD_WPSSESSION)) == 0) {
            /* WPS Session */
            len = woal_priv_setwpssession(priv, buf, priv_cmd.total_len);
            goto handled;
        } else
            if (strnicmp
                (buf + strlen(CMD_MARVELL), PRIV_CMD_OTPUSERDATA,
                 strlen(PRIV_CMD_OTPUSERDATA)) == 0) {
            /* OTP user data */
            len = woal_priv_otpuserdata(priv, buf, priv_cmd.total_len);
            goto handled;
        } else
            if (strnicmp
                (buf + strlen(CMD_MARVELL), PRIV_CMD_COUNTRYCODE,
                 strlen(PRIV_CMD_COUNTRYCODE)) == 0) {
            /* OTP user data */
            len = woal_priv_set_get_countrycode(priv, buf, priv_cmd.total_len);
            goto handled;
        } else
            if (strnicmp
                (buf + strlen(CMD_MARVELL), PRIV_CMD_TCPACKENH,
                 strlen(PRIV_CMD_TCPACKENH)) == 0) {
            /* OTP user data */
            len = woal_priv_setgettcpackenh(priv, buf, priv_cmd.total_len);
            goto handled;
        } else {
            /* Fall through, after stripping off the custom header */
            buf += strlen(CMD_MARVELL);
        }
    }
#ifdef STA_SUPPORT
    if (strncmp(buf, "RSSILOW-THRESHOLD", strlen("RSSILOW-THRESHOLD")) == 0) {
        pdata = buf + strlen("RSSILOW-THRESHOLD") + 1;
        if (MLAN_STATUS_SUCCESS != woal_set_rssi_low_threshold(priv, pdata)) {
            ret = -EFAULT;
            goto done;
        }
        len = sprintf(buf, "OK\n") + 1;
    } else if (strncmp(buf, "RSSI", strlen("RSSI")) == 0) {
        if (MLAN_STATUS_SUCCESS != woal_get_bss_info(priv,
                                                     MOAL_IOCTL_WAIT,
                                                     &bss_info)) {
            ret = -EFAULT;
            goto done;
        }
        if (bss_info.media_connected) {
            if (MLAN_STATUS_SUCCESS != woal_get_signal_info(priv,
                                                            MOAL_IOCTL_WAIT,
                                                            &signal)) {
                ret = -EFAULT;
                goto done;
            }
            len = sprintf(buf, "%s rssi %d\n", bss_info.ssid.ssid,
                          signal.bcn_rssi_avg) + 1;
        } else {
            len = sprintf(buf, "OK\n") + 1;
        }
    } else if (strncmp(buf, "LINKSPEED", strlen("LINKSPEED")) == 0) {
        if (MLAN_STATUS_SUCCESS != woal_set_get_data_rate(priv, MLAN_ACT_GET,
                                                          &rate)) {
            ret = -EFAULT;
            goto done;
        }
        PRINTM(MIOCTL, "tx rate=%d\n", (int) rate.rate);
        len =
            sprintf(buf, "LinkSpeed %d\n", (int) (rate.rate * 500000 / 1000000))
            + 1;
    } else
#endif
    if (strncmp(buf, "MACADDR", strlen("MACADDR")) == 0) {
        len = sprintf(buf, "Macaddr = %02X:%02X:%02X:%02X:%02X:%02X\n",
                      priv->current_addr[0], priv->current_addr[1],
                      priv->current_addr[2], priv->current_addr[3],
                      priv->current_addr[4], priv->current_addr[5]) + 1;
    }
#ifdef STA_SUPPORT
    else if (strncmp(buf, "GETPOWER", strlen("GETPOWER")) == 0) {
        if (MLAN_STATUS_SUCCESS != woal_get_powermode(priv, &power_mode)) {
            ret = -EFAULT;
            goto done;
        }
        len = sprintf(buf, "powermode = %d\n", power_mode) + 1;
    } else if (strncmp(buf, "SCAN-ACTIVE", strlen("SCAN-ACTIVE")) == 0) {
        if (MLAN_STATUS_SUCCESS != woal_set_scan_type(priv,
                                                      MLAN_SCAN_TYPE_ACTIVE)) {
            ret = -EFAULT;
            goto done;
        }
        priv->scan_type = MLAN_SCAN_TYPE_ACTIVE;
        PRINTM(MIOCTL, "Set Active Scan\n");
        len = sprintf(buf, "OK\n") + 1;
    } else if (strncmp(buf, "SCAN-PASSIVE", strlen("SCAN-PASSIVE")) == 0) {
        if (MLAN_STATUS_SUCCESS != woal_set_scan_type(priv,
                                                      MLAN_SCAN_TYPE_PASSIVE)) {
            ret = -EFAULT;
            goto done;
        }
        priv->scan_type = MLAN_SCAN_TYPE_PASSIVE;
        PRINTM(MIOCTL, "Set Passive Scan\n");
        len = sprintf(buf, "OK\n") + 1;
    } else if (strncmp(buf, "POWERMODE", strlen("POWERMODE")) == 0) {
        pdata = buf + strlen("POWERMODE") + 1;
        if (MLAN_STATUS_SUCCESS != woal_set_powermode(priv, pdata)) {
            ret = -EFAULT;
            goto done;
        }
        len = sprintf(buf, "OK\n") + 1;
    } else if (strncmp(buf, "COUNTRY", strlen("COUNTRY")) == 0) {
        memset(country_code, 0, sizeof(country_code));
        memcpy(country_code, buf + strlen("COUNTRY") + 1,
               strlen(buf) - strlen("COUNTRY") - 1);
        PRINTM(MIOCTL, "Set COUNTRY %s\n", country_code);
        if (MLAN_STATUS_SUCCESS != woal_set_region_code(priv, country_code)) {
            ret = -EFAULT;
            goto done;
        }
        len = sprintf(buf, "OK\n") + 1;
    } else if (memcmp(buf, WEXT_CSCAN_HEADER, WEXT_CSCAN_HEADER_SIZE) == 0) {
        PRINTM(MIOCTL, "Set Combo Scan\n");
        if (MLAN_STATUS_SUCCESS != woal_set_combo_scan(priv, buf,
                                                       priv_cmd.total_len)) {
            ret = -EFAULT;
            goto done;
        }
        len = sprintf(buf, "OK\n") + 1;
    } else if (strncmp(buf, "GETBAND", strlen("GETBAND")) == 0) {
        if (MLAN_STATUS_SUCCESS != woal_get_band(priv, &band)) {
            ret = -EFAULT;
            goto done;
        }
        len = sprintf(buf, "Band %d\n", band) + 1;
    } else if (strncmp(buf, "SETBAND", strlen("SETBAND")) == 0) {
        pband = buf + strlen("SETBAND") + 1;
        if (MLAN_STATUS_SUCCESS != woal_set_band(priv, pband)) {
            ret = -EFAULT;
            goto done;
        }
        len = sprintf(buf, "OK\n") + 1;
    }
#endif
    else if (strncmp(buf, "START", strlen("START")) == 0) {
        len = sprintf(buf, "OK\n") + 1;
    } else if (strncmp(buf, "STOP", strlen("STOP")) == 0) {
        len = sprintf(buf, "OK\n") + 1;
    }
#ifdef UAP_SUPPORT
    else if (strncmp(buf, "AP_BSS_START", strlen("AP_BSS_START")) == 0) {
        if ((ret == woal_uap_bss_ctrl(priv, MOAL_IOCTL_WAIT, UAP_BSS_START)))
            goto done;
        len = sprintf(buf, "OK\n") + 1;
    } else if (strncmp(buf, "AP_BSS_STOP", strlen("AP_BSS_STOP")) == 0) {
        if ((ret == woal_uap_bss_ctrl(priv, MOAL_IOCTL_WAIT, UAP_BSS_STOP)))
            goto done;
        len = sprintf(buf, "OK\n") + 1;
    } else if (strncmp(buf, "AP_SET_CFG", strlen("AP_SET_CFG")) == 0) {
        pdata = buf + strlen("AP_SET_CFG") + 1;
        if ((ret =
             woal_uap_set_ap_cfg(priv, pdata,
                                 priv_cmd.used_len - strlen("AP_SET_CFG") - 1)))
            goto done;
        len = sprintf(buf, "OK\n") + 1;
    } else if (strncmp(buf, "WL_FW_RELOAD", strlen("WL_FW_RELOAD")) == 0) {
        len = sprintf(buf, "OK\n") + 1;
    } else if (strncmp(buf, "AP_GET_STA_LIST", strlen("AP_GET_STA_LIST")) == 0) {
        // TODO Add STA list support
        len = sprintf(buf, "OK\n") + 1;
    }
#endif
    else if (strncmp(buf, "SETSUSPENDOPT", strlen("SETSUSPENDOPT")) == 0) {
        /* it will be done by GUI */
        len = sprintf(buf, "OK\n") + 1;
    } else if (strncmp(buf, "BTCOEXMODE", strlen("BTCOEXMODE")) == 0) {
        len = sprintf(buf, "OK\n") + 1;
    } else if (strncmp(buf, "BTCOEXSCAN-START", strlen("BTCOEXSCAN-START")) ==
               0) {
        len = sprintf(buf, "OK\n") + 1;
    } else if (strncmp(buf, "BTCOEXSCAN-STOP", strlen("BTCOEXSCAN-STOP")) == 0) {
        len = sprintf(buf, "OK\n") + 1;
    } else if (strncmp(buf, "BTCOEXSCAN-START", strlen("BTCOEXSCAN-START")) ==
               0) {
        len = sprintf(buf, "OK\n") + 1;
    } else if (strncmp(buf, "BTCOEXSCAN-STOP", strlen("BTCOEXSCAN-STOP")) == 0) {
        len = sprintf(buf, "OK\n") + 1;
    }
#ifdef STA_SUPPORT
    else if (strncmp(buf, "BGSCAN-START", strlen("BGSCAN-START")) == 0) {
        len = sprintf(buf, "OK\n") + 1;
    } else if (strncmp(buf, "BGSCAN-CONFIG", strlen("BGSCAN-CONFIG")) == 0) {
        if (MLAN_STATUS_SUCCESS !=
            woal_set_bg_scan(priv, buf, priv_cmd.total_len)) {
            ret = -EFAULT;
            goto done;
        }
        priv->bg_scan_start = MTRUE;
        priv->bg_scan_reported = MFALSE;
        len = sprintf(buf, "OK\n") + 1;
    } else if (strncmp(buf, "BGSCAN-STOP", strlen("BGSCAN-STOP")) == 0) {
        if (priv->bg_scan_start && !priv->scan_cfg.rssi_threshold) {
            if (MLAN_STATUS_SUCCESS != woal_stop_bg_scan(priv)) {
                ret = -EFAULT;
                goto done;
            }
            priv->bg_scan_start = MFALSE;
            priv->bg_scan_reported = MFALSE;
        }
        len = sprintf(buf, "OK\n") + 1;
    } else if (strncmp(buf, "RXFILTER-START", strlen("RXFILTER-START")) == 0) {
#ifdef MEF_CFG_RX_FILTER
        if ((ret = woal_set_rxfilter(priv, MTRUE))) {
            goto done;
        }
#endif
        len = sprintf(buf, "OK\n") + 1;
    } else if (strncmp(buf, "RXFILTER-STOP", strlen("RXFILTER-STOP")) == 0) {
#ifdef MEF_CFG_RX_FILTER
        if ((ret = woal_set_rxfilter(priv, MFALSE))) {
            goto done;
        }
#endif
        len = sprintf(buf, "OK\n") + 1;
    }
#ifdef STA_CFG80211
    else if (strncmp(buf, "GET_RSSI_STATUS", strlen("GET_RSSI_STATUS")) == 0) {
        if (priv->rssi_status == MLAN_EVENT_ID_FW_BCN_RSSI_LOW)
            len = sprintf(buf, "EVENT=BEACON_RSSI_LOW\n");
        else if (priv->rssi_status == MLAN_EVENT_ID_FW_PRE_BCN_LOST)
            len = sprintf(buf, "EVENT=PRE_BEACON_LOST\n");
        else
            len = sprintf(buf, "OK\n") + 1;
        priv->rssi_status = 0;
    }
#endif
    else if (strncmp(buf, "RXFILTER-ADD", strlen("RXFILTER-ADD")) == 0) {
        pdata = buf + strlen("RXFILTER-ADD") + 1;
        if (MLAN_STATUS_SUCCESS != woal_add_rxfilter(priv, pdata)) {
            ret = -EFAULT;
            goto done;
        }
        len = sprintf(buf, "OK\n") + 1;
    } else if (strncmp(buf, "RXFILTER-REMOVE", strlen("RXFILTER-REMOVE")) == 0) {
        pdata = buf + strlen("RXFILTER-REMOVE") + 1;
        if (MLAN_STATUS_SUCCESS != woal_remove_rxfilter(priv, pdata)) {
            ret = -EFAULT;
            goto done;
        }
        len = sprintf(buf, "OK\n") + 1;
    } else if (strncmp(buf, "QOSINFO", strlen("QOSINFO")) == 0) {
        pdata = buf + strlen("QOSINFO") + 1;
        if (MLAN_STATUS_SUCCESS != woal_set_qos_cfg(priv, pdata)) {
            ret = -EFAULT;
            goto done;
        }
        len = sprintf(buf, "OK\n") + 1;
    } else if (strncmp(buf, "SLEEPPD", strlen("SLEEPPD")) == 0) {
        pdata = buf + strlen("SLEEPPD") + 1;
        if (MLAN_STATUS_SUCCESS != woal_set_sleeppd(priv, pdata)) {
            ret = -EFAULT;
            goto done;
        }
        len = sprintf(buf, "OK\n") + 1;
    } else if (strncmp(buf, "SET_AP_WPS_P2P_IE",
                       strlen("SET_AP_WPS_P2P_IE")) == 0) {
        pdata = buf + strlen("SET_AP_WPS_P2P_IE") + 1;
        /* Android cmd format: "SET_AP_WPS_P2P_IE 1" -- beacon IE
           "SET_AP_WPS_P2P_IE 2" -- proberesp IE "SET_AP_WPS_P2P_IE 4" --
           assocresp IE */
#if defined(STA_CFG80211) && defined(UAP_CFG80211)
        if (MLAN_STATUS_SUCCESS != woal_set_ap_wps_p2p_ie(priv, (t_u8 *) pdata,
                                                          priv_cmd.used_len -
                                                          strlen
                                                          ("SET_AP_WPS_P2P_IE")
                                                          - 1)) {
            ret = -EFAULT;
            goto done;
        }
#endif
        len = sprintf(buf, "OK\n") + 1;
    }
#endif
    else if (strncmp(buf, "P2P_DEV_ADDR", strlen("P2P_DEV_ADDR")) == 0) {
        memset(buf, 0x0, priv_cmd.total_len);
        memcpy(buf, priv->current_addr, ETH_ALEN);
        len = ETH_ALEN;
    } else if (strncmp(buf, ("P2P_GET_NOA"), strlen("P2P_GET_NOA")) == 0) {
        /* TODO Just return '\0' */
        memset(buf, 0x0, priv_cmd.total_len);
        *buf = 0;
        len = 1;
    } else {
        PRINTM(MIOCTL, "Unknow PRIVATE command: %s, ignored\n", buf);
        ret = -EFAULT;
        goto done;
    }

  handled:
    PRINTM(MIOCTL, "PRIV Command return: %s, length=%d\n", buf, len);

    if (len > 0) {
        priv_cmd.used_len = len;
        if (priv_cmd.used_len < priv_cmd.total_len)
            memset(priv_cmd.buf + priv_cmd.used_len, 0,
                   priv_cmd.total_len - priv_cmd.used_len);
        if (copy_to_user(priv_cmd.buf, buf, priv_cmd.used_len)) {
            PRINTM(MERROR, "%s: failed to copy data to user buffer\n",
                   __FUNCTION__);
            ret = -EFAULT;
            goto done;
        }
        if (copy_to_user
            (req->ifr_data, &priv_cmd, sizeof(android_wifi_priv_cmd))) {
            PRINTM(MERROR, "%s: failed to copy command header to user buffer\n",
                   __FUNCTION__);
            ret = -EFAULT;
        }
    } else {
        ret = len;
    }

  done:
    if (buf)
        kfree(buf);
    LEAVE();
    return ret;
}

/********************************************************
		Global Functions
********************************************************/
/**
 *  @brief ioctl function - entry point
 *
 *  @param dev		A pointer to net_device structure
 *  @param req	   	A pointer to ifreq structure
 *  @param cmd 		Command
 *
 *  @return          0 --success, otherwise fail
 */
int
woal_do_ioctl(struct net_device *dev, struct ifreq *req, int cmd)
{
    int ret = 0;

    ENTER();

    PRINTM(MINFO, "woal_do_ioctl: ioctl cmd = 0x%x\n", cmd);
    switch (cmd) {
#ifdef WIFI_DIRECT_SUPPORT
    case WOAL_WIFIDIRECT_HOST_CMD:
        ret = woal_hostcmd_ioctl(dev, req);
        break;
#endif
    case WOAL_CUSTOM_IE_CFG:
        ret = woal_custom_ie_ioctl(dev, req);
        break;
    case WOAL_MGMT_FRAME_TX:
        ret = woal_send_host_packet(dev, req);
        break;
    case WOAL_ANDROID_PRIV_CMD:
        ret = woal_android_priv_cmd(dev, req);
        break;
    case WOAL_GET_BSS_TYPE:
        ret = woal_get_bss_type(dev, req);
        break;
    default:
#if defined(STA_WEXT)
#ifdef STA_SUPPORT
        ret = woal_wext_do_ioctl(dev, req, cmd);
#else
        ret = -EINVAL;
#endif
#else
	/*
	 * FIXME Some of the IOCTLs are not supported by Marvel driver
	 * for CFG80211 and hence it is failing to turn on from UI,
	 * so returing 0(Sucess) for the time being.
	 */
        ret = 0;
#endif
        break;
    }

    LEAVE();
    return ret;
}
