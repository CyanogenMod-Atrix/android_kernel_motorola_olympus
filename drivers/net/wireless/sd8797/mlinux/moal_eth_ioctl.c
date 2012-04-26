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
/** Private command: HT Tx Cfg */
#define PRIV_CMD_HTTXCFG    "httxcfg"
#define PRIV_CMD_DATARATE   "getdatarate"
#define PRIV_CMD_TXRATECFG  "txratecfg"

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

/********************************************************
		Local Functions
********************************************************/
/**
 * @brief Parse a string to extract arguments
 *
 * @param pos           Pointer to the arguments string
 * @param data          Pointer to the arguments buffer
 * @param len           Pointer to the number of arguments extracted
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

#ifdef WIFI_DIRECT_SUPPORT
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
    sprintf(respbuf, "OK");
    ret = 3;

  error:
    if (req)
        kfree(req);

    LEAVE();
    return ret;
}
#endif

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
#ifdef WIFI_DIRECT_SUPPORT
        } else
            if (strnicmp
                (buf + strlen(CMD_MARVELL), PRIV_CMD_HOSTCMD,
                 strlen(PRIV_CMD_HOSTCMD)) == 0) {
            /* hostcmd configuration */
            len = woal_priv_hostcmd(priv, buf, priv_cmd.total_len);
            goto handled;
#endif
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
        } else {
            /* Fall through, after stripping off the custom header */
            buf += strlen(CMD_MARVELL);
        }
    }
#ifdef STA_SUPPORT
    if (strncmp(buf, "RSSI", strlen("RSSI")) == 0) {
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
        if (MLAN_STATUS_SUCCESS != woal_stop_bg_scan(priv)) {
            ret = -EFAULT;
            goto done;
        }
        priv->bg_scan_start = MFALSE;
        priv->bg_scan_reported = MFALSE;
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
    } else if (strncmp(buf, "RXFILTER-ADD", strlen("RXFILTER-ADD")) == 0) {
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
        if (copy_to_user(priv_cmd.buf, buf, len)) {
            PRINTM(MERROR, "%s: failed to copy data to user buffer\n",
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
