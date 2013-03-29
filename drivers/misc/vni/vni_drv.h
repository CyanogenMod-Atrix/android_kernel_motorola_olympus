/******************************************************************************
 * vni_drv.h --  Header file for Virtual Network Interfaces                   *
 *                                                                            * 
 * Copyright (c) 2010 Motorola                                                * 
 *                                                                            *
 ******************************************************************************/
/*   DATE        OWNER       COMMENT                                          *
 *   ----------  ----------  -----------------------------------------------  *
 *   2010/09/10  Motorola    Initial version                                  *
 ******************************************************************************/

#ifndef __VNI_DRV_H__
#define __VNI_DRV_H__

#include <net/arp.h>		/* neighbor stuff */

enum vni_mode {
    VNI_PASS = 0,   /* every packet is transmitted (default) */
    VNI_PERCENT,    /* pass some percent of the packets */
    VNI_TIME,       /* work (and fail) on a timely basis */
};

#define VNI_NAMELEN 16

/* Structure used to exchange data during ioctl commands */
struct vni_userinfo {
    char name[VNI_NAMELEN];
    int mode;
};

/* These are the two ioctl commands needed to interact with Virtual Network Interface */
#define SIOCINSANESETINFO SIOCDEVPRIVATE
#define SIOCINSANEGETINFO (SIOCDEVPRIVATE+1)

extern struct neigh_ops arp_broken_ops;

extern struct net_device *vni_dev;

/* --------------------------------------------------------------------------
 * open and close
 */
int vni_open(struct net_device *dev);

int vni_close(struct net_device *dev);

/* --------------------------------------------------------------------------
 * get_stats: return a pointer to the device statistics
 */
struct net_device_stats *vni_get_stats(struct net_device *dev);

/* --------------------------------------------------------------------------
 * neighbors set up: needed for ARP to work
 */
int vni_neigh_setup(struct neighbour *n);

int vni_neigh_setup_dev(struct net_device *dev, struct neigh_parms *p);

/* --------------------------------------------------------------------------
 * xmit: actual delivery of the data packets
 */
int vni_xmit(struct sk_buff *skb, struct net_device *dev);

/* --------------------------------------------------------------------------
 * ioctl: let user programs configure this interface
 */
int vni_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd);

/* --------------------------------------------------------------------------
 * The initialization function: it is used to assign fields in the structure
 * Initializes the operation table and also the slave device to transmit the data 
 */
void vni_init(struct net_device *dev);

/* --------------------------------------------------------------------------
 * module entry point
 */

int __init vni_init_module(void);

/* --------------------------------------------------------------------------
 * module exit point
 */

void __exit vni_exit_module(void);

#endif /* __VNI_DRV_H__ */
