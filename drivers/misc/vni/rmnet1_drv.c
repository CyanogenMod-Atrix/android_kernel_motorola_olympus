/******************************************************************************
 * rmnet1_drv.c --  source for the "rmnet1"  Virtual Network Interface        *
 *                                                                            *
 * Copyright (c) 2010 Motorola                                                *
 *                                                                            * 
 * This file creates the Virtual network Interface RMNET0 which writes the    * 
 * data to USB2 interface. This is useful for the Multiple PDP context        *
 * scenarios                                                                  *
 *                                                                            *   
 ******************************************************************************/
/*   DATE        OWNER       COMMENT                                          *
 *   ----------  ----------  -----------------------------------------------  *
 *   2010/09/10  Motorola    Initial version                                  *
 *                                                                            *
 ******************************************************************************/

#ifndef __KERNEL__
#  define __KERNEL__
#endif
/* The Makefile takes care of adding -DMODULE */

#include <linux/etherdevice.h>
#include <linux/module.h>

#include <linux/kernel.h>	/* printk() */
#include <linux/slab.h>		/* kmalloc() */
#include <linux/errno.h>	/* error codes */
#include <linux/netdevice.h>	/* basic data structures */
#include <linux/init.h>		/* __init */
#include <linux/if_arp.h>	/* ARPHRD_ETHER */
#include <linux/version.h>	/* LINUX_VERSION_CODE */
#include <net/arp.h>		/* neighbor stuff */
#include <asm/uaccess.h>	/* memcpy and such */

#include "vni_drv.h"

#define DRIVER_AUTHOR		"Motorola"
#define DRIVER_DESC		"Virtual Network Interface Driver"

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_DESC);

/* --------------------------------------------------------------------------
 * definition of the "private" data structure used by this interface
 */
struct vni_private {
    struct net_device_stats priv_stats;
    struct net_device *priv_device; /* interface used to xmit data */
    int priv_mode; /* how to drop packets */
};

/* --------------------------------------------------------------------------
 * header stuff: fall back on the slave interface to deal with this stuff
 * Creates the Header
 */
static int vni_create_header(struct sk_buff *skb, struct net_device *dev, 
        unsigned short type, const void *daddr, const void *saddr, unsigned len)
{
    struct vni_private *priv = netdev_priv(vni_dev);
    int retval;

    skb->dev = priv->priv_device;
    retval = skb->dev->header_ops->create(skb, skb->dev, type, 
					  daddr, saddr, len);
    skb->dev = dev;
    return retval;
}

/* --------------------------------------------------------------------------
 * header stuff: fall back on the slave interface to deal with this stuff
 * Rebuilds the Header
 */
static int vni_rebuild_header(struct sk_buff *skb)
{
    struct vni_private *priv = netdev_priv(vni_dev);
    int retval;

    skb->dev = priv->priv_device;
    retval = skb->dev->header_ops->rebuild(skb);
    skb->dev = vni_dev;
    return retval;
}

/* --------------------------------------------------------------------------
 * create default header_ops struct
 */
static const struct header_ops vni_header_ops = {
    .create  = vni_create_header,
    .rebuild = vni_rebuild_header,
    .cache   = NULL, /* disable caching */
};

/* --------------------------------------------------------------------------
 * create default netdev_ops struct
 */
static const struct net_device_ops vni_net_device_ops = {
    .ndo_open        = vni_open,
    .ndo_stop        = vni_close,
    .ndo_do_ioctl    = vni_ioctl,
    .ndo_get_stats   = vni_get_stats,
    .ndo_start_xmit  = vni_xmit,
    .ndo_neigh_setup = vni_neigh_setup_dev,
};

module_init(vni_init_module);
module_exit(vni_exit_module);
