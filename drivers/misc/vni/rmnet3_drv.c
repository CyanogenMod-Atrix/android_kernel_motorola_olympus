/******************************************************************************
 * rmnet3_drv.c --  source for the "rmnet3"  Virtual Network Interface        *
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

struct net_device *vni_dev;

/* --------------------------------------------------------------------------
 * definition of the "private" data structure used by this interface
 */
struct vni_private {
    struct net_device_stats priv_stats;
    struct net_device *priv_device; /* interface used to xmit data */
    int priv_mode; /* how to drop packets */
};

/* --------------------------------------------------------------------------
 * open and close
 */
int vni_open(struct net_device *dev)
{
    /* mark the device as operational */
    printk("%s: device opened\n", dev->name);
    netif_start_queue(dev);
    return 0;
}

int vni_close(struct net_device *dev)
{
    printk("%s: device closed\n", dev->name);
    netif_stop_queue(dev);
    return 0;
}

/* --------------------------------------------------------------------------
 * get_stats: return a pointer to the device statistics
 */
struct net_device_stats *vni_get_stats(struct net_device *dev)
{
    return &((struct vni_private *)netdev_priv(dev))->priv_stats;
}

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
 * neighbors set up: needed for ARP to work
 */
int vni_neigh_setup(struct neighbour *n)
{
    if (n->nud_state == NUD_NONE) {
        n->ops = &arp_broken_ops;
        n->output = n->ops->output;
    }
    return 0;
}

int vni_neigh_setup_dev(struct net_device *dev, struct neigh_parms *p)
{
    if (p->tbl->family == AF_INET) {
        p->neigh_setup = vni_neigh_setup;
        p->ucast_probes = 0;
        p->mcast_probes = 0;
    }
    return 0;
}

/* --------------------------------------------------------------------------
 * xmit: actual delivery of the data packets
 */
int vni_xmit(struct sk_buff *skb, struct net_device *dev)
{
    struct vni_private *priv = netdev_priv(dev);
    
    if (!priv->priv_device) { 
        /* cannot send to anyone, just return */
        kfree_skb(skb);
        priv->priv_stats.tx_errors++;
        priv->priv_stats.tx_dropped++;
        return 0;
    }
    
    /* pass it to the real interface */

    priv->priv_stats.tx_packets++;
    priv->priv_stats.tx_bytes += skb->len;

    skb->dev = priv->priv_device;
    skb->priority = 1;
    dev_queue_xmit (skb);
    return 0;
}

/* --------------------------------------------------------------------------
 * ioctl: let user programs configure this interface
 */
int vni_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{
    int err;
    
    struct net_device *slave;
    struct vni_private *priv = netdev_priv(dev);
    /* hold a local (kernel-space) copy of the configuration data */
    struct vni_userinfo info;
    /* and a pointer into user space as well */
    struct vni_userinfo *uptr = (struct vni_userinfo *)ifr->ifr_data;
    
    /* only authorized users can control the interface */
    if (cmd == SIOCINSANESETINFO && !capable(CAP_NET_ADMIN))
        return -EPERM;
    
    /* process the command */
    switch(cmd) {
        case SIOCINSANEGETINFO: /* return configuration to user space */
        
            /* interface name */
            memset(info.name, 0, VNI_NAMELEN);
            if (priv->priv_device)
                strncpy(info.name, priv->priv_device->name, VNI_NAMELEN-1);
        
            /* parameters */
            info.mode = priv->priv_mode;
            /* return the data structure to  user space */
            err = copy_to_user(uptr, &info, sizeof(info));
            if (err) return err;
            break;

        case SIOCINSANESETINFO:
        
            /* retrieve the data structure from user space */
            err = copy_from_user(&info, uptr, sizeof(info));
            if (err) return err;

            printk("name: %s", info.name);

            /* interface name */
            slave = __dev_get_by_name(&init_net, info.name);
            if (!slave)
                return -ENODEV;
            if (slave->type != ARPHRD_ETHER && slave->type != ARPHRD_LOOPBACK)
                return -EINVAL;

            /* The interface is good, get hold of it */
            priv->priv_device = slave;
            if (slave->header_ops)
                dev->header_ops = &vni_header_ops;
            else
                dev->header_ops = NULL;

            /* also, and clone its IP, MAC and other information */
            memcpy(dev->dev_addr,  slave->dev_addr,  ETH_ALEN);
            memcpy(dev->broadcast, slave->broadcast, sizeof(slave->broadcast));

            /* accept the parameters (no checks here) */
            priv->priv_mode = info.mode;

            break;

        default:
            return -EOPNOTSUPP;
    }
    return 0;
}


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

/* --------------------------------------------------------------------------
 * The initialization function: it is used to assign fields in the structure
 * Initializes the operation table and also the slave device to transmit the data 
 */
void vni_init(struct net_device *dev)
{
    struct net_device *slave;
    struct vni_private *priv; 

    ether_setup(dev); /* assign some of the fields as "generic ethernet" */
    memset(netdev_priv(dev), 0, sizeof(struct vni_private));
    
    dev->netdev_ops = &vni_net_device_ops;

    /* Assign random MAC address */
    random_ether_addr(dev->dev_addr);

    /* Hardcode the transmitting module to USB2 */
    /* interface name */
    priv = netdev_priv(dev);
    slave = __dev_get_by_name(&init_net, "usb2");
    if (!slave)
    {
         printk(" Slave Interface Doesn't exists , Returning\n");
         return;
    }
    if (slave->type != ARPHRD_ETHER && slave->type != ARPHRD_LOOPBACK)
    {
         printk("This is not compatible interface");
         return;

    }

    /* The interface is good, get hold of it */
    priv->priv_device = slave;
    if (slave->header_ops)
           dev->header_ops = &vni_header_ops;
    else
           dev->header_ops = NULL;

    /* also, and clone its IP, MAC and other information */
    memcpy(dev->dev_addr,  slave->dev_addr,  ETH_ALEN);
    memcpy(dev->broadcast, slave->broadcast, sizeof(slave->broadcast));

    /* Assign default value (no checks here) */
    priv->priv_mode = VNI_PASS;

}

/* --------------------------------------------------------------------------
 * module entry point
 */

int __init vni_init_module(void)
{
    int err;    
    vni_dev = alloc_netdev(sizeof(struct vni_private), 
                              "rmnet3", vni_init);

    if (!vni_dev)
        return -ENOMEM;
    
    if (( err = dev_alloc_name(vni_dev, vni_dev->name) )) {
        printk(KERN_WARNING "%s: allocate name, error %i\n", 
               vni_dev->name, err);
        return -EIO;
    }
    
    if (( err = register_netdev(vni_dev) )) {
        printk(KERN_WARNING "%s: can't register, error %i\n", 
               vni_dev->name, err);
        return -EIO;
    }
 

    printk("%s: device registered\n", vni_dev->name);
    return 0;
}

/* --------------------------------------------------------------------------
 * module exit point
 */

void __exit vni_exit_module(void)
{   
     
    unregister_netdev(vni_dev);
    printk("%s: device unregistered\n", vni_dev->name);
    free_netdev(vni_dev);
}

module_init(vni_init_module);
module_exit(vni_exit_module);
