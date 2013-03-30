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

#endif /* __VNI_DRV_H__ */
