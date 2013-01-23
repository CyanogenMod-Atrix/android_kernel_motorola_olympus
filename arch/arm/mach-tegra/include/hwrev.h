/*
 *  * Copyright (C) 2010 Motorola, Inc.
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

#ifndef __HWREV_H__
#define __HWREV_H__

#define HWREV_REV(x)  ((x) & 0x0FFF)

#define HWREV_UNDEFINED  0x0000

#define HWREV_TYPE_S     0x1000
#define HWREV_TYPE_M     0x2000
#define HWREV_TYPE_P     0x8000

/* portable with debugging enabled */
#define HWREV_TYPE_DEBUG 0x9000

/* Final hardware can't depend on revision bytes */
#define HWREV_TYPE_FINAL 0xA000

#define HWREV_TYPE_IS_BRASSBOARD(x)  (((x) & 0xF000) == HWREV_TYPE_S)
#define HWREV_TYPE_IS_MORTABLE(x)    (((x) & 0xF000) == HWREV_TYPE_M)
#define HWREV_TYPE_IS_PORTABLE(x)   ((((x) & 0xF000) == HWREV_TYPE_P)||(((x) & 0xF000) == HWREV_TYPE_DEBUG))
#define HWREV_TYPE_IS_DEBUG(x)       (((x) & 0xF000) == HWREV_TYPE_DEBUG)
#define HWREV_TYPE_IS_FINAL(x)       (((x) & 0xF000) == HWREV_TYPE_FINAL)


#define HWREV_REV_0      0x0000
#define HWREV_REV_1      0x0100
#define HWREV_REV_1B     0x01B0
#define HWREV_REV_1C     0x01C0
#define HWREV_REV_2      0x0200
#define HWREV_REV_2B     0x02B0
#define HWREV_REV_2C     0x02C0
#define HWREV_REV_3      0x0300
#define HWREV_REV_3B     0x03B0
#define HWREV_REV_4      0x0400
#define HWREV_REV_4A     0x04A0
#define HWREV_REV_4B     0x04B0
#define HWREV_REV_4F0    0x04F0
#define HWREV_REV_4FB    0x04FB

#endif
