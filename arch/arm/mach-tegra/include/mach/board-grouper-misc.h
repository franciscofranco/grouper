/*
 * arch/arm/mach-tegra/include/mach/board-grouper-misc.h
 *
 * Copyright (C) 2012-2013 ASUSTek Computer Incorporation
 * Author: Paris Yeh <paris_yeh@asus.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/* The grouper_pcbid is hexadecimal representation as follows.
 *
 *	PCB_ID[3] is KB_COL[7], and
 *	PCB_ID[4] is KB_ROW[2], and
 *	PCB_ID[5] is KB_COL[5].
 *
 *		PCBA revision
 *	========================================
 *	PCB_ID[5] PCB_ID[4] PCB_ID[3]	revision
 *	0	  0	    0		SR3
 *	0	  0	    1		ER1
 *	0	  1	    0		Reserved
 *	0	  1	    1		Reserved
 *	1	  0	    0		Reserved
 *	1	  0	    1		Reserved
 *	1	  1	    1		Reserved
 *	========================================
 *
 */
#ifndef GROUPER_TEGRA3_DEVKIT_MISC_HW_H
#define GROUPER_TEGRA3_DEVKIT_MISC_HW_H

#if defined(__cplusplus)
extern "C"
{
#endif
/*
 * The HW_FIELD_* macros are helper macros for the public HW_DRF_* macros.
 */
#define HW_FIELD_LOWBIT(x)      (0?x)
#define HW_FIELD_HIGHBIT(x)     (1?x)
#define HW_FIELD_SIZE(x)        (HW_FIELD_HIGHBIT(x)-HW_FIELD_LOWBIT(x)+1)
#define HW_FIELD_SHIFT(x)       ((0?x)%32)
#define HW_FIELD_MASK(x)        (0xFFFFFFFFUL>>(31-((1?x)%32)+((0?x)%32)))
#define HW_FIELD_BITS(val, x)   (((val) & HW_FIELD_MASK(x))<<HW_FIELD_SHIFT(x))
#define HW_FIELD_SHIFTMASK(x)   (HW_FIELD_MASK(x)<< (HW_FIELD_SHIFT(x)))

/** HW_DRF_VAL - read a field from a register.

    @param d register domain (hardware block)
    @param r register name
    @param f register field
    @param v register value
 */
#define HW_DRF_VAL(d,r,f,v) \
    (((v)>> HW_FIELD_SHIFT(d##_##r##_0_##f##_RANGE)) & \
        HW_FIELD_MASK(d##_##r##_0_##f##_RANGE))

/** HW_DRF_SIZE - read a occupied length from a register.

    @param d register domain (hardware block)
    @param r register name
    @param f register field
 */
#define HW_DRF_SIZE(d,r,f) \
    (HW_FIELD_SIZE(d##_##r##_0_##f##_RANGE))

#define TEGRA3_DEVKIT_MISC_PCBID_NUM	3

/* PCBA hardware revision on grouper platform */
#define TEGRA3_DEVKIT_MISC_HW_0_PCBA_RANGE	2:0
#define TEGRA3_DEVKIT_MISC_HW_0_PCBA_DEFAULT	0x0UL	//SR3
#define TEGRA3_DEVKIT_MISC_HW_0_PCBA_1	0x1UL		//ER1
#define TEGRA3_DEVKIT_MISC_HW_0_PCBA_2	0x2UL		//Reserved
#define TEGRA3_DEVKIT_MISC_HW_0_PCBA_3	0x3UL		//Reserved
#define TEGRA3_DEVKIT_MISC_HW_0_PCBA_4	0x4UL		//Reserved
#define TEGRA3_DEVKIT_MISC_HW_0_PCBA_5	0x5UL		//Reserved
#define TEGRA3_DEVKIT_MISC_HW_0_PCBA_6	0x6UL		//Reserved
#define TEGRA3_DEVKIT_MISC_HW_0_PCBA_7	0x7UL		//Reserved

enum grouper_pcba_revision {
	GROUPER_PCBA_INVALID = -1,
	GROUPER_PCBA_SR3 = 0,
	GROUPER_PCBA_ER1,
	GROUPER_PCBA_MAX,
};

int __init grouper_misc_init(void);

/* Query pin status of equipped hw revision of development PCBA defined in PCB pins.
 *   @ret unsigned int
 *      Return unsigned integer to reflect the PCB pin status of equipped PCBA.
 *      Otherwise -1 (Not supported) will be returned.
 */
unsigned int grouper_query_pcba_revision(void);

#if defined(__cplusplus)
}
#endif

#endif

