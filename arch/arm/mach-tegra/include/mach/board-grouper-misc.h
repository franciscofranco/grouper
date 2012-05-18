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
 *	0	  1	    0		ER2
 *	0	  1	    1		ER3/PR1
 *	1	  0	    0		PR2
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

enum grouper_pcba_revision {
	GROUPER_PCBA_INVALID = -1,
	GROUPER_PCBA_SR3 = 0,
	GROUPER_PCBA_ER1 = 1,
	GROUPER_PCBA_ER2 = 2,
	GROUPER_PCBA_ER3 = 3,
	GROUPER_PCBA_PR1 = GROUPER_PCBA_ER3,
	GROUPER_PCBA_PR2 = 4,
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

