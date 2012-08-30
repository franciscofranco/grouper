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
 *	PCB_ID[5] is KB_COL[5], and
 *	PCB_ID[8] is GMI_CS2_N.
 *
 *		PCBA revision
 *	===============================================================
 *	PCB_ID[5] PCB_ID[4] PCB_ID[3]	revision
 *	0	  0	    0		nakasi SR3
 *	0	  0	    1		nakasi ER1
 *	0	  1	    0		nakasi ER2
 *	0	  1	    1		nakasi ER3/PR1/PR2
 *	===============================================================
 *
 *		PMIC module
 *	===============================================================
 *	PCB_ID[8]	type
 *	0		Maxim
 *	1		TI
 *	===============================================================
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
	GROUPER_PCBA_PR2 = GROUPER_PCBA_PR1,
};

enum grouper_pmic_id {
	GROUPER_PMIC_MAXIM = 0,
	GROUPER_PMIC_TI = 1,
	GROUPER_PMIC_NUM,
};

int __init grouper_misc_init(void);

/* Query pin status of equipped hw revision of development PCBA defined in PCB pins.
 *   @ret unsigned int
 *      Return unsigned integer to reflect the PCB pin status of equipped PCBA.
 *      Otherwise -1 (Not supported) will be returned.
 */
unsigned int grouper_query_pcba_revision(void);

/* Query pin status of equipped pmic defined in PCB pins.
 *   @ret unsigned int
 *      Return unsigned integer to reflect the PCB pin status of equipped PCBA.
 *      Otherwise -1 (Not supported) will be returned.
 */
unsigned int grouper_query_pmic_id(void);

#if defined(__cplusplus)
}
#endif

#endif

