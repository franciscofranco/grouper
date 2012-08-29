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
 *	PCB_ID[0] is KB_ROW[4], and
 *	PCB_ID[1] is KB_ROW[5], and
 *	PCB_ID[2] is KB_COL[4], and
 *	PCB_ID[3] is KB_COL[7], and
 *	PCB_ID[4] is KB_ROW[2], and
 *	PCB_ID[5] is KB_COL[5], and
 *	PCB_ID[6] is GMI_CS0_N, and
 *	PCB_ID[7] is GMI_CS1_N, and
 *	PCB_ID[8] is GMI_CS2_N, and
 *	PCB_ID[9] is GMI_WP_N
 *
 *		BT/WIFI module
 *	===============================================================
 *	PCB_ID[1] PCB_ID[0]	type
 *	1	  0		AZWAVE AW-NH665
 *	===============================================================
 *
 *		GPS
 *	===============================================================
 *	PCB_ID[9] PCB_ID[2]	type
 *	0	  0		Broadcom BCM47511
 *	0	  1		Broadcom BCM4751
 *	===============================================================
 *
 *		PCBA revision (nakasi)
 *	===============================================================
 *	PCB_ID[5] PCB_ID[4] PCB_ID[3]	revision
 *	0	  0	    0		nakasi SR3
 *	0	  0	    1		nakasi ER1
 *	0	  1	    0		nakasi ER2
 *	0	  1	    1		nakasi ER3/PR1/PR2
 *		PCBA revision (bach)
 *	===============================================================
 *	PCB_ID[5] PCB_ID[4] PCB_ID[3]	revision
 *	1	  0	    0		bach/ME370TG rev_1.0(SR1)
 *	1	  0	    1		bach/ME370TG rev_1.1(SR2)
 *	1	  1	    0		bach/ME370TG rev_1.2(SR3)
 *	1	  1	    0		bach/ME370TG rev_1.3(SR4)
 *	1	  1	    1		bach/ME370TG rev_1.4(ER1)
 *	0	  0	    0		bach/ME370TG rev_1.4(ER2)
 *	0	  0	    1		bach/ME370TG rev_1.5(PR)
 *	===============================================================
 *
 *		Audio codec
 *	===============================================================
 *	PCB_ID[7] PCB_ID[6]	type
 *	0	  0		Realtek ALC5642
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

/* The grouper_projectid is hexadecimal representation as follows.
 *
 *	PROJECT_ID[0], aka PCB_ID[10], is GMI_CS4_N, and
 *	PROJECT_ID[1], aka PCB_ID[11], is GMI_CS6_N, and
 *	PROJECT_ID[2], aka PCB_ID[12], is GMI_WAIT, and
 *	PROJECT_ID[3], aka PCB_ID[13], is GMI_CS3_N.
 *
 *		Project identification
 *	====================================================
 *	PRJ_ID[3] PRJ_ID[2] PRJ_ID[1] PRJ_ID[0]	project name
 *	0	  0	    0	      0		nakasi/ME370T
 *	0	  0	    0	      1		bach
 *	0	  0	    1	      0		ME370TG
 *	====================================================
 *
 */

#ifndef GROUPER_TEGRA3_DEVKIT_MISC_HW_H
#define GROUPER_TEGRA3_DEVKIT_MISC_HW_H

#if defined(__cplusplus)
extern "C"
{
#endif
struct pins {
	/* gpio number */
	const unsigned gpio;

	/* pingroup number */
	const unsigned pingroup;

	/* gpio description*/
	const char *label;

	/* Indicates extended pins beyond nakasi project
	 * These extended pins are set to pull-down and output disabled in
	 * pinmux init to avoid floating on nakasi project before reading, but
	 * needing to be reset after reading for power saving.
	 * On nakasi project, pin would be restored with no-pull and
	 * input/output buffer disabled.
	 * On other projects, pin would be set to no-pull and output buffer
	 * disabled.
	 */
	const bool extended_pins;
};

enum grouper_pcba_revision {
	GROUPER_PCBA_INVALID = -1,
	GROUPER_PCBA_SR3 = 0,
	GROUPER_PCBA_ER1 = 1,
	GROUPER_PCBA_ER2 = 2,
	GROUPER_PCBA_ER3 = 3,
	GROUPER_PCBA_PR1 = GROUPER_PCBA_ER3,
	GROUPER_PCBA_PR2 = GROUPER_PCBA_PR1,
};

enum tilapia_pcba_revision {
	TILAPIA_PCBA_ER2 = 0,
	TILAPIA_PCBA_PR = 1,
	TILAPIA_PCBA_SR1 = 4,
	TILAPIA_PCBA_SR2 = 5,
	TILAPIA_PCBA_SR3 = 6,
	TILAPIA_PCBA_SR4 = TILAPIA_PCBA_SR3,
	TILAPIA_PCBA_ER1 = 7,
};

enum grouper_project_id {
	GROUPER_PROJECT_INVALID = -1,
	GROUPER_PROJECT_NAKASI = 0,
	GROUPER_PROJECT_NAKASI_3G = 1,
	GROUPER_PROJECT_BACH = GROUPER_PROJECT_NAKASI_3G,
	GROUPER_PROJECT_ME370TG = 2,
	GROUPER_PROJECT_NUM,
};

enum grouper_gps_module {
	GROUPER_GPS_BCM47511 = 0,
	GROUPER_GPS_BCM4751 = 1,
	GROUPER_GPS_NUM,
};

enum grouper_pmic_id {
	GROUPER_PMIC_MAXIM = 0,
	GROUPER_PMIC_TI = 1,
	GROUPER_PMIC_NUM,
};

int __init grouper_misc_init(void);

/*
 * Reset unused pins accordingly for power saving
 */
int __init grouper_misc_reset(void);

/* Query pin status of equipped bt/wifi module defined in PCB pins.
 *   @ret unsigned int
 *      Return unsigned integer to reflect the PCB pin status of equipped PCBA.
 *      Otherwise -1 (Not supported) will be returned.
 */
unsigned int grouper_query_bt_wifi_module(void);

/* Query pin status of equipped hw revision defined in PCB pins.
 *   @ret unsigned int
 *      Return unsigned integer to reflect the PCB pin status of equipped PCBA.
 *      Otherwise -1 (Not supported) will be returned.
 */
unsigned int grouper_query_pcba_revision(void);

/* Query pin status of equipped audio codec defined in PCB pins.
 *   @ret unsigned int
 *      Return unsigned integer to reflect the PCB pin status of equipped PCBA.
 *      Otherwise -1 (Not supported) will be returned.
 */
unsigned int grouper_query_audio_codec(void);

/* Query pin status of equipped gps module defined in PCB pins.
 *   @ret unsigned int
 *      Return unsigned integer to reflect the PCB pin status of equipped PCBA.
 *      Otherwise -1 (Not supported) will be returned.
 */
unsigned int grouper_query_gps_module(void);

/* Query pin status of equipped pmic defined in PCB pins.
 *   @ret unsigned int
 *      Return unsigned integer to reflect the PCB pin status of equipped PCBA.
 *      Otherwise -1 (Not supported) will be returned.
 */
unsigned int grouper_query_pmic_id(void);


/* Acquire project identification in integer format.
 *   @ret unsigned int
 *      Return unsigned integer to reflect enum type grouper_project_id
 *      Otherwise -1 (invalid) will be returned.
 */
unsigned int grouper_get_project_id(void);

#if defined(__cplusplus)
}
#endif

#endif

