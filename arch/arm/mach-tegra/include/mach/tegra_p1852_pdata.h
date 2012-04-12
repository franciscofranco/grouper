/*
 * arch/arm/mach-tegra/include/mach/tegra_p1852_pdata.h
 *
 * Copyright 2012 NVIDIA, Inc.
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

#ifndef __MACH_TEGRA_P1852_PDATA_H
#define __MACH_TEGRA_P1852_PDATA_H

#define NUM_AUDIO_CONTROLLERS 4

/* data format supported */
enum i2s_data_format {
	format_i2s = 0x1,
	format_dsp = 0x2,
	format_rjm = 0x4,
	format_ljm = 0x8,
	format_tdm = 0x10
};

struct codec_info_s {
	/* Name of the Codec Dai on the system */
	char *codec_dai_name;
	/* Name of the I2S controller dai its connected to */
	char *cpu_dai_name;
	char *codec_name;	/* Name of the Codec Driver */
	char *name;			/* Name of the Codec-Dai-Link */
	enum i2s_data_format i2s_format;
	int master;			/* Codec is Master or Slave */
};

struct tegra_p1852_platform_data {
	struct codec_info_s codec_info[NUM_AUDIO_CONTROLLERS];
};
#endif
