/*
 * Copyright (C) 2010-2012 NVIDIA Corporation
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

#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/io.h>

#include <mach/clk.h>
#include <mach/iomap.h>

#include "clock.h"

#define clk_writel(value, reg) \
	__raw_writel(value, (u32)reg_clk_base + (reg))
#define clk_readl(reg) \
	__raw_readl((u32)reg_clk_base + (reg))

#define OSC_FREQ_DET			0x58
#define OSC_FREQ_DET_TRIG		(1<<31)

#define OSC_FREQ_DET_STATUS		0x5C
#define OSC_FREQ_DET_BUSY		(1<<31)
#define OSC_FREQ_DET_CNT_MASK		0xFFFF

static void __iomem *reg_clk_base = IO_ADDRESS(TEGRA_CLK_RESET_BASE);
static unsigned long osc_input_freq;

unsigned long tegra_clk_measure_input_freq(void)
{
	u32 clock_autodetect;

	if (osc_input_freq)
		return osc_input_freq;

	clk_writel(OSC_FREQ_DET_TRIG | 1, OSC_FREQ_DET);
	do {} while (clk_readl(OSC_FREQ_DET_STATUS) & OSC_FREQ_DET_BUSY);
	clock_autodetect = clk_readl(OSC_FREQ_DET_STATUS);

	if (clock_autodetect >= 732 - 3 && clock_autodetect <= 732 + 3)
		osc_input_freq = 12000000;
	else if (clock_autodetect >= 794 - 3 && clock_autodetect <= 794 + 3)
		osc_input_freq = 13000000;
	else if (clock_autodetect >= 1172 - 3 && clock_autodetect <= 1172 + 3)
		osc_input_freq = 19200000;
	else if (clock_autodetect >= 1587 - 3 && clock_autodetect <= 1587 + 3)
		osc_input_freq = 26000000;
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	else if (clock_autodetect >= 1025 - 3 && clock_autodetect <= 1025 + 3)
		osc_input_freq = 16800000;
	else if (clock_autodetect >= 2344 - 3 && clock_autodetect <= 2344 + 3)
		osc_input_freq = 38400000;
	else if (clock_autodetect >= 2928 - 3 && clock_autodetect <= 2928 + 3)
		osc_input_freq = 48000000;
#endif
	else {
		pr_err("%s: Unexpected clock autodetect value %d", __func__,
			clock_autodetect);
		BUG();
	}
	return osc_input_freq;
}
