/*
 * arch/arm/mach-tegra/board-kai.h
 *
 * Copyright (c) 2012, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef _MACH_TEGRA_BOARD_KAI_H
#define _MACH_TEGRA_BOARD_KAI_H

#include <mach/gpio.h>
#include <mach/irqs.h>

/* Processor Board  ID */
#define BOARD_E1565	0xF41

/* Board Fab version */
#define BOARD_FAB_A00			0x0
#define BOARD_FAB_A01			0x1
#define BOARD_FAB_A02			0x2
#define BOARD_FAB_A03			0x3
#define BOARD_FAB_A04			0x4
#define BOARD_FAB_A05			0x5

int kai_charge_init(void);
int kai_regulator_init(void);
int kai_suspend_init(void);
int kai_sdhci_init(void);
int kai_pinmux_init(void);
int kai_panel_init(void);
int kai_keys_init(void);
int kai_pins_state_init(void);
int kai_power_off_init(void);
int kai_edp_init(void);
void __init kai_tsensor_init(void);

#define TDIODE_OFFSET	(10000) /* in millicelsius */

#endif
