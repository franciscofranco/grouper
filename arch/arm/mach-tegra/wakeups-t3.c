/*
 * Copyright (c) 2011, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/io.h>

#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/gpio.h>

#include "gpio-names.h"
#include "wakeups.h"

static struct tegra_wake_info tegra_wake_event_data_t3[] = {
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PO5), POLARITY_NONE},	/* wake0 */
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PV1), POLARITY_NONE},	/* wake1 */
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PL1), POLARITY_NONE},	/* wake2 */
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PB6), POLARITY_NONE},	/* wake3 */
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PN7), POLARITY_NONE},	/* wake4 */
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PBB6), POLARITY_NONE},	/* wake5 */
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PU5), POLARITY_NONE},	/* wake6 */
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PU6), POLARITY_NONE},	/* wake7 */
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PC7), POLARITY_NONE},	/* wake8 */
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PS2), POLARITY_NONE},	/* wake9 */
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PAA1), POLARITY_NONE},	/* wake10 */
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PW3), POLARITY_NONE},	/* wake11 */
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PW2), POLARITY_NONE},	/* wake12 */
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PY6), POLARITY_NONE},	/* wake13 */
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PDD3), POLARITY_NONE},	/* wake14 */
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PJ2), POLARITY_NONE},	/* wake15 */
	{INT_RTC, POLARITY_NONE},				/* wake16 */
	{INT_KBC, POLARITY_NONE},				/* wake17 */
	{INT_EXTERNAL_PMU, POLARITY_NONE},			/* wake18 */
	{INT_USB, POLARITY_EDGE_ANY}, /* TEGRA_USB1_VBUS, */		/* wake19 */
	{-EINVAL, POLARITY_EDGE_ANY}, /* TEGRA_USB2_VBUS, */		/* wake20 */
	{INT_USB, POLARITY_EDGE_ANY}, /* TEGRA_USB1_ID, */		/* wake21 */
	{-EINVAL, POLARITY_EDGE_ANY}, /* TEGRA_USB2_ID, */		/* wake22 */
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PI5), POLARITY_NONE},	/* wake23 */
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PV0), POLARITY_NONE},	/* wake24 */
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PS4), POLARITY_NONE},	/* wake25 */
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PS5), POLARITY_NONE},	/* wake26 */
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PS0), POLARITY_NONE},	/* wake27 */
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PS6), POLARITY_NONE},	/* wake28 */
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PS7), POLARITY_NONE},	/* wake29 */
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PN2), POLARITY_NONE},	/* wake30 */
	{-EINVAL, POLARITY_NONE}, /* not used */			/* wake31 */
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PO4), POLARITY_NONE},	/* wake32 */
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PJ0), POLARITY_NONE},	/* wake33 */
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PK2), POLARITY_NONE},	/* wake34 */
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PI6), POLARITY_NONE},	/* wake35 */
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PBB1), POLARITY_NONE},	/* wake36 */
	{-EINVAL, POLARITY_NONE}, /* TEGRA_USB3_VBUS, */		/* wake37 */
	{-EINVAL, POLARITY_NONE}, /* TEGRA_USB3_ID, */		/* wake38 */
	{INT_USB, POLARITY_LEVEL_HI}, /* TEGRA_USB1_UTMIP, */		/* wake39 */
	{INT_USB2, POLARITY_LEVEL_HI}, /* TEGRA_USB2_UTMIP, */	/* wake40 */
	{INT_USB3, POLARITY_LEVEL_HI} /* TEGRA_USB3_UTMIP, */	/* wake41 */
};

struct tegra_wake_info *tegra_wake_event_data = tegra_wake_event_data_t3;
unsigned int tegra_wake_event_data_size = ARRAY_SIZE(tegra_wake_event_data_t3);

