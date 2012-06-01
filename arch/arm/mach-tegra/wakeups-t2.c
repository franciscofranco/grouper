/*
 * Copyright (c) 2011, Google, Inc.
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

static int tegra_wake_event_irq_t2[] = {
	[0]  = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PO5),
	[1]  = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PV3),
	[2]  = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PL1),
	[3]  = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PB6),
	[4]  = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PN7),
	[5]  = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PA0),
	[6]  = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PU5),
	[7]  = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PU6),
	[8]  = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PC7),
	[9]  = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PS2),
	[10] = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PAA1),
	[11] = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PW3),
	[12] = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PW2),
	[13] = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PY6),
	[14] = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PV6),
	[15] = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PJ7),
	[16] = INT_RTC,
	[17] = INT_KBC,
	[18] = INT_EXTERNAL_PMU,
	[19] = -EINVAL, /* TEGRA_USB1_VBUS, */
	[20] = -EINVAL, /* TEGRA_USB3_VBUS, */
	[21] = -EINVAL, /* TEGRA_USB1_ID, */
	[22] = -EINVAL, /* TEGRA_USB3_ID, */
	[23] = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PI5),
	[24] = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PV2),
	[25] = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PS4),
	[26] = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PS5),
	[27] = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PS0),
	[28] = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PQ6),
	[29] = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PQ7),
	[30] = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PN2),
};

int *tegra_wake_event_irq = tegra_wake_event_irq_t2;
unsigned int tegra_wake_event_irq_size = ARRAY_SIZE(tegra_wake_event_irq_t2);
