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

static struct tegra_wake_info tegra_wake_event_data_t2[] = {
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PO5), POLARITY_NONE},
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PV3), POLARITY_NONE},
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PL1), POLARITY_NONE},
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PB6), POLARITY_NONE},
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PN7), POLARITY_NONE},
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PA0), POLARITY_NONE},
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PU5), POLARITY_NONE},
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PU6), POLARITY_NONE},
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PC7), POLARITY_NONE},
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PS2), POLARITY_NONE},
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PAA1), POLARITY_NONE},
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PW3), POLARITY_NONE},
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PW2), POLARITY_NONE},
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PY6), POLARITY_NONE},
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PV6), POLARITY_NONE},
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PJ7), POLARITY_NONE},
	{INT_RTC, POLARITY_NONE},
	{INT_KBC, POLARITY_NONE},
	{INT_EXTERNAL_PMU, POLARITY_NONE},
	{INT_USB, POLARITY_EDGE_ANY}, /* TEGRA_USB1_VBUS, */
	{-EINVAL, POLARITY_EDGE_ANY}, /* TEGRA_USB3_VBUS, */
	{-EINVAL, POLARITY_EDGE_ANY}, /* TEGRA_USB1_ID, */
	{-EINVAL, POLARITY_EDGE_ANY}, /* TEGRA_USB3_ID, */
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PI5), POLARITY_NONE},
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PV2), POLARITY_NONE},
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PS4), POLARITY_NONE},
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PS5), POLARITY_NONE},
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PS0), POLARITY_NONE},
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PQ6), POLARITY_NONE},
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PQ7), POLARITY_NONE},
	{TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PN2), POLARITY_NONE},
};

struct tegra_wake_info *tegra_wake_event_data = tegra_wake_event_data_t2;
unsigned int tegra_wake_event_data_size = ARRAY_SIZE(tegra_wake_event_data_t2);

