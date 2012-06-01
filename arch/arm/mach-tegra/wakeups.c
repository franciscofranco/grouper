/*
 * Copyright (c) 2012, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/interrupt.h>

#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/gpio.h>

#include "gpio-names.h"
#include "wakeups.h"

extern int *tegra_wake_event_irq;
extern unsigned int tegra_wake_event_irq_size;

int tegra_irq_to_wake(int irq)
{
	int i;
	int wake_irq;
	int search_gpio;
	static int last_wake = -1;

	/* Two level wake irq search for gpio based wakeups -
	 * 1. check for GPIO irq(based on tegra_wake_event_irq table)
	 * e.g. for a board, wake7 based on GPIO PU6 and irq==390 done first
	 * 2. check for gpio bank irq assuming search for GPIO irq
	 *    preceded this search.
	 * e.g. in this step check for gpio bank irq GPIO6 irq==119
	 */
	for (i = 0; i < tegra_wake_event_irq_size; i++) {
		/* return if step 1 matches */
		if (tegra_wake_event_irq[i] == irq) {
			pr_info("Wake%d for irq=%d\n", i, irq);
			last_wake = i;
			return i;
		}

		/* step 2 below uses saved last_wake from step 1
		 * in previous call */
		search_gpio = irq_to_gpio(
			tegra_wake_event_irq[i]);
		if (search_gpio < 0)
			continue;
		wake_irq = tegra_gpio_get_bank_int_nr(search_gpio);
		if (wake_irq < 0)
			continue;
		if ((last_wake == i) &&
			(wake_irq == irq)) {
			pr_info("gpio bank wake found: wake%d for irq=%d\n",
				i, irq);
			return i;
		}
	}

	return -EINVAL;
}

int tegra_wake_to_irq(int wake)
{
	if (wake < 0)
		return -EINVAL;

	if (wake >= tegra_wake_event_irq_size)
		return -EINVAL;

	return tegra_wake_event_irq[wake];
}

