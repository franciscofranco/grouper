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

extern struct tegra_wake_info *tegra_wake_event_data;
extern unsigned int tegra_wake_event_data_size;

/*
 * FIXME: unable to pass rising and falling
 * flags from usb driver hence using polarity field
 * from wake table to set wake_mask_any
 * for selected usb wake sources - VBUS and ID
 */
static int update_wake_mask(unsigned int index, int flow_type,
	struct wake_mask_types *wake_msk)
{
	int trigger_val;
	/*
	 * set wake function calls with flow_type as -1
	 * set wake type function calls update_wake_mask with
	 * the wake polarity
	 */
	if (flow_type == -1) {
		pr_debug("Wake%d flow_type=%d\n",
			index, flow_type);
		/* use argument wake_mask_hi to return mask */
		wake_msk->wake_mask_hi |= (1ULL << index);
	} else {
		trigger_val = (flow_type & IRQF_TRIGGER_MASK);
		if ((tegra_wake_event_data[index].polarity ==
			POLARITY_EDGE_ANY) ||
			(trigger_val ==
			(IRQF_TRIGGER_RISING |
			IRQF_TRIGGER_FALLING))) {
			pr_debug("Wake%d flow_type=ANY\n", index);
			wake_msk->wake_mask_any |= (1ULL << index);
		} else if ((trigger_val == IRQF_TRIGGER_HIGH) ||
			(trigger_val == IRQF_TRIGGER_RISING)) {
			pr_debug("Wake%d flow_type=HI\n", index);
			wake_msk->wake_mask_hi |= (1ULL << index);
		} else if ((trigger_val == IRQF_TRIGGER_LOW) ||
			(trigger_val == IRQF_TRIGGER_FALLING)) {
			pr_debug("Wake%d flow_type=LO\n", index);
			wake_msk->wake_mask_lo |= (1ULL << index);
		} else {
			pr_err("Error: Wake%d UNKNOWN flow_type=%d\n",
				index, flow_type);
			return -EINVAL;
		}
	}
	return 0;
}

int tegra_irq_to_wake(unsigned int irq, int flow_type,
	struct wake_mask_types *wake_msk)
{
	int i;
	int err;

	wake_msk->wake_mask_hi = 0ULL;
	wake_msk->wake_mask_lo = 0ULL;
	wake_msk->wake_mask_any = 0ULL;
	/*
	 * check for irq based on tegra_wake_event_data table
	 */
	for (i = 0; i < tegra_wake_event_data_size; i++) {
		if (tegra_wake_event_data[i].irq == irq) {
			err = update_wake_mask(i, flow_type, wake_msk);
			if (err)
				return err;
			continue;
		}
	}

	if (wake_msk->wake_mask_hi || wake_msk->wake_mask_lo ||
		wake_msk->wake_mask_any) {
		pr_debug("Enabling wake sources for irq=%d, mask hi=%#llx, lo=%#llx, any=%#llx, flow_type=%d\n",
			irq, wake_msk->wake_mask_hi, wake_msk->wake_mask_lo,
			wake_msk->wake_mask_any, flow_type);
		return 0;
	}
	return -EINVAL;
}

int tegra_wake_to_irq(int wake)
{
	if (wake < 0)
		return -EINVAL;

	if (wake >= tegra_wake_event_data_size)
		return -EINVAL;

	return tegra_wake_event_data[wake].irq;
}

int tegra_disable_wake_source(int wake)
{
	if (wake >= tegra_wake_event_data_size)
		return -EINVAL;

	tegra_wake_event_data[wake].irq = -EINVAL;

	return 0;
}

