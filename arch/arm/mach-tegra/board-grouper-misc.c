/*
 * arch/arm/mach-tegra/board-grouper-misc.c
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
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>

#include <mach/board-grouper-misc.h>
#include "gpio-names.h"
#include "fuse.h"

#define GROUPER_MISC_ATTR(module) \
static struct kobj_attribute module##_attr = { \
	.attr = { \
		.name = __stringify(module), \
		.mode = 0444, \
	}, \
	.show = module##_show, \
}

/* PCBID is composed of four GPIO pins */
static unsigned int grouper_pcbid;

unsigned int grouper_query_pcba_revision(void)
{
	return grouper_pcbid & 0x7;
}
EXPORT_SYMBOL(grouper_query_pcba_revision);

unsigned int grouper_query_pmic_id(void)
{
	unsigned int value = 0;
	value = (grouper_pcbid & 0x8) >> 3;

	return value;
}
EXPORT_SYMBOL(grouper_query_pmic_id);

static ssize_t grouper_chipid_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	char *s = buf;

	s += sprintf(s, "%016llx\n", tegra_chip_uid());
	return (s - buf);
}

static ssize_t grouper_pcbid_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	char *s = buf;
	int i;

	for (i = 4; i > 0; i--)
		s += sprintf(s, "%c", grouper_pcbid & (1 << (i - 1)) ? '1' : '0');
	s += sprintf(s, "b\n");
	return (s - buf);
}

GROUPER_MISC_ATTR(grouper_chipid);
GROUPER_MISC_ATTR(grouper_pcbid);

static struct attribute *attr_list[] = {
	&grouper_chipid_attr.attr,
	&grouper_pcbid_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = attr_list,
};

static struct platform_device *grouper_misc_device;

static int pcbid_init(void)
{
	int ret;

	ret = gpio_request(TEGRA_GPIO_PQ7, "PCB_ID3");
	if (ret) {
		gpio_free(TEGRA_GPIO_PQ7);
		return ret;
        }

	ret = gpio_request(TEGRA_GPIO_PR2, "PCB_ID4");
	if (ret) {
		gpio_free(TEGRA_GPIO_PQ7);
		gpio_free(TEGRA_GPIO_PR2);
		return ret;
        }

	ret = gpio_request(TEGRA_GPIO_PQ5, "PCB_ID5");
	if (ret) {
		gpio_free(TEGRA_GPIO_PQ7);
		gpio_free(TEGRA_GPIO_PR2);
		gpio_free(TEGRA_GPIO_PQ5);
		return ret;

        }
	ret = gpio_request(TEGRA_GPIO_PK3, "PCB_ID8");
	if (ret) {
		gpio_free(TEGRA_GPIO_PQ7);
		gpio_free(TEGRA_GPIO_PR2);
		gpio_free(TEGRA_GPIO_PQ5);
		gpio_free(TEGRA_GPIO_PK3);
		return ret;
        }

	tegra_gpio_enable(TEGRA_GPIO_PQ7);
	tegra_gpio_enable(TEGRA_GPIO_PR2);
	tegra_gpio_enable(TEGRA_GPIO_PQ5);
	tegra_gpio_enable(TEGRA_GPIO_PK3);

	gpio_direction_input(TEGRA_GPIO_PQ7);
	gpio_direction_input(TEGRA_GPIO_PR2);
	gpio_direction_input(TEGRA_GPIO_PQ5);
	gpio_direction_input(TEGRA_GPIO_PK3);

	grouper_pcbid = ((gpio_get_value(TEGRA_GPIO_PK3) << 3 ) |
			(gpio_get_value(TEGRA_GPIO_PQ5) << 2) |
			(gpio_get_value(TEGRA_GPIO_PR2) << 1) |
			gpio_get_value(TEGRA_GPIO_PQ7));
	return 0;
}

int __init grouper_misc_init(void)
{
	int ret = 0;

	pr_debug("%s: start\n", __func__);

	// create a platform device
	grouper_misc_device = platform_device_alloc("grouper_misc", -1);

        if (!grouper_misc_device) {
		ret = -ENOMEM;
		goto fail_platform_device;
        }

	// add a platform device to device hierarchy
	ret = platform_device_add(grouper_misc_device);
	if (ret) {
		pr_err("[MISC]: cannot add device to platform.\n");
		goto fail_platform_add_device;
	}

	ret = sysfs_create_group(&grouper_misc_device->dev.kobj, &attr_group);
	if (ret) {
		pr_err("[MISC]: cannot create sysfs group.\n");
		goto fail_sysfs;
	}

	// acquire pcb_id info
	ret = pcbid_init();
	if (ret) {
		pr_err("[MISC]: cannot acquire PCB_ID info.\n");
		goto fail_sysfs;
	}

	return ret;

fail_sysfs:
	platform_device_del(grouper_misc_device);

fail_platform_add_device:
	platform_device_put(grouper_misc_device);

fail_platform_device:
	return ret;
}
