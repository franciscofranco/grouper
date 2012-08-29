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
#include <mach/pinmux.h>
#include <mach/pinmux-t3.h>
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

/* PCBID is composed of ten GPIO pins */
static unsigned int grouper_pcbid;

static const struct pins grouper_pcbid_pins[] = {
	{TEGRA_GPIO_PR4, TEGRA_PINGROUP_KB_ROW4, "PCB_ID0", false},
	{TEGRA_GPIO_PR5, TEGRA_PINGROUP_KB_ROW5, "PCB_ID1", false},
	{TEGRA_GPIO_PQ4, TEGRA_PINGROUP_KB_COL4, "PCB_ID2", false},
	{TEGRA_GPIO_PQ7, TEGRA_PINGROUP_KB_COL7, "PCB_ID3", false},
	{TEGRA_GPIO_PR2, TEGRA_PINGROUP_KB_ROW2, "PCB_ID4", false},
	{TEGRA_GPIO_PQ5, TEGRA_PINGROUP_KB_COL5, "PCB_ID5", false},
	{TEGRA_GPIO_PJ0, TEGRA_PINGROUP_GMI_CS0_N, "PCB_ID6", false},
	{TEGRA_GPIO_PJ2, TEGRA_PINGROUP_GMI_CS1_N, "PCB_ID7", false},
	{TEGRA_GPIO_PK3, TEGRA_PINGROUP_GMI_CS2_N, "PCB_ID8", false},
	{TEGRA_GPIO_PC7, TEGRA_PINGROUP_GMI_WP_N, "PCB_ID9", true},
};

/* PROJECTID is composed of four GPIO pins */
static unsigned int grouper_projectid;

static const struct pins grouper_projectid_pins[] = {
	{TEGRA_GPIO_PK2, TEGRA_PINGROUP_GMI_CS4_N, "PROJECT_ID0", true},
	{TEGRA_GPIO_PI3, TEGRA_PINGROUP_GMI_CS6_N, "PROJECT_ID1", true},
	{TEGRA_GPIO_PI7, TEGRA_PINGROUP_GMI_WAIT, "PROJECT_ID2", true},
	{TEGRA_GPIO_PK4, TEGRA_PINGROUP_GMI_CS3_N, "PROJECT_ID3", true},
};

unsigned int grouper_query_bt_wifi_module(void)
{
	unsigned int value = 0;
	value = grouper_pcbid & 0x003;

	return value;
}
EXPORT_SYMBOL(grouper_query_bt_wifi_module);

unsigned int grouper_query_pcba_revision(void)
{
	unsigned int value = 0;
	value = (grouper_pcbid & 0x038) >> 3;

	return value;
}
EXPORT_SYMBOL(grouper_query_pcba_revision);

unsigned int grouper_query_audio_codec(void)
{
	unsigned int value = 0;
	value = (grouper_pcbid & 0x0c0) >> 6;

	return value;
}
EXPORT_SYMBOL(grouper_query_audio_codec);

unsigned int grouper_query_gps_module(void)
{
	unsigned int value = 0;
	value = ((grouper_pcbid & 0x004) >> 2) +
		(((grouper_pcbid & 0x200) >> 9) << 1);

	return value;
}
EXPORT_SYMBOL(grouper_query_gps_module);

unsigned int grouper_query_pmic_id(void)
{
	unsigned int value = 0;
	value = (grouper_pcbid & 0x100) >> 8;

	return value;
}
EXPORT_SYMBOL(grouper_query_pmic_id);

unsigned int grouper_get_project_id(void)
{
	unsigned int value = 0;
	value = grouper_projectid & 0xf;

	return value;
}
EXPORT_SYMBOL(grouper_get_project_id);

static unsigned int grouper_get_pcb_id(void)
{
	unsigned int value = 0;
	/* concatenate projectid[13:10] with pcbid[9:0] in new format. */
	value = (grouper_get_project_id() << ARRAY_SIZE(grouper_pcbid_pins)) |
		(grouper_pcbid & 0x3ff);

	return value;
}

static ssize_t grouper_chipid_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	char *s = buf;

	s += sprintf(s, "%016llx\n", tegra_chip_uid());
	return s - buf;
}

static ssize_t grouper_pcbid_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	char *s = buf;

	s += sprintf(s, "%04x\n", grouper_get_pcb_id());
	return s - buf;
}

static ssize_t grouper_projectid_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	char *s = buf;

	s += sprintf(s, "%02x\n", grouper_get_project_id());
	return s - buf;
}

GROUPER_MISC_ATTR(grouper_chipid);
GROUPER_MISC_ATTR(grouper_pcbid);
GROUPER_MISC_ATTR(grouper_projectid);

static struct attribute *attr_list[] = {
	&grouper_chipid_attr.attr,
	&grouper_pcbid_attr.attr,
	&grouper_projectid_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = attr_list,
};

static struct platform_device *grouper_misc_device;

static int board_pins_init(
	const struct pins *board_pins,
	unsigned int pin_size,
	unsigned int *pin_value)
{
	int ret = 0, i = 0;


	for (i = 0; i < pin_size; i++) {
		ret = gpio_request(board_pins[i].gpio, board_pins[i].label);

		if (ret) {
			while (i >= 0) {
				gpio_free(board_pins[i].gpio);
				i--;
			}
			return ret;
		}
		tegra_gpio_enable(board_pins[i].gpio);
		gpio_direction_input(board_pins[i].gpio);
		*pin_value |= gpio_get_value(board_pins[i].gpio) << i;
	}

	return 0;
}

static void board_pins_reset(
	const struct pins *board_pins,
	unsigned int pin_size)
{
	int i = 0;
	enum grouper_project_id project_id = grouper_get_project_id();

	for (i = 0; i < pin_size; i++) {
		if (board_pins[i].extended_pins) {
			/* set no-pull */
			tegra_pinmux_set_pullupdown(board_pins[i].pingroup,
				TEGRA_PUPD_NORMAL);
			if (project_id == GROUPER_PROJECT_NAKASI) {
				/* disable input buffer */
				tegra_pinmux_set_io(board_pins[i].pingroup,
				TEGRA_PIN_OUTPUT);
				/* mask GPIO_CNF */
				tegra_gpio_disable(board_pins[i].gpio);
				gpio_free(board_pins[i].gpio);
			}
		}
	}
}

int __init grouper_misc_init(void)
{
	int ret = 0;

	pr_debug("%s: start\n", __func__);

	/* create a platform device */
	grouper_misc_device = platform_device_alloc("grouper_misc", -1);

        if (!grouper_misc_device) {
		ret = -ENOMEM;
		goto fail_platform_device;
        }

	/* add a platform device to device hierarchy */
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

	/* acquire pcb_id info */
	ret = board_pins_init(grouper_pcbid_pins,
		ARRAY_SIZE(grouper_pcbid_pins), &grouper_pcbid);
	if (ret) {
		pr_err("[MISC]: cannot acquire PCB_ID info.\n");
		goto fail_sysfs;
	}

	/* acquire project_id info */
	ret = board_pins_init(grouper_projectid_pins,
		ARRAY_SIZE(grouper_projectid_pins), &grouper_projectid);
	if (ret) {
		pr_err("[MISC]: cannot acquire PROJECT_ID info.\n");
		goto fail_sysfs;
	}

	/* print out pcb_id and project_id info */
	pr_info("[MISC]: pcbid=0x%04x (projectid=0x%02x)\n",
		grouper_get_pcb_id(), grouper_get_project_id());

	return ret;

fail_sysfs:
	platform_device_del(grouper_misc_device);

fail_platform_add_device:
	platform_device_put(grouper_misc_device);

fail_platform_device:
	return ret;
}

int __init grouper_misc_reset(void)
{
	/* reset pcb_id pins */
	board_pins_reset(grouper_pcbid_pins,
		ARRAY_SIZE(grouper_pcbid_pins));

	/* reset project_id pins */
	board_pins_reset(grouper_projectid_pins,
		ARRAY_SIZE(grouper_projectid_pins));

	return 0;
}
