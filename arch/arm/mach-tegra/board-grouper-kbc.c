/*
 * arch/arm/mach-tegra/board-grouper-kbc.c
 * Keys configuration for Nvidia tegra3 grouper platform.
 *
 * Copyright (C) 2012 NVIDIA, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>

#include <mach/irqs.h>
#include <mach/io.h>
#include <mach/iomap.h>
#include <mach/kbc.h>
#include "board.h"
#include "board-grouper.h"

#include "gpio-names.h"
#include "devices.h"

#define GROUPER_ROW_COUNT	1
#define GROUPER_COL_COUNT	4

static const u32 kbd_keymap[] = {
	KEY(0, 0, KEY_RESERVED),
	KEY(0, 1, KEY_RESERVED),
	KEY(0, 2, KEY_VOLUMEUP),
	KEY(0, 3, KEY_VOLUMEDOWN),
};

static const struct matrix_keymap_data keymap_data = {
	.keymap	 = kbd_keymap,
	.keymap_size    = ARRAY_SIZE(kbd_keymap),
};

static struct tegra_kbc_platform_data grouper_kbc_platform_data = {
	.debounce_cnt = 20,
	.repeat_cnt = 1,
	.scan_count = 30,
	.wakeup = false,
	.keymap_data = &keymap_data,
	.wake_cnt = 0,
};

int __init grouper_kbc_init(void)
{
	struct tegra_kbc_platform_data *data = &grouper_kbc_platform_data;
	int i;
	struct board_info board_info;

	tegra_get_board_info(&board_info);
	BUG_ON(board_info.board_id != BOARD_E1565);

	pr_info("Registering tegra-kbc\n");
	tegra_kbc_device.dev.platform_data = &grouper_kbc_platform_data;

	for (i = 0; i < GROUPER_ROW_COUNT; i++) {
		data->pin_cfg[i].num = i;
		data->pin_cfg[i].is_row = true;
		data->pin_cfg[i].en = true;
	}
	for (i = 0; i < GROUPER_COL_COUNT; i++) {
		/*
		 * Avoid keypad scan (ROW0,COL0) and (ROW0, COL1)
		 * KBC-COL1 (GPIO-Q-01) is unused, and
		 * KBC-COL0(GPIO-Q-00) and AP_ONKEY#(GPIO-P-00) are
		 * both wired with power button, but we configure
		 * AP_ONKEY pin for power button instead.
		 */
		if (i <= 1) continue;
		data->pin_cfg[i + KBC_PIN_GPIO_16].num = i;
		data->pin_cfg[i + KBC_PIN_GPIO_16].en = true;
	}

	platform_device_register(&tegra_kbc_device);
	return 0;
}

#define GPIO_KEY(_id, _gpio, _iswake)		\
	{					\
		.code = _id,			\
		.gpio = TEGRA_GPIO_##_gpio,	\
		.active_low = 1,		\
		.desc = #_id,			\
		.type = EV_KEY,			\
		.wakeup = _iswake,		\
		.debounce_interval = 10,	\
	}

static struct gpio_keys_button grouper_keys[] = {
	[0] = GPIO_KEY(KEY_POWER, PV0, 1),
};

static struct gpio_keys_platform_data grouper_keys_platform_data = {
	.buttons	= grouper_keys,
	.nbuttons	= ARRAY_SIZE(grouper_keys),
};

static struct platform_device grouper_keys_device = {
	.name   = "gpio-keys",
	.id     = 0,
	.dev    = {
		.platform_data  = &grouper_keys_platform_data,
	},
};

int __init grouper_keys_init(void)
{
	int i;
	struct board_info board_info;

	tegra_get_board_info(&board_info);
	BUG_ON(board_info.board_id != BOARD_E1565);

	pr_info("Registering gpio keys\n");

	/* Enable gpio mode for other pins */
	for (i = 0; i < grouper_keys_platform_data.nbuttons; i++)
		tegra_gpio_enable(grouper_keys_platform_data.
					buttons[i].gpio);

	platform_device_register(&grouper_keys_device);

	return 0;
}
