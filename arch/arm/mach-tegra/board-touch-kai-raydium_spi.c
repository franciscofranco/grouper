/*
 * arch/arm/mach-tegra/board-touch-raydium_spi.c
 *
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
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/platform_data/rm31080a_ts.h>

#if defined(CONFIG_MACH_KAI)
#include "board-kai.h"
#endif

/* Raydium touchscreen                     Driver data */
/*-----------------------------------------------------*/

struct rm_spi_ts_platform_data rm31080ts_data = {
	.gpio_reset = TOUCH_GPIO_RST_RAYDIUM_SPI,
};

struct spi_board_info rm31080a_spi_board[] = {
	{
		.modalias = "rm_ts_spidev",
		.bus_num = 0,
		.chip_select = 0,
		.irq = TEGRA_GPIO_TO_IRQ(TOUCH_GPIO_IRQ_RAYDIUM_SPI),
		.max_speed_hz = 18*1000*1000,
		.mode = SPI_MODE_0,
		.platform_data = &rm31080ts_data,
	},
};

int __init touch_init_raydium(void)
{
	tegra_gpio_enable(TOUCH_GPIO_IRQ_RAYDIUM_SPI);
	gpio_request(TOUCH_GPIO_IRQ_RAYDIUM_SPI, "raydium-irq");
	gpio_direction_input(TOUCH_GPIO_IRQ_RAYDIUM_SPI);

	tegra_gpio_enable(TOUCH_GPIO_RST_RAYDIUM_SPI);
	gpio_request(TOUCH_GPIO_RST_RAYDIUM_SPI, "raydium-reset");
	gpio_direction_output(TOUCH_GPIO_RST_RAYDIUM_SPI, 0);

	msleep(1);
	gpio_set_value(TOUCH_GPIO_RST_RAYDIUM_SPI, 1);
	msleep(100);

	spi_register_board_info(rm31080a_spi_board,
					ARRAY_SIZE(rm31080a_spi_board));
	return 0;
}
