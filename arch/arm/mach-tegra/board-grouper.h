/*
 * arch/arm/mach-tegra/board-grouper.h
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

#ifndef _MACH_TEGRA_BOARD_GROUPER_H
#define _MACH_TEGRA_BOARD_GROUPER_H

#include <mach/gpio.h>
#include <mach/irqs.h>
#include <linux/mfd/max77663-core.h>
#include "gpio-names.h"

/* Processor Board  ID */
#define BOARD_E1565	0xF41

/* Board Fab version */
#define BOARD_FAB_A00			0x0
#define BOARD_FAB_A01			0x1
#define BOARD_FAB_A02			0x2
#define BOARD_FAB_A03			0x3
#define BOARD_FAB_A04			0x4
#define BOARD_FAB_A05			0x5

/* External peripheral act as gpio */
/* MAX77663 GPIO */
#define MAX77663_GPIO_BASE	TEGRA_NR_GPIOS
#define MAX77663_GPIO_END	(MAX77663_GPIO_BASE + MAX77663_GPIO_NR)

/* CAMERA RELATED GPIOs on GROUPER */
#define CAM2_RST_GPIO		TEGRA_GPIO_PBB4
#define CAM2_POWER_DWN_GPIO	TEGRA_GPIO_PBB6
/* Audio-related GPIOs */
#define TEGRA_GPIO_CDC_IRQ		TEGRA_GPIO_PW3
#define TEGRA_GPIO_SPKR_EN		-1
#define TEGRA_GPIO_HP_DET		TEGRA_GPIO_PW2
#define TEGRA_GPIO_INT_MIC_EN		TEGRA_GPIO_PK3
#define TEGRA_GPIO_EXT_MIC_EN		TEGRA_GPIO_PK4
/* Tegra Modem related GPIOs */
#define TEGRA_GPIO_W_DISABLE		TEGRA_GPIO_PDD7
#define TEGRA_GPIO_MODEM_RSVD1		TEGRA_GPIO_PV0
#define TEGRA_GPIO_MODEM_RSVD2		TEGRA_GPIO_PH7

/* Stat LED GPIO */
#define TEGRA_GPIO_STAT_LED		(MAX77663_GPIO_BASE + MAX77663_GPIO7)


/*****************Interrupt tables ******************/
/* External peripheral act as interrupt controller */
/* MAX77663 IRQs */
#define MAX77663_IRQ_BASE	TEGRA_NR_IRQS
#define MAX77663_IRQ_END	(MAX77663_IRQ_BASE + MAX77663_IRQ_NR)
#define MAX77663_IRQ_ACOK_RISING MAX77663_IRQ_ONOFF_ACOK_RISING
#define MAX77663_IRQ_ACOK_FALLING MAX77663_IRQ_ONOFF_ACOK_FALLING

/* UART port which is used by bluetooth*/
#define BLUETOOTH_UART_DEV_NAME "/dev/ttyHS2"

int grouper_charge_init(void);
int grouper_regulator_init(void);
int grouper_suspend_init(void);
int grouper_sdhci_init(void);
int grouper_pinmux_init(void);
int grouper_panel_init(void);
int grouper_sensors_init(void);
int grouper_keys_init(void);
int grouper_pins_state_init(void);
int grouper_emc_init(void);
int grouper_edp_init(void);
int grouper_ti_regulator_init(void);
int grouper_ti_suspend_init(void);
int grouper_ti_edp_init(void);

void __init grouper_tsensor_init(void);
int __init touch_init_raydium(int irq_gpio, int reset_gpio, int platform);
int __init touch_init_synaptics_grouper(void);

#define TOUCH_GPIO_IRQ_RAYDIUM_SPI      TEGRA_GPIO_PZ3
#define TOUCH_GPIO_RST_RAYDIUM_SPI      TEGRA_GPIO_PN5

#define SYNAPTICS_ATTN_GPIO             TEGRA_GPIO_PZ3
#define SYNAPTICS_RESET_GPIO            TEGRA_GPIO_PN5

#define GROUPER_TS_ID1      TEGRA_GPIO_PI7
#define GROUPER_TS_ID2      TEGRA_GPIO_PC7
#define GROUPER_TS_ID1_PG   TEGRA_PINGROUP_GMI_WAIT
#define GROUPER_TS_ID2_PG   TEGRA_PINGROUP_GMI_WP_N

#define MPU_TYPE_MPU3050	1
#define MPU_TYPE_MPU6050	2
#define MPU_GYRO_TYPE		MPU_TYPE_MPU6050
#define MPU_GYRO_IRQ_GPIO	TEGRA_GPIO_PX1
#define MPU_GYRO_ADDR		0x68
#define MPU_GYRO_BUS_NUM	2
#define MPU_GYRO_ORIENTATION	{ 0, 1, 0, 1, 0, 0, 0, 0, -1 }
#if (MPU_GYRO_TYPE == MPU_TYPE_MPU3050)
#define MPU_ACCEL_NAME		"kxtf9"
#define MPU_ACCEL_IRQ_GPIO	0	// No kxtf9 irq-PIN
#define MPU_ACCEL_ADDR		0x0F
#define MPU_ACCEL_BUS_NUM	2
#define MPU_ACCEL_ORIENTATION	{ 0, 1, 0, 1, 0, 0, 0, 0, -1 }
#endif
#define MPU_COMPASS_NAME	"ami306"
#define MPU_COMPASS_IRQ_GPIO	TEGRA_GPIO_PW0
#define MPU_COMPASS_ADDR	0x0E
#define MPU_COMPASS_BUS_NUM	2
#define MPU_COMPASS_ORIENTATION	{ 0, -1, 0, -1, 0, 0, 0, 0, -1 }

#define TDIODE_OFFSET	(10000) /* in millicelsius */

#endif
