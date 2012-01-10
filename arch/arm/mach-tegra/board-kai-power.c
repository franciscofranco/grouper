/*
 * arch/arm/mach-tegra/board-kai-power.c
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
#include <linux/i2c.h>
#include <linux/pda_power.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/regulator/machine.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/regulator/gpio-switch-regulator.h>
#include <linux/power/gpio-charger.h>

#include <asm/mach-types.h>

#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/pinmux.h>
#include <mach/edp.h>

#include "gpio-names.h"
#include "board.h"
#include "board-kai.h"
#include "pm.h"
#include "wakeups-t3.h"
#include "tegra3_tsensor.h"

#define PMC_CTRL		0x0
#define PMC_CTRL_INTR_LOW	(1 << 17)

int __init kai_regulator_init(void)
{
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	u32 pmc_ctrl;

	/* configure the power management controller to trigger PMU
	 * interrupts when low */

	pmc_ctrl = readl(pmc + PMC_CTRL);
	writel(pmc_ctrl | PMC_CTRL_INTR_LOW, pmc + PMC_CTRL);

	return 0;
}

static void kai_board_suspend(int lp_state, enum suspend_stage stg)
{
	if ((lp_state == TEGRA_SUSPEND_LP1) && (stg == TEGRA_SUSPEND_BEFORE_CPU))
		tegra_console_uart_suspend();
}

static void kai_board_resume(int lp_state, enum resume_stage stg)
{
	if ((lp_state == TEGRA_SUSPEND_LP1) && (stg == TEGRA_RESUME_AFTER_CPU))
		tegra_console_uart_resume();
}

static struct tegra_suspend_platform_data kai_suspend_data = {
	.cpu_timer	= 2000,
	.cpu_off_timer	= 200,
	.suspend_mode	= TEGRA_SUSPEND_LP0,
	.core_timer	= 0x7e7e,
	.core_off_timer = 0,
	.corereq_high	= true,
	.sysclkreq_high	= true,
	.cpu_lp2_min_residency = 2000,
	.board_suspend = kai_board_suspend,
	.board_resume = kai_board_resume,
};

int __init kai_suspend_init(void)
{
	tegra_init_suspend(&kai_suspend_data);
	return 0;
}

static void kai_power_off(void)
{
	pr_err("kai: Powering off the device\n");

	while (1)
		;
}

int __init kai_power_off_init(void)
{
	pm_power_off = kai_power_off;

	return 0;
}

static struct tegra_tsensor_pmu_data  tpdata = {
	.poweroff_reg_addr = 0x3F,
	.poweroff_reg_data = 0x80,
	.reset_tegra = 1,
	.controller_type = 0,
	.i2c_controller_id = 4,
	.pinmux = 0,
	.pmu_16bit_ops = 0,
	.pmu_i2c_addr = 0x2D,
};

void __init kai_tsensor_init(void)
{
	tegra3_tsensor_init(&tpdata);
}

#ifdef CONFIG_TEGRA_EDP_LIMITS

int __init kai_edp_init(void)
{
	unsigned int regulator_mA;

	regulator_mA = get_maximum_cpu_current_supported();
	if (!regulator_mA)
		regulator_mA = 6000; /* regular T30/s */
	pr_info("%s: CPU regulator %d mA\n", __func__, regulator_mA);

	tegra_init_cpu_edp_limits(regulator_mA);
	return 0;
}
#endif
