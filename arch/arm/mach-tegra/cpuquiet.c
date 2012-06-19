/*
 * arch/arm/mach-tegra/cpuquiet.c
 *
 * Cpuquiet driver for Tegra3 CPUs
 *
 * Copyright (c) 2012 NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
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
#include <linux/module.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/cpu.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/pm_qos_params.h>
#include <linux/cpuquiet.h>

#include "pm.h"
#include "cpu-tegra.h"
#include "clock.h"

#define INITIAL_STATE		TEGRA_CPQ_IDLE
#define UP_DELAY_MS		70
#define DOWN_DELAY_MS		2000

static struct mutex *tegra3_cpu_lock;
static struct workqueue_struct *cpuquiet_wq;
static struct delayed_work cpuquiet_work;
static struct work_struct minmax_work;

static bool no_lp;
module_param(no_lp, bool, 0644);

static unsigned long up_delay;
module_param(up_delay, ulong, 0644);
static unsigned long down_delay;
module_param(down_delay, ulong, 0644);

static int mp_overhead = 10;
module_param(mp_overhead, int, 0644);
static unsigned int idle_top_freq;
module_param(idle_top_freq, uint, 0644);
static unsigned int idle_bottom_freq;
module_param(idle_bottom_freq, uint, 0644);

static struct clk *cpu_clk;
static struct clk *cpu_g_clk;
static struct clk *cpu_lp_clk;

static struct cpumask cr_online_requests;

enum {
	TEGRA_CPQ_DISABLED = 0,
	TEGRA_CPQ_IDLE,
	TEGRA_CPQ_SWITCH_TO_LP,
	TEGRA_CPQ_SWITCH_TO_G,
};

static int cpq_state;

static int update_core_config(unsigned int cpunumber, bool up)
{
	int ret = -EINVAL;
	unsigned int nr_cpus = num_online_cpus();
	int max_cpus = pm_qos_request(PM_QOS_MAX_ONLINE_CPUS) ? : 4;
	int min_cpus = pm_qos_request(PM_QOS_MIN_ONLINE_CPUS);

	if (cpq_state == TEGRA_CPQ_DISABLED || cpunumber >= nr_cpu_ids)
		return ret;

	if (up) {
		if(is_lp_cluster()) {
			cpumask_set_cpu(cpunumber, &cr_online_requests);
			ret = -EBUSY;
		} else {
			if (tegra_cpu_edp_favor_up(nr_cpus, mp_overhead) &&
			    nr_cpus < max_cpus)
				ret = cpu_up(cpunumber);
		}
	} else {
		if (is_lp_cluster()) {
			ret = -EBUSY;
		} else {
			if (nr_cpus > min_cpus)
				ret = cpu_down(cpunumber);
		}
	}

	return ret;
}

static int tegra_quiesence_cpu(unsigned int cpunumber)
{
        return update_core_config(cpunumber, false);
}

static int tegra_wake_cpu(unsigned int cpunumber)
{
        return update_core_config(cpunumber, true);
}

static struct cpuquiet_driver tegra_cpuquiet_driver = {
        .name                   = "tegra",
        .quiesence_cpu          = tegra_quiesence_cpu,
        .wake_cpu               = tegra_wake_cpu,
};

static void apply_core_config(void)
{
	unsigned int cpu;

	if (is_lp_cluster() || cpq_state == TEGRA_CPQ_DISABLED)
		return;

	for_each_cpu_mask(cpu, cr_online_requests) {
		if (cpu < nr_cpu_ids && !cpu_online(cpu))
			if (!tegra_wake_cpu(cpu))
				cpumask_clear_cpu(cpu, &cr_online_requests);
	}
}

static void tegra_cpuquiet_work_func(struct work_struct *work)
{
	bool update_cr_config = false;

	mutex_lock(tegra3_cpu_lock);

	switch(cpq_state) {
		case TEGRA_CPQ_DISABLED:
		case TEGRA_CPQ_IDLE:
			break;
		case TEGRA_CPQ_SWITCH_TO_G:
			if (is_lp_cluster()) {
				if(!clk_set_parent(cpu_clk, cpu_g_clk)) {
					/*catch-up with governor target speed */
					tegra_cpu_set_speed_cap(NULL);
					/* process pending core requests*/
					update_cr_config = true;
				}
			}
			break;
		case TEGRA_CPQ_SWITCH_TO_LP:
			if (!is_lp_cluster() && !no_lp &&
				!pm_qos_request(PM_QOS_MIN_ONLINE_CPUS)
				&& num_online_cpus() == 1) {
				if (!clk_set_parent(cpu_clk, cpu_lp_clk)) {
					/*catch-up with governor target speed*/
					tegra_cpu_set_speed_cap(NULL);
				}
			}
			break;
		default:
			pr_err("%s: invalid tegra hotplug state %d\n",
		       __func__, cpq_state);
	}

	mutex_unlock(tegra3_cpu_lock);

	if (update_cr_config)
		apply_core_config();
}

static void min_max_constraints_workfunc(struct work_struct *work)
{
	int count = -1;
	bool up = false;
	unsigned int cpu;

	int nr_cpus = num_online_cpus();
	int max_cpus = pm_qos_request(PM_QOS_MAX_ONLINE_CPUS) ? : 4;
	int min_cpus = pm_qos_request(PM_QOS_MIN_ONLINE_CPUS);

	if (is_lp_cluster())
		return;

	if (nr_cpus < min_cpus) {
		up = true;
		count = min_cpus - nr_cpus;
	} else if (nr_cpus > max_cpus && max_cpus >= min_cpus) {
		count = nr_cpus - max_cpus;
	}

	for (;count > 0; count--) {
		if (up) {
			cpu = cpumask_next_zero(0, cpu_online_mask);
			if (cpu < nr_cpu_ids)
				cpu_up(cpu);
			else
				break;
		} else {
			cpu = cpumask_next(0, cpu_online_mask);
			if (cpu < nr_cpu_ids)
				cpu_down(cpu);
			else
				break;
		}
	}
}

static int min_cpus_notify(struct notifier_block *nb, unsigned long n, void *p)
{
	mutex_lock(tegra3_cpu_lock);

	if ((n >= 1) && is_lp_cluster()) {
		/* make sure cpu rate is within g-mode range before switching */
		unsigned int speed = max(
			tegra_getspeed(0), clk_get_min_rate(cpu_g_clk) / 1000);
		tegra_update_cpu_speed(speed);

		clk_set_parent(cpu_clk, cpu_g_clk);
	}

	tegra_cpu_set_speed_cap(NULL);
	mutex_unlock(tegra3_cpu_lock);

	schedule_work(&minmax_work);

	return NOTIFY_OK;
}

static int max_cpus_notify(struct notifier_block *nb, unsigned long n, void *p)
{
	if (n < num_online_cpus())
		schedule_work(&minmax_work);

	return NOTIFY_OK;
}

void tegra_auto_hotplug_governor(unsigned int cpu_freq, bool suspend)
{
	if (!is_g_cluster_present())
		return;

	if (cpq_state == TEGRA_CPQ_DISABLED)
		return;

	if (suspend) {
		cpq_state = TEGRA_CPQ_IDLE;

		/* Switch to G-mode if suspend rate is high enough */
		if (is_lp_cluster() && (cpu_freq >= idle_bottom_freq)) {
			clk_set_parent(cpu_clk, cpu_g_clk);
		}
		return;
	}

	if (is_lp_cluster() && pm_qos_request(PM_QOS_MIN_ONLINE_CPUS) >= 2) {
		if (cpq_state != TEGRA_CPQ_SWITCH_TO_G) {
			/* Force switch */
			cpq_state = TEGRA_CPQ_SWITCH_TO_G;
			queue_delayed_work(
				cpuquiet_wq, &cpuquiet_work, up_delay);
		}
		return;
	}

	if (is_lp_cluster() && (cpu_freq >= idle_top_freq || no_lp)) {
		cpq_state = TEGRA_CPQ_SWITCH_TO_G;
		queue_delayed_work(cpuquiet_wq, &cpuquiet_work, up_delay);
	} else if (!is_lp_cluster() && !no_lp &&
		   cpu_freq <= idle_bottom_freq) {
		cpq_state = TEGRA_CPQ_SWITCH_TO_LP;
		queue_delayed_work(cpuquiet_wq, &cpuquiet_work, down_delay);
	} else {
		cpq_state = TEGRA_CPQ_IDLE;
	}
}

static struct notifier_block min_cpus_notifier = {
	.notifier_call = min_cpus_notify,
};

static struct notifier_block max_cpus_notifier = {
	.notifier_call = max_cpus_notify,
};

int tegra_auto_hotplug_init(struct mutex *cpu_lock)
{
	/*
	 * Not bound to the issuer CPU (=> high-priority), has rescue worker
	 * task, single-threaded, freezable.
	 */
	cpuquiet_wq = alloc_workqueue(
		"cpuquiet", WQ_UNBOUND | WQ_RESCUER | WQ_FREEZABLE, 1);

	if (!cpuquiet_wq)
		return -ENOMEM;

	INIT_DELAYED_WORK(&cpuquiet_work, tegra_cpuquiet_work_func);
	INIT_WORK(&minmax_work, min_max_constraints_workfunc);

	cpu_clk = clk_get_sys(NULL, "cpu");
	cpu_g_clk = clk_get_sys(NULL, "cpu_g");
	cpu_lp_clk = clk_get_sys(NULL, "cpu_lp");

	if (IS_ERR(cpu_clk) || IS_ERR(cpu_g_clk) || IS_ERR(cpu_lp_clk))
		return -ENOENT;

	idle_top_freq = clk_get_max_rate(cpu_lp_clk) / 1000;
	idle_bottom_freq = clk_get_min_rate(cpu_g_clk) / 1000;

	up_delay = msecs_to_jiffies(UP_DELAY_MS);
	down_delay = msecs_to_jiffies(DOWN_DELAY_MS);
	cpumask_clear(&cr_online_requests);
	tegra3_cpu_lock = cpu_lock;

	cpq_state = INITIAL_STATE;

	pr_info("Tegra cpuquiet initialized: %s\n",
		(cpq_state == TEGRA_CPQ_DISABLED) ? "disabled" : "enabled");

	if (pm_qos_add_notifier(PM_QOS_MIN_ONLINE_CPUS, &min_cpus_notifier))
		pr_err("%s: Failed to register min cpus PM QoS notifier\n",
			__func__);
	if (pm_qos_add_notifier(PM_QOS_MAX_ONLINE_CPUS, &max_cpus_notifier))
		pr_err("%s: Failed to register max cpus PM QoS notifier\n",
			__func__);

	return cpuquiet_register_driver(&tegra_cpuquiet_driver);
}

void tegra_auto_hotplug_exit(void)
{
	destroy_workqueue(cpuquiet_wq);
        cpuquiet_unregister_driver(&tegra_cpuquiet_driver);
}
