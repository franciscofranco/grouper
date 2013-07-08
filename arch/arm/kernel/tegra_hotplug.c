/*
 * Copyright (c) 2013, Francisco Franco <franciscofranco.1990@gmail.com>. 
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
 * Simple no bullshit hot[un]plug driver for SMP
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/cpu.h>
#include <linux/workqueue.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/earlysuspend.h>
#include <linux/rq_stats.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/hotplug.h>

#define DEFAULT_FIRST_LEVEL 60
#define DEFAULT_CORE_ONLINE_BOOST 760000
#define DEFAULT_CORES_ON_TOUCH 2
#define HIGH_LOAD_COUNTER 20

struct cpu_stats
{
    unsigned int total_cpus;
    unsigned int default_first_level;
    unsigned int suspend_frequency;
    unsigned int cores_on_touch;
};

static struct cpu_stats stats;
static struct workqueue_struct *wq;
static struct delayed_work decide_hotplug;

int counter;


static void scale_interactive_tunables(unsigned int above_hispeed_delay,
    unsigned int timer_rate, 
    unsigned int min_sample_time)
{
    scale_above_hispeed_delay(above_hispeed_delay);
    scale_timer_rate(timer_rate);
    scale_min_sample_time(min_sample_time);
}


static void decide_hotplug_func(struct work_struct *work)
{
    int cpu;
    int cpu_boost;

    if (report_load_at_max_freq() >= stats.default_first_level)
    {
        if (likely(counter < HIGH_LOAD_COUNTER))
            counter += 2;
    }

    else
    {
        if (counter > 0)
            counter--;
    }

    if (unlikely(is_touching && num_online_cpus() < stats.cores_on_touch))
    {
        for_each_possible_cpu(cpu_boost)
        {
            if (!cpu_online(cpu_boost) && cpu_boost < stats.cores_on_touch) 
            {
                cpu_up(cpu_boost);
            }
        }
    }

    if (counter >= 10) 
    {
        for_each_possible_cpu(cpu) 
        {
            if (!cpu_online(cpu)) 
            {
                cpu_up(cpu);
            }
        }
        scale_interactive_tunables(0, 10000, 80000);
    }

    else
    {
        if (num_online_cpus() > 2 && !is_touching)
        {
            for_each_online_cpu(cpu) 
            {
                if (cpu > 1) 
                {
                    cpu_down(cpu);
                }
            }
            scale_interactive_tunables(15000, 30000, 45000);
        } 
    }

    queue_delayed_work(wq, &decide_hotplug, msecs_to_jiffies(HZ));
}

static void tegra_hotplug_early_suspend(struct early_suspend *handler)
{    
    int cpu;

    /* cancel the hotplug work when the screen is off and flush the WQ */
    cancel_delayed_work_sync(&decide_hotplug);
    flush_workqueue(wq);

    pr_info("Early Suspend stopping Hotplug work...\n");
    
    for_each_online_cpu(cpu) 
    {
        if (cpu) 
        {
            cpu_down(cpu);
        }
    }
}

static void tegra_hotplug_late_resume(struct early_suspend *handler)
{  
    int cpu;

    /* online all cores when the screen goes online */
    for_each_possible_cpu(cpu) 
    {
        if (cpu) 
        {
            cpu_up(cpu);
        }
    }

    counter = 0;
    
    pr_info("Late Resume starting Hotplug work...\n");
    queue_delayed_work(wq, &decide_hotplug, HZ);
}

static struct early_suspend tegra_hotplug_suspend =
{
    .level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1,
    .suspend = tegra_hotplug_early_suspend,
    .resume = tegra_hotplug_late_resume,
};

/* sysfs functions for external driver */
void update_first_level(unsigned int level)
{
    stats.default_first_level = level;
}

void update_suspend_frequency(unsigned int freq)
{
    stats.suspend_frequency = freq;
}

void update_cores_on_touch(unsigned int num)
{
    stats.cores_on_touch = num;
}

unsigned int get_first_level()
{
    return stats.default_first_level;
}

unsigned int get_suspend_frequency()
{
    return stats.suspend_frequency;
}

unsigned int get_cores_on_touch()
{
    return stats.cores_on_touch;
}
/* end sysfs functions from external driver */

int __init tegra_hotplug_init(void)
{
    pr_info("Mako Hotplug driver started.\n");
    
    /* init everything here */
    stats.total_cpus = num_present_cpus();
    stats.default_first_level = DEFAULT_FIRST_LEVEL;
    stats.cores_on_touch = DEFAULT_CORES_ON_TOUCH;

    wq = alloc_workqueue("tegra_hotplug_workqueue", 
                    WQ_UNBOUND | WQ_RESCUER | WQ_FREEZABLE, 1);
    
    if (!wq)
        return -ENOMEM;
    
    INIT_DELAYED_WORK(&decide_hotplug, decide_hotplug_func);
    queue_delayed_work(wq, &decide_hotplug, HZ*25);
    
    register_early_suspend(&tegra_hotplug_suspend);
    
    return 0;
}
late_initcall(tegra_hotplug_init);
