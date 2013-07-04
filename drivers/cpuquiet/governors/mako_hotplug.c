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
 * Simple no bullshit hot[in]plug driver for SMP
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/cpu.h>
#include <linux/workqueue.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/cpuquiet.h>
#include <linux/rq_stats.h>

// from cpuquiet.c
extern unsigned int cpq_max_cpus(void);
extern unsigned int cpq_min_cpus(void);

typedef enum {
	DISABLED,
	IDLE,
} MAKO_HOTPLUG_STATE;

static MAKO_HOTPLUG_STATE mako_hotplug_state;

#define DEFAULT_FIRST_LEVEL 70

struct cpu_stats
{
    unsigned int total_cpus;
    unsigned int default_first_level;
};

static struct cpu_stats stats;
static struct workqueue_struct *wq;
static struct delayed_work decide_hotplug;

int counter;
int timer = HZ;

static void decide_hotplug_func(struct work_struct *work)
{
    int cpu;

	if (mako_hotplug_state == DISABLED)
		return;

    if (report_load_at_max_freq() >= stats.default_first_level)
    {
        if (likely(counter < 50))    
            counter++;
    }
    else
    {
        if (counter > 0)
            counter--;
    }
    
    if (counter >= 10) 
    {
        for_each_possible_cpu(cpu) 
        {
            if (!cpu_online(cpu) && num_online_cpus() < cpq_max_cpus()) 
            {
                cpuquiet_wake_cpu(cpu);
            }
        }
        timer = 250;
    }

    else
    {
        if (num_online_cpus() > 1)
        {
            for_each_online_cpu(cpu) 
            {
                if (cpu > 1 && num_online_cpus() > cpq_min_cpus()) 
                {
                    cpuquiet_quiesence_cpu(cpu);
                }
            }
            timer = HZ;
        } 
    }

    queue_delayed_work(wq, &decide_hotplug, msecs_to_jiffies(timer));
}

static void mako_hotplug_device_busy(void)
{
	pr_info("Mako Hotplug driver busy.\n");
	if (mako_hotplug_state != DISABLED) {
		mako_hotplug_state = DISABLED;
		cancel_delayed_work_sync(&decide_hotplug);
	}
}

static void mako_hotplug_device_free(void)
{
	pr_info("Mako Hotplug driver free.\n");
	if (mako_hotplug_state == DISABLED) {
		mako_hotplug_state = IDLE;
    	queue_delayed_work(wq, &decide_hotplug, HZ*25);
	}
}

static void mako_hotplug_stop(void)
{
	pr_info("Mako Hotplug driver stoped.\n");
    mako_hotplug_state = DISABLED;	
	cancel_delayed_work_sync(&decide_hotplug);
	destroy_workqueue(wq);
}

static int mako_hotplug_start(void)
{
	pr_info("Mako Hotplug driver started.\n");
    
    /* init everything here */
    stats.total_cpus = num_present_cpus();
    stats.default_first_level = DEFAULT_FIRST_LEVEL;

    wq = alloc_workqueue("mako_hotplug_workqueue", 
                    WQ_UNBOUND | WQ_RESCUER | WQ_FREEZABLE, 1);
    
    if (!wq)
        return -ENOMEM;
 
    mako_hotplug_state = IDLE;
        
    INIT_DELAYED_WORK(&decide_hotplug, decide_hotplug_func);
    queue_delayed_work(wq, &decide_hotplug, HZ*25);
    
    return 0;
}

struct cpuquiet_governor mako_hotplug_governor = {
	.name		   	  = "mako_hotplug",
	.start			  = mako_hotplug_start,
	.device_free_notification = mako_hotplug_device_free,
	.device_busy_notification = mako_hotplug_device_busy,
	.stop			  = mako_hotplug_stop,
	.owner		   	  = THIS_MODULE,
};

static int __init init_mako_hotplug(void)
{
	return cpuquiet_register_governor(&mako_hotplug_governor);
}

static void __exit exit_mako_hotplug(void)
{
	cpuquiet_unregister_governor(&mako_hotplug_governor);
}

MODULE_LICENSE("GPL");
module_init(init_mako_hotplug);
module_exit(exit_mako_hotplug);
