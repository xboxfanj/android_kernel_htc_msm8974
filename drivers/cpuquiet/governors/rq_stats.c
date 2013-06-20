/*
 * Copyright (C) 2013 maxwen
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Author: maxwen
 *
 * rq_stats based cpuquiet governor
 * based on tegra_mpdecision.c
 * Copyright (c) 2012, Dennis Rassmann <showp1984@gmail.com>
 */

#include <linux/kernel.h>
#include <linux/cpuquiet.h>
#include <linux/cpumask.h>
#include <linux/module.h>
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <linux/cpu.h>
#include <linux/sched.h>
#include <linux/cpufreq.h>

#define DEBUG 0
#define RQ_STATS_TAG                       "[RQ_STATS]: "

// from cpuquiet.c
extern unsigned int cpq_max_cpus(void);
extern unsigned int cpq_min_cpus(void);

typedef enum {
	DISABLED,
	IDLE,
	DOWN,
	UP,
} RQ_STATS_STATE;

static struct delayed_work rq_stats_work;
static struct kobject *rq_stats_kobject;

/* configurable parameters */
static unsigned int sample_rate = 60;		/* msec */
static unsigned int start_delay = 20000;
static RQ_STATS_STATE rq_stats_state;
static struct workqueue_struct *rq_stats_wq;

static unsigned int NwNs_Threshold[8] = {20, 14, 26, 16, 30, 18, 0, 20};
static unsigned int TwTs_Threshold[8] = {60, 0, 60, 90, 60, 90, 0, 90};

extern unsigned int get_rq_info(void);

DEFINE_MUTEX(rq_stats_work_lock);

static unsigned int get_lightest_loaded_cpu_n(void)
{
	unsigned long min_avg_runnables = ULONG_MAX;
	unsigned int cpu = nr_cpu_ids;
	int i;

	for_each_online_cpu(i) {
		unsigned int nr_runnables = get_avg_nr_running(i);

		if (i > 0 && min_avg_runnables > nr_runnables) {
			cpu = i;
			min_avg_runnables = nr_runnables;
		}
	}

	return cpu;
}

static void update_rq_stats_state(void)
{
	static bool first_call = true;
	unsigned int rq_depth;
	unsigned int nr_cpu_online;
	unsigned int max_cpus = cpq_max_cpus();
	unsigned int min_cpus = cpq_min_cpus();
	static cputime64_t total_time = 0;
	static cputime64_t last_time;
	cputime64_t current_time;
	cputime64_t this_time = 0;
	int index;
		
	if (rq_stats_state == DISABLED)
		return;

	current_time = ktime_to_ms(ktime_get());
	if (current_time <= start_delay){
		rq_stats_state = IDLE;
		return;
	}

	if (first_call) {
		first_call = false;
	} else {
		this_time = current_time - last_time;
	}
	total_time += this_time;
	rq_depth = get_rq_info();
	nr_cpu_online = num_online_cpus();
	rq_stats_state = IDLE;

	if (nr_cpu_online) {
		index = (nr_cpu_online - 1) * 2;
		if ((nr_cpu_online < CONFIG_NR_CPUS) && (rq_depth >= NwNs_Threshold[index])) {
			if (total_time >= TwTs_Threshold[index]) {
            	if (nr_cpu_online < max_cpus){
#if DEBUG
            		pr_info(RQ_STATS_TAG "UP rq_depth=%d total_time=%lld NwNs_Threshold[index]=%d TwTs_Threshold[index]=%d nr_cpu_online=%d min_cpus=%d max_cpus=%d\n", rq_depth, total_time, NwNs_Threshold[index], TwTs_Threshold[index], nr_cpu_online, min_cpus, max_cpus);
#endif
                	rq_stats_state = UP;
                }
			}
		} else if (rq_depth <= NwNs_Threshold[index+1]) {
			if (total_time >= TwTs_Threshold[index+1] ) {
            	if ((nr_cpu_online > 1) && (nr_cpu_online > min_cpus)){
#if DEBUG
            		pr_info(RQ_STATS_TAG "DOWN rq_depth=%d total_time=%lld NwNs_Threshold[index+1]=%d TwTs_Threshold[index+1]=%d nr_cpu_online=%d min_cpus=%d max_cpus=%d\n", rq_depth, total_time, NwNs_Threshold[index+1], TwTs_Threshold[index+1], nr_cpu_online, min_cpus, max_cpus);
#endif
                   	rq_stats_state = DOWN;
                }
			}
		} else {
			rq_stats_state = IDLE;
			total_time = 0;
		}
	} else {
		total_time = 0;
	}

	if (rq_stats_state != IDLE) {
		total_time = 0;
	}

	last_time = ktime_to_ms(ktime_get());
}

static void rq_stats_work_func(struct work_struct *work)
{
	bool up = false;
	bool sample = false;
	unsigned int cpu = nr_cpu_ids;

	mutex_lock(&rq_stats_work_lock);

	update_rq_stats_state();

	switch (rq_stats_state) {
	case DISABLED:
		break;
	case IDLE:
		sample = true;
		break;
	case UP:
		cpu = cpumask_next_zero(0, cpu_online_mask);
		up = true;
		sample = true;
		break;
	case DOWN:
		cpu = get_lightest_loaded_cpu_n();
		sample = true;
		break;
	default:
		pr_err("%s: invalid cpuquiet runnable governor state %d\n",
			__func__, rq_stats_state);
		break;
	}

	if (sample)
		queue_delayed_work(rq_stats_wq, &rq_stats_work,
					msecs_to_jiffies(sample_rate));

	if (cpu < nr_cpu_ids) {
		if (up)
			cpuquiet_wake_cpu(cpu);
		else
			cpuquiet_quiesence_cpu(cpu);
	}

	mutex_unlock(&rq_stats_work_lock);
}

#define show_one_twts(file_name, arraypos)                              \
static ssize_t show_##file_name                                         \
(struct kobject *kobj, struct attribute *attr, char *buf)               \
{                                                                       \
	return sprintf(buf, "%u\n", TwTs_Threshold[arraypos]);          \
}
show_one_twts(twts_threshold_0, 0);
show_one_twts(twts_threshold_1, 1);
show_one_twts(twts_threshold_2, 2);
show_one_twts(twts_threshold_3, 3);
show_one_twts(twts_threshold_4, 4);
show_one_twts(twts_threshold_5, 5);
show_one_twts(twts_threshold_6, 6);
show_one_twts(twts_threshold_7, 7);

#define store_one_twts(file_name, arraypos)                             \
static ssize_t store_##file_name                                        \
(struct kobject *a, struct attribute *b, const char *buf, size_t count) \
{                                                                       \
	unsigned int input;                                             \
	int ret;                                                        \
	ret = sscanf(buf, "%u", &input);                                \
	if (ret != 1)                                                   \
		return -EINVAL;                                         \
	TwTs_Threshold[arraypos] = input;                               \
	return count;                                                   \
}                                                                       \
define_one_global_rw(file_name);                                                                       
store_one_twts(twts_threshold_0, 0);
store_one_twts(twts_threshold_1, 1);
store_one_twts(twts_threshold_2, 2);
store_one_twts(twts_threshold_3, 3);
store_one_twts(twts_threshold_4, 4);
store_one_twts(twts_threshold_5, 5);
store_one_twts(twts_threshold_6, 6);
store_one_twts(twts_threshold_7, 7);

#define show_one_nwns(file_name, arraypos)                              \
static ssize_t show_##file_name                                         \
(struct kobject *kobj, struct attribute *attr, char *buf)               \
{                                                                       \
	return sprintf(buf, "%u\n", NwNs_Threshold[arraypos]);          \
}                                                                       
show_one_nwns(nwns_threshold_0, 0);
show_one_nwns(nwns_threshold_1, 1);
show_one_nwns(nwns_threshold_2, 2);
show_one_nwns(nwns_threshold_3, 3);
show_one_nwns(nwns_threshold_4, 4);
show_one_nwns(nwns_threshold_5, 5);
show_one_nwns(nwns_threshold_6, 6);
show_one_nwns(nwns_threshold_7, 7);

#define store_one_nwns(file_name, arraypos)                             \
static ssize_t store_##file_name                                        \
(struct kobject *a, struct attribute *b, const char *buf, size_t count) \
{                                                                       \
	unsigned int input;                                             \
	int ret;                                                        \
	ret = sscanf(buf, "%u", &input);                                \
	if (ret != 1)                                                   \
		return -EINVAL;                                         \
	NwNs_Threshold[arraypos] = input;                               \
	return count;                                                   \
}                                                                       \
define_one_global_rw(file_name);
store_one_nwns(nwns_threshold_0, 0);
store_one_nwns(nwns_threshold_1, 1);
store_one_nwns(nwns_threshold_2, 2);
store_one_nwns(nwns_threshold_3, 3);
store_one_nwns(nwns_threshold_4, 4);
store_one_nwns(nwns_threshold_5, 5);
store_one_nwns(nwns_threshold_6, 6);
store_one_nwns(nwns_threshold_7, 7);

CPQ_BASIC_ATTRIBUTE(sample_rate, 0644, uint);

static struct attribute *rq_stats_attributes[] = {
	&sample_rate_attr.attr,
	&twts_threshold_0.attr,
	&twts_threshold_1.attr,
	&twts_threshold_2.attr,
	&twts_threshold_3.attr,
	&twts_threshold_4.attr,
	&twts_threshold_5.attr,
	&twts_threshold_6.attr,
	&twts_threshold_7.attr,
	&nwns_threshold_0.attr,
	&nwns_threshold_1.attr,
	&nwns_threshold_2.attr,
	&nwns_threshold_3.attr,
	&nwns_threshold_4.attr,
	&nwns_threshold_5.attr,
	&nwns_threshold_6.attr,
	&nwns_threshold_7.attr,
	NULL,
};

static const struct sysfs_ops rq_stats_sysfs_ops = {
	.show = cpuquiet_auto_sysfs_show,
	.store = cpuquiet_auto_sysfs_store,
};

static struct kobj_type ktype_runnables = {
	.sysfs_ops = &rq_stats_sysfs_ops,
	.default_attrs = rq_stats_attributes,
};

static int rq_stats_sysfs(void)
{
	int err;

	rq_stats_kobject = kzalloc(sizeof(*rq_stats_kobject),
				GFP_KERNEL);

	if (!rq_stats_kobject)
		return -ENOMEM;

	err = cpuquiet_kobject_init(rq_stats_kobject, &ktype_runnables,
				"rq_stats");

	if (err)
		kfree(rq_stats_kobject);

	return err;
}

static void rq_stats_device_busy(void)
{
#if DEBUG
	pr_info(RQ_STATS_TAG "rq_stats_device_busy");
#endif
	if (rq_stats_state != DISABLED) {
		rq_stats_state = DISABLED;
		cancel_delayed_work_sync(&rq_stats_work);
	}
}

static void rq_stats_device_free(void)
{
#if DEBUG
	pr_info(RQ_STATS_TAG "rq_stats_device_free");
#endif
	if (rq_stats_state == DISABLED) {
		rq_stats_state = IDLE;
		rq_stats_work_func(NULL);
	}
}

static void rq_stats_stop(void)
{
	rq_stats_state = DISABLED;
	cancel_delayed_work_sync(&rq_stats_work);
	destroy_workqueue(rq_stats_wq);
	kobject_put(rq_stats_kobject);
}

static int rq_stats_start(void)
{
	int err;

	err = rq_stats_sysfs();
	if (err)
		return err;

	rq_stats_wq = alloc_workqueue("cpuquiet-rq_stats",
			WQ_UNBOUND | WQ_RESCUER | WQ_FREEZABLE, 1);
	if (!rq_stats_wq)
		return -ENOMEM;

	INIT_DELAYED_WORK(&rq_stats_work, rq_stats_work_func);

	rq_stats_state = IDLE;
	rq_stats_work_func(NULL);

	return 0;
}

struct cpuquiet_governor rq_stats_governor = {
	.name		   	  = "rq_stats",
	.start			  = rq_stats_start,
	.device_free_notification = rq_stats_device_free,
	.device_busy_notification = rq_stats_device_busy,
	.stop			  = rq_stats_stop,
	.owner		   	  = THIS_MODULE,
};

static int __init init_rq_stats(void)
{
	return cpuquiet_register_governor(&rq_stats_governor);
}

static void __exit exit_rq_stats(void)
{
	cpuquiet_unregister_governor(&rq_stats_governor);
}

MODULE_LICENSE("GPL");
module_init(init_rq_stats);
module_exit(exit_rq_stats);
