/*
 * Copyright (C) 2013 maxwen
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Author: maxwen
 *
 * load_stats based cpuquiet governor
 * uses load status to hotplug cpus
 */

#include <linux/kernel.h>
#include <linux/cpuquiet.h>
#include <linux/cpumask.h>
#include <linux/module.h>
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <linux/cpu.h>
#include <linux/sched.h>
#include <linux/rq_stats.h>
#include <linux/kthread.h>

#define DEBUG 0
#define LOAD_STATS_TAG                       "[LOAD_STATS]: "

// from cpuquiet.c
extern unsigned int cpq_max_cpus(void);
extern unsigned int cpq_min_cpus(void);

typedef enum {
	DISABLED,
	IDLE,
	DOWN,
	UP,
} LOAD_STATS_STATE;

static struct delayed_work load_stats_work;
static struct kobject *load_stats_kobject;

/* configurable parameters */
static unsigned int sample_rate = 70;		/* msec */
static unsigned int start_delay = 20000;
static LOAD_STATS_STATE load_stats_state;
static struct workqueue_struct *load_stats_wq;

static unsigned int load_threshold[8] = {80, 70, 70, 60, 60, 50, 50, 40};
static unsigned int twts_threshold[8] = {140, 0, 140, 190, 140, 190, 0, 190};

extern unsigned int get_rq_info(void);

static u64 input_boost_end_time = 0;
static bool input_boost_running = false;
static unsigned int input_boost_duration = 3 * 70; /* ms */
static unsigned int input_boost_cpus = 2;
static unsigned int input_boost_enabled = true;
static bool input_boost_task_alive = false;
static struct task_struct *input_boost_task;

DEFINE_MUTEX(load_stats_work_lock);

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

static void update_load_stats_state(void)
{
	static bool first_call = true;
	unsigned int load;
	unsigned int nr_cpu_online;
	unsigned int max_cpus = cpq_max_cpus();
	unsigned int min_cpus = cpq_min_cpus();
	static cputime64_t total_time = 0;
	static cputime64_t last_time;
	cputime64_t current_time;
	cputime64_t this_time = 0;
	int index;
		
	if (load_stats_state == DISABLED)
		return;

	if (!rq_data_init_done)
		return;

	current_time = ktime_to_ms(ktime_get());
	if (current_time <= start_delay){
		load_stats_state = IDLE;
		return;
	}

	if (first_call) {
		first_call = false;
	} else {
		this_time = current_time - last_time;
	}
	total_time += this_time;
	load = report_load_at_max_freq();
	nr_cpu_online = num_online_cpus();
	load_stats_state = IDLE;

	if (nr_cpu_online) {
		index = (nr_cpu_online - 1) * 2;
		if ((nr_cpu_online < CONFIG_NR_CPUS) && (load >= load_threshold[index])) {
			if (total_time >= twts_threshold[index]) {
           		if (nr_cpu_online < max_cpus){
#if DEBUG
           			pr_info(LOAD_STATS_TAG "UP load=%d total_time=%lld load_threshold[index]=%d twts_threshold[index]=%d nr_cpu_online=%d min_cpus=%d max_cpus=%d\n", load, total_time, load_threshold[index], twts_threshold[index], nr_cpu_online, min_cpus, max_cpus);
#endif
           	    	load_stats_state = UP;
           	    }
			}
		} else if (load <= load_threshold[index+1]) {
			if (total_time >= twts_threshold[index+1] ) {
           		if ((nr_cpu_online > 1) && (nr_cpu_online > min_cpus)){
#if DEBUG
           			pr_info(LOAD_STATS_TAG "DOWN load=%d total_time=%lld load_threshold[index+1]=%d twts_threshold[index+1]=%d nr_cpu_online=%d min_cpus=%d max_cpus=%d\n", load, total_time, load_threshold[index+1], twts_threshold[index+1], nr_cpu_online, min_cpus, max_cpus);
#endif
                   	load_stats_state = DOWN;
                }
			}
		} else {
			load_stats_state = IDLE;
			total_time = 0;
		}
	} else {
		total_time = 0;
	}

	if (input_boost_running && current_time > input_boost_end_time)
		input_boost_running = false;

	if (input_boost_running){
		if (load_stats_state != UP){
			load_stats_state = IDLE;
#if DEBUG
			pr_info(LOAD_STATS_TAG "IDLE because of input boost\n");
#endif
		}
	}
	
	if (load_stats_state != IDLE)
		total_time = 0;

	last_time = ktime_to_ms(ktime_get());
}

static bool __load_stats_work_func(void)
{
	bool up = false;
	bool sample = false;
	unsigned int cpu = nr_cpu_ids;

	switch (load_stats_state) {
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
			__func__, load_stats_state);
		break;
	}

	if (cpu < nr_cpu_ids) {
		if (up)
			cpuquiet_wake_cpu(cpu);
		else
			cpuquiet_quiesence_cpu(cpu);
	}
	return sample;
}

static void load_stats_work_func(struct work_struct *work)
{
	bool sample = false;

	mutex_lock(&load_stats_work_lock);

	update_load_stats_state();

	sample = __load_stats_work_func();

	if (sample)
		queue_delayed_work(load_stats_wq, &load_stats_work,
				msecs_to_jiffies(sample_rate));

	mutex_unlock(&load_stats_work_lock);
}

static int load_stats_boost_task(void *data) {
	unsigned int nr_cpu_online;
	int i;

	while (1) {
		set_current_state(TASK_INTERRUPTIBLE);
		schedule();

		if (kthread_should_stop())
			break;

		set_current_state(TASK_RUNNING);

		if (input_boost_running)
			continue;

		mutex_lock(&load_stats_work_lock);
		
		input_boost_running = true;
			
		/* do boost work */
		nr_cpu_online = num_online_cpus();
		if (nr_cpu_online < input_boost_cpus){
			for (i = nr_cpu_online; i < input_boost_cpus; i++){
				load_stats_state = UP;
#if DEBUG
				pr_info(LOAD_STATS_TAG "UP because of input boost\n");
#endif
				__load_stats_work_func();
			}
		}
		input_boost_end_time = ktime_to_ms(ktime_get()) + input_boost_duration;
			
		mutex_unlock(&load_stats_work_lock);
	}

#if DEBUG
	pr_info(LOAD_STATS_TAG "%s: input_boost_thread stopped\n", __func__);
#endif

	return 0;
}

static ssize_t show_twts_threshold(struct cpuquiet_attribute *cattr, char *buf)
{
	char *out = buf;
		
	out += sprintf(out, "%u %u %u %u %u %u %u %u\n", twts_threshold[0], twts_threshold[1], twts_threshold[2], twts_threshold[3], 
	    twts_threshold[4], twts_threshold[5], twts_threshold[6], twts_threshold[7]);

	return out - buf;
}

static ssize_t store_twts_threshold(struct cpuquiet_attribute *cattr,
					const char *buf, size_t count)
{
	int ret;
	int i;
    unsigned int user_twts_threshold[8] = {0, 0, 0, 0, 0, 0, 0, 0};

	ret = sscanf(buf, "%u %u %u %u %u %u %u %u", &user_twts_threshold[0], &user_twts_threshold[1], &user_twts_threshold[2] , &user_twts_threshold[3],
	    &user_twts_threshold[4], &user_twts_threshold[5], &user_twts_threshold[6], &user_twts_threshold[7]);

	if (ret < 8)
		return -EINVAL;

	for (i = 0; i < 8; i++)
		if (user_twts_threshold[i] < 0)
			return -EINVAL;

	for (i = 0; i < 8; i++)
		twts_threshold[i]=user_twts_threshold[i];
            
	return count;
}

static ssize_t show_load_threshold(struct cpuquiet_attribute *cattr, char *buf)
{
	char *out = buf;
		
	out += sprintf(out, "%u %u %u %u %u %u %u %u\n", load_threshold[0], load_threshold[1], load_threshold[2], load_threshold[3], 
	    load_threshold[4], load_threshold[5], load_threshold[6], load_threshold[7]);

	return out - buf;
}

static ssize_t store_load_threshold(struct cpuquiet_attribute *cattr,
					const char *buf, size_t count)
{
	int ret;
	int i;
    unsigned int user_load_threshold[8] = {0, 0, 0, 0, 0, 0, 0, 0};

	ret = sscanf(buf, "%u %u %u %u %u %u %u %u", &user_load_threshold[0], &user_load_threshold[1], &user_load_threshold[2] , &user_load_threshold[3],
	    &user_load_threshold[4], &user_load_threshold[5], &user_load_threshold[6], &user_load_threshold[7]);

	if (ret < 8)
		return -EINVAL;

	for (i = 0; i < 8; i++)
		if (user_load_threshold[i] < 0 || user_load_threshold[i] > 100)
			return -EINVAL;

	for (i = 0; i < 8; i++)
		load_threshold[i]=user_load_threshold[i];
            
	return count;
}

CPQ_BASIC_ATTRIBUTE(sample_rate, 0644, uint);
CPQ_BASIC_ATTRIBUTE(input_boost_enabled, 0644, uint);
CPQ_BASIC_ATTRIBUTE(input_boost_cpus, 0644, uint);
CPQ_BASIC_ATTRIBUTE(input_boost_duration, 0644, uint);
CPQ_ATTRIBUTE_CUSTOM(twts_threshold, 0644, show_twts_threshold, store_twts_threshold);
CPQ_ATTRIBUTE_CUSTOM(load_threshold, 0644, show_load_threshold, store_load_threshold);

static struct attribute *load_stats_attributes[] = {
	&sample_rate_attr.attr,
	&input_boost_enabled_attr.attr,
	&input_boost_cpus_attr.attr,
	&input_boost_duration_attr.attr,
	&twts_threshold_attr.attr,
	&load_threshold_attr.attr,
	NULL,
};

static const struct sysfs_ops load_stats_sysfs_ops = {
	.show = cpuquiet_auto_sysfs_show,
	.store = cpuquiet_auto_sysfs_store,
};

static struct kobj_type ktype_runnables = {
	.sysfs_ops = &load_stats_sysfs_ops,
	.default_attrs = load_stats_attributes,
};

static int load_stats_sysfs(void)
{
	int err;

	load_stats_kobject = kzalloc(sizeof(*load_stats_kobject),
				GFP_KERNEL);

	if (!load_stats_kobject)
		return -ENOMEM;

	err = cpuquiet_kobject_init(load_stats_kobject, &ktype_runnables,
				"load_stats");

	if (err)
		kfree(load_stats_kobject);

	return err;
}

static void load_stats_device_busy(void)
{
#if DEBUG
	pr_info(LOAD_STATS_TAG "%s\n", __func__);
#endif
	if (load_stats_state != DISABLED) {
		load_stats_state = DISABLED;
		cancel_delayed_work_sync(&load_stats_work);
	}
}

static void load_stats_device_free(void)
{
#if DEBUG
	pr_info(LOAD_STATS_TAG "%s\n", __func__);
#endif
	if (load_stats_state == DISABLED) {
		load_stats_state = IDLE;
		load_stats_work_func(NULL);
	}
}

static void load_stats_touch_event(void)
{	
	if (input_boost_enabled && !input_boost_running){
		if (input_boost_task_alive)
			wake_up_process(input_boost_task);
		
	}
}

static void load_stats_stop(void)
{
	load_stats_state = DISABLED;
	cancel_delayed_work_sync(&load_stats_work);

	enable_rq_load_calc(false);
	
	if (input_boost_task_alive)
		kthread_stop(input_boost_task);

	destroy_workqueue(load_stats_wq);
	kobject_put(load_stats_kobject);
}

static int load_stats_start(void)
{
	int err;
	struct sched_param param = { .sched_priority = 1 };
	
	err = load_stats_sysfs();
	if (err)
		return err;

	load_stats_wq = alloc_workqueue("cpuquiet-load_stats",
			WQ_UNBOUND | WQ_MEM_RECLAIM | WQ_HIGHPRI, 1);
	if (!load_stats_wq)
		return -ENOMEM;

	INIT_DELAYED_WORK(&load_stats_work, load_stats_work_func);

	enable_rq_load_calc(true);

	input_boost_task = kthread_create (
			load_stats_boost_task,
					NULL,
					"cpuquiet_input_boost_task"
			);

	if (IS_ERR(input_boost_task))
		pr_err(LOAD_STATS_TAG "%s: failed to create input boost task\n", __func__);
	else {
		sched_setscheduler_nocheck(input_boost_task, SCHED_RR, &param);
		get_task_struct(input_boost_task);
		input_boost_task_alive = true;
#if DEBUG
		pr_info(LOAD_STATS_TAG "%s: input boost task created\n", __func__);
#endif
	}
	
	load_stats_state = IDLE;
	load_stats_work_func(NULL);

	return 0;
}

struct cpuquiet_governor load_stats_governor = {
	.name		   	  = "load_stats",
	.start			  = load_stats_start,
	.device_free_notification = load_stats_device_free,
	.device_busy_notification = load_stats_device_busy,
	.stop			  = load_stats_stop,
	.touch_event_notification = load_stats_touch_event,
	.owner		   	  = THIS_MODULE,
};

static int __init init_load_stats(void)
{
	return cpuquiet_register_governor(&load_stats_governor);
}

static void __exit exit_load_stats(void)
{
	cpuquiet_unregister_governor(&load_stats_governor);
}

MODULE_LICENSE("GPL");
module_init(init_load_stats);
module_exit(exit_load_stats);
