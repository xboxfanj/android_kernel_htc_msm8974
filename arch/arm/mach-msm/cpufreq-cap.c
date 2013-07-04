/* arch/arm/mach-msm/cpufreq-cap.c
 *
 * Screen off freq cap drivers for MSM
 *
 * Copyright (C) 2013 maxwen
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 */

#include <linux/earlysuspend.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/cpufreq.h>
#include <linux/completion.h>
#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/sched.h>
#include <linux/suspend.h>
#include <mach/cpufreq.h>

#define CPUFREQ_CAP_TAG                       "[CPUFREQ_CAP]: "

static struct kobject *auto_sysfs_kobject;
#define DEFAULT_SCREEN_OFF_FREQ_CAP 702000
static unsigned int screen_off_max_freq = DEFAULT_SCREEN_OFF_FREQ_CAP;
static bool screen_off_cap = true;
#ifdef CONFIG_HAS_EARLYSUSPEND
struct early_suspend cpufreq_early_suspender;
#endif
static bool screen_off_cap_active = false;

#ifdef CONFIG_HAS_EARLYSUSPEND
static void cpufreq_early_suspend(struct early_suspend *h)
{
	int cpu;
	
	if (screen_off_cap){
		pr_info(CPUFREQ_CAP_TAG "%s: limit freq to %d\n", __func__, screen_off_max_freq);
		screen_off_cap_active = true;

        for_each_possible_cpu(cpu)
			msm_cpufreq_set_freq_limits(cpu, MSM_CPUFREQ_NO_LIMIT, screen_off_max_freq);
	}
}

static void cpufreq_late_resume(struct early_suspend *h)
{
	int cpu;
	
	if (screen_off_cap){
		pr_info(CPUFREQ_CAP_TAG "%s: release limit freq to %d\n", __func__, screen_off_max_freq);
		screen_off_cap_active = false;

        for_each_possible_cpu(cpu)
        	msm_cpufreq_set_freq_limits(cpu, MSM_CPUFREQ_NO_LIMIT, MSM_CPUFREQ_NO_LIMIT);
	}
}
#endif

static ssize_t show_screen_off_cap(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	char *out = buf;
		
	out += sprintf(out, "%d\n", screen_off_cap);

	return out - buf;
}

static ssize_t store_screen_off_cap(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned int n;
		
	ret = sscanf(buf, "%d", &n);

	if ((ret != 1) || n < 0 || n > 1)
		return -EINVAL;

	screen_off_cap = n;	
	return count;
}

static ssize_t show_screen_off_max_freq(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	char *out = buf;
		
	out += sprintf(out, "%d\n", screen_off_max_freq);

	return out - buf;
}

static ssize_t store_screen_off_max_freq(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned int n;
		
	ret = sscanf(buf, "%d", &n);

	if ((ret != 1))
		return -EINVAL;

	screen_off_max_freq = n;	
	return count;
}


static struct kobj_attribute screen_off_cap_attr =
	__ATTR(screen_off_cap, 0644, show_screen_off_cap, store_screen_off_cap);
static struct kobj_attribute screen_off_max_freq_attr =
	__ATTR(screen_off_max_freq, 0644, show_screen_off_max_freq, store_screen_off_max_freq);

static struct attribute *cpufreq_cap_attr[] = {
	&screen_off_cap_attr.attr,
	&screen_off_max_freq_attr.attr,
	NULL,
};

static struct attribute_group cpufreq_cap_attr_group = {
	.attrs = cpufreq_cap_attr,
};

static int cpufreq_cap_auto_sysfs(void)
{
	int err;

	auto_sysfs_kobject= kobject_create_and_add("cpufreq_cap", kernel_kobj);
	if (!auto_sysfs_kobject)
		return -ENOMEM;


	err = sysfs_create_group(auto_sysfs_kobject, &cpufreq_cap_attr_group);
	if (err)
		kobject_put(auto_sysfs_kobject);

	return err;
}

static int cpufreq_cap_init(void)
{
	cpufreq_cap_auto_sysfs();

#ifdef CONFIG_HAS_EARLYSUSPEND
	// will cap core freq on screen off
	cpufreq_early_suspender.suspend = cpufreq_early_suspend;
	cpufreq_early_suspender.resume = cpufreq_late_resume;
	cpufreq_early_suspender.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 100;
	register_early_suspend(&cpufreq_early_suspender);
#endif
	
	return 0;
}

static void cpufreq_cap_exit(void)
{
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&cpufreq_early_suspender);
#endif
	kobject_put(auto_sysfs_kobject);	
}

module_init(cpufreq_cap_init);
module_exit(cpufreq_cap_exit);
