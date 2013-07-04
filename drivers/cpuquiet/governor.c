/*
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
 *
 */

#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/cpuquiet.h>

#include "cpuquiet.h"

LIST_HEAD(cpuquiet_governors);
struct cpuquiet_governor *cpuquiet_curr_governor;
static struct cpuquiet_governor* default_gov = NULL;

extern int cpq_auto_hotplug_init(void);

/*
 * only after register we can check for the kernel
 * config default governor and set it
 */
void cpuquiet_set_default_governor(struct cpuquiet_governor* gov)
{
#ifdef CONFIG_CPUQUIET_DEFAULT_GOV_BALANCED
	if (!strnicmp("balanced", gov->name, CPUQUIET_NAME_LEN))
		default_gov = gov;
#endif
#ifdef CONFIG_CPUQUIET_DEFAULT_GOV_RQ_STATS
	if (!strnicmp("rq_stats", gov->name, CPUQUIET_NAME_LEN))
		default_gov = gov;
#endif
#ifdef CONFIG_CPUQUIET_DEFAULT_GOV_RUNNABLE
	if (!strnicmp("runnable", gov->name, CPUQUIET_NAME_LEN))
		default_gov = gov;
#endif
#ifdef CONFIG_CPUQUIET_DEFAULT_GOV_MAKO_HOTPLUG
	if (!strnicmp("mako_hotplug", gov->name, CPUQUIET_NAME_LEN))
		default_gov = gov;
#endif
#ifdef CONFIG_CPUQUIET_DEFAULT_GOV_LOAD_STATS
	if (!strnicmp("load_stats", gov->name, CPUQUIET_NAME_LEN))
		default_gov = gov;
#endif
	if (default_gov != NULL)
		cpuquiet_switch_governor(default_gov); 
}

struct cpuquiet_governor *cpuquiet_get_first_governor(void)
{
	if (!list_empty(&cpuquiet_governors))
		return list_entry(cpuquiet_governors.next,
					struct cpuquiet_governor,
					governor_list);
	else
		return NULL;
}

struct cpuquiet_governor *cpuquiet_find_governor(const char *str)
{
	struct cpuquiet_governor *gov;

	list_for_each_entry(gov, &cpuquiet_governors, governor_list)
		if (!strnicmp(str, gov->name, CPUQUIET_NAME_LEN))
			return gov;

	return NULL;
}

int cpuquiet_switch_governor(struct cpuquiet_governor *gov)
{
	int err = 0;

	if (cpuquiet_curr_governor) {
		if (cpuquiet_curr_governor->stop)
			cpuquiet_curr_governor->stop();
		module_put(cpuquiet_curr_governor->owner);
	}

	cpuquiet_curr_governor = gov;

	if (gov) {
		if (!try_module_get(cpuquiet_curr_governor->owner))
			return -EINVAL;
		if (gov->start)
			err = gov->start();
		if (!err)
			cpuquiet_curr_governor = gov;
	}

	return err;
}

int __init cpuquiet_register_governor(struct cpuquiet_governor *gov)
{
	int ret = -EEXIST;

	if (!gov)
		return -EINVAL;

	// if there is at least one governor start hotplugging
	if (!cpuquiet_get_driver())
		cpq_auto_hotplug_init();
	
	mutex_lock(&cpuquiet_lock);
	if (cpuquiet_find_governor(gov->name) == NULL) {
		ret = 0;
		list_add_tail(&gov->governor_list, &cpuquiet_governors);
		if (!cpuquiet_curr_governor && cpuquiet_get_driver())
			// if this is the default gov switch to it
			cpuquiet_set_default_governor(gov);
	}
	mutex_unlock(&cpuquiet_lock);

	return ret;
}

void cpuquiet_unregister_governor(struct cpuquiet_governor *gov)
{
	if (!gov)
		return;

	mutex_lock(&cpuquiet_lock);
	if (cpuquiet_curr_governor == gov)
		cpuquiet_switch_governor(NULL);
	list_del(&gov->governor_list);
	mutex_unlock(&cpuquiet_lock);
}

void cpuquiet_device_busy(void)
{
	if (cpuquiet_curr_governor &&
			cpuquiet_curr_governor->device_busy_notification)
		cpuquiet_curr_governor->device_busy_notification();
}

void cpuquiet_device_free(void)
{
	if (cpuquiet_curr_governor &&
			cpuquiet_curr_governor->device_free_notification)
		cpuquiet_curr_governor->device_free_notification();
}
