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
#include <linux/input.h>
#include <linux/slab.h>

#include "cpuquiet.h"

LIST_HEAD(cpuquiet_governors);
struct cpuquiet_governor *cpuquiet_curr_governor;
static struct cpuquiet_governor* default_gov = NULL;

extern int cpq_auto_hotplug_init(void);
static int cpuquiet_input_init(void);
static bool input_dev_connected = false;

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
	
	if (!input_dev_connected)
		cpuquiet_input_init();

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

void cpuquiet_touch_event(void)
{
	if (cpuquiet_curr_governor &&
			cpuquiet_curr_governor->touch_event_notification)
		cpuquiet_curr_governor->touch_event_notification();
}

static void cpuquiet_input_event(struct input_handle *handle, unsigned int type,
		unsigned int code, int value) {
	if (type == EV_SYN && code == SYN_REPORT) {
		cpuquiet_touch_event();
	}
}

static int input_dev_filter(const char* input_dev_name) {
	int ret = 0;
	if (strstr(input_dev_name, "touchscreen")
			|| strstr(input_dev_name, "-ts")
			|| strstr(input_dev_name, "-keypad")
			|| strstr(input_dev_name, "-nav")
			|| strstr(input_dev_name, "-oj")) {
	} else {
		ret = 1;
	}
	return ret;
}

static int cpuquiet_input_connect(struct input_handler *handler,
		struct input_dev *dev, const struct input_device_id *id) {
	struct input_handle *handle;
	int error;

	/* filter out those input_dev that we don't care */
	if (input_dev_filter(dev->name))
		return 0;

	pr_info(CPUQUIET_TAG "%s input connect to %s\n", __func__, dev->name);

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "cpuquiet";

	error = input_register_handle(handle);
	if (error)
		goto err2;

	error = input_open_device(handle);
	if (error)
		goto err1;

	input_dev_connected = true;
	return 0;
	err1: input_unregister_handle(handle);
	err2: kfree(handle);
	return error;
}

static void cpuquiet_input_disconnect(struct input_handle *handle) {
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id cpuquiet_ids[] = { { .driver_info = 1 }, { }, };

static struct input_handler cpuquiet_input_handler = { 
	.event = cpuquiet_input_event,
	.connect = cpuquiet_input_connect, 
	.disconnect = cpuquiet_input_disconnect,
	.name = "cpuquiet", 
	.id_table = cpuquiet_ids, 
	};

static int cpuquiet_input_init(void)
{
	return input_register_handler(&cpuquiet_input_handler);
}

