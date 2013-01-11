

#include <linux/pm_qos.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/kernel.h>

#include <linux/uaccess.h>
#include <linux/export.h>

struct pm_qos_object {
	struct pm_qos_constraints *constraints;
	struct miscdevice pm_qos_power_miscdev;
	char *name;
};

static DEFINE_SPINLOCK(pm_qos_lock);

static struct pm_qos_object null_pm_qos;

static BLOCKING_NOTIFIER_HEAD(cpu_dma_lat_notifier);
static struct pm_qos_constraints cpu_dma_constraints = {
	.list = PLIST_HEAD_INIT(cpu_dma_constraints.list),
	.target_value = PM_QOS_CPU_DMA_LAT_DEFAULT_VALUE,
	.default_value = PM_QOS_CPU_DMA_LAT_DEFAULT_VALUE,
	.type = PM_QOS_MIN,
	.notifiers = &cpu_dma_lat_notifier,
};
static struct pm_qos_object cpu_dma_pm_qos = {
	.constraints = &cpu_dma_constraints,
	.name = "cpu_dma_latency",
};

static BLOCKING_NOTIFIER_HEAD(network_lat_notifier);
static struct pm_qos_constraints network_lat_constraints = {
	.list = PLIST_HEAD_INIT(network_lat_constraints.list),
	.target_value = PM_QOS_NETWORK_LAT_DEFAULT_VALUE,
	.default_value = PM_QOS_NETWORK_LAT_DEFAULT_VALUE,
	.type = PM_QOS_MIN,
	.notifiers = &network_lat_notifier,
};
static struct pm_qos_object network_lat_pm_qos = {
	.constraints = &network_lat_constraints,
	.name = "network_latency",
};


static BLOCKING_NOTIFIER_HEAD(network_throughput_notifier);
static struct pm_qos_constraints network_tput_constraints = {
	.list = PLIST_HEAD_INIT(network_tput_constraints.list),
	.target_value = PM_QOS_NETWORK_THROUGHPUT_DEFAULT_VALUE,
	.default_value = PM_QOS_NETWORK_THROUGHPUT_DEFAULT_VALUE,
	.type = PM_QOS_MAX,
	.notifiers = &network_throughput_notifier,
};
static struct pm_qos_object network_throughput_pm_qos = {
	.constraints = &network_tput_constraints,
	.name = "network_throughput",
};


static struct pm_qos_object *pm_qos_array[] = {
	&null_pm_qos,
	&cpu_dma_pm_qos,
	&network_lat_pm_qos,
	&network_throughput_pm_qos
};

static ssize_t pm_qos_power_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos);
static ssize_t pm_qos_power_read(struct file *filp, char __user *buf,
		size_t count, loff_t *f_pos);
static int pm_qos_power_open(struct inode *inode, struct file *filp);
static int pm_qos_power_release(struct inode *inode, struct file *filp);

static const struct file_operations pm_qos_power_fops = {
	.write = pm_qos_power_write,
	.read = pm_qos_power_read,
	.open = pm_qos_power_open,
	.release = pm_qos_power_release,
	.llseek = noop_llseek,
};

static inline int pm_qos_get_value(struct pm_qos_constraints *c)
{
	if (plist_head_empty(&c->list))
		return c->default_value;

	switch (c->type) {
	case PM_QOS_MIN:
		return plist_first(&c->list)->prio;

	case PM_QOS_MAX:
		return plist_last(&c->list)->prio;

	default:
		
		BUG();
		return PM_QOS_DEFAULT_VALUE;
	}
}

s32 pm_qos_read_value(struct pm_qos_constraints *c)
{
	return c->target_value;
}

static inline void pm_qos_set_value(struct pm_qos_constraints *c, s32 value)
{
	c->target_value = value;
}

int pm_qos_update_target(struct pm_qos_constraints *c, struct plist_node *node,
			 enum pm_qos_req_action action, int value)
{
	unsigned long flags;
	int prev_value, curr_value, new_value;

	spin_lock_irqsave(&pm_qos_lock, flags);
	prev_value = pm_qos_get_value(c);
	if (value == PM_QOS_DEFAULT_VALUE)
		new_value = c->default_value;
	else
		new_value = value;

	switch (action) {
	case PM_QOS_REMOVE_REQ:
		plist_del(node, &c->list);
		break;
	case PM_QOS_UPDATE_REQ:
		plist_del(node, &c->list);
	case PM_QOS_ADD_REQ:
		plist_node_init(node, new_value);
		plist_add(node, &c->list);
		break;
	default:
		
		;
	}

	curr_value = pm_qos_get_value(c);
	pm_qos_set_value(c, curr_value);

	spin_unlock_irqrestore(&pm_qos_lock, flags);

	if (prev_value != curr_value) {
		blocking_notifier_call_chain(c->notifiers,
					     (unsigned long)curr_value,
					     NULL);
		return 1;
	} else {
		return 0;
	}
}

/**
 * pm_qos_flags_remove_req - Remove device PM QoS flags request.
 * @pqf: Device PM QoS flags set to remove the request from.
 * @req: Request to remove from the set.
 */
static void pm_qos_flags_remove_req(struct pm_qos_flags *pqf,
				    struct pm_qos_flags_request *req)
{
	s32 val = 0;

	list_del(&req->node);
	list_for_each_entry(req, &pqf->list, node)
		val |= req->flags;

	pqf->effective_flags = val;
}

/**
 * pm_qos_update_flags - Update a set of PM QoS flags.
 * @pqf: Set of flags to update.
 * @req: Request to add to the set, to modify, or to remove from the set.
 * @action: Action to take on the set.
 * @val: Value of the request to add or modify.
 *
 * Update the given set of PM QoS flags and call notifiers if the aggregate
 * value has changed.  Returns 1 if the aggregate constraint value has changed,
 * 0 otherwise.
 */
bool pm_qos_update_flags(struct pm_qos_flags *pqf,
			 struct pm_qos_flags_request *req,
			 enum pm_qos_req_action action, s32 val)
{
	unsigned long irqflags;
	s32 prev_value, curr_value;

	spin_lock_irqsave(&pm_qos_lock, irqflags);

	prev_value = list_empty(&pqf->list) ? 0 : pqf->effective_flags;

	switch (action) {
	case PM_QOS_REMOVE_REQ:
		pm_qos_flags_remove_req(pqf, req);
		break;
	case PM_QOS_UPDATE_REQ:
		pm_qos_flags_remove_req(pqf, req);
	case PM_QOS_ADD_REQ:
		req->flags = val;
		INIT_LIST_HEAD(&req->node);
		list_add_tail(&req->node, &pqf->list);
		pqf->effective_flags |= val;
		break;
	default:
		/* no action */
		;
	}

	curr_value = list_empty(&pqf->list) ? 0 : pqf->effective_flags;

	spin_unlock_irqrestore(&pm_qos_lock, irqflags);

	return prev_value != curr_value;
}

/**
 * pm_qos_request - returns current system wide qos expectation
 * @pm_qos_class: identification of which qos value is requested
 *
 * This function returns the current target value.
 */
int pm_qos_request(int pm_qos_class)
{
	return pm_qos_read_value(pm_qos_array[pm_qos_class]->constraints);
}
EXPORT_SYMBOL_GPL(pm_qos_request);

int pm_qos_request_active(struct pm_qos_request *req)
{
	return req->pm_qos_class != 0;
}
EXPORT_SYMBOL_GPL(pm_qos_request_active);

static void __pm_qos_update_request(struct pm_qos_request *req,
			   s32 new_value)
{
	if (new_value != req->node.prio)
		pm_qos_update_target(
			pm_qos_array[req->pm_qos_class]->constraints,
			&req->node, PM_QOS_UPDATE_REQ, new_value);
}

static void pm_qos_work_fn(struct work_struct *work)
{
	s32 new_value = PM_QOS_DEFAULT_VALUE;
	struct pm_qos_request *req = container_of(to_delayed_work(work),
						  struct pm_qos_request,
						  work);

	if (!req || !pm_qos_request_active(req))
		return;

	if (new_value != req->node.prio)
		pm_qos_update_target(
			pm_qos_array[req->pm_qos_class]->constraints,
			&req->node, PM_QOS_UPDATE_REQ, new_value);
}


void pm_qos_add_request(struct pm_qos_request *req,
			int pm_qos_class, s32 value)
{
	if (!req) 
		return;

	if (pm_qos_request_active(req)) {
		WARN(1, KERN_ERR "pm_qos_add_request() called for already added request\n");
		return;
	}
	req->pm_qos_class = pm_qos_class;
	INIT_DELAYED_WORK(&req->work, pm_qos_work_fn);
	pm_qos_update_target(pm_qos_array[pm_qos_class]->constraints,
			     &req->node, PM_QOS_ADD_REQ, value);
}
EXPORT_SYMBOL_GPL(pm_qos_add_request);

void pm_qos_update_request(struct pm_qos_request *req,
			   s32 new_value)
{
	if (!req) 
		return;

	if (!pm_qos_request_active(req)) {
		WARN(1, KERN_ERR "pm_qos_update_request() called for unknown object\n");
		return;
	}

	cancel_delayed_work_sync(&req->work);

	__pm_qos_update_request(req, new_value);
}
EXPORT_SYMBOL_GPL(pm_qos_update_request);

void pm_qos_update_request_timeout(struct pm_qos_request *req, s32 new_value,
				   unsigned long timeout_us)
{
	if (!req)
		return;
	if (WARN(!pm_qos_request_active(req),
		 "%s called for unknown object.", __func__))
		return;

	cancel_delayed_work_sync(&req->work);

	if (new_value != req->node.prio)
		pm_qos_update_target(
			pm_qos_array[req->pm_qos_class]->constraints,
			&req->node, PM_QOS_UPDATE_REQ, new_value);

	schedule_delayed_work(&req->work, usecs_to_jiffies(timeout_us));
}

void pm_qos_remove_request(struct pm_qos_request *req)
{
	if (!req) 
		return;
		

	if (!pm_qos_request_active(req)) {
		WARN(1, KERN_ERR "pm_qos_remove_request() called for unknown object\n");
		return;
	}

	cancel_delayed_work_sync(&req->work);

	pm_qos_update_target(pm_qos_array[req->pm_qos_class]->constraints,
			     &req->node, PM_QOS_REMOVE_REQ,
			     PM_QOS_DEFAULT_VALUE);
	memset(req, 0, sizeof(*req));
}
EXPORT_SYMBOL_GPL(pm_qos_remove_request);

int pm_qos_add_notifier(int pm_qos_class, struct notifier_block *notifier)
{
	int retval;

	retval = blocking_notifier_chain_register(
			pm_qos_array[pm_qos_class]->constraints->notifiers,
			notifier);

	return retval;
}
EXPORT_SYMBOL_GPL(pm_qos_add_notifier);

int pm_qos_remove_notifier(int pm_qos_class, struct notifier_block *notifier)
{
	int retval;

	retval = blocking_notifier_chain_unregister(
			pm_qos_array[pm_qos_class]->constraints->notifiers,
			notifier);

	return retval;
}
EXPORT_SYMBOL_GPL(pm_qos_remove_notifier);

static int register_pm_qos_misc(struct pm_qos_object *qos)
{
	qos->pm_qos_power_miscdev.minor = MISC_DYNAMIC_MINOR;
	qos->pm_qos_power_miscdev.name = qos->name;
	qos->pm_qos_power_miscdev.fops = &pm_qos_power_fops;

	return misc_register(&qos->pm_qos_power_miscdev);
}

static int find_pm_qos_object_by_minor(int minor)
{
	int pm_qos_class;

	for (pm_qos_class = 0;
		pm_qos_class < PM_QOS_NUM_CLASSES; pm_qos_class++) {
		if (minor ==
			pm_qos_array[pm_qos_class]->pm_qos_power_miscdev.minor)
			return pm_qos_class;
	}
	return -1;
}

static int pm_qos_power_open(struct inode *inode, struct file *filp)
{
	long pm_qos_class;

	pm_qos_class = find_pm_qos_object_by_minor(iminor(inode));
	if (pm_qos_class >= 0) {
		struct pm_qos_request *req = kzalloc(sizeof(*req), GFP_KERNEL);
		if (!req)
			return -ENOMEM;

		pm_qos_add_request(req, pm_qos_class, PM_QOS_DEFAULT_VALUE);
		filp->private_data = req;

		return 0;
	}
	return -EPERM;
}

static int pm_qos_power_release(struct inode *inode, struct file *filp)
{
	struct pm_qos_request *req;

	req = filp->private_data;
	pm_qos_remove_request(req);
	kfree(req);

	return 0;
}


static ssize_t pm_qos_power_read(struct file *filp, char __user *buf,
		size_t count, loff_t *f_pos)
{
	s32 value;
	unsigned long flags;
	struct pm_qos_request *req = filp->private_data;

	if (!req)
		return -EINVAL;
	if (!pm_qos_request_active(req))
		return -EINVAL;

	spin_lock_irqsave(&pm_qos_lock, flags);
	value = pm_qos_get_value(pm_qos_array[req->pm_qos_class]->constraints);
	spin_unlock_irqrestore(&pm_qos_lock, flags);

	return simple_read_from_buffer(buf, count, f_pos, &value, sizeof(s32));
}

static ssize_t pm_qos_power_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	s32 value;
	struct pm_qos_request *req;

	if (count == sizeof(s32)) {
		if (copy_from_user(&value, buf, sizeof(s32)))
			return -EFAULT;
	} else if (count <= 11) { 
		char ascii_value[11];
		unsigned long int ulval;
		int ret;

		if (copy_from_user(ascii_value, buf, count))
			return -EFAULT;

		if (count > 10) {
			if (ascii_value[10] == '\n')
				ascii_value[10] = '\0';
			else
				return -EINVAL;
		} else {
			ascii_value[count] = '\0';
		}
		ret = kstrtoul(ascii_value, 16, &ulval);
		if (ret) {
			pr_debug("%s, 0x%lx, 0x%x\n", ascii_value, ulval, ret);
			return -EINVAL;
		}
		value = (s32)lower_32_bits(ulval);
	} else {
		return -EINVAL;
	}

	req = filp->private_data;
	pm_qos_update_request(req, value);

	return count;
}


static int __init pm_qos_power_init(void)
{
	int ret = 0;
	int i;

	BUILD_BUG_ON(ARRAY_SIZE(pm_qos_array) != PM_QOS_NUM_CLASSES);

	for (i = 1; i < PM_QOS_NUM_CLASSES; i++) {
		ret = register_pm_qos_misc(pm_qos_array[i]);
		if (ret < 0) {
			printk(KERN_ERR "pm_qos_param: %s setup failed\n",
			       pm_qos_array[i]->name);
			return ret;
		}
	}

	return ret;
}

late_initcall(pm_qos_power_init);
