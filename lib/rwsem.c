/* rwsem.c: R/W semaphores: contention handling functions
 *
 * Written by David Howells (dhowells@redhat.com).
 * Derived from arch/i386/kernel/semaphore.c
 *
 * Writer lock-stealing by Alex Shi <alex.shi@intel.com>
 */
#include <linux/rwsem.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/export.h>

void __init_rwsem(struct rw_semaphore *sem, const char *name,
		  struct lock_class_key *key)
{
#ifdef CONFIG_DEBUG_LOCK_ALLOC
	debug_check_no_locks_freed((void *)sem, sizeof(*sem));
	lockdep_init_map(&sem->dep_map, name, key, 0);
#endif
	sem->count = RWSEM_UNLOCKED_VALUE;
	raw_spin_lock_init(&sem->wait_lock);
	INIT_LIST_HEAD(&sem->wait_list);
}

EXPORT_SYMBOL(__init_rwsem);

struct rwsem_waiter {
	struct list_head list;
	struct task_struct *task;
	unsigned int flags;
#define RWSEM_WAITING_FOR_READ	0x00000001
#define RWSEM_WAITING_FOR_WRITE	0x00000002
};

#define RWSEM_WAKE_ANY        0 
#define RWSEM_WAKE_NO_ACTIVE  1 
#define RWSEM_WAKE_READ_OWNED 2 

static struct rw_semaphore *
__rwsem_do_wake(struct rw_semaphore *sem, int wake_type)
{
	struct rwsem_waiter *waiter;
	struct task_struct *tsk;
	struct list_head *next;
	signed long woken, loop, adjustment;

	waiter = list_entry(sem->wait_list.next, struct rwsem_waiter, list);
	if (!(waiter->flags & RWSEM_WAITING_FOR_WRITE))
		goto readers_only;

	if (wake_type == RWSEM_WAKE_READ_OWNED)
		goto out;

	/* Wake up the writing waiter and let the task grab the sem: */
	wake_up_process(waiter->task);
	goto out;

 readers_only:
	if (wake_type == RWSEM_WAKE_ANY &&
	    rwsem_atomic_update(0, sem) < RWSEM_WAITING_BIAS)
		
		goto out;

	woken = 0;
	do {
		woken++;

		if (waiter->list.next == &sem->wait_list)
			break;

		waiter = list_entry(waiter->list.next,
					struct rwsem_waiter, list);

	} while (waiter->flags & RWSEM_WAITING_FOR_READ);

	adjustment = woken * RWSEM_ACTIVE_READ_BIAS;
	if (waiter->flags & RWSEM_WAITING_FOR_READ)
		
		adjustment -= RWSEM_WAITING_BIAS;

	rwsem_atomic_add(adjustment, sem);

	next = sem->wait_list.next;
	for (loop = woken; loop > 0; loop--) {
		waiter = list_entry(next, struct rwsem_waiter, list);
		next = waiter->list.next;
		tsk = waiter->task;
		smp_mb();
		waiter->task = NULL;
		wake_up_process(tsk);
		put_task_struct(tsk);
	}

	sem->wait_list.next = next;
	next->prev = &sem->wait_list;

 out:
	return sem;
}

/* Try to get write sem, caller holds sem->wait_lock: */
static int try_get_writer_sem(struct rw_semaphore *sem,
					struct rwsem_waiter *waiter)
{
	struct rwsem_waiter *fwaiter;
	long oldcount, adjustment;

	/* only steal when first waiter is writing */
	fwaiter = list_entry(sem->wait_list.next, struct rwsem_waiter, list);
	if (!(fwaiter->flags & RWSEM_WAITING_FOR_WRITE))
		return 0;

	adjustment = RWSEM_ACTIVE_WRITE_BIAS;
	/* Only one waiter in the queue: */
	if (fwaiter == waiter && waiter->list.next == &sem->wait_list)
		adjustment -= RWSEM_WAITING_BIAS;

try_again_write:
	oldcount = rwsem_atomic_update(adjustment, sem) - adjustment;
	if (!(oldcount & RWSEM_ACTIVE_MASK)) {
		/* No active lock: */
		struct task_struct *tsk = waiter->task;

		list_del(&waiter->list);
		smp_mb();
		put_task_struct(tsk);
		tsk->state = TASK_RUNNING;
		return 1;
	}
	/* some one grabbed the sem already */
	if (rwsem_atomic_update(-adjustment, sem) & RWSEM_ACTIVE_MASK)
		return 0;
	goto try_again_write;
}

static struct rw_semaphore __sched *
rwsem_down_failed_common(struct rw_semaphore *sem,
			 unsigned int flags, signed long adjustment)
{
	struct rwsem_waiter waiter;
	struct task_struct *tsk = current;
	signed long count;

	set_task_state(tsk, TASK_UNINTERRUPTIBLE);

	
	raw_spin_lock_irq(&sem->wait_lock);
	waiter.task = tsk;
	waiter.flags = flags;
	get_task_struct(tsk);

	if (list_empty(&sem->wait_list))
		adjustment += RWSEM_WAITING_BIAS;
	list_add_tail(&waiter.list, &sem->wait_list);

	
	count = rwsem_atomic_update(adjustment, sem);

	if (count == RWSEM_WAITING_BIAS)
		sem = __rwsem_do_wake(sem, RWSEM_WAKE_NO_ACTIVE);
	else if (count > RWSEM_WAITING_BIAS &&
		 adjustment == -RWSEM_ACTIVE_WRITE_BIAS)
		sem = __rwsem_do_wake(sem, RWSEM_WAKE_READ_OWNED);

	raw_spin_unlock_irq(&sem->wait_lock);

	
	for (;;) {
		if (!waiter.task)
			break;

		raw_spin_lock_irq(&sem->wait_lock);
		/* Try to get the writer sem, may steal from the head writer: */
		if (flags == RWSEM_WAITING_FOR_WRITE)
			if (try_get_writer_sem(sem, &waiter)) {
				raw_spin_unlock_irq(&sem->wait_lock);
				return sem;
			}
		raw_spin_unlock_irq(&sem->wait_lock);
		schedule();
		set_task_state(tsk, TASK_UNINTERRUPTIBLE);
	}

	tsk->state = TASK_RUNNING;

	return sem;
}

struct rw_semaphore __sched *rwsem_down_read_failed(struct rw_semaphore *sem)
{
	return rwsem_down_failed_common(sem, RWSEM_WAITING_FOR_READ,
					-RWSEM_ACTIVE_READ_BIAS);
}

struct rw_semaphore __sched *rwsem_down_write_failed(struct rw_semaphore *sem)
{
	return rwsem_down_failed_common(sem, RWSEM_WAITING_FOR_WRITE,
					-RWSEM_ACTIVE_WRITE_BIAS);
}

struct rw_semaphore *rwsem_wake(struct rw_semaphore *sem)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&sem->wait_lock, flags);

	
	if (!list_empty(&sem->wait_list))
		sem = __rwsem_do_wake(sem, RWSEM_WAKE_ANY);

	raw_spin_unlock_irqrestore(&sem->wait_lock, flags);

	return sem;
}

struct rw_semaphore *rwsem_downgrade_wake(struct rw_semaphore *sem)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&sem->wait_lock, flags);

	
	if (!list_empty(&sem->wait_list))
		sem = __rwsem_do_wake(sem, RWSEM_WAKE_READ_OWNED);

	raw_spin_unlock_irqrestore(&sem->wait_lock, flags);

	return sem;
}

EXPORT_SYMBOL(rwsem_down_read_failed);
EXPORT_SYMBOL(rwsem_down_write_failed);
EXPORT_SYMBOL(rwsem_wake);
EXPORT_SYMBOL(rwsem_downgrade_wake);
