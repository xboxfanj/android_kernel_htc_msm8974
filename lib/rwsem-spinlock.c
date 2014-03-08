/* rwsem-spinlock.c: R/W semaphores: contention handling functions for
 * generic spinlock implementation
 *
 * Copyright (c) 2001   David Howells (dhowells@redhat.com).
 * - Derived partially from idea by Andrea Arcangeli <andrea@suse.de>
 * - Derived also from comments by Linus
 */
#include <linux/rwsem.h>
#include <linux/sched.h>
#include <linux/export.h>

enum rwsem_waiter_type {
	RWSEM_WAITING_FOR_WRITE,
	RWSEM_WAITING_FOR_READ
};

struct rwsem_waiter {
	struct list_head list;
	struct task_struct *task;
	enum rwsem_waiter_type type;
};

int rwsem_is_locked(struct rw_semaphore *sem)
{
	int ret = 1;
	unsigned long flags;

	if (raw_spin_trylock_irqsave(&sem->wait_lock, flags)) {
		ret = (sem->activity != 0);
		raw_spin_unlock_irqrestore(&sem->wait_lock, flags);
	}
	return ret;
}
EXPORT_SYMBOL(rwsem_is_locked);

void __init_rwsem(struct rw_semaphore *sem, const char *name,
		  struct lock_class_key *key)
{
#ifdef CONFIG_DEBUG_LOCK_ALLOC
	debug_check_no_locks_freed((void *)sem, sizeof(*sem));
	lockdep_init_map(&sem->dep_map, name, key, 0);
#endif
	sem->activity = 0;
	raw_spin_lock_init(&sem->wait_lock);
	INIT_LIST_HEAD(&sem->wait_list);
}
EXPORT_SYMBOL(__init_rwsem);

static inline struct rw_semaphore *
__rwsem_do_wake(struct rw_semaphore *sem, int wakewrite)
{
	struct rwsem_waiter *waiter;
	struct task_struct *tsk;
	int woken;

	waiter = list_entry(sem->wait_list.next, struct rwsem_waiter, list);

	if (waiter->type == RWSEM_WAITING_FOR_WRITE) {
		if (wakewrite)
			/* Wake up a writer. Note that we do not grant it the
			 * lock - it will have to acquire it when it runs. */
			wake_up_process(waiter->task);
		goto out;
	}

	/* grant an infinite number of read locks to the front of the queue */
	woken = 0;
	do {
		struct list_head *next = waiter->list.next;

		list_del(&waiter->list);
		tsk = waiter->task;
		smp_mb();
		waiter->task = NULL;
		wake_up_process(tsk);
		put_task_struct(tsk);
		woken++;
		if (next == &sem->wait_list)
			break;
		waiter = list_entry(next, struct rwsem_waiter, list);
	} while (waiter->type != RWSEM_WAITING_FOR_WRITE);

	sem->activity += woken;

 out:
	return sem;
}

static inline struct rw_semaphore *
__rwsem_wake_one_writer(struct rw_semaphore *sem)
{
	struct rwsem_waiter *waiter;

	waiter = list_entry(sem->wait_list.next, struct rwsem_waiter, list);
	wake_up_process(waiter->task);

	return sem;
}

void __sched __down_read(struct rw_semaphore *sem)
{
	struct rwsem_waiter waiter;
	struct task_struct *tsk;
	unsigned long flags;

	raw_spin_lock_irqsave(&sem->wait_lock, flags);

	if (sem->activity >= 0 && list_empty(&sem->wait_list)) {
		
		sem->activity++;
		raw_spin_unlock_irqrestore(&sem->wait_lock, flags);
		goto out;
	}

	tsk = current;
	set_task_state(tsk, TASK_UNINTERRUPTIBLE);

	
	waiter.task = tsk;
	waiter.type = RWSEM_WAITING_FOR_READ;
	get_task_struct(tsk);

	list_add_tail(&waiter.list, &sem->wait_list);

	
	raw_spin_unlock_irqrestore(&sem->wait_lock, flags);

	
	for (;;) {
		if (!waiter.task)
			break;
		schedule();
		set_task_state(tsk, TASK_UNINTERRUPTIBLE);
	}

	tsk->state = TASK_RUNNING;
 out:
	;
}

int __down_read_trylock(struct rw_semaphore *sem)
{
	unsigned long flags;
	int ret = 0;


	raw_spin_lock_irqsave(&sem->wait_lock, flags);

	if (sem->activity >= 0 && list_empty(&sem->wait_list)) {
		
		sem->activity++;
		ret = 1;
	}

	raw_spin_unlock_irqrestore(&sem->wait_lock, flags);

	return ret;
}

/*
 * get a write lock on the semaphore
 */
void __sched __down_write_nested(struct rw_semaphore *sem, int subclass)
{
	struct rwsem_waiter waiter;
	struct task_struct *tsk;
	unsigned long flags;

	raw_spin_lock_irqsave(&sem->wait_lock, flags);

	/* set up my own style of waitqueue */
	tsk = current;
	waiter.task = tsk;
	waiter.type = RWSEM_WAITING_FOR_WRITE;

	list_add_tail(&waiter.list, &sem->wait_list);

	/* wait for someone to release the lock */
	for (;;) {
		/*
		 * That is the key to support write lock stealing: allows the
		 * task already on CPU to get the lock soon rather than put
		 * itself into sleep and waiting for system woke it or someone
		 * else in the head of the wait list up.
		 */
		if (sem->activity == 0)
			break;
		set_task_state(tsk, TASK_UNINTERRUPTIBLE);
		raw_spin_unlock_irqrestore(&sem->wait_lock, flags);
		schedule();
		raw_spin_lock_irqsave(&sem->wait_lock, flags);
	}
	/* got the lock */
	sem->activity = -1;
	list_del(&waiter.list);

	raw_spin_unlock_irqrestore(&sem->wait_lock, flags);
}

void __sched __down_write(struct rw_semaphore *sem)
{
	__down_write_nested(sem, 0);
}

int __down_write_trylock(struct rw_semaphore *sem)
{
	unsigned long flags;
	int ret = 0;

	raw_spin_lock_irqsave(&sem->wait_lock, flags);

	if (sem->activity == 0) {
		/* got the lock */
		sem->activity = -1;
		ret = 1;
	}

	raw_spin_unlock_irqrestore(&sem->wait_lock, flags);

	return ret;
}

void __up_read(struct rw_semaphore *sem)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&sem->wait_lock, flags);

	if (--sem->activity == 0 && !list_empty(&sem->wait_list))
		sem = __rwsem_wake_one_writer(sem);

	raw_spin_unlock_irqrestore(&sem->wait_lock, flags);
}

void __up_write(struct rw_semaphore *sem)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&sem->wait_lock, flags);

	sem->activity = 0;
	if (!list_empty(&sem->wait_list))
		sem = __rwsem_do_wake(sem, 1);

	raw_spin_unlock_irqrestore(&sem->wait_lock, flags);
}

void __downgrade_write(struct rw_semaphore *sem)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&sem->wait_lock, flags);

	sem->activity = 1;
	if (!list_empty(&sem->wait_list))
		sem = __rwsem_do_wake(sem, 0);

	raw_spin_unlock_irqrestore(&sem->wait_lock, flags);
}

