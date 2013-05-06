
#ifndef FREEZER_H_INCLUDED
#define FREEZER_H_INCLUDED

#include <linux/debug_locks.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/atomic.h>

#ifdef CONFIG_FREEZER
extern atomic_t system_freezing_cnt;	
extern bool pm_freezing;		
extern bool pm_nosig_freezing;		

static inline bool frozen(struct task_struct *p)
{
	return p->flags & PF_FROZEN;
}

extern bool freezing_slow_path(struct task_struct *p);

static inline bool freezing(struct task_struct *p)
{
	if (likely(!atomic_read(&system_freezing_cnt)))
		return false;
	return freezing_slow_path(p);
}

extern void __thaw_task(struct task_struct *t);

extern bool __refrigerator(bool check_kthr_stop);
extern int freeze_processes(void);
extern int freeze_kernel_threads(void);
extern void thaw_processes(void);
extern void thaw_kernel_threads(void);

/*
 * HACK: prevent sleeping while atomic warnings due to ARM signal handling
 * disabling irqs
 */
static inline bool try_to_freeze_nowarn(void)
{
	if (likely(!freezing(current)))
		return false;
	return __refrigerator(false);
}

/*
 * DO NOT ADD ANY NEW CALLERS OF THIS FUNCTION
 * If try_to_freeze causes a lockdep warning it means the caller may deadlock
 */
static inline bool try_to_freeze_unsafe(void)
{
	might_sleep();
	if (likely(!freezing(current)))
		return false;
	return __refrigerator(false);
}

static inline bool try_to_freeze(void)
{
	if (!(current->flags & PF_NOFREEZE))
		debug_check_no_locks_held();
	return try_to_freeze_unsafe();
}

extern bool freeze_task(struct task_struct *p);
extern bool set_freezable(void);

#ifdef CONFIG_CGROUP_FREEZER
extern bool cgroup_freezing(struct task_struct *task);
#else 
static inline bool cgroup_freezing(struct task_struct *task)
{
	return false;
}
#endif 



static inline void freezer_do_not_count(void)
{
	current->flags |= PF_FREEZER_SKIP;
}

static inline void freezer_count(void)
{
	current->flags &= ~PF_FREEZER_SKIP;
	try_to_freeze();
}

/* DO NOT ADD ANY NEW CALLERS OF THIS FUNCTION */
static inline void freezer_count_unsafe(void)
{
	current->flags &= ~PF_FREEZER_SKIP;
	smp_mb();
	try_to_freeze_unsafe();
}

/**
 * freezer_should_skip - whether to skip a task when determining frozen
 *			 state is reached
 * @p: task in quesion
 *
 * This function is used by freezers after establishing %true freezing() to
 * test whether a task should be skipped when determining the target frozen
 * state is reached.  IOW, if this function returns %true, @p is considered
 * frozen enough.
 */
static inline int freezer_should_skip(struct task_struct *p)
{
	return !!(p->flags & PF_FREEZER_SKIP);
}


#define freezable_schedule()						\
({									\
	freezer_do_not_count();						\
	schedule();							\
	freezer_count();						\
})

/* DO NOT ADD ANY NEW CALLERS OF THIS FUNCTION */
#define freezable_schedule_unsafe()					\
({									\
	freezer_do_not_count();						\
	schedule();							\
	freezer_count_unsafe();						\
})

/* Like schedule_timeout_killable(), but should not block the freezer. */
#define freezable_schedule_timeout_killable(timeout)			\
({									\
	long __retval;							\
	freezer_do_not_count();						\
	__retval = schedule_timeout_killable(timeout);			\
	freezer_count();						\
	__retval;							\
})

/* DO NOT ADD ANY NEW CALLERS OF THIS FUNCTION */
#define freezable_schedule_timeout_killable_unsafe(timeout)		\
({									\
	long __retval;							\
	freezer_do_not_count();						\
	__retval = schedule_timeout_killable(timeout);			\
	freezer_count_unsafe();						\
	__retval;							\
})

/*
 * Freezer-friendly wrappers around wait_event_interruptible(),
 * wait_event_killable() and wait_event_interruptible_timeout(), originally
 * defined in <linux/wait.h>
 */

#define wait_event_freezekillable(wq, condition)			\
({									\
	int __retval;							\
	freezer_do_not_count();						\
	__retval = wait_event_killable(wq, (condition));		\
	freezer_count();						\
	__retval;							\
})

/* DO NOT ADD ANY NEW CALLERS OF THIS FUNCTION */
#define wait_event_freezekillable_unsafe(wq, condition)			\
({									\
	int __retval;							\
	freezer_do_not_count();						\
	__retval = wait_event_killable(wq, (condition));		\
	freezer_count_unsafe();						\
	__retval;							\
})

#define wait_event_freezable(wq, condition)				\
({									\
	int __retval;							\
	freezer_do_not_count();						\
	__retval = wait_event_interruptible(wq, (condition));		\
	freezer_count();						\
	__retval;							\
})

#define wait_event_freezable_timeout(wq, condition, timeout)		\
({									\
	long __retval = timeout;					\
	freezer_do_not_count();						\
	__retval = wait_event_interruptible_timeout(wq,	(condition),	\
				__retval);				\
	freezer_count();						\
	__retval;							\
})

#else 
static inline bool frozen(struct task_struct *p) { return false; }
static inline bool freezing(struct task_struct *p) { return false; }
static inline void __thaw_task(struct task_struct *t) {}

static inline bool __refrigerator(bool check_kthr_stop) { return false; }
static inline int freeze_processes(void) { return -ENOSYS; }
static inline int freeze_kernel_threads(void) { return -ENOSYS; }
static inline void thaw_processes(void) {}
static inline void thaw_kernel_threads(void) {}

static inline bool try_to_freeze(void) { return false; }

static inline void freezer_do_not_count(void) {}
static inline void freezer_count(void) {}
static inline int freezer_should_skip(struct task_struct *p) { return 0; }
static inline void set_freezable(void) {}

#define freezable_schedule()  schedule()

#define freezable_schedule_unsafe()  schedule()

#define freezable_schedule_timeout_killable(timeout)			\
	schedule_timeout_killable(timeout)

#define freezable_schedule_timeout_killable_unsafe(timeout)		\
	schedule_timeout_killable(timeout)

#define wait_event_freezable(wq, condition)				\
		wait_event_interruptible(wq, condition)

#define wait_event_freezable_timeout(wq, condition, timeout)		\
		wait_event_interruptible_timeout(wq, condition, timeout)

#define wait_event_freezekillable(wq, condition)		\
		wait_event_killable(wq, condition)

#define wait_event_freezekillable_unsafe(wq, condition)			\
		wait_event_killable(wq, condition)

#endif /* !CONFIG_FREEZER */

#endif	
