ifdef CONFIG_FUNCTION_TRACER
CFLAGS_REMOVE_clock.o = -pg
endif

obj-y += core.o clock.o idle_task.o fair.o rt.o stop_task.o
obj-$(CONFIG_SMP) += cpupri.o
obj-$(CONFIG_SCHED_AUTOGROUP) += auto_group.o
obj-$(CONFIG_SCHEDSTATS) += stats.o
obj-$(CONFIG_SCHED_DEBUG) += debug.o
obj-$(CONFIG_MSM_DCVS) += sched_avg.o
