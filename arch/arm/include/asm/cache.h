#ifndef __ASMARM_CACHE_H
#define __ASMARM_CACHE_H

#define L1_CACHE_SHIFT		CONFIG_ARM_L1_CACHE_SHIFT
#define L1_CACHE_BYTES		(1 << L1_CACHE_SHIFT)

/*
 * Set the prefetch distance in units of L1_CACHE_BYTES based on the
 * cache line size. The prefetch distance is used by the memcpy,
 * copy_from_user, copy_to_user versions that are optimized
 * for ARM v6 and v7 platforms, as well as the copy_page function
 * on ARM v5, v6 and v7 platforms.
 */

#if L1_CACHE_BYTES == 64
/*
 * This value was calibrated on a Cortex A8-based SOC with a 32-bit
 * DDR3 interface. Other Cortex cores and architectures may benefit
 * from a different setting.
 */
#define PREFETCH_DISTANCE 3
#else
/*
 * This value was calibrated on the ARM v6-based SOC used in the Raspbery
 * Pi. Other architectures may benefit from a different setting.
 */
#define PREFETCH_DISTANCE 3
#endif

#define ARCH_DMA_MINALIGN	L1_CACHE_BYTES

#if defined(CONFIG_AEABI) && (__LINUX_ARM_ARCH__ >= 5)
#define ARCH_SLAB_MINALIGN 8
#endif

#define __read_mostly __attribute__((__section__(".data..read_mostly")))

#endif
