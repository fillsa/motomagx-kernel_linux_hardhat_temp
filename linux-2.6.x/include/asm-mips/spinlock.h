/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 1999, 2000 by Ralf Baechle
 * Copyright (C) 1999, 2000 Silicon Graphics, Inc.
 */
#ifndef _ASM_SPINLOCK_H
#define _ASM_SPINLOCK_H

#include <asm/atomic.h>
#include <asm/page.h>

#include <linux/config.h>
#include <linux/list.h>
#include <linux/compiler.h>
#include <asm/war.h>

/*
 * Your basic SMP spinlocks, allowing only a single CPU anywhere
 */
#define __RAW_SPIN_LOCK_UNLOCKED { 0 }
#define RAW_SPIN_LOCK_UNLOCKED (raw_spinlock_t) __RAW_SPIN_LOCK_UNLOCKED
#define __raw_spin_lock_init(x)        do { *(x) = RAW_SPIN_LOCK_UNLOCKED; } while(0)

#define __raw_spin_is_locked(x)        ((x)->lock != 0)
#define __raw_spin_unlock_wait(x) \
	do { barrier(); } while (__raw_spin_is_locked(x))
#define __raw_spin_lock_flags(lock, flags) __raw_spin_lock(lock)

/*
 * Simple spin lock operations.  There are two variants, one clears IRQ's
 * on the local processor, one does not.
 *
 * We make no fairness assumptions.  They have a cost.
 */

static inline void __raw_spin_lock(raw_spinlock_t *lock)
{
	unsigned int tmp;

	if (R10000_LLSC_WAR) {
		__asm__ __volatile__(
		"	.set	noreorder	# _raw_spin_lock	\n"
		"1:	ll	%1, %2					\n"
		"	bnez	%1, 1b					\n"
		"	 li	%1, 1					\n"
		"	sc	%1, %0					\n"
		"	beqzl	%1, 1b					\n"
		"	 nop						\n"
		"	sync						\n"
		"	.set	reorder					\n"
		: "=m" (lock->lock), "=&r" (tmp)
		: "m" (lock->lock)
		: "memory");
	} else {
		__asm__ __volatile__(
		"	.set	noreorder	# _raw_spin_lock	\n"
		"1:	ll	%1, %2					\n"
		"	bnez	%1, 1b					\n"
		"	 li	%1, 1					\n"
		"	sc	%1, %0					\n"
		"	beqz	%1, 1b					\n"
		"	 sync						\n"
		"	.set	reorder					\n"
		: "=m" (lock->lock), "=&r" (tmp)
		: "m" (lock->lock)
		: "memory");
	}
}

static inline void __raw_spin_unlock(raw_spinlock_t *lock)
{
	__asm__ __volatile__(
	"	.set	noreorder	# _raw_spin_unlock	\n"
	"	sync						\n"
	"	sw	$0, %0					\n"
	"	.set\treorder					\n"
	: "=m" (lock->lock)
	: "m" (lock->lock)
	: "memory");
}

static inline unsigned int __raw_spin_trylock(raw_spinlock_t *lock)
{
	unsigned int temp, res;

	if (R10000_LLSC_WAR) {
		__asm__ __volatile__(
		"	.set	noreorder	# _raw_spin_trylock	\n"
		"1:	ll	%0, %3					\n"
		"	ori	%2, %0, 1				\n"
		"	sc	%2, %1					\n"
		"	beqzl	%2, 1b					\n"
		"	 nop						\n"
		"	andi	%2, %0, 1				\n"
		"	sync						\n"
		"	.set	reorder"
		: "=&r" (temp), "=m" (lock->lock), "=&r" (res)
		: "m" (lock->lock)
		: "memory");
	} else {
		__asm__ __volatile__(
		"	.set	noreorder	# _raw_spin_trylock	\n"
		"1:	ll	%0, %3					\n"
		"	ori	%2, %0, 1				\n"
		"	sc	%2, %1					\n"
		"	beqz	%2, 1b					\n"
		"	 andi	%2, %0, 1				\n"
		"	sync						\n"
		"	.set	reorder"
		: "=&r" (temp), "=m" (lock->lock), "=&r" (res)
		: "m" (lock->lock)
		: "memory");
	}

	return res == 0;
}

#define __RAW_RW_LOCK_UNLOCKED { 0 }
#define RAW_RW_LOCK_UNLOCKED (raw_rwlock_t) __RAW_RW_LOCK_UNLOCKED
 
#define __raw_rwlock_init(x) do { *(x) = RAW_RW_LOCK_UNLOCKED; } while(0)
#define __raw_rwlock_is_locked(x)      ((x)->lock)

#define __raw_read_can_lock(rw)		((rw)->lock >= 0)
#define __raw_write_can_lock(rw)	(!(rw)->lock)

static inline void __raw_read_lock(rwlock_t *rw)
{
	unsigned int tmp;

	if (R10000_LLSC_WAR) {
		__asm__ __volatile__(
		"	.set	noreorder	# _raw_read_lock	\n"
		"1:	ll	%1, %2					\n"
		"	bltz	%1, 1b					\n"
		"	 addu	%1, 1					\n"
		"	sc	%1, %0					\n"
		"	beqzl	%1, 1b					\n"
		"	 nop						\n"
		"	sync						\n"
		"	.set	reorder					\n"
		: "=m" (rw->lock), "=&r" (tmp)
		: "m" (rw->lock)
		: "memory");
	} else {
		__asm__ __volatile__(
		"	.set	noreorder	# _raw_read_lock	\n"
		"1:	ll	%1, %2					\n"
		"	bltz	%1, 1b					\n"
		"	 addu	%1, 1					\n"
		"	sc	%1, %0					\n"
		"	beqz	%1, 1b					\n"
		"	 sync						\n"
		"	.set	reorder					\n"
		: "=m" (rw->lock), "=&r" (tmp)
		: "m" (rw->lock)
		: "memory");
	}
}

/* Note the use of sub, not subu which will make the kernel die with an
   overflow exception if we ever try to unlock an rwlock that is already
   unlocked or is being held by a writer.  */
static inline void __raw_read_unlock(raw_rwlock_t *rw)
{
	unsigned int tmp;

	if (R10000_LLSC_WAR) {
		__asm__ __volatile__(
		"1:	ll	%1, %2		# _raw_read_unlock	\n"
		"	sub	%1, 1					\n"
		"	sc	%1, %0					\n"
		"	beqzl	%1, 1b					\n"
		"	sync						\n"
		: "=m" (rw->lock), "=&r" (tmp)
		: "m" (rw->lock)
		: "memory");
	} else {
		__asm__ __volatile__(
		"	.set	noreorder	# _raw_read_unlock	\n"
		"1:	ll	%1, %2					\n"
		"	sub	%1, 1					\n"
		"	sc	%1, %0					\n"
		"	beqz	%1, 1b					\n"
		"	 sync						\n"
		"	.set	reorder					\n"
		: "=m" (rw->lock), "=&r" (tmp)
		: "m" (rw->lock)
		: "memory");
	}
}

static inline void __raw_write_lock(raw_rwlock_t *rw)
{
	unsigned int tmp;

	if (R10000_LLSC_WAR) {
		__asm__ __volatile__(
		"	.set	noreorder	# _raw_write_lock	\n"
		"1:	ll	%1, %2					\n"
		"	bnez	%1, 1b					\n"
		"	 lui	%1, 0x8000				\n"
		"	sc	%1, %0					\n"
		"	beqzl	%1, 1b					\n"
		"	 nop						\n"
		"	sync						\n"
		"	.set	reorder					\n"
		: "=m" (rw->lock), "=&r" (tmp)
		: "m" (rw->lock)
		: "memory");
	} else {
		__asm__ __volatile__(
		"	.set	noreorder	# _raw_write_lock	\n"
		"1:	ll	%1, %2					\n"
		"	bnez	%1, 1b					\n"
		"	 lui	%1, 0x8000				\n"
		"	sc	%1, %0					\n"
		"	beqz	%1, 1b					\n"
		"	 nop						\n"
		"	sync						\n"
		"	.set	reorder					\n"
		: "=m" (rw->lock), "=&r" (tmp)
		: "m" (rw->lock)
		: "memory");
	}
}

static inline void __raw_write_unlock(raw_rwlock_t *rw)
{
	__asm__ __volatile__(
	"	sync			# _raw_write_unlock	\n"
	"	sw	$0, %0					\n"
	: "=m" (rw->lock)
	: "m" (rw->lock)
	: "memory");
}

#define __raw_read_trylock(lock) generic_raw_read_trylock(lock)

static inline int __raw_write_trylock(raw_rwlock_t *rw)
{
	unsigned int tmp;
	int ret;

	if (R10000_LLSC_WAR) {
		__asm__ __volatile__(
		"	.set	noreorder	# _raw_write_trylock	\n"
		"	li	%2, 0					\n"
		"1:	ll	%1, %3					\n"
		"	bnez	%1, 2f					\n"
		"	 lui	%1, 0x8000				\n"
		"	sc	%1, %0					\n"
		"	beqzl	%1, 1b					\n"
		"	 nop						\n"
		"	sync						\n"
		"	li	%2, 1					\n"
		"	.set	reorder					\n"
		"2:							\n"
		: "=m" (rw->lock), "=&r" (tmp), "=&r" (ret)
		: "m" (rw->lock)
		: "memory");
	} else {
		__asm__ __volatile__(
		"	.set	noreorder	# _raw_write_trylock	\n"
		"	li	%2, 0					\n"
		"1:	ll	%1, %3					\n"
		"	bnez	%1, 2f					\n"
		"	lui	%1, 0x8000				\n"
		"	sc	%1, %0					\n"
		"	beqz	%1, 1b					\n"
		"	 sync						\n"
		"	li	%2, 1					\n"
		"	.set	reorder					\n"
		"2:							\n"
		: "=m" (rw->lock), "=&r" (tmp), "=&r" (ret)
		: "m" (rw->lock)
		: "memory");
	}

	return ret;
}

#endif /* _ASM_SPINLOCK_H */
