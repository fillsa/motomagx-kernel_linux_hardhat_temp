/*
 *  include/asm-s390/spinlock.h
 *
 *  S390 version
 *    Copyright (C) 1999 IBM Deutschland Entwicklung GmbH, IBM Corporation
 *    Author(s): Martin Schwidefsky (schwidefsky@de.ibm.com)
 *
 *  Derived from "include/asm-i386/spinlock.h"
 */

#ifndef __ASM_SPINLOCK_H
#define __ASM_SPINLOCK_H

static inline int
_raw_compare_and_swap(volatile unsigned int *lock,
		      unsigned int old, unsigned int new)
{
	asm volatile ("cs %0,%3,0(%4)"
		      : "=d" (old), "=m" (*lock)
		      : "0" (old), "d" (new), "a" (lock), "m" (*lock)
		      : "cc", "memory" );
	return old;
}

/*
 * Simple spin lock operations.  There are two variants, one clears IRQ's
 * on the local processor, one does not.
 *
 * We make no fairness assumptions. They have a cost.
 */

typedef struct {
	volatile unsigned int lock;
#ifdef CONFIG_PREEMPT
	unsigned int break_lock;
#endif
} __attribute__ ((aligned (4))) spinlock_t;

#define SPIN_LOCK_UNLOCKED	(spinlock_t) { 0 }
#define spin_lock_init(lp)	do { (lp)->lock = 0; } while(0)
#define spin_unlock_wait(lp)	do { barrier(); } while(((volatile spinlock_t *)(lp))->lock)
#define spin_is_locked(x)	((x)->lock != 0)
#define _raw_spin_lock_flags(lock, flags) _raw_spin_lock(lock)

extern void _raw_spin_lock_wait(spinlock_t *lp, unsigned int pc);
extern int _raw_spin_trylock_retry(spinlock_t *lp, unsigned int pc);

static inline void _raw_spin_lock(spinlock_t *lp)
{
	unsigned long pc = (unsigned long) __builtin_return_address(0);

	if (unlikely(_raw_compare_and_swap(&lp->lock, 0, pc) != 0))
		_raw_spin_lock_wait(lp, pc);
}

static inline int _raw_spin_trylock(spinlock_t *lp)
{
	unsigned long pc = (unsigned long) __builtin_return_address(0);

	if (likely(_raw_compare_and_swap(&lp->lock, 0, pc) == 0))
		return 1;
	return _raw_spin_trylock_retry(lp, pc);
}

static inline void _raw_spin_unlock(spinlock_t *lp)
{
	_raw_compare_and_swap(&lp->lock, lp->lock, 0);
}
		
/*
 * Read-write spinlocks, allowing multiple readers
 * but only one writer.
 *
 * NOTE! it is quite common to have readers in interrupts
 * but no interrupt writers. For those circumstances we
 * can "mix" irq-safe locks - any writer needs to get a
 * irq-safe write-lock, but readers can get non-irqsafe
 * read-locks.
 */
typedef struct {
	volatile unsigned int lock;
	volatile unsigned long owner_pc;
#ifdef CONFIG_PREEMPT
	unsigned int break_lock;
#endif
} rwlock_t;

#define RW_LOCK_UNLOCKED (rwlock_t) { 0, 0 }

#define rwlock_init(x)	do { *(x) = RW_LOCK_UNLOCKED; } while(0)
//del 2.6.11	#define rwlock_is_locked(x) ((x)->lock != 0)

/**
 * read_can_lock - would read_trylock() succeed?
 * @lock: the rwlock in question.
 */
#define read_can_lock(x) ((int)(x)->lock >= 0)

/**
 * write_can_lock - would write_trylock() succeed?
 * @lock: the rwlock in question.
 */
#define write_can_lock(x) ((x)->lock == 0)

extern void _raw_read_lock_wait(rwlock_t *lp);
extern int _raw_read_trylock_retry(rwlock_t *lp);
extern void _raw_write_lock_wait(rwlock_t *lp);
extern int _raw_write_trylock_retry(rwlock_t *lp);

static inline void _raw_read_lock(rwlock_t *rw)
{
	unsigned int old;
	old = rw->lock & 0x7fffffffU;
	if (_raw_compare_and_swap(&rw->lock, old, old + 1) != old)
		_raw_read_lock_wait(rw);
}

static inline void _raw_read_unlock(rwlock_t *rw)
{
	unsigned int old, cmp;

	old = rw->lock;
	do {
		cmp = old;
		old = _raw_compare_and_swap(&rw->lock, old, old - 1);
	} while (cmp != old);
}

static inline void _raw_write_lock(rwlock_t *rw)
{
	if (unlikely(_raw_compare_and_swap(&rw->lock, 0, 0x80000000) != 0))
		_raw_write_lock_wait(rw);
}

static inline void _raw_write_unlock(rwlock_t *rw)
{
	_raw_compare_and_swap(&rw->lock, 0x80000000, 0);
}

static inline int _raw_read_trylock(rwlock_t *rw)
{
	unsigned int old;
	old = rw->lock & 0x7fffffffU;
	if (likely(_raw_compare_and_swap(&rw->lock, old, old + 1) == old))
		return 1;
	return _raw_read_trylock_retry(rw);
}

static inline int _raw_write_trylock(rwlock_t *rw)
{
	if (likely(_raw_compare_and_swap(&rw->lock, 0, 0x80000000) == 0))
		return 1;
	return _raw_write_trylock_retry(rw);
}

#endif /* __ASM_SPINLOCK_H */
