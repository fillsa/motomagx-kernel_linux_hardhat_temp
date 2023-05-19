/*
 * Copyright (C) 2004,2005,2007 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Date     Author    Comment
 * 06/2004  Motorola  Initial creation.
 * 08/2004  Motorola  linux-2.6.7 MXC 15jun04 engineering code drop.
 * 02/2005  Motorola  Integrate Feb 21st, '05 MontaVista Linux CEE 4.0 Freescale MXC Drop 
 * 07/2007  Motorola  Modified.
 * 08/2007  Motorola  Add comments.
 */


#ifndef __ASM_SPINLOCK_H
#define __ASM_SPINLOCK_H

#if __LINUX_ARM_ARCH__ < 6
#error SMP not supported on pre-ARMv6 CPUs
#endif

/*
 * ARMv6 Spin-locking.
 *
 * We (exclusively) read the old value, and decrement it.  If it
 * hits zero, we may have won the lock, so we try (exclusively)
 * storing it.
 *
 * Unlocked value: 0
 * Locked value: 1
 */
typedef struct {
	volatile unsigned int lock;
#ifdef CONFIG_PREEMPT
	unsigned int break_lock;
#endif
} spinlock_t;

#define SPIN_LOCK_UNLOCKED	(spinlock_t) { 0 }

#define spin_lock_init(x)	do { *(x) = SPIN_LOCK_UNLOCKED; } while (0)
#define spin_is_locked(x)	((x)->lock != 0)
#define spin_unlock_wait(x)	do { barrier(); } while (spin_is_locked(x))
#define _raw_spin_lock_flags(lock, flags) _raw_spin_lock(lock)

static inline void _raw_spin_lock(spinlock_t *lock)
{
	unsigned long tmp;

	__asm__ __volatile__(
"1:	ldrex	%0, [%1]\n"
"	teq	%0, #0\n"
"	strexeq	%0, %2, [%1]\n"
"	teqeq	%0, #0\n"
"	bne	1b"
	: "=&r" (tmp)
	: "r" (&lock->lock), "r" (1)
	: "cc", "memory");
}

static inline int _raw_spin_trylock(spinlock_t *lock)
{
	unsigned long tmp;

	__asm__ __volatile__(
"	ldrex	%0, [%1]\n"
"	teq	%0, #0\n"
"	strexeq	%0, %2, [%1]"
	: "=&r" (tmp)
	: "r" (&lock->lock), "r" (1)
	: "cc", "memory");

	return tmp == 0;
}

static inline void _raw_spin_unlock(spinlock_t *lock)
{
	__asm__ __volatile__(
"	str	%1, [%0]"
	:
	: "r" (&lock->lock), "r" (0)
	: "cc", "memory");
}

/*
 * RWLOCKS
 */
typedef struct {
	volatile unsigned int lock;
#ifdef CONFIG_PREEMPT
	unsigned int break_lock;
#endif
} rwlock_t;

#define RW_LOCK_UNLOCKED	(rwlock_t) { 0 }
#ifdef CONFIG_MOT_WFN495
#define rwlock_init(x)		do { *(x) = RW_LOCK_UNLOCKED; } while (0)
#define rwlock_is_locked(x) (*((volatile unsigned int *)(x)) != 0)
#else
#define rwlock_init(x)          do { *(x) + RW_LOCK_UNLOCKED; } while (0)
#endif /* CONFIG_MOT_WFN495 */

/*
 * Write locks are easy - we just set bit 31.  When unlocking, we can
 * just write zero since the lock is exclusively held.
 */
static inline void _raw_write_lock(rwlock_t *rw)
{
	unsigned long tmp;

	__asm__ __volatile__(
"1:	ldrex	%0, [%1]\n"
"	teq	%0, #0\n"
"	strexeq	%0, %2, [%1]\n"
"	teq	%0, #0\n"
"	bne	1b"
	: "=&r" (tmp)
	: "r" (&rw->lock), "r" (0x80000000)
	: "cc", "memory");
}

static inline void _raw_write_unlock(rwlock_t *rw)
{
	__asm__ __volatile__(
	"str	%1, [%0]"
	:
	: "r" (&rw->lock), "r" (0)
	: "cc", "memory");
}

/*
 * Read locks are a bit more hairy:
 *  - Exclusively load the lock value.
 *  - Increment it.
 *  - Store new lock value if positive, and we still own this location.
 *    If the value is negative, we've already failed.
 *  - If we failed to store the value, we want a negative result.
 *  - If we failed, try again.
 * Unlocking is similarly hairy.  We may have multiple read locks
 * currently active.  However, we know we won't have any write
 * locks.
 */
static inline void _raw_read_lock(rwlock_t *rw)
{
	unsigned long tmp, tmp2;

	__asm__ __volatile__(
"1:	ldrex	%0, [%2]\n"
"	adds	%0, %0, #1\n"
"	strexpl	%1, %0, [%2]\n"
"	rsbpls	%0, %1, #0\n"
"	bmi	1b"
	: "=&r" (tmp), "=&r" (tmp2)
	: "r" (&rw->lock)
	: "cc", "memory");
}

static inline void _raw_read_unlock(rwlock_t *rw)
{
#ifdef CONFIG_MOT_WFN495
	unsigned long tmp, tmp2;
#endif /* CONFIG_MOT_WFN495 */

	__asm__ __volatile__(
"1:	ldrex	%0, [%2]\n"
"	sub	%0, %0, #1\n"
"	strex	%1, %0, [%2]\n"
"	teq	%1, #0\n"
"	bne	1b"
	: "=&r" (tmp), "=&r" (tmp2)
	: "r" (&rw->lock)
	: "cc", "memory");
}

#define _raw_read_trylock(lock) generic_raw_read_trylock(lock)

static inline int _raw_write_trylock(rwlock_t *rw)
{
	unsigned long tmp;

	__asm__ __volatile__(
"1:	ldrex	%0, [%1]\n"
"	teq	%0, #0\n"
"	strexeq	%0, %2, [%1]"
	: "=&r" (tmp)
	: "r" (&rw->lock), "r" (0x80000000)
	: "cc", "memory");

	return tmp == 0;
}

#endif /* __ASM_SPINLOCK_H */
