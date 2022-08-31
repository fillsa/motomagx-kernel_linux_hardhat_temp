/*
 * linux/kernel/posix_timers.c
 *
 *
 * 2002-10-15  Posix Clocks & timers
 *                           by George Anzinger george@mvista.com
 *
 *			     Copyright (C) 2002 2003 by MontaVista Software.
 *
 * 2004-06-01  Fix CLOCK_REALTIME clock/timer TIMER_ABSTIME bug.
 *			     Copyright (C) 2004 Boris Hu
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * MontaVista Software | 1237 East Arques Avenue | Sunnyvale | CA 94085 | USA
 */

/* These are all the functions necessary to implement
 * POSIX clocks & timers
 */
#include <linux/mm.h>
#include <linux/smp_lock.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/time.h>

#include <asm/uaccess.h>
#include <asm/semaphore.h>
#include <linux/list.h>
#include <linux/init.h>
#include <linux/compiler.h>
#include <linux/idr.h>
#include <linux/hrtime.h>
#include <linux/posix-timers.h>
#include <linux/syscalls.h>
#include <linux/wait.h>
#include <linux/sc_math.h>
#include <asm/div64.h>
#include <linux/workqueue.h>

#ifndef div_long_long_rem

#define div_long_long_rem(dividend,divisor,remainder) ({ \
		       u64 result = dividend;		\
		       *remainder = do_div(result,divisor); \
		       result; })

#endif
#define CLOCK_REALTIME_RES TICK_NSEC  /* In nano seconds. */

/*
 * Management arrays for POSIX timers.	 Timers are kept in slab memory
 * Timer ids are allocated by an external routine that keeps track of the
 * id and the timer.  The external interface is:
 *
 * void *idr_find(struct idr *idp, int id);           to find timer_id <id>
 * int idr_get_new(struct idr *idp, void *ptr);       to get a new id and
 *                                                    related it to <ptr>
 * void idr_remove(struct idr *idp, int id);          to release <id>
 * void idr_init(struct idr *idp);                    to initialize <idp>
 *                                                    which we supply.
 * The idr_get_new *may* call slab for more memory so it must not be
 * called under a spin lock.  Likewise idr_remore may release memory
 * (but it may be ok to do this under a lock...).
 * idr_find is just a memory look up and is quite fast.  A -1 return
 * indicates that the requested id does not exist.
 */

/*
 * Lets keep our timers in a slab cache :-)
 */
static kmem_cache_t *posix_timers_cache;
static struct idr posix_timers_id;
static DEFINE_SPINLOCK(idr_lock);

/*
 * Just because the timer is not in the timer list does NOT mean it is
 * inactive.  It could be in the "fire" routine getting a new expire time.
 */
#define TIMER_INACTIVE 1

#if defined(CONFIG_SMP) || defined(CONFIG_PREEMPT_SOFTIRQS)
# define timer_active(tmr) \
		((tmr)->it_timer.entry.prev != (void *)TIMER_INACTIVE)
# define set_timer_inactive(tmr) \
		do { \
			(tmr)->it_timer.entry.prev = (void *)TIMER_INACTIVE; \
		} while (0)
#else
# define timer_active(tmr) BARFY	/* error to use outside of SMP | RT */
# define set_timer_inactive(tmr) do { } while (0)
#endif
/*
 * For RT the timer call backs are preemptable.  This means that folks
 * trying to delete timers may run into timers that are "active" for
 * long times.  To help out with this we provide a wake up function to
 * wake up a caller who wants waking when a timer clears the call back.
 * This is the same sort of thing that the del_timer_sync does, but we
 * need (in the HRT case) to cover two lists and not just the one.
 */
#ifdef CONFIG_PREEMPT_SOFTIRQS
#include <linux/wait.h>
static DECLARE_WAIT_QUEUE_HEAD(timer_wake_queue);
#define wake_timer_waiters() wake_up(&timer_wake_queue)
#define wait_for_timer(timer) wait_event(timer_wake_queue, !timer_active(timer))

#else
#define wake_timer_waiters()
#define wait_for_timer(timer)
#endif
/*
 * we assume that the new SIGEV_THREAD_ID shares no bits with the other
 * SIGEV values.  Here we put out an error if this assumption fails.
 */
#if SIGEV_THREAD_ID != (SIGEV_THREAD_ID & \
                       ~(SIGEV_SIGNAL | SIGEV_NONE | SIGEV_THREAD))
#error "SIGEV_THREAD_ID must not share bit with other SIGEV values!"
#endif


/*
 * The timer ID is turned into a timer address by idr_find().
 * Verifying a valid ID consists of:
 *
 * a) checking that idr_find() returns other than -1.
 * b) checking that the timer id matches the one in the timer itself.
 * c) that the timer owner is in the callers thread group.
 */

/*
 * CLOCKs: The POSIX standard calls for a couple of clocks and allows us
 *	    to implement others.  This structure defines the various
 *	    clocks and allows the possibility of adding others.	 We
 *	    provide an interface to add clocks to the table and expect
 *	    the "arch" code to add at least one clock that is high
 *	    resolution.	 Here we define the standard CLOCK_REALTIME as a
 *	    1/HZ resolution clock.
 *
 * RESOLUTION: Clock resolution is used to round up timer and interval
 *	    times, NOT to report clock times, which are reported with as
 *	    much resolution as the system can muster.  In some cases this
 *	    resolution may depend on the underlaying clock hardware and
 *	    may not be quantifiable until run time, and only then is the
 *	    necessary code is written.	The standard says we should say
 *	    something about this issue in the documentation...
 *
 * FUNCTIONS: The CLOCKs structure defines possible functions to handle
 *	    various clock functions.  For clocks that use the standard
 *	    system timer code these entries should be NULL.  This will
 *	    allow dispatch without the overhead of indirect function
 *	    calls.  CLOCKS that depend on other sources (e.g. WWV or GPS)
 *	    must supply functions here, even if the function just returns
 *	    ENOSYS.  The standard POSIX timer management code assumes the
 *	    following: 1.) The k_itimer struct (sched.h) is used for the
 *	    timer.  2.) The list, it_lock, it_clock, it_id and it_process
 *	    fields are not modified by timer code.
 *
 *          At this time all functions EXCEPT clock_nanosleep can be
 *          redirected by the CLOCKS structure.  Clock_nanosleep is in
 *          there, but the code ignores it.
 *
 * Permissions: It is assumed that the clock_settime() function defined
 *	    for each clock will take care of permission checks.	 Some
 *	    clocks may be set able by any user (i.e. local process
 *	    clocks) others not.	 Currently the only set able clock we
 *	    have is CLOCK_REALTIME and its high res counter part, both of
 *	    which we beg off on and pass to do_sys_settimeofday().
 */

static struct k_clock posix_clocks[MAX_CLOCKS];
IF_HIGH_RES(static long arch_res_by_2;)
/*
 * We only have one real clock that can be set so we need only one abs list,
 * even if we should want to have several clocks with differing resolutions.
 */
static struct k_clock_abs abs_list = {.list = LIST_HEAD_INIT(abs_list.list),
				      .lock = SPIN_LOCK_UNLOCKED};

static void posix_timer_fn(unsigned long);
static u64 do_posix_clock_monotonic_gettime_parts(
	struct timespec *tp, struct timespec *mo, long *arch_cycle);
int do_posix_clock_monotonic_gettime(struct timespec *tp);
static int do_posix_clock_monotonic_get(clockid_t, struct timespec *tp);

static struct k_itimer *lock_timer(timer_t timer_id, unsigned long *flags);

static inline void unlock_timer(struct k_itimer *timr, unsigned long flags)
{
	spin_unlock_irqrestore(&timr->it_lock, flags);
}

/*
 * Call the k_clock hook function if non-null, or the default function.
 */
#define CLOCK_DISPATCH(clock, call, arglist) \
 	((clock) < 0 ? posix_cpu_##call arglist : \
 	 (posix_clocks[clock].call != NULL \
 	  ? (*posix_clocks[clock].call) arglist : common_##call arglist))

/*
 * Default clock hook functions when the struct k_clock passed
 * to register_posix_clock leaves a function pointer null.
 *
 * The function common_CALL is the default implementation for
 * the function pointer CALL in struct k_clock.
 */

static inline int common_clock_getres(clockid_t which_clock,
				      struct timespec *tp)
{
	tp->tv_sec = 0;
	tp->tv_nsec = posix_clocks[which_clock].res;
	return 0;
}

static inline int common_clock_get(clockid_t which_clock, struct timespec *tp)
{
	getnstimeofday(tp);
	return 0;
}

static inline int common_clock_set(clockid_t which_clock, struct timespec *tp)
{
	return do_sys_settimeofday(tp, NULL);
}

static inline int common_timer_create(struct k_itimer *new_timer)
{
	INIT_LIST_HEAD(&new_timer->it.real.abs_timer_entry);
	init_timer(&new_timer->it.real.timer);
	new_timer->it.real.timer.data = (unsigned long) new_timer;
	new_timer->it.real.timer.function = posix_timer_fn;
	set_timer_inactive(new_timer);
	return 0;
}

/*
 * These ones are defined below.
 */
static int common_nsleep(clockid_t, int flags, struct timespec *t);
static void common_timer_get(struct k_itimer *, struct itimerspec *);
static int common_timer_set(struct k_itimer *, int,
			    struct itimerspec *, struct itimerspec *);
static int common_timer_del(struct k_itimer *timer);

/*
 * Return nonzero iff we know a priori this clockid_t value is bogus.
 */
static inline int invalid_clockid(clockid_t which_clock)
{
	if (which_clock < 0)	/* CPU clock, posix_cpu_* will check it */
		return 0;
	if ((unsigned) which_clock >= MAX_CLOCKS)
		return 1;
	if (posix_clocks[which_clock].clock_getres != NULL)
		return 0;
#ifndef CLOCK_DISPATCH_DIRECT
	if (posix_clocks[which_clock].res != 0)
		return 0;
#endif
	return 1;
}


/*
 * Initialize everything, well, just everything in Posix clocks/timers ;)
 */
static __init int init_posix_timers(void)
{
	struct k_clock clock_realtime = {.res = CLOCK_REALTIME_RES,
					 .abs_struct = &abs_list
	};
	struct k_clock clock_monotonic = {.res = CLOCK_REALTIME_RES,
		.abs_struct = NULL,
		.clock_get = do_posix_clock_monotonic_gettime,
		.clock_set = do_posix_clock_nosettime
	};

	register_posix_clock(CLOCK_REALTIME, &clock_realtime);
	register_posix_clock(CLOCK_MONOTONIC, &clock_monotonic);

	posix_timers_cache = kmem_cache_create("posix_timers_cache",
					sizeof (struct k_itimer), 0, 0, NULL, NULL);
	idr_init(&posix_timers_id);
#ifdef CONFIG_HIGH_RES_TIMERS
	/*
	 * Possibly switched out at boot time
	 */
	if (hrtimer_use) {
		clock_realtime.res = hr_time_resolution;
		register_posix_clock(CLOCK_REALTIME_HR, &clock_realtime);
		clock_monotonic.res = hr_time_resolution;
		register_posix_clock(CLOCK_MONOTONIC_HR, &clock_monotonic);
	}
	arch_res_by_2 = nsec_to_arch_cycle(hr_time_resolution >> 1);
#endif

#ifdef	final_clock_init
	final_clock_init();	/* defined as needed by arch header file */
#endif

	return 0;
}

__initcall(init_posix_timers);

#ifndef CONFIG_HIGH_RES_TIMERS
static void tstojiffie(struct timespec *tp, int res, u64 *jiff)
{
	struct timespec tv = *tp;
	tv.tv_nsec += res - 1;

	timespec_norm(&tv);

	/*
	 * The scaling constants are defined in <linux/time.h>
	 * The difference between there and here is that we do the
	 * res rounding and compute a 64-bit result (well so does that
	 * but it then throws away the high bits).
  	 */
	*jiff =  (((u64)tv.tv_sec * SEC_CONVERSION) +
		  (((u64)tv.tv_nsec * NSEC_CONVERSION) >>
		   (NSEC_JIFFIE_SC - SEC_JIFFIE_SC))) >> SEC_JIFFIE_SC;
}
#else
static long tstojiffie(struct timespec *tp, int res, u64 *jiff)
{
	struct timespec tv = *tp;
	u64 raw_jiff;
	unsigned long mask_jiff;
	long rtn;

	tv.tv_nsec += res - 1;

	timespec_norm(&tv);

	/*
	 * This much like the above, except, well you know that bit
	 * we shift off the right end to get jiffies.  Well that is
	 * the arch_cycle part and here we pick that up and convert
	 * it to arch_cycles.
	 *
	 * Also, we need to be a bit more rigorous about resolution,
	 * See the next line...
	 */
	tv.tv_nsec -= tv.tv_nsec % res;

	raw_jiff =  (((u64)tv.tv_sec * SEC_CONVERSION) +
		     (((u64)tv.tv_nsec * NSEC_CONVERSION) >>
		(NSEC_JIFFIE_SC - SEC_JIFFIE_SC)));
	*jiff = raw_jiff >> SEC_JIFFIE_SC;

	mask_jiff = raw_jiff & ((1 << SEC_JIFFIE_SC) -1);

	rtn = ((u64)mask_jiff * arch_cycles_per_jiffy +
		(1 << SEC_JIFFIE_SC) -1) >> SEC_JIFFIE_SC;

	return rtn;
}
#endif


/*
 * This function adjusts the timer as needed as a result of the clock
 * being set.  It should only be called for absolute timers, and then
 * under the abs_list lock.  It computes the time difference and sets
 * the new jiffies value in the timer.  It also updates the timers
 * reference wall_to_monotonic value.  It is complicated by the fact
 * that tstojiffies() only handles positive times and it needs to work
 * with both positive and negative times.  Also, for negative offsets,
 * we need to make the res round up actually round up (not down).
 *
 * Return is true if there is a new time, else false.
 */
static long add_clockset_delta(struct k_itimer *timr,
			       struct timespec *new_wall_to)
{
	struct timespec delta;
	u64 exp;
	IF_HIGH_RES(long arch_cycle);

	set_normalized_timespec(&delta,
				new_wall_to->tv_sec -
				timr->it.real.wall_to_prev.tv_sec,
				new_wall_to->tv_nsec -
				timr->it.real.wall_to_prev.tv_nsec);
	if (likely(!(delta.tv_sec | delta.tv_nsec)))
		return 0;
	if (delta.tv_sec < 0) {
		/* clock advanced */
		set_normalized_timespec(&delta, 
					-delta.tv_sec,
					-posix_clocks[timr->it_clock].res - 
					delta.tv_nsec);
		IF_HIGH_RES(arch_cycle = -)
			tstojiffie(&delta, posix_clocks[timr->it_clock].res,
				   &exp);
		exp = -exp;
	} else {
		/* clock retarded */
		IF_HIGH_RES(arch_cycle = )
			tstojiffie(&delta, posix_clocks[timr->it_clock].res, 
				   &exp);
	}
	timr->it.real.wall_to_prev= *new_wall_to;
	timr->it.mmtimer.expires += exp;
#ifdef CONFIG_HIGH_RES_TIMERS
	timr->it.real.timer.arch_cycle_expires += arch_cycle;
	full_normalize_jiffies(&timr->it.real.timer.expires,
			       &timr->it.real.timer.arch_cycle_expires);
#endif
	return 1;
}

static void remove_from_abslist(struct k_itimer *timr)
{
	if (!list_empty(&timr->it.real.abs_timer_entry)) {
		spin_lock(&abs_list.lock);
		list_del_init(&timr->it.real.abs_timer_entry);
		spin_unlock(&abs_list.lock);
	}
}

static void schedule_next_timer(struct k_itimer *timr)
{
	struct timespec new_wall_to;
	struct now_struct now;
	unsigned long seq;

	/*
	 * Set up the timer for the next interval (if there is one).
	 * Note: this code uses the abs_timer_lock to protect
	 * wall_to_prev and must hold it until exp is set, not exactly
	 * obvious...

	 * This function is used for CLOCK_REALTIME* and
	 * CLOCK_MONOTONIC* timers.  If we ever want to handle other
	 * CLOCKs, the calling code (do_schedule_next_timer) would need
	 * to pull the "clock" info from the timer and dispatch the
	 * "other" CLOCKs "next timer" code (which, I suppose should
	 * also be added to the k_clock structure).
	 */

	/* Set up the timer for the next interval (if there is one) */
#ifdef CONFIG_HIGH_RES_TIMERS
	if (!(timr->it.real.incr | timr->it_arch_cycle_incr))
		return;
#else
	if (!(timr->it.real.incr))
		return;
#endif
	do {
		seq = read_seqbegin(&xtime_lock);
		new_wall_to =	wall_to_monotonic;
		posix_get_now(&now);
	} while (read_seqretry(&xtime_lock, seq));

	if (!list_empty(&timr->it.real.abs_timer_entry)) {
		spin_lock(&abs_list.lock);
		add_clockset_delta(timr, &new_wall_to);

		posix_bump_timer(timr, &now);

		spin_unlock(&abs_list.lock);
	} else {
		posix_bump_timer(timr, &now);
	}
	timr->it_overrun_last = timr->it_overrun;
	timr->it_overrun = -1;
	++timr->it_requeue_pending;
	add_timer(&timr->it.real.timer);
}

/*
 * This function is exported for use by the signal deliver code.  It is
 * called just prior to the info block being released and passes that
 * block to us.  It's function is to update the overrun entry AND to
 * restart the timer.  It should only be called if the timer is to be
 * restarted (i.e. we have flagged this in the sys_private entry of the
 * info block).
 *
 * To protect aginst the timer going away while the interrupt is queued,
 * we require that the it_requeue_pending flag be set.
 */
void do_schedule_next_timer(struct siginfo *info)
{
	struct k_itimer *timr;
	unsigned long flags;

	timr = lock_timer(info->si_tid, &flags);

	if (!timr || timr->it_requeue_pending != info->si_sys_private)
		goto exit;

	schedule_next_timer(timr);
	info->si_overrun = timr->it_overrun_last;
exit:
	if (timr)
		unlock_timer(timr, flags);
}

int posix_timer_event(struct k_itimer *timr,int si_private)
{
	memset(&timr->sigq->info, 0, sizeof(siginfo_t));
	timr->sigq->info.si_sys_private = si_private;
	/*
	 * Send signal to the process that owns this timer.

	 * This code assumes that all the possible abs_lists share the
	 * same lock (there is only one list at this time). If this is
	 * not the case, the CLOCK info would need to be used to find
	 * the proper abs list lock.
	 */

	timr->sigq->info.si_signo = timr->it_sigev_signo;
	timr->sigq->info.si_errno = 0;
	timr->sigq->info.si_code = SI_TIMER;
	timr->sigq->info.si_tid = timr->it_id;
	timr->sigq->info.si_value = timr->it_sigev_value;
	if (timr->it_sigev_notify & SIGEV_THREAD_ID) {
		if (unlikely(timr->it_process->flags & PF_EXITING)) {
			timr->it_sigev_notify = SIGEV_SIGNAL;
			put_task_struct(timr->it_process);
			timr->it_process = timr->it_process->group_leader;
			goto group;
		}
		return send_sigqueue(timr->it_sigev_signo, timr->sigq,
			timr->it_process);
	}
	else {
	group:
		return send_group_sigqueue(timr->it_sigev_signo, timr->sigq,
			timr->it_process);
	}
}

/*
 * This function gets called when a POSIX.1b interval timer expires.  It
 * is used as a callback from the kernel internal timer.  The
 * run_timer_list code ALWAYS calls with interrupts on.

 * This code is for CLOCK_REALTIME* and CLOCK_MONOTONIC* timers.
 */
static void posix_timer_fn(unsigned long __data)
{
	struct k_itimer *timr = (struct k_itimer *) __data;
	unsigned long flags;
	unsigned long seq;
	struct timespec delta, new_wall_to;
	u64 exp = 0;
	int do_notify = 1;
	IF_HIGH_RES(long arch_cycle);

	spin_lock_irqsave(&timr->it_lock, flags);
	set_timer_inactive(timr);
	if (!list_empty(&timr->it.real.abs_timer_entry)) {
		spin_lock(&abs_list.lock);
		do {
			seq = read_seqbegin(&xtime_lock);
			new_wall_to =	wall_to_monotonic;
		} while (read_seqretry(&xtime_lock, seq));
		set_normalized_timespec(&delta,
					new_wall_to.tv_sec -
					timr->it.real.wall_to_prev.tv_sec,
					new_wall_to.tv_nsec -
					timr->it.real.wall_to_prev.tv_nsec);
		if (likely((delta.tv_sec | delta.tv_nsec ) == 0)) {
			/* do nothing, timer is on time */
		} else if (delta.tv_sec < 0) {
			/* do nothing, timer is already late */
		} else {
			/* timer is early due to a clock set */
			IF_HIGH_RES(arch_cycle = )
			tstojiffie(&delta,
				   posix_clocks[timr->it_clock].res,
				   &exp);
			timr->it.real.wall_to_prev = new_wall_to;
			timr->it.real.timer.expires += exp;
#ifdef CONFIG_HIGH_RES_TIMERS
			timr->it.real.timer.arch_cycle_expires += arch_cycle;
			normalize_jiffies(&timr->it.real.timer.expires,
					  &timr->it.real.timer.arch_cycle_expires);
#endif
			add_timer(&timr->it.real.timer);
			do_notify = 0;
		}
		spin_unlock(&abs_list.lock);

	}
	if (do_notify)  {
		int si_private=0;

		if (timr->it.real.incr IF_HIGH_RES( | timr->it_arch_cycle_incr))
			si_private = ++timr->it_requeue_pending;
		else {
			remove_from_abslist(timr);
		}

		if (posix_timer_event(timr, si_private))
			/*
			 * signal was not sent because of sig_ignor
			 * we will not get a call back to restart it AND
			 * it should be restarted.
			 */
			schedule_next_timer(timr);
	}
	unlock_timer(timr, flags); /* hold thru abs lock to keep irq off */
	wake_timer_waiters();
}


static inline struct task_struct * good_sigevent(sigevent_t * event)
{
	struct task_struct *rtn = current->group_leader;

	if ((event->sigev_notify & SIGEV_THREAD_ID ) &&
		(!(rtn = find_task_by_pid(event->sigev_notify_thread_id)) ||
		 rtn->tgid != current->tgid ||
		 (event->sigev_notify & ~SIGEV_THREAD_ID) != SIGEV_SIGNAL))
		return NULL;

	if (((event->sigev_notify & ~SIGEV_THREAD_ID) != SIGEV_NONE) &&
	    ((event->sigev_signo <= 0) || (event->sigev_signo > SIGRTMAX)))
		return NULL;

	return rtn;
}

void register_posix_clock(int clock_id, struct k_clock *new_clock)
{
	if ((unsigned) clock_id >= MAX_CLOCKS) {
		printk("POSIX clock register failed for clock_id %d\n",
		       clock_id);
		return;
	}
	posix_clocks[clock_id] = *new_clock;
}

static struct k_itimer * alloc_posix_timer(void)
{
	struct k_itimer *tmr;
	tmr = kmem_cache_alloc(posix_timers_cache, GFP_KERNEL);
	if (!tmr)
		return tmr;
	memset(tmr, 0, sizeof (struct k_itimer));
	INIT_LIST_HEAD(&tmr->it.real.abs_timer_entry);
	if (unlikely(!(tmr->sigq = sigqueue_alloc()))) {
		kmem_cache_free(posix_timers_cache, tmr);
		tmr = NULL;
	}
	return tmr;
}

#define IT_ID_SET	1
#define IT_ID_NOT_SET	0
static void release_posix_timer(struct k_itimer *tmr, int it_id_set)
{
	if (it_id_set) {
		unsigned long flags;
		spin_lock_irqsave(&idr_lock, flags);
		idr_remove(&posix_timers_id, tmr->it_id);
		spin_unlock_irqrestore(&idr_lock, flags);
	}
	sigqueue_free(tmr->sigq);
	if (unlikely(tmr->it_process) &&
	    tmr->it_sigev_notify == (SIGEV_SIGNAL|SIGEV_THREAD_ID))
		put_task_struct(tmr->it_process);
	kmem_cache_free(posix_timers_cache, tmr);
}

/* Create a POSIX.1b interval timer. */

asmlinkage long
sys_timer_create(clockid_t which_clock,
		 struct sigevent __user *timer_event_spec,
		 timer_t __user * created_timer_id)
{
	int error = 0;
	struct k_itimer *new_timer = NULL;
	int new_timer_id;
	struct task_struct *process = NULL;
	unsigned long flags;
	sigevent_t event;
	int it_id_set = IT_ID_NOT_SET;

	if ((unsigned) which_clock >= MAX_CLOCKS ||
				!posix_clocks[which_clock].res)
		return -EINVAL;

	new_timer = alloc_posix_timer();
	if (unlikely(!new_timer))
		return -EAGAIN;

	spin_lock_init(&new_timer->it_lock);
 retry:
	if (unlikely(!idr_pre_get(&posix_timers_id, GFP_KERNEL))) {
		error = -EAGAIN;
		goto out;
	}
	spin_lock_irq(&idr_lock);
	error = idr_get_new(&posix_timers_id,
			    (void *) new_timer,
			    &new_timer_id);
	spin_unlock_irq(&idr_lock);
	if (error == -EAGAIN)
		goto retry;
	else if (error) {
		/*
		 * Wierd looking, but we return EAGAIN if the IDR is
		 * full (proper POSIX return value for this)
		 */
		error = -EAGAIN;
		goto out;
	}

	it_id_set = IT_ID_SET;
	new_timer->it_id = (timer_t) new_timer_id;
	new_timer->it_clock = which_clock;
	new_timer->it.real.incr = 0;
	new_timer->it_overrun = -1;
	if (posix_clocks[which_clock].timer_create) {
		error =  posix_clocks[which_clock].timer_create(new_timer);
		if (error)
			goto out;
	} else {
		init_timer(&new_timer->it.real.timer);
		new_timer->it.real.timer.expires = 0;
		new_timer->it.real.timer.data = (unsigned long) new_timer;
		new_timer->it.real.timer.function = posix_timer_fn;
		set_timer_inactive(new_timer);
	}

	/*
	 * return the timer_id now.  The next step is hard to
	 * back out if there is an error.
	 */
	if (copy_to_user(created_timer_id,
			 &new_timer_id, sizeof (new_timer_id))) {
		error = -EFAULT;
		goto out;
	}
	if (timer_event_spec) {
		if (copy_from_user(&event, timer_event_spec, sizeof (event))) {
			error = -EFAULT;
			goto out;
		}
		new_timer->it_sigev_notify = event.sigev_notify;
		new_timer->it_sigev_signo = event.sigev_signo;
		new_timer->it_sigev_value = event.sigev_value;

		read_lock(&tasklist_lock);
		if ((process = good_sigevent(&event))) {
			/*
			 * We may be setting up this process for another
			 * thread.  It may be exiting.  To catch this
			 * case the we check the PF_EXITING flag.  If
			 * the flag is not set, the siglock will catch
			 * him before it is too late (in exit_itimers).
			 *
			 * The exec case is a bit more invloved but easy
			 * to code.  If the process is in our thread
			 * group (and it must be or we would not allow
			 * it here) and is doing an exec, it will cause
			 * us to be killed.  In this case it will wait
			 * for us to die which means we can finish this
			 * linkage with our last gasp. I.e. no code :)
			 */
			spin_lock_irqsave(&process->sighand->siglock, flags);
			if (!(process->flags & PF_EXITING)) {
				new_timer->it_process = process;
				list_add(&new_timer->list,
					 &process->signal->posix_timers);
				spin_unlock_irqrestore(
					&process->sighand->siglock, flags);
				if (new_timer->it_sigev_notify == 
				    (SIGEV_SIGNAL|SIGEV_THREAD_ID))
					get_task_struct(process);
			} else {
				spin_unlock_irqrestore(
					&process->sighand->siglock, flags);
				process = NULL;
			}
		}
		read_unlock(&tasklist_lock);
		if (!process) {
			error = -EINVAL;
			goto out;
		}
	} else {
		new_timer->it_sigev_notify = SIGEV_SIGNAL;
		new_timer->it_sigev_signo = SIGALRM;
		new_timer->it_sigev_value.sival_int = new_timer->it_id;
		process = current->group_leader;
		spin_lock_irqsave(&process->sighand->siglock, flags);
		new_timer->it_process = process;
		list_add(&new_timer->list, &process->signal->posix_timers);
		spin_unlock_irqrestore(&process->sighand->siglock, flags);
	}

 	/*
	 * In the case of the timer belonging to another task, after
	 * the task is unlocked, the timer is owned by the other task
	 * and may cease to exist at any time.  Don't use or modify
	 * new_timer after the unlock call.
	 */

out:
	if (error)
		release_posix_timer(new_timer, it_id_set);

	return error;
}

/*
 * good_timespec
 *
 * This function checks the elements of a timespec structure.
 *
 * Arguments:
 * ts	     : Pointer to the timespec structure to check
 *
 * Return value:
 * If a NULL pointer was passed in, or the tv_nsec field was less than 0
 * or greater than NSEC_PER_SEC, or the tv_sec field was less than 0,
 * this function returns 0. Otherwise it returns 1.
 */
static int good_timespec(const struct timespec *ts)
{
	if ((!ts) || (ts->tv_sec < 0) ||
			((unsigned) ts->tv_nsec >= NSEC_PER_SEC))
		return 0;
	return 1;
}

/*
 * Locking issues: We need to protect the result of the id look up until
 * we get the timer locked down so it is not deleted under us.  The
 * removal is done under the idr spinlock so we use that here to bridge
 * the find to the timer lock.  To avoid a dead lock, the timer id MUST
 * be release with out holding the timer lock.
 */
static struct k_itimer * lock_timer(timer_t timer_id, unsigned long *flags)
{
	struct k_itimer *timr;
	/*
	 * Watch out here.  We do a irqsave on the idr_lock and pass the
	 * flags part over to the timer lock.  Must not let interrupts in
	 * while we are moving the lock.
	 */

	spin_lock_irqsave(&idr_lock, *flags);
	timr = (struct k_itimer *) idr_find(&posix_timers_id, (int) timer_id);
	if (timr) {
		spin_lock(&timr->it_lock);
		spin_unlock(&idr_lock);

		if ((timr->it_id != timer_id) || !(timr->it_process) ||
				timr->it_process->tgid != current->tgid) {
			unlock_timer(timr, *flags);
			timr = NULL;
		}
	} else
		spin_unlock_irqrestore(&idr_lock, *flags);

	return timr;
}

/*
 * Get the time remaining on a POSIX.1b interval timer.  This function
 * is ALWAYS called with spin_lock_irq on the timer, thus it must not
 * mess with irq.
 *
 * We have a couple of messes to clean up here.  First there is the case
 * of a timer that has a requeue pending.  These timers should appear to
 * be in the timer list with an expiry as if we were to requeue them
 * now.
 *
 * The second issue is the SIGEV_NONE timer which may be active but is
 * not really ever put in the timer list (to save system resources).
 * This timer may be expired, and if so, we will do it here.  Otherwise
 * it is the same as a requeue pending timer WRT to what we should
 * report.
 */

#ifdef CONFIG_HIGH_RES_TIMERS 
struct now_struct zero_now = {0, 0};
#define timeleft (expire.jiffies | expire.arch_cycle)
#else
struct now_struct zero_now = {0};
#define timeleft (expire.jiffies)
#endif

static void
common_timer_get(struct k_itimer *timr, struct itimerspec *cur_setting)
{
	struct now_struct now, expire;

	do {
		get_expire(&expire, timr);
	} while ((volatile long) (timr->it.mmtimer.expires) != expire.jiffies);

	posix_get_now(&now);

	if (timeleft && 
	    ((timr->it_sigev_notify & ~SIGEV_THREAD_ID) == SIGEV_NONE) && 
	    !(timr->it.real.incr IF_HIGH_RES(| timr->it_arch_cycle_incr)) &&
	    posix_time_before(&timr->it.real.timer, &now)) {
		put_expire(&zero_now, timr);
		expire = zero_now;
	}
	if (timeleft) {
		if (timr->it_requeue_pending & REQUEUE_PENDING ||
		    (timr->it_sigev_notify & ~SIGEV_THREAD_ID) == SIGEV_NONE) {
			posix_bump_timer(timr, &now);
			get_expire(&expire, timr);
		}
		else
			if (!timer_pending(&timr->it.real.timer))
				expire = zero_now;
		if (timeleft) {
			sub_now(&expire, &now);
		}
	}
	jiffies_to_timespec(expire.jiffies, &cur_setting->it_value);
	jiffies_to_timespec(timr->it.real.incr, &cur_setting->it_interval);

#ifdef CONFIG_HIGH_RES_TIMERS 
	set_normalized_timespec(&cur_setting->it_value, 
				cur_setting->it_value.tv_sec, 
				cur_setting->it_value.tv_nsec +
				arch_cycle_to_nsec(expire.arch_cycle));

	set_normalized_timespec(&cur_setting->it_interval, 
				cur_setting->it_interval.tv_sec,
				cur_setting->it_interval.tv_nsec +
				arch_cycle_to_nsec(timr->it_arch_cycle_incr));
#endif
	if (cur_setting->it_value.tv_sec < 0) {
		cur_setting->it_value.tv_nsec = 1;
		cur_setting->it_value.tv_sec = 0;
	}
}

/* Get the time remaining on a POSIX.1b interval timer. */
asmlinkage long
sys_timer_gettime(timer_t timer_id, struct itimerspec __user *setting)
{
	struct k_itimer *timr;
	struct itimerspec cur_setting;
	unsigned long flags;

	timr = lock_timer(timer_id, &flags);
	if (!timr)
		return -EINVAL;

	CLOCK_DISPATCH(timr->it_clock, timer_get, (timr, &cur_setting));

	unlock_timer(timr, flags);

	if (copy_to_user(setting, &cur_setting, sizeof (cur_setting)))
		return -EFAULT;

	return 0;
}
/*
 * Get the number of overruns of a POSIX.1b interval timer.  This is to
 * be the overrun of the timer last delivered.  At the same time we are
 * accumulating overruns on the next timer.  The overrun is frozen when
 * the signal is delivered, either at the notify time (if the info block
 * is not queued) or at the actual delivery time (as we are informed by
 * the call back to do_schedule_next_timer().  So all we need to do is
 * to pick up the frozen overrun.
 */

asmlinkage long
sys_timer_getoverrun(timer_t timer_id)
{
	struct k_itimer *timr;
	int overrun;
	long flags;

	timr = lock_timer(timer_id, &flags);
	if (!timr)
		return -EINVAL;

	overrun = timr->it_overrun_last;
	unlock_timer(timr, flags);

	return overrun;
}
/*
 * Adjust for absolute time
 *
 * If absolute time is given and it is not CLOCK_MONOTONIC, we need to
 * adjust for the offset between the timer clock (CLOCK_MONOTONIC) and
 * what ever clock he is using.
 *
 * If it is relative time, we need to add the current (CLOCK_MONOTONIC)
 * time to it to get the proper time for the timer.
 *
 * err will be set to -EINVAL if offset is larger than MAX_JIFFY_OFFSET.
 * Caller should set err otherwise prior to call if he desires to test
 * this value.
 */
static int adjust_abs_time(struct k_clock *clock, 
			   struct timespec *tp, 
			   int abs, 
			   u64 *exp, 
			   struct timespec *wall_to,
			   int *err)
{
	struct timespec now;
	struct timespec oc = *tp;
	u64 jiffies_64_f;
	long arch_cycle;
	int rtn =0;

	if (abs) {
		/*
		 * The mask picks up the 4 basic clocks
		 */
		if ((clock - &posix_clocks[0]) <= CLOCK_MONOTONIC_HR) {
			jiffies_64_f = do_posix_clock_monotonic_gettime_parts(
				&now,  wall_to, &arch_cycle);
			/*
			 * If we are doing a MONOTONIC clock
			 */
			if(clock->clock_get ==
			    posix_clocks[CLOCK_MONOTONIC].clock_get){
				now.tv_sec += wall_to->tv_sec;
				now.tv_nsec += wall_to->tv_nsec;
			}
		} else {
			/*
			 * Not one of the basic clocks
			 */
			clock->clock_get(clock - posix_clocks, &now);
			jiffies_64_f = get_jiffies_64();
		}
		/*
		 * Take away now to get delta
		 */
		oc.tv_sec -= now.tv_sec;
		oc.tv_nsec -= now.tv_nsec;
		/*
		 * Normalize...
		 */
		timespec_norm(&oc);
		while ((oc.tv_nsec) < 0) {
			oc.tv_nsec += NSEC_PER_SEC;
			oc.tv_sec--;
		}
	}else{
#ifdef CONFIG_HIGH_RES_TIMERS
		do_atomic_on_xtime_seq(
			jiffies_64_f = jiffies_64;
			arch_cycle = get_arch_cycles((u32) jiffies_64_f);
			);
#else
		jiffies_64_f = get_jiffies_64();
#endif
	}
	/*
	 * Check if the requested time is prior to now (if so set now)
	 */
	if (oc.tv_sec < 0)
		oc.tv_sec = oc.tv_nsec = 0;

	if (oc.tv_sec | oc.tv_nsec) {
		oc.tv_nsec += clock->res;
		timespec_norm(&oc);
	}

	IF_HIGH_RES(rtn = arch_cycle + )
		tstojiffie(&oc, clock->res, exp);

	/*
	 * return the actual jiffies expire time, full 64 bits
	 */
#ifdef CONFIG_HIGH_RES_TIMERS
	/*
	 * If a low res clock don't confuse things with arch cycles.
	 */
	if (clock->res != tick_nsec) {
		while (rtn >= arch_cycles_per_jiffy) {
			rtn -= arch_cycles_per_jiffy;
			jiffies_64_f++;
		}
	} else {
		rtn = 0;
	}
#endif
	if (*exp > ((u64)MAX_JIFFY_OFFSET))
		*err = -EINVAL;

	*exp += jiffies_64_f;
	return rtn;
}

/* Set a POSIX.1b interval timer. */
/* timr->it_lock is taken. */
static inline int
common_timer_set(struct k_itimer *timr, int flags,
		 struct itimerspec *new_setting, struct itimerspec *old_setting)
{
	struct k_clock *clock = &posix_clocks[timr->it_clock];
	u64 expire_64;
	int rtn = 0;
	IF_HIGH_RES(long arch_cycle_expire;)

	if (old_setting)
		common_timer_get(timr, old_setting);

	/* disable the timer */
	timr->it.real.incr = 0;
	IF_HIGH_RES(timr->it_arch_cycle_incr = 0;)
	/*
	 * careful here.  If smp we could be in the "fire" routine which will
	 * be spinning as we hold the lock.  But this is ONLY an SMP issue.
	 */
#if defined(CONFIG_SMP) || defined(CONFIG_PREEMPT_SOFTIRQS)
	if (timer_active(timr) && !del_timer(&timr->it.real.timer)) {
		/*
		 * It can only be active if on an other cpu (unless RT).
		 * Since we have cleared the interval stuff above, it
		 * should clear once we release the spin lock.  Of
		 * course once we do that anything could happen,
		 * including the complete melt down of the timer.  So
		 * return with a "retry" exit status.  If RT we do a
		 * formal wait as the function code is fully
		 * preemptable...
		 */
		return TIMER_RETRY;
	}
	set_timer_inactive(timr);
#else
	del_timer(&timr->it.real.timer);
#endif
	remove_from_abslist(timr);

	timr->it_requeue_pending = (timr->it_requeue_pending + 2) & 
		~REQUEUE_PENDING;
	timr->it_overrun_last = 0;
	timr->it_overrun = -1;
	/*
	 *switch off the timer when it_value is zero
	 */
	if (!new_setting->it_value.tv_sec && !new_setting->it_value.tv_nsec) {
		timr->it.mmtimer.expires = 0;
		IF_HIGH_RES(timr->it.real.timer.arch_cycle_expires = 0;)
		return 0;
	}

	IF_HIGH_RES(arch_cycle_expire = )
		adjust_abs_time(clock, &new_setting->it_value, 
				flags & TIMER_ABSTIME, &expire_64, 
				&(timr->it.real.wall_to_prev), &rtn);
	/*
	 * Check if the requested time is more than the timer code
	 * can handle (if so we error out).
	 */
	if (rtn)
			/*
			 * This is a considered response, not exactly in
			 * line with the standard (in fact it is silent on
			 * possible overflows).  We assume such a large
			 * value is ALMOST always a programming error and
			 * try not to compound it by setting a really dumb
			 * value.
			 */
		return rtn;

	timr->it.mmtimer.expires = (unsigned long)expire_64;
	IF_HIGH_RES(timr->it.real.timer.arch_cycle_expires = arch_cycle_expire;)
	IF_HIGH_RES(timr->it_arch_cycle_incr =)
		tstojiffie(&new_setting->it_interval, clock->res, &expire_64);
#ifdef HIGH_RES_TIMERS
	if (clock->res == tick_nsec)
		timr->it_arch_cycle_incr = 0;
#endif
	timr->it.real.incr = (unsigned long)expire_64;

	/*
	 * We do not even queue SIGEV_NONE timers!  But we do put them
	 * in the abs list so we can do that right.
	 */
	if (((timr->it_sigev_notify & ~SIGEV_THREAD_ID) != SIGEV_NONE))
		add_timer(&timr->it.real.timer);

	if (flags & TIMER_ABSTIME && clock->abs_struct) {
		spin_lock(&clock->abs_struct->lock);
		list_add_tail(&(timr->it.real.abs_timer_entry),
			      &(clock->abs_struct->list));
		spin_unlock(&clock->abs_struct->lock);
	}
	return 0;
}

/* Set a POSIX.1b interval timer */
asmlinkage long
sys_timer_settime(timer_t timer_id, int flags,
		  const struct itimerspec __user *new_setting,
		  struct itimerspec __user *old_setting)
{
	struct k_itimer *timr;
	struct itimerspec new_spec, old_spec;
	int error = 0;
	long flag;
	struct itimerspec *rtn = old_setting ? &old_spec : NULL;

	if (!new_setting)
		return -EINVAL;

	if (copy_from_user(&new_spec, new_setting, sizeof (new_spec)))
		return -EFAULT;

	if ((!good_timespec(&new_spec.it_interval)) ||
	    (!good_timespec(&new_spec.it_value)))
		return -EINVAL;
retry:
	timr = lock_timer(timer_id, &flag);
	if (!timr)
		return -EINVAL;

	error = CLOCK_DISPATCH(timr->it_clock, timer_set,
			       (timr, flags, &new_spec, rtn));

	unlock_timer(timr, flag);
	if (error == TIMER_RETRY) {
		wait_for_timer(timr);
		rtn = NULL;	/* We already got the old time... */
		goto retry;
	}

	if (old_setting && !error && copy_to_user(old_setting,
						  &old_spec, sizeof (old_spec)))
		error = -EFAULT;

	return error;
}

static inline int common_timer_del(struct k_itimer *timer)
{
	timer->it.real.incr = 0;
	IF_HIGH_RES(timer->it_arch_cycle_incr = 0;)
#if defined(CONFIG_SMP) || defined(CONFIG_PREEMPT_SOFTIRQS)
	if (timer_active(timer) && !del_timer(&timer->it_timer)) {
		/*
		 * It can only be active if on an other cpu (unless RT).
		 * Since we have cleared the interval stuff above, it
		 * should clear once we release the spin lock.  Of
		 * course once we do that anything could happen,
		 * including the complete melt down of the timer.  So
		 * return with a "retry" exit status.  For RT we do a
		 * formal wait as it could take a while.
		 */
		return TIMER_RETRY;
	}
#else
	del_timer(&timer->it.real.timer);
#endif
	remove_from_abslist(timer);

	return 0;
}

static inline int timer_delete_hook(struct k_itimer *timer)
{
	return CLOCK_DISPATCH(timer->it_clock, timer_del, (timer));
}

/* Delete a POSIX.1b interval timer. */
asmlinkage long
sys_timer_delete(timer_t timer_id)
{
	struct k_itimer *timer;
	long flags;

#if defined(CONFIG_SMP) || defined(CONFIG_PREEMPT_SOFTIRQS)
	int error;
retry_delete:
#endif
	timer = lock_timer(timer_id, &flags);
	if (!timer)
		return -EINVAL;

#if defined(CONFIG_SMP) || defined(CONFIG_PREEMPT_SOFTIRQS)
	error = timer_delete_hook(timer);

	if (error == TIMER_RETRY) {
		unlock_timer(timer, flags);
		wait_for_timer(timer);
		goto retry_delete;
	}
#else
	timer_delete_hook(timer);
#endif
	spin_lock(&current->sighand->siglock);
	list_del(&timer->list);
	spin_unlock(&current->sighand->siglock);
	/*
	 * This keeps any tasks waiting on the spin lock from thinking
	 * they got something (see the lock code above).
	 */
	if (timer->it_process) {
		if (timer->it_sigev_notify == (SIGEV_SIGNAL|SIGEV_THREAD_ID))
			put_task_struct(timer->it_process);
		timer->it_process = NULL;
	}
	unlock_timer(timer, flags);
	release_posix_timer(timer, IT_ID_SET);
	return 0;
}
/*
 * return timer owned by the process, used by exit_itimers
 */
static inline void itimer_delete(struct k_itimer *timer)
{
	unsigned long flags;

#if defined(CONFIG_SMP) || defined(CONFIG_PREEMPT_SOFTIRQS)
	int error;
retry_delete:
#endif
	spin_lock_irqsave(&timer->it_lock, flags);

#if defined(CONFIG_SMP) || defined(CONFIG_PREEMPT_SOFTIRQS)
	error = timer_delete_hook(timer);

	if (error == TIMER_RETRY) {
		unlock_timer(timer, flags);
		wait_for_timer(timer);
		goto retry_delete;
	}
#else
	timer_delete_hook(timer);
#endif
	list_del(&timer->list);
	/*
	 * This keeps any tasks waiting on the spin lock from thinking
	 * they got something (see the lock code above).
	 */
	if (timer->it_process) {
		if (timer->it_sigev_notify == (SIGEV_SIGNAL|SIGEV_THREAD_ID))
			put_task_struct(timer->it_process);
		timer->it_process = NULL;
	}
	unlock_timer(timer, flags);
	release_posix_timer(timer, IT_ID_SET);
}

/*
 * This is called by __exit_signal, only when there are no more
 * references to the shared signal_struct.
 */
void exit_itimers(struct signal_struct *sig)
{
	struct k_itimer *tmr;

	while (!list_empty(&sig->posix_timers)) {
		tmr = list_entry(sig->posix_timers.next, struct k_itimer, list);
		itimer_delete(tmr);
	}
}

/*
 * And now for the "clock" calls
 *
 * These functions are called both from timer functions (with the timer
 * spin_lock_irq() held and from clock calls with no locking.	They must
 * use the save flags versions of locks.
 */
extern unsigned long wall_jiffies;

static void get_wall_time(struct timespec *tp)
{
#ifdef CONFIG_HIGH_RES_TIMERS
	*tp = xtime;
	tp->tv_nsec += arch_cycle_to_nsec(get_arch_cycles(wall_jiffies));
#else
	getnstimeofday(tp);
#endif
}


/*
 * We do ticks here to avoid the irq lock ( they take sooo long).
 * The seqlock is great here.  Since we a reader, we don't really care
 * if we are interrupted since we don't take lock that will stall us or
 * any other cpu. Voila, no irq lock is needed.
 *
 */

static u64 do_posix_clock_monotonic_gettime_parts(
		struct timespec *tp, struct timespec *mo, long *arch_cycle)
{
	u64 jiff;
	unsigned int seq;

	do {
		seq = read_seqbegin(&xtime_lock);
		get_wall_time(tp);
		*mo = wall_to_monotonic;
		jiff = jiffies_64;
		IF_HIGH_RES(*arch_cycle = get_arch_cycles((u32)jiff);)

	} while(read_seqretry(&xtime_lock, seq));

#ifdef CONFIG_HIGH_RES_TIMERS
	while(*arch_cycle >= arch_cycles_per_jiffy) {
		*arch_cycle -= arch_cycles_per_jiffy;
		jiff++;
	}
#endif

	return jiff;
}

int do_posix_clock_monotonic_gettime(struct timespec *tp)
{
	struct timespec wall_to_mono;
	long arch_cycle;

	do_posix_clock_monotonic_gettime_parts(tp, &wall_to_mono, &arch_cycle);

	tp->tv_sec += wall_to_mono.tv_sec;
	tp->tv_nsec += wall_to_mono.tv_nsec;

	timespec_norm(tp);
	return 0;
}

int do_posix_clock_nosettime(struct timespec *tp)
{
	return -EINVAL;
}

int do_posix_clock_notimer_create(struct k_itimer *timer)
{
	return -EINVAL;
}
EXPORT_SYMBOL_GPL(do_posix_clock_notimer_create);

int do_posix_clock_nonanosleep(clockid_t clock, int flags, struct timespec *t)
{
#ifndef ENOTSUP
	return -EOPNOTSUPP;	/* aka ENOTSUP in userland for POSIX */
#else  /*  parisc does define it separately.  */
	return -ENOTSUP;
#endif
}
EXPORT_SYMBOL_GPL(do_posix_clock_nonanosleep);

asmlinkage long
sys_clock_settime(clockid_t which_clock, const struct timespec __user *tp)
{
	struct timespec new_tp;

	if (invalid_clockid(which_clock))
		return -EINVAL;
	if (copy_from_user(&new_tp, tp, sizeof (*tp)))
		return -EFAULT;
	return CLOCK_DISPATCH(which_clock, clock_set, (which_clock, &new_tp));
}

asmlinkage long
sys_clock_gettime(clockid_t which_clock, struct timespec __user *tp)
{
	struct timespec kernel_tp;
	int error;

	if (invalid_clockid(which_clock))
		return -EINVAL;
	error = CLOCK_DISPATCH(which_clock, clock_get,
			       (which_clock, &kernel_tp));
	if (!error && copy_to_user(tp, &kernel_tp, sizeof (kernel_tp)))
		error = -EFAULT;

	return error;

}

asmlinkage long
sys_clock_getres(clockid_t which_clock, struct timespec __user *tp)
{
	struct timespec rtn_tp;
	int error;

	if (invalid_clockid(which_clock))
		return -EINVAL;

	error = CLOCK_DISPATCH(which_clock, clock_getres,
			       (which_clock, &rtn_tp));

	if (!error && tp && copy_to_user(tp, &rtn_tp, sizeof (rtn_tp))) {
		error = -EFAULT;
	}
 
	return error;

}

static void nanosleep_wake_up(unsigned long __data)
{
	struct task_struct *p = (struct task_struct *) __data;
	wake_up_process(p);
}

/*
 * The standard says that an absolute nanosleep call MUST wake up at
 * the requested time in spite of clock settings.  Here is what we do:
 * For each nanosleep call that needs it (only absolute and not on
 * CLOCK_MONOTONIC* (as it can not be set)) we thread a little structure
 * into the "nanosleep_abs_list".  All we need is the task_struct pointer.
 * When ever the clock is set we just wake up all those tasks.	 The rest
 * is done by the while loop in clock_nanosleep().
 *
 * On locking, clock_was_set() is called from update_wall_clock which
 * holds (or has held for it) a write_lock_irq( xtime_lock) and is
 * called from the timer bh code.  Thus we need the irq save locks.
 *
 * Also, on the call from update_wall_clock, that is done as part of a
 * softirq thing.  We don't want to delay the system that much (possibly
 * long list of timers to fix), so we defer that work to keventd.
 */

static DECLARE_WAIT_QUEUE_HEAD(nanosleep_abs_wqueue);
static DECLARE_WORK(clock_was_set_work, (void(*)(void*))clock_was_set, NULL);

static DECLARE_MUTEX(clock_was_set_lock);

void clock_was_set(void)
{
	struct k_itimer *timr;
	struct timespec new_wall_to;
	LIST_HEAD(cws_list);
	unsigned long seq;


	if (unlikely(in_interrupt())) {
		schedule_work(&clock_was_set_work);
		return;
	}
	wake_up_all(&nanosleep_abs_wqueue);

	/*
	 * Check if there exist TIMER_ABSTIME timers to correct.
	 *
	 * Notes on locking: This code is run in task context with irq
	 * on.  We CAN be interrupted!  All other usage of the abs list
	 * lock is under the timer lock which holds the irq lock as
	 * well.  We REALLY don't want to scan the whole list with the
	 * interrupt system off, AND we would like a sequence lock on
	 * this code as well.  Since we assume that the clock will not
	 * be set often, it seems ok to take and release the irq lock
	 * for each timer.  In fact add_timer will do this, so this is
	 * not an issue.  So we know when we are done, we will move the
	 * whole list to a new location.  Then as we process each entry,
	 * we will move it to the actual list again.  This way, when our
	 * copy is empty, we are done.  We are not all that concerned
	 * about preemption so we will use a semaphore lock to protect
	 * aginst reentry.  This way we will not stall another
	 * processor.  It is possible that this may delay some timers
	 * that should have expired, given the new clock, but even this
	 * will be minimal as we will always update to the current time,
	 * even if it was set by a task that is waiting for entry to
	 * this code.  Timers that expire too early will be caught by
	 * the expire code and restarted.

	 * Absolute timers that repeat are left in the abs list while
	 * waiting for the task to pick up the signal.  This means we
	 * may find timers that are not in the "add_timer" list, but are
	 * in the abs list.  We do the same thing for these, save
	 * putting them back in the "add_timer" list.  (Note, these are
	 * left in the abs list mainly to indicate that they are
	 * ABSOLUTE timers, a fact that is used by the re-arm code, and
	 * for which we have no other flag.)

	 */

	down(&clock_was_set_lock);
	spin_lock_irq(&abs_list.lock);
	list_splice_init(&abs_list.list, &cws_list);
	spin_unlock_irq(&abs_list.lock);
	do {
		do {
			seq = read_seqbegin(&xtime_lock);
			new_wall_to =	wall_to_monotonic;
		} while (read_seqretry(&xtime_lock, seq));

		spin_lock_irq(&abs_list.lock);
		if (list_empty(&cws_list)) {
			spin_unlock_irq(&abs_list.lock);
			break;
		}
		timr = list_entry(cws_list.next, struct k_itimer,
				  it.real.abs_timer_entry);

		list_del_init(&timr->it.real.abs_timer_entry);
		if (add_clockset_delta(timr, &new_wall_to) &&
		    del_timer(&timr->it.real.timer))  /* timer run yet? */
			/*
			 * Note that we only do this if the timer is/was
			 * in the list.  If it happens to be active an
			 * not in the timer list, it must be in the call
			 * back function, we leave it to that code to do
			 * the right thing.  I.e we do NOT need
			 * del_timer_sync()
			 */
			add_timer(&timr->it.real.timer);
		list_add(&timr->it.real.abs_timer_entry, &abs_list.list);
		spin_unlock_irq(&abs_list.lock);
	} while (1);

	up(&clock_was_set_lock);
}

long clock_nanosleep_restart(struct restart_block *restart_block);


asmlinkage long
sys_clock_nanosleep(clockid_t which_clock, int flags,
		    const struct timespec __user *rqtp,
		    struct timespec __user *rmtp)
{
	struct timespec t;
	struct restart_block *restart_block =
	    &(current_thread_info()->restart_block);
	int ret;

	if (invalid_clockid(which_clock))
		return -EINVAL;

	if (copy_from_user(&t, rqtp, sizeof (struct timespec)))
		return -EFAULT;

	if ((unsigned) t.tv_nsec >= NSEC_PER_SEC || t.tv_sec < 0)
		return -EINVAL;

	/*
	 * Do this here as nsleep function does not have the real address.
	 */
	restart_block->arg1 = (unsigned long)rmtp;

	ret = CLOCK_DISPATCH(which_clock, nsleep, (which_clock, flags, &t));

	if ((ret == -ERESTART_RESTARTBLOCK) && rmtp &&
					copy_to_user(rmtp, &t, sizeof (t)))
		return -EFAULT;
	return ret;
}
#ifdef CONFIG_HIGH_RES_TIMERS
#define get_jiffies_time(result)				\
({								\
	unsigned int seq;					\
	u64 jiff;						\
	do {							\
		seq = read_seqbegin(&xtime_lock);		\
		jiff = jiffies_64;				\
		*result = get_arch_cycles((unsigned long)jiff);	\
	} while(read_seqretry(&xtime_lock, seq));		\
        while(*result >= arch_cycles_per_jiffy) {		\
                *result -= arch_cycles_per_jiffy;		\
                jiff++;						\
        }							\
	jiff;							\
})

#else
#define get_jiffies_time(result) get_jiffies_64()

#endif


static int common_nsleep(clockid_t which_clock,
			 int flags, struct timespec *tsave)
{
	struct timespec t, dum;
	struct timer_list new_timer;
	DECLARE_WAITQUEUE(abs_wqueue, current);
	u64 rq_time = (u64)0;
	s64 left, jiff_u64_f;
	IF_HIGH_RES(long arch_cycle_left;)
	int abs;
	struct restart_block *restart_block =
	    &(current_thread_info()->restart_block);

	abs_wqueue.flags = 0;
	init_timer(&new_timer);
	new_timer.expires = 0;
	new_timer.data = (unsigned long) current;
	new_timer.function = nanosleep_wake_up;
	abs = flags & TIMER_ABSTIME;

	if (restart_block->fn == clock_nanosleep_restart) {
		/*
		 * Interrupted by a non-delivered signal, pick up remaining
		 * time and continue.  Remaining time is in arg2 & 3.
		 */
		restart_block->fn = do_no_restart_syscall;

		rq_time = restart_block->arg3;
		rq_time = (rq_time << 32) + restart_block->arg2;
		if (!rq_time)
			return -EINTR;
		IF_HIGH_RES(new_timer.arch_cycle_expires = restart_block->arg4;)
		left = rq_time - get_jiffies_time(&arch_cycle_left);


#ifndef CONFIG_HIGH_RES_TIMERS
		if (left <= (s64)0)
			return 0;	/* Already passed */
#else
		if ((left < (s64)0) || 
		    ((left == (s64)0) && (new_timer.arch_cycle_expires <= arch_cycle_left)))
			return 0;
#endif
	}

	if (abs && (posix_clocks[which_clock].clock_get !=
			    posix_clocks[CLOCK_MONOTONIC].clock_get))
		add_wait_queue(&nanosleep_abs_wqueue, &abs_wqueue);

	do {
		t = *tsave;
		if (abs || !(rq_time IF_HIGH_RES(| new_timer.arch_cycle_expires))) {
			int dum2;
			IF_HIGH_RES(new_timer.arch_cycle_expires =)
			adjust_abs_time(&posix_clocks[which_clock], &t, abs,
					&rq_time, &dum, &dum2);
		}

		jiff_u64_f = get_jiffies_time(&arch_cycle_left);
		left = rq_time - jiff_u64_f;
		if (left >= (s64)MAX_JIFFY_OFFSET)
			left = (s64)MAX_JIFFY_OFFSET;
#ifndef CONFIG_HIGH_RES_TIMERS
		if (left < (s64)0)
			break;
#else
		if ((left < (s64)0) || 
		    ((left == (s64)0) && (new_timer.arch_cycle_expires <= arch_cycle_left)))
			break;
#endif

		new_timer.expires = (unsigned long)jiff_u64_f + left;
		__set_current_state(TASK_INTERRUPTIBLE);
		add_timer(&new_timer);
		schedule();

		del_timer_sync(&new_timer);
		left = rq_time - get_jiffies_time(&arch_cycle_left);
	} while ((left > (s64)0  
#ifdef CONFIG_HIGH_RES_TIMERS
		  || (left == (s64)0 && (new_timer.arch_cycle_expires > arch_cycle_left))
#endif
			 ) &&  !test_thread_flag(TIF_SIGPENDING));

	if (abs_wqueue.task_list.next)
		finish_wait(&nanosleep_abs_wqueue, &abs_wqueue);


#ifdef CONFIG_HIGH_RES_TIMERS
	if (left > (s64)0 || 
	    (left == (s64)0 && (new_timer.arch_cycle_expires > arch_cycle_left)))
#else
	if (left > (s64)0 )
#endif
	{
		/*
		 * Always restart abs calls from scratch to pick up any
		 * clock shifting that happened while we are away.
		 */
		if (abs)
			return -ERESTARTNOHAND;

		left *= TICK_NSEC;
#ifdef CONFIG_HIGH_RES_TIMERS
		left += arch_cycle_to_nsec(new_timer.arch_cycle_expires - arch_cycle_left);
#endif
		tsave->tv_sec = div_long_long_rem(left, 
						  NSEC_PER_SEC, 
						  &tsave->tv_nsec);
		/*
		 * Restart works by saving the time remaing in 
		 * arg2 & 3 (it is 64-bits of jiffies).  The other
		 * info we need is the clock_id (saved in arg0). 
		 * The sys_call interface needs the users 
		 * timespec return address which _it_ saves in arg1.
		 * Since we have cast the nanosleep call to a clock_nanosleep
		 * both can be restarted with the same code.
		 */
		restart_block->fn = clock_nanosleep_restart;
		restart_block->arg0 = which_clock;
		/*
		 * Caller sets arg1
		 */
		restart_block->arg2 = rq_time & 0xffffffffLL;
		restart_block->arg3 = rq_time >> 32;
		IF_HIGH_RES(restart_block->arg4 = new_timer.arch_cycle_expires;)

		return -ERESTART_RESTARTBLOCK;
	}
	return 0;
}
/*
 * This will restart clock_nanosleep.
 */
long
clock_nanosleep_restart(struct restart_block *restart_block)
{
	struct timespec t;
	int ret = common_nsleep(restart_block->arg0, 0, &t);

	if ((ret == -ERESTART_RESTARTBLOCK) && restart_block->arg1 &&
	    copy_to_user((struct timespec __user *)(restart_block->arg1), &t,
			 sizeof (t)))
		return -EFAULT;
	return ret;
}
