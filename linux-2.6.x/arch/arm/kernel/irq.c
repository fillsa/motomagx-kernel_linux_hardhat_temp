/*
 *  linux/arch/arm/kernel/irq.c
 *
 *  Copyright (C) 1992 Linus Torvalds
 *  Modifications for ARM processor Copyright (C) 1995-2000 Russell King.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  This file contains the code used by various IRQ handling routines:
 *  asking for different IRQ's should be done through these routines
 *  instead of just grabbing them. Thus setups with different IRQ numbers
 *  shouldn't result in any weird surprises, and installing new handlers
 *  should be easier.
 *
 *  IRQ's are in fact implemented a bit like signal handlers for the kernel.
 *  Naturally it's not a 1:1 relation, but there are similarities.
 */
/*
 * Copyright (C) 2007 Motorola, Inc.
 *
 * Date           Author            Comment
 * ===========  ==========  ====================================
 * 10/15/2007   Motorola    FIQ related modified.
 */

#include <linux/config.h>
#include <linux/kernel_stat.h>
#include <linux/module.h>
#include <linux/signal.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/ptrace.h>
#include <linux/slab.h>
#include <linux/random.h>
#include <linux/smp.h>
#include <linux/init.h>
#include <linux/seq_file.h>
#include <linux/errno.h>
#include <linux/list.h>
#include <linux/kallsyms.h>
#include <linux/proc_fs.h>
#include <linux/kthread.h>
#include <linux/spinlock.h>
#include <linux/ilatency.h>
#include <linux/ltt-events.h>
#include <linux/vst.h>

#include <asm/irq.h>
#include <asm/system.h>
#include <asm/mach/irq.h>

/*
 * Maximum IRQ count.  Currently, this is arbitary.  However, it should
 * not be set too low to prevent false triggering.  Conversely, if it
 * is set too high, then you could miss a stuck IRQ.
 *
 * Maybe we ought to set a timer and re-enable the IRQ at a later time?
 */
#define MAX_IRQ_CNT	100000

static int
__do_irq(unsigned int irq, struct irqaction *action, struct pt_regs *regs);

static int noirqdebug;
static volatile unsigned long irq_err_count;
static DEFINE_RAW_SPINLOCK(irq_controller_lock); //static DEFINE_SPINLOCK(irq_controller_lock);
static LIST_HEAD(irq_pending);

struct irqdesc irq_desc[NR_IRQS];
void (*init_arch_irq)(void) __initdata = NULL;

extern asmlinkage long sys_sched_setscheduler(pid_t pid, int policy,
                                       struct sched_param __user *param);
#ifdef CONFIG_PREEMPT_HARDIRQS
int hardirq_preemption = 1;
void direct_timer_interrupt(struct pt_regs *regs);
#endif

struct timer_update_handler timer_update = {
	.function	= NULL,
	.skip		= 0,
};

/*
 * No architecture-specific irq_finish function defined in arm/arch/irqs.h.
 */
#ifndef irq_finish
#define irq_finish(irq) do { } while (0)
#endif

/*
 * No architecture-specific irq_finish function defined in arm/arch/irqs.h.
 */
#ifndef irq_finish
#define irq_finish(irq) do { } while (0)
#endif

/*
 * Dummy mask/unmask handler
 */
void dummy_mask_unmask_irq(unsigned int irq)
{
}

irqreturn_t no_action(int irq, void *dev_id, struct pt_regs *regs)
{
	return IRQ_NONE;
}

void do_bad_IRQ(unsigned int irq, struct irqdesc *desc, struct pt_regs *regs)
{
	irq_err_count += 1;
	printk(KERN_ERR "IRQ: spurious interrupt %d\n", irq);
}

static struct irqchip bad_chip = {
	.ack	= dummy_mask_unmask_irq,
	.mask	= dummy_mask_unmask_irq,
	.unmask = dummy_mask_unmask_irq,
};

static struct irqdesc bad_irq_desc = {
	.chip		= &bad_chip,
	.handle		= do_bad_IRQ,
	.pend		= LIST_HEAD_INIT(bad_irq_desc.pend),
	.disable_depth	= 1,
};

#ifdef CONFIG_SMP
void synchronize_irq(unsigned int irq)
{
	struct irqdesc *desc = irq_desc + irq;

	while (desc->running)
		barrier();
}
EXPORT_SYMBOL(synchronize_irq);

#define smp_set_running(desc)	do { desc->running = 1; } while (0)
#define smp_clear_running(desc)	do { desc->running = 0; } while (0)
#else
#define smp_set_running(desc)	do { } while (0)
#define smp_clear_running(desc)	do { } while (0)
#endif

/**
 *	disable_irq_nosync - disable an irq without waiting
 *	@irq: Interrupt to disable
 *
 *	Disable the selected interrupt line.  Enables and disables
 *	are nested.  We do this lazily.
 *
 *	This function may be called from IRQ context.
 */
void disable_irq_nosync(unsigned int irq)
{
	struct irqdesc *desc = irq_desc + irq;
	unsigned long flags;

	spin_lock_irqsave(&irq_controller_lock, flags);
	desc->disable_depth++;
	list_del_init(&desc->pend);
	spin_unlock_irqrestore(&irq_controller_lock, flags);
}
EXPORT_SYMBOL(disable_irq_nosync);

/**
 *	disable_irq - disable an irq and wait for completion
 *	@irq: Interrupt to disable
 *
 *	Disable the selected interrupt line.  Enables and disables
 *	are nested.  This functions waits for any pending IRQ
 *	handlers for this interrupt to complete before returning.
 *	If you use this function while holding a resource the IRQ
 *	handler may need you will deadlock.
 *
 *	This function may be called - with care - from IRQ context.
 */
void disable_irq(unsigned int irq)
{
	struct irqdesc *desc = irq_desc + irq;

	disable_irq_nosync(irq);
	if (desc->action)
		synchronize_irq(irq);
}
EXPORT_SYMBOL(disable_irq);

/**
 *	enable_irq - enable interrupt handling on an irq
 *	@irq: Interrupt to enable
 *
 *	Re-enables the processing of interrupts on this IRQ line.
 *	Note that this may call the interrupt handler, so you may
 *	get unexpected results if you hold IRQs disabled.
 *
 *	This function may be called from IRQ context.
 */
void enable_irq(unsigned int irq)
{
	struct irqdesc *desc = irq_desc + irq;
	unsigned long flags;

	spin_lock_irqsave(&irq_controller_lock, flags);
	if (unlikely(!desc->disable_depth)) {
		printk("enable_irq(%u) unbalanced from %p\n", irq,
			__builtin_return_address(0));
	} else if (!--desc->disable_depth) {
		desc->probing = 0;
		desc->chip->unmask(irq);

		/*
		 * If the interrupt is waiting to be processed,
		 * try to re-run it.  We can't directly run it
		 * from here since the caller might be in an
		 * interrupt-protected region.
		 */
		if (desc->pending && list_empty(&desc->pend)) {
			desc->pending = 0;
			if (!desc->chip->retrigger ||
			    desc->chip->retrigger(irq))
				list_add(&desc->pend, &irq_pending);
		}

		smp_clear_running(desc);
	}
	spin_unlock_irqrestore(&irq_controller_lock, flags);
}
EXPORT_SYMBOL(enable_irq);

/*
 * Enable wake on selected irq
 */
void enable_irq_wake(unsigned int irq)
{
	struct irqdesc *desc = irq_desc + irq;
	unsigned long flags;

	spin_lock_irqsave(&irq_controller_lock, flags);
	if (desc->chip->wake)
		desc->chip->wake(irq, 1);
	spin_unlock_irqrestore(&irq_controller_lock, flags);
}
EXPORT_SYMBOL(enable_irq_wake);

void disable_irq_wake(unsigned int irq)
{
	struct irqdesc *desc = irq_desc + irq;
	unsigned long flags;

	spin_lock_irqsave(&irq_controller_lock, flags);
	if (desc->chip->wake)
		desc->chip->wake(irq, 0);
	spin_unlock_irqrestore(&irq_controller_lock, flags);
}
EXPORT_SYMBOL(disable_irq_wake);

int show_interrupts(struct seq_file *p, void *v)
{
	int i = *(loff_t *) v, cpu;
	struct irqaction * action;
	unsigned long flags;

	if (i == 0) {
		char cpuname[12];

		seq_printf(p, "    ");
		for_each_present_cpu(cpu) {
			sprintf(cpuname, "CPU%d", cpu);
			seq_printf(p, " %10s", cpuname);
		}
		seq_putc(p, '\n');
	}

	if (i < NR_IRQS) {
		spin_lock_irqsave(&irq_controller_lock, flags);
	    	action = irq_desc[i].action;
		if (!action)
			goto unlock;

		seq_printf(p, "%3d: ", i);
		for_each_present_cpu(cpu)
			seq_printf(p, "%10u ", kstat_cpu(cpu).irqs[i]);
		seq_printf(p, "  %s", action->name);
		for (action = action->next; action; action = action->next)
			seq_printf(p, ", %s", action->name);

		seq_putc(p, '\n');
unlock:
		spin_unlock_irqrestore(&irq_controller_lock, flags);
	} else if (i == NR_IRQS) {
#ifdef CONFIG_ARCH_ACORN
		show_fiq_list(p, v);
#endif
#ifdef CONFIG_SMP
		show_ipi_list(p);
#endif
		seq_printf(p, "Err: %10lu\n", irq_err_count);
	}
	return 0;
}

#ifdef CONFIG_LTT

int ltt_snapshot_irqs(int rchan)
{
	int i, count, relay_error;
	unsigned long flags;
	char buf[256];
	struct irqaction *action;

	for (i = relay_error = 0; i < NR_IRQS; i++) {
		spin_lock_irqsave(&irq_controller_lock, flags);
		if ( (action = irq_desc[i].action) == NULL) {
			spin_unlock_irqrestore(&irq_controller_lock, flags);
			continue;
		}
		count = snprintf(buf, sizeof(buf),
		                "IRQ: %d; NAME: %s\n", i, action->name);
		if (relay_write(rchan, buf, count, -1, NULL) != count) {
			relay_error = 1;
			goto relay_write_err;
		}
		for (action = action->next; action; action = action->next) {
			count = snprintf(buf, sizeof(buf),	
				"IRQ: %d; NAME: %s\n", i, action->name);
			if (relay_write(rchan, buf, count, -1, NULL) != count) {
				relay_error = 1;
				goto relay_write_err;
			}

		smp_clear_running(desc);
		}
		spin_unlock_irqrestore(&irq_controller_lock, flags);	
	}

relay_write_err :
	if (relay_error) {
		spin_unlock_irqrestore(&irq_controller_lock, flags);
		printk(KERN_ALERT "Can't write into a relayfs channel\n");
		return -ENOMEM;
	}

	return 0;
}

#endif

/*
 * IRQ lock detection.
 *
 * Hopefully, this should get us out of a few locked situations.
 * However, it may take a while for this to happen, since we need
 * a large number if IRQs to appear in the same jiffie with the
 * same instruction pointer (or within 2 instructions).
 */
static int check_irq_lock(struct irqdesc *desc, int irq, struct pt_regs *regs)
{
	unsigned long instr_ptr = instruction_pointer(regs);

	if (desc->lck_jif == jiffies &&
	    desc->lck_pc >= instr_ptr && desc->lck_pc < instr_ptr + 8) {
		desc->lck_cnt += 1;

		if (desc->lck_cnt > MAX_IRQ_CNT) {
			printk(KERN_ERR "IRQ LOCK: IRQ%d is locking the system, disabled\n", irq);
			return 1;
		}
	} else {
		desc->lck_cnt = 0;
		desc->lck_pc  = instruction_pointer(regs);
		desc->lck_jif = jiffies;
	}
	return 0;
}

static void
report_bad_irq(unsigned int irq, struct pt_regs *regs, struct irqdesc *desc, int ret)
{
	static int count = 100;
	struct irqaction *action;

	if (!count || noirqdebug)
		return;

	count--;

	if (ret != IRQ_HANDLED && ret != IRQ_NONE) {
		printk("irq%u: bogus retval mask %x\n", irq, ret);
	} else {
		printk("irq%u: nobody cared\n", irq);
	}
	show_regs(regs);
	dump_stack();
	printk(KERN_ERR "handlers:");
	action = desc->action;
	do {
		printk("\n" KERN_ERR "[<%p>]", action->handler);
		print_symbol(" (%s)", (unsigned long)action->handler);
		action = action->next;
	} while (action);
	printk("\n");
}

static void do_hardirq(struct irqdesc *desc)
{
	struct irqaction * action;
	unsigned int irq = desc - irq_desc;
                                                                                                                    
	local_irq_disable();
                                                                                                                    
	if (desc->status & IRQ_INPROGRESS) {
		action = desc->action;
		spin_lock(&irq_controller_lock);
		for (;;) {
			irqreturn_t action_ret = 0;
                                                                                                                    
			if (action) {
				action_ret = __do_irq(irq, action, NULL);
				spin_unlock(&irq_controller_lock);
				local_irq_enable();
				cond_resched();
				spin_lock_irq(&irq_controller_lock);
			}

			if (likely(!(desc->status & IRQ_PENDING)))
				break;
			desc->status &= ~IRQ_PENDING;
		}
		desc->status &= ~IRQ_INPROGRESS;
		desc->running = 0;
                /*
                 * The ->end() handler has to deal with interrupts which got
                 * disabled while the handler was running.
                 */
		desc->chip->unmask(irq);
		spin_unlock(&irq_controller_lock);
	}
	local_irq_enable();
}

extern asmlinkage void __do_softirq(void );

static int curr_irq_prio = 49;
                                                                                                                    
static int do_irqd(void * __desc)
{
	struct sched_param param = { 0, };
	struct irqdesc *desc = __desc;
#ifdef CONFIG_SMP
	int irq = desc - irq_desc;
	cpumask_t mask;
                                                                                                                    
	mask = cpumask_of_cpu(any_online_cpu(irq_affinity[irq]));
	set_cpus_allowed(current, mask);
#endif
	current->flags |= PF_NOFREEZE | PF_HARDIRQ;
                                                                                                                    
        /*
         * Scale irq thread priorities from prio 50 to prio 25
         */
	param.sched_priority = curr_irq_prio;
	if (param.sched_priority > 25)
		curr_irq_prio = param.sched_priority - 1;
                                                                                                                    
	sys_sched_setscheduler(current->pid, SCHED_FIFO, &param);
                                                                                                                    
	while (!kthread_should_stop()) {
		set_current_state(TASK_INTERRUPTIBLE);
		do_hardirq(desc);
		cond_resched();
		__do_softirq();
#ifdef CONFIG_SMP
                /*
                 * Did IRQ affinities change?
                 */
		if (!cpu_isset(smp_processor_id(), irq_affinity[irq])) {
			mask = cpumask_of_cpu(any_online_cpu(irq_affinity[irq]));
			set_cpus_allowed(current, mask);
		}
#endif
		schedule();
	}
	__set_current_state(TASK_RUNNING);
	return 0;
}

void recalculate_desc_flags(struct irqdesc *desc)
{
	struct irqaction *action;

	desc->status &= ~IRQ_NODELAY;
	for (action = desc->action ; action; action = action->next)
		if (action->flags & SA_NODELAY)
			desc->status |= IRQ_NODELAY;
}
static int ok_to_create_irq_threads;

static int start_irq_thread(int irq, struct irqdesc *desc)
{
	if (desc->thread || !ok_to_create_irq_threads)
		return 0;
                                                                                                                    
	desc->thread = kthread_create(do_irqd, desc, "IRQ %d", irq);
	if (!desc->thread) {
		printk(KERN_ERR "irqd: could not create IRQ thread %d!\n", irq);
		return -ENOMEM;
	}
                                                                                                                    
        /*
         * An interrupt may have come in before the thread pointer was
         * stored in desc->thread; make sure the thread gets woken up in
         * such a case:
         */
#ifdef CONFIG_SMP
	smp_mb();
#endif                 
                                                                                                   
	if (desc->status & IRQ_INPROGRESS)
		wake_up_process(desc->thread);
                                                                                                                    
	return 0;
}

int redirect_hardirq(struct irqdesc *desc)
{
        /*
         * Direct execution:
         */
	if ((desc->status & IRQ_NODELAY) || !desc->thread)
		return 0;
                                                                                                                    
	if (desc->thread && desc->thread->state != TASK_RUNNING)
		wake_up_process(desc->thread);
	
	return 1;
}


static int
__do_irq(unsigned int irq, struct irqaction *action, struct pt_regs *regs)
{
	unsigned int status;
	int ret, retval = 0;

	spin_unlock(&irq_controller_lock);

	if (!hardirq_count() || !(action->flags & SA_INTERRUPT))
		local_irq_enable();
	interrupt_overhead_stop();

	status = 0;
	do {
		ret = action->handler(irq, action->dev_id, regs);
		if (ret == IRQ_HANDLED)
			status |= action->flags;
		retval |= ret;
		action = action->next;
	} while (action);

	if (status & SA_SAMPLE_RANDOM)
		add_interrupt_randomness(irq);

#ifdef CONFIG_NO_IDLE_HZ
	if (timer_update.function && irq != timer_update.skip)
		timer_update.function(irq, 0, regs);
#endif

	spin_lock_irq(&irq_controller_lock);

	return retval;
}

/*
 * This is for software-decoded IRQs.  The caller is expected to
 * handle the ack, clear, mask and unmask issues.
 */
void
do_simple_IRQ(unsigned int irq, struct irqdesc *desc, struct pt_regs *regs)
{
	struct irqaction *action;
	const unsigned int cpu = smp_processor_id();

	desc->triggered = 1;

#ifdef CONFIG_PREEMPT_HARDIRQS
	if (unlikely(irq == ARCH_TIMER_IRQ))
		direct_timer_interrupt(regs);
#endif

	kstat_cpu(cpu).irqs[irq]++;

	smp_set_running(desc);

#ifdef CONFIG_PREEMPT_HARDIRQS
	if (desc->action) desc->status |= IRQ_INPROGRESS;
	if (redirect_hardirq(desc))
		return;
#endif

	action = desc->action;
	if (action) {
		int ret = __do_irq(irq, action, regs);
		if (ret != IRQ_HANDLED)
			report_bad_irq(irq, regs, desc, ret);
	}

	smp_clear_running(desc);
}

/*
 * Most edge-triggered IRQ implementations seem to take a broken
 * approach to this.  Hence the complexity.
 */
void
do_edge_IRQ(unsigned int irq, struct irqdesc *desc, struct pt_regs *regs)
{
	const unsigned int cpu = smp_processor_id();

	desc->triggered = 1;

	/*
	 * If we're currently running this IRQ, or its disabled,
	 * we shouldn't process the IRQ.  Instead, turn on the
	 * hardware masks.
	 */
	if (unlikely(desc->running || desc->disable_depth))
		goto running;

	/*
	 * Acknowledge and clear the IRQ, but don't mask it.
	 */
	desc->chip->ack(irq);

	/*
	 * Mark the IRQ currently in progress.
	 */
	desc->running = 1;

	kstat_cpu(cpu).irqs[irq]++;

#ifdef CONFIG_PREEMPT_HARDIRQS
	if (desc->action) desc->status |= IRQ_INPROGRESS;
	if (redirect_hardirq(desc))
		return;
#endif

	do {
		struct irqaction *action;

		action = desc->action;
		if (!action)
			break;

		if (desc->pending && !desc->disable_depth) {
			desc->pending = 0;
			desc->chip->unmask(irq);
		}

		__do_irq(irq, action, regs);
	} while (desc->pending && !desc->disable_depth);

	desc->status &= ~IRQ_INPROGRESS;
	desc->running = 0;

	/*
	 * If we were disabled or freed, shut down the handler.
	 */
	if (likely(desc->action && !check_irq_lock(desc, irq, regs)))
		return;

 running:
	/*
	 * We got another IRQ while this one was masked or
	 * currently running.  Delay it.
	 */
	desc->pending = 1;
	desc->status |= IRQ_PENDING;
	desc->chip->mask(irq);
	desc->chip->ack(irq);
}

/*
 * Level-based IRQ handler.  Nice and simple.
 */
void
do_level_IRQ(unsigned int irq, struct irqdesc *desc, struct pt_regs *regs)
{
	struct irqaction *action;
	const unsigned int cpu = smp_processor_id();

	desc->triggered = 1;

	/*
	 * Acknowledge, clear _AND_ disable the interrupt.
	 */
	desc->chip->ack(irq);

#ifdef CONFIG_PREEMPT_HARDIRQS
	if (unlikely(irq == ARCH_TIMER_IRQ))
		direct_timer_interrupt(regs);
#endif

	if (likely(!desc->disable_depth)) {
		kstat_cpu(cpu).irqs[irq]++;

		smp_set_running(desc);

		/*
		 * Return with this interrupt masked if no action
		 */
		action = desc->action;
		if (action) {
			int ret = 0;
#ifdef CONFIG_PREEMPT_HARDIRQS
			desc->status |= IRQ_INPROGRESS;
			ret = IRQ_HANDLED;

			if (redirect_hardirq(desc))
				return;
#endif
			ret = __do_irq(irq, desc->action, regs);

			if (ret != IRQ_HANDLED)
				report_bad_irq(irq, regs, desc, ret);

			if (likely(!desc->disable_depth &&
				   !check_irq_lock(desc, irq, regs)))
				desc->chip->unmask(irq);
		}
		desc->status &= ~IRQ_INPROGRESS;

		smp_clear_running(desc);
	}
}

static void do_pending_irqs(struct pt_regs *regs)
{
	struct list_head head, *l, *n;

	do {
		struct irqdesc *desc;

		/*
		 * First, take the pending interrupts off the list.
		 * The act of calling the handlers may add some IRQs
		 * back onto the list.
		 */
		head = irq_pending;
		INIT_LIST_HEAD(&irq_pending);
		head.next->prev = &head;
		head.prev->next = &head;

		/*
		 * Now run each entry.  We must delete it from our
		 * list before calling the handler.
		 */
		list_for_each_safe(l, n, &head) {
			desc = list_entry(l, struct irqdesc, pend);
			list_del_init(&desc->pend);
			desc->handle(desc - irq_desc, desc, regs);
		}

		/*
		 * The list must be empty.
		 */
		BUG_ON(!list_empty(&head));
	} while (!list_empty(&irq_pending));
}

/*
 * do_IRQ handles all hardware IRQ's.  Decoded IRQs should not
 * come via this function.  Instead, they should provide their
 * own 'handler'
 */
asmlinkage notrace void asm_do_IRQ(unsigned int irq, struct pt_regs *regs)
{
	struct irqdesc *desc = irq_desc + irq;

	ltt_ev_irq_entry(irq, !(user_mode(regs)));

	/*
	 * Some hardware gives randomly wrong interrupts.  Rather
	 * than crashing, do something sensible.
	 */
	if (irq >= NR_IRQS)
		desc = &bad_irq_desc;

	interrupt_overhead_start();
	irq_enter();
	vst_wakeup(regs, 0);
	spin_lock(&irq_controller_lock);
	desc->handle(irq, desc, regs);

	/*
	 * Now re-run any pending interrupts.
	 */
	if (!list_empty(&irq_pending))
		do_pending_irqs(regs);

	irq_finish(irq);

	spin_unlock(&irq_controller_lock);
	irq_exit();
	latency_check();

	ltt_ev_irq_exit();
}

void __set_irq_handler(unsigned int irq, irq_handler_t handle, int is_chained)
{
	struct irqdesc *desc;
	unsigned long flags;

	if (irq >= NR_IRQS) {
		printk(KERN_ERR "Trying to install handler for IRQ%d\n", irq);
		return;
	}

	if (handle == NULL)
		handle = do_bad_IRQ;

	desc = irq_desc + irq;

	if (is_chained && desc->chip == &bad_chip)
		printk(KERN_WARNING "Trying to install chained handler for IRQ%d\n", irq);

	spin_lock_irqsave(&irq_controller_lock, flags);
	if (handle == do_bad_IRQ) {
		desc->chip->mask(irq);
		desc->chip->ack(irq);
		desc->disable_depth = 1;
	}
	desc->handle = handle;
	if (handle != do_bad_IRQ && is_chained) {
		desc->valid = 0;
		desc->probe_ok = 0;
		desc->disable_depth = 0;
		desc->chip->unmask(irq);
	}
	spin_unlock_irqrestore(&irq_controller_lock, flags);
}

void set_irq_chip(unsigned int irq, struct irqchip *chip)
{
	struct irqdesc *desc;
	unsigned long flags;

	if (irq >= NR_IRQS) {
		printk(KERN_ERR "Trying to install chip for IRQ%d\n", irq);
		return;
	}

	if (chip == NULL)
		chip = &bad_chip;

	desc = irq_desc + irq;
	spin_lock_irqsave(&irq_controller_lock, flags);
	desc->chip = chip;
	spin_unlock_irqrestore(&irq_controller_lock, flags);
}

int set_irq_type(unsigned int irq, unsigned int type)
{
	struct irqdesc *desc;
	unsigned long flags;
	int ret = -ENXIO;

	if (irq >= NR_IRQS) {
		printk(KERN_ERR "Trying to set irq type for IRQ%d\n", irq);
		return -ENODEV;
	}

	desc = irq_desc + irq;
	if (desc->chip->type) {
		spin_lock_irqsave(&irq_controller_lock, flags);
		ret = desc->chip->type(irq, type);
		spin_unlock_irqrestore(&irq_controller_lock, flags);
	}

	return ret;
}
EXPORT_SYMBOL(set_irq_type);

void set_irq_flags(unsigned int irq, unsigned int iflags)
{
	struct irqdesc *desc;
	unsigned long flags;

	if (irq >= NR_IRQS) {
		printk(KERN_ERR "Trying to set irq flags for IRQ%d\n", irq);
		return;
	}

	desc = irq_desc + irq;
	spin_lock_irqsave(&irq_controller_lock, flags);
	desc->valid = (iflags & IRQF_VALID) != 0;
	desc->probe_ok = (iflags & IRQF_PROBE) != 0;
	desc->noautoenable = (iflags & IRQF_NOAUTOEN) != 0;
	spin_unlock_irqrestore(&irq_controller_lock, flags);
}

int setup_irq(unsigned int irq, struct irqaction *new)
{
	int shared = 0;
	struct irqaction *old, **p;
	unsigned long flags;
	struct irqdesc *desc;

	/*
	 * Some drivers like serial.c use request_irq() heavily,
	 * so we have to be careful not to interfere with a
	 * running system.
	 */
	if (new->flags & SA_SAMPLE_RANDOM) {
		/*
		 * This function might sleep, we want to call it first,
		 * outside of the atomic block.
		 * Yes, this might clear the entropy pool if the wrong
		 * driver is attempted to be loaded, without actually
		 * installing a new handler, but is this really a problem,
		 * only the sysadmin is able to do this.
		 */
	        rand_initialize_irq(irq);
	}

	/*
	 * The following block of code has to be executed atomically
	 */
	desc = irq_desc + irq;
#ifdef CONFIG_PREEMPT_HARDIRQS
	if (!(new->flags & SA_NODELAY)) {
		if (start_irq_thread(irq, desc))
			return -ENOMEM;
	}
#endif
	spin_lock_irqsave(&irq_controller_lock, flags);
	p = &desc->action;

	if ((old = *p) != NULL) {
		/* Can't share interrupts unless both agree to */
		if (!(old->flags & new->flags & SA_SHIRQ)) {
			spin_unlock_irqrestore(&irq_controller_lock, flags);
			return -EBUSY;
		}

		/* add new interrupt at end of irq queue */
		do {
			p = &old->next;
			old = *p;
		} while (old);
		shared = 1;
	}

	*p = new;

	if (!shared) {
 		desc->probing = 0;
		desc->running = 0;
		desc->pending = 0;
		desc->disable_depth = 1;
		if (!desc->noautoenable) {
			desc->disable_depth = 0;
			desc->chip->unmask(irq);
		}
	}


#ifdef CONFIG_PREEMPT_HARDIRQS
	recalculate_desc_flags(desc);
#endif

	spin_unlock_irqrestore(&irq_controller_lock, flags);
	return 0;
}

/**
 *	request_irq - allocate an interrupt line
 *	@irq: Interrupt line to allocate
 *	@handler: Function to be called when the IRQ occurs
 *	@irqflags: Interrupt type flags
 *	@devname: An ascii name for the claiming device
 *	@dev_id: A cookie passed back to the handler function
 *
 *	This call allocates interrupt resources and enables the
 *	interrupt line and IRQ handling. From the point this
 *	call is made your handler function may be invoked. Since
 *	your handler function must clear any interrupt the board
 *	raises, you must take care both to initialise your hardware
 *	and to set up the interrupt handler in the right order.
 *
 *	Dev_id must be globally unique. Normally the address of the
 *	device data structure is used as the cookie. Since the handler
 *	receives this value it makes sense to use it.
 *
 *	If your interrupt is shared you must pass a non NULL dev_id
 *	as this is required when freeing the interrupt.
 *
 *	Flags:
 *
 *	SA_SHIRQ		Interrupt is shared
 *
 *	SA_INTERRUPT		Disable local interrupts while processing
 *
 *	SA_SAMPLE_RANDOM	The interrupt can be used for entropy
 *
 */
int request_irq(unsigned int irq, irqreturn_t (*handler)(int, void *, struct pt_regs *),
		 unsigned long irq_flags, const char * devname, void *dev_id)
{
	unsigned long retval;
	struct irqaction *action;

	if (irq >= NR_IRQS || !irq_desc[irq].valid || !handler ||
	    (irq_flags & SA_SHIRQ && !dev_id))
		return -EINVAL;

	action = (struct irqaction *)kmalloc(sizeof(struct irqaction), GFP_KERNEL);
	if (!action)
		return -ENOMEM;

	action->handler = handler;
	action->flags = irq_flags;
	cpus_clear(action->mask);
	action->name = devname;
	action->next = NULL;
	action->dev_id = dev_id;

	retval = setup_irq(irq, action);

	if (retval)
		kfree(action);
	return retval;
}

EXPORT_SYMBOL(request_irq);

/**
 *	free_irq - free an interrupt
 *	@irq: Interrupt line to free
 *	@dev_id: Device identity to free
 *
 *	Remove an interrupt handler. The handler is removed and if the
 *	interrupt line is no longer in use by any driver it is disabled.
 *	On a shared IRQ the caller must ensure the interrupt is disabled
 *	on the card it drives before calling this function.
 *
 *	This function must not be called from interrupt context.
 */
void free_irq(unsigned int irq, void *dev_id)
{
	struct irqaction * action, **p;
	unsigned long flags;

	if (irq >= NR_IRQS || !irq_desc[irq].valid) {
		printk(KERN_ERR "Trying to free IRQ%d\n",irq);
		dump_stack();
		return;
	}

	spin_lock_irqsave(&irq_controller_lock, flags);
	for (p = &irq_desc[irq].action; (action = *p) != NULL; p = &action->next) {
		if (action->dev_id != dev_id)
			continue;

	    	/* Found it - now free it */
		*p = action->next;
		break;
	}
	spin_unlock_irqrestore(&irq_controller_lock, flags);

	if (!action) {
		printk(KERN_ERR "Trying to free free IRQ%d\n",irq);
		dump_stack();
	} else {
		synchronize_irq(irq);
		kfree(action);
	}
}

EXPORT_SYMBOL(free_irq);

static DECLARE_MUTEX(probe_sem);

/* Start the interrupt probing.  Unlike other architectures,
 * we don't return a mask of interrupts from probe_irq_on,
 * but return the number of interrupts enabled for the probe.
 * The interrupts which have been enabled for probing is
 * instead recorded in the irq_desc structure.
 */
unsigned long probe_irq_on(void)
{
	unsigned int i, irqs = 0;
	unsigned long delay;

	down(&probe_sem);

	/*
	 * first snaffle up any unassigned but
	 * probe-able interrupts
	 */
	spin_lock_irq(&irq_controller_lock);
	for (i = 0; i < NR_IRQS; i++) {
		if (!irq_desc[i].probe_ok || irq_desc[i].action)
			continue;

		irq_desc[i].probing = 1;
		irq_desc[i].triggered = 0;
		if (irq_desc[i].chip->type)
			irq_desc[i].chip->type(i, IRQT_PROBE);
		irq_desc[i].chip->unmask(i);
		irqs += 1;
	}
	spin_unlock_irq(&irq_controller_lock);

	/*
	 * wait for spurious interrupts to mask themselves out again
	 */
	for (delay = jiffies + HZ/10; time_before(jiffies, delay); )
		/* min 100ms delay */;

	/*
	 * now filter out any obviously spurious interrupts
	 */
	spin_lock_irq(&irq_controller_lock);
	for (i = 0; i < NR_IRQS; i++) {
		if (irq_desc[i].probing && irq_desc[i].triggered) {
			irq_desc[i].probing = 0;
			irqs -= 1;
		}
	}
	spin_unlock_irq(&irq_controller_lock);

	return irqs;
}

EXPORT_SYMBOL(probe_irq_on);

unsigned int probe_irq_mask(unsigned long irqs)
{
	unsigned int mask = 0, i;

	spin_lock_irq(&irq_controller_lock);
	for (i = 0; i < 16 && i < NR_IRQS; i++)
		if (irq_desc[i].probing && irq_desc[i].triggered)
			mask |= 1 << i;
	spin_unlock_irq(&irq_controller_lock);

	up(&probe_sem);

	return mask;
}
EXPORT_SYMBOL(probe_irq_mask);

/*
 * Possible return values:
 *  >= 0 - interrupt number
 *    -1 - no interrupt/many interrupts
 */
int probe_irq_off(unsigned long irqs)
{
	unsigned int i;
	int irq_found = NO_IRQ;

	/*
	 * look at the interrupts, and find exactly one
	 * that we were probing has been triggered
	 */
	spin_lock_irq(&irq_controller_lock);
	for (i = 0; i < NR_IRQS; i++) {
		if (irq_desc[i].probing &&
		    irq_desc[i].triggered) {
			if (irq_found != NO_IRQ) {
				irq_found = NO_IRQ;
				goto out;
			}
			irq_found = i;
		}
	}

	if (irq_found == -1)
		irq_found = NO_IRQ;
out:
	spin_unlock_irq(&irq_controller_lock);

	up(&probe_sem);

	return irq_found;
}

EXPORT_SYMBOL(probe_irq_off);

#ifdef CONFIG_SMP
static void route_irq(struct irqdesc *desc, unsigned int irq, unsigned int cpu)
{
	pr_debug("IRQ%u: moving from cpu%u to cpu%u\n", irq, desc->cpu, cpu);

	spin_lock_irq(&irq_controller_lock);
	desc->cpu = cpu;
	desc->chip->set_cpu(desc, irq, cpu);
	spin_unlock_irq(&irq_controller_lock);
}

#ifdef CONFIG_PROC_FS
static int
irq_affinity_read_proc(char *page, char **start, off_t off, int count,
		       int *eof, void *data)
{
	struct irqdesc *desc = irq_desc + ((int)data);
	int len = cpumask_scnprintf(page, count, desc->affinity);

	if (count - len < 2)
		return -EINVAL;
	page[len++] = '\n';
	page[len] = '\0';

	return len;
}

static int
irq_affinity_write_proc(struct file *file, const char __user *buffer,
			unsigned long count, void *data)
{
	unsigned int irq = (unsigned int)data;
	struct irqdesc *desc = irq_desc + irq;
	cpumask_t affinity, tmp;
	int ret = -EIO;

	if (!desc->chip->set_cpu)
		goto out;

	ret = cpumask_parse(buffer, count, affinity);
	if (ret)
		goto out;

	cpus_and(tmp, affinity, cpu_online_map);
	if (cpus_empty(tmp)) {
		ret = -EINVAL;
		goto out;
	}

	desc->affinity = affinity;
	route_irq(desc, irq, first_cpu(tmp));
	ret = count;

 out:
	return ret;
}
#endif
#endif

void __init init_irq_proc(void)
{
#if defined(CONFIG_SMP) && defined(CONFIG_PROC_FS)
	struct proc_dir_entry *dir;
	int irq;

	dir = proc_mkdir("irq", 0);
	if (!dir)
		return;

	for (irq = 0; irq < NR_IRQS; irq++) {
		struct proc_dir_entry *entry;
		struct irqdesc *desc;
		char name[16];

		desc = irq_desc + irq;
		memset(name, 0, sizeof(name));
		snprintf(name, sizeof(name) - 1, "%u", irq);

		desc->procdir = proc_mkdir(name, dir);
		if (!desc->procdir)
			continue;

		entry = create_proc_entry("smp_affinity", 0600, desc->procdir);
		if (entry) {
			entry->nlink = 1;
			entry->data = (void *)irq;
			entry->read_proc = irq_affinity_read_proc;
			entry->write_proc = irq_affinity_write_proc;
		}
	}
#endif
}

void __init init_hardirqs(void)
{
	int i;
	ok_to_create_irq_threads = 1;                                                                                               
	for (i = 0; i < NR_IRQS; i++) {
		struct irqdesc *desc = irq_desc + i;
		
		if (desc->action && !(desc->status & IRQ_NODELAY))
			start_irq_thread(i, desc);
	}
}

void __init init_IRQ(void)
{
	struct irqdesc *desc;
	extern void init_dma(void);
	int irq;
#ifdef CONFIG_MOT_FEAT_FIQ_IN_C
	extern void mxc_fiq_init(void);
#endif

#ifdef CONFIG_SMP
	bad_irq_desc.affinity = CPU_MASK_ALL;
	bad_irq_desc.cpu = smp_processor_id();
#endif

#ifdef CONFIG_MOT_FEAT_FIQ_IN_C
	mxc_fiq_init();
#endif

	for (irq = 0, desc = irq_desc; irq < NR_IRQS; irq++, desc++) {
		*desc = bad_irq_desc;
		INIT_LIST_HEAD(&desc->pend);
#ifdef CONFIG_PREEMPT_HARDIRQS
		spin_lock_init(&desc->lock);
		desc->thread = NULL;
#endif
	}

	init_arch_irq();
	init_dma();
}

static int __init noirqdebug_setup(char *str)
{
	noirqdebug = 1;
	return 1;
}

__setup("noirqdebug", noirqdebug_setup);
