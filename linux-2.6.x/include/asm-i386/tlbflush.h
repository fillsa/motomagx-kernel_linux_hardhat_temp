#ifndef _I386_TLBFLUSH_H
#define _I386_TLBFLUSH_H

#include <linux/config.h>
#include <linux/mm.h>
#include <asm/processor.h>

/*
 * TLB-flush needs to be nonpreemptible on PREEMPT_RT due to the
 * following complex race scenario:
 *
 * if the current task is lazy-TLB and does a TLB flush and
 * gets preempted after the movl %%r3, %0 but before the
 * movl %0, %%cr3 then its ->active_mm might change and it will
 * install the wrong cr3 when it switches back. This is not a
 * problem for the lazy-TLB task itself, but if the next task it
 * switches to has an ->mm that is also the lazy-TLB task's
 * new ->active_mm, then the scheduler will assume that cr3 is
 * the new one, while we overwrote it with the old one. The result
 * is the wrong cr3 in the new (non-lazy-TLB) task, which typically
 * causes an infinite pagefault upon the next userspace access.
 */
#define __flush_tlb()							\
	do {								\
		unsigned int tmpreg;					\
									\
		preempt_disable();					\
		__asm__ __volatile__(					\
			"movl %%cr3, %0;              \n"		\
			"movl %0, %%cr3;  # flush TLB \n"		\
			: "=r" (tmpreg)					\
			:: "memory");					\
		preempt_enable();					\
	} while (0)

/*
 * Global pages have to be flushed a bit differently. Not a real
 * performance problem because this does not happen often.
 */
#define __flush_tlb_global()						\
	do {								\
		unsigned int tmpreg, cr4, cr4_orig;			\
									\
		preempt_disable();					\
		__asm__ __volatile__(					\
			"movl %%cr4, %2;  # turn off PGE     \n"	\
			"movl %2, %1;                        \n"	\
			"andl %3, %1;                        \n"	\
			"movl %1, %%cr4;                     \n"	\
			"movl %%cr3, %0;                     \n"	\
			"movl %0, %%cr3;  # flush TLB        \n"	\
			"movl %2, %%cr4;  # turn PGE back on \n"	\
			: "=&r" (tmpreg), "=&r" (cr4), "=&r" (cr4_orig)	\
			: "i" (~X86_CR4_PGE)				\
			: "memory");					\
		preempt_enable();					\
	} while (0)

extern unsigned long pgkern_mask;

# define __flush_tlb_all()						\
	do {								\
		if (cpu_has_pge)					\
			__flush_tlb_global();				\
		else							\
			__flush_tlb();					\
	} while (0)

#define cpu_has_invlpg	(boot_cpu_data.x86 > 3)

#define __flush_tlb_single(addr) \
	__asm__ __volatile__("invlpg %0": :"m" (*(char *) addr))

#ifdef CONFIG_X86_INVLPG
# define __flush_tlb_one(addr) __flush_tlb_single(addr)
#else
# define __flush_tlb_one(addr)						\
	do {								\
		if (cpu_has_invlpg)					\
			__flush_tlb_single(addr);			\
		else							\
			__flush_tlb();					\
	} while (0)
#endif

/*
 * TLB flushing:
 *
 *  - flush_tlb() flushes the current mm struct TLBs
 *  - flush_tlb_all() flushes all processes TLBs
 *  - flush_tlb_mm(mm) flushes the specified mm context TLB's
 *  - flush_tlb_page(vma, vmaddr) flushes one page
 *  - flush_tlb_range(vma, start, end) flushes a range of pages
 *  - flush_tlb_kernel_range(start, end) flushes a range of kernel pages
 *  - flush_tlb_pgtables(mm, start, end) flushes a range of page tables
 *
 * ..but the i386 has somewhat limited tlb flushing capabilities,
 * and page-granular flushes are available only on i486 and up.
 */

#ifndef CONFIG_SMP

#define flush_tlb() __flush_tlb()
#define flush_tlb_all() __flush_tlb_all()
#define local_flush_tlb() __flush_tlb()

static inline void flush_tlb_mm(struct mm_struct *mm)
{
	/*
	 * This is safe on PREEMPT_RT because if we preempt
	 * right after the check but before the __flush_tlb(),
	 * and if ->active_mm changes, then we might miss a
	 * TLB flush, but that TLB flush happened already when
	 * ->active_mm was changed:
	 */
	if (mm == current->active_mm)
		__flush_tlb();
}

static inline void flush_tlb_page(struct vm_area_struct *vma,
	unsigned long addr)
{
	if (vma->vm_mm == current->active_mm)
		__flush_tlb_one(addr);
}

static inline void flush_tlb_range(struct vm_area_struct *vma,
	unsigned long start, unsigned long end)
{
	if (vma->vm_mm == current->active_mm)
		__flush_tlb();
}

#else

#include <asm/smp.h>

#define local_flush_tlb() \
	__flush_tlb()

extern void flush_tlb_all(void);
extern void flush_tlb_current_task(void);
extern void flush_tlb_mm(struct mm_struct *);
extern void flush_tlb_page(struct vm_area_struct *, unsigned long);

#define flush_tlb()	flush_tlb_current_task()

static inline void flush_tlb_range(struct vm_area_struct * vma, unsigned long start, unsigned long end)
{
	flush_tlb_mm(vma->vm_mm);
}

#define TLBSTATE_OK	1
#define TLBSTATE_LAZY	2

struct tlb_state
{
	struct mm_struct *active_mm;
	int state;
	char __cacheline_padding[L1_CACHE_BYTES-8];
};
DECLARE_PER_CPU(struct tlb_state, cpu_tlbstate);


#endif

#define flush_tlb_kernel_range(start, end) flush_tlb_all()

static inline void flush_tlb_pgtables(struct mm_struct *mm,
				      unsigned long start, unsigned long end)
{
	/* i386 does not keep any page table caches in TLB */
}

#endif /* _I386_TLBFLUSH_H */
