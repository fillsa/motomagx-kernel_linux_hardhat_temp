/*
 *  linux/arch/arm/mm/proc-v6.S
 *
 *  Copyright (C) 2001 Deep Blue Solutions Ltd.
 *  Copyright (C) 2006-2007 Motorola Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  This is the "shell" of the ARMv6 processor support.
 *
 *
 * Date        Author            Comment
 * ==========  ================  ========================
 * 09/26/2006  Motorola          Initial creation
 * 10/25/2006   Motorola        Modify the assembly check to include r1p1 as well.
 * 11/15/2006  Motorola          Added workaround errata 380532
 * 12/21/2006   Motorola        Modify icache flushes to flush the icache using
 *                                the recommended ARM errata workaround
 * 02/21/2007  Motorola          Added fix for errata 411920
 * 03/14/2007   Motorola        Improve L2 Cache performance by changing the CPU wait time.
 * 03/28/2007  Motorola          Added cache optimization
 * 07/14/2008  Motorola          Invalidated main locked TLB.
 */
#include <linux/linkage.h>
#include <asm/assembler.h>
#include <asm/constants.h>
#include <asm/procinfo.h>
#include <asm/pgtable.h>

#include "proc-macros.S"

#define D_CACHE_LINE_SIZE	32

#ifdef CONFIG_CPU_CACHE_L210
#include <asm/hardware.h>

#ifdef CONFIG_MOT_FEAT_CACHE_OPTIMIZATION
#include <asm/arch/mxc_pm.h>
#endif /* CONFIG_MOT_FEAT_CACHE_OPTIMIZATION */

	.macro ENABLE_L2_CACHE
	mcr	p15, 0, r0, c7, c10, 4		@ drain write buffer

	mov	r1, #L2CC_BASE_ADDR
	ldr	r2, [r1, #0x100]		@ ARM L210 Control
	bic	r2, r2, #0x1
	str	r2, [r1, #0x100]		@ disable L2

	mcr	p15, 0, r0, c7, c10, 4		@ drain write buffer

	/*
	 * Configure L2 Cache:
	 * - 128k size(16k way)
	 * - 8-way associativity
	 * - 0 ws TAG/VALID/DIRTY
	 * - 7 ws DATA R/W
	 */
	ldr	r2, [r1, #0x104]		@ ARM L210 Auxiliary Control
	and	r2, r2, #0xFE000000
	orr	r2, r2, #0x00030000

#ifdef CONFIG_MOT_FEAT_CACHE_OPTIMIZATION
#if CORE_MAX_FREQ >= CORE_FREQ_532 // #if CORE_MAX_FREQ == CORE_FREQ_532
	orr	r2, r2, #0x00000024
#elif CORE_MAX_FREQ == CORE_FREQ_399
	orr	r2, r2, #0x0000001B
#elif CORE_MAX_FREQ == CORE_FREQ_266
	orr	r2, r2, #0x00000012
#elif CORE_MAX_FREQ == CORE_FREQ_133
	orr	r2, r2, #0x00000009
#else
#error "Unsupported maximum core frequency!"
#endif /* CORE_MAX_FREQ */
#endif /* CONFIG_MOT_FEAT_CACHE_OPTIMIZATION

#ifndef CONFIG_MOT_FEAT_CACHE_OPTIMIZATION
	orr	r2, r2, #0x00000024
#endif /* CONFIG_MOT_FEAT_CACHE_OPTIMIZATION */

	str	r2, [r1, #0x104]

	mcr	p15, 0, r0, c7, c10, 4		@ drain write buffer

	/* Invalidate By Way */
	ldr	r2, =0xff
	str	r2, [r1, #0x77C]		@ ARM L210 Invalidate By Way

L2_inv_loop:
	ldr	r2, [r1, #0x77C]
	cmp	r2, #0
	bne	L2_inv_loop

	mcr	p15, 0, r0, c7, c10, 4		@ drain write buffer
	/* Force L2 write through */
#ifndef CONFIG_MOT_FEAT_CACHE_OPTIMIZATION
	ldr	r2, =0x2
	str	r2, [r1, #0xF40]
#endif /* CONFIG_MOT_FEAT_CACHE_OPTIMIZATION */
	/* Enable L2 */
	ldr	r2, [r1, #0x100]		@ ARM L210 Control
	orr	r2, r2, #0x1
	str	r2, [r1, #0x100]
	.endm	/* ENABLE_L2_CACHE */

#else
#define ENABLE_L2_CACHE
#endif /* CONFIG_CPU_CACHE_L210 */

	.macro	cpsie, flags
	.ifc \flags, f
	.long	0xf1080040
	.exitm
	.endif
	.ifc \flags, i
	.long	0xf1080080
	.exitm
	.endif
	.ifc \flags, if
	.long	0xf10800c0
	.exitm
	.endif
	.err
	.endm

#ifndef CONFIG_MOT_WFN477
	.macro	cpsid, flags
	.ifc \flags, f
	.long	0xf10c0040
	.exitm
	.endif
	.ifc \flags, i
	.long	0xf10c0080
	.exitm
	.endif
	.ifc \flags, if
	.long	0xf10c00c0
	.exitm
	.endif
	.err
	.endm
#endif

ENTRY(cpu_v6_proc_init)
	mov	pc, lr

ENTRY(cpu_v6_proc_fin)
	stmfd	sp!, {lr}
	cpsid	if				@ disable interrupts
	bl	v6_flush_kern_cache_all
	mrc	p15, 0, r0, c1, c0, 0		@ ctrl register
	bic	r0, r0, #0x1000			@ ...i............
	bic	r0, r0, #0x0006			@ .............ca.
	mcr	p15, 0, r0, c1, c0, 0		@ disable caches
	ldmfd	sp!, {pc}

/*
 *	cpu_v6_reset(loc)
 *
 *	Perform a soft reset of the system.  Put the CPU into the
 *	same state as it would be if it had been reset, and branch
 *	to what would be the reset vector.
 *
 *	- loc   - location to jump to for soft reset
 *
 *	It is assumed that:
 */
	.align	5
ENTRY(cpu_v6_reset)
	mov	pc, r0

/*
 *	cpu_v6_do_idle()
 *
 *	Idle the processor (eg, wait for interrupt).
 *
 *	IRQs are already disabled.
 */
ENTRY(cpu_v6_do_idle)
	mcr	p15, 0, r1, c7, c0, 4		@ wait for interrupt
	mov	pc, lr

ENTRY(cpu_v6_dcache_clean_area)
#ifndef TLB_CAN_READ_FROM_L1_CACHE
1:	mcr	p15, 0, r0, c7, c10, 1		@ clean D entry
	add	r0, r0, #D_CACHE_LINE_SIZE
	subs	r1, r1, #D_CACHE_LINE_SIZE
	bhi	1b
#endif
	mov	pc, lr

/*
 *	cpu_arm926_switch_mm(pgd_phys, tsk)
 *
 *	Set the translation table base pointer to be pgd_phys
 *
 *	- pgd_phys - physical address of new TTB
 *
 *	It is assumed that:
 *	- we are not using split page tables
 */
ENTRY(cpu_v6_switch_mm)
	mov	r2, #0
	ldr	r1, [r1, #MM_CONTEXT_ID]	@ get mm->context.id
#ifdef CONFIG_CPU_CACHE_L210
	orr	r0, r0, #0x18			@ set outer cacheable write back pgtable
#endif
	mcr	p15, 0, r2, c7, c5, 6		@ flush BTAC/BTB	
	mcr	p15, 0, r2, c7, c10, 4		@ drain write buffer
	mcr	p15, 0, r0, c2, c0, 0		@ set TTB 0
	mcr	p15, 0, r1, c13, c0, 1		@ set context ID
	mov	pc, lr

/*
 *	cpu_v6_set_pte(ptep, pte)
 *
 *	Set a level 2 translation table entry.
 *
 *	- ptep  - pointer to level 2 translation table entry
 *		  (hardware version is stored at -1024 bytes)
 *	- pte   - PTE value to store
 *
 *	Permissions:
 *	  YUWD  APX AP1 AP0	SVC	User
 *	  0xxx   0   0   0	no acc	no acc
 *	  100x   1   0   1	r/o	no acc
 *	  10x0   1   0   1	r/o	no acc
 *	  1011   0   0   1	r/w	no acc
 *	  110x   0   1   0	r/w	r/o
 *	  11x0   0   1   0	r/w	r/o
 *	  1111   0   1   1	r/w	r/w
 */
ENTRY(cpu_v6_set_pte)
	str	r1, [r0], #-2048		@ linux version

	bic	r2, r1, #0x000007f0
	bic	r2, r2, #0x00000003
	orr	r2, r2, #PTE_EXT_AP0 | 2

	tst	r1, #L_PTE_WRITE
	tstne	r1, #L_PTE_DIRTY
	orreq	r2, r2, #PTE_EXT_APX

	tst	r1, #L_PTE_USER
	orrne	r2, r2, #PTE_EXT_AP1
	tstne	r2, #PTE_EXT_APX
	bicne	r2, r2, #PTE_EXT_APX | PTE_EXT_AP0

	tst	r1, #L_PTE_YOUNG
	biceq	r2, r2, #PTE_EXT_APX | PTE_EXT_AP_MASK

@	tst	r1, #L_PTE_EXEC
@	orreq	r2, r2, #PTE_EXT_XN

	tst	r1, #L_PTE_PRESENT
	moveq	r2, #0

	str	r2, [r0]
	mcr	p15, 0, r0, c7, c10, 1 @ flush_pte
	mov	pc, lr




cpu_v6_name:
	.asciz	"Some Random V6 Processor"
	.align

	.section ".text.init", #alloc, #execinstr

/*
 *	__v6_setup
 *
 *	Initialise TLB, Caches, and MMU state ready to switch the MMU
 *	on.  Return in r0 the new CP15 C1 control register setting.
 *
 *	We automatically detect if we have a Harvard cache, and use the
 *	Harvard cache control instructions insead of the unified cache
 *	control instructions.
 *
 *	This should be able to cover all ARMv6 cores.
 *
 *	It is assumed that:
 *	- cache type register is implemented
 */
__v6_setup:
	mov	r0, #0
	mcr	p15, 0, r0, c7, c14, 0		@ clean+invalidate D cache

	/* Fix for ARM errata 411920 */ 
#ifndef CONFIG_MOT_WFN477
	mcr	p15, 0, r0, c7, c5, 0		@ invalidate I cache
#else
	mrs	r1, cpsr
	cpsid	ifa						@ disable interrupts
	mcr	p15, 0, r0, c7, c12, 5		@ Stop Prefetch Range
	mcr	p15, 0, r0, c7, c5, 0		@ Invalidate Entire Instruction Cache
	mcr	p15, 0, r0, c7, c5, 0		@ Invalidate Entire Instruction Cache
	mcr	p15, 0, r0, c7, c5, 0		@ Invalidate Entire Instruction Cache
	mcr	p15, 0, r0, c7, c5, 0		@ Invalidate Entire Instruction Cache
	msr	cpsr_c, r1					@ reenable interrupts
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
#endif

	mcr	p15, 0, r0, c7, c15, 0		@ clean+invalidate cache
	mcr	p15, 0, r0, c7, c10, 4		@ drain write buffer
	mcr	p15, 0, r0, c8, c7, 0		@ invalidate I + D TLBs
#ifdef CONFIG_MOT_FEAT_FLUSH_LOCKED_TLB
	/* Invalidate all locked entries in Main TLB ___START___*/
	mrc     p15, 7, r1, c15, c1, 0          @ Read control register 
	orr     r3, r1, #0xf0    		@ Disable Main TLB Load and Match (0xf0)
	mcr     p15, 7, r3, c15, c1, 0          @ Write control register
	mov     r2, #0x80000000 		@ Start 0x80000000
Inval_locked_TLB:
	mcr     p15, 5, r2, c15, c4, 2 		@ Write to Read Main TLB Entry
	mrc     p15, 5, r3, c15, c5, 2 		@ Read Main TLB VA
	cmp     r3, #0                          @ VA=0 means invalid TLB  
	beq     Inval_next_TLB                  @ Jump to next entry
	mcr     p15, 0, r3, c8, c7, 1		@ Invalidate the address in r3
Inval_next_TLB:
	add     r2, r2, #1
	cmp     r2, #0x80000007 		@ End 0x80000007(8 locked entries)
	bls     Inval_locked_TLB                @ less or equal 0x80000007
	mcr     p15, 7, r1, c15, c1, 0 		@ Restore Main TLB load and Match
	/* Invalidate all locked entries in Main TLB ___END___*/
#endif /*CONFIG_MOT_FEAT_FLUSH_LOCKED_TLB*/
	mcr	p15, 0, r0, c2, c0, 2		@ TTB control register
#ifdef CONFIG_CPU_CACHE_L210
	orr	r4, r4, #0x18			@ set outer cacheable write back pgtable
#endif
	mcr	p15, 0, r4, c2, c0, 1		@ load TTB1
	ENABLE_L2_CACHE
#ifdef CONFIG_VFP
	mrc	p15, 0, r0, c1, c0, 2
	orr	r0, r0, #(0xf << 20)
	mcr	p15, 0, r0, c1, c0, 2		@ Enable full access to VFP
#endif
	/* The conditional code below (setting the undocumented bit 31 in
	 * the auxiliary control register and the FI bit in the control
	 * register) disables hit-under-miss without putting the processor
	 * into full low interrupt latency mode to workaround possible
	 * cache data corruption in ARM1136 r0pX parts when hit-under-miss
	 * is enabled.
	 */
	ldr	r5, =0x4107b360			@ id for ARM1136 r0pX
	mrc	p15, 0, r0, c0, c0, 0		@ get processor id
	bic	r0, r0, #0xf			@ mask out part bits [3:0]

#ifdef CONFIG_MOT_WFN462
        @ Workaround for errata 380532 
	bic	r0, r0, #0x00100000		@ mask out revision 1 bits too
#endif

	teq	r0, r5				@ check for the faulty core
	mrceq	p15, 0, r0, c1, c0, 1		@ load aux control reg
	orreq	r0, r0, #0x80000000		@ set bit 31
	mcreq	p15, 0, r0, c1, c0, 1		@ write aux control reg
	mrc	p15, 0, r0, c1, c0, 0		@ read control register
	ldr	r5, v6_cr1_clear		@ get mask for bits to clear
	bic	r0, r0, r5			@ clear bits them
	ldr	r5, v6_cr1_set			@ get mask for bits to set
	orr	r0, r0, r5			@ set them
	orreq	r0, r0, #0x00200000		@ set the FI bit
	mov	pc, lr				@ return to head.S:__ret

	/*
	 *         V X F   I D LR
	 * .... ...E PUI. .T.T 4RVI ZFRS BLDP WCAM
	 * rrrr rrrx xxx0 0101 xxxx xxxx x111 xxxx < forced
	 *         0 110       0011 1.00 .111 1101 < we want
	 */
	.type	v6_cr1_clear, #object
	.type	v6_cr1_set, #object
v6_cr1_clear:
	.word	0x01e0fb7f
v6_cr1_set:
	.word	0x00c0387d

	.type	v6_processor_functions, #object
ENTRY(v6_processor_functions)
	.word	v6_early_abort
	.word	cpu_v6_proc_init
	.word	cpu_v6_proc_fin
	.word	cpu_v6_reset
	.word	cpu_v6_do_idle
	.word	cpu_v6_dcache_clean_area
	.word	cpu_v6_switch_mm
	.word	cpu_v6_set_pte
	.size	v6_processor_functions, . - v6_processor_functions

	.type	cpu_arch_name, #object
cpu_arch_name:
	.asciz	"armv6"
	.size	cpu_arch_name, . - cpu_arch_name

	.type	cpu_elf_name, #object
cpu_elf_name:
	.asciz	"v6"
	.size	cpu_elf_name, . - cpu_elf_name
	.align

	.section ".proc.info.init", #alloc, #execinstr

	/*
	 * Match any ARMv6 processor core.
	 */
	.type	__v6_proc_info, #object
__v6_proc_info:
	.long	0x0007b000
	.long	0x0007f000
	.long   PMD_TYPE_SECT | \
		PMD_SECT_BUFFERABLE | \
		PMD_SECT_CACHEABLE | \
		PMD_SECT_AP_WRITE | \
		PMD_SECT_AP_READ
	b	__v6_setup
	.long	cpu_arch_name
	.long	cpu_elf_name
	.long	HWCAP_SWP|HWCAP_HALF|HWCAP_THUMB|HWCAP_FAST_MULT|HWCAP_VFP|HWCAP_EDSP|HWCAP_JAVA
	.long	cpu_v6_name
	.long	v6_processor_functions
	.long	v6wbi_tlb_fns
	.long	v6_user_fns
	.long	v6_cache_fns
	.size	__v6_proc_info, . - __v6_proc_info
