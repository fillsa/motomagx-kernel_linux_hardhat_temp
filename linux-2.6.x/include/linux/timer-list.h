#ifndef _LINUX_TIMER_LIST_H
#define _LINUX_TIMER_LIST_H
/*
 * include/linux/timer-list.h
 *
*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2, or (at your option) any
 * later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * The internal timer list defines and declarations.
 *
 *  Copyright (C) 1991, 1992  Linus Torvalds
 *  2002-10-01	High res timers code by George Anzinger 
 *		    Copyright (C)2002 by MontaVista Software.
 *  2004        Added VST and IDLE code, by George Anzinger
 *                  Copyright (C)2004 by MontaVista Software.
 *                  Copyright 2004 Sony Corporation.
 *                  Copyright 2004 Matsushita Electric Industrial Co., Ltd.
 */
#include <linux/config.h>
#include <linux/percpu.h>
#include <linux/vst.h>

/*
 * per-CPU timer vector definitions:
 */

#define TVN_BITS (CONFIG_BASE_SMALL ? 4 : 6)
#define TVR_BITS (CONFIG_BASE_SMALL ? 6 : 8)
#define TVN_SIZE (1 << TVN_BITS)
#define TVR_SIZE (1 << TVR_BITS)
#define TVN_MASK (TVN_SIZE - 1)
#define TVR_MASK (TVR_SIZE - 1)

struct timer_base_s {
	spinlock_t lock;
	struct timer_list *running_timer;
};

typedef struct tvec_s {
	struct list_head vec[TVN_SIZE];
} tvec_t;

typedef struct tvec_root_s {
	struct list_head vec[TVR_SIZE];
} tvec_root_t;

struct tvec_t_base_s {
	struct timer_base_s t_base;
	unsigned long timer_jiffies;
	wait_queue_head_t wait_for_running_timer;
	tvec_root_t tv1;
	tvec_t tv2;
	tvec_t tv3;
	tvec_t tv4;
	tvec_t tv5;
	VST_VISIT_COUNT
} ____cacheline_aligned_in_smp;

typedef struct tvec_t_base_s tvec_base_t;
static DEFINE_PER_CPU(tvec_base_t, tvec_bases);


#endif
