/*
 *  linux/fs/proc/proc_misc.c
 *
 *  Copyright (C) 2006 Motorola Inc.
 *  Copyright (C) 1992  by Linus Torvalds
 *  based on ideas by Darren Senn
 *
 */

/* ChangeLog:
 * (mm-dd-yyyy) Author    Comment
 * 06-27-2006   Motorola  Add SLABFREE info to /proc/slabinfo 
 * 			  Make meminfo_read_proc available for kpanic
 */

/*  This used to be the part of array.c. See the rest of history and credits
 *  there. I took this into a separate file and switched the thing to generic
 *  proc_file_inode_operations, leaving in array.c only per-process stuff.
 *  Inumbers allocation made dynamic (via create_proc_entry()).  AV, May 1999.
 *
 * Changes:
 * Fulton Green      :  Encapsulated position metric calculations.
 *			<kernel@FultonGreen.com>
 */

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <linux/kernel.h>
#include <linux/kernel_stat.h>
#include <linux/tty.h>
#include <linux/string.h>
#include <linux/mman.h>
#include <linux/proc_fs.h>
#include <linux/ioport.h>
#include <linux/config.h>
#include <linux/mm.h>
#include <linux/mmzone.h>
#include <linux/pagemap.h>
#include <linux/swap.h>
#include <linux/slab.h>
#include <linux/smp.h>
#include <linux/signal.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/smp_lock.h>
#include <linux/seq_file.h>
#include <linux/times.h>
#include <linux/profile.h>
#include <linux/blkdev.h>
#include <linux/hugetlb.h>
#include <linux/jiffies.h>
#include <linux/sysrq.h>
#include <linux/vmalloc.h>
#include <asm/uaccess.h>
#include <asm/pgtable.h>
#include <asm/io.h>
#include <asm/tlb.h>
#include <asm/div64.h>

#define LOAD_INT(x) ((x) >> FSHIFT)
#define LOAD_FRAC(x) LOAD_INT(((x) & (FIXED_1-1)) * 100)
/*
 * Warning: stuff below (imported functions) assumes that its output will fit
 * into one page. For some of those functions it may be wrong. Moreover, we
 * have a way to deal with that gracefully. Right now I used straightforward
 * wrappers, but this needs further analysis wrt potential overflows.
 */
extern int get_hardware_list(char *);
extern int get_stram_list(char *);
extern int get_chrdev_list(char *);
extern int get_blkdev_list(char *);
extern int get_filesystem_list(char *);
extern int get_exec_domain_list(char *);
extern int get_dma_list(char *);
extern int get_locks_status (char *, char **, off_t, int);
#ifdef CONFIG_MOT_FEAT_KPANIC
extern int kpanic_in_progress;
#endif /* CONFIG_MOT_FEAT_KPANIC */

static int proc_calc_metrics(char *page, char **start, off_t off,
				 int count, int *eof, int len)
{
	if (len <= off+count) *eof = 1;
	*start = page + off;
	len -= off;
	if (len>count) len = count;
	if (len<0) len = 0;
	return len;
}

static int loadavg_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	int a, b, c;
	int len;

	a = avenrun[0] + (FIXED_1/200);
	b = avenrun[1] + (FIXED_1/200);
	c = avenrun[2] + (FIXED_1/200);
	len = sprintf(page,"%d.%02d %d.%02d %d.%02d %ld/%d %d\n",
		LOAD_INT(a), LOAD_FRAC(a),
		LOAD_INT(b), LOAD_FRAC(b),
		LOAD_INT(c), LOAD_FRAC(c),
		nr_running(), nr_threads, last_pid);
	return proc_calc_metrics(page, start, off, count, eof, len);
}

struct vmalloc_info {
	unsigned long used;
	unsigned long largest_chunk;
};

static struct vmalloc_info get_vmalloc_info(void)
{
	unsigned long prev_end = VMALLOC_START;
	struct vm_struct* vma;
	struct vmalloc_info vmi;
	vmi.used = 0;

	read_lock(&vmlist_lock);

	if(!vmlist)
		vmi.largest_chunk = (VMALLOC_END-VMALLOC_START);
	else
		vmi.largest_chunk = 0;

	for (vma = vmlist; vma; vma = vma->next) {
		unsigned long free_area_size =
			(unsigned long)vma->addr - prev_end;
		vmi.used += vma->size;
		if (vmi.largest_chunk < free_area_size )

			vmi.largest_chunk = free_area_size;
		prev_end = vma->size + (unsigned long)vma->addr;
	}
	if(VMALLOC_END-prev_end > vmi.largest_chunk)
		vmi.largest_chunk = VMALLOC_END-prev_end;

	read_unlock(&vmlist_lock);
	return vmi;
}

static int uptime_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	struct timespec uptime;
	struct timespec idle;
	int len;
	u64 idle_jiffies = init_task.utime + init_task.stime;

	do_posix_clock_monotonic_gettime(&uptime);
	jiffies_to_timespec(idle_jiffies, &idle);
	len = sprintf(page,"%lu.%02lu %lu.%02lu\n",
			(unsigned long) uptime.tv_sec,
			(uptime.tv_nsec / (NSEC_PER_SEC / 100)),
			(unsigned long) idle.tv_sec,
			(idle.tv_nsec / (NSEC_PER_SEC / 100)));

	return proc_calc_metrics(page, start, off, count, eof, len);
}

#ifndef CONFIG_MOT_FEAT_KPANIC
static 
#endif /* CONFIG_MOT_FEAT_KPANIC */
int meminfo_read_proc(char *page, char **start, off_t off,
                                 int count, int *eof, void *data)
{
	struct sysinfo i;
	int len;
	struct page_state ps;
	unsigned long inactive;
	unsigned long active;
	unsigned long free;
	unsigned long vmtot;
	unsigned long committed;
	unsigned long allowed;
	struct vmalloc_info vmi;

	get_page_state(&ps);
	get_zone_counts(&active, &inactive, &free);

/*
 * display in kilobytes.
 */
#define K(x) ((x) << (PAGE_SHIFT - 10))
	si_meminfo(&i);
	si_swapinfo(&i);
	committed = atomic_read(&vm_committed_space);
	allowed = ((totalram_pages - hugetlb_total_pages())
		* sysctl_overcommit_ratio / 100) + total_swap_pages;

	vmtot = (VMALLOC_END-VMALLOC_START)>>10;
	vmi = get_vmalloc_info();
	vmi.used >>= 10;
	vmi.largest_chunk >>= 10;

	/*
	 * Tagged format, for easy grepping and expansion.
	 */
	len = sprintf(page,
		"MemTotal:     %8lu kB\n"
		"MemFree:      %8lu kB\n"
		"Buffers:      %8lu kB\n"
		"Cached:       %8lu kB\n"
		"SwapCached:   %8lu kB\n"
		"Active:       %8lu kB\n"
		"Inactive:     %8lu kB\n"
		"HighTotal:    %8lu kB\n"
		"HighFree:     %8lu kB\n"
		"LowTotal:     %8lu kB\n"
		"LowFree:      %8lu kB\n"
		"SwapTotal:    %8lu kB\n"
		"SwapFree:     %8lu kB\n"
		"Dirty:        %8lu kB\n"
		"Writeback:    %8lu kB\n"
		"Mapped:       %8lu kB\n"
		"Slab:         %8lu kB\n"
		"CommitLimit:  %8lu kB\n"
		"Committed_AS: %8lu kB\n"
		"PageTables:   %8lu kB\n"
		"VmallocTotal: %8lu kB\n"
		"VmallocUsed:  %8lu kB\n"
		"VmallocChunk: %8lu kB\n",
		K(i.totalram),
		K(i.freeram),
		K(i.bufferram),
		K(get_page_cache_size()-total_swapcache_pages-i.bufferram),
		K(total_swapcache_pages),
		K(active),
		K(inactive),
		K(i.totalhigh),
		K(i.freehigh),
		K(i.totalram-i.totalhigh),
		K(i.freeram-i.freehigh),
		K(i.totalswap),
		K(i.freeswap),
		K(ps.nr_dirty),
		K(ps.nr_writeback),
		K(ps.nr_mapped),
		K(ps.nr_slab),
		K(allowed),
		K(committed),
		K(ps.nr_page_table_pages),
		vmtot,
		vmi.used,
		vmi.largest_chunk
		);

		len += hugetlb_report_meminfo(page + len);

#ifdef CONFIG_MOT_FEAT_KPANIC
	if (kpanic_in_progress) {
		printk(KERN_EMERG "%s\n", page);
		/* the return value will be ignored */
		return -1;
	}
	else
#endif /* CONFIG_MOT_FEAT_KPANIC */
	return proc_calc_metrics(page, start, off, count, eof, len);
#undef K
}

extern struct seq_operations fragmentation_op;
static int fragmentation_open(struct inode *inode, struct file *file)
{
	(void)inode;
	return seq_open(file, &fragmentation_op);
}

static struct file_operations fragmentation_file_operations = {
	.open		= fragmentation_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
};

static int version_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	extern char *linux_banner;
	int len;

	strcpy(page, linux_banner);
	len = strlen(page);
	return proc_calc_metrics(page, start, off, count, eof, len);
}

extern struct seq_operations cpuinfo_op;
static int cpuinfo_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &cpuinfo_op);
}
static struct file_operations proc_cpuinfo_operations = {
	.open		= cpuinfo_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
};

extern struct seq_operations vmstat_op;
static int vmstat_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &vmstat_op);
}
static struct file_operations proc_vmstat_file_operations = {
	.open		= vmstat_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
};

#ifdef CONFIG_PROC_HARDWARE
static int hardware_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	int len = get_hardware_list(page);
	return proc_calc_metrics(page, start, off, count, eof, len);
}
#endif

#ifdef CONFIG_STRAM_PROC
static int stram_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	int len = get_stram_list(page);
	return proc_calc_metrics(page, start, off, count, eof, len);
}
#endif

extern struct seq_operations partitions_op;
static int partitions_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &partitions_op);
}
static struct file_operations proc_partitions_operations = {
	.open		= partitions_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
};

extern struct seq_operations diskstats_op;
static int diskstats_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &diskstats_op);
}
static struct file_operations proc_diskstats_operations = {
	.open		= diskstats_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
};

#ifdef CONFIG_MODULES
extern struct seq_operations modules_op;
static int modules_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &modules_op);
}
static struct file_operations proc_modules_operations = {
	.open		= modules_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
};
#endif

extern struct seq_operations slabinfo_op;
extern ssize_t slabinfo_write(struct file *, const char __user *, size_t, loff_t *);
static int slabinfo_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &slabinfo_op);
}
static struct file_operations proc_slabinfo_operations = {
	.open		= slabinfo_open,
	.read		= seq_read,
	.write		= slabinfo_write,
	.llseek		= seq_lseek,
	.release	= seq_release,
};

int show_stat(struct seq_file *p, void *v)
{
	int i;
	extern unsigned long total_forks;
	unsigned long jif;
	u64	sum = 0, user = 0, nice = 0, system = 0,
		idle = 0, iowait = 0, irq = 0, softirq = 0;

	jif = - wall_to_monotonic.tv_sec;
	if (wall_to_monotonic.tv_nsec)
		--jif;

	for_each_cpu(i) {
		int j;

		user += kstat_cpu(i).cpustat.user;
		nice += kstat_cpu(i).cpustat.nice;
		system += kstat_cpu(i).cpustat.system;
		idle += kstat_cpu(i).cpustat.idle;
		iowait += kstat_cpu(i).cpustat.iowait;
		irq += kstat_cpu(i).cpustat.irq;
		softirq += kstat_cpu(i).cpustat.softirq;
		for (j = 0 ; j < NR_IRQS ; j++)
			sum += kstat_cpu(i).irqs[j];
	}

	seq_printf(p, "cpu  %llu %llu %llu %llu %llu %llu %llu\n",
		(unsigned long long)jiffies_64_to_clock_t(user),
		(unsigned long long)jiffies_64_to_clock_t(nice),
		(unsigned long long)jiffies_64_to_clock_t(system),
		(unsigned long long)jiffies_64_to_clock_t(idle),
		(unsigned long long)jiffies_64_to_clock_t(iowait),
		(unsigned long long)jiffies_64_to_clock_t(irq),
		(unsigned long long)jiffies_64_to_clock_t(softirq));
	for_each_online_cpu(i) {

		/* Copy values here to work around gcc-2.95.3, gcc-2.96 */
		user = kstat_cpu(i).cpustat.user;
		nice = kstat_cpu(i).cpustat.nice;
		system = kstat_cpu(i).cpustat.system;
		idle = kstat_cpu(i).cpustat.idle;
		iowait = kstat_cpu(i).cpustat.iowait;
		irq = kstat_cpu(i).cpustat.irq;
		softirq = kstat_cpu(i).cpustat.softirq;
		seq_printf(p, "cpu%d %llu %llu %llu %llu %llu %llu %llu\n",
			i,
			(unsigned long long)jiffies_64_to_clock_t(user),
			(unsigned long long)jiffies_64_to_clock_t(nice),
			(unsigned long long)jiffies_64_to_clock_t(system),
			(unsigned long long)jiffies_64_to_clock_t(idle),
			(unsigned long long)jiffies_64_to_clock_t(iowait),
			(unsigned long long)jiffies_64_to_clock_t(irq),
			(unsigned long long)jiffies_64_to_clock_t(softirq));
	}
	seq_printf(p, "intr %llu", (unsigned long long)sum);

#if !defined(CONFIG_PPC64) && !defined(CONFIG_ALPHA)
	for (i = 0; i < NR_IRQS; i++)
		seq_printf(p, " %u", kstat_irqs(i));
#endif

	seq_printf(p,
		"\nctxt %llu\n"
		"btime %lu\n"
		"processes %lu\n"
		"procs_running %lu\n"
		"procs_blocked %lu\n",
		nr_context_switches(),
		(unsigned long)jif,
		total_forks,
		nr_running(),
		nr_iowait());

#ifdef CONFIG_PREEMPT_RT
	{
		unsigned long nr_uninterruptible_cpu(int cpu);
		extern int pi_walk, pi_null, pi_prio;
		extern int rt_overload_schedule,
			   rt_overload_wakeup, rt_overload_pulled;
		unsigned long rt_nr_running_cpu(int cpu);
		extern atomic_t rt_overload;

		int i;

		seq_printf(p, "rt_overload_schedule: %d\n",
					rt_overload_schedule);
		seq_printf(p, "rt_overload_wakeup:   %d\n",
					rt_overload_wakeup);
		seq_printf(p, "rt_overload_pulled:   %d\n",
					rt_overload_pulled);
		seq_printf(p, "pi_null: %d\n", pi_null);
		seq_printf(p, "pi_prio: %d\n", pi_prio);
		seq_printf(p, "pi_walk: %d\n", pi_walk);
		seq_printf(p, "nr_running(): %ld\n",
			nr_running());
		seq_printf(p, "nr_uninterruptible(): %ld\n",
			nr_uninterruptible());
		for_each_cpu(i)
			seq_printf(p, "nr_uninterruptible(%d): %ld\n",
				i, nr_uninterruptible_cpu(i));
		for_each_cpu(i)
			seq_printf(p, "rt_nr_running(%d): %ld\n",
				i, rt_nr_running_cpu(i));
		seq_printf(p, "rt_overload: %d\n", atomic_read(&rt_overload));
		
	}
#endif

	return 0;
}

static int stat_open(struct inode *inode, struct file *file)
{
	unsigned size = 4096 * (1 + num_possible_cpus() / 32);
	char *buf;
	struct seq_file *m;
	int res;

	/* don't ask for more than the kmalloc() max size, currently 128 KB */
	if (size > 128 * 1024)
		size = 128 * 1024;
	buf = kmalloc(size, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	res = single_open(file, show_stat, NULL);
	if (!res) {
		m = file->private_data;
		m->buf = buf;
		m->size = size;
	} else
		kfree(buf);
	return res;
}
static struct file_operations proc_stat_operations = {
	.open		= stat_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int devices_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	int len = get_chrdev_list(page);
	len += get_blkdev_list(page+len);
	return proc_calc_metrics(page, start, off, count, eof, len);
}

/*
 * /proc/interrupts
 */
static void *int_seq_start(struct seq_file *f, loff_t *pos)
{
	return (*pos <= NR_IRQS) ? pos : NULL;
}

static void *int_seq_next(struct seq_file *f, void *v, loff_t *pos)
{
	(*pos)++;
	if (*pos > NR_IRQS)
		return NULL;
	return pos;
}

static void int_seq_stop(struct seq_file *f, void *v)
{
	/* Nothing to do */
}


extern int show_interrupts(struct seq_file *f, void *v); /* In arch code */
static struct seq_operations int_seq_ops = {
	.start = int_seq_start,
	.next  = int_seq_next,
	.stop  = int_seq_stop,
	.show  = show_interrupts
};

int interrupts_open(struct inode *inode, struct file *filp)
{
	return seq_open(filp, &int_seq_ops);
}

static struct file_operations proc_interrupts_operations = {
	.open		= interrupts_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
};

static int filesystems_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	int len = get_filesystem_list(page);
	return proc_calc_metrics(page, start, off, count, eof, len);
}

static int cmdline_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	int len;

	len = sprintf(page, "%s\n", saved_command_line);
	return proc_calc_metrics(page, start, off, count, eof, len);
}

static int locks_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	int len = get_locks_status(page, start, off, count);

	if (len < count)
		*eof = 1;
	return len;
}

static int execdomains_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	int len = get_exec_domain_list(page);
	return proc_calc_metrics(page, start, off, count, eof, len);
}

#ifdef CONFIG_LATENCY_TRACE
extern struct seq_operations latency_trace_op;
static int latency_trace_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &latency_trace_op);
}
static struct file_operations proc_latency_trace_operations = {
	.open		= latency_trace_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
};
#endif

#ifdef CONFIG_MAGIC_SYSRQ
/*
 * writing 'C' to /proc/sysrq-trigger is like sysrq-C
 */
static ssize_t write_sysrq_trigger(struct file *file, const char __user *buf,
				   size_t count, loff_t *ppos)
{
	if (count) {
		char c;

		if (get_user(c, buf))
			return -EFAULT;
		__handle_sysrq(c, NULL, NULL);
	}
	return count;
}

static struct file_operations proc_sysrq_trigger_operations = {
	.write		= write_sysrq_trigger,
};
#endif

#ifdef CONFIG_MOT_FEAT_SLABFREE_PROC
extern int slab_cache_freeable_info(void);
static int slabfree_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{       
        int ret = 0;
        int len = 0;
                
        ret = slab_cache_freeable_info();
        if (ret == -EAGAIN)
                len = sprintf(page, "BUSY: %d\n", -1);
        else
                len = sprintf(page, "SLABFREE: %d\n", (unsigned int)(ret * PAGE_SIZE));

        return proc_calc_metrics(page, start, off, count, eof, len);
}
#endif /* CONFIG_MOT_FEAT_SLABFREE_PROC */

struct proc_dir_entry *proc_root_kcore;

static void create_seq_entry(char *name, mode_t mode, struct file_operations *f)
{
	struct proc_dir_entry *entry;
	entry = create_proc_entry(name, mode, NULL);
	if (entry)
		entry->proc_fops = f;
}

void __init proc_misc_init(void)
{
	struct proc_dir_entry *entry;
	static struct {
		char *name;
		int (*read_proc)(char*,char**,off_t,int,int*,void*);
	} *p, simple_ones[] = {
		{"loadavg",     loadavg_read_proc},
		{"uptime",	uptime_read_proc},
		{"meminfo",	meminfo_read_proc},
		{"version",	version_read_proc},
#ifdef CONFIG_PROC_HARDWARE
		{"hardware",	hardware_read_proc},
#endif
#ifdef CONFIG_STRAM_PROC
		{"stram",	stram_read_proc},
#endif
		{"devices",	devices_read_proc},
		{"filesystems",	filesystems_read_proc},
		{"cmdline",	cmdline_read_proc},
		{"locks",	locks_read_proc},
		{"execdomains",	execdomains_read_proc},
#ifdef CONFIG_MOT_FEAT_SLABFREE_PROC
		{"slabfree", slabfree_read_proc},
#endif /* CONFIG_MOT_FEAT_SLABFREE_PROC */
		{NULL,}
	};
	for (p = simple_ones; p->name; p++)
		create_proc_read_entry(p->name, 0, NULL, p->read_proc, NULL);

	proc_symlink("mounts", NULL, "self/mounts");

	/* And now for trickier ones */
	entry = create_proc_entry("kmsg", S_IRUSR, &proc_root);
	if (entry)
		entry->proc_fops = &proc_kmsg_operations;
	create_seq_entry("cpuinfo", 0, &proc_cpuinfo_operations);
	create_seq_entry("partitions", 0, &proc_partitions_operations);
	create_seq_entry("stat", 0, &proc_stat_operations);
	create_seq_entry("interrupts", 0, &proc_interrupts_operations);
	create_seq_entry("slabinfo",S_IWUSR|S_IRUGO,&proc_slabinfo_operations);
	create_seq_entry("buddyinfo",S_IRUGO, &fragmentation_file_operations);
	create_seq_entry("vmstat",S_IRUGO, &proc_vmstat_file_operations);
	create_seq_entry("diskstats", 0, &proc_diskstats_operations);
#ifdef CONFIG_MODULES
	create_seq_entry("modules", 0, &proc_modules_operations);
#endif
#ifdef CONFIG_SCHEDSTATS
	create_seq_entry("schedstat", 0, &proc_schedstat_operations);
#endif
#ifdef CONFIG_LATENCY_TRACE
	create_seq_entry("latency_trace", 0, &proc_latency_trace_operations);
#endif
#ifdef CONFIG_PROC_KCORE
	proc_root_kcore = create_proc_entry("kcore", S_IRUSR, NULL);
	if (proc_root_kcore) {
		proc_root_kcore->proc_fops = &proc_kcore_operations;
		proc_root_kcore->size =
				(size_t)high_memory - PAGE_OFFSET + PAGE_SIZE;
	}
#endif
#ifdef CONFIG_MAGIC_SYSRQ
	entry = create_proc_entry("sysrq-trigger", S_IWUSR, NULL);
	if (entry)
		entry->proc_fops = &proc_sysrq_trigger_operations;
#endif
#ifdef CONFIG_PPC32
	{
		extern struct file_operations ppc_htab_operations;
		entry = create_proc_entry("ppc_htab", S_IRUGO|S_IWUSR, NULL);
		if (entry)
			entry->proc_fops = &ppc_htab_operations;
	}
#endif
}