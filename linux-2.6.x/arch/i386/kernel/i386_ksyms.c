#include <linux/config.h>
#include <linux/module.h>
#include <asm/checksum.h>
#include <asm/desc.h>

/* This is definitely a GPL-only symbol */
EXPORT_SYMBOL_GPL(cpu_gdt_table);

#ifdef CONFIG_ASM_SEMAPHORES
EXPORT_SYMBOL(__compat_down_failed);
EXPORT_SYMBOL(__compat_down_failed_interruptible);
EXPORT_SYMBOL(__compat_down_failed_trylock);
EXPORT_SYMBOL(__compat_up_wakeup);
#endif
/* Networking helper routines. */
EXPORT_SYMBOL(csum_partial_copy_generic);

EXPORT_SYMBOL(__get_user_1);
EXPORT_SYMBOL(__get_user_2);
EXPORT_SYMBOL(__get_user_4);

EXPORT_SYMBOL(__put_user_1);
EXPORT_SYMBOL(__put_user_2);
EXPORT_SYMBOL(__put_user_4);
EXPORT_SYMBOL(__put_user_8);

EXPORT_SYMBOL(strpbrk);
EXPORT_SYMBOL(strstr);

#ifdef CONFIG_SMP
extern void FASTCALL( __write_lock_failed(rwlock_t *rw));
extern void FASTCALL( __read_lock_failed(rwlock_t *rw));
#ifdef CONFIG_ASM_SEMAPHORES
EXPORT_SYMBOL(__write_lock_failed);
EXPORT_SYMBOL(__read_lock_failed);
#endif
#endif

EXPORT_SYMBOL(csum_partial);
