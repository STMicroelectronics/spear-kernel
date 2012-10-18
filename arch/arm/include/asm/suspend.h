#ifndef __ASM_ARM_SUSPEND_H
#define __ASM_ARM_SUSPEND_H

static inline int arch_prepare_suspend(void) { return 0; }

extern void cpu_resume(void);
extern int cpu_suspend(unsigned long, int (*)(unsigned long));

#endif
