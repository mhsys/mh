/*
 * Copyright (c) 2015, Gianluca Guida
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#ifndef __cpu_h
#define __cpu_h

#ifndef _ASSEMBLER
#include <uk/types.h>
#include <uk/assert.h>
#include <uk/queue.h>
#include <uk/pcpu.h>

#define UKERN_MAX_CPUS 64
#define UKERN_MAX_PHYSCPUS 64

struct cpu_info {
	/* Must be first */
	uint32_t cpu_id;	/* fs:0 */
	struct thread *thread;	/* fs:4 */
	struct cpu_info *self;
	struct pcpu pcpu;

	uint16_t phys_id;
	uint16_t plat_id;

	struct thread *idle_thread;
	uint64_t softirq;

	unsigned usrpgfault;
	jmp_buf usrpgfaultctx;
	vaddr_t usrpgaddr;

	int tlbop; /* TLB shootdown */

	 TAILQ_HEAD(, thread) resched;
};

extern cpumask_t cpus_active;

void cpu_wakeall(void);
void cpu_enter(void);
int cpu_add(uint16_t physid, uint16_t acpiid);
struct cpu_info *cpuinfo_get(unsigned id);

void cpu_nmi(int cpu);
void cpu_nmi_mask(cpumask_t map);
void cpu_nmi_broadcast(void);
void cpu_ipi(int cpu, uint8_t vct);
void cpu_ipi_mask(cpumask_t map, uint8_t vct);
void cpu_ipi_broadcast(uint8_t vct);

/* FFS, use ffs! */
#define once_cpumask(__mask, __op)					\
	do {								\
		int i = 0;						\
		cpumask_t m = (__mask) & cpus_active;			\
									\
		/* https://xkcd.com/292/ */				\
		/* allow safe use of continue */			\
		goto _start;						\
		while (m != 0) {					\
			i++;						\
			m >>= 1;					\
		_start:							\
			if (m & 1) {					\
				__op;					\
				break;					\
			}						\
		};							\
	} while (0)

#define foreach_cpumask(__mask, __op)					\
	do {								\
		int i = 0;						\
		cpumask_t m = (__mask) & cpus_active;			\
									\
		/* https://xkcd.com/292/ */				\
		/* allow safe use of continue */			\
		goto _start;						\
		while (m != 0) {					\
			i++;						\
			m >>= 1;					\
		_start:							\
			if (m & 1) __op;				\
		};							\
	} while(0)

static inline struct cpu_info *current_cpu(void)
{
	return (struct cpu_info *)pcpu_getdata();
}

static inline int cpu_number(void)
{
	return current_cpu()->cpu_id;
}

static inline struct thread *current_thread(void)
{
	return current_cpu()->thread;
}

static inline void set_current_thread(struct thread *t)
{
	current_cpu()->thread = t;
}

static inline struct pcpu *current_pcpu(void)
{
	return &current_cpu()->pcpu;
}


#else				/* _ASSEMBLER */

#define CPU_NUMBER(_reg)			\
    movl %fs:0, _reg;				\
    movl (_reg), _reg

#define CURRENT_THREAD(_reg)			\
    movl %fs:0, _reg;				\
    movl 4(_reg), _reg

#endif

#endif
