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


#ifndef _ukern_kern_h
#define _ukern_kern_h

#include <uk/types.h>
#include <uk/errno.h>
#include <uk/queue.h>
#include <machine/uk/pmap.h>
#include <uk/sys.h>
#include <uk/bus.h>

#define MAXSIGNALS (sizeof(u_long) * 8)
#define MAXDEVS 16

#define copy_to_user(uaddr, src, sz) __usrcpy(uaddr, (void *)uaddr, src, sz)
#define copy_from_user(dst, uaddr, sz) __usrcpy(uaddr, dst, (void *)uaddr, sz)

int xcopy_from(vaddr_t va, struct thread *th, vaddr_t xva, size_t sz);
int xcopy_to(struct thread *th, vaddr_t xva, vaddr_t va, size_t sz);

#define THST_RUNNING 0
#define THST_RUNNABLE 1
#define THST_STOPPED 2
#define THST_ZOMBIE 3

#define thread_is_idle(_th) (_th == current_cpu()->idle_thread)
#define thread_has_interrupts(_th) (((_th)->userfl & THFL_INTR)	\
				    && (_th)->softintrs)

struct timer {
	int valid;
	uint64_t time;
	struct thread *th;
	int sig;
	void (*handler) (uint64_t);
	 LIST_ENTRY(timer) list;
};

struct thread {
	jmp_buf ctx;
	struct pmap *pmap;

	void *stack_4k;
	void *frame;

	struct usrdev *usrdevs[MAXDEVS];
	struct bus bus;

	struct thread *parent;
	lock_t children_lock;
	 LIST_HEAD(, thread) active_children;
	 LIST_HEAD(, thread) zombie_children;
	 LIST_ENTRY(thread) child_list;
	int exit_status;

	pid_t pid;

	int setuid;
	uid_t ruid;
	uid_t euid;
	uid_t suid;
	gid_t rgid;
	gid_t egid;
	gid_t sgid;

	uaddr_t sigip;
	uaddr_t sigsp;
#define THFL_INTR (1L << 0)
	uint32_t userfl;

	int vtt_almdiff;
	uint64_t vtt_offset;
	uint64_t vtt_rttbase;

	struct timer rtt_alarm;
	struct timer vtt_alarm;

	u_long softintrs;
	uint16_t status;
	unsigned cpu;
	 TAILQ_ENTRY(thread) sched_list;

	uaddr_t tls;
};

struct cpu {
	struct thread *idle_thread;
	 TAILQ_HEAD(, thread) resched;
	uint64_t softirq;

	unsigned usrpgfault;
	jmp_buf usrpgfaultctx;
	vaddr_t usrpgaddr;
};

int __getperm(struct thread *th, uid_t uid, gid_t gid, uint32_t mode);
int __usrcpy(uaddr_t uaddr, void *dst, void *src, size_t sz);

void klog_putc(int c);

void thintr(unsigned xcpt, vaddr_t va, unsigned long err);
void kern_boot(void);
void kern_bootap(void);

#define SOFTIRQ_ZOMBIE (1 << 0)
#define SOFTIRQ_TIMER  (1 << 1)
struct cpu *cpu_setup(int id);
void cpu_softirq_raise(int);
void cpu_kick(void);
void do_softirq(void);

struct thread *thfork(void);
void thraise(struct thread *th, unsigned vect);

int iomap(vaddr_t vaddr, pfn_t mmiopfn, pmap_prot_t prot);
int iounmap(vaddr_t vaddr);

unsigned vmpopulate(vaddr_t addr, size_t sz, pmap_prot_t prot);
unsigned vmclear(vaddr_t addr, size_t sz);
int vmmap(vaddr_t, pmap_prot_t prot);
int vmmap32(vaddr_t, pmap_prot_t prot);
int vmmove(vaddr_t dst, vaddr_t src);
int vmchprot(vaddr_t, pmap_prot_t prot);
int vmunmap(vaddr_t);

int hwcreat(struct sys_hwcreat_cfg *cfg, mode_t mode);

int childstat(struct sys_childstat *cs);

int devcreat(struct sys_creat_cfg *cfg, unsigned sig, devmode_t mode);
int devpoll(unsigned did, struct sys_poll_ior *ior);
int devwriospace(unsigned did, unsigned id, uint32_t port, uint64_t val);
int devirq(unsigned did, unsigned id, unsigned irq);
int devread(unsigned did, unsigned id, unsigned long iova, size_t sz, unsigned long va);
int devwrite(unsigned did, unsigned id, unsigned long va, size_t sz, unsigned long iova);
void devremove(unsigned did);

int devopen(uint64_t id);
int devirqmap(unsigned dd, unsigned irq, unsigned sig);
int devexport(unsigned dd, vaddr_t va, size_t sz, uint64_t *iopfn);
int devunexport(unsigned dd, vaddr_t va);
int devin(unsigned dd, uint32_t port, uint64_t * valptr);
int devout(unsigned dd, uint32_t port, uint64_t val);
int devinfo(unsigned dd, struct sys_info_cfg *cfg);
int deviomap(unsigned dd, vaddr_t va, paddr_t mmioaddr, pmap_prot_t prot);
int deviounmap(unsigned dd, vaddr_t va);
void devclose(unsigned dd);

void wake(struct thread *);
void schedule(int newst);
void die(void) __dead;

struct irqsig {
	struct thread *th;
	int eoi;
	unsigned sig;
	uint32_t filter;
	void (*handler) (unsigned);
	LIST_ENTRY(irqsig) list;
};

int doeoi(unsigned irq);
void irqsignal(unsigned irq, int level);
int irqregister(struct irqsig *irqsig, unsigned irq, struct thread *th,
		unsigned sig, uint32_t filter);
void irqunregister(struct irqsig *irqsig);

uint64_t timer_readcounter(void);
uint64_t timer_readperiod(void);
void timer_setcounter(uint64_t cnt);
void timer_event(void);

void thalrm(uint32_t diff);
void thvtalrm(uint32_t diff);
uint64_t thvtt(struct thread *th);

#endif
