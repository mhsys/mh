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
#define MAXDEVDS 16

#define copy_to_user(uaddr, src, sz) __usrcpy(uaddr, (void *)uaddr, src, sz)
#define copy_from_user(dst, uaddr, sz) __usrcpy(uaddr, dst, (void *)uaddr, sz)

#define THST_RUNNING 0
#define THST_RUNNABLE 1
#define THST_STOPPED 2
#define THST_DELETED 3

#define thread_is_idle(_th) (_th == current_cpu()->idle_thread)
#define thread_has_interrupts(_th) (((_th)->userfl & THFL_INTR)	\
				    && (_th)->softintrs)

struct thread {
	jmp_buf ctx;
	struct pmap *pmap;

	void *stack_4k;
	void *frame;

	struct usrdev *usrdev;
	struct bus bus;

	int setuid;
	uid_t ruid;
	uid_t euid;
	uid_t suid;
	gid_t rgid;
	gid_t egid;
	gid_t sgid;

	lock_t bstrap_lock;
	void *bstrap_ptr;
	size_t bstrap_sz;
	uid_t bstrap_uid;
	gid_t bstrap_gid;

	uaddr_t sigip;
	uaddr_t sigsp;
#define THFL_INTR (1L << 0)
	uint32_t userfl;

	u_long softintrs;
	uint16_t status;
	unsigned cpu;
	TAILQ_ENTRY(thread) sched_list;
};

struct cpu {
	struct thread *idle_thread;
	TAILQ_HEAD(, thread) resched;
	uint64_t softirq;

	unsigned usrpgfault;
	jmp_buf usrpgfaultctx;
};

int __getperm(struct thread *th, uid_t uid, gid_t gid, uint32_t mode);
int __usrcpy(uaddr_t uaddr, void *dst, void *src, size_t sz);

void thintr(unsigned xcpt, vaddr_t va, unsigned long err);
void thbootstrap(struct thread *th, uid_t uid, gid_t gid, void *ptr, size_t sz);
void kern_boot(void);
void kern_bootap(void);

struct cpu *cpu_setup(int id);
void cpu_softirq_raise(int);
void do_softirq(void);

struct thread *thfork(void);
void thraise(struct thread *th, unsigned vect);

unsigned vmpopulate(vaddr_t addr, size_t sz, pmap_prot_t prot);
unsigned vmclear(vaddr_t addr, size_t sz);
int vmmap(vaddr_t, pmap_prot_t prot);
int vmmove(vaddr_t dst, vaddr_t src);
int vmchprot(vaddr_t, pmap_prot_t prot);
int vmunmap(vaddr_t);

int devcreat(uint64_t id, unsigned sig, devmode_t mode);
int devpoll(struct sys_poll_ior *ior);
int deveio(unsigned id);
int devirq(unsigned id, unsigned irq);
int devimport(unsigned id, unsigned iopfn, unsigned va);
void devremove(void);
int devopen(uint64_t id);
int devirqmap(unsigned dd, unsigned irq, unsigned sig);
int devexport(unsigned dd, vaddr_t va, unsigned iopfn);
int devin(unsigned dd, uint64_t port, uint64_t *valptr);
int devout(unsigned dd, uint64_t port, uint64_t val);
void devclose(unsigned dd);

void wake(struct thread *);
void schedule(int newst);
void die(void) __dead;

struct irqsig {
	struct thread *th;
	unsigned sig;
	LIST_ENTRY(irqsig) list;
};

void irqsignal(unsigned irq);
int irqregister(struct irqsig *irqsig, unsigned irq, struct thread *th, unsigned sig);
void irqunregister(struct irqsig *irqsig);

#endif
