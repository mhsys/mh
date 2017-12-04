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


#include <uk/stddef.h>
#include <uk/types.h>
#include <uk/setjmp.h>
#include <uk/string.h>
#include <uk/logio.h>
#include <uk/param.h>
#include <uk/elf.h>
#include <uk/locks.h>
#include <uk/pfndb.h>
#include <uk/fixmems.h>
#include <uk/heap.h>
#include <uk/vmap.h>
#include <uk/structs.h>
#include <uk/cpu.h>
#include <machine/uk/pmap.h>
#include <machine/uk/machdep.h>
#include <machine/uk/platform.h>
#include <machine/uk/pmap.h>
#include <uk/bus.h>
#include <uk/usrdev.h>
#include <uk/sys.h>
#include "kern.h"

static struct slab threads;
static cpumask_t cpu_idlemap = 0;

static struct thread *__kern_init = NULL;

static lock_t sched_lock = 0;
static TAILQ_HEAD(thread_list, thread) running_threads =
TAILQ_HEAD_INITIALIZER(running_threads);
static TAILQ_HEAD(, thread) stopped_threads =
TAILQ_HEAD_INITIALIZER(stopped_threads);

lock_t timers_lock = 0;
static LIST_HEAD(, timer) timers = LIST_HEAD_INITIALIZER(timers);

lock_t softirq_lock = 0;
uint64_t softirqs = 0;

static lock_t irqsigs_lock;
LIST_HEAD(, irqsig) irqsigs[MAXIRQS];

void __bad_thing(int user, const char *format, ...)
{
	va_list ap;

	extern void _boot_putc(int);
	_setputcfn(_boot_putc, NULL);
	if (user)
		printf("USRKILL: ");
	else
		printf("PANIC at CPU#%d: ", cpu_number());
	va_start(ap, format);
	vprintf(format, ap);
	va_end(ap);
	printf("\n");

	if (user)
		die();
	else
		__goodbye();
}

/* Use copy_{to,from}_user(), do not call this directly */
int __usrcpy(uaddr_t uaddr, void *dst, void *src, size_t sz)
{

	if (!__chkuaddr(uaddr, sz))
		return -EFAULT;

	current_cpu()->usrpgfault = 1;
	__insn_barrier();
	if (_setjmp(current_cpu()->usrpgfaultctx)) {
		current_cpu()->usrpgaddr = 0;
		current_cpu()->usrpgfault = 0;
		return -EFAULT;
	}

	memcpy(dst, src, sz);

	__insn_barrier();
	current_cpu()->usrpgfault = 0;
	current_cpu()->usrpgaddr = 0;
	return 0;
}

int __getperm(struct thread *th, uid_t uid, gid_t gid, mode_t mode)
{
	int perm;

	if (uid == th->euid) {
		perm = (mode & 0700);
		perm >>= 6;
	} else if (gid == th->egid) {
		perm = (mode & 0070);
		perm >>= 3;
	} else {
		perm = (mode & 0007);
	}
	return perm;
}

static pid_t getnewpid(void)
{
	static pid_t ret, pid = 0;

	ret = pid++;

	/* XXX: Okay, this is not serious. This is a hack. A bad one. */
	assert(pid != 0);
	return ret;
}

static void releasepid(pid_t pid)
{
}

static struct thread *thnew(void (*__start) (void))
{
	struct thread *th;

	th = structs_alloc(&threads);
	th->pmap = pmap_alloc();
	th->stack_4k = alloc4k();
	memset(th->stack_4k, 0, 4096);
	th->userfl = 0;
	th->softintrs = 0;

	th->pid = getnewpid();

	th->setuid = 0;
	th->ruid = 0;
	th->euid = 0;
	th->suid = 0;
	th->rgid = 0;
	th->egid = 0;
	th->sgid = 0;

	memset(&th->usrdevs, 0, sizeof(th->usrdevs));
	memset(&th->bus, 0, sizeof(th->bus));
	memset(&th->rtt_alarm, 0, sizeof(th->rtt_alarm));
	memset(&th->vtt_alarm, 0, sizeof(th->vtt_alarm));

	th->vtt_almdiff = 0;
	th->vtt_offset = 0;
	th->vtt_rttbase = 0;

	_setupjmp(th->ctx, __start, th->stack_4k + 0xff0);
	return th;
}

static void __childstart(void)
{
	struct thread *th = current_thread();

	/* Child returns zero */
	usrframe_setret(th->frame, 0);
	__insn_barrier();
	usrframe_enter(th->frame);
	/* Not reached */
}

struct thread *thfork(void)
{
	int i;
	vaddr_t va;
	struct thread *nth, *cth = current_thread();

	nth = structs_alloc(&threads);
	nth->pmap = pmap_copy();
	nth->stack_4k = alloc4k();
	nth->frame = (uint8_t *) nth->stack_4k;
	memcpy(nth->frame, cth->frame, sizeof(struct usrframe));

	nth->userfl = cth->userfl;
	nth->softintrs = cth->softintrs;

	nth->pid = getnewpid();

	nth->children_lock = 0;
	LIST_INIT(&nth->active_children);
	LIST_INIT(&nth->zombie_children);

	nth->setuid = cth->setuid;
	nth->ruid = cth->ruid;
	nth->euid = cth->euid;
	nth->suid = cth->suid;
	nth->rgid = cth->rgid;
	nth->egid = cth->egid;
	nth->sgid = cth->sgid;

	nth->sigip = cth->sigip;
	nth->sigsp = cth->sigsp;

	nth->vtt_almdiff = 0;
	nth->vtt_rttbase = 0;
	nth->vtt_offset = 0;

	/* XXX: What to do on fork */
	memset(&nth->rtt_alarm, 0, sizeof(nth->rtt_alarm));
	memset(&nth->vtt_alarm, 0, sizeof(nth->vtt_alarm));

	/* It makes sense not to clone user devs.
	 *
	 * 1. How should we name the new device?
	 * 2. It is straightforward from userspace to re-creat the
	 * same device.
	 * 3. It is more complex to clone from the kernel.
	 */
	memset(&nth->usrdevs, 0, sizeof(nth->usrdevs));

	/* Fork Bus */
	memset(&nth->bus, 0, sizeof(nth->bus));
	for (i = 0; i < MAXBUSDEVS; i++)
		bus_copy(&cth->bus, i, nth, &nth->bus, i);

	/* NB: Stack is mostly at the end of the 4k page.
	 * Frame is at the very beginning. */
	_setupjmp(nth->ctx, __childstart, nth->stack_4k + 0xff0);

	/* Copy the no cow area */
	for (va = ZCOWBASE; va < ZCOWEND; va += PAGE_SIZE) {
		pfn_t pfn;
		int ret;

		/* XXX: Allocate in KERN pool to directly map and
		 * copy, avoiding global vmapping. Fix properly by
		 * implementing local high mem mapping, using only one
		 * slot and a couple of INVLPG's */
		pfn = pgalloc(1, PFNT_USER, GFP_KERN);
		pfndb_clrref(pfn);
		if (copy_from_user((void *) ptova(pfn), va, PAGE_SIZE)) {
			/* Not present */
			pgfree(pfn, 1);
			continue;
		}
		ret = pmap_uenter(nth->pmap, va, pfn, PROT_USER_WR, &pfn);
		assert(!ret && (pfn == PFN_INVALID));
	}
	/* Finished handling the pmaps. */
	pmap_commit(NULL);

	nth->parent = cth;
	spinlock(&cth->children_lock);
	LIST_INSERT_HEAD(&cth->active_children, nth, child_list);
	spinunlock(&cth->children_lock);

	/* No need to flush nth->pmap, not used yet */
	nth->status = THST_RUNNABLE;
	spinlock(&sched_lock);
	TAILQ_INSERT_TAIL(&running_threads, nth, sched_list);
	spinunlock(&sched_lock);

	return nth;
}

void thintr(unsigned vect, vaddr_t va, unsigned long err)
{
	struct thread *th = current_thread();
	uint32_t ofl = th->userfl;

	th->userfl &= ~THFL_INTR;	/* Restored on IRET */
	usrframe_signal(th->frame, th->sigip, th->sigsp, ofl, vect, va,
			err);
}

void thextintr(unsigned vect, uint64_t sigs)
{
	struct thread *th = current_thread();
	uint32_t ofl = th->userfl;

	th->userfl &= ~THFL_INTR;	/* Restored on IRET */
	usrframe_extint(th->frame, th->sigip, th->sigsp, ofl, vect, sigs);
}

void thraise(struct thread *th, unsigned vect)
{

	assert(vect < MAXSIGNALS);
	__sync_or_and_fetch(&th->softintrs, (1LL << vect));
	wake(th);
}

static void thfree(struct thread *th)
{
	/* thread must not be active, on any cpu */
	pmap_free(th->pmap);
	free4k(th->stack_4k);
	releasepid(th->pid);
	structs_free(th);
}

static void thswitch(struct thread *th)
{
	pmap_switch(th->pmap);
	set_current_thread(th);
	if (thread_is_idle(th))
		__sync_or_and_fetch(&cpu_idlemap, (1L << cpu_number()));
	usrframe_switch();
}

struct cpu *cpu_setup(int id)
{
	struct cpu *cpu;

	cpu = heap_alloc(sizeof(struct cpu));
	cpu->idle_thread = NULL;
	TAILQ_INIT(&cpu->resched);
	cpu->softirq = 0;
	cpu->usrpgfault = 0;
	return cpu;
}

void cpu_softirq_raise(int id)
{
	current_cpu()->softirq |= id;
}

void cpu_kick(void)
{

	if (thread_is_idle(current_thread()))
		schedule(THST_STOPPED);
}

static void do_zombie(void);
static void do_timer(void);

void do_softirq(void)
{
	struct thread *th = current_thread();
	uint64_t si;

	if (th->userfl & THFL_INTR) {
		si = __sync_fetch_and_and(&th->softintrs, 0);
		if (si)
			thextintr(INTR_EXT, si);
	}

	while (current_cpu()->softirq) {
		si = current_cpu()->softirq;
		current_cpu()->softirq = 0;

		if (si & SOFTIRQ_ZOMBIE) {
			do_zombie();
			si &= ~SOFTIRQ_ZOMBIE;
		}

		if (si & SOFTIRQ_TIMER) {
			do_timer();
			si &= ~SOFTIRQ_TIMER;
		}
	}
}

static void do_zombie(void)
{
	struct thread *th, *tmp;

	TAILQ_FOREACH_SAFE(th, &current_cpu()->resched, sched_list, tmp) {
		TAILQ_REMOVE(&current_cpu()->resched, th, sched_list);
		assert(th->status == THST_ZOMBIE);
		spinlock(&th->parent->children_lock);
		LIST_REMOVE(th, child_list);
		LIST_INSERT_HEAD(&th->parent->zombie_children, th,
				 child_list);
		spinunlock(&th->parent->children_lock);
		thraise(th->parent, INTR_CHILD);
		break;
	}
}

void wake(struct thread *th)
{
	spinlock(&sched_lock);
	switch (th->status) {
	case THST_ZOMBIE:
		printf("Waking zombie thread?");
	case THST_RUNNABLE:
		break;
	case THST_RUNNING:
		cpu_ipi(th->cpu, VECT_KICK);
		break;
	case THST_STOPPED:
		TAILQ_REMOVE(&stopped_threads, th, sched_list);
		th->status = THST_RUNNABLE;
		TAILQ_INSERT_TAIL(&running_threads, th, sched_list);
		once_cpumask(cpu_idlemap, cpu_ipi(i, VECT_KICK));
		break;
	}
	spinunlock(&sched_lock);
}

void schedule(int newst)
{
	struct thread *th = NULL;
	struct thread *oldth;

	/* Nothing to do */
	if (newst == THST_RUNNING)
		return;

	/* Volatile: changes in this function. */
	oldth = (volatile struct thread *) current_thread();

	/* Do not sleep if active signals */
	if ((newst == THST_STOPPED) && thread_has_interrupts(oldth))
		return;

	if (!_setjmp(oldth->ctx)) {

		/* Default new  thread. If  we are just  yielding (our
		 * next status is RUNNABLE), don't go idle. */
		if (newst == THST_RUNNABLE)
			th = oldth;
		else
			th = current_cpu()->idle_thread;

		spinlock(&sched_lock);
		/* Schedule per-cpu? Actually simpler */
		if (!TAILQ_EMPTY(&running_threads)) {
			th = TAILQ_FIRST(&running_threads);
			TAILQ_REMOVE(&running_threads, th, sched_list);
			th->status = THST_RUNNING;
			th->cpu = cpu_number();
		}
		spinunlock(&sched_lock);

		if (th == oldth)
			goto _skip_resched;

		oldth->vtt_offset = thvtt(oldth);
		if (oldth->vtt_alarm.valid) {
			uint64_t ctime = timer_readcounter();
			uint64_t ttime = oldth->vtt_alarm.time;
			oldth->vtt_almdiff = ttime > ctime ?
				ttime - ctime : 1;
		}
		timer_remove(&oldth->vtt_alarm);
		thswitch(th);	/* SWITCH current_thread() here. */
		th->vtt_rttbase = timer_readcounter();
		if (th->vtt_almdiff) {
			thvtalrm(th->vtt_almdiff);
		}

		if (thread_is_idle(oldth))
			goto _skip_resched;

		/* resched old thread */
		oldth->status = newst;
		switch (oldth->status) {
		case THST_RUNNABLE:
			spinlock(&sched_lock);
			TAILQ_INSERT_TAIL(&running_threads, oldth,
					  sched_list);
			spinunlock(&sched_lock);
			break;
		case THST_STOPPED:
			spinlock(&sched_lock);
			TAILQ_INSERT_TAIL(&stopped_threads, oldth,
					  sched_list);
			spinunlock(&sched_lock);
			break;
		case THST_ZOMBIE:
			spinlock(&sched_lock);
			TAILQ_INSERT_TAIL(&current_cpu()->resched, oldth,
					  sched_list);
			spinunlock(&sched_lock);
			cpu_softirq_raise(SOFTIRQ_ZOMBIE);
			break;
		default:
			panic("Uknown schedule state %d\n", oldth->status);
		}

	      _skip_resched:

		_longjmp(th->ctx, 1);
		panic("WTF.");
	}
}

int childstat(struct sys_childstat *cs)
{
	pid_t pid;
	struct thread *child, *th = current_thread();

	spinlock(&th->children_lock);
	child = LIST_FIRST(&th->zombie_children);
	if (child != NULL)
		LIST_REMOVE(child, child_list);
	spinunlock(&th->children_lock);

	if (child == NULL)
		return -ESRCH;

	cs->exit_status = child->exit_status;
	pid = child->pid;
	thfree(child);
	return pid;
}

__dead void __exit(int status)
{
	int i;
	struct thread *th = current_thread();
	struct thread *child, *tmp;

	if (th->parent == NULL) {
		panic("Killed System Process %d\n", th->pid);
	}

	/* In the future, remove shared mapping mechanisms before
	 * mappings */
	bus_remove(&th->bus);
	for (i = 0; i < MAXDEVS; i++)
		devremove(i);
	vmclear(USERBASE, USEREND - USERBASE);
	/* Let INIT inherit children of dead process */
	spinlock(&th->children_lock);
	LIST_FOREACH_SAFE(child, &th->active_children, child_list, tmp) {
		LIST_REMOVE(child, child_list);
		/* Safe. We shouldn't ever be called with th == __kern_init */
		spinlock(&__kern_init->children_lock);
		LIST_INSERT_HEAD(&__kern_init->active_children,
				 child, child_list);
		spinunlock(&__kern_init->children_lock);
		child->parent = __kern_init;
	}
	LIST_FOREACH_SAFE(child, &th->zombie_children, child_list, tmp) {
		LIST_REMOVE(child, child_list);
		/* Safe. We shouldn't ever be called with th == __kern_init */
		spinlock(&__kern_init->children_lock);
		LIST_INSERT_HEAD(&__kern_init->zombie_children,
				 child, child_list);
		spinunlock(&__kern_init->children_lock);
		child->parent = __kern_init;
	}
	spinunlock(&th->children_lock);
	thraise(__kern_init, INTR_CHILD);

	th->exit_status = status;
	schedule(THST_ZOMBIE);
}

__dead void die(void)
{
	__exit(-2);
}

static void idle(void)
{
	schedule(THST_STOPPED);
	do_softirq();
	platform_idle();
}

int iomap(vaddr_t vaddr, pfn_t mmiopfn, pmap_prot_t prot)
{
	int ret = 0;
	pfn_t opfn;

	if (!pfn_is_mmio(mmiopfn))
		return -EINVAL;

	ret = pmap_uiomap(NULL, vaddr, mmiopfn, prot, &opfn);
	pmap_commit(NULL);

	if (ret < 0)
		return ret;

	if (opfn != PFN_INVALID) {
		__freepage(opfn);
		ret = 1;
	}
	return ret;
}

int iounmap(vaddr_t vaddr)
{
	int ret = 0;

	ret = pmap_uiounmap(NULL, vaddr);
	pmap_commit(NULL);
	return ret;
}

unsigned vmpopulate(vaddr_t addr, size_t sz, pmap_prot_t prot)
{
	int i, ret = 0;
	pfn_t pfn;

	for (i = 0; i < round_page(sz) >> PAGE_SHIFT; i++) {
		pfn = __allocuser();
		pmap_uenter(NULL, trunc_page(addr) + i * PAGE_SIZE, pfn,
			    prot, &pfn);
		if (pfn != PFN_INVALID) {
			__freepage(pfn);
			ret++;
		}
	}
	pmap_commit(NULL);
	/* Warning: accessing user space without checking (we are
	   single threaded, area just mapped and still locked, it is
	   fine, but future-fragile). */
	memset((void *) trunc_page(addr), 0, round_page(sz));
	return ret;
}

unsigned vmclear(vaddr_t addr, size_t sz)
{
	unsigned ret = 0;
	pfn_t pfn;
	int i;

	for (i = 0; i < round_page(sz) >> PAGE_SHIFT; i++) {
		pmap_uclear(NULL, addr + i * PAGE_SIZE, &pfn);
		if (pfn != PFN_INVALID) {
			__freepage(pfn);
			ret++;
		}
	}
	pmap_commit(NULL);
	return ret;
}

int vmmap(vaddr_t addr, pmap_prot_t prot)
{
	int ret = 0;
	pfn_t pfn, opfn;

	pfn = __allocuser();
	ret = pmap_uenter(NULL, addr, pfn, prot, &opfn);
	pmap_commit(NULL);

	if (ret < 0) {
		__freepage(pfn);
		return ret;
	}

	if (opfn != PFN_INVALID) {
		__freepage(opfn);
		ret = 1;
	}

	memset((void *)trunc_page(addr), 0, PAGE_SIZE);
	return ret;
}

int vmmap32(vaddr_t addr, pmap_prot_t prot)
{
	int ret = 0;
	pfn_t pfn, opfn;

	pfn = __allocuser32();
	ret = pmap_uenter(NULL, addr, pfn, prot, &opfn);
	pmap_commit(NULL);

	if (ret < 0) {
		__freepage(pfn);
		return ret;
	}

	if (opfn != PFN_INVALID) {
		__freepage(opfn);
		ret = 1;
	}
	return ret;
}

int vmunmap(vaddr_t addr)
{
	int ret = 0;
	pfn_t pfn;

	ret = pmap_uenter(NULL, addr, PFN_INVALID, 0, &pfn);
	pmap_commit(NULL);

	if (ret < 0)
		return ret;

	if (pfn != PFN_INVALID) {
		__freepage(pfn);
		ret = 1;
	}
	return ret;
}

int vmmove(vaddr_t dst, vaddr_t src)
{
	int ret;
	pfn_t pfn;

	ret = pmap_uchaddr(NULL, src, dst, &pfn);
	pmap_commit(NULL);

	if (ret < 0)
		return ret;

	if (pfn != PFN_INVALID) {
		__freepage(pfn);
		ret = 1;
	}
	return ret;
}

int vmchprot(vaddr_t addr, pmap_prot_t prot)
{
	int ret;

	ret = pmap_uchprot(NULL, addr, prot);
	pmap_commit(NULL);
	return ret;
}

int hwcreat(struct sys_hwcreat_cfg *cfg, mode_t mode)
{
	struct thread *th = current_thread();

	if (hwdev_creat(cfg, mode) == NULL)
		return -EEXIST;
	return 0;
}

int devcreat(struct sys_creat_cfg *cfg, unsigned sig, mode_t mode)
{
	int i;
	struct thread *th = current_thread();

	if (sig > MAXSIGNALS)
		return -EINVAL;

	for (i = 0; i < MAXDEVS; i++)
		if (th->usrdevs[i] == NULL)
			break;
	if (i >= MAXDEVS)
		return -EBUSY;

	th->usrdevs[i] = usrdev_creat(cfg, sig, mode);
	if (th->usrdevs[i] == NULL)
		return -EEXIST;
	return i;
}

int devpoll(unsigned did, struct sys_poll_ior *polld)
{
	int ret = -ENOENT;
	struct thread *th = current_thread();

	if (did >= MAXDEVS)
		return -EINVAL;

	if (th->usrdevs[did])
		ret = usrdev_poll(th->usrdevs[did], polld);
	return ret;
}

int devirq(unsigned did, unsigned id, unsigned irq)
{
	int ret = -ENOENT;
	struct thread *th = current_thread();

	if (did >= MAXDEVS)
		return -EINVAL;

	if (th->usrdevs[did])
		ret = usrdev_irq(th->usrdevs[did], id, irq);
	return ret;
}

int devwriospace(unsigned did, unsigned id, uint32_t port, uint64_t val)
{
	int ret = -ENOENT;
	struct thread *th = current_thread();

	if (did >= MAXDEVS)
		return -EINVAL;

	if (th->usrdevs[did])
		ret = usrdev_wriospace(th->usrdevs[did], id, port, val);
	return ret;
}

int devread(unsigned did, unsigned id, unsigned long iova, size_t sz, unsigned long va)
{
	int ret = -ENOENT;
	struct thread *th = current_thread();

	if (did >= MAXDEVS)
		return -EINVAL;

	if (th->usrdevs[did])
		ret = usrdev_read(th->usrdevs[did], id, iova, sz, va);

	return ret;
}

int devwrite(unsigned did, unsigned id, unsigned long va, size_t sz, unsigned long iova)
{
	int ret = -ENOENT;
	struct thread *th = current_thread();

	if (did >= MAXDEVS)
		return -EINVAL;

	if (th->usrdevs[did])
		ret = usrdev_write(th->usrdevs[did], id, va, sz, iova);

	return ret;
}

void devremove(unsigned did)
{
	struct usrdev *ud;
	struct thread *th = current_thread();

	assert(did < MAXDEVS);

	if (th->usrdevs[did]) {
		ud = th->usrdevs[did];
		usrdev_destroy(ud);
		th->usrdevs[did] = NULL;
	}
}

int devopen(uint64_t id)
{
	int dd;
	struct thread *th = current_thread();

	dd = bus_plug(&th->bus, id);
	return dd;
}

int devexport(unsigned dd, vaddr_t va, size_t sz, uint64_t *iova)
{
	struct thread *th = current_thread();

	return bus_export(&th->bus, dd, va, sz, iova);
}

int devunexport(unsigned dd, vaddr_t va)
{
	struct thread *th = current_thread();

	return bus_unexport(&th->bus, dd, va);
}

int devinfo(unsigned dd, struct sys_info_cfg *cfg)
{
	struct thread *th = current_thread();

	return bus_info(&th->bus, dd, cfg);
}

int devirqmap(unsigned dd, unsigned irq, unsigned sig)
{
	struct thread *th = current_thread();

	return bus_irqmap(&th->bus, dd, irq, sig);
}

int deveoi(unsigned dd, unsigned irq, unsigned sig)
{
	struct thread *th = current_thread();

	return bus_eoi(&th->bus, dd, irq);
}

int devin(unsigned dd, uint32_t port, uint64_t * valptr)
{
	struct thread *th = current_thread();

	return bus_in(&th->bus, dd, port, valptr);
}

int devout(unsigned dd, uint32_t port, uint64_t val)
{
	struct thread *th = current_thread();

	return bus_out(&th->bus, dd, port, val);
}

int devrdcfg(unsigned dd, uint32_t off, uint8_t sz, uint64_t *val)
{
	struct thread *th = current_thread();

	return bus_rdcfg(&th->bus, dd, off, sz, val);
}

int devwrcfg(unsigned dd, uint32_t off, uint8_t sz, uint64_t *val)
{
	struct thread *th = current_thread();

	return bus_wrcfg(&th->bus, dd, off, sz, val);
}

int deviomap(unsigned dd, vaddr_t va, paddr_t mmioaddr, pmap_prot_t prot)
{
	struct thread *th = current_thread();

	return bus_iomap(&th->bus, dd, va, mmioaddr, prot);
}

int deviounmap(unsigned dd, vaddr_t va)
{
	struct thread *th = current_thread();

	return bus_iounmap(&th->bus, dd, va);
}

void devclose(unsigned dd)
{
	struct thread *th = current_thread();

	bus_unplug(&th->bus, dd);
}

int irqeoi(unsigned irq)
{
	int doeoi;
	struct irqsig *irqsig;

	assert(irq < MAXIRQS);
	spinlock(&irqsigs_lock);
	LIST_FOREACH(irqsig, &irqsigs[irq], list) {
		if (irqsig->handler)
			continue;
		if (irqsig->eoi) {
			doeoi = 1;
			irqsig->eoi = 0;
		}
	}
	spinunlock(&irqsigs_lock);

	if (!doeoi)
		return -EINVAL;

	platform_irqon(irq);
	return 0;
}

void irqsignal(unsigned irq, int level)
{
	struct irqsig *irqsig;

	if (level) {
		/* Level IRQ: EOI will be sent to the APIC right
		 * after we return. Disable the IRQ line, it will
		 * be re-enabled by the interrupt handler. 
		 */
		platform_irqoff(irq);
	}

	assert(irq < MAXIRQS);
	spinlock(&irqsigs_lock);
	LIST_FOREACH(irqsig, &irqsigs[irq], list) {
		if (irqsig->handler) {
			irqsig->handler(irqsig->sig);
			continue;
		}
		if (irqsig->filter && !platform_irqfilter(irqsig->filter))
			continue;
		if (level)
			irqsig->eoi = 1;
		thraise(irqsig->th, irqsig->sig);
	}
	spinunlock(&irqsigs_lock);
}

void irqunregister(struct irqsig *irqsig)
{

	spinlock(&irqsigs_lock);
	LIST_REMOVE(irqsig, list);
	spinunlock(&irqsigs_lock);
}

int irqregister(struct irqsig *irqsig, unsigned irq, struct thread *th,
		unsigned sig, uint32_t filter)
{
	if (irq >= MAXIRQS)
		return -EINVAL;

	irqsig->th = th;
	irqsig->sig = sig;
	irqsig->filter = filter;
	irqsig->handler = NULL;

	spinlock(&irqsigs_lock);
	LIST_INSERT_HEAD(&irqsigs[irq], irqsig, list);
	spinunlock(&irqsigs_lock);
	platform_irqon(irq);
	return 0;
}

int irqfnregister(struct irqsig *irqsig, unsigned irq,
		  void (*handler) (void), unsigned opq)
{
	if (irq >= MAXIRQS)
		return -EINVAL;
	irqsig->th = NULL;
	irqsig->sig = opq;
	irqsig->filter = 0;
	irqsig->handler = handler;
	spinlock(&irqsigs_lock);
	LIST_INSERT_HEAD(&irqsigs[irq], irqsig, list);
	spinunlock(&irqsigs_lock);
	platform_irqon(irq);
	return 0;
}

static void do_timer(void)
{
	int updated;
	uint64_t cnt = timer_readcounter();
	struct timer *c, *t;

	spinlock(&timers_lock);
	updated = 0;
	LIST_FOREACH_SAFE(c, &timers, list, t) {
		if (c->time > cnt)
			break;
		if (c->handler != NULL) {
			c->handler(cnt);
		} else {
			if (c->sig != 0) {
				thraise(c->th,
					c->sig - 1);
			}
		}
		c->valid = 0;
		LIST_REMOVE(c, list);
		updated = 1;
	}
	if (updated) {
		/* Update hw timer. */
		c = LIST_FIRST(&timers);
		if (c == NULL)
			timer_disablealarm();
		else
			timer_setalarm(c->time);
	}
	spinunlock(&timers_lock);
}

static void timer_register(struct timer *t)
{
	uint64_t cnt = timer_readcounter();
	struct timer *c;

	if (!t->valid)
		return;

	spinlock(&timers_lock);
	c = LIST_FIRST(&timers);
	if (c == NULL) {
		LIST_INSERT_HEAD(&timers, t, list);
		goto out;
	}
	/* XXX: rollover */
	while (t->time > c->time) {
		if (LIST_NEXT(c, list) == NULL) {
			LIST_INSERT_AFTER(c, t, list);
			goto out;
		}
		c = LIST_NEXT(c, list);
	}
	LIST_INSERT_AFTER(c, t, list);
      out:
	/* Update hw timer. */
	c = LIST_FIRST(&timers);
	timer_setalarm(c->time);
	spinunlock(&timers_lock);
}

void timer_remove(struct timer *timer)
{
	struct timer *c;

	if (!timer->valid)
		return;
	spinlock(&timers_lock);
	LIST_REMOVE(timer, list);
	/* Update hw timer. */
	c = LIST_FIRST(&timers);
	if (c == NULL)
		timer_disablealarm();
	else
		timer_setalarm(c->time);
	spinunlock(&timers_lock);
	timer->valid = 0;
}

void thalrm(uint32_t diff)
{
	struct thread *th = current_thread();
	struct timer *t = &th->rtt_alarm;

	timer_remove(t);
	t->time = timer_readcounter() + diff;
	t->th = th;
	t->handler = NULL;
	t->valid = 1;
	timer_register(t);
}

void thvtalrm(uint32_t vtdiff)
{
	struct thread *th = current_thread();
	struct timer *t = &th->vtt_alarm;

	timer_remove(t);
	t->time = timer_readcounter() + vtdiff;
	t->th = th;
	t->handler = NULL;
	t->valid = 1;
	dprintf("Setting vtalm at %lx\n", (uint32_t)t->time);
	timer_register(t);
}

uint64_t thvtt(struct thread *th)
{
	assert(th == current_thread());
	return timer_readcounter() - th->vtt_rttbase + th->vtt_offset;
}

static vaddr_t elfld(void *elfimg)
{
	int i;
#define ELFOFF(_o) ((void *)((uintptr_t)elfimg + (_o)))

	char elfid[] = { 0x7f, 'E', 'L', 'F', };
	struct elfhdr *hdr = (struct elfhdr *) elfimg;
	struct elfph *ph = (struct elfph *) ELFOFF(hdr->phoff);

	assert(!memcmp(hdr->id, elfid, 4));
	for (i = 0; i < hdr->phs; i++, ph++) {
		if (ph->type != PHT_LOAD)
			continue;
		if (ph->fsize) {
			/*
			 * memcpy() to user and populate on fault.
			 */

			current_cpu()->usrpgfault = 1;
			__insn_barrier();
			if (_setjmp(current_cpu()->usrpgfaultctx)) {
				vmpopulate(current_cpu()->usrpgaddr,
					   PAGE_SIZE, PROT_USER_WRX);
				current_cpu()->usrpgaddr = 0;
			}
			memcpy((void *) ph->va, (void *) ELFOFF(ph->off),
			       ph->fsize);
			__insn_barrier();
			current_cpu()->usrpgfault = 0;
			current_cpu()->usrpgaddr = 0;

		}
		if (ph->msize - ph->fsize > 0) {
			/*
			 * memset() to user and populate on fault.
			 */

			current_cpu()->usrpgfault = 1;
			__insn_barrier();
			if (_setjmp(current_cpu()->usrpgfaultctx)) {
				vmpopulate(current_cpu()->usrpgaddr,
					   PAGE_SIZE, PROT_USER_WRX);
				current_cpu()->usrpgaddr = 0;
			}
			memset((void *) (ph->va + ph->fsize), 0,
			       ph->msize - ph->fsize);
			__insn_barrier();
			current_cpu()->usrpgfault = 0;
			current_cpu()->usrpgaddr = 0;
		}
	}

	return (vaddr_t) hdr->entry;
}

static void __initstart(void)
{
	vaddr_t entry;
	struct usrframe usrframe;
	extern void *_init_start;

	entry = elfld(_init_start);
	usrframe_setup(&usrframe, entry, 0);
	__insn_barrier();
	usrframe_enter(&usrframe);
	/* Not reached */
}

void kern_boot(void)
{
	struct thread *th;

	memset(irqsigs, 0, sizeof(irqsigs));
	devices_init();
	printf("Kernel loaded at va %08lx:%08lx\n", UKERNTEXTOFF,
	       UKERNEND);
	/* initialise threads */
	setup_structcache(&threads, thread);

	th = structs_alloc(&threads);
	th->pmap = pmap_current();
	th->stack_4k = NULL;
	th->userfl = 0;
	th->softintrs = 0;

	th->pid = getnewpid();
	th->parent = NULL;
	th->children_lock = 0;
	LIST_INIT(&th->active_children);
	LIST_INIT(&th->zombie_children);

	th->setuid = 0;
	th->ruid = 0;
	th->euid = 0;
	th->suid = 0;
	th->rgid = 0;
	th->egid = 0;
	th->sgid = 0;

	th->vtt_almdiff = 0;
	th->vtt_offset = 0;
	th->vtt_rttbase = 0;

	memset(&th->rtt_alarm, 0, sizeof(th->rtt_alarm));
	memset(&th->vtt_alarm, 0, sizeof(th->vtt_alarm));

	set_current_thread(th);
	current_cpu()->idle_thread = th;
	/* We are idle thread now. */

	cpu_wakeall();

	/* Create init */
	th = thnew(__initstart);
	th->parent = NULL;
	th->children_lock = 0;
	LIST_INIT(&th->active_children);
	LIST_INIT(&th->zombie_children);

	__kern_init = th;

	spinlock(&sched_lock);
	TAILQ_INSERT_TAIL(&running_threads, th, sched_list);
	spinunlock(&sched_lock);
	idle();
}

void kern_bootap(void)
{
	struct thread *th;

	printf("CPU %02d on.\n", cpu_number());

	/* initialise idle thread */
	th = structs_alloc(&threads);
	th->pmap = pmap_current();
	th->stack_4k = NULL;
	th->userfl = 0;
	th->softintrs = 0;

	th->pid = getnewpid();
	th->parent = NULL;
	th->children_lock = 0;
	LIST_INIT(&th->active_children);
	LIST_INIT(&th->zombie_children);

	th->setuid = 0;
	th->ruid = 0;
	th->euid = 0;
	th->suid = 0;
	th->rgid = 0;
	th->egid = 0;
	th->sgid = 0;

	th->vtt_almdiff = 0;
	th->vtt_offset = 0;
	th->vtt_rttbase = 0;

	memset(&th->rtt_alarm, 0, sizeof(th->rtt_alarm));
	memset(&th->vtt_alarm, 0, sizeof(th->vtt_alarm));

	set_current_thread(th);
	current_cpu()->idle_thread = th;

	idle();
}
