/*	$NetBSD: rumpfiber.c,v 1.12 2015/02/15 00:54:32 justin Exp $	*/

/*
 * Copyright (c) 2007-2013 Antti Kantee.  All Rights Reserved.
 * Copyright (c) 2014 Justin Cormack.  All Rights Reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/* Based partly on code from Xen Minios with the following license */

/* 
 ****************************************************************************
 * (C) 2005 - Grzegorz Milos - Intel Research Cambridge
 ****************************************************************************
 *
 *        File: sched.c
 *      Author: Grzegorz Milos
 *     Changes: Robert Kaiser
 *              
 *        Date: Aug 2005
 * 
 * Environment: Xen Minimal OS
 * Description: simple scheduler for Mini-Os
 *
 * The scheduler is non-preemptive (cooperative), and schedules according 
 * to Round Robin algorithm.
 *
 ****************************************************************************
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
 * DEALINGS IN THE SOFTWARE.
 */

#include <sys/cdefs.h>

#if !defined(lint)
__RCSID("$NetBSD: rumpfiber.c,v 1.12 2015/02/15 00:54:32 justin Exp $");
#endif /* !lint */

#include <assert.h>
#include <errno.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/param.h>

#include <rump/rumpuser.h>

#include <mrg.h>
#include <mrg/blk.h>

struct thread {
    char *name;
    void *lwp;
    void *cookie;
    int64_t wakeup_time;
    TAILQ_ENTRY(thread) thread_list;
    lwt_t *lwt;
    int flags;
    int threrrno;
};

#define RUNNABLE_FLAG   0x00000001
#define THREAD_MUSTJOIN 0x00000002
#define THREAD_JOINED   0x00000004
#define THREAD_EXTSTACK 0x00000008
#define THREAD_TIMEDOUT 0x00000010

#define STACKSIZE 65536

//#include "rumpuser_int.h"
//#include "rumpfiber.h"

//#define RUMP_TRACE_DETAIL() printf("%s: called\n", __FUNCTION__)
//#define RUMP_TRACE() printf("%s: called\n", __FUNCTION__)

#define ET(_v_) return (_v_) ? rumpuser__errtrans(_v_) : 0;

#ifndef RUMP_TRACE_DETAIL
#define RUMP_TRACE_DETAIL()
#endif

#ifndef RUMP_TRACE
#define RUMP_TRACE()
#endif

struct rumpuser_hyperup rumpuser__hyp;

int rumpuser__random_init(void);
int  rumpuser__errtrans(int);

static void init_sched(void);
static void join_thread(struct thread *);
static void switch_threads(struct thread *prev, struct thread *next);
static void schedule(void);
static void wake(struct thread *thread);
static void block(void);


TAILQ_HEAD(thread_list, thread);

static struct thread_list exited_threads = TAILQ_HEAD_INITIALIZER(exited_threads);
static struct thread_list thread_list = TAILQ_HEAD_INITIALIZER(thread_list);

static void (*scheduler_hook)(void *, void *);

#define printk(...)				\
	{					\
		printf("MRGRUMP:");		\
		printf(__VA_ARGS__);		\
	}

static inline void
rumpkern_unsched(int *nlocks, void *interlock)
{

	rumpuser__hyp.hyp_backend_unschedule(0, nlocks, interlock);
}

static inline void
rumpkern_sched(int nlocks, void *interlock)
{

	rumpuser__hyp.hyp_backend_schedule(nlocks, interlock);
}

static inline struct thread *
get_current(void)
{

	return lwt_getprivate();
}

void *calloc(size_t n, size_t sz)
{
	void *ptr;

	ptr = malloc(n * sz); /* XXX: add calloc to libc, this is insecure */
	memset(ptr, 0, n * sz);
	return ptr;
}

/* TODO see notes in rumpuser_thread_create, have flags here */
struct thread *
create_thread(const char *name, void (*f)(void *), void *data)
{
	struct thread *thread = calloc(1, sizeof(struct thread));

	if (!thread) {
		return NULL;
	}

	thread->lwt = lwt_create_priv(f, data, STACKSIZE, (void *)thread);
	thread->name = strdup(name);

	thread->lwp = NULL;
	TAILQ_INSERT_TAIL(&thread_list, thread, thread_list);

	return thread;
}

void
set_thread(const char *name)
{
	struct thread *thread = calloc(1, sizeof(struct thread));

	assert (thread);

	thread->lwt = lwt_getcurrent();
	lwt_setprivate(thread);

	thread->lwp = NULL;
	TAILQ_INSERT_TAIL(&thread_list, thread, thread_list);
}

struct join_waiter {
    struct thread *jw_thread;
    struct thread *jw_wanted;
    TAILQ_ENTRY(join_waiter) jw_entries;
};
static TAILQ_HEAD(, join_waiter) joinwq = TAILQ_HEAD_INITIALIZER(joinwq);

void
exit_thread(void)
{
	struct thread *thread = get_current();
	struct join_waiter *jw_iter;

	/* if joinable, gate until we are allowed to exit */
	while (thread->flags & THREAD_MUSTJOIN) {
		thread->flags |= THREAD_JOINED;

		/* see if the joiner is already there */
		TAILQ_FOREACH(jw_iter, &joinwq, jw_entries) {
			if (jw_iter->jw_wanted == thread) {
				wake(jw_iter->jw_thread);
				break;
			}
		}
		block();
	}

	/* Remove from the thread list */
	TAILQ_REMOVE(&thread_list, thread, thread_list);
	/* Put onto exited list */
	TAILQ_INSERT_HEAD(&exited_threads, thread, thread_list);
	rumpkern_unsched(NULL, NULL);
	lwt_exit();
}

static void
join_thread(struct thread *joinable)
{
	struct join_waiter jw;
	struct thread *thread = get_current();

	assert(joinable->flags & THREAD_MUSTJOIN);

	/* wait for exiting thread to hit thread_exit() */
	while (! (joinable->flags & THREAD_JOINED)) {

		jw.jw_thread = thread;
		jw.jw_wanted = joinable;
		TAILQ_INSERT_TAIL(&joinwq, &jw, jw_entries);
		block();
		TAILQ_REMOVE(&joinwq, &jw, jw_entries);
	}

	/* signal exiting thread that we have seen it and it may now exit */
	assert(joinable->flags & THREAD_JOINED);
	joinable->flags &= ~THREAD_MUSTJOIN;

	wake(joinable);
}

void wake(struct thread *thread)
{
	int nlocks;

	rumpkern_unsched(&nlocks, NULL);
	lwt_wake(thread->lwt);
	rumpkern_sched(nlocks, NULL);
}

void block(void)
{
	int nlocks;

	rumpkern_unsched(&nlocks, NULL);
	lwt_sleep();
	rumpkern_sched(nlocks, NULL);
}

static void
init_sched(void)
{
	struct thread *thread = calloc(1, sizeof(struct thread));

	if (!thread) {
		abort();
	}

	thread->name = strdup("init");
	thread->flags = 0;
	thread->wakeup_time = -1;
	thread->lwt = lwt_getcurrent();
	thread->lwp = NULL;
	TAILQ_INSERT_TAIL(&thread_list, thread, thread_list);
	lwt_setprivate(thread);
}

void
set_sched_hook(void (*f)(void *, void *))
{

	scheduler_hook = f;
}

/* rump functions below */

int
rumpuser_init(int version, const struct rumpuser_hyperup *hyp)
{
	int rv;

	if (version != RUMPUSER_VERSION) {
		printk("rumpuser version mismatch\n");
		abort();
	}

	RUMP_TRACE_DETAIL();
	rv = rumpuser__random_init();
	if (rv != 0) {
		ET(rv);
	}

	RUMP_TRACE_DETAIL();
	rumpuser__hyp = *hyp;

	init_sched();

	return 0;
}

int
rumpuser_clock_gettime(int enum_rumpclock, int64_t *sec, long *nsec)
{
	*sec = 42;
	*nsec = 42;
	RUMP_TRACE_DETAIL();
	ET(0);
}

int
rumpuser_clock_sleep(int enum_rumpclock, int64_t sec, long nsec)
{
	enum rumpclock rclk = enum_rumpclock;
	uint64_t msec;
	int nlocks;
	RUMP_TRACE_DETAIL();
	rumpkern_unsched(&nlocks, NULL);
	switch (rclk) {
	case RUMPUSER_CLOCK_RELWALL:
		msec = sec * 1000 + nsec / (1000*1000UL);
		lwt_yield();
		break;
	case RUMPUSER_CLOCK_ABSMONO:
		msec = sec * 1000 + nsec / (1000*1000UL);
		lwt_yield();
		break;
	}
	rumpkern_sched(nlocks, NULL);

	return 0;
}

int
rumpuser_getparam(const char *name, void *buf, size_t blen)
{
	int rv;
	const char *ncpu = "1";

	RUMP_TRACE_DETAIL();
	if (strcmp(name, RUMPUSER_PARAM_NCPU) == 0) {
		strncpy(buf, ncpu, blen);
		rv = 0;
	} else if (strcmp(name, RUMPUSER_PARAM_HOSTNAME) == 0) {
		char tmp[MAXHOSTNAMELEN];

		snprintf(buf, blen, "rump-%05d", (int)sys_getpid());
		rv = 0;
	} else if (strcmp(name, "RUMP_VERBOSE") == 0) {
		strncpy(buf, "1", 2);
		rv = 0;
	} else if (*name == '_') {
		rv = EINVAL;
	} else {
	        rv = EINVAL;
	}

	ET(rv);
}

void
rumpuser_putchar(int c)
{

	printf("%c", c);
}

__dead void
rumpuser_exit(int rv)
{
	RUMP_TRACE_DETAIL();
	sys_wait(); /* XXX: */
	if (rv == RUMPUSER_PANIC)
		abort();
	else
		exit(rv);
}

void
rumpuser_seterrno(int error)
{
	printf("setting errno to %d\n", error);
	RUMP_TRACE_DETAIL();
	errno = error;
}

/*
 * This is meant for safe debugging prints from the kernel.
 */
void
rumpuser_dprintf(const char *format, ...)
{
  	va_list ap;
  	RUMP_TRACE_DETAIL();
	va_start(ap, format);
	vprintf(format, ap);
	va_end(ap);
}

int
rumpuser_kill(int64_t pid, int rumpsig)
{
	RUMP_TRACE_DETAIL();
	printf("\"Killing\"");
	sys_wait();
}

/* thread functions */

TAILQ_HEAD(waithead, waiter);
struct waiter {
	struct thread *who;
	TAILQ_ENTRY(waiter) entries;
	int onlist;
};

static int
wait(struct waithead *wh, uint64_t msec)
{
	struct waiter w;
	w.who = get_current();
	TAILQ_INSERT_TAIL(wh, &w, entries);
	w.onlist = 1;
	lwt_sleep();
	return 0;
}

static void
wakeup_one(struct waithead *wh)
{
	struct waiter *w;

	if ((w = TAILQ_FIRST(wh)) != NULL) {
		TAILQ_REMOVE(wh, w, entries);
		w->onlist = 0;
		wake(w->who);
	}
}

static void
wakeup_all(struct waithead *wh)
{
	struct waiter *w;

	while ((w = TAILQ_FIRST(wh)) != NULL) {
		TAILQ_REMOVE(wh, w, entries);
		w->onlist = 0;
		wake(w->who);
	}
}

int
rumpuser_thread_create(void *(*f)(void *), void *arg, const char *thrname,
	int joinable, int pri, int cpuidx, void **tptr)
{
	struct thread *thr;

	RUMP_TRACE_DETAIL();

	thr = create_thread(thrname, (void (*)(void *))f, arg);

	if (!thr)
		return EINVAL;

	/*
	 * XXX: should be supplied as a flag to create_thread() so as to
	 * _ensure_ it's set before the thread runs (and could exit).
	 * now we're trusting unclear semantics of create_thread()
	 */
	if (thr && joinable)
		thr->flags |= THREAD_MUSTJOIN;

	*tptr = thr;
	return 0;
}

void
rumpuser_thread_exit(void)
{
	RUMP_TRACE_DETAIL();
	exit_thread();
	/* unreached */
	while(1);
}

int
rumpuser_thread_join(void *p)
{

	RUMP_TRACE_DETAIL();
	join_thread(p);
	return 0;
}

struct rumpuser_mtx {
	struct waithead waiters;
	int v;
	int flags;
	struct thread *o;
};

void
rumpuser_mutex_init(struct rumpuser_mtx **mtxp, int flags)
{
	struct rumpuser_mtx *mtx;

	RUMP_TRACE_DETAIL();
	mtx = malloc(sizeof(*mtx));
	memset(mtx, 0, sizeof(*mtx));
	mtx->flags = flags;
	TAILQ_INIT(&mtx->waiters);
	*mtxp = mtx;
}

void
rumpuser_mutex_enter(struct rumpuser_mtx *mtx)
{
	int nlocks;

	RUMP_TRACE_DETAIL();
	if (rumpuser_mutex_tryenter(mtx) != 0) {
		while (rumpuser_mutex_tryenter(mtx) != 0) {
			rumpkern_unsched(&nlocks, NULL);
			wait(&mtx->waiters, 0);
			rumpkern_sched(nlocks, NULL);
		}
	}
}

void
rumpuser_mutex_enter_nowrap(struct rumpuser_mtx *mtx)
{
	int rv;
	RUMP_TRACE_DETAIL();
	rv = rumpuser_mutex_tryenter(mtx);
	/* one VCPU supported, no preemption => must succeed */
	if (rv != 0) {
		printk("no voi ei\n");
	}
}

int
rumpuser_mutex_tryenter(struct rumpuser_mtx *mtx)
{
	struct thread *t = get_current();

	RUMP_TRACE_DETAIL();
	if (mtx->v && mtx->o != t)
		return EBUSY;

	mtx->v++;
	mtx->o = t;

	return 0;
}

void
rumpuser_mutex_exit(struct rumpuser_mtx *mtx)
{

	RUMP_TRACE_DETAIL();
	assert(mtx->v > 0);
	if (--mtx->v == 0) {
		mtx->o = NULL;
		wakeup_one(&mtx->waiters);
	}
}

void
rumpuser_mutex_destroy(struct rumpuser_mtx *mtx)
{

	RUMP_TRACE_DETAIL();
	assert(TAILQ_EMPTY(&mtx->waiters) && mtx->o == NULL);
	free(mtx);
}

void
rumpuser_mutex_owner(struct rumpuser_mtx *mtx, struct lwp **lp)
{

	RUMP_TRACE_DETAIL();
	*lp = mtx->o == NULL ? NULL : mtx->o->lwp;
}

struct rumpuser_rw {
	struct waithead rwait;
	struct waithead wwait;
	int v;
	struct lwp *o;
};

void
rumpuser_rw_init(struct rumpuser_rw **rwp)
{
	struct rumpuser_rw *rw;

	RUMP_TRACE_DETAIL();
	rw = malloc(sizeof(*rw));
	memset(rw, 0, sizeof(*rw));
	TAILQ_INIT(&rw->rwait);
	TAILQ_INIT(&rw->wwait);

	*rwp = rw;
}

void
rumpuser_rw_enter(int enum_rumprwlock, struct rumpuser_rw *rw)
{
	enum rumprwlock lk = enum_rumprwlock;
	struct waithead *w = NULL;
	int nlocks;

	RUMP_TRACE_DETAIL();
	switch (lk) {
	case RUMPUSER_RW_WRITER:
		w = &rw->wwait;
		break;
	case RUMPUSER_RW_READER:
		w = &rw->rwait;
		break;
	}

	if (rumpuser_rw_tryenter(enum_rumprwlock, rw) != 0) {
		rumpkern_unsched(&nlocks, NULL);
		while (rumpuser_rw_tryenter(enum_rumprwlock, rw) != 0)
			wait(w, 0);
		rumpkern_sched(nlocks, NULL);
	}
}

int
rumpuser_rw_tryenter(int enum_rumprwlock, struct rumpuser_rw *rw)
{
	enum rumprwlock lk = enum_rumprwlock;
	int rv;

	RUMP_TRACE_DETAIL();
	switch (lk) {
	case RUMPUSER_RW_WRITER:
		if (rw->o == NULL) {
			rw->o = rumpuser_curlwp();
			rv = 0;
		} else {
			rv = EBUSY;
		}
		break;
	case RUMPUSER_RW_READER:
		if (rw->o == NULL && TAILQ_EMPTY(&rw->wwait)) {
			rw->v++;
			rv = 0;
		} else {
			rv = EBUSY;
		}
		break;
	default:
		rv = EINVAL;
	}

	return rv;
}

void
rumpuser_rw_exit(struct rumpuser_rw *rw)
{

	if (rw->o) {
		rw->o = NULL;
	} else {
		rw->v--;
	}

	RUMP_TRACE_DETAIL();
	/* standard procedure, don't let readers starve out writers */
	if (!TAILQ_EMPTY(&rw->wwait)) {
		if (rw->o == NULL)
			wakeup_one(&rw->wwait);
	} else if (!TAILQ_EMPTY(&rw->rwait) && rw->o == NULL) {
		wakeup_all(&rw->rwait);
	}
}

void
rumpuser_rw_destroy(struct rumpuser_rw *rw)
{

	RUMP_TRACE_DETAIL();
	free(rw);
}

void
rumpuser_rw_held(int enum_rumprwlock, struct rumpuser_rw *rw, int *rvp)
{
	enum rumprwlock lk = enum_rumprwlock;

	RUMP_TRACE_DETAIL();
	switch (lk) {
	case RUMPUSER_RW_WRITER:
		*rvp = rw->o == rumpuser_curlwp();
		break;
	case RUMPUSER_RW_READER:
		*rvp = rw->v > 0;
		break;
	}
}

void
rumpuser_rw_downgrade(struct rumpuser_rw *rw)
{

	RUMP_TRACE_DETAIL();
	assert(rw->o == rumpuser_curlwp());
	rw->v = -1;
}

int
rumpuser_rw_tryupgrade(struct rumpuser_rw *rw)
{

	RUMP_TRACE_DETAIL();
	if (rw->v == -1) {
		rw->v = 1;
		rw->o = rumpuser_curlwp();
		return 0;
	}

	return EBUSY;
}

struct rumpuser_cv {
	struct waithead waiters;
	int nwaiters;
};

void
rumpuser_cv_init(struct rumpuser_cv **cvp)
{
	struct rumpuser_cv *cv;

	RUMP_TRACE_DETAIL();
	cv = malloc(sizeof(*cv));
	memset(cv, 0, sizeof(*cv));
	TAILQ_INIT(&cv->waiters);
	*cvp = cv;
}

void
rumpuser_cv_destroy(struct rumpuser_cv *cv)
{

	RUMP_TRACE_DETAIL();
	assert(cv->nwaiters == 0);
	free(cv);
}

static void
cv_unsched(struct rumpuser_mtx *mtx, int *nlocks)
{

	rumpkern_unsched(nlocks, mtx);
	rumpuser_mutex_exit(mtx);
}

static void
cv_resched(struct rumpuser_mtx *mtx, int nlocks)
{

	/* see rumpuser(3) */
	if ((mtx->flags & (RUMPUSER_MTX_KMUTEX | RUMPUSER_MTX_SPIN)) ==
	    (RUMPUSER_MTX_KMUTEX | RUMPUSER_MTX_SPIN)) {
		rumpkern_sched(nlocks, mtx);
		rumpuser_mutex_enter_nowrap(mtx);
	} else {
		rumpuser_mutex_enter_nowrap(mtx);
		rumpkern_sched(nlocks, mtx);
	}
}

void
rumpuser_cv_wait(struct rumpuser_cv *cv, struct rumpuser_mtx *mtx)
{
	int nlocks;

	RUMP_TRACE_DETAIL();
	cv->nwaiters++;
	cv_unsched(mtx, &nlocks);
	wait(&cv->waiters, 0);
	cv_resched(mtx, nlocks);
	cv->nwaiters--;
}

void
rumpuser_cv_wait_nowrap(struct rumpuser_cv *cv, struct rumpuser_mtx *mtx)
{

	RUMP_TRACE_DETAIL();
	cv->nwaiters++;
	rumpuser_mutex_exit(mtx);
	wait(&cv->waiters, 0);
	rumpuser_mutex_enter_nowrap(mtx);
	cv->nwaiters--;
}

int
rumpuser_cv_timedwait(struct rumpuser_cv *cv, struct rumpuser_mtx *mtx,
	int64_t sec, int64_t nsec)
{
	int nlocks;
	int rv;

	RUMP_TRACE_DETAIL();
	cv->nwaiters++;
	cv_unsched(mtx, &nlocks);
	rv = wait(&cv->waiters, sec * 1000 + nsec / (1000*1000));
	cv_resched(mtx, nlocks);
	cv->nwaiters--;

	return rv;
}

void
rumpuser_cv_signal(struct rumpuser_cv *cv)
{

	RUMP_TRACE_DETAIL();
	wakeup_one(&cv->waiters);
}

void
rumpuser_cv_broadcast(struct rumpuser_cv *cv)
{

	RUMP_TRACE_DETAIL();
	wakeup_all(&cv->waiters);
}

void
rumpuser_cv_has_waiters(struct rumpuser_cv *cv, int *rvp)
{

	RUMP_TRACE_DETAIL();
	*rvp = cv->nwaiters != 0;
}

/*
 * curlwp
 */

void
rumpuser_curlwpop(int enum_rumplwpop, struct lwp *l)
{
	struct thread *thread;
	enum rumplwpop op = enum_rumplwpop;

	RUMP_TRACE_DETAIL();
	switch (op) {
	case RUMPUSER_LWP_CREATE:
	case RUMPUSER_LWP_DESTROY:
		break;
	case RUMPUSER_LWP_SET:
		thread = get_current();
		thread->lwp = l;
		break;
	case RUMPUSER_LWP_CLEAR:
		thread = get_current();
		assert(thread->lwp == l);
		thread->lwp = NULL;
		break;
	}
}

struct lwp *
rumpuser_curlwp(void)
{

	RUMP_TRACE_DETAIL();
	return get_current() ? get_current()->lwp : NULL;
}

int
rumpuser_daemonize_begin(void)
{
}

int
rumpuser_daemonize_done(int error)
{
}


int
rumpuser_malloc(size_t howmuch, int alignment, void **memp)
{
	uint8_t *mem = NULL;
	int rv = 0;

	RUMP_TRACE();
	if (alignment == 0)
		alignment = sizeof(void *);


	mem = (uint8_t *)malloc(howmuch + alignment - 1);
	memset(mem, 0, howmuch + alignment - 1);
	if (mem != NULL) {
		mem = (void *)(((uintptr_t)mem + alignment - 1) & ~(alignment - 1));
	} else {
		ET(-ENOMEM);
	}
	//	printf("allocating %d (%d -> %d) -> %p\n", (int)howmuch, (int)alignment, (int)howmuch + (int)alignment - 1, mem);
	*memp = (void *)mem;
	ET(rv);
}

/*ARGSUSED1*/
void
rumpuser_free(void *ptr, size_t size)
{

	RUMP_TRACE();
  //	free(ptr);
}

int
rumpuser_anonmmap(void *prefaddr, size_t size, int alignbit,
	int exec, void **memp)
{
	vaddr_t mem;
	uint8_t type = VFNT_RWDATA;
	int rv;


	RUMP_TRACE();
	if (exec)
	    type = VFNT_WREXEC;
	mem = vmap_alloc(size, type);
	if (mem == 0) {
		rv = errno;
	} else {
		*memp = (void *)mem;
		rv = 0;
	}

	ET(rv);
}

void
rumpuser_unmap(void *addr, size_t len)
{

	RUMP_TRACE();
	vmap_free((vaddr_t)addr, len);
}

/*
 * no dynamic linking supported
 */
void
rumpuser_dl_bootstrap(rump_modinit_fn domodinit,
	rump_symload_fn symload, rump_compload_fn compload)
{

	RUMP_TRACE();
	return;
}

int
rumpuser__random_init(void)
{

	RUMP_TRACE();
	return 0;
}

int
rumpuser_getrandom(void *buf, size_t b, int f, size_t *r)
{
	RUMP_TRACE();
  	return 42;
}

int
rumpuser_getfileinfo(const char *path, uint64_t *sizep, int *ftp)
{
	struct blkdisk *blk;
	
	RUMP_TRACE();
	blk = blk_open(squoze((char *)path));
	if (blk == NULL)
		return -ENOENT;

	if (sizep)
		*sizep = blk->info.blkno * blk->info.blksz;
	if (ftp)
		*ftp = RUMPUSER_FT_BLK;

	blk_close(blk);
	return 0;
}

/* Yeah... */
static int next=0;
static struct blkdisk *fildes[256];

int
rumpuser_open(const char *path, int ruflags, int *fdp)
{
	struct blkdisk *blk;

	blk = blk_open(squoze((char *)path));
	if (blk == NULL)
		return -ENOENT;

	if (next >= 256) {
		blk_close(blk);
		return -1;
	}

	fildes[next++] = blk;

	if (fdp)
		*fdp = next-1;
	return 0;
}

int
rumpuser_close(int fd)
{
	RUMP_TRACE();
	blk_close(fildes[fd]);
	fildes[fd] = NULL;
	return 0;
}

int
rumpuser_iovread(int fd, struct rumpuser_iovec *ruiov, size_t iovlen,
	int64_t roff, size_t *retp)
{
	RUMP_TRACE();
	ET(-ENOSYS);
}


int
rumpuser_iovwrite(int fd, const struct rumpuser_iovec *ruiov, size_t iovlen,
	int64_t roff, size_t *retp)
{
	RUMP_TRACE();
	ET(-ENOSYS);
}

int
rumpuser_syncfd(int fd, int flags, uint64_t start, uint64_t len)
{
	RUMP_TRACE();
	ET(-ENOSYS);
}

struct rumpmrg_bio {
	int evt;
	int ret;
	rump_biodone_fn biodone;
	void *bioarg;
};

unsigned char *datap;

void _rumpmrg_bio_ast(void *arg)
{
	int dummy;
	struct rumpmrg_bio *biop = (struct rumpmrg_bio *)arg;

	set_thread("bio_ast");

	rumpuser__hyp.hyp_schedule();
	biop->biodone(biop->bioarg, biop->ret, biop->ret);
	rumpuser__hyp.hyp_unschedule();
	evtfree(biop->evt);
	free(biop);
}

void
rumpuser_bio(int fd, int op, void *data, size_t dlen, int64_t doff,
	rump_biodone_fn biodone, void *bioarg)
{
	struct rumpmrg_bio bio;
	uint64_t blkid;
	size_t blkno;
	struct rumpmrg_bio *biop;

	memset(data, 0x55, dlen);

	RUMP_TRACE();
	blkid = doff / 512;
	assert (doff % 512 == 0);
	blkno = (dlen + 511) / 512;

	biop = malloc(sizeof(*biop));
	biop->evt = evtalloc();
	biop->biodone = biodone;
	biop->bioarg = bioarg;

	evtast(biop->evt, _rumpmrg_bio_ast, biop);
	if (op & RUMPUSER_BIO_READ) {
		blk_read(fildes[fd], data, dlen, blkid, blkno, biop->evt, &biop->ret);
		datap = data;
	} else {
		printf("<Write disabled>\n");
		//blk_write(fildes[fd], blkid, blkno, data, dlen, biop->evt, &biop->ret);
	}

}

/*
 * rumpuser_component
 */

/*
 * Copyright (c) 2013 Antti Kantee.  All Rights Reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */


/*
 * These interfaces affect the shlib major/minor; they can be called from
 * any program when applicable.  The rest of the interfaces provided
 * by rumpuser are part of the rump kernel/hypervisor contract and
 * are versioned by RUMPUSER_VERSION.
 */

#include "rumpuser_component.h"
extern struct rumpuser_hyperup rumpuser__hyp;
//#include "rumpuser_int.h"

void *
rumpuser_component_unschedule(void)
{
	int nlocks;

	rumpkern_unsched(&nlocks, NULL);
	return (void *)(intptr_t)nlocks;
}

void
rumpuser_component_schedule(void *cookie)
{
	int nlocks = (int)(intptr_t)cookie;

	rumpkern_sched(nlocks, NULL);
}

void
rumpuser_component_kthread(void)
{

	rumpuser__hyp.hyp_schedule();
	rumpuser__hyp.hyp_lwproc_newlwp(0);
	rumpuser__hyp.hyp_unschedule();
}

struct lwp *
rumpuser_component_curlwp(void)
{
	struct lwp *l;

	rumpuser__hyp.hyp_schedule();
	l = rumpuser__hyp.hyp_lwproc_curlwp();
	rumpuser__hyp.hyp_unschedule();

	return l;
}

void
rumpuser_component_switchlwp(struct lwp *l)
{

	rumpuser__hyp.hyp_schedule();
	rumpuser__hyp.hyp_lwproc_switch(l);
	rumpuser__hyp.hyp_unschedule();
}

void
rumpuser_component_kthread_release(void)
{

	rumpuser__hyp.hyp_schedule();
	rumpuser__hyp.hyp_lwproc_release();
	rumpuser__hyp.hyp_unschedule();
}

int
rumpuser_component_errtrans(int hosterr)
{

	return rumpuser__errtrans(hosterr);
}

