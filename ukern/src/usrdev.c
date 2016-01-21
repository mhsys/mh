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


#include <uk/types.h>
#include <uk/bus.h>
#include <uk/structs.h>
#include <uk/kern.h>
#include <uk/usrdev.h>
#include <machine/uk/cpu.h>
#include <uk/pgalloc.h>
#include <lib/lib.h>

#define MAXUSRDEVREMS 16
#define MAXUSRDEVAPERTS 16

//static struct slab usrdevs;
static struct slab usrioreqs;

enum usrior_op {
	IOR_OP_OUT,
};

struct usrioreq {
	unsigned id;
	enum usrior_op op;
	union {
		struct {
			/* OUT */
			uint64_t port;
			uint64_t val;
		};
	};
	uid_t uid;
	gid_t gid;

	 TAILQ_ENTRY(usrioreq) queue;
};

struct apert {
	vaddr_t procva;
	unsigned procid;
	vaddr_t devva;
	l1e_t l1e;
};

struct remth {
	int use:1;		/* Entry in use (proc) */
	int bsy:1;		/* Request in progress (device) */
	unsigned id;		/* Remote ID descriptor */
	struct thread *th;	/* Remote threads */
	unsigned irqmap[IRQMAPSZ];	/* Interrupt Remapping Table */
	struct apert apertbl[MAXUSRDEVAPERTS];
};

struct usrdev {
	lock_t lock;
	struct dev dev;		/* Registered device */
	struct thread *th;	/* Device thread */
	unsigned sig;		/* Request Signal */

	struct remth remths[MAXUSRDEVREMS];

	 TAILQ_HEAD(, usrioreq) ioreqs;
};

struct usrdev_procopq {
	uint32_t id;
};

static int _usrdev_open(void *devopq, uint64_t did)
{
	int i, ret;
	struct usrdev *ud = (struct usrdev *) devopq;

	/* Current thread: Process */

	spinlock(&ud->lock);
	for (i = 0; i < MAXUSRDEVREMS; i++) {
		if (!ud->remths[i].use && !ud->remths[i].bsy)
			break;
	}
	if (i >= MAXUSRDEVREMS) {
		ret = -ENFILE;
		goto out;
	}
	ud->remths[i].th = current_thread();
	ud->remths[i].use = 1;
	ud->remths[i].id = i;
	ret = i;
      out:
	spinunlock(&ud->lock);
	printf("open %" PRIx64 "! = %d", did, ret);
	return ret;
}

static int _usrdev_in(void *devopq, unsigned id, uint64_t port,
		      uint64_t * val)
{
	return -ENODEV;
}

static int _usrdev_out(void *devopq, unsigned id, uint64_t port,
		       uint64_t val)
{
	/* Current thread: Process */
	struct usrdev *ud = (struct usrdev *) devopq;
	struct thread *th = current_thread();
	struct usrioreq *ior;

	assert (id < MAXUSRDEVREMS);

	ior = structs_alloc(&usrioreqs);
	ior->id = id;
	ior->op = IOR_OP_OUT;

	ior->port = port;
	ior->val = val;

	printf("uid: %d, gid %d\n", th->euid, th->egid);
	ior->uid = th->euid;
	ior->gid = th->egid;

	spinlock(&ud->lock);
	if (ud->remths[id].bsy) {
		spinunlock(&ud->lock);
		structs_free(ior);
		return -EBUSY;
	}
	ud->remths[id].bsy = 1;
	TAILQ_INSERT_TAIL(&ud->ioreqs, ior, queue);
	thraise(ud->th, ud->sig);
	spinunlock(&ud->lock);
	return 0;
}

static int _usrdev_export(void *devopq, unsigned id, vaddr_t va,
			  unsigned iopfn)
{
	/* Current thread: Process */
	int ret;
	l1e_t expl1e;
	struct apert *apt;
	struct usrdev *ud = (struct usrdev *) devopq;

	assert(__isuaddr(va));
	assert(id < MAXUSRDEVREMS);

	if (iopfn >= MAXUSRDEVAPERTS)
		return -EINVAL;

	ret = pmap_uexport(NULL, va, &expl1e);
	if (ret)
		return ret;

	spinlock(&ud->lock);
	apt = &(ud->remths[id].apertbl[iopfn]);
	if (apt->procva) {
		ret = pmap_uexport_cancel(NULL, apt->procva);
		assert(!ret);
	}
	if (apt->devva) {
		pfn_t pfn;

		ret = pmap_uimport_swap(ud->th->pmap,
					apt->devva,
					apt->l1e,
					expl1e, &pfn);
		assert(!ret);

		if (pfn != PFN_INVALID) {
			printf("Freeing where imported %x\n", pfn);
			__freepage(pfn);
			ret++;
		}
		pmap_commit(ud->th->pmap);
	}
	pmap_commit(NULL);
	apt->procva = va;
	apt->procid = id;
	apt->l1e = expl1e;
	spinunlock(&ud->lock);
	return 0;
}

static int _usrdev_irqmap(void *devopq, unsigned id, unsigned irq,
			  unsigned sig)
{
	struct usrdev *ud = (struct usrdev *) devopq;

	/* Current thread: Process */
	assert(id < MAXUSRDEVREMS);
	if (irq >= IRQMAPSZ)
		return -EINVAL;

	spinlock(&ud->lock);
	ud->remths[id].irqmap[irq] = sig;
	spinunlock(&ud->lock);
	return 0;
}

static void _usrdev_close(void *devopq, unsigned id)
{
	int i, ret;
	l1e_t l1e;
	struct apert *apt;
	vaddr_t procva, devva;
	unsigned procid;
	struct usrdev *ud = (struct usrdev *) devopq;
	/* Current thread: Process or Device */

	printf("CLOSE!\n");
	spinlock(&ud->lock);
	for (i = 0; i < MAXUSRDEVAPERTS; i++) {
		apt = &(ud->remths[id].apertbl[i]);
		procid = apt->procid;
		procva = apt->procva;
		devva = apt->devva;
		if (procid != id)
			continue;
		if (devva) {
			assert(procva);
			printf("canceling dev %p\n", devva);
			ret = pmap_uimport_cancel(ud->th->pmap, devva);
			assert(!ret);
		}
		if (procva) {
			printf("canceling prog %p\n", procva);
			ret = pmap_uexport_cancel(ud->remths[id].th->pmap,
						  procva);
			assert(!ret);
		}
		apt->procid = 0;
		apt->procva = 0;
		apt->l1e = 0;
	}
	ud->remths[id].use = 0;
	spinunlock(&ud->lock);
	pmap_commit(ud->remths[id].th->pmap);
	pmap_commit(ud->th->pmap);
	printf("close(%d)!\n", id);
}

static struct devops usrdev_ops = {
	.open = _usrdev_open,
	.close = _usrdev_close,
	.in = _usrdev_in,
	.out = _usrdev_out,
	.export = _usrdev_export,
	.irqmap = _usrdev_irqmap,
};

struct usrdev *usrdev_creat(uint64_t id, unsigned sig, devmode_t mode)
{
	struct usrdev *ud;
	struct thread *th = current_thread();

	ud = heap_alloc(sizeof(*ud));
	ud->lock = 0;
	ud->th = th;
	ud->sig = sig;
	memset(&ud->remths, 0, sizeof(ud->remths));
	TAILQ_INIT(&ud->ioreqs);

	dev_init(&ud->dev, id, (void *) ud, &usrdev_ops, th->euid, th->egid, mode);
	if (dev_attach(&ud->dev)) {
		heap_free(ud);
		ud = NULL;
	}
	return ud;
}

int usrdev_poll(struct usrdev *ud, struct sys_poll_ior *poll)
{
	int id;
	struct usrioreq *ior;

retry:
	spinlock(&ud->lock);
	ior = TAILQ_FIRST(&ud->ioreqs);
	if (ior != NULL)
		TAILQ_REMOVE(&ud->ioreqs, ior, queue);
	spinunlock(&ud->lock);

	if (ior == NULL)
		return -1;

	switch (ior->op) {
	case IOR_OP_OUT:
		poll->op = SYS_POLL_OP_OUT;
		poll->port = ior->port;
		poll->val = ior->val;
		break;
	default:
		panic("Invalid IOR entry type %d!\n", ior->op);
		goto retry;
	}
	poll->gid = ior->gid;
	poll->uid = ior->uid;
	id = ior->id;
	structs_free(ior);
	printf("poll: returning %d\n", id);
	return id;
}

int usrdev_eio(struct usrdev *ud, unsigned id)
{
	if (id >= MAXUSRDEVREMS)
		return -EINVAL;
	spinlock(&ud->lock);
	printf("set busy of %d to zero\n", id);
	ud->remths[id].bsy = 0;
	spinunlock(&ud->lock);
	return 0;
}

int usrdev_irq(struct usrdev *ud, unsigned id, unsigned irq)
{
	unsigned sig;

	if ((id >= MAXUSRDEVREMS)
	    || (irq >= IRQMAPSZ))
		return -EINVAL;
	spinlock(&ud->lock);
	sig = ud->remths[id].irqmap[irq];
	printf("-> %d, %d\n", ud->remths[id].use, sig);
	if (ud->remths[id].use && sig) {
		printf("raising");
		thraise(ud->remths[id].th, sig);
	}
	spinunlock(&ud->lock);
	return 0;
}

int usrdev_import(struct usrdev *ud, unsigned id, unsigned iopfn,
		  unsigned va)
{
	int ret;
	pfn_t pfn;
	struct apert *apt;

	assert(__isuaddr(va));
	if (id >= MAXUSRDEVREMS)
		return -EINVAL;
	if (iopfn >= MAXUSRDEVAPERTS)
		return -EINVAL;
	spinlock(&ud->lock);
	apt = &(ud->remths[id].apertbl[iopfn]);
	if (apt->devva) {
		ret = pmap_uimport_cancel(NULL, apt->devva);
		assert(!ret);
	}
	apt->devva = va;
	ret = pmap_uimport(NULL, va, apt->l1e, &pfn);
	assert(!ret);
	pmap_commit(NULL);
	spinunlock(&ud->lock);

	if (pfn != PFN_INVALID) {
		printf("freeing where imported %x\n", pfn);
		__freepage(pfn);
		ret++;
	}
	return 0;
}

int usrdev_bootstrap(struct usrdev *ud, unsigned id, vaddr_t va, size_t sz,
		     mode_t mode)
{
	int ret;
	pfn_t pfn;
	void *ptr;

	if (id >= MAXUSRDEVREMS)
		return -EINVAL;
	if (sz >= PAGE_SIZE)
		return -EINVAL;

	ptr = heap_alloc(sz);
	ret = copy_from_user(ptr, va, sz);
	if (ret) {
		heap_free(ptr);
		return -EFAULT;
	}
	
	assert(__isuaddr(va) && __isuaddr(va + sz));
	spinlock(&ud->lock);
	/* Check for setuid/setgid. -1: Current */
	thbootstrap(ud->remths[id].th, -1, -1, ptr, sz);
	spinunlock(&ud->lock);
	return 0;
}

void usrdev_destroy(struct usrdev *ud)
{
	dev_detach(&ud->dev);
	heap_free(ud);
}

void usrdevs_init(void)
{
	setup_structcache(&usrioreqs, usrioreq);
	//	setup_structcache(&usrdevs, usrdev);
}
