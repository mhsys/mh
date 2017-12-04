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
#include <uk/logio.h>
#include <uk/string.h>
#include <uk/bus.h>
#include <uk/structs.h>
#include <uk/kern.h>
#include <uk/usrdev.h>
#include <uk/cpu.h>
#include <uk/pgalloc.h>
#include <uk/heap.h>

/* User devices and OP serialization:
 *
 * Certain rules of serialization of the events must apply:
 *
 * Events pertaining the same 'id' must be polled in the order of
 * insertion.  A special case is clone, that has two ID, the clone and
 * the original 'id': In this case this operation belong to both
 * 'id's, so they must be polled in order.
 * 
 * In order to make things simpler, we apply a FIFO ior polling to
 * make thing both easy and correct.
 */


#define MAXUSRDEVREMS 256
#define MAXUSRDEVAPERTS 16
#define IOSPACESZ 32

static struct slab usrioreqs;

enum usrior_op {
	IOR_OP_OPEN,
	IOR_OP_CLONE,
	IOR_OP_OUT,
	IOR_OP_CLOSE,
};

struct usrioreq {
	unsigned id;
	enum usrior_op op;
	union {
		/* Nothing for OPEN */
		struct {
			/* CLONE */
			unsigned clone_id;
		};
		struct {
			/* OUT */
			uint8_t size;
			uint16_t port;
			uint64_t val;
		};
		/* Nothing for CLOSE */
	};
	uid_t uid;
	gid_t gid;

	TAILQ_ENTRY(usrioreq) queue;
};

struct apert {
	vaddr_t va;
	size_t sz;
};

struct remth {
	int use:1;			/* Entry in use (proc) */
	unsigned id;	       		/* Remote ID descriptor */
	struct thread *th;		/* Remote threads */
	uint8_t iospace[IOSPACESZ];	/* I/O Space */
	unsigned irqmap[IRQMAPSZ];	/* Interrupt Remapping Table */
  	struct apert apertbl[MAXUSRDEVAPERTS];
};

struct usrdev_cfg {
	uint32_t vid;
	uint32_t did;
	uint8_t nirqs;
	uint64_t usercfg[SYS_DEVCONFIG_MAXUSERCFG];
};

struct usrdev {
	lock_t lock;
	struct dev dev;		/* Registered device */
	struct thread *th;	/* Device thread */
	struct usrdev_cfg cfg;  /* Device Configuration Space */
	unsigned sig;		/* Request Signal */
	unsigned detaching;     /* Shutting down, stop operations. */

	struct remth remths[MAXUSRDEVREMS];

	TAILQ_HEAD(,usrioreq) ioreqs;
};

struct usrdev_procopq {
	uint32_t id;
};

static int _usrdev_open(void *devopq, uint64_t did)
{
	int i, ret;
	struct thread *th = current_thread();
	struct usrioreq *ior;
	int unused_ior = 1;
	struct usrdev *ud = (struct usrdev *) devopq;

	/* Current thread: Process */
	ior = structs_alloc(&usrioreqs);
	memset(ior, 0, sizeof(*ior));
	ior->op = IOR_OP_OPEN;
	ior->gid = th->egid;
	ior->uid = th->euid;

	spinlock(&ud->lock);
	for (i = 0; i < MAXUSRDEVREMS; i++) {
		if (!ud->remths[i].use)
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

	if (ud->detaching) {
		unused_ior = 1;
		ret = -ENODEV;
		goto out;
	}

	ior->id = i;
	TAILQ_INSERT_TAIL(&ud->ioreqs, ior, queue);
	thraise(ud->th, ud->sig);
	unused_ior = 0;

      out:
	spinunlock(&ud->lock);
	if (unused_ior)
		structs_free(ior);
	return ret;
}

static int _usrdev_clone(void *devopq, unsigned id, struct thread *nth)
{
	int i, ret;
	struct thread *th = current_thread();
	struct usrioreq *ior;
	int unused_ior = 1;
	struct usrdev *ud = (struct usrdev *) devopq;

	/* Current thread: Process */
	ior = structs_alloc(&usrioreqs);
	memset(ior, 0, sizeof(*ior));
	ior->op = IOR_OP_CLONE;
	ior->id = id;
	ior->gid = th->egid;
	ior->uid = th->euid;

	spinlock(&ud->lock);
	for (i = 0; i < MAXUSRDEVREMS; i++) {
		if (!ud->remths[i].use)
			break;
	}
	if (i >= MAXUSRDEVREMS) {
		ret = -ENFILE;
		goto out;
	}
	ud->remths[i].th = nth;
	ud->remths[i].use = 1;
	ud->remths[i].id = i;
	memcpy(ud->remths[i].iospace,
	       ud->remths[id].iospace,
	       sizeof(ud->remths[i].iospace));
	memcpy(ud->remths[i].irqmap,
	       ud->remths[id].irqmap,
	       sizeof(ud->remths[i].irqmap));
	/* do not copy ud->remths[i].apertbl */
	ret = i;

	if (ud->detaching) {
		unused_ior = 1;
		ret = -ENODEV;
		goto out;
	}

	ior->clone_id = i;
	TAILQ_INSERT_TAIL(&ud->ioreqs, ior, queue);
	thraise(ud->th, ud->sig);
	unused_ior = 0;
 out:
	spinunlock(&ud->lock);
	if (unused_ior)
		structs_free(ior);
	return ret;
}

static int _usrdev_in(void *devopq, unsigned id, uint32_t port,
		      uint64_t * val)
{
	struct usrdev *ud = (struct usrdev *) devopq;
	uint8_t iosize = 1 << (port & IOPORT_SIZEMASK);
	uint16_t ioport = port >> IOPORT_SIZESHIFT;
	uint64_t ioval;
	uint8_t *ptr;
	int i, start, end;

	start = MIN(ioport, IOSPACESZ);
	end = MIN(start + iosize, IOSPACESZ);

	ioval = -1;
	ptr = (uint8_t *)&ioval;
	spinlock(&ud->lock);
	for (i = start; i < end; i++) {
		ptr[i - start] = ud->remths[id].iospace[i];
	}
	spinunlock(&ud->lock);
	*val = ioval;
	return 0;
}

static int _usrdev_out(void *devopq, unsigned id, uint32_t port,
		       uint64_t val)
{
	/* Current thread: Process */
	struct usrdev *ud = (struct usrdev *) devopq;
	struct thread *th = current_thread();
	uint8_t iosize = port & IOPORT_SIZEMASK;
	uint16_t ioport = port >> IOPORT_SIZESHIFT;
	struct usrioreq *ior;

	assert (id < MAXUSRDEVREMS);

	ior = structs_alloc(&usrioreqs);
	ior->id = id;
	ior->op = IOR_OP_OUT;

	ior->size = iosize;
	ior->port = ioport;
	ior->val = val;

	ior->uid = th->euid;
	ior->gid = th->egid;

	spinlock(&ud->lock);
	if (ud->detaching) {
		spinunlock(&ud->lock);
		structs_free(ior);
		/* Only temporarily a lie. Device will be destroyed
		 * soon. */
		return -ENODEV;
	}

	TAILQ_INSERT_TAIL(&ud->ioreqs, ior, queue);
	thraise(ud->th, ud->sig);
	spinunlock(&ud->lock);
	return 0;
}

static int _usrdev_export(void *devopq, unsigned id, vaddr_t va,
			  size_t sz, uint64_t *iova)
{
	/* Current thread: Process */
	int i;
	struct apert *apt;
	struct usrdev *ud = (struct usrdev *) devopq;

	assert(id < MAXUSRDEVREMS);

	if (sz == 0)
		return -EINVAL;

	spinlock(&ud->lock);
	apt = ud->remths[id].apertbl;

	for (i = 0; i < MAXUSRDEVAPERTS; i++)
		if (apt[i].sz == 0)
			break;

	if (i >= MAXUSRDEVAPERTS) {
		spinunlock(&ud->lock);
		return -EBUSY;
	}

	apt[i].va = va;
	apt[i].sz = sz;
	spinunlock(&ud->lock);

	*iova = va;
	return 0;
}

static int _usrdev_unexport(void *devopq, unsigned id, vaddr_t va)
{
	/* Current thread: Process */
	int i;
	struct apert *apt;
	struct usrdev *ud = (struct usrdev *) devopq;

	spinlock(&ud->lock);
	apt = ud->remths[id].apertbl;

	for (i = 0; i < MAXUSRDEVAPERTS; i++)
		if (apt[i].va == va)
			break;

	if (i >= MAXUSRDEVAPERTS) {
		spinunlock(&ud->lock);
		return -ENOENT;
	}

	apt[i].va = 0;
	apt[i].sz = 0;
	spinunlock(&ud->lock);
	return 0;
}

static int _usrdev_info(void *devopq, unsigned id, struct sys_info_cfg *cfg)
{
	struct usrdev *ud = (struct usrdev *) devopq;

	/* Current thread: Process */
	assert(id < MAXUSRDEVREMS);

	spinlock(&ud->lock);
	cfg->niopfns = IOSPACESZ;
	cfg->vendorid = ud->cfg.vid;
	cfg->deviceids[0] = ud->cfg.did;
	cfg->segs[0].len = ud->cfg.nirqs;
	spinunlock(&ud->lock);
	return 0;
}

static int _usrdev_rdcfg(void *devopq, unsigned id, uint32_t off, uint8_t size, uint64_t *val)
{
	uint8_t aln, shift;
	uint32_t i;
	uint64_t mask, res;
	struct usrdev *ud = (struct usrdev *) devopq;

	switch(size) {
	case 1:
		aln = 0;
		mask = 0xff;
		break;
	case 2:
		aln = 1;
		mask = 0xffff;
		break;
	case 4:
		aln = 3;
		mask = 0xffffffff;
		break;
	case 8:
		aln = 7;
		mask = (uint64_t)-1;
		break;
	default:
		return -EINVAL;
	}

	if (off & aln)
		return -EINVAL;

	i = off >> 3;
	shift = (off & 0x7) * 8;

	/* Current thread: Process */
	assert(id < MAXUSRDEVREMS);

	if (i >= SYS_DEVCONFIG_MAXUSERCFG) {
		return -EINVAL;
	}

	spinlock(&ud->lock);
	res = ud->cfg.usercfg[i];
	spinunlock(&ud->lock);

	res >>= shift;
	res &= mask;
	*val = res;
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
	ud->remths[id].irqmap[irq] = sig + 1;
	spinunlock(&ud->lock);
	return 0;
}

static void _usrdev_close(void *devopq, unsigned id)
{
	int i, ret;
	struct thread *th = current_thread();
	struct apert *apt;
	struct usrioreq *ior;
	int unused_ior = 0;
	struct usrdev *ud = (struct usrdev *) devopq;

	ior = structs_alloc(&usrioreqs);
	memset(ior, 0, sizeof(*ior));
	ior->id = id;
	ior->op = IOR_OP_CLOSE;
	ior->gid = th->egid;
	ior->uid = th->euid;

	/* Current thread: Process or Device */
	spinlock(&ud->lock);
	apt = ud->remths[id].apertbl;
	for (i = 0; i < MAXUSRDEVAPERTS; i++) {
		apt[i].va = 0;
		apt[i].sz = 0;
	}
	ud->remths[id].use = 0;

	if (ud->detaching) {
		unused_ior = 1;
	} else {
		TAILQ_INSERT_TAIL(&ud->ioreqs, ior, queue);
		thraise(ud->th, ud->sig);
		unused_ior = 0;
	}
	spinunlock(&ud->lock);
	pmap_commit(ud->remths[id].th->pmap);
	pmap_commit(ud->th->pmap);

	if (unused_ior)
		structs_free(ior);
}

static struct devops usrdev_ops = {
	.open = _usrdev_open,
	.clone = _usrdev_clone,
	.close = _usrdev_close,
	.in = _usrdev_in,
	.out = _usrdev_out,
	.export = _usrdev_export,
	.unexport = _usrdev_unexport,
	.info = _usrdev_info,
	.rdcfg = _usrdev_rdcfg,
	.irqmap = _usrdev_irqmap,
};

struct usrdev *usrdev_creat(struct sys_creat_cfg *cfg, unsigned sig, devmode_t mode)
{
	struct usrdev *ud;
	struct thread *th = current_thread();

	assert(sizeof(cfg->usercfg) == sizeof(ud->cfg.usercfg));

	ud = heap_alloc(sizeof(*ud));
	ud->lock = 0;
	ud->th = th;
	ud->sig = sig;
	ud->detaching = 0;
	ud->cfg.vid = cfg->vendorid;
	ud->cfg.did = cfg->deviceid;
	ud->cfg.nirqs = cfg->nirqs;
	memcpy(ud->cfg.usercfg, cfg->usercfg, sizeof(cfg->usercfg));
	memset(&ud->remths, 0, sizeof(ud->remths));
	TAILQ_INIT(&ud->ioreqs);

	dev_init(&ud->dev, cfg->nameid, (void *) ud, &usrdev_ops, th->euid, th->egid, mode);
	if (dev_attach(&ud->dev)) {
		heap_free(ud);
		ud = NULL;
	}
	return ud;
}

int usrdev_poll(struct usrdev *ud, struct sys_poll_ior *poll)
{
	int id;
	pid_t pid;
	struct usrioreq *ior;

retry:
	spinlock(&ud->lock);
	ior = TAILQ_FIRST(&ud->ioreqs);
	if (ior != NULL) {
		TAILQ_REMOVE(&ud->ioreqs, ior, queue);
		pid = ud->remths[ior->id].th->pid;
	}
	spinunlock(&ud->lock);

	if (ior == NULL)
		return -ENOENT;

	memset(poll, 0, sizeof(*poll));
	
	switch (ior->op) {
	case IOR_OP_OPEN:
		poll->op = SYS_POLL_OP_OPEN;
		break;
	case IOR_OP_CLONE:
		poll->op = SYS_POLL_OP_CLONE;
		poll->clone_id = ior->clone_id;
		break;
	case IOR_OP_OUT:
		poll->op = SYS_POLL_OP_OUT;
		poll->size = ior->size;
		poll->port = ior->port;
		poll->val = ior->val;
		break;
	case IOR_OP_CLOSE:
		poll->op = SYS_POLL_OP_CLOSE;
		break;
	default:
		panic("Invalid IOR entry type %d!\n", ior->op);
		goto retry;
	}
	poll->pid = pid;
	poll->gid = ior->gid;
	poll->uid = ior->uid;
	id = ior->id;
	structs_free(ior);
	return id;
}

int usrdev_wriospace(struct usrdev *ud, unsigned id, uint32_t port,
		     uint64_t val)
{
	uint8_t iosize = 1 << (port & IOPORT_SIZEMASK);
	uint16_t ioport = port >> IOPORT_SIZESHIFT;
	int i, start, end;
	uint8_t *ptr;

	start = MIN(ioport, IOSPACESZ);
	end = MIN(start + iosize, IOSPACESZ);

	ptr = (uint8_t *)&val;
	spinlock(&ud->lock);
	for (i = start; i < end; i++)
		ud->remths[id].iospace[i] = ptr[i - start];
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
	sig = ud->remths[id].irqmap[irq] - 1;
	if ((sig != -1) && ud->remths[id].use) {
		thraise(ud->remths[id].th, sig);
	}
	spinunlock(&ud->lock);
	return 0;
}

int usrdev_read(struct usrdev *ud, unsigned id, unsigned long iova, size_t sz, vaddr_t va)
{
	int i, ret;
	struct apert *apt;

	if (id >= MAXUSRDEVREMS)
		return -EINVAL;

	spinlock(&ud->lock);
	if (!ud->remths[id].use) {
		spinunlock(&ud->lock);
		return -ENOENT;
	}

	apt = ud->remths[id].apertbl;

	for (i = 0; i < MAXUSRDEVAPERTS; i++)
		if (iova >= apt[i].va 
		    && iova <= apt[i].va + apt[i].sz
		    && iova + sz >= apt[i].va
		    && iova + sz <= apt[i].va + apt[i].sz)
			break;

	if (i >= MAXUSRDEVAPERTS) {
		spinunlock(&ud->lock);
		return -EINVAL;
	}

	ret = xcopy_from(va, ud->remths[id].th, iova, sz);

	spinunlock(&ud->lock);

	return ret;
}

int usrdev_write(struct usrdev *ud, unsigned id, vaddr_t va, size_t sz, unsigned long iova)
{
	int i, ret;
	struct apert *apt;

	if (id >= MAXUSRDEVREMS)
		return -EINVAL;

	spinlock(&ud->lock);
	if (!ud->remths[id].use) {
		spinunlock(&ud->lock);
		return -ENOENT;
	}

	apt = ud->remths[id].apertbl;

	for (i = 0; i < MAXUSRDEVAPERTS; i++)
		if (iova >= apt[i].va 
		    && iova <= apt[i].va + apt[i].sz
		    && iova + sz >= apt[i].va
		    && iova + sz <= apt[i].va + apt[i].sz)
			break;

	if (i >= MAXUSRDEVAPERTS) {
		spinunlock(&ud->lock);
		return -EINVAL;
	}

	ret = xcopy_to(ud->remths[id].th, iova, va, sz);

	spinunlock(&ud->lock);

	return ret;
}

void usrdev_destroy(struct usrdev *ud)
{
	int sig;
	struct usrioreq *ior, *tmp;

	ud->detaching = 1;
	dev_detach(&ud->dev);
	spinlock(&ud->lock);
	TAILQ_FOREACH_SAFE(ior, &ud->ioreqs, queue, tmp) {
		structs_free(ior);
	}
	spinunlock(&ud->lock);

	heap_free(ud);
}

void usrdevs_init(void)
{

	setup_structcache(&usrioreqs, usrioreq);
}
