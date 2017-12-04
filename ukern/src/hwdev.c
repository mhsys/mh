/*
 * Copyright (c) 2016, Gianluca Guida
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
#include <uk/string.h>
#include <uk/logio.h>
#include <uk/bus.h>
#include <uk/structs.h>
#include <uk/kern.h>
#include <uk/pfndb.h>
#include <uk/hwdev.h>
#include <uk/cpu.h>
#include <machine/uk/platform.h>
#include <uk/pgalloc.h>
#include <uk/heap.h>
#include <uk/errno.h>

#define MAXHWDEVIDS SYS_INFO_MAX_DEVIDS
#define MAXHWDEVREMS 256

struct hwdma {
	pfn_t iopfn;
	vaddr_t va;
	 LIST_ENTRY(hwdma) list;
};

struct hwsig {
	struct irqsig irqsig;
	 LIST_ENTRY(hwsig) list;
};

struct hwmap {
	vaddr_t va;
	 LIST_ENTRY(hwmap) list;
};

struct remth {
	int use:1;		/* Entry in use */
	 LIST_HEAD(, hwsig) hwsigs;	/* Signals mapped */
	 LIST_HEAD(, hwmap) hwmaps;	/* MMIO mapped */
	 LIST_HEAD(, hwdma) hwdmas;	/* Exported pages */
};

struct seg {
	uint16_t base;
	uint16_t len;
};

struct memseg {
	paddr_t base;
	unsigned long len;
};

struct hwdev_cfg {
	uint64_t vid;
	uint64_t did[MAXHWDEVIDS];

	uint8_t nirqsegs;
	uint8_t npiosegs;

	uint8_t nmemsegs;
	struct memseg *memsegptr;

	struct seg segs[0];
};

struct hwdev {
	lock_t lock;
	struct dev dev;		/* Registered device */
	struct thread *th;	/* Device thread */

	struct remth remths[MAXHWDEVREMS];

	struct hwdev_cfg *cfg;	/* Device Configuration Space */
};

static int _hwdev_open(void *devopq, uint64_t did)
{
	int i, ret;
	struct hwdev *hd = (struct hwdev *) devopq;

	spinlock(&hd->lock);
	for (i = 0; i < MAXHWDEVREMS; i++) {
		if (!hd->remths[i].use)
			break;
	}
	if (i >= MAXHWDEVREMS) {
		ret = -ENFILE;
		goto out;
	}
	hd->remths[i].use = 1;
	LIST_INIT(&hd->remths[i].hwdmas);
	LIST_INIT(&hd->remths[i].hwsigs);
	LIST_INIT(&hd->remths[i].hwmaps);
	ret = i;
      out:
	spinunlock(&hd->lock);
	return ret;
}

static int _hwdev_in(void *devopq, unsigned id, uint32_t port,
		     uint64_t * val)
{
	int i, found;
	struct hwdev *hd = (struct hwdev *) devopq;
	struct hwdev_cfg *cfg = hd->cfg;
	struct seg *ptr = cfg->segs + cfg->nirqsegs;
	uint8_t iosize = 1 << (port & IOPORT_SIZEMASK);
	uint16_t ioport = port >> IOPORT_SIZESHIFT;

	found = 0;
	for (i = 0; i < cfg->npiosegs; i++, ptr++) {
		uint16_t start = ptr->base;
		uint16_t end = ptr->base + ptr->len - 1;

		if (start <= ioport && ioport <= end) {
			found = 1;
			break;
		}
	}
	if (!found)
		return -EPERM;


	switch (iosize) {
	case 1:
		*val = platform_inb(ioport);
		break;
	case 2:
		*val = platform_inw(ioport);
		break;
	case 4:
		*val = platform_inl(ioport);
		break;
	default:
		printf("Unsupported I/O port size %d at port %llx\n",
		       iosize, port);
		return -EINVAL;
	}
	return 0;
}

static int _hwdev_out(void *devopq, unsigned id, uint32_t port,
		      uint64_t val)
{
	int i, found;
	struct hwdev *hd = (struct hwdev *) devopq;
	struct hwdev_cfg *cfg = hd->cfg;
	struct seg *ptr = cfg->segs + cfg->nirqsegs;
	uint8_t iosize = 1 << (port & IOPORT_SIZEMASK);
	uint16_t ioport = port >> IOPORT_SIZESHIFT;

	found = 0;
	for (i = 0; i < cfg->npiosegs; i++, ptr++) {
		uint16_t start = ptr->base;
		uint16_t end = ptr->base + ptr->len - 1;

		if (start <= ioport && ioport <= end) {
			found = 1;
			break;
		}
	}
	if (!found)
		return -EPERM;


	switch (iosize) {
	case 1:
		platform_outb(ioport, val);
		break;
	case 2:
		platform_outw(ioport, val);
		break;
	case 4:
		platform_outl(ioport, val);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int _hwdev_export(void *devopq, unsigned id, vaddr_t va, size_t sz, uint64_t *iova)
{
	struct hwdev *hd = (struct hwdev *) devopq;
	struct hwdma *pd;
	pfn_t pfn = 0;
	l1e_t l1e;
	int ret;

	spinlock(&hd->lock);

	/* We don't support multiple pages without IOMMU. */
	if (trunc_page(va + sz - 1) != trunc_page(va))
		return -EINVAL;

	/* Export VA addresses */
	ret = pmap_uwire(NULL, va);
	if (ret)
		goto export_err;

	ret = pmap_phys(NULL, va, &pfn);
	if (ret) {
		assert(!pmap_uunwire(NULL, va));
		goto export_err;
	}

	pd = heap_alloc(sizeof(*pd));
	pd->va = va;
	pd->iopfn = pfn;
	LIST_INSERT_HEAD(&hd->remths[id].hwdmas, pd, list);
	*iova = ptoa(pfn) + (va & PAGE_MASK);
	ret = 0;
      export_err:
	spinunlock(&hd->lock);
	return ret;
}

static int _hwdev_unexport(void *devopq, unsigned id, vaddr_t va)
{
	int ret;
	struct hwdma *pd, *found;
	struct hwdev *hd = (struct hwdev *) devopq;

	spinlock(&hd->lock);
	found = NULL;
	LIST_FOREACH(pd, &hd->remths[id].hwdmas, list) {
		if (pd->va == va) {
			found = pd;
			break;
		}
	}

	if (!found) {
		ret = -ENOENT;
		goto unexport_err;
	}

	LIST_REMOVE(found, list);
	ret = pmap_uunwire(NULL, pd->va);
	assert(!ret);
	heap_free(found);
 unexport_err:
	spinunlock(&hd->lock);
	return ret;
}

static int _hwdev_iomap(void *devopq, unsigned id, vaddr_t va,
			paddr_t mmioaddr, pmap_prot_t prot)
{
	struct hwdev *hd = (struct hwdev *) devopq;
	pfn_t mmiopfn = (pfn_t) atop(mmioaddr);
	struct hwdev_cfg *cfg = hd->cfg;
	struct memseg *ptr = cfg->memsegptr;
	int i, found, ret;
	struct hwmap *hwmap;

	found = 0;
	for (i = 0; i < cfg->nmemsegs; i++, ptr++) {
		paddr_t start = trunc_page(ptr->base);
		paddr_t end = round_page(ptr->base + ptr->len);

		if (start <= mmioaddr && mmioaddr <= end) {
			found = 1;
			break;
		}
	}
	if (!found)
		return -EPERM;

	if ((va & PAGE_MASK) != (mmioaddr & PAGE_MASK)) {
		/* Do not confuse  the caller by thinking  one can get
		 * away with  mapping at  different page  offsets.  Do
		 * not  make  implicitly  a vfn-to-pfn  mapping  (thus
		 * ignoring the page  offset) as other implementations
		 * of other device types  might actually support this,
		 * e.g. through copying. */
		return -EINVAL;
	}

	if (!pfn_is_valid(mmiopfn))
		return -EINVAL;

	ret = iomap(va, mmiopfn, prot);
	if (ret < 0)
		return ret;

	hwmap = heap_alloc(sizeof(*hwmap));
	hwmap->va = va;
	spinlock(&hd->lock);
	LIST_INSERT_HEAD(&hd->remths[id].hwmaps, hwmap, list);
	spinunlock(&hd->lock);
	return 0;
}

static int _hwdev_iounmap(void *devopq, unsigned id, vaddr_t va)
{
	struct hwdev *hd = (struct hwdev *) devopq;
	struct hwmap *pm, *tpm;

	spinlock(&hd->lock);
	assert(hd->remths[id].use);
	LIST_FOREACH_SAFE(pm, &hd->remths[id].hwmaps, list, tpm) {
		if (pm->va == va) {
			LIST_REMOVE(pm, list);
			iounmap(va);
			heap_free(pm);
			spinunlock(&hd->lock);
			return 0;
		}
	}
	spinunlock(&hd->lock);
	return -ENOENT;
}


static int _hwdev_info(void *devopq, unsigned id,
			struct sys_info_cfg *cfg)
{
	int i;
	struct hwdev *hd = (struct hwdev *) devopq;
	struct hwdev_cfg *hdcfg = hd->cfg;

	cfg->vendorid = hdcfg->vid;
	for (i = 0; i < MAXHWDEVIDS; i++)
		cfg->deviceids[i] = hdcfg->did[i];

	cfg->flags |= SYS_INFO_FLAGS_HW;
	cfg->niopfns = -1;
	cfg->nirqsegs = hdcfg->nirqsegs;
	cfg->npiosegs = hdcfg->npiosegs;
	cfg->nmemsegs = hdcfg->nmemsegs;

	for (i = 0; i < hdcfg->nirqsegs + cfg->npiosegs; i++) {
		cfg->segs[i].base = hdcfg->segs[i].base;
		cfg->segs[i].len = hdcfg->segs[i].len;
	}
	for (i = 0; i < hdcfg->nmemsegs; i++) {
		cfg->memsegs[i].base = hdcfg->memsegptr[i].base;
		cfg->memsegs[i].len = hdcfg->memsegptr[i].len;
	}

	return 0;
}

static int _hwdev_irqmap(void *devopq, unsigned id, unsigned irq,
			 unsigned sig)
{
	struct hwdev *hd = (struct hwdev *) devopq;
	struct hwdev_cfg *cfg = hd->cfg;
	struct thread *th = current_thread();
	struct seg *ptr = hd->cfg->segs;
	struct hwsig *hwsig;
	int i, found, ret;

	spinlock(&hd->lock);
	found = 0;
	for (i = 0; i < cfg->nirqsegs; i++, ptr++) {
		uint16_t start = ptr->base;
		uint16_t end = ptr->base + ptr->len - 1;

		if (start <= irq && irq <= end) {
			found = 1;
			break;
		}
	}
	if (!found) {
		spinunlock(&hd->lock);
		return -ENOENT;
	}

	hwsig = heap_alloc(sizeof(struct hwsig));
	ret = irqregister(&hwsig->irqsig, irq, th, sig,
			  0 /* No filter */ );
	if (ret < 0) {
		heap_free(hwsig);
		spinunlock(&hd->lock);
		return ret;
	}
	LIST_INSERT_HEAD(&hd->remths[id].hwsigs, hwsig, list);
	spinunlock(&hd->lock);
	return ret;
}

static int _hwdev_eoi(void *devopq, unsigned id, unsigned irq,
			 unsigned sig)
{
	struct hwdev *hd = (struct hwdev *) devopq;
	struct hwdev_cfg *cfg = hd->cfg;
	struct thread *th = current_thread();
	struct seg *ptr = hd->cfg->segs;
	int i, found, ret;

	spinlock(&hd->lock);
	found = 0;
	for (i = 0; i < cfg->nirqsegs; i++, ptr++) {
		uint16_t start = ptr->base;
		uint16_t end = ptr->base + ptr->len - 1;

		if (start <= irq && irq <= end) {
			found = 1;
			break;
		}
	}
	if (!found) {
		spinunlock(&hd->lock);
		return -ENOENT;
	}

	irqeoi(irq);
	spinunlock(&hd->lock);
	return 0;
}

static void _hwdev_close(void *devopq, unsigned id)
{
	struct hwdev *hd = (struct hwdev *) devopq;
	struct hwsig *ps, *tps;
	struct hwmap *pm, *tpm;
	struct hwdma *pd, *tpd;

	spinlock(&hd->lock);
	assert(hd->remths[id].use);
	LIST_FOREACH_SAFE(pm, &hd->remths[id].hwmaps, list, tpm) {
		LIST_REMOVE(pm, list);
		iounmap(pm->va);
		heap_free(pm);
	}
	LIST_FOREACH_SAFE(ps, &hd->remths[id].hwsigs, list, tps) {
		LIST_REMOVE(ps, list);
		irqunregister(&ps->irqsig);
		heap_free(ps);
	}
	LIST_FOREACH_SAFE(pd, &hd->remths[id].hwdmas, list, tpd) {
		LIST_REMOVE(pd, list);
		assert(!pmap_uunwire(NULL, pd->va));
		heap_free(pd);
	}
	hd->remths[id].use = 0;
	spinunlock(&hd->lock);
}

static struct devops hwdev_ops = {
	.open = _hwdev_open,
	.close = _hwdev_close,
	.in = _hwdev_in,
	.out = _hwdev_out,
	.export = _hwdev_export,
	.unexport = _hwdev_unexport,
	.iomap = _hwdev_iomap,
	.iounmap = _hwdev_iounmap,
	.info = _hwdev_info,
	.irqmap = _hwdev_irqmap,
	.eoi = _hwdev_eoi,
};

struct hwdev *hwdev_creat(struct sys_hwcreat_cfg *syscfg, devmode_t mode)
{
	int i, segs, nmemsegs;
	size_t size;
	struct hwdev *hd;
	struct thread *th = current_thread();

	segs = syscfg->nirqsegs + syscfg->npiosegs;
	nmemsegs = syscfg->nmemsegs;

	size = sizeof(*hd);
	size += sizeof(struct hwdev_cfg);
	size += sizeof(struct seg) * segs;
	size += sizeof(struct memseg) * nmemsegs;

	hd = heap_alloc(size);
	hd->lock = 0;
	hd->th = th;
	memset(&hd->remths, 0, sizeof(hd->remths));
	hd->cfg = (struct hwdev_cfg *) (hd + 1);
	hd->cfg->vid = syscfg->vendorid;
	for (i = 0; i < MAXHWDEVIDS; i++)
		hd->cfg->did[i] = syscfg->deviceids[i];
	hd->cfg->nirqsegs = syscfg->nirqsegs;
	hd->cfg->npiosegs = syscfg->npiosegs;
	hd->cfg->nmemsegs = syscfg->nmemsegs;

	for (i = 0; i < segs; i++) {
		hd->cfg->segs[i].base = syscfg->segs[i].base;
		hd->cfg->segs[i].len = syscfg->segs[i].len;
	}
	hd->cfg->memsegptr = (struct memseg *) &hd->cfg->segs[i];
	for (i = 0; i < nmemsegs; i++) {
		hd->cfg->memsegptr[i].base = syscfg->memsegs[i].base;
		hd->cfg->memsegptr[i].len = syscfg->memsegs[i].len;
	}

	dev_init(&hd->dev, syscfg->nameid, (void *) hd, &hwdev_ops,
		 th->euid, th->egid, mode);
	if (dev_attach(&hd->dev)) {
		heap_free(hd);
		hd = NULL;
	}
	return hd;
}
