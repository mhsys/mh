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
#include <uk/stddef.h>
#include <uk/assert.h>
#include <uk/string.h>
#include <uk/logio.h>
#include <uk/locks.h>
#include <machine/uk/pmap.h>
#include <machine/uk/pae.h>
#include <machine/uk/tlb.h>
#include <uk/cpu.h>
#include <uk/fixmems.h>
#include <uk/structs.h>
#include <uk/pfndb.h>
#include <uk/kern.h>

struct slab pmap_cache;
l1e_t *pmap_kernel_l1;

__regparm void __setpdptr(void *);

#define DEUKERNBASE(_p) ((uintptr_t)(_p) - UKERNBASE)

#define pmap_current()						\
    ((struct pmap *)((uintptr_t)UKERNBASE + __getpdptr()	\
		     - offsetof(struct pmap, pdptr)))
struct pmap *pmap_alloc(void)
{
	l1e_t *l1s;
	struct pmap *pmap;

	l1s = (l1e_t *) alloc12k();
	if (l1s == NULL)
		return NULL;
	memset(l1s, 0, 12 * 1024);

	pmap = structs_alloc(&pmap_cache);
	if (pmap == NULL) {
		free12k(l1s);
		return NULL;
	}

	l1s[NPTES * 2 + LINOFF + 0] = mklinl1e(DEUKERNBASE(l1s),
					       PG_A | PG_D | PG_W | PG_P);
	l1s[NPTES * 2 + LINOFF + 1] = mklinl1e(DEUKERNBASE(l1s + NPTES),
					       PG_A | PG_D | PG_W | PG_P);
	l1s[NPTES * 2 + LINOFF + 2] =
		mklinl1e(DEUKERNBASE(l1s + NPTES * 2),
			 PG_A | PG_D | PG_W | PG_P);
	l1s[NPTES * 2 + LINOFF + 3] =
		mklinl1e(DEUKERNBASE(pmap_kernel_l1),
			 PG_A | PG_D | PG_W | PG_P);

	pmap->pdptr[0] = mkl2e(DEUKERNBASE(l1s), PG_P);
	pmap->pdptr[1] = mkl2e(DEUKERNBASE(l1s + 512), PG_P);
	pmap->pdptr[2] = mkl2e(DEUKERNBASE(l1s + 1024), PG_P);
	pmap->pdptr[3] = mkl2e(DEUKERNBASE(pmap_kernel_l1), PG_P);
	pmap->l1s = l1s;

	pmap->lock = 0;
	pmap->refcnt = 0;

	return pmap;
}

void pmap_free(struct pmap *pmap)
{
	assert(pmap != NULL && pmap != pmap_current());
	free12k(pmap->l1s);
	structs_free(pmap);
}

static void __setl1e(l1e_t * l1p, l1e_t l1e)
{
	volatile uint32_t *ptr = (volatile uint32_t *) l1p;

	*ptr = 0;
	*++ptr = l1e >> 32;
	*--ptr = l1e & 0xffffffff;
}

static l1e_t _pmap_set(struct pmap *pmap, l1e_t * l1p, l1e_t nl1e)
{
	l1e_t ol1e;

	ol1e = *l1p;
	pmap->tlbflush = __tlbflushp(ol1e, nl1e);
	__setl1e(l1p, nl1e);

	return ol1e;
}

int _pmap_fault(vaddr_t va, unsigned long err, struct usrframe *f)
{
	int is_cow = 0;
	u_long xcpterr;
	struct pmap *pmap = pmap_current();
	l1e_t l1e, *l1p = __val1tbl(va) + L1OFF(va);

	spinlock(&pmap->lock);
	l1e = *l1p;
	if (l1e_cow(l1e)) {
		is_cow = 1;
		if (pfn_getref(l1epfn(l1e)) == 1) {
			printf("Fixing last COW!\n");
			/* Last COW mapping. */
			l1e = l1e_mknormal(l1e);
			/* Do not set writable. It is user
			 * responsibility to do so. */
			__setl1e(l1p, l1e);
			/* No TLB flush. We haven't changed
			 * permissions. */
			spinunlock(&pmap->lock);
			return 1 /* Spurious, return */ ;
		}
	}
	spinunlock(&pmap->lock);

	/* Translate #PF err code */
	if (err & 1)		/* P */
		xcpterr = PG_ERR_REASON_PROT;
	else
		xcpterr = PG_ERR_REASON_NOTP;

	if (is_cow)
		xcpterr |= PG_ERR_INFO_COW;
	if (err & 2)
		xcpterr |= PG_ERR_INFO_WRITE;
	assert(err & 4);	/* User access here */

	thintr(XCPT_PGFAULT, f->cr2, xcpterr);
	return 0;
}

int pmap_kenter(struct pmap *pmap, vaddr_t va, pfn_t pfn,
		pmap_prot_t prot, pfn_t * opfn)
{
	int ret = 0;
	l1e_t ol1e, nl1e, *l1p;

	assert(!__isuaddr(va));
	if (is_prot_present(prot))
		assert(!is_prot_user(prot));

	if (pmap == NULL)
		pmap = pmap_current();

	if (pmap == pmap_current())
		l1p = __val1tbl(va) + L1OFF(va);
	else
		panic("set to different pmap voluntarily not supported.");

	nl1e = mkl1e(ptoa(pfn), prot);

	spinlock(&pmap->lock);
	ol1e = _pmap_set(pmap, l1p, nl1e);
	spinunlock(&pmap->lock);

	if (ol1e & PG_P)
		*opfn = atop(ol1e);
	else
		*opfn = PFN_INVALID;
	return ret;
}

int pmap_uiomap(struct pmap *pmap, vaddr_t va, pfn_t pfn,
		pmap_prot_t prot, pfn_t * opfn)
{
	l1e_t ol1e, nl1e, *l1p;

	assert(__isuaddr(va));
	if (is_prot_present(prot)) {
		assert(is_prot_user(prot));
		assert(pfn_is_mmio(pfn));
	}

	if (pmap == NULL)
		pmap = pmap_current();

	if (pmap == pmap_current())
		l1p = __val1tbl(va) + L1OFF(va);
	else
		panic("iomap on foreign pmaps arbitrarily not supported.");

	nl1e = l1e_mkiomap(mkl1e(ptoa(pfn), prot));
	spinlock(&pmap->lock);
	ol1e = *l1p;
	if (l1e_external(ol1e)) {
		/* Externally controlled */
		spinunlock(&pmap->lock);
		if (opfn)
			*opfn = PFN_INVALID;
		return -EBUSY;
	}
	pmap->tlbflush = __tlbflushp(ol1e, nl1e);
	__setl1e(l1p, nl1e);
	spinunlock(&pmap->lock);

	if (l1e_normal(ol1e) && !pfn_decref(l1epfn(ol1e)))
		*opfn = l1epfn(ol1e);
	else
		*opfn = PFN_INVALID;
	return 0;
}

int pmap_uiounmap(struct pmap *pmap, vaddr_t va)
{
	l1e_t *l1p, nl1e, ol1e;

	assert(__isuaddr(va));
	if (pmap == NULL)
		pmap = pmap_current();

	if (pmap == pmap_current())
		l1p = __val1tbl(va) + L1OFF(va);
	else
		l1p = pmap->l1s + NPTES * L2OFF(va) + L1OFF(va);

	spinlock(&pmap->lock);
	ol1e = *l1p;
	if (!l1e_iomap(ol1e)) {
		/* Not iomap! */
		spinunlock(&pmap->lock);
		return -1;
	}
	nl1e = mkl1e(PFN_INVALID, 0);
	pmap->tlbflush = __tlbflushp(ol1e, nl1e);
	__setl1e(l1p, nl1e);
	spinunlock(&pmap->lock);

	return 0;
}

int pmap_phys(struct pmap *pmap, vaddr_t va, pfn_t *pfn)
{
	int ret;
	l1e_t *l1p, l1e;

	if (pmap == NULL)
		pmap = pmap_current();
	if (pmap == pmap_current())
		l1p = __val1tbl(va) + L1OFF(va);
	else
		panic("phys() of different pmap voluntarily not supported.");

	ret = -EFAULT;
	spinlock(&pmap->lock);
	l1e = *l1p;
	if (l1e_external(l1e) && l1e_present(l1e)) {
		*pfn = l1epfn(l1e);
		ret = 0;
	}
	spinunlock(&pmap->lock);

	return ret;
}

/* Enter a new user page (or an non-present entry) in the pmap */
int pmap_uenter(struct pmap *pmap, vaddr_t va, pfn_t pfn,
		pmap_prot_t prot, pfn_t * opfn)
{
	l1e_t ol1e, nl1e, *l1p;

	assert(__isuaddr(va));
	if (is_prot_present(prot)) {
		assert(is_prot_user(prot));
		assert(pfn_is_userpage(pfn));
	}

	if (pmap == NULL)
		pmap = pmap_current();

	if (pmap == pmap_current())
		l1p = __val1tbl(va) + L1OFF(va);
	else
		l1p = pmap->l1s + NPTES * L2OFF(va) + L1OFF(va);

	nl1e = l1e_mknormal(mkl1e(ptoa(pfn), prot));
	spinlock(&pmap->lock);
	ol1e = *l1p;
	if (l1e_external(ol1e)) {
		/* Externally controlled */
		spinunlock(&pmap->lock);
		if (opfn)
			*opfn = PFN_INVALID;
		return -EBUSY;
	}
	if (nl1e & PG_P)
		pfn_incref(l1epfn(nl1e));
	pmap->tlbflush = __tlbflushp(ol1e, nl1e);
	__setl1e(l1p, nl1e);
	spinunlock(&pmap->lock);

	if (l1e_normal(ol1e) && !pfn_decref(l1epfn(ol1e)))
		*opfn = l1epfn(ol1e);
	else
		*opfn = PFN_INVALID;
	return 0;
}

int pmap_uchaddr(struct pmap *pmap, vaddr_t va, vaddr_t newva,
		 pfn_t * opfn)
{
	l1e_t nl1e, ol1e, *l1p1, *l1p2;

	assert(__isuaddr(va));
	assert(__isuaddr(newva));

	if (pmap == NULL)
		pmap = pmap_current();

	if (pmap == pmap_current()) {
		l1p1 = __val1tbl(va) + L1OFF(va);
		l1p2 = __val1tbl(newva) + L1OFF(newva);
	} else
		panic("pmap_uchaddr not implemented (unneeded).");

	spinlock(&pmap->lock);
	nl1e = *l1p1;
	if (l1e_external(nl1e)) {
		/* Cant move externally controlled page */
		spinunlock(&pmap->lock);
		*opfn = PFN_INVALID;
		return -EBUSY;
	}
	ol1e = *l1p2;
	if (l1e_external(ol1e)) {
		/* Can't move into externally controlled page */
		spinunlock(&pmap->lock);
		*opfn = PFN_INVALID;
		return -EBUSY;
	}
	__setl1e(l1p1, mkl1e(ptoa(PFN_INVALID), 0));
	__setl1e(l1p2, nl1e);
	pmap->tlbflush |= __tlbflushp(ol1e, nl1e);
	pmap->tlbflush |= __tlbflushp(nl1e, mkl1e(PFN_INVALID, 0));
	spinunlock(&pmap->lock);

	if (l1e_normal(ol1e) && !pfn_decref(l1epfn(ol1e)))
		*opfn = l1epfn(ol1e);
	else
		*opfn = PFN_INVALID;
	return 0;
}

int pmap_uchprot(struct pmap *pmap, vaddr_t va, pmap_prot_t prot)
{
	l1e_t ol1e, nl1e, *l1p;


	assert(is_prot_present(prot) && "can't unmap with chprot");
	assert(is_prot_user(prot));
	assert(__isuaddr(va));

	if (pmap == NULL)
		pmap = pmap_current();

	if (pmap == pmap_current())
		l1p = __val1tbl(va) + L1OFF(va);
	else
		panic("set to different pmap voluntarily not supported.");

	spinlock(&pmap->lock);
	ol1e = *l1p;
	if (l1e_external(ol1e)) {
		/* Can't change external mappings. */
		spinunlock(&pmap->lock);
		return -EBUSY;
	}
	if (!(l1eflags(ol1e) & PG_P)) {
		/* Not present, do not change. */
		spinunlock(&pmap->lock);
		return -EPERM;
	}
	if (is_prot_writeable(prot) && l1e_cow(ol1e)) {
		/* Can't make COW writable with chprot */
		spinunlock(&pmap->lock);
		return -EINVAL;
	}

	assert(l1e_normal(ol1e));
	nl1e = l1e_mknormal(mkl1e(ptoa(l1epfn(ol1e)), prot));

	pmap->tlbflush = __tlbflushp(ol1e, nl1e);
	__setl1e(l1p, nl1e);
	spinunlock(&pmap->lock);

	return 0;
}

int pmap_hmap(struct pmap *pmap, vaddr_t va)
{
	l1e_t xl1e, ol1e, *l1p;

	assert(__isuaddr(va));
	if (pmap == NULL)
		pmap = pmap_current();

	if (pmap == pmap_current())
		l1p = __val1tbl(va) + L1OFF(va);
	else
		l1p = pmap->l1s + NPTES * L2OFF(va) + L1OFF(va);

	spinlock(&pmap->lock);
	xl1e = *l1p;

	l1p = __val1tbl(KVA_HYPER) + L1OFF(KVA_HYPER);
	ol1e = *l1p;
	assert(!l1e_present(ol1e));
	__setl1e(l1p, xl1e);
	__flush_local_tlbs(); // invalidate single entry!

	/* Keep pmap lock on! */
}

int pmap_hunmap(struct pmap *pmap)
{
	l1e_t nl1e, *l1p;

	if (pmap == NULL)
		pmap = pmap_current();

	l1p = __val1tbl(KVA_HYPER) + L1OFF(KVA_HYPER);
	nl1e = mkl1e(PFN_INVALID, 0);
	__setl1e(l1p, nl1e);
	__flush_local_tlbs(); // invalidate single entry!

	spinunlock(&pmap->lock);
}

int pmap_uwire(struct pmap *pmap, vaddr_t va)
{
	l1e_t ol1e, nl1e, *l1p;

	assert(__isuaddr(va));
	if (pmap == NULL)
		pmap = pmap_current();

	if (pmap == pmap_current())
		l1p = __val1tbl(va) + L1OFF(va);
	else
		panic("wire to different pmap voluntarily not supported.");

	spinlock(&pmap->lock);
	ol1e = *l1p;
	if (l1e_iomap(ol1e)) {
		/* Externally controlled already. */
		spinunlock(&pmap->lock);
		return -EBUSY;
	}
	pfn_incwir(l1epfn(ol1e));
	nl1e = l1e_mkwired(ol1e);
	pmap->tlbflush = __tlbflushp(ol1e, nl1e);
	__setl1e(l1p, nl1e);
	spinunlock(&pmap->lock);

	return 0;
}

int pmap_uunwire(struct pmap *pmap, vaddr_t va)
{
	l1e_t ol1e, nl1e, *l1p;

	assert(__isuaddr(va));
	if (pmap == NULL)
		pmap = pmap_current();

	if (pmap == pmap_current())
		l1p = __val1tbl(va) + L1OFF(va);
	else
		l1p = pmap->l1s + NPTES * L2OFF(va) + L1OFF(va);

	spinlock(&pmap->lock);
	ol1e = *l1p;
	if (!l1e_wired(ol1e)) {
		/* Not wired! */
		spinunlock(&pmap->lock);
		return -1;
	}
	if (!pfn_decwir(l1epfn(ol1e))) {
		nl1e = l1e_mknormal(ol1e);
		pmap->tlbflush = __tlbflushp(ol1e, nl1e);
		__setl1e(l1p, nl1e);
	}
	spinunlock(&pmap->lock);

	return 0;
}

struct pmap *pmap_copy(void)
{
	struct pmap *pmap, *new;
	l1e_t *orig, *copy, l1e;
	vaddr_t va;

	pmap = pmap_current();
	new = pmap_alloc();

	spinlock(&pmap->lock);

	for (va = ZCOWBASE; va < ZCOWEND; va += PAGE_SIZE) {
		/* No COW area. Leave blank */
		copy = new->l1s + NPTES * L2OFF(va) + L1OFF(va);
		__setl1e(copy, 0);
	}
	for (va = COWBASE; va < COWEND; va += PAGE_SIZE) {
		orig = __val1tbl(va) + L1OFF(va);
		copy = new->l1s + NPTES * L2OFF(va) + L1OFF(va);
		l1e = *orig;

		if (!(l1e & PG_P)) {
			/* Not present, copy */
			__setl1e(copy, l1e);
		} else if (l1e_external(l1e)) {
			/* Do not inherit I/O mappings. This means
			 * that wired memory (exported by hwdev) will
			 * be lost on fork's child. */
			__setl1e(copy, 0);
		} else {
			/* COW, even for readonly pages */
			assert(l1e & PG_U);
			pfn_incref(l1epfn(l1e));
			/* Remove writable */
			l1e = l1e_mkcow(l1e);
			__setl1e(orig, l1e);
			__setl1e(copy, l1e);
			pmap->tlbflush |= TLBF_NORMAL;
		}
	}
	/* TLB of copy not affected. We just created it */
	spinunlock(&pmap->lock);

	return new;
}

void pmap_commit(struct pmap *pmap)
{

	if (pmap == NULL)
		pmap = pmap_current();

	spinlock(&pmap->lock);
	if (pmap->tlbflush & TLBF_GLOBAL)
		__flush_tlbs(-1, TLBF_GLOBAL);
	else if (pmap->tlbflush & TLBF_NORMAL)
		__flush_tlbs(pmap->cpumap, TLBF_NORMAL);
	pmap->tlbflush = 0;
	spinunlock(&pmap->lock);
}

void pmap_switch(struct pmap *pmap)
{
	struct pmap *oldpmap;

	oldpmap = pmap_current();
	pmap->refcnt++;
	pmap->cpumap |= ((cpumask_t) 1 << cpu_number());
	__setpdptr(pmap->pdptr);
	oldpmap->refcnt--;
	oldpmap->cpumap &= ~((cpumask_t) 1 << cpu_number());
}

struct pmap *pmap_boot(void)
{
	struct pmap *bpmap;

	/* Safe now to alloc a pmap */
	bpmap = pmap_alloc();
	if (bpmap == NULL)
		panic("BPMAP: FAILED");

	/* Set pmap current */
	/* No need for per-cpu pointer, exactly what CR3 is, minus
	   the offset of pdptr in pmap. Incidentally, it is zero now. */
	bpmap->refcnt++;
	__setpdptr(bpmap->pdptr);

	return bpmap;
}

void pmap_init(void)
{
	int i;
	l1e_t *kl1;

	setup_structcache(&pmap_cache, pmap);

	/* Initialise kernel pagetable */
	pmap_kernel_l1 = alloc4k();
	if (pmap_kernel_l1 < 0)
		panic("BOOTL1: Alloc failed");

	kl1 = (void *) pmap_kernel_l1;
	for (i = KL1_SDMAP; i < KL1_EDMAP; i++)
		kl1[i] = mkl1e((i * PAGE_SIZE), PG_A | PG_D | PG_W | PG_P);
	for (; i < NPTES; i++)
		kl1[i] = 0;

	/* Safe now to alloc a pmap */
}
