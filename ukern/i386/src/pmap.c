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
#include <uk/assert.h>
#include <uk/locks.h>
#include <machine/uk/pmap.h>
#include <machine/uk/pae.h>
#include <machine/uk/tlb.h>
#include <machine/uk/cpu.h>
#include <uk/fixmems.h>
#include <uk/structs.h>
#include <uk/pgalloc.h>
#include <lib/lib.h>

struct slab pmap_cache;
l1e_t *pmap_kernel_l1;

__regparm void __setpdptr(void *);

#define DEUKERNBASE(_p) ((uintptr_t)(_p) - UKERNBASE)

#define userpage_incref(_x) pfndb_incref(_x)
#define userpage_decref(_x) pfndb_decref(_x)
#define userpage_getref(_x) pfndb_getref(_x)

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

int
_pmap_fault(vaddr_t va)
{
	int ret = -1;
	struct pmap *pmap = pmap_current();
	l1e_t l1e, *l1p = __val1tbl(va) + L1OFF(va);	

	/* Check that is not a spurious COW fault */
	spinlock(&pmap->lock);
	l1e = *l1p;
	if (((l1e & (PG_P|PG_U|PG_W|PG_COW)) == (PG_P|PG_U|PG_COW))
	    && (userpage_getref(l1epfn(l1e)) == 1)) {
		printf("Fixing last COW!\n");
		/* Last COW mapping. */
		l1e &= ~PG_COW;
		l1e |= PG_W;
		__setl1e(l1p, l1e);
		/* No TLB flush. We only increased permissions */
		ret = 0;
	}
	spinunlock(&pmap->lock);
	return ret;
}

int
pmap_enter(struct pmap * pmap, vaddr_t va, paddr_t pa, pmap_prot_t prot, pfn_t *pfn)
{
	pfn_t opfn;
	l1e_t ol1e, nl1e, *l1p;

	if (pmap == NULL)
		pmap = pmap_current();

	nl1e = mkl1e(pa, prot);
	if ((prot & (PG_U|PG_P)) == (PG_U|PG_P))
		userpage_incref(l1epfn(nl1e));
	
	spinlock(&pmap->lock);
	if (pmap == pmap_current())
		l1p = __val1tbl(va) + L1OFF(va);
	else
		l1p = pmap->l1s + L2OFF(va) + L1OFF(va);

	ol1e = *l1p;
	pmap->tlbflush = __tlbflushp(ol1e, nl1e);

	__setl1e(l1p, nl1e);
	spinunlock(&pmap->lock);

	if (((ol1e & (PG_U|PG_P)) == (PG_U|PG_P))
	    && !userpage_decref(l1epfn(ol1e)))
		opfn = l1epfn(ol1e);
	else
		opfn = PFN_INVALID;

	if (pfn != NULL)
		*pfn = opfn;
	return 0;
}	

int pmap_chprot(struct pmap *pmap, vaddr_t va, pmap_prot_t prot)
{
	l1e_t ol1e, nl1e, *l1p;


	assert((prot & PG_P) && "can't unmap with chprot");
	if (pmap == NULL)
		pmap = pmap_current();

	if (pmap == pmap_current())
		l1p = __val1tbl(va) + L1OFF(va);
	else
		panic("set to different pmap voluntarily not supported.");

	spinlock(&pmap->lock);
	ol1e = *l1p;
	if (!(l1eflags(ol1e) & PG_P)) {
		/* Not present, do not change. */
		spinunlock(&pmap->lock);
		return -1;
	}
	if ((l1eflags(ol1e) & PG_COW) && (prot & PG_W)) {
		/* Can't make COW writable with chprot */
		spinunlock(&pmap->lock);
		return -1;
	}
	nl1e = mkl1e(ptoa(l1epfn(ol1e)), prot);
	pmap->tlbflush = __tlbflushp(ol1e, nl1e);
	__setl1e(l1p, nl1e);

	spinunlock(&pmap->lock);

	return 0;
}

struct pmap *pmap_copy(struct pmap *pmap)
{
	struct pmap *new = pmap_alloc();
	l1e_t *orig, *copy, l1e;
	vaddr_t va;

	if (pmap == NULL)
		pmap = pmap_current();

	if (pmap != pmap_current())
		panic("copy from a different pmap voluntarily not supported.");

	spinlock(&pmap->lock);
	for (va = NOCOWBASE; va < NOCOWEND; va += PAGE_SIZE) {
		/* NO COW area. Leave blank */
		copy = new->l1s + NPTES * L2OFF(va) + L1OFF(va);	  
		__setl1e(copy, 0);		
	}
	for (va = NOCOWEND; va < USEREND; va += PAGE_SIZE) {
		orig = __val1tbl(va) + L1OFF(va);
		copy = new->l1s + NPTES * L2OFF(va) + L1OFF(va);
		l1e = *orig;

		if (!(l1e & PG_P)) {
			/* Not present, copy */
			__setl1e(copy, l1e);
		} else if (l1e_is_foreign(l1e)) {
			/* Foreign. Leave blank */
			__setl1e(copy, 0);
		} else {
			/* COW, even for readonly pages */
			assert(l1e & PG_U);
			userpage_incref(l1epfn(l1e));			
			l1e &= ~PG_W;
			l1e |= PG_COW;
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
