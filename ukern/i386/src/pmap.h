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


#ifndef __i386_pmap_h
#define __i386_pmap_h

#include <uk/queue.h>
#include <uk/param.h>
#include <machine/uk/pae.h>

struct pmap {
	/* Needs to be first and cache aligned (32-byte needed by HW) */
	l2e_t pdptr[NPDPTE];
	l1e_t *l1s;

	unsigned tlbflush;
	cpumask_t cpumap;
	unsigned refcnt;
	lock_t lock;
};

struct pv_entry {
	LIST_ENTRY(pv_entry) list_entry;
	struct pmap *pmap;
	unsigned vfn;
};

void pmap_init(void);
struct pmap *pmap_boot(void);
struct pmap *pmap_alloc(void);
void pmap_switch(struct pmap *pmap);
void pmap_free(struct pmap *);
int _pmap_fault(vaddr_t va);

#define PROT_KERNWRX   (PROT_KERNX | PG_A | PG_D | PG_W | PG_D)
#define PROT_KERNWR    (PROT_KERN | PG_A | PG_D | PG_W | PG_D)
#define PROT_KERN      (PG_P | PG_NX)
#define PROT_KERNX     PG_P
#define PROT_GLOBAL    PG_G
#define PROT_USER      (PG_U | PG_NX | PG_P)
#define PROT_USER_X    (PG_U | PG_P)
#define PROT_USER_WR   (PG_W | PG_U | PG_NX | PG_P)
#define PROT_USER_WRX  (PG_W | PG_U | PG_P)
typedef unsigned pmap_prot_t;

#define FAULT_P 1
#define FAULT_W 2
#define FAULT_U 4
#define FAULT_X 8
typedef unsigned pmap_fault_t;

uintptr_t __getpdptr(void);

l1e_t pmap_setl1e(struct pmap *pmap, vaddr_t va, l1e_t l1e);
int pmap_enter(struct pmap *pmap, vaddr_t va, paddr_t pa,
	       pmap_prot_t prot, pfn_t *pfn);
int pmap_chprot(struct pmap *pmap, vaddr_t va, pmap_prot_t prot);
void pmap_commit(struct pmap *pmap);

#define pmap_clear(_pmap, _va, _pfn) pmap_enter((_pmap), (_va), 0, 0, _pfn)
#define pmap_current()                                          \
    ((struct pmap *)((uintptr_t)UKERNBASE + __getpdptr()        \
                     - offsetof(struct pmap, pdptr)))

#endif
