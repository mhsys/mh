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
#include <uk/param.h>
#include <lib/lib.h>

#include <uk/pfndb.h>

static lock_t pfndblock;
static ipfn_t *pfndb = NULL;

uint32_t pfndb_max = 0;
static u_long pfndb_stats[PFNT_NUM];

static const char *pfndb_names[PFNT_NUM] = {
	"Invalid Entries",
	"Page-tables",
	"Pagezone (1)",
	"Pagezone (S)",
	"Pagezone (E)",
	"Fixmem",
	"User",
	"Free",
	"System",
	"Reserved",
};

static void pfndb_stats_inctype(uint8_t t)
{

	assert(t < PFNT_NUM);
	pfndb_stats[t]++;
}

static void pfndb_stats_dectype(uint8_t t)
{

	if (t != PFNT_INVALID)
		pfndb_stats[t]--;
}

void *pfndb_setup(void *addr, unsigned maxpfn)
{

	pfndb = (ipfn_t *) addr;
	memset(pfndb, 0, maxpfn * sizeof(ipfn_t));
	pfndb_max = maxpfn;

	return pfndb + pfndb_max;
}

void pfndb_subst(uint8_t t1, uint8_t t2)
{
	unsigned i;

	spinlock(&pfndblock);
	for (i = 0; i < pfndb_max; i++)
		if (pfndb[i].type == t1) {
			pfndb[i].type = t2;
			pfndb_stats_dectype(t1);
			pfndb_stats_inctype(t2);
		}
	spinunlock(&pfndblock);
}

void pfndb_add(unsigned pfn, uint8_t t)
{
	uint8_t ot;
	ipfn_t *p = &pfndb[pfn];

	assert(pfn <= pfndb_max);

	spinlock(&pfndblock);
	ot = p->type;

	/* Init-time overlap checking */
	if (ot > t) {
		spinunlock(&pfndblock);
		return;
	}
	p->type = t;
	spinunlock(&pfndblock);

	pfndb_stats_dectype(ot);
	pfndb_stats_inctype(t);
}

void pfndb_settype(unsigned pfn, uint8_t t)
{
	uint8_t ot;
	ipfn_t *p = &pfndb[pfn];

	assert(pfn <= pfndb_max);

	spinlock(&pfndblock);
	ot = p->type;
	p->type = t;
	p->ref = 0;
	spinunlock(&pfndblock);

	pfndb_stats_dectype(ot);
	pfndb_stats_inctype(t);
}

uint8_t pfndb_type(unsigned pfn)
{
	uint8_t t;
	ipfn_t *p = &pfndb[pfn];

	assert(pfn <= pfndb_max);

	spinlock(&pfndblock);
	t = p->type;
	spinunlock(&pfndblock);
	return t;
}

void pfndb_clrref(unsigned pfn)
{
	ipfn_t *p = &pfndb[pfn];

	assert(pfn <= pfndb_max);
	p->ref = 0; /* Unlocked */
}

void pfndb_incref(unsigned pfn)
{
	ipfn_t *p = &pfndb[pfn];

	assert(pfn <= pfndb_max);
	__sync_add_and_fetch(&p->ref, 1);
}

uint64_t pfndb_getref(unsigned pfn)
{
	ipfn_t *p = &pfndb[pfn];

	assert(pfn <= pfndb_max);
	return __sync_add_and_fetch(&p->ref, 0);
}

uint64_t pfndb_decref(unsigned pfn)
{
	ipfn_t *p = &pfndb[pfn];

	assert(pfn <= pfndb_max);
	return __sync_add_and_fetch(&p->ref, -1);
}

void *pfndb_getptr(unsigned pfn)
{
	void *ptr;
	ipfn_t *p = &pfndb[pfn];

	assert(pfn <= pfndb_max);

	spinlock(&pfndblock);
	ptr = (void *) p->ptr;
	spinunlock(&pfndblock);

	return ptr;
}

void pfndb_printstats(void)
{
	uint8_t i;

	printf("PFN Database stats:\n");
	for (i = 1; i < PFNT_NUM; i++)
		printf("\t%-20s:\t%-10ld\n",
		       pfndb_names[i], pfndb_stats[i]);
}

void pfndb_printranges(void)
{
	uint8_t t;
	ipfn_t *sptr, *ptr, *max;

	ptr = pfndb;
	max = pfndb + pfndb_max;

	do {
		sptr = ptr;
		t = ptr->type;
		while (++ptr < max && ptr->type == t);
		printf("\t%016llx:%016llx -> %-20s\n",
		       (uint64_t) (sptr - pfndb) * PAGE_SIZE,
		       (uint64_t) (ptr - pfndb) * PAGE_SIZE,
		       pfndb_names[t]);

	} while (ptr < max);
}
