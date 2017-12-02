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
#include <uk/param.h>
#include <uk/logio.h>
#include <machine/uk/pmap.h>
#include <machine/uk/cpu.h>
#include <machine/uk/i386.h>
#include <uk/pfndb.h>
#include <uk/fixmems.h>
#include <uk/heap.h>
#include <uk/vmap.h>
#include <acpica/acpica.h>
#include <uk/kern.h>

char *_boot_cmdline = NULL;

void _load_segs(unsigned int, struct tss *, struct cpu_info **cpu);

struct e820e {
	uint64_t addr;
	uint64_t len;
	uint32_t type;
	uint32_t acpi;
} __packed *p;

void serial_init(void)
{
	outb(SERIAL_PORT + 1, 0);
	outb(SERIAL_PORT + 3, 0x80);
	outb(SERIAL_PORT + 0, 3);
	outb(SERIAL_PORT + 1, 0);
	outb(SERIAL_PORT + 3, 3);
	outb(SERIAL_PORT + 2, 0xc7);
	outb(SERIAL_PORT + 4, 0xb);
}

void serial_putc(int c)
{
	while (!(inb(SERIAL_PORT + 5) & 0x20));
	outb(SERIAL_PORT, c);
}

void _boot_putc_init(void)
{
	b800_putcinit();
#ifdef SERIAL_PUTC
	serial_init();
#endif
}

void _boot_putc(int ch)
{
	b800_putc(ch);
#ifdef SERIAL_PUTC
	serial_putc(ch);
#endif
}

void _boot_sysputc(int ch)
{
	b800_sysputc(ch);
#ifdef SERIAL_PUTC
	serial_putc(ch);
#endif
}

void platform_init(void)
{
#ifdef SERIAL_PUTC
	serial_init();
#endif
	pic_off();
	acpi_findrootptr();
	acpi_init();
	gsi_dump();
	hpet_init();
}

#define E820_START       ((struct e820e *)UKERN_BSMAP16)
#define E820_FOREACH(_p)				\
    for ((_p) = E820_START; p->type || p->len || p->addr; p++)

void sysboot(void)
{
	void *lpfndb;
	unsigned i, cpuid;
	uint32_t pfn_maxmem = 0;
	uint32_t pfn_maxaddr = 0xffffffff >> PAGE_SHIFT;	/* MMIO arrives up there */

	printf("Welcome!\n");
	if (_boot_cmdline != NULL)
		printf("\ncmdline: %s\n\n", _boot_cmdline);

	/* Search for maxpfns */
	E820_FOREACH(p) {
		uint64_t lpfn;
		lpfn = (p->addr + p->len) >> PAGE_SHIFT;

		printf("\t%016llx:%016llx -> %10s(%ld)\n",
		       p->addr, p->len,
		       p->type == 1 ? "Memory" : "Reserved",
		       (long) p->type);

		if (lpfn > pfn_maxaddr)
			pfn_maxaddr = lpfn;

		if (p->type == 1 && lpfn > pfn_maxmem)
			pfn_maxmem = lpfn;
	}
	printf("max PFNs: mem %8x addr %8x\n", pfn_maxmem, pfn_maxaddr);

	/* Initialise PFNDB */
	lpfndb = pfndb_setup((void *) UKERN_PFNDB, pfn_maxaddr);
	E820_FOREACH(p) {
		int t = p->type == 1 ? PFNT_FREE : PFNT_MMIO;
		int pfn = atop(trunc_page(p->addr));
		int lpfn = atop(round_page(p->addr + p->len));

		for (; pfn < lpfn; pfn++)
			pfndb_add(pfn, t);
	}

	/* No need to mark 0 page as SYSTEM, given all there's in it */

	/* Mark holes as MMIO */
	pfndb_subst(PFNT_INVALID, PFNT_MMIO);

	/* Mark kernel as SYSTEM */
	for (i = 0; i <= atop(round_page(UKERNEND - UKERNBASE)); i++)
		pfndb_add(i, PFNT_SYSTEM);

	/* Mark PFNDB as SYSTEM */
	for (i = atop(UKERN_PFNDB - UKERNBASE);
	     i <= atop(trunc_page((unsigned long) lpfndb - UKERNBASE));
	     i++)
		pfndb_add(i, PFNT_SYSTEM);

	pginit();
	fixmems_init();
	pmap_init();

	printf("Booting...\n");
	pmap_boot();

	heap_init();
	vmap_init();
	vmap_free(KVA_SVMAP, VMAPSIZE);

	platform_init();
	__insn_barrier();	/* LAPIC now mapped */

	/* Finish up initialization quickly.
	   We can now setup per-cpu data. */
	cpuid = cpu_number_from_lapic();
	_load_segs(cpuid, &cpuinfo_get(cpuid)->tss,
		   &cpuinfo_get(cpuid)->self);
	__insn_barrier();	/* FS: now valid */

	kern_boot();
}

void sysboot_ap(void)
{
	unsigned cpuid;

	pmap_boot();
	__insn_barrier();	/* LAPIC now mapped */
	/* Enable LAPIC */
	lapic_configure();
	cpuid = cpu_number_from_lapic();
	_load_segs(cpuid, &cpuinfo_get(cpuid)->tss,
		   &cpuinfo_get(cpuid)->self);
	__insn_barrier();	/* FS: now valid */
	printf("CPU %d on.\n", cpu_number());

	kern_bootap();
}
