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
#include <uk/heap.h>
#include <uk/vmap.h>
#include "i386.h"
#include <lib/lib.h>
#include <machine/uk/cpu.h>
#include <machine/uk/apic.h>

void *lapic_base = NULL;
unsigned lapics_no;
struct lapic_desc {
	unsigned physid;
	unsigned platformid;
	uint32_t lint[2];
} *lapics;
unsigned ioapics_no;
struct ioapic_desc {
	void *base;
	unsigned irq;
	unsigned pins;
} *ioapics;
unsigned gsis_no;
struct gsi_desc {
	unsigned irq;
	enum gsimode mode;
	unsigned ioapic;
	unsigned pin;
} *gsis;




/* Memory Registers */
#define IO_REGSEL 0x00
#define IO_WIN    0x10

/* I/O Registers */
#define IO_ID		0x00
#define IO_VER		0x01
#define IO_ARB		0x02
#define IO_RED_LO(x) 	(0x10 + 2*(x))
#define IO_RED_HI(x)	(0x11 + 2*(x))

void lapic_init(paddr_t base, unsigned no)
{
	lapic_base = kvmap(base, LAPIC_SIZE);
	lapics = heap_alloc(sizeof(struct lapic_desc) * no);
	lapics_no = no;
	dprintf("LAPIC PA: %08llx VA: %p\n", base, lapic_base);
}

void lapic_add(uint16_t physid, uint16_t plid)
{
	static unsigned i = 0;

	lapics[i].physid = physid;
	lapics[i].platformid = plid;
	lapics[i].lint[0] = 0x10000;
	lapics[i].lint[1] = 0x10000;
	i++;
}

void lapic_add_nmi(uint8_t pid, int l)
{
	int i;

	if (pid == 0xff) {
		for (i = 0; i < lapics_no; i++)
			lapics[i].lint[l] =
				(1L << 16) | (APIC_DLVR_NMI << 8);
		return;
	}
	for (i = 0; i < lapics_no; i++) {
		if (lapics[i].platformid == pid) {
			if (l)
				l = 1;
			lapics[i].lint[l] =
				(APIC_DLVR_NMI << 8);
			return;
		}
	}
	printf("Warning: LAPIC NMI for non-existing platform ID %d\n",
	       pid);
}

void lapic_configure(void)
{
	unsigned i, physid = lapic_getcurrent();
	struct lapic_desc *d = NULL;

	for (i = 0; i < lapics_no; i++) {
		if (lapics[i].physid == physid)
			d = lapics + i;
	}
	if (d == NULL) {
		printf("Warning: Current CPU not in Platform Tables!\n");
		/* Try to continue, ignore the NMI configuration */
	} else {
		lapic_write(L_LVT_LINT(0), d->lint[0]);
		lapic_write(L_LVT_LINT(1), d->lint[1]);
	}
	/* Enable LAPIC */
	lapic_write(L_MISC, lapic_read(L_MISC) | 0x100);
}

void lapic_platform_done(void)
{
	int i;

	for (i = 0; i < lapics_no; i++)
		cpu_add(lapics[i].physid, lapics[i].platformid);
	/* Since we're here, configure LAPIC of BSP */
	lapic_configure();
}

void ioapic_init(unsigned no)
{

	ioapics = heap_alloc(sizeof(struct ioapic_desc) * no);
	ioapics_no = no;
}

static void ioapic_write(unsigned i, uint8_t reg, uint32_t val)
{
	volatile uint32_t *regsel =
		(uint32_t *) (ioapics[i].base + IO_REGSEL);
	volatile uint32_t *win = (uint32_t *) (ioapics[i].base + IO_WIN);

	*regsel = reg;
	*win = val;
}

static uint32_t ioapic_read(unsigned i, uint8_t reg)
{
	volatile uint32_t *regsel =
		(uint32_t *) (ioapics[i].base + IO_REGSEL);
	volatile uint32_t *win = (uint32_t *) (ioapics[i].base + IO_WIN);

	*regsel = reg;
	return *win;
}

void ioapic_add(unsigned num, paddr_t base, unsigned irqbase)
{
	unsigned i;

	ioapics[num].base = kvmap(base, IOAPIC_SIZE);
	ioapics[num].irq = irqbase;
	ioapics[num].pins = 1 + ((ioapic_read(num, IO_VER) >> 16) & 0xff);

	/* Mask all interrupts */
	for (i = 0; i < ioapics[num].pins; i++) {
		ioapic_write(num, IO_RED_LO(i), 0x00010000);
		ioapic_write(num, IO_RED_HI(i), 0x00000000);
	}
	printf("IOAPIC ID: %02d PA: %08llx VA: %p IRQ:%02d PINS: %02d\n",
	       num, base, ioapics[num].base, irqbase, ioapics[num].pins);
}

unsigned ioapic_irqs(void)
{
	unsigned i;
	unsigned lirq, maxirq = 0;

	for (i = 0; i < ioapics_no; i++) {
		lirq = ioapics[i].irq + ioapics[i].pins;
		if (lirq > maxirq)
			maxirq = lirq;
	}
	return maxirq;
}

void ioapic_platform_done(void)
{
}

void gsi_init(void)
{
	unsigned i, irqs = ioapic_irqs();
	gsis_no = irqs;
	gsis = heap_alloc(sizeof(struct gsi_desc) * irqs);

	/* Setup identity map, edge triggered (this is ISA) */
	for (i = 0; i < gsis_no; i++) {
		gsis[i].mode = EDGE;
		gsis[i].irq = i;
	}
}

void gsi_setup(unsigned i, unsigned irq, enum gsimode mode)
{
	if (i >= gsis_no) {
		printf("Warning: GSI %d bigger than existing I/O APIC GSIs\n", i);
		return;
	}
	gsis[i].irq = irq;
	gsis[i].mode = mode;
}

static void irqresolve(unsigned gsi)
{
	unsigned irq = gsis[gsi].irq;
	unsigned i, start, end;

	for (i = 0; i < ioapics_no; i++) {
		start = ioapics[i].irq;
		end = start + ioapics[i].pins;

		if ((irq >= start) && (irq < end)) {
			gsis[gsi].ioapic = i;
			gsis[gsi].pin = irq - start;
			return;
		}
	}
	printf("Hello Impossiblity, I was waiting for you!\n");
	asm volatile ("ud2;");
}

void gsi_setup_done(void)
{
	unsigned i;
	uint32_t hi, lo;

	lo = 0;			/* No vector */
	lo |= 1L << 8;		/* Lowest Priority */
	lo |= 1L << 16;		/* Masked */
	hi = 0xffL << 24;	/* All Processors */

	for (i = 0; i < gsis_no; i++) {
		/* Now that we have the proper GSI to IRQ mapping, resolve the
		 * IOAPIC/PIN of the GSI. */
		irqresolve(i);

		/* Setup Masked IOAPIC entry with no vector information */
		switch (gsis[i].mode) {
		default:
			printf("Warning: GSI table corrupted. "
			       "Setting GSI %d to EDGE\n", i);
		case EDGE:
			break;
		case LVLHI:
			lo |= (1L << 15);
			break;
		case LVLLO:
			lo |= ((1L << 15) | (1L << 13));
			break;
		}

		ioapic_write(gsis[i].ioapic, IO_RED_LO(gsis[i].pin), lo);
		ioapic_write(gsis[i].ioapic, IO_RED_HI(gsis[i].pin), hi);
	}
}

void gsi_dump(void)
{
	unsigned i;

	for (i = 0; i < gsis_no; i++) {
		printf("GSI: %02d IRQ: %02d MODE: %5s APIC: %02d PIN: %02d\n",
		       i, gsis[i].irq,
		       gsis[i].mode == EDGE ? "EDGE"
		       : gsis[i].mode == LVLHI ? "LVLHI" : "LVLLO",
		       gsis[i].ioapic, gsis[i].pin);
	}
}

void gsi_register(unsigned gsi, unsigned vect)
{
	uint32_t lo;

	assert(gsi < gsis_no);
	assert(vect < 256);

	printf("Registering GSI %d to vect %d\n", gsi, vect);
	lo = ioapic_read(gsis[gsi].ioapic, IO_RED_LO(gsis[gsi].pin));
	ioapic_write(gsis[gsi].ioapic, IO_RED_LO(gsis[gsi].pin),
		     lo | vect);
	printf("GIANL: %02d: [%d/%d] = %08x\n", gsi, gsis[gsi].ioapic,
	       gsis[gsi].pin, lo | vect);
}

void gsi_enable(unsigned gsi)
{
	uint32_t lo;

	assert(gsi < gsis_no);
	lo = ioapic_read(gsis[gsi].ioapic, IO_RED_LO(gsis[gsi].pin));
	lo &= ~0x10000L;	/* UNMASK */
	ioapic_write(gsis[gsi].ioapic, IO_RED_LO(gsis[gsi].pin), lo);;
}

void gsi_disable(unsigned gsi)
{
	uint32_t lo;

	assert(gsi < gsis_no);
	lo = ioapic_read(gsis[gsi].ioapic, IO_RED_LO(gsis[gsi].pin));
	lo |= 0x10000L;		/* MASK */
	ioapic_write(gsis[gsi].ioapic, IO_RED_LO(gsis[gsi].pin), lo);;
}