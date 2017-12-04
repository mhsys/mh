/*
 * Copyright (c) 2017, Gianluca Guida
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

#include <uk/assert.h>
#include <uk/types.h>
#include <uk/logio.h>
#include <uk/string.h>
#include <uk/vmap.h>
#include <uk/kern.h>
#include <machine/uk/machdep.h>

#include "ioapic.h"
#include "i386.h"

/* ACPICA includes */
#include "acpi.h"

#define HPET_SIZE 0x1000

#define REG_GENCAP 0x00
#define LEG_RT_CAP (1L << 15)

#define REG_GENCFG 0x10
#define LEG_RT_CNF (1L << 1)
#define ENABLE_CNF (1L << 0)

#define REG_GENISR 0x20

#define REG_COUNTER 0xF0

#define REG_TMRCAP(_n) ((0x20 * (_n)) + 0x100)
#define INT_TYPE_CNF (1LL << 1)
#define INT_ENB_CNF (1LL << 2)
#define REG_TMRCMP(_n) ((0x20 * (_n)) + 0x108)

static volatile void *hpet_base;
static uint32_t period_femto;
static uint64_t gencfg;
static uint64_t tmrcfg;

#define TMR 0
static int irqlvl = 0;
static int irqno;
static struct irqsig hpetirq;

static uint64_t hpet_read(uint16_t offset)
{
	uint32_t hi1, hi2, lo;
	uint64_t ret;
	volatile uint32_t *ptr = hpet_base + offset;

	do {
		hi1 = *(ptr + 1);
		lo = *ptr;
		hi2 = *(ptr + 1);
	} while (hi1 != hi2);

	return (uint64_t) hi1 << 32 | lo;
}

static void hpet_write(uint16_t offset, uint64_t val)
{
	volatile uint32_t *ptr = hpet_base + offset;
	*(ptr + 1) = val >> 32;
	*ptr = (uint32_t) val;
}

static void hpet_pause(void)
{
	hpet_write(REG_GENCFG, gencfg);
}

static void hpet_resume(void)
{
	hpet_write(REG_GENCFG, ENABLE_CNF | gencfg);
}

#if 0
static void hpet_dump(void)
{
	dprintf("{%08x%08x,%08x%08x,%08x%08x,%08x%08x}",
		(uint32_t) (hpet_read(REG_GENCFG) >> 32),
		(uint32_t) (hpet_read(REG_GENCFG)),
		(uint32_t) (hpet_read(REG_COUNTER) >> 32),
		(uint32_t) (hpet_read(REG_COUNTER)),
		(uint32_t) (hpet_read(REG_TMRCAP(TMR)) >> 32),
		(uint32_t) (hpet_read(REG_TMRCAP(TMR))),
		(uint32_t) (hpet_read(REG_TMRCMP(TMR)) >> 32),
		(uint32_t) (hpet_read(REG_TMRCMP(TMR))));
}
#endif

void hpet_irqhandler(unsigned opq)
{
	assert(opq == 0);
	if (irqlvl)
		hpet_write(REG_GENISR, hpet_read(REG_GENISR) | 1);
	hpet_write(REG_TMRCAP(TMR), tmrcfg & ~INT_ENB_CNF);
	cpu_softirq_raise(SOFTIRQ_TIMER);
}

void hpet_init(void)
{
	ACPI_STATUS r;
	ACPI_TABLE_HPET *tbl;
	uint64_t gencap;
	int i, num;

	r = AcpiGetTable("HPET", 1, (ACPI_TABLE_HEADER **) & tbl);
	if (r != AE_OK) {
		printf("No ACPI HPET table found");
		return;
	}

	hpet_base = kvmap(tbl->Address.Address, HPET_SIZE);
	gencap = hpet_read(REG_GENCAP);
	gencfg = 0;
	period_femto = gencap >> 32;
	num = 1 + (gencap >> 8) & 0xf;
	dprintf("HPET:\n");
	dprintf("\tfound at %" PRIx64 ", mapped at %p\n",
		tbl->Address.Address, hpet_base);
	dprintf("\tperiod:  %-20lx\n", period_femto);
	dprintf("\tcounters:%-20lx\n", num);

	hpet_pause();		/* Stop, in case it's running */

	/* Disable interrupts on all counters. */
	for (i = 0; i < num; i++) {
		hpet_write(REG_TMRCAP(i), 0);
	}

	tmrcfg = 0;

	/* Find IRQ of first counter. */
	if (gencap & LEG_RT_CAP && (TMR < 2)) {
		dprintf("HPET: using Legacy Routing.\n");
		gencfg |= LEG_RT_CNF;
		if (TMR == 0) {
			irqno = 2;
		} else {
			irqno = 8;
		}
	} else {
		uint32_t irqcap = hpet_read(REG_TMRCAP(TMR)) >> 32;
		if (irqcap == 0) {
			printf("HPET: no IRQ avaiable, can't use counter 0.\n");
			return;
		}
		irqno = ffs(irqcap) - 1;
		tmrcfg |= (irqno << 9);
		dprintf("HPET: using Interrupt Routing (%d - %x).\n",
			irqno, irqcap);
	}

	gsi_set_irqtype(irqno, LVLHI);

	if (gsi_get_irqtype(irqno) != EDGE) {
		dprintf("HPET: using level interrupt.\n");

		/* Reset ISR just in case. */
		hpet_write(REG_GENISR, hpet_read(REG_GENISR) | 1);
		irqlvl = 1;
		tmrcfg |= INT_TYPE_CNF;
	}

	/* Start Time of Boot. */
	hpet_write(REG_COUNTER, 0);

	/* Setup Counter 0 */
	hpet_write(REG_TMRCAP(TMR), tmrcfg);

	hpet_resume();		/* Enable HPET */

	/* Register timer's IRQ. */
	irqfnregister(&hpetirq, irqno, hpet_irqhandler, 0);
}

uint64_t timer_readcounter(void)
{
	return hpet_read(0xf0);
}

uint64_t timer_readperiod(void)
{
	return period_femto;
}

void timer_setcounter(uint64_t cnt)
{
	hpet_pause();
	hpet_write(0xf0, cnt);
	hpet_resume();
}

void timer_setalarm(uint64_t cnt)
{
	hpet_write(REG_TMRCMP(TMR), cnt);
	hpet_write(REG_TMRCAP(TMR), tmrcfg | INT_ENB_CNF);
}

void timer_disablealarm(void)
{
	hpet_write(REG_TMRCAP(TMR), tmrcfg & ~INT_ENB_CNF);
}
