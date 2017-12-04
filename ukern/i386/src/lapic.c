#include <uk/types.h>
#include <uk/logio.h>
#include <uk/heap.h>
#include <uk/vmap.h>
#include <uk/cpu.h>
#include "i386.h"
#include "lapic.h"
#include "param.h"

void *lapic_base = NULL;
unsigned lapics_no;
struct lapic_desc {
	unsigned physid;
	unsigned platformid;
	uint32_t lint[2];
} *lapics;


/*
 * Local APIC.
 */

#define APIC_DLVR_FIX   0
#define APIC_DLVR_PRIO  1
#define APIC_DLVR_SMI   2
#define APIC_DLVR_NMI   4
#define APIC_DLVR_INIT  5
#define APIC_DLVR_START 6
#define APIC_DLVR_EINT  7

/* LAPIC registers.  */
#define L_IDREG		0x20
#define L_VER		0x30
#define L_TSKPRIO	0x80
#define L_ARBPRIO	0x90
#define L_PROCPRIO	0xa0
#define L_EOI		0xb0
#define L_LOGDEST	0xd0
#define L_DESTFMT	0xe0
#define L_MISC		0xf0	/* Spurious vector */
#define L_ISR		0x100	/* 256 bit */
#define L_TMR		0x180	/* 256 bit */
#define L_IRR		0x200	/* 256 bit */
#define L_ERR		0x280
#define L_ICR_LO	0x300
#define L_ICR_HI	0x310
#define L_LVT_TIMER	0x320
#define L_LVT_THERM	0x330
#define L_LVT_PFMCNT	0x340
#define L_LVT_LINT(x)	(0x350 + (x * 0x10))
#define L_LVT_ERR	0x370
#define L_TMR_IC	0x380
#define L_TMR_CC	0x390
#define L_TMR_DIV	0x3e0

#define LAPIC_SIZE      (1UL << 12)

static uint32_t lapic_read(unsigned reg)
{

	return *((volatile uint32_t *) (lapic_base + reg));
}

static void lapic_write(unsigned reg, uint32_t data)
{

	*((volatile uint32_t *) (lapic_base + reg)) = data;
}

unsigned lapic_getcurrent(void)
{

	return (unsigned)(lapic_read(L_IDREG) >> 24);
}

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
			lapics[i].lint[l] = (APIC_DLVR_NMI << 8);
			return;
		}
	}
	printf("Warning: LAPIC NMI for non-existing platform ID %d\n",
	       pid);
}

void lapic_platform_done(void)
{
	int i;

	for (i = 0; i < lapics_no; i++)
		cpu_add(lapics[i].physid, lapics[i].platformid);
}

static void lapic_configure(void)
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

static void lapic_ipi(unsigned physid, uint8_t dlvr, uint8_t vct)
{
	uint32_t hi, lo;

	lo = 0x4000 | (dlvr & 0x7) << 8 | vct;
	hi = (physid & 0xff) << 24;

	lapic_write(L_ICR_HI, hi);
	lapic_write(L_ICR_LO, lo);
}

static void lapic_ipi_broadcast(uint8_t dlvr, uint8_t vct)
{
	uint32_t lo;

	lo = (dlvr & 0x7) << 8 | vct
		| /*ALLBUTSELF*/ 0xc0000 | /*ASSERT*/ 0x4000;
	lapic_write(L_ICR_HI, 0);
	lapic_write(L_ICR_LO, lo);
}

void lapic_eoi(void)
{
	lapic_write(L_EOI, 0);
}


/*
 * PCPU module: abstracted CPU operations.
 */

unsigned pcpu_init(void)
{
	lapic_configure();
	return lapic_getcurrent();
}

void pcpu_setup(struct pcpu *pcpu, void *data)
{
	unsigned pcpuid;
	void _set_tss(unsigned, void *);
	void _set_fs(unsigned, void *);

	pcpuid = lapic_getcurrent();
	pcpu->tss.iomap = 108;
	pcpu->data = data;
	_set_tss(pcpuid, &pcpu->tss);
	_set_fs(pcpuid, &pcpu->data);
}

void pcpu_nmi(int pcpuid)
{
	lapic_ipi(pcpuid, APIC_DLVR_NMI, 0);
}

void pcpu_nmiall(void)
{
	lapic_ipi_broadcast(APIC_DLVR_NMI, 0);
}

void pcpu_ipi(int pcpuid, unsigned vct)
{
	lapic_ipi(pcpuid, APIC_DLVR_FIX, vct);
}

void pcpu_ipiall(unsigned vct)
{
	lapic_ipi_broadcast(APIC_DLVR_FIX, vct);
}

void *pcpu_getdata(void)
{
	void *ptr;

	asm volatile ("movl %%fs:0, %0\n":"=r" (ptr));
	return ptr;
}

void pcpu_wake(unsigned pcpuid)
{
	extern void _ap_start, _ap_end;

	cmos_write(0xf, 0xa);	// Shutdown causes warm reset.

	if (pcpuid == lapic_getcurrent())
		return;

	/* Setup AP bootstrap page */
	memcpy((void *) (UKERNBASE + UKERN_APBOOT(pcpuid)), &_ap_start,
	       (size_t) & _ap_end - (size_t) & _ap_start);

	/* Setup Warm Reset Vector */
	*(volatile uint16_t *) (UKERNBASE + 0x467) = UKERN_APBOOT(pcpuid) & 0xf;
	*(volatile uint16_t *) (UKERNBASE + 0x469) = UKERN_APBOOT(pcpuid) >> 4;

	/* INIT-SIPI-SIPI sequence. */
	lapic_ipi(pcpuid, APIC_DLVR_INIT, 0);
	_delay();
	lapic_ipi(pcpuid, APIC_DLVR_START,
		  UKERN_APBOOT(pcpuid) >> 12);
	_delay();
	lapic_ipi(pcpuid, APIC_DLVR_START,
		  UKERN_APBOOT(pcpuid) >> 12);
	_delay();
}
