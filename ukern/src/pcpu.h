#ifndef __pcpu_h
#define __pcpu_h

/*
 * pCPU Interface.
 *
 * This interface abstracts low-level CPU operations. MD part needs to
 * implement 'struct pcpu'.
 */

#include <machine/uk/pcpu.h>

unsigned pcpu_init(void);
void pcpu_wake(unsigned pcpuid);
void pcpu_setup(struct pcpu *, void *data);
void pcpu_nmi(int pcpuid);
void pcpu_nmiall(void);
void pcpu_ipi(int pcpuid, unsigned vct);
void pcpu_ipiall(unsigned vct);

void *pcpu_getdata(void);

#endif
