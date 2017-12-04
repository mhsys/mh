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


#ifndef _i386_apic_h
#define _i386_apic_h

/*
 * I/O APIC
 */

#define IOAPIC_SIZE     (0x20)

extern unsigned ioapics_no;

void ioapic_init(unsigned no);
void ioapic_add(unsigned num, paddr_t base, unsigned irqbase);


/*
 * Global System Interrupts
 */

enum gsimode {
	EDGE,
	LVLHI,
	LVLLO,
};

void gsi_init(void);
void gsi_setup(unsigned i, unsigned irq, enum gsimode mode);
void gsi_setup_done(void);
void gsi_dump(void);
void gsi_set_irqtype(unsigned irq, enum gsimode mode);
int gsi_is_level(unsigned gsi);
enum gsimode gsi_get_irqtype(unsigned gsi);

void gsi_register(unsigned gsi, unsigned vect);
void gsi_enable(unsigned gsi);
void gsi_disable(unsigned gsi);

#endif
