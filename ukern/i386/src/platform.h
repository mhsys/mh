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


#ifndef _i386_platform_h
#define _i386_platform_h

#include "i386.h"

void platform_init(void);

static inline unsigned platform_inb(unsigned port)
{

	return inb(port);
}

static inline unsigned platform_inw(unsigned port)
{

	return inw(port);
}

static inline unsigned platform_inl(unsigned port)
{

	return inl(port);
}

static inline void platform_outb(unsigned port, unsigned val)
{
	outb(port, val);
}

static inline void platform_outw(unsigned port, unsigned val)
{
	outw(port, val);
}

static inline void platform_outl(unsigned port, unsigned val)
{
	outl(port, val);
}

static inline void platform_idle(void)
{

	asm volatile ("sti\n1: hlt\n jmp 1b\n");
}

static inline void platform_irqon(unsigned irq)
{

	/* 1:1 Mapping GSI-IRQ */
	gsi_enable(irq);
}

static inline void platform_irqoff(unsigned irq)
{
	gsi_disable(irq);
}

/* IRQ filter used only for PCI interrupts */
static inline int platform_irqfilter(uint32_t irqfilt)
{
	uint8_t bus, dev, fn;

	bus = (irqfilt >> 16) && 0xff;
	dev = (irqfilt >> 8) && 0xff;
	fn = irqfilt & 0xff;
	return pci_cfg_intsts(bus, dev, fn);
}

#endif
