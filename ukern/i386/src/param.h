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


#ifndef __i386_param_h
#define __i386_param_h

#ifdef _UKERNEL

#ifdef _ASSEMBLER
#define __ULONG(_c) _c
#else
#define __ULONG(_c) ((unsigned long)(_c))
#endif

#include <machine/uk/vmparam.h>

#define MAXCPUS         32

/* i386 Virtual Address Space Map:
 *
 * 00000000:b7ffffff     User mappings
 * c0000000:UKERNEND     Kernel code and data
 * c0000000:DMAP_END     Kernel 1:1 direct mapping
 * DMAP_END:ffe00000     Kernel VA allocation range
 */

#define USERBASE  __ULONG(0x00000000)
#define NOCOWBASE __ULONG(0x00000000)
#define NOCOWEND  __ULONG(2 * PAGE_SIZE)
#define USEREND   __paeoffva(2, LINOFF, 0)
#define UKERNLIN  USEREND
#define UKERNBASE __ULONG(0xc0000000)
#define UKERNEND  _ukern_end
#define DMAPSIZE  __ULONG(768 << 20)
#define KVA_SDMAP __ULONG(UKERNBASE)
#define KVA_EDMAP __ULONG(UKERNBASE + DMAPSIZE)
#define KVA_SVMAP __ULONG(KVA_EDMAP)
#define KVA_EVMAP __ULONG(0xffe00000)
#define VMAPSIZE  __ULONG(KVA_EVMAP - KVA_SVMAP)

#define UKERNCMPOFF     0x100000	/* 1Mb */
#define UKERNTEXTOFF    (UKERNBASE + UKERNCMPOFF)

/*
 * Physical address ranges:
 *
 * 00000000:00100000     Boot-time allocation memory (640k)
 *                       + BIOS area
 * 00100000:_end         Kernel code + data
 * _end    :LKMEMEND     Low 1:1 Kernel memory
 * KMEMSTRT:KMEMEND      1:1 Kernel memory
 * HIGHSTRT:HIGHEND      High memory, accessed through mapping
 */

#define LKMEMSTRT        0
#define LKMEMEND         __ULONG(16 << 20)
#define KMEMSTRT         LKMEMEND
#define KMEMEND          DMAPSIZE
#define HIGHSTRT         DMAPSIZE

/* Temporary physical boot-time allocation physical address */
#define UKERN_BCODE16   0x10000	/* 16bit code */
#define UKERN_BSTCK16   0x2fffe	/* 16bit stack */
#define UKERN_BSMAP16   0x30000	/* E820 map */

#define UKERN_BL3TABLE  0x50000	/* Temporary L3 table */
#define UKERN_BL2TABLE  0x51000	/* Temporary L2 table */
#define UKERN_BGDTREG   0x52000	/* Temporary GDT reg */
#define UKERN_BGDTABLE  0x52030	/* Temporary GDT */

#define UKERN_APBOOTPG  0x60000	/* AP stack */
#define UKERN_APBOOT(x) (UKERN_APBOOTPG + ((x) * 4096))

/* Non-temporary boot-time static memory allocation */
#define UKERN_PFNDB     (((UKERNEND + 127) >> 7) << 7)


#define KCS 0x08
#define KDS 0x10
#define UCS 0x1b
#define UDS 0x23

/* Boot time 16 bit segs */
#define CS16  0x18
#define RDS16 0x20

#define SEG16_ADDR(_a) (((_a) >> 4) & 0xf000)
#define OFF16_ADDR(_a) ((_a) & 0xffff)

#ifndef _ASSEMBLER
extern unsigned long _ukern_end;
#endif

#endif				/* _UKERNEL */

#endif
