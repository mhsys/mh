/*
 * Copyright (c) 2015-2016, Gianluca Guida
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


#ifndef __mrg_runtime_h
#define __mrg_runtime_h

#include <sys/types.h>
#include <inttypes.h>
#include <stdarg.h>
#include <errno.h>
#include <squoze.h>


/*
 * VM Memory Management
 */

#define VFNT_INVALID 0
#define VFNT_FREE    1
#define VFNT_RODATA  2
#define VFNT_RWDATA  3
#define VFNT_EXEC    4
#define VFNT_WREXEC  5
#define VFNT_MMIO    6
#define VFNT_MEM32   7

vaddr_t vmap_alloc(size_t size, uint8_t type);
void vmap_free(vaddr_t va, size_t size);
void vmap_info(vaddr_t va, vaddr_t * start, size_t * size, uint8_t * type);

int brk(void *);
void *sbrk(int);

void *dma32_alloc(size_t len);
void dma32_free(void *vaddr, size_t len);


/*
 * Interrupt allocation and handling.
 */

unsigned intalloc(void);
void intfree(unsigned);
void inthandler(unsigned, void (*)(int, void *), void *);
#include "mrg_preempt.h"


/*
 * Lightweight Threads.
 */

#include "mrg_lwt.h"


/*
 * Event handling.
 */

int evtalloc(void);
void evtwait(int evt);
void evtclear(int evt);
void evtast(int evt, void (*func) (void *), void *);
void evtset(int evt);
void evtfree(int evt);
void __evtset(int evt);


/*
 * Device handling.
 */

struct _DEVICE;
typedef struct _DEVICE DEVICE;

struct dinfo {
	uint64_t nameid;
	uint64_t vendorid;
	uint64_t *devids;

	unsigned ndevids;
	unsigned nirqs;
	unsigned npios;
	unsigned nmemsegs;
};

enum dio_op {
	PORT_IN,
	PORT_OUT,
};

DEVICE *dopen(char *devname);
int din(DEVICE * d, uint32_t port, uint64_t * val);
int dout(DEVICE * d, uint32_t port, uint64_t val);
int dmapirq(DEVICE * d, unsigned irq, int evt);
int dgetirq(DEVICE * d, int irqno);
int dgetpio(DEVICE * d, int piono);
ssize_t dgetmemrange(DEVICE * d, unsigned rangeno, uint64_t * base);
int dgetinfo(DEVICE * d, struct dinfo *i);
int drdcfg(DEVICE * d, unsigned off, uint8_t sz, uint64_t *val);
int dwrcfg(DEVICE *d, unsigned off, uint8_t sz, uint64_t val);
void *diomap(DEVICE * d, uint64_t base, size_t len);
int dexport(DEVICE * d, void *vaddr, size_t sz, iova_t *iova);
int dunexport(DEVICE * d, void *vaddr);
void dclose(DEVICE * d);


/*
 * Devices.
 */

int devcreat(struct sys_creat_cfg *cfg, devmode_t mode, int evt);
int devpoll(unsigned did, struct sys_poll_ior *ior);
int devwriospace(unsigned did, unsigned id, uint32_t port, uint64_t val);
int devraiseirq(unsigned did, unsigned id, unsigned irq);
int devread(unsigned did, unsigned id, u_long iova, size_t sz, void *va);
int devwrite(unsigned did, unsigned id, void *va, size_t sz, u_long iova);
#endif
