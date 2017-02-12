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

#include <uk/errno.h>
#include <uk/types.h>
#include <uk/bus.h>
#include <uk/sys.h>
#include <uk/kern.h>
#include <machine/uk/cpu.h>
#include <machine/uk/machdep.h>
#include <lib/lib.h>

#define SYSDEV_OWNER 0

#define SYSTEM_NAMEID squoze("SYSTEM")
#define SYSTEM_VENDORID squoze("MHSYS")

static struct dev system_dev;

static int _sysdev_open(void *devopq, uint64_t did)
{
	dprintf("systemdev: opening");
	return 0;
}

static void _sysdev_close(void *devopq, unsigned id)
{
	return;
}

static int _sysdev_in(void *devopq, unsigned id, uint32_t port,
		      uint64_t * val)
{
	uint8_t iosize = 1 << (port & IOPORT_SIZEMASK);
	uint16_t ioport = port >> IOPORT_SIZESHIFT;
	struct thread *th = current_thread();
	uint64_t ioval;

	switch (ioport) {
	case SYSDEVIO_TMRCNT:
		ioval = timer_readcounter();
		break;
	case SYSDEVIO_TMRPRD:
		ioval = timer_readperiod();
		break;
	default:
		ioval = -1;
		break;
	}
	dprintf("Reading SYS port: %d = %" PRIx64 "\n", ioport, ioval);
	*val = ioval;
	return 0;
}

static int _sysdev_out(void *devopq, unsigned id, uint32_t port,
		       uint64_t val)
{
	struct thread *th = current_thread();
	uint8_t iosize = port & IOPORT_SIZEMASK;
	uint16_t ioport = port >> IOPORT_SIZESHIFT;

	switch (ioport) {
	case SYSDEVIO_ALMCNT:
		/* Adder is 32 bit */
		val = (uint32_t) val + timer_readcounter();
		thalrm(val);
		break;
	default:
		break;
	}
	dprintf("Writing SYS port: %d, val: %" PRIx64 "\n", ioport, val);
}

static int _sysdev_export(void *devopq, unsigned id, vaddr_t va,
			  unsigned long *iopfnptr)
{
	return -ENOSYS;
}

static int _sysdev_rdcfg(void *devopq, unsigned id,
			 struct sys_rdcfg_cfg *cfg)
{
	cfg->nameid = SYSTEM_NAMEID;
	cfg->vendorid = SYSTEM_VENDORID;

	memset(cfg->deviceids, 0, sizeof(cfg->deviceids));
	cfg->niopfns = -1;
	cfg->nirqsegs = -1;
	cfg->npiosegs = -1;
	cfg->nmemsegs = -1;
	return 0;
}

static int _sysdev_irqmap(void *devopq, unsigned id, unsigned irq,
			  unsigned sig)
{
	struct thread *th = current_thread();

	if (irq != 0)
		return -EINVAL;

	th->sysdev.alarm_sig = sig + 1;
	return 0;
}

static int _sysdev_iomap(void *devopq, unsigned id, vaddr_t va,
			 paddr_t mmioaddr, pmap_prot_t prot)
{
	return -ENOSYS;
}

static int _sysdev_iounmap(void *devopq, unsigned id, vaddr_t va)
{
	return -ENOSYS;
}

static struct devops sysdev_ops = {
	.open = _sysdev_open,
	.close = _sysdev_close,
	.in = _sysdev_in,
	.out = _sysdev_out,
	.export = _sysdev_export,
	.iomap = _sysdev_iomap,
	.iounmap = _sysdev_iounmap,
	.rdcfg = _sysdev_rdcfg,
	.irqmap = _sysdev_irqmap,
};

void sysdev_init(void)
{
	dev_init(&system_dev, SYSTEM_NAMEID, NULL, &sysdev_ops,
		 SYSDEV_OWNER, 0, 0111);
	dev_attach(&system_dev);
}
