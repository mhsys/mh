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


#include <uk/types.h>
#include <uk/string.h>
#include <uk/logio.h>
#include <uk/assert.h>
#include <uk/locks.h>
#include <uk/bus.h>
#include <uk/sys.h>
#include <uk/kern.h>
#include <uk/cpu.h>

/*
 * Kernel Log Ring Buffer.
 */

#define KLOGBUFSZ (64*1024)
static lock_t kbuflock;
static uint8_t klogbuf[KLOGBUFSZ];
static volatile unsigned _c = 0;
static volatile unsigned _p = 0;
static size_t logsize = 0;

#define seq(_v) (((_v) + 1) % KLOGBUFSZ)

static void __klogdev_raise_avl();

void klog_putc(int ch)
{
	uint8_t v;
	unsigned c = _c;
	unsigned p = _p;
	unsigned s_p = seq(p);

	spinlock(&kbuflock);
	if (s_p != c) {
		klogbuf[p] = ch;
		_p = s_p;
		logsize++;
	}
	spinunlock(&kbuflock);
	__klogdev_raise_avl();
}

static size_t _klog_size(void)
{
	size_t sz;

	spinlock(&kbuflock);
	sz = logsize;
	spinunlock(&kbuflock);
	return sz;
}

static uint8_t _klog_getc(void)
{
	uint8_t ch;
	unsigned c = _c;
	unsigned p = _p;

	spinlock(&kbuflock);
	if (c != p) {
		ch = klogbuf[c];
		_c = seq(c);
		logsize--;
	}
	spinunlock(&kbuflock);

	return ch;
}


/*
 * KLOGDEV device.
 */

#define KLOGDEV_OWNER 0

#define KLOGDEV_NAMEID squoze("KLOG")
#define KLOGDEV_VENDORID squoze("MHSYS")

static lock_t kloglock = 0;
static int avl_isr = 0;
static int avl_ie = 0;
static int avl_sig = 0;
static int bsy = 0;
static struct thread *klogth = NULL;
static struct dev klog_dev;

static void __klogdev_raise_avl(void)
{
	spinlock(&kloglock);
	if (avl_ie && !avl_isr && avl_sig) {
		assert(klogth != NULL);
		thraise(klogth, avl_sig);
	}
	avl_isr = 1;
	spinunlock(&kloglock);
}

static int _klogdev_open(void *devopq, uint64_t did)
{
	struct thread *th = current_thread();

	spinlock(&kloglock);
	if (bsy) {
		spinunlock(&kloglock);
		printf("klogdev: busy\n");
		return -EBUSY;
	}
	klogth = th;
	avl_ie = 0;
	avl_isr = 0;
	avl_sig = 0;
	bsy = 1;
	spinunlock(&kloglock);
	return 0;
}


static void _klogdev_close(void *devopq, unsigned id)
{
	spinlock(&kloglock);
	assert(bsy);
	bsy = 0;
	avl_sig = 0;
	avl_isr = 0;
	avl_ie = 0;
	klogth = NULL;
	spinunlock(&kloglock);
}


static int _klogdev_in(void *devopq, unsigned id, uint32_t port,
		      uint64_t * val)
{
	uint8_t iosize = port & IOPORT_SIZEMASK;
	uint16_t ioport = port >> IOPORT_SIZESHIFT;
	uint64_t ioval;

	switch (ioport) {
	case KLOGDEVIO_GETC:
		ioval = 0;
		iosize = 1 << iosize;
		while (iosize--) {
			ioval <<= 8;
			ioval |= _klog_getc();
		}
		break;
	case KLOGDEVIO_SZ:
		ioval = _klog_size();
		break;
	default:
		ioval = -1;
		break;
	}

	*val = ioval;
	return 0;
}

static int _klogdev_out(void *devopq, unsigned id, uint32_t port,
			uint64_t val)
{
	uint8_t iosize = port & IOPORT_SIZEMASK;
	uint16_t ioport = port >> IOPORT_SIZESHIFT;

	switch (ioport) {
	case KLOGDEVIO_IE:
		spinlock(&kloglock);
		if (val & 1) {
			avl_ie = 1;
		} else {
			avl_ie = 0;
		}
		spinunlock(&kloglock);
		break;
	case KLOGDEVIO_ISR:
		spinlock(&kloglock);
		if (val & 1) {
			avl_isr = 0;
		}
		spinunlock(&kloglock);
		break;
	default:
		break;
	}
}

static int _klogdev_export(void *devopq, unsigned id, vaddr_t va,
			  unsigned long *iopfnptr)
{
	return -ENOSYS;
}

static int _klogdev_info(void *devopq, unsigned id,
			 struct sys_info_cfg *cfg)
{
	cfg->nameid = KLOGDEV_NAMEID;
	cfg->vendorid = KLOGDEV_VENDORID;

	memset(cfg->deviceids, 0, sizeof(cfg->deviceids));
	cfg->niopfns = -1;
	cfg->nirqsegs = -1;
	cfg->npiosegs = -1;
	cfg->nmemsegs = -1;
	return 0;
}

static int _klogdev_irqmap(void *devopq, unsigned id, unsigned irq,
			  unsigned sig)
{
	struct thread *th = current_thread();

	switch (irq) {
	case KLOGDEVIO_AVLINT:
		avl_sig = sig;
		break;
	default:
		break;
	}
	return 0;
}

static int _klogdev_iomap(void *devopq, unsigned id, vaddr_t va,
			 paddr_t mmioaddr, pmap_prot_t prot)
{
	return -ENOSYS;
}

static int _klogdev_iounmap(void *devopq, unsigned id, vaddr_t va)
{
	return -ENOSYS;
}

static struct devops klogdev_ops = {
	.open = _klogdev_open,
	.close = _klogdev_close,
	.in = _klogdev_in,
	.out = _klogdev_out,
	.export = _klogdev_export,
	.iomap = _klogdev_iomap,
	.iounmap = _klogdev_iounmap,
	.info = _klogdev_info,
	.irqmap = _klogdev_irqmap,
};

void klogdev_init(void)
{
	dev_init(&klog_dev, KLOGDEV_NAMEID, NULL, &klogdev_ops,
		 KLOGDEV_OWNER, 0, 0111);
	dev_attach(&klog_dev);
}
