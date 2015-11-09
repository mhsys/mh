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


#include <uk/sys.h>
#include <uk/kern.h>
#include <machine/uk/cpu.h>
#include <machine/uk/machdep.h>
#include <lib/lib.h>

static int sys_putc(int ch)
{
	sysputc(ch);
	return 0;
}

static int sys_inthdlr(vaddr_t ip, vaddr_t sp)
{
	struct thread *th = current_thread();

	if (!__inthdlr_chk(ip, sp)) {
		return -1;
	}

	th->sigip = ip;
	th->sigsp = sp;
	return 0;
}

static int sys_map(vaddr_t vaddr, sys_map_flags_t perm)
{
	int np, ret;
	pmap_prot_t prot;

	/* _probably_ should not kill it and just return an error,
	 * like a nice, respectful OS */
	procassert(__chkuaddr(trunc_page(vaddr), PAGE_SIZE));
	np = perm & MAP_NEW;
	perm &= ~MAP_NEW;

	switch (perm) {
	case MAP_RDONLY:
		prot = PROT_USER;
		break;
	case MAP_RDEXEC:
		prot = PROT_USER_X;
		break;
	case MAP_WRITE:
		prot = PROT_USER_WR;
		break;
	case MAP_WREXEC:
		prot = PROT_USER_WRX;
		break;
	case MAP_NONE:
		prot = 0;
		np = 0;
		break;
	default:
		printf("perm = %x\n", perm);
		return -1;
	}

	if (np)
		ret = vmmap(vaddr, prot);
	else if (!prot)
		ret = vmunmap(vaddr);
	else
		ret = vmchprot(vaddr, prot);
	return ret;
}

/* TEMPORARY */
static struct apt *self_apt = NULL;
/* TEMPORARY */
static int sys_aptcreat(pid_t dst, size_t sz)
{
	struct thread *th = current_thread();

	/* CHECK FOR PID (exists), check for permission to create(?) */
	/* Not only direct. */
	if (sz >= PAGE_SIZE * MAXAPTSIZE)
		return -1;

	self_apt = aptcreat(th->pmap, th->pmap, atop(sz));
	return 0;
}

/* TEMPORARY */
static int sys_aptexport(uaddr_t va, aptaddr_t aptva)
{
	/* _probably_ should not kill it and just return an error,
	 * like a nice, respectful OS */
	procassert(__chkuaddr(trunc_page(va), PAGE_SIZE));
	/* CHECK aptva */
	return aptexport(self_apt, va, aptva);
}

/* TEMPORARY */
static int sys_aptimport(aptaddr_t aptva, uaddr_t va)
{
	/* _probably_ should not kill it and just return an error,
	 * like a nice, respectful OS */
	procassert(__chkuaddr(trunc_page(va), PAGE_SIZE));
	/* CHECK aptva */
	return aptimport(self_apt, aptva, va);
}

static int sys_die(void)
{
	die();
	return 0;
}

int sys_call(int sc, unsigned long a1, unsigned long a2, unsigned long a3)
{
	switch (sc) {
	case SYS_PUTC:
		return sys_putc(a1);
	case SYS_INTHDLR:
		return sys_inthdlr(a1, a2);
	case SYS_MAP:
		return sys_map(a1, a2);
	case SYS_DIE:
		return sys_die();

		/* TEMPORARY */
	case SYS_APTCREAT:
		return sys_aptcreat(a1, a2);
	case SYS_APTIMPORT:
		return sys_aptimport(a1, a2);
	case SYS_APTEXPORT:
		return sys_aptexport(a1, a2);


	default:
		return -1;
	}
}
