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

static int sys_iret(void)
{
	struct thread *th = current_thread();

	return usrframe_iret(th->frame);
}

static int sys_sti(void)
{
	struct thread *th = current_thread();

	th->userfl |= THFL_INTR;
	return 0;
}

static int sys_cli(void)
{
	struct thread *th = current_thread();

	th->userfl &= ~THFL_INTR;
	return 0;
}

static int sys_wait(void)
{
	struct thread *th = current_thread();

	printf("Sleeping");
	/* Can't sleep with Interrupts disabled */
	th->userfl |= THFL_INTR;
	schedule(THST_STOPPED);
	printf("Woken");
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

static int sys_fork(void)
{
	struct thread *th;

	th = thfork();
	return !!th;
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
	case SYS_IRET:
		return sys_iret();
	case SYS_STI:
		return sys_sti();
	case SYS_CLI:
		return sys_cli();
	case SYS_WAIT:
		return sys_wait();
	case SYS_MAP:
		return sys_map(a1, a2);
	case SYS_FORK:
		return sys_fork();
	case SYS_DIE:
		return sys_die();
	default:
		return -1;
	}
}
