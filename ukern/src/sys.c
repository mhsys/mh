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


#include <uk/types.h>
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

static int sys_inthdlr(uaddr_t ip, uaddr_t sp)
{
	struct thread *th = current_thread();

	if (!__inthdlr_chk(ip, sp)) {
		return -EACCES;
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

static int sys_fork(void)
{
	struct thread *th;

	th = thfork();
	return ! !th;
}

static int sys_creat(u_long id, unsigned sig, devmode_t mode)
{
	return devcreat(id, sig, mode);
}

static int sys_poll(uaddr_t uior)
{
	int id, ret;
	struct sys_poll_ior sior;

	id = devpoll(&sior);
	if (id < 0)
		return id;

	printf("sior.val = %" PRIx64 "\n", sior.val);
	ret = copy_to_user(uior, &sior, sizeof(struct sys_poll_ior));
	if (ret)
		return ret;
	return id;
}

static int sys_bootstrap(unsigned id, vaddr_t va, size_t sz)
{
	return devbootstrap(id, va, sz, /* mode */0);
}

static int sys_eio(unsigned id)
{
	return deveio(id);
}

static int sys_import(unsigned id, unsigned iopfn, u_long va)
{
	return devimport(id, iopfn, va);
}

static int sys_irq(unsigned id, unsigned irq)
{
	return devirq(id, irq);
}

static int sys_open(u_long id)
{
	return devopen(id);
}

static int sys_export(unsigned ddno, u_long va, unsigned iopfn)
{
	return devexport(ddno, va, iopfn);
}

static int sys_mapirq(unsigned ddno, unsigned id, unsigned sig)
{

	return devirqmap(ddno, id, sig);
}

static int sys_in(unsigned ddno, uint64_t port, uaddr_t valptr)
{
	int ret;
	uint64_t val;

	ret = devin(ddno, port, &val);
	if (ret)
		return ret;

	ret = copy_to_user(valptr, &val, sizeof(val));
	return ret;
}

static int sys_out(unsigned ddno, uint64_t port, uint64_t val)
{

	return devout(ddno, port, val);
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

static int sys_move(vaddr_t dst, vaddr_t src)
{
	procassert(__chkuaddr(trunc_page(dst), PAGE_SIZE));
	procassert(__chkuaddr(trunc_page(src), PAGE_SIZE));
	return vmmove(dst, src);
}

static int sys_yield(void)
{

	schedule(THST_RUNNABLE);
	return 0;
}

static int sys_die(void)
{
	die();
	return 0;
}

static uid_t sys_getuid(int sel)
{
	struct thread *th = current_thread();

	if (sel < 0)
		return th->suid;
	if (sel > 0)
		return th->euid;
	return th->ruid;
}

static gid_t sys_getgid(int sel)
{
	struct thread *th = current_thread();

	if (sel < 0)
		return th->sgid;
	if (sel > 0)
		return th->egid;
	return th->rgid;
}

static int sys_setuid(uid_t uid)
{
	struct thread *th = current_thread();

	/* Super User or Real User */
	if (th->euid && (th->ruid != uid))
		return -EPERM;

	th->ruid = uid;
	th->euid = uid;
	th->suid = uid;
	return 0;
}

static int sys_seteuid(uid_t uid)
{
	struct thread *th = current_thread();

	/* Super User or Real or Saved User */
	if (th->euid && (th->ruid != uid) && (th->suid != uid))
		return -EPERM;

	th->euid = uid;
	return 0;
}

static int sys_issetuid(void)
{
	return !!current_thread()->setuid;
}

static int sys_setgid(gid_t gid)
{
	struct thread *th = current_thread();

	/* Super User or Real Group */
	if (th->euid && (th->rgid != gid))
		return -EPERM;

	th->rgid = gid;
	th->egid = gid;
	th->sgid = gid;
	return 0;
}

int sys_setegid(gid_t gid)
{
	struct thread *th = current_thread();

	/* Super User or Real or Saved Group */
	if (th->euid && (th->rgid != gid) && (th->sgid != gid))
		return -EPERM;

	th->egid = gid;
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
	case SYS_YIELD:
		return sys_yield();
	case SYS_MAP:
		return sys_map(a1, a2);
	case SYS_MOVE:
		return sys_move(a1, a2);
	case SYS_FORK:
		return sys_fork();
	case SYS_DIE:
		return sys_die();
	case SYS_CREAT:
		return sys_creat(a1, a2, a3);
	case SYS_POLL:
		return sys_poll(a1);
	case SYS_BOOTSTRAP:
		return sys_bootstrap(a1, a2, a3);
	case SYS_EIO:
		return sys_eio(a1);
	case SYS_IMPORT:
		return sys_import(a1, a2, a3);
	case SYS_IRQ:
		return sys_irq(a1, a2);
	case SYS_OPEN:
		return sys_open(a1);
	case SYS_EXPORT:
		return sys_export(a1, a2, a3);
	case SYS_MAPIRQ:
		return sys_mapirq(a1, a2, a3);
	case SYS_IN:
		return sys_in(a1, a2, a3);
	case SYS_OUT:
		return sys_out(a1, a2, a3);
	case SYS_GETUID:
		return sys_getuid(a1);
	case SYS_SETUID:
		return sys_setuid(a1);
	case SYS_SETEUID:
		return sys_seteuid(a1);
	case SYS_GETGID:
		return sys_getgid(a1);
	case SYS_SETGID:
		return sys_setgid(a1);
	case SYS_SETEGID:
		return sys_setegid(a1);
	default:
		return -ENOSYS;
	}
}
