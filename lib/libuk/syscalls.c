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


#include <microkernel.h>
#include "__sys.h"

__dead void sys_die(int status)
{
	int dummy;

	__syscall1(SYS_DIE, status, dummy);
	/* Not reached */
}

int sys_putc(int ch)
{
	int ret;

	__syscall1(SYS_PUTC, ch, ret);
	return ret;
}

int sys_inthdlr(void *entry, void *stack)
{
	int ret;

	__syscall2(SYS_INTHDLR,
		   (unsigned long) entry, (unsigned long) stack, ret);
	return ret;
}

void sys_sti(void)
{
	int dummy;

	__syscall0(SYS_STI, dummy);
}

void sys_cli(void)
{
	int dummy;

	__syscall0(SYS_CLI, dummy);
}

void sys_wait(void)
{
	int dummy;

	__syscall0(SYS_WAIT, dummy);
}

void sys_yield(void)
{
	int dummy;

	__syscall0(SYS_YIELD, dummy);
}

int sys_childstat(struct sys_childstat *cs)
{
	int ret;

	__syscall1(SYS_CHILDSTAT, (unsigned long) cs, ret);
	return ret;
}

int sys_fork(void)
{
	int ret;

	__syscall0(SYS_FORK, ret);
	return ret;
}

int sys_getpid(void)
{
	int ret;

	__syscall0(SYS_GETPID, ret);
	return ret;
}

int sys_raise(int signal)
{
	int ret;

	__syscall1(SYS_RAISE, signal, ret);
	return ret;
}

int sys_tls(void *tls)
{
	int ret;

	__syscall1(SYS_TLS, (unsigned long)tls, ret);
	return ret;
}

int sys_map(vaddr_t vaddr, sys_map_flags_t perm)
{
	int ret;

	__syscall2(SYS_MAP,
		   (unsigned long) vaddr, (unsigned long) perm, ret);
	return ret;
}

int sys_move(vaddr_t dst, vaddr_t src)
{
	int ret;

	__syscall2(SYS_MOVE,
		   (unsigned long) dst, (unsigned long) src, ret);
	return ret;

}

int sys_open(uint64_t id)
{
	int ret;

#if _LP64
	__syscall1(SYS_OPEN, (unsigned long) id, ret);
#else

	__syscall2(SYS_OPEN32, (unsigned long) (id >> 32), (unsigned long) id, ret);
#endif
	return ret;
}

int sys_open32(u_long hi, u_long lo)
{
	int ret;

	__syscall2(SYS_OPEN32, hi, lo, ret);
	return ret;
}

int sys_iomap(unsigned ddno, u_long va, uint64_t mmioaddr)
{
	int ret;

	__syscall3(SYS_IOMAP, (unsigned long) ddno, (unsigned long) va,
		   (unsigned long) mmioaddr, ret);
	return ret;
}

int sys_iounmap(unsigned ddno, u_long va)
{
	int ret;

	__syscall2(SYS_IOUNMAP, (unsigned long) ddno, (unsigned long) va,
		   ret);
	return ret;
}

int sys_export(unsigned ddno, u_long va, size_t sz, iova_t *iova)
{
	int ret;

	__syscall4(SYS_EXPORT, (unsigned long) ddno, (unsigned long) va,
		   (unsigned long) sz, (unsigned long) iova, ret);
	return ret;
}

int sys_unexport(unsigned ddno, vaddr_t va)
{
	int ret;

	__syscall2(SYS_UNEXPORT, (unsigned long) ddno, (unsigned long) va, ret);

	return ret;
}

int sys_mapirq(unsigned ddno, unsigned id, unsigned sig)
{
	int ret;

	__syscall3(SYS_MAPIRQ, (unsigned long) ddno, (unsigned long) id,
		   (unsigned long) sig, ret);
	return ret;

}

int sys_eoi(unsigned ddno, unsigned irq)
{
	int ret;

	__syscall2(SYS_EOI, (unsigned long) ddno, (unsigned long) irq, ret);
	return ret;
}

int sys_in(unsigned ddno, u_long port, uint64_t * val)
{
	int ret;

	__syscall3(SYS_IN, (unsigned long) ddno, (unsigned long) port,
		   (unsigned long) val, ret);
	return ret;
}

int sys_out(unsigned ddno, uint32_t  port, uint64_t val)
{
	int ret;

#if _LP64
	__syscall3(SYS_OUT, (unsigned long) ddno, (unsigned long) port,
		   (unsigned long) val, ret);
#else
	__syscall4(SYS_OUT32, (unsigned long) ddno, (unsigned long) port,
		   (unsigned long) (val >> 32), (unsigned long) val, ret);
#endif
	return ret;
}

int sys_info(unsigned ddno, struct sys_info_cfg *cfg)
{
	int ret;

	__syscall2(SYS_INFO, (unsigned long) ddno, (unsigned long) cfg,
		   ret);
	return ret;
}

int sys_rdcfg(unsigned ddno, uint32_t off, uint8_t sz, uint64_t *val)
{
	int ret;

	__syscall4(SYS_RDCFG, (unsigned long)ddno, (unsigned long)off,
		   (unsigned long)sz, (unsigned long)val, ret);
	return ret;
}

int sys_wrcfg(unsigned ddno, uint32_t off, uint8_t sz, uint64_t *val)
{
	int ret;

	__syscall4(SYS_WRCFG, (unsigned long)ddno, (unsigned long)off,
		   (unsigned long)sz, (unsigned long)val, ret);
	return ret;
}

int sys_close(unsigned ddno)
{
	int ret;

	__syscall1(SYS_CLOSE, (unsigned long) ddno, ret);
	return ret;
}

int sys_creat(struct sys_creat_cfg *cfg, unsigned sig, devmode_t mode)
{
	int ret;

	__syscall3(SYS_CREAT, (unsigned long) cfg, (unsigned long) sig,
		   (unsigned long) mode, ret);
	return ret;
}

int sys_poll(unsigned did, struct sys_poll_ior *ior)
{
	int ret;

	__syscall2(SYS_POLL, did, (unsigned long) ior, ret);
	return ret;
}

int sys_wriospc(unsigned did, unsigned id, uint32_t port, unsigned long val)
{
	int ret;

	__syscall4(SYS_WRIOSPC, (unsigned long) did, (unsigned long) id, (unsigned long) port, val, ret);
	return ret;
}

int sys_irq(unsigned did, unsigned id, unsigned irq)
{
	int ret;

	__syscall3(SYS_IRQ, (unsigned long) did, (unsigned long) id, (unsigned long ) irq, ret);
	return ret;
}

int sys_read(unsigned did, unsigned id, unsigned long iova, size_t sz, u_long va)
{
	int ret;

	__syscall5(SYS_READ, (unsigned long) did, (unsigned long) id, (unsigned long) iova,
		   (unsigned long) sz, va, ret);
	return ret;
}

int sys_write(unsigned did, unsigned id, unsigned long va, size_t sz, u_long iova)
{
	int ret;

	__syscall5(SYS_WRITE, (unsigned long) did, (unsigned long) id, va,
		   (unsigned long) sz, iova, ret);
	return ret;
}

int sys_hwcreat(struct sys_hwcreat_cfg *cfg, devmode_t mode)
{
	int ret;

	__syscall2(SYS_HWCREAT, (unsigned long) cfg, mode, ret);
	return ret;
}

uid_t sys_getuid(int sel)
{
	int ret;

	__syscall1(SYS_GETUID, (unsigned long) sel, ret);
	return ret;
}

gid_t sys_getgid(int sel)
{
	int ret;

	__syscall1(SYS_GETGID, (unsigned long) sel, ret);
	return ret;
}

int sys_setuid(uid_t uid)
{
	int ret;

	__syscall1(SYS_SETUID, (unsigned long) uid, ret);
	return ret;
}

int sys_seteuid(uid_t uid)
{
	int ret;

	__syscall1(SYS_SETEUID, (unsigned long) uid, ret);
	return ret;
}

int sys_setsuid(uid_t uid)
{
	int ret;

	__syscall1(SYS_SETSUID, (unsigned long) uid, ret);
	return ret;
}

int sys_setgid(gid_t gid)
{
	int ret;

	__syscall1(SYS_SETGID, (unsigned long) gid, ret);
	return ret;
}

int sys_setegid(gid_t gid)
{
	int ret;

	__syscall1(SYS_SETEGID, (unsigned long) gid, ret);
	return ret;
}

int sys_setsgid(gid_t gid)
{
	int ret;

	__syscall1(SYS_SETSGID, (unsigned long) gid, ret);
	return ret;
}
