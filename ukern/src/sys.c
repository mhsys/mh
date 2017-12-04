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
#include <uk/string.h>
#include <uk/logio.h>
#include <uk/sys.h>
#include <uk/kern.h>
#include <uk/cpu.h>
#include <machine/uk/machdep.h>

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

	/* Can't sleep with Interrupts disabled */
	th->userfl |= THFL_INTR;
	schedule(THST_STOPPED);
	return 0;
}

static int sys_raise(unsigned sint)
{
	struct thread *th = current_thread();

	thraise(th, sint);
	return 0;
}

static int sys_fork(void)
{
	struct thread *th;

	th = thfork();
	return th == NULL ? 0 : th->pid;
}

static int sys_getpid(void)
{
	struct thread *th = current_thread();

	return th->pid;
}

static int sys_hwcreat(uaddr_t ucfg, devmode_t mode)
{
	struct thread *th = current_thread();
	struct sys_hwcreat_cfg cfg;

	if (th->euid)
		return -EPERM;
	if (!__chkuaddr(ucfg, sizeof(cfg)))
		return -EINVAL;
	if (copy_from_user(&cfg, ucfg, sizeof(cfg)))
		return -EFAULT;
	if (cfg.nirqsegs + cfg.npiosegs > SYS_HWCREAT_MAX_SEGMENTS)
		return -EINVAL;
	if (cfg.nmemsegs > SYS_HWCREAT_MAX_MEMSEGMENTS)
		return -EINVAL;

	return hwcreat(&cfg, mode);
}

static int sys_creat(uaddr_t ucfg, unsigned sig, devmode_t mode)
{
	struct sys_creat_cfg cfg;

	if (!__chkuaddr(ucfg, sizeof(cfg)))
		return -EINVAL;
	if (copy_from_user(&cfg, ucfg, sizeof(cfg)))
		return -EFAULT;
	return devcreat(&cfg, sig, mode);
}

static int sys_poll(unsigned did, uaddr_t uior)
{
	int id, ret;
	struct sys_poll_ior sior;

	id = devpoll(did, &sior);
	if (id < 0)
		return id;

	ret = copy_to_user(uior, &sior, sizeof(struct sys_poll_ior));
	if (ret)
		return ret;
	return id;
}

static int sys_wriospace(unsigned did, unsigned id, uint32_t port,
			 uint64_t val)
{
	return devwriospace(did, id, port, val);
}

static int sys_read(unsigned did, unsigned id, unsigned long iova, size_t sz, u_long va)
{
     if (!__chkuaddr(trunc_page(va), sz))
		return -EINVAL;
     return devread(did, id, iova, sz, va);
}

static int sys_write(unsigned did, unsigned id, u_long va, size_t sz, u_long iova)
{
     if (!__chkuaddr(trunc_page(va), sz))
		return -EINVAL;
     return devwrite(did, id, va, sz, iova);
}

static int sys_irq(unsigned did, unsigned id, unsigned irq)
{
	return devirq(did, id, irq);
}

static int sys_open32(u_long hi, u_long lo)
{
	uint64_t id = ((uint64_t) hi << 32) | lo;
	return devopen(id);
}

static int sys_open(uint64_t id)
{
	return devopen(id);
}

static int sys_export(unsigned ddno, u_long va, size_t sz, uaddr_t uiovaptr)
{
	int ret;
        uint64_t iova;

	if (!__chkuaddr(va, sz))
		return -EFAULT;

	if (!__chkuaddr(uiovaptr, sizeof(iova)))
		return -EINVAL;

	ret = devexport(ddno, va, sz, &iova);
	if (ret)
		return ret;

	ret = copy_to_user(uiovaptr, &iova, sizeof(iova));
	if (ret) {
		devunexport(ddno, va);
		return ret;
	}

	return ret;
}

static int sys_unexport(unsigned ddno, uaddr_t va)
{
	if (!__chkuaddr(va, 1))
		return -EINVAL;

	return devunexport(ddno, va);
}

static int sys_info(unsigned ddno, uaddr_t ucfg)
{
	int ret;
	struct sys_info_cfg cfg;

	if (!__chkuaddr(ucfg, sizeof(cfg)))
		return -EFAULT;

	memset(&cfg, 0, sizeof(struct sys_info_cfg));
	ret = devinfo(ddno, &cfg);
	if (ret)
		return ret;
	if (copy_to_user(ucfg, &cfg, sizeof(cfg)))
		return -EFAULT;
	return 0;
}

static int sys_mapirq(unsigned ddno, unsigned irq, unsigned sig)
{

	return devirqmap(ddno, irq, sig);
}

static int sys_eoi(unsigned ddno, unsigned irq)
{
	return deveoi(ddno, irq);
}

static int sys_in(unsigned ddno, uint32_t port, uaddr_t valptr)
{
	int ret;
	uint64_t val;

	if (!__chkuaddr(valptr, sizeof(val)))
		return -EFAULT;

	ret = devin(ddno, port, &val);
	if (ret)
		return ret;

	ret = copy_to_user(valptr, &val, sizeof(val));
	return ret;
}

static int sys_out32(unsigned ddno, uint32_t port, uint32_t hival,
		     uint32_t loval)
{
	uint64_t val = ((uint64_t) hival << 32) | loval;

	return devout(ddno, port, val);
}

static int sys_out(unsigned ddno, uint32_t port, uint64_t val)
{

	return devout(ddno, port, val);
}

static int sys_rdcfg(unsigned ddno, uint32_t off, uint8_t sz, uaddr_t valptr)
{
	int ret;
	uint64_t val = 0;

	switch (sz) {
	case 1:
	case 2:
	case 4:
	case 8:
		break;
	default:
		return -EINVAL;
	}

	if (!__chkuaddr(valptr, sizeof(val)))
		return -EFAULT;

	ret = devrdcfg(ddno, off, sz, &val);
	if (ret)
		return ret;

	ret = copy_to_user(valptr, &val, sizeof(val));
	return ret;
}

static int sys_wrcfg(unsigned ddno, uint32_t off, uint8_t sz, uaddr_t valptr)
{
	int ret;
	uint64_t val = 0;

	switch (sz) {
	case 1:
	case 2:
	case 4:
	case 8:
		break;
	default:
		return -EINVAL;
	}

	if (!__chkuaddr(valptr, sizeof(val)))
		return -EFAULT;

	ret = copy_from_user(&val, valptr, sizeof(val));
	if (ret)
		return ret;

	ret = devwrcfg(ddno, off, sz, val);
	return ret;
}

static int sys_close(unsigned ddno)
{

	devclose(ddno);
	return 0;
}

static int sys_map(vaddr_t vaddr, sys_map_flags_t perm)
{
	struct thread *th = current_thread();
	int np, np32, ret;
	pmap_prot_t prot;

	/* _probably_ should not kill it and just return an error,
	 * like a nice, respectful OS */
	procassert(__chkuaddr(trunc_page(vaddr), PAGE_SIZE));
	np = perm & MAP_NEW;
	np32 = perm & MAP_NEW32;
	perm &= ~(MAP_NEW | MAP_NEW32);

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
		np32 = 0;
		break;
	default:
		return -EINVAL;
	}

	if (np)
		ret = vmmap(vaddr, prot);
	else if (np32) {
		if (th->euid)
			ret = -EPERM;
		else
			ret = vmmap32(vaddr, prot);
	} else if (!prot)
		ret = vmunmap(vaddr);
	else
		ret = vmchprot(vaddr, prot);
	return ret;
}

static int sys_move(vaddr_t dst, vaddr_t src)
{
	/* _probably_ should not kill it and just return an error,
	 * like a nice, respectful OS */
	procassert(__chkuaddr(trunc_page(dst), PAGE_SIZE));
	procassert(__chkuaddr(trunc_page(src), PAGE_SIZE));
	return vmmove(dst, src);
}

/* Only  32-bit  addresses  supported  on  32-bit  architectures  from
   now. Add  sys_iomap32 if the  need of  I/O addresses at  top 64bits
   arises. */
static int sys_iomap(unsigned ddno, vaddr_t va, uint64_t mmioaddr)
{
	int ret;
	pmap_prot_t prot = PROT_USER_WR;

	if (!__chkuaddr(trunc_page(va), PAGE_SIZE))
		return -EINVAL;
	return deviomap(ddno, va, mmioaddr, prot);
}

static int sys_iounmap(unsigned ddno, vaddr_t va)
{
	int ret;

	if (!__chkuaddr(trunc_page(va), PAGE_SIZE))
		return -EINVAL;
	return deviounmap(ddno, va);
}

static int sys_yield(void)
{

	schedule(THST_RUNNABLE);
	return 0;
}

static int sys_childstat(uaddr_t ucs)
{
	int pid, ret;
	struct sys_childstat cs;

	pid = childstat(&cs);
	ret = copy_to_user(ucs, &cs, sizeof(struct sys_childstat));
	if (ret)
		return ret;
	return pid;
}

static __dead void sys_die(int status)
{
	__exit(status);
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
	return ! !current_thread()->setuid;
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

int sys_call(int sc,
	     unsigned long a1, unsigned long a2,
	     unsigned long a3, u_long a4, u_long a5)
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
	case SYS_CHILDSTAT:
		return sys_childstat(a1);
	case SYS_MAP:
		return sys_map(a1, a2);
	case SYS_MOVE:
		return sys_move(a1, a2);
	case SYS_GETPID:
		return sys_getpid();
	case SYS_RAISE:
		return sys_raise(a1);
	case SYS_FORK:
		return sys_fork();
	case SYS_DIE:
		sys_die(a1);
		return 0;
	case SYS_CREAT:
		return sys_creat(a1, a2, a3);
	case SYS_POLL:
		return sys_poll(a1, a2);
	case SYS_WRIOSPC:
		return sys_wriospace(a1, a2, a3, a4);
	case SYS_READ:
		return sys_read(a1, a2, a3, a4, a5);
	case SYS_WRITE:
		return sys_write(a1, a2, a3, a4, a5);
	case SYS_IRQ:
		return sys_irq(a1, a2, a3);
	case SYS_OPEN:
		return sys_open(a1);
	case SYS_OPEN32:
		return sys_open32(a1, a2);
	case SYS_EXPORT:
		return sys_export(a1, a2, a3, a4);
	case SYS_UNEXPORT:
		return sys_unexport(a1, a2);
	case SYS_INFO:
		return sys_info(a1, a2);
	case SYS_RDCFG:
		return sys_rdcfg(a1, a2, a3, a4);
	case SYS_MAPIRQ:
		return sys_mapirq(a1, a2, a3);
	case SYS_EOI:
		return sys_eoi(a1, a2);
	case SYS_IN:
		return sys_in(a1, a2, a3);
	case SYS_OUT32:
		return sys_out32(a1, a2, a3, a4);
	case SYS_OUT:
		return sys_out(a1, a2, a3);
	case SYS_IOMAP:
		return sys_iomap(a1, a2, a3);
	case SYS_IOUNMAP:
		return sys_iounmap(a1, a2);
	case SYS_CLOSE:
		return sys_close(a1);
	case SYS_HWCREAT:
		return sys_hwcreat(a1, a2);
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
