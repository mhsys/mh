#include <machine/uk/cpu.h>
#include <uk/sys.h>
#include <ukern/kern.h>
#include <lib/lib.h>

static int sys_putc(int ch)
{
	sysputc(ch);
	return 0;
}

static int sys_die(void)
{
	die();
	return 0;
}

static int sys_xcptentry(vaddr_t entry, vaddr_t stack)
{
	struct thread *th = current_thread();

	__usrentry_setup(&th->xcptentry, entry, stack);
	th->flags |= THFL_XCPTENTRY;
	return 0;
}

static int sys_xcptreturn(unsigned long ret)
{
	struct thread *th = current_thread();

	if (!(th->flags & THFL_IN_XCPTENTRY)
		|| !(th->flags & THFL_XCPTENTRY)
		|| ret) {
		die();
		return 0;
	}

	th->flags &= ~THFL_IN_XCPTENTRY;
	th->flags |= THFL_IN_USRENTRY;
	__insn_barrier();
	__usrentry_enter(th->usrentry.data);
	/* Not reached */
	return 0;
}

int sys_call(int sc, unsigned long a1, unsigned long a2)
{
	switch(sc) {
	case SYS_PUTC:
		return sys_putc(a1);
	case SYS_DIE:
		return sys_die();
	case SYS_XCPTENTRY:
		return sys_xcptentry(a1, a2);
	case SYS_XCPTRET:
		return sys_xcptreturn(a1);
	default:
		return -1;
	}
}
