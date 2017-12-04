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
#include <uk/setjmp.h>
#include <uk/logio.h>
#include <uk/cpu.h>
#include <machine/uk/param.h>
#include <machine/uk/machdep.h>
#include <uk/locks.h>
#include <uk/kern.h>

#include "i386.h"
#include "lapic.h"
#include "tlb.h"

static lock_t __crash_lock = 0;
int __crash_requested = 0;

static char *exceptions[] = {
	"Divide by zero exception",
	"Debug exception",
	"NMI",
	"Overflow exception",
	"Breakpoint exception",
	"Bound range exceeded",
	"Invalid opcode",
	"No math coprocessor",
	"Double fault",
	"Coprocessor segment overrun",
	"Invalid TSS",
	"Segment not present",
	"Stack segment fault",
	"General protection fault",
	"Page fault",
	"Reserved exception",
	"Floating point error",
	"Alignment check fault",
	"Machine check fault",
	"SIMD Floating-Point Exception",
};

static void framedump(struct usrframe *f)
{

	printf("\tCR3: %08x\tCR2: %08x\terr: %08x\n",
	       f->cr3, f->cr2, f->err);
	printf("\tCS: %04x\tEIP: %08x\tEFLAGS: %08x\n",
	       (int) f->cs, f->eip, f->eflags);
	printf("\tEAX: %08x\tEBX: %08x\tECX: %08x\tEDX:%08x\n",
	       f->eax, f->ebx, f->ecx, f->edx);
	printf("\tEDI: %08x\tESI: %08x\tEBP: %08x\tESP:%08x\n",
	       f->edi, f->esi, f->ebp, f->espx);
	printf("\tDS: %04x\tES: %04x\tFS: %04x\tGS: %04x\n",
	       f->ds, f->es, f->fs, f->gs);
}

static unsigned vect_to_xcpt(uint32_t vect)
{
	switch (vect) {
	case 14:
		return XCPT_PGFAULT;
	default:
		return vect + 50;
	}
}

int nmi_entry(uint32_t vect, struct usrframe *f)
{
	/* NMI Handler:
	 *
	 * Remarkably the only interrupt/exception from which we can
	 * return, when it is triggered in kernel mode.
	 */
	if (__predict_false(__crash_requested)) {
#ifdef CONFIG_CRASH_DUMP_ALLCPUS
		spinlock(&__crash_lock);
		printf("CPU#%d Frame Dump:\n", cpu_number());
		framedump(f);
		spinunlock(&__crash_lock);
#endif /* CONFIG_CRASH_DUMP_ALLCPUS */
		asm volatile ("cli; hlt");
		/* Not reached */
		return -1;
	}

	__flush_tlbs_on_nmi();

	/* Process softirqs if returning to userspace */
	if (f->cs == UCS || thread_is_idle(current_thread())) {
		current_thread()->frame = f;
		do_softirq();
		current_thread()->frame = NULL;
	}
	return 0;
}

int xcpt_entry(uint32_t vect, struct usrframe *f)
{

	/* Process crash request */
	if (__predict_false(vect == 0x6 && __crash_requested)) {
		printf("Kernel Panic at CPU#%d:\n", cpu_number());
		spinlock(&__crash_lock);
		printf("CPU#%d Frame Dump:\n", cpu_number());
		framedump(f);
		spinunlock(&__crash_lock);
		cpu_nmi_broadcast();
		asm volatile ("cli; hlt");
		return -1;
	}

	if (__predict_false(f->cs != UCS)) {

		if (current_cpu()->usrpgfault && (vect == 14)) {
			current_cpu()->usrpgaddr = f->cr2;
			_longjmp(current_cpu()->usrpgfaultctx, 1);
			/* Not reached */
		}
		/* Exception in Kernel. Very bad. */
		printf("\nException #%2u at %02x:%08x, addr %08x",
		       vect, f->cs, f->eip, f->cr2);

		if (vect < 20)
			printf(": %s\n", exceptions[vect]);
		else
			printf("\n");
		framedump(f);
		__goodbye();
		/* Not reached */
		return -1;
	}

	/* Userspace exception. */
	current_thread()->frame = f;
	switch (vect) {
	case 14:
		if (__predict_false(f->err & (1 << 3))) {
			panic("Reserved Bit violation in pagetable!");
			/* Not reached */
		}
		_pmap_fault(f->cr2, f->err, f);
		break;
	default:
		thintr(vect_to_xcpt(vect), f->cr2, f->err);
		break;
	}
	do_softirq();
	current_thread()->frame = NULL;

	return 0;
}

int syscall_entry(struct usrframe *f)
{
	struct thread *th = current_thread();

	assert(f->cs == UCS);
	th->frame = f;
	f->eax = sys_call(f->eax, f->edi, f->esi, f->ecx, f->edx, f->ebx);
	do_softirq();
	th->frame = NULL;
	return 0;
}

int intr_entry(uint32_t vect, struct usrframe *f)
{

	struct thread *th = current_thread();

	/* We only expect interrupts in userspace. */
	assert(f->cs == UCS || thread_is_idle(th));
	th->frame = f;

	if (vect == VECT_KICK) {
		lapic_eoi();
		cpu_kick();
	} else if (vect >= VECT_IRQ0) {
		unsigned irq = vect - VECT_IRQ0;
		irqsignal(irq, gsi_is_level(irq));
		lapic_eoi();
	}

	do_softirq();
	th->frame = NULL;
	return 0;
}

void ___usrentry_enter(void *);

void usrframe_switch(void)
{
	current_pcpu()->tss.esp0 =
		(uint32_t) current_thread()->stack_4k + 0xff0;
}

void usrframe_enter(struct usrframe *f)
{

	current_pcpu()->tss.esp0 =
		(uint32_t) current_thread()->stack_4k + 0xff0;
	current_pcpu()->tss.ss0 = KDS;
	___usrentry_enter((void *) f);
}

uint32_t usrframe_iret(struct usrframe *f)
{
	int r;
	struct iretframe {
		uint32_t eax;
		uint32_t eip;
		uint32_t esp;
		uint32_t efl;
	} __packed iretf;

	r = copy_from_user(&iretf, f->esp, sizeof(iretf));
	if (r != 0) {
		printf("can't read from stack.  Die.");
		die();
		/* not reached */
	}
	f->eip = iretf.eip;
	f->esp = iretf.esp;
	current_thread()->userfl = iretf.efl;
	return iretf.eax;
}

void usrframe_signal(struct usrframe *f, vaddr_t ip, vaddr_t sp,
		     uint32_t fl, unsigned xcpt, vaddr_t va, u_long err)
{
	int r;
	/* We write to this structure. Change SIGFRAME_SIZE in case
	 * you change this */
	struct stackframe {
		uint32_t arg1;
		uint32_t arg2;
		uint32_t arg3;
		uint32_t eip;
		uint32_t esp;
		uint32_t efl;
	} __packed sf = {
		.arg3 = err,
		.arg2 = va,
		.arg1 = xcpt,
		.eip = f->eip,
		.esp = f->esp,
		.efl = fl,
	};
	uaddr_t usp = sp - sizeof(sf);

	if (!ip || !sp) {
		printf("no interrupt handler");
		die();
		/* not reached */
	}

	/* Write the stack with return addr and arguments */
	r = copy_to_user(usp, &sf, sizeof(sf));
	if (r != 0) {
		printf("can't write to stack.  Die.");
		die();
		/* not reached */
	}

	/* Change the user entry on success. */
	f->esp = usp;
	f->eip = ip;
}

void usrframe_extint(struct usrframe *f, vaddr_t ip, vaddr_t sp,
		     uint32_t fl, unsigned xcpt, uint64_t sigs)
{
	usrframe_signal(f, ip, sp, fl, xcpt, sigs >> 32,
			sigs & 0xffffffff);
}

void usrframe_setret(struct usrframe *f, unsigned long r)
{
	f->eax = 0;
}

void usrframe_setup(struct usrframe *f, vaddr_t ip, vaddr_t sp)
{
	memset(f, 0, sizeof(*f));
	f->ds = UDS;
	f->es = UDS;
	f->fs = UDS;
	f->gs = UDS;
	f->eip = ip;
	f->cs = UCS;
	f->eflags = 0x202;
	f->esp = sp;
	f->ss = UDS;
}
