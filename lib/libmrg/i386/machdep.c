#include <sys/types.h>
#include <microkernel.h>
#include <setjmp.h>
#include <stdio.h>
#include <mrg.h>

void _setupjmp(jmp_buf, void *, void *);

void framedump(struct intframe *f)
{

	printf("\tEIP: %08x\tEFLAGS: %08x\n", f->eip, f->eflags);
	printf("\tEAX: %08x\tEBX: %08x\tECX: %08x\tEDX:%08x\n",
	       f->eax, f->ebx, f->ecx, f->edx);
	printf("\tEDI: %08x\tESI: %08x\tEBP: %08x\tESP:%08x\n",
	       f->edi, f->esi, f->ebp, f->esp);
	printf("\tDS: %08x\tES: %08x\tFS: %08x\tGS: %08x\n",
	       f->ds, f->es, f->fs, f->gs);
}

void framelongjmp(volatile struct intframe *f, jmp_buf *jb)
{
	uint32_t *buf = (uint32_t *)(*jb);

	f->eip = buf[0];
	f->ebx = buf[1];
	f->esp = buf[2] + 4; /* ret consumes the stack */
	f->ebp = buf[3];
	f->esi = buf[4];
	f->edi = buf[5];
	f->eax = 1;
}

void
lwt_makebuf(lwt_t *lwt, void (*start)(void *), void * arg,
	    void *stack_base, size_t stack_size)
{
	void *sp = (void *)(stack_base + stack_size - 16);

	*(void **)(sp + 12) = arg;
	*(void **)(sp + 8) = start;
	*(void **)(sp + 4) = (void*)0x50500505; /* Canary for return value */
	*(void **)sp = 0; /* _longjmp uses ret, which consumes this */
	_setupjmp(lwt->buf, (void *)__lwt_func, sp);
}
