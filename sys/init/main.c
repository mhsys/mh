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


#include <sys/inttypes.h>
#include <machine/vmparam.h>
#include <microkernel.h>
#include <syslib.h>
#include <drex/drex.h>

static void framedump(struct intframe *f)
{

	printf("EIP: %x\tEFLAGS: %x\n", f->eip, f->eflags);
	printf("\tEAX: %x\tEBX: %x\tECX: %x\tEDX:%x\n",
	       f->eax, f->ebx, f->ecx, f->edx);
	printf("\tEDI: %x\tESI: %x\tEBP: %x\tESP:%x\n",
	       f->edi, f->esi, f->ebp, f->esp);
	printf("\tDS: %x\tES: %x\tFS: %x\tGS: %x\n",
	       f->ds, f->es, f->fs, f->gs);
}

int __sys_inthandler(int vect, u_long va, u_long err, struct intframe *f)
{
	static int i = 0;

	if (vect == INTR_EXT) {
		return;
	}
	printf("\nException %d, va = %p, err = %lx\n", vect, va, err);

	framedump(f);
	printf("\n");


}

int *d = (void *) (50L * PAGE_SIZE);

int main()
{
	int i;

	siginit();


	printf("Hello!\n");

	printf("brk: %x\n", drex_sbrk(50L * PAGE_SIZE));
	printf("Reading, d = %d\n", *d);

	printf("Writing.\n");
	*d = 0;
	printf("Done writing.\n");

	printf("Unnmapping: %d", vmunmap(d));
	printf("And accessing it again!\n");
	printf("d is %d\n", *d);
	printf("%d creat()", sys_creat(500, 9, 0111));

	if (sys_fork()) {
		int i;
		int *d = (int *) (0x53 * PAGE_SIZE);

		printf("Parent!\n");
		while (1) {
			int ret;
			unsigned id;
			struct sys_poll_ior ior;

			sys_wait();
			sys_cli();
			id = sys_poll(&ior);
			ret = sys_import(id, 1, d);
			printf("import: %d\n", ret);
			printf("I/O at port %" PRIx64 " with val %" PRIx64
			       "\n", ior.port, ior.val);
			printf("d is %x (%p)\n", *d, d);
			sys_irq(id, 3);
			printf("BOOTSTRAP: %d", sys_bootstrap(id, 0x70L * PAGE_SIZE, 500));
			sys_eio(id);
		}
	} else {
		int desc, ret;
		int *p = (int *) (0x12 * PAGE_SIZE);

		printf("child!\n");
		*p = 0;
		desc = sys_open(500);
		sys_mapirq(desc, 3, 5);
		printf("MAPPING %d\n", sys_export(0, p, 1));
		while (1) {
			(*p) += 1;
			printf("-> P is %lx\n", (unsigned long) *p);
			sys_out(desc, 10, 255);
			sys_wait();
			printf("IRQ received!\n");
		}
	}

	printf("Goodbye!\n");
	sys_die();
}
