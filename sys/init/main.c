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


#include <sys/stdint.h>
#include <machine/vmparam.h>
#include <microkernel.h>
#include <syslib.h>

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

int __sys_inthandler(int vect, unsigned long info, struct intframe *f)
{
	static int i = 0;

	printf("\nException %d, va = %p\n", vect, info);
	framedump(f);

	if (vect == XCPT_PGFAULT) {
		if (i == 0) {
			printf("Mapping read only: %d",
			       vmmap(info, VM_PROT_RO));
			i++;
		} else if (i == 1) {
			printf("Mapping writeable: %d",
			       vmchprot(info, VM_PROT_RW));
			i++;
		} else if (i == 2) {
			printf("A-ha! Mapping again!\n");
			vmmap(info, VM_PROT_RO);
			i++;
		} else {
			printf("WTF?\n");
			sys_die();
		}
	}
	printf("\n");
}

int *d = (void *) (8 * PAGE_SIZE);

int main()
{
	int i;

	siginit();


	printf("Hello!\n");

	printf("Reading, d = %d\n", *d);

	printf("Writing.\n");
	*d = 0;
	printf("Done writing.\n");

	printf("Unnmapping: %d", vmunmap(d));
	printf("And accessing it again!\n");
	printf("d is %d\n", *d);

	if (sys_fork()) {
	  printf("Parent!\n");
	} else {
	  printf("child!\n");
	  while(1)
	    printf("b");
	  sys_wait();
	}

	printf("Goodbye!\n");
	sys_die();
}
