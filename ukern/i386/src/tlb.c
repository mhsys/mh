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
#include <uk/cpu.h>
#include <machine/uk/tlb.h>

static int __tlbcnt = 0;

void __flush_tlbs_on_nmi(void)
{
	int tlbf = __sync_fetch_and_and(&current_cpu()->tlbop, 0);

	if (tlbf & TLBF_GLOBAL)
		__flush_global_tlbs();
	else if (tlbf & TLBF_NORMAL)
		__flush_local_tlbs();

	__sync_sub_and_fetch(&__tlbcnt, 1);
}


void __flush_local_tlbs(void)
{

	asm volatile ("mov %%cr3, %%eax; mov %%eax, %%cr3\n":::"eax");
}

void __flush_global_tlbs(void)
{

	asm volatile ("mov %%cr4, %%eax\n"
		      "and $0xffffff7f, %%eax\n"
		      "mov %%eax, %%cr4\n"
		      "or  $0x00000080, %%eax\n"
		      "mov %%eax, %%cr4\n":::"eax");
}


/* *INDENT-OFF* (indent confused by foreach block */
void __flush_tlbs(cpumask_t cpumask, unsigned tlbf)
{

	foreach_cpumask(cpumask, {
			struct cpu_info *ci;

			if (i == cpu_number())
				continue;

			ci = cpuinfo_get(i);
			if (ci == NULL)
				continue;
			__sync_fetch_and_or(&ci->tlbop, tlbf);
			__sync_add_and_fetch(&__tlbcnt, 1);
			cpu_nmi(i);
		});

	if (tlbf & TLBF_GLOBAL)
		__flush_global_tlbs();
	else if (tlbf & TLBF_NORMAL)
		__flush_local_tlbs();

	/* Synchronous wait */
	while (__sync_add_and_fetch(&__tlbcnt, 0) != 0) {
		asm volatile ("pause; pause; pause;");
	}
}
/* *INDENT-ON* */
