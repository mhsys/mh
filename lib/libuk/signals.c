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


#include <machine/vmparam.h>
#include <microkernel.h>

extern void __inthdlr(void);

/* This will be put by the linker script in the NOCOW area. On fork
   the memory will be copied, guaranteeing signal handlers to keep
   working. */
char _sigstack[2048] __section(".zcow") = { 0 };

int _libuk_signal_handler(int vect, u_long va, u_long err,
			  struct intframe *f)
{

	if (vect == XCPT_PGFAULT)
		__sys_pgfaulthandler(va, err, f);
	else
		__sys_inthandler(vect, va, err, f);
}

int _libuk_signals_nointr(int vect, u_long va, u_long err,
			  struct intframe *f)
{

	return -1;
}

int _libuk_signals_nopgfault(u_long va, u_long err, struct intframe *f)
{
	printf("What!");
	return -1;
}

void siginit(void)
{
	void *stkptr = (void *) (_sigstack + 2048);
	sys_inthdlr(__inthdlr, stkptr);
}

__weak_alias(__sys_inthandler, _libuk_signals_nointr)
