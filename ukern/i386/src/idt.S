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


#include <machine/uk/asm.h>
#include <machine/uk/param.h>

	.altmacro
.macro _do_entry vct
	pushal
	mov  %cr3, %eax
	push %eax
	mov %cr2, %eax
	push %eax
	pushw %gs
	pushw %fs
	pushw %es
	pushw %ds
	mov $KDS, %ax
	mov %ax, %ds
	mov %ax, %es
	call _C_LABEL(lapic_getcurrent)
	shl $5, %ax
	add $(6 << 3), %ax
	mov %ax, %fs
	xor %eax, %eax
	mov %ax, %gs
	
	mov %esp, %eax
	pushl %eax

	.if \vct == VECT_SYSC

	call _C_LABEL(syscall_entry)
	add $4, %esp

	.else

	pushl $\vct

	.if \vct == 2
	call _C_LABEL(nmi_entry)
	.elseif \vct < 32
	call _C_LABEL(xcpt_entry)
	.else
	call _C_LABEL(intr_entry)
	.endif

	add $8, %esp

	.endif
.endm

.macro _do_iret_exit
	popw %ds
	popw %es
	popw %fs
	popw %gs
	/* Skip CR2 and CR3 */
	addl $8, %esp
	popal
	/* Skip error code */
	add $4, %esp
	iret
.endm

/* XXX:TODO: differentiate between NMI and the rest, and expect
  exceptions and interrupts to be all from userspace. (WHAT ABOUT IPI?
	same, interrupt disabled) */

.macro handler_e vct
	.globl hdlr_&\vct
	.align 64
hdlr_&\vct&:
	_do_entry \vct
	_do_iret_exit
.endm

.macro handler vct
	.globl hdlr_&\vct
	.align 64
hdlr_&\vct&:
	pushl $0
	_do_entry \vct
	_do_iret_exit
.endm
	

.macro repeat macro from to
	\macro \from
	.if \to-\from
	repeat \macro,%(\from+1),\to
	.endif
.endm

.macro _setidt vct usr
	mov $hdlr_&\vct, %ecx
	mov $KCS, %ax
	shl $16, %eax
	mov %cx, %ax
	mov %eax, _C_LABEL(_kern_idt) + (\vct * 8)
	mov %ecx, %eax
	.if \usr > 0
	mov $0xee00, %ax
	.else
	mov $0x8e00, %ax
	.endif
	mov %eax, _C_LABEL(_kern_idt) + (\vct * 8) + 4
.endm

.macro setidt vct
	_setidt \vct 0
.endm

ENTRY(_setup_idt)
	repeat setidt 0, 31
	repeat setidt 32, 63
	repeat setidt 64, 95
	repeat setidt 96, 127
	repeat setidt 128, 159
	_setidt 33 1 /* syscall */
	ret

	repeat handler 0, 7
	handler 8
	handler 9
	repeat handler_e 10, 14
	handler 15
	handler 16
	handler_e 17
	repeat handler 18, 31
	repeat handler 32, 63
	repeat handler 64, 127
	repeat handler 128, 191
	repeat handler 192, 255

ENTRY(___usrentry_enter)
	push %esp
	mov %esp, %ebp
	mov 8(%ebp), %esp
	_do_iret_exit
END(___usrentry_enter)

.data
	.align 64
LABEL(_kern_idt)
	.skip  256 * 8

LABEL(_kern_idtr)
	.word (256 * 8) - 1
	.long _C_LABEL(_kern_idt)

