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


#ifndef _microkernel_h_
#define _microkernel_h_

#include <sys/types.h>
#include <machine/microkernel.h>
#include <uk/sys.h>

/* Syscalls */
__dead void sys_die();
int sys_inthdlr(void (*func) (vaddr_t, vaddr_t), void *stack);
void sys_cli(void);
void sys_sti(void);
void sys_wait(void);
int sys_fork(void);
int sys_map(vaddr_t vaddr, sys_map_flags_t perm);
int sys_move(vaddr_t dst, vaddr_t src);


int sys_putc(int ch);

extern int __sys_inthandler(int, u_long, u_long, struct intframe *);
extern int __sys_pgfaulthandler(u_long, u_long, struct intframe *);


/* Signals helper library */
void siginit(void);

/* VM helper library */
typedef enum {
	VM_PROT_NIL = MAP_NONE,
	VM_PROT_RO = MAP_RDONLY,
	VM_PROT_RX = MAP_RDEXEC,
	VM_PROT_RW = MAP_WRITE,
	VM_PROT_WX = MAP_WREXEC,
} vm_prot_t;

int vmmap(vaddr_t addr, vm_prot_t prot);
int vmunmap(vaddr_t addr);
int vmchprot(vaddr_t addr, vm_prot_t prot);

#endif