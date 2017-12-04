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


#include <uk/param.h>
#include <machine/uk/asm.h>
	
#define MULTIBOOT_HDRSIGN 0x1badb002 
#define MULTIBOOT_REGSIGN 0x2badb002
#define _BOOT_FLAGS       0x00000003

#define MBIOFFSET_FLAGS   0
#define _MBIF_CMDLINE     (1 << 2)
#define MBIOFFSET_CMDLINE 16

ENTRY(start)
ENTRY(_start)
	jmp multiboot_entry

	.align 4
multiboot_header:
	.long MULTIBOOT_HDRSIGN
	.long _BOOT_FLAGS
	.long -(MULTIBOOT_HDRSIGN+_BOOT_FLAGS)
	.long multiboot_header
	.long _start
	.long _edata
	.long _end
	.long multiboot_entry

multiboot_entry:
	movl $(stacktop - UKERNBASE), %esp

	/* Clear flags (cld and cli) */
	pushl $0
	popf

	/* Check Multiboot */
	cmpl $MULTIBOOT_REGSIGN, %eax
	jne badmultiboot

	/* Setup boot paging structures */
	mov $0, %eax
	mov $(8 * 4), %ecx
	mov $UKERN_BL3TABLE, %edi
	rep stosb
	movl $(UKERN_BL2TABLE + 1), UKERN_BL3TABLE
	movl $(UKERN_BL2TABLE + 1), UKERN_BL3TABLE + 24
	mov $0, %eax
	mov $4096, %ecx
	mov $UKERN_BL2TABLE, %edi
	rep stosb

	mov  $0xe3, %eax
	mov  $512, %ecx
	mov  $UKERN_BL2TABLE, %edi
1:	mov  %eax, (%edi)
	add  $(1 << 21), %eax
	add  $8, %edi
	loop 1b

	/* Set segments in a known state */
	call _C_LABEL(_setup_boot_gdt)
	lgdt UKERN_BGDTREG

	/* Enable paging */
	mov %cr4, %eax
	or  $0x30, %eax
	mov  %eax, %cr4
	mov $UKERN_BL3TABLE, %eax
	mov %eax, %cr3
	mov  %cr0, %eax
	or   $((1<<31)|(1<<16)), %eax
	mov  %eax, %cr0
	ljmp $0x08, $1f
1:	mov $0x10, %eax
	mov %ax, %ds
	mov %ax, %es
	mov %ax, %ss
	mov $0, %eax
	mov %ax, %fs
	mov %ax, %gs

	/* Setup IDT */
	call _C_LABEL(_setup_idt)
	lidt _C_LABEL(_kern_idtr)

	/* Save Multiboot CMDLINE */
	mov MBIOFFSET_FLAGS(%ebx), %eax
	and $_MBIF_CMDLINE, %eax
	jz 1f
	mov MBIOFFSET_CMDLINE(%ebx), %eax
	jmp 2f
1:      xor %eax, %eax
2:	mov %eax, _C_LABEL(_boot_cmdline)

	/* Prepare real mode re-entry */
	mov $start_code16, %esi
	mov $UKERN_BCODE16, %edi
	mov $size_code16, %ecx
1:      rep movsb

	/* Call Real mode */
	mov $UKERN_BCODE16, %eax
	call *%eax

	/* Rebase pointers still in use */
	addl $UKERNBASE, %esp
	addl $UKERNBASE, vptr
	addl $UKERNBASE, vptrmax
	addl $UKERNBASE, vptrscrsrc
	addl $UKERNBASE, vptrscrdst

	call _C_LABEL(_fixup_gdt)
	call _C_LABEL(_clear_bss)
	call _C_LABEL(_boot_putcinit)
	call sysboot
halt:
	pushl $haltstring
	call printf
	add $4, %esp
loop: 
	cli
	hlt
	jmp loop

badmultiboot:
	lea badbootstring, %esi
	call print
	jmp loop


print:
	movl $0x1000, %ecx
	movb $0x87, %ah
	movl $0xb8000, %edi
1:	lodsb
	stosw
	cmp $0, %al
	loopne 1b
	ret

ENTRY(_clear_bss)
	push %edi
	mov $0x0, %eax
	mov $_ebss, %ecx
	sub $_sbss, %ecx
	mov $_sbss, %edi
	rep stosb
	pop %edi
	ret
END(_clear_bss)

ENTRY(_setup_boot_gdt)
	/* Zero whole page */
	mov $0, %eax
	mov $1024, %ecx
	mov $UKERN_BGDTREG, %edi
	rep stosl

	mov $UKERN_BGDTREG, %eax
	movw $(128 * 8 - 1), (%eax)
	movl $(UKERNBASE + UKERN_BGDTABLE), 2(%eax)

	movl $UKERN_BGDTABLE + 8, %eax
	/* 32-bit code */
	movl $0x0000ffff, (%eax)
	movl $0x00cf9a00, 4(%eax)
	add $8, %eax
	/* 32-bit data */
	movl $0x0000ffff, (%eax)
	movl $0x00cf9200, 4(%eax)
	add $8, %eax
	/* 16-bit code */
	movl $0x0000ffff, (%eax)
	movl $0x00009a01, 4(%eax)
	add $8, %eax
	/* 16-bit code */
	movl $0x0000ffff, (%eax)
	movl $0x00009201, 4(%eax)
	add $8, %eax
	ret

ENTRY(_fixup_gdt)
	mov $UKERN_BGDTABLE + UKERNBASE + (8 * 3), %eax
	/* User 32-bit code */
	movl $0x0000ffff, (%eax)
	movl $0x00cffa00, 4(%eax)
	add $8, %eax
	/* User 32-bit data */
	movl $0x0000ffff, (%eax)
	movl $0x00cff200, 4(%eax)
	add $8, %eax
	ret
END(_fixup_gdt)

	.globl _C_LABEL(_set_tss)
_C_LABEL(_set_tss):
	push  %ebp
	mov   %esp, %ebp
	/* TSS (5 + 2*n) */
	movl  8(%ebp), %eax
	shl   $4, %eax
	add $(UKERNBASE + UKERN_BGDTABLE + (5 * 8)), %eax
	movl $0, (%eax)
	movl $0, 4(%eax)
	movl 0xc(%ebp), %ecx
	movw $104, (%eax)
	movw %cx, 2(%eax)
	shr  $16, %ecx
	movb %cl, 4(%eax)
	movb %ch, 7(%eax)
	orl  $0x00408900, 4(%eax)
	mov  8(%ebp), %eax
	shl  $1, %eax
	add  $5, %eax
	shl  $3, %eax
	ltr  %ax
	pop  %ebp
	ret

	.globl _C_LABEL(_set_fs)
_C_LABEL(_set_fs):
	push  %ebp
	mov   %esp, %ebp
	/* FS (5 + 2*n + 1)*/
	mov  8(%ebp), %ecx
	mov  %ecx, %eax
	shl  $4, %eax
	add $(UKERNBASE + UKERN_BGDTABLE + ((5 + 1) * 8)), %eax
	movl $0, (%eax)
	movl $0, 4(%eax)
	movw $4, (%eax)
	movl 0xc(%ebp), %ecx
	movw %cx, 2(%eax)
	shr  $16, %ecx
	movb %cl, 4(%eax)
	movb %ch, 7(%eax)
	orl  $0x00409200, 4(%eax)
	movl 8(%ebp), %eax
	shl  $1, %eax
	add  $(5+1), %eax
	shl  $3, %eax
	mov  %ax, %fs
	pop  %ebp
	ret

/*
 * AP BOOTSTRAP.
 *
 * This code is copied into per-cpu pages.
 * These same page will become the AP kernel
 * stack.
 */
	.code16
	.globl _C_LABEL(_ap_start)
ENTRY(_ap_start)
	cli
	movw %cs, %ax
	movw %ax, %ds

	lgdtl .Lap_gdtreg - _C_LABEL(_ap_start)
	lidtl .Lap_idtreg - _C_LABEL(_ap_start)

	movw $0xb800, %ax
	movw %ax, %es
	movw $('A'<<8 + 'A'), %es:0

	/* Save AP Boostrap Page Segment. */
	mov %cs, %bx

	mov $1, %ax
	lmsw %ax
	ljmpl $0x8, $(apsetup - UKERNBASE)

.Lap_gdtreg:
	.hword (128 * 8) - 1
	.long UKERN_BGDTABLE

.Lap_idtreg:
	.hword (256 * 8) - 1
	.long _C_LABEL(_kern_idt) - UKERNBASE
ENTRY(_ap_end)

	.code32
apsetup: /* bx:	AP Bootstrap Segment */
	/* Setup segments. */
	mov $KDS, %ax
	mov %ax, %ss
	mov %ax, %ds
	mov %ax, %es
	mov %ax, %fs
	mov %ax, %gs

	/* Setup CR registers. */
	mov %cr4, %eax
	or  $0x30, %eax
	mov  %eax, %cr4
	mov $UKERN_BL3TABLE, %eax
	mov %eax, %cr3
	mov  %cr0, %eax
	or   $((1<<31)|(1<<16)), %eax
	mov  %eax, %cr0
	ljmp $0x08, $1f
1:
	/* Reload GDT and IDT */
	lgdt UKERN_BGDTREG
	lidt _C_LABEL(_kern_idtr)
	ljmp $KCS, $1f
1:
	/* Setup stack */
	xor %eax, %eax
	mov %bx, %ax
	shl $4, %eax
	add $(UKERNBASE + 0x1000), %eax
	mov %eax, %esp
	call sysboot_ap
1:	hlt
	jmp 1b

	.data
haltstring: 
	.asciz "System Halted."
haltstring_size = . - haltstring
	
badbootstring: 
	.asciz "Multiboot compliant bootloader needed."
badbootstring_size = . - badbootstring

LABEL(_ukern_end)
	.long _end

LABEL(_init_start)
	.long _init_elf

LABEL(_init_end)
	.long _einit_elf

	.align 4096
stackbtm:
	.skip 4096
stacktop:

