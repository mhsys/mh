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


#ifndef _i386_machdep_h
#define _i386_machdep_h

#include <uk/types.h>

struct usrframe;

/* This must be equal to sizeof(struct stackframe) */
#define SIGFRAME_SIZE (sizeof(uint32_t) * 6)
#define __inthdlr_chk(_ip, _sp)						\
  (__chkuaddr(ip, 4) && __chkuaddr(sp, SIGFRAME_SIZE))

void usrframe_enter(struct usrframe *f);
void usrframe_setup(struct usrframe *f, vaddr_t ip, vaddr_t sp);
void usrframe_signal(struct usrframe *f, vaddr_t ip, vaddr_t sp,
		     uint32_t fl, unsigned xcpt, vaddr_t info, u_long err);
void usrframe_extint(struct usrframe *f, vaddr_t ip, vaddr_t sp,
		     uint32_t fl, unsigned xcpt, uint64_t sigs);
void usrframe_setret(struct usrframe *f, unsigned long r);
uint32_t usrframe_iret(struct usrframe *f);
void usrframe_switch();
void usrframe_settls(struct usrframe *f, uaddr_t tls);

uint64_t timer_readcounter(void);
uint64_t timer_readperiod(void);
void timer_setcounter(uint64_t);
void timer_setalarm(uint64_t);
void timer_handler_cb(void);

extern int __crash_requested;

#define __goodbye()				\
	do {					\
		__crash_requested = 1;		\
		__insn_barrier();		\
		asm volatile("ud2");		\
	} while(0)

#endif
