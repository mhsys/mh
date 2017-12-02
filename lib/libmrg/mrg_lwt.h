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


#ifndef __mrg_lwt_h
#define __mrg_lwt_h

#include <sys/types.h>
#include <sys/queue.h>
#include <setjmp.h>

struct lwt {
#define LWTF_ACTIVE   1
#define LWTF_XCPT     2
	int flags;

	jmp_buf buf;
	jmp_buf xcptbuf;
	void *priv;

	void (*start)(void *);
	void *arg;
	void *stack;
	size_t stack_size;

	SIMPLEQ_ENTRY(lwt) list;
};
typedef struct lwt lwt_t;

void __lwt_func(void *, void *);

void lwt_makebuf(lwt_t *lwt, void (*start)(void *), void * arg,
		 void *stack_base, size_t stack_size);
lwt_t *lwt_create_priv(void (*start)(void *), void *arg, size_t stack_size, void *priv);
#define lwt_create(_fn, _a, _st) lwt_create_priv(_fn, _a, _st, NULL)

void __lwt_wake(lwt_t *lwt);

void lwt_exception_clear(void);

void lwt_wake(lwt_t *lwt);
void lwt_yield(void);
void lwt_sleep(void);
void lwt_pause(void);
void lwt_exit(void);


lwt_t *lwt_getcurrent(void);
void *lwt_getprivate(void);
void lwt_setprivate(void *ptr);

#define lwt_exception(__block)					\
	do {							\
		if (_setjmp(lwt_getcurrent()->xcptbuf)) {	\
			lwt_getcurrent()->flags &= ~LWTF_XCPT;	\
			{ __block }				\
		} else {					\
			lwt_getcurrent()->flags |= LWTF_XCPT;	\
		}						\
	} while(0)

#endif
