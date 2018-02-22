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


#ifndef _uk_sys_h_
#define _uk_sys_h_

#define SYS_DEVCFG_MAXDEVIDS 8
#define SYS_DEVCONFIG_MAXUSERCFG 2

/*
 * Syscalls
 */
#define SYS_DIE 0
#define SYS_INTHDLR 1
#define SYS_IRET 2
#define SYS_STI 3
#define SYS_CLI 4
#define SYS_WAIT 5
#define SYS_FORK 6
#define SYS_YIELD 7

#ifndef _ASSEMBLER
struct sys_childstat {
	int exit_status;
};
#endif
#define SYS_CHILDSTAT 8
#define SYS_GETPID 9
#define SYS_RAISE 10
#define SYS_TLS 11

#ifndef _ASSEMBLER
typedef enum {
	MAP_NEW = 0x100,
	MAP_NEW32 = 0x200,
	MAP_NONE = 0,
	MAP_RDONLY = 1,
	MAP_RDEXEC = 2,
	MAP_WRITE = 3,
	MAP_WREXEC = 4,
} sys_map_flags_t;
#endif
#define SYS_MAP  0x10
#define SYS_MOVE 0x11

#ifndef _ASSEMBLER
struct sys_info_cfg {
#define SYS_INFO_IRQSEG(_cfg, _i) ((_cfg)->segs[(_i)])
#define SYS_INFO_IOSEG(_cfg, _i) ((_cfg)->segs[(_cfg)->nirqsegs + (_i)])
#define SYS_INFO_MAX_DEVIDS SYS_DEVCFG_MAXDEVIDS
#define SYS_INFO_MAX_SEGMENTS 24
#define SYS_INFO_MAX_MEMSEGMENTS 24
#define SYS_INFO_FLAGS_HW    1
#define SYS_INFO_FLAGS_HWPCI 2
	uint64_t nameid;
	uint64_t vendorid;
	uint64_t deviceids[SYS_INFO_MAX_DEVIDS];

	uint8_t niopfns;
	uint8_t nirqsegs;
	uint8_t npiosegs;
	uint8_t nmemsegs;
	uint8_t flags;

	struct sys_info_seg {
		uint16_t base;
		uint16_t len;
	} segs[SYS_INFO_MAX_SEGMENTS];

	struct sys_info_memseg {
		uint64_t base;
		uint32_t len;
	} memsegs[SYS_INFO_MAX_MEMSEGMENTS];
};
#endif
#define SYS_OPEN    0x20
#define SYS_MAPIRQ  0x21
/* Platform device port mangling */
#define IOPORT_SIZESHIFT 2
#define IOPORT_SIZEMASK 0x3
#define IOPORT_BYTE(_x) (((_x) << IOPORT_SIZESHIFT) | 0)
#define IOPORT_WORD(_x) (((_x) << IOPORT_SIZESHIFT) | 1)
#define IOPORT_DWORD(_x) (((_x) << IOPORT_SIZESHIFT) | 2)
#define IOPORT_QWORD(_x) (((_x) << IOPORT_SIZESHIFT) | 3)
#define SYS_IN      0x22
#define SYS_OUT     0x23
#define SYS_EXPORT  0x24
#define SYS_UNEXPORT 0x25
#define SYS_INFO    0x26
#define SYS_IOMAP   0x27
#define SYS_IOUNMAP 0x28
#define SYS_OPEN32  0x29
#define SYS_OUT32   0x2A
#define SYS_RDCFG   0x2B
#define SYS_WRCFG   0x2C
#define SYS_EOI     0x2D
#define SYS_CLOSE  0x2F

#define SYS_CREAT_CFG_MAXUSERCFG SYS_DEVCONFIG_MAXUSERCFG
#ifndef _ASSEMBLER
struct sys_creat_cfg {
	uint64_t nameid;
	uint32_t vendorid;
	uint32_t deviceid;
	uint8_t nirqs;

	uint64_t usercfg[SYS_CREAT_CFG_MAXUSERCFG];
};

enum sys_poll_ior_op {
	SYS_POLL_OP_OPEN,
	SYS_POLL_OP_CLONE,
	SYS_POLL_OP_OUT,
	SYS_POLL_OP_CLOSE,
};

struct sys_poll_ior {
	enum sys_poll_ior_op op;
	union {
		struct {
			uint8_t size;
			uint16_t port;
			uint64_t val;
		};
		struct {
			unsigned clone_id;
		};
	};
	pid_t pid;
	uid_t uid;
	gid_t gid;
};
#endif
#define SYS_CREAT  0x30
#define SYS_POLL   0x31
#define SYS_WRIOSPC 0x32
#define SYS_IRQ    0x33
#define SYS_READ   0x34
#define SYS_WRITE  0x35

#ifndef _ASSEMBLER
#define SYS_HWCREAT_MAX_DEVIDS SYS_DEVCFG_MAXDEVIDS
#define SYS_HWCREAT_MAX_SEGMENTS 24
#define SYS_HWCREAT_MAX_MEMSEGMENTS 24

struct sys_hwcreat_cfg {
	uint64_t nameid;
	uint64_t vendorid;
	uint64_t deviceids[SYS_HWCREAT_MAX_DEVIDS];

	uint8_t nirqsegs;
	uint8_t npiosegs;
	uint8_t nmemsegs;

	struct sys_hwcreat_seg {
		uint16_t base;
		uint16_t len;
	} segs[SYS_HWCREAT_MAX_SEGMENTS];

	struct sys_hwcreat_memseg {
		uint64_t base;
		uint32_t len;
	} memsegs[SYS_HWCREAT_MAX_MEMSEGMENTS];
};
#endif
#define SYS_HWCREAT  0x40

#define SYS_GETUID   0x50
#define SYS_SETUID   0x51
#define SYS_SETEUID  0x52
#define SYS_SETSUID  0x53
#define SYS_ISSETUID 0x54

#define SYS_GETGID   0x60
#define SYS_SETGID   0x61
#define SYS_SETEGID  0x62
#define SYS_SETSGID  0x63

/* System-processes only */
#define SYS_PUTC 0x1000

/*
 * Exceptions
 */
#define XCPT_PGFAULT 0
#define PG_ERR_REASON_NOTP  0
#define PG_ERR_REASON_PROT  1
#define PG_ERR_REASON_MASK  1
#define PG_ERR_INFO_COW   128
#define PG_ERR_INFO_WRITE 256

#define INTR_EXT     64
#define INTR_CHILD   0
#define MAX_EXTINTRS 64

/*
 * System Device
 */

/* I/O Ports */
#define SYSDEVIO_TMRPRD 0 /* Read */
#define SYSDEVIO_RTTCNT 1 /* Read */
#define SYSDEVIO_RTTALM 1 /* Write */
#define SYSDEVIO_VTTCNT 2 /* Read */
#define SYSDEVIO_VTTALM 2 /* Write */

#define SYSDEVIO_CONSON 0x100 /* Write */

/* Interrupts */
#define SYSDEVIO_RTTINT 0
#define SYSDEVIO_VTTINT 1

/*
 * Kernel Log Device
 */

/* I/O Ports */
#define KLOGDEVIO_GETC   0 /* Read */
#define KLOGDEVIO_SZ     1 /* Read */
#define KLOGDEVIO_IE     2 /* Read/Write */
#define KLOGDEVIO_ISR    3 /* Read/Write */
/* Interrupts */
#define KLOGDEVIO_AVLINT 0

#ifdef _UKERNEL
int sys_call(int sc, unsigned long a1, unsigned long a2,
	     unsigned long a3, unsigned long a4, unsigned long a5);
#endif
#ifndef _ASSEMBLER
#include <uk/exttypes.h>
#endif

#endif
