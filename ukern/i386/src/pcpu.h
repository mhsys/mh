#ifndef __i386_pcpu_h
#define __i386_pcpu_h

#include "i386.h"

struct pcpu {
	void *data;
	struct tss tss;
};

#endif
