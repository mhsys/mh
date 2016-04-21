#include <sys/null.h>
#include <sys/types.h>
#include <sys/queue.h>
#include <sys/errno.h>
#include <sys/bitops.h>
#include <microkernel.h>
#include <stdlib.h>
#include <assert.h>

#include <drex/preempt.h>

/*
 * IRQ queues and interrupts.
 */

typedef void (*intfn_t)(int);
unsigned __preemption_level = 0;
intfn_t handlers[MAX_EXTINTRS] = {0, };
static uint64_t free_intrs = ~(1LL); /* Disable allocation of INTCHLD */

int __sys_inthandler(int prio, uint64_t si, struct intframe *f)
{
	intfn_t fn;
	unsigned intr;

	while (si != 0) {
		intr = ffs64(si) - 1;
		si &= ~((uint64_t)1 << intr);

		printf("Sending event to INT %d\n", intr);
		assert(intr < MAX_EXTINTRS);
		fn = handlers[intr];

		if (fn == NULL)
			printf("INT%d: ignored\n");
		else
			fn(intr);
	}
	return 0;
}


unsigned
intalloc(void)
{
	unsigned intr;

	assert(free_intrs != 0);
	intr = ffs64(free_intrs) - 1;
	assert(intr < MAX_EXTINTRS);
	free_intrs &= ~((uint64_t)1 << intr);
	return intr;
}

void
intfree(unsigned intr)
{
	assert(intr < 64);
	free_intrs |= ((uint64_t)1 << intr);
}

void
inthandler(unsigned intr, void (*hdlr)(int))
{
	assert(intr < MAX_EXTINTRS);
	preempt_disable();
	handlers[intr] = hdlr;
	preempt_enable();
}
