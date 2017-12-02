#include <mrg.h>
#include <sys/bitops.h>


/*
 * The plan:
 * 
 * Pretty much VMS
 *
 * an event can set up any of:
 *
 * - AST. an AST is an softirq that can be raised (need raise
 *   syscall).  an AST will occupy only on interrupt line, and be
 *   handled in software.
 *
 * - ECB. A control block (size decided by the creator of the event,
 *   event filled by the setter).
 *
 * - Wake list. LWTs will wait on an event.
 */

#define MAXEVTQWORD 3
#define MAXEVT (64 * MAXEVTQWORD)
static uint64_t alloc_evts[MAXEVTQWORD] = { 0, }; 
static uint64_t set_evts[MAXEVTQWORD] = {0, };
static lwt_t *wait_evts[MAXEVT] = {0, };

void evtfree(int evt)
{
	int i, j;

	i = evt/64;
	j = evt%64;

	preempt_disable();
	alloc_evts[i] &= ~(1 << j);
	preempt_enable();
}

int evtalloc(void)
{
	int i, evt = -1;;

	preempt_disable();
	for (i = 0; i < MAXEVTQWORD; i++) {
		if (alloc_evts[i] != ~(uint64_t)0) {
			evt = ffs64(~alloc_evts[i]) -1;
			alloc_evts[i] |= ((uint64_t)1 << evt);
			set_evts[i] &= ~((uint64_t)1 << evt);
			wait_evts[evt] = NULL;
			preempt_enable();
			return evt + i * 64;
		}
	}
	preempt_enable();

	assert(0 && "evt exhausted");
	return evt;
}

void evtset(int evt)
{
	preempt_disable();
	__evtset(evt);
	preempt_enable();
}

void __evtset(int evt)
{
	int i = evt / 64, r = evt % 64;

	assert(evt < MAXEVT);

	/* Interrupt context */
	set_evts[i] |= ((uint64_t)1 << r);
	if (wait_evts[evt] != NULL)
		__lwt_wake(wait_evts[evt]);
}

void evtwait(int evt)
{
	int i = evt / 64, r = evt % 64;

	assert(evt < MAXEVT);

	preempt_disable();
	/* Check event is already set. */
	if (set_evts[i] & ((uint64_t)1 << r)) {
		preempt_enable();
		return;
	}

	/* XXX: add to queue */
	assert(wait_evts[evt] == NULL);
	wait_evts[evt] = lwt_getcurrent();
	lwt_sleep();
	wait_evts[evt] = NULL;
	preempt_enable();
}

void evtast(int evt, void (*func)(void *), void *arg)
{
	int i = evt / 64, r = evt % 64;
	lwt_t *lwt;

	assert(evt < MAXEVT);

	lwt = lwt_create(func, (void *)arg, 1024);

	preempt_disable();
	/* XXX: add to queue */
	assert(wait_evts[evt] == NULL);
	wait_evts[evt] = lwt;

	/* Check event is already set. */
	if (set_evts[i] & ((uint64_t)1 << r)) {
		__lwt_wake(wait_evts[evt]);
		return;
	}
	preempt_enable();
}

void evtclear(int evt)
{
  	int i = evt / 64, r = evt % 64;


	assert(evt < MAXEVT);

	preempt_disable();
	set_evts[i] &= ~((uint64_t)1 << r);
	preempt_enable();
}
