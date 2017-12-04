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


#include <uk/types.h>
#include <uk/string.h>
#include <uk/heap.h>
#include <uk/logio.h>
#include <uk/cpu.h>
#include <machine/uk/pcpu.h>

cpumask_t cpus_active = 0;

static unsigned number_cpus = 0;
static unsigned cpu_phys_to_id[UKERN_MAX_PHYSCPUS] = { -1, };
static struct cpu_info *cpus[UKERN_MAX_CPUS] = { 0, };

unsigned cpuid_fromphys(unsigned physid)
{
	unsigned id;

	dbgassert(physid < UKERN_MAX_PHYSCPUS);
	id = cpu_phys_to_id[physid];
	dbgassert(id < UKERN_MAX_CPUS);
	return id;
}

struct cpu_info *cpuinfo_get(unsigned id)
{

	if (id >= UKERN_MAX_CPUS) {
		dprintf("CPU ID %02d too big\n", id);
		return NULL;
	} else if (id >= number_cpus) {
		dprintf("Requested non-active cpu %d\n", id);
		return NULL;
	}
	return cpus[id];
}

unsigned cpu_numpresent(void)
{
	return number_cpus;
}

int cpu_add(uint16_t physid, uint16_t platformid)
{
	int id;
	struct cpu_info *cpuinfo;

	if (physid >= UKERN_MAX_PHYSCPUS) {
		printf("CPU Phys ID %02x too big. Skipping.\n", physid);
		return -1;
	}

	if (number_cpus >= UKERN_MAX_CPUS) {
		printf("Too many CPUs. Skipping.\n");
		return -1;
	}

	id = number_cpus++;
	dprintf("Adding CPU %d (PHYS:%d, PLAT:%d)\n", id, physid, platformid);

	cpuinfo = heap_alloc(sizeof(struct cpu_info));
	cpuinfo->cpu_id = id;

	cpuinfo->phys_id = physid;
	cpuinfo->plat_id = platformid;


	cpuinfo->self = cpuinfo;
	TAILQ_INIT(&cpuinfo->resched);


	cpus[id] = cpuinfo;
	cpu_phys_to_id[physid] = id;

	cpus_active |= ((cpumask_t) 1 << id);

	return id;
}

void cpu_nmi(int cpu)
{
	struct cpu_info *ci = cpuinfo_get(cpu);

	if (ci != NULL)
		pcpu_nmi(ci->phys_id);
}

void cpu_nmi_broadcast(void)
{
	pcpu_nmiall();
}

void cpu_nmi_mask(cpumask_t map)
{
	foreach_cpumask(map, cpu_nmi(i));
}

void cpu_ipi(int cpu, uint8_t vct)
{
	struct cpu_info *ci = cpuinfo_get(cpu);

	if (ci != NULL)
		pcpu_ipi(ci->phys_id, vct);
}

void cpu_ipi_broadcast(uint8_t vct)
{
	pcpu_ipiall(vct);
}

void cpu_ipi_mask(cpumask_t map, uint8_t vct)
{
	foreach_cpumask(map, cpu_ipi(i, vct));
}

void cpu_enter(void)
{
	unsigned cpuid;
	unsigned pcpuid;
	struct cpu_info *cpu;

	/* Setup Physical CPU */
	pcpuid = pcpu_init();

	/* Setup pCPU. */
	cpuid = cpuid_fromphys(pcpuid);
	cpu = cpuinfo_get(cpuid);
	pcpu_setup(&cpu->pcpu, cpu);
}

void cpu_wakeall(void)
{
	unsigned cpuid = cpu_number();
	int i;
	struct cpu_info *cpu;

	for (i = 0; i < number_cpus; i++) {
		if (i == cpuid)
			continue;
		printf("Waking up CPU %02d...", i);
		cpu = cpuinfo_get(i);
		pcpu_wake(cpu->phys_id);
	}

}
