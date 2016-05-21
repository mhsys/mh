#include <stdio.h>
#include <stdlib.h>
#include <microkernel.h>
#include <squoze.h>
#include <assert.h>
#include <sys/param.h>
#include <mrg.h>
#include <mrg/consio.h>
#include "internal.h"

struct device_list devices = SLIST_HEAD_INITIALIZER(devices);

void
devadd(struct sys_hwcreat_cfg *cfg)
{
	int i, ret;
	struct device *d;

	ret = sys_hwcreat(cfg, 0111);
	if (ret) {
		char *name = unsquoze(cfg->nameid);
		printf("Error adding device \"%s\": %d\n", name, ret);
		free(name);
		free(d);
	}

	d = malloc(sizeof(*d));
	assert(d != NULL);
	d->nameid = cfg->nameid;
	d->vendorid = cfg->vendorid;
	for (i = 0; i < MIN(DEVICEIDS,SYS_RDCFG_MAX_DEVIDS); i++)
		     d->deviceids[i] = cfg->deviceids[i];
	SLIST_INSERT_HEAD(&devices, d, list);
}

int
main()
{
	int ret;
	int consevt = -1;
	DEVICE *console;

	printf("MRG bootstrap initiated.\n");

	ret = pltacpi_init();
	if (ret < 0) {
		printf("ACPI platform initialization failed: %d\n", ret);
		return -1;
	}

	ret = pltpci_init();
	if (ret < 0)
		printf("PCI bus scan failed: %d\n", ret);

	/* ret = pltusb_init(); */

	/* fork and create console device. */
	ret = pltconsole_process();
	if (ret < 0) {
		printf("console process couldn't start: %d\n", ret);
		return ret;
	}
	printf("Console process started as pid %d\n", ret);

	/* fork and create kloggerd.
	 * ret = kloggerd_process(); */

	/* Become SIGCHILD handler */

	/* open console */
	console = dopen("console");
	while (console == NULL) {
		printf("(%d)...", ret);
		lwt_pause();
		console = dopen("console");
	}

	consevt = evtalloc();
	dirq(console, CONSIO_IRQ_KBDATA, consevt);

	uint64_t val = 0x4141414141414141LL;
	ret = dout(console, CONSIO_OUTDATA, val);
	if (ret < 0)
		return ret;

	ret = dout(console, IOPORT_QWORD(CONSIO_OUTDATA), val);
	ret = dout(console, IOPORT_BYTE(CONSIO_OUTDATA), '\n');
	ret = dout(console, IOPORT_BYTE(CONSIO_OUTDATA), val);
	ret = dout(console, IOPORT_QWORD(CONSIO_OUTDATA), val);
	ret = dout(console, IOPORT_QWORD(CONSIO_OUTDATA), val);
	ret = dout(console, IOPORT_QWORD(CONSIO_OUTDATA), val);
	ret = dout(console, IOPORT_QWORD(CONSIO_OUTDATA), val);
	ret = dout(console, IOPORT_QWORD(CONSIO_OUTDATA), val);

	while (1) {
		/* Request data */
		dout(console, IOPORT_BYTE(CONSIO_DEVSTS), CONSIO_DEVSTS_KBDVAL);
		evtwait(consevt);
		din(console, IOPORT_QWORD(CONSIO_KBDATA), &val);
		printf("%c", (char)val);
		evtclear(consevt);
	}

	ret = din(console, 1, &val);
	printf("val = %llx\n", val);

	/* start command loop
	 * ret = pltcommand_setup(); */
	lwt_sleep();
}
