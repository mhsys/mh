#ifndef _sys_dirtio_h
#define _sys_dirtio_h

#define PORT_DIRTIO_IN		0x10000

#define DIRTIO_MAGIC 		0x74726976
#define DIRTIO_VER		0x00000000

#define PORT_DIRTIO_MAGIC	0x1
#define PORT_DIRTIO_VER		0x2
#define PORT_DIRTIO_ISR		0x3
#define PORT_DIRTIO_DSR		0x4

/* Per-queue */
#define PORT_DIRTIO_QMAX	0x10
#define PORT_DIRTIO_QSIZE	0x11
#define PORT_DIRTIO_READY 	0x12
#define PORT_DIRTIO_NOTIFY	0x13



#include <sys/dirtio_types.h>
#include <sys/dirtio_config.h>
#include <sys/dirtio_ring.h>

struct dirtio_hdr {
  __dirtio32 ioval;
  struct dring ring;

};

struct dirtio_dev {
  unsigned queues;
  unsigned *qmax;
  unsigned *qsize;
  unsigned *qready;
  uint32_t isr; /* Interrupt Status Register */
  uint32_t dsr; /* Device Status Register */

  int (*notify)(unsigned queue);
};

void dirtio_dev_init(struct dirtio_dev *dev, unsigned queues,
		     unsigned *qmax, unsigned *qsize, unsigned *qready);

#endif