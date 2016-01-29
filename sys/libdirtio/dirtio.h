#ifndef _sys_dirtio_h
#define _sys_dirtio_h

#include <microkernel.h>
#include <sys/uio.h>
#include <machine/vmparam.h>
#include <uk/exttypes.h>

#define DIRTIO_VID		0x00f00ffa

#define DIRTIO_DID_CONSOLE	0x00000001

#define PORT_DIRTIO_IN		0x10000

#define DIRTIO_MAGIC 		0x74726964
#define DIRTIO_VER		0x00000000

#define PORT_DIRTIO_MAGIC	0x1
#define PORT_DIRTIO_VER		0x2
#define PORT_DIRTIO_DSR		0x3

/* Per-queue */
#define PORT_DIRTIO_QMAX	0x10
#define PORT_DIRTIO_QSIZE	0x11
#define PORT_DIRTIO_READY 	0x12
#define PORT_DIRTIO_NOTIFY	0x13
#define PORT_DIRTIO_QDESC	0x14
#define PORT_DIRTIO_QAVAIL	0x15
#define PORT_DIRTIO_QUSED	0x16

#include <sys/dirtio_types.h>
#include <sys/dirtio_config.h>
#include <sys/dirtio_ring.h>


/*
 * Used by the desc code to communicate between device and driver.
 *
 * Located at the beginning of I/O memory.
 */
struct dirtio_hdr {
  __dirtio32 ioval;
};


/*
 * Driver.
 */

struct diodevice;

struct dirtio_desc {
	int id;
	int eiosig;
	struct dirtio_hdr *hdr;

	uint64_t nameid;
	uint64_t vendorid;
	uint64_t deviceid;
	unsigned eioirq;
	int kq;
};

struct dioqueue {
	struct diodevice *dio;
	unsigned idx;
	unsigned *freedescs;
	struct dring dring;
};

struct diodevice {
	int id;
	void *buf;
	uint64_t *bmap;
	struct dirtio_desc desc;
	size_t hdrmax;
	size_t bufsz;
	unsigned nqueues;
	struct dioqueue dqueues[0];
};

int dirtio_desc_open(uint64_t nameid, void *map, struct dirtio_desc *dio);
int dirtio_mmio_inw(struct dirtio_desc *dio, uint32_t port, uint8_t queue,
		    uint64_t *val);
int dirtio_mmio_out(struct dirtio_desc *dio, uint32_t port, uint8_t queue,
		    uint64_t val);
int dirtio_mmio_outw(struct dirtio_desc *dio, uint32_t port, uint8_t queue,
		     uint64_t val);
void dirtio_desc_close(struct dirtio_desc *dio);

#define DIRTIO_BUFS(_d) ((PAGE_SIZE - (_d)->hdrmax) / (_d)->bufsz)
#define DIRTIO_BUFBMAPSZ(_d) ((DIRTIO_BUFS(_d) + 63) / 64)
#define DIRTIO_BUFBMAPMASK(_d) ~((1LL << (DIRTIO_BUFS(_d) % 64)) - 1)

void dioqueue_init(struct dioqueue *dq, unsigned num, void *ioptr,
		   struct diodevice *dio);

size_t dirtio_allocv(struct diodevice *dio, struct iovec *iovec, int num,
		     size_t len);


struct diodevice *dirtio_pipe_open(uint64_t nameid);

/*
 * Device descriptor.
 */

struct dioqueue_dev {
	struct dring_dev dring;
	unsigned last_seen_avail;
};

struct dirtio_dev {
	unsigned queues;
	unsigned *qmax;
	unsigned *qsize;
	unsigned *qready;
	uint32_t dsr; /* Device Status Register */


	struct dioqueue_dev *dioqueues;
	int (*notify)(unsigned id, unsigned queue);
};

struct dirtio_dev_iovec {
	unsigned idx;
	struct iovec iov[0];
};

void dirtio_dev_init(unsigned queues, int (*notify)(unsigned, unsigned),
		     unsigned *qmax, unsigned *qsize, unsigned *qready,
		     struct dioqueue_dev *dqs);
int dirtio_dev_creat(struct sys_creat_cfg *cfg, devmode_t mode);
int dioqueue_dev_getv(unsigned id, unsigned queue,
		      struct dirtio_dev_iovec **iovecp, unsigned *num);

int dirtio_dev_pipe_setup(uint64_t nameid, uint64_t deviceid, devmode_t mode);

#endif
