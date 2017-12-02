#include <sys/rbtree.h>
#include <stdlib.h>
#include <stdio.h>
#include <mrg.h>
#include <mrg/blk.h>

static int blk_compare_nodes(void *ctx, const void *n1, const void *n2)
{
	const struct blkdisk *blk1 = (const struct blkdisk *)n1;
	const struct blkdisk *blk2 = (const struct blkdisk *)n2;

	if (blk1->nameid < blk2->nameid)
		return -1;
	if (blk1->nameid > blk2->nameid)
		return 1;
	return 0;
}

static int blk_compare_key(void *ctx, const void *n, const void *key)
{
	const struct blkdisk *blk = (const struct blkdisk *)n;
	const uint64_t nameid = *(const uint64_t *)key;

	if (nameid < blk->nameid)
		return 1;
	if (nameid > blk->nameid)
		return -1;
	return 0;
}

static const rb_tree_ops_t blk_rbtree_ops = {
	.rbto_compare_nodes = blk_compare_nodes,
	.rbto_compare_key = blk_compare_key,
	.rbto_node_offset = offsetof(struct blkdisk, rb_node),
	.rbto_context = NULL,
};

static int initialised = 0;
static rb_tree_t blk_rbtree;

void
blk_add(uint64_t disk, uint64_t blkdev, struct blkinfo *info, const struct blkops *ops, void *opsopq)
{
	struct blkdisk *blkdisk;

	if (!initialised) {
		rb_tree_init(&blk_rbtree, &blk_rbtree_ops);
		initialised = 1;
	}

	blkdisk = malloc(sizeof(*blkdisk));
	assert(blkdisk != NULL);
	blkdisk->nameid = disk;
	blkdisk->blkdevid = blkdev;
	blkdisk->info = *info;
	blkdisk->ops = ops;
	blkdisk->opq = opsopq;
	blkdisk->ref = 0;
	blkdisk->invalid = 0;
	rb_tree_insert_node(&blk_rbtree, (void *)blkdisk);

	printf("BLK: added device %s\n", unsquoze_inline(disk).str);
}

void
blk_del(uint64_t disk)
{
	struct blkdisk *blkdisk;

	blkdisk = rb_tree_find_node(&blk_rbtree, (void *) &disk);
	if (blkdisk == NULL) {
		printf("%s: blkdev %s does not exist.\n", __FUNCTION__, unsquoze_inline(disk).str);
		return;
	}

	blkdisk->invalid = 1;
	rb_tree_remove_node(&blk_rbtree, (void *) blkdisk);


	if (blkdisk->ref == 0)
		free(blkdisk);
}

uint64_t
blk_iter(uint64_t cur)
{
	struct blkdisk *blk;

	if (!cur) {
		blk = (struct blkdisk *)rb_tree_iterate(&blk_rbtree, NULL, RB_DIR_LEFT);
	} else {
		blk = (struct blkdisk *)rb_tree_find_node(&blk_rbtree, (void *) &cur);
		if (blk != NULL)
			blk = (struct  blkdisk *)rb_tree_iterate(&blk_rbtree, blk, RB_DIR_RIGHT);
	}

	if (blk == NULL)
		return 0;
	else
		return blk->nameid;
}

struct blkdisk *
blk_open(uint64_t disk)
{
	struct blkdisk *blk;

	if (!initialised) {
		rb_tree_init(&blk_rbtree, &blk_rbtree_ops);
		initialised = 1;
		return NULL;
	}

	blk = rb_tree_find_node(&blk_rbtree, (void *) &disk);
	if (blk == NULL) {
		printf("%s: blkdev %s does not exist.\n", __FUNCTION__, unsquoze_inline(disk).str);
		return NULL;
	}

	assert (!blk->invalid);
	blk->ref++;
	return blk;
}

int
blk_info(struct blkdisk *blk, struct blkinfo *info)
{
	if (blk->invalid)
		return -ENODEV;
	*info = blk->info;
}

int
blk_read(struct blkdisk *blk, void *addr, size_t sz, uint64_t blkid, size_t blkno, int evt, int *retp)
{
	if (blk->invalid)
		return -ENODEV;
	return blk->ops->blkrd(blk->opq, addr, sz, blkid, blkno, evt, retp);
}

int
blk_write(struct blkdisk *blk, uint64_t blkid, size_t blkno, void *addr, size_t sz, int evt, int *retp)
{
	if (blk->invalid)
		return -ENODEV;
	return blk->ops->blkwr(blk->opq, blkid, blkno, addr, sz, evt, retp);
}

void
blk_close(struct blkdisk *blk)
{
	blk->ref--;
	if (blk->invalid && !blk->ref) {
		/* We're the last reference. Cleanup. */
		free(blk);
	}
}
