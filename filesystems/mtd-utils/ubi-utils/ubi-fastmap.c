/*
 * Copyright (c) 2014-2015, Linux Foundation.  All Rights Reserved.
 *
 * Copyright (c) 2012 Linutronix GmbH
 * Author: Richard Weinberger <richard@nod.at>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See
 * the GNU General Public License for more details.
 *
 */

#define PROGRAM_NAME	"ubi-fastmap"

#include <sys/stat.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <getopt.h>
#include <string.h>
#include <fcntl.h>
#include <mtd_swab.h>
#include <crc32.h>

#include <mtd/ubi-media.h>
#include <mtd/ubi-user.h>
#include <libubigen.h>
#include "common.h"
#include "ubiutils-common.h"

#include <linux/list.h>

#include "include/ubi-fastmap.h"

/* Used only for dbg. Will be removed later. */
void dbg_dump_fs_data(struct list_head *used, int used_cnt,
		struct ubigen_vol_info *vi, int nsects, int peb_cnt)
{
	int i, j;
	struct ubi_wl_peb *peb;
	printf("--------------Collected FM data-----------------\n");
	printf("Used lst (cnt = %d)\n", used_cnt);
	list_for_each_entry(peb, used, list)
		printf("[%d,%d], ", peb->pnum, peb->ec);
	printf("\n\nGo Over volumes (nsects = %d)", nsects);
	for (i = 0; i < nsects; i++){
		printf("\nVol-%d:\n", vi[i].id);
		for (j = 0; j < peb_cnt; j++)
			if (vi[i].pebs && vi[i].pebs[j] != -1)
				printf("[%d,%d], ", j, vi[i].pebs[j]);
	}
	printf("\n");
}

#define roundup(x, y) (					\
{							\
	const typeof(y) __y = y;			\
	(((x) + (__y - 1)) / __y) * __y;		\
}							\
)

/**
 * ubi_calc_fm_size - calculates the fastmap size in bytes
 * @ui: libubigen information (device description object)
 *
 * The calculated size is rounded up to LEB size.
 */
static size_t ubi_calc_fm_size(struct ubigen_info *ui)
{
	size_t size;

	size = sizeof(struct ubi_fm_sb) + \
		sizeof(struct ubi_fm_hdr) + \
		sizeof(struct ubi_fm_scan_pool) + \
		sizeof(struct ubi_fm_scan_pool) + \
		(ui->max_leb_count * sizeof(struct ubi_fm_ec)) + \
		(sizeof(struct ubi_fm_eba) + \
		(ui->max_leb_count * sizeof(__be32))) + \
		sizeof(struct ubi_fm_volhdr) * UBI_MAX_VOLUMES;
	return roundup(size, ui->leb_size);
}

static void init_vid_hdr(const struct ubigen_info *ui,
			 struct ubi_vid_hdr *hdr, uint32_t vol_id)
{
	uint32_t crc;

	hdr->vol_type = UBI_VID_DYNAMIC;
	hdr->vol_id = cpu_to_be32(vol_id);
	hdr->compat = UBI_COMPAT_DELETE;
	hdr->magic = cpu_to_be32(UBI_VID_HDR_MAGIC);
	hdr->version = ui->ubi_ver;

	crc = mtd_crc32(UBI_CRC32_INIT, hdr, UBI_VID_HDR_SIZE_CRC);
	hdr->hdr_crc = cpu_to_be32(crc);
}

static int write_new_fm_to_file(struct ubigen_info *ui,
		struct ubi_fastmap_layout *new_fm,
		void *fm_raw, int fm_size, int out)
{
	struct ubi_vid_hdr *avhdr;
	char *outbuf;
	int ret, i;

	outbuf = malloc(ui->peb_size);
	if (!outbuf) {
		sys_errmsg("cannot allocate %d bytes of memory", ui->peb_size);
		ret = -ENOMEM;
		goto out_free;
	}

	avhdr = (struct ubi_vid_hdr *)(&outbuf[ui->vid_hdr_offs]);
	for (i = 0; i < new_fm->used_blocks; i++) {
		memset(outbuf, 0xFF, ui->data_offs);
		ubigen_init_ec_hdr(ui, (struct ubi_ec_hdr *)outbuf,
				new_fm->e[0].ec);

		memset(avhdr, 0, sizeof(struct ubi_vid_hdr));
		avhdr->sqnum = cpu_to_be32(1);
		avhdr->lnum = cpu_to_be32(i);
		init_vid_hdr(ui, avhdr, UBI_FM_SB_VOLUME_ID);

		if (i * ui->leb_size > fm_size) {
			sys_errmsg("memory leak!");
			ret = -ENOMEM;
			goto out_free;
		}
		memcpy(outbuf + ui->data_offs,
				fm_raw + i * ui->leb_size, ui->leb_size);

		if (lseek(out, (off_t)(new_fm->e[i].pnum * ui->peb_size),
				SEEK_SET) != (off_t)(new_fm->e[i].pnum * ui->peb_size)) {
			sys_errmsg("cannot seek output file");
			ret = -EPERM;
			goto out_free;
		}
		if (write(out, outbuf, ui->peb_size) != ui->peb_size) {
			sys_errmsg("cannot write %d bytes to the output file",
					ui->peb_size);
			ret = -EPERM;
			goto out_free;
		}
	}
	ret = 0;

out_free:
	free(outbuf);
	return ret;
}

static void *generate_fm_raw_data(struct ubigen_info *ui, int fm_size,
		struct list_head *used, int fs_start_peb, int ec,
		struct ubigen_vol_info *vi, int nsects)
{
	size_t fm_pos = 0;
	void *fm_raw;
	struct ubi_fm_sb *fmsb;
	struct ubi_fm_hdr *fmh;
	struct ubi_fm_scan_pool *fmpl1, *fmpl2;
	struct ubi_fm_ec *fec;
	struct ubi_fm_volhdr *fvh;
	struct ubi_fm_eba *feba;
	int i, j, free_peb_count, used_peb_count, vol_count;
	int max_pool_size;
	struct ubi_wl_peb *used_peb;

	fm_raw = malloc(fm_size);
	if (!fm_raw)
		return NULL;

	memset(fm_raw, 0, fm_size);
	fmsb = (struct ubi_fm_sb *)fm_raw;
	fm_pos += sizeof(*fmsb);
	if (fm_pos > fm_size)
		goto out_free;

	fmh = (struct ubi_fm_hdr *)(fm_raw + fm_pos);
	fm_pos += sizeof(*fmh);
	if (fm_pos > fm_size)
			goto out_free;

	fmsb->magic = cpu_to_be32(UBI_FM_SB_MAGIC);
	fmsb->version = UBI_FM_FMT_VERSION;
	fmsb->used_blocks = cpu_to_be32(fm_size / ui->leb_size);
	/* the max sqnum will be filled in while *reading* the fastmap */
	fmsb->sqnum = cpu_to_be32(1);

	fmh->magic = cpu_to_be32(UBI_FM_HDR_MAGIC);
	free_peb_count = 0;
	used_peb_count = 0;
	vol_count = 0;

	fmpl1 = (struct ubi_fm_scan_pool *)(fm_raw + fm_pos);
	fm_pos += sizeof(*fmpl1);
	fmpl1->magic = cpu_to_be32(UBI_FM_POOL_MAGIC);
	fmpl1->size = 0;
	/*
	 * fm_pool.max_size is 5% of the total number of PEBs but it's also
	 * between UBI_FM_MAX_POOL_SIZE and UBI_FM_MIN_POOL_SIZE.
	 */
	max_pool_size = min((ui->max_leb_count / 100) * 5,
						UBI_FM_MAX_POOL_SIZE);
	if (max_pool_size < UBI_FM_MIN_POOL_SIZE)
		max_pool_size = UBI_FM_MIN_POOL_SIZE;

	fmpl1->max_size = cpu_to_be16(max_pool_size);

	fmpl2 = (struct ubi_fm_scan_pool *)(fm_raw + fm_pos);
	fm_pos += sizeof(*fmpl2);
	fmpl2->magic = cpu_to_be32(UBI_FM_POOL_MAGIC);
	fmpl2->size = 0;
	fmpl2->max_size = cpu_to_be16(UBI_FM_WL_POOL_SIZE);

	/* free list */
	for (i = fs_start_peb + fm_size / ui->leb_size;
			i < ui->max_leb_count; i++) {
		fec = (struct ubi_fm_ec *)(fm_raw + fm_pos);
		fec->pnum = cpu_to_be32(i);
		fec->ec = cpu_to_be32(ec);
		free_peb_count++;
		fm_pos += sizeof(*fec);
		if (fm_pos > fm_size)
			goto out_free;
	}
	fmh->free_peb_count = cpu_to_be32(free_peb_count);
	/* go over used */
	list_for_each_entry(used_peb, used, list) {
		fec = (struct ubi_fm_ec *)(fm_raw + fm_pos);
		fec->pnum = cpu_to_be32(used_peb->pnum);
		fec->ec = cpu_to_be32(used_peb->ec);
		used_peb_count++;
		fm_pos += sizeof(*fec);
		if (fm_pos > fm_size)
			goto out_free;
	}
	fmh->used_peb_count = cpu_to_be32(used_peb_count);

	fmh->scrub_peb_count = 0;
	fmh->erase_peb_count = 0;

	/* Go over all the volumes */
	for (i = 0; i < nsects; i++) {
		vol_count++;
		fvh = (struct ubi_fm_volhdr *)(fm_raw + fm_pos);
		fm_pos += sizeof(*fvh);
		if (fm_pos > fm_size)
			goto out_free;

		fvh->magic = cpu_to_be32(UBI_FM_VHDR_MAGIC);
		fvh->vol_id = cpu_to_be32(vi[i].id);
		if (vi[i].type != UBI_VID_DYNAMIC &&
				vi[i].type != UBI_VID_STATIC)
			goto out_free;

		fvh->vol_type = (vi[i].type == UBI_VID_DYNAMIC ?
				UBI_DYNAMIC_VOLUME : UBI_STATIC_VOLUME);
		fvh->used_ebs = cpu_to_be32(vi[i].used_ebs);
		fvh->data_pad = cpu_to_be32(vi[i].data_pad);
		fvh->last_eb_bytes = cpu_to_be32(vi[i].usable_leb_size);

		feba = (struct ubi_fm_eba *)(fm_raw + fm_pos);
		fm_pos += sizeof(*feba) +
				(sizeof(__be32) * vi[i].reserved_pebs);
		if (fm_pos > fm_size)
			goto out_free;

		if (!vi[i].pebs)
			goto out_free;
		for (j = 0; j < vi[i].reserved_pebs; j++)
			feba->pnum[j] = cpu_to_be32(vi[i].pebs[j]);

		feba->reserved_pebs = cpu_to_be32(j);
		feba->magic = cpu_to_be32(UBI_FM_EBA_MAGIC);
	}
	fmh->vol_count = cpu_to_be32(vol_count);
	fmh->bad_peb_count = 0;

	return fm_raw;

out_free:
	free(fm_raw);
	return NULL;
}

/**
 * add_fastmap_data - Adds fastmap data to the generated image file
 * @ui: UBI device description object
 * @anchor_peb: number of PEB to write the fastmap anchor to
 * @fs_start_peb: number of the first free PEB that can be used
 * 		for fastmap data
 * @ec: erase counter value to set for fastmap PEBs
 * @used: list of used PEBs
 * @vi: Volumes info
 * @nsects: number of volumes at @vi
 * @out: output image file handler
 *
 * Returns 0 on success, < 0 indicates an internal error.
 */
int add_fastmap_data(struct ubigen_info *ui, int anchor_peb,
		int fs_start_peb, int ec, struct list_head *used,
		struct ubigen_vol_info *vi, int nsects, int out)
{
	void *fm_raw = NULL;
	struct ubi_fastmap_layout *new_fm;
	int ret, i;
	int fm_size = ubi_calc_fm_size(ui);
	struct ubi_fm_sb *fmsb;

	new_fm = malloc(sizeof(*new_fm));
	if (!new_fm) {
		ret = -ENOMEM;
		goto out_free;
	}

	new_fm->used_blocks = fm_size / ui->leb_size;

	if (new_fm->used_blocks > UBI_FM_MAX_BLOCKS) {
		sys_errmsg("fastmap too large");
		ret = -ENOSPC;
		goto out_free;
	}
	fm_raw = generate_fm_raw_data(ui, fm_size, used, fs_start_peb, ec,
			vi, nsects);
	if (!fm_raw) {
		sys_errmsg("fastmap too large");
		ret = -ENOSPC;
		goto out_free;
	}

	for (i = 1; i < new_fm->used_blocks - 1; i++) {
		new_fm->e[i].pnum = fs_start_peb + i - 1;
		new_fm->e[i].ec = ec;
	}
	new_fm->e[0].pnum = anchor_peb;
	new_fm->e[0].ec = ec;

	fmsb = (struct ubi_fm_sb *)fm_raw;
	for (i = 0; i < new_fm->used_blocks; i++) {
		fmsb->block_loc[i] = cpu_to_be32(new_fm->e[i].pnum);
		fmsb->block_ec[i] = cpu_to_be32(new_fm->e[i].ec);
	}

	fmsb->data_crc = 0;
	fmsb->data_crc = cpu_to_be32(mtd_crc32(UBI_CRC32_INIT,
						fm_raw, fm_size));
	ret = write_new_fm_to_file(ui, new_fm, fm_raw, fm_size, out);

out_free:
	free(fm_raw);
	free(new_fm);
	return ret;
}
