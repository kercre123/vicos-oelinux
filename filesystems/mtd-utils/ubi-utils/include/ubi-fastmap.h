/*
 * Copyright (c) 2014-2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Copyright (c) International Business Machines Corp., 2006
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See
 * the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * Authors: Artem Bityutskiy (Битюцкий Артём)
 *          Thomas Gleixner
 *          Frank Haverkamp
 *          Oliver Lohmann
 *          Andreas Arnez
 */

#ifndef __UBI_FASTMAP_H__
#define __UBI_FASTMAP_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <asm/byteorder.h>
#include <libubigen.h>
#include <linux/list.h>

/* A fastmap can use up to UBI_FM_MAX_BLOCKS PEBs */
#define UBI_FM_MAX_BLOCKS	32

/* fastmap on-flash data structure format version */
#define UBI_FM_FMT_VERSION	1

#define UBI_FM_SB_MAGIC		0x7B11D69F
#define UBI_FM_HDR_MAGIC	0xD4B82EF7
#define UBI_FM_VHDR_MAGIC	0xFA370ED1
#define UBI_FM_POOL_MAGIC	0x67AF4D08
#define UBI_FM_EBA_MAGIC	0xf0c040a8

/**
 * struct ubi_fm_sb - UBI fastmap super block
 * @magic: fastmap super block magic number (%UBI_FM_SB_MAGIC)
 * @version: format version of this fastmap
 * @data_crc: CRC over the fastmap data
 * @used_blocks: number of PEBs used by this fastmap
 * @block_loc: an array containing the location of all PEBs of the fastmap
 * @block_ec: the erase counter of each used PEB
 * @sqnum: highest sequence number value at the time while taking the fastmap
 *
 */
struct ubi_fm_sb {
	__be32 magic;
	__u8 version;
	__u8 padding1[3];
	__be32 data_crc;
	__be32 used_blocks;
	__be32 block_loc[UBI_FM_MAX_BLOCKS];
	__be32 block_ec[UBI_FM_MAX_BLOCKS];
	__be64 sqnum;
	__u8 padding2[32];
}  __attribute__ ((packed));

/**
 * struct ubi_fm_hdr - header of the fastmap data set
 * @magic: fastmap header magic number (%UBI_FM_HDR_MAGIC)
 * @free_peb_count: number of free PEBs known by this fastmap
 * @used_peb_count: number of used PEBs known by this fastmap
 * @scrub_peb_count: number of to be scrubbed PEBs known by this fastmap
 * @bad_peb_count: number of bad PEBs known by this fastmap
 * @erase_peb_count: number of bad PEBs which have to be erased
 * @vol_count: number of UBI volumes known by this fastmap
 */
struct ubi_fm_hdr {
	__be32 magic;
	__be32 free_peb_count;
	__be32 used_peb_count;
	__be32 scrub_peb_count;
	__be32 bad_peb_count;
	__be32 erase_peb_count;
	__be32 vol_count;
	__u8 padding[4];
}  __attribute__ ((packed));

/* struct ubi_fm_hdr is followed by two struct ubi_fm_scan_pool */

/*
 * 5% of the total number of PEBs have to be scanned while attaching
 * from a fastmap.
 * But the size of this pool is limited to be between UBI_FM_MIN_POOL_SIZE and
 * UBI_FM_MAX_POOL_SIZE
 */
#define UBI_FM_MIN_POOL_SIZE	8
#define UBI_FM_MAX_POOL_SIZE	256

#define UBI_FM_WL_POOL_SIZE	25

/**
 * struct ubi_fm_scan_pool - Fastmap pool PEBs to be scanned while attaching
 * @magic: pool magic numer (%UBI_FM_POOL_MAGIC)
 * @size: current pool size
 * @max_size: maximal pool size
 * @pebs: an array containing the location of all PEBs in this pool
 */
struct ubi_fm_scan_pool {
	__be32 magic;
	__be16 size;
	__be16 max_size;
	__be32 pebs[UBI_FM_MAX_POOL_SIZE];
	__be32 padding[4];
}  __attribute__ ((packed));

/* ubi_fm_scan_pool is followed by nfree+nused struct ubi_fm_ec records */

/**
 * struct ubi_fm_ec - stores the erase counter of a PEB
 * @pnum: PEB number
 * @ec: ec of this PEB
 */
struct ubi_fm_ec {
	__be32 pnum;
	__be32 ec;
}  __attribute__ ((packed));

/**
 * struct ubi_wl_peb - in-memory representation of a used PEB
 * @pnum: PEB number
 * @ec: ec of this PEB
 * @used: link to the used list
 */
struct ubi_wl_peb {
	int pnum;
	int ec;
	struct list_head list;
};

/**
 * struct ubi_fastmap_layout - in-memory fastmap data structure.
 * @e: PEBs used by the current fastmap
 * @to_be_tortured: if non-zero tortured this PEB
 * @used_blocks: number of used PEBs
 * @max_pool_size: maximal size of the user pool
 * @max_wl_pool_size: maximal size of the pool used by the WL sub-system
 */
struct ubi_fastmap_layout {
	struct ubi_wl_peb e[UBI_FM_MAX_BLOCKS];
	int used_blocks;
	int max_pool_size;
	int max_wl_pool_size;
};

/**
 * struct ubi_fm_volhdr - Fastmap volume header
 * it identifies the start of an eba table
 * @magic: Fastmap volume header magic number (%UBI_FM_VHDR_MAGIC)
 * @vol_id: volume id of the fastmapped volume
 * @vol_type: type of the fastmapped volume
 * @data_pad: data_pad value of the fastmapped volume
 * @used_ebs: number of used LEBs within this volume
 * @last_eb_bytes: number of bytes used in the last LEB
 */
struct ubi_fm_volhdr {
	__be32 magic;
	__be32 vol_id;
	__u8 vol_type;
	__u8 padding1[3];
	__be32 data_pad;
	__be32 used_ebs;
	__be32 last_eb_bytes;
	__u8 padding2[8];
}  __attribute__ ((packed));

/* struct ubi_fm_volhdr is followed by one struct ubi_fm_eba records */

/**
 * struct ubi_fm_eba - denotes an association beween a PEB and LEB
 * @magic: EBA table magic number
 * @reserved_pebs: number of table entries
 * @pnum: PEB number of LEB (LEB is the index)
 */
struct ubi_fm_eba {
	__be32 magic;
	__be32 reserved_pebs;
	__be32 pnum[0];
}  __attribute__ ((packed));

void dump_fs_data(struct list_head *used, int used_cnt,
		struct ubigen_vol_info *vi, int nsects, int peb_cnt);

int add_fastmap_data(struct ubigen_info *ui, int anchor_peb,
		int fs_start_peb, int ec, struct list_head *used,
		struct ubigen_vol_info *vi, int nsects, int out);

#ifdef __cplusplus
}
#endif

#endif /* !__UBI_FASTMAP_H__ */

