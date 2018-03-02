/*
 * Copyright (c) 2013, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#define _LARGEFILE64_SOURCE /* enable lseek64() */
/******************************************************************************
 * INCLUDE SECTION
 ******************************************************************************/
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/fs.h>
#include <limits.h>
#include <dirent.h>
#include <inttypes.h>
#include <linux/kernel.h>
#include <asm/byteorder.h>
#include <map>
#include <vector>
#include <string>
#define LOG_TAG "gpt-utils"
#include <cutils/log.h>
#include <cutils/properties.h>
#include "gpt-utils.h"
#include <endian.h>
#include <zlib.h>
/******************************************************************************
 * DEFINE SECTION
 ******************************************************************************/
#define BLK_DEV_FILE    "/dev/block/mmcblk0"
/* list the names of the backed-up partitions to be swapped */
/* extension used for the backup partitions - tzbak, abootbak, etc. */
#define BAK_PTN_NAME_EXT    "bak"
/* GPT defines */
#define MAX_LUNS                    26
//This will allow us to get the root lun path from the path to the partition.
//i.e: from /dev/block/sdaXXX get /dev/block/sda. The assumption here is that
//the boot critical luns lie between sda to sdz which is acceptable because
//only user added external disks,etc would lie beyond that limit which do not
//contain partitions that interest us here.
#define PATH_TRUNCATE_LOC (sizeof("/dev/block/sda") - 1)
//From /dev/block/sda get just sda
#define LUN_NAME_START_LOC (sizeof("/dev/block/") - 1)
#define BOOT_LUN_A_ID 1
#define BOOT_LUN_B_ID 2
/******************************************************************************
 * MACROS
 ******************************************************************************/
#define GET_4_BYTES(ptr)    ((uint32_t) *((uint8_t *)(ptr)) | \
        ((uint32_t) *((uint8_t *)(ptr) + 1) << 8) | \
        ((uint32_t) *((uint8_t *)(ptr) + 2) << 16) | \
        ((uint32_t) *((uint8_t *)(ptr) + 3) << 24))
#define GET_8_BYTES(ptr)    ((uint64_t) *((uint8_t *)(ptr)) | \
        ((uint64_t) *((uint8_t *)(ptr) + 1) << 8) | \
        ((uint64_t) *((uint8_t *)(ptr) + 2) << 16) | \
        ((uint64_t) *((uint8_t *)(ptr) + 3) << 24) | \
        ((uint64_t) *((uint8_t *)(ptr) + 4) << 32) | \
        ((uint64_t) *((uint8_t *)(ptr) + 5) << 40) | \
        ((uint64_t) *((uint8_t *)(ptr) + 6) << 48) | \
        ((uint64_t) *((uint8_t *)(ptr) + 7) << 56))
#define PUT_4_BYTES(ptr, y)   *((uint8_t *)(ptr)) = (y) & 0xff; \
        *((uint8_t *)(ptr) + 1) = ((y) >> 8) & 0xff; \
        *((uint8_t *)(ptr) + 2) = ((y) >> 16) & 0xff; \
        *((uint8_t *)(ptr) + 3) = ((y) >> 24) & 0xff;
/******************************************************************************
 * TYPES
 ******************************************************************************/
using namespace std;
enum gpt_state {
    GPT_OK = 0,
    GPT_BAD_SIGNATURE,
    GPT_BAD_CRC
};

/******************************************************************************
 * FUNCTIONS
 ******************************************************************************/
/**
 *  ==========================================================================
 *
 *  \brief  Read/Write len bytes from/to block dev
 *
 *  \param [in] fd      block dev file descriptor (returned from open)
 *  \param [in] rw      RW flag: 0 - read, != 0 - write
 *  \param [in] offset  block dev offset [bytes] - RW start position
 *  \param [in] buf     Pointer to the buffer containing the data
 *  \param [in] len     RW size in bytes. Buf must be at least that big
 *
 *  \return  0 on success
 *
 *  ==========================================================================
 */
static int blk_rw(int fd, int rw, int64_t offset, uint8_t *buf, unsigned len)
{
    int r;
    if (lseek64(fd, offset, SEEK_SET) < 0) {
        fprintf(stderr, "block dev lseek64 %" PRIi64 " failed: %s\n", offset,
                strerror(errno));
        return -1;
    }
    if (rw)
        r = write(fd, buf, len);
    else
        r = read(fd, buf, len);
    if (r < 0)
        fprintf(stderr, "block dev %s failed: %s\n", rw ? "write" : "read",
                strerror(errno));
    else
        r = 0;
    return r;
}
/**
 *  ==========================================================================
 *
 *  \brief  Search within GPT for partition entry with the given name
 *  or it's backup twin (name-bak).
 *
 *  \param [in] ptn_name        Partition name to seek
 *  \param [in] pentries_start  Partition entries array start pointer
 *  \param [in] pentries_end    Partition entries array end pointer
 *  \param [in] pentry_size     Single partition entry size [bytes]
 *
 *  \return  First partition entry pointer that matches the name or NULL
 *
 *  ==========================================================================
 */
static uint8_t *gpt_pentry_seek(const char *ptn_name,
                                const uint8_t *pentries_start,
                                const uint8_t *pentries_end,
                                uint32_t pentry_size)
{
    char *pentry_name;
    unsigned len = strlen(ptn_name);
    for (pentry_name = (char *) (pentries_start + PARTITION_NAME_OFFSET);
         pentry_name < (char *) pentries_end; pentry_name += pentry_size) {
        char name8[MAX_GPT_NAME_SIZE / 2];
        unsigned i;
        /* Partition names in GPT are UTF-16 - ignoring UTF-16 2nd byte */
        for (i = 0; i < sizeof(name8); i++)
            name8[i] = pentry_name[i * 2];
        if (!strncmp(ptn_name, name8, len))
            if (name8[len] == 0 || !strcmp(&name8[len], BAK_PTN_NAME_EXT))
                return (uint8_t *) (pentry_name - PARTITION_NAME_OFFSET);
    }
    return NULL;
}


/**
 *  ==========================================================================
 *
 *  \brief  Checks GPT state (header signature and CRC)
 *
 *  \param [in] fd      block dev file descriptor
 *  \param [in] gpt     GPT header to be checked
 *  \param [out] state  GPT header state
 *
 *  \return  0 on success
 *
 *  ==========================================================================
 */
static int gpt_get_state(int fd, enum gpt_instance gpt, enum gpt_state *state)
{
    int64_t gpt_header_offset;
    uint32_t gpt_header_size;
    uint8_t  *gpt_header = NULL;
    uint32_t crc;
    uint32_t blk_size = 0;
    *state = GPT_OK;
    if (ioctl(fd, BLKSSZGET, &blk_size) != 0) {
            fprintf(stderr, "Failed to get GPT device block size: %s\n",
                            strerror(errno));
            goto error;
    }
    gpt_header = (uint8_t*)malloc(blk_size);
    if (!gpt_header) {
            fprintf(stderr, "gpt_get_state:Failed to alloc memory for header\n");
            goto error;
    }
    if (gpt == PRIMARY_GPT)
        gpt_header_offset = blk_size;
    else {
        gpt_header_offset = lseek64(fd, 0, SEEK_END) - blk_size;
        if (gpt_header_offset < 0) {
            fprintf(stderr, "gpt_get_state:Seek to end of GPT part fail\n");
            goto error;
        }
    }
    if (blk_rw(fd, 0, gpt_header_offset, gpt_header, blk_size)) {
        fprintf(stderr, "gpt_get_state: blk_rw failed\n");
        goto error;
    }
    if (memcmp(gpt_header, GPT_SIGNATURE, sizeof(GPT_SIGNATURE)))
        *state = GPT_BAD_SIGNATURE;
    gpt_header_size = GET_4_BYTES(gpt_header + HEADER_SIZE_OFFSET);
    crc = GET_4_BYTES(gpt_header + HEADER_CRC_OFFSET);
    /* header CRC is calculated with this field cleared */
    PUT_4_BYTES(gpt_header + HEADER_CRC_OFFSET, 0);
    if (crc32(0, gpt_header, gpt_header_size) != crc)
        *state = GPT_BAD_CRC;
    free(gpt_header);
    return 0;
error:
    if (gpt_header)
            free(gpt_header);
    return -1;
}
/**
 *  ==========================================================================
 *
 *  \brief  Sets GPT header state (used to corrupt and fix GPT signature)
 *
 *  \param [in] fd     block dev file descriptor
 *  \param [in] gpt    GPT header to be checked
 *  \param [in] state  GPT header state to set (GPT_OK or GPT_BAD_SIGNATURE)
 *
 *  \return  0 on success
 *
 *  ==========================================================================
 */
static int gpt_set_state(int fd, enum gpt_instance gpt, enum gpt_state state)
{
    int64_t gpt_header_offset;
    uint32_t gpt_header_size;
    uint8_t  *gpt_header = NULL;
    uint32_t crc;
    uint32_t blk_size = 0;
    if (ioctl(fd, BLKSSZGET, &blk_size) != 0) {
            fprintf(stderr, "Failed to get GPT device block size: %s\n",
                            strerror(errno));
            goto error;
    }
    gpt_header = (uint8_t*)malloc(blk_size);
    if (!gpt_header) {
            fprintf(stderr, "Failed to alloc memory for gpt header\n");
            goto error;
    }
    if (gpt == PRIMARY_GPT)
        gpt_header_offset = blk_size;
    else {
        gpt_header_offset = lseek64(fd, 0, SEEK_END) - blk_size;
        if (gpt_header_offset < 0) {
            fprintf(stderr, "Failed to seek to end of GPT device\n");
            goto error;
        }
    }
    if (blk_rw(fd, 0, gpt_header_offset, gpt_header, blk_size)) {
        fprintf(stderr, "Failed to r/w gpt header\n");
        goto error;
    }
    if (state == GPT_OK)
        memcpy(gpt_header, GPT_SIGNATURE, sizeof(GPT_SIGNATURE));
    else if (state == GPT_BAD_SIGNATURE)
        *gpt_header = 0;
    else {
        fprintf(stderr, "gpt_set_state: Invalid state\n");
        goto error;
    }
    gpt_header_size = GET_4_BYTES(gpt_header + HEADER_SIZE_OFFSET);
    /* header CRC is calculated with this field cleared */
    PUT_4_BYTES(gpt_header + HEADER_CRC_OFFSET, 0);
    crc = crc32(0, gpt_header, gpt_header_size);
    PUT_4_BYTES(gpt_header + HEADER_CRC_OFFSET, crc);
    if (blk_rw(fd, 1, gpt_header_offset, gpt_header, blk_size)) {
        fprintf(stderr, "gpt_set_state: blk write failed\n");
        goto error;
    }
    return 0;
error:
    if(gpt_header)
           free(gpt_header);
    return -1;
}


//Given a parttion name(eg: rpm) get the path to the block device that
//represents the GPT disk the partition resides on. In the case of emmc it
//would be the default emmc dev(/dev/mmcblk0).
static int get_dev_path_from_partition_name(const char *partname,
                char *buf,
                size_t buflen)
{
        struct stat st;
        char path[PATH_MAX] = {0};
        if (!partname || !buf || buflen < ((PATH_TRUNCATE_LOC) + 1)) {
                ALOGE("%s: Invalid argument", __func__);
                goto error;
        }
        snprintf(buf, buflen, "/dev/mmcblk0");
        return 0;
error:
        return -1;
}
int gpt_utils_get_partition_map(vector<string>& ptn_list,
                map<string, vector<string> >& partition_map) {
        char devpath[PATH_MAX] = {'\0'};
        map<string, vector<string> >::iterator it;
        if (ptn_list.size() < 1) {
                fprintf(stderr, "%s: Invalid ptn list\n", __func__);
                return -1;
        }
        //Go through the passed in list
        for (uint32_t i = 0; i < ptn_list.size(); i++)
        {
                //Key in the map is the path to the device that holds the
                //partition
                if (get_dev_path_from_partition_name(ptn_list[i].c_str(),
                                devpath,
                                sizeof(devpath))) {
                        //Not necessarily an error. The partition may just
                        //not be present.
                        continue;
                }
                string path = devpath;
                it = partition_map.find(path);
                if (it != partition_map.end()) {
                        it->second.push_back(ptn_list[i]);
                } else {
                        vector<string> str_vec;
                        str_vec.push_back( ptn_list[i]);
                        partition_map.insert(pair<string, vector<string> >
                                        (path, str_vec));
                }
                memset(devpath, '\0', sizeof(devpath));
        }
        return 0;
}
//Get the block size of the disk represented by decsriptor fd
static uint32_t gpt_get_block_size(int fd)
{
        uint32_t block_size = 0;
        if (fd < 0) {
                ALOGE("%s: invalid descriptor",
                                __func__);
                goto error;
        }
        if (ioctl(fd, BLKSSZGET, &block_size) != 0) {
                ALOGE("%s: Failed to get GPT dev block size : %s",
                                __func__,
                                strerror(errno));
                goto error;
        }
        return block_size;
error:
        return 0;
}
//Write the GPT header present in the passed in buffer back to the
//disk represented by fd
static int gpt_set_header(uint8_t *gpt_header, int fd,
                enum gpt_instance instance)
{
        uint32_t block_size = 0;
        off_t gpt_header_offset = 0;
        if (!gpt_header || fd < 0) {
                ALOGE("%s: Invalid arguments",
                                __func__);
                goto error;
        }
        block_size = gpt_get_block_size(fd);
        ALOGI("%s: Block size is : %d", __func__, block_size);
        if (block_size == 0) {
                ALOGE("%s: Failed to get block size", __func__);
                goto error;
        }
        if (instance == PRIMARY_GPT)
                gpt_header_offset = block_size;
        else
                gpt_header_offset = lseek64(fd, 0, SEEK_END) - block_size;
        if (gpt_header_offset <= 0) {
                ALOGE("%s: Failed to get gpt header offset",__func__);
                goto error;
        }
        ALOGI("%s: Writing back header to offset %ld", __func__,
                gpt_header_offset);
        if (blk_rw(fd, 1, gpt_header_offset, gpt_header, block_size)) {
                ALOGE("%s: Failed to write back GPT header", __func__);
                goto error;
        }
        return 0;
error:
        return -1;
}
//Read out the GPT header for the disk that contains the partition partname
static uint8_t* gpt_get_header(const char *partname, enum gpt_instance instance)
{
        uint8_t* hdr = NULL;
        char devpath[PATH_MAX] = {0};
        int64_t hdr_offset = 0;
        uint32_t block_size = 0;
        int fd = -1;
        if (!partname) {
                ALOGE("%s: Invalid partition name", __func__);
                goto error;
        }
        if (get_dev_path_from_partition_name(partname, devpath, sizeof(devpath))
                        != 0) {
                ALOGE("%s: Failed to resolve path for %s",
                                __func__,
                                partname);
                goto error;
        }
        fd = open(devpath, O_RDWR);
        if (fd < 0) {
                ALOGE("%s: Failed to open %s : %s",
                                __func__,
                                devpath,
                                strerror(errno));
                goto error;
        }
        block_size = gpt_get_block_size(fd);
        if (block_size == 0)
        {
                ALOGE("%s: Failed to get gpt block size for %s",
                                __func__,
                                partname);
                goto error;
        }
        hdr = (uint8_t*)malloc(block_size);
        if (!hdr) {
                ALOGE("%s: Failed to allocate memory for gpt header",
                                __func__);
        }
        if (instance == PRIMARY_GPT)
                hdr_offset = block_size;
        else {
                hdr_offset = lseek64(fd, 0, SEEK_END) - block_size;
        }
        if (hdr_offset < 0) {
                ALOGE("%s: Failed to get gpt header offset",
                                __func__);
                goto error;
        }
        if (blk_rw(fd, 0, hdr_offset, hdr, block_size)) {
                ALOGE("%s: Failed to read GPT header from device",
                                __func__);
                goto error;
        }
        close(fd);
        return hdr;
error:
        if (fd >= 0)
                close(fd);
        if (hdr)
                free(hdr);
        return NULL;
}
//Returns the partition entry array based on the
//passed in buffer which contains the gpt header.
//The fd here is the descriptor for the 'disk' which
//holds the partition
static uint8_t* gpt_get_pentry_arr(uint8_t *hdr, int fd)
{
        uint64_t pentries_start = 0;
        uint32_t pentry_size = 0;
        uint32_t block_size = 0;
        uint32_t pentries_arr_size = 0;
        uint8_t *pentry_arr = NULL;
        int rc = 0;
        if (!hdr) {
                ALOGE("%s: Invalid header", __func__);
                goto error;
        }
        if (fd < 0) {
                ALOGE("%s: Invalid fd", __func__);
                goto error;
        }
        block_size = gpt_get_block_size(fd);
        if (!block_size) {
                ALOGE("%s: Failed to get gpt block size for",
                                __func__);
                goto error;
        }
        pentries_start = GET_8_BYTES(hdr + PENTRIES_OFFSET) * block_size;
        pentry_size = GET_4_BYTES(hdr + PENTRY_SIZE_OFFSET);
        pentries_arr_size =
                GET_4_BYTES(hdr + PARTITION_COUNT_OFFSET) * pentry_size;
        pentry_arr = (uint8_t*)calloc(1, pentries_arr_size);
        if (!pentry_arr) {
                ALOGE("%s: Failed to allocate memory for partition array",
                                __func__);
                goto error;
        }
        rc = blk_rw(fd, 0,
                        pentries_start,
                        pentry_arr,
                        pentries_arr_size);
        if (rc) {
                ALOGE("%s: Failed to read partition entry array",
                                __func__);
                goto error;
        }
        return pentry_arr;
error:
        if (pentry_arr)
                free(pentry_arr);
        return NULL;
}
static int gpt_set_pentry_arr(uint8_t *hdr, int fd, uint8_t* arr)
{
        uint32_t block_size = 0;
        uint64_t pentries_start = 0;
        uint32_t pentry_size = 0;
        uint32_t pentries_arr_size = 0;
        int rc = 0;
        if (!hdr || fd < 0 || !arr) {
                ALOGE("%s: Invalid argument", __func__);
                goto error;
        }
        block_size = gpt_get_block_size(fd);
        if (!block_size) {
                ALOGE("%s: Failed to get gpt block size for",
                                __func__);
                goto error;
        }
        ALOGI("%s : Block size is %d", __func__, block_size);
        pentries_start = GET_8_BYTES(hdr + PENTRIES_OFFSET) * block_size;
        pentry_size = GET_4_BYTES(hdr + PENTRY_SIZE_OFFSET);
        pentries_arr_size =
                GET_4_BYTES(hdr + PARTITION_COUNT_OFFSET) * pentry_size;
        ALOGI("%s: Writing partition entry array of size %d to offset %" PRIu64,
                        __func__,
                        pentries_arr_size,
                        pentries_start);
        rc = blk_rw(fd, 1,
                        pentries_start,
                        arr,
                        pentries_arr_size);
        if (rc) {
                ALOGE("%s: Failed to read partition entry array",
                                __func__);
                goto error;
        }
        return 0;
error:
        return -1;
}
//Allocate a handle used by calls to the "gpt_disk" api's
struct gpt_disk * gpt_disk_alloc()
{
        struct gpt_disk *disk;
        disk = (struct gpt_disk *)malloc(sizeof(struct gpt_disk));
        if (!disk) {
                ALOGE("%s: Failed to allocate memory", __func__);
                goto end;
        }
        memset(disk, 0, sizeof(struct gpt_disk));
end:
        return disk;
}
//Free previously allocated/initialized handle
void gpt_disk_free(struct gpt_disk *disk)
{
        if (!disk)
                return;
        if (disk->hdr)
                free(disk->hdr);
        if (disk->hdr_bak)
                free(disk->hdr_bak);
        if (disk->pentry_arr)
                free(disk->pentry_arr);
        if (disk->pentry_arr_bak)
                free(disk->pentry_arr_bak);
        free(disk);
        return;
}
//fills up the passed in gpt_disk struct with information about the
//disk represented by path dev. Returns 0 on success and -1 on error.
int gpt_disk_get_disk_info(const char *dev, struct gpt_disk *dsk)
{
        struct gpt_disk *disk = NULL;
        int fd = -1;
        uint32_t gpt_header_size = 0;
        if (!dsk || !dev) {
                ALOGE("%s: Invalid arguments", __func__);
                goto error;
        }
        disk = dsk;
        disk->hdr = gpt_get_header(dev, PRIMARY_GPT);
        if (!disk->hdr) {
                ALOGE("%s: Failed to get primary header", __func__);
                goto error;
        }
        gpt_header_size = GET_4_BYTES(disk->hdr + HEADER_SIZE_OFFSET);
        disk->hdr_crc = crc32(0, disk->hdr, gpt_header_size);
        disk->hdr_bak = gpt_get_header(dev, PRIMARY_GPT);
        if (!disk->hdr_bak) {
                ALOGE("%s: Failed to get backup header", __func__);
                goto error;
        }
        disk->hdr_bak_crc = crc32(0, disk->hdr_bak, gpt_header_size);
        //Descriptor for the block device. We will use this for further
        //modifications to the partition table
        if (get_dev_path_from_partition_name(dev,
                                disk->devpath,
                                sizeof(disk->devpath)) != 0) {
                ALOGE("%s: Failed to resolve path for %s",
                                __func__,
                                dev);
                goto error;
        }
        fd = open(disk->devpath, O_RDWR);
        if (fd < 0) {
                ALOGE("%s: Failed to open %s: %s",
                                __func__,
                                disk->devpath,
                                strerror(errno));
                goto error;
        }
        disk->pentry_arr = gpt_get_pentry_arr(disk->hdr, fd);
        if (!disk->pentry_arr) {
                ALOGE("%s: Failed to obtain partition entry array",
                                __func__);
                goto error;
        }
        disk->pentry_arr_bak = gpt_get_pentry_arr(disk->hdr_bak, fd);
        if (!disk->pentry_arr_bak) {
                ALOGE("%s: Failed to obtain backup partition entry array",
                                __func__);
                goto error;
        }
        disk->pentry_size = GET_4_BYTES(disk->hdr + PENTRY_SIZE_OFFSET);
        disk->pentry_arr_size =
                GET_4_BYTES(disk->hdr + PARTITION_COUNT_OFFSET) *
                disk->pentry_size;
        disk->pentry_arr_crc = GET_4_BYTES(disk->hdr + PARTITION_CRC_OFFSET);
        disk->pentry_arr_bak_crc = GET_4_BYTES(disk->hdr_bak +
                        PARTITION_CRC_OFFSET);
        disk->block_size = gpt_get_block_size(fd);
        close(fd);
        disk->is_initialized = GPT_DISK_INIT_MAGIC;
        return 0;
error:
        if (fd >= 0)
                close(fd);
        return -1;
}
//Get pointer to partition entry from a allocated gpt_disk structure
uint8_t* gpt_disk_get_pentry(struct gpt_disk *disk,
                const char *partname,
                enum gpt_instance instance)
{
        uint8_t *ptn_arr = NULL;
        if (!disk || !partname || disk->is_initialized != GPT_DISK_INIT_MAGIC) {
                ALOGE("%s: Invalid argument",__func__);
                goto error;
        }
        ptn_arr = (instance == PRIMARY_GPT) ?
                disk->pentry_arr : disk->pentry_arr_bak;
        return (gpt_pentry_seek(partname, ptn_arr,
                        ptn_arr + disk->pentry_arr_size ,
                        disk->pentry_size));
error:
        return NULL;
}
//Update CRC values for the various components of the gpt_disk
//structure. This function should be called after any of the fields
//have been updated before the structure contents are written back to
//disk.
int gpt_disk_update_crc(struct gpt_disk *disk)
{
        uint32_t gpt_header_size = 0;
        if (!disk || (disk->is_initialized != GPT_DISK_INIT_MAGIC)) {
                ALOGE("%s: invalid argument", __func__);
                goto error;
        }
        //Recalculate the CRC of the primary partiton array
        disk->pentry_arr_crc = crc32(0,
                        disk->pentry_arr,
                        disk->pentry_arr_size);
        //Recalculate the CRC of the backup partition array
        disk->pentry_arr_bak_crc = crc32(0,
                        disk->pentry_arr_bak,
                        disk->pentry_arr_size);
        //Update the partition CRC value in the primary GPT header
        PUT_4_BYTES(disk->hdr + PARTITION_CRC_OFFSET, disk->pentry_arr_crc);
        //Update the partition CRC value in the backup GPT header
        PUT_4_BYTES(disk->hdr_bak + PARTITION_CRC_OFFSET,
                        disk->pentry_arr_bak_crc);
        //Update the CRC value of the primary header
        gpt_header_size = GET_4_BYTES(disk->hdr + HEADER_SIZE_OFFSET);
        //Header CRC is calculated with its own CRC field set to 0
        PUT_4_BYTES(disk->hdr + HEADER_CRC_OFFSET, 0);
        PUT_4_BYTES(disk->hdr_bak + HEADER_CRC_OFFSET, 0);
        disk->hdr_crc = crc32(0, disk->hdr, gpt_header_size);
        disk->hdr_bak_crc = crc32(0, disk->hdr_bak, gpt_header_size);
        PUT_4_BYTES(disk->hdr + HEADER_CRC_OFFSET, disk->hdr_crc);
        PUT_4_BYTES(disk->hdr_bak + HEADER_CRC_OFFSET, disk->hdr_bak_crc);
        return 0;
error:
        return -1;
}
//Write the contents of struct gpt_disk back to the actual disk
int gpt_disk_commit(struct gpt_disk *disk)
{
        int fd = -1;
        if (!disk || (disk->is_initialized != GPT_DISK_INIT_MAGIC)){
                ALOGE("%s: Invalid args", __func__);
                goto error;
        }
        fd = open(disk->devpath, O_RDWR);
        if (fd < 0) {
                ALOGE("%s: Failed to open %s: %s",
                                __func__,
                                disk->devpath,
                                strerror(errno));
                goto error;
        }
        ALOGI("%s: Writing back primary GPT header", __func__);
        //Write the primary header
        if(gpt_set_header(disk->hdr, fd, PRIMARY_GPT) != 0) {
                ALOGE("%s: Failed to update primary GPT header",
                                __func__);
                goto error;
        }
        ALOGI("%s: Writing back primary partition array", __func__);
        //Write back the primary partition array
        if (gpt_set_pentry_arr(disk->hdr, fd, disk->pentry_arr)) {
                ALOGE("%s: Failed to write primary GPT partition arr",
                                __func__);
                goto error;
        }
        close(fd);
        return 0;
error:
        if (fd >= 0)
                close(fd);
        return -1;
}
