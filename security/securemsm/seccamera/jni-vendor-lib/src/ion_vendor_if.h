/**
 * Copyright (c) 2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 *
=============================================================

                          EDIT HISTORY FOR FILE

when       who     what, where, why
--------   ---     ------------------------------------------
06/14/17   gs      Initial version
=============================================================*/
#ifndef ION_IF_H
#define ION_IF_H

#include "common.h"
#include <linux/msm_ion.h>

#define ION_IF_SIZE_4K (0x1000)
#define ION_IF_SIZE_1M (0x100000)
#define ION_IF_SIZE_2M (0x200000)

typedef struct ion_if_info_t {
    int32_t ion_fd_;
    int32_t ifd_data_fd_;
    struct ion_handle_data ion_alloc_handle_;
    unsigned char* ion_sbuffer_;
    uint32_t sbuf_len_;
    uint32_t flags_;
} ion_if_info_t;

int32_t ion_if_memalloc(uint32_t heap_id, ion_if_info_t* handle, uint32_t size, uint32_t flags, uint32_t align);
int32_t ion_if_dealloc(ion_if_info_t* handle);

#endif //ION_IF_H
