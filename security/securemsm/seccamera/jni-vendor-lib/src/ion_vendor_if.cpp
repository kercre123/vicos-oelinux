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
#include <unistd.h>
#include <dirent.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/eventfd.h>
#include <errno.h>
#include <linux/msm_ion.h>
#include <sys/mman.h>

#include "ion_vendor_if.h"

#ifdef LOG_TAG
    #undef LOG_TAG
#endif
#define LOG_TAG "SECCAM-LIB-VENDOR-ION-IF"


//=========================================================================
// @brief: Allocate ION memory
//=========================================================================
int32_t ion_if_memalloc(uint32_t heap_id, ion_if_info_t* handle, uint32_t size, uint32_t flags, uint32_t align) {
    int32_t ret = 0;
    int32_t iret = 0;
    int32_t fd = 0;
    unsigned char* v_addr;
    struct ion_allocation_data ion_alloc_data;
    int32_t ion_fd;
    int32_t rc;
    struct ion_fd_data ifd_data;
    struct ion_handle_data handle_data;

    if(handle == NULL){
        LOG_ERR(LOG_TAG, "ion_if_memalloc - null handle received");
        return -1;
    }
    ion_fd  = open("/dev/ion", O_RDONLY);
    if (ion_fd < 0) {
        LOG_ERR(LOG_TAG, "ion_if_memalloc - cannot open ION device");
        return -1;
    }
    handle->ion_sbuffer_ = NULL;
    handle->ifd_data_fd_ = 0;
    handle->flags_ = flags;

    // Size of allocation
    ion_alloc_data.len = (size + (align-1)) & (~(align-1));
    // 4K aligned
    ion_alloc_data.align = align;

    // memory is allocated from EBI heap
    ion_alloc_data.heap_id_mask= ION_HEAP(heap_id);

    // Force the memory to be contiguous
    ion_alloc_data.flags = handle->flags_ | ION_FLAG_FORCE_CONTIGUOUS;

    // IOCTL call to ION for memory request
    rc = ioctl(ion_fd, ION_IOC_ALLOC, &ion_alloc_data);
    if (rc) {
        LOG_ERR(LOG_TAG, "ion_if_memalloc - error while trying to allocate data");
        goto alloc_fail;
    }

    if (ion_alloc_data.handle) {
        ifd_data.handle = ion_alloc_data.handle;
    } else {
        LOG_ERR(LOG_TAG, "ion_if_memalloc - ION alloc data returned a NULL");
        goto alloc_fail;
    }
    // Call MAP ioctl to retrieve the ifd_data.fd file descriptor
    rc = ioctl(ion_fd, ION_IOC_MAP, &ifd_data);
    if (rc) {
        LOG_ERR(LOG_TAG, "ion_if_memalloc - failed doing ION_IOC_MAP call");
        goto ioctl_fail;
    }

    // Make the ion mmap call
    v_addr = (unsigned char *)mmap(NULL, ion_alloc_data.len,
                                PROT_READ | PROT_WRITE,
                                MAP_SHARED, ifd_data.fd, 0);
    if (v_addr == MAP_FAILED) {
        LOG_ERR(LOG_TAG, "ion_if_memalloc - ION MMAP failed");
        ret = -1;
        goto map_fail;
    }
    handle->ion_fd_ = ion_fd;
    handle->ifd_data_fd_ = ifd_data.fd;
    handle->ion_sbuffer_ = v_addr;
    handle->ion_alloc_handle_.handle = ion_alloc_data.handle;
    handle->sbuf_len_ = size;
    LOG_DEBUG(LOG_TAG, "ion_if_memalloc - allocated ION %s buffer size = %d",
            (ION_FLAG_SECURE == (flags & ION_FLAG_SECURE))?"(Secure)":"",
            size);
    return ret;

map_fail:
    if(handle->ion_sbuffer_ != NULL)    {
        ret = munmap(handle->ion_sbuffer_, ion_alloc_data.len);
        if (ret) {
            LOG_ERR(LOG_TAG, "ion_if_memalloc - failed to unmap memory for load image. ret = %d", ret);
        }
    }

ioctl_fail:
    handle_data.handle = ion_alloc_data.handle;
    if (handle->ifd_data_fd_) {
        close(handle->ifd_data_fd_);
    }
    iret = ioctl(ion_fd, ION_IOC_FREE, &handle_data);
    if (iret){
        LOG_ERR(LOG_TAG, "ion_if_memalloc - ION free ioctl returned error = %d", iret);
    }

alloc_fail:
    LOG_ERR(LOG_TAG, "ion_if_memalloc - ION allocation %d", ret);
    if (ion_fd) {
        close(ion_fd);
    }
    return ret;
}

//=========================================================================
// @brief: Deallocate ION memory
//=========================================================================
int32_t ion_if_dealloc(ion_if_info_t* handle) {
    struct ion_handle_data handle_data;
    int32_t ret = 0;

    if(handle == NULL){
        LOG_ERR(LOG_TAG, "ion_if_dealloc - null handle received");
        return -1;
    }

    // Deallocate the memory for the listener
    ret = munmap(handle->ion_sbuffer_, (handle->sbuf_len_ + (ION_IF_SIZE_4K - 1)) & (~(ION_IF_SIZE_4K-1)));
    if (ret) {
        LOG_ERR(LOG_TAG, "ion_if_dealloc - unmapping ION Buffer failed with ret = %d", ret);
    }

    handle_data.handle = handle->ion_alloc_handle_.handle;
    close(handle->ifd_data_fd_);
    ret = ioctl(handle->ion_fd_, ION_IOC_FREE, &handle_data);
    if (ret) {
        LOG_ERR(LOG_TAG, "ion_if_dealloc - memory free ioctl failed with ret = %d", ret);
    }
    close(handle->ion_fd_);
    LOG_DEBUG(LOG_TAG, "ion_if_dealloc - buffer deallocated");
    return ret;
}

