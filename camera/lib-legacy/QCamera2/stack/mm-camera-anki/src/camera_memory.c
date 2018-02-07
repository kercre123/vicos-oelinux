/**
 * File: camera_memory.c
 *
 * Author: chapados
 * Created: 1/29/2018
 *
 * Description: camera helper functions for managing ION memory
 *
 * Copyright: Anki, Inc. 2018
 *
 **/

#include <errno.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <linux/msm_ion.h>

#include "camera_memory.h"
#include "log.h"

int anki_camera_allocate_ion_memory(size_t len, anki_camera_buf_t* out_buf)
{
  int rc = 0;
  struct ion_handle_data handle_data;
  struct ion_allocation_data alloc;
  struct ion_fd_data ion_info_fd;
  int main_ion_fd = 0;
  void *data = NULL;

  main_ion_fd = open("/dev/ion", O_RDONLY);
  if (main_ion_fd <= 0) {
    loge("%s: Ion dev open failed %s\n", __FUNCTION__, strerror(errno));
    goto ION_OPEN_FAILED;
  }

  memset(&alloc, 0, sizeof(alloc));
  alloc.len = len;
  /* to make it page size aligned */
  alloc.len = (alloc.len + 4095U) & (~4095U);
  alloc.align = 4096;
  alloc.flags = ION_FLAG_CACHED;
  alloc.heap_id_mask = 0x1 << ION_IOMMU_HEAP_ID;
  rc = ioctl(main_ion_fd, ION_IOC_ALLOC, &alloc);
  if (rc < 0) {
    loge("%s: ION allocation failed\n", __FUNCTION__);
    goto ION_ALLOC_FAILED;
  }

  memset(&ion_info_fd, 0, sizeof(ion_info_fd));
  ion_info_fd.handle = alloc.handle;
  rc = ioctl(main_ion_fd, ION_IOC_SHARE, &ion_info_fd);
  if (rc < 0) {
    loge("%s: ION map failed %s\n", __FUNCTION__, strerror(errno));
    goto ION_MAP_FAILED;
  }

  data = mmap(NULL,
              alloc.len,
              PROT_READ  | PROT_WRITE,
              MAP_SHARED,
              ion_info_fd.fd,
              0);

  if (data == MAP_FAILED) {
    loge("%s: ION_MMAP_FAILED: %s (%d)\n", __FUNCTION__, strerror(errno), errno);
    goto ION_MAP_FAILED;
  }
  out_buf->main_ion_fd = main_ion_fd;
  out_buf->fd = ion_info_fd.fd;
  out_buf->handle = ion_info_fd.handle;
  out_buf->size = alloc.len;
  out_buf->data = data;
  return 0;

ION_MAP_FAILED:
  memset(&handle_data, 0, sizeof(handle_data));
  handle_data.handle = ion_info_fd.handle;
  ioctl(main_ion_fd, ION_IOC_FREE, &handle_data);
ION_ALLOC_FAILED:
  close(main_ion_fd);
ION_OPEN_FAILED:
  return -1;
}

int anki_camera_deallocate_ion_memory(anki_camera_buf_t *buf)
{
  struct ion_handle_data handle_data;
  int rc = 0;

  if ((buf->data != NULL) && (buf->fd > 0) && (buf->handle > 0)) {
    rc = munmap(buf->data, buf->size);
  }

  if (rc == -1) {
    loge("%s: failed to unmap ION mem: %s", __FUNCTION__, strerror(errno));
  }

  if (buf->fd > 0) {
    close(buf->fd);
    buf->fd = 0;
  }

  if (buf->main_ion_fd > 0) {
    memset(&handle_data, 0, sizeof(handle_data));
    handle_data.handle = buf->handle;
    rc = ioctl(buf->main_ion_fd, ION_IOC_FREE, &handle_data);
    if (rc != 0) {
      loge("%s: failed to free ION mem: %s", __FUNCTION__, strerror(errno));
    }
    close(buf->main_ion_fd);
    buf->main_ion_fd = 0;
  }
  return rc;
}
