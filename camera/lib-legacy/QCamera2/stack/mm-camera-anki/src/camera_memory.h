/**
 * File: camera_memory.h
 *
 * Author: chapados
 * Created: 1/29/2018
 *
 * Description: camera helper functions for managing ION memory                                     
 *
 * Copyright: Anki, Inc. 2018
 *
 **/

#ifndef __anki_camera_memory_h__
#define __anki_camera_memory_h__

#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <linux/msm_ion.h>

typedef struct {
  int                     fd;
  int                     main_ion_fd;
  ion_user_handle_t       handle;
  size_t                  size;
  void *                  data;
} anki_camera_buf_t;

int anki_camera_allocate_ion_memory(size_t len, anki_camera_buf_t* out_buf);
int anki_camera_deallocate_ion_memory(anki_camera_buf_t *buf);

#endif // __anki_camera_memory_h__
