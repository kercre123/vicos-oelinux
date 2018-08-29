/**
 * File: mm_camera_obj.h
 *
 * Author: Al Chaussee
 * Created: 7/14/2018
 *
 * Description: Definition of a CameraObj
 *
 * Copyright: Anki, Inc. 2018
 *
 **/

#ifndef __mm_camera_obj_h__
#define __mm_camera_obj_h__

#include "mm_camera_stream_types.h"
#include "mm_qcamera_app.h"

typedef struct cameraobj_t {
  mm_camera_lib_handle lib_handle;
  void* callback_ctx;
  pthread_mutex_t callback_lock;
  struct anki_camera_params params;
  int is_running;
  anki_camera_pixel_format_t pixel_format;

  int resetable_fields; 
    
  pthread_mutex_t shutdown_lock;
  int shutdown_lock_inited;

} CameraObj;

#endif
