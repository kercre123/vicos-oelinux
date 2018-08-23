/**
 * File: mm_camera_stream_preview.c
 *
 * Author: Al Chaussee
 * Created: 7/14/2018
 *
 * Description: Everything related to managing preview camera streams
 *
 * Copyright: Anki, Inc. 2018
 *
 **/

#ifndef __mm_camera_stream_preview_h__
#define __mm_camera_stream_preview_h__

#include "mm_camera_obj.h"
#include "camera_process.h"

void camera_install_callback_preview(camera_cb cb, CameraObj* camera);

int victor_start_preview(mm_camera_lib_handle *handle);

int victor_stop_preview(mm_camera_test_obj_t* test_obj);

#endif // __mm_camera_stream_rdi_h__