/**
 * File: mm_camera_stream_preview.c
 *
 * Author: Al Chaussee
 * Created: 7/14/2018
 *
 * Description: 
 *
 * Copyright: Anki, Inc. 2018
 *
 **/

#ifndef __mm_camera_stream_preview_h__
#define __mm_camera_stream_preview_h__

#include "mm_camera_obj.h"
#include "camera_process.h"

void camera_install_callback_preview(camera_cb cb, CameraObj* camera);

mm_camera_channel_t * anki_mm_app_add_preview_channel(mm_camera_test_obj_t *test_obj);

mm_camera_stream_t * anki_mm_app_add_preview_stream(mm_camera_test_obj_t *test_obj,
                mm_camera_channel_t *channel,
                mm_camera_buf_notify_t stream_cb,
                void *userdata,
                uint8_t num_bufs);

int victor_start_preview(mm_camera_lib_handle *handle);


int victor_stop_preview(mm_camera_test_obj_t* test_obj);

#endif // __mm_camera_stream_rdi_h__