/**
 * File: mm_camera_stream_rdi.c
 *
 * Author: Al Chaussee
 * Created: 7/14/2018
 *
 * Description: 
 *
 * Copyright: Anki, Inc. 2018
 *
 **/

#ifndef __mm_camera_stream_rdi_h__
#define __mm_camera_stream_rdi_h__

#include "mm_camera_obj.h"
#include "mm_qcamera_app.h"
#include "camera_process.h"

mm_camera_channel_t * anki_mm_app_add_rdi_channel(mm_camera_test_obj_t *test_obj,
    uint8_t num_burst,
    mm_camera_buf_notify_t stream_cb);

mm_camera_stream_t * anki_mm_app_add_rdi_stream(mm_camera_test_obj_t *test_obj,
    mm_camera_channel_t *channel,
    mm_camera_buf_notify_t stream_cb,
    void *userdata,
    uint8_t num_bufs,
    uint8_t num_burst);

int victor_start_rdi(mm_camera_test_obj_t *test_obj,
                     uint8_t num_burst,
                     mm_camera_buf_notify_t stream_cb);

int victor_stop_rdi(mm_camera_test_obj_t *test_obj);

void camera_install_callback(camera_cb cb, CameraObj* camera);

int anki_mm_camera_start_rdi_capture(mm_camera_lib_handle *handle);

#endif // __mm_camera_stream_rdi_h__