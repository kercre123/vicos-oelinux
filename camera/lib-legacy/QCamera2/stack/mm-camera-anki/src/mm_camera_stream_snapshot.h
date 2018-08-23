/**
 * File: mm_camera_stream_snapshot.h
 *
 * Author: Al Chaussee
 * Created: 7/14/2018
 *
 * Description: Everything related to managing snapshot camera streams
 *
 * Copyright: Anki, Inc. 2018
 *
 **/

#ifndef __mm_camera_stream_snapshot_h__
#define __mm_camera_stream_snapshot_h__

#include "mm_camera_obj.h"
#include "camera_process.h"

void camera_install_callback_snapshot(camera_cb cb, CameraObj* camera);

int mm_anki_app_start_snapshot(mm_camera_test_obj_t *test_obj,
                               uint8_t num_snapshots);

int mm_anki_app_stop_snapshot(mm_camera_test_obj_t *test_obj);



#endif // __mm_camera_stream_snapshot_h__