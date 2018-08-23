/**
 * File: mm_camera_anki.h 
 *
 * Author: chapados
 * Created: 1/29/2018
 *
 * Description: mm-camera-interface adapter for camera capture from internal qcom stack             
 *
 * Copyright: Anki, Inc. 2018
 *
 **/

#ifndef __mm_camera_anki_h__
#define __mm_camera_anki_h__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "camera_process.h"
#include "mm_camera_stream_types.h"
  
#define ANKI_CAMERA_FPS_MAX 30

// Initializes the camera
int camera_init();

// Starts capturing frames in new thread, sends them to callback `cb`.
int camera_start(struct anki_camera_params* params, void* cb_ctx);

// Set capture parameters
int camera_set_params(struct anki_camera_params* params);

// Stops capturing frames
int camera_stop();

// De-initializes camera, makes it available to rest of system
int camera_cleanup();

int camera_set_exposure(uint16_t exposure_ms, float gain);

int camera_set_awb(float r_gain, float g_gain, float b_gain);

int camera_set_capture_format(struct anki_camera_capture* capture,
                              anki_camera_pixel_format_t format,
                              int(*realloc_with_format)
                                (struct anki_camera_capture* capture,
                                 anki_camera_pixel_format_t format));

int camera_start_snapshot();

int camera_stop_snapshot();

#ifdef __cplusplus
}
#endif


#endif // __mm_camera_anki_h__

