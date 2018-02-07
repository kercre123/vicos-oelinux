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

#include "camera_format.h"
#include "camera_params.h"

// Camera callback: called with captured `image` of `width` by `height`, with specified bits per pixel
// (called from separate thread)
//typedef int(*camera_cb)(uint8_t* image, int width, int height, int bpp);
typedef int(*camera_cb)(const uint8_t* image,
                        uint64_t timestamp,
                        uint32_t frame_id,
                        int width,
                        int height,
                        void* cb_ctx);

#define ANKI_CAMERA_FPS_MAX 30

struct anki_camera_params {
  camera_cb frame_callback;
  struct anki_camera_capture_params capture_params;
};

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

#ifdef __cplusplus
}
#endif


#endif // __mm_camera_anki_h__

