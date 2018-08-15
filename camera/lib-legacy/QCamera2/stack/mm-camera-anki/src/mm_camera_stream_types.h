/**
 * File: mm_camera_stream_types.h 
 *
 * Author: chapados
 * Created: 1/29/2018
 *
 * Description: Various things used by the mm_camera_stream_*s            
 *
 * Copyright: Anki, Inc. 2018
 *
 **/

#ifndef __mm_camera_stream_types_h__
#define __mm_camera_stream_types_h__

#include <stdint.h>

#include "camera_format.h"
#include "camera_params.h"

#include <pthread.h>

// Camera callback: called with captured `image` of `width` by `height`, with specified bits per pixel
// (called from separate thread)
//typedef int(*camera_cb)(uint8_t* image, int width, int height, int bpp);
typedef int(*camera_cb)(const uint8_t* image,
                        uint64_t timestamp,
                        uint32_t frame_id,
                        int width,
                        int height,
                        anki_camera_pixel_format_t format,
                        void* cb_ctx);

struct anki_camera_params {
  camera_cb frame_callback_raw;
  camera_cb frame_callback_preview;
  camera_cb frame_callback_snapshot;
  struct anki_camera_capture_params capture_params;
};


#endif // __mm_camera_stream_types_h__
