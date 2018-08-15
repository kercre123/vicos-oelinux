/**
 * File: camera_process.h
 *
 * Author: chapados
 * Created: 1/29/2018
 *
 * Description: Anki-specific camera capture + downsampling processor
 *
 * Copyright: Anki, Inc. 2018
 *
 **/

#ifndef __mm_anki_camera_process_h__
#define __mm_anki_camera_process_h__

#include <stdatomic.h>
#include "camera_memory.h"
#include "camera_format.h"
#include "camera_params.h"

#define ANKI_CAMERA_MAX_FRAME_COUNT 6

// capture buffer memory
typedef struct {
  _Atomic uint32_t write_idx;
  _Atomic uint32_t frame_locks[ANKI_CAMERA_MAX_FRAME_COUNT];
} anki_camera_buf_lock_t;

typedef struct {
  uint8_t magic[4];
  anki_camera_buf_lock_t locks;
  uint32_t frame_count;
  uint32_t frame_size;
  uint32_t frame_offsets[ANKI_CAMERA_MAX_FRAME_COUNT];
  uint8_t  data[0];
} anki_camera_buf_header_t;

typedef struct {
  uint64_t timestamp;
  uint32_t frame_id;
  uint32_t width;
  uint32_t height;
  uint32_t bytes_per_row;
  uint8_t  bits_per_pixel;
  uint8_t  format;
  uint8_t  _reserved[2];
  uint32_t _pad_to_64[8];
  uint8_t  data[0];
} anki_camera_frame_t;

typedef struct {
  uint16_t exposure_ms;
  float gain;
} anki_camera_exposure_t;

typedef struct {
  float r_gain;
  float g_gain;
  float b_gain;
} anki_camera_awb_t;

typedef enum {
  ANKI_CAMERA_NOT_READY,
  ANKI_CAMERA_IDLE,
  ANKI_CAMERA_RUNNING
} anki_camera_state_t;

struct anki_camera_capture {
  anki_camera_state_t state;
  anki_camera_buf_t buffer;
};


int camera_capture_init(struct anki_camera_capture* capture, anki_camera_pixel_format_t format);
int camera_capture_release(struct anki_camera_capture* capture);
int camera_capture_start(struct anki_camera_capture* capture,
                         struct anki_camera_capture_params* capture_params);
int camera_capture_stop(struct anki_camera_capture* capture);
int is_camera_capture_running(struct anki_camera_capture* capture);
int is_camera_capture_ready(struct anki_camera_capture* capture);
int is_camera_capture_initialized(struct anki_camera_capture* capture);
int camera_capture_set_exposure(anki_camera_exposure_t exposure);
int camera_capture_set_awb(anki_camera_awb_t awb);
int camera_capture_set_format(struct anki_camera_capture* capture,
                              anki_camera_pixel_format_t format);
int camera_capture_start_snapshot();
int camera_capture_stop_snapshot();

#endif // __mm_anki_camera_process_h__
