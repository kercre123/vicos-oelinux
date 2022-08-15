/**
 * File: camera_process.c
 *
 * Author: chapados
 * Created: 1/29/2018
 *
 * Description: Anki-specific camera capture + downsampling processor                               
 *
 * Copyright: Anki, Inc. 2018
 *
 **/

#include <assert.h>
#include <errno.h>
#include <stdint.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <getopt.h>

#include "mm_camera_anki.h"
#include "debayer.h"
#include "camera_process.h"
#include "camera_memory.h"
#include "log.h"

static int  s_dump_images = 0;
static char s_output_path[256] = "/data/misc/camera/test";

void debug_dump_frame(uint8_t *frame, int num_bytes, char* prefix)
{
  char file_name[512];
  static int frame_idx = 0;
  const char* name = "anki_camera";
  const char* ext = "rgb";
  int file_fd;

  snprintf(file_name, sizeof(file_name), "%s/%s_%s_%04d.%s",
           s_output_path, prefix, name, frame_idx++, ext);
  file_fd = open(file_name, O_RDWR | O_CREAT, 0777);
  if (file_fd < 0) {
    loge("%s: cannot open file %s \n", __func__, file_name);
  }
  else {
    write(file_fd, frame, num_bytes);
  }

  close(file_fd);
  logd("dump %s", file_name);
}

int get_next_buffer_slot(anki_camera_buf_header_t* header, 
                         anki_camera_pixel_format_t format,
                         uint32_t* out_slot)
{
  int slot_found = -1;
    
  if(header == NULL)
  {
    return slot_found;
  }

  uint32_t slot = atomic_load(&header->locks.write_idx);

  for (uint32_t i = 0; i < header->frame_count; ++i) {
    slot = (slot + 1) % header->frame_count;
    uint32_t v = atomic_load(&(header->locks.frame_locks[slot]));
    if (v == 0) {
      slot_found = 1;
      break;
    }
  }

  if (slot_found) {
    *out_slot = slot;
  }

  return (slot_found) ? 0 : -1;
}

// Callback is received on a separate thread.
// Lock shared buffer data before modifying data
int anki_camera_frame_callback(const uint8_t* image,
                               uint64_t timestamp,
                               uint32_t frame_id,
                               int width,
                               int height,
                               anki_camera_pixel_format_t format,
                               void* cb_ctx)
{
  struct anki_camera_capture* capture = (struct anki_camera_capture*)cb_ctx;
  uint8_t* data = (uint8_t*)capture->buffer.data;
  anki_camera_buf_header_t* header = (anki_camera_buf_header_t*)data;

  // get the next slot for writing
  uint32_t slot;
  int rc = get_next_buffer_slot(header, format, &slot);
  if (rc == -1) {
    loge("%s: No buffer space available", __FUNCTION__);
    return -1;
  }

  // lock slot for writing
  uint32_t lock_status = 0;
  _Atomic uint32_t *slot_lock = &(header->locks.frame_locks[slot]);
  if (!atomic_compare_exchange_strong(slot_lock, &lock_status, 1)) {
    loge("%s: could not lock frame for writing (slot %u)", __FUNCTION__, slot);
    return -1;
  }

  // Process frame
  const uint32_t frame_offset = header->frame_offsets[slot];
  anki_camera_frame_t* frame = (anki_camera_frame_t*)&data[frame_offset];
  const size_t frame_data_len = frame->bytes_per_row * frame->height;

  frame->frame_id = frame_id;
  frame->timestamp = timestamp;
  
  int downsampled_width = width / 10 * 8; // First account for 10 bits to 8 bits.
  downsampled_width = downsampled_width / 2; // Then account for downsampling

  int downsampled_height = height / 2;
  
  if (frame->format == ANKI_CAM_FORMAT_BAYER_MIPI_BGGR10 ||
      frame->format == ANKI_CAM_FORMAT_BAYER_MIPI_BGGR10_2MP ||
      frame->format == ANKI_CAM_FORMAT_YUV ||
      frame->format == ANKI_CAM_FORMAT_YUV_2MP ) {
    // no downsample, just copy
    memcpy(frame->data, image, frame_data_len);
  }
  else if ( ((downsampled_width * 3) != frame->bytes_per_row) || // RGB is 3 bytes per width
       (downsampled_height != frame->height) ) {
    loge("%s: want a downsampled image of %dx%d, but got a downsampled image of %dx%d",
        __FUNCTION__, (frame->bytes_per_row / 3), frame->height,
        downsampled_width, downsampled_height);
    return -1;
  }
  else {
    // downsample
    bayer_mipi_bggr10_downsample(image, frame->data, width, height, 10);
  }

  if(s_dump_images)
  {
    char* s;
    if(frame->format == ANKI_CAM_FORMAT_RGB888 ||
       frame->format == ANKI_CAM_FORMAT_RGB888_2MP)
    {
      s = "rgb";
    }
    else if(frame->format == ANKI_CAM_FORMAT_YUV ||
	    frame->format == ANKI_CAM_FORMAT_YUV_2MP )
    {
      s = "yuv";
    }
    debug_dump_frame(frame->data, frame_data_len, s);
  }

  // unlock frame
  // Do this in a loop?  It must succeed...
  lock_status = 1;
  if (!atomic_compare_exchange_strong(slot_lock, &lock_status, 0)) {
    loge("%s: could not unlock frame: %u", __FUNCTION__, slot);
    return -1;
  }

  // update write_idx
  uint32_t current_write_idx = atomic_load(&(header->locks.write_idx));
  if (!atomic_compare_exchange_strong(&(header->locks.write_idx), &current_write_idx, slot)) {
    loge("%s: could not update write_idx (%u -> %u)", __FUNCTION__, current_write_idx, slot);
    return -1;
  }

  logd("%s: FRAME frame_id=%u slot=%u ts=%lld",
       __func__, frame_id, slot, timestamp);

  return 0;
}

static anki_camera_frame_t s_frame_info_bayer_mipi_bggr10 = {
  .width = 1280,
  .height = 720,
  .bytes_per_row = (10 * 1280) / 8,
  .bits_per_pixel = 10,
  .format = ANKI_CAM_FORMAT_BAYER_MIPI_BGGR10,
};

static anki_camera_frame_t s_frame_info_bayer_mipi_bggr10_2mp = {
  .width = 1600,
  .height = 1200,
  .bytes_per_row = (10 * 1600) / 8,
  .bits_per_pixel = 10,
  .format = ANKI_CAM_FORMAT_BAYER_MIPI_BGGR10_2MP,
};

static anki_camera_frame_t s_frame_info_rgb888 = {
  .width = 640,
  .height = 360,
  .bytes_per_row = 640 * 3,
  .bits_per_pixel = 8,
  .format = ANKI_CAM_FORMAT_RGB888,
};

static anki_camera_frame_t s_frame_info_rgb888_2mp = {
  .width = 800,
  .height = 600,
  .bytes_per_row = 800 * 3,
  .bits_per_pixel = 8,
  .format = ANKI_CAM_FORMAT_RGB888_2MP,
};

static anki_camera_frame_t s_frame_info_yuv420sp = {
  .width = 1280,
  .height = 1080, // YUV data has 1080 rows which convert to 720 pixels
  .bytes_per_row = 1280,
  .bits_per_pixel = 12, // A block of 2x2 pixels is 4 Y + 1 U + 1 V = 6 bytes / 4 pixels = 12 bits/pixel
  .format = ANKI_CAM_FORMAT_YUV,
};

static anki_camera_frame_t s_frame_info_yuv420sp_2mp = {
  .width = 1600,
  .height = 1200, // YUV data has 1080 rows which convert to 720 pixels
  .bytes_per_row = 1600,
  .bits_per_pixel = 12, // A block of 2x2 pixels is 4 Y + 1 U + 1 V = 6 bytes / 4 pixels = 12 bits/pixel
  .format = ANKI_CAM_FORMAT_YUV,
};

int get_camera_frame_info(const anki_camera_pixel_format_t format, anki_camera_frame_t* out_frame)
{
  if (out_frame == NULL) {
    return -1;
  }

  int rc = 0;
  switch (format) {
  case ANKI_CAM_FORMAT_BAYER_MIPI_BGGR10: {
    *out_frame = s_frame_info_bayer_mipi_bggr10;
    break;
  }
  case ANKI_CAM_FORMAT_RGB888: {
    *out_frame = s_frame_info_rgb888;
    break;
  }
  case ANKI_CAM_FORMAT_YUV: {
    *out_frame = s_frame_info_yuv420sp;
    break;
  }
  case ANKI_CAM_FORMAT_BAYER_MIPI_BGGR10_2MP: {
    *out_frame = s_frame_info_bayer_mipi_bggr10_2mp;
    break;
  }
  case ANKI_CAM_FORMAT_RGB888_2MP: {
    *out_frame = s_frame_info_rgb888_2mp;
    break;
  }
  case ANKI_CAM_FORMAT_YUV_2MP: {
    *out_frame = s_frame_info_yuv420sp_2mp;
    break;
  }
  default: {
    loge("%s: unknown format: %u", __FUNCTION__, format);
    rc = -1;
    break;
  }
  }

  return rc;
}

int camera_capture_alloc(struct anki_camera_capture* capture,
                         anki_camera_pixel_format_t format)
{
  assert(capture->buffer.data == NULL);
  memset(capture, 0, sizeof(struct anki_camera_capture));
  capture->buffer.fd = -1;
  capture->buffer.main_ion_fd = -1;
  capture->buffer.handle = -1;

  // calculate frame payload size based on format
  anki_camera_frame_t frame_info;
  int rc = get_camera_frame_info(format, &frame_info);
  if (rc == -1) {
    loge("%s: requesting storage for invalid format: %d", __FUNCTION__, format);
    return rc;
  }

  // size of pixel payload for 1 frame
  size_t frame_data_len = frame_info.bytes_per_row * frame_info.height;

  // align frame data len to 64 byte boundary
  frame_data_len = (frame_data_len + 63U) & (~63U);

  // calc size of metadata
  size_t header_len = sizeof(anki_camera_buf_header_t);
  size_t frame_header_len = sizeof(anki_camera_frame_t);
  size_t frame_len = frame_header_len + frame_data_len;

  size_t buf_len = header_len + (frame_len * ANKI_CAMERA_MAX_FRAME_COUNT);

  anki_camera_buf_t* camera_buf = &(capture->buffer);
  rc = anki_camera_allocate_ion_memory(buf_len, camera_buf);
  if (rc < 0) {
    loge("%s: error allocating capture buffer", __FUNCTION__);
    return rc;
  }

  // initialize metadata
  uint8_t* buf = (uint8_t*)camera_buf->data;
  anki_camera_buf_header_t* header = (anki_camera_buf_header_t*)buf;

  header->magic[0] = 'C';
  header->magic[1] = 'A';
  header->magic[2] = 'M';
  header->magic[3] = '0';

  header->frame_count = ANKI_CAMERA_MAX_FRAME_COUNT;
  header->frame_size = frame_len;

  header->locks.write_idx = ATOMIC_VAR_INIT(0);

  // initialize per-frame data
  for (uint32_t i = 0; i < ANKI_CAMERA_MAX_FRAME_COUNT; ++i) {
    header->locks.frame_locks[i] = ATOMIC_VAR_INIT(0);
    const uint32_t frame_offset = header_len + (frame_len * i);
    header->frame_offsets[i] = frame_offset;

    // fill out frame metadata
    anki_camera_frame_t* frame = (anki_camera_frame_t*)&buf[frame_offset];
    switch (format) {
    case ANKI_CAM_FORMAT_BAYER_MIPI_BGGR10: {
      memcpy(frame, &s_frame_info_bayer_mipi_bggr10, sizeof(s_frame_info_bayer_mipi_bggr10));
    }
    break;
    case ANKI_CAM_FORMAT_RGB888: {
      memcpy(frame, &s_frame_info_rgb888, sizeof(s_frame_info_rgb888));
    }
    break;
    case ANKI_CAM_FORMAT_YUV: {
      memcpy(frame, &s_frame_info_yuv420sp, sizeof(s_frame_info_yuv420sp));
    }
    break;
    case ANKI_CAM_FORMAT_BAYER_MIPI_BGGR10_2MP: {
      memcpy(frame, &s_frame_info_bayer_mipi_bggr10_2mp, sizeof(s_frame_info_bayer_mipi_bggr10_2mp));
    }
    break;
    case ANKI_CAM_FORMAT_RGB888_2MP: {
      memcpy(frame, &s_frame_info_rgb888_2mp, sizeof(s_frame_info_rgb888_2mp));
    }
    break;
    }
  }

  return 0;
}

int is_camera_capture_running(struct anki_camera_capture* capture)
{
  return (capture->state == ANKI_CAMERA_RUNNING);
}

int is_camera_capture_ready(struct anki_camera_capture* capture)
{
  return (capture->state == ANKI_CAMERA_IDLE);
}

int is_camera_capture_initialized(struct anki_camera_capture* capture)
{
  return (capture->state != ANKI_CAMERA_NOT_READY);
}

int camera_capture_init(struct anki_camera_capture* capture,
                        anki_camera_pixel_format_t format)
{

  int rc = camera_capture_alloc(capture, format);
  if (rc != 0) {
    loge("%s: error allocating capture buffer", __FUNCTION__);
  }

  capture->state = ANKI_CAMERA_IDLE;

  return rc;
}

int camera_capture_release(struct anki_camera_capture* capture)
{
  assert(capture->buffer.data != NULL);

  int rc = anki_camera_deallocate_ion_memory(&capture->buffer);
  if (rc < 0) {
    loge("%s: error deallocating capture buffer", __FUNCTION__);
  }

  memset(capture, 0, sizeof(struct anki_camera_capture));

  return 0;
}

int camera_capture_start(struct anki_camera_capture* capture,
                         struct anki_camera_capture_params* capture_params)
{
  assert(capture->buffer.data != NULL);

  int rc = camera_init();
  if (rc != 0) {
    loge("%s: error initializing camera capture", __FUNCTION__);
    return rc;
  }

  struct anki_camera_params params = {
    .capture_params = *capture_params,
    .frame_callback_raw = anki_camera_frame_callback,
    .frame_callback_preview = anki_camera_frame_callback,
    .frame_callback_snapshot = anki_camera_frame_callback
  };

  rc = camera_start(&params, capture);
  if (rc != 0) {
    loge("%s: error starting camera capture", __FUNCTION__);
    return rc;
  }

  capture->state = ANKI_CAMERA_RUNNING;

  return 0;
}

int camera_capture_stop(struct anki_camera_capture* capture)
{
  int rc;
  assert(capture->buffer.data != NULL);

  rc = camera_stop();
  if(rc != 0)
  {
    loge("%s: failed to stop camera", __func__);
    return rc;
  }

  rc = camera_cleanup();
  if(rc != 0)
  {
    loge("%s: failed to cleanup camera");
    return rc;
  }

  capture->state = ANKI_CAMERA_IDLE;
  return 0;
}

int camera_capture_set_exposure(anki_camera_exposure_t exposure)
{
  return camera_set_exposure(exposure.exposure_ms, exposure.gain);
}

int camera_capture_set_awb(anki_camera_awb_t awb)
{
  return camera_set_awb(awb.r_gain, awb.g_gain, awb.b_gain);
}

int anki_camera_reallocate_ion_memory(struct anki_camera_capture* capture,
                                      anki_camera_pixel_format_t format)
{
  assert(capture->buffer.data != NULL);

  int rc = anki_camera_deallocate_ion_memory(&capture->buffer);
  if (rc < 0) {
    loge("%s: error deallocating capture buffer", __FUNCTION__);
  }

  rc = camera_capture_init(capture, format);
  if (rc < 0) {
    loge("%s: error allocating capture buffer", __FUNCTION__);
  }

  return rc;
}

int camera_capture_set_format(struct anki_camera_capture* capture,
                              anki_camera_pixel_format_t format)
{
  return camera_set_capture_format(capture, format, anki_camera_reallocate_ion_memory);
}

int camera_capture_start_snapshot()
{
  return camera_start_snapshot();
}

int camera_capture_stop_snapshot()
{
  return camera_stop_snapshot();
}
