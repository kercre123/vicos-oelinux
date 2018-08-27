/**
 * File: mm_camera_stream_rdi.c
 *
 * Author: Al Chaussee
 * Created: 7/14/2018
 *
 * Description: Everything related to managing rdi/raw camera streams
 *
 * Copyright: Anki, Inc. 2018
 *
 **/

#include <dlfcn.h>
#include <sys/mman.h>

#include "mm_qcamera_dbg.h"
#include "mm_camera_stream_rdi.h"

/*************************************/

#define DEFAULT_RAW_RDI_FORMAT        CAM_FORMAT_BAYER_MIPI_RAW_10BPP_BGGR


mm_camera_stream_t * anki_mm_app_add_rdi_stream(mm_camera_test_obj_t *test_obj,
    mm_camera_channel_t *channel,
    mm_camera_buf_notify_t stream_cb,
    void *userdata,
    uint8_t num_bufs,
    uint8_t num_burst)
{
  CDBG_ERROR("ADDING RAW STREAM\n");
  int rc = MM_CAMERA_OK;
  size_t i;
  mm_camera_stream_t *stream = NULL;
  cam_capability_t *cam_cap = (cam_capability_t *)(test_obj->cap_buf.buf.buffer);
  cam_format_t fmt = CAM_FORMAT_MAX;
  cam_stream_buf_plane_info_t *buf_planes;

  stream = mm_app_add_stream(test_obj, channel);
  if (NULL == stream) {
    CDBG_ERROR("%s: add stream failed\n", __func__);
    return NULL;
  }

  // BRC: Supported raw formats based on capabilities
  // CAM_FORMAT_BAYER_MIPI_RAW_10BPP_BGGR
  // CAM_FORMAT_BAYER_IDEAL_RAW_QCOM_10BPP_BGGR
  // CAM_FORMAT_BAYER_QCOM_RAW_10BPP_BGGR
  // CAM_FORMAT_YUV_422_NV16
  // CAM_FORMAT_YUV_422_NV61
  // Only CAM_FORMAT_BAYER_MIPI_RAW_10BPP_BGGR seems to actually bypass ISP

  CDBG("%s: raw_dim w:%d height:%d\n", __func__,
       cam_cap->raw_dim.width,
       cam_cap->raw_dim.height);
  for (i = 0; i < cam_cap->supported_raw_fmt_cnt; i++) {
    cam_format_t cur_fmt = cam_cap->supported_raw_fmts[i];
    CDBG("%s: supported_raw_fmts[%zu]=%d\n", __func__, i, cur_fmt);
    if (DEFAULT_RAW_RDI_FORMAT == cur_fmt) {
      fmt = cur_fmt;
      break;
    }
  }

  if (CAM_FORMAT_MAX == fmt) {
    CDBG_ERROR("%s: rdi format not supported\n", __func__);
    return NULL;
  }

  // BRC: Leaving this commented out as documentation about what formats
  // actually work as expected with RDI (from mm_qcamera_rdi.c)
  // if (!(CAM_FORMAT_BAYER_MIPI_RAW_8BPP_GBRG <= fmt &&
  //       CAM_FORMAT_BAYER_MIPI_RAW_12BPP_BGGR >= fmt)) {
  //     CDBG_ERROR("%s: rdi does not support DEFAULT_RAW_FORMAT\n", __func__);
  //     return NULL;
  // }

  stream->s_config.mem_vtbl.get_bufs = mm_app_stream_initbuf;
  stream->s_config.mem_vtbl.put_bufs = mm_app_stream_deinitbuf;
  stream->s_config.mem_vtbl.clean_invalidate_buf = mm_app_stream_clean_invalidate_buf;
  stream->s_config.mem_vtbl.invalidate_buf = mm_app_stream_invalidate_buf;
  stream->s_config.mem_vtbl.user_data = (void *)stream;
  stream->s_config.stream_cb = stream_cb;
  stream->s_config.userdata = userdata;
  stream->num_of_bufs = num_bufs;

  stream->s_config.stream_info = (cam_stream_info_t *)stream->s_info_buf.buf.buffer;
  memset(stream->s_config.stream_info, 0, sizeof(cam_stream_info_t));
  stream->s_config.stream_info->stream_type = CAM_STREAM_TYPE_RAW;
  if (num_burst == 0) {
    stream->s_config.stream_info->streaming_mode = CAM_STREAMING_MODE_CONTINUOUS;
  }
  else {
    stream->s_config.stream_info->streaming_mode = CAM_STREAMING_MODE_BURST;
    stream->s_config.stream_info->num_of_burst = num_burst;
  }
  stream->s_config.stream_info->fmt = fmt;
  stream->s_config.stream_info->dim.width = cam_cap->raw_dim.width;
  stream->s_config.stream_info->dim.height = cam_cap->raw_dim.height;
  stream->s_config.padding_info = cam_cap->padding_info;

  rc = mm_app_config_stream(test_obj, channel, stream, &stream->s_config);
  if (MM_CAMERA_OK != rc) {
    CDBG_ERROR("%s:config rdi stream err=%d\n", __func__, rc);
    return NULL;
  }

  buf_planes = &stream->s_config.stream_info->buf_planes;
  CDBG("%s: plane_info %dx%d len:%d frame_len:%d\n", __func__,
       buf_planes->plane_info.mp[0].stride, buf_planes->plane_info.mp[0].scanline,
       buf_planes->plane_info.mp[0].len, buf_planes->plane_info.frame_len);

  return stream;
}

mm_camera_channel_t * anki_mm_app_add_rdi_channel(mm_camera_test_obj_t *test_obj,
    uint8_t num_burst,
    mm_camera_buf_notify_t stream_cb)
{
  mm_camera_channel_t *channel = NULL;
  mm_camera_stream_t *stream = NULL;

  channel = mm_app_add_channel(test_obj,
                               MM_CHANNEL_TYPE_RDI,
                               NULL,
                               NULL,
                               NULL);
  if (NULL == channel) {
    CDBG_ERROR("%s: add channel failed", __func__);
    return NULL;
  }

  stream = anki_mm_app_add_rdi_stream(test_obj,
                                      channel,
                                      stream_cb,
                                      (void *)test_obj,
                                      RDI_BUF_NUM,
                                      num_burst);
  if (NULL == stream) {
    CDBG_ERROR("%s: add stream failed\n", __func__);
    mm_app_del_channel(test_obj, channel);
    return NULL;
  }

  CDBG("%s: channel=%d stream=%d\n", __func__, channel->ch_id, stream->s_id);
  return channel;
}

int mm_app_stop_and_del_rdi_channel(mm_camera_test_obj_t *test_obj,
                                    mm_camera_channel_t *channel)
{
  int rc = MM_CAMERA_OK;
  mm_camera_stream_t *stream = NULL;
  uint8_t i;

  rc = mm_app_stop_channel(test_obj, channel);
  if (MM_CAMERA_OK != rc) {
    CDBG_ERROR("%s:Stop RDI failed rc=%d\n", __func__, rc);
  }

  for (i = 0; i < channel->num_streams; i++) {
    stream = &channel->streams[i];
    rc = mm_app_del_stream(test_obj, channel, stream);
    if (MM_CAMERA_OK != rc) {
      CDBG_ERROR("%s:del stream(%d) failed rc=%d\n", __func__, i, rc);
    }
  }

  rc = mm_app_del_channel(test_obj, channel);
  if (MM_CAMERA_OK != rc) {
    CDBG_ERROR("%s:delete channel failed rc=%d\n", __func__, rc);
  }

  return rc;
}

int victor_start_rdi(mm_camera_test_obj_t *test_obj,
                     uint8_t num_burst,
                     mm_camera_buf_notify_t stream_cb)
{
  int rc = MM_CAMERA_OK;
  mm_camera_channel_t *channel = NULL;

  channel = anki_mm_app_add_rdi_channel(test_obj, num_burst, stream_cb);
  if (NULL == channel) {
    CDBG_ERROR("%s: add channel failed", __func__);
    return -MM_CAMERA_E_GENERAL;
  }

  rc = mm_app_start_channel(test_obj, channel);
  if (MM_CAMERA_OK != rc) {
    CDBG_ERROR("%s:start rdi failed rc=%d\n", __func__, rc);
    mm_app_del_channel(test_obj, channel);
    return rc;
  }

  return rc;
}

int victor_stop_rdi(mm_camera_test_obj_t *test_obj)
{
  int rc = MM_CAMERA_OK;

  mm_camera_channel_t *channel =
    &test_obj->channels[MM_CHANNEL_TYPE_RDI];

  rc = mm_app_stop_and_del_rdi_channel(test_obj, channel);
  if (MM_CAMERA_OK != rc) {
    CDBG_ERROR("%s:Stop RDI failed rc=%d\n", __func__, rc);
  }

  return rc;
}

CameraObj* _rdi_camera = NULL;
camera_cb  user_frame_callback = NULL;
void camera_install_callback(camera_cb cb, CameraObj* camera)
{
  _rdi_camera = camera;
  user_frame_callback = cb;
}

static void mm_app_snapshot_notify_cb_raw(mm_camera_super_buf_t *bufs,
    void *user_data)
{
  int rc;
  static int frameid = 0;

  rc = pthread_mutex_trylock(&_rdi_camera->shutdown_lock);
  if(rc != 0)
  {
    return;
  }

  pthread_mutex_lock(&_rdi_camera->callback_lock);

  uint32_t i = 0;
  mm_camera_test_obj_t *pme = (mm_camera_test_obj_t *)user_data;
  mm_camera_channel_t *channel = NULL;
  mm_camera_stream_t *m_stream = NULL;
  mm_camera_buf_def_t *m_frame = NULL;

  if ((frameid % _rdi_camera->params.capture_params.fps_reduction) != 0) {
    goto EXIT;
  }

  if (!user_frame_callback) {
    goto EXIT;
  }

  /* find channel */
  for (i = 0; i < MM_CHANNEL_TYPE_MAX; i++) {
    if (pme->channels[i].ch_id == bufs->ch_id) {
      channel = &pme->channels[i];
      break;
    }
  }
  if (NULL == channel) {
    CDBG_ERROR("%s: Wrong channel id (%d)", __func__, bufs->ch_id);
    rc = -1;
    goto EXIT;
  }

  /* find snapshot stream */
  for (i = 0; i < channel->num_streams; i++) {
    if (channel->streams[i].s_config.stream_info->stream_type == CAM_STREAM_TYPE_RAW) {
      m_stream = &channel->streams[i];
      break;
    }
  }
  if (NULL == m_stream) {
    CDBG_ERROR("%s: cannot find snapshot stream", __func__);
    rc = -1;
    goto EXIT;
  }

  // Get raw frame info from stream
  cam_stream_buf_plane_info_t *buf_planes = &m_stream->s_config.stream_info->buf_planes;
  const int raw_frame_width = buf_planes->plane_info.mp[0].stride;
  const int raw_frame_height = buf_planes->plane_info.mp[0].scanline;


  // find snapshot frame
  if (user_frame_callback) {
    for (i = 0; i < bufs->num_bufs; i++) {
      if (bufs->bufs[i]->stream_id == m_stream->s_id) {

        m_frame = bufs->bufs[i];
        if (NULL == m_frame) {
          CDBG_ERROR("%s: main frame is NULL", __func__);
          rc = -1;
          goto EXIT;
        }

        uint8_t* inbuf = (uint8_t *)m_frame->buffer + m_frame->planes[i].data_offset;
        uint64_t timestamp = (m_frame->ts.tv_nsec + m_frame->ts.tv_sec * 1000000000LL);

        rc = user_frame_callback(inbuf,
                                 timestamp,
                                 frameid,
                                 raw_frame_width,
                                 raw_frame_height,
                                 ANKI_CAM_FORMAT_RGB888,
                                 _rdi_camera->callback_ctx);

        break;
      }
    }
  }

EXIT:
  for (i = 0; i < bufs->num_bufs; i++) {
    if (MM_CAMERA_OK != pme->cam->ops->qbuf(bufs->camera_handle,
                                            bufs->ch_id,
                                            bufs->bufs[i])) {
      CDBG_ERROR("%s: Failed in Qbuf\n", __func__);
    }
  }

  ++frameid;

  pthread_mutex_unlock(&_rdi_camera->callback_lock);
  pthread_mutex_unlock(&_rdi_camera->shutdown_lock);
}

int anki_mm_camera_start_rdi_capture(mm_camera_lib_handle *handle)
{
  int rc = victor_start_rdi(&handle->test_obj, 0, mm_app_snapshot_notify_cb_raw);
  if (rc != MM_CAMERA_OK) {
    CDBG_ERROR("%s: mm_app_start_rdi() err=%d\n",
               __func__, rc);
    return rc;
  }
  return 0;
}
