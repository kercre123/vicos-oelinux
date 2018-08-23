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

#include <dlfcn.h>
#include <sys/mman.h>

#include "mm_qcamera_dbg.h"
#include "mm_camera_stream_preview.h"

/*************************************/

CameraObj* _camera = NULL;
camera_cb  user_frame_callback_preview = NULL;
void camera_install_callback_preview(camera_cb cb, CameraObj* camera)
{
  _camera = camera;
  user_frame_callback_preview = cb;
}

static void mm_app_snapshot_notify_cb_preview(mm_camera_super_buf_t *bufs,
    void *user_data)
{
  int rc;
  static int frameid = 0;

  pthread_mutex_lock(&_camera->callback_lock);

  uint32_t i = 0;
  mm_camera_test_obj_t *pme = (mm_camera_test_obj_t *)user_data;
  mm_camera_channel_t *channel = NULL;
  mm_camera_stream_t *m_stream = NULL;
  mm_camera_buf_def_t *m_frame = NULL;

  if ((frameid % _camera->params.capture_params.fps_reduction) != 0) {
    goto EXIT;
  }

  if (!user_frame_callback_preview) {
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
    if (channel->streams[i].s_config.stream_info->stream_type == CAM_STREAM_TYPE_PREVIEW) {
      m_stream = &channel->streams[i];
      break;
    }
  }
  if (NULL == m_stream) {
    CDBG_ERROR("%s: cannot find preview stream", __func__);
    rc = -1;
    goto EXIT;
  }

  // Get raw frame info from stream
  cam_stream_buf_plane_info_t *buf_planes = &m_stream->s_config.stream_info->buf_planes;
  const int raw_frame_width = buf_planes->plane_info.mp[0].stride;
  const int raw_frame_height = buf_planes->plane_info.mp[0].scanline;

  // find snapshot frame
  if (user_frame_callback_preview) {
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
  
        rc = user_frame_callback_preview(inbuf,
                                         timestamp,
                                         frameid,
                                         raw_frame_width,
                                         raw_frame_height,
                                         ANKI_CAM_FORMAT_YUV,
                                         _camera->callback_ctx);

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

  pthread_mutex_unlock(&_camera->callback_lock);
}

mm_camera_stream_t * anki_mm_app_add_preview_stream(mm_camera_test_obj_t *test_obj,
                mm_camera_channel_t *channel,
                mm_camera_buf_notify_t stream_cb,
                void *userdata,
                uint8_t num_bufs)
{
  CDBG_ERROR("ADDING PREVIEW STREAM\n");
    int rc = MM_CAMERA_OK;
    mm_camera_stream_t *stream = NULL;
    cam_capability_t *cam_cap = (cam_capability_t *)(test_obj->cap_buf.buf.buffer);

    stream = mm_app_add_stream(test_obj, channel);
    if (NULL == stream) {
        CDBG_ERROR("%s: add stream failed\n", __func__);
        return NULL;
    }
    stream->s_config.mem_vtbl.get_bufs = mm_app_stream_initbuf;
    stream->s_config.mem_vtbl.put_bufs = mm_app_stream_deinitbuf;
    stream->s_config.mem_vtbl.clean_invalidate_buf =
      mm_app_stream_clean_invalidate_buf;
    stream->s_config.mem_vtbl.invalidate_buf = mm_app_stream_invalidate_buf;
    stream->s_config.mem_vtbl.user_data = (void *)stream;
    stream->s_config.stream_cb = stream_cb;
    stream->s_config.userdata = userdata;
    stream->num_of_bufs = num_bufs;

    stream->s_config.stream_info = (cam_stream_info_t *)stream->s_info_buf.buf.buffer;
    memset(stream->s_config.stream_info, 0, sizeof(cam_stream_info_t));
    stream->s_config.stream_info->stream_type = CAM_STREAM_TYPE_PREVIEW;
    stream->s_config.stream_info->streaming_mode = CAM_STREAMING_MODE_CONTINUOUS;
    stream->s_config.stream_info->fmt = DEFAULT_PREVIEW_FORMAT;

    stream->s_config.stream_info->dim.width = DEFAULT_PREVIEW_WIDTH;
    stream->s_config.stream_info->dim.height = DEFAULT_PREVIEW_HEIGHT;

    stream->s_config.padding_info = cam_cap->padding_info;

    rc = mm_app_config_stream(test_obj, channel, stream, &stream->s_config);
    if (MM_CAMERA_OK != rc) {
        CDBG_ERROR("%s:config preview stream err=%d\n", __func__, rc);
        return NULL;
    }

    return stream;
}

mm_camera_channel_t * anki_mm_app_add_preview_channel(mm_camera_test_obj_t *test_obj)
{
    mm_camera_channel_t *channel = NULL;
    mm_camera_stream_t *stream = NULL;

    channel = mm_app_add_channel(test_obj,
                                 MM_CHANNEL_TYPE_PREVIEW,
                                 NULL,
                                 NULL,
                                 NULL);
    if (NULL == channel) {
        CDBG_ERROR("%s: add channel failed", __func__);
        return NULL;
    }

    stream = anki_mm_app_add_preview_stream(test_obj,
              channel,
              mm_app_snapshot_notify_cb_preview,
              (void *)test_obj,
              PREVIEW_BUF_NUM);

    if (NULL == stream) {
        CDBG_ERROR("%s: add stream failed\n", __func__);
        mm_app_del_channel(test_obj, channel);
        return NULL;
    }

    return channel;
}

int victor_start_preview(mm_camera_lib_handle *handle)
{
  mm_camera_test_obj_t* test_obj = &(handle->test_obj);

  int rc = MM_CAMERA_OK;
  mm_camera_channel_t *p_ch = NULL;

  test_obj->enable_reproc = ENABLE_REPROCESSING;

  p_ch = anki_mm_app_add_preview_channel(test_obj);
  if (NULL == p_ch) 
  {
    CDBG_ERROR("%s: add preview channel failed", __func__);
    rc = -MM_CAMERA_E_GENERAL;
    return rc;
  }

  rc = mm_app_start_channel(test_obj, p_ch);
  if (MM_CAMERA_OK != rc) {
      CDBG_ERROR("%s:start preview failed rc=%d\n", __func__, rc);
      int i;
      mm_camera_stream_t* stream = NULL;
      for (i = 0; i < p_ch->num_streams; i++) {
          stream = &p_ch->streams[i];
          mm_app_del_stream(test_obj, p_ch, stream);
      }
      mm_app_del_channel(test_obj, p_ch);
      return rc;
  }

  handle->stream_running = 1;

  return rc;
}

int victor_stop_preview(mm_camera_test_obj_t* test_obj)
{
  int rc = MM_CAMERA_OK;
  mm_camera_channel_t *p_ch = NULL;

  p_ch = mm_app_get_channel_by_type(test_obj, MM_CHANNEL_TYPE_PREVIEW);

  rc = mm_app_stop_and_del_channel(test_obj, p_ch);
  if (MM_CAMERA_OK != rc) 
  {
    CDBG_ERROR("%s:Stop Preview failed rc=%d\n", __func__, rc);
  }

  return rc;
}
