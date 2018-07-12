/**
 * File: mm_camera_anki.c
 *
 * Author: chapados
 * Created: 1/29/2018
 *
 * Description: mm-camera-interface adapter for camera capture from internal qcom stack
 *
 * Copyright: Anki, Inc. 2018
 *
 **/

#include <dlfcn.h>
#include <sys/mman.h>

//#include "kernel_includes.h"

#include "mm_qcamera_app.h"
#include "mm_qcamera_dbg.h"

/*************************************/
#include "mm_camera_anki.h"
#include "camera_process.h"

typedef struct {
  float r_gain;
  float g_gain;
  float b_gain;
} awb_gain_t;

typedef struct {
  awb_gain_t gain;
  // This is only a partial definition of the awb_update_t struct in
  // mct_event_stats.h which I did not want to include here because I
  // think it contains things relating to the ISP which do not belong here
  // This works because only the gain member of that struct is used by
  // sensor_set_awb_video_hdr_update in sensor.c, the gain member is also the
  // first member of that struct
} awb_update_t;

#define DEFAULT_RAW_RDI_FORMAT        CAM_FORMAT_BAYER_MIPI_RAW_10BPP_BGGR

typedef struct cameraobj_t {
  mm_camera_lib_handle lib_handle;
  void* callback_ctx;
  pthread_mutex_t callback_lock;
  struct anki_camera_params params;
  int is_running;
  anki_camera_pixel_format_t pixel_format;
} CameraObj;

static CameraObj gTheCamera;

int initBatchUpdate(mm_camera_test_obj_t *test_obj)
{
    parm_buffer_new_t *param_buf = ( parm_buffer_new_t * ) test_obj->parm_buf.mem_info.data;

    memset(param_buf, 0, sizeof(ONE_MB_OF_PARAMS));
    param_buf->num_entry = 0;
    param_buf->curr_size = 0;
    param_buf->tot_rem_size = ONE_MB_OF_PARAMS - sizeof(parm_buffer_new_t);

    return MM_CAMERA_OK;
}

int commitSetBatch(mm_camera_test_obj_t *test_obj)
{
    int rc = MM_CAMERA_OK;
    parm_buffer_new_t *param_buf = (parm_buffer_new_t *)test_obj->parm_buf.mem_info.data;

    if (param_buf->num_entry > 0) {
        rc = test_obj->cam->ops->set_parms(test_obj->cam->camera_handle, param_buf);
        CDBG("%s: commitSetBatch done\n",__func__);
    }

    return rc;
}


int commitGetBatch(mm_camera_test_obj_t *test_obj)
{
    int rc = MM_CAMERA_OK;
    parm_buffer_new_t *param_buf = (parm_buffer_new_t *)test_obj->parm_buf.mem_info.data;

    if (param_buf->num_entry > 0) {
        rc = test_obj->cam->ops->get_parms(test_obj->cam->camera_handle, param_buf);
        CDBG("%s: commitGetBatch done\n",__func__);
    }
    return rc;
}

int AddSetParmEntryToBatch(mm_camera_test_obj_t *test_obj,
                           cam_intf_parm_type_t paramType,
                           uint32_t paramLength,
                           void *paramValue)
{
    uint32_t j = 0;
    parm_buffer_new_t *param_buf = (parm_buffer_new_t *) test_obj->parm_buf.mem_info.data;
    uint32_t num_entry = param_buf->num_entry;
    uint32_t size_req = paramLength + sizeof(parm_entry_type_new_t);
    uint32_t aligned_size_req = (size_req + 3U) & (~3U);
    parm_entry_type_new_t *curr_param = (parm_entry_type_new_t *)&param_buf->entry[0];

    /* first search if the key is already present in the batch list
     * this is a search penalty but as the batch list is never more
     * than a few tens of entries at most,it should be ok.
     * if search performance becomes a bottleneck, we can
     * think of implementing a hashing mechanism.
     * but it is still better than the huge memory required for
     * direct indexing
     */
    for (j = 0; j < num_entry; j++) {
      if (paramType == curr_param->entry_type) {
        CDBG_ERROR("%s:Batch parameter overwrite for param: %d\n",
                                                __func__, paramType);
        break;
      }
      curr_param = GET_NEXT_PARAM(curr_param, parm_entry_type_new_t);
    }

    //new param, search not found
    if (j == num_entry) {
      if (aligned_size_req > param_buf->tot_rem_size) {
        CDBG_ERROR("%s:Batch buffer running out of size, commit and resend\n",__func__);
        commitSetBatch(test_obj);
        initBatchUpdate(test_obj);
      }

      curr_param = (parm_entry_type_new_t *)(&param_buf->entry[0] +
                                                  param_buf->curr_size);
      param_buf->curr_size += aligned_size_req;
      param_buf->tot_rem_size -= aligned_size_req;
      param_buf->num_entry++;
    }

    curr_param->entry_type = paramType;
    curr_param->size = (size_t)paramLength;
    curr_param->aligned_size = aligned_size_req;
    memcpy(&curr_param->data[0], paramValue, paramLength);
    CDBG("%s: num_entry: %d, paramType: %d, paramLength: %d, aligned_size_req: %d\n",
            __func__, param_buf->num_entry, paramType, paramLength, aligned_size_req);

    return MM_CAMERA_OK;
}

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

camera_cb  user_frame_callback = NULL;
void camera_install_callback(camera_cb cb)
{
  user_frame_callback = cb;
}

camera_cb  user_frame_callback_preview = NULL;
void camera_install_callback_preview(camera_cb cb)
{
  user_frame_callback_preview = cb;
}

static void mm_app_snapshot_notify_cb_preview(mm_camera_super_buf_t *bufs,
    void *user_data)
{
  int rc;
  static int frameid = 0;

  uint32_t i = 0;
  mm_camera_test_obj_t *pme = (mm_camera_test_obj_t *)user_data;
  mm_camera_channel_t *channel = NULL;
  mm_camera_stream_t *m_stream = NULL;
  mm_camera_buf_def_t *m_frame = NULL;

  if ((frameid % gTheCamera.params.capture_params.fps_reduction) != 0) {
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

        uint8_t* inbuf = (uint8_t *)m_frame->buffer + m_frame->planes[i].data_offset;;
        uint64_t timestamp = (m_frame->ts.tv_nsec + m_frame->ts.tv_sec * 1000000000LL);
	
        if(pthread_mutex_trylock(&gTheCamera.callback_lock) == 0)
        {
          rc = user_frame_callback_preview(inbuf,
                                           timestamp,
                                           frameid,
                                           raw_frame_width,
                                           raw_frame_height,
                                           gTheCamera.callback_ctx);
          pthread_mutex_unlock(&gTheCamera.callback_lock);
        }
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

static void mm_app_snapshot_notify_cb_raw(mm_camera_super_buf_t *bufs,
    void *user_data)
{
  int rc;
  static int frameid = 0;

  uint32_t i = 0;
  mm_camera_test_obj_t *pme = (mm_camera_test_obj_t *)user_data;
  mm_camera_channel_t *channel = NULL;
  mm_camera_stream_t *m_stream = NULL;
  mm_camera_buf_def_t *m_frame = NULL;

  if ((frameid % gTheCamera.params.capture_params.fps_reduction) != 0) {
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

        if(pthread_mutex_trylock(&gTheCamera.callback_lock) == 0)
        {
          rc = user_frame_callback(inbuf,
                                   timestamp,
                                   frameid,
                                   raw_frame_width,
                                   raw_frame_height,
                                   gTheCamera.callback_ctx);
          pthread_mutex_unlock(&gTheCamera.callback_lock);
        }
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

int setExposure(mm_camera_test_obj_t *test_obj, uint16_t exposure_ms, float gain)
{
    int rc = MM_CAMERA_OK;

    rc = initBatchUpdate(test_obj);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: Batch camera parameter update failed\n", __func__);
        goto ERROR;
    }

    cam_manual_exposure_t manual_exp;
    // Best case camera can capture 746 (window height + vBlank) lines @ 30fps
    // which is 22.38 lines/ms
    manual_exp.linecnt = (uint16_t)(((float)exposure_ms * 22.38));
    manual_exp.gain = gain;

    rc = AddSetParmEntryToBatch(test_obj,
                                CAM_INTF_PARM_RAW_MANUAL_EXPOSURE,
                                sizeof(manual_exp),
                                &manual_exp);

    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: Exposure parameter not added to batch\n", __func__);
        goto ERROR;
    }

    rc = commitSetBatch(test_obj);
    if (rc != MM_CAMERA_OK) {
        CDBG_ERROR("%s: Batch parameters commit failed\n", __func__);
        goto ERROR;
    }

ERROR:
    return rc;
}

int setAWBGain(mm_camera_test_obj_t *test_obj, float r_gain, float g_gain, float b_gain)
{
  int rc = MM_CAMERA_OK;

  rc = initBatchUpdate(test_obj);
  if (rc != MM_CAMERA_OK) {
      CDBG_ERROR("%s: Batch camera parameter update failed\n", __func__);
      goto ERROR;
  }

  awb_update_t awb_update;
  awb_update.gain.r_gain = r_gain;
  awb_update.gain.g_gain = g_gain;
  awb_update.gain.b_gain = b_gain;

  rc = AddSetParmEntryToBatch(test_obj,
                              CAM_INTF_PARM_RAW_AWB_GAIN,
                              sizeof(awb_update),
                              &awb_update);

  if (rc != MM_CAMERA_OK) {
      CDBG_ERROR("%s: Exposure parameter not added to batch\n", __func__);
      goto ERROR;
  }

  rc = commitSetBatch(test_obj);
  if (rc != MM_CAMERA_OK) {
      CDBG_ERROR("%s: Batch parameters commit failed\n", __func__);
      goto ERROR;
  }

ERROR:
    return rc;
}

/**************************************************************/
int camera_set_exposure(uint16_t exposure_ms, float gain)
{
  int rc;
  CameraObj* camera = &gTheCamera;

  if(camera->pixel_format == ANKI_CAM_FORMAT_YUV)
  {
    CDBG("%s: Not setting exposure while pixel_format is YUV", __func__);
    return -1;
  }

  rc = setExposure(&(camera->lib_handle.test_obj), exposure_ms, gain);

  return rc;
}

int camera_set_awb(float r_gain, float g_gain, float b_gain)
{
  int rc;
  CameraObj* camera = &gTheCamera;

  if(camera->pixel_format == ANKI_CAM_FORMAT_YUV)
  {
    CDBG("%s: Not setting awb while pixel_format is YUV", __func__);
    return -1;
  }

  rc = setAWBGain(&(camera->lib_handle.test_obj), r_gain, g_gain, b_gain);

  return rc;
}

int start_camera_capture()
{
  int rc = MM_CAMERA_OK;
  CameraObj* camera = &gTheCamera;

  switch(gTheCamera.pixel_format)
  {
    // For now bayer and rgb888 are the same
    // since we downsample bayer to rgb
    case ANKI_CAM_FORMAT_BAYER_MIPI_BGGR10:
    case ANKI_CAM_FORMAT_RGB888:
    {
      rc = anki_mm_camera_start_rdi_capture(&(camera->lib_handle));
    }
    break;

    case ANKI_CAM_FORMAT_YUV:
    {
      rc = victor_start_preview(&(camera->lib_handle));
    }
    break;
  }
  return rc;
}

int stop_camera_capture()
{
  int rc = MM_CAMERA_OK;
  CameraObj* camera = &gTheCamera;

  switch(gTheCamera.pixel_format)
  {
    // For now bayer and rgb888 are the same
    // since we downsample bayer to rgb
    case ANKI_CAM_FORMAT_BAYER_MIPI_BGGR10:
    case ANKI_CAM_FORMAT_RGB888:
    {
      rc = victor_stop_rdi(&(camera->lib_handle.test_obj));
    }
    break;

    case ANKI_CAM_FORMAT_YUV:
    {
      rc = victor_stop_preview(&(camera->lib_handle.test_obj));
      camera->lib_handle.stream_running = 0;
    }
    break;
  }
  return rc;
}

int camera_set_capture_format(struct anki_camera_capture* capture,
                              anki_camera_pixel_format_t format,
                              int(*realloc_with_format)
                                (struct anki_camera_capture* capture,
                                 anki_camera_pixel_format_t format))
{
  int rc;
  
  // Stop current camera capture
  rc = stop_camera_capture();
  if (rc != MM_CAMERA_OK) {
    CDBG_ERROR("%s:camera_set_capture_format() stop_capture err=%d format %d->%d\n", 
               __func__, 
               rc,
               gTheCamera.pixel_format,
               format);
    return -1;
  }

  // `capture` is the same as `gTheCamera.callback_ctx` both set from
  // `&ctx->camera` in camera_server.c
  // so we need to guard with callback_lock
  pthread_mutex_lock(&gTheCamera.callback_lock);

  // Reallocate memory for the new format
  rc = realloc_with_format(capture, format);
  if (rc != MM_CAMERA_OK) {
    pthread_mutex_unlock(&gTheCamera.callback_lock);
    CDBG_ERROR("%s:camera_set_capture_format() realloc_with_format err=%d format %d->%d\n", 
               __func__, 
               rc,
               gTheCamera.pixel_format,
               format);
    return -1;
  }
  
  pthread_mutex_unlock(&gTheCamera.callback_lock);

  // Update format
  gTheCamera.pixel_format = format;
  
  // Start camera capture using the new format
  rc = start_camera_capture();
  if (rc != MM_CAMERA_OK) {
    CDBG_ERROR("%s:camera_set_capture_format() start_capture err=%d format %d\n", 
               __func__, 
               rc,
               format);
    return -1;
  }

  return rc;
}

int camera_init()
{
  int rc = 0;
  /* mm_camera_test_obj_t test_obj; */
  /* memset(&test_obj, 0, sizeof(mm_camera_test_obj_t)); */

  memset(&gTheCamera, 0, sizeof(gTheCamera));

  pthread_mutex_init(&gTheCamera.callback_lock, NULL);

  //open camera lib
  rc = mm_camera_lib_open(&gTheCamera.lib_handle, 0);
  if (rc != MM_CAMERA_OK) {
    CDBG_ERROR("%s:mm_camera_lib_open() err=%d\n", __func__, rc);
    return -1;
  }

  //get number cameras
  int num_cameras =  gTheCamera.lib_handle.app_ctx.num_cameras;
  if (num_cameras <= 0) {
    CDBG_ERROR("%s: No camera sensors reported!", __func__);
    rc = -1;
  } //else we are goint to use the first one.

  return rc;
}

int camera_start(struct anki_camera_params* params, void* callback_ctx)
{
  int rc = MM_CAMERA_OK;

  camera_set_params(params);
  gTheCamera.callback_ctx = callback_ctx;

  rc = start_camera_capture();

  if (rc != MM_CAMERA_OK) {
    CDBG_ERROR("%s:mm_camera_lib_send_command() err=%d\n", __func__, rc);
    return rc;
  }

  gTheCamera.is_running = 1;

  return rc;
}

int camera_set_params(struct anki_camera_params* params)
{
  if (params->frame_callback_raw != NULL) {
    camera_install_callback(params->frame_callback_raw);
  }

  if (params->frame_callback_preview != NULL) {
    camera_install_callback_preview(params->frame_callback_preview);
  }

  const uint8_t fps_reduction = params->capture_params.fps_reduction;
  if ((fps_reduction == 0) ||
      (fps_reduction > ANKI_CAMERA_FPS_MAX)) {
    gTheCamera.params.capture_params.fps_reduction = 1;
  }
  else {
    gTheCamera.params.capture_params.fps_reduction = fps_reduction;
  }

  gTheCamera.pixel_format = params->capture_params.pixel_format;

//  if (!gTheCamera.is_running) {
//    gTheCamera.params.frame_format = params->frame_format;
//  }

  return 0;
}

int camera_stop()
{
  int rc;
  CameraObj* camera = &gTheCamera;

  rc = stop_camera_capture();

  if (rc != MM_CAMERA_OK) {
    CDBG_ERROR("%s: mm_app_stop_capture() err=%d\n",
               __func__, rc);
  }

  gTheCamera.is_running = 0;
  
  pthread_mutex_lock(&gTheCamera.callback_lock);
  gTheCamera.callback_ctx = NULL;
  pthread_mutex_unlock(&gTheCamera.callback_lock);

  // Not sure how safe this is...
  // This mutex is guarding against `callback_ctx` changing
  // while in a frame callback. 
  // There are a couple of cases...
  //  - Not in a frame callback when stop_camera_capture is called so
  //    no more frame callbacks should get called (this is an assumption)
  //  - In a callback when calling `stop_camera_capture()`
  //    and the function returns before the callback does, the above lock should
  //    block until the callback is finished at which point we should not get
  //    any more callbacks (same assumption as previous point)
  //  - In a callback when calling stop_camera_capture() and the callback finishes
  //    first. Will aquire the above lock without blocking and again no more callbacks
  //    should be called.
  // Assuming that all frame callbacks will stop being called after
  // the call to stop_camera_capture() then this should be fine
  pthread_mutex_destroy(&gTheCamera.callback_lock);

  return rc;
}


int camera_cleanup()
{
  mm_camera_lib_close(&gTheCamera.lib_handle);
  return 0;
}


#ifdef DEBUG_ANKI_CAMERA_STANDALONE_TEST

int main(int argc, char* argv[])
{
  int rc = camera_init();
  if (rc != 0) { return rc; }

  struct anki_camera_params params = {
    .capture_params = { .fps_reduction = 1 },
    .frame_callback = NULL,
  };
  rc = camera_start(&params, NULL);
  if (rc != 0) { return rc; }

  while (1) {
  }

  camera_stop();
  camera_cleanup();
}

#endif
