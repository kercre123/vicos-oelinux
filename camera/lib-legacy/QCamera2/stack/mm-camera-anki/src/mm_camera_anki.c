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
#include "mm_camera_stream_rdi.h"
#include "mm_camera_stream_preview.h"
#include "mm_camera_stream_snapshot.h"

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
    case ANKI_CAM_FORMAT_BAYER_MIPI_BGGR10_2MP:
    case ANKI_CAM_FORMAT_RGB888_2MP:
    {
      rc = anki_mm_camera_start_rdi_capture(&(camera->lib_handle));
    }
    break;

    case ANKI_CAM_FORMAT_YUV:
    case ANKI_CAM_FORMAT_YUV_2MP:
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

int camera_start_snapshot()
{
  if(gTheCamera.pixel_format != ANKI_CAM_FORMAT_YUV)
  {
    CDBG_ERROR("%s: can't take snapshot when not in yuv/preview mode",
               __func__);
    return -1;
  }

  int rc = stop_camera_capture();
  if(rc != 0)
  {
    CDBG_ERROR("%s: failed to stop camera capture",
               __func__);
    return rc;
  }

  rc = mm_anki_app_start_snapshot(&(gTheCamera.lib_handle.test_obj), 1);
  if(rc != 0)
  {
    CDBG_ERROR("%s: failed to start snapshot",
               __func__);
    return rc;
  }

  return rc;
}

int camera_stop_snapshot()
{
  int rc = mm_anki_app_stop_snapshot(&(gTheCamera.lib_handle.test_obj));
  if(rc != 0)
  {
    CDBG_ERROR("%s: failed to stop snapshot",
               __func__);
    return -1;
  }

  rc = start_camera_capture();
  if(rc != 0)
  {
    CDBG_ERROR("%s: failed to restart capture",
               __func__);
    return -1;
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
  
  // `capture` is the same as `gTheCamera.callback_ctx` both set from
  // `&ctx->camera` in camera_server.c
  // so we need to guard with callback_lock
  pthread_mutex_lock(&gTheCamera.callback_lock);

  // Stop current camera capture
  rc = stop_camera_capture();
  if (rc != MM_CAMERA_OK) {
    CDBG_ERROR("%s:camera_set_capture_format() stop_capture err=%d format %d->%d\n", 
               __func__, 
               rc,
               gTheCamera.pixel_format,
               format);
    pthread_mutex_unlock(&gTheCamera.callback_lock);
    return -1;
  }

  // Reallocate memory for the new format
  rc = realloc_with_format(capture, format);
  if (rc != MM_CAMERA_OK) {
    CDBG_ERROR("%s:camera_set_capture_format() realloc_with_format err=%d format %d->%d\n", 
               __func__, 
               rc,
               gTheCamera.pixel_format,
               format);
    pthread_mutex_unlock(&gTheCamera.callback_lock);
    return -1;
  }

  // Update format
  gTheCamera.pixel_format = format;
  
  // Start camera capture using the new format
  rc = start_camera_capture();
  if (rc != MM_CAMERA_OK) {
    CDBG_ERROR("%s:camera_set_capture_format() start_capture err=%d format %d\n", 
               __func__, 
               rc,
               format);
    pthread_mutex_unlock(&gTheCamera.callback_lock);
    return -1;
  }

  pthread_mutex_unlock(&gTheCamera.callback_lock);

  return rc;
}

int camera_init()
{
  int rc = 0;
  /* mm_camera_test_obj_t test_obj; */
  /* memset(&test_obj, 0, sizeof(mm_camera_test_obj_t)); */

  memset(&gTheCamera, 0, ((uint8_t*)&gTheCamera.resetable_fields - (uint8_t*)&gTheCamera));

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

  if(!gTheCamera.shutdown_lock_inited)
  {
    pthread_mutex_init(&gTheCamera.shutdown_lock, NULL);
    gTheCamera.shutdown_lock_inited = 1;
  }
  else
  {
    pthread_mutex_unlock(&gTheCamera.shutdown_lock);
  }

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
    camera_install_callback(params->frame_callback_raw, &gTheCamera);
  }

  if (params->frame_callback_preview != NULL) {
    camera_install_callback_preview(params->frame_callback_preview, &gTheCamera);
  }

  if (params->frame_callback_snapshot != NULL) {
    camera_install_callback_snapshot(params->frame_callback_snapshot, &gTheCamera);
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
  pthread_mutex_lock(&gTheCamera.shutdown_lock);

  CameraObj* camera = &gTheCamera;

  pthread_mutex_lock(&camera->callback_lock);

  rc = stop_camera_capture();

  if (rc != MM_CAMERA_OK) {
    CDBG_ERROR("%s: mm_app_stop_capture() err=%d\n",
               __func__, rc);
  }

  gTheCamera.is_running = 0;
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
