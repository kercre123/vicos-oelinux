/**
 * File: mm_camera_stream_snapshot.c
 *
 * Author: Al Chaussee
 * Created: 7/14/2018
 *
 * Description: Everything related to managing snapshot camera streams
 *
 * Copyright: Anki, Inc. 2018
 *
 **/

#include <dlfcn.h>
#include <sys/mman.h>

#include "mm_qcamera_dbg.h"
#include "mm_camera_stream_snapshot.h"

/*************************************/

CameraObj* _camera_snapshot = NULL;
camera_cb  user_frame_callback_snapshot = NULL;
void camera_install_callback_snapshot(camera_cb cb, CameraObj* camera)
{
  _camera_snapshot = camera;
  user_frame_callback_snapshot = cb;
}

static void mm_anki_app_snapshot_notify_cb(mm_camera_super_buf_t *bufs,
                                           void *user_data)
{
    static int frameid = 0;

    pthread_mutex_lock(&_camera_snapshot->callback_lock);

    int rc = 0;
    uint32_t i = 0;
    mm_camera_test_obj_t *pme = (mm_camera_test_obj_t *)user_data;
    mm_camera_channel_t *channel = NULL;
    mm_camera_stream_t *p_stream = NULL;
    mm_camera_stream_t *m_stream = NULL;
    mm_camera_buf_def_t *p_frame = NULL;
    mm_camera_buf_def_t *m_frame = NULL;

    CDBG("%s: BEGIN\n", __func__);

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
        goto error;
    }

    /* find snapshot stream */
    for (i = 0; i < channel->num_streams; i++) {
        if (channel->streams[i].s_config.stream_info->stream_type == CAM_STREAM_TYPE_SNAPSHOT) {
            m_stream = &channel->streams[i];
            break;
        }
    }
    if (NULL == m_stream) {
        CDBG_ERROR("%s: cannot find snapshot stream", __func__);
        rc = -1;
        goto error;
    }

    /* find snapshot frame */
    for (i = 0; i < bufs->num_bufs; i++) {
        if (bufs->bufs[i]->stream_id == m_stream->s_id) {
            m_frame = bufs->bufs[i];
            break;
        }
    }
    if (NULL == m_frame) {
        CDBG_ERROR("%s: main frame is NULL", __func__);
        rc = -1;
        goto error;
    }

    // REMOVE ME!
    mm_app_dump_frame(m_frame, "main", "yuv", m_frame->frame_idx);

    // Get raw frame info from stream
    cam_stream_buf_plane_info_t *buf_planes = &m_stream->s_config.stream_info->buf_planes;
    const int raw_frame_width = buf_planes->plane_info.mp[0].stride;
    const int raw_frame_height = buf_planes->plane_info.mp[0].scanline;

    uint8_t* inbuf = (uint8_t *)m_frame->buffer + m_frame->planes[i].data_offset;
    uint64_t timestamp = (m_frame->ts.tv_nsec + m_frame->ts.tv_sec * 1000000000LL);

    rc = user_frame_callback_snapshot(inbuf,
                                      timestamp,
                                      frameid,
                                      raw_frame_width,
                                      raw_frame_height,
                                      ANKI_CAM_FORMAT_YUV,
                                      _camera_snapshot->callback_ctx);

    mm_app_cache_ops((mm_camera_app_meminfo_t *)m_frame->mem_info,
                     ION_IOC_CLEAN_INV_CACHES);

error:
    /* buf done rcvd frames in error case */
    if ( 0 != rc ) {
        for (i=0; i<bufs->num_bufs; i++) {
            if (MM_CAMERA_OK != pme->cam->ops->qbuf(bufs->camera_handle,
                                                    bufs->ch_id,
                                                    bufs->bufs[i])) {
                CDBG_ERROR("%s: Failed in Qbuf\n", __func__);
            }
            mm_app_cache_ops((mm_camera_app_meminfo_t *)bufs->bufs[i]->mem_info,
                             ION_IOC_INV_CACHES);
        }
    }

    CDBG("%s: END\n", __func__);

    ++frameid;

    pthread_mutex_unlock(&_camera_snapshot->callback_lock);
}

int mm_anki_app_start_snapshot(mm_camera_test_obj_t *test_obj,
                              uint8_t num_snapshots)
{
    int32_t rc = MM_CAMERA_OK;
    mm_camera_channel_t *channel = NULL;
    mm_camera_stream_t *s_main = NULL;
    mm_camera_channel_attr_t attr;

    memset(&attr, 0, sizeof(mm_camera_channel_attr_t));
    attr.notify_mode = MM_CAMERA_SUPER_BUF_NOTIFY_CONTINUOUS;
    attr.max_unmatched_frames = 3;
    channel = mm_app_add_channel(test_obj,
                                 MM_CHANNEL_TYPE_CAPTURE,
                                 &attr,
                                 mm_anki_app_snapshot_notify_cb,
                                 test_obj);
    if (NULL == channel) {
        CDBG_ERROR("%s: add channel failed", __func__);
        return -MM_CAMERA_E_GENERAL;
    }

    s_main = mm_app_add_snapshot_stream(test_obj,
                                        channel,
                                        NULL,
                                        NULL,
                                        CAPTURE_BUF_NUM,
                                        num_snapshots);
    if (NULL == s_main) {
        CDBG_ERROR("%s: add main snapshot stream failed\n", __func__);
        mm_app_del_channel(test_obj, channel);
        return rc;
    }

    rc = mm_app_start_channel(test_obj, channel);
    if (MM_CAMERA_OK != rc) {
        CDBG_ERROR("%s:start zsl failed rc=%d\n", __func__, rc);
        mm_app_del_stream(test_obj, channel, s_main);
        mm_app_del_channel(test_obj, channel);
        return rc;
    }

    return rc;
}

int mm_anki_app_stop_snapshot(mm_camera_test_obj_t *test_obj)
{
    int rc = MM_CAMERA_OK;
    mm_camera_channel_t *ch = NULL;

    ch = mm_app_get_channel_by_type(test_obj, MM_CHANNEL_TYPE_CAPTURE);

    rc = mm_app_stop_and_del_channel(test_obj, ch);
    if (MM_CAMERA_OK != rc) {
        CDBG_ERROR("%s:stop capture channel failed rc=%d\n", __func__, rc);
    }

    return rc;
}