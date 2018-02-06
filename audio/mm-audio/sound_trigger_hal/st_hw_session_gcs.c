/*
 *
 * This file implements the hw session functionality specific to LSM HW
 * sessions that use GCS.
 *
 * Copyright (c) 2016-2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 *
 */
#define LOG_TAG "sound_trigger_hw"
/* #define LOG_NDEBUG 0 */
#define LOG_NDDEBUG 0

#include <pthread.h>
#include <string.h>
#include <errno.h>
#include <dlfcn.h>
#include <cutils/log.h>
#include <fcntl.h>
#include <stdlib.h>
#include "st_hw_session_gcs.h"
#include "sound_trigger_platform.h"
#include "sound_trigger_hw.h"
#include "st_graphite_api.h"
#include "gcs_api.h"

#define GCS_LIB "libgcs.so"
#define GCS_CONCURRENT_READS_CNT 2
#define WDSP_SYSFS_NAME "/dev/wcd_dsp0_control"

static int reg_sm(st_hw_session_t *p_ses,
    void *sm_data,
    sound_trigger_sound_model_type_t sm_type);
static int reg_sm_params(st_hw_session_t *p_ses,
    unsigned int recognition_mode,
    bool capture_requested,
    unsigned int num_conf_levels,
    unsigned char *conf_levels,
    struct sound_trigger_recognition_config *rc_config,
    sound_trigger_sound_model_type_t sm_type);
static int read_pcm(st_hw_session_t *p_ses,
    unsigned char *buf,
    unsigned int bytes);
static void process_lab_capture(st_hw_session_t *p_ses);
static int dereg_sm(st_hw_session_t *p_ses, bool capture_requested);
static int dereg_sm_params(st_hw_session_t *p_ses);
static int start(st_hw_session_t *p_ses);
static int restart(st_hw_session_t *p_ses,
    unsigned int recognition_mode,
    bool capture_requested, unsigned int num_conf_levels,
    unsigned char *conf_levels,
    struct sound_trigger_recognition_config *rc_config __unused,
    sound_trigger_sound_model_type_t sm_type);
static int stop(st_hw_session_t *p_ses);
static int stop_buffering(st_hw_session_t *p_ses, bool capture_requested);
static int set_device(st_hw_session_t *p_ses,
    bool enable);
static int disable_device(st_hw_session_t *p_ses);
static int enable_device(st_hw_session_t *p_ses);

static struct pcm_config stdev_cpe_pcm_config = {
    .channels = SOUND_TRIGGER_CHANNEL_MODE_MONO,
    .rate = SOUND_TRIGGER_SAMPLING_RATE_16000,
    .period_size = ST_GRAPHITE_LAB_PERIOD_SIZE_IN_SAMPLES,
    .period_count = ST_GRAPHITE_LAB_PERIOD_COUNT,
    .format = PCM_FORMAT_S16_LE,
};

static struct st_session_fptrs fptrs_gcs = {
    .reg_sm = reg_sm,
    .reg_sm_params = reg_sm_params,
    .dereg_sm = dereg_sm,
    .dereg_sm_params = dereg_sm_params,
    .start = start,
    .restart = restart,
    .stop = stop,
    .stop_buffering = stop_buffering,
    .disable_device = disable_device,
    .enable_device = enable_device,
    .set_device = set_device,
    .read_pcm = read_pcm,
    .process_lab_capture = process_lab_capture,
};

/* gcs functions loaded from dynamic library */
static int32_t(*gcs_init_fn)(void);
static int32_t(*gcs_open_fn)(uint32_t UID,
    uint32_t DID,
    uint32_t *graph_handle);
static int32_t(*gcs_enable_fn)(uint32_t graph_handle,
    void *non_persist_ucal,
    uint32_t size_ucal);
static int32_t(*gcs_disable_fn)(uint32_t graph_handle);
static int32_t(*gcs_close_fn)(uint32_t graph_handle);
static int32_t(*gcs_load_data_fn)(uint32_t graph_handle,
    void *data,
    uint32_t data_size,
    uint32_t *data_handle);
static int32_t(*gcs_unload_data_fn)(uint32_t graph_handle, uint32_t data_handle);
static int32_t(*gcs_deinit_fn)(void);
static int32_t(*gcs_register_for_event_fn)(uint32_t graph_handle,
    struct gcs_module_param_info *module_param_info,
    event_cb_ptr cb_handler, void *cookie);

static int32_t(*gcs_register_data_cmd_handler_fn)(uint32_t graph_handle,
    void *data_cmd_cb_handler,
    void *cookie);
static int32_t(*gcs_start_buff_xfer_fn)(uint32_t graph_handle, enum gcs_data_xfer dir);
static int32_t(*gcs_stop_buff_xfer_fn)(uint32_t graph_handle, enum gcs_data_xfer dir);
static int32_t(*gcs_send_data_cmd_fn)(uint32_t graph_handle,
    int8_t *payload,
    uint32_t payload_size);

static int32_t(*gcs_enable_device_fn)(uint32_t graph_handle,
    uint32_t UID, int8_t *payload, uint32_t payload_size);
static int32_t(*gcs_disable_device_fn)(uint32_t graph_handle);

/* used to output pcm to file for debugging */
ST_DBG_DECLARE(static FILE *lab_fp_gcs = NULL;);
ST_DBG_DECLARE(static FILE *lab_fp_client = NULL;);

struct st_hw_gcs_data {
    void *lib_handle;
    int sysfs_fd;
};

static struct st_hw_gcs_data gcs_data = {.lib_handle = NULL, .sysfs_fd = 0 };

/* circular buffer utility functions */

/*
 Returns number of written bytes in a circular buffer with
 read ptr rd_p and write ptr wr_p
*/
inline uint32_t circ_buff_filled(uint8_t *rd_p, uint8_t *wr_p, uint32_t size)
{
    return (wr_p > rd_p ? (uint32_t)(wr_p - rd_p - 1) : (uint32_t)(size - (rd_p - wr_p + 1)));
}

inline uint32_t circ_buff_avail_space(uint8_t *rd_p,
    uint8_t *wr_p,
    uint32_t size)
{
    return (wr_p > rd_p ? (uint32_t)(size - (wr_p - rd_p - 1)) : (uint32_t)(rd_p - wr_p + 1));
}

inline uint8_t* circ_buff_move_ptr_fwd(uint8_t *circ_buff_start,
    uint8_t *ptr,
    uint32_t bytes_num,
    uint32_t size)
{
    return (ptr + bytes_num < circ_buff_start + size) ?
           ptr + bytes_num : ptr + bytes_num - size;
}

int32_t gcs_event_cb(uint32_t graph_hdl,
    struct gcs_event_rsp *ev,
    void *private_data)
{
    ALOGD("%s: Enter...", __func__);
    st_hw_session_gcs_t *p_ses = NULL;
    int32_t status = 0;
    int mutex_ret = 0;

    if (!private_data || !ev) {
        ALOGE("%s: received invalid params", __func__);
        status = -EINVAL;
        goto exit;
    }

    p_ses = (st_hw_session_gcs_t *)private_data;

    if (p_ses->graph_handle != graph_hdl) {
        ALOGE("%s: graph_hdl mismatch param has %d but private data has %d",
            __func__, graph_hdl, p_ses->graph_handle);
        status = -EINVAL;
        goto exit;
    }

    if (ev->payload_size == 0) {
        ALOGE("%s: received detection payload size of 0", __func__);
        status = -EINVAL;
        goto exit;
    }

    /*
     * We use try_lock here, if a detection event is currently being processed, we
     * will ignore incoming detections until processing the current one is completed.
     * Anyway we are not expected to get any detection notification from FW until the
     * previous one is processed and the application restarts recognition
     */
    mutex_ret = pthread_mutex_trylock(&p_ses->callback_thread_lock);
    if (!mutex_ret) {
        ALOGV("%s: got mutex lock", __func__);
        p_ses->detect_payload_size = ev->payload_size;
        memcpy(p_ses->detect_payload, ev->payload, p_ses->detect_payload_size);
        p_ses->detection_signaled = true;
        pthread_cond_signal(&p_ses->callback_thread_cond);
        pthread_mutex_unlock(&p_ses->callback_thread_lock);
    } else {
        ALOGD("%s: previous event in-progress, ignore event", __func__);
    }

exit:
    return status;
}

int32_t gcs_data_cmdrsp_cb(uint32_t graph_handle,
    void *rsp,
    size_t rsp_size,
    void *cookie, int32_t cmd_status)
{
    int status = 0;
    st_hw_session_gcs_t *p_ses = NULL;
    struct graphite_data_cmdrsp_hdr *hdr = NULL;
    struct gcs_cmd_readrsp_payload_t *payload = NULL;
    uint32_t buff_sz = 0, bytes_to_tail = 0, avail_space = 0;
    uint8_t *rd_ptr = NULL,*wr_ptr = NULL,*buff = NULL;

    p_ses = (st_hw_session_gcs_t *)cookie;

    if (!p_ses) {
        ALOGE("%s: received NULL cookie", __func__);
        return -EINVAL;
    }

    if (p_ses->graph_handle != graph_handle) {
        ALOGE("%s: graph_hdl mismatch param has %d but private data has %d",
            __func__, graph_handle, p_ses->graph_handle);
        return -EINVAL;
    }

    if (0 > cmd_status) {
        ALOGE("%s: received failed cmdrsp status: %d", __func__, cmd_status);
        return status;
    }

    if (0 == rsp_size) {
        ALOGE("%s: received response size of 0", __func__);
        return -EINVAL;
    }

    /* parse CMDRSP message */
    hdr = (struct graphite_data_cmdrsp_hdr *)rsp;

    if (hdr->module_id != p_ses->gcs_usecase->params[READ_RSP].module_id ||
        hdr->instance_id != p_ses->gcs_usecase->params[READ_RSP].instance_id ||
        hdr->cmd_id != p_ses->gcs_usecase->params[READ_RSP].param_id) {
        ALOGE("%s: received unexpected parameters module_id %d, instance_id %d"
            ", cmd_id %d expected module_id %d, instance_id %d, cmd_id %d",
            __func__, hdr->module_id, hdr->instance_id, hdr->cmd_id,
            p_ses->gcs_usecase->params[READ_RSP].module_id,
            p_ses->gcs_usecase->params[READ_RSP].instance_id,
            p_ses->gcs_usecase->params[READ_RSP].param_id);
        return -EINVAL;
    }

    /* received response for READ CMD */
    payload = (struct gcs_cmd_readrsp_payload_t *)((uint8_t *)rsp
        + sizeof(struct graphite_data_cmdrsp_hdr));

    if (GCS_READ_CMDRSP_STATUS_SUCCESS == payload->status) {
        buff_sz = hdr->size_in_bytes - sizeof(struct gcs_cmd_readrsp_payload_t);
        buff = (uint8_t *)payload + sizeof(struct gcs_cmd_readrsp_payload_t);

        ST_DBG_FILE_WRITE(lab_fp_gcs, buff, buff_sz);

        pthread_mutex_lock(&p_ses->circ_buff_lock);
        if (!p_ses->exit_buffering) {

            rd_ptr = p_ses->rd_ptr;
            wr_ptr = p_ses->wr_ptr;

            /* we overwrite old data if we dont have enough space, this
             is to avoid blocking in callback context */
            avail_space = circ_buff_avail_space(rd_ptr, wr_ptr,
                p_ses->circ_buff_sz);

            /* copy the bytes */
            bytes_to_tail = p_ses->circ_buff_tail - wr_ptr + 1;
            ALOGVV("%s: RCVD = %d, AVAIL = %d, TO_TAIL = %d", __func__,
                buff_sz, avail_space, bytes_to_tail);

            if (bytes_to_tail < buff_sz) {
                /* buffer wraps */
                memcpy(wr_ptr, buff, bytes_to_tail);
                memcpy(p_ses->circ_buff, buff + bytes_to_tail,
                    buff_sz - bytes_to_tail);
            } else {
                memcpy(wr_ptr, buff, buff_sz);
            }
            p_ses->wr_ptr = circ_buff_move_ptr_fwd(p_ses->circ_buff, wr_ptr, buff_sz,
                p_ses->circ_buff_sz);
            if (buff_sz > avail_space) {
                /* in-case we have an overwrite, update rd_ptr to move forward as well */
                ALOGD("%s: overwrite occured buff_size %d, avail_space %d",
                    __func__, buff_sz, avail_space);
                p_ses->rd_ptr = circ_buff_move_ptr_fwd(p_ses->circ_buff, rd_ptr, buff_sz - avail_space,
                    p_ses->circ_buff_sz);
            }

            /*
             * wakeup both reader thread to read and buffering thread to
             * request more data
             */
            ALOGVV("%s: signaling circ_buff_cond", __func__);
            ++p_ses->read_rsp_cnt;
            pthread_cond_broadcast(&p_ses->circ_buff_cond);
        }
        pthread_mutex_unlock(&p_ses->circ_buff_lock);
    }

    return status;
}

static void* callback_thread_loop(void *context)
{
    st_hw_session_gcs_t *p_ses = (st_hw_session_gcs_t *)context;
    st_hw_sess_event_t hw_ses_event; /* used to report event to st_session */
    struct gcs_det_engine_event *p_det = NULL;
    uint32_t event_status = 0;

    if (!p_ses) {
        ALOGE("%s: Received null session ptr", __func__);
        goto exit;
    }
    ALOGV("%s: callback thread started...", __func__);
    pthread_mutex_lock(&p_ses->callback_thread_lock);
    while (!p_ses->detection_signaled && !p_ses->exit_detection) {
        pthread_cond_wait(&(p_ses->callback_thread_cond),
            &(p_ses->callback_thread_lock));
        ALOGV("%s: came out of cond_wait...", __func__);

        if (p_ses->exit_detection)
            break;

        if (p_ses->detection_signaled) {
            ALOGV("%s: detection signaled", __func__);
            p_ses->detection_signaled = false;

            p_det = (struct gcs_det_engine_event *)p_ses->detect_payload;

            /* get detection status */
            if (p_det->status == GCS_DETECTION_ENGINE_EVENT_DETECTED)
                event_status = RECOGNITION_STATUS_SUCCESS;
            else if (p_det->status == GCS_DETECTION_ENGINE_EVENT_FAILED)
                event_status = RECOGNITION_STATUS_FAILURE;

            /* inform st_sessoin of the event, st_session knows how to parse list of conf levels */
            ALOGV("%s: notifying st_session of the detection event, status = %d",
                __func__, event_status);

            ST_DBG_DECLARE(FILE *debug_fd;);
            ST_DBG_FILE_OPEN_WR(debug_fd, "/data/misc/audio/", "detection_event", "bin", 1);
            ST_DBG_FILE_WRITE(debug_fd, p_ses->detect_payload,  p_ses->detect_payload_size);
            ST_DBG_FILE_CLOSE(debug_fd);

            hw_ses_event.event_id = ST_HW_SESS_EVENT_DETECTED;
            hw_ses_event.payload.detected.timestamp = 0;
            hw_ses_event.payload.detected.detect_status = event_status;
            hw_ses_event.payload.detected.payload_size = p_det->custom_payload_sz;
            hw_ses_event.payload.detected.detect_payload =
                p_ses->detect_payload + sizeof(struct gcs_det_engine_event);
            p_ses->common.callback_to_st_session(&hw_ses_event, p_ses->common.cookie);
        }

    }

    pthread_mutex_unlock(&p_ses->callback_thread_lock);
    ALOGV("%s: callback thread ending...", __func__);
exit:
    return NULL;
}

static int reg_sm(st_hw_session_t *p_ses,
    void *sm_data,
    sound_trigger_sound_model_type_t sm_type __unused)
{
    int status = 0;
    uint8_t *load_sm_msg = NULL;
    struct graphite_cal_header *sm_msg_hdr = NULL;
    size_t load_sm_msg_sz = 0;
    struct st_vendor_info *v_info = p_ses->vendor_uuid_info;
    struct gcs_module_param_info gcs_module_info; /* used to register for ev*/
    struct sound_trigger_phrase_sound_model *phrase_sm =
        (struct sound_trigger_phrase_sound_model *)sm_data;
    struct sound_trigger_sound_model *common_sm =
        (struct sound_trigger_sound_model *)sm_data;
    size_t sm_data_size = 0;
    uint32_t sm_data_offset = 0;
    st_hw_session_gcs_t *p_gcs_ses = (st_hw_session_gcs_t *)p_ses;
    int st_device = 0, device_acdb_id = 0;
    int capture_device;

    /* load WDSP image */
    if (gcs_data.sysfs_fd >= 0) {
        write(gcs_data.sysfs_fd, "1", 1);
    } else {
        ALOGE("%s: Failed to open %s with exiting ", __func__,
            WDSP_SYSFS_NAME);
        return -EIO;
    }

    /* get the acdb device id to use when opening a graph */
    capture_device = platform_stdev_get_capture_device(p_ses->stdev->platform);
    st_device = platform_stdev_get_device(p_ses->stdev->platform,
        v_info, capture_device, p_ses->exec_mode);
    if (st_device == ST_DEVICE_NONE) {
        ALOGE("%s: Could not find valid device",__func__);
        status = -EINVAL;
        goto cleanup2;
    }

    device_acdb_id = platform_stdev_get_acdb_id(st_device,
        p_ses->exec_mode);
    if (0 > device_acdb_id) {
        ALOGE("%s: Could not get device ACDB ID", __func__);
        status = -EINVAL;
        goto cleanup2;
    }

    /* allocate a gcs use-case for this session from vendor info */
    platform_alloc_gcs_usecase(v_info, &p_gcs_ses->gcs_usecase);
    if (!p_gcs_ses->gcs_usecase) {
        ALOGE("%s: failed to allocate gcs usecase for the session", __func__);
        status = -ENOMEM;
        goto cleanup2;
    }

    p_ses->config = stdev_cpe_pcm_config;

    ALOGD("%s:[%d] calling gcs_open with uid %d, did %d", __func__,
        p_ses->sm_handle, p_gcs_ses->gcs_usecase->uid, device_acdb_id);
    status = gcs_open_fn(p_gcs_ses->gcs_usecase->uid, device_acdb_id,
        &p_gcs_ses->graph_handle);
    if (status) {
        ALOGE("%s: gcs_open failed with status %d", __func__, status);
        goto cleanup1;
    }

    if (sm_type == SOUND_MODEL_TYPE_KEYPHRASE) {
        sm_data_size = phrase_sm->common.data_size;
        sm_data_offset = phrase_sm->common.data_offset;
    } else {
        sm_data_size = common_sm->data_size;
        sm_data_offset = common_sm->data_offset;
    }

    /* calculate size of load sm msg */
    load_sm_msg_sz = sizeof(struct graphite_cal_header); /* param header for sound model param */
    load_sm_msg_sz += sm_data_size; /* SM opaque data size from upper layers */
    load_sm_msg_sz = ALIGN(load_sm_msg_sz, 4);

    load_sm_msg = calloc(load_sm_msg_sz, sizeof(uint8_t));
    if (!load_sm_msg) {
        ALOGE("%s: failed to allocate memory for sm msg, size = %zu", __func__,
            load_sm_msg_sz);
        status = -ENOMEM;
        goto cleanup1;
    }
    sm_msg_hdr = (struct graphite_cal_header *)load_sm_msg;
    sm_msg_hdr->module_id = p_gcs_ses->gcs_usecase->
        params[LOAD_SOUND_MODEL].module_id;
    sm_msg_hdr->instance_id = p_gcs_ses->gcs_usecase->
        params[LOAD_SOUND_MODEL].instance_id;
    sm_msg_hdr->param_id = p_gcs_ses->gcs_usecase->
        params[LOAD_SOUND_MODEL].param_id;
    sm_msg_hdr->size = sm_data_size;

    ALOGV("%s: sm cal header MID %x, IID %x, PID %x \n sm data %p, sm size %zu,"
        " alloc size %zu", __func__, sm_msg_hdr->module_id,
        sm_msg_hdr->instance_id, sm_msg_hdr->param_id, sm_data,
        sm_data_size, load_sm_msg_sz);

    memcpy(load_sm_msg + sizeof(struct graphite_cal_header),
        (uint8_t *)sm_data + sm_data_offset, sm_data_size);


    ST_DBG_DECLARE(FILE *debug_fd = NULL;);
    ST_DBG_FILE_OPEN_WR(debug_fd, "/data/misc/audio/", "load_sm", "bin", 1);
    ST_DBG_FILE_WRITE(debug_fd, load_sm_msg, load_sm_msg_sz);
    ST_DBG_FILE_CLOSE(debug_fd);

    ALOGD("%s:[%d] calling gcs_load_data with graph_handle %d, load_sm_msg %p, "
        "load_sm_msg_sz %zu", __func__, p_ses->sm_handle, p_gcs_ses->graph_handle,
        load_sm_msg, load_sm_msg_sz);
    status = gcs_load_data_fn(p_gcs_ses->graph_handle, load_sm_msg,
        (uint32_t)load_sm_msg_sz, &p_gcs_ses->loaded_sm_handle);
    if (status) {
        ALOGE("%s: gcs_load_data failed with status %d", __func__, status);
        goto cleanup1;
    }

    free(load_sm_msg);
    load_sm_msg = NULL;

    /* register callback for event handling */
    gcs_module_info.module_info.MID =
        p_gcs_ses->gcs_usecase->params[DETECTION_EVENT].module_id;
    gcs_module_info.module_info.IID =
        p_gcs_ses->gcs_usecase->params[DETECTION_EVENT].instance_id;
    gcs_module_info.PID =
        p_gcs_ses->gcs_usecase->params[DETECTION_EVENT].param_id;

    ALOGD("%s:[%d] calling gcs_register_for_event with MID %x, IID %x, PID %x", __func__,
        p_ses->sm_handle, gcs_module_info.module_info.MID,
        gcs_module_info.module_info.IID, gcs_module_info.PID);
    status = gcs_register_for_event_fn(p_gcs_ses->graph_handle, &gcs_module_info,
        gcs_event_cb, p_gcs_ses);
    if (status) {
        ALOGE("%s: gcs_register_for_event failed with status %d", __func__, status);
        goto cleanup1;
    }

    /* register callback for data cmd handling */
    ALOGD("%s:[%d] calling gcs_register_data_cmd_handler with handle %d, cb %p, "
        "cookie %p", __func__, p_ses->sm_handle,
        p_gcs_ses->graph_handle, gcs_data_cmdrsp_cb, p_gcs_ses);
    status = gcs_register_data_cmd_handler_fn(p_gcs_ses->graph_handle,
        (data_cmd_cb_ptr)gcs_data_cmdrsp_cb, p_gcs_ses);
    if (status) {
        ALOGE("%s: gcs_register_data_cmd_handler failed with status %d",
            __func__, status);
        goto cleanup1;
    }

    return status;

cleanup1:
    if (load_sm_msg)
        free(load_sm_msg);
    platform_free_gcs_usecase(v_info, p_gcs_ses->gcs_usecase);
cleanup2:
    /* unload WDSP image */
    if (gcs_data.sysfs_fd >= 0)
        write(gcs_data.sysfs_fd, "0", 1);

    return status;
}

static int reg_sm_params(st_hw_session_t *p_ses,
    unsigned int recognition_mode,
    bool capture_requested,
    unsigned int custom_payload_size,
    unsigned char *custom_payload,
    struct sound_trigger_recognition_config *rc_config,
    sound_trigger_sound_model_type_t sm_type __unused)
{
    int status = 0;
    struct gcs_det_engine_config_param *p_msg = NULL;
    struct gcs_det_engine_custom_config_param *cc_msg = NULL;
    size_t det_config_size = 0;
    size_t custom_config_hdr_size = 0;
    uint8_t *msg_offset = NULL;
    st_hw_session_gcs_t *p_hw_ses = (st_hw_session_gcs_t *)p_ses;
    struct st_vendor_info *v_info = p_ses->vendor_uuid_info;
    uint32_t rt_bytes_one_sec;
    bool custom_config = false;

    if (NULL != p_hw_ses->nonpersistent_cal) {
        ALOGE("%s: nonpersistent cal data already cached! failing.", __func__);
        return -EINVAL;
    }

    /*
     * Set custom_config flag if mID and pID have been set in platform xml
     * file. If the flag is set, the opaque data will be wrapped in a header
     * containing mID, pID, etc.  Else, the opaque data is sent as it is,
     * and it is assumed that it is formatted from within.
     */
    if (p_hw_ses->gcs_usecase->params[CUSTOM_CONFIG].module_id &&
            p_hw_ses->gcs_usecase->params[CUSTOM_CONFIG].param_id &&
            rc_config->data_size)
        custom_config = true;

    /*
     * Calculate the size of non-persistent params payload, we include
     * DETECTION_ENGINE_CONFIG param only in-case smlib is present.
     * This must be 4 byte aligned so that the following custom_config
     * header and payload can be processed by the DSP.
     */
    if (v_info->smlib_handle) {
        det_config_size += sizeof(struct gcs_det_engine_config_param);
        det_config_size += custom_payload_size;
        det_config_size = ALIGN(det_config_size, 4);
    }

    /* Calculate custom config size, include DETECTION_ENGINE_CUSTOM_CONFIG param */
    if (custom_config)
        custom_config_hdr_size += sizeof(struct gcs_det_engine_custom_config_param);

    p_hw_ses->nonpersistent_cal_size = det_config_size + custom_config_hdr_size +
        rc_config->data_size;
    p_hw_ses->nonpersistent_cal_size = ALIGN(p_hw_ses->nonpersistent_cal_size, 4);

    if (p_hw_ses->nonpersistent_cal_size) {
        /* allocate memory for non-persitent params */
        p_hw_ses->nonpersistent_cal = calloc(1, p_hw_ses->nonpersistent_cal_size);
        if (!p_hw_ses->nonpersistent_cal) {
            ALOGE("%s: failed to alloc nonpersistent cal ", __func__);
            status = -ENOMEM;
            goto exit;
        }

        msg_offset = p_hw_ses->nonpersistent_cal;

        /* Set DETECTION_ENGINE_CONFIG params only if smlib is present */
        if (v_info->smlib_handle) {
            p_msg = (struct gcs_det_engine_config_param *)msg_offset;
            p_msg->cal_hdr.module_id =
                p_hw_ses->gcs_usecase->params[CONFIDENCE_LEVELS].module_id;
            p_msg->cal_hdr.instance_id =
                p_hw_ses->gcs_usecase->params[CONFIDENCE_LEVELS].instance_id;
            p_msg->cal_hdr.param_id =
                p_hw_ses->gcs_usecase->params[CONFIDENCE_LEVELS].param_id;

            ALOGV("%s: nonpersistent cal header MID %x, IID %x, PID %x", __func__,
                p_msg->cal_hdr.module_id, p_msg->cal_hdr.instance_id,
                p_msg->cal_hdr.param_id);

            /*
             * DETECTION_ENGINE_CONFIG payload consists of:
             *   mode
             *   custom_payload_size
             *   custom_payload
             */
            if (custom_config) {
                /*
                 * Opaque data is part of the DETECTION_ENGINE_CUSTOM_CONFIG param
                 * payload, not part of DETECTION_ENGINE_CONFIG param payload
                 */
                p_msg->cal_hdr.size = det_config_size -
                    sizeof(struct graphite_cal_header);
            } else {
                /* Opaque data is part of DETECTION_ENGINE_CONFIG param payload */
                p_msg->cal_hdr.size = p_hw_ses->nonpersistent_cal_size -
                    sizeof(struct graphite_cal_header);
            }

            /*
             * SVA doesn't support per keyword recogntion mode.
             * use the per soundmodel recognition mode
             */
            if (recognition_mode & RECOGNITION_MODE_VOICE_TRIGGER) {
                p_msg->mode = GCS_DETECTION_ENGINE_CONFIG_KW_DET_ENABLE;

                if (recognition_mode & RECOGNITION_MODE_USER_IDENTIFICATION)
                    p_msg->mode |= GCS_DETECTION_ENGINE_CONFIG_USER_VER_ENABLE;

                if (p_ses->stdev->detect_failure)
                    p_msg->mode |=
                        GCS_DETECTION_ENGINE_CONFIG_FAILURE_DET_ENABLE;

            } else {
                    ALOGE("%s: Unknown recognition mode %d", __func__,
                        recognition_mode);
                    status = -EINVAL;
                    goto err_free_nonpersist;
            }
            p_msg->custom_payload_sz = custom_payload_size;

            /* set custom payload */
            msg_offset += sizeof(struct gcs_det_engine_config_param);
            if (custom_payload)
                memcpy(msg_offset, (uint8_t *)custom_payload, custom_payload_size);
        }

        /*
         * Set the pointer such that DETECTION_ENGINE_CUSTOM_CONFIG param payload
         * starts at a 4 byte aligned address regardless of padded bytes.
         */
        msg_offset = p_hw_ses->nonpersistent_cal;
        msg_offset += det_config_size;

        /* Set DETECTION_ENGINE_CUSTOM_CONFIG params */
        if (custom_config) {

            cc_msg = (struct gcs_det_engine_custom_config_param *)msg_offset;
            cc_msg->cal_hdr.module_id =
                p_hw_ses->gcs_usecase->params[CUSTOM_CONFIG].module_id;
            cc_msg->cal_hdr.instance_id =
                p_hw_ses->gcs_usecase->params[CUSTOM_CONFIG].instance_id;
            cc_msg->cal_hdr.param_id =
                p_hw_ses->gcs_usecase->params[CUSTOM_CONFIG].param_id;

            ALOGV("%s: custom config header MID %x, IID %x, PID %x", __func__,
                cc_msg->cal_hdr.module_id, cc_msg->cal_hdr.instance_id,
                cc_msg->cal_hdr.param_id);

            cc_msg->cal_hdr.size = p_hw_ses->nonpersistent_cal_size -
                det_config_size - sizeof(struct graphite_cal_header);
            msg_offset += custom_config_hdr_size;
        }

        if (rc_config->data_size) {
            ALOGV("%s: copying opaque data, size = %d ", __func__,
                rc_config->data_size);
            memcpy(msg_offset, (unsigned char *)rc_config +
                rc_config->data_offset, rc_config->data_size);
        }
    }

    if (capture_requested) {
        /* allocate buffers used for LAB transfer*/
        rt_bytes_one_sec = (p_ses->config.rate * p_ses->config.channels *
            (pcm_format_to_bits(p_ses->config.format) >> 3));

        p_hw_ses->circ_buff_sz = ((v_info->kw_duration +
            v_info->client_capture_read_delay) * rt_bytes_one_sec) / 1000;

        ALOGV("%s: size of circ buff = %d", __func__, p_hw_ses->circ_buff_sz);

        p_hw_ses->circ_buff = calloc(1, p_hw_ses->circ_buff_sz);
        if (!p_hw_ses->circ_buff) {
            ALOGE("%s: failed to allocate circ buffer", __func__);
            status = -ENOMEM;
            goto err_free_nonpersist;
        }
        p_hw_ses->circ_buff_tail = p_hw_ses->circ_buff +
            p_hw_ses->circ_buff_sz - 1;

        /* reset buffering related data */
        p_hw_ses->exit_buffering = false;
        /*
         * see comment in header as to why rd_ptr and wr_ptr are
         * initialized this way
         * full_condition : wr_ptr == rd_ptr
         * empty_condition: wr_ptr == rd_ptr+1
         */
        p_hw_ses->rd_ptr = p_hw_ses->circ_buff;
        p_hw_ses->wr_ptr = p_hw_ses->rd_ptr + 1;
    }

    return status;

err_free_nonpersist:
    if(p_hw_ses->nonpersistent_cal)
        free(p_hw_ses->nonpersistent_cal);
exit:
    return status;
}

static int dereg_sm(st_hw_session_t *p_ses, bool capture_requested __unused)
{
    int status = 0;
    st_hw_session_gcs_t *p_gcs_ses = (st_hw_session_gcs_t *)p_ses;

    ALOGD("%s:[%d] calling gcs_unload_data with handle %d, loaded data handle %d",
        __func__, p_ses->sm_handle, p_gcs_ses->graph_handle,
        p_gcs_ses->loaded_sm_handle);
    status = gcs_unload_data_fn(p_gcs_ses->graph_handle,
        p_gcs_ses->loaded_sm_handle);
    if (status) {
        ALOGE("%s: gcs_unload_data failed with status %d", __func__, status);
    }

    ALOGD("%s:[%d] calling gcs_close on handle %d", __func__, p_ses->sm_handle,
        p_gcs_ses->graph_handle);
    status = gcs_close_fn(p_gcs_ses->graph_handle);
    if (status) {
        ALOGE("%s: gcs_close failed with status %d", __func__, status);
    }

    platform_free_gcs_usecase(p_ses->vendor_uuid_info, p_gcs_ses->gcs_usecase);

    /*
     * unload WDSP image, driver keeps reference count in-case
     * other sessions are still active
     */
    ALOGV("%s: writing echo 0 on sysfs node", __func__);
    if (gcs_data.sysfs_fd >= 0) {
        write(gcs_data.sysfs_fd, "0", 1);
    } else {
        ALOGE("%s: sysfs fd invalid %d ", __func__, gcs_data.sysfs_fd);
        status = -EIO;
    }
    ALOGV("%s: done writing echo 0 on sysfs node", __func__);

    return status;
}

static int dereg_sm_params(st_hw_session_t *p_ses)
{
    int status = 0;
    st_hw_session_gcs_t *p_gcs_ses = (st_hw_session_gcs_t *)p_ses;

    if (p_gcs_ses->nonpersistent_cal)
        free(p_gcs_ses->nonpersistent_cal);
    p_gcs_ses->nonpersistent_cal = NULL;
    p_gcs_ses->nonpersistent_cal_size = 0;

    /*
     * we can free without mutex lock here as at this point
     * no thread would be using circular buffer
     */
    if (p_gcs_ses->circ_buff) {
        free(p_gcs_ses->circ_buff);
        p_gcs_ses->circ_buff = NULL;
    }

    return status;
}

static int start(st_hw_session_t *p_ses)
{
    int status = 0;
    st_hw_session_gcs_t *p_gcs_ses = (st_hw_session_gcs_t *)p_ses;

    ST_DBG_DECLARE(FILE *debug_fd;);
    ST_DBG_FILE_OPEN_WR(debug_fd, "/data/misc/audio/", "inpersist_params", "bin", 1);
    ST_DBG_FILE_WRITE(debug_fd, p_gcs_ses->nonpersistent_cal,
        p_gcs_ses->nonpersistent_cal_size);
    ST_DBG_FILE_CLOSE(debug_fd);


    ALOGD("%s:[%d] calling gcs_enable with handle %d, non-persist cal %p, sz %zu",
        __func__, p_ses->sm_handle, p_gcs_ses->graph_handle,
        p_gcs_ses->nonpersistent_cal, p_gcs_ses->nonpersistent_cal_size);
    status = gcs_enable_fn(p_gcs_ses->graph_handle, p_gcs_ses->nonpersistent_cal,
        (uint32_t)p_gcs_ses->nonpersistent_cal_size);
    if (status)
        ALOGE("%s: gcs_enable failed with status %d", __func__, status);

    return status;
}

static int restart(st_hw_session_t *p_ses,
    unsigned int recognition_mode,
    bool capture_requested,
    unsigned int num_conf_levels,
    unsigned char *conf_levels,
    struct sound_trigger_recognition_config *rc_config __unused,
    sound_trigger_sound_model_type_t sm_type)
{
    int status = 0;
    /* TODO use RESTART capability exposed by Graphite */

    /* stop session */
    status = stop(p_ses);
    if (status) {
        ALOGE("%s: failed to stop err %d", __func__, status);
        goto exit;
    }

    status = dereg_sm_params(p_ses);
    if (status) {
        ALOGE("%s: failed to dereg_sm_params err %d", __func__, status);
        goto exit;
    }

    status = reg_sm_params(p_ses, recognition_mode, capture_requested,
        num_conf_levels, conf_levels, rc_config, sm_type);
    if (status) {
        ALOGE("%s: failed to reg_sm_params err %d", __func__, status);
        goto cleanup_reg_sm_params;
    }

    status = start(p_ses);
    if (status) {
        ALOGE("%s: failed to start err %d", __func__, status);
        goto cleanup_start;
    }

    return status;

cleanup_start:
    stop(p_ses);
cleanup_reg_sm_params:
    dereg_sm_params(p_ses);
exit:
    return status;
}

static int stop(st_hw_session_t *p_ses)
{
    int status = 0;
    st_hw_session_gcs_t *p_gcs_ses = (st_hw_session_gcs_t *)p_ses;

    ALOGD("%s:[%d] calling gcs_disable with handle %d", __func__,
        p_ses->sm_handle, p_gcs_ses->graph_handle);
    status = gcs_disable_fn(p_gcs_ses->graph_handle);
    if (status)
        ALOGE("%s: gcs_disable failed with status %d", __func__, status);

    return status;
}

static int stop_buffering(st_hw_session_t *p_ses __unused,
    bool capture_requested __unused)
{
    int status = 0;
    st_hw_session_gcs_t *p_hw_ses = (st_hw_session_gcs_t *)p_ses;
    struct timespec tspec;
    int ret;

    if (capture_requested) {
        /* signal stop of bufferinng */
        ALOGV("%s: acquirung circ buff lock", __func__);
        pthread_mutex_lock(&p_hw_ses->circ_buff_lock);
        p_hw_ses->exit_buffering = true;
        ALOGV("%s: signaling circ_buff_cond", __func__);
        pthread_cond_broadcast(&p_hw_ses->circ_buff_cond);

        /* wait for buffering thread to exit */
        while (p_hw_ses->lab_processing_active) {
            clock_gettime(CLOCK_REALTIME, &tspec);
            tspec.tv_sec += ST_READ_WAIT_TIME_OUT_SEC;
            ALOGV("%s: waiting on exit cond", __func__);
            ret = pthread_cond_timedwait(&(p_hw_ses->circ_buff_cond),
                &p_hw_ses->circ_buff_lock, &tspec);
            ALOGV("%s: done waiting on exit cond", __func__);
            if (ret) {
                ALOGE("%s: ERROR. wait timed out, ret %d", __func__, ret);
                ret = -EIO;
                break;
            }
        }
        pthread_mutex_unlock(&p_hw_ses->circ_buff_lock);
    }

    return status;
}

static int set_device(st_hw_session_t *p_ses, bool enable)
{
    char st_device_name[DEVICE_NAME_MAX_SIZE] = { 0 };
    int ref_cnt_idx = 0, ref_cnt = 0;
    int status = 0;
    st_device_t st_device;
    audio_devices_t capture_device;

    if (enable) {
        capture_device = platform_stdev_get_capture_device(p_ses->stdev->platform);
        st_device = platform_stdev_get_device(p_ses->stdev->platform,
            p_ses->vendor_uuid_info, capture_device, p_ses->exec_mode);

        if (platform_stdev_get_device_name(p_ses->stdev->platform, p_ses->exec_mode,
                st_device, st_device_name) < 0) {
            ALOGE("%s: Invalid sound trigger device returned", __func__);
            return -EINVAL;
        }

        ref_cnt_idx = (p_ses->exec_mode * ST_DEVICE_MAX) + st_device;
        pthread_mutex_lock(&p_ses->stdev->ref_cnt_lock);
        ref_cnt = ++(p_ses->stdev->dev_ref_cnt[ref_cnt_idx]);
        pthread_mutex_unlock(&p_ses->stdev->ref_cnt_lock);

        if (1 == ref_cnt) {
            status = platform_stdev_send_calibration(p_ses->stdev->platform,
                capture_device,
                p_ses->exec_mode,
                p_ses->vendor_uuid_info,
                ACDB_LSM_APP_TYPE_NO_TOPOLOGY,
                false, ST_DEVICE_CAL);

            if (!status) {
                ALOGD("%s: enable device (%x) = %s", __func__, st_device,
                      st_device_name);
                audio_route_apply_and_update_path(p_ses->stdev->audio_route,
                                                  st_device_name);
                p_ses->stdev->capture_device = capture_device;
            }
        }
        p_ses->st_device = st_device;
        p_ses->st_device_name = strdup(st_device_name);
    } else {
        if (!p_ses->st_device_name) {
            ALOGE("%s: Invalid sound trigger device name", __func__);
            return -EINVAL;
        }

        ref_cnt_idx = (p_ses->exec_mode * ST_DEVICE_MAX) + p_ses->st_device;
        pthread_mutex_lock(&p_ses->stdev->ref_cnt_lock);
        ref_cnt = p_ses->stdev->dev_ref_cnt[ref_cnt_idx];
        if (0 < ref_cnt) {
            ref_cnt = --(p_ses->stdev->dev_ref_cnt[ref_cnt_idx]);
            pthread_mutex_unlock(&p_ses->stdev->ref_cnt_lock);
        } else {
            ALOGV("%s: ref_cnt = %d", __func__, ref_cnt);
            pthread_mutex_unlock(&p_ses->stdev->ref_cnt_lock);
            return status;
        }

        if (0 == ref_cnt) {
            ALOGD("%s: disable device (%x) = %s", __func__, p_ses->st_device,
                  p_ses->st_device_name);
            audio_route_reset_and_update_path(p_ses->stdev->audio_route,
                                              p_ses->st_device_name);
        }
        free(p_ses->st_device_name);
    }
    return status;
}

static int disable_device(st_hw_session_t *p_ses)
{
    int status = 0, rc = 0;
    st_hw_session_gcs_t *p_gcs_ses = (st_hw_session_gcs_t *)p_ses;

    ALOGD("%s:[%d] calling gcs_disable_device with handle %d", __func__,
        p_ses->sm_handle, p_gcs_ses->graph_handle);
    status = gcs_disable_device_fn(p_gcs_ses->graph_handle);
    if (status) {
        ALOGE("%s: gcs_disable_device failed status %d", __func__, status);
        rc = status;
    }

    status = set_device(p_ses, false);
    if (status) {
        ALOGE("%s: set_device disable failed status %d", __func__, status);
        rc = status;
    }

    return rc;
}

static int enable_device(st_hw_session_t *p_ses)
{
    int status = 0, acdb_id = 0;
    st_hw_session_gcs_t *p_gcs_ses = (st_hw_session_gcs_t *)p_ses;

    status = set_device(p_ses, true);
    if (status) {
        ALOGE("%s: set_device enable failed status %d", __func__, status);
        return status;
    }

    acdb_id = platform_stdev_get_acdb_id(p_ses->st_device, p_ses->exec_mode);
    if (acdb_id < 0) {
        status = -EINVAL;
        goto exit;
    }

    ALOGD("%s:[%d] calling gcs_enable_device with handle %d, acdb_id %d",
          __func__, p_ses->sm_handle, p_gcs_ses->graph_handle, acdb_id);
    status = gcs_enable_device_fn(p_gcs_ses->graph_handle, acdb_id, NULL, 0);
    if (status) {
        ALOGE("%s: gcs_enable_device failed status %d", __func__, status);
        goto exit;
    }
    return status;

exit:
    set_device(p_ses, false);
    return status;
}

static void process_lab_capture(st_hw_session_t *p_ses)
{
    int status = 0;
    st_hw_session_gcs_t *p_hw_ses = (st_hw_session_gcs_t *)p_ses;
    struct gcs_data_cmd_t read_cmd;

    read_cmd.hdr.cmd_id = p_hw_ses->gcs_usecase->params[READ_REQ].param_id;
    read_cmd.hdr.module_id = p_hw_ses->gcs_usecase->params[READ_REQ].module_id;
    read_cmd.hdr.instance_id = p_hw_ses->gcs_usecase->params[READ_REQ].instance_id;
    read_cmd.hdr.reserved = 0;
    read_cmd.hdr.token = 0;
    read_cmd.hdr.size_in_bytes = sizeof(struct gcs_cmd_read_payload_t);
    /*
     * 64 is to reserve spave for CMI header as per driver team, not clear on
     * rational now need to check with them.
     */
    read_cmd.payload.read.size_in_bytes = stdev_cpe_pcm_config.period_size *
        SOUND_TRIGGER_BYTES_PER_SAMPLE + sizeof(struct graphite_data_cmdrsp_hdr)\
        + sizeof(struct gcs_cmd_readrsp_payload_t) + 64;

    ST_DBG_DECLARE(FILE *read_fp = NULL;);
    ST_DBG_FILE_OPEN_WR(read_fp, "/data/misc/audio/", "read_msg", "bin", 1);
    ST_DBG_FILE_WRITE(read_fp, (char *)&read_cmd, sizeof(struct graphite_data_cmd_hdr) +
        sizeof(struct gcs_cmd_read_payload_t));
    ST_DBG_FILE_CLOSE(read_fp);

    ALOGV("%s:[%d] read_cmd module_id 0x%x, instance_id 0x%x, cmd_id 0x%x, "
        "read_sz %d", __func__, p_ses->sm_handle, read_cmd.hdr.module_id,
        read_cmd.hdr.instance_id, read_cmd.hdr.cmd_id,
        read_cmd.payload.read.size_in_bytes);

    pthread_mutex_lock(&p_hw_ses->circ_buff_lock);
    /*
     * Check needed in-case the client called stop_buffering before we came
     * here
     */
    if (p_hw_ses->exit_buffering) {
        pthread_mutex_unlock(&p_hw_ses->circ_buff_lock);
        return;
    }

    ALOGD("%s: issuing gcs_start_buff_xfer", __func__);
    status = gcs_start_buff_xfer_fn(p_hw_ses->graph_handle, GCS_XFER_READ);
    if (status) {
        ALOGE("%s: failed to start buffer xfer, err %d", __func__, status);
        pthread_mutex_unlock(&p_hw_ses->circ_buff_lock);
        return;
    }

    p_hw_ses->lab_processing_active = true;

    ST_DBG_FILE_OPEN_WR(lab_fp_gcs, "/data/misc/audio/", "lab_gcs_to_sthal", "bin", 1);
    ST_DBG_FILE_OPEN_WR(lab_fp_client, "/data/misc/audio/", "lab_sthal_to_client", "bin", 1);

    p_hw_ses->read_rsp_cnt = GCS_CONCURRENT_READS_CNT;

    while (p_hw_ses->read_rsp_cnt && !p_hw_ses->exit_buffering) {

        pthread_mutex_unlock(&p_hw_ses->circ_buff_lock);
        ALOGVV("%s: issuing read cmd to gcs", __func__);
        status = gcs_send_data_cmd_fn(p_hw_ses->graph_handle, (int8_t *)&read_cmd,
                sizeof(struct graphite_data_cmd_hdr) + sizeof(struct gcs_cmd_read_payload_t));
        pthread_mutex_lock(&p_hw_ses->circ_buff_lock);

        /* break from loop in case read failed or exit requested */
        if (status) {
            ALOGE("%s: failed to send read cmd, err %d", __func__, status);
            break;
        }

        if (p_hw_ses->exit_buffering)
            break;

        /* wait for read response */
        if (--p_hw_ses->read_rsp_cnt == 0) {
            ALOGVV("%s: waiting on circ_buff_cond", __func__);
            pthread_cond_wait(&(p_hw_ses->circ_buff_cond),
                &(p_hw_ses->circ_buff_lock));
            ALOGVV("%s: ended wait on circ_buff_cond", __func__);
        }
    }
    ALOGV("%s: exited buffering loop ", __func__);
    pthread_mutex_unlock(&p_hw_ses->circ_buff_lock);

    /*
     * Below call will cause the DSP to flush any pending reads and
     * return a read rsp, this read rsp will be ignored in the callback
     * because the exit_buffering flag is set
     */
    ALOGD("%s:[%d] calling gcs_stop_buff_xfer ", __func__, p_ses->sm_handle);
    status = gcs_stop_buff_xfer_fn(p_hw_ses->graph_handle, GCS_XFER_READ);
    if (status)
        ALOGE("%s: failed to stop buffer xfer, err %d", __func__, status);

    ST_DBG_FILE_CLOSE(lab_fp_gcs);
    ST_DBG_FILE_CLOSE(lab_fp_client);

    /*
     * Signal back to thread calling stop_buffering that
     * buffering has exited
     */
    pthread_mutex_lock(&p_hw_ses->circ_buff_lock);
    p_hw_ses->lab_processing_active = false;
    pthread_cond_signal(&p_hw_ses->circ_buff_cond);
    pthread_mutex_unlock(&p_hw_ses->circ_buff_lock);
    return;
}

static int read_pcm(st_hw_session_t *p_ses __unused,
    unsigned char *client_buf,
    unsigned int bytes)
{
    int status = 0;
    st_hw_session_gcs_t *p_gcs_ses = (st_hw_session_gcs_t *)p_ses;
    struct timespec tspec;
    uint32_t filled_bytes = 0, copy_bytes = 0, bytes_to_tail = 0;
    int ret = 0;

    ALOGVV("%s: Enter, bytes requested %d", __func__, bytes);

    pthread_mutex_lock(&p_gcs_ses->circ_buff_lock);

    while (bytes > 0 && !p_gcs_ses->exit_buffering) {

        filled_bytes = circ_buff_filled(p_gcs_ses->rd_ptr,
            p_gcs_ses->wr_ptr, p_gcs_ses->circ_buff_sz);

        if (0 == filled_bytes) {
            /* not enough data, block and wait for more data */
            clock_gettime(CLOCK_REALTIME, &tspec);
            tspec.tv_sec += ST_READ_WAIT_TIME_OUT_SEC;
            ALOGVV("%s: waiting on cond, bytes=%d", __func__, bytes);
            ret = pthread_cond_timedwait(&(p_gcs_ses->circ_buff_cond),
                &(p_gcs_ses->circ_buff_lock), &tspec);
            ALOGVV("%s: done waiting on cond, bytes=%d", __func__, bytes);
            if (ret) {
                ALOGE("%s: ERROR. read wait timed out, ret %d", __func__, ret);
                ret = -EIO;
                break;
            }
            if (p_gcs_ses->exit_buffering) {
                ALOGV("%s: exiting buffering ", __func__);
                status = -EIO;
                break;
            }

            filled_bytes = circ_buff_filled(p_gcs_ses->rd_ptr,
                p_gcs_ses->wr_ptr, p_gcs_ses->circ_buff_sz);
        }

        if (filled_bytes > 0) {
            copy_bytes = MIN(filled_bytes, bytes);
            bytes_to_tail = p_gcs_ses->circ_buff_tail -
                p_gcs_ses->rd_ptr;

            if (copy_bytes > bytes_to_tail) {
                memcpy(client_buf, p_gcs_ses->rd_ptr + 1, bytes_to_tail);
                memcpy(client_buf + bytes_to_tail, p_gcs_ses->circ_buff,
                    copy_bytes - bytes_to_tail);
            } else {
                memcpy(client_buf, p_gcs_ses->rd_ptr + 1, copy_bytes);
            }
            ALOGVV("%s: copied %d bytes", __func__, copy_bytes);

            ST_DBG_FILE_WRITE(lab_fp_client, client_buf, copy_bytes);

            p_gcs_ses->rd_ptr = circ_buff_move_ptr_fwd(p_gcs_ses->circ_buff,
                p_gcs_ses->rd_ptr, copy_bytes, p_gcs_ses->circ_buff_sz);
            bytes -= copy_bytes;
            client_buf += copy_bytes;
        }
    }

    pthread_mutex_unlock(&p_gcs_ses->circ_buff_lock);

    ALOGVV("%s: Exit...", __func__);
    return status;
}

int st_hw_sess_gcs_init(st_hw_session_t *const p_ses,
    hw_ses_event_callback_t cb,
    void *cookie, st_exec_mode_t exec_mode,
    struct st_vendor_info *v_info,
    sound_model_handle_t sm_handle,
    sound_trigger_device_t *stdev)
{
    int status = 0;
    pthread_attr_t attr;
    st_hw_session_gcs_t *p_hw_ses = (st_hw_session_gcs_t *)p_ses;

    if (!v_info) {
        ALOGE("%s: received null v_info", __func__);
        return -EINVAL;
    }

    ALOGV("%s:[%d] Enter, exec_mode %d, callback %p, cookie %p",
        __func__, sm_handle, exec_mode, cb, cookie);

    p_ses->exec_mode = exec_mode;
    p_ses->callback_to_st_session = cb;
    p_ses->cookie = cookie;
    p_ses->vendor_uuid_info = v_info;
    p_ses->sm_handle = sm_handle;
    p_ses->fptrs = &fptrs_gcs;
    p_ses->stdev = stdev;
    p_hw_ses->nonpersistent_cal = NULL;
    p_hw_ses->detect_payload_size = 0;
    p_hw_ses->detection_signaled = false;
    p_hw_ses->exit_detection = false;
    p_hw_ses->exit_buffering = false;
    p_hw_ses->lab_processing_active = false;
    pthread_cond_init(&p_hw_ses->callback_thread_cond, NULL);
    pthread_mutex_init(&p_hw_ses->callback_thread_lock, NULL);

    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
    status = pthread_create(&p_hw_ses->callback_thread, &attr,
        callback_thread_loop, p_ses);
    if (status) {
        ALOGE("%s failed to create cb thread, error %d", __func__, status);
        goto error;
    }
    pthread_attr_destroy(&attr);

    /* initialize lab transfer objects */
    pthread_mutex_init(&(p_hw_ses->circ_buff_lock), NULL);
    pthread_cond_init(&(p_hw_ses->circ_buff_cond), NULL);

    return 0;

error:
    pthread_attr_destroy(&attr);
    pthread_cond_destroy(&p_hw_ses->callback_thread_cond);
    pthread_mutex_destroy(&p_hw_ses->callback_thread_lock);

    return status;
}

void st_hw_sess_gcs_deinit(st_hw_session_t *const p_ses)
{
    st_hw_session_gcs_t *p_hw_sess = (st_hw_session_gcs_t *)p_ses;

    /* signal detection callback thread to exit */
    pthread_mutex_lock(&p_hw_sess->callback_thread_lock);
    p_hw_sess->exit_detection = true;
    pthread_cond_signal(&p_hw_sess->callback_thread_cond);
    pthread_mutex_unlock(&p_hw_sess->callback_thread_lock);

    ALOGV("%s: waiting for callback thread to join", __func__);
    pthread_join(p_hw_sess->callback_thread, (void **)NULL);
    ALOGV("%s: callback thread exited", __func__);

    pthread_cond_destroy(&p_hw_sess->callback_thread_cond);
    pthread_mutex_destroy(&p_hw_sess->callback_thread_lock);
    pthread_mutex_destroy(&p_hw_sess->circ_buff_lock);
    pthread_cond_destroy(&p_hw_sess->circ_buff_cond);
}

int st_hw_gcs_init(void)
{
    int status = 0;

    /* load GCS library */
    gcs_data.lib_handle = dlopen(GCS_LIB, RTLD_NOW);
    if (!gcs_data.lib_handle) {
        ALOGE("%s: Unable to open %s, error %s", __func__, GCS_LIB,
            dlerror());
        status = -ENOENT;
        goto exit;
    }

    dlerror(); /* clear errors */
    DLSYM(gcs_data.lib_handle, gcs_init, status);
    if (status)
        goto exit;
    DLSYM(gcs_data.lib_handle, gcs_deinit, status);
    if (status)
        goto exit;
    DLSYM(gcs_data.lib_handle, gcs_open, status);
    if (status)
        goto exit;
    DLSYM(gcs_data.lib_handle, gcs_close, status);
    if (status)
        goto exit;
    DLSYM(gcs_data.lib_handle, gcs_load_data, status);
    if (status)
        goto exit;
    DLSYM(gcs_data.lib_handle, gcs_unload_data, status);
    if (status)
        goto exit;
    DLSYM(gcs_data.lib_handle, gcs_enable, status);
    if (status)
        goto exit;
    DLSYM(gcs_data.lib_handle, gcs_disable, status);
    if (status)
        goto exit;
    DLSYM(gcs_data.lib_handle, gcs_register_for_event, status);
    if (status)
        goto exit;
    DLSYM(gcs_data.lib_handle, gcs_register_data_cmd_handler, status);
    if (status)
        goto exit;
    DLSYM(gcs_data.lib_handle, gcs_start_buff_xfer, status);
    if (status)
        goto exit;
    DLSYM(gcs_data.lib_handle, gcs_stop_buff_xfer, status);
    if (status)
        goto exit;
    DLSYM(gcs_data.lib_handle, gcs_send_data_cmd, status);
    if (status)
        goto exit;
    DLSYM(gcs_data.lib_handle, gcs_enable_device, status);
    if (status)
        goto exit;
    DLSYM(gcs_data.lib_handle, gcs_disable_device, status);
    if (status)
        goto exit;

    /* initialize GCS */
    ALOGD("%s: calling gcs_init", __func__);
    status = gcs_init_fn();
    if (status) {
        ALOGE("%s: gcs_init failed with status %d", __func__, status);
        goto exit;
    }

    /* open handle used to load WDSP image */
    gcs_data.sysfs_fd = open(WDSP_SYSFS_NAME, O_WRONLY);
    if (gcs_data.sysfs_fd < 0) {
        ALOGW("%s: Failed to open %s with error: %s open will retried",
            __func__, WDSP_SYSFS_NAME, strerror(errno));
    }

    return status;

exit:
    if (gcs_data.lib_handle)
        dlclose(gcs_data.lib_handle);
    return status;
}

void st_hw_gcs_deinit(void)
{
    close(gcs_data.sysfs_fd);
    ALOGD("%s: calling gcs_deinit", __func__);
    gcs_deinit_fn();
    if (gcs_data.lib_handle)
        dlclose(gcs_data.lib_handle);
}

