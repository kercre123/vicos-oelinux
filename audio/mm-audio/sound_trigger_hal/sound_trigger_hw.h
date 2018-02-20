/* sound_trigger_hw.h
 *
 * This library contains the API to load sound models with
 * DSP and start/stop detection of associated key phrases.
 *
 * Copyright (c) 2013-2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 *
 */

#ifndef SOUND_TRIGGER_HW_H
#define SOUND_TRIGGER_HW_H

#include <cutils/list.h>
#include <pthread.h>
#include <errno.h>
#include <hardware/sound_trigger.h>
#include <tinyalsa/asoundlib.h>
#include <audio_route/audio_route.h>

#include "sound_trigger_prop_intf.h"
#include "sound_trigger_platform.h"
#include "st_common_defs.h"
#include "st_session.h"

#if LINUX_ENABLED
#ifdef __LP64__
#define AUDIO_HAL_LIBRARY_PATH "/usr/lib64/audio.primary.%s.so"
#else
#define AUDIO_HAL_LIBRARY_PATH "/usr/lib/audio.primary.%s.so"
#endif
#else
#ifdef __LP64__
#define AUDIO_HAL_LIBRARY_PATH "/system/lib64/hw/audio.primary.%s.so"
#else
#define AUDIO_HAL_LIBRARY_PATH "/system/lib/hw/audio.primary.%s.so"
#endif
#endif

typedef enum {
    ST_DEVICE_SSR_APE = 0x1,
    ST_DEVICE_SSR_CPE = 0x2
}st_ssr_state_t;

#define USECASE_STRING_SIZE (50)

struct use_case_info {
    bool active;
    int pcm_id;
    int pcm_back_end;
    char use_case[USECASE_STRING_SIZE];
};

/* ADPCM decoder APIs */
typedef int (*g722_init_decoder_t)(void *decoder_inp);
typedef int (*g722_dec_get_total_byte_size_t)(int *total_byte_size);
typedef int (*g722_dec_process_t)(short *in_buf, short *out_buf,
                                   int in_bytes, int *out_samples,
                                   void *decoder_inp);

struct sound_trigger_device {
    struct sound_trigger_hw_device device;
    pthread_mutex_t lock;
    pthread_cond_t  cond;
    struct listnode sound_model_list;
    struct mixer *mixer;
    struct audio_route *audio_route;
    volatile int session_id;
    audio_devices_t capture_device;
    pthread_mutex_t ref_cnt_lock;
    int *dev_ref_cnt;
    audio_devices_t available_devices;
    pthread_t transitions_thread;
    pthread_cond_t transitions_cond;

    unsigned int tx_concurrency_active;
    unsigned int rx_concurrency_active;
    bool audio_concurrency_active;
    bool rx_concurrency_disabled;
    bool sw_mad;
    bool run_on_ape;
    int num_sessions_configured;
    unsigned int max_cpe_sessions;
    unsigned int avail_cpe_phrases;
    unsigned int avail_cpe_users;
    unsigned int max_ape_sessions;
    unsigned int avail_ape_phrases;
    unsigned int avail_ape_users;
    unsigned int max_wdsp_sessions;
    unsigned int max_arm_sessions;
    bool support_dev_switch;
    bool transit_to_adsp_on_playback;
    int transit_to_wdsp_wait_time;
    bool stop_transitions_thread_loop;

    unsigned int rx_conc_max_st_ses;
    struct use_case_info *ape_pcm_use_cases;
    struct use_case_info *cpe_pcm_use_cases;
    struct use_case_info *arm_pcm_use_cases;
    bool detect_failure;
    void *platform;
    st_hw_type_t hw_type;
    st_exec_mode_t exec_mode;
    st_exec_mode_t client_req_exec_mode;
    int client_req_exec_mode_status;

    void *audio_hal_handle;
    audio_hw_call_back_t audio_hal_cb;
    bool is_gcs;

    struct listnode vendor_uuid_list;
    void *smlib_handle;
    smlib_generate_sound_trigger_recognition_config_payload_t
                                    generate_st_recognition_config_payload;
    smlib_generate_sound_trigger_phrase_recognition_event_t
                                    generate_st_phrase_recognition_event;
    smlib_generate_sound_trigger_phrase_recognition_event_t
                                    generate_st_phrase_recognition_event_v2;


    void *adpcm_dec_lib_handle;
    g722_init_decoder_t adpcm_dec_init;
    g722_dec_get_total_byte_size_t adpcm_dec_get_scratch_size;
    g722_dec_process_t adpcm_dec_process;

    struct listnode adm_cfg_list;
    audio_devices_t ec_ref_dev;
};

typedef struct sound_trigger_device sound_trigger_device_t;

st_session_t* get_sound_trigger_session
(
    struct sound_trigger_device *stdev,
    sound_model_handle_t sound_model_handle
);

void update_hw_mad_exec_mode(st_exec_mode_t mode, st_profile_type_t profile_type);

/* Debug helper macros */
/* #define ST_DEBUG_DUMP */
#define ST_DEBUG_DUMP_LOCATION "/data/misc/audio"

#ifdef ST_DEBUG_DUMP
#define ST_DBG_DECLARE(args...) args

#define ST_DBG_FILE_OPEN_WR(fptr, fpath, fname, fextn, fcount) \
do {\
    char fptr_fn[100];\
\
    snprintf(fptr_fn, sizeof(fptr_fn), fpath"/"fname"_%d."fextn, fcount);\
    fptr = fopen(fptr_fn, "wb");\
    if (!fptr) { \
        ALOGE("%s: File open failed %s: %s", \
              __func__, fptr_fn, strerror(errno)); \
    } \
} while (0)

#define ST_DBG_FILE_CLOSE(fptr) \
do {\
    if (fptr) { fclose (fptr); }\
} while (0)

#define ST_DBG_FILE_WRITE(fptr, buf, buf_size) \
do {\
    if (fptr) {\
        size_t ret_bytes = fwrite(buf, 1, buf_size, fptr);\
        if (ret_bytes != (size_t)buf_size) {\
            ALOGE("%s: fwrite %zu < %zu", __func__,\
                  ret_bytes, (size_t)buf_size);\
        }\
        fflush(fptr);\
    }\
} while (0)
#else
#define ST_DBG_DECLARE(args...)
#define ST_DBG_FILE_OPEN_WR(fptr, path, fname, fextn, fcount) (0)
#define ST_DBG_FILE_CLOSE(fptr) (0)
#define ST_DBG_FILE_WRITE(fptr, buf, buf_size) (0)
#endif

#endif /* SOUND_TRIGGER_HW_H */
