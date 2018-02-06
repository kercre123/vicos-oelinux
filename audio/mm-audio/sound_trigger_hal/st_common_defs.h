/* st_common_defs.h
 *
 *
 * Copyright (c) 2016 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */
#ifndef ST_COMMON_DEFS_H
#define ST_COMMON_DEFS_H

#include <stdbool.h>

/* #define VERY_VERBOSE_LOGGING */
#ifdef VERY_VERBOSE_LOGGING
#define ALOGVV ALOGV
#else
#define ALOGVV(a...) do { } while(0)
#endif

#define SOUND_TRIGGER_APE_BUFFER_DURATION_MS (1000)
#define SOUND_TRIGGER_PCM_BUFFER_DURATION_MS (160)

#define SOUND_TRIGGER_CPE_LAB_DRV_BUF_DURATION_MS (240)
#define SOUND_TRIGGER_CPE_PERIOD_COUNT (6)

#define ST_GRAPHITE_LAB_BUF_DURATION_MS (480)
#define ST_GRAPHITE_LAB_PERIOD_COUNT (6)

#define SOUND_TRIGGER_SAMPLING_RATE_16000 (16000)
#define SOUND_TRIGGER_SAMPLING_RATE_48000 (48000)
#define SOUND_TRIGGER_SAMPLING_RATE_384000 (384000)

#define SOUND_TRIGGER_CHANNEL_MODE_MONO (1)
#define SOUND_TRIGGER_CHANNEL_MODE_STEREO (2)
#define SOUND_TRIGGER_CHANNEL_MODE_QUAD (4)
#define SOUND_TRIGGER_CHANNEL_MODE_HEX (6)
#define SOUND_TRIGGER_CHANNEL_MODE_OCT (8)

#define SOUND_TRIGGER_BIT_WIDTH  (16)

#define ST_READ_WAIT_TIME_OUT_SEC (2)

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define SET_BIT(a, b) (a |= b)
#define CLEAR_BIT(a, b) (a &= ~(b))
#define CHECK_BIT(a, b) ((a) & (b))

#define SET_STATE(a, b) SET_BIT(a, b)
#define CLEAR_STATE(a, b) CLEAR_BIT(a, b)
#define CHECK_STATE(a, b) CHECK_BIT(a, b)

#define ALIGN(number, align) \
        ((number + align - 1) & ~(align - 1))
#define CALCULATE_PERIOD_SIZE(duration_ms, sample_rate, period_cnt, align) \
       (ALIGN(((sample_rate * duration_ms) /(period_cnt * 1000)), align))

/* fwk mode definitions */
#define SOUND_TRIGGER_EVENT_NON_TIME_STAMP_MODE (0)
#define SOUND_TRIGGER_EVENT_TIME_STAMP_MODE (1)

/* note as per C99 we cannot cast a function ptr to void* hence we have to
   take the ptr to function ptr and cast that to void* as below */
#define DLSYM(handle, name, err) \
do {\
    const char* error; \
    *(void**)&name##_fn = dlsym(handle, #name);\
    if ((error = dlerror())) {\
        ALOGE("%s: dlsym failed for %s error %s", __func__, #name, error);\
        err = -ENODEV;\
    }\
}while(0)\

typedef uint32_t audio_devices_t;

/* TODO: move to common defines */
typedef enum {
    ST_DEVICE_HW_NONE,
    ST_DEVICE_HW_APE,
    ST_DEVICE_HW_CPE,
    ST_DEVICE_HW_ARM
}st_hw_type_t;

typedef enum st_exec_mode {
    ST_EXEC_MODE_NONE = -1,
    ST_EXEC_MODE_ADSP,
    ST_EXEC_MODE_CPE,
    ST_EXEC_MODE_ARM,
    ST_EXEC_MODE_MAX
} st_exec_mode_t;

/* defines possible configuration modes for
   execution mode that can be selecet through
   config file */
typedef enum st_exec_mode_config {
   EXEC_MODE_CFG_APE,
   EXEC_MODE_CFG_CPE,
   EXEC_MODE_CFG_DYNAMIC,
   EXEC_MODE_CFG_ARM
} st_exec_mode_config_t;

#endif /* ST_COMMON_DEFS_H */
