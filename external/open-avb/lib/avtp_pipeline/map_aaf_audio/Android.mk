LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_C_INCLUDES := $(LOCAL_PATH)
LOCAL_EXPORT_C_INCLUDE_DIRS := $(LOCAL_C_INCLUDES)

LOCAL_SRC_FILES := openavb_map_aaf_audio.c

LOCAL_MODULE := libopenavb_map_aaf_audio

LOCAL_SHARED_LIBRARIES := libopenavb libopenavb_map_uncmp_audio

include $(BUILD_SHARED_LIBRARY)