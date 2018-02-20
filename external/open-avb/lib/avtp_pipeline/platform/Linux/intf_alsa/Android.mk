LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_C_INCLUDES := $(LOCAL_PATH) \
        external/tinyalsa/include

LOCAL_EXPORT_C_INCLUDE_DIRS := $(LOCAL_C_INCLUDES)

LOCAL_SRC_FILES := openavb_intf_tinyalsa.c

LOCAL_MODULE := libopenavb_intf_tinyalsa

LOCAL_SHARED_LIBRARIES := libopenavb \
        libopenavb_map_aaf_audio \
        libopenavb_map_uncmp_audio \
        libtinyalsa

include $(BUILD_SHARED_LIBRARY)