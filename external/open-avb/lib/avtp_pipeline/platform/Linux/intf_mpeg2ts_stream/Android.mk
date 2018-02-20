LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_C_INCLUDES := $(LOCAL_PATH)
LOCAL_EXPORT_C_INCLUDE_DIRS := $(LOCAL_C_INCLUDES)

LOCAL_SRC_FILES := openavb_intf_mpeg2ts_stream.c

LOCAL_MODULE := libopenavb_intf_mpeg2ts_stream

LOCAL_SHARED_LIBRARIES := libopenavb libopenavb_map_mpeg2ts libmpeg2ts

include $(BUILD_SHARED_LIBRARY)