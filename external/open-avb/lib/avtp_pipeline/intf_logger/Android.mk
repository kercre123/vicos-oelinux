LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_C_INCLUDES := $(LOCAL_PATH)
LOCAL_EXPORT_C_INCLUDE_DIRS := $(LOCAL_C_INCLUDES)

LOCAL_SRC_FILES := openavb_intf_logger.c

LOCAL_MODULE := libopenavb_intf_logger

LOCAL_SHARED_LIBRARIES := libopenavb

include $(BUILD_SHARED_LIBRARY)