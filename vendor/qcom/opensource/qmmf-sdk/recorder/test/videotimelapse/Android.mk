LOCAL_PATH := $(call my-dir)

QMMF_SDK_TOP_SRCDIR := $(LOCAL_PATH)/../../..

include $(QMMF_SDK_TOP_SRCDIR)/build.mk

ifneq (,$(BUILD_QMMMF))

# Build recorder test application binary

include $(CLEAR_VARS)

include $(QMMF_SDK_TOP_SRCDIR)/common.mk

LOCAL_C_INCLUDES += $(TOP)/system/media/camera/include
LOCAL_C_INCLUDES += $(TARGET_OUT_HEADERS)/mm-core/omxcore
LOCAL_C_INCLUDES += $(TOP)/hardware/qcom/media

LOCAL_SRC_FILES  := qmmf_video_time_lapse.cc
LOCAL_SRC_FILES  += qmmf_video_time_lapse_main.cc

LOCAL_SHARED_LIBRARIES += libqmmf_recorder_client libav_codec
LOCAL_SHARED_LIBRARIES += libcamera_client libcamera_metadata

LOCAL_MODULE = qmmf_video_recorder_timelapse

include $(BUILD_EXECUTABLE)

endif # BUILD_QMMMF
