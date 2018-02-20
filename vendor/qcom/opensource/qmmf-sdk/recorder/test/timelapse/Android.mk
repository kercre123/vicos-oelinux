LOCAL_PATH := $(call my-dir)

QMMF_SDK_TOP_SRCDIR := $(LOCAL_PATH)/../../..

include $(QMMF_SDK_TOP_SRCDIR)/build.mk

ifneq (,$(BUILD_QMMMF))

# Build recorder test application binary

include $(CLEAR_VARS)

include $(QMMF_SDK_TOP_SRCDIR)/common.mk

LOCAL_C_INCLUDES += $(TOP)/system/media/camera/include
LOCAL_C_INCLUDES += $(TOP)/hardware/qcom/camera/QCamera2/HAL3
ifeq ($(IS_ANDROID_O_OR_ABOVE),true)
LOCAL_C_INCLUDES += $(TOP)/system/core/base/include
endif

LOCAL_SRC_FILES  := qmmf_time_lapse.cc
LOCAL_SRC_FILES  += qmmf_time_lapse_main.cc

LOCAL_SHARED_LIBRARIES += libqmmf_utils libqmmf_recorder_client
LOCAL_SHARED_LIBRARIES += libcamera_client
ifneq ($(DISABLE_DISPLAY),1)
LOCAL_SHARED_LIBRARIES += libqmmf_display_client
endif
LOCAL_SHARED_LIBRARIES += libcamera_metadata

LOCAL_MODULE = qmmf_recorder_timelapse

include $(BUILD_EXECUTABLE)

endif # BUILD_QMMMF
