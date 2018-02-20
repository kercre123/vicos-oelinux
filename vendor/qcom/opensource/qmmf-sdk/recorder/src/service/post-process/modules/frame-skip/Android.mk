LOCAL_PATH := $(call my-dir)

QMMF_SDK_TOP_SRCDIR := $(LOCAL_PATH)/../../../../../..

include $(QMMF_SDK_TOP_SRCDIR)/build.mk

ifneq (,$(BUILD_QMMMF))

# Build qmmf camera hal reprocess library
# libqmmf_postproc_frame_skip.so

include $(CLEAR_VARS)

include $(QMMF_SDK_TOP_SRCDIR)/common.mk

LOCAL_C_INCLUDES += $(TOP)/hardware/qcom/camera/QCamera2/HAL3
LOCAL_C_INCLUDES += $(TARGET_OUT_HEADERS)/mm-core/omxcore
LOCAL_C_INCLUDES += $(TOP)/hardware/qcom/media
LOCAL_C_INCLUDES += $(TOP)/external/jsoncpp/include
ifeq ($(IS_ANDROID_O_OR_ABOVE),true)
LOCAL_C_INCLUDES += $(TOP)/system/core/base/include
endif

LOCAL_SRC_FILES := qmmf_postproc_frame_skip.cc

LOCAL_SHARED_LIBRARIES += libcamera_client

LOCAL_STATIC_LIBRARIES += libjsoncpp

LOCAL_MODULE = libqmmf_postproc_frame_skip

include $(BUILD_SHARED_LIBRARY)

endif # BUILD_QMMMF
