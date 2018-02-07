LOCAL_PATH := $(call my-dir)

QMMF_SDK_TOP_SRCDIR := $(LOCAL_PATH)/../../..

include $(QMMF_SDK_TOP_SRCDIR)/build.mk

ifneq (,$(BUILD_QMMMF))

# Build qmmf recorder client library
# libqmmf_recorder_client.so

include $(CLEAR_VARS)

include $(QMMF_SDK_TOP_SRCDIR)/common.mk

LOCAL_C_INCLUDES += $(TOP)/system/media/camera/include
LOCAL_C_INCLUDES += $(TARGET_OUT_HEADERS)/mm-core/omxcore
LOCAL_C_INCLUDES += $(TOP)/hardware/qcom/media

LOCAL_SRC_FILES := qmmf_recorder.cc
LOCAL_SRC_FILES += qmmf_recorder_client.cc
LOCAL_SRC_FILES += qmmf_recorder_client_ion.cc
LOCAL_SRC_FILES += qmmf_recorder_extra_param.cc

LOCAL_SHARED_LIBRARIES += libcamera_metadata libcamera_client
LOCAL_SHARED_LIBRARIES += libqmmf_camera_adaptor libbinder

LOCAL_MODULE = libqmmf_recorder_client

include $(BUILD_SHARED_LIBRARY)

endif # BUILD_QMMMF
