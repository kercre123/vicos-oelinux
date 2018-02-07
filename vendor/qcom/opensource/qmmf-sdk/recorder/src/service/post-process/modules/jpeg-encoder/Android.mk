LOCAL_PATH := $(call my-dir)

QMMF_SDK_TOP_SRCDIR := $(LOCAL_PATH)/../../../../../..

include $(QMMF_SDK_TOP_SRCDIR)/build.mk

ifneq (,$(BUILD_QMMMF))

# Build qmmf camera hal reprocess library
# libqmmf_jpeg.so

include $(CLEAR_VARS)

include $(QMMF_SDK_TOP_SRCDIR)/common.mk

LOCAL_C_INCLUDES += $(TOP)/hardware/qcom/camera/QCamera2/HAL3
LOCAL_C_INCLUDES += $(TARGET_OUT_HEADERS)/mm-core/omxcore
LOCAL_C_INCLUDES += $(TOP)/hardware/qcom/media
LOCAL_C_INCLUDES += $(TOP)/hardware/qcom/display
LOCAL_C_INCLUDES += $(TOP)/hardware/qcom/camera/QCamera2/stack/common
LOCAL_C_INCLUDES += $(TOP)/hardware/qcom/camera/mm-image-codec/qexif
LOCAL_C_INCLUDES += $(TOP)/hardware/qcom/camera/mm-image-codec/qomx_core

LOCAL_SRC_FILES := qmmf_jpeg_core.cc
LOCAL_SRC_FILES += qmmf_jpeg.cc

LOCAL_SHARED_LIBRARIES += libcamera_client libmmjpeg_interface

LOCAL_MODULE = libqmmf_jpeg

include $(BUILD_SHARED_LIBRARY)

endif # BUILD_QMMMF
