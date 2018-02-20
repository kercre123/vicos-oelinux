LOCAL_PATH := $(call my-dir)

QMMF_SDK_TOP_SRCDIR := $(LOCAL_PATH)/../..

include $(QMMF_SDK_TOP_SRCDIR)/build.mk

ifneq (,$(BUILD_QMMMF))

# Build libqmmf_camera_adaptor.so

include $(CLEAR_VARS)

include $(QMMF_SDK_TOP_SRCDIR)/common.mk

LOCAL_C_INCLUDES += $(TOP)/system/media/camera/include
LOCAL_C_INCLUDES += $(TOP)/hardware/qcom/camera/QCamera2/HAL3
ifeq ($(IS_ANDROID_O_OR_ABOVE),true)
LOCAL_C_INCLUDES += $(TOP)/system/core/base/include
endif
ifeq ($(TARGET_USES_GRALLOC1),true)
LOCAL_C_INCLUDES += $(TOP)/hardware/qcom/display
LOCAL_C_INCLUDES += $(TOP)/system/core/libgrallocusage/include
endif

LOCAL_SRC_FILES := qmmf_camera3_device_client.cc
LOCAL_SRC_FILES += qmmf_camera3_monitor.cc
LOCAL_SRC_FILES += qmmf_camera3_request_handler.cc
LOCAL_SRC_FILES += qmmf_camera3_prepare_handler.cc
LOCAL_SRC_FILES += qmmf_camera3_stream.cc
LOCAL_SRC_FILES += qmmf_camera3_thread.cc
LOCAL_SRC_FILES += qmmf_camera3_utils.cc
LOCAL_SRC_FILES += qmmf_camera3_smooth_zoom.cc

LOCAL_SHARED_LIBRARIES += libcamera_metadata libhardware libcamera_client
ifeq ($(TARGET_USES_GRALLOC1), true)
LOCAL_STATIC_LIBRARIES += libgrallocusage
endif

LOCAL_MODULE = libqmmf_camera_adaptor

include $(BUILD_SHARED_LIBRARY)

# Adaptor gtest app

include $(CLEAR_VARS)

include $(QMMF_SDK_TOP_SRCDIR)/common.mk

ifeq ($(TARGET_USES_GRALLOC1), true)
LOCAL_C_INCLUDES += $(TOP)/hardware/qcom/display
endif
LOCAL_C_INCLUDES += $(TOP)/system/media/camera/include
LOCAL_C_INCLUDES += $(TOP)/hardware/qcom/camera/QCamera2/HAL3

LOCAL_SRC_FILES := gtest/qmmf_camera_adaptor_gtest.cc

LOCAL_SHARED_LIBRARIES += libqmmf_camera_adaptor libcamera_client

LOCAL_MODULE = qmmf_camera_adaptor_gtest

ifeq ($(LOCAL_VENDOR_MODULE),true)
LOCAL_VENDOR_MODULE := false
endif

include $(BUILD_NATIVE_TEST)

# Dual adaptor gtest app

include $(CLEAR_VARS)

include $(QMMF_SDK_TOP_SRCDIR)/common.mk

ifeq ($(TARGET_USES_GRALLOC1), true)
LOCAL_C_INCLUDES += $(TOP)/hardware/qcom/display
endif
LOCAL_C_INCLUDES += $(TOP)/system/media/camera/include

LOCAL_SRC_FILES := gtest/qmmf_dual_camera_adaptor_gtest.cc

LOCAL_SHARED_LIBRARIES += libqmmf_camera_adaptor libcamera_client

LOCAL_MODULE = qmmf_camera_dual_adaptor_gtest

ifeq ($(LOCAL_VENDOR_MODULE),true)
LOCAL_VENDOR_MODULE := false
endif

include $(BUILD_NATIVE_TEST)

endif # BUILD_QMMMF
