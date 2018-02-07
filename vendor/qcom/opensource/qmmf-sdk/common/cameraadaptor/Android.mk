LOCAL_PATH := $(call my-dir)

QMMF_SDK_TOP_SRCDIR := $(LOCAL_PATH)/../..

include $(QMMF_SDK_TOP_SRCDIR)/build.mk

ifneq (,$(BUILD_QMMMF))

# Build libqmmf_camera_adaptor.so

include $(CLEAR_VARS)

include $(QMMF_SDK_TOP_SRCDIR)/common.mk

LOCAL_C_INCLUDES += $(TOP)/hardware/qcom/display
LOCAL_C_INCLUDES += $(TOP)/system/media/camera/include
LOCAL_C_INCLUDES += $(TOP)/hardware/qcom/camera/QCamera2/HAL3

LOCAL_SRC_FILES := qmmf_camera3_device_client.cc
LOCAL_SRC_FILES += qmmf_camera3_monitor.cc
LOCAL_SRC_FILES += qmmf_camera3_request_handler.cc
LOCAL_SRC_FILES += qmmf_camera3_prepare_handler.cc
LOCAL_SRC_FILES += qmmf_camera3_stream.cc
LOCAL_SRC_FILES += qmmf_camera3_thread.cc
LOCAL_SRC_FILES += qmmf_camera3_utils.cc

LOCAL_SHARED_LIBRARIES += libcamera_metadata libhardware libcamera_client

LOCAL_MODULE = libqmmf_camera_adaptor

include $(BUILD_SHARED_LIBRARY)

# Adaptor gtest app

include $(CLEAR_VARS)

include $(QMMF_SDK_TOP_SRCDIR)/common.mk

LOCAL_C_INCLUDES += $(TOP)/hardware/qcom/display
LOCAL_C_INCLUDES += $(TOP)/system/media/camera/include
LOCAL_C_INCLUDES += $(TOP)/hardware/qcom/camera/QCamera2/HAL3

LOCAL_SRC_FILES := gtest/qmmf_camera_adaptor_gtest.cc

LOCAL_SHARED_LIBRARIES += libqmmf_camera_adaptor libcamera_client

LOCAL_MODULE = qmmf_camera_adaptor_gtest

include $(BUILD_NATIVE_TEST)

# Dual adaptor gtest app

include $(CLEAR_VARS)

include $(QMMF_SDK_TOP_SRCDIR)/common.mk

LOCAL_C_INCLUDES += $(TOP)/hardware/qcom/display
LOCAL_C_INCLUDES += $(TOP)/system/media/camera/include

LOCAL_SRC_FILES := gtest/qmmf_dual_camera_adaptor_gtest.cc

LOCAL_SHARED_LIBRARIES += libqmmf_camera_adaptor libcamera_client

LOCAL_MODULE = qmmf_camera_dual_adaptor_gtest

include $(BUILD_NATIVE_TEST)

endif # BUILD_QMMMF
