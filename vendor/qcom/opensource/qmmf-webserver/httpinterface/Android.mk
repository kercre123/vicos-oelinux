LOCAL_PATH := $(call my-dir)

QMMF_WEBSERVER_TOP_SRCDIR := $(LOCAL_PATH)/..

include $(QMMF_WEBSERVER_TOP_SRCDIR)/build.mk

ifneq (,$(BUILD_QMMMF))

# Build libhttp_interface

include $(CLEAR_VARS)

include $(QMMF_WEBSERVER_TOP_SRCDIR)/common.mk

LOCAL_C_INCLUDES += $(TOP)/hardware/qcom/camera/QCamera2/HAL3
LOCAL_C_INCLUDES += $(TOP)/system/media/camera/include
LOCAL_C_INCLUDES += $(QMMF_SDK_TOP_SRCDIR)/common/avqueue
LOCAL_C_INCLUDES += $(TOP)/external/live555/liveMedia/include
LOCAL_C_INCLUDES += $(TOP)/external/live555/groupsock/include
LOCAL_C_INCLUDES += $(TOP)/external/live555/UsageEnvironment/include
LOCAL_C_INCLUDES += $(TOP)/external/live555/BasicUsageEnvironment/include
LOCAL_C_INCLUDES += $(TARGET_OUT_HEADERS)/mm-osal/include
LOCAL_C_INCLUDES += $(TARGET_OUT_HEADERS)/mm-mux

LOCAL_SRC_FILES := src/qmmf_http_interface.cc
LOCAL_SRC_FILES += src/qmmf_camera_configuration.cc

LOCAL_SHARED_LIBRARIES += libqmmf_live555rtsp_in
LOCAL_SHARED_LIBRARIES += libqmmf_mux_interface
LOCAL_SHARED_LIBRARIES += libqmmf_vam_interface
LOCAL_SHARED_LIBRARIES += libcamera_client
LOCAL_SHARED_LIBRARIES += libqmmf_recorder_client
LOCAL_SHARED_LIBRARIES += libqmmf_display_client

LOCAL_MODULE = libhttp_interface

include $(BUILD_SHARED_LIBRARY)

# gtest app

include $(CLEAR_VARS)

include $(QMMF_WEBSERVER_TOP_SRCDIR)/common.mk

LOCAL_SRC_FILES := gtest/qmmf_http_interface_gtest.cc

LOCAL_SHARED_LIBRARIES += libhttp_interface

LOCAL_MODULE = qmmf_http_interface_gtest

include $(BUILD_NATIVE_TEST)

endif # BUILD_QMMMF
