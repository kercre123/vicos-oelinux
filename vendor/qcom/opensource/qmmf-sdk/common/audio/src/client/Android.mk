LOCAL_PATH := $(call my-dir)

QMMF_SDK_TOP_SRCDIR := $(LOCAL_PATH)/../../../..

include $(QMMF_SDK_TOP_SRCDIR)/build.mk

ifneq (,$(BUILD_QMMMF))

# Build qmmf audio client library

include $(CLEAR_VARS)

include $(QMMF_SDK_TOP_SRCDIR)/common.mk

LOCAL_SRC_FILES := qmmf_audio_endpoint.cc
LOCAL_SRC_FILES += qmmf_audio_endpoint_client.cc

LOCAL_SHARED_LIBRARIES += libbinder

LOCAL_MODULE = libqmmf_audio_client

include $(BUILD_SHARED_LIBRARY)

endif # BUILD_QMMMF
