LOCAL_PATH := $(call my-dir)

QMMF_SDK_TOP_SRCDIR := $(LOCAL_PATH)/../../..

include $(QMMF_SDK_TOP_SRCDIR)/build.mk

ifneq (,$(BUILD_QMMMF))

# Build qmmf system client library
# libqmmf_system_client.so

include $(CLEAR_VARS)

include $(QMMF_SDK_TOP_SRCDIR)/common.mk

LOCAL_SRC_FILES := qmmf_system.cc
LOCAL_SRC_FILES += qmmf_system_client.cc

LOCAL_SHARED_LIBRARIES += libbinder
LOCAL_SHARED_LIBRARIES += libhardware

LOCAL_MODULE = libqmmf_system_client

include $(BUILD_SHARED_LIBRARY)

endif # BUILD_QMMMF
