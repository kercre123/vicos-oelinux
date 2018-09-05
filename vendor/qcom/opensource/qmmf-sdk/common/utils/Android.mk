LOCAL_PATH := $(call my-dir)

QMMF_SDK_TOP_SRCDIR := $(LOCAL_PATH)/../..

include $(QMMF_SDK_TOP_SRCDIR)/build.mk

ifneq (,$(BUILD_QMMMF))

# Build libqmmf_utils.so

include $(CLEAR_VARS)

include $(QMMF_SDK_TOP_SRCDIR)/common.mk

LOCAL_SRC_FILES := qmmf_condition.cc

LOCAL_SHARED_LIBRARIES += libc++

LOCAL_MODULE = libqmmf_utils

include $(BUILD_SHARED_LIBRARY)

endif # BUILD_QMMMF
