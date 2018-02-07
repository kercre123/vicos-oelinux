LOCAL_PATH := $(call my-dir)

QMMF_SDK_TOP_SRCDIR := $(LOCAL_PATH)/../../..

include $(QMMF_SDK_TOP_SRCDIR)/build.mk

ifneq (,$(BUILD_QMMMF))

# Build display test application binary
include $(CLEAR_VARS)

include $(QMMF_SDK_TOP_SRCDIR)/common.mk

LOCAL_SRC_FILES := qmmf_display_test.cc

LOCAL_SHARED_LIBRARIES += libqmmf_display_client

LOCAL_MODULE = qmmf_display_test

include $(BUILD_EXECUTABLE)

endif # BUILD_QMMMF
