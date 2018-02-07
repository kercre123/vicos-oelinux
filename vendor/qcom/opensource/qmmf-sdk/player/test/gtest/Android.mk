LOCAL_PATH := $(call my-dir)

QMMF_SDK_TOP_SRCDIR := $(LOCAL_PATH)/../../..

include $(QMMF_SDK_TOP_SRCDIR)/build.mk

ifneq (,$(BUILD_QMMMF))

# Build player test application binary
include $(CLEAR_VARS)

include $(QMMF_SDK_TOP_SRCDIR)/common.mk

LOCAL_SRC_FILES := qmmf_player_gtest.cc
LOCAL_SRC_FILES += qmmf_player_parser.cc

LOCAL_SHARED_LIBRARIES += libqmmf_player_client

LOCAL_MODULE = qmmf_player_gtest

include $(BUILD_NATIVE_TEST)

endif # BUILD_QMMMF
