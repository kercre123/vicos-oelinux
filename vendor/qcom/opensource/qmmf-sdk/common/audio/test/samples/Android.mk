LOCAL_PATH := $(call my-dir)

QMMF_SDK_TOP_SRCDIR := $(LOCAL_PATH)/../../../..

include $(QMMF_SDK_TOP_SRCDIR)/build.mk

ifneq (,$(BUILD_QMMMF))

# Build audio test application binary
include $(CLEAR_VARS)

include $(QMMF_SDK_TOP_SRCDIR)/common.mk

ifeq ($(IS_ANDROID_O_OR_ABOVE),true)
LOCAL_C_INCLUDES += $(TOP)/system/core/base/include
endif

LOCAL_SRC_FILES := qmmf_audio_test.cc
LOCAL_SRC_FILES += qmmf_audio_test_ion.cc
LOCAL_SRC_FILES += qmmf_audio_test_wav.cc

LOCAL_SHARED_LIBRARIES += libqmmf_audio_client

LOCAL_MODULE = qmmf_audio_test

include $(BUILD_EXECUTABLE)

endif # BUILD_QMMMF
