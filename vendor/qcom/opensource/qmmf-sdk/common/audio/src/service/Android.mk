LOCAL_PATH := $(call my-dir)

QMMF_SDK_TOP_SRCDIR := $(LOCAL_PATH)/../../../..

include $(QMMF_SDK_TOP_SRCDIR)/build.mk

ifneq (,$(BUILD_QMMMF))

# Build qmmf audio service library
include $(CLEAR_VARS)

include $(QMMF_SDK_TOP_SRCDIR)/common.mk

LOCAL_SRC_FILES := qmmf_audio_service.cc
LOCAL_SRC_FILES += qmmf_audio_ion.cc
LOCAL_SRC_FILES += qmmf_audio_frontend.cc
LOCAL_SRC_FILES += qmmf_audio_backend_source.cc
LOCAL_SRC_FILES += qmmf_audio_backend_sink.cc

LOCAL_SHARED_LIBRARIES += libhardware libbinder libqmmf_audio_client libqahw

LOCAL_MODULE = libqmmf_audio_service

include $(BUILD_SHARED_LIBRARY)

endif # BUILD_QMMMF
