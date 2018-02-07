LOCAL_PATH := $(call my-dir)

QMMF_SDK_TOP_SRCDIR := $(LOCAL_PATH)/..

include $(QMMF_SDK_TOP_SRCDIR)/build.mk

ifneq (,$(BUILD_QMMMF))

# Build libqmmf_av_codec.so

include $(CLEAR_VARS)

include $(QMMF_SDK_TOP_SRCDIR)/common.mk

LOCAL_C_INCLUDES += $(TOP)/system/media/camera/include
LOCAL_C_INCLUDES += $(TARGET_OUT_HEADERS)/mm-core/omxcore
LOCAL_C_INCLUDES += $(TOP)/hardware/qcom/media

LOCAL_SRC_FILES := src/qmmf_avcodec.cc

LOCAL_SHARED_LIBRARIES += libqmmf_codec_adaptor

LOCAL_MODULE = libqmmf_av_codec

include $(BUILD_SHARED_LIBRARY)

# AVCodec sample test app

include $(CLEAR_VARS)

include $(QMMF_SDK_TOP_SRCDIR)/common.mk

LOCAL_C_INCLUDES += $(TOP)/system/media/camera/include

LOCAL_SRC_FILES := test/sample/qmmf_avcodec_test.cc

LOCAL_SHARED_LIBRARIES += libqmmf_av_codec

LOCAL_MODULE = qmmf_av_codec_test

include $(BUILD_EXECUTABLE)

# AVCodec gtest app

include $(CLEAR_VARS)

include $(QMMF_SDK_TOP_SRCDIR)/common.mk

LOCAL_C_INCLUDES += $(TOP)/system/media/camera/include

LOCAL_SRC_FILES := test/gtest/qmmf_avcodec_gtest.cc

LOCAL_SHARED_LIBRARIES += libqmmf_av_codec

LOCAL_MODULE = qmmf_av_codec_gtest

include $(BUILD_NATIVE_TEST)

# Codec Adaptor sample audio decode test app

include $(CLEAR_VARS)

include $(QMMF_SDK_TOP_SRCDIR)/common.mk

LOCAL_C_INCLUDES += $(TOP)/system/media/camera/include

LOCAL_SRC_FILES := test/sample/qmmf_audio_decode_test.cc

LOCAL_SHARED_LIBRARIES += libqmmf_av_codec

LOCAL_MODULE = qmmf_av_codec_audio_decode_test

include $(BUILD_EXECUTABLE)

endif # BUILD_QMMMF
