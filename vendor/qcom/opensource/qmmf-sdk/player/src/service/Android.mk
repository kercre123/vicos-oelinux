LOCAL_PATH := $(call my-dir)

QMMF_SDK_TOP_SRCDIR := $(LOCAL_PATH)/../../..

include $(QMMF_SDK_TOP_SRCDIR)/build.mk

ifneq (,$(BUILD_QMMMF))

# Build qmmf player service library
# libqmmf_player_service.so

include $(CLEAR_VARS)

include $(QMMF_SDK_TOP_SRCDIR)/common.mk

LOCAL_C_INCLUDES += $(TOP)/system/media/camera/include
LOCAL_C_INCLUDES += $(TARGET_OUT_HEADERS)/mm-core/omxcore
LOCAL_C_INCLUDES += $(TOP)/hardware/qcom/media

LOCAL_SRC_FILES := qmmf_player_service.cc
LOCAL_SRC_FILES += qmmf_player_impl.cc
LOCAL_SRC_FILES += qmmf_player_common.cc
LOCAL_SRC_FILES += qmmf_player_remote_cb.cc
LOCAL_SRC_FILES += qmmf_player_audio_decoder_core.cc
LOCAL_SRC_FILES += qmmf_player_video_decoder_core.cc
LOCAL_SRC_FILES += qmmf_player_audio_sink.cc
LOCAL_SRC_FILES += qmmf_player_video_sink.cc

LOCAL_SHARED_LIBRARIES += libqmmf_player_client libqmmf_codec_adaptor
LOCAL_SHARED_LIBRARIES += libqmmf_audio_client libqmmf_display_client
LOCAL_SHARED_LIBRARIES += libbinder libfastcvopt

LOCAL_MODULE = libqmmf_player_service

include $(BUILD_SHARED_LIBRARY)

endif # BUILD_QMMMF
