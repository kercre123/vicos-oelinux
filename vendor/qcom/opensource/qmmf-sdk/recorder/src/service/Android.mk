LOCAL_PATH := $(call my-dir)

QMMF_SDK_TOP_SRCDIR := $(LOCAL_PATH)/../../..

include $(QMMF_SDK_TOP_SRCDIR)/build.mk

ifneq (,$(BUILD_QMMMF))

# Build qmmf recorder service library
# libqmmf_recorder_service.so

include $(CLEAR_VARS)

include $(QMMF_SDK_TOP_SRCDIR)/common.mk

LOCAL_CFLAGS += -DENABLE_360=1
LOCAL_C_INCLUDES += $(TOP)/hardware/qcom/camera/QCamera2/HAL3
LOCAL_C_INCLUDES += $(TARGET_OUT_HEADERS)/mm-core/omxcore
LOCAL_C_INCLUDES += $(TOP)/hardware/qcom/media
LOCAL_C_INCLUDES += $(TOP)/hardware/qcom/display

# reprocess-related includes
LOCAL_C_INCLUDES += $(TOP)/hardware/qcom/camera/QCamera2/stack/common \
LOCAL_C_INCLUDES += $(TOP)/hardware/qcom/camera/mm-image-codec/qomx_core \
LOCAL_C_INCLUDES += $(TOP)/hardware/qcom/camera/mm-image-codec/qexif \

LOCAL_SRC_FILES := qmmf_recorder_service.cc
LOCAL_SRC_FILES += qmmf_recorder_impl.cc
LOCAL_SRC_FILES += qmmf_recorder_ion.cc
LOCAL_SRC_FILES += qmmf_remote_cb.cc
LOCAL_SRC_FILES += qmmf_camera_source.cc
LOCAL_SRC_FILES += qmmf_camera_context.cc
LOCAL_SRC_FILES += qmmf_camera_rescaler.cc
LOCAL_SRC_FILES += qmmf_encoder_core.cc
LOCAL_SRC_FILES += qmmf_audio_source.cc
LOCAL_SRC_FILES += qmmf_audio_raw_track_source.cc
LOCAL_SRC_FILES += qmmf_audio_encoded_track_source.cc
LOCAL_SRC_FILES += qmmf_audio_encoder_core.cc
LOCAL_SRC_FILES += qmmf_multicamera_manager.cc
LOCAL_SRC_FILES += qmmf_jpeg_encoder.cc
LOCAL_SRC_FILES += qmmf_camera_jpeg.cc
LOCAL_SRC_FILES += qmmf_exif_generator.cc
LOCAL_SRC_FILES += qmmf_camera_reprocess_impl.cc
LOCAL_SRC_FILES += post-process/factory/qmmf_postproc_factory.cc
LOCAL_SRC_FILES += post-process/node/qmmf_postproc_node.cc
LOCAL_SRC_FILES += post-process/memory/qmmf_postproc_memory_pool.cc
LOCAL_SRC_FILES += post-process/pipe/qmmf_postproc_pipe.cc
LOCAL_SRC_FILES += post-process/common/qmmf_postproc_thread.cc

LOCAL_SHARED_LIBRARIES += libqmmf_postproc_algo libqmmf_jpeg
LOCAL_SHARED_LIBRARIES += libqmmf_camera_hal_jpeg
LOCAL_SHARED_LIBRARIES += libqmmf_camera_hal_reproc libqmmf_postproc_test
LOCAL_SHARED_LIBRARIES += libqmmf_recorder_client libqmmf_camera_adaptor
LOCAL_SHARED_LIBRARIES += libqmmf_codec_adaptor libqmmf_audio_client
LOCAL_SHARED_LIBRARIES += libqmmf_overlay libqmmf_display_client
LOCAL_SHARED_LIBRARIES += libcamera_client libbinder libhardware libfastcvopt libC2D2

LOCAL_MODULE = libqmmf_recorder_service

include $(BUILD_SHARED_LIBRARY)

include $(LOCAL_PATH)/post-process/modules/Android.mk

endif # BUILD_QMMMF
