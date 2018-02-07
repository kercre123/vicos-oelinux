LOCAL_PATH := $(call my-dir)

QMMF_SDK_TOP_SRCDIR := $(LOCAL_PATH)/../..

include $(QMMF_SDK_TOP_SRCDIR)/build.mk

ifneq (,$(BUILD_QMMMF))

# Build libqmmf_codec_adaptor.so

include $(CLEAR_VARS)

include $(QMMF_SDK_TOP_SRCDIR)/common.mk

LOCAL_C_INCLUDES += $(TARGET_OUT_HEADERS)/mm-core/omxcore
LOCAL_C_INCLUDES += $(TOP)/hardware/qcom/media
LOCAL_C_INCLUDES += $(TOP)/system/media/camera/include
ifneq ($(TARGET_USES_GRALLOC1), true)
    LOCAL_C_INCLUDES += $(TOP)/hardware/qcom/display/libgralloc
else
    LOCAL_C_INCLUDES += $(TOP)/hardware/qcom/display/libgralloc1
endif

LOCAL_SRC_FILES := src/qmmf_omx_client.cc
LOCAL_SRC_FILES += src/qmmf_avcodec.cc

LOCAL_MODULE = libqmmf_codec_adaptor

include $(BUILD_SHARED_LIBRARY)

endif # BUILD_QMMMF
