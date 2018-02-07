LOCAL_PATH := $(call my-dir)

QMMF_SDK_TOP_SRCDIR := $(LOCAL_PATH)/../../..

include $(QMMF_SDK_TOP_SRCDIR)/build.mk

ifneq (,$(BUILD_QMMMF))

# Build libqmmf_demuxer.so
include $(CLEAR_VARS)

include $(QMMF_SDK_TOP_SRCDIR)/common.mk

LOCAL_C_INCLUDES += $(TARGET_OUT_HEADERS)/mm-parser/include
LOCAL_C_INCLUDES += $(TARGET_OUT_HEADERS)/mm-osal/include
LOCAL_C_INCLUDES += $(TARGET_OUT_HEADERS)/common/inc

LOCAL_SRC_FILES := qmmf_demuxer_sourceport.cc
LOCAL_SRC_FILES += qmmf_demuxer_intf.cc

LOCAL_SHARED_LIBRARIES += libqmmf_codec_adaptor libmmosal libmmparser_lite

LOCAL_MODULE = libqmmf_demuxer

include $(BUILD_SHARED_LIBRARY)

endif # BUILD_QMMMF
