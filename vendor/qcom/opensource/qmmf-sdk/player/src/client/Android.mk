LOCAL_PATH := $(call my-dir)

QMMF_SDK_TOP_SRCDIR := $(LOCAL_PATH)/../../..

include $(QMMF_SDK_TOP_SRCDIR)/build.mk

ifneq (,$(BUILD_QMMMF))

# Build qmmf player client library
# libqmmf_player_client.so

include $(CLEAR_VARS)

include $(QMMF_SDK_TOP_SRCDIR)/common.mk

LOCAL_C_INCLUDES += $(TOP)/system/media/camera/include

LOCAL_SRC_FILES := qmmf_player.cc
LOCAL_SRC_FILES += qmmf_player_client.cc

LOCAL_SHARED_LIBRARIES += libbinder

LOCAL_MODULE = libqmmf_player_client

include $(BUILD_SHARED_LIBRARY)

endif # BUILD_QMMMF
