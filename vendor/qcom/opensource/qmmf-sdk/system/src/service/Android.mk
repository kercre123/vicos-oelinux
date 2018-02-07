LOCAL_PATH := $(call my-dir)

QMMF_SDK_TOP_SRCDIR := $(LOCAL_PATH)/../../..

include $(QMMF_SDK_TOP_SRCDIR)/build.mk

ifneq (,$(BUILD_QMMMF))

# Build qmmf system service library
# libqmmf_system_service.so

include $(CLEAR_VARS)

include $(QMMF_SDK_TOP_SRCDIR)/common.mk

LOCAL_C_INCLUDES += $(TOP)/hardware/qcom/camera/QCamera2/HAL3
LOCAL_C_INCLUDES += $(TARGET_OUT_HEADERS)/mm-core/omxcore
LOCAL_C_INCLUDES += $(TOP)/hardware/qcom/media
LOCAL_C_INCLUDES += $(TOP)/hardware/qcom/display

LOCAL_SRC_FILES := qmmf_system_service.cc
LOCAL_SRC_FILES += qmmf_system_implementation.cc
LOCAL_SRC_FILES += qmmf_system_ion.cc
LOCAL_SRC_FILES += qmmf_system_devices.cc
LOCAL_SRC_FILES += qmmf_system_keytone.cc
LOCAL_SRC_FILES += qmmf_system_trigger.cc

LOCAL_SHARED_LIBRARIES += libqmmf_system_client
LOCAL_SHARED_LIBRARIES += libqmmf_audio_client
LOCAL_SHARED_LIBRARIES += libbinder
LOCAL_SHARED_LIBRARIES += libhardware

LOCAL_MODULE = libqmmf_system_service

include $(BUILD_SHARED_LIBRARY)

endif # BUILD_QMMMF
