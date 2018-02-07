LOCAL_PATH := $(call my-dir)

QMMF_SDK_TOP_SRCDIR := $(LOCAL_PATH)/../../..

include $(QMMF_SDK_TOP_SRCDIR)/build.mk

ifneq (,$(BUILD_QMMMF))

# Build qmmf display service library
# libqmmf_display_service.so

include $(CLEAR_VARS)

include $(QMMF_SDK_TOP_SRCDIR)/common.mk

LOCAL_C_INCLUDES += $(TOP)/hardware/qcom/display
LOCAL_C_INCLUDES += $(TOP)/hardware/qcom/display/sdm/include
LOCAL_C_INCLUDES += $(TOP)/system/core/libsync/include
LOCAL_C_INCLUDES += $(TARGET_OUT_HEADERS)/qcom/display

LOCAL_SRC_FILES := qmmf_display_service.cc
LOCAL_SRC_FILES += qmmf_display_impl.cc
LOCAL_SRC_FILES += qmmf_remote_cb.cc
LOCAL_SRC_FILES += qmmf_display_sdm_buffer_allocator.cc
LOCAL_SRC_FILES += qmmf_display_sdm_debugger.cc
LOCAL_SRC_FILES += qmmf_display_sdm_buffer_sync_handler.cc

LOCAL_SHARED_LIBRARIES += libqmmf_display_client libmemalloc libbinder
LOCAL_SHARED_LIBRARIES += libsdmcore

LOCAL_MODULE = libqmmf_display_service

include $(BUILD_SHARED_LIBRARY)

endif # BUILD_QMMMF
