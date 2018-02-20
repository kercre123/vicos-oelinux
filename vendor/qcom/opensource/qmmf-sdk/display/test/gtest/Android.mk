LOCAL_PATH := $(call my-dir)

QMMF_SDK_TOP_SRCDIR := $(LOCAL_PATH)/../../..

include $(QMMF_SDK_TOP_SRCDIR)/build.mk

ifneq (,$(BUILD_QMMMF))

# Build display test application binary

include $(CLEAR_VARS)

include $(QMMF_SDK_TOP_SRCDIR)/common.mk

LOCAL_C_INCLUDES += $(TOP)/system/media/camera/include
LOCAL_C_INCLUDES += $(TARGET_OUT_HEADERS)/qcom/display
LOCAL_C_INCLUDES += $(TARGET_OUT_HEADERS)/qcom/display/sdm

LOCAL_SRC_FILES  := qmmf_display_gtest.cc
LOCAL_SRC_FILES  += qmmf_display_buffer_allocator.cc

LOCAL_SHARED_LIBRARIES += libqmmf_display_client libmemalloc

LOCAL_MODULE = qmmf_display_gtest

ifeq ($(LOCAL_VENDOR_MODULE),true)
LOCAL_VENDOR_MODULE := false
endif

include $(BUILD_NATIVE_TEST)

endif # BUILD_QMMMF
