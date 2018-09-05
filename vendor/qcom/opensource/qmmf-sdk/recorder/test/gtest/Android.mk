LOCAL_PATH := $(call my-dir)

QMMF_SDK_TOP_SRCDIR := $(LOCAL_PATH)/../../..

include $(QMMF_SDK_TOP_SRCDIR)/build.mk

ifneq (,$(BUILD_QMMMF))

# Build recorder test application binary

include $(CLEAR_VARS)

include $(QMMF_SDK_TOP_SRCDIR)/common.mk
LOCAL_CFLAGS += -DUSE_SKIA=1
LOCAL_CFLAGS += -DUSE_CAIRO=0

LOCAL_C_INCLUDES += $(TOP)/system/media/camera/include
LOCAL_C_INCLUDES += $(TOP)/hardware/qcom/camera/QCamera2/HAL3
LOCAL_C_INCLUDES += $(TOP)/external/skia/include/core/

LOCAL_SRC_FILES := qmmf_recorder_gtest.cc

LOCAL_SHARED_LIBRARIES += libqmmf_recorder_client libqmmf_av_queue
ifneq ($(DISABLE_DISPLAY),1)
LOCAL_SHARED_LIBRARIES += libqmmf_display_client
endif
LOCAL_SHARED_LIBRARIES += libcamera_client libskia

ifeq ($(USE_SURFACEFLINGER),1)
LOCAL_SHARED_LIBRARIES += libgui libandroid
endif

LOCAL_MODULE = qmmf_recorder_gtest

ifeq ($(LOCAL_VENDOR_MODULE),true)
LOCAL_VENDOR_MODULE := false
endif

include $(BUILD_NATIVE_TEST)

# Build recorder 360 camera test application binary

include $(CLEAR_VARS)

include $(QMMF_SDK_TOP_SRCDIR)/common.mk

LOCAL_C_INCLUDES += $(TOP)/system/media/camera/include
LOCAL_C_INCLUDES += $(TOP)/hardware/qcom/camera/QCamera2/HAL3

LOCAL_SRC_FILES := qmmf_recorder_360cam_gtest.cc

LOCAL_SHARED_LIBRARIES += libqmmf_recorder_client libqmmf_av_queue
LOCAL_SHARED_LIBRARIES += libcamera_client
LOCAL_SHARED_LIBRARIES += libqmmf_recorder_client
ifneq ($(DISABLE_DISPLAY),1)
LOCAL_SHARED_LIBRARIES += libqmmf_display_client
endif

LOCAL_MODULE = qmmf_recorder_360cam_gtest

ifeq ($(LOCAL_VENDOR_MODULE),true)
LOCAL_VENDOR_MODULE := false
endif

include $(BUILD_NATIVE_TEST)

endif # BUILD_QMMMF
