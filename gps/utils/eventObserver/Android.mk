#/******************************************************************************
#*@file Android.mk
#* brief Rules for compiling native code associated with libulp library
#*  ******************************************************************************/
ifneq ($(BUILD_TINY_ANDROID),true)

LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)
ULP2_ROOT_DIR = ../
LOC_API_DIR = $(TOP)/hardware/qcom/gps/loc_api
INTERNAL_ROOT_DIR = $(TOP)/vendor/qcom/proprietary/gps-noship/internal

# This is the target being built.
LOCAL_MODULE:= libevent_observer

# All of the source files that we will compile.
LOCAL_SRC_FILES:= \
    EventObserver.cpp

# All of the shared libraries we link against.
LOCAL_SHARED_LIBRARIES := \
    libcutils \
    libutils \
    libgps.utils \
    liblog

LOCAL_C_INCLUDES += \
  $(TARGET_OUT_HEADERS)/gps.utils \
  $(TARGET_OUT_HEADERS)/libloc_core \
  $(LOCAL_PATH)/$(ULP2_ROOT_DIR)/inc \
  $(TOP)/vendor/qcom/proprietary/gps/framework/native/service \
  $(TARGET_OUT_HEADERS)/libloc_pla

LOCAL_CFLAGS+=$(GPS_FEATURES) \
    -D_ANDROID_ \

# Install additional non-MP headers
LOCAL_COPY_HEADERS_TO:= libevent_observer
LOCAL_COPY_HEADERS := \
  EventObserver.h


LOCAL_LDFLAGS += -Wl,--export-dynamic
LOCAL_MODULE_TAGS := optional

LOCAL_PRELINK_MODULE := false
LOCAL_MODULE_OWNER := qti
LOCAL_PROPRIETARY_MODULE := true

include $(BUILD_SHARED_LIBRARY)

endif # not BUILD_TINY_ANDROID



