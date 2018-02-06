ifneq ($(BUILD_TINY_ANDROID),true)

LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

# ---------------------------------------------------------------------------------
# IPC Message Queue Client Library
# ---------------------------------------------------------------------------------

LOCAL_MODULE_TAGS := optional

LOCAL_MODULE            := libloc_mq_client
LOCAL_CFLAGS              := $(COMMON_CFLAGS)
LOCAL_CFLAGS                += -fno-short-enums
LOCAL_CFLAGS                    += $(QC_LOC_FW_LOCAL_C_FLAGS)

LOCAL_PRELINK_MODULE        := false
LOCAL_SHARED_LIBRARIES        := libc libcutils libloc_pla
LOCAL_STATIC_LIBRARIES      :=   libloc_base_util

LOCAL_C_INCLUDES += \
  $(LOCAL_PATH) \
  $(TARGET_OUT_HEADERS)/gps.utils \
  $(TARGET_OUT_HEADERS)/libloc \
  $(TARGET_OUT_HEADERS)/libloc_pla

LOCAL_SRC_FILES:= \
  mq_client.cpp \
  mq_client_controller.cpp \
  IPCMessagingProxy.cpp

LOCAL_MODULE_OWNER := qti

include $(BUILD_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_COPY_HEADERS_TO := libloc/mq_client
LOCAL_COPY_HEADERS := \
    ../mq_client.h \
    ../mq_client_controller.h \
    ../IPCMessagingProxy.h
include $(BUILD_COPY_HEADERS)

endif # not BUILD_TINY_ANDROID
