ifneq ($(BUILD_TINY_ANDROID),true)

LOCAL_PATH := $(call my-dir)

# ---------------------------------------------------------------------------------
# Base Utility Library
# ---------------------------------------------------------------------------------
include $(CLEAR_VARS)

LOCAL_MODULE_TAGS := optional

LOCAL_MODULE := libloc_base_util
LOCAL_CFLAGS := $(COMMON_CFLAGS)
LOCAL_CFLAGS += -fno-short-enums
ifeq (, $(filter aarch64 arm64, $(TARGET_ARCH)))
LOCAL_CFLAGS += -DANDROID_32_BIT_PTR_SUPPORT
endif

LOCAL_C_INCLUDES += \
  $(LOCAL_PATH) \
  external/boringssl/src/include \
  external/sqlite/dist \
  $(TARGET_OUT_HEADERS)/libloc

LOCAL_SRC_FILES:= \
  config_file.cpp \
  log.cpp \
  memorystream.cpp \
  nvparam_mgr.cpp \
  postcard.cpp \
  sync.cpp \
  string_routines.cpp \
  time_routines.cpp \
  fault_tolerant_file.cpp

LOCAL_CFLAGS += $(QC_LOC_FW_LOCAL_C_FLAGS)

LOCAL_MODULE_OWNER := qti

include $(BUILD_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_COPY_HEADERS_TO := libloc/base_util
LOCAL_COPY_HEADERS := \
    ../config_file.h \
    ../log.h \
    ../postcard.h \
    ../sync.h \
    ../queue.h \
    ../list.h \
    ../memorystream.h \
    ../time_routines.h \
    ../vector.h \
    ../fault_tolerant_file.h \
    ../array.h \
    ../string_routines.h \
    ../nvparam_mgr.h

include $(BUILD_COPY_HEADERS)

endif # not BUILD_TINY_ANDROID
