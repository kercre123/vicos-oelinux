LOCAL_PATH := $(call my-dir)

QMMF_SDK_TOP_SRCDIR := $(LOCAL_PATH)/../..

include $(QMMF_SDK_TOP_SRCDIR)/build.mk

ifneq (,$(BUILD_QMMMF))

# Build libqmmf_overlay.so

include $(CLEAR_VARS)

include $(QMMF_SDK_TOP_SRCDIR)/common.mk

LOCAL_CFLAGS += -DUSE_SKIA=1
LOCAL_CFLAGS += -DUSE_CAIRO=0

LOCAL_C_INCLUDES += $(TOP)/external/skia/include/core/

LOCAL_SRC_FILES := qmmf_overlay.cc

LOCAL_SHARED_LIBRARIES += libC2D2 libskia

LOCAL_MODULE = libqmmf_overlay

include $(BUILD_SHARED_LIBRARY)

# Codec Adaptor sample test app

include $(CLEAR_VARS)

include $(QMMF_SDK_TOP_SRCDIR)/common.mk

LOCAL_SRC_FILES := test/qmmf_overlay_test.cc

LOCAL_SHARED_LIBRARIES += libqmmf_overlay

LOCAL_MODULE = qmmf_overlay_test

include $(BUILD_EXECUTABLE)

endif # BUILD_QMMMF
