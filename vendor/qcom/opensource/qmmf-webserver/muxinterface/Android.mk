LOCAL_PATH := $(call my-dir)

QMMF_WEBSERVER_TOP_SRCDIR := $(LOCAL_PATH)/..

include $(QMMF_WEBSERVER_TOP_SRCDIR)/build.mk

ifneq (,$(BUILD_QMMMF))

# Build libhmux_interface

include $(CLEAR_VARS)

include $(QMMF_WEBSERVER_TOP_SRCDIR)/common.mk

LOCAL_C_INCLUDES += $(TOP)/system/media/camera/include
LOCAL_C_INCLUDES += $(TARGET_OUT_HEADERS)/mm-osal/include
LOCAL_C_INCLUDES += $(TARGET_OUT_HEADERS)/mm-mux

LOCAL_SRC_FILES := src/qmmf_mux_interface.cc

LOCAL_SHARED_LIBRARIES += libFileMux libqmmf_av_codec

LOCAL_MODULE = libqmmf_mux_interface

include $(BUILD_SHARED_LIBRARY)

endif # BUILD_QMMMF
