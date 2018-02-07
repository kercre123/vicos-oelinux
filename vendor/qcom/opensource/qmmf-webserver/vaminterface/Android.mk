LOCAL_PATH := $(call my-dir)

QMMF_WEBSERVER_TOP_SRCDIR := $(LOCAL_PATH)/..

include $(QMMF_WEBSERVER_TOP_SRCDIR)/build.mk

ifneq (,$(BUILD_QMMMF))

# Build libvam_interface

include $(CLEAR_VARS)

include $(QMMF_WEBSERVER_TOP_SRCDIR)/common.mk

LOCAL_C_INCLUDES += $(TOP)/system/media/camera/include

LOCAL_SRC_FILES = src/qmmf_vam_dummy_interface.cc

LOCAL_SHARED_LIBRARIES += libqmmf_recorder_client

LOCAL_MODULE = libqmmf_vam_interface

include $(BUILD_SHARED_LIBRARY)

endif # BUILD_QMMMF
