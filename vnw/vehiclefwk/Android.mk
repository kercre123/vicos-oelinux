LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE_TAGS := optional

LOCAL_CERTIFICATE := platform
LOCAL_JAVA_LIBRARIES := iviaidl

LOCAL_SRC_FILES := $(call all-subdir-java-files)

LOCAL_CERTIFICATE := platform

LOCAL_MODULE := vehiclefwk

LOCAL_MODULE_PATH := $(TARGET_OUT_JAVA_LIBRARIES)

include $(BUILD_JAVA_LIBRARY)
