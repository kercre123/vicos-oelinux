#
# Copy vnwconf-SAMPLE, vnwmappings and distraction-settings to /system/etc/
#

LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)
LOCAL_MODULE:= vnwconf.xml
LOCAL_MODULE_CLASS := EXECUTABLES
LOCAL_SRC_FILES := vnwconf-SAMPLE.xml
LOCAL_MODULE_TAGS := optional
LOCAL_PROPRIETARY_MODULE := true
LOCAL_MODULE_OWNER := qti
LOCAL_MODULE_PATH := $(TARGET_ROOT_OUT)
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_MODULE:= vnwmappings.xml
LOCAL_MODULE_CLASS := EXECUTABLES
LOCAL_SRC_FILES := vnwmappings.xml
LOCAL_MODULE_TAGS := optional
LOCAL_PROPRIETARY_MODULE := true
LOCAL_MODULE_OWNER := qti
LOCAL_MODULE_PATH := $(TARGET_OUT_ETC)/vnw
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_MODULE:= distraction-settings.xml
LOCAL_MODULE_CLASS := EXECUTABLES
LOCAL_SRC_FILES := distraction-settings.xml
LOCAL_MODULE_TAGS := optional
LOCAL_PROPRIETARY_MODULE := true
LOCAL_MODULE_OWNER := qti
LOCAL_MODULE_PATH := $(TARGET_OUT_ETC)/vnw
include $(BUILD_PREBUILT)