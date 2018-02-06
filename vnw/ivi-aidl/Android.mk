LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)

LOCAL_MODULE_TAGS := optional
LOCAL_SRC_FILES += com/qualcomm/qti/ivi/aidl/IVehicleService.aidl

LOCAL_CERTIFICATE := platform
LOCAL_MODULE := iviaidl

LOCAL_MODULE_PATH := $(TARGET_OUT_JAVA_LIBRARIES)

include $(BUILD_JAVA_LIBRARY)
