LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_SRC_FILES := android_ve_jni.cpp
LOCAL_SHARED_LIBRARIES := \
    libandroid_runtime \
    libnativehelper \
    libutils \
    libbinder \
    libcutils \

LOCAL_C_INCLUDES += \
    frameworks/base/core/jni \
    frameworks/native/include \

LOCAL_MODULE:= libve_jni
LOCAL_MODULE_TAGS := optional
include $(BUILD_SHARED_LIBRARY)


include $(CLEAR_VARS)

LOCAL_MODULE_TAGS := optional

LOCAL_PROGUARD_ENABLED := disabled

LOCAL_SRC_FILES := $(call all-java-files-under, src)

LOCAL_CERTIFICATE := platform
LOCAL_JAVA_LIBRARIES := vehiclefwk
LOCAL_JAVA_LIBRARIES += distraction-service
LOCAL_JNI_SHARED_LIBRARIES := libve_jni

ifeq ($(call is-platform-sdk-version-at-least, 23),true)
LOCAL_STATIC_JAVA_LIBRARIES := libprotobuf-java-micro
else
LOCAL_STATIC_JAVA_LIBRARIES := libprotobuf-java-2.3.0-micro
endif

LOCAL_CERTIFICATE := platform
LOCAL_PACKAGE_NAME := VehicleExplorer

include $(BUILD_PACKAGE)

include $(call all-makefiles-under,$(LOCAL_PATH))
