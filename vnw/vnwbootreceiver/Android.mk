LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

LOCAL_MODULE_TAGS := optional

LOCAL_PROGUARD_ENABLED := disabled

LOCAL_CERTIFICATE := platform
#LOCAL_JAVA_LIBRARIES := iviaidl
#LOCAL_JAVA_LIBRARIES += ivifwk

LOCAL_SRC_FILES := $(call all-java-files-under, src)

LOCAL_CERTIFICATE := platform

LOCAL_PACKAGE_NAME := vnwbootrecvr

include $(BUILD_PACKAGE)

include $(call all-makefiles-under,$(LOCAL_PATH))

#################
include $(CLEAR_VARS)

LOCAL_MODULE := com.qti.can.xml

LOCAL_MODULE_TAGS :=optional

LOCAL_MODULE_CLASS :=ETC

LOCAL_MODULE_PATH := $(TARGET_OUT_ETC)/permissions

LOCAL_SRC_FILES := $(LOCAL_MODULE)

include $(BUILD_PREBUILT)
##################
