ifneq ($(BUILD_TINY_ANDROID),true)

LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE        := libseccam_vendor
LOCAL_MODULE_TAGS    := optional

LOCAL_CFLAGS        := $(COMMON_CFLAGS) \
                        -fno-short-enums \
                        -g -fdiagnostics-show-option -Wno-format \
                        -Wno-missing-braces -Wno-missing-field-initializers \
                        -std=gnu++0x -fpermissive -Wno-unused-parameter

LOCAL_PRELINK_MODULE := false
LOCAL_MODULE_OWNER := qti
LOCAL_PROPRIETARY_MODULE := true

SECUREMSM_SHIP_PATH      := vendor/qcom/proprietary/securemsm

LOCAL_C_INCLUDES += \
  $(LOCAL_PATH) src\
  $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include \
  vendor/qcom/proprietary/securemsm/QSEEComAPI \
  $(TARGET_OUT_HEADERS)/common/inc \
  $(SECUREMSM_SHIP_PATH)/proxydaemon

LOCAL_SHARED_LIBRARIES := \
    libc \
    liblog \
    libutils \
    libbinder

LOCAL_SRC_FILES := \
    src/jni_vendor_if.cpp \
    src/tz_app_vendor_if.cpp \
    src/ion_vendor_if.cpp

include $(BUILD_SHARED_LIBRARY)

endif # not BUILD_TINY_ANDROID
