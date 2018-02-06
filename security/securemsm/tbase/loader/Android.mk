# =============================================================================

LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_C_INCLUDES += $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include \
                    vendor/qcom/proprietary/securemsm/QSEEComAPI \
                    $(TARGET_OUT_HEADERS)/common/inc

LOCAL_ADDITIONAL_DEPENDENCIES := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr

LOCAL_SHARED_LIBRARIES += \
        libcutils \
        libutils \
        libQSEEComAPI \

LOCAL_CFLAGS += -DLOG_TAG=\"TbaseLoader\" \
                $(QSEECOM_CFLAGS)

LOCAL_SRC_FILES += loadtbase.c

LOCAL_MODULE := tbaseLoader
LOCAL_MODULE_TAGS := optional
LOCAL_PRELINK_MODULE := false
LOCAL_MODULE_OWNER := qti
include $(BUILD_EXECUTABLE)
