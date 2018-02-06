ifeq ($(call is-board-platform-in-list,msm8974 msm8960 msm8660 msm8226 msm8610 msm8916 msm8916_32 msm8916_32_k64 msm8916_64 apq8084 msm8939 msm8994 msm8909 msm8996 msm8992 msm8952 msm8937 msm8953 msm8998 sdm660),true)

LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)
LOCAL_C_INCLUDES := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include \
                    $(LOCAL_PATH)/../QSEEComAPI \
                    $(LOCAL_PATH)/../sampleclient \
                    $(TARGET_OUT_HEADERS)/common/inc \
                    $(LOCAL_PATH)/../sse/SecureUILib \

LOCAL_ADDITIONAL_DEPENDENCIES := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr

LOCAL_SHARED_LIBRARIES := \
        libc \
        libcutils \
        libutils \
        libQSEEComAPI \
        libdl \
        libSecureUILib

LOCAL_MODULE := qseecom_sample_client
LOCAL_SRC_FILES := qseecom_sample_client.c
LOCAL_MODULE_TAGS := optional
LOCAL_CFLAGS := $(QSEECOM_CFLAGS)
LOCAL_PRELINK_MODULE := false
LOCAL_MODULE_OWNER := qcom
include $(BUILD_EXECUTABLE)

endif # end filter
