ifeq ($(call is-board-platform-in-list, msm8994 msm8996 msm8992 msm8952 msm8937 msm8953 msm8998 sdm660),true)

LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)
LOCAL_C_INCLUDES := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include \
                    $(LOCAL_PATH)/../soterclient \
                    $(TARGET_OUT_HEADERS)/common/inc \

LOCAL_ADDITIONAL_DEPENDENCIES := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr

LOCAL_SHARED_LIBRARIES := \
        libc \
        libcutils \
        libutils \
        libdl \
        libhardware

LOCAL_MODULE := soter_client
LOCAL_SRC_FILES := soter_client.c
LOCAL_MODULE_TAGS := optional
LOCAL_PRELINK_MODULE := false
LOCAL_MODULE_OWNER := qti
include $(BUILD_EXECUTABLE)

endif # end filter
