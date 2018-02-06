# verify it isn't a simulator build
ifeq ($(call is-vendor-board-platform,QCOM),true)
ifeq ($(call is-board-platform-in-list,msm8998),true)

LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_SRC_FILES := spdaemon.c

LOCAL_C_INCLUDES += $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include
LOCAL_C_INCLUDES += $(TARGET_OUT_HEADERS)/common/inc
LOCAL_C_INCLUDES += $(LOCAL_PATH)/../../../securemsm/spcomlib/inc
LOCAL_C_INCLUDES += $(TARGET_OUT_HEADERS)/libmdmdetect/inc/
LOCAL_C_INCLUDES += $(TARGET_OUT_HEADERS)/libperipheralclient/inc/

LOCAL_ADDITIONAL_DEPENDENCIES += $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr

LOCAL_SHARED_LIBRARIES := libspcom libutils libcutils liblog libperipheral_client libmdmdetect

LOCAL_CFLAGS += -fno-exceptions -fno-short-enums -DANDROID

ifeq ($(TARGET_BUILD_VARIANT),user)
LOCAL_CFLAGS+= -DUSER_BUILD
endif

LOCAL_MODULE := spdaemon

LOCAL_MODULE_TAGS := optional

LOCAL_PRELINK_MODULE := false

LOCAL_MODULE_OWNER := qti

include $(BUILD_EXECUTABLE)

endif
endif

