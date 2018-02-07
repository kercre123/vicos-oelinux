LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_CFLAGS := -DWITHOUT_IFADDRS -Wno-unused-parameter -frtti

LOCAL_C_INCLUDES := $(LOCAL_PATH)/common \
                    $(LOCAL_PATH)/linux/src


LOCAL_SRC_FILES := linux/src/daemon_cl.cpp \
                   common/ptp_message.cpp \
                   common/avbts_osnet.cpp \
                   common/ieee1588port.cpp \
                   common/ieee1588clock.cpp \
                   linux/src/linux_hal_common.cpp \
                   linux/src/platform.cpp \
                   linux/src/linux_hal_generic.cpp \
                   linux/src/linux_hal_generic_adj.cpp

LOCAL_MODULE := gptp

include $(BUILD_EXECUTABLE)
