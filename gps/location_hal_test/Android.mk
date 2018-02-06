
LOCAL_PATH:=$(call my-dir)
include $(CLEAR_VARS)

LOCAL_SRC_FILES:= \
    location_hal_test.cpp

LOCAL_CFLAGS:= \
    -DDEBUG \
    -D_ANDROID_ \

LOCAL_C_INCLUDES += \
        $(TARGET_OUT_HEADERS)/gps.utils \
        $(TARGET_OUT_HEADERS)/libloc_core \
        $(TARGET_OUT_HEADERS)/libgeofence \
        $(TARGET_OUT_HEADERS)/libizat_core \
        $(TARGET_OUT_HEADERS)/libflp \
        $(TARGET_OUT_HEADERS)/libloc_pla \
        $(TARGET_OUT_HEADERS)/liblbs_core \
        $(TARGET_OUT_HEADERS)/common/inc

LOCAL_SHARED_LIBRARIES := \
    libutils \
    libcutils \
    libgps.utils \
    libhardware \
    libgeofence \
    libandroid_runtime \
    libflp \
    libloc_eng

LOCAL_PRELINK_MODULE:=false

LOCAL_MODULE:=location_hal_test
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_OWNER := qti
LOCAL_PROPRIETARY_MODULE := true
include $(BUILD_EXECUTABLE)

