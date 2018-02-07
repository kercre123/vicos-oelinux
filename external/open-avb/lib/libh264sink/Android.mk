LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)

LOCAL_SRC_FILES:= \
        Avbh264Stream.cpp\

LOCAL_SHARED_LIBRARIES := \
        libstagefright liblog libutils libbinder libstagefright_foundation \
        libmedia libgui libcutils libui

LOCAL_C_INCLUDES:= \
        frameworks/av/media/libstagefright \
        $(TOP)/frameworks/native/include/media/openmax

LOCAL_CFLAGS += -Wno-multichar -Werror -Wall
LOCAL_CLANG := true

ifeq ($(call is-platform-sdk-version-at-least,25),true)
LOCAL_CFLAGS += -DUSE_MEDIA_CODEC_BUFFER
endif

LOCAL_MODULE_TAGS := optional

LOCAL_MODULE:= libh264sink

include $(BUILD_SHARED_LIBRARY)

