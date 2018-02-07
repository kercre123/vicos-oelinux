LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)

LOCAL_SRC_FILES:=         \
	AvbMpeg2tsStream.cpp \

LOCAL_SHARED_LIBRARIES := \
        libstagefright liblog libutils libbinder libgui \
        libstagefright_foundation libmedia libcutils

LOCAL_C_INCLUDES:= \
        frameworks/av/media/libstagefright \

LOCAL_CFLAGS += -Wall -Werror -Wunused -Wunreachable-code
#LOCAL_CLANG := true

#LOCAL_MODULE_TAGS := optional

LOCAL_MODULE:= libmpeg2ts

include $(BUILD_SHARED_LIBRARY)


