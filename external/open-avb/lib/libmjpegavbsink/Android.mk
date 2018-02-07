LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)

LOCAL_SRC_FILES:= \
    AvbMjpegSink.cpp \
    GlSink.cpp \
    main.cpp \
    MjpegFrame.cpp

LOCAL_CFLAGS += -DGL_GLEXT_PROTOTYPES -DEGL_EGLEXT_PROTOTYPES

ifeq ($(call is-platform-sdk-version-at-least,25),true)
LOCAL_CFLAGS += -DUSE_SK_CODEC
endif

LOCAL_CFLAGS += -Wall -Werror -Wunused -Wunreachable-code

LOCAL_SHARED_LIBRARIES := \
    libcutils \
    libutils \
    libskia \
    libEGL \
    libGLESv1_CM \
    libgui \
    liblog

LOCAL_MODULE:= libmjpegavbsink

include $(BUILD_SHARED_LIBRARY)
