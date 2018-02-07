LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_CFLAGS := -Wall -Wextra -Wno-parentheses -ggdb -D_GNU_SOURCE

LOCAL_C_INCLUDES := $(LOCAL_PATH) \
                    $(LOCAL_PATH)/../common

LOCAL_SRC_FILES := mrpd.c \
                   mvrp.c \
                   msrp.c \
                   mmrp.c \
                   mrp.c \
                   ../common/parse.c \
                   ../common/eui64set.c \

LOCAL_MODULE := mrpd


include $(BUILD_EXECUTABLE)


