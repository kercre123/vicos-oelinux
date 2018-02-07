ROOT_DIR := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_EAVB_PATH := $(ROOT_DIR)

include $(call first-makefiles-under,$(LOCAL_EAVB_PATH))
