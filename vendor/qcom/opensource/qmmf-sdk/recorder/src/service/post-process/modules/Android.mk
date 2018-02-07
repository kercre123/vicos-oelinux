LOCAL_PATH := $(call my-dir)

MY_PATH := $(LOCAL_PATH)

include $(MY_PATH)/camera-hal-jpeg/Android.mk

include $(MY_PATH)/camera-hal-reproc/Android.mk

include $(MY_PATH)/algo/Android.mk

include $(MY_PATH)/jpeg-encoder/Android.mk

include $(MY_PATH)/test/Android.mk
