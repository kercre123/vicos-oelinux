LOCAL_PATH := $(call my-dir)

QMMF_SDK_TOP_SRCDIR := $(LOCAL_PATH)/../../..

include $(QMMF_SDK_TOP_SRCDIR)/build.mk

ifneq (,$(BUILD_QMMMF))

# Build player test application binary

include $(CLEAR_VARS)

include $(QMMF_SDK_TOP_SRCDIR)/common.mk

LOCAL_C_INCLUDES += $(TARGET_OUT_HEADERS)/mm-parser/include
LOCAL_C_INCLUDES += $(TARGET_OUT_HEADERS)/mm-osal/include

LOCAL_SRC_FILES := qmmf_player_test.cc

LOCAL_SHARED_LIBRARIES += libqmmf_demuxer libmmosal libmmparser_lite
LOCAL_SHARED_LIBRARIES += libqmmf_player_client

LOCAL_MODULE = qmmf_player_test

include $(BUILD_EXECUTABLE)

# Build player test application binary

include $(CLEAR_VARS)

include $(QMMF_SDK_TOP_SRCDIR)/common.mk

LOCAL_SRC_FILES := qmmf_player_parser_test.cc
LOCAL_SRC_FILES += qmmf_player_parser.cc

LOCAL_SHARED_LIBRARIES += libqmmf_player_client

LOCAL_MODULE = qmmf_player_parser_test

include $(BUILD_EXECUTABLE)

endif # BUILD_QMMMF
