#/******************************************************************************
#*@file Android.mk
#* brief Rules for compiling native code associated with libulp library
#*  ******************************************************************************/
ifneq ($(BUILD_TINY_ANDROID),true)

LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)
ULP2_ROOT_DIR = ../
LOC_API_DIR = $(TOP)/hardware/qcom/gps/loc_api
INTERNAL_ROOT_DIR = $(TOP)/vendor/qcom/proprietary/gps-noship/internal
OSAL_ROOT_DIR = $(INTERNAL_ROOT_DIR)/osal

# This is the target being built.
LOCAL_MODULE:= libulp2

# All of the source files that we will compile.
LOCAL_SRC_FILES:= \
    ulp_main.cpp \
    ulp_data.cpp \
    ulp_msg.cpp \
    ulp_gnss.cpp \
    ulp_amt.cpp \
    ulp_gnp.cpp \
    ulp_quipc.cpp \
    ulp_xtwifi.cpp \
    ulp_brain.cpp \
    ulp_monitor.cpp \
    ulp_debug.cpp \
    ulp_log.cpp \
    LocUlpProxy.cpp \
    LocAmtProxy.cpp \
    ulp_zpp.cpp

# All of the shared libraries we link against.
LOCAL_SHARED_LIBRARIES := \
    libcutils \
    libutils \
    libgps.utils \
    libdl \
    libloc_core \
    libizat_core \
    liblog \
    liblbs_core \
    libloc_pla \
    libDRPlugin \
    libevent_observer

LOCAL_C_INCLUDES += \
  $(TARGET_OUT_HEADERS)/gps.utils \
  $(TARGET_OUT_HEADERS)/libloc_core \
  $(TARGET_OUT_HEADERS)/libizat_core \
  $(TARGET_OUT_HEADERS)/liblocationservice \
  $(TARGET_OUT_HEADERS)/liblbs_core \
  $(LOCAL_PATH)/$(ULP2_ROOT_DIR)/inc \
  $(TARGET_OUT_HEADERS)/libflp \
  $(TARGET_OUT_HEADERS)/common/inc \
  $(TARGET_OUT_HEADERS)/libloc_pla \
  $(TARGET_OUT_HEADERS)/libDRPlugin \
  $(TARGET_OUT_HEADERS)/libevent_observer \
  $(LOCAL_PATH)/$(ULP2_ROOT_DIR)/inc \
  $(INTERNAL_ROOT_DIR)/drplugin  \
  $(OSAL_ROOT_DIR) \
  $(TOP)/vendor/qcom/proprietary/gps/framework/native/service \
  $(LOCAL_PATH)/$(ULP2_ROOT_DIR)/inc

LOCAL_CFLAGS+=$(GPS_FEATURES) \
    -D_ANDROID_ \

LOCAL_LDFLAGS += -Wl,--export-dynamic
LOCAL_MODULE_TAGS := optional

LOCAL_PRELINK_MODULE := false
LOCAL_MODULE_OWNER := qcom
LOCAL_PROPRIETARY_MODULE := true

include $(BUILD_SHARED_LIBRARY)

endif # not BUILD_TINY_ANDROID



