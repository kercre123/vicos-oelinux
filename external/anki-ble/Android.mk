LOCAL_PATH:= $(call my-dir)

#
# Workaround for libchrome and -DNDEBUG usage.
#
# Test whether the original HOST_GLOBAL_CFLAGS and
# TARGET_GLOBAL_CFLAGS contain -DNDEBUG .
# This is needed as a workaround to make sure that
# libchrome and local files calling logging::InitLogging()
# are consistent with the usage of -DNDEBUG .
# ========================================================
ifneq (,$(findstring NDEBUG,$(HOST_GLOBAL_CFLAGS)))
  btservice_orig_HOST_NDEBUG := -DBT_LIBCHROME_NDEBUG
else
  btservice_orig_HOST_NDEBUG :=
endif
ifneq (,$(findstring NDEBUG,$(TARGET_GLOBAL_CFLAGS)))
  btservice_orig_TARGET_NDEBUG := -DBT_LIBCHROME_NDEBUG
else
  btservice_orig_TARGET_NDEBUG :=
endif


# Anki BLE Server
# ========================================================
include $(CLEAR_VARS)
LOCAL_SRC_FILES := \
	anki_ble_server.cpp \
	server_main.cpp
LOCAL_C_INCLUDES += $(LOCAL_PATH)
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE := anki-ble-server
LOCAL_STATIC_LIBRARIES += libbluetooth-client
LOCAL_SHARED_LIBRARIES += \
	libbinder \
	libchrome \
	libutils \
	libcutils

LOCAL_INIT_RC := anki-ble-server.rc

LOCAL_CFLAGS += $(bluetooth_CFLAGS) $(btservice_orig_TARGET_NDEBUG)
LOCAL_CONLYFLAGS += $(bluetooth_CONLYFLAGS)
LOCAL_CPPFLAGS += $(bluetooth_CPPFLAGS)

include $(BUILD_EXECUTABLE)
