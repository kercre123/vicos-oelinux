LOCAL_PATH := $(call my-dir)
QMMF_SDK_TOP_SRCDIR := $(LOCAL_PATH)/..

QMMF_TOPO_MAN_INCLUDE_PATH := $(QMMF_SDK_TOP_SRCDIR)/include/qmmf-plugin

include $(QMMF_SDK_TOP_SRCDIR)/build.mk

ifneq (,$(BUILD_QMMMF))

include $(CLEAR_VARS)

LOCAL_COPY_HEADERS_TO := qmmf-sdk
LOCAL_COPY_HEADERS := ../include/qmmf-plugin/qmmf_alg_plugin.h
LOCAL_COPY_HEADERS += ../include/qmmf-plugin/qmmf_alg_types.h
LOCAL_COPY_HEADERS += ../include/qmmf-plugin/qmmf_alg_utils.h
LOCAL_COPY_HEADERS += ../include/qmmf-plugin/qmmf_alg_intf.h

include $(BUILD_COPY_HEADERS)

# Build qmmf test outplace algorithm library

include $(CLEAR_VARS)

include $(QMMF_SDK_TOP_SRCDIR)/common.mk

LOCAL_C_INCLUDES += $(QMMF_TOPO_MAN_INCLUDE_PATH)

LOCAL_SRC_FILES := sample-plugins/qmmf_test_algo_outplace.cc

LOCAL_MODULE = libqmmf_test_algo_outplace

include $(BUILD_SHARED_LIBRARY)

# Build qmmf test inplace algorithm library

include $(CLEAR_VARS)

include $(QMMF_SDK_TOP_SRCDIR)/common.mk

LOCAL_C_INCLUDES += $(QMMF_TOPO_MAN_INCLUDE_PATH)

LOCAL_SRC_FILES := sample-plugins/qmmf_test_algo_inplace.cc

LOCAL_MODULE = libqmmf_test_algo_inplace

include $(BUILD_SHARED_LIBRARY)

# Build qmmf test outplace algorithm with history library

include $(CLEAR_VARS)

include $(QMMF_SDK_TOP_SRCDIR)/common.mk

LOCAL_C_INCLUDES += $(QMMF_TOPO_MAN_INCLUDE_PATH)

LOCAL_SRC_FILES := sample-plugins/qmmf_test_algo_outplace_history.cc

LOCAL_MODULE = libqmmf_test_algo_outplace_history

include $(BUILD_SHARED_LIBRARY)

# Build qmmf test inplace algorithm with history library

include $(CLEAR_VARS)

include $(QMMF_SDK_TOP_SRCDIR)/common.mk

LOCAL_C_INCLUDES += $(QMMF_TOPO_MAN_INCLUDE_PATH)

LOCAL_SRC_FILES := sample-plugins/qmmf_test_algo_inplace_history.cc

LOCAL_MODULE = libqmmf_test_algo_inplace_history

include $(BUILD_SHARED_LIBRARY)

# Build qmmf test resizer library

include $(CLEAR_VARS)

include $(QMMF_SDK_TOP_SRCDIR)/common.mk

LOCAL_C_INCLUDES += $(QMMF_TOPO_MAN_INCLUDE_PATH)

LOCAL_SRC_FILES := sample-plugins/qmmf_test_algo_resizer.cc

LOCAL_MODULE = qmmf_test_algo_resizer

include $(BUILD_SHARED_LIBRARY)

# Build qmmf utils gtest application binary

include $(CLEAR_VARS)

include $(QMMF_SDK_TOP_SRCDIR)/common.mk

LOCAL_C_INCLUDES += $(QMMF_TOPO_MAN_INCLUDE_PATH)

LOCAL_SRC_FILES := gtest/qmmf_utils_gtest.cc

LOCAL_MODULE = qmmf_utils_gtest

include $(BUILD_NATIVE_TEST)

# Build qmmf algorithm interface gtest application binary

include $(CLEAR_VARS)

include $(QMMF_SDK_TOP_SRCDIR)/common.mk

LOCAL_C_INCLUDES += $(QMMF_TOPO_MAN_INCLUDE_PATH)

LOCAL_SRC_FILES := gtest/qmmf_algo_interface_gtest.cc

LOCAL_MODULE = qmmf_algo_interface_gtest

include $(BUILD_NATIVE_TEST)

endif # BUILD_QMMMF
