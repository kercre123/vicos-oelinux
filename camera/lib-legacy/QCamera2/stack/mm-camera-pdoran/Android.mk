# This is an experiment to understand how the applications get built, what flags do what, how to access different parts
# of the camera process, the ISP, etc. This is not intended as product code, just code to poke at the qualcomm camera.

# COPIED FROM mm-camera-test
# TODO: Why do we need this?
ifeq ($(call is-vendor-board-platform,QCOM),true)
OLD_LOCAL_PATH := $(LOCAL_PATH)
LOCAL_PATH:=$(call my-dir)
# END COPY

# Build mm-pdoran-camera application
include $(CLEAR_VARS)

LOCAL_CPP_EXTENSION := .cpp

# COPIED FROM mm-camera-test
# TODO: Do we need this?
# LOCAL_CFLAGS:= \
#         -DAMSS_VERSION=$(AMSS_VERSION) \
#         $(mmcamera_debug_defines) \
#         $(mmcamera_debug_cflags) \
#         $(USE_SERVER_TREE)

# ifeq ($(strip $(TARGET_USES_ION)),true)
# LOCAL_CFLAGS += -DUSE_ION
# endif

# LOCAL_CFLAGS += -D_ANDROID_
# END COPY 

LOCAL_CPPFLAGS += -std=c++11
LOCAL_CPPFLAGS += -Wall
LOCAL_CPPFLAGS += -Wextra
LOCAL_CPPFLAGS += -Werror

LOCAL_SRC_FILES:= \
        src/main_pdoran.cpp \
        src/application.cpp \
        src/camera.cpp

LOCAL_C_INCLUDES:=$(LOCAL_PATH)/src
LOCAL_C_INCLUDES+=$(LOCAL_PATH)/inc

LOCAL_C_INCLUDES+= $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include
# TODO: Do we need this?
# LOCAL_ADDITIONAL_DEPENDENCIES := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr

# LOCAL_SHARED_LIBRARIES += libmm-qcamera
# LOCAL_SHARED_LIBRARIES += libmmcamera_interface
# LOCAL_SHARED_LIBRARIES += libcutils
# LOCAL_SHARED_LIBRARIES += libdl
# LOCAL_SHARED_LIBRARIES += liblog

LOCAL_STATIC_LIBRARIES := 

LOCAL_MODULE:= mm-pdoran-camera

# COPIED FROM mm-camera-test
LOCAL_CLANG := false
LOCAL_32_BIT_ONLY := true
# END COPY
include $(BUILD_EXECUTABLE)

LOCAL_PATH := $(OLD_LOCAL_PATH)

endif
