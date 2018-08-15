ifeq ($(call is-vendor-board-platform,QCOM),true)
OLD_LOCAL_PATH := $(LOCAL_PATH)
LOCAL_PATH:=$(call my-dir)

# Build lib for interface with mm-camera/stack (libanki-camera)
include $(CLEAR_VARS)

LOCAL_CFLAGS:= \
        -DAMSS_VERSION=$(AMSS_VERSION) \
        $(mmcamera_debug_defines) \
        $(mmcamera_debug_cflags) \
        $(USE_SERVER_TREE)

ifeq ($(strip $(TARGET_USES_ION)),true)
LOCAL_CFLAGS += -DUSE_ION
endif

LOCAL_CFLAGS += -D_ANDROID_

LOCAL_SRC_FILES:= \
        src/mm_camera_stream_rdi.c \
        src/mm_camera_stream_preview.c \
        src/mm_camera_stream_snapshot.c \
        src/mm_camera_anki.c

LOCAL_C_INCLUDES:=$(LOCAL_PATH)/src
LOCAL_C_INCLUDES+=$(LOCAL_PATH)/inc
LOCAL_C_INCLUDES+=$(LOCAL_PATH)/../mm-camera-test/inc
LOCAL_C_INCLUDES+=$(LOCAL_PATH)/../common
LOCAL_C_INCLUDES+= \
        frameworks/native/include/media/openmax \
        $(LOCAL_PATH)/../../../mm-image-codec/qexif \
        $(LOCAL_PATH)/../../../mm-image-codec/qomx_core

LOCAL_C_INCLUDES+= $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include/media
LOCAL_C_INCLUDES+= $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include
LOCAL_ADDITIONAL_DEPENDENCIES := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr

LOCAL_CFLAGS += -DCAMERA_ION_HEAP_ID=ION_IOMMU_HEAP_ID

LOCAL_CFLAGS += -DCAMERA_GRALLOC_HEAP_ID=GRALLOC_USAGE_PRIVATE_MM_HEAP
LOCAL_CFLAGS += -DCAMERA_GRALLOC_FALLBACK_HEAP_ID=GRALLOC_USAGE_PRIVATE_IOMMU_HEAP
LOCAL_CFLAGS += -DCAMERA_ION_FALLBACK_HEAP_ID=ION_IOMMU_HEAP_ID
LOCAL_CFLAGS += -DCAMERA_GRALLOC_CACHING_ID=0
LOCAL_CFLAGS += -DNUM_RECORDING_BUFFERS=9

LOCAL_CFLAGS += -Wall -Wextra -Werror

LOCAL_SHARED_LIBRARIES:= \
         libmm-qcamera libcutils libdl libmmcamera_interface liblog

LOCAL_MODULE_TAGS := optional

LOCAL_MODULE:= libmmcamera-anki
LOCAL_CLANG := false
LOCAL_32_BIT_ONLY := true
include $(BUILD_STATIC_LIBRARY)


# Build anki camera daemon (mm-anki-camera)
include $(CLEAR_VARS)

LOCAL_CFLAGS:= \
        -DAMSS_VERSION=$(AMSS_VERSION) \
        $(mmcamera_debug_defines) \
        $(mmcamera_debug_cflags) \
        $(USE_SERVER_TREE)

ifeq ($(strip $(TARGET_USES_ION)),true)
LOCAL_CFLAGS += -DUSE_ION
endif

LOCAL_CFLAGS += -D_ANDROID_
LOCAL_CFLAGS += -DUSE_ANDROID_LOGGING=1
LOCAL_CFLAGS += -std=c11

LOCAL_SRC_FILES:= \
        src/log.c \
        src/debayer.c \
        src/camera_server.c \
        src/camera_process.c \
        src/camera_memory.c \
	src/main_anki_camera.c

LOCAL_C_INCLUDES:=$(LOCAL_PATH)/src
LOCAL_C_INCLUDES+=$(LOCAL_PATH)/inc

LOCAL_C_INCLUDES+= $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include/media
LOCAL_C_INCLUDES+= $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include
LOCAL_ADDITIONAL_DEPENDENCIES := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr

LOCAL_CFLAGS += -Wall -Wextra -Werror

LOCAL_SHARED_LIBRARIES:= \
         libmm-qcamera libcutils libdl libmmcamera_interface liblog
LOCAL_STATIC_LIBRARIES:= \
         libmmcamera-anki
LOCAL_MODULE_TAGS := optional

LOCAL_MODULE:= mm-anki-camera
LOCAL_CLANG := false
LOCAL_32_BIT_ONLY := true
include $(BUILD_EXECUTABLE)

LOCAL_PATH := $(OLD_LOCAL_PATH)

endif
