BUILD_COMPONENTS =
BUILD_COMPONENTS += common/cameraadaptor
BUILD_COMPONENTS += common/overlay
BUILD_COMPONENTS += common/audio/src/client
BUILD_COMPONENTS += common/audio/src/service
BUILD_COMPONENTS += common/audio/test/samples
BUILD_COMPONENTS += common/codecadaptor
BUILD_COMPONENTS += common/avqueue
BUILD_COMPONENTS += plugins
BUILD_COMPONENTS += avcodec
BUILD_COMPONENTS += display/src/client
BUILD_COMPONENTS += recorder/src/service/post-process/modules/algo
BUILD_COMPONENTS += recorder/src/service/post-process/modules/camera-hal-jpeg
BUILD_COMPONENTS += recorder/src/service/post-process/modules/camera-hal-reproc
BUILD_COMPONENTS += recorder/src/service/post-process/modules/jpeg-encoder
BUILD_COMPONENTS += recorder/src/service/post-process/modules/test
BUILD_COMPONENTS += display/src/service
BUILD_COMPONENTS += display/test/samples
BUILD_COMPONENTS += display/test/gtest
BUILD_COMPONENTS += recorder/src/client
BUILD_COMPONENTS += recorder/src/service
BUILD_COMPONENTS += recorder/test/samples
BUILD_COMPONENTS += recorder/test/gtest
BUILD_COMPONENTS += recorder/test/timelapse
BUILD_COMPONENTS += recorder/test/videotimelapse
BUILD_COMPONENTS += player/src/client
BUILD_COMPONENTS += player/src/service
BUILD_COMPONENTS += player/test/demuxer
BUILD_COMPONENTS += player/test/samples
BUILD_COMPONENTS += player/test/gtest
BUILD_COMPONENTS += system/src/client
BUILD_COMPONENTS += system/src/service
BUILD_COMPONENTS += system/test/samples
BUILD_COMPONENTS += player/test/transcode_app
BUILD_COMPONENTS += qmmf-server


SUBDIRS = $(BUILD_COMPONENTS)

pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = qmmf-sdk.pc
EXTRA_DIST = $(pkgconfig_DATA)

Export qmmf-sdk headers:
sdkincdir=$(includedir)/qmmf-sdk
sdkinc_HEADERS =$(top_srcdir)/include/qmmf-plugin/qmmf_alg_plugin.h
sdkinc_HEADERS +=$(top_srcdir)/include/qmmf-plugin/qmmf_alg_types.h
sdkinc_HEADERS +=$(top_srcdir)/include/qmmf-plugin/qmmf_alg_utils.h
sdkinc_HEADERS +=$(top_srcdir)/include/qmmf-plugin/qmmf_alg_intf.h
