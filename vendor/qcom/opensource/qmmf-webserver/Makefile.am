BUILD_COMPONENTS =
BUILD_COMPONENTS += rtspinterface
BUILD_COMPONENTS += muxinterface
BUILD_COMPONENTS += jpegencoder
BUILD_COMPONENTS += database
BUILD_COMPONENTS += vaminterface
BUILD_COMPONENTS += httpinterface

SUBDIRS = $(BUILD_COMPONENTS)

pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = qmmf-support.pc.in
EXTRA_DIST = $(pkgconfig_DATA)
