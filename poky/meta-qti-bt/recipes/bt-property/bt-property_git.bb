inherit autotools-brokensep pkgconfig

DESCRIPTION = "Bluetooth Property Daemon"
HOMEPAGE = "http://codeaurora.org/"
LICENSE = "Apache-2.0"

LIC_FILES_CHKSUM = "file://${COREBASE}/meta/files/common-licenses/\
${LICENSE};md5=89aea4e17d99a7cacdbeed46a0096b10"

DEPENDS = "common glib-2.0 liblog"
FILESPATH =+ "${WORKSPACE}:"
SRC_URI = "file://qcom-opensource/bt/property-ops/"

EXTRA_OECONF = " \
                --with-glib \
               "
S = "${WORKDIR}/qcom-opensource/bt/property-ops/"

CFLAGS_append = " -DUSE_ANDROID_LOGGING "
LDFLAGS_append = " -llog "
