inherit native autotools pkgconfig

DESCRIPTION = "EXT4 UTILS"
HOMEPAGE = "http://developer.android.com/"
LICENSE = "Apache-2.0"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta/files/common-licenses/\
${LICENSE};md5=89aea4e17d99a7cacdbeed46a0096b10"

PR = "r1"

DEPENDS = "libselinux-native libsparse-native libcutils-native libpcre-native"

FILESPATH =+ "${WORKSPACE}/system/extras/:"
SRC_URI = "file://ext4_utils"

S = "${WORKDIR}/ext4_utils"

EXTRA_OECONF = "--with-core-includes=${WORKSPACE}/system/core/include"

CPPFLAGS += "-I${STAGING_INCDIR}/libselinux"
