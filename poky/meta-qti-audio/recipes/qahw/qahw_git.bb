inherit autotools pkgconfig

DESCRIPTION = "qahw"
SECTION = "multimedia"
LICENSE = "BSD"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta/files/common-licenses/\
${LICENSE};md5=3775480a712fc46a69647678acb234cb"

FILESPATH =+ "${WORKSPACE}/:"
SRC_URI  = "file://hardware/qcom/audio/qahw/"

S = "${WORKDIR}/hardware/qcom/audio/qahw/"
PR = "r0"

DEPENDS = "glib-2.0 libhardware liblog libcutils system-media"
EXTRA_OECONF = "--with-glib"

SOLIBS = ".so"
FILES_SOLIBSDEV = ""
