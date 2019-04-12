inherit qcommon qlicense qprebuilt

DESCRIPTION = "acdb-mcs Library"
PR = "r0"

SRC_DIR = "${WORKSPACE}/audio/mm-audio/audio-acdb-util/acdb-mcs/"
S = "${WORKDIR}/audio/mm-audio/audio-acdb-util/acdb-mcs"

EXTRA_OECONF_append = " --with-sanitized-headers=${STAGING_KERNEL_BUILDDIR}/usr/include"

DEPENDS = "acdbloader tinyalsa"
