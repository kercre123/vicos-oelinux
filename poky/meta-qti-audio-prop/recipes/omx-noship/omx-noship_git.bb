inherit qcommon qlicense qprebuilt

DESCRIPTION = "omx-noship Library"
PR = "r0"

SRC_DIR = "${WORKSPACE}/audio/mm-audio-noship/omx/"
S = "${WORKDIR}/audio/mm-audio-noship/omx/"

DEPENDS = "media libcutils glib-2.0"

EXTRA_OECONF += "--with-sanitized-headers=${STAGING_KERNEL_BUILDDIR}/usr/include"
EXTRA_OECONF += "--with-glib"
EXTRA_OEMAKE += "DEFAULT_INCLUDES=-I${WORKSPACE}/hardware/qcom/media/mm-core/inc"

SOLIBS = ".so"
FILES_SOLIBSDEV = ""
