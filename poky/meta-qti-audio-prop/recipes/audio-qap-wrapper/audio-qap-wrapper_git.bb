inherit qcommon qlicense qprebuilt

DESCRIPTION = "qap"
SECTION = "multimedia"
PR = "r0"

DEPENDS = "glib-2.0 libcutils system-media audio-qaf audio-ip-handler"

EXTRA_OEMAKE = "DEFAULT_INCLUDES= CPPFLAGS="-I. -I${STAGING_KERNEL_BUILDDIR}/usr/include""

SRC_DIR = "${WORKSPACE}/audio/mm-audio/qap_wrapper/"
S = "${WORKDIR}/audio/mm-audio/qap_wrapper/"

EXTRA_OECONF = "--with-glib"
EXTRA_OECONF += "--with-sanitized-headers=${STAGING_KERNEL_BUILDDIR}/usr/include"

FILES_SOLIBSDEV = ""
FILES_${PN} += "${libdir}/*.so"
